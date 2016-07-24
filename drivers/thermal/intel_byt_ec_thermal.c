/*
 * intel_byt_ec_thermal.c - Intel Baytrail(M) Platform Thermal Driver
 *
 * Copyright (C) 2013 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This driver talks to the Embedded Controller(EC) on the platform,
 * to retrieve temperature from the Thermal sensors (if available).
 * EC Firmware support is required for this driver to work.
 */

#define pr_fmt(fmt)  "intel_byt_ec_thermal: " fmt

#include <linux/pm.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/thermal.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>

#include <asm/intel_byt_ec.h>
#include <asm/intel_mid_thermal.h>

#define DEVICE_NAME "byt_ec_thermal"

/* Number of Thermal sensors on the platform */
#define NUM_THERMAL_SENSORS	7

/* Registers that govern Thermal Monitoring */
#define TEMP_SENSOR_SELECT	0x52
#define TEMP_THRESH_HIGH	0x53
#define TEMP_THRESH_LOW		0x54
#define TEMP_THRESH_STS		0x55

/* EC commands */
#define SET_TEMP_THRESHOLD	0x4A
#define SET_FAN_SPEED		0x1A

/* Fan control registers */
#define PWM_PORT		0x41
#define PWM_VALUE		0x44

/* Fan states are mapped from 0 to 3 */
#define MAX_FAN_STATES		4

/*
 * Hysteresis value is 4 bits wide. A value of 0x01
 * corresponds to 1C and 0x0F corresponds to 15C.
 * Can be changed at run-time through Sysfs.
 */
#define DEFAULT_HYST_mC		2000

#define NUM_ALERT_LEVELS	2
#define ALERT_RW_MASK		0x03

static DEFINE_MUTEX(thrm_update_lock);

/*
 * The EC can have many thermal sensors connected to it. For this
 * platform, this array holds the register address to read the
 * temperature reported by these sensors.
 */
static const int ec_sensors[NUM_THERMAL_SENSORS] = {
		0x01, 0x02, 0x50, 0xBA, 0xBB, 0x7E, 0xB9};

struct thermal_device_info {
	struct intel_mid_thermal_sensor *sensor;
	int sensor_index;
	u8 trips[NUM_ALERT_LEVELS];
};

struct thermal_data {
	struct platform_device		*pdev;
	struct notifier_block		nb;
	struct thermal_zone_device	**tzd;
	struct thermal_cooling_device	*fan_cdev;
	int cur_fan_state;

	/* Values obtained from platform data */
	int		num_sensors;
	struct		intel_mid_thermal_sensor *sensors;
};
static struct thermal_data *tdata;

static int set_fan_speed(unsigned long rpm)
{
	u8 val;
	int ret;

	/* Select the PWM port */
	ret = byt_ec_read_byte(PWM_PORT, &val);
	if (ret < 0)
		return ret;

	ret = byt_ec_write_byte(PWM_PORT, val | (1 << 0));
	if (ret < 0)
		return ret;

	/* Configure the Fan RPM registers with new value */
	ret = byt_ec_write_byte(PWM_VALUE, rpm);
	if (ret < 0)
		return ret;

	/* Send EC command to update Fan RPM */
	ret = byt_ec_send_cmd(0x1A);
	if (ret)
		pr_err("EC command to set Fan speed failed:%d\n", ret);

	return ret;
}

static int set_hyst_mC(long hyst_mC)
{
	int ret;
	u8 val, hystC;

	hystC = hyst_mC / 1000;

	/* Hysteresis is 4 bits wide */
	if (hystC > 15)
		return -EINVAL;

	mutex_lock(&thrm_update_lock);

	/* Bits[4:7] of TEMP_SENSOR_SELECT hold the hysteresis in C */
	ret = byt_ec_read_byte(TEMP_SENSOR_SELECT, &val);
	if (ret < 0)
		goto exit;

	/* Set Bits[4:7] of 'val' to Hysteresis value */
	val = (val & 0x0F) | (hystC << 4);

	ret = byt_ec_write_byte(TEMP_SENSOR_SELECT, val);

exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static int get_hyst_mC(long *hyst)
{
	int ret;
	u8 val;

	mutex_lock(&thrm_update_lock);

	/* Bits[4:7] of TEMP_SENSOR_SELECT hold the hysteresis in C */
	ret = byt_ec_read_byte(TEMP_SENSOR_SELECT, &val);
	if (ret < 0)
		goto exit;

	/* Return the hysteresis in mC */
	*hyst = (val >> 4) * 1000;
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static int set_trip_temp(struct thermal_device_info *td_info,
			int flag, int temp)
{
	u8 val;
	u8 new_val, old_val;
	u8 new_reg, old_reg;
	int ret;

	/*
	 * EC requires us to update both the thresholds simultaneously.
	 * 'new_val' is the threshold that needs to programmed 'now'.
	 * 'old_val' is what is stored previously. Same logic applies
	 * to the 'address' of these registers as well. Convert the
	 * values from mC to C before writing into the registers.
	 */
	new_val = temp;
	old_val = td_info->trips[flag % 1];

	if (flag) {
		new_reg = TEMP_THRESH_HIGH;
		old_reg = TEMP_THRESH_LOW;
	} else {
		new_reg = TEMP_THRESH_LOW;
		old_reg = TEMP_THRESH_HIGH;
	}

	ret = byt_ec_read_byte(TEMP_SENSOR_SELECT, &val);
	if (ret < 0)
		return ret;

	/* Bits[0:3] of TEMP_SENSOR_SELECT select the required sensor */
	val = (val & 0xF0) | td_info->sensor_index;

	ret = byt_ec_write_byte(TEMP_SENSOR_SELECT, val);
	if (ret < 0)
		return ret;

	ret = byt_ec_write_byte(new_reg, new_val);
	if (ret < 0)
		return ret;

	ret = byt_ec_write_byte(old_reg, old_val);
	if (ret < 0)
		return ret;

	/* Send SET_TEMP_THRESHOLD command */
	ret = byt_ec_send_cmd(SET_TEMP_THRESHOLD);
	if (ret < 0)
		return ret;

	/*
	 * There is no way to read the trip points back from
	 * the EC. So, store the value in our local structure.
	 */
	td_info->trips[flag] = new_val;
	return ret;
}

static int update_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret;
	u8 val;
	struct thermal_device_info *td_info = tzd->devdata;
	int ec_reg = ec_sensors[td_info->sensor_index];

	ret = byt_ec_read_byte(ec_reg, &val);
	if (ret < 0)
		return ret;

	/* Value read from EC is in C; convert to mC */
	*temp = val * 1000;
	return 0;
}

static ssize_t show_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret;

	mutex_lock(&thrm_update_lock);

	ret = update_temp(tzd, temp);

	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t store_trip_temp(struct thermal_zone_device *tzd,
				int trip, long trip_temp)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;

	if (trip_temp != 0 && trip_temp < 1000) {
		dev_err(&tzd->device, "Temperature should be in mC\n");
		return -EINVAL;
	}

	mutex_lock(&thrm_update_lock);

	ret = set_trip_temp(td_info, trip == 1, trip_temp / 1000);
	if (ret)
		dev_err(&tzd->device, "Setting trip point failed:%d\n", ret);

	mutex_unlock(&thrm_update_lock);

	return ret;
}

static ssize_t show_trip_temp(struct thermal_zone_device *tzd,
				int trip, long *trip_temp)
{
	struct thermal_device_info *td_info = tzd->devdata;

	mutex_lock(&thrm_update_lock);

	/* Convert to mC */
	*trip_temp = td_info->trips[trip] * 1000;

	mutex_unlock(&thrm_update_lock);

	return 0;
}

static ssize_t store_trip_hyst(struct thermal_zone_device *tzd,
				int trip, long hyst)
{
	/* We expect the Hysteresis in mC */
	if (hyst != 0 && hyst < 1000) {
		dev_err(&tzd->device, "Temperature should be in mC\n");
		return -EINVAL;
	}

	/*
	 * All thermal sensors share the same hysteresis for
	 * all trip points.
	 */
	return set_hyst_mC(hyst);
}

static ssize_t show_trip_hyst(struct thermal_zone_device *tzd,
				int trip, long *hyst)
{
	return get_hyst_mC(hyst);
}

static ssize_t show_trip_type(struct thermal_zone_device *tzd,
			int trip, enum thermal_trip_type *trip_type)
{
	/* All are passive trip points */
	*trip_type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static int fan_get_max_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	*state = MAX_FAN_STATES;
	return 0;
}

static int fan_get_cur_state(struct thermal_cooling_device *cdev,
				unsigned long *state)
{
	mutex_lock(&thrm_update_lock);

	*state = tdata->cur_fan_state;

	mutex_unlock(&thrm_update_lock);
	return 0;
}

static int fan_set_cur_state(struct thermal_cooling_device *cdev,
				unsigned long state)
{
	int ret;
	unsigned int rpm;

	if (state >= MAX_FAN_STATES || state < 0) {
		pr_err("Invalid Fan state:%ld\n", state);
		return -EINVAL;
	}

	mutex_lock(&thrm_update_lock);

	switch (state) {
	case 0:
		rpm = 0;
		break;
	case 1:
		rpm = 50;
		break;
	case 2:
		rpm = 100;
		break;
	case 3:
		rpm = 100;
		break;
	default:
		rpm = 0;
	}

	ret = set_fan_speed(rpm);
	if (ret < 0)
		goto exit;

	tdata->cur_fan_state = state;
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static struct thermal_device_info *initialize_sensor(int index,
				struct intel_mid_thermal_sensor *sensor)
{
	struct thermal_device_info *td_info =
		kzalloc(sizeof(struct thermal_device_info), GFP_KERNEL);

	if (!td_info)
		return NULL;

	td_info->sensor = sensor;
	td_info->sensor_index = index;

	return td_info;
}

static void notify_thermal_event(int indx)
{
	int ret;
	long cur_temp;
	char *thermal_event[3];

	struct thermal_zone_device *tzd = tdata->tzd[indx];

	mutex_lock(&thrm_update_lock);

	/*
	 * Read the current Temperature and send it to user land;
	 * so that the user space can avoid a sysfs read.
	 */
	ret = update_temp(tzd, &cur_temp);
	if (ret) {
		dev_err(&tzd->device, "Cannot update temperature\n");
		goto exit;
	}

	pr_info("Thermal Event: sensor: %s, cur_temp: %ld\n",
					tzd->type, cur_temp);
	thermal_event[0] = kasprintf(GFP_KERNEL, "NAME=%s", tzd->type);
	thermal_event[1] = kasprintf(GFP_KERNEL, "TEMP=%ld", cur_temp);
	thermal_event[2] = NULL;

	kobject_uevent_env(&tzd->device.kobj, KOBJ_CHANGE, thermal_event);

	kfree(thermal_event[1]);
	kfree(thermal_event[0]);

exit:
	mutex_unlock(&thrm_update_lock);
	return;
}

static void handle_therm_trip(void)
{
	u8 sts;
	int i, ret;

	ret = byt_ec_read_byte(TEMP_THRESH_STS, &sts);
	if (ret < 0) {
		pr_err("Failed to read status register:%d\n", ret);
		return;
	}

	for (i = 0; i < NUM_THERMAL_SENSORS; i++) {
		if (!(sts & (1 << i)))
			continue;

		notify_thermal_event(i);

		/* Clear the interrupt by writing 0 into it */
		sts = sts & ~(1 << i);
		ret = byt_ec_write_byte(TEMP_THRESH_STS, sts);
		if (ret < 0) {
			pr_err("Failed to clear status register:%d\n", ret);
			return;
		}
		break;
	}
}

static int ec_thermal_evt_callback(struct notifier_block *nb,
					unsigned long event, void *data)
{
	switch (event) {
	case BYT_EC_SCI_THERMAL:
		pr_info("SCI THERMAL EVENT\n");
		break;
	case BYT_EC_SCI_THERMTRIP:
		handle_therm_trip();
		break;
	}

	return 0;
}

static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = show_temp,
	.get_trip_type = show_trip_type,
	.get_trip_temp = show_trip_temp,
	.set_trip_temp = store_trip_temp,
	.get_trip_hyst = show_trip_hyst,
	.set_trip_hyst = store_trip_hyst,
};

static struct thermal_cooling_device_ops fan_cooling_ops = {
	.get_max_state = fan_get_max_state,
	.get_cur_state = fan_get_cur_state,
	.set_cur_state = fan_set_cur_state,
};

static int byt_ec_thermal_probe(struct platform_device *pdev)
{
	int i, size, ret;
	struct intel_mid_thermal_platform_data *pdata;

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Unable to fetch platform data\n");
		return -EINVAL;
	}

	tdata = devm_kzalloc(&pdev->dev,
				sizeof(struct thermal_data), GFP_KERNEL);
	if (!tdata) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	tdata->pdev = pdev;
	tdata->num_sensors = pdata->num_sensors;
	tdata->sensors = pdata->sensors;
	platform_set_drvdata(pdev, tdata);

	if (tdata->num_sensors != NUM_THERMAL_SENSORS)
		dev_warn(&pdev->dev, "Number of sensors do not match\n");

	size = sizeof(struct thermal_zone_device *) * tdata->num_sensors;
	tdata->tzd = kzalloc(size, GFP_KERNEL);
	if (!tdata->tzd) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	/* Register each sensor with the generic thermal framework */
	for (i = 0; i < tdata->num_sensors; i++) {
		tdata->tzd[i] = thermal_zone_device_register(
				tdata->sensors[i].name,
				NUM_ALERT_LEVELS, ALERT_RW_MASK,
				initialize_sensor(i, &tdata->sensors[i]),
				&tzd_ops,
				NULL, 0, 0);
		if (IS_ERR(tdata->tzd[i])) {
			ret = PTR_ERR(tdata->tzd[i]);
			dev_err(&pdev->dev,
				"registering thermal sensor %s failed: %d\n",
				tdata->sensors[i].name, ret);
			goto exit_reg;
		}
	}

	/* Register Fan as a cooling device */
	tdata->fan_cdev = thermal_cooling_device_register("Fan_EC", NULL,
						&fan_cooling_ops);
	if (IS_ERR(tdata->fan_cdev)) {
		ret = PTR_ERR(tdata->fan_cdev);
		tdata->fan_cdev = NULL;
		goto exit_reg;
	}

	/* Set default Hysteresis */
	ret = set_hyst_mC(DEFAULT_HYST_mC);
	if (ret) {
		dev_err(&pdev->dev,
			"Setting default hysteresis failed:%d\n", ret);
		goto exit_cdev;
	}

	/* Register for EC SCI events */
	tdata->nb.notifier_call = &ec_thermal_evt_callback;
	byt_ec_evt_register_notify(&tdata->nb);

	return 0;

exit_cdev:
	thermal_cooling_device_unregister(tdata->fan_cdev);
exit_reg:
	while (--i >= 0)
		thermal_zone_device_unregister(tdata->tzd[i]);
	kfree(tdata->tzd);
	return ret;
}

static int byt_ec_thermal_resume(struct device *dev)
{
	dev_info(dev, "resume called.\n");
	return 0;
}

static int byt_ec_thermal_suspend(struct device *dev)
{
	dev_info(dev, "suspend called.\n");
	return 0;
}

static int byt_ec_thermal_remove(struct platform_device *pdev)
{
	int i;
	struct thermal_data *tdata = platform_get_drvdata(pdev);

	if (!tdata)
		return 0;

	for (i = 0; i < tdata->num_sensors; i++)
		thermal_zone_device_unregister(tdata->tzd[i]);

	kfree(tdata->tzd);
	thermal_cooling_device_unregister(tdata->fan_cdev);

	return 0;
}

/*********************************************************************
 *		Driver initialization and finalization
 *********************************************************************/

static const struct dev_pm_ops thermal_pm_ops = {
	.suspend = byt_ec_thermal_suspend,
	.resume = byt_ec_thermal_resume,
};

static struct platform_driver byt_ec_thermal_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.pm = &thermal_pm_ops,
		},
	.probe = byt_ec_thermal_probe,
	.remove = byt_ec_thermal_remove,
};

static int byt_ec_thermal_module_init(void)
{
	return platform_driver_register(&byt_ec_thermal_driver);
}

static void byt_ec_thermal_module_exit(void)
{
	platform_driver_unregister(&byt_ec_thermal_driver);
}

late_initcall(byt_ec_thermal_module_init);
module_exit(byt_ec_thermal_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Baytrail-M Platform Thermal Driver");
MODULE_LICENSE("GPL");
