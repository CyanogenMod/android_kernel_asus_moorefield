/*
 * intel_mid_gps.c: Intel interface for gps devices
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Kuppuswamy, Sathyanarayanan
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <acpi/acpi.h>
#include <acpi/acpi_bus.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/efi.h>
#include <linux/wakelock.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_hsu.h>
#include <asm/intel_scu_flis.h>
#include <linux/intel_mid_gps.h>
#include <linux/lnw_gpio.h>
#ifdef CONFIG_BROADCOM_BCM4774_GPS
#include <linux/hrtimer.h>
#include <linux/delay.h>
#endif

#define DRIVER_NAME "intel_mid_gps"

#define ACPI_DEVICE_ID_BCM4752  "BCM4752"
#define ACPI_DEVICE_ID_BCM47521 "BCM47521"
#define ACPI_DEVICE_ID_BCM47531 "BCM47531"

struct device *tty_dev = NULL;
struct wake_lock hostwake_lock;

#ifdef CONFIG_BROADCOM_BCM4774_GPS
struct bcm_gps_lpm {
    int mcu_resp_gpio_pin;
    int mcu_req_gpio_pin;

    struct hrtimer enter_lpm_timer;
    ktime_t enter_lpm_delay;

    struct device *tty_dev;

    int port;
} gps_lpm;

static bool bcm477x_hello(void)
{
    int count=0, retries=0;

    gpio_set_value(gps_lpm.mcu_req_gpio_pin, 1);
    while (!gpio_get_value(gps_lpm.mcu_resp_gpio_pin)) {
    	if (count++ > 100) {
    	    gpio_set_value(gps_lpm.mcu_req_gpio_pin, 0);
    	    return false;
    	}

    	mdelay(1);

    	/*if awake, done */
    	if (gpio_get_value(gps_lpm.mcu_resp_gpio_pin)) break;

    	if (count%20==0 && retries++ < 3) {
    	    gpio_set_value(gps_lpm.mcu_req_gpio_pin, 0);
    	    mdelay(1);
    	    gpio_set_value(gps_lpm.mcu_req_gpio_pin, 1);
    	    mdelay(1);
    	}
    }
    return true;
}

/*
 * bcm4773_bye - set mcu_req low to let chip go to sleep
 *
 */
static void bcm477x_bye(void)
{
    gpio_set_value(gps_lpm.mcu_req_gpio_pin, 0);
}
#endif

/*********************************************************************
 *		Driver GPIO toggling functions
 *********************************************************************/

static int intel_mid_gps_reset(struct device *dev, const char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);

	if (!buf)
		return -EINVAL;

	if (!gps)
		return -EINVAL;

	if (sscanf(buf, "%i", &gps->reset) != 1)
		return -EINVAL;

	if (gpio_is_valid(gps->gpio_reset))
		gpio_set_value(gps->gpio_reset,
				gps->reset ? RESET_ON : RESET_OFF);
	else
		return -EINVAL;

	return 0;
}

static int intel_mid_gps_enable(struct device *dev, const char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);
	set_flis_value(0x3221, 0x2120);

	if (!buf)
		return -EINVAL;

	if (!gps)
		return -EINVAL;

	if (sscanf(buf, "%i", &gps->enable) != 1)
		return -EINVAL;

	if (gpio_is_valid(gps->gpio_enable))
		gpio_set_value(gps->gpio_enable,
				gps->enable ? ENABLE_ON : ENABLE_OFF);
	else
		return -EINVAL;

	return 0;
}

/*********************************************************************
 *		Driver sysfs attribute functions
 *********************************************************************/

static ssize_t intel_mid_gps_reset_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);

	if (!gps)
		return -EINVAL;

	return sprintf(buf, "%d\n", gps->reset);
}

static ssize_t intel_mid_gps_reset_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	ret = intel_mid_gps_reset(dev, buf);

	return !ret ? size : ret;
}

static ssize_t intel_mid_gps_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct intel_mid_gps_platform_data *gps = dev_get_drvdata(dev);

	if (!gps)
		return -EINVAL;

	return sprintf(buf, "%d\n", gps->enable);
}

static ssize_t intel_mid_gps_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	int ret;

	ret = intel_mid_gps_enable(dev, buf);

	return !ret ? size : ret;
}

static DEVICE_ATTR(reset, S_IRUGO|S_IWUSR, intel_mid_gps_reset_show,
				intel_mid_gps_reset_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, intel_mid_gps_enable_show,
				intel_mid_gps_enable_store);

static struct attribute *intel_mid_gps_attrs[] = {
	&dev_attr_reset.attr,
	&dev_attr_enable.attr,
	NULL,
};

static struct attribute_group intel_mid_gps_attr_group = {
	.name = DRIVER_NAME,
	.attrs = intel_mid_gps_attrs,
};

/*********************************************************************
 *		Driver GPIO probe/remove functions
 *********************************************************************/

static irqreturn_t intel_mid_gps_hostwake_isr(int irq, void *dev)
{
	struct intel_mid_gps_platform_data *pdata = dev_get_drvdata(dev);
	int hostwake;

	hostwake = gpio_get_value(pdata->gpio_hostwake);

	tty_dev = intel_mid_hsu_set_wake_peer(pdata->hsu_port, NULL);
	if (!tty_dev) {
		pr_err("%s: unable to get the HSU tty device \n", __func__);
	}

	irq_set_irq_type(irq, hostwake ? IRQF_TRIGGER_FALLING :
			 IRQF_TRIGGER_RISING);

	if (hostwake) {
		wake_lock(&hostwake_lock);
		if (tty_dev)
			pm_runtime_get(tty_dev);
	}
	else {
		if (tty_dev)
			pm_runtime_put(tty_dev);
		wake_unlock(&hostwake_lock);
	}

	return IRQ_HANDLED;
}

#ifdef CONFIG_BROADCOM_BCM4774_GPS
static enum hrtimer_restart gps_enter_lpm(struct hrtimer *timer)
{
	bcm477x_bye();

	return HRTIMER_NORESTART;
}
static void bcm_gps_lpm_wake_peer(struct device *dev)
{
	gps_lpm.tty_dev = dev;

	hrtimer_try_to_cancel(&gps_lpm.enter_lpm_timer);

	bcm477x_hello();

	hrtimer_start(&gps_lpm.enter_lpm_timer, gps_lpm.enter_lpm_delay,
		HRTIMER_MODE_REL);
}
#endif

static int intel_mid_gps_init(struct platform_device *pdev)
{
	int ret;
	struct intel_mid_gps_platform_data *pdata = dev_get_drvdata(&pdev->dev);
	struct kset *sysdev;
	struct kobject *plat;

#ifdef CONFIG_BROADCOM_BCM4774_GPS
	gps_lpm.port = CONFIG_BROADCOM_BCM4774_GPS_UART_PORT;

	hrtimer_init(&gps_lpm.enter_lpm_timer, CLOCK_MONOTONIC, HRTIMER_MODE_REL);
	gps_lpm.enter_lpm_delay = ktime_set(1, 0);  /* 1 sec */
	gps_lpm.enter_lpm_timer.function = gps_enter_lpm;
#endif

	/* we need to rename the sysfs entry to match the one created with SFI,
	   and we are sure that there is always one GPS per platform */
	if (ACPI_HANDLE(&pdev->dev)) {
		ret = sysfs_rename_dir(&pdev->dev.kobj, DRIVER_NAME);
		if (ret)
			pr_err("%s: failed to rename sysfs entry\n", __func__);
	}

	ret = sysfs_create_group(&pdev->dev.kobj, &intel_mid_gps_attr_group);
	if (ret)
		dev_err(&pdev->dev,
			"Failed to create intel_mid_gps sysfs interface\n");

	/* With ACPI device tree, GPS is UART child, we need to create symlink at
	   /sys/devices/platform/ level (user libgps is expecting this). */
	if (ACPI_HANDLE(&pdev->dev)) {
		sysdev = pdev->dev.kobj.kset;
		if (sysdev) {
			plat = kset_find_obj(sysdev, "platform");
			if (plat) {
				ret = sysfs_create_link(plat,
					&pdev->dev.kobj, DRIVER_NAME);
				if (ret)
					dev_err(&pdev->dev,
					"%s: symlink creation failed\n", __func__);
			}
		}
	}

	/* Handle reset GPIO */
	if (gpio_is_valid(pdata->gpio_reset)) {

		/* Request gpio */
		ret = gpio_request(pdata->gpio_reset, "intel_mid_gps_reset");
		if (ret < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
					__func__, pdata->gpio_reset, ret);
			goto error_gpio_reset_request;
		}

		/* Force GPIO muxmode */
		lnw_gpio_set_alt(pdata->gpio_reset, LNW_GPIO);

		/* set gpio direction */
		ret = gpio_direction_output(pdata->gpio_reset, pdata->reset);
		if (ret < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
					__func__, pdata->gpio_reset, ret);
			goto error_gpio_reset_direction;
		}
	}

	/* Handle enable GPIO */
	if (gpio_is_valid(pdata->gpio_enable)) {

		/* Request gpio */
		ret = gpio_request(pdata->gpio_enable, "intel_mid_gps_enable");
		if (ret < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
					__func__, pdata->gpio_enable, ret);
			goto error_gpio_enable_request;
		}

		/* Force GPIO muxmode */
		lnw_gpio_set_alt(pdata->gpio_enable, LNW_GPIO);

		/* set gpio direction */
		ret = gpio_direction_output(pdata->gpio_enable, pdata->enable);
		if (ret < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
					__func__, pdata->gpio_enable, ret);
			goto error_gpio_enable_direction;
		}

		ret = gpio_export(pdata->gpio_enable, false);
		if (ret < 0) {
		    pr_err("%s: Unable to export GPIO:%d err:%d\n",
			           __func__, pdata->gpio_enable, ret);
		}
	}

	/* Handle hostwake GPIO */
	if (gpio_is_valid(pdata->gpio_hostwake)) {
		int irq_id = -EINVAL;

		/* Request gpio */
		ret = gpio_request(pdata->gpio_hostwake, "intel_mid_gps_hostwake");
		if (ret < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
					__func__, pdata->gpio_hostwake, ret);
			goto error_gpio_hostwake_request;
		}

		/* Force GPIO muxmode */
		lnw_gpio_set_alt(pdata->gpio_hostwake, LNW_GPIO);

		/* set gpio direction */
		ret = gpio_direction_input(pdata->gpio_hostwake);
		if (ret < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
					__func__, pdata->gpio_hostwake, ret);
			goto error_gpio_hostwake_direction;
		}

		/* configure irq handling */
		irq_id = gpio_to_irq(pdata->gpio_hostwake);
		irq_set_irq_wake(irq_id, 1);

		ret = request_irq(irq_id, intel_mid_gps_hostwake_isr,
				  IRQF_TRIGGER_RISING,
				  "gps_hostwake", &pdev->dev);
		if (ret) {
			pr_err("%s: unable to request irq %d \n", __func__, irq_id);
			goto error_gpio_hostwake_direction;
		}

		wake_lock_init(&hostwake_lock, WAKE_LOCK_SUSPEND, "gps_hostwake_lock");
	}

#ifdef CONFIG_BROADCOM_BCM4774_GPS
	tty_dev = intel_mid_hsu_set_wake_peer(1, bcm_gps_lpm_wake_peer);
	if (!tty_dev) {
		pr_err("Error no tty dev");
		return -ENODEV;
	}

	bcm_gps_lpm_wake_peer(tty_dev);
#endif

	return 0;

error_gpio_hostwake_direction:
	gpio_free(pdata->gpio_hostwake);
error_gpio_hostwake_request:
error_gpio_enable_direction:
	gpio_free(pdata->gpio_enable);
error_gpio_enable_request:
error_gpio_reset_direction:
	gpio_free(pdata->gpio_reset);
error_gpio_reset_request:
	sysfs_remove_group(&pdev->dev.kobj, &intel_mid_gps_attr_group);

	return ret;
}

static void intel_mid_gps_deinit(struct platform_device *pdev)
{
	struct intel_mid_gps_platform_data *pdata = dev_get_drvdata(&pdev->dev);

	if (gpio_is_valid(pdata->gpio_enable))
		gpio_free(pdata->gpio_enable);

	if (gpio_is_valid(pdata->gpio_reset))
		gpio_free(pdata->gpio_reset);

	if (gpio_is_valid(pdata->gpio_hostwake)) {
		free_irq(gpio_to_irq(pdata->gpio_hostwake), &pdev->dev);
		gpio_free(pdata->gpio_hostwake);
		wake_lock_destroy(&hostwake_lock);
	}

	sysfs_remove_group(&pdev->dev.kobj, &intel_mid_gps_attr_group);
}

static int intel_mid_gps_probe(struct platform_device *pdev)
{
	struct intel_mid_gps_platform_data *pdata = NULL;
	int ret = 0;
	int debug_gpio_pin = 0;

	pr_info("%s probe called\n", dev_name(&pdev->dev));

	if (ACPI_HANDLE(&pdev->dev)) {
		/* create a new platform data that will be
		   populated with gpio data from ACPI table */
		unsigned long long port;
		pdata = kzalloc(sizeof(struct intel_mid_gps_platform_data),
			GFP_KERNEL);

		if (!pdata)
			return -ENOMEM;

		pdata->gpio_reset = acpi_get_gpio_by_name(&pdev->dev,"RSET", NULL);
		pdata->gpio_enable = acpi_get_gpio_by_name(&pdev->dev,"ENAB", NULL);
		pdata->gpio_hostwake = acpi_get_gpio_by_name(&pdev->dev,"HSTW", NULL);

		pr_info("%s enable: %d, reset: %d, hostwake: %d\n", __func__,
			pdata->gpio_enable, pdata->gpio_reset, pdata->gpio_hostwake);

		if (ACPI_FAILURE(acpi_evaluate_integer((acpi_handle)ACPI_HANDLE(&pdev->dev),
						       "UART", NULL, &port)))
			dev_err(&pdev->dev, "Error evaluating acpi table, no HSU port\n");
		else {
			pdata->hsu_port = (unsigned int)port;
			pr_info("GPS HSU port read from ACPI = %d\n", pdata->hsu_port);
		}

		platform_set_drvdata(pdev, pdata);
	} else {
		platform_set_drvdata(pdev, pdev->dev.platform_data);

#ifdef UART_DEBUG
		debug_gpio_pin = get_gpio_by_name("AUDIO_DEBUG#");
		if (debug_gpio_pin > 0) {
		    ret = gpio_request_one(debug_gpio_pin, GPIOF_DIR_OUT, "AUDIO_DEBUG#");
		    if (ret)
			pr_err("gpio_request AUDIO_DEBUG failed!\n");
		    gpio_direction_output(debug_gpio_pin, 0);
		}
#else
		debug_gpio_pin = get_gpio_by_name("AUDIO_DEBUG#");
		if (debug_gpio_pin > 0) {
		    ret = gpio_request_one(debug_gpio_pin, GPIOF_DIR_OUT, "AUDIO_DEBUG#");
		    if (ret)
			pr_err("gpio_request AUDIO_DEBUG failed!\n");
		    gpio_direction_output(debug_gpio_pin, 1);
		}
#endif

#ifdef CONFIG_BROADCOM_BCM4774_GPS
		gps_lpm.mcu_req_gpio_pin = get_gpio_by_name("AP_GPS_REQ");
		if (gpio_is_valid(gps_lpm.mcu_req_gpio_pin)) {
		    ret = gpio_request(gps_lpm.mcu_req_gpio_pin, "MCU_REQ");
		    if (ret < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
			       __func__, gps_lpm.mcu_req_gpio_pin, ret);
		    }
		    ret = gpio_direction_output(gps_lpm.mcu_req_gpio_pin, 0);
		    if (ret < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
			       __func__, gps_lpm.mcu_req_gpio_pin, ret);
		    }
		}

		gps_lpm.mcu_resp_gpio_pin = get_gpio_by_name("AP_GPS_RESP");
		if (gpio_is_valid(gps_lpm.mcu_resp_gpio_pin)) {
		    ret = gpio_request(gps_lpm.mcu_resp_gpio_pin, "MCU_RESP");
		    if (ret < 0) {
			pr_err("%s: Unable to request GPIO:%d, err:%d\n",
			       __func__, gps_lpm.mcu_resp_gpio_pin, ret);
		    }
		    /* set gpio direction */
		    ret = gpio_direction_input(gps_lpm.mcu_resp_gpio_pin);
		    if (ret < 0) {
			pr_err("%s: Unable to set GPIO:%d direction, err:%d\n",
			       __func__, gps_lpm.mcu_resp_gpio_pin, ret);
		    }
		}
#endif
	}

	ret = intel_mid_gps_init(pdev);

	if (ret) {
		dev_err(&pdev->dev, "Failed to initalize %s\n",
			dev_name(&pdev->dev));

		if (ACPI_HANDLE(&pdev->dev))
			kfree(pdata);
	}
	return ret;
}

static int intel_mid_gps_remove(struct platform_device *pdev)
{
	intel_mid_gps_deinit(pdev);

	if (ACPI_HANDLE(&pdev->dev))
		kfree(dev_get_drvdata(&pdev->dev));

	return 0;
}

static void intel_mid_gps_shutdown(struct platform_device *pdev)
{
	struct intel_mid_gps_platform_data *pdata = dev_get_drvdata(&pdev->dev);

	pr_info("%s shutdown called\n", dev_name(&pdev->dev));

	/* Turn gps off if not done already */
	if (gpio_is_valid(pdata->gpio_enable))
		gpio_set_value(pdata->gpio_enable, ENABLE_OFF);
}

/*********************************************************************
 *		Driver initialisation and finalization
 *********************************************************************/

#ifdef CONFIG_ACPI
static struct acpi_device_id acpi_gps_id_table[] = {
	/* ACPI IDs here */
	{ ACPI_DEVICE_ID_BCM4752,  0 },
	{ ACPI_DEVICE_ID_BCM47521, 0 },
	{ ACPI_DEVICE_ID_BCM47531, 0 },
	{ }
};

MODULE_DEVICE_TABLE(acpi, acpi_gps_id_table);
#endif

static struct platform_driver intel_mid_gps_driver = {
	.probe		= intel_mid_gps_probe,
	.remove		= intel_mid_gps_remove,
	.driver		= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(acpi_gps_id_table),
#endif
	},
	.shutdown	= intel_mid_gps_shutdown,
};

static int __init intel_mid_gps_driver_init(void)
{
	return platform_driver_register(&intel_mid_gps_driver);
}

static void __exit intel_mid_gps_driver_exit(void)
{
	platform_driver_unregister(&intel_mid_gps_driver);
}

module_init(intel_mid_gps_driver_init);
module_exit(intel_mid_gps_driver_exit);

MODULE_AUTHOR("Kuppuswamy, Sathyanarayanan");
MODULE_DESCRIPTION("Intel MID GPS driver");
MODULE_LICENSE("GPL");
