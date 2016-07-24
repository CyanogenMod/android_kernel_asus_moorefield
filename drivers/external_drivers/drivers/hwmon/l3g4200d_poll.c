/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
*
* File Name		: l3g4200d_poll.c
* Authors		: MH - C&I BU - Application Team
*			: Carmine Iascone (carmine.iascone@st.com)
*			: Matteo Dameno (matteo.dameno@st.com)
*			: Both authors are willing to be considered the contact
*			: and update points for the driver.
* Version		: V 1.1 sysfs
* Date			: 2011/02/28
* Description		: L3G4200D digital output gyroscope sensor API
*
********************************************************************************
*
* This program is free software; you can redistribute it and/or modify
* it under the terms of the GNU General Public License version 2 as
* published by the Free Software Foundation.
*
* THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
* OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
* PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*
********************************************************************************
* REVISON HISTORY
*
* VERSION	| DATE		| AUTHORS		| DESCRIPTION
* 1.0		| 2010/11/19	| Carmine Iascone	| First Release
* 1.1		| 2011/02/28	| Matteo Dameno		| Self Test Added
*******************************************************************************/
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/input.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/input/l3g4200d_poll.h>
#include <linux/delay.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>

/* l3g4200d gyroscope registers */
#define WHO_AM_I        0x0F

#define CTRL_REG1       0x20    /* CTRL REG1 */
#define CTRL_REG2       0x21    /* CTRL REG2 */
#define CTRL_REG3       0x22    /* CTRL_REG3 */
#define CTRL_REG4       0x23    /* CTRL_REG4 */
#define CTRL_REG5       0x24    /* CTRL_REG5 */

/* CTRL_REG1 */
#define PM_ON		0x08
#define ENABLE_ALL_AXES	0x07
#define BW00		0x00
#define BW01		0x10
#define BW10		0x20
#define BW11		0x30
#define ODR100		0x00  /* ODR = 100Hz */
#define ODR200		0x40  /* ODR = 200Hz */
#define ODR400		0x80  /* ODR = 400Hz */
#define ODR800		0xC0  /* ODR = 800Hz */
#define ODR_MASK        0xF0

/* CTRL_REG4 bits */
#define	FS_MASK				0x30
#define	SELFTEST_MASK			0x06
#define L3G4200D_SELFTEST_DIS		0x00
#define L3G4200D_SELFTEST_EN_POS	0x02
#define L3G4200D_SELFTEST_EN_NEG	0x04

#define AXISDATA_REG    0x28
#define AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RESUME_ENTRIES		5

#define MAX_POLL_DELAY 500

/** Registers Contents */
#define WHOAMI_L3G4200D		0x00D3	/* Expected content for WAI register*/
#define WHOAMI_L3GD20		0x00D4  /* WAI register for l3gd20 */
#define WHOAMI_L3GD20H		0x00D7  /* WAI register for l3gd20h */

/* After device enable a short delay
 * is needed for device to be stable.
 */
#define L3GD20_STABLE_DELAY	100
#define L3G4200D_STABLE_DELAY	300

struct output_rate {
	int poll_rate_ms;
	u8 mask;
};

static const struct output_rate odr_table[] = {
	{2, ODR800|BW10},
	{3, ODR400|BW01},
	{5, ODR200|BW00},
	{10, ODR100|BW00},
};

struct l3g4200d_data {
	struct i2c_client *client;
	struct l3g4200d_gyr_platform_data *pdata;

	struct mutex lock;

	struct input_dev *input_dev;
	u8 resume_state[RESUME_ENTRIES];
	struct hrtimer work_timer;
	struct completion report_complete;
	struct task_struct *thread;
	bool hrtimer_running;
	int need_resume;
	int hw_init_delay;
};

static void l3g4200d_update_odr_bits(struct l3g4200d_data *gyro)
{
	int i;

	/* find the lowest ODR that could meet the poll interval requirement.
	 * If couldn't find one, use the highest one.
	 */
	for (i = ARRAY_SIZE(odr_table) - 1; i > 0; i--) {
		if (odr_table[i].poll_rate_ms <= gyro->pdata->poll_interval)
			break;
	}

	gyro->resume_state[RES_CTRL_REG1] &= (~ODR_MASK);
	gyro->resume_state[RES_CTRL_REG1] |= odr_table[i].mask;
}

static int l3g4200d_hw_init(struct l3g4200d_data *gyro)
{
	int ret;

	/* check if the chip reports the correct id */
	ret = i2c_smbus_read_byte_data(gyro->client, WHO_AM_I);
	if (ret < 0)
		return ret;

	switch (ret) {
	case WHOAMI_L3G4200D:
		gyro->hw_init_delay = L3G4200D_STABLE_DELAY;
		break;
	case WHOAMI_L3GD20:
	case WHOAMI_L3GD20H:
		gyro->hw_init_delay = L3GD20_STABLE_DELAY;
		break;
	default:
		dev_err(&gyro->client->dev, "Invalid device id, %x", ret);
		return -EINVAL;
	}

	/* set default settings, and put gyro in power down mode */
	gyro->resume_state[RES_CTRL_REG1] = ENABLE_ALL_AXES;
	gyro->resume_state[RES_CTRL_REG2] = 0x00;
	gyro->resume_state[RES_CTRL_REG3] = 0x00;
	gyro->resume_state[RES_CTRL_REG4] = gyro->pdata->fs_range;
	gyro->resume_state[RES_CTRL_REG5] = 0x00;
	l3g4200d_update_odr_bits(gyro);

	ret = i2c_smbus_write_byte_data(gyro->client, CTRL_REG1,
		gyro->resume_state[RES_CTRL_REG1]);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(gyro->client, CTRL_REG2,
		gyro->resume_state[RES_CTRL_REG2]);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(gyro->client, CTRL_REG3,
		gyro->resume_state[RES_CTRL_REG3]);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(gyro->client, CTRL_REG4,
		gyro->resume_state[RES_CTRL_REG4]);
	if (ret < 0)
		return ret;

	ret = i2c_smbus_write_byte_data(gyro->client, CTRL_REG5,
		gyro->resume_state[RES_CTRL_REG5]);
	if (ret < 0)
		return ret;

	return ret;
}

static void l3g4200d_queue_delayed_work(struct l3g4200d_data *gyro, int more)
{
	ktime_t poll_delay;

	poll_delay = ktime_set(0, (gyro->pdata->poll_interval + more) * NSEC_PER_MSEC);

	hrtimer_start(&gyro->work_timer, poll_delay, HRTIMER_MODE_REL);
	gyro->hrtimer_running = true;
}

static void l3g4200d_report_data(struct l3g4200d_data *gyro)
{
	u8 buf[6] = { 0 };
	s16 hw_d[3];
	s16 x, y, z;
	int ret;

	ret = i2c_smbus_read_i2c_block_data(gyro->client,
			AUTO_INCREMENT | AXISDATA_REG,
			sizeof(buf), &buf[0]);
	if (ret < 0) {
		dev_err(&gyro->client->dev, "Failed to read axis data.\n");
		return;
	}

	hw_d[0] = (s16) ((buf[1] << 8) | buf[0]);
	hw_d[1] = (s16) ((buf[3] << 8) | buf[2]);
	hw_d[2] = (s16) ((buf[5] << 8) | buf[4]);

	x = ((gyro->pdata->negate_x) ? (-hw_d[gyro->pdata->axis_map_x])
			: (hw_d[gyro->pdata->axis_map_x]));
	y = ((gyro->pdata->negate_y) ? (-hw_d[gyro->pdata->axis_map_y])
			: (hw_d[gyro->pdata->axis_map_y]));
	z = ((gyro->pdata->negate_z) ? (-hw_d[gyro->pdata->axis_map_z])
			: (hw_d[gyro->pdata->axis_map_z]));

#ifdef DEBUG
	{
		struct timeval now;
		jiffies_to_timeval(jiffies, &now);
		dev_dbg(&gyro->client->dev, "%ld.%ld: X=%d, Y=%d, Z=%d",
				now.tv_sec, now.tv_usec,
				(int) x,
				(int) y,
				(int) z);
	}
#endif

	input_report_rel(gyro->input_dev, REL_X, x);
	input_report_rel(gyro->input_dev, REL_Y, y);
	input_report_rel(gyro->input_dev, REL_Z, z);
	input_sync(gyro->input_dev);
}

static int l3g4200d_enabled(struct l3g4200d_data *gyro)
{
	return gyro->resume_state[RES_CTRL_REG1] & PM_ON;
}

static int report_event(void *data)
{
	struct l3g4200d_data *gyro = data;
	while(1)
	{
		/* wait for report event */
		wait_for_completion(&gyro->report_complete);

		mutex_lock(&gyro->lock);
		if (!l3g4200d_enabled(gyro)) {
			mutex_unlock(&gyro->lock);
			continue;
		}
		l3g4200d_report_data(gyro);
		mutex_unlock(&gyro->lock);
        }

	return 0;
}

static enum hrtimer_restart l3g4200d_poll_work(struct hrtimer *timer)
{
	struct l3g4200d_data *gyro =
		container_of(timer, struct l3g4200d_data, work_timer);

	if (!l3g4200d_enabled(gyro))
		return HRTIMER_NORESTART;

	complete(&gyro->report_complete);
	l3g4200d_queue_delayed_work(gyro, 0);

	return HRTIMER_NORESTART;
}

static int l3g4200d_enable(struct l3g4200d_data *gyro)
{
	int err;

	if (l3g4200d_enabled(gyro))
		return 0;

	/* Change power down mode bit of CTRL_REG1 to enable the gyro chip */
	gyro->resume_state[RES_CTRL_REG1] |= PM_ON;
	err = i2c_smbus_write_byte_data(gyro->client, CTRL_REG1,
			gyro->resume_state[RES_CTRL_REG1]);
	if (err < 0) {
		dev_err(&gyro->client->dev, "Failed to enable gyro.\n");
		return err;
	}

	l3g4200d_queue_delayed_work(gyro, gyro->hw_init_delay);
	return 0;
}

static int l3g4200d_disable(struct l3g4200d_data *gyro)
{
	int err;

	if (!l3g4200d_enabled(gyro))
		return 0;

	/* Change power down mode bit of CTRL_REG1 to disable the gyro chip */
	gyro->resume_state[RES_CTRL_REG1] &= ~PM_ON;
	err = i2c_smbus_write_byte_data(gyro->client, CTRL_REG1,
			gyro->resume_state[RES_CTRL_REG1]);
	if (err < 0) {
		dev_err(&gyro->client->dev, "Failed to disable gyro.\n");
		return err;
	}

	if(gyro->hrtimer_running) {
		gyro->hrtimer_running = false;
		hrtimer_cancel(&gyro->work_timer);
	}

	return 0;
}

static ssize_t attr_polling_rate_show(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	mutex_lock(&gyro->lock);
	val = gyro->pdata->poll_interval;
	mutex_unlock(&gyro->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_polling_rate_store(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long interval_ms;
	int saved_interval;
	int err;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if (interval_ms < (unsigned long) odr_table[0].poll_rate_ms) {
		dev_info(dev, "polling interval is too small, set it to %d\n",
				odr_table[0].poll_rate_ms);
		interval_ms = odr_table[0].poll_rate_ms;
	}

	/* set max poll interval to be 500 ms */
	if (interval_ms > MAX_POLL_DELAY) {
		dev_info(dev, "polling interval is too big, set it to 500\n");
		interval_ms = MAX_POLL_DELAY;
	}

	mutex_lock(&gyro->lock);

	saved_interval = gyro->pdata->poll_interval;
	gyro->pdata->poll_interval = (int)interval_ms;
	l3g4200d_update_odr_bits(gyro);
	err = i2c_smbus_write_byte_data(gyro->client, CTRL_REG1,
			gyro->resume_state[RES_CTRL_REG1]);
	if (err < 0) {
		dev_err(&gyro->client->dev, "Failed to change poll interval.\n");
		gyro->pdata->poll_interval = saved_interval;
		l3g4200d_update_odr_bits(gyro);
		mutex_unlock(&gyro->lock);
		return -EIO;
	}

	mutex_unlock(&gyro->lock);

	return size;
}

static ssize_t attr_range_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int range = 0;
	u8 val;

	mutex_lock(&gyro->lock);

	val = gyro->resume_state[RES_CTRL_REG4] & FS_MASK;
	switch (val) {
	case L3G4200D_GYR_FS_250DPS:
		range = 250;
		break;
	case L3G4200D_GYR_FS_500DPS:
		range = 500;
		break;
	case L3G4200D_GYR_FS_2000DPS:
	default:
		range = 2000;
		break;
	}

	mutex_unlock(&gyro->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_range_store(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;
	u8 saved;
	u8 range_bits;
	int ret;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val != 250 && val != 500 && val != 2000)
		return -EINVAL;

	mutex_lock(&gyro->lock);
	switch (val) {
	case 250:
		range_bits = L3G4200D_GYR_FS_250DPS;
		break;
	case 500:
		range_bits = L3G4200D_GYR_FS_500DPS;
		break;
	case 2000:
	default:
		range_bits = L3G4200D_GYR_FS_2000DPS;
		break;
	}

	saved = gyro->resume_state[RES_CTRL_REG4];
	gyro->resume_state[RES_CTRL_REG4] &= ~FS_MASK;
	gyro->resume_state[RES_CTRL_REG4] |= range_bits;
	ret = i2c_smbus_write_byte_data(gyro->client, CTRL_REG4,
			gyro->resume_state[RES_CTRL_REG4]);
	if (ret < 0) {
		dev_err(&gyro->client->dev, "Failed to change fs range.\n");
		gyro->resume_state[RES_CTRL_REG4] = saved;
		mutex_unlock(&gyro->lock);
		return ret;
	}

	mutex_unlock(&gyro->lock);
	return size;
}

static ssize_t attr_enable_show(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int val;

	mutex_lock(&gyro->lock);
	val = (gyro->resume_state[RES_CTRL_REG1] & PM_ON) ? 1 : 0;
	mutex_unlock(&gyro->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_enable_store(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);
	if (val)
		l3g4200d_enable(gyro);
	else
		l3g4200d_disable(gyro);
	mutex_unlock(&gyro->lock);

	return size;
}

static ssize_t attr_get_selftest(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	u8  self_test_bits;
	int val;

	mutex_lock(&gyro->lock);
	self_test_bits = gyro->resume_state[RES_CTRL_REG4] & SELFTEST_MASK;
	mutex_unlock(&gyro->lock);

	switch (self_test_bits) {
	case L3G4200D_SELFTEST_EN_POS:
		val = 1;
		break;
	case L3G4200D_SELFTEST_EN_NEG:
		val = -1;
		break;
	default:
		val = 0;
		break;
	}

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_selftest(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	long val;
	u8 saved;
	int ret;

	if (strict_strtol(buf, 10, &val))
		return -EINVAL;

	mutex_lock(&gyro->lock);

	saved = gyro->resume_state[RES_CTRL_REG4];
	gyro->resume_state[RES_CTRL_REG4] &= ~SELFTEST_MASK;
	if (val < 0)
		gyro->resume_state[RES_CTRL_REG4] |= L3G4200D_SELFTEST_EN_NEG;
	else if (val > 0)
		gyro->resume_state[RES_CTRL_REG4] |= L3G4200D_SELFTEST_EN_POS;
	else
		gyro->resume_state[RES_CTRL_REG4] |= L3G4200D_SELFTEST_DIS;

	ret = i2c_smbus_write_byte_data(gyro->client, CTRL_REG4,
			gyro->resume_state[RES_CTRL_REG4]);
	if (ret < 0) {
		dev_err(&gyro->client->dev, "Failed to change fs range.\n");
		gyro->resume_state[RES_CTRL_REG4] = saved;
		mutex_unlock(&gyro->lock);
		return ret;
	}

	mutex_unlock(&gyro->lock);

	return size;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	ssize_t data_size = 0;
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);
	int rc;
	int i;
	u8 ctl_regs[5] = { 0 };

	/* read and display the 5 control registers */
	mutex_lock(&gyro->lock);
	rc = i2c_smbus_read_i2c_block_data(gyro->client,
			AUTO_INCREMENT | CTRL_REG1,
			sizeof(ctl_regs), &ctl_regs[0]);
	if (rc < 0) {
		dev_err(&gyro->client->dev,
				"Failed to read control register data.\n");
		mutex_unlock(&gyro->lock);
		return -EINVAL;
	}
	mutex_unlock(&gyro->lock);

	for (i = 0; i < 5; ++i)
		data_size += sprintf(&buf[data_size], "CTRL_REG%d=0x%x\n",
				i, (unsigned int)ctl_regs[i]);
	return data_size;
}


static struct device_attribute attributes[] = {
	__ATTR(poll, 0644, attr_polling_rate_show, attr_polling_rate_store),
	__ATTR(enable, 0644, attr_enable_show, attr_enable_store),
	__ATTR(range, 0644, attr_range_show, attr_range_store),
	__ATTR(enable_selftest, 0644, attr_get_selftest, attr_set_selftest),
	__ATTR(registers, 0400, attr_reg_get, NULL),
};

static int create_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto error;
	return 0;

error:
	for (i = i - 1; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int remove_sysfs_interfaces(struct device *dev)
{
	int i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
	return 0;
}

static int l3g4200d_input_init(struct l3g4200d_data *gyro)
{
	int err;
	struct device *dev = &gyro->client->dev;

	gyro->input_dev = input_allocate_device();
	if (!gyro->input_dev) {
		dev_err(dev, "input device allocation failed.\n");
		return -ENOMEM;
	}

	gyro->input_dev->name = L3G4200D_GYR_DEV_NAME;
	gyro->input_dev->id.bustype = BUS_I2C;
	gyro->input_dev->dev.parent = dev;
	input_set_drvdata(gyro->input_dev, gyro);

	set_bit(EV_REL, gyro->input_dev->evbit);
	set_bit(REL_X, gyro->input_dev->relbit);
	set_bit(REL_Y, gyro->input_dev->relbit);
	set_bit(REL_Z, gyro->input_dev->relbit);

	err = input_register_device(gyro->input_dev);
	if (err) {
		dev_err(dev, "unable to register input device.\n");
		input_free_device(gyro->input_dev);
		return err;
	}

	return 0;
}

static int l3g4200d_validate_pdata(struct l3g4200d_data *gyro)
{
	if (gyro->pdata->axis_map_x > 2 ||
			gyro->pdata->axis_map_y > 2 ||
			gyro->pdata->axis_map_z > 2) {
		dev_err(&gyro->client->dev,
				"invalid axis_map value x:%u y:%u z%u\n",
				gyro->pdata->axis_map_x,
				gyro->pdata->axis_map_y,
				gyro->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (gyro->pdata->negate_x > 1 ||
			gyro->pdata->negate_y > 1 ||
			gyro->pdata->negate_z > 1) {
		dev_err(&gyro->client->dev,
				"invalid negate value x:%u y:%u z:%u\n",
				gyro->pdata->negate_x,
				gyro->pdata->negate_y,
				gyro->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (gyro->pdata->poll_interval < gyro->pdata->min_interval) {
		dev_err(&gyro->client->dev,
				"minimum poll interval violated\n");
		return -EINVAL;
	}
	return 0;
}


static int l3g4200d_probe(struct i2c_client *client,
		const struct i2c_device_id *devid)
{
	int err;
	struct l3g4200d_data *gyro;

	dev_info(&client->dev, "probe start.\n");

	if (client->dev.platform_data == NULL) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto err_out;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable.\n");
		err = -ENODEV;
		goto err_out;
	}

	gyro = kzalloc(sizeof(*gyro), GFP_KERNEL);
	if (gyro == NULL) {
		dev_err(&client->dev, "failed to allocate memory.\n");
		err = -ENOMEM;
		goto err_out;
	}

	gyro->pdata = kmalloc(sizeof(*gyro->pdata), GFP_KERNEL);
	if (gyro->pdata == NULL) {
		dev_err(&client->dev, "failed to allocate memory for pdata.");
		err = -ENOMEM;
		goto err_free_gyro;
	}
	memcpy(gyro->pdata, client->dev.platform_data, sizeof(*gyro->pdata));

	err = l3g4200d_validate_pdata(gyro);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data.\n");
		goto err_free_pdata;
	}

	gyro->client = client;
	i2c_set_clientdata(client, gyro);

	err = l3g4200d_input_init(gyro);
	if (err < 0)
		goto err_free_pdata;

	err = l3g4200d_hw_init(gyro);
	if (err < 0) {
		dev_err(&client->dev, "failed to init l3g4200d hardware.\n");
		goto err_clean_input;
	}

	err = create_sysfs_interfaces(&client->dev);
	if (err < 0)
		goto err_clean_input;

	mutex_init(&gyro->lock);
	hrtimer_init(&gyro->work_timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	gyro->work_timer.function = l3g4200d_poll_work;
	gyro->hrtimer_running = false;

	init_completion(&gyro->report_complete);
	gyro->thread = kthread_run(report_event, gyro, "gyro_report_event");
	if (IS_ERR(gyro->thread)) {
		dev_err(&client->dev,
			"unable to create report_event thread\n");
		goto err_clean_input;
	}
	dev_info(&client->dev, "probed.\n");
	return 0;

err_clean_input:
	input_unregister_device(gyro->input_dev);
err_free_pdata:
	kfree(gyro->pdata);
err_free_gyro:
	kfree(gyro);
err_out:
	dev_err(&client->dev, "Driver Initialization failed, %d\n", err);
	return err;
}

static int l3g4200d_remove(struct i2c_client *client)
{
	struct l3g4200d_data *gyro = i2c_get_clientdata(client);

	dev_info(&client->dev, "L3G4200D driver removing\n");

	remove_sysfs_interfaces(&client->dev);

	mutex_lock(&gyro->lock);
	l3g4200d_disable(gyro);
	mutex_unlock(&gyro->lock);

	kthread_stop(gyro->thread);
	mutex_destroy(&gyro->lock);
	input_unregister_device(gyro->input_dev);
	kfree(gyro->pdata);
	kfree(gyro);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int l3g4200d_suspend(struct device *dev)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	mutex_lock(&gyro->lock);
	gyro->need_resume = l3g4200d_enabled(gyro);
	l3g4200d_disable(gyro);
	mutex_unlock(&gyro->lock);

	return 0;
}

static int l3g4200d_resume(struct device *dev)
{
	struct l3g4200d_data *gyro = dev_get_drvdata(dev);

	mutex_lock(&gyro->lock);
	if (gyro->need_resume)
		l3g4200d_enable(gyro);
	mutex_unlock(&gyro->lock);

	return 0;
}

static const struct dev_pm_ops l3g4200d_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(l3g4200d_suspend,
			l3g4200d_resume)
};
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id l3g4200d_id[] = {
	{ L3G4200D_GYR_DEV_NAME , 0 },
	{ L3GD20H_GYR_DEV_NAME , 0 },
	{},
};

MODULE_DEVICE_TABLE(i2c, l3g4200d_id);

static struct i2c_driver l3g4200d_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = L3G4200D_GYR_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &l3g4200d_pm_ops,
#endif /* CONFIG_PM_SLEEP */
	},
	.probe = l3g4200d_probe,
	.remove = l3g4200d_remove,
	.id_table = l3g4200d_id,
};

static int __init l3g4200d_init(void)
{
	pr_info("%s: l3g4200d polling driver init\n", L3G4200D_GYR_DEV_NAME);
	return i2c_add_driver(&l3g4200d_driver);
}

static void __exit l3g4200d_exit(void)
{
	pr_info("L3G4200D polling driver exit\n");
	i2c_del_driver(&l3g4200d_driver);
	return;
}

module_init(l3g4200d_init);
module_exit(l3g4200d_exit);

MODULE_DESCRIPTION("l3g4200d digital gyroscope sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");
