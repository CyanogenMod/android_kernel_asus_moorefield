/*
 * lsm303dlhc_compass.c - ST LSM303DLHC Compass Driver
 *
 * Copyright (C) 2010 Intel Corp
 *
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 */
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/sysfs.h>
#include <linux/workqueue.h>
#include <linux/input.h>

#define DRIVER_NAME "lsm303cmp"

#define LSM303CMP_REG_CRA 0x00
	#define LSM303CMP_DOR_75 (6 << 2)
#define LSM303CMP_REG_CRB 0x01
	#define LSM303CMP_GAIN_600 (3 << 5)
#define LSM303CMP_REG_MODE 0x02
	#define LSM303CMP_MODE_CONT 0
	#define LSM303CMP_MODE_SLEEP (1 << 1)
#define LSM303CMP_REG_OUT_X 0x03
	#define LSM303CMP_BLOCK_READ (1 << 7)
#define LSM303CMP_REG_SR 0x09
	#define LSM303CMP_LOCK (1 << 1)
	#define LSM303CMP_DRDY (1 << 0)

#define LSM303CMP_MAX_DOR 75
#define LSM303CMP_DEFAULT_POLL_MS 200

struct lsm303cmp_driver_data {
	struct i2c_client *client;
	struct input_dev *input_device;

	int enabled;
	int need_resume;
	int poll_ms;

	/*
	 * This mutex protect lsm303cmp from data race condition.
	 * Any function need to conmmunicate with lsm303cmp device
	 * or to set lsm303cmp delay_ms and enabled state should acquire
	 * this lock first.
	 */
	struct mutex lock;
	struct delayed_work work;
};

static void lsm303cmp_disable(struct lsm303cmp_driver_data *drv_data)
{
	drv_data->enabled = 0;
	cancel_delayed_work(&drv_data->work);
	i2c_smbus_write_byte_data(drv_data->client, LSM303CMP_REG_MODE,
		LSM303CMP_MODE_SLEEP);
}

static void lsm303cmp_enable(struct lsm303cmp_driver_data *drv_data)
{
	drv_data->enabled = 1;
	i2c_smbus_write_byte_data(drv_data->client, LSM303CMP_REG_MODE,
		LSM303CMP_MODE_CONT);
	schedule_delayed_work(&drv_data->work,
		msecs_to_jiffies(drv_data->poll_ms));
}

/**
 * lsm303cmp_attr_get_poll - get lsm303cmp poll delay.
 * @dev: lsm303cmp device structure.
 * @attr: lsm303cmp sysfs attribute structure.
 * @buf: return lsm303cmp poll delay ms.
 *
 * Returns buf size write to userspace.
 */
static ssize_t lsm303cmp_attr_get_poll(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct lsm303cmp_driver_data *drv_data;
	int ret;

	drv_data = (struct lsm303cmp_driver_data *)dev_get_drvdata(dev);
	mutex_lock(&drv_data->lock);
	ret = sprintf(buf, "%d\n", drv_data->poll_ms);
	mutex_unlock(&drv_data->lock);
	return ret;
}

/**
 * lsm303cmp_attr_set_poll - set lsm303cmp poll delay.
 * @dev: lsm303cmp device structure.
 * @attr: lsm303cmp sysfs attribute structure.
 * @buf: lsm303cmp poll delay ms.
 * @size: buf size.
 *
 * Returns buf size read from userspace on success.
 * or an error code.
 */
static ssize_t lsm303cmp_attr_set_poll(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct lsm303cmp_driver_data *drv_data;
	unsigned long poll_ms;

	drv_data = dev_get_drvdata(dev);
	if (strict_strtoul(buf, 10, &poll_ms))
		return -EINVAL;
	mutex_lock(&drv_data->lock);
	drv_data->poll_ms = poll_ms * LSM303CMP_MAX_DOR < 1000 ?
		1000/LSM303CMP_MAX_DOR + 5 : poll_ms;
	mutex_unlock(&drv_data->lock);

	return size;
}

static DEVICE_ATTR(poll, S_IRUGO | S_IWUSR,
		lsm303cmp_attr_get_poll, lsm303cmp_attr_set_poll);

/**
 * lsm303cmp_attr_get_enable - get lsm303cmp enable state.
 * @dev: lsm303cmp device structure.
 * @attr: lsm303cmp sysfs attribute structure.
 * @buf: return lsm303cmp enable state.
 *
 * Returns buf size write to userspace.
 */
static ssize_t lsm303cmp_attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lsm303cmp_driver_data *drv_data;
	int ret;

	drv_data = dev_get_drvdata(dev);
	mutex_lock(&drv_data->lock);
	ret = sprintf(buf, "%d\n", drv_data->enabled);
	mutex_unlock(&drv_data->lock);
	return ret;
}

/**
 * lsm303cmp_attr_set_enable - set lsm303cmp enable state.
 * @dev: lsm303cmp device structure.
 * @attr: lsm303cmp sysfs attribute structure.
 * @buf: lsm303cmp enable state. Only value 0 and 1 are valid inputs.
 * @size: buf size.
 *
 * Returns buf size read from userspace on success.
 * or an error code.
 */
static ssize_t lsm303cmp_attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lsm303cmp_driver_data *drv_data;
	unsigned long enable;

	drv_data = dev_get_drvdata(dev);
	if (strict_strtoul(buf, 10, &enable))
		return -EINVAL;
	mutex_lock(&drv_data->lock);
	if (enable == 0 && drv_data->enabled == 1)
		lsm303cmp_disable(drv_data);
	if (enable == 1 && drv_data->enabled == 0)
		lsm303cmp_enable(drv_data);
	mutex_unlock(&drv_data->lock);

	return size;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		lsm303cmp_attr_get_enable, lsm303cmp_attr_set_enable);

static struct attribute *lsm303cmp_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_poll.attr,
	NULL,
};

static struct attribute_group lsm303cmp_attr_group = {
	.name = DRIVER_NAME,
	.attrs = lsm303cmp_attributes,
};

static void lsm303cmp_report_data(struct lsm303cmp_driver_data *drv_data,
	s16 *xyz_data)
{
	input_report_rel(drv_data->input_device, REL_X, xyz_data[0]);
	input_report_rel(drv_data->input_device, REL_Y, xyz_data[2]);
	input_report_rel(drv_data->input_device, REL_Z, xyz_data[1]);
	input_sync(drv_data->input_device);
}

static void lsm303cmp_work(struct work_struct *work)
{
	s16 xyz_data[3] = { 0 };
	struct lsm303cmp_driver_data *drv_data;
	struct i2c_client *client;
	int i;

	drv_data = container_of((struct delayed_work *)work,
				struct lsm303cmp_driver_data, work);
	client = drv_data->client;
	mutex_lock(&drv_data->lock);

	/* This work might be canceld already, check enabled staues first */
	if (drv_data->enabled == 0) {
		mutex_unlock(&drv_data->lock);
		return;
	}
	i2c_smbus_read_i2c_block_data(client,
		LSM303CMP_REG_OUT_X | LSM303CMP_BLOCK_READ, 6, (u8 *)xyz_data);

	for (i = 0; i < 3; i++)
		xyz_data[i] = be16_to_cpu(xyz_data[i]);
	lsm303cmp_report_data(drv_data, xyz_data);

	schedule_delayed_work(&drv_data->work,
		msecs_to_jiffies(drv_data->poll_ms));
	mutex_unlock(&drv_data->lock);
}

static int lsm303cmp_init_device(struct lsm303cmp_driver_data *lsm303cmp)
{
	int res;
	struct i2c_client *client;

	client = lsm303cmp->client;
	res = i2c_smbus_write_byte_data(client, LSM303CMP_REG_CRA,
		LSM303CMP_DOR_75);
	if (res != 0)
		return res;
	res = i2c_smbus_write_byte_data(client, LSM303CMP_REG_CRB,
		LSM303CMP_GAIN_600);
	if (res != 0)
		return res;
	res = i2c_smbus_write_byte_data(client, LSM303CMP_REG_MODE,
		LSM303CMP_MODE_SLEEP);
	if (res != 0)
		return res;
	return 0;
}

static int lsm303cmp_init_input(struct lsm303cmp_driver_data *lsm303cmp)
{
	int res;
	struct input_dev *input_device;
	struct i2c_client *client;

	client = lsm303cmp->client;
	input_device = input_allocate_device();
	if (!input_device) {
		dev_err(&client->dev, "Failed on allocate input device\n");
		return -ENOMEM;
	}
	input_device->name = DRIVER_NAME;
	input_device->id.bustype = BUS_I2C;

	/*
	 * Driver use EV_REL event to report data to user space
	 * instead of EV_ABS. Because EV_ABS event will be ignored
	 * if current input has same value as former one. which effect
	 * data smooth
	 */
	set_bit(EV_REL, input_device->evbit);
	set_bit(REL_X, input_device->relbit);
	set_bit(REL_Y, input_device->relbit);
	set_bit(REL_Z, input_device->relbit);

	res = input_register_device(input_device);
	if (res < 0) {
		dev_err(&client->dev, "Failed on resigter input device\n");
		input_free_device(input_device);
		return res;
	}
	lsm303cmp->input_device = input_device;
	return 0;
}

static int lsm303cmp_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	int res;
	struct lsm303cmp_driver_data *lsm303cmp;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "i2c_check_functionality error\n");
		return -EINVAL;
	}

	lsm303cmp = kzalloc(sizeof(struct lsm303cmp_driver_data),
		GFP_KERNEL);
	if (!lsm303cmp) {
		dev_err(&client->dev, "Failed on allocate memory\n");
		return -ENOMEM;
	}

	dev_set_drvdata(&client->dev, lsm303cmp);
	lsm303cmp->client = client;
	lsm303cmp->enabled = 0;
	lsm303cmp->poll_ms = LSM303CMP_DEFAULT_POLL_MS;
	mutex_init(&lsm303cmp->lock);
	INIT_DELAYED_WORK(&lsm303cmp->work, lsm303cmp_work);

	res = lsm303cmp_init_device(lsm303cmp);
	if (res != 0) {
		dev_err(&client->dev, "Failed on init device\n");
		goto error_init_device;
	}

	res = lsm303cmp_init_input(lsm303cmp);
	if (res < 0)
		goto error_input_device;

	res = sysfs_create_group(&client->dev.kobj, &lsm303cmp_attr_group);
	if (res != 0) {
		dev_err(&client->dev, "Failed on create sysfs\n");
		goto error_sysfs;
	}

	return res;

error_sysfs:
	input_unregister_device(lsm303cmp->input_device);
error_input_device:
error_init_device:
	kfree(lsm303cmp);
	return res;
}

static int lsm303cmp_remove(struct i2c_client *client)
{
	struct lsm303cmp_driver_data *lsm303cmp;

	lsm303cmp = i2c_get_clientdata(client);
	mutex_lock(&lsm303cmp->lock);
	lsm303cmp_disable(lsm303cmp);
	sysfs_remove_group(&client->dev.kobj, &lsm303cmp_attr_group);
	input_unregister_device(lsm303cmp->input_device);

	/*
	 * At this point there might be a last work function running,
	 * wait for it finish before we free lsm303cmp data structure. Here
	 * using cancel_delayed_work_sync will not cause any data race
	 * issue because all device interface are deregistered.
	 */
	mutex_unlock(&lsm303cmp->lock);
	cancel_delayed_work_sync(&lsm303cmp->work);

	kfree(lsm303cmp);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lsm303cmp_resume(struct device *dev)
{
	struct lsm303cmp_driver_data *drv_data = dev_get_drvdata(dev);

	mutex_lock(&drv_data->lock);
	if (drv_data->need_resume)
		lsm303cmp_enable(drv_data);
	mutex_unlock(&drv_data->lock);

	return 0;
}

static int lsm303cmp_suspend(struct device *dev)
{
	struct lsm303cmp_driver_data *drv_data = dev_get_drvdata(dev);

	mutex_lock(&drv_data->lock);
	drv_data->need_resume = drv_data->enabled;
	if (drv_data->enabled)
		lsm303cmp_disable(drv_data);
	mutex_unlock(&drv_data->lock);

	return 0;
}

static const struct dev_pm_ops lsm303cmp_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lsm303cmp_suspend,
			lsm303cmp_resume)
};
#endif /* CONFIG_PM_SLEEP */

static struct i2c_device_id lsm303cmp_id[2] = {
	{ DRIVER_NAME, 0 },
	{ },
};

static struct i2c_driver lsm303cmp_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name	= DRIVER_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &lsm303cmp_pm_ops,
#endif /* CONFIG_PM_SLEEP */
	},
	.id_table = lsm303cmp_id,
	.probe = lsm303cmp_probe,
	.remove = lsm303cmp_remove,
};

static int __init lsm303cmp_init(void)
{
	return i2c_add_driver(&lsm303cmp_driver);
}

static void __exit lsm303cmp_exit(void)
{
	i2c_del_driver(&lsm303cmp_driver);
}

module_init(lsm303cmp_init);
module_exit(lsm303cmp_exit);

MODULE_AUTHOR("Li Jian<jian.d.li@intel.com>");
MODULE_DESCRIPTION("lsm303dlhc Compass Driver");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("i2c:lsm303cmp");
