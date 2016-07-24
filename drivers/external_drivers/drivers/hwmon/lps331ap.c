/*
 * lps331ap.c - ST LPS331AP Pressure Sensor Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 */

#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>

#define REF_P_XL	0x08
#define REF_P_L		0x09
#define REF_P_H		0x0A
#define REF_T_L		0x0B
#define REF_T_H		0x0C
#define WHO_AM_I	0x0F

#define RES_CONF	0x10

#define CTRL_REG1	0x20
#define POWER_MODE	7
#define POWER_DOWN	(0 << POWER_MODE)
#define POWER_ON	(1 << POWER_MODE)
#define ODR_SELECT	4
#define ODR0		(0 << ODR_SELECT)
#define ODR1		(1 << ODR_SELECT)
#define ODR2		(2 << ODR_SELECT)
#define ODR3		(3 << ODR_SELECT)
#define ODR4		(4 << ODR_SELECT)
#define ODR5		(5 << ODR_SELECT)
#define ODR6		(6 << ODR_SELECT)
#define ODR7		(7 << ODR_SELECT)
#define DIFF_EN		3
#define INT_CIRCUIT_ENABLE	(1 << DIFF_EN)
#define INT_CIRCUIT_DISABLE	(0 << DIFF_EN)
#define BDU			2
#define CONT_DATA_UPDATE	(0 << BDU)
#define BLOCK_DATA_UPDATE	(1 << BDU)
#define DELTA_EN	1
#define DELTA_P_ENABLE		(1 << DELTA_EN)
#define DELTA_P_DISABLE		(0 << DELTA_EN)
#define SIM_SELECT	0
#define WIRE_INTER_4		(0 << SIM_SELECT)
#define WIRE_INTER_3		(1 << SIM_SELECT)

#define CTRL_REG2	0x21
#define BOOT		0x80
#define SWRESET		0x04
#define AUTO_ZERO	0x02
#define ONE_SHOT	0x01

#define CTRL_REG3	0x22
#define INT_H_L		7
#define INT_ACTIVE_H	(0 << INT_H_L)
#define INT_ACTIVE_L	(1 << INT_H_L)
#define PP_OD		6
#define PUSH_PULL		(0 << PP_OD)
#define OPEN_DRAIN		(1 << PP_OD)
#define INT2_CTRL	3
#define INT2_P_HIGH		(1 << INT2_CTRL)
#define INT2_P_LOW		(2 << INT2_CTRL)
#define INT2_P_H_L		(3 << INT2_CTRL)
#define INT2_DATARDY	(4 << INT2_CTRL)
#define INT1_CTRL	0
#define INT1_P_HIGH		(1 << INT1_CTRL)
#define INT1_P_LOW		(2 << INT1_CTRL)
#define INT1_P_H_L		(3 << INT1_CTRL)
#define INT1_DATARDY	(4 << INT1_CTRL)


#define INT_CFG_REG	0x23
#define INT_CFG_LIR	0x04
#define INT_CFG_PL_E	0x02
#define INT_CFG_PH_E	0x01

#define INT_SRC_REG	0x24
#define INT_SRC_IA	0x04
#define INT_SRC_PL	0x02
#define INT_SRC_PH	0x01

#define THS_P_LOW_REG	0x25
#define THS_P_HIGH_REG	0x26

#define STATUS_REG	0x27
#define STATUS_P_DA	0x02
#define STATUS_T_DA	0x01
#define STATUS_P_OR	0x20
#define STATUS_T_OR	0x10

#define I2C_AUTO_INC	0x80
#define PRESS_OUT_XL	0x28
#define PRESS_OUT_L	0x29
#define PRESS_OUT_H	0x2A

#define TEMP_OUT_L	0x2B
#define TEMP_OUT_H	0x2C

#define AMP_CTRL	0x30

#define LPS331AP_DEFAULT_DELAY	200
#define LPS331AP_MIN_DELAY	75

#define LPS331AP_DEVICE_NAME "lps331ap"
#define LPS331AP_DEVICE_ID	0xbb

struct lps331ap_data {
	struct i2c_client *client;
	/*
	 * This mutex protect lps331ap from data race condition.
	 * Any function need to conmmunicate with lps331ap device
	 * or to set lps331ap delay_ms and enabled state should acquire
	 * this lock first.
	 */
	struct mutex lock;

	struct delayed_work input_work;

	struct input_dev *input_dev;

	int delay_ms;
	int enabled;
	int need_resume;
};

static int lps331ap_read(struct lps331ap_data *lps331ap, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(lps331ap->client, reg);
	if (ret >= 0) {
		*val = ret;
		ret = 0;
	}

	return ret;
}

static int lps331ap_write(struct lps331ap_data *lps331ap, u8 reg, u8 val)
{
	return i2c_smbus_write_byte_data(lps331ap->client, reg, val);
}

static void lps331ap_get_data(struct lps331ap_data *lps331ap,
		s32 *data)
{
	int err;
	u8 buf[3] = { 0 };

	err = i2c_smbus_read_i2c_block_data(lps331ap->client,
			PRESS_OUT_XL | I2C_AUTO_INC, 3, buf);
	if (err < 0)
		return;

	data[0] = (s8)buf[2] << 16 | buf[1] << 8 | buf[0];

	err = i2c_smbus_read_i2c_block_data(lps331ap->client,
			TEMP_OUT_L | I2C_AUTO_INC, 2, buf);
	if (err < 0)
		return;

	data[1] = (s8)buf[1] << 8 | buf[0];
}

static void lps331ap_report_values(struct lps331ap_data *lps331ap,
		s32 *data)
{
	input_report_rel(lps331ap->input_dev, REL_X, (int)data[0]);
	input_report_rel(lps331ap->input_dev, REL_Y, (int)data[1]);

	input_sync(lps331ap->input_dev);
}

static int lps331ap_initchip(struct lps331ap_data *lps331ap)
{
	int ret;
	u8 val;

	ret = lps331ap_read(lps331ap, WHO_AM_I, &val);
	if (ret < 0)
		return ret;

	if (LPS331AP_DEVICE_ID != val) {
		dev_err(&lps331ap->client->dev, "device id not match\n");
		return -ENODEV;
	}

	ret = lps331ap_write(lps331ap, RES_CONF, 0x34);
	if (ret < 0)
		return ret;

	ret = lps331ap_write(lps331ap, CTRL_REG1, POWER_DOWN);
	if (ret < 0)
		return ret;

	ret = lps331ap_write(lps331ap, CTRL_REG1,
			POWER_DOWN | ODR7 | INT_CIRCUIT_DISABLE
			| BLOCK_DATA_UPDATE	| DELTA_P_DISABLE);
	return ret;
}

static void lps331ap_power_off(struct lps331ap_data *lps331ap)
{
	int err;
	u8 val;

	err = lps331ap_read(lps331ap, CTRL_REG1, &val);
	if (err < 0) {
		dev_err(&lps331ap->client->dev, "power off failed: %d\n", err);
		return;
	}

	val &= ~POWER_ON;
	err = lps331ap_write(lps331ap, CTRL_REG1, val);
	if (err < 0)
		dev_err(&lps331ap->client->dev, "power off failed: %d\n", err);
}

static int lps331ap_power_on(struct lps331ap_data *lps331ap)
{
	int err;
	u8 val;

	err = lps331ap_read(lps331ap, CTRL_REG1, &val);
	if (err < 0) {
		dev_err(&lps331ap->client->dev, "power on failed: %d\n", err);
		return err;
	}

	val |= POWER_ON;
	err = lps331ap_write(lps331ap, CTRL_REG1, val);
	if (err < 0)
		dev_err(&lps331ap->client->dev, "power on failed: %d\n", err);

	return err;
}

static void lps331ap_enable(struct lps331ap_data *lps331ap)
{
	lps331ap->enabled = 1;
	lps331ap_power_on(lps331ap);
	schedule_delayed_work(&lps331ap->input_work,
			msecs_to_jiffies(lps331ap->delay_ms));
}

static void lps331ap_disable(struct lps331ap_data *lps331ap)
{
	lps331ap->enabled = 0;
	cancel_delayed_work(&lps331ap->input_work);
	lps331ap_power_off(lps331ap);
}

static void lps331ap_input_work_func(struct work_struct *work)
{
	struct lps331ap_data *lps331ap;
	s32 pt[2] = { 0 };

	lps331ap = container_of((struct delayed_work *)work,
			struct lps331ap_data, input_work);

	mutex_lock(&lps331ap->lock);

	if (!lps331ap->enabled)
		goto out;

	lps331ap_get_data(lps331ap, pt);
	lps331ap_report_values(lps331ap, pt);

	schedule_delayed_work(&lps331ap->input_work,
			msecs_to_jiffies(lps331ap->delay_ms));
out:
	mutex_unlock(&lps331ap->lock);
}

static int lps331ap_input_init(struct lps331ap_data *lps331ap)
{
	int err;
	struct input_dev *input;

	INIT_DELAYED_WORK(&lps331ap->input_work, lps331ap_input_work_func);
	input = input_allocate_device();
	if (!input) {
		dev_err(&lps331ap->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	input->name = "lps331ap_pressure";
	input->id.bustype = BUS_I2C;
	input->dev.parent = &lps331ap->client->dev;
	input_set_drvdata(input, lps331ap);

	set_bit(EV_REL, input->evbit);
	set_bit(REL_X, input->relbit);
	set_bit(REL_Y, input->relbit);

	err = input_register_device(input);
	if (err) {
		dev_err(&lps331ap->client->dev,
				"unable to register input device %s: %d\n",
				input->name, err);
		goto err1;
	}

	lps331ap->input_dev = input;
	return 0;
err1:
	input_free_device(input);
	return err;
}

/* sysfs */
static ssize_t lps331ap_delay_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lps331ap_data *lps331ap = i2c_get_clientdata(client);
	int delay_ms;

	mutex_lock(&lps331ap->lock);
	delay_ms = lps331ap->delay_ms;
	mutex_unlock(&lps331ap->lock);

	return sprintf(buf, "%d\n", delay_ms);
}

static ssize_t lps331ap_delay_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long polltime;
	struct i2c_client *client = to_i2c_client(dev);
	struct lps331ap_data *lps331ap = i2c_get_clientdata(client);

	if (strict_strtoul(buf, 10, &polltime))
		return -EINVAL;

	polltime = (polltime > LPS331AP_MIN_DELAY) ?
			polltime : LPS331AP_MIN_DELAY;

	mutex_lock(&lps331ap->lock);
	lps331ap->delay_ms = polltime;
	mutex_unlock(&lps331ap->lock);

	return count;
}

static ssize_t lps331ap_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lps331ap_data *lps331ap = i2c_get_clientdata(client);
	int enabled;

	mutex_lock(&lps331ap->lock);
	enabled = lps331ap->enabled;
	mutex_unlock(&lps331ap->lock);

	return sprintf(buf, "%d\n", lps331ap->enabled);
}

#define LPS331AP_SYSFS_POWERON		1
#define LPS331AP_SYSFS_POWERDOWN	0

static ssize_t lps331ap_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lps331ap_data *lps331ap = i2c_get_clientdata(client);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val != LPS331AP_SYSFS_POWERON && val != LPS331AP_SYSFS_POWERDOWN)
		return -EINVAL;

	mutex_lock(&lps331ap->lock);
	if (val && !lps331ap->enabled)
		lps331ap_enable(lps331ap);
	else if (!val && lps331ap->enabled)
		lps331ap_disable(lps331ap);
	else
		dev_dbg(&client->dev, "invalid power switch\n");

	mutex_unlock(&lps331ap->lock);

	return count;
}

static DEVICE_ATTR(poll, S_IRUGO|S_IWUSR, lps331ap_delay_show,
		lps331ap_delay_store);
static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, lps331ap_enable_show,
		lps331ap_enable_store);

static struct attribute *lps331ap_attributes[] = {
	&dev_attr_poll.attr,
	&dev_attr_enable.attr,
	NULL
};

static struct attribute_group lps331ap_attribute_group = {
	.attrs = lps331ap_attributes
};

static int lps331ap_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	int err;
	struct lps331ap_data *lps331ap;

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	lps331ap = kzalloc(sizeof(struct lps331ap_data), GFP_KERNEL);
	if (!lps331ap) {
		dev_err(&client->dev,
				"failed to allocate memory data for mudule data\n");
		return -ENOMEM;
	}

	mutex_init(&lps331ap->lock);
	lps331ap->client = client;
	i2c_set_clientdata(client, lps331ap);

	lps331ap->delay_ms = LPS331AP_DEFAULT_DELAY;

	err = lps331ap_initchip(lps331ap);
	if (err < 0) {
		dev_err(&client->dev,
				"lps331ap_initchip failed with %d\n", err);
		goto fail_init;
	}

	err = lps331ap_input_init(lps331ap);
	if (err < 0) {
		dev_err(&client->dev, "input init error\n");
		goto err_input_init;
	}

	lps331ap->enabled = 0;

	err = sysfs_create_group(&client->dev.kobj, &lps331ap_attribute_group);
	if (err) {
		dev_err(&client->dev, "sysfs can not create group\n");
		goto err_sysfs;
	}

	return 0;

err_sysfs:
	input_unregister_device(lps331ap->input_dev);
err_input_init:
fail_init:
	kfree(lps331ap);
	return err;
}

static int lps331ap_remove(struct i2c_client *client)
{
	struct lps331ap_data *lps331ap = i2c_get_clientdata(client);

	sysfs_remove_group(&client->dev.kobj, &lps331ap_attribute_group);
	cancel_delayed_work_sync(&lps331ap->input_work);
	input_unregister_device(lps331ap->input_dev);
	lps331ap_power_off(lps331ap);
	kfree(lps331ap);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lps331ap_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lps331ap_data *lps331ap = i2c_get_clientdata(client);

	mutex_lock(&lps331ap->lock);
	lps331ap->need_resume = lps331ap->enabled;
	if (lps331ap->enabled)
		lps331ap_disable(lps331ap);
	mutex_unlock(&lps331ap->lock);

	return 0;
}

static int lps331ap_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct lps331ap_data *lps331ap = i2c_get_clientdata(client);

	mutex_lock(&lps331ap->lock);
	if (lps331ap->need_resume)
		lps331ap_enable(lps331ap);
	mutex_unlock(&lps331ap->lock);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static const struct dev_pm_ops lps331ap_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lps331ap_suspend,
			lps331ap_resume)
};

static const struct i2c_device_id lps331ap_id[] = {
	{LPS331AP_DEVICE_NAME, 0},
	{},
};

static struct i2c_driver lps331ap_driver = {
	.driver = {
		.name = LPS331AP_DEVICE_NAME,
		.owner = THIS_MODULE,
		.pm = &lps331ap_pm_ops,
	},
	.probe = lps331ap_probe,
	.remove = lps331ap_remove,
	.id_table = lps331ap_id,
};

static int __init lps331ap_init(void)
{
	return i2c_add_driver(&lps331ap_driver);
}

static void __exit lps331ap_exit(void)
{
	i2c_del_driver(&lps331ap_driver);
}

module_init(lps331ap_init);
module_exit(lps331ap_exit);

MODULE_DESCRIPTION("lps331ap pressure sensor driver");
MODULE_AUTHOR("Xun Wang <xun.a.wang@intel.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:lps331ap");
