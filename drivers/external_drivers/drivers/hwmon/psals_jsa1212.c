/*
 * JSA01212 Sensor Driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
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
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/workqueue.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/acpi_gpio.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <asm/div64.h>

/*i2c registers*/
#define REG_CONFIGURE		0x01
#define REG_INTERRUPT		0x02
#define REG_PROX_LT		0x03
#define REG_PROX_HT		0x04
#define REG_ALSIR_TH1		0x05
#define REG_ALSIR_TH2		0x06
#define REG_ALSIR_TH3		0x07
#define REG_PROX_DATA		0x08
#define REG_ALSIR_DT1		0x09
#define REG_ALSIR_DT2		0x0a
#define REG_ALS_RNG		0x0b

#define PS_FLAG			0x80
#define ALS_FLAG		0x08

/*INT_GPIO*/
#define	INT_GPIO 		136

#define DRIVER_NAME 		"JSA01212:00"
#define PS_INPUT_NAME		"jsa1212_ps"
#define ALS_INPUT_NAME		"jsa1212_als"

#define CONFIG_JSA1212_DEBUG
#ifdef CONFIG_JSA1212_DEBUG
static unsigned int debug_level = 0;
#define DBG_LEVEL1		1
#define DBG_LEVEL2		2
#define DBG_LEVEL3		3
#define DBG_LEVEL4		4
#define SENSOR_DBG(level, fmt, ...)			\
do {							\
	if (level <= debug_level)			\
		printk(KERN_DEBUG "<jsa1212>[%d]%s  "	\
			 fmt "\n",			\
			__LINE__, __func__,		\
			##__VA_ARGS__);			\
} while (0)

#else
#define SENSOR_DBG(level, ...)
#endif

/*Private data for each sensor*/
struct sensor_data {
	struct i2c_client *client;
	struct input_dev *input_ps;
	struct input_dev *input_als;
	struct mutex lock;
	/*used to detect ps far event*/
	struct delayed_work work_ps;
#define PS_WORK_INTERVAL		500
	int interval;
#define PS_NEAR_FAR_THRESHOLD		0xc8
	int ps_threshold;
	int gpio_int;
	int irq;

#define PS_DISABLE	(0<<0)
#define PS_ENABLE	(1<<0)
#define PS_STATE_FLAG	(1<<0)
#define ALS_DISABLE	(0<<1)
#define ALS_ENABLE	(1<<1)
#define ALS_STATE_FLAG	(1<<1)
	int state;
	int state_suspend;

}jsa1212_data;

static int sensor_read(struct sensor_data *data, u8 reg, u8 len, u8 *val)
{
	int ret;

	if (len > 1) {
		ret = i2c_smbus_read_i2c_block_data(data->client,
						reg, len, val);
		if (ret < 0)
			dev_err(&data->client->dev, "Err: read reg %02x=%02x\n",
							reg, ret);
		return ret;
	} else {

		ret = i2c_smbus_read_byte_data(data->client, reg);
		if (ret >= 0) {
			SENSOR_DBG(DBG_LEVEL3, "read i2c reg %02x=%02x",
								reg, ret);
			*val = ret;
			return 0;
		} else {
			dev_err(&data->client->dev,
				"Err: return %d when read i2c reg %02x\n",
				ret, reg);
			return ret;
		}
	}
}

static int sensor_write(struct sensor_data *data, u8 reg, u8 val)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL3, "write i2c reg %02x=%02x", reg, val);

	ret = i2c_smbus_write_byte_data(data->client, reg, val);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Err: return %d when write i2c reg %02x=%02x\n",
			ret, reg, val);
	}
	return ret;
}

#define PS_ENABLE_FLAG		(1 << 7)
#define PS_DISABLE_FLAG		(0 << 7)
#define PULSES_INTERVAL_200MS	(0b010 << 4)
#define PULSES_INTERVAL_100MS	(0b011 << 4)
#define PULSES_INTERVAL_75MS	(0b100 << 4)
#define PULSED_CURRENT_100MA	(0 << 3)
#define PULSED_CURRENT_200MA	(1 << 3)
#define ALS_ENABLE_FLAG		(1 << 2)
#define ALS_DISABLE_FLAG	(0 << 2)

#define PXS_CONVERSION_4	(0b01 << 1)
#define PXS_CONVERSION_8	(0b10 << 1)
#define PXS_CONVERSION_16	(0b11 << 1)
#define ALS_CONVERSION_4	(0b01 << 5)
#define ALS_CONVERSION_8	(0b10 << 5)
#define ALS_CONVERSION_16	(0b11 << 5)

#define ALS_LUX_2048		(0b000 << 0) /* 0.50 lux/count */
#define ALS_LUX_1024		(0b001 << 0) /* 0.25 lux/count */
static int sensor_init(struct sensor_data *data)
{
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	ret = sensor_write(data, REG_CONFIGURE,
		PS_DISABLE_FLAG | PULSES_INTERVAL_100MS |
		PULSED_CURRENT_100MA | ALS_DISABLE_FLAG);
	if (ret < 0)
		return ret;

	ret = sensor_write(data, REG_INTERRUPT, PXS_CONVERSION_4|ALS_CONVERSION_4);
	if (ret < 0)
		return ret;

	return sensor_write(data, REG_ALS_RNG, ALS_LUX_2048);
}

static int sensor_set_mode(struct sensor_data *data, int state)
{
	int ret = 0;
	u8 val;

	SENSOR_DBG(DBG_LEVEL2, "%s change state from %08x to %08x",
			data->client->name, data->state, state);

	if ((data->state & PS_STATE_FLAG) != (state & PS_STATE_FLAG)) {
		if ((state & PS_STATE_FLAG) == PS_ENABLE) {
			SENSOR_DBG(DBG_LEVEL3, "enable ps");

			//ret = sensor_write(data, REG_PROX_LT, 0x64);
			ret = sensor_write(data, REG_PROX_LT,
						data->ps_threshold);
			if (ret < 0)
				goto out;
			//ret = sensor_write(data, REG_PROX_HT, 0x08);
			ret = sensor_write(data, REG_PROX_HT,
						data->ps_threshold);
			if (ret < 0)
				goto out;

			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			ret = sensor_write(data, REG_CONFIGURE, val | 0x80);
			if (ret < 0)
				goto out;

			data->state |= PS_ENABLE;
		} else {
			SENSOR_DBG(DBG_LEVEL3, "disable ps");

			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			ret = sensor_write(data, REG_CONFIGURE, val & 0x7f);
			if (ret < 0)
				goto out;

			data->state &= (~PS_ENABLE);
		}
	}

	if ((data->state & ALS_STATE_FLAG) != (state & ALS_STATE_FLAG)) {
		if ((state & ALS_STATE_FLAG) == ALS_ENABLE) {

			SENSOR_DBG(DBG_LEVEL3, "enable als");

			ret = sensor_write(data, REG_ALSIR_TH1, 0x64);
			if (ret < 0)
				goto out;
			ret = sensor_write(data, REG_ALSIR_TH2, 0x08);
			if (ret < 0)
				goto out;
			ret = sensor_write(data, REG_ALSIR_TH3, 0x3e);
			if (ret < 0)
				goto out;

			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			ret = sensor_write(data, REG_CONFIGURE, val | 0x04);
			if (ret < 0)
				goto out;

			data->state |= ALS_ENABLE;
		} else {
			SENSOR_DBG(DBG_LEVEL3, "disable als");

			ret = sensor_read(data, REG_CONFIGURE, 1, &val);
			if (ret < 0)
				goto out;
			ret = sensor_write(data, REG_CONFIGURE, val & 0xfb);
			if (ret < 0)
				goto out;

			data->state &= (~ALS_ENABLE);
		}
	}

out:
	return ret;
}

static int sensor_suspend(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	disable_irq(data->irq);
	mutex_lock(&data->lock);
	data->state_suspend = data->state;
	sensor_set_mode(data, PS_DISABLE | ALS_DISABLE);
	mutex_unlock(&data->lock);
	return 0;
}

static int sensor_resume(struct device *dev)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	enable_irq(data->irq);
	mutex_lock(&data->lock);
	sensor_set_mode(data, data->state_suspend);
	mutex_unlock(&data->lock);
	return 0;
}

static void sensor_poll_work(struct work_struct *work)
{
	int ret;
	int delay;
	u8 val;
	struct sensor_data *data = container_of(to_delayed_work(work),
					struct sensor_data, work_ps);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	if ((data->state & PS_STATE_FLAG) != PS_ENABLE)
		goto out;

	/*exit when get far event*/
	ret = sensor_read(data, REG_PROX_DATA, 1, &val);
	if (ret < 0)
		goto out;
	if (val <= data->ps_threshold) {
		SENSOR_DBG(DBG_LEVEL2, "PS far %02x", val);

		input_report_abs(data->input_ps, ABS_X, 1);
		input_sync(data->input_ps);

		sensor_write(data, REG_PROX_HT, data->ps_threshold);
		goto out;
	}

	delay = msecs_to_jiffies(data->interval);
	schedule_delayed_work(&data->work_ps, delay);
out:
	mutex_unlock(&data->lock);
}

static irqreturn_t sensor_interrupt_handler(int irq, void *pri)
{
	int ret;
	u8 status;
	struct sensor_data *data = (struct sensor_data *)pri;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	ret = sensor_read(data, REG_INTERRUPT, 1, &status);
	if (ret < 0)
		goto out;

	if (status & PS_FLAG) {
		u8 val;
		int delay;

		SENSOR_DBG(DBG_LEVEL3, "PS irq:%02x", status);

		ret = sensor_read(data, REG_PROX_DATA, 1, &val);
		if (ret < 0)
			goto out;
		if (val > data->ps_threshold) {
			SENSOR_DBG(DBG_LEVEL3, "PS near:%02x", val);

			input_report_abs(data->input_ps, ABS_X, 0);
			input_sync(data->input_ps);

			/*change threshold to avoid interrupt and
			schedule delaywork to poll near event*/
			ret = sensor_write(data, REG_PROX_HT, 0xff);
			if (ret < 0)
				goto out;

			delay = msecs_to_jiffies(data->interval);
			schedule_delayed_work(&data->work_ps, delay);
		} else {
			SENSOR_DBG(DBG_LEVEL3, "Why here PS far:%02x", val);

			input_report_abs(data->input_ps, ABS_X, 1);
			input_sync(data->input_ps);
			sensor_write(data, REG_PROX_HT, data->ps_threshold);
		}
	}

	if (status & ALS_FLAG) {
		u8 raw[2];
		int val, low, high;

		SENSOR_DBG(DBG_LEVEL3, "ALS irq:%02x", status);

		ret = sensor_read(data, REG_ALSIR_DT1, 2, raw);
		if (ret < 0)
			goto out;
		val = raw[0] + ((raw[1] & 0xf) << 8);

		low = val * 98 /100;
		high = val * 102 /100;
		if (high == val)
			high += 1;

		ret = sensor_write(data, REG_ALSIR_TH1, low & 0xff);
		if (ret < 0)
			goto out;
		ret = sensor_write(data, REG_ALSIR_TH2,
				((low >> 8) & 0xf) | ((high & 0xf) << 4));
		if (ret < 0)
			goto out;
		ret = sensor_write(data, REG_ALSIR_TH3, high >> 4);
		if (ret < 0)
			goto out;

		SENSOR_DBG(DBG_LEVEL2, "ALS data:%08x", val);
		input_report_abs(data->input_als, ABS_X, val);
		input_sync(data->input_als);

	}

	/*clear both ps and als flag bit7, bit3*/
	ret = sensor_write(data, REG_INTERRUPT, status & 0x77);
	if (ret < 0) {
		dev_err(&data->client->dev, "sensor_write returns %d\n", ret);
	}
out:
	mutex_unlock(&data->lock);
	return IRQ_HANDLED;
}

static int sensor_get_data_init(struct sensor_data *data)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	ret = gpio_request(data->gpio_int, DRIVER_NAME);
	if (ret < 0) {
		dev_err(&data->client->dev, "Err: request gpio %d\n",
						data->gpio_int);
		goto out;
	}

	gpio_direction_input(data->gpio_int);
	data->irq = gpio_to_irq(data->gpio_int);
	//irq_set_status_flags(data->irq, IRQ_NOAUTOEN);
	ret = request_threaded_irq(data->irq, NULL, sensor_interrupt_handler,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT, DRIVER_NAME, data);
	if (ret < 0) {
		dev_err(&data->client->dev,
			"Fail to request irq:%d ret=%d\n", data->irq, ret);
		gpio_free(data->gpio_int);
		return ret;
	}

	INIT_DELAYED_WORK(&data->work_ps, sensor_poll_work);
out:
	return ret;
}

static int sensor_input_init(struct sensor_data *data)
{
	int ret;
	struct input_dev *input;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	input = input_allocate_device();
	if (!input) {
		dev_err(&data->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}
	input->name = PS_INPUT_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&data->client->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		input_free_device(input);
		return ret;
	}
	data->input_ps = input;

	input = input_allocate_device();
	if (!input) {
		dev_err(&data->client->dev, "input device allocate failed\n");
		ret = -ENOMEM;
		goto err;
	}
	input->name = ALS_INPUT_NAME;
	input->id.bustype = BUS_I2C;
	input->dev.parent = &data->client->dev;
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&data->client->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		input_free_device(input);
		goto err;
	}
	data->input_als = input;
	return 0;
err:
	input_unregister_device(data->input_ps);
	return ret;
}

#define SENSOR_SYSFS_POWERON		1
#define SENSOR_SYSFS_POWERDOWN		0
static ssize_t ps_sensor_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int enabled;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	if (data->state & PS_ENABLE)
		enabled = 1;
	else
		enabled = 0;

	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t ps_sensor_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	unsigned long val;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, " %x", (int)val);

	if (val != SENSOR_SYSFS_POWERON && val != SENSOR_SYSFS_POWERDOWN)
		return -EINVAL;

	mutex_lock(&data->lock);

	if (val) {
		u8 ps;
		int ret;

		sensor_set_mode(data, data->state | PS_ENABLE);
		/*in case of far event can't trigger at the first time*/
		msleep(1);
		ret = sensor_read(data, REG_PROX_DATA, 1, &ps);
		if (ret >= 0) {
			input_report_abs(data->input_ps, ABS_X,
						ps <= data->ps_threshold);
			input_sync(data->input_ps);
		}
	} else {
		int state = data->state & (~PS_ENABLE);
		sensor_set_mode(data, state);
	}

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t als_sensor_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	int enabled;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	mutex_lock(&data->lock);

	if (data->state & ALS_ENABLE)
		enabled = 1;
	else
		enabled = 0;

	mutex_unlock(&data->lock);

	return sprintf(buf, "%d\n", enabled);
}

static ssize_t als_sensor_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	unsigned long val;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, " %x", (int)val);

	if (val != SENSOR_SYSFS_POWERON && val != SENSOR_SYSFS_POWERDOWN)
		return -EINVAL;

	mutex_lock(&data->lock);

	if (val)
		sensor_set_mode(data, data->state | ALS_ENABLE);
	else {
		int state = data->state & (~ALS_ENABLE);
		sensor_set_mode(data, state);
	}

	mutex_unlock(&data->lock);

	return count;
}

static ssize_t ps_sensor_thresh_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	return sprintf(buf, "%d\n", data->ps_threshold);
}

static ssize_t ps_sensor_thresh_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct sensor_data *data = i2c_get_clientdata(client);
	unsigned long val;

	SENSOR_DBG(DBG_LEVEL1, "%s", data->client->name);

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	SENSOR_DBG(DBG_LEVEL1, " %x", (int)val);

	mutex_lock(&data->lock);

	data->ps_threshold = val;
	sensor_write(data, REG_PROX_LT, data->ps_threshold);
	sensor_write(data, REG_PROX_HT, data->ps_threshold);

	mutex_unlock(&data->lock);

	return count;
}

#ifdef CONFIG_JSA1212_DEBUG
static ssize_t sensor_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", debug_level);
}

static ssize_t sensor_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	debug_level = val;
	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, sensor_debug_show,
		sensor_debug_store);
#endif

static DEVICE_ATTR(ps_enable, S_IRUGO|S_IWUSR,
		ps_sensor_enable_show, ps_sensor_enable_store);
static DEVICE_ATTR(als_enable, S_IRUGO|S_IWUSR,
		als_sensor_enable_show, als_sensor_enable_store);
static DEVICE_ATTR(ps_thresh, S_IRUGO|S_IWUSR,
		ps_sensor_thresh_show, ps_sensor_thresh_store);

static struct attribute *sensor_default_attributes[] = {
	&dev_attr_ps_enable.attr,
	&dev_attr_als_enable.attr,
	&dev_attr_ps_thresh.attr,
#ifdef CONFIG_JSA1212_DEBUG
	&dev_attr_debug.attr,
#endif
	NULL
};

static struct attribute_group sensor_default_attribute_group = {
	.attrs = sensor_default_attributes
};

static int sensor_data_init(struct i2c_client *client,
			struct sensor_data *data)
{
	int ret = 0;

	data->client = client;
	mutex_init(&data->lock);
	data->state = 0;
	data->interval = PS_WORK_INTERVAL;
	data->ps_threshold = PS_NEAR_FAR_THRESHOLD;

	data->gpio_int = acpi_get_gpio_by_index(&client->dev, 0, NULL);
	if (data->gpio_int < 0) {
		dev_warn(&client->dev, "Fail to get gpio pin by ACPI\n");
		data->gpio_int = INT_GPIO;
	}
	SENSOR_DBG(DBG_LEVEL3, "gpios:%d", data->gpio_int);

	i2c_set_clientdata(client, data);
	return ret;
}

static int sensor_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	int ret = 0;

	SENSOR_DBG(DBG_LEVEL3, "i2c device:%s", devid->name);

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	sensor_data_init(client, &jsa1212_data);

	ret = sensor_input_init(&jsa1212_data);
	if (ret < 0) {
		dev_err(&client->dev, "input init %d\n", ret);
		goto out;
	}

	ret = sensor_get_data_init(&jsa1212_data);
	if (ret) {
		dev_err(&client->dev, "sensor_get_data_init\n");
		goto get_data;
	}

	ret = sysfs_create_group(&client->dev.kobj,
					&sensor_default_attribute_group);
	if (ret) {
		dev_err(&client->dev, "sysfs create group\n");
		goto sys_init;
	}

	ret = sensor_init(&jsa1212_data);
	if (ret) {
		dev_err(&client->dev, "sensor_init\n");
		goto init;
	}

	return 0;
init:
	sysfs_remove_group(&jsa1212_data.client->dev.kobj,
					&sensor_default_attribute_group);
sys_init:
	free_irq(jsa1212_data.irq, &jsa1212_data);
	gpio_free(jsa1212_data.gpio_int);
get_data:
	input_unregister_device(jsa1212_data.input_als);
	input_unregister_device(jsa1212_data.input_ps);
out:
	return ret;
}

static int sensor_remove(struct i2c_client *client)
{
	struct sensor_data *data = i2c_get_clientdata(client);

	free_irq(data->irq, data);
	sysfs_remove_group(&data->client->dev.kobj,
					&sensor_default_attribute_group);
	gpio_free(data->gpio_int);

	input_unregister_device(data->input_als);
	input_unregister_device(data->input_ps);

	return 0;
}

static const struct i2c_device_id jsa1212_id[] = {
	{DRIVER_NAME, 0},
	{},
};

static const struct dev_pm_ops sensor_pm = {
	SET_SYSTEM_SLEEP_PM_OPS(sensor_suspend, sensor_resume)
};

static struct i2c_driver jsa1212_driver = {
	.probe		= sensor_probe,
	.remove		= sensor_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	= &sensor_pm,
#endif
	},
	.id_table = jsa1212_id,
};

//#define CONFIG_JSA1212_MANUAL_DEVICE
#ifdef CONFIG_JSA1212_MANUAL_DEVICE
static int register_i2c_device(int bus, int addr, char *name)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	struct i2c_board_info info;

	SENSOR_DBG(DBG_LEVEL2, "%s", name);

	memset(&info, 0, sizeof(info));
	strlcpy(info.type, name, I2C_NAME_SIZE);
	adapter = i2c_get_adapter(bus);
	if (!adapter) {
		printk(KERN_ERR "Err: invalid i2c adapter %d\n", bus);
		return -ENODEV;
	} else {
		struct i2c_client *client;
		info.addr = addr;
		client = i2c_new_device(adapter, &info);
		if (!client) {
			printk(KERN_ERR "Fail to add i2c device%d:%d\n",
						bus, addr);
		}
	}
	return ret;
}
#endif

static int __init psals_jsa1212_init(void)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL2, "%s", DRIVER_NAME);

#ifdef CONFIG_JSA1212_MANUAL_DEVICE
	register_i2c_device(3, 0x44, DRIVER_NAME);
#endif

	ret = i2c_add_driver(&jsa1212_driver);
	if (ret < 0)
		printk(KERN_ERR "Fail to register jsa1212 driver\n");

	return ret;
}

static void __exit psals_jsa1212_exit(void)
{
	SENSOR_DBG(DBG_LEVEL2, "%s", DRIVER_NAME);

	i2c_del_driver(&jsa1212_driver);
}

module_init(psals_jsa1212_init);
module_exit(psals_jsa1212_exit);

MODULE_DESCRIPTION("JSA1212 Sensor Driver");
MODULE_AUTHOR("qipeng.zha@intel.com");
MODULE_LICENSE("GPL");
