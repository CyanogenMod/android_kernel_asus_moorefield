/*
 * SX9500 proximity sensor driver
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
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/platform_device.h>
#include <linux/mutex.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/sysfs.h>
#include <linux/input.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/i2c.h>

/* GPIOS
* INT_GPIO: SAR_PROX_INT
* RST_GPIO: SIO_PWM[1]
* OUT_GPIO: GPIO_SUS2, modem_sar
*/
#define	INT_GPIO 		149
#define	RST_GPIO		95
#define	OUT_GPIO 		132

#define DRIVER_NAME 		"SASX9500:00"
#define INPUT_NAME		"sx9500"

#define CONFIG_PS_SX9500_DEBUG

#ifdef CONFIG_PS_SX9500_DEBUG
static unsigned int debug_level = 0;
#define DBG_LEVEL1		1
#define DBG_LEVEL2		2
#define DBG_LEVEL3		3
#define DBG_LEVEL4		4
#define SENSOR_DBG(level, fmt, ...)			\
do {							\
	if (level <= debug_level)			\
		printk(KERN_DEBUG "<sx9500>[%d]%s  "	\
			 fmt "\n",			\
			__LINE__, __func__,		\
			##__VA_ARGS__);			\
} while (0)

#else
#define SENSOR_DBG(level, ...)
#endif

/*
 *  I2C Registers
 */
#define SX9500_IRQSTAT_REG	0x00
#define SX9500_TCHCMPSTAT_REG	0x01
#define SX9500_IRQ_ENABLE_REG	0x03
#define SX9500_CPS_CTRL0_REG	0x06
#define SX9500_CPS_CTRL1_REG	0x07
#define SX9500_CPS_CTRL2_REG	0x08
#define SX9500_CPS_CTRL3_REG	0x09
#define SX9500_CPS_CTRL4_REG	0x0A
#define SX9500_CPS_CTRL5_REG	0x0B
#define SX9500_CPS_CTRL6_REG	0x0C
#define SX9500_CPS_CTRL7_REG	0x0D
#define SX9500_CPS_CTRL8_REG	0x0E
#define SX9500_SENSOR_SEL_REG	0x20
#define SX9500_SOFTRESET_REG	0x7F

/*SoftReset register*/
#define SX9500_SOFTRESET	0xDE

/*irqstate register*/
#define CONVDONEIRQ		(1 << 3)
#define COMPDONEIRQ		(1 << 4)
#define FARIRQ			(1 << 5)
#define CLOSEIRQ		(1 << 6)
#define RESETIRQ		(1 << 7)

struct sx9500_data {
	struct i2c_client *client;
	struct input_dev *input_dev;
	struct delayed_work init_work;
	struct mutex lock;
	int enabled;
	int enabled_suspend; /*state before suspend*/
	int gpio_int;
	int gpio_rst;
	int gpio_out;
	int irq;
};

struct reg_data {
	unsigned char reg;
	unsigned char val;
}sx9500_reg_setup[] = {
{
        /*enable colse and far interrupt,
	skip compensation and converion interrupt*/
	.reg = SX9500_IRQ_ENABLE_REG,
	.val = 0x60,
},
{
	.reg = SX9500_CPS_CTRL1_REG,
	.val = 0x43,
},
{
	.reg = SX9500_CPS_CTRL2_REG,
	.val = 0x77,
},
{
	.reg = SX9500_CPS_CTRL3_REG,
	.val = 0x01,
},
{
	.reg = SX9500_CPS_CTRL4_REG,
	.val = 0x80,
},
{
	.reg = SX9500_CPS_CTRL5_REG,
	.val = 0x16,
},
{
        /*threshold:80*/
	.reg = SX9500_CPS_CTRL6_REG,
	.val = 0x16,
},
{
	.reg = SX9500_CPS_CTRL7_REG,
	/*.val = 0x40,*/
	.val = 0x30,
},
{
	.reg = SX9500_CPS_CTRL8_REG,
	.val = 0x00,
},
{
        /*scan period:90ms*/
	.reg = SX9500_CPS_CTRL0_REG,
	.val = 0x24,
},
{
        /*select registers for read back CS2 sensor data*/
	.reg = SX9500_SENSOR_SEL_REG,
	.val = 0x02,
},
};

static int sx9500_read(struct sx9500_data *sx9500, u8 reg, u8 *val)
{
	int ret;

	ret = i2c_smbus_read_byte_data(sx9500->client, reg);
	if (ret >= 0) {
		SENSOR_DBG(DBG_LEVEL3, "read i2c reg %02x=%02x", reg, ret);
		*val = ret;
		return 0;
	} else {
		dev_err(&sx9500->client->dev,
			"Err: return %d when read i2c reg %02x\n",
			ret, reg);
		return ret;
	}
}

static int sx9500_write(struct sx9500_data *sx9500, u8 reg, u8 val)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL3, "write i2c reg %02x=%02x", reg, val);

	ret = i2c_smbus_write_byte_data(sx9500->client, reg, val);
	if (ret < 0) {
		dev_err(&sx9500->client->dev,
			"Err: return %d when write i2c reg %02x=%02x\n",
			ret, reg, val);
	}
	return ret;
}

static int sx9500_init_chip(struct sx9500_data *sx9500)
{
	int ret;
	int i = 0;
	u8 val;

	SENSOR_DBG(DBG_LEVEL3, "enabled:%d", sx9500->enabled);

	/*assert nrst pin*/
	ret = gpio_direction_output(sx9500->gpio_rst, 1);
	if (ret < 0)
		dev_err(&sx9500->client->dev,
			"Fail to output gpio ret=%d\n", ret);

	while((gpio_get_value_cansleep(sx9500->gpio_int) == 0) && ((++i) < 10)) {
		sx9500_read(sx9500, SX9500_IRQSTAT_REG, &val);
		msleep(20);
	}

	if (i >= 10)
		dev_err(&sx9500->client->dev, "Fail to reset\n");

	ret = sx9500_write(sx9500, SX9500_SOFTRESET_REG, SX9500_SOFTRESET);
	if (ret < 0)
		return ret;

	msleep(300);
	ret = sx9500_read(sx9500, SX9500_IRQSTAT_REG, &val);
	if (ret < 0)
		return ret;

	for (i = 0; i < ARRAY_SIZE(sx9500_reg_setup); i++) {
		ret = sx9500_write(sx9500, sx9500_reg_setup[i].reg,
					   sx9500_reg_setup[i].val);
		if (ret < 0)
			return ret;
	}

	/*make sure no irq is pending*/
	ret = sx9500_read(sx9500, SX9500_IRQSTAT_REG, &val);
	if (ret < 0)
		return ret;

	return ret;
}

static int sx9500_enable(struct sx9500_data *sx9500)
{
	int ret = 0;
	u8 val;

	SENSOR_DBG(DBG_LEVEL3, "enabled:%d", sx9500->enabled);

	mutex_lock(&sx9500->lock);
	if (!sx9500->enabled) {
		/*fix me, input far event firstly,
		get diff and compared to threshold ?*/
		input_report_abs(sx9500->input_dev, ABS_X, 1);
		input_sync(sx9500->input_dev);

		ret = sx9500_init_chip(sx9500);
		if (ret < 0) {
			dev_err(&sx9500->client->dev, "init chip %d\n", ret);
		}

		/*enable cs2 sensor*/
		ret = sx9500_read(sx9500, SX9500_CPS_CTRL0_REG, &val);
		if (ret < 0)
			goto out;
		ret = sx9500_write(sx9500, SX9500_CPS_CTRL0_REG, val | 0x4);
		if (ret < 0)
			goto out;

		enable_irq(sx9500->irq);

		sx9500->enabled = 1;
	}

out:
	mutex_unlock(&sx9500->lock);
	return ret;
}

static int sx9500_disable(struct sx9500_data *sx9500)
{
	int ret = 0;
	u8 val;

	SENSOR_DBG(DBG_LEVEL3, "enabled:%d", sx9500->enabled);

	mutex_lock(&sx9500->lock);
	if (sx9500->enabled) {
		/*disable all sensor pins*/
		ret = sx9500_read(sx9500, SX9500_CPS_CTRL0_REG, &val);
		if (ret < 0)
			goto out;
		ret = sx9500_write(sx9500, SX9500_CPS_CTRL0_REG, val & 0xf0);
		if (ret < 0)
			goto out;

		disable_irq(sx9500->irq);
		sx9500->enabled = 0;
	}
out:
	mutex_unlock(&sx9500->lock);
	return ret;
}

static irqreturn_t sx9500_irq(int irq, void *dev_id)
{
	int ret = 0;
	u8 val;
	struct sx9500_data *sx9500 = (struct sx9500_data*)dev_id;

	SENSOR_DBG(DBG_LEVEL3, "enabled:%d", sx9500->enabled);

#ifdef CONFIG_PS_SX9500_DEBUG
	if (debug_level) {
		sx9500_read(sx9500, 0x21, &val);
		sx9500_read(sx9500, 0x22, &val);
		sx9500_read(sx9500, 0x23, &val);
		sx9500_read(sx9500, 0x24, &val);
		sx9500_read(sx9500, 0x25, &val);
		sx9500_read(sx9500, 0x26, &val);
	}
#endif

	ret = sx9500_read(sx9500, SX9500_IRQSTAT_REG, &val);
	if (ret < 0)
		goto out;

	if (val & FARIRQ) {
		SENSOR_DBG(DBG_LEVEL3, "Far irq:%02x", val);

		ret = gpio_direction_output(sx9500->gpio_out, 1);
		if (ret < 0)
			dev_err(&sx9500->client->dev, "set gpio out\n");

		input_report_abs(sx9500->input_dev, ABS_X, 1);
		input_sync(sx9500->input_dev);

	} else if (val & CLOSEIRQ) {
		SENSOR_DBG(DBG_LEVEL3, "Close irq:%02x", val);

		ret = gpio_direction_output(sx9500->gpio_out, 0);
		if (ret < 0)
			dev_err(&sx9500->client->dev, "set gpio out\n");

		input_report_abs(sx9500->input_dev, ABS_X, 0);
		input_sync(sx9500->input_dev);

	} else if (val & RESETIRQ) {
		SENSOR_DBG(DBG_LEVEL3, "Reset irq:%02x", val);
	} else {
		dev_err(&sx9500->client->dev, "Non support irq: %02x\n", val);
	}

out:
	return IRQ_HANDLED;
}

static int sx9500_gpios_init(struct sx9500_data *sx9500)
{
	int ret;
	struct i2c_client *client = sx9500->client;

	sx9500->gpio_rst = acpi_get_gpio_by_index(&client->dev, 0, NULL);
	if (sx9500->gpio_rst < 0) {
		dev_warn(&client->dev, "Fail to get gpio pin by ACPI\n");
		sx9500->gpio_rst = RST_GPIO;
	}

	sx9500->gpio_out = acpi_get_gpio_by_index(&client->dev, 1, NULL);
	if (sx9500->gpio_out < 0) {
		dev_warn(&client->dev, "Fail to get gpio pin by ACPI\n");
		sx9500->gpio_out = OUT_GPIO;
	}

	sx9500->gpio_int = acpi_get_gpio_by_index(&client->dev, 2, NULL);
	if (sx9500->gpio_int < 0) {
		dev_warn(&client->dev, "Fail to get gpio pin by ACPI\n");
		sx9500->gpio_int = INT_GPIO;
	}

	SENSOR_DBG(DBG_LEVEL3, "gpios:%d,%d,%d", sx9500->gpio_int,
					sx9500->gpio_rst, sx9500->gpio_out);

	ret = gpio_request(sx9500->gpio_rst, DRIVER_NAME);
	if (ret < 0) {
		dev_err(&sx9500->client->dev, "Err: request gpio %d\n",
						sx9500->gpio_rst);
		return ret;
	}

	ret = gpio_request(sx9500->gpio_out, DRIVER_NAME);
	if (ret < 0) {
		dev_err(&sx9500->client->dev, "Err: request gpio %d\n",
						sx9500->gpio_out);
		gpio_free(sx9500->gpio_rst);
		return ret;
	}

	ret = gpio_request(sx9500->gpio_int, DRIVER_NAME);
	if (ret < 0) {
		dev_err(&sx9500->client->dev, "Err: request gpio %d\n",
						sx9500->gpio_int);
		gpio_free(sx9500->gpio_rst);
		gpio_free(sx9500->gpio_out);
		return ret;
	}
	return ret;
}

static int sx9500_request_irq(struct sx9500_data *sx9500)
{
	int ret;

	gpio_direction_input(sx9500->gpio_int);
	sx9500->irq = gpio_to_irq(sx9500->gpio_int);
	irq_set_status_flags(sx9500->irq, IRQ_NOAUTOEN);
	ret = request_threaded_irq(sx9500->irq, NULL, sx9500_irq,
			IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
			DRIVER_NAME, sx9500);
	if (ret < 0) {
		dev_err(&sx9500->client->dev,
			"Fail to request irq:%d ret=%d\n", sx9500->irq, ret);
		return ret;
	}

	return 0;
}

static int sx9500_input_init(struct sx9500_data *sx9500)
{
	int ret;
	struct input_dev *input;

	input = input_allocate_device();
	if (!input) {
		dev_err(&sx9500->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}
	input->name = INPUT_NAME;
	input->dev.parent = &sx9500->client->dev;
	set_bit(EV_ABS, input->evbit);
	set_bit(ABS_X, input->absbit);

	ret = input_register_device(input);
	if (ret) {
		dev_err(&sx9500->client->dev,
			"unable to register input device %s:%d\n",
			input->name, ret);
		goto err;
	}

	sx9500->input_dev = input;
	return 0;
err:
	input_free_device(input);
	return ret;
}

#ifdef CONFIG_PS_SX9500_DEBUG
static ssize_t sx9500_debug_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%u\n", debug_level);
}

static ssize_t sx9500_debug_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	debug_level = val;
	return count;
}

static DEVICE_ATTR(debug, S_IRUGO|S_IWUSR, sx9500_debug_show,
		sx9500_debug_store);
#endif

static ssize_t sx9500_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct sx9500_data *sx9500 = dev_get_drvdata(dev);

	return sprintf(buf, "%d\n", sx9500->enabled);
}

static ssize_t sx9500_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9500_data *sx9500 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val)
		sx9500_enable(sx9500);

	return count;
}

static ssize_t sx9500_disable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct sx9500_data *sx9500 = dev_get_drvdata(dev);
	unsigned long val;

	if (kstrtoul(buf, 0, &val))
		return -EINVAL;

	if (val)
		sx9500_disable(sx9500);

	return count;
}

static DEVICE_ATTR(enable, S_IRUGO|S_IWUSR, sx9500_enable_show,
		sx9500_enable_store);
static DEVICE_ATTR(disable, S_IRUGO|S_IWUSR, sx9500_enable_show,
		sx9500_disable_store);

static struct attribute *sx9500_attributes[] = {
	&dev_attr_enable.attr,
	&dev_attr_disable.attr,
#ifdef CONFIG_PS_SX9500_DEBUG
	&dev_attr_debug.attr,
#endif
	NULL
};

static struct attribute_group sx9500_attribute_group = {
	.attrs = sx9500_attributes
};

static void sx9500_init_work_func(struct work_struct *work)
{
	struct sx9500_data *sx9500 = container_of((struct delayed_work *)work,
					struct sx9500_data, init_work);

	SENSOR_DBG(DBG_LEVEL3, "enabled:%d", sx9500->enabled);

	sx9500_enable(sx9500);
}

static struct sx9500_data sx9500;
static int sx9500_probe(struct i2c_client *client,
			const struct i2c_device_id *devid)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL3, "%s", client->name);

	if (!i2c_check_functionality(client->adapter,
		I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable\n");
		return -ENODEV;
	}

	sx9500.client = client;
	sx9500.enabled = 0;
	sx9500.enabled_suspend = 0;
	mutex_init(&sx9500.lock);

	i2c_set_clientdata(client, &sx9500);

	ret = sx9500_gpios_init(&sx9500);
	if (ret) {
		goto out;
	}

	ret = sx9500_request_irq(&sx9500);
	if (ret < 0) {
		dev_err(&client->dev, "request irq\n");
		goto ret_irq;
	}

	ret = sx9500_input_init(&sx9500);
	if (ret < 0) {
		dev_err(&client->dev, "input init %d\n", ret);
		goto ret_input;
	}

	ret = sysfs_create_group(&client->dev.kobj, &sx9500_attribute_group);
	if (ret) {
		dev_err(&client->dev, "sysfs create group\n");
		goto ret_sys;
	}

	/*since the init will take 0.3s, put in delayed work*/
	INIT_DELAYED_WORK(&sx9500.init_work, sx9500_init_work_func);
	schedule_delayed_work(&sx9500.init_work, msecs_to_jiffies(300));

	return 0;
ret_sys:
	input_unregister_device(sx9500.input_dev);
ret_input:
	free_irq(sx9500.gpio_int, &sx9500);
ret_irq:
	gpio_free(sx9500.gpio_rst);
	gpio_free(sx9500.gpio_out);
	gpio_free(sx9500.gpio_int);
out:
	return ret;
}

static int sx9500_remove(struct i2c_client *client)
{
	struct sx9500_data *sx9500 = i2c_get_clientdata(client);

	cancel_delayed_work_sync(&sx9500->init_work);
	sx9500_disable(sx9500);
	sysfs_remove_group(&client->dev.kobj, &sx9500_attribute_group);
	free_irq(sx9500->irq, sx9500);
	input_unregister_device(sx9500->input_dev);
	gpio_free(sx9500->gpio_rst);
	gpio_free(sx9500->gpio_out);
	gpio_free(sx9500->gpio_int);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sx9500_suspend(struct device *dev)
{
	struct sx9500_data *sx9500 = dev_get_drvdata(dev);

	sx9500->enabled_suspend = sx9500->enabled;
	sx9500_disable(sx9500);
	return 0;
}

static int sx9500_resume(struct device *dev)
{
	struct sx9500_data *sx9500 = dev_get_drvdata(dev);

	if (sx9500->enabled_suspend)
		sx9500_enable(sx9500);
	return 0;
}
static SIMPLE_DEV_PM_OPS(sx9500_pm, sx9500_suspend, sx9500_resume);
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id sx9500_id[] = {
	{DRIVER_NAME, 0},
	{},
};

static struct i2c_driver sx9500_driver = {
	.probe		= sx9500_probe,
	.remove		= sx9500_remove,
	.driver	= {
		.name	= DRIVER_NAME,
		.owner	= THIS_MODULE,
#ifdef CONFIG_PM_SLEEP
		.pm	= &sx9500_pm,
#endif
	},
	.id_table = sx9500_id,
};

//#define CONFIG_SX9500_MANUAL_DEVICE
#ifdef CONFIG_SX9500_MANUAL_DEVICE
static int register_i2c_device(int bus, int addr, char *name)
{
	int ret = 0;
	struct i2c_adapter *adapter;
	struct i2c_board_info info;

	SENSOR_DBG(DBG_LEVEL2, "%s", name);

	memset(&info, 0, sizeof(info));
	strlcpy(info.type, name, I2C_NAME_SIZE);
	adapter = i2c_get_adapter(bus);
	if (!adapter)
		return -ENODEV;
	else {
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

static int __init ps_sx9500_init(void)
{
	int ret;

	SENSOR_DBG(DBG_LEVEL2, "%s", DRIVER_NAME);

#ifdef CONFIG_SX9500_MANUAL_DEVICE
	register_i2c_device(3, 0x28, DRIVER_NAME);
#endif

	ret = i2c_add_driver(&sx9500_driver);
	if (ret < 0)
		printk(KERN_ERR "Fail to register ps sx9500 driver\n");

	return ret;
}

static void __exit ps_sx9500_exit(void)
{
	SENSOR_DBG(DBG_LEVEL2, "%s", DRIVER_NAME);

	i2c_del_driver(&sx9500_driver);
}

module_init(ps_sx9500_init);
module_exit(ps_sx9500_exit);

MODULE_AUTHOR("IO&Sensor");
MODULE_DESCRIPTION("Proximity Sensor SX9500");
MODULE_LICENSE("GPL");
