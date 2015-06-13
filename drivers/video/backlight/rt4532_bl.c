/*
 * Backlight driver for Analog Devices RT4532 Backlight Devices
 *
 * Copyright 2009-2011 Analog Devices Inc.
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/backlight.h>
#include <linux/i2c/rt4532.h>
#include <linux/HWVersion.h>

extern int Read_PROJ_ID(void);


#define BL_EN_GPIO   188

static struct i2c_client *rt4532_client;
static int backlight_en_gpio;

static int rt4532_i2c_read(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		printk("[ERROR]failed to read reg 0x%x: %d\n", reg, ret);

	return ret;
}

static int rt4532_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		printk("[ERROR]failed to write reg 0x%x: %d\n", reg, ret);

	return ret;
}

int rt4532_brightness_set(int level)
{
	if (level) {
		if (!gpio_get_value(backlight_en_gpio)) {
			gpio_set_value_cansleep(backlight_en_gpio, 1);
			rt4532_i2c_write(rt4532_client, CONFIG_REG,
				(rt4532_i2c_read(rt4532_client, CONFIG_REG) & (~PWM_EN)));
		}
		rt4532_i2c_write(rt4532_client, BL_PWM_CTL_REG, level);
	} else if (gpio_get_value(backlight_en_gpio)) {
		rt4532_i2c_write(rt4532_client, BL_PWM_CTL_REG, level);
		gpio_set_value_cansleep(backlight_en_gpio, 0);
	}

	printk("[DISP] %s, brightness level = %d\n", __func__, level);
	return 0;
}
EXPORT_SYMBOL(rt4532_brightness_set);


static int rt4532_remove(struct i2c_client *client)
{
	return 0;
}

static int rt4532_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	printk("[DISP] %s\n", __func__);
	rt4532_client = client;

	if (Read_PROJ_ID() != PROJ_ID_ZX550ML) {
		backlight_en_gpio = BL_EN_GPIO;
		if (gpio_request(backlight_en_gpio, "backlight_en")) {
			printk("[ERROR]Faild to request backlight enable gpio\n");
			return -EINVAL;
		}
	}

	/* Reset to default setting for CONFIG_REG */
	rt4532_i2c_write(rt4532_client, CONFIG_REG,
		(rt4532_i2c_read(rt4532_client, CONFIG_REG) & (~PWM_EN)));
	rt4532_i2c_write(rt4532_client, BL_PWM_CTL_REG, 102); //set default brightness

	return 0;
}

static const struct i2c_device_id rt4532_id[] = {
	{ "rt4532_bl" },
	{},
};
MODULE_DEVICE_TABLE(i2c, rt4532_id);

static struct i2c_driver rt4532_i2c_driver = {
	.driver	= {
		.name	= "rt4532_bl",
		.owner	= THIS_MODULE,
	},
	.probe	= rt4532_probe,
	.remove	= rt4532_remove,
	.id_table	= rt4532_id,
};

static int __init rt4532_init(void)
{
	int ret;
	printk("[DISP] %s\n", __func__);

	ret = i2c_add_driver(&rt4532_i2c_driver);
	if (ret)
		printk("%s: i2c_add_driver failed\n", __func__);

	return ret;
}

module_init(rt4532_init);


