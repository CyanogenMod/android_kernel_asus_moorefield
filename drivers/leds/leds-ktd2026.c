/*
 * leds-ktd2026.c - LED driver for KTD2026
 *
 * Copyright (C) 2016 Asus
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
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
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/leds.h>
#include <linux/earlysuspend.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/HWVersion.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>

#define LED_INFO(...)		printk("[LED] " __VA_ARGS__);
#define LED_ERR(...)			printk("[LED_ERR] " __VA_ARGS__);

/* KTD2026 IC registers */
#define EN_RST_REG			0x00
#define FLASH_P_REG			0x01
#define FLASH_ON_1_REG		0x02
#define FLASH_ON_2_REG		0x03
#define CHANNEL_CTRL_REG		0x04
#define RAMP_RATE_REG			0x05
#define LED_1_IOUT_REG			0x06
#define LED_2_IOUT_REG			0x07
#define LED_3_IOUT_REG			0x08
#define LED_4_IOUT_REG			0x09
/* KTD2026 ILED settings */
#define IOUT_0MA				0x00
#define IOUT_10MA				0x4F

extern int Read_PROJ_ID(void);

/* +++global variables+++ */
static int red_led_flag, green_led_flag, red_blink_flag, green_blink_flag, disable_led_flag;
static struct i2c_client *led_client;
/* ---global variables--- */

struct led_info_data {
	struct led_classdev cdev;
	struct work_struct work;
};

struct led_info_priv {
	int num_leds;
	struct led_info_data leds[];
};

static int led_i2c_read(struct i2c_client *client, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(client, reg);
	if (ret < 0)
		LED_ERR("failed to read reg 0x%x: %d\n", reg, ret);

	return ret;
}

static int led_i2c_write(struct i2c_client *client, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(client, reg, val);
	if (ret < 0)
		LED_ERR("failed to write reg 0x%x: %d\n", reg, ret);

	return ret;
}

static ssize_t disable_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	int len=0;

	len += sprintf(buf + len, "%d\n", disable_led_flag);
	return len;
}
static ssize_t disable_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
	sscanf(buf, "%d", &disable_led_flag);
	return count;
}
static DEVICE_ATTR(disable, 0664, disable_show, disable_store);

static ssize_t blink_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_info_data *led_dat = container_of(led_cdev, struct led_info_data, cdev);
	int len=0;

	if (!strcmp(led_dat->cdev.name, "red"))
		len += sprintf(buf + len, "%d\n", red_blink_flag);
	else if (!strcmp(led_dat->cdev.name, "green")&&(green_blink_flag==1))
		len += sprintf(buf + len, "%d\n", green_blink_flag);

	return len;
}
static ssize_t blink_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_info_data *led_dat = container_of(led_cdev, struct led_info_data, cdev);
	int value;

	sscanf(buf, "%d", &value);
	LED_INFO("%s +++, value=%d, led=%s\n", __func__, value, led_dat->cdev.name);

	if (disable_led_flag==0) {
		if (value==0) {
			/* stop blink */
			if ((!strcmp(led_dat->cdev.name, "red"))&&(red_blink_flag==1)) {
				if (!green_blink_flag) {
					led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
					led_i2c_write(led_client, LED_1_IOUT_REG, IOUT_0MA);
				} else {
					led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x08);
				}
				red_blink_flag = 0;
				red_led_flag = 0;
			} else if ((!strcmp(led_dat->cdev.name, "green"))&&(green_blink_flag==1)) {
				if (!red_blink_flag) {
					led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
					led_i2c_write(led_client, LED_2_IOUT_REG, IOUT_0MA);
				} else {
					led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x02);
				}
				green_blink_flag = 0;
				green_led_flag = 0;
			}
			if ((red_led_flag==0)&&(green_led_flag==0)&&(red_blink_flag==0)&&(green_blink_flag==0))
				led_i2c_write(led_client, EN_RST_REG, 0x0f);
		} else if (value>0&&value<=100) {
			/* start blink */
			if (!strcmp(led_dat->cdev.name, "red")) {
				/* reset chips */
				led_i2c_write(led_client, EN_RST_REG, 0x07);
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
				led_i2c_write(led_client, LED_1_IOUT_REG, IOUT_10MA);
				led_i2c_write(led_client, FLASH_P_REG, 0x1E);
				led_i2c_write(led_client, FLASH_ON_1_REG, 0x04);
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x02);
				red_led_flag = 1;
				red_blink_flag = 1;
			} else if (!strcmp(led_dat->cdev.name, "green")) {
				/* reset chips */
				led_i2c_write(led_client, EN_RST_REG, 0x07);
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
				led_i2c_write(led_client, LED_2_IOUT_REG, IOUT_10MA);
				led_i2c_write(led_client, FLASH_P_REG, 0x0E);
				led_i2c_write(led_client, FLASH_ON_1_REG, 0x08);
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x08);
				green_led_flag = 1;
				green_blink_flag = 1;
			}
		} else {
			LED_INFO("%s, incorrect pwm value:%d (0-100).\n", __func__, value);
		}
	}
	return count;
}
static DEVICE_ATTR(blink, 0644,
        blink_show, blink_store);

static inline int sizeof_led_info_priv(int num_leds)
{
	return sizeof(struct led_info_priv) +
		(sizeof(struct led_info_data) * num_leds);
}

void ktd2026_led_brightness_set(int led, int brightness) {	//led=0:red, led=1:green
	LED_INFO("%s +++ , brightness=%d, led=%d\n", __func__, brightness, led);
	if (brightness==0) {
		if ((led==0)&&(red_led_flag==1)) {
			if (!green_led_flag) {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
				led_i2c_write(led_client, LED_1_IOUT_REG, IOUT_0MA);
			} else {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x04);
			}
			red_led_flag = 0;
		} else if ((led==1)&&(green_led_flag==1)) {
			if (!red_led_flag) {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
				led_i2c_write(led_client, LED_2_IOUT_REG, IOUT_0MA);
			} else {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x01);
			}
			green_led_flag = 0;
		}
		if ((red_led_flag==0)&&(green_led_flag==0)&&(red_blink_flag==0)&&(green_blink_flag==0))
			led_i2c_write(led_client, EN_RST_REG, 0x0f);
	} else if (brightness>0&&brightness<=255) {
		if (led==0) {
			if (!green_led_flag) {
				/* reset chips */
				led_i2c_write(led_client, EN_RST_REG, 0x07);
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
			}
			led_i2c_write(led_client, LED_1_IOUT_REG, IOUT_10MA);
			if (!green_led_flag) {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x01);
			} else {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x05);
			}
			red_led_flag = 1;
		} else if (led==1) {
			if (!red_led_flag) {
				/* reset chips */
				led_i2c_write(led_client, EN_RST_REG, 0x07);
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x00);
			}
			led_i2c_write(led_client, LED_2_IOUT_REG, IOUT_10MA);
			if (!red_led_flag) {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x04);
			} else {
				led_i2c_write(led_client, CHANNEL_CTRL_REG, 0x05);
			}
			green_led_flag = 1;
		}
	}
}
EXPORT_SYMBOL(ktd2026_led_brightness_set);

static void asus_led_set_brightness(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct led_info_data *led_dat = container_of(led_cdev, struct led_info_data, cdev);

	LED_INFO("%s +++ , brightness=%d, led=%s, disable_led_flag=%d\n", __func__, value, led_dat->cdev.name, disable_led_flag);

	if (disable_led_flag==0) {
		if (value==0) {
			if (!strcmp(led_dat->cdev.name, "red")) {
				ktd2026_led_brightness_set(0, 0);
			} else if (!strcmp(led_dat->cdev.name, "green")) {
				ktd2026_led_brightness_set(1, 0);
			}
		} else if (value>0&&value<=255) {
			if (!strcmp(led_dat->cdev.name, "red")) {
				ktd2026_led_brightness_set(0, value);
			} else if (!strcmp(led_dat->cdev.name, "green")) {
				ktd2026_led_brightness_set(1, value);
			}
		}
	}
}

static void delete_led(struct led_info_data *led)
{
	led_classdev_unregister(&led->cdev);
}

static int ktd2026_led_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct led_info_priv *priv;
	struct i2c_adapter *adapter;
	int i, ret = 0;

	LED_INFO("%s +++\n", __func__);
	adapter = to_i2c_adapter(client->dev.parent);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	priv = kzalloc(sizeof_led_info_priv(2), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;
	
	priv->num_leds = 2;
	priv->leds[0].cdev.name = "red";
	priv->leds[1].cdev.name = "green";
	for (i = 0; i < priv->num_leds; i++) {
		priv->leds[i].cdev.brightness_set = asus_led_set_brightness;
		priv->leds[i].cdev.brightness = LED_OFF;
		priv->leds[i].cdev.max_brightness = 255;
		ret = led_classdev_register(&client->dev, &priv->leds[i].cdev);
		if (ret < 0)
			LED_ERR("led_classdev_register led[%d] fail\n", i);
		ret = device_create_file(priv->leds[i].cdev.dev, &dev_attr_disable);
		if (ret)
			LED_ERR("device_create_file disable in led[%d] fail\n", i);
		ret = device_create_file(priv->leds[i].cdev.dev, &dev_attr_blink);
		if (ret)
			LED_ERR("device_create_file blink in led[%d] fail\n", i);
	}

	/* initial flags */
	disable_led_flag = 0;
	red_led_flag = 0;
	green_led_flag = 0;
	red_blink_flag = 0;
	green_blink_flag = 0;
	/* set chip into shutdown mode */
	led_i2c_write(client, EN_RST_REG, 0x0f);

	led_client = client;
	LED_INFO("%s ---\n", __func__);
	return 0;
}

void ktd2026_led_shutdown(struct i2c_client *client)
{
	struct led_info_priv *priv;
	int i;

	LED_INFO("%s +++ \n", __func__);
	led_i2c_write(led_client, LED_1_IOUT_REG, IOUT_0MA);
	led_i2c_write(led_client, LED_2_IOUT_REG, IOUT_0MA);
	/* set chip into shutdown mode */
	led_i2c_write(client, EN_RST_REG, 0x0f);
	priv = dev_get_drvdata(&client->dev);
	for (i = 0; i < priv->num_leds; i++) {
		device_remove_file(priv->leds[i].cdev.dev, &dev_attr_disable);
		device_remove_file(priv->leds[i].cdev.dev, &dev_attr_blink);
		delete_led(&priv->leds[i]);
	}
	dev_set_drvdata(&client->dev, NULL);
	kfree(priv);

	return ;
}

static const struct i2c_device_id ktd2026_led_id[] = {
	{ "ktd2026" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ktd2026_led_id);

static struct i2c_board_info ktd2026_board_info[] = {
	{
		I2C_BOARD_INFO("ktd2026", 0x30),
	},
};

static struct i2c_driver ktd2026_led_driver = {
	.driver   = {
		.name    = "ktd2026",
		.owner   = THIS_MODULE,
	},
	.probe    = ktd2026_led_probe,
	.shutdown = ktd2026_led_shutdown,
	.id_table = ktd2026_led_id,
};

static int __init ktd2026_init(void)
{
	int ret;

	LED_INFO("%s\n", __func__);

	/* check if the project support KTD2026 */
	if (Read_PROJ_ID()!=PROJ_ID_ZS550ML_SEC) {
		LED_INFO("this project doesn't support ktd2026, skip init function.\n");
		return 0;
	}

	ret = i2c_add_driver(&ktd2026_led_driver);
	if (ret) {
		LED_ERR("%s: i2c_add_driver for main charger failed\n", __func__);
		return ret;
	}
	ret = i2c_register_board_info(4, ktd2026_board_info, ARRAY_SIZE(ktd2026_board_info));
	if (ret) {
		LED_ERR("i2c_register_board_info for main charger failed\n");
		return ret;
	}

	return ret;
}
module_init(ktd2026_init);

static void __exit ktd2026_exit(void)
{
	LED_INFO("%s\n", __func__);
	i2c_del_driver(&ktd2026_led_driver);
}
module_exit(ktd2026_exit);

MODULE_AUTHOR("Chih-Hsuan Chang <chih-hsuan_chang@asus.com>");
MODULE_DESCRIPTION("Asus LED Driver");
MODULE_LICENSE("GPL");
