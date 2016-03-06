/*
 * leds-asus.c - Asus charging LED driver
 *
 * Copyright (C) 2014 Asus
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

#define LED_INFO(...)			printk("[LED] " __VA_ARGS__);
#define LED_ERR(...)			printk("[LED_ERR] " __VA_ARGS__);
#define CONTROL_LED			1

#define CHGDETGPO_REG		0x105
#define INDCAT_EN			0x7E
/* this is for TCA6507 IC settings*/
#define GREEN_BIT				BIT(0)
#define RED_BIT					BIT(1)
#define SELECT0_REG				0x00
#define SELECT1_REG				0x01
#define SELECT2_REG				0x02
#define FADE_ON_TIME_REG		0x03
#define FULLY_ON_TIME_REG		0x04
#define FADE_OFF_TIME_REG		0x05
#define F_FULLY_OFF_TIME_REG	0x06
#define S_FULLY_OFF_TIME_REG	0x07

extern int Read_PROJ_ID(void);

/* +++global variables+++ */
static int red_led_flag, green_led_flag, red_blink_flag, green_blink_flag, power_on_flag;
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

#ifdef CONTROL_LED
static int disable_led_flag;
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
static DEVICE_ATTR(disable, 0664,
        disable_show, disable_store);
#endif

static void led_enable_set(u16 reg, int value)
{
	int ret;
	uint8_t ctrldata;

	LED_INFO("%s, reg=0x%X, value=%d\n", __func__, reg, value);

	/* set bit ALTFUNCEN = 0 */
	ret = intel_scu_ipc_ioread8(CHGDETGPO_REG, &ctrldata);
	if (ret) {
		LED_ERR(" IPC Failed to read %d\n", ret);
	}
	ctrldata &= ~(BIT(6));
	ret = intel_scu_ipc_iowrite8(CHGDETGPO_REG, ctrldata);
	if (ret) {
			LED_ERR(" IPC Failed to write %d\n", ret);
	}

	ret = intel_scu_ipc_ioread8(reg, &ctrldata);
	if (ret) {
		LED_ERR(" IPC Failed to read %d\n", ret);
	}
	if (value)
		ctrldata |= (BIT(5)|BIT(4)|BIT(0));
	else
		ctrldata &= ~(BIT(5)|BIT(4)|BIT(0));
	ret = intel_scu_ipc_iowrite8(reg, ctrldata);
	if (ret) {
			LED_ERR(" IPC Failed to write %d\n", ret);
	}
	power_on_flag = value;
}

static ssize_t blink_show(struct device *dev,
        struct device_attribute *attr, char *buf)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_info_data *led_dat = container_of(led_cdev, struct led_info_data, cdev);
	int len=0;

	if(!strcmp(led_dat->cdev.name, "red"))
		len += sprintf(buf + len, "%d\n", red_blink_flag);
	else if(!strcmp(led_dat->cdev.name, "green")&&(green_blink_flag==1))
		len += sprintf(buf + len, "%d\n", green_blink_flag);

	return len;
}

/* This table is pulled from a kernel driver for a tca6507 chip and seems (experimentally)
 * to be close enough to what our chip is doing to go with it.
 */
static const int blink_ms[] = {
	64, 128, 192, 256, 384, 512, 768, 1024, 1536, 2048, 3072, 4096, 5760, 8128, 16320
};
#define N_BLINK_MS (sizeof(blink_ms) / sizeof(blink_ms[0]))

static int ms_to_value(int ms)
{
	int i;

	/* Divide by 2 because the value is used for both the time in state and fade time */
	ms /= 2;

	for (i = 0; i < N_BLINK_MS-1 && blink_ms[i] < ms; i++) {}
	return 0x40 + i;
}

static ssize_t blink_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count)
{
	struct led_classdev *led_cdev = dev_get_drvdata(dev);
	struct led_info_data *led_dat = container_of(led_cdev, struct led_info_data, cdev);
	int value;
	int on_time_ms = 1, off_time_ms = 1000;
	int on_value, off_value;

	sscanf(buf, "%d %d %d", &value, &on_time_ms, &off_time_ms);
	on_value = ms_to_value(on_time_ms);
	off_value = ms_to_value(off_time_ms);
	LED_INFO("%s +++, value=%d, led=%s on_time=%d|%x off_time=%d|%x\n", __func__,
		value, led_dat->cdev.name, on_time_ms, on_value, off_time_ms, off_value);
#ifdef CONTROL_LED
	if(disable_led_flag==0) {
#endif
		if(value==0) {
			/* stop blink */
			if((!strcmp(led_dat->cdev.name, "red"))&&(power_on_flag==1)) {
				led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)&(~RED_BIT)));
				led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT2_REG)&(~RED_BIT)));
				red_blink_flag = 0;
				red_led_flag = 0;
			}else if((!strcmp(led_dat->cdev.name, "green"))&&(power_on_flag==1)) {
				led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)&(~GREEN_BIT)));
				led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT2_REG)&(~GREEN_BIT)));
				green_blink_flag = 0;
				green_led_flag = 0;
			}
			/*power off*/
			if((red_led_flag==0)&&(green_led_flag==0)&&(red_blink_flag==0)&&(green_blink_flag==0))
				led_enable_set(INDCAT_EN, 0);
		}
		else if(value>0&&value<=100) {
			/* power on*/
			led_enable_set(INDCAT_EN, 1);
			/* start blink */
			if(!strcmp(led_dat->cdev.name, "red")) {
				led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)|RED_BIT));
				led_i2c_write(led_client, SELECT2_REG, (led_i2c_read(led_client, SELECT2_REG)|RED_BIT));
				red_led_flag = 1;
				red_blink_flag = 1;
			}else if(!strcmp(led_dat->cdev.name, "green")) {
				led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)|GREEN_BIT));
				led_i2c_write(led_client, SELECT2_REG, (led_i2c_read(led_client, SELECT2_REG)|GREEN_BIT));
				green_led_flag = 1;
				green_blink_flag = 1;
			}

			led_i2c_write(led_client, FADE_ON_TIME_REG, on_value);
			led_i2c_write(led_client, FULLY_ON_TIME_REG, on_value+1);
			led_i2c_write(led_client, F_FULLY_OFF_TIME_REG, off_value);
			led_i2c_write(led_client, FADE_OFF_TIME_REG, on_value);
			led_i2c_write(led_client, S_FULLY_OFF_TIME_REG, off_value);
		}else
			LED_INFO("%s, incorrect pwm value:%d (0-100).\n", __func__, value);
#ifdef CONTROL_LED
	}
#endif
	return count;
}
static DEVICE_ATTR(blink, 0644,
        blink_show, blink_store);

static inline int sizeof_led_info_priv(int num_leds)
{
	return sizeof(struct led_info_priv) +
		(sizeof(struct led_info_data) * num_leds);
}

void led_brightness_set(int led, int brightness) {	//led=0:red, led=1:green
	LED_INFO("%s +++ , brightness=%d, led=%d\n", __func__, brightness, led);
	if(brightness==0) {
		if((led==0)&&(red_led_flag==1)) {
			led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)&(~RED_BIT)));
			red_led_flag = 0;
		}else if((led==1)&&(green_led_flag==1)) {
			led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)&(~GREEN_BIT)));
			green_led_flag = 0;
		}
		if((red_led_flag==0)&&(green_led_flag==0)&&(red_blink_flag==0)&&(green_blink_flag==0))
			led_enable_set(INDCAT_EN, 0);
	}else if(brightness>0&&brightness<=255) {
		led_enable_set(INDCAT_EN, 1);
		if(led==0) {
			led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)|RED_BIT));
			red_led_flag = 1;
		}else if(led==1) {
			led_i2c_write(led_client, SELECT1_REG, (led_i2c_read(led_client, SELECT1_REG)|GREEN_BIT));
			green_led_flag = 1;
		}
	}
}
EXPORT_SYMBOL(led_brightness_set);

static void asus_led_set_brightness(struct led_classdev *led_cdev,
	enum led_brightness value)
{
	struct led_info_data *led_dat = container_of(led_cdev, struct led_info_data, cdev);

	LED_INFO("%s +++ , brightness=%d, led=%s\n", __func__, value, led_dat->cdev.name);
#ifdef CONTROL_LED
	LED_INFO("disable_led_flag=%d\n", disable_led_flag);
#endif
#ifdef CONTROL_LED
	if(disable_led_flag==0) {
#endif
		if(value==0) {
			if(!strcmp(led_dat->cdev.name, "red")) {
				led_brightness_set(0, 0);
			}else if(!strcmp(led_dat->cdev.name, "green")) {
				led_brightness_set(1, 0);
			}
		}else if(value>0&&value<=255) {
			if(!strcmp(led_dat->cdev.name, "red")) {
				led_brightness_set(0, value);
			}else if(!strcmp(led_dat->cdev.name, "green")) {
				led_brightness_set(1, value);
			}
		}
#ifdef CONTROL_LED
	}
#endif
}

static void delete_led(struct led_info_data *led)
{
	led_classdev_unregister(&led->cdev);
	//cancel_work_sync(&led->work);
}

static int asus_led_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	struct led_platform_data *pdata;
	struct led_info_priv *priv;
	struct i2c_adapter *adapter;
	int i, ret = 0;

	LED_INFO("%s +++\n", __func__);
	/* check if the project support TCA6507 */
	if (Read_PROJ_ID()==PROJ_ID_ZS550ML_SEC) {
		LED_INFO("this project doesn't support tca6507, skip probe function.\n");
		return 0;
	}

	adapter = to_i2c_adapter(client->dev.parent);
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -EIO;

	pdata = client->dev.platform_data;
	if (pdata && pdata->num_leds) {
		priv = kzalloc(sizeof_led_info_priv(pdata->num_leds),
				GFP_KERNEL);
		if (!priv)
			return -ENOMEM;

		priv->num_leds = pdata->num_leds;
		for (i = 0; i < priv->num_leds; i++) {
			priv->leds[i].cdev.name = pdata->leds[i].name;
			priv->leds[i].cdev.brightness_set = asus_led_set_brightness;
			priv->leds[i].cdev.brightness = LED_OFF;
			priv->leds[i].cdev.max_brightness = 255;
			ret = led_classdev_register(&client->dev, &priv->leds[i].cdev);
			if (ret < 0)
				LED_ERR("led_classdev_register led[%d] fail\n", i);
#ifdef CONTROL_LED
			disable_led_flag = 0;
			ret = device_create_file(priv->leds[i].cdev.dev, &dev_attr_disable);
			if (ret)
				LED_ERR("device_create_file disable in led[%d] fail\n", i);
#endif
			red_led_flag = 0;
			green_led_flag = 0;
			red_blink_flag = 0;
			green_blink_flag = 0;
			ret = device_create_file(priv->leds[i].cdev.dev, &dev_attr_blink);
			if (ret)
				LED_ERR("device_create_file blink in led[%d] fail\n", i);
		}
	}
	led_client = client;

	LED_INFO("%s ---\n", __func__);
	return 0;
}

static int asus_led_remove(struct i2c_client *client)
{
	struct led_info_priv *priv;
	int i;

	LED_INFO("%s +++ \n", __func__);
	priv = dev_get_drvdata(&client->dev);
	for (i = 0; i < priv->num_leds; i++) {
#ifdef CONTROL_LED
		device_remove_file(priv->leds[i].cdev.dev, &dev_attr_disable);
#endif
		device_remove_file(priv->leds[i].cdev.dev, &dev_attr_blink);
		delete_led(&priv->leds[i]);
	}
	dev_set_drvdata(&client->dev, NULL);
	kfree(priv);

	return 0;
}

static const struct i2c_device_id asus_led_id[] = {
	{ "tca6507" },
	{ }
};
MODULE_DEVICE_TABLE(i2c, asus_led_id);

static struct i2c_driver asus_led_driver = {
	.driver   = {
		.name    = "tca6507",
		.owner   = THIS_MODULE,
	},
	.probe    = asus_led_probe,
	.remove   = asus_led_remove,
	.id_table = asus_led_id,
};
module_i2c_driver(asus_led_driver);

MODULE_AUTHOR("Chih-Hsuan Chang <chih-hsuan_chang@asus.com>");
MODULE_DESCRIPTION("Asus LED Driver");
MODULE_LICENSE("GPL");
