/*
 * setup of GPIO control of LEDs and buttons on Asus 500CG.
 *
 *
 * Copyright (C) 2013 Asus
 * Written by Chih-Hsuan Chang <chih-hsuan_chang@asus.com>

 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2
 * as published by the Free Software Foundation.
 */

#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/string.h>
#include <linux/module.h>
#include <linux/leds.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/gpio_keys.h>
#include <linux/i2c.h>

static struct led_info asus_led_leds[] = {
	{
		.name = "red",
	},
	{
		.name = "green",
	},
};
static struct led_platform_data asus_led_pdata = {
	.num_leds = ARRAY_SIZE(asus_led_leds),
	.leds = asus_led_leds,
};
void *asus_led_platform_data(void *info)
{
	return &asus_led_pdata;
}

MODULE_AUTHOR("Chih-Hsuan Chang <chih-hsuan_chang@asus.com>");
MODULE_LICENSE("GPL");
