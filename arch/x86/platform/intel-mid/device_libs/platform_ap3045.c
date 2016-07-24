/*
* setup of GPIO control of LEDs and buttons on Asus ZE553.
*
*
* Copyright (C) 2015 ASUS Corporation
* Author: Daniel Chan
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
#include <linux/ap3045.h>


void *asus_ap3045_platform_data(void *info)
{
	static struct ap3045_data asus_ap3045_pdata = {
		.gpio = 44 ,
		.reg = 0x1c ,
	};
	return &asus_ap3045_pdata;

	return 0;
}

MODULE_LICENSE("GPL");
