/*
 * platform_accel.c: accel platform data initilization file
 *
 * (C) Copyright 2013
 * Author : cheng_kao
 *
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "platform_accel.h"
#include <linux/kxtj9.h>

void *accel_platform_data(void *info)
{
	struct kxtj9_platform_data accel_data_kxtj9;
	accel_data_kxtj9.gpio = get_gpio_by_name("accel_int");
	return &accel_data_kxtj9;
}
