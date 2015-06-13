/*
 * platform_lsm303.c: lsm303 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/input/lis3dh.h>
#include <asm/intel-mid.h>
#include "platform_lsm303.h"

void *lsm303dlhc_accel_platform_data(void *info)
{
	static struct lis3dh_acc_platform_data accel;

	accel.poll_interval = 200;
	accel.g_range = 2;
	accel.negate_x = 1;
	accel.negate_y = 0;
	accel.negate_z = 0;
	accel.axis_map_x = 0;
	accel.axis_map_y = 1;
	accel.axis_map_z = 2;
	accel.gpio_int1 = get_gpio_by_name("accel_int");
	accel.gpio_int2 = get_gpio_by_name("accel_2");
	accel.model = MODEL_LSM303DLHC;

	return &accel;
}
