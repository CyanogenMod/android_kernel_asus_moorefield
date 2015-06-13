/*
 * platform_lis3dh.c: lis3dh platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include <linux/input/lis3dh.h>
#include "platform_lis3dh.h"

void *lis3dh_platform_data(void *info)
{
	static struct lis3dh_acc_platform_data lis3dh_pdata;

	lis3dh_pdata.poll_interval = 200;
	lis3dh_pdata.g_range = 2;
	lis3dh_pdata.negate_x = 1;
	lis3dh_pdata.negate_y = 0;
	lis3dh_pdata.negate_z = 0;
	lis3dh_pdata.axis_map_x = 0;
	lis3dh_pdata.axis_map_y = 1;
	lis3dh_pdata.axis_map_z = 2;
	lis3dh_pdata.gpio_int1 = 60;
	lis3dh_pdata.gpio_int2 = 61;

	return &lis3dh_pdata;

}
