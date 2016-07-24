/*
 * platform_l3g4200d.c:  l3g4200d platform data initilization file
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
#include <linux/input/l3g4200d_poll.h>
#include <asm/intel-mid.h>
#include "platform_l3g4200d.h"

void __init *l3g4200d_platform_data(void *info)
{
	static struct l3g4200d_gyr_platform_data l3g4200d_pdata;

	l3g4200d_pdata.fs_range = L3G4200D_GYR_FS_2000DPS;
	l3g4200d_pdata.poll_interval = 200;
	l3g4200d_pdata.negate_x = 0;
	l3g4200d_pdata.negate_y = 0;
	l3g4200d_pdata.negate_z = 0;
	l3g4200d_pdata.axis_map_x = 0;
	l3g4200d_pdata.axis_map_y = 1;
	l3g4200d_pdata.axis_map_z = 2;

	return &l3g4200d_pdata;
}
