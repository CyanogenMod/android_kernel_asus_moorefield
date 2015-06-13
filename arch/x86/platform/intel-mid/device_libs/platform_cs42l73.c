/*
 * platform_cs42l73.c: cs42l73 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:	Hevendra R <hevendrax.raja.reddy@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <asm/platform_cs42l73.h>

static struct cs42l73_pdata pdata;

void __init *cs42l73_platform_data(void *info)
{
	pdata.codec_rst = get_gpio_by_name("audiocodec_rst");
	return &pdata;
}
