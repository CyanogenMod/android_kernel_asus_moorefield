/*
 * platform_wm5102.c: wm51020 platform data initilization file
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
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mfd/arizona/pdata.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/intel_pmic.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include "platform_wm5102.h"

static const struct arizona_micd_range micd_ctp_ranges[] = {
	{ .max =  11, .key = KEY_MEDIA },
	{ .max =  28, .key = KEY_MEDIA },
	{ .max =  54, .key = KEY_MEDIA },
	{ .max = 100, .key = KEY_MEDIA },
	{ .max = 186, .key = KEY_MEDIA },
	{ .max = 430, .key = KEY_MEDIA },
};

static struct arizona_pdata wm5102_pdata  = {
	.ldoena = 44,
	.clk32k_src = ARIZONA_32KZ_MCLK2,
	.irq_flags = IRQF_TRIGGER_FALLING,
	.dmic_ref = {ARIZONA_DMIC_MICBIAS1, ARIZONA_DMIC_MICBIAS3, 0, 0},
	.inmode = {ARIZONA_INMODE_SE, ARIZONA_INMODE_DMIC, 0, 0},
	.jd_gpio5 = 0,
	.micd_ranges = micd_ctp_ranges,
	.num_micd_ranges = ARRAY_SIZE(micd_ctp_ranges),
};

void __init *wm5102_platform_data(void *info)
{
	int gpio;
	struct i2c_board_info *i2c_info = (struct i2c_board_info *)info;

	gpio = get_gpio_by_name("gpio_codec_int");
	i2c_info->irq = gpio + INTEL_MID_IRQ_OFFSET;

	return &wm5102_pdata;
}
