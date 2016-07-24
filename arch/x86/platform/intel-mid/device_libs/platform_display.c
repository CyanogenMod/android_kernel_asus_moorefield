/*
 * platform_display.c: put platform display configuration in
 * this file. If any platform level display related configuration
 * has to be made, then that configuration shoule be in this
 * file.
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/i2c.h>
#include <linux/printk.h>
#include <linux/platform_data/lp855x.h>
#include <asm/spid.h>

static struct i2c_board_info __initdata lp8556_i2c_device = {
	I2C_BOARD_INFO("lp8556", 0x2C),
};

#define LP8556_MODE_SL_2MS_FL_HV_PWM_12BIT     0x3E
#define LP8556_FAST_CONFIG     BIT(7) /* use it if EPROMs should be maintained
					when exiting the low power mode */

struct lp855x_rom_data lp8556_rom_data[] = {
		{ LP8556_CFG3, LP8556_MODE_SL_2MS_FL_HV_PWM_12BIT }
};

struct lp855x_platform_data platform_data = {
	.name = "lp8556",
	.device_control = LP8556_FAST_CONFIG,
	.initial_brightness = 0,
	.period_ns = 5000000, /* 200 Hz */
	.size_program = ARRAY_SIZE(lp8556_rom_data),
	.rom_data = lp8556_rom_data,
};

void *lp8556_get_platform_data(void)
{

	return (void *)&platform_data;
}

static int __init platform_display_module_init(void)
{

	lp8556_i2c_device.platform_data = lp8556_get_platform_data();

	if (lp8556_i2c_device.platform_data == NULL) {
		pr_debug("failed to get platform data for lp8556.");
		return -EINVAL;
	}

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR0) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR1))
		return i2c_register_board_info(3, &lp8556_i2c_device, 1);

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, CRV2) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, CRV2))
		return i2c_register_board_info(4, &lp8556_i2c_device, 1);


	return -EPERM;
}

rootfs_initcall(platform_display_module_init);

