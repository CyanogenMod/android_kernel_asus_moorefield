/*
 * platform_lm3642.c: lm3642 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/types.h>
#include <asm/intel-mid.h>

#include <media/lm3642.h>

#include "platform_lm3642.h"
#include "platform_camera.h"

void *lm3642_platform_data_func(void *info)
{
	static struct lm3642_platform_data platform_data;

	if (!IS_BYT) {
		platform_data.gpio_strobe = get_gpio_by_name("GP_FLASH_STROBE");
		platform_data.gpio_torch  = get_gpio_by_name("GP_FLASH_TORCH");
	} else {
		platform_data.gpio_strobe = 121;
		platform_data.gpio_torch  = 122;
	}

	if (platform_data.gpio_strobe == -1) {
		pr_err("%s: Unable to find GP_FLASH_STROBE\n", __func__);
		return NULL;
	}
	if (platform_data.gpio_torch == -1) {
		pr_err("%s: Unable to find GP_FLASH_TORCH\n", __func__);
		return NULL;
	}

	pr_info("camera pdata: lm3642: strobe %d torch %d\n",
		platform_data.gpio_strobe, platform_data.gpio_torch);

	platform_data.torch_en = 0;
	platform_data.flash_en = 0;
	platform_data.tx_en = 0;
	platform_data.ivfm_en = 0;

	return &platform_data;
}
