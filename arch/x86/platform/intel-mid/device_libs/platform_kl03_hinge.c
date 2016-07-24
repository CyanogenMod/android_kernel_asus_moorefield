/*
 * platform_kl03_hinge.c: kl03 hinge platform data initilization file
 *
 * (C) Copyright 2015 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel-mid.h>
#include <linux/kl03_hinge.h>
#include "platform_kl03_hinge.h"

void *kl03_hinge_platform_data(void *info)
{
	static struct hinge_platform_data platform_data;

	platform_data.irq_hinge_gpio = get_gpio_by_name(KL03_HINGE_HINGE_GPIO);
	platform_data.irq_photo_gpio = get_gpio_by_name(KL03_HINGE_PHOTO_GPIO);
	platform_data.irq_preview_gpio = get_gpio_by_name(KL03_HINGE_PREVIEW_GPIO);
	platform_data.bootcfg_gpio = get_gpio_by_name(KL03_HINGE_BOOTCFG_GPIO);
	platform_data.reset_gpio = get_gpio_by_name(KL03_HINGE_RESET_GPIO);

	/* signal polarity of bootcfg and reset : active low */
	platform_data.active_polarity = 0;

	return &platform_data;
}

