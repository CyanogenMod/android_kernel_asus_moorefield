/*
 * platform_pn544.c: pn544 platform data initilization file
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
#include <linux/i2c.h>
#include <linux/nfc/pn544.h>
#include <asm/intel-mid.h>
#include "platform_pn544.h"

void *pn544_platform_data(void *info)
{
	static struct pn544_i2c_platform_data plat;

	plat.irq_gpio = get_gpio_by_name(NFC_HOST_INT_GPIO);
	if (plat.irq_gpio == -1)
		return NULL;
	plat.ven_gpio = get_gpio_by_name(NFC_ENABLE_GPIO);
	if (plat.ven_gpio  == -1)
		return NULL;
	plat.firm_gpio = get_gpio_by_name(NFC_FW_RESET_GPIO);
	if (plat.firm_gpio == -1)
		return NULL;

	/* On MFLD AND CLVT platforms, I2C xfers must be split
	 * to avoid I2C FIFO underrun errors in I2C bus driver */
	switch (intel_mid_identify_cpu()) {
	case INTEL_MID_CPU_CHIP_PENWELL:
		plat.max_i2c_xfer_size = 31;
		break;
	case INTEL_MID_CPU_CHIP_CLOVERVIEW:
		plat.max_i2c_xfer_size = 255;
		break;
	default:
		break;
	}

	return &plat;
}
