/*
 * plattform_mid_pwmbl.c: PWM Backlight driver platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:Arda Coskunses <arda.coskunses@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/pci.h>
#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <asm/spid.h>
#include <linux/intel_pwm_world_led.h>

static struct world_led_pdata mrfld_world_led_pci_data = {
	.time_divisor = 0x40,
	.base_unit = 0x80,
	.alt_fn = 1,
	.ext_drv = 1,
	.gpio_pwm = WORLD_LED_PWM_GPIO,
	.name = "pwm",
};

static struct world_led_pdata *get_world_led_platform_data(struct pci_dev *pdev)
{
	struct world_led_pdata *pdata = NULL;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_WORLD_LED:
		pdata = &mrfld_world_led_pci_data;
		if (pdata != NULL)
			pdata->gpio_pwm = get_gpio_by_name("world_led_pwm");

		break;
	default:
		break;
	}

	return pdata;
}

static void world_led_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_world_led_platform_data(pci_dev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_WORLD_LED,
			world_led_pci_early_quirks);
