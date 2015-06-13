/*
 * platform_mrfl_thermal.c: Platform data initilization file for
 *			Intel Merrifield Platform thermal driver
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/input.h>
#include <linux/kernel.h>
#include <linux/mfd/intel_msic.h>
#include <linux/platform_device.h>
#include <asm/intel_mid_thermal.h>
#include <asm/intel-mid.h>
#include <asm/intel_mid_remoteproc.h>
#include "platform_mrfl_thermal.h"

static struct intel_mid_thermal_sensor mrfl_sensors[] = {
	{
		.name = "SYSTHERM0",
		.index = 0,
	},
	{
		.name = "SYSTHERM1",
		.index = 1,
	},
	{
		.name = "SYSTHERM2",
		.index = 2,
	},
	{
		.name = MSIC_DIE_NAME,
		.direct = true,
		.index = 3,
	},
	{
		.name = "FrontSkin",
	},
	{
		.name = "BackSkin",
	},
};

/* Bodegabay - PRh thermal sensor list */
static struct intel_mid_thermal_sensor bdgb_sensors[] = {
	{
		.name = "SYSTHERM0",
		.index = 0,
	},
	{
		.name = "SYSTHERM1",
		.index = 1,
	},
	{
		.name = "SYSTHERM2",
		.index = 2,
	},
	{
		.name = MSIC_DIE_NAME,
		.direct = true,
		.index = 3,
	},
	{	/*
		 * Weight/Offset & systherm to be used in xml for bodegabay
		 * FrontSkin:slope=410, intercept=16808
		 * Systherm = SYSTHERM0
		 */
		.name = "FrontSkin",
	},
	{	/*
		 * Weight/Offset & systherm to be used in xml for bodegabay
		 * FrontSkin:slope=665, intercept=8375
		 * Systherm = SYSTHERM0
		 */
		.name = "BackSkin",
	},
};

static struct intel_mid_thermal_platform_data pdata[] = {
	[mrfl_thermal] = {
		.num_sensors = 4,
		.sensors = mrfl_sensors,
		.num_virtual_sensors = 2,
	},
	[bdgb_thermal] = {
		.num_sensors = 4,
		.sensors = bdgb_sensors,
		.num_virtual_sensors = 2,
	},
};

void __init *mrfl_thermal_platform_data(void *info)
{
	struct platform_device *pdev;
	struct sfi_device_table_entry *entry = info;

	pdev = platform_device_alloc(MRFL_THERM_DEV_NAME, -1);
	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
			MRFL_THERM_DEV_NAME);
		return NULL;
	}

	if (platform_device_add(pdev)) {
		pr_err("failed to add thermal platform device\n");
		platform_device_put(pdev);
		return NULL;
	}

	if (INTEL_MID_BOARD(2, PHONE, MRFL, BB, ENG) ||
			(INTEL_MID_BOARD(2, PHONE, MRFL, BB, PRO)))
		pdev->dev.platform_data = &pdata[bdgb_thermal];
	else
		pdev->dev.platform_data = &pdata[mrfl_thermal];

	install_irq_resource(pdev, entry->irq);
	register_rpmsg_service("rpmsg_mrfl_thermal", RPROC_SCU,
				RP_BCOVE_THERMAL);

	return 0;
}
