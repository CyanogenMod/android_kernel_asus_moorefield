/*
 * platform_msic_vdd.c: MSIC VDD platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Chaurasia, Avinash K <avinash.k.chaurasia@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/scatterlist.h>
#include <linux/sfi.h>
#include <linux/init.h>
#include <linux/gpio.h>
#include <linux/mfd/intel_msic.h>
#include <asm/intel-mid.h>
#include "platform_msic.h"
#include "platform_msic_vdd.h"

#define BCUIRQ 0x24

/* to correct annotation for byt */
#ifdef CONFIG_ACPI
void *msic_vdd_platform_data(void *info)
#else
void __init *msic_vdd_platform_data(void *info)
#endif
{
	static struct intel_msic_vdd_pdata msic_vdd_pdata;
	struct platform_device *pdev = NULL;
	struct sfi_device_table_entry *entry = info;
	int ret = 0;

	pdev = platform_device_alloc(MSIC_VDD_DEV_NAME, -1);

	if (!pdev) {
		pr_err("out of memory for SFI platform dev %s\n",
					MSIC_VDD_DEV_NAME);
		goto out;
	}

	msic_vdd_pdata.msi = BCUIRQ;
	msic_vdd_pdata.disable_unused_comparator = false;
	msic_vdd_pdata.is_clvp = false;

	/* Disabling VCRIT and VWARNB for clvp */
	if (INTEL_MID_BOARD(1, PHONE, CLVTP)) {
		msic_vdd_pdata.disable_unused_comparator =
			 DISABLE_VCRIT;
		msic_vdd_pdata.is_clvp = true;
	}
	pdev->dev.platform_data = &msic_vdd_pdata;

	/* Disable BCU actions for BYT_CR_V2 */
	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, CRV2) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, CRV2))
		msic_vdd_pdata.disable_unused_comparator =
		DISABLE_VCRIT | DISABLE_VWARNB | DISABLE_VWARNA;

	ret = platform_device_add(pdev);
	if (ret) {
		pr_err("failed to add %s platform device\n",
					MSIC_VDD_DEV_NAME);
		platform_device_put(pdev);
		goto out;
	}
	/* is needed only for CTP only */
	if (msic_vdd_pdata.is_clvp)
		install_irq_resource(pdev, entry->irq);
out:
	return &msic_vdd_pdata;
}
