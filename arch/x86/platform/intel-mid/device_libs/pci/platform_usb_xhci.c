
/* platform_usb_xhci.c: USB XHCI platform quirk file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/pci.h>
#include <linux/pm_runtime.h>
#include <asm/intel-mid.h>

static void xhci_pci_early_quirks(struct pci_dev *pci_dev)
{
	dev_dbg(&pci_dev->dev, "set run wake flag\n");
	device_set_run_wake(&pci_dev->dev, true);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_USH,
			xhci_pci_early_quirks);

static void quirk_byt_ush_d3_delay(struct pci_dev *dev)
{
	dev->d3_delay = 10;
}
DECLARE_PCI_FIXUP_ENABLE(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_BYT_USH,
			quirk_byt_ush_d3_delay);
