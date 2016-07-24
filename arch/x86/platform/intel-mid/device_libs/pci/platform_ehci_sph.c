
/* platform_ehci_sph_pci.c: USB EHCI/SPH platform data initilization file
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
#include <asm/intel-mid.h>
#include <linux/usb/ehci_sph_pci.h>

static struct ehci_sph_pdata sph_pdata = {
	.has_gpio = 0,
	.gpio_cs_n = 0,
	.gpio_reset_n = 0,
	.enabled = 0
};

static int is_board_ctp_prx(void)
{
	return  INTEL_MID_BOARD(1, PHONE, CLVTP) &&
		(SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR01)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR02)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10PM) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10P)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR10)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR15M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR15)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR30M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR30)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20A)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR19M)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR199M) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR20B)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR30A)  ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR30AM) ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR31)   ||
		 SPID_HARDWARE_ID(CLVTP, PHONE, RHB, PR31M));
}

static struct ehci_sph_pdata *get_sph_platform_data(struct pci_dev *pdev)
{
	struct ehci_sph_pdata *pdata = &sph_pdata;

	switch (pdev->device) {
	case PCI_DEVICE_ID_INTEL_CLV_SPH:
		if (is_board_ctp_prx()) {
			/* request SPH CS_N gpio by name */
			pdata->gpio_cs_n = get_gpio_by_name("sph_phy_cs_n");
			if (pdata->gpio_cs_n == -1) {
				dev_err(&pdev->dev,
					"%s: No SPH gpio "
					"pin sph_phy_cs_n\n", __func__);
				return NULL;
			}

			/* request SPH RST_N gpio by name */
			pdata->gpio_reset_n =
					get_gpio_by_name("sph_phy_reset_n");
			if (pdata->gpio_reset_n == -1) {
				dev_err(&pdev->dev,
					"%s: No SPH gpio "
					"pin sph_phy_reset_n\n", __func__);
				return NULL;
			}
			pdata->has_gpio = 1;
		}
		break;

	default:
		return NULL;
		break;
	}

	return pdata;
}

static void sph_pci_early_quirks(struct pci_dev *pci_dev)
{
	pci_dev->dev.platform_data = get_sph_platform_data(pci_dev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_SPH,
			sph_pci_early_quirks);


static void sph_resume_quirks(struct pci_dev *dev)
{
	struct ehci_sph_pdata	*pdata = NULL;
	int			err;

	pdata = dev->dev.platform_data;

	if (pdata == NULL) {
		dev_err(&dev->dev, "SPH platform data is NULL\n");
		return;
	}

	if (pdata->enabled) {
		dev_info(&dev->dev, "Bypass SPH resume quirk for sph enabled\n");
		return;
	}

	/* Set SPH power state to D1 after resume */
	err = pci_set_power_state(dev, PCI_D1);

	if (err < 0) {
		dev_err(&dev->dev, "set SPH to D1 failed, err = %d\n", err);
		return;
	}
}

DECLARE_PCI_FIXUP_RESUME(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_CLV_SPH,
			sph_resume_quirks);
