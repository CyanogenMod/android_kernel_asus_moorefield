/*
 * This driver provides interface for buggy IPs
 * Copyright (c) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 */
#include "intel_soc_pci_acpi_quirk.h"

#define PCI_IUNIT_CSI_CONTROL_CHT	0xe8

bool acpi_pci_quirk_power_manageable(struct pci_dev *dev)
{
	/*
	 * FIXME : A h/w bug causes the system hang if access
	 * is made after I-unit is powered off.
	 */
	return (dev->vendor == PCI_VENDOR_ID_INTEL && (
			dev->device == PCI_DEVICE_ID_IUNIT_CHT ||
			dev->device == PCI_DEVICE_ID_IUNIT_BYT));
}
EXPORT_SYMBOL(acpi_pci_quirk_power_manageable);

pci_power_t acpi_pci_quirk_choose_state(struct pci_dev *pdev)
{
	if (!acpi_pci_quirk_power_manageable(pdev))
		return PCI_POWER_ERROR;

	/*
	 * Use D3cold for ATOMISP as after power off, kernel
	 * wouldn't access PCI space. This is to w/a for the
	 * above stated bug..
	 */
	return PCI_D3cold;
}
EXPORT_SYMBOL(acpi_pci_quirk_choose_state);

static bool is_cht_iunit(struct pci_dev *dev)
{
	return (dev->vendor == PCI_VENDOR_ID_INTEL &&
		dev->device == PCI_DEVICE_ID_IUNIT_CHT);

}

int acpi_pci_quirk_set_state(struct pci_dev *dev, pci_power_t state)
{
	int data, error = -EINVAL;

	if (!acpi_pci_quirk_power_manageable(dev))
		return PCI_POWER_ERROR;

	/* FIXME: make it generic to address other islands if needed */
	switch (state) {
	case PCI_D0:
		error = pmc_nc_set_power_state(ISP_PWR_ISLAND,
				OSPM_ISLAND_UP, 0x39);
		/*
		 * Workaround for CHT HSD 4797062
		 * after IUNIT powers up, do following immediately:
		 * 1. driver should issue a PCI read cycle to IUNIT pci
		 * config space.
		 * 2. driver should enable camera ports
		 * (clear [2:0] of port 0x1c, offset 0xe8)
		 */
		if (is_cht_iunit(dev) && !error) {
			pci_read_config_dword(dev, PCI_IUNIT_CSI_CONTROL_CHT,
					      &data);
			data &= ~(0x7);
			pci_write_config_dword(dev, PCI_IUNIT_CSI_CONTROL_CHT,
					       data);
		}
		break;

	case PCI_D1:
	case PCI_D2:
	case PCI_D3cold:
		if (dev_pm_qos_flags(&dev->dev, PM_QOS_FLAG_NO_POWER_OFF) ==
			PM_QOS_FLAGS_ALL) {
			error = -EBUSY;
			break;
		}

	case PCI_D3hot:
		/*
		 * Workaround for CHT HSD 4797062
		 * before IUNIT powers down, do following immediately
		 * driver should disable camera ports
		 * (set [2:0] of port 0x1c, offset 0xe8).
		 */
		if (is_cht_iunit(dev)) {
			pci_read_config_dword(dev, PCI_IUNIT_CSI_CONTROL_CHT, &data);
			data |= 0x7;
			pci_write_config_dword(dev, PCI_IUNIT_CSI_CONTROL_CHT, data);
		}
		error = pmc_nc_set_power_state(ISP_PWR_ISLAND,
				OSPM_ISLAND_DOWN, 0x39);
		break;
	}

	return error;
}
EXPORT_SYMBOL(acpi_pci_quirk_set_state);
