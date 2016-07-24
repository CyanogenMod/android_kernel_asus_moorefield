/*
 * vlv2.c: Intel ValleyView2 platform specific setup code
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Bin Gao <bin.gao@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/pci.h>
#include <asm/setup.h>
#include <asm/time.h>
#include <asm/i8259.h>
#include <asm/intel-mid.h>
#include <asm/processor.h>
#include <asm/pci_x86.h>
#include <linux/delay.h>

static int vlv2_pci_enable_irq(struct pci_dev *pdev)
{
	struct io_apic_irq_attr irq_attr;

	irq_attr.ioapic = mp_find_ioapic(pdev->irq);
	if (irq_attr.ioapic < 0) {
		dev_warn(&pdev->dev, "VLV2: no IOAPIC found for IRQ %d\n", pdev->irq);
		return -ENODEV;
	}
	irq_attr.ioapic_pin = pdev->irq;
	irq_attr.trigger = 1; /* level */
	irq_attr.polarity = 1; /* active low */
	io_apic_set_pci_routing(&pdev->dev, pdev->irq, &irq_attr);

	return 0;
}

#define VALLEYVIEW2_FAMILY	0x30670
#define CPUID_MASK		0xffff0
enum cpuid_regs {
	CR_EAX = 0,
	CR_ECX,
	CR_EDX,
	CR_EBX
};

static int is_valleyview(void)
{
	u32 regs[4];
	cpuid(1, &regs[CR_EAX], &regs[CR_EBX], &regs[CR_ECX], &regs[CR_EDX]);

	return ((regs[CR_EAX] & CPUID_MASK) == VALLEYVIEW2_FAMILY);
}

/*
 * ACPI DSDT table doesn't have correct PCI interrupt routing information
 * for some devices, so here we have a kernel workaround to fix this issue.
 * The workaround can be easily removed by not appending "vlv2" paramerter
 * to kernel command line.
 */
static int __init vlv2_pci_irq_fixup(void)
{
	/* HACK: intel_mid_identify_cpu() is not set yet */

	/* More hack: do cpuid instruction. This will be done here (helper is_valleyview) for now
	 * as temp w/a until reworked function is provided from Bin Gao.
	 */
	if (is_valleyview()) {
		pr_info("VLV2: fix up PCI interrupt routing\n");
		pcibios_enable_irq = vlv2_pci_enable_irq;
	}

	return 0;
}
fs_initcall(vlv2_pci_irq_fixup);

/*
 * REVERTME
 * On BYT-all, FDK & EDK2, we faced a regression linked to a Punit contribution
 * for Windows. Punit FW version 0x0C. (TNTE)
 * The regression is tracked by BZs 185823, 180195, 181921.
 * Here this is a WA to warn Punit Android is running
 * (and ask it to disable the problematic contribution)
 */

#define PUNIT_PORT 0x04
#define BIOS_CONFIG 0x06

static int __init vlv2_warn_punit_os_is_android(void)
{
	if (is_valleyview()) {
		u32 punit_bios_config_value;
		int retry = 0;

		pr_info("VLV2: warn punit OS is Android\n");

		/* Bits 27:18 from PUNIT BIOS_CONFIG register are unused, using bit 21 as
		 * "Android OS is running" information */
		punit_bios_config_value = intel_mid_msgbus_read32(PUNIT_PORT, BIOS_CONFIG);
		punit_bios_config_value |= (1 << 21);
		intel_mid_msgbus_write32(PUNIT_PORT, BIOS_CONFIG, punit_bios_config_value);
		punit_bios_config_value = intel_mid_msgbus_read32(PUNIT_PORT, BIOS_CONFIG);
		while ((retry < 10) && (!(punit_bios_config_value & (1 << 21)))) {
			msleep(1);
			punit_bios_config_value =
				intel_mid_msgbus_read32(PUNIT_PORT, BIOS_CONFIG);
			retry++;
		}
		if (!(punit_bios_config_value & (1 << 21))) {
			pr_err("Punit Bios config register update failed\n");
			return -EIO;
		} else
			pr_info("Informing PUnit we are running Android\n");
		return 0;
	}
	return 0;
}
late_initcall(vlv2_warn_punit_os_is_android);


