/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef CSS2600_REGS_H
#define CSS2600_REGS_H

/* Broxton A0 offsets */

#define CSS2600_MRFLD_DATA_IOMMU_OFFSET			0x00070000
#define CSS2600_MRFLD_ICACHE_IOMMU_OFFSET		0x000a0000

#define CSS2600_BXT_A0_ISYS_IOMMU0_OFFSET		0x000e0000
#define CSS2600_BXT_A0_ISYS_IOMMU1_OFFSET		0x000e0100

#define CSS2600_BXT_A0_ISYS_OFFSET			0x00100000
#define CSS2600_BXT_A0_PSYS_OFFSET			0x00400000

#define CSS2600_BXT_A0_PSYS_IOMMU0_OFFSET		0x003c0000
#define CSS2600_BXT_A0_PSYS_IOMMU1_OFFSET		0x003c0100

/* Broxton A0 PCI config registers */
#define CSS2600_REG_PCI_PCIECAPHDR_PCIECAP		0x70
#define CSS2600_REG_PCI_DEVICECAP			0x74
#define CSS2600_REG_PCI_DEVICECTL_DEVICESTS		0x78
#define CSS2600_REG_PCI_MSI_CAPID			0xac
#define CSS2600_REG_PCI_MSI_ADDRESS_LO			0xb0
#define CSS2600_REG_PCI_MSI_ADDRESS_HI			0xb4
#define CSS2600_REG_PCI_MSI_DATA			0xb8
#define CSS2600_REG_PCI_PMCAP				0xd0
#define CSS2600_REG_PCI_PMCS				0xd4
#define CSS2600_REG_PCI_MANUFACTURING_ID		0xf8
#define CSS2600_REG_PCI_IUNIT_ACCESS_CTRL_VIOL		0xfc

/* Broxton A0 ISYS registers */
/* Isys DMA CIO info register */
#define CSS2600_REG_ISYS_INFO_CIO_DMA0(a)		(0x81810 + (a) * 0x40)
#define CSS2600_REG_ISYS_INFO_CIO_DMA0(a)		(0x81810 + (a) * 0x40)
#define CSS2600_REG_ISYS_INFO_CIO_DMA1(a)		(0x93010 + (a) * 0x40)
#define CSS2600_REG_ISYS_INFO_CIO_DMA_IS(a)		(0xb0610 + (a) * 0x40)
#define CSS2600_ISYS_NUM_OF_DMA0_CHANNELS		16
#define CSS2600_ISYS_NUM_OF_DMA1_CHANNELS		32
#define CSS2600_ISYS_NUM_OF_IS_CHANNELS			4
/*Isys Info register offsets*/
#define CSS2600_REG_ISYS_INFO_SEG_0_CONFIG_ICACHE_MASTER	0x14
#define CSS2600_REG_ISYS_INFO_SEG_CMEM_MASTER(a)	(0x2C + (a * 12))
#define CSS2600_REG_ISYS_INFO_SEG_XMEM_MASTER(a)	(0x5C + (a * 12))

/*Iunit Info bits*/
/*VCO*/
#define CSS2600_INFO_ENABLE_SNOOP			BIT(0)
#define CSS2600_INFO_IMR_DESTINED			BIT(1)
#define CSS2600_INFO_REQUEST_DESTINATION_BUT_REGS	0
#define CSS2600_INFO_REQUEST_DESTINATION_PRIMARY	BIT(4)
#define CSS2600_INFO_REQUEST_DESTINATION_P2P		(BIT(4) | BIT(5))
/*VC1*/
#define CSS2600_INFO_DEADLINE_PTR                      BIT(1)
#define CSS2600_INFO_ZLW                               BIT(2)
#define CSS2600_INFO_STREAM_ID_SET(a)	((a & 0xF) << 4)
#define CSS2600_INFO_ADDRESS_SWIZZ                     BIT(8)

#define CSS2401_PUNIT_PORT				0x04
#define CSS2401_REG_ISPSSPM0				0x39
#define CSS2401_REG_ISPSSPM0_ISPSSS_SHIFT		24
#define CSS2401_REG_ISPSSPM0_ISPSSC_MASK		0x3
#define CSS2401_REG_ISPSSPM0_IUNIT_POWER_ON		0
#define CSS2401_REG_ISPSSPM0_IUNIT_POWER_OFF		3
#define CSS2401_REG_ISPSSPM0_IUNIT_POWER_MASK		0x3

/* PCI config space registers for 2401 */
#define CSS2401_REG_PCI_PMCS				0x84
#define CSS2401_REG_PCI_MSI_CAPID			0x90
#define CSS2401_PCI_MSI_CAPID_MSI_ENABLE_BIT		BIT(16)
#define CSS2401_REG_PCI_MSI_ADDR			0x94
#define CSS2401_REG_PCI_MSI_DATA			0x98
#define CSS2401_REG_PCI_INTERRUPT_CTRL			0x9c
#define CSS2401_PCI_INTERRUPT_CTRL_INTR_IER		BIT(24)
#define CSS2401_PCI_INTERRUPT_CTRL_INTR_IIR		BIT(16)
#define CSS2401_REG_PCI_CSI_ACCESS_CTRL_VIOL		0xd4
#define CSS2401_REG_PCI_CSI_AFE_HS_CONTROL		0xdc
#define CSS2401_REG_PCI_CSI_AFE_RCOMP_CONTROL		0xe0
/*
 * If cleared, the high speed clock going to the digital logic is gated when
 * RCOMP update is happening. The clock is gated for a minimum of 100 nsec.
 * If this bit is set, then the high speed clock is not gated during the
 * update cycle.
 */
#define CSS2401_PCI_CSI_HS_OVR_CLK_GATE_ON_UPDATE	0x800000
#define CSS2401_REG_PCI_CSI_CONTROL			0xe8
#define CSS2401_PCI_CSI_CONTROL_PARPATHEN		BIT(24)
#define CSS2401_REG_PCI_CSI_AFE_TRIM_CONTROL		0xe4
#define CSS2401_REG_PCI_CSI_DEADLINE_CONTROL		0xec
#define CSS2401_REG_PCI_CSI_RCOMP_CONTROL		0xf4
#define CSS2401_REG_PCI_I_CONTROL			0xfc
/*
 * Enables the combining of adjacent 32-byte read requests to the same
 * cache line. When cleared, each 32-byte read request is sent as a
 * separate request on the IB interface.
 */
#define CSS2401_PCI_I_CONTROL_ENABLE_READ_COMBINING	BIT(0)

/*
 * Enables the combining of adjacent 32-byte write requests to the same
 * cache line. When cleared, each 32-byte write request is sent as a
 * separate request on the IB interface.
 */
#define CSS2401_PCI_I_CONTROL_ENABLE_WRITE_COMBINING	BIT(1)

/* Iunit 2401 MMIO registers */
#define CSS2401_REG_CSI_RECEIVER_SELECTION		0x8081c
#define CSS2401_CSI_RECEIVER_SELECTION_ARASAN		0
#define CSS2401_CSI_RECEIVER_SELECTION_INTEL		1

#define CSS2401_REG_IRQ_CTRL_GB				0x00000500
#define CSS2401_REG_IRQ_CTRL_IFMT			0x00030a00
#define CSS2401_REG_IRQ_CTRL_ISYS			0x0008c000
#define CSS2401_REG_IRQ_CTRL_ISEL			0x00090200

#define CSS2401_REG_IRQ_STATUS_OFFSET			0x08
#define CSS2401_REG_IRQ_CLEAR_OFFSET			0x0c

#define CSS2401_REG_IRQ_GB_SHIFT			0
#define CSS2401_REG_IRQ_IFMT_SHIFT			16
#define CSS2401_REG_IRQ_ISYS_SHIFT			14
#define CSS2401_REG_IRQ_ISEL_SHIFT			15

#define CSS2401_REG_NUM_IRQ_GB				32
#define CSS2401_REG_NUM_IRQ_IFMT			5
#define CSS2401_REG_NUM_IRQ_ISYS			19
#define CSS2401_REG_NUM_IRQ_ISEL			4

#endif /* CSS2600_REGS_H */
