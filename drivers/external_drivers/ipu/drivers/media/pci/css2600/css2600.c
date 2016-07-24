/*
 * Copyright (c) 2013--2014 Intel Corporation. All Rights Reserved.
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

#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/firmware.h>
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/pm_qos.h>
#include <linux/pm_runtime.h>
#include <linux/timer.h>

#include "css2600.h"
#include "css2600-pdata.h"
#include "css2600-bus.h"
#include "css2600-regs.h"
#include "css2600-wrapper-2401.h"

#define CSS2600_PCI_BAR		0

struct css2600_dma_mapping;

static struct css2600_bus_device *css2600_mmu_init(
	struct pci_dev *pdev, struct device *parent, void __iomem *base[],
	unsigned int nr_base, unsigned int nr, unsigned int type)
{
	struct css2600_mmu_pdata *pdata =
		devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);
	unsigned int i;

	if (!pdata)
		return ERR_PTR(-ENOMEM);

	BUG_ON(nr_base > CSS2600_MMU_MAX_DEVICES);

	for (i = 0; i < nr_base; i++)
		pdata->base[i] = base[i];

	pdata->nr_base = nr_base;

	pdata->type = type;

	return css2600_bus_add_device(pdev, parent, pdata, NULL,
				      CSS2600_MMU_NAME, nr);
}

static struct css2600_bus_device *css2600_isys_init(
	struct pci_dev *pdev, struct device *parent,
	struct css2600_bus_iommu *iommu, void __iomem *base,
	const struct css2600_isys_internal_pdata *ipdata,
	struct css2600_isys_subdev_pdata *spdata, unsigned int nr,
	unsigned int type)
{
	struct css2600_isys_pdata *pdata =
		devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->base = base;
	pdata->type = type;
	pdata->ipdata = ipdata;
	pdata->spdata = spdata;

	return css2600_bus_add_device(pdev, parent, pdata, iommu,
				      CSS2600_ISYS_NAME, nr);
}

static struct css2600_bus_device *css2600_psys_init(
	struct pci_dev *pdev, struct device *parent,
	struct css2600_bus_iommu *iommu, void __iomem *base,
	unsigned int nr)
{
	struct css2600_psys_pdata *pdata =
		devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->base = base;

	return css2600_bus_add_device(pdev, parent, pdata, iommu,
				      CSS2600_PSYS_NAME, nr);
}

static struct css2600_bus_device *css2600_buttress_init(
	struct pci_dev *pdev, struct device *parent,
	void __iomem *base, unsigned int nr)
{
	struct css2600_buttress_pdata *pdata =
		devm_kzalloc(&pdev->dev, sizeof(*pdata), GFP_KERNEL);

	if (!pdata)
		return ERR_PTR(-ENOMEM);

	pdata->base = base;

	return css2600_bus_add_device(pdev, parent, pdata, NULL,
				      CSS2600_BUTTRESS_NAME, nr);
}

static inline void css2600_call_isr(struct css2600_bus_device *adev)
{
	if (!adev || !adev->adrv || !adev->adrv->isr)
		return;

	adev->adrv->isr(adev);
}

static struct css2600_irq_block_2401 {
	uint32_t irq_enable_shift;
	uint32_t num_irqs;
	uint32_t irq_base;
} css2401_irqs[] = {
	{
		CSS2401_REG_IRQ_IFMT_SHIFT,
		CSS2401_REG_NUM_IRQ_IFMT,
		CSS2401_REG_IRQ_CTRL_IFMT,
	},
	{
		CSS2401_REG_IRQ_ISYS_SHIFT,
		CSS2401_REG_NUM_IRQ_ISYS,
		CSS2401_REG_IRQ_CTRL_ISYS,
	},
	{
		CSS2401_REG_IRQ_ISEL_SHIFT,
		CSS2401_REG_NUM_IRQ_ISEL,
		CSS2401_REG_IRQ_CTRL_ISEL,
	},
};

static void css2600_clear_irq_2401(struct css2600_device *isp)
{
	uint32_t status = readl(isp->base + CSS2401_REG_IRQ_CTRL_GB +
				CSS2401_REG_IRQ_STATUS_OFFSET);
	int i;

	if (ffs(status) >= CSS2401_REG_NUM_IRQ_GB)
		return;

	for (i = 0; i < ARRAY_SIZE(css2401_irqs); i++) {
		uint32_t status_sub;

		if (!(status & (1 << css2401_irqs[i].irq_enable_shift)))
			continue;

		status_sub = readl(isp->base + css2401_irqs[i].irq_base +
				   CSS2401_REG_IRQ_STATUS_OFFSET);
		writel(status_sub, isp->base + css2401_irqs[i].irq_base +
		       CSS2401_REG_IRQ_CLEAR_OFFSET);
		status &= ~(1 << css2401_irqs[i].irq_enable_shift);
	}

	writel(status, isp->base + CSS2401_REG_IRQ_CTRL_GB +
	       CSS2401_REG_IRQ_CLEAR_OFFSET);
}

static int css2600_isr_2401_one(struct css2600_device *isp)
{
#if IS_ENABLED(CONFIG_VIDEO_CSS2600_2401)
	enum ia_css_fwctrl_event_type event;
	int rval = -ia_css_fwctrl_dequeue_event(&event);

	if (rval)
		return rval;

	switch (event) {
	default:
		dev_dbg(&isp->pdev->dev, "unknown event type %u\n",
			event);
		break;
	case IA_CSS_FWCTRL_NO_EVENT:
		dev_dbg(&isp->pdev->dev, "no interrupt source\n");
		break;
	case IA_CSS_FWCTRL_ISYS_EVENT:
		dev_dbg(&isp->pdev->dev, "interrupt source isys\n");
		css2600_call_isr(isp->isys);
		break;
	case IA_CSS_FWCTRL_PSYS_EVENT:
		dev_dbg(&isp->pdev->dev, "interrupt source psys\n");
		css2600_call_isr(isp->psys);
		break;
	}
#endif /* IS_ENABLED(CONFIG_VIDEO_CSS2600_2401) */

	return 0;
}

irqreturn_t css2600_isr_thread(int irq, void *isp_ptr)
{
	struct css2600_device *isp = isp_ptr;

	switch (isp->pdev->device) {
	case CSS2600_HW_MRFLD_2401:
		while (!css2600_isr_2401_one(isp));
		break;
	}
	return IRQ_HANDLED;
}

static irqreturn_t css2600_isr(int irq, void *priv)
{
	struct css2600_device *isp = priv;

	switch (isp->pdev->device) {
	case CSS2600_HW_BXT_FPGA:
	case CSS2600_HW_BXT:
		if (IS_ENABLED(VIDEO_CSS2600_ISYS))
			css2600_call_isr(isp->isys);
		if (IS_ENABLED(VIDEO_CSS2600_PSYS))
			css2600_call_isr(isp->psys);
		break;
	case CSS2600_HW_MRFLD_2401: {
		u32 val;

		css2600_clear_irq_2401(isp);

		pci_read_config_dword(isp->pdev,
				      CSS2401_REG_PCI_INTERRUPT_CTRL, &val);
		val |= CSS2401_PCI_INTERRUPT_CTRL_INTR_IIR;
		pci_write_config_dword(isp->pdev,
				       CSS2401_REG_PCI_INTERRUPT_CTRL, val);

		return IRQ_WAKE_THREAD;
	}
	}

	return IRQ_HANDLED;
}

int css2600_pci_config_setup(struct pci_dev *dev, bool enable_msi)
{
	u16 pci_command;
	u32 val32;

	switch (dev->device) {
	case CSS2600_HW_MRFLD_2401:
		pci_read_config_dword(dev, CSS2401_REG_PCI_MSI_CAPID, &val32);
		pci_write_config_dword(dev, CSS2401_REG_PCI_MSI_CAPID,
				       val32 |
				       CSS2401_PCI_MSI_CAPID_MSI_ENABLE_BIT);

		pci_write_config_dword(dev, CSS2401_REG_PCI_INTERRUPT_CTRL,
				       CSS2401_PCI_INTERRUPT_CTRL_INTR_IIR |
				       CSS2401_PCI_INTERRUPT_CTRL_INTR_IER);
		break;
	}

	pci_read_config_word(dev, PCI_COMMAND, &pci_command);
	pci_command |= PCI_COMMAND_MEMORY | PCI_COMMAND_MASTER;
	if (enable_msi) {
		int rval = pci_enable_msi(dev);
		if (rval) {
			dev_err(&dev->dev, "Failed to enable msi (%d)\n", rval);
			return rval;
		}
		pci_command |= PCI_COMMAND_INTX_DISABLE;
	} else {
		pci_command &= ~PCI_COMMAND_INTX_DISABLE;
	}
	pci_write_config_word(dev, PCI_COMMAND, pci_command);

	return 0;
}

static const struct css2600_isys_internal_pdata isys_ipdata_2600 = {
	.csi2 = {
		.nports = 4,
		.nlanes = { 4, 1, 2, 2 },
	},
	.tpg = {
		.ntpgs = 1,
		.offsets = { 0x79200 },
	},
	.isa = {
		.offset = 0x10000,
	},
};

static const struct css2600_isys_internal_pdata isys_ipdata_2401 = {
	.csi2 = {
		.nports = 3,
		.nlanes = { 4, 1, 2 },
		.offsets = { 0x000c0400, 0x000c2400, 0x000c4400 },
	},
};

static int css2600_pci_probe(struct pci_dev *pdev,
			     const struct pci_device_id *id)
{
	struct css2600_device *isp;
	phys_addr_t phys;
	void __iomem *mmu_base[CSS2600_MMU_MAX_DEVICES];
	struct css2600_bus_iommu *isys_iommu, *psys_iommu;
	void __iomem * const *iomap;
	void __iomem *isys_base = NULL;
	void __iomem *psys_base = NULL;
	char *fw_binary;
	unsigned int iommus = 0;
	unsigned int dma_mask = 39;
	int rval;

	isp = devm_kzalloc(&pdev->dev, sizeof(*isp), GFP_KERNEL);
	if (!isp) {
		dev_err(&pdev->dev, "Failed to alloc CI ISP structure\n");
		return -ENOMEM;
	}
	isp->pdev = pdev;
	INIT_LIST_HEAD(&isp->devices);

	isys_iommu = devm_kzalloc(&pdev->dev, sizeof(*isys_iommu), GFP_KERNEL);
	psys_iommu = devm_kzalloc(&pdev->dev, sizeof(*psys_iommu), GFP_KERNEL);
	if (!isys_iommu || !psys_iommu) {
		dev_err(&pdev->dev, "Can't allocate memory for iommu\n");
		return -ENOMEM;
	}

	/* Share IOMMU mapping between isys and psys */
	isys_iommu->m = psys_iommu->m = devm_kzalloc(
		&pdev->dev, sizeof(*isys_iommu->m), GFP_KERNEL);
	if (!isys_iommu->m) {
		dev_err(&pdev->dev,
			"Can't allocate memory for iommu mapping\n");
		return -ENOMEM;
	}

	rval = pcim_enable_device(pdev);
	if (rval) {
		dev_err(&pdev->dev, "Failed to enable CI ISP device (%d)\n",
			rval);
		return rval;
	}

	phys = pci_resource_start(pdev, CSS2600_PCI_BAR);

	rval = pcim_iomap_regions(pdev, 1 << CSS2600_PCI_BAR, pci_name(pdev));
	if (rval) {
		dev_err(&pdev->dev, "Failed to I/O memory remapping (%d)\n",
			rval);
		return rval;
	}
	dev_info(&pdev->dev, "physical base address 0x%llx\n", phys);

	iomap = pcim_iomap_table(pdev);
	if (!iomap) {
		dev_err(&pdev->dev, "Failed to iomap table (%d)\n", rval);
		return -ENODEV;
	}

	isp->base = iomap[CSS2600_PCI_BAR];
	dev_info(&pdev->dev, "mapped as: 0x%p\n", isp->base);

	pci_set_drvdata(pdev, isp);

	pci_set_master(pdev);

	switch (isp->pdev->device) {
	case CSS2600_HW_MRFLD_2401:
		psys_base = isp->base;
		isys_base = isp->base;
		fw_binary = CSS2401_FIRMWARE;
		break;
	case CSS2600_HW_BXT_FPGA:
		dma_mask = 32;
		/* fall through */
	case CSS2600_HW_BXT:
		psys_base = isp->base + CSS2600_BXT_A0_PSYS_OFFSET;
		isys_base = isp->base + CSS2600_BXT_A0_ISYS_OFFSET;
		fw_binary = NULL;
		break;
	default:
		dev_err(&pdev->dev, "Not supported device\n");
		return -EINVAL;
	}

	rval = pci_set_dma_mask(pdev, DMA_BIT_MASK(dma_mask));
	if (!rval)
		rval = pci_set_consistent_dma_mask(pdev,
						   DMA_BIT_MASK(dma_mask));
	if (rval) {
		dev_err(&pdev->dev, "Failed to set DMA mask (%d)\n", rval);
		return rval;
	}

	if (fw_binary)
		rval = request_firmware(&isp->fw, fw_binary, &pdev->dev);
	if (rval) {
		dev_err(&pdev->dev, "Requesting firmware failed\n");
		return rval;
	}
	css2600_wrapper_init(psys_base, isys_base, isp->fw);

	if (pdev->device == CSS2600_HW_BXT) {
		isp->buttress = css2600_buttress_init(pdev, &pdev->dev, isp->base, 0);
		rval = PTR_ERR(isp->buttress);
		if (IS_ERR(isp->buttress))
			goto out_css2600_bus_del_devices;
	}
	if (pdev->device == CSS2600_HW_BXT
	    || (pdev->device == CSS2600_HW_BXT_FPGA
		&& IS_ENABLED(CONFIG_VIDEO_CSS2600_ISYS))) {
		mmu_base[0] = isys_base + CSS2600_BXT_A0_ISYS_IOMMU0_OFFSET;
		mmu_base[1] = isys_base + CSS2600_BXT_A0_ISYS_IOMMU1_OFFSET;
		isp->isys_iommu = css2600_mmu_init(pdev, &isp->buttress->dev,
			mmu_base, 2, 0, CSS2600_MMU_TYPE_CSS2600);
		rval = PTR_ERR(isp->isys_iommu);
		if (IS_ERR(isp->isys_iommu)) {
			dev_err(&pdev->dev, "can't create isys iommu device\n");
			return -ENOMEM;
		}

		rval = css2600_isys_iomem_filters_add(&pdev->dev, mmu_base, 2, 0xc);
		if (rval)
			goto out_css2600_bus_del_devices;
		iommus++;

		isys_iommu->dev = &isp->isys_iommu->dev;

		isp->isys = css2600_isys_init(
			pdev, &isp->buttress->dev, isys_iommu, isys_base,
			&isys_ipdata_2600, pdev->dev.platform_data, 0,
			pdev->device == CSS2600_HW_BXT ?
			CSS2600_ISYS_TYPE_CSS2600 :
			CSS2600_ISYS_TYPE_CSS2600_FPGA);
		rval = PTR_ERR(isp->isys);
		if (IS_ERR(isp->isys))
			goto out_css2600_bus_del_devices;
	}

	if (pdev->device == CSS2600_HW_BXT
	    || (pdev->device == CSS2600_HW_BXT_FPGA
		&& IS_ENABLED(CONFIG_VIDEO_CSS2600_PSYS))) {
		mmu_base[0] = psys_base + CSS2600_BXT_A0_PSYS_IOMMU0_OFFSET;
		mmu_base[1] = psys_base + CSS2600_BXT_A0_PSYS_IOMMU1_OFFSET;
		isp->psys_iommu = css2600_mmu_init(pdev, &isp->isys->dev,
			mmu_base, 2, 1, CSS2600_MMU_TYPE_CSS2600);
		rval = PTR_ERR(isp->psys_iommu);
		if (IS_ERR(isp->psys_iommu)) {
			dev_err(&pdev->dev, "can't create psys iommu device\n");
			goto out_css2600_bus_del_devices;
		}

		rval = css2600_isys_iomem_filters_add(&pdev->dev, mmu_base, 2, 0xc);
		if (rval)
			goto out_css2600_bus_del_devices;
		iommus++;

		psys_iommu->dev = &isp->psys_iommu->dev;
		isp->psys = css2600_psys_init(pdev, &isp->buttress->dev, psys_iommu, psys_base, 0);
		rval = PTR_ERR(isp->isys);
		if (IS_ERR(isp->isys))
			goto out_css2600_bus_del_devices;
	}

	if (pdev->device == CSS2600_HW_MRFLD_2401) {
		u32 val;

		mmu_base[0] = isp->base + CSS2600_MRFLD_DATA_IOMMU_OFFSET;
		mmu_base[1] = isp->base + CSS2600_MRFLD_ICACHE_IOMMU_OFFSET;
		isp->isys_iommu = css2600_mmu_init(pdev, &pdev->dev, mmu_base,
			2, 0, CSS2600_MMU_TYPE_CSS2401);
		rval = PTR_ERR(isp->isys_iommu);
		if (IS_ERR(isp->isys_iommu)) {
			dev_err(&pdev->dev, "can't create iommu device\n");
			goto out_css2600_bus_del_devices;
		}

		rval = css2600_isys_iomem_filters_add(&pdev->dev, mmu_base, 2, 0xc);
		if (rval)
			goto out_css2600_bus_del_devices;
		iommus++;

		isys_iommu->dev = &isp->isys_iommu->dev;
		isp->isys = css2600_isys_init(
			pdev, &pdev->dev, isys_iommu, isp->base,
			&isys_ipdata_2401, pdev->dev.platform_data, 0,
			CSS2600_ISYS_TYPE_CSS2401);
		rval = PTR_ERR(isp->isys);
		if (rval < 0)
			goto out_css2600_bus_del_devices;
		isp->psys = css2600_psys_init(pdev, &isp->isys->dev, isys_iommu,
					      isp->base, 0);
		rval = PTR_ERR(isp->psys);
		if (rval < 0)
			goto out_css2600_bus_del_devices;

		writel(CSS2401_CSI_RECEIVER_SELECTION_INTEL,
		       isp->base + CSS2401_REG_CSI_RECEIVER_SELECTION);

		/* Ensure read/write combining is enabled. */
		pci_read_config_dword(pdev, CSS2401_REG_PCI_I_CONTROL, &val);
		val |= CSS2401_PCI_I_CONTROL_ENABLE_READ_COMBINING |
			CSS2401_PCI_I_CONTROL_ENABLE_WRITE_COMBINING;
		pci_write_config_dword(pdev, CSS2401_REG_PCI_I_CONTROL, val);

		/*
		 * Hardware bugs require setting CSI_HS_OVR_CLK_GATE_ON_UPDATE.
		 * ANN/CHV: RCOMP updates do not happen when using CSI2+ path
		 * and sensor sending "continuous clock".
		 * TNG/ANN/CHV: MIPI packets are lost if the HS entry sequence
		 * is missed, and IUNIT can hang.
		 * For both issues, setting this bit is a workaround.
		 */
		pci_read_config_dword(pdev, CSS2401_REG_PCI_CSI_RCOMP_CONTROL,
				      &val);
		val |= CSS2401_PCI_CSI_HS_OVR_CLK_GATE_ON_UPDATE;
		pci_write_config_dword(pdev, CSS2401_REG_PCI_CSI_RCOMP_CONTROL,
				       val);

		pci_read_config_dword(pdev, CSS2401_REG_PCI_CSI_CONTROL, &val);
		val |= CSS2401_PCI_CSI_CONTROL_PARPATHEN;
		pci_write_config_dword(pdev, CSS2401_REG_PCI_CSI_CONTROL, val);
	}

	rval = css2600_pci_config_setup(pdev,
					pdev->device != CSS2600_HW_BXT_FPGA);
	if (rval)
		goto out_css2600_bus_del_devices;

	rval = css2600_wrapper_set_iommus(iommus);
	if (rval)
		goto out_css2600_bus_del_devices;

	rval = css2600_wrapper_set_device(&isp->isys->dev);
	if (rval)
		goto out_css2600_bus_del_devices;

	rval = devm_request_threaded_irq(&pdev->dev, pdev->irq,	css2600_isr,
					 css2600_isr_thread, IRQF_SHARED,
					 CSS2600_NAME, isp);
	if (rval) {
		dev_err(&pdev->dev, "Requesting threaded irq failed(%d)\n",
			rval);
		goto out_css2600_bus_del_devices;
	}

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;

out_css2600_bus_del_devices:
	css2600_isys_iomem_filter_remove(&pdev->dev);
	css2600_bus_del_devices(pdev);
	release_firmware(isp->fw);

	return rval;
}

static void css2600_pci_remove(struct pci_dev *pdev)
{
	struct css2600_device *isp = pci_get_drvdata(pdev);

	css2600_isys_iomem_filter_remove(&pdev->dev);
	css2600_bus_del_devices(pdev);

	pm_runtime_forbid(&pdev->dev);
	pm_runtime_get_noresume(&pdev->dev);

	free_irq(pdev->irq, &pdev->dev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);

	release_firmware(isp->fw);
}

static DEFINE_PCI_DEVICE_TABLE(css2600_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, CSS2600_HW_BXT_FPGA)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, CSS2600_HW_BXT)},
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, CSS2600_HW_MRFLD_2401)},
	{0,}
};

MODULE_DEVICE_TABLE(pci, css2600_pci_tbl);

static struct pci_driver css2600_pci_driver = {
	.driver = {
#ifdef CONFIG_PM
		.pm = &(const struct dev_pm_ops){},
#endif /* CONFIG_PM */
	},
	.name = CSS2600_NAME,
	.id_table = css2600_pci_tbl,
	.probe = css2600_pci_probe,
	.remove = css2600_pci_remove,
};

static int __init css2600_init(void)
{
	int rval = css2600_bus_register();
	if (rval) {
		pr_warn("can't register css2600 bus (%d)\n", rval);
		return rval;
	}

	rval = pci_register_driver(&css2600_pci_driver);
	if (rval) {
		pr_warn("can't register pci driver (%d)\n", rval);
		goto out_pci_register_driver;
	}

	return 0;

out_pci_register_driver:
	css2600_bus_unregister();

	return rval;
}

static void __exit css2600_exit(void)
{
	pci_unregister_driver(&css2600_pci_driver);
	css2600_bus_unregister();
}

module_init(css2600_init);
module_exit(css2600_exit);

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@linux.intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel css2600 pci driver");
