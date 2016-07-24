/*
 * Intel LPSS DMA Controller Driver
 *
 * Copyright (C) 2014, Intel Corporation
 * Authors: Huiquan Zhong <huiquan.zhong@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/spinlock.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/pci.h>
#include <acpi/acpi_bus.h>

#include "lpss_dma.h"

#define LPSS_DMA_MAX_TBL	5
#define LPSS_DMA_REG_SIZE	0x400

enum lpss_dma_id_t {
	byt_lpss_dma_0 = 0,
	byt_lpss_dma_1,
	byt_lpss_dma_2,
	cht_lpss_dma_0 = byt_lpss_dma_0,
};

struct dma_slave_cfg {
	int	dma_type;
	int	ch_base;
};

struct dma_ctl_info {
	struct device		*pdev;
	bool			active;
	int			ch_num;
	unsigned int		irq;
	void __iomem		*membase;
	struct dma_slave_cfg	tbl[LPSS_DMA_MAX_TBL];
};

static struct dma_ctl_info dma_ctl_infos[] = {
	[byt_lpss_dma_1] = {
		.active = false,
		.ch_num = 6,
		.tbl = {
			{ lpss_spi_dma, 0 },
			{ lpss_hsu_dma, 2 },
			{ lpss_hsu_dma, 4 },
		},
	},
	[byt_lpss_dma_2] = {
		/* ... */
	},
};

int lpss_get_controller_info(int dma_type, int index, struct lpss_dma_info *dma_info)
{
	struct dma_ctl_info *ctl_info;
	struct dma_slave_cfg *cfg;
	int dma_idx, i, count = 0;

	for (dma_idx = 0 ; dma_idx < ARRAY_SIZE(dma_ctl_infos); dma_idx++) {
		ctl_info = &dma_ctl_infos[dma_idx];
		if (!ctl_info->active)
			continue;

		for (i = 0; i < ctl_info->ch_num/2; i++) {
			cfg = &ctl_info->tbl[i];
			if (cfg->dma_type == dma_type && (count++ == index)) {
				dma_info->pdev = ctl_info->pdev;
				dma_info->membase = ctl_info->membase;
				dma_info->irq = ctl_info->irq;
				dma_info->ch_base = cfg->ch_base;
				dma_info->dma_version = LPSS_DMA_V1;
				dev_info(dma_info->pdev, "Found lpss dma: irq=%d, ch_base=%d\n",
						dma_info->irq, dma_info->ch_base);
				return 0;
			}
		}
	}

	return -EINVAL;
}

/**
 * lpss_dma_pci_probe -	PCI Probe
 * @pdev: Controller PCI device structure
 * @id: pci device id structure
 *
 * Initialize the PCI device, map BARs, query driver data.
 * Call setup_dma to complete contoller and chan initilzation
 */
static int lpss_dma_pci_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	struct dma_ctl_info *ctl_info;
	u32 base_addr, bar_size;
	int dma_idx, err;

	dma_idx = id->driver_data;
	ctl_info = &dma_ctl_infos[dma_idx];
	ctl_info->active = true;

	err = pci_enable_device(pdev);
	if (err)
		goto err_enable_device;

	err = pci_request_regions(pdev, "byt_lpss_dma");
	if (err)
		goto err_request_regions;

	err = pci_set_dma_mask(pdev, DMA_BIT_MASK(32));
	if (err)
		goto err_set_dma_mask;

	err = pci_set_consistent_dma_mask(pdev, DMA_BIT_MASK(32));
	if (err)
		goto err_set_dma_mask;

	pci_dev_get(pdev);

	base_addr = pci_resource_start(pdev, 0);
	bar_size  = pci_resource_len(pdev, 0);
	ctl_info->membase = devm_ioremap_nocache(&pdev->dev, base_addr, LPSS_DMA_REG_SIZE);
	if (!ctl_info->membase) {
		dev_err(&pdev->dev, "%s:ioremap failed\n", __func__);
		err = -ENOMEM;
		goto err_ioremap;
	}

	ctl_info->pdev = &pdev->dev;
	ctl_info->irq = pdev->irq;

	pci_set_master(pdev);

	dev_info(&pdev->dev, "%s: probe for %#x irq=%d\n", __func__, pdev->device, pdev->irq);

	pm_runtime_put_noidle(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;

err_ioremap:
	pci_dev_put(pdev);
err_set_dma_mask:
	pci_release_regions(pdev);
err_request_regions:
	pci_disable_device(pdev);
err_enable_device:
	dev_err(&pdev->dev, "%s:Probe failed %d\n", __func__, err);
	return err;
}

/**
 * lpss_dma_pci_remove -	PCI remove
 * @pdev: Controller PCI device structure
 *
 * Free up all resources and data
 * Call shutdown_dma to complete contoller and chan cleanup
 */
static void lpss_dma_pci_remove(struct pci_dev *pdev)
{
	pm_runtime_forbid(&pdev->dev);
	pci_dev_put(pdev);
	pci_release_regions(pdev);
	pci_disable_device(pdev);
}

/* Power Management */
static int dma_suspend(struct device *dev)
{
	dev_dbg(dev, "%s: dma_suspend called\n", __func__);

	return 0;
}

static int dma_resume(struct device *dev)
{
	dev_dbg(dev, "%s: dma_resume called\n", __func__);

	return 0;
}

static int dma_runtime_suspend(struct device *dev)
{
	return dma_suspend(dev);
}

static int dma_runtime_resume(struct device *dev)
{
	return dma_resume(dev);
}

static int dma_runtime_idle(struct device *dev)
{
	dev_dbg(dev, "%s: dma_runtime_idle called\n", __func__);

	return pm_schedule_suspend(dev, 0);
}

static struct pci_device_id lpss_dma_pci_ids[] = {
	/* Baytrail Low Speed Peripheral DMA */
	{ PCI_VDEVICE(INTEL, 0x0F06), byt_lpss_dma_1 },
	{ PCI_VDEVICE(INTEL, 0x0F40), byt_lpss_dma_2 },
	{ 0, }
};

MODULE_DEVICE_TABLE(pci, lpss_dma_pci_ids);

static const struct dev_pm_ops lpss_dma_pm = {
	.suspend_late = dma_suspend,
	.resume_early = dma_resume,
	SET_RUNTIME_PM_OPS(dma_runtime_suspend,
			dma_runtime_resume,
			dma_runtime_idle)
};

static struct pci_driver lpss_dma_pci_driver = {
	.name		=	"lpss_dma_pci",
	.id_table	=	lpss_dma_pci_ids,
	.probe		=	lpss_dma_pci_probe,
	.remove		=	lpss_dma_pci_remove,
#ifdef CONFIG_PM
	.driver = {
		.pm = &lpss_dma_pm,
	},
#endif
};

static const struct acpi_device_id lpss_dma_acpi_ids[] = {
	{ "INTL9C60", byt_lpss_dma_0 },
	{ "80862286", cht_lpss_dma_0 },
	{ "808622C0", cht_lpss_dma_0 },
	{ },
};

int lpss_dma_acpi_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	acpi_handle handle = ACPI_HANDLE(dev);
	struct acpi_device *device;
	const struct acpi_device_id *id;
	const char *hid;
	int ret, uid, dma_idx = 0;
	struct dma_ctl_info *ctl_info;
	struct resource *rsrc, *ioarea;

	ret = acpi_bus_get_device(handle, &device);
	if (ret) {
		dev_err(dev, "%s: could not get acpi device - %d\n", __func__, ret);
		return -ENODEV;
	}

	if (acpi_bus_get_status(device) || !device->status.present) {
		dev_err(dev, "%s: device has invalid status", __func__);
		return -ENODEV;
	}

	hid = acpi_device_hid(device);
	if (kstrtoint(device->pnp.unique_id, 10, &uid))
		return -EINVAL;

	/* Apply default dma_mask if needed */
	if (!pdev->dev.dma_mask) {
		pdev->dev.dma_mask = &pdev->dev.coherent_dma_mask;
		pdev->dev.coherent_dma_mask = DMA_BIT_MASK(32);
	}

	ret = dma_set_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "dma_set_mask failed with err:%d", ret);
		return ret;
	}

	ret = dma_set_coherent_mask(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_err(dev, "_coherent_mask failed with err:%d", ret);
		return ret;
	}

        for (id = lpss_dma_acpi_ids; id->id[0]; id++)
		if (!strncmp(id->id, hid, strlen(id->id)))
			dma_idx = id->driver_data + uid;

	ctl_info = &dma_ctl_infos[dma_idx];
	ctl_info->active = true;

	rsrc = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!rsrc) {
		dev_err(dev, "%s: Invalid memory resource", __func__);
		return -EIO;
	}

	ioarea = request_mem_region(rsrc->start, resource_size(rsrc), pdev->name);
	if (!ioarea) {
		dev_err(&pdev->dev, "LPSS DMA region already claimed\n");
		return -EBUSY;
	}

	ctl_info->membase = devm_ioremap_nocache(&pdev->dev, rsrc->start, resource_size(rsrc));
	if (!ctl_info->membase) {
		dev_err(dev, "%s: unable to map resource: %#x", __func__, (unsigned int)rsrc->start);
		return -EIO;
	}

	ctl_info->irq = platform_get_irq(pdev, 0);
	if (ctl_info->irq < 0) {
		dev_err(dev, "invalid irq:%d\n", ctl_info->irq);
		return ctl_info->irq;
	}

	ctl_info->pdev = dev;

	dev_info(dev, "%s for %s:%d irq=%d\n", __func__, hid, uid, ctl_info->irq);

	pm_runtime_set_active(&pdev->dev);
	pm_runtime_enable(&pdev->dev);
	pm_runtime_allow(&pdev->dev);

	return 0;
}

int lpss_dma_acpi_remove(struct platform_device *pdev)
{
	struct resource *mem;
	pm_runtime_forbid(&pdev->dev);
	mem = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (mem) {
		release_mem_region(mem->start, resource_size(mem));
	}
	return 0;
}

static struct platform_driver lpss_dma_acpi_driver = {
	.driver = {
		.name			= "lpss_dma_acpi",
		.owner			= THIS_MODULE,
		.acpi_match_table	= lpss_dma_acpi_ids,
		.pm			= &lpss_dma_pm,
	},
	.probe	= lpss_dma_acpi_probe,
	.remove	= lpss_dma_acpi_remove,
};

static int __init byt_lpss_dma_init(void)
{
	int ret;

	pr_debug("LPSS DMA Driver Load...");

	ret = pci_register_driver(&lpss_dma_pci_driver);
	if (ret)
		pr_err("PCI dev registration failed");

	ret = platform_driver_register(&lpss_dma_acpi_driver);
	if (ret)
		pr_err("Platform dev registration failed");

	return ret;
}
subsys_initcall(byt_lpss_dma_init);

static void __exit byt_lpss_dma_exit(void)
{
	pci_unregister_driver(&lpss_dma_pci_driver);
	platform_driver_unregister(&lpss_dma_acpi_driver);
}
module_exit(byt_lpss_dma_exit);

MODULE_AUTHOR("Huiquan Zhong <huiquan.zhong@intel.com>");
MODULE_DESCRIPTION("Intel (R) LPSS DMAC Driver");
MODULE_LICENSE("GPL v2");
