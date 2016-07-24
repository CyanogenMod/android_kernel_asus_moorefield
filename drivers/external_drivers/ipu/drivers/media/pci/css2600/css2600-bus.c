/*
 * Copyright (c) 2013--2014 Intel Corporation. All Rights Reserved.
 *
 * Bus implementation based on the bt8xx driver.
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

#include <linux/delay.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/iommu.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/pci.h>

#include "css2600.h"
#include "css2600-dma.h"

static int css2600_bus_match(struct device *dev, struct device_driver *drv)
{
	struct css2600_bus_driver *adrv = to_css2600_bus_driver(drv);

	dev_dbg(dev, "bus match: \"%s\" --- \"%s\"\n", dev_name(dev),
		adrv->wanted);

	return !strncmp(dev_name(dev), adrv->wanted, strlen(adrv->wanted));
}

static int css2600_bus_probe(struct device *dev)
{
	struct css2600_bus_device *adev = to_css2600_bus_device(dev);
	struct css2600_bus_driver *adrv = to_css2600_bus_driver(dev->driver);
	int rval;

	adev->adrv = adrv;
	rval = adrv->probe ? adrv->probe(adev) : -ENODEV;

	if (rval) {
		css2600_bus_set_drvdata(adev, NULL);
		adev->adrv = NULL;
	}

	return rval;
}

static int css2600_bus_remove(struct device *dev)
{
	struct css2600_bus_device *adev = to_css2600_bus_device(dev);
	struct css2600_bus_driver *adrv = to_css2600_bus_driver(dev->driver);

	if (adrv->remove)
		adrv->remove(adev);

	return 0;
}

static struct bus_type css2600_bus = {
	.name = CSS2600_BUS_NAME,
	.match = css2600_bus_match,
	.probe = css2600_bus_probe,
	.remove = css2600_bus_remove,
};

struct mutex css2600_bus_mutex;

static void css2600_bus_release(struct device *dev)
{
	struct css2600_bus_device *adev = to_css2600_bus_device(dev);

	kfree(adev);
}

struct css2600_bus_device *css2600_bus_add_device(
	struct pci_dev *pdev, struct device *parent, void *pdata,
	struct css2600_bus_iommu *iommu, char *name, unsigned int nr)
{
	struct css2600_bus_device *adev;
	struct css2600_device *isp = pci_get_drvdata(pdev);
	int rval;

	adev = kzalloc(sizeof(*adev), GFP_KERNEL);
	if (!adev)
		return ERR_PTR(-ENOMEM);

	adev->dev.parent = parent;
	adev->dev.bus = &css2600_bus;
	adev->dev.release = css2600_bus_release;
	adev->dev.archdata.dma_ops = &css2600_dma_ops;
	adev->dev.dma_mask = pdev->dev.dma_mask;
	adev->dev.coherent_dma_mask = pdev->dev.coherent_dma_mask;
	adev->iommu = iommu;
	adev->pdata = pdata;
	dev_set_name(&adev->dev, "%s%d", name, nr);

	rval = device_register(&adev->dev);
	if (rval) {
		put_device(&adev->dev);
		return ERR_PTR(rval);
	}

	mutex_lock(&css2600_bus_mutex);
	list_add(&adev->list, &isp->devices);
	mutex_unlock(&css2600_bus_mutex);

	return adev;
}

void css2600_bus_del_devices(struct pci_dev *pdev)
{
	struct css2600_device *isp = pci_get_drvdata(pdev);
	struct css2600_bus_device *adev, *save;

	mutex_lock(&css2600_bus_mutex);

	list_for_each_entry_safe(adev, save, &isp->devices, list) {
		list_del(&adev->list);
		device_unregister(&adev->dev);
	}

	mutex_unlock(&css2600_bus_mutex);
}

int css2600_bus_register_driver(struct css2600_bus_driver *adrv)
{
	adrv->drv.bus = &css2600_bus;
	return driver_register(&adrv->drv);
}
EXPORT_SYMBOL(css2600_bus_register_driver);

int css2600_bus_unregister_driver(struct css2600_bus_driver *adrv)
{
	driver_unregister(&adrv->drv);
	return 0;
}
EXPORT_SYMBOL(css2600_bus_unregister_driver);

int css2600_bus_register(void)
{
	mutex_init(&css2600_bus_mutex);
	return bus_register(&css2600_bus);
}
EXPORT_SYMBOL(css2600_bus_register);

void css2600_bus_unregister(void)
{
	mutex_destroy(&css2600_bus_mutex);
	return bus_unregister(&css2600_bus);
}
EXPORT_SYMBOL(css2600_bus_unregister);

void css2600_bus_set_iommu(struct iommu_ops *ops)
{
	bus_set_iommu(&css2600_bus, ops);
}
EXPORT_SYMBOL(css2600_bus_set_iommu);
