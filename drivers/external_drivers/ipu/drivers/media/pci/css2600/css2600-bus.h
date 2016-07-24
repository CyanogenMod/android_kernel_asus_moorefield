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

#ifndef CSS2600_BUS_H
#define CSS2600_BUS_H

#include <linux/device.h>
#include <linux/list.h>
#include <linux/mm.h>
#include <linux/pci.h>

#define CSS2600_BUS_NAME	CSS2600_NAME "-bus"

struct iommu_domain;

struct css2600_bus_device {
	struct device dev;
	struct list_head list;
	void *pdata;
	struct css2600_bus_driver *adrv;
	void *iommu;
};

#define to_css2600_bus_device(_dev) \
	container_of(_dev, struct css2600_bus_device, dev)

struct css2600_bus_driver {
	struct device_driver	drv;
	char			wanted[20];
	int			(*probe)(struct css2600_bus_device *adev);
	void			(*remove)(struct css2600_bus_device *adev);
	void			(*isr)(struct css2600_bus_device *adev);
};

struct css2600_bus_iommu_mapping {
	void *mapping;
};

struct css2600_bus_iommu {
	struct css2600_bus_iommu_mapping *m;
	struct device *dev;
};

#define to_css2600_bus_driver(_drv) \
	container_of(_drv, struct css2600_bus_driver, drv)

struct css2600_bus_device *css2600_bus_add_device(
	struct pci_dev *pdev, struct device *parent, void *pdata,
	struct css2600_bus_iommu *iommu, char *name, unsigned int nr);
void css2600_bus_del_devices(struct pci_dev *pdev);

int css2600_bus_register_driver(struct css2600_bus_driver *adrv);
int css2600_bus_unregister_driver(struct css2600_bus_driver *adrv);

int css2600_bus_register(void);
void css2600_bus_unregister(void);

void css2600_bus_set_iommu(struct iommu_ops *ops);

#define module_css2600_bus_driver(drv)			\
	module_driver(drv, css2600_bus_register_driver, \
		      css2600_bus_unregister_driver)

#define css2600_bus_set_drvdata(adev, data) dev_set_drvdata(&(adev)->dev, data)
#define css2600_bus_get_drvdata(adev) dev_get_drvdata(&(adev)->dev)

#endif
