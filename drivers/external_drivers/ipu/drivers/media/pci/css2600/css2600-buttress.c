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
#include <linux/dma-attrs.h>
#include <linux/iommu.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/sizes.h>

#include "css2600.h"
#include "css2600-bus.h"
#include "css2600-buttress.h"

static int css2600_buttress_probe(struct css2600_bus_device *adev)
{
	struct css2600_buttress *buttress;

	buttress = devm_kzalloc(&adev->dev, sizeof(*buttress), GFP_KERNEL);
	if (!buttress)
		return -ENOMEM;

	dev_info(&adev->dev, "buttress probe %p %p\n", adev, &adev->dev);
	css2600_bus_set_drvdata(adev, buttress);

	return 0;
}

static void css2600_buttress_remove(struct css2600_bus_device *adev)
{
	dev_info(&adev->dev, "removed\n");
}

static struct css2600_bus_driver css2600_buttress_driver = {
	.probe = css2600_buttress_probe,
	.remove = css2600_buttress_remove,
	.wanted = CSS2600_BUTTRESS_NAME,
	.drv = {
		.name = CSS2600_BUTTRESS_NAME,
		.owner = THIS_MODULE,
	},
};

module_css2600_bus_driver(css2600_buttress_driver);

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@linux.intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel css2600 buttress driver");
