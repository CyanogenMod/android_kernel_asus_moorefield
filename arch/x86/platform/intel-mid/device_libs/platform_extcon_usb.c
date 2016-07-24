/*
 * platform_extcon_usb.c: extcon_usb platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Vamsi Krishna Kalidindi <vamsi.krishnax.kalidindi@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

static struct platform_device extcon_device = {
	.name           = "extcon-usb",
	.id             = -1,
};

static int __init extcon_usb_init(void)
{
	int err;
	err = platform_device_register(&extcon_device);
	if (err < 0)
		pr_err("Fail to register switch-usb platform device.\n");
	return 0;
}

device_initcall(extcon_usb_init);
