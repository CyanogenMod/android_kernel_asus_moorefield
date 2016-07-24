/*
 * pmic_ipc.c: Intel PMIC Driver interface based SCU IPC mechanism
 *
 * (C) Copyright 2015 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 *
 */

/* Includes */
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/suspend.h>
#include <linux/irq.h>
#include <linux/mfd/core.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel-mid.h>
#include "./pmic.h"
#include <linux/gpio.h>


static struct intel_mid_pmic *pmic_ipc;

static int pmic_ipc_readb(int reg)
{
	int ret = 0;
	u8 data;

	ret = intel_scu_ipc_ioread8(reg, &data);
	if (ret)
		return -EIO;

	return data;
}

static int pmic_ipc_writeb(int reg, u8 val)
{
	int ret;

	ret = intel_scu_ipc_iowrite8(reg, val);
	if (ret)
		return -EIO;

	return 0;
}

static const struct platform_device_id pmic_ipc_id[] = {
	{"pmic_ipc", (kernel_ulong_t)&dollar_cove_pmic},
	{}
};

static int pmic_ipc_probe(struct platform_device *pdev)
{
	const struct platform_device_id *id;

	if (pmic_ipc) {
		dev_err(&pdev->dev, "More than one PMIC device found\n");
		return -EBUSY;
	}

	for (id = pmic_ipc_id; id->name[0]; id++)
		if (!strncmp(id->name, dev_name(&pdev->dev), strlen(id->name)))
			pmic_ipc = (struct intel_mid_pmic *)id->driver_data;

	pmic_ipc->dev	= &pdev->dev;
	if (get_gpio_by_name("pmic_irq") < 0) {
		dev_err(&pdev->dev, "PMIC-IRQ GPIO not found\n");
		return -EBUSY;
	} else {
		pmic_ipc->pmic_int_gpio = get_gpio_by_name("pmic_irq");
	}
	pmic_ipc->readb	= pmic_ipc_readb;
	pmic_ipc->writeb = pmic_ipc_writeb;
	return intel_pmic_add(pmic_ipc);
}

static int pmic_ipc_remove(struct platform_device *pdev)
{
	int ret = intel_pmic_remove(pmic_ipc);

	pmic_ipc = NULL;
	return ret;
}

MODULE_DEVICE_TABLE(platform, pmic_ipc_id);

static struct platform_driver pmic_ipc_driver = {
	.probe = pmic_ipc_probe,
	.remove = pmic_ipc_remove,
	.id_table = pmic_ipc_id,
	.driver = {
		.name = "pmic_ipc",
		.owner = THIS_MODULE,
	},
};

static int __init pmic_ipc_init(void)
{
	return platform_driver_register(&pmic_ipc_driver);
}
module_init(pmic_ipc_init);

static void __exit pmic_ipc_exit(void)
{
	platform_driver_unregister(&pmic_ipc_driver);
}
module_exit(pmic_ipc_exit);

MODULE_AUTHOR("Asutosh Pathak <asutosh.pathak@intel.com>");
MODULE_DESCRIPTION("Intel PMIC IPC driver");
MODULE_LICENSE("GPL");

