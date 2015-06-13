/*
 * PCI glue for HECI provider device (ISH) driver
 *
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
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/version.h>
#include <linux/sched.h>
#include <linux/uuid.h>
#include <linux/compat.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include "heci.h"
#include "heci_dev.h"
#include "hw-ish.h"
#include "utils.h"

#ifdef dev_dbg
#undef dev_dbg
#endif
static void no_dev_dbg(void *v, char *s, ...)
{
}
#define dev_dbg no_dev_dbg

/*#define dev_dbg dev_err*/

/*
 *  heci driver strings
 */
static bool nomsi;
module_param_named(nomsi, nomsi, bool, S_IRUGO | S_IWUSR);
MODULE_PARM_DESC(nomsi, "don't use msi (default = false)");

/* Currently this driver works as long as there is only a single AMT device. */
static struct pci_dev *heci_pci_device;

static DEFINE_PCI_DEVICE_TABLE(heci_ish_pci_tbl) = {
	{PCI_DEVICE(PCI_VENDOR_ID_INTEL, 0x22D8)},
	{0, }
};

MODULE_DEVICE_TABLE(pci, heci_ish_pci_tbl);

static DEFINE_MUTEX(heci_mutex);

#ifdef TIMER_POLLING
/*
 * DD -- ISH timer-polling workaround for H-FPGA
 * (and other platforms that fail to deliver interrupts)
 * NOTE: currently this will break (crash) if driver is unloaded
 */

#include <linux/timer.h>

struct timer_list ish_poll_timer;
void *timer_data;
struct work_struct ish_poll_work;

void ish_poll_work_fn(void *prm)
{
	irqreturn_t rv;

	rv = heci_ish_irq_thread_handler(0, timer_data);
}

void ish_poll_timer_fn(unsigned long unused)
{
	irqreturn_t rv;

	rv = heci_ish_irq_quick_handler(0, timer_data);

	/* Wake workqueue */
	if (rv == IRQ_WAKE_THREAD)
		schedule_work(&ish_poll_work);

	/* Reschedule timer */
	ish_poll_timer.expires += 2;
	add_timer(&ish_poll_timer);
}

#endif	/* TIMER_POLLING */


/**
 * heci_probe - Device Initialization Routine
 *
 * @pdev: PCI device structure
 * @ent: entry in heci_ish_pci_tbl
 *
 * returns 0 on success, <0 on failure.
 */
static int heci_ish_probe(struct pci_dev *pdev,
			const struct pci_device_id *ent)
{
	struct heci_device *dev;
	struct heci_ish_hw *hw;
	int err;

#if defined(SUPPORT_A0_ONLY)
	pdev->revision = REVISION_ID_CHT_A0;
#elif defined(SUPPORT_B0_ONLY)
	pdev->revision = REVISION_ID_CHT_B0;
#endif
	mutex_lock(&heci_mutex);
	if (heci_pci_device) {
		err = -EEXIST;
		goto end;
	}
	/* enable pci dev */
	err = pci_enable_device(pdev);
	if (err) {
		dev_err(&pdev->dev, "heci: Failed to enable pci device.\n");
		goto end;
	}
	/* set PCI host mastering  */
	pci_set_master(pdev);
	/* pci request regions for heci driver */
	err = pci_request_regions(pdev, KBUILD_MODNAME);
	if (err) {
		dev_err(&pdev->dev, "heci: Failed to get pci regions.\n");
		goto disable_device;
	}

	/* allocates and initializes the heci dev structure */
	dev = heci_ish_dev_init(pdev);
	if (!dev) {
		err = -ENOMEM;
		goto release_regions;
	}
	hw = to_ish_hw(dev);

	/* mapping  IO device memory */
	hw->mem_addr = pci_iomap(pdev, 0, 0);
	if (!hw->mem_addr) {
		dev_err(&pdev->dev, "mapping I/O device memory failure.\n");
		err = -ENOMEM;
		goto free_device;
	}

	/* clear spurious interrupts */
	heci_clear_interrupts(dev);
	dev_dbg(&pdev->dev, "heci: after heci_clear_interrupts\n");

	heci_pci_device = pdev;

	/* request and enable interrupt   */
#ifndef TIMER_POLLING
	err = request_threaded_irq(pdev->irq,
			heci_ish_irq_quick_handler,
			heci_ish_irq_thread_handler,
			IRQF_SHARED, KBUILD_MODNAME, dev);
	if (err) {
		dev_err(&pdev->dev, "heci: request_threaded_irq failure. irq = %d\n",
			pdev->irq);
		goto free_device;
	}
	dev_dbg(&pdev->dev, "heci: after request_threaded_irq\n");
#else
	/* Init & prepare workqueue */
	INIT_WORK(&ish_poll_work, ish_poll_work_fn);

	/* Create and schedule ISH polling timer */
	init_timer(&ish_poll_timer);
	ish_poll_timer.data = 0;
	ish_poll_timer.function = ish_poll_timer_fn;
	ish_poll_timer.expires = jiffies + 2;
	timer_data = dev;
	add_timer(&ish_poll_timer);

	/* Init ISH polling timers workqueue */
#endif

	/* PCI quirk: prevent from being put into D3 state */
	pdev->dev_flags |= PCI_DEV_FLAGS_NO_D3;

#if 0
	/* TEST: in order to test reverse (FW-initiated) reset flow,
	 * set "host ready" here and wait until FW starts its reset
	 */
	dev->recvd_hw_ready = 0;
	heci_ish_set_host_rdy(dev);
#endif

#ifdef	D3_RCR
	/* After that we can enable ISH DMA operation */
	writel(IPC_RMP2_DMA_ENABLED, hw->mem_addr + IPC_REG_ISH_RMP2);

	/* Send 0 IPC message so that ISH FW wakes up if it was already asleep */
	writel(IPC_DRBL_BUSY_BIT, hw->mem_addr + IPC_REG_HOST2ISH_DRBL);
#endif

	if (heci_start(dev)) {
		dev_err(&pdev->dev, "heci: Init hw failure.\n");
		err = -ENODEV;
		goto release_irq;
	}
	dev_dbg(&pdev->dev, "heci: after heci_start\n");

	err = heci_register(dev);
	if (err)
		goto release_irq;
	dev_dbg(&pdev->dev, "heci: after heci_register\n");

	pci_set_drvdata(pdev, dev);
	dev_dbg(&pdev->dev, "heci: after pci_set_drvdata\n");

	mutex_unlock(&heci_mutex);

	return 0;

	heci_deregister(dev);
release_irq:
	/* disable interrupts */
	heci_disable_interrupts(dev);
	free_irq(pdev->irq, dev);

free_device:
	pci_iounmap(pdev, hw->mem_addr);
	kfree(dev);
release_regions:
	pci_release_regions(pdev);
disable_device:
	pci_disable_device(pdev);
end:
	mutex_unlock(&heci_mutex);
	dev_err(&pdev->dev, "heci: Driver initialization failed.\n");
	return err;
}

/**
 * heci_remove - Device Removal Routine
 *
 * @pdev: PCI device structure
 *
 * heci_remove is called by the PCI subsystem to alert the driver
 * that it should release a PCI device.
 */
static void heci_ish_remove(struct pci_dev *pdev)
{
	struct heci_device *dev;
	struct heci_ish_hw *hw;

	if (heci_pci_device != pdev) {
		dev_err(&pdev->dev, "heci: heci_pci_device != pdev\n");
		return;
	}

	dev = pci_get_drvdata(pdev);
	if (!dev) {
		dev_err(&pdev->dev, "heci: dev =NULL\n");
		return;
	}

	hw = to_ish_hw(dev);

	mutex_lock(&dev->device_lock);

	/* disable interrupts */
	heci_disable_interrupts(dev);
	free_irq(pdev->irq, dev);
	pci_disable_msi(pdev);

	pci_iounmap(pdev, hw->mem_addr);

	heci_pci_device = NULL;

	mutex_unlock(&dev->device_lock);

	flush_scheduled_work();

	pci_set_drvdata(pdev, NULL);

	heci_deregister(dev);

	kfree(dev);

	pci_release_regions(pdev);
	pci_disable_device(pdev);

}

#define HECI_ISH_PM_OPS	NULL

/*
 *  PCI driver structure
 */
static struct pci_driver heci_ish_driver = {
	.name = KBUILD_MODNAME,
	.id_table = heci_ish_pci_tbl,
	.probe = heci_ish_probe,
	.remove = heci_ish_remove,
	.shutdown = heci_ish_remove,
	.driver.pm = HECI_ISH_PM_OPS,
};

module_pci_driver(heci_ish_driver);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) Integrated Sensor Hub IPC");
MODULE_LICENSE("GPL v2");
