/**
 * linux/modules/drivers/modem_control/mdm_apcdmp.c
 *
 * Version 1.0
 *
 * This code allows to trigger IMC modems coredump.
 * There is a list of commands available in include/linux/mdm_apcdmp.h
 * Current version supports the following modems :
 * - IMC7260
 * - IMC2230
 * There is no guarantee for other modems
 *
 * Intel Mobile driver for modem debugging.
 *
 * Copyright (C) 2015 ASUS Corporation. All rights reserved.
 *
 * Contact: Yuehtsang Li <Yuehtsang_Li@asus.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/poll.h>
#include <asm/uaccess.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/mdm_apcdmp.h>
#include <linux/workqueue.h>

#define DRVNAME "mdm_apcdmp"
#define APCDMP_PIN_IMC726X      6
#define APCDMP_PIN_IMC2230      0

static void debug_work_handler(struct work_struct *work);
static DECLARE_DELAYED_WORK(debug_work, debug_work_handler);

static int devone_devs = 1;        /* device count */
static int devone_major = 0;       /* MAJOR: dynamic allocation */
static int devone_minor = 0;       /* MINOR: static allocation */
static struct cdev devone_cdev;
static struct class *devone_class = NULL;
static dev_t devone_dev;

struct devone_data {
	rwlock_t lock;
	unsigned char val;
};

static void debug_work_handler(struct work_struct *work)
{
        pr_info(DRVNAME ": %s = %d \n",__func__);
        BUG();
}
/**
 * mdm_apcdmp_configure_gpio - Configure GPIOs
 * @gpio: GPIO to configure
 * @direction: GPIO direction - 0: IN | 1: OUT
 *
 */
static inline int mdm_apcdmp_configure_gpio(int gpio,
					  int direction,
					  int value, const char *desc)
{
	int ret;

	ret = gpio_request(gpio, desc);

	if (direction)
		ret += gpio_direction_output(gpio, value);
	else
		ret += gpio_direction_input(gpio);

	if (ret) {
		pr_err(DRVNAME ": Unable to configure GPIO%d (%s)", gpio, desc);
		ret = -ENODEV;
	}

	return ret;
}

static void mdm_apcdmp_gpio_init(void)
{
        if (!mdm_apcdmp_configure_gpio(APCDMP_PIN_IMC726X,1,0,"726_AP_REQ_DUMP"))
                pr_info(DRVNAME ": %s gpio_%d\n",__func__,APCDMP_PIN_IMC726X);

        if (!mdm_apcdmp_configure_gpio(APCDMP_PIN_IMC2230,1,0,"223_AP_REQ_DUMP"))
                pr_info(DRVNAME ": %s gpio_%d\n",__func__,APCDMP_PIN_IMC2230);
}

static void mdm_apcdmp_gpio_free(void)
{
        gpio_free(APCDMP_PIN_IMC726X);
        pr_info(DRVNAME ": %s gpio_%d\n",__func__,APCDMP_PIN_IMC726X);

        gpio_free(APCDMP_PIN_IMC2230);
        pr_info(DRVNAME ": %s gpio_%d\n",__func__,APCDMP_PIN_IMC2230);
}


void mdm_apcdmp_IMC726X_execute(void)
{
        pr_info(DRVNAME ": %s entry\n",__func__);
        gpio_set_value(APCDMP_PIN_IMC726X, 1);
        msleep(100);
        gpio_set_value(APCDMP_PIN_IMC726X, 0);
        pr_info(DRVNAME ": %s\n exit",__func__);

}

EXPORT_SYMBOL_GPL(mdm_apcdmp_IMC726X_execute);

void mdm_apcdmp_IMC2230_execute(void)
{
        pr_info(DRVNAME ": %s entry\n",__func__);
        gpio_set_value(APCDMP_PIN_IMC2230, 1);
        msleep(100);
        gpio_set_value(APCDMP_PIN_IMC2230, 0);
        pr_info(DRVNAME ": %s\n exit",__func__);

}

EXPORT_SYMBOL_GPL(mdm_apcdmp_IMC2230_execute);

void mdm_apcdmp_waiting_coredump_recevied_then_BUG(void)
{
        pr_info(DRVNAME ": %s entry\n",__func__);
        schedule_delayed_work(&debug_work, 120 * HZ);
        pr_info(DRVNAME ": %s\n exit",__func__);
}
EXPORT_SYMBOL_GPL(mdm_apcdmp_waiting_coredump_recevied_then_BUG);
/*****************************************************************************
 *
 * Char device functions
 *
 ****************************************************************************/

/**
 *  mdm_apcdmp_dev_open - Manage device access
 *  @inode: The node
 *  @filep: Reference to file
 *
 *  Called when a process tries to open the device file
 */
static int mdm_apcdmp_dev_open(struct inode *inode, struct file *filep)
{
	pr_err(DRVNAME ": open\n");
        return 0;
}

/**
 *  mdm_apcdmp_dev_close - Reset open state
 *  @inode: The node
 *  @filep: Reference to file
 *
 *  Called when a process closes the device file.
 */
static int mdm_apcdmp_dev_close(struct inode *inode, struct file *filep)
{
	pr_err(DRVNAME ": close\n");
	return 0;
}

/**
 *  mdm_apcdmp_dev_read - Device read function
 *  @filep: Reference to file
 *  @data: User data
 *  @count: Bytes read.
 *  @ppos: Reference to position in file.
 *
 *  Called when a process, which already opened the dev file, attempts to
 *  read from it. Not allowed.
 */
static ssize_t mdm_apcdmp_dev_read(struct file *filep,
				 char __user *data,
				 size_t count, loff_t *ppos)
{
	pr_err(DRVNAME ": Nothing to read\n");
	return -EINVAL;
}

/**
 *  mdm_apcdmp_dev_write - Device write function
 *  @filep: Reference to file
 *  @data: User data
 *  @count: Bytes read.
 *  @ppos: Reference to position in file.
 *
 *  Called when a process writes to dev file.
 *  Not allowed.
 */
static ssize_t mdm_apcdmp_dev_write(struct file *filep,
				  const char __user *data,
				  size_t count, loff_t *ppos)
{
	pr_err(DRVNAME ": Nothing to write to\n");
	return -EINVAL;
}


/**
 *  mdm_apcdmp_dev_ioctl - Process ioctl requests
 *  @filep: Reference to file that stores private data.
 *  @cmd: Command that should be executed.
 *  @arg: Command's arguments.
 *
 */
long mdm_apcdmp_dev_ioctl(struct file *filep, unsigned int cmd, unsigned long arg)
{
	long ret = 0;

	pr_info(DRVNAME ": ioctl request 0x%x \n", cmd);
        switch (cmd) {
                case MDM_APCDMP_IMC726X:
                        pr_info(DRVNAME ": request modem to coredmp by %d \n",MDM_APCDMP_IMC726X);
                        mdm_apcdmp_IMC726X_execute();
                        goto done;
                        break;

                case MDM_APCDMP_IMC2230:
                        pr_info(DRVNAME ": request modem to coredmp by %d \n",MDM_APCDMP_IMC2230);
                        mdm_apcdmp_IMC2230_execute();
                        goto done;
                        break;

                case MDM_APCDMP_DEBUG_IMC726X:
                        pr_info(DRVNAME ": request modem to coredmp by %d \n",MDM_APCDMP_IMC726X);
                        mdm_apcdmp_IMC726X_execute();
                        mdm_apcdmp_waiting_coredump_recevied_then_BUG();
                        goto done;
                        break;

                case MDM_APCDMP_DEBUG_IMC2230:
                        pr_info(DRVNAME ": request modem to coredmp by %d \n",MDM_APCDMP_IMC2230);
                        mdm_apcdmp_IMC2230_execute();
                        mdm_apcdmp_waiting_coredump_recevied_then_BUG();
                        goto done;
                        break;

                default:
		        pr_err(DRVNAME ": ioctl command %x unknown\n", cmd);
		        ret = -ENOIOCTLCMD;
	}

done:
	return ret;
}
/**
 * Device driver file operations
 */
static const struct file_operations mdm_apcdmp_fops = {
	.open = mdm_apcdmp_dev_open,
	.read = mdm_apcdmp_dev_read,
	.write = mdm_apcdmp_dev_write,
	.release = mdm_apcdmp_dev_close,
	.unlocked_ioctl = mdm_apcdmp_dev_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = mdm_apcdmp_dev_ioctl
#endif
};

static int mdm_apcdmp_module_init(void)
{
	dev_t dev = MKDEV(devone_major, 0);
	int alloc_ret = 0;
	int major;
	int cdev_err = 0;
	struct device *class_dev = NULL;

	alloc_ret = alloc_chrdev_region(&dev, 0, devone_devs,DRVNAME );
	if (alloc_ret)
		goto error;
	devone_major = major = MAJOR(dev);

	cdev_init(&devone_cdev, &mdm_apcdmp_fops);
	devone_cdev.owner = THIS_MODULE;
	devone_cdev.ops = &mdm_apcdmp_fops;
	cdev_err = cdev_add(&devone_cdev, MKDEV(devone_major, devone_minor), devone_devs);
	if (cdev_err)
		goto error;

	/* register class */
	devone_class = class_create(THIS_MODULE, DRVNAME);
	if (IS_ERR(devone_class)) {
		goto error;
	}
	devone_dev = MKDEV(devone_major, devone_minor);
	class_dev = device_create(
					devone_class,
					NULL,
					devone_dev,
					NULL,
					"mdm_apcdmp%d",
					devone_minor);

        pr_info(DRVNAME ": driver loaded\n");
	printk(KERN_ALERT "devone driver(major %d) installed.\n", major);

        mdm_apcdmp_gpio_init();
	return 0;

error:
	if (cdev_err == 0)
		cdev_del(&devone_cdev);

	if (alloc_ret == 0)
		unregister_chrdev_region(dev, devone_devs);

	return -1;
}

static void mdm_apcdmp_module_exit(void)
{

	dev_t dev = MKDEV(devone_major, 0);

	/* unregister class */
	device_destroy(devone_class, devone_dev);
	class_destroy(devone_class);

	cdev_del(&devone_cdev);
	unregister_chrdev_region(dev, devone_devs);

        cancel_delayed_work_sync(&debug_work);

	printk(KERN_ALERT "devone driver removed.\n");
        pr_info(DRVNAME ": driver unloaded\n");

}

module_init(mdm_apcdmp_module_init);
module_exit(mdm_apcdmp_module_exit);

MODULE_AUTHOR("Yuehtsang Li <Yuehtsang_Li@asus.com>");
MODULE_DESCRIPTION("ASUS Modem AP Coredump driver");
MODULE_LICENSE("GPL");
//MODULE_ALIAS("platform:" DEVICE_NAME);
