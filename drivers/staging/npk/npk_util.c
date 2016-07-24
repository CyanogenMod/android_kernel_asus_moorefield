/*
 * Intel NPK utility driver provides ioctl to access NPK driver registers
 *
 * Copyright (c) 2013 Intel Corporation, Kai Li <kai.li@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307 USA
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/compat.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/uaccess.h>
#include <linux/npk.h>
#include "npk_pci.h"

/**
 * npk_util_reg_read
 *
 * This function allows to read memory mapped NPK CSR registers
 */
static int npk_util_reg_read(unsigned long arg)
{
	struct npk_reg_cmd ncmd;
	int ret;

	if (copy_from_user(&ncmd, (void __user *)arg, sizeof(struct npk_reg_cmd)))
		return -EFAULT;

	ret = npk_reg_read(ncmd.offset, &ncmd.data);
	if (ret) {
		pr_err("register read failed\n");
		return -EFAULT;
	}

	if (copy_to_user((void __user *)arg, &ncmd, sizeof(struct npk_reg_cmd)))
		return -EFAULT;

	return ret;
}

/**
 * npk_util_reg_write
 *
 * This function allows to write memory mapped NPK CSR registers
 */
static int npk_util_reg_write(unsigned long arg)
{
	struct npk_reg_cmd ncmd;
	int ret;

	if (copy_from_user(&ncmd, (void __user *)arg, sizeof(struct npk_reg_cmd)))
		return -EFAULT;

	ret = npk_reg_write(ncmd.offset, ncmd.data);

	return ret;
}

/**
 * npk_util_pci_read
 *
 * This function allows to read NPK PCI configuration space
 */
static int npk_util_pci_read(unsigned long arg)
{
	struct pci_cfg_cmd pcmd;
	int ret;

	if (copy_from_user(&pcmd, (void __user *)arg, sizeof(struct pci_cfg_cmd)))
		return -EFAULT;

	ret = npk_pci_read(pcmd.offset, &pcmd.data, pcmd.size);

	if (ret) {
		pr_err("pci read failed\n");
		return -EFAULT;
	}

	if (copy_to_user((void __user *)arg, &pcmd, sizeof(struct pci_cfg_cmd)))
		return -EFAULT;

	return 0;
}

/**
 * npk_util_pci_write
 *
 * This function allows to write NPK PCI configuration space
 */
static int npk_util_pci_write(unsigned long arg)
{
	struct pci_cfg_cmd pcmd;
	int ret;

	if (copy_from_user(&pcmd, (void __user *)arg, sizeof(struct pci_cfg_cmd)))
		return -EFAULT;

	ret = npk_pci_write(pcmd.offset, pcmd.data, pcmd.size);
	if (ret) {
		pr_err("pci write failed\n");
		return -EFAULT;
	}

	return 0;
}

#define MEM_READ_WRITE_MAX_SIZE 0x20000

/**
 * npk_util_mem_read
 *
 * This function allows to read several NPK PCI registers at once.
 * The npk_mem_cmd structure holds an array of offsets, and an array
 * of data (value that is read for each offset).
 */
static int npk_util_mem_read(unsigned long arg)
{
	struct npk_mem_cmd kcmd;
	int ret = 0;
	int i;
	int length;
	u32 *offset;
	u32 *data;

	if (copy_from_user(&kcmd, (void __user *)arg, sizeof(struct npk_mem_cmd)))
		return -EFAULT;

	/* sanity check */
	if (kcmd.size > MEM_READ_WRITE_MAX_SIZE)
		return -EINVAL;

	length = sizeof(u32) * kcmd.size;
	offset = kmalloc(length, GFP_KERNEL);
	if (!offset)
		return -ENOMEM;

	if (copy_from_user(offset, (void __user *)kcmd.addr, length)) {
		ret = -EFAULT;
		goto err_offset;
	}

	data = kmalloc(length, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err_offset;
	}

	for (i = 0; i < kcmd.size; i++) {
		ret = npk_reg_read(offset[i], &data[i]);
		if (ret) {
			ret = -EFAULT;
			goto err_data;
		}
	}

	if (copy_to_user((void __user *)kcmd.data, data, length))
		ret = -EFAULT;

err_data:
	kfree(data);
err_offset:
	kfree(offset);

	return ret;
}

/**
 * npk_util_mem_write
 *
 * This function allows to write several NPK PCI registers at once.
 * The npk_mem_cmd structure holds an array of offsets and an array of
 * data to apply for each offset.
 */
static int npk_util_mem_write(unsigned long arg)
{
	struct npk_mem_cmd kcmd;
	int ret = 0;
	int i;
	int length;
	u32 *offset;
	u32 *data;

	if (copy_from_user(&kcmd, (void __user *)arg, sizeof(struct npk_mem_cmd)))
		return -EFAULT;

	/* sanity check */
	if (kcmd.size > MEM_READ_WRITE_MAX_SIZE)
		return -EINVAL;

	length = sizeof(u32) * kcmd.size;
	offset = kmalloc(length, GFP_KERNEL);
	if (!offset)
		return -ENOMEM;

	if (copy_from_user(offset, (void __user *)kcmd.addr, length)) {
		ret = -EFAULT;
		goto err_offset;
	}

	data = kmalloc(length, GFP_KERNEL);
	if (!data) {
		ret = -ENOMEM;
		goto err_offset;
	}

	if (copy_from_user(data, (void __user *)kcmd.data, length)) {
		ret = -EFAULT;
		goto err_data;
	}

	for (i = 0; i < kcmd.size; i++) {
		ret = npk_reg_write(offset[i], data[i]);
		if (ret) {
			ret = -EFAULT;
			goto err_data;
		}
	}

err_data:
	kfree(data);
err_offset:
	kfree(offset);

	return ret;
}

static long npk_util_ioctl(struct file *fp,
			   unsigned int cmd, unsigned long arg)
{
	int ret;

	switch (cmd) {
	case NPKIOC_REG_READ:
		ret = npk_util_reg_read(arg);
		break;

	case NPKIOC_REG_WRITE:
		ret = npk_util_reg_write(arg);
		break;

	case NPKIOC_PCI_REG_READ:
		ret = npk_util_pci_read(arg);
		break;

	case NPKIOC_PCI_REG_WRITE:
		ret = npk_util_pci_write(arg);
		break;

	case NPKIOC_MEM_READ:
		ret = npk_util_mem_read(arg);
		break;

	case NPKIOC_MEM_WRITE:
		ret = npk_util_mem_write(arg);
		break;

	default:
		pr_err("unknown ioctl called!\n");
		ret = -EINVAL;
	 }

	return ret;
}

#ifdef CONFIG_COMPAT
static long npk_util_compat_ioctl(struct file *file, uint command, ulong u)
{
	return npk_util_ioctl(file, command, (ulong)compat_ptr(u));
}
#else
#define npk_util_compat_ioctl NULL
#endif

static const struct file_operations npk_util_fops = {
	.unlocked_ioctl = npk_util_ioctl,
	.compat_ioctl = npk_util_compat_ioctl,
};

static struct miscdevice npk_util = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "npk_util",
	.fops = &npk_util_fops,
};

static int __init npk_util_init(void)
{
	return misc_register(&npk_util);
}

static void __exit npk_util_exit(void)
{
	misc_deregister(&npk_util);
}

module_init(npk_util_init);
module_exit(npk_util_exit);

MODULE_AUTHOR("Kai Li <kai.li@intel.com>");
MODULE_DESCRIPTION("Intel NPK Utility driver");
MODULE_LICENSE("GPL v2");


