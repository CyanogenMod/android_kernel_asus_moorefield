/*
 * PCI interface driver for Intel Northpeak
 *
 * Copyright (C) 2013 Intel Corporation, Kai Li <kai.li@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation,
 * Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ":%s: " fmt, __func__

#include <linux/pci.h>
#include <linux/slab.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/mm.h>
#include <linux/list.h>
#include <linux/compat.h>
#include <linux/npk.h>
#include "npk_pci.h"

#define DRIVER_NAME "npk_pci"

static struct npk_pci_device *g_dev;

/* lock to protect concurrent access to the NPK device */
static DEFINE_MUTEX(lock);

/* array of SW Masters that are reserved for the kernel */
static struct npk_sth_master g_sth_master[NR_CPUS];

static struct npk_sth_reg npk_1_0_sth_reg = {
	.sthcap0 = 0x4000,
	.sthcap1 = 0x4004
};

static struct npk_msu_reg npk_1_0_msu_reg[MAX_NUM_MSU] = {
	{
		.mscctrl = 0xa0100,
		.mscsts = 0xa0104,
		.mscbar = 0xa0108,
		.mscdestsz = 0xa010c,
		.mscmwp = 0xa0110,
		.msctrp = 0xa0114,
		.msctwp = 0xa0118,
		.mscnwsa = 0xa011c
	},
	{
		.mscctrl = 0xa0200,
		.mscsts = 0xa0204,
		.mscbar = 0xa0208,
		.mscdestsz = 0xa020c,
		.mscmwp = 0xa0210,
		.msctrp = 0xa0214,
		.msctwp = 0xa0218,
		.mscnwsa = 0xa021c
	},
};

static struct npk_gth_reg npk_1_0_gth_reg = {
	.swdest0 = 0x8,
	.gswtdest = 0x88,
	.smcr0 = 0x9c,
	.scr = 0xc8,
	.gthstat = 0xd4,
	.scr2 = 0xd8,
	.scrpd = 0xe0
};

static struct npk_pti_reg npk_1_0_pti_reg = {
	.ptictrl = 0x1c00
};

static struct npk_output npk_1_0_output = {
	.msc0 = 0,
	.msc1 = 1,
	.pti = 2
};

static bool npk_1_0_check_reg_range(u32 offset)
{
	/* Check that the offset is valid with regard to the North Peak
	 * MTB/CSR memory map.
	 */
	if ((offset > 0x7fff && offset < 0x20000) ||
	    (offset > 0x5ffff && offset < 0x80000) ||
	    (offset > 0xa1fff && offset < 0xffc00) ||
	    (offset > 0xfffff))
		return false;

	return true;
}

static bool npk_1_0_check_pci_range(u32 offset)
{
	/* Check that the offset is valid with regard to the North Peak
	 * PCI configuration space.
	 */
	if ((offset < 0x04) ||
	    (offset > 0x07 && offset < 0x3c) ||
	    (offset > 0x4c && offset < 0x80) ||
	    (offset > 0x93))
		return false;

	return true;
}

/* device info for NPK 1.0 HW */
static const struct npk_device_info npk_1_0_info = {
	.sth_reg = &npk_1_0_sth_reg,
	.msu_reg = npk_1_0_msu_reg,
	.gth_reg = &npk_1_0_gth_reg,
	.pti_reg = &npk_1_0_pti_reg,
	.output = &npk_1_0_output,
	.check_reg_range = npk_1_0_check_reg_range,
	.check_pci_range = npk_1_0_check_pci_range,
};

/**
 * This function allocates STH channels for SVEN instrumentation
 * in the user-space.
 * The STH channels that are reserved for user-space are organized
 * by page, each page is holding several STH channels.
 * There is a list of "free" pages, and a list of "used" pages.
 * If a free page is available, all the STH channels it holds are
 * allocated to the requester.
 *
 * This function assumes that the caller has grabbed the mutex that
 * prevents concurrent access to the device.
 */
static int alloc_channel(struct npk_pci_device *npkdev,
			 unsigned long arg)
{
	struct sth_channel_blk *tmp;
	struct sth_page_request_info info;

	if (copy_from_user(&info, (void __user *)arg,
			   sizeof(struct sth_page_request_info)))
		return -EFAULT;

	if (info.info_size < sizeof(struct sth_page_request_info))
		return -EFAULT;

	if (list_empty(&npkdev->free_queue)) {
		pr_info("no free channel\n");
		return -EAGAIN;
	}

	tmp = list_entry(npkdev->free_queue.next, struct sth_channel_blk,
			 element);

	info.info_size = sizeof(struct sth_page_request_info);
	info.channel_size = sizeof(struct sven_sth_channel);
	info.offset = tmp->pg_offset * PAGE_SIZE;
	info.length = tmp->length;

	if (copy_to_user((void __user *)arg, &info,
			 sizeof(struct sth_page_request_info)))
		return -EFAULT;

	list_move_tail(npkdev->free_queue.next, &npkdev->used_queue);

	return 0;
}

/**
 * This function releases one page of STH channels that were reserved
 * for user-space.
 *
 * This function assumes that the caller has grabbed the mutex that
 * prevents concurrent access to the device.
 */
static int free_channel(struct npk_pci_device *npkdev,
			unsigned long arg)
{
	struct sth_page_request_info info;
	struct sth_channel_blk *tmp;
	unsigned long offset;
	int found = 0;

	if (copy_from_user(&info, (void __user *)arg,
		sizeof(struct sth_page_request_info)))
		return -EFAULT;

	if (info.info_size < sizeof(struct sth_page_request_info))
		return -EFAULT;

	offset = info.offset >> PAGE_SHIFT;

	list_for_each_entry(tmp, &npkdev->used_queue, element) {
		 if (tmp->pg_offset == offset) {
			list_move_tail(&tmp->element, &npkdev->free_queue);
			found = 1;
			break;
		}
	}

	if (!found) {
		pr_err("given channel cannot be found\n");
		return -EFAULT;
	}

	return 0;
}

static long npk_sth_ioctl(struct file *filp, unsigned cmd, unsigned long arg)
{
	int ret;

	mutex_lock(&lock);

	if (!g_dev) {
		mutex_unlock(&lock);
		return -ENODEV;
	}

	switch (cmd) {
	case NPKIOC_ALLOC_PAGE_SVEN:
		ret = alloc_channel(g_dev, arg);
		break;
	case NPKIOC_FREE_PAGE_SVEN:
		ret = free_channel(g_dev, arg);
		break;
	default:
		ret = -ENOIOCTLCMD;
		break;
	}

	mutex_unlock(&lock);
	return ret;
}

/**
 * This function does the mapping of one page of MMIO holding
 * STH channels reserved for user-space (the STH channels are
 * previously reserved with the NPKIOC_ALLOC_PAGE_SVEN ioctl cmd).
 */
static int npk_sth_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct sth_channel_blk *tmp;
	int found = 0;

	mutex_lock(&lock);

	if ((!g_dev) || (vma->vm_end < vma->vm_start)) {
		mutex_unlock(&lock);
		return -EINVAL;
	}

	if ((vma->vm_end - vma->vm_start) > PAGE_SIZE) {
		mutex_unlock(&lock);
		pr_err("invalid map size\n");
		return -EINVAL;
	}

	list_for_each_entry(tmp, &g_dev->used_queue, element) {
		if (tmp->pg_offset == vma->vm_pgoff) {
			vma->vm_pgoff = tmp->phy;
			found = 1;
			break;
		}
	}

	if (!found) {
		mutex_unlock(&lock);
		return -EINVAL;
	}

	mutex_unlock(&lock);

	vma->vm_flags |= VM_IO | VM_DONTEXPAND;
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, vma->vm_pgoff,
			    PAGE_SIZE, vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

#ifdef CONFIG_COMPAT
static long npk_sth_compat_ioctl(struct file *file, uint command, ulong u)
{
	return npk_sth_ioctl(file, command, (ulong)compat_ptr(u));
}
#else
#define npk_sth_compat_ioctl NULL
#endif

static const struct file_operations npk_sth_fops = {
	.unlocked_ioctl = npk_sth_ioctl,
	.compat_ioctl = npk_sth_compat_ioctl,
	.mmap = npk_sth_mmap
};

static struct miscdevice npk_sth = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "npk_sth",
	.fops = &npk_sth_fops,
};

/* This function assumes that the caller has grabbed the mutex that
 * prevents concurrent access to the device.
 */
static void *alloc_sven_ptr(int cpu)
{
	struct npk_sth_master *master = &g_sth_master[cpu];
	int channel;

	channel = find_next_zero_bit(master->channel_map,
				     g_dev->chlcnt,
				     FIRST_SVEN_KERNEL_CHANNEL);

	if ((channel < FIRST_SVEN_KERNEL_CHANNEL) ||
	    (channel > LAST_SVEN_KERNEL_CHANNEL)) {
		pr_err("could not get a free channel for sven for cpu %d\n", cpu);
		return NULL;
	}

	set_bit(channel, master->channel_map);

	return ((struct sven_sth_channel *)master->base) + channel;
}

/* This function assumes that the caller has grabbed the mutex that
 * prevents concurrent access to the device.
 */
static void free_sth_sven_ptr(int cpu, void __iomem *channel_base)
{
	struct npk_sth_master *master = &g_sth_master[cpu];
	int channel;

	channel = (channel_base - master->base) / sizeof(struct sven_sth_channel);
	if ((channel < FIRST_SVEN_KERNEL_CHANNEL) ||
	    (channel > LAST_SVEN_KERNEL_CHANNEL)) {
		pr_err("invalid channel for sven for cpu %d\n", cpu);
		return;
	}

	clear_bit(channel, master->channel_map);
}

/**
 * npk_alloc_sth_sven_ptr
 *
 * This function allocates an available STH channel reserved for
 * SVEN instrumentation in the kernel (each CPU is associated with
 * a master, and there are several channels reserved for each master).
 */
void *npk_alloc_sth_sven_ptr(int cpu)
{
	void *ret;

	mutex_lock(&lock);

	if (g_dev == NULL) {
		mutex_unlock(&lock);
		return NULL;
	}

	if (cpu < nr_cpu_ids && cpu_possible(cpu))
		ret = alloc_sven_ptr(cpu);
	else
		ret = NULL;

	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(npk_alloc_sth_sven_ptr);

/**
 * npk_free_sth_sven_ptr
 *
 * This function releases the given STH channel for a given CPU.
 */
void npk_free_sth_sven_ptr(int cpu, void __iomem *channel)
{
	mutex_lock(&lock);

	if (g_dev == NULL) {
		mutex_unlock(&lock);
		return;
	}

	if (cpu < nr_cpu_ids && cpu_possible(cpu))
		free_sth_sven_ptr(cpu, channel);

	mutex_unlock(&lock);

}
EXPORT_SYMBOL(npk_free_sth_sven_ptr);

/* This function assumes that the caller has grabbed the mutex that
 * prevents concurrent access to the device.
 */
static int npk_reg_rdwr(u32 offset, u32 *data, u32 cmd)
{
	int ret = 0;

	/* Check that the offset is within a supported range of registers */
	if (!g_dev->dev_info->check_reg_range(offset))
		return -EINVAL;

	if (cmd == NPK_REG_CMD_R)
		*data = ioread32(g_dev->csr_mtb_bar + offset);
	else if (cmd == NPK_REG_CMD_W)
		iowrite32(*data, g_dev->csr_mtb_bar + offset);
	else
		return -EFAULT;

	return ret;
}

/**
 * npk_reg_read
 *
 * This function allows to read memory mapped NPK CSR registers
 */
int npk_reg_read(u32 offset, u32 *data)
{
	int ret;

	mutex_lock(&lock);

	if (g_dev == NULL) {
		mutex_unlock(&lock);
		return -ENODEV;
	}

	ret = npk_reg_rdwr(offset, data, NPK_REG_CMD_R);

	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(npk_reg_read);

/**
 * npk_reg_read_no_lock
 *
 * "No lock" version of the npk_reg_read function.
 *
 * This function assumes the caller has grabbed the mutex that
 * protects the npk device.
 */
int npk_reg_read_no_lock(u32 offset, u32 *data)
{
	return npk_reg_rdwr(offset, data, NPK_REG_CMD_R);
}
EXPORT_SYMBOL(npk_reg_read_no_lock);

/**
 * npk_reg_write
 *
 * This function allows to write memory mapped NPK CSR registers
 */
int npk_reg_write(u32 offset, u32 data)
{
	int ret;

	mutex_lock(&lock);

	if (g_dev == NULL) {
		mutex_unlock(&lock);
		return -ENODEV;
	}

	ret = npk_reg_rdwr(offset, &data, NPK_REG_CMD_W);

	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(npk_reg_write);

/**
 * npk_reg_write_no_lock
 *
 * "No lock" version of the npk_reg_write function.
 *
 * This function assumes the caller has grabbed the mutex that
 * protects the device.
 */
int npk_reg_write_no_lock(u32 offset, u32 data)
{
	return npk_reg_rdwr(offset, &data, NPK_REG_CMD_W);
}
EXPORT_SYMBOL(npk_reg_write_no_lock);

/**
 * npk_pci_read
 *
 * This function allows to read NPK PCI configuration space
 */
int npk_pci_read(u32 offset, u32 *data, u32 size)
{
	int ret = 0;

	mutex_lock(&lock);

	if (g_dev == NULL) {
		mutex_unlock(&lock);
		return -ENODEV;
	}

	if (data == NULL) {
		pr_err("invalid data pointer\n");
		mutex_unlock(&lock);
		return -EFAULT;
	}

	switch (size) {
	case 1:
	{
		u8 val;
		ret = pci_read_config_byte(g_dev->pdev, offset, &val);
		*data = val;
		break;
	}
	case 2:
	{
		u16 val;
		ret = pci_read_config_word(g_dev->pdev, offset, &val);
		*data = val;
		break;
	}
	case 4:
		ret = pci_read_config_dword(g_dev->pdev, offset, data);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(npk_pci_read);

/**
 * npk_pci_write
 *
 * This function allows to write NPK PCI configuration space
 */
int npk_pci_write(u32 offset, u32 data, u32 size)
{
	int ret = 0;

	mutex_lock(&lock);

	if (g_dev == NULL) {
		mutex_unlock(&lock);
		return -ENODEV;
	}

	/* Check that the offset is valid */
	if (!g_dev->dev_info->check_pci_range(offset)) {
		mutex_unlock(&lock);
		return -EINVAL;
	}

	switch (size) {
	case 1:
	{
		u8 val = data & 0xff;
		ret = pci_write_config_byte(g_dev->pdev, offset, val);
		break;
	}
	case 2:
	{
		u16 val = data & 0xffff;
		ret = pci_write_config_word(g_dev->pdev, offset, val);
		break;
	}
	case 4:
		ret = pci_write_config_dword(g_dev->pdev, offset, data);
		break;
	default:
		ret = -EINVAL;
		break;
	}

	mutex_unlock(&lock);
	return ret;
}
EXPORT_SYMBOL(npk_pci_write);

struct npk_pci_device *get_npk_dev(void)
{
	if (g_dev)
		mutex_lock(&lock);

	return g_dev;
}
EXPORT_SYMBOL(get_npk_dev);

void put_npk_dev(void)
{
	mutex_unlock(&lock);
}
EXPORT_SYMBOL(put_npk_dev);

/**
 * The STH channels that are reserved for user-space are organized
 * by page, each page is holding several STH channels.
 * There is a list of "free" pages, and a list of "used" pages.
 */
static int init_mmap_cxt(struct npk_pci_device *npkdev)
{
	struct sth_channel_blk *tmp;
	unsigned long phy;
	int num_u_page;
	int num_k_page;
	int i;

	INIT_LIST_HEAD(&npkdev->free_queue);
	INIT_LIST_HEAD(&npkdev->used_queue);

	num_k_page = npkdev->k_iolen / PAGE_SIZE;
	num_u_page = npkdev->u_iolen / PAGE_SIZE;

	npkdev->sth_head = kcalloc(num_u_page, sizeof(struct sth_channel_blk), GFP_KERNEL);
	if (npkdev->sth_head == NULL) {
		pr_err("channel block allocation failed\n");
		return -ENOMEM;
	}

	phy = (npkdev->sw_paddr >> PAGE_SHIFT) + num_k_page;
	for (i = 0, tmp = npkdev->sth_head; i < num_u_page; i++, tmp++) {
		tmp->phy = phy + i;
		tmp->pg_offset = num_k_page + i;
		tmp->length = PAGE_SIZE;

		list_add_tail(&tmp->element, &npkdev->free_queue);
	}

	return 0;
}

/**
 * The SW BAR maps MMIO address space for software trace messages to the STH.
 * The NPK driver divides that area into a number of SW Masters.
 * For each Master, there are CHLCNT Channels.
 * Some of the Masters/Channels are reserved for the kernel, some others are
 * reserved for the user-space. For the kernel, there is one Master per CPU.
 * All the remaining SW Masters are reserved for the user-space.
 */
static int npk_map_sw_region(struct npk_pci_device *npkdev)
{
	struct npk_sth_master *master = &g_sth_master[0];
	u32 val;
	u16 swmstp, swmstr;
	int i;

	/* The number of channels per Master is provided by CHLCNT register */
	val = ioread32(npkdev->csr_mtb_bar + npkdev->dev_info->sth_reg->sthcap1);
	npkdev->chlcnt = val & 0xFF;
	if (npkdev->chlcnt > MAX_NUM_OF_STH_CHANNELS) {
		pr_warn("capping number of STH channels to %d\n", MAX_NUM_OF_STH_CHANNELS);
		npkdev->chlcnt = MAX_NUM_OF_STH_CHANNELS;
	}

	/* SWMSTR (in STHCAP0) indicates the first number of SW Masters
	 * SWMSTP (in STHCAP0) indicates the last number of SW Masters
	 * Total number of SW Master is given by (SWMSTP - SWMSTR + 1)
	 */
	val = ioread32(npkdev->csr_mtb_bar + npkdev->dev_info->sth_reg->sthcap0);
	swmstp = val >> 16;
	swmstr = val & 0xFFFF;
	if ((swmstp <= swmstr) ||
	    (swmstp - swmstr + 1) < nr_cpu_ids) {
		pr_err("invalid number of supported SW masters\n");
		return -ENODEV;
	}

	/* sw_iolen is the total length of the STH MMIO area for SW masters
	 * k_iolen is the length of the STH MMIO area for the kernel
	 * u_iolen is the length of the STH MMIO area for the user-space
	 */
	npkdev->sw_iolen = (swmstp - swmstr + 1) * npkdev->chlcnt * sizeof(struct sven_sth_channel);
	npkdev->k_iolen = nr_cpu_ids * npkdev->chlcnt * sizeof(struct sven_sth_channel);
	npkdev->u_iolen = npkdev->sw_iolen - npkdev->k_iolen;
	if (npkdev->sw_iolen < PAGE_SIZE) {
		pr_err("invalid mmio size (%d) is less than one page\n", npkdev->sw_iolen);
		return -ENODEV;
	}

	pr_debug("SW STH MMIO address: %08lx, length: %08x\n",
		 (unsigned long)npkdev->sw_paddr, npkdev->sw_iolen);

	npkdev->sw_bar = ioremap_nocache(npkdev->sw_paddr, npkdev->k_iolen);
	if (!npkdev->sw_bar) {
		pr_err("unable to ioremap sw bar\n");
		return -ENODEV;
	}

	for (i = 0; i < nr_cpu_ids; i++)
		master[i].base = ((struct sven_sth_channel *)npkdev->sw_bar) + i * npkdev->chlcnt;

	return 0;
}

static int npk_pci_probe(struct pci_dev *pdev,
			 const struct pci_device_id *dev_id)
{
	struct npk_pci_device *npkdev;
	int csr_mtb_bar = 0;
	int sw_bar = 2;
	int ret;

	pr_debug("found NPK PCI Device (ID: %04x:%04x)\n", pdev->vendor, pdev->device);

	mutex_lock(&lock);

	if (g_dev) {
		mutex_unlock(&lock);
		pr_err("only one NPK PCI device shall be probed\n");
		return -EFAULT;
	}

	ret = pci_enable_device(pdev);
	if (ret)
		goto err_unlock;

	npkdev = kzalloc(sizeof(struct npk_pci_device), GFP_KERNEL);
	if (!npkdev) {
		pr_err("cannot allocate memory for NPK device\n");
		ret = -ENOMEM;
		goto err_disable;
	}

	npkdev->pdev = pci_dev_get(pdev);
	npkdev->dev_info = (struct npk_device_info *)dev_id->driver_data;

	npkdev->csr_mtb_paddr = pci_resource_start(pdev, csr_mtb_bar);
	npkdev->csr_mtb_bar = ioremap_nocache(npkdev->csr_mtb_paddr, NPK_IO_LEN);
	if (!npkdev->csr_mtb_bar) {
		pr_err("unable to ioremap csr_mtb bar\n");
		ret = -ENODEV;
		goto err_free;
	}

	npkdev->sw_paddr = pci_resource_start(pdev, sw_bar);
	ret = npk_map_sw_region(npkdev);
	if (ret)
		goto err_iounmap_csr_mtb;

	ret = init_mmap_cxt(npkdev);
	if (ret)
		goto err_iounmap_sw;

	ret = pci_request_region(pdev, sw_bar, DRIVER_NAME);
	if (ret) {
		ret = -ENODEV;
		goto err_free_sth;
	}

	ret = misc_register(&npk_sth);
	if (ret)
		goto err_release;

	g_dev = npkdev;
	pci_set_drvdata(pdev, npkdev);

	mutex_unlock(&lock);
	pr_debug("PCI probe completed\n");
	return 0;

err_release:
	pci_release_region(pdev, sw_bar);
err_free_sth:
	kfree(npkdev->sth_head);
err_iounmap_sw:
	iounmap(npkdev->sw_bar);
err_iounmap_csr_mtb:
	iounmap(npkdev->csr_mtb_bar);
err_free:
	pci_dev_put(pdev);
	kfree(npkdev);
err_disable:
	pci_disable_device(pdev);
err_unlock:
	mutex_unlock(&lock);
	return ret;
}

static void npk_pci_remove(struct pci_dev *pdev)
{
	int sw_bar = 2;

	mutex_lock(&lock);

	misc_deregister(&npk_sth);

	pci_release_region(pdev, sw_bar);
	iounmap(g_dev->sw_bar);
	iounmap(g_dev->csr_mtb_bar);

	kfree(g_dev->sth_head);
	kfree(g_dev);

	pci_set_drvdata(pdev, NULL);
	g_dev = NULL;

	pci_dev_put(pdev);
	pci_disable_device(pdev);

	mutex_unlock(&lock);
}

#ifdef CONFIG_PM
static int npk_suspend(struct pci_dev *pdev, pm_message_t state)
{
	return 0;
}

static int npk_resume(struct pci_dev *pdev)
{
	return 0;
}
#else
#define npk_suspend	NULL
#define npk_resume	NULL
#endif

#define FPGA_NPK_PCI_ID 0xC0DE     /* FPGA */
#define SPT_VP_NPK_PCI_ID 0x0963   /* Sunrise Point Virtual Platform */
#define SPT_NPK_PCI_ID 0x9D26      /* Sunrise Point */
#define BXT_VP_NPK_PCI_ID 0x0A8E   /* Broxton Virtual Platform */
#define BXT_NPK_PCI_ID 0x0A80      /* Broxton */

#define NPK_PCI_DEVICE(dev, info) {		\
	.vendor = PCI_VENDOR_ID_INTEL,		\
	.device = dev,				\
	.subvendor = PCI_ANY_ID,		\
	.subdevice = PCI_ANY_ID,		\
	.driver_data = (unsigned long)info }

static DEFINE_PCI_DEVICE_TABLE(npk_ids) = {
	NPK_PCI_DEVICE(FPGA_NPK_PCI_ID, &npk_1_0_info),
	NPK_PCI_DEVICE(SPT_VP_NPK_PCI_ID, &npk_1_0_info),
	NPK_PCI_DEVICE(SPT_NPK_PCI_ID, &npk_1_0_info),
	NPK_PCI_DEVICE(BXT_VP_NPK_PCI_ID, &npk_1_0_info),
	NPK_PCI_DEVICE(BXT_NPK_PCI_ID, &npk_1_0_info),
	{0},
};

static struct pci_driver npk_pci = {
	.name = DRIVER_NAME,
	.id_table = npk_ids,
	.probe = npk_pci_probe,
	.remove = npk_pci_remove,
	.suspend = npk_suspend,
	.resume = npk_resume,
};

static int __init npk_init(void)
{
	int ret;

	ret = pci_register_driver(&npk_pci);
	if (ret)
		pr_err("driver initialization failure\n");

	return ret;
}

static void __exit npk_exit(void)
{
	pci_unregister_driver(&npk_pci);
}

module_init(npk_init);
module_exit(npk_exit);

MODULE_AUTHOR("Kai Li <kai.li@intel.com>");
MODULE_DESCRIPTION("PCI interface driver for Intel NPK ");
MODULE_LICENSE("GPL v2");

