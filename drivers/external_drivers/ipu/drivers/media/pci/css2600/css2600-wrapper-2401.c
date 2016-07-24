/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
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

#include <asm/cacheflush.h>

#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "css2600-bus.h"
#include "css2600-dma.h"
#include "css2600-wrapper-2401.h"
#include "css2600-mmu.h"
#include "lib2401/ia_css_env.h"

enum css2401_wrapper_rws {
	CSS2401_READ,
	CSS2401_WRITE,
	CSS2401_SET,
};

struct my_css_memory_buffer_item {
	struct list_head list;
	dma_addr_t iova;
	void *addr;
	size_t bytes;
	struct dma_attrs attrs;
};

struct device;

struct css2600_isys_iomem_filter {
	void *addr;
	size_t size;
	struct list_head head;
	struct device *dev;
};

static struct {
	struct device *dev; /*ISYS*/
	void __iomem *isp_base; /*IUNIT PCI base address*/
	struct iommu_domain *domain;
	struct list_head buffers;
	struct list_head filters;
	spinlock_t lock;
	struct dma_map_ops *ops;
	struct ia_css_env css_env;
	const struct firmware *fw;
	unsigned int iommus, iommus_registered;
	unsigned int css_init_done;
} mine;

/*Small buffers for CSS layer internal usage*/
static void *glue_ia_alloc(size_t bytes, bool zero_mem)
{
	if (zero_mem)
		return kzalloc(bytes, GFP_KERNEL);
	else
		return kmalloc(bytes, GFP_KERNEL);
}
static void glue_ia_free(void *ptr)
{
	kfree(ptr);
}

/*Flush cache*/
static void glue_ia_flush(struct ia_css_acc_fw *fw)
{
	wbinvd();
}

static ia_css_ptr glue_css_alloc(size_t bytes, uint32_t attributes)
{
	struct dma_map_ops *dma_ops = mine.dev->archdata.dma_ops;
	struct my_css_memory_buffer_item *buf;
	unsigned long flags;

	might_sleep();

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return 0;

	/*Attribute checking*/
	if (attributes & IA_CSS_MEM_ATTR_CONTIGUOUS)
		dma_set_attr(DMA_ATTR_FORCE_CONTIGUOUS, &buf->attrs);

	/*alloc using css2600 dma driver*/
	buf->bytes = bytes;
	buf->addr = dma_ops->alloc(mine.dev, buf->bytes, &buf->iova,
				   GFP_KERNEL, &buf->attrs);

	if (!buf->addr) {
		kfree(buf);
		return 0;
	}

	pr_debug("glue: mapping %lu bytes to %p, iova 0x%8.8x\n", buf->bytes,
		 buf->addr, buf->iova);

	spin_lock_irqsave(&mine.lock, flags);
	list_add(&buf->list, &mine.buffers);
	spin_unlock_irqrestore(&mine.lock, flags);

	return buf->iova;
}

static void glue_css_free(ia_css_ptr iova)
{
	struct dma_map_ops *dma_ops = mine.dev->archdata.dma_ops;
	struct my_css_memory_buffer_item *buf = NULL;
	unsigned long flags;

	might_sleep();

	pr_debug("looking for iova %8.8x\n", iova);

	spin_lock_irqsave(&mine.lock, flags);
	list_for_each_entry(buf, &mine.buffers, list) {
		pr_debug("buffer iova %8.8x\n", (uint32_t)buf->iova);
		if (buf->iova != iova)
			continue;

		pr_debug("found it!\n");
		list_del(&buf->list);
		spin_unlock_irqrestore(&mine.lock, flags);
		dma_ops->free(mine.dev, buf->bytes, buf->addr,
			      buf->iova, &buf->attrs);
		kfree(buf);
		return;
	}
	pr_warn("Can't find iova object %8.8x\n", iova);
	spin_unlock_irqrestore(&mine.lock, flags);
}

int css2600_wrapper_register_buffer(dma_addr_t iova, void *addr, size_t bytes)
{
	struct my_css_memory_buffer_item *buf;
	unsigned long flags;

	might_sleep();

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return -ENOMEM;

	buf->bytes = bytes;
	buf->iova = iova;
	buf->addr = addr;

	spin_lock_irqsave(&mine.lock, flags);
	list_add(&buf->list, &mine.buffers);
	spin_unlock_irqrestore(&mine.lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_register_buffer);

void css2600_wrapper_unregister_buffer(dma_addr_t iova)
{
	struct my_css_memory_buffer_item *buf = NULL;
	unsigned long flags;

	might_sleep();

	spin_lock_irqsave(&mine.lock, flags);
	list_for_each_entry(buf, &mine.buffers, list) {
		if (buf->iova != iova)
			continue;

		list_del(&buf->list);
		spin_unlock_irqrestore(&mine.lock, flags);
		kfree(buf);
		return;
	}
	pr_warn("Can't find iova object %8.8x\n", iova);
	spin_unlock_irqrestore(&mine.lock, flags);
}
EXPORT_SYMBOL_GPL(css2600_wrapper_unregister_buffer);

static int read_write_set(dma_addr_t iova, void *data, size_t bytes,
			  enum css2401_wrapper_rws rws)
{
	struct my_css_memory_buffer_item *buf;
	unsigned long flags;

	pr_debug("looking for iova %8.8x\n", iova);

	spin_lock_irqsave(&mine.lock, flags);
	list_for_each_entry(buf, &mine.buffers, list) {
		void *addr;
		const char *rws_debug[] = { "read", "writ", "set" };

		if (iova < buf->iova || iova + bytes > buf->iova + buf->bytes)
			continue;

		addr = buf->addr + iova - buf->iova;

		pr_debug("glue: %sing %d bytes at %p\n",
			 rws_debug[rws], bytes, buf->addr);

		switch (rws) {
		case CSS2401_READ:
			clflush_cache_range(addr, bytes);
			memcpy(data, addr, bytes);
			break;
		case CSS2401_WRITE:
			memcpy(addr, data, bytes);
			clflush_cache_range(addr, bytes);
			break;
		case CSS2401_SET:
			memset(addr, *(int *)data, bytes);
			clflush_cache_range(addr, bytes);
			break;
		}

		goto out;
	}

	pr_warn("Can't find virtual address for buffer %ul\n", iova);

out:
	spin_unlock_irqrestore(&mine.lock, flags);

	return 0;
}

static int glue_css_load(ia_css_ptr ptr, void *data, size_t bytes)
{
	read_write_set(ptr, data, bytes, CSS2401_READ);
	return 0;
}

static int glue_css_store(ia_css_ptr ptr, const void *data, size_t bytes)
{
	read_write_set(ptr, (void *)data, bytes, CSS2401_WRITE);
	return 0;
}

static int glue_css_set(ia_css_ptr ptr, int c, size_t bytes)
{
	read_write_set(ptr, &c, bytes, CSS2401_SET);
	return 0;
}

static ia_css_ptr glue_css_mmap(const void *ptr, const size_t size,
				uint16_t attribute, void *context)
{
	pr_err("Memory mapping from CSS not supported!\n");
	return 0;
}

static int glue_print_error(const char *fmt, va_list a)
{
	vprintk(fmt, a);
	return 0;
}

static int glue_print_debug(const char *fmt, va_list a)
{
	vprintk(fmt, a);
	return 0;
}

static int filter_match(void *addr)
{
	struct css2600_isys_iomem_filter *filter;
	unsigned long flags;
	int rval = 0;

	spin_lock_irqsave(&mine.lock, flags);
	list_for_each_entry(filter, &mine.filters, head) {
		if (addr < filter->addr || addr > filter->addr + filter->size)
			continue;

		rval = 1;
		break;
	}
	spin_unlock_irqrestore(&mine.lock, flags);

	return rval;
}

int css2600_isys_iomem_filter_add(struct device *dev, void __iomem *addr,
				  size_t size)
{
	struct css2600_isys_iomem_filter *filter =
		kzalloc(sizeof(*filter), GFP_KERNEL);
	unsigned long flags;

	if (!filter)
		return -ENOMEM;

	filter->addr = addr;
	filter->size = size;

	spin_lock_irqsave(&mine.lock, flags);
	list_add(&filter->head, &mine.filters);
	spin_unlock_irqrestore(&mine.lock, flags);

	return 0;
}
EXPORT_SYMBOL_GPL(css2600_isys_iomem_filter_add);

int css2600_isys_iomem_filters_add(struct device *dev, void __iomem **addr,
				   unsigned int naddr, size_t size)
{
	unsigned int i;
	int rval;

	for (i = 0; i < naddr; i++) {
		int rval = css2600_isys_iomem_filter_add(dev, addr[i], size);

		if (rval)
			goto out;
	}

	return 0;

out:
	css2600_isys_iomem_filter_remove(dev);

	return rval;
}
EXPORT_SYMBOL_GPL(css2600_isys_iomem_filters_add);

void css2600_isys_iomem_filter_remove(struct device *dev)
{
	struct css2600_isys_iomem_filter *filter, *safe;
	unsigned long flags;

	if (!dev)
		return;

	spin_lock_irqsave(&mine.lock, flags);
	list_for_each_entry_safe(filter, safe, &mine.filters, head) {
		if (filter->dev != dev)
			continue;

		list_del(&filter->head);
		kfree(filter);
	}
	spin_unlock_irqrestore(&mine.lock, flags);
}
EXPORT_SYMBOL_GPL(css2600_isys_iomem_filter_remove);

static void glue_hw_store8(hrt_address addr, uint8_t data)
{
	addr = (unsigned long)mine.isp_base + (addr & 0x003fffff);

	if (filter_match((void *)addr))
		return;

	writeb(data, (void *)addr);
}

static void glue_hw_store16(hrt_address addr, uint16_t data)
{
	addr = (unsigned long)mine.isp_base + (addr & 0x003fffff);

	if (filter_match((void *)addr))
		return;

	writew(data, (void *)addr);
}

static void glue_hw_store32(hrt_address addr, uint32_t data)
{
	addr = (unsigned long)mine.isp_base + (addr & 0x003fffff);

	if (filter_match((void *)addr))
		return;

	writel(data, (void *)addr);
}

static uint8_t glue_hw_load8(hrt_address addr)
{
	return readb(mine.isp_base + (addr & 0x003fffff));
}

static uint16_t glue_hw_load16(hrt_address addr)
{
	return readw(mine.isp_base + (addr & 0x003fffff));
}

static uint32_t glue_hw_load32(hrt_address addr)
{
	return readl(mine.isp_base + (addr & 0x003fffff));
}

/*Copied from HRT, unefficient???*/
static void glue_mem_store(hrt_address to, const void *from, uint32_t n)
{
	int i;
	hrt_address _to = to;
	const char *_from = (const char *)from;
	for (i = 0; i < n; i++, _to++, _from++)
		glue_hw_store8(_to, *_from);
}

static void glue_mem_load(hrt_address from, void *to, uint32_t n)
{
	int i;
	char *_to = (char *)to;
	hrt_address _from = from;
	for (i = 0; i < n; i++, _to++, _from++)
		*_to = glue_hw_load8(_from);
}

static int css2600_wrapper_ready_for_init(void)
{
	return mine.iommus && mine.iommus_registered >= mine.iommus &&
		mine.domain && mine.dev;
}

int css2600_wrapper_init_done(void)
{
	return css2600_wrapper_ready_for_init() && mine.css_init_done;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_init_done);

static int init_css(void)
{
	struct ia_css_fwctrl_devconfig devconfig;
	struct ia_css_fw css_fw;
	int rval;

	if (!css2600_wrapper_ready_for_init())
		return 0;

	if (mine.css_init_done)
		return 0;

	css_fw.data = (void *)mine.fw->data;
	css_fw.bytes = mine.fw->size;
	rval = ia_css_load_firmware(&mine.css_env, &css_fw);
	if (rval) {
		dev_err(mine.dev, "css load fw failed (%d)\n", rval);
		return -EIO;
	}

	rval = ia_css_init(&mine.css_env, NULL, 0, IA_CSS_IRQ_TYPE_PULSE);
	if (rval) {
		dev_err(mine.dev, "ia_css_init failed (%d)\n", rval);
		return -EIO;
	}

	devconfig.firmware_address = libcss2401_get_sp_fw();
	rval = ia_css_fwctrl_device_open(&devconfig);
	if (rval) {
		dev_err(mine.dev,
			"ia_css_fwctrl_device_open() failed: %d\n", rval);
		return -EIO;
	}

	mine.css_init_done = 1;

	return 0;
}

/*Call this from ISYS driver init*/
void css2600_wrapper_init(void __iomem *basepsys, void __iomem *baseisys,
			  const struct firmware *fw)
{
	INIT_LIST_HEAD(&mine.buffers);
	INIT_LIST_HEAD(&mine.filters);
	spin_lock_init(&mine.lock);

	/*Set PCI base address for HW access*/
	mine.isp_base = basepsys;
	mine.fw = fw;

	/*Map functions to function pointer table*/
	/*IA memory alloc*/
	mine.css_env.cpu_mem_env.alloc = glue_ia_alloc;
	mine.css_env.cpu_mem_env.free = glue_ia_free;
	mine.css_env.cpu_mem_env.flush = glue_ia_flush;

	/*CSS memory alloc*/
	mine.css_env.css_mem_env.alloc = glue_css_alloc;
	mine.css_env.css_mem_env.free = glue_css_free;
	mine.css_env.css_mem_env.store = glue_css_store;
	mine.css_env.css_mem_env.set = glue_css_set;
	mine.css_env.css_mem_env.load = glue_css_load;
	mine.css_env.css_mem_env.mmap = glue_css_mmap;
	/*CSS HW access*/
	mine.css_env.hw_access_env.store_8 = glue_hw_store8;
	mine.css_env.hw_access_env.store_16 = glue_hw_store16;
	mine.css_env.hw_access_env.store_32 = glue_hw_store32;
	mine.css_env.hw_access_env.load_8 = glue_hw_load8;
	mine.css_env.hw_access_env.load_16 = glue_hw_load16;
	mine.css_env.hw_access_env.load_32 = glue_hw_load32;
	mine.css_env.hw_access_env.load = glue_mem_load;
	mine.css_env.hw_access_env.store = glue_mem_store;

	/*Print functions*/
	mine.css_env.print_env.error_print = glue_print_error;
	mine.css_env.print_env.debug_print = glue_print_debug;

	init_css();
}
EXPORT_SYMBOL_GPL(css2600_wrapper_init);

int css2600_wrapper_set_domain(struct iommu_domain *domain)
{
	BUG_ON(mine.domain && mine.domain != domain);

	mine.domain = domain;

	dev_info(mine.dev, "set domain\n");

	return init_css();
}
EXPORT_SYMBOL_GPL(css2600_wrapper_set_domain);

int css2600_wrapper_set_device(struct device *dev)
{
	mine.dev = dev;

	return init_css();
}
EXPORT_SYMBOL_GPL(css2600_wrapper_set_device);

int css2600_wrapper_iommu_add(void)
{
	mine.iommus_registered++;

	dev_info(mine.dev, "registering iommu %d/%d\n",
		 mine.iommus_registered, mine.iommus);

	return init_css();
}
EXPORT_SYMBOL_GPL(css2600_wrapper_iommu_add);

int css2600_wrapper_set_iommus(unsigned int iommus)
{
	mine.iommus = iommus;

	return init_css();
}
EXPORT_SYMBOL_GPL(css2600_wrapper_set_iommus);

MODULE_AUTHOR("Jouni Ukkonen <jouni.ukkonen@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CSS wrapper");
