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
#include <linux/io.h>

#include <linux/dma-mapping.h>
#include <linux/firmware.h>
#include <linux/iommu.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/spinlock.h>

#include "css2600-bus.h"
#include "css2600-dma.h"
#include "css2600-mmu.h"

#include "libcss2600.h"
#include "vied_subsystem_access_initialization.h"
#include "vied_subsystem_access.h"
#include "shared_memory_access.h"
#include "shared_memory_map.h"

#define ISYS 0
#define PSYS 1

struct base_address {
	void __iomem *isys_base;
	void __iomem *psys_base;
	const struct dma_map_ops *ops;
	spinlock_t lock;
	struct iommu_domain *domain;
	struct list_head buffers;
	uint32_t css_map_done;
	struct mutex memory_mutex_alloc;
	unsigned int iommus, iommus_registered;
	struct device *dev;
} mine;


struct my_css_memory_buffer_item {
	struct list_head list;
	dma_addr_t iova;
	unsigned long *addr;
	size_t bytes;
	struct dma_attrs attrs;
};

/*
 * Css2600 driver set base address for css use
 */
void css2600_wrapper_init(void __iomem *basepsys, void __iomem *baseisys,
			  const struct firmware *fw)
{
	mine.isys_base = baseisys;
	mine.psys_base = basepsys;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_init);

unsigned long long get_hrt_base_address(void)
{
	return 0;
}
EXPORT_SYMBOL_GPL(get_hrt_base_address);


/*
 * Subsystem access functions to access IUNIT MMIO space
 */
#define host_addr(dev, addr)					\
	({							\
		void *addr_host;				\
		if (ISYS == dev)				\
			addr_host = mine.isys_base + addr;	\
		else if (PSYS == dev)				\
			addr_host = mine.psys_base + addr;	\
		addr_host;					\
	})

void vied_subsystem_store_32(vied_subsystem_t dev,
			     vied_subsystem_address_t addr, uint32_t data)
{
	if (dev != ISYS && dev != PSYS)
		return;

	writel(data, host_addr(dev, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_store_32);

void vied_subsystem_store_16(vied_subsystem_t dev,
			     vied_subsystem_address_t addr, uint16_t data)
{
	if (dev != ISYS && dev != PSYS)
		return;

	writew(data, host_addr(dev, addr));

}
EXPORT_SYMBOL_GPL(vied_subsystem_store_16);

void vied_subsystem_store_8(vied_subsystem_t dev,
			     vied_subsystem_address_t addr, uint8_t data)
{
	if (dev != ISYS && dev != PSYS)
		return;

	writeb(data, host_addr(dev, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_store_8);

void vied_subsystem_store(vied_subsystem_t dev,
			  vied_subsystem_address_t addr,
			  const void *data, unsigned int size)
{
	if (dev != ISYS && dev != PSYS)
		return;

	pr_info("%s 0x%x size: %d\n", __FUNCTION__, addr, size);
	memcpy_toio(host_addr(dev, addr), data, size);
}
EXPORT_SYMBOL_GPL(vied_subsystem_store);

uint32_t vied_subsystem_load_32(vied_subsystem_t dev,
				vied_subsystem_address_t addr)
{
	if (dev != ISYS && dev != PSYS)
		return 0;

	return readl(host_addr(dev, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_load_32);

uint16_t vied_subsystem_load_16(vied_subsystem_t dev,
				vied_subsystem_address_t addr)
{
	if (dev != ISYS && dev != PSYS)
		return 0;

	return readw(host_addr(dev, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_load_16);

uint8_t vied_subsystem_load_8(vied_subsystem_t dev,
				vied_subsystem_address_t addr)
{
	if (dev != ISYS && dev != PSYS)
		return 0;

	return readb(host_addr(dev, addr));
}
EXPORT_SYMBOL_GPL(vied_subsystem_load_8);

void vied_subsystem_load(vied_subsystem_t dev,
			 vied_subsystem_address_t addr,
			 void *data, unsigned int size)
{
	if (dev != ISYS && dev != PSYS)
		return;
	pr_info("%s 0x%x size: %d\n", __FUNCTION__, addr, size);
	memcpy_fromio(data, host_addr(dev, addr), size);

}
EXPORT_SYMBOL_GPL(vied_subsystem_load);
/*
 * Initialize base address for subsystem
 */
void vied_subsystem_access_initialize(vied_subsystem_t system)
{

}
EXPORT_SYMBOL_GPL(vied_subsystem_access_initialize);

/*Shared memory access codes written by Dash Biswait, copied from FPGA environment*/

/**
 * \brief Initialize the shared memory interface administration on the host.
 * \param idm: id of ddr memory
 * \param host_ddr_addr: physical address of memory as seen from host
 * \param memory_size: size of ddr memory in bytes
 * \param ps: size of page in bytes (for instance 4096)
 */
int shared_memory_allocation_initialize(vied_memory_t idm, vied_physical_address_t host_ddr_addr, size_t memory_size, size_t ps)
{
	return 0;
}
EXPORT_SYMBOL_GPL(shared_memory_allocation_initialize);

/**
 * \brief De-initialize the shared memory interface administration on the host.
 *
 */
void shared_memory_allocation_uninitialize(vied_memory_t idm)
{
}
EXPORT_SYMBOL_GPL(shared_memory_allocation_uninitialize);

/**
 * \brief Initialize the shared memory interface administration on the host.
 * \param id: id of subsystem
 * \param idm: id of ddr memory
 * \param mmu_ps: size of page in bits
 * \param mmu_pnrs: page numbers
 * \param ddr_addr: base address
 * \param inv_tlb: invalidate tbl
 * \param sbt: set l1 base address
 */
int shared_memory_map_initialize(vied_subsystem_t id, vied_memory_t idm, size_t mmu_ps, size_t mmu_pnrs, vied_physical_address_t ddr_addr, shared_memory_invalidate_mmu_tlb inv_tlb, shared_memory_set_page_table_base_address sbt)
{
	return 0;
}
EXPORT_SYMBOL_GPL(shared_memory_map_initialize);

/**
 * \brief De-initialize the shared memory interface administration on the host.
 *
 */
void shared_memory_map_uninitialize(vied_subsystem_t id, vied_memory_t idm)
{
}
EXPORT_SYMBOL_GPL(shared_memory_map_uninitialize);

static uint8_t alloc_cookie;

/**
 * \brief Allocate (DDR) shared memory space and return a host virtual address. Returns NULL when insufficient memory available
 */
host_virtual_address_t shared_memory_alloc(vied_memory_t idm, size_t bytes)
{

	const struct dma_map_ops *dma_ops = mine.dev->archdata.dma_ops;
	struct my_css_memory_buffer_item *buf;
	unsigned long flags;
	pr_info("%s: in, size: %zu\n", __FUNCTION__, bytes);

	if (!bytes)
		return (unsigned long)&alloc_cookie;

	might_sleep();

	buf = kzalloc(sizeof(*buf), GFP_KERNEL);
	if (!buf)
		return 0;

	/*alloc using css2600 dma driver*/
	buf->bytes = PAGE_ALIGN(bytes);
	buf->addr = dma_ops->alloc(mine.dev, buf->bytes, &buf->iova,
				   GFP_KERNEL, &buf->attrs);

	if (!buf->addr) {
		kfree(buf);
		return 0;
	}

	pr_info("%s: mapping %lu bytes to %p, iova 0x%8.8llx\n", __FUNCTION__ ,buf->bytes,
		 buf->addr, buf->iova);

	spin_lock_irqsave(&mine.lock, flags);
	list_add(&buf->list, &mine.buffers);
	spin_unlock_irqrestore(&mine.lock, flags);

	return (unsigned long)buf->addr;
}
EXPORT_SYMBOL_GPL(shared_memory_alloc);

/**
 * \brief Free (DDR) shared memory space.
 */
void shared_memory_free(vied_memory_t idm, host_virtual_address_t addr)
{
	const struct dma_map_ops *dma_ops = mine.dev->archdata.dma_ops;
	struct my_css_memory_buffer_item *buf = NULL;
	unsigned long flags;

	if ((void *)addr == &alloc_cookie)
		return;

	might_sleep();

	pr_debug("looking for iova %8.8llx\n", addr);

	spin_lock_irqsave(&mine.lock, flags);
	list_for_each_entry(buf, &mine.buffers, list) {
		pr_debug("buffer addr %8.8lx\n", (long)buf->addr);
		if ((long)buf->addr != addr)
			continue;

		pr_debug("found it!\n");
		list_del(&buf->list);
		spin_unlock_irqrestore(&mine.lock, flags);
		dma_ops->free(mine.dev, buf->bytes, buf->addr,
			      buf->iova, &buf->attrs);
		kfree(buf);
		return;
	}
	pr_warn("Can't find mem object %8.8llx\n", addr);
	spin_unlock_irqrestore(&mine.lock, flags);

}
EXPORT_SYMBOL_GPL(shared_memory_free);

/**
 * \brief Convert a host virtual address to a CSS virtual address and update the MMU.
 */
vied_virtual_address_t shared_memory_map(vied_subsystem_t id, vied_memory_t idm, host_virtual_address_t addr)
{
	struct my_css_memory_buffer_item *buf = NULL;
	unsigned long flags;

	if ((void *)addr == &alloc_cookie)
		return 0;

	spin_lock_irqsave(&mine.lock, flags);
	list_for_each_entry(buf, &mine.buffers, list) {
		pr_err("%s %8.8lx\n",__FUNCTION__, (long)buf->addr);
		if ((long)buf->addr != addr)
			continue;

		pr_err("mapped!!\n");
		spin_unlock_irqrestore(&mine.lock, flags);
		return buf->iova;
	}
	pr_warn("Can't find mapped object %8.8llx\n", addr);
	spin_unlock_irqrestore(&mine.lock, flags);
	return 0;
}
EXPORT_SYMBOL_GPL(shared_memory_map);

/**
 * \brief Free a CSS virtual address and update the MMU.
 */
void shared_memory_unmap(vied_subsystem_t id, vied_memory_t idm, vied_virtual_address_t addr)
{
}
EXPORT_SYMBOL_GPL(shared_memory_unmap);

/**
 * \brief Store a byte into (DDR) shared memory space using a host virtual address
 */
void shared_memory_store_8(vied_memory_t idm, host_virtual_address_t addr,
			   uint8_t data)
{
	pr_debug("%s: Enter addr = 0x%llx data = 0x%x\n", __func__, addr, data);
	*((uint8_t *) addr) = data;
	mb();
	/*Invalidate the cache lines to flush the content to ddr.*/
	clflush_cache_range((void *)addr, sizeof(uint8_t));
}
EXPORT_SYMBOL_GPL(shared_memory_store_8);

/**
 * \brief Store a 16-bit word into (DDR) shared memory space using a host virtual address
 */
void shared_memory_store_16(vied_memory_t idm, host_virtual_address_t addr,
			    uint16_t data)
{
	pr_debug("%s: Enter addr = 0x%llx data = 0x%x\n", __func__, addr, data);
	*((uint16_t *) addr) = data;
	mb();
	/*Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint16_t));
}
EXPORT_SYMBOL_GPL(shared_memory_store_16);

/**
 * \brief Store a 32-bit word into (DDR) shared memory space using a host virtual address
 */
void shared_memory_store_32(vied_memory_t idm, host_virtual_address_t addr,
			    uint32_t data)
{
	pr_debug("%s: Enter addr = 0x%llx data = 0x%x\n", __func__, addr,  data);
	*((uint32_t *) addr) = data;
	mb();
	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint32_t));
}
EXPORT_SYMBOL_GPL(shared_memory_store_32);

/**
 * \brief Store a number of bytes into (DDR) shared memory space using a host virtual address
 */
void shared_memory_store(vied_memory_t idm, host_virtual_address_t addr,
			 const void *data, size_t bytes)
{
	pr_debug("%s: Enter addr = 0x%llx\n", __func__, addr);

	if (!data)
		pr_err("%s: data ptr is null\n", __func__);
	else {
		const uint8_t *pdata = data;
		uint8_t *paddr = (uint8_t *) addr;
		size_t i = 0;

		for (; i < bytes; ++i)
			*paddr++ = *pdata++;

		mb();
		/* Invalidate the cache lines to flush the content to ddr. */
		clflush_cache_range((void *)addr, bytes);
	}
}
EXPORT_SYMBOL_GPL(shared_memory_store);

/**
 * \brief Set a number of bytes of (DDR) shared memory space to 0 using a host virtual address
 */
void shared_memory_zero(vied_memory_t idm, host_virtual_address_t addr,
			size_t bytes)
{
	pr_debug("%s: Enter addr = 0x%llx data = 0x%lu\n", __func__, addr,  bytes);
	memset((void *)addr, 0, bytes);
	clflush_cache_range((void *)addr, bytes);
}
EXPORT_SYMBOL_GPL(shared_memory_zero);

/**
 * \brief Load a byte from (DDR) shared memory space using a host virtual address
 */
uint8_t shared_memory_load_8(vied_memory_t idm, host_virtual_address_t addr)
{
	uint8_t data = 0;

	pr_debug("%s: Enter addr = 0x%llx\n", __func__, addr);
	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint8_t));
	mb();
	data = *(uint8_t *)addr;
	return data;
}
EXPORT_SYMBOL_GPL(shared_memory_load_8);

/**
 * \brief Load a 16-bit word from (DDR) shared memory space using a host virtual address
 */
uint16_t shared_memory_load_16(vied_memory_t idm, host_virtual_address_t addr)
{
	uint16_t data = 0;

	pr_debug("%s: Enter addr = 0x%llx\n", __func__, addr);
	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint16_t));
	mb();
	data = *(uint16_t *) addr;
	return data;
}
EXPORT_SYMBOL_GPL(shared_memory_load_16);

/**
 * \brief Load a 32-bit word from (DDR) shared memory space using a host virtual address
 */
uint32_t shared_memory_load_32(vied_memory_t idm, host_virtual_address_t addr)
{
	uint32_t data = 0;

	pr_debug("%s: Enter addr = 0x%llx\n", __func__, addr);
	/* Invalidate the cache lines to flush the content to ddr. */
	clflush_cache_range((void *)addr, sizeof(uint32_t));
	mb();
	data = *(uint32_t *) addr;
	return data;
}
EXPORT_SYMBOL_GPL(shared_memory_load_32);

/**
 * \brief Load a number of bytes from (DDR) shared memory space using a host virtual address
 */
void shared_memory_load(vied_memory_t idm, host_virtual_address_t addr,
			void *data, size_t bytes)
{
	pr_debug("%s: Enter addr = 0x%llx\n", __func__, addr);

	if (!data)
		pr_err("%s: data ptr is null\n", __func__);
	else {
		uint8_t *pdata = data;
		uint8_t *paddr = (uint8_t *) addr;
		size_t i = 0;

		/* Invalidate the cache lines to flush the content to ddr. */
		clflush_cache_range((void *)addr, bytes);
		mb();
		for (; i < bytes; ++i)
			*pdata++ = *paddr++;
	}
}
EXPORT_SYMBOL_GPL(shared_memory_load);

int init_wrapper(void)
{
	mutex_init(&mine.memory_mutex_alloc);
	INIT_LIST_HEAD(&mine.buffers);
	spin_lock_init(&mine.lock);
	return 0;
}

int css2600_wrapper_set_domain(struct iommu_domain *domain)
{
	BUG_ON(mine.domain && mine.domain != domain);

	mine.domain = domain;

	dev_info(mine.dev, "set domain\n");

	return 0;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_set_domain);

int css2600_wrapper_set_device(struct device *dev)
{
	mine.dev = dev;

	return 0;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_set_device);

int css2600_wrapper_iommu_add(void)
{
	mine.iommus_registered++;

	dev_info(mine.dev, "registering iommu %d/%d\n",
		 mine.iommus_registered, mine.iommus);

	return 0;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_iommu_add);

int css2600_wrapper_init_done(void)
{
	if (!mine.iommus || mine.iommus_registered < mine.iommus ||
	    !mine.domain || !mine.dev)
		return 0;

	return 1;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_init_done);

int css2600_wrapper_set_iommus(unsigned int iommus)
{
	mine.iommus = iommus;

	return 0;
}
EXPORT_SYMBOL_GPL(css2600_wrapper_set_iommus);

int css2600_isys_iomem_filters_add(struct device *dev, void __iomem **addr,
				   unsigned int naddr, size_t size)
{
	return 0;
}
EXPORT_SYMBOL_GPL(css2600_isys_iomem_filters_add);

void css2600_isys_iomem_filter_remove(struct device *dev)
{
	return;
}
EXPORT_SYMBOL_GPL(css2600_isys_iomem_filter_remove);

module_init(init_wrapper);
MODULE_AUTHOR("Jouni Ukkonen <jouni.ukkonen@intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CSS wrapper");
