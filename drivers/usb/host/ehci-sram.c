/*
 * EHCI HCD (Host Controller Driver) SRAM Glue.
 *
 */
#ifndef CONFIG_HAVE_GENERIC_DMA_COHERENT

#include <linux/slab.h>
#include <linux/dmapool.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/dma-mapping.h>
#include <linux/device.h>
#include <linux/list.h>
#include <linux/export.h>
#include <linux/mutex.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/stat.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/types.h>
#include <linux/wait.h>
#include "ehci-sram.h"

struct dma_coherent_mem {
	void            *virt_base;
	dma_addr_t      device_base;
	phys_addr_t     pfn_base;
	int             size;
	int             flags;
	unsigned long   *bitmap;
};


static void *ehci_sram_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle,
			     gfp_t gfp, struct dma_attrs *attrs);
static void ehci_sram_free(struct device *dev, size_t size, void *vaddr,
			   dma_addr_t dma_handle, struct dma_attrs *attrs);

static struct dma_map_ops sram_ops;
static struct dma_map_ops *dma_ops_org;

/* NOTE: the following routine should be in arch/x86/include/asm/dma-mapping.h
 * along with get_dma_ops.
 */
static inline void set_dma_ops(struct device *dev, struct dma_map_ops *ops)
{
#ifdef CONFIG_X86_DEV_DMA_OPS
	dev->archdata.dma_ops = ops;
#endif
}

/* Attach SRAM to device */
int ehci_sram_declare(struct device *dev, dma_addr_t bus_addr,
		      dma_addr_t device_addr, size_t size, int flags)
{
	void __iomem *mem_base = NULL;
	int pages = size >> PAGE_SHIFT;
	int bitmap_size = BITS_TO_LONGS(pages) * sizeof(long);

	dma_ops_org = get_dma_ops(dev);

	if ((flags & (DMA_MEMORY_MAP | DMA_MEMORY_IO)) == 0)
		goto out;
	if (!size)
		goto out;
	if (dev->dma_mem)
		goto out;

	mem_base = ioremap(bus_addr, size);
	if (!mem_base)
		goto out;

	dev->dma_mem = kzalloc(sizeof(struct dma_coherent_mem), GFP_KERNEL);
	if (!dev->dma_mem)
		goto out;
	dev->dma_mem->bitmap = kzalloc(bitmap_size, GFP_KERNEL);
	if (!dev->dma_mem->bitmap)
		goto free1_out;

	dev->dma_mem->virt_base = mem_base;
	dev->dma_mem->device_base = device_addr;
	dev->dma_mem->pfn_base = PFN_DOWN(bus_addr);
	dev->dma_mem->size = pages;
	dev->dma_mem->flags = flags;

	memcpy(&sram_ops, dma_ops_org, sizeof(struct dma_map_ops));
	sram_ops.alloc = ehci_sram_alloc;
	sram_ops.free = ehci_sram_free;
	set_dma_ops(dev, &sram_ops);

	return 1;

 free1_out:
	kfree(dev->dma_mem);
 out:
	if (mem_base)
		iounmap(mem_base);
	return 0;
}

/* Detach SRAM from device */
void ehci_sram_release(struct device *dev)
{
	struct dma_coherent_mem *mem = dev->dma_mem;

	if (!mem)
		return;
	set_dma_ops(dev, dma_ops_org);
	dev->dma_mem = NULL;
	iounmap(mem->virt_base);
	kfree(mem->bitmap);
	kfree(mem);
}

/**
 * ehci_sram_alloc() - try to allocate memory from the SRAM
 *
 * @dev:	device from which we allocate memory
 * @size:	size of requested memory area
 * @dma_handle:	This will be filled with the correct dma handle
 *
 * This function allocate a block from SRAM.
 *
 * Returns the virtual address to allocated area, or NULL if failed.
 */
void *ehci_sram_alloc(struct device *dev, size_t size, dma_addr_t *dma_handle,
		      gfp_t gfp, struct dma_attrs *attrs)
{
	struct dma_coherent_mem *mem;
	int order = get_order(size);
	int pageno;
	void *pret;

	if (!dev)
		return 0;
	mem = dev->dma_mem;
	if (!mem)
		return 0;

	pret = NULL;

	if (unlikely(size > (mem->size << PAGE_SHIFT)))
		goto err;

	pageno = bitmap_find_free_region(mem->bitmap, mem->size, order);
	if (unlikely(pageno < 0))
		goto err;

	/*
	 * Memory was found in the per-device area.
	 */
	*dma_handle = mem->device_base + (pageno << PAGE_SHIFT);
	pret = mem->virt_base + (pageno << PAGE_SHIFT);
	memset(pret, 0, size);

	return pret;

err:
	/*
	 * In the case where the allocation can not be satisfied from the
	 * per-device area, try to fall back to generic memory if the
	 * constraints allow it.
	 */
	if (dma_ops_org->alloc)
		return dma_ops_org->alloc(dev, size, dma_handle, gfp, attrs);

	return NULL;
}

/**
 * ehci_sram_free() - try to free the memory allocated from SRAM
 * @dev:	device from which the memory was allocated
 * @size:	size of memory area
 * @vaddr:	virtual address of allocated pages
 *
 * This checks whether the memory was allocated from the SRAM
 * and if so, releases that memory.
 *
 * Returns 1 if we correctly released the memory, or 0 if not
 */
void ehci_sram_free(struct device *dev, size_t size, void *vaddr,
		    dma_addr_t dma_handle, struct dma_attrs *attrs)
{
	struct dma_coherent_mem *mem = dev ? dev->dma_mem : NULL;
	int order = get_order(size);

	if (mem && vaddr >= mem->virt_base && vaddr <
		   (mem->virt_base + (mem->size << PAGE_SHIFT))) {
		int page = (vaddr - mem->virt_base) >> PAGE_SHIFT;

		bitmap_release_region(mem->bitmap, page, order);
	} else if (mem && dma_ops_org->free) {
		dma_ops_org->free(dev, size, vaddr, dma_handle, attrs);
	}
}

#endif
