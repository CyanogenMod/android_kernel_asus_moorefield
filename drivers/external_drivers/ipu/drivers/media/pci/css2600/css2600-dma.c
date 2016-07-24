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

#include <asm/cacheflush.h>

#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/gfp.h>
#include <linux/highmem.h>
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/vmalloc.h>

#include "css2600-dma.h"
#include "css2600-mmu.h"

/* Begin of things adapted from arch/arm/mm/dma-mapping.c */
static void __dma_clear_buffer(struct page *page, size_t size)
{
	/*
	 * Ensure that the allocated pages are zeroed, and that any data
	 * lurking in the kernel direct-mapped region is invalidated.
	 */
	if (PageHighMem(page)) {
		while (size > 0) {
			void *ptr = kmap_atomic(page);
			memset(ptr, 0, PAGE_SIZE);
			clflush_cache_range(ptr, PAGE_SIZE);
			kunmap_atomic(ptr);
			page++;
			size -= PAGE_SIZE;
		}
	} else {
		void *ptr = page_address(page);
		memset(ptr, 0, size);
		clflush_cache_range(ptr, size);
	}
}

static struct page **__iommu_alloc_buffer(struct device *dev, size_t size,
					  gfp_t gfp, struct dma_attrs *attrs)
{
	struct page **pages;
	int count = size >> PAGE_SHIFT;
	int array_size = count * sizeof(struct page *);
	int i = 0;

	if (array_size <= PAGE_SIZE)
		pages = kzalloc(array_size, gfp);
	else
		pages = vzalloc(array_size);
	if (!pages)
		return NULL;

	gfp |= __GFP_NOWARN;

	while (count) {
		int j, order = __fls(count);

		pages[i] = alloc_pages(gfp, order);
		while (!pages[i] && order)
			pages[i] = alloc_pages(gfp, --order);
		if (!pages[i])
			goto error;

		if (order) {
			split_page(pages[i], order);
			j = 1 << order;
			while (--j)
				pages[i + j] = pages[i] + j;
		}

		__dma_clear_buffer(pages[i], PAGE_SIZE << order);
		i += 1 << order;
		count -= 1 << order;
	}

	return pages;
error:
	while (i--)
		if (pages[i])
			__free_pages(pages[i], 0);
	if (array_size <= PAGE_SIZE)
		kfree(pages);
	else
		vfree(pages);
	return NULL;
}

static int __iommu_free_buffer(struct device *dev, struct page **pages,
			       size_t size, struct dma_attrs *attrs)
{
	int count = size >> PAGE_SHIFT;
	int array_size = count * sizeof(struct page *);
	int i;

	for (i = 0; i < count; i++)
		if (pages[i])
			__free_pages(pages[i], 0);

	if (array_size <= PAGE_SIZE)
		kfree(pages);
	else
		vfree(pages);
	return 0;
}

/* End of things adapted from arch/arm/mm/dma-mapping.c */

static void *css2600_dma_alloc(struct device *dev, size_t size,
			       dma_addr_t *dma_handle, gfp_t gfp,
			       struct dma_attrs *attrs)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu = dev_get_drvdata(aiommu->dev);
	struct page **pages;
	struct iova *iova;
	struct vm_struct *area;
	int i;
	int rval;

	size = PAGE_ALIGN(size);

	iova = alloc_iova(&mmu->dmap->iovad, size >> PAGE_SHIFT,
		  DMA_BIT_MASK(CSS2600_MMU_ADDRESS_BITS) >> PAGE_SHIFT, 0);
	if (!iova)
		return NULL;

	pages = __iommu_alloc_buffer(dev, size, gfp, attrs);
	if (!pages)
		goto out_free_iova;

	for (i = 0; iova->pfn_lo + i <= iova->pfn_hi; i++) {
		rval = iommu_map(mmu->dmap->domain,
				 (iova->pfn_lo + i) << PAGE_SHIFT,
				 page_to_phys(pages[i]), PAGE_SIZE, 0);
		if (rval)
			goto out_unmap;
	}

	area = __get_vm_area(size, 0, VMALLOC_START, VMALLOC_END);
	if (!area)
		goto out_unmap;

	area->pages = pages;

	if (map_vm_area(area, PAGE_KERNEL, &pages))
		goto out_vunmap;

	*dma_handle = iova->pfn_lo << PAGE_SHIFT;

	mmu->tlb_invalidate(mmu);

	return area->addr;

out_vunmap:
	vunmap(area->addr);

out_unmap:
	__iommu_free_buffer(dev, pages, size, attrs);
	for (i--; i >= 0; i--) {
		iommu_unmap(mmu->dmap->domain, (iova->pfn_lo + i) << PAGE_SHIFT,
			    PAGE_SIZE);
	}
out_free_iova:
	__free_iova(&mmu->dmap->iovad, iova);

	return NULL;
}

static void css2600_dma_free(struct device *dev, size_t size, void *vaddr,
			     dma_addr_t dma_handle, struct dma_attrs *attrs)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu = dev_get_drvdata(aiommu->dev);
	struct vm_struct *area = find_vm_area(vaddr);
	struct iova *iova = find_iova(&mmu->dmap->iovad,
				      dma_handle >> PAGE_SHIFT);

	if (WARN_ON(!area))
		return;

	if (WARN_ON(!area->pages))
		return;

	size = PAGE_ALIGN(size);

	iommu_unmap(mmu->dmap->domain, iova->pfn_lo << PAGE_SHIFT,
		    (iova->pfn_hi - iova->pfn_lo + 1) << PAGE_SHIFT);

	__free_iova(&mmu->dmap->iovad, iova);

	__iommu_free_buffer(dev, area->pages, size, attrs);

	vunmap(vaddr);

	mmu->tlb_invalidate(mmu);
}

static int css2600_dma_mmap(struct device *dev, struct vm_area_struct *vma,
			    void *addr, dma_addr_t iova, size_t size,
			    struct dma_attrs *attrs)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct vm_struct *area = find_vm_area(addr);
	size_t count = PAGE_ALIGN(size) >> PAGE_SHIFT;
	size_t i;

	if (!area)
		return -EFAULT;

	if (vma->vm_start & ~PAGE_MASK)
		return -EINVAL;

	if (size > area->size)
		return -EFAULT;

	for (i = 0; i < count; i++)
		vm_insert_page(vma, vma->vm_start + (i << PAGE_SHIFT),
			       area->pages[i]);

	return 0;
}

static void css2600_dma_unmap_sg(struct device *dev, struct scatterlist *sglist,
				 int nents, enum dma_data_direction dir,
				 struct dma_attrs *attrs)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu = dev_get_drvdata(aiommu->dev);
	struct iova *iova = find_iova(&mmu->dmap->iovad,
				      sg_dma_address(sglist) >> PAGE_SHIFT);

	if (!nents)
		return;

	BUG_ON(!iova);

	iommu_unmap(mmu->dmap->domain, iova->pfn_lo << PAGE_SHIFT,
		    (iova->pfn_hi - iova->pfn_lo + 1) << PAGE_SHIFT);

	mmu->tlb_invalidate(mmu);

	__free_iova(&mmu->dmap->iovad, iova);
}

static int css2600_dma_map_sg(struct device *dev, struct scatterlist *sglist,
			      int nents, enum dma_data_direction dir,
			      struct dma_attrs *attrs)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu = dev_get_drvdata(aiommu->dev);
	struct scatterlist *sg;
	struct iova *iova;
	size_t size = 0;
	uint32_t iova_addr;
	int i;

	for_each_sg(sglist, sg, nents, i)
		size += PAGE_ALIGN(sg->length) >> PAGE_SHIFT;

	dev_dbg(dev, "mapping sg %d entries, %zu pages\n", nents, size);

	iova = alloc_iova(&mmu->dmap->iovad, size,
		  DMA_BIT_MASK(CSS2600_MMU_ADDRESS_BITS) >> PAGE_SHIFT, 0);
	if (!iova)
		return 0;

	dev_dbg(dev, "iova low pfn %lu, high pfn %lu\n", iova->pfn_lo,
		iova->pfn_hi);

	iova_addr = iova->pfn_lo;

	for_each_sg(sglist, sg, nents, i) {
		int rval;

		dev_dbg(dev,
			"mapping entry %d: iova 0x%8.8x, physical 0x%16.16llx\n",
			i, iova_addr << PAGE_SHIFT, page_to_phys(sg_page(sg)));
		rval = iommu_map(mmu->dmap->domain, iova_addr << PAGE_SHIFT,
				 page_to_phys(sg_page(sg)),
				 PAGE_ALIGN(sg->length), 0);
		if (rval)
			goto out_fail;
		sg_dma_address(sg) = iova_addr << PAGE_SHIFT;
#ifdef CONFIG_NEED_SG_DMA_LENGTH
		sg_dma_len(sg) = sg->length;
#endif /* CONFIG_NEED_SG_DMA_LENGTH */

		iova_addr += PAGE_ALIGN(sg->length) >> PAGE_SHIFT;
	}

	mmu->tlb_invalidate(mmu);

	return nents;

out_fail:
	css2600_dma_unmap_sg(dev, sglist, i, dir, attrs);

	return 0;
}

static void css2600_dma_sync_single_for_cpu(
	struct device *dev, dma_addr_t dma_handle, size_t size,
	enum dma_data_direction dir)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu = dev_get_drvdata(aiommu->dev);

	clflush_cache_range(
		phys_to_virt(iommu_iova_to_phys(
				     mmu->dmap->domain, dma_handle)), size);
}

static void css2600_dma_sync_sg_for_cpu(
	struct device *dev, struct scatterlist *sglist, int nents,
	enum dma_data_direction dir)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu = dev_get_drvdata(aiommu->dev);
	struct scatterlist *sg;
	int i;

	for_each_sg(sglist, sg, nents, i) {
		clflush_cache_range(
			phys_to_virt(iommu_iova_to_phys(
					     mmu->dmap->domain,
					     sg_dma_address(sg))),
			sg->length);
	}
}

struct dma_map_ops css2600_dma_ops = {
	.alloc = css2600_dma_alloc,
	.free = css2600_dma_free,
	.mmap = css2600_dma_mmap,
	.map_sg = css2600_dma_map_sg,
	.unmap_sg = css2600_dma_unmap_sg,
	.sync_single_for_cpu = css2600_dma_sync_single_for_cpu,
	.sync_single_for_device = css2600_dma_sync_single_for_cpu,
	.sync_sg_for_cpu = css2600_dma_sync_sg_for_cpu,
	.sync_sg_for_device = css2600_dma_sync_sg_for_cpu,
};
EXPORT_SYMBOL_GPL(css2600_dma_ops);
