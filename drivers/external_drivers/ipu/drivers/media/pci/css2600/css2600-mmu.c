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
#include <linux/iommu.h>
#include <linux/iova.h>
#include <linux/module.h>
#include <linux/sizes.h>

#include "css2600.h"
#include "css2600-bus.h"
#include "css2600-dma.h"
#include "css2600-mmu.h"
#include "css2600-wrapper-2401.h"
#include "css2600-regs.h"

#define ISP_PAGE_SHIFT		12
#define ISP_PAGE_SIZE		(1U << ISP_PAGE_SHIFT)
#define ISP_PAGE_MASK		(~(ISP_PAGE_SIZE - 1))

#define ISP_L1PT_SHIFT		22
#define ISP_L1PT_MASK		(~((1U << ISP_L1PT_SHIFT) - 1))

#define ISP_L2PT_SHIFT		12
#define ISP_L2PT_MASK		(~(ISP_L1PT_MASK|(~(ISP_PAGE_MASK))))

#define ISP_L1PT_PTES           1024
#define ISP_L2PT_PTES           1024

#define ISP_PADDR_SHIFT		12

#define REG_TLB_INVALIDATE	0x0000
#define TLB_INVALIDATE		1
#define REG_L1_PHYS		0x0004 /* 27-bit pfn */
#define REG_INFO		0x0008

#define TBL_PHYS_ADDR(a)	((phys_addr_t)(a) << ISP_PADDR_SHIFT)
#define TBL_VIRT_ADDR(a)	phys_to_virt(TBL_PHYS_ADDR(a))

static void mmu_writel(struct css2600_mmu *mmu, uint32_t value,
		       uint32_t offset)
{
	unsigned int i;

	for (i = 0; i < mmu->nr_base; i++)
		writel(value, mmu->base[i] + offset);
}

static void tlb_invalidate(struct css2600_mmu *mmu)
{
	mmu_writel(mmu, TLB_INVALIDATE, REG_TLB_INVALIDATE);
}

#ifdef DEBUG
static void page_table_dump(struct css2600_mmu_domain *adom)
{
	uint32_t l1_idx;

	pr_debug("begin IOMMU page table dump\n");

	for (l1_idx = 0; l1_idx < ISP_L1PT_PTES; l1_idx++) {
		uint32_t l2_idx;
		uint32_t iova = (phys_addr_t)l1_idx << ISP_L1PT_SHIFT;

		if (adom->pgtbl[l1_idx] == adom->dummy_l2_tbl)
			continue;
		pr_debug("l1 entry %u; iovas 0x%8.8x--0x%8.8x, at %p\n",
			 l1_idx, iova, iova + ISP_PAGE_SIZE,
			 (void *)TBL_PHYS_ADDR(adom->pgtbl[l1_idx]));

		for (l2_idx = 0; l2_idx < ISP_L2PT_PTES; l2_idx++) {
			uint32_t *l2_pt = TBL_VIRT_ADDR(adom->pgtbl[l1_idx]);
			uint32_t iova2 = iova + (l2_idx << ISP_L2PT_SHIFT);

			if (l2_pt[l2_idx] == adom->dummy_page)
				continue;

			pr_debug("\tl2 entry %u; iova 0x%8.8x, phys %p\n",
				 l2_idx, iova2,
				 (void *)TBL_PHYS_ADDR(l2_pt[l2_idx]));
		}
	}

	pr_debug("end IOMMU page table dump\n");
}
#endif /* DEBUG */

static uint32_t *alloc_page_table(struct css2600_mmu_domain *adom, bool l1)
{
	uint32_t *pt = (uint32_t *)__get_free_page(GFP_KERNEL | GFP_DMA32);
	int i;

	if (!pt)
		return NULL;

	pr_debug("__get_free_page() == %p\n", pt);

	for (i = 0; i < ISP_L1PT_PTES; i++)
		pt[i] = l1 ? adom->dummy_l2_tbl : adom->dummy_page;

	return pt;
}

static int css2600_mmu_domain_init(struct iommu_domain *domain)
{
	struct css2600_mmu_domain *adom;
	void *ptr;

	adom = kzalloc(sizeof(*adom), GFP_KERNEL);
	if (!adom)
		return -ENOMEM;

	domain->priv = adom;
	adom->domain = domain;

	ptr = (void *)__get_free_page(GFP_KERNEL | GFP_DMA32);
	if (!ptr)
		goto err;

	adom->dummy_page = virt_to_phys(ptr) >> ISP_PAGE_SHIFT;

	ptr = alloc_page_table(adom, false);
	if (!ptr)
		goto err;

	adom->dummy_l2_tbl = virt_to_phys(ptr) >> ISP_PAGE_SHIFT;

	/*
	 * We always map the L1 page table (a single page as well as
	 * the L2 page tables).
	 */
	adom->pgtbl = alloc_page_table(adom, true);
	if (!adom->pgtbl)
		goto err;

	INIT_LIST_HEAD(&adom->mmu_devs);
	spin_lock_init(&adom->lock);

	pr_debug("domain initialised\n");
	pr_debug("ops %p\n", domain->ops);

	return 0;

err:
	free_page((unsigned long)TBL_VIRT_ADDR(adom->dummy_page));
	free_page((unsigned long)TBL_VIRT_ADDR(adom->dummy_l2_tbl));
	kfree(adom);

	return -ENOMEM;
}

static void css2600_mmu_domain_destroy(struct iommu_domain *domain)
{
	struct css2600_mmu_domain *adom = domain->priv;
	uint32_t l1_idx;

	for (l1_idx = 0; l1_idx < ISP_L1PT_PTES; l1_idx++)
		if (adom->pgtbl[l1_idx] != adom->dummy_l2_tbl)
			free_page((unsigned long)
				  TBL_VIRT_ADDR(adom->pgtbl[l1_idx]));

	free_page((unsigned long)TBL_VIRT_ADDR(adom->dummy_page));
	free_page((unsigned long)TBL_VIRT_ADDR(adom->dummy_l2_tbl));
	free_page((unsigned long)adom->pgtbl);
	kfree(adom);
}

static int css2600_mmu_attach_dev(struct iommu_domain *domain,
				  struct device *dev)
{
	struct css2600_mmu_domain *adom = domain->priv;

	spin_lock(&adom->lock);

	adom->users++;

	dev_dbg(dev, "domain attached\n");

	spin_unlock(&adom->lock);

	return css2600_wrapper_set_domain(domain);
}

static void css2600_mmu_detach_dev(struct iommu_domain *domain,
				   struct device *dev)
{
	struct css2600_mmu_domain *adom = domain->priv;

	spin_lock(&adom->lock);

	adom->users--;
	dev_dbg(dev, "domain detached\n");

	spin_unlock(&adom->lock);
}

static int l2_map(struct iommu_domain *domain, unsigned long iova,
		  phys_addr_t paddr, size_t size)
{
	struct css2600_mmu_domain *adom = domain->priv;
	uint32_t l1_idx = iova >> ISP_L1PT_SHIFT;
	uint32_t l1_entry = adom->pgtbl[l1_idx];
	uint32_t *l2_pt;
	uint32_t iova_start = iova;
	unsigned int l2_idx;
	unsigned long flags;

	pr_debug("mapping l2 page table for l1 index %u (iova %8.8x)\n",
		 l1_idx, (uint32_t)iova);

	if (l1_entry == adom->dummy_l2_tbl) {
		uint32_t *l2_virt = alloc_page_table(adom, false);

		if (!l2_virt)
			return -ENOMEM;

		l1_entry = virt_to_phys(l2_virt) >> ISP_PADDR_SHIFT;
		pr_debug("allocated page for l1_idx %u\n", l1_idx);

		spin_lock_irqsave(&adom->lock, flags);
		if (adom->pgtbl[l1_idx] == adom->dummy_l2_tbl) {
			adom->pgtbl[l1_idx] = l1_entry;
#ifdef CONFIG_X86
			clflush_cache_range(&adom->pgtbl[l1_idx],
					    sizeof(adom->pgtbl[l1_idx]));
#endif /* CONFIG_X86 */
		} else {
			spin_unlock_irqrestore(&adom->lock, flags);
			free_page((unsigned long)TBL_VIRT_ADDR(l1_entry));
			spin_lock_irqsave(&adom->lock, flags);
		}
	} else {
		spin_lock_irqsave(&adom->lock, flags);
	}

	l2_pt = TBL_VIRT_ADDR(adom->pgtbl[l1_idx]);

	pr_debug("l2_pt at %p\n", l2_pt);

	paddr = ALIGN(paddr, ISP_PAGE_SIZE);

	l2_idx = (iova_start & ISP_L2PT_MASK) >> ISP_L2PT_SHIFT;

	pr_debug("l2_idx %u, phys 0x%8.8x\n", l2_idx, l2_pt[l2_idx]);
	if (l2_pt[l2_idx] != adom->dummy_page) {
		spin_unlock_irqrestore(&adom->lock, flags);
		return -EBUSY;
	}

	l2_pt[l2_idx] = paddr >> ISP_PADDR_SHIFT;

	spin_unlock_irqrestore(&adom->lock, flags);

#ifdef CONFIG_X86
	clflush_cache_range(&l2_pt[l2_idx], sizeof(l2_pt[l2_idx]));
#endif /* CONFIG_X86 */

	pr_debug("l2 index %u mapped as 0x%8.8x\n", l2_idx,
		 l2_pt[l2_idx]);

	return 0;
}

static int css2600_mmu_map(struct iommu_domain *domain, unsigned long iova,
			   phys_addr_t paddr, size_t size, int prot)
{
	uint32_t iova_start = round_down(iova, ISP_PAGE_SIZE);
	uint32_t iova_end = ALIGN(iova + size, ISP_PAGE_SIZE);

	pr_debug("mapping iova 0x%8.8x--0x%8.8x, size %zu at paddr 0x%10.10llx\n",
		 iova_start, iova_end, size, paddr);

	return l2_map(domain, iova_start, paddr, size);
}

static size_t l2_unmap(struct iommu_domain *domain, unsigned long iova,
		       phys_addr_t dummy, size_t size)
{
	struct css2600_mmu_domain *adom = domain->priv;
	uint32_t l1_idx = iova >> ISP_L1PT_SHIFT;
	uint32_t *l2_pt = TBL_VIRT_ADDR(adom->pgtbl[l1_idx]);
	uint32_t iova_start = iova;
	unsigned int l2_idx;
	size_t unmapped = 0;

	pr_debug("unmapping l2 page table for l1 index %u (iova 0x%8.8lx)\n",
		 l1_idx, iova);

	if (adom->pgtbl[l1_idx] == adom->dummy_l2_tbl)
		return -EINVAL;

	pr_debug("l2_pt at %p\n", l2_pt);

	for (l2_idx = (iova_start & ISP_L2PT_MASK) >> ISP_L2PT_SHIFT;
	     (iova_start & ISP_L1PT_MASK) + (l2_idx << ISP_PAGE_SHIFT)
		     < iova_start + size && l2_idx < ISP_L2PT_PTES;
	     l2_idx++) {
		unsigned long flags;

		pr_debug("l2 index %u unmapped, was 0x%10.10llx\n",
			 l2_idx, TBL_PHYS_ADDR(l2_pt[l2_idx]));
		spin_lock_irqsave(&adom->lock, flags);
		l2_pt[l2_idx] = adom->dummy_page;
		spin_unlock_irqrestore(&adom->lock, flags);
#ifdef CONFIG_X86
		clflush_cache_range(&l2_pt[l2_idx], sizeof(l2_pt[l2_idx]));
#endif /* CONFIG_X86 */
		unmapped++;
	}

	return unmapped << ISP_PAGE_SHIFT;
}

static size_t css2600_mmu_unmap(struct iommu_domain *domain, unsigned long iova,
				size_t size)
{
	return l2_unmap(domain, iova, 0, size);
}

static phys_addr_t css2600_mmu_iova_to_phys(struct iommu_domain *domain,
					    dma_addr_t iova)
{
	struct css2600_mmu_domain *adom = domain->priv;
	uint32_t *l2_pt = TBL_VIRT_ADDR(adom->pgtbl[iova >> ISP_L1PT_SHIFT]);

	return (phys_addr_t)l2_pt[(iova & ISP_L2PT_MASK) >> ISP_L2PT_SHIFT]
		<< ISP_PAGE_SHIFT;
}

static int css2600_mmu_device_group(struct device *dev, unsigned int *groupid)
{
	return 0;
}

static int css2600_mmu_domain_has_cap(struct iommu_domain *domain,
				      unsigned long cap)
{
	return 0;
}

static struct css2600_dma_mapping *alloc_dma_mapping(struct device *dev)
{
	struct css2600_dma_mapping *dmap = kzalloc(sizeof(*dmap), GFP_KERNEL);

	if (!dmap)
		return NULL;

	dmap->domain = iommu_domain_alloc(dev->bus);
	if (!dmap->domain) {
		kfree(dmap);
		return NULL;
	}

	init_iova_domain(&dmap->iovad,
		DMA_BIT_MASK(CSS2600_MMU_ADDRESS_BITS) >> PAGE_SHIFT);

	kref_init(&dmap->ref);

	pr_debug("alloc mapping\n");

	return dmap;
}

static void free_dma_mapping(struct kref *ref)
{
	struct css2600_dma_mapping *dmap =
		container_of(ref, struct css2600_dma_mapping, ref);

	put_iova_domain(&dmap->iovad);
	iommu_domain_free(dmap->domain);
	kfree(dmap);
}

static int css2600_mmu_add_device(struct device *dev)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu;
	struct css2600_dma_mapping *dmap;
	struct css2600_mmu_domain *adom;
	int rval;

	if (!aiommu)
		return 0;

	mmu = dev_get_drvdata(aiommu->dev);

	pr_debug("attach dev %s\n", dev_name(dev));

	if (!mmu)
		return 0;

	mmu->dmap = aiommu->m->mapping;

	if (!mmu->dmap) {
		mmu->dmap = alloc_dma_mapping(dev);
		if (!mmu->dmap)
			return -ENOMEM;
	}
	dmap = mmu->dmap;

	adom = dmap->domain->priv;
	adom->dmap = dmap;
	aiommu->m->mapping = dmap;

	rval = iommu_attach_device(dmap->domain, dev);
	if (rval)
		return rval;

	mmu->pgtbl = virt_to_phys(adom->pgtbl);

	kref_get(&dmap->ref);

	return 0;
}

static void css2600_mmu_remove_device(struct device *dev)
{
	struct css2600_bus_iommu *aiommu = to_css2600_bus_device(dev)->iommu;
	struct css2600_mmu *mmu = dev_get_drvdata(aiommu->dev);

	kref_put(&mmu->dmap->ref, &free_dma_mapping);
}

struct iommu_ops css2600_iommu_ops = {
	.domain_init	= css2600_mmu_domain_init,
	.domain_destroy	= css2600_mmu_domain_destroy,
	.attach_dev	= css2600_mmu_attach_dev,
	.detach_dev	= css2600_mmu_detach_dev,
	.map		= css2600_mmu_map,
	.unmap		= css2600_mmu_unmap,
	.iova_to_phys	= css2600_mmu_iova_to_phys,
	.device_group	= css2600_mmu_device_group,
	.domain_has_cap	= css2600_mmu_domain_has_cap,
	.add_device	= css2600_mmu_add_device,
	.remove_device	= css2600_mmu_remove_device,
	.pgsize_bitmap	= SZ_4K,
};

static int css2600_mmu_probe(struct css2600_bus_device *adev)
{
	struct css2600_mmu_pdata *pdata;
	struct css2600_mmu *mmu;

	mmu = devm_kzalloc(&adev->dev, sizeof(*mmu), GFP_KERNEL);
	if (!mmu)
		return -ENOMEM;

	pdata = adev->pdata;

	iova_cache_get();

	dev_dbg(&adev->dev, "mmu probe %p %p\n", adev, &adev->dev);
	css2600_bus_set_drvdata(adev, mmu);

	css2600_bus_set_iommu(&css2600_iommu_ops);

	mmu->base = pdata->base;
	mmu->nr_base = pdata->nr_base;
	mmu->tlb_invalidate = &tlb_invalidate;
	mmu_writel(mmu, mmu->pgtbl >> ISP_PADDR_SHIFT, REG_L1_PHYS);
	if (pdata->type == CSS2600_MMU_TYPE_CSS2600) {
		if (mmu->nr_base != 2)
			return -EINVAL;
		writel(CSS2600_INFO_REQUEST_DESTINATION_PRIMARY,
		       mmu->base[0] + REG_INFO);
		writel(CSS2600_INFO_REQUEST_DESTINATION_BUT_REGS,
		       mmu->base[1] + REG_INFO);
	}
	/*
	 * FIXME: We can't unload this --- bus_set_iommu() will
	 * register a notifier which must stay until the devices are
	 * gone.
	 */
	__module_get(THIS_MODULE);

	return css2600_wrapper_iommu_add();
}

/*
 * Leave iommu ops as they were --- this means we must be called as
 * the very last.
 */
static void css2600_mmu_remove(struct css2600_bus_device *adev)
{
	iova_cache_put();
	dev_dbg(&adev->dev, "removed\n");
}

static void css2600_mmu_isr(struct css2600_bus_device *adev)
{
	dev_info(&adev->dev, "Yeah!\n");
}

static struct css2600_bus_driver css2600_mmu_driver = {
	.probe = css2600_mmu_probe,
	.remove = css2600_mmu_remove,
	.isr = css2600_mmu_isr,
	.wanted = CSS2600_MMU_NAME,
	.drv = {
		.name = CSS2600_MMU_NAME,
		.owner = THIS_MODULE,
	},
};

module_css2600_bus_driver(css2600_mmu_driver);

MODULE_AUTHOR("Sakari Ailus <sakari.ailus@linux.intel.com>");
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Intel css2600 mmu driver");
