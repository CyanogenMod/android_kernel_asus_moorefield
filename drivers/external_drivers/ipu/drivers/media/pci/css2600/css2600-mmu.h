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

#ifndef CSS2600_MMU_H
#define CSS2600_MMU_H

#include "css2600.h"
#include "css2600-pdata.h"

struct pci_dev;

/*
 * @pgtbl: virtual address of the l1 page table (one page)
 */
struct css2600_mmu_domain {
	uint32_t __iomem *pgtbl;
	struct list_head mmu_devs;
	struct iommu_domain *domain;
	spinlock_t lock;
	unsigned int users;
	struct css2600_dma_mapping *dmap;
	uint32_t dummy_l2_tbl;
	uint32_t dummy_page;
};

/*
 * @pgtbl: physical address of the l1 page table
 */
struct css2600_mmu {
	struct list_head node;
	unsigned int users;

	void __iomem **base;
	unsigned int nr_base;

	phys_addr_t pgtbl;
	struct device *dev;

	struct css2600_dma_mapping *dmap;

	void (*tlb_invalidate)(struct css2600_mmu *mmu);
};

#endif
