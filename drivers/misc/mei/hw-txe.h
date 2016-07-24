/*
 *
 * Intel Management Engine Interface (Intel MEI) Linux driver
 * Copyright (c) 2003-2012, Intel Corporation.
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
 */

#ifndef _MEI_HW_TXE_H_
#define _MEI_HW_TXE_H_

#include "hw.h"
#include "hw-txe-regs.h"

#define MEI_TXI_RPM_TIMEOUT    500 /* ms */

extern bool nopg;



/* Flatten Hierarchy interrupt cuase */
#define TXE_INTR_READINESS_BIT  0 /* HISR_INT_0_STS */
#define TXE_INTR_READINESS      HISR_INT_0_STS
#define TXE_INTR_ALIVENESS_BIT  1 /* HISR_INT_1_STS */
#define TXE_INTR_ALIVENESS      HISR_INT_1_STS
#define TXE_INTR_OUT_DB_BIT     2 /* HISR_INT_2_STS */
#define TXE_INTR_OUT_DB         HISR_INT_2_STS
#define TXE_INTR_IN_READY_BIT   8 /* beyond HISR */
#define TXE_INTR_IN_READY       BIT(8)

struct mei_txe_hw;

/**
 * struct mei_txe_hw - txe hardware specifics
 *
 * @mem_addr - BAR0 and BAR1
 * @aliveness - aliveness (power gating) state of the hardware
 * @wait_aliveness_resp  - aliveness wait queue
 * @readiness_state - readiness state of the hardware
 * @intr_cause - translated interrupt cause
 */
struct mei_txe_hw {
	void __iomem *mem_addr[NUM_OF_MEM_BARS];
	u32 aliveness;
	u32 hhier;
	wait_queue_head_t wait_aliveness_resp;

	u32 readiness_state;
	unsigned long intr_cause;

	/** mei mm support */
	struct mei_mm_device *mdev;
	void *pool_vaddr;
	dma_addr_t pool_paddr;
	size_t pool_size;

	void (*pool_release) (struct mei_txe_hw *hw);
};

#define to_txe_hw(dev) (struct mei_txe_hw *)((dev)->hw)

static inline struct mei_device *hw_txe_to_mei(struct mei_txe_hw *hw)
{
	return container_of((void *)hw, struct mei_device, hw);
}

struct mei_device *mei_txe_dev_init(struct pci_dev *pdev);

irqreturn_t mei_txe_irq_quick_handler(int irq, void *dev_id);
irqreturn_t mei_txe_irq_thread_handler(int irq, void *dev_id);

void mei_txe_intr_save(struct mei_device *dev);
void mei_txe_intr_restore(struct mei_device *dev);

int mei_txe_aliveness_set_sync(struct mei_device *dev, u32 req);

int mei_txe_setup_satt2(struct mei_device *dev, phys_addr_t addr, u32 range);

int mei_txe_aliveness_set_sync(struct mei_device *dev, u32 req);

#endif /* _MEI_HW_TXE_H_ */

