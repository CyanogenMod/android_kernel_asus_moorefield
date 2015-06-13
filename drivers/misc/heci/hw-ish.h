/*
 * H/W layer of HECI provider device (ISH)
 *
 * Copyright (c) 2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _HECI_HW_ISH_H_
#define _HECI_HW_ISH_H_

#include <linux/pci.h>
#include "hw.h"
#include "hw-ish-regs.h"
#include "heci_dev.h"

struct IPC_MNG_PAYLOAD_TYPE {
	u16 reset_id;
	u16 reserved;
};

struct heci_ish_hw {
	void __iomem *mem_addr;
	/*
	 * hw states of host and fw(ME)
	 */
	u32 host_hw_state;
	u32 ish_hw_state;
	u32 doorbell_val;

};

#define to_ish_hw(dev) (struct heci_ish_hw *)((dev)->hw)

struct heci_device *heci_ish_dev_init(struct pci_dev *pdev);

irqreturn_t heci_ish_irq_quick_handler(int irq, void *dev_id);
irqreturn_t heci_ish_irq_thread_handler(int irq, void *dev_id);

void heci_ish_clr_host_rdy(struct heci_device *dev);
void heci_ish_set_host_rdy(struct heci_device *dev);
bool heci_ish_hw_is_ready(struct heci_device *dev);

#endif /* _HECI_HW_ISH_H_ */
