/*
 * Intel LPSS DMA Internal Header File
 *
 * Copyright (C) 2014, Intel Corporation
 * Authors: Huiquan Zhong <huiquan.zhong@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef __LPSS_DMA_H__
#define __LPSS_DMA_H__

#include <linux/interrupt.h>
#include <linux/dma-direction.h>
#include <linux/lpss_dma.h>

#define INT_MASK_WE	0x8

#define EN_CHAN(ch)		\
	((BIT(8) << ch) | (BIT(0) << ch))

#define DIS_CHAN(ch)		\
	(BIT(8) << ch)

#define UNMASK_INT(ch)	\
	((BIT(8) << ch) | (BIT(0) << ch))

#define MASK_INT(ch)		\
	(BIT(8) << ch)

#define LPSS_DMA_CH_SIZE	0x58

/* lpss dma register and bits*/
#define LPSS_SAR		0x00 /* Source Address Register*/
#define LPSS_DAR		0x08 /* Destination Address Register*/
#define LPSS_LLP		0x10 /* Linked List Pointer Register*/
#define LPSS_CTL_LO		0x18 /* Control Register*/
#define LPSS_CTL_HI		0x1C /* Control Register*/
#define LPSS_CFG_LO		0x40 /* Configuration Register Low*/
#define LPSS_CFG_HI		0x44 /* Configuration Register high*/

#define LPSS_STATUS_TFR		0x2E8
#define LPSS_STATUS_BLOCK	0x2F0
#define LPSS_STATUS_ERR		0x308

#define LPSS_RAW_TFR		0x2C0
#define LPSS_RAW_BLOCK		0x2C8
#define LPSS_RAW_ERR		0x2E0

#define LPSS_MASK_TFR		0x310
#define LPSS_MASK_BLOCK		0x318
#define LPSS_MASK_SRC_TRAN	0x320
#define LPSS_MASK_DST_TRAN	0x328
#define LPSS_MASK_ERR		0x330

#define LPSS_CLEAR_TFR		0x338
#define LPSS_CLEAR_BLOCK	0x340
#define LPSS_CLEAR_SRC_TRAN	0x348
#define LPSS_CLEAR_DST_TRAN	0x350
#define LPSS_CLEAR_ERR		0x358

#define LPSS_INTR_STATUS	0x360
#define LPSS_DMA_CFG		0x398
#define LPSS_DMA_CHAN_EN	0x3A0

#define DST_WIDTH_OFF		1
#define SRC_WIDTH_OFF		4
#define DSR_MSIZE_OFF		11
#define SRC_MSIZE_OFF		14

#define LPSS_MEM_TO_PER		(BIT(8) | BIT(20))
#define LPSS_PER_TO_MEM		(BIT(10) | BIT(21))
#define FIFO_MODE		BIT(1)

#define PROTCTL_OFF		2
#define SRC_PER_OFF_V1		7
#define DST_PER_OFF_V1		11

#define DST_BURST_ALIGN		BIT(0)
#define SRC_BURST_ALIGN		BIT(1)

#define SRC_PER_OFF_V2		0
#define DST_PER_OFF_V2		4

#define CH_SUSPEND		(BIT(8))
#define CH_DRAIN		(BIT(10))

#define FIFO_EMPTY		(BIT(9))

struct lpss_dma_chan_dev {
	struct lpss_dma_chan *chan;
	struct device device;
	int dev_id;
	atomic_t *idr_ref;
};

enum lpss_dma_direction {
	LPSS_DMA_TX = 0,
	LPSS_DMA_RX = 1,
};

struct lpss_dma_chan {
	void __iomem            *ch_regs;

	int			ch_id;

	enum lpss_dma_direction	dir;
	enum lpss_dma_buswidth	dma_buswidth;
	enum lpss_dma_msize	dma_burstsize;
	dma_addr_t 		src_addr;
	dma_addr_t 		dst_addr;
	size_t			len;

	u32			ctl_lo;
	u32			ctl_hi;
	u32			cfg_lo;
	u32			cfg_hi;

	spinlock_t              lock;
	bool                    busy;
	size_t			complete_len;

	unsigned int 		dma_cnt;

	struct lpss_dma_chan_dev *dev;

	lpss_dma_tx_callback callback;
	void *callback_param;

	struct completion *chan_comp;
};

enum {
	flag_inited = 0,
	flag_suspend,
};

struct lpss_dma {
	struct device           *dev;
	unsigned int            irq;
	void __iomem            *dma_base;
	unsigned long 		len;
	int			ch_base;
	int			dma_version;
	unsigned long		flags;
	unsigned long		intr_mask;

	struct lpss_dma_chan 	tx_chan;
	struct lpss_dma_chan 	rx_chan;

	struct tasklet_struct   tasklet;

	char 			dma_name[DMA_NAME_SIZE];
};

static inline struct lpss_dma *chan_to_lpss_dma(struct lpss_dma_chan *chan)
{
	if (chan->dir == LPSS_DMA_TX)
		return container_of(chan, struct lpss_dma, tx_chan);
	else
		return container_of(chan, struct lpss_dma, rx_chan);
}

void lpss_dma_complete(struct lpss_dma_chan *chan, bool call);

int lpss_dma_setup(struct lpss_dma *dma);

int lpss_dma_hw_start(struct lpss_dma_chan *chan);
void lpss_dma_hw_stop(struct lpss_dma_chan *chan);

u32 lpss_dma_get_src_addr(struct lpss_dma_chan *txc);
u32 lpss_dma_get_dst_addr(struct lpss_dma_chan *rxc);

void enable_lpss_dma(struct lpss_dma *dma);
void lpss_chan_prepare(struct lpss_dma_chan *chan);

#endif
