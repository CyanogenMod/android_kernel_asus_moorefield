/*
 * Intel LPSS DMA Controller Driver
 *
 * Copyright (C) 2014, Intel Corporation
 * Authors: Huiquan Zhong <huiquan.zhong@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/dma-mapping.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/mm.h>
#include <linux/device.h>
#include <linux/interrupt.h>
#include <linux/percpu.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/jiffies.h>
#include "lpss_dma.h"

void lpss_dma_dump_reg(struct lpss_dma *dma)
{
	struct lpss_dma_chan *txc, *rxc;

	txc = &dma->tx_chan;
	rxc = &dma->rx_chan;
	pr_debug("<<<<<<<<<<<< LPSS DMA Dump Start >>>>>>>>>>>>\n");
	pr_debug("LPSS DMA Dump for DMA: %s, Channel Base:%d\n",
			dma->dma_name, dma->ch_base);
	pr_debug("DMA_CHAN_EN:\t%#x\n", ioread32(dma->dma_base + LPSS_DMA_CHAN_EN));
	pr_debug("DMA_CFG:\t%#x\n", ioread32(dma->dma_base + LPSS_DMA_CFG));
	pr_debug("STATUS_TFR:\t%#x\n", ioread32(dma->dma_base + LPSS_STATUS_TFR));
	pr_debug("STATUS_ERR:\t%#x\n", ioread32(dma->dma_base + LPSS_STATUS_ERR));
	pr_debug("MASK_TFR:\t%#x\n", ioread32(dma->dma_base + LPSS_MASK_TFR));
	pr_debug("MASK_ERR:\t%#x\n", ioread32(dma->dma_base + LPSS_MASK_ERR));

	pr_debug("LPSS DMA Dump for TX Chan:\n");
	pr_debug("SAR:\t\t%#x\n", ioread32(txc->ch_regs + LPSS_SAR));
	pr_debug("DAR:\t\t%#x\n", ioread32(txc->ch_regs + LPSS_DAR));
	pr_debug("CTL_HI:\t%#x\n", ioread32(txc->ch_regs + LPSS_CTL_HI));
	pr_debug("CTL_LO:\t%#x\n", ioread32(txc->ch_regs + LPSS_CTL_LO));
	pr_debug("CFG_HI:\t%#x\n", ioread32(txc->ch_regs + LPSS_CFG_HI));
	pr_debug("CFG_LO:\t%#x\n", ioread32(txc->ch_regs + LPSS_CFG_LO));

	pr_debug("LPSS DMA Dump for RX Chan:\n");
	pr_debug("SAR:\t\t%#x\n", ioread32(rxc->ch_regs + LPSS_SAR));
	pr_debug("DAR:\t\t%#x\n", ioread32(rxc->ch_regs + LPSS_DAR));
	pr_debug("CTL_HI:\t%#x\n", ioread32(rxc->ch_regs + LPSS_CTL_HI));
	pr_debug("CTL_LO:\t%#x\n", ioread32(rxc->ch_regs + LPSS_CTL_LO));
	pr_debug("CFG_HI:\t%#x\n", ioread32(rxc->ch_regs + LPSS_CFG_HI));
	pr_debug("CFG_LO:\t%#x\n", ioread32(rxc->ch_regs + LPSS_CFG_LO));
	pr_debug("<<<<<<<<<<<< LPSS DMA Dump ends >>>>>>>>>>>>\n");
}

static void lpss_dma_wait_for_suspend(struct lpss_dma_chan *chan)
{
	struct lpss_dma *dma = chan_to_lpss_dma(chan);
	u32 cfg_lo, mask, chan_en;
	int i;
	const int max_loops = 100;

	if (dma->dma_version == LPSS_DMA_V1)
		mask = CH_SUSPEND;
	else
		mask = CH_SUSPEND | CH_DRAIN;

	/* Suspend channel */
	cfg_lo = ioread32(chan->ch_regs + LPSS_CFG_LO);
	cfg_lo |= mask;
	iowrite32(cfg_lo, chan->ch_regs + LPSS_CFG_LO);
	/* wait till FIFO gets empty */
	/* FIFO should be cleared in a couple of milli secs,
	   but most of the time after a 'cpu_relax' */
	for (i = 0; i < max_loops; i++) {
		cfg_lo = ioread32(chan->ch_regs + LPSS_CFG_LO);
		chan_en = ioread32(dma->dma_base + LPSS_DMA_CHAN_EN);
		if (!(chan_en & (1 << chan->ch_id)) || (cfg_lo & FIFO_EMPTY))
			break;
		/* use udelay since this might called from atomic context,
		   and use incremental backoff time */
		if (i)
			udelay(i);
		else
			cpu_relax();
	}

	if (i == max_loops)
		dev_warn(dma->dev, "Waited 5 ms for chan[%d] FIFO to get empty\n",
			chan->ch_id);
	else
		dev_dbg(dma->dev, "waited for %d loops for chan[%d] FIFO to get empty",
			i, chan->ch_id);

	/* disable channel */
	iowrite32(DIS_CHAN(chan->ch_id), dma->dma_base + LPSS_DMA_CHAN_EN);

	cfg_lo = ioread32(chan->ch_regs + LPSS_CFG_LO);
	cfg_lo &= ~mask;
	iowrite32(cfg_lo, chan->ch_regs + LPSS_CFG_LO);

	return;
}

/* --- lpss dma irq and handler --- */
static void lpss_dma_handler(struct lpss_dma *dma)
{
	struct lpss_dma_chan *chan = NULL;
	u32 status, raw_tfr;

	raw_tfr = ioread32(dma->dma_base + LPSS_RAW_TFR);
	status = raw_tfr & dma->intr_mask;

	if (status & (0x1 << dma->ch_base)) {	/* tx chan */
		chan = &dma->tx_chan;
		status &= ~(1 << chan->ch_id);
		iowrite32((1 << chan->ch_id), dma->dma_base + LPSS_CLEAR_TFR);
		lpss_dma_complete(chan, true);
	}

	if (status & (0x2 << dma->ch_base)) {	/* rx chan*/
		chan = &dma->rx_chan;
		status &= ~(1 << chan->ch_id);
		iowrite32((1 << chan->ch_id), dma->dma_base + LPSS_CLEAR_TFR);
		lpss_dma_complete(chan, true);
	}

	status = ioread32(dma->dma_base + LPSS_RAW_ERR);
	if (status)
		dev_err(dma->dev, "%s: raw error status: %#x\n", __func__, status);
}

static irqreturn_t lpss_dma_irq(int irq, void *data)
{
	struct lpss_dma *dma = data;

	u32 tfr_status, err_status;

	if (test_bit(flag_suspend, &dma->flags))
		return IRQ_NONE;

	/* Read the interrupt status registers */
	tfr_status = ioread32(dma->dma_base + LPSS_STATUS_TFR);
	err_status = ioread32(dma->dma_base + LPSS_STATUS_ERR);

	tfr_status &= dma->intr_mask;

	/* Common case if the IRQ is shared with other devices */
	if (!tfr_status && !err_status)
		return IRQ_NONE;

	dev_dbg(dma->dev, "%s: trf_Status %x, Mask %x\n", __func__, tfr_status, dma->intr_mask);

	lpss_dma_handler(dma);

	return IRQ_HANDLED;
}

#define BLOCK_MAX_SIZE_V1	4095	/* 2^12 -1 */
#define BLOCK_MAX_SIZE_V2	262143	/* 2^18 -1 */

static unsigned int lpss_dma_calc_block_ts(int dma_version, size_t len,
			enum lpss_dma_buswidth dma_buswidth)
{
	int byte_size;
	unsigned int block_ts;

	switch (dma_buswidth) {
	case LPSS_DMA_BUSWIDTH_1_BYTE:
		byte_size = 1;
		break;
	case LPSS_DMA_BUSWIDTH_2_BYTES:
		byte_size = 2;
		break;
	case LPSS_DMA_BUSWIDTH_4_BYTES:
	default:
		byte_size = 4;
		break;
	}

	if (dma_version == LPSS_DMA_V1) {
		block_ts = len/byte_size;
		if (block_ts > BLOCK_MAX_SIZE_V1)
			block_ts = BLOCK_MAX_SIZE_V1;
	} else {
		block_ts = len;
		if (len > BLOCK_MAX_SIZE_V2)
			block_ts = BLOCK_MAX_SIZE_V2;
	}

	return block_ts;
}

int lpss_dma_hw_start(struct lpss_dma_chan *chan)
{
	struct lpss_dma *dma = chan_to_lpss_dma(chan);

	chan->ctl_hi = lpss_dma_calc_block_ts(dma->dma_version, chan->len, chan->dma_buswidth);

	dev_dbg(dma->dev, "lpss_dma_start: chan[%d] cnt=%d, ctl_hi=%#x, sar=%#lx, dar=%#lx\n",
		chan->ch_id, chan->dma_cnt, chan->ctl_hi, chan->src_addr, chan->dst_addr);

	/* clear channel interrupt */
	iowrite32((1 << chan->ch_id), dma->dma_base + LPSS_CLEAR_TFR);
	iowrite32((1 << chan->ch_id), dma->dma_base + LPSS_CLEAR_ERR);

	/* enable channel interrupt */
	iowrite32(UNMASK_INT(chan->ch_id), dma->dma_base + LPSS_MASK_TFR);
	set_bit(chan->ch_id, &dma->intr_mask);
	iowrite32(UNMASK_INT(chan->ch_id), dma->dma_base + LPSS_MASK_ERR);

	iowrite32(chan->src_addr, chan->ch_regs + LPSS_SAR);
	iowrite32(chan->dst_addr, chan->ch_regs + LPSS_DAR);
	iowrite32(0, chan->ch_regs + LPSS_LLP);
	iowrite32(chan->cfg_hi, chan->ch_regs + LPSS_CFG_HI);
	iowrite32(chan->cfg_lo, chan->ch_regs + LPSS_CFG_LO);
	iowrite32(chan->ctl_lo, chan->ch_regs + LPSS_CTL_LO);
	iowrite32(chan->ctl_hi, chan->ch_regs + LPSS_CTL_HI);

	/* enable channel*/
	iowrite32(EN_CHAN(chan->ch_id), dma->dma_base + LPSS_DMA_CHAN_EN);

	return 0;
}

void lpss_dma_hw_stop(struct lpss_dma_chan *chan)
{
	struct lpss_dma *dma = chan_to_lpss_dma(chan);

	dev_dbg(dma->dev, "lpss_dma_stop: chan[%d] cnt=%d\n", chan->ch_id, chan->dma_cnt);
	/* disable channel interrupt */
	iowrite32(MASK_INT(chan->ch_id), dma->dma_base + LPSS_MASK_TFR);
	clear_bit(chan->ch_id, &dma->intr_mask);
	iowrite32(MASK_INT(chan->ch_id), dma->dma_base + LPSS_MASK_ERR);

	/* clear channel interrupt */
	iowrite32((1 << chan->ch_id), dma->dma_base + LPSS_CLEAR_TFR);
	iowrite32((1 << chan->ch_id), dma->dma_base + LPSS_CLEAR_ERR);

	lpss_dma_wait_for_suspend(chan);

}

u32 lpss_dma_get_src_addr(struct lpss_dma_chan *chan)
{
	return ioread32(chan->ch_regs + LPSS_SAR);
}

u32 lpss_dma_get_dst_addr(struct lpss_dma_chan *chan)
{
	return ioread32(chan->ch_regs + LPSS_DAR);
}

void lpss_chan_prepare(struct lpss_dma_chan *chan)
{
	struct lpss_dma *dma = chan_to_lpss_dma(chan);
	u32 ctl_lo, cfg_lo, cfg_hi;

	/* CTL_LO */
	ctl_lo = 0x1;	/* Interrupt enable bit */

	ctl_lo |= chan->dma_buswidth << DST_WIDTH_OFF;
	ctl_lo |= chan->dma_buswidth << SRC_WIDTH_OFF;

	ctl_lo |= chan->dma_burstsize << DSR_MSIZE_OFF;
	ctl_lo |= chan->dma_burstsize << SRC_MSIZE_OFF;

	if (chan->dir == LPSS_DMA_TX)
		ctl_lo |= LPSS_MEM_TO_PER;
	else
		ctl_lo |= LPSS_PER_TO_MEM;

	if (dma->dma_version ==  LPSS_DMA_V1) {
		cfg_lo = 0;
		cfg_hi = (0x1 << PROTCTL_OFF | chan->ch_id << SRC_PER_OFF_V1 | FIFO_MODE
			| chan->ch_id << DST_PER_OFF_V1);
	} else {
		cfg_lo = DST_BURST_ALIGN | SRC_BURST_ALIGN;
		cfg_hi = ((chan->ch_id << SRC_PER_OFF_V2) | (chan->ch_id << DST_PER_OFF_V2));
	}

	chan->ctl_lo = ctl_lo;
	chan->cfg_lo = cfg_lo;
	chan->cfg_hi = cfg_hi;

	dev_dbg(dma->dev, "lpss: calc reg: ctl_lo=%#x, cfg_hi=%#x, cfg_lo=%#x\n",
			ctl_lo, cfg_hi, cfg_lo);
}

void enable_lpss_dma(struct lpss_dma *dma)
{
	/*enable dma controller*/
	iowrite32(BIT(0), dma->dma_base + LPSS_DMA_CFG);
}

int lpss_dma_setup(struct lpss_dma *dma)
{
	int err;

	lpss_chan_prepare(&dma->tx_chan);
	lpss_chan_prepare(&dma->rx_chan);

	err = devm_request_irq(dma->dev, dma->irq, lpss_dma_irq,
				IRQF_SHARED, "LPSS_DMA", dma);
	if (err != 0) {
		dev_err(dma->dev, "lpss dma request irq failed\n");
		return err;
	}

	enable_lpss_dma(dma);

	return 0;
}
