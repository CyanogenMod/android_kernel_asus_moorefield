/*
 * Intel LPSS DMA Core Framework
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
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/percpu.h>
#include <linux/mutex.h>
#include <linux/jiffies.h>
#include <linux/pm_runtime.h>
#include "lpss_dma.h"

static struct lpss_dma_chan *dev_to_dma_chan(struct device *dev)
{
	struct lpss_dma_chan_dev *chan_dev;

	chan_dev = container_of(dev, typeof(*chan_dev), device);
	return chan_dev->chan;
}

static ssize_t show_dma_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lpss_dma_chan *chan = dev_to_dma_chan(dev);
	struct lpss_dma *dma = chan_to_lpss_dma(chan);
	int ret;

	if (chan->dir == LPSS_DMA_TX)
		ret = sprintf(buf, "%s-tx\n", dma->dma_name);
	else
		ret = sprintf(buf, "%s-rx\n", dma->dma_name);

	return ret;
}

static ssize_t show_busy(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lpss_dma_chan *chan;
	int err;

	chan = dev_to_dma_chan(dev);
	if (chan)
		err = sprintf(buf, "%d\n", chan->busy);
	else
		err = -ENODEV;

	return err;
}

static ssize_t show_dma_cnt(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct lpss_dma_chan *chan;
	int err;

	chan = dev_to_dma_chan(dev);
	if (chan)
		err = sprintf(buf, "%d\n", chan->dma_cnt);
	else
		err = -ENODEV;

	return err;
}

static struct device_attribute lpss_dma_attrs[] = {
	__ATTR(name, S_IRUSR, show_dma_name, NULL),
	__ATTR(busy, S_IRUSR, show_busy, NULL),
	__ATTR(dma_cnt, S_IRUSR, show_dma_cnt, NULL),
	__ATTR_NULL
};

static void chan_dev_release(struct device *dev)
{
	struct lpss_dma_chan_dev *chan_dev;

	chan_dev = container_of(dev, typeof(*chan_dev), device);

	kfree(chan_dev);
}

static struct class lpss_dma_class = {
	.name		= "lpss_dma",
	.dev_attrs	= lpss_dma_attrs,
	.dev_release	= chan_dev_release,
};

static void lpss_dma_lock(struct lpss_dma_chan *chan, unsigned long *flags)
{
	struct lpss_dma *dma = chan_to_lpss_dma(chan);

	disable_irq(dma->irq);
	spin_lock_irqsave(&chan->lock, *flags);
}

static void lpss_dma_unlock(struct lpss_dma_chan *chan, unsigned long *flags)
{
	struct lpss_dma *dma = chan_to_lpss_dma(chan);

	spin_unlock_irqrestore(&chan->lock, *flags);
	enable_irq(dma->irq);
}

void lpss_dma_complete(struct lpss_dma_chan *chan, bool call)
{
	struct lpss_dma *dma = chan_to_lpss_dma(chan);
	lpss_dma_tx_callback callback_txd = NULL;
	void *param_txd = NULL;

	if (chan->dir == LPSS_DMA_TX)
		dma_unmap_single(dma->dev, chan->src_addr, chan->len, DMA_TO_DEVICE);
	else {
		dma_sync_single_for_cpu(dma->dev, chan->dst_addr,
				chan->len, DMA_FROM_DEVICE);
		dma_unmap_single(dma->dev, chan->dst_addr, chan->len, DMA_FROM_DEVICE);
	}

	if (call) {
		chan->busy = false;
		chan->complete_len = chan->len;
	}

	if (!chan->complete_len)
		return;

	if (chan->callback) {
		callback_txd = chan->callback;
		param_txd = chan->callback_param;
		callback_txd(param_txd);
	}

	if (chan->chan_comp) {
		complete(chan->chan_comp);
	}
}

static void _lpss_dma_stop_chan(struct lpss_dma_chan *chan)
{
	lpss_dma_hw_stop(chan);
	chan->busy = false;
	if (chan->dir == LPSS_DMA_TX)
		chan->complete_len = lpss_dma_get_src_addr(chan) - chan->src_addr;
	else
		chan->complete_len = lpss_dma_get_dst_addr(chan) - chan->dst_addr;
	lpss_dma_complete(chan, false);
}

int lpss_dma_start_tx(void *lpss_dma, void *src, size_t len)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *txc = &dma->tx_chan;
	unsigned long flags;
	int ret;

	BUG_ON(!dma);
	BUG_ON(!len);

	if (test_bit(flag_suspend, &dma->flags))
		return -EINVAL;

	lpss_dma_lock(txc, &flags);
	if (unlikely(txc->busy)) {
		lpss_dma_unlock(txc, &flags);
		dev_err(dma->dev, "%s: %s tx channel is busy\n", __func__, dma->dma_name);
		return -EBUSY;
	}
	txc->busy = true;
	txc->complete_len = 0;
	txc->src_addr = dma_map_single(dma->dev, src, len, DMA_TO_DEVICE);
	txc->len = len;
	txc->dma_cnt ++;

	ret = lpss_dma_hw_start(txc);

	lpss_dma_unlock(txc, &flags);

	return ret;
}

int lpss_dma_stop_tx(void *lpss_dma)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *txc = &dma->tx_chan;
	unsigned long flags;

	BUG_ON(!dma);

	if (test_bit(flag_suspend, &dma->flags))
		return -EINVAL;

	lpss_dma_lock(txc, &flags);
	if (!txc->busy) {
		lpss_dma_unlock(txc, &flags);
		return -EPERM;
	}

	_lpss_dma_stop_chan(txc);

	lpss_dma_unlock(txc, &flags);

	return 0;
}

int lpss_dma_start_rx(void *lpss_dma, void *dst, size_t len)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *rxc = &dma->rx_chan;
	unsigned long flags;
	int ret;

	BUG_ON(!dma);
	BUG_ON(!len);

	if (test_bit(flag_suspend, &dma->flags))
		return -EINVAL;

	lpss_dma_lock(rxc, &flags);
	if (unlikely(rxc->busy)) {
		lpss_dma_unlock(rxc, &flags);
		dev_err(dma->dev, "%s: %s rx channel is busy\n", __func__, dma->dma_name);
		return -EBUSY;
	}
	rxc->busy = true;
	rxc->complete_len = 0;
	rxc->dst_addr = dma_map_single(dma->dev, dst, len, DMA_FROM_DEVICE);
	rxc->len = len;
	rxc->dma_cnt ++;

	ret = lpss_dma_hw_start(rxc);

	lpss_dma_unlock(rxc, &flags);

	return ret;
}

int lpss_dma_stop_rx(void *lpss_dma)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *rxc = &dma->rx_chan;
	unsigned long flags;

	BUG_ON(!dma);

	if (test_bit(flag_suspend, &dma->flags))
		return -EINVAL;

	lpss_dma_lock(rxc, &flags);
	if (!rxc->busy) {
		lpss_dma_unlock(rxc, &flags);
		return -EPERM;
	}

	_lpss_dma_stop_chan(rxc);

	lpss_dma_unlock(rxc, &flags);

	return 0;
}

size_t lpss_dma_get_tx_count(void *lpss_dma)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *txc = &dma->tx_chan;

	BUG_ON(!dma);

	return txc->complete_len;
}

size_t lpss_dma_get_rx_count(void *lpss_dma)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *rxc = &dma->rx_chan;

	BUG_ON(!dma);

	return rxc->complete_len;
}

int lpss_dma_set_buswidth(void *lpss_dma, enum lpss_dma_buswidth buswidth)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *txc = &dma->tx_chan;
	struct lpss_dma_chan *rxc = &dma->rx_chan;

	BUG_ON(!dma);

	txc->dma_buswidth = rxc->dma_buswidth = buswidth;

	lpss_chan_prepare(txc);
	lpss_chan_prepare(rxc);

	return 0;
}

int lpss_dma_set_burstsize(void *lpss_dma, enum lpss_dma_msize burstsize)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *txc = &dma->tx_chan;
	struct lpss_dma_chan *rxc = &dma->rx_chan;

	BUG_ON(!dma);

	txc->dma_burstsize = rxc->dma_burstsize = burstsize;

	lpss_chan_prepare(txc);
	lpss_chan_prepare(rxc);

	return 0;
}

void lpss_dma_resume(void *lpss_dma)
{
	struct lpss_dma *dma = lpss_dma;

	BUG_ON(!dma);

	dev_dbg(dma->dev, "lpss: %s\n", __func__);

	pm_runtime_get_sync(dma->dev);

	enable_lpss_dma(dma);

	clear_bit(flag_suspend, &dma->flags);

	return;
}

void lpss_dma_suspend(void *lpss_dma)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *txc = &dma->tx_chan;
	struct lpss_dma_chan *rxc = &dma->rx_chan;
	unsigned long flags;

	BUG_ON(!dma);

	dev_dbg(dma->dev, "lpss: %s\n", __func__);

	set_bit(flag_suspend, &dma->flags);

	/* If TX, RX channel is busying, should stop first */
	lpss_dma_lock(txc, &flags);
	if (txc->busy)
		_lpss_dma_stop_chan(txc);
	lpss_dma_unlock(txc, &flags);

	lpss_dma_lock(rxc, &flags);
	if (rxc->busy)
		_lpss_dma_stop_chan(rxc);
	lpss_dma_unlock(rxc, &flags);

	pm_runtime_put(dma->dev);

	return;
}

int lpss_dma_sysfs_init(struct lpss_dma *dma)
{
	struct lpss_dma_chan *txc, *rxc;
	static int dev_id;
	int ret;

	/* 1. tx channel sysfs init */
	txc = &dma->tx_chan;
	txc->dev = kzalloc(sizeof(*txc->dev), GFP_KERNEL);
	if (txc->dev == NULL) {
		dev_err(dma->dev, "%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	txc->dev->device.class = &lpss_dma_class;
	txc->dev->device.parent = dma->dev;
	txc->dev->chan = txc;
	txc->dev->dev_id = dev_id;

	dev_set_name(&txc->dev->device, "dma%dchan%d",
			dev_id, txc->ch_id - dma->ch_base);

	ret = device_register(&txc->dev->device);
	if (ret < 0) {
		dev_err(dma->dev, "%s: device_register failed\n", __func__);
		kfree(txc->dev);
		return ret;
	}

	/* 2, rx channel sysfs init */
	rxc = &dma->rx_chan;
	rxc->dev = kzalloc(sizeof(*rxc->dev), GFP_KERNEL);
	if (rxc->dev == NULL) {
		dev_err(dma->dev, "%s: kzalloc failed\n", __func__);
		return -ENOMEM;
	}

	rxc->dev->device.class = &lpss_dma_class;
	rxc->dev->device.parent = dma->dev;
	rxc->dev->chan = rxc;
	rxc->dev->dev_id = dev_id;

	dev_set_name(&rxc->dev->device, "dma%dchan%d",
			dev_id, rxc->ch_id - dma->ch_base);

	ret = device_register(&rxc->dev->device);
	if (ret < 0) {
		dev_err(dma->dev, "%s: device_register failed\n", __func__);
		kfree(rxc->dev);
		return ret;
	}

	dev_id ++;

	return ret;
}

void *lpss_dma_register(struct lpss_dma_info *info)
{
	struct lpss_dma	*dma;
	struct lpss_dma_chan *txc, *rxc;
	int err;

	BUG_ON(!info);

	dma = devm_kzalloc(info->pdev, sizeof(struct lpss_dma), GFP_KERNEL);
	if (!dma) {
		dev_err(info->pdev, "%s: kzalloc failed probe\n", __func__);
		return NULL;
	}

	dma->dev = info->pdev;
	dma->dma_base = info->membase;
	dma->irq = info->irq;
	dma->ch_base = info->ch_base;
	dma->dma_version = info->dma_version;
	strcpy(dma->dma_name, info->name);

	/* 1. tx channel setup */
	txc = &dma->tx_chan;
	txc->ch_id = dma->ch_base;
	txc->ch_regs = dma->dma_base + (LPSS_DMA_CH_SIZE * txc->ch_id);
	txc->dir = LPSS_DMA_TX;
	txc->dma_buswidth = info->dma_buswidth;
	txc->dma_burstsize = info->dma_burstsize;
	txc->dst_addr = info->per_tx_addr;
	if (info->tx_callback) {
		txc->callback = info->tx_callback;
		txc->callback_param = info->callback_param;
	}
	if (info->tx_comp) {
		txc->chan_comp = info->tx_comp;
	}
	spin_lock_init(&txc->lock);

	/* 2. rx channel setup */
	rxc = &dma->rx_chan;
	rxc->ch_id = dma->ch_base + 1;
	rxc->ch_regs = dma->dma_base + (LPSS_DMA_CH_SIZE * rxc->ch_id);
	rxc->dir = LPSS_DMA_RX;
	rxc->dma_buswidth = info->dma_buswidth;
	rxc->dma_burstsize = info->dma_burstsize;
	rxc->src_addr = info->per_rx_addr;
	if (info->rx_callback) {
		rxc->callback = info->rx_callback;
		rxc->callback_param = info->callback_param;
	}
	if (info->rx_comp) {
		rxc->chan_comp = info->rx_comp;
	}
	spin_lock_init(&rxc->lock);

	lpss_dma_sysfs_init(dma);

	err = lpss_dma_setup(dma);
	if (err < 0)
		return NULL;

	return dma;
}

void lpss_dma_unregister(void *lpss_dma)
{
	struct lpss_dma *dma = lpss_dma;
	struct lpss_dma_chan *txc, *rxc;

	BUG_ON(!dma);

	txc = &dma->tx_chan;
	txc->dev->chan = NULL;
	device_unregister(&txc->dev->device);

	rxc = &dma->rx_chan;
	rxc->dev->chan = NULL;
	device_unregister(&rxc->dev->device);

	devm_kfree(dma->dev, dma);
}

void *lpss_dma_alloc(size_t len)
{
	if (len <= 0)
		return NULL;

	return kzalloc(len, GFP_KERNEL);
}

void lpss_dma_free(void *addr)
{
	kfree(addr);
}

static int __init lpss_dma_init(void)
{
	return class_register(&lpss_dma_class);
}

arch_initcall(lpss_dma_init);
