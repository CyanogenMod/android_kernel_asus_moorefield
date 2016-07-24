/*
 * PXA2xx SPI DMA engine support.
 *
 * Copyright (C) 2013, Intel Corporation
 * Author: Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/dmaengine.h>
#include <linux/pxa2xx_ssp.h>
#include <linux/scatterlist.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/pci.h>

#include "spi-pxa2xx.h"

static void pxa2xx_spi_unmap_dma_buffers(struct driver_data *drv_data)
{
	struct device *dev = &drv_data->pdev->dev;

	if (!drv_data->dma_mapped)
		return;

	dma_unmap_single(dev, drv_data->rx_dma, drv_data->len, DMA_FROM_DEVICE);
	dma_unmap_single(dev, drv_data->tx_dma, drv_data->len, DMA_TO_DEVICE);

	drv_data->dma_mapped = 0;
}

static void pxa2xx_spi_dma_transfer_complete(struct driver_data *drv_data,
					     bool error)
{
	struct spi_message *msg = drv_data->cur_msg;

	/*
	 * It is possible that one CPU is handling ROR interrupt and other
	 * just gets DMA completion. Calling pump_transfers() twice for the
	 * same transfer leads to problems thus we prevent concurrent calls
	 * by using ->dma_running.
	 */
	if (atomic_dec_and_test(&drv_data->dma_running)) {
		void __iomem *reg = drv_data->ioaddr;

		/*
		 * If the other CPU is still handling the ROR interrupt we
		 * might not know about the error yet. So we re-check the
		 * ROR bit here before we clear the status register.
		 */
		if (!error) {
			u32 status = read_SSSR(reg) & drv_data->mask_sr;
			error = status & SSSR_ROR;
		}

		/* Clear status & disable interrupts */
		write_SSCR1(read_SSCR1(reg) & ~drv_data->dma_cr1, reg);
		write_SSSR_CS(drv_data, drv_data->clear_sr);
		if (!pxa25x_ssp_comp(drv_data))
			write_SSTO(0, reg);

		if (!error) {
			pxa2xx_spi_unmap_dma_buffers(drv_data);

			/* Handle the last bytes of unaligned transfer */
			drv_data->tx += drv_data->tx_map_len;
			drv_data->write(drv_data);

			drv_data->rx += drv_data->rx_map_len;
			drv_data->read(drv_data);

			msg->actual_length += drv_data->len;
			msg->state = pxa2xx_spi_next_transfer(drv_data);
		} else {
			/* In case we got an error we disable the SSP now */
			write_SSCR0(read_SSCR0(reg) & ~SSCR0_SSE, reg);

			msg->state = ERROR_STATE;
		}

		tasklet_schedule(&drv_data->pump_transfers);
	}
}

static void pxa2xx_spi_dma_callback(void *data)
{
	pxa2xx_spi_dma_transfer_complete(data, false);
}

static struct dma_async_tx_descriptor *
pxa2xx_spi_dma_prepare_one(struct driver_data *drv_data,
			   enum dma_transfer_direction dir)
{
	struct pxa2xx_spi_master *pdata = drv_data->master_info;
	struct chip_data *chip = drv_data->cur_chip;
	enum dma_slave_buswidth width;
	enum intel_mid_dma_msize msize;
	enum dma_ctrl_flags flag;
	struct intel_mid_dma_slave *mid_slave;
	struct dma_slave_config *cfg;
	struct dma_chan *chan;
	int ret;

	switch (drv_data->n_bytes) {
	case 1:
		width = DMA_SLAVE_BUSWIDTH_1_BYTE;
		break;
	case 2:
		width = DMA_SLAVE_BUSWIDTH_2_BYTES;
		break;
	default:
		width = DMA_SLAVE_BUSWIDTH_4_BYTES;
		break;
	}

	switch (chip->dma_burst_size) {
	case 1:
		msize = LNW_DMA_MSIZE_1;
		break;
	case 4:
		msize = LNW_DMA_MSIZE_4;
		break;
	case 8:
	default:
		msize = LNW_DMA_MSIZE_8;
		break;
	}

	flag = DMA_PREP_INTERRUPT | DMA_CTRL_ACK;

	if (dir == DMA_MEM_TO_DEV) {
		mid_slave = &drv_data->dmas_tx;
		cfg = &mid_slave->dma_slave;

		mid_slave->hs_mode = LNW_DMA_HW_HS;
		mid_slave->cfg_mode = LNW_DMA_MEM_TO_PER;
		mid_slave->device_instance = drv_data->master->bus_num;

		cfg->direction = dir;
		cfg->src_addr = drv_data->tx_dma;
		cfg->dst_addr = drv_data->ssdr_physical;
		cfg->src_addr_width = width;
		cfg->dst_addr_width = width;
		cfg->src_maxburst = msize;
		cfg->dst_maxburst = msize;
		cfg->slave_id = pdata->tx_slave_id;

		chan = drv_data->tx_chan;
	} else {
		mid_slave = &drv_data->dmas_rx;
		cfg = &mid_slave->dma_slave;

		mid_slave->hs_mode = LNW_DMA_HW_HS;
		mid_slave->cfg_mode = LNW_DMA_PER_TO_MEM;
		mid_slave->device_instance = drv_data->master->bus_num;

		cfg->direction = dir;
		cfg->src_addr = drv_data->ssdr_physical;
		cfg->dst_addr = drv_data->rx_dma;
		cfg->src_addr_width = width;
		cfg->dst_addr_width = width;
		cfg->src_maxburst = msize;
		cfg->dst_maxburst = msize;
		cfg->slave_id = pdata->rx_slave_id;

		chan = drv_data->rx_chan;
	}

	ret = dmaengine_slave_config(chan, cfg);
	if (ret) {
		dev_warn(&drv_data->pdev->dev, "DMA slave config failed\n");
		return NULL;
	}

	return chan->device->device_prep_dma_memcpy(chan, cfg->dst_addr,
			cfg->src_addr, drv_data->len, flag);
}

static bool pxa2xx_spi_dma_filter(struct dma_chan *chan, void *param)
{
	struct pci_dev		*dmac;

	dmac = pci_get_device(PCI_VENDOR_ID_INTEL, PCI_DEVICE_ID_INTEL_GP_DMAC2_MOOR, NULL);
	if (!dmac) {
		pr_err("spi-pxa2xx: Can't find GP DMAC");
		return false;
	}

	if (&dmac->dev == chan->device->dev)
		return true;

	return false;
}

bool pxa2xx_spi_dma_is_possible(size_t len)
{
	return len <= MAX_DMA_LEN;
}

int pxa2xx_spi_map_dma_buffers(struct driver_data *drv_data)
{
	const struct chip_data *chip = drv_data->cur_chip;
	struct device *dev = &drv_data->pdev->dev;

	if (!chip->enable_dma)
		return 0;

	/* Don't bother with DMA if we can't do even a single burst */
	if (drv_data->len < (chip->dma_burst_size * drv_data->n_bytes))
		return 0;

	if (!drv_data->tx) {
		drv_data->tx = drv_data->dummy;
		drv_data->tx_end = drv_data->tx + drv_data->len;
	}
	drv_data->tx_map_len = drv_data->len;
	drv_data->tx_dma = dma_map_single(dev, drv_data->tx, drv_data->tx_map_len,
					DMA_TO_DEVICE);
	if (unlikely(dma_mapping_error(dev, drv_data->tx_dma))) {
		dev_err(dev, "ERROR : tx dma mapping failed\n");
		return 0;
	}

	if (!drv_data->rx) {
		drv_data->rx = drv_data->dummy;
		drv_data->rx_end = drv_data->rx + drv_data->len;
	}
	drv_data->rx_map_len = drv_data->len;
	drv_data->rx_dma = dma_map_single(dev, drv_data->rx, drv_data->rx_map_len,
					DMA_FROM_DEVICE);
	if (unlikely(dma_mapping_error(dev, drv_data->rx_dma))) {
		dma_unmap_single(dev, drv_data->tx_dma,
			drv_data->tx_map_len, DMA_TO_DEVICE);
		dev_err(dev, "ERROR : rx dma mapping failed\n");
		return 0;
	}

	return 1;
}

irqreturn_t pxa2xx_spi_dma_transfer(struct driver_data *drv_data)
{
	u32 status;

	status = read_SSSR(drv_data->ioaddr) & drv_data->mask_sr;
	if (status & SSSR_ROR) {
		dev_err(&drv_data->pdev->dev, "FIFO overrun\n");

		dmaengine_terminate_all(drv_data->rx_chan);
		dmaengine_terminate_all(drv_data->tx_chan);

		pxa2xx_spi_dma_transfer_complete(drv_data, true);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

int pxa2xx_spi_dma_prepare(struct driver_data *drv_data, u32 dma_burst)
{
	struct dma_async_tx_descriptor *tx_desc, *rx_desc;
	struct chip_data *chip = drv_data->cur_chip;

	chip->dma_burst_size = dma_burst;

	tx_desc = pxa2xx_spi_dma_prepare_one(drv_data, DMA_MEM_TO_DEV);
	if (!tx_desc) {
		dev_err(&drv_data->pdev->dev,
			"failed to get DMA TX descriptor\n");
		return -EBUSY;
	}

	rx_desc = pxa2xx_spi_dma_prepare_one(drv_data, DMA_DEV_TO_MEM);
	if (!rx_desc) {
		dev_err(&drv_data->pdev->dev,
			"failed to get DMA RX descriptor\n");
		return -EBUSY;
	}

	/* We are ready when RX completes */
	rx_desc->callback = pxa2xx_spi_dma_callback;
	rx_desc->callback_param = drv_data;

	dmaengine_submit(rx_desc);
	dmaengine_submit(tx_desc);
	return 0;
}

void pxa2xx_spi_dma_start(struct driver_data *drv_data)
{
	dma_async_issue_pending(drv_data->rx_chan);
	dma_async_issue_pending(drv_data->tx_chan);

	atomic_set(&drv_data->dma_running, 1);
}

int pxa2xx_spi_dma_setup(struct driver_data *drv_data)
{
	struct pxa2xx_spi_master *pdata = drv_data->master_info;
	dma_cap_mask_t mask;

	dma_cap_zero(mask);
	dma_cap_set(DMA_SLAVE, mask);

	drv_data->dummy = devm_kzalloc(&drv_data->pdev->dev, SZ_4K, GFP_KERNEL);
	if (!drv_data->dummy)
		return -ENOMEM;

	drv_data->tx_chan = dma_request_channel(mask, pxa2xx_spi_dma_filter,
						pdata);
	if (!drv_data->tx_chan)
		return -ENODEV;

	drv_data->rx_chan = dma_request_channel(mask, pxa2xx_spi_dma_filter,
						pdata);
	if (!drv_data->rx_chan) {
		dma_release_channel(drv_data->tx_chan);
		drv_data->tx_chan = NULL;
		return -ENODEV;
	}

	return 0;
}

void pxa2xx_spi_dma_release(struct driver_data *drv_data)
{
	if (drv_data->rx_chan) {
		dmaengine_terminate_all(drv_data->rx_chan);
		dma_release_channel(drv_data->rx_chan);
		drv_data->rx_chan = NULL;
	}
	if (drv_data->tx_chan) {
		dmaengine_terminate_all(drv_data->tx_chan);
		dma_release_channel(drv_data->tx_chan);
		drv_data->tx_chan = NULL;
	}
}

void pxa2xx_spi_dma_suspend(struct driver_data *drv_data)
{
	struct dma_chan *txchan, *rxchan;

	if (!drv_data->master_info->enable_dma)
		return;

	if (drv_data->rx_chan) {
		rxchan = drv_data->rx_chan;
		rxchan->device->device_control(rxchan, DMA_TERMINATE_ALL, 0);
		rxchan->device->device_control(rxchan, DMA_PAUSE, 0);
	}

	if (drv_data->tx_chan) {
		txchan = drv_data->tx_chan;
		txchan->device->device_control(txchan, DMA_TERMINATE_ALL, 0);
		txchan->device->device_control(txchan, DMA_PAUSE, 0);
	}
}

void pxa2xx_spi_dma_resume(struct driver_data *drv_data)
{
	struct dma_chan *txchan, *rxchan;

	if (!drv_data->master_info->enable_dma)
		return;

	if (drv_data->rx_chan) {
		rxchan = drv_data->rx_chan;
		rxchan->device->device_control(rxchan, DMA_RESUME, 0);
	}

	if (drv_data->tx_chan) {
		txchan = drv_data->tx_chan;
		txchan->device->device_control(txchan, DMA_RESUME, 0);
	}
}

#define SPI_MAX_FIFO 16
int pxa2xx_spi_set_dma_burst_and_threshold(struct chip_data *chip,
					   struct spi_device *spi,
					   u8 bits_per_word, u32 *burst_code,
					   u32 *threshold)
{
	u32 burst_size = 8;

	/* Workaround: less then 1Mhz, need  to set burst_size to 1 */
	if (chip->speed_hz < 1000000 || (bits_per_word == 32 && chip->speed_hz < 8000000))
		burst_size = 1;

	*burst_code = burst_size;

	*threshold = SSCR1_RxTresh(burst_size)
		   | SSCR1_TxTresh(SPI_MAX_FIFO - burst_size);

	return 0;
}
