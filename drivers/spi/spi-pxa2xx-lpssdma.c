/*
 * PXA2xx SPI LPSS DMA support.
 *
 * Copyright (C) 2014, Intel Corporation
 * Author: Huiquan Zhong <huiquan.zhong@intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/init.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <linux/pxa2xx_ssp.h>
#include <linux/sizes.h>
#include <linux/spi/spi.h>
#include <linux/spi/pxa2xx_spi.h>
#include <linux/lpss_dma.h>

#include "spi-pxa2xx.h"

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
		write_SSCR1(read_SSCR1(reg) & ~drv_data->int_cr1, reg);
		write_SSSR_CS(drv_data, drv_data->clear_sr);
		if (!pxa25x_ssp_comp(drv_data))
			write_SSTO(0, reg);

		if (!error) {
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
	struct driver_data *drv_data = data;
	struct chip_data *chip = drv_data->cur_chip;
	void __iomem *reg = drv_data->ioaddr;
	u32 cr1;

	drv_data->tx += drv_data->tx_map_len;
	drv_data->rx += drv_data->rx_map_len;

	/* use interrupt to poll the last bytes */
	cr1 = chip->cr1 | chip->threshold | drv_data->int_cr1;
	write_SSCR1(cr1, reg);
}

bool pxa2xx_spi_dma_is_possible(size_t len)
{
	return len <= MAX_DMA_LEN;
}

int pxa2xx_spi_map_dma_buffers(struct driver_data *drv_data)
{
	const struct chip_data *chip = drv_data->cur_chip;

	if (!chip->enable_dma)
		return 0;

	/* Don't bother with DMA if we can't do even a single burst */
	if (drv_data->len < chip->dma_burst_size)
		return 0;

	if (!drv_data->tx) {
		drv_data->tx = drv_data->dummy;
		drv_data->tx_end = drv_data->tx + drv_data->len;
	}

	if (!drv_data->rx) {
		drv_data->rx = drv_data->dummy;
		drv_data->rx_end = drv_data->rx + drv_data->len;
	}

	return 1;
}

irqreturn_t pxa2xx_spi_dma_transfer(struct driver_data *drv_data)
{
	void *lpss_dma = drv_data->dma_priv;
	u32 status;

	status = read_SSSR(drv_data->ioaddr) & drv_data->mask_sr;
	if (status & SSSR_ROR) {
		dev_err(&drv_data->pdev->dev, "FIFO overrun\n");

		lpss_dma_stop_tx(lpss_dma);
		lpss_dma_stop_rx(lpss_dma);

		pxa2xx_spi_dma_transfer_complete(drv_data, true);
		return IRQ_HANDLED;
	}

	if (status & SSSR_TINT) {
		write_SSSR(SSSR_TINT, drv_data->ioaddr);
		if (drv_data->read(drv_data)) {
			pxa2xx_spi_dma_transfer_complete(drv_data, false);
			return IRQ_HANDLED;
		}
	}

	/* Drain rx fifo, Fill tx fifo and prevent overruns */
	do {
		if (drv_data->read(drv_data)) {
			pxa2xx_spi_dma_transfer_complete(drv_data, false);
			return IRQ_HANDLED;
		}
	} while (drv_data->write(drv_data));

	if (drv_data->read(drv_data)) {
		pxa2xx_spi_dma_transfer_complete(drv_data, false);
		return IRQ_HANDLED;
	}

	return IRQ_HANDLED;
}

int pxa2xx_spi_dma_prepare(struct driver_data *drv_data, u32 dma_burst)
{
	const struct chip_data *chip = drv_data->cur_chip;
	void *lpss_dma = drv_data->dma_priv;
	enum lpss_dma_buswidth width;
	enum lpss_dma_msize burstsize;
	u32 trail_bytes;

	switch (drv_data->n_bytes) {
	case 1:
		width = LPSS_DMA_BUSWIDTH_1_BYTE;
		break;
	case 2:
		width = LPSS_DMA_BUSWIDTH_2_BYTES;
		break;
	case 4:
	default:
		width = LPSS_DMA_BUSWIDTH_4_BYTES;
		break;
	}

	switch (dma_burst) {
	case 8:
		burstsize = LPSS_DMA_MSIZE_8;
		break;
	case 16:
	default:
		burstsize = LPSS_DMA_MSIZE_16;
		break;
	}

	trail_bytes = drv_data->len % (chip->dma_burst_size * drv_data->n_bytes);
	drv_data->tx_map_len = drv_data->rx_map_len = drv_data->len - trail_bytes;

	lpss_dma_set_buswidth(lpss_dma, width);
	lpss_dma_set_burstsize(lpss_dma, burstsize);

	return 0;
}

void pxa2xx_spi_dma_start(struct driver_data *drv_data)
{
	void *lpss_dma = drv_data->dma_priv;

	lpss_dma_start_tx(lpss_dma, drv_data->tx, drv_data->tx_map_len);
	lpss_dma_start_rx(lpss_dma, drv_data->rx, drv_data->rx_map_len);

	atomic_set(&drv_data->dma_running, 1);
}

static struct lpss_dma_info pxa2xx_dma_info = {
	.dma_buswidth = LPSS_DMA_BUSWIDTH_1_BYTE,
	.dma_burstsize = LPSS_DMA_MSIZE_8,
	.rx_callback = pxa2xx_spi_dma_callback,
};

#define LPSS_IDMA_OFFSET	0x800
int pxa2xx_spi_dma_setup(struct driver_data *drv_data)
{
	struct ssp_device *ssp = drv_data->ssp;
	struct lpss_dma_info *dma_info = &pxa2xx_dma_info;
	int ret;

	drv_data->dummy = lpss_dma_alloc(SZ_4K);
	if (!drv_data->dummy)
		return -ENOMEM;

	/* setup lpss dma */
	dma_info->pdev = &drv_data->pdev->dev;
	dma_info->membase = drv_data->ioaddr + LPSS_IDMA_OFFSET;
	dma_info->irq = ssp->irq;
	snprintf(dma_info->name, sizeof(dma_info->name) - 1, "%s_%d", "pxa2xx-spi", ssp->port_id);
	dma_info->per_tx_addr = dma_info->per_rx_addr = drv_data->ssdr_physical;
	dma_info->callback_param = drv_data;

	/* get baytrail or cherrytrail lpss dma info */
	ret = lpss_get_controller_info(lpss_spi_dma, ssp->port_id - 1, dma_info);
	if (ret < 0)
		goto err;

	drv_data->dma_priv = lpss_dma_register(dma_info);
	if (!drv_data->dma_priv) {
		ret = -ENODEV;
		goto err;
	}

	return 0;
err:
	lpss_dma_free(drv_data->dummy);
	return ret;
}

void pxa2xx_spi_dma_release(struct driver_data *drv_data)
{
	void *lpss_dma = drv_data->dma_priv;

	lpss_dma_free(drv_data->dummy);
	lpss_dma_unregister(lpss_dma);
}

void pxa2xx_spi_dma_suspend(struct driver_data *drv_data)
{

}

void pxa2xx_spi_dma_resume(struct driver_data *drv_data)
{

}

int pxa2xx_spi_set_dma_burst_and_threshold(struct chip_data *chip,
					   struct spi_device *spi,
					   u8 bits_per_word, u32 *burst_code,
					   u32 *threshold)
{
	struct pxa2xx_spi_chip *chip_info = spi->controller_data;

	/*
	 * If the DMA burst size is given in chip_info we use that,
	 * otherwise we use the default. Also we use the default FIFO
	 * thresholds for now.
	 */
	*burst_code = chip_info ? chip_info->dma_burst_size : 8;
	*threshold = SSCR1_RxTresh(RX_THRESH_DFLT)
		   | SSCR1_TxTresh(TX_THRESH_DFLT);

	chip->lpss_rx_threshold = SSIRF_RxThresh(*burst_code);

	return 0;
}
