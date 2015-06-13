/*
 * lpss_dma.h - Intel LPSS DMA Header File
 *
 * Copyright (C) 2014, Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This driver supports the following platform
 *	Baytrail, Cherrytrail, Broxton
 */

#ifndef LPSS_DMA_H_
#define LPSS_DMA_H_

/* DMA Burst size */
enum lpss_dma_msize {
	LPSS_DMA_MSIZE_1 = 0x0,
	LPSS_DMA_MSIZE_4 = 0x1,
	LPSS_DMA_MSIZE_8 = 0x2,
	LPSS_DMA_MSIZE_16 = 0x3,
};

/* DMA transfer width */
enum lpss_dma_buswidth {
	LPSS_DMA_BUSWIDTH_1_BYTE = 0x0,
	LPSS_DMA_BUSWIDTH_2_BYTES = 0x1,
	LPSS_DMA_BUSWIDTH_4_BYTES = 0x2,
};

#define DMA_NAME_SIZE 32

#define LPSS_DMA_V1	1	/* for BYT and CHT platform */
#define LPSS_DMA_V2	0	/* for BXT platform */

typedef void (*lpss_dma_tx_callback)(void *lpss_lpss_dmaaram);

/* LPSS DMA platform info */
struct lpss_dma_info {
	struct device *pdev;
	void __iomem   *membase;
	unsigned int irq;
	int dma_version;
	int ch_base;

	char name[DMA_NAME_SIZE];

	dma_addr_t per_tx_addr;
	dma_addr_t per_rx_addr;
	enum lpss_dma_buswidth dma_buswidth;
	enum lpss_dma_msize dma_burstsize;

	lpss_dma_tx_callback tx_callback;
	lpss_dma_tx_callback rx_callback;
	void *callback_param;

	struct completion *tx_comp;
	struct completion *rx_comp;
};

/* --- LPSS DMA API --- */
void *lpss_dma_register(struct lpss_dma_info *info);
void lpss_dma_unregister(void *lpss_dma);

void *lpss_dma_alloc(size_t len);
void lpss_dma_free(void *addr);

int lpss_dma_start_tx(void *lpss_dma, void *src, size_t len);
int lpss_dma_stop_tx(void *lpss_dma);
size_t lpss_dma_get_tx_count(void *lpss_dma);

int lpss_dma_start_rx(void *lpss_dma, void *dest, size_t len);
int lpss_dma_stop_rx(void *lpss_dma);
size_t lpss_dma_get_rx_count(void *lpss_dma);

int lpss_dma_set_buswidth(void *lpss_dma, enum lpss_dma_buswidth);
int lpss_dma_set_burstsize(void *lpss_dma, enum lpss_dma_msize);

void lpss_dma_resume(void *lpss_dma);
void lpss_dma_suspend(void *lpss_dma);

/* dma channel user type, for byt and cht platform */
enum {
	lpss_hsu_dma,
	lpss_spi_dma,
	lpss_i2c_dma,
};

#ifdef CONFIG_BYT_LPSS_DMA
int lpss_get_controller_info(int dma_type, int index, struct lpss_dma_info *dma_info);
#else
static inline int lpss_get_controller_info(int dma_type, int index, struct lpss_dma_info *dma_info)
{
	return 0;
}
#endif
#endif
