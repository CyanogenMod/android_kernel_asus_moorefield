/*
 * platform_hsi.c: hsi platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/hsi/hsi.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <asm/intel-mid.h>
#include "platform_hsi_modem.h"


#if ((defined CONFIG_HSI_FFL_TTY) && (defined CONFIG_HSI_DLP))
/* Selection of HSI_FFL_TTY and HSI_DLP.
 * Needed for co-existance of FFL and DLP drivers for CTPSCALE
 * Defaulting to HSI_DLP to not break CTP products except CTPSCALELT */
#undef CONFIG_HSI_FFL_TTY
#define HSI_CLIENT_CNT  2

#elif (defined(CONFIG_HSI_FFL_TTY) || defined(CONFIG_HSI_DLP))
#define HSI_CLIENT_CNT	2

#else
#define HSI_CLIENT_CNT	1
#endif

extern int nbr_hsi_clients;

void *hsi_modem_platform_data(void *data)
{
	int is_v2;
	static const char hsi_char_name[]	= "hsi_char";
#if defined(CONFIG_HSI_FFL_TTY)
	static const char hsi_ffl_name[]	= "hsi-ffl";
#elif defined(CONFIG_HSI_DLP)
	static const char hsi_dlp_name[]	= "hsi-dlp";
#endif

	static struct hsi_board_info hsi_info[HSI_CLIENT_CNT] = {
		[0] = {
			.name = hsi_char_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 200000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		},
#if defined(CONFIG_HSI_FFL_TTY)
		[1] = {
			.name = hsi_ffl_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 100000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		}
#elif defined(CONFIG_HSI_DLP)
		[1] = {
			.name = hsi_dlp_name,
			.hsi_id = 0,
			.port = 0,
			.archdata = NULL,
			.tx_cfg.speed = 100000,	/* tx clock, kHz */
			.tx_cfg.channels = 8,
			.tx_cfg.mode = HSI_MODE_FRAME,
			.tx_cfg.arb_mode = HSI_ARB_RR,
			.rx_cfg.flow = HSI_FLOW_SYNC,
			.rx_cfg.mode = HSI_MODE_FRAME,
			.rx_cfg.channels = 8
		}
#endif
	};

#if defined(CONFIG_HSI_FFL_TTY)
	static struct hsi_mid_platform_data mid_info = {
		.tx_dma_channels[0] = -1,
		.tx_dma_channels[1] = 5,
		.tx_dma_channels[2] = -1,
		.tx_dma_channels[3] = -1,
		.tx_dma_channels[4] = -1,
		.tx_dma_channels[5] = -1,
		.tx_dma_channels[6] = -1,
		.tx_dma_channels[7] = -1,
		.tx_sg_entries[0] = 1,
		.tx_sg_entries[1] = 1,
		.tx_sg_entries[2] = 1,
		.tx_sg_entries[3] = 1,
		.tx_sg_entries[4] = 1,
		.tx_sg_entries[5] = 1,
		.tx_sg_entries[6] = 1,
		.tx_sg_entries[7] = 1,
		.tx_fifo_sizes[0] = -1,
		.tx_fifo_sizes[1] = 1024,
		.tx_fifo_sizes[2] = -1,
		.tx_fifo_sizes[3] = -1,
		.tx_fifo_sizes[4] = -1,
		.tx_fifo_sizes[5] = -1,
		.tx_fifo_sizes[6] = -1,
		.tx_fifo_sizes[7] = -1,
		.rx_dma_channels[0] = -1,
		.rx_dma_channels[1] = 1,
		.rx_dma_channels[2] = -1,
		.rx_dma_channels[3] = -1,
		.rx_dma_channels[4] = -1,
		.rx_dma_channels[5] = -1,
		.rx_dma_channels[6] = -1,
		.rx_dma_channels[7] = -1,
		.rx_sg_entries[0] = 1,
		.rx_sg_entries[1] = 1,
		.rx_sg_entries[2] = 1,
		.rx_sg_entries[3] = 1,
		.rx_sg_entries[4] = 1,
		.rx_sg_entries[5] = 1,
		.rx_sg_entries[6] = 1,
		.rx_sg_entries[7] = 1,
		.rx_fifo_sizes[0] = -1,
		.rx_fifo_sizes[1] = 1024,
		.rx_fifo_sizes[2] = -1,
		.rx_fifo_sizes[3] = -1,
		.rx_fifo_sizes[4] = -1,
		.rx_fifo_sizes[5] = -1,
		.rx_fifo_sizes[6] = -1,
		.rx_fifo_sizes[7] = -1,
	};

#elif defined(CONFIG_HSI_DLP)
	/* HSI IP v2 ? */
	is_v2 = (SPID_PLATFORM_ID(INTEL, MRFL, PHONE) ||
			 SPID_PLATFORM_ID(INTEL, MRFL, TABLET));
	if (is_v2) {
		static struct hsi_mid_platform_data mid_info_v2 = {
		  /* TX/RX DMA channels mapping */
		  .tx_dma_channels = {  0,  1,  2,  3,  4,  -1, -1,  -1},
		  .rx_dma_channels = {  5,  6,  7,  8,  9,  -1, -1,  -1},

		  /* TX/RX FIFOs sizes */
		  .tx_fifo_sizes = { 256, 256, 256, 256, 256,  -1,  -1,  -1},
		  .rx_fifo_sizes = { 256, 256, 256, 256, 256,  -1,  -1,  -1},

		  /* RX/TX Threshold */
		  .tx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},
		  .rx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},

		  /* TX/RX SG entries count */
		  .tx_sg_entries = {  1,  1,  64,  64,  64,  1,  1,  1},
		  .rx_sg_entries = {  1,  1,   1,   1,   1,  1,  1,  1}
		};

		hsi_info[0].platform_data = (void *)&mid_info_v2;
		hsi_info[1].platform_data = (void *)&mid_info_v2;
	} else {
		static struct hsi_mid_platform_data mid_info_v1 = {
		  /* TX/RX DMA channels mapping */
		  .tx_dma_channels = {  -1,  0,  1,  2,  3,  -1, -1,  -1},
		  .rx_dma_channels = {  -1,  4,  5,  6,  7,  -1, -1,  -1},

		  /* TX/RX FIFOs sizes */
		  .tx_fifo_sizes = { 128, 128, 256, 256, 256,  -1,  -1,  -1},
		  .rx_fifo_sizes = { 128, 128, 256, 256, 256,  -1,  -1,  -1},

		  /* RX/TX Threshold */
		  .tx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},
		  .rx_fifo_thres = { 32, 32, 32, 32, 32, 32, 32, 32},

		  /* TX/RX SG entries count */
		  .tx_sg_entries = {  1,  1,  64,  64,  64,  1,  1,  1},
		  .rx_sg_entries = {  1,  1,   1,   1,   1,  1,  1,  1}
		};

		hsi_info[0].platform_data = (void *)&mid_info_v1;
		hsi_info[1].platform_data = (void *)&mid_info_v1;
	};
#else
	static struct hsi_mid_platform_data mid_info = {};
#endif

#if defined(CONFIG_HSI_FFL_TTY)
	hsi_info[0].platform_data = (void *)&mid_info;
	hsi_info[1].platform_data = (void *)&mid_info;
#endif

	pr_info("HSI platform data setup\n");
	nbr_hsi_clients = HSI_CLIENT_CNT;

	return &hsi_info[0];
}


struct hsi_mid_pci_platform_data pci_pdata;

static struct hsi_mid_pci_platform_data *intel_hsi_mid_get_pci_pdata(
	struct pci_dev *pdev)
{
	pci_pdata.gpio_wake = get_gpio_by_name("hsi_cawake");
	pci_pdata.use_oob_cawake = SPID_PLATFORM_ID(INTEL, MRFL, PHONE) ||
		SPID_PLATFORM_ID(INTEL, MRFL, TABLET);

	return &pci_pdata;
}

static void intel_hsi_mid_pci_early_quirks(struct pci_dev *pdev)
{
	pdev->dev.platform_data = intel_hsi_mid_get_pci_pdata(pdev);
}

DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, HSI_PNW_PCI_DEVICE_ID,
			intel_hsi_mid_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, HSI_CLV_PCI_DEVICE_ID,
			intel_hsi_mid_pci_early_quirks);
DECLARE_PCI_FIXUP_EARLY(PCI_VENDOR_ID_INTEL, HSI_TNG_PCI_DEVICE_ID,
			intel_hsi_mid_pci_early_quirks);
