/*
 * platform_edlp_modem.c: hsi EDLP platform data initilization file
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
#include <linux/hsi/hsi_info_board.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <asm/intel-mid.h>
#include "platform_edlp_modem.h"

#define HSI_CLIENT_CNT	2

static struct hsi_platform_data hsi_platform_info;

/* Conversion table: SFI_MDM_NAME to mdm version */
static struct sfi_to_mdm sfi_to_mdm_assoc_table[] = {
	/* IMC products */
	{"XMM_6260", MODEM_6260},
	{"XMM6260", MODEM_6260},
	{"XMM_6268", MODEM_6268},
	{"XMM6268", MODEM_6268},
	{"XMM_6360", MODEM_6360},
	{"XMM6360", MODEM_6360},
	{"XMM_7160", MODEM_7160},
	{"XMM7160", MODEM_7160},
	{"XMM_7260", MODEM_7260},
	{"XMM7260", MODEM_7260},
	/* Any other IMC products: set to 7160 by default */
	{"XMM", MODEM_7160},
	/* RMC products, not supported */
	{"CYGNUS", MODEM_UNSUP},
	{"PEGASUS", MODEM_UNSUP},
	{"RMC", MODEM_UNSUP},
	/* Whatever it may be, it's not supported */
	{"", MODEM_UNSUP},
};

void edlp_store_modem_ver(const char *modem_name)
{
	int index = 0;

	if (hsi_platform_info.hsi_client_info.mdm_ver == MODEM_UNSUP) {
		/* Retrieve modem ID from modem name */
		while (sfi_to_mdm_assoc_table[index].hsi_name[0]) {
			/* Search for mdm_name in table.
			 * Consider support as far as generic name is
			 * in the table.
			 */
			if (strstr(modem_name,
				sfi_to_mdm_assoc_table[index].hsi_name)) {
				hsi_platform_info.hsi_client_info.mdm_ver =
				sfi_to_mdm_assoc_table[index].modem_type;
			goto out;
				}
				index++;
		}
		hsi_platform_info.hsi_client_info.mdm_ver = MODEM_UNSUP;
	}
out:
	pr_info("%s: update modem version with SFI modem entry ==> %d",
			__func__,
		 hsi_platform_info.hsi_client_info.mdm_ver);
}


void *edlp_modem_platform_data(void *data)
{
	int is_v2, i;
	static const char hsi_char_name[]	= "hsi_char";
	static const char hsi_dlp_name[]	= "hsi-dlp";

	struct hsi_board_info *hsi_sfi_info = (struct hsi_board_info *) data;

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
	};

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

		hsi_info[0].rx_cfg.flow = HSI_FLOW_PIPE;
		hsi_info[1].rx_cfg.flow = HSI_FLOW_PIPE;
		memcpy(&hsi_platform_info.hsi_mid_info, &mid_info_v2,
					sizeof(mid_info_v2));
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

		memcpy(&hsi_platform_info.hsi_mid_info, &mid_info_v1,
			sizeof(mid_info_v1));
	}

	for (i = 0; i < HSI_CLIENT_CNT; i++)
		hsi_info[i].platform_data = (void *)&hsi_platform_info;

	pr_info("HSI EDLP platform data setup\n");
	return &hsi_info[0];
}

void sfi_handle_edlp_dev(struct sfi_device_table_entry *pentry,
						struct devs_id *dev)
{
	struct hsi_board_info hsi_info;
	void *pdata = NULL;

	if (pentry->type == SFI_DEV_TYPE_HSI) {
		pr_info("HSI bus = %d, name = %16.16s, port = %d\n",
				pentry->host_num,
			pentry->name,
			pentry->addr);
		memset(&hsi_info, 0, sizeof(hsi_info));
		hsi_info.name = pentry->name;
		hsi_info.hsi_id = pentry->host_num;
		hsi_info.port = pentry->addr;

		pdata = dev->get_platform_data(&hsi_info);
		if (pdata) {
			hsi_platform_info.hsi_client_info.mdm_ver = MODEM_UNSUP;
			pr_info("SFI register platform data for HSI device %s\n",
					dev->name);
			hsi_register_board_info(pdata, 2);
		}
	} else
		edlp_store_modem_ver(pentry->name);
}

