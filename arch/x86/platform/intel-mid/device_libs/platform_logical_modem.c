/*
 * platform_logical_modem.c: logical hsi platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
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
#include "platform_logical_modem.h"

extern int nbr_hsi_clients;

static struct platform_device hsi_netdevice0 = {
	.name = "hsi_net_device",
	.id = 0,
};

static struct platform_device *renesas_devices[] __initdata = {
	&hsi_netdevice0,
};

static struct hsi_board_info hsi_info[HSI_MID_MAX_CLIENTS] = {
	[0] =   {
		.name = "client0",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz */
			.arb_mode	= HSI_ARB_RR,
		},
		.rx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz but used in tx */
			.flow	= HSI_FLOW_PIPE,
		},
	},
	[1] =   {
		.name = "client1",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz */
			.arb_mode	= HSI_ARB_RR,
		},
		.rx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz but used in tx */
			.flow	= HSI_FLOW_PIPE,
		},
	},
	[2] =   {
		.name = "client2",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz */
			.arb_mode	= HSI_ARB_RR,
		},
		.rx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz but used in tx */
			.flow	= HSI_FLOW_PIPE,
		},
	},
	[3] =   {
		.name = "client3",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz */
			.arb_mode	= HSI_ARB_RR,
		},
		.rx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 8,
			.speed		= 200000, /* 200 MHz but used in tx */
			.flow	= HSI_FLOW_PIPE,
		},
	},
	[4] = {
		.name = "hsi_flash",
		.hsi_id = 0,
		.port = 0,
		.tx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 1,
			.speed		= 5000, /* 5 MHz */
			.arb_mode	= HSI_ARB_RR,
		},
		.rx_cfg = {
			.mode		= HSI_MODE_FRAME,
			.channels	= 1,
			.speed		= 0,
			.flow	= HSI_FLOW_SYNC,
		},
	},
};

static struct hsi_mid_platform_data mid_info = {
	.tx_dma_channels[0] = 0,
	.tx_dma_channels[1] = 1,
	.tx_dma_channels[2] = 2,
	.tx_dma_channels[3] = 3,
	.tx_dma_channels[4] = -1,
	.tx_dma_channels[5] = -1,
	.tx_dma_channels[6] = -1,
	.tx_dma_channels[7] = -1,
	.tx_sg_entries[0] = 1,
	.tx_sg_entries[1] = 64,
	.tx_sg_entries[2] = 64,
	.tx_sg_entries[3] = 64,
	.tx_sg_entries[4] = 1,
	.tx_sg_entries[5] = 1,
	.tx_sg_entries[6] = 1,
	.tx_sg_entries[7] = 1,
	.tx_fifo_sizes[0] = 256,
	.tx_fifo_sizes[1] = 256,
	.tx_fifo_sizes[2] = 256,
	.tx_fifo_sizes[3] = 256,
	.tx_fifo_sizes[4] = -1,
	.tx_fifo_sizes[5] = -1,
	.tx_fifo_sizes[6] = -1,
	.tx_fifo_sizes[7] = -1,
	.rx_dma_channels[0] = 4,
	.rx_dma_channels[1] = 5,
	.rx_dma_channels[2] = 6,
	.rx_dma_channels[3] = 7,
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
	.rx_fifo_sizes[0] = 256,
	.rx_fifo_sizes[1] = 256,
	.rx_fifo_sizes[2] = 256,
	.rx_fifo_sizes[3] = 256,
	.rx_fifo_sizes[4] = -1,
	.rx_fifo_sizes[5] = -1,
	.rx_fifo_sizes[6] = -1,
	.rx_fifo_sizes[7] = -1,
};

void __init *logical_platform_data(void *data)
{
	int rst_out = get_gpio_by_name("mdm_rst_out");
	int pwr_on = get_gpio_by_name("mdm_pwr_on");
	int i;

	printk(KERN_INFO "HSI logical platform data setup\n");

	printk(KERN_INFO "HSI GPIOs rst_out:%d, pwr_on:%d, max client:%d\n",
		rst_out,
		pwr_on,
		HSI_MID_MAX_CLIENTS);

	mid_info.gpio_mdm_rst_out = rst_out;
	mid_info.gpio_mdm_pwr_on = pwr_on;
	nbr_hsi_clients = HSI_MID_MAX_CLIENTS;

	for (i = 0; i < HSI_MID_MAX_CLIENTS; i++)
		hsi_info[i].platform_data = (void *)&mid_info;

	platform_add_devices(renesas_devices, ARRAY_SIZE(renesas_devices));

	return &hsi_info[0];
}
