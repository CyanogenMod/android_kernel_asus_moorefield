/*
 * hsi_dlp.h
 *
 * Intel Mobile Communication modem protocol driver for DLP
 * (Data Link Protocl (LTE)).
 * This driver is implementing a 5-channel HSI protocol consisting of:
 * - An internal communication control channel;
 * - A multiplexed channel exporting a TTY interface;
 * - Three dedicated high speed channels exporting each a network interface.
 * All channels are using fixed-length pdus, although of different sizes.
 *
 * Copyright (C) 2010-2011 Intel Corporation. All rights reserved.
 *
 * Contact: Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>
 *          Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */
#ifndef _HSI_DLP_H
#define _HSI_DLP_H

#include <linux/ioctl.h>

/**
 * struct hsi_dlp_stats - statistics related to the TX and RX side
 * @data_sz: total size of actual transferred data
 * @pdus_cnt: total number of transferred puds
 * @overflow_cnt: total number of transfer stalls due to FIFO full
 */
struct hsi_dlp_stats {
	unsigned long long	data_sz;
	unsigned int		pdus_cnt;
	unsigned int		overflow_cnt;
};

#define HSI_DLP_MAGIC	0x77

/*
 * HSI_DLP_RESET_TX_STATS	- reset the TX statistics
 */
#define HSI_DLP_RESET_TX_STATS		_IO(HSI_DLP_MAGIC, 32)

/*
 * HSI_DLP_GET_TX_STATS		- get the TX statistics
 */
#define HSI_DLP_GET_TX_STATS		_IOR(HSI_DLP_MAGIC, 32, \
					     struct hsi_dlp_stats)

/*
 * HSI_DLP_RESET_RX_STATS	- reset the RX statistics
 */
#define HSI_DLP_RESET_RX_STATS		_IO(HSI_DLP_MAGIC, 33)

/*
 * HSI_DLP_GET_RX_STATS		- get the RX statistics
 */
#define HSI_DLP_GET_RX_STATS		_IOR(HSI_DLP_MAGIC, 33, \
					     struct hsi_dlp_stats)

/*
 * HSI_DLP_SET_FLASHING_MODE	- Activate/Deactivate the flashing mode
 */
#define HSI_DLP_SET_FLASHING_MODE	_IOW(HSI_DLP_MAGIC, 42, unsigned int)

#endif /* _HSI_DLP_H */

