/*
 *
 * hsi_ffl_tty.h
 *
 * Fixed frame length modem protocol over HSI: IOCTL definitions
 *
 * Copyright (C) 2010 Intel Corporation. All rights reserved.
 *
 * Contact: Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>
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
#ifndef _HSI_FFL_TTY_H
#define _HSI_FFL_TTY_H

#include <linux/ioctl.h>

/**
 * struct hsi_ffl_stats - statistics related to the TX and RX side
 * @data_sz: total size of actual transferred data
 * @frame_cnt: total number of transferred frames
 * @overflow_cnt: total number of transfer stalls due to FIFO full
 */
struct hsi_ffl_stats {
	unsigned long long	data_sz;
	unsigned int		frame_cnt;
	unsigned int		overflow_cnt;
};

#define FFL_TTY_MAGIC	0x77

/*
 * FFL_TTY_RESET_TX	-	 reset the TX state machine (flushes it)
 */
#define FFL_TTY_RESET_TX		_IO(FFL_TTY_MAGIC, 0)

/*
 * FFL_TTY_RESET_RX	-	 reset the RX state machine (flushes it)
 */
#define FFL_TTY_RESET_RX		_IO(FFL_TTY_MAGIC, 1)

/*
 * FFL_TTY_GET_TX_STATE	-	 get the current state of the TX state machine
 */
#define FFL_TTY_GET_TX_STATE		_IOR(FFL_TTY_MAGIC, 2, unsigned int)

/*
 * FFL_TTY_GET_RX_STATE	-	 get the current state of the RX state machine
 */
#define FFL_TTY_GET_RX_STATE		_IOR(FFL_TTY_MAGIC, 3, unsigned int)

/*
 * FFL_TTY_MODEM_RESET		- reset the modem (solicited reset)
 * Shared with SPI
 */
#define FFL_TTY_MODEM_RESET		_IO(FFL_TTY_MAGIC, 4)

/*
 * FFL_TTY_MODEM_STATE		- return 1 if first transmission completed
 * Shared with SPI
 */
#define FFL_TTY_MODEM_STATE		_IOR(FFL_TTY_MAGIC, 5, int)

/*
 * FFL_TTY_GET_HANGUP_REASON	- return reason for latest hangup
 * Shared with SPI
 */
#define FFL_TTY_GET_HANGUP_REASON	_IOR(FFL_TTY_MAGIC, 6, int)

/*
 * FFL_TTY_SET_TX_WAIT_MAX	- set the maximal size of the TX waiting FIFO
 */
#define FFL_TTY_SET_TX_WAIT_MAX		_IOW(FFL_TTY_MAGIC, 8, unsigned int)

/*
 * FFL_TTY_GET_TX_WAIT_MAX	- get the maximal size of the TX waiting FIFO
 */
#define FFL_TTY_GET_TX_WAIT_MAX		_IOR(FFL_TTY_MAGIC, 8, unsigned int)

/*
 * FFL_TTY_SET_RX_WAIT_MAX	- set the maximal size of the RX waiting FIFO
 */
#define FFL_TTY_SET_RX_WAIT_MAX		_IOW(FFL_TTY_MAGIC, 9, unsigned int)

/*
 * FFL_TTY_GET_RX_WAIT_MAX	- get the maximal size of the RX waiting FIFO
 */
#define FFL_TTY_GET_RX_WAIT_MAX		_IOR(FFL_TTY_MAGIC, 9, unsigned int)

/*
 * FFL_TTY_SET_TX_CTRL_MAX	- set the maximal size of the TX controller FIFO
 */
#define FFL_TTY_SET_TX_CTRL_MAX		_IOW(FFL_TTY_MAGIC, 10, unsigned int)

/*
 * FFL_TTY_GET_TX_CTRL_MAX	- get the maximal size of the TX controller FIFO
 */
#define FFL_TTY_GET_TX_CTRL_MAX		_IOR(FFL_TTY_MAGIC, 10, unsigned int)

/*
 * FFL_TTY_SET_RX_CTRL_MAX	- set the maximal size of the RX controller FIFO
 */
#define FFL_TTY_SET_RX_CTRL_MAX		_IOW(FFL_TTY_MAGIC, 11, unsigned int)

/*
 * FFL_TTY_GET_RX_CTRL_MAX	- get the maximal size of the RX controller FIFO
 */
#define FFL_TTY_GET_RX_CTRL_MAX		_IOR(FFL_TTY_MAGIC, 11, unsigned int)

/*
 * FFL_TTY_SET_TX_DELAY		- set the TX delay in us
 */
#define FFL_TTY_SET_TX_DELAY		_IOW(FFL_TTY_MAGIC, 12, unsigned int)

/*
 * FFL_TTY_GET_TX_DELAY		- get the TX delay in us
 */
#define FFL_TTY_GET_TX_DELAY		_IOR(FFL_TTY_MAGIC, 12, unsigned int)

/*
 * FFL_TTY_SET_RX_DELAY		- set the RX delay in us
 */
#define FFL_TTY_SET_RX_DELAY		_IOW(FFL_TTY_MAGIC, 13, unsigned int)

/*
 * FFL_TTY_GET_RX_DELAY		- get the RX delay in us
 */
#define FFL_TTY_GET_RX_DELAY		_IOR(FFL_TTY_MAGIC, 13, unsigned int)

/*
 * FFL_TTY_SET_TX_FLOW		- set the TX flow type (PIPE, SYNC)
 */
#define FFL_TTY_SET_TX_FLOW		_IOW(FFL_TTY_MAGIC, 16, unsigned int)

/*
 * FFL_TTY_GET_TX_FLOW		- get the TX flow type (PIPE, SYNC)
 */
#define FFL_TTY_GET_TX_FLOW		_IOR(FFL_TTY_MAGIC, 16, unsigned int)

/*
 * FFL_TTY_SET_RX_FLOW		- set the RX flow type (PIPE, SYNC)
 */
#define FFL_TTY_SET_RX_FLOW		_IOW(FFL_TTY_MAGIC, 17, unsigned int)

/*
 * FFL_TTY_GET_RX_FLOW		- get the RX flow type (PIPE, SYNC)
 */
#define FFL_TTY_GET_RX_FLOW		_IOR(FFL_TTY_MAGIC, 17, unsigned int)

/*
 * FFL_TTY_SET_TX_MODE		- set the TX mode type (FRAME, STREAM)
 */
#define FFL_TTY_SET_TX_MODE		_IOW(FFL_TTY_MAGIC, 18, unsigned int)

/*
 * FFL_TTY_GET_TX_MODE		- get the TX mode type (FRAME, STREAM)
 */
#define FFL_TTY_GET_TX_MODE		_IOR(FFL_TTY_MAGIC, 18, unsigned int)

/*
 * FFL_TTY_SET_RX_MODE		- set the RX mode type (FRAME, STREAM)
 */
#define FFL_TTY_SET_RX_MODE		_IOW(FFL_TTY_MAGIC, 19, unsigned int)

/*
 * FFL_TTY_GET_RX_MODE		- get the RX mode type (FRAME, STREAM)
 */
#define FFL_TTY_GET_RX_MODE		_IOR(FFL_TTY_MAGIC, 19, unsigned int)

/*
 * FFL_TTY_SET_TX_CHANNELS	- set the maximal number of TX channels
 */
#define FFL_TTY_SET_TX_CHANNELS		_IOW(FFL_TTY_MAGIC, 20, unsigned int)

/*
 * FFL_TTY_GET_TX_CHANNELS	- get the maximal number of TX channels
 */
#define FFL_TTY_GET_TX_CHANNELS		_IOR(FFL_TTY_MAGIC, 20, unsigned int)

/*
 * FFL_TTY_SET_RX_CHANNELS	- set the maximal number of RX channels
 */
#define FFL_TTY_SET_RX_CHANNELS		_IOW(FFL_TTY_MAGIC, 21, unsigned int)

/*
 * FFL_TTY_GET_RX_CHANNELS	- get the maximal number of RX channels
 */
#define FFL_TTY_GET_RX_CHANNELS		_IOR(FFL_TTY_MAGIC, 21, unsigned int)

/*
 * FFL_TTY_SET_TX_CHANNEL	- set the FFL TX channel
 */
#define FFL_TTY_SET_TX_CHANNEL		_IOW(FFL_TTY_MAGIC, 22, unsigned int)

/*
 * FFL_TTY_GET_TX_CHANNEL	- get the FFL TX channel
 */
#define FFL_TTY_GET_TX_CHANNEL		_IOR(FFL_TTY_MAGIC, 22, unsigned int)

/*
 * FFL_TTY_SET_RX_CHANNEL	- set the FFL RX channel
 */
#define FFL_TTY_SET_RX_CHANNEL		_IOW(FFL_TTY_MAGIC, 23, unsigned int)

/*
 * FFL_TTY_GET_RX_CHANNEL	- get the FFL RX channel
 */
#define FFL_TTY_GET_RX_CHANNEL		_IOR(FFL_TTY_MAGIC, 23, unsigned int)

/*
 * FFL_TTY_SET_TX_FRAME_LEN	- set the FFL TX frame length
 */
#define FFL_TTY_SET_TX_FRAME_LEN	_IOW(FFL_TTY_MAGIC, 24, unsigned int)

/*
 * FFL_TTY_GET_TX_FRAME_LEN	- get the FFL TX frame length
 */
#define FFL_TTY_GET_TX_FRAME_LEN	_IOR(FFL_TTY_MAGIC, 24, unsigned int)

/*
 * FFL_TTY_SET_RX_FRAME_LEN	- set the FFL RX frame length
 */
#define FFL_TTY_SET_RX_FRAME_LEN	_IOW(FFL_TTY_MAGIC, 25, unsigned int)

/*
 * FFL_TTY_GET_RX_FRAME_LEN	- get the FFL RX frame length
 */
#define FFL_TTY_GET_RX_FRAME_LEN	_IOR(FFL_TTY_MAGIC, 25, unsigned int)

/*
 * FFL_TTY_SET_TX_ARB_MODE	- set the FFL TX arbitration (RR ou priority)
 */
#define FFL_TTY_SET_TX_ARB_MODE		_IOW(FFL_TTY_MAGIC, 28, unsigned int)

/*
 * FFL_TTY_GET_TX_ARB_MODE	- get the FFL TX arbitration (RR or priority)
 */
#define FFL_TTY_GET_TX_ARB_MODE		_IOR(FFL_TTY_MAGIC, 28, unsigned int)

/*
 * FFL_TTY_SET_TX_FREQUENCY	- set the maximum FFL TX frequency (in kbit/s)
 */
#define FFL_TTY_SET_TX_FREQUENCY	_IOW(FFL_TTY_MAGIC, 30, unsigned int)

/*
 * FFL_TTY_GET_TX_FREQUENCY	- get the maximum FFL TX frequency (in kbit/s)
 */
#define FFL_TTY_GET_TX_FREQUENCY	_IOR(FFL_TTY_MAGIC, 30, unsigned int)

/*
 * FFL_TTY_RESET_TX_STATS	- reset the TX statistics
 */
#define FFL_TTY_RESET_TX_STATS		_IO(FFL_TTY_MAGIC, 32)

/*
 * FFL_TTY_GET_TX_STATS		- get the TX statistics
 */
#define FFL_TTY_GET_TX_STATS		_IOR(FFL_TTY_MAGIC, 32, \
					     struct hsi_ffl_stats)

/*
 * FFL_TTY_RESET_RX_STATS	- reset the RX statistics
 */
#define FFL_TTY_RESET_RX_STATS		_IO(FFL_TTY_MAGIC, 33)

/*
 * FFL_TTY_GET_RX_STATS		- get the RX statistics
 */
#define FFL_TTY_GET_RX_STATS		_IOR(FFL_TTY_MAGIC, 33, \
					     struct hsi_ffl_stats)

/*
 * FFL_TTY_SEND_BREAK		- send a BREAK frame to the modem
 */
#define FFL_TTY_SEND_BREAK		_IO(FFL_TTY_MAGIC, 34)

#endif /* _HSI_FFL_TTY_H */

