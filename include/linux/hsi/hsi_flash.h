/*
 * hsi_flash.h
 *
 * Part of the HSI flash device driver.
 *
 * Copyright (C) 2010 Nokia Corporation. All rights reserved.
 *
 * Contact: Andras Domokos <andras.domokos at nokia.com>
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


#ifndef __HSI_FLASH_H
#define __HSI_FLASH_H

#define HSI_FLASH_MAGIC		0/*'k'*/
#define HSC_IOW(num, dtype)	_IOW(HSI_FLASH_MAGIC, num, dtype)
#define HSC_IOR(num, dtype)	_IOR(HSI_FLASH_MAGIC, num, dtype)
#define HSC_IOWR(num, dtype)	_IOWR(HSI_FLASH_MAGIC, num, dtype)
#define HSC_IO(num)		_IO(HSI_FLASH_MAGIC, num)

#define HIOCRESET		HSC_IO(16)
#define HIOCSNDBRK		    HSC_IO(18)
#define HIOCSPEED		HSC_IO(19)
#define HIOCGWAKE		HSC_IO(20)
#define HIOCGBRK		HSC_IO(21)
#define HIOCTXCTRL		HSC_IO(22)

#define HSC_PM_DISABLE		0
#define HSC_PM_ENABLE		1

#define HSC_MODE_STREAM		1
#define HSC_MODE_FRAME		2
#define HSC_FLOW_SYNC		0
#define HSC_ARB_RR		0
#define HSC_ARB_PRIO		1

struct hsc_rx_config {
	uint32_t mode;
	uint32_t flow;
	uint32_t channels;
};

struct hsc_tx_config {
	uint32_t mode;
	uint32_t flow;
	uint32_t channels;
	uint32_t speed;
	uint32_t arb_mode;
};

#endif /* __HSI_FLASH_H */
