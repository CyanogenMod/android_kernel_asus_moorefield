/*
 * hsi_dlp_board.h
 *
 * Header for the HSI dlp client driver.
 *
 * Copyright (C) 2010, 2011 Intel Corporation. All rights reserved.
 *
 * Contact: Frederic BERAT <fredericx.berat@intel.com>
 *          Faouaz TENOUTIT <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#ifndef __HSI_CLIENT_BOARD_H__
#define __HSI_CLIENT_BOARD_H__

#include <asm/intel-mid.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/module.h>


/* Supported Modem IDs*/
enum {
	MODEM_UNSUP,
	MODEM_6260,
	MODEM_6268,
	MODEM_6360,
	MODEM_7160,
	MODEM_7260
};

struct hsi_client_base_info {
	/* modem infos */
	int		mdm_ver;
};

struct sfi_to_mdm {
	char hsi_name[SFI_NAME_LEN + 1];
	int modem_type;
};

struct hsi_platform_data {
	struct hsi_mid_platform_data hsi_mid_info;
	struct hsi_client_base_info hsi_client_info;
};

/* Retrieve modem parameters on ACPI framework */
int hsi_retrieve_modem_platform_data(struct device *dev);


#endif				/* __HSI_DLP_BOARD_H__ */
