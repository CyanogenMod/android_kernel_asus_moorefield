/******************************************************************************
 * Intel Management Engine Interface (Intel MEI) Linux driver
 * Intel MEI Interface Header
 *
 * This file is provided under a dual BSD/GPLv2 license.  When using or
 * redistributing this file, you may do so under either license.
 *
 * GPL LICENSE SUMMARY
 *
 * Copyright(c) 2003 - 2012 Intel Corporation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of version 2 of the GNU General Public License as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA 02110,
 * USA
 *
 * The full GNU General Public License is included in this distribution
 * in the file called LICENSE.GPL.
 *
 * Contact Information:
 *	Intel Corporation.
 *	linux-mei@linux.intel.com
 *	http://www.intel.com
 *
 * BSD LICENSE
 *
 * Copyright(c) 2003 - 2012 Intel Corporation. All rights reserved.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 *  * Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  * Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  * Neither the name Intel Corporation nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 * A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 * OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 * SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 * DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 * THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 *****************************************************************************/

#ifndef _MEI_NFC_H
#define _MEI_NFC_H

#include <linux/types.h>

struct mei_nfc_cmd {
	uint8_t command;
	uint8_t status;
	uint16_t req_id;
	uint32_t reserved;
	uint16_t data_size;
	uint8_t sub_command;
	uint8_t data[];
} __packed;

struct mei_nfc_reply {
	uint8_t command;
	uint8_t status;
	uint16_t req_id;
	uint32_t reserved;
	uint16_t data_size;
	uint8_t sub_command;
	uint8_t reply_status;
	uint8_t data[];
} __packed;

struct mei_nfc_if_version {
	uint8_t radio_version_sw[3];
	uint8_t reserved[3];
	uint8_t radio_version_hw[3];
	uint8_t i2c_addr;
	uint8_t fw_ivn;
	uint8_t vendor_id;
	uint8_t radio_type;
} __packed;

struct mei_nfc_connect {
	uint8_t fw_ivn;
	uint8_t vendor_id;
} __packed;

struct mei_nfc_connect_resp {
	uint8_t fw_ivn;
	uint8_t vendor_id;
	uint16_t me_major;
	uint16_t me_minor;
	uint16_t me_hotfix;
	uint16_t me_build;
} __packed;

struct mei_nfc_hci_hdr {
	u8 cmd;
	u8 status;
	u16 req_id;
	u32 reserved;
	u16 data_size;
} __packed;

#define MEI_NFC_CMD_MAINTENANCE 0x00
#define MEI_NFC_CMD_HCI_SEND 0x01
#define MEI_NFC_CMD_HCI_RECV 0x02

#define MEI_NFC_SUBCMD_CONNECT    0x00
#define MEI_NFC_SUBCMD_IF_VERSION 0x01

#define MEI_NFC_HEADER_SIZE 10
#define MEI_NFC_MAX_HCI_PAYLOAD 300

/* Vendors */
#define MEI_NFC_VENDOR_INSIDE 0x00

/* Radio types */
#define MEI_NFC_VENDOR_INSIDE_UREAD 0x00

#endif /* _MEI_NFC_H */
