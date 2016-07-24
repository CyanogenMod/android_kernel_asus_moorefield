/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __IA_CSS_SYSCOM_CONFIG_H__
#define __IA_CSS_SYSCOM_CONFIG_H__

/**
  * Parameter struct for ia_css_syscom_initialize
  */
struct ia_css_syscom_config
{
	/* address of firmware in DDR/IMR */
	unsigned long long host_firmware_address;

	unsigned int ssid;
	unsigned int mmid;
	unsigned int num_input_queues;
	unsigned int num_output_queues;
	unsigned int input_queue_size; /* max # tokens per queue */
	unsigned int output_queue_size; /* max # tokens per queue */
	unsigned int input_token_size; /* in bytes */
	unsigned int output_token_size; /* in bytes */

	/* firmware-specific configuration data */
	void* specific_addr;
	unsigned int specific_size;
};

#endif /*__IA_CSS_SYSCOM_CONFIG_H__*/

