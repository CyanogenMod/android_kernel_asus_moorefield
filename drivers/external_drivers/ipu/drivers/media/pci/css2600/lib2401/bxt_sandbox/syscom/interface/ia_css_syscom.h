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

#ifndef __IA_CSS_SYSCOM_H__
#define __IA_CSS_SYSCOM_H__


/*
 * The CSS Subsystem Communication Interface - Host side
 *
 * It provides subsystem initialzation, send ports and receive ports
 * The PSYS and ISYS interfaces are implemented on top of this interface.
 */

#include "ia_css_syscom_config.h"

struct ia_css_syscom_context;

/**
 * ia_css_syscom_open() - initialize a subsystem context
 * @config: pointer to the configuration data (read)
 *
 * Purpose:
 * - initialize host side data structures
 * - boot the subsystem?
 *
 * Return: struct ia_css_syscom_context* on success, 0 otherwise.
 */
extern struct ia_css_syscom_context*
ia_css_syscom_open(
	struct ia_css_syscom_config  *config
);

extern int
ia_css_syscom_close(
	struct ia_css_syscom_context *context
);

/**
 * Open a port for sending tokens to the subsystem
 * @context: pointer to the subsystem context
 * @port: send port index
 */
extern int
ia_css_syscom_send_port_open(
	struct ia_css_syscom_context *context,
	unsigned int port
);

extern int
ia_css_syscom_send_port_close(
	struct ia_css_syscom_context *context,
	unsigned int port
);

/**
 * Get the number of tokens that can be sent to a port without error.
 * @context: pointer to the subsystem context
 * @port: send port index
 * @num_tokens (output): number of tokens that can be sent
 */
extern int
ia_css_syscom_send_port_available(
	struct ia_css_syscom_context *context,
        unsigned int port,
	unsigned int* num_tokens
);

/**
 * Send a token to the subsystem port
 * Returns an error when there is no space for the token
 * The token size is determined during initialization
 * @context: pointer to the subsystem context
 * @port: send port index
 * @token: pointer to the token value that is transferred to the subsystem
 */
extern int
ia_css_syscom_send_port_transfer(
	struct ia_css_syscom_context *context,
        unsigned int port,
	const void* token
);

/**
 * Open a port for receiving tokens to the subsystem
 * @context: pointer to the subsystem context
 * @port: receive port index
 */
extern int
ia_css_syscom_recv_port_open(
	struct ia_css_syscom_context* context,
	unsigned int port
);

extern int
ia_css_syscom_recv_port_close(
	struct ia_css_syscom_context* context,
	unsigned int port
);

/**
 * Get the number of tokens that can be received from a port without errors.
 * @context: pointer to the subsystem context
 * @port: receive port index
 * @num_tokens (output): number of tokens that can be received
 */
extern int
ia_css_syscom_recv_port_available(
	struct ia_css_syscom_context* context,
        unsigned int port,
	unsigned int* num_tokens
);

/*
 * Receive a token
 * returns an error when the response queue is empty
 */
/**
 * Receive a token from the subsystem port
 * Returns an error when there are no tokens available
 * The token size is determined during initialization
 * @context: pointer to the subsystem context
 * @port: receive port index
 * @token (output): pointer to (space for) the token to be received
 */
extern int
ia_css_syscom_recv_port_transfer(
	struct ia_css_syscom_context* context,
        unsigned int port,
	void* token
);

#endif /* __IA_CSS_SYSCOM_H__*/
