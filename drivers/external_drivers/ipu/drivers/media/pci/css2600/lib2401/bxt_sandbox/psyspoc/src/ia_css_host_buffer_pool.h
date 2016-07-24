/* Release Version: irci_master_20140715_0307 */
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

#ifndef __IA_CSS_HOST_BUFFER_POOL_H__
#define __IA_CSS_HOST_BUFFER_POOL_H__

#include "system_types.h"

struct ia_css_host_buffer_pool{

	/* Total number of buffers in the pool */
	unsigned int num_buffers;

	void *pool_head;

	/* Function to allocate elements in the pool. "size" size of each element in
	 * the pool. returns ENOMEM on error. Each element is stores as a token and
	 * void pointer union pair. */
	int (*init)(struct ia_css_host_buffer_pool *this_bufpool, size_t size,
		unsigned int num_bufs);

	/* free all elements in the pool */
	void (*uninit)(struct ia_css_host_buffer_pool *this_bufpool);

	/* acquire a host address. Note that this can be 32 or 64 bit. */
	void *(*acquire_host_buf)(struct ia_css_host_buffer_pool *this_bufpool);

	/* release a host address. */
	void (*release_host_buf)(struct ia_css_host_buffer_pool *this_bufpool,
		void *buffer);
	/* release a host token. */
	void (*release_host_token)(struct ia_css_host_buffer_pool *this_bufpool,
		uint64_t token);

	/* map from token to host buffer */
	void *(*host_buf_map)(struct ia_css_host_buffer_pool *this_bufpool,
		uint64_t token);
	/* map from host buffer to token */
	uint64_t (*token_buf_map)(struct ia_css_host_buffer_pool *this_bufpool,
		void *host_buf);
};


 /**
 * @brief ia_css_create_bufpool() - Create an instance of buffer pool
 * @param buffer pool handle: number of buffer in buffers in the pool
 */
extern void ia_css_create_host_bufpool(
	struct ia_css_host_buffer_pool *bufpool
);

#endif /*__IA_CSS_HOST_BUFFER_POOL_H__*/
