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

#ifndef __IA_CSS_BUFFER_POOL_H__
#define __IA_CSS_BUFFER_POOL_H__

#include "system_types.h"

struct ia_css_buffer_pool{

	/* Total number of buffers in the pool */
	unsigned int num_buffers;

	void *pool_head;

	int (*init)(struct ia_css_buffer_pool *this_bufpool, size_t size,
		unsigned int num_bufs);

	void (*uninit)(struct ia_css_buffer_pool *this_bufpool);

	hrt_vaddress (*acquire_buf)(struct ia_css_buffer_pool *this_bufpool);

	void (*release_buf)(struct ia_css_buffer_pool *this_bufpool, hrt_vaddress buffer);
};


 /**
 * @brief ia_css_create_bufpool() - Create an instance of buffer pool
 * @param buffer pool handle: number of buffer in buffers in the pool
 */
extern void ia_css_create_bufpool(
	struct ia_css_buffer_pool *bufpool
);

#endif /*__IA_CSS_BUFFER_POOL_H__*/
