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

#ifndef __IA_CSS_ISYS_PRIVATE_H__
#define __IA_CSS_ISYS_PRIVATE_H__


#include "type_support.h"
#include "input_system.h"
/* Needed for the definitions of STREAM_ID_MAX */
#include "ia_css_isysapi.h"
#include "ia_css_pocapp_comm.h" /*BXTPOC_COMM_QUEUE_SIZE*/


/* Set any of the 2 for the respective error handling */
#define VERIFY_DEVSTATE 1
#define VERIFY_STRSTATE 1

#if (VERIFY_DEVSTATE != 0)
/**
 * enum device_state
 */
enum device_state {
	IA_CSS_ISYS_DEVICE_STATE_IDLE = 0,
	IA_CSS_ISYS_DEVICE_STATE_CONFIGURED
};
#endif /* VERIFY_DEVSTATE */

#define ISYSPOC_PENDING_FRAME_SIZE	(BXTPOC_COMM_QUEUE_SIZE)
/**
 * enum stream_state
 */
enum stream_state {
	IA_CSS_ISYS_STREAM_STATE_IDLE = 0,
	IA_CSS_ISYS_STREAM_STATE_OPENED,
	IA_CSS_ISYS_STREAM_STATE_STARTED,
	IA_CSS_ISYS_STREAM_STATE_CLOSED
};


struct ia_css_isys_pending_frames {
	struct ia_css_isys_frame_buff_set frame_set[ISYSPOC_PENDING_FRAME_SIZE];
	uint32_t read_index;
	uint32_t write_index;
};

struct ia_css_isys_context {
	unsigned int stream_nof_output_pins[STREAM_ID_MAX];
#if (VERIFY_DEVSTATE != 0)
	enum device_state dev_state;
#endif /* VERIFY_DEVSTATE */
	enum stream_state stream_state_array[STREAM_ID_MAX];
	virtual_input_system_stream_t virtual_input_system[STREAM_ID_MAX];
	input_system_cfg_t port_cfg[N_IA_CSS_ISYS_STREAM_SRC];
	struct ia_css_isys_pending_frames pending_frames[STREAM_ID_MAX];
};

int isyspoc_init_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle);

int isyspoc_add_to_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle,
	const struct ia_css_isys_frame_buff_set *next_frame,
	uint32_t *buf_index);

int isyspoc_get_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle,
	uint32_t buf_index,
	struct ia_css_isys_frame_buff_set *next_frame);

int isyspoc_remove_from_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle,
	uint32_t buf_index);


#endif /*__IA_CSS_ISYS_PRIVATE_H_INCLUDED__*/
