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

#include "ia_css_isys_ext_public.h"
#include "ia_css_isys_private.h"
#include "type_support.h"
#include "assert_support.h"
#include "math_support.h"
#include "error_support.h"
#include "platform_support.h"

int isyspoc_init_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle)
{
	struct ia_css_isys_pending_frames *pending_frame_set;

	assert(stream_handle < STREAM_ID_MAX);
	assert(ctx);

	pending_frame_set = &ctx->pending_frames[stream_handle];

	/* Reset the pendign frames */
	memset(pending_frame_set, 0, sizeof(struct ia_css_isys_pending_frames));
	return 0;
}

int isyspoc_add_to_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle,
	const struct ia_css_isys_frame_buff_set *next_frame,
	uint32_t *buf_index)
{
	struct ia_css_isys_pending_frames *pending_frame_set;

	assert(stream_handle < STREAM_ID_MAX);
	assert(ctx);
	assert(next_frame);
	assert(buf_index);

	pending_frame_set = &ctx->pending_frames[stream_handle];

	if (OP_std_modadd(pending_frame_set->write_index, 1, ISYSPOC_PENDING_FRAME_SIZE)
		== pending_frame_set->write_index) {
		/* No buffer space left. cannot add */
		return ENOBUFS;
	}

	/* We have space*/
	pending_frame_set->write_index = OP_std_modadd(pending_frame_set->write_index,
		1, ISYSPOC_PENDING_FRAME_SIZE);

	/*Update the frame set*/
	pending_frame_set->frame_set[pending_frame_set->write_index] = *next_frame;

	*buf_index = pending_frame_set->write_index;
	return 0;
}

int isyspoc_get_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle,
	uint32_t buf_index,
	struct ia_css_isys_frame_buff_set *next_frame)
{
	struct ia_css_isys_pending_frames *pending_frame_set;

	assert(ctx);
	assert(stream_handle < STREAM_ID_MAX);
	assert(next_frame);
	assert(buf_index < ISYSPOC_PENDING_FRAME_SIZE);

	pending_frame_set = &ctx->pending_frames[stream_handle];

	assert(buf_index == OP_std_modadd(pending_frame_set->read_index,
		1, ISYSPOC_PENDING_FRAME_SIZE));

	*next_frame = pending_frame_set->frame_set[buf_index];
	return 0;
}


int isyspoc_remove_from_pending_frame(
	struct ia_css_isys_context *ctx,
	int stream_handle,
	uint32_t buf_index)
{
	struct ia_css_isys_pending_frames *pending_frame_set;

	assert(ctx);
	assert(stream_handle < STREAM_ID_MAX);
	assert(buf_index < ISYSPOC_PENDING_FRAME_SIZE);

	pending_frame_set = &ctx->pending_frames[stream_handle];

	assert(buf_index == OP_std_modadd(pending_frame_set->read_index,
		1, ISYSPOC_PENDING_FRAME_SIZE));

	/* We have space*/
	pending_frame_set->read_index = OP_std_modadd(pending_frame_set->read_index,
		1, ISYSPOC_PENDING_FRAME_SIZE);

	return 0;
}
