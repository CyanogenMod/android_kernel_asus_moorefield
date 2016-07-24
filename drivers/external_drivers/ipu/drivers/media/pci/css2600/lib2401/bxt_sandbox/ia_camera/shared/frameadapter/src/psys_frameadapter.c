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

#include "ia_css_psys_frameadapter.h"
#include "assert_support.h"
#include "platform_support.h"

void ia_css_psys_helper_framedesc_to_frameinfo(
	struct ia_css_frame_info *info,
	const ia_css_frame_descriptor_t *desc)
{
	assert(info != NULL && desc != NULL);
	memset(info, 0, sizeof(struct ia_css_frame_info));
	info->res.width = desc->dimension[IA_CSS_COL_DIMENSION];
	info->res.height = desc->dimension[IA_CSS_ROW_DIMENSION];
	info->padded_width = desc->stride[IA_CSS_COL_DIMENSION];
	/* convert from psys data formats to css formats. */
	info->format = ia_css_psys_helper_css_frame_format(
		desc->frame_format_type);
	info->raw_bit_depth = desc->bpp;
	/* default: grbg */
	info->raw_bayer_order = 0;
}

enum ia_css_frame_format ia_css_psys_helper_css_frame_format(
	enum ia_css_frame_format_type psys_frame_type)
{
	enum ia_css_frame_format css_frame_format;

	switch (psys_frame_type) {
	case IA_CSS_DATA_FORMAT_NV11:
		css_frame_format = IA_CSS_FRAME_FORMAT_NV11;
		break;

	case IA_CSS_DATA_FORMAT_NV12:
		css_frame_format = IA_CSS_FRAME_FORMAT_NV12;
		break;

	case IA_CSS_DATA_FORMAT_NV16:
		css_frame_format = IA_CSS_FRAME_FORMAT_NV16;
		break;

	case IA_CSS_DATA_FORMAT_NV21:
		css_frame_format = IA_CSS_FRAME_FORMAT_NV21;
		break;

	case IA_CSS_DATA_FORMAT_NV61:
		css_frame_format = IA_CSS_FRAME_FORMAT_NV61;
		break;

	case IA_CSS_DATA_FORMAT_YV12:
		css_frame_format = IA_CSS_FRAME_FORMAT_YV12;
		break;

	case IA_CSS_DATA_FORMAT_YV16:
		css_frame_format = IA_CSS_FRAME_FORMAT_YV16;
		break;

	case IA_CSS_DATA_FORMAT_YUV420:
		css_frame_format = IA_CSS_FRAME_FORMAT_YUV420;
		break;

	case IA_CSS_DATA_FORMAT_YUV420_LINE:
		css_frame_format = IA_CSS_FRAME_FORMAT_YUV_LINE;
		break;

	case IA_CSS_DATA_FORMAT_YUV422:
		css_frame_format = IA_CSS_FRAME_FORMAT_YUV422;
		break;

	case IA_CSS_DATA_FORMAT_YUV444:
		css_frame_format = IA_CSS_FRAME_FORMAT_YUV444;
		break;

	case IA_CSS_DATA_FORMAT_UYVY:
		css_frame_format = IA_CSS_FRAME_FORMAT_UYVY;
		break;

	case IA_CSS_DATA_FORMAT_YUYV:
		css_frame_format = IA_CSS_FRAME_FORMAT_YUYV;
		break;

	case IA_CSS_DATA_FORMAT_RGB565:
		css_frame_format = IA_CSS_FRAME_FORMAT_RGB565;
		break;

	case IA_CSS_DATA_FORMAT_RGBA888:
		css_frame_format = IA_CSS_FRAME_FORMAT_RGBA888;
		break;

	case IA_CSS_DATA_FORMAT_RGB888:
		css_frame_format = IA_CSS_FRAME_FORMAT_PLANAR_RGB888;
		break;

	case IA_CSS_DATA_FORMAT_RAW:
		css_frame_format = IA_CSS_FRAME_FORMAT_RAW;
		break;

	case IA_CSS_DATA_FORMAT_RAW_PACKED:
		css_frame_format = IA_CSS_FRAME_FORMAT_RAW_PACKED;
		break;

	case IA_CSS_DATA_FORMAT_BINARY_8:
		css_frame_format = IA_CSS_FRAME_FORMAT_BINARY_8;
		break;

	default:
		/* We don't allow conversion for formats like 3A. */
		assert(0);
		break;
	}

	return css_frame_format;
}
