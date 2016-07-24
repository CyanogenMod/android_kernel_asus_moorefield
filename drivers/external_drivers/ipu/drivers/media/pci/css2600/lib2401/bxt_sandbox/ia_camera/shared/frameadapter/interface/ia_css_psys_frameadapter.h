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

#ifndef __IA_CSS_PSYS_FRAME_ADAPTER_H__
#define __IA_CSS_PSYS_FRAME_ADAPTER_H__

#include "ia_css_program_group_data.h"
#include "ia_css_frame_public.h"

/* Convert from psys frame descriptor to css frame info */
void ia_css_psys_helper_framedesc_to_frameinfo(
	struct ia_css_frame_info *info,
	const ia_css_frame_descriptor_t *desc);

/* Get css frame format from psys frame format */
enum ia_css_frame_format ia_css_psys_helper_css_frame_format(
	enum ia_css_frame_format_type psys_frame_type);

#endif /* __IA_CSS_PSYS_FRAME_ADAPTER_H__ */
