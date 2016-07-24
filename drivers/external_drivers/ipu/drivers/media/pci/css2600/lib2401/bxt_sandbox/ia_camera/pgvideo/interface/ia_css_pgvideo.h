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

#ifndef __IA_CSS_PGVIDEO_H__
#define __IA_CSS_PGVIDEO_H__

#include "ia_css_pg_param_internal.h" /* ia_css_pg_param_t*/
#include "ia_css_psys_program_group_manifest.h"
#include "ia_css_program_group_param.h"

size_t
ia_camera_sizeof_video_cont_nobds_isp2_manifest(void);

ia_css_program_group_manifest_t *
ia_camera_video_cont_nobds_isp2_pg_manifest_get(void *blob);

extern size_t
ia_camera_sizeof_video_cont_nobds_isp2_pg_param(void);

extern ia_css_program_group_param_t *
ia_camera_video_cont_nobds_isp2_pg_param_get(void *blob);

extern ia_css_pg_param_t *
ia_camera_video_cont_nobds_isp2_pg_cached_param_get(
	void *blob,
	ia_css_frame_descriptor_t *in_desc,
	ia_css_frame_descriptor_t *out_desc,
	ia_css_frame_descriptor_t *vf_out_desc);

extern size_t
ia_camera_sizeof_video_cont_nobds_isp2_pg_cached_param(void);

#endif /* __IA_CSS_PGVIDEO_H__ */
