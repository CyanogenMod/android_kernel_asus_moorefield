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

#ifndef _IA_CSS_PSYS_PGPOC_CONTEXT_H
#define _IA_CSS_PSYS_PGPOC_CONTEXT_H

#include "ia_css_psys_process_group.h" /* ia_css_process_group_t*/
#include "ia_css_psys_program_group_manifest.h" /* ia_css_program_group_manifest_t
					ia_css_program_group_param_t*/
#include "ia_css_psys_cmd_comm.h" /*ia_css_psysapi_cmd_t*/
#include "ia_css_binary.h" /* struct ia_css_binary*/
#include "sh_css_params.h" /* struct ia_css_isp_parameters*/
#include "sh_css_defs.h"

typedef struct ia_css_psys_pgpoc_context ia_css_psys_pgpoc_context_t;

struct ia_css_psys_pgpoc_context {
	ia_css_process_group_t *process_group;
	const ia_css_program_group_manifest_t *pg_manifest;
	const ia_css_program_group_param_t *param;
	ia_css_psysapi_cmd_t host_cmd;
	struct ia_css_binary binary;
	struct ia_css_isp_3a_statistics *s3a_stats;
	struct ia_css_isp_parameters isp_params;
	struct ia_css_frame_info allocated_in_info;
	struct ia_css_frame_info out_info;
	struct ia_css_frame_info out_vf_info;
	struct ia_css_frame *delay_frames[NUM_VIDEO_DELAY_FRAMES];   /* reference input frame */
	struct ia_css_frame *tnr_frames[NUM_VIDEO_TNR_FRAMES];   /* tnr frames */
	struct ia_css_isp_dvs_statistics *dvs_stats;
	bool enable_vf_output;
};
#endif /*_IA_CSS_PSYS_PGPOC_CONTEXT_H */
