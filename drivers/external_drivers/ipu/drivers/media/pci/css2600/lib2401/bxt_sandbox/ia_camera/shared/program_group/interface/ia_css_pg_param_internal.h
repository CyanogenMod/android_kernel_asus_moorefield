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

#ifndef __IA_CSS_PG_PARAM_INTERNAL_H__
#define __IA_CSS_PG_PARAM_INTERNAL_H__

#include "ia_css_program_group_data.h"
#include "sh_css_uds.h"
#include "ia_css_buffer.h"
#include "ia_css_psys_frameadapter.h" /*struct ia_css_supported_frame_pins*/
#include "ia_css_psys_manifest_types.h" /* ia_css_terminal_manifest_t */

#include "type_support.h"

#define SIZE_OF_IA_CSS_PG_PARAM_STRUCT_IN_BITS\
	(IA_CSS_DIMENSION_IN_BITS * IA_CSS_N_DATA_DIMENSION\
	+ (12 * IA_CSS_UINT8_T_BITS) \
	+ (2 * IA_CSS_UINT32_T_BITS) \
	+ SH_CSS_UDS_INFO_IN_BITS \
	+ SH_CSS_CROP_POS_IN_BITS)

typedef struct ia_css_pg_param ia_css_pg_param_t;

struct ia_css_pg_param {
	ia_css_dimension_t dvs_envelope[IA_CSS_N_DATA_DIMENSION];
	uint32_t required_bds_factor;
	uint32_t dvs_frame_delay;
	uint8_t xnr;
	uint8_t enable_vf_output;
	uint8_t isp_vf_downscale_bits;
	uint8_t isp_deci_log_factor;
	uint8_t isp_pipe_version;
	uint8_t top_cropping;
	uint8_t left_cropping;
	/* Add support for zoom config. */
	struct sh_css_uds_info uds;
	struct sh_css_crop_pos sp_out_crop_pos;
	/* Preview binary specific Parameters*/
	uint8_t copy_output;
	uint8_t fixed_s3a_deci_log;
};

extern uint32_t ia_css_psyspoc_util_calc_deci_log_factor(
	const ia_css_frame_descriptor_t *desc);

/* Helper to find 3A size. Support for other datatypes to be added. */
extern size_t ia_css_psyspoc_util_get_data_size(
		ia_css_pg_param_t *param,
		const ia_css_frame_descriptor_t *input_frame_desc,
		ia_css_frame_format_type_t type);

extern int32_t ia_css_psyspoc_utils_get_terminal_buffer_size(
		ia_css_terminal_manifest_t *terminal_manifest);

#endif /* __IA_CSS_PG_PARAM_INTERNAL_H__ */
