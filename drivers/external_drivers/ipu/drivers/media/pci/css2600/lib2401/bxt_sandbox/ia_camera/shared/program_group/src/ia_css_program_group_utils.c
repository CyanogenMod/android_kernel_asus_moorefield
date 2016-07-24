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

#include "assert_support.h"
#include "ia_css_pg_param_internal.h" /*ia_css_pg_param_t*/
#include "sh_css_defs.h"   /* SH_CSS_BDS_FACTOR_1_00*/
#include "ia_css_frame_public.h"  /*IA_CSS_FRAME_DELAY_1*/
#include "gdc_device.h"          /*HRT_GDC_N*/
#include "isp/modes/interface/isp_const.h"
#include "ia_css_program_group_data.h" /*ia_css_frame_format_type*/
#if !defined(HAS_NO_HMEM)
#ifndef __INLINE_HMEM__
#define __INLINE_HMEM__
#endif
#include "hmem.h"
#endif /* !defined(HAS_NO_HMEM) */

#include "ia_css_psys_manifest_types.h" /* ia_css_terminal_manifest_t */
#include "ia_css_psys_program_group_manifest.h"
#include "ia_css_psys_terminal_manifest.h"

static int32_t
grid_deci_factor_log2(int width, int height)
{
	int fact, fact1;
	fact = 5;
	while (ISP_BQ_GRID_WIDTH(width, fact - 1) <= SH_CSS_MAX_BQ_GRID_WIDTH &&
	       ISP_BQ_GRID_HEIGHT(height, fact - 1) <= SH_CSS_MAX_BQ_GRID_HEIGHT
	       && fact > 3)
		fact--;

	/* fact1 satisfies the specification of grid size. fact and fact1 is
	   not the same for some resolution (fact=4 and fact1=5 for 5mp). */
	if (width >= 2560)
		fact1 = 5;
	else if (width >= 1280)
		fact1 = 4;
	else
		fact1 = 3;
	return max(fact, fact1);
}

uint32_t ia_css_psyspoc_util_calc_deci_log_factor(
	const ia_css_frame_descriptor_t *desc)
{
	/* TODO: Handle bds out info and fixed_s3a_deci_log when available. */
	unsigned int sc_3a_dis_width = desc->dimension[IA_CSS_COL_DIMENSION];
	unsigned int sc_3a_dis_padded_width = desc->stride[IA_CSS_COL_DIMENSION];
	unsigned int sc_3a_dis_height = desc->dimension[IA_CSS_ROW_DIMENSION];
	unsigned int s3a_isp_width;
	/*Silent the compiler*/
	(void)sc_3a_dis_width;
	s3a_isp_width = _ISP_S3A_ELEMS_ISP_WIDTH(sc_3a_dis_padded_width, 0);
	return grid_deci_factor_log2(s3a_isp_width, sc_3a_dis_height);
}

extern size_t ia_css_psyspoc_util_get_data_size(
		ia_css_pg_param_t *param,
		const ia_css_frame_descriptor_t *input_frame_desc,
		ia_css_frame_format_type_t type)
{
	int  s3atbl_isp_height;
	unsigned int s3a_log_deci = 0;
	unsigned int sc_3a_dis_height = 0,
		     sc_3a_dis_padded_width = 0,
		     s3a_isp_width = 0;
	uint32_t vmem_size, hmem_size;
	uint16_t width, height;
	if (type == IA_CSS_DATA_S3A_HISTOGRAM) {
#if !defined(HAS_NO_HMEM)
		hmem_size =  sizeof_hmem(HMEM0_ID);
		hmem_size = CEIL_MUL(hmem_size, HIVE_ISP_DDR_WORD_BYTES);
		return hmem_size;
#else
		return 0;
#endif
	} else if ((type == IA_CSS_DATA_S3A_STATISTICS_HI) ||
		   (type == IA_CSS_DATA_S3A_STATISTICS_LO)) {

	width = input_frame_desc->dimension[IA_CSS_COL_DIMENSION];
	height = input_frame_desc->dimension[IA_CSS_ROW_DIMENSION];
	sc_3a_dis_padded_width = CEIL_MUL(width + param->left_cropping , 2*ISP_VEC_NELEMS);
	sc_3a_dis_height = height + param->top_cropping;

	s3a_isp_width = _ISP_S3A_ELEMS_ISP_WIDTH(sc_3a_dis_padded_width,
		param->left_cropping);

	if (param->fixed_s3a_deci_log) {
		s3a_log_deci = param->fixed_s3a_deci_log;
	} else {
		s3a_log_deci = grid_deci_factor_log2(s3a_isp_width,
							    sc_3a_dis_height);
	}
	s3atbl_isp_height =
		_ISP_S3ATBL_ISP_HEIGHT(sc_3a_dis_height,
			s3a_log_deci);
	vmem_size = ISP_S3ATBL_HI_LO_STRIDE_BYTES * s3atbl_isp_height;
	vmem_size  = CEIL_MUL(vmem_size, HIVE_ISP_DDR_WORD_BYTES);
	return vmem_size;
	} else {
		return 0;
	}
}

int32_t ia_css_psyspoc_utils_get_terminal_buffer_size(
		ia_css_terminal_manifest_t *terminal_manifest)
{
	size_t size = 0;

	assert(terminal_manifest != NULL);
	if( terminal_manifest == NULL) {
		return -1;
	}

	if (IA_CSS_TERMINAL_TYPE_PARAM_CACHED ==
			ia_css_terminal_manifest_get_type(terminal_manifest)) {
		ia_css_param_terminal_manifest_t *param_terminal_manifest =
			(ia_css_param_terminal_manifest_t *)terminal_manifest;

		int16_t section_count = ia_css_param_terminal_manifest_get_section_count(
						param_terminal_manifest);

		ia_css_parameter_manifest_t *last_param =
			ia_css_param_terminal_manifest_get_parameter_manifest(
					param_terminal_manifest,
					section_count - 1);

		assert(last_param != NULL);
		if(last_param == NULL) {
			return -1;
		}
		size = last_param->mem_offset + last_param->mem_size;
	} else {
		/*TODO: get table size for  data terminals ??.
		 * Note: Input and output data terminal's size values are non-static*/
		assert(false);
	}

	return size;
}
