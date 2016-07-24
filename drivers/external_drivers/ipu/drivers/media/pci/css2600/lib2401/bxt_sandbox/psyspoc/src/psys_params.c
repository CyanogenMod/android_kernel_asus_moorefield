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

#include "gdc_device.h"		/* gdc_lut_store(), ... */
#include "sh_css_internal.h"
#include "ia_css_memory_access.h"
#include "assert_support.h"
#include "sh_css_param_dvs.h"
#include "ia_css_isp_param.h"
#include "ia_css_debug.h"

#include "memory_access.h"

#define IA_CSS_INCLUDE_PARAMETERS
#include "ia_css_isp_params.h"

#include "sh_css_param_shading.h"

#include "sh_css_frac.h" /* sDIGIT_FITTING */
#include "sh_css_params.h"
#include "ia_css_psyspoc_params.h"
#include "ia_css_psys_process_group.h"
#include "ia_css_psys_pgpoc_context.h"   /* ia_css_psys_pg_poc_context_t*/

/* Include all kernel host interfaces for ISP1 */
#include "dvs/dvs_1.0/ia_css_dvs.host.h"
#include "anr/anr_1.0/ia_css_anr.host.h"
#include "cnr/cnr_1.0/ia_css_cnr.host.h"
#include "csc/csc_1.0/ia_css_csc.host.h"
#include "de/de_1.0/ia_css_de.host.h"
#include "dp/dp_1.0/ia_css_dp.host.h"
#include "bnr/bnr_1.0/ia_css_bnr.host.h"
#include "fpn/fpn_1.0/ia_css_fpn.host.h"
#include "gc/gc_1.0/ia_css_gc.host.h"
#include "macc/macc_1.0/ia_css_macc.host.h"
#include "ctc/ctc_1.0/ia_css_ctc.host.h"
#include "ob/ob_1.0/ia_css_ob.host.h"
#include "raw/raw_1.0/ia_css_raw.host.h"
#include "s3a/s3a_1.0/ia_css_s3a.host.h"
#include "sc/sc_1.0/ia_css_sc.host.h"
#include "tnr/tnr_1.0/ia_css_tnr.host.h"
#include "uds/uds_1.0/ia_css_uds.host.h"
#include "wb/wb_1.0/ia_css_wb.host.h"
#include "ynr/ynr_1.0/ia_css_ynr.host.h"
#include "xnr/xnr_1.0/ia_css_xnr.host.h"

/* Include additional kernel host interfaces for ISP2 */
#include "aa/aa_2/ia_css_aa2.host.h"
#include "anr/anr_2/ia_css_anr2.host.h"
#include "bh/bh_2/ia_css_bh.host.h"
#include "cnr/cnr_2/ia_css_cnr2.host.h"
#include "de/de_2/ia_css_de2.host.h"
#include "gc/gc_2/ia_css_gc2.host.h"
#include "ctc/ctc_2/ia_css_ctc2.host.h"
#include "ynr/ynr_2/ia_css_ynr2.host.h"

/* include configs as set by client*/
#include "psys_params_sections.h"

#define SCTBL_BYTES(binary) \
	(sizeof(unsigned short) * (binary)->sctbl_height * \
	 (binary)->sctbl_aligned_width_per_color * IA_CSS_SC_NUM_COLORS)

static const struct ia_css_dz_config default_dz_config = {
	HRT_GDC_N,
	HRT_GDC_N
};

static const struct ia_css_vector default_motion_config = {
	0,
	0
};
/* Parameter encoding is not yet orthogonal.
   This function hnadles some of the exceptions.
*/
static void set_param_exceptions(struct ia_css_isp_parameters *params)
{
	assert (params != NULL);

	/* Copy also to DP. Should be done by the driver. */
	params->dp_config.gr = params->wb_config.gr;
	params->dp_config.r  = params->wb_config.r;
	params->dp_config.b  = params->wb_config.b;
	params->dp_config.gb = params->wb_config.gb;
}
static void init_default_param_configs(
	struct ia_css_isp_parameters *params,
	unsigned int isp_pipe_version)
{
	ia_css_set_nr_config(params, &default_nr_config);
	ia_css_set_s3a_config(params, &default_3a_config);
	ia_css_set_wb_config(params, &default_wb_config);
	ia_css_set_csc_config(params, &default_cc_config);
	ia_css_set_tnr_config(params, &default_tnr_config);
	ia_css_set_ob_config(params, &default_ob_config);
	ia_css_set_dp_config(params, &default_dp_config);
	ia_css_set_de_config(params, &default_de_config);
	ia_css_set_gc_config(params, &default_gc_config);
	ia_css_set_anr_config(params, &default_anr_config);
	ia_css_set_anr2_config(params, &default_anr_thres);
	ia_css_set_ce_config(params, &default_ce_config);
	ia_css_set_xnr_table_config(params, &default_xnr_table);
	ia_css_set_ecd_config(params, &default_ecd_config);
	ia_css_set_ynr_config(params, &default_ynr_config);
	ia_css_set_fc_config(params, &default_fc_config);
	ia_css_set_cnr_config(params, &default_cnr_config);
	ia_css_set_macc_config(params, &default_macc_config);
	ia_css_set_ctc_config(params, &default_ctc_config);
	ia_css_set_aa_config(params, &default_aa_config);
	ia_css_set_r_gamma_config(params, &default_r_gamma_table);
	ia_css_set_g_gamma_config(params, &default_g_gamma_table);
	ia_css_set_b_gamma_config(params, &default_b_gamma_table);
	ia_css_set_yuv2rgb_config(params, &default_yuv2rgb_cc_config);
	ia_css_set_rgb2yuv_config(params, &default_rgb2yuv_cc_config);
	ia_css_set_xnr_config(params, &default_xnr_config);

	set_param_exceptions(params);

	/* generated code does not handle these params */
	params->motion_config = default_motion_config;
	params->ee_config = default_ee_config;
	params->yee_config.ee = default_ee_config;
	if(isp_pipe_version == 1)
		params->macc_table = default_macc_table;
	else
		params->macc_table = default_macc2_table;
	params->gc_table = default_gamma_table;
	params->raw_config = default_baa_config;
	params->dz_config = default_dz_config;
	params->yee_config.nr = default_nr_config;
}

void ia_css_psyspoc_handle_params(ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context)
{
	unsigned int param_id, mem;
	struct ia_css_pipeline_stage stage;
	struct ia_css_binary *binary;
	struct ia_css_isp_parameters *isp_params;
	struct sh_css_ddr_address_map *mem_map;
	ia_css_psysapi_cmd_t *cmd;
	unsigned raw_bit_depth;
	int retval = -1;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_handle_params(): enter\n");

	assert(process_group != NULL && context != NULL);

	cmd = &context->host_cmd;
	raw_bit_depth = context->allocated_in_info.raw_bit_depth;
	memset(&stage, 0, sizeof(struct ia_css_pipeline_stage));

	binary = &context->binary;
	isp_params = &context->isp_params;

	memset(isp_params, 0, sizeof(struct ia_css_isp_parameters));
	mem_map = &cmd->isp_param_info.mem_map;
	stage.binary = binary;

	/* TODO: Get configs from cached paramters or some other buffer attached to
	 * terminals. */
	init_default_param_configs(isp_params, cmd->sp_stage.isp_pipe_version);

	retval = psyspoc_update_cached_param_configs_from_sections(isp_params,
			process_group,
			context->pg_manifest);
	assert(retval == 0);
	if (retval != 0) {
		/*alternatively we can choose to continue with default parameters.*/
		return;
	}


	ia_css_isp_param_enable_pipeline(&stage.binary->mem_params);
	if (isp_params->config_changed[IA_CSS_OB_ID]) {
		ia_css_ob_configure(&isp_params->stream_configs.ob,
			    cmd->sp_stage.isp_pipe_version, raw_bit_depth);
	}
	if (isp_params->config_changed[IA_CSS_S3A_ID]) {
		ia_css_s3a_configure(raw_bit_depth);
	}

	for (param_id = 0; param_id < IA_CSS_NUM_PARAMETER_IDS; param_id++) {
		/* Handle shading correction elsewhere. */
		if (param_id == IA_CSS_SC_ID)
			continue;

		ia_css_kernel_process_param[param_id](0, &stage, isp_params);
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_handle_params(): commit param config to iunit address\n");
	for (mem = 0; mem < N_IA_CSS_MEMORIES; mem++) {
		const struct ia_css_isp_data *isp_data =
			ia_css_isp_param_get_isp_mem_init(
				&binary->info->sp.mem_initializers,
				IA_CSS_PARAM_CLASS_PARAM, mem);
		size_t size = isp_data->size;

		if(!isp_params->isp_mem_params_changed[0][0][mem])
			continue;

		/* we do not support changing params on process group (yet) */
		if(size > 0 && mem_map->isp_mem_param[0][mem] == mmgr_NULL) {
			const struct ia_css_host_data *params;
			hrt_vaddress ddr_mem_ptr;

			ddr_mem_ptr = mmgr_malloc(size);
			assert(ddr_mem_ptr != 0);
			mem_map->isp_mem_param[0][mem] = ddr_mem_ptr;

			params = ia_css_isp_param_get_mem_init(
				&binary->mem_params,
				IA_CSS_PARAM_CLASS_PARAM, mem);
			assert(params != NULL);
			mmgr_store(ddr_mem_ptr, params->address, size);
		}
	}

	/* dvs_6axis table is reused until process group is destroyed. */
	if (binary->info->sp.enable.dvs_6axis && mem_map->dvs_6axis_params_y == mmgr_NULL) {
		/* because UV is packed into the Y plane, calc total
		 * YYU size = /2 gives size of UV-only,
		 * total YYU size = UV-only * 3.
		 */
		mem_map->dvs_6axis_params_y = mmgr_malloc((size_t)((DVS_6AXIS_BYTES(binary) / 2) * 3));
		if (isp_params->dvs_6axis_config == NULL) /* Generate default DVS unity table on start up*/
			{
				struct ia_css_resolution dvs_offset;
				dvs_offset.width  = (PIX_SHIFT_FILTER_RUN_IN_X + binary->dvs_envelope.width) / 2;
				dvs_offset.height = (PIX_SHIFT_FILTER_RUN_IN_Y + binary->dvs_envelope.height) / 2;

				isp_params->dvs_6axis_config = generate_dvs_6axis_table(&binary->out_frame_info[0].res,
										    &dvs_offset);
			}

			store_dvs_6axis_config(isp_params,
						binary,
						mem_map->dvs_6axis_params_y);
	}

	/* sc_tbl is reused until process group is destroyed. */
	if (binary->info->sp.enable.sc && mem_map->sc_tbl == mmgr_NULL) {
		/* shading table is full resolution, reduce */
		prepare_shading_table(
			(const struct ia_css_shading_table *)isp_params->sc_table,
			isp_params->sensor_binning,
			&isp_params->sc_config,
			binary);
		assert(isp_params->sc_config != NULL);

		mem_map->sc_tbl = mmgr_malloc((size_t)(SCTBL_BYTES(binary)));
		assert(mem_map->sc_tbl != mmgr_NULL);

		ia_css_params_store_sctbl(&stage, mem_map->sc_tbl, isp_params->sc_config);
		ia_css_kernel_process_param[IA_CSS_SC_ID](0, &stage, isp_params);

		ia_css_shading_table_free(isp_params->sc_config);
		isp_params->sc_config = NULL;
	}

	/* macc_tbl is reused until process group is destroyed. */
	if (binary->info->sp.enable.macc && mem_map->macc_tbl == mmgr_NULL) {
		struct ia_css_macc_table converted_macc_table;
		unsigned int i, j, idx;
		unsigned int idx_map[] = {
			0, 1, 3, 2, 6, 7, 5, 4, 12, 13, 15, 14, 10, 11, 9, 8};

		for (i = 0; i < IA_CSS_MACC_NUM_AXES; i++) {
			idx = 4*idx_map[i];
			j   = 4*i;
			if (cmd->sp_stage.isp_pipe_version == 1) {
				converted_macc_table.data[idx] =
				  sDIGIT_FITTING(isp_params->macc_table.data[j],
				  13, SH_CSS_MACC_COEF_SHIFT);
				converted_macc_table.data[idx+1] =
				  sDIGIT_FITTING(isp_params->macc_table.data[j+1],
				  13, SH_CSS_MACC_COEF_SHIFT);
				converted_macc_table.data[idx+2] =
				  sDIGIT_FITTING(isp_params->macc_table.data[j+2],
				  13, SH_CSS_MACC_COEF_SHIFT);
				converted_macc_table.data[idx+3] =
				  sDIGIT_FITTING(isp_params->macc_table.data[j+3],
				  13, SH_CSS_MACC_COEF_SHIFT);
			} else {
				converted_macc_table.data[idx] =
					isp_params->macc_table.data[j];
				converted_macc_table.data[idx+1] =
					isp_params->macc_table.data[j+1];
				converted_macc_table.data[idx+2] =
					isp_params->macc_table.data[j+2];
				converted_macc_table.data[idx+3] =
					isp_params->macc_table.data[j+3];
			}
		}
		mem_map->macc_tbl = mmgr_malloc(sizeof(struct ia_css_macc_table));
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"mem_map->macc_tbl = 0x%x", mem_map->macc_tbl);
		assert(mem_map->macc_tbl != 0);
		mmgr_store(mem_map->macc_tbl,
			converted_macc_table.data,
			sizeof(converted_macc_table.data));
	}

	/* TODO: Handle other tables. */

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_handle_params(): exit\n");
}
