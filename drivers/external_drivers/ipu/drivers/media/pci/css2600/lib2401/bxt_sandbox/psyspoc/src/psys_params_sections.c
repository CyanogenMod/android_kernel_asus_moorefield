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

#include "ia_css_memory_access.h"
#include "assert_support.h"

#include "memory_access.h"
#define IA_CSS_INCLUDE_PARAMETERS
#include "ia_css_isp_params.h"

#include "sh_css_params.h"

#include "ia_css_psys_process_group.h"
#include "ia_css_psys_terminal.h"
#include "ia_css_psys_program_group_manifest.h"
#include "ia_css_psys_terminal_manifest.h"
#include "ia_css_psys_manifest_types.h"  /* ia_css_parameter_manifest_t*/

/* include configs as set by client*/
#include "psys_params_sections.h"
#include "ia_camera_utils_cached_param_configs.h"  /* ia_css_3a_config_ext_t */
#include "s3a/s3a_1.0/ia_css_s3a.host.h"
#include "s3a/s3a_1.0/ia_css_s3a_types.h"

static int ia_css_set_s3a_config_ext(
	struct ia_css_isp_parameters *isp_params,
	const ia_css_3a_config_ext_t *ext_config);

static int convert_3a_ext_to_isp_params(
		struct ia_css_3a_config *config,
		const ia_css_3a_config_ext_t *ext_config);

static ia_css_param_terminal_manifest_t *psyspoc_get_param_terminal_manifest(
		const ia_css_program_group_manifest_t *pg_manifest);

static hrt_vaddress get_cached_param_data(
	const ia_css_process_group_t *process_group);

int psyspoc_update_cached_param_configs_from_sections(
	struct ia_css_isp_parameters *isp_params,
	const ia_css_process_group_t *process_group,
	const ia_css_program_group_manifest_t *pg_manifest)
{
	int i, retval = -1;
	int section_count = 0;
	ia_css_param_terminal_manifest_t *pmanifest = NULL;
	ia_css_parameter_manifest_t *curr_param = NULL;
	hrt_vaddress terminal_data = 0;
	bool is_available = false;

	assert(isp_params != NULL);
	assert(process_group !=  NULL);
	assert(pg_manifest !=  NULL);

	if (isp_params == NULL || process_group == NULL || pg_manifest == NULL) {
		return retval;
	}

	pmanifest =  psyspoc_get_param_terminal_manifest(pg_manifest);
	if (pmanifest == NULL) {
		return retval;
	}

	/* get section count from cached terminal manifest*/
	section_count = ia_css_param_terminal_manifest_get_section_count(pmanifest);


	/* get terminal's hrt adress from process_group*/
	terminal_data = get_cached_param_data(process_group);

	if (terminal_data ==  VIED_NULL) {
		return retval;
	}
	/* for each section,
	 * a) based on kernel ID,
	 * 	load parameter configurations into isp_params
	 * */

	for (i = 0 ; i < section_count; i++) {
		curr_param =  ia_css_param_terminal_manifest_get_parameter_manifest(
				pmanifest,
				i);
		switch (curr_param->kernel_id) {
		case(IA_CAMERA_PSYS_PARAMS_KERNEL_ID_S3A):
		{
			ia_css_3a_config_ext_t ext_config;

			COMPILATION_ERROR_IF( SIZE_OF_IA_CSS_3A_CONFIG_EXT_IN_BITS
					!= (CHAR_BIT * sizeof(ia_css_3a_config_ext_t)));

			/* get 3a param from section offset */
			mmgr_load((terminal_data + curr_param->mem_offset),
					&ext_config,
					curr_param->mem_size);

			/* store these params into isp params */
			retval = ia_css_set_s3a_config_ext(isp_params,
					(const ia_css_3a_config_ext_t *)&ext_config);

			/*atleast 1 section is available for configuration*/
			is_available = true;
			break;
		}
		default:
			break;
		}
	}

	if (is_available == false) {
		/* for this binary, none of configurations are coming from driver.*/
		retval = 0; /* it is fine.*/
	}
	return retval;
}

static int ia_css_set_s3a_config_ext(
	struct ia_css_isp_parameters *isp_params,
	const ia_css_3a_config_ext_t *ext_config)
{
	struct ia_css_3a_config config;
	int retval = -1;

	assert(isp_params != NULL);
	assert(ext_config !=  NULL);
	retval = convert_3a_ext_to_isp_params(&config, ext_config);
	ia_css_set_s3a_config(isp_params, &config);
	return retval;
}

static int convert_3a_ext_to_isp_params(
		struct ia_css_3a_config *config,
		const ia_css_3a_config_ext_t *ext_config)
{
	int i, retval = -1;

	assert(config != NULL);
	assert(ext_config !=  NULL);
	if (config == NULL || ext_config == NULL) {
		return retval;
	}
	config->ae_y_coef_r = (ia_css_u0_16)ext_config->ae_y_coef_r;
	config->ae_y_coef_g = (ia_css_u0_16)ext_config->ae_y_coef_g;
	config->ae_y_coef_b = (ia_css_u0_16)ext_config->ae_y_coef_b;
	config->awb_lg_high_raw = (ia_css_u0_16)ext_config->awb_lg_high_raw;
	config->awb_lg_low = (ia_css_u0_16)ext_config->awb_lg_low;
	config->awb_lg_high = (ia_css_u0_16)ext_config->awb_lg_high;

	for (i = 0 ; i < 7; i++) {
		config->af_fir1_coef[i] = (ia_css_s0_15)ext_config->af_fir1_coef[i];
		config->af_fir2_coef[i] = (ia_css_s0_15)ext_config->af_fir2_coef[i];
	}
	retval = 0;
	return retval;
}

hrt_vaddress get_cached_param_data(
	const ia_css_process_group_t *process_group)
{
	uint32_t i, terminal_count;
	ia_css_terminal_t *terminal;
	assert(process_group != NULL);

	terminal_count = ia_css_process_group_get_terminal_count(process_group);
	for (i = 0; i < terminal_count; i++) {
		terminal = ia_css_process_group_get_terminal(process_group, i);
		assert(terminal != NULL);
		if (terminal == NULL) {
			return VIED_NULL;
		}
		if (ia_css_is_terminal_parameter_terminal(terminal))
			return ((hrt_vaddress)ia_css_terminal_get_buffer(terminal));

	}
	return VIED_NULL;
}

static ia_css_param_terminal_manifest_t *psyspoc_get_param_terminal_manifest(
		const ia_css_program_group_manifest_t *pg_manifest)
{
	int terminal_count, i;
	ia_css_param_terminal_manifest_t *pmanifest;

	assert(pg_manifest !=  NULL);
	if (pg_manifest == NULL) {
		return NULL;
	}
	/*get cached_param_terminal manifest from program group manifest */
	terminal_count = ia_css_program_group_manifest_get_terminal_count(pg_manifest);

	for (i = 0; i < terminal_count; i++) {
		ia_css_terminal_manifest_t *manifest =
			 ia_css_program_group_manifest_get_terminal_manifest(pg_manifest, i);
		assert(manifest !=  NULL);
		if (manifest == NULL) {
			return NULL;
		}
		if (true == ia_css_is_terminal_manifest_parameter_terminal(manifest)) {
			pmanifest = (ia_css_param_terminal_manifest_t*)manifest;
			return pmanifest;
		}
	}

	return NULL;
}

int psyspoc_get_pg_param_from_sections(
	const ia_css_process_group_t *process_group,
	const ia_css_program_group_manifest_t *pg_manifest,
	ia_css_pg_param_t *param)
{
	int retval = -1, i;
	hrt_vaddress cached_param_data;
	ia_css_param_terminal_manifest_t *pmanifest;
	ia_css_parameter_manifest_t *curr_param = NULL;
	int section_count = 0;

	assert((process_group != NULL) && (param != NULL) && (pg_manifest != NULL));
	if (process_group == NULL || param == NULL || pg_manifest == NULL) {
		return retval;
	}

	/* param terminal manifest contains section information */
	pmanifest =  psyspoc_get_param_terminal_manifest(pg_manifest);

	if (pmanifest == NULL) {
		return retval;
	}

	/* get section count from cached terminal manifest*/
	section_count = ia_css_param_terminal_manifest_get_section_count(pmanifest);

	/* get terminal's hrt adress from process_group*/
	cached_param_data = get_cached_param_data((const ia_css_process_group_t *)process_group);

	assert(cached_param_data != VIED_NULL);
	if (cached_param_data == VIED_NULL) {
		return retval;
	}

	for (i = 0 ; i < section_count; i++) {
		curr_param =  ia_css_param_terminal_manifest_get_parameter_manifest(
				pmanifest,
				i);
		if (curr_param->kernel_id == IA_CAMERA_PSYS_PARAMS_KERNEL_ID_POC) {
			mmgr_load(cached_param_data + curr_param->mem_offset,
				  param,
				 curr_param->mem_size);
			retval = 0;
		}
	}


	return retval;
}
