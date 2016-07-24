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

#ifndef __PSYS_PARAMS_SECTIONS__
#define  __PSYS_PARAMS_SECTIONS__
#include "ia_css_isp_param.h"

#define IA_CSS_INCLUDE_PARAMETERS
#include "ia_css_isp_params.h"

#include "sh_css_params.h"
#include "ia_css_psyspoc_params.h"
#include "ia_css_psys_process_group.h"
#include "ia_css_psys_program_group_manifest.h"
#include "ia_css_pg_param_internal.h"

extern int psyspoc_update_cached_param_configs_from_sections(
	struct ia_css_isp_parameters *isp_params,
	const ia_css_process_group_t *process_group,
	const ia_css_program_group_manifest_t *pg_manifest);

extern int psyspoc_get_pg_param_from_sections(
	const ia_css_process_group_t *process_group,
	const ia_css_program_group_manifest_t *pg_manifest,
	ia_css_pg_param_t *param);
#endif  /*__PSYS_PARAMS_SECTIONS__ */
