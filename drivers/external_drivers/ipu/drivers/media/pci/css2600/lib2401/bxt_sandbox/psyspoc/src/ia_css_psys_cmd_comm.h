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

#ifndef __IA_CSS_PSYS_CMD_COMM_H__
#define __IA_CSS_PSYS_CMD_COMM_H__

#include "sh_css_internal.h"
#include "uds/uds_1.0/ia_css_uds_param.h"

#define IA_CSS_PSYSPOC_CMD_FINISHED	(1)

typedef struct ia_css_psysapi_cmd ia_css_psysapi_cmd_t;

struct ia_css_psysapi_cmd {
	/* TODO: Should all elements that SP access from DDR be 256 bit aligned? */
	struct sh_css_isp_stage isp_stage;
	struct sh_css_sp_stage sp_stage;
	struct sh_css_sp_pipeline sp_pipeline;
	/* TODO: uds_params should be moved to pg specific param modules? */
	CSS_ALIGN(struct sh_css_sp_uds_params uds_params[SH_CSS_MAX_STAGES], 32);
	CSS_ALIGN(struct ia_css_isp_parameter_set_info isp_param_info, 32);
	/*Return event from SP*/
	uint32_t psys_event;
	uint64_t token;
	uint32_t s3a_stats_buffer_hi;
	uint32_t s3a_stats_buffer_lo;
	uint32_t s3a_hist_buffer;
	uint32_t current_stripe;
	uint32_t stripe_limit;
	struct ia_css_isp_dvs_statistics dvs_stats;
};

#endif  /* __IA_CSS_PSYS_CMD_COMM_H__ */
