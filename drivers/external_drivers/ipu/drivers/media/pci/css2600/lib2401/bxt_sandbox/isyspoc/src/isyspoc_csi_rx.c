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

#include <type_support.h>
#include "ia_css_isys_ext_public.h"
#include "input_system.h"
#include "ia_css_isys.h"
#include "assert_support.h"
/* The FW bridged types are included through the following */
#include "ia_css_isysapi.h"
#include "ia_css_isys_private.h"
#include "isyspoc_2401.h"
#include "ia_css_debug.h"

int ia_css_isysapi_rx_set_csi_port_cfg(
	HANDLE context,
	enum ia_css_isys_mipi_data_type dt, /* from this we can get format type*/
	enum ia_css_isys_mipi_vc channel_id,
	enum ia_css_isys_stream_source src,
	int32_t num_lanes)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	bool rc;
	unsigned int fmt_type = 0;

	if (ctx == NULL)
		return EINVAL;

	assert(src<N_IA_CSS_ISYS_STREAM_SRC);
	memset(&ctx->port_cfg[src].csi_port_attr, 0,
		sizeof(ctx->port_cfg[src].csi_port_attr));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_isysapi_rx_set_csi_port_cfg(): Enter\n");
	rc = isys_convert_mipi_dt_to_mipi_format(dt,
				MIPI_PREDICTOR_NONE,
				&fmt_type);

	ctx->port_cfg[src].csi_port_attr.active_lanes = num_lanes;
	ctx->port_cfg[src].csi_port_attr.fmt_type = fmt_type;
	ctx->port_cfg[src].csi_port_attr.ch_id = (int32_t)channel_id;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_isysapi_rx_set_csi_port_cfg(): Exit\n");

	return 0;
}

int ia_css_isysapi_rx_set_tpg_cfg(
	HANDLE context,
	enum ia_css_isys_stream_source src,
	const struct pixelgen_tpg_cfg_s *cfg)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;

	if (ctx == NULL)
		return EINVAL;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_isysapi_rx_set_tpg_cfg(): Enter\n");
	memcpy(&(ctx->port_cfg[src].tpg_port_attr),
		cfg,
		sizeof(struct pixelgen_tpg_cfg_s));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_isysapi_rx_set_tpg_cfg(): Exit\n");
	return 0;
}

int ia_css_isysapi_rx_set_prbs_cfg(
	HANDLE context,
	enum ia_css_isys_stream_source src,
	const struct pixelgen_prbs_cfg_s *cfg)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;

	if (ctx == NULL)
		return EINVAL;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_isysapi_rx_set_prbs_cfg(): Enter\n");
	memcpy(&(ctx->port_cfg[src].prbs_port_attr),
		cfg,
		sizeof(struct pixelgen_prbs_cfg_s));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_isysapi_rx_set_prbs_cfg(): Exit\n");
	return 0;
}
