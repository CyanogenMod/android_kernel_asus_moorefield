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

#ifndef _IA_CSS_ISYS_EXT_PUBLIC_H
#define _IA_CSS_ISYS_EXT_PUBLIC_H

#include <type_support.h>
#include "pixelgen_global.h"   /* struct pixelgen_tpg_cfg_t
				pixelgen_prbs_cfg_t
				*/
#include "ia_css_isysapi_fw_types.h"  /* HANDLE */

typedef void* HANDLE;

/**
*@brief: ia_css_isysapi_rx_set_csi_port_cfg
*  setup csi port configuration information for given stream_handle.
* @param context : isys context stores rx config for all stream_handles.
* @param dt : Input mipi data type.
* @param channel_id : virtual channel.
* @param src : which port is this configuration related to
* @param num_lanes : number of active lanes for given port.
* @return  error code (errno.h)
*/
int ia_css_isysapi_rx_set_csi_port_cfg(
	HANDLE context,
	enum ia_css_isys_mipi_data_type dt, /* from this we can get format type*/
	enum ia_css_isys_mipi_vc channel_id,
	enum ia_css_isys_stream_source src,
	int32_t num_lanes);

/**
*@brief: ia_css_isysapi_rx_set_tpg_cfg
*  setup tpg configuration information for given stream_handle.
* @param context : isys context stores rx config for all stream_handles.
* @param pixelgen_tpg_cfg_s : tpg configuration input.
* @return  error code (errno.h)
*/
extern int ia_css_isysapi_rx_set_tpg_cfg(
	HANDLE context,
	enum ia_css_isys_stream_source src,
	const struct pixelgen_tpg_cfg_s *cfg
);

/**
*@brief: ia_css_isysapi_rx_set_prbs_cfg
*  setup prbs configuration information for given stream_handle.
* @param context : isys context stores rx config for all stream_handles.
* @param pixelgen_prbs_cfg_s : prbs configuration input.
* @return  error code (errno.h)
*/
extern int ia_css_isysapi_rx_set_prbs_cfg(
	HANDLE context,
	enum ia_css_isys_stream_source src,
	const struct pixelgen_prbs_cfg_s *cfg
);

#endif /* _IA_CSS_ISYS_EXT_PUBLIC_H */

