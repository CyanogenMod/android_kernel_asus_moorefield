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

#ifndef __ISYSPOC_2401_H__
#define __ISYSPOC_2401_H__

/* The following is needed for the function arguments */
#include "ia_css_isysapi_types.h"
#include "ia_css_isysapi_fw_types.h"
#include "ia_css_isys.h"
#include <input_system.h>

extern bool ia_css_isys_translate_stream_cfg_to_isys_stream_descr(
		const struct ia_css_isys_stream_cfg_data *stream_cfg,
		ia_css_isys_descr_t	*isys_stream_descr,
		unsigned int ip_num
);

extern bool isys_convert_mipi_dt_to_mipi_format(
	enum ia_css_isys_mipi_data_type dt,
	mipi_predictor_t compression,
	unsigned int *fmt_type
);
#endif
