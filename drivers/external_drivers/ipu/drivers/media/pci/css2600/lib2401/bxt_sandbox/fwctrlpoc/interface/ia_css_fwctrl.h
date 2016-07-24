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

#ifndef __IA_CSS_FWCTRL_H__
#define __IA_CSS_FWCTRL_H__

#include "ia_css_isyspoc_comm.h"
#include "ia_css_psys_cmd_comm.h"
#include "type_support.h"
#include "sp.h"

/**
 * @brief  ia_css_fwctrl_psys_send_msg
 *
 * @return:
 *     int  : status of the API execution
 */
extern int ia_css_fwctrl_psys_send_msg(
	const ia_css_psysapi_cmd_t *psys_msg
);

/**
 * @brief  ia_css_fwctrl_psys_receive_msg
 *
 * @return:
 *     int  : status of the API execution
 */
extern int ia_css_fwctrl_psys_receive_msg(
	ia_css_psysapi_cmd_t *psys_msg
);

/**
 * @brief  ia_css_fwctrl_isys_send_msg
 *
 * @return:
 *     int  : status of the API execution
 */
extern int ia_css_fwctrl_isys_stream_send_msg(
	int stream_handle,
	const ia_css_isyspoc_cmd_msg_t *isys_msg
);

/*
 * @brief  ia_css_fwctrl_isys_receive_msg
 *
 * @return:
 *     int  : status of the API execution
 */
extern int ia_css_fwctrl_isys_receive_msg(
	ia_css_isyspoc_cmd_msg_t *isys_msg
);

#endif
