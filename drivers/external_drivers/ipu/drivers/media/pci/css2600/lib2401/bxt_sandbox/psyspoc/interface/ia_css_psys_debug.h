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

#ifndef _IA_CSS_PSYS_DEBUG_H_
#define _IA_CSS_PSYS_DEBUG_H_

#include "ia_css_psys_cmd_comm.h"
#include "ia_css_isp_param_types.h"
#include "sh_css_internal.h"
#include "ia_css_frame_comm.h"
#include "assert_support.h"
#include "ia_css_debug.h"
#include "sh_css_defs.h"
#include "sh_css_uds.h"

/*! \brief Dump all related command info data
 * \param 	cmd pointer to psysapi command
 * \return	None
 */
extern void ia_css_debug_psys_cmd_print(
	const ia_css_psysapi_cmd_t *cmd);

#endif /* _IA_CSS_PSYS_DEBUG_H_ */

