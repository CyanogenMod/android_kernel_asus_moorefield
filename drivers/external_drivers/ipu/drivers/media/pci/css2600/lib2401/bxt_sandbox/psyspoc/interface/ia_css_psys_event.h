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

#ifndef __IA_CSS_PSYS_EVENT_H__
#define __IA_CSS_PSYS_EVENT_H__

#include "type_support.h"

typedef enum ia_css_psys_event_type ia_css_psys_event_type_t;

enum ia_css_psys_event_type {
	IA_CSS_PSYS_EVENT_TYPE_EXCEPTION = -1,
	/* or buffer full */
	/* If dispatcher is on host, this event can be used to look for other
	 * process groups that can be started. */
	IA_CSS_PSYS_EVENT_TYPE_FRAME_FULL = 0,
	IA_CSS_PSYS_EVENT_TYPE_CMD_COMPLETE,
};

#endif /* __IA_CSS_PSYS_EVENT_H__ */
