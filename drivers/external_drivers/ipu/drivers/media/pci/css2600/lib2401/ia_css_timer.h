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

#ifndef __IA_CSS_TIMER_H__
#define __IA_CSS_TIMER_H__

/** @file
 * Timer interface definitions
 */

#include <type_support.h>		/* for uint32_t */

/* timer reading definition */
typedef uint32_t clock_value_t;

/* code measurement common struct */
struct ia_css_time_meas {
	clock_value_t	start_timer_value, end_timer_value;	/* measured time in ticks */
};

#endif  /* __IA_CSS_TIMER_H__ */
