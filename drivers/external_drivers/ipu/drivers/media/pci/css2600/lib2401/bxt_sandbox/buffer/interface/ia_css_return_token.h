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

#ifndef __IA_CSS_RETURN_TOKEN__
#define __IA_CSS_RETURN_TOKEN__

#include "platform_support.h"	/*inline*/

// ia_css_return_token: data item of exacly 8 bytes (64 bits)
// which can be used to pass a return token back to the host

typedef unsigned long long ia_css_return_token;

static inline void
ia_css_return_token_copy(ia_css_return_token* to, const ia_css_return_token* from)
{
	// copy a return token on VIED processor
	int* dst = (int*)to;
	int* src = (int*)from;
	dst[0] = src[0];
	dst[1] = src[1];
}

static inline void
ia_css_return_token_zero(ia_css_return_token* to)
{
	// zero return token on VIED processor
	int* dst = (int*)to;
	dst[0] = 0;
	dst[1] = 0;
}

/////////

// compile time assert
#define CT_ASSERT(cnd) ((void)sizeof(char[(cnd)?1:-1]))

static inline void _check_return_token_size(void)
{
  CT_ASSERT(sizeof(int)==4);
  CT_ASSERT(sizeof(ia_css_return_token)==8);
}

#endif
