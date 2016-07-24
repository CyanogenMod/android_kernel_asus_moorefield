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

#ifndef _IA_CSS_CIRCBUF_COMM_H
#define _IA_CSS_CIRCBUF_COMM_H

#include <type_support.h>  /* uint8_t, uint32_t */

/****************************************************************
 *
 * Portable Data structures
 *
 ****************************************************************/
/**
 * @brief Data structure for the circular descriptor.
 */
typedef struct ia_css_circbuf_desc_s ia_css_circbuf_desc_t;
struct ia_css_circbuf_desc_s {
	uint8_t size;	/* the maximum number of elements*/
	uint8_t step;   /* number of bytes per element */
	uint8_t start;	/* index of the oldest element */
	uint8_t end;	/* index at which to write the new element */
};
#define SIZE_OF_IA_CSS_CIRCBUF_DESC_S_STRUCT				\
	(4 * sizeof(uint8_t))

/**
 * @brief Data structure for the circular buffer element.
 */
typedef struct ia_css_circbuf_elem_s ia_css_circbuf_elem_t;
struct ia_css_circbuf_elem_s {
	uint32_t val;	/* the value stored in the element */
};
#define SIZE_OF_IA_CSS_CIRCBUF_ELEM_S_STRUCT				\
	(sizeof(uint32_t))

#endif /*_IA_CSS_CIRCBUF_COMM_H*/
