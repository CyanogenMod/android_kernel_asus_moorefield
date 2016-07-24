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

#ifndef __IA_CSS_PSYS_TERMINAL_MANIFEST_SIM_H_INCLUDED__
#define __IA_CSS_PSYS_TERMINAL_MANIFEST_SIM_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_terminal_manifest.sim.h
 *
 * Define the methods on the terminal manifest object: Simulation only
 */

#include <ia_css_psys_manifest_types.h>

#include <type_support.h>					/* size_t */

/*! Compute the size of storage required for allocating the terminal manifest object

 @param	terminal_type[in]		type of the terminal manifest {parameter, data}
 @param  dimension[in]	                number of sections in cached param terminal.
					currently used only for cached param terminal
 @return 0 on error
*/
extern size_t ia_css_sizeof_terminal_manifest(
	const ia_css_terminal_type_t			terminal_type,
	const	uint16_t 				dimension);

/*! Create (the storage for) the terminal manifest object

 @param	terminal_type[in]		type of the terminal manifest {parameter, data}

 @return NULL on error
 */
extern ia_css_terminal_manifest_t *ia_css_terminal_manifest_alloc(
	const ia_css_terminal_type_t			terminal_type);

/*! Destroy (the storage of) the terminal manifest object

 @param	manifest[in]			terminal manifest

 @return NULL
 */
extern ia_css_terminal_manifest_t *ia_css_terminal_manifest_free(
	ia_css_terminal_manifest_t				*manifest);

#endif /* __IA_CSS_PSYS_TERMINAL_MANIFEST_SIM_H_INCLUDED__ */
