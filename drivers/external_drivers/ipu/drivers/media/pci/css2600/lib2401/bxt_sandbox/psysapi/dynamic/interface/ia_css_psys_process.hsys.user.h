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

#ifndef __IA_CSS_PSYS_PROCESS_HSYS_USER_H_INCLUDED__
#define __IA_CSS_PSYS_PROCESS_HSYS_USER_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_process.hsys.user.h
 *
 * Define the methods on the process object: Hsys user interface
 */

#include <ia_css_program_group_param.h>		/* ia_css_program_param_t */

#include <ia_css_psys_process_types.h>
#include <ia_css_psys_manifest_types.h>

#include <type_support.h>					/* uint8_t */

/*
 * Creation
 */

/*! Compute the size of storage required for allocating the process object

 @param	manifest[in]			program manifest
 @param	param[in]				program parameters

 @return 0 on error
 */
extern size_t ia_css_sizeof_process(
	const ia_css_program_manifest_t			*manifest,
	const ia_css_program_param_t			*param);

/*! Create (the storage for) the process object

 @param	manifest[in]			program manifest
 @param	param[in]				program parameters

 @return NULL on error
 */
extern ia_css_process_t *ia_css_process_create(
	const ia_css_program_manifest_t			*manifest,
	const ia_css_program_param_t			*param);

/*! Destroy (the storage of) the process object

 @param	process[in]				process object

 @return NULL
 */
extern ia_css_process_t *ia_css_process_destroy(
	ia_css_process_t						*process);

/*
 * Access functions
 */

/*! Print the process object to file/stream

 @param	process[in]				process object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_process_print(
	const ia_css_process_t					*process,
	void									*fid);

#endif /* __IA_CSS_PSYS_PROCESS_HSYS_USER_H_INCLUDED__ */
