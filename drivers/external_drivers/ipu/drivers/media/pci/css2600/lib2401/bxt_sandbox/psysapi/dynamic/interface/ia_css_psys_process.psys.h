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

#ifndef __IA_CSS_PSYS_PROCESS_PSYS_H_INCLUDED__
#define __IA_CSS_PSYS_PROCESS_PSYS_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_process.psys.h
 *
 * Define the methods on the process object: Psys embedded interface
 */

#include <ia_css_psys_process_types.h>

/*
 * Process manager
 */

/*! Acquire the resources specificed in process object

 @param	process[in]				process object

 Postcondition: This is a try process if any of the
 resources is not available, all succesfully acquired
 ones will be release and the function will return an
 error

 @return < 0 on error
 */
extern int ia_css_process_acquire(
	ia_css_process_t						*process);

/*! Release the resources specificed in process object

 @param	process[in]				process object

 @return < 0 on error
 */
extern int ia_css_process_release(
	ia_css_process_t						*process);


#endif /* __IA_CSS_PSYS_PROCESS_PSYS_H_INCLUDED__ */
