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

#ifndef __IA_CSS_PSYS_PROCESS_GROUP_PSYS_H_INCLUDED__
#define __IA_CSS_PSYS_PROCESS_GROUP_PSYS_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_process_group.psys.h
 *
 * Define the methods on the process group object: Psys embedded interface
 */

#include <ia_css_psys_process_types.h>

/*
 * Dispatcher
 */

/*! Perform the run command on the process group

 @param	process_group[in]		process group object

 Note: Run indicates that the process group will execute

 Precondition: The process group must be started or
 suspended and the processes have acquired the necessary
 internal resources

 @return < 0 on error
 */
extern int ia_css_process_group_run(
	ia_css_process_group_t					*process_group);

/*! Perform the stop command on the process group

 @param	process_group[in]		process group object

 Note: Stop indicates that the process group has completed execution

 Postcondition: The external resoruces can now be detached

 @return < 0 on error
 */
extern int ia_css_process_group_stop(
	ia_css_process_group_t					*process_group);


#endif /* __IA_CSS_PSYS_PROCESS_GROUP_PSYS_H_INCLUDED__ */
