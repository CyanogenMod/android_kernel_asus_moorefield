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

#ifndef __IA_CSS_PSYS_PROCESS_H_INCLUDED__
#define __IA_CSS_PSYS_PROCESS_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_process.h
 *
 * Define the methods on the process object that are not part of a single interface
 */

#include <ia_css_psys_process_types.h>

#include <vied_nci_psys_system_global.h>

#include <type_support.h>					/* uint8_t */

/*
 * Creation
 */
#include <ia_css_psys_process.hsys.user.h>

/*
 * Internal resources
 */
#include <ia_css_psys_process.hsys.kernel.h>

/*
 * Process manager
 */
#include <ia_css_psys_process.psys.h>

/*
 * Command processor
 */

/*! Execute a command locally or send it to be processed remotely

 @param	process[in]				process object
 @param	cmd[in]					command

 @return < 0 on error
 */
extern int ia_css_process_cmd(
	ia_css_process_t						*process,
	const ia_css_process_cmd_t				cmd);

/*! Get the stored size of the process object

 @param	process[in]				process object

 @return size, 0 on error
 */
extern size_t ia_css_process_get_size(
	const ia_css_process_t					*process);

/*! Get the (pointer to) the process group parent of the process object

 @param	process[in]				process object

 @return the pointer to the parent, NULL on error
 */
extern ia_css_process_group_t *ia_css_process_get_parent(
	const ia_css_process_t					*process);

/*! Set the (pointer to) the process group parent of the process object

 @param	process[in]				process object
 @param	parent[in]				(pointer to the) process group parent object

 @return < 0 on error
 */
extern int ia_css_process_set_parent(
	ia_css_process_t						*process,
	ia_css_process_group_t					*parent);

/*! Get the unique ID of program used by the process object

 @param	process[in]				process object

 @return ID, 0 on error
 */
extern ia_css_program_ID_t ia_css_process_get_program_ID(
	const ia_css_process_t					*process);

/*! Get the state of the the process object

 @param	process[in]				process object

 @return state, limit value on error
 */
extern ia_css_process_state_t ia_css_process_get_state(
	const ia_css_process_t					*process_group);

/*! Get the assigned cell of the the process object

 @param	process[in]				process object

 @return cell ID, limit value on error
 */
extern vied_nci_cell_ID_t ia_css_process_get_cell(
	const ia_css_process_t					*process);

/*! Get the number of cells the process object depends on

 @param	process[in]				process object

 @return number of cells
 */
extern uint8_t ia_css_process_get_cell_dependency_count(
	const ia_css_process_t					*process);

/*! Get the number of terminals the process object depends on

 @param	process[in]				process object

 @return number of terminals
 */
extern uint8_t ia_css_process_get_terminal_dependency_count(
	const ia_css_process_t					*process);

#endif /* __IA_CSS_PSYS_PROCESS_H_INCLUDED__ */
