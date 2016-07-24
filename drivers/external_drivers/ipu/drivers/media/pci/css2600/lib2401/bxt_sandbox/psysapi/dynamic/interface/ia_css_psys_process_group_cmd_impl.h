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

#ifndef __IA_CSS_PSYS_PROCESS_GROUP_CMD_IMPL_H_
#define __IA_CSS_PSYS_PROCESS_GROUP_CMD_IMPL_H_

#include "type_support.h"
#include "ia_css_psys_process_group.h"

struct ia_css_process_group_s {
	uint32_t							size;											/**< Size of this structure */
	ia_css_program_group_ID_t			ID;												/**< Referal ID to program group FW */
	ia_css_process_group_state_t		state;											/**< State of the process group FSM */
	uint64_t							token;											/**< User (callback) token / user context reference, zero is an error value */
	uint64_t							private_token;									/**< private token / context reference, zero is an error value */
	uint8_t								process_count;									/**< Parameter dependent number of processes in this process group */
	uint8_t				 				terminal_count;									/**< Parameter dependent number of terminals on this process group */
	uint8_t				 				subgraph_count;									/**< Parameter dependent number of independent subgraphs in this process group */
	uint16_t							fragment_count;									/**< Number of fragments offered on each terminal */
	uint16_t							fragment_state;									/**< Current fragment of processing */
	uint16_t							fragment_limit;									/**< Watermark to control fragment processing */
	vied_nci_resource_bitmap_t			resource_bitmap;								/**< Bitmap of the compute resources used by the process group  */
	ia_css_process_t					**processes;									/**< Array[process_count] of process addresses in this process group */
	ia_css_terminal_t					**terminals;									/**< Array[terminal_count] of terminal addresses on this process group */
};

/*! Callback after process group is created. Implementations can provide
 * suitable actions needed when process group is created.

 @param	process_group[in]				process group object
 @param	program_group_manifest[in]		program group manifest
 @param	program_group_param[in]			program group parameters

 @return 0 on success and non-zero on failure
 */
extern int ia_css_process_group_on_create(
	ia_css_process_group_t					*process_group,
	const ia_css_program_group_manifest_t	*program_group_manifest,
	const ia_css_program_group_param_t		*program_group_param);

/*! Callback before process group is about to be destoyed. Any implementation
 * specific cleanups can be done here.

 @param	process_group[in]				process group object

 @return 0 on success and non-zero on failure
 */
extern int ia_css_process_group_on_destroy(
	ia_css_process_group_t					*process_group);

/*
 * Command processor
 */

/*! Execute a command locally or send it to be processed remotely

 @param	process_group[in]		process group object
 @param	cmd[in]					command

 @return < 0 on error
 */
extern int ia_css_process_group_exec_cmd(
	ia_css_process_group_t					*process_group,
	const ia_css_process_group_cmd_t		cmd);


#endif /* __IA_CSS_PSYS_PROCESS_GROUP_CMD_IMPL_H_ */
