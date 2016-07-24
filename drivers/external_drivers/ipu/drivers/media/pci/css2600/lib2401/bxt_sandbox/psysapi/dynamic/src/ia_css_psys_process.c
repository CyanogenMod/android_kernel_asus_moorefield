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


#include <ia_css_psys_process.h>

#include <ia_css_psys_process_group.h>
#include <ia_css_psys_program_manifest.h>

#include <error_support.h>
#include <print_support.h>
#include <misc_support.h>
/* #include <string_support.h>		memset() */
/* #include <string.h	*/
#include <assert_support.h>
#include <cpu_mem_support.h>

#include <vied_nci_psys_system_global.h>

struct ia_css_process_s {
	uint32_t							size;											/**< Size of this structure */
	ia_css_program_ID_t					ID;												/**< Referal ID to a specific program FW */
	ia_css_process_group_t				*parent;										/**< Reference to the process group */
	ia_css_process_state_t				state;											/**< State of the process FSM dependent on the parent FSM */
	vied_nci_resource_id_t				cell_id;										/**< (mandatory) specification of a cell to be used by this process */
	vied_nci_resource_id_t				int_mem_id[VIED_NCI_N_MEM_TYPE_ID];				/**< (internal) Memory ID; This is redundant, derived from cell_id */
	vied_nci_resource_size_t			int_mem_offset[VIED_NCI_N_MEM_TYPE_ID];			/**< (internal) Memory allocation offset given to this process */
	vied_nci_resource_id_t				ext_mem_id[VIED_NCI_N_DATA_MEM_TYPE_ID];		/**< (external) Memory ID */
	vied_nci_resource_size_t			ext_mem_offset[VIED_NCI_N_DATA_MEM_TYPE_ID];	/**< (external) Memory allocation offset given to this process */
	vied_nci_resource_size_t			dev_chn_offset[VIED_NCI_N_DEV_CHN_ID];			/**< Device channel allocation offset given to this process */
	uint8_t								cell_dependency_count;							/**< Number of processes (mapped on cells) this process depends on */
	uint8_t								terminal_dependency_count;						/**< Number of terminals this process depends on */
	vied_nci_resource_id_t				*cell_dependencies;								/**< Array[dependency_count] of ID's of the cells that provide input */
	uint8_t								*terminal_dependencies;							/**< Array[terminal_dependency_count] of indices of connected terminals */
};



size_t ia_css_sizeof_process(
	const ia_css_program_manifest_t			*manifest,
	const ia_css_program_param_t			*param)
{
	size_t	size = 0;

	uint8_t	program_dependency_count, terminal_dependency_count;

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);

	size += sizeof(ia_css_process_t);

	program_dependency_count = ia_css_program_manifest_get_program_dependency_count(manifest);
	terminal_dependency_count = ia_css_program_manifest_get_terminal_dependency_count(manifest);

	size += program_dependency_count*sizeof(vied_nci_resource_id_t);
	size += terminal_dependency_count*sizeof(uint8_t);

EXIT:
	return size;
}

ia_css_process_t *ia_css_process_create(
	const ia_css_program_manifest_t		*manifest,
	const ia_css_program_param_t		*param)
{
	size_t	size = 0;
	int retval = -1;
	ia_css_process_t	*process = NULL;

	/* size_t	size = ia_css_sizeof_process(manifest, param); */
	uint8_t	program_dependency_count, terminal_dependency_count;

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);

	process = (ia_css_process_t *)ia_css_cpu_mem_alloc(sizeof(ia_css_process_t));
	verifexit(process != NULL, EINVAL);
	ia_css_cpu_mem_set_zero(process, sizeof(ia_css_process_t));
	size += sizeof(ia_css_process_t);

	process->state = IA_CSS_PROCESS_CREATED;

	program_dependency_count = ia_css_program_manifest_get_program_dependency_count(manifest);
	terminal_dependency_count = ia_css_program_manifest_get_terminal_dependency_count(manifest);

	/* A process requires at least one input or output */
    verifexit((program_dependency_count + terminal_dependency_count) != 0, EINVAL);

	if (program_dependency_count != 0) {
		process->cell_dependencies = (vied_nci_resource_id_t *)ia_css_cpu_mem_alloc(program_dependency_count * sizeof(vied_nci_resource_id_t));
		verifexit(process->cell_dependencies != NULL, EINVAL);
		size += program_dependency_count * sizeof(vied_nci_resource_id_t);
		ia_css_cpu_mem_set_zero((void *)process->cell_dependencies, program_dependency_count * sizeof(uint8_t));
	}

	if (terminal_dependency_count != 0) {
		process->terminal_dependencies = (uint8_t *)ia_css_cpu_mem_alloc(terminal_dependency_count * sizeof(uint8_t));
		verifexit(process->terminal_dependencies != NULL, EINVAL);
		size += terminal_dependency_count * sizeof(uint8_t);
		ia_css_cpu_mem_set_zero((void *)process->terminal_dependencies, terminal_dependency_count * sizeof(uint8_t));
	}

	process->size = ia_css_sizeof_process(manifest, param);
	verifexit(process->size == size, EINVAL);

	process->ID = ia_css_program_manifest_get_program_ID(manifest);

	verifexit(process->ID != 0, EINVAL);

	process->cell_dependency_count = program_dependency_count;
	process->terminal_dependency_count = terminal_dependency_count;

	process->parent = NULL;

	verifexit(ia_css_process_clear_all(process) == 0, EINVAL);

	process->state = IA_CSS_PROCESS_READY;
	retval = 0;

EXIT:
	if (retval != 0) {
		process = ia_css_process_destroy(process);
	}
	return process;
}

ia_css_process_t *ia_css_process_destroy(
	ia_css_process_t				 		*process)
{
	if (process != NULL) {
		if (process->cell_dependencies != NULL) {
		ia_css_cpu_mem_free((void *)process->cell_dependencies);
		}
		if (process->terminal_dependencies != NULL) {
		ia_css_cpu_mem_free((void *)process->terminal_dependencies);
		}
		ia_css_cpu_mem_free((void *)process);
		process = NULL;
	}
	return process;
}

int ia_css_process_set_cell(
	ia_css_process_t						*process,
	const vied_nci_cell_ID_t				cell_id)
{
	int	retval = -1;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	vied_nci_resource_bitmap_t		resource_bitmap, bit_mask;
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(parent != NULL, EINVAL);
/* Some programs are mapped on a fixed cell, when the process group is created */
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED) || (parent_state == IA_CSS_PROCESS_GROUP_CREATED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);

/* Some programs are mapped on a fixed cell, thus check is not secure, but it will detect a preset, the process manager will do the secure check */
	verifexit(ia_css_process_get_cell(process) == VIED_NCI_N_CELL_ID, EINVAL);

	bit_mask = vied_nci_cell_bit_mask(cell_id);
	resource_bitmap = ia_css_process_group_get_resource_bitmap(parent);

	verifexit(bit_mask != 0, EINVAL);
	verifexit(vied_nci_is_bitmap_clear(bit_mask, resource_bitmap), EINVAL);

	process->cell_id = (vied_nci_resource_id_t)cell_id;
	resource_bitmap = vied_nci_bitmap_set(resource_bitmap, bit_mask);

	retval = ia_css_process_group_set_resource_bitmap(parent, resource_bitmap);
EXIT:
	return retval;
}

int ia_css_process_clear_cell(
	ia_css_process_t						*process)
{
	int	retval = -1;
	vied_nci_cell_ID_t				cell_id = ia_css_process_get_cell(process);
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	vied_nci_resource_bitmap_t		resource_bitmap, bit_mask;
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(parent != NULL, EINVAL);
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);

	bit_mask = vied_nci_cell_bit_mask(cell_id);
	resource_bitmap = ia_css_process_group_get_resource_bitmap(parent);

	verifexit(bit_mask != 0, EINVAL);
	verifexit(vied_nci_is_bitmap_set(bit_mask, resource_bitmap), EINVAL);

	process->cell_id = (vied_nci_resource_id_t)VIED_NCI_N_CELL_ID;
	resource_bitmap = vied_nci_bitmap_clear(resource_bitmap, bit_mask);

	retval = ia_css_process_group_set_resource_bitmap(parent, resource_bitmap);
EXIT:
	return retval;
}

vied_nci_cell_ID_t ia_css_process_get_cell(
	const ia_css_process_t					*process)
{
	vied_nci_cell_ID_t	cell_id = VIED_NCI_N_CELL_ID;
	if (process != NULL) {
		cell_id = (vied_nci_cell_ID_t)(process->cell_id);
	}
	return cell_id;
}

int ia_css_process_set_int_mem(
	ia_css_process_t						*process,
	const vied_nci_mem_ID_t					mem_id,
	const vied_nci_resource_size_t			offset)
{
	int	retval = -1;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	vied_nci_cell_ID_t				cell_id = ia_css_process_get_cell(process);
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);

/* "vied_nci_has_cell_mem_of_id()" will return false on error, e.g. when the cell or mem id is invalid  */
	if (vied_nci_has_cell_mem_of_id(cell_id, mem_id)) {
		vied_nci_mem_type_ID_t	mem_type = vied_nci_mem_get_type(mem_id);
assert(mem_type < VIED_NCI_N_MEM_TYPE_ID);
		process->int_mem_id[mem_type] = mem_id;
		process->int_mem_offset[mem_type] = offset;
		retval = 0;
	}
EXIT:
	return retval;
}

int ia_css_process_clear_int_mem(
	ia_css_process_t						*process,
	const vied_nci_mem_type_ID_t			mem_type_id)
{
	int	retval = -1;
	int	mem_index;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	vied_nci_cell_ID_t				cell_id = ia_css_process_get_cell(process);
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);
	verifexit(mem_type_id < VIED_NCI_N_MEM_TYPE_ID, EINVAL);

/* We could just clear the field, but lets check the state for consistency first */
	for (mem_index = 0; mem_index < (int)VIED_NCI_N_MEM_TYPE_ID; mem_index++) {
		if (vied_nci_is_cell_mem_of_type(cell_id, mem_index, mem_type_id)) {
			vied_nci_mem_ID_t	mem_id = vied_nci_cell_get_mem(cell_id, mem_index);
assert(vied_nci_is_mem_of_type(mem_id, mem_type_id));
assert((process->int_mem_id[mem_type_id] == mem_id) || (process->int_mem_id[mem_type_id] == VIED_NCI_N_MEM_ID));
			process->int_mem_id[mem_type_id] = VIED_NCI_N_MEM_ID;
			process->int_mem_offset[mem_type_id] = ~0;
			retval = 0;
		}
	}

EXIT:
	return retval;
}


int ia_css_process_set_ext_mem(
	ia_css_process_t						*process,
	const vied_nci_mem_ID_t					mem_id,
	const vied_nci_resource_size_t			offset)
{
	int	retval = -1;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	vied_nci_cell_ID_t				cell_id = ia_css_process_get_cell(process);
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);

/* Check that the memory actually exists, "vied_nci_has_cell_mem_of_id()" will return false on error */
	if (!vied_nci_has_cell_mem_of_id(cell_id, mem_id) && (mem_id < VIED_NCI_N_MEM_ID)) {
		vied_nci_mem_type_ID_t	mem_type = vied_nci_mem_get_type(mem_id);
		verifexit(mem_type < VIED_NCI_N_DATA_MEM_TYPE_ID, EINVAL);
		process->ext_mem_id[mem_type] = mem_id;
		process->ext_mem_offset[mem_type] = offset;
		retval = 0;
	}

EXIT:
	return retval;
}

int ia_css_process_clear_ext_mem(
	ia_css_process_t						*process,
	const vied_nci_mem_type_ID_t			mem_type_id)
{
	int	retval = -1;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);
	verifexit(mem_type_id < VIED_NCI_N_DATA_MEM_TYPE_ID, EINVAL);

	process->ext_mem_id[mem_type_id] = VIED_NCI_N_MEM_ID;
	process->ext_mem_offset[mem_type_id] = ~0;

	retval = 0;
EXIT:
	return retval;
}

int ia_css_process_set_dev_chn(
	ia_css_process_t						*process,
	const vied_nci_dev_chn_ID_t				dev_chn_id,
	const vied_nci_resource_size_t			offset)
{
	int	retval = -1;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);

	verifexit(dev_chn_id <= VIED_NCI_N_DEV_CHN_ID, EINVAL);

	process->dev_chn_offset[dev_chn_id] = offset;

	retval = 0;
EXIT:
	return retval;
}

int ia_css_process_clear_dev_chn(
	ia_css_process_t						*process,
	const vied_nci_dev_chn_ID_t				dev_chn_id)
{
	int	retval = -1;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	ia_css_process_group_state_t	parent_state = ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(((parent_state == IA_CSS_PROCESS_GROUP_BLOCKED) || (parent_state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
	verifexit(state == IA_CSS_PROCESS_READY, EINVAL);

	verifexit(dev_chn_id <= VIED_NCI_N_DEV_CHN_ID, EINVAL);

	process->dev_chn_offset[dev_chn_id] = ~0;

	retval = 0;
EXIT:
	return retval;
}

int ia_css_process_clear_all(
	ia_css_process_t						*process)
{
	int	retval = -1;
	ia_css_process_group_t			*parent = ia_css_process_get_parent(process);
	ia_css_process_group_state_t	parent_state =  ia_css_process_group_get_state(parent);
	ia_css_process_state_t			state = ia_css_process_get_state(process);
	int	mem_index;
	int	dev_chn_index;

	verifexit(process != NULL, EINVAL);
/* Resource clear can only be called in excluded states contrary to set */
	verifexit((parent_state != IA_CSS_PROCESS_GROUP_RUNNING) || (parent_state == IA_CSS_N_PROCESS_GROUP_STATES), EINVAL);
	verifexit((state == IA_CSS_PROCESS_CREATED) || (state == IA_CSS_PROCESS_READY), EINVAL);

	for (dev_chn_index = 0; dev_chn_index < VIED_NCI_N_DEV_CHN_ID; dev_chn_index++) {
		process->dev_chn_offset[dev_chn_index] = ~0;
	}
/* No difference whether a cell_id has been set or not, clear all */
	for (mem_index = 0; mem_index < VIED_NCI_N_DATA_MEM_TYPE_ID; mem_index++) {
		process->ext_mem_id[mem_index] = VIED_NCI_N_MEM_ID;
		process->ext_mem_offset[mem_index] = ~0;
	}
	for (mem_index = 0; mem_index < VIED_NCI_N_MEM_TYPE_ID; mem_index++) {
		process->int_mem_id[mem_index] = VIED_NCI_N_MEM_ID;
		process->int_mem_offset[mem_index] = ~0;
	}
	process->cell_id = (vied_nci_resource_id_t)VIED_NCI_N_CELL_ID;

	retval = 0;
EXIT:
	return retval;
}

int ia_css_process_acquire(
	ia_css_process_t						*process)
{
	int	retval = -1;

	verifexit(process != NULL, EINVAL);
	retval = 0;
EXIT:
	return retval;
}

int ia_css_process_release(
	ia_css_process_t						*process)
{
	int	retval = -1;

	verifexit(process != NULL, EINVAL);
	retval = 0;
EXIT:
	return retval;
}

int ia_css_process_print(
	const ia_css_process_t					*process,
	void									*fid)
{
	int		retval = -1;
	int		i, mem_index, dev_chn_index;
	uint8_t	cell_dependency_count, terminal_dependency_count;
	vied_nci_cell_ID_t	cell_id = ia_css_process_get_cell(process);

	NOT_USED(fid);

	verifexit(process != NULL, EINVAL);

	PRINT("ia_css_process_print\n");
	PRINT("sizeof(process) = %d\n",(int)ia_css_process_get_size(process));
	PRINT("program(process) = %d\n",(int)ia_css_process_get_program_ID(process));
/*	PRINT("program(process) = %s\n",(ia_css_program_ID_string(ia_css_process_get_program_ID(process))); */
	PRINT("state(process) = %d\n",(int)ia_css_process_get_state(process));
	PRINT("parent(process) = %p\n",(void *)ia_css_process_get_parent(process));

	PRINT("cell(process) = %d\n",(int)process->cell_id);
/*	PRINT("cell(process) = %s\n",vied_nci_cell_string(process->cell_id)); */


	for (mem_index = 0; mem_index < (int)VIED_NCI_N_MEM_TYPE_ID; mem_index++) {
		vied_nci_mem_type_ID_t	mem_type = vied_nci_cell_get_mem_type(cell_id, mem_index);
		vied_nci_mem_ID_t		mem_id = (vied_nci_mem_ID_t)(process->int_mem_id[mem_index]);

		verifexit(((mem_id == vied_nci_cell_get_mem(cell_id, mem_index)) || (mem_id == VIED_NCI_N_MEM_ID)), EINVAL);

		PRINT("\ttype(internal mem) type = %d\n",(int)mem_type);
/*		PRINT("\ttype(internal mem) type = %s\n",vied_nci_mem_type_string(mem_type)); */
		PRINT("\tid(internal mem) id = %d\n",(int)mem_id);
/*		PRINT("\tid(internal mem) id = %s\n",vied_nci_mem_ID_string(mem_id)); */
		PRINT("\ttype(internal mem) offset = %d\n",process->int_mem_offset[mem_index]);
	}

	for (mem_index = 0; mem_index < (int)VIED_NCI_N_DATA_MEM_TYPE_ID; mem_index++) {
		vied_nci_mem_ID_t		mem_id = (vied_nci_mem_ID_t)(process->ext_mem_id[mem_index]);
		vied_nci_mem_type_ID_t	mem_type = vied_nci_mem_get_type(mem_id);

		verifexit((!vied_nci_has_cell_mem_of_id(cell_id, mem_id) || (mem_id == VIED_NCI_N_MEM_ID)), EINVAL);

		PRINT("\ttype(external mem) type = %d\n",(int)mem_type);
/*		PRINT("\ttype(external mem) type = %s\n",vied_nci_mem_type_string(mem_type)); */
		PRINT("\tid(external mem) id = %d\n",(int)mem_id);
/*		PRINT("\tid(external mem) id = %s\n",vied_nci_mem_ID_string(mem_id)); */
		PRINT("\ttype(external mem) offset = %d\n",process->ext_mem_offset[mem_index]);
	}

	for (dev_chn_index = 0; dev_chn_index < (int)VIED_NCI_N_DEV_CHN_ID; dev_chn_index++) {
		PRINT("\ttype(device channel) type = %d\n",(int)dev_chn_index);
/*		PRINT("\ttype(device channel) type = %s\n",vied_nci_dev_chn_type_string(dev_chn_index)); */
		PRINT("\ttype(device channel) offset = %d\n",process->dev_chn_offset[dev_chn_index]);
	}

	cell_dependency_count = ia_css_process_get_cell_dependency_count(process);
	if (cell_dependency_count == 0) {
		PRINT("cell_dependencies[%d] {};\n",cell_dependency_count);
	} else {
		PRINT("cell_dependencies[%d] {",cell_dependency_count);
		for (i = 0; i < (int)cell_dependency_count - 1; i++) {
			PRINT("%4d, ",process->cell_dependencies[i]);
		}
		PRINT("%4d}\n",process->cell_dependencies[i]);
	}

	terminal_dependency_count = ia_css_process_get_terminal_dependency_count(process);
	if (terminal_dependency_count == 0) {
		PRINT("terminal_dependencies[%d] {};\n",terminal_dependency_count);
	} else {
		terminal_dependency_count = ia_css_process_get_terminal_dependency_count(process);
		PRINT("terminal_dependencies[%d] {",terminal_dependency_count);
		for (i = 0; i < (int)terminal_dependency_count - 1; i++) {
			PRINT("%4d, ",process->terminal_dependencies[i]);
		}
		PRINT("%4d}\n",process->terminal_dependencies[i]);
	}

	retval = 0;
EXIT:
	return retval;
}


size_t ia_css_process_get_size(
	const ia_css_process_t					*process)
{
	size_t	size = 0;

	if (process != NULL) {
		size = process->size;
	}

	return size;
}

ia_css_process_state_t ia_css_process_get_state(
	const ia_css_process_t					*process)
{
	ia_css_process_state_t	state = IA_CSS_N_PROCESS_STATES;

	if (process != NULL) {
		state = process->state;
	}

	return state;
}

uint8_t ia_css_process_get_cell_dependency_count(
	const ia_css_process_t					*process)
{
	uint8_t	cell_dependency_count = 0;

	verifexit(process != NULL, EINVAL);
	cell_dependency_count = process->cell_dependency_count;

EXIT:
	return cell_dependency_count;
}

uint8_t ia_css_process_get_terminal_dependency_count(
	const ia_css_process_t					*process)
{
	uint8_t	terminal_dependency_count = 0;

	verifexit(process != NULL, EINVAL);
	terminal_dependency_count = process->terminal_dependency_count;

EXIT:
	return terminal_dependency_count;
}

int ia_css_process_set_parent(
	ia_css_process_t						*process,
	ia_css_process_group_t					*parent)
{
	int	retval = -1;

	verifexit(process != NULL, EINVAL);
	verifexit(parent != NULL, EINVAL);

	process->parent = parent;
	retval = 0;
EXIT:
	return retval;
}

ia_css_process_group_t *ia_css_process_get_parent(
	const ia_css_process_t					*process)
{
	ia_css_process_group_t	*parent = NULL;

	verifexit(process != NULL, EINVAL);

	parent = process->parent;
EXIT:
	return parent;
}

ia_css_program_ID_t ia_css_process_get_program_ID(
	const ia_css_process_t					*process)
{
	ia_css_program_ID_t		id = 0;

	verifexit(process != NULL, EINVAL);

	id = process->ID;

EXIT:
	return id;
}

int ia_css_process_cmd(
	ia_css_process_t						*process,
	const ia_css_process_cmd_t				cmd)
{
	int	retval = -1;
	ia_css_process_state_t	state = ia_css_process_get_state(process);

	verifexit(process != NULL, EINVAL);
	verifexit(state != IA_CSS_PROCESS_ERROR, EINVAL);
	verifexit(state < IA_CSS_N_PROCESS_STATES, EINVAL);

	switch (cmd) {
	case IA_CSS_PROCESS_CMD_NOP:
		break;
	case IA_CSS_PROCESS_CMD_ACQUIRE:
		verifexit(state == IA_CSS_PROCESS_READY, EINVAL);
		break;
	case IA_CSS_PROCESS_CMD_RELEASE:
		verifexit(state == IA_CSS_PROCESS_READY, EINVAL);
		break;
	case IA_CSS_PROCESS_CMD_START:
		verifexit((state == IA_CSS_PROCESS_READY) || (state == IA_CSS_PROCESS_STOPPED), EINVAL);
		process->state = IA_CSS_PROCESS_STARTED;
		break;
	case IA_CSS_PROCESS_CMD_LOAD:
		verifexit(state == IA_CSS_PROCESS_STARTED, EINVAL);
		process->state = IA_CSS_PROCESS_RUNNING;
		break;
	case IA_CSS_PROCESS_CMD_STOP:
		verifexit((state == IA_CSS_PROCESS_RUNNING) || (state == IA_CSS_PROCESS_SUSPENDED), EINVAL);
		process->state = IA_CSS_PROCESS_STOPPED;
		break;
	case IA_CSS_PROCESS_CMD_SUSPEND:
		verifexit(state == IA_CSS_PROCESS_RUNNING, EINVAL);
		process->state = IA_CSS_PROCESS_SUSPENDED;
		break;
	case IA_CSS_PROCESS_CMD_RESUME:
		verifexit(state == IA_CSS_PROCESS_SUSPENDED, EINVAL);
		process->state = IA_CSS_PROCESS_RUNNING;
		break;
	case IA_CSS_N_PROCESS_CMDS:		/* Fall through */
	default:
		verifexit(false, EINVAL);
		break;
	}
	retval = 0;
EXIT:
	return retval;
}

