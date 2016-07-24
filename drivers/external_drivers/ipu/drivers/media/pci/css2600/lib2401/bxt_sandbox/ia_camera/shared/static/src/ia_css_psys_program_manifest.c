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


#include <ia_css_psys_program_manifest.h>
#include <ia_css_psys_program_group_manifest.h>

#include <ia_css_program_group_param_types.h>	/* Kernel enable bitmap */

#include <vied_nci_psys_system_global.h>
#include "ia_css_psys_program_group_private.h"

#include <type_support.h>
#include <error_support.h>

size_t ia_css_sizeof_program_manifest(
	const uint8_t							program_dependency_count,
	const uint8_t							terminal_dependency_count)
{
	size_t	size = 0;

	size += sizeof(ia_css_program_manifest_t);

	size += program_dependency_count * sizeof(uint8_t);
	size += terminal_dependency_count * sizeof(uint8_t);

	return size;
}

bool ia_css_has_program_manifest_fixed_cell(
	const ia_css_program_manifest_t			*manifest)
{
	bool	has_fixed_cell = false;

	vied_nci_cell_ID_t		cell_id;
	vied_nci_cell_type_ID_t	cell_type_id;

	verifexit(manifest != NULL, EINVAL);

	cell_id = ia_css_program_manifest_get_cell_ID(manifest);
	cell_type_id = ia_css_program_manifest_get_cell_type_ID(manifest);
	has_fixed_cell = ((cell_id != VIED_NCI_N_CELL_ID) && (cell_type_id == VIED_NCI_N_CELL_TYPE_ID));
EXIT:
	return has_fixed_cell;
}

size_t ia_css_program_manifest_get_size(
	const ia_css_program_manifest_t			*manifest)
{
	size_t	size = 0;

	if (manifest != NULL) {
		size = manifest->size;
	}

	return size;
}

ia_css_program_ID_t ia_css_program_manifest_get_program_ID(
	const ia_css_program_manifest_t			*manifest)
{
	ia_css_program_ID_t		program_id = 0;

	if (manifest != NULL) {
		program_id = manifest->ID;
	}

	return program_id;
}

int ia_css_program_manifest_set_program_ID(
	ia_css_program_manifest_t			*manifest,
	ia_css_program_ID_t id)
{
	int ret = -1;

	if (manifest != NULL) {
		manifest->ID = id;
		ret = 0;
	}

	return ret;
}

ia_css_program_group_manifest_t *ia_css_program_manifest_get_parent(
	const ia_css_program_manifest_t			*manifest)
{
	ia_css_program_group_manifest_t	*parent = NULL;
	char *base;
	verifexit(manifest != NULL, EINVAL);

	base = (char*)
		((char*)manifest + manifest->parent_offset);

	parent = (ia_css_program_group_manifest_t*) (base);
EXIT:
	return parent;
}

int ia_css_program_manifest_set_parent_offset(
	ia_css_program_manifest_t	*manifest,
	int32_t program_offset)
{
	int	retval = -1;

	verifexit(manifest != NULL, EINVAL);

	/* parent is at negative offset away from current program offset*/
	manifest->parent_offset = -program_offset;

	retval = 0;
EXIT:
	return retval;
}

ia_css_program_type_t ia_css_program_manifest_get_type(
	const ia_css_program_manifest_t			*manifest)
{
	ia_css_program_type_t	program_type = IA_CSS_N_PROGRAM_TYPES;

	if (manifest != NULL) {
		program_type = manifest->program_type;
	}

	return program_type;
}

int ia_css_program_manifest_set_type(
	ia_css_program_manifest_t				*manifest,
	const ia_css_program_type_t				program_type)
{
	int	retval = -1;

	if (manifest != NULL) {
		manifest->program_type = program_type;
		retval = 0;
	}

	return retval;
}

ia_css_kernel_bitmap_t ia_css_program_manifest_get_kernel_bitmap(
	const ia_css_program_manifest_t			*manifest)
{
	ia_css_kernel_bitmap_t	kernel_bitmap = 0;

	if (manifest != NULL) {
		kernel_bitmap = manifest->kernel_bitmap;
	}

	return kernel_bitmap;
}

int ia_css_program_manifest_set_kernel_bitmap(
	ia_css_program_manifest_t				*manifest,
	const ia_css_kernel_bitmap_t			kernel_bitmap)
{
	int	retval = -1;

	if (manifest != NULL) {
		manifest->kernel_bitmap = kernel_bitmap;
		retval = 0;
	}

	return retval;
}

vied_nci_cell_ID_t ia_css_program_manifest_get_cell_ID(
	const ia_css_program_manifest_t			*manifest)
{
	vied_nci_cell_ID_t		cell_id = VIED_NCI_N_CELL_ID;

	verifexit(manifest != NULL, EINVAL);

	cell_id = (vied_nci_cell_ID_t)(manifest->cell_id);
EXIT:
	return cell_id;
}

vied_nci_cell_type_ID_t ia_css_program_manifest_get_cell_type_ID(
	const ia_css_program_manifest_t			*manifest)
{
	vied_nci_cell_type_ID_t	cell_type_id = VIED_NCI_N_CELL_TYPE_ID;

	verifexit(manifest != NULL, EINVAL);

	cell_type_id = (vied_nci_cell_type_ID_t)(manifest->cell_type_id);
EXIT:
	return cell_type_id;
}

vied_nci_resource_size_t ia_css_program_manifest_get_int_mem_size(
	const ia_css_program_manifest_t			*manifest,
	const vied_nci_mem_type_ID_t			mem_type_id)
{
	vied_nci_resource_size_t	int_mem_size = 0;

	verifexit(manifest != NULL, EINVAL);
	verifexit(mem_type_id < VIED_NCI_N_MEM_TYPE_ID, EINVAL);

	int_mem_size = manifest->int_mem_size[mem_type_id];
EXIT:
	return int_mem_size;
}

vied_nci_resource_size_t ia_css_program_manifest_get_ext_mem_size(
	const ia_css_program_manifest_t			*manifest,
	const vied_nci_mem_type_ID_t			mem_type_id)
{
	vied_nci_resource_size_t	ext_mem_size = 0;

	verifexit(manifest != NULL, EINVAL);
	verifexit(mem_type_id < VIED_NCI_N_DATA_MEM_TYPE_ID, EINVAL);

	ext_mem_size = manifest->ext_mem_size[mem_type_id];
EXIT:
	return ext_mem_size;
}

vied_nci_resource_size_t ia_css_program_manifest_get_dev_chn_size(
	const ia_css_program_manifest_t			*manifest,
	const vied_nci_dev_chn_ID_t				dev_chn_id)
{
	vied_nci_resource_size_t	dev_chn_size = 0;

	verifexit(manifest != NULL, EINVAL);
	verifexit(dev_chn_id < VIED_NCI_N_DEV_CHN_ID, EINVAL);

	dev_chn_size = manifest->dev_chn_size[dev_chn_id];
EXIT:
	return dev_chn_size;
}

uint8_t ia_css_program_manifest_get_program_dependency_count(
	const ia_css_program_manifest_t			*manifest)
{
	uint8_t	program_dependency_count = 0;
	if (manifest != NULL) {
		program_dependency_count = manifest->program_dependency_count;
	}
	return program_dependency_count;
}

uint8_t ia_css_program_manifest_get_program_dependency(
	const ia_css_program_manifest_t			*manifest,
	const unsigned int						index)
{
	uint8_t	program_dependency = ~0;
	uint8_t *program_dep_ptr;
	uint8_t	program_dependency_count = ia_css_program_manifest_get_program_dependency_count(manifest);
	if (index < program_dependency_count) {
		program_dep_ptr = (uint8_t*)((uint8_t*)manifest + manifest->program_dependency_offset +
				index * sizeof(uint8_t));
		program_dependency = *program_dep_ptr;
	}
	return program_dependency;
}

int ia_css_program_manifest_set_program_dependency(
	ia_css_program_manifest_t				*manifest,
	const uint8_t							program_dependency,
	const unsigned int						index)
{
	int	retval = -1;
	uint8_t *program_dep_ptr;
	uint8_t	program_dependency_count = ia_css_program_manifest_get_program_dependency_count(manifest);
	uint8_t	program_count = ia_css_program_group_manifest_get_program_count(ia_css_program_manifest_get_parent(manifest));
	if ((index < program_dependency_count) && (program_dependency < program_count)) {
		program_dep_ptr = (uint8_t*)((uint8_t*)manifest + manifest->program_dependency_offset + index * sizeof(uint8_t));
		 *program_dep_ptr = program_dependency;
		retval = 0;
	}
	return retval;
}

uint8_t ia_css_program_manifest_get_terminal_dependency_count(
	const ia_css_program_manifest_t			*manifest)
{
	uint8_t	terminal_dependency_count = 0;
	if (manifest != NULL) {
		terminal_dependency_count = manifest->terminal_dependency_count;
	}
	return terminal_dependency_count;
}

uint8_t ia_css_program_manifest_get_terminal_dependency(
	const ia_css_program_manifest_t	*manifest,
	const unsigned int	index)
{
	uint8_t	terminal_dependency = ~0;
	uint8_t *terminal_dep_ptr;
	uint8_t	terminal_dependency_count = ia_css_program_manifest_get_terminal_dependency_count(manifest);
	if (index < terminal_dependency_count) {
		terminal_dep_ptr = (uint8_t*)((uint8_t*)manifest + manifest->terminal_dependency_offset + index);
		terminal_dependency = *terminal_dep_ptr;
	}
	return terminal_dependency;
}

int ia_css_program_manifest_set_terminal_dependency(
	ia_css_program_manifest_t				*manifest,
	const uint8_t							terminal_dependency,
	const unsigned int						index)
{
	int	retval = -1;
	uint8_t *terminal_dep_ptr;
	uint8_t	terminal_dependency_count = ia_css_program_manifest_get_terminal_dependency_count(manifest);
	uint8_t	terminal_count = ia_css_program_group_manifest_get_terminal_count(ia_css_program_manifest_get_parent(manifest));
	if ((index < terminal_dependency_count) &&
			(terminal_dependency < terminal_count)) {
		terminal_dep_ptr = (uint8_t*)((uint8_t*)manifest + manifest->terminal_dependency_offset + index);
		 *terminal_dep_ptr = terminal_dependency;
		retval = 0;
	}
	return retval;
}

bool ia_css_is_program_manifest_subnode_program_type(
	const ia_css_program_manifest_t			*manifest)
{
	ia_css_program_type_t		program_type = ia_css_program_manifest_get_type(manifest);
/* The error return is the limit value, so no need to check on the manifest pointer */
	return (program_type == IA_CSS_PROGRAM_TYPE_PARALLEL_SUB) || (program_type == IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB);
}

bool ia_css_is_program_manifest_supernode_program_type(
	const ia_css_program_manifest_t			*manifest)
{
	ia_css_program_type_t		program_type = ia_css_program_manifest_get_type(manifest);
/* The error return is the limit value, so no need to check on the manifest pointer */
	return (program_type == IA_CSS_PROGRAM_TYPE_PARALLEL_SUPER) || (program_type == IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER);
}

bool ia_css_is_program_manifest_singular_program_type(
	const ia_css_program_manifest_t			*manifest)
{
	ia_css_program_type_t		program_type = ia_css_program_manifest_get_type(manifest);
/* The error return is the limit value, so no need to check on the manifest pointer */
	return (program_type == IA_CSS_PROGRAM_TYPE_SINGULAR);
}

void ia_css_program_manifest_init(
	ia_css_program_manifest_t	*blob,
	const uint8_t	program_dependency_count,
	const uint8_t	terminal_dependency_count)
{
	/*TODO: add assert*/
	if (!blob)
		return;
	blob->ID = 1;
	blob->program_dependency_count = program_dependency_count;
	blob->terminal_dependency_count = terminal_dependency_count;
	blob->program_dependency_offset = sizeof(ia_css_program_manifest_t);
	blob->terminal_dependency_offset = blob->program_dependency_offset +
		sizeof(uint8_t) * program_dependency_count;
	blob->size = ia_css_sizeof_program_manifest(program_dependency_count,
			terminal_dependency_count);
}
