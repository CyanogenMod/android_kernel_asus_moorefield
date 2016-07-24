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


#include <ia_css_psys_program_group_manifest.h>

#include <ia_css_psys_program_manifest.h>
#include <ia_css_psys_terminal_manifest.h>

#include <ia_css_program_group_param.h>
#include <ia_css_kernel_bitmap.h>
#include "ia_css_psys_program_group_private.h"
#include <type_support.h>
#include <error_support.h>
#include <print_support.h>
#include <assert_support.h>

/* #include <bitop_support.h> */
#include <vied_nci_psys_system_global.h>	/* Safer bit mask functions */

size_t ia_css_sizeof_program_group_manifest(
	const uint8_t							program_count,
	const uint8_t							terminal_count,
	const uint8_t							*program_dependency_count,
	const uint8_t							*terminal_dependency_count,
	const ia_css_terminal_type_t					*terminal_type,
	const uint16_t							section_count)
{
	size_t	size = 0;
	int		i;

	verifexit(program_count != 0, EINVAL);
	verifexit(terminal_count != 0, EINVAL);
	verifexit(program_dependency_count != NULL, EINVAL);
	verifexit(terminal_dependency_count != NULL, EINVAL);

	size += sizeof(ia_css_program_group_manifest_t);

/* All functions in the loops below can set errno, thus no need to exit on error, all can fail silently */
	for (i = 0; i < (int)program_count; i++) {
		size += ia_css_sizeof_program_manifest(program_dependency_count[i], terminal_dependency_count[i]);
	}

	for (i = 0; i < (int)terminal_count; i++) {
		size += ia_css_sizeof_terminal_manifest(terminal_type[i], section_count);
	}

EXIT:
	return size;
}


/*
 * Model and/or check refinements
 * - Parallel programs do not yet have mutual exclusive alternatives
 * - The pgram dependencies do not need to be acyclic
 * - Parallel programs need to have an equal kernel requirement
 */
bool ia_css_is_program_group_manifest_valid(
	const ia_css_program_group_manifest_t	*manifest)
{
	int		i;
	bool	is_valid = false;

	uint8_t							terminal_count = ia_css_program_group_manifest_get_terminal_count(manifest);
	uint8_t							program_count = ia_css_program_group_manifest_get_program_count(manifest);
	ia_css_kernel_bitmap_t			total_bitmap = ia_css_program_group_manifest_get_kernel_bitmap(manifest);
	ia_css_kernel_bitmap_t			check_bitmap = ia_css_kernel_bitmap_clear();
/* Use a standard bitmap type for the minimum logic to check the DAG, generic functions can be used for the kernel enable bitmaps; Later */
	vied_nci_resource_bitmap_t		terminal_bitmap = vied_nci_bit_mask(VIED_NCI_RESOURCE_BITMAP_BITS);
	int								terminal_bitmap_weight;
	bool							has_parameter_terminal = false;

	verifexit(manifest != NULL, EINVAL);
	verifexit(ia_css_program_group_manifest_get_size(manifest) != 0, EINVAL);
	verifexit(ia_css_program_group_manifest_get_alignment(manifest) != 0, EINVAL);
	verifexit(ia_css_program_group_manifest_get_program_group_ID(manifest) != 0, EINVAL);

	verifexit(program_count != 0, EINVAL);
	verifexit(terminal_count != 0, EINVAL);
	verifexit(!ia_css_is_kernel_bitmap_empty(total_bitmap), EINVAL);
	verifexit(vied_nci_is_bitmap_empty(terminal_bitmap), EINVAL);

/* Check the kernel bitmaps for terminals */
	for (i = 0; i < (int)terminal_count; i++) {
		ia_css_terminal_manifest_t	*terminal_manifest_i = ia_css_program_group_manifest_get_terminal_manifest(manifest, i);
		bool						is_parameter = ia_css_is_terminal_manifest_parameter_terminal(terminal_manifest_i);
		bool						is_data = ia_css_is_terminal_manifest_data_terminal(terminal_manifest_i);

		if (is_parameter) {
/* There can be only one (cached) parameter terminal it serves kernels, not programs */
			verifexit(!has_parameter_terminal, EINVAL);
			has_parameter_terminal = is_parameter;
		} else {
			ia_css_data_terminal_manifest_t	*dterminal_manifest_i = (ia_css_data_terminal_manifest_t *)terminal_manifest_i;
			ia_css_kernel_bitmap_t		terminal_bitmap_i = ia_css_data_terminal_manifest_get_kernel_bitmap(dterminal_manifest_i);
			verifexit(is_data, EINVAL);
/* A terminal must depend on kernels that are a subset of the total, correction, it can only depend on one kernel */
			verifexit(!ia_css_is_kernel_bitmap_empty(terminal_bitmap_i), EINVAL);
			verifexit(ia_css_is_kernel_bitmap_subset(total_bitmap, terminal_bitmap_i), EINVAL);
			verifexit(ia_css_is_kernel_bitmap_onehot(terminal_bitmap_i), EINVAL);
		}
	}

/* Check the kernel bitmaps for programs */
	for (i = 0; i < (int)program_count; i++) {
		int		j;
		ia_css_program_manifest_t	*program_manifest_i = ia_css_program_group_manifest_get_program_manifest(manifest, i);
		ia_css_program_type_t		program_type_i = ia_css_program_manifest_get_type(program_manifest_i);
		ia_css_kernel_bitmap_t		program_bitmap_i = ia_css_program_manifest_get_kernel_bitmap(program_manifest_i);
		uint8_t						program_dependency_count_i = ia_css_program_manifest_get_program_dependency_count(program_manifest_i);
		uint8_t						terminal_dependency_count_i = ia_css_program_manifest_get_terminal_dependency_count(program_manifest_i);
		uint8_t						program_dependency_i0 = ia_css_program_manifest_get_program_dependency(program_manifest_i, 0);
		bool						is_sub_i = ia_css_is_program_manifest_subnode_program_type(program_manifest_i);
		bool						is_exclusive_sub_i = (program_type_i == IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB);
		bool						is_super_i = ia_css_is_program_manifest_supernode_program_type(program_manifest_i);

/* A program must have kernels that are a subset of the total */
		verifexit(!ia_css_is_kernel_bitmap_empty(program_bitmap_i), EINVAL);
		verifexit(ia_css_is_kernel_bitmap_subset(total_bitmap, program_bitmap_i), EINVAL);
		verifexit((program_type_i != IA_CSS_N_PROGRAM_TYPES), EINVAL);
		verifexit((program_dependency_count_i + terminal_dependency_count_i) != 0, EINVAL);
/*
 * Checks for subnodes
 * - Parallel subnodes cannot depend on terminals
 * - Exclusive subnodes must depend on fewer terminals than the supernode
 * - Subnodes only depend on a supernode of the same type
 * - Must have a subset of the supernode's kernels (but not equal)
 * - This tests only positive cases
 * Checks for singular or supernodes
 * - Cannot depend on exclusive subnodes
 * - No intersection between kernels (too strict for multiple instances ?)
 */
		if (is_sub_i) {
/* Subnode */
			ia_css_program_manifest_t	*program_manifest_k = ia_css_program_group_manifest_get_program_manifest(manifest, program_dependency_i0);
			ia_css_program_type_t		program_type_k = ia_css_program_manifest_get_type(program_manifest_k);
			ia_css_kernel_bitmap_t		program_bitmap_k = ia_css_program_manifest_get_kernel_bitmap(program_manifest_k);
			verifexit(program_dependency_count_i == 1, EINVAL);
			if (is_exclusive_sub_i) {
				verifexit(terminal_dependency_count_i <= ia_css_program_manifest_get_terminal_dependency_count(program_manifest_k), EINVAL);
			} else{
				verifexit(terminal_dependency_count_i == 0, EINVAL);
			}
			verifexit(program_type_k == (is_exclusive_sub_i?IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER:IA_CSS_PROGRAM_TYPE_PARALLEL_SUPER), EINVAL);
			verifexit(!ia_css_is_kernel_bitmap_equal(program_bitmap_k, program_bitmap_i), EINVAL);
			verifexit(ia_css_is_kernel_bitmap_subset(program_bitmap_k, program_bitmap_i), EINVAL);
		} else {
/* Singular or Supernode */
			int	k;
			for (k = 0; k < program_dependency_count_i; k++) {
				uint8_t	program_dependency_k = ia_css_program_manifest_get_program_dependency(program_manifest_i, k);
				ia_css_program_manifest_t	*program_manifest_k = ia_css_program_group_manifest_get_program_manifest(manifest, (int)program_dependency_k);
				ia_css_program_type_t		program_type_k	= ia_css_program_manifest_get_type(program_manifest_k);
				ia_css_kernel_bitmap_t		program_bitmap_k = ia_css_program_manifest_get_kernel_bitmap(program_manifest_k);
				verifexit(program_dependency_k < program_count, EINVAL);
				verifexit(program_type_k != IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB, EINVAL);
				verifexit(ia_css_is_kernel_bitmap_intersection_empty(program_bitmap_i, program_bitmap_k), EINVAL);
			}
		}

/* Check for relations */
		for (j = 0; j < (int)program_count; j++) {
			int	k;
			ia_css_program_manifest_t	*program_manifest_j = ia_css_program_group_manifest_get_program_manifest(manifest, j);
			ia_css_program_type_t		program_type_j	= ia_css_program_manifest_get_type(program_manifest_j);
			ia_css_kernel_bitmap_t		program_bitmap_j = ia_css_program_manifest_get_kernel_bitmap(program_manifest_j);
			uint8_t						program_dependency_count_j = ia_css_program_manifest_get_program_dependency_count(program_manifest_j);
			uint8_t						program_dependency_j0 = ia_css_program_manifest_get_program_dependency(program_manifest_j, 0);
			bool						is_sub_j = ia_css_is_program_manifest_subnode_program_type(program_manifest_j);
			bool						is_super_j = ia_css_is_program_manifest_supernode_program_type(program_manifest_j);

			bool						is_j_subset_i = ia_css_is_kernel_bitmap_subset(program_bitmap_i, program_bitmap_j);
			bool						is_i_subset_j = ia_css_is_kernel_bitmap_subset(program_bitmap_j, program_bitmap_i);

/* Test below would fail for i==j */
			if (i == j) {
				continue;
			}

/* Empty sets are always subsets, but meaningless */
		verifexit(!ia_css_is_kernel_bitmap_empty(program_bitmap_j), EINVAL);

/*
 * Checks for mutual subnodes
 * - Parallel subnodes must have an equal set of kernels
 * - Exclusive subnodes must have an unequal set of kernels
 * Checks for subnodes
 * - Subnodes must have a subset of kernels
 */
			if (((program_type_i == IA_CSS_PROGRAM_TYPE_PARALLEL_SUB) && (program_type_j == IA_CSS_PROGRAM_TYPE_PARALLEL_SUB)) ||
				((program_type_i == IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB) && (program_type_j == IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB))) {

				verifexit(program_dependency_count_j == 1, EINVAL);
				verifexit(program_dependency_i0 != i, EINVAL);
				verifexit(program_dependency_j0 != i, EINVAL);

				if (program_dependency_i0 == program_dependency_j0) {
					verifexit(is_sub_i, EINVAL);
/* Subnodes are subsets */
					verifexit((is_j_subset_i || is_i_subset_j), EINVAL);
/* That must be equal for parallel subnodes, must be unequal for exlusive subnodes */
					verifexit(((is_j_subset_i && is_i_subset_j) ^ is_exclusive_sub_i), EINVAL);
				}
				if (is_j_subset_i || is_i_subset_j) {
					verifexit(program_dependency_i0 == program_dependency_j0, EINVAL);
				}
			}

			if (((program_type_i == IA_CSS_PROGRAM_TYPE_PARALLEL_SUPER) && (program_type_j == IA_CSS_PROGRAM_TYPE_PARALLEL_SUB)) ||
				((program_type_i == IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER) && (program_type_j == IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB))) {

				verifexit(program_dependency_count_j == 1, EINVAL);
				verifexit(!is_i_subset_j, EINVAL);

				if (program_dependency_j0 == i) {
					verifexit(program_dependency_i0 != program_dependency_j0, EINVAL);
					verifexit(is_super_i, EINVAL);
					verifexit(is_j_subset_i, EINVAL);
				}
				if (is_j_subset_i) {
					verifexit(program_dependency_j0 == i, EINVAL);
				}
			}

/*
 * Checks for dependent nodes
 * - Cannot depend on exclusive subnodes
 * - No intersection between kernels (too strict for multiple instances ?) unless a subnode
 */
			for (k = 0; k < (int)program_dependency_count_j; k++) {
				uint8_t	program_dependency_k = ia_css_program_manifest_get_program_dependency(program_manifest_j, k);
				if (program_dependency_k == i) {
/* program[j] depends on program[i] */
					verifexit((i != j), EINVAL);
					verifexit(program_type_i != IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB, EINVAL);
					verifexit(ia_css_is_kernel_bitmap_intersection_empty(program_bitmap_i, program_bitmap_j) ^ is_sub_j, EINVAL);
				}
			}

/*
 * Checks for supernodes and subnodes
 * - Detect nodes that kernel-wise are subsets, but not connected to the correct supernode
 * - We do not (yet) detect if programs properly depend on all parallel nodes
 */
			if (!ia_css_is_kernel_bitmap_intersection_empty(program_bitmap_i, program_bitmap_j)) {
/* This test will pass if the program manifest is NULL, but that's no concern here */
				verifexit(!ia_css_is_program_manifest_singular_program_type(program_manifest_i), EINVAL);
				verifexit(!ia_css_is_program_manifest_singular_program_type(program_manifest_j), EINVAL);
				verifexit((is_j_subset_i || is_i_subset_j), EINVAL);

				if (is_super_i) {
					verifexit(is_sub_j, EINVAL);
					verifexit(program_dependency_j0 == i, EINVAL);
				}
				if (is_super_j) {
					verifexit(is_sub_i, EINVAL);
					verifexit(program_dependency_i0 == j, EINVAL);
				}
			}
		}
		check_bitmap = ia_css_kernel_bitmap_union(check_bitmap, program_bitmap_i);
/* A terminal can be bound to only a single (of multiple concurrent) program(s), i.e. the one that holds the iterator to control it */
/* Only singular and super nodes can depend on a terminal. This loop accumulates all terminal dependencies over all programs */
		for (j = 0; j < (int)terminal_dependency_count_i; j++) {
			uint8_t	terminal_dependency = ia_css_program_manifest_get_terminal_dependency(program_manifest_i, j);
			verifexit(terminal_dependency < terminal_count, EINVAL);
			if (program_type_i != IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB) {
/* If the subnode always came after the supernode we could check for presence */
				terminal_bitmap = vied_nci_bit_mask_set_unique(terminal_bitmap, terminal_dependency);
				verifexit(!vied_nci_is_bitmap_empty(terminal_bitmap), EINVAL);
			}
		}
	}
	verifexit(ia_css_is_kernel_bitmap_equal(total_bitmap, check_bitmap), EINVAL);

	terminal_bitmap_weight = vied_nci_bitmap_compute_weight(terminal_bitmap);
	verifexit(terminal_bitmap_weight >= 0, EINVAL);
	if (has_parameter_terminal) {
		verifexit((terminal_bitmap_weight == (terminal_count - 1)), EINVAL);
	} else {
		verifexit((terminal_bitmap_weight == terminal_count), EINVAL);
	}

	is_valid = true;
EXIT:
	return is_valid;
}

size_t ia_css_program_group_manifest_get_size(
	const ia_css_program_group_manifest_t	*manifest)
{
	size_t	size = 0;

	if (manifest != NULL) {
		size = manifest->size;
	}

	return size;
}

ia_css_program_group_ID_t ia_css_program_group_manifest_get_program_group_ID(
	const ia_css_program_group_manifest_t	*manifest)
{
	ia_css_program_group_ID_t	id = 0;

	if (manifest != NULL) {
		id = manifest->ID;
	}

	return id;
}

int ia_css_program_group_manifest_set_program_group_ID(
	ia_css_program_group_manifest_t *manifest,
	ia_css_program_group_ID_t id)
{
	int	retval = -1;

	if (manifest != NULL) {
		manifest->ID = id;
		retval = 0;
	}

	return retval;
}

int ia_css_program_group_manifest_set_alignment(
	ia_css_program_group_manifest_t	*manifest,
	const uint8_t alignment)
{
	int	retval = -1;

	if (manifest != NULL) {
		manifest->alignment = alignment;
		retval = 0;
	}

	return retval;
}

uint8_t ia_css_program_group_manifest_get_alignment(
	const ia_css_program_group_manifest_t	*manifest)
{
	uint8_t	alignment = 0;

	if (manifest != NULL) {
		alignment = manifest->alignment;
	}

	return alignment;
}

int ia_css_program_group_manifest_set_kernel_bitmap(
	ia_css_program_group_manifest_t	*manifest,
	const ia_css_kernel_bitmap_t	bitmap)
{
	int retval = -1;

	if (manifest != NULL) {
		manifest->kernel_bitmap = bitmap;
		retval = 0;
	}

	return retval;
}

ia_css_kernel_bitmap_t ia_css_program_group_manifest_get_kernel_bitmap(
	const ia_css_program_group_manifest_t	*manifest)
{
	ia_css_kernel_bitmap_t	bitmap = 0;

	if (manifest != NULL) {
		bitmap = manifest->kernel_bitmap;
	}

	return bitmap;
}

ia_css_program_manifest_t *ia_css_program_group_manifest_get_program_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int						program_index)
{
	ia_css_program_manifest_t	*program_manifest = NULL;
	ia_css_program_manifest_t	*prg_manifest_base;
	uint8_t			program_count = ia_css_program_group_manifest_get_program_count(manifest);

	verifexit(manifest != NULL, EINVAL);
	verifexit(program_index < program_count, EINVAL);

	prg_manifest_base = (ia_css_program_manifest_t *)((char *)manifest +
		manifest->program_manifest_offset);
	program_manifest = &prg_manifest_base[program_index];
EXIT:
	return program_manifest;
}

ia_css_data_terminal_manifest_t *ia_css_program_group_manifest_get_data_terminal_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int						terminal_index)
{
	ia_css_data_terminal_manifest_t	*data_terminal_manifest = NULL;
	ia_css_terminal_manifest_t		*terminal_manifest = ia_css_program_group_manifest_get_terminal_manifest(manifest, terminal_index);
	verifexit(ia_css_is_terminal_manifest_data_terminal(terminal_manifest), EINVAL);
	data_terminal_manifest = (ia_css_data_terminal_manifest_t *)terminal_manifest;
EXIT:
	return data_terminal_manifest;
}

ia_css_param_terminal_manifest_t *ia_css_program_group_manifest_get_param_terminal_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int		terminal_index)
{
	ia_css_param_terminal_manifest_t	*param_terminal_manifest = NULL;
	ia_css_terminal_manifest_t		*terminal_manifest = ia_css_program_group_manifest_get_terminal_manifest(manifest, terminal_index);

	verifexit(ia_css_is_terminal_manifest_parameter_terminal(terminal_manifest), EINVAL);
	param_terminal_manifest = (ia_css_param_terminal_manifest_t *)terminal_manifest;
EXIT:
	return param_terminal_manifest;
}

ia_css_terminal_manifest_t *ia_css_program_group_manifest_get_terminal_manifest(
	const ia_css_program_group_manifest_t	*manifest,
	const unsigned int		terminal_index)
{
	ia_css_terminal_manifest_t *terminal_manifest = NULL;
	ia_css_terminal_manifest_t *terminal_manifest_base;
	uint8_t	terminal_count = ia_css_program_group_manifest_get_terminal_count(manifest);
	uint8_t i = 0;
	uint32_t offset;

	verifexit(manifest != NULL, EINVAL);
	verifexit(terminal_index < terminal_count, EINVAL);

	terminal_manifest_base = (ia_css_terminal_manifest_t *)((char *)manifest +
		manifest->terminal_manifest_offset);
	terminal_manifest = terminal_manifest_base;
	while (i < terminal_index) {
		offset = ia_css_terminal_manifest_get_size(terminal_manifest);
		terminal_manifest = (ia_css_terminal_manifest_t *)((char *)terminal_manifest + offset);
		i++;
	}
EXIT:
	return terminal_manifest;
}

uint8_t ia_css_program_group_manifest_get_program_count(
	const ia_css_program_group_manifest_t	*manifest)
{
	uint8_t	program_count = 0;
	if (manifest != NULL) {
		program_count = manifest->program_count;
	}

	return program_count;
}

uint8_t ia_css_program_group_manifest_get_terminal_count(
	const ia_css_program_group_manifest_t	*manifest)
{
	uint8_t	terminal_count = 0;
	if (manifest != NULL) {
		terminal_count = manifest->terminal_count;
	}

	return terminal_count;
}

void ia_css_program_group_manifest_init(
	ia_css_program_group_manifest_t *blob,
	const uint8_t	program_count,
	const uint8_t	terminal_count,
	const uint8_t   *program_dependencies,
	const uint8_t   *terminal_dependencies,
	const ia_css_terminal_type_t	*terminal_type,
	const uint16_t	section_count)
{
	int i;
	uint32_t offset =0;
	char *prg_manifest_base, *terminal_manifest_base;
	size_t program_size = ia_css_sizeof_program_manifest(program_count, terminal_count);
	/*
	 * assert(blob != NULL);
	 */
	COMPILATION_ERROR_IF(
		SIZE_OF_TERMINAL_MANIFEST_STRUCT_IN_BITS != (CHAR_BIT * sizeof(ia_css_terminal_manifest_t)));
	COMPILATION_ERROR_IF(
		SIZE_OF_DATA_TERMINAL_MANIFEST_STRUCT_IN_BITS != (CHAR_BIT * sizeof(ia_css_data_terminal_manifest_t)));
	COMPILATION_ERROR_IF(
		SIZE_OF_PROGRAM_GROUP_MANIFEST_STRUCT_IN_BITS != (CHAR_BIT * sizeof(ia_css_program_group_manifest_t)));
	COMPILATION_ERROR_IF(
		SIZE_OF_PROGRAM_MANIFEST_STRUCT_IN_BITS != (CHAR_BIT * sizeof(ia_css_program_manifest_t)));
	COMPILATION_ERROR_IF(
		SIZE_OF_PARAM_TERMINAL_MANIFEST_STRUCT_IN_BITS != (CHAR_BIT * sizeof(ia_css_param_terminal_manifest_t)));
	/* A program group ID cannot be zero */
	blob->ID = 1;
	blob->program_count = program_count;
	blob->terminal_count = terminal_count;
	blob->program_manifest_offset = sizeof(ia_css_program_group_manifest_t);
	blob->terminal_manifest_offset = blob->program_manifest_offset +
		program_size * program_count;

	prg_manifest_base = (char *)
		(((char*)blob) + blob->program_manifest_offset);
	offset = blob->program_manifest_offset;
	for (i = 0; i < (int)program_count; i++) {
		ia_css_program_manifest_init(
			(ia_css_program_manifest_t *)prg_manifest_base,
			program_dependencies[i], terminal_dependencies[i]);
		ia_css_program_manifest_set_parent_offset(
			(ia_css_program_manifest_t *)prg_manifest_base,
			offset);
		prg_manifest_base += program_size;
		offset += program_size;
	}

	offset = blob->terminal_manifest_offset;
	terminal_manifest_base = (char *) (((char*)blob) + offset);
	for (i = 0; i < (int)terminal_count; i++) {
		size_t terminal_size;
		ia_css_terminal_manifest_init(
			(ia_css_terminal_manifest_t *)terminal_manifest_base,
			terminal_type[i],
			section_count);
		ia_css_terminal_manifest_set_parent_offset(
				(ia_css_terminal_manifest_t *)terminal_manifest_base,
				offset);
		terminal_size = ia_css_sizeof_terminal_manifest(terminal_type[i], section_count);
		terminal_manifest_base += terminal_size;
		offset += terminal_size;
	}
	blob->size = offset;
}
