
#include <ia_css_psys_program_manifest.h>
#include <ia_css_psys_program_group_manifest.h>

#include <ia_css_program_group_param_types.h>	/* Kernel enable bitmap */

#include <vied_nci_psys_system_global.h>
#include "ia_css_psys_program_group_private.h"

#include <type_support.h>
#include <error_support.h>
#include <print_support.h>
/* #include <string_support.h>		memset() */
/* #include <string.h>		*/
#include <cpu_mem_support.h>

ia_css_program_manifest_t *ia_css_program_manifest_alloc(
	const uint8_t							program_dependency_count,
	const uint8_t							terminal_dependency_count)
{
	size_t	size = 0;
	ia_css_program_manifest_t *manifest = NULL;

	errno = 0;

	manifest = (ia_css_program_manifest_t *)ia_css_cpu_mem_alloc(sizeof(ia_css_program_manifest_t));
	verifexit(manifest != NULL, ENOBUFS);
	size += sizeof(ia_css_program_manifest_t);
	ia_css_cpu_mem_set_zero((void *)manifest, sizeof(ia_css_program_manifest_t));

/* A program ID cannot be zero */
	manifest->ID = 1;
/* A program requires at least one input or output */
    verifexit((program_dependency_count + terminal_dependency_count) != 0, EINVAL);

	manifest->program_dependency_count = program_dependency_count;
	manifest->terminal_dependency_count = terminal_dependency_count;

    if (program_dependency_count != 0) {
		manifest->program_dependencies = (uint8_t *)ia_css_cpu_mem_alloc(program_dependency_count * sizeof(uint8_t));
		verifexit(manifest->program_dependencies != NULL, ENOBUFS);
		size += program_dependency_count * sizeof(uint8_t);
		ia_css_cpu_mem_set_zero((void *)manifest->program_dependencies, program_dependency_count * sizeof(uint8_t));
	}

    if (terminal_dependency_count != 0) {
		manifest->terminal_dependencies = (uint8_t *)ia_css_cpu_mem_alloc(terminal_dependency_count * sizeof(uint8_t));
		verifexit(manifest->terminal_dependencies != NULL, ENOBUFS);
		size += terminal_dependency_count * sizeof(uint8_t);
		ia_css_cpu_mem_set_zero((void *)manifest->terminal_dependencies, terminal_dependency_count * sizeof(uint8_t));
	}

	verifexit(ia_css_program_manifest_set_type(manifest, IA_CSS_N_PROGRAM_TYPES) == 0, EINVAL);
	manifest->cell_id = VIED_NCI_N_CELL_ID;
	manifest->cell_type_id = VIED_NCI_N_CELL_TYPE_ID;

	manifest->size = ia_css_sizeof_program_manifest(program_dependency_count, terminal_dependency_count);
	verifexit(manifest->size == size, EINVAL);

EXIT:

	if (errno != 0) {
		manifest = ia_css_program_manifest_free(manifest);
	}

	return manifest;
}

ia_css_program_manifest_t *ia_css_program_manifest_free(
	ia_css_program_manifest_t				*manifest)
{
	if (manifest != NULL) {
		ia_css_cpu_mem_free((void *)manifest->terminal_dependencies);
		manifest->terminal_dependencies = NULL;
		ia_css_cpu_mem_free((void *)manifest->program_dependencies);
		manifest->program_dependencies = NULL;

		ia_css_cpu_mem_free((void *)manifest);
		manifest = NULL;
	}
	return manifest;
}

int ia_css_program_manifest_print(
	const ia_css_program_manifest_t			*manifest,
	void									*fid)
{
	int			retval = -1;
	FILE		*f = (FILE *)fid;
	int			i, mem_index, dev_chn_index;

	vied_nci_cell_type_ID_t	cell_type_id;
	uint8_t					program_dependency_count;
	uint8_t					terminal_dependency_count;
	ia_css_kernel_bitmap_t	bitmap;

	verifexit(manifest != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(f,"ia_css_program_manifest_print\n");
	fprintf(f,"sizeof(manifest) = %d\n",(int)ia_css_program_manifest_get_size(manifest));
	fprintf(f,"program ID = %d\n",(int)ia_css_program_manifest_get_program_ID(manifest));
/*	fprintf(f,"program ID = %s\n",ia_css_program_ID_string(ia_css_program_manifest_get_program_ID(manifest))); */

	bitmap = ia_css_program_manifest_get_kernel_bitmap(manifest);
	verifexit(ia_css_kernel_bitmap_print(bitmap, fid) == 0, EINVAL);

	if (ia_css_has_program_manifest_fixed_cell(manifest)) {
		vied_nci_cell_ID_t	cell_id = ia_css_program_manifest_get_cell_ID(manifest);
		cell_type_id = vied_nci_cell_get_type(cell_id);
		fprintf(f,"cell(program) = %d\n",(int)cell_id);
/*		fprintf(f,"cell(program) = %s\n",vied_nci_cell_string(cell_id)); */
	} else {
		cell_type_id = ia_css_program_manifest_get_cell_type_ID(manifest);
	}
	fprintf(f,"cell type(program) = %d\n",(int)cell_type_id);
/*	fprintf(f,"cell_type(program) = %s\n",vied_nci_cell_type_string(cell_type_id)); */

	for (mem_index = 0; mem_index < (int)VIED_NCI_N_MEM_TYPE_ID; mem_index++) {
		vied_nci_mem_type_ID_t	mem_type = vied_nci_cell_type_get_mem_type(cell_type_id, mem_index);

		fprintf(f,"\ttype(internal mem) type = %d\n",(int)mem_type);
/*		fprintf(f,"\ttype(internal mem) type = %s\n",vied_nci_mem_type_string(mem_type)); */
		fprintf(f,"\ttype(internal mem) size = %d\n",manifest->int_mem_size[mem_index]);
	}

	for (mem_index = 0; mem_index < (int)VIED_NCI_N_DATA_MEM_TYPE_ID; mem_index++) {
		vied_nci_mem_type_ID_t	mem_type = (vied_nci_mem_type_ID_t)mem_index;

		fprintf(f,"\ttype(external mem) type = %d\n",(int)mem_type);
/*		fprintf(f,"\ttype(external mem) type = %s\n",vied_nci_mem_type_string(mem_type)); */
		fprintf(f,"\ttype(external mem) size = %d\n",manifest->ext_mem_size[mem_index]);
	}

	for (dev_chn_index = 0; dev_chn_index < (int)VIED_NCI_N_DEV_CHN_ID; dev_chn_index++) {
		fprintf(f,"\ttype(device channel) type = %d\n",(int)dev_chn_index);
/*		fprintf(f,"\ttype(device channel) type = %s\n",vied_nci_dev_chn_type_string(dev_chn_index)); */
		fprintf(f,"\ttype(device channel) size = %d\n",manifest->dev_chn_size[dev_chn_index]);
	}

	program_dependency_count = ia_css_program_manifest_get_program_dependency_count(manifest);
	if (program_dependency_count == 0) {
		fprintf(f,"program_dependencies[%d] {};\n",program_dependency_count);
	} else {
		fprintf(f,"program_dependencies[%d] {",program_dependency_count);
		for (i = 0; i < (int)program_dependency_count - 1; i++) {
			fprintf(f,"%4d, ",manifest->program_dependencies[i]);
		}
		fprintf(f,"%4d}\n",manifest->program_dependencies[i]);
	}

	terminal_dependency_count = ia_css_program_manifest_get_terminal_dependency_count(manifest);
	if (terminal_dependency_count == 0) {
		fprintf(f,"terminal_dependencies[%d] {};\n",terminal_dependency_count);
	} else {
		fprintf(f,"terminal_dependencies[%d] {",terminal_dependency_count);
		for (i = 0; i < (int)terminal_dependency_count - 1; i++) {
			fprintf(f,"%4d, ",manifest->terminal_dependencies[i]);
		}
		fprintf(f,"%4d}\n",manifest->terminal_dependencies[i]);
	}

	retval = 0;
EXIT:
	return retval;
}

