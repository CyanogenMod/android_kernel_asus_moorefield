
#include <ia_css_psys_program_group_manifest.h>

#include <ia_css_psys_program_manifest.h>
#include <ia_css_psys_terminal_manifest.h>

#include <ia_css_program_group_param.h>
#include <ia_css_kernel_bitmap.h>
#include "ia_css_psys_program_group_private.h"
#include <type_support.h>
#include <error_support.h>
#include <print_support.h>
#include <vied_nci_psys_system_global.h>	/* Safer bit mask functions */
#include <cpu_mem_support.h>


ia_css_program_group_manifest_t *ia_css_program_group_manifest_alloc(
	const uint8_t							program_count,
	const uint8_t							terminal_count,
	const uint8_t							*program_dependency_count,
	const uint8_t							*terminal_dependency_count,
	const ia_css_terminal_type_t			*terminal_type)
{
	size_t	size = 0;
	int		i;
	ia_css_program_group_manifest_t *manifest = NULL;

	errno = 0;

/* A program group cannot be empty */
	verifexit(program_count != 0, EINVAL);
	verifexit(terminal_count != 0, EINVAL);
	manifest = (ia_css_program_group_manifest_t *)ia_css_cpu_mem_alloc(sizeof(ia_css_program_group_manifest_t));
	verifexit(manifest != NULL, ENOBUFS);
	size += sizeof(ia_css_program_group_manifest_t);
	ia_css_cpu_mem_set_zero(manifest, 0, sizeof(ia_css_program_group_manifest_t));

	manifest->program_count = program_count;
	manifest->terminal_count = terminal_count;

	manifest->program_manifest = (ia_css_program_manifest_t **)ia_css_cpu_mem_alloc(program_count * sizeof(ia_css_program_manifest_t *));
	verifexit(manifest->program_manifest != NULL, ENOBUFS);
	size += program_count * sizeof(ia_css_program_manifest_t *);
	ia_css_cpu_mem_set_zero(manifest->program_manifest, 0, program_count * sizeof(ia_css_program_manifest_t *));

	for (i = 0; i < (int)program_count; i++) {
		manifest->program_manifest[i] = ia_css_program_manifest_alloc(program_dependency_count[i], terminal_dependency_count[i]);
		verifexit(manifest->program_manifest[i] != NULL, ENOBUFS);
		size += ia_css_program_manifest_get_size(manifest->program_manifest[i]);
		ia_css_program_manifest_set_parent(manifest->program_manifest[i], manifest);
	}

	manifest->terminal_manifest = (ia_css_terminal_manifest_t **)ia_css_cpu_mem_alloc(terminal_count * sizeof(ia_css_terminal_manifest_t *));
	verifexit(manifest->terminal_manifest != NULL, ENOBUFS);
	size += terminal_count * sizeof(ia_css_terminal_manifest_t *);
	ia_css_cpu_mem_set_zero(manifest->terminal_manifest, 0, terminal_count * sizeof(ia_css_terminal_manifest_t *));

	for (i = 0; i < (int)terminal_count; i++) {
		manifest->terminal_manifest[i] = ia_css_terminal_manifest_alloc(terminal_type[i]);
		verifexit(manifest->terminal_manifest[i] != NULL, ENOBUFS);
		size += ia_css_terminal_manifest_get_size(manifest->terminal_manifest[i]);
		ia_css_terminal_manifest_set_parent(manifest->terminal_manifest[i], manifest);
	}

	manifest->size = ia_css_sizeof_program_group_manifest(program_count, terminal_count, program_dependency_count, terminal_dependency_count, terminal_type);
	verifexit(manifest->size == size, EINVAL);

EXIT:

	if (errno != 0) {
		manifest = ia_css_program_group_manifest_free(manifest);
	}

	return manifest;
}

ia_css_program_group_manifest_t *ia_css_program_group_manifest_free(
	ia_css_program_group_manifest_t			*manifest)
{
	int		i;

	if (manifest != NULL) {
		if (manifest->terminal_manifest != NULL) {
			uint8_t	terminal_count = ia_css_program_group_manifest_get_terminal_count(manifest);

			for (i = (int)terminal_count - 1; i >= 0; i--) {
				manifest->terminal_manifest[i] = ia_css_terminal_manifest_free(manifest->terminal_manifest[i]);
			}
			ia_css_cpu_mem_free((void *)manifest->terminal_manifest);
			manifest->terminal_manifest = NULL;
		}

		if (manifest->program_manifest != NULL) {
			uint8_t	program_count = ia_css_program_group_manifest_get_program_count(manifest);

			for (i = (int)program_count - 1; i >= 0; i--) {
				manifest->program_manifest[i] = ia_css_program_manifest_free(manifest->program_manifest[i]);
			}
			ia_css_cpu_mem_free((void *)manifest->program_manifest);
			manifest->program_manifest = NULL;
		}
		ia_css_cpu_mem_free((void *)manifest);
		manifest = NULL;
	}

	return manifest;
}

ia_css_program_group_manifest_t *ia_css_program_group_manifest_create(
	const unsigned int						specification)
{
	int			i;
	int			retval;
	ia_css_program_group_manifest_t		*manifest = NULL;
	ia_css_program_manifest_t			*program_manifest = NULL;
	ia_css_data_terminal_manifest_t		*dterminal_manifest = NULL;
	ia_css_kernel_bitmap_t				bitmap;
	uint8_t		program_count;
	uint8_t		terminal_count;
	uint8_t		kernel_count;
	uint8_t		subgraph_count;

	errno = 0;

	switch (specification) {
	case 0: {
		uint8_t	program_dependency_count[10] = {0, 0, 1, 1, 1, 1, 1, 1, 3, 1};
		uint8_t	terminal_dependency_count[10] = {1, 1, 0, 0, 2, 1, 0, 0, 2, 1};
		ia_css_terminal_type_t	terminal_type[7] = {
			IA_CSS_TERMINAL_TYPE_DATA_IN,
			IA_CSS_TERMINAL_TYPE_DATA_IN,
			IA_CSS_TERMINAL_TYPE_PARAM_STREAM,
			IA_CSS_TERMINAL_TYPE_DATA_IN,
			IA_CSS_TERMINAL_TYPE_PARAM_STREAM,
			IA_CSS_TERMINAL_TYPE_DATA_OUT,
			IA_CSS_TERMINAL_TYPE_DATA_OUT};
		program_count = 10;
		terminal_count = 7;
		kernel_count = 17;
		subgraph_count = 1;

		manifest = ia_css_program_group_manifest_alloc(program_count, terminal_count, &program_dependency_count[0], &terminal_dependency_count[0], &terminal_type[0]);
		verifexit(manifest != NULL, EINVAL);
/* A program group ID cannot be zero */
		manifest->ID = 1;
		manifest->alignment = 32;
		manifest->subgraph_count = subgraph_count;

		bitmap = ia_css_kernel_bitmap_clear();
		for (i = 0; i < (int)kernel_count; i++) {
			bitmap = ia_css_kernel_bitmap_set(bitmap, i);
		}
		manifest->kernel_bitmap = bitmap;

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, 0);
		verifexit(dterminal_manifest != NULL, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 0) == 0, EINVAL);

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, 1);
		verifexit(dterminal_manifest != NULL, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 4) == 0, EINVAL);

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, 2);
		verifexit(dterminal_manifest != NULL, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 8) == 0, EINVAL);

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, 3);
		verifexit(dterminal_manifest != NULL, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 9) == 0, EINVAL);

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, 4);
		verifexit(dterminal_manifest != NULL, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 13) == 0, EINVAL);

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, 5);
		verifexit(dterminal_manifest != NULL, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 14) == 0, EINVAL);

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, 6);
		verifexit(dterminal_manifest != NULL, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 16) == 0, EINVAL);

		dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(manifest, terminal_count);
		verifexit(dterminal_manifest == NULL, EINVAL);
/* We caused an error */
		errno = 0;
		bitmap = ia_css_kernel_bitmap_clear();

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 0);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_SINGULAR) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 0);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 1);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 2);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 3);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		retval = ia_css_program_manifest_set_program_dependency(program_manifest, 0, 0);
		verifexit(retval < 0, EINVAL);
/* We caused an error */
		errno = 0;
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 0, 0) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 1);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_PARALLEL_SUPER) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 4);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 5);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 6);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 7);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 1, 0) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 2);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_PARALLEL_SUB) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 5);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 6);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 1, 0) == 0, EINVAL);
		retval = ia_css_program_manifest_set_terminal_dependency(program_manifest, 0, 0);
		verifexit(retval < 0, EINVAL);
/* We caused an error */
		errno = 0;

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 3);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_PARALLEL_SUB) == 0, EINVAL);
		verifexit(program_manifest != NULL, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 5);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 6);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 1, 0) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 4);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 8);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 9);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 0, 0) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 2, 0) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 3, 1) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 5);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 9);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 4, 0) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 3, 0) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 6);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 10);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 11);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 12);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 4, 0) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 7);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 10);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 11);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 6, 0) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 8);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_SINGULAR) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 13);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 14);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 1, 0) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 2, 1) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 3, 2) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 4, 0) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 5, 1) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, 9);
		verifexit(program_manifest != NULL, EINVAL);
		verifexit(ia_css_program_manifest_set_type(program_manifest, IA_CSS_PROGRAM_TYPE_SINGULAR) == 0, EINVAL);
		bitmap = ia_css_kernel_bitmap_clear();
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 15);
		bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 16);
		verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
		ia_css_program_manifest_set_kernel_bitmap(program_manifest, bitmap);
		verifexit(ia_css_program_manifest_set_program_dependency(program_manifest, 6, 0) == 0, EINVAL);
		verifexit(ia_css_program_manifest_set_terminal_dependency(program_manifest, 6, 0) == 0, EINVAL);

		program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, program_count);
		verifexit(program_manifest == NULL, EINVAL);
/* We caused an error */
		errno = 0;
		bitmap = ia_css_kernel_bitmap_clear();

		} break;
	case 1:
	case 2:
	case 3: {
		uint8_t	*program_dependency_count = NULL;
		uint8_t	*terminal_dependency_count = NULL;
		ia_css_terminal_type_t	*terminal_type = NULL;
		program_count = 0;
		terminal_count = 0;
		kernel_count = 0;
		subgraph_count = 0;

		manifest = ia_css_program_group_manifest_alloc(program_count, terminal_count, program_dependency_count, terminal_dependency_count, terminal_type);
		verifexit(manifest != NULL, EINVAL);
/* A program group ID cannot be zero */
		manifest->ID = 1;
		manifest->alignment = 32;
		manifest->subgraph_count = subgraph_count;

		bitmap = ia_css_kernel_bitmap_clear();
		for (i = 0; i < (int)kernel_count; i++) {
			bitmap = ia_css_kernel_bitmap_set(bitmap, i);
		}
		manifest->kernel_bitmap = bitmap;

			} break;
	default:
		verifexit(false, EINVAL);
		break;
	}

EXIT:
	if (errno != 0) {
		manifest = ia_css_program_group_manifest_free(manifest);
	}

	return manifest;
}

ia_css_program_group_manifest_t *ia_css_program_group_manifest_destroy(
	ia_css_program_group_manifest_t			*manifest)
{
	return ia_css_program_group_manifest_free(manifest);
}

ia_css_program_group_manifest_t	*ia_css_program_group_manifest_read(
	void									*fid)
{
	ia_css_program_group_manifest_t	*manifest = NULL;
	fpos_t	fpos;
	size_t	size = 0;

	verifexit(fgetpos((FILE *)fid, &fpos) == 0, EINVAL);
	verifexit(fread((void *)size, sizeof(size_t), 1, (FILE *)fid) == 1, EINVAL);
	verifexit(fsetpos((FILE *)fid, &fpos) == 0, EINVAL);

	manifest = (ia_css_program_group_manifest_t *)ia_css_cpu_mem_alloc(size);
/* Obviously this will only work if we have serialised the object */
	verifexit(fread((void *)manifest, size, 1, (FILE *)fid) == 1, EINVAL);

EXIT:
	if (errno != 0) {
		manifest = ia_css_program_group_manifest_free(manifest);
	}
	return manifest;
}

int ia_css_program_group_manifest_write(
	const ia_css_program_group_manifest_t	*manifest,
	void									*fid)
{
	int	retval = -1;
	size_t	size = ia_css_program_group_manifest_get_size(manifest);

	verifexit(manifest != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

/* This will only work but is meaningless until we have serialised the object */
	verifexit(fwrite((const void *)manifest, size, 1, (FILE *)fid) == size, EINVAL);

	retval = 0;
EXIT:
	return retval;
}

int ia_css_program_group_manifest_print(
	const ia_css_program_group_manifest_t	*manifest,
	void									*fid)
{
	int	retval = -1;
	FILE	*f = (FILE *)fid;
	int		i;
	uint8_t	program_count, terminal_count;
	ia_css_kernel_bitmap_t	bitmap;

	verifexit(manifest != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(f,"ia_css_program_group_manifest_print\n");
	fprintf(f,"sizeof(manifest) = %d\n",(int)ia_css_program_group_manifest_get_size(manifest));
	fprintf(f,"alignment(manifest) = %d\n",(int)ia_css_program_group_manifest_get_alignment(manifest));

	fprintf(f,"program group ID = %d\n",(int)ia_css_program_group_manifest_get_program_group_ID(manifest));
/*	fprintf(f,"program group ID = %s\n",ia_css_program_group_ID_string(ia_css_program_group_manifest_get_program_group_ID(manifest))); */

	program_count = ia_css_program_group_manifest_get_program_count(manifest);
	terminal_count = ia_css_program_group_manifest_get_terminal_count(manifest);

	bitmap = ia_css_program_group_manifest_get_kernel_bitmap(manifest);
	verifexit(ia_css_kernel_bitmap_print(bitmap, fid) == 0, EINVAL);

	fprintf(f,"%d program manifests\n",(int)program_count);
	for (i = 0; i < (int)program_count; i++) {
		ia_css_program_manifest_t	*program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, i);
		retval = ia_css_program_manifest_print(program_manifest, fid);
		verifjmpexit(retval == 0);
	}
	fprintf(f,"%d terminal manifests\n",(int)terminal_count);
	for (i = 0; i < (int)terminal_count; i++) {
		ia_css_terminal_manifest_t	*terminal_manifest = ia_css_program_group_manifest_get_terminal_manifest(manifest, i);
		retval = ia_css_terminal_manifest_print(terminal_manifest, fid);
		verifjmpexit(retval == 0);
	}

	retval = 0;
EXIT:
	return retval;
}
