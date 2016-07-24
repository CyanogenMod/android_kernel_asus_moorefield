
#include <ia_css_psys_terminal_manifest.h>
#include <ia_css_psys_parameter_manifest.h>		/* Specific to the type cached parameter terminal */

#include <ia_css_program_group_data.h>			/* Data object types on the terminals */
#include <ia_css_program_group_param_types.h>	/* Kernel enable bitmap */

#include <ia_css_psys_sim_data.h>				/* ia_css_psys_ran_val() */
#include "ia_css_psys_program_group_private.h"

#include <type_support.h>
#include <error_support.h>
#include <math_support.h>						/* MIN() */
#include <assert_support.h>


size_t ia_css_sizeof_terminal_manifest(
	const ia_css_terminal_type_t			terminal_type,
	const uint16_t					dimension)
{
	size_t	size = 0;

	if (terminal_type == IA_CSS_TERMINAL_TYPE_PARAM_CACHED) {
		size = sizeof(ia_css_param_terminal_manifest_t);
		size += dimension*sizeof(ia_css_parameter_manifest_t);
	} else if (terminal_type < IA_CSS_N_TERMINAL_TYPES) {
		size = sizeof(ia_css_data_terminal_manifest_t);
	}

	return size;
}


bool ia_css_is_terminal_manifest_parameter_terminal(
	const ia_css_terminal_manifest_t		*manifest)
{
/* will return an error value on error */
	ia_css_terminal_type_t	terminal_type = ia_css_terminal_manifest_get_type(manifest);
	return (terminal_type == IA_CSS_TERMINAL_TYPE_PARAM_CACHED);
}

bool ia_css_is_terminal_manifest_data_terminal(
	const ia_css_terminal_manifest_t		*manifest)
{
/* will return an error value on error */
	ia_css_terminal_type_t	terminal_type = ia_css_terminal_manifest_get_type(manifest);
	return ((terminal_type != IA_CSS_TERMINAL_TYPE_PARAM_CACHED) && (terminal_type < IA_CSS_N_TERMINAL_TYPES));
}



size_t ia_css_terminal_manifest_get_size(
	const ia_css_terminal_manifest_t		*manifest)
{
	size_t	size = 0;

	if (manifest != NULL) {
		size = manifest->size;
	}

	return size;
}

ia_css_terminal_type_t ia_css_terminal_manifest_get_type(
	const ia_css_terminal_manifest_t		*manifest)
{
	ia_css_terminal_type_t	terminal_type = IA_CSS_N_TERMINAL_TYPES;

	if (manifest != NULL) {
		terminal_type = manifest->terminal_type;
	}

	return terminal_type;
}

int ia_css_terminal_manifest_set_type(
	ia_css_terminal_manifest_t				*manifest,
	const ia_css_terminal_type_t			terminal_type)
{
	int	retval = -1;

	if (manifest != NULL) {
		manifest->terminal_type = terminal_type;
		retval = 0;
	}

	return retval;
}


ia_css_program_group_manifest_t *ia_css_terminal_manifest_get_parent(
	const ia_css_terminal_manifest_t		*manifest)
{
	ia_css_program_group_manifest_t	*parent = NULL;
	char *base;

	verifexit(manifest != NULL, EINVAL);
	base = (char*)
		((char*)manifest + manifest->parent_offset);

	parent = (ia_css_program_group_manifest_t * )(base);
EXIT:
	return parent;
}

int ia_css_terminal_manifest_set_parent_offset(
	ia_css_terminal_manifest_t	*manifest,
	int32_t terminal_offset)
{
	int	retval = -1;
	verifexit(manifest != NULL, EINVAL);

	/* parent is at negative offset away from current terminal offset*/
	manifest->parent_offset = -terminal_offset;

	retval = 0;
EXIT:
	return retval;
}

uint8_t ia_css_param_terminal_manifest_get_section_count(
	const ia_css_param_terminal_manifest_t	*manifest)
{
	uint8_t	section_count = 0;

	if (manifest != NULL) {
		section_count = manifest->section_count;
	}

	return section_count;
}

ia_css_parameter_manifest_t *ia_css_param_terminal_manifest_get_parameter_manifest(
	const ia_css_param_terminal_manifest_t	*manifest,
	const unsigned int						section_index)
{
	ia_css_parameter_manifest_t	*parameter_manifest = NULL;
	uint8_t				section_count = ia_css_param_terminal_manifest_get_section_count(manifest);

	char *section_base;

	verifexit(manifest != NULL, EINVAL);
	verifexit(section_index < section_count, EINVAL);
	section_base = ((char*) (char*)manifest + manifest->section_offset +
			section_index * sizeof(ia_css_parameter_manifest_t));

	parameter_manifest = (ia_css_parameter_manifest_t *)(section_base);
EXIT:
	return parameter_manifest;
}

ia_css_frame_format_bitmap_t ia_css_data_terminal_manifest_get_frame_format_bitmap(
	const ia_css_data_terminal_manifest_t	*manifest)
{
	ia_css_frame_format_bitmap_t	bitmap = 0;

	if (manifest != NULL) {
		bitmap = manifest->frame_format_bitmap;
	}

	return bitmap;
}

int ia_css_data_terminal_manifest_set_frame_format_bitmap(
	ia_css_data_terminal_manifest_t	*manifest,
	ia_css_frame_format_bitmap_t bitmap)
{
	int ret = -1;

	if (manifest != NULL) {
		manifest->frame_format_bitmap = bitmap;
		ret = 0;
	}

	return ret;
}

ia_css_connection_bitmap_t ia_css_data_terminal_manifest_get_connection_bitmap(
	const ia_css_data_terminal_manifest_t	*manifest)
{
	ia_css_connection_bitmap_t	connection_bitmap = 0;

	if (manifest != NULL) {
		connection_bitmap = manifest->connection_bitmap;
	}

	return connection_bitmap;
}

ia_css_kernel_bitmap_t ia_css_data_terminal_manifest_get_kernel_bitmap(
	const ia_css_data_terminal_manifest_t	*manifest)
{
	ia_css_kernel_bitmap_t	kernel_bitmap = 0;

	if (manifest != NULL) {
		kernel_bitmap = manifest->kernel_bitmap;
	}

	return kernel_bitmap;
}

int ia_css_data_terminal_manifest_set_kernel_bitmap(
	ia_css_data_terminal_manifest_t			*manifest,
	const ia_css_kernel_bitmap_t			kernel_bitmap)
{
	int	retval = -1;

	if (manifest != NULL) {
		manifest->kernel_bitmap = kernel_bitmap;
		retval = 0;
	}

	return retval;
}

int ia_css_data_terminal_manifest_set_kernel_bitmap_unique(
	ia_css_data_terminal_manifest_t			*manifest,
	const unsigned int						index)
{
	int	retval = -1;

	if (manifest != NULL) {
		ia_css_kernel_bitmap_t	kernel_bitmap = ia_css_kernel_bitmap_clear();
		kernel_bitmap = ia_css_kernel_bitmap_set(kernel_bitmap, index);
		verifexit(kernel_bitmap != 0, EINVAL);
		verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap(manifest, kernel_bitmap) == 0, EINVAL);
		retval = 0;
	}

EXIT:
	return retval;
}

void ia_css_data_terminal_manifest_set_min_size(
	ia_css_data_terminal_manifest_t	*manifest,
	const uint16_t			min_size[IA_CSS_N_DATA_DIMENSION])
{
	verifexit(manifest != NULL, EINVAL);

	manifest->min_size[IA_CSS_COL_DIMENSION] = min_size[IA_CSS_COL_DIMENSION];
	manifest->min_size[IA_CSS_ROW_DIMENSION] = min_size[IA_CSS_ROW_DIMENSION];

EXIT:
	return;
}

void ia_css_data_terminal_manifest_set_max_size(
	ia_css_data_terminal_manifest_t	*manifest,
	const uint16_t			max_size[IA_CSS_N_DATA_DIMENSION])
{
	verifexit(manifest != NULL, EINVAL);

	manifest->max_size[IA_CSS_COL_DIMENSION] = max_size[IA_CSS_COL_DIMENSION];
	manifest->max_size[IA_CSS_ROW_DIMENSION] = max_size[IA_CSS_ROW_DIMENSION];

EXIT:
	return;
}

void ia_css_data_terminal_manifest_get_min_size(
	const ia_css_data_terminal_manifest_t	*manifest,
	uint16_t			min_size[IA_CSS_N_DATA_DIMENSION])
{
	verifexit(manifest != NULL, EINVAL);

	min_size[IA_CSS_COL_DIMENSION] = manifest->min_size[IA_CSS_COL_DIMENSION];
	min_size[IA_CSS_ROW_DIMENSION] = manifest->min_size[IA_CSS_ROW_DIMENSION] ;

EXIT:
	return;
}

void ia_css_data_terminal_manifest_get_max_size(
	const ia_css_data_terminal_manifest_t	*manifest,
	uint16_t			max_size[IA_CSS_N_DATA_DIMENSION])
{
	verifexit(manifest != NULL, EINVAL);

	max_size[IA_CSS_COL_DIMENSION] = manifest->max_size[IA_CSS_COL_DIMENSION];
	max_size[IA_CSS_ROW_DIMENSION] = manifest->max_size[IA_CSS_ROW_DIMENSION] ;

EXIT:
	return;
}


/* BXTPOC project specific extensions to the APIS*/
int ia_css_terminal_manifest_init(ia_css_terminal_manifest_t *blob,
	const ia_css_terminal_type_t	terminal_type,
	const uint16_t section_count)
{
	int retval = -1;
	verifexit((blob != NULL), EINVAL);
	blob->size = ia_css_sizeof_terminal_manifest(terminal_type, section_count);
	blob->terminal_type = terminal_type;
	if (IA_CSS_TERMINAL_TYPE_PARAM_CACHED == terminal_type){
		ia_css_param_terminal_manifest_t *pblob = (ia_css_param_terminal_manifest_t *)blob;
		pblob->section_count = section_count;
		pblob->section_offset = sizeof(ia_css_param_terminal_manifest_t);
	}
	retval = 0;
EXIT:
	return retval;
}
