
#include <ia_css_psys_terminal_manifest.h>
#include "ia_css_psys_program_group_private.h"

#include <type_support.h>
#include <error_support.h>
#include <print_support.h>
#include <cpu_mem_support.h>

static const char *ia_css_terminal_type_string(
	const ia_css_terminal_type_t			terminal_type);

ia_css_terminal_manifest_t *ia_css_terminal_manifest_alloc(
	const ia_css_terminal_type_t			terminal_type)
{
	size_t	size = 0;
	ia_css_terminal_manifest_t	*manifest = NULL;

	errno = 0;

	if (terminal_type == IA_CSS_TERMINAL_TYPE_PARAM_CACHED) {
		manifest = (ia_css_terminal_manifest_t *)ia_css_cpu_mem_alloc(sizeof(ia_css_param_terminal_manifest_t));
		size += sizeof(ia_css_param_terminal_manifest_t);
		ia_css_cpu_mem_set_zero(manifest, sizeof(ia_css_param_terminal_manifest_t));
	} else if (terminal_type < IA_CSS_N_TERMINAL_TYPES) {
		manifest = (ia_css_terminal_manifest_t *)ia_css_cpu_mem_alloc(sizeof(ia_css_data_terminal_manifest_t));
		size += sizeof(ia_css_data_terminal_manifest_t);
		ia_css_cpu_mem_set_zero(manifest, sizeof(ia_css_param_terminal_manifest_t));
	} else {
		verifexit(false, EINVAL);
	}

	manifest->size = ia_css_sizeof_terminal_manifest(terminal_type);
	verifexit(manifest->size == size, EINVAL);

	verifexit(ia_css_terminal_manifest_set_type(manifest, terminal_type) == 0, EINVAL);

EXIT:
	if (errno != 0) {
		manifest = ia_css_terminal_manifest_free(manifest);
	}

	return manifest;
}

ia_css_terminal_manifest_t *ia_css_terminal_manifest_free(
	ia_css_terminal_manifest_t				*manifest)
{
	ia_css_cpu_mem_free((void *)manifest);
	manifest = NULL;
	return manifest;
}

int ia_css_terminal_manifest_print(
	const ia_css_terminal_manifest_t		*manifest,
	void									*fid)
{
	int	retval = -1;
	FILE	*f = (FILE *)fid;
	ia_css_terminal_type_t	terminal_type = ia_css_terminal_manifest_get_type(manifest);

	verifexit(manifest != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(f,"ia_css_terminal_manifest_print\n");
	fprintf(f,"sizeof(manifest) = %d\n",(int)ia_css_terminal_manifest_get_size(manifest));

/*	fprintf(f,"typeof(manifest) = %d\n",(int)terminal_type); */
	fprintf(f,"typeof(manifest) = %s\n",ia_css_terminal_type_string(terminal_type));

	if (terminal_type == IA_CSS_TERMINAL_TYPE_PARAM_CACHED) {
		ia_css_param_terminal_manifest_t	*pterminal_manifest = (ia_css_param_terminal_manifest_t *)manifest;
		uint8_t								section_count = ia_css_param_terminal_manifest_get_section_count(pterminal_manifest);
		int	i;

		fprintf(f,"sections(manifest) = %d\n",(int)section_count);
		for (i = 0; i < section_count; i++) {
			ia_css_parameter_manifest_print(ia_css_param_terminal_manifest_get_parameter_manifest(pterminal_manifest, i), fid);
		}
	} else if (terminal_type < IA_CSS_N_TERMINAL_TYPES) {
		ia_css_data_terminal_manifest_t	*dterminal_manifest = (ia_css_data_terminal_manifest_t *)manifest;
		int	i;

		fprintf(f,"formats(manifest) = %04x\n",(int)ia_css_data_terminal_manifest_get_frame_format_bitmap(dterminal_manifest));
		fprintf(f,"connection(manifest) = %04x\n",(int)ia_css_data_terminal_manifest_get_connection_bitmap(dterminal_manifest));
		fprintf(f,"dependent(manifest) = %d\n",(int)dterminal_manifest->terminal_dependency);

		fprintf(f,"\tmin_size[%d]   = {",IA_CSS_N_DATA_DIMENSION);
		for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
			fprintf(f,"%4d, ",dterminal_manifest->min_size[i]);
		}
		fprintf(f,"%4d}\n",dterminal_manifest->min_size[i]);

		fprintf(f,"\tmax_size[%d]   = {",IA_CSS_N_DATA_DIMENSION);
		for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
			fprintf(f,"%4d, ",dterminal_manifest->max_size[i]);
		}
		fprintf(f,"%4d}\n",dterminal_manifest->max_size[i]);

		fprintf(f,"\tmin_fragment_size[%d]   = {",IA_CSS_N_DATA_DIMENSION);
		for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
			fprintf(f,"%4d, ",dterminal_manifest->min_fragment_size[i]);
		}
		fprintf(f,"%4d}\n",dterminal_manifest->min_fragment_size[i]);

		fprintf(f,"\tmax_fragment_size[%d]   = {",IA_CSS_N_DATA_DIMENSION);
		for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
			fprintf(f,"%4d, ",dterminal_manifest->max_fragment_size[i]);
		}
		fprintf(f,"%4d}\n",dterminal_manifest->max_fragment_size[i]);
	}

	retval = 0;
EXIT:
	return retval;
}

static const char *terminal_type_strings[IA_CSS_N_TERMINAL_TYPES + 1] = {
	"IA_CSS_TERMINAL_TYPE_DATA_IN",
	"IA_CSS_TERMINAL_TYPE_DATA_OUT",
	"IA_CSS_TERMINAL_TYPE_PARAM_STREAM",
	"IA_CSS_TERMINAL_TYPE_PARAM_CACHED",
	"IA_CSS_TERMINAL_TYPE_STATE_IN",
	"IA_CSS_TERMINAL_TYPE_STATE_OUT",
	"UNDEFINED_TERMINAL_TYPE"};

static const char *ia_css_terminal_type_string(
	const ia_css_terminal_type_t			terminal_type)
{
	return terminal_type_strings[min(terminal_type, IA_CSS_N_TERMINAL_TYPES)];
}
