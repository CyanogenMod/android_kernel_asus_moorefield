#include "ia_css_psys_terminal.h"
#include "ia_css_psys_terminal.hsys.user.h"
#include "ia_css_psys_program_group_manifest.h"
#include "ia_css_psys_terminal_manifest.h"
#include "ia_css_psys_program_manifest.h"
#include "ia_css_psys_process_group.h"
#include "ia_css_psys_manifest_types.h"
#include "ia_css_program_group_param.h"

#include "assert_support.h"
#include "error_support.h"
#include "cpu_mem_support.h"

ia_css_program_group_param_t *ia_css_bxt_pss_program_group_param_create(
	const unsigned int						specification,
	const ia_css_program_group_manifest_t	*manifest)
{
	ia_css_kernel_bitmap_t			kernel_enable_bitmap;
	ia_css_kernel_bitmap_t			kernel_manifest_bitmap;
	uint8_t		program_count;
	uint8_t		terminal_count;
	uint16_t	fragment_count;
	ia_css_program_group_param_t *prg_group_param = NULL;
	int size;
	int i, check_group_param = 0;
	ia_css_terminal_type_t	terminal_format_type[3] = {
		IA_CSS_DATA_CUSTOM,
		IA_CSS_DATA_CUSTOM,
		IA_CSS_N_FRAME_FORMAT_TYPES};

	const ia_css_frame_format_type_t format_types[3] = {
		IA_CSS_DATA_CUSTOM_NO_DESCRIPTOR,  /*set by client.*/
		IA_CSS_DATA_CUSTOM_NO_DESCRIPTOR,  /*set by client.*/
		IA_CSS_N_FRAME_FORMAT_TYPES};

	errno = 0;

	verifexit(ia_css_is_program_group_manifest_valid(manifest), EINVAL);

	check_group_param = 1;

	program_count = 1;
	terminal_count = 3;
	switch (specification) {
	case 0:		/* Fall through */
		fragment_count = 1;
		break;
	default:
		fragment_count = 2;
		break;
	}


	size = ia_css_sizeof_program_group_param(
		program_count,
		terminal_count,
		fragment_count);

	prg_group_param = (ia_css_program_group_param_t *)ia_css_cpu_mem_alloc(size);
	verifexit(prg_group_param != NULL, EINVAL);
	ia_css_cpu_mem_set_zero(prg_group_param, size);
	ia_css_program_group_param_init(prg_group_param,
				program_count,
				terminal_count,
				fragment_count,
				format_types);

	switch (specification) {
	case 0:		/* Fall through */
	case 1:		/* Fall through */
	case 2:		/* Fall through */
	case 3:		/* Fall through */
		program_count = ia_css_program_group_manifest_get_program_count(manifest);
		terminal_count = ia_css_program_group_manifest_get_terminal_count(manifest);

		kernel_manifest_bitmap = ia_css_program_group_manifest_get_kernel_bitmap(manifest);
		kernel_enable_bitmap = kernel_manifest_bitmap;

		break;
	default:
		verifexit(false, EINVAL);
		break;
	}

	ia_css_program_group_param_set_kernel_enable_bitmap(prg_group_param, kernel_enable_bitmap);
	/* setup terminals */
	for (i = 0; i < terminal_count; i++) {
		ia_css_terminal_param_t *terminal_param = ia_css_program_group_param_get_terminal_param(prg_group_param, i);
		verifexit(terminal_param != NULL, EINVAL);
		terminal_param->frame_format_type = terminal_format_type[i];
		terminal_param->dimensions[0] = 32;
		terminal_param->dimensions[1] = 32;
		terminal_param->bpp = 16;
		/* setup fragments */
		// TODO: fill in...
		terminal_param->fragment_dimensions[0] = 16;
		terminal_param->fragment_dimensions[1] = 32;
		terminal_param->stride = 64;
		terminal_param->offset = 0;
	}
EXIT:
	if (errno && check_group_param) {
		ia_css_cpu_mem_free(prg_group_param);
		prg_group_param = NULL;
	}
	return prg_group_param;
}

ia_css_program_group_manifest_t *
ia_css_bxt_pss_program_group_manifest_create(
	const unsigned int specification)
{
	ia_css_program_group_manifest_t *prg_group_manifest;
	ia_css_program_manifest_t *prg_manifest;
	ia_css_data_terminal_manifest_t *dterminal_manifest;
	ia_css_param_terminal_manifest_t *pterminal_manifest;
	uint32_t program_count;
	uint32_t terminal_count;
	uint32_t subgraph_count;
	uint32_t kernel_count;
	ia_css_kernel_bitmap_t			bitmap;
	int ret;
	const ia_css_terminal_type_t terminal_types[3] = {
						IA_CSS_TERMINAL_TYPE_DATA_IN,
						IA_CSS_TERMINAL_TYPE_DATA_OUT,
						IA_CSS_TERMINAL_TYPE_PARAM_CACHED
						};
	const uint16_t section_count = 0;
	uint8_t program_dependency[1] = {0};
	uint8_t terminal_dependency[1] = {2};
	int i;

	int size;

	(void)specification;

	program_count = 1;
	terminal_count = 3;
	subgraph_count = 1;
	kernel_count = 4;

	size = ia_css_sizeof_program_group_manifest(program_count,
						terminal_count,
						program_dependency,
						terminal_dependency,
						terminal_types,
						section_count);

	prg_group_manifest = (ia_css_program_group_manifest_t *)ia_css_cpu_mem_alloc(size);
	verifexit(prg_group_manifest != NULL, EINVAL);
	ia_css_cpu_mem_set_zero(prg_group_manifest, size);
	ia_css_program_group_manifest_init(
		prg_group_manifest,
		program_count,
		terminal_count,
		program_dependency,
		terminal_dependency,
		terminal_types,
		section_count);

	ret = ia_css_program_group_manifest_set_program_group_ID(prg_group_manifest,
		100);
	assert(ret == 0);
	ia_css_program_group_manifest_set_alignment(prg_group_manifest, 32);
	//ia_css_program_group_manifest_set_program_group_subgraph_count(prg_group_manifest, subgraph_count);

	bitmap = ia_css_kernel_bitmap_clear();
	for (i = 0; i < (int)kernel_count; i++) {
		bitmap = ia_css_kernel_bitmap_set(bitmap, i);
	}
	ia_css_program_group_manifest_set_kernel_bitmap(prg_group_manifest, bitmap);

	dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(prg_group_manifest, 0);
	verifexit(dterminal_manifest != NULL, EINVAL);
	verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 0) == 0, EINVAL);
	//verifexit(ia_css_terminal_manifest_specify_resources((ia_css_terminal_manifest_t *)dterminal_manifest, ia_css_data_format_bit_mask(terminal_format_type[0])) == 0, EINVAL);

	dterminal_manifest = ia_css_program_group_manifest_get_data_terminal_manifest(prg_group_manifest, 1);
	verifexit(dterminal_manifest != NULL, EINVAL);
	verifexit(ia_css_data_terminal_manifest_set_kernel_bitmap_unique(dterminal_manifest, 0) == 0, EINVAL);
	//verifexit(ia_css_terminal_manifest_specify_resources((ia_css_terminal_manifest_t *)dterminal_manifest, ia_css_data_format_bit_mask(terminal_format_type[1])) == 0, EINVAL);

	pterminal_manifest = ia_css_program_group_manifest_get_param_terminal_manifest(prg_group_manifest, 2);
	verifexit(pterminal_manifest != NULL, EINVAL);
	//verifexit(ia_css_terminal_manifest_specify_resources((ia_css_terminal_manifest_t *)pterminal_manifest, kernel_count) == 0, EINVAL);

	/* Program manifest */
	prg_manifest = ia_css_program_group_manifest_get_program_manifest(
		prg_group_manifest, 0);
	assert(prg_manifest != NULL);

	ret = ia_css_program_manifest_set_program_ID(prg_manifest,
		101);
	assert(ret == 0);

	verifexit(ia_css_program_manifest_set_type(prg_manifest, IA_CSS_PROGRAM_TYPE_SINGULAR) == 0, EINVAL);
	bitmap = ia_css_kernel_bitmap_clear();
	bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 0);
	bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 1);
	bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 2);
	bitmap = ia_css_kernel_bitmap_set_unique(bitmap, 3);
	verifexit(!ia_css_is_kernel_bitmap_empty(bitmap), EINVAL);
	verifexit(ia_css_program_manifest_set_kernel_bitmap(prg_manifest, bitmap) == 0, EINVAL);

	verifexit(ia_css_program_manifest_set_terminal_dependency(prg_manifest, 0, 0) == 0, EINVAL);
	verifexit(ia_css_program_manifest_set_terminal_dependency(prg_manifest, 1, 1) == 0, EINVAL);
EXIT:
	if (errno) {
		ia_css_cpu_mem_free(prg_group_manifest);
		prg_group_manifest = NULL;
	}
	return prg_group_manifest;
}
