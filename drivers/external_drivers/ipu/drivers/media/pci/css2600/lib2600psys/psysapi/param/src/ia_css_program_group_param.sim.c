
#include <ia_css_program_group_param.h>

#include <ia_css_program_group_param_private.h>
#include <ia_css_psys_manifest_types.h>
#include <ia_css_psys_program_group_manifest.h>

#include <error_support.h>
#include <print_support.h>
#include <misc_support.h>
#include <cpu_mem_support.h>


ia_css_program_group_param_t *ia_css_program_group_param_create(
	const unsigned int						specification,
	const ia_css_program_group_manifest_t	*manifest)
{
	ia_css_program_group_param_t	*param = NULL;
	ia_css_kernel_bitmap_t			kernel_enable_bitmap;
	ia_css_kernel_bitmap_t			kernel_manifest_bitmap;
	uint8_t		program_count;
	uint8_t		terminal_count;
	uint16_t	fragment_count;

	errno = 0;

	verifexit(ia_css_is_program_group_manifest_valid(manifest), EINVAL);

	switch (specification) {
	case 0:		/* Fall through */
	case 1:		/* Fall through */
	case 2:		/* Fall through */
	case 3:		/* Fall through */
		fragment_count = 2;
		program_count = ia_css_program_group_manifest_get_program_count(manifest);
		terminal_count = ia_css_program_group_manifest_get_terminal_count(manifest);

		kernel_manifest_bitmap = ia_css_program_group_manifest_get_kernel_bitmap(manifest);

		do {
			kernel_enable_bitmap = ia_css_kernel_ran_bitmap();
			kernel_enable_bitmap = ia_css_kernel_bitmap_intersection(kernel_manifest_bitmap, kernel_enable_bitmap);
		} while (ia_css_is_kernel_bitmap_empty(kernel_enable_bitmap));

		break;
	default:
		verifexit(false, EINVAL);
		break;
	}

	param = ia_css_program_group_param_alloc(program_count, terminal_count, fragment_count);
	verifexit(param != NULL, EINVAL);

	param->kernel_enable_bitmap = kernel_enable_bitmap;

EXIT:

	if (errno != 0) {
		param = ia_css_program_group_param_destroy(param);
	}

	return param;
}

ia_css_program_group_param_t *ia_css_program_group_param_destroy(
	ia_css_program_group_param_t			*param)
{
	return ia_css_program_group_param_free(param);
}


ia_css_program_group_param_t *ia_css_program_group_param_alloc(
	const uint8_t							program_count,
	const uint8_t							terminal_count,
	const uint16_t							fragment_count)
{
	size_t	size = 0;
	int		i;
	ia_css_program_group_param_t	*param = NULL;

	errno = 0;

	param = (ia_css_program_group_param_t *)ia_css_cpu_mem_alloc(sizeof(ia_css_program_group_param_t));
	verifexit(param != NULL, ENOBUFS);
	size += sizeof(ia_css_program_group_param_t);
	ia_css_cpu_mem_set_zero(param, sizeof(ia_css_program_group_param_t));

	param->program_count = program_count;
	param->terminal_count = terminal_count;
	param->fragment_count = fragment_count;

	param->program_param = (ia_css_program_param_t **)ia_css_cpu_mem_alloc(program_count * sizeof(ia_css_program_param_t *));
	verifexit(param->program_param != NULL, ENOBUFS);
	size += program_count * sizeof(ia_css_program_param_t *);
	ia_css_cpu_mem_set_zero(param->program_param, program_count * sizeof(ia_css_program_param_t *));

	for (i = 0; i < program_count; i++) {
		param->program_param[i] = ia_css_program_param_alloc();
		verifexit(param->program_param[i] != NULL, ENOBUFS);
		size += ia_css_program_param_get_size(param->program_param[i]);

		param->program_param[i]->parent = param;
	}

	param->terminal_param = (ia_css_terminal_param_t **)ia_css_cpu_mem_alloc(terminal_count * sizeof(ia_css_terminal_param_t *));
	verifexit(param->terminal_param != NULL, ENOBUFS);
	size += terminal_count * sizeof(ia_css_terminal_param_t *);
	ia_css_cpu_mem_set_zero(param->terminal_param, terminal_count * sizeof(ia_css_terminal_param_t *));

	for (i = 0; i < terminal_count; i++) {
		param->terminal_param[i] = ia_css_terminal_param_alloc();
		verifexit(param->terminal_param[i] != NULL, ENOBUFS);
		size += ia_css_terminal_param_get_size(param->terminal_param[i]);

		param->terminal_param[i]->parent = param;
	}

/*
	param->size = ia_css_sizeof_program_param();
	verifexit(param->size == size, EINVAL);
 */
	param->size = size;
EXIT:

	if (errno != 0) {
		param = ia_css_program_group_param_free(param);
	}

	return param;
}

ia_css_program_group_param_t *ia_css_program_group_param_free(
	ia_css_program_group_param_t			*param)
{
	int		i;

	if (param != NULL) {
		if (param->terminal_param != NULL) {
			uint8_t	terminal_count = ia_css_program_group_param_get_terminal_count(param);

			for (i = (int)terminal_count - 1; i >= 0; i--) {
				param->terminal_param[i] = ia_css_terminal_param_free(param->terminal_param[i]);
			}

			ia_css_cpu_mem_free((void *)param->terminal_param);
			param->terminal_param = NULL;
		}

		if (param->program_param != NULL) {
			uint8_t	program_count = ia_css_program_group_param_get_program_count(param);

			for (i = (int)program_count - 1; i >= 0; i--) {
				param->program_param[i] = ia_css_program_param_free(param->program_param[i]);
			}

			ia_css_cpu_mem_free((void *)param->program_param);
			param->program_param = NULL;
		}
		ia_css_cpu_mem_free((void *)param);
		param = NULL;
	}
	return param;
}

int ia_css_program_group_param_print(
	const ia_css_program_group_param_t		*param,
	void									*fid)
{
	int	retval = -1;
	FILE	*f = (FILE *)fid;
	int		i;
	uint8_t	program_count, terminal_count;
	ia_css_kernel_bitmap_t	bitmap;

	verifexit(param != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(f,"ia_css_program_group_param_print\n");
	fprintf(f,"sizeof(program_group_param) = %d\n",(int)ia_css_program_group_param_get_size(param));

	program_count = ia_css_program_group_param_get_program_count(param);
	terminal_count = ia_css_program_group_param_get_terminal_count(param);

	bitmap = ia_css_program_group_param_get_kernel_enable_bitmap(param);
	verifexit(ia_css_kernel_bitmap_print(bitmap, fid) == 0, EINVAL);

	fprintf(f,"%d program params\n",(int)program_count);
	for (i = 0; i < (int)program_count; i++) {
		ia_css_program_param_t	*program_param = ia_css_program_group_param_get_program_param(param, i);
		retval = ia_css_program_param_print(program_param, fid);
		verifjmpexit(retval == 0);
	}
	fprintf(f,"%d terminal params\n",(int)terminal_count);
	for (i = 0; i < (int)terminal_count; i++) {
		ia_css_terminal_param_t	*terminal_param = ia_css_program_group_param_get_terminal_param(param, i);
		retval = ia_css_terminal_param_print(terminal_param, fid);
		verifjmpexit(retval == 0);
	}

	retval = 0;
EXIT:
	return retval;
}

ia_css_program_param_t *ia_css_program_param_alloc(void)
{
	size_t	size = 0;
	ia_css_program_param_t	*param = ia_css_cpu_mem_alloc(sizeof(ia_css_program_param_t));
	verifexit(param != NULL, ENOBUFS);
	size += sizeof(ia_css_program_param_t);
	ia_css_cpu_mem_set_zero(param, sizeof(ia_css_program_param_t));
/*
	param->size = ia_css_sizeof_program_param();
	verifexit(param->size == size, EINVAL);
 */
	param->size = size;
	param->parent = NULL;
EXIT:
	return param;
}

ia_css_program_param_t *ia_css_program_param_free(
	ia_css_program_param_t					*param)
{
	ia_css_cpu_mem_free((void *)param);
	param = NULL;
	return param;
}

int ia_css_program_param_print(
	const ia_css_program_param_t			*param,
	void						*fid)
{
	int	retval = -1;
	FILE	*f = (FILE *)fid;
	ia_css_kernel_bitmap_t	bitmap;

	verifexit(param != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(f,"ia_css_program_param_print\n");
	fprintf(f,"sizeof(program_param) = %d\n",(int)ia_css_program_param_get_size(param));

	bitmap = ia_css_program_param_get_kernel_enable_bitmap(param);
	verifexit(ia_css_kernel_bitmap_print(bitmap, fid) == 0, EINVAL);

	retval = 0;
EXIT:
	return retval;
}

ia_css_terminal_param_t *ia_css_terminal_param_alloc(void)
{
	size_t	size = 0;
	ia_css_terminal_param_t	*param = (ia_css_terminal_param_t *)ia_css_cpu_mem_alloc(sizeof(ia_css_terminal_param_t));
	verifexit(param != NULL, ENOBUFS);
	size += sizeof(ia_css_terminal_param_t);
	ia_css_cpu_mem_set_zero(param, sizeof(ia_css_terminal_param_t));
/*
	param->size = ia_css_sizeof_terminal_param();
	verifexit(param->size == size, EINVAL);
 */
	param->size = size;
EXIT:
	return param;
}

ia_css_terminal_param_t *ia_css_terminal_param_free(
	ia_css_terminal_param_t					*param)
{
	ia_css_cpu_mem_free((void *)param);
	param = NULL;
	return param;
}

int ia_css_terminal_param_print(
	const ia_css_terminal_param_t			*param,
	void									*fid)
{
	int	retval = -1;
	FILE	*f = (FILE *)fid;

	verifexit(param != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(f,"ia_css_terminal_param_print\n");
	fprintf(f,"sizeof(terminal_param) = %d\n",(int)ia_css_terminal_param_get_size(param));

	fprintf(f,"\tframe_format_type = %d\n",param->frame_format_type);
/*	fprintf(f,"\tframe_format_type = %s\n",ia_css_frame_format_string(param->frame_format_type)); */

	retval = 0;
EXIT:
	return retval;
}
