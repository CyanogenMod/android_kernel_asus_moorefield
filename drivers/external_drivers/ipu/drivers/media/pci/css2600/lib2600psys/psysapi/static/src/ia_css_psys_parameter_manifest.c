
#include <ia_css_psys_parameter_manifest.h>

#include <type_support.h>
#include <error_support.h>
#include <print_support.h>

size_t ia_css_sizeof_parameter_manifest(void)
{
	size_t	size = sizeof(ia_css_parameter_manifest_t);

	return size;
}

int ia_css_parameter_manifest_print(
	const ia_css_parameter_manifest_t		*manifest,
	void									*fid)
{
	int	retval = -1;
	FILE	*f = (FILE *)fid;

	verifexit(manifest != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(f,"ia_css_parameter_manifest_print\n");
	fprintf(f,"kernel_id = %d\n",(int)manifest->kernel_id);
	fprintf(f,"mem_type_id = %d\n",(int)manifest->kernel_id);
	fprintf(f,"mem_size = %d\n",(int)manifest->kernel_id);
	fprintf(f,"mem_offset = %d\n",(int)manifest->kernel_id);

	retval = 0;
EXIT:
	return retval;
}
