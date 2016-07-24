
#include <ia_css_kernel_bitmap.h>

#include <ia_css_psys_sim_data.h>

#include <type_support.h>
#include <error_support.h>
#include <print_support.h>

int ia_css_kernel_bitmap_print(
	const ia_css_kernel_bitmap_t			bitmap,
	void									*fid)
{
	int	retval = -1;
	FILE	*f = (FILE *)fid;
	ia_css_kernel_bitmap_t	loc_bitmap = bitmap;
	int		i;

	verifexit(fid != NULL, EINVAL);

	fprintf(f,"{ ");
	for (i = 0; (i < IA_CSS_KERNEL_BITMAP_BITS) && !ia_css_is_kernel_bitmap_empty(loc_bitmap); i++) {
		unsigned int	bit = loc_bitmap & 0x1; /* ia_css_kernel_bitmap_get_lsb(loc_bitmap);*/
		loc_bitmap = loc_bitmap >> 1;		/*ia_css_kernel_bitmap_shift(loc_bitmap);*/
		fprintf(f,"%d ",bit);
	}
	fprintf(f,"}\n");

	retval = 0;
EXIT:
	return retval;
}

ia_css_kernel_bitmap_t ia_css_kernel_ran_bitmap(void)
{
	ia_css_kernel_bitmap_t	bitmap = 0;
	int	i;

	for (i = 0; i < IA_CSS_KERNEL_BITMAP_BITS; i += 32) {
		unsigned int	ranvar = ia_css_psys_ran_var(32);
		bitmap <<= 32;
		bitmap |= ranvar;
	}

	return bitmap;
}
