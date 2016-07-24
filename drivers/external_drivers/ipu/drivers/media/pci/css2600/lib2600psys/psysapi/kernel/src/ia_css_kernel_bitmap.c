
#include <ia_css_kernel_bitmap.h>

#include <ia_css_psys_sim_data.h>

#include <type_support.h>
#include <error_support.h>


static int ia_css_kernel_bitmap_compute_weight(
	const ia_css_kernel_bitmap_t			bitmap);

static int ia_css_kernel_bitmap_get_lsb(
	const ia_css_kernel_bitmap_t			bitmap);

static int ia_css_kernel_bitmap_shift(
	const ia_css_kernel_bitmap_t			bitmap);


bool ia_css_is_kernel_bitmap_intersection_empty(
	const ia_css_kernel_bitmap_t			bitmap0,
	const ia_css_kernel_bitmap_t			bitmap1)
{
	ia_css_kernel_bitmap_t intersection = ia_css_kernel_bitmap_intersection(bitmap0, bitmap1);
	return ia_css_is_kernel_bitmap_empty(intersection);
}

bool ia_css_is_kernel_bitmap_empty(
	const ia_css_kernel_bitmap_t			bitmap)
{
	return (bitmap == 0);
}

bool ia_css_is_kernel_bitmap_equal(
	const ia_css_kernel_bitmap_t			bitmap0,
	const ia_css_kernel_bitmap_t			bitmap1)
{
	return (bitmap0 == bitmap1);
}

bool ia_css_is_kernel_bitmap_onehot(
	const ia_css_kernel_bitmap_t			bitmap)
{
	return ia_css_kernel_bitmap_compute_weight(bitmap) == 1;
}

bool ia_css_is_kernel_bitmap_subset(
	const ia_css_kernel_bitmap_t			bitmap0,
	const ia_css_kernel_bitmap_t			bitmap1)
{
	ia_css_kernel_bitmap_t intersection = ia_css_kernel_bitmap_intersection(bitmap0, bitmap1);
	return ia_css_is_kernel_bitmap_equal(intersection, bitmap1);
}

ia_css_kernel_bitmap_t ia_css_kernel_bitmap_clear(void)
{
	return 0;
}

ia_css_kernel_bitmap_t ia_css_kernel_bitmap_union(
	const ia_css_kernel_bitmap_t			bitmap0,
	const ia_css_kernel_bitmap_t			bitmap1)
{
	return (bitmap0 | bitmap1);
}

ia_css_kernel_bitmap_t ia_css_kernel_bitmap_intersection(
	const ia_css_kernel_bitmap_t			bitmap0,
	const ia_css_kernel_bitmap_t			bitmap1)
{
	return (bitmap0 & bitmap1);
}

ia_css_kernel_bitmap_t ia_css_kernel_bitmap_set(
	const ia_css_kernel_bitmap_t			bitmap,
	const unsigned int						index)
{
	ia_css_kernel_bitmap_t	bit_mask = ia_css_kernel_bit_mask(index);
	return ia_css_kernel_bitmap_union(bitmap, bit_mask);
}

ia_css_kernel_bitmap_t ia_css_kernel_bitmap_set_unique(
	const ia_css_kernel_bitmap_t			bitmap,
	const unsigned int						index)
{
	ia_css_kernel_bitmap_t	ret = ia_css_kernel_bitmap_clear();
	ia_css_kernel_bitmap_t	bit_mask = ia_css_kernel_bit_mask(index);
	if (ia_css_is_kernel_bitmap_intersection_empty(bitmap, bit_mask) && !ia_css_is_kernel_bitmap_empty(bit_mask)) {
		ret = ia_css_kernel_bitmap_union(bitmap, bit_mask);
	}
	return ret;
}

ia_css_kernel_bitmap_t ia_css_kernel_bit_mask(
	const unsigned int						index)
{
	ia_css_kernel_bitmap_t	bit_mask = ia_css_kernel_bitmap_clear();
	if (index < IA_CSS_KERNEL_BITMAP_BITS) {
		bit_mask = (ia_css_kernel_bitmap_t)1 << index;
	}
	return bit_mask;
}


static int ia_css_kernel_bitmap_compute_weight(
	const ia_css_kernel_bitmap_t			bitmap)
{
	ia_css_kernel_bitmap_t	loc_bitmap = bitmap;
	int	weight = 0;
	int	i;
/* In fact; do not need the iterator "i" */
	for (i = 0; (i < IA_CSS_KERNEL_BITMAP_BITS) && !ia_css_is_kernel_bitmap_empty(loc_bitmap); i++) {
		weight += ia_css_kernel_bitmap_get_lsb(loc_bitmap);
		loc_bitmap = ia_css_kernel_bitmap_shift(loc_bitmap);
	}

	return weight;
}

static int ia_css_kernel_bitmap_get_lsb(
	const ia_css_kernel_bitmap_t			bitmap)
{
	return bitmap & 0x01;
}

static int ia_css_kernel_bitmap_shift(
	const ia_css_kernel_bitmap_t			bitmap)
{
	ia_css_kernel_bitmap_t	loc_bitmap = bitmap;
	return loc_bitmap >>= 1;
}
