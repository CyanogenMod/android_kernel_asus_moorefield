
#include <ia_css_psys_sim_data.h>

static unsigned int ia_css_psys_ran_seed = 0;

void ia_css_psys_ran_set_seed (
    const unsigned int      seed)
{
	ia_css_psys_ran_seed = seed;
	return;
}

static unsigned int ia_css_psys_ran_int (void)
{
	ia_css_psys_ran_seed = 1664525UL * ia_css_psys_ran_seed + 1013904223UL;
	return ia_css_psys_ran_seed;
}

unsigned int ia_css_psys_ran_var (
    const unsigned int      bit_depth)
{
	unsigned int     out,tmp = ia_css_psys_ran_int();

	if (bit_depth > 32) {
	    out = tmp;
	} else if (bit_depth == 0) {
	    out = 0;
	} else {
	    out = (unsigned short)(tmp >> (32 - bit_depth));
	}

return out;
}

unsigned int ia_css_psys_ran_val (
    const unsigned int      range)
{
	unsigned int     out,tmp = ia_css_psys_ran_int();
	if (range > 1) {
		out = tmp % range;
	} else {
		out = 0;
	}
	return out;
}

unsigned int ia_css_psys_ran_interval (
    const unsigned int      lo,
    const unsigned int      hi)
{
	unsigned int	out,tmp = ia_css_psys_ran_int();
	unsigned int	range = hi - lo;
	if ((range > 1) && (lo < hi)) {
		out = lo + (tmp % range);
	} else {
		out = 0;
	}
	return out;
}
