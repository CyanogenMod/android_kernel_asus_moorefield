#ifndef __IA_CSS_PSYS_SIM_DATA_H_INCLUDED__
#define __IA_CSS_PSYS_SIM_DATA_H_INCLUDED__

/*! Set the seed if the random number generator

 @param	seed[in]				Random number generator seed
 */
extern void ia_css_psys_ran_set_seed (
    const unsigned int      seed);

/*! Generate a random number of a specified bit depth

 @param	bit_depth[in]			The number of bits of the random output

 @return out, weight(out) <= bit_depth, 0 on error
 */
extern unsigned int ia_css_psys_ran_var (
    const unsigned int      bit_depth);

/*! Generate a random number of a specified range

 @param	range[in]				The range of the random output

 @return 0 <= out < range, 0 on error
 */
extern unsigned int ia_css_psys_ran_val (
    const unsigned int      range);

/*! Generate a random number in a specified interval

 @param	lo[in]				The lower bound of the random output range
 @param	hi[in]				The higher bound of the random output range

 @return lo <= out < hi, 0 on error
 */
extern unsigned int ia_css_psys_ran_interval (
    const unsigned int      lo,
    const unsigned int      hi);

#endif /* __IA_CSS_PSYS_SIM_DATA_H_INCLUDED__  */



