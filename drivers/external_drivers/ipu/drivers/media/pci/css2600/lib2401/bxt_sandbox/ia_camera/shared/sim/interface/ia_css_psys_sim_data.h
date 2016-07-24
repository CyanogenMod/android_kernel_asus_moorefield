/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

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



