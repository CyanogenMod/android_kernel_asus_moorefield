/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (C) 2010 - 2013 Intel Corporation.
 * All Rights Reserved.
 *
 * The source code contained or described herein and all documents
 * related to the source code ("Material") are owned by Intel Corporation
 * or licensors. Title to the Material remains with Intel
 * Corporation or its licensors. The Material contains trade
 * secrets and proprietary and confidential information of Intel or its
 * licensors. The Material is protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Material may
 * be used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No License under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or
 * delivery of the Materials, either expressly, by implication, inducement,
 * estoppel or otherwise. Any license under such intellectual property rights
 * must be express and approved by Intel in writing.
 */


#ifndef _REF_VECTOR_FUNC_H_INCLUDED_
#define _REF_VECTOR_FUNC_H_INCLUDED_

#include "storage_class.h"

#ifdef INLINE_VECTOR_FUNC
#define STORAGE_CLASS_REF_VECTOR_FUNC_H STORAGE_CLASS_INLINE
#define STORAGE_CLASS_REF_VECTOR_DATA_H STORAGE_CLASS_INLINE_DATA
#else /* INLINE_VECTOR_FUNC */
#define STORAGE_CLASS_REF_VECTOR_FUNC_H STORAGE_CLASS_EXTERN
#define STORAGE_CLASS_REF_VECTOR_DATA_H STORAGE_CLASS_EXTERN_DATA
#endif  /* INLINE_VECTOR_FUNC */


#include "ref_vector_func_types.h"

/** @brief Doubling multiply accumulate with saturation
 *
 * @param[in] acc accumulator
 * @param[in] a multiply input
 * @param[in] b multiply input
  *
 * @return		acc + (a*b)
 *
 * This function will do a doubling multiply ont
 * inputs a and b, and will add the result to acc.
 * in case of an overflow of acc, it will saturate.
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector2w OP_1w_maccd_sat(
	tvector2w acc,
	tvector1w a,
	tvector1w b );

/** @brief Doubling multiply accumulate
 *
 * @param[in] acc accumulator
 * @param[in] a multiply input
 * @param[in] b multiply input
  *
 * @return		acc + (a*b)
 *
 * This function will do a doubling multiply ont
 * inputs a and b, and will add the result to acc.
 * in case of overflow it will not saturate but wrap around.
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector2w OP_1w_maccd(
	tvector2w acc,
	tvector1w a,
	tvector1w b );

/** @brief Re-aligning multiply
 *
 * @param[in] a multiply input
 * @param[in] b multiply input
 * @param[in] shift shift amount
 *
 * @return		(a*b)>>shift
 *
 * This function will multiply a with b, followed by a right
 * shift with rounding. the result is saturated and casted
 * to single precision.
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w OP_1w_mul_realigning(
	tvector1w a,
	tvector1w b,
	tscalar1w shift );

/** @brief Config Unit Input Processing
 *
 * @param[in] a 	    input
 * @param[in] input_scale   input scaling factor
 * @param[in] input_offset  input offset factor
 *
 * @return		    scaled & offset added input	clamped to MAXVALUE
 *
 * As part of input processing for piecewise linear estimation config unit,
 * this function will perform scaling followed by adding offset and
 * then clamping to the MAX InputValue
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w OP_1w_input_scaling_offset_clamping(
	tvector1w a,
	tscalar1w_5bit_signed input_scale,
	tscalar1w_5bit_signed input_offset);

/** @brief Config Unit Output Processing
 *
 * @param[in] a 	     output
 * @param[in] output_scale   output scaling factor
 *
 * @return		     scaled & clamped output value
 *
 * As part of output processing for piecewise linear estimation config unit,
 * This function will perform scaling and then clamping to output
 * MAX value.
 */


STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w OP_1w_output_scaling_clamping(
	tvector1w a,
	tscalar1w_5bit_signed output_scale);

/** @brief Config Unit Piecewiselinear estimation
 *
 * @param[in] a 	           input
 * @param[in] test_config_points   config parameter structure
 *
 * @return		     	   piecewise linear estimated output
 *
 * Given a set of N points {(x1,y1),()x2,y2), ....,(xn,yn)}, to find
 * the functional value at an arbitrary point around the input set,
 * this function will perform input processing followed by piecewise
 * linear estimation and then output processing to yield the final value.
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w OP_1w_piecewise_estimation(
	tvector1w a,
	ref_config_points test_config_points);

/** @brief XCU (Fast Config Unit Piecewise linear estimation
 *
 * @param[in] a input
 * @param[in] test_config_points   config parameter structure
 *
 * @return		     	   piecewise linear estimated output
 *
 * Given a set of N points, not necessariliy equidistant,
 * {(x1,y1), (x2,y2), ...., (xn,yn)}, to find
 * the functional value at an arbitrary point around the input set,
 * this function will perform input processing followed by piecewise
 * linear estimation and then output processing to yield the final value.
 * Piecewise liner estimation is performed with the help of LUT generated
 * based on configuration input. Range of min and max config point is
 * divided into 32 equal intervals and LUT is created for each of these 32
 * intervals for slope, y_offset and x_prev. Interval of current point x
 * is calculated by dividing it with the range approximated to nearest
 * power of 2 (upper bound)
 *
 * * @details
 * Given a set of N configuration points, not necessarily equidistant,
 * {(x1,y1), (x2,y2),....(xn,yn)}, not necessarily euqidistant, this function
 * gives the piecewise linear estimation for any arbitrary point around the
 * config points. The distance between the minimum config point and maximum
 * config point (range) is divided into ISP_NWAY equal intervals i.e. the
 * LUT size is equal to ISP_NWAY. It is assumed that is always a power of 2.
 * In the current case, 32 intervals are used as the ISP is 32 way. It should
 * be noted that some approximation is introduced here as the range may not be
 * an integer multiple of ISP_NWAY.
 * A LUT is created from the configuration input for slope, y_offset and
 * x_prev_value. Input values of a particular conifg point are replicated in
 * the LUT till the interval reaches the next config point. LUT is then filled
 * with the data of the next config point. For example:
 * LUT for Slope can look like S = [s1 s2 s2 s2 s2 s3 s3 s4 s5 s5 s6 s6 s6..],
 * depending on the distance between the config points. Similarly the data
 * is filled for y_offset and X-prev_values. Once this LUT is created,
 * the interval of the any input x is identified by dividing it by the range
 * (approximated to the nearest power of 2 (upper bound)). This is done for
 * fast division operation to identify the interval of input x. Once the
 * interavl of the input is idenitfied, its config data is retrieved from the
 * LUT and output y is calculated. Input points less than x1 are treated as
 * simple case of using first y_pffset as the output.
 */

STORAGE_CLASS_REF_VECTOR_FUNC_H  tvector1w OP_1w_XCU(
	tvector1w x,
	ref_config_points test_config_points);


/** @brief Coring
 *
 * @param[in] coring_vec   Amount of coring based on brightness level
 * @param[in] filt_input   Vector of input pixels on which Coring is applied
 * @param[in] m_CnrCoring0 Coring Level0
 *
 * @return                 vector of filtered pixels after coring is applied
 *
 * This function will perform adaptive coring based on brightness level to
 * remove noise
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w coring(
	tvector1w coring_vec,
	tvector1w filt_input,
	tscalar1w m_CnrCoring0 );

/** @brief Normalised FIR with coefficients [3,4,1]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [3,4,1],
 *-5dB at Fs/2, -90 degree phase shift (quarter pixel)
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_5dB_m90_nrm (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [1,4,3]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,4,3],
 *-5dB at Fs/2, +90 degree phase shift (quarter pixel)
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_5dB_p90_nrm (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [1,2,1]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,2,1], -6dB at Fs/2
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_6dB_nrm (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [13,16,3]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [13,16,3],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_6dB_nrm_ph0 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [9,16,7]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [9,16,7],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_6dB_nrm_ph1 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [5,16,11]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [5,16,11],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_6dB_nrm_ph2 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with coefficients [1,16,15]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,16,15],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_6dB_nrm_ph3 (
	const s_1w_1x3_matrix		m);

/** @brief Normalised FIR with programable phase shift
 *
 * @param[in] m	1x3 matrix with pixels
 * @param[in] coeff	phase shift
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [8-coeff,16,8+coeff],
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_6dB_nrm_calc_coeff (
	const s_1w_1x3_matrix		m, tscalar1w_3bit coeff);

/** @brief 3 tab FIR with coefficients [1,1,1]
 *
 * @param[in] m	1x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * FIR with coefficients [1,1,1], -9dB at Fs/2 normalized with factor 1/2
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x3m_9dB_nrm (
	const s_1w_1x3_matrix		m);


/** @brief Normalised 2D FIR with coefficients  [1;2;1] * [1,2,1]
 *
 * @param[in] m	3x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients  [1;2;1] * [1,2,1]
 * Unity gain filter through repeated scaling and rounding
 *	- 6 rotate operations per output
 *	- 8 vector operations per output
 * _______
 *   14 total operations
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir3x3m_6dB_nrm (
	const s_1w_3x3_matrix		m);

/** @brief Normalised 2D FIR with coefficients  [1;1;1] * [1,1,1]
 *
 * @param[in] m	3x3 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;1;1] * [1,1,1]
 *
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 6 rotate operations per output
 *	- 8 vector operations per output
 * _______
 *   14 operations
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir3x3m_9dB_nrm (
	const s_1w_3x3_matrix		m);

/** @brief Normalised dual output 2D FIR with coefficients  [1;2;1] * [1,2,1]
 *
 * @param[in] m	4x3 matrix with pixels
 *
 * @return		two filtered outputs (2x1 matrix)
 *
 * This function will calculate the
 * Normalised FIR with coefficients  [1;2;1] * [1,2,1]
 * and produce two outputs (vertical)
 * Unity gain filter through repeated scaling and rounding
 * compute two outputs per call to re-use common intermediates
 *	- 4 rotate operations per output
 *	- 6 vector operations per output (alternative possible, but in this
 *	    form it's not obvious to re-use variables)
 * _______
 *   10 total operations
 */
 STORAGE_CLASS_REF_VECTOR_FUNC_H s_1w_2x1_matrix fir3x3m_6dB_out2x1_nrm (
	const s_1w_4x3_matrix		m);

/** @brief Normalised dual output 2D FIR with coefficients [1;1;1] * [1,1,1]
 *
 * @param[in] m	4x3 matrix with pixels
 *
 * @return		two filtered outputs (2x1 matrix)
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;1;1] * [1,1,1]
 * and produce two outputs (vertical)
 * (near) Unity gain filter through repeated scaling and rounding
 * compute two outputs per call to re-use common intermediates
 *	- 4 rotate operations per output
 *	- 7 vector operations per output (alternative possible, but in this
 *	    form it's not obvious to re-use variables)
 * _______
 *   11 total operations
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H s_1w_2x1_matrix fir3x3m_9dB_out2x1_nrm (
	const s_1w_4x3_matrix		m);

/** @brief Normalised 2D FIR 5x5
 *
 * @param[in] m	5x5 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;1;1] * [1;2;1] * [1,2,1] * [1,1,1]
 * and produce a filtered output
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 20 rotate operations per output
 *	- 28 vector operations per output
 * _______
 *   48 total operations
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir5x5m_15dB_nrm (
	const s_1w_5x5_matrix	m);

/** @brief Normalised FIR 1x5
 *
 * @param[in] m	1x5 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1,2,1] * [1,1,1] = [1,4,6,4,1]
 * and produce a filtered output
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 4 rotate operations per output
 *	- 5 vector operations per output
 * _______
 *   9 total operations
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x5m_12dB_nrm (
	const s_1w_1x5_matrix m);

/** @brief Normalised 2D FIR 5x5
 *
 * @param[in] m	5x5 matrix with pixels
 *
 * @return		filtered output
 *
 * This function will calculate the
 * Normalised FIR with coefficients [1;2;1] * [1;2;1] * [1,2,1] * [1,2,1]
 * and produce a filtered output
 * (near) Unity gain filter through repeated scaling and rounding
 *	- 20 rotate operations per output
 *	- 30 vector operations per output
 * _______
 *   50 total operations
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir5x5m_12dB_nrm (
	const s_1w_5x5_matrix m);

STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x5m_box (
	s_1w_1x5_matrix m);

STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w fir1x9m_box (
	s_1w_1x9_matrix m);

/** @brief Mean of 1x3 matrix
 *
 *  @param[in] m 1x3 matrix with pixels
 *
 *  @return mean of 1x3 matrix
 *
 * This function calculates the mean of 1x3 pixels,
 * with a factor of 4/3.
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w mean1x3m(
	s_1w_1x3_matrix m);

/** @brief Mean of 3x3 matrix
 *
 *  @param[in] m 3x3 matrix with pixels
 *
 *  @return mean of 3x3 matrix
 *
 * This function calculates the mean of 3x3 pixels,
 * with a factor of 16/9.
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w mean3x3m(
	s_1w_3x3_matrix m);

/** @brief Mean of 1x4 matrix
 *
 *  @param[in] m 1x4 matrix with pixels
 *
 *  @return mean of 1x4 matrix
 *
 * This function calculates the mean of 1x4 pixels
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w mean1x4m(
	s_1w_1x4_matrix m);

/** @brief Mean of 4x4 matrix
 *
 *  @param[in] m 4x4 matrix with pixels
 *
 *  @return mean of 4x4 matrix
 *
 * This function calculates the mean of 4x4 matrix with pixels
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w mean4x4m(
	s_1w_4x4_matrix m);

/** @brief Mean of 2x3 matrix
 *
 *  @param[in] m 2x3 matrix with pixels
 *
 *  @return mean of 2x3 matrix
 *
 * This function calculates the mean of 2x3 matrix with pixels
 * with a factor of 8/6.
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w mean2x3m(
	s_1w_2x3_matrix m);


/** @brief Mean of 1x6 matrix
 *
 *  @param[in] m 1x6 matrix with pixels
 *
 *  @return mean of 1x6 matrix
 *
 * This function calculates the mean of 1x6 matrix with pixels
 * with a factor of 8/6.
*/
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w mean1x6m(
	s_1w_1x6_matrix m);

/** @brief Bi-linear Interpolation optimized(approximate)
 *
 * @param[in] a input0
 * @param[in] b input1
 * @param[in] c cloned weight factor
  *
 * @return		(a-b)*c + b
 *
 * This function will do bi-linear Interpolation on
 * inputs a and b using constant weight factor c
 *
 * Inputs a,b are assumed in S1.15 format
 * Weight factor has to be in range [0,1] and is assumed to be in S2.14 format
 *
 * The bilinear interpolation equation is (a*c) + b*(1-c),
 * But this is implemented as (a-b)*c + b for optimization
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w OP_1w_bilinear_interpol_approx_c(
	tvector1w a,
	tvector1w b,
	tscalar1w_signed_positive c);

/** @brief Bi-linear Interpolation optimized(approximate)
 *
 * @param[in] a input0
 * @param[in] b input1
 * @param[in] c weight factor
  *
 * @return		(a-b)*c + b
 *
 * This function will do bi-linear Interpolation on
 * inputs a and b using weight factor c
 *
 * Inputs a,b are assumed in S1.15 format
 * Weight factor has to be in range [0,1] and is assumed to be in S2.14 format
 *
 * The bilinear interpolation equation is (a*c) + b*(1-c),
 * But this is implemented as (a-b)*c + b for optimization
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w OP_1w_bilinear_interpol_approx(
	tvector1w a,
	tvector1w b,
	tvector1w_signed_positive c);

/** @brief Bi-linear Interpolation
 *
 * @param[in] a input0
 * @param[in] b input1
 * @param[in] c weight factor
  *
 * @return		(a*c) + b*(1-c)
 *
 * This function will do bi-linear Interpolation on
 * inputs a and b using weight factor c
 *
 * Inputs a,b are assumed in S1.15 format
 * Weight factor has to be in range [0,1] and is assumed to be in S2.14 format
 *
 * The bilinear interpolation equation is (a*c) + b*(1-c),
 */
STORAGE_CLASS_REF_VECTOR_FUNC_H tvector1w OP_1w_bilinear_interpol(
	tvector1w a,
	tvector1w b,
	tscalar1w_signed_positive c);

#ifndef INLINE_VECTOR_FUNC
#define STORAGE_CLASS_REF_VECTOR_FUNC_C
#define STORAGE_CLASS_REF_VECTOR_DATA_C const
#else /* INLINE_VECTOR_FUNC */
#define STORAGE_CLASS_REF_VECTOR_FUNC_C STORAGE_CLASS_REF_VECTOR_FUNC_H
#define STORAGE_CLASS_REF_VECTOR_DATA_C STORAGE_CLASS_REF_VECTOR_DATA_H
#include "ref_vector_func.c"
#define VECTOR_FUNC_INLINED
#endif  /* INLINE_VECTOR_FUNC */

#endif /*_REF_VECTOR_FUNC_H_INCLUDED_*/
