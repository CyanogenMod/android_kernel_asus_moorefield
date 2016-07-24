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

#ifndef __ISP_OP2W_H_INCLUDED__
#define __ISP_OP2W_H_INCLUDED__

/*
 * This file is part of the Multi-precision vector operations exstension package.
 */

/*
 * Single-precision vector operations
 */

/*
 * Prerequisites:
 *
 */
#include "storage_class.h"

#ifdef INLINE_ISP_OP2W
#define STORAGE_CLASS_ISP_OP2W_FUNC_H STORAGE_CLASS_INLINE
#define STORAGE_CLASS_ISP_OP2W_DATA_H STORAGE_CLASS_INLINE_DATA
#else /* INLINE_ISP_OP2W */
#define STORAGE_CLASS_ISP_OP2W_FUNC_H STORAGE_CLASS_EXTERN
#define STORAGE_CLASS_ISP_OP2W_DATA_H STORAGE_CLASS_EXTERN_DATA
#endif  /* INLINE_ISP_OP2W */

/*
 * Single-precision data type specification
 */

#include "isp_op2w_types.h"

/*
 * Single-precision prototype specification
 */

/* Arithmetic */

/** @brief bitwise AND
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		bitwise and of both input arguments
 *
 * This function will calculate the bitwise and.
 * result = _a & _b
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_and(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief bitwise OR
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		bitwise or of both input arguments
 *
 * This function will calculate the bitwise or.
 * result = _a | _b
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_or(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief bitwise XOR
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		bitwise xor of both input arguments
 *
 * This function will calculate the bitwise xor.
 * result = _a ^ _b
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_xor(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief bitwise inverse
 *
 * @param[in] _a	first argument
 *
 * @return		bitwise inverse of both input arguments
 *
 * This function will calculate the bitwise inverse.
 * result = ~_a
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_inv(
    const tvector2w     _a);

/* Additive */

/** @brief addition
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		sum of both input arguments
 *
 * This function will calculate the sum of the input arguments.
 * in case of overflow it will wrap around.
 * result = _a + _b
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_add(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief substraction
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_b substracted from _a.
 *
 * This function will substract _b from _a.
 * in case of overflow it will wrap around.
 * result = _a - _b
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_sub(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief saturated addition
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		saturated sum of both input arguments
 *
 * This function will calculate the sum of the input arguments.
 * in case of overflow it will saturate
 * result = CLIP(_a + _b, MIN_RANGE, MAX_RANGE);
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_addsat(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief saturated substraction
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		saturated substraction of both input arguments
 *
 * This function will substract _b from _a.
 * in case of overflow it will saturate
 * result = CLIP(_a - _b, MIN_RANGE, MAX_RANGE);
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_subsat(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief substraction with shift right
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		(a - b) >> 1
 *
 * This function will substract _b from _a.
 * and right shift the result with 1 bit.
 * result = (_a - _b) >> 1
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_subasr1(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief saturated substraction
 *
 * @param[in] _a	input
 *
 * @return		absolute value of the input
 *
 * This function will calculate the absolute value of the input
 * if (_a > 0) return _a;<br>
 * else return -_a;<br>
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_abs(
    const tvector2w     _a);

/** @brief subabs
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		abs(a-b);
 *
 * This function will calculate the absolute value
 * of the difference of both inputs
 * result = abs(_a - _b);
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_subabs(
    const tvector2w     _a,
    const tvector2w     _b);

/* Multiplicative */

/** @brief integer multiply
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		product of _a and _b
 *
 * This function will calculate the product
 * of the input arguments and returns the LSB
 * aligned single precison result.
 * result = _a * _b;
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_mul(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief fractional multiply
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		product of _a and _b
 *
 * This function will calculate the product
 * of the input arguments and returns the MSB
 * aligned double precison result.
 * result =((_a * _b) << 1) >> (2*NUM_BITS);
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_qmul(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief fractional multiply with rounding
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		product of _a and _b
 *
 * This function will calculate the fixed point
 * product of the input arguments
 * and returns a double precison result.
 * Depending on the rounding mode of the core
 * it will round to nearest or to nearest even.
 * result = ((_a * _b) << 1) >> (2*NUM_BITS);
 */

STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_qrmul(
    const tvector2w     _a,
    const tvector2w     _b);

/* Comparative */

/** @brief equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a == _b
 *
 * This function will return true if both inputs
 * are equal, and false if not equal.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tflags OP_2w_eq(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief not equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a != _b
 *
 * This function will return false if both inputs
 * are equal, and true if not equal.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tflags OP_2w_ne(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief less or equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a <= _b
 *
 * This function will return true if _a is smaller
 * or equal than _b.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tflags OP_2w_le(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief less then
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a < _b
 *
 * This function will return true if _a is smaller
 * than _b.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tflags OP_2w_lt(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief greater or equal
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a >= _b
 *
 * This function will return true if _a is greater
 * or equal than _b.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tflags OP_2w_ge(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief greater than
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a > _b
 *
 * This function will return true if _a is greater
 * than _b.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tflags OP_2w_gt(
    const tvector2w     _a,
    const tvector2w     _b);

/* Shift */

/** @brief aritmetic shift right
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a >> _b
 *
 * This function will shift _a with _b bits to the right.
 * preserving the sign bit.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_asr(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief aritmetic shift right with rounding
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a >> _b
 *
 * This function will shift _a with _b bits to the right.
 * and depending on the rounding mode of the core
 * it will round to nearest or to nearest even.
 *
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_asrrnd(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief aritmetic shift left
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a << _b
 *
 * This function will shift _a with _b bits to the left.
 *
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_asl(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief saturating aritmetic shift left
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a << _b
 *
 * This function will shift _a with _b bits to the left.
 *
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_aslsat(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief logical shift left
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a << _b
 *
 * This function will shift _a with _b bits to the left.
 * It will insert zero's on the right.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_lsl(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief logical shift right
 *
 * @param[in] _a	input
 * @param[in] _b	shift amount
 *
 * @return		_a >> _b
 *
 * This function will shift _a with _b bits to the right.
 * It will insert zero's on the left
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_lsr(
    const tvector2w     _a,
    const tvector2w     _b);

/* clipping */

/** @brief Clip asymetrical
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a clipped between ~_b and b
 *
 * This function will clip the first argument between
 * the negated version of _b and _b.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_clip_asym(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief Clip zero
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a clipped beteween 0 and _b
 *
 * This function will clip the first argument between
 * zero and _b.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_clipz(
    const tvector2w     _a,
    const tvector2w     _b);

/* division */

/** @brief Divide
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a / _b
 *
 * This function will divide the first argument by
 * the second argument.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_div(
    const tvector2w     _a,
    const tvector2w     _b);
/** @brief Divide
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a / _b
 *
 * This function will perform halving division of
 * the first argument by the second argument.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector1w OP_2w_divh(
    const tvector2w     _a,
    const tvector1w     _b);

/** @brief Modulo
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		_a % _b
 *
 * This function will return _a modulo _b.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_mod(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief Square root
 *
 * @param[in] _a	input
 *
 * @return		square root of _a
 *
 * This function will calculate the square root of _a
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_sqrt(
    const tvector2w     _a);

/* Miscellaneous */

/** @brief Multiplexer
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 * @param[in] _c	condition
 *
 * @return		_c ? _a : _b
 *
 * This function will return _a if the condition _c
 * is true and _b otherwise.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_mux(
    const tvector2w     _a,
    const tvector2w     _b,
    const tflags           _c);

/** @brief Average
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		average(_a,_b)
 *
 * This function will calculate the average of
 * the two input arguments.
 * And depending on the rounding mode of the core
 * it will round to nearest or to nearest even.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_avgrnd(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief Minimum
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		(_a < _b) ? _a : _b;
 *
 * This function will return the smallest of both
 * input arguments.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_min(
    const tvector2w     _a,
    const tvector2w     _b);

/** @brief Maximum
 *
 * @param[in] _a	first argument
 * @param[in] _b	second argument
 *
 * @return		(_a > _b) ? _a : _b;
 *
 * This function will return the largest of both
 * input arguments.
 */
STORAGE_CLASS_ISP_OP2W_FUNC_H tvector2w OP_2w_max(
    const tvector2w     _a,
    const tvector2w     _b);

#ifndef INLINE_ISP_OP2W
#define STORAGE_CLASS_ISP_OP2W_FUNC_C
#define STORAGE_CLASS_ISP_OP2W_DATA_C const
#else /* INLINE_ISP_OP2W */
#define STORAGE_CLASS_ISP_OP2W_FUNC_C STORAGE_CLASS_ISP_OP2W_FUNC_H
#define STORAGE_CLASS_ISP_OP2W_DATA_C STORAGE_CLASS_ISP_OP2W_DATA_H
#include "isp_op2w.c"
#define ISP_OP2W_INLINED
#endif  /* INLINE_ISP_OP2W */

#endif /* __ISP_OP2W_H_INCLUDED__ */
