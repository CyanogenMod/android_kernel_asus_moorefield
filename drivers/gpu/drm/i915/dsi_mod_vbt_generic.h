/*
 * Copyright (C) 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 *	Shobhit Kumar <shobhit.kumar@intel.com>
 */
#ifndef __DSI_MOD_VBT_GENERIC_H__
#define __DSI_MOD_VBT_GENERIC_H__

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>
#include "intel_drv.h"

#define MIPI_TRANSFER_MODE_SHIFT	0
#define MIPI_VIRTUAL_CHANNEL_SHIFT	1
#define MIPI_PORT_SHIFT			3

#define MIPI_GENERIC_SHORT_WRITE_0_PARAM	0x03
#define MIPI_GENERIC_SHORT_WRITE_1_PARAM	0x13
#define MIPI_GENERIC_SHORT_WRITE_2_PARAM	0x23
#define MIPI_GENERIC_READ_0_PARAM		0x04
#define MIPI_GENERIC_READ_1_PARAM		0x14
#define MIPI_GENERIC_READ_2_PARAM		0x24
#define MIPI_GENERIC_LONG_WRITE			0x29
#define MIPI_MAN_DCS_SHORT_WRITE_0_PARAM	0x05
#define MIPI_MAN_DCS_SHORT_WRITE_1_PARAM	0x15
#define MIPI_MAN_DCS_READ_0_PARAM		0x06
#define MIPI_MAN_DCS_LONG_WRITE			0x39

#define NS_MHZ_RATIO 1000000

#define PREPARE_CNT_MAX		0x3F
#define EXIT_ZERO_CNT_MAX	0x3F
#define CLK_ZERO_CNT_MAX	0xFF
#define TRAIL_CNT_MAX		0x1F

static inline u32 ceil_div(u32 numerator, u32 denominator)
{
	u32 result = numerator / denominator;
	if (numerator % denominator)
		result += 1;

	return result;
}

typedef u8 * (*FN_MIPI_ELEM_EXEC)(struct intel_dsi *intel_dsi, u8 *data);

struct gpio_table {
	u16 function_reg;
	u16 pad_reg;
	u8 init;
};

#endif /* __DSI_MOD_VBT_GENERIC_H__ */
