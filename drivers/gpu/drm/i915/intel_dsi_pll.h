/*
 * Copyright © 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including withouti limitation
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
 * Author: Shobhit Kumar <shobhit.kumar@intel.com>  */

#ifndef _INTEL_DSI_PLL_H
#define _INTEL_DSI_PLL_H

int intel_calculate_dsi_pll_mnp(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode,
			struct intel_dsi_mnp *intel_dsi_mnp, u32 dsi_clk);
int intel_drrs_configure_dsi_pll(struct intel_dsi *intel_dsi,
			struct intel_dsi_mnp *intel_dsi_mnp);
int intel_configure_dsi_pll(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode);
int intel_enable_dsi_pll(struct intel_dsi *intel_dsi);
int intel_disable_dsi_pll(struct intel_dsi *intel_dsi);
int intel_get_dsi_clk(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *dsi_clk);
#endif /* _INTEL_DSI_H */
