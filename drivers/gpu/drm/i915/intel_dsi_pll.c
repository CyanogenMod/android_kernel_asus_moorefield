/*
 * Copyright © 2013-2013 Intel Corporation
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
 *	Yogesh Mohan Marimuthu <yogesh.mohan.marimuthu@intel.com>
 */

#include <linux/kernel.h>
#include "intel_drv.h"
#include "i915_drv.h"
#include "intel_dsi.h"

struct dsi_clock_table {
	u32 freq;
	u8 m;
	u8 p;
};

u32 lfsr_converts[] = {
		426, 469, 234, 373, 442, 221, 110, 311, 411,	/* 62 - 70 */
	461, 486, 243, 377, 188, 350, 175, 343, 427, 213,	/* 71 - 80 */
	106, 53, 282, 397, 454, 227, 113, 56, 284, 142,		/* 81 - 90 */
	71, 35							/* 91 - 92 */
};

struct dsi_clock_table dsi_clk_tbl[] = {
		{300, 72, 6}, {313, 75, 6}, {323, 78, 6}, {333, 80, 6},
		{343, 82, 6}, {353, 85, 6}, {363, 87, 6}, {373, 90, 6},
		{383, 92, 6}, {390, 78, 5}, {393, 79, 5}, {400, 80, 5},
		{401, 80, 5}, {402, 80, 5}, {403, 81, 5}, {404, 81, 5},
		{405, 81, 5}, {406, 81, 5}, {407, 81, 5}, {408, 82, 5},
		{409, 82, 5}, {410, 82, 5}, {411, 82, 5}, {412, 82, 5},
		{413, 83, 5}, {414, 83, 5}, {415, 83, 5}, {416, 83, 5},
		{417, 83, 5}, {418, 84, 5}, {419, 84, 5}, {420, 84, 5},
		{430, 86, 5}, {440, 88, 5}, {450, 90, 5}, {460, 92, 5},
		{470, 75, 4}, {480, 77, 4}, {490, 78, 4}, {500, 80, 4},
		{510, 82, 4}, {520, 83, 4}, {530, 85, 4}, {540, 86, 4},
		{550, 88, 4}, {560, 90, 4}, {570, 91, 4}, {580, 70, 3},
		{590, 71, 3}, {600, 72, 3}, {610, 73, 3}, {620, 74, 3},
		{630, 76, 3}, {640, 77, 3}, {650, 78, 3}, {660, 79, 3},
		{670, 80, 3}, {680, 82, 3}, {690, 83, 3}, {700, 84, 3},
		{710, 85, 3}, {720, 86, 3}, {730, 88, 3}, {740, 89, 3},
		{750, 90, 3}, {760, 91, 3}, {770, 92, 3}, {780, 62, 2},
		{790, 63, 2}, {800, 64, 2}, {880, 70, 2}, {900, 72, 2},
		{1000, 80, 2},		/* dsi clock frequency in Mhz*/
};

/* To recalculate the pclk considering dual link and Burst mode */
int intel_drrs_calc_pclk(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *pclk)
{
	int pkt_pixel_size;		/* in bits */

	*pclk = mode->clock;

	pkt_pixel_size = intel_get_bits_per_pixel(intel_dsi);
	if (pkt_pixel_size < 0) {
		DRM_ERROR("Unsupported pixel format\n");
		return pkt_pixel_size;
	}

	/* In dual link mode each port needs half of pixel clock */
	if (intel_dsi->dual_link)
		adjust_pclk_for_dual_link(intel_dsi, mode, pclk);

	/* Retaining the same Burst mode ratio for DRRS. Need to be tested */
	if (intel_dsi->video_mode_type == DSI_VIDEO_BURST)
		adjust_pclk_for_burst_mode(pclk, intel_dsi->burst_mode_ratio);

	DRM_DEBUG_KMS("mode->clock : %d, pclk : %d\n", mode->clock, *pclk);
	return 0;
}

/* Get DSI clock from pixel clock */
int dsi_clk_from_pclk(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *dsi_clk) {
	u32 dsi_bit_clock_hz;
	int pkt_pixel_size;		/* in bits */
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 pclk;
	int ret;

	pkt_pixel_size = intel_get_bits_per_pixel(intel_dsi);
	if (pkt_pixel_size < 0) {
		DRM_ERROR("Unsupported pixel format\n");
		return pkt_pixel_size;
	}

	/* For Acer AUO B080XAT panel, use a fixed DSI data rate of 513 Mbps */
	if (dev_priv->mipi_panel_id == MIPI_DSI_AUO_B080XAT_PANEL_ID) {
		*dsi_clk = 513;
		return 0;
	}

	ret = intel_drrs_calc_pclk(intel_dsi, mode, &pclk);
	if (ret < 0)
		return ret;

	/* DSI data rate = pixel clock * bits per pixel / lane count
	   pixel clock is converted from KHz to Hz */
	dsi_bit_clock_hz = (((pclk * 1000) * pkt_pixel_size) / intel_dsi->lane_count);

	/* return DSI data rate as Mbps */
	*dsi_clk = dsi_bit_clock_hz / (1000 * 1000);
	return 0;
}

int dsi_15percent_formula(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *dsi_clk)
{
	u32 bpp;
	u32 dsi_pixel_clk;

	if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB888)
		bpp = 24;
	else if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB666_LOOSE)
		bpp = 24;
	else
		bpp = 18;

	dsi_pixel_clk = (mode->clock * bpp) /
			(intel_dsi->lane_count * 1000);
	*dsi_clk = /*((dsi_pixel_clk * 15) / 100) + */dsi_pixel_clk;

	return 0;
}

int intel_get_dsi_clk(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode, u32 *dsi_clk)
{

	return dsi_clk_from_pclk(intel_dsi, mode, dsi_clk);
	/*return dsi_15percent_formula(intel_dsi, mode, dsi_clk);*/
}

int mnp_from_clk_table(u32 dsi_clk, struct intel_dsi_mnp *intel_dsi_mnp)
{
	unsigned int i;
	u8 m = 0;
	u8 n = 0;
	u8 p = 0;
	u32 m_seed;

	if (dsi_clk < 300 || dsi_clk > 1000)
		return -ECHRNG;

	for (i = 0; i < sizeof(dsi_clk_tbl)/sizeof(struct dsi_clock_table);
			i++) {
		if (dsi_clk_tbl[i].freq > dsi_clk)
			break;
	}

	if (i == sizeof(dsi_clk_tbl)/sizeof(struct dsi_clock_table))
		return -ECHRNG;

	m = dsi_clk_tbl[i].m;
	p = dsi_clk_tbl[i].p;

	m_seed = lfsr_converts[m - 62];
	n = 1;
	intel_dsi_mnp->m = m_seed;
	intel_dsi_mnp->n = n;
	intel_dsi_mnp->p = p;

	return 0;
}

int dsi_calc_mnp(u32 dsi_clk, struct intel_dsi_mnp *intel_dsi_mnp)
{
	u32 m, n, p;
	u32 ref_clk;
	u32 error;
	u32 tmp_error;
	u32 target_dsi_clk;
	u32 calc_dsi_clk;
	u32 calc_m;
	u32 calc_p;
	u32 m_seed;

	if (dsi_clk < 300 || dsi_clk > 1150) {
		DRM_ERROR("DSI CLK Out of Range\n");
		return -ECHRNG;
	}

	ref_clk = 25000;
	target_dsi_clk = dsi_clk * 1000;
	error = 0xFFFFFFFF;
	tmp_error = 0xFFFFFFFF;
	calc_m = 0;
	calc_p = 0;

	for (m = 62; m <= 92; m++) {
		for (p = 2; p <= 6; p++) {
			/* Find the optimal m and p divisors
			with minimal error +/- the required clock */
			calc_dsi_clk = (m * ref_clk) / p;
			if (calc_dsi_clk == target_dsi_clk) {
				calc_m = m;
				calc_p = p;
				error = 0;
				break;
			} else if (calc_dsi_clk > target_dsi_clk)
				tmp_error = calc_dsi_clk - target_dsi_clk;
			else
				tmp_error = target_dsi_clk - calc_dsi_clk;

			if (tmp_error < error) {
					error = tmp_error;
					calc_m = m;
					calc_p = p;
			}
		}

		if (error == 0)
			break;
	}

	m_seed = lfsr_converts[calc_m - 62];
	n = 1;
	intel_dsi_mnp->m = m_seed;
	intel_dsi_mnp->n = n;
	intel_dsi_mnp->p = calc_p;

	return 0;
}

static void intel_configure_dsi_pll_reg(struct drm_i915_private *dev_priv,
				struct intel_dsi_mnp *intel_dsi_mnp,
				struct intel_dsi *intel_dsi,
						bool divider_update_only)
{
	u32 dsi_pll_ctrl = 0, dsi_pll_div = 0;

	if (divider_update_only) {
		dsi_pll_ctrl = vlv_cck_read(dev_priv, CCK_REG_DSI_PLL_CONTROL);
		dsi_pll_ctrl &= ~DSI_PLL_P1_POST_DIV_MASK;
	} else {
		vlv_cck_write(dev_priv, CCK_REG_DSI_PLL_CONTROL, 0);
		dsi_pll_ctrl |= DSI_PLL_CLK_GATE_DSI0_DSIPLL;
	}

	if (intel_dsi->dual_link)
		dsi_pll_ctrl |= DSI_PLL_CLK_GATE_DSI1_DSIPLL;

	dsi_pll_ctrl |= (1 <<
			(DSI_PLL_P1_POST_DIV_SHIFT + intel_dsi_mnp->p - 2));
	dsi_pll_div = ((intel_dsi_mnp->n - 1) <<
			DSI_PLL_N1_DIV_SHIFT) | intel_dsi_mnp->m;

	DRM_DEBUG_KMS("dsi pll div %08x, ctrl %08x\n",
					dsi_pll_div, dsi_pll_ctrl);

	vlv_cck_write(dev_priv, CCK_REG_DSI_PLL_DIVIDER, dsi_pll_div);
	vlv_cck_write(dev_priv, CCK_REG_DSI_PLL_CONTROL, dsi_pll_ctrl);
}

int intel_drrs_configure_dsi_pll(struct intel_dsi *intel_dsi,
					struct intel_dsi_mnp *intel_dsi_mnp)
{
	struct drm_i915_private *dev_priv =
			intel_dsi->base.base.dev->dev_private;
	struct intel_crtc *intel_crtc =
				to_intel_crtc(intel_dsi->base.base.crtc);
	struct intel_mipi_drrs_work *work =
				dev_priv->drrs.mipi_drrs_work;
	u32 dsi_pll_ctrl, vactive;
	u32 dsl_offset = PIPEDSL(intel_crtc->pipe), dsl;
	unsigned long timeout;

	intel_configure_dsi_pll_reg(dev_priv, intel_dsi_mnp, intel_dsi, true);

	dsi_pll_ctrl = vlv_cck_read(dev_priv, CCK_REG_DSI_PLL_CONTROL);
	dsi_pll_ctrl &= (~DSI_PLL_VCO_EN);

	vactive = (I915_READ(VTOTAL(intel_crtc->pipe)) &
				VERTICAL_ACTIVE_DISPLAY_MASK) + 1;

	timeout = jiffies + msecs_to_jiffies(50);

	do {
		if (atomic_read(&work->abort_wait_loop) == 1) {
			DRM_DEBUG_KMS("Aborting the pll update\n");
			return -EPERM;
		}

		dsl = (I915_READ(dsl_offset) & DSL_LINEMASK_GEN3);

		if (jiffies >= timeout) {
			DRM_ERROR("Timeout at waiting for Vblank\n");
			return -ETIMEDOUT;
		}
	} while (dsl <= vactive);

	mutex_lock(&dev_priv->dpio_lock);
	/* Toggle the VCO_EN to bring in the new dividers values */
	vlv_cck_write(dev_priv, CCK_REG_DSI_PLL_CONTROL, dsi_pll_ctrl);

	dsi_pll_ctrl |= DSI_PLL_VCO_EN;
	vlv_cck_write(dev_priv, CCK_REG_DSI_PLL_CONTROL, dsi_pll_ctrl);
	mutex_unlock(&dev_priv->dpio_lock);

	DRM_DEBUG_KMS("PLL Changed between DSL: %d, %d\n",
			dsl, I915_READ(dsl_offset) & DSL_LINEMASK_GEN3);

	if (wait_for(I915_READ(PIPECONF(PIPE_A)) & PIPECONF_DSI_PLL_LOCKED,
									20)) {
		DRM_ERROR("DSI PLL lock failed\n");
		return -EACCES;
	}

	return 0;
}

int intel_calculate_dsi_pll_mnp(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode,
			struct intel_dsi_mnp *intel_dsi_mnp, u32 dsi_clk)
{
	int ret;

	if (dsi_clk == 0) {
		ret = intel_get_dsi_clk(intel_dsi, mode, &dsi_clk);
		if (ret < 0)
			return ret;
	}

	return dsi_calc_mnp(dsi_clk, intel_dsi_mnp);
}

int intel_configure_dsi_pll(struct intel_dsi *intel_dsi,
		struct drm_display_mode *mode)
{
	struct drm_i915_private *dev_priv =
			intel_dsi->base.base.dev->dev_private;
	struct drm_encoder *encoder = &(intel_dsi->base.base);
	struct intel_connector *intel_connector = intel_dsi->attached_connector;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_crtc_config *config = &intel_crtc->config;
	int ret;

	DRM_DEBUG_KMS("\n");

	ret = intel_calculate_dsi_pll_mnp(intel_dsi, mode,
					&config->dsi_mnp, intel_dsi->dsi_clock_freq);
	if (ret < 0)
		return ret;

	/* In case of DRRS, Calculating the divider values for downclock_mode */
	if (intel_connector->panel.downclock_avail &&
		dev_priv->drrs_state.type >= SEAMLESS_DRRS_SUPPORT) {
		ret = intel_calculate_dsi_pll_mnp(intel_dsi,
			intel_connector->panel.downclock_mode, &config->dsi_mnp2, 0);
		if (ret < 0)
			return ret;
	}


	intel_configure_dsi_pll_reg(dev_priv, &config->dsi_mnp, intel_dsi, false);


	return 0;
}

int intel_enable_dsi_pll(struct intel_dsi *intel_dsi)
{
	struct drm_encoder *encoder = &(intel_dsi->base.base);
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct drm_display_mode *mode = &intel_crtc->config.adjusted_mode;
	struct drm_i915_private *dev_priv =
					intel_dsi->base.base.dev->dev_private;
	u32 tmp;

	DRM_DEBUG_KMS("\n");
	if ((I915_READ(PIPECONF(PIPE_A)) & PIPECONF_DSI_PLL_LOCKED)) {
		DRM_DEBUG_KMS("DSI PLL Already locked\n");
		return 0;
	}

	mutex_lock(&dev_priv->dpio_lock);
	intel_configure_dsi_pll(intel_dsi, mode);

	/* wait at least 0.5 us after ungating before enabling VCO */
	usleep_range(1, 10);

	tmp = vlv_cck_read(dev_priv, CCK_REG_DSI_PLL_CONTROL);
	tmp |= DSI_PLL_VCO_EN;
	/* enable DPLL ref clock */
	I915_WRITE_BITS(_DPLL_A, DPLL_REFA_CLK_ENABLE_VLV,
						DPLL_REFA_CLK_ENABLE_VLV);
	udelay(1000);
	vlv_cck_write(dev_priv, CCK_REG_DSI_PLL_CONTROL, tmp);

	mutex_unlock(&dev_priv->dpio_lock);

	tmp = vlv_cck_read(dev_priv, CCK_REG_DSI_PLL_CONTROL);

	if (tmp & 0x1)
		DRM_DEBUG_KMS("DSI PLL Locked\n");
	else
		DRM_DEBUG_DRIVER("DSI PLL Lock failed\n");
	return 0;
}

int intel_disable_dsi_pll(struct intel_dsi *intel_dsi)
{
	struct drm_i915_private *dev_priv = intel_dsi->base.base.dev->dev_private;
	u32 tmp;

	DRM_DEBUG_KMS("\n");

	mutex_lock(&dev_priv->dpio_lock);
	tmp = vlv_cck_read(dev_priv, CCK_REG_DSI_PLL_CONTROL);
	tmp &= ~DSI_PLL_VCO_EN;
	tmp |= DSI_PLL_LDO_GATE;
	vlv_cck_write(dev_priv, CCK_REG_DSI_PLL_CONTROL, tmp);
	if ((I915_READ(PIPECONF(PIPE_A)) & (PIPECONF_ENABLE == 0)) &&
		(I915_READ(PIPECONF(PIPE_B)) & (PIPECONF_ENABLE == 0)))
		I915_WRITE_BITS(_DPLL_A, 0x00000000, DPLL_REFA_CLK_ENABLE_VLV);

	mutex_unlock(&dev_priv->dpio_lock);

	return 0;
}
