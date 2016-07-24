/*
 * Copyright © 2013 Intel Corporation
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
 * Author: Jani Nikula <jani.nikula@intel.com>
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#ifdef CONFIG_CRYSTAL_COVE
#include "linux/mfd/intel_mid_pmic.h"
#endif
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "intel_dsi_pll.h"

/* the sub-encoders aka panel drivers */
static const struct intel_dsi_device intel_dsi_devices[] = {
	{
		.panel_id = MIPI_DSI_GENERIC_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "vbt-generic-dsi-vid-mode-display",
		.dev_ops = &vbt_generic_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_AUO_B101UAN01_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-b101uan01-dsi-vid-mode-display",
		.dev_ops = &auo_b101uan01_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_AUO_B080XAT_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-b080xat-dsi-vid-mode-display",
		.dev_ops = &auo_b080xat_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "auo-panasonic-dsi-vid-mode-display",
		.dev_ops = &panasonic_vvx09f006a00_dsi_display_ops,
	},
	{
		.panel_id = MIPI_DSI_JDI_LPM070W425B_PANEL_ID,
		.type = INTEL_DSI_VIDEO_MODE,
		.name = "jdi-lpm070w425b-dsi-vid-mode-display",
		.dev_ops = &jdi_lpm070w425b_dsi_display_ops,
	},
};

static struct intel_dsi *intel_attached_dsi(struct drm_connector *connector)
{
	return container_of(intel_attached_encoder(connector),
			    struct intel_dsi, base);
}

static inline bool is_vid_mode(struct intel_dsi *intel_dsi)
{
	return intel_dsi->dev.type == INTEL_DSI_VIDEO_MODE;
}

static inline bool is_cmd_mode(struct intel_dsi *intel_dsi)
{
	return intel_dsi->dev.type == INTEL_DSI_COMMAND_MODE;
}

static void intel_dsi_hot_plug(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");
}

int intel_get_bits_per_pixel(struct intel_dsi *intel_dsi)
{
	int bits_per_pixel;		/* in bits */

	if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB888)
		bits_per_pixel = 24;
	else if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB666_LOOSE)
		bits_per_pixel = 24;
	else if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB666)
		bits_per_pixel = 18;
	else if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB565)
		bits_per_pixel = 16;
	else
		return -ECHRNG;

	return bits_per_pixel;
}

void adjust_pclk_for_dual_link(struct intel_dsi *intel_dsi,
				struct drm_display_mode *mode, u32 *pclk)
{
	struct drm_device *dev = intel_dsi->base.base.dev;

	/* In dual link mode each port needs half of pixel clock */
	*pclk = *pclk / 2;

	/* in case of C0 and above setting we can enable pixel_overlap
	 * if needed by panel. In this case we need to increase the pixel
	 * clock for extra pixels
	 */
	if (IS_VALLEYVIEW_C0(dev) && (intel_dsi->dual_link &
					MIPI_DUAL_LINK_FRONT_BACK)) {
		*pclk += DIV_ROUND_UP(mode->vtotal * intel_dsi->pixel_overlap *
							mode->vrefresh, 1000);
	}
}

void adjust_pclk_for_burst_mode(u32 *pclk, u16 burst_mode_ratio)
{
	*pclk = DIV_ROUND_UP(*pclk * burst_mode_ratio, 100);
}

static bool intel_dsi_compute_config(struct intel_encoder *encoder,
				     struct intel_crtc_config *config)
{
	struct intel_dsi *intel_dsi = container_of(encoder, struct intel_dsi,
						   base);
	struct intel_connector *intel_connector = intel_dsi->attached_connector;
	struct drm_display_mode *fixed_mode = intel_connector->panel.fixed_mode;
	struct drm_display_mode *adjusted_mode = &config->adjusted_mode;
	struct drm_display_mode *mode = &config->requested_mode;
	struct intel_crtc *intel_crtc = encoder->new_crtc;
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_KMS("\n");

	if (fixed_mode)
		intel_fixed_panel_mode(fixed_mode, adjusted_mode);

	/* Panel native resolution and desired mode can be different in
	these two cases:
	1. Generic driver specifies scaling reqd flag.
	2. Static driver for Panasonic panel with BYT_CR

	Fixme: Remove static driver's panel ID check as we are planning to
	enable generic driver by default */
	if ((dev_priv->scaling_reqd) ||
		(BYT_CR_CONFIG && (i915_mipi_panel_id ==
		MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID)))  {
		intel_connector->panel.fitting_mode = AUTOSCALE;
		DRM_DEBUG_DRIVER("Enabling panel fitter as scaling required flag set\n");
	}

	if (IS_VALLEYVIEW(dev)) {
		intel_gmch_panel_fitting(intel_crtc, config,
			intel_connector->panel.fitting_mode);
	}

	if (intel_dsi->dev.dev_ops->mode_fixup)
		return intel_dsi->dev.dev_ops->mode_fixup(&intel_dsi->dev,
							  mode, adjusted_mode);

	config->dither = config->pipe_bpp == 18 ? 1 : 0;

	return true;

}

static void intel_dsi_pre_pll_enable(struct intel_encoder *encoder)
{
	DRM_DEBUG_KMS("\n");

	/* nothing to do here as we do necessary stuff in pre_enable
	 * to comply to the hw team recommended DSI sequence */
}

static void band_gap_reset(struct drm_i915_private *dev_priv)
{
	mutex_lock(&dev_priv->dpio_lock);

	intel_flisdsi_write32(dev_priv, 0x08, 0x0001);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0005);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0025);
	udelay(150);
	intel_flisdsi_write32(dev_priv, 0x0F, 0x0000);
	intel_flisdsi_write32(dev_priv, 0x08, 0x0000);

	mutex_unlock(&dev_priv->dpio_lock);
}

static void intel_dsi_power_on(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (BYT_CR_CONFIG) {
		/*  cabc disable */
		vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL1_VDDEN_GPIONC_9_PCONF0, VLV_GPIO_CFG);
		vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL1_VDDEN_GPIONC_9_PAD, VLV_GPIO_INPUT_DIS);

		/* panel enable */
		vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL1_BKLTCTL_GPIONC_11_PCONF0, VLV_GPIO_CFG);
		vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL1_BKLTCTL_GPIONC_11_PAD, VLV_GPIO_INPUT_EN);
		udelay(500);
	} else
		intel_mid_pmic_writeb(PMIC_PANEL_EN, 0x01);
}

void intel_dsi_device_ready(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 tmp;
	int count = 1;

	DRM_DEBUG_KMS("\n");

	/* program rcomp for compliance
	 * reduce form 50 ohms to 45 ohms */
	intel_flisdsi_write32(dev_priv, 0x04, 0x0004);

	band_gap_reset(dev_priv);

	if (intel_dsi->dev.dev_ops->power_on)
		intel_dsi->dev.dev_ops->power_on(&intel_dsi->dev);

	msleep(intel_dsi->panel_on_delay);

	if (intel_dsi->dev.dev_ops->panel_reset)
		intel_dsi->dev.dev_ops->panel_reset(&intel_dsi->dev);

	/* Disable DPOunit clock gating, can stall pipe */
	tmp = I915_READ(DPLL(pipe));
	tmp |= DPLL_RESERVED_BIT;
	I915_WRITE(DPLL(pipe), tmp);

	tmp = I915_READ(DSPCLK_GATE_D);
	tmp |= VSUNIT_CLOCK_GATE_DISABLE;
	I915_WRITE(DSPCLK_GATE_D, tmp);

	intel_enable_dsi_pll(intel_dsi);
	if (intel_dsi->operation_mode == DSI_VIDEO_MODE) {

		I915_WRITE_BITS(MIPI_PORT_CTRL(pipe), LP_OUTPUT_HOLD,
							LP_OUTPUT_HOLD);

		usleep_range(1000, 1500);

		if (intel_dsi->dual_link)
			count = 2;
		do {

			I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), DEVICE_READY |
					ULPS_STATE_EXIT, DEVICE_READY |
					ULPS_STATE_MASK);

			usleep_range(2000, 2500);
			I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), DEVICE_READY,
					DEVICE_READY | ULPS_STATE_MASK);
			usleep_range(2000, 2500);
			I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), 0x00,
					DEVICE_READY | ULPS_STATE_MASK);
			usleep_range(2000, 2500);
			I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), DEVICE_READY,
					DEVICE_READY | ULPS_STATE_MASK);
			usleep_range(2000, 2500);
			/* For Port C for dual link */
			pipe = PIPE_B;
		} while (--count > 0);
	}
}

void intel_dsi_port_enable(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 val, port_control = 0;

	if (intel_dsi->dual_link) {
		port_control = (intel_dsi->dual_link - 1)
						<< DUAL_LINK_MODE_SHIFT;
		if (pipe == PIPE_A)
			port_control |= LANE_CONFIGURATION_DUAL_LINK_A;
		else
			port_control |= LANE_CONFIGURATION_DUAL_LINK_B;

		/*Pixel overlap count; only for VLV CO stepping */
		if (IS_VALLEYVIEW_C0(dev) && (intel_dsi->dual_link & MIPI_DUAL_LINK_FRONT_BACK)) {
			val = I915_READ(VLV_CHICKEN_3);
			val &= ~PIXEL_OVERLAP_CNT_MASK |
				intel_dsi->pixel_overlap <<
				PIXEL_OVERLAP_CNT_SHIFT;
			I915_WRITE(VLV_CHICKEN_3, val);
		}

		/* Port A */
		val = I915_READ(MIPI_PORT_CTRL(0));
		val = val | port_control;
		I915_WRITE(MIPI_PORT_CTRL(0), val | DPI_ENABLE);

		if (!IS_VALLEYVIEW_C0(dev)) {
			/* for stepping before C0; we need to enable
			* PORTC explicitly. From C0 onwards enable PORT A
			* also enabled PORT C for dual link
			*/
			val = I915_READ(MIPI_PORT_CTRL(1));
			I915_WRITE(MIPI_PORT_CTRL(1), val | DPI_ENABLE);
		} else {
			if (intel_crtc->config.dither) {
				val = I915_READ(MIPI_PORT_CTRL(0));
				val = val | DITHERING_ENABLE;
				I915_WRITE(MIPI_PORT_CTRL(0), val);
				val = I915_READ(MIPI_PORT_CTRL(1));
				val = val | DITHERING_ENABLE;
				I915_WRITE(MIPI_PORT_CTRL(1), val);
			}
		}
		usleep_range(2000, 2500);
	} else {
		val = I915_READ(MIPI_PORT_CTRL(pipe));
		val = val | port_control;
		if (intel_crtc->config.dither && IS_VALLEYVIEW_C0(dev))
			val |= DITHERING_ENABLE;
		I915_WRITE(MIPI_PORT_CTRL(pipe), val | DPI_ENABLE);
		usleep_range(2000, 2500);
	}

}

void intel_dsi_port_disable(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;

	if (intel_dsi->dual_link) {
		I915_WRITE_BITS(MIPI_PORT_CTRL(0), 0, DPI_ENABLE);
		POSTING_READ(MIPI_PORT_CTRL(0));

		I915_WRITE_BITS(MIPI_PORT_CTRL(1), 0, DPI_ENABLE);
		POSTING_READ(MIPI_PORT_CTRL(1));
	} else {
		I915_WRITE_BITS(MIPI_PORT_CTRL(pipe), 0, DPI_ENABLE);
		POSTING_READ(MIPI_PORT_CTRL(pipe));
	}

	usleep_range(2000, 2500);
}

static void intel_dsi_pre_enable(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	DRM_DEBUG_KMS("\n");

	intel_enable_dsi_pll(intel_dsi);

	if (is_cmd_mode(intel_dsi)) {
		/* XXX: Implement me */
		I915_WRITE(MIPI_MAX_RETURN_PKT_SIZE(pipe), 8 * 4);
	}
	else {
		intel_dsi->hs = 0;
		dpi_send_cmd(intel_dsi, TURN_ON);
		usleep_range(1000, 1500);
		if (intel_dsi->dev.dev_ops->enable)
			intel_dsi->dev.dev_ops->enable(&intel_dsi->dev);

		intel_dsi_port_enable(encoder);
	}
}

static void intel_dsi_enable(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;

	DRM_DEBUG_KMS("\n");

	/* Adjust backlight timing for specific panel */
	if (intel_dsi->backlight_on_delay >= 20)
		msleep(intel_dsi->backlight_on_delay);
	else
		usleep_range(intel_dsi->backlight_on_delay * 1000,
			(intel_dsi->backlight_on_delay * 1000) + 500);

	intel_panel_enable_backlight(dev, pipe);

	if (intel_dsi->dev.dev_ops->enable_backlight)
		intel_dsi->dev.dev_ops->enable_backlight(&intel_dsi->dev);
}

static void intel_dsi_disable(struct intel_encoder *encoder)
{
	struct drm_device *dev = encoder->base.dev;
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);

	DRM_DEBUG_KMS("\n");

	intel_panel_disable_backlight(dev);
	if (intel_dsi->backlight_off_delay >= 20)
		msleep(intel_dsi->backlight_off_delay);
	else
		usleep_range(intel_dsi->backlight_off_delay * 1000,
			(intel_dsi->backlight_off_delay * 1000) + 500);

	if (is_cmd_mode(intel_dsi)) {
		/* XXX Impementation TBD */
	} else {
		/* Send Shutdown command to the panel in LP mode */
		intel_dsi->hs = 0;
		dpi_send_cmd(intel_dsi, SHUTDOWN);
		usleep_range(1000, 1500);

	}
}

static void intel_dsi_power_off(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (BYT_CR_CONFIG) {
		/* Disable Panel */
		vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL1_BKLTCTL_GPIONC_11_PCONF0, VLV_GPIO_CFG);
		vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL1_BKLTCTL_GPIONC_11_PAD, VLV_GPIO_INPUT_DIS);
		udelay(500);
	} else
		intel_mid_pmic_writeb(PMIC_PANEL_EN, 0x00);
}

void intel_dsi_clear_device_ready(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 val;
	int count = 1;

	DRM_DEBUG_KMS("\n");

	if (intel_dsi->dual_link)
		count = 2;

	do {
		I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), ULPS_STATE_ENTER | DEVICE_READY,
								ULPS_STATE_MASK | DEVICE_READY);
		usleep_range(2000, 2500);

		I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), ULPS_STATE_EXIT | DEVICE_READY,
								ULPS_STATE_MASK | DEVICE_READY);
		usleep_range(2000, 2500);

		I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), ULPS_STATE_ENTER | DEVICE_READY,
								ULPS_STATE_MASK | DEVICE_READY);
		usleep_range(2000, 2500);

		if (pipe == PIPE_A) {
			if (wait_for(((I915_READ(MIPI_PORT_CTRL(0)) & 0x20000)
							== 0x00000), 30))
				DRM_ERROR("DSI LP not going Low\n");

			I915_WRITE_BITS(MIPI_PORT_CTRL(0), 0, LP_OUTPUT_HOLD);
			usleep_range(1000, 1500);
	}

		I915_WRITE_BITS(MIPI_DEVICE_READY(pipe), 0x00, DEVICE_READY);
		usleep_range(2000, 2500);

		pipe = PIPE_B;
	} while (--count > 0);

	intel_disable_dsi_pll(intel_dsi);

	val = I915_READ(DSPCLK_GATE_D);
	val &= ~VSUNIT_CLOCK_GATE_DISABLE;
	I915_WRITE(DSPCLK_GATE_D, val);

	if (intel_dsi->dev.dev_ops->disable_panel_power)
		intel_dsi->dev.dev_ops->disable_panel_power(&intel_dsi->dev);

	if (intel_dsi->dev.dev_ops->power_off)
		intel_dsi->dev.dev_ops->power_off(&intel_dsi->dev);

	msleep(intel_dsi->panel_off_delay);
	msleep(intel_dsi->panel_pwr_cycle_delay);
}

static void intel_dsi_post_disable(struct intel_encoder *encoder)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->base.crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(&encoder->base);
	int pipe = intel_crtc->pipe;
	u32 tmp;
	int count = 1;
	wait_for_dsi_fifo_empty(intel_dsi);
	intel_dsi_port_disable(encoder);

	/* Panel commands can be sent when clock is in LP11 */

	if (intel_dsi->dual_link)
		count = 2;
	do {
		tmp = I915_READ(MIPI_DEVICE_READY(pipe));
		tmp &= ~DEVICE_READY;
		I915_WRITE(MIPI_DEVICE_READY(pipe), tmp);

		tmp = I915_READ(MIPI_CTRL(pipe));
		tmp &= ~ESCAPE_CLOCK_DIVIDER_MASK;
		I915_WRITE(MIPI_CTRL(pipe), tmp |
				intel_dsi->escape_clk_div <<
				ESCAPE_CLOCK_DIVIDER_SHIFT);

		tmp = I915_READ(MIPI_DSI_FUNC_PRG(pipe));
		tmp &= ~VID_MODE_FORMAT_MASK;
		I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), tmp);

		I915_WRITE(MIPI_EOT_DISABLE(pipe), CLOCKSTOP);
		tmp = I915_READ(MIPI_DSI_FUNC_PRG(pipe));
		tmp &= ~VID_MODE_FORMAT_MASK;
		I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), tmp);

		tmp = I915_READ(MIPI_DEVICE_READY(pipe));
		tmp &= DEVICE_READY;
		I915_WRITE(MIPI_DEVICE_READY(pipe), tmp);
		pipe = PIPE_B;
	} while (--count > 0);

	/* if disable packets are sent before sending shutdown packet then in
	* some next enable sequence send turn on packet error is observed */

	if (intel_dsi->dev.dev_ops->disable)
		intel_dsi->dev.dev_ops->disable(&intel_dsi->dev);

	wait_for_dsi_fifo_empty(intel_dsi);
	intel_dsi_clear_device_ready(encoder);
}

static bool intel_dsi_get_hw_state(struct intel_encoder *encoder,
				   enum pipe *pipe)
{
	struct drm_i915_private *dev_priv = encoder->base.dev->dev_private;
	u32 port, func;
	enum pipe p;

	DRM_DEBUG_KMS("\n");

	/* XXX: this only works for one DSI output */
	for (p = PIPE_A; p <= PIPE_B; p++) {
		port = I915_READ(MIPI_PORT_CTRL(p));
		func = I915_READ(MIPI_DSI_FUNC_PRG(p));

		if ((port & DPI_ENABLE) || (func & CMD_MODE_DATA_WIDTH_MASK)) {
			if (I915_READ(MIPI_DEVICE_READY(p)) & DEVICE_READY) {
				*pipe = p;
				return true;
			}
		}
	}

	return false;
}

static void intel_dsi_get_config(struct intel_encoder *encoder,
				 struct intel_crtc_config *pipe_config)
{
	DRM_DEBUG_KMS("\n");

	/* XXX: read flags, set to adjusted_mode */
}

static int intel_dsi_mode_valid(struct drm_connector *connector,
				struct drm_display_mode *mode)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct drm_display_mode *fixed_mode = intel_connector->panel.fixed_mode;
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);

	DRM_DEBUG_KMS("\n");

	if (mode->flags & DRM_MODE_FLAG_DBLSCAN) {
		DRM_DEBUG_KMS("MODE_NO_DBLESCAN\n");
		return MODE_NO_DBLESCAN;
	}

	if (fixed_mode) {
		if (mode->hdisplay > fixed_mode->hdisplay)
			return MODE_PANEL;
		if (mode->vdisplay > fixed_mode->vdisplay)
			return MODE_PANEL;
	}

	return intel_dsi->dev.dev_ops->mode_valid(&intel_dsi->dev, mode);
}


/* return pixels in terms of txbyteclkhs */
static u32 txbyteclkhs(u32 pixels, int bpp, int lane_count,
							int burst_mode_ratio)
{
	/* burst_mode_ratio is multiplied by 100 when calculated to protect
	 * precision so divide by 100 here */

	return DIV_ROUND_UP(pixels * bpp * burst_mode_ratio,
							8 * lane_count * 100);
}

static void set_dsi_timings(struct drm_encoder *encoder,
			    const struct drm_display_mode *mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	unsigned int bpp = intel_crtc->config.pipe_bpp;
	unsigned int lane_count = intel_dsi->lane_count;
	int count = 1;
	u16 mode_hactive;

	u16 hactive, hfp, hsync, hbp, vfp, vsync, vbp;

	hactive = mode->hdisplay;

	hfp = mode->hsync_start - mode->hdisplay;
	hsync = mode->hsync_end - mode->hsync_start;
	hbp = mode->htotal - mode->hsync_end;

	if (intel_dsi->dual_link) {
		hactive /= 2;
		if (IS_VALLEYVIEW_C0(dev) &&
			(intel_dsi->dual_link & MIPI_DUAL_LINK_FRONT_BACK))
			hactive += intel_dsi->pixel_overlap;
		hfp /= 2;
		hsync /= 2;
		hbp /= 2;

		count = 2;
	}

	mode_hactive = hactive;
	vfp = mode->vsync_start - mode->vdisplay;
	vsync = mode->vsync_end - mode->vsync_start;
	vbp = mode->vtotal - mode->vsync_end;

	/* horizontal values are in terms of high speed byte clock */
	hactive = txbyteclkhs(hactive, bpp, lane_count,
					intel_dsi->burst_mode_ratio);
	hfp = txbyteclkhs(hfp, bpp, lane_count,
					intel_dsi->burst_mode_ratio);
	hsync = txbyteclkhs(hsync, bpp, lane_count,
					intel_dsi->burst_mode_ratio);
	hbp = txbyteclkhs(hbp, bpp, lane_count,
					intel_dsi->burst_mode_ratio);

	/* FIXME: Find better way to do this */
	/* For 7x10 panel we need to have BLLP added to active */
	/* Trying to find optimal BLLP Multiplier */
	/*	2.875 - Original multiplier, Works with flicker */
	/*	2.000 - works but still some flicker */
	/*	1.500 - Works, No Flicker */
	/*	1.250 - Works, No Flicker */
	/*	1.100 - Doesn't work */
	/* FIXME: Acer Mango spec requires to run the DSI clock at 500 to
	 * 560Mbps. Recomendation is to run at 513 Mbps. The addition dsi
	 * clock is to be filled with NULL packets. Refer to acer panel
	 * spec for more details.
	 */
	if (dev_priv->mipi_panel_id == MIPI_DSI_AUO_B080XAT_PANEL_ID)
		hactive = (hactive * 10) / 8;

	do {

		I915_WRITE(MIPI_HACTIVE_AREA_COUNT(pipe), hactive);
		I915_WRITE(MIPI_HFP_COUNT(pipe), hfp);

		/* meaningful for video mode non-burst sync pulse mode only,
		 * can be zero for non-burst sync events and burst modes */
		I915_WRITE(MIPI_HSYNC_PADDING_COUNT(pipe), hsync);
		I915_WRITE(MIPI_HBP_COUNT(pipe), hbp);

		/* vertical values are in terms of lines */
		I915_WRITE(MIPI_VFP_COUNT(pipe), vfp);
		I915_WRITE(MIPI_VSYNC_PADDING_COUNT(pipe), vsync);
		I915_WRITE(MIPI_VBP_COUNT(pipe), vbp);

		I915_WRITE(MIPI_DPI_RESOLUTION(pipe),
			(mode->vdisplay << VERTICAL_ADDRESS_SHIFT) |
			(mode_hactive << HORIZONTAL_ADDRESS_SHIFT));
		pipe = PIPE_B;
	} while (--count > 0);
}

static void dsi_config(struct drm_encoder *encoder)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int pipe = intel_crtc->pipe;
	u32 tmp;
	int count = 1;

	DRM_DEBUG_KMS("\n");

	if (intel_dsi->dual_link)
		count = 2;

	do {
		/* escape clock divider, 20MHz, shared for A and C. device ready must be
		 * off when doing this! txclkesc? */
		tmp = I915_READ(MIPI_CTRL(0));
		tmp &= ~ESCAPE_CLOCK_DIVIDER_MASK;
		I915_WRITE(MIPI_CTRL(0), tmp | ESCAPE_CLOCK_DIVIDER_1);

		/* read request priority is per pipe */
		tmp = I915_READ(MIPI_CTRL(pipe));
		tmp &= ~READ_REQUEST_PRIORITY_MASK;
		I915_WRITE(MIPI_CTRL(pipe), tmp | READ_REQUEST_PRIORITY_HIGH);

		/* XXX: why here, why like this? handling in irq handler?! */
		I915_WRITE(MIPI_INTR_EN(pipe), 0xffffffff);

		/* why here, was elsewhere... also 2a, 0c, 60, 08 for values */
		I915_WRITE(MIPI_DPHY_PARAM(pipe), intel_dsi->dphy_reg);

		pipe = PIPE_B;
	} while (--count > 0);
}

static void intel_dsi_mode_set(struct intel_encoder *intel_encoder)
{
	struct drm_encoder *encoder = &intel_encoder->base;
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	int pipe = intel_crtc->pipe;
	unsigned int bpp = intel_crtc->config.pipe_bpp;
	struct drm_display_mode *adjusted_mode = &intel_crtc->config.adjusted_mode;
	u32 val, count = 1;

	if (intel_dsi->dual_link)
		count = 2;

	do {
		intel_dsi_device_ready(intel_encoder);

		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);

		dsi_config(encoder);

		I915_WRITE(MIPI_LP_RX_TIMEOUT(pipe),
						intel_dsi->lp_rx_timeout);
		I915_WRITE(MIPI_TURN_AROUND_TIMEOUT(pipe),
						intel_dsi->turn_arnd_val);
		I915_WRITE(MIPI_DEVICE_RESET_TIMER(pipe),
						intel_dsi->rst_timer_val);
		/* in terms of low power clock */
		I915_WRITE(MIPI_INIT_COUNT(pipe), intel_dsi->init_count);

		I915_WRITE(MIPI_HIGH_LOW_SWITCH_COUNT(pipe),
						intel_dsi->hs_to_lp_count);
		I915_WRITE(MIPI_LP_BYTECLK(pipe), intel_dsi->lp_byte_clk);

		I915_WRITE(MIPI_CLK_LANE_SWITCH_TIME_CNT(pipe),
			((u32)intel_dsi->clk_lp_to_hs_count
			<< LP_HS_SSW_CNT_SHIFT) |
			(intel_dsi->clk_hs_to_lp_count << HS_LP_PWR_SW_CNT_SHIFT));

		if (is_vid_mode(intel_dsi)) {

			if (intel_dsi->video_mode_type == DSI_VIDEO_BURST) {
				I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
					txbyteclkhs(adjusted_mode->htotal, bpp,
					intel_dsi->lane_count,
					intel_dsi->burst_mode_ratio) + 1);
			} else {
				I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
				txbyteclkhs(adjusted_mode->vtotal *
				adjusted_mode->htotal,
				bpp, intel_dsi->lane_count,
				intel_dsi->burst_mode_ratio) + 1);
			}
		} else {
			val = intel_dsi->channel << CMD_MODE_CHANNEL_NUMBER_SHIFT |
				intel_dsi->lane_count << DATA_LANES_PRG_REG_SHIFT |
				intel_dsi->data_width;
			I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

			I915_WRITE(MIPI_HS_TX_TIMEOUT(pipe),
				txbyteclkhs(adjusted_mode->hdisplay *
				adjusted_mode->vdisplay,
				bpp, intel_dsi->lane_count,
				intel_dsi->burst_mode_ratio) + 1);

			I915_WRITE(MIPI_DBI_BW_CTRL(pipe), intel_dsi->bw_timer);
		}

		I915_WRITE(MIPI_EOT_DISABLE(pipe), CLOCKSTOP);

		val = I915_READ(MIPI_DSI_FUNC_PRG(pipe));
		val &= ~VID_MODE_FORMAT_MASK;
		I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);

		pipe = PIPE_B;
	} while (--count > 0);

	if (intel_dsi->dev.dev_ops->send_otp_cmds)
		intel_dsi->dev.dev_ops->send_otp_cmds(&intel_dsi->dev);

	set_dsi_timings(encoder, adjusted_mode);

	if (intel_dsi->dual_link)
		count = 2;
	else
		count = 1;

	pipe = PIPE_A;

	do {

		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);

		/* Some panels might have resolution which is not a multiple of
		 * 64 like 1366 x 768. Enable RANDOM resolution support for such
		 * panels by default */
		I915_WRITE(MIPI_VIDEO_MODE_FORMAT(pipe),
					intel_dsi->video_frmt_cfg_bits |
					intel_dsi->video_mode_type |
					IP_TG_CONFIG |
					RANDOM_DPI_DISPLAY_RESOLUTION);

		val = 0;
		if (intel_dsi->eotp_pkt == 0)
			val |= EOT_DISABLE;

		if (intel_dsi->clock_stop)
			val |= CLOCKSTOP;

		I915_WRITE(MIPI_EOT_DISABLE(pipe), val);

		val = intel_dsi->channel << VID_MODE_CHANNEL_NUMBER_SHIFT |
			intel_dsi->lane_count << DATA_LANES_PRG_REG_SHIFT |
			intel_dsi->pixel_format;
		I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

		I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);

		I915_WRITE(MIPI_INTR_STAT(pipe), 0xFFFFFFFF);

		I915_WRITE(MIPI_INTR_STAT(pipe), 0xFFFFFFFF);
		/* Max packet return size limits the size of returning
		 * packet so that host processor can prevent buffer overflow
		 * condition when receiving data from peripheral. DCS read
		 * need this to be set.*/
		I915_WRITE(MIPI_MAX_RETURN_PKT_SIZE(pipe), 0xff);

		pipe = PIPE_B;
	} while (--count > 0);
}

static enum drm_connector_status
intel_dsi_detect(struct drm_connector *connector, bool force)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	DRM_DEBUG_KMS("\n");

	return intel_dsi->dev.dev_ops->detect(&intel_dsi->dev);
}

static int intel_dsi_get_modes(struct drm_connector *connector)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_display_mode *mode;
	struct drm_display_mode *input_mode = NULL;
	int count = 0;
	DRM_DEBUG_KMS("\n");

	if (!intel_connector->panel.fixed_mode) {
		DRM_DEBUG_KMS("no fixed mode\n");
		return count;
	}

	input_mode = intel_connector->panel.fixed_mode;
	mode = drm_mode_duplicate(connector->dev,
				  input_mode);
	if (!mode) {
		DRM_DEBUG_KMS("drm_mode_duplicate failed\n");
		return count;
	}

	drm_mode_probed_add(connector, mode);
	count++;

	if (intel_connector->panel.downclock_mode) {
		mode = drm_mode_duplicate(connector->dev,
				intel_connector->panel.downclock_mode);
		if (!mode) {
			DRM_DEBUG_KMS("drm_mode_duplicate failed\n");
			return count;
		}

		drm_mode_probed_add(connector, mode);
		count++;
	}

	/*Fill the panel info here*/
	intel_dsi->dev.dev_ops->get_info(0, connector);
	return count;
}

static void intel_dsi_destroy(struct drm_connector *connector)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);

	DRM_DEBUG_KMS("\n");
	intel_panel_fini(&intel_connector->panel);
	intel_panel_destroy_backlight(connector->dev);
	drm_sysfs_connector_remove(connector);
	drm_connector_cleanup(connector);
	kfree(connector);
}

static int intel_dsi_set_property(struct drm_connector *connector,
		struct drm_property *property,
		uint64_t val)
{
	struct intel_dsi *intel_dsi = intel_attached_dsi(connector);
	struct drm_i915_private *dev_priv = connector->dev->dev_private;
	struct intel_connector *intel_connector = to_intel_connector(connector);
	struct intel_encoder *encoder = intel_connector->encoder;
	struct intel_crtc *intel_crtc = encoder->new_crtc;
	int ret;

	ret = drm_object_property_set_value(&connector->base, property, val);
	if (ret)
		return ret;

	if (property == dev_priv->force_pfit_property) {

		if (intel_connector->panel.fitting_mode == val)
			return 0;

		intel_connector->panel.fitting_mode = val;

		if (IS_VALLEYVIEW(dev_priv->dev)) {

			/* In case of BYT_CR platform with the panasonic panel of
			 * resolution 19x10, panel fitter needs to be enabled always
			 * becoz we simulate the 12x8 mode due to memory limitation
			 */
			if ((dev_priv->scaling_reqd) ||
			(BYT_CR_CONFIG && (i915_mipi_panel_id ==
				MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID))) {
				if (intel_connector->panel.fitting_mode == PFIT_OFF)
					return 0;
			}

			intel_gmch_panel_fitting(intel_crtc, &intel_crtc->config,
				intel_connector->panel.fitting_mode);
			DRM_DEBUG_DRIVER("panel fitting mode = %x", intel_connector->panel.fitting_mode);
			return 0;
		} else
			goto done;
	}

	if (property == dev_priv->scaling_src_size_property) {
		intel_crtc->scaling_src_size = val;
		DRM_DEBUG_DRIVER("src size = %x", intel_crtc->scaling_src_size);
		return 0;
	}
done:
	if (intel_dsi->base.base.crtc)
		intel_crtc_restore_mode(intel_dsi->base.base.crtc);
	return 0;
}

/* Encoder dpms must, add functionality later */
void intel_dsi_encoder_dpms(struct drm_encoder *encoder, int mode)
{
	struct drm_device *dev = encoder->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi = enc_to_intel_dsi(encoder);
	struct intel_encoder *intel_encoder = &intel_dsi->base;
	struct intel_crtc *intel_crtc = to_intel_crtc(encoder->crtc);
	int pipe = intel_crtc->pipe;
	u32 val;
	int count = 1;

	DRM_DEBUG_KMS("\n");

	if (dev_priv->mipi_fw) {
		/* IAFW enabled MIPI; we do not need to
		 * do anything for this dpms call
		 *
		 * Next enabling will be done by driver
		 * so clear the mmipi_fw flag
		 */
		dev_priv->mipi_fw = 0;
		DRM_DEBUG_KMS("FW enabled MIPI nothing to do\n");
		return;
	}

	if (mode != DRM_MODE_DPMS_ON)
		mode = DRM_MODE_DPMS_OFF;

	DRM_DEBUG_DRIVER("DPMS mode = %d\n", mode);

	if (mode == DRM_MODE_DPMS_ON) {
		intel_dsi_device_ready(intel_encoder);

		/* need to resend OTP as panel off was done in
		 * DPMS off */

		/* Clock needs to be in LP11 mode before we can send
		 * commands to panel */

		if (intel_dsi->dual_link)
			count = 2;

		do {

			I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);
			I915_WRITE(MIPI_EOT_DISABLE(pipe), CLOCKSTOP);

			val = I915_READ(MIPI_DSI_FUNC_PRG(pipe));
			val &= ~VID_MODE_FORMAT_MASK;
			I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);

			I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);

			pipe = PIPE_B;
		} while (--count > 0);

		if (intel_dsi->dev.dev_ops->send_otp_cmds)
			intel_dsi->dev.dev_ops->send_otp_cmds(&intel_dsi->dev);

		pipe = PIPE_A;
		if (intel_dsi->dual_link)
			count = 2;
		else
			count = 1;

		do {
			/* Now we need to restore MIPI_DSI_FUNC_PRG to needed value */
			I915_WRITE(MIPI_DEVICE_READY(pipe), 0x0);

			val = intel_dsi->channel <<
						VID_MODE_CHANNEL_NUMBER_SHIFT |
			intel_dsi->lane_count << DATA_LANES_PRG_REG_SHIFT |
			intel_dsi->pixel_format;

			I915_WRITE(MIPI_DSI_FUNC_PRG(pipe), val);
			I915_WRITE(MIPI_DEVICE_READY(pipe), 0x1);
			pipe = PIPE_B;
		} while (--count > 0);
	}
}

static const struct drm_encoder_funcs intel_dsi_funcs = {
	.destroy = intel_encoder_destroy,
};

static const struct drm_encoder_helper_funcs intel_dsi_helper_funcs = {
	.dpms = intel_dsi_encoder_dpms,
};

static const struct drm_connector_helper_funcs intel_dsi_connector_helper_funcs = {
	.get_modes = intel_dsi_get_modes,
	.mode_valid = intel_dsi_mode_valid,
	.best_encoder = intel_best_encoder,
};

static const struct drm_connector_funcs intel_dsi_connector_funcs = {
	.dpms = intel_connector_dpms,
	.detect = intel_dsi_detect,
	.destroy = intel_dsi_destroy,
	.fill_modes = drm_helper_probe_single_connector_modes,
	.set_property = intel_dsi_set_property,
};

static void
intel_dsi_add_properties(struct intel_dsi *intel_dsi,
				struct drm_connector *connector)
{
	intel_attach_force_pfit_property(connector);
	intel_attach_scaling_src_size_property(connector);
}

static void intel_mipi_drrs_work_fn(struct work_struct *__work)
{
	struct intel_mipi_drrs_work *work =
		container_of(to_delayed_work(__work),
			struct intel_mipi_drrs_work, work);
	struct intel_encoder *intel_encoder = work->intel_encoder;
	struct drm_i915_private *dev_priv =
				intel_encoder->base.dev->dev_private;
	struct intel_dsi_mnp *intel_dsi_mnp;
	struct intel_dsi *intel_dsi = NULL;
	struct intel_crtc *intel_crtc = NULL;
	struct drm_display_mode *prev_mode = NULL;
	bool resume_idleness_detection = false, fallback_attempt = false;
	int ret, retry_cnt = 3;

	intel_dsi = enc_to_intel_dsi(&intel_encoder->base);
	intel_crtc = intel_encoder->new_crtc;

init:
	if (work->target_rr_type == DRRS_HIGH_RR) {
		intel_dsi_mnp = &intel_crtc->config.dsi_mnp;
	} else if (work->target_rr_type == DRRS_LOW_RR) {
		intel_dsi_mnp = &intel_crtc->config.dsi_mnp2;
	} else if (work->target_rr_type == DRRS_MEDIA_RR) {
		if (intel_calculate_dsi_pll_mnp(intel_dsi,
				work->target_mode,
				&intel_crtc->config.dsi_mnp3, 0) < 0)
			return;
		intel_dsi_mnp = &intel_crtc->config.dsi_mnp3;
	} else {
		DRM_ERROR("Unknown refreshrate_type\n");
		return;
	}

	if (dev_priv->drrs_state.refresh_rate_type == DRRS_MEDIA_RR &&
			work->target_rr_type == DRRS_HIGH_RR)
		resume_idleness_detection = true;

retry:
	ret = intel_drrs_configure_dsi_pll(intel_dsi, intel_dsi_mnp);
	if (ret == 0) {
		DRM_DEBUG_KMS("cur_rr_type: %d, cur_rr: %d, target_rr_type: %d, target_rr: %d\n",
				dev_priv->drrs_state.refresh_rate_type,
				intel_crtc->base.mode.vrefresh,
				work->target_rr_type, work->target_mode->vrefresh);

		mutex_lock(&dev_priv->drrs_state.mutex);
		dev_priv->drrs_state.refresh_rate_type =
						work->target_rr_type;
		mutex_unlock(&dev_priv->drrs_state.mutex);

		DRM_INFO("Refresh Rate set to : %dHz\n",
						work->target_mode->vrefresh);

		intel_crtc->base.mode.vrefresh = work->target_mode->vrefresh;
		intel_crtc->base.mode.clock = work->target_mode->clock;

		if (resume_idleness_detection)
			intel_update_drrs(intel_encoder->base.dev);
	} else if (ret == -ETIMEDOUT && retry_cnt) {
		retry_cnt--;
		DRM_DEBUG_KMS("Retry left ... <%d>\n", retry_cnt);
		goto retry;
	} else if (ret == -EACCES && !fallback_attempt) {
		DRM_ERROR("Falling back to the previous DRRS state. %d->%d\n",
				work->target_rr_type,
				dev_priv->drrs_state.refresh_rate_type);

		mutex_lock(&dev_priv->drrs_state.mutex);
		dev_priv->drrs_state.target_rr_type =
					dev_priv->drrs_state.refresh_rate_type;
		mutex_unlock(&dev_priv->drrs_state.mutex);

		work->target_rr_type = dev_priv->drrs_state.target_rr_type;
		drm_mode_destroy(intel_encoder->base.dev, work->target_mode);

		if (work->target_rr_type == DRRS_HIGH_RR) {
			prev_mode =
				dev_priv->drrs.connector->panel.fixed_mode;
			resume_idleness_detection = true;
		} else if (work->target_rr_type == DRRS_LOW_RR) {
			prev_mode =
				dev_priv->drrs.connector->panel.downclock_mode;
		} else if (work->target_rr_type == DRRS_MEDIA_RR) {
			prev_mode =
				dev_priv->drrs.connector->panel.target_mode;
		}

		work->target_mode = drm_mode_duplicate(intel_encoder->base.dev,
								prev_mode);
		if (!work->target_mode) {
			DRM_ERROR("target mode creation failed\n");
			return;
		}
		fallback_attempt = true;
		goto init;
	} else {
		if (fallback_attempt)
			DRM_ERROR("DRRS State Fallback attempt failed\n");
		if (ret == -ETIMEDOUT)
			DRM_ERROR("TIMEDOUT in all retry attempt\n");
	}

	drm_mode_destroy(intel_encoder->base.dev, work->target_mode);
}

void
intel_dsi_set_drrs_state(struct intel_encoder *intel_encoder)
{
	struct drm_i915_private *dev_priv =
				intel_encoder->base.dev->dev_private;
	struct drm_display_mode *target_mode =
				dev_priv->drrs.connector->panel.target_mode;
	struct intel_mipi_drrs_work *work = dev_priv->drrs.mipi_drrs_work;
	unsigned int ret;

	ret = work_busy(&work->work.work);
	if (ret) {
		if (work->target_mode)
			if (work->target_mode->vrefresh ==
						target_mode->vrefresh) {
				DRM_DEBUG_KMS("Repeated request for %dHz\n",
							target_mode->vrefresh);
				return;
			}
		DRM_DEBUG_KMS("Cancelling an queued/executing work\n");
		atomic_set(&work->abort_wait_loop, 1);
		cancel_delayed_work_sync(&work->work);
		atomic_set(&work->abort_wait_loop, 0);
		if (ret & WORK_BUSY_PENDING)
			drm_mode_destroy(intel_encoder->base.dev,
							work->target_mode);

	}
	work->intel_encoder = intel_encoder;
	work->target_rr_type = dev_priv->drrs_state.target_rr_type;
	work->target_mode = drm_mode_duplicate(intel_encoder->base.dev,
								target_mode);

	schedule_delayed_work(&dev_priv->drrs.mipi_drrs_work->work, 0);
}

int intel_dsi_drrs_deferred_work_init(struct drm_device *dev)
{
	struct intel_mipi_drrs_work *work;
	struct drm_i915_private *dev_priv = dev->dev_private;

	work = kzalloc(sizeof(struct intel_mipi_drrs_work), GFP_KERNEL);
	if (!work) {
		DRM_ERROR("Failed to allocate mipi DRRS work structure\n");
		return -ENOMEM;
	}

	atomic_set(&work->abort_wait_loop, 0);
	INIT_DELAYED_WORK(&work->work, intel_mipi_drrs_work_fn);
	work->target_mode = NULL;

	dev_priv->drrs.mipi_drrs_work = work;
	return 0;
}

void intel_dsi_drrs_init(struct intel_connector *intel_connector,
				struct drm_display_mode *downclock_mode)
{
	struct drm_connector *connector = &intel_connector->base;
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (intel_dsi_drrs_deferred_work_init(dev) < 0)
		return;

	if (intel_drrs_init(dev, intel_connector, downclock_mode) < 0)
		kfree(dev_priv->drrs.mipi_drrs_work);
	else if (dev_priv->drrs_state.type == SEAMLESS_DRRS_SUPPORT) {
		/* In DSI SEAMLESS DRRS is a SW driven feature */
		dev_priv->drrs_state.type = SEAMLESS_DRRS_SUPPORT_SW;
		intel_attach_drrs_capability_property(connector,
						dev_priv->drrs_state.type);
	}
}

bool intel_dsi_init(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi;
	struct intel_encoder *intel_encoder;
	struct drm_encoder *encoder;
	struct intel_connector *intel_connector;
	struct drm_connector *connector;
	struct drm_display_mode *fixed_mode = NULL;
	struct drm_display_mode *downclock_mode = NULL;
	const struct intel_dsi_device *dsi;
	unsigned int i;

	DRM_DEBUG_KMS("\n");

	intel_dsi = kzalloc(sizeof(*intel_dsi), GFP_KERNEL);
	if (!intel_dsi)
		return false;

	intel_connector = kzalloc(sizeof(*intel_connector), GFP_KERNEL);
	if (!intel_connector) {
		kfree(intel_dsi);
		return false;
	}

	intel_encoder = &intel_dsi->base;
	encoder = &intel_encoder->base;
	intel_dsi->attached_connector = intel_connector;
	connector = &intel_connector->base;

	drm_encoder_init(dev, encoder, &intel_dsi_funcs, DRM_MODE_ENCODER_DSI);

	/* XXX: very likely not all of these are needed */
	intel_encoder->hot_plug = intel_dsi_hot_plug;
	intel_encoder->compute_config = intel_dsi_compute_config;
	intel_encoder->pre_pll_enable = intel_dsi_pre_pll_enable;
	intel_encoder->pre_enable = intel_dsi_pre_enable;
	intel_encoder->enable = intel_dsi_enable;
	intel_encoder->mode_set = intel_dsi_mode_set;
	intel_encoder->disable = intel_dsi_disable;
	intel_encoder->post_disable = intel_dsi_post_disable;
	intel_encoder->get_hw_state = intel_dsi_get_hw_state;
	intel_encoder->get_config = intel_dsi_get_config;
	intel_encoder->set_drrs_state = intel_dsi_set_drrs_state;

	intel_connector->get_hw_state = intel_connector_get_hw_state;

	/* Initialize panel id based on kernel param.
	 * If no kernel param use panel id from VBT
	 * If no  param and no VBT initialize with
	 * default ASUS panel ID for now */
	if (i915_mipi_panel_id <= 0) {
		/* check if panel id available from VBT */
		if (!dev_priv->vbt.dsi.panel_id) {
			/* default Panasonic panel */
			dev_priv->mipi_panel_id = MIPI_DSI_PANASONIC_VXX09F006A00_PANEL_ID;
		} else
			dev_priv->mipi_panel_id = dev_priv->vbt.dsi.panel_id;
	} else
		dev_priv->mipi_panel_id = i915_mipi_panel_id;

	for (i = 0; i < ARRAY_SIZE(intel_dsi_devices); i++) {
		dsi = &intel_dsi_devices[i];
		if (dsi->panel_id == dev_priv->mipi_panel_id) {
			intel_dsi->dev = *dsi;

			if (dsi->dev_ops->init(&intel_dsi->dev))
				break;
		}
	}

	if (i == ARRAY_SIZE(intel_dsi_devices)) {
		DRM_DEBUG_KMS("no device found\n");
		goto err;
	}

	intel_encoder->type = INTEL_OUTPUT_DSI;
	intel_encoder->crtc_mask = (1 << 0); /* XXX */

	intel_encoder->cloneable = false;
	drm_connector_init(dev, connector, &intel_dsi_connector_funcs,
			   DRM_MODE_CONNECTOR_DSI);

	drm_encoder_helper_add(encoder, &intel_dsi_helper_funcs);
	drm_connector_helper_add(connector, &intel_dsi_connector_helper_funcs);

	connector->display_info.subpixel_order = SubPixelHorizontalRGB; /*XXX*/
	connector->interlace_allowed = false;
	connector->doublescan_allowed = false;

	intel_dsi_add_properties(intel_dsi, connector);
	intel_connector_attach_encoder(intel_connector, intel_encoder);

	drm_sysfs_connector_add(connector);

	fixed_mode = dsi->dev_ops->get_modes(&intel_dsi->dev);
	if (!fixed_mode) {
		DRM_DEBUG_KMS("no fixed mode\n");
		goto err;
	}

	dev_priv->is_mipi = true;
	fixed_mode->type |= DRM_MODE_TYPE_PREFERRED;
	if (INTEL_INFO(dev)->gen > 6) {
		downclock_mode = intel_dsi_calc_panel_downclock(dev,
							fixed_mode, connector);
		if (downclock_mode)
			intel_dsi_drrs_init(intel_connector, downclock_mode);
		else
			DRM_DEBUG_KMS("Downclock_mode is not found\n");
	}

	if (dev_priv->vbt.dsi.seq_version < 3) {
		intel_dsi->dev.dev_ops->power_on =
				intel_dsi_power_on;
		intel_dsi->dev.dev_ops->power_off =
				intel_dsi_power_off;
	}

	intel_panel_init(&intel_connector->panel, fixed_mode, downclock_mode);
	intel_panel_setup_backlight(connector);
	intel_connector->panel.fitting_mode = 0;

	return true;
err:
	drm_encoder_cleanup(&intel_encoder->base);
	kfree(intel_dsi);
	kfree(intel_connector);

	return false;
}
