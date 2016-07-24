/*
 * Copyright © 2006-2010 Intel Corporation
 * Copyright (c) 2006 Dave Airlie <airlied@linux.ie>
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
 *	Eric Anholt <eric@anholt.net>
 *      Dave Airlie <airlied@linux.ie>
 *      Jesse Barnes <jesse.barnes@intel.com>
 *      Chris Wilson <chris@chris-wilson.co.uk>
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/moduleparam.h>
#include "intel_drv.h"
#include "linux/mfd/intel_mid_pmic.h"
#include <linux/pwm.h>
#include <linux/platform_data/lp855x.h>
#include <asm/spid.h>
#include "intel_dsi.h"

#define PCI_LBPC 0xf4 /* legacy/combination backlight modes */

void
intel_fixed_panel_mode(struct drm_display_mode *fixed_mode,
		       struct drm_display_mode *adjusted_mode)
{
	adjusted_mode->hdisplay = fixed_mode->hdisplay;
	adjusted_mode->hsync_start = fixed_mode->hsync_start;
	adjusted_mode->hsync_end = fixed_mode->hsync_end;
	adjusted_mode->htotal = fixed_mode->htotal;

	adjusted_mode->vdisplay = fixed_mode->vdisplay;
	adjusted_mode->vsync_start = fixed_mode->vsync_start;
	adjusted_mode->vsync_end = fixed_mode->vsync_end;
	adjusted_mode->vtotal = fixed_mode->vtotal;

	adjusted_mode->clock = fixed_mode->clock;
}

/* adjusted_mode has been preset to be the panel's fixed mode */
void
intel_pch_panel_fitting(struct intel_crtc *intel_crtc,
			struct intel_crtc_config *pipe_config,
			int fitting_mode)
{
	struct drm_display_mode *mode, *adjusted_mode;
	int x, y, width, height;

	mode = &pipe_config->requested_mode;
	adjusted_mode = &pipe_config->adjusted_mode;

	x = y = width = height = 0;

	/* Native modes don't need fitting */
	if (adjusted_mode->hdisplay == mode->hdisplay &&
	    adjusted_mode->vdisplay == mode->vdisplay)
		goto done;

	switch (fitting_mode) {
	case DRM_MODE_SCALE_CENTER:
		width = mode->hdisplay;
		height = mode->vdisplay;
		x = (adjusted_mode->hdisplay - width + 1)/2;
		y = (adjusted_mode->vdisplay - height + 1)/2;
		break;

	case DRM_MODE_SCALE_ASPECT:
		/* Scale but preserve the aspect ratio */
		{
			u32 scaled_width = adjusted_mode->hdisplay * mode->vdisplay;
			u32 scaled_height = mode->hdisplay * adjusted_mode->vdisplay;
			if (scaled_width > scaled_height) { /* pillar */
				width = scaled_height / mode->vdisplay;
				if (width & 1)
					width++;
				x = (adjusted_mode->hdisplay - width + 1) / 2;
				y = 0;
				height = adjusted_mode->vdisplay;
			} else if (scaled_width < scaled_height) { /* letter */
				height = scaled_width / mode->hdisplay;
				if (height & 1)
				    height++;
				y = (adjusted_mode->vdisplay - height + 1) / 2;
				x = 0;
				width = adjusted_mode->hdisplay;
			} else {
				x = y = 0;
				width = adjusted_mode->hdisplay;
				height = adjusted_mode->vdisplay;
			}
		}
		break;

	case DRM_MODE_SCALE_FULLSCREEN:
		x = y = 0;
		width = adjusted_mode->hdisplay;
		height = adjusted_mode->vdisplay;
		break;

	default:
		WARN(1, "bad panel fit mode: %d\n", fitting_mode);
		return;
	}

done:
	pipe_config->pch_pfit.pos = (x << 16) | y;
	pipe_config->pch_pfit.size = (width << 16) | height;
}

static void
centre_horizontally(struct drm_display_mode *mode,
		    int width)
{
	u32 border, sync_pos, blank_width, sync_width;

	/* keep the hsync and hblank widths constant */
	sync_width = mode->crtc_hsync_end - mode->crtc_hsync_start;
	blank_width = mode->crtc_hblank_end - mode->crtc_hblank_start;
	sync_pos = (blank_width - sync_width + 1) / 2;

	border = (mode->hdisplay - width + 1) / 2;
	border += border & 1; /* make the border even */

	mode->crtc_hdisplay = width;
	mode->crtc_hblank_start = width + border;
	mode->crtc_hblank_end = mode->crtc_hblank_start + blank_width;

	mode->crtc_hsync_start = mode->crtc_hblank_start + sync_pos;
	mode->crtc_hsync_end = mode->crtc_hsync_start + sync_width;
}

static void
centre_vertically(struct drm_display_mode *mode,
		  int height)
{
	u32 border, sync_pos, blank_width, sync_width;

	/* keep the vsync and vblank widths constant */
	sync_width = mode->crtc_vsync_end - mode->crtc_vsync_start;
	blank_width = mode->crtc_vblank_end - mode->crtc_vblank_start;
	sync_pos = (blank_width - sync_width + 1) / 2;

	border = (mode->vdisplay - height + 1) / 2;

	mode->crtc_vdisplay = height;
	mode->crtc_vblank_start = height + border;
	mode->crtc_vblank_end = mode->crtc_vblank_start + blank_width;

	mode->crtc_vsync_start = mode->crtc_vblank_start + sync_pos;
	mode->crtc_vsync_end = mode->crtc_vsync_start + sync_width;
}

static inline u32 panel_fitter_scaling(u32 source, u32 target)
{
	/*
	 * Floating point operation is not supported. So the FACTOR
	 * is defined, which can avoid the floating point computation
	 * when calculating the panel ratio.
	 */
#define ACCURACY 12
#define FACTOR (1 << ACCURACY)
	u32 ratio = source * FACTOR / target;
	return (FACTOR * ratio + FACTOR/2) / FACTOR;
}

void intel_gmch_panel_fitting(struct intel_crtc *intel_crtc,
			      struct intel_crtc_config *pipe_config,
			      int fitting_mode)
{
	struct drm_device *dev = intel_crtc->base.dev;
	u32 pfit_control = 0, pfit_pgm_ratios = 0, border = 0;
	struct drm_display_mode *mode, *adjusted_mode;
	uint32_t scaling_src_w, scaling_src_h = 0;

	intel_crtc->base.panning_en = false;

	mode = &pipe_config->requested_mode;
	adjusted_mode = &pipe_config->adjusted_mode;

	if (IS_VALLEYVIEW(dev)) {
		scaling_src_w = ((intel_crtc->scaling_src_size >>
				SCALING_SRCSIZE_SHIFT) &
				SCALING_SRCSIZE_MASK) + 1;
		scaling_src_h = (intel_crtc->scaling_src_size &
				SCALING_SRCSIZE_MASK) + 1;

		/* The input src size should be < 2kx2k */
		if ((scaling_src_w > PFIT_SIZE_LIMIT) ||
			(scaling_src_h > PFIT_SIZE_LIMIT)) {
			DRM_ERROR("Wrong panel fitter input src conf");
			goto out;
		}

		if (fitting_mode == AUTOSCALE)
			pfit_control = PFIT_SCALING_AUTO;
		else if (fitting_mode == PILLARBOX)
			pfit_control = PFIT_SCALING_PILLAR;
		else if (fitting_mode == LETTERBOX)
			pfit_control = PFIT_SCALING_LETTER;
		else {
			pipe_config->gmch_pfit.control &= ~PFIT_ENABLE;
			intel_crtc->base.panning_en = false;
			goto out1;
		}
		pfit_control |= (PFIT_ENABLE | (intel_crtc->pipe
					<< PFIT_PIPE_SHIFT));
		intel_crtc->base.panning_en = true;
		goto out;
	}

	/* Native modes don't need fitting */
	if (adjusted_mode->hdisplay == mode->hdisplay &&
	    adjusted_mode->vdisplay == mode->vdisplay)
		goto out;

	switch (fitting_mode) {
	case DRM_MODE_SCALE_CENTER:
		/*
		 * For centered modes, we have to calculate border widths &
		 * heights and modify the values programmed into the CRTC.
		 */
		centre_horizontally(adjusted_mode, mode->hdisplay);
		centre_vertically(adjusted_mode, mode->vdisplay);
		border = LVDS_BORDER_ENABLE;
		break;
	case DRM_MODE_SCALE_ASPECT:
		/* Scale but preserve the aspect ratio */
		if (INTEL_INFO(dev)->gen >= 4) {
			u32 scaled_width = adjusted_mode->hdisplay *
				mode->vdisplay;
			u32 scaled_height = mode->hdisplay *
				adjusted_mode->vdisplay;

			/* 965+ is easy, it does everything in hw */
			if (scaled_width > scaled_height)
				pfit_control |= PFIT_ENABLE |
					PFIT_SCALING_PILLAR;
			else if (scaled_width < scaled_height)
				pfit_control |= PFIT_ENABLE |
					PFIT_SCALING_LETTER;
			else if (adjusted_mode->hdisplay != mode->hdisplay)
				pfit_control |= PFIT_ENABLE | PFIT_SCALING_AUTO;
		} else {
			u32 scaled_width = adjusted_mode->hdisplay *
				mode->vdisplay;
			u32 scaled_height = mode->hdisplay *
				adjusted_mode->vdisplay;
			/*
			 * For earlier chips we have to calculate the scaling
			 * ratio by hand and program it into the
			 * PFIT_PGM_RATIO register
			 */
			if (scaled_width > scaled_height) { /* pillar */
				centre_horizontally(adjusted_mode,
						    scaled_height /
						    mode->vdisplay);

				border = LVDS_BORDER_ENABLE;
				if (mode->vdisplay != adjusted_mode->vdisplay) {
					u32 bits = panel_fitter_scaling(mode->vdisplay, adjusted_mode->vdisplay);
					pfit_pgm_ratios |= (bits << PFIT_HORIZ_SCALE_SHIFT |
							    bits << PFIT_VERT_SCALE_SHIFT);
					pfit_control |= (PFIT_ENABLE |
							 VERT_INTERP_BILINEAR |
							 HORIZ_INTERP_BILINEAR);
				}
			} else if (scaled_width < scaled_height) { /* letter */
				centre_vertically(adjusted_mode,
						  scaled_width /
						  mode->hdisplay);

				border = LVDS_BORDER_ENABLE;
				if (mode->hdisplay != adjusted_mode->hdisplay) {
					u32 bits = panel_fitter_scaling(mode->hdisplay, adjusted_mode->hdisplay);
					pfit_pgm_ratios |= (bits << PFIT_HORIZ_SCALE_SHIFT |
							    bits << PFIT_VERT_SCALE_SHIFT);
					pfit_control |= (PFIT_ENABLE |
							 VERT_INTERP_BILINEAR |
							 HORIZ_INTERP_BILINEAR);
				}
			} else {
				/* Aspects match, Let hw scale both directions */
				pfit_control |= (PFIT_ENABLE |
						 VERT_AUTO_SCALE | HORIZ_AUTO_SCALE |
						 VERT_INTERP_BILINEAR |
						 HORIZ_INTERP_BILINEAR);
			}
		}
		break;
	case DRM_MODE_SCALE_FULLSCREEN:
		/*
		 * Full scaling, even if it changes the aspect ratio.
		 * Fortunately this is all done for us in hw.
		 */
		if (mode->vdisplay != adjusted_mode->vdisplay ||
		    mode->hdisplay != adjusted_mode->hdisplay) {
			pfit_control |= PFIT_ENABLE;
			if (INTEL_INFO(dev)->gen >= 4)
				pfit_control |= PFIT_SCALING_AUTO;
			else
				pfit_control |= (VERT_AUTO_SCALE |
						 VERT_INTERP_BILINEAR |
						 HORIZ_AUTO_SCALE |
						 HORIZ_INTERP_BILINEAR);
		}
		break;
	default:
		WARN(1, "bad panel fit mode: %d\n", fitting_mode);
		return;
	}

	/* 965+ wants fuzzy fitting */
	/* FIXME: handle multiple panels by failing gracefully */
	if (INTEL_INFO(dev)->gen >= 4)
		pfit_control |= ((intel_crtc->pipe << PFIT_PIPE_SHIFT) |
				 PFIT_FILTER_FUZZY);

out:
	if ((pfit_control & PFIT_ENABLE) == 0) {
		pfit_control = 0;
		pfit_pgm_ratios = 0;
		intel_crtc->scaling_src_size = 0;
	}

	/* Make sure pre-965 set dither correctly for 18bpp panels. */
	if (INTEL_INFO(dev)->gen < 4 && pipe_config->pipe_bpp == 18)
		pfit_control |= PANEL_8TO6_DITHER_ENABLE;

	pipe_config->gmch_pfit.control = pfit_control;
out1:
	pipe_config->gmch_pfit.pgm_ratios = pfit_pgm_ratios;
	pipe_config->gmch_pfit.lvds_border_bits = border;
}

static int is_backlight_combination_mode(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (INTEL_INFO(dev)->gen >= 4)
		return I915_READ(BLC_PWM_CTL2) & BLM_COMBINATION_MODE;

	if (IS_GEN2(dev))
		return I915_READ(BLC_PWM_CTL) & BLM_LEGACY_MODE;

	return 0;
}

/* XXX: query mode clock or hardware clock and program max PWM appropriately
 * when it's 0.
 */
static u32 i915_read_blc_pwm_ctl(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val;

	WARN_ON_SMP(!spin_is_locked(&dev_priv->backlight.lock));

	/* Restore the CTL value if it lost, e.g. GPU reset */

	if (HAS_PCH_SPLIT(dev_priv->dev)) {
		val = I915_READ(BLC_PWM_PCH_CTL2);
		if (dev_priv->regfile.saveBLC_PWM_CTL2 == 0) {
			dev_priv->regfile.saveBLC_PWM_CTL2 = val;
		} else if (val == 0) {
			val = dev_priv->regfile.saveBLC_PWM_CTL2;
			I915_WRITE(BLC_PWM_PCH_CTL2, val);
		}
	} else {
		val = I915_READ(BLC_PWM_CTL);
		if (dev_priv->regfile.saveBLC_PWM_CTL == 0) {
			dev_priv->regfile.saveBLC_PWM_CTL = val;
			if (INTEL_INFO(dev)->gen >= 4)
				dev_priv->regfile.saveBLC_PWM_CTL2 =
					I915_READ(BLC_PWM_CTL2);
		} else if (val == 0) {
			val = dev_priv->regfile.saveBLC_PWM_CTL;
			I915_WRITE(BLC_PWM_CTL, val);
			if (INTEL_INFO(dev)->gen >= 4)
				I915_WRITE(BLC_PWM_CTL2,
					   dev_priv->regfile.saveBLC_PWM_CTL2);
		}
	}

	return val;
}

u32 intel_panel_get_max_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 max;

	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi)
		return 0xff;

	max = i915_read_blc_pwm_ctl(dev);

	if (HAS_PCH_SPLIT(dev)) {
		max >>= 16;
	} else {
		if (INTEL_INFO(dev)->gen < 4)
			max >>= 17;
		else
			max >>= 16;

		if (is_backlight_combination_mode(dev))
			max *= 0xff;
	}

	DRM_DEBUG_DRIVER("max backlight PWM = %d\n", max);

	return max;
}

static int i915_panel_invert_brightness;
MODULE_PARM_DESC(invert_brightness, "Invert backlight brightness "
	"(-1 force normal, 0 machine defaults, 1 force inversion), please "
	"report PCI device ID, subsystem vendor and subsystem device ID "
	"to dri-devel@lists.freedesktop.org, if your machine needs it. "
	"It will then be included in an upcoming module version.");
module_param_named(invert_brightness, i915_panel_invert_brightness, int, 0600);
static u32 intel_panel_compute_brightness(struct drm_device *dev, u32 val)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	if (i915_panel_invert_brightness < 0)
		return val;

	if (i915_panel_invert_brightness > 0 ||
	    dev_priv->quirks & QUIRK_INVERT_BRIGHTNESS) {
		u32 max = intel_panel_get_max_backlight(dev);
		if (max)
			return max - val;
	}

	return val;
}

static u32 intel_panel_get_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = 0;
	unsigned long flags;


	/*
	 * PMIC i2c write for backlight control is accessed only
	 * from intel_panel.c and need not be in spin_lock
	 * There are anyway mutex to protect the i2c read in the
	 * PMIC driver
	 *
	 * Was causing BUG as mutex was taken within spin_lock
	 */
	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
		if (BYT_CR_CONFIG) {
			val = lpio_bl_read(0, LPIO_PWM_CTRL);
			val &= 0xff;
		} else
			val = intel_mid_pmic_readb(0x4E);
	}

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	if (HAS_PCH_SPLIT(dev)) {
		val = I915_READ(BLC_PWM_CPU_CTL) & BACKLIGHT_DUTY_CYCLE_MASK;
	} else if (!(IS_VALLEYVIEW(dev) && dev_priv->is_mipi)) {
		val = I915_READ(BLC_PWM_CTL) & BACKLIGHT_DUTY_CYCLE_MASK;
		if (INTEL_INFO(dev)->gen < 4)
			val >>= 1;

		if (is_backlight_combination_mode(dev)) {
			u8 lbpc = 0;

			pci_read_config_byte(dev->pdev, PCI_LBPC, &lbpc);
			val *= lbpc;
		}
	}

	/* When DPST is enabled, reading the backlight register will
	 - give the DPST adjusted backlight value. Since DPST works
	 * without user knowing a perceived difference in the backlight,
	 * the programmed backlight isn't the correct value to return.
	 * So, get the user perceived backlight level from DPST. */
	if (dev_priv->dpst.enabled)
		val = i915_dpst_get_brightness(dev);
	else
		val = intel_panel_compute_brightness(dev, val);

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	DRM_DEBUG_DRIVER("get backlight PWM = %d\n", val);
	return val;
}

static void intel_pch_panel_set_backlight(struct drm_device *dev, u32 level)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = I915_READ(BLC_PWM_CPU_CTL) & ~BACKLIGHT_DUTY_CYCLE_MASK;
	I915_WRITE(BLC_PWM_CPU_CTL, val | level);
}

void intel_panel_actually_set_backlight(struct drm_device *dev, u32 level)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 tmp;

	DRM_DEBUG_DRIVER("set backlight PWM = %d\n", level);
	level = intel_panel_compute_brightness(dev, level);

	if (HAS_PCH_SPLIT(dev))
		return intel_pch_panel_set_backlight(dev, level);

	if (is_backlight_combination_mode(dev)) {
		u32 max = intel_panel_get_max_backlight(dev);
		u8 lbpc;

		/* we're screwed, but keep behaviour backwards compatible */
		if (!max)
			max = 1;

		lbpc = level * 0xfe / max + 1;
		level /= lbpc;
		pci_write_config_byte(dev->pdev, PCI_LBPC, lbpc);
	}

	tmp = I915_READ(BLC_PWM_CTL);
	if (INTEL_INFO(dev)->gen < 4)
		level <<= 1;
	tmp &= ~BACKLIGHT_DUTY_CYCLE_MASK;
	I915_WRITE(BLC_PWM_CTL, tmp | level);
}

void intel_panel_actually_set_mipi_backlight(struct drm_device *dev, u32 level)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	/* For BYT-CR */
	if (dev_priv->vbt.dsi.config->pmic_soc_blc) {
		/* FixMe: if level is zero still a pulse is observed consuming
		power. To fix this issue if requested level is zero then
		disable pwm and enabled it again if brightness changes */
		lpio_bl_write_bits(0, LPIO_PWM_CTRL, (0xff - level), 0xFF);
		lpio_bl_update(0, LPIO_PWM_CTRL);
	} else
		intel_mid_pmic_writeb(PMIC_PWM_LEVEL, level);
}

/* set backlight brightness to level in range [0..max] */
void intel_panel_set_backlight(struct drm_device *dev, u32 level, u32 max)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 freq;
	unsigned long flags;

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	freq = intel_panel_get_max_backlight(dev);
	if (!freq) {
		/* we are screwed, bail out */
		spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);
		return;
	}

	/* scale to hardware */
	level = level * freq / max;

	dev_priv->backlight.level = level;
	if (dev_priv->backlight.device)
		dev_priv->backlight.device->props.brightness = level;


	if (dev_priv->backlight.enabled) {
		if (dev_priv->dpst.enabled)
			level = i915_dpst_compute_brightness(dev, level);

		if (!dev_priv->is_mipi)
			intel_panel_actually_set_backlight(dev, level);
	}

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	if (dev_priv->is_mipi)
		intel_panel_actually_set_mipi_backlight(dev, level);
	/* TODO: the below value is quite good for FFRD8, but BYT CR needs
	MIPI panle some more tuning. Also need to check is it a HW limitation*/
	usleep_range(2000, 3000);
}

void intel_panel_disable_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_dsi *intel_dsi = NULL;
	struct drm_crtc *crtc = NULL;
	struct intel_encoder *encoder = NULL;
	unsigned long flags;


	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
		list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
			for_each_encoder_on_crtc(dev, crtc, encoder) {
				if (encoder->type == INTEL_OUTPUT_DSI)
					intel_dsi =
					enc_to_intel_dsi(&encoder->base);
			}
		}

		intel_panel_actually_set_mipi_backlight(dev, 0);

		if (dev_priv->vbt.dsi.config->pmic_soc_blc) {
			/* cancel any delayed work scheduled */
			cancel_delayed_work_sync(
				&dev_priv->bkl_delay_enable_work);

			/* disable the backlight enable signal */
			if ((intel_dsi && dev_priv->vbt.dsi.seq_version >= 3)) {
				if (intel_dsi->dev.dev_ops->disable_backlight)
					intel_dsi->dev.dev_ops->disable_backlight(&intel_dsi->dev);
			} else {
				vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
						PANEL1_BKLTEN_GPIONC_10_PCONF0,
						VLV_GPIO_CFG);
				vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
						PANEL1_BKLTEN_GPIONC_10_PAD,
						VLV_GPIO_INPUT_DIS);
				udelay(500);
			}
			lpio_bl_write_bits(0, LPIO_PWM_CTRL, 0x00, 0x80000000);
		} else {
			if ((intel_dsi && dev_priv->vbt.dsi.seq_version >= 3)) {
				if (intel_dsi->dev.dev_ops->disable_backlight)
					intel_dsi->dev.dev_ops->disable_backlight(&intel_dsi->dev);
			} else {
				intel_mid_pmic_writeb(PMIC_PWM_EN, 0x00);
				intel_mid_pmic_writeb(PMIC_BKL_EN, 0x7F);
			}
		}
	}

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	dev_priv->backlight.enabled = false;

	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
		spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);
		return;
	}

	if (INTEL_INFO(dev)->gen >= 4 &&
				!(IS_VALLEYVIEW(dev) && dev_priv->is_mipi)) {
		uint32_t reg, tmp;

		intel_panel_actually_set_backlight(dev, 0);

		reg = HAS_PCH_SPLIT(dev) ? BLC_PWM_CPU_CTL2 : BLC_PWM_CTL2;

		I915_WRITE(reg, I915_READ(reg) & ~BLM_PWM_ENABLE);

		if (HAS_PCH_SPLIT(dev)) {
			tmp = I915_READ(BLC_PWM_PCH_CTL1);
			tmp &= ~BLM_PCH_PWM_ENABLE;
			I915_WRITE(BLC_PWM_PCH_CTL1, tmp);
		}
	}

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);
}
#ifdef CONFIG_CRYSTAL_COVE
static void scheduled_led_chip_programming(struct work_struct *work)
{
	lp855x_ext_write_byte(LP8556_CFG9,
			LP8556_VBOOST_MAX_NA_21V |
			LP8556_JUMP_DIS |
			LP8556_JMP_TSHOLD_10P |
			LP8556_JMP_VOLT_0_5V);
	lp855x_ext_write_byte(LP8556_CFG5,
			LP8556_PWM_DRECT_DIS |
			LP8556_PS_MODE_5P5D |
			LP8556_PWM_FREQ_9616HZ);
	lp855x_ext_write_byte(LP8556_CFG7,
			LP8556_RSRVD_76 |
			LP8556_DRV3_EN |
			LP8556_DRV2_EN |
			LP8556_RSRVD_32 |
			LP8556_IBOOST_LIM_1_8A_NA);
	lp855x_ext_write_byte(LP8556_LEDSTREN,
			LP8556_5LEDSTR);
}
#endif

static uint32_t compute_pwm_base(uint16_t freq)
{
	uint32_t base_unit;

	if (freq < 400)
		freq = 400;
	/*The PWM block is clocked by the 25MHz oscillator clock.
	* The output frequency can be estimated with the equation:
	* Target frequency = XOSC * Base_unit_value/256
	*/
	base_unit = (freq * 256) / 25;

	/* Also Base_unit_value need to converted to QM.N notation
	* to program the value in register
	* Using the following for converting to Q8.8 notation
	* For QM.N representation, consider a floating point variable 'a' :
	* Step 1: Calculate b = a* 2^N , where N is the fractional length of the variable.
	* Note that a is represented in decimal.
	* Step 2: Round the value of 'b' to the nearest integer value. For example:
	* RoundOff (1.05) --> 1
	* RoundOff (1.5)  --> 2
	* Step 3: Convert 'b' from decimal to binary representation and name the new variable 'c'
	*/
	base_unit = base_unit * 256;
	base_unit = DIV_ROUND_CLOSEST(base_unit, 1000000);

	return base_unit;
}

void intel_panel_enable_backlight(struct drm_device *dev,
				  enum pipe pipe)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	enum transcoder cpu_transcoder =
		intel_pipe_to_cpu_transcoder(dev_priv, pipe);
	struct intel_dsi *intel_dsi = NULL;
	struct drm_crtc *crtc = NULL;
	struct intel_encoder *encoder = NULL;
	unsigned long flags = 0;
	uint32_t pwm_base;

	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi) {
		uint32_t val;

		list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
			for_each_encoder_on_crtc(dev, crtc, encoder) {
				if (encoder->type == INTEL_OUTPUT_DSI)
					intel_dsi = enc_to_intel_dsi(
							&encoder->base);
			}
		}
		/* For BYT-CR */
		if (dev_priv->vbt.dsi.config->pmic_soc_blc) {
			/* GPIOC_94 config to PWM0 function */
			val = vlv_gps_core_read(dev_priv,
					GP_CAMERASB07_GPIONC_22_PCONF0);
			vlv_gps_core_write(dev_priv,
					GP_CAMERASB07_GPIONC_22_PCONF0,
					0x2000CC01);
			vlv_gps_core_write(dev_priv,
					GP_CAMERASB07_GPIONC_22_PAD, 0x5);

			/*
			 * PWM enable
			 * Assuming only 1 LFP
			 */
			pwm_base = compute_pwm_base(
					dev_priv->vbt.backlight.pwm_freq_hz);
			pwm_base = pwm_base << 8;
			lpio_bl_write(0, LPIO_PWM_CTRL, pwm_base);
			lpio_bl_update(0, LPIO_PWM_CTRL);
			lpio_bl_write_bits(0, LPIO_PWM_CTRL, 0x80000000,
							0x80000000);
			lpio_bl_update(0, LPIO_PWM_CTRL);

			if ((intel_dsi && dev_priv->vbt.dsi.seq_version >= 3)) {
				if (intel_dsi->dev.dev_ops->enable_backlight)
					intel_dsi->dev.dev_ops->
					enable_backlight(&intel_dsi->dev);
			} else {
				/* Backlight enable */
				vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
						PANEL1_BKLTEN_GPIONC_10_PCONF0,
						VLV_GPIO_CFG);
				vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
						PANEL1_BKLTEN_GPIONC_10_PAD,
						VLV_GPIO_INPUT_EN);

				udelay(500);
			}
			if (lpdata)
				schedule_delayed_work(&dev_priv->bkl_delay_enable_work,
						msecs_to_jiffies(30));

		} else {
			if ((intel_dsi && dev_priv->vbt.dsi.seq_version >= 3)) {
				if (intel_dsi->dev.dev_ops->enable_backlight)
					intel_dsi->dev.dev_ops->
					enable_backlight(&intel_dsi->dev);
			} else {
				intel_mid_pmic_writeb(PMIC_BKL_EN, 0xFF);
				intel_mid_pmic_writeb(PMIC_PWM_EN, 0x01);
			}
		}
	}

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);

	if (dev_priv->backlight.level == 0) {
		if (dev_priv->backlight.device)
			dev_priv->backlight.device->props.brightness =
				dev_priv->backlight.level;
	}

	if (INTEL_INFO(dev)->gen >= 4 &&
				!(IS_VALLEYVIEW(dev) && dev_priv->is_mipi)) {
		uint32_t reg, tmp;

		reg = HAS_PCH_SPLIT(dev) ? BLC_PWM_CPU_CTL2 : BLC_PWM_CTL2;


		tmp = I915_READ(reg);

		/* Note that this can also get called through dpms changes. And
		 * we don't track the backlight dpms state, hence check whether
		 * we have to do anything first. */
		if (tmp & BLM_PWM_ENABLE)
			goto set_level;

		if (INTEL_INFO(dev)->num_pipes == 3)
			tmp &= ~BLM_PIPE_SELECT_IVB;
		else
			tmp &= ~BLM_PIPE_SELECT;

		if (cpu_transcoder == TRANSCODER_EDP)
			tmp |= BLM_TRANSCODER_EDP;
		else
			tmp |= BLM_PIPE(cpu_transcoder);
		tmp &= ~BLM_PWM_ENABLE;

		I915_WRITE(reg, tmp);
		POSTING_READ(reg);
		I915_WRITE(reg, tmp | BLM_PWM_ENABLE);

		if (HAS_PCH_SPLIT(dev) &&
		    !(dev_priv->quirks & QUIRK_NO_PCH_PWM_ENABLE)) {
			tmp = I915_READ(BLC_PWM_PCH_CTL1);
			tmp |= BLM_PCH_PWM_ENABLE;
			tmp &= ~BLM_PCH_OVERRIDE_ENABLE;
			I915_WRITE(BLC_PWM_PCH_CTL1, tmp);
		}
	}

set_level:
	/* Call below after setting BLC_PWM_CPU_CTL2 and BLC_PWM_PCH_CTL1.
	 * BLC_PWM_CPU_CTL may be cleared to zero automatically when these
	 * registers are set.
	 */
	dev_priv->backlight.enabled = true;
	if (!dev_priv->is_mipi)
		intel_panel_actually_set_backlight(dev,
						dev_priv->backlight.level);

	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi)
		intel_panel_actually_set_mipi_backlight(dev,
					dev_priv->backlight.level);
}

static void intel_panel_init_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	dev_priv->backlight.level = intel_panel_get_backlight(dev);
	if (dev_priv->backlight.level == 0) {
		/*To handle scenarios where GOP missed to update the LFP backlight register*/
		if (IS_VALLEYVIEW(dev) && dev_priv->is_mipi)
			dev_priv->backlight.level = dev_priv->vbt.init_backlight_level;
	}
	dev_priv->backlight.enabled = dev_priv->backlight.level != 0;

	if (BYT_CR_CONFIG)
		INIT_DELAYED_WORK(&dev_priv->bkl_delay_enable_work,
				scheduled_led_chip_programming);
}

enum drm_connector_status
intel_panel_detect(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* Assume that the BIOS does not lie through the OpRegion... */
	if (!i915_panel_ignore_lid && dev_priv->opregion.lid_state) {
		return ioread32(dev_priv->opregion.lid_state) & 0x1 ?
			connector_status_connected :
			connector_status_disconnected;
	}

	switch (i915_panel_ignore_lid) {
	case -2:
		return connector_status_connected;
	case -1:
		return connector_status_disconnected;
	default:
		return connector_status_unknown;
	}
}

#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
static int intel_panel_update_status(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);
	intel_panel_set_backlight(dev, bd->props.brightness,
				  bd->props.max_brightness);
	return 0;
}

static int intel_panel_get_brightness(struct backlight_device *bd)
{
	struct drm_device *dev = bl_get_data(bd);
	return intel_panel_get_backlight(dev);
}

static const struct backlight_ops intel_panel_bl_ops = {
	.update_status = intel_panel_update_status,
	.get_brightness = intel_panel_get_brightness,
};

int intel_panel_setup_backlight(struct drm_connector *connector)
{
	struct drm_device *dev = connector->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct backlight_properties props;
	unsigned long flags;

	intel_panel_init_backlight(dev);

	if (WARN_ON(dev_priv->backlight.device))
		return -ENODEV;

	memset(&props, 0, sizeof(props));
	props.type = BACKLIGHT_RAW;
	props.brightness = dev_priv->backlight.level;

	spin_lock_irqsave(&dev_priv->backlight.lock, flags);
	props.max_brightness = intel_panel_get_max_backlight(dev);
	spin_unlock_irqrestore(&dev_priv->backlight.lock, flags);

	if (props.max_brightness == 0) {
		DRM_DEBUG_DRIVER("Failed to get maximum backlight value\n");
		return -ENODEV;
	}
	dev_priv->backlight.device =
		backlight_device_register("intel_backlight",
					  &connector->kdev, dev,
					  &intel_panel_bl_ops, &props);

	if (IS_ERR(dev_priv->backlight.device)) {
		DRM_ERROR("Failed to register backlight: %ld\n",
			  PTR_ERR(dev_priv->backlight.device));
		dev_priv->backlight.device = NULL;
		return -ENODEV;
	}
	return 0;
}

void intel_panel_destroy_backlight(struct drm_device *dev)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	if (dev_priv->backlight.device) {
		backlight_device_unregister(dev_priv->backlight.device);
		dev_priv->backlight.device = NULL;
	}
}
#else
int intel_panel_setup_backlight(struct drm_connector *connector)
{
	intel_panel_init_backlight(connector->dev);
	return 0;
}

void intel_panel_destroy_backlight(struct drm_device *dev)
{
	return;
}
#endif

int intel_panel_init(struct intel_panel *panel,
			struct drm_display_mode *fixed_mode,
			struct drm_display_mode *downclock_mode)
{
	panel->fixed_mode = fixed_mode;
	panel->downclock_mode = downclock_mode;

	return 0;
}

/*
 * intel_dsi_calc_panel_downclock - calculate the reduced downclock for DSI
 * @dev: drm device
 * @fixed_mode : panel native mode
 * @connector: DSI connector
 *
 * Return downclock_avail
 * Calculate the reduced downclock for DSI.
 */

struct drm_display_mode *
intel_dsi_calc_panel_downclock(struct drm_device *dev,
			struct drm_display_mode *fixed_mode,
			struct drm_connector *connector)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_display_mode *downclock_mode = NULL;

	if (dev_priv->vbt.drrs_min_vrefresh == 0)
		return downclock_mode;

	/* Allocate */
	downclock_mode = drm_mode_duplicate(dev, fixed_mode);
	if (!downclock_mode) {
		DRM_DEBUG_KMS("%s: No memory\n", __func__);
		return NULL;
	}

	downclock_mode->vrefresh = dev_priv->vbt.drrs_min_vrefresh;
	DRM_DEBUG("drrs_min_vrefresh = %u\n", downclock_mode->vrefresh);
	downclock_mode->clock =  downclock_mode->vrefresh *
		downclock_mode->vtotal * downclock_mode->htotal / 1000;

	return downclock_mode;
}

/*
 * intel_find_panel_downclock - find the reduced downclock for LVDS in EDID
 * @dev: drm device
 * @fixed_mode : panel native mode
 * @connector: LVDS/eDP connector
 *
 * Return downclock_avail
 * Find the reduced downclock for LVDS/eDP in EDID.
 */

struct drm_display_mode *
intel_find_panel_downclock(struct drm_device *dev,
			struct drm_display_mode *fixed_mode,
			struct drm_connector *connector)
{
	struct drm_display_mode *scan, *tmp_mode;
	int temp_downclock;
	if (!fixed_mode) {
		DRM_ERROR("Mode can't be NULL\n");
		return NULL;
	}
	temp_downclock = fixed_mode->clock;
	tmp_mode = NULL;

	list_for_each_entry(scan, &connector->probed_modes, head) {
		/*
		 * If one mode has the same resolution with the fixed_panel
		 * mode while they have the different refresh rate, it means
		 * that the reduced downclock is found. In such
		 * case we can set the different FPx0/1 to dynamically select
		 * between low and high frequency.
		*/
		if (scan->hdisplay == fixed_mode->hdisplay &&
		scan->hsync_start == fixed_mode->hsync_start &&
		scan->hsync_end == fixed_mode->hsync_end &&
		scan->htotal == fixed_mode->htotal &&
		scan->vdisplay == fixed_mode->vdisplay &&
		scan->vsync_start == fixed_mode->vsync_start &&
		scan->vsync_end == fixed_mode->vsync_end &&
		scan->vtotal == fixed_mode->vtotal) {
			if (scan->clock < temp_downclock) {
				/*
				 * The downclock is already found. But we
				 * expect to find the lower downclock.
				 */
				temp_downclock = scan->clock;
				tmp_mode = scan;
			}
		}
	}

	if (temp_downclock < fixed_mode->clock)
		return drm_mode_duplicate(dev, tmp_mode);
	else
		return NULL;
}

void intel_panel_fini(struct intel_panel *panel)
{
	struct intel_connector *intel_connector =
		container_of(panel, struct intel_connector, panel);

	if (panel->fixed_mode)
		drm_mode_destroy(intel_connector->base.dev, panel->fixed_mode);

	if (panel->downclock_mode)
		drm_mode_destroy(intel_connector->base.dev,
				panel->downclock_mode);
}
