/*****************************************************************************
 *
 * Copyright © 2010 Intel Corporation
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
 ******************************************************************************/
#include <linux/console.h>

#include "psb_drv.h"
#include "pmu_tng.h"
#include "psb_fb.h"
#include "psb_intel_reg.h"
#include "displayclass_interface.h"
#include "mdfld_dsi_output.h"
#include "pwr_mgmt.h"
#include "mdfld_dsi_dbi_dsr.h"
#include "mdfld_dsi_dbi.h"
#include <linux/kernel.h>
#include <string.h>
#include "android_hdmi.h"


static bool is_vblank_period(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct android_hdmi_priv *hdmi_priv = NULL;
	u32 reg_offset = 0;
	int vdisplay = 0, vrefresh = 0;
	int delay_us = 5, dsl_threshold = 0, plane_flip_time = 200;
	int retry = 0;
	int dsl = 0;

	if (!dev || !dev->dev_private)
		return false;

	dev_priv = (struct drm_psb_private *)dev->dev_private;

	switch (pipe) {
	case PIPEA:
		reg_offset = 0;
		dsi_config = dev_priv->dsi_configs[0];
		if (dsi_config && dsi_config->mode) {
			vdisplay = dsi_config->mode->vdisplay;
			vrefresh = dsi_config->mode->vrefresh;
		}
		break;
	case PIPEB:
		reg_offset = 0x1000;
		hdmi_priv = dev_priv->hdmi_priv;
		if (hdmi_priv && hdmi_priv->current_mode) {
			vdisplay = hdmi_priv->current_mode->vdisplay;
			vrefresh = hdmi_priv->current_mode->vrefresh;
		}
		break;
	case PIPEC:
		reg_offset = 0x2000;
		dsi_config = dev_priv->dsi_configs[1];
		if (dsi_config && dsi_config->mode) {
			vdisplay = dsi_config->mode->vdisplay;
			vrefresh = dsi_config->mode->vrefresh;
		}
		break;
	default:
		DRM_ERROR("Invalid pipe %d\n", pipe);
		return false;
	}

	if (vdisplay <= 0) {
		DRM_ERROR("Invalid vertical active region for pipe %d.\n", pipe);
		return false;
	}

	retry = (int)(1000000 / (vrefresh * delay_us));
	dsl_threshold = vdisplay - (int)(1000000 / (vrefresh * plane_flip_time));
	while (--retry && ((REG_READ(PIPEADSL + reg_offset) & PIPE_LINE_CNT_MASK) >= dsl_threshold))
		udelay(delay_us);

	if (!retry) {
		DRM_ERROR("Pipe %d DSL is sticky.\n", pipe);
		return false;
	}

	dsl = REG_READ(PIPEADSL + reg_offset) & PIPE_LINE_CNT_MASK;
	if (dsl >= dsl_threshold)
		DRM_INFO("DSL is at line %u for pipe %d.\n", dsl, pipe);

	return true;
}


void DCCBAvoidFlipInVblankInterval(struct drm_device *dev, int pipe)
{
	if ((pipe == PIPEB) ||
		(is_panel_vid_or_cmd(dev) == MDFLD_DSI_ENCODER_DPI))
		is_vblank_period(dev, pipe);
}
