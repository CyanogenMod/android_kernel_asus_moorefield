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
 *	   Shobhit Kumar <shobhit.kumar@intel.com>
 *
 *
 */

#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_edid.h>
#include <drm/i915_drm.h>
#include <linux/slab.h>
#include <video/mipi_display.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_auo_b080xat.h"


static void b080xat_get_panel_info(int pipe, struct drm_connector *connector)
{
	DRM_DEBUG_KMS("\n");
	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = B080XAT_10x7_PANEL_WIDTH;
		connector->display_info.height_mm = B080XAT_10x7_PANEL_HEIGHT;
	}

	return;
}

bool b080xat_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	DRM_DEBUG_KMS("\n");

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
	intel_dsi->eotp_pkt = 0;
	intel_dsi->port_bits = 0;
	intel_dsi->dsi_clock_freq = 513;
	intel_dsi->video_mode_type = DSI_VIDEO_BURST;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB666_LOOSE;
	intel_dsi->escape_clk_div = ESCAPE_CLOCK_DIVIDER_1;
	intel_dsi->lp_rx_timeout = 0xffff;
	intel_dsi->turn_arnd_val = 0x3f;
	intel_dsi->rst_timer_val = 0xff;
	intel_dsi->init_count = 0x7d0;
	intel_dsi->hs_to_lp_count = 0x46;
	intel_dsi->lp_byte_clk = 4;
	intel_dsi->bw_timer = 0;
	intel_dsi->clk_lp_to_hs_count = 0x24;
	intel_dsi->clk_hs_to_lp_count = 0x0F;
	intel_dsi->video_frmt_cfg_bits = DISABLE_VIDEO_BTA;
	intel_dsi->dphy_reg = 0x3F10430D;
	intel_dsi->port = 0; /* PORT_A by default */
	intel_dsi->burst_mode_ratio = 100;
	intel_dsi->backlight_off_delay = 20;
	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;

	return true;
}

void b080xat_create_resources(struct intel_dsi_device *dsi) { }

void b080xat_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	if (enable) {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);

		dsi_vc_dcs_write_1(intel_dsi, 0, MIPI_DCS_SET_TEAR_ON, 0x00);

		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x14, 0x55);

	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
}

int b080xat_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool b080xat_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void b080xat_panel_reset(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL0_BKLTCTL_GPIONC_5_PCONF0, 0x2000CC00);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL0_BKLTCTL_GPIONC_5_PAD, 0x00000004);
	udelay(500);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL0_BKLTCTL_GPIONC_5_PAD, 0x00000005);
	usleep_range(10000, 12000);

}

void b080xat_disable_panel_power(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL0_BKLTCTL_GPIONC_5_PCONF0, 0x2000CC00);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
				PANEL0_BKLTCTL_GPIONC_5_PAD, 0x00000004);
	udelay(500);
}

void b080xat_disable(struct intel_dsi_device *dsi)
{
	/*struct intel_dsi *intel_dsi =
				container_of(dsi, struct intel_dsi, dev);*/

	DRM_DEBUG_KMS("\n");

//	dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);
//	msleep(20);
}

enum drm_connector_status b080xat_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

bool b080xat_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *b080xat_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode;
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 768;
	mode->hsync_start = 828;
	mode->hsync_end = 892;
	mode->htotal = 948;

	mode->vdisplay = 1024;
	mode->vsync_start = 1160;
	mode->vsync_end = 1210;
	mode->vtotal = 1240;

	mode->vrefresh = 60;

	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;
	intel_dsi->pclk = mode->clock;
	DRM_DEBUG_KMS("pclk : %d\n", intel_dsi->pclk);

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

void b080xat_dump_regs(struct intel_dsi_device *dsi) { }

void b080xat_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops auo_b080xat_dsi_display_ops = {
	.init = b080xat_init,
	.get_info = b080xat_get_panel_info,
	.create_resources = b080xat_create_resources,
	.dpms = b080xat_dpms,
	.mode_valid = b080xat_mode_valid,
	.mode_fixup = b080xat_mode_fixup,
	.panel_reset = b080xat_panel_reset,
	.disable_panel_power = b080xat_disable_panel_power,
	.disable = b080xat_disable,
	.detect = b080xat_detect,
	.get_hw_state = b080xat_get_hw_state,
	.get_modes = b080xat_get_modes,
	.destroy = b080xat_destroy,
	.dump_regs = b080xat_dump_regs,
};
