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
#include <asm/intel-mid.h>
#include "i915_drv.h"
#include "intel_drv.h"
#include "intel_dsi.h"
#include "intel_dsi_cmd.h"
#include "dsi_mod_jdi_lpm070w425b.h"


static void lpm070w425b_get_panel_info(int pipe,
				struct drm_connector *connector)
{
	DRM_DEBUG_KMS("\n");
	if (!connector)
		return;

	if (pipe == 0) {
		/* FIXME: the actual width is 94.5, height is 151.2 */
		connector->display_info.width_mm = 95;
		connector->display_info.height_mm = 151;
	}

	return;
}

bool lpm070w425b_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	intel_dsi->hs = true;
	intel_dsi->channel = 0;
	intel_dsi->lane_count = 4;
	intel_dsi->eotp_pkt = 0;
	intel_dsi->video_mode_type = DSI_VIDEO_NBURST_SEVENT;
	intel_dsi->pixel_format = VID_MODE_FORMAT_RGB888;
	intel_dsi->port_bits = 0;
	intel_dsi->turn_arnd_val = 0x30;
	intel_dsi->rst_timer_val = 0xffff;
	intel_dsi->hs_to_lp_count = 0x2f;
	intel_dsi->lp_byte_clk = 7;
	intel_dsi->bw_timer = 0x820;
	intel_dsi->clk_lp_to_hs_count = 0x2f;
	intel_dsi->clk_hs_to_lp_count = 0x16;
	intel_dsi->video_frmt_cfg_bits = 0x8;
	intel_dsi->dphy_reg = 0x2a18681f;
	intel_dsi->port = 0; /* PORT_A by default */
	intel_dsi->burst_mode_ratio = 100;
	intel_dsi->backlight_off_delay = 20;
	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;

	return true;
}

void lpm070w425b_create_resources(struct intel_dsi_device *dsi) { }

void lpm070w425b_dpms(struct intel_dsi_device *dsi, bool enable)
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

int lpm070w425b_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool lpm070w425b_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	return true;
}

void lpm070w425b_panel_reset(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
					PANEL0_BKLTCTL_GPIONC_5_PCONF0, 0x2000CC00);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
					PANEL0_BKLTCTL_GPIONC_5_PAD, 0x00000004);
	usleep_range(2000, 2500);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
					PANEL0_BKLTCTL_GPIONC_5_PAD, 0x00000005);
	msleep(20);
}

void  lpm070w425b_disable_panel_power(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
					PANEL0_BKLTCTL_GPIONC_5_PCONF0, 0x2000CC00);
	vlv_gpio_write(dev_priv, IOSF_PORT_GPIO_NC,
					PANEL0_BKLTCTL_GPIONC_5_PAD, 0x00000004);
	usleep_range(2000, 2500);
}

void lpm070w425b_send_otp_cmds(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);
	msleep(50);

	dsi_vc_dcs_write_0(intel_dsi, 0, 0x01);
	usleep_range(5000, 7000);
	{
		unsigned char data[] = {0xb0, 0x00};
		dsi_vc_generic_write(intel_dsi, 0, data, 2);
	}
	{
		unsigned char data[] = {0xb3, 0x14, 0x08, 0x00, 0x22, 0x00};
		dsi_vc_generic_write(intel_dsi, 0, data, 6);
	}
	{
		unsigned char data[] = {0xb4, 0x0c};
		dsi_vc_generic_write(intel_dsi, 0, data, 6);
	}
	{
		unsigned char data[] = {0xb6, 0x3a, 0xD3};
		dsi_vc_generic_write(intel_dsi, 0, data, 6);
	}
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x3A, 0x77);
		dsi_vc_dcs_write_1(intel_dsi, 0, 0x36, 0xC0);
	{
		unsigned char data[] = {0x2A, 0x00, 0x00, 0x04, 0xAF};
		dsi_vc_generic_write(intel_dsi, 0, data, 6);
	}
	{
		unsigned char data[] = {0x2B, 0x00, 0x00, 0x07, 0x7F};
		dsi_vc_generic_write(intel_dsi, 0, data, 6);
	}

}

void lpm070w425b_enable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	dsi_vc_dcs_write_0(intel_dsi, 0, 0x11);
	msleep(120);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x29);
}

void lpm070w425b_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	dsi_vc_dcs_write_0(intel_dsi, 0, 0x28);
	msleep(20);
	dsi_vc_dcs_write_0(intel_dsi, 0, 0x10);
	msleep(80);
}

enum drm_connector_status lpm070w425b_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

bool lpm070w425b_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *lpm070w425b_get_modes(struct intel_dsi_device *dsi)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	/* beta = 00, alpha = 45 */
	/* from renesas spec alpha + beta <= 45 */
	mode->hdisplay = 1200;
	mode->hsync_start = 1300;
	mode->hsync_end = 1340;
	mode->htotal = 1380;


	/* Added more vblank so more time for frame update */
	mode->vdisplay = 1920;
	mode->vsync_start = 1925;
	mode->vsync_end = 1930;
	mode->vtotal = 1935;

	mode->vrefresh = 60;

	mode->clock =  (mode->vrefresh * mode->vtotal *
		mode->htotal) / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

void lpm070w425b_dump_regs(struct intel_dsi_device *dsi) { }

void lpm070w425b_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops jdi_lpm070w425b_dsi_display_ops = {
	.init = lpm070w425b_init,
	.get_info = lpm070w425b_get_panel_info,
	.create_resources = lpm070w425b_create_resources,
	.dpms = lpm070w425b_dpms,
	.mode_valid = lpm070w425b_mode_valid,
	.mode_fixup = lpm070w425b_mode_fixup,
	.panel_reset = lpm070w425b_panel_reset,
	.disable_panel_power = lpm070w425b_disable_panel_power,
	.send_otp_cmds = lpm070w425b_send_otp_cmds,
	.enable = lpm070w425b_enable,
	.disable = lpm070w425b_disable,
	.detect = lpm070w425b_detect,
	.get_hw_state = lpm070w425b_get_hw_state,
	.get_modes = lpm070w425b_get_modes,
	.destroy = lpm070w425b_destroy,
	.dump_regs = lpm070w425b_dump_regs,
};
