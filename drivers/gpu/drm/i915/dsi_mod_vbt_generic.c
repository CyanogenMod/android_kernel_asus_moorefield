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
#include "dsi_mod_vbt_generic.h"

struct gpio_table gtable[] = {
	{ GPI0_NC_0_HV_DDI0_HPD, GPIO_NC_0_HV_DDI0_PAD, 0 },
	{ GPIO_NC_1_HV_DDI0_DDC_SDA, GPIO_NC_1_HV_DDI0_DDC_SDA_PAD, 0 },
	{ GPIO_NC_2_HV_DDI0_DDC_SCL, GPIO_NC_2_HV_DDI0_DDC_SCL_PAD, 0 },
	{ GPIO_NC_3_PANEL0_VDDEN, GPIO_NC_3_PANEL0_VDDEN_PAD, 0 },
	{ GPIO_NC_4_PANEL0_BLKEN, GPIO_NC_4_PANEL0_BLKEN_PAD, 0 },
	{ GPIO_NC_5_PANEL0_BLKCTL, GPIO_NC_5_PANEL0_BLKCTL_PAD, 0 },
	{ GPIO_NC_6_PCONF0, GPIO_NC_6_PAD, 0 },
	{ GPIO_NC_7_PCONF0, GPIO_NC_7_PAD, 0 },
	{ GPIO_NC_8_PCONF0, GPIO_NC_8_PAD, 0 },
	{ GPIO_NC_9_PCONF0, GPIO_NC_9_PAD, 0 },
	{ GPIO_NC_10_PCONF0, GPIO_NC_10_PAD, 0},
	{ GPIO_NC_11_PCONF0, GPIO_NC_11_PAD, 0}
};

static u8 *mipi_exec_send_packet(struct intel_dsi *intel_dsi, u8 *data)
{
	u8 type, byte, mode, vc, port;
	u16 len;

	byte = *data++;
	mode = (byte >> MIPI_TRANSFER_MODE_SHIFT) & 0x1;
	vc = (byte >> MIPI_VIRTUAL_CHANNEL_SHIFT) & 0x3;
	port = (byte >> MIPI_PORT_SHIFT) & 0x3;

	/* LP or HS mode */
	intel_dsi->hs = mode;

	intel_dsi->port = port;

	/* get packet type and increment the pointer */
	type = *data++;

	len = *((u16 *) data);
	data += 2;

	switch (type) {
	case MIPI_GENERIC_SHORT_WRITE_0_PARAM:
		dsi_vc_generic_write_0(intel_dsi, vc);
		break;
	case MIPI_GENERIC_SHORT_WRITE_1_PARAM:
		dsi_vc_generic_write_1(intel_dsi, vc, *data);
		break;
	case MIPI_GENERIC_SHORT_WRITE_2_PARAM:
		dsi_vc_generic_write_2(intel_dsi, vc, *data, *(data + 1));
		break;
	case MIPI_GENERIC_READ_0_PARAM:
	case MIPI_GENERIC_READ_1_PARAM:
	case MIPI_GENERIC_READ_2_PARAM:
		DRM_DEBUG_DRIVER("Generic Read not yet implemented or used\n");
		break;
	case MIPI_GENERIC_LONG_WRITE:
		dsi_vc_generic_write(intel_dsi, vc, data, len);
		break;
	case MIPI_MAN_DCS_SHORT_WRITE_0_PARAM:
		dsi_vc_dcs_write_0(intel_dsi, vc, *data);
		break;
	case MIPI_MAN_DCS_SHORT_WRITE_1_PARAM:
		dsi_vc_dcs_write_1(intel_dsi, vc, *data, *(data + 1));
		break;
	case MIPI_MAN_DCS_READ_0_PARAM:
		DRM_DEBUG_DRIVER("DCS Read not yet implemented or used\n");
		break;
	case MIPI_MAN_DCS_LONG_WRITE:
		dsi_vc_dcs_write(intel_dsi, vc, data, len);
		break;
	};

	data += len;

	return data;
}

static u8 *mipi_exec_delay(struct intel_dsi *intel_dsi, u8 *data)
{
	u32 delay = *((u32 *) data);

	DRM_DEBUG_DRIVER("MIPI: executing delay element\n");
	usleep_range(delay, delay + 10);
	data += 4;

	return data;
}

static u8 *mipi_exec_gpio(struct intel_dsi *intel_dsi, u8 *data)
{
	u8 gpio, action;
	u16 function, pad;
	u32 val;
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	DRM_DEBUG_DRIVER("MIPI: executing gpio element\n");
	gpio = *data++;

	/* pull up/down */
	action = *data++;

	function = gtable[gpio].function_reg;
	pad = gtable[gpio].pad_reg;

	if (!gtable[gpio].init) {
		/* program the function */
		vlv_gpio_nc_write(dev_priv, function, 0x2000CC00);
		gtable[gpio].init = 1;
	}

	val = 0x4 | action;

	/* pull up/down */
	vlv_gpio_nc_write(dev_priv, pad, val);

	return data;
}

FN_MIPI_ELEM_EXEC exec_elem[] = {
	NULL, /* reserved */
	mipi_exec_send_packet,
	mipi_exec_delay,
	mipi_exec_gpio,
	NULL, /* status read; later */
};

/*
 * MIPI Sequence from VBT #53 parsing logic
 * We have already separated each seqence during bios parsing
 * Following is generic execution function for any sequence
 */

static char *seq_name[] = {
	"UNDEFINED",
	"MIPI_SEQ_ASSERT_RESET",
	"MIPI_SEQ_INIT_OTP",
	"MIPI_SEQ_DISPLAY_ON",
	"MIPI_SEQ_DISPLAY_OFF",
	"MIPI_SEQ_DEASSERT_RESET"
};

static void generic_exec_sequence(struct intel_dsi *intel_dsi, char *sequence)
{
	u8 *data = sequence;
	FN_MIPI_ELEM_EXEC mipi_elem_exec;
	int index;

	if (!sequence)
		return;

	DRM_DEBUG_DRIVER("Starting MIPI sequence - %s\n", seq_name[*data]);

	/* go to the first element of the sequence */
	data++;

	/* parse each byte till we reach end of sequence byte - 0x00 */
	while (1) {
		index = *data;
		mipi_elem_exec = exec_elem[index];

		/* goto element payload */
		data++;

		/* execute the element specifc rotines */
		data = mipi_elem_exec(intel_dsi, data);

		/*
		 * After processing the element, data should point to
		 * next element or end of sequence
		 * check if have we reached end of sequence
		 */
		if (*data == 0x00)
			break;
	}
}

static void generic_get_panel_info(int pipe, struct drm_connector *connector)
{
	struct intel_connector *intel_connector = to_intel_connector(connector);

	DRM_DEBUG_KMS("\n");

	if (!connector)
		return;

	if (pipe == 0) {
		connector->display_info.width_mm = intel_connector->panel.fixed_mode->width_mm;
		connector->display_info.height_mm =  intel_connector->panel.fixed_mode->height_mm;
	}
	return;
}

bool generic_init(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct mipi_config *mipi_config = dev_priv->vbt.dsi.config;
	struct mipi_pps_data *pps = dev_priv->vbt.dsi.pps;
	struct drm_display_mode *mode = dev_priv->vbt.lfp_lvds_vbt_mode;
	u32 bits_per_pixel = 24;
	u32 tlpx_ns, extra_byte_count, bitrate, tlpx_ui;
	u32 ui_num, ui_den;
	u32 prepare_cnt, exit_zero_cnt, clk_zero_cnt, trail_cnt;
	u32 ths_prepare_ns, tclk_trail_ns;
	u32 tclk_prepare_clkzero, ths_prepare_hszero;
	u32 pclk, computed_ddr;
	u16 burst_mode_ratio;

	DRM_DEBUG_KMS("\n");

	intel_dsi->eotp_pkt = mipi_config->eot_disabled ? 0 : 1;
	intel_dsi->clock_stop = mipi_config->clk_stop ? 1 : 0;
	intel_dsi->lane_count = mipi_config->lane_cnt + 1;
	intel_dsi->pixel_format = mipi_config->videomode_color_format << 7;
	intel_dsi->dual_link = mipi_config->dual_link;
	intel_dsi->pixel_overlap = mipi_config->pixel_overlap;

	intel_dsi->port = 0;

	if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB666)
		bits_per_pixel = 18;
	else if (intel_dsi->pixel_format == VID_MODE_FORMAT_RGB565)
		bits_per_pixel = 16;

	pclk = mode->clock;

	/* In dual link mode each port needs half of pixel clock */
	if (intel_dsi->dual_link) {
		pclk = pclk / 2;

		/* in case of C0 and above setting we can enable pixel_overlap
		 * if needed by panel. In this case we need to increase the pixel
		 * clock for extra pixels
		 */
		if (IS_VALLEYVIEW_C0(dev) &&
				(intel_dsi->dual_link & MIPI_DUAL_LINK_FRONT_BACK)) {
			pclk += ceil_div(mode->vtotal * intel_dsi->pixel_overlap * 60,
				1000);
		}
	}
	intel_dsi->operation_mode = mipi_config->cmd_mode;
	intel_dsi->video_mode_type = mipi_config->vtm;
	intel_dsi->escape_clk_div = mipi_config->byte_clk_sel;
	intel_dsi->lp_rx_timeout = mipi_config->lp_rx_timeout;
	intel_dsi->turn_arnd_val = mipi_config->turn_around_timeout;
	intel_dsi->rst_timer_val = mipi_config->device_reset_timer;
	intel_dsi->init_count = mipi_config->master_init_timer;
	intel_dsi->bw_timer = mipi_config->dbi_bw_timer;
	intel_dsi->video_frmt_cfg_bits = mipi_config->bta ? DISABLE_VIDEO_BTA : 0;

	/* Burst Mode Ratio
	 * Target ddr frequency from VBT / non burst ddr freq
	 * multiply by 100 to preserver remainder
	 */
	if (intel_dsi->video_mode_type == DSI_VIDEO_BURST) {
		if (mipi_config->target_burst_mode_freq) {
			computed_ddr = (pclk * bits_per_pixel) /
								intel_dsi->lane_count;
			if (mipi_config->target_burst_mode_freq <
							computed_ddr) {
				DRM_ERROR("DDR clock is less than computed\n");
				return false;
			}

			burst_mode_ratio = ceil_div(
				mipi_config->target_burst_mode_freq * 100,
				computed_ddr);
			pclk = ceil_div(pclk * burst_mode_ratio, 100);
		} else {
			DRM_ERROR("Burst mode target is not set\n");
			return false;
		}
	} else
		burst_mode_ratio = 100;

	intel_dsi->burst_mode_ratio = burst_mode_ratio;
	intel_dsi->pclk = pclk;
	DRM_DEBUG_DRIVER("dsi->pclk = %d\n", intel_dsi->pclk);

	/* FIX ME:
	 * Check if pixel clock required is within the limits
	 */

	bitrate	= (pclk * bits_per_pixel) / intel_dsi->lane_count;

	switch (intel_dsi->escape_clk_div) {
	case 0:
		tlpx_ns = 50;
		break;
	case 1:
		tlpx_ns = 100;
		break;

	case 2:
		tlpx_ns = 200;
		break;
	default:
		tlpx_ns = 50;
		break;
	}

	switch (intel_dsi->lane_count) {
	case 1:
	case 2:
		extra_byte_count = 2;
		break;
	case 3:
		extra_byte_count = 4;
		break;
	case 4:
	default:
		extra_byte_count = 3;
		break;
	}

	/*
	 * ui(s) = 1/f [f in hz]
	 * ui(ns) = 10^9/f*10^6 [f in Mhz] -> 10^3/f(Mhz)
	 *
	 * LP byte clock = TLPX/8ui
	 *
	 * Since txddrclkhs_i is 2xUI, the count values programmed in
	 * DPHY param register are divided by 2
	 *
	 */

	/* in Kbps */
	ui_num = bitrate;
	ui_den = NS_MHZ_RATIO;

	tclk_prepare_clkzero = mipi_config->tclk_prepare_clkzero;
	ths_prepare_hszero = mipi_config->ths_prepare_hszero;

	/* B060 */
	intel_dsi->lp_byte_clk = ceil_div(tlpx_ns * ui_num, 8 * ui_den);

	/* count values in UI = (ns value) * (bitrate / (2 * 10^6)) */
	/* prepare count */
	ths_prepare_ns =
		(mipi_config->ths_prepare >  mipi_config->tclk_prepare) ?
				mipi_config->ths_prepare :
				mipi_config->tclk_prepare;

	prepare_cnt = ceil_div(ths_prepare_ns * ui_num,	ui_den * 2);

	/* exit zero count */
	exit_zero_cnt = ceil_div(
				(ths_prepare_hszero - ths_prepare_ns) * ui_num,
				ui_den * 2
				);

	/*
	 * Exit zero  is unified val ths_zero and ths_exit
	 * minimum value for ths_exit = 110ns
	 * min (exit_zero_cnt * 2) = 110/UI
	 * exit_zero_cnt = 55/UI
	 */
	 if (exit_zero_cnt < (55 * ui_num / ui_den))
		if ((55 * ui_num) % ui_den)
			exit_zero_cnt += 1;

	/* clk zero count */
	clk_zero_cnt = ceil_div(
			(tclk_prepare_clkzero -	ths_prepare_ns)
			* ui_num, 2 * ui_den);

	/* trail count */
	tclk_trail_ns = (mipi_config->tclk_trail > mipi_config->ths_trail) ?
			mipi_config->tclk_trail : mipi_config->ths_trail;
	trail_cnt = ceil_div(tclk_trail_ns * ui_num, 2 * ui_den);

	if (prepare_cnt > PREPARE_CNT_MAX ||
		exit_zero_cnt > EXIT_ZERO_CNT_MAX ||
		clk_zero_cnt > CLK_ZERO_CNT_MAX ||
		trail_cnt > TRAIL_CNT_MAX)
		DRM_DEBUG_DRIVER("Values crossing maximum limits\n");

	if (prepare_cnt > PREPARE_CNT_MAX)
		prepare_cnt = PREPARE_CNT_MAX;

	if (exit_zero_cnt > EXIT_ZERO_CNT_MAX)
		exit_zero_cnt = EXIT_ZERO_CNT_MAX;

	if (clk_zero_cnt > CLK_ZERO_CNT_MAX)
		clk_zero_cnt = CLK_ZERO_CNT_MAX;

	if (trail_cnt > TRAIL_CNT_MAX)
		trail_cnt = TRAIL_CNT_MAX;

	/* B080 */
	intel_dsi->dphy_reg = exit_zero_cnt << 24 | trail_cnt << 16 |
						clk_zero_cnt << 8 | prepare_cnt;

	/*
	 * LP to HS switch count = 4TLPX + PREP_COUNT * 2 + EXIT_ZERO_COUNT * 2
	 *					+ 10UI + Extra Byte Count
	 *
	 * HS to LP switch count = THS-TRAIL + 2TLPX + Extra Byte Count
	 * Extra Byte Count is calculated according to number of lanes.
	 * High Low Switch Count is the Max of LP to HS and
	 * HS to LP switch count
	 *
	 */
	tlpx_ui = ceil_div(tlpx_ns * ui_num, ui_den);

	/* B044 */
	intel_dsi->hs_to_lp_count =
		ceil_div(
			4 * tlpx_ui + prepare_cnt * 2 +
			exit_zero_cnt * 2 + 10,
			8);

	intel_dsi->hs_to_lp_count += extra_byte_count;

	/* B088 */
	/* LP -> HS for clock lanes
	 * LP clk sync + LP11 + LP01 + tclk_prepare + tclk_zero +
	 *						extra byte count
	 * 2TPLX + 1TLPX + 1 TPLX(in ns) + prepare_cnt * 2 + clk_zero_cnt *
	 *					2(in UI) + extra byte count
	 * In byteclks = (4TLPX + prepare_cnt * 2 + clk_zero_cnt *2 (in UI)) /
	 *					8 + extra byte count
	 */
	intel_dsi->clk_lp_to_hs_count =
		ceil_div(
			4 * tlpx_ui + prepare_cnt * 2 +
			clk_zero_cnt * 2,
			8);

	intel_dsi->clk_lp_to_hs_count += extra_byte_count;

	/* HS->LP for Clock Lanes
	 * Low Power clock synchronisations + 1Tx byteclk + tclk_trail +
	 *						Extra byte count
	 * 2TLPX + 8UI + (trail_count*2)(in UI) + Extra byte count
	 * In byteclks = (2*TLpx(in UI) + trail_count*2 +8)(in UI)/8 +
	 *						Extra byte count
	 */
	intel_dsi->clk_hs_to_lp_count =
		ceil_div(2 * tlpx_ui + trail_cnt * 2 + 8,
			8);
	intel_dsi->clk_hs_to_lp_count += extra_byte_count;

	DRM_DEBUG_KMS("EOT %s\n", intel_dsi->eotp_pkt ? "ENABLED" : "DISABLED");
	DRM_DEBUG_KMS("CLOCKSTOP %s\n", intel_dsi->clock_stop ?
						"ENABLED" : "DISABLED");
	DRM_DEBUG_KMS("Mode %s\n", intel_dsi->operation_mode ? "COMMAND" : "VIDEO");

	if (intel_dsi->dual_link == MIPI_DUAL_LINK_FRONT_BACK)
		DRM_DEBUG_KMS("Dual link: MIPI_DUAL_LINK_FRONT_BACK\n");
	else if (intel_dsi->dual_link == MIPI_DUAL_LINK_PIXEL_ALT)
		DRM_DEBUG_KMS("Dual link: MIPI_DUAL_LINK_PIXEL_ALT\n");
	else
		DRM_DEBUG_KMS("Dual link: NONE\n");

	DRM_DEBUG_KMS("Pixel Format %d\n", intel_dsi->pixel_format);
	DRM_DEBUG_KMS("TLPX %d\n", intel_dsi->escape_clk_div);
	DRM_DEBUG_KMS("LP RX Timeout 0x%x\n", intel_dsi->lp_rx_timeout);
	DRM_DEBUG_KMS("Turnaround Timeout 0x%x\n", intel_dsi->turn_arnd_val);
	DRM_DEBUG_KMS("Init Count 0x%x\n", intel_dsi->init_count);
	DRM_DEBUG_KMS("HS to LP Count 0x%x\n", intel_dsi->hs_to_lp_count);
	DRM_DEBUG_KMS("LP Byte Clock %d\n", intel_dsi->lp_byte_clk);
	DRM_DEBUG_KMS("DBI BW Timer 0x%x\n", intel_dsi->bw_timer);
	DRM_DEBUG_KMS("LP to HS Clock Count 0x%x\n", intel_dsi->clk_lp_to_hs_count);
	DRM_DEBUG_KMS("HS to LP Clock Count 0x%x\n", intel_dsi->clk_hs_to_lp_count);
	DRM_DEBUG_KMS("BTA %s\n",
			intel_dsi->video_frmt_cfg_bits & DISABLE_VIDEO_BTA ?
			"DISABLED" : "ENABLED");
	DRM_DEBUG_KMS("B060 = 0x%Xx, B080 = 0x%x, B044 = 0x%x, B088 = 0x%x\n",
			intel_dsi->lp_byte_clk, intel_dsi->dphy_reg, intel_dsi->hs_to_lp_count,
			(intel_dsi->clk_lp_to_hs_count << LP_HS_SSW_CNT_SHIFT) |
			(intel_dsi->clk_hs_to_lp_count << HS_LP_PWR_SW_CNT_SHIFT));

	/* delays in VBT are in unit of 100us, so need to convert
	 * here in ms
	 * Delay (100us) * 100 /1000 = Delay / 10 (ms) */
	intel_dsi->backlight_off_delay = pps->bl_disable_delay / 10;
	intel_dsi->backlight_on_delay = pps->bl_enable_delay / 10;
	intel_dsi->panel_on_delay = pps->panel_on_delay / 10;
	intel_dsi->panel_off_delay = pps->panel_off_delay / 10;
	intel_dsi->panel_pwr_cycle_delay = pps->panel_power_cycle_delay / 10;

	intel_dsi->send_shutdown = true;
	intel_dsi->shutdown_pkt_delay = 20;

	return true;
}

void generic_create_resources(struct intel_dsi_device *dsi) { }

void generic_dpms(struct intel_dsi_device *dsi, bool enable)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);

	DRM_DEBUG_KMS("\n");

	/* Basic code. Might need rework */
	if (enable) {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_EXIT_SLEEP_MODE);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_ON);
	} else {
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_SET_DISPLAY_OFF);
		dsi_vc_dcs_write_0(intel_dsi, 0, MIPI_DCS_ENTER_SLEEP_MODE);
	}
}

int generic_mode_valid(struct intel_dsi_device *dsi,
		   struct drm_display_mode *mode)
{
	return MODE_OK;
}

bool generic_mode_fixup(struct intel_dsi_device *dsi,
		    const struct drm_display_mode *mode,
		    struct drm_display_mode *adjusted_mode) {
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	/* If desired mode is different from panel's supported mode, we will try to
	use scaling to achieve. But the modeset should be of proper mode timings
	So make the adjusted mode same as panel supported mode */
	if (dev_priv->scaling_reqd) {
		adjusted_mode->hdisplay =
			dev_priv->vbt.lfp_lvds_vbt_mode->hdisplay;
		adjusted_mode->vdisplay =
			dev_priv->vbt.lfp_lvds_vbt_mode->vdisplay;

		/* Configure hw mode */
		drm_mode_set_name(adjusted_mode);
		drm_mode_set_crtcinfo(adjusted_mode, 0);
		adjusted_mode->type |= DRM_MODE_TYPE_PREFERRED;
		DRM_DEBUG_DRIVER("Sending %dx%d as adjusted mode",
			adjusted_mode->hdisplay, adjusted_mode->vdisplay);
	}
	return true;
}

void generic_panel_reset(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->vbt.dsi.sequence[MIPI_SEQ_ASSERT_RESET];

	generic_exec_sequence(intel_dsi, sequence);
}

void generic_disable_panel_power(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->vbt.dsi.sequence[MIPI_SEQ_DEASSERT_RESET];

	generic_exec_sequence(intel_dsi, sequence);
}

void generic_send_otp_cmds(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->vbt.dsi.sequence[MIPI_SEQ_INIT_OTP];

	generic_exec_sequence(intel_dsi, sequence);
}

void generic_enable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->vbt.dsi.sequence[MIPI_SEQ_DISPLAY_ON];

	generic_exec_sequence(intel_dsi, sequence);
}

void generic_disable(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;

	char *sequence = dev_priv->vbt.dsi.sequence[MIPI_SEQ_DISPLAY_OFF];

	generic_exec_sequence(intel_dsi, sequence);
}

enum drm_connector_status generic_detect(struct intel_dsi_device *dsi)
{
	return connector_status_connected;
}

bool generic_get_hw_state(struct intel_dsi_device *dev)
{
	return true;
}

struct drm_display_mode *generic_get_modes(struct intel_dsi_device *dsi)
{
	struct intel_dsi *intel_dsi = container_of(dsi, struct intel_dsi, dev);
	struct drm_device *dev = intel_dsi->base.base.dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_display_mode *target = dev_priv->vbt.lfp_lvds_vbt_mode;

	/* If desired mode is different from panel's supported mode, we will try to
	use scaling to achieve */
	if (dev_priv->scaling_reqd) {
		target = drm_mode_duplicate(dev, (const struct drm_display_mode *)
			dev_priv->vbt.lfp_lvds_vbt_mode);
		if (!target) {
			DRM_ERROR("Out of memory, scaling will fail\n");
			return dev_priv->vbt.lfp_lvds_vbt_mode;
		}

		/* Fixme: Updating only the X and Y resolution of the desired
		mode, not full timings */
		target->hdisplay =
			dev_priv->vbt.target_res.xres;
		target->vdisplay =
			dev_priv->vbt.target_res.yres;
		DRM_DEBUG_DRIVER("Sending target timings");
	}

	target->vrefresh = drm_mode_vrefresh(target);
	target->type |= DRM_MODE_TYPE_PREFERRED;
	return target;
}

void generic_dump_regs(struct intel_dsi_device *dsi) { }

void generic_destroy(struct intel_dsi_device *dsi) { }

/* Callbacks. We might not need them all. */
struct intel_dsi_dev_ops vbt_generic_dsi_display_ops = {
	.init = generic_init,
	.get_info = generic_get_panel_info,
	.create_resources = generic_create_resources,
	.dpms = generic_dpms,
	.mode_valid = generic_mode_valid,
	.mode_fixup = generic_mode_fixup,
	.panel_reset = generic_panel_reset,
	.disable_panel_power = generic_disable_panel_power,
	.send_otp_cmds = generic_send_otp_cmds,
	.enable = generic_enable,
	.disable = generic_disable,
	.detect = generic_detect,
	.get_hw_state = generic_get_hw_state,
	.get_modes = generic_get_modes,
	.destroy = generic_destroy,
	.dump_regs = generic_dump_regs
};
