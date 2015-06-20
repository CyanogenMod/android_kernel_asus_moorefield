/*
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
 * Authors:
 * Geng Xiujun <xiujun.geng@intel.com>
 */

#include "displays/yb_cmi_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <asm/intel_scu_pmic.h>
#include <linux/gpio.h>
#include "psb_drv.h"

static u8 yb_cmi_set_address_mode[] = {0x36, 0x01};
/* enable cabc by default */
static u8 yb_cmi_panel_control_1[] = {0xb1, 0xf0};
static u8 yb_cmi_panel_control_2[] = {0xb2, 0x00};

/**
 * GPIO pin definition
 */
static int yb_cmi_gpio_panel_reset = 128;
static int yb_cmi_gpio_panel_stdby = 33;
static int yb_cmi_gpio_bklt_en = 162;

#define YB_CMI_PANEL_WIDTH	165
#define YB_CMI_PANEL_HEIGHT	105
static
int yb_cmi_vid_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	/* wait 10ms for VADD stable */
	mdelay(10);

	gpio_direction_output(yb_cmi_gpio_panel_stdby, 1);
	mdelay(10);
	gpio_direction_output(yb_cmi_gpio_panel_reset, 1);
	mdelay(10);
	gpio_direction_output(yb_cmi_gpio_panel_reset, 0);
	mdelay(10);
	gpio_direction_output(yb_cmi_gpio_panel_reset, 1);
	msleep(100);

	if (drm_psb_enable_cabc)
		yb_cmi_panel_control_1[1] = 0xf0;
	else
		yb_cmi_panel_control_1[1] = 0x30;

	mdfld_dsi_send_gen_short_lp(sender, yb_cmi_panel_control_1[0],
			yb_cmi_panel_control_1[1], 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_short_lp(sender, yb_cmi_panel_control_2[0],
			yb_cmi_panel_control_2[1], 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_short_lp(sender, yb_cmi_set_address_mode[0],
			yb_cmi_set_address_mode[1], 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	return 0;
}

static
void yb_cmi_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;

	struct csc_setting csc = {	.pipe = 0,
					.type = CSC_REG_SETTING,
					.enable_state = true,
					.data_len = CSC_REG_COUNT,
					.data.csc_reg_data = {
						0xFE504C2, 0xF57, 0x4AE0F96, 0xFBB, 0xB50007, 0x342}
				};
	struct gamma_setting gamma = {	.pipe = 0,
					.type = GAMMA_REG_SETTING,
					.enable_state = true,
					.data_len = GAMMA_10_BIT_TABLE_COUNT,
					.gamma_tableX100 = {
						0x000000, 0x040201, 0x080503, 0x0B0704,
						0x0E0906, 0x100B07, 0x130E09, 0x16100A,
						0x18120C, 0x1B140E, 0x1D160F, 0x201811,
						0x221A13, 0x251D14, 0x271F16, 0x292118,
						0x2C231A, 0x2E251B, 0x30271D, 0x33291F,
						0x352B21, 0x372D23, 0x392F24, 0x3C3126,
						0x3E3328, 0x40352A, 0x42372C, 0x443A2E,
						0x463C30, 0x483E31, 0x4B4033, 0x4D4235,
						0x4F4437, 0x514639, 0x53483B, 0x554A3D,
						0x574C3F, 0x594E41, 0x5B5043, 0x5D5245,
						0x5F5447, 0x615648, 0x63584A, 0x655A4C,
						0x675C4E, 0x695E50, 0x6B6052, 0x6D6254,
						0x6F6456, 0x716658, 0x73685A, 0x756A5C,
						0x776C5E, 0x796E60, 0x7B7062, 0x7D7264,
						0x7F7466, 0x807668, 0x82786A, 0x847A6C,
						0x867C6E, 0x887D70, 0x8A7F72, 0x8C8175,
						0x8E8377, 0x908579, 0x91877B, 0x93897D,
						0x958B7F, 0x978D81, 0x998F83, 0x9B9185,
						0x9D9387, 0x9E9589, 0xA0978B, 0xA2998D,
						0xA49B8F, 0xA69D91, 0xA89F94, 0xA9A196,
						0xABA398, 0xADA59A, 0xAFA79C, 0xB1A89E,
						0xB2AAA0, 0xB4ACA2, 0xB6AEA4, 0xB8B0A6,
						0xBAB2A9, 0xBBB4AB, 0xBDB6AD, 0xBFB8AF,
						0xC1BAB1, 0xC3BCB3, 0xC4BEB5, 0xC6C0B7,
						0xC8C2BA, 0xCAC4BC, 0xCBC6BE, 0xCDC7C0,
						0xCFC9C2, 0xD1CBC4, 0xD2CDC6, 0xD4CFC9,
						0xD6D1CB, 0xD8D3CD, 0xD9D5CF, 0xDBD7D1,
						0xDDD9D3, 0xDFDBD6, 0xE0DDD8, 0xE2DEDA,
						0xE4E0DC, 0xE5E2DE, 0xE7E4E0, 0xE9E6E3,
						0xEBE8E5, 0xECEAE7, 0xEEECE9, 0xF0EEEB,
						0xF1F0ED, 0xF3F2F0, 0xF5F4F2, 0xF7F5F4,
						0xF8F7F6, 0xFAF9F8, 0xFCFBFB, 0xFDFDFD}
					};

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xff;
	hw_ctx->high_low_switch_count = 0x17;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x3;
	hw_ctx->clk_lane_switch_time_cnt = 0x17000A;

	/*148Mbps dsi clock rate*/
	hw_ctx->dphy_param = 0x120a2b0c;

	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	if (dsi_config->enable_gamma_csc & ENABLE_CSC) {
		/* setting the tuned csc setting */
		drm_psb_enable_color_conversion = 1;
		mdfld_intel_crtc_set_color_conversion(dev, &csc);
	}

	if (dsi_config->enable_gamma_csc & ENABLE_GAMMA) {
		/* setting the tuned gamma setting */
		drm_psb_enable_gamma = 1;
		mdfld_intel_crtc_set_gamma(dev, &gamma);
	}
}

static
int yb_cmi_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		/*
		 * FIXME: WA to detect the panel connection status, and need to
		 * implement detection feature with get_power_mode DSI command.
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("hw begin failed\n");
			return -EAGAIN;
		}

		dpll_val = REG_READ(regs->dpll_reg);
		device_ready_val = REG_READ(regs->device_ready_reg);
		if ((device_ready_val & DSI_DEVICE_READY) &&
		    (dpll_val & DPLL_VCO_ENABLE)) {
			dsi_config->dsi_hw_context.panel_on = true;
			psb_enable_vblank(dev, pipe);
		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not detected!\n", __func__);
		}

		status = MDFLD_DSI_PANEL_CONNECTED;

		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n", __func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	/**
	 * FIXME:
	 * workaround for Yukka Beach power on, skip FW setting
	 */
	dsi_config->dsi_hw_context.panel_on = false;
	return status;
}

static
int yb_cmi_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*turn on backlight*/
	gpio_direction_output(yb_cmi_gpio_bklt_en, 1);
	msleep(100);

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}
	msleep(100);

	return 0;
}

static
int yb_cmi_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	gpio_direction_output(yb_cmi_gpio_bklt_en, 0);
	mdelay(10);

	/*send SHUT_DOWN packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}
	/*according HW DSI spec, need wait for 100ms*/
	msleep(100);

	gpio_direction_output(yb_cmi_gpio_panel_stdby, 0);
	msleep(100);
	gpio_direction_output(yb_cmi_gpio_panel_reset, 0);
	mdelay(10);

	return 0;
}

#define PWM0DUTYCYCLE	0x67
#define DUTY_VALUE_MAX	0x63
#define BRIGHTNESS_LEVEL_MAX	100
static
int yb_cmi_vid_set_brightness(struct mdfld_dsi_config *dsi_config, int level)
{
	int duty_val = 0;
	int ret = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	duty_val = ((DUTY_VALUE_MAX + 1) * level) / BRIGHTNESS_LEVEL_MAX;

	ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, duty_val);
	if (ret)
		DRM_ERROR("write brightness duty value faild\n");

	return ret;
}

struct drm_display_mode *yb_cmi_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 1024;
	mode->vdisplay = 600;
	mode->hsync_start = mode->hdisplay + 160;
	mode->hsync_end = mode->hsync_start + 80;
	mode->htotal = mode->hsync_end + 160;
	mode->vsync_start = mode->vdisplay + 12;
	mode->vsync_end = mode->vsync_start + 10;
	mode->vtotal = mode->vsync_end + 23;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void yb_cmi_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = YB_CMI_PANEL_WIDTH;
		pi->height_mm = YB_CMI_PANEL_HEIGHT;
	}
}

static int yb_cmi_vid_gpio_init(void)
{
	int ret = 0;

	ret = get_gpio_by_name("lcd-reset");
	if (ret < 0)
		DRM_ERROR("Faild to get panel reset GPIO\n");
	else
		yb_cmi_gpio_panel_reset = ret;

	ret = get_gpio_by_name("lcd-stdby");
	if (ret < 0)
		DRM_ERROR("Faild to get panel reset GPIO\n");
	else
		yb_cmi_gpio_panel_stdby = ret;

	ret = get_gpio_by_name("lcd-bklt");
	if (ret < 0)
		DRM_ERROR("Faild to get panel reset GPIO\n");
	else
		yb_cmi_gpio_bklt_en = ret;

	gpio_request(yb_cmi_gpio_panel_reset, "lcd-reset");
	gpio_request(yb_cmi_gpio_panel_stdby, "lcd-stdby");
	gpio_request(yb_cmi_gpio_bklt_en, "lcd-bklt");

	return 0;
}

#define PWM0CLKDIV1	0x61
#define PWM0CLKDIV0	0x62
static int yb_cmi_vid_brightness_init(void)
{
	int ret;

	PSB_DEBUG_ENTRY("\n");

	ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);
	if (!ret)
		ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x25);

	if (ret)
		pr_err("%s: PWM0CLKDIV set failed\n", __func__);
	else
		PSB_DEBUG_ENTRY("PWM0CLKDIV set to 0x%04x\n", 0x25);

	return ret;
}

void yb_cmi_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = yb_cmi_vid_get_config_mode;
	p_funcs->get_panel_info = yb_cmi_vid_get_panel_info;
	p_funcs->drv_ic_init = yb_cmi_vid_ic_init;
	p_funcs->dsi_controller_init = yb_cmi_vid_dsi_controller_init;
	p_funcs->detect = yb_cmi_vid_detect;
	p_funcs->power_on = yb_cmi_vid_power_on;
	p_funcs->power_off = yb_cmi_vid_power_off;
	p_funcs->set_brightness = yb_cmi_vid_set_brightness;

	/**
	 * FIXME:
	 * disable CABC by default due to HW bug, it'll be fixed in DV2
	 */
	drm_psb_enable_cabc = 0;

	ret = yb_cmi_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for yb cmi panel\n");

	ret = yb_cmi_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");
}
