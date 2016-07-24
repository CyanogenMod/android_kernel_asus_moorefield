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
 */

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include <linux/sfi.h>
#include "psb_drv.h"

/**
 * set GPIO_MIPI_PANEL_RESET to 57 for CTP VV platform
 * set GPIO_MIPI_PANEL_RESET to 117 (core_gpio[21] for Hydra)
 */
#define GPIO_MIPI_PANEL_RESET 57
static u8 r63311_mcs_protect_off[] = {0xb0, 0x04};
static u8 r63311_nop_command[]  = {0x00};
static u8 r63311_nop_command1[] = {0x00};
static u8 r63311_interface_setting[] = {
	0xb3, 0x14, 0x00, 0x00,
	0x00, 0x00, 0x00};
static u8 r63311_dsi_control[] = {0xb6, 0x3a, 0xd3};
static u8 r63311_display_setting1[] = {
	0xc1, 0x84, 0x60, 0x50,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0c, 0x01,
	0x58, 0x73, 0xae, 0x31,
	0x20, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x10, 0x10, 0x10, 0x10,
	0x00, 0x00, 0x00, 0x22,
	0x02, 0x02, 0x00};
static u8 r63311_display_setting2[] = {
	0xc2, 0x30, 0xf7, 0x80,
	0x0c, 0x08, 0x00, 0x00};
static u8 r63311_source_timing_setting[] = {
	0xc4, 0x70, 0x00, 0x00,
	0x00, 0x00, 0x04, 0x00,
	0x00, 0x00, 0x11, 0x06,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x04, 0x00, 0x00,
	0x00, 0x11, 0x06};
static u8 r63311_ltps_timing_setting[] = {
	0xc6, 0x06, 0x6d, 0x06,
	0x6d, 0x06, 0x6d, 0x00,
	0x00, 0x00, 0x00, 0x06,
	0x6d, 0x06, 0x6d, 0x06,
	0x6d, 0x15, 0x19, 0x07,
	0x00, 0x01, 0x06, 0x6d,
	0x06, 0x6d, 0x06, 0x6d,
	0x00, 0x00, 0x00, 0x00,
	0x06, 0x6d, 0x06, 0x6d,
	0x06, 0x6d, 0x15, 0x19,
	0x07};
static u8 r63311_gamma_setting_a[] = {
	0xc7, 0x00, 0x09, 0x14,
	0x1d, 0x2c, 0x44, 0x40,
	0x52, 0x5f, 0x67, 0x6b,
	0x70, 0x00, 0x09, 0x14,
	0x1d, 0x2c, 0x44, 0x40,
	0x52, 0x5f, 0x67, 0x6b,
	0x70};
static u8 r63311_gamma_setting_b[] = {
	0xc8, 0x00, 0x09, 0x14,
	0x1d, 0x2c, 0x44, 0x40,
	0x52, 0x5f, 0x67, 0x6b,
	0x70, 0x00, 0x09, 0x14,
	0x1d, 0x2c, 0x44, 0x40,
	0x52, 0x5f, 0x67, 0x6b,
	0x70};
static u8 r63311_gamma_setting_c[] = {
	0xc9, 0x00, 0x09, 0x14,
	0x1d, 0x2c, 0x44, 0x40,
	0x52, 0x5f, 0x67, 0x6b,
	0x70, 0x00, 0x09, 0x14,
	0x1d, 0x2c, 0x44, 0x40,
	0x52, 0x5f, 0x67, 0x6b,
	0x70};
static u8 r63311_panel_interface_control[] = {0xcc, 0x09};
static u8 r63311_power_setting[] = {
	0xd0, 0x00, 0x00, 0x19,
	0x18, 0x99, 0x99, 0x19,
	0x01, 0x89, 0x00, 0x55,
	0x19, 0x99, 0x01};
static u8 r63311_power_setting_for_internal[] = {
	0xd3, 0x1b, 0x33, 0xbb,
	0xcc, 0xc4, 0x33, 0x33,
	0x33, 0x00, 0x01, 0x00,
	0xa0, 0xd8, 0xa0, 0x0d,
	0x39, 0x33, 0x44, 0x22,
	0x70, 0x02, 0x39, 0x03,
	0x3d, 0xbf, 0x00};
static u8 r63311_vcom_setting[] = {
	0xd5, 0x06, 0x00, 0x00,
	0x01, 0x44, 0x01, 0x44};
static u8 r63311_vcom_setting1[] = {
	0xd5, 0x06, 0x00, 0x00,
	0x01, 0x44, 0x01, 0x44};
static u8 r63311_exit_sleep_mode[] = {0x11, 0x00, 0x00, 0x00};
static u8 r63311_set_display_off[] = {0x28, 0x00, 0x00, 0x00};
static u8 r63311_enter_sleep_mode[] = {0x10, 0x00, 0x00, 0x00};
static u8 r63311_display_on[] = {0x29, 0x00, 0x00, 0x00};
static u8 r63311_color_enhance[] = {
	0xca, 0x01, 0x80, 0xff,
	0xff, 0xff, 0xff, 0xdc,
	0xdc, 0x13, 0x20, 0x80,
	0x80, 0x0a, 0x4a, 0x37,
	0xa0, 0x55, 0xf8, 0x0c,
	0x0c, 0x20, 0x10, 0x3f,
	0x3f, 0x00, 0x00, 0x10,
	0x10, 0x3f, 0x3f, 0x3f,
	0x3f};
static u8 r63311_cabc_parameter[] = {
	0xb8, 0x18, 0x80, 0x18,
	0x18, 0xcf, 0x1f, 0x00,
	0x0c, 0x12, 0x6c, 0x11,
	0x6c, 0x12, 0x0c, 0x12,
	0xda, 0x6d, 0xff, 0xff,
	0x10, 0x67, 0xa3, 0xdb,
	0xfb, 0xff};
static u8 r63311_cabc_parameter1[] = {
	0xb9, 0x00, 0x30, 0x18,
	0x18, 0x9f, 0x1f, 0x80};
static u8 r63311_cabc_parameter2[] = {
	0xba, 0x00, 0x30, 0x04,
	0x40, 0x9f, 0x1f, 0xb7};
static u8 r63311_dimming_function[] = {
	0xce, 0x00, 0x06, 0x00,
	0xc1, 0x00, 0x0e, 0x14};
static u8 r63311_enable_pwm[] = {0x53, 0x2c};
static u8 r63311_cabc_function[] = {0x55, 0x03};

struct drm_display_mode *r63311_vid_get_config_mode(void)
{

	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 1080;
	mode->vdisplay = 1920;
	mode->hsync_start = 1080+120;
	mode->hsync_end = 1080+120+8;
	mode->htotal = 1080+120+8+60;
	mode->vsync_start = 1920+12;
	mode->vsync_end = 1920+12+4;
	mode->vtotal = 1920+12+4+12;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void r63311_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (pipe == 0) {
		pi->width_mm = TMD_PANEL_WIDTH;
		pi->height_mm = TMD_PANEL_HEIGHT;
	}
}

static int mdfld_dsi_r63311_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	mdfld_dsi_send_gen_long_lp(sender, r63311_mcs_protect_off, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_nop_command, 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_nop_command1, 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_interface_setting, 7, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_dsi_control, 3, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_hs(sender, r63311_color_enhance, 33, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	if (drm_psb_enable_cabc) {
		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter, 26, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter1, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			return -EIO;

		udelay(5);
		mdfld_dsi_send_gen_long_hs(sender, r63311_cabc_parameter2, 8, 0);
		if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
			return -EIO;
	}

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_display_setting1, 35, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_display_setting2, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_source_timing_setting, 23, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_ltps_timing_setting, 41, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_gamma_setting_a, 25, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_gamma_setting_b, 25, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_gamma_setting_c, 25, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_panel_interface_control, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_power_setting, 15, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_power_setting_for_internal, 27, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_vcom_setting, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_lp(sender, r63311_vcom_setting1, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_gen_long_hs(sender, r63311_dimming_function, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_mcs_long_hs(sender, r63311_enable_pwm, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	udelay(5);
	mdfld_dsi_send_mcs_long_hs(sender, r63311_cabc_function, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
#if 0
	mdfld_dsi_send_gen_long_lp(sender, pr2_exit_sleep_mode, 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
	mdelay(120);
	mdfld_dsi_send_gen_long_lp(sender, pr2_display_on, 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
#endif
	mdelay(1);

	return 0;
}

static void
mdfld_dsi_r63311_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x18;
	hw_ctx->device_reset_timer = 0xffff ;
	hw_ctx->high_low_switch_count = 0x35;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x30;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->clk_lane_switch_time_cnt = 0x2B0014;
	hw_ctx->dphy_param = 0x2A18681F;
	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}


static int mdfld_dsi_r63311_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	u32 cmd, cabc_result;
	struct drm_device *dev = dsi_config->dev;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	cmd = 0x29;
	err = mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
	if (err) {
		DRM_ERROR("Failed to send cmd 0x11\n");
		return err;
	}
	mdelay(10);

	cmd = 0x11;

	err = mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
	if (err) {
		DRM_ERROR("Failed to send cmd 0x29\n");
		return err;
	}
	msleep(150);

	return 0;
}

static int mdfld_dsi_r63311_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	struct drm_device *dev = dsi_config->dev;
	int err;
	u32 cmd;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	cmd = 0x28;
	mdfld_dsi_send_gen_long_lp(sender, &cmd, 1, 0);
	mdelay(10);

	cmd = 0x10;
	mdfld_dsi_send_mcs_long_hs(sender, &cmd, 1, 0);
	mdelay(100);

	return 0;
}


static int mdfld_dsi_r63311_detect(struct mdfld_dsi_config *dsi_config)
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

	return status;
}

static int mdfld_dsi_r63311_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	static bool gpio_requested;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!gpio_requested) {
		ret = gpio_request(GPIO_MIPI_PANEL_RESET, "gfx");
		if (ret) {
			DRM_ERROR("Failed to request gpio %d, ret = %d\n",
					GPIO_MIPI_PANEL_RESET, ret);
			return ret;
		}
	}
	gpio_requested = true;
	/**
	 * removing GPIO 55 and 43 reset since they're Lenovo platform specific.
	 * gpio_direction_output(43, 1);
	 */
	gpio_direction_output(GPIO_MIPI_PANEL_RESET, 0);

	/**
	 * removing GPIO 55 and 43 reset since they're Lenovo platform specific.
	 * gpio_direction_output(55, 1);
	 */
	mdelay(2);
	gpio_direction_output(GPIO_MIPI_PANEL_RESET, 1);
	mdelay(5);

	return 0;
}

static int mdfld_dsi_r63311_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u32 backlight_value;
	u8 data[3] = {0};
	u32 cmd = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	backlight_value = ((level * 0xfff) / 100) & 0xfff;
	data[0] = 0x51;
	/* DBV[11] to DBV[8] */
	data[1] = (backlight_value>>8) & 0xff;
	/* DBV[7] to DBV[0] */
	data[2] = backlight_value & 0xff;
	mdfld_dsi_send_mcs_long_hs(sender, data, 3, 0);
	mdelay(5);
	mdfld_dsi_send_mcs_short_hs(sender, 0x53, 0x24, 1, 0);
	mdelay(1);

	return 0;
}

void r63311_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = r63311_vid_get_config_mode;
	p_funcs->get_panel_info = r63311_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_r63311_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_r63311_drv_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_r63311_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_r63311_detect;
	p_funcs->power_on = mdfld_dsi_r63311_power_on;
	p_funcs->power_off = mdfld_dsi_r63311_power_off;
	p_funcs->set_brightness = mdfld_dsi_r63311_set_brightness;
}

static int jdi_lcd_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: jdi panel detected\n", __func__);
	intel_mid_panel_register(r63311_vid_init);

	return 0;
}

struct platform_driver jdi_r63311_lcd_driver = {
	.probe	= jdi_lcd_probe,
	.driver	= {
		.name	= "JDI_R63311",
		.owner	= THIS_MODULE,
	},
};
