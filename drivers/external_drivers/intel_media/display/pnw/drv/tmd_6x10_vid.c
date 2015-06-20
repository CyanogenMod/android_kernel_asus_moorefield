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
 * Jim Liu <jim.liu@intel.com>
 * Jackie Li <yaodong.li@intel.com>
 * Gideon Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
 */

#include "displays/tmd_6x10_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"

static u8 pr2_mcs_protect_off[]      = {0xb0, 0x04};
static u8 pr2_pixel_format[]         = {0xb3, 0x00, 0x87, 0x00};
static u8 pr2_dsi_control[]          = {0xb6, 0x30, 0x83, 0x00};
static u8 pr2_control[]              = {0xc3, 0x00, 0x19, 0x00};
static u8 pr2_test_mode_0[]          = {0xc4, 0x03, 0x00, 0x00};
static u8 pr2_can_skip[]             = {0xc6, 0x00, 0x00, 0x00};
static u8 pr2_source_amplifiers[]    = {0xd2, 0xb3, 0x00, 0x00};
static u8 pr2_power_supply_circuit[] = {0xd3, 0x33, 0x03, 0x00};
static u8 pr2_vreg_setting[]         = {0xd5, 0x00, 0x00, 0x00};
static u8 pr2_test_mode_2[]          = {0xd6, 0x01, 0x00, 0x00};
static u8 pr2_timing_control_2[]     = {0xd9, 0x5b, 0x7f, 0x05};
static u8 pr2_vcs_setting[]          = {0xdd, 0x53, 0x00, 0x00};
static u8 pr2_vcom_dc_setting[]      = {0xde, 0x43, 0x00, 0x00};
static u8 pr2_mcs_protect_on[]       = {0xb0, 0x03, 0x00, 0x00};
static u8 pr2_set_address_mode[]     = {0x36, 0x00, 0x00, 0x00};
static u8 pr2_set_pixel_format[]     = {0x3a, 0x70, 0x00, 0x00};
static u8 pr2_exit_sleep_mode[]      = {0x11, 0x00, 0x00, 0x00};
static u8 pr2_set_display_on[]       = {0x29, 0x00, 0x00, 0x00};
static u8 pr2_set_display_off[]      = {0x28, 0x00, 0x00, 0x00};
static u8 pr2_enter_sleep_mode[]     = {0x10, 0x00, 0x00, 0x00};
static u8 pr2_enter_low_power_mode[] = {0xb1, 0x01, 0x00, 0x00};
static u8 pr2_panel_driving[] = {
	0xc0, 0x01, 0xfe, 0x65,
	0x00, 0x00, 0x00, 0x00};
static u8 pr2_v_timing[] = {
	0xc1, 0x00, 0x10, 0x00,
	0x01, 0x00, 0x00, 0x00};
static u8 pr2_h_timing[] = {
	0xc5, 0x00, 0x01, 0x05,
	0x04, 0x5e, 0x00, 0x00,
	0x00, 0x00, 0x0b, 0x17,
	0x05, 0x00, 0x00, 0x00};
static u8 pr2_gamma_set_a[] = {
	0xc8, 0x0a, 0x15, 0x18,
	0x1b, 0x1c, 0x0d, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};
static u8 pr2_gamma_set_b[] = {
	0xc9, 0x0d, 0x1d, 0x1f,
	0x1f, 0x1f, 0x10, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};
static u8 pr2_gamma_set_c[] = {
	0xca, 0x1e, 0x1f, 0x1e,
	0x1d, 0x1d, 0x10, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};
static u8 pr2_charge_pump_setting[] = {
	0xd0, 0x02, 0x00, 0xa3,
	0xb8, 0x00, 0x00, 0x00};
static u8 pr2_test_mode_1[] = {
	0xd1, 0x10, 0x14, 0x53,
	0x64, 0x00, 0x00, 0x00};
static u8 pr2_timing_control_0[] = {
	0xd7, 0x09, 0x00, 0x84,
	0x81, 0x61, 0xbc, 0xb5,
	0x05, 0x00, 0x00, 0x00};
static u8 pr2_timing_control_1[] = {
	0xd8, 0x04, 0x25, 0x90,
	0x4c, 0x92, 0x00, 0x00};
static u8 pr2_white_balance[] = {
	0xcb, 0x00, 0x00, 0x00,
	0x1c, 0x00, 0x00, 0x00};
static u8 pr2_test_mode_3[] = {
	0xe4, 0x00, 0x00, 0x22,
	0xaa, 0x00, 0x00, 0x00};
static u8 pr2_backlight_control_1[] = {
	0xb8, 0x01, 0x0f, 0x0f,
	0xff, 0xff, 0xc8, 0xc8,
	0x0f, 0x0f, 0x18, 0x18,
	0x90, 0x90, 0x00, 0x02,
	0x0c, 0x1d, 0x37, 0x5a,
	0x87, 0xbe, 0xff, 0x00};
static u8 pr2_backlight_control_2[] = {
	0xb9, 0x01, 0xcc, 0x00,
	0x18, 0x00, 0x00, 0x00};

#define MIPI_RESET_GPIO_DEFAULT	128

static
int mdfld_dsi_pr2_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	mdfld_dsi_send_gen_long_lp(sender, pr2_mcs_protect_off, 2, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_pixel_format, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_dsi_control, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_panel_driving, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_v_timing, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_control, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_test_mode_0, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_h_timing, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_can_skip, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_gamma_set_a, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_gamma_set_b, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_gamma_set_c, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_charge_pump_setting, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_test_mode_1, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_source_amplifiers, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_power_supply_circuit, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_vreg_setting, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_test_mode_2, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_timing_control_0, 12, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_timing_control_1, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_timing_control_2, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_white_balance, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_vcs_setting, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_vcom_dc_setting, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_test_mode_3, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, pr2_mcs_protect_on, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, pr2_set_address_mode, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, pr2_set_pixel_format, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	return 0;
}

static
void mdfld_dsi_pr2_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 3;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_3_1;

	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 1;

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

	/*420Mbps DSI data rate*/
	hw_ctx->dphy_param = 0x160D3610;

	/*400Mbps bypass mode DSI data rate*/
	if (hw_ctx->pll_bypass_mode && hw_ctx->cck_div)
		hw_ctx->dphy_param = 0x150C340F;
	/*800Mbps bypass mode DSI data rate*/
	if (hw_ctx->pll_bypass_mode && !hw_ctx->cck_div)
		hw_ctx->dphy_param = 0x2A18681F;
	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/*setup mipi port configuration*/
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}

static
int mdfld_dsi_pr2_detect(struct mdfld_dsi_config *dsi_config)
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

static
int mdfld_dsi_pr2_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

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
	/**
	 * According HW DSI spec, here need wait for 100ms. This is
	 * not necessary as the code is anyway sleeping 141 ms later
	 * in this function
	 */

	mdfld_dsi_send_gen_long_hs(sender, pr2_mcs_protect_off, 2, 0);
	/*change power state*/
	mdfld_dsi_send_mcs_long_hs(sender, pr2_exit_sleep_mode, 4, 0);

	/*120ms delay is needed between enter and exit sleep mode per spec*/
	msleep(120);

	/*enable PWMON*/
	pr2_backlight_control_2[1] |= 0x01;
	mdfld_dsi_send_mcs_long_hs(sender,
			pr2_backlight_control_2, 8, 0);

	/*set display on*/
	mdfld_dsi_send_mcs_long_hs(sender, pr2_set_display_on, 4, 0);
	/* Per panel spec, 21ms delay is needed */
	msleep(21);

	/*Enable BLON , CABC*/
	if (drm_psb_enable_cabc) {
		pr2_backlight_control_1[1] |= 0x01;
		mdfld_dsi_send_gen_long_hs(sender,
				pr2_backlight_control_1, 24, 0);
		DRM_INFO("%s: enable pr2 cabc\n", __func__);
	}

	return 0;
}

static
int mdfld_dsi_pr2_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*send SHUT_DOWN packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}
	/*according HW DSI spec, need wait for 100ms*/
	msleep(100);

	mdfld_dsi_send_gen_long_hs(sender, pr2_mcs_protect_off, 2, 0);
	/*change power state here*/
	mdfld_dsi_send_mcs_long_hs(sender, pr2_set_display_off, 4, 0);

	/*disable BLCON, disable CABC*/
	pr2_backlight_control_1[1] &= ~0x01;
	mdfld_dsi_send_gen_long_hs(sender,
			pr2_backlight_control_1, 6, 0);
	/* Per panel spec, 21ms delay is needed */
	msleep(21);

	mdfld_dsi_send_mcs_long_hs(sender, pr2_enter_sleep_mode, 4, 0);
	/*120ms delay is needed between enter and exit sleep mode per spec*/
	msleep(120);

	/*put panel into deep standby mode*/
	mdfld_dsi_send_gen_long_hs(sender,
			pr2_enter_low_power_mode, 4, 0);

	mdfld_dsi_send_gen_long_hs(sender, pr2_mcs_protect_on, 4, 0);

	return 0;
}

static
int mdfld_dsi_pr2_set_brightness(struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (255 * level) / 100;

	mdfld_dsi_send_gen_long_hs(sender, pr2_mcs_protect_off, 2, 0);

	/*update duty value*/
	pr2_backlight_control_2[2] = duty_val;

	if (drm_psb_enable_cabc) {
		if (level < 50) {
			pr2_backlight_control_1[1] = 0x00;
			pr2_backlight_control_1[2] = 0x0f;
			pr2_backlight_control_1[3] = 0x0f;
		} else if (level < 66) {
			/* Case 10% */
			pr2_backlight_control_1[1] = 0x01;
			pr2_backlight_control_1[2] = 0x07;
			pr2_backlight_control_1[3] = 0x07;
			pr2_backlight_control_1[6] = 0xe4;
			pr2_backlight_control_1[7] = 0xe4;
		} else if (level < 82) {
			/* Case 20% */
			pr2_backlight_control_1[1] = 0x01;
			pr2_backlight_control_1[2] = 0x0b;
			pr2_backlight_control_1[3] = 0x0b;
			pr2_backlight_control_1[6] = 0xd4;
			pr2_backlight_control_1[7] = 0xd4;
		} else {
			/* Case 30% */
			pr2_backlight_control_1[1] = 0x01;
			pr2_backlight_control_1[2] = 0x0f;
			pr2_backlight_control_1[3] = 0x0f;
			pr2_backlight_control_1[6] = 0xc8;
			pr2_backlight_control_1[7] = 0xc8;
		}

		mdfld_dsi_send_gen_long_hs(sender,
				pr2_backlight_control_1, 24, 0);
	}

	mdfld_dsi_send_gen_long_hs(sender, pr2_backlight_control_2, 8, 0);
	mdfld_dsi_send_gen_long_hs(sender, pr2_mcs_protect_on, 4, 0);

	return 0;
}

static
int mdfld_dsi_pr2_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	static int mipi_reset_gpio;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("mipi-reset");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio, " \
				  "use default reset pin\n");
			ret = MIPI_RESET_GPIO_DEFAULT;
		}

		mipi_reset_gpio = ret;

		ret = gpio_request(mipi_reset_gpio, "mipi_display");
		if (ret) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return -EINVAL;
		}

		gpio_direction_output(mipi_reset_gpio, 0);
	}

	gpio_set_value_cansleep(mipi_reset_gpio, 0);
	/* HW reset need minmum 3ms */
	usleep_range(3000, 4000);

	gpio_set_value_cansleep(mipi_reset_gpio, 1);

	/* After reset and Before sending IC init sequence,
	 * need wait 7ms, this time has confirmed ok by panel vender*/
	usleep_range(7000, 7100);

	return 0;
}

struct drm_display_mode *pr2_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 800;
	mode->vdisplay = 1024;
	mode->hsync_start = 823;
	mode->hsync_end = 831;
	mode->htotal = 847;
	mode->vsync_start = 1031;
	mode->vsync_end = 1033;
	mode->vtotal = 1035;
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void pr2_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = TMD_PANEL_WIDTH;
		pi->height_mm = TMD_PANEL_HEIGHT;
	}
}

void tmd_6x10_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = pr2_vid_get_config_mode;
	p_funcs->get_panel_info = pr2_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_pr2_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_pr2_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_pr2_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_pr2_detect;
	p_funcs->power_on = mdfld_dsi_pr2_power_on;
	p_funcs->power_off = mdfld_dsi_pr2_power_off;
	p_funcs->set_brightness = mdfld_dsi_pr2_set_brightness;
}

static int tmd_6x10_lcd_vid_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: TMD_6X10 panel detected\n", __func__);
	intel_mid_panel_register(tmd_6x10_vid_init);

	return 0;
}

struct platform_driver tmd_lcd_driver = {
	.probe	= tmd_6x10_lcd_vid_probe,
	.driver	= {
		.name	= "TMD BB PRx",
		.owner	= THIS_MODULE,
	},
};
