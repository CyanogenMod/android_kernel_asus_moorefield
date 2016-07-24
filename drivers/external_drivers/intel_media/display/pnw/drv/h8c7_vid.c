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
 */

#include "displays/h8c7_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include <linux/sfi.h>

#undef DEBUG

static u8 h8c7_soft_reset[]      = {0x01};
static u8 h8c7_exit_sleep_mode[] = {0x11, 0x00, 0x00, 0x00};
static u8 h8c7_mcs_protect_off[] = {0xb9, 0xff, 0x83, 0x92};
static u8 h8c7_set_tear_on[] = {0x35, 0x00, 0x00, 0x00};
static u8 h8c7_set_brightness[] = {0x51, 0x00, 0x00, 0x00};
static u8 h8c7_set_full_brightness[] = {0x51, 0xff, 0x00, 0x00};
static u8 h8c7_turn_on_backlight[] = {0x53, 0x24, 0x00, 0x00};
static u8 h8c7_disable_cabc[] = {0x55, 0x00, 0x00, 0x00};
static u8 h8c7_ic_bias_current[] = {
	0xbf, 0x05, 0x60, 0x82,
	0x00, 0x00, 0x00, 0x00};
static u8 h8c7_set_power[] = {
	0xb1, 0x7c, 0x00, 0x44,
	0x24, 0x00, 0x0d, 0x0d,
	0x12, 0x1a, 0x3f, 0x3f,
	0x42, 0x72, 0x00, 0x00};
static u8 h8c7_set_power_dstb[] = {
	0xb1, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 h8c7_set_disp_reg[] = {
	0xb2, 0x0f, 0xc8, 0x05,
	0x0f, 0x08, 0x84, 0x00,
	0xff, 0x05, 0x0f, 0x04,
	0x20, 0x00, 0x00, 0x00};
static u8 h8c7_set_command_cyc[] = {
	0xb4, 0x00, 0x00, 0x05,
	0x00, 0xa0, 0x05, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x06, 0x07, 0x07,
	0x00, 0x00, 0x00, 0x00};
static u8 h8c7_set_mipi_ctrl[] = {0xba, 0x12, 0x83, 0x00};
static u8 h8c7_video_mode[] = {0xc2, 0x03, 0x00, 0x00};
static u8 h8c7_set_blanking_opt_2[]  = {0xc7, 0x00, 0x40, 0x00};
static u8 h8c7_set_panel[] = {0xcc, 0x08, 0x00, 0x00};
static u8 h8c7_set_eq_func_ltps[] = {0xd4, 0x0c, 0x00, 0x00};
static u8 h8c7_set_ltps_ctrl_output[] = {
	0xd5, 0x00, 0x08, 0x08,
	0x00, 0x44, 0x55, 0x66,
	0x77, 0xcc, 0xcc, 0xcc,
	0xcc, 0x00, 0x77, 0x66,
	0x55, 0x44, 0xcc, 0xcc,
	0xcc, 0xcc, 0x00, 0x00};
static u8 h8c7_set_video_cyc[] = {
	0xd8, 0x00, 0x00, 0x04,
	0x00, 0xa0, 0x04, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x06, 0x07, 0x07,
	0x00, 0x00, 0x00, 0x00};
static u8 h8c7_gamma_r[] = {
	0xe0, 0x3a, 0x3e, 0x3c,
	0x2f, 0x31, 0x32, 0x33,
	0x46, 0x04, 0x08, 0x0c,
	0x0d, 0x10, 0x0f, 0x11,
	0x10, 0x17, 0x3a, 0x3e,
	0x3c, 0x2f, 0x31, 0x32,
	0x33, 0x46, 0x04, 0x08,
	0x0c, 0x0d, 0x10, 0x0f,
	0x11, 0x10, 0x17, 0x00};
static u8 h8c7_gamma_g[] = {
	0xe1, 0x3b, 0x3e, 0x3d,
	0x31, 0x31, 0x32, 0x33,
	0x46, 0x03, 0x07, 0x0b,
	0x0d, 0x10, 0x0e, 0x11,
	0x10, 0x17, 0x3b, 0x3e,
	0x3d, 0x31, 0x31, 0x32,
	0x33, 0x46, 0x03, 0x07,
	0x0b, 0x0d, 0x10, 0x0e,
	0x11, 0x10, 0x17, 0x00};
static u8 h8c7_gamma_b[] = {
	0xe2, 0x01, 0x06, 0x07,
	0x2d, 0x2a, 0x32, 0x1f,
	0x40, 0x05, 0x0c, 0x0e,
	0x11, 0x14, 0x12, 0x13,
	0x0f, 0x18, 0x01, 0x06,
	0x07, 0x2d, 0x2a, 0x32,
	0x1f, 0x40, 0x05, 0x0c,
	0x0e, 0x11, 0x14, 0x12,
	0x13, 0x0f, 0x18, 0x00};
static u8 h8c7_enter_set_cabc[] = {
	0xc9, 0x1f, 0x00, 0x1e,
	0x1e, 0x00, 0x00, 0x00,
	0x01, 0xe3, 0x00, 0x00};
static u8 h8c7_mcs_protect_on[]      = {0xb9, 0x00, 0x00, 0x00};
static u8 h8c7_set_address_mode[]    = {0x36, 0x00, 0x00, 0x00};
static u8 h8c7_set_pixel_format[] = {0x3a, 0x70, 0x00, 0x00};
static u8 h8c7_set_display_on[] = {0x29, 0x00, 0x00, 0x00};
static u8 h8c7_set_display_off[] = {0x28, 0x00, 0x00, 0x00};
static u8 h8c7_enter_sleep_mode[] = {0x10, 0x00, 0x00, 0x00};

#define MIN_BRIGHTNESS_LEVEL 54
#define MAX_BRIGHTNESS_LEVEL 100

static int mdfld_h8c7_dpi_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	/**
	 * soft reset will let panel exit from deep standby mode and
	 * keep at standy mode.
	 */
	mdfld_dsi_send_gen_long_hs(sender, h8c7_soft_reset, 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/*wait for 5ms*/
	mdelay(5);

	/* set password and wait for 10ms. */
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_off, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set TE on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_tear_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_full_brightness, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* set backlight on and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_turn_on_backlight, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* disalble CABC and wait for 10ms. */
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_disable_cabc, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_ic_bias_current, 8, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_power, 16, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_disp_reg, 16, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_command_cyc, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_mipi_ctrl, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_video_mode, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_blanking_opt_2, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_panel, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_eq_func_ltps, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_ltps_ctrl_output, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_set_video_cyc, 24, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_r, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_g, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, h8c7_gamma_b, 36, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_enter_set_cabc, 10, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* disable password and wait for 10ms. */
	mdfld_dsi_send_gen_long_lp(sender, h8c7_mcs_protect_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_address_mode, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_set_pixel_format, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	return 0;
}

static
void mdfld_h8c7_dpi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 3;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_3_1;
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x1f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x28;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x20124e1a;

	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;

	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);
	/*setup mipi port configuration*/
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}

static
int mdfld_dsi_h8c7_detect(struct mdfld_dsi_config *dsi_config)
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

static int mdfld_dsi_h8c7_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* sleep out and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_exit_sleep_mode, 4, 0);
	wait_timeout = jiffies + (3 * HZ / 20);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/*set display on*/
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_display_on, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* FIXME Enable CABC later*/

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	return 0;
}

static int mdfld_dsi_h8c7_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;
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

	/* FIXME disable CABC later*/

	/*set display off*/
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_display_off, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/* sleep in and wait for 150ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_enter_sleep_mode, 4, 0);
	wait_timeout = jiffies + (3 * HZ / 20);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	/**
	 * MIPI spec shows it must wait 5ms
	 * before sneding next command
	 */
	mdelay(5);

	/*enter deep standby mode*/
	err = mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
	if (err)
		DRM_ERROR("Failed to turn off protection\n");

	err = mdfld_dsi_send_gen_long_hs(sender, h8c7_set_power_dstb, 14, 0);
	if (err)
		DRM_ERROR("Failed to enter DSTB\n");
	mdelay(5);
	mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_on, 4, 0);
	return 0;
}

int mdfld_dsi_h8c7_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (255 * level) / 100;
	h8c7_set_brightness[1] = duty_val;

	/* set backlight to full brightness and wait for 10ms. */
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_brightness, 4, 0);
	wait_timeout = jiffies + (HZ / 100);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	return 0;
}

#ifdef DEBUG
static
int mdfld_dsi_h8c7_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	static int mipi_reset_gpio;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("mipi-reset");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio, " \
				  "use default reset pin\n");
			ret = 128;
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
	mdelay(11);

	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	mdelay(5);

	return 0;
}
#endif

static
struct drm_display_mode *h8c7_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 720;
	mode->vdisplay = 1280;
	mode->hsync_start = 816;
	mode->hsync_end = 824;
	mode->htotal = 920;
	mode->vsync_start = 1284;
	mode->vsync_end = 1286;
	mode->vtotal = 1300;
	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static
void h8c7_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = PANEL_4DOT3_WIDTH;
	pi->height_mm = PANEL_4DOT3_HEIGHT;
}

void h8c7_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = h8c7_get_config_mode;
	p_funcs->get_panel_info = h8c7_get_panel_info;
	p_funcs->reset = NULL;
	p_funcs->drv_ic_init = mdfld_h8c7_dpi_ic_init;
	p_funcs->dsi_controller_init = mdfld_h8c7_dpi_controller_init;
	p_funcs->detect = mdfld_dsi_h8c7_detect;
	p_funcs->power_on = mdfld_dsi_h8c7_power_on;
	p_funcs->power_off = mdfld_dsi_h8c7_power_off;
	p_funcs->set_brightness = mdfld_dsi_h8c7_set_brightness;
}
