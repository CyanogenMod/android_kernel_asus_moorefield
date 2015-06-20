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
 * Austin Hu <austin.hu@intel.com>
 */

#include "displays/gi_sony_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"

#define GPIO_MIPI_PANEL_RESET 128

static u8 gi_sony_enter_sleep_mode[] = {0x10, 0x00, 0x00, 0x00};
static u8 gi_sony_exit_sleep_mode[] = {0x11, 0x00, 0x00, 0x00};
static u8 gi_sony_set_brightness[] = {0x51, 0x00, 0x00, 0x00};
static u8 gi_sony_select_CABC_mode[] = {0x55, 0x03, 0x00, 0x00};
static u8 gi_sony_enable_CABC_bl_off[] = {0x53, 0x28, 0x00, 0x00};
static u8 gi_sony_enable_CABC_bl_on[] = {0x53, 0x2c, 0x00, 0x00};
static u8 gi_sony_set_display_on[] = {0x29, 0x00, 0x00, 0x00};
static u8 gi_sony_set_display_off[] = {0x28, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_row_add[] = {0x2b, 0x00, 0x00, 0x01,
	0xdf, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_address_mode[] = {0x36, 0xd0, 0x00, 0x00};
static u8 gi_l5f3_set_column_add[] = {
	0x2a, 0x00, 0x00, 0x01,
	0x3f, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_pixel_format[] = {0x3a, 0x77, 0x00, 0x00};
static u8 gi_l5f3_set_te_scanline[] = {0x44, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_tear_on[] = {0x35, 0x00, 0x00, 0x00};
static u8 gi_l5f3_passwd1_on[] = {0xf0, 0x5a, 0x5a, 0x00};
static u8 gi_l5f3_passwd2_on[] = {0xf1, 0x5a, 0x5a, 0x00};
static u8 gi_l5f3_dstb_on[] = {0xdf, 0x01, 0x00, 0x00};
static u8 gi_l5f3_set_disctl[] = {
	0xf2, 0x3b, 0x4a, 0x0f,
	0x04, 0x10, 0x08, 0x08,
	0x00, 0x08, 0x08, 0x00,
	0x00, 0x00, 0x00, 0x4c,
	0x04, 0x10, 0x20, 0x20};
static u8 gi_l5f3_set_pwrctl[] = {
	0xf4, 0x07, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x44, 0x05,
	0x00, 0x44, 0x05, 0x00};
static u8 gi_l5f3_set_vcmctl[] = {
	0xf5, 0x00, 0x15, 0x17,
	0x00, 0x00, 0x02, 0x00,
	0x00, 0x00, 0x00, 0x15,
	0x17, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_srcctl[] = {
	0xf6, 0x01, 0x00, 0x08,
	0x03, 0x01, 0x00, 0x01,
	0x00, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_ifctl[] = {
	0xf7, 0x98, 0x81, 0x10,
	0x03, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_panelctl[] = {0xf8, 0x55, 0x00, 0x00};
static u8 gi_l5f3_set_gammasel[] = {0xf9, 0x27, 0x00, 0x00};
static u8 gi_l5f3_set_pgammactl[] = {
	0xfa, 0x0c, 0x04, 0x06,
	0x1e, 0x1f, 0x21, 0x24,
	0x21, 0x2d, 0x2f, 0x2e,
	0x2e, 0x0f, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_ngammactl[] = {
	0xfb, 0x0c, 0x04, 0x0f,
	0x2e, 0x2e, 0x2f, 0x2d,
	0x21, 0x24, 0x21, 0x1f,
	0x1e, 0x06, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00};
static u8 gi_l5f3_set_miectl1[] = {0xc0, 0x80, 0x80, 0x10};
static u8 gi_l5f3_set_bcmode[] = {0xc1, 0x13, 0x00, 0x00};
static u8 gi_l5f3_set_wrmiectl2[] = {
	0xc2, 0x08, 0x00, 0x00,
	0x01, 0xdf, 0x00, 0x00,
	0x01, 0x3f, 0x00, 0x00};
static u8 gi_l5f3_set_wrblctl[] = {0xc3, 0x00, 0x10, 0x20};
static u8 gi_l5f3_passwd1_off[] = {0xf0, 0xa5, 0xa5, 0x00};
static u8 gi_l5f3_disable_cabc[] = {0x55, 0x00, 0x00, 0x00};
static u8 gi_l5f3_exit_sleep_mode[] = {0x11, 0x00, 0x00, 0x00};
static u8 gi_l5f3_turn_on_backlight[] = {0x53, 0x24, 0x00, 0x00};

/* FIXME Optimize the delay time after PO.  */
static
int mdfld_gi_l5f3_dpi_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_column_add, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_row_add, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_address_mode, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_pixel_format, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_te_scanline, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_set_tear_on, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_passwd1_on, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_disctl, 20, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_pwrctl, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_vcmctl, 16, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_srcctl, 12, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_ifctl, 8, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_panelctl, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_gammasel, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_pgammactl, 20, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_ngammactl, 20, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_miectl1, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_bcmode, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_wrmiectl2, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_set_wrblctl, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_gen_long_lp(sender, gi_l5f3_passwd1_off, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_turn_on_backlight, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_disable_cabc, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, gi_l5f3_exit_sleep_mode, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	return 0;
}

static
void mdfld_dsi_gi_sony_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	/* [SC1] in bb2 is 3 */
	dsi_config->lane_count = 1;
	/* [SC1] in bb2 is MDFLD_DSI_DATA_LANE_3_1 */
	/*
	 * FIXME: JLIU7 dsi_config->lane_config =
	 * MDFLD_DSI_DATA_LANE_4_0;
	 */
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	hw_ctx->pll_bypass_mode = 0;
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xff;
	hw_ctx->high_low_switch_count = 0x25;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x0;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150a600f;
	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;
	/*set up func_prg*/
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);
	/*setup mipi port configuration*/
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;
}

static
int mdfld_dsi_gi_sony_detect(struct mdfld_dsi_config *dsi_config)
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

	/*FIXME:
	 * IFIW should also apply patch 53200 to avoid screen flash
	 * otherwise, driver should do power off and on
	 */
	dsi_config->dsi_hw_context.panel_on = 0;

	return status;
}

static int mdfld_dsi_gi_sony_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*change power state*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_exit_sleep_mode, 4, 0);

	msleep(120);

	/*enable CABC with backlight off*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_select_CABC_mode, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_off, 4, 0);

	/*set display on*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_set_display_on, 4, 0);

	msleep(21);

	/*enable BLON , CABC*/
	if (1) {
		mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_on,
				4, 0);
		printk(KERN_ALERT "enable SC1 cabc\n");
	}

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	return 0;
}

static int mdfld_dsi_gi_sony_power_off(struct mdfld_dsi_config *dsi_config)
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

	/*change power state here*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_set_display_off, 4, 0);

	/*disable BLCON, disable CABC*/
	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enable_CABC_bl_off, 4, 0);
	printk(KERN_ALERT "disable SC1 cabc\n");

	msleep(21);

	mdfld_dsi_send_mcs_long_hs(sender, gi_sony_enter_sleep_mode, 4, 0);

	msleep(120);

	/* DSTB, deep standby sequenc */
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_passwd2_on, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, gi_l5f3_dstb_on, 4, 0);
	PSB_DEBUG_ENTRY("putting panel into deep sleep standby\n");

	msleep(50);

	return 0;
}

static int mdfld_dsi_gi_sony_set_brightness(struct mdfld_dsi_config *dsi_config,
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

	/*update duty value*/
	gi_sony_set_brightness[0] =  (0x00000051 | (duty_val << 8));
	/* [SC1] change backlight control- brightness */
	mdfld_dsi_send_gen_long_hs(sender, gi_sony_set_brightness, 4, 0);

	return 0;
}

static
int mdfld_dsi_gi_sony_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	static int mipi_reset_gpio;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("mipi-reset");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio\n");
			return -EINVAL;
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

	/*low pluse wudtg in sleep in mode*/
	mdelay(11);

	gpio_set_value_cansleep(mipi_reset_gpio, 1);

	/*minmum reset time*/
	mdelay(20);

	return 0;
}

static
struct drm_display_mode *gi_sony_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 320;
	mode->vdisplay = 480;
	/* HFP = 2, HSYNC = 8, HBP = 20 */
	mode->hsync_start = mode->hdisplay + 20;
	mode->hsync_end = mode->hsync_start + 8;
	mode->htotal = mode->hsync_end + 20;
	/* VFP = 4, VSYNC = 4, VBP = 12 */
	mode->vsync_start = mode->vdisplay + 4;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 12;
	mode->clock = (mode->htotal * mode->vtotal) * 60 / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static
void gi_sony_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = PANEL_3DOT47_WIDTH;
	pi->height_mm = PANEL_3DOT47_HEIGHT;
}

void gi_sony_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = gi_sony_vid_get_config_mode;
	p_funcs->get_panel_info = gi_sony_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_gi_sony_panel_reset;
	p_funcs->drv_ic_init = mdfld_gi_l5f3_dpi_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_gi_sony_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_gi_sony_detect;
	p_funcs->power_on = mdfld_dsi_gi_sony_power_on;
	p_funcs->power_off = mdfld_dsi_gi_sony_power_off;
	p_funcs->set_brightness = mdfld_dsi_gi_sony_set_brightness;
}
