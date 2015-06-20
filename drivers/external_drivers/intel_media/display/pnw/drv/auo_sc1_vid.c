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
 * Ivan Chou <ivan.y.chou@intel.com>
 * Austin Hu <austin.hu@intel.com>
 */

#include "displays/auo_sc1_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"

#define GPIO_MIPI_PANEL_RESET 128

static u8 sc1_mcs_protect_on[] = {0xb0, 0x03, 0x00, 0x00};
static u8 sc1_mcs_protect_off[] = {0xb0, 0x04, 0x00, 0x00};
static u8 sc1_enter_sleep_mode[] = {0x10, 0x00, 0x00, 0x00};
static u8 sc1_exit_sleep_mode[] = {0x11, 0x00, 0x00, 0x00};
static u8 sc1_set_brightness[] = {0x51, 0x00, 0x00, 0x00};
static u8 sc1_select_CABC_mode[] = {0x55, 0x03, 0x00, 0x00};
static u8 sc1_enable_CABC_bl_off[] = {0x53, 0x28, 0x00, 0x00};
static u8 sc1_enable_CABC_bl_on[] = {0x53, 0x2c, 0x00, 0x00};
static u8 sc1_set_display_on[] = {0x29, 0x00, 0x00, 0x00};
static u8 sc1_set_display_off[] = {0x28, 0x00, 0x00, 0x00};
static u8 sc1_enter_low_power_mode[] = {0xb1, 0x01, 0x00, 0x00};

int mdfld_dsi_sc1_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	unsigned long wait_timeout;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	/*wait for 5ms*/
	wait_timeout = jiffies + (HZ / 200);
	while (time_before_eq(jiffies, wait_timeout))
		cpu_relax();

	return 0;
	/* Now In Sleep Mode */
}

static
void mdfld_dsi_sc1_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	/* [SC1] in bb2 is 3 */
	dsi_config->lane_count = 2;
	/* [SC1] in bb2 is MDFLD_DSI_DATA_LANE_3_1 */
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;
	hw_ctx->pll_bypass_mode = 1;
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
int mdfld_dsi_sc1_detect(struct mdfld_dsi_config *dsi_config)
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

static int mdfld_dsi_sc1_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 4, 0);

	/*change power state*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_exit_sleep_mode, 4, 0);

	msleep(120);

	/*enable CABC with backlight off*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_select_CABC_mode, 4, 0);
	mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_off, 4, 0);

	/*set display on*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_display_on, 4, 0);

	msleep(21);

	/*enable BLON , CABC*/
	if (drm_psb_enable_cabc) {
		mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_on, 4, 0);
		DRM_INFO("enable SC1 cabc\n");
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

static int mdfld_dsi_sc1_power_off(struct mdfld_dsi_config *dsi_config)
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

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 4, 0);

	/*change power state here*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_set_display_off, 4, 0);

	/*disable BLCON, disable CABC*/
	mdfld_dsi_send_mcs_long_hs(sender, sc1_enable_CABC_bl_off, 4, 0);
	printk(KERN_ALERT "disable SC1 cabc\n");

	msleep(21);

	mdfld_dsi_send_mcs_long_hs(sender, sc1_enter_sleep_mode, 4, 0);

	msleep(120);

	/*put panel into deep standby mode*/
	mdfld_dsi_send_gen_long_hs(sender, sc1_enter_low_power_mode, 4, 0);

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_on, 4, 0);
	return 0;
}

static int mdfld_dsi_sc1_set_brightness(struct mdfld_dsi_config *dsi_config,
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

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_off, 4, 0);

	/*update duty value*/
	sc1_set_brightness[0] =  (0x00000051 | (duty_val << 8));
	/* [SC1] change backlight control- brightness */
	mdfld_dsi_send_gen_long_hs(sender, sc1_set_brightness, 4, 0);

	mdfld_dsi_send_gen_long_hs(sender, sc1_mcs_protect_on, 4, 0);

	return 0;
}

static
int mdfld_dsi_sc1_panel_reset(struct mdfld_dsi_config *dsi_config)
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
struct drm_display_mode *sc1_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	/* [SC1] use john's value */
	mode->hdisplay = 540;
	mode->vdisplay = 960;
	/* HFP = 40, HSYNC = 10, HBP = 20 */
	mode->hsync_start = mode->hdisplay + 40;
	mode->hsync_end = mode->hsync_start + 10;
	mode->htotal = mode->hsync_end + 20;
	/* VFP = 4, VSYNC = 2, VBP = 4 */
	mode->vsync_start = mode->vdisplay + 4;
	mode->vsync_end = mode->vsync_start + 2;
	mode->vtotal = mode->vsync_end + 4;
	mode->clock = 16500;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static
void sc1_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (pipe == 0) {
		pi->width_mm = TMD_PANEL_WIDTH;
		pi->height_mm = TMD_PANEL_HEIGHT;
	}
}

void auo_sc1_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = sc1_vid_get_config_mode;
	p_funcs->get_panel_info = sc1_vid_get_panel_info;
	p_funcs->reset = mdfld_dsi_sc1_panel_reset;
	p_funcs->drv_ic_init = mdfld_dsi_sc1_ic_init;
	p_funcs->dsi_controller_init = mdfld_dsi_sc1_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_sc1_detect;
	p_funcs->power_on = mdfld_dsi_sc1_power_on;
	p_funcs->power_off = mdfld_dsi_sc1_power_off;
	p_funcs->set_brightness = mdfld_dsi_sc1_set_brightness;
}
