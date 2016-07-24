/*
 * Copyright (c)  2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
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

#include "displays/auo_sc1_cmd.h"
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"

#define GPIO_MIPI_PANEL_RESET 128

static
void mdfld_auo_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	dsi_config->lane_count = 2;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_2_2;

	/* This is for 400 mhz. Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 1;

	hw_ctx->mipi_control = 0x0;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x15;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x2;
	hw_ctx->lp_byteclk = 0x0;
	hw_ctx->clk_lane_switch_time_cnt = 0xa0014;
	hw_ctx->dphy_param = 0x150c3408;
	hw_ctx->dbi_bw_ctrl = 0x820;
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | TE_TRIGGER_GPIO_PIN;
	hw_ctx->mipi |= dsi_config->lane_config;
	hw_ctx->dsi_func_prg = (0xa000 | dsi_config->lane_count);
}

static
struct drm_display_mode *auo_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

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
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	PSB_DEBUG_ENTRY("hdisplay is %d\n", mode->hdisplay);
	PSB_DEBUG_ENTRY("vdisplay is %d\n", mode->vdisplay);
	PSB_DEBUG_ENTRY("HSS is %d\n", mode->hsync_start);
	PSB_DEBUG_ENTRY("HSE is %d\n", mode->hsync_end);
	PSB_DEBUG_ENTRY("htotal is %d\n", mode->htotal);
	PSB_DEBUG_ENTRY("VSS is %d\n", mode->vsync_start);
	PSB_DEBUG_ENTRY("VSE is %d\n", mode->vsync_end);
	PSB_DEBUG_ENTRY("vtotal is %d\n", mode->vtotal);
	PSB_DEBUG_ENTRY("clock is %d\n", mode->clock);

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static int __mdfld_auo_dsi_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 cmd = 0;
	u8 param = 0;
	u8 param_set[4];
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* Send DCS commands. */
	cmd = exit_sleep_mode;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	msleep(120);

	cmd = write_display_brightness;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			(u8)0xff,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = write_ctrl_cabc;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			CABC_MODE_MOVING_IMAGE,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = write_ctrl_display;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			BRIGHT_CNTL_BLOCK_ON | DISPLAY_DIMMING_ON,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_tear_on;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			&param,
			1,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_column_address;
	param_set[0] = 0 >> 8;
	param_set[1] = 0;
	param_set[2] = 539 >> 8;
	param_set[3] = (u8)539;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			param_set,
			4,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_page_addr;
	param_set[0] = 0 >> 8;
	param_set[1] = 0;
	param_set[2] = 959 >> 8;
	param_set[3] = (u8)959;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			param_set,
			4,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = set_display_on;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	cmd = write_ctrl_display;
	err = mdfld_dsi_send_mcs_short_hs(sender,
			cmd,
			BRIGHT_CNTL_BLOCK_ON | DISPLAY_DIMMING_ON |
			BACKLIGHT_ON,
			1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	return err;
}

static int __mdfld_auo_dsi_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 cmd = 0;
	int err = 0;

	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/*enter sleep mode*/
	cmd = enter_sleep_mode;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	msleep(120);

	/*set tear off*/
	cmd = set_tear_off;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}


	/*set display off*/
	cmd = set_display_off;
	err = mdfld_dsi_send_dcs(sender,
			cmd,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", cmd);
		return err;
	}

	return err;
}

static
void auo_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	pi->width_mm = AUO_PANEL_WIDTH;
	pi->height_mm = AUO_PANEL_HEIGHT;
}

static
int mdfld_auo_dsi_cmd_detect(struct mdfld_dsi_config *dsi_config)
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
int mdfld_auo_dsi_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	u8 backlight_value;
	u8 param[4];
	int err = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	backlight_value = ((level * 0xff) / 100) & 0xff;

	param[0] = backlight_value;
	param[1] = 0x00;
	param[2] = 0x00;
	err = mdfld_dsi_send_dcs(sender,
			write_display_brightness,
			param,
			3,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);

	if (err)
		DRM_ERROR("DCS 0x%x sent failed\n", exit_sleep_mode);

	return 0;
}

static
int mdfld_auo_dsi_panel_reset(struct mdfld_dsi_config *dsi_config)
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
	mdelay(20);

	gpio_set_value_cansleep(mipi_reset_gpio, 1);

	/*minmum reset time*/
	mdelay(20);

	return 0;
}

void auo_sc1_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = auo_cmd_get_config_mode;
	p_funcs->get_panel_info = auo_cmd_get_panel_info;
	p_funcs->reset = mdfld_auo_dsi_panel_reset;
	p_funcs->dsi_controller_init = mdfld_auo_dsi_controller_init;
	p_funcs->detect = mdfld_auo_dsi_cmd_detect;
	p_funcs->set_brightness = mdfld_auo_dsi_cmd_set_brightness;
	p_funcs->power_on = __mdfld_auo_dsi_power_on;
	p_funcs->power_off = __mdfld_auo_dsi_power_off;
}
