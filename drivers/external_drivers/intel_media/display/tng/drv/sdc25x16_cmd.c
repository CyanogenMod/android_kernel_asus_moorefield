/*
 * Copyright © 2014 Intel Corporation
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
 * Faxing Lu <faxing.lu@intel.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_pkg_sender.h"
#include <asm/intel_scu_pmic.h>
#include "displays/sdc25x16_cmd.h"

static int vdd_1_8v_gpio;

static u8 sdc_column_addr[] = {
			0x2a, 0x00, 0x00, 0x04, 0xff};
static u8 sdc_page_addr[] = {
			0x2b, 0x00, 0x00, 0x06, 0x3f};
#if 0
static	u8 sdc_set_300nit[34] = { 0x83,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x00,
								0x80, 0x80,
								0x00,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x00,
								0x80, 0x80,
								0x00,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x80,
								0x80, 0x00,
								0x80, 0x80,
								0x00};
static	u8 sdc_set_AID[] = { 0x85, 0x06, 0x00 };
static	u8 sdc_300_ELVSS[] = { 0xbb, 0x19};
static	u8 sdc_set_ACL_off[] = { 0xbb, 0x10};
#endif


static	u8 sdc_gamma_setting[] = { 0x82, 0x1f};
static	u8 sdc_AOR_setting[] = { 0x85, 0x6, 0};
static	u8 sdc_global_para_53[] = { 0xb0, 0x34};
static	u8 sdc_ELVSS_para[] = { 0xbb, 0xF};
static	u8 sdc_global_para_47[] = { 0xb0, 0x2e};
static	u8 sdc_gamma_update[] = { 0xbb, 0x1};


static	u8 sdc_global_para_70[] = { 0xb0, 0x45};
static	u8 sdc_set_ACL_on[] = { 0xbb, 0x12};

static u8 sdc_brightness_list[21][5] = {
			{0x1f, 0x06, 0x00, 0x00, 0x0f},
			{0x00, 0x06, 0x00, 0x19, 0x1b},
			{0x01, 0x06, 0x00, 0x1a, 0x1c},
			{0x02, 0x06, 0x00, 0x1c, 0x1e},
			{0x03, 0x06, 0x00, 0x1d, 0x1f},
			{0x04, 0x58, 0x00, 0x1d, 0x1f},
			{0x05, 0x87, 0x00, 0x1d, 0x1f},
			{0x06, 0xad, 0x00, 0x1d, 0x1f},
			{0x07, 0xad, 0x00, 0x1f, 0x1f},
			{0x08, 0xad, 0x00, 0x1f, 0x1f},
			{0x09, 0xad, 0x00, 0x1f, 0x1f},
			{0x0a, 0xad, 0x00, 0x1f, 0x1f},
			{0x0b, 0xad, 0x00, 0x1f, 0x1f},
			{0x0c, 0xad, 0x00, 0x1f, 0x1f},
			{0x0d, 0xb3, 0x00, 0x1f, 0x1f},
			{0x0e, 0xd2, 0x00, 0x1f, 0x1f},
			{0x0f, 0xe8, 0x00, 0x1f, 0x1f},
			{0x10, 0xfe, 0x00, 0x1f, 0x1f},
			{0x11, 0x11, 0x01, 0x1f, 0x1f},
			{0x12, 0x23, 0x01, 0x1f, 0x1f},
			{0x13, 0x32, 0x01, 0x1f, 0x1f}
			};

static
int sdc25x16_set_dimming(struct mdfld_dsi_pkg_sender *sender,
	int index)
{
	int err = 0;

	PSB_DEBUG_ENTRY("\n");
	sdc_gamma_setting[1] = sdc_brightness_list[index][0];
	sdc_AOR_setting[1] = sdc_brightness_list[index][1];
	sdc_AOR_setting[2] = sdc_brightness_list[index][2];
	sdc_ELVSS_para[1] = sdc_brightness_list[index][3];
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_gamma_setting,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_setting\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_AOR_setting,
			3, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_setting\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_53,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_53\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_ELVSS_para,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_ELVSS_para\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_47,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_47\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_gamma_update,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_update\n",
		__func__, __LINE__);
		goto set_dimming_err;
	}

	return 0;
set_dimming_err:
	err = -EIO;
	return err;
}

static
int sdc25x16_cmd_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
		= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	int i;

	PSB_DEBUG_ENTRY("\n");
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}
	i = 10;
	err = sdc25x16_set_dimming(sender, i);
	if (err) {
		DRM_ERROR("%s: %d: brightness setting error\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_70,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_70\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_set_ACL_on,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_set_ACL_on\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_global_para_47,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_global_para_47\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_gen_long_hs(sender,
			sdc_gamma_update,
			2, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: sdc_gamma_update\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_long_hs(sender,
			sdc_column_addr,
			5, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Column Address\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_long_hs(sender,
			sdc_page_addr,
			5, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Page Address\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_tear_on, 0x0, 1,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Tear On\n",
		__func__, __LINE__);
		goto ic_init_err;
	}
	return 0;
ic_init_err:
	err = -EIO;
	return err;
}

static
void sdc25x16_cmd_controller_init(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
				&dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x0;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->high_low_switch_count = 0x2b;
	hw_ctx->clk_lane_switch_time_cnt =  0x2b0014;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->dphy_param = 0x2a18681f;
	hw_ctx->eot_disable = 0x1 | BIT8;
	hw_ctx->init_count = 0xf0;
	hw_ctx->dbi_bw_ctrl = 1024;
	hw_ctx->hs_ls_dbi_enable = 0x0;
	hw_ctx->dsi_func_prg = ((DBI_DATA_WIDTH_OPT2 << 13) |
				dsi_config->lane_count);

	hw_ctx->mipi = SEL_FLOPPED_HSTX	| PASS_FROM_SPHY_TO_AFE |
		DUAL_LINK_ENABLE | DUAL_LINK_CAPABLE | TE_TRIGGER_GPIO_PIN | BANDGAP_CHICKEN_BIT;
	hw_ctx->video_mode_format = 0xf;
}
static
int sdc25x16_cmd_panel_connection_detect(
	struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		status = MDFLD_DSI_PANEL_CONNECTED;
	} else {
		DRM_INFO("%s: do NOT support dual panel\n",
		__func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static
int sdc25x16_cmd_power_on(
	struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	/* Set Display on 0x29 */
	err = mdfld_dsi_send_mcs_short_hs(sender, set_display_on, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
		goto power_err;
	}
power_err:
	return err;
}

static int sdc25x16_cmd_power_off(
		struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
			set_display_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display Off\n",
		__func__, __LINE__);
		goto power_off_err;
	}
	msleep(150);

	err = mdfld_dsi_send_mcs_short_hs(sender,
			enter_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Enter Sleep Mode\n",
		__func__, __LINE__);
		goto power_off_err;
	}
	msleep(120);

	return 0;
power_off_err:
	err = -EIO;
	return err;
}

static
int sdc25x16_cmd_set_brightness(
		struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i = 0;

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	i = 20 - (20 * level)/ 255;
	sdc25x16_set_dimming(sender, i);
	return 0;
}

static
int sdc25x16_cmd_panel_reset(
		struct mdfld_dsi_config *dsi_config)
{
	int ret = 0;

	msleep(30);
	if (vdd_1_8v_gpio == 0) {
		vdd_1_8v_gpio = 155;
		ret = gpio_request(vdd_1_8v_gpio, "vdd_1_8v_gpio");
		if (ret) {
			DRM_ERROR("Faild to request vdd_1_8v gpio\n");
			return -EINVAL;
		}
	}
	gpio_direction_output(vdd_1_8v_gpio, 0);
	gpio_set_value_cansleep(vdd_1_8v_gpio, 0);
	usleep_range(2000, 2500);
	gpio_set_value_cansleep(vdd_1_8v_gpio, 1);
	msleep(800);
	return 0;
}

static
int sdc25x16_cmd_exit_deep_standby(
		struct mdfld_dsi_config *dsi_config)
{
	static bool bFirst = true;

	PSB_DEBUG_ENTRY("\n");
	if (bFirst) bFirst = false;
	else {
		msleep(30);
		gpio_direction_output(vdd_1_8v_gpio, 0);
		gpio_set_value_cansleep(vdd_1_8v_gpio, 0);
		usleep_range(2000, 2500);
		gpio_set_value_cansleep(vdd_1_8v_gpio, 1);
		usleep_range(2000, 2500);
	}
	return 0;
}

static
struct drm_display_mode *sdc25x16_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 2560;

	mode->hsync_start = mode->hdisplay + 48;
	mode->hsync_end = mode->hsync_start + 32;
	mode->htotal = mode->hsync_end + 80;

	mode->vdisplay = 1600;
	mode->vsync_start = mode->vdisplay + 3;
	mode->vsync_end = mode->vsync_start + 33;
	mode->vtotal = mode->vsync_end + 10;


	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static
void sdc25x16_cmd_get_panel_info(int pipe,
		struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = 130;
		pi->height_mm = 181;
	}
}

void sdc25x16_cmd_init(struct drm_device *dev,
		struct panel_funcs *p_funcs)
{
	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}
	PSB_DEBUG_ENTRY("\n");
	p_funcs->reset = sdc25x16_cmd_panel_reset;
	p_funcs->power_on = sdc25x16_cmd_power_on;
	p_funcs->power_off = sdc25x16_cmd_power_off;
	p_funcs->drv_ic_init = sdc25x16_cmd_drv_ic_init;
	p_funcs->get_config_mode = sdc25x16_cmd_get_config_mode;
	p_funcs->get_panel_info = sdc25x16_cmd_get_panel_info;
	p_funcs->dsi_controller_init =
			sdc25x16_cmd_controller_init;
	p_funcs->detect =
			sdc25x16_cmd_panel_connection_detect;
	p_funcs->set_brightness =
			sdc25x16_cmd_set_brightness;
	p_funcs->exit_deep_standby =
				sdc25x16_cmd_exit_deep_standby;

}

