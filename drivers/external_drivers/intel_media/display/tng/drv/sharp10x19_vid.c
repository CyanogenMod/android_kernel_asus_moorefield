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
 * Faxing Lu <faxing.lu@intel.com>
 *
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_esd.h"
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_rpmsg.h>
#include <asm/intel_mid_remoteproc.h>
#include "displays/sharp10x19_vid.h"

static int mipi_reset_gpio;
static int bias_en_gpio;
static u8 sharp10x19_set_mode[] = {0xb3, 0x14};

static int mdfld_dsi_sharp10x19_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	err = mdfld_dsi_send_mcs_short_hs(sender,
		access_protect,
		0x4, 1 , 0);
	if (err) {
		DRM_ERROR("%s: %d: Set access_protect off\n", __func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_short_hs(sender,
		0xd6,
		0x01, 1 , 0);
	if (err) {
		DRM_ERROR("%s: %d: Remove NVM reload\n", __func__, __LINE__);
		goto ic_init_err;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
		write_ctrl_display,
		0xc, 1 , 0);
	if (err) {
		DRM_ERROR("%s: %d: Set backlight\n", __func__, __LINE__);
		goto ic_init_err;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender,
		write_ctrl_cabc, dsi_config->cabc_mode, 1,
		MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: write_ctrl_cabc\n", __func__, __LINE__);
		goto ic_init_err;
	}
	err = mdfld_dsi_send_mcs_short_hs(sender,
						  set_pixel_format, 0x77, 1,
						  MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: set_pixel_format\n", __func__, __LINE__);
		goto ic_init_err;
	}

	return 0;
ic_init_err:
	err = -EIO;
	return err;

}

static void
mdfld_sharp10x19_dpi_controller_init(
	struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx =
		&dsi_config->dsi_hw_context;
	int mipi_pixel_format = 0x4;

	PSB_DEBUG_ENTRY("\n");

	/* Override global default to set special initial cabc mode for
	 * this display type. */
	dsi_config->cabc_mode = CABC_MODE_OFF;

	/*reconfig lane configuration*/
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->turn_around_timeout = 0x14;
	hw_ctx->high_low_switch_count = 0x2b;
	hw_ctx->clk_lane_switch_time_cnt = 0x2b0014;
	hw_ctx->init_count = 0;
	hw_ctx->eot_disable = 0;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->dphy_param = 0x2a18681f;

	/*setup video mode format*/
	hw_ctx->video_mode_format = 0xf;
	hw_ctx->dsi_func_prg = ((mipi_pixel_format << 7) | dsi_config->lane_count);

	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		BANDGAP_CHICKEN_BIT | dsi_config->lane_config;
}

int mdfld_dsi_sharp10x19_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	int pipe = dsi_config->pipe;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	u32 power_island = 0;

	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		power_island = pipe_to_island(pipe);

		if (!power_island_get(power_island)) {
			DRM_ERROR("Failed to turn on power island\n");
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
		power_island_put(power_island);
	} else {
		DRM_INFO("%s: do NOT support dual panel\n",
		__func__);
		status = MDFLD_DSI_PANEL_DISCONNECTED;
	}

	return status;
}

static int mdfld_dsi_sharp10x19_power_on(
		struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("Turn on video mode TMD panel...\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	err = mdfld_dsi_send_mcs_short_hs(sender,
						  exit_sleep_mode, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: exit_sleep_mode\n", __func__, __LINE__);
		goto power_on_err;
	}
	msleep(120);

	/* Set Display on 0x29 */
	err = mdfld_dsi_send_mcs_short_hs(sender, set_display_on, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display On\n", __func__, __LINE__);
		goto power_on_err;
	}

	/*send TURN_ON packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	err = mdfld_dsi_send_gen_short_hs(sender,access_protect, 0x4, 2,
                        MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set MCAP\n",__func__, __LINE__);
		goto power_on_err;
	}

	err = mdfld_dsi_send_gen_long_hs(sender, sharp10x19_set_mode, 2,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Mode\n", __func__, __LINE__);
		goto power_on_err;
	}

	msleep(20);

	err = mdfld_dsi_send_mcs_short_hs(sender,
			write_display_brightness, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Brightness\n", __func__, __LINE__);
		goto power_on_err;
	}

	return 0;
power_on_err:
	err = -EIO;
	return err;
}
static int mdfld_dsi_sharp10x19_power_off(
			struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	PSB_DEBUG_ENTRY("Turn off video mode TMD panel...\n");
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	/*send SHUT_DOWN packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		goto power_off_err;
	}
	msleep(100);

	/* Set Display off */
	err = mdfld_dsi_send_mcs_short_hs(sender, set_display_off, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Display Off\n", __func__, __LINE__);
		goto power_off_err;
	}
	/* Wait for 1 frame after set_display_on. */
	msleep(20);

	/* Sleep In */
	err = mdfld_dsi_send_mcs_short_hs(sender, enter_sleep_mode, 0, 0,
			MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Enter Sleep Mode\n", __func__, __LINE__);
		goto power_off_err;
	}
	msleep(60);
	err = mdfld_dsi_send_gen_short_hs(sender,
		access_protect, 4, 2,
		MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Access Protect\n", __func__, __LINE__);
		goto power_off_err;
	}
	err = mdfld_dsi_send_gen_short_hs(sender, low_power_mode, 1, 2,
					  MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("%s: %d: Set Low Power Mode\n", __func__, __LINE__);
		goto power_off_err;
	}
	if (bias_en_gpio)
		gpio_set_value_cansleep(bias_en_gpio, 0);
	if (mipi_reset_gpio)
		gpio_set_value_cansleep(mipi_reset_gpio, 0);
	usleep_range(1000, 1500);
	return 0;

power_off_err:
	return -EIO;
}
static
struct drm_display_mode *sharp10x19_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->hdisplay = 1080;

	mode->hsync_start = mode->hdisplay + 106;
	mode->hsync_end = mode->hsync_start + 10;
	mode->htotal = mode->hsync_end + 50;

	mode->vdisplay = 1920;
	mode->vsync_start = mode->vdisplay + 4;
	mode->vsync_end = mode->vsync_start + 4;
	mode->vtotal = mode->vsync_end + 4;

	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal *
		mode->htotal / 1000;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	mode->type |= DRM_MODE_TYPE_PREFERRED;

	return mode;
}

static
void sharp10x19_get_panel_info(int pipe,
				struct panel_info *pi)
{
	pi->width_mm = 58;
	pi->height_mm = 103;
}

int mdfld_dsi_sharp10x19_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	PSB_DEBUG_ENTRY("level = %d\n", level);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	mdfld_dsi_send_mcs_short_hs(sender,
		write_display_brightness, level, 1,
		MDFLD_DSI_SEND_PACKAGE);

	/* give some time for new backlight value to take effect */
	msleep(20);

	return 0;
}

static void _get_panel_reset_gpio(void)
{
	int ret = 0;
	if (mipi_reset_gpio == 0) {
		ret = get_gpio_by_name("disp0_rst");
		if (ret < 0) {
			DRM_ERROR("Faild to get panel reset gpio, " \
				  "use default reset pin\n");
			return;
		}
		mipi_reset_gpio = ret;
		ret = gpio_request(mipi_reset_gpio, "mipi_display");
		if (ret) {
			DRM_ERROR("Faild to request panel reset gpio\n");
			return;
		}
		gpio_direction_output(mipi_reset_gpio, 0);
		pr_info("gpio_reseta=%d\n", mipi_reset_gpio);
	}

}
static void __vpro2_power_ctrl(bool on)
{
	u8 addr, value;
	addr = 0xad;
	if (intel_scu_ipc_ioread8(addr, &value))
		DRM_ERROR("%s: %d: failed to read vPro2\n", __func__, __LINE__);

	/* Control vPROG2 power rail with 2.85v. */
	if (on)
		value |= 0x1;
	else
		value &= ~0x1;

	if (intel_scu_ipc_iowrite8(addr, value))
		DRM_ERROR("%s: %d: failed to write vPro2\n",
				__func__, __LINE__);
}

static int mdfld_dsi_sharp10x19_panel_reset(struct mdfld_dsi_config *dsi_config)
{
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	__vpro2_power_ctrl(true);
	usleep_range(2000, 2500);
	if (bias_en_gpio == 0) {
		bias_en_gpio = 189;
		ret = gpio_request(bias_en_gpio, "bias_enable");
		if (ret) {
			DRM_ERROR("Faild to request bias_enable gpio\n");
			return -EINVAL;
		}
		gpio_direction_output(bias_en_gpio, 0);
		pr_info("gpio_bias_enable=%d\n", bias_en_gpio);
	}
	_get_panel_reset_gpio();

	gpio_direction_output(bias_en_gpio, 0);
	gpio_direction_output(mipi_reset_gpio, 0);
	gpio_set_value_cansleep(bias_en_gpio, 0);
	gpio_set_value_cansleep(mipi_reset_gpio, 0);

	usleep_range(2000, 2500);
	gpio_set_value_cansleep(bias_en_gpio, 1);
	usleep_range(2000, 2500);
	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	usleep_range(2000, 2500);

	return 0;
}


void sharp10x19_vid_init(struct drm_device *dev,
		struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");
	p_funcs->get_config_mode = sharp10x19_get_config_mode;
	p_funcs->get_panel_info = sharp10x19_get_panel_info;
	p_funcs->reset = mdfld_dsi_sharp10x19_panel_reset;
	p_funcs->dsi_controller_init =
			mdfld_sharp10x19_dpi_controller_init;
	p_funcs->detect = mdfld_dsi_sharp10x19_detect;
	p_funcs->power_on = mdfld_dsi_sharp10x19_power_on;
	p_funcs->power_off = mdfld_dsi_sharp10x19_power_off;
	p_funcs->drv_ic_init = mdfld_dsi_sharp10x19_drv_ic_init;
	p_funcs->set_brightness =
		mdfld_dsi_sharp10x19_set_brightness;
	p_funcs->drv_set_cabc_mode = display_cmn_set_cabc_mode;
	p_funcs->drv_get_cabc_mode = display_cmn_get_cabc_mode;
}
