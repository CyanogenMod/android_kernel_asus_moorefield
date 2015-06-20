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

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include <linux/gpio.h>
#include <linux/regulator/driver.h>
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))
#include <linux/regulator/intel_pmic.h>
#endif
#include <linux/regulator/machine.h>

struct h8c7_regulator_factory {
	bool h8c7_mmc2_on;
	struct regulator *regulator;
};

static struct h8c7_regulator_factory h8c7_regulator_status;

static u8 h8c7_exit_sleep_mode[]     = {0x11};
static u8 h8c7_set_tear_on[]         = {0x35, 0x00};
static u8 h8c7_set_brightness[]      = {0x51, 0x00};
static u8 h8c7_turn_on_backlight[]   = {0x53, 0x24};
static u8 h8c7_turn_off_backlight[]  = {0x53, 0x00};
static u8 h8c7_disable_cabc[]        = {0x55, 0x00};
static u8 h8c7_set_mipi_ctrl[]       = {0xba, 0x12, 0x83};
static u8 h8c7_command_mode[]        = {0xc2, 0x08};
static u8 h8c7_set_panel[]           = {0xcc, 0x08};
static u8 h8c7_set_eq_func_ltps[]    = {0xd4, 0x0c};
static u8 h8c7_set_address_mode[]    = {0x36, 0x00};
static u8 h8c7_set_te_scanline[]     = {0x44, 0x00, 0x00, 0x00};
static u8 h8c7_set_pixel_format[]    = {0x3a, 0x77};
static u8 h8c7_mcs_protect_off[]     = {0xb9, 0xff, 0x83, 0x92};
static u8 h8c7_mcs_protect_on[]      = {0xb9, 0x00, 0x00, 0x00};
static u8 h8c7_set_blanking_opt_2[]  = {0xc7, 0x00, 0x40};
static u8 h8c7_mcs_clumn_addr[]      = {0x2a, 0x00, 0x00, 0x02, 0xcf};
static u8 h8c7_mcs_page_addr[]       = {0x2b, 0x00, 0x00, 0x04, 0xff};
static u8 h8c7_ic_bias_current[] = {
	0xbf, 0x05, 0xe0, 0x02,
	0x00};
static u8 h8c7_set_power[] = {
	0xb1, 0x7c, 0x00, 0x44,
	0x94, 0x00, 0x0d, 0x0d,
	0x12, 0x1f, 0x3f, 0x3f,
	0x42, 0x72};
static u8 h8c7_set_power_dstb[] = {
	0xb1, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 h8c7_set_disp_reg[] = {
	0xb2, 0x0f, 0xc8, 0x01,
	0x01, 0x06, 0x84, 0x00,
	0xff, 0x01, 0x01, 0x06,
	0x20};
static u8 h8c7_set_command_cyc[] = {
	0xb4, 0x00, 0x00, 0x05,
	0x00, 0xa0, 0x05, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x04, 0x07, 0x07,
	0x01, 0x00, 0x1a, 0x77};
static u8 h8c7_set_ltps_ctrl_output[] = {
	0xd5, 0x00, 0x08, 0x08,
	0x00, 0x44, 0x55, 0x66,
	0x77, 0xcc, 0xcc, 0xcc,
	0xcc, 0x00, 0x77, 0x66,
	0x55, 0x44, 0xcc, 0xcc,
	0xcc, 0xcc};
static u8 h8c7_set_video_cyc[] = {
	0xd8, 0x00, 0x00, 0x05,
	0x00, 0xa0, 0x05, 0x16,
	0x9d, 0x30, 0x03, 0x16,
	0x00, 0x03, 0x03, 0x00,
	0x1b, 0x04, 0x07, 0x07,
	0x01, 0x00, 0x1a, 0x77};
static u8 h8c7_gamma_r[] = {
	0xe0, 0x00, 0x1f, 0x23,
	0x3f, 0x3f, 0x3f, 0x33,
	0x55, 0x06, 0x0e, 0x0e,
	0x11, 0x14, 0x12, 0x14,
	0x1d, 0x1f, 0x00, 0x1f,
	0x23, 0x3f, 0x3f, 0x3f,
	0x33, 0x55, 0x06, 0x0e,
	0x0e, 0x11, 0x14, 0x12,
	0x14, 0x1d, 0x1f};
static u8 h8c7_gamma_g[] = {
	0xe1, 0x00, 0x1f, 0x23,
	0x3f, 0x3f, 0x3f, 0x33,
	0x55, 0x06, 0x0e, 0x0e,
	0x11, 0x14, 0x12, 0x14,
	0x1d, 0x1f, 0x00, 0x1f,
	0x23, 0x3f, 0x3f, 0x3f,
	0x33, 0x55, 0x06, 0x0e,
	0x0e, 0x11, 0x14, 0x12,
	0x14, 0x1d, 0x1f};
static u8 h8c7_gamma_b[] = {
	0xe2, 0x00, 0x1f, 0x23,
	0x3f, 0x3f, 0x3f, 0x33,
	0x55, 0x06, 0x0e, 0x0e,
	0x11, 0x14, 0x12, 0x14,
	0x1d, 0x1f, 0x00, 0x1f,
	0x23, 0x3f, 0x3f, 0x3f,
	0x33, 0x55, 0x06, 0x0e,
	0x0e, 0x11, 0x14, 0x12,
	0x14, 0x1d, 0x1f};
static u8 h8c7_enter_set_cabc[] = {
	0xc9, 0x1f, 0x00, 0x1e,
	0x1e, 0x00, 0x20, 0x00,
	0x01, 0xe3};
static u8 h8c7_set_stba[] = {
	0xc0, 0x01, 0x94};

static u8 h8c7_set_cabc_gain[] = {
	0xca, 0x40, 0x3d, 0x3a,
	0x36, 0x35, 0x34, 0x2d,
	0x25, 0x22};

#define MIPI_RESET_GPIO_DEFAULT 128

static
int mdfld_h8c7_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	struct mdfld_dsi_hw_registers *regs;

	if (!sender)
		return -EINVAL;

	regs = &dsi_config->regs;

	PSB_DEBUG_ENTRY("\n");
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	/* swtich to 2 data lane */
	REG_WRITE(regs->device_ready_reg, 0x0);
	udelay(1);
	REG_WRITE(regs->dsi_func_prg_reg, 0xA002);
	udelay(1);
	REG_WRITE(regs->device_ready_reg, 0x1);
	udelay(1);

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_exit_sleep_mode[0], 0, 0, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdelay(150);

	/* set password*/
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_ic_bias_current, 5, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_power, 14, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_disp_reg, 13, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_command_cyc, 24, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_mipi_ctrl, sizeof(h8c7_set_mipi_ctrl), 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* switch back to 3 data lane */
	mdfld_dsi_wait_for_fifos_empty(sender);
	REG_WRITE(regs->device_ready_reg, 0x0);
	udelay(1);
	REG_WRITE(regs->dsi_func_prg_reg, 0xA003);
	udelay(1);
	REG_WRITE(regs->device_ready_reg, 0x1);

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_stba, 3, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_command_mode[0], h8c7_command_mode[1], 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_blanking_opt_2,
				   sizeof(h8c7_set_blanking_opt_2), 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_set_panel[0], h8c7_set_panel[1], 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_set_eq_func_ltps[0], h8c7_set_eq_func_ltps[1], 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_ltps_ctrl_output, 22, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_video_cyc, 24, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

#if 0
	mdfld_dsi_send_mcs_long_lp(sender, h8c7_gamma_r, 35, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_gamma_g, 35, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_lp(sender, h8c7_gamma_b, 35, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
#endif

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_set_pixel_format[0], h8c7_set_pixel_format[1], 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_clumn_addr, 5, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_page_addr, 5, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_set_address_mode[0], h8c7_set_address_mode[1], 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_te_scanline, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_set_tear_on[0], h8c7_set_tear_on[1], 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_enter_set_cabc, 10, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)

		return -EIO;
	/* set backlight on*/
	mdfld_dsi_send_mcs_short_hs(sender, h8c7_turn_on_backlight[0], h8c7_turn_on_backlight[1], 1 , 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	/* disalble CABC*/
	mdfld_dsi_send_mcs_short_hs(sender, h8c7_disable_cabc[0], h8c7_disable_cabc[1], 1, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;

	mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_protect_on, 4, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
		return -EIO;
	mdelay(5);
	return 0;
}

static
void mdfld_h8c7_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;

	struct csc_setting csc = {	.pipe = 0,
								.type = CSC_REG_SETTING,
								.enable_state = true,
								.data_len = CSC_REG_COUNT,
								.data.csc_reg_data = {
									0xFFB0424, 0xFDF, 0x4320FF1, 0xFDC, 0xFF50FF5, 0x415}
							 };
	struct gamma_setting gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x030303, 0x050505, 0x070707,
										0x090909, 0x0C0C0C, 0x0E0E0E, 0x101010,
										0x121212, 0x141414, 0x171717, 0x191919,
										0x1B1B1B, 0x1D1D1D, 0x1F1F1F, 0x212121,
										0x232323, 0x252525, 0x282828, 0x2A2A2A,
										0x2C2C2C, 0x2E2E2E, 0x303030, 0x323232,
										0x343434, 0x363636, 0x383838, 0x3A3A3A,
										0x3C3C3C, 0x3E3E3E, 0x404040, 0x424242,
										0x444444, 0x464646, 0x484848, 0x4A4A4A,
										0x4C4C4C, 0x4E4E4E, 0x505050, 0x525252,
										0x545454, 0x565656, 0x585858, 0x5A5A5A,
										0x5C5C5C, 0x5E5E5E, 0x606060, 0x626262,
										0x646464, 0x666666, 0x686868, 0x6A6A6A,
										0x6C6C6C, 0x6E6E6E, 0x707070, 0x727272,
										0x747474, 0x767676, 0x787878, 0x7A7A7A,
										0x7C7C7C, 0x7E7E7E, 0x808080, 0x828282,
										0x848484, 0x868686, 0x888888, 0x8A8A8A,
										0x8C8C8C, 0x8E8E8E, 0x909090, 0x929292,
										0x949494, 0x969696, 0x989898, 0x999999,
										0x9B9B9B, 0x9D9D9D, 0x9F9F9F, 0xA1A1A1,
										0xA3A3A3, 0xA5A5A5, 0xA7A7A7, 0xA9A9A9,
										0xABABAB, 0xADADAD, 0xAFAFAF, 0xB1B1B1,
										0xB3B3B3, 0xB5B5B5, 0xB6B6B6, 0xB8B8B8,
										0xBABABA, 0xBCBCBC, 0xBEBEBE, 0xC0C0C0,
										0xC2C2C2, 0xC4C4C4, 0xC6C6C6, 0xC8C8C8,
										0xCACACA, 0xCCCCCC, 0xCECECE, 0xCFCFCF,
										0xD1D1D1, 0xD3D3D3, 0xD5D5D5, 0xD7D7D7,
										0xD9D9D9, 0xDBDBDB, 0xDDDDDD, 0xDFDFDF,
										0xE1E1E1, 0xE3E3E3, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF7F7F7, 0xF9F9F9, 0xFBFBFB, 0xFDFDFD}
								 };


	PSB_DEBUG_ENTRY("\n");

	/*reconfig lane configuration*/
	dsi_config->lane_count = 3;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_3_1;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;
	/* This is for 400 mhz.  Set it to 0 for 800mhz */
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x00;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x1f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x20;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0x20000E;
	hw_ctx->hs_ls_dbi_enable = 0x0;
	/* HW team suggested 1390 for bandwidth setting */
	hw_ctx->dbi_bw_ctrl = 1390;
	hw_ctx->dphy_param = 0x20124E1A;
	hw_ctx->dsi_func_prg = (0xa000 | dsi_config->lane_count);
	hw_ctx->mipi = TE_TRIGGER_GPIO_PIN;
	hw_ctx->mipi |= dsi_config->lane_config;

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
struct drm_display_mode *h8c7_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->htotal = 920;
	mode->hdisplay = 720;
	mode->hsync_start = 816;
	mode->hsync_end = 824;
	mode->vtotal = 1300;
	mode->vdisplay = 1280;
	mode->vsync_start = 1294;
	mode->vsync_end = 1296;
	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
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

static
int mdfld_dsi_h8c7_cmd_power_on(struct mdfld_dsi_config *dsi_config)
{

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	int enable_err, enabled = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (!IS_ERR(h8c7_regulator_status.regulator)) {

		if (!h8c7_regulator_status.h8c7_mmc2_on) {
			PSB_DEBUG_ENTRY("Before power on, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
			PSB_DEBUG_ENTRY("Begin to power on\n");
			h8c7_regulator_status.h8c7_mmc2_on = true;
		} else {
			DRM_ERROR("power on several times without off\n");
		}

		enabled = regulator_is_enabled(h8c7_regulator_status.regulator);
		enable_err = regulator_enable(h8c7_regulator_status.regulator);
		if (enable_err < 0) {
			regulator_put(h8c7_regulator_status.regulator);
			DRM_ERROR("FATAL:enable h8c7 regulator error\n");
		}

		/* vemmc2 need 50ms delay due to stability
		** If already enabled, no need to wait for this delay.
		** This code isn't race proof but since in addition to
		** this panel driver only touch driver is enabling this
		** regulator and does it after this function has been
		** finished, this code works well enough for now.
		*/
		if (!enabled)
			msleep(50);
		PSB_DEBUG_ENTRY("After power on, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
	}

	/*set tear on*/
	err = mdfld_dsi_send_dcs(sender,
		 set_tear_on,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("faild to set_tear_on mode\n");
		goto power_err;
	}

	/*turn on display*/
	err = mdfld_dsi_send_dcs(sender,
		 set_display_on,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("faild to set_display_on mode\n");
		goto power_err;
	}
	if (drm_psb_enable_cabc) {
		/* turn on cabc */
		h8c7_disable_cabc[1] = 0x2;
		mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc,
					   sizeof(h8c7_disable_cabc), 0);
		mdelay(5);
		mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
		mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_cabc_gain, 10, 0);
		mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_on, 4, 0);
		DRM_INFO("%s enable h8c7 cabc\n", __func__);
	}
power_err:
	return err;
}

static int mdfld_dsi_h8c7_cmd_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	/* turn off cabc */
	h8c7_disable_cabc[1] = 0x0;
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc,
				   sizeof(h8c7_disable_cabc), 0);

	/*turn off backlight*/
	err = mdfld_dsi_send_mcs_long_hs(sender, h8c7_turn_off_backlight,
					 sizeof(h8c7_turn_off_backlight), 0);
	if (err) {
		DRM_ERROR("%s: failed to turn off backlight\n", __func__);
		goto out;
	}
	mdelay(1);


	/*turn off display */
	err = mdfld_dsi_send_dcs(sender,
		 set_display_off,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("sent set_display_off faild\n");
		goto out;
	}

	/*set tear off */
	err = mdfld_dsi_send_dcs(sender,
		 set_tear_off,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("sent set_tear_off faild\n");
		goto out;
	}

	/*Enter sleep mode */
	err = mdfld_dsi_send_dcs(sender,
			enter_sleep_mode,
			NULL,
			0,
			CMD_DATA_SRC_SYSTEM_MEM,
			MDFLD_DSI_SEND_PACKAGE);

	if (err) {
		DRM_ERROR("DCS 0x%x sent failed\n", enter_sleep_mode);
		goto out;
	}

	/**
	 * MIPI spec shows it must wait 5ms
	 * before sneding next command
	 */
	mdelay(5);

	/*enter deep standby mode*/
	err = mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
	if (err) {
		DRM_ERROR("Failed to turn off protection\n");
		goto out;
	}

	err = mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_power_dstb, 14, 0);
	if (err)
		DRM_ERROR("Failed to enter DSTB\n");
	mdelay(5);
	mdfld_dsi_send_mcs_long_hs(sender, h8c7_mcs_protect_on, 4, 0);

out:
	if (!IS_ERR(h8c7_regulator_status.regulator)) {
		if (h8c7_regulator_status.h8c7_mmc2_on) {
			h8c7_regulator_status.h8c7_mmc2_on = false;
			PSB_DEBUG_GENERAL("Begin to power off\n");
		} else
			DRM_ERROR("power off several times without on\n");
		regulator_disable(h8c7_regulator_status.regulator);
		PSB_DEBUG_GENERAL("After power off, regulator is %d\n",
			regulator_is_enabled(h8c7_regulator_status.regulator));
	}

	return err;
}

static
void h8c7_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");

	if (pipe == 0) {
		pi->width_mm = PANEL_4DOT3_WIDTH;
		pi->height_mm = PANEL_4DOT3_HEIGHT;
	}
}

static
int mdfld_dsi_h8c7_cmd_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

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
			mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
			mdfld_dsi_send_gen_long_hs(sender, h8c7_set_disp_reg, 13, 0);
			mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_on, 4, 0);

		} else {
			dsi_config->dsi_hw_context.panel_on = false;
			DRM_INFO("%s: panel is not initialized!\n", __func__);
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
int mdfld_dsi_h8c7_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
		int level)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int duty_val = 0;
	static bool b_cabc_initialized = false;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	if (drm_psb_enable_cabc && !b_cabc_initialized) {
		h8c7_disable_cabc[1] = 0x2;
		mdfld_dsi_send_mcs_long_hs(sender, h8c7_disable_cabc,
					   sizeof(h8c7_disable_cabc), 0);
		mdelay(5);
		mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_off, 4, 0);
		mdfld_dsi_send_mcs_long_hs(sender, h8c7_set_cabc_gain, 10, 0);
		mdfld_dsi_send_gen_long_hs(sender, h8c7_mcs_protect_on, 4, 0);
		b_cabc_initialized = true;
	}

	duty_val = (255 * level) / 100;
	h8c7_set_brightness[1] = duty_val;

	mdfld_dsi_send_mcs_short_hs(sender, h8c7_set_brightness[0], h8c7_set_brightness[1], 1, 0);

	return 0;
}

static
int mdfld_dsi_h8c7_cmd_panel_reset(struct mdfld_dsi_config *dsi_config)
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
	mdelay(11);

	gpio_set_value_cansleep(mipi_reset_gpio, 1);
	mdelay(5);

	return 0;
}

void h8c7_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ena_err;

	if (!dev || !p_funcs) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = h8c7_cmd_get_config_mode;
	p_funcs->get_panel_info = h8c7_cmd_get_panel_info;
	p_funcs->reset = mdfld_dsi_h8c7_cmd_panel_reset;
	p_funcs->drv_ic_init = mdfld_h8c7_drv_ic_init;
	p_funcs->dsi_controller_init = mdfld_h8c7_dsi_controller_init;
	p_funcs->detect = mdfld_dsi_h8c7_cmd_detect;
	p_funcs->power_on = mdfld_dsi_h8c7_cmd_power_on;
	p_funcs->power_off = mdfld_dsi_h8c7_cmd_power_off;
	p_funcs->set_brightness = mdfld_dsi_h8c7_cmd_set_brightness;

	/* Please check the file pmic_avp.c for the correct regulator name */
	h8c7_regulator_status.regulator = regulator_get(NULL, "vemmc2");
	if (IS_ERR(h8c7_regulator_status.regulator)) {
		DRM_ERROR("H8C7 device failed to get mmc regulator\n");
		return ;
	}

	h8c7_regulator_status.h8c7_mmc2_on =
		regulator_is_enabled(h8c7_regulator_status.regulator);

	if (h8c7_regulator_status.h8c7_mmc2_on) {
		ena_err = regulator_enable(h8c7_regulator_status.regulator);
		if (ena_err < 0) {
			regulator_put(h8c7_regulator_status.regulator);
			DRM_ERROR("FATAL:enable h8c7 regulator error\n");
		}
	}
	/*Temperory Work Around to keep VEMMC2 rail ON as current display
	panel doesnt support this */
	ena_err = regulator_enable(h8c7_regulator_status.regulator);
	if (ena_err < 0) {
		regulator_put(h8c7_regulator_status.regulator);
		DRM_ERROR("FATAL:enable h8c7 regulator error in WA\n");
	}
}

static int h8c7_lcd_cmd_probe(struct platform_device *pdev)
{
	DRM_INFO("%s: H8C7 panel detected\n", __func__);
	intel_mid_panel_register(h8c7_cmd_init);

	return 0;
}

struct platform_driver h8c7_lcd_driver = {
	.probe	= h8c7_lcd_cmd_probe,
	.driver	= {
		.name	= "H8C7 CMD RHB",
		.owner	= THIS_MODULE,
	},
};
