/*
 * Copyright (c)  2014 Intel Corporation
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
 */

#include <video/mipi_display.h>
#include <linux/lnw_gpio.h>
#include <linux/gpio.h>
#include <linux/intel_mid_pm.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel-mid.h>

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "displays/td4300_vid.h"
#include <linux/HWVersion.h>


#define TD4300_DEBUG 1

/*
 * GPIO pin definition
 */
//#define TD4300_BL_EN_GPIO   188
#define TD4300_BL_PWM_GPIO  183

#define PWMCTRL_REG 0xFF013C00
#define PWMCTRL_SIZE 0x80
#define PWM_BASE_UNIT 0x1555 //25,000Hz


union pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static void __iomem *pwmctrl_mmio;
static int panel_reset_gpio;
static int panel_en_gpio;
static int backlight_en_gpio;
static int backlight_pwm_gpio;
static int stb1_en_gpio;
static int stb2_en_gpio;

struct mipi_dsi_cmd{
	int delay;
	int len;
	u8 *commands;
};

static struct mdfld_dsi_config *td4300_dsi_config;

static struct mipi_dsi_cmd *td4300_power_on_table = NULL;
static int td4300_power_on_table_size = 0;


/* ====Initial settings==== */
static u8 cm1_01[] = {0xB0, 0x04};
static u8 cm1_02[] = {0xB3, 0x31, 0x00, 0x06};
static u8 cm1_03[] = {0xB4, 0x00};
static u8 cm1_04[] = {0xB6, 0x33, 0xD3, 0x01, 0xFF, 0xFF};
static u8 cm1_05[] = {0xB8, 0x57, 0x3D, 0x19, 0x1E, 0x0A, 0x50, 0x50};
static u8 cm1_06[] = {0xB9, 0x6F, 0x3D, 0x28, 0x3C, 0x14, 0xC8, 0xC8};
static u8 cm1_07[] = {0xBA, 0xB5, 0x33, 0x41, 0x64, 0x23, 0xA0, 0xA0};
static u8 cm1_08[] = {0xBB, 0x14, 0x14};
static u8 cm1_09[] = {0xBC, 0x37, 0x32};
static u8 cm1_10[] = {0xBD, 0x64, 0x32};
static u8 cm1_11[] = {0xBE, 0x04};
static u8 cm1_12[] = {0xC0, 0x00};
static u8 cm1_13[] = {0xC1, 0x84, 0x08, 0x00, 0xFF, 0xFF, 0x2F, 0x7F, 0xFC, 0x37, 0x9B,
	0xF3, 0xFE, 0x67, 0xCD, 0xB9, 0xC6, 0xFE, 0x07, 0xC7, 0xE4, 0xFB, 0xC5, 0x1F, 0xFF,
	0xFF, 0x0F, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x40, 0x00, 0x62, 0x03,
	0x02, 0x03, 0x82, 0x00, 0x01, 0x00, 0x01};
static u8 cm1_14[] = {0xC2, 0x01, 0xF7, 0x80, 0x04, 0x64, 0x08, 0x0C, 0x10, 0x00, 0x08,
	0x60, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm1_15[] = {0xC3, 0x78, 0x77, 0x87, 0x78, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x3C, 0x33, 0xC3, 0x3C, 0x30, 0x3C, 0x43, 0xC4, 0x3C, 0x40, 0x01, 0x01, 0x03, 0x28,
	0x00, 0x01, 0x03, 0x01, 0x10, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00,
	0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x13, 0x00, 0x26, 0x00, 0x26, 0x00,
	0x26, 0x00, 0x26, 0x00, 0x26, 0x00, 0x26, 0x00, 0x40, 0x20};
static u8 cm1_16[] = {0xC4, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x02,
	0x02, 0x31, 0x01, 0x00, 0x00, 0x00};
static u8 cm1_17[] = {0xC5, 0x10, 0x00, 0x00, 0x00, 0x00, 0x70, 0x00};
static u8 cm1_18[] = {0xC6, 0x5F, 0x05, 0x55, 0x02, 0x55, 0x01, 0x0E, 0x01, 0x02, 0x01,
	0x02, 0x01, 0x02, 0x01, 0x02, 0x01, 0x02, 0x05, 0x15, 0x07, 0x7F, 0xFF};
static u8 cm1_19[] = {0xC7, 0x08, 0x14, 0x1b, 0x24, 0x31, 0x3D, 0x45, 0x52, 0x34, 0x3C,
	0x48, 0x56, 0x61, 0x6B, 0x7a, 0x08, 0x14, 0x1b, 0x24, 0x31, 0x3D, 0x45, 0x52, 0x34,
	0x3C, 0x48, 0x56, 0x61, 0x6B, 0x7A};
static u8 cm1_20[] = {0xCA, 0x1C, 0xFC, 0xFC, 0xFC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm1_21[] = {0xCB, 0x50, 0x9F, 0x9F, 0xAF, 0x00, 0x00, 0x02, 0x00, 0x04, 0x00,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm1_22[] = {0xCC, 0x0B};
static u8 cm1_23[] = {0xCD, 0x09, 0x00, 0x22, 0x00, 0x22, 0x00, 0x22, 0x00, 0xA3, 0xA3,
	0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0xA0, 0x15, 0x00, 0x00};
static u8 cm1_24[] = {0xCE, 0x5D, 0x40, 0x49, 0x53, 0x59, 0x5E, 0x63, 0x68, 0x6E, 0x74,
	0x7E, 0x8A, 0x98, 0xA8, 0xBB, 0xD0, 0xFF, 0x04, 0x00, 0x04, 0x04, 0x42, 0x00, 0x69,
	0x5A};
static u8 cm1_25[] = {0xCF, 0x48, 0x10};
static u8 cm1_26[] = {0xD0, 0x33, 0x54, 0xD4, 0x31, 0x01, 0x10, 0x10, 0x10, 0x19, 0x19,
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm1_27[] = {0xD1, 0x00};
static u8 cm1_28[] = {0xD3, 0x1B, 0x3B, 0xBB, 0x77, 0x77, 0x77, 0xBB, 0xB3, 0x33, 0x00,
	0x00, 0x60, 0x66, 0xC1, 0xC1, 0x33, 0xBB, 0xF2, 0xFD, 0xC6};
static u8 cm1_29[] = {0xD5, 0x03, 0x00, 0x00, 0x00, 0x45, 0x00, 0x45};
static u8 cm1_30[] = {0xD6, 0xC1};
static u8 cm1_31[] = {0xD7, 0xF6, 0xFF, 0x03, 0x05, 0x41, 0x24, 0x80, 0x1F, 0xC7, 0x1F,
	0x1B, 0x00, 0x0C, 0x07, 0x20, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C, 0xF0, 0x1F, 0x00,
	0x0C, 0x00, 0x00, 0xAA, 0x67, 0x7E, 0x5D, 0x06, 0x00};
static u8 cm1_32[] = {0xD9, 0x00, 0x00};
static u8 cm1_33[] = {0xDE, 0x00, 0x3F, 0xFF, 0x90};
static u8 cm1_34[] = {0xF1, 0x00, 0x00};



/* ====Power on commnad==== */
static struct mipi_dsi_cmd zs550ml_power_on_table[] = {
	{0, sizeof(cm1_01), cm1_01},
	{0, sizeof(cm1_02), cm1_02},
	{0, sizeof(cm1_03), cm1_03},
	{0, sizeof(cm1_04), cm1_04},
	{0, sizeof(cm1_05), cm1_05},
	{0, sizeof(cm1_06), cm1_06},
	{0, sizeof(cm1_07), cm1_07},
	{0, sizeof(cm1_08), cm1_08},
	{0, sizeof(cm1_09), cm1_09},
	{0, sizeof(cm1_10), cm1_10},
	{0, sizeof(cm1_11), cm1_11},
	{0, sizeof(cm1_12), cm1_12},
	{0, sizeof(cm1_13), cm1_13},
	{0, sizeof(cm1_14), cm1_14},
	{0, sizeof(cm1_15), cm1_15},
	{0, sizeof(cm1_16), cm1_16},
	{0, sizeof(cm1_17), cm1_17},
	{0, sizeof(cm1_18), cm1_18},
	{0, sizeof(cm1_19), cm1_19},
	{0, sizeof(cm1_20), cm1_20},
	{0, sizeof(cm1_21), cm1_21},
	{0, sizeof(cm1_22), cm1_22},
	{0, sizeof(cm1_23), cm1_23},
	{0, sizeof(cm1_24), cm1_24},
	{0, sizeof(cm1_25), cm1_25},
	{0, sizeof(cm1_26), cm1_26},
	{0, sizeof(cm1_27), cm1_27},
	{0, sizeof(cm1_28), cm1_28},
	{0, sizeof(cm1_29), cm1_29},
	{0, sizeof(cm1_30), cm1_30},
	{0, sizeof(cm1_31), cm1_31},
	{0, sizeof(cm1_32), cm1_32},
	{0, sizeof(cm1_33), cm1_33},
	{0, sizeof(cm1_34), cm1_34},
};

static int send_mipi_cmd_gen(struct mdfld_dsi_pkg_sender * sender,
				struct mipi_dsi_cmd *cmd) {
	int err = 0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	switch(cmd->len) {
		case 1:
			err = mdfld_dsi_send_gen_short_lp(sender,
				cmd->commands[0],
				0,
				1,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		case 2:
			err = mdfld_dsi_send_gen_short_lp(sender,
				cmd->commands[0],
				cmd->commands[1],
				2,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		default:
			err = mdfld_dsi_send_gen_long_lp(sender,
				cmd->commands,
				cmd->len,
				MDFLD_DSI_SEND_PACKAGE);
			break;
	}

	if (err != 0 || sender->status) {
		printk("[DISP] %s : sent failed with status=%d\n", __func__, sender->status);
		return -EIO;
	}

	if (cmd->delay)
		mdelay(cmd->delay);

	return 0;

}

static int send_mipi_cmd_mcs(struct mdfld_dsi_pkg_sender * sender,
				struct mipi_dsi_cmd *cmd) {
	int err = 0;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	switch(cmd->len) {
		case 1:
			err = mdfld_dsi_send_mcs_short_lp(sender,
				cmd->commands[0],
				0,
				0,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		case 2:
			err = mdfld_dsi_send_mcs_short_lp(sender,
				cmd->commands[0],
				cmd->commands[1],
				1,
				MDFLD_DSI_SEND_PACKAGE);
			break;
		default:
			err = mdfld_dsi_send_mcs_long_lp(sender,
				cmd->commands,
				cmd->len,
				MDFLD_DSI_SEND_PACKAGE);
			break;
	}

	if (err != 0 || sender->status) {
		printk("[DISP] %s : sent failed with status=%d\n", __func__, sender->status);
		return -EIO;
	}

	if (cmd->delay)
		mdelay(cmd->delay);

	return 0;

}


struct delayed_work td4300_panel_reset_delay_work;
struct workqueue_struct *td4300_panel_reset_delay_wq;
static int td4300_vid_drv_ic_reset_workaround(struct mdfld_dsi_config *dsi_config) {

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	mdfld_dsi_send_mcs_short_lp(sender, 0xFF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(10);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}

	return 0;
}

static void td4300_vid_panel_reset_delay_work(struct work_struct *work)
{
//	printk("[DEBUG] %s\n", __func__);
	td4300_vid_drv_ic_reset_workaround(panel_reset_dsi_config);
	queue_delayed_work(td4300_panel_reset_delay_wq, &td4300_panel_reset_delay_work, msecs_to_jiffies(5000));
}


static int td4300_vid_drv_ic_init(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	printk("[DISP] %s START\n", __func__);

	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(5000, 5050);
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(5000, 5050);
	gpio_set_value_cansleep(panel_reset_gpio, 1);

	usleep_range(20000, 20100);

	/* panel initial settings */
	for (i = 0; i < td4300_power_on_table_size; i++)
		send_mipi_cmd_mcs(sender, &td4300_power_on_table[i]);

	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(120);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	printk("[DISP] %s END\n", __func__);
	return 0;
}

static void
td4300_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx;
	if (!dsi_config || !(&dsi_config->dsi_hw_context)) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	printk("[DISP] %s\n", __func__);
	/* Reconfig lane configuration */
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	hw_ctx = &dsi_config->dsi_hw_context;
	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xFFFF;
	hw_ctx->turn_around_timeout = 0xFFFF;
	
	hw_ctx->high_low_switch_count = 0x2f;
	hw_ctx->clk_lane_switch_time_cnt = 0x3d0016;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->dphy_param = 0x351B6D1F;

	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0x7D0;
	hw_ctx->dsi_func_prg = (RGB_888_FMT << FMT_DPI_POS) |
		dsi_config->lane_count;
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		BANDGAP_CHICKEN_BIT;
	hw_ctx->video_mode_format = 0xF;

	td4300_dsi_config = dsi_config;

	/* Panel initial settings assigned */
	printk("[DISP] Synaptics TD4300 Panel initial cmds registered\n");
	td4300_power_on_table = zs550ml_power_on_table;
	td4300_power_on_table_size = ARRAY_SIZE(zs550ml_power_on_table);

}

static int td4300_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	usleep_range(1000, 1100);
//	queue_delayed_work(td4300_panel_reset_delay_wq, &td4300_panel_reset_delay_work, msecs_to_jiffies(5000));

	return 0;
}

static void __vpro3_power_ctrl(bool on)
{
	u8 addr, value;
	addr = 0xae;
	if (intel_scu_ipc_ioread8(addr, &value))
		DRM_ERROR("%s: %d: failed to read vPro3\n", __func__, __LINE__);
	printk("[DEBUG] vpro3 = %x\n", value);

	/* Control vPROG3 power rail with 2.85v. */
	if (on)
		value |= 0x1;
	else
		value &= ~0x1;

	if (intel_scu_ipc_iowrite8(addr, value))
		DRM_ERROR("%s: %d: failed to write vPro3\n",
				__func__, __LINE__);
}


static int td4300_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
//	cancel_delayed_work_sync(&td4300_panel_reset_delay_work);
	usleep_range(1000, 1500);
	/* Send power off command*/
	err = mdfld_dsi_send_mcs_short_hs(sender, MIPI_DCS_SET_DISPLAY_OFF, 0,
					  0, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Display Off\n");
		return err;
	}

	err = mdfld_dsi_send_mcs_short_hs(sender, MIPI_DCS_ENTER_SLEEP_MODE, 0,
					  0, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Enter Sleep Mode\n");
		return err;
	}
	usleep_range(1000, 1500);

	err = mdfld_dsi_send_mcs_short_hs(sender, 0x4F, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Enter Deep Standby Mode\n");
		return err;
	}

	usleep_range(50000, 55000);

	/* Turn off VBAT*/
	gpio_set_value_cansleep(stb1_en_gpio, 0);
	gpio_set_value_cansleep(stb2_en_gpio, 0);

	return 0;
}

static int td4300_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	printk("[DISP] %s\n", __func__);

	/* Open 2V9 power */
	__vpro3_power_ctrl(true);
	usleep_range(1000, 1100);
	/* Open  VBAT*/
	gpio_set_value_cansleep(stb1_en_gpio, 1);
	gpio_set_value_cansleep(stb2_en_gpio, 1);
	usleep_range(15000, 15100);
/* postpone to drv_ic_init
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10100);
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(10000, 10100);
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10100);
*/
	return 0;
}


static int td4300_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	u32 reg_level;
	union pwmctrl_reg pwmctrl;

	/* Re-assign the minimum brightness value to 15 */
	if (level > 0 && level <= 15)
		level = 15;

	reg_level = ~level & 0xFF;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = reg_level;

	if (!pwmctrl_mmio)
		pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG, 4);

	if (pwmctrl_mmio) {
		if (level) {
			pmu_set_pwm(PCI_D0);
			lnw_gpio_set_alt(backlight_pwm_gpio, 1);
//			gpio_set_value_cansleep(backlight_en_gpio, 1);

			pwmctrl.part.pwmenable = 1;
			writel(pwmctrl.full, pwmctrl_mmio);
		} else {
			pwmctrl.part.pwmenable = 0;
			writel(pwmctrl.full, pwmctrl_mmio);
			gpio_set_value_cansleep(backlight_pwm_gpio, 0);
			lnw_gpio_set_alt(backlight_pwm_gpio, 0);
			usleep_range(10000, 10100);
//			gpio_set_value_cansleep(backlight_en_gpio, 0);
			pmu_set_pwm(PCI_D3hot);
		}
	} else {
		DRM_ERROR("Cannot map pwmctrl\n");
	}

	printk("[DISP] brightness level = %d\n", level);

	return 0;
}

struct drm_display_mode *td4300_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	printk("[DISP] %s\n", __func__);

	/* RECOMMENDED PORCH SETTING
		HSA=2, HBP=100, HFP=107
		VSA=1,   VBP=4, VFP=4	 */
	mode->hdisplay = 1080;
	mode->hsync_start = mode->hdisplay + 107;
	mode->hsync_end = mode->hsync_start + 2;
	mode->htotal = mode->hsync_end + 100;

	mode->vdisplay = 1920;
	mode->vsync_start = mode->vdisplay + 4;
	mode->vsync_end = mode->vsync_start + 1;
	mode->vtotal = mode->vsync_end + 4;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void td4300_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	pi->width_mm = 68;
	pi->height_mm = 121;
}

static int td4300_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	printk("[DISP] %s\n", __func__);

	panel_reset_gpio = get_gpio_by_name("DISP_RST_N");
	if (panel_reset_gpio < 0) {
		DRM_ERROR("Faild to get panel reset gpio\n");
		return -EINVAL;
	}

	if (gpio_request(panel_reset_gpio, "panel_reset")) {
		DRM_ERROR("Faild to request panel reset gpio\n");
		return -EINVAL;
	}

	stb1_en_gpio = get_gpio_by_name("STB1_EN");
	if (stb1_en_gpio < 0) {
		DRM_ERROR("Faild to get STB1 enable gpio\n");
		return -EINVAL;
	}
	if (gpio_request(stb1_en_gpio, "STB1_EN")) {
		DRM_ERROR("Faild to request STB1 enable gpio\n");
		return -EINVAL;
	}

	stb2_en_gpio = get_gpio_by_name("STB2_EN");
	if (stb2_en_gpio < 0) {
		DRM_ERROR("Faild to get STB2 enable gpio\n");
		return -EINVAL;
	}
	if (gpio_request(stb2_en_gpio, "STB2_EN")) {
		DRM_ERROR("Faild to request STB2 enable gpio\n");
		return -EINVAL;
	}
/*
	backlight_en_gpio = TD4300_BL_EN_GPIO;
	if (gpio_request(backlight_en_gpio, "backlight_en")) {
		DRM_ERROR("Faild to request backlight enable gpio\n");
		return -EINVAL;
	}
*/
	backlight_pwm_gpio = TD4300_BL_PWM_GPIO;
	if (gpio_request(backlight_pwm_gpio, "backlight_pwm")) {
		DRM_ERROR("Faild to request backlight PWM gpio\n");
		return -EINVAL;
	}

	/* Initializing pwm for being able to adjust backlight when just opening the phone. */
	pmu_set_pwm(PCI_D0);
	lnw_gpio_set_alt(backlight_pwm_gpio, 1);

	dsi_config->dsi_hw_context.panel_on = false;

	return MDFLD_DSI_PANEL_CONNECTED;
}


#ifdef TD4300_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(td4300_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

    send_mipi_ret = mdfld_dsi_send_mcs_short_lp(sender,x0,x1,1,0);

	DRM_INFO("[DISPLAY] send %x,%x : ret = %d\n",x0,x1,send_mipi_ret);

    return count;
}

static ssize_t send_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "%d\n",send_mipi_ret);
}


static ssize_t read_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(td4300_dsi_config);

    sscanf(buf, "%x", &x0);

    read_mipi_ret = mdfld_dsi_read_mcs_lp(sender,x0,&read_mipi_data,1);
    if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL)
        read_mipi_ret = -EIO;

	DRM_INFO("[DISPLAY] read 0x%x :ret=%d data=0x%x\n", x0, read_mipi_ret, read_mipi_data);

    return count;
}

static ssize_t read_mipi_show(struct device *dev,
	struct device_attribute *attr, const char *buf)
{
	return snprintf(buf, PAGE_SIZE, "ret=%d data=0x%x\n",read_mipi_ret,read_mipi_data);
}

DEVICE_ATTR(send_mipi_td4300,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_td4300,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *td4300_attrs[] = {
        &dev_attr_send_mipi_td4300.attr,
        &dev_attr_read_mipi_td4300.attr,
        NULL
};

static struct attribute_group td4300_attr_group = {
        .attrs = td4300_attrs,
        .name = "td4300",
};

#endif

void td4300_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	printk("[DISP] %s\n", __func__);

	p_funcs->get_config_mode = td4300_vid_get_config_mode;
	p_funcs->get_panel_info = td4300_vid_get_panel_info;
	p_funcs->dsi_controller_init = td4300_vid_dsi_controller_init;
	p_funcs->detect = td4300_vid_detect;
	p_funcs->power_on = td4300_vid_power_on;
	p_funcs->drv_ic_init = td4300_vid_drv_ic_init;
	p_funcs->power_off = td4300_vid_power_off;
	p_funcs->reset = td4300_vid_reset;
	p_funcs->set_brightness = td4300_vid_set_brightness;
/*
	printk("[DISP] Novatek reset workqueue init!\n");
	INIT_DELAYED_WORK(&td4300_panel_reset_delay_work, td4300_vid_panel_reset_delay_work);
	td4300_panel_reset_delay_wq = create_workqueue("td4300_panel_reset_delay_timer");
	if (unlikely(!td4300_panel_reset_delay_wq)) {
		printk("%s : unable to create Panel reset workqueue\n", __func__);
	}
*/
#ifdef TD4300_DEBUG
	sysfs_create_group(&dev->dev->kobj, &td4300_attr_group);
#endif

}
