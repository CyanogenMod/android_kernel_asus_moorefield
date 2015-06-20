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
#include <linux/intel_mid_pm.h>
#include <asm/intel_scu_pmic.h>
#include <linux/i2c/rt4532.h>

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "displays/otm1284a_vid.h"
#include <linux/HWVersion.h>

extern int Read_PROJ_ID(void);

#define OTM1901A_DEBUG 1

/*
 * GPIO pin definition
 */
#define OTM1901A_BL_EN_GPIO   188
#define OTM1901A_BL_PWM_GPIO  183

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


struct mipi_dsi_cmd{
	int delay;
	int len;
	u8 *commands;
};

struct mipi_dsi_cmd_orise{
	int gamma_enable;
	int delay;
	int len1;
	u8 *commands1;
	int len2;
	u8 *commands2;
};

static struct mdfld_dsi_config *otm1901a_dsi_config;

static struct mipi_dsi_cmd_orise *otm1901a_power_on_table = NULL;
static int otm1901a_power_on_table_size = 0;


/* ====Initial settings==== */
static u8 cm_FF_1[] = {0xFF, 0x19, 0x01, 0x01};
static u8 cm_FF_2[] = {0xFF, 0x19, 0x01};
static u8 cm_FF_3[] = {0xFF, 0xFF, 0xFF, 0xFF};
/*
static u8 cm1_001[] = {0x00, 0xDA};
static u8 cm1_002[] = {0xF5, 0x21};
static u8 cm1_003[] = {0x00, 0x00};
static u8 cm1_004[] = {0xFB, 0x01};
static u8 cm1_005[] = {0x00, 0x00};
static u8 cm1_006[] = {0xEB, 0x10};
*/
static u8 cm1_007[] = {0x00, 0x00};
static u8 cm1_008[] = {0x1C, 0x33};
static u8 cm1_009[] = {0x00, 0xA0};
static u8 cm1_010[] = {0xC4, 0x3F, 0x2F, 0x0F, 0x3F};
static u8 cm1_011[] = {0x00, 0xA4};
static u8 cm1_012[] = {0xC4, 0x3F, 0x2F, 0x0F, 0x3F};
static u8 cm1_013[] = {0x00, 0x00};
static u8 cm1_014[] = {0xE1, 0x08, 0x10, 0x19, 0x24, 0x2C, 0x33, 0x42, 0x55, 0x60, 0x75, 0x82, 0x8C, 0x6C, 0x66, 0x62, 0x56, 0x46, 0x38, 0x2F, 0x29, 0x22, 0x18, 0x0E, 0x0D};
static u8 cm1_015[] = {0x00, 0x00};
static u8 cm1_016[] = {0xE2, 0x08, 0x10, 0x19, 0x24, 0x2C, 0x33, 0x42, 0x55, 0x60, 0x75, 0x82, 0x8C, 0x6C, 0x66, 0x62, 0x56, 0x46, 0x38, 0x2F, 0x29, 0x22, 0x18, 0x0E, 0x0D};
static u8 cm1_017[] = {0x00, 0x00};
static u8 cm1_018[] = {0xE3, 0x08, 0x10, 0x19, 0x24, 0x2C, 0x33, 0x42, 0x55, 0x60, 0x75, 0x82, 0x8C, 0x6C, 0x66, 0x62, 0x56, 0x46, 0x38, 0x2F, 0x29, 0x22, 0x18, 0x0E, 0x0D};
static u8 cm1_019[] = {0x00, 0x00};
static u8 cm1_020[] = {0xE4, 0x08, 0x10, 0x19, 0x24, 0x2C, 0x33, 0x42, 0x55, 0x60, 0x75, 0x82, 0x8C, 0x6C, 0x66, 0x62, 0x56, 0x46, 0x38, 0x2F, 0x29, 0x22, 0x18, 0x0E, 0x0D};
static u8 cm1_021[] = {0x00, 0x00};
static u8 cm1_022[] = {0xE5, 0x08, 0x10, 0x19, 0x24, 0x2C, 0x33, 0x42, 0x55, 0x60, 0x75, 0x82, 0x8C, 0x6C, 0x66, 0x62, 0x56, 0x46, 0x38, 0x2F, 0x29, 0x22, 0x18, 0x0E, 0x0D};
static u8 cm1_023[] = {0x00, 0x00};
static u8 cm1_024[] = {0xE6, 0x08, 0x10, 0x19, 0x24, 0x2C, 0x33, 0x42, 0x55, 0x60, 0x75, 0x82, 0x8C, 0x6C, 0x66, 0x62, 0x56, 0x46, 0x38, 0x2F, 0x29, 0x22, 0x18, 0x0E, 0x0D};

static u8 cm1_025[] = {0x00, 0x92};
static u8 cm1_026[] = {0xE9, 0x00};

static u8 sleep_out[] = {0x11};
static u8 sleep_in[] = {0x10};
static u8 display_on[] = {0x29};
static u8 display_off[] = {0x28};


/* ====Power on commnad==== */
static struct mipi_dsi_cmd_orise ze551ml_INX_power_on_table[] = {
//	{0, 0, sizeof(cm1_001), cm1_001, sizeof(cm1_002), cm1_002},
//	{0, 0, sizeof(cm1_003), cm1_003, sizeof(cm1_004), cm1_004},
//	{0, 0, sizeof(cm1_005), cm1_005, sizeof(cm1_006), cm1_006},
	{0, 0, sizeof(cm1_007), cm1_007, sizeof(cm1_008), cm1_008},
	{0, 0, sizeof(cm1_009), cm1_009, sizeof(cm1_010), cm1_010},
	{0, 0, sizeof(cm1_011), cm1_011, sizeof(cm1_012), cm1_012},
	{1, 0, sizeof(cm1_013), cm1_013, sizeof(cm1_014), cm1_014},
	{1, 0, sizeof(cm1_015), cm1_015, sizeof(cm1_016), cm1_016},
	{1, 0, sizeof(cm1_017), cm1_017, sizeof(cm1_018), cm1_018},
	{1, 0, sizeof(cm1_019), cm1_019, sizeof(cm1_020), cm1_020},
	{1, 0, sizeof(cm1_021), cm1_021, sizeof(cm1_022), cm1_022},
	{1, 0, sizeof(cm1_023), cm1_023, sizeof(cm1_024), cm1_024},
	{0, 0, sizeof(cm1_025), cm1_025, sizeof(cm1_026), cm1_026},
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

static int send_mipi_cmd_orise(struct mdfld_dsi_pkg_sender * sender,
				struct mipi_dsi_cmd_orise *cmd) {
	int err = 0;
	int i;
	int r;
	u8 data3[20]={0};

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	if (cmd->gamma_enable) {
		mdfld_dsi_send_mcs_short_lp(sender, cmd->commands1[0], cmd->commands1[1], 1, MDFLD_DSI_SEND_PACKAGE);
		for (i=0; i<(cmd->len2 - 1); i++)
			mdfld_dsi_send_mcs_short_lp(sender, cmd->commands2[0], cmd->commands2[i+1], 1, MDFLD_DSI_SEND_PACKAGE);
	} else {

		for (i=0; i<(cmd->len2 - 1); i++) {
			mdfld_dsi_send_mcs_short_lp(sender, cmd->commands1[0], cmd->commands1[1]+i, 1, MDFLD_DSI_SEND_PACKAGE);
			mdfld_dsi_send_mcs_short_lp(sender, cmd->commands2[0], cmd->commands2[1+i], 1, MDFLD_DSI_SEND_PACKAGE);
		}

#if 0
		printk("-----------------------\n");
		r = mdfld_dsi_send_mcs_short_lp(sender, 0x0 , cmd->commands1[1], 1, 0);
		r = mdfld_dsi_read_gen_lp(sender,cmd->commands2[0],0,1,data3, cmd->len2-1);

		printk("read: %d, 0x%02x%02x",r,cmd->commands2[0], cmd->commands1[1]);
		for(i=0;i<cmd->len2-1;i++){
			printk(" 0x%02x", data3[i]);
		}
		printk("\n");
#endif
	}

	if (err != 0 || sender->status) {
		printk("[DISP] %s : sent failed with status=%d\n", __func__, sender->status);
		return -EIO;
	}

	if (cmd->delay)
		mdelay(cmd->delay);

	return 0;

}


static int otm1901a_vid_drv_ic_init(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	printk("[DISP] %s\n", __func__);

	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(10000, 10100);
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10100);

	/* panel initial settings */
	mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender, cm_FF_1, sizeof(cm_FF_1), MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_gen_long_lp(sender, cm_FF_2, sizeof(cm_FF_2), MDFLD_DSI_SEND_PACKAGE);

	for(i = 0; i < otm1901a_power_on_table_size; i++)
		send_mipi_cmd_orise(sender, &otm1901a_power_on_table[i]);

	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(200);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	return 0;
}

static void
otm1901a_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
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
	hw_ctx->high_low_switch_count = 0x2c;
	hw_ctx->clk_lane_switch_time_cnt = 0x390015;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->dphy_param = 0x3119651d;

	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0x7D0;
	hw_ctx->dsi_func_prg = (RGB_888_FMT << FMT_DPI_POS) |
		dsi_config->lane_count;
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		BANDGAP_CHICKEN_BIT;
	hw_ctx->video_mode_format = 0xF;

	otm1901a_dsi_config = dsi_config;

	/* Panel initial settings assigned */
	printk("[DISP] Load INX panel initial settings.\n");
	otm1901a_power_on_table = ze551ml_INX_power_on_table;
	otm1901a_power_on_table_size = ARRAY_SIZE(ze551ml_INX_power_on_table);

}

static int otm1901a_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	usleep_range(1000, 1100);

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


static int otm1901a_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

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

	usleep_range(50000, 55000);

	return 0;
}

static int otm1901a_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	printk("[DISP] %s\n", __func__);

	/* Open 2V9 power */
	__vpro3_power_ctrl(true);
	usleep_range(10000, 10100);
/* Move to drv_init
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(10000, 10100);
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10100);
*/
	return 0;
}


static int otm1901a_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	u32 reg_level = ~level & 0xFF;
	union pwmctrl_reg pwmctrl;

#ifdef CONFIG_BACKLIGHT_RT4532
	rt4532_brightness_set(level);
#endif

	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = reg_level;

	if (!pwmctrl_mmio)
		pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG, 4);

	if (pwmctrl_mmio) {
		if (level) {
			if (!gpio_get_value(backlight_en_gpio)) {
				pmu_set_pwm(PCI_D0);
				lnw_gpio_set_alt(backlight_pwm_gpio, 1);
				gpio_set_value_cansleep(backlight_en_gpio, 1);
			}

			pwmctrl.part.pwmenable = 1;
			writel(pwmctrl.full, pwmctrl_mmio);
		} else if (gpio_get_value(backlight_en_gpio)) {
			writel(pwmctrl.full, pwmctrl_mmio);
			pwmctrl.part.pwmenable = 0;
			gpio_set_value_cansleep(backlight_en_gpio, 0);
			lnw_gpio_set_alt(backlight_pwm_gpio, 0);
			pmu_set_pwm(PCI_D3hot);
		}
	} else {
		DRM_ERROR("Cannot map pwmctrl\n");
	}

	printk("[DISP] brightness level = %d\n", level);

	return 0;
}

struct drm_display_mode *otm1901a_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	printk("[DISP] %s\n", __func__);

	/* RECOMMENDED PORCH SETTING
		HSA=4, HBP=50, HFP=50
		VSA=1,   VBP=9, VFP=14	 */
	mode->hdisplay = 1080;
	mode->hsync_start = mode->hdisplay + 50;
	mode->hsync_end = mode->hsync_start + 4;
	mode->htotal = mode->hsync_end + 50;

	mode->vdisplay = 1920;
	mode->vsync_start = mode->vdisplay + 16;
	mode->vsync_end = mode->vsync_start + 1;
	mode->vtotal = mode->vsync_end + 15;


	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void otm1901a_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	pi->width_mm = 70;
	pi->height_mm = 129;
}

static int otm1901a_vid_detect(struct mdfld_dsi_config *dsi_config)
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

#ifndef CONFIG_BACKLIGHT_RT4532
	backlight_en_gpio = OTM1901A_BL_EN_GPIO;
	if (gpio_request(backlight_en_gpio, "backlight_en")) {
		DRM_ERROR("Faild to request backlight enable gpio\n");
		return -EINVAL;
	}

	backlight_pwm_gpio = OTM1901A_BL_PWM_GPIO;
	if (gpio_request(backlight_pwm_gpio, "backlight_pwm")) {
		DRM_ERROR("Faild to request backlight PWM gpio\n");
		return -EINVAL;
	}

	/* Initializing pwm for being able to adjust backlight when just opening the phone. */
	pmu_set_pwm(PCI_D0);
	lnw_gpio_set_alt(backlight_pwm_gpio, 1);
#endif
	dsi_config->dsi_hw_context.panel_on = false;

	return MDFLD_DSI_PANEL_CONNECTED;
}


#ifdef OTM1901A_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(otm1901a_dsi_config);

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
			= mdfld_dsi_get_pkg_sender(otm1901a_dsi_config);

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

DEVICE_ATTR(send_mipi_otm1901a,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_otm1901a,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *otm1901a_attrs[] = {
        &dev_attr_send_mipi_otm1901a.attr,
        &dev_attr_read_mipi_otm1901a.attr,
        NULL
};

static struct attribute_group otm1901a_attr_group = {
        .attrs = otm1901a_attrs,
        .name = "otm1901a",
};

#endif

void otm1901a_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	printk("[DISP] %s\n", __func__);

	p_funcs->get_config_mode = otm1901a_vid_get_config_mode;
	p_funcs->get_panel_info = otm1901a_vid_get_panel_info;
	p_funcs->dsi_controller_init = otm1901a_vid_dsi_controller_init;
	p_funcs->detect = otm1901a_vid_detect;
	p_funcs->power_on = otm1901a_vid_power_on;
	p_funcs->drv_ic_init = otm1901a_vid_drv_ic_init;
	p_funcs->power_off = otm1901a_vid_power_off;
	p_funcs->reset = otm1901a_vid_reset;
	p_funcs->set_brightness = otm1901a_vid_set_brightness;

#ifdef OTM1901A_DEBUG
	sysfs_create_group(&dev->dev->kobj, &otm1901a_attr_group);
#endif
}
