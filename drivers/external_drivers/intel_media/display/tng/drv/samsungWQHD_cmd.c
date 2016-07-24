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

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "displays/samsungWQHD_cmd.h"
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>

#define SAMSUNG_WQHD_DEBUG 1

/*
 * GPIO pin definition
 */
#define SAMSUNG_WQHD_BL_EN_GPIO   188
#define SAMSUNG_WQHD_BL_PWM_GPIO  183

static int panel_reset_gpio;
static int backlight_en_gpio;
static int backlight_pwm_gpio;
static int stb1_en_gpio;
static int stb2_en_gpio;


struct mipi_dsi_cmd{
	int delay;
	int len;
	u8 *commands;
};

static struct mdfld_dsi_config *samsungWQHD_dsi_config;

static struct mipi_dsi_cmd *samsungWQHD_power_on_table = NULL;
static int samsungWQHD_power_on_table_size = 0;


/* ====Initial settings==== */
static u8 cm1_01[] = {0xF0, 0x5A, 0x5A};
static u8 cm1_02[] = {0xC4, 0x03};
static u8 cm1_03[] = {0xF9, 0x03};
static u8 cm1_04[] = {0xC2,
		      0x00, 0x08, 0xD8, 0xD8, 0x00, 0x80, 0x2B, 0x05,
		      0x08, 0x0E, 0x07, 0x0B, 0x05, 0x0D, 0x0A, 0x15,
		      0x13, 0x20, 0x1E};
static u8 cm1_05[] = {0xF0, 0xA5, 0xA5};

static u8 cm1_06[] = {0x35, 0x00};
static u8 cm1_07[] = {0x36, 0x00};
static u8 cm1_08[] = {0xF0, 0x5A, 0x5A};
static u8 cm1_09[] = {0xCC, 0x4C, 0x50};
static u8 cm1_10[] = {0xED, 0x44};
static u8 cm1_11[] = {0xF0, 0xA5, 0xA5};

static u8 cm1_12[] = {0x53, 0x24};
//static u8 cm1_13[] = {0x51, 0xFF};
static u8 cm1_14[] = {0x55, 0x03};


/* ====Power on commnad==== */
static struct mipi_dsi_cmd ze553ml_power_on_table[] = {
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
//	{0, sizeof(cm1_13), cm1_13},
	{0, sizeof(cm1_14), cm1_14},
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



static int samsungWQHD_cmd_drv_ic_init(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	printk("[DISP] %s\n", __func__);

	/* HW_RST control */
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(1000, 1500);
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(1000, 1500);
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10500);
	/* HW_RST control */


	/* panel initial settings */
	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	usleep_range(20000, 20100);

	for (i = 0; i < samsungWQHD_power_on_table_size; i++)
		send_mipi_cmd_mcs(sender, &samsungWQHD_power_on_table[i]);

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


static int samsungWQHD_cmd_exit_deep_standby(struct mdfld_dsi_config *dsi_config)
{
	printk("[DISP] %s\n", __func__);

	/* Open 2V9 power */
	__vpro3_power_ctrl(true);
	usleep_range(1000, 1100);
	/* Open  VBAT*/
	gpio_set_value_cansleep(stb1_en_gpio, 1);
	gpio_set_value_cansleep(stb2_en_gpio, 1);
	usleep_range(15000, 15100);

	return 0;
}


static void samsungWQHD_cmd_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
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
	hw_ctx->mipi_control = 0x0;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xFFFF;
	hw_ctx->turn_around_timeout = 0xFFFF;

	hw_ctx->high_low_switch_count = 0x2c;
	hw_ctx->clk_lane_switch_time_cnt =  0x2e0016;
	hw_ctx->lp_byteclk = 0x6;
	hw_ctx->dphy_param = 0x2a18681f;

	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0x7D0;
	hw_ctx->dbi_bw_ctrl = 0x400;
	hw_ctx->hs_ls_dbi_enable = 0x0;
	hw_ctx->dsi_func_prg = ((DBI_DATA_WIDTH_OPT2 << 13) |
				dsi_config->lane_count);
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE |
			BANDGAP_CHICKEN_BIT | TE_TRIGGER_GPIO_PIN | DUAL_LINK_ENABLE | DUAL_LINK_CAPABLE;

	hw_ctx->video_mode_format = 0xf;

	samsungWQHD_dsi_config = dsi_config;

	/* Panel initial settings assigned */
	samsungWQHD_power_on_table = ze553ml_power_on_table;
	samsungWQHD_power_on_table_size = ARRAY_SIZE(ze553ml_power_on_table);
}

static int samsungWQHD_cmd_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	usleep_range(120000, 120100);

	return 0;
}

static int samsungWQHD_cmd_power_off(struct mdfld_dsi_config *dsi_config)
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
	err = mdfld_dsi_send_mcs_short_hs(sender, 0x28, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Set Display Off\n");
		return err;
	}
	usleep_range(35000, 35500);
	err = mdfld_dsi_send_mcs_short_hs(sender, 0x10, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("Failed to Enter Sleep Mode\n");
		return err;
	}
	usleep_range(150000, 150500);

	/* Turn off VBAT*/
	gpio_set_value_cansleep(stb1_en_gpio, 0);
	gpio_set_value_cansleep(stb2_en_gpio, 0);
	return 0;
}

static int samsungWQHD_cmd_reset(struct mdfld_dsi_config *dsi_config)
{
	printk("[DISP] %s\n", __func__);
#if 0
	/* Open 2V9 power */
	__vpro3_power_ctrl(true);
	usleep_range(15000, 15100);
/* postpone to drv_ic_init
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10100);
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(10000, 10100);
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10100);
*/
#endif
	return 0;
}


static int samsungWQHD_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	struct mdfld_dsi_pkg_sender *sender =
			mdfld_dsi_get_pkg_sender(dsi_config);
	u8 duty_val = 0;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	duty_val = (0xFF * level) / 255;
	mdfld_dsi_send_mcs_short_hs(sender,
				0x51, duty_val, 1,
				MDFLD_DSI_SEND_PACKAGE);

	printk("[DISP] brightness level = %d\n", level);

	return 0;
}

struct drm_display_mode *samsungWQHD_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	printk("[DISP] %s\n", __func__);

	/* RECOMMENDED PORCH SETTING
		HSA=, HBP=, HFP=
		VSA=, VBP=, VFP=	 */
	mode->hdisplay = 1440;
	mode->hsync_start = mode->hdisplay + 48;
	mode->hsync_end = mode->hsync_start + 32;
	mode->htotal = mode->hsync_end + 80;

	mode->vdisplay = 2560;
	mode->vsync_start = mode->vdisplay + 3;
	mode->vsync_end = mode->vsync_start + 33;
	mode->vtotal = mode->vsync_end + 10;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void samsungWQHD_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	pi->width_mm = 68;
	pi->height_mm = 121;
}

static int samsungWQHD_cmd_detect(struct mdfld_dsi_config *dsi_config)
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

	backlight_en_gpio = SAMSUNG_WQHD_BL_EN_GPIO;
	if (gpio_request(backlight_en_gpio, "backlight_en")) {
		DRM_ERROR("Faild to request backlight enable gpio\n");
		return -EINVAL;
	}

	backlight_pwm_gpio = SAMSUNG_WQHD_BL_PWM_GPIO;
	if (gpio_request(backlight_pwm_gpio, "backlight_pwm")) {
		DRM_ERROR("Faild to request backlight PWM gpio\n");
		return -EINVAL;
	}

	/* Initializing pwm for being able to adjust backlight when just opening the phone. */
//	pmu_set_pwm(PCI_D0);
//	lnw_gpio_set_alt(backlight_pwm_gpio, 1);

	dsi_config->dsi_hw_context.panel_on = false;

	return MDFLD_DSI_PANEL_CONNECTED;
}


#ifdef SAMSUNG_WQHD_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(samsungWQHD_dsi_config);

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
			= mdfld_dsi_get_pkg_sender(samsungWQHD_dsi_config);

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

DEVICE_ATTR(send_mipi_samsungWQHD,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_samsungWQHD,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *samsungWQHD_attrs[] = {
        &dev_attr_send_mipi_samsungWQHD.attr,
        &dev_attr_read_mipi_samsungWQHD.attr,
        NULL
};

static struct attribute_group samsungWQHD_attr_group = {
        .attrs = samsungWQHD_attrs,
        .name = "SAMSUNG_WQHD",
};

#endif

void samsungWQHD_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	printk("[DISP] %s\n", __func__);

	p_funcs->get_config_mode = samsungWQHD_cmd_get_config_mode;
	p_funcs->reset = samsungWQHD_cmd_reset;
	p_funcs->power_on = samsungWQHD_cmd_power_on;
	p_funcs->power_off = samsungWQHD_cmd_power_off;
	p_funcs->drv_ic_init = samsungWQHD_cmd_drv_ic_init;
	p_funcs->get_panel_info = samsungWQHD_cmd_get_panel_info;
	p_funcs->dsi_controller_init = samsungWQHD_cmd_dsi_controller_init;
	p_funcs->detect = samsungWQHD_cmd_detect;
	p_funcs->set_brightness = samsungWQHD_cmd_set_brightness;
	p_funcs->exit_deep_standby = samsungWQHD_cmd_exit_deep_standby;


#ifdef SAMSUNG_WQHD_DEBUG
	sysfs_create_group(&dev->dev->kobj, &samsungWQHD_attr_group);
#endif

}
