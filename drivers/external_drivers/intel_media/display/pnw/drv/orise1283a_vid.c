/*
 * Copyright (c)  2012 Intel Corporation
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

#include "displays/orise1283a_vid.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include <linux/gpio.h>
#include "psb_drv.h"
#include <linux/lnw_gpio.h>
#include <linux/init.h>
#include <asm/intel_scu_pmic.h>

#include <linux/HWVersion.h>

extern int Read_PROJ_ID(void);
static int board_proj_id=0;

#define ORISE1283A_PANEL_NAME	"ORISE1283A"

#define ORISE1283A_DEBUG 1
#define ENABLE_CSC_GAMMA 1
#define ENABLE_SHORT_PACKAGE_CMD 1

struct delayed_work panel_reset_delay_work;
struct workqueue_struct *panel_reset_delay_wq;

static int orise1283a_vid_drv_ic_reset_workaround(struct mdfld_dsi_config *dsi_config) {

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(10);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);
/*	mdfld_dsi_send_mcs_short_lp(sender, 0x20, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, 0x13, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, 0x38, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, 0x36, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_lp(sender, 0x55, 0, 0, MDFLD_DSI_SEND_PACKAGE);
*/
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}

	return 0;
}

static void orise1283a_vid_panel_reset_delay_work(struct work_struct *work)
{
//	printk("[DEBUG] %s\n", __func__);
	orise1283a_vid_drv_ic_reset_workaround(panel_reset_dsi_config);
	queue_delayed_work(panel_reset_delay_wq, &panel_reset_delay_work, msecs_to_jiffies(3000));
}


/*
 * GPIO pin definition
 */
#define PMIC_GPIO_BACKLIGHT_EN	0x7E
#define PMIC_GPIO_VEMMC2CNT	0xDA 	//panel power control 2.8v


#define DISP_RST_N		57


#define PWM_SOC_ENABLE 1
#define PWMCTRL_REG 0xffae9000
#define PWMCTRL_SIZE 0x80
static void __iomem *pwmctrl_mmio;
#define PWM_ENABLE_GPIO 49
//#define PWM_BASE_UNIT 0x444 //5,000Hz
#define PWM_BASE_UNIT 0x1555 //25,000Hz


#if PWM_SOC_ENABLE
union sst_pwmctrl_reg {
	struct {
		u32 pwmtd:8;
		u32 pwmbu:22;
		u32 pwmswupdate:1;
		u32 pwmenable:1;
	} part;
	u32 full;
};

static int pwm_configure(int duty)
{
	union sst_pwmctrl_reg pwmctrl;

	/*Read the PWM register to make sure there is no pending
	*update.
	*/
	pwmctrl.full = readl(pwmctrl_mmio);

	/*check pwnswupdate bit */
	if (pwmctrl.part.pwmswupdate)
		return -EBUSY;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = duty;
	writel(pwmctrl.full,  pwmctrl_mmio);

	return 0;
}

static void pwm_enable(){
	union sst_pwmctrl_reg pwmctrl;

	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);

	/*Enable the PWM by setting PWM enable bit to 1 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 1;
	writel(pwmctrl.full, pwmctrl_mmio);
}

static void pwm_disable(){
	union sst_pwmctrl_reg pwmctrl;
	/*setting PWM enable bit to 0 */
	pwmctrl.full = readl(pwmctrl_mmio);
	pwmctrl.part.pwmenable = 0;
	writel(pwmctrl.full,  pwmctrl_mmio);

	gpio_set_value(PWM_ENABLE_GPIO, 0);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, 0);
}
#endif

struct orise1283a_vid_data{
	u8 project_id;
	unsigned int gpio_lcd_en;
	unsigned int gpio_bl_en;
	unsigned int gpio_stb1_en;
	unsigned int gpio_stb2_en;
	unsigned int gpio_lcd_rst;
};
struct mipi_dsi_cmd{
	int delay;
	int len;
	u8 *commands;
};

static struct orise1283a_vid_data gpio_settings_data;

static struct mdfld_dsi_config *orise1283a_dsi_config;

#define CMD_SIZE(x) (sizeof((x)) / sizeof((x)[0]))

/* ====Initial settings==== */
/* OTM1283A settings */
#if !ENABLE_SHORT_PACKAGE_CMD
static u8 cm_00[] = {0x00, 0x00};
static u8 cm_80[] = {0x00, 0x80};
static u8 cm_81[] = {0x00, 0x81};
static u8 cm_82[] = {0x00, 0x82};
static u8 cm_87[] = {0x00, 0x87};
static u8 cm_8E[] = {0x00, 0x8E};
static u8 cm_90[] = {0x00, 0x90};
static u8 cm_91[] = {0x00, 0x91};
static u8 cm_94[] = {0x00, 0x94};
static u8 cm_97[] = {0x00, 0x97};
static u8 cm_9E[] = {0x00, 0x9E};
static u8 cm_A0[] = {0x00, 0xA0};
static u8 cm_A2[] = {0x00, 0xA2};
static u8 cm_A3[] = {0x00, 0xA3};
static u8 cm_A4[] = {0x00, 0xA4};
static u8 cm_A7[] = {0x00, 0xA7};
static u8 cm_AE[] = {0x00, 0xAE};
static u8 cm_B0[] = {0x00, 0xB0};
static u8 cm_B2[] = {0x00, 0xB2};
static u8 cm_B3[] = {0x00, 0xB3};
static u8 cm_B4[] = {0x00, 0xB4};
static u8 cm_B5[] = {0x00, 0xB5};
static u8 cm_B6[] = {0x00, 0xB6};
static u8 cm_B7[] = {0x00, 0xB7};
static u8 cm_B8[] = {0x00, 0xB8};
static u8 cm_BA[] = {0x00, 0xBA};
static u8 cm_BB[] = {0x00, 0xBB};
static u8 cm_BE[] = {0x00, 0xBE};
static u8 cm_C0[] = {0x00, 0xC0};
static u8 cm_C6[] = {0x00, 0xC6};
static u8 cm_C7[] = {0x00, 0xC7};
static u8 cm_CE[] = {0x00, 0xCE};
static u8 cm_D0[] = {0x00, 0xD0};
static u8 cm_D7[] = {0x00, 0xD7};
static u8 cm_DE[] = {0x00, 0xDE};
static u8 cm_E0[] = {0x00, 0xE0};
static u8 cm_E7[] = {0x00, 0xE7};
static u8 cm_F0[] = {0x00, 0xF0};
static u8 cm_F7[] = {0x00, 0xF7};

static u8 cm_36_1[] = {0x36, 0x00};
static u8 cm_C0_1[] = {0xC0, 0x00, 0x64, 0x00, 0x10, 0x10, 0x00, 0x64};
static u8 cm_C0_2[] = {0xC0, 0x10, 0x10};
static u8 cm_C0_3[] = {0xC0, 0x00, 0x56, 0x00, 0x01, 0x00, 0x04};
static u8 cm_C0_4[] = {0xC0, 0x10};
static u8 cm_C0_5[] = {0xC0, 0x00, 0x50};
static u8 cm_C0_6[] = {0xC0, 0x55};
static u8 cm_C0_7[] = {0xC0, 0x00};
static u8 cm_C0_8[] = {0xC0, 0x15};
static u8 cm_C0_9[] = {0xC0, 0x16};
static u8 cm_C1_1[] = {0xC1, 0x55};
static u8 cm_C4_1[] = {0xC4, 0x49};
static u8 cm_C4_2[] = {0xC4 ,0x05, 0x10, 0x06, 0x02, 0x05, 0x15, 0x10};
static u8 cm_C4_3[] = {0xC4, 0x05, 0x10, 0x07, 0x02, 0x05, 0x15, 0x10};
static u8 cm_C4_4[] = {0xC4, 0x00, 0x00};
static u8 cm_C4_5[] = {0xC4, 0x82};
static u8 cm_C4_6[] = {0xC4, 0x02};
static u8 cm_C5_1[] = {0xC5, 0x46, 0x40};
static u8 cm_C5_2[] = {0xC5, 0x04, 0xB8};
static u8 cm_C5_3[] = {0xC5, 0x80};
static u8 cm_C5_4[] = {0xC5, 0x00, 0x6F, 0xFF, 0x00, 0x6F, 0xFF};
static u8 cm_C5_5[] = {0xC5, 0x50};
static u8 cm_C5_6[] = {0xC5, 0x66};
static u8 cm_C5_7[] = {0xC5, 0xC0};

static u8 cm_D8_1[] = {0xD8, 0xCF, 0xCF};
//static u8 cm_D9_1[] = {0xD9, 0x9B};
static u8 cm_B0_1[] = {0xB0, 0x03};
static u8 cm_D0_1[] = {0xD0, 0x40};
static u8 cm_D1_1[] = {0xD1, 0x00, 0x00};

static u8 cm_CB_1[] = {0xCB, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_CB_2[] = {0xCB, 0x00, 0x00, 0x00, 0x00};
static u8 cm_CB_3[] = {0xCB, 0x00};
static u8 cm_CB_4[] = {0xCB, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x00};
static u8 cm_CB_5[] = {0xCB, 0x00, 0x00, 0x00, 0x05, 0x05, 0x00, 0x05};
static u8 cm_CB_6[] = {0xCB, 0x05};
static u8 cm_CB_7[] = {0xCB, 0x00, 0x00, 0x05, 0x05, 0x00, 0x05, 0x05};
static u8 cm_CB_8[] = {0xCB, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
static u8 cm_CB_9[] = {0xCB, 0xff, 0xff, 0xff, 0xff};

static u8 cm_CC_1[] = {0xCC, 0x0E, 0x10, 0x0A, 0x0C, 0x02, 0x04, 0x00};
static u8 cm_CC_2[] = {0xCC, 0x00, 0x00, 0x00, 0x2E, 0x2D, 0x00, 0x29};
static u8 cm_CC_3[] = {0xCC, 0x2A};
static u8 cm_CC_4[] = {0xCC, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_CC_5[] = {0xCC, 0x00};
static u8 cm_CC_6[] = {0xCC, 0x0D, 0x0F, 0x09, 0x0B, 0x01, 0x03, 0x00};
static u8 cm_CC_7[] = {0xCC, 0x00, 0x00, 0x2E, 0x2D, 0x00, 0x29, 0x2A};
static u8 cm_CC_8[] = {0xCC, 0x0B, 0x09, 0x0F, 0x0D, 0x03, 0x01, 0x00};
static u8 cm_CC_9[] = {0xCC, 0x00, 0x00, 0x00, 0x2D, 0x2E, 0x00, 0x29};
static u8 cm_CC_10[] = {0xCC, 0x0C, 0x0A, 0x10, 0x0E, 0x04, 0x02, 0x00};
static u8 cm_CC_11[] = {0xCC, 0x00, 0x00, 0x2D, 0x2E, 0x00, 0x29, 0x2A};

static u8 cm_CE_1[] = {0xCE, 0x8B, 0x03, 0x18, 0x8A, 0x03, 0x18, 0x89};
static u8 cm_CE_2[] = {0xCE, 0x03, 0x18, 0x88, 0x03, 0x18};
static u8 cm_CE_3[] = {0xCE, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm_CE_4[] = {0xCE, 0x38, 0x07, 0x05, 0x00, 0x00, 0x18, 0x00};
static u8 cm_CE_5[] = {0xCE, 0x38, 0x06, 0x05, 0x01, 0x00, 0x18, 0x00};
static u8 cm_CE_6[] = {0xCE, 0x38, 0x05, 0x05, 0x02, 0x00, 0x18, 0x00};
static u8 cm_CE_7[] = {0xCE, 0x38, 0x04, 0x05, 0x03, 0x00, 0x18, 0x00};
static u8 cm_CE_8[] = {0xCE, 0x38, 0x03, 0x05, 0x04, 0x00, 0x18, 0x00};
static u8 cm_CE_9[] = {0xCE, 0x38, 0x02, 0x05, 0x05, 0x00, 0x18, 0x00};
static u8 cm_CE_10[] = {0xCE, 0x38, 0x01, 0x05, 0x06, 0x00, 0x18, 0x00};
static u8 cm_CE_11[] = {0xCE, 0x38, 0x00, 0x05, 0x07, 0x00, 0x18, 0x00};

static u8 cm_CF_1[] = {0xCF, 0x00, 0x00, 0x00, 0x00, 0x00 ,0x00, 0x00};
static u8 cm_CF_2[] = {0xCF, 0x3D, 0x02, 0x15, 0x20, 0x00, 0x00, 0x01};
static u8 cm_CF_3[] = {0xCF, 0x81, 0x00, 0x03, 0x08};

static u8 cm_F5_1[] = {0xF5, 0x02, 0x11, 0x02, 0x11};
static u8 cm_F5_2[] = {0xF5, 0x00, 0x00};
static u8 cm_F5_3[] = {0xF5, 0x02};
static u8 cm_F5_4[] = {0xF5, 0x03};


static u8 cm_E1_1[] = {0xE1, 0x00, 0x0c, 0x12, 0x0d, 0x06, 0x0e, 0x0a,
	0x09, 0x05, 0x07, 0x0f, 0x09, 0x10, 0x12, 0x0d, 0x00};
static u8 cm_E2_1[] = {0xE2, 0x00, 0x0c, 0x12, 0x0d, 0x06, 0x0e, 0x0a,
	0x09, 0x05, 0x07, 0x0f, 0x09, 0x10, 0x12, 0x0d, 0x00};
static u8 cm_E1_2[] = {0xE1, 0x00, 0x09, 0x0f, 0x0d, 0x08, 0x0f, 0x0c,
	0x09, 0x04, 0x07, 0x0d, 0x09, 0x10, 0x11, 0x0b, 0x00};
static u8 cm_E2_2[] = {0xE2, 0x00, 0x09, 0x0f, 0x0d, 0x08, 0x0f, 0x0c,
	0x09, 0x04, 0x07, 0x0d, 0x09, 0x10, 0x11, 0x0b, 0x00};
#endif

static u8 cm_FF_1[] = {0xFF, 0x12, 0x83, 0x01};
static u8 cm_FF_2[] = {0xFF, 0x12, 0x83};
static u8 cm_FF_3[] = {0xFF, 0xFF, 0xFF, 0xFF};

static u8 sleep_out[] = {0x11};
static u8 sleep_in[] = {0x10};
static u8 display_on[] = {0x29};
static u8 display_off[] = {0x28};

/* ====Power on commnad==== */
/* OTM1283A settings */
#if !ENABLE_SHORT_PACKAGE_CMD
static struct mipi_dsi_cmd a500cg_power_on_table[] = {
	//Command2
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_FF_1), cm_FF_1},
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_FF_2), cm_FF_2},
	//PANEL setting
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_C0_1), cm_C0_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_C0_2), cm_C0_2},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_C0_3), cm_C0_3},
	{0, sizeof(cm_A2), cm_A2},
	{0, sizeof(cm_C0_7), cm_C0_7},
	{0, sizeof(cm_A3), cm_A3},
	{0, sizeof(cm_C0_8), cm_C0_8},
	{0, sizeof(cm_A4), cm_A4},
	{0, sizeof(cm_C0_4), cm_C0_4},
	{0, sizeof(cm_B3), cm_B3},
	{0, sizeof(cm_C0_5), cm_C0_5},
	{0, sizeof(cm_81), cm_81},
	{0, sizeof(cm_C1_1), cm_C1_1},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_C4_1), cm_C4_1},
	{0, sizeof(cm_B4), cm_B4},
	{0, sizeof(cm_C0_6), cm_C0_6},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_36_1), cm_36_1},
	//POWER SETTING
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_C4_2), cm_C4_2},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_C4_3), cm_C4_3},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_C4_4), cm_C4_4},
	{0, sizeof(cm_91), cm_91},
	{0, sizeof(cm_C5_1), cm_C5_1},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_D8_1), cm_D8_1},
//	{0, sizeof(cm_00), cm_00},
//	{0, sizeof(cm_D9_1), cm_D9_1},
	{0, sizeof(cm_81), cm_81},
	{0, sizeof(cm_C4_5), cm_C4_5},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_C5_2), cm_C5_2},
	{0, sizeof(cm_BB), cm_BB},
	{0, sizeof(cm_C5_3), cm_C5_3},
	{0, sizeof(cm_82), cm_82},
	{0, sizeof(cm_C4_6), cm_C4_6},
	{0, sizeof(cm_C6), cm_C6},
	{0, sizeof(cm_B0_1), cm_B0_1},
	//CONTROL SETTING
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_D0_1), cm_D0_1},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_D1_1), cm_D1_1},
	//PANEL TIMING STATE
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CB_2), cm_CB_2},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_9E), cm_9E},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_AE), cm_AE},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_BE), cm_BE},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CB_4), cm_CB_4},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CB_5), cm_CB_5},
	{0, sizeof(cm_CE), cm_CE},
	{0, sizeof(cm_CB_6), cm_CB_6},
	{0, sizeof(cm_D0), cm_D0},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_D7), cm_D7},
	{0, sizeof(cm_CB_4), cm_CB_4},
	{0, sizeof(cm_DE), cm_DE},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_E0), cm_E0},
	{0, sizeof(cm_CB_7), cm_CB_7},
	{0, sizeof(cm_E7), cm_E7},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_F0), cm_F0},
	{0, sizeof(cm_CB_8), cm_CB_8},
	{0, sizeof(cm_F7), cm_F7},
	{0, sizeof(cm_CB_9), cm_CB_9},
	//PANEL PAD MAPPING
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CC_1), cm_CC_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CC_2), cm_CC_2},
	{0, sizeof(cm_8E), cm_8E},
	{0, sizeof(cm_CC_3), cm_CC_3},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CC_4), cm_CC_4},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CC_6), cm_CC_6},
	{0, sizeof(cm_9E), cm_9E},
	{0, sizeof(cm_CC_5), cm_CC_5},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CC_7), cm_CC_7},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CC_4), cm_CC_4},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CC_8), cm_CC_8},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CC_9), cm_CC_9},
	{0, sizeof(cm_BE), cm_BE},
	{0, sizeof(cm_CC_3), cm_CC_3},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CC_4), cm_CC_4},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CC_10), cm_CC_10},
	{0, sizeof(cm_CE), cm_CE},
	{0, sizeof(cm_CC_5), cm_CC_5},
	{0, sizeof(cm_D0), cm_D0},
	{0, sizeof(cm_CC_11), cm_CC_11},
	{0, sizeof(cm_D7), cm_D7},
	{0, sizeof(cm_CC_4), cm_CC_4},
	//PANEL TIMING SETTING
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CE_1), cm_CE_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CE_2), cm_CE_2},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CE_3), cm_CE_3},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CE_3), cm_CE_3},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CE_4), cm_CE_4},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CE_5), cm_CE_5},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CE_6), cm_CE_6},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CE_7), cm_CE_7},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CE_8), cm_CE_8},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CE_9), cm_CE_9},
	{0, sizeof(cm_D0), cm_D0},
	{0, sizeof(cm_CE_10), cm_CE_10},
	{0, sizeof(cm_D7), cm_D7},
	{0, sizeof(cm_CE_11), cm_CE_11},
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CF_2), cm_CF_2},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CF_3), cm_CF_3},
	{0, sizeof(cm_B5), cm_B5},
	{0, sizeof(cm_C5_4), cm_C5_4},
	//FOR POWER IC
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_F5_1), cm_F5_1},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_C5_5), cm_C5_5},
	{0, sizeof(cm_94), cm_94},
	{0, sizeof(cm_C5_6), cm_C5_6},
	//VGL01/02
	{0, sizeof(cm_B2), cm_B2},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_B4), cm_B4},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_B6), cm_B6},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_B8), cm_B8},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_94), cm_94},
	{0, sizeof(cm_F5_3), cm_F5_3},
	{0, sizeof(cm_BA), cm_BA},
	{0, sizeof(cm_F5_4), cm_F5_4},
	{0, sizeof(cm_B4), cm_B4},
	{0, sizeof(cm_C5_7), cm_C5_7},
	//GAMMA
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_E1_1), cm_E1_1},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_E2_1), cm_E2_1},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_FF_3), cm_FF_3},
};

static struct mipi_dsi_cmd a600cg_power_on_table[] = {
	//Command2
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_FF_1), cm_FF_1},
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_FF_2), cm_FF_2},
	//PANEL setting
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_C0_1), cm_C0_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_C0_2), cm_C0_2},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_C0_3), cm_C0_3},
	{0, sizeof(cm_A2), cm_A2},
	{0, sizeof(cm_C0_7), cm_C0_7},
	{0, sizeof(cm_A3), cm_A3},
	{0, sizeof(cm_C0_7), cm_C0_7},
	{0, sizeof(cm_A4), cm_A4},
	{0, sizeof(cm_C0_9), cm_C0_9},
	{0, sizeof(cm_B3), cm_B3},
	{0, sizeof(cm_C0_5), cm_C0_5},
	{0, sizeof(cm_81), cm_81},
	{0, sizeof(cm_C1_1), cm_C1_1},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_C4_1), cm_C4_1},
	{0, sizeof(cm_B4), cm_B4},
	{0, sizeof(cm_C0_6), cm_C0_6},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_36_1), cm_36_1},
	//POWER SETTING
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_C4_2), cm_C4_2},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_C4_3), cm_C4_3},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_C4_4), cm_C4_4},
	{0, sizeof(cm_91), cm_91},
	{0, sizeof(cm_C5_1), cm_C5_1},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_D8_1), cm_D8_1},
//	{0, sizeof(cm_00), cm_00},
//	{0, sizeof(cm_D9_1), cm_D9_1},
	{0, sizeof(cm_81), cm_81},
	{0, sizeof(cm_C4_5), cm_C4_5},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_C5_2), cm_C5_2},
	{0, sizeof(cm_BB), cm_BB},
	{0, sizeof(cm_C5_3), cm_C5_3},
	{0, sizeof(cm_82), cm_82},
	{0, sizeof(cm_C4_6), cm_C4_6},
	{0, sizeof(cm_C6), cm_C6},
	{0, sizeof(cm_B0_1), cm_B0_1},
	//CONTROL SETTING
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_D0_1), cm_D0_1},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_D1_1), cm_D1_1},
	//PANEL TIMING STATE
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CB_2), cm_CB_2},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_9E), cm_9E},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_AE), cm_AE},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_BE), cm_BE},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CB_4), cm_CB_4},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CB_5), cm_CB_5},
	{0, sizeof(cm_CE), cm_CE},
	{0, sizeof(cm_CB_6), cm_CB_6},
	{0, sizeof(cm_D0), cm_D0},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_D7), cm_D7},
	{0, sizeof(cm_CB_4), cm_CB_4},
	{0, sizeof(cm_DE), cm_DE},
	{0, sizeof(cm_CB_3), cm_CB_3},
	{0, sizeof(cm_E0), cm_E0},
	{0, sizeof(cm_CB_7), cm_CB_7},
	{0, sizeof(cm_E7), cm_E7},
	{0, sizeof(cm_CB_1), cm_CB_1},
	{0, sizeof(cm_F0), cm_F0},
	{0, sizeof(cm_CB_8), cm_CB_8},
	{0, sizeof(cm_F7), cm_F7},
	{0, sizeof(cm_CB_9), cm_CB_9},
	//PANEL PAD MAPPING
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CC_1), cm_CC_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CC_2), cm_CC_2},
	{0, sizeof(cm_8E), cm_8E},
	{0, sizeof(cm_CC_3), cm_CC_3},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CC_4), cm_CC_4},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CC_6), cm_CC_6},
	{0, sizeof(cm_9E), cm_9E},
	{0, sizeof(cm_CC_5), cm_CC_5},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CC_7), cm_CC_7},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CC_4), cm_CC_4},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CC_8), cm_CC_8},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CC_9), cm_CC_9},
	{0, sizeof(cm_BE), cm_BE},
	{0, sizeof(cm_CC_3), cm_CC_3},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CC_4), cm_CC_4},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CC_10), cm_CC_10},
	{0, sizeof(cm_CE), cm_CE},
	{0, sizeof(cm_CC_5), cm_CC_5},
	{0, sizeof(cm_D0), cm_D0},
	{0, sizeof(cm_CC_11), cm_CC_11},
	{0, sizeof(cm_D7), cm_D7},
	{0, sizeof(cm_CC_4), cm_CC_4},
	//PANEL TIMING SETTING
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CE_1), cm_CE_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CE_2), cm_CE_2},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CE_3), cm_CE_3},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CE_3), cm_CE_3},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CE_4), cm_CE_4},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CE_5), cm_CE_5},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CE_6), cm_CE_6},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CE_7), cm_CE_7},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CE_8), cm_CE_8},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CE_9), cm_CE_9},
	{0, sizeof(cm_D0), cm_D0},
	{0, sizeof(cm_CE_10), cm_CE_10},
	{0, sizeof(cm_D7), cm_D7},
	{0, sizeof(cm_CE_11), cm_CE_11},
	{0, sizeof(cm_80), cm_80},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_87), cm_87},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_97), cm_97},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_A0), cm_A0},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_A7), cm_A7},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_B0), cm_B0},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_B7), cm_B7},
	{0, sizeof(cm_CF_1), cm_CF_1},
	{0, sizeof(cm_C0), cm_C0},
	{0, sizeof(cm_CF_2), cm_CF_2},
	{0, sizeof(cm_C7), cm_C7},
	{0, sizeof(cm_CF_3), cm_CF_3},
	{0, sizeof(cm_B5), cm_B5},
	{0, sizeof(cm_C5_4), cm_C5_4},
	//FOR POWER IC
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_F5_1), cm_F5_1},
	{0, sizeof(cm_90), cm_90},
	{0, sizeof(cm_C5_5), cm_C5_5},
	{0, sizeof(cm_94), cm_94},
	{0, sizeof(cm_C5_6), cm_C5_6},
	//VGL01/02
	{0, sizeof(cm_B2), cm_B2},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_B4), cm_B4},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_B6), cm_B6},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_B8), cm_B8},
	{0, sizeof(cm_F5_2), cm_F5_2},
	{0, sizeof(cm_94), cm_94},
	{0, sizeof(cm_F5_3), cm_F5_3},
	{0, sizeof(cm_BA), cm_BA},
	{0, sizeof(cm_F5_4), cm_F5_4},
	{0, sizeof(cm_B4), cm_B4},
	{0, sizeof(cm_C5_7), cm_C5_7},
	//GAMMA
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_E1_2), cm_E1_2},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_E2_2), cm_E2_2},
	{0, sizeof(cm_00), cm_00},
	{0, sizeof(cm_FF_3), cm_FF_3},
};
#endif
static int send_mipi_cmd_gen(struct mdfld_dsi_pkg_sender * sender,
				struct mipi_dsi_cmd *cmd) {
	int err = 0;

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

static int orise1283a_vid_drv_ic_init(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i;
	u8 data2[3] = {0};

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	/* panel initial settings */
	mdfld_dsi_read_mcs_lp(sender, 0xB9, data2, 3);
	if (board_proj_id == PROJ_ID_A500CG || board_proj_id == PROJ_ID_A501CG || board_proj_id == PROJ_ID_A502CG) {
		printk("[DISP] %s : A500CG series init : ", __func__);
#if !ENABLE_SHORT_PACKAGE_CMD
		printk(" Long package\n");
		for(i = 0; i < ARRAY_SIZE(a500cg_power_on_table); i++)
			send_mipi_cmd_gen(sender, &a500cg_power_on_table[i]);
#else
		printk(" Short package\n");
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_1, sizeof(cm_FF_1), MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_2, sizeof(cm_FF_2), MDFLD_DSI_SEND_PACKAGE);
		//PANEL SETTING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x64, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x64, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x56, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x15, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x50, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC1, 0x55, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x49, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x55, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x36, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		//POWER SETTING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x15, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x15, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x46, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x40, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD8, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD8, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xB0, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		//CONTROL SETTING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD0, 0x40, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xEA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xEB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xEC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xED, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xFA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		//PANEL PAD MAPPING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x3D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x15, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x20, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x08, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x6F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x6F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		//FOR POWER IC
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x11, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x11, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x50, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x66, 1, MDFLD_DSI_SEND_PACKAGE);
		//VGL01/02
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		//GAMMA
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x12, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x12, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x12, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x12, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_3, sizeof(cm_FF_3), MDFLD_DSI_SEND_PACKAGE);
#endif
	} else {
		printk("[DISP] %s : A600CG series init :", __func__);
#if !ENABLE_SHORT_PACKAGE_CMD
		printk(" Long package\n");
		for(i = 0; i < ARRAY_SIZE(a600cg_power_on_table); i++)
			send_mipi_cmd_gen(sender, &a600cg_power_on_table[i]);
#else
		printk(" Short package\n");
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_1, sizeof(cm_FF_1), MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_2, sizeof(cm_FF_2), MDFLD_DSI_SEND_PACKAGE);
		//PANEL SETTING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x64, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x64, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x56, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x16, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x50, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC1, 0x55, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x49, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC0, 0x55, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x36, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		//POWER SETTING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x15, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x15, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x46, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x40, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD8, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD8, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC4, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xB0, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		//CONTROL SETTING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD0, 0x40, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xD1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xE9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xEA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xEB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xEC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xED, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xF9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xFA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCB, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		//PANEL PAD MAPPING
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x0E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCE, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2E, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x29, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x2A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCC, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x38, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xD9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x05, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x18, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xDD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCE, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x82, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x83, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x85, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x86, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x87, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x88, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x89, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x8D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x95, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x96, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x97, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x98, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x99, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9A, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x9D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xA9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xAD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBB, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBC, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBD, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x3D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC1, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x15, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x20, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x01, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x81, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xC9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xCA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xCF, 0x08, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x6F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x6F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xFF, 1, MDFLD_DSI_SEND_PACKAGE);
		//FOR POWER IC
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x11, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x93, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x11, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x50, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0x66, 1, MDFLD_DSI_SEND_PACKAGE);
		//VGL01/02
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB2, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB3, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB7, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB8, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB9, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x94, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x02, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xBA, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xF5, 0x03, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB4, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xC5, 0xC0, 1, MDFLD_DSI_SEND_PACKAGE);
		//GAMMA
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x08, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x11, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x0B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE1, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x08, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0F, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0C, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x04, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x07, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0D, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x09, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x10, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x11, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x0B, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0xE2, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_3, sizeof(cm_FF_3), MDFLD_DSI_SEND_PACKAGE);
#endif
	}
	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(120);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}

	return 0;
}

static void
orise1283a_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	struct drm_device *dev = dsi_config->dev;
#if ENABLE_CSC_GAMMA
	struct csc_setting csc = {	.pipe = 0,
								.type = CSC_REG_SETTING,
								.enable_state = true,
								.data_len = CSC_REG_COUNT,
								.data.csc_reg_data = {
									0x400, 0x0, 0x4000000, 0x0, 0x0, 0x400}
							 };
	struct gamma_setting gamma = {	.pipe = 0,
									.type = GAMMA_REG_SETTING,
									.enable_state = true,
									.data_len = GAMMA_10_BIT_TABLE_COUNT,
									.gamma_tableX100 = {
										0x000000, 0x020202, 0x040404, 0x060606,
										0x080808, 0x0A0A0A, 0x0C0C0C, 0x0E0E0E,
										0x101010, 0x121212, 0x141414, 0x161616,
										0x181818, 0x1A1A1A, 0x1C1C1C, 0x1E1E1E,
										0x202020, 0x222222, 0x242424, 0x262626,
										0x282828, 0x2A2A2A, 0x2C2C2C, 0x2E2E2E,
										0x303030, 0x323232, 0x343434, 0x363636,
										0x383838, 0x3A3A3A, 0x3C3C3C, 0x3E3E3E,
										0x404040, 0x424242, 0x444444, 0x464646,
										0x484848, 0x4A4A4A, 0x4C4C4C, 0x4E4E4E,
										0x505050, 0x525252, 0x545454, 0x565656,
										0x585858, 0x5A5A5A, 0x5C5C5C, 0x5E5E5E,
										0x606060, 0x626262, 0x646464, 0x666666,
										0x686868, 0x6A6A6A, 0x6C6C6C, 0x6E6E6E,
										0x707070, 0x727272, 0x747474, 0x767676,
										0x787878, 0x7A7A7A, 0x7C7C7C, 0x7E7E7E,
										0x808080, 0x828282, 0x848484, 0x868686,
										0x888888, 0x8A8A8A, 0x8C8C8C, 0x8E8E8E,
										0x909090, 0x929292, 0x949494, 0x969696,
										0x989898, 0x9A9A9A, 0x9C9C9C, 0x9E9E9E,
										0xA0A0A0, 0xA2A2A2, 0xA4A4A4, 0xA6A6A6,
										0xA8A8A8, 0xAAAAAA, 0xACACAC, 0xAEAEAE,
										0xB0B0B0, 0xB2B2B2, 0xB4B4B4, 0xB6B6B6,
										0xB8B8B8, 0xBABABA, 0xBCBCBC, 0xBEBEBE,
										0xC0C0C0, 0xC2C2C2, 0xC4C4C4, 0xC6C6C6,
										0xC8C8C8, 0xCACACA, 0xCCCCCC, 0xCECECE,
										0xD0D0D0, 0xD2D2D2, 0xD4D4D4, 0xD6D6D6,
										0xD8D8D8, 0xDADADA, 0xDCDCDC, 0xDEDEDE,
										0xE0E0E0, 0xE2E2E2, 0xE4E4E4, 0xE6E6E6,
										0xE8E8E8, 0xEAEAEA, 0xECECEC, 0xEEEEEE,
										0xF0F0F0, 0xF2F2F2, 0xF4F4F4, 0xF6F6F6,
										0xF8F8F8, 0xFAFAFA, 0xFCFCFC, 0xFEFEFE,
										0x010000, 0x010000, 0x010000}
								 };

#endif
	printk("[DISP] %s\n", __func__);
	/* Reconfig lane configuration */
	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;
	dsi_config->enable_gamma_csc = ENABLE_GAMMA | ENABLE_CSC;

	hw_ctx->cck_div = 0;
	hw_ctx->pll_bypass_mode = 0;
	hw_ctx->mipi_control = 0x18;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xdcf50;
	hw_ctx->lp_rx_timeout = 0xffff;
	hw_ctx->turn_around_timeout = 0x3f;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x15;
	hw_ctx->init_count = 0x7d0;
	hw_ctx->eot_disable = 0x3;
	hw_ctx->lp_byteclk = 0x5;
	hw_ctx->clk_lane_switch_time_cnt = 0x15000a;
	hw_ctx->dphy_param = 0x150c340f;

	/* Setup video mode format */
	hw_ctx->video_mode_format = 0xf;

	/* Set up func_prg, RGB888(0x200) */
	hw_ctx->dsi_func_prg = (0x200 | dsi_config->lane_count);

	/* Setup mipi port configuration */
	hw_ctx->mipi = PASS_FROM_SPHY_TO_AFE | dsi_config->lane_config;

	orise1283a_dsi_config = dsi_config;

#if ENABLE_CSC_GAMMA
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
#endif
}

static int orise1283a_vid_detect(struct mdfld_dsi_config *dsi_config)
{
	int status;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;
	int pipe = dsi_config->pipe;

	printk("[DISP] %s\n", __func__);

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
			psb_enable_vblank(dev, pipe);
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

static int orise1283a_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	usleep_range(1000, 1200);

	/* Send TURN_ON packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_TURN_ON);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return err;
	}

	/* EN_VDD_BL*/
#if PWM_SOC_ENABLE
	pwm_enable();
#endif
//	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x06);

	queue_delayed_work(panel_reset_delay_wq, &panel_reset_delay_work, msecs_to_jiffies(5000));

	return 0;
}

static int orise1283a_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
//	struct orise1283a_vid_data *pdata = &gpio_settings_data;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	cancel_delayed_work_sync(&panel_reset_delay_work);

	/* Turn off the backlight*/
#if PWM_SOC_ENABLE
	pwm_disable();
#endif
	usleep_range(1000, 1500);

	/* Send SHUT_DOWN packet */
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
					    MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	if (err) {
		DRM_ERROR("Failed to send turn off packet\n");
		return err;
	}

	/* Send power off command*/
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	mdfld_dsi_send_mcs_short_lp(sender, 0x28, 0x00, 0, 0);
	mdfld_dsi_send_mcs_short_lp(sender, 0x10, 0x00, 0, 0);
	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}
	usleep_range(50000, 55000);
	/* Driver IC power off sequence*/
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x04);
#if 0 /* Skip put down the HW_RST pin */
	gpio_direction_output(pdata->gpio_lcd_rst, 0);
	usleep_range(120000, 121000);
#endif
	return 0;
}

static int orise1283a_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	struct orise1283a_vid_data *pdata = &gpio_settings_data;
	printk("[DISP] %s\n", __func__);

      // start the initial state
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x04);
//	usleep_range(1000, 1500);
//	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x00);

	// start power on sequence
//	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x06);
//	usleep_range(5000, 5500);

	gpio_direction_output(pdata->gpio_lcd_rst, 1);
	usleep_range(10000, 11000);
	gpio_direction_output(pdata->gpio_lcd_rst, 0);
	usleep_range(10000, 11000);
	gpio_direction_output(pdata->gpio_lcd_rst, 1);
	usleep_range(10000, 11000);

	return 0;
}


#define PWM0CLKDIV1 0x61
#define PWM0CLKDIV0 0x62

#define PWM0DUTYCYCLE 0x67
#define DUTY_VALUE_MAX 0x63

/*
#define BRI_SETTING_MIN 10
//#define BRI_SETTING_DEF 170
#define BRI_SETTING_MAX 255
*/
static int orise1283a_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	int duty_val = 0;
	int ret = 0;
	unsigned int pwm_min, pwm_max;
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
#if PWM_SOC_ENABLE
	if (board_proj_id == PROJ_ID_A600CG)
		pwm_min = 5;
	else
		pwm_min = 13;
	pwm_max = 255;

	if (level <= 0) {
		duty_val = 0;
	} else if (level > 0 && (level <= pwm_min)) {
		duty_val = pwm_min;
	} else if ((level > pwm_min) && (level <= pwm_max)) {
		duty_val = level;
	} else if (level > pwm_max)
		duty_val = pwm_max;

	pwm_configure(duty_val);
#else
	duty_val = ((DUTY_VALUE_MAX + 1) * level) / 255;
	ret = intel_scu_ipc_iowrite8(PWM0DUTYCYCLE, (duty_val > DUTY_VALUE_MAX ? DUTY_VALUE_MAX : duty_val));
	if (ret)
		DRM_ERROR("write brightness duty value faild\n");
#endif

	printk("[DISP] brightness level = %d , duty_val = %d\n", level, duty_val);

	return 0;
}

struct drm_display_mode *orise1283a_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	printk("[DISP] %s\n", __func__);
	/* RECOMMENDED PORCH SETTING
		HSA=18, HBP=48, HFP=64
		VSA=3,   VBP=14, VFP=9	 */
	mode->hdisplay = 720;
	mode->vdisplay = 1280;
	mode->hsync_start = 784;
	mode->hsync_end = 802;
	mode->htotal = 850;
	mode->vsync_start = 1289;
	mode->vsync_end = 1292;
	mode->vtotal = 1306;

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;
	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void orise1283a_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (board_proj_id == PROJ_ID_A600CG) {
		pi->width_mm = 74;
		pi->height_mm = 131;
	} else {
		pi->width_mm = 62;
		pi->height_mm = 110;
	}
}

static int orise1283a_vid_gpio_init(void)
{
	int ret;
	struct orise1283a_vid_data *pdata = &gpio_settings_data;

	printk("[DISP] %s\n", __func__);

	pdata->gpio_lcd_rst = get_gpio_by_name("DISP_RST_N");

	ret = gpio_request(pdata->gpio_lcd_rst, "DISP_RST_N");
	if (ret < 0)
		DRM_ERROR("Faild to get panel GPIO DISP_RST_N:%d\n", pdata->gpio_lcd_rst);

//	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0x01);

	return 0;
}

static int orise1283a_vid_brightness_init(void)
{
	int ret = 0;

	printk("[DISP] %s\n", __func__);
#if PWM_SOC_ENABLE
	pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG,PWMCTRL_SIZE);
	lnw_gpio_set_alt(PWM_ENABLE_GPIO, LNW_ALT_2);
#else
	ret = intel_scu_ipc_iowrite8(PWM0CLKDIV1, 0x00);
	if (!ret)
		ret = intel_scu_ipc_iowrite8(PWM0CLKDIV0, 0x25);

	if (ret)
		printk("[DISP] %s: PWM0CLKDIV set failed\n", __func__);
	else
		printk("[DISP] PWM0CLKDIV set to 0x%04x\n", 0x25);
#endif

	return ret;
}


#ifdef ORISE1283A_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(orise1283a_dsi_config);

    sscanf(buf, "%x,%x", &x0, &x1);

//	send_mipi_ret = mdfld_dsi_send_mcs_short_lp(sender,x0,x1,1,0);
	send_mipi_ret = mdfld_dsi_send_gen_short_lp(sender,x0,x1,2,0);

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
			= mdfld_dsi_get_pkg_sender(orise1283a_dsi_config);

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

DEVICE_ATTR(send_mipi_orise1283a,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_orise1283a,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *orise1283a_attrs[] = {
        &dev_attr_send_mipi_orise1283a.attr,
        &dev_attr_read_mipi_orise1283a.attr,
        NULL
};

static struct attribute_group orise1283a_attr_group = {
        .attrs = orise1283a_attrs,
        .name = "orise1283a",
};

#endif

void orise1283a_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	printk("[DISP] %s\n", __func__);

	/* Read Project ID */
	board_proj_id = Read_PROJ_ID();

	p_funcs->get_config_mode = orise1283a_vid_get_config_mode;
	p_funcs->get_panel_info = orise1283a_vid_get_panel_info;
	p_funcs->dsi_controller_init = orise1283a_vid_dsi_controller_init;
	p_funcs->detect = orise1283a_vid_detect;
	p_funcs->power_on = orise1283a_vid_power_on;
	p_funcs->drv_ic_init = orise1283a_vid_drv_ic_init;
	p_funcs->power_off = orise1283a_vid_power_off;
	p_funcs->reset = orise1283a_vid_reset;
	p_funcs->set_brightness = orise1283a_vid_set_brightness;

	ret = orise1283a_vid_gpio_init();
	if (ret)
		DRM_ERROR("Faild to request GPIO for ORISE1283A panel\n");

	ret = orise1283a_vid_brightness_init();
	if (ret)
		DRM_ERROR("Faild to initilize PWM of MSCI\n");

	printk("[DISP] HSD panel reset workqueue init!\n");
	INIT_DELAYED_WORK(&panel_reset_delay_work, orise1283a_vid_panel_reset_delay_work);
	panel_reset_delay_wq = create_workqueue("panel_reset_delay_timer");
	if (unlikely(!panel_reset_delay_wq)) {
		printk("%s : unable to create Panel reset workqueue\n", __func__);
	}

#ifdef ORISE1283A_DEBUG
    sysfs_create_group(&dev->dev->kobj, &orise1283a_attr_group);
#endif

}

static int orise1283a_vid_shutdown(struct platform_device *pdev)
{
	struct orise1283a_vid_data *pdata = &gpio_settings_data;
	printk("[DISP] %s\n", __func__);

	mdfld_dsi_dpi_set_power(encoder_lcd, 0);
//	intel_scu_ipc_iowrite8(PMIC_GPIO_BACKLIGHT_EN, 0);
//	orise1283a_vid_set_brightness(orise1283a_dsi_config, 0);
//	orise1283a_vid_power_off(orise1283a_dsi_config);
	usleep_range(50000, 55000);
	gpio_direction_output(pdata->gpio_lcd_rst, 0);
	usleep_range(120000, 121000);

	intel_scu_ipc_iowrite8(PMIC_GPIO_VEMMC2CNT, 0x04);

	return 0;
}


static int orise1283a_vid_lcd_probe(struct platform_device *pdev)
{
	printk("[DISP] %s: ORISE1283A panel detected\n", __func__);
	intel_mid_panel_register(orise1283a_vid_init);

	return 0;
}

struct platform_driver orise1283a_lcd_driver = {
	.probe	= orise1283a_vid_lcd_probe,
	.shutdown = orise1283a_vid_shutdown,
	.driver	= {
		.name	= ORISE1283A_PANEL_NAME,
		.owner	= THIS_MODULE,
	},
};
