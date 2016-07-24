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

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "displays/otm1284a_vid.h"
#include <linux/HWVersion.h>

#define ZE550ML_TM 1
#define ZE550ML_CPT 2
#define ZE550ML_TM_SR 3
#define ZE550ML_TM_MP 4

extern int Read_LCD_ID(void);
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
static int lcd_id = ZE550ML_TM_MP;


#define OTM1284A_DEBUG 1

/*
 * GPIO pin definition
 */
#define OTM1284A_BL_EN_GPIO   188
#define OTM1284A_BL_PWM_GPIO  183

#define PWMCTRL_REG 0xFF013C00
#define PWMCTRL_SIZE 0x80
#define PWM_BASE_UNIT 0x1555 //25,000Hz

#define PULL_DOWN_EN			BIT(9)
#define PULL_UP_EN			BIT(8)
#define BL_EN_REG 0xFF0C2530

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

static struct mdfld_dsi_config *otm1284a_dsi_config;

static struct mipi_dsi_cmd_orise *otm1284a_power_on_table = NULL;
static int otm1284a_power_on_table_size = 0;


/* ====Initial settings==== */
static u8 cm_FF_1[] = {0xFF, 0x12, 0x84, 0x01};
static u8 cm_FF_2[] = {0xFF, 0x12, 0x84};
static u8 cm_FF_3[] = {0xFF, 0xFF, 0xFF, 0xFF};
static u8 cm_FF_4[] = {0xFF, 0x10, 0x02};
static u8 cm_FF_5[] = {0xFF, 0x30, 0x02};
//-------------------- panel setting --------------------//
static u8 cm3_001[] = {0x00, 0x80};
static u8 cm3_002[] = {0xc0, 0x00, 0x64, 0x00, 0x0e, 0x12, 0x00, 0x64, 0x0e, 0x12};
static u8 cm3_003[] = {0x00, 0xb4};
static u8 cm3_004[] = {0xc0, 0x55};
static u8 cm3_005[] = {0x00, 0x80};
static u8 cm3_006[] = {0xc4, 0x30};
static u8 cm3_007[] = {0x00, 0x81};
static u8 cm3_008[] = {0xc4, 0x84};
static u8 cm3_009[] = {0x00, 0x8a};
static u8 cm3_010[] = {0xc4, 0x40};
static u8 cm3_011[] = {0x00, 0x81};
static u8 cm3_012[] = {0xc1, 0x55};
static u8 cm3_013[] = {0x00, 0x82};
static u8 cm3_014[] = {0xc4, 0x0a};
static u8 cm3_015[] = {0x00, 0xc6};
static u8 cm3_016[] = {0xb0, 0x03};
static u8 cm3_017[] = {0x00, 0xc2};
static u8 cm3_018[] = {0xf5, 0x40};
static u8 cm3_019[] = {0x00, 0xc3};
static u8 cm3_020[] = {0xf5, 0x85};
static u8 cm3_021[] = {0x00, 0x87};
static u8 cm3_022[] = {0xc4, 0x18};
//-------------------- power setting --------------------//
static u8 cm3_023[] = {0x00, 0xa0};
static u8 cm3_024[] = {0xc4, 0x05, 0x10, 0x06, 0x02, 0x05, 0x15, 0x10, 0x05, 0x10, 0x07, 0x02, 0x05, 0x15, 0x10};
static u8 cm3_025[] = {0x00, 0xb0};
static u8 cm3_026[] = {0xc4, 0x00, 0x00};
static u8 cm3_027[] = {0x00, 0xbb};
static u8 cm3_028[] = {0xc5, 0x8a};
static u8 cm3_029[] = {0x00, 0x91};
static u8 cm3_030[] = {0xc5, 0x16, 0x52};
static u8 cm3_031[] = {0x00, 0x00};
static u8 cm3_032[] = {0xd8, 0xa5, 0xa5};
static u8 cm3_033[] = {0x00, 0x00};
static u8 cm3_034[] = {0xd9, 0x56};
static u8 cm3_035[] = {0x00, 0xb3};
static u8 cm3_036[] = {0xc5, 0x84};
//-------------------- power IC  --------------------//
static u8 cm3_037[] = {0x00, 0x90};
static u8 cm3_038[] = {0xf5, 0x02, 0x11, 0x02, 0x15};
static u8 cm3_039[] = {0x00, 0x90};
static u8 cm3_040[] = {0xc5, 0x50};
static u8 cm3_041[] = {0x00, 0x94};
static u8 cm3_042[] = {0xc5, 0x66};
static u8 cm3_043[] = {0x00, 0xb2};
static u8 cm3_044[] = {0xf5, 0x00, 0x00};
static u8 cm3_045[] = {0x00, 0xb4};
static u8 cm3_046[] = {0xf5, 0x00, 0x00};
static u8 cm3_047[] = {0x00, 0xb6};
static u8 cm3_048[] = {0xf5, 0x00, 0x00};
static u8 cm3_049[] = {0x00, 0xb8};
static u8 cm3_050[] = {0xf5, 0x00, 0x00};
static u8 cm3_051[] = {0x00, 0x94};
static u8 cm3_052[] = {0xf5, 0x00, 0x00};
static u8 cm3_053[] = {0x00, 0xd2};
static u8 cm3_054[] = {0xf5, 0x06, 0x15};
static u8 cm3_055[] = {0x00, 0xb4};
static u8 cm3_056[] = {0xc5, 0xcc};
//-------------------- panel timing state control --------------------//
static u8 cm3_057[] = {0x00, 0x80};
static u8 cm3_058[] = {0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_059[] = {0x00, 0x90};
static u8 cm3_060[] = {0xcb, 0x00, 0x00, 0x05, 0x05, 0x00, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_061[] = {0x00, 0xa0};
static u8 cm3_062[] = {0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_063[] = {0x00, 0xb0};
static u8 cm3_064[] = {0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_065[] = {0x00, 0xc0};
static u8 cm3_066[] = {0xcb, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_067[] = {0x00, 0xd0};
static u8 cm3_068[] = {0xcb, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x05, 0x00, 0x00};
static u8 cm3_069[] = {0x00, 0xe0};
static u8 cm3_070[] = {0xcb, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x05, 0x05};
static u8 cm3_071[] = {0x00, 0xf0};
static u8 cm3_072[] = {0xcb, 0xff, 0xcf, 0x00, 0x03, 0xc0, 0xf0, 0xff, 0x0c, 0x30, 0x00, 0x0c};
//-------------------- panel pad mapping control --------------------//
static u8 cm3_073[] = {0x00, 0x80};
static u8 cm3_074[] = {0xcc, 0x0c, 0x0a, 0x10, 0x0e, 0x02, 0x04, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_075[] = {0x00, 0x90};
static u8 cm3_076[] = {0xcc, 0x00, 0x00, 0x00, 0x00, 0x06, 0x2d, 0x2e, 0x0b, 0x09, 0x0f, 0x0d, 0x01, 0x03, 0x00, 0x00};
static u8 cm3_077[] = {0x00, 0xa0};
static u8 cm3_078[] = {0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x05, 0x2d, 0x2e};
static u8 cm3_079[] = {0x00, 0xb0};
static u8 cm3_080[] = {0xcc, 0x0d, 0x0f, 0x09, 0x0b, 0x03, 0x01, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_081[] = {0x00, 0xc0};
static u8 cm3_082[] = {0xcc, 0x00, 0x00, 0x00, 0x00, 0x05, 0x2e, 0x2d, 0x0e, 0x10, 0x0a, 0x0c, 0x04, 0x02, 0x00, 0x00};
static u8 cm3_083[] = {0x00, 0xd0};
static u8 cm3_084[] = {0xcc, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x06, 0x2E, 0x2D};
//-------------------- PANEL TIMING SETTING --------------------//
static u8 cm3_085[] = {0x00, 0x80};
static u8 cm3_086[] = {0xce, 0x8b, 0x03, 0x18, 0x8a, 0x03, 0x18, 0x89, 0x03, 0x18, 0x88, 0x03, 0x18};
static u8 cm3_087[] = {0x00, 0x90};
static u8 cm3_088[] = {0xce, 0x38, 0x10, 0x18, 0x38, 0x0f, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_089[] = {0x00, 0xa0};
static u8 cm3_090[] = {0xce, 0x38, 0x07, 0x05, 0x00, 0x00, 0x18, 0x00, 0x38, 0x06, 0x05, 0x01, 0x00, 0x18, 0x00};
static u8 cm3_091[] = {0x00, 0xb0};
static u8 cm3_092[] = {0xce, 0x38, 0x05, 0x05, 0x02, 0x00, 0x18, 0x00, 0x38, 0x04, 0x05, 0x03, 0x00, 0x18, 0x00};
static u8 cm3_093[] = {0x00, 0xc0};
static u8 cm3_094[] = {0xce, 0x38, 0x03, 0x05, 0x04, 0x00, 0x18, 0x00, 0x38, 0x02, 0x05, 0x05, 0x00, 0x18, 0x00};
static u8 cm3_095[] = {0x00, 0xd0};
//static u8 cm3_096[] = {0xcf, 0x3d, 0x02, 0x15, 0x20, 0x00, 0x00, 0x01, 0x81, 0x00, 0x03, 0x08};
static u8 cm3_096[] = {0xce, 0x38, 0x01, 0x05, 0x06, 0x00, 0x18, 0x00, 0x38, 0x00, 0x05, 0x07, 0x00, 0x18, 0x00};
static u8 cm3_097[] = {0x00, 0x80};
static u8 cm3_098[] = {0xcf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_099[] = {0x00, 0x90};
static u8 cm3_100[] = {0xcf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_101[] = {0x00, 0xa0};
static u8 cm3_102[] = {0xcf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_103[] = {0x00, 0xb0};
static u8 cm3_104[] = {0xcf, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
static u8 cm3_105[] = {0x00, 0xc0};
static u8 cm3_106[] = {0xcf, 0x01, 0x01, 0x20, 0x20, 0x00, 0x00, 0x01, 0x81, 0x00, 0x03, 0x08};
static u8 cm3_107[] = {0x00, 0xb5};
static u8 cm3_108[] = {0xc5, 0x38, 0x00, 0x3f, 0x38, 0x00, 0x3f};
//------------------Gamma------------------//
static u8 cm3_109[] = {0x00, 0x00};
static u8 cm3_110[] = {0xE1,0x02,0x1d,0x2c,0x3c,0x4d,0x5c,0x5e,0x89,0x7a,0x93,0x6e,0x59,0x69,0x41,0x40,0x31,0x21,0x11,0x06,0x02};
static u8 cm3_111[] = {0x00, 0x00};
static u8 cm3_112[] = {0xE2,0x02,0x1d,0x2c,0x3c,0x4d,0x5c,0x5e,0x89,0x7a,0x93,0x6e,0x59,0x69,0x41,0x40,0x31,0x21,0x11,0x06,0x02};
static u8 cm3_113[] = {0x00, 0xa0};
static u8 cm3_114[] = {0xc1, 0x02};

static u8 cm3_115[] = {0xc5, 0x38, 0x20, 0xbf, 0x38, 0x20, 0xbf};
static u8 cm3_116[] = {0xc5, 0x46, 0x42};


static u8 cm4_001[] = {0x00, 0x80};
static u8 cm4_002[] = {0xB0, 0x01};
static u8 cm4_003[] = {0x00, 0x81};
static u8 cm4_004[] = {0xB0, 0x11};
static u8 cm4_005[] = {0x00, 0x82};
static u8 cm4_006[] = {0xB0, 0x01};
static u8 cm4_007[] = {0x00, 0x83};
static u8 cm4_008[] = {0xB0, 0x41};
static u8 cm4_009[] = {0x00, 0x84};
static u8 cm4_010[] = {0xB0, 0x06};
static u8 cm4_011[] = {0x00, 0x91};
static u8 cm4_012[] = {0xB0, 0x9A};



static u8 sleep_out[] = {0x11};
static u8 sleep_in[] = {0x10};
static u8 display_on[] = {0x29};
static u8 display_off[] = {0x28};


/* ====Power on commnad==== */
static struct mipi_dsi_cmd_orise ze550ml_TM_power_on_table[] = {
	{0, 0, sizeof(cm3_001), cm3_001, sizeof(cm3_002), cm3_002},
	{0, 0, sizeof(cm3_003), cm3_003, sizeof(cm3_004), cm3_004},
	{0, 0, sizeof(cm3_005), cm3_005, sizeof(cm3_006), cm3_006},
	{0, 0, sizeof(cm3_007), cm3_007, sizeof(cm3_008), cm3_008},
	{0, 0, sizeof(cm3_009), cm3_009, sizeof(cm3_010), cm3_010},
	{0, 0, sizeof(cm3_011), cm3_011, sizeof(cm3_012), cm3_012},
	{0, 0, sizeof(cm3_013), cm3_013, sizeof(cm3_014), cm3_014},
	{0, 0, sizeof(cm3_015), cm3_015, sizeof(cm3_016), cm3_016},
	{0, 0, sizeof(cm3_017), cm3_017, sizeof(cm3_018), cm3_018},
	{0, 0, sizeof(cm3_019), cm3_019, sizeof(cm3_020), cm3_020},
	{0, 0, sizeof(cm3_021), cm3_021, sizeof(cm3_022), cm3_022},
	{0, 0, sizeof(cm3_023), cm3_023, sizeof(cm3_024), cm3_024},
	{0, 0, sizeof(cm3_025), cm3_025, sizeof(cm3_026), cm3_026},
	{0, 0, sizeof(cm3_027), cm3_027, sizeof(cm3_028), cm3_028},
	{0, 0, sizeof(cm3_029), cm3_029, sizeof(cm3_030), cm3_030},
	{0, 0, sizeof(cm3_031), cm3_031, sizeof(cm3_032), cm3_032},
	{0, 0, sizeof(cm3_033), cm3_033, sizeof(cm3_034), cm3_034},
	{0, 0, sizeof(cm3_035), cm3_035, sizeof(cm3_036), cm3_036},
	{0, 0, sizeof(cm3_037), cm3_037, sizeof(cm3_038), cm3_038},
	{0, 0, sizeof(cm3_039), cm3_039, sizeof(cm3_040), cm3_040},
	{0, 0, sizeof(cm3_041), cm3_041, sizeof(cm3_042), cm3_042},
	{0, 0, sizeof(cm3_043), cm3_043, sizeof(cm3_044), cm3_044},
	{0, 0, sizeof(cm3_045), cm3_045, sizeof(cm3_046), cm3_046},
	{0, 0, sizeof(cm3_047), cm3_047, sizeof(cm3_048), cm3_048},
	{0, 0, sizeof(cm3_049), cm3_049, sizeof(cm3_050), cm3_050},
	{0, 0, sizeof(cm3_051), cm3_051, sizeof(cm3_052), cm3_052},
	{0, 0, sizeof(cm3_053), cm3_053, sizeof(cm3_054), cm3_054},
	{0, 0, sizeof(cm3_055), cm3_055, sizeof(cm3_056), cm3_056},
	{0, 0, sizeof(cm3_057), cm3_057, sizeof(cm3_058), cm3_058},
	{0, 0, sizeof(cm3_059), cm3_059, sizeof(cm3_060), cm3_060},
	{0, 0, sizeof(cm3_061), cm3_061, sizeof(cm3_062), cm3_062},
	{0, 0, sizeof(cm3_063), cm3_063, sizeof(cm3_064), cm3_064},
	{0, 0, sizeof(cm3_065), cm3_065, sizeof(cm3_066), cm3_066},
	{0, 0, sizeof(cm3_067), cm3_067, sizeof(cm3_068), cm3_068},
	{0, 0, sizeof(cm3_069), cm3_069, sizeof(cm3_070), cm3_070},
	{0, 0, sizeof(cm3_071), cm3_071, sizeof(cm3_072), cm3_072},
	{0, 0, sizeof(cm3_073), cm3_073, sizeof(cm3_074), cm3_074},
	{0, 0, sizeof(cm3_075), cm3_075, sizeof(cm3_076), cm3_076},
	{0, 0, sizeof(cm3_077), cm3_077, sizeof(cm3_078), cm3_078},
	{0, 0, sizeof(cm3_079), cm3_079, sizeof(cm3_080), cm3_080},
	{0, 0, sizeof(cm3_081), cm3_081, sizeof(cm3_082), cm3_082},
	{0, 0, sizeof(cm3_083), cm3_083, sizeof(cm3_084), cm3_084},
	{0, 0, sizeof(cm3_085), cm3_085, sizeof(cm3_086), cm3_086},
	{0, 0, sizeof(cm3_087), cm3_087, sizeof(cm3_088), cm3_088},
	{0, 0, sizeof(cm3_089), cm3_089, sizeof(cm3_090), cm3_090},
	{0, 0, sizeof(cm3_091), cm3_091, sizeof(cm3_092), cm3_092},
	{0, 0, sizeof(cm3_093), cm3_093, sizeof(cm3_094), cm3_094},
	{0, 0, sizeof(cm3_095), cm3_095, sizeof(cm3_096), cm3_096},
	{0, 0, sizeof(cm3_097), cm3_097, sizeof(cm3_098), cm3_098},
	{0, 0, sizeof(cm3_099), cm3_099, sizeof(cm3_100), cm3_100},
	{0, 0, sizeof(cm3_101), cm3_101, sizeof(cm3_102), cm3_102},
	{0, 0, sizeof(cm3_103), cm3_103, sizeof(cm3_104), cm3_104},
	{0, 0, sizeof(cm3_105), cm3_105, sizeof(cm3_106), cm3_106},
	{0, 0, sizeof(cm3_107), cm3_107, sizeof(cm3_108), cm3_108},
	{1, 0, sizeof(cm3_109), cm3_109, sizeof(cm3_110), cm3_110},
	{1, 0, sizeof(cm3_111), cm3_111, sizeof(cm3_112), cm3_112},
	{0, 0, sizeof(cm3_113), cm3_113, sizeof(cm3_114), cm3_114},
};

static struct mipi_dsi_cmd_orise ze550ml_TLPX_power_on_table[] = {
	{0, 0, sizeof(cm4_001), cm4_001, sizeof(cm4_002), cm4_002},
	{0, 0, sizeof(cm4_003), cm4_003, sizeof(cm4_004), cm4_004},
	{0, 0, sizeof(cm4_005), cm4_005, sizeof(cm4_006), cm4_006},
	{0, 0, sizeof(cm4_007), cm4_007, sizeof(cm4_008), cm4_008},
	{0, 0, sizeof(cm4_009), cm4_009, sizeof(cm4_010), cm4_010},
	{0, 0, sizeof(cm4_011), cm4_011, sizeof(cm4_012), cm4_012},
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

struct delayed_work orise1284a_panel_reset_delay_work;
struct workqueue_struct *orise1284a_panel_reset_delay_wq;
static int orise1284a_vid_drv_ic_reset_workaround(struct mdfld_dsi_config *dsi_config) {

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i;

	sender->status = MDFLD_DSI_PKG_SENDER_FREE;

	mdfld_dsi_send_mcs_short_hs(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(5);
	mdfld_dsi_send_mcs_short_hs(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_mcs_short_hs(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, 0xFF, 0x12, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, 0xFF, 0x84, 1, MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_mcs_short_hs(sender, 0x00, 0x90, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, 0xB6, 0xB6, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, 0xB3, 0x06, 1, MDFLD_DSI_SEND_PACKAGE);

	mdfld_dsi_send_mcs_short_hs(sender, 0x00, 0x80, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, 0xFF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, 0xFF, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);

	if (sender->status == MDFLD_DSI_CONTROL_ABNORMAL) {
		printk("[DISP] %s MDFLD_DSI_CONTROL_ABNORMAL !!\n", __func__);
		return -EIO;
	}

	return 0;
}

static void orise1284a_vid_panel_reset_delay_work(struct work_struct *work)
{
//	printk("[DEBUG] %s\n", __func__);
	orise1284a_vid_drv_ic_reset_workaround(panel_reset_dsi_config);
	queue_delayed_work(orise1284a_panel_reset_delay_wq, &orise1284a_panel_reset_delay_work, msecs_to_jiffies(5000));
}


static int otm1284a_vid_drv_ic_init(struct mdfld_dsi_config *dsi_config){

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

	if(lcd_id == ZE550ML_TM_MP) {
		for(i = 0; i < otm1284a_power_on_table_size; i++)
			send_mipi_cmd_orise(sender, &otm1284a_power_on_table[i]);
	} else if (lcd_id == ZE550ML_TM) {
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0xB5, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm3_115, sizeof(cm3_115), MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x00, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_3, sizeof(cm_FF_3), MDFLD_DSI_SEND_PACKAGE);
	} else if (lcd_id == ZE550ML_CPT) {
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x91, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm3_116, sizeof(cm3_116), MDFLD_DSI_SEND_PACKAGE);
		for(i = 0; i < otm1284a_power_on_table_size; i++)
			send_mipi_cmd_orise(sender, &otm1284a_power_on_table[i]);
	} else if (lcd_id == ZE550ML_TM_SR) {
		mdfld_dsi_send_mcs_short_lp(sender, 0x00, 0x92, 1, MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_gen_long_lp(sender, cm_FF_5, sizeof(cm_FF_5), MDFLD_DSI_SEND_PACKAGE);

		for(i = 0; i < otm1284a_power_on_table_size; i++)
			send_mipi_cmd_orise(sender, &otm1284a_power_on_table[i]);

//		mdfld_dsi_send_gen_long_lp(sender, cm_FF_3, sizeof(cm_FF_3), MDFLD_DSI_SEND_PACKAGE);
	}
	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(200);
	mdfld_dsi_send_mcs_short_lp(sender, 0x29, 0, 0, MDFLD_DSI_SEND_PACKAGE);

	return 0;
}

static void
otm1284a_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
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
	hw_ctx->mipi_control = 0x38;
	hw_ctx->intr_en = 0xFFFFFFFF;
	hw_ctx->hs_tx_timeout = 0xFFFFFF;
	hw_ctx->lp_rx_timeout = 0xFFFFFF;
	hw_ctx->device_reset_timer = 0xFFFF;
	hw_ctx->turn_around_timeout = 0xFFFF;
	if (lcd_id == ZE550ML_TM || lcd_id == ZE550ML_TM_SR || lcd_id == ZE550ML_TM_MP){
		hw_ctx->high_low_switch_count = 0x27;
		hw_ctx->clk_lane_switch_time_cnt = 0x280011;
		hw_ctx->lp_byteclk = 0x6;
		hw_ctx->dphy_param = 0x310f330d;
	} else {
		hw_ctx->high_low_switch_count = 0x22;
		hw_ctx->clk_lane_switch_time_cnt = 0x23000f;
		hw_ctx->lp_byteclk = 0x5;
		hw_ctx->dphy_param = 0x2a0d2c0b;
	}
	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0x7D0;
	hw_ctx->dsi_func_prg = (RGB_888_FMT << FMT_DPI_POS) |
		dsi_config->lane_count;
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		BANDGAP_CHICKEN_BIT;
	hw_ctx->video_mode_format = 0xF;

	otm1284a_dsi_config = dsi_config;

	/* Panel initial settings assigned */
	if (lcd_id == ZE550ML_TM_SR) {
		printk("[DISP] Load TM SR panel initial settings.\n");
		otm1284a_power_on_table = ze550ml_TM_power_on_table;
		otm1284a_power_on_table_size = ARRAY_SIZE(ze550ml_TM_power_on_table);
	} else if (lcd_id == ZE550ML_TM_MP || lcd_id == ZE550ML_CPT) {
		printk("[DISP] Load ZE550ML MP panel initial settings.\n");
		otm1284a_power_on_table = ze550ml_TLPX_power_on_table;
		otm1284a_power_on_table_size = ARRAY_SIZE(ze550ml_TLPX_power_on_table);
	}
}

static int otm1284a_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	usleep_range(1000, 1100);
	queue_delayed_work(orise1284a_panel_reset_delay_wq, &orise1284a_panel_reset_delay_work, msecs_to_jiffies(5000));
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


static int otm1284a_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	cancel_delayed_work_sync(&orise1284a_panel_reset_delay_work);
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

static int otm1284a_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	printk("[DISP] %s\n", __func__);

	/* Open 2V9 power */
	__vpro3_power_ctrl(true);
	usleep_range(10000, 10100);
/* postpone to drv_ic_init
	gpio_set_value_cansleep(panel_reset_gpio, 0);
	usleep_range(10000, 10100);
	gpio_set_value_cansleep(panel_reset_gpio, 1);
	usleep_range(10000, 10100);
*/
	return 0;
}


static int otm1284a_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	u32 reg_level;
	union pwmctrl_reg pwmctrl;
	static void __iomem *bl_en_mmio;

	/* Re-assign the minimum brightness value to 15 */
	if (level > 0 && level <= 15)
		level = 15;

	reg_level = ~level & 0xFF;
	pwmctrl.part.pwmswupdate = 0x1;
	pwmctrl.part.pwmbu = PWM_BASE_UNIT;
	pwmctrl.part.pwmtd = reg_level;

	if (!pwmctrl_mmio)
		pwmctrl_mmio = ioremap_nocache(PWMCTRL_REG, 4);
	if (!bl_en_mmio)
		bl_en_mmio = ioremap_nocache(BL_EN_REG, 4);

	if (pwmctrl_mmio) {
		if (level) {
			if (!gpio_get_value(backlight_en_gpio)) {
				writel((readl(bl_en_mmio) | PULL_UP_EN) & (~PULL_DOWN_EN), bl_en_mmio);
				pmu_set_pwm(PCI_D0);
				lnw_gpio_set_alt(backlight_pwm_gpio, 1);
				gpio_set_value_cansleep(backlight_en_gpio, 1);
			}

			pwmctrl.part.pwmenable = 1;
			writel(pwmctrl.full, pwmctrl_mmio);
		} else if (gpio_get_value(backlight_en_gpio)) {
			pwmctrl.part.pwmenable = 0;
			writel(pwmctrl.full, pwmctrl_mmio);
			gpio_set_value_cansleep(backlight_pwm_gpio, 0);
			lnw_gpio_set_alt(backlight_pwm_gpio, 0);
			usleep_range(10000, 10100);
			gpio_set_value_cansleep(backlight_en_gpio, 0);
			writel((readl(bl_en_mmio) | PULL_DOWN_EN) & (~PULL_UP_EN), bl_en_mmio);
			pmu_set_pwm(PCI_D3hot);
		}
	} else {
		DRM_ERROR("Cannot map pwmctrl\n");
	}

	printk("[DISP] brightness level = %d\n", level);

	return 0;
}

struct drm_display_mode *otm1284a_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	printk("[DISP] %s\n", __func__);
	if (lcd_id == ZE550ML_TM || lcd_id == ZE550ML_TM_SR || lcd_id == ZE550ML_TM_MP)  {
		/* RECOMMENDED PORCH SETTING
		HSA=12, HBP=64, HFP=64
		VSA=5, VBP=13, VFP=10	 Orise*/
		mode->hdisplay = 720;
		mode->hsync_start = mode->hdisplay + 64;
		mode->hsync_end = mode->hsync_start + 12;
		mode->htotal = mode->hsync_end + 64;

		mode->vdisplay = 1280;
		mode->vsync_start = mode->vdisplay + 10;
		mode->vsync_end = mode->vsync_start + 5;
		mode->vtotal = mode->vsync_end + 13;
	} else {	//TM
		/* RECOMMENDED PORCH SETTING
		HSA=2, HBP=42, HFP=44
		VSA=2, VBP=14, VFP=16	 Orise*/
		mode->hdisplay = 720;
		mode->hsync_start = mode->hdisplay + 44;
		mode->hsync_end = mode->hsync_start + 2;
		mode->htotal = mode->hsync_end + 42;

		mode->vdisplay = 1280;
		mode->vsync_start = mode->vdisplay + 16;
		mode->vsync_end = mode->vsync_start + 2;
		mode->vtotal = mode->vsync_end + 14;
	}

	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void otm1284a_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	pi->width_mm = 68;
	pi->height_mm = 121;
}

static int otm1284a_vid_detect(struct mdfld_dsi_config *dsi_config)
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
	backlight_en_gpio = OTM1284A_BL_EN_GPIO;
	if (gpio_request(backlight_en_gpio, "backlight_en")) {
		DRM_ERROR("Faild to request backlight enable gpio\n");
		return -EINVAL;
	}

	backlight_pwm_gpio = OTM1284A_BL_PWM_GPIO;
	if (gpio_request(backlight_pwm_gpio, "backlight_pwm")) {
		DRM_ERROR("Faild to request backlight PWM gpio\n");
		return -EINVAL;
	}

	/* Initializing pwm for being able to adjust backlight when just opening the phone. */
	pmu_set_pwm(PCI_D0);
	lnw_gpio_set_alt(backlight_pwm_gpio, 1);
#endif

	dsi_config->dsi_hw_context.panel_on = true;

	return MDFLD_DSI_PANEL_CONNECTED;
}


#ifdef OTM1284A_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(otm1284a_dsi_config);

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
			= mdfld_dsi_get_pkg_sender(otm1284a_dsi_config);

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

DEVICE_ATTR(send_mipi_otm1284a,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_otm1284a,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *otm1284a_attrs[] = {
        &dev_attr_send_mipi_otm1284a.attr,
        &dev_attr_read_mipi_otm1284a.attr,
        NULL
};

static struct attribute_group otm1284a_attr_group = {
        .attrs = otm1284a_attrs,
        .name = "otm1284a",
};

#endif

void otm1284a_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	printk("[DISP] %s\n", __func__);

	p_funcs->get_config_mode = otm1284a_vid_get_config_mode;
	p_funcs->get_panel_info = otm1284a_vid_get_panel_info;
	p_funcs->dsi_controller_init = otm1284a_vid_dsi_controller_init;
	p_funcs->detect = otm1284a_vid_detect;
	p_funcs->power_on = otm1284a_vid_power_on;
	p_funcs->drv_ic_init = otm1284a_vid_drv_ic_init;
	p_funcs->power_off = otm1284a_vid_power_off;
	p_funcs->reset = otm1284a_vid_reset;
	p_funcs->set_brightness = otm1284a_vid_set_brightness;

	printk("[DISP] Orise reset workqueue init!\n");
	INIT_DELAYED_WORK(&orise1284a_panel_reset_delay_work, orise1284a_vid_panel_reset_delay_work);
	orise1284a_panel_reset_delay_wq = create_workqueue("orise1284a_panel_reset_delay_timer");
	if (unlikely(!orise1284a_panel_reset_delay_wq)) {
		printk("%s : unable to create Panel reset workqueue\n", __func__);
	}
#ifdef OTM1284A_DEBUG
	sysfs_create_group(&dev->dev->kobj, &otm1284a_attr_group);
#endif

	if (Read_LCD_ID() == ZE550ML_LCD_ID_OTM_TM) {
		switch(Read_HW_ID()) {
			case HW_ID_SR1:
			case HW_ID_SR2:
				lcd_id = ZE550ML_TM_SR;
				break;
			case HW_ID_ER:
			case HW_ID_ER1_1:
			case HW_ID_ER1_2:
				lcd_id = ZE550ML_TM;
				break;
			default:
				lcd_id = ZE550ML_TM_MP;
				break;
		}
	} else {
		lcd_id = ZE550ML_CPT;
	}
	printk("[DISP] %s : Panel ID = %d, ", __func__, lcd_id);

	switch(lcd_id) {
		case 1:
			printk("ZE550ML_TM registered.\n");
			break;
		case 2:
			printk("ZE550ML_CPT registered.\n");
			break;
		case 3:
			printk("ZE550ML_TM SR registered.\n");
			break;
		case 4:
			printk("ZE550ML_TM MP registered.\n");
			break;
	}
}
