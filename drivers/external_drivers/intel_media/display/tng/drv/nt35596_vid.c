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
#include <linux/i2c/rt4532.h>

#include "mdfld_dsi_dpi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "displays/nt35596_vid.h"
#include <linux/HWVersion.h>
#include "psb_drv.h"

extern int Read_HW_ID(void);
extern int Read_LCD_ID(void);
extern int Read_PROJ_ID(void);

#define NT35596_DEBUG 1

/*
 * GPIO pin definition
 */
#define NT35596_BL_EN_GPIO   188
#define NT35596_BL_PWM_GPIO  183

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

static struct mdfld_dsi_config *nt35596_dsi_config;

static struct mipi_dsi_cmd *nt35596_power_on_table = NULL;
static int nt35596_power_on_table_size = 0;


/* ====Initial settings==== */
static u8 cm1_01[] = {0xFF, 0x05};
static u8 cm1_02[] = {0xFB, 0x01};
static u8 cm1_03[] = {0xC5, 0x31};
static u8 cm1_04[] = {0xFF, 0x01};
static u8 cm1_05[] = {0xFB, 0x01};
static u8 cm1_06[] = {0x00, 0x01};
static u8 cm1_07[] = {0x01, 0x55};
static u8 cm1_08[] = {0x02, 0x40};
static u8 cm1_09[] = {0x05, 0xA0};
static u8 cm1_010[] = {0x06, 0x0A};
static u8 cm1_011[] = {0x07, 0x14};
static u8 cm1_012[] = {0x08, 0x0C};
static u8 cm1_013[] = {0x0B, 0x91};
static u8 cm1_014[] = {0x0C, 0x91};
static u8 cm1_015[] = {0x0E, 0xAB};
static u8 cm1_016[] = {0x0F, 0xA4};
static u8 cm1_017[] = {0x11, 0x10};
static u8 cm1_018[] = {0x13, 0x43};
static u8 cm1_019[] = {0x14, 0x14};
static u8 cm1_020[] = {0x15, 0x13};
static u8 cm1_021[] = {0x16, 0x13};
static u8 cm1_022[] = {0x18, 0x00};
static u8 cm1_023[] = {0x19, 0x77};
static u8 cm1_024[] = {0x1A, 0x55};
static u8 cm1_025[] = {0x1B, 0x13};
static u8 cm1_026[] = {0x1C, 0x00};
static u8 cm1_027[] = {0x1D, 0x00};
static u8 cm1_028[] = {0x1E, 0x13};
static u8 cm1_029[] = {0x1F, 0x00};
static u8 cm1_030[] = {0x35, 0x00};
static u8 cm1_031[] = {0x66, 0x00};
static u8 cm1_032[] = {0x58, 0x81};
static u8 cm1_033[] = {0x59, 0x01};
static u8 cm1_034[] = {0x5A, 0x01};
static u8 cm1_035[] = {0x5B, 0x01};
static u8 cm1_036[] = {0x5C, 0x81};
static u8 cm1_037[] = {0x5D, 0x81};
static u8 cm1_038[] = {0x5E, 0x01};
static u8 cm1_039[] = {0x5F, 0x01};
static u8 cm1_040[] = {0x6D, 0x22};
static u8 cm1_041[] = {0x72, 0x31};
static u8 cm1_042[] = {0xFF, 0x05};
static u8 cm1_043[] = {0xFB, 0x01};
static u8 cm1_044[] = {0x00, 0x00};
static u8 cm1_045[] = {0x01, 0x00};
static u8 cm1_046[] = {0x02, 0x03};
static u8 cm1_047[] = {0x03, 0x04};
static u8 cm1_048[] = {0x04, 0x00};
static u8 cm1_049[] = {0x05, 0x11};
static u8 cm1_050[] = {0x06, 0x0C};
static u8 cm1_051[] = {0x07, 0x0B};
static u8 cm1_052[] = {0x08, 0x01};
static u8 cm1_053[] = {0x09, 0x00};
static u8 cm1_054[] = {0x0A, 0x18};
static u8 cm1_055[] = {0x0B, 0x16};
static u8 cm1_056[] = {0x0C, 0x14};
static u8 cm1_057[] = {0x0D, 0x17};
static u8 cm1_058[] = {0x0E, 0x15};
static u8 cm1_059[] = {0x0F, 0x13};
static u8 cm1_060[] = {0x10, 0x00};
static u8 cm1_061[] = {0x11, 0x00};
static u8 cm1_062[] = {0x12, 0x03};
static u8 cm1_063[] = {0x13, 0x04};
static u8 cm1_064[] = {0x14, 0x00};
static u8 cm1_065[] = {0x15, 0x11};
static u8 cm1_066[] = {0x16, 0x0C};
static u8 cm1_067[] = {0x17, 0x0B};
static u8 cm1_068[] = {0x18, 0x01};
static u8 cm1_069[] = {0x19, 0x00};
static u8 cm1_070[] = {0x1A, 0x18};
static u8 cm1_071[] = {0x1B, 0x16};
static u8 cm1_072[] = {0x1C, 0x14};
static u8 cm1_073[] = {0x1D, 0x17};
static u8 cm1_074[] = {0x1E, 0x15};
static u8 cm1_075[] = {0x1F, 0x13};
static u8 cm1_076[] = {0x20, 0x00};
static u8 cm1_077[] = {0x21, 0x02};
static u8 cm1_078[] = {0x22, 0x09};
static u8 cm1_079[] = {0x23, 0x67};
static u8 cm1_080[] = {0x24, 0x06};
static u8 cm1_081[] = {0x25, 0x2D};
static u8 cm1_082[] = {0x29, 0x58};
static u8 cm1_083[] = {0x2A, 0x11};
static u8 cm1_084[] = {0x2B, 0x04};
static u8 cm1_085[] = {0x2F, 0x02};
static u8 cm1_086[] = {0x30, 0x01};
static u8 cm1_087[] = {0x31, 0x49};
static u8 cm1_088[] = {0x32, 0x23};
static u8 cm1_089[] = {0x33, 0x01};
static u8 cm1_090[] = {0x34, 0x03};
static u8 cm1_091[] = {0x35, 0x6B};
static u8 cm1_092[] = {0x36, 0x00};
static u8 cm1_093[] = {0x37, 0x1D};
static u8 cm1_094[] = {0x38, 0x00};
static u8 cm1_095[] = {0x5D, 0x23};
static u8 cm1_096[] = {0x61, 0x15};
static u8 cm1_097[] = {0x65, 0x00};
static u8 cm1_098[] = {0x69, 0x04};
static u8 cm1_099[] = {0x6C, 0x51};
static u8 cm1_0100[] = {0x7A, 0x02};
static u8 cm1_0101[] = {0x7B, 0x80};
static u8 cm1_0102[] = {0x7C, 0xD8};
static u8 cm1_0103[] = {0x7D, 0x10};
static u8 cm1_0104[] = {0x7E, 0x06};
static u8 cm1_0105[] = {0x7F, 0x1B};
static u8 cm1_0106[] = {0x81, 0x06};
static u8 cm1_0107[] = {0x82, 0x02};
static u8 cm1_0108[] = {0x8A, 0x33};
static u8 cm1_0109[] = {0x93, 0x06};
static u8 cm1_0110[] = {0x94, 0x06};
static u8 cm1_0111[] = {0x9B, 0x0F};
static u8 cm1_0112[] = {0xA4, 0x0F};
static u8 cm1_0113[] = {0xC5, 0x31};
static u8 cm1_0114[] = {0xE7, 0x80};
static u8 cm1_0115[] = {0xFF, 0x01};
static u8 cm1_0116[] = {0xFB, 0x01};
static u8 cm1_0117[] = {0x75, 0x00};
static u8 cm1_0118[] = {0x76, 0x0F};
static u8 cm1_0119[] = {0x77, 0x00};
static u8 cm1_0120[] = {0x78, 0x21};
static u8 cm1_0121[] = {0x79, 0x00};
static u8 cm1_0122[] = {0x7A, 0x44};
static u8 cm1_0123[] = {0x7B, 0x00};
static u8 cm1_0124[] = {0x7C, 0x65};
static u8 cm1_0125[] = {0x7D, 0x00};
static u8 cm1_0126[] = {0x7E, 0x7F};
static u8 cm1_0127[] = {0x7F, 0x00};
static u8 cm1_0128[] = {0x80, 0x95};
static u8 cm1_0129[] = {0x81, 0x00};
static u8 cm1_0130[] = {0x82, 0xA9};
static u8 cm1_0131[] = {0x83, 0x00};
static u8 cm1_0132[] = {0x84, 0xBB};
static u8 cm1_0133[] = {0x85, 0x00};
static u8 cm1_0134[] = {0x86, 0xCA};
static u8 cm1_0135[] = {0x87, 0x00};
static u8 cm1_0136[] = {0x88, 0xFF};
static u8 cm1_0137[] = {0x89, 0x01};
static u8 cm1_0138[] = {0x8A, 0x2A};
static u8 cm1_0139[] = {0x8B, 0x01};
static u8 cm1_0140[] = {0x8C, 0x6B};
static u8 cm1_0141[] = {0x8D, 0x01};
static u8 cm1_0142[] = {0x8E, 0x9E};
static u8 cm1_0143[] = {0x8F, 0x01};
static u8 cm1_0144[] = {0x90, 0xEF};
static u8 cm1_0145[] = {0x91, 0x02};
static u8 cm1_0146[] = {0x92, 0x2D};
static u8 cm1_0147[] = {0x93, 0x02};
static u8 cm1_0148[] = {0x94, 0x2F};
static u8 cm1_0149[] = {0x95, 0x02};
static u8 cm1_0150[] = {0x96, 0x68};
static u8 cm1_0151[] = {0x97, 0x02};
static u8 cm1_0152[] = {0x98, 0xA2};
static u8 cm1_0153[] = {0x99, 0x02};
static u8 cm1_0154[] = {0x9A, 0xC8};
static u8 cm1_0155[] = {0x9B, 0x02};
static u8 cm1_0156[] = {0x9C, 0xF9};
static u8 cm1_0157[] = {0x9D, 0x03};
static u8 cm1_0158[] = {0x9E, 0x1C};
static u8 cm1_0159[] = {0x9F, 0x03};
static u8 cm1_0160[] = {0xA0, 0x46};
static u8 cm1_0161[] = {0xA2, 0x03};
static u8 cm1_0162[] = {0xA3, 0x53};
static u8 cm1_0163[] = {0xA4, 0x03};
static u8 cm1_0164[] = {0xA5, 0x61};
static u8 cm1_0165[] = {0xA6, 0x03};
static u8 cm1_0166[] = {0xA7, 0x71};
static u8 cm1_0167[] = {0xA9, 0x03};
static u8 cm1_0168[] = {0xAA, 0x81};
static u8 cm1_0169[] = {0xAB, 0x03};
static u8 cm1_0170[] = {0xAC, 0x94};
static u8 cm1_0171[] = {0xAD, 0x03};
static u8 cm1_0172[] = {0xAE, 0xA8};
static u8 cm1_0173[] = {0xAF, 0x03};
static u8 cm1_0174[] = {0xB0, 0xB9};
static u8 cm1_0175[] = {0xB1, 0x03};
static u8 cm1_0176[] = {0xB2, 0xC2};
static u8 cm1_0177[] = {0xB3, 0x00};
static u8 cm1_0178[] = {0xB4, 0x0F};
static u8 cm1_0179[] = {0xB5, 0x00};
static u8 cm1_0180[] = {0xB6, 0x21};
static u8 cm1_0181[] = {0xB7, 0x00};
static u8 cm1_0182[] = {0xB8, 0x44};
static u8 cm1_0183[] = {0xB9, 0x00};
static u8 cm1_0184[] = {0xBA, 0x65};
static u8 cm1_0185[] = {0xBB, 0x00};
static u8 cm1_0186[] = {0xBC, 0x7F};
static u8 cm1_0187[] = {0xBD, 0x00};
static u8 cm1_0188[] = {0xBE, 0x95};
static u8 cm1_0189[] = {0xBF, 0x00};
static u8 cm1_0190[] = {0xC0, 0xA9};
static u8 cm1_0191[] = {0xC1, 0x00};
static u8 cm1_0192[] = {0xC2, 0xBB};
static u8 cm1_0193[] = {0xC3, 0x00};
static u8 cm1_0194[] = {0xC4, 0xCA};
static u8 cm1_0195[] = {0xC5, 0x00};
static u8 cm1_0196[] = {0xC6, 0xFF};
static u8 cm1_0197[] = {0xC7, 0x01};
static u8 cm1_0198[] = {0xC8, 0x2A};
static u8 cm1_0199[] = {0xC9, 0x01};
static u8 cm1_0200[] = {0xCA, 0x6B};
static u8 cm1_0201[] = {0xCB, 0x01};
static u8 cm1_0202[] = {0xCC, 0x9E};
static u8 cm1_0203[] = {0xCD, 0x01};
static u8 cm1_0204[] = {0xCE, 0xEF};
static u8 cm1_0205[] = {0xCF, 0x02};
static u8 cm1_0206[] = {0xD0, 0x2D};
static u8 cm1_0207[] = {0xD1, 0x02};
static u8 cm1_0208[] = {0xD2, 0x2F};
static u8 cm1_0209[] = {0xD3, 0x02};
static u8 cm1_0210[] = {0xD4, 0x68};
static u8 cm1_0211[] = {0xD5, 0x02};
static u8 cm1_0212[] = {0xD6, 0xA2};
static u8 cm1_0213[] = {0xD7, 0x02};
static u8 cm1_0214[] = {0xD8, 0xC8};
static u8 cm1_0215[] = {0xD9, 0x02};
static u8 cm1_0216[] = {0xDA, 0xF9};
static u8 cm1_0217[] = {0xDB, 0x03};
static u8 cm1_0218[] = {0xDC, 0x1C};
static u8 cm1_0219[] = {0xDD, 0x03};
static u8 cm1_0220[] = {0xDE, 0x46};
static u8 cm1_0221[] = {0xDF, 0x03};
static u8 cm1_0222[] = {0xE0, 0x53};
static u8 cm1_0223[] = {0xE1, 0x03};
static u8 cm1_0224[] = {0xE2, 0x61};
static u8 cm1_0225[] = {0xE3, 0x03};
static u8 cm1_0226[] = {0xE4, 0x71};
static u8 cm1_0227[] = {0xE5, 0x03};
static u8 cm1_0228[] = {0xE6, 0x81};
static u8 cm1_0229[] = {0xE7, 0x03};
static u8 cm1_0230[] = {0xE8, 0x94};
static u8 cm1_0231[] = {0xE9, 0x03};
static u8 cm1_0232[] = {0xEA, 0xA8};
static u8 cm1_0233[] = {0xEB, 0x03};
static u8 cm1_0234[] = {0xEC, 0xB9};
static u8 cm1_0235[] = {0xED, 0x03};
static u8 cm1_0236[] = {0xEE, 0xC2};
static u8 cm1_0237[] = {0xEF, 0x00};
static u8 cm1_0238[] = {0xF0, 0x0F};
static u8 cm1_0239[] = {0xF1, 0x00};
static u8 cm1_0240[] = {0xF2, 0x21};
static u8 cm1_0241[] = {0xF3, 0x00};
static u8 cm1_0242[] = {0xF4, 0x44};
static u8 cm1_0243[] = {0xF5, 0x00};
static u8 cm1_0244[] = {0xF6, 0x65};
static u8 cm1_0245[] = {0xF7, 0x00};
static u8 cm1_0246[] = {0xF8, 0x7F};
static u8 cm1_0247[] = {0xF9, 0x00};
static u8 cm1_0248[] = {0xFA, 0x95};
static u8 cm1_0249[] = {0xFF, 0x00};
static u8 cm1_0250[] = {0xFF, 0x02};
static u8 cm1_0251[] = {0xFB, 0x01};
static u8 cm1_0252[] = {0x00, 0x00};
static u8 cm1_0253[] = {0x01, 0xA9};
static u8 cm1_0254[] = {0x02, 0x00};
static u8 cm1_0255[] = {0x03, 0xBB};
static u8 cm1_0256[] = {0x04, 0x00};
static u8 cm1_0257[] = {0x05, 0xCA};
static u8 cm1_0258[] = {0x06, 0x00};
static u8 cm1_0259[] = {0x07, 0xFF};
static u8 cm1_0260[] = {0x08, 0x01};
static u8 cm1_0261[] = {0x09, 0x2A};
static u8 cm1_0262[] = {0x0A, 0x01};
static u8 cm1_0263[] = {0x0B, 0x6B};
static u8 cm1_0264[] = {0x0C, 0x01};
static u8 cm1_0265[] = {0x0D, 0x9E};
static u8 cm1_0266[] = {0x0E, 0x01};
static u8 cm1_0267[] = {0x0F, 0xEF};
static u8 cm1_0268[] = {0x10, 0x02};
static u8 cm1_0269[] = {0x11, 0x2D};
static u8 cm1_0270[] = {0x12, 0x02};
static u8 cm1_0271[] = {0x13, 0x2F};
static u8 cm1_0272[] = {0x14, 0x02};
static u8 cm1_0273[] = {0x15, 0x68};
static u8 cm1_0274[] = {0x16, 0x02};
static u8 cm1_0275[] = {0x17, 0xA2};
static u8 cm1_0276[] = {0x18, 0x02};
static u8 cm1_0277[] = {0x19, 0xC8};
static u8 cm1_0278[] = {0x1A, 0x02};
static u8 cm1_0279[] = {0x1B, 0xF9};
static u8 cm1_0280[] = {0x1C, 0x03};
static u8 cm1_0281[] = {0x1D, 0x1C};
static u8 cm1_0282[] = {0x1E, 0x03};
static u8 cm1_0283[] = {0x1F, 0x46};
static u8 cm1_0284[] = {0x20, 0x03};
static u8 cm1_0285[] = {0x21, 0x53};
static u8 cm1_0286[] = {0x22, 0x03};
static u8 cm1_0287[] = {0x23, 0x61};
static u8 cm1_0288[] = {0x24, 0x03};
static u8 cm1_0289[] = {0x25, 0x71};
static u8 cm1_0290[] = {0x26, 0x03};
static u8 cm1_0291[] = {0x27, 0x81};
static u8 cm1_0292[] = {0x28, 0x03};
static u8 cm1_0293[] = {0x29, 0x94};
static u8 cm1_0294[] = {0x2A, 0x03};
static u8 cm1_0295[] = {0x2B, 0xA8};
static u8 cm1_0296[] = {0x2D, 0x03};
static u8 cm1_0297[] = {0x2F, 0xB9};
static u8 cm1_0298[] = {0x30, 0x03};
static u8 cm1_0299[] = {0x31, 0xC2};
static u8 cm1_0300[] = {0x32, 0x00};
static u8 cm1_0301[] = {0x33, 0x0F};
static u8 cm1_0302[] = {0x34, 0x00};
static u8 cm1_0303[] = {0x35, 0x21};
static u8 cm1_0304[] = {0x36, 0x00};
static u8 cm1_0305[] = {0x37, 0x44};
static u8 cm1_0306[] = {0x38, 0x00};
static u8 cm1_0307[] = {0x39, 0x65};
static u8 cm1_0308[] = {0x3A, 0x00};
static u8 cm1_0309[] = {0x3B, 0x7F};
static u8 cm1_0310[] = {0x3D, 0x00};
static u8 cm1_0311[] = {0x3F, 0x95};
static u8 cm1_0312[] = {0x40, 0x00};
static u8 cm1_0313[] = {0x41, 0xA9};
static u8 cm1_0314[] = {0x42, 0x00};
static u8 cm1_0315[] = {0x43, 0xBB};
static u8 cm1_0316[] = {0x44, 0x00};
static u8 cm1_0317[] = {0x45, 0xCA};
static u8 cm1_0318[] = {0x46, 0x00};
static u8 cm1_0319[] = {0x47, 0xFF};
static u8 cm1_0320[] = {0x48, 0x01};
static u8 cm1_0321[] = {0x49, 0x2A};
static u8 cm1_0322[] = {0x4A, 0x01};
static u8 cm1_0323[] = {0x4B, 0x6B};
static u8 cm1_0324[] = {0x4C, 0x01};
static u8 cm1_0325[] = {0x4D, 0x9E};
static u8 cm1_0326[] = {0x4E, 0x01};
static u8 cm1_0327[] = {0x4F, 0xEF};
static u8 cm1_0328[] = {0x50, 0x02};
static u8 cm1_0329[] = {0x51, 0x2D};
static u8 cm1_0330[] = {0x52, 0x02};
static u8 cm1_0331[] = {0x53, 0x2F};
static u8 cm1_0332[] = {0x54, 0x02};
static u8 cm1_0333[] = {0x55, 0x68};
static u8 cm1_0334[] = {0x56, 0x02};
static u8 cm1_0335[] = {0x58, 0xA2};
static u8 cm1_0336[] = {0x59, 0x02};
static u8 cm1_0337[] = {0x5A, 0xC8};
static u8 cm1_0338[] = {0x5B, 0x02};
static u8 cm1_0339[] = {0x5C, 0xF9};
static u8 cm1_0340[] = {0x5D, 0x03};
static u8 cm1_0341[] = {0x5E, 0x1C};
static u8 cm1_0342[] = {0x5F, 0x03};
static u8 cm1_0343[] = {0x60, 0x46};
static u8 cm1_0344[] = {0x61, 0x03};
static u8 cm1_0345[] = {0x62, 0x53};
static u8 cm1_0346[] = {0x63, 0x03};
static u8 cm1_0347[] = {0x64, 0x61};
static u8 cm1_0348[] = {0x65, 0x03};
static u8 cm1_0349[] = {0x66, 0x71};
static u8 cm1_0350[] = {0x67, 0x03};
static u8 cm1_0351[] = {0x68, 0x81};
static u8 cm1_0352[] = {0x69, 0x03};
static u8 cm1_0353[] = {0x6A, 0x94};
static u8 cm1_0354[] = {0x6B, 0x03};
static u8 cm1_0355[] = {0x6C, 0xA8};
static u8 cm1_0356[] = {0x6D, 0x03};
static u8 cm1_0357[] = {0x6E, 0xB9};
static u8 cm1_0358[] = {0x6F, 0x03};
static u8 cm1_0359[] = {0x70, 0xC2};
static u8 cm1_0360[] = {0x71, 0x00};
static u8 cm1_0361[] = {0x72, 0x0F};
static u8 cm1_0362[] = {0x73, 0x00};
static u8 cm1_0363[] = {0x74, 0x21};
static u8 cm1_0364[] = {0x75, 0x00};
static u8 cm1_0365[] = {0x76, 0x44};
static u8 cm1_0366[] = {0x77, 0x00};
static u8 cm1_0367[] = {0x78, 0x65};
static u8 cm1_0368[] = {0x79, 0x00};
static u8 cm1_0369[] = {0x7A, 0x7F};
static u8 cm1_0370[] = {0x7B, 0x00};
static u8 cm1_0371[] = {0x7C, 0x95};
static u8 cm1_0372[] = {0x7D, 0x00};
static u8 cm1_0373[] = {0x7E, 0xA9};
static u8 cm1_0374[] = {0x7F, 0x00};
static u8 cm1_0375[] = {0x80, 0xBB};
static u8 cm1_0376[] = {0x81, 0x00};
static u8 cm1_0377[] = {0x82, 0xCA};
static u8 cm1_0378[] = {0x83, 0x00};
static u8 cm1_0379[] = {0x84, 0xFF};
static u8 cm1_0380[] = {0x85, 0x01};
static u8 cm1_0381[] = {0x86, 0x2A};
static u8 cm1_0382[] = {0x87, 0x01};
static u8 cm1_0383[] = {0x88, 0x6B};
static u8 cm1_0384[] = {0x89, 0x01};
static u8 cm1_0385[] = {0x8A, 0x9E};
static u8 cm1_0386[] = {0x8B, 0x01};
static u8 cm1_0387[] = {0x8C, 0xEF};
static u8 cm1_0388[] = {0x8D, 0x02};
static u8 cm1_0389[] = {0x8E, 0x2D};
static u8 cm1_0390[] = {0x8F, 0x02};
static u8 cm1_0391[] = {0x90, 0x2F};
static u8 cm1_0392[] = {0x91, 0x02};
static u8 cm1_0393[] = {0x92, 0x68};
static u8 cm1_0394[] = {0x93, 0x02};
static u8 cm1_0395[] = {0x94, 0xA2};
static u8 cm1_0396[] = {0x95, 0x02};
static u8 cm1_0397[] = {0x96, 0xC8};
static u8 cm1_0398[] = {0x97, 0x02};
static u8 cm1_0399[] = {0x98, 0xF9};
static u8 cm1_0400[] = {0x99, 0x03};
static u8 cm1_0401[] = {0x9A, 0x1C};
static u8 cm1_0402[] = {0x9B, 0x03};
static u8 cm1_0403[] = {0x9C, 0x46};
static u8 cm1_0404[] = {0x9D, 0x03};
static u8 cm1_0405[] = {0x9E, 0x53};
static u8 cm1_0406[] = {0x9F, 0x03};
static u8 cm1_0407[] = {0xA0, 0x61};
static u8 cm1_0408[] = {0xA2, 0x03};
static u8 cm1_0409[] = {0xA3, 0x71};
static u8 cm1_0410[] = {0xA4, 0x03};
static u8 cm1_0411[] = {0xA5, 0x81};
static u8 cm1_0412[] = {0xA6, 0x03};
static u8 cm1_0413[] = {0xA7, 0x94};
static u8 cm1_0414[] = {0xA9, 0x03};
static u8 cm1_0415[] = {0xAA, 0xA8};
static u8 cm1_0416[] = {0xAB, 0x03};
static u8 cm1_0417[] = {0xAC, 0xB9};
static u8 cm1_0418[] = {0xAD, 0x03};
static u8 cm1_0419[] = {0xAE, 0xC2};
static u8 cm1_0420[] = {0xAF, 0x00};
static u8 cm1_0421[] = {0xB0, 0x0F};
static u8 cm1_0422[] = {0xB1, 0x00};
static u8 cm1_0423[] = {0xB2, 0x21};
static u8 cm1_0424[] = {0xB3, 0x00};
static u8 cm1_0425[] = {0xB4, 0x44};
static u8 cm1_0426[] = {0xB5, 0x00};
static u8 cm1_0427[] = {0xB6, 0x65};
static u8 cm1_0428[] = {0xB7, 0x00};
static u8 cm1_0429[] = {0xB8, 0x7F};
static u8 cm1_0430[] = {0xB9, 0x00};
static u8 cm1_0431[] = {0xBA, 0x95};
static u8 cm1_0432[] = {0xBB, 0x00};
static u8 cm1_0433[] = {0xBC, 0xA9};
static u8 cm1_0434[] = {0xBD, 0x00};
static u8 cm1_0435[] = {0xBE, 0xBB};
static u8 cm1_0436[] = {0xBF, 0x00};
static u8 cm1_0437[] = {0xC0, 0xCA};
static u8 cm1_0438[] = {0xC1, 0x00};
static u8 cm1_0439[] = {0xC2, 0xFF};
static u8 cm1_0440[] = {0xC3, 0x01};
static u8 cm1_0441[] = {0xC4, 0x2A};
static u8 cm1_0442[] = {0xC5, 0x01};
static u8 cm1_0443[] = {0xC6, 0x6B};
static u8 cm1_0444[] = {0xC7, 0x01};
static u8 cm1_0445[] = {0xC8, 0x9E};
static u8 cm1_0446[] = {0xC9, 0x01};
static u8 cm1_0447[] = {0xCA, 0xEF};
static u8 cm1_0448[] = {0xCB, 0x02};
static u8 cm1_0449[] = {0xCC, 0x2D};
static u8 cm1_0450[] = {0xCD, 0x02};
static u8 cm1_0451[] = {0xCE, 0x2F};
static u8 cm1_0452[] = {0xCF, 0x02};
static u8 cm1_0453[] = {0xD0, 0x68};
static u8 cm1_0454[] = {0xD1, 0x02};
static u8 cm1_0455[] = {0xD2, 0xA2};
static u8 cm1_0456[] = {0xD3, 0x02};
static u8 cm1_0457[] = {0xD4, 0xC8};
static u8 cm1_0458[] = {0xD5, 0x02};
static u8 cm1_0459[] = {0xD6, 0xF9};
static u8 cm1_0460[] = {0xD7, 0x03};
static u8 cm1_0461[] = {0xD8, 0x1C};
static u8 cm1_0462[] = {0xD9, 0x03};
static u8 cm1_0463[] = {0xDA, 0x46};
static u8 cm1_0464[] = {0xDB, 0x03};
static u8 cm1_0465[] = {0xDC, 0x53};
static u8 cm1_0466[] = {0xDD, 0x03};
static u8 cm1_0467[] = {0xDE, 0x61};
static u8 cm1_0468[] = {0xDF, 0x03};
static u8 cm1_0469[] = {0xE0, 0x71};
static u8 cm1_0470[] = {0xE1, 0x03};
static u8 cm1_0471[] = {0xE2, 0x81};
static u8 cm1_0472[] = {0xE3, 0x03};
static u8 cm1_0473[] = {0xE4, 0x94};
static u8 cm1_0474[] = {0xE5, 0x03};
static u8 cm1_0475[] = {0xE6, 0xA8};
static u8 cm1_0476[] = {0xE7, 0x03};
static u8 cm1_0477[] = {0xE8, 0xB9};
static u8 cm1_0478[] = {0xE9, 0x03};
static u8 cm1_0479[] = {0xEA, 0xC2};
static u8 cm1_0480[] = {0xFF, 0x04};
static u8 cm1_0481[] = {0x08, 0x0C};
static u8 cm1_0482[] = {0xFB, 0x01};
static u8 cm1_0483[] = {0xFF, 0x00};
static u8 cm1_0484[] = {0x35, 0x01};
static u8 cm1_0485[] = {0xD3, 0x06};
static u8 cm1_0486[] = {0xD4, 0x04};

static u8 cm2_001[] = {0xFF, 0x05};
static u8 cm2_002[] = {0xFB, 0x01};
static u8 cm2_003[] = {0xC5, 0x31};
static u8 cm2_004[] = {0xFF, 0x00};
static u8 cm2_005[] = {0xFB, 0x01};
static u8 cm2_006[] = {0x35, 0x01};
static u8 cm2_007[] = {0xD3, 0x06};
static u8 cm2_008[] = {0xD4, 0x04};

static u8 cm3_001[] = {0xFF, 0x00};
static u8 cm3_002[] = {0xD3, 0x06};
static u8 cm3_003[] = {0xD4, 0x04};
static u8 cm3_004[] = {0xFF, 0x01};
static u8 cm3_005[] = {0xFB, 0x01};
static u8 cm3_006[] = {0x15, 0x0F};
static u8 cm3_007[] = {0x16, 0x0F};
static u8 cm3_008[] = {0x1B, 0x1B};
static u8 cm3_009[] = {0x1C, 0xF7};
static u8 cm3_010[] = {0x60, 0x0F};
static u8 cm3_011[] = {0x58, 0x82};
static u8 cm3_012[] = {0x59, 0x00};
static u8 cm3_013[] = {0x5A, 0x02};
static u8 cm3_014[] = {0x5B, 0x00};
static u8 cm3_015[] = {0x5C, 0x82};
static u8 cm3_016[] = {0x5D, 0x80};
static u8 cm3_017[] = {0x5E, 0x02};
static u8 cm3_018[] = {0x5F, 0x00};
static u8 cm3_019[] = {0x66, 0x01};
static u8 cm3_020[] = {0xFF, 0x05};
static u8 cm3_021[] = {0xFB, 0x01};
static u8 cm3_022[] = {0x85, 0x05};
static u8 cm3_023[] = {0xA6, 0x04};
static u8 cm3_024[] = {0xFF, 0xFF};
static u8 cm3_025[] = {0xFB, 0x01};
static u8 cm3_026[] = {0x4F, 0x03};
static u8 cm3_027[] = {0xFF, 0x00};

/* ESD workaround */
static u8 cm3_101[] = {0x05, 0x1A};
static u8 cm3_102[] = {0x06, 0x10};
static u8 cm3_103[] = {0x07, 0x00};
static u8 cm3_104[] = {0x15, 0x1A};
static u8 cm3_105[] = {0x16, 0x10};
static u8 cm3_106[] = {0x17, 0x10};
static u8 cm3_107[] = {0x53, 0x06};
static u8 cm3_108[] = {0x7E, 0x05};
static u8 cm3_109[] = {0x7F, 0x20};

/* Display noise */
static u8 cm3_110[] = {0x86, 0x1B};
static u8 cm3_111[] = {0x87, 0x39};
static u8 cm3_112[] = {0x88, 0x1B};
static u8 cm3_113[] = {0x89, 0x39};
static u8 cm3_114[] = {0xB5, 0x20};
static u8 cm3_115[] = {0x8C, 0x01};
static u8 cm3_116[] = {0x4D, 0x00};
static u8 cm3_117[] = {0x4E, 0x00};
static u8 cm3_118[] = {0x4F, 0x11};
static u8 cm3_119[] = {0x50, 0x11};
static u8 cm3_120[] = {0x54, 0x70};

/* ====Power on commnad==== */
static struct mipi_dsi_cmd ze551ml_power_on_table[] = {
	{0, sizeof(cm1_01), cm1_01},
	{0, sizeof(cm1_02), cm1_02},
	{0, sizeof(cm1_03), cm1_03},
	{0, sizeof(cm1_04), cm1_04},
	{0, sizeof(cm1_05), cm1_05},
	{0, sizeof(cm1_06), cm1_06},
	{0, sizeof(cm1_07), cm1_07},
	{0, sizeof(cm1_08), cm1_08},
	{0, sizeof(cm1_09), cm1_09},
	{0, sizeof(cm1_010), cm1_010},
	{0, sizeof(cm1_011), cm1_011},
	{0, sizeof(cm1_012), cm1_012},
	{0, sizeof(cm1_013), cm1_013},
	{0, sizeof(cm1_014), cm1_014},
	{0, sizeof(cm1_015), cm1_015},
	{0, sizeof(cm1_016), cm1_016},
	{0, sizeof(cm1_017), cm1_017},
	{0, sizeof(cm1_018), cm1_018},
	{0, sizeof(cm1_019), cm1_019},
	{0, sizeof(cm1_020), cm1_020},
	{0, sizeof(cm1_021), cm1_021},
	{0, sizeof(cm1_022), cm1_022},
	{0, sizeof(cm1_023), cm1_023},
	{0, sizeof(cm1_024), cm1_024},
	{0, sizeof(cm1_025), cm1_025},
	{0, sizeof(cm1_026), cm1_026},
	{0, sizeof(cm1_027), cm1_027},
	{0, sizeof(cm1_028), cm1_028},
	{0, sizeof(cm1_029), cm1_029},
	{0, sizeof(cm1_030), cm1_030},
	{0, sizeof(cm1_031), cm1_031},
	{0, sizeof(cm1_032), cm1_032},
	{0, sizeof(cm1_033), cm1_033},
	{0, sizeof(cm1_034), cm1_034},
	{0, sizeof(cm1_035), cm1_035},
	{0, sizeof(cm1_036), cm1_036},
	{0, sizeof(cm1_037), cm1_037},
	{0, sizeof(cm1_038), cm1_038},
	{0, sizeof(cm1_039), cm1_039},
	{0, sizeof(cm1_040), cm1_040},
	{0, sizeof(cm1_041), cm1_041},
	{0, sizeof(cm1_042), cm1_042},
	{0, sizeof(cm1_043), cm1_043},
	{0, sizeof(cm1_044), cm1_044},
	{0, sizeof(cm1_045), cm1_045},
	{0, sizeof(cm1_046), cm1_046},
	{0, sizeof(cm1_047), cm1_047},
	{0, sizeof(cm1_048), cm1_048},
	{0, sizeof(cm1_049), cm1_049},
	{0, sizeof(cm1_050), cm1_050},
	{0, sizeof(cm1_051), cm1_051},
	{0, sizeof(cm1_052), cm1_052},
	{0, sizeof(cm1_053), cm1_053},
	{0, sizeof(cm1_054), cm1_054},
	{0, sizeof(cm1_055), cm1_055},
	{0, sizeof(cm1_056), cm1_056},
	{0, sizeof(cm1_057), cm1_057},
	{0, sizeof(cm1_058), cm1_058},
	{0, sizeof(cm1_059), cm1_059},
	{0, sizeof(cm1_060), cm1_060},
	{0, sizeof(cm1_061), cm1_061},
	{0, sizeof(cm1_062), cm1_062},
	{0, sizeof(cm1_063), cm1_063},
	{0, sizeof(cm1_064), cm1_064},
	{0, sizeof(cm1_065), cm1_065},
	{0, sizeof(cm1_066), cm1_066},
	{0, sizeof(cm1_067), cm1_067},
	{0, sizeof(cm1_068), cm1_068},
	{0, sizeof(cm1_069), cm1_069},
	{0, sizeof(cm1_070), cm1_070},
	{0, sizeof(cm1_071), cm1_071},
	{0, sizeof(cm1_072), cm1_072},
	{0, sizeof(cm1_073), cm1_073},
	{0, sizeof(cm1_074), cm1_074},
	{0, sizeof(cm1_075), cm1_075},
	{0, sizeof(cm1_076), cm1_076},
	{0, sizeof(cm1_077), cm1_077},
	{0, sizeof(cm1_078), cm1_078},
	{0, sizeof(cm1_079), cm1_079},
	{0, sizeof(cm1_080), cm1_080},
	{0, sizeof(cm1_081), cm1_081},
	{0, sizeof(cm1_082), cm1_082},
	{0, sizeof(cm1_083), cm1_083},
	{0, sizeof(cm1_084), cm1_084},
	{0, sizeof(cm1_085), cm1_085},
	{0, sizeof(cm1_086), cm1_086},
	{0, sizeof(cm1_087), cm1_087},
	{0, sizeof(cm1_088), cm1_088},
	{0, sizeof(cm1_089), cm1_089},
	{0, sizeof(cm1_090), cm1_090},
	{0, sizeof(cm1_091), cm1_091},
	{0, sizeof(cm1_092), cm1_092},
	{0, sizeof(cm1_093), cm1_093},
	{0, sizeof(cm1_094), cm1_094},
	{0, sizeof(cm1_095), cm1_095},
	{0, sizeof(cm1_096), cm1_096},
	{0, sizeof(cm1_097), cm1_097},
	{0, sizeof(cm1_098), cm1_098},
	{0, sizeof(cm1_099), cm1_099},
	{0, sizeof(cm1_0100), cm1_0100},
	{0, sizeof(cm1_0101), cm1_0101},
	{0, sizeof(cm1_0102), cm1_0102},
	{0, sizeof(cm1_0103), cm1_0103},
	{0, sizeof(cm1_0104), cm1_0104},
	{0, sizeof(cm1_0105), cm1_0105},
	{0, sizeof(cm1_0106), cm1_0106},
	{0, sizeof(cm1_0107), cm1_0107},
	{0, sizeof(cm1_0108), cm1_0108},
	{0, sizeof(cm1_0109), cm1_0109},
	{0, sizeof(cm1_0110), cm1_0110},
	{0, sizeof(cm1_0111), cm1_0111},
	{0, sizeof(cm1_0112), cm1_0112},
	{0, sizeof(cm1_0113), cm1_0113},
	{0, sizeof(cm1_0114), cm1_0114},
	{0, sizeof(cm1_0115), cm1_0115},
	{0, sizeof(cm1_0116), cm1_0116},
	{0, sizeof(cm1_0117), cm1_0117},
	{0, sizeof(cm1_0118), cm1_0118},
	{0, sizeof(cm1_0119), cm1_0119},
	{0, sizeof(cm1_0120), cm1_0120},
	{0, sizeof(cm1_0121), cm1_0121},
	{0, sizeof(cm1_0122), cm1_0122},
	{0, sizeof(cm1_0123), cm1_0123},
	{0, sizeof(cm1_0124), cm1_0124},
	{0, sizeof(cm1_0125), cm1_0125},
	{0, sizeof(cm1_0126), cm1_0126},
	{0, sizeof(cm1_0127), cm1_0127},
	{0, sizeof(cm1_0128), cm1_0128},
	{0, sizeof(cm1_0129), cm1_0129},
	{0, sizeof(cm1_0130), cm1_0130},
	{0, sizeof(cm1_0131), cm1_0131},
	{0, sizeof(cm1_0132), cm1_0132},
	{0, sizeof(cm1_0133), cm1_0133},
	{0, sizeof(cm1_0134), cm1_0134},
	{0, sizeof(cm1_0135), cm1_0135},
	{0, sizeof(cm1_0136), cm1_0136},
	{0, sizeof(cm1_0137), cm1_0137},
	{0, sizeof(cm1_0138), cm1_0138},
	{0, sizeof(cm1_0139), cm1_0139},
	{0, sizeof(cm1_0140), cm1_0140},
	{0, sizeof(cm1_0141), cm1_0141},
	{0, sizeof(cm1_0142), cm1_0142},
	{0, sizeof(cm1_0143), cm1_0143},
	{0, sizeof(cm1_0144), cm1_0144},
	{0, sizeof(cm1_0145), cm1_0145},
	{0, sizeof(cm1_0146), cm1_0146},
	{0, sizeof(cm1_0147), cm1_0147},
	{0, sizeof(cm1_0148), cm1_0148},
	{0, sizeof(cm1_0149), cm1_0149},
	{0, sizeof(cm1_0150), cm1_0150},
	{0, sizeof(cm1_0151), cm1_0151},
	{0, sizeof(cm1_0152), cm1_0152},
	{0, sizeof(cm1_0153), cm1_0153},
	{0, sizeof(cm1_0154), cm1_0154},
	{0, sizeof(cm1_0155), cm1_0155},
	{0, sizeof(cm1_0156), cm1_0156},
	{0, sizeof(cm1_0157), cm1_0157},
	{0, sizeof(cm1_0158), cm1_0158},
	{0, sizeof(cm1_0159), cm1_0159},
	{0, sizeof(cm1_0160), cm1_0160},
	{0, sizeof(cm1_0161), cm1_0161},
	{0, sizeof(cm1_0162), cm1_0162},
	{0, sizeof(cm1_0163), cm1_0163},
	{0, sizeof(cm1_0164), cm1_0164},
	{0, sizeof(cm1_0165), cm1_0165},
	{0, sizeof(cm1_0166), cm1_0166},
	{0, sizeof(cm1_0167), cm1_0167},
	{0, sizeof(cm1_0168), cm1_0168},
	{0, sizeof(cm1_0169), cm1_0169},
	{0, sizeof(cm1_0170), cm1_0170},
	{0, sizeof(cm1_0171), cm1_0171},
	{0, sizeof(cm1_0172), cm1_0172},
	{0, sizeof(cm1_0173), cm1_0173},
	{0, sizeof(cm1_0174), cm1_0174},
	{0, sizeof(cm1_0175), cm1_0175},
	{0, sizeof(cm1_0176), cm1_0176},
	{0, sizeof(cm1_0177), cm1_0177},
	{0, sizeof(cm1_0178), cm1_0178},
	{0, sizeof(cm1_0179), cm1_0179},
	{0, sizeof(cm1_0180), cm1_0180},
	{0, sizeof(cm1_0181), cm1_0181},
	{0, sizeof(cm1_0182), cm1_0182},
	{0, sizeof(cm1_0183), cm1_0183},
	{0, sizeof(cm1_0184), cm1_0184},
	{0, sizeof(cm1_0185), cm1_0185},
	{0, sizeof(cm1_0186), cm1_0186},
	{0, sizeof(cm1_0187), cm1_0187},
	{0, sizeof(cm1_0188), cm1_0188},
	{0, sizeof(cm1_0189), cm1_0189},
	{0, sizeof(cm1_0190), cm1_0190},
	{0, sizeof(cm1_0191), cm1_0191},
	{0, sizeof(cm1_0192), cm1_0192},
	{0, sizeof(cm1_0193), cm1_0193},
	{0, sizeof(cm1_0194), cm1_0194},
	{0, sizeof(cm1_0195), cm1_0195},
	{0, sizeof(cm1_0196), cm1_0196},
	{0, sizeof(cm1_0197), cm1_0197},
	{0, sizeof(cm1_0198), cm1_0198},
	{0, sizeof(cm1_0199), cm1_0199},
	{0, sizeof(cm1_0200), cm1_0200},
	{0, sizeof(cm1_0201), cm1_0201},
	{0, sizeof(cm1_0202), cm1_0202},
	{0, sizeof(cm1_0203), cm1_0203},
	{0, sizeof(cm1_0204), cm1_0204},
	{0, sizeof(cm1_0205), cm1_0205},
	{0, sizeof(cm1_0206), cm1_0206},
	{0, sizeof(cm1_0207), cm1_0207},
	{0, sizeof(cm1_0208), cm1_0208},
	{0, sizeof(cm1_0209), cm1_0209},
	{0, sizeof(cm1_0210), cm1_0210},
	{0, sizeof(cm1_0211), cm1_0211},
	{0, sizeof(cm1_0212), cm1_0212},
	{0, sizeof(cm1_0213), cm1_0213},
	{0, sizeof(cm1_0214), cm1_0214},
	{0, sizeof(cm1_0215), cm1_0215},
	{0, sizeof(cm1_0216), cm1_0216},
	{0, sizeof(cm1_0217), cm1_0217},
	{0, sizeof(cm1_0218), cm1_0218},
	{0, sizeof(cm1_0219), cm1_0219},
	{0, sizeof(cm1_0220), cm1_0220},
	{0, sizeof(cm1_0221), cm1_0221},
	{0, sizeof(cm1_0222), cm1_0222},
	{0, sizeof(cm1_0223), cm1_0223},
	{0, sizeof(cm1_0224), cm1_0224},
	{0, sizeof(cm1_0225), cm1_0225},
	{0, sizeof(cm1_0226), cm1_0226},
	{0, sizeof(cm1_0227), cm1_0227},
	{0, sizeof(cm1_0228), cm1_0228},
	{0, sizeof(cm1_0229), cm1_0229},
	{0, sizeof(cm1_0230), cm1_0230},
	{0, sizeof(cm1_0231), cm1_0231},
	{0, sizeof(cm1_0232), cm1_0232},
	{0, sizeof(cm1_0233), cm1_0233},
	{0, sizeof(cm1_0234), cm1_0234},
	{0, sizeof(cm1_0235), cm1_0235},
	{0, sizeof(cm1_0236), cm1_0236},
	{0, sizeof(cm1_0237), cm1_0237},
	{0, sizeof(cm1_0238), cm1_0238},
	{0, sizeof(cm1_0239), cm1_0239},
	{0, sizeof(cm1_0240), cm1_0240},
	{0, sizeof(cm1_0241), cm1_0241},
	{0, sizeof(cm1_0242), cm1_0242},
	{0, sizeof(cm1_0243), cm1_0243},
	{0, sizeof(cm1_0244), cm1_0244},
	{0, sizeof(cm1_0245), cm1_0245},
	{0, sizeof(cm1_0246), cm1_0246},
	{0, sizeof(cm1_0247), cm1_0247},
	{0, sizeof(cm1_0248), cm1_0248},
	{0, sizeof(cm1_0249), cm1_0249},
	{0, sizeof(cm1_0250), cm1_0250},
	{0, sizeof(cm1_0251), cm1_0251},
	{0, sizeof(cm1_0252), cm1_0252},
	{0, sizeof(cm1_0253), cm1_0253},
	{0, sizeof(cm1_0254), cm1_0254},
	{0, sizeof(cm1_0255), cm1_0255},
	{0, sizeof(cm1_0256), cm1_0256},
	{0, sizeof(cm1_0257), cm1_0257},
	{0, sizeof(cm1_0258), cm1_0258},
	{0, sizeof(cm1_0259), cm1_0259},
	{0, sizeof(cm1_0260), cm1_0260},
	{0, sizeof(cm1_0261), cm1_0261},
	{0, sizeof(cm1_0262), cm1_0262},
	{0, sizeof(cm1_0263), cm1_0263},
	{0, sizeof(cm1_0264), cm1_0264},
	{0, sizeof(cm1_0265), cm1_0265},
	{0, sizeof(cm1_0266), cm1_0266},
	{0, sizeof(cm1_0267), cm1_0267},
	{0, sizeof(cm1_0268), cm1_0268},
	{0, sizeof(cm1_0269), cm1_0269},
	{0, sizeof(cm1_0270), cm1_0270},
	{0, sizeof(cm1_0271), cm1_0271},
	{0, sizeof(cm1_0272), cm1_0272},
	{0, sizeof(cm1_0273), cm1_0273},
	{0, sizeof(cm1_0274), cm1_0274},
	{0, sizeof(cm1_0275), cm1_0275},
	{0, sizeof(cm1_0276), cm1_0276},
	{0, sizeof(cm1_0277), cm1_0277},
	{0, sizeof(cm1_0278), cm1_0278},
	{0, sizeof(cm1_0279), cm1_0279},
	{0, sizeof(cm1_0280), cm1_0280},
	{0, sizeof(cm1_0281), cm1_0281},
	{0, sizeof(cm1_0282), cm1_0282},
	{0, sizeof(cm1_0283), cm1_0283},
	{0, sizeof(cm1_0284), cm1_0284},
	{0, sizeof(cm1_0285), cm1_0285},
	{0, sizeof(cm1_0286), cm1_0286},
	{0, sizeof(cm1_0287), cm1_0287},
	{0, sizeof(cm1_0288), cm1_0288},
	{0, sizeof(cm1_0289), cm1_0289},
	{0, sizeof(cm1_0290), cm1_0290},
	{0, sizeof(cm1_0291), cm1_0291},
	{0, sizeof(cm1_0292), cm1_0292},
	{0, sizeof(cm1_0293), cm1_0293},
	{0, sizeof(cm1_0294), cm1_0294},
	{0, sizeof(cm1_0295), cm1_0295},
	{0, sizeof(cm1_0296), cm1_0296},
	{0, sizeof(cm1_0297), cm1_0297},
	{0, sizeof(cm1_0298), cm1_0298},
	{0, sizeof(cm1_0299), cm1_0299},
	{0, sizeof(cm1_0300), cm1_0300},
	{0, sizeof(cm1_0301), cm1_0301},
	{0, sizeof(cm1_0302), cm1_0302},
	{0, sizeof(cm1_0303), cm1_0303},
	{0, sizeof(cm1_0304), cm1_0304},
	{0, sizeof(cm1_0305), cm1_0305},
	{0, sizeof(cm1_0306), cm1_0306},
	{0, sizeof(cm1_0307), cm1_0307},
	{0, sizeof(cm1_0308), cm1_0308},
	{0, sizeof(cm1_0309), cm1_0309},
	{0, sizeof(cm1_0310), cm1_0310},
	{0, sizeof(cm1_0311), cm1_0311},
	{0, sizeof(cm1_0312), cm1_0312},
	{0, sizeof(cm1_0313), cm1_0313},
	{0, sizeof(cm1_0314), cm1_0314},
	{0, sizeof(cm1_0315), cm1_0315},
	{0, sizeof(cm1_0316), cm1_0316},
	{0, sizeof(cm1_0317), cm1_0317},
	{0, sizeof(cm1_0318), cm1_0318},
	{0, sizeof(cm1_0319), cm1_0319},
	{0, sizeof(cm1_0320), cm1_0320},
	{0, sizeof(cm1_0321), cm1_0321},
	{0, sizeof(cm1_0322), cm1_0322},
	{0, sizeof(cm1_0323), cm1_0323},
	{0, sizeof(cm1_0324), cm1_0324},
	{0, sizeof(cm1_0325), cm1_0325},
	{0, sizeof(cm1_0326), cm1_0326},
	{0, sizeof(cm1_0327), cm1_0327},
	{0, sizeof(cm1_0328), cm1_0328},
	{0, sizeof(cm1_0329), cm1_0329},
	{0, sizeof(cm1_0330), cm1_0330},
	{0, sizeof(cm1_0331), cm1_0331},
	{0, sizeof(cm1_0332), cm1_0332},
	{0, sizeof(cm1_0333), cm1_0333},
	{0, sizeof(cm1_0334), cm1_0334},
	{0, sizeof(cm1_0335), cm1_0335},
	{0, sizeof(cm1_0336), cm1_0336},
	{0, sizeof(cm1_0337), cm1_0337},
	{0, sizeof(cm1_0338), cm1_0338},
	{0, sizeof(cm1_0339), cm1_0339},
	{0, sizeof(cm1_0340), cm1_0340},
	{0, sizeof(cm1_0341), cm1_0341},
	{0, sizeof(cm1_0342), cm1_0342},
	{0, sizeof(cm1_0343), cm1_0343},
	{0, sizeof(cm1_0344), cm1_0344},
	{0, sizeof(cm1_0345), cm1_0345},
	{0, sizeof(cm1_0346), cm1_0346},
	{0, sizeof(cm1_0347), cm1_0347},
	{0, sizeof(cm1_0348), cm1_0348},
	{0, sizeof(cm1_0349), cm1_0349},
	{0, sizeof(cm1_0350), cm1_0350},
	{0, sizeof(cm1_0351), cm1_0351},
	{0, sizeof(cm1_0352), cm1_0352},
	{0, sizeof(cm1_0353), cm1_0353},
	{0, sizeof(cm1_0354), cm1_0354},
	{0, sizeof(cm1_0355), cm1_0355},
	{0, sizeof(cm1_0356), cm1_0356},
	{0, sizeof(cm1_0357), cm1_0357},
	{0, sizeof(cm1_0358), cm1_0358},
	{0, sizeof(cm1_0359), cm1_0359},
	{0, sizeof(cm1_0360), cm1_0360},
	{0, sizeof(cm1_0361), cm1_0361},
	{0, sizeof(cm1_0362), cm1_0362},
	{0, sizeof(cm1_0363), cm1_0363},
	{0, sizeof(cm1_0364), cm1_0364},
	{0, sizeof(cm1_0365), cm1_0365},
	{0, sizeof(cm1_0366), cm1_0366},
	{0, sizeof(cm1_0367), cm1_0367},
	{0, sizeof(cm1_0368), cm1_0368},
	{0, sizeof(cm1_0369), cm1_0369},
	{0, sizeof(cm1_0370), cm1_0370},
	{0, sizeof(cm1_0371), cm1_0371},
	{0, sizeof(cm1_0372), cm1_0372},
	{0, sizeof(cm1_0373), cm1_0373},
	{0, sizeof(cm1_0374), cm1_0374},
	{0, sizeof(cm1_0375), cm1_0375},
	{0, sizeof(cm1_0376), cm1_0376},
	{0, sizeof(cm1_0377), cm1_0377},
	{0, sizeof(cm1_0378), cm1_0378},
	{0, sizeof(cm1_0379), cm1_0379},
	{0, sizeof(cm1_0380), cm1_0380},
	{0, sizeof(cm1_0381), cm1_0381},
	{0, sizeof(cm1_0382), cm1_0382},
	{0, sizeof(cm1_0383), cm1_0383},
	{0, sizeof(cm1_0384), cm1_0384},
	{0, sizeof(cm1_0385), cm1_0385},
	{0, sizeof(cm1_0386), cm1_0386},
	{0, sizeof(cm1_0387), cm1_0387},
	{0, sizeof(cm1_0388), cm1_0388},
	{0, sizeof(cm1_0389), cm1_0389},
	{0, sizeof(cm1_0390), cm1_0390},
	{0, sizeof(cm1_0391), cm1_0391},
	{0, sizeof(cm1_0392), cm1_0392},
	{0, sizeof(cm1_0393), cm1_0393},
	{0, sizeof(cm1_0394), cm1_0394},
	{0, sizeof(cm1_0395), cm1_0395},
	{0, sizeof(cm1_0396), cm1_0396},
	{0, sizeof(cm1_0397), cm1_0397},
	{0, sizeof(cm1_0398), cm1_0398},
	{0, sizeof(cm1_0399), cm1_0399},
	{0, sizeof(cm1_0400), cm1_0400},
	{0, sizeof(cm1_0401), cm1_0401},
	{0, sizeof(cm1_0402), cm1_0402},
	{0, sizeof(cm1_0403), cm1_0403},
	{0, sizeof(cm1_0404), cm1_0404},
	{0, sizeof(cm1_0405), cm1_0405},
	{0, sizeof(cm1_0406), cm1_0406},
	{0, sizeof(cm1_0407), cm1_0407},
	{0, sizeof(cm1_0408), cm1_0408},
	{0, sizeof(cm1_0409), cm1_0409},
	{0, sizeof(cm1_0410), cm1_0410},
	{0, sizeof(cm1_0411), cm1_0411},
	{0, sizeof(cm1_0412), cm1_0412},
	{0, sizeof(cm1_0413), cm1_0413},
	{0, sizeof(cm1_0414), cm1_0414},
	{0, sizeof(cm1_0415), cm1_0415},
	{0, sizeof(cm1_0416), cm1_0416},
	{0, sizeof(cm1_0417), cm1_0417},
	{0, sizeof(cm1_0418), cm1_0418},
	{0, sizeof(cm1_0419), cm1_0419},
	{0, sizeof(cm1_0420), cm1_0420},
	{0, sizeof(cm1_0421), cm1_0421},
	{0, sizeof(cm1_0422), cm1_0422},
	{0, sizeof(cm1_0423), cm1_0423},
	{0, sizeof(cm1_0424), cm1_0424},
	{0, sizeof(cm1_0425), cm1_0425},
	{0, sizeof(cm1_0426), cm1_0426},
	{0, sizeof(cm1_0427), cm1_0427},
	{0, sizeof(cm1_0428), cm1_0428},
	{0, sizeof(cm1_0429), cm1_0429},
	{0, sizeof(cm1_0430), cm1_0430},
	{0, sizeof(cm1_0431), cm1_0431},
	{0, sizeof(cm1_0432), cm1_0432},
	{0, sizeof(cm1_0433), cm1_0433},
	{0, sizeof(cm1_0434), cm1_0434},
	{0, sizeof(cm1_0435), cm1_0435},
	{0, sizeof(cm1_0436), cm1_0436},
	{0, sizeof(cm1_0437), cm1_0437},
	{0, sizeof(cm1_0438), cm1_0438},
	{0, sizeof(cm1_0439), cm1_0439},
	{0, sizeof(cm1_0440), cm1_0440},
	{0, sizeof(cm1_0441), cm1_0441},
	{0, sizeof(cm1_0442), cm1_0442},
	{0, sizeof(cm1_0443), cm1_0443},
	{0, sizeof(cm1_0444), cm1_0444},
	{0, sizeof(cm1_0445), cm1_0445},
	{0, sizeof(cm1_0446), cm1_0446},
	{0, sizeof(cm1_0447), cm1_0447},
	{0, sizeof(cm1_0448), cm1_0448},
	{0, sizeof(cm1_0449), cm1_0449},
	{0, sizeof(cm1_0450), cm1_0450},
	{0, sizeof(cm1_0451), cm1_0451},
	{0, sizeof(cm1_0452), cm1_0452},
	{0, sizeof(cm1_0453), cm1_0453},
	{0, sizeof(cm1_0454), cm1_0454},
	{0, sizeof(cm1_0455), cm1_0455},
	{0, sizeof(cm1_0456), cm1_0456},
	{0, sizeof(cm1_0457), cm1_0457},
	{0, sizeof(cm1_0458), cm1_0458},
	{0, sizeof(cm1_0459), cm1_0459},
	{0, sizeof(cm1_0460), cm1_0460},
	{0, sizeof(cm1_0461), cm1_0461},
	{0, sizeof(cm1_0462), cm1_0462},
	{0, sizeof(cm1_0463), cm1_0463},
	{0, sizeof(cm1_0464), cm1_0464},
	{0, sizeof(cm1_0465), cm1_0465},
	{0, sizeof(cm1_0466), cm1_0466},
	{0, sizeof(cm1_0467), cm1_0467},
	{0, sizeof(cm1_0468), cm1_0468},
	{0, sizeof(cm1_0469), cm1_0469},
	{0, sizeof(cm1_0470), cm1_0470},
	{0, sizeof(cm1_0471), cm1_0471},
	{0, sizeof(cm1_0472), cm1_0472},
	{0, sizeof(cm1_0473), cm1_0473},
	{0, sizeof(cm1_0474), cm1_0474},
	{0, sizeof(cm1_0475), cm1_0475},
	{0, sizeof(cm1_0476), cm1_0476},
	{0, sizeof(cm1_0477), cm1_0477},
	{0, sizeof(cm1_0478), cm1_0478},
	{0, sizeof(cm1_0479), cm1_0479},
	{0, sizeof(cm1_0480), cm1_0480},
	{0, sizeof(cm1_0481), cm1_0481},
	{0, sizeof(cm1_0482), cm1_0482},
	{0, sizeof(cm1_0483), cm1_0483},
	{0, sizeof(cm1_0484), cm1_0484},
	{0, sizeof(cm1_0485), cm1_0485},
	{0, sizeof(cm1_0486), cm1_0486},
};

static struct mipi_dsi_cmd ze551ml_TM_ER_power_on_table[] = {
	{0, sizeof(cm2_001), cm2_001},
	{0, sizeof(cm2_002), cm2_002},
	{0, sizeof(cm2_003), cm2_003},
	{0, sizeof(cm2_004), cm2_004},
	{0, sizeof(cm2_005), cm2_005},
	{0, sizeof(cm2_006), cm2_006},
	{0, sizeof(cm2_007), cm2_007},
	{0, sizeof(cm2_008), cm2_008},
};

static struct mipi_dsi_cmd ze551ml_AUO_power_on_table[] = {
	{0, sizeof(cm3_001), cm3_001},
	{0, sizeof(cm3_002), cm3_002},
	{0, sizeof(cm3_003), cm3_003},
	{0, sizeof(cm3_020), cm3_020},
	{0, sizeof(cm3_021), cm3_021},
	{0, sizeof(cm3_101), cm3_101},
	{0, sizeof(cm3_102), cm3_102},
	{0, sizeof(cm3_103), cm3_103},
	{0, sizeof(cm3_104), cm3_104},
	{0, sizeof(cm3_105), cm3_105},
	{0, sizeof(cm3_106), cm3_106},
	{0, sizeof(cm3_107), cm3_107},
	{0, sizeof(cm3_108), cm3_108},
	{0, sizeof(cm3_109), cm3_109},
        {0, sizeof(cm3_110), cm3_110},
        {0, sizeof(cm3_111), cm3_111},
        {0, sizeof(cm3_112), cm3_112},
        {0, sizeof(cm3_113), cm3_113},
        {0, sizeof(cm3_114), cm3_114},
        {0, sizeof(cm3_115), cm3_115},
        {0, sizeof(cm3_116), cm3_116},
        {0, sizeof(cm3_117), cm3_117},
        {0, sizeof(cm3_118), cm3_118},
        {0, sizeof(cm3_119), cm3_119},
        {0, sizeof(cm3_120), cm3_120},
	{0, sizeof(cm3_027), cm3_027},
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


struct delayed_work nt35596_panel_reset_delay_work;
struct workqueue_struct *nt35596_panel_reset_delay_wq;
static int nt35596_vid_drv_ic_reset_workaround(struct mdfld_dsi_config *dsi_config) {

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

static void nt35596_vid_panel_reset_delay_work(struct work_struct *work)
{
//	printk("[DEBUG] %s\n", __func__);
	nt35596_vid_drv_ic_reset_workaround(panel_reset_dsi_config);
	queue_delayed_work(nt35596_panel_reset_delay_wq, &nt35596_panel_reset_delay_work, msecs_to_jiffies(5000));
}


static int nt35596_vid_drv_ic_init(struct mdfld_dsi_config *dsi_config){

	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int i;
	static int first_boot = 1;

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}

	printk("[DISP] %s\n", __func__);

	if (first_boot) {
		if (Read_LCD_ID() == ZE551ML_LCD_ID_NT_AUO) {
			gpio_set_value_cansleep(panel_reset_gpio, 1);
			usleep_range(5000, 5050);
			gpio_set_value_cansleep(panel_reset_gpio, 0);
			usleep_range(5000, 5050);
			gpio_set_value_cansleep(panel_reset_gpio, 1);
			usleep_range(120000, 120100);
		} else {
			/* two finger reset to fix some panel abnormal status*/
			gpio_set_value_cansleep(panel_reset_gpio, 1);
			usleep_range(5000, 5050);
			gpio_set_value_cansleep(panel_reset_gpio, 0);
			usleep_range(5000, 5050);
			gpio_set_value_cansleep(panel_reset_gpio, 1);
			usleep_range(120000, 120100);

			gpio_set_value_cansleep(panel_reset_gpio, 0);
			usleep_range(5000, 5050);
			gpio_set_value_cansleep(panel_reset_gpio, 1);
			usleep_range(5000, 5050);
			gpio_set_value_cansleep(panel_reset_gpio, 0);
			usleep_range(5000, 5050);
			gpio_set_value_cansleep(panel_reset_gpio, 1);
			usleep_range(20000, 20050);
		}
		first_boot = 0;
	} else {
		gpio_set_value_cansleep(panel_reset_gpio, 1);
		usleep_range(5000, 5050);
		gpio_set_value_cansleep(panel_reset_gpio, 0);
		usleep_range(5000, 5050);
		gpio_set_value_cansleep(panel_reset_gpio, 1);

		if (Read_LCD_ID() == ZE551ML_LCD_ID_NT_AUO)
			usleep_range(60000, 60100);
		else
			usleep_range(20000, 20100);
	}

	/* panel initial settings */
	for (i = 0; i < nt35596_power_on_table_size; i++)
		send_mipi_cmd_mcs(sender, &nt35596_power_on_table[i]);

	mdfld_dsi_send_mcs_short_lp(sender, 0x11, 0, 0, MDFLD_DSI_SEND_PACKAGE);
	mdelay(15);
	queue_delayed_work(nt35596_panel_reset_delay_wq, &nt35596_panel_reset_delay_work, msecs_to_jiffies(100));

	return 0;
}

static void
nt35596_vid_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
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
	if (Read_LCD_ID() == ZE551ML_LCD_ID_NT_AUO) {
		hw_ctx->high_low_switch_count = 0x2f;
		hw_ctx->clk_lane_switch_time_cnt = 0x3d0016;
		hw_ctx->lp_byteclk = 0x6;
		hw_ctx->dphy_param = 0x351B6D1F;
	} else {
		hw_ctx->high_low_switch_count = 0x31;
		hw_ctx->clk_lane_switch_time_cnt = 0x3e0018;
		hw_ctx->lp_byteclk = 0x6;
		hw_ctx->dphy_param = 0x3F1F7817;
	}
	hw_ctx->eot_disable = 0x3;
	hw_ctx->init_count = 0x7D0;
	hw_ctx->dsi_func_prg = (RGB_888_FMT << FMT_DPI_POS) |
		dsi_config->lane_count;
	hw_ctx->mipi = MIPI_PORT_EN | PASS_FROM_SPHY_TO_AFE |
		BANDGAP_CHICKEN_BIT;
	hw_ctx->video_mode_format = 0xF;

	nt35596_dsi_config = dsi_config;

	/* Panel initial settings assigned */
	if (Read_LCD_ID() == ZE551ML_LCD_ID_NT_AUO) {
		printk("[DISP] AUO Panel initial cmds registered\n");
		nt35596_power_on_table = ze551ml_AUO_power_on_table;
		nt35596_power_on_table_size = ARRAY_SIZE(ze551ml_AUO_power_on_table);
	} else if (Read_HW_ID() == HW_ID_SR1 || Read_HW_ID() == HW_ID_SR2) {
		printk("[DISP] TM SR Panel initial cmds registered\n");
		nt35596_power_on_table = ze551ml_power_on_table;
		nt35596_power_on_table_size = ARRAY_SIZE(ze551ml_power_on_table);
	} else {
		printk("[DISP] TM ER Panel initial cmds registered\n");
		nt35596_power_on_table = ze551ml_TM_ER_power_on_table;
		nt35596_power_on_table_size = ARRAY_SIZE(ze551ml_TM_ER_power_on_table);
	}
}

static int nt35596_vid_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	usleep_range(1000, 1100);
//	queue_delayed_work(nt35596_panel_reset_delay_wq, &nt35596_panel_reset_delay_work, msecs_to_jiffies(5000));

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


static int nt35596_vid_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;

	printk("[DISP] %s\n", __func__);
	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return -EINVAL;
	}
	cancel_delayed_work_sync(&nt35596_panel_reset_delay_work);
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

	return 0;
}

static int nt35596_vid_reset(struct mdfld_dsi_config *dsi_config)
{
	printk("[DISP] %s\n", __func__);

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
	return 0;
}


static int nt35596_vid_set_brightness(struct mdfld_dsi_config *dsi_config,
					 int level)
{
	u32 reg_level;
	union pwmctrl_reg pwmctrl;
	struct backlight_device *psb;

#ifdef CONFIG_BACKLIGHT_RT4532
	rt4532_brightness_set(level);
#endif
	/* Re-assign the minimum brightness value to 2 */
	//if (level < 2)
	//	level = 2;
	
	reg_level = ~level & 0xFF;
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
			pwmctrl.part.pwmenable = 0;
			writel(pwmctrl.full, pwmctrl_mmio);
			gpio_set_value_cansleep(backlight_pwm_gpio, 0);
			lnw_gpio_set_alt(backlight_pwm_gpio, 0);
			usleep_range(10000, 10100);
			gpio_set_value_cansleep(backlight_en_gpio, 0);
			pmu_set_pwm(PCI_D3hot);
		}
	} else {
		DRM_ERROR("Cannot map pwmctrl\n");
	}
	if (level)
		{
			//printk("[DISP NT] brightness level = %d\n", level);
		}

	return 0;
}

struct drm_display_mode *nt35596_vid_get_config_mode(void)
{
	struct drm_display_mode *mode;

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	printk("[DISP] %s\n", __func__);

	if (Read_LCD_ID() == ZE551ML_LCD_ID_NT_AUO) {
		/* RECOMMENDED PORCH SETTING
			HSA=8, HBP=16, HFP=90
			VSA=2,   VBP=4, VFP=4	 */
		mode->hdisplay = 1080;
		mode->hsync_start = mode->hdisplay + 90;
		mode->hsync_end = mode->hsync_start + 8;
		mode->htotal = mode->hsync_end + 16;

		mode->vdisplay = 1920;
		mode->vsync_start = mode->vdisplay + 4;
		mode->vsync_end = mode->vsync_start + 2;
		mode->vtotal = mode->vsync_end + 4;
	} else {
		/* RECOMMENDED PORCH SETTING
			HSA=8, HBP=16, HFP=90
			VSA=2,   VBP=4, VFP=4	 */
		mode->hdisplay = 1080;
		mode->hsync_start = mode->hdisplay + 90;
		mode->hsync_end = mode->hsync_start + 8;
		mode->htotal = mode->hsync_end + 16;

		mode->vdisplay = 1920;
		mode->vsync_start = mode->vdisplay + 4;
		mode->vsync_end = mode->vsync_start + 2;
		mode->vtotal = mode->vsync_end + 4;
	}
	mode->vrefresh = 60;
	mode->clock = mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void nt35596_vid_get_panel_info(int pipe, struct panel_info *pi)
{
	if (!pi) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	pi->width_mm = 68;
	pi->height_mm = 121;
}

static int nt35596_vid_detect(struct mdfld_dsi_config *dsi_config)
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
	backlight_en_gpio = NT35596_BL_EN_GPIO;
	if (gpio_request(backlight_en_gpio, "backlight_en")) {
		DRM_ERROR("Faild to request backlight enable gpio\n");
		return -EINVAL;
	}

	backlight_pwm_gpio = NT35596_BL_PWM_GPIO;
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


#ifdef NT35596_DEBUG
static int send_mipi_ret = -1;
static int read_mipi_ret = -1;
static u8 read_mipi_data = 0;

static ssize_t send_mipi_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
    int x0=0, x1=0;
    struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(nt35596_dsi_config);

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
			= mdfld_dsi_get_pkg_sender(nt35596_dsi_config);

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

DEVICE_ATTR(send_mipi_nt35596,S_IRUGO | S_IWUSR, send_mipi_show,send_mipi_store);
DEVICE_ATTR(read_mipi_nt35596,S_IRUGO | S_IWUSR, read_mipi_show,read_mipi_store);


static struct attribute *nt35596_attrs[] = {
        &dev_attr_send_mipi_nt35596.attr,
        &dev_attr_read_mipi_nt35596.attr,
        NULL
};

static struct attribute_group nt35596_attr_group = {
        .attrs = nt35596_attrs,
        .name = "nt35596",
};

#endif

void nt35596_vid_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	int ret = 0;
	printk("[DISP] %s\n", __func__);

	p_funcs->get_config_mode = nt35596_vid_get_config_mode;
	p_funcs->get_panel_info = nt35596_vid_get_panel_info;
	p_funcs->dsi_controller_init = nt35596_vid_dsi_controller_init;
	p_funcs->detect = nt35596_vid_detect;
	p_funcs->power_on = nt35596_vid_power_on;
	p_funcs->drv_ic_init = nt35596_vid_drv_ic_init;
	p_funcs->power_off = nt35596_vid_power_off;
	p_funcs->reset = nt35596_vid_reset;
	p_funcs->set_brightness = nt35596_vid_set_brightness;

	printk("[DISP] Novatek reset workqueue init!\n");
	INIT_DELAYED_WORK(&nt35596_panel_reset_delay_work, nt35596_vid_panel_reset_delay_work);
	nt35596_panel_reset_delay_wq = create_workqueue("nt35596_panel_reset_delay_timer");
	if (unlikely(!nt35596_panel_reset_delay_wq)) {
		printk("%s : unable to create Panel reset workqueue\n", __func__);
	}

#ifdef NT35596_DEBUG
	sysfs_create_group(&dev->dev->kobj, &nt35596_attr_group);
#endif

}
