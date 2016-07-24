/*
 * drivers/media/i2c/smiapp/smiapp-quirk.c
 *
 * Generic driver for SMIA/SMIA++ compliant camera modules
 *
 * Copyright (C) 2011--2012 Nokia Corporation
 * Contact: Sakari Ailus <sakari.ailus@iki.fi>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 *
 */

#include <linux/delay.h>

#include "smiapp.h"
#include "smiapp-quirk-reg.h"

static int smiapp_write_8(struct smiapp_sensor *sensor, u16 reg, u8 val)
{
	return smiapp_write(sensor, SMIAPP_REG_MK_U8(reg), val);
}

static int smiapp_write_8s(struct smiapp_sensor *sensor,
			   struct smiapp_reg_8 *regs, int len)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	int rval;

	for (; len > 0; len--, regs++) {
		rval = smiapp_write_8(sensor, regs->reg, regs->val);
		if (rval < 0) {
			dev_err(&client->dev,
				"error %d writing reg 0x%4.4x, val 0x%2.2x",
				rval, regs->reg, regs->val);
			return rval;
		}
	}

	return 0;
}

void smiapp_replace_limit(struct smiapp_sensor *sensor,
			  u32 limit, u64 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);

	dev_dbg(&client->dev, "quirk: 0x%8.8x \"%s\" = %llu, 0x%llx\n",
		smiapp_reg_limits[limit].addr,
		smiapp_reg_limits[limit].what, val, val);
	sensor->limits[limit] = val;
}

static int jt8ew9_limits(struct smiapp_sensor *sensor)
{
	if (sensor->minfo.revision_number_major < 0x03)
		sensor->frame_skip = 1;

	/* Below 24 gain doesn't have effect at all, */
	/* but ~59 is needed for full dynamic range */
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_ANALOGUE_GAIN_CODE_MIN, 59);
	smiapp_replace_limit(
		sensor, SMIAPP_LIMIT_ANALOGUE_GAIN_CODE_MAX, 6000);

	return 0;
}

static int jt8ew9_post_poweron(struct smiapp_sensor *sensor)
{
	struct smiapp_reg_8 regs[] = {
		{ 0x30a3, 0xd8 }, /* Output port control : LVDS ports only */
		{ 0x30ae, 0x00 }, /* 0x0307 pll_multiplier maximum value on PLL input 9.6MHz ( 19.2MHz is divided on pre_pll_div) */
		{ 0x30af, 0xd0 }, /* 0x0307 pll_multiplier maximum value on PLL input 9.6MHz ( 19.2MHz is divided on pre_pll_div) */
		{ 0x322d, 0x04 }, /* Adjusting Processing Image Size to Scaler Toshiba Recommendation Setting */
		{ 0x3255, 0x0f }, /* Horizontal Noise Reduction Control Toshiba Recommendation Setting */
		{ 0x3256, 0x15 }, /* Horizontal Noise Reduction Control Toshiba Recommendation Setting */
		{ 0x3258, 0x70 }, /* Analog Gain Control Toshiba Recommendation Setting */
		{ 0x3259, 0x70 }, /* Analog Gain Control Toshiba Recommendation Setting */
		{ 0x325f, 0x7c }, /* Analog Gain Control Toshiba Recommendation Setting */
		{ 0x3302, 0x06 }, /* Pixel Reference Voltage Control Toshiba Recommendation Setting */
		{ 0x3304, 0x00 }, /* Pixel Reference Voltage Control Toshiba Recommendation Setting */
		{ 0x3307, 0x22 }, /* Pixel Reference Voltage Control Toshiba Recommendation Setting */
		{ 0x3308, 0x8d }, /* Pixel Reference Voltage Control Toshiba Recommendation Setting */
		{ 0x331e, 0x0f }, /* Black Hole Sun Correction Control Toshiba Recommendation Setting */
		{ 0x3320, 0x30 }, /* Black Hole Sun Correction Control Toshiba Recommendation Setting */
		{ 0x3321, 0x11 }, /* Black Hole Sun Correction Control Toshiba Recommendation Setting */
		{ 0x3322, 0x98 }, /* Black Hole Sun Correction Control Toshiba Recommendation Setting */
		{ 0x3323, 0x64 }, /* Black Hole Sun Correction Control Toshiba Recommendation Setting */
		{ 0x3325, 0x83 }, /* Read Out Timing Control Toshiba Recommendation Setting */
		{ 0x3330, 0x18 }, /* Read Out Timing Control Toshiba Recommendation Setting */
		{ 0x333c, 0x01 }, /* Read Out Timing Control Toshiba Recommendation Setting */
		{ 0x3345, 0x2f }, /* Black Hole Sun Correction Control Toshiba Recommendation Setting */
		{ 0x33de, 0x38 }, /* Horizontal Noise Reduction Control Toshiba Recommendation Setting */
		/* Taken from v03. No idea what the rest are. */
		{ 0x32e0, 0x05 },
		{ 0x32e1, 0x05 },
		{ 0x32e2, 0x04 },
		{ 0x32e5, 0x04 },
		{ 0x32e6, 0x04 },

	};

	return smiapp_write_8s(sensor, regs, ARRAY_SIZE(regs));
}

const struct smiapp_quirk smiapp_jt8ew9_quirk = {
	.limits = jt8ew9_limits,
	.post_poweron = jt8ew9_post_poweron,
};

static int imx125es_post_poweron(struct smiapp_sensor *sensor)
{
	/* Taken from v02. No idea what the other two are. */
	struct smiapp_reg_8 regs[] = {
		/*
		 * 0x3302: clk during frame blanking:
		 * 0x00 - HS mode, 0x01 - LP11
		 */
		{ 0x3302, 0x01 },
		{ 0x302d, 0x00 },
		{ 0x3b08, 0x8c },
	};

	return smiapp_write_8s(sensor, regs, ARRAY_SIZE(regs));
}

const struct smiapp_quirk smiapp_imx125es_quirk = {
	.post_poweron = imx125es_post_poweron,
};

static int jt8ev1_limits(struct smiapp_sensor *sensor)
{
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_X_ADDR_MAX, 4271);
	smiapp_replace_limit(sensor,
			     SMIAPP_LIMIT_MIN_LINE_BLANKING_PCK_BIN, 184);

	return 0;
}

static int jt8ev1_post_poweron(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	int rval;

	struct smiapp_reg_8 regs[] = {
		{ 0x3031, 0xcd }, /* For digital binning (EQ_MONI) */
		{ 0x30a3, 0xd0 }, /* FLASH STROBE enable */
		{ 0x3237, 0x00 }, /* For control of pulse timing for ADC */
		{ 0x3238, 0x43 },
		{ 0x3301, 0x06 }, /* For analog bias for sensor */
		{ 0x3302, 0x06 },
		{ 0x3304, 0x00 },
		{ 0x3305, 0x88 },
		{ 0x332a, 0x14 },
		{ 0x332c, 0x6b },
		{ 0x3336, 0x01 },
		{ 0x333f, 0x1f },
		{ 0x3355, 0x00 },
		{ 0x3356, 0x20 },
		{ 0x33bf, 0x20 }, /* Adjust the FBC speed */
		{ 0x33c9, 0x20 },
		{ 0x33ce, 0x30 }, /* Adjust the parameter for logic function */
		{ 0x33cf, 0xec }, /* For Black sun */
		{ 0x3328, 0x80 }, /* Ugh. No idea what's this. */
	};

	struct smiapp_reg_8 regs_96[] = {
		{ 0x30ae, 0x00 }, /* For control of ADC clock */
		{ 0x30af, 0xd0 },
		{ 0x30b0, 0x01 },
	};

	rval = smiapp_write_8s(sensor, regs, ARRAY_SIZE(regs));
	if (rval < 0)
		return rval;

	switch (sensor->platform_data->ext_clk) {
	case 9600000:
		return smiapp_write_8s(sensor, regs_96,
				       ARRAY_SIZE(regs_96));
	default:
		dev_warn(&client->dev, "no MSRs for %d Hz ext_clk\n",
			 sensor->platform_data->ext_clk);
		return 0;
	}
}

static int jt8ev1_pre_streamon(struct smiapp_sensor *sensor)
{
	return smiapp_write_8(sensor, 0x3328, 0x00);
}

static int jt8ev1_post_streamoff(struct smiapp_sensor *sensor)
{
	int rval;

	/* Workaround: allows fast standby to work properly */
	rval = smiapp_write_8(sensor, 0x3205, 0x04);
	if (rval < 0)
		return rval;

	/* Wait for 1 ms + one line => 2 ms is likely enough */
	usleep_range(2000, 2000);

	/* Restore it */
	rval = smiapp_write_8(sensor, 0x3205, 0x00);
	if (rval < 0)
		return rval;

	return smiapp_write_8(sensor, 0x3328, 0x80);
}

static unsigned long jt8ev1_pll_flags(struct smiapp_sensor *sensor)
{
	return SMIAPP_PLL_FLAG_OP_PIX_CLOCK_PER_LANE;
}

const struct smiapp_quirk smiapp_jt8ev1_quirk = {
	.limits = jt8ev1_limits,
	.post_poweron = jt8ev1_post_poweron,
	.pre_streamon = jt8ev1_pre_streamon,
	.post_streamoff = jt8ev1_post_streamoff,
	.pll_flags = jt8ev1_pll_flags,
};

static int tcm8500md_limits(struct smiapp_sensor *sensor)
{
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_PLL_IP_FREQ_HZ, 2700000);

	return 0;
}

const struct smiapp_quirk smiapp_tcm8500md_quirk = {
	.limits = tcm8500md_limits,
};

static int imx135_limits(struct smiapp_sensor *sensor)
{
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_PRE_PLL_CLK_DIV, 15);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_PLL_IP_FREQ_HZ, 1000000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_PLL_OP_FREQ_HZ,
			     1120000000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_OP_PIX_CLK_FREQ_HZ,
			     199680000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_VT_PIX_CLK_FREQ_HZ,
			     199680000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_OP_SYS_CLK_FREQ_HZ,
			     1120000000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_VT_SYS_CLK_FREQ_HZ,
			     1120000000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_DIGITAL_CROP_CAPABILITY,
			     SMIAPP_DIGITAL_CROP_CAPABILITY_INPUT_CROP);
	return 0;
}

static struct imx135_timing_reg_set {
	u32 op_sys_clk_freq_hz_min;
	u32 op_sys_clk_freq_hz_max;
	u8 tclk_post;
	u8 ths_prepare;
	u8 ths_zero_min;
	u8 ths_trail;
	u8 tclk_trail_min;
	u8 tclk_prepare;
	u8 tclk_zero;
	u8 tlpx;
	u8 tclk_pre;
	u8 tclk_exit;
	u8 tlpxesc;
} imx135_timing_reg_sets[] = {
	{ 128000000, 148000000, 0x57, 0x0f, 0x27, 0x0f, 0x0f, 0x07, 0x37, 0x1f, 0x1f, 0x17, 0x02, },
	{ 164000000, 192000000, 0x57, 0x0f, 0x2f, 0x17, 0x0f, 0x0f, 0x3f, 0x1f, 0x1f, 0x17, 0x02, },
	{ 192000000, 216000000, 0x57, 0x0f, 0x2f, 0x17, 0x0f, 0x0f, 0x47, 0x27, 0x1f, 0x17, 0x02, },
	{ 216000000, 224000000, 0x57, 0x17, 0x2f, 0x17, 0x0f, 0x0f, 0x47, 0x27, 0x1f, 0x17, 0x02, },
	{ 224000000, 256000000, 0x5f, 0x17, 0x37, 0x17, 0x17, 0x0f, 0x4f, 0x27, 0x1f, 0x17, 0x02, },
	{ 256000000, 288000000, 0x5f, 0x17, 0x37, 0x17, 0x17, 0x17, 0x57, 0x27, 0x1f, 0x17, 0x02, },
	{ 288000000, 320000000, 0x5f, 0x17, 0x37, 0x1f, 0x17, 0x17, 0x57, 0x27, 0x1f, 0x17, 0x02, },
	{ 320000000, 384000000, 0x5f, 0x1f, 0x3f, 0x1f, 0x1f, 0x17, 0x67, 0x27, 0x1f, 0x17, 0x02, },
	{ 384000000, 420000000, 0x67, 0x1f, 0x47, 0x1f, 0x1f, 0x17, 0x77, 0x27, 0x1f, 0x17, 0x02, },
	{ 420000000, 448000000, 0x67, 0x1f, 0x47, 0x27, 0x1f, 0x17, 0x77, 0x27, 0x1f, 0x17, 0x02, },
	{ 448000000, 512000000, 0x67, 0x27, 0x47, 0x27, 0x27, 0x1f, 0x87, 0x2f, 0x1f, 0x17, 0x02, },
	{ 512000000, 548000000, 0x6f, 0x27, 0x47, 0x27, 0x27, 0x27, 0x8f, 0x37, 0x1f, 0x17, 0x02, },
	{ 548000000, 576000000, 0x6f, 0x27, 0x47, 0x2f, 0x27, 0x27, 0x97, 0x37, 0x1f, 0x17, 0x02, },
	{ 576000000, 640000000, 0x6f, 0x27, 0x4f, 0x2f, 0x2f, 0x2f, 0x9f, 0x37, 0x1f, 0x17, 0x02, },
	{ 640000000, 676000000, 0x77, 0x2f, 0x4f, 0x2f, 0x2f, 0x37, 0xa7, 0x37, 0x1f, 0x17, 0x02, },
	{ 676000000, 704000000, 0x77, 0x2f, 0x4f, 0x37, 0x2f, 0x37, 0xaf, 0x37, 0x1f, 0x17, 0x02, },
	{ 704000000, 768000000, 0x77, 0x2f, 0x5f, 0x37, 0x37, 0x37, 0xbf, 0x3f, 0x1f, 0x17, 0x02, },
	{ 768000000, 808000000, 0x7f, 0x37, 0x5f, 0x37, 0x37, 0x3f, 0xc7, 0x3f, 0x1f, 0x17, 0x02, },
	{ 808000000, 832000000, 0x7f, 0x37, 0x5f, 0x3f, 0x37, 0x3f, 0xcf, 0x3f, 0x1f, 0x17, 0x02, },
	{ 832000000, 896000000, 0x7f, 0x37, 0x67, 0x3f, 0x3f, 0x47, 0xdf, 0x47, 0x1f, 0x17, 0x02, },
	{ 896000000, 936000000, 0x87, 0x3f, 0x67, 0x3f, 0x3f, 0x4f, 0xdf, 0x47, 0x1f, 0x17, 0x02, },
	{ 936000000, 960000000, 0x87, 0x3f, 0x6f, 0x47, 0x3f, 0x4f, 0xe7, 0x47, 0x1f, 0x17, 0x02, },
	{ 960000000, 1024000000, 0x87, 0x3f, 0x77, 0x47, 0x47, 0x57, 0xf7, 0x4f, 0x1f, 0x17, 0x02, },
	{ 1024000000, 1064000000, 0x8f, 0x47, 0x77, 0x47, 0x47, 0x57, 0xff, 0x4f, 0x1f, 0x17, 0x02, },
	{ 1064000000, 1088000000, 0x8f, 0x47, 0x7f, 0x4f, 0x47, 0x5f, 0xff, 0x4f, 0x1f, 0x17, 0x02, },
	{ 1088000000, 1120000000 + 1, 0x8f, 0x47, 0x87, 0x4f, 0x4f, 0x67, 0xff, 0x4f, 0x1f, 0x17, 0x02, },
	{ }, /* Guardian */
};

static int imx135_pre_streamon(struct smiapp_sensor *sensor)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->src->sd);
	struct smiapp_reg_8 regs[] = {
		{ 0x3008, 0xb0 },
		{ 0x320a, 0x01 },
		{ 0x320d, 0x10 },
		{ 0x3216, 0x2e },
		{ 0x3230, 0x0a },
		{ 0x3228, 0x05 },
		{ 0x3229, 0x02 },
		{ 0x322c, 0x02 },
		{ 0x3302, 0x10 },
		{ 0x3390, 0x45 },
		{ 0x3409, 0x0c },
		{ 0x340b, 0xf5 },
		{ 0x340c, 0x2d },
		{ 0x3412, 0x41 },
		{ 0x3413, 0xad },
		{ 0x3414, 0x1e },
		{ 0x3427, 0x04 },
		{ 0x3480, 0x1e },
		{ 0x3484, 0x1e },
		{ 0x3488, 0x1e },
		{ 0x348c, 0x1e },
		{ 0x3490, 0x1e },
		{ 0x3494, 0x1e },
		{ 0x349c, 0x38 },
		{ 0x34a3, 0x38 },
		{ 0x3511, 0x8f },
		{ 0x3518, 0x00 },
		{ 0x3519, 0x94 },
		{ 0x3833, 0x20 },
		{ 0x3893, 0x01 },
		{ 0x38c2, 0x08 },
		{ 0x38c3, 0x08 },
		{ 0x3c09, 0x01 },
		{ 0x4000, 0x0e },
		{ 0x4300, 0x00 },
		{ 0x4316, 0x12 },
		{ 0x4317, 0x22 },
		{ 0x4318, 0x00 },
		{ 0x4319, 0x00 },
		{ 0x431a, 0x00 },
		{ 0x4324, 0x03 },
		{ 0x4325, 0x20 },
		{ 0x4326, 0x03 },
		{ 0x4327, 0x84 },
		{ 0x4328, 0x03 },
		{ 0x4329, 0x20 },
		{ 0x432a, 0x03 },
		{ 0x432b, 0x84 },
		{ 0x432c, 0x01 },
		{ 0x4401, 0x3f },
		{ 0x4402, 0xff },
		{ 0x4412, 0x3f },
		{ 0x4413, 0xff },
		{ 0x441d, 0x28 },
		{ 0x4444, 0x00 },
		{ 0x4445, 0x00 },
		{ 0x4446, 0x3f },
		{ 0x4447, 0xff },
		{ 0x4452, 0x00 },
		{ 0x4453, 0xa0 },
		{ 0x4454, 0x08 },
		{ 0x4455, 0x00 },
		{ 0x4458, 0x18 },
		{ 0x4459, 0x18 },
		{ 0x445a, 0x3f },
		{ 0x445b, 0x3a },
		{ 0x4462, 0x00 },
		{ 0x4463, 0x00 },
		{ 0x4464, 0x00 },
		{ 0x4465, 0x00 },
		{ 0x446e, 0x01 },
		{ 0x4500, 0x1f },
		{ 0x600a, 0x00 },
		{ 0x380a, 0x00 },
		{ 0x380b, 0x00 },
		{ 0x4103, 0x00 },
		{ 0x4243, 0x9a },
		{ 0x4330, 0x01 },
		{ 0x4331, 0x90 },
		{ 0x4332, 0x02 },
		{ 0x4333, 0x58 },
		{ 0x4334, 0x03 },
		{ 0x4335, 0x20 },
		{ 0x4336, 0x03 },
		{ 0x4337, 0x84 },
		{ 0x433c, 0x01 },
		{ 0x4340, 0x02 },
		{ 0x4341, 0x58 },
		{ 0x4342, 0x03 },
		{ 0x4343, 0x52 },
		{ 0x4364, 0x0b },
		{ 0x4368, 0x00 },
		{ 0x4369, 0x0f },
		{ 0x436a, 0x03 },
		{ 0x436b, 0xa8 },
		{ 0x436c, 0x00 },
		{ 0x436d, 0x00 },
		{ 0x436e, 0x00 },
		{ 0x436f, 0x06 },
		{ 0x4281, 0x21 },
		{ 0x4282, 0x18 },
		{ 0x4283, 0x04 },
		{ 0x4284, 0x08 },
		{ 0x4287, 0x7f },
		{ 0x4288, 0x08 },
		{ 0x428c, 0x08 },
		{ 0x4297, 0x00 },
		{ 0x4299, 0x7e },
		{ 0x42a4, 0xfb },
		{ 0x42a5, 0x7e },
		{ 0x42a6, 0xdf },
		{ 0x42a7, 0xb7 },
		{ 0x42af, 0x03 },
		{ 0x4207, 0x03 },
		{ 0x4218, 0x00 },
		{ 0x421b, 0x20 },
		{ 0x421f, 0x04 },
		{ 0x4222, 0x02 },
		{ 0x4223, 0x22 },
		{ 0x422e, 0x54 },
		{ 0x422f, 0xfb },
		{ 0x4230, 0xff },
		{ 0x4231, 0xfe },
		{ 0x4232, 0xff },
		{ 0x4235, 0x58 },
		{ 0x4236, 0xf7 },
		{ 0x4237, 0xfd },
		{ 0x4239, 0x4e },
		{ 0x423a, 0xfc },
		{ 0x423b, 0xfd },
		{ 0x4300, 0x00 },
		{ 0x4316, 0x12 },
		{ 0x4317, 0x22 },
		{ 0x4318, 0x00 },
		{ 0x4319, 0x00 },
		{ 0x431a, 0x00 },
		{ 0x4324, 0x03 },
		{ 0x4325, 0x20 },
		{ 0x4326, 0x03 },
		{ 0x4327, 0x84 },
		{ 0x4328, 0x03 },
		{ 0x4329, 0x20 },
		{ 0x432a, 0x03 },
		{ 0x432b, 0x20 },
		{ 0x432c, 0x01 },
		{ 0x432d, 0x01 },
		{ 0x4338, 0x02 },
		{ 0x4339, 0x00 },
		{ 0x433a, 0x00 },
		{ 0x433b, 0x02 },
		{ 0x435a, 0x03 },
		{ 0x435b, 0x84 },
		{ 0x435e, 0x01 },
		{ 0x435f, 0xff },
		{ 0x4360, 0x01 },
		{ 0x4361, 0xf4 },
		{ 0x4362, 0x03 },
		{ 0x4363, 0x84 },
		{ 0x437b, 0x01 },
		{ 0x4400, 0x00 }, /* stats off isp do not support stats*/
		{ 0x4401, 0x3f },
		{ 0x4402, 0xff },
		{ 0x4404, 0x13 },
		{ 0x4405, 0x26 },
		{ 0x4406, 0x07 },
		{ 0x4408, 0x20 },
		{ 0x4409, 0xe5 },
		{ 0x440a, 0xfb },
		{ 0x440c, 0xf6 },
		{ 0x440d, 0xea },
		{ 0x440e, 0x20 },
		{ 0x4410, 0x00 },
		{ 0x4411, 0x00 },
		{ 0x4412, 0x3f },
		{ 0x4413, 0xff },
		{ 0x4414, 0x1f },
		{ 0x4415, 0xff },
		{ 0x4416, 0x20 },
		{ 0x4417, 0x00 },
		{ 0x4418, 0x1f },
		{ 0x4419, 0xff },
		{ 0x441a, 0x20 },
		{ 0x441b, 0x00 },
		{ 0x441d, 0x40 },
		{ 0x441e, 0x1e },
		{ 0x441f, 0x38 },
		{ 0x4420, 0x01 },
		{ 0x4444, 0x00 },
		{ 0x4445, 0x00 },
		{ 0x4446, 0x1d },
		{ 0x4447, 0xf9 },
		{ 0x4452, 0x00 },
		{ 0x4453, 0xa0 },
		{ 0x4454, 0x08 },
		{ 0x4455, 0x00 },
		{ 0x4456, 0x0f },
		{ 0x4457, 0xff },
		{ 0x4458, 0x18 },
		{ 0x4459, 0x18 },
		{ 0x445a, 0x3f },
		{ 0x445b, 0x3a },
		{ 0x445c, 0x00 },
		{ 0x445d, 0x28 },
		{ 0x445e, 0x01 },
		{ 0x445f, 0x90 },
		{ 0x4460, 0x00 },
		{ 0x4461, 0x60 },
		{ 0x4462, 0x00 },
		{ 0x4463, 0x00 },
		{ 0x4464, 0x00 },
		{ 0x4465, 0x00 },
		{ 0x446c, 0x00 },
		{ 0x446d, 0x00 },
		{ 0x446e, 0x00 },
		{ 0x452a, 0x02 },
		{ 0x0712, 0x01 },
		{ 0x0713, 0x00 },
		{ 0x0714, 0x01 },
		{ 0x0715, 0x00 },
		{ 0x0716, 0x01 },
		{ 0x0717, 0x00 },
		{ 0x0718, 0x01 },
		{ 0x0719, 0x00 },
		{ 0x4500, 0x1f },
		{ 0x0205, 0x00 },
		{ 0x020e, 0x01 },
		{ 0x020f, 0x00 },
		{ 0x0210, 0x02 },
		{ 0x0211, 0x00 },
		{ 0x0212, 0x02 },
		{ 0x0213, 0x00 },
		{ 0x0214, 0x01 },
		{ 0x0215, 0x00 },
		{ 0x0230, 0x00 }, /* hdr setting */
		{ 0x0231, 0x00 },
		{ 0x0233, 0x00 },
		{ 0x0234, 0x00 },
		{ 0x0235, 0x40 },
		{ 0x0238, 0x00 },
		{ 0x0239, 0x04 },
		{ 0x023b, 0x00 },
		{ 0x023c, 0x01 },
		{ 0x33b0, 0x04 },
		{ 0x33b1, 0x00 },
		{ 0x33b3, 0x00 },
		{ 0x33b4, 0x01 },
		{ 0x3800, 0x00 },
		{ 0x4203, 0xff },
		{ 0x7006, 0x04 },
		{ 0x301D, 0x30 },
		{ 0x4400, 0x00 },
	};
	struct imx135_timing_reg_set *tregs;
	u8 watermark_high, watermark_low;
	int rval;

	for (tregs = imx135_timing_reg_sets; tregs->op_sys_clk_freq_hz_min;
	     tregs++) {
		struct smiapp_reg_8 timing_regs[] = {
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TCLK_POST,
					      tregs->tclk_post),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_THS_PREPARE,
					      tregs->ths_prepare),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_THS_ZERO_MIN,
					      tregs->ths_zero_min),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_THS_TRAIL,
					      tregs->ths_trail),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TCLK_TRAIL_MIN,
					      tregs->tclk_trail_min),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TCLK_PREPARE,
					      tregs->tclk_prepare),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TCLK_ZERO,
					      tregs->tclk_zero),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TLPX,
					      tregs->tlpx),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TCLK_PRE,
					      tregs->tclk_pre),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TCLK_EXIT,
					      tregs->tclk_exit),
			SMIAPP_MK_QUIRK_REG_8(SMIAPP_IMX135_REG_U8_TLPXESC,
					      tregs->tlpxesc),
		};

		if (sensor->pll.op_sys_clk_freq_hz
		    < tregs->op_sys_clk_freq_hz_min ||
		    sensor->pll.op_sys_clk_freq_hz
		    > tregs->op_sys_clk_freq_hz_max)
			continue;

		dev_dbg(&client->dev,
			"csi timing configuration for %u--%u Hz\n",
			tregs->op_sys_clk_freq_hz_min,
			tregs->op_sys_clk_freq_hz_max);

		rval = smiapp_write_8s(sensor, timing_regs,
				       ARRAY_SIZE(timing_regs));
		if (rval < 0)
			return rval;

		break;
	}

	if (!tregs->op_sys_clk_freq_hz_min) {
		dev_err(&client->dev, "can't find valid csi timing values!\n");
		return -EINVAL;
	}

	rval = smiapp_write_8s(sensor, regs, ARRAY_SIZE(regs));
	if (rval < 0)
		return rval;

	rval = smiapp_write(sensor, SMIAPP_IMX135_REG_U16_SCL_BYPASS,
			    (sensor->binning_horizontal == 1 &&
			     sensor->binning_vertical == 1 &&
			     sensor->scaling_mode == 0) ?
			    SMIAPP_IMX135_U16_SCL_BYPASS_ENABLE : 0);
	if (rval)
		return rval;

	rval = smiapp_write(
		sensor, SMIAPP_IMX135_REG_U8_PLL_SOLO_DRIVE,
		SMIAPP_IMX135_U8_PLL_SOLO_DRIVE_SINGLE_PLL);
	if (rval)
		return rval;

	/*
	 * These mipi watermark registers are specific to the mode and x
	 * output size. If direct cropping is used without any binning or
	 * scaling, the value 0x0010 works. Otherwise this is specific to
	 * a particular output width configured.
	 *
	 * TODO: revisit this setting logic. Probably this should come from
	 * the user space component based on width it configures.
	 */
	if (sensor->binning_horizontal == 1 &&
	    sensor->binning_vertical == 1 &&
	    sensor->scaling_mode == 0) {
		watermark_high = 0x00;
		watermark_low = 0x10;
	} else if (sensor->src->crop[SMIAPP_PAD_SRC].width >= 4000) {
		watermark_high = 0x00;
		watermark_low = 0x10;
	} else if (sensor->src->crop[SMIAPP_PAD_SRC].width >= 3000) {
		watermark_high = 0x02;
		watermark_low = 0xa0;
	} else if (sensor->src->crop[SMIAPP_PAD_SRC].width == 2576) {
		watermark_high = 0x03;
		watermark_low = 0xae;
	} else if (sensor->src->crop[SMIAPP_PAD_SRC].width == 2336 ||
		   sensor->src->crop[SMIAPP_PAD_SRC].width == 2064) {
		watermark_high = 0x04;
		watermark_low = 0xe2;
	} else if (sensor->src->crop[SMIAPP_PAD_SRC].width >= 2000) {
		watermark_high = 0x03;
		watermark_low = 0xae;
	} else if (sensor->src->crop[SMIAPP_PAD_SRC].width == 1936 ||
		   sensor->src->crop[SMIAPP_PAD_SRC].width == 1296) {
		watermark_high = 0x04;
		watermark_low = 0xe2;
	} else {
		watermark_high = 0x04;
		watermark_low = 0xe2;
	}
	rval = smiapp_write(
		     sensor, SMIAPP_IMX135_REG_U8_MIPI_WMARK_H, watermark_high);
	if (rval)
		return rval;

	rval = smiapp_write(
		     sensor, SMIAPP_IMX135_REG_U8_MIPI_WMARK_L, watermark_low);
	if (rval)
		return rval;

	rval = smiapp_write(
		sensor, SMIAPP_IMX135_REG_U16_WRITE_HSIZE,
		sensor->src->crop[SMIAPP_PAD_SRC].width);
	if (rval)
		return rval;

	rval = smiapp_write(
		sensor, SMIAPP_IMX135_REG_U16_WRITE_VSIZE,
		sensor->src->crop[SMIAPP_PAD_SRC].height);
	if (rval)
		return rval;

	rval = smiapp_write(
		sensor, SMIAPP_IMX135_REG_U16_WRITE_SCALE_RSZ_H,
		sensor->scaling_mode == 0 ? 0 : sensor->scaler->compose.width);
	if (rval)
		return rval;

	rval = smiapp_write(
		sensor, SMIAPP_IMX135_REG_U16_WRITE_SCALE_RSZ_V,
		sensor->scaling_mode == 0 ? 0 : sensor->scaler->compose.height);
	if (rval)
		return rval;

	/* Op sys clock divisor is either 1 or 2. */
	return smiapp_write(sensor, SMIAPP_IMX135_REG_U8_DIV,
			    SMIAPP_IMX135_U8_DIV_CKDIV_MODE_MANUAL |
			    sensor->pll.op_sys_clk_div);
}

static int imx135_reg_access(struct smiapp_sensor *sensor, bool write, u32 *reg,
			     u32 *val)
{
	switch (*reg) {
	case SMIAPP_REG_U16_DATA_FORMAT_DESCRIPTOR(4):
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_CSI_LANE_MODE:
		*reg = SMIAPP_IMX135_REG_U8_CSI_LANE_MODE;
		break;
	case SMIAPP_REG_U16_EXTCLK_FREQUENCY_MHZ:
		*reg = SMIAPP_IMX135_REG_U16_EXTCLK_FREQUENCY_MHZ;
		break;
	case SMIAPP_REG_U16_VT_SYS_CLK_DIV:
		*reg = SMIAPP_IMX135_REG_U8_VT_SYS_CLK_DIV;
		break;
	case SMIAPP_REG_U16_VT_PIX_CLK_DIV:
		*reg = SMIAPP_IMX135_REG_U8_VT_PIX_CLK_DIV;
		break;
	case SMIAPP_REG_U16_OP_SYS_CLK_DIV:
		*reg = SMIAPP_IMX135_REG_U8_OP_SYS_CLK_DIV;
		break;
	case SMIAPP_REG_U16_OP_PIX_CLK_DIV:
		*reg = SMIAPP_IMX135_REG_U8_OP_PIX_CLK_DIV;
		break;
	case SMIAPP_REG_U16_PLL_MULTIPLIER:
		*reg = SMIAPP_IMX135_REG_U16_PLL_OP_MULTIPLIER;
		break;
	/*
	 * Cropping is performed after binning by this sensor, not
	 * before.
	 */
	case SMIAPP_REG_U16_DIGITAL_CROP_X_OFFSET:
		*reg = SMIAPP_IMX135_REG_U16_DIG_CROP_X_START;
		break;
	case SMIAPP_REG_U16_DIGITAL_CROP_Y_OFFSET:
		*reg = SMIAPP_IMX135_REG_U16_DIG_CROP_Y_START;
		break;
	case SMIAPP_REG_U16_DIGITAL_CROP_IMAGE_WIDTH:
		*reg = SMIAPP_IMX135_REG_U16_DIG_CROP_X_SIZE;
		break;
	case SMIAPP_REG_U16_DIGITAL_CROP_IMAGE_HEIGHT:
		*reg = SMIAPP_IMX135_REG_U16_DIG_CROP_Y_SIZE;
		break;
	case SMIAPP_REG_U8_BINNING_MODE:
		*reg = SMIAPP_IMX135_REG_U8_BINNING_MODE;
		break;
	case SMIAPP_REG_U8_BINNING_TYPE:
		*reg = SMIAPP_IMX135_REG_U8_BINNING_TYPE;
		break;
	case SMIAPP_REG_U16_TEST_PATTERN_MODE:
		*reg = SMIAPP_IMX135_REG_U16_TEST_PATTERN_MODE;
		break;
	case SMIAPP_REG_U16_TEST_DATA_RED:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_RED;
		break;
	case SMIAPP_REG_U16_TEST_DATA_GREENR:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_GREENR;
		break;
	case SMIAPP_REG_U16_TEST_DATA_BLUE:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_BLUE;
		break;
	case SMIAPP_REG_U16_TEST_DATA_GREENB:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_GREENB;
		break;
	case SMIAPP_REG_U32_REQUESTED_LINK_BIT_RATE_MBPS:
		/* No such register in this sensor */
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_BINNING_CAPABILITY:
		*val = SMIAPP_BINNING_CAPABILITY_YES;
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_BINNING_SUBTYPES:
		*val = 3;
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_BINNING_TYPE_n(0):
		*val = 0x11;
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_BINNING_TYPE_n(1):
		*val = 0x22;
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_BINNING_TYPE_n(2):
		*val = 0x44;
		return -ENOIOCTLCMD;
	}

	return 0;
}

static unsigned long imx135_pll_flags(struct smiapp_sensor *sensor)
{
	unsigned long flags = SMIAPP_PLL_FLAG_PIX_CLOCK_DOUBLE;

	if (sensor->platform_data->lanes == 4)
		flags |= SMIAPP_PLL_FLAG_OP_PIX_DIV_HALF;

	return flags;
}

const struct smiapp_quirk smiapp_imx135_quirk = {
	.limits = imx135_limits,
	.pre_streamon = imx135_pre_streamon,
	.pll_flags = imx135_pll_flags,
	.reg_access = imx135_reg_access,
};

static int imx132_reg_access(struct smiapp_sensor *sensor, bool write, u32 *reg,
			     u32 *val)
{
	switch (*reg) {
	case SMIAPP_REG_U16_DATA_FORMAT_DESCRIPTOR(4):
	case SMIAPP_REG_U8_FAST_STANDBY_CTRL:
	case SMIAPP_REG_U8_DPHY_CTRL:
	case SMIAPP_REG_U32_REQUESTED_LINK_BIT_RATE_MBPS:
	case SMIAPP_REG_U16_VT_PIX_CLK_DIV:
	case SMIAPP_REG_U16_OP_PIX_CLK_DIV:
	case SMIAPP_REG_U16_OP_SYS_CLK_DIV:
	case SMIAPP_REG_U16_DIGITAL_CROP_X_OFFSET:
	case SMIAPP_REG_U16_DIGITAL_CROP_Y_OFFSET:
	case SMIAPP_REG_U16_DIGITAL_CROP_IMAGE_WIDTH:
	case SMIAPP_REG_U16_DIGITAL_CROP_IMAGE_HEIGHT:
	case SMIAPP_REG_U16_SCALING_MODE:
	case SMIAPP_REG_U16_SCALE_M:
	case SMIAPP_REG_U8_BINNING_MODE:
	case SMIAPP_REG_U8_BINNING_TYPE:
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_BINNING_CAPABILITY:
		*val = SMIAPP_BINNING_CAPABILITY_YES;
		return -ENOIOCTLCMD;
	/*
	 * Restrict binning to 1x1 : No binning!
	 * TODO: Revisit when binning is enabled
	*/
	case SMIAPP_REG_U8_BINNING_SUBTYPES:
		*val = 1;
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U8_BINNING_TYPE_n(0):
		*val = 0x11;
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U16_VT_SYS_CLK_DIV:
		/* Set the register configuration based on the divider */
		switch (*val) {
		case 2:
			*val = 0;
			break;
		case 4:
			*val = 1;
			break;
		case 1:
		default:
			*val = 2;
			break;
		}
		*reg = SMIAPP_IMX132_REG_U8_POST_PLL_DIV;
		break;
	case SMIAPP_REG_U8_CSI_LANE_MODE:
		/*
		 * IMX132 with 2 lanes -> set register as 0x00
		 * IMX132 with 1 lane  -> set register as 0x01
		 * Ignore rest as imx132 has only maximum 2 lanes supported.
		 * The contents of "val" being passed here is
		 * "sensor->platform_data->lanes - 1"
		 */
		*val = (*val == 1) ? 0 : 1;
		*reg = SMIAPP_IMX132_REG_U8_CSI_LANES;
		break;
	case SMIAPP_REG_U16_EXTCLK_FREQUENCY_MHZ:
		/*
		 * When start streaming, the internal control circuit of imx132
		 * needs to wait for a settling time and it depends on the
		 * external clock. This setting time is 200 micro sec and is
		 * controlled by using PLL STAB TIMER register. This settling
		 * time is calculated using the following formula
		 *
		 * PLL settling time =
		 *      ((PLL_STAB_TIMER * 64) + 63) * 1 / EXTCLK[in MHz]
		 *
		 * PLL_STAB_TIMER = (200 * EXTCLK_MHz - 63) / 64
		 */
		*val = DIV_ROUND_UP(sensor->platform_data->ext_clk / 5000 - 63,
				    64);
		*reg = SMIAPP_IMX132_REG_U8_PLL_STAB_TIMER;
		break;
	/*
	 * Test pattern mode registers share same values across imx135, imx132
	 * and imx214. Hence the same imx135 definition is being used for these
	 * sensors.
	 */
	case SMIAPP_REG_U16_TEST_PATTERN_MODE:
		*reg = SMIAPP_IMX135_REG_U16_TEST_PATTERN_MODE;
		break;
	case SMIAPP_REG_U16_TEST_DATA_RED:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_RED;
		break;
	case SMIAPP_REG_U16_TEST_DATA_GREENR:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_GREENR;
		break;
	case SMIAPP_REG_U16_TEST_DATA_BLUE:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_BLUE;
		break;
	case SMIAPP_REG_U16_TEST_DATA_GREENB:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_GREENB;
		break;
	}

	return 0;
}

static int imx132_post_poweron(struct smiapp_sensor *sensor)
{
	struct smiapp_reg_8 regs[] = {
		{ 0x3087, 0x53 },
		{ 0x308B, 0x5A },
		{ 0x3094, 0x11 },
		{ 0x309D, 0xA4 },
		{ 0x30AA, 0x01 },
		{ 0x30C6, 0x00 },
		{ 0x30C7, 0x00 },
		{ 0x3118, 0x2F },
		{ 0x312A, 0x00 },
		{ 0x312B, 0x0B },
		{ 0x312C, 0x0B },
		{ 0x312D, 0x13 },
		{ 0x303D, 0x10 },
		{ 0x303E, 0x5A },
		{ 0x3040, 0x00 },
		{ 0x3041, 0x00 },
		{ 0x3048, 0x00 },
		{ 0x304C, 0x2F },
		{ 0x304D, 0x02 },
		{ 0x3064, 0x92 },
		{ 0x306A, 0x10 },
		{ 0x309B, 0x00 },
		{ 0x309E, 0x41 },
		{ 0x30A0, 0x10 },
		{ 0x30A1, 0x0B },
		{ 0x30B2, 0x00 },
		{ 0x30D5, 0x00 },
		{ 0x30D6, 0x00 },
		{ 0x30D7, 0x00 },
		{ 0x30D8, 0x00 },
		{ 0x30D9, 0x00 },
		{ 0x30DA, 0x00 },
		{ 0x30DB, 0x00 },
		{ 0x30DC, 0x00 },
		{ 0x30DD, 0x00 },
		{ 0x30DE, 0x00 },
		{ 0x3102, 0x0C },
		{ 0x3103, 0x33 },
		{ 0x3104, 0x18 },
		{ 0x3105, 0x00 },
		{ 0x3106, 0x65 },
		{ 0x3107, 0x00 },
		{ 0x3108, 0x06 },
		{ 0x3109, 0x04 },
		{ 0x310A, 0x04 },
		{ 0x315C, 0x3D },
		{ 0x315D, 0x3C },
		{ 0x316E, 0x3E },
		{ 0x316F, 0x3D },
		{ 0x3304, 0x07 },
		{ 0x3305, 0x06 },
		{ 0x3306, 0x19 },
		{ 0x3307, 0x03 },
		{ 0x3308, 0x0F },
		{ 0x3309, 0x07 },
		{ 0x330A, 0x0C },
		{ 0x330B, 0x06 },
		{ 0x330C, 0x0B },
		{ 0x330D, 0x07 },
		{ 0x330E, 0x03 },
		{ 0x3318, 0x62 },
		{ 0x3322, 0x09 },
		{ 0x3342, 0x00 },
		{ 0x3348, 0xE0 },
	};
	int rval;

	rval = smiapp_write_8s(sensor, regs, ARRAY_SIZE(regs));
	if (rval)
		return rval;

	rval = smiapp_write(sensor, SMIAPP_REG_U16_DIGITAL_GAIN_GREENR, 0x0100);
	if (rval)
		return rval;

	rval = smiapp_write(sensor, SMIAPP_REG_U16_DIGITAL_GAIN_RED, 0x01a0);
	if (rval)
		return rval;

	rval = smiapp_write(sensor, SMIAPP_REG_U16_DIGITAL_GAIN_BLUE, 0x0200);
	if (rval)
		return rval;

	return smiapp_write(sensor, SMIAPP_REG_U16_DIGITAL_GAIN_GREENB, 0x0100);
}

static int imx132_limits(struct smiapp_sensor *sensor)
{
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_PLL_OP_FREQ_HZ,
			     1000000000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_VT_SYS_CLK_DIV, 1);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_VT_SYS_CLK_DIV, 4);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_VT_SYS_CLK_FREQ_HZ,
			     384000000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_VT_SYS_CLK_FREQ_HZ,
			     806400000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_VT_PIX_CLK_DIV, 8);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_VT_PIX_CLK_DIV, 10);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_VT_PIX_CLK_FREQ_HZ,
			     53760000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_VT_PIX_CLK_FREQ_HZ,
			     80640000);
	return 0;
}

static unsigned long imx132_pll_flags(struct smiapp_sensor *sensor)
{
	return SMIAPP_PLL_FLAG_VT_PIX_CLOCK_PER_LANE
	       | SMIAPP_PLL_FLAG_OP_PIX_CLOCK_PER_LANE;
}

const struct smiapp_quirk smiapp_imx132_quirk = {
	.flags = imx132_pll_flags,
	.limits = imx132_limits,
	.post_poweron = imx132_post_poweron,
	.reg_access = imx132_reg_access,
};

static int imx214_post_poweron(struct smiapp_sensor *sensor)
{
	struct smiapp_reg_8 regs[] = {
		{ 0x4601, 0x00 },
		{ 0x4642, 0x05 },
		{ 0x6276, 0x00 },
		{ 0x900E, 0x06 },
		{ 0xA802, 0x90 },
		{ 0xA803, 0x11 },
		{ 0xA804, 0x62 },
		{ 0xA805, 0x77 },
		{ 0xA806, 0xAE },
		{ 0xA807, 0x34 },
		{ 0xA808, 0xAE },
		{ 0xA809, 0x35 },
		{ 0xA80A, 0x62 },
		{ 0xA80B, 0x83 },
		{ 0xAE33, 0x00 },
		{ 0x4174, 0x00 },
		{ 0x4175, 0x11 },
		{ 0x4612, 0x29 },
		{ 0x461B, 0x12 },
		{ 0x461F, 0x06 },
		{ 0x4635, 0x07 },
		{ 0x4637, 0x30 },
		{ 0x463F, 0x18 },
		{ 0x4641, 0x0D },
		{ 0x465B, 0x12 },
		{ 0x465F, 0x11 },
		{ 0x4663, 0x11 },
		{ 0x4667, 0x0F },
		{ 0x466F, 0x0F },
		{ 0x470E, 0x09 },
		{ 0x4909, 0xAB },
		{ 0x490B, 0x95 },
		{ 0x4915, 0x5D },
		{ 0x4A5F, 0xFF },
		{ 0x4A61, 0xFF },
		{ 0x4A73, 0x62 },
		{ 0x4A85, 0x00 },
		{ 0x4A87, 0xFF },
		{ 0x583C, 0x04 },
		{ 0x620E, 0x04 },
		{ 0x6EB2, 0x01 },
		{ 0x6EB3, 0x00 },
		{ 0x9300, 0x02 },
		{ 0x5041, 0x00 }, /* No Embedded data lines */
		{ 0x0220, 0x00 }, /* HDR Mode off */
		{ 0x0221, 0x11 }, /* HDR Mode off */
		{ 0x0222, 0x01 }, /* HDR Mode off */
		{ 0x3000, 0x35 }, /* HDR Mode off */
		{ 0x3a02, 0xff }, /* HDR Mode off */
		{ 0x3054, 0x01 },
		{ 0x305c, 0x11 },
		{ 0x0b06, 0x01 }, /* DEF CORR EN */
		{ 0x30a2, 0x00 },
		{ 0x30b4, 0x00 },
		{ 0x3013, 0x00 }, /* STATS off */
		{ 0x0204, 0x00 }, /* Initial gain values */
		{ 0x0205, 0x00 },
		{ 0x020E, 0x01 },
		{ 0x020F, 0x00 },
		{ 0x0210, 0x01 },
		{ 0x0211, 0x00 },
		{ 0x0212, 0x01 },
		{ 0x0213, 0x00 },
		{ 0x0214, 0x01 },
		{ 0x0215, 0x00 },
		{ 0x0216, 0x00 },
		{ 0x0217, 0x00 },
		{ 0x4170, 0x00 }, /* Analogue setting */
		{ 0x4171, 0x10 }, /* Analogue setting */
		{ 0x4176, 0x00 }, /* Analogue setting */
		{ 0x4177, 0x3C }, /* Analogue setting */
		{ 0xAE20, 0x04 }, /* Analogue setting */
		{ 0xAE21, 0x5C }, /* Analogue setting */
		{ 0x0310, 0x00 }, /* PLL_MULT_DRV to single */
	};

	return smiapp_write_8s(sensor, regs, ARRAY_SIZE(regs));
}

static int imx214_pre_streamon(struct smiapp_sensor *sensor)
{
	/*
	 * MIPI watermark registers. These depends on many factors such as
	 * output size, PLL INCK etc. and must be provided by the vendor.
	 *
	 * TODO: This must come from the user space. Handle this accordingly
	 */
	struct smiapp_reg_8 wmark_regs[] = {
		{ 0x3a03, 0x09 },
		{ 0x3a04, 0x20 },
		{ 0x3a05, 0x01 },
	};

	return smiapp_write_8s(sensor, wmark_regs, ARRAY_SIZE(wmark_regs));
}

static int imx214_limits(struct smiapp_sensor *sensor)
{
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_PRE_PLL_CLK_DIV, 8);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_PLL_IP_FREQ_HZ, 1000000);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MIN_PLL_MULTIPLIER, 12);
	smiapp_replace_limit(sensor, SMIAPP_LIMIT_MAX_PLL_MULTIPLIER, 1200);
	return 0;
}

static unsigned long imx214_pll_flags(struct smiapp_sensor *sensor)
{
	return SMIAPP_PLL_FLAG_VT_PIX_CLOCK_PER_LANE
		| SMIAPP_PLL_FLAG_OP_PIX_CLOCK_PER_LANE;
}

static int imx214_reg_access(struct smiapp_sensor *sensor, bool write, u32 *reg,
			     u32 *val)
{
	switch (*reg) {
	case SMIAPP_REG_U16_DATA_FORMAT_DESCRIPTOR(4):
		return -ENOIOCTLCMD;
	case SMIAPP_REG_U32_REQUESTED_LINK_BIT_RATE_MBPS:
		*val = DIV_ROUND_UP(
		  sensor->pll.op_sys_clk_freq_hz * sensor->platform_data->lanes,
		  1000000 / 256 / 256);
		break;
	/*
	 * Test pattern mode registers share same values across imx135, imx132
	 * and imx214. Hence the same imx135 definition is being used for these
	 * sensors.
	 */
	case SMIAPP_REG_U16_TEST_PATTERN_MODE:
		*reg = SMIAPP_IMX135_REG_U16_TEST_PATTERN_MODE;
		break;
	case SMIAPP_REG_U16_TEST_DATA_RED:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_RED;
		break;
	case SMIAPP_REG_U16_TEST_DATA_GREENR:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_GREENR;
		break;
	case SMIAPP_REG_U16_TEST_DATA_BLUE:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_BLUE;
		break;
	case SMIAPP_REG_U16_TEST_DATA_GREENB:
		*reg = SMIAPP_IMX135_REG_U16_TEST_DATA_GREENB;
		break;
	}

	return 0;
}

const struct smiapp_quirk smiapp_imx214_quirk = {
	.limits = imx214_limits,
	.pll_flags = imx214_pll_flags,
	.reg_access = imx214_reg_access,
	.post_poweron = imx214_post_poweron,
	.pre_streamon = imx214_pre_streamon,
};
