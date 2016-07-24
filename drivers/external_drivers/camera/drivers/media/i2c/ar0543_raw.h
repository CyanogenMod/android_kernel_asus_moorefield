/*
 * Support for Aptina AR0543_RAW camera sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __AR0543_RAW_H__
#define __AR0543_RAW_H__
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <linux/types.h>
#include <media/media-entity.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>

/* ASUS_BSP++ */
#include <linux/HWVersion.h>
extern int Read_HW_ID(void);
/* static unsigned int HW_ID = 0xFF; */
/* ASUS_BSP-- */

#define AR0543_RAW_RES_WIDTH_MAX	2592
#define AR0543_RAW_RES_HEIGHT_MAX	1944
#define MAX_FMTS 1

#define	AR0543_RAW_NAME	"ar0543_raw"
#define	AR0543_RAW_ADDR	0x36
#define AR0543_RAW_ID	0x4800
#define AR0543_RAW_ID2	0x4b01

#define	LAST_REG_SETING		{0xffff, 0xff}
#define	is_last_reg_setting(item) ((item).reg == 0xffff)
#define I2C_MSG_LENGTH		0x2

#define AR0543_RAW_INVALID_CONFIG	0xffffffff

#define AR0543_RAW_MAX_FOCUS_POS	255
#define AR0543_RAW_MAX_FOCUS_NEG	(-255)

#define AR0543_RAW_INTG_UNIT_US	100
#define AR0543_RAW_MCLK		192

#define AR0543_RAW_REG_BITS	16
#define AR0543_RAW_REG_MASK	0xFFFF

/* This should be added into include/linux/videodev2.h */
#ifndef V4L2_IDENT_AR0543_RAW
#define V4L2_IDENT_AR0543_RAW	8245
#endif

/*
 * ar0543_raw System control registers
 */
#define AR0543_RAW_SC_CMMN_CHIP_ID                 0x0000
#define AR0543_RAW_SC_CMMN_REV_ID		        0x0002

#define GROUPED_PARAMETER_UPDATE		0x0000
#define GROUPED_PARAMETER_HOLD			0x0100
#define AR0543_RAW_GROUPED_PARAMETER_HOLD		0x0104

#define AR0543_RAW_VT_PIX_CLK_DIV			0x0300
#define AR0543_RAW_VT_SYS_CLK_DIV			0x0302
#define AR0543_RAW_PRE_PLL_CLK_DIV			0x0304
#define AR0543_RAW_PLL_MULTIPLIER			0x0306
#define AR0543_RAW_OP_PIX_DIV			0x0308
#define AR0543_RAW_OP_SYS_DIV			0x030A
#define AR0543_RAW_FRAME_LENGTH_LINES		0x0340
#define AR0543_RAW_LINE_LENGTH_PCK			0x0342
#define AR0543_RAW_COARSE_INTG_TIME_MIN		0x1004
#define AR0543_RAW_COARSE_INTG_TIME_MAX		0x1006
#define AR0543_RAW_FINE_INTG_TIME_MIN		0x1008
#define AR0543_RAW_FINE_INTG_MIN_DEF		0x4FE
#define AR0543_RAW_FINE_INTG_TIME_MAX		0x100A
#define AR0543_RAW_FINE_INTG_MAX_DEF		0x3EE

#define AR0543_RAW_READ_MODE				0x3040
#define AR0543_RAW_READ_MODE_X_ODD_INC		(BIT(6) | BIT(7) | BIT(8))
#define AR0543_RAW_READ_MODE_Y_ODD_INC		(BIT(0) | BIT(1) | BIT(2) |\
						BIT(3) | BIT(4) | BIT(5))

#define AR0543_RAW_HORIZONTAL_START_H		0x0344
#define AR0543_RAW_VERTICAL_START_H		0x0346
#define AR0543_RAW_HORIZONTAL_END_H		0x0348
#define AR0543_RAW_VERTICAL_END_H			0x034a
#define AR0543_RAW_HORIZONTAL_OUTPUT_SIZE_H	0x034c
#define AR0543_RAW_VERTICAL_OUTPUT_SIZE_H		0x034e

#define AR0543_RAW_COARSE_INTEGRATION_TIME		0x3012
#define AR0543_RAW_FINE_INTEGRATION_TIME		0x3014
#define AR0543_RAW_ROW_SPEED			0x3016
#define AR0543_RAW_GLOBAL_GAIN			0x305e
#define AR0543_RAW_GLOBAL_GAIN_WR			0x1000
#define AR0543_RAW_TEST_PATTERN_MODE		0x3070
#define AR0543_RAW_VCM_SLEW_STEP			0x30F0
#define AR0543_RAW_VCM_SLEW_STEP_MAX		0x7
#define AR0543_RAW_VCM_SLEW_STEP_MASK		0x7
#define AR0543_RAW_VCM_CODE			0x30F2
#define AR0543_RAW_VCM_SLEW_TIME			0x30F4
#define AR0543_RAW_VCM_SLEW_TIME_MAX		0xffff
#define AR0543_RAW_VCM_ENABLE			0x8000

/* ar0543_raw SCCB */
#define AR0543_RAW_SCCB_CTRL			0x3100
#define AR0543_RAW_AEC_PK_EXPO_H			0x3500
#define AR0543_RAW_AEC_PK_EXPO_M			0x3501
#define AR0543_RAW_AEC_PK_EXPO_L			0x3502
#define AR0543_RAW_AEC_MANUAL_CTRL			0x3503
#define AR0543_RAW_AGC_ADJ_H			0x3508
#define AR0543_RAW_AGC_ADJ_L			0x3509

#define AR0543_RAW_FOCAL_LENGTH_NUM	439	/*4.39mm*/
#define AR0543_RAW_FOCAL_LENGTH_DEM	100
#define AR0543_RAW_F_NUMBER_DEFAULT_NUM	24
#define AR0543_RAW_F_NUMBER_DEM	10

#define AR0543_RAW_X_ADDR_MIN	0X1180
#define AR0543_RAW_Y_ADDR_MIN	0X1182
#define AR0543_RAW_X_ADDR_MAX	0X1184
#define AR0543_RAW_Y_ADDR_MAX	0X1186

#define AR0543_RAW_MIN_FRAME_LENGTH_LINES	0x1140
#define AR0543_RAW_MAX_FRAME_LENGTH_LINES	0x1142
#define AR0543_RAW_MIN_LINE_LENGTH_PCK	0x1144
#define AR0543_RAW_MAX_LINE_LENGTH_PCK	0x1146
#define AR0543_RAW_MIN_LINE_BLANKING_PCK	0x1148
#define AR0543_RAW_MIN_FRAME_BLANKING_LINES 0x114A
#define AR0543_RAW_X_OUTPUT_SIZE	0x034C
#define AR0543_RAW_Y_OUTPUT_SIZE	0x034E

#define AR0543_RAW_BIN_FACTOR_MAX			3

/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define AR0543_RAW_FOCAL_LENGTH_DEFAULT 0x1B70064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define AR0543_RAW_F_NUMBER_DEFAULT 0x18000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define AR0543_RAW_F_NUMBER_RANGE 0x180a180a

/* Defines for register writes and register array processing */
#define AR0543_RAW_BYTE_MAX	30
#define AR0543_RAW_SHORT_MAX	16
#define I2C_RETRY_COUNT		5
#define AR0543_RAW_TOK_MASK	0xfff0

#define	AR0543_RAW_STATUS_POWER_DOWN	0x0
#define	AR0543_RAW_STATUS_STANDBY		0x2
#define	AR0543_RAW_STATUS_ACTIVE		0x3
#define	AR0543_RAW_STATUS_VIEWFINDER	0x4

/* ASUS_BSP Wesley, for vcm test */
#define VCM_ADDR           0x0c
/* #define VCM_CODE_MSB       0x03 */
/* #define VCM_CODE_LSB       0x04 */
#define VCM_MAX_FOCUS_POS  1023

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

enum ar0543_raw_tok_type {
	AR0543_RAW_8BIT  = 0x0001,
	AR0543_RAW_16BIT = 0x0002,
	AR0543_RAW_RMW   = 0x0010,
	AR0543_RAW_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	AR0543_RAW_TOK_DELAY  = 0xfe00	/* delay token for reg list */
};

/*
 * If register address or register width is not 32 bit width,
 * user needs to convert it manually
 */

struct s_register_setting {
	u32 reg;
	u32 val;
};

struct s_output_format {
	struct v4l2_format v4l2_fmt;
	int fps;
};

/**
 * struct ar0543_raw_fwreg - Firmware burst command
 * @type: FW burst or 8/16 bit register
 * @addr: 16-bit offset to register or other values depending on type
 * @val: data value for burst (or other commands)
 *
 * Define a structure for sensor register initialization values
 */
struct ar0543_raw_fwreg {
	enum ar0543_raw_tok_type type; /* value, register or FW burst string */
	u16 addr;	/* target address */
	u32 val[8];
};

/**
 * struct ar0543_raw_reg - MI sensor  register format
 * @type: type of the register
 * @reg: 16-bit offset to register
 * @val: 8/16/32-bit register value
 *
 * Define a structure for sensor register initialization values
 */
struct ar0543_raw_reg {
	enum ar0543_raw_tok_type type;
	union {
		u16 sreg;
		struct ar0543_raw_fwreg *fwreg;
	} reg;
	u32 val;	/* @set value for read/mod/write, @mask */
	u32 val2;	/* optional: for rmw, OR mask */
};

/* Store macro values' debug names */
struct macro_string {
	u8 val;
	char *string;
};

static inline const char *
macro_to_string(const struct macro_string *array, int size, u8 val)
{
	int i;
	for (i = 0; i < size; i++) {
		if (array[i].val == val)
			return array[i].string;
	}
	return "Unknown VAL";
}

struct ar0543_raw_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

struct ar0543_raw_resolution {
	u8 *desc;
	int res;
	int width;
	int height;
	int fps;
	bool used;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	const struct ar0543_raw_reg *regs;
	u8 bin_factor_x;
	u8 bin_factor_y;
	unsigned short skip_frames;
};

struct ar0543_raw_format {
	u8 *desc;
	u32 pixelformat;
	struct s_register_setting *regs;
};

#define AR0543_DEFAULT_AF_10CM	448
#define AR0543_DEFAULT_AF_INF	256
#define AR0543_DEFAULT_AF_START	150
#define	AR0543_DEFAULT_AF_END	700

struct ar0543_raw_af_data {
	u16 af_inf_pos;
	u16 af_1m_pos;
	u16 af_10cm_pos;
	u16 af_start_curr;
	u8 module_id;
	u8 vendor_id;
	u16 default_af_inf_pos;
	u16 default_af_10cm_pos;
	u16 default_af_start;
	u16 default_af_end;
};

#define AR0543_RAW_FUSEID_SIZE		8
#define AR0543_RAW_FUSEID_START_ADDR	0x31f4

/* ar0543_raw device structure */
struct ar0543_raw_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;

	struct camera_sensor_platform_data *platform_data;
	int fmt_idx;
	int status;
	int streaming;
	int power;
	u16 sensor_id;
	u8 sensor_revision;
	u16 coarse_itg;
	u16 fine_itg;
	u16 gain;
	u32 focus;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	int run_mode;
	struct timespec timestamp_t_focus_abs;
	s16 number_of_steps;
	struct mutex input_lock; /* serialize sensor's ioctl */
	void *otp_data;
	struct ar0543_raw_af_data af_data;
	void *fuseid;
	/* Older VCMs could not maintain the focus position in standby mode. */
	bool keeps_focus_pos;

	struct attribute_group sensor_i2c_attribute; /* Add for ATD command+++*/
};

#define AR0543_RAW_MAX_WRITE_BUF_SIZE	32
struct ar0543_raw_write_buffer {
	u16 addr;
	u8 data[AR0543_RAW_MAX_WRITE_BUF_SIZE];
};

struct ar0543_raw_write_ctrl {
	int index;
	struct ar0543_raw_write_buffer buffer;
};


#define AR0543_RAW_OTP_READ_EN               0x301A
#define AR0543_RAW_OTP_TIMING                0x3134
#define AR0543_RAW_OTP_RECORD_TYPE           0x304C
#define AR0543_RAW_OTP_AUTO_READ             0x304A
#define AR0543_RAW_OTP_READY_REG_DONE       (1 << 5)
#define AR0543_RAW_OTP_READY_REG_OK         (1 << 6)

#define AR0543_RAW_OTP_AF_INF_POS_H         0x3800
#define AR0543_RAW_OTP_AF_INF_POS_L         0x3801
#define AR0543_RAW_OTP_AF_1M_POS            0x3802
#define AR0543_RAW_OTP_AF_10CM_POS          0x3804
#define AR0543_RAW_OTP_AF_START_CURR        0x3806
#define AR0543_RAW_OTP_AF_MODULE_ID         0x3808
#define AR0543_RAW_OTP_AF_VENDOR_ID         0x3809

#define AR0543_RAW_OTP_DATA_SIZE		0x60

#define GROUPED_PARAMETER_HOLD_ENABLE	{AR0543_RAW_8BIT, {0x0104}, 0x1}

#define GROUPED_PARAMETER_HOLD_DISABLE	{AR0543_RAW_8BIT, {0x0104}, 0x0}

#define INIT_VCM_CONTROL {AR0543_RAW_16BIT, {0x30F0}, 0x800C} /*slew_rate[2:0]*/
static const struct ar0543_raw_reg ar0543_raw_init_vcm[] = {
	INIT_VCM_CONTROL,				   /* VCM_CONTROL */
	{AR0543_RAW_16BIT, {0x30F2}, 0x0000}, /* VCM_NEW_CODE */
	{AR0543_RAW_16BIT, {0x30F4}, 0x0080}, /* VCM_STEP_TIME */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

/* #define RESET_REGISTER	{AR0543_RAW_16BIT, {0x301A}, 0x4A38} */
static const struct ar0543_raw_reg ar0543_raw_reset_register[] = {
	{AR0543_RAW_8BIT, {0x0103}, 0x01},
	{AR0543_RAW_TOK_DELAY, {0}, 50},     /* DELAY=50//Initialization Time */

	/* stop_streaming */
	{AR0543_RAW_8BIT, {0x0100}, 0x00},
	/* dual_lane_MIPI_interface */
	{AR0543_RAW_16BIT, {0x301A}, 0x0618},
	{AR0543_RAW_16BIT, {0x3064}, 0xB800},
	{AR0543_RAW_16BIT, {0x31AE}, 0x0202},
	{AR0543_RAW_16BIT, {0x0112}, 0x0A0A},

	/* REV1_recommended_settings */
	{AR0543_RAW_16BIT, {0x316A}, 0x8400},
	{AR0543_RAW_16BIT, {0x316C}, 0x8400},
	{AR0543_RAW_16BIT, {0x316E}, 0x8400},
	{AR0543_RAW_16BIT, {0x3EFA}, 0x1A1F},
	{AR0543_RAW_16BIT, {0x3ED2}, 0xD965},
	{AR0543_RAW_16BIT, {0x3ED8}, 0x7F1B},
	{AR0543_RAW_16BIT, {0x3EDA}, 0x2F11},
	{AR0543_RAW_16BIT, {0x3EE2}, 0x0060},
	{AR0543_RAW_16BIT, {0x3EF2}, 0xD965},
	{AR0543_RAW_16BIT, {0x3EF8}, 0x797F},
	{AR0543_RAW_16BIT, {0x3EFC}, 0x286F},
	{AR0543_RAW_16BIT, {0x3EFE}, 0x2C01},

	/* REV1_pixel_timing */
	{AR0543_RAW_16BIT, {0x3E00}, 0x042F},
	{AR0543_RAW_16BIT, {0x3E02}, 0xFFFF},
	{AR0543_RAW_16BIT, {0x3E04}, 0xFFFF},
	{AR0543_RAW_16BIT, {0x3E06}, 0xFFFF},
	{AR0543_RAW_16BIT, {0x3E08}, 0x8071},
	{AR0543_RAW_16BIT, {0x3E0A}, 0x7281},
	{AR0543_RAW_16BIT, {0x3E0C}, 0x4011},
	{AR0543_RAW_16BIT, {0x3E0E}, 0x8010},
	{AR0543_RAW_16BIT, {0x3E10}, 0x60A5},
	{AR0543_RAW_16BIT, {0x3E12}, 0x4080},
	{AR0543_RAW_16BIT, {0x3E14}, 0x4180},
	{AR0543_RAW_16BIT, {0x3E16}, 0x0018},
	{AR0543_RAW_16BIT, {0x3E18}, 0x46B7},
	{AR0543_RAW_16BIT, {0x3E1A}, 0x4994},
	{AR0543_RAW_16BIT, {0x3E1C}, 0x4997},
	{AR0543_RAW_16BIT, {0x3E1E}, 0x4682},
	{AR0543_RAW_16BIT, {0x3E20}, 0x0018},
	{AR0543_RAW_16BIT, {0x3E22}, 0x4241},
	{AR0543_RAW_16BIT, {0x3E24}, 0x8000},
	{AR0543_RAW_16BIT, {0x3E26}, 0x1880},
	{AR0543_RAW_16BIT, {0x3E28}, 0x4785},
	{AR0543_RAW_16BIT, {0x3E2A}, 0x4992},
	{AR0543_RAW_16BIT, {0x3E2C}, 0x4997},
	{AR0543_RAW_16BIT, {0x3E2E}, 0x4780},
	{AR0543_RAW_16BIT, {0x3E30}, 0x4D80},
	{AR0543_RAW_16BIT, {0x3E32}, 0x100C},
	{AR0543_RAW_16BIT, {0x3E34}, 0x8000},
	{AR0543_RAW_16BIT, {0x3E36}, 0x184A},
	{AR0543_RAW_16BIT, {0x3E38}, 0x8042},
	{AR0543_RAW_16BIT, {0x3E3A}, 0x001A},
	{AR0543_RAW_16BIT, {0x3E3C}, 0x9610},
	{AR0543_RAW_16BIT, {0x3E3E}, 0x0C80},
	{AR0543_RAW_16BIT, {0x3E40}, 0x4DC6},
	{AR0543_RAW_16BIT, {0x3E42}, 0x4A80},
	{AR0543_RAW_16BIT, {0x3E44}, 0x0018},
	{AR0543_RAW_16BIT, {0x3E46}, 0x8042},
	{AR0543_RAW_16BIT, {0x3E48}, 0x8041},
	{AR0543_RAW_16BIT, {0x3E4A}, 0x0018},
	{AR0543_RAW_16BIT, {0x3E4C}, 0x804B},
	{AR0543_RAW_16BIT, {0x3E4E}, 0xB74B},
	{AR0543_RAW_16BIT, {0x3E50}, 0x8010},
	{AR0543_RAW_16BIT, {0x3E52}, 0x6056},
	{AR0543_RAW_16BIT, {0x3E54}, 0x001C},
	{AR0543_RAW_16BIT, {0x3E56}, 0x8211},
	{AR0543_RAW_16BIT, {0x3E58}, 0x8056},
	{AR0543_RAW_16BIT, {0x3E5A}, 0x827C},
	{AR0543_RAW_16BIT, {0x3E5C}, 0x0970},
	{AR0543_RAW_16BIT, {0x3E5E}, 0x8082},
	{AR0543_RAW_16BIT, {0x3E60}, 0x7281},
	{AR0543_RAW_16BIT, {0x3E62}, 0x4C40},
	{AR0543_RAW_16BIT, {0x3E64}, 0x8E4D},
	{AR0543_RAW_16BIT, {0x3E66}, 0x8110},
	{AR0543_RAW_16BIT, {0x3E68}, 0x0CAF},
	{AR0543_RAW_16BIT, {0x3E6A}, 0x4D80},
	{AR0543_RAW_16BIT, {0x3E6C}, 0x100C},
	{AR0543_RAW_16BIT, {0x3E6E}, 0x8440},
	{AR0543_RAW_16BIT, {0x3E70}, 0x4C81},
	{AR0543_RAW_16BIT, {0x3E72}, 0x7C5F},
	{AR0543_RAW_16BIT, {0x3E74}, 0x7000},
	{AR0543_RAW_16BIT, {0x3E76}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E78}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E7A}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E7C}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E7E}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E80}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E82}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E84}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E86}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E88}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E8A}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E8C}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E8E}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E90}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E92}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E94}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E96}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E98}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E9A}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E9C}, 0x0000},
	{AR0543_RAW_16BIT, {0x3E9E}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EA0}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EA2}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EA4}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EA6}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EA8}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EAA}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EAC}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EAE}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EB0}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EB2}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EB4}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EB6}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EB8}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EBA}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EBC}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EBE}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EC0}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EC2}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EC4}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EC6}, 0x0000},
	{AR0543_RAW_16BIT, {0x3EC8}, 0x0000},
	{AR0543_RAW_16BIT, {0x3ECA}, 0x0000},
	{AR0543_RAW_16BIT, {0x3170}, 0x2150},
	{AR0543_RAW_16BIT, {0x317A}, 0x0150},
	{AR0543_RAW_16BIT, {0x3ECC}, 0x2200},
	{AR0543_RAW_16BIT, {0x3174}, 0x0000},
	{AR0543_RAW_16BIT, {0x3176}, 0x0000},
	{AR0543_RAW_16BIT, {0x30BC}, 0x0384},
	{AR0543_RAW_16BIT, {0x30C0}, 0x1220},
	{AR0543_RAW_16BIT, {0x30D4}, 0x9200},
	{AR0543_RAW_16BIT, {0x30B2}, 0xC000},

	{AR0543_RAW_16BIT, {0x31B0}, 0x00C4},
	{AR0543_RAW_16BIT, {0x31B2}, 0x0064},


	{AR0543_RAW_16BIT, {0x31B4}, 0x0E77},
	{AR0543_RAW_16BIT, {0x31B6}, 0x0D24},
	{AR0543_RAW_16BIT, {0x31B8}, 0x020E},
	{AR0543_RAW_16BIT, {0x31BA}, 0x0710},
	{AR0543_RAW_16BIT, {0x31BC}, 0x2A0D},
	{AR0543_RAW_16BIT, {0x31BE}, 0xC003},

	/* updated June 2013--ADACD and 2DDC settings */
	/* ADACD: low gain */
#ifdef CONFIG_ME175CG
	{AR0543_RAW_16BIT, {0x3100}, 0x0002},
#else
	{AR0543_RAW_16BIT, {0x3100}, 0x0000},
#endif
	{AR0543_RAW_16BIT, {0x3102}, 0x0064},
	{AR0543_RAW_16BIT, {0x3104}, 0x0B6D},
	{AR0543_RAW_16BIT, {0x3106}, 0x0201},
	{AR0543_RAW_16BIT, {0x3108}, 0x0905},
	{AR0543_RAW_16BIT, {0x310A}, 0x002A},
	/* 2DDC: low gain */
	{AR0543_RAW_16BIT, {0x31E0}, 0x1F01},
	{AR0543_RAW_16BIT, {0x3F02}, 0x0001},
	{AR0543_RAW_16BIT, {0x3F04}, 0x0032},
	{AR0543_RAW_16BIT, {0x3F06}, 0x015E},
	{AR0543_RAW_16BIT, {0x3F08}, 0x0190},

	{AR0543_RAW_16BIT, {0x305E}, 0x1127},

	/* Smaller_FallOff60+++ */
	{AR0543_RAW_16BIT, {0x3780}, 0x0000},
	{AR0543_RAW_16BIT, {0x3600}, 0x0110},
	{AR0543_RAW_16BIT, {0x3602}, 0x1F89},
	{AR0543_RAW_16BIT, {0x3604}, 0x02D1},
	{AR0543_RAW_16BIT, {0x3606}, 0xC2CD},
	{AR0543_RAW_16BIT, {0x3608}, 0xACB1},
	{AR0543_RAW_16BIT, {0x360A}, 0x02B0},
	{AR0543_RAW_16BIT, {0x360C}, 0x096C},
	{AR0543_RAW_16BIT, {0x360E}, 0x5870},
	{AR0543_RAW_16BIT, {0x3610}, 0x80EC},
	{AR0543_RAW_16BIT, {0x3612}, 0xC111},
	{AR0543_RAW_16BIT, {0x3614}, 0x0250},
	{AR0543_RAW_16BIT, {0x3616}, 0x2A08},
	{AR0543_RAW_16BIT, {0x3618}, 0x1AD0},
	{AR0543_RAW_16BIT, {0x361A}, 0xCF2E},
	{AR0543_RAW_16BIT, {0x361C}, 0x8E71},
	{AR0543_RAW_16BIT, {0x361E}, 0x0370},
	{AR0543_RAW_16BIT, {0x3620}, 0xF14B},
	{AR0543_RAW_16BIT, {0x3622}, 0x0E51},
	{AR0543_RAW_16BIT, {0x3624}, 0xCCCB},
	{AR0543_RAW_16BIT, {0x3626}, 0xD251},
	{AR0543_RAW_16BIT, {0x3640}, 0xA8AB},
	{AR0543_RAW_16BIT, {0x3642}, 0x9B8D},
	{AR0543_RAW_16BIT, {0x3644}, 0xD2CE},
	{AR0543_RAW_16BIT, {0x3646}, 0xA76B},
	{AR0543_RAW_16BIT, {0x3648}, 0x1C70},
	{AR0543_RAW_16BIT, {0x364A}, 0x5069},
	{AR0543_RAW_16BIT, {0x364C}, 0x242E},
	{AR0543_RAW_16BIT, {0x364E}, 0xAA8A},
	{AR0543_RAW_16BIT, {0x3650}, 0xAE0F},
	{AR0543_RAW_16BIT, {0x3652}, 0xC76D},
	{AR0543_RAW_16BIT, {0x3654}, 0x562A},
	{AR0543_RAW_16BIT, {0x3656}, 0x7BCD},
	{AR0543_RAW_16BIT, {0x3658}, 0x702D},
	{AR0543_RAW_16BIT, {0x365A}, 0xCA6E},
	{AR0543_RAW_16BIT, {0x365C}, 0x9CEE},
	{AR0543_RAW_16BIT, {0x365E}, 0xF287},
	{AR0543_RAW_16BIT, {0x3660}, 0xA10E},
	{AR0543_RAW_16BIT, {0x3662}, 0x78EE},
	{AR0543_RAW_16BIT, {0x3664}, 0x11CE},
	{AR0543_RAW_16BIT, {0x3666}, 0x962F},
	{AR0543_RAW_16BIT, {0x3680}, 0x03D1},
	{AR0543_RAW_16BIT, {0x3682}, 0x16AE},
	{AR0543_RAW_16BIT, {0x3684}, 0xA2D1},
	{AR0543_RAW_16BIT, {0x3686}, 0xD8CF},
	{AR0543_RAW_16BIT, {0x3688}, 0x8EF3},
	{AR0543_RAW_16BIT, {0x368A}, 0x15D1},
	{AR0543_RAW_16BIT, {0x368C}, 0x9BAE},
	{AR0543_RAW_16BIT, {0x368E}, 0xBF32},
	{AR0543_RAW_16BIT, {0x3690}, 0xB910},
	{AR0543_RAW_16BIT, {0x3692}, 0x244E},
	{AR0543_RAW_16BIT, {0x3694}, 0x5F30},
	{AR0543_RAW_16BIT, {0x3696}, 0x128C},
	{AR0543_RAW_16BIT, {0x3698}, 0xA6D2},
	{AR0543_RAW_16BIT, {0x369A}, 0x2EEF},
	{AR0543_RAW_16BIT, {0x369C}, 0x0611},
	{AR0543_RAW_16BIT, {0x369E}, 0x0631},
	{AR0543_RAW_16BIT, {0x36A0}, 0xFC8D},
	{AR0543_RAW_16BIT, {0x36A2}, 0xBE51},
	{AR0543_RAW_16BIT, {0x36A4}, 0x80B0},
	{AR0543_RAW_16BIT, {0x36A6}, 0xED52},
	{AR0543_RAW_16BIT, {0x36C0}, 0x8FAD},
	{AR0543_RAW_16BIT, {0x36C2}, 0xC2CB},
	{AR0543_RAW_16BIT, {0x36C4}, 0x6710},
	{AR0543_RAW_16BIT, {0x36C6}, 0x26F0},
	{AR0543_RAW_16BIT, {0x36C8}, 0xCF91},
	{AR0543_RAW_16BIT, {0x36CA}, 0x456E},
	{AR0543_RAW_16BIT, {0x36CC}, 0xC0AE},
	{AR0543_RAW_16BIT, {0x36CE}, 0xA5F0},
	{AR0543_RAW_16BIT, {0x36D0}, 0x156F},
	{AR0543_RAW_16BIT, {0x36D2}, 0x18D1},
	{AR0543_RAW_16BIT, {0x36D4}, 0xAEED},
	{AR0543_RAW_16BIT, {0x36D6}, 0xA6CE},
	{AR0543_RAW_16BIT, {0x36D8}, 0x178E},
	{AR0543_RAW_16BIT, {0x36DA}, 0x140C},
	{AR0543_RAW_16BIT, {0x36DC}, 0xB40E},
	{AR0543_RAW_16BIT, {0x36DE}, 0x85AB},
	{AR0543_RAW_16BIT, {0x36E0}, 0x216E},
	{AR0543_RAW_16BIT, {0x36E2}, 0x23CE},
	{AR0543_RAW_16BIT, {0x36E4}, 0x6E2E},
	{AR0543_RAW_16BIT, {0x36E6}, 0xBEAF},
	{AR0543_RAW_16BIT, {0x3700}, 0x9CF1},
	{AR0543_RAW_16BIT, {0x3702}, 0xAD4F},
	{AR0543_RAW_16BIT, {0x3704}, 0x8154},
	{AR0543_RAW_16BIT, {0x3706}, 0x6550},
	{AR0543_RAW_16BIT, {0x3708}, 0x2DD5},
	{AR0543_RAW_16BIT, {0x370A}, 0xF791},
	{AR0543_RAW_16BIT, {0x370C}, 0xFD6E},
	{AR0543_RAW_16BIT, {0x370E}, 0xF411},
	{AR0543_RAW_16BIT, {0x3710}, 0x2272},
	{AR0543_RAW_16BIT, {0x3712}, 0x1E94},
	{AR0543_RAW_16BIT, {0x3714}, 0xBC11},
	{AR0543_RAW_16BIT, {0x3716}, 0x860E},
	{AR0543_RAW_16BIT, {0x3718}, 0x898F},
	{AR0543_RAW_16BIT, {0x371A}, 0xAACF},
	{AR0543_RAW_16BIT, {0x371C}, 0x3072},
	{AR0543_RAW_16BIT, {0x371E}, 0x96D1},
	{AR0543_RAW_16BIT, {0x3720}, 0xE84D},
	{AR0543_RAW_16BIT, {0x3722}, 0x8454},
	{AR0543_RAW_16BIT, {0x3724}, 0x0DD2},
	{AR0543_RAW_16BIT, {0x3726}, 0x2C95},
	{AR0543_RAW_16BIT, {0x3782}, 0x0544},
	{AR0543_RAW_16BIT, {0x3784}, 0x03B4},
	{AR0543_RAW_16BIT, {0x37C0}, 0x0000},
	{AR0543_RAW_16BIT, {0x37C2}, 0x0000},
	{AR0543_RAW_16BIT, {0x37C4}, 0x0000},
	{AR0543_RAW_16BIT, {0x37C6}, 0x0000},
	{AR0543_RAW_16BIT, {0x3780}, 0x8000},
	/* Smaller_FallOff60--- */

	{AR0543_RAW_16BIT, {0x3ECE}, 0x000A},
	{AR0543_RAW_16BIT, {0x0400}, 0x0000},
	{AR0543_RAW_16BIT, {0x0404}, 0x0010},

	/* PLL_Configuration */
	{AR0543_RAW_16BIT, {0x0300}, 0x06},
	{AR0543_RAW_16BIT, {0x0302}, 0x01},
	{AR0543_RAW_16BIT, {0x0304}, 0x02},
	{AR0543_RAW_16BIT, {0x0306}, 0x46},
	{AR0543_RAW_16BIT, {0x0308}, 0x0A},
	{AR0543_RAW_16BIT, {0x030A}, 0x01},
	{AR0543_RAW_TOK_DELAY, {0}, 5},/* DELAY=5 */

	/* 2592*1944 @15FPS */
	{AR0543_RAW_16BIT, {0x0400}, 0x0000},
	{AR0543_RAW_16BIT, {0x0404}, 0x0010},
	{AR0543_RAW_16BIT, {0x034C}, 0x0A20},
	{AR0543_RAW_16BIT, {0x034E}, 0x0798},
	{AR0543_RAW_16BIT, {0x0344}, 0x0008},
	{AR0543_RAW_16BIT, {0x0346}, 0x0008},
	{AR0543_RAW_16BIT, {0x0348}, 0x0A27},
	{AR0543_RAW_16BIT, {0x034A}, 0x079F},
#ifdef CONFIG_ME175CG
	{AR0543_RAW_16BIT, {0x3040}, 0xC041},
#else
	{AR0543_RAW_16BIT, {0x3040}, 0x0041},
#endif
	{AR0543_RAW_16BIT, {0x3010}, 0x00A0},
	{AR0543_RAW_16BIT, {0x3012}, 0x07E4},
	{AR0543_RAW_16BIT, {0x3014}, 0x02CE},
	{AR0543_RAW_16BIT, {0x0340}, 0x07E5},
	{AR0543_RAW_16BIT, {0x0342}, 0x0E6E},

	{AR0543_RAW_8BIT, {0x0104}, 0x00 },

	/* === End of Initial Setting === */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static const struct ar0543_raw_reg ar0543_raw_soft_standby[] = {
	{AR0543_RAW_8BIT, {0x0100}, 0x00},
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static const struct ar0543_raw_reg ar0543_raw_streaming[] = {
	{AR0543_RAW_8BIT, {0x0100}, 0x01},
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static const struct ar0543_raw_reg ar0543_raw_param_hold[] = {
	{AR0543_RAW_8BIT, {0x0104}, 0x01},	/* GROUPED_PARAMETER_HOLD */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static const struct ar0543_raw_reg ar0543_raw_param_update[] = {
	{AR0543_RAW_8BIT, {0x0104}, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};


static struct ar0543_raw_reg const ar0543_raw_2592x1944_15fps[] = {
	/* [2592*1944 @15FPS] */
	{AR0543_RAW_8BIT, {0x0104}, 0x01},

	/* Timing Settings */
	{AR0543_RAW_16BIT, {0x0400}, 0x0000},
	{AR0543_RAW_16BIT, {0x0404}, 0x0010},
	{AR0543_RAW_16BIT, {0x034C}, 0x0A20},
	{AR0543_RAW_16BIT, {0x034E}, 0x0798},
	{AR0543_RAW_16BIT, {0x0344}, 0x0008},
	{AR0543_RAW_16BIT, {0x0346}, 0x0008},
	{AR0543_RAW_16BIT, {0x0348}, 0x0A27},
	{AR0543_RAW_16BIT, {0x034A}, 0x079F},
#ifdef CONFIG_ME175CG
	{AR0543_RAW_16BIT, {0x3040}, 0xC041},
#else
	{AR0543_RAW_16BIT, {0x3040}, 0x0041},
#endif
	{AR0543_RAW_16BIT, {0x3010}, 0x00A0},
	{AR0543_RAW_16BIT, {0x3012}, 0x07E4},
	{AR0543_RAW_16BIT, {0x3014}, 0x02CE},
	{AR0543_RAW_16BIT, {0x0340}, 0x07E5},
	{AR0543_RAW_16BIT, {0x0342}, 0x0E6E},

	{AR0543_RAW_8BIT, {0x0104}, 0x00},
	{AR0543_RAW_TOK_DELAY, {0}, 5},/* DELAY=5 */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static struct ar0543_raw_reg const ar0543_raw_1080p_30fps[] = {
	/* [1936*1096 @30FPS] */
	{AR0543_RAW_8BIT, {0x0104}, 0x01},

	/* Timing Settings */
	{AR0543_RAW_16BIT, {0x0400}, 0x0000},
	{AR0543_RAW_16BIT, {0x0404}, 0x0010},
	{AR0543_RAW_16BIT, {0x034C}, 0x0790},
	{AR0543_RAW_16BIT, {0x034E}, 0x0448},
	{AR0543_RAW_16BIT, {0x0344}, 0x0008},
	{AR0543_RAW_16BIT, {0x0346}, 0x0008},
	{AR0543_RAW_16BIT, {0x0348}, 0x0797},
	{AR0543_RAW_16BIT, {0x034A}, 0x044F},
#ifdef CONFIG_ME175CG

	{AR0543_RAW_16BIT, {0x3040}, 0xC041},
#else
	{AR0543_RAW_16BIT, {0x3040}, 0x0041},
#endif
	{AR0543_RAW_16BIT, {0x3010}, 0x00A0},
	{AR0543_RAW_16BIT, {0x3012}, 0x058E},
	{AR0543_RAW_16BIT, {0x3014}, 0x02CE},
	{AR0543_RAW_16BIT, {0x0340}, 0x058F},
	{AR0543_RAW_16BIT, {0x0342}, 0x0BE0},

	{AR0543_RAW_8BIT, {0x0104}, 0x00},
	{AR0543_RAW_TOK_DELAY, {0}, 5},/* DELAY=5 */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static struct ar0543_raw_reg const ar0543_raw_960p_30fps[] = {
	/* [1296*976 @30FPS] */
	{AR0543_RAW_8BIT, {0x0104}, 0x01},

	/* Timing Settings */
	{AR0543_RAW_16BIT, {0x0400}, 0x0000},
	{AR0543_RAW_16BIT, {0x0404}, 0x0010},
	{AR0543_RAW_16BIT, {0x034C}, 0x0510},
	{AR0543_RAW_16BIT, {0x034E}, 0x03D0},
	{AR0543_RAW_16BIT, {0x0344}, 0x0008},
	{AR0543_RAW_16BIT, {0x0346}, 0x0008},
	{AR0543_RAW_16BIT, {0x0348}, 0x0A25},
	{AR0543_RAW_16BIT, {0x034A}, 0x07A5},
#ifdef CONFIG_ME175CG
	{AR0543_RAW_16BIT, {0x3040}, 0xC4C3},
#else
	{AR0543_RAW_16BIT, {0x3040}, 0x04C3},
#endif
	{AR0543_RAW_16BIT, {0x3010}, 0x0184},
	{AR0543_RAW_16BIT, {0x3012}, 0x04F5},
	{AR0543_RAW_16BIT, {0x3014}, 0x05F8},
	{AR0543_RAW_16BIT, {0x0340}, 0x04F6},
	{AR0543_RAW_16BIT, {0x0342}, 0x0C4E},

	{AR0543_RAW_8BIT, {0x0104}, 0x00},
	{AR0543_RAW_TOK_DELAY, {0}, 5},/* DELAY=5 */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};


static struct ar0543_raw_reg const ar0543_raw_720p_30fps[] = {
	/* [1296*736 @30FPS] */
	{AR0543_RAW_8BIT, {0x0104}, 0x01},

	/* Timing Settings */
	{AR0543_RAW_16BIT, {0x0400}, 0x0000},
	{AR0543_RAW_16BIT, {0x0404}, 0x0010},
	{AR0543_RAW_16BIT, {0x034C}, 0x0510},
	{AR0543_RAW_16BIT, {0x034E}, 0x02E0},
	{AR0543_RAW_16BIT, {0x0344}, 0x0008},
	{AR0543_RAW_16BIT, {0x0346}, 0x0008},
	{AR0543_RAW_16BIT, {0x0348}, 0x0A25},
	{AR0543_RAW_16BIT, {0x034A}, 0x05C5},
#ifdef CONFIG_ME175CG
	{AR0543_RAW_16BIT, {0x3040}, 0xC4C3},
#else
	{AR0543_RAW_16BIT, {0x3040}, 0x04C3},
#endif
	{AR0543_RAW_16BIT, {0x3010}, 0x0184},
	{AR0543_RAW_16BIT, {0x3012}, 0x04A0},
	{AR0543_RAW_16BIT, {0x3014}, 0x05F8},
	{AR0543_RAW_16BIT, {0x0340}, 0x04A1},
	{AR0543_RAW_16BIT, {0x0342}, 0x0C4E},

	{AR0543_RAW_8BIT, {0x0104}, 0x00},
	{AR0543_RAW_TOK_DELAY, {0}, 5},/* DELAY=5 */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};


static struct ar0543_raw_reg const ar0543_raw_VGA_30fps[] = {
	/* [656*496 @30FPS] },
	 668*500 that bining from 2592*1944 and scale from 1296*972 */
	{AR0543_RAW_8BIT, {0x0104}, 0x01},

	/* Timing Settings */
	{AR0543_RAW_16BIT, {0x0400}, 0x0002},
	{AR0543_RAW_16BIT, {0x0404}, 0x001F},
	{AR0543_RAW_16BIT, {0x034C}, 0x029C},
	{AR0543_RAW_16BIT, {0x034E}, 0x01F4},
	{AR0543_RAW_16BIT, {0x0344}, 0x0008},
	{AR0543_RAW_16BIT, {0x0346}, 0x0008},
	{AR0543_RAW_16BIT, {0x0348}, 0x0A25},
	{AR0543_RAW_16BIT, {0x034A}, 0x079D},
#ifdef CONFIG_ME175CG
	{AR0543_RAW_16BIT, {0x3040}, 0xC4C3},
#else
	{AR0543_RAW_16BIT, {0x3040}, 0x04C3},
#endif
	{AR0543_RAW_16BIT, {0x3010}, 0x0184},
	{AR0543_RAW_16BIT, {0x3012}, 0x0414},
	{AR0543_RAW_16BIT, {0x3014}, 0x05F8},
	{AR0543_RAW_16BIT, {0x0340}, 0x0415},
	{AR0543_RAW_16BIT, {0x0342}, 0x0DF4},

	{AR0543_RAW_8BIT, {0x0104}, 0x00},
	{AR0543_RAW_TOK_DELAY, {0}, 5},/* DELAY=5 */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static struct ar0543_raw_reg const ar0543_raw_CIF_30fps[] = {
	/* [368*304  @30FPS]},
	  432*324 that bining from 2592*1944 and scale from 1296*972 */
	{AR0543_RAW_8BIT, {0x0104}, 0x01},

	/* Timing Settings */
	{AR0543_RAW_16BIT, {0x0400}, 0x0002},
	{AR0543_RAW_16BIT, {0x0404}, 0x0030},
	{AR0543_RAW_16BIT, {0x034C}, 0x01B0},
	{AR0543_RAW_16BIT, {0x034E}, 0x0144},
	{AR0543_RAW_16BIT, {0x0344}, 0x0008},
	{AR0543_RAW_16BIT, {0x0346}, 0x0008},
	{AR0543_RAW_16BIT, {0x0348}, 0x0A25},
	{AR0543_RAW_16BIT, {0x034A}, 0x079D},
#ifdef CONFIG_ME175CG
	{AR0543_RAW_16BIT, {0x3040}, 0xC4C3},
#else
	{AR0543_RAW_16BIT, {0x3040}, 0x04C3},
#endif
	{AR0543_RAW_16BIT, {0x3010}, 0x0184},
	{AR0543_RAW_16BIT, {0x3012}, 0x0414},
	{AR0543_RAW_16BIT, {0x3014}, 0x05F8},
	{AR0543_RAW_16BIT, {0x0340}, 0x0415},
	{AR0543_RAW_16BIT, {0x0342}, 0x0DF4},

	{AR0543_RAW_8BIT, {0x0104}, 0x00},
	{AR0543_RAW_TOK_DELAY, {0}, 5},/* DELAY=5 */
	{AR0543_RAW_TOK_TERM, {0}, 0}
};

static struct ar0543_raw_resolution ar0543_raw_res_preview[] = {
	{
		 .desc =	"PREVIEW_960p_30fps"	,
		 .width =	1296	,
		 .height =	976	,
		 .fps =		30	,
		 .used =	0	,
		 .pixels_per_line = 0x0C4E, /* consistent with regs arrays */
		 .lines_per_frame = 0x04F6, /* consistent with regs arrays */
		 .regs =	ar0543_raw_960p_30fps	,
		 .bin_factor_x =	1,
		 .bin_factor_y =	1,
		 .skip_frames = 1, /*change skip num from 1 to 0 after 3A init
				    param invalid issue fixed*/
	},
	{
		 /* For 2560x1920 output */
		 .desc =	"PREVIEW_2592x1944_15fps"	,
		 .width =	2592	,
		 .height =	1944	,
		 .fps =		15	,
		 .used =	0	,
		 .pixels_per_line = 0x0E6E, /* consistent with regs arrays */
		 .lines_per_frame = 0x07E5, /* consistent with regs arrays */
		 .regs =	ar0543_raw_2592x1944_15fps,
		 .bin_factor_x =	0,
		 .bin_factor_y =	0,
		 .skip_frames = 1,
	},
};

#define N_RES_PREVIEW (ARRAY_SIZE(ar0543_raw_res_preview))

#if 0
static struct ar0543_raw_resolution ar0543_raw_res_still[] = {
	{
		 /* For 2560x1920 output */
		 .desc =	"STILL_2592x1944_15fps"	,
		 .width =	2592	,
		 .height =	1944	,
		 .fps =		15	,
		 .used =	0	,
		 .pixels_per_line = 0x0E6E, /* consistent with regs arrays */
		 .lines_per_frame = 0x07E5, /* consistent with regs arrays */
		 .regs =	ar0543_raw_2592x1944_15fps,
		 .bin_factor_x =	0,
		 .bin_factor_y =	0,
		 .skip_frames = 1,
	},
};

#define N_RES_STILL (ARRAY_SIZE(ar0543_raw_res_still))
#endif

static struct ar0543_raw_resolution ar0543_raw_res_video[] = {
	{
		 .desc =	 "VIDEO_CIF_30fps"  ,
		 .width =	 368 ,
		 .height =  304 ,
		 .fps =	 30  ,
		 .used =	 0	 ,
		 .pixels_per_line = 0x0DF4, /* consistent with regs arrays */
		 .lines_per_frame = 0x0415, /* consistent with regs arrays */
		 .regs =	 ar0543_raw_CIF_30fps	 ,
		 .bin_factor_x =	 2,
		 .bin_factor_y =	 2,
		 .skip_frames = 1,
	},
	{
		 .desc =	 "VIDEO_VGA_30fps"	 ,
		 .width =	 656 ,
		 .height =  496 ,
		 .fps =	 30  ,
		 .used =	 0	 ,
		 .pixels_per_line = 0x0DF4, /* consistent with regs arrays */
		 .lines_per_frame = 0x0415, /* consistent with regs arrays */
		 .regs =	 ar0543_raw_VGA_30fps	 ,
		 .bin_factor_x =	 2,
		 .bin_factor_y =	 2,
		 .skip_frames = 1,
	},
	{
		 .desc =	 "VIDEO_720p_30fps"  ,
		 .width =	 1296	 ,
		 .height =  736 ,
		 .fps =	 30  ,
		 .used =	 0	 ,
		 .pixels_per_line = 0x0C4E, /* consistent with regs arrays */
		 .lines_per_frame = 0x04A1, /* consistent with regs arrays */
		 .regs =	 ar0543_raw_720p_30fps	 ,
		 .bin_factor_x =	 1,
		 .bin_factor_y =	 1,
		 .skip_frames = 1,
	},
	{
		 .desc =	  "VIDEO_960p_30fps"  ,
		 .width =   1296	  ,
		 .height =  976 ,
		 .fps =	  30  ,
		 .used =	  0   ,
		 .pixels_per_line = 0x0C4E, /* consistent with regs arrays */
		 .lines_per_frame = 0x04F6, /* consistent with regs arrays */
		 .regs =	  ar0543_raw_960p_30fps   ,
		 .bin_factor_x =	  1,
		 .bin_factor_y =	  1,
		 .skip_frames = 1,
	},
	{
		 .desc =	  "VIDEO_1080p_30fps" ,
		 .width =   1936	  ,
		 .height =  1096	  ,
		 .fps =	  30  ,
		 .used =	  0   ,
		 .pixels_per_line = 0x0BE0, /* consistent with regs arrays */
		 .lines_per_frame = 0x058F, /* consistent with regs arrays */
		 .regs =	  ar0543_raw_1080p_30fps  ,
		 .bin_factor_x =	  1,
		 .bin_factor_y =	  1,
		 .skip_frames = 1,
	},
};

#define N_RES_VIDEO (ARRAY_SIZE(ar0543_raw_res_video))

static struct ar0543_raw_resolution *ar0543_raw_res = ar0543_raw_res_preview;
static int N_RES = N_RES_PREVIEW;

#endif
