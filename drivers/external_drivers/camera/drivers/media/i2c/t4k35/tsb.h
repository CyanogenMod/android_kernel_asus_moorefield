/*
 * Support for Toshiba camera sensor.
 *
 * Copyright (c) 2010 Intel Corporation. All Rights Reserved.
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

#ifndef __TSB_H__
#define __TSB_H__
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/spinlock.h>
#include <linux/types.h>
#include <linux/videodev2.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include <media/v4l2-subdev.h>
#include "t4k35.h"

#define TSB_MCLK		192

/* TODO - This should be added into include/linux/videodev2.h */
#ifndef V4L2_IDENT_TSB
#define V4L2_IDENT_TSB	8245
#endif

/*
 * tsb System control registers
 */
#define TSB_MASK_5BIT	0x1F
#define TSB_MASK_4BIT	0xF
#define TSB_MASK_2BIT	0x3
#define TSB_MASK_11BIT	0x7FF
#define TSB_INTG_BUF_COUNT		2

#define TSB_FINE_INTG_TIME		0xA41	// Read Only register for Toshiba sensor

#define TSB_VT_PIX_CLK_DIV			0x0301
#define TSB_VT_SYS_CLK_DIV			0x0303
//#define TSB_PRE_PLL_CLK_DIV			0x0305	// PRE_PLL_CLK_DIV
#define TSB_PRE_PLL_CLK_DIV			0x030D	// PRE_PLL_ST_CLK_DIV
//#define TSB_PLL_MULTIPLIER			0x0306	// PLL_MULTIPLER_
#define TSB_PLL_MULTIPLIER			0x030E	// PLL_MULTIPLER_ST
//#define IMX_OP_PIX_DIV			0x0309	// NO exist for Toshiba sensor
#define TSB_OP_SYS_DIV			0x030B
#define TSB_FRAME_LENGTH_LINES		0x0340
#define TSB_LINE_LENGTH_PIXELS		0x0342
//#define IMX_COARSE_INTG_TIME_MIN	0x1004	// NO exist for Toshiba sensor
//#define IMX_COARSE_INTG_TIME_MAX	0x1006	// NO exist for Toshiba sensor
//#define IMX_BINNING_ENABLE		0x0390
#define TSB_H_BINNING_ENABLE		0x0900	// 0,1,2 for 1/1,1/2,1/4 binning
#define TSB_V_BINNING_ENABLE		0x0901	// 0,1,2 for 1/1,1/2,1/4 binning
#define TSB_BINNING_TYPE		0x0902	// 0,1 for Ave,Add mode

//#define IMX_READ_MODE				0x0390	// binning enable for imx

//#define IMX_HORIZONTAL_START_H 0x0344	// NO exist for Toshiba sensor
#define TSB_VERTICAL_START_H 0x0346
//#define IMX_HORIZONTAL_END_H 0x0348	// NO exist for Toshiba sensor
#define TSB_VERTICAL_END_H 0x034A
#define TSB_HORIZONTAL_OUTPUT_SIZE_H 0x034c
#define TSB_VERTICAL_OUTPUT_SIZE_H 0x034e

#define TSB_COARSE_INTEGRATION_TIME		0x0202
#define TSB_TEST_PATTERN_MODE			0x0600
#define TSB_IMG_ORIENTATION			0x0101
#define TSB_VFLIP_BIT			2
#define TSB_HFLIP_BIT			1
#define TSB_GLOBAL_GAIN			0x0204
//#define IMX_SHORT_AGC_GAIN		0x0233	// NO exist for Toshiba sensor
#define TSB_DGC_ADJ		0x0210
#define TSB_DGC_LEN		10
#define TSB_MAX_EXPOSURE_SUPPORTED 0xfff9
#define TSB_MAX_GLOBAL_GAIN_SUPPORTED 0x0fff
#define TSB_MAX_DIGITAL_GAIN_SUPPORTED 0x03ff

#define MAX_FMTS 1
//#define IMX_OTP_DATA_SIZE		1280

#define TSB_SUBDEV_PREFIX "t4k"
#define TSB_DRIVER	"t4k35"

/* Sensor ids from identification register */
#define TSB_NAME_K37	"t4k37"
#define TSB_NAME_K35	"t4k35"
#define T4K37_ID	0x1C21
#define T4K37_NAME_ID 0x37
#define T4K35_ID	0x1481
#define T4K35_NAME_ID 0x35

#if 0	// TSB
/* Sensor id based on i2c_device_id table
 * (Fuji module can not be detected based on sensor registers) */
#define IMX135_FUJI_ID			0x0136
#define IMX_NAME_135_FUJI		"imx135fuji"

/* imx175 - use dw9714 vcm */
#define IMX175_MERRFLD 0x175
#define IMX175_VALLEYVIEW 0x176
#define IMX135_SALTBAY 0x135
#define IMX135_VICTORIABAY 0x136
#define IMX132_SALTBAY 0x132
#define IMX134_VALLEYVIEW 0x134
#endif	// TSB

/* otp - specific settings */
#define E2PROM_ADDR 0xa0
#define E2PROM_LITEON_12P1BA869D_ADDR 0xa0
#define E2PROM_ABICO_SS89A839_ADDR 0xa8
#define DEFAULT_OTP_SIZE 1280
#define E2PROM_LITEON_12P1BA869D_SIZE 544

#define TSB_ID_DEFAULT	0x0000
#define TSB_CHIP_ID	0x0000

#define T4K37_RES_WIDTH_MAX	    4208
#define T4K37_RES_HEIGHT_MAX	3120
#define T4K35_RES_WIDTH_MAX	    3280
#define T4K35_RES_HEIGHT_MAX	2464

/* Defines for lens/VCM */
#define TSB_FOCAL_LENGTH_NUM	369	/*3.69mm*/
#define TSB_FOCAL_LENGTH_DEM	100
#define TSB_F_NUMBER_DEFAULT_NUM	22
#define TSB_F_NUMBER_DEM	10
#define TSB_INVALID_CONFIG	0xffffffff
#define TSB_MAX_FOCUS_POS	1023
#define TSB_MAX_FOCUS_NEG	(-1023)
#define TSB_VCM_SLEW_STEP_MAX	0x3f
#define TSB_VCM_SLEW_TIME_MAX	0x1f

#define TSB_BIN_FACTOR_MAX			4
#define TSB_INTEGRATION_TIME_MARGIN	6
/*
 * focal length bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define TSB_FOCAL_LENGTH_DEFAULT 0x1710064

/*
 * current f-number bits definition:
 * bits 31-16: numerator, bits 15-0: denominator
 */
#define TSB_F_NUMBER_DEFAULT 0x16000a

/*
 * f-number range bits definition:
 * bits 31-24: max f-number numerator
 * bits 23-16: max f-number denominator
 * bits 15-8: min f-number numerator
 * bits 7-0: min f-number denominator
 */
#define TSB_F_NUMBER_RANGE 0x160a160a

struct tsb_vcm {
	int (*power_up)(struct v4l2_subdev *sd);
	int (*power_down)(struct v4l2_subdev *sd);
	int (*init)(struct v4l2_subdev *sd);
	int (*t_focus_vcm)(struct v4l2_subdev *sd, u16 val);
	int (*t_focus_abs)(struct v4l2_subdev *sd, s32 value);
	int (*t_focus_rel)(struct v4l2_subdev *sd, s32 value);
	int (*q_focus_status)(struct v4l2_subdev *sd, s32 *value);
	int (*q_focus_abs)(struct v4l2_subdev *sd, s32 *value);
	int (*t_vcm_slew)(struct v4l2_subdev *sd, s32 value);
	int (*t_vcm_timing)(struct v4l2_subdev *sd, s32 value);
};

struct tsb_otp {
	void *(*otp_read)(struct v4l2_subdev *sd);
	u32 start_addr;
	u32 size;
	u8 dev_addr;
};

struct max_res {
	int res_max_width;
	int res_max_height;
};

struct max_res tsb_max_res = {
	//[T4K37_ID] = {
		.res_max_width = T4K35_RES_WIDTH_MAX,
		.res_max_height = T4K35_RES_HEIGHT_MAX,
	//},
	//[T4K35_ID] = {
		//.res_max_width = T4K35_RES_WIDTH_MAX,
		//.res_max_height = T4K35_RES_HEIGHT_MAX,
	//},
};

struct tsb_settings {
	struct tsb_reg const *init_settings;
	struct tsb_resolution *res_preview;
	struct tsb_resolution *res_still;
	struct tsb_resolution *res_video;
	int n_res_preview;
	int n_res_still;
	int n_res_video;
};

struct tsb_settings tsb_set = {
	//[T4K37_MERRFLD] = {
		.init_settings = t4k35_init_settings,
		.res_preview = t4k35_res_preview,
		.res_still = t4k35_res_still,
		.res_video = t4k35_res_video,
		.n_res_preview = ARRAY_SIZE(t4k35_res_preview),
		.n_res_still = ARRAY_SIZE(t4k35_res_still),
		.n_res_video = ARRAY_SIZE(t4k35_res_video),
	//},
//	[T4K35_MERRFLD] = {
//		.init_settings = t4k35_init_settings,
//		.res_preview = t4k35_res_preview,
//		.res_still = t4k35_res_still,
//		.res_video = t4k35_res_video,
//		.n_res_preview = ARRAY_SIZE(t4k35_res_preview),
//		.n_res_still = ARRAY_SIZE(t4k35_res_still),
//		.n_res_video = ARRAY_SIZE(t4k35_res_video),
//	},
};

#define	v4l2_format_capture_type_entry(_width, _height, \
		_pixelformat, _bytesperline, _colorspace) \
	{\
		.type = V4L2_BUF_TYPE_VIDEO_CAPTURE,\
		.fmt.pix.width = (_width),\
		.fmt.pix.height = (_height),\
		.fmt.pix.pixelformat = (_pixelformat),\
		.fmt.pix.bytesperline = (_bytesperline),\
		.fmt.pix.colorspace = (_colorspace),\
		.fmt.pix.sizeimage = (_height)*(_bytesperline),\
	}

#define	s_output_format_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps) \
	{\
		.v4l2_fmt = v4l2_format_capture_type_entry(_width, \
			_height, _pixelformat, _bytesperline, \
				_colorspace),\
		.fps = (_fps),\
	}

#define	s_output_format_reg_entry(_width, _height, _pixelformat, \
		_bytesperline, _colorspace, _fps, _reg_setting) \
	{\
		.s_fmt = s_output_format_entry(_width, _height,\
				_pixelformat, _bytesperline, \
				_colorspace, _fps),\
		.reg_setting = (_reg_setting),\
	}

struct s_ctrl_id {
	struct v4l2_queryctrl qc;
	int (*s_ctrl)(struct v4l2_subdev *sd, u32 val);
	int (*g_ctrl)(struct v4l2_subdev *sd, u32 *val);
};

#define	v4l2_queryctrl_entry_integer(_id, _name,\
		_minimum, _maximum, _step, \
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_INTEGER, \
		.name = _name, \
		.minimum = (_minimum), \
		.maximum = (_maximum), \
		.step = (_step), \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}
#define	v4l2_queryctrl_entry_boolean(_id, _name,\
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_BOOLEAN, \
		.name = _name, \
		.minimum = 0, \
		.maximum = 1, \
		.step = 1, \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}

#define	s_ctrl_id_entry_integer(_id, _name, \
		_minimum, _maximum, _step, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_integer(_id, _name,\
				_minimum, _maximum, _step,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

#define	s_ctrl_id_entry_boolean(_id, _name, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_boolean(_id, _name,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}


struct tsb_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, s32 value);
};

/* tsb device structure */
struct tsb_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct camera_sensor_platform_data *platform_data;
	struct mutex input_lock; /* serialize sensor's ioctl */
	int fmt_idx;
	int status;
	int streaming;
	int power;
	int run_mode;
	int vt_pix_clk_freq_mhz;
	int fps_index;
	u32 focus;
	u16 sensor_id;			/* Sensor id from registers */
	u16 i2c_id;			/* Sensor id from i2c_device_id */
	u16 coarse_itg;
	u16 fine_itg;
	u16 digital_gain;
	u16 gain;
	u16 pixels_per_line;
	u16 lines_per_frame;
	u8 fps;
	const struct tsb_reg *regs;
	u8 res;
	u8 type;
	u8 sensor_revision;
	u8 *otp_data;
	struct tsb_settings *mode_tables;
	struct tsb_vcm *vcm_driver;
	struct tsb_otp *otp_driver;
	const struct tsb_resolution *curr_res_table;
	int entries_curr_table;
	const struct firmware *fw;

	/* used for h/b blank tuning */
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_ctrl *pixel_rate;
	struct v4l2_ctrl *h_blank;
	struct v4l2_ctrl *v_blank;
};

#define to_tsb_sensor(x) container_of(x, struct tsb_device, sd)

#define TSB_MAX_WRITE_BUF_SIZE	32
struct tsb_write_buffer {
	u16 addr;
	u8 data[TSB_MAX_WRITE_BUF_SIZE];
};

struct tsb_write_ctrl {
	int index;
	struct tsb_write_buffer buffer;
};

static const struct tsb_reg tsb_soft_standby[] = {
	{TSB_8BIT, 0x0100, 0x00},
	{TSB_TOK_TERM, 0, 0}
};

static const struct tsb_reg tsb_streaming[] = {
	{TSB_8BIT, 0x0100, 0x01},
	{TSB_TOK_TERM, 0, 0}
};

static const struct tsb_reg tsb_param_hold[] = {
	{TSB_8BIT, 0x0104, 0x01},	/* GROUPED_PARAMETER_HOLD */
	{TSB_TOK_TERM, 0, 0}
};

static const struct tsb_reg tsb_param_update[] = {
	{TSB_8BIT, 0x0104, 0x00},	/* GROUPED_PARAMETER_HOLD */
	{TSB_TOK_TERM, 0, 0}
};

extern int ad5816g_vcm_power_up(struct v4l2_subdev *sd);
extern int ad5816g_vcm_power_down(struct v4l2_subdev *sd);
extern int ad5816g_vcm_init(struct v4l2_subdev *sd);

extern int ad5816g_t_focus_vcm(struct v4l2_subdev *sd, u16 val);
extern int ad5816g_t_focus_abs(struct v4l2_subdev *sd, s32 value);
extern int ad5816g_t_focus_rel(struct v4l2_subdev *sd, s32 value);
extern int ad5816g_q_focus_status(struct v4l2_subdev *sd, s32 *value);
extern int ad5816g_q_focus_abs(struct v4l2_subdev *sd, s32 *value);
extern int ad5816g_t_vcm_slew(struct v4l2_subdev *sd, s32 value);
extern int ad5816g_t_vcm_timing(struct v4l2_subdev *sd, s32 value);

extern int drv201_vcm_power_up(struct v4l2_subdev *sd);
extern int drv201_vcm_power_down(struct v4l2_subdev *sd);
extern int drv201_vcm_init(struct v4l2_subdev *sd);

extern int drv201_t_focus_vcm(struct v4l2_subdev *sd, u16 val);
extern int drv201_t_focus_abs(struct v4l2_subdev *sd, s32 value);
extern int drv201_t_focus_rel(struct v4l2_subdev *sd, s32 value);
extern int drv201_q_focus_status(struct v4l2_subdev *sd, s32 *value);
extern int drv201_q_focus_abs(struct v4l2_subdev *sd, s32 *value);
extern int drv201_t_vcm_slew(struct v4l2_subdev *sd, s32 value);
extern int drv201_t_vcm_timing(struct v4l2_subdev *sd, s32 value);

extern int dw9714_vcm_power_up(struct v4l2_subdev *sd);
extern int dw9714_vcm_power_down(struct v4l2_subdev *sd);
extern int dw9714_vcm_init(struct v4l2_subdev *sd);

extern int dw9714_t_focus_vcm(struct v4l2_subdev *sd, u16 val);
extern int dw9714_t_focus_abs(struct v4l2_subdev *sd, s32 value);
extern int dw9714_t_focus_rel(struct v4l2_subdev *sd, s32 value);
extern int dw9714_q_focus_status(struct v4l2_subdev *sd, s32 *value);
extern int dw9714_q_focus_abs(struct v4l2_subdev *sd, s32 *value);
extern int dw9714_t_vcm_slew(struct v4l2_subdev *sd, s32 value);
extern int dw9714_t_vcm_timing(struct v4l2_subdev *sd, s32 value);

extern int dw9719_vcm_power_up(struct v4l2_subdev *sd);
extern int dw9719_vcm_power_down(struct v4l2_subdev *sd);
extern int dw9719_vcm_init(struct v4l2_subdev *sd);

extern int dw9719_t_focus_vcm(struct v4l2_subdev *sd, u16 val);
extern int dw9719_t_focus_abs(struct v4l2_subdev *sd, s32 value);
extern int dw9719_t_focus_rel(struct v4l2_subdev *sd, s32 value);
extern int dw9719_q_focus_status(struct v4l2_subdev *sd, s32 *value);
extern int dw9719_q_focus_abs(struct v4l2_subdev *sd, s32 *value);
extern int dw9719_t_vcm_slew(struct v4l2_subdev *sd, s32 value);
extern int dw9719_t_vcm_timing(struct v4l2_subdev *sd, s32 value);

extern int vcm_power_up(struct v4l2_subdev *sd);
extern int vcm_power_down(struct v4l2_subdev *sd);

struct tsb_vcm tsb_vcm = {
	//[T4K37_MERRFLD] = {
		.power_up = dw9714_vcm_power_up,
		.power_down = dw9714_vcm_power_down,
		.init = dw9714_vcm_init,
		.t_focus_vcm = dw9714_t_focus_vcm,
		.t_focus_abs = dw9714_t_focus_abs,
		.t_focus_rel = dw9714_t_focus_rel,
		.q_focus_status = dw9714_q_focus_status,
		.q_focus_abs = dw9714_q_focus_abs,
		.t_vcm_slew = dw9714_t_vcm_slew,
		.t_vcm_timing = dw9714_t_vcm_timing,
	//},
	//[T4K35_MERRFLD] = {
		//.power_up = dw9714_vcm_power_up,
		//.power_down = dw9714_vcm_power_down,
		//.init = dw9714_vcm_init,
		//.t_focus_vcm = dw9714_t_focus_vcm,
		//.t_focus_abs = dw9714_t_focus_abs,
		//.t_focus_rel = dw9714_t_focus_rel,
		//.q_focus_status = dw9714_q_focus_status,
		//.q_focus_abs = dw9714_q_focus_abs,
		//.t_vcm_slew = dw9714_t_vcm_slew,
		//.t_vcm_timing = dw9714_t_vcm_timing,
	//},
	//[TSB_ID_DEFAULT] = {
		//.power_up = vcm_power_up,
		//.power_down = vcm_power_down,
	//},
};

extern void *tsb_otp_read(struct v4l2_subdev *sd);
struct tsb_otp tsb_otps = {
                .otp_read = tsb_otp_read,
                .size = DEFAULT_OTP_SIZE,
};

#endif

