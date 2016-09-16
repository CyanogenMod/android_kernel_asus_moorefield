/*
 * Support for HM056_raw Camera Sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#ifndef __HM2056_H__
#define __HM2056_H__

#include <linux/kernel.h>
#include <linux/types.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/videodev2.h>
#include <linux/spinlock.h>
#include <media/v4l2-subdev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/v4l2-mediabus.h>
#include <media/media-entity.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <linux/HWVersion.h>

extern int Read_HW_ID(void);
//static unsigned int HW_ID = 0xFF;

#define V4L2_IDENT_HM2056_RAW			8245
#define HM2056_NAME				"hm2056_raw"

#define I2C_RETRY_COUNT				5
#define MSG_LEN_OFFSET				2
#define MAX_FMTS				1

//registers
#define HM2056_REG_CHIP_ID_H			0x0001
#define HM2056_REG_CHIP_ID_L			0x0002

#define HM2056_REG_BLANKING_ROW_H			0x0010
#define HM2056_REG_BLANKING_ROW_L			0x0011
#define HM2056_REG_BLANKING_COLUMN_CLK		0x0012
#define HM2056_REG_BLANKING_COLUMN			0x0013

#define HM2056_REG_INTEGRATION_TIME_H		0x0015
#define HM2056_REG_INTEGRATION_TIME_L		0x0016

#define HM2056_REG_AGAIN			0x0018
#define HM2056_REG_DGAIN			0x001D

#define HM2056_REG_COMMAND_UPDATE		0x0100

#define HM2056_REG_H_START_L			0x05E4
#define HM2056_REG_H_START_H			0x05E5
#define HM2056_REG_H_END_L			0x05E6
#define HM2056_REG_H_END_H			0x05E7

#define HM2056_REG_V_START_L			0x05E8
#define HM2056_REG_V_START_H			0x05E9
#define HM2056_REG_V_END_L			0x05EA
#define HM2056_REG_V_END_H			0x05EB

/* DEVICE_ID */
#define HM2056_MOD_ID				0x2056

#define HM2056_FINE_INTG_TIME_MIN		0
#define HM2056_FINE_INTG_TIME_MAX_MARGIN	65535
#define HM2056_COARSE_INTG_TIME_MIN		0
#define HM2056_COARSE_INTG_TIME_MAX_MARGIN	65535

enum hm2056_tok_type {
	HM2056_8BIT  = 0x0001,
	HM2056_TOK_TERM   = 0xf000,	/* terminating token for reg list */
	HM2056_TOK_DELAY  = 0xfe00,	/* delay token for reg list */
	HM2056_TOK_MASK = 0xfff0
};

struct hm2056_reg {
	enum hm2056_tok_type type;
	u16 reg;
	u32 val;	/* @set value for read/mod/write, @mask */
};

struct hm2056_raw_device {
	struct v4l2_subdev sd;
	struct media_pad pad;
	struct v4l2_mbus_framefmt format;
	struct mutex input_lock;

	struct hm2056_raw_res_struct *hm2056_raw_res;
	int n_res;
	int fmt_idx;
	int run_mode;

	struct camera_sensor_platform_data *platform_data;
	char name[32];

	struct attribute_group sensor_i2c_attribute; //Add for ATD read camera status+++
};

struct hm2056_raw_res_struct {
	u8 *desc;
	int width;
	int height;
	int fps;
	int skip_frames;
	bool used;
	struct hm2056_reg const *regs;
	int horizontal_start;
	int horizontal_end;
	int vertical_start;
	int vertical_end;
	int pixel_clk;
	int line_length_pck;
	int frame_length_lines;
	u8 bin_factor_x;
	u8 bin_factor_y;
	u8 bin_mode;
};

struct hm2056_raw_control {
	struct v4l2_queryctrl qc;
	int (*query)(struct v4l2_subdev *sd, s32 *value);
	int (*tweak)(struct v4l2_subdev *sd, int value);
};

/* 2 bytes used for address: 256 bytes total */
#define HM2056_RAW_MAX_WRITE_BUF_SIZE	1024
struct hm2056_write_buffer {
	u16 addr;
	u8 data[HM2056_RAW_MAX_WRITE_BUF_SIZE];
};

struct hm2056_write_ctrl {
	int index;
	struct hm2056_write_buffer buffer;
};

static const struct i2c_device_id hm2056_raw_id[] = {
	{HM2056_NAME, 0},
	{}
};

static struct hm2056_reg const hm2056_stream_on[] = {
	{HM2056_8BIT, 0x0000, 0x01},
	{HM2056_8BIT, 0x0100, 0x01},
	{HM2056_8BIT, 0x0101, 0x01},
	{HM2056_8BIT, 0x0005, 0x01},	//Turn on rolling shutter
	{HM2056_TOK_TERM, 0, 0}
};

static struct hm2056_reg const hm2056_stream_off[] = {
	{HM2056_8BIT, 0x0005, 0x00},	//Turn on rolling shutter
	{HM2056_TOK_TERM, 0, 0}
};

/*
 * initial settings
 */
static struct hm2056_reg const hm2056_init[] = {
	{HM2056_8BIT, 0x0022, 0x00},	// Reset
	{HM2056_8BIT, 0x0020, 0x00},
	{HM2056_8BIT, 0x0025, 0x00},	//CKCFG 80 from system clock, 00 from PLL
	{HM2056_8BIT, 0x0026, 0x85},	//PLL1CFG should be 07 when system clock, should be 87 when PLL
	{HM2056_8BIT, 0x0027, 0x03},	//Raw output
	{HM2056_8BIT, 0x0028, 0xC0},	//Raw output
	{HM2056_8BIT, 0x002A, 0x2F},	// CLK - 20131106
	{HM2056_8BIT, 0x002B, 0x04},	// CLK - 20131106
	{HM2056_8BIT, 0x002C, 0x0A},	//Set default vaule for CP and resistance of LPF to 1010
	{HM2056_8BIT, 0x0004, 0x10},
	{HM2056_8BIT, 0x0006, 0x00},	// Flip/Mirror
	{HM2056_8BIT, 0x000D, 0x00},	// 20120220 to fix morie
	{HM2056_8BIT, 0x000E, 0x00},	// Binning ON
	{HM2056_8BIT, 0x000F, 0x00},	// IMGCFG
	{HM2056_8BIT, 0x0010, 0x00},	// VBI - 20131106
	{HM2056_8BIT, 0x0011, 0x5F},	// VBI - 20131119
	{HM2056_8BIT, 0x0012, 0x04},	//2012.02.08
	{HM2056_8BIT, 0x0013, 0x00},
	{HM2056_8BIT, 0x0040, 0x20},	//20120224 for BLC stable
	{HM2056_8BIT, 0x0053, 0x0A},
	{HM2056_8BIT, 0x0044, 0x06},	//enable BLC_phase2
	{HM2056_8BIT, 0x0046, 0xD8},	//enable BLC_phase1, disable BLC_phase2 dithering
	{HM2056_8BIT, 0x004A, 0x0A},	//disable BLC_phase2 hot pixel filter
	{HM2056_8BIT, 0x004B, 0x72},
	{HM2056_8BIT, 0x0075, 0x01},	//in OMUX data swap for debug usage
	{HM2056_8BIT, 0x0070, 0x5F},	// HBlank related - 20131106
	{HM2056_8BIT, 0x0071, 0xAB},	// HBlank related - 20131106
	{HM2056_8BIT, 0x0072, 0x55},	// HBlank related - 20131106
	{HM2056_8BIT, 0x0073, 0x50},
	{HM2056_8BIT, 0x0077, 0x04},
	{HM2056_8BIT, 0x0080, 0xC8},	//2012.02.08
	{HM2056_8BIT, 0x0082, 0xE2},
	{HM2056_8BIT, 0x0083, 0xF0},
	{HM2056_8BIT, 0x0085, 0x11},	//Enable Thin-Oxide Case (Kwangoh kim), Set ADC power to 100% Enable thermal sensor control bit[7] 0:on 1:off 2012 02 13 (YL)
	{HM2056_8BIT, 0x0086, 0x02},	//K.Kim, 0x2011.12.09
	{HM2056_8BIT, 0x0087, 0x80},	//K.Kim, 0x2011.12.09
	{HM2056_8BIT, 0x0088, 0x6C},
	{HM2056_8BIT, 0x0089, 0x2E},
	{HM2056_8BIT, 0x008A, 0x7D},	//20120224 for BLC stable
	{HM2056_8BIT, 0x008D, 0x20},
	{HM2056_8BIT, 0x0090, 0x00},	//1.5x(Change Gain Table )
	{HM2056_8BIT, 0x0091, 0x10},	//3x  (3x CTIA)
	{HM2056_8BIT, 0x0092, 0x11},	//6x  (3x CTIA + 2x PGA)
	{HM2056_8BIT, 0x0093, 0x12},	//12x (3x CTIA + 4x PGA)
	{HM2056_8BIT, 0x0094, 0x16},	//24x (3x CTIA + 8x PGA)
	{HM2056_8BIT, 0x0095, 0x08},	//1.5x  20120217 for color shift
	{HM2056_8BIT, 0x0096, 0x00},	//3x    20120217 for color shift
	{HM2056_8BIT, 0x0097, 0x10},	//6x    20120217 for color shift
	{HM2056_8BIT, 0x0098, 0x11},	//12x   20120217 for color shift
	{HM2056_8BIT, 0x0099, 0x12},	//24x   20120217 for color shift
	{HM2056_8BIT, 0x009A, 0x16},	//24x
	{HM2056_8BIT, 0x009B, 0x34},
	{HM2056_8BIT, 0x00A0, 0x00},
	{HM2056_8BIT, 0x00A1, 0x04},	//2012.02.06(for Ver.C)
	{HM2056_8BIT, 0x011F, 0xFF},	//simple bpc P31 & P33[4] P40 P42 P44[5]
	{HM2056_8BIT, 0x0120, 0x13},	//36:50Hz, 37:60Hz, BV_Win_Weight_En=1
	{HM2056_8BIT, 0x0121, 0x01},	//NSatScale_En=0, NSatScale=0
	{HM2056_8BIT, 0x0122, 0x39},
	{HM2056_8BIT, 0x0123, 0xC2},
	{HM2056_8BIT, 0x0124, 0xCE},
	{HM2056_8BIT, 0x0125, 0x20},
	{HM2056_8BIT, 0x0126, 0x50},
	{HM2056_8BIT, 0x0128, 0x1F},
	{HM2056_8BIT, 0x0132, 0x10},
	{HM2056_8BIT, 0x0136, 0x0A},
	{HM2056_8BIT, 0x0131, 0xB8},	//simle bpc enable[4]
	{HM2056_8BIT, 0x0140, 0x14},
	{HM2056_8BIT, 0x0141, 0x0A},
	{HM2056_8BIT, 0x0142, 0x14},
	{HM2056_8BIT, 0x0143, 0x0A},
	{HM2056_8BIT, 0x0144, 0x06},	//Sort bpc hot pixel ratio
	{HM2056_8BIT, 0x0145, 0x00},
	{HM2056_8BIT, 0x0146, 0x20},
	{HM2056_8BIT, 0x0147, 0x0A},
	{HM2056_8BIT, 0x0148, 0x10},
	{HM2056_8BIT, 0x0149, 0x0C},
	{HM2056_8BIT, 0x014A, 0x80},
	{HM2056_8BIT, 0x014B, 0x80},
	{HM2056_8BIT, 0x014C, 0x2E},
	{HM2056_8BIT, 0x014D, 0x2E},
	{HM2056_8BIT, 0x014E, 0x05},
	{HM2056_8BIT, 0x014F, 0x05},
	{HM2056_8BIT, 0x0150, 0x0D},
	{HM2056_8BIT, 0x0155, 0x00},
	{HM2056_8BIT, 0x0156, 0x10},
	{HM2056_8BIT, 0x0157, 0x0A},
	{HM2056_8BIT, 0x0158, 0x0A},
	{HM2056_8BIT, 0x0159, 0x0A},
	{HM2056_8BIT, 0x015A, 0x05},
	{HM2056_8BIT, 0x015B, 0x05},
	{HM2056_8BIT, 0x015C, 0x05},
	{HM2056_8BIT, 0x015D, 0x05},
	{HM2056_8BIT, 0x015E, 0x08},
	{HM2056_8BIT, 0x015F, 0xFF},
	{HM2056_8BIT, 0x0160, 0x50},	// OTP BPC 2line & 4line enable
	{HM2056_8BIT, 0x0161, 0x20},
	{HM2056_8BIT, 0x0162, 0x14},
	{HM2056_8BIT, 0x0163, 0x0A},
	{HM2056_8BIT, 0x0164, 0x10},	// OTP 4line Strength
	{HM2056_8BIT, 0x0165, 0x08},
	{HM2056_8BIT, 0x0166, 0x0A},
	{HM2056_8BIT, 0x018C, 0x24},
	{HM2056_8BIT, 0x018D, 0x04},	//Cluster correction enable singal from thermal sensor (YL 2012 02 13)
	{HM2056_8BIT, 0x018E, 0x00},	//Enable Thermal sensor control bit[7] (YL 2012 02 13)
	{HM2056_8BIT, 0x018F, 0x11},	//Cluster Pulse enable T1[0] T2[1] T3[2] T4[3]
	{HM2056_8BIT, 0x0190, 0x80},	//A11 BPC Strength[7:3], cluster correct P11[0]P12[1]P13[2]
	{HM2056_8BIT, 0x0191, 0x47},	//A11[0],A7[1],Sort[3],A13 AVG[6]
	{HM2056_8BIT, 0x0192, 0x48},	//A13 Strength[4:0],hot pixel detect for cluster[6]
	{HM2056_8BIT, 0x0193, 0x64},
	{HM2056_8BIT, 0x0194, 0x32},
	{HM2056_8BIT, 0x0195, 0xc8},
	{HM2056_8BIT, 0x0196, 0x96},
	{HM2056_8BIT, 0x0197, 0x64},
	{HM2056_8BIT, 0x0198, 0x32},
	{HM2056_8BIT, 0x0199, 0x14},	//A13 hot pixel th
	{HM2056_8BIT, 0x019A, 0x20},	// A13 edge detect th
	{HM2056_8BIT, 0x019B, 0x14},
	{HM2056_8BIT, 0x01BA, 0x10},	//BD
	{HM2056_8BIT, 0x01BB, 0x04},
	{HM2056_8BIT, 0x01D8, 0x40},
	{HM2056_8BIT, 0x01DE, 0x60},
	{HM2056_8BIT, 0x01E4, 0x04},
	{HM2056_8BIT, 0x01E5, 0x04},
	{HM2056_8BIT, 0x01E6, 0x04},
	{HM2056_8BIT, 0x01F2, 0x0C},
	{HM2056_8BIT, 0x01F3, 0x14},
	{HM2056_8BIT, 0x01F8, 0x04},
	{HM2056_8BIT, 0x01F9, 0x0C},
	{HM2056_8BIT, 0x01FE, 0x02},
	{HM2056_8BIT, 0x01FF, 0x04},
	{HM2056_8BIT, 0x0380, 0xFC},
	{HM2056_8BIT, 0x0381, 0x4A},
	{HM2056_8BIT, 0x0382, 0x36},
	{HM2056_8BIT, 0x038A, 0x40},
	{HM2056_8BIT, 0x038B, 0x08},
	{HM2056_8BIT, 0x038C, 0xC1},
	{HM2056_8BIT, 0x038E, 0x40},
	{HM2056_8BIT, 0x038F, 0x09},
	{HM2056_8BIT, 0x0390, 0xD0},
	{HM2056_8BIT, 0x0391, 0x05},
	{HM2056_8BIT, 0x0393, 0x80},
	{HM2056_8BIT, 0x0395, 0x21},	//AEAWB skip count
	{HM2056_8BIT, 0x0420, 0x84},	//Digital Gain offset
	{HM2056_8BIT, 0x0421, 0x00},
	{HM2056_8BIT, 0x0422, 0x00},
	{HM2056_8BIT, 0x0423, 0x83},
	{HM2056_8BIT, 0x0466, 0x14},
	{HM2056_8BIT, 0x0460, 0x01},
	{HM2056_8BIT, 0x0461, 0xFF},
	{HM2056_8BIT, 0x0462, 0xFF},
	{HM2056_8BIT, 0x0478, 0x01},
	{HM2056_8BIT, 0x047A, 0x00},	//ELOFFNRB
	{HM2056_8BIT, 0x047B, 0x00},	//ELOFFNRY
	{HM2056_8BIT, 0x0540, 0x00},
	{HM2056_8BIT, 0x0541, 0x9D},	//60Hz Flicker
	{HM2056_8BIT, 0x0542, 0x00},
	{HM2056_8BIT, 0x0543, 0xBC},	//50Hz Flicker
	{HM2056_8BIT, 0x05E4, 0x00},	//Windowing
	{HM2056_8BIT, 0x05E5, 0x00},
	{HM2056_8BIT, 0x05E6, 0x53},
	{HM2056_8BIT, 0x05E7, 0x06},
	{HM2056_8BIT, 0x0698, 0x00},
	{HM2056_8BIT, 0x0699, 0x00},
	{HM2056_8BIT, 0x0B20, 0xAE},	//Set clock lane is on at sending packet, Patrick: 0xAE
	{HM2056_8BIT, 0x0078, 0x80},	//Set clock lane is on at sending packet, Patrick: new setting
	{HM2056_8BIT, 0x007C, 0x09},	//pre-hsync setting, Patrick: 0x09
	{HM2056_8BIT, 0x007D, 0x3E},	//pre-vsync setting
	{HM2056_8BIT, 0x0B02, 0x01},	// TLPX WIDTH, Add by Wilson, 20111114
	{HM2056_8BIT, 0x0B03, 0x03},
	{HM2056_8BIT, 0x0B04, 0x01},
	{HM2056_8BIT, 0x0B05, 0x08},
	{HM2056_8BIT, 0x0B06, 0x02},
	{HM2056_8BIT, 0x0B07, 0x28},	//MARK1 WIDTH
	{HM2056_8BIT, 0x0B0E, 0x0B},	//CLK FRONT PORCH WIDTH
	{HM2056_8BIT, 0x0B0F, 0x04},	//CLK BACK PORCH WIDTH
	{HM2056_8BIT, 0x0B22, 0x02},	//HS_EXIT Eanble
	{HM2056_8BIT, 0x0B39, 0x03},	//Clock Lane HS_EXIT WIDTH(at least 100ns)
	{HM2056_8BIT, 0x0B11, 0x7F},	//Clock Lane LP Driving Strength
	{HM2056_8BIT, 0x0B12, 0x7F},	//Data Lane LP Driving Strength
	{HM2056_8BIT, 0x0B17, 0xE0},	//D-PHY Power Down Control
	{HM2056_8BIT, 0x0B22, 0x02},
	{HM2056_8BIT, 0x0B30, 0x0F},	//D-PHY Reset, set to 1 for normal operation
	{HM2056_8BIT, 0x0B31, 0x02},	//[1]: PHASE_SEL = 1 First Data at rising edge
	{HM2056_8BIT, 0x0B32, 0x00},	//[4]: DBG_ULPM
	{HM2056_8BIT, 0x0B33, 0x00},	//DBG_SEL
	{HM2056_8BIT, 0x0B35, 0x00},
	{HM2056_8BIT, 0x0B36, 0x00},
	{HM2056_8BIT, 0x0B37, 0x00},
	{HM2056_8BIT, 0x0B38, 0x00},
	{HM2056_8BIT, 0x0B39, 0x03},
	{HM2056_8BIT, 0x0B3A, 0x00},	//CLK_HS_EXIT, Add by Wilson, 20111114
	{HM2056_8BIT, 0x0B3B, 0x12},	//Turn on PHY LDO
	{HM2056_8BIT, 0x0B3F, 0x01},	//MIPI reg delay, Add by Wilson, 20111114
	{HM2056_8BIT, 0x0024, 0x40},	//[6]: MIPI Enable
	{HM2056_TOK_TERM, 0, 0}
};

static struct hm2056_reg const hm2056_uxga_14fps[] = {
	{HM2056_8BIT, 0x0024, 0x00},
	{HM2056_8BIT, 0x0006, 0x00},
	{HM2056_8BIT, 0x000D, 0x00},
	{HM2056_8BIT, 0x000E, 0x00},
	{HM2056_8BIT, 0x0010, 0x00},
	{HM2056_8BIT, 0x0011, 0x8A},
	{HM2056_8BIT, 0x0012, 0x04},
	{HM2056_8BIT, 0x0013, 0x00},
	{HM2056_8BIT, 0x002A, 0x2F},
	{HM2056_8BIT, 0x0071, 0xAB},
	{HM2056_8BIT, 0x0074, 0x13},
	{HM2056_8BIT, 0x0082, 0xE2},
	{HM2056_8BIT, 0x0131, 0xB8},
	{HM2056_8BIT, 0x0144, 0x06},
	{HM2056_8BIT, 0x0190, 0x80},
	{HM2056_8BIT, 0x0192, 0x48},
	{HM2056_8BIT, 0x05E6, 0x53},
	{HM2056_8BIT, 0x05E7, 0x06},
	{HM2056_8BIT, 0x05EA, 0xC3},
	{HM2056_8BIT, 0x05EB, 0x04},
	{HM2056_8BIT, 0x0024, 0x40},
	{HM2056_TOK_DELAY, 0, 80}, // Delay longer than 1 frame time ~71ms
	{HM2056_TOK_TERM, 0, 0}
};

static struct hm2056_reg const hm2056_uxga_16_9_18fps[] = {
	{HM2056_8BIT, 0x0024, 0x00},
	{HM2056_8BIT, 0x0006, 0x14},
	{HM2056_8BIT, 0x000D, 0x00},
	{HM2056_8BIT, 0x000E, 0x00},
	{HM2056_8BIT, 0x0010, 0x00},
	{HM2056_8BIT, 0x0011, 0x8A},
	{HM2056_8BIT, 0x0012, 0x04},
	{HM2056_8BIT, 0x0013, 0x00},
	{HM2056_8BIT, 0x002A, 0x2F},
	{HM2056_8BIT, 0x0071, 0xAB},
	{HM2056_8BIT, 0x0074, 0x13},
	{HM2056_8BIT, 0x0082, 0xE2},
	{HM2056_8BIT, 0x0131, 0xB8},
	{HM2056_8BIT, 0x0144, 0x06},
	{HM2056_8BIT, 0x0190, 0x80},
	{HM2056_8BIT, 0x0192, 0x48},
	{HM2056_8BIT, 0x05E6, 0x53},
	{HM2056_8BIT, 0x05E7, 0x06},
	{HM2056_8BIT, 0x05EA, 0x8F},
	{HM2056_8BIT, 0x05EB, 0x03},
	{HM2056_8BIT, 0x0024, 0x40},
	{HM2056_TOK_DELAY, 0, 60}, // Delay longer than 1 frame time ~55ms
	{HM2056_TOK_TERM, 0, 0}
};


/*
 * 1296x732
 */
static struct hm2056_reg const hm2056_720p_25fps[] = {
	{HM2056_8BIT, 0x0024, 0x00},
	{HM2056_8BIT, 0x0006, 0x08},	// Flip/Mirror
	{HM2056_8BIT, 0x000D, 0x00},	// 20120220 to fix morie
	{HM2056_8BIT, 0x000E, 0x00},	// Binning ON
	{HM2056_8BIT, 0x0010, 0x00},
	{HM2056_8BIT, 0x0011, 0xBE},
	{HM2056_8BIT, 0x0012, 0x08},	//2012.02.08
	{HM2056_8BIT, 0x0013, 0x00},
	{HM2056_8BIT, 0x002A, 0x2F},
	{HM2056_8BIT, 0x0071, 0x99},
	{HM2056_8BIT, 0x0074, 0x13},
	{HM2056_8BIT, 0x0082, 0xE2},
	{HM2056_8BIT, 0x0131, 0xB8},	//simle bpc enable[4]
	{HM2056_8BIT, 0x0144, 0x04},	//Sort bpc hot pixel ratio
	{HM2056_8BIT, 0x0190, 0x87},	//A11 BPC Strength[7:3], cluster correct P11[0]P12[1]P13[2]
	{HM2056_8BIT, 0x0192, 0x50},	//A13 Strength[4:0],hot pixel detect for cluster[6]
	{HM2056_8BIT, 0x05E6, 0x0F},
	{HM2056_8BIT, 0x05E7, 0x05},
	{HM2056_8BIT, 0x05EA, 0xDB},
	{HM2056_8BIT, 0x05EB, 0x02},
	{HM2056_8BIT, 0x0024, 0x40},
	{HM2056_TOK_DELAY, 0, 40}, //Delay longer than 1 frame time ~37ms
	{HM2056_TOK_TERM, 0, 0}
};

/*
 * 810 x 626
 */
static struct hm2056_reg const hm2056_svga_30fps[] = {
	{HM2056_8BIT, 0x0024, 0x00},
	{HM2056_8BIT, 0x0006, 0x00},	// Flip/Mirror
	{HM2056_8BIT, 0x000D, 0x11},	// 20120220 to fix morie
	{HM2056_8BIT, 0x000E, 0x11},	// Binning ON
	{HM2056_8BIT, 0x0010, 0x01},
	{HM2056_8BIT, 0x0011, 0x3A},
	{HM2056_8BIT, 0x0012, 0x1C},	//2012.02.08
	{HM2056_8BIT, 0x0013, 0x01},
	{HM2056_8BIT, 0x002A, 0x2C},
	{HM2056_8BIT, 0x0071, 0xFF},
	{HM2056_8BIT, 0x0074, 0x1B},
	{HM2056_8BIT, 0x0082, 0xA2},
	{HM2056_8BIT, 0x0131, 0xB8},
	{HM2056_8BIT, 0x0144, 0x06},
	{HM2056_8BIT, 0x0190, 0x80},
	{HM2056_8BIT, 0x0192, 0x48},
	{HM2056_8BIT, 0x05E6, 0x2B},
	{HM2056_8BIT, 0x05E7, 0x03},
	{HM2056_8BIT, 0x05EA, 0x71},
	{HM2056_8BIT, 0x05EB, 0x02},
	{HM2056_8BIT, 0x0024, 0x40},
	{HM2056_TOK_DELAY, 0, 40}, // Delay longer than 1 frame time ~33ms
	{HM2056_TOK_TERM, 0, 0}
};

/*
 * Modes supported by the hm2056_raw driver.
 * Please, keep them in ascending order.
 */
 static struct hm2056_raw_res_struct hm2056_raw_res_preview[] = {
	{
	.desc	= "hm2056_svga_30fps",
	.width	= 812,
	.height	= 626,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2056_svga_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1623,
	.vertical_start = 0,
	.vertical_end = 1251,
	.pixel_clk = 36000000,
	.line_length_pck = 1223,
	.frame_length_lines = 940,
	.bin_factor_x = 2,
	.bin_factor_y = 2,
	.bin_mode = 1,
	},
	{
	.desc	= "hm2056_720p_25fps",
	.width	= 1296,
	.height	= 732,
	.fps	= 27,
	.used	= 0,
	.regs	= hm2056_720p_25fps,
	.skip_frames = 1,
	.horizontal_start = 162,
	.horizontal_end = 1457,
	.vertical_start = 244,
	.vertical_end = 975,
	.pixel_clk = 39000000,
	.line_length_pck = 1619,
	.frame_length_lines = 922,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
	{
	.desc	= "hm2056_uxga_16_9_18fps",
	.width	= 1620,
	.height	= 912,
	.fps	= 18,
	.used	= 0,
	.regs	= hm2056_uxga_16_9_18fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1619,
	.vertical_start = 0,
	.vertical_end = 911,
	.pixel_clk = 39000000,
	.line_length_pck = 1919,
	.frame_length_lines = 1050,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
	{
	.desc	= "hm2056_UXGA_14fps",
	.width	= 1620,
	.height	= 1220,
	.fps	= 14,
	.used	= 0,
	.regs	= hm2056_uxga_14fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1619,
	.vertical_start = 0,
	.vertical_end = 1219,
	.pixel_clk = 39000000,
	.line_length_pck = 1919,
	.frame_length_lines = 1358,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};

#define N_RES_PREVIEW (ARRAY_SIZE(hm2056_raw_res_preview))

static struct hm2056_raw_res_struct hm2056_raw_res_still[] = {
	{
	.desc	= "hm2056_svga_30fps",
	.width	= 812,
	.height	= 626,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2056_svga_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1623,
	.vertical_start = 0,
	.vertical_end = 1251,
	.pixel_clk = 36000000,
	.line_length_pck = 1223,
	.frame_length_lines = 940,
	.bin_factor_x = 2,
	.bin_factor_y = 2,
	.bin_mode = 1,
	},
	{
	.desc	= "hm2056_720p_25fps",
	.width	= 1296,
	.height	= 732,
	.fps	= 27,
	.used	= 0,
	.regs	= hm2056_720p_25fps,
	.skip_frames = 1,
	.horizontal_start = 162,
	.horizontal_end = 1457,
	.vertical_start = 244,
	.vertical_end = 975,
	.pixel_clk = 39000000,
	.line_length_pck = 1619,
	.frame_length_lines = 922,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
	{
	.desc	= "hm2056_uxga_16_9_18fps",
	.width	= 1620,
	.height	= 912,
	.fps	= 18,
	.used	= 0,
	.regs	= hm2056_uxga_16_9_18fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1619,
	.vertical_start = 0,
	.vertical_end = 911,
	.pixel_clk = 39000000,
	.line_length_pck = 1919,
	.frame_length_lines = 1050,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
	{
	.desc	= "hm2056_uxga_14fps",
	.width	= 1620,
	.height	= 1220,
	.fps	= 14,
	.used	= 0,
	.regs	= hm2056_uxga_14fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1619,
	.vertical_start = 0,
	.vertical_end = 1219,
	.pixel_clk = 39000000,
	.line_length_pck = 1919,
	.frame_length_lines = 1358,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};

#define N_RES_STILL (ARRAY_SIZE(hm2056_raw_res_still))

static struct hm2056_raw_res_struct hm2056_raw_res_video[] = {
	{
	.desc	= "hm2056_svga_30fps",
	.width	= 812,
	.height	= 626,
	.fps	= 30,
	.used	= 0,
	.regs	= hm2056_svga_30fps,
	.skip_frames = 1,
	.horizontal_start = 0,
	.horizontal_end = 1619,
	.vertical_start = 0,
	.vertical_end = 1219,
	.pixel_clk = 36000000,
	.line_length_pck = 1223,
	.frame_length_lines = 940,
	.bin_factor_x = 2,
	.bin_factor_y = 2,
	.bin_mode = 1,
	},
	{
	.desc	= "hm2056_720p_25fps",
	.width	= 1296,
	.height	= 732,
	.fps	= 25,
	.used	= 0,
	.regs	= hm2056_720p_25fps,
	.skip_frames = 1,
	.horizontal_start = 162,
	.horizontal_end = 1457,
	.vertical_start = 244,
	.vertical_end = 975,
	.pixel_clk = 39000000,
	.line_length_pck = 1619,
	.frame_length_lines = 922,
	.bin_factor_x = 1,
	.bin_factor_y = 1,
	.bin_mode = 0,
	},
};


#define N_RES_VIDEO (ARRAY_SIZE(hm2056_raw_res_video))
#endif
