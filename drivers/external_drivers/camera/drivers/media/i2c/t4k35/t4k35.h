#ifndef __T4K35_H__
#define __T4K35_H__
#include "common.h"

/************************** settings for tsb *************************/
static struct tsb_reg const t4k35_Full_3280x2464_20fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x01},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x09},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0xF0},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x0E},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0xBE},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x0C},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0xD0},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x09},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0xA0},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x00},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x10},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x00},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x00},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x0C},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0xD0},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x09},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0xA0},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x07},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x80},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x00},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x00},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x00},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_Full_3280x2464_6fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x01},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x1D},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0x28},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x10},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0xC0},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x0C},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0xD0},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x09},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0xA0},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x00},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x10},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x00},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x00},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x0C},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0xD0},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x09},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0xA0},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x07},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x80},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x00},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x00},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x00},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_Full_3280x1852_26fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x01},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x07},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0xE0},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x0E},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0x50},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x01},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x30},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x08},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x6B},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x0C},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0xD0},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x07},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0x3C},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x00},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x10},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x00},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x00},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x0C},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0xD0},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x07},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0x3C},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x07},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x80},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x00},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x00},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_Full_3280x1852_8fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x01},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x17},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0xA4},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x0F},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0x7E},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x01},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x30},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x08},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x6B},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x0C},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0xD0},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x07},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0x3C},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x00},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x10},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x00},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x00},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x0C},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0xD0},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x07},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0x3C},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x07},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x80},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x00},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x00},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_Bin_1632x1228_30fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x02},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x07},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0x0A},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x0D},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0xE0},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x06},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0x60},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x04},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0xCC},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x00},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x10},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x04},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x02},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x06},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0x60},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x04},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0xCC},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x03},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0xC0},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x01},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x01},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_Bin_1616x916_30fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x02},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x07},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0x0A},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x0D},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0xE0},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x06},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0x50},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x03},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0x94},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x00},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x10},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x0C},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x9E},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x06},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0x50},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x03},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0x94},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x03},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0xC0},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x01},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x01},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

/*****************************video************************/

static struct tsb_reg const t4k35_FHD_1936x1096_30fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x05},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x01},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x07},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0xA0},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x0F},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0x5E},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x01},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x30},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x08},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x6B},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x07},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0x90},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x04},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0x48},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x02},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x1B},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x06},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x00},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x0C},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0xC4},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x07},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0x3A},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x07},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x80},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x00},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x00},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_HD_1296x736_30fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x03},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x05},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0x98},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x11},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0x76},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x05},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0x10},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x02},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0xE0},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x02},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x14},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x0A},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x9C},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x06},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0x54},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x03},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0x98},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x02},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x80},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x01},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x01},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_Bin_656x496_30fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x06},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x05},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0x98},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x11},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0x76},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x02},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0x90},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x01},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0xF0},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x02},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x27},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x08},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x02},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x06},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0x58},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x04},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0xCC},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x01},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x40},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x01},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x01},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_Bin_192x160_30fps_2lane[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030B, 0x06},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0340, 0x05},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0x98},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x11},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0x76},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x00},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0xC0},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x00},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0xA0},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x02},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0404, 0x7A},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0409, 0x52},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x00},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x05},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0xC4},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x04},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0xCE},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0820, 0x01},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x40},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0900, 0x01},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x01},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x32F7, 0x01},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_TOK_TERM, 0, 0}
};

static struct tsb_reg const t4k35_init_settings[] = {
	GROUPED_PARAMETER_HOLD_ENABLE,
	{TSB_8BIT, 0x0101, 0x00},    // -/-/-/-/-/-/IMAGE_ORIENT[1:0];
	{TSB_8BIT, 0x0103, 0x00},    // -/-/-/-/-/-/MIPI_RST/SOFTWARE_RESET;
	{TSB_8BIT, 0x0104, 0x00},    // -/-/-/-/-/-/-/GROUP_PARA_HOLD;
	{TSB_8BIT, 0x0105, 0x00},    // -/-/-/-/-/-/-/MSK_CORRUPT_FR;
	{TSB_8BIT, 0x0110, 0x00},    // -/-/-/-/-/CSI_CHAN_IDNTF[2:0];
	{TSB_8BIT, 0x0111, 0x02},    // -/-/-/-/-/-/CSI_SIGNAL_MOD[1:0];
	{TSB_8BIT, 0x0112, 0x0A},    // CSI_DATA_FORMAT[15:8];
	{TSB_8BIT, 0x0113, 0x0A},    // CSI_DATA_FORMAT[7:0];
	{TSB_8BIT, 0x0114, 0x01},    // -/-/-/-/-/-/CSI_LANE_MODE[1:0];
	{TSB_8BIT, 0x0130, 0x13},    // EXTCLK_FRQ_MHZ[15:8];
	{TSB_8BIT, 0x0131, 0x33},    // EXTCLK_FRQ_MHZ[7:0];
	{TSB_8BIT, 0x0141, 0x00},    // -/-/-/-/-/cont_select_1B/CTX_SW_CTL/R_CONT_STATUS;
	{TSB_8BIT, 0x0142, 0x00},    // -/-/-/-/CONT_MDSEL_FRVAL[1:0]/CONT_FRCNT_MSK/CONT_GRHOLD_MSK;
	{TSB_8BIT, 0x0143, 0x00},    // R_FRAME_COUNT[7:0];
	{TSB_8BIT, 0x0202, 0x09},    // COAR_INTEGR_TIM[15:8];
	{TSB_8BIT, 0x0203, 0xB2},    // COAR_INTEGR_TIM[7:0];
	{TSB_8BIT, 0x0204, 0x00},    // -/-/-/-/ANA_GA_CODE_GL[11:8];
	{TSB_8BIT, 0x0205, 0x28},    // ANA_GA_CODE_GL[7:0];
	{TSB_8BIT, 0x0210, 0x01},    // -/-/-/-/-/-/DG_GA_GREENR[9:8];
	{TSB_8BIT, 0x0211, 0x00},    // DG_GA_GREENR[7:0];
	{TSB_8BIT, 0x0212, 0x01},    // -/-/-/-/-/-/DG_GA_RED[9:8];
	{TSB_8BIT, 0x0213, 0x00},    // DG_GA_RED[7:0];
	{TSB_8BIT, 0x0214, 0x01},    // -/-/-/-/-/-/DG_GA_BLUE[9:8];
	{TSB_8BIT, 0x0215, 0x00},    // DG_GA_BLUE[7:0];
	{TSB_8BIT, 0x0216, 0x01},    // -/-/-/-/-/-/DG_GA_GREENB[9:8];
	{TSB_8BIT, 0x0217, 0x00},    // DG_GA_GREENB[7:0];
	{TSB_8BIT, 0x0230, 0x00},    // -/-/-/HDR_MODE[4:0];
	{TSB_8BIT, 0x0232, 0x04},    // HDR_RATIO_1[7:0];
	{TSB_8BIT, 0x0234, 0x00},    // HDR_SHT_INTEGR_TIM[15:8];
	{TSB_8BIT, 0x0235, 0x19},    // HDR_SHT_INTEGR_TIM[7:0];
	{TSB_8BIT, 0x0301, 0x01},    // -/-/-/-/VT_PIX_CLK_DIV[3:0];
	{TSB_8BIT, 0x0303, 0x06},    // -/-/-/-/VT_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x0305, 0x03},    // -/-/-/-/-/PRE_PLL_CLK_DIV[2:0];
	{TSB_8BIT, 0x0306, 0x00},    // -/-/-/-/-/-/-/PLL_MULTIPLIER[8];
	{TSB_8BIT, 0x0307, 0xC8},    // PLL_MULTIPLIER[7:0];
	{TSB_8BIT, 0x030B, 0x01},    // -/-/-/-/OP_SYS_CLK_DIV[3:0];
	{TSB_8BIT, 0x030D, 0x03},    // -/-/-/-/-/PRE_PLL_ST_CLK_DIV[2:0];
	{TSB_8BIT, 0x030E, 0x00},    // -/-/-/-/-/-/-/PLL_MULT_ST[8];
	{TSB_8BIT, 0x030F, 0x78},    // PLL_MULT_ST[7:0];
	{TSB_8BIT, 0x0310, 0x00},    // -/-/-/-/-/-/-/OPCK_PLLSEL;
	{TSB_8BIT, 0x0340, 0x09},    // FR_LENGTH_LINES[15:8];
	{TSB_8BIT, 0x0341, 0xB8},    // FR_LENGTH_LINES[7:0];
	{TSB_8BIT, 0x0342, 0x0D},    // LINE_LENGTH_PCK[15:8];
	{TSB_8BIT, 0x0343, 0x9C},    // LINE_LENGTH_PCK[7:0];
	{TSB_8BIT, 0x0344, 0x00},    // -/-/-/-/H_CROP[3:0];
	{TSB_8BIT, 0x0346, 0x00},    // Y_ADDR_START[15:8];
	{TSB_8BIT, 0x0347, 0x00},    // Y_ADDR_START[7:0];
	{TSB_8BIT, 0x034A, 0x09},    // Y_ADDR_END[15:8];
	{TSB_8BIT, 0x034B, 0x9F},    // Y_ADDR_END[7:0];
	{TSB_8BIT, 0x034C, 0x0C},    // X_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034D, 0xD0},    // X_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x034E, 0x09},    // Y_OUTPUT_SIZE[15:8];
	{TSB_8BIT, 0x034F, 0xA0},    // Y_OUTPUT_SIZE[7:0];
	{TSB_8BIT, 0x0401, 0x00},    // -/-/-/-/-/-/SCALING_MODE[1:0];
	{TSB_8BIT, 0x0403, 0x00},    // -/-/-/-/-/-/SPATIAL_SAMPLING[1:0];
	{TSB_8BIT, 0x0404, 0x10},    // SCALE_M[7:0];
	{TSB_8BIT, 0x0408, 0x00},    // DCROP_XOFS[15:8];
	{TSB_8BIT, 0x0409, 0x00},    // DCROP_XOFS[7:0];
	{TSB_8BIT, 0x040A, 0x00},    // DCROP_YOFS[15:8];
	{TSB_8BIT, 0x040B, 0x00},    // DCROP_YOFS[7:0];
	{TSB_8BIT, 0x040C, 0x0C},    // DCROP_WIDTH[15:8];
	{TSB_8BIT, 0x040D, 0xD0},    // DCROP_WIDTH[7:0];
	{TSB_8BIT, 0x040E, 0x09},    // DCROP_HIGT[15:8];
	{TSB_8BIT, 0x040F, 0xA0},    // DCROP_HIGT[7:0];
	{TSB_8BIT, 0x0601, 0x00},    // TEST_PATT_MODE[7:0];
	{TSB_8BIT, 0x0602, 0x02},    // -/-/-/-/-/-/TEST_DATA_RED[9:8];
	{TSB_8BIT, 0x0603, 0xC0},    // TEST_DATA_RED[7:0];
	{TSB_8BIT, 0x0604, 0x02},    // -/-/-/-/-/-/TEST_DATA_GREENR[9:8];
	{TSB_8BIT, 0x0605, 0xC0},    // TEST_DATA_GREENR[7:0];
	{TSB_8BIT, 0x0606, 0x02},    // -/-/-/-/-/-/TEST_DATA_BLUE[9:8];
	{TSB_8BIT, 0x0607, 0xC0},    // TEST_DATA_BLUE[7:0];
	{TSB_8BIT, 0x0608, 0x02},    // -/-/-/-/-/-/TEST_DATA_GREENB[9:8];
	{TSB_8BIT, 0x0609, 0xC0},    // TEST_DATA_GREENB[7:0];
	{TSB_8BIT, 0x060A, 0x00},    // HO_CURS_WIDTH[15:8];
	{TSB_8BIT, 0x060B, 0x00},    // HO_CURS_WIDTH[7:0];
	{TSB_8BIT, 0x060C, 0x00},    // HO_CURS_POSITION[15:8];
	{TSB_8BIT, 0x060D, 0x00},    // HO_CURS_POSITION[7:0];
	{TSB_8BIT, 0x060E, 0x00},    // VE_CURS_WIDTH[15:8];
	{TSB_8BIT, 0x060F, 0x00},    // VE_CURS_WIDTH[7:0];
	{TSB_8BIT, 0x0610, 0x00},    // VE_CURS_POSITION[15:8];
	{TSB_8BIT, 0x0611, 0x00},    // VE_CURS_POSITION[7:0];
	{TSB_8BIT, 0x0800, 0x88},    // TCLK_POST[7:3]/-/-/-/;
	{TSB_8BIT, 0x0801, 0x38},    // THS_PREPARE[7:3]/-/-/-/;
	{TSB_8BIT, 0x0802, 0x78},    // THS_ZERO[7:3]/-/-/-/;
	{TSB_8BIT, 0x0803, 0x48},    // THS_TRAIL[7:3]/-/-/-/;
	{TSB_8BIT, 0x0804, 0x48},    // TCLK_TRAIL[7:3]/-/-/-/;
	{TSB_8BIT, 0x0805, 0x40},    // TCLK_PREPARE[7:3]/-/-/-/;
	{TSB_8BIT, 0x0806, 0x00},    // TCLK_ZERO[7:3]/-/-/-/;
	{TSB_8BIT, 0x0807, 0x48},    // TLPX[7:3]/-/-/-/;
	{TSB_8BIT, 0x0808, 0x01},    // -/-/-/-/-/-/DPHY_CTRL[1:0];
	{TSB_8BIT, 0x0820, 0x0A},    // MSB_LBRATE[31:24];
	{TSB_8BIT, 0x0821, 0x30},    // MSB_LBRATE[23:16];
	{TSB_8BIT, 0x0822, 0x00},    // MSB_LBRATE[15:8];
	{TSB_8BIT, 0x0823, 0x00},    // MSB_LBRATE[7:0];
	{TSB_8BIT, 0x0900, 0x00},    // -/-/-/-/-/-/H_BIN[1:0];
	{TSB_8BIT, 0x0901, 0x00},    // -/-/-/-/-/-/V_BIN_MODE[1:0];
	{TSB_8BIT, 0x0902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING[1:0];
	{TSB_8BIT, 0x0A05, 0x01},    // -/-/-/-/-/-/-/MAP_DEF_EN;
	{TSB_8BIT, 0x0A06, 0x01},    // -/-/-/-/-/-/-/SGL_DEF_EN;
	{TSB_8BIT, 0x0A07, 0x98},    // SGL_DEF_W[7:0];
	{TSB_8BIT, 0x0A0A, 0x01},    // -/-/-/-/-/-/-/COMB_CPLT_SGL_DEF_EN;
	{TSB_8BIT, 0x0A0B, 0x98},    // COMB_CPLT_SGL_DEF_W[7:0];
	{TSB_8BIT, 0x0C00, 0x00},    // -/-/-/-/-/-/GLBL_RST_CTRL1[1:0];
	{TSB_8BIT, 0x0C02, 0x00},    // GLBL_RST_CFG_1[7:0];
	{TSB_8BIT, 0x0C04, 0x00},    // TRDY_CTRL[15:8];
	{TSB_8BIT, 0x0C05, 0x32},    // TRDY_CTRL[7:0];
	{TSB_8BIT, 0x0C06, 0x00},    // TRDOUT_CTRL[15:8];
	{TSB_8BIT, 0x0C07, 0x10},    // TRDOUT_CTRL[7:0];
	{TSB_8BIT, 0x0C08, 0x00},    // TSHT_STB_DLY_CTRL[15:8];
	{TSB_8BIT, 0x0C09, 0x49},    // TSHT_STB_DLY_CTRL[7:0];
	{TSB_8BIT, 0x0C0A, 0x01},    // TSHT_STB_WDTH_CTRL[15:8];
	{TSB_8BIT, 0x0C0B, 0x68},    // TSHT_STB_WDTH_CTRL[7:0];
	{TSB_8BIT, 0x0C0C, 0x00},    // TFLSH_STB_DLY_CTRL[15:8];
	{TSB_8BIT, 0x0C0D, 0x34},    // TFLSH_STB_DLY_CTRL[7:0];
	{TSB_8BIT, 0x0C0E, 0x00},    // TFLSH_STB_WDTH_CTRL[15:8];
	{TSB_8BIT, 0x0C0F, 0x40},    // TFLSH_STB_WDTH_CTRL[7:0];
	{TSB_8BIT, 0x0C12, 0x01},    // FLASH_ADJ[7:0];
	{TSB_8BIT, 0x0C14, 0x00},    // FLASH_LINE[15:8];
	{TSB_8BIT, 0x0C15, 0x01},    // FLASH_LINE[7:0];
	{TSB_8BIT, 0x0C16, 0x00},    // FLASH_DELAY[15:8];
	{TSB_8BIT, 0x0C17, 0x20},    // FLASH_DELAY[7:0];
	{TSB_8BIT, 0x0C18, 0x00},    // FLASH_WIDTH[15:8];
	{TSB_8BIT, 0x0C19, 0x40},    // FLASH_WIDTH[7:0];
	{TSB_8BIT, 0x0C1A, 0x00},    // -/-/FLASH_MODE[5:0];
	{TSB_8BIT, 0x0C1B, 0x00},    // -/-/-/-/-/-/-/FLASH_TRG;
	{TSB_8BIT, 0x0F00, 0x00},    // -/-/-/-/-/ABF_LUT_CTL[2:0];
	{TSB_8BIT, 0x0F01, 0x01},    // -/-/-/-/-/-/ABF_LUT_MODE[1:0];
	{TSB_8BIT, 0x0F02, 0x01},    // ABF_ES_A[15:8];
	{TSB_8BIT, 0x0F03, 0x40},    // ABF_ES_A[7:0];
	{TSB_8BIT, 0x0F04, 0x00},    // -/-/-/-/ABF_AG_A[11:8];
	{TSB_8BIT, 0x0F05, 0x40},    // ABF_AG_A[7:0];
	{TSB_8BIT, 0x0F06, 0x01},    // -/-/-/-/-/-/-/ABF_DG_GR_A[8];
	{TSB_8BIT, 0x0F07, 0x00},    // ABF_DG_GR_A[7:0];
	{TSB_8BIT, 0x0F08, 0x01},    // -/-/-/-/-/-/-/ABF_DG_R_A[8];
	{TSB_8BIT, 0x0F09, 0x00},    // ABF_DG_R_A[7:0];
	{TSB_8BIT, 0x0F0A, 0x01},    // -/-/-/-/-/-/-/ABF_DG_B_A[8];
	{TSB_8BIT, 0x0F0B, 0x00},    // ABF_DG_B_A[7:0];
	{TSB_8BIT, 0x0F0C, 0x01},    // -/-/-/-/-/-/-/ABF_DG_GB_A[8];
	{TSB_8BIT, 0x0F0D, 0x00},    // ABF_DG_GB_A[7:0];
	{TSB_8BIT, 0x0F0E, 0x00},    // -/-/-/-/-/-/-/F_ENTRY_A;
	{TSB_8BIT, 0x0F0F, 0x01},    // ABF_ES_B[15:8];
	{TSB_8BIT, 0x0F10, 0x50},    // ABF_ES_B[7:0];
	{TSB_8BIT, 0x0F11, 0x00},    // -/-/-/-/ABF_AG_B[11:8];
	{TSB_8BIT, 0x0F12, 0x50},    // ABF_AG_B[7:0];
	{TSB_8BIT, 0x0F13, 0x01},    // -/-/-/-/-/-/-/ABF_DG_GR_B[8];
	{TSB_8BIT, 0x0F14, 0x00},    // ABF_DG_GR_B[7:0];
	{TSB_8BIT, 0x0F15, 0x01},    // -/-/-/-/-/-/-/ABF_DG_R_B[8];
	{TSB_8BIT, 0x0F16, 0x00},    // ABF_DG_R_B[7:0];
	{TSB_8BIT, 0x0F17, 0x01},    // -/-/-/-/-/-/-/ABF_DG_B_B[8];
	{TSB_8BIT, 0x0F18, 0x00},    // ABF_DG_B_B[7:0];
	{TSB_8BIT, 0x0F19, 0x01},    // -/-/-/-/-/-/-/ABF_DG_GB_B[8];
	{TSB_8BIT, 0x0F1A, 0x00},    // ABF_DG_GB_B[7:0];
	{TSB_8BIT, 0x0F1B, 0x00},    // -/-/-/-/-/-/-/F_ENTRY_B;
	{TSB_8BIT, 0x0F1C, 0x01},    // ABF_ES_C[15:8];
	{TSB_8BIT, 0x0F1D, 0x60},    // ABF_ES_C[7:0];
	{TSB_8BIT, 0x0F1E, 0x00},    // -/-/-/-/ABF_AG_C[11:8];
	{TSB_8BIT, 0x0F1F, 0x60},    // ABF_AG_C[7:0];
	{TSB_8BIT, 0x0F20, 0x01},    // -/-/-/-/-/-/-/ABF_DG_GR_C[8];
	{TSB_8BIT, 0x0F21, 0x00},    // ABF_DG_GR_C[7:0];
	{TSB_8BIT, 0x0F22, 0x01},    // -/-/-/-/-/-/-/ABF_DG_R_C[8];
	{TSB_8BIT, 0x0F23, 0x00},    // ABF_DG_R_C[7:0];
	{TSB_8BIT, 0x0F24, 0x01},    // -/-/-/-/-/-/-/ABF_DG_B_C[8];
	{TSB_8BIT, 0x0F25, 0x00},    // ABF_DG_B_C[7:0];
	{TSB_8BIT, 0x0F26, 0x01},    // -/-/-/-/-/-/-/ABF_DG_GB_C[8];
	{TSB_8BIT, 0x0F27, 0x00},    // ABF_DG_GB_C[7:0];
	{TSB_8BIT, 0x0F28, 0x00},    // -/-/-/-/-/-/-/F_ENTRY_C;
	{TSB_8BIT, 0x1101, 0x00},    // -/-/-/-/-/-/IMAGE_ORIENT_1B[1:0];
	{TSB_8BIT, 0x1143, 0x00},    // R_FRAME_COUNT_1B[7:0];
	{TSB_8BIT, 0x1202, 0x00},    // COAR_INTEGR_TIM_1B[15:8];
	{TSB_8BIT, 0x1203, 0x19},    // COAR_INTEGR_TIM_1B[7:0];
	{TSB_8BIT, 0x1204, 0x00},    // -/-/-/-/ANA_GA_CODE_GL_1B[11:8];
	{TSB_8BIT, 0x1205, 0x40},    // ANA_GA_CODE_GL_1B[7:0];
	{TSB_8BIT, 0x1210, 0x01},    // -/-/-/-/-/-/DG_GA_GREENR_1B[9:8];
	{TSB_8BIT, 0x1211, 0x00},    // DG_GA_GREENR_1B[7:0];
	{TSB_8BIT, 0x1212, 0x01},    // -/-/-/-/-/-/DG_GA_RED_1B[9:8];
	{TSB_8BIT, 0x1213, 0x00},    // DG_GA_RED_1B[7:0];
	{TSB_8BIT, 0x1214, 0x01},    // -/-/-/-/-/-/DG_GA_BLUE_1B[9:8];
	{TSB_8BIT, 0x1215, 0x00},    // DG_GA_BLUE_1B[7:0];
	{TSB_8BIT, 0x1216, 0x01},    // -/-/-/-/-/-/DG_GA_GREENB_1B[9:8];
	{TSB_8BIT, 0x1217, 0x00},    // DG_GA_GREENB_1B[7:0];
	{TSB_8BIT, 0x1230, 0x00},    // -/-/-/HDR_MODE_1B[4:0];
	{TSB_8BIT, 0x1232, 0x04},    // HDR_RATIO_1_1B[7:0];
	{TSB_8BIT, 0x1234, 0x00},    // HDR_SHT_INTEGR_TIM_1B[15:8];
	{TSB_8BIT, 0x1235, 0x19},    // HDR_SHT_INTEGR_TIM_1B[7:0];
	{TSB_8BIT, 0x1340, 0x09},    // FR_LENGTH_LINES_1B[15:8];
	{TSB_8BIT, 0x1341, 0xCE},    // FR_LENGTH_LINES_1B[7:0];
	{TSB_8BIT, 0x1342, 0x0E},    // LINE_LENGTH_PCK_1B[15:8];
	{TSB_8BIT, 0x1343, 0x74},    // LINE_LENGTH_PCK_1B[7:0];
	{TSB_8BIT, 0x1344, 0x00},    // -/-/-/-/H_CROP_1B[3:0];
	{TSB_8BIT, 0x1346, 0x00},    // Y_ADDR_START_1B[15:8];
	{TSB_8BIT, 0x1347, 0x00},    // Y_ADDR_START_1B[7:0];
	{TSB_8BIT, 0x134A, 0x09},    // Y_ADDR_END_1B[15:8];
	{TSB_8BIT, 0x134B, 0x9F},    // Y_ADDR_END_1B[7:0];
	{TSB_8BIT, 0x134C, 0x0C},    // X_OUTPUT_SIZE_1B[15:8];
	{TSB_8BIT, 0x134D, 0xD0},    // X_OUTPUT_SIZE_1B[7:0];
	{TSB_8BIT, 0x134E, 0x09},    // Y_OUTPUT_SIZE_1B[15:8];
	{TSB_8BIT, 0x134F, 0xA0},    // Y_OUTPUT_SIZE_1B[7:0];
	{TSB_8BIT, 0x1401, 0x00},    // -/-/-/-/-/-/SCALING_MODE_1B[1:0];
	{TSB_8BIT, 0x1403, 0x00},    // -/-/-/-/-/-/SPATIAL_SAMPLING_1B[1:0];
	{TSB_8BIT, 0x1404, 0x10},    // SCALE_M_1B[7:0];
	{TSB_8BIT, 0x1408, 0x00},    // DCROP_XOFS_1B[15:8];
	{TSB_8BIT, 0x1409, 0x00},    // DCROP_XOFS_1B[7:0];
	{TSB_8BIT, 0x140A, 0x00},    // DCROP_YOFS_1B[15:8];
	{TSB_8BIT, 0x140B, 0x00},    // DCROP_YOFS_1B[7:0];
	{TSB_8BIT, 0x140C, 0x0C},    // DCROP_WIDTH_1B[15:8];
	{TSB_8BIT, 0x140D, 0xD0},    // DCROP_WIDTH_1B[7:0];
	{TSB_8BIT, 0x140E, 0x09},    // DCROP_HIGT_1B[15:8];
	{TSB_8BIT, 0x140F, 0xA0},    // DCROP_HIGT_1B[7:0];
	{TSB_8BIT, 0x1601, 0x00},    // TEST_PATT_MODE_1B[7:0];
	{TSB_8BIT, 0x1602, 0x02},    // -/-/-/-/-/-/TEST_DATA_RED_1B[9:8];
	{TSB_8BIT, 0x1603, 0xC0},    // TEST_DATA_RED_1B[7:0];
	{TSB_8BIT, 0x1604, 0x02},    // -/-/-/-/-/-/TEST_DATA_GREENR_1B[9:8];
	{TSB_8BIT, 0x1605, 0xC0},    // TEST_DATA_GREENR_1B[7:0];
	{TSB_8BIT, 0x1606, 0x02},    // -/-/-/-/-/-/TEST_DATA_BLUE_1B[9:8];
	{TSB_8BIT, 0x1607, 0xC0},    // TEST_DATA_BLUE_1B[7:0];
	{TSB_8BIT, 0x1608, 0x02},    // -/-/-/-/-/-/TEST_DATA_GREENB_1B[9:8];
	{TSB_8BIT, 0x1609, 0xC0},    // TEST_DATA_GREENB_1B[7:0];
	{TSB_8BIT, 0x160A, 0x00},    // HO_CURS_WIDTH_1B[15:8];
	{TSB_8BIT, 0x160B, 0x00},    // HO_CURS_WIDTH_1B[7:0];
	{TSB_8BIT, 0x160C, 0x00},    // HO_CURS_POSITION_1B[15:8];
	{TSB_8BIT, 0x160D, 0x00},    // HO_CURS_POSITION_1B[7:0];
	{TSB_8BIT, 0x160E, 0x00},    // VE_CURS_WIDTH_1B[15:8];
	{TSB_8BIT, 0x160F, 0x00},    // VE_CURS_WIDTH_1B[7:0];
	{TSB_8BIT, 0x1610, 0x00},    // VE_CURS_POSITION_1B[15:8];
	{TSB_8BIT, 0x1611, 0x00},    // VE_CURS_POSITION_1B[7:0];
	{TSB_8BIT, 0x1900, 0x00},    // -/-/-/-/-/-/H_BIN_1B[1:0];
	{TSB_8BIT, 0x1901, 0x00},    // -/-/-/-/-/-/V_BIN_MODE_1B[1:0];
	{TSB_8BIT, 0x1902, 0x00},    // -/-/-/-/-/-/BINNING_WEIGHTING_1B[1:0];
	{TSB_8BIT, 0x300A, 0x00},    // Reserved ;
	{TSB_8BIT, 0x301A, 0x44},    // Reserved ;
	{TSB_8BIT, 0x301B, 0x44},    // Reserved ;
	{TSB_8BIT, 0x3024, 0x00},    // Reserved ;
	{TSB_8BIT, 0x3025, 0x65},    // Reserved ;
	{TSB_8BIT, 0x3037, 0x06},    // Reserved ;
	{TSB_8BIT, 0x3038, 0x06},    // Reserved ;
	{TSB_8BIT, 0x3039, 0x06},    // Reserved ;
	{TSB_8BIT, 0x303A, 0x06},    // Reserved ;
	{TSB_8BIT, 0x303B, 0x0B},    // Reserved ;
	{TSB_8BIT, 0x303C, 0x03},    // Reserved ;
	{TSB_8BIT, 0x303D, 0x03},    // Reserved ;
	{TSB_8BIT, 0x3053, 0xC0},    // Reserved ;
	{TSB_8BIT, 0x305D, 0x10},    // Reserved ;
	{TSB_8BIT, 0x305E, 0x06},    // Reserved ;
	{TSB_8BIT, 0x306B, 0x08},    // Reserved ;
	{TSB_8BIT, 0x3073, 0x1C},    // Reserved ;
	{TSB_8BIT, 0x3074, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x3075, 0x03},    // Reserved ;
	{TSB_8BIT, 0x3076, 0x7F},    // Reserved ;
	{TSB_8BIT, 0x307E, 0x02},    // Reserved ;
	{TSB_8BIT, 0x308D, 0x03},    // Reserved ;
	{TSB_8BIT, 0x308E, 0x20},    // Reserved ;
	{TSB_8BIT, 0x3091, 0x04},    // Reserved ;
	{TSB_8BIT, 0x3096, 0x75},    // Reserved ;
	{TSB_8BIT, 0x3097, 0x7E},    // Reserved ;
	{TSB_8BIT, 0x3098, 0x20},    // Reserved ;
	{TSB_8BIT, 0x30A0, 0x82},    // Reserved ;
	{TSB_8BIT, 0x30AB, 0x30},    // Reserved ;
	{TSB_8BIT, 0x30CE, 0x60},    // Reserved ;
	{TSB_8BIT, 0x30CF, 0x75},    // Reserved ;
	{TSB_8BIT, 0x30D2, 0xB3},    // Reserved ;
	{TSB_8BIT, 0x30D5, 0x09},    // Reserved ;
	{TSB_8BIT, 0x3134, 0x01},    // Reserved ;
	{TSB_8BIT, 0x314D, 0x80},    // Reserved ;
	{TSB_8BIT, 0x315B, 0x22},    // Reserved ;
	{TSB_8BIT, 0x315C, 0x22},    // Reserved ;
	{TSB_8BIT, 0x315D, 0x02},    // Reserved ;
	{TSB_8BIT, 0x3165, 0x67},    // Reserved ;
	{TSB_8BIT, 0x3168, 0xF1},    // Reserved ;
	{TSB_8BIT, 0x3169, 0x77},    // Reserved ;
	{TSB_8BIT, 0x316A, 0x77},    // Reserved ;
	{TSB_8BIT, 0x3173, 0x30},    // Reserved ;
	{TSB_8BIT, 0x31B1, 0x40},    // Reserved ;
	{TSB_8BIT, 0x31C1, 0x27},    // Reserved ;
	{TSB_8BIT, 0x31DB, 0x15},    // Reserved ;
	{TSB_8BIT, 0x31DC, 0xE0},    // Reserved ;
	{TSB_8BIT, 0x3204, 0x00},    // Reserved ;
	{TSB_8BIT, 0x3231, 0x00},    // PWB_RG[7:0];
	{TSB_8BIT, 0x3232, 0x00},    // PWB_GRG[7:0];
	{TSB_8BIT, 0x3233, 0x00},    // PWB_GBG[7:0];
	{TSB_8BIT, 0x3234, 0x00},    // PWB_BG[7:0];
	{TSB_8BIT, 0x3282, 0xC0},    // ABPC_EN/ABPC_CK_EN/-/-/-/-/-/-/;
	{TSB_8BIT, 0x3284, 0x06},    // Reserved ;
	{TSB_8BIT, 0x3285, 0x03},    // Reserved ;
	{TSB_8BIT, 0x3286, 0x02},    // Reserved ;
	{TSB_8BIT, 0x328A, 0x03},    // Reserved ;
	{TSB_8BIT, 0x328B, 0x02},    // Reserved ;
	{TSB_8BIT, 0x3290, 0x20},    // Reserved ;
	{TSB_8BIT, 0x3294, 0x10},    // Reserved ;
	{TSB_8BIT, 0x32A8, 0x84},    // Reserved ;
	{TSB_8BIT, 0x32B3, 0x10},    // Reserved ;
	{TSB_8BIT, 0x32B4, 0x1F},    // Reserved ;
	{TSB_8BIT, 0x32B7, 0x3B},    // Reserved ;
	{TSB_8BIT, 0x32BB, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x32BC, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x32BE, 0x04},    // Reserved ;
	{TSB_8BIT, 0x32BF, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x32C0, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x32C6, 0x50},    // Reserved ;
	{TSB_8BIT, 0x32C8, 0x0E},    // Reserved ;
	{TSB_8BIT, 0x32C9, 0x0E},    // Reserved ;
	{TSB_8BIT, 0x32CA, 0x0E},    // Reserved ;
	{TSB_8BIT, 0x32CB, 0x0E},    // Reserved ;
	{TSB_8BIT, 0x32CC, 0x0E},    // Reserved ;
	{TSB_8BIT, 0x32CD, 0x0E},    // Reserved ;
	{TSB_8BIT, 0x32CE, 0x08},    // Reserved ;
	{TSB_8BIT, 0x32CF, 0x08},    // Reserved ;
	{TSB_8BIT, 0x32D0, 0x08},    // Reserved ;
	{TSB_8BIT, 0x32D1, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x32D2, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x32D3, 0x0F},    // Reserved ;
	{TSB_8BIT, 0x32D4, 0x08},    // Reserved ;
	{TSB_8BIT, 0x32D5, 0x08},    // Reserved ;
	{TSB_8BIT, 0x32D6, 0x08},    // Reserved ;
	{TSB_8BIT, 0x32DD, 0x02},    // Reserved ;
	{TSB_8BIT, 0x32E0, 0x20},    // Reserved ;
	{TSB_8BIT, 0x32E1, 0x20},    // Reserved ;
	{TSB_8BIT, 0x32E2, 0x20},    // Reserved ;
	{TSB_8BIT, 0x32F7, 0x00},    // -/-/-/-/-/-/-/PP_DCROP_SW;
	{TSB_8BIT, 0x3307, 0x28},    // Reserved ;
	{TSB_8BIT, 0x3308, 0x27},    // Reserved ;
	{TSB_8BIT, 0x3309, 0x0D},    // Reserved ;
	{TSB_8BIT, 0x3384, 0x10},    // Reserved ;
	{TSB_8BIT, 0x3424, 0x00},    // -/-/-/-/TRIG_Z5_X/TX_TRIGOPT/CLKULPS/ESCREQ;
	{TSB_8BIT, 0x3425, 0x78},    // ESCDATA[7:0];
	{TSB_8BIT, 0x3427, 0xC0},    // MIPI_CLKVBLK/MIPI_CLK_MODE/-/-/HS_SR_CNT[1:0]/LP_SR_CNT[1:0];
	{TSB_8BIT, 0x3430, 0xA7},    // NUMWAKE[7:0];
	{TSB_8BIT, 0x3431, 0x60},    // NUMINIT[7:0];
	{TSB_8BIT, 0x3432, 0x11},    // -/-/-/CLK0_M/-/-/-/LNKBTWK_ON;
	GROUPED_PARAMETER_HOLD_DISABLE,
	{TSB_TOK_TERM, 0, 0}
};
/* TODO settings of preview/still/video will be updated with new use case */
struct tsb_resolution t4k35_res_preview[] = {
	{
		.desc = "PREVIEW_Bin_1616x916_30fps_2lane",
		.regs = t4k35_Bin_1616x916_30fps_2lane,
		.width = 1616,
		.height = 916,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 30,
				 .pixels_per_line = 0x0DE0,
				 .lines_per_frame = 0x070A,
			},
			{
			}
		},
	},
	{
		.desc = "PREVIEW_Bin_1632x1228_30fps_2lane",
		.regs = t4k35_Bin_1632x1228_30fps_2lane,
		.width = 1632,
		.height = 1228,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 30,
				 .pixels_per_line = 0x0DE0,
				 .lines_per_frame = 0x070A,
			},
			{
			}
		},
	},
	{
		.desc = "PREVIEW_Full_3280x1852_26fps_2lane",
		.regs = t4k35_Full_3280x1852_26fps_2lane,
		.width = 3280,
		.height = 1852,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 26,
				 .pixels_per_line = 0x0E50,
				 .lines_per_frame = 0x07E0,
			},
			{
			}
		},
	},
	{
		.desc = "PREVIEW_Full_3280x2464_20fps_2lane",
		.regs = t4k35_Full_3280x2464_20fps_2lane,
		.width = 3280,
		.height = 2464,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 20,
				 .pixels_per_line = 0x0EBE,
				 .lines_per_frame = 0x09F0,
			},
			{
			}
		},
	},
};

struct tsb_resolution t4k35_res_still[] = {
	{
		.desc = "STILL_Bin_1616x916_30fps_2lane",
		.regs = t4k35_Bin_1616x916_30fps_2lane,
		.width = 1616,
		.height = 916,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 30,
				 .pixels_per_line = 0x0DE0,
				 .lines_per_frame = 0x070A,
			},
			{
			}
		},
	},
	{
		.desc = "STILL_Bin_1632x1228_30fps_2lane",
		.regs = t4k35_Bin_1632x1228_30fps_2lane,
		.width = 1632,
		.height = 1228,
		.bin_factor_x = 2,
		.bin_factor_y = 2,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 30,
				 .pixels_per_line = 0x0DE0,
				 .lines_per_frame = 0x070A,
			},
			{
			}
		},
	},
	{
		.desc = "STILL_Full_3280x1852_8fps_2lane",
		.regs = t4k35_Full_3280x1852_8fps_2lane,
		.width = 3280,
		.height = 1852,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 8,
				 .pixels_per_line = 0x0F7E,
				 .lines_per_frame = 0x17A4,
			},
			{
			}
		},
	},
	{
		.desc = "STILL_Full_3280x2464_6fps_2lane",
		.regs = t4k35_Full_3280x2464_6fps_2lane,
		.width = 3280,
		.height = 2464,
		.bin_factor_x = 1,
		.bin_factor_y = 1,
		.used = 0,
		.skip_frames = 3,
		.fps_options = {
			{
				 .fps = 6,
				 .pixels_per_line = 0x10C0,
				 .lines_per_frame = 0x1D28,
			},
			{
			}
		},
	},
};

struct tsb_resolution t4k35_res_video[] = {
        {
                .desc = "VIDEO_QCIF_192x160_30fps_2lane",
                .regs = t4k35_Bin_192x160_30fps_2lane,
                .width = 192,
                .height = 160,
                .bin_factor_x = 2,
                .bin_factor_y = 2,
                .used = 0,
                .skip_frames = 3,
                .fps_options = {
                        {
                                 .fps = 30,
                                 .pixels_per_line = 0x1176,
                                 .lines_per_frame = 0x0598,
                        },
                        {
                        }
                },
        },
        {
                .desc = "VIDEO_VGA_656x496_30fps_2lane",
                .regs = t4k35_Bin_656x496_30fps_2lane,
                .width = 656,
                .height = 496,
                .bin_factor_x = 2,
                .bin_factor_y = 2,
                .used = 0,
                .skip_frames = 3,
                .fps_options = {
                        {
                                 .fps = 30,
                                 .pixels_per_line = 0x1176,
                                 .lines_per_frame = 0x0598,
                        },
                        {
                        }
                },
        },
        {
                .desc = "VIDEO_HD_1296x736_30fps_2lane",
                .regs = t4k35_HD_1296x736_30fps_2lane,
                .width = 1296,
                .height = 736,
                .bin_factor_x = 2,
                .bin_factor_y = 2,
                .used = 0,
                .skip_frames = 3,
                .fps_options = {
                        {
                                 .fps = 30,
                                 .pixels_per_line = 0x1176,
                                 .lines_per_frame = 0x0598,
                        },
                        {
                        }
                },
        },
        {
                .desc = "VIDEO_FHD_1936x1096_30fps_2lane",
                .regs = t4k35_FHD_1936x1096_30fps_2lane,
                .width = 1936,
                .height = 1096,
                .bin_factor_x = 1,
                .bin_factor_y = 1,
                .used = 0,
                .skip_frames = 3,
                .fps_options = {
                        {
                                 .fps = 30,
                                 .pixels_per_line = 0x0F5E,
                                 .lines_per_frame = 0x07A0,
                        },
                        {
                        }
                },
        },
};

#endif
