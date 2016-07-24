#ifndef _FF_AF_AWB_FR_GRD_API_H
#define _FF_AF_AWB_FR_GRD_API_H
#include "ff_af_awb_fr_grd_ccbcr.h"


#define FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_AF_AWB_FR_GRD_id + addr, (val))

#define FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, addr) \
	vied_subsystem_load_32(psys0, FF_AF_AWB_FR_GRD_id + addr)

#define FF_af_awb_fr_grd_set_Enable(FF_AF_AWB_FR_GRD_id, Y_FR_En, AWB_FR_En, FF_En) \
	do {											\
	int reg_value = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_GRD_CFG_ADDR);\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_GRD_CFG_ADDR,  ((FF_En & 0x1) << 30)|((AWB_FR_En & 0x1) <<29) | ((Y_FR_En & 0x1)<<28) | (reg_value & 0x8FFFFFFF)); \
	reg_value = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_GRD_CFG_ADDR);\
	}while(0);

#define FF_af_awb_fr_grd_set_Cfg(FF_AF_AWB_FR_GRD_id, grid_width, grid_height, block_width, block_height,grid_height_per_slice) \
do {											\
	int reg_value = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_GRD_CFG_ADDR);\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_GRD_CFG_ADDR,  ((grid_height_per_slice & 0xF) << 24)|((block_height & 0xF) << 20)| \
	((block_width & 0xF) << 16)|((grid_height & 0x3F) <<8) | (grid_width & 0x3F) | (reg_value & 0xF0000000));\
}while(0);

#define FF_af_awb_fr_grd_set_Offset_start(FF_AF_AWB_FR_GRD_id, x_start, y_start) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_GRD_START_ADDR, ((y_start & 0x3FFF) << 16) | (x_start & 0x3FFF));

#define FF_af_awb_fr_grd_set_Offset_end(FF_AF_AWB_FR_GRD_id, x_end, y_end) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_GRD_END_ADDR, ((y_end & 0x3FFF) << 16) | (x_end & 0x3FFF));

#define FF_af_awb_fr_grd_set_Y_ShftR_vals(FF_AF_AWB_FR_GRD_id, ShftR_val_Y00, ShftR_val_Y01, ShftR_val_Y10, ShftR_val_Y11) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_AVG_SHFT_CTRL_0_ADDR, ((ShftR_val_Y11 & 0xF) << 24) |((ShftR_val_Y10 & 0xF) << 16) | ((ShftR_val_Y01 & 0xF) << 8) | (ShftR_val_Y00 & 0xF));

#define FF_af_awb_fr_grd_set_RGB_ShftR_vals(FF_AF_AWB_FR_GRD_id, ShftR_val_R, ShftR_val_G, ShftR_val_B) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_AVG_SHFT_CTRL_1_ADDR, ((ShftR_val_B & 0x1F) << 16) | ((ShftR_val_G & 0x1F) << 8) | (ShftR_val_R & 0x1F));

#define FF_af_awb_fr_grd_set_Y_est_gains(FF_AF_AWB_FR_GRD_id, g00, g01, g02, g03, g10, g11, g12, g13, g20, g21, g22, g23, g30, g31, g32, g33) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y_EST_GAIN_0_ADDR,  ((g03 & 0x3F) << 24)|((g02 & 0x3F) << 16)|((g01 & 0x3F) <<8) | (g00 & 0x3F));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y_EST_GAIN_1_ADDR,  ((g13 & 0x3F) << 24)|((g12 & 0x3F) << 16)|((g11 & 0x3F) <<8) | (g10 & 0x3F));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y_EST_GAIN_2_ADDR,  ((g23 & 0x3F) << 24)|((g22 & 0x3F) << 16)|((g21 & 0x3F) <<8) | (g20 & 0x3F));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y_EST_GAIN_3_ADDR,  ((g33 & 0x3F) << 24)|((g32 & 0x3F) << 16)|((g31 & 0x3F) <<8) | (g30 & 0x3F));\
	}while(0);

#define FF_af_awb_fr_grd_set_Y_est_mask(FF_AF_AWB_FR_GRD_id, mask_y0, mask_y1) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y_EST_MASK_ADDR, ((mask_y1 & 0xFFFF) << 16) | (mask_y0 & 0xFFFF));

#define FF_af_awb_fr_grd_set_Y_est_OutputEnable(FF_AF_AWB_FR_GRD_id, oe_y0, oe_y1) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y_EST_OUTPUT_ENABLE_ADDR, ((oe_y1 & 0xF) << 4) | (oe_y0 & 0xF));

#define FF_af_awb_fr_grd_set_Y_est_normalization(FF_AF_AWB_FR_GRD_id, on00_y0, on01_y0, on10_y0, on11_y0, on00_y1, on01_y1, on10_y1, on11_y1) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y_EST_NORMALIZATION_ADDR, ((on11_y1 & 0xF) << 28) |((on10_y1 & 0xF) << 24) | ((on01_y1 & 0xF) << 20) | ((on00_y1 & 0xF) << 16) | \
	((on11_y0 & 0xF) << 12) |((on10_y0 & 0xF) << 8) | ((on01_y0 & 0xF) << 4) | (on00_y0 & 0xF));

#define FF_af_awb_fr_grd_set_RGB_mask(FF_AF_AWB_FR_GRD_id, r_select, g_select, b_select) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_RG_SELECT_MASK_ADDR,  ((g_select & 0xFFFF) <<16) | (r_select & 0xFFFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_B_SELECT_MASK_ADDR,  b_select & 0xFFFF);\
	}while(0);

#define FF_af_awb_fr_grd_set_Y00_coeffs_sign(FF_AF_AWB_FR_GRD_id, a1, a2, a3, a4, a5, a6, signVec) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y00_COEFF_0_ADDR,  ((a4 & 0xFF) << 24)|((a3 & 0xFF) << 16)|((a2 & 0xFF) <<8) | (a1 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y00_COEFF_1_ADDR,  ((a6 & 0xFF) <<8) | (a5 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y00_SIGN_ADDR,  (signVec & 0x1FF));\
	}while(0);

#define FF_af_awb_fr_grd_set_Y01_coeffs_sign(FF_AF_AWB_FR_GRD_id, a1, a2, a3, a4, a5, a6, signVec) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y01_COEFF_0_ADDR,  ((a4 & 0xFF) << 24)|((a3 & 0xFF) << 16)|((a2 & 0xFF) <<8) | (a1 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y01_COEFF_1_ADDR,  ((a6 & 0xFF) <<8) | (a5 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y01_SIGN_ADDR,  (signVec & 0x1FF));\
	}while(0);

#define FF_af_awb_fr_grd_set_Y10_coeffs_sign(FF_AF_AWB_FR_GRD_id, a1, a2, a3, a4, a5, a6, signVec) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y10_COEFF_0_ADDR,  ((a4 & 0xFF) << 24)|((a3 & 0xFF) << 16)|((a2 & 0xFF) <<8) | (a1 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y10_COEFF_1_ADDR,  ((a6 & 0xFF) <<8) | (a5 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y10_SIGN_ADDR,  (signVec & 0x1FF));\
	}while(0);

#define FF_af_awb_fr_grd_set_Y11_coeffs_sign(FF_AF_AWB_FR_GRD_id, a1, a2, a3, a4, a5, a6, signVec) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y11_COEFF_0_ADDR,  ((a4 & 0xFF) << 24)|((a3 & 0xFF) << 16)|((a2 & 0xFF) <<8) | (a1 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y11_COEFF_1_ADDR,  ((a6 & 0xFF) <<8) | (a5 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_Y11_SIGN_ADDR,  (signVec & 0x1FF));\
	}while(0);

#define FF_af_awb_fr_grd_set_R_coeffs_sign(FF_AF_AWB_FR_GRD_id, a1, a2, a3, a4, a5, a6, signVec) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_R_COEFF_0_ADDR,  ((a4 & 0xFF) << 24)|((a3 & 0xFF) << 16)|((a2 & 0xFF) <<8) | (a1 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_R_COEFF_1_ADDR,  ((a6 & 0xFF) <<8) | (a5 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_R_SIGN_ADDR,  (signVec & 0x1FF));\
	}while(0);

#define FF_af_awb_fr_grd_set_G_coeffs_sign(FF_AF_AWB_FR_GRD_id, a1, a2, a3, a4, a5, a6, signVec) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_G_COEFF_0_ADDR,  ((a4 & 0xFF) << 24)|((a3 & 0xFF) << 16)|((a2 & 0xFF) <<8) | (a1 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_G_COEFF_1_ADDR,  ((a6 & 0xFF) <<8) | (a5 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_G_SIGN_ADDR,  (signVec & 0x1FF));\
	}while(0);

#define FF_af_awb_fr_grd_set_B_coeffs_sign(FF_AF_AWB_FR_GRD_id, a1, a2, a3, a4, a5, a6, signVec) \
	do {											\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_B_COEFF_0_ADDR,  ((a4 & 0xFF) << 24)|((a3 & 0xFF) << 16)|((a2 & 0xFF) <<8) | (a1 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_B_COEFF_1_ADDR,  ((a6 & 0xFF) <<8) | (a5 & 0xFF));\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_B_SIGN_ADDR,  (signVec & 0x1FF));\
	}while(0);

#define FF_af_awb_fr_grd_set_normalization_factor(FF_AF_AWB_FR_GRD_id, nf_y00, nf_y01, nf_y10, nf_y11, nf_r, nf_g, nf_b) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_NF_ADDR, ((nf_b & 0xF) << 24) | ((nf_g & 0xF) << 20) | ((nf_b & 0xF) << 16) | \
	((nf_y11 & 0xF) << 12) |((nf_y10 & 0xF) << 8) | ((nf_y01 & 0xF) << 4) | (nf_y00 & 0xF));

#define FF_af_awb_fr_grd_set_RGB_Set0(FF_AF_AWB_FR_GRD_id, index, red_avg, green_avg, blue_avg) \
	do {\
	unsigned int B_avg_lsb = (blue_avg & 0xFF);\
	unsigned int B_avg_msb = (blue_avg >> 8) & 0xF;\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET0_BASE_ADDR+8*index, ((B_avg_lsb & 0xFF) << 24) | ((green_avg & 0xFFF) << 12)| (red_avg & 0xFFF)); \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET0_BASE_ADDR+8*index+4, B_avg_msb); \
	} while(0);

#define FF_af_awb_fr_grd_set_Y_Set0(FF_AF_AWB_FR_GRD_id, index, y00_avg, y01_avg, y10_avg, y11_avg) \
	do {											\
	unsigned int Y10_avg_lsb = y10_avg & 0xFF;		\
	unsigned int Y10_avg_msb = (y10_avg >> 8)&0xF;		\
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET0_BASE_ADDR+8*index, ((Y10_avg_lsb & 0xFF) << 24) | ((y01 & 0xFFF) << 12)| (y00 & 0xFFF)); \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET0_BASE_ADDR+8*index+4, ((y11 & 0xFFF) << 4) | (Y10_avg_msb)); \
	} while(0);

#define FF_af_awb_fr_grd_set_RGB_Set1(FF_AF_AWB_FR_GRD_id, index, red_avg, green_avg, blue_avg) \
	do {											\
		unsigned int B_avg_lsb = blue_avg & 0xFF;		\
		unsigned int B_avg_msb = (blue_avg >> 8)&0xF;		\
		FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET1_BASE_ADDR+8*index, (B_avg_lsb & 0xFF) << 24) | ((green_avg & 0xFFF) << 12)| (red_avg & 0xFFF)); \
		FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET1_BASE_ADDR+8*index+4, B_avg_msb); \
	} while(0);

#define FF_af_awb_fr_grd_set_Y_Set1(FF_AF_AWB_FR_GRD_id, index, y00_avg, y01_avg, y10_avg, y11_avg) \
	do {											\
		unsigned int Y10_avg_lsb = y10_avg & 0xFF;		\
		unsigned int Y10_avg_msb = (y10_avg >> 8)&0xF;		\
		FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET1_BASE_ADDR+8*index, ((Y10_avg_lsb & 0xFF) << 24) | ((y01 & 0xFFF) << 12)| (y00 & 0xFFF)); \
		FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET1_BASE_ADDR+8*index+4, ((y11 & 0xFFF) << 4) | (Y10_avg_msb)); \
	} while(0);

#define FF_af_awb_fr_grd_set_sensor_mode(FF_AF_AWB_FR_GRD_id, sensor_mode) \
	FF_af_awb_fr_grd_set_register(FF_AF_AWB_FR_GRD_id, FF_AF_AWB_FR_SENSOR_CFG_0_ADDR, (sensor_mode));


#define FF_af_awb_fr_grd_get_Y_Set0(FF_AF_AWB_FR_GRD_id, index, y00_avg, y01_avg, y10_avg, y11_avg) \
	do {												\
		unsigned int reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET0_BASE_ADDR+8*index);			\
		y00_avg = reg & 0xFFF;			 \
		y01_avg = (reg >>12) & 0xFFF;     \
		unsigned int y10_avg_lsb = (reg >> 24) & 0xFF;	  \
		reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET0_BASE_ADDR+8*index+4);			\
		unsigned int y10_avg_msb = reg & 0xF;	  \
		y11_avg = (reg >>4) & 0xFFF;     \
		y10_avg = y10_avg_lsb | (y10_avg_msb << 8); \
	} while(0);

#define FF_af_awb_fr_grd_get_RGB_Set0(FF_AF_AWB_FR_GRD_id, index,red_avg, blue_avg, green_avg) \
	do {												\
		unsigned int reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET0_BASE_ADDR + 8*index);			\
		red_avg = reg  & 0xFFF;     \
		green_avg = (reg >>12) & 0xFFF;     \
		unsigned int blue_avg_lsb = (reg >>24) & 0xFF;     \
		reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET0_BASE_ADDR + 8*index+4); \
		unsigned int blue_avg_msb = reg & 0xF;     \
		blue_avg = blue_avg_lsb | (blue_avg_msb << 8); \
	} while(0);

#define FF_af_awb_fr_grd_get_Y_Set1(FF_AF_AWB_FR_GRD_id, index,y00_avg, y01_avg, y10_avg, y11_avg) \
	do {												\
		unsigned int reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET1_BASE_ADDR+8*index);			\
		y00_avg = reg & 0xFFF;			 \
		y01_avg = (reg >>12) & 0xFFF;     \
		unsigned int y10_avg_lsb = (reg>>24) & 0xFF;	  \
		reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AF_Y_FR_SET1_BASE_ADDR+8*index+4);			\
		unsigned int y10_avg_msb = reg & 0xF;	  \
		y11_avg = (reg >>4) & 0xFFF;     \
		y10_avg = y10_avg_lsb | (y10_avg_msb << 8); \
	} while(0);

#define FF_af_awb_fr_grd_get_RGB_Set1(FF_AF_AWB_FR_GRD_id, index,red_avg, blue_avg, green_avg) \
	do {												\
		unsigned int reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET1_BASE_ADDR + 8*index);			\
		red_avg = reg  & 0xFFF;     \
		green_avg = (reg >>12) & 0xFFF;     \
		unsigned int blue_avg_lsb = (reg >>24) & 0xFF;     \
		reg = FF_af_awb_fr_grd_get_register(FF_AF_AWB_FR_GRD_id, MEM_STAT_AWB_RGB_FR_SET1_BASE_ADDR + 8*index+4); \
		unsigned int blue_avg_msb = reg & 0xF;     \
		blue_avg = blue_avg_lsb | (blue_avg_msb << 8); \
	} while(0);




#endif // _FF_AF_AWB_FR_GRD_API_H
