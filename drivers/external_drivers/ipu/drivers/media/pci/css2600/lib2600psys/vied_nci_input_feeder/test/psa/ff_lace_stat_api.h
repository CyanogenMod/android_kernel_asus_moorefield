#ifndef _FF_LACE_STAT_API_H
#define _FF_LACE_STAT_API_H
#include "ff_lace_stat_ccbcr.h"

// General

#define FF_lace_stat_set_register(FF_LACE_STAT_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_LACE_STAT_id + addr, (val))

#define FF_lace_stat_get_register(FF_LACE_STAT_id, addr) \
	vied_subsystem_load_32(psys0, FF_LACE_STAT_id + addr)



// Global and grid configuration

#define FF_lace_stat_set_glb_Cfg(FF_LACE_STAT_id, LH_mode, Y_ds_mode) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_GLB_CFG_ADDR,  ( ((Y_ds_mode & 0x3) <<6) | (LH_mode & 0x7) )   );

#define FF_lace_stat_set_y_grd_hor_Cfg(FF_LACE_STAT_id, grid_width, block_width) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_STAT_Y_GRD_HOR_CFG_ADDR,((block_width & 0xF) <<16) | (grid_width & 0x3F));

#define FF_lace_stat_set_y_grd_hor_Roi(FF_LACE_STAT_id, x_start, x_end) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_STAT_Y_GRD_HOR_ROI_ADDR,((x_end & 0xFFF) <<16) | (x_start & 0xFFF));

// no UV histogram in A0 : ww2013'31: enabled writing to those registers
#define FF_lace_stat_set_uv_grd_hor_Cfg(FF_LACE_STAT_id, grid_width, block_width) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_STAT_UV_GRD_HOR_CFG_ADDR,((block_width & 0xF) <<16) | (grid_width & 0x3F));

#define FF_lace_stat_set_uv_grd_hor_Roi(FF_LACE_STAT_id, x_start, x_end) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_STAT_UV_GRD_HOR_ROI_ADDR,((x_end & 0xFFF) <<16) | (x_start & 0xFFF));



#define FF_lace_stat_set_grd_vrt_Cfg(FF_LACE_STAT_id, grid_height, block_height, grid_height_per_slice) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_STAT_GRD_VRT_CFG_ADDR,( (grid_height_per_slice & 0xFF) <<24) | ((block_height & 0xF) <<20)| ( (grid_height & 0x3F) <<8));

#define FF_lace_stat_set_grd_vrt_Roi(FF_LACE_STAT_id, y_start, y_end) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_STAT_GRD_VRT_ROI_ADDR,((y_end & 0xFFF) <<16) | (y_start & 0xFFF));



// Histogram arrays registers

#define FF_lace_stat_set_loc_hist_regs_set0(FF_LACE_STAT_id, index, Bin0, Bin1, Bin2, Bin3) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, (MEM_H0_SET0_BASE_ADDR+4*index),((Bin3 & 0xFF) <<24) | ((Bin2 & 0xFF) <<16) | ((Bin1 & 0xFF) <<8) | (Bin0 & 0xFF) );

#define FF_lace_stat_set_loc_hist_regs_set1(FF_LACE_STAT_id, index, Bin0, Bin1, Bin2, Bin3) \
	FF_lace_stat_set_register(FF_LACE_STAT_id, (MEM_H0_SET1_BASE_ADDR+4*index),((Bin3 & 0xFF) <<24) | ((Bin2 & 0xFF) <<16) | ((Bin1 & 0xFF) <<8) | (Bin0 & 0xFF) );

#define FF_lace_stat_get_loc_hist_regs_set0(FF_LACE_STAT_id, index, Bin0, Bin1, Bin2, Bin3) \
	do {												\
	unsigned int reg = FF_lace_stat_get_register(FF_LACE_STAT_id, MEM_H0_SET0_BASE_ADDR+4*index);			\
	Bin0 = reg & 0xFF; \
	Bin1 = (reg >>8) & 0xFF; \
	Bin2 = (reg >>16) & 0xFF; \
	Bin3 = (reg >>24) & 0xFF; \
	} while(0);

#define FF_lace_stat_get_loc_hist_regs_set1(FF_LACE_STAT_id, index, Bin0, Bin1, Bin2, Bin3) \
	do {												\
	unsigned int reg = FF_lace_stat_get_register(FF_LACE_STAT_id, MEM_H0_SET1_BASE_ADDR+4*index);			\
	Bin0 = reg & 0xFF; \
	Bin1 = (reg >>8) & 0xFF; \
	Bin2 = (reg >>16) & 0xFF; \
	Bin3 = (reg >>24) & 0xFF; \
	} while(0);



// Hist array reseting

// (Write Only bit) this flag will trigger hist array resetting in the start of frame (the parameter allows writing a '0')
#define FF_lace_stat_set_Rst_loc_hist_arr(FF_LACE_STAT_id, Rst_loc_hist_array) \
	do {											\
	int reg_value = FF_lace_stat_get_register(FF_LACE_STAT_id, FF_LACE_GLB_CFG_ADDR); \
	FF_lace_stat_set_register(FF_LACE_STAT_id, FF_LACE_GLB_CFG_ADDR,( (Rst_loc_hist_array & 0x1) <<20) | (reg_value & 0xFFEFFFFF) ); \
	}while(0);

// (Read Only bit) this flag will indicate hist arrays reset completion
#define FF_lace_stat_get_Done_rst_loc_hist_array(FF_LACE_STAT_id, Done_rst_loc_hist_array) \
	do {	 \
	int reg_value = FF_lace_stat_get_register(FF_LACE_STAT_id, FF_LACE_GLB_CFG_ADDR); \
	Done_rst_loc_hist_array = ((reg_value & 0x00200000) >> 21); \
	}while(0);

#endif // _FF_LACE_STAT_API_H
