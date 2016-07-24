#ifndef _FF_RGBS_GRD_API_H
#define _FF_RGBS_GRD_API_H
//#include "ff_rgbs_grd_regs.h"
#include "ff_rgbs_grd_ccbcr.h"


#define FF_rgbs_grd_set_register(FF_RGBS_GRD_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_RGBS_GRD_id + addr, (val))

#define FF_rgbs_grd_get_register(FF_RGBS_GRD_id, addr) \
	vied_subsystem_load_32(psys0, FF_RGBS_GRD_id + addr)

#define FF_rgbs_grd_enable(FF_RGBS_GRD_id)								\
	do {											\
	int thrsh1_reg_value = FF_rgbs_grd_get_register(FF_RGBS_GRD_id, FF_AWB_RGBS_THR_1_ADDR);\
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, FF_AWB_RGBS_THR_1_ADDR, (0x01 <<30) | (thrsh1_reg_value & 0xBFFFFFFF));\
	}while(0)

#define FF_rgbs_grd_disable(FF_RGBS_GRD_id)								\
	do {											\
	int thrsh1_reg_value = FF_rgbs_grd_get_register(FF_RGBS_GRD_id, FF_AWB_RGBS_THR_1_ADDR);\
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, FF_AWB_RGBS_THR_1_ADDR, thrsh1_reg_value & 0xBFFFFFFF);\
	}while(0)

#define FF_rgbs_grd_set_Threshold(FF_RGBS_GRD_id, Rgbs_thr_gr, Rgbs_thr_r, Rgbs_thr_b, Rgbs_thr_gb, rgbs_incl_sat) \
	do {												\
	int thrsh1_reg_value = 0;									\
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, FF_AWB_RGBS_THR_0_ADDR, ((Rgbs_thr_r & 0x1FFF) << 16) | (Rgbs_thr_gr & 0x1FFF));	\
	thrsh1_reg_value = FF_rgbs_grd_get_register(FF_RGBS_GRD_id, FF_AWB_RGBS_THR_1_ADDR);		\
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, FF_AWB_RGBS_THR_1_ADDR, (thrsh1_reg_value & 0xC0000000) | ((Rgbs_thr_gb & 0x1FFF) << 16) | (Rgbs_thr_b & 0x1FFF) | (rgbs_incl_sat << 31));	\
	} while(0);

#define FF_rgbs_grd_set_Cfg(FF_RGBS_GRD_id, rgbs_grid_width, rgbs_grid_height, rgbs_block_width, rgbs_block_height,grid_height_per_slice) \
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, FF_AWB_RGBS_GRD_CFG_ADDR,  ((grid_height_per_slice & 0xFF) << 22)|((rgbs_block_height & 0x7) << 19)| \
	((rgbs_block_width & 0x7) << 16)|((rgbs_grid_height & 0x7F) <<8) | (rgbs_grid_width & 0x7F));	\

#define FF_rgbs_grd_set_Offset_start(FF_RGBS_GRD_id, rgbs_x_start, rgbs_y_start) \
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, FF_AWB_RGBS_GRD_START_ADDR, ((rgbs_y_start & 0xFFF) << 16) | (rgbs_x_start & 0xFFF));

#define FF_rgbs_grd_set_Offset_end(FF_RGBS_GRD_id, rgbs_x_end, rgbs_y_end) \
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, FF_AWB_RGBS_GRD_END_ADDR, ((rgbs_y_end & 0xFFF) << 16) | (rgbs_x_end & 0xFFF));	\

#define FF_rgbs_grd_set_Col_Set0(FF_RGBS_GRD_id, index, greenr_avg, red_avg, blue_avg, greenb_avg) \
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id, MEM_RGBS_SET0_BASE_ADDR+4*index*2,((greenb_avg & 0xFF) << 24) |((blue_avg & 0xFF) << 16)| ((red_avg & 0xFF) << 8) | (greenr_avg & 0xFF));

#define FF_rgbs_grd_set_Sat_Set0(FF_RGBS_GRD_id, index, sat_ratio) \
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id,MEM_RGBS_SET0_BASE_ADDR+4*(2*index+1), sat_ratio & 0xFF);

#define FF_rgbs_grd_set_Col_Set1(FF_RGBS_GRD_id, index, greenr_avg, red_avg, blue_avg, greenb_avg) \
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id,MEM_RGBS_SET1_BASE_ADDR+4*2*index,((greenb_avg & 0xFF) << 24) |((blue_avg & 0xFF) << 16)| ((red_avg & 0xFF) << 8) | (greenr_avg & 0xFF));

#define FF_rgbs_grd_set_Sat_Set1(FF_RGBS_GRD_id, index, sat_ratio) \
	FF_rgbs_grd_set_register(FF_RGBS_GRD_id,MEM_RGBS_SET1_BASE_ADDR+4*(2*index+1), sat_ratio & 0xFF);

#define FF_rgbs_grd_get_Sat_Set0(FF_RGBS_GRD_id, index, sat_ratio) \
	do {												\
	unsigned int reg = FF_rgbs_grd_get_register(FF_RGBS_GRD_id, MEM_RGBS_SET0_BASE_ADDR+4*(2*index+1));			\
	sat_ratio = reg & 0xFF;									\
	} while(0);

#define FF_rgbs_grd_get_Sat_Set1(FF_RGBS_GRD_id, index, sat_ratio) \
	do {												\
	unsigned int reg = FF_rgbs_grd_get_register(FF_RGBS_GRD_id, MEM_RGBS_SET1_BASE_ADDR+4*(2*index+1));			\
	sat_ratio = reg & 0xFF;									\
	} while(0);

#define FF_rgbs_grd_get_Col_Set0(FF_RGBS_GRD_id, index, greenr_avg, red_avg, blue_avg, greenb_avg) \
	do {												\
	unsigned int reg = FF_rgbs_grd_get_register(FF_RGBS_GRD_id, MEM_RGBS_SET0_BASE_ADDR+4*index*2);			\
	greenr_avg = reg & 0xFF;									\
	red_avg = (reg >>8) & 0xFF;     \
	blue_avg = (reg >>16) & 0xFF;     \
	greenb_avg = (reg >>24) & 0xFF;     \
	} while(0);

#define FF_rgbs_grd_get_Col_Set1(FF_RGBS_GRD_id, index, greenr_avg, red_avg, blue_avg, greenb_avg) \
	do {												\
	unsigned int reg = FF_rgbs_grd_get_register(FF_RGBS_GRD_id, MEM_RGBS_SET1_BASE_ADDR+4*2*index);			\
	greenr_avg = reg & 0xFF;									\
	red_avg = (reg >>8) & 0xFF;     \
	blue_avg = (reg >>16) & 0xFF;     \
	greenb_avg = (reg >>24) & 0xFF;     \
	} while(0);

#endif // _FF_RGBS_GRD_API_H
