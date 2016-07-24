#ifndef _FF_DVS_API_H
#define _FF_DVS_API_H
#include <hrt/api.h>
#include "ff_dvs_ccbcr.h"

#define FF_dvs_set_register(FF_dvs_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_dvs_id + addr, (val))

#define FF_dvs_get_register(FF_dvs_id, addr) \
  vied_subsystem_load_32(psys0, FF_dvs_id + addr)

 ////////// Setters /////////////////

 // Kappa, match_shift, binning_mode
#define FF_dvs_set_kappa_matchGradeShift_binMode(FF_dvs_id, kappa, match_grade_shift, binning_mode)  	\
	FF_dvs_set_register(FF_dvs_id, FF_DVS_GLOBAL_CFG_ADDR , ((kappa) & (0xFF)) | (((match_grade_shift) & (0xF)) << 8) | (((binning_mode) & (0x1)) << 12))

// Grid config
#define FF_dvs_set_grid_cfg(FF_dvs_id, level_num, grid_width, grid_height, block_width, block_height)  	    \
do {                                                                                                        \
    unsigned int baseAddress[3] = {FF_DVS_L0_GRD_CFG_ADDR, FF_DVS_L1_GRD_CFG_ADDR, FF_DVS_L2_GRD_CFG_ADDR}; \
    FF_dvs_set_register(FF_dvs_id, baseAddress[level_num]  , ((grid_width) & (0x1F)) | (((grid_height) & (0x1F)) << 8) | (((block_width) & (0xFF)) << 16) |  (((block_height) & (0xFF)) << 24)); \
} while (0)


// Grid start
#define FF_dvs_set_grid_start(FF_dvs_id, level_num, x_start, y_start, L0_En)  	                                    \
do {                                                                                                                \
    unsigned int baseAddress[3] = {FF_DVS_L0_GRD_START_ADDR, FF_DVS_L1_GRD_START_ADDR, FF_DVS_L2_GRD_START_ADDR};   \
    FF_dvs_set_register(FF_dvs_id, baseAddress[level_num]  , ((x_start) & (0xFFF)) | (((y_start) & (0xFFF)) << 16) | (((L0_En) & (0x1)) << 30)); \
} while (0)


// Grid End
#define FF_dvs_set_grid_end(FF_dvs_id, level_num, x_end, y_end)  	                                            \
do {                                                                                                            \
    unsigned int baseAddress[3] = {FF_DVS_L0_GRD_END_ADDR, FF_DVS_L1_GRD_END_ADDR, FF_DVS_L2_GRD_END_ADDR};     \
    FF_dvs_set_register(FF_dvs_id, baseAddress[level_num]  , ((x_end) & (0xFFF)) | (((y_end) & (0xFFF)) << 16));    \
} while (0)


// FE_ROI config
#define FF_dvs_set_fe_roi_cfg(FF_dvs_id, level_num, x_start, y_start, x_end, y_end)  	                                \
do {                                                                                                                    \
    unsigned int baseAddress[3] = {FF_DVS_L0_FE_ROI_CFG_ADDR, FF_DVS_L1_FE_ROI_CFG_ADDR, FF_DVS_L2_FE_ROI_CFG_ADDR};    \
    FF_dvs_set_register(FF_dvs_id, baseAddress[level_num]  , ((x_start) & (0xFF)) | (((y_start) & (0xFF)) << 8) | (((x_end) & (0xFF)) << 16) |  (((y_end) & (0xFF)) << 24)); \
} while (0)


#endif // _FF_DVS_API_H


