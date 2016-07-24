#ifndef _FF_AE_HIST_API_H
#define _FF_AE_HIST_API_H
#include "ff_ae_hist_ccbcr.h"


#define ff_ae_hist_set_register(FF_AE_HIST_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_AE_HIST_id + addr, (val))

#define ff_ae_hist_get_register(FF_AE_HIST_id, addr) \
	vied_subsystem_load_32(psys0, FF_AE_HIST_id + addr)


#define ff_ae_hist_set_rst_enable(FF_AE_HIST_id, rst)								\
	do {											\
	int reg_value = ff_ae_hist_get_register(FF_AE_HIST_id, FF_AE_GLB_HIST_GRD_CFG_ADDR);\
	ff_ae_hist_set_register(FF_AE_HIST_id, FF_AE_GLB_HIST_GRD_CFG_ADDR, ((rst <<30) | (reg_value & 0xBFFFFFFF)));\
	}while(0);

#define ff_ae_hist_get_rst_done(FF_AE_HIST_id,rst_done)								    \
do {											                                        \
	int reg_value = ff_ae_hist_get_register(FF_AE_HIST_id, FF_AE_GLB_HIST_GRD_CFG_ADDR);\
	rst_done = reg_value>>31;\
}while(0)


#define ff_ae_hist_set_cfg(FF_AE_HIST_id, AE_en, grid_width, grid_height, block_width, block_height, rst_hist_array)    \
do {                                                                                                                    \
    ff_ae_hist_set_register(FF_AE_HIST_id, FF_AE_GLB_HIST_GRD_CFG_ADDR, ((rst_hist_array) << 30) | ((AE_en) << 29) |    \
                            ((block_height) << 20) | ((block_width) << 16) | ((grid_height) << 8) | (grid_width));      \
}                                                                                                                       \
while (0)


#define ff_ae_hist_get_rst_hist_array_done(FF_AE_HIST_id)   \
       ((unsigned int)ff_ae_hist_get_register(FF_AE_HIST_id, FF_AE_GLB_HIST_GRD_CFG_ADDR) >> 31)

#define ff_ae_hist_set_grd_Offset_start(FF_AE_HIST_id, x_start, y_start) \
	ff_ae_hist_set_register(FF_AE_HIST_id, FF_AE_GLB_HIST_GRD_OFFST_ADDR, ((y_start & 0x3FFF) << 16) | (x_start & 0x3FFF));

#define ff_ae_hist_set_grd_end(FF_AE_HIST_id, x_end, y_end) \
	ff_ae_hist_set_register(FF_AE_HIST_id, FF_AE_GLB_HIST_GRD_END_ADDR, ((y_end & 0x3FFF) << 16) | (x_end & 0x3FFF));


#define ff_ae_hist_set_weight_grd(FF_AE_HIST_id, index, cells) \
	ff_ae_hist_set_register(FF_AE_HIST_id, MEM_AE_WEIGHT_GRID_BASE_ADDR + ((index) << 2), (((cells[7] & 0xF) << 28) |     \
                            ((cells[6] & 0xF) << 24) | ((cells[5] & 0xF) << 20) |                                           \
                            ((cells[4] & 0xF) << 16) | ((cells[3] & 0xF) << 12) |                                           \
                            ((cells[2] & 0xF) << 8) | ((cells[1] & 0xF) << 4) | (cells[0] & 0xF)));

#define ff_ae_hist_get_hist(FF_AE_HIST_id, colorId, bin, hist_bin)                                      \
do {                                                                                                    \
    unsigned int baseAddr[8] = {MEM_AE_GLB_HIST_C0_BASE_ADDR, MEM_AE_GLB_HIST_C1_BASE_ADDR,             \
                                MEM_AE_GLB_HIST_C2_BASE_ADDR, MEM_AE_GLB_HIST_C3_BASE_ADDR,             \
                                MEM_AE_GLB_HIST_C4_BASE_ADDR, MEM_AE_GLB_HIST_C5_BASE_ADDR,             \
                                MEM_AE_GLB_HIST_C6_BASE_ADDR, MEM_AE_GLB_HIST_C7_BASE_ADDR};            \
    hist_bin = (ff_ae_hist_get_register(FF_AE_HIST_id, baseAddr[colorId] + ((bin) << 2))) & 0x00FFFFFF; \
}                                                                                                       \
while (0)


#define ff_ae_hist_set_Sensor_Cfg(FF_id, sensorMode, Pattern)                                       \
do {                                                                                                \
    ff_ae_hist_set_register(FF_id, FF_AE_SENSOR_CFG_0_ADDR, (sensorMode));                          \
    ff_ae_hist_set_register(FF_id, FF_AE_SENSOR_CFG_1_ADDR, ((((Pattern[7]) & 0x7) << 28) |         \
                           (((Pattern[6]) & 0x7) << 24) | (((Pattern[5]) & 0x7) << 20) |            \
                           (((Pattern[4]) & 0x7) << 16) | (((Pattern[3]) & 0x7) << 12) |            \
                           (((Pattern[2]) & 0x7) << 8)  | (((Pattern[1]) & 0x7) << 4) | ((Pattern[0]) & 0x7))); \
    ff_ae_hist_set_register(FF_id, FF_AE_SENSOR_CFG_2_ADDR, ((((Pattern[15]) & 0x7) << 28) |        \
                           (((Pattern[14]) & 0x7) << 24) | (((Pattern[13]) & 0x7) << 20) |          \
                           (((Pattern[12]) & 0x7) << 16) | (((Pattern[11]) & 0x7) << 12) |          \
                           (((Pattern[10]) & 0x7) << 8)  | (((Pattern[9]) & 0x7) << 4) | ((Pattern[8]) & 0x7))); \
} while(0)

#endif // _FF_AE_HIST_API_H


