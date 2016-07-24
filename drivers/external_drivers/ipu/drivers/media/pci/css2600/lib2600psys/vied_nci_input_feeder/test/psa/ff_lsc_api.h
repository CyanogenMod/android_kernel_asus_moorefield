#ifndef _ff_lsc_api_h
#define _ff_lsc_api_h

#include "ff_lsc_ccbcr.h"

#define ff_lsc_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr)

#define ff_lsc_get_register(FF_id, addr) \
	vied_subsystem_load_32(ff_lsc_crq_port(FF_id), addr)

#define ff_lsc_set_grid_params(FF_id, grid_width, grid_height, block_width, block_height, grid_height_per_slice) \
	ff_lsc_set_register(FF_id, FF_LSC_GRD_CFG0_ADDR, grid_width | 			                         \
							    (grid_height << 8) |		                 \
							    (block_width << 16) |		                 \
							    (block_height << 20) |		                 \
							    (grid_height_per_slice << 24));

#define ff_lsc_set_grid_start(FF_id, x_start, y_start)	                                                        \
	ff_lsc_set_register(FF_id, FF_LSC_GRD_CFG1_ADDR, (x_start | (y_start << 16)));

#define ff_lsc_disable(FF_id)								                         \
	ff_lsc_set_register(FF_id, FF_LSC_GRD_CFG2_ADDR, 0x00000000);

#define ff_lsc_enable_and_set_Exp_and_sensor_mode(FF_id, LscExp, sensor_mode)                                    \
        ff_lsc_set_register(FF_id, FF_LSC_GRD_CFG2_ADDR, ((0x00000001 << 8)  | ((LscExp) << 12) | ((sensor_mode) << 16)));

#define ff_lsc_set_lut_entry_1_1(FF_id, SetIdx, GridIdx, val1, val2)                                        \
do {								                                            \
	unsigned int lutBase, address, LutIdx, RegIdx;                                                      \
        LutIdx = floor((GridIdx % 8) / 2);                                                                  \
        RegIdx = (64 * SetIdx) + floor( GridIdx/8);                                                         \
        if (0 == LutIdx) {                                                                                  \
            lutBase = MEM_LUT_CH0_SET0_BASE_ADDR;                                                           \
        } else if (1 == LutIdx) {					                                    \
            lutBase = MEM_LUT_CH1_SET0_BASE_ADDR;                                                           \
        } else if (2 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH2_SET0_BASE_ADDR;                                                            \
        } else if (3 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH3_SET0_BASE_ADDR;                                                            \
        }                                                                                                   \
        address = (RegIdx << 2) + lutBase; 				                                    \
        ff_lsc_set_register(FF_id, address, (val2 << 16) | (val1 & 0xFFFF));                               \
        }                                                                                                   \
while(0)

#define ff_lsc_set_lut_entry_2_2(FF_id, SetIdx, GridIdx, val1, val2, val3, val4)                            \
do {								                                            \
	unsigned int lutBase, address, LutIdx, RegIdx;                                                      \
        LutIdx = (2 * (GridIdx % 2)) + 0;                                                                   \
        RegIdx = (64 * SetIdx) + floor( GridIdx/2);                                                         \
        if (0 == LutIdx) {                                                                                  \
            lutBase = MEM_LUT_CH0_SET0_BASE_ADDR;                                                           \
        } else if (1 == LutIdx) {					                                    \
            lutBase = MEM_LUT_CH1_SET0_BASE_ADDR;                                                           \
        } else if (2 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH2_SET0_BASE_ADDR;                                                            \
        } else if (3 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH3_SET0_BASE_ADDR;                                                            \
        }                                                                                                   \
        address = (RegIdx << 2) + lutBase; 				                                    \
        ff_lsc_set_register(FF_id, address, (val2 << 16) | (val1 & 0xFFFF));                               \
        LutIdx = (2 * (GridIdx % 2)) + 1;                                                                   \
        if (0 == LutIdx) {                                                                                  \
            lutBase = MEM_LUT_CH0_SET0_BASE_ADDR;                                                           \
        } else if (1 == LutIdx) {					                                    \
            lutBase = MEM_LUT_CH1_SET0_BASE_ADDR;                                                           \
        } else if (2 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH2_SET0_BASE_ADDR;                                                            \
        } else if (3 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH3_SET0_BASE_ADDR;                                                            \
        }                                                                                                   \
        address = (RegIdx << 2) + lutBase; 				                                    \
        ff_lsc_set_register(FF_id, address, (val4 << 16) | (val3 & 0xFFFF));                                \
}                                                                                                           \
while(0)

#define ff_lsc_set_lut_entry_4_4(FF_id, SetIdx, GridIdx, map_arr)                                           \
do {								                                            \
	unsigned int lutBase, address, LutIdx, RegIdx, channel;                                             \
        short val1,val2;                                                                                    \
        int data;                                                                                           \
        for (channel=0; channel<16; channel+=2) {                                                           \
         channelRow = ((channel) - (channel % 4))/4;                                                        \
         channelColumn = (channel % 4);                                                                     \
         LutIdx = ((channelRow % 2) * 2) + floor(channelColumn/2);                                          \
         RegIdx = (64 * SetIdx) + (32 * floor(channelRow/2)) + GridIdx;                                     \
         val1 = map_arr[channel];                                                                           \
         val2 = map_arr[channel + 1];                                                                       \
         if (0 == LutIdx) {                                                                                 \
           lutBase = MEM_LUT_CH0_SET0_BASE_ADDR;                                                            \
         } else if (1 == LutIdx) {					                                    \
            lutBase = MEM_LUT_CH1_SET0_BASE_ADDR;                                                           \
         } else if (2 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH2_SET0_BASE_ADDR;                                                            \
         } else if (3 == LutIdx) {					                                    \
           lutBase = MEM_LUT_CH3_SET0_BASE_ADDR;                                                            \
         }                                                                                                  \
            address = (RegIdx << 2) + lutBase; 				                                    \
            ff_lsc_set_register(FF_id, address, (val2 << 16) | (val1 & 0xFFFF));                            \
         }                                                                                                  \
 }                                                                                                          \
while(0)
#endif // _ff_lsc_api_h


