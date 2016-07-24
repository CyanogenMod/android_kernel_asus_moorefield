#ifndef _ff_pcln_api_h
#define _ff_pcln_api_h

#include "ff_pcln_ccbcr.h"


#define ff_pcln_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_pcln_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_pcln_set_sensor_mode_and_LinEn(FF_id, sensor_mode, PclnEn) \
    ff_pcln_set_register(FF_id, FF_PCLN_CTRL_ADDR, ((sensor_mode&0x3) << 4) | (PclnEn&0x1));

#define ff_pcln_get_sensor_mode_and_LinEn(FF_id, sensor_mode, PclnEn)   \
 do {                                                                   \
   unsigned int val = ff_pcln_get_register(FF_id, FF_PCLN_CTRL_ADDR);   \
   sensor_mode = val >> 4;                                              \
   PclnEn = val & 0x00000001;                                           \
 }                                                                      \
 while (0)


#define ff_pcln_set_matrix(FF_id, arr)                                          \
do {                                                                            \
    unsigned int data;											                \
    unsigned int reg_addr = MEM_BANK_FF_PCLN_LUT_ID0_BASE_REG_ID;               \
    unsigned int arr_idx, ent_idx;                                              \
    for(arr_idx = 0; arr_idx < 16; arr_idx++)                                   \
    {                                                                           \
        for (ent_idx = 0; ent_idx < 32; ent_idx++)                              \
        {                                                                       \
            data  = arr[arr_idx][2*ent_idx];            			            \
            data |= arr[arr_idx][2*ent_idx+1] << 16;                            \
            ff_pcln_set_register(FF_id, reg_addr, data);                        \
            reg_addr += 4;                                                      \
        }                                                                       \
        if(arr_idx < 8)                                                         \
        {                                                                       \
            data  = arr[arr_idx][64];                      			            \
            data  |= arr[arr_idx][65] << 16;                                    \
            ff_pcln_set_register(FF_id, FF_PCLN_LUT_64_ID0_ADDR + arr_idx*4, data);   \
        }                                                                       \
	}                                                                           \
}                                                                               \
while (0)

#endif
