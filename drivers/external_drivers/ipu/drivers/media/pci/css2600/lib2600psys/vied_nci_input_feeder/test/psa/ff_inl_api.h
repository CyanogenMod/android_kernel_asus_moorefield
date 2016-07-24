#ifndef _ff_inl_api_h
#define _ff_inl_api_h

#include "ff_inl_ccbcr.h"

#define ff_inl_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_inl_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_inl_set_shift_depth_and_LinEn(FF_id, shiftdepth, linEn) \
    ff_inl_set_register(FF_id, FF_INL_CTRL_ADDR, ((shiftdepth) << 4) | (linEn));

#define ff_inl_set_shift_depth_and_ValBypass(FF_id, shiftdepth, valBypass) \
    ff_inl_set_register(FF_id, FF_INL_CTRL_ADDR, ((shiftdepth) << 4) | (valBypass));


#define ff_inl_set_matrix(FF_id, arr)                               \
do {                                                                \
    unsigned int data;						    \
    unsigned int reg_addr = MEM_FF_INL_EVENODD_LUT_BASE_ADDR;       \
    unsigned int arr_idx = 0;                                       \
    while (arr_idx < 256) {                                         \
            data  = arr[arr_idx++];                 		    \
            data |= arr[arr_idx++] << 16;                           \
	    ff_inl_set_register(FF_id, reg_addr, data);             \
			reg_addr += 4;                              \
    }                                                               \
    reg_addr = FF_INL_LUT_256_ADDR;                                 \
    data  = arr[arr_idx];                                           \
    ff_inl_set_register(FF_id, FF_INL_LUT_256_ADDR, data);               	    \
}                                                                   \
while (0)

#endif
