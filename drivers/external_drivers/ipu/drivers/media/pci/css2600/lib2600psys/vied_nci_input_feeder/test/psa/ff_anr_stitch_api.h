#ifndef _ff_anr_stitch_api_h
#define _ff_anr_stitch_api_h

#include "ff_anr_stitch_ccbcr.h"

#define ff_anr_stitch_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_anr_stitch_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_anr_stitch_enable(FF_id) \
    ff_anr_stitch_set_register(FF_id, FF_ANR_STITCH_CFG_ADDR, 1);

#define ff_anr_stitch_set_frame_size(FF_id, width, height) \
	ff_anr_stitch_set_register(FF_id, FF_ANR_STITCH_CFG1_ADDR, ((height) << 16) | (width))

#define ff_anr_stitch_disable(FF_id) \
    ff_anr_stitch_set_register(FF_id, FF_ANR_STITCH_CFG_ADDR, 0);

#define ff_anr_stitch_matrix(FF_id, arr)                   \
do {                                                       \
    unsigned int reg_addr = FF_ANR_STITCH_REG_0_ADDR;      \
    unsigned int arr_idx = 0;                              \
    while (arr_idx < 64) {                                 \
        unsigned int data = arr[arr_idx++];                \
        if (arr_idx < 64) {                                \
            data |= arr[arr_idx++] << 6;                   \
            data |= arr[arr_idx++] << 12;                  \
        }                                                  \
        ff_anr_stitch_set_register(FF_id, reg_addr, data); \
        reg_addr += 4;                                     \
    }                                                      \
}                                                          \
while (0)

#endif
