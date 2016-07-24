#ifndef _ff_dm_api_h
#define _ff_dm_api_h

#include "ff_dm_ccbcr.h"


#define ff_dm_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_dm_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_dm_enable(FF_id, char_en, fcc_en, frame_width) \
	ff_dm_set_register(FF_id, FF_DM_CTRL0_ADDR, (1 | ((char_en) << 1) | ((fcc_en) << 2) | ((frame_width) << 16)))

#define ff_dm_disable(FF_id) \
	ff_dm_set_register(FF_id, FF_DM_CTRL0_ADDR, 0)

#define ff_dm_set_coeffs_ctrl(FF_id, gamma_sc, lc_ctrl, cr_param1, cr_param2, coring_param)                             \
do {                                                                                                                    \
    ff_dm_set_register(FF_id, FF_DM_CTRL1_ADDR, ((gamma_sc) | ((lc_ctrl)<<8) | ((cr_param1)<<16) | ((cr_param2)<<24))); \
    ff_dm_set_register(FF_id, FF_DM_CTRL2_ADDR, (coring_param));                                                        \
}                                                                                                                       \
while (0)

#define ff_dm_get_coeffs_ctrl(FF_id, gamma_sc, lc_ctrl, cr_param1, cr_param2, coring_param) \
do {                                                                                        \
    unsigned int val = ff_dm_get_register(FF_id, FF_DM_CTRL1_ADDR);                         \
    gamma_sc = val & 0x1F;                                                                  \
    lc_ctrl = (val >> 8) & 0x1F;                                                            \
    cr_param1 = (val >> 16) & 0x1F;                                                         \
    cr_param2 = (val >> 24) & 0x1F;                                                         \
    val = ff_dm_get_register(FF_id, FF_DM_CTRL2_ADDR);                                      \
    coring_param = val & 0x1F;                                                              \
}                                                                                           \
while (0)


#endif
