#ifndef _ff_anr_search_api_h
#define _ff_anr_search_api_h

#include "ff_anr_search_ccbcr.h"


#define ff_anr_search_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_anr_search_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_anr_search_enable(FF_id) \
	ff_anr_search_set_register(FF_id, FF_ANR_SEARCH_CFG0_ADDR, 1)

#define ff_anr_search_set_frame_size(FF_id, width, height) \
	ff_anr_search_set_register(FF_id, FF_ANR_SEARCH_CFG1_ADDR, ((height) << 16) | (width))

#define ff_anr_search_disable(FF_id) \
	ff_anr_search_set_register(FF_id, FF_ANR_SEARCH_CFG0_ADDR, 0)

#endif
