#ifndef _ff_tile_to_strm_api_h
#define _ff_tile_to_strm_api_h

#include "ff_tile_to_strm_ccbcr.h"


#define ff_tile_to_strm_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_tile_to_strm_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_tile_to_strm_enable(FF_id) \
	ff_tile_to_strm_set_register(FF_id, FF_TILE_TO_STRM_CFG0_ADDR, 1)

#define ff_tile_to_strm_set_frame_size(FF_id, width, height) \
	ff_tile_to_strm_set_register(FF_id, FF_TILE_TO_STRM_CFG1_ADDR, ((height) << 16) | (width))

#define ff_tile_to_strm_disable(FF_id) \
	ff_tile_to_strm_set_register(FF_id, FF_TILE_TO_STRM_CFG0_ADDR, 0);


#endif
