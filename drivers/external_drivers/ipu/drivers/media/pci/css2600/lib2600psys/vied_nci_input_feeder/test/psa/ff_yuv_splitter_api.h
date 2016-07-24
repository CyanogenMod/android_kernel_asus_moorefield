#ifndef _ff_yuv_splitter_api_h
#define _ff_yuv_splitter_api_h

#include "ff_yuv_splitter_ccbcr.h"



#define ff_yuv_splitter_set_register(FF_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_yuv_splitter_get_register(FF_id, addr) \
  vied_subsystem_load_32(psys0, FF_id + addr)



#define ff_yuv_splitter_set_Col_Size(FF_id, ColSize ) \
do {												\
	ff_yuv_splitter_set_register(FF_id, FF_YUV_SPLITTER_COL_SIZE_ADDR,  ColSize);		\
} while(0);

#endif // _ff_yuv_splitter_api_h
