#ifndef _ff_yuv_collector_api_h
#define _ff_yuv_collector_api_h

#include "ff_yuv_collector_ccbcr.h"




#define ff_yuv_collector_set_register(FF_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_yuv_collector_get_register(FF_id, addr) \
  vied_subsystem_load_32(psys0, FF_id + addr)


#endif // _ff_yuv_collector_api_h

