#ifndef _FF_Y_BIN_API_H
#define _FF_Y_BIN_API_H
#include <hrt/api.h>
#include "ff_y_bin_ccbcr.h"


#define FF_y_bin_set_register(FF_y_bin_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_y_bin_id + addr, (val))

#define FF_y_bin_get_register(FF_y_bin_id, addr) \
  vied_subsystem_load_32(psys0, FF_y_bin_id + addr)

#define FF_y_bin_set_2x2_mode(FF_y_bin_id)  	\
	FF_y_bin_set_register(FF_y_bin_id, Y_BIN_CTRL_ADDR, (0))

#define FF_y_bin_set_4x4_mode(FF_y_bin_id)  	\
	FF_y_bin_set_register(FF_y_bin_id, Y_BIN_CTRL_ADDR, (1))

#define FF_y_bin_get_bin_mode(FF_y_bin_id)  	\
	FF_y_bin_get_register(FF_y_bin_id, FF_y_bin_GEN_PARAM_ADDR)


#endif // _FF_Y_BIN_API_H


