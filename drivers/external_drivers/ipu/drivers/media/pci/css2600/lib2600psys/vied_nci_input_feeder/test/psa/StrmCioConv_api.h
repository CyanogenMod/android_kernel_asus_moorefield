#ifndef _STRM_CIO_CONV_API_H
#define _STRM_CIO_CONV_API_H

#include <hrt/api.h>
#include "StrmCioConv_ccbcr.h"

#define HRT_StrmCioConv_set_register(StrmCioConv_id, addr, val) \
  vied_subsystem_store_32(psys0, StrmCioConv_id + addr, (val))

#define HRT_StrmCioConv_get_register(StrmCioConv_id, addr) \
  vied_subsystem_store_32(psys0, StrmCioConv_id + addr)



// More target focused APIs
#define HRT_StrmCioConv_SetAddr(StrmCioConv_id, Addr) \
  HRT_StrmCioConv_set_register(StrmCioConv_id, ACK_ADDR_ADDR, Addr);

#define HRT_StrmCioConv_SetCmd(StrmCioConv_id, Cmd) \
  HRT_StrmCioConv_set_register(StrmCioConv_id, ACK_CMD_ADDR, Cmd);


#define HRT_StrmCioConv_GetAddr(StrmCioConv_id) \
  HRT_StrmCioConv_get_register(StrmCioConv_id, ACK_ADDR_ADDR);

#define HRT_StrmCioConv_GetCmd(StrmCioConv_id) \
  HRT_StrmCioConv_get_register(StrmCioConv_id, ACK_CMD_ADDR);

#endif // _STRM_CIO_CONV_API_H
