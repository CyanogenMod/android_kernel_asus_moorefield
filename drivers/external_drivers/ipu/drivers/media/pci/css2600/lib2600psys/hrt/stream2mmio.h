#ifndef _HRT_stream2mmio_H
#define _HRT_stream2mmio_H

#include <hrt/api.h>

#include "stream2mmio_defs.h"

#define hrt_stream2mmio_slave_port(stream2mmio_id) HRTCAT(stream2mmio_id,_sl_in)
//#define hrt_stream2mmio_master_port(stream2mmio_id) HRTCAT(stream2mmio_id,_mt_out)
#define hrt_stream2mmio_register_address(reg)  (_STREAM2MMIO_REG_ALIGN * (reg))

#define hrt_stream2mmio_set_register(stream2mmio_id, reg, val) \
  _hrt_slave_port_store_32_volatile( hrt_stream2mmio_slave_port(stream2mmio_id), hrt_stream2mmio_register_address(reg), (val))

#define hrt_stream2mmio_get_register(stream2mmio_id, reg) \
  _hrt_slave_port_load_32_volatile( hrt_stream2mmio_slave_port(stream2mmio_id), hrt_stream2mmio_register_address(reg))

#define hrt_stream2mmio_set_sid_register(stream2mmio_id, sid, reg, val) \
  hrt_stream2mmio_set_register(stream2mmio_id, (16+reg+(sid*_STREAM2MMIO_SID_REG_OFFSET)), (val))

#define hrt_stream2mmio_get_sid_register(stream2mmio_id, sid, reg) \
  hrt_stream2mmio_get_register(stream2mmio_id, (16+reg+(sid*_STREAM2MMIO_SID_REG_OFFSET)))

#define hrt_stream2mmio_snd_cmd(stream2mmio_id, sid, cmd) \
  hrt_stream2mmio_set_sid_register(stream2mmio_id, sid, _STREAM2MMIO_COMMAND_REG_ID, cmd)

#define hrt_stream2mmio_rcv_ack(stream2mmio_id, sid) \
  hrt_stream2mmio_get_sid_register(stream2mmio_id, sid, _STREAM2MMIO_ACKNOWLEDGE_REG_ID)

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////
// Special function for MIPI Stream2MMIO (more sid registers)
#define hrt_mipi_stream2mmio_set_sid_register(stream2mmio_id, sid, reg, val) \
  hrt_stream2mmio_set_register(stream2mmio_id, (16+reg+(sid*_MIPI_STREAM2MMIO_SID_REG_OFFSET)), (val))

#define hrt_mipi_stream2mmio_get_sid_register(stream2mmio_id, sid, reg) \
  hrt_stream2mmio_get_register(stream2mmio_id, (16+reg+(sid*_MIPI_STREAM2MMIO_SID_REG_OFFSET)))

///////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////////

#endif /* _HRT_stream2mmio_H */
