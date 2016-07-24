#ifndef _HRT_MASTER_PORT_H 
#define _HRT_MASTER_PORT_H 

#include "type_support.h"
#include "var.h"

/* Congratulations, you have reached the end of the HRT.
 * Here we split between the hardware implementation (memcpy / assignments)
 * and the software backends (_hrt_master_port_load / _hrt_master_port_store)
 */

#define hrt_master_port_store_char(addr,data)   hrt_master_port_store_8 (addr,data)
#define hrt_master_port_store_short(addr,data)  hrt_master_port_store_16(addr,data)
#define hrt_master_port_store_int(addr,data)    hrt_master_port_store_32(addr,data)
#define hrt_master_port_store_long(addr,data)   hrt_master_port_store_32(addr,data)
#define hrt_master_port_store_uchar(addr,data)  hrt_master_port_store_8(addr,data)
#define hrt_master_port_store_ushort(addr,data) hrt_master_port_store_16(addr,data)
#define hrt_master_port_store_uint(addr,data)   hrt_master_port_store_32(addr,data)
#define hrt_master_port_store_ulong(addr,data)  hrt_master_port_store_32(addr,data)

#define hrt_master_port_load_char(addr)   ((int8_t)  hrt_master_port_load_8(addr))
#define hrt_master_port_load_short(addr)  ((int16_t) hrt_master_port_load_16(addr))
#define hrt_master_port_load_int(addr)    ((int32_t) hrt_master_port_load_32(addr))
#define hrt_master_port_load_long(addr)   ((int32_t) hrt_master_port_load_32(addr))
#define hrt_master_port_load_uchar(addr)  hrt_master_port_load_8(addr)
#define hrt_master_port_load_ushort(addr) hrt_master_port_load_16(addr)
#define hrt_master_port_load_uint(addr)   hrt_master_port_load_32(addr)
#define hrt_master_port_load_ulong(addr)  hrt_master_port_load_32(addr)



#define _hrt_master_port_store_8_volatile(address, data)   hrt_master_port_store_8(address, data)
#define _hrt_master_port_load_8_volatile(address)          ((int8_t)  hrt_master_port_load_8(address))
#define _hrt_master_port_uload_8_volatile(address)         hrt_master_port_load_8(address)
#define _hrt_master_port_store_16_volatile(address, data)  hrt_master_port_store_16(address, data)
#define _hrt_master_port_load_16_volatile(address)         ((int16_t) hrt_master_port_load_16(address))
#define _hrt_master_port_uload_16_volatile(address)        hrt_master_port_load_16(address)
#define _hrt_master_port_store_32_volatile(address, data)  hrt_master_port_store_32(address, data)
#define _hrt_master_port_load_32_volatile(address)         ((int32_t) hrt_master_port_load_32(address))
#define _hrt_master_port_uload_32_volatile(address)        hrt_master_port_load_32(address)

#define _hrt_master_port_store_8_volatile_msg(a,d,m)  _hrt_master_port_store_8_msg(a,d,m)
#define _hrt_master_port_store_16_volatile_msg(a,d,m) _hrt_master_port_store_16_msg(a,d,m)
#define _hrt_master_port_store_32_volatile_msg(a,d,m) _hrt_master_port_store_32_msg(a,d,m)
#define _hrt_master_port_load_8_volatile_msg(a,m)     _hrt_master_port_load_8_msg(a,m) 
#define _hrt_master_port_load_16_volatile_msg(a,m)    _hrt_master_port_load_16_msg(a,m)
#define _hrt_master_port_load_32_volatile_msg(a,m)    _hrt_master_port_load_32_msg(a,m)
#define _hrt_master_port_uload_8_volatile_msg(a,m)    _hrt_master_port_uload_8_msg(a,m)
#define _hrt_master_port_uload_16_volatile_msg(a,m)   _hrt_master_port_uload_16_msg(a,m)
#define _hrt_master_port_uload_32_volatile_msg(a,m)   _hrt_master_port_uload_32_msg(a,m)

#ifdef HRT_HW

#define _hrt_master_port_store_8_msg(a,d,m)           hrt_master_port_store_8(a,d)
#define _hrt_master_port_store_16_msg(a,d,m)          hrt_master_port_store_16(a,d)
#define _hrt_master_port_store_32_msg(a,d,m)          hrt_master_port_store_32(a,d)
#define _hrt_master_port_load_8_msg(a,m)              ((int8_t)  hrt_master_port_load_8(a))
#define _hrt_master_port_load_16_msg(a,m)             ((int16_t) hrt_master_port_load_16(a))
#define _hrt_master_port_load_32_msg(a,m)             ((int32_t) hrt_master_port_load_32(a))
#define _hrt_master_port_uload_8_msg(a,m)             hrt_master_port_load_8(a)
#define _hrt_master_port_uload_16_msg(a,m)            hrt_master_port_load_16(a)
#define _hrt_master_port_uload_32_msg(a,m)            hrt_master_port_load_32(a)
#define _hrt_master_port_unaligned_store_msg(a,d,s,m) hrt_master_port_store(a,d,s)
#define _hrt_master_port_unaligned_load_msg(a,d,s,m)  hrt_master_port_load(a,d,s)
#define _hrt_master_port_unaligned_set_msg(a,d,s,m)   hrt_master_port_set(a,d,s)

#else   /* HRT_HW */

#define _hrt_master_port_store_8_msg(a,d,m)           ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_store_8(a,d); })
#define _hrt_master_port_store_16_msg(a,d,m)          ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_store_16(a,d); })
#define _hrt_master_port_store_32_msg(a,d,m)          ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_store_32(a,d); })
#define _hrt_master_port_load_8_msg(a,m)              ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); ((int8_t)  hrt_master_port_load_8(a)); })
#define _hrt_master_port_load_16_msg(a,m)             ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); ((int16_t) hrt_master_port_load_16(a)); })
#define _hrt_master_port_load_32_msg(a,m)             ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); ((int32_t) hrt_master_port_load_32(a)); })
#define _hrt_master_port_uload_8_msg(a,m)             ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_load_8(a); })
#define _hrt_master_port_uload_16_msg(a,m)            ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_load_16(a); })
#define _hrt_master_port_uload_32_msg(a,m)            ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_load_32(a); })
#define _hrt_master_port_unaligned_store_msg(a,d,s,m) ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_store(a,d,s); })
#define _hrt_master_port_unaligned_load_msg(a,d,s,m)  ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_load(a,d,s); })
#define _hrt_master_port_unaligned_set_msg(a,d,s,m)   ({ if(_hrt_not_null(m)) hrt_error((char*)(m)); hrt_master_port_set(a,d,s); })

#endif  /* HRT_HW */

#include "hrt_backend.h"


#endif /* _HRT_MASTER_PORT_H */
