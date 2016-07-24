#ifndef _HRT_MEMORY_H
#define _HRT_MEMORY_H

#include "hive_types.h"
#include "slave.h"
#include "system_api.h"

/* these macros are also in <core>/include/hive/attributes.h 
 * therefor we need to undefine them for the crun
 */
#ifndef _HRT_NO_OVERRIDE_MEM_ATTRS
#ifdef MEM
#undef MEM
#endif
#define MEM(memory)

#ifdef ON
#undef ON
#endif
#define ON(resource)

#ifdef AT
#undef AT
#endif
#define AT(address)

#ifdef MAP_TO
#undef MAP_TO
#endif
#define MAP_TO(memory,address)

#ifdef SYNC_WITH
#undef SYNC_WITH
#endif
#define SYNC_WITH(idx)
#endif /* __HIVECC */

/* fixed data size, aligned address, pass by value*/
#define hrt_mem_load_8(cell, mem, addr)  HRTCAT(_hrt_mem_load_8_, mem)(cell, addr)
#define hrt_mem_uload_8(cell, mem, addr)  HRTCAT(_hrt_mem_uload_8_, mem)(cell, addr)
#define hrt_mem_load_16(cell, mem, addr) HRTCAT(_hrt_mem_load_16_, mem)(cell, addr)
#define hrt_mem_uload_16(cell, mem, addr) HRTCAT(_hrt_mem_uload_16_, mem)(cell, addr)
#define hrt_mem_load_32(cell, mem, addr) HRTCAT(_hrt_mem_load_32_, mem)(cell, addr)
#define hrt_mem_uload_32(cell, mem, addr) HRTCAT(_hrt_mem_uload_32_, mem)(cell, addr)

#define hrt_mem_store_8(cell, mem, addr, value)  HRTCAT(_hrt_mem_store_8_, mem)(cell, addr, value)
#define hrt_mem_store_16(cell, mem, addr, value) HRTCAT(_hrt_mem_store_16_, mem)(cell, addr, value)
#define hrt_mem_store_32(cell, mem, addr, value) HRTCAT(_hrt_mem_store_32_, mem)(cell, addr, value)

/* any data size, unaligned address, pass by reference */
#define hrt_mem_load(cell, mem, addr, data, size)  HRTCAT(_hrt_mem_load_,  mem)(cell, addr, data, size)
#define hrt_mem_store(cell, mem, addr, data, size) HRTCAT(_hrt_mem_store_, mem)(cell, addr, data, size)
#define hrt_mem_set(cell, mem, addr, data, size)   HRTCAT(_hrt_mem_set_,   mem)(cell, addr, data, size)
#define hrt_mem_zero(cell, mem, address, size)     hrt_mem_set(cell, mem, address, 0, size)

/* support for C builtin data types
 * the sizes used are the sizes as defined on the hive processor.
  */
#define hrt_mem_load_char(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,char) (cell, mem, addr)
#define hrt_mem_load_uchar(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,uchar) (cell, mem, addr)
#define hrt_mem_load_short(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,short) (cell, mem, addr)
#define hrt_mem_load_ushort(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,ushort) (cell, mem, addr)
#define hrt_mem_load_int(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,int) (cell, mem, addr)
#define hrt_mem_load_uint(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,uint) (cell, mem, addr)
#define hrt_mem_load_long(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,long) (cell, mem, addr)
#define hrt_mem_load_ulong(cell, mem, addr) \
  HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,ulong) (cell, mem, addr)

#define hrt_mem_store_char(cell, mem, addr, value) \
  HRTCAT(hrt_mem_store_,HRT_TYPE_BITS(cell, char))(cell, mem, addr, value)
#define hrt_mem_store_short(cell, mem, addr, value) \
  HRTCAT(hrt_mem_store_,HRT_TYPE_BITS(cell, short))(cell, mem, addr, value)
#define hrt_mem_store_int(cell, mem, addr, value) \
  HRTCAT(hrt_mem_store_,HRT_TYPE_BITS(cell, int))(cell, mem, addr, value)
#define hrt_mem_store_long(cell, mem, addr, value) \
  HRTCAT(hrt_mem_store_,HRT_TYPE_BITS(cell, long))(cell, mem, addr, value)

#define hrt_mem_store_uchar(cell, mem, addr, value) \
  hrt_mem_store_char(cell, mem, addr, value)
#define hrt_mem_store_ushort(cell, mem, addr, value) \
  hrt_mem_store_short(cell, mem, addr, value)
#define hrt_mem_store_uint(cell, mem, addr, value) \
  hrt_mem_store_int(cell, mem, addr, value)
#define hrt_mem_store_ulong(cell, mem, addr, value) \
  hrt_mem_store_long(cell, mem, addr, value)

/* 48 bits support */
#define hrt_mem_store_48(cell, mem, addr, value) \
{\
  long long val = (long long)value; \
  hrt_mem_store(cell, mem, addr, (void*)&val, 5); \
}

#define hrt_mem_load_48(cell, mem, addr) \
({ \
  long long val  = 0;\
  hrt_mem_load(cell, mem, addr, (void*)&val, 5);\
  val; \
})

#define hrt_mem_uload_48(cell, mem, addr) \
({ \
  unsigned long long val  = 0;\
  hrt_mem_load(cell, mem, addr, (void*)&val, 5);\
  val; \
})

#define hrt_mem_slave_port_address(cell, mem) \
  _hrt_cell_mem_master_to_slave_address(cell, mem, 0x0)

#define hrt_mem_master_port_address(cell, mem) \
  (_hrt_slave_port_master_port_address(_hrt_cell_mem_slave_port(cell, mem)) + hrt_mem_slave_port_address(cell, mem))

#endif /* _HRT_MEMORY_H */
