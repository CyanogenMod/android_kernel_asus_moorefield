#ifndef _HRT_SYSTEM_MEMORY_H
#define _HRT_SYSTEM_MEMORY_H

#include "hive_types.h"
#include "slave.h"
#include "system_api.h"

#ifdef C_RUN_DYNAMIC_LINK_PROGRAMS
/* If HIVEDEVICES is overridden such that it does not contain
   the default hss devices  anymore then the
   #include <sysmem_request_hrt.h> will fail.
   Hint: add $(HIVEBIN)/../devices */
#include <sysmem_request_hrt.h> /* Memory/hrt/include */
#include "expert_api.h"
#endif

#define _hrt_sysmem_slave_port(mem) HRTCAT(mem, _ip0)

#if defined(C_RUN) && !defined(C_RUN_DYNAMIC_LINK_PROGRAMS)

#define hrt_sysmem_scalar_store(type, var, data, mem, addr) \
  ((var) = (data))
#define hrt_sysmem_scalar_load(type, var, mem, addr) \
  (var)

#define hrt_sysmem_indexed_store(type, array, index, data, mem, addr) \
  (((array)[index]) = (data))
#define hrt_sysmem_indexed_load(type, array, index, mem, addr) \
  ((array)[index])

#else /* C_RUN */

#ifdef C_RUN_DYNAMIC_LINK_PROGRAMS

static void *
__hrt_sysmem_ident_address(hive_device_id mem, const char *sym)
{
  hrt_sysmem_request_t request;
  hrt_sysmem_init_ident_address_request(sym, &request);
  return hrtx_sysmem_simulator_request(mem, &request);
}

#define _hrt_sysmem_ident_address(mem,sym) ((unsigned int) __hrt_sysmem_ident_address(mem,HRTSTR(sym)))
#define hrt_sysmem_scalar_store(type, var, data, mem, addr) \
  ({ \
    hive_address var_addr = _hrt_sysmem_ident_address(mem,var); \
    HRTCAT(hrt_sysmem_store_,type)(mem, var_addr, data); \
  })
#define hrt_sysmem_scalar_load(type, var, mem, addr) \
  ({ \
    hive_address var_addr = _hrt_sysmem_ident_address(mem,var); \
    HRTCAT(hrt_sysmem_load_,type)(mem, var_addr); \
  })
#define hrt_sysmem_indexed_store(type, array, index, data, mem, addr) \
  ({ \
    hive_address array_addr = _hrt_sysmem_ident_address(mem,array); \
    HRTCAT(hrt_sysmem_store_,type)(mem, array_addr + ((index)*sizeof(type)), data); \
  })
#define hrt_sysmem_indexed_load(type, array, index, mem, addr) \
  ({ \
    hive_address array_addr = _hrt_sysmem_ident_address(mem,array); \
    HRTCAT(hrt_sysmem_load_,type)(mem, array_addr + ((index)*sizeof(type))); \
  }) 

static void
_hrt_sysmem_map_var(hive_mem_id mem, const char *ident, volatile void *native_address, unsigned int size)
{
  hrt_sysmem_request_t request;
  hrt_sysmem_init_map_ident_request(ident, native_address, size, &request);
  return (void) hrtx_sysmem_simulator_request(mem, &request);
}

#define hrt_sysmem_map_scalar(mem,var)  _hrt_sysmem_map_var(mem,HRTSTR(var),&var,sizeof(var))
#define hrt_sysmem_map_indexed(mem,var) _hrt_sysmem_map_var(mem,HRTSTR(var),var,sizeof(var))

#else /* C_RUN_DYNAMIC_LINK_PROGRAMS */

#define hrt_sysmem_scalar_store(type, var, data, mem, addr) \
        HRTCAT(hrt_sysmem_store_,type)(mem, addr, data)
#define hrt_sysmem_scalar_load(type, var, mem, addr) \
        HRTCAT(hrt_sysmem_load_,type)(mem, addr)
#define hrt_sysmem_indexed_store(type, array, index, data, mem, addr) \
        HRTCAT(hrt_sysmem_store_,type)(mem, (addr) + ((index)*sizeof(type)), data)
#define hrt_sysmem_indexed_load(type, array, index, mem, addr) \
        HRTCAT(hrt_sysmem_load_,type)(mem, (addr) + ((index)*sizeof(type)))

#endif /* C_RUN_DYNAMIC_LINK_PROGRAMS */

#endif /* C_RUN */

#define hrt_sysmem_store(mem, addr, data, size) \
        _hrt_slave_port_store(_hrt_sysmem_slave_port(mem), addr, data, size)
#define hrt_sysmem_load(mem, addr, data, size) \
        _hrt_slave_port_load(_hrt_sysmem_slave_port(mem), addr, data, size)

#define hrt_sysmem_set(mem, addr, data, size) \
        _hrt_slave_port_set(_hrt_sysmem_slave_port(mem), addr, data, size)
#define hrt_sysmem_zero(mem, address, size) \
        hrt_sysmem_set(mem, address, 0, size)

#define hrt_sysmem_bus_address(mem) \
        _hrt_slave_port_master_port_address(_hrt_sysmem_slave_port(mem))

#define hrt_sysmem_load_8(sysmem, addr) \
  _hrt_slave_port_load_8(_hrt_sysmem_slave_port(sysmem), (addr))
#define hrt_sysmem_load_16(sysmem, addr) \
  _hrt_slave_port_load_16(_hrt_sysmem_slave_port(sysmem), (addr))
#define hrt_sysmem_load_32(sysmem, addr) \
  _hrt_slave_port_load_32(_hrt_sysmem_slave_port(sysmem), (addr))

#define hrt_sysmem_store_8(sysmem, addr, value) \
        _hrt_slave_port_store_8(_hrt_sysmem_slave_port(sysmem), (addr), value)
#define hrt_sysmem_store_16(sysmem, addr, value) \
        _hrt_slave_port_store_16(_hrt_sysmem_slave_port(sysmem), (addr), value)
#define hrt_sysmem_store_32(sysmem, addr, value) \
        _hrt_slave_port_store_32(_hrt_sysmem_slave_port(sysmem), (addr), value)

#define hrt_sysmem_load_char(sysmem, addr)  hrt_sysmem_load_8(sysmem, addr)
#define hrt_sysmem_load_short(sysmem, addr) hrt_sysmem_load_16(sysmem, addr)
#define hrt_sysmem_load_int(sysmem, addr)   hrt_sysmem_load_32(sysmem, addr)
#define hrt_sysmem_load_long(sysmem, addr)  hrt_sysmem_load_32(sysmem, addr)

#define hrt_sysmem_store_char(sysmem, addr, value)  hrt_sysmem_store_8(sysmem, addr, value)
#define hrt_sysmem_store_short(sysmem, addr, value) hrt_sysmem_store_16(sysmem, addr, value)
#define hrt_sysmem_store_int(sysmem, addr, value)   hrt_sysmem_store_32(sysmem, addr, value)
#define hrt_sysmem_store_long(sysmem, addr, value)  hrt_sysmem_store_32(sysmem, addr, value)

#endif /* _HRT_SYSTEM_MEMORY_H */
