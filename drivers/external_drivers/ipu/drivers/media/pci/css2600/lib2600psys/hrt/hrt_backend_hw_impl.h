#ifndef _HRT_BACKEND_HW_IMPL_H 
#define _HRT_BACKEND_HW_IMPL_H

#include <string.h>
#include "system_api.h"

#ifndef _HRT_MEM
#define _HRT_MEM
#endif

typedef volatile hive_int8  _HRT_MEM *hive_int8_v_ptr;
typedef volatile hive_int16  _HRT_MEM *hive_int16_v_ptr;
typedef volatile hive_int32  _HRT_MEM *hive_int32_v_ptr;

/* We need to support an address translation at the end of hrt on an
   ia32 host. Pointers on that host are 32-bits. The input address is
   more than 32-bits. Therefore we should not cast to hive pointers,
   otherwise address bits get lost. */
   
static inline void hrt_master_port_store_8(hrt_address address, uint8_t data)
{
  *(hive_int8_v_ptr)(address) = data;
}
static inline void hrt_master_port_store_16(hrt_address address, uint16_t data)
{
  *(hive_int16_v_ptr)(address) = data;
}
static inline void hrt_master_port_store_32(hrt_address address, uint32_t data)
{
  *(hive_int32_v_ptr)(address) = data;
}
static inline uint8_t hrt_master_port_load_8(hrt_address address)
{
  return *(hive_int8_v_ptr)(address);
}
static inline uint16_t hrt_master_port_load_16(hrt_address address)
{
  return *(hive_int16_v_ptr)(address);
}
static inline uint32_t hrt_master_port_load_32(hrt_address address)
{
  return *(hive_int32_v_ptr)(address);
}

static inline void hrt_master_port_store(hrt_address address, const void *data, size_t bytes)
{
  memcpy((void*)(address), (const char*)data, bytes);
}
static inline void hrt_master_port_load(hrt_address address, void *data, size_t bytes)
{
  memcpy((char*)data, (void*)(address), bytes);
}
static inline void hrt_master_port_set(hrt_address address, int data, size_t bytes)
{
  memset((void*)(address), data, bytes);
}

#endif /* _HRT_BACKEND_HW_HIVECC_VIRS_IMPL_H */
