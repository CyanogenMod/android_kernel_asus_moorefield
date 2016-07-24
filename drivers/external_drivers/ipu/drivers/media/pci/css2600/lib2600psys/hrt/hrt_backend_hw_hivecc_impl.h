#ifndef _HRT_BACKEND_HW_HIVECC_IMPL_H 
#define _HRT_BACKEND_HW_HIVECC_IMPL_H

/* If we run HRT on a hive processor, we need to make sure
   that the master port transactions go to the master interface,
   not to the local memory. */
#include <hive/support.h>

#ifdef HRT_MT_INT
#define _HRT_MEM MEM(HRT_MT_INT)
#endif
#ifndef _HRT_MEM
#define _HRT_MEM
#endif

typedef volatile hive_int8   _HRT_MEM *hive_int8_v_ptr;
typedef volatile hive_int16  _HRT_MEM *hive_int16_v_ptr;
typedef volatile hive_int32  _HRT_MEM *hive_int32_v_ptr;
typedef char                 _HRT_MEM *hive_char_ptr;
typedef const char           _HRT_MEM *hive_const_char_ptr;

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

/* Because of the master interface pointer attributes, we cannot use the
   regular memcpy and memset functions. So here are their replacements without
   any optimizations whatsoever. These functions are not expected to be used
   in production code, so the speed should not be critical. */

static inline void hrt_master_port_store(hrt_address address, const void *data, size_t bytes)
{
//  hive_char_ptr to = (hive_char_ptr)(address);
  hrt_address to = address;
  const char *from = (const char *)data;
  unsigned i;
  for(i=0; i<bytes; i++, to++, from++) hrt_master_port_store_8(to, *from);
}
static inline void hrt_master_port_load(hrt_address address, void *data, size_t bytes)
{
//  hive_const_char_ptr from = (hive_char_ptr)(address);
  hrt_address from = address;
  unsigned i;
  char *to = (char *) data;
  for(i=0; i<bytes; i++, to++, from++) *to = hrt_master_port_load_8(from);
}
static inline void hrt_master_port_set(hrt_address address, int data, size_t bytes)
{
  unsigned i;
  // hive_char_ptr _ptr = (hive_char_ptr)address;
  hrt_address ptr = address;
  for(i=0; i<bytes; i++, ptr++) hrt_master_port_store_8(ptr, data);
}
#endif /* _HRT_BACKEND_HW_HIVECC_IMPL_H */
