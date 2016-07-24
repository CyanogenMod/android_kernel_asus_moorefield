#ifndef _HRT_MASTER_PORT_HW_H 
#define _HRT_MASTER_PORT_HW_H 

#include <string.h>

#include "hive_types.h"
#include "system_api.h"

#ifdef __HIVECC
/* If we run HRT on a hive processor, we need to make sure
   that the master port transactions go to the master interface,
   not to the local memory. */
#include <hive/support.h>
#ifdef HRT_MT_INT
#define _HRT_MEM MEM(HRT_MT_INT)
#endif
#endif /* __HIVECC */

#ifndef _HRT_MEM
#define _HRT_MEM
#endif

// typedef hive_int8   _HRT_MEM *hive_int8_ptr;
// typedef hive_uint8  _HRT_MEM *hive_uint8_ptr;
// typedef hive_int16  _HRT_MEM *hive_int16_ptr;
// typedef hive_uint16 _HRT_MEM *hive_uint16_ptr;
// typedef hive_int32  _HRT_MEM *hive_int32_ptr;
// typedef hive_uint32 _HRT_MEM *hive_uint32_ptr;
// typedef volatile hive_int8  _HRT_MEM *hive_int8_v_ptr;
// typedef volatile hive_uint8 _HRT_MEM *hive_uint8_v_ptr;
// typedef volatile hive_int16  _HRT_MEM *hive_int16_v_ptr;
// typedef volatile hive_uint16 _HRT_MEM *hive_uint16_v_ptr;
// typedef volatile hive_int32  _HRT_MEM *hive_int32_v_ptr;
// typedef volatile hive_uint32 _HRT_MEM *hive_uint32_v_ptr;
// typedef char       _HRT_MEM *hive_char_ptr;
// typedef const char _HRT_MEM *hive_const_char_ptr;

#ifndef HRT_USE_VIR_ADDRS

#define _hrt_master_port_store_8(address,data)  (*(hive_int8_v_ptr)(address) = (data))
#define _hrt_master_port_store_16(address,data) (*(hive_int16_v_ptr)(address) = (data))
#define _hrt_master_port_store_32(address,data) (*(hive_int32_v_ptr)(address) = (data))
#define _hrt_master_port_load_8(address)        (*(hive_int8_v_ptr)(address))
#define _hrt_master_port_load_16(address)       (*(hive_int16_v_ptr)(address))
#define _hrt_master_port_load_32(address)       (*(hive_int32_v_ptr)(address))

#endif

#define _hrt_master_port_store_8_msg(a,d,m)           _hrt_master_port_store_8(a,d)
#define _hrt_master_port_store_16_msg(a,d,m)          _hrt_master_port_store_16(a,d)
#define _hrt_master_port_store_32_msg(a,d,m)          _hrt_master_port_store_32(a,d)
#define _hrt_master_port_load_8_msg(a,m)              _hrt_master_port_load_8(a)
#define _hrt_master_port_load_16_msg(a,m)             _hrt_master_port_load_16(a)
#define _hrt_master_port_load_32_msg(a,m)             _hrt_master_port_load_32(a)
#define _hrt_master_port_unaligned_store_msg(a,d,s,m) _hrt_master_port_unaligned_store(a,d,s)
#define _hrt_master_port_unaligned_load_msg(a,d,s,m)  _hrt_master_port_unaligned_load(a,d,s)
#define _hrt_master_port_unaligned_set_msg(a,d,s,m)   _hrt_master_port_unaligned_set(a,d,s)



#endif /* _HRT_MASTER_PORT_HW_H */
