#ifndef _HRT_VAR_H
#define _HRT_VAR_H

#include "version.h"
#include "system_api.h"
#include "hive_types.h"

#define hrt_int_type_of_char   char
#define hrt_int_type_of_uchar  unsigned char
#define hrt_int_type_of_short  short
#define hrt_int_type_of_ushort unsigned short
#define hrt_int_type_of_int    int
#define hrt_int_type_of_uint   unsigned int
#define hrt_int_type_of_long   long
#define hrt_int_type_of_ulong  unsigned long
#define hrt_int_type_of_ptr    unsigned int

#define hrt_host_type_of_char   char
#define hrt_host_type_of_uchar  unsigned char
#define hrt_host_type_of_short  short
#define hrt_host_type_of_ushort unsigned short
#define hrt_host_type_of_int    int
#define hrt_host_type_of_uint   unsigned int
#define hrt_host_type_of_long   long
#define hrt_host_type_of_ulong  unsigned long
#define hrt_host_type_of_ptr    void*

#define HRT_TYPE_BYTES(cell, type) (HRT_TYPE_BITS(cell, type)/8)
#define HRT_HOST_TYPE(cell_type)   HRTCAT(hrt_host_type_of_, cell_type)
#define HRT_INT_TYPE(type)         HRTCAT(hrt_int_type_of_, type)

#ifdef C_RUN

#ifdef C_RUN_DYNAMIC_LINK_PROGRAMS
extern void *csim_processor_get_crun_symbol(hive_proc_id p, const char *sym);
#define _hrt_cell_get_crun_symbol(cell,sym)          csim_processor_get_crun_symbol(cell,HRTSTR(sym))
#define _hrt_cell_get_crun_indexed_symbol(cell,sym)  csim_processor_get_crun_symbol(cell,HRTSTR(sym))
#else
#define _hrt_cell_get_crun_symbol(cell,sym)         (&sym)
#define _hrt_cell_get_crun_indexed_symbol(cell,sym) (sym)
#endif //  C_RUN_DYNAMIC_LINK_PROGRAMS

#define hrt_scalar_store(cell, type, var, data) \
	((*(HRT_HOST_TYPE(type)*)_hrt_cell_get_crun_symbol(cell,var)) = (data))
#define hrt_scalar_load(cell, type, var) \
	((*(HRT_HOST_TYPE(type)*)_hrt_cell_get_crun_symbol(cell,var)))

#define hrt_indexed_store(cell, type, array, index, data) \
	((((HRT_HOST_TYPE(type)*)_hrt_cell_get_crun_indexed_symbol(cell,array))[index]) = (data))
#define hrt_indexed_load(cell, type, array, index) \
	(((HRT_HOST_TYPE(type)*)_hrt_cell_get_crun_indexed_symbol(cell,array))[index])

#else /* C_RUN */

#define hrt_store(address,msg,size,data)\
  (HRTCAT3(_hrt_direct_store_,size,_msg)(address, msg, data));

#define hrt_load(address,msg,size)\
  (HRTCAT3(_hrt_direct_load_,size,_msg)(address, msg))

#define hrt_get_address(cell, mem, addr)\
  (_hrt_slave_port_master_port_address(_hrt_cell_mem_slave_port(cell, mem)) + _hrt_cell_mem_master_to_slave_address(cell, mem, addr))

#define hrt_get_message(cell, mem)\
  (_hrt_not_null(_hrt_cell_mem_error_message(cell, mem)))?_hrt_cell_mem_error_message(cell, mem):(_hrt_slave_port_error_message(_hrt_cell_mem_slave_port(cell, mem)))

#define hrt_scalar_store_new(cell, type, var, data) \
  hrt_store(hrt_get_address(cell, HRTCAT(HIVE_MEM_,var),  HRTCAT(HIVE_ADDR_,var)), hrt_get_message(cell, HRTCAT(HIVE_MEM_,var)), HRT_TYPE_BITS(cell, type), (HRT_INT_TYPE(type))(data));

#define hrt_indexed_store_new(cell, type, array, index, data) \
  hrt_store(hrt_get_address(cell, HRTCAT(HIVE_MEM_,array), (HRTCAT(HIVE_ADDR_,array))+((index)*HRT_TYPE_BYTES(cell, type))), hrt_get_message(cell, HRTCAT(HIVE_MEM_,array)), HRT_TYPE_BITS(cell, type), (HRT_INT_TYPE(type))(data));

#define hrt_scalar_load_new(cell,type,var) \
  (HRT_HOST_TYPE(type))hrt_load(hrt_get_address(cell, HRTCAT(HIVE_MEM_,var),  HRTCAT(HIVE_ADDR_,var)), hrt_get_message(cell, HRTCAT(HIVE_MEM_,var)), HRT_TYPE_BITS(cell, type))

#define hrt_indexed_load_new(cell, type, array, index) \
  (HRT_HOST_TYPE(type))hrt_load(hrt_get_address(cell, HRTCAT(HIVE_MEM_,array), (HRTCAT(HIVE_ADDR_,array))+((index)*HRT_TYPE_BYTES(cell, type))), hrt_get_message(cell, HRTCAT(HIVE_MEM_,array)), HRT_TYPE_BITS(cell, type))

#define hrt_scalar_store(cell, type, var, data) \
  HRTCAT(hrt_mem_store_,HRT_TYPE_BITS(cell, type))(\
	       cell, \
	       HRTCAT(HIVE_MEM_,var), \
	       HRTCAT(HIVE_ADDR_,var), \
	       (HRT_INT_TYPE(type))(data))

#define hrt_scalar_load(cell, type, var) \
  (HRT_HOST_TYPE(type))(HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,type)( \
	       cell, \
	       HRTCAT(HIVE_MEM_,var), \
	       HRTCAT(HIVE_ADDR_,var)))

#define hrt_indexed_store(cell, type, array, index, data) \
  HRTCAT(hrt_mem_store_,HRT_TYPE_BITS(cell, type))(\
	       cell, \
	       HRTCAT(HIVE_MEM_,array), \
	       (HRTCAT(HIVE_ADDR_,array))+((index)*HRT_TYPE_BYTES(cell, type)), \
	       (HRT_INT_TYPE(type))(data))

#define hrt_indexed_load(cell, type, array, index) \
  (HRT_HOST_TYPE(type))(HRTCAT4(_hrt_mem_load_,HRT_PROC_TYPE(cell),_,type) ( \
         cell, \
	       HRTCAT(HIVE_MEM_,array), \
	       (HRTCAT(HIVE_ADDR_,array))+((index)*HRT_TYPE_BYTES(cell, type))))

#endif /* C_RUN */

#endif /* _HRT_VAR_H */
