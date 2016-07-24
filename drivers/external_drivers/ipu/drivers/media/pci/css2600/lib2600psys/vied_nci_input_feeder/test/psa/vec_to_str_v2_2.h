#ifndef _vec_to_str_v2_2_h_
#define _vec_to_str_v2_2_h_

#include <hrt/api.h>
#include "vec_to_str_v2_2_defs.h"
#include "print_support.h"

#define hrt_vec_to_str_v2_2_register_address(reg) (_VTS_ADRRESS_ALIGN_REG * (reg))

/* The functions to configure the device */
#define hrt_vec_to_str_v2_2_store(vec_to_str_v2_2_id, val) \
  hrt_vec_to_str_v2_2_set_vec_register(vec_to_str_v2_2_id, _VTS_SLVEC_PORTREG_ADDR (val))

#define hrt_vec_to_str_v2_2_set_vec_register(vec_to_str_v2_2_id, reg, val) \
  vied_subsystem_store_32(psys0, vec_to_str_v2_2_id + hrt_vec_to_str_v2_2_register_address(reg), (val))

#define hrt_vec_to_str_v2_2_set_cfg_register(vec_to_str_v2_2_id, reg, val) \
  vied_subsystem_store_32(psys0, vec_to_str_v2_2_id + hrt_vec_to_str_v2_2_register_address(reg), (val))

/* The functions to retreive the status/counters */
#define hrt_vec_to_str_v2_2_get_cfg_register(vec_to_str_v2_2_id, reg) \
  vied_subsystem_load_32(psys0, vec_to_str_v2_2_id + hrt_vec_to_str_v2_2_register_address(reg))

#define hrt_vec_to_str_v2_2_get_sl_vec_counter(vec_to_str_v2_2_id) \
  hrt_vec_to_str_v2_2_get_cfg_register(vec_to_str_v2_2_id, _VTS_VEC_COUNTER_REG)

#define hrt_vec_to_str_v2_2_get_str_out_counter(vec_to_str_v2_2_id) \
  hrt_vec_to_str_v2_2_get_cfg_register(vec_to_str_v2_2_id, _VTS_STR_COUNTER_REG)

#define hrt_vec_to_str_v2_2_get_sl_vec_port_status(vec_to_str_v2_2_id) \
  hrt_vec_to_str_v2_2_get_cfg_register(vec_to_str_v2_2_id, _VTS_VEC_STATUS_REG)

#define hrt_vec_to_str_v2_2_get_sl_cfg_port_status(vec_to_str_v2_2_id) \
  hrt_vec_to_str_v2_2_get_cfg_register(vec_to_str_v2_2_id, _VTS_STR_STATUS_REG)


#define hrt_vec_to_str_v2_2_dump_registers(vec_to_str_v2_2_id) \
{ \
  fprintf(stderr, "//-----------------------------------\n"); \
  fprintf(stderr, "// Vector to Stream (v2) status dump:\n"); \
  fprintf(stderr, "//-----------------------------------\n"); \
    unsigned int sl_vec_counter   = hrt_vec_to_str_v2_2_get_sl_vec_counter(vec_to_str_v2_2_id); \
    unsigned int str_out_counter  = hrt_vec_to_str_v2_2_get_str_out_counter(vec_to_str_v2_2_id); \
    unsigned int vec_status       = hrt_vec_to_str_v2_2_get_sl_vec_port_status(vec_to_str_v2_2_id); \
    unsigned int str_status       = hrt_vec_to_str_v2_2_get_sl_cfg_port_status(vec_to_str_v2_2_id); \
      fprintf(stderr, "+ Counter status\n"); \
      fprintf(stderr, "\t- Slave  Vector input  counter (vectors in):0x%x\n",sl_vec_counter); \
      fprintf(stderr, "\t- Stream Port   output counter (transfers out accepted):0x%x\n",str_out_counter); \
      fprintf(stderr, "+ Slave Vector input port\n"); \
    if(vec_status!=0xCAFEBABE) { \
        fprintf(stderr, "\t+ status: 0x%x\n",vec_status); \
      if((vec_status&0x80000000)==0) \
        fprintf(stderr, "\t- CS signal:LOW\n"); \
      else \
        fprintf(stderr, "\t- CS signal:HIGH\n"); \
      if((vec_status&0x40000000)==0) \
        fprintf(stderr, "\t- WE_N signal:LOW\n"); \
      else \
        fprintf(stderr, "\t- Slave WE_N signal:HIGH\n"); \
      if((vec_status&0x1FFFFFFF)==0) \
        fprintf(stderr, "\t- ADDRESS signals:zero\n"); \
      else \
        fprintf(stderr, "\t- ADDRESS signals:0x%x\n",vec_status); \
    }else{\
      fprintf(stderr, "\t- The status value of Slave Vector input:not valid (0xCAFEBABE)\n"); \
    }\
    fprintf(stderr, "+ Stream Port output\n"); \
    if(str_status!=0xCAFEBABE) { \
        fprintf(stderr, "\t+ status: 0x%x\n",str_status); \
      if((str_status&0x00000002)==0) \
        fprintf(stderr, "\t- valid signal:LOW\n"); \
      else \
        fprintf(stderr, "\t- valid signal:HIGH\n"); \
      if((str_status&0x00000001)==0) \
        fprintf(stderr, "\t- accept signal:LOW\n"); \
      else \
        fprintf(stderr, "\t- accept signal:HIGH\n"); \
    }else{\
      fprintf(stderr, "\t- The status value of Stream Port output:not valid (0xCAFEBABE)\n"); \
    }\
}

#endif /* _vec_to_str_v2_2_h_ */
