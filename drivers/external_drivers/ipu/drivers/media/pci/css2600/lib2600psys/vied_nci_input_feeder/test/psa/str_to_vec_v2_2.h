#ifndef _str_to_vec_v2_2_h_
#define _str_to_vec_v2_2_h_

#include <hrt/api.h>
#include "str_to_vec_v2_2_defs.h"

#define hrt_str_to_vec_v2_2_register_address(reg) (_STR_TO_VEC_V2_2_REG_ALIGN * (reg))

#define hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, reg) \
  vied_subsystem_load_32(psys0, str_to_vec_v2_2_id + hrt_str_to_vec_v2_2_register_address(reg))

#define hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, reg, val) \
  vied_subsystem_store_32(psys0, str_to_vec_v2_2_id + hrt_str_to_vec_v2_2_register_address(reg), (val) )

//#define hrt_str_to_vec_v2_2_get_register2(str_to_vec_v2_2_id, reg) \
//  _hrt_slave_port_load_32_volatile(hrt_str_to_vec_v2_2_slave_port(str_to_vec_v2_2_id), hrt_str_to_vec_v2_2_register_address(reg))
//#define HRT_GA_ACB_get_register(ACB_id, addr) \
//  _hrt_slave_port_load_32_volatile(HRT_GA_ACB_crq_port(ACB_id), addr)


#define hrt_str_to_vec_v2_2_snd_cmd(str_to_vec_v2_2_id, cmd) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_CMD, cmd)

#define hrt_str_to_vec_v2_2_set_ack_k_vec(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_ACK_K_VEC_REG, val)

#define hrt_str_to_vec_v2_2_set_pxl_line(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_PXL_LINE_REG, val)

#define hrt_str_to_vec_v2_2_set_line_frame(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_LINE_FRAME_REG, val)

#define hrt_str_to_vec_v2_2_set_yuv420_en(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_YUV420_EN_REG, val)

#define hrt_str_to_vec_v2_2_set_interleave_en(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_INTERLEAVE_EN_REG, val)

#define hrt_str_to_vec_v2_2_set_dev_null_en(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_DEV_NULL_EN_REG, val)

#ifndef C_RUN
  #define hrt_str_to_vec_v2_2_set_crun(str_to_vec_v2_2_id, var)
#else
  #define hrt_str_to_vec_v2_2_set_crun(str_to_vec_v2_2_id, var) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_CRUN_REG, var)
#endif

//COMMANDS:
#define hrt_str_to_vec_v2_2_initialize(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_set_crun(str_to_vec_v2_2_id, 1); \
  hrt_str_to_vec_v2_2_snd_cmd(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_CMD_INIT & 0x01F)

#define hrt_str_to_vec_v2_2_return_n_vecs(str_to_vec_v2_2_id, n_vecs) \
  hrt_str_to_vec_v2_2_snd_cmd(str_to_vec_v2_2_id, ((n_vecs << 16) & 0xFFFF0000) | (_STR_TO_VEC_V2_2_CMD_PROC_N_VEC & 0x01F))


//GENERIC BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_set_0_st_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_ST_ADDR, val)

#define hrt_str_to_vec_v2_2_set_1_st_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_ST_ADDR, val)

#define hrt_str_to_vec_v2_2_set_2_st_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_ST_ADDR, val)

#define hrt_str_to_vec_v2_2_set_3_st_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_ST_ADDR, val)

#define hrt_str_to_vec_v2_2_set_4_st_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_ST_ADDR, val)

#define hrt_str_to_vec_v2_2_set_5_st_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_ST_ADDR, val)

#define hrt_str_to_vec_v2_2_set_0_end_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_END_ADDR, val)

#define hrt_str_to_vec_v2_2_set_1_end_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_END_ADDR, val)

#define hrt_str_to_vec_v2_2_set_2_end_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_END_ADDR, val)

#define hrt_str_to_vec_v2_2_set_3_end_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_END_ADDR, val)

#define hrt_str_to_vec_v2_2_set_4_end_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_END_ADDR, val)

#define hrt_str_to_vec_v2_2_set_5_end_addr(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_END_ADDR, val)

#define hrt_str_to_vec_v2_2_set_0_offset(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_OFFSET, val)

#define hrt_str_to_vec_v2_2_set_1_offset(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_OFFSET, val)

#define hrt_str_to_vec_v2_2_set_2_offset(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_OFFSET, val)

#define hrt_str_to_vec_v2_2_set_3_offset(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_OFFSET, val)

#define hrt_str_to_vec_v2_2_set_4_offset(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_OFFSET, val)

#define hrt_str_to_vec_v2_2_set_5_offset(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_OFFSET, val)

#define hrt_str_to_vec_v2_2_set_0_stride(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_STRIDE, val)

#define hrt_str_to_vec_v2_2_set_1_stride(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_STRIDE, val)

#define hrt_str_to_vec_v2_2_set_2_stride(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_STRIDE, val)

#define hrt_str_to_vec_v2_2_set_3_stride(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_STRIDE, val)

#define hrt_str_to_vec_v2_2_set_4_stride(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_STRIDE, val)

#define hrt_str_to_vec_v2_2_set_5_stride(str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_STRIDE, val)


//Setting address buffers:
#define hrt_str_to_vec_v2_2_ceildiv(n,m)    (((n)+(m)-1)/(m))
static inline unsigned hrt_str_to_vec_next_power_of_two(unsigned bits)
{
  unsigned int pow2_val, type_val;
  type_val=0;
  for (pow2_val = 1; pow2_val < hrt_str_to_vec_v2_2_ceildiv(bits, 8); pow2_val <<= 1)
  {
    type_val++;
  }
  return type_val;
}

#define hrt_str_to_vec_v2_2_type(vec_bits) hrt_str_to_vec_next_power_of_two(vec_bits)

//#define hrt_str_to_vec_v2_2_type(vec_bits)  ((unsigned int)(ceil(log((double)(hrt_str_to_vec_v2_2_ceildiv(vec_bits,8)))/log(2))))

#ifdef C_RUN
  #define hrt_str_to_vec_v2_2_align(vec_bits) \
    offset_val = hrt_str_to_vec_v2_2_ceildiv(vec_bits,8)
  #define hrt_str_to_vec_v2_2_start_addr(start_addr, vec_bits, type_val) start_addr
#else
  #define hrt_str_to_vec_v2_2_align(vec_bits) offset_val = 1
  #define hrt_str_to_vec_v2_2_start_addr(start_addr, vec_bits, type_val) \
    ((start_addr) >> type_val)
#endif

#define hrt_str_to_vec_v2_2_set_buffer(WhichBuffer, str_to_vec_v2_2_id, start_addr, vecs_p_line, nr_lines, vec_bits) \
{ \
  unsigned int offset_val, stride_val, start_addr_val, type_val; \
  type_val=hrt_str_to_vec_v2_2_type(vec_bits); \
  hrt_str_to_vec_v2_2_align(vec_bits); \
  stride_val = (vecs_p_line) * (offset_val); \
  start_addr_val = hrt_str_to_vec_v2_2_start_addr(start_addr, vec_bits, type_val); \
  hrt_str_to_vec_v2_2_set_##WhichBuffer(st_addr, str_to_vec_v2_2_id, start_addr_val); \
  hrt_str_to_vec_v2_2_set_##WhichBuffer(offset, str_to_vec_v2_2_id, offset_val); \
  hrt_str_to_vec_v2_2_set_##WhichBuffer(stride, str_to_vec_v2_2_id, stride_val); \
  hrt_str_to_vec_v2_2_set_##WhichBuffer(end_addr, str_to_vec_v2_2_id, start_addr_val + (nr_lines) * stride_val); \
}


//BAYER BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_set_bayer_gr(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_0_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_bayer_r(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_1_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_bayer_b(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_2_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_bayer_gb(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_3_##WhatToSet(str_to_vec_v2_2_id, val)


//BAYER_4PPC BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_set_bayer_4ppc_gr(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_0_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_bayer_4ppc_r(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_1_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_bayer_4ppc_b(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_2_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_bayer_4ppc_gb(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_3_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_bayer_4ppc_grr(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_0_##WhatToSet(str_to_vec_v2_2_id, val)
#define hrt_str_to_vec_v2_2_set_bayer_4ppc_bgb(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_1_##WhatToSet(str_to_vec_v2_2_id, val)



//YUV420 BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_set_yuv420_y0(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_0_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv420_y1(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_1_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv420_u(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_2_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv420_v(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_3_##WhatToSet(str_to_vec_v2_2_id, val)



//YUV422 BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_set_yuv422_y0(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_0_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv422_y1(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_1_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv422_u0(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_2_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv422_v0(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_3_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv422_u1(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_4_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_yuv422_v1(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_5_##WhatToSet(str_to_vec_v2_2_id, val)


//RGB BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_set_rgb_r0(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_0_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_rgb_g0(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_1_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_rgb_b0(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_2_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_rgb_r1(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_0_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_rgb_g1(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_1_##WhatToSet(str_to_vec_v2_2_id, val)

#define hrt_str_to_vec_v2_2_set_rgb_b1(WhatToSet, str_to_vec_v2_2_id, val) \
  hrt_str_to_vec_v2_2_set_2_##WhatToSet(str_to_vec_v2_2_id, val)




#define hrt_str_to_vec_v2_2_get_false_command(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_IRQ_FALSE_CMD_REG)

#define hrt_str_to_vec_v2_2_get_ack_k_vec(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_ACK_K_VEC_REG)

#define hrt_str_to_vec_v2_2_get_pxl_line(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_PXL_LINE_REG)

#define hrt_str_to_vec_v2_2_get_line_frame(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_LINE_FRAME_REG)

#define hrt_str_to_vec_v2_2_get_yuv420_en(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_YUV420_EN_REG)

#define hrt_str_to_vec_v2_2_get_interleave_en(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_INTERLEAVE_EN_REG)

#define hrt_str_to_vec_v2_2_get_dev_null_en(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_DEV_NULL_EN_REG)

#define hrt_str_to_vec_v2_2_get_pxl_cur_line(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_PXL_CUR_LINE_REG);

#define hrt_str_to_vec_v2_2_get_lines_done(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_LINES_DONE_REG);

#define hrt_str_to_vec_v2_2_get_io_status(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_IO_STATUS_REG)

#define hrt_str_to_vec_v2_2_get_ack_status(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_ACK_STATUS_REG)

#define hrt_str_to_vec_v2_2_get_main_status(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_MAIN_STATUS_REG)

#define hrt_str_to_vec_v2_2_get_tracker_status(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_TRACK_STATUS_REG)


//GET GENERIC BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_get_0_st_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_ST_ADDR)

#define hrt_str_to_vec_v2_2_get_1_st_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_ST_ADDR)

#define hrt_str_to_vec_v2_2_get_2_st_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_ST_ADDR)

#define hrt_str_to_vec_v2_2_get_3_st_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_ST_ADDR)

#define hrt_str_to_vec_v2_2_get_4_st_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_ST_ADDR)

#define hrt_str_to_vec_v2_2_get_5_st_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_ST_ADDR)

#define hrt_str_to_vec_v2_2_get_0_end_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_END_ADDR)

#define hrt_str_to_vec_v2_2_get_1_end_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_END_ADDR)

#define hrt_str_to_vec_v2_2_get_2_end_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_END_ADDR)

#define hrt_str_to_vec_v2_2_get_3_end_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_END_ADDR)

#define hrt_str_to_vec_v2_2_get_4_end_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_END_ADDR)

#define hrt_str_to_vec_v2_2_get_5_end_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_END_ADDR)

#define hrt_str_to_vec_v2_2_get_0_offset(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_OFFSET)

#define hrt_str_to_vec_v2_2_get_1_offset(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_OFFSET)

#define hrt_str_to_vec_v2_2_get_2_offset(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_OFFSET)

#define hrt_str_to_vec_v2_2_get_3_offset(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_OFFSET)

#define hrt_str_to_vec_v2_2_get_4_offset(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_OFFSET)

#define hrt_str_to_vec_v2_2_get_5_offset(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_OFFSET)

#define hrt_str_to_vec_v2_2_get_0_stride(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_STRIDE)

#define hrt_str_to_vec_v2_2_get_1_stride(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_STRIDE)

#define hrt_str_to_vec_v2_2_get_2_stride(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_STRIDE)

#define hrt_str_to_vec_v2_2_get_3_stride(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_STRIDE)

#define hrt_str_to_vec_v2_2_get_4_stride(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_STRIDE)

#define hrt_str_to_vec_v2_2_get_5_stride(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_STRIDE)

#define hrt_str_to_vec_v2_2_get_0_cur_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_0_CUR_ADDR)

#define hrt_str_to_vec_v2_2_get_1_cur_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_1_CUR_ADDR)

#define hrt_str_to_vec_v2_2_get_2_cur_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_2_CUR_ADDR)

#define hrt_str_to_vec_v2_2_get_3_cur_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_3_CUR_ADDR)

#define hrt_str_to_vec_v2_2_get_4_cur_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_4_CUR_ADDR)

#define hrt_str_to_vec_v2_2_get_5_cur_addr(str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_register(str_to_vec_v2_2_id, _STR_TO_VEC_V2_2_5_CUR_ADDR)



//GET BAYER BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_get_bayer_gr(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_0_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_bayer_r(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_1_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_bayer_b(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_2_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_bayer_gb(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_3_##WhatToGet(str_to_vec_v2_2_id)


//GET BAYER_4PPC BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_get_bayer_4ppc_gr(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_0_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_bayer_4ppc_r(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_1_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_bayer_4ppc_b(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_2_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_bayer_4ppc_gb(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_3_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_bayer_4ppc_grr(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_0_##WhatToGet(str_to_vec_v2_2_id)
#define hrt_str_to_vec_v2_2_get_bayer_4ppc_bgb(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_1_##WhatToGet(str_to_vec_v2_2_id)


//GET YUV420 BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_get_yuv420_y0(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_0_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv420_y1(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_1_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv420_u(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_2_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv420_v(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_3_##WhatToGet(str_to_vec_v2_2_id)


//GET YUV422 BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_get_yuv422_y0(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_0_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv422_y1(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_1_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv422_u0(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_2_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv422_v0(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_3_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv422_u1(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_4_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_yuv422_v1(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_5_##WhatToGet(str_to_vec_v2_2_id)


//GET RGB BUFFER CONFIG:
#define hrt_str_to_vec_v2_2_get_rgb_r0(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_0_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_rgb_g0(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_1_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_rgb_b0(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_2_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_rgb_r1(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_0_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_rgb_g1(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_1_##WhatToGet(str_to_vec_v2_2_id)

#define hrt_str_to_vec_v2_2_get_rgb_b1(WhatToGet, str_to_vec_v2_2_id) \
  hrt_str_to_vec_v2_2_get_2_##WhatToGet(str_to_vec_v2_2_id)




//DUMP FUNCTION:
#define hrt_str_to_vec_v2_2_dump_registers(str_to_vec_v2_2_id) \
{ \
  unsigned int ack_k_vec      = hrt_str_to_vec_v2_2_get_ack_k_vec(str_to_vec_v2_2_id); \
  unsigned int pxl_line       = hrt_str_to_vec_v2_2_get_pxl_line(str_to_vec_v2_2_id); \
  unsigned int line_frame     = hrt_str_to_vec_v2_2_get_line_frame(str_to_vec_v2_2_id); \
  unsigned int yuv420_en      = hrt_str_to_vec_v2_2_get_yuv420_en(str_to_vec_v2_2_id); \
  unsigned int interleave_en  = hrt_str_to_vec_v2_2_get_interleave_en(str_to_vec_v2_2_id); \
  unsigned int dev_null_en    = hrt_str_to_vec_v2_2_get_dev_null_en(str_to_vec_v2_2_id); \
  \
  unsigned int buf0_st_addr   = hrt_str_to_vec_v2_2_get_0_st_addr(str_to_vec_v2_2_id); \
  unsigned int buf0_end_addr  = hrt_str_to_vec_v2_2_get_0_end_addr(str_to_vec_v2_2_id); \
  unsigned int buf0_offset    = hrt_str_to_vec_v2_2_get_0_offset(str_to_vec_v2_2_id); \
  unsigned int buf0_stride    = hrt_str_to_vec_v2_2_get_0_stride(str_to_vec_v2_2_id); \
  unsigned int buf0_cur_addr  = hrt_str_to_vec_v2_2_get_0_cur_addr(str_to_vec_v2_2_id); \
  \
  unsigned int buf1_st_addr   = hrt_str_to_vec_v2_2_get_1_st_addr(str_to_vec_v2_2_id); \
  unsigned int buf1_end_addr  = hrt_str_to_vec_v2_2_get_1_end_addr(str_to_vec_v2_2_id); \
  unsigned int buf1_offset    = hrt_str_to_vec_v2_2_get_1_offset(str_to_vec_v2_2_id); \
  unsigned int buf1_stride    = hrt_str_to_vec_v2_2_get_1_stride(str_to_vec_v2_2_id); \
  unsigned int buf1_cur_addr  = hrt_str_to_vec_v2_2_get_1_cur_addr(str_to_vec_v2_2_id); \
  \
  unsigned int buf2_st_addr   = hrt_str_to_vec_v2_2_get_2_st_addr(str_to_vec_v2_2_id); \
  unsigned int buf2_end_addr  = hrt_str_to_vec_v2_2_get_2_end_addr(str_to_vec_v2_2_id); \
  unsigned int buf2_offset    = hrt_str_to_vec_v2_2_get_2_offset(str_to_vec_v2_2_id); \
  unsigned int buf2_stride    = hrt_str_to_vec_v2_2_get_2_stride(str_to_vec_v2_2_id); \
  unsigned int buf2_cur_addr  = hrt_str_to_vec_v2_2_get_2_cur_addr(str_to_vec_v2_2_id); \
  \
  unsigned int buf3_st_addr   = hrt_str_to_vec_v2_2_get_3_st_addr(str_to_vec_v2_2_id); \
  unsigned int buf3_end_addr  = hrt_str_to_vec_v2_2_get_3_end_addr(str_to_vec_v2_2_id); \
  unsigned int buf3_offset    = hrt_str_to_vec_v2_2_get_3_offset(str_to_vec_v2_2_id); \
  unsigned int buf3_stride    = hrt_str_to_vec_v2_2_get_3_stride(str_to_vec_v2_2_id); \
  unsigned int buf3_cur_addr  = hrt_str_to_vec_v2_2_get_3_cur_addr(str_to_vec_v2_2_id); \
  \
  unsigned int buf4_st_addr   = hrt_str_to_vec_v2_2_get_4_st_addr(str_to_vec_v2_2_id); \
  unsigned int buf4_end_addr  = hrt_str_to_vec_v2_2_get_4_end_addr(str_to_vec_v2_2_id); \
  unsigned int buf4_offset    = hrt_str_to_vec_v2_2_get_4_offset(str_to_vec_v2_2_id); \
  unsigned int buf4_stride    = hrt_str_to_vec_v2_2_get_4_stride(str_to_vec_v2_2_id); \
  unsigned int buf4_cur_addr  = hrt_str_to_vec_v2_2_get_4_cur_addr(str_to_vec_v2_2_id); \
  \
  unsigned int buf5_st_addr   = hrt_str_to_vec_v2_2_get_5_st_addr(str_to_vec_v2_2_id); \
  unsigned int buf5_end_addr  = hrt_str_to_vec_v2_2_get_5_end_addr(str_to_vec_v2_2_id); \
  unsigned int buf5_offset    = hrt_str_to_vec_v2_2_get_5_offset(str_to_vec_v2_2_id); \
  unsigned int buf5_stride    = hrt_str_to_vec_v2_2_get_5_stride(str_to_vec_v2_2_id); \
  unsigned int buf5_cur_addr  = hrt_str_to_vec_v2_2_get_5_cur_addr(str_to_vec_v2_2_id); \
\
  unsigned int pxl_cur_line   = hrt_str_to_vec_v2_2_get_pxl_cur_line(str_to_vec_v2_2_id); \
  unsigned int lines_done     = hrt_str_to_vec_v2_2_get_lines_done(str_to_vec_v2_2_id); \
  unsigned int io_status      = hrt_str_to_vec_v2_2_get_io_status(str_to_vec_v2_2_id); \
  unsigned int ack_status     = hrt_str_to_vec_v2_2_get_ack_status(str_to_vec_v2_2_id); \
  unsigned int main_status    = hrt_str_to_vec_v2_2_get_main_status(str_to_vec_v2_2_id); \
  unsigned int tracker_status = hrt_str_to_vec_v2_2_get_tracker_status(str_to_vec_v2_2_id); \
\
  fprintf(stderr, "Stream To Vec V2 status dump\n"); \
\
  {\
    fprintf(stderr, " - The device is set to 0x%X pixel components per line\n", pxl_line); \
    fprintf(stderr, " - The device is set to 0x%X lines per frame\n", line_frame); \
    fprintf(stderr, " - The device will output an acknowledge after 0x%X vectors\n",ack_k_vec); \
    if(yuv420_en!=0xDEADBEEF)\
    { \
      if(yuv420_en==1)\
        fprintf(stderr, " - YUV420 is enabled\n"); \
      else\
        fprintf(stderr, " - YUV420 is not enabled\n"); \
    }\
    if(interleave_en!=0xDEADBEEF)\
    { \
      if(interleave_en==1)\
        fprintf(stderr, " - Interleaving is enabled\n"); \
      else\
        fprintf(stderr, " - Interleaving is not enabled\n"); \
    }\
    if(dev_null_en)\
      fprintf(stderr, " - Incoming pixel components are accepted and discarded after a frame is done or reset\n");\
    else \
      fprintf(stderr, " - Incoming pixel components are NOT accepted after a frame is done or reset\n");\
    fprintf(stderr, "\n - The device counted 0x%X pixel components streamed in so far for the current line\n",pxl_cur_line); \
    fprintf(stderr, " - The device finished processing 0x%X lines for the current frame\n",lines_done); \
    fprintf(stderr, "\n - Buffer 0; Bayer GR, BAYER_4PPC GR (not interleaved), BAYER_4PPC R-Gr-R-Gr (interleaved), or YUV Y0:\n"); \
    fprintf(stderr, "   + Configured as follows:\n"); \
    fprintf(stderr, "     o Start address:  0x%X\n", buf0_st_addr); \
    fprintf(stderr, "     o End address:    0x%X\n", buf0_end_addr); \
    fprintf(stderr, "     o Offset:         0x%X\n", buf0_offset); \
    fprintf(stderr, "     o Stride:         0x%X\n", buf0_stride); \
    fprintf(stderr, "   + Current status:\n"); \
    fprintf(stderr, "     o Current address (next write will go to this address): 0x%X\n", buf0_cur_addr); \
\
    fprintf(stderr, " - Buffer 1; Bayer R, BAYER_4PPC R (not interleaved), BAYER_4PPC Gb-B-Gb-B (interleaved), or YUV Y1:\n"); \
    fprintf(stderr, "   + Configured as follows:\n"); \
    fprintf(stderr, "     o Start address:  0x%X\n", buf1_st_addr); \
    fprintf(stderr, "     o End address:    0x%X\n", buf1_end_addr); \
    fprintf(stderr, "     o Offset:         0x%X\n", buf1_offset); \
    fprintf(stderr, "     o Stride:         0x%X\n", buf1_stride); \
    fprintf(stderr, "   + Current status:\n"); \
    fprintf(stderr, "     o Current address (next write will go to this address): 0x%X\n", buf1_cur_addr); \
\
    fprintf(stderr, " - Buffer 2; Bayer B, BAYER_4PPC B (not interleaved), or YUV U0:\n"); \
    fprintf(stderr, "   + Configured as follows:\n"); \
    fprintf(stderr, "     o Start address:  0x%X\n", buf2_st_addr); \
    fprintf(stderr, "     o End address:    0x%X\n", buf2_end_addr); \
    fprintf(stderr, "     o Offset:         0x%X\n", buf2_offset); \
    fprintf(stderr, "     o Stride:         0x%X\n", buf2_stride); \
    fprintf(stderr, "   + Current status:\n"); \
    fprintf(stderr, "     o Current address (next write will go to this address): 0x%X\n", buf2_cur_addr); \
\
    fprintf(stderr, " - Buffer 3; Bayer B, BAYER_4PPC Gb (not interleaved), or YUV V0:\n"); \
    fprintf(stderr, "   + Configured as follows:\n"); \
    fprintf(stderr, "     o Start address:  0x%X\n", buf3_st_addr); \
    fprintf(stderr, "     o End address:    0x%X\n", buf3_end_addr); \
    fprintf(stderr, "     o Offset:         0x%X\n", buf3_offset); \
    fprintf(stderr, "     o Stride:         0x%X\n", buf3_stride); \
    fprintf(stderr, "   + Current status:\n"); \
    fprintf(stderr, "     o Current address (next write will go to this address): 0x%X\n", buf3_cur_addr); \
\
    if(buf4_st_addr!=0xDEADBEEF) \
    { \
      fprintf(stderr, " - Buffer 4; BAYER_4PPC G1 or YUV422 U1:\n"); \
      fprintf(stderr, "   + Configured as follows:\n"); \
      fprintf(stderr, "     o Start address:  0x%X\n", buf4_st_addr); \
      fprintf(stderr, "     o End address:    0x%X\n", buf4_end_addr); \
      fprintf(stderr, "     o Offset:         0x%X\n", buf4_offset); \
      fprintf(stderr, "     o Stride:         0x%X\n", buf4_stride); \
      fprintf(stderr, "   + Current status:\n"); \
      fprintf(stderr, "     o Current address (next write will go to this address): 0x%X\n", buf4_cur_addr); \
    }\
    if(buf5_st_addr!=0xDEADBEEF) \
    { \
      fprintf(stderr, " - Buffer 5; BAYER_4PPC B1 or YUV422 V1:\n"); \
      fprintf(stderr, "   + Configured as follows:\n"); \
      fprintf(stderr, "     o Start address:  0x%X\n", buf5_st_addr); \
      fprintf(stderr, "     o End address:    0x%X\n", buf5_end_addr); \
      fprintf(stderr, "     o Offset:         0x%X\n", buf5_offset); \
      fprintf(stderr, "     o Stride:         0x%X\n", buf5_stride); \
      fprintf(stderr, "   + Current status:\n"); \
      fprintf(stderr, "     o Current address (next write will go to this address): 0x%X\n", buf5_cur_addr); \
    }\
\
    if(io_status==0xCAFEBABE) { \
      fprintf(stderr, "\n - The input output status can not be given with the HSS model (0x%X)\n",io_status); \
    } else { \
      fprintf(stderr, "\n - The status of the output acknowledge port:\n"); \
      fprintf(stderr, "   + VALID is "); \
      if(io_status&(0x1<<0)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "   + ACCEPT is "); \
      if(io_status&(0x1<<1)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "\n - The status of the input pixel component port:\n"); \
      fprintf(stderr, "   + VALID is "); \
      if(io_status&(0x1<<2)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "   + ACCEPT is "); \
      if(io_status&(0x1<<3)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, " - The status of CIO Master port:\n"); \
      fprintf(stderr, "   + CS is "); \
      if(io_status&(0x1<<4)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "   + RUN is "); \
      if(io_status&(0x1<<5)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "   + WE_N is "); \
      if(io_status&(0x1<<6)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "   + The address from 'up_bit' down to 'low_bit' is: 0x%X \n     o Low_bit: ceil(log2(bits_to_bytes(master port width)))\n     o up_bit: low_bit+20\n",(io_status&(0x1FFFFFF<<7))>>7); \
    }\
\
    if(ack_status==0xCAFEBABE) { \
      fprintf(stderr, "\n - The status of the acknowledge controller can not be given with the HSS model (0x%X)\n",ack_status); \
    } else { \
      fprintf(stderr, "\n - The state of the acknowledge controller FSM is: "); \
      switch(ack_status&(0x3<<0)) { \
        case(0): fprintf(stderr, "0, in_int_waccept is zero\n"); break;\
        case(1): fprintf(stderr, "1, in_int_waccept is one, but no ack to send\n"); break;\
        case(2): fprintf(stderr, "2, sending internal ack\n"); break;\
        case(3): fprintf(stderr, "3, returning command ack\n"); break;\
        default: fprintf(stderr, "Unkown state!!!!!"); break;\
      }\
      fprintf(stderr, " - The status of the return ack (to the ack cntrl):\n"); \
      fprintf(stderr, "   + VALID is "); \
      if(ack_status&(0x1<<2)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "   + ACCEPT is "); \
      if(ack_status&(0x1<<3)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, " - The status of the internal generated ack:\n"); \
      fprintf(stderr, "   + VALID is "); \
      if(ack_status&(0x1<<4)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, "   + ACCEPT is "); \
      if(ack_status&(0x1<<5)) \
        fprintf(stderr, "HIGH\n"); \
      else \
        fprintf(stderr, "LOW\n"); \
      fprintf(stderr, " - The acknowledgement controller is "); \
      if(!(ack_status&(0x1<<6))) \
        fprintf(stderr, "NOT "); \
      fprintf(stderr, "stalling the sending of vectors\n"); \
    }\
\
    if(main_status==0xCAFEBABE) { \
      fprintf(stderr, "\n - The status of the main controller and main signals can not be given with the HSS model (0x%X)\n",main_status); \
    } else { \
      fprintf(stderr, "\n - The state of the main controller FSM is: "); \
      if(!(main_status&(0x1<<0))) { \
        fprintf(stderr, "NORMAL\n");\
      } else { \
        fprintf(stderr, "WAIT_ON_RUN\n     (CS is high, but RUN is low)\n");\
      }\
      fprintf(stderr, " - The vector currently selected: %d \n",(main_status&(0xF<<1))>>1); \
      fprintf(stderr, " - The vector data currently selected: %d \n",(main_status&(0xF<<5))>>5); \
      fprintf(stderr, " - The vector address currently selected: %d \n",(main_status&(0x7<<9))>>9); \
      fprintf(stderr, " - The initialized signal is "); \
      if(main_status&(0x1<<12)) \
        fprintf(stderr, "HIGH (the device is initialized)\n"); \
      else \
        fprintf(stderr, "LOW (the device is not initialized; after reset or a frame is done)\n"); \
      fprintf(stderr, " - Give n vector counter is "); \
      if(main_status&(0x1<<13)) \
        fprintf(stderr, "ZERO\n"); \
      else \
        fprintf(stderr, "NOT ZERO\n"); \
      fprintf(stderr, "\n - The state of the command controller FSM is: "); \
      switch((main_status&(0x7<<14))>>14) { \
        case(0): fprintf(stderr, "IDLE\n     (Wait for something to do)\n"); break;\
        case(1): fprintf(stderr, "INIT\n     (Initialize the device)\n"); break;\
        case(2): fprintf(stderr, "RET INIT\n     (Return an initialization ack)\n"); break;\
        case(3): fprintf(stderr, "RET CMD\n     (Return a false command ack)\n"); break;\
        case(4): fprintf(stderr, "RET 0VEC\n     (Return a zero vector ack because device is not in initialized state)\n"); break;\
        case(5): fprintf(stderr, "GIVE_N_VEC\n     (Give N vector command received)\n"); break;\
        case(6): fprintf(stderr, "WAIT_VEC\n     (Wait until all N vectors are processed)\n"); break;\
        default: fprintf(stderr, "Unkown state!!!!!"); break; \
      }\
    }\
\
    if(tracker_status==0xCAFEBABE) { \
      fprintf(stderr, "\n - The status of the tracker can not be given with the HSS model (0x%X)\n",tracker_status); \
    } else { \
      fprintf(stderr, "\n - The hold receiving signal from the tracker is "); \
      if(tracker_status&(0x1<<0)) { \
        fprintf(stderr, "HIGH (no pixel components should be received)\n");\
      } else { \
        fprintf(stderr, "LOW (new pixel components can be received)\n");\
      }\
      fprintf(stderr, " - The '1st half of vector is full' signal from the tracker is "); \
      if(tracker_status&(0x1<<1)) { \
        fprintf(stderr, "HIGH\n");\
      } else { \
        fprintf(stderr, "LOW\n");\
      }\
      fprintf(stderr, " - The '2nd half of vector is full' signal from the tracker is "); \
      if(tracker_status&(0x1<<2)) { \
        fprintf(stderr, "HIGH\n");\
      } else { \
        fprintf(stderr, "LOW\n");\
      }\
      fprintf(stderr, " - The frame is "); \
      if(tracker_status&(0x1<<3)) { \
        fprintf(stderr, "DONE\n");\
      } else { \
        fprintf(stderr, "NOT yet DONE\n");\
      }\
      fprintf(stderr, " - The give n vectors counter is "); \
      if(tracker_status&(0x1<<4)) { \
        fprintf(stderr, "ZERO\n");\
      } else { \
        fprintf(stderr, "NOT ZERO\n");\
      }\
      fprintf(stderr, " - The vectors that are available and ready for sending:\n   + Vector: "); \
      for(i=0;i<12;i++) {\
        if(tracker_status&(0x1<<(5+i))) { \
          fprintf(stderr, " %d |",i);\
        }\
      }\
      fprintf(stderr, "\n"); \
      fprintf(stderr, " - The vectors that are the last of their line:\n   + Vector: "); \
      for(i=0;i<12;i++) {\
        if(tracker_status&(0x1<<(5+i))) { \
          fprintf(stderr, " %d |",i);\
        }\
      }\
      fprintf(stderr, "\n"); \
    }\
  }\
}

#endif /* _str_to_vec_v2_2_h_ */
