#ifndef _str_to_vec_defs_h_
#define _str_to_vec_defs_h_

#define _STR_TO_VEC_V2_2_USE_BAYER         use_bayer
#define _STR_TO_VEC_V2_2_USE_BAYER_2PPC    use_bayer_2ppc
#define _STR_TO_VEC_V2_2_USE_YUV420        use_yuv420
#define _STR_TO_VEC_V2_2_USE_YUV           use_yuv
#define _STR_TO_VEC_V2_2_USE_RGB           use_rgb		// ww2013'35 BXT

#define _STR_TO_VEC_V2_2_CMD_FIFO_DEPTH      cmd_fifo_depth
#define _STR_TO_VEC_V2_2_INT_ACK_FIFO_DEPTH  int_ack_fifo_depth
#define _STR_TO_VEC_V2_2_ACK_IN_FIFO_DEPTH   ack_in_fifo_depth
#define _STR_TO_VEC_V2_2_STR_IN_FIFO_DEPTH   str_in_fifo_depth
#define _STR_TO_VEC_V2_2_PIXEL_BITS          pixel_bits

#define _STR_TO_VEC_V2_2_CMD_INIT           16
#define _STR_TO_VEC_V2_2_CMD_PROC_N_VEC     1

#define _STR_TO_VEC_V2_2_ACK_INIT            0
#define _STR_TO_VEC_V2_2_ACK_N_VEC           1

#define _STR_TO_VEC_V2_2_REG_ALIGN           4

#define _STR_TO_VEC_V2_2_CMD                 0
#define _STR_TO_VEC_V2_2_ACK_K_VEC_REG       1
#define _STR_TO_VEC_V2_2_PXL_LINE_REG        2
#define _STR_TO_VEC_V2_2_LINE_FRAME_REG      3
#define _STR_TO_VEC_V2_2_YUV420_EN_REG       4
#define _STR_TO_VEC_V2_2_INTERLEAVE_EN_REG   5
#define _STR_TO_VEC_V2_2_DEV_NULL_EN_REG     6
#define _STR_TO_VEC_V2_2_IRQ_FALSE_CMD_REG   7
#define _STR_TO_VEC_V2_2_PXL_CUR_LINE_REG    38
#define _STR_TO_VEC_V2_2_LINES_DONE_REG      39
#define _STR_TO_VEC_V2_2_IO_STATUS_REG       40
#define _STR_TO_VEC_V2_2_ACK_STATUS_REG      41
#define _STR_TO_VEC_V2_2_MAIN_STATUS_REG     42
#define _STR_TO_VEC_V2_2_TRACK_STATUS_REG    43
#define _STR_TO_VEC_V2_2_CRUN_REG            44


#define _STR_TO_VEC_V2_2_0_ST_ADDR   8
#define _STR_TO_VEC_V2_2_0_END_ADDR  9
#define _STR_TO_VEC_V2_2_0_OFFSET    10
#define _STR_TO_VEC_V2_2_0_STRIDE    11

#define _STR_TO_VEC_V2_2_1_ST_ADDR  12
#define _STR_TO_VEC_V2_2_1_END_ADDR 13
#define _STR_TO_VEC_V2_2_1_OFFSET   14
#define _STR_TO_VEC_V2_2_1_STRIDE   15

#define _STR_TO_VEC_V2_2_2_ST_ADDR   16
#define _STR_TO_VEC_V2_2_2_END_ADDR  17
#define _STR_TO_VEC_V2_2_2_OFFSET    18
#define _STR_TO_VEC_V2_2_2_STRIDE    19

#define _STR_TO_VEC_V2_2_3_ST_ADDR   20
#define _STR_TO_VEC_V2_2_3_END_ADDR  21
#define _STR_TO_VEC_V2_2_3_OFFSET    22
#define _STR_TO_VEC_V2_2_3_STRIDE    23

#define _STR_TO_VEC_V2_2_4_ST_ADDR  24
#define _STR_TO_VEC_V2_2_4_END_ADDR 25
#define _STR_TO_VEC_V2_2_4_OFFSET   26
#define _STR_TO_VEC_V2_2_4_STRIDE   27

#define _STR_TO_VEC_V2_2_5_ST_ADDR   28
#define _STR_TO_VEC_V2_2_5_END_ADDR  29
#define _STR_TO_VEC_V2_2_5_OFFSET    30
#define _STR_TO_VEC_V2_2_5_STRIDE    31

#define _STR_TO_VEC_V2_2_0_CUR_ADDR  32
#define _STR_TO_VEC_V2_2_1_CUR_ADDR  33
#define _STR_TO_VEC_V2_2_2_CUR_ADDR  34
#define _STR_TO_VEC_V2_2_3_CUR_ADDR  35
#define _STR_TO_VEC_V2_2_4_CUR_ADDR  36
#define _STR_TO_VEC_V2_2_5_CUR_ADDR  37

#endif /* _str_to_vec_defs_h_ */
