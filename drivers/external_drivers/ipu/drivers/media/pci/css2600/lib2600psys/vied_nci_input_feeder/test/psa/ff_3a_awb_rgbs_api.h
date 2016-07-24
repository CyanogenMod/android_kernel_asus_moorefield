#ifndef _FF_AWB_RGBS_API_H
#define _FF_AWB_RGBS_API_H
#include "ff_3a_awb_rgbs_ccbcr.h"


#define ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_AWB_RGBS_id + addr, (val))

#define ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, addr) \
  vied_subsystem_load_32(psys0, FF_AWB_RGBS_id + addr)


#define ff_awb_rgbs_get_average(FF_AWB_RGBS_id, set_idx, idx, val, val1)                                    \
do {                                                                                                        \
    unsigned int addr = set_idx ? MEM_AWB_RGBS_COL_SET1_BASE_ADDR : MEM_AWB_RGBS_COL_SET0_BASE_ADDR;    \
    addr += idx * 8;                                                                                        \
    val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, addr);                                                \
    addr += 4;                                                                                              \
    val1 = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, addr);                                               \
} while (0);

#define ff_awb_rgbs_get_saturation(FF_AWB_RGBS_id, set_idx, idx, val, val1)                                 \
do {                                                                                                        \
    unsigned int addr = set_idx ? MEM_AWB_RGBS_SAT_SET1_BASE_ADDR : MEM_AWB_RGBS_SAT_SET0_BASE_ADDR;                \
    addr += idx * 4;                                                                                        \
    val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, addr);                                                \
    addr += 4;                                                                                              \
    val1 = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, addr);                                               \
} while (0);

#define ff_awb_rgbs_grd_cfg_set(FF_AWB_RGBS_id, g_w, g_h, b_w, b_h, ghpc, awb_en, rgbs_en, ris)             \
do {                                                                                                        \
    unsigned int val;                                                                                       \
    val = ((ris & 0x1) << 30) | ((rgbs_en & 0x1) << 29) | ((awb_en & 0x1) << 28) | ((ghpc & 0xF) << 22) |   \
          ((b_h & 0x7) << 19) | ((b_w & 0x7) << 16) | ((g_h & 0x7F) << 8) | (g_w & 0x7F);                   \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_GRD_CFG_ADDR, val);                             \
} while(0);

#define ff_awb_rgbs_grd_cfg_set_full(FF_AWB_RGBS_id, grd_reg)                                               \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_GRD_CFG_ADDR, grd_reg);

#define ff_awb_rgbs_grd_cfg_get(FF_AWB_RGBS_id, g_w, g_h, b_w, b_h, ghpc, awb_en, rgbs_en, ris)             \
do {                                                                                                        \
    unsigned int val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_GRD_CFG_ADDR);               \
    ris = (val >> 30) & 0x1;                                                                                \
    rgbs_en = (val >> 29) & 0x1;                                                                            \
    awb_en = (val >> 28) & 0x1;                                                                             \
    ghpc = (val >> 22) & 0xF;                                                                               \
    b_h = (val >> 19) & 0x7;                                                                                \
    b_w = (val >> 16) & 0x7;                                                                                \
    g_h = (val >> 8) & 0x7F;                                                                                \
    g_w = val & 0x7F;                                                                                       \
} while(0);

#define ff_awb_rgbs_grd_start_set(FF_AWB_RGBS_id, x_start, y_start)                                         \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_GRD_START_ADDR, ((y_start & 0x3FFF) << 16) | (x_start & 0x3FFF));

#define ff_awb_rgbs_grd_start_get(FF_AWB_RGBS_id, x_start, y_start)                                         \
do {                                                                                                        \
    unsigned int val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_GRD_START_ADDR);             \
    x_start = val & 0x7FFF;                                                                                 \
    y_start = (val >> 16) & 0x7FFF;                                                                         \
} while(0);

#define ff_awb_rgbs_grd_end_set(FF_AWB_RGBS_id, x_end, y_end)                                                         \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_GRD_END_ADDR, ((y_end & 0x3FFF) << 16) | (x_end & 0x3FFF));

#define ff_awb_rgbs_grd_end_get(FF_AWB_RGBS_id, x_end, y_end)                                               \
do {                                                                                                        \
    unsigned int val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_GRD_END_ADDR);               \
    x_end = val & 0x7FFF;                                                                                   \
    y_end = (val >> 16) & 0x7FFF;                                                                           \
} while(0);

#define ff_awb_rgbs_sensor_mode_set(FF_AWB_RGBS_id, sensor_mode)                                            \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SENSOR_CFG_0_ADDR, sensor_mode & 0x3);

#define ff_awb_rgbs_sensor_mode_get(FF_AWB_RGBS_id, sensor_mode)                                            \
do {                                                                                                        \
    unsigned int val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SENSOR_CFG_0_ADDR);          \
    sensor_mode = val & 0x1;                                                                                \
} while(0);

#define ff_awb_sensor_pattern_set(FF_AWB_RGBS_id, pat_arr)                          \
do {                                                                                \
    unsigned int val;                                                               \
    val = (((pat_arr[1][3] & 0x7) << 28) | ((pat_arr[1][2] & 0x7) << 24) |          \
           ((pat_arr[1][1] & 0x7) << 20) | ((pat_arr[1][0] & 0x7) << 16) |          \
           ((pat_arr[0][3] & 0x7) << 12) | ((pat_arr[0][2] & 0x7) << 8)  |          \
           ((pat_arr[0][1] & 0x7) << 4)  | (pat_arr[0][0] & 0x7));                  \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SENSOR_CFG_1_ADDR, val);\
    val = (((pat_arr[3][3] & 0x7) << 28) | ((pat_arr[3][2] & 0x7) << 24) |          \
           ((pat_arr[3][1] & 0x7) << 20) | ((pat_arr[3][0] & 0x7) << 16) |          \
           ((pat_arr[2][3] & 0x7) << 12) | ((pat_arr[2][2] & 0x7) << 8)  |          \
           ((pat_arr[2][1] & 0x7) << 4)  | (pat_arr[2][0] & 0x7));                  \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SENSOR_CFG_2_ADDR, val);\
} while(0);

#define ff_awb_sensor_pattern_get(FF_AWB_RGBS_id, pat_arr)                                          \
do {                                                                                                \
    unsigned int val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SENSOR_CFG_1_ADDR);  \
    pat_arr[0][0] = val & 0x7;                                                                      \
    pat_arr[0][1] = (val >> 4)  & 0x7;                                                              \
    pat_arr[0][2] = (val >> 8)  & 0x7;                                                              \
    pat_arr[0][3] = (val >> 12) & 0x7;                                                              \
    pat_arr[1][0] = (val >> 16) & 0x7;                                                              \
    pat_arr[1][1] = (val >> 20) & 0x7;                                                              \
    pat_arr[1][2] = (val >> 24) & 0x7;                                                              \
    pat_arr[1][3] = (val >> 28) & 0x7;                                                              \
    val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SENSOR_CFG_2_ADDR);               \
    pat_arr[2][0] = val & 0x7;                                                                      \
    pat_arr[2][1] = (val >> 4)  & 0x7;                                                              \
    pat_arr[2][2] = (val >> 8)  & 0x7;                                                              \
    pat_arr[2][3] = (val >> 12) & 0x7;                                                              \
    pat_arr[3][0] = (val >> 16) & 0x7;                                                              \
    pat_arr[3][1] = (val >> 20) & 0x7;                                                              \
    pat_arr[3][2] = (val >> 24) & 0x7;                                                              \
    pat_arr[3][3] = (val >> 28) & 0x7;                                                              \
} while(0);

#define ff_awb_rgbs_thrsh_set(FF_AWB_RGBS_id, thrsh_arr)                                                                            \
do {                                                                                                                                \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_0_ADDR, ((thrsh_arr[1] & 0xFFF) << 16) | (thrsh_arr[0] & 0xFFF)); \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_1_ADDR, ((thrsh_arr[3] & 0xFFF) << 16) | (thrsh_arr[2] & 0xFFF)); \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_2_ADDR, ((thrsh_arr[5] & 0xFFF) << 16) | (thrsh_arr[4] & 0xFFF)); \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_3_ADDR, ((thrsh_arr[7] & 0xFFF) << 16) | (thrsh_arr[6] & 0xFFF)); \
} while(0);

#define ff_awb_rgbs_thrsh_get(FF_AWB_RGBS_id, thrsh_arr)                                        \
do {                                                                                            \
    unsigned int val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_0_ADDR);   \
    thrsh_arr[0] = val & 0xFFF;                                                                 \
    thrsh_arr[1] = (val >> 16) & 0xFFF;                                                         \
    val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_1_ADDR)                 \
    thrsh_arr[2] = val & 0xFFF;                                                                 \
    thrsh_arr[3] = (val >> 16) & 0xFFF;                                                         \
    val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_2_ADDR)                 \
    thrsh_arr[4] = val & 0xFFF;                                                                 \
    thrsh_arr[5] = (val >> 16) & 0xFFF;                                                         \
    val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_THRSH_3_ADDR)                 \
    thrsh_arr[6] = val & 0xFFF;                                                                 \
    thrsh_arr[7] = (val >> 16) & 0xFFF;                                                         \
} while(0);

#define ff_awb_rgbs_shft_ctrl_set(FF_AWB_RGBS_id, shft_ctrl_arr)                            \
do {                                                                                        \
    unsigned int val;                                                                       \
    val = ((shft_ctrl_arr[3] & 0x1F) << 24) | ((shft_ctrl_arr[2] & 0x1F) << 16) |           \
           ((shft_ctrl_arr[1] & 0x1F) << 8) | (shft_ctrl_arr[0] & 0x1F);                    \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_AVG_SHFT_CTRL_0_ADDR, val);     \
    val = ((shft_ctrl_arr[7] & 0x1F) << 24) | ((shft_ctrl_arr[6] & 0x1F) << 16) |           \
           ((shft_ctrl_arr[5] & 0x1F) << 8) | (shft_ctrl_arr[4] & 0x1F);                    \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_AVG_SHFT_CTRL_1_ADDR, val);     \
} while(0);

#define ff_awb_rgbs_shft_ctrl_get(FF_AWB_RGBS_id, shft_ctrl_arr)                            \
do {                                                                                                \
    unsigned int val= ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_AVG_SHFT_CTRL_0_ADDR);\
    shft_ctrl_arr[0] = val & 0x1F;                                                                  \
    shft_ctrl_arr[1] = (val >> 8) & 0x1F;                                                           \
    shft_ctrl_arr[2] = (val >> 16) & 0x1F;                                                          \
    shft_ctrl_arr[3] = (val >> 24) & 0x1F;                                                          \
    val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_AVG_SHFT_CTRL_1_ADDR);            \
    shft_ctrl_arr[4] = val & 0x1F;                                                                  \
    shft_ctrl_arr[5] = (val >> 8) & 0x1F;                                                           \
    shft_ctrl_arr[6] = (val >> 16) & 0x1F;                                                          \
    shft_ctrl_arr[7] = (val >> 24) & 0x1F;                                                          \
} while(0);

#define ff_awb_rgbs_sat_shft_ctrl_set(FF_AWB_RGBS_id, shiftr_sat)                                   \
    ff_3a_awb_rgbs_set_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SAT_SHFT_CTRL_ADDR, shiftr_sat & 0x3FFF);

#define ff_awb_rgbs_sat_shft_ctrl_get(FF_AWB_RGBS_id, shiftr_sat)                                   \
do {                                                                                                \
    unsigned int val = ff_3a_awb_rgbs_get_register(FF_AWB_RGBS_id, FF_AWB_RGBS_SAT_SHFT_CTRL_ADDR); \
    shiftr_sat = val & 0x3FFF;                                                                      \
} while(0);


#endif // _FF_AWB_RGBS_API_H
