#ifndef _ff_anr_transform_api_h
#define _ff_anr_transform_api_h

#include "ff_anr_transform_ccbcr.h"


#define ff_anr_transform_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_anr_transform_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_anr_transform_enable(FF_id, thr_en) \
	ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_CFG_ADDR, ((thr_en) << 1) | 1);

#define ff_anr_transform_disable(FF_id) \
	ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_CFG_ADDR, 0);

#define ff_anr_transform_set_params(FF_id, Xreset, Yreset, N, a)                                                \
do {                                                                                                            \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_R_CALC_1_ADDR, (((Yreset) & 0x1FFF) << 16) | (Xreset & 0x1FFF));  \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_R_CALC_2_ADDR, ((N) << 24) | ((Xreset) * (Xreset)));  \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_R_CALC_3_ADDR, ((a) << 24) | ((Yreset) * (Yreset)));  \
}                                                                                                               \
while (0)

#define ff_anr_transform_set_Alphas(FF_id, plane, alpha_Gr, alpha_R, alpha_B, alpha_Gb)                         \
do {                                                                                                            \
    int planeDiff = (FF_ANR_TRANSFORM_PLANE_1_ALPHA_0_ADDR - FF_ANR_TRANSFORM_PLANE_0_ALPHA_0_ADDR);            \
    int offset = plane * planeDiff;                                                                             \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_PLANE_0_ALPHA_0_ADDR + offset, ((alpha_R) << 16) | alpha_Gr);\
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_PLANE_0_ALPHA_1_ADDR + offset, ((alpha_Gb) << 16) | alpha_B);\
}                                                                                                               \
while (0)

#define ff_anr_transform_set_AlphaDCs(FF_id, plane, alpha_Gr, alpha_R, alpha_B, alpha_Gb)                       \
do {                                                                                                            \
    int planeDiff = (FF_ANR_TRANSFORM_PLANE_1_ALPHA_DC0_ADDR - FF_ANR_TRANSFORM_PLANE_0_ALPHA_DC0_ADDR);        \
    int offset = plane * planeDiff;                                                                             \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_PLANE_0_ALPHA_DC0_ADDR + offset, ((alpha_R) << 16) | alpha_Gr);\
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_PLANE_0_ALPHA_DC1_ADDR + offset, ((alpha_Gb) << 16) | alpha_B);\
}                                                                                                               \
while (0)

#define ff_anr_transform_set_Betas(FF_id, plane, alpha_Gr, alpha_R, alpha_B, alpha_Gb)                          \
do {                                                                                                            \
    int planeDiff = (FF_ANR_TRANSFORM_PLANE_1_BETA_0_ADDR - FF_ANR_TRANSFORM_PLANE_0_BETA_0_ADDR);              \
    int offset = plane * planeDiff;                                                                             \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_PLANE_0_BETA_0_ADDR + offset, ((alpha_R) << 16) | alpha_Gr);\
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_PLANE_0_BETA_1_ADDR + offset, ((alpha_Gb) << 16) | alpha_B);\
}                                                                                                               \
while (0)

// Set the W parameters - the Array is 15 parameters long
#define ff_anr_transform_set_w(FF_id, plane, color, arr)                                                        \
do {                                                                                                            \
    short planeDiff = (FF_ANR_TRANSFORM_PLANE_1_GR_REG_0_W_ADDR - FF_ANR_TRANSFORM_PLANE_0_GR_REG_0_W_ADDR);    \
    short colorDiff = (FF_ANR_TRANSFORM_PLANE_0_R_REG_0_W_ADDR - FF_ANR_TRANSFORM_PLANE_0_GR_REG_0_W_ADDR);     \
    unsigned int baseAddr = FF_ANR_TRANSFORM_PLANE_0_GR_REG_0_W_ADDR + ((plane) * planeDiff);                   \
    baseAddr += (color) * colorDiff;                                                                            \
    ff_anr_transform_set_register(FF_id, baseAddr     , (arr[1] << 16) | arr[0]);                               \
    ff_anr_transform_set_register(FF_id, baseAddr + 4 , (arr[3] << 16) | arr[2]);                               \
    ff_anr_transform_set_register(FF_id, baseAddr + 8 , (arr[5] << 16) | arr[4]);                               \
    ff_anr_transform_set_register(FF_id, baseAddr + 12, (arr[7] << 16) | arr[6]);                               \
    ff_anr_transform_set_register(FF_id, baseAddr + 16, (arr[9] << 16) | arr[8]);                               \
    ff_anr_transform_set_register(FF_id, baseAddr + 20, (arr[11] << 16) | arr[10]);                             \
    ff_anr_transform_set_register(FF_id, baseAddr + 24, (arr[13] << 16) | arr[12]);                             \
    ff_anr_transform_set_register(FF_id, baseAddr + 28,  arr[14]);                                              \
}                                                                                                               \
while (0)


// The Sqrt_Lut is preset of 25 values
#define ff_anr_transform_set_sqrt_lut(FF_id, sqrt_arr)                                                              \
do {                                                                                                                \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_0_ADDR, ((sqrt_arr[1]) << 16) | sqrt_arr[0]);    \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_1_ADDR, ((sqrt_arr[3]) << 16) | sqrt_arr[2]);    \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_2_ADDR, ((sqrt_arr[5]) << 16) | sqrt_arr[4]);    \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_3_ADDR, ((sqrt_arr[7]) << 16) | sqrt_arr[6]);    \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_4_ADDR, ((sqrt_arr[9]) << 16) | sqrt_arr[8]);    \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_5_ADDR, ((sqrt_arr[11]) << 16) | sqrt_arr[10]);  \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_6_ADDR, ((sqrt_arr[13]) << 16) | sqrt_arr[12]);  \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_7_ADDR, ((sqrt_arr[15]) << 16) | sqrt_arr[14]);  \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_8_ADDR, ((sqrt_arr[17]) << 16) | sqrt_arr[16]);  \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_9_ADDR, ((sqrt_arr[19]) << 16) | sqrt_arr[18]);  \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_10_ADDR, ((sqrt_arr[21]) << 16) | sqrt_arr[20]); \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_11_ADDR, ((sqrt_arr[23]) << 16) | sqrt_arr[22]); \
    ff_anr_transform_set_register(FF_id, FF_ANR_TRANSFORM_SQRT_LUT_12_ADDR, sqrt_arr[24]);                          \
}                                                                                                                   \
while (0)

#endif
