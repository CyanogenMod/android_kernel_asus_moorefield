#ifndef _FF_GAMMA_TM_API_H
#define _FF_GAMMA_TM_API_H
#include "ff_Gamma_TM_ccbcr.h"

#define GAMMA_LUT_NUM_ENTRYS    385
#define TM_LUT_NUM_ENTRYS       513
#define GAMMA_LUT_NUM_REGS      ((GAMMA_LUT_NUM_ENTRYS+1)/2)
#define TM_LUT_NUM_REGS         ((TM_LUT_NUM_ENTRYS+1)/2)
#define GAMMA_TM_SPARE_NUM_REG  7
#define GAMMA_SEG_CTRL_NUM_REG  5
#define TM_SEG_CTRL_NUM_REG     1
#define GAMMA_TM_TOTAL_NUM_REG  (GAMMA_LUT_NUM_REGS + TM_LUT_NUM_REGS + GAMMA_TM_SPARE_NUM_REG + GAMMA_SEG_CTRL_NUM_REG + TM_SEG_CTRL_NUM_REG)

#define ff_Gamma_TM_func_set_register(FF_GAMMA_TM_id, addr, val)                           \
  vied_subsystem_store_32(psys0, FF_GAMMA_TM_id + addr, (val))

#define ff_Gamma_TM_func_get_register(FF_GAMMA_TM_id, addr)                                \
  vied_subsystem_load_32(psys0, FF_GAMMA_TM_id + addr)


#define ff_set_Global_Ctrl0_reg(FF_GAMMA_TM_id, ff_en, gamma_en, tm_en, gamma_before_tm, a1, a2, a3)    \
do                                                                                                      \
{                                                                                                       \
    unsigned int val = ((a3 & 0x3) << 12) | ((a2 & 0x3) << 8) | ((a1 & 0x3) << 4) |                     \
    ((gamma_before_tm & 0x1) << 3) | ((tm_en & 0x1) << 2) | ((gamma_en & 0x1) << 1) | (ff_en & 0x1);    \
    ff_Gamma_TM_func_set_register(FF_GAMMA_TM_id, FF_GTM_GLOBAL_CTRL0_ADDR, val);                       \
}                                                                                                       \
while (0);

#define ff_get_Global_Ctrl0_reg(FF_GAMMA_TM_id, ff_en, gamma_en, tm_en, gamma_before_tm, a1, a2, a3)    \
do                                                                                                      \
{                                                                                                       \
    unsigned int val;                                                                                   \
    ff_Gamma_TM_func_get_register(FF_GAMMA_TM_id, FF_GTM_GLOBAL_CTRL0_ADDR, val);                       \
    ff_en           = val & 0x1;                                                                        \
    gamma_en        = (val >> 1)  & 0x1;                                                                \
    tm_en           = (val >> 2)  & 0x1;                                                                \
    gamma_before_tm = (val >> 3)  & 0x1;                                                                \
    a1              = (val >> 4)  & 0x3;                                                                \
    a2              = (val >> 8)  & 0x3;                                                                \
    a3              = (val >> 12) & 0x3;                                                                \
}                                                                                                       \
while (0);

#define ff_set_Global_Ctrl1_reg(FF_GAMMA_TM_id, gamma_lut_size, tm_lut_size)        \
do                                                                                  \
{                                                                                   \
    unsigned int val = ((tm_lut_size & 0x3ff) << 16) | (gamma_lut_size & 0x3ff);    \
    ff_Gamma_TM_func_set_register(FF_GAMMA_TM_id, FF_GTM_GLOBAL_CTRL1_ADDR, val);   \
}                                                                                   \
while (0);

#define ff_get_Global_Ctrl1_reg(FF_GAMMA_TM_id, gamma_lut_size, tm_lut_size)        \
do                                                                                  \
{                                                                                   \
    unsigned int val;                                                               \
    ff_Gamma_TM_func_get_register(FF_GAMMA_TM_id, FF_GTM_GLOBAL_CTRL1_ADDR, val);   \
    gamma_lut_size  = val & 0x3FF;                                                  \
    tm_lut_size     = (val >> 16) & 0x3FF;                                          \
}                                                                                   \
while (0);


#define ff_set_Gamma_Lut_Entry(FF_GAMMA_TM_id, index, Lut_Entry0, Lut_Entry1)                                   \
do                                                                                                              \
{                                                                                                               \
    unsigned int addr = 0 + 4 * index;                                                                          \
    ff_Gamma_TM_func_set_register(FF_GAMMA_TM_id, addr, ((Lut_Entry1 & 0x7FFF) << 16) | (Lut_Entry0 & 0x7FFF)); \
}                                                                                                               \
while (0);

#define ff_set_Gamma_Lut_Entry_Array(FF_GAMMA_TM_id, Gamma_Lut_Entry_Arr)                                   \
do                                                                                                          \
{                                                                                                           \
    int i;                                                                                                  \
    for (i = 0; i < GAMMA_LUT_NUM_REGS; i++)                                                                \
    {                                                                                                       \
        ff_set_Gamma_Lut_Entry(FF_GAMMA_TM_id, i, Gamma_Lut_Entry_Arr[2*i], Gamma_Lut_Entry_Arr[2*i + 1])   \
    }                                                                                                       \
}                                                                                                           \
while(0);

#define ff_set_TM_Lut_Entry(FF_GAMMA_TM_id, index, Lut_Entry0, Lut_Entry1)                                      \
do                                                                                                              \
{                                                                                                               \
    unsigned int addr = 0x320 + 4 * index;                                                                      \
    ff_Gamma_TM_func_set_register(FF_GAMMA_TM_id, addr, ((Lut_Entry1 & 0x7FFF) << 16) | (Lut_Entry0 & 0x7FFF)); \
}                                                                                                               \
while (0);

#define ff_set_TM_Lut_Entry_Array(FF_GAMMA_TM_id, TM_Lut_Entry_Arr)                                 \
do                                                                                                  \
{                                                                                                   \
    int i;                                                                                          \
    for (i = 0; i < TM_LUT_NUM_REGS; i++)                                                           \
    {                                                                                               \
        ff_set_TM_Lut_Entry(FF_GAMMA_TM_id, i, TM_Lut_Entry_Arr[2*i], TM_Lut_Entry_Arr[2*i + 1])    \
    }                                                                                               \
}                                                                                                   \
while(0);

#define ff_set_Gamma_Seg_Cfg(FF_GAMMA_TM_id, Base_Level_arr, Step_Size_arr, Start_Bin_arr)                          \
do                                                                                                                  \
{                                                                                                                   \
    int i;                                                                                                          \
    unsigned int reg = 0;                                                                                           \
    for (i = 0; i < GAMMA_SEG_CTRL_NUM_REG; i++)                                                                    \
    {                                                                                                               \
        reg = ((Start_Bin_arr[i] & 0x3FF) << 20) | ((Step_Size_arr[i] & 0xF) << 16) | (Base_Level_arr[i] & 0x7FFF); \
        ff_Gamma_TM_func_set_register(FF_GAMMA_TM_id, GAMMA_SEG_CFG_0_ADDR + 4 * i, reg);                           \
    }                                                                                                               \
}                                                                                                                   \
while (0);

#define ff_set_TM_Seg_Cfg(FF_GAMMA_TM_id, Base_Level_arr, Step_Size_arr, Start_Bin_arr)                             \
do                                                                                                                  \
{                                                                                                                   \
    int i;                                                                                                          \
    unsigned int reg = 0;                                                                                           \
    for (i = 0; i < TM_SEG_CTRL_NUM_REG; i++)                                                                       \
    {                                                                                                               \
        reg = ((Start_Bin_arr[i] & 0x3FF) << 20) | ((Step_Size_arr[i] & 0xF) << 16) | (Base_Level_arr[i] & 0x7FFF); \
        ff_Gamma_TM_func_set_register(FF_GAMMA_TM_id, TM_SEG_CFG_0_ADDR + 4 * i, reg);                              \
    }                                                                                                               \
}                                                                                                                   \
while (0);

#endif // _FF_GAMMA_TM_API_H
