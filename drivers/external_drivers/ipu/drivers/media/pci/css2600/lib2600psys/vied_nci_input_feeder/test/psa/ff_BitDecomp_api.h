#ifndef _FF_BITDECOMP_API_H
#define _FF_BITDECOMP_API_H
#include "ff_BitDecomp_ccbcr.h"


#define ff_BitDecomp_func_set_register(FF_BITDECOMP_id, addr, val)                           \
  vied_subsystem_store_32(psys0, FF_BITDECOMP_id + addr, (val))

#define ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr)                                \
  vied_subsystem_load_32(psys0, FF_BITDECOMP_id + addr)


#define ff_BitDecomp_enable(FF_BITDECOMP_id)                                                \
do                                                                                          \
{                                                                                           \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, FF_BDC_CTRL_ADDR);   \
    ff_BitDecomp_func_set_register(FF_BITDECOMP_id, FF_BDC_CTRL_ADDR, reg | 0x10000);       \
}                                                                                           \
while (0);

#define ff_BitDecomp_disable(FF_BITDECOMP_id)                                               \
do                                                                                          \
{                                                                                           \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, FF_BDC_CTRL_ADDR);   \
    ff_BitDecomp_func_set_register(FF_BITDECOMP_id, FF_BDC_CTRL_ADDR, reg & 0xFFFEFFFF);    \
}                                                                                           \
while (0);

#define ff_BitDecomp_set_Lut_Size(FF_BITDECOMP_id, Lut_Size)                                \
do                                                                                          \
{                                                                                           \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, FF_BDC_CTRL_ADDR);   \
    ff_BitDecomp_func_set_register(FF_BITDECOMP_id, FF_BDC_CTRL_ADDR, (reg & 0xFF) | Lut_Size);\
}                                                                                           \
while (0);

#define ff_BitDecomp_get_Lut_Size(FF_BITDECOMP_id, Lut_Size)                                \
do                                                                                          \
{                                                                                           \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, FF_BDC_CTRL_ADDR);   \
    Lut_Size = reg & 0xFF;                                                                  \
}                                                                                           \
while (0);

#define ff_BitDecomp_set_Base_Level(FF_BITDECOMP_id, index, Base_Level)                     \
do                                                                                          \
{                                                                                           \
    unsigned int addr = FF_BDC_SEG_0_CTRL_ADDR + 4 * index;                                 \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr);               \
    ff_BitDecomp_func_set_register(FF_BITDECOMP_id, addr, (reg & 0xFFFF0000) | (Base_Level & 0x3FFF)); \
}                                                                                           \
while (0);

#define ff_BitDecomp_set_Base_Level_Array(FF_BITDECOMP_id, Base_Level)                      \
do                                                                                          \
{                                                                                           \
    int i;                                                                                  \
    for (i = 0; i < 8; i++)                                                                 \
    {                                                                                       \
        ff_BitDecomp_set_Base_Level(FF_BITDECOMP_id, i, Base_Level[i]);                     \
    }                                                                                       \
}                                                                                           \
while (0);

#define ff_BitDecomp_get_Base_Level(FF_BITDECOMP_id, index, Base_Level)         \
do                                                                              \
{                                                                               \
    unsigned int addr = FF_BDC_SEG_0_CTRL_ADDR + 4 * index;                     \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr);   \
    Base_Level = reg & 0xFFFF;                                                  \
}                                                                               \
while (0);

#define ff_BitDecomp_set_Step_Size(FF_BITDECOMP_id, index, Step_Size)                                       \
do                                                                                                          \
{                                                                                                           \
    unsigned int addr = FF_BDC_SEG_0_CTRL_ADDR + 4 * index;                                                 \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr);                               \
    ff_BitDecomp_func_set_register(FF_BITDECOMP_id, addr, (reg & 0xFFF0FFFF) | ((Step_Size & 0xF) << 16));  \
}                                                                                                           \
while (0);

#define ff_BitDecomp_set_Step_Size_Array(FF_BITDECOMP_id, Step_Size)            \
do                                                                              \
{                                                                               \
    int i;                                                                      \
    for (i = 0; i < 8; i++)                                                     \
    {                                                                           \
        ff_BitDecomp_set_Step_Size(FF_BITDECOMP_id, i, Step_Size[i]);           \
    }                                                                           \
}                                                                               \
while (0);

#define ff_BitDecomp_get_Step_Size(FF_BITDECOMP_id, index, Step_Size)           \
do                                                                              \
{                                                                               \
    unsigned int addr = FF_BDC_SEG_0_CTRL_ADDR + 4 * index;                     \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr);   \
    Step_Size = (reg & 0xF0000) >> 16;                                          \
}                                                                               \
while (0);

#define ff_BitDecomp_set_Start_Bin(FF_BITDECOMP_id, index, Start_Bin)                                       \
do                                                                                                          \
{                                                                                                           \
    unsigned int addr = FF_BDC_SEG_0_CTRL_ADDR + 4 * index;                                                 \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr);                               \
    ff_BitDecomp_func_set_register(FF_BITDECOMP_id, addr, (reg & 0xE00FFFFF) | ((Start_Bin & 0x1FF) << 20));\
}                                                                                                           \
while (0);

#define ff_BitDecomp_set_Start_Bin_Array(FF_BITDECOMP_id, Start_Bin)            \
do                                                                              \
{                                                                               \
    int i;                                                                      \
    for (i = 0; i < 8; i++)                                                     \
    {                                                                           \
        ff_BitDecomp_set_Start_Bin(FF_BITDECOMP_id, i, Start_Bin[i]);           \
    }                                                                           \
}                                                                               \
while (0);

#define ff_BitDecomp_get_Start_Bin(FF_BITDECOMP_id, index, Start_Bin)           \
do                                                                              \
{                                                                               \
    unsigned int addr = FF_BDC_SEG_0_CTRL_ADDR + 4 * index;                     \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr);   \
    Start_Bin = (reg & 0x1FF00000) >> 20;                                       \
}                                                                               \
while (0);

#define ff_BitDecomp_set_Lut_Entry(FF_BITDECOMP_id, index, Lut_Entry0, Lut_Entry1)                              \
do                                                                                                              \
{                                                                                                               \
    unsigned int addr = 0 + 4 * index;                                                                          \
    ff_BitDecomp_func_set_register(FF_BITDECOMP_id, addr, ((Lut_Entry1 & 0xFFFF) << 16) | (Lut_Entry0 & 0xFFFF));\
}                                                                                                               \
while (0);

#define ff_BitDecomp_set_Lut_Entry_Array(FF_BITDECOMP_id, Lut_Entry_Arr)                            \
do                                                                                                  \
{                                                                                                   \
    int i;                                                                                          \
    for (i = 0; i < 128; i++)                                                                       \
    {                                                                                               \
        ff_BitDecomp_set_Lut_Entry(FF_BITDECOMP_id, i, Lut_Entry_Arr[2*i], Lut_Entry_Arr[2*i + 1])  \
    }                                                                                               \
}                                                                                                   \
while(0);

#define ff_BitDecomp_get_Lut_Entry(FF_BITDECOMP_id, index, Lut_Entry0, Lut_Entry1)  \
do                                                                                  \
{                                                                                   \
    unsigned int addr = 0 + 4 * index;                                              \
    unsigned int reg = ff_BitDecomp_func_get_register(FF_BITDECOMP_id, addr);       \
    Lut_Entry0 = reg & 0xFFFF;                                                      \
    Lut_Entry1 = (reg >> 16) & 0xFFFF;                                              \
}                                                                                   \
while (0);

#define ff_BitDecomp_set_Lut_Seg_Cfg(FF_BITDECOMP_id, Base_Level_arr, Step_Size_arr, Start_Bin_arr)                 \
do                                                                                                                  \
{                                                                                                                   \
    int i;                                                                                                          \
    unsigned int reg = 0;                                                                                           \
    for (i = 0; i < 8; i++)                                                                                         \
    {                                                                                                               \
        reg = ((Start_Bin_arr[i] & 0x1FF) << 20) | ((Step_Size_arr[i] & 0xF) << 16) | (Base_Level_arr[i] & 0x3FFF); \
        ff_BitDecomp_func_set_register(FF_BITDECOMP_id, FF_BDC_SEG_0_CTRL_ADDR + 4 * i, reg);                            \
    }                                                                                                               \
}                                                                                                                   \
while (0);

#endif // _FF_BITDECOMP_API_H
