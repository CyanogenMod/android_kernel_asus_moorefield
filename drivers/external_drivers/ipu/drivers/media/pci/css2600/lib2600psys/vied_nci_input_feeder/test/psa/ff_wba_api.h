#ifndef _FF_WBA_API_H
#define _FF_WBA_API_H
#include "ff_wba_regs.h"

#define ff_wba_set_register(FF_WBA_id, addr, val)               \
  vied_subsystem_store_32(psys0, FF_WBA_id + addr, (val))

#define ff_wba_get_register(FF_WBA_id, addr) \
  vied_subsystem_store_32(psys0, FF_WBA_id + addr)


#define ff_wba_enable(FF_WBA_id)                                \
  ff_wba_set_register(FF_WBA_id, FF_WBA_GEN_CTRL, 0x01)

#define ff_wba_disable(FF_WBA_id)                               \
  ff_wba_set_register(FF_WBA_id, FF_WBA_GEN_CTRL, 0x00)

#define ff_wba_set_Sensor_Mode(FF_WBA_id, Sensor_mode)          \
  ff_wba_set_register(FF_WBA_id, FF_WBA_SENSOR_CFG_0, (Sensor_mode & 0x3))

#define ff_wba_get_Sensor_Mode(FF_WBA_id, Sensor_mode)                      \
do {                                                                        \
    unsigned int reg0 = ff_wba_get_register(FF_WBA_id, FF_WBA_SENSOR_CFG_0);\
    Sensor_mode = reg0 & 0x3;                                               \
} while(0);

#define ff_wba_set_WB_Gains(FF_WBA_id, Gch0, Gch1, Gch2, Gch3, Gch4, Gch5, Gch6, Gch7)  \
do {                                                                                    \
    ff_wba_set_register(FF_WBA_id, FF_WBA_GAIN_CTRL0, (Gch1 << 16) | (Gch0 & 0xFFFF));  \
    ff_wba_set_register(FF_WBA_id, FF_WBA_GAIN_CTRL1, (Gch3 << 16) | (Gch2 & 0xFFFF));	\
    ff_wba_set_register(FF_WBA_id, FF_WBA_GAIN_CTRL2, (Gch5 << 16) | (Gch4 & 0xFFFF));  \
    ff_wba_set_register(FF_WBA_id, FF_WBA_GAIN_CTRL3, (Gch7 << 16) | (Gch6 & 0xFFFF));  \
} while(0);


#define ff_wba_get_WB_Gains(FF_WBA_id, Gch0, Gch1, Gch2, Gch3, Gch4, Gch5, Gch6, Gch7)  \
do {                                                                                    \
    unsigned int reg0 = ff_wba_get_register(FF_WBA_id, FF_WBA_GAIN_CTRL0);              \
    unsigned int reg1 = ff_wba_get_register(FF_WBA_id, FF_WBA_GAIN_CTRL1);              \
    unsigned int reg2 = ff_wba_get_register(FF_WBA_id, FF_WBA_GAIN_CTRL2);              \
    unsigned int reg3 = ff_wba_get_register(FF_WBA_id, FF_WBA_GAIN_CTRL3);              \
    Gch0 = reg0 & 0xFFFF;                                                               \
    Gch1 = reg0 >> 16;                                                                  \
    Gch2 = reg1 & 0xFFFF;                                                               \
    Gch3 = reg1 >> 16;                                                                  \
    Gch4 = reg2 & 0xFFFF;                                                               \
    Gch5 = reg2 >> 16;                                                                  \
    Gch6 = reg3 & 0xFFFF;                                                               \
    Gch7 = reg3 >> 16;                                                                  \
} while(0);

#define ff_wba_set_CFA_Pattern(FF_WBA_id, Pat_00, Pat_01, Pat_02, Pat_03, Pat_10, Pat_11, Pat_12, Pat_13,   \
                                          Pat_20, Pat_21, Pat_22, Pat_23, Pat_30, Pat_31, Pat_32, Pat_33)   \
do {                                                                                                        \
    ff_wba_set_register(FF_WBA_id, FF_WBA_SENSOR_CFG_1,                                                     \
        ((Pat_13 & 0xF) << 28) | ((Pat_12 & 0xF) << 24) | ((Pat_11 & 0xF) << 20) | ((Pat_10 & 0xF) << 16) | \
        ((Pat_03 & 0xF) << 12) | ((Pat_02 & 0xF) << 8)  | ((Pat_01 & 0xF) << 4)  | ((Pat_00 & 0xF)));       \
    ff_wba_set_register(FF_WBA_id, FF_WBA_SENSOR_CFG_2,                                                     \
        ((Pat_33 & 0xF) << 28) | ((Pat_32 & 0xF) << 24) | ((Pat_31 & 0xF) << 20) | ((Pat_30 & 0xF) << 16) | \
        ((Pat_23 & 0xF) << 12) | ((Pat_22 & 0xF) << 8)  | ((Pat_21 & 0xF) << 4)  | ((Pat_20 & 0xF)));       \
} while(0);

#define ff_wba_get_CFA_Pattern(FF_WBA_id, Pat_00, Pat_01, Pat_02, Pat_03, Pat_10, Pat_11, Pat_12, Pat_13,   \
                                          Pat_20, Pat_21, Pat_22, Pat_23, Pat_30, Pat_31, Pat_32, Pat_33)   \
do {                                                                                                        \
    unsigned int reg0 = ff_wba_get_register(FF_WBA_id, FF_WBA_SENSOR_CFG_1);                                \
    unsigned int reg1 = ff_wba_get_register(FF_WBA_id, FF_WBA_SENSOR_CFG_2);                                \
    Pat_00 = reg0 & 0xF;                                                                                    \
    Pat_01 = (reg0 >> 4)  & 0xF;                                                                            \
    Pat_02 = (reg0 >> 8)  & 0xF;                                                                            \
    Pat_03 = (reg0 >> 12) & 0xF;                                                                            \
    Pat_10 = (reg0 >> 16) & 0xF;                                                                            \
    Pat_11 = (reg0 >> 20) & 0xF;                                                                            \
    Pat_12 = (reg0 >> 24) & 0xF;                                                                            \
    Pat_13 = (reg0 >> 28) & 0xF;                                                                            \
    Pat_20 = reg1 & 0xF;                                                                                    \
    Pat_21 = (reg1 >> 4)  & 0xF;                                                                            \
    Pat_22 = (reg1 >> 8)  & 0xF;                                                                            \
    Pat_23 = (reg1 >> 12) & 0xF;                                                                            \
    Pat_30 = (reg1 >> 16) & 0xF;                                                                            \
    Pat_31 = (reg1 >> 20) & 0xF;                                                                            \
    Pat_32 = (reg1 >> 24) & 0xF;                                                                            \
    Pat_33 = (reg1 >> 28) & 0xF;                                                                            \
} while(0);

#endif // _FF_WBA_API_H
