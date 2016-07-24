#ifndef _ff_ae_ccm_regs_h
#define _ff_ae_ccm_regs_h
#include "ff_ae_ccm_ccbcr.h"


#define ff_ae_ccm_set_register(FF_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_ae_ccm_get_register(FF_id, addr) \
  vied_subsystem_load_32(psys0, FF_id + addr)

#define ff_ae_ccm_set_wb_Coeff_Matrix(FF_id, gainArr) \
do {												                                                                        \
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_WB_COEFF_0_ADDR, ((((gainArr[1]) & 0x07FF) << 16) | ((gainArr[0]) & 0x07FF)));	\
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_WB_COEFF_1_ADDR, ((((gainArr[3]) & 0x07FF) << 16) | ((gainArr[2]) & 0x07FF)));	\
    ff_ae_ccm_set_register(FF_id, FF_AE_CCM_WB_COEFF_2_ADDR, ((((gainArr[5]) & 0x07FF) << 16) | ((gainArr[4]) & 0x07FF)));	\
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_WB_COEFF_3_ADDR, ((((gainArr[7]) & 0x07FF) << 16) | ((gainArr[6]) & 0x07FF)));	\
} while(0)

#define ff_ae_ccm_get_wb_Coeff_Matrix(FF_id, gainArr)                                \
do {												                                 \
    unsigned int data = ff_ae_ccm_get_register(FF_id, FF_AE_CCM_WB_COEFF_0_ADDR);	 \
    gainArr[0] = data & 0x07FF;                                                      \
    gainArr[1] = data >> 16;                                                         \
	data = ff_ae_ccm_get_register(FF_id, FF_AE_CCM_WB_COEFF_1_ADDR);	             \
    gainArr[2] = data & 0x07FF;                                                      \
    gainArr[3] = data >> 16;                                                         \
    data = ff_ae_ccm_get_register(FF_id, FF_AE_CCM_WB_COEFF_2_ADDR);	             \
    gainArr[4] = data & 0x07FF;                                                      \
    gainArr[5] = data >> 16;                                                         \
	data = ff_ae_ccm_get_register(FF_id, FF_AE_CCM_WB_COEFF_3_ADDR);	             \
    gainArr[6] = data & 0x07FF;                                                      \
    gainArr[7] = data >> 16;                                                         \
} while(0)


#define ff_ae_ccm_set_ccm_Coeff_Matrix(FF_id, mArr)                                                                         \
do {												                                                                        \
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_0_ADDR, (((mArr[1]) & 0x03FF) << 16) | ((mArr[0]) & 0x03FF));		\
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_1_ADDR, (((mArr[3]) & 0x03FF) << 16) | ((mArr[2]) & 0x03FF));		\
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_2_ADDR, (((mArr[5]) & 0x03FF) << 16) | ((mArr[4]) & 0x03FF));		\
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_3_ADDR, (((mArr[7]) & 0x03FF) << 16) | ((mArr[6]) & 0x03FF));		\
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_4_ADDR, (((mArr[9]) & 0x03FF) << 16) | ((mArr[8]) & 0x03FF));		\
	ff_ae_ccm_set_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_5_ADDR, (((mArr[11]) & 0x03FF) << 16) | ((mArr[10]) & 0x03FF));	\
} while(0)

// The macro below assumes that mXY is a short variable (signed 16 bits)
#define ff_ae_ccm_get_ccm_Coeff_Matrix(FF_id, mArr)                                     \
do {												                                    \
	unsigned int reg0 = FF_ccm_get_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_0_ADDR);	    \
	unsigned int reg1 = FF_ccm_get_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_1_ADDR);	    \
	unsigned int reg2 = FF_ccm_get_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_2_ADDR);	    \
	unsigned int reg3 = FF_ccm_get_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_3_ADDR);	    \
	unsigned int reg4 = FF_ccm_get_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_4_ADDR);	    \
	unsigned int reg5 = FF_ccm_get_register(FF_id, FF_AE_CCM_CCM_MAT_COEFF_5_ADDR);	    \
	mArr[0]  = reg0 & 0xFFFF;									                        \
	mArr[1]  = reg0 >> 16;									                            \
	mArr[2]  = reg1 & 0xFFFF;									                        \
    mArr[3]  = reg1 >> 16;									                            \
	mArr[4]  = reg2 & 0xFFFF;									                        \
	mArr[5]  = reg2 >> 16;									                            \
	mArr[6]  = reg3 & 0xFFFF;									                        \
    mArr[7]  = reg3 >> 16;									                            \
	mArr[8]  = reg4 & 0xFFFF;									                        \
	mArr[9]  = reg4 >> 16;									                            \
	mArr[10] = reg5 & 0xFFFF;`									                        \
    mArr[11] = reg5 >> 16;									                            \
    mArr[0]  = (m11 & 0x200) ? (m11 | 0xFC00) : m11;                                    \
    mArr[1]  = (m12 & 0x200) ? (m12 | 0xFC00) : m12;                                    \
    mArr[2]  = (m13 & 0x200) ? (m13 | 0xFC00) : m13;                                    \
    mArr[3]  = (m14 & 0x200) ? (m14 | 0xFC00) : m14;                                    \
    mArr[4]  = (m21 & 0x200) ? (m21 | 0xFC00) : m21;                                    \
    mArr[5]  = (m22 & 0x200) ? (m22 | 0xFC00) : m22;                                    \
    mArr[6]  = (m23 & 0x200) ? (m23 | 0xFC00) : m23;                                    \
    mArr[7]  = (m24 & 0x200) ? (m24 | 0xFC00) : m24;                                    \
    mArr[8]  = (m31 & 0x200) ? (m31 | 0xFC00) : m31;                                    \
    mArr[9]  = (m32 & 0x200) ? (m32 | 0xFC00) : m32;                                    \
    mArr[10] = (m33 & 0x200) ? (m33 | 0xFC00) : m33;                                    \
    mArr[11] = (m34 & 0x200) ? (m34 | 0xFC00) : m34;                                    \
} while(0)

#define ff_ae_ccm_set_Ycalc_Coeff_Matrix(FF_id, Ynum, mArr)                                                     \
do {                                                                                                            \
    unsigned int baseAddr[4] = {FF_AE_CCM_Y0_CALC_MAT_COEFF_0_ADDR, FF_AE_CCM_Y1_CALC_MAT_COEFF_0_ADDR, FF_AE_CCM_Y2_CALC_MAT_COEFF_0_ADDR, FF_AE_CCM_Y3_CALC_MAT_COEFF_0_ADDR};  \
    ff_ae_ccm_set_register(FF_id, baseAddr[Ynum] + 0x0, (((mArr[1]) & 0x03FF) << 16) | ((mArr[0]) & 0x3FF));    \
    ff_ae_ccm_set_register(FF_id, baseAddr[Ynum] + 0x4, (((mArr[3]) & 0x03FF) << 16) | ((mArr[2]) & 0x3FF));    \
    ff_ae_ccm_set_register(FF_id, baseAddr[Ynum] + 0x8, (((mArr[5]) & 0x03FF) << 16) | ((mArr[4]) & 0x3FF));    \
    ff_ae_ccm_set_register(FF_id, baseAddr[Ynum] + 0xC, (((mArr[7]) & 0x03FF) << 16) | ((mArr[6]) & 0x3FF));    \
} while(0)

// The macro below assumes that mX is a short variable (signed 16 bits)
#define ff_ae_ccm_get_Ycalc_Coeff_Matrix(FF_id, Ynum, mArr)                                               \
do {                                                                                                      \
    unsigned int baseAddr[4] = {FF_AE_CCM_Y0_CALC_MAT_COEFF_0_ADDR, FF_AE_CCM_Y1_CALC_MAT_COEFF_0_ADDR, FF_AE_CCM_Y2_CALC_MAT_COEFF_0_ADDR, FF_AE_CCM_Y3_CALC_MAT_COEFF_0_ADDR};  \
    unsigned int data = ff_ae_ccm_get_register(FF_id, (baseAddr[Ynum] + 0x0));                            \
    mArr[0] = data & 0x3FF;                                                                               \
    mArr[1] = data >> 16;                                                                                 \
    data = ff_ae_ccm_get_register(FF_id, (baseAddr[Ynum] + 0x4));                                         \
    mArr[2] = data & 0x3FF;                                                                               \
    mArr[3] = data >> 16;                                                                                 \
    data = ff_ae_ccm_get_register(FF_id, (baseAddr[Ynum] + 0x8));                                         \
    mArr[4] = data & 0x3FF;                                                                               \
    mArr[5] = data >> 16;                                                                                 \
    data = ff_ae_ccm_get_register(FF_id, (baseAddr[Ynum] + 0xC));                                         \
    mArr[6] = data & 0x3FF;                                                                               \
    mArr[7] = data >> 16;                                                                                 \
    mArr[0] = (m0 & 0x2FF) ? (m0 | 0xFC00) : (m0);                                                        \
    mArr[1] = (m1 & 0x2FF) ? (m1 | 0xFC00) : (m1);                                                        \
    mArr[2] = (m2 & 0x2FF) ? (m2 | 0xFC00) : (m2);                                                        \
    mArr[3] = (m3 & 0x2FF) ? (m3 | 0xFC00) : (m3);                                                        \
    mArr[4] = (m4 & 0x2FF) ? (m4 | 0xFC00) : (m4);                                                        \
    mArr[5] = (m5 & 0x2FF) ? (m5 | 0xFC00) : (m5);                                                        \
    mArr[6] = (m6 & 0x2FF) ? (m6 | 0xFC00) : (m6);                                                        \
    mArr[7] = (m7 & 0x2FF) ? (m7 | 0xFC00) : (m7);                                                        \
} while(0)


#define ff_ae_ccm_set_Sensor_Cfg(FF_id, sensorMode, yCalcEn, Pattern)                                       \
do {                                                                                                        \
    ff_ae_ccm_set_register(FF_id, FF_AE_CCM_SENSOR_CFG_0_ADDR, ((yCalcEn) << 4) | (sensorMode));            \
    ff_ae_ccm_set_register(FF_id, FF_AE_CCM_SENSOR_CFG_1_ADDR, ((((Pattern[7]) & 0x7) << 28) |              \
                           (((Pattern[6]) & 0x7) << 24) | (((Pattern[5]) & 0x7) << 20) |                    \
                           (((Pattern[4]) & 0x7) << 16) | (((Pattern[3]) & 0x7) << 12) |                    \
                           (((Pattern[2]) & 0x7) << 8)  | (((Pattern[1]) & 0x7) << 4) | ((Pattern[0]) & 0x7))); \
    ff_ae_ccm_set_register(FF_id, FF_AE_CCM_SENSOR_CFG_2_ADDR, ((((Pattern[15]) & 0x7) << 28) |             \
                           (((Pattern[14]) & 0x7) << 24) | (((Pattern[13]) & 0x7) << 20) |                  \
                           (((Pattern[12]) & 0x7) << 16) | (((Pattern[11]) & 0x7) << 12) |                  \
                           (((Pattern[10]) & 0x7) << 8)  | (((Pattern[9]) & 0x7) << 4) | ((Pattern[8]) & 0x7))); \
} while(0)


#endif // _ff_ae_ccm_regs_h


