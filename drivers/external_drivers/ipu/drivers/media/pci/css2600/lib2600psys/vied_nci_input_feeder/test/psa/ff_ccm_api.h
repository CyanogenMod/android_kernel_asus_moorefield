#ifndef _FF_CCM_API_H
#define _FF_CCM_API_H
#include "ff_ccm_regs.h"

#define FF_ccm_set_register(FF_CCM_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_CCM_id + addr, (val))

#define FF_ccm_get_register(FF_CCM_id, addr) \
  vied_subsystem_load_32(psys0, FF_CCM_id + addr)


#define FF_ccm_enable(FF_CCM_id)                                \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_GEN_CTRL, 0x01)

#define FF_ccm_disable(FF_CCM_id)                               \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_GEN_CTRL, 0x00)

#define FF_ccm_set_Coeff_Matrix(FF_CCM_id, m11, m12, m13, m21, m22, m23, m31, m32, m33, Or, Og, Ob) \
do {                                                                                                \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_MAT_COEFF_0, ((m12 & 0x7FFF) << 16) | (m11 & 0x7FFF));    \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_MAT_COEFF_1, ((Or & 0xFFFF) << 16)  | (m13 & 0x7FFF));    \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_MAT_COEFF_2, ((m22 & 0x7FFF) << 16) | (m21 & 0xFFFF));    \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_MAT_COEFF_3, ((Og & 0xFFFF) << 16)  | (m23 & 0x7FFF));    \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_MAT_COEFF_4, ((m32 & 0x7FFF) << 16) | (m31 & 0x7FFF));    \
    FF_ccm_set_register(FF_CCM_id, FF_CCM_MAT_COEFF_5, ((Ob & 0xFFFF) << 16)  | (m33 & 0x7FFF));    \
} while(0);


#define FF_ccm_get_Coeff_Matrix(FF_CCM_id, m11, m12, m13, m21, m22, m23, m31, m32, m33, Or, Og, Ob) \
do {                                                                                                \
    unsigned int reg0 = FF_ccm_get_register(FF_CCM_id, FF_CCM_MAT_COEFF_0);                         \
    unsigned int reg1 = FF_ccm_get_register(FF_CCM_id, FF_CCM_MAT_COEFF_1);                         \
    unsigned int reg2 = FF_ccm_get_register(FF_CCM_id, FF_CCM_MAT_COEFF_2);                         \
    unsigned int reg3 = FF_ccm_get_register(FF_CCM_id, FF_CCM_MAT_COEFF_3);                         \
    unsigned int reg4 = FF_ccm_get_register(FF_CCM_id, FF_CCM_MAT_COEFF_4);                         \
    unsigned int reg5 = FF_ccm_get_register(FF_CCM_id, FF_CCM_MAT_COEFF_5);                         \
    m11 = reg0 & 0x7FFF;                                                                            \
    m12 = (reg0 >> 16) & 0x7FFF;                                                                    \
    m13 = reg1 & 0x7FFF;                                                                            \
    Or = (reg1 >> 16) & 0xFFFF;                                                                     \
    m21 = reg2 & 0x7FFF;                                                                            \
    m22 = (reg2 >> 16) & 0x7FFF;                                                                    \
    m23 = reg3 & 0x7FFF;                                                                            \
    Og = (reg3 >> 16) & 0xFFFF;                                                                     \
    m31 = reg4 & 0x7FFF;                                                                            \
    m32 = (reg4 >> 16) & 0x7FFF;                                                                    \
    m33 = reg5 & 0x7FFF;                                                                            \
    Ob = (reg5 >> 16) & 0xFFFF;                                                                     \
} while(0);

#endif // _FF_CCM_API_H
