#ifndef _FF_GDDPC_API_H
#define _FF_GDDPC_API_H
#include "ff_gddpc_ccbcr.h"


#define ff_gddpc_set_register(FF_GDDPC_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_GDDPC_id + addr, (val))

#define ff_gddpc_get_register(FF_GDDPC_id, addr) \
	vied_subsystem_load_32(psys0, FF_GDDPC_id + addr)


#define ff_gddpc_set_gd_controls(FF_GDDPC_id, GlobalEn, GdcEn, GreenPos, InvScale, LcdeDetailsPres, SensorMode)   \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_GDC_CONTROL_ADDR, ((SensorMode) << 20) | ((LcdeDetailsPres) << 8) | ((InvScale) << 4) | ((GreenPos) << 2) | ((GdcEn) << 1) | (GlobalEn))

#define ff_gddpc_set_dpc_controls(FF_GDDPC_id, DpcEn, DynamicEn, StaticEn, AfEn)   \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_CONTROL_ADDR, ((AfEn) << 3) | ((StaticEn) << 2) | ((DynamicEn) << 1) | (DpcEn))

#define ff_gddpc_set_static_lut_limits(FF_GDDPC_id, LimitSet0, LimitSet1)   \
do {                                                                                            \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_STATIC_LUT_LIMIT_SET0_ADDR, (LimitSet0));   \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_STATIC_LUT_LIMIT_SET1_ADDR, (LimitSet1));   \
}                                                                                               \
while (0)

#define ff_gddpc_set_static_lut_entry(FF_GDDPC_id, CfgSet, Entry, RowOffset, Col, IsDefect00, IsDefect01, IsDefect10, IsDefect11)\
do {                                                                                            \
    int addrBase[2] = {MEM_DPLUT_SET0_BASE_ADDR, MEM_DPLUT_SET1_BASE_ADDR};                     \
    ff_gddpc_set_register(FF_GDDPC_id, addrBase[(CfgSet)] + ((Entry) << 2), ((IsDefect11) << 23) | ((IsDefect10) << 22) | ((IsDefect01) << 21) | ((IsDefect00) << 20) | ((RowOffset) << 13) | (Col));  \
}                                                                                               \
while (0)

#define ff_gddpc_set_static_lut_entry(FF_GDDPC_id, CfgSet, Entry, RowOffset, Col, IsDefect00, IsDefect01, IsDefect10, IsDefect11)\
do {                                                                                            \
    int addrBase[2] = {MEM_DPLUT_SET0_BASE_ADDR, MEM_DPLUT_SET1_BASE_ADDR};                     \
    ff_gddpc_set_register(FF_GDDPC_id, addrBase[(CfgSet)] + ((Entry) << 2), ((IsDefect11) << 23) | ((IsDefect10) << 22) | ((IsDefect01) << 21) | ((IsDefect00) << 20) | ((RowOffset) << 13) | (Col));  \
}                                                                                               \
while (0)

#define ff_gddpc_get_static_lut_limits(FF_GDDPC_id, LimitSet0, LimitSet1)                       \
do {                                                                                            \
    unsigned int val = ff_gddpc_get_register(FF_GDDPC_id, FF_GDDPC_DPC_STATIC_LUT_LIMIT_ADDR);  \
    LimitSet0 = val & 0x000003FF;                                                               \
    LimitSet1 = (val >> 16) & 0x000003FF;                                                       \
}                                                                                               \
while (0)


#define ff_gddpc_get_af_lut_limits(FF_GDDPC_id, LimitSet0, LimitSet1)                       \
do {                                                                                        \
    unsigned int val = ff_gddpc_get_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_LUT_LIMIT_ADDR);  \
    LimitSet0 = val & 0x000003FF;                                                           \
    LimitSet1 = (val >> 16) & 0x000003FF;                                                   \
}                                                                                           \
while (0)


#define ff_gddpc_set_af_grid_cfg(FF_GDDPC_id, XOffsetArr, YOffsetArr, XPeriodArr, YPeriodArr, GridEnMask)               \
do {                                                                                                                    \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET0_OFFSET_X_ADDR, ((XOffsetArr[1]) << 16) | (XOffsetArr[0]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET0_OFFSET_Y_ADDR, ((YOffsetArr[1]) << 16) | (YOffsetArr[0]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET0_PERIOD_X_ADDR, ((XPeriodArr[1]) << 16) | (XPeriodArr[0]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET0_PERIOD_Y_ADDR, ((YPeriodArr[1]) << 16) | (YPeriodArr[0]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET1_OFFSET_X_ADDR, ((XOffsetArr[3]) << 16) | (XOffsetArr[2]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET1_OFFSET_Y_ADDR, ((YOffsetArr[3]) << 16) | (YOffsetArr[2]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET1_PERIOD_X_ADDR, ((XPeriodArr[3]) << 16) | (XPeriodArr[2]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET1_PERIOD_Y_ADDR, ((YPeriodArr[3]) << 16) | (YPeriodArr[2]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET2_OFFSET_X_ADDR, ((XOffsetArr[5]) << 16) | (XOffsetArr[4]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET2_OFFSET_Y_ADDR, ((YOffsetArr[5]) << 16) | (YOffsetArr[4]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET2_PERIOD_X_ADDR, ((XPeriodArr[5]) << 16) | (XPeriodArr[4]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET2_PERIOD_Y_ADDR, ((YPeriodArr[5]) << 16) | (YPeriodArr[4]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET3_OFFSET_X_ADDR, ((XOffsetArr[7]) << 16) | (XOffsetArr[6]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET3_OFFSET_Y_ADDR, ((YOffsetArr[7]) << 16) | (YOffsetArr[6]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET3_PERIOD_X_ADDR, ((XPeriodArr[7]) << 16) | (XPeriodArr[6]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_AF_SET3_PERIOD_Y_ADDR, ((YPeriodArr[7]) << 16) | (YPeriodArr[6]));  \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_AF_GRID_ENABLE_ADDR, ((GridEnMask) & 0xFF));                            \
}                                                                                                                       \
while (0)


#define ff_gddpc_set_hdr_factors(FF_GDDPC_id, HdrArr)                                                                   \
do {                                                                                                                    \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET0_ADDR, ((HdrArr[1]) << 16) | (HdrArr[0]));           \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET1_ADDR, ((HdrArr[3]) << 16) | (HdrArr[2]));           \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET2_ADDR, ((HdrArr[5]) << 16) | (HdrArr[4]));           \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET3_ADDR, ((HdrArr[7]) << 16) | (HdrArr[6]));           \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET4_ADDR, ((HdrArr[9]) << 16) | (HdrArr[8]));           \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET5_ADDR, ((HdrArr[11]) << 16) | (HdrArr[10]));         \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET6_ADDR, ((HdrArr[13]) << 16) | (HdrArr[12]));         \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_FACTOR_SET7_ADDR, ((HdrArr[15]) << 16) | (HdrArr[14]));         \
}                                                                                                                       \
while (0)

#define ff_gddpc_set_inv_hdr_factors(FF_GDDPC_id, InvHdrArr)                                                                \
do {                                                                                                                        \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET0_ADDR, ((InvHdrArr[1]) << 16) | (InvHdrArr[0]));     \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET1_ADDR, ((InvHdrArr[3]) << 16) | (InvHdrArr[2]));     \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET2_ADDR, ((InvHdrArr[5]) << 16) | (InvHdrArr[4]));     \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET3_ADDR, ((InvHdrArr[7]) << 16) | (InvHdrArr[6]));     \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET4_ADDR, ((InvHdrArr[9]) << 16) | (InvHdrArr[8]));     \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET5_ADDR, ((InvHdrArr[11]) << 16) | (InvHdrArr[10]));   \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET6_ADDR, ((InvHdrArr[13]) << 16) | (InvHdrArr[12]));   \
    ff_gddpc_set_register(FF_GDDPC_id, FF_GDDPC_DPC_HDR_INV_FACTOR_SET7_ADDR, ((InvHdrArr[15]) << 16) | (InvHdrArr[14]));   \
}                                                                                                                           \
while (0)

#define ff_gddpc_set_dpc_control_unit1(FF_GDDPC_id, Index, LbEstType, UbEstType, PelEstType, PdEstType, LbMask, UbMask) \
do {                                                                                                                    \
    int value = ((LbMask) << 10) | ((PdEstType) << 8) | ((PelEstType) << 4) | ((UbEstType) << 2) | (LbEstType);         \
    int baseAddr = FF_GDDPC_CFGTAB_EID0_BMASK0_ADDR;                                                                    \
    int setSize = FF_GDDPC_CFGTAB_EID1_BMASK0_ADDR - FF_GDDPC_CFGTAB_EID0_BMASK0_ADDR;                                  \
    baseAddr += (setSize * Index);                                                                                      \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr, value);                                                                \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 4, (UbMask));                                                         \
}                                                                                                                       \
while (0)

#define ff_gddpc_set_dpc_control_unit2(FF_GDDPC_id, Index, MPos0Arr, MPos1Arr, MNegArr, PdArr)                                                       \
do {                                                                                                                                                 \
    int baseAddr = FF_GDDPC_CFGTAB_EID0_PMASK0_ADDR;                                                                                                 \
    int setSize = FF_GDDPC_CFGTAB_EID1_PMASK0_ADDR - FF_GDDPC_CFGTAB_EID0_PMASK0_ADDR;                                                               \
    int value = ((MNegArr[1]) << 25) | ((MPos1Arr[1]) << 20) | ((MPos0Arr[1]) << 15) | ((MNegArr[0]) << 10) | ((MPos1Arr[0]) << 5) | (MPos0Arr[0]);  \
    baseAddr += (setSize * Index);                                                                                                                   \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr, value);                                                                                             \
    value = ((MNegArr[3]) << 25) | ((MPos1Arr[3]) << 20) | ((MPos0Arr[3]) << 15) | ((MNegArr[2]) << 10) | ((MPos1Arr[2]) << 5) | (MPos0Arr[2]);      \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x4, value);                                                                                       \
    value = ((MNegArr[5]) << 25) | ((MPos1Arr[5]) << 20) | ((MPos0Arr[5]) << 15) | ((MNegArr[4]) << 10) | ((MPos1Arr[4]) << 5) | (MPos0Arr[4]);      \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x8, value);                                                                                       \
    value = ((MNegArr[7]) << 25) | ((MPos1Arr[7]) << 20) | ((MPos0Arr[7]) << 15) | ((MNegArr[6]) << 10) | ((MPos1Arr[6]) << 5) | (MPos0Arr[6]);      \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0xC, value);                                                                                       \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x10, ((PdArr[2]) << 10) | ((PdArr[1]) << 5) | (PdArr[0]));                                        \
}                                                                                                                                                    \
while (0)

#define ff_gddpc_set_gddpc_configUnit(FF_GDDPC_id, setNum, xarr, Aarr, Barr)                                           \
do {                                                                                                                   \
    int cfgUnitSetSize = FF_GDDPC_CFGUNT_ID1_FIXEDX0_ADDR - FF_GDDPC_CFGUNT_ID0_FIXEDX0_ADDR;                          \
    int baseAddr = (setNum * cfgUnitSetSize) + FF_GDDPC_CFGUNT_ID0_FIXEDX0_ADDR;                                       \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr       , (xarr[3] << 24) | (xarr[2] << 16) | (xarr[1] << 8) | xarr[0]); \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x04, (xarr[7] << 24) | (xarr[6] << 16) | (xarr[5] << 8) | xarr[4]); \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x08, ((Aarr[1]& 0xFFFF) << 16) | (Aarr[0] & 0xFFFF));               \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x0C, ((Aarr[3]& 0xFFFF) << 16) | (Aarr[2] & 0xFFFF));               \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x10, ((Aarr[5]& 0xFFFF) << 16) | (Aarr[4] & 0xFFFF));               \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x14,                             (Aarr[6] & 0xFFFF));               \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x18, ((Barr[1]& 0xFFFF) << 16) | (Barr[0] & 0xFFFF));               \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x1C, ((Barr[3]& 0xFFFF) << 16) | (Barr[2] & 0xFFFF));               \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x20, ((Barr[5]& 0xFFFF) << 16) | (Barr[4] & 0xFFFF));               \
    ff_gddpc_set_register(FF_GDDPC_id, baseAddr + 0x24,                             (Barr[6] & 0xFFFF));               \
}                                                                                                                      \
while (0)



#endif // _FF_GDDPC_API_H
