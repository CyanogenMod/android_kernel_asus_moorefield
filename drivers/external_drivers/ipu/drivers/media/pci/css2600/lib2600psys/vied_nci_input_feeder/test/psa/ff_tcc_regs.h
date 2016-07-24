#ifndef _ff_tcc_regs_h
#define _ff_tcc_regs_h

#include "ff_tcc_ccbcr.h"

#define FF_TCC_GEN_CTRL						GEN_CTRL_ADDR		//0x0000
#define FF_TCC_MACC_SECTOR					MACC_A_B_0_ADDR		//0x0004 // the same sector for A_B and C_D
#define FF_TCC_INV_Y_LUT					YINV_LUT_0_ADDR		//0x0084 // Data for Inverse Y estimation LUT
//#define FF_TCC_INV_Y_LUT_LAST_ADD			0x009D
#define FF_TCC_GAIN_PCWL_LUT				MEM_TCC_GAIN_LUT_BASE_ADDR	//0x00A0
//#define FF_TCC_GAIN_PCWL_LUT_LAST_ADD		0x02A1 // Data for Gain Pcwl LUT
#define FF_TCC_R_SQRT_EST_LUT				SQRT_LUT_0_ADDR		//0x02A4 // Data for R Square root estimation LUT


#endif // _ff_tcc_regs_h
