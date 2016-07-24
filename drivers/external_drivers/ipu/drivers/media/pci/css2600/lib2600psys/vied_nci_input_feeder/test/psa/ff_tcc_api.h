#ifndef _ff_tcc_api_h
#define _ff_tcc_api_h

#include "ff_tcc_regs.h"


#define ff_tcc_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_tcc_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)


#define FF_tcc_set_gen_ctrl_reg(FF_id,  Delta, Gamma, GainAccToY, BlendShift, tccEnable) \
	do {												\
	int Mask = 0x001F;						\
	ff_tcc_set_register(FF_id, FF_TCC_GEN_CTRL, ( ((Delta & Mask) << 24) | ((Gamma & Mask) << 16) | (GainAccToY << 4) | (BlendShift << 1) |(tccEnable)));		\
	} while(0);


#define FF_tcc_set_macc_abcd_lut(FF_id, index, a, b , c , d) \
	do {												\
	int Mask = 0x1FFF;						\
	ff_tcc_set_register(FF_id, (FF_TCC_MACC_SECTOR + (2*index*0x4)), (((b & Mask) << 16) | (a & Mask )));		\
	ff_tcc_set_register(FF_id, (FF_TCC_MACC_SECTOR + (((2*index+1)*0x4 ))), (((d & Mask) << 16) | (c & Mask )));		\
	} while(0);


#define FF_tcc_set_inv_y_est_lut(FF_id, index, lowWord ,highWord) \
	ff_tcc_set_register(FF_id, (FF_TCC_INV_Y_LUT + (index*0x4)), ((highWord << 16) | (lowWord )));

#define FF_tcc_set_r_sqrt_est_lut(FF_id, index, lowWord ,highWord) \
	ff_tcc_set_register(FF_id, (FF_TCC_R_SQRT_EST_LUT + (index*0x4)), ((highWord << 16) | (lowWord )));

#define FF_tcc_set_gain_pcwl_lut(FF_id, index, lowWord ,highWord) \
	ff_tcc_set_register(FF_id, (FF_TCC_GAIN_PCWL_LUT + (index*0x4)), ((highWord << 16) | (lowWord )));

#endif // _ff_tcc_api_h
