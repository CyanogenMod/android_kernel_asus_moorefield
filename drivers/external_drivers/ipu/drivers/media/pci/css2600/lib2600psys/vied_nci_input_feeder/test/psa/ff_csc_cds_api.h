#ifndef _FF_CSC_CDS_API_H
#define _FF_CSC_CDS_API_H
#include <hrt/api.h>
#include "ff_csc_cds_ccbcr.h"

#define FF_csc_cds_set_register(FF_CSC_CDS_id, addr, val) \
  vied_subsystem_store_32(psys0, FF_CSC_CDS_id + addr, (val))

#define FF_csc_cds_get_register(FF_CSC_CDS_id, addr) \
  vied_subsystem_load_32(psys0, FF_CSC_CDS_id + addr)


#define FF_csc_cds_enable(FF_CSC_CDS_id)								\
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_CDS_GEN_CTRL, 0x01)

#define FF_csc_cds_disable(FF_CSC_CDS_id)								\
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_CDS_GEN_CTRL, 0x00)




#define FF_csc_cds_set_Coeff_Matrix(FF_CSC_CDS_id, m11, m12, m13, m21, m22, m23, m31, m32, m33, b1, b2, b3) \
do {												\
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_0_ADDR, ((m12 & 0x7FFF)<< 16) | (m11 & 0x7FFF));		\
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_1_ADDR, ( (b1 & 0x3FFF)<< 16) | (m13 & 0x7FFF) );	        \
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_2_ADDR, ((m22 & 0x7FFF)<< 16) | (m21 & 0x7FFF));		\
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_3_ADDR, ( (b2 & 0x3FFF)<< 16) | (m23 & 0x7FFF) );	        \
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_4_ADDR, ((m32 & 0x7FFF)<< 16) | (m31 & 0x7FFF));		\
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_5_ADDR, ( (b3 & 0x3FFF)<< 16) | (m33 & 0x7FFF) );	        \
} while(0);



#define FF_csc_cds_get_Coeff_Matrix(FF_CSC_CDS_id, m11, m12, m13, m21, m22, m23, m31, m32, m33, Or, Og, Ob) \
do {												\
	unsigned int reg0 = FF_csc_cds_get_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_0_ADDR);			\
	unsigned int reg1 = FF_csc_cds_get_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_1_ADDR);			\
	unsigned int reg2 = FF_csc_cds_get_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_2_ADDR);			\
	unsigned int reg3 = FF_csc_cds_get_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_3_ADDR);			\
	unsigned int reg4 = FF_csc_cds_get_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_4_ADDR);			\
	unsigned int reg5 = FF_csc_cds_get_register(FF_CSC_CDS_id, FF_CSC_MAT_COEFF_5_ADDR);			\
	m11 = reg0 & 0xFFFF;									\
	m12 = reg0 >> 16;									\
	m13 = reg1 & 0xFFFF;									\
	Or = (reg1 >> 16) & 0x3FFF;								\
	m21 = reg2 & 0xFFFF;									\
	m22 = reg2 >> 16;									\
	m23 = reg3 & 0xFFFF;									\
	Og = (reg3 >> 16) & 0x3FFF;								\
	m31 = reg4 & 0xFFFF;									\
	m32 = reg4 >> 16;									\
	m33 = reg5 & 0xFFFF;`									\
	Ob = (reg5 >> 16) & 0x3FFF;								\
} while(0);



#define FF_csc_cds_set_ds_Coeff_Matrix( FF_CSC_CDS_id,  ds00,  ds01,  ds02,  ds03, \
				        ds10,  ds11,  ds12,  ds13 ,  norm ,  enable ,  output) \
do { \
	short mask1 = 0x0003;		\
	short maskNorm = 0x001F;		\
	short mask2 = 0x0001;		\
	int reg = ( ( (output & mask2) << 25 )  | ( (enable & mask2) << 24 ) | ( (norm & maskNorm) << 16 ) | ((ds13 & mask1) << 14 ) | \
		      (	(ds12 & mask1) << 12 )    | ( (ds11 & mask1) << 10  )  | ( (ds10 & mask1) << 8 )     | ( (ds03 & mask1) << 6 ) | \
		      ( (ds02 & mask1) << 4  )    | ( (ds01 & mask1) << 2  )   |  (ds00 & mask1) ) ; \
	FF_csc_cds_set_register(FF_CSC_CDS_id, FF_UVDS_PARAM_ADDR , reg );		\
} while(0);	\

  /*
void FF_csc_cds_enable (int FF_CSC_CDS_id);
void FF_csc_cds_disable (int FF_CSC_CDS_id);
void FF_csc_cds_set_Coeff_Matrix(int FF_CSC_CDS_id, int m11, int m12, int m13, int m21, int m22, int m23, int m31, int m32, int m33, int b1, int b2, int b3);
void FF_csc_cds_get_Coeff_Matrix(int FF_CSC_CDS_id, int* m11, int* m12, int* m13,
					int* m21, int* m22, int* m23, int* m31, int* m32,
					int* m33, int* b1, int* b2, int* b3);

void FF_csc_cds_set_ds_Coeff_Matrix(int FF_CSC_CDS_id, int ds00, int ds01, int ds02, int ds03, int ds10, int ds11, int ds12, int ds31 , int ds_nf , int enable , int output);

*/



#endif // _FF_CSC_CDS_API_H


