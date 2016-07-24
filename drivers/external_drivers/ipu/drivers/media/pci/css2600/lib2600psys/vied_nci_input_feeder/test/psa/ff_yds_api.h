#ifndef _ff_yds_api_h
#define _ff_yds_api_h


#include "ff_yds_regs.h"



#define ff_yds_set_register(FF_id, addr, val) \
	vied_subsystem_store_32(psys0, FF_id + addr, (val))

#define ff_yds_get_register(FF_id, addr) \
	vied_subsystem_load_32(psys0, FF_id + addr)


#define FF_yds_set_ds_Coeff_Matrix( FF_id,  ds00,  ds01,  ds02,  ds03, \
	ds10,  ds11,  ds12,  ds13 ,  norm ,   output) \
	do { \
	short mask1 = 0x0003;		\
	short maskNorm = 0x001F;		\
	short mask2 = 0x0001;		\
	int reg = ( ( (output & mask2) << 25 )  | ( (norm & maskNorm) << 16 ) | ((ds13 & mask1) << 14 ) | \
	(	(ds12 & mask1) << 12 )    | ( (ds11 & mask1) << 10  )  | ( (ds10 & mask1) << 8 )     | ( (ds03 & mask1) << 6 ) | \
	( (ds02 & mask1) << 4  )    | ( (ds01 & mask1) << 2  )   |  (ds00 & mask1) ) ; \
	ff_yds_set_register(FF_id, FF_YDS_PARAM , reg );		\
	} while(0);	\






#endif /* _ff_yds_api_h */

