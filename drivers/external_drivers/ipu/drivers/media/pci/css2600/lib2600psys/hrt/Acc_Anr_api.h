#ifndef _ACC_ANR_API_H
#define _ACC_ANR_API_H

#define ANR_ACB_path(Acc_id)			HRTCAT(Acc_id,_ACB)
#define ANR_SEARCH_path(Acc_id)			HRTCAT(Acc_id,_SEARCH)
#define ANR_TRANSFORM_path(Acc_id)		HRTCAT(Acc_id,_TRANSFORM)
#define ANR_STITCH_path(Acc_id)			HRTCAT(Acc_id,_STITCH)
#define ANR_TILE_path(Acc_id)	        HRTCAT(Acc_id,_TILE)
#define ANR_ACK_CONV_path(Acc_id)   	HRTCAT(Acc_id,_AckConv) 	// ww2013'39 defining ack memory space

#endif //_ACC_ANR_API_H
