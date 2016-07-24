#ifndef YUV_V2_API_H
#define YUV_V2_API_H

#include <hrt/api.h>

#define YUV_ACB_path(Acc_Cluster_id)		HRTCAT(Acc_Cluster_id,_ACB)
#define YUV_GTM_path(Acc_Cluster_id)        HRTCAT(Acc_Cluster_id,_GTM)
#define YUV_CSC_path(Acc_Cluster_id)        HRTCAT(Acc_Cluster_id,_CSC)
#define YUV_ACK_CONV_path(Acc_Cluster_id)   HRTCAT(Acc_Cluster_id,_AckConv)
#define YUV_BDC_path(Acc_Cluster_id)        HRTCAT(Acc_Cluster_id,_BDC)
#define YUV_TCC_path(Acc_Cluster_id)        HRTCAT(Acc_Cluster_id,_TCC)

#define YUV_Splitter_path(Acc_Cluster_id)	HRTCAT(Acc_Cluster_id,_YUV_SPLITTER)
#define YUV_Y_ee_nr_path(Acc_Cluster_id)	HRTCAT(Acc_Cluster_id,_Y_EE_NR)
#define YUV1_Iefd_path(Acc_Cluster_id)	    HRTCAT(Acc_Cluster_id,_IEFD)
#define YUV_Y_ds_path(Acc_Cluster_id)		HRTCAT(Acc_Cluster_id,_YDS)
//#define YUV_Ch_nr_path(Acc_Cluster_id)		HRTCAT(Acc_Cluster_id,_CHNR)
#define YUV_Collector_path(Acc_Cluster_id)	HRTCAT(Acc_Cluster_id,_YUV_COLLECTOR)


#endif //YUV_V2_API_H

