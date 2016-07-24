#ifndef _PSA_CLUSTER_API_H
#define _PSA_CLUSTER_API_H

#include <hrt/api.h>

#include "Acc_WBA_api.h"
#include "Acc_Ccm_api.h"
#include "Acc_GTC_api.h"
#include "Acc_Dvs_api.h"
#include "Acc_Lace_Stat_api.h"
#include "Acc_Yuv_V2_api.h"
#include "Acc_Anr_api.h"
#include "Acc_Dm_api.h"
#include "Acc_Scaler_api.h"



// Vector to Stream path
#define vec2str_yuv2_path(PSA_Cluster_id) 		HRTCAT(PSA_Cluster_id,_vec2str_yuv2)


// Bayer WBA
#define Acc_Bayer_WB_path(PSA_Cluster_id) 		HRTCAT(PSA_Cluster_id,_Bayer_WB)

// CCM
#define Acc_CCM_path(PSA_Cluster_id) 			HRTCAT(PSA_Cluster_id,_CCM)

// Gamma TM
#define Acc_GTC_path(PSA_Cluster_id) 			HRTCAT(PSA_Cluster_id,_GTC)

// DVS
#define Acc_DVS_path(PSA_Cluster_id)   			HRTCAT(PSA_Cluster_id,_DVS)

// LACE Statistics
#define Acc_LACE_STAT_path(PSA_Cluster_id)  	HRTCAT(PSA_Cluster_id,_Lace_Stat)

// YUV1
#define Acc_YUV1_path(PSA_Cluster_id) 			HRTCAT(PSA_Cluster_id,_YUV1_Processing)

// Bayer Anr
#define Acc_Anr_path(PSA_Cluster_id) 			HRTCAT(PSA_Cluster_id,_Bayer_ANR)

// Advanced Demosaic
#define Acc_ADVANCED_DEMOSAIC_path(PSA_Cluster_id)	HRTCAT(PSA_Cluster_id,_Advanced_Demosaic)

// Stream to Vector paths
#define str2vec_rgb_path(PSA_Cluster_id) 			HRTCAT(PSA_Cluster_id,_str2vec_rgb)
#define str2vec_bayer1_path(PSA_Cluster_id) 		HRTCAT(PSA_Cluster_id,_str2vec_bayer1)
#define str2vec_bayer2_path(PSA_Cluster_id) 		HRTCAT(PSA_Cluster_id,_str2vec_bayer2)
#define str2vec_yuv1_path(PSA_Cluster_id) 			HRTCAT(PSA_Cluster_id,_str2vec_yuv1)
#define str2vec_yuv2_path(PSA_Cluster_id) 			HRTCAT(PSA_Cluster_id,_str2vec_yuv2)
#define str2vec_ack_conv_rgb_path(PSA_Cluster_id) 	HRTCAT(PSA_Cluster_id,_str2vec_ack_conv_rgb)
#define str2vec_ack_conv_bayer1_path(PSA_Cluster_id) HRTCAT(PSA_Cluster_id,_str2vec_ack_conv_bayer1)
#define str2vec_ack_conv_bayer2_path(PSA_Cluster_id) HRTCAT(PSA_Cluster_id,_str2vec_ack_conv_bayer2)
#define str2vec_ack_conv_yuv1_path(PSA_Cluster_id) 	HRTCAT(PSA_Cluster_id,_str2vec_ack_conv_yuv1)
#define str2vec_ack_conv_yuv2_path(PSA_Cluster_id) 	HRTCAT(PSA_Cluster_id,_str2vec_ack_conv_yuv2)



//General Purpose registers related definitions
#define Acc_GP_Regs_port(PSA_GP_REGS_id)                HRTCAT(PSA_GP_REGS_id,_slv_in)

#define Acc_GP_Regs_set_register(PSA_GP_REGS_id, addr, val) \
    _hrt_slave_port_store_32_volatile(Acc_GP_Regs_port(PSA_GP_REGS_id), addr, (val))

#define Acc_GP_Regs_get_register(PSA_GP_REGS_id, addr) \
    _hrt_slave_port_load_32_volatile(Acc_GP_Regs_port(PSA_GP_REGS_id), addr)

#define Acc_GP_Regs_path(PSA_Cluster_id)               HRTCAT(PSA_Cluster_id,_acc_gp_reg)

#endif
