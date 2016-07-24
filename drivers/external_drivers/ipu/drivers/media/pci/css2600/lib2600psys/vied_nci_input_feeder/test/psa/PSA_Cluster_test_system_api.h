#ifndef PSA_CLUSTER_TEST_SYSTEM_API_H
#define PSA_CLUSTER_TEST_SYSTEM_API_H

//#include <hrt/api.h>	// ww2013'30

#include "Acc_WBA_api.h"  // TODELETE
#include "Acc_Anr_api.h"  // TODELETE
#include "PSA_Cluster_api.h"
#include "GA_cio2pixel_cluster_api.h"
#include "GA_pixel2cio_cluster_api.h"
#include "GA_ack2cio_cluster_api.h"

//#define PSA_Cluster_Path        PSA_Cluster_TE_system_bus_PSA_Cfg_Bus
#define PSA_Cluster_Path        PSA_Cluster_TE_PSA_Cluster_Inst


//------------------------------------ PSA Cluster GAs Components -------------------------------------//

// Bayer WBA
#define PSA_WBA_ACB_path 						WBA_ACB_path(Acc_Bayer_WB_path(PSA_Cluster_Path))
#define PSA_WBA_WBA_path 						WBA_WBA_path(Acc_Bayer_WB_path(PSA_Cluster_Path))
#define PSA_WBA_BC_path 						WBA_BC_path(Acc_Bayer_WB_path(PSA_Cluster_Path))
#define PSA_WBA_ACK_CONV_path 					WBA_ACK_CONV_path(Acc_Bayer_WB_path(PSA_Cluster_Path))

// CCM
#define PSA_CCM_ACB_path 						CCM_ACB_path(Acc_CCM_path(PSA_Cluster_Path))
#define PSA_CCM_CCM_path 						CCM_CCM_path(Acc_CCM_path(PSA_Cluster_Path))
#define PSA_CCM_BDC_path 						CCM_BDC_path(Acc_CCM_path(PSA_Cluster_Path))
#define PSA_CCM_ACK_CONV_path 					CCM_ACK_CONV_path(Acc_CCM_path(PSA_Cluster_Path))


// Gamma TM
#define PSA_GTC_ACB_path  						GTC_ACB_path(Acc_GTC_path(PSA_Cluster_Path))
#define PSA_GTC_GTM_path   						GTC_GTM_path(Acc_GTC_path(PSA_Cluster_Path))
#define PSA_GTC_CSC_CDS_path 					GTC_CSC_CDS_path(Acc_GTC_path(PSA_Cluster_Path))
#define PSA_GTC_ACK_CONV_path 					GTC_ACK_CONV_path(Acc_GTC_path(PSA_Cluster_Path))


// DVS Components
// TODO: ackspace DVS
#define PSA_DVS_ACB_path						DVS_ACB_path(Acc_DVS_path(PSA_Cluster_Path))
#define PSA_DVS_YBIN_path						DVS_YBIN_path(Acc_DVS_path(PSA_Cluster_Path))
#define PSA_DVS_DVS_path						DVS_DVS_path(Acc_DVS_path(PSA_Cluster_Path))



// LACE Statistics Components
// TODO: ackspace LACE STAT
#define PSA_LACE_STAT_ACB_path 					LACE_STAT_ACB_path(Acc_LACE_STAT_path(PSA_Cluster_Path))
#define PSA_LACE_STAT_LACE_STAT_path 			LACE_STAT_LACE_STAT_path(Acc_LACE_STAT_path(PSA_Cluster_Path))


// YUV1 GA Componnents
// TODO: ackspace YUV
#define PSA_YUV1_ACB_path           			YUV_ACB_path((Acc_YUV1_path(PSA_Cluster_Path))
#define PSA_YUV1_Splitter_path      			YUV_Splitter_path(Acc_YUV1_path(PSA_Cluster_Path))
#define PSA_YUV1_Y_ee_nr_path       			YUV_Y_ee_nr_path(Acc_YUV1_path(PSA_Cluster_Path))
#define PSA_YUV1_Y_ds_path          			YUV_Y_ds_path(Acc_YUV1_path(PSA_Cluster_Path))
//#define PSA_YUV1_Ch_nr_path         			YUV_Ch_nr_path(Acc_YUV1_path(PSA_Cluster_Path))
#define PSA_YUV1_Collector_path     			YUV_Collector_path(Acc_YUV1_path(PSA_Cluster_Path))


// Bayer Anr Components
#define PSA_ANR_ACB_path 						ANR_ACB_path(Acc_Anr_path(PSA_Cluster_Path))
#define PSA_ANR_SEARCH_path						ANR_SEARCH_path(Acc_Anr_path(PSA_Cluster_Path))
#define PSA_ANR_TRANSFORM_path 					ANR_TRANSFORM_path(Acc_Anr_path(PSA_Cluster_Path))
#define PSA_ANR_STITCH_path 					ANR_STITCH_path(Acc_Anr_path(PSA_Cluster_Path))
#define PSA_ANR_TILE_path 						ANR_TILE_path(Acc_Anr_path(PSA_Cluster_Path))
#define PSA_ANR_ACK_CONV_path 					ANR_ACK_CONV_path(Acc_Anr_path(PSA_Cluster_Path))


// Advanced Demosaic Components
#define PSA_ADVANCED_DEMOSAIC_ACB_path 			ADVANCED_DEMOSAIC_ACB_path(Acc_ADVANCED_DEMOSAIC_path(PSA_Cluster_Path))
#define PSA_ADVANCED_DEMOSAIC_DM_path 			ADVANCED_DEMOSAIC_DM_path(Acc_ADVANCED_DEMOSAIC_path(PSA_Cluster_Path))
#define PSA_ADVANCED_DEMOSAIC_ACK_CONV_path		ADVANCED_DEMOSAIC_ACK_CONV_path(Acc_ADVANCED_DEMOSAIC_path(PSA_Cluster_Path))	// ww2013'38 defining ack memory space


// Stream to Vector paths
#define PSA_str2vec_rgb_path 			   	str2vec_rgb_path(PSA_Cluster_Path)		// str2vec ackmem
#define PSA_str2vec_bayer1_path 		   	str2vec_bayer1_path(PSA_Cluster_Path)
#define PSA_str2vec_bayer2_path 		   	str2vec_bayer2_path(PSA_Cluster_Path)
#define PSA_str2vec_yuv1_path 			   	str2vec_yuv1_path(PSA_Cluster_Path)
#define PSA_str2vec_yuv2_path 			   	str2vec_yuv2_path(PSA_Cluster_Path)
#define PSA_str2vec_ACK_CONV_rgb_path	   	str2vec_ack_conv_rgb_path(PSA_Cluster_Path)
#define PSA_str2vec_ACK_CONV_bayer1_path   	str2vec_ack_conv_bayer1_path(PSA_Cluster_Path)
#define PSA_str2vec_ACK_CONV_bayer2_path   	str2vec_ack_conv_bayer2_path(PSA_Cluster_Path)
#define PSA_str2vec_ACK_CONV_yuv1_path		str2vec_ack_conv_yuv1_path(PSA_Cluster_Path)
#define PSA_str2vec_ACK_CONV_yuv2_path		str2vec_ack_conv_yuv2_path(PSA_Cluster_Path)

// General Purpose Registers
#define PSA_GP_REGS_path                   	Acc_GP_Regs_path(PSA_Cluster_Path)



//------------------------------------ PSA Cluster Send\Receive -------------------------------------//

/*
//PSA - send
#define PSACio2Pixel_path       Cio2Pixel_path(GA_TE_cio2pixel)
#define PSA_Pix_Adr_in          HRTCAT(PSACio2Pixel_path,_data_in)
#define HRT_PSA_snd_pixel(pix)  hrt_fifo_snd(PSA_Pix_Adr_in,pix)

//PSA - receive
#define PSAPixel2Cio_path       Pixel2Cio_path(GA_TE_pixel2cio)
#define PSA_Pix_Adr_out         HRTCAT(PSAPixel2Cio_path,_sl_in)
#define HRT_PSA_rcv_pixel       _hrt_slave_port_load_32_volatile(PSA_Pix_Adr_out, 0)
*/

//PSA_WBA - send
#define PSA_WBACio2Pixel_path       Cio2Pixel_path(GA_TE_cio2pixel)
#define PSA_WBA_Pix_Adr_in          HRTCAT(PSA_WBACio2Pixel_path,_data_in)
#define HRT_PSA_WBA_snd_pixel(pix)  hrt_fifo_snd(PSA_WBA_Pix_Adr_in,pix)

//PSA_WBA - receive
#define PSA_WBAPixel2Cio_path       Pixel2Cio_path(GA_TE_pixel2cio)
#define PSA_WBA_Pix_Adr_out         HRTCAT(PSA_WBAPixel2Cio_path,_sl_in)
#define HRT_PSA_WBA_rcv_pixel       _hrt_slave_port_load_32_volatile(PSA_WBA_Pix_Adr_out, 0)


// Unfinished (ww2013'30 STOPPING POINT)

#endif // PSA_CLUSTER_TEST_SYSTEM_API_H
