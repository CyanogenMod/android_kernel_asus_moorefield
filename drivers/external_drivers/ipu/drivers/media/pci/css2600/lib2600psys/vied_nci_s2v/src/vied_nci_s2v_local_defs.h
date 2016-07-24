/*
 * vied_nci_s2v_local_defs.h
 *
 *  Created on: May 8, 2014
 *     Authors: mmarkov1
 *              vilic
 */

#ifndef _VIED_NCI_S2V_LOCAL_DEFS_H_
#define _VIED_NCI_S2V_LOCAL_DEFS_H_

#define VIED_NCI_S2V_ACK_ADDR_ADDR 0x4
#define VIED_NCI_S2V_ACK_CMD_ADDR  0x0

#define S2V_ACK_ADDR(str2vec_id) (\
    (str2vec_id == VIED_NCI_S2V_BAYER1_ID ) ? HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_1_ADDR : \
    (str2vec_id == VIED_NCI_S2V_BAYER2_ID ) ? HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_2_ADDR : \
    (str2vec_id == VIED_NCI_S2V_RGB_ID )    ? HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_RGB_ADDR : \
    (str2vec_id == VIED_NCI_S2V_YUV1_ID )   ? HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_1_ADDR : \
    (str2vec_id == VIED_NCI_S2V_YUV2_ID )   ? HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_2_ADDR : \
                                              0)


#define S2V_BASE_ADDR(str2vec_id) (\
    (str2vec_id == VIED_NCI_S2V_BAYER1_ID ) ? HOST_AB_PS_A_PSA_STR_2_VEC_BAYER_1_ADDR : \
    (str2vec_id == VIED_NCI_S2V_BAYER2_ID ) ? HOST_AB_PS_A_PSA_STR_2_VEC_BAYER_2_ADDR : \
    (str2vec_id == VIED_NCI_S2V_RGB_ID )    ? HOST_AB_PS_A_PSA_STR_2_VEC_RGB_ADDR : \
    (str2vec_id == VIED_NCI_S2V_YUV1_ID )   ? HOST_AB_PS_A_PSA_STR_2_VEC_YUV_1_ADDR : \
    (str2vec_id == VIED_NCI_S2V_YUV2_ID )   ? HOST_AB_PS_A_PSA_STR_2_VEC_YUV_2_ADDR : \
                                              0)
#endif

