/*
 * vied_nci_s2v_device_address.h
 *
 *  Created on: May 8, 2014
 *     Authors: tversteg
 *              vilic
 */

#ifndef VIED_NCI_S2V_DEVICE_ADDRESS_H_
#define VIED_NCI_S2V_DEVICE_ADDRESS_H_

#include "vied_nci_s2v_defs.h"

/*
 * NOTE:
 * Temporary cell address mapping
 * !!!!
 */

static const unsigned int STR2VEC_IP_BASE[VIED_NCI_N_S2V_ID] = {
	0xD7600, /* str2vec_bayer1_sl_cfg */
	0xD7700, /* str2vec_bayer2_sl_cfg*/
	0xD7A00, /* str2vec_rgb_sl_cfg */
	0xD7800, /* str2vec_yuv1_sl_cfg */
	0xD7900, /* str2vec_yuv2_sl_cfg */
};

static const unsigned int STR2VEC_IP_ACK[VIED_NCI_N_S2V_ID] = {
	0xD7D00, /* str2vec_ack_conv_bayer1_crq_in */
	0xD7D10, /* str2vec_ack_conv_bayer2_crq_in */
	0xD7D40, /* str2vec_ack_conv_rgb_crq_in */
	0xD7D20, /* str2vec_ack_conv_yuv1_crq_in */
	0xD7D30, /* str2vec_ack_conv_yuv2_crq_in */
};

#endif /* VIED_NCI_S2V_DEVICE_ADDRESS_H_ */

