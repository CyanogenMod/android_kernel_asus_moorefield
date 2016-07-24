/*
 * vied_nci_s2v_defs.h
 *
 *  Created on: May 8, 2014
 *     Authors: mmarkov1
 *              vilic
 */

#ifndef _VIED_NCI_S2V_DEFS_
#define _VIED_NCI_S2V_DEFS_

#include <type_support.h>


/* PSA stream to vector device */
enum vied_nci_s2v_id {
	VIED_NCI_S2V_BAYER1_ID = 0,
	VIED_NCI_S2V_BAYER2_ID,
	VIED_NCI_S2V_RGB_ID,
	VIED_NCI_S2V_YUV1_ID,
	VIED_NCI_S2V_YUV2_ID,
	VIED_NCI_N_S2V_ID,
	VIED_NCI_S2V_UNKNOWN_ID
};

/*
 * The StreamToVec device writes each of the color component vectors to a circular buffer in the VMEM/BAMEM
 * which is defined by a start address, and end address, offset and stride.
 * For instance for a Bayer type VecToStr there is a circular buffer defined for Gr, R, B, and Gb.
 * The offset is used to increment the address from one vector write to the following one.
 * The stride is used to increment the address when an end of line is reached.
 */

#define VIED_NCI_N_S2V_BUFFERS	6
#define S2V_MAX_BUF_CFG_VALUE	(1<<26)-1

struct vied_nci_s2v_buf_s {
    unsigned int start_address;
    unsigned int end_address;
    unsigned int offset;
    unsigned int stride;
};

typedef struct vied_nci_s2v_buf_s vied_nci_s2v_buf_t;


/* Return type for S2V NCI functions */
typedef enum {
    VIED_NCI_S2V_SUCCESS       =  0,
    VIED_NCI_S2V_OPEN_FAIL     = -1,
    VIED_NCI_S2V_LOAD_FAIL     = -2,
    VIED_NCI_S2V_STORE_FAIL    = -3,
    VIED_NCI_S2V_ROUTING_FAIL  = -4
} vied_nci_s2v_err_t;


#endif
