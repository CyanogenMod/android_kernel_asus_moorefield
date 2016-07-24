/*
 * vied_nci_acc_isys_device_address.h
 *
 *  Created on: April 9, 2014
 *      Author: vilic
 *              mmarkov1
 */

#ifndef VIED_NCI_ACC_ISYS_DEVICE_ADDRESS_H_
#define VIED_NCI_ACC_ISYS_DEVICE_ADDRESS_H_

#include "vied_nci_acc_isys_defs.h"

/*
 * NOTE:
 * Temporary cell address mapping of the ISYS ISA accelerators!
 * !!!!
 * Table entries MUST be inline with associated enum values!
 * !!!!
 */
static const unsigned int ACCELERATOR_IP_ACB[VIED_NCI_N_ACC_ID] = {
	0xA6600, /* accelerator ip ICA */
	0xA4D00, /* accelerator ip LSC */
	0xA0000, /* accelerator ip DPC */
	0xAA000, /* accelerator ip Scaler */
	0xAEC00, /* accelerator ip AWB */
	0xAD800, /* accelerator ip AE */
	0xAF500, /* accelerator ip AF_AWB_FR */
};

static const unsigned int ACCELERATOR_IP_ACK[VIED_NCI_N_ACC_ID] = {
	0xA6700, /* accelerator ip ICA */
	0xA4E00, /* accelerator ip LSC */
	0xA0050, /* accelerator ip DPC */
	0xAA800, /* accelerator ip Scaler */
	0xAEE00, /* accelerator ip AWB */
	0xADC00, /* accelerator ip AE */
	0xAF600, /* accelerator ip AF_AWB_FR */
};


static const unsigned int ACCELERATOR_IP_FF[VIED_NCI_N_ACC_FF] = {
	0xA6800, /* fixed function ip ICA_INL */
	0xA5000, /* fixed function ip ICA_GBL */
	0xA5D00, /* fixed function ip ICA_PCLN */
	0xA4000, /* fixed function ip LSC_LSC */
	0xA0060, /* fixed function ip DPC_GDDPC */
	0xA7000, /* fixed function ip Scaler_SCALER */
	0xAE000, /* fixed function ip AWB_AWRG */
	0xADE00, /* fixed function ip AE_CCM */
	0xAB000, /* fixed function ip AE_WGHT_HIST */
	0xAF000, /* fixed function ip AF_AWB_FR_AF_AWB_FR_GRD */
};

static const unsigned int ACCELERATOR_IP_FF_PARAMS_BASE[VIED_NCI_N_ACC_FF] = {
	0x0, /* fixed function ip ICA_INL */
	0x0, /* fixed function ip ICA_GBL */
	0x0, /* fixed function ip ICA_PCLN */
	0x0, /* fixed function ip LSC_LSC */
	0x0, /* fixed function ip DPC_GDDPC */
	0x0, /* fixed function ip Scaler_SCALER */
	0x0, /* fixed function ip AWB_AWRG */
	0x0, /* fixed function ip AE_CCM */
	0x0, /* fixed function ip AE_WGHT_HIST */
	0x0, /* fixed function ip AF_AWB_FR_AF_AWB_FR_GRD */
};

static const unsigned int ACCELERATOR_IP_FF_PARAMS_END[VIED_NCI_N_ACC_FF] = {
	0x208, /* fixed function ip ICA_INL */
	0x82C, /* fixed function ip ICA_GBL */
	0x824, /* fixed function ip ICA_PCLN */
	0xC0C, /* fixed function ip LSC_LSC */
	0x0, /* fixed function ip DPC_GDDPC */
	0x0, /* fixed function ip Scaler_SCALER */
	0x0, /* fixed function ip AWB_AWRG */
	0x0, /* fixed function ip AE_CCM */
	0x0, /* fixed function ip AE_WGHT_HIST */
	0x0, /* fixed function ip AF_AWB_FR_AF_AWB_FR_GRD */
};

static const unsigned int LSC_PARAM_SET_BASE_ADDRESS[VIED_NCI_N_LSC_PARAM_SET] = {
	0x00000,
	0x00100,
	0x00200,
	0x00300,
	0x00400,
	0x00500,
	0x00600,
	0x00700,
	0x00800,
	0x00900,
	0x00a00,
	0x00b00,
};

#define ACCELERATOR_IP_FF_PARAM_SET(ff_id, set_id) (\
	(ff_id == VIED_NCI_LSC_LSC_FF) ? LSC_PARAM_SET_BASE_ADDRESS[set_id] : \
					 0)

static const unsigned int LSC_PARAM_SET_END_ADDRESS[VIED_NCI_N_LSC_PARAM_SET] = {
	0x00100,
	0x00200,
	0x00300,
	0x00400,
	0x00500,
	0x00600,
	0x00700,
	0x00800,
	0x00900,
	0x00a00,
	0x00b00,
	0x00c00,
};

#define ACCELERATOR_IP_FF_PARAM_SET_END(ff_id, set_id) (\
	(ff_id == VIED_NCI_LSC_LSC_FF) ? LSC_PARAM_SET_END_ADDRESS[set_id] : \
					 0)

#define ACCELERATOR_IP_FF_META_DATA(ff_id, set_id) (\
					 0)

#define ISA_GP_REGS_ADDR 0xF800

// ISA GP regs address space
#define ISA_FRAME_SIZE_ADDR         0x0000
#define ISA_SCALED_FRAME_SIZE_ADDR  0x0004
#define ISA_ACKBUS_SRST_OUT_ADDR    0x0008
#define ISA_IC_SRST_OUT_ADDR        0x000C
#define ISA_LSC_SRST_OUT_ADDR       0x0010
#define ISA_SCAL_SRST_OUT_ADDR      0x0014
#define ISA_STA_AWB_SRST_OUT_ADDR   0x0018
#define ISA_STA_AE_SRST_OUT_ADDR    0x001C
#define ISA_STA_AF_SRST_OUT_ADDR    0x0020
#define ISA_DPC_SRST_OUT_ADDR       0x0024
#define ISA_MUX_SEL_ADDR            0x0028

static const unsigned int MUX_SEL[VIED_NCI_N_ACC_ID] = {
	0, /* VIED_NCI_ICA_ID */
	1, /* VIED_NCI_LSC_ID */
	2, /* VIED_NCI_DPC_ID */
	2, /* VIED_NCI_IDS_ID */
	2, /* VIED_NCI_AWB_ID */
	2, /* VIED_NCI_AF_ID  */
	2, /* VIED_NCI_AE_ID  */
};

#define ACCELERATOR_IP_FF_META_DATA_END(ff_id, set_id) (\
					 0)

#define ACCELERATOR_IP_FF_META_DATA_END(ff_id, set_id) (\
					 0)

#endif /* VIED_NCI_ACC_ISA_DEVICE_ADDRESS_H_ */
