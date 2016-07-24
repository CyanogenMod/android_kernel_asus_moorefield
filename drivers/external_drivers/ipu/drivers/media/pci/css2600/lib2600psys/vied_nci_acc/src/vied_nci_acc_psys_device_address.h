/*
 * vied_nci_acc_psys_device_address.h
 *
 *  Created on: Feb 10, 2014
 *     Authors: tversteg, mmarkov1
 */

#ifndef VIED_NCI_ACC_PSYS_DEVICE_ADDRESS_H_
#define VIED_NCI_ACC_PSYS_DEVICE_ADDRESS_H_

#include "vied_nci_acc_psys_defs.h"

/*
 * NOTE:
 * Temporary cell address mapping of the PSA accelerators!
 * !!!!
 * Table entries MUST be inline with associated enum values!
 * !!!!
 */
static const unsigned int ACCELERATOR_IP_ACB[VIED_NCI_N_ACC_ID] = {
	/* PSA accelerators */
	0xDB300, /* accelerator ip WBA */
	0xDA000, /* accelerator ip ANR */
	0xDC000, /* accelerator ip DEMOSAIC */
	0xDD100, /* accelerator ip CCM */
	0xD8800, /* accelerator ip GTC */
	0xD9000, /* accelerator ip YUV1 */
	0xD0000, /* accelerator ip DVS */
	0xD6800, /* accelerator ip LACE */
	/* ISA accelerators */
	0xE6600, /* accelerator ip ICA */
	0xE4D00, /* accelerator ip LSC */
	0xE0000, /* accelerator ip DPC */
	0xEA000, /* accelerator ip Scaler */
	0xEEC00, /* accelerator ip AWB */
	0xED800, /* accelerator ip AE */
	0xEF500, /* accelerator ip AF_AWB_FR */
};

static const unsigned int ACCELERATOR_IP_ACK[VIED_NCI_N_ACC_ID] = {
	/* PSA accelerators */
	0xDB400, /* accelerator ip WBA */
	0xDA100, /* accelerator ip ANR */
	0xDC050, /* accelerator ip DEMOSAIC */
	0xDD200, /* accelerator ip CCM */
	0xD8900, /* accelerator ip GTC */
	0xD9100, /* accelerator ip YUV1 */
	0xD0100, /* accelerator ip DVS */
	0xD6C00, /* accelerator ip LACE */
	/* ISA accelerators */
	0xE6700, /* accelerator ip ICA */
	0xE4E00, /* accelerator ip LSC */
	0xE0050, /* accelerator ip DPC */
	0xEA800, /* accelerator ip Scaler */
	0xEEE00, /* accelerator ip AWB */
	0xEDC00, /* accelerator ip AE */
	0xEF600, /* accelerator ip AF_AWB_FR */
};


static const unsigned int ACCELERATOR_IP_FF[VIED_NCI_N_ACC_FF] = {
	/* PSA fixed functions */
	0xDB500, /* Bayer_WB_WBA_crq_in */
	0xDB000, /* Bayer_WB_BC_crq_in */
	0xDA200, /* Bayer_ANR_SEARCH_crq_in */
	0xDA300, /* Bayer_ANR_TRANSFORM_crq_in */
	0xDA900, /* Bayer_ANR_STITCH_crq_in */
	0xDAB00, /* Bayer_ANR_TILE_crq_in */
	0xDC060, /* Advanced_Demosaic_DM_crq_in */
	0xDD600, /* CCM_CCM_crq_in */
	0xDD300, /* CCM_BDC_crq_in */
	0xD8A00, /* GTC_CSC_CDS_crq_in */
	0xD8000, /* GTC_GTM_crq_in */
	0xD9300, /* YUV1_Processing_IEFD_crq_in */
	0xD9400, /* YUV1_Processing_YDS_crq_in */
	0xD9500, /* YUV1_Processing_TCC_crq_in */
	0xD0200, /* DVS_YBIN_crq_in */
	0xD0300, /* DVS_DVS_crq_in */
	0xD5000, /* Lace_Stat_LACE_STAT_crq_in */
	/* ISA fixed functions */
	0xE6800, /* fixed function ip ICA_INL */
	0xE5000, /* fixed function ip ICA_GBL */
	0xE5D00, /* fixed function ip ICA_PCLN */
	0xE4000, /* fixed function ip LSC_LSC */
	0xE0060, /* fixed function ip DPC_GDDPC */
	0xE7000, /* fixed function ip Scaler_SCALER */
	0xEE000, /* fixed function ip AWB_AWRG */
	0xEDE00, /* fixed function ip AE_CCM */
	0xEB000, /* fixed function ip AE_WGHT_HIST */
	0xEF000, /* fixed function ip AF_AWB_FR_AF_AWB_FR_GRD */
};


static const unsigned int ACCELERATOR_IP_FF_PARAMS_BASE[VIED_NCI_N_ACC_FF] = {
	/* PSA fixed functions */
	0x0, /* Bayer_WB_WBA_crq_in */
	0x0, /* Bayer_WB_BC_crq_in */
	0x0, /* Bayer_ANR_SEARCH_crq_in */
	0x0, /* Bayer_ANR_TRANSFORM_crq_in */
	0x0, /* Bayer_ANR_STITCH_crq_in */
	0x0, /* Bayer_ANR_TILE_crq_in */
	0x0, /* Advanced_Demosaic_DM_crq_in */
	0x0, /* CCM_CCM_crq_in */
	0x0, /* CCM_BDC_crq_in */
	0x0, /* GTC_CSC_CDS_crq_in */
	0x0, /* GTC_GTM_crq_in */
	0x0, /* YUV1_Processing_IEFD_crq_in */
	0x0, /* YUV1_Processing_YDS_crq_in */
	0x0, /* YUV1_Processing_TCC_crq_in */
	0x0, /* DVS_YBIN_crq_in */
	0x0, /* DVS_DVS_crq_in */
	0x0, /* Lace_Stat_LACE_STAT_crq_in */
	/* ISA fixed functions */
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
	/* PSA fixed functions */
	0x020, /* Bayer_WB_WBA_crq_in */
	0x224, /* Bayer_WB_BC_crq_in */
	0x008, /* Bayer_ANR_SEARCH_crq_in */
	0x238, /* Bayer_ANR_TRANSFORM_crq_in */
	0x088, /* Bayer_ANR_STITCH_crq_in */
	0x008, /* Bayer_ANR_TILE_crq_in */
	0x00C, /* Advanced_Demosaic_DM_crq_in */
	0x224, /* CCM_CCM_crq_in */
	0x01C, /* CCM_BDC_crq_in */
	0x01C, /* GTC_CSC_CDS_crq_in */
	0x78C, /* GTC_GTM_crq_in */
	0x028, /* YUV1_Processing_IEFD_crq_in */
	0x004, /* YUV1_Processing_YDS_crq_in */
	0x2D4, /* YUV1_Processing_TCC_crq_in */
	0x004, /* DVS_YBIN_crq_in */
	0x034, /* DVS_DVS_crq_in */
	0x01C, /* Lace_Stat_LACE_STAT_crq_in */
	/* ISA fixed functions */
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


static const unsigned int LACE_OUT_BASE_ADDRESS[VIED_NCI_N_LACE_OUT_SET] = {
	0x0100, /* set 0 */
	0x0600, /* set 1 */
};

static const unsigned int DVS_OUT_BASE_ADDRESS[VIED_NCI_N_DVS_OUT_SET] = {
	0x02420, /* l 0, set 0 */
	0x024e0, /* l 0, set 1 */
	0x025a0, /* l 1, set 0 */
	0x02660, /* l 1, set 1 */
	0x02720, /* l 2, set 0 */
	0x027c0, /* l 2, set 1 */
};

static const unsigned int LACE_OUT_END_ADDRESS[VIED_NCI_N_LACE_OUT_SET] = {
	0x0600, /* set 0 */
	0x0b00, /* set 1 */
};

static const unsigned int DVS_OUT_END_ADDRESS[VIED_NCI_N_DVS_OUT_SET] = {
	0x024e0, /* l 0, set 0 */
	0x025a0, /* l 0, set 1 */
	0x02660, /* l 1, set 0 */
	0x02720, /* l 1, set 1 */
	0x027e0, /* l 2, set 0 */
	0x028a0, /* l 2, set 1 */
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
	(ff_id = VIED_NCI_LSC_LSC_FF) ? LSC_PARAM_SET_BASE_ADDRESS[set_id] : \
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
	(ff_id = VIED_NCI_LACE_LACE_FF) ? LACE_OUT_BASE_ADDRESS[set_id] : \
	(ff_id = VIED_NCI_DVS_DVS_FF  ) ? DVS_OUT_BASE_ADDRESS[set_id]  : \
	                                  0)

#define ACCELERATOR_IP_FF_META_DATA_END(ff_id, set_id) (\
	(ff_id = VIED_NCI_LACE_LACE_FF) ? LACE_OUT_END_ADDRESS[set_id] : \
	(ff_id = VIED_NCI_DVS_DVS_FF  ) ? DVS_OUT_END_ADDRESS[set_id]  : \
	                                  0)

#define ISA_GP_REGS_ADDR 0xEF800

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

#endif /* VIED_NCI_ACC_PSYS_DEVICE_ADDRESS_H_ */
