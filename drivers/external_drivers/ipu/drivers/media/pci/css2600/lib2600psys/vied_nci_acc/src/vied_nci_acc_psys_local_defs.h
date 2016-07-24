/*
 * vied_nci_acc_psys_local_defs.h
 *
 *  Created on: Jan 15, 2014
 *     Authors: mmarkov1
 *              vilic
 */

#ifndef _VIED_NCI_ACC_PSYS_LOCAL_DEFS_H_
#define _VIED_NCI_ACC_PSYS_LOCAL_DEFS_H_

#define VIED_NCI_ACC_ACK_ADDR_ADDR 0x4
#define VIED_NCI_ACC_ACK_CMD_ADDR  0x0

#define VIED_NCI_REGISTER_SIZE_BYTES 4

#define  LACE_MEM_H0_SET0_BASE_ADDR   0x00100
#define  LACE_MEM_H0_SET1_BASE_ADDR   0x00600

#define  LACE_MEM_H0_SET0_END_ADDR   0x00600
#define  LACE_MEM_H0_SET1_END_ADDR   0x00b00

// Standard device API macros
#define vied_nci_acc_device_set_register(ACB_id, addr, val) \
  vied_subsystem_store_32(psys0, ACB_id + addr, (val))

#define vied_nci_acc_device_get_register(ACB_id, addr) \
  vied_subsystem_load_32(psys0, ACB_id + addr)


#define ACC_ACB_path(acc_handle) (\
	/* PSA accelerators */ \
	(acc_handle == VIED_NCI_WBA_ID      ) ? HOST_AB_PS_A_PSA_WB_ACB_ADDR  :\
	(acc_handle == VIED_NCI_ANR_ID      ) ? HOST_AB_PS_A_PSA_ANR_ACB_ADDR :\
	(acc_handle == VIED_NCI_DEMOSAIC_ID ) ? HOST_AB_PS_A_PSA_DM_ACB_ADDR  :\
	(acc_handle == VIED_NCI_CCM_ID      ) ? HOST_AB_PS_A_PSA_CCM_ACB_ADDR :\
	(acc_handle == VIED_NCI_GTC_ID      ) ? HOST_AB_PS_A_PSA_GTC_ACB_ADDR :\
	(acc_handle == VIED_NCI_YUV1_ID     ) ? HOST_AB_PS_A_PSA_YUV1_ACB_ADDR:\
	(acc_handle == VIED_NCI_DVS_ID      ) ? HOST_AB_PS_A_PSA_DVS_ACB_ADDR :\
	(acc_handle == VIED_NCI_LACE_ID     ) ? HOST_AB_PS_A_PSA_LACE_ACB_ADDR : \
	/* ISA accelerators */ \
	(acc_handle == VIED_NCI_ICA_ID ) ? HOST_PS_ISLC_ISA_INPUT_CORR_ACB_ADDR  :\
	(acc_handle == VIED_NCI_LSC_ID ) ? HOST_PS_ISLC_ISA_LSC_ACB_ADDR :\
	(acc_handle == VIED_NCI_DPC_ID ) ? HOST_PS_ISLC_ISA_GDDPC_ACB_ADDR  :\
	(acc_handle == VIED_NCI_IDS_ID ) ? 0 /*HOST_PS_ISLC_ISA_SCALER_ACB_ADDR*/ :\
	(acc_handle == VIED_NCI_AWB_ID ) ? HOST_PS_ISLC_ISA_AWB_ACB_ADDR :\
	(acc_handle == VIED_NCI_AE_ID  ) ? HOST_PS_ISLC_ISA_AF_AWB_FR_GRD_ACB_ADDR :\
	(acc_handle == VIED_NCI_AF_ID  ) ? HOST_PS_ISLC_ISA_AE_STATS_ACB_ADDR:\
	                                   0)


#define ACC_ack_path(acc_handle) (\
	/* PSA accelerators */ \
	(acc_handle == VIED_NCI_WBA_ID      ) ? HOST_AB_PS_A_PSA_WB_AckConv_ADDR  :\
	(acc_handle == VIED_NCI_ANR_ID      ) ? HOST_AB_PS_A_PSA_ANR_AckConv_ADDR :\
	(acc_handle == VIED_NCI_DEMOSAIC_ID ) ? HOST_AB_PS_A_PSA_DM_AckConv_ADDR  :\
	(acc_handle == VIED_NCI_CCM_ID      ) ? HOST_AB_PS_A_PSA_CCM_AckConv_ADDR :\
	(acc_handle == VIED_NCI_GTC_ID      ) ? HOST_AB_PS_A_PSA_GTC_AckConv_ADDR :\
	(acc_handle == VIED_NCI_YUV1_ID     ) ? HOST_AB_PS_A_PSA_YUV1_AckConv_ADDR:\
	(acc_handle == VIED_NCI_DVS_ID      ) ? HOST_AB_PS_A_PSA_DVS_AckConv_ADDR :\
	(acc_handle == VIED_NCI_LACE_ID     ) ? HOST_AB_PS_A_PSA_LACE_AckConv_ADDR : \
	/* ISA accelerators */ \
	(acc_handle == VIED_NCI_ICA_ID ) ? HOST_PS_ISLC_ISA_INPUT_CORR_ACK_CONV_ADDR  :\
	(acc_handle == VIED_NCI_LSC_ID ) ? HOST_PS_ISLC_ISA_LSC_ACK_CONV_ADDR :\
	(acc_handle == VIED_NCI_DPC_ID ) ? HOST_PS_ISLC_ISA_GDDPC_ACK_CONV_ADDR  :\
	(acc_handle == VIED_NCI_IDS_ID ) ? 0 /*HOST_PS_ISLC_ISA_SCALER_ACK_CONV_ADDR*/ :\
	(acc_handle == VIED_NCI_AWB_ID ) ? HOST_PS_ISLC_ISA_AWB_ACK_CONV_ADDR :\
	(acc_handle == VIED_NCI_AF_ID  ) ? HOST_PS_ISLC_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR:\
	(acc_handle == VIED_NCI_AE_ID  ) ? HOST_PS_ISLC_ISA_AE_STATS_ACK_CONV_ADDR :\
	                                   0)


#define FF_ADDR(fixed_function_id) (\
	/* PSA fixed functions */ \
	(fixed_function_id == VIED_NCI_WBA_WBA_FF          ) ? HOST_AB_PS_A_PSA_WB_WBA_ADDR : \
	(fixed_function_id == VIED_NCI_WBA_BC_FF           ) ? HOST_AB_PS_A_PSA_WB_BC_ADDR : \
	(fixed_function_id == VIED_NCI_ANR_SEARCH_FF       ) ? HOST_AB_PS_A_PSA_ANR_SEARCH_ADDR : \
	(fixed_function_id == VIED_NCI_ANR_TRANSFORM_FF    ) ? HOST_AB_PS_A_PSA_ANR_TRANSFORM_ADDR : \
	(fixed_function_id == VIED_NCI_ANR_STITCH_FF       ) ? HOST_AB_PS_A_PSA_ANR_STITCH_ADDR : \
	(fixed_function_id == VIED_NCI_ANR_TILE_FF         ) ? HOST_AB_PS_A_PSA_ANR_TILE_ADDR : \
	(fixed_function_id == VIED_NCI_DEMOSAIC_DEMOSAIC_FF) ? HOST_AB_PS_A_PSA_DM_ADDR : \
	(fixed_function_id == VIED_NCI_CCM_CCM_FF          ) ? HOST_AB_PS_A_PSA_CCM_CCM_ADDR : \
	(fixed_function_id == VIED_NCI_CCM_BDC_FF          ) ? HOST_AB_PS_A_PSA_CCM_BDC_ADDR : \
	(fixed_function_id == VIED_NCI_GTC_CSC_CDS_FF      ) ? HOST_AB_PS_A_PSA_GTC_CSC_ADDR : \
	(fixed_function_id == VIED_NCI_GTC_GTM_FF          ) ? HOST_AB_PS_A_PSA_GTC_GTM_ADDR : \
	(fixed_function_id == VIED_NCI_YUV1_YEENR_FF       ) ? 0/*HOST_AB_PS_A_PSA_YUV1_Y_EE_NR_ADDR */: \
	(fixed_function_id == VIED_NCI_YUV1_YDS_FF         ) ? HOST_AB_PS_A_PSA_YUV1_YDS_ADDR : \
	(fixed_function_id == VIED_NCI_YUV1_TCC_FF         ) ? HOST_AB_PS_A_PSA_YUV1_TCC_ADDR : \
	(fixed_function_id == VIED_NCI_DVS_YBIN_FF         ) ? HOST_AB_PS_A_PSA_DVS_YBIN_ADDR : \
	(fixed_function_id == VIED_NCI_DVS_DVS_FF          ) ? HOST_AB_PS_A_PSA_DVS_DVS_ADDR : \
	(fixed_function_id == VIED_NCI_LACE_LACE_FF        ) ? HOST_AB_PS_A_PSA_LACE_STAT_ADDR : \
	/* ISA fixed functions */ \
	(fixed_function_id == VIED_NCI_ICA_INL_FF      ) ? HOST_PS_ISLC_ISA_INPUT_CORR_INL_ADDR : \
	(fixed_function_id == VIED_NCI_ICA_GBL_FF      ) ? HOST_PS_ISLC_ISA_INPUT_CORR_GBL_ADDR : \
	(fixed_function_id == VIED_NCI_ICA_PCLN_FF     ) ? HOST_PS_ISLC_ISA_INPUT_CORR_PCLN_ADDR : \
	(fixed_function_id == VIED_NCI_LSC_LSC_FF      ) ? HOST_PS_ISLC_ISA_LSC_LSC_ADDR : \
	(fixed_function_id == VIED_NCI_DPC_DPC_FF      ) ? HOST_PS_ISLC_ISA_GDDPC_GDDPC_ADDR : \
	(fixed_function_id == VIED_NCI_IDS_SCALER_FF   ) ? 0 /*HOST_PS_ISLC_ISA_SCALER_SCALER_ADDR*/ : \
	(fixed_function_id == VIED_NCI_AWB_AWRG_FF     ) ? HOST_PS_ISLC_ISA_AWB_AWRG_ADDR : \
	(fixed_function_id == VIED_NCI_AF_AF_FF        ) ? HOST_PS_ISLC_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR : \
	(fixed_function_id == VIED_NCI_AE_WGHT_HIST_FF ) ? HOST_PS_ISLC_ISA_AE_STATS_WGHT_HIST_ADDR : \
	(fixed_function_id == VIED_NCI_AE_CCM_FF       ) ? HOST_PS_ISLC_ISA_AE_STATS_CCM_ADDR : \
	                                                   0)


#define FF_PARAMS_BASE_ADDR(fixed_function_id) (\
	/* PSA fixed functions */ \
	(fixed_function_id == VIED_NCI_WBA_WBA_FF          ) ? 0 : \
	(fixed_function_id == VIED_NCI_WBA_BC_FF           ) ? 0 : \
	(fixed_function_id == VIED_NCI_ANR_SEARCH_FF       ) ? 0 : \
	(fixed_function_id == VIED_NCI_ANR_TRANSFORM_FF    ) ? 0 : \
	(fixed_function_id == VIED_NCI_ANR_STITCH_FF       ) ? 0 : \
	(fixed_function_id == VIED_NCI_ANR_TILE_FF         ) ? 0 : \
	(fixed_function_id == VIED_NCI_DEMOSAIC_DEMOSAIC_FF) ? 0 : \
	(fixed_function_id == VIED_NCI_CCM_CCM_FF          ) ? 0 : \
	(fixed_function_id == VIED_NCI_CCM_BDC_FF          ) ? 0 : \
	(fixed_function_id == VIED_NCI_GTC_CSC_CDS_FF      ) ? 0 : \
	(fixed_function_id == VIED_NCI_GTC_GTM_FF          ) ? 0 : \
	(fixed_function_id == VIED_NCI_YUV1_YEENR_FF       ) ? 0 : \
	(fixed_function_id == VIED_NCI_YUV1_YDS_FF         ) ? 0 : \
	(fixed_function_id == VIED_NCI_YUV1_TCC_FF         ) ? 0 : \
	(fixed_function_id == VIED_NCI_DVS_YBIN_FF         ) ? 0 : \
	(fixed_function_id == VIED_NCI_DVS_DVS_FF          ) ? 0 : \
	(fixed_function_id == VIED_NCI_LACE_LACE_FF        ) ? 0 : \
	/* ISA fixed functions */ \
	(fixed_function_id == VIED_NCI_ICA_INL_FF      ) ? 0 : \
	(fixed_function_id == VIED_NCI_ICA_GBL_FF      ) ? 0 : \
	(fixed_function_id == VIED_NCI_ICA_PCLN_FF     ) ? 0 : \
	(fixed_function_id == VIED_NCI_LSC_LSC_FF      ) ? 0 : \
	(fixed_function_id == VIED_NCI_DPC_DPC_FF      ) ? 0 : \
	(fixed_function_id == VIED_NCI_IDS_SCALER_FF   ) ? 0 : \
	(fixed_function_id == VIED_NCI_AWB_AWRG_FF     ) ? 0 : \
	(fixed_function_id == VIED_NCI_AF_AF_FF        ) ? 0 : \
	(fixed_function_id == VIED_NCI_AE_WGHT_HIST_FF ) ? 0 : \
	(fixed_function_id == VIED_NCI_AE_CCM_FF       ) ? 0 : \
	                                                   0)


#define FF_PARAMS_END_ADDR(fixed_function_id) (\
	/* PSA fixed functions */ \
	(fixed_function_id == VIED_NCI_WBA_WBA_FF          ) ? 0x20 : \
	(fixed_function_id == VIED_NCI_WBA_BC_FF           ) ? 0x224 : \
	(fixed_function_id == VIED_NCI_ANR_SEARCH_FF       ) ? 0x8 : \
	(fixed_function_id == VIED_NCI_ANR_TRANSFORM_FF    ) ? 0x238 : \
	(fixed_function_id == VIED_NCI_ANR_STITCH_FF       ) ? 0x88 : \
	(fixed_function_id == VIED_NCI_ANR_TILE_FF         ) ? 0x8 : \
	(fixed_function_id == VIED_NCI_DEMOSAIC_DEMOSAIC_FF) ? 0xC : \
	(fixed_function_id == VIED_NCI_CCM_CCM_FF          ) ? 0x224 : \
	(fixed_function_id == VIED_NCI_CCM_BDC_FF          ) ? 0x1C : \
	(fixed_function_id == VIED_NCI_GTC_CSC_CDS_FF      ) ? 0x1C : \
	(fixed_function_id == VIED_NCI_GTC_GTM_FF          ) ? 0x78C : \
	(fixed_function_id == VIED_NCI_YUV1_YEENR_FF       ) ? 0x28 : \
	(fixed_function_id == VIED_NCI_YUV1_YDS_FF         ) ? 0x4 : \
	(fixed_function_id == VIED_NCI_YUV1_TCC_FF         ) ? 0x2D4 : \
	(fixed_function_id == VIED_NCI_DVS_YBIN_FF         ) ? 0x4 : \
	(fixed_function_id == VIED_NCI_DVS_DVS_FF          ) ? 0x34 : \
	(fixed_function_id == VIED_NCI_LACE_LACE_FF        ) ? 0x1C : \
	/* ISA fixed functions */ \
	(fixed_function_id == VIED_NCI_ICA_INL_FF      ) ? 0x208 : \
	(fixed_function_id == VIED_NCI_ICA_GBL_FF      ) ? 0x82C : \
	(fixed_function_id == VIED_NCI_ICA_PCLN_FF     ) ? 0x824 : \
	(fixed_function_id == VIED_NCI_LSC_LSC_FF      ) ? 0xC0C : \
	(fixed_function_id == VIED_NCI_DPC_DPC_FF      ) ? 0 : \
	(fixed_function_id == VIED_NCI_IDS_SCALER_FF   ) ? 0 : \
	(fixed_function_id == VIED_NCI_AWB_AWRG_FF     ) ? 0 : \
	(fixed_function_id == VIED_NCI_AF_AF_FF        ) ? 0 : \
	(fixed_function_id == VIED_NCI_AE_WGHT_HIST_FF ) ? 0 : \
	(fixed_function_id == VIED_NCI_AE_CCM_FF       ) ? 0 : \
	                                                   0)

#define  LSC_MEM_LUT_CH0_SET0_BASE_ADDR   0x00000
#define  LSC_MEM_LUT_CH0_SET1_BASE_ADDR   0x00100
#define  LSC_MEM_LUT_CH0_SET2_BASE_ADDR   0x00200
#define  LSC_MEM_LUT_CH1_SET0_BASE_ADDR   0x00300
#define  LSC_MEM_LUT_CH1_SET1_BASE_ADDR   0x00400
#define  LSC_MEM_LUT_CH1_SET2_BASE_ADDR   0x00500
#define  LSC_MEM_LUT_CH2_SET0_BASE_ADDR   0x00600
#define  LSC_MEM_LUT_CH2_SET1_BASE_ADDR   0x00700
#define  LSC_MEM_LUT_CH2_SET2_BASE_ADDR   0x00800
#define  LSC_MEM_LUT_CH3_SET0_BASE_ADDR   0x00900
#define  LSC_MEM_LUT_CH3_SET1_BASE_ADDR   0x00a00
#define  LSC_MEM_LUT_CH3_SET2_BASE_ADDR   0x00b00

#define LSC_PARAM_SET_BASE_ADDR(set_id) (\
	(set_id ==  0) ?  LSC_MEM_LUT_CH0_SET0_BASE_ADDR : \
	(set_id ==  1) ?  LSC_MEM_LUT_CH0_SET1_BASE_ADDR : \
	(set_id ==  2) ?  LSC_MEM_LUT_CH0_SET2_BASE_ADDR : \
	(set_id ==  3) ?  LSC_MEM_LUT_CH1_SET0_BASE_ADDR : \
	(set_id ==  4) ?  LSC_MEM_LUT_CH1_SET1_BASE_ADDR : \
	(set_id ==  5) ?  LSC_MEM_LUT_CH1_SET2_BASE_ADDR : \
	(set_id ==  6) ?  LSC_MEM_LUT_CH2_SET0_BASE_ADDR : \
	(set_id ==  7) ?  LSC_MEM_LUT_CH2_SET1_BASE_ADDR : \
	(set_id ==  8) ?  LSC_MEM_LUT_CH2_SET2_BASE_ADDR : \
	(set_id ==  9) ?  LSC_MEM_LUT_CH3_SET0_BASE_ADDR : \
	(set_id == 10) ?  LSC_MEM_LUT_CH3_SET1_BASE_ADDR : \
	(set_id == 11) ?  LSC_MEM_LUT_CH3_SET2_BASE_ADDR : \
	                  0)

#define FF_PARAM_SET_BASE_ADDR(fixed_function_id, set_id) (\
	(fixed_function_id == VIED_NCI_LSC_LSC_FF ) ? LSC_PARAM_SET_BASE_ADDR(set_id) : \
                                                    0)

#define  LSC_MEM_LUT_CH0_SET0_END_ADDR   0x00100
#define  LSC_MEM_LUT_CH0_SET1_END_ADDR   0x00200
#define  LSC_MEM_LUT_CH0_SET2_END_ADDR   0x00300
#define  LSC_MEM_LUT_CH1_SET0_END_ADDR   0x00400
#define  LSC_MEM_LUT_CH1_SET1_END_ADDR   0x00500
#define  LSC_MEM_LUT_CH1_SET2_END_ADDR   0x00600
#define  LSC_MEM_LUT_CH2_SET0_END_ADDR   0x00700
#define  LSC_MEM_LUT_CH2_SET1_END_ADDR   0x00800
#define  LSC_MEM_LUT_CH2_SET2_END_ADDR   0x00900
#define  LSC_MEM_LUT_CH3_SET0_END_ADDR   0x00a00
#define  LSC_MEM_LUT_CH3_SET1_END_ADDR   0x00b00
#define  LSC_MEM_LUT_CH3_SET2_END_ADDR   0x00c00

#define LSC_PARAM_SET_END_ADDR(set_id) (\
	(set_id ==  0) ?  LSC_MEM_LUT_CH0_SET0_END_ADDR : \
	(set_id ==  1) ?  LSC_MEM_LUT_CH0_SET1_END_ADDR : \
	(set_id ==  2) ?  LSC_MEM_LUT_CH0_SET2_END_ADDR : \
	(set_id ==  3) ?  LSC_MEM_LUT_CH1_SET0_END_ADDR : \
	(set_id ==  4) ?  LSC_MEM_LUT_CH1_SET1_END_ADDR : \
	(set_id ==  5) ?  LSC_MEM_LUT_CH1_SET2_END_ADDR : \
	(set_id ==  6) ?  LSC_MEM_LUT_CH2_SET0_END_ADDR : \
	(set_id ==  7) ?  LSC_MEM_LUT_CH2_SET1_END_ADDR : \
	(set_id ==  8) ?  LSC_MEM_LUT_CH2_SET2_END_ADDR : \
	(set_id ==  9) ?  LSC_MEM_LUT_CH3_SET0_END_ADDR : \
	(set_id == 10) ?  LSC_MEM_LUT_CH3_SET1_END_ADDR : \
	(set_id == 11) ?  LSC_MEM_LUT_CH3_SET2_END_ADDR : \
	                  0)

#define FF_PARAM_SET_END_ADDR(fixed_function_id, set_id) (\
	(fixed_function_id == VIED_NCI_LSC_LSC_FF ) ? LSC_PARAM_SET_END_ADDR(set_id) : \
	                                              0)

#define LACE_META_DATA_SET_BASE_ADDR(set_id) (\
	(set_id = 0) ? LACE_MEM_H0_SET0_BASE_ADDR : \
	(set_id = 1) ? LACE_MEM_H0_SET1_BASE_ADDR : \
	               0x0)

/* TODO: fill the table for DVS meta data sets */
#define DVS_META_DATA_SET_BASE_ADDR(set_id) (\
	0)

#define LACE_META_DATA_SET_END_ADDR(set_id) (\
	(set_id = 0) ? LACE_MEM_H0_SET0_END_ADDR : \
	(set_id = 1) ? LACE_MEM_H0_SET1_END_ADDR : \
	               0x0)

/* TODO: fill the table for DVS meta data sets */
#define DVS_META_DATA_SET_END_ADDR(set_id) (\
	0)

#define FF_META_DATA_SET_BASE_ADDR(fixed_function_id, set_id) (\
	(fixed_function_id == VIED_NCI_LACE_LACE_FF ) ? LACE_META_DATA_SET_BASE_ADDR(set_id) : \
	(fixed_function_id == VIED_NCI_DVS_DVS_FF   ) ? DVS_META_DATA_SET_BASE_ADDR(set_id)  : \
	                                                0)

#define FF_META_DATA_SET_END_ADDR(fixed_function_id, set_id) (\
	(fixed_function_id == VIED_NCI_LACE_LACE_FF ) ? LACE_META_DATA_SET_END_ADDR(set_id) : \
	(fixed_function_id == VIED_NCI_DVS_DVS_FF   ) ? DVS_META_DATA_SET_END_ADDR(set_id)  : \
	                                                0)

#define MUX_SEL(acc_handle) (\
	(acc_handle == VIED_NCI_ICA_ID ) ? 0 :\
	(acc_handle == VIED_NCI_LSC_ID ) ? 1 :\
	(acc_handle == VIED_NCI_DPC_ID ) ? 2 :\
	(acc_handle == VIED_NCI_IDS_ID ) ? 2 :\
	(acc_handle == VIED_NCI_AWB_ID ) ? 2 :\
	(acc_handle == VIED_NCI_AF_ID  ) ? 2 :\
	(acc_handle == VIED_NCI_AE_ID  ) ? 2 :\
	                                   0)

#define ACC_2_S2V(acc_handle) (\
	(acc_handle == VIED_NCI_WBA_ID      ) ? VIED_NCI_S2V_BAYER1_ID:\
	(acc_handle == VIED_NCI_ANR_ID      ) ? VIED_NCI_S2V_BAYER2_ID:\
	(acc_handle == VIED_NCI_DEMOSAIC_ID ) ? VIED_NCI_S2V_RGB_ID   :\
	(acc_handle == VIED_NCI_CCM_ID      ) ? VIED_NCI_S2V_RGB_ID   :\
	(acc_handle == VIED_NCI_GTC_ID      ) ? VIED_NCI_S2V_YUV1_ID  :\
	(acc_handle == VIED_NCI_YUV1_ID     ) ? VIED_NCI_S2V_YUV2_ID  :\
	                                        VIED_NCI_S2V_UNKNOWN_ID)

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
