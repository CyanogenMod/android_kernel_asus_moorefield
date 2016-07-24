#ifndef _VIED_NCI_ACC_ISYS_LOCAL_DEFS_H_
#define _VIED_NCI_ACC_ISYS_LOCAL_DEFS_H_

#define VIED_NCI_ACC_ACK_ADDR_ADDR 0x4
#define VIED_NCI_ACC_ACK_CMD_ADDR  0x0

#define VIED_NCI_REGISTER_SIZE_BYTES 4

// Standard device API macros
#define vied_nci_acc_device_set_register(ACB_id, addr, val) \
  vied_subsystem_store_32(isys0, ACB_id + addr, (val))

#define vied_nci_acc_device_get_register(ACB_id, addr) \
  vied_subsystem_load_32(isys0, ACB_id + addr)


#define ACC_ACB_path(acc_handle) (\
	(acc_handle == VIED_NCI_ICA_ID ) ? HOST_IS_A_ISA_INPUT_CORR_ACB_ADDR  :\
	(acc_handle == VIED_NCI_LSC_ID ) ? HOST_IS_A_ISA_LSC_ACB_ADDR :\
	(acc_handle == VIED_NCI_DPC_ID ) ? HOST_IS_A_ISA_GDDPC_ACB_ADDR  :\
	(acc_handle == VIED_NCI_IDS_ID ) ? HOST_IS_A_ISA_SCALER_ACB_ADDR :\
	(acc_handle == VIED_NCI_AWB_ID ) ? HOST_IS_A_ISA_AWB_ACB_ADDR :\
	(acc_handle == VIED_NCI_AE_ID  ) ? HOST_IS_A_ISA_AF_AWB_FR_GRD_ACB_ADDR :\
	(acc_handle == VIED_NCI_AF_ID  ) ? HOST_IS_A_ISA_AE_STATS_ACB_ADDR:\
	                                   0)


#define ACC_ack_path(acc_handle) (\
	(acc_handle == VIED_NCI_ICA_ID ) ? HOST_IS_A_ISA_INPUT_CORR_ACK_CONV_ADDR  :\
	(acc_handle == VIED_NCI_LSC_ID ) ? HOST_IS_A_ISA_LSC_ACK_CONV_ADDR :\
	(acc_handle == VIED_NCI_DPC_ID ) ? HOST_IS_A_ISA_GDDPC_ACK_CONV_ADDR  :\
	(acc_handle == VIED_NCI_IDS_ID ) ? HOST_IS_A_ISA_SCALER_ACK_CONV_ADDR :\
	(acc_handle == VIED_NCI_AWB_ID ) ? HOST_IS_A_ISA_AWB_ACK_CONV_ADDR :\
	(acc_handle == VIED_NCI_AF_ID  ) ? HOST_IS_A_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR:\
	(acc_handle == VIED_NCI_AE_ID  ) ? HOST_IS_A_ISA_AE_STATS_ACK_CONV_ADDR :\
	                                   0)


#define FF_ADDR(fixed_function_id) (\
	(fixed_function_id == VIED_NCI_ICA_INL_FF      ) ? HOST_IS_A_ISA_INPUT_CORR_INL_ADDR : \
	(fixed_function_id == VIED_NCI_ICA_GBL_FF      ) ? HOST_IS_A_ISA_INPUT_CORR_GBL_ADDR : \
	(fixed_function_id == VIED_NCI_ICA_PCLN_FF     ) ? HOST_IS_A_ISA_INPUT_CORR_PCLN_ADDR : \
	(fixed_function_id == VIED_NCI_LSC_LSC_FF      ) ? HOST_IS_A_ISA_LSC_LSC_ADDR : \
	(fixed_function_id == VIED_NCI_DPC_DPC_FF      ) ? HOST_IS_A_ISA_GDDPC_GDDPC_ADDR : \
	(fixed_function_id == VIED_NCI_IDS_SCALER_FF   ) ? HOST_IS_A_ISA_SCALER_SCALER_ADDR : \
	(fixed_function_id == VIED_NCI_AWB_AWRG_FF     ) ? HOST_IS_A_ISA_AWB_AWRG_ADDR : \
	(fixed_function_id == VIED_NCI_AF_AF_FF        ) ? HOST_IS_A_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR : \
	(fixed_function_id == VIED_NCI_AE_WGHT_HIST_FF ) ? HOST_IS_A_ISA_AE_STATS_WGHT_HIST_ADDR : \
	(fixed_function_id == VIED_NCI_AE_CCM_FF       ) ? HOST_IS_A_ISA_AE_STATS_CCM_ADDR : \
	                                                   0)


#define FF_PARAMS_BASE_ADDR(fixed_function_id) (\
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

#define FF_META_DATA_SET_BASE_ADDR(fixed_function_id, set_id) (\
	                                              0)

#define FF_META_DATA_SET_END_ADDR(fixed_function_id, set_id) (\
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

#endif /* _VIED_NCI_ACC_ISA_LOCAL_DEFS_H_ */
