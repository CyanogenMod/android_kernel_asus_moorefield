#ifndef _ISA_CLUSTER_API_H
#define _ISA_CLUSTER_API_H

#include <hrt/api.h>

#include "Acc_Input_Corr_api.h"
#include "Acc_WBA_api.h"
#include "Acc_Stats_Af_Awb_FR_Grd_api.h"
#include "Acc_Stats_Ae_api_V2.h"
#include "Acc_Gddpc_api.h"
#include "Acc_Lsc_api.h"
#include "Acc_Scaler_api.h"


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


//Input Correction
#define Acc_Input_Corr_Path(ISA_Cluster_id)         HRTCAT(ISA_Cluster_id,_Input_Corr)

//AWB Statistics 
#define Acc_AWB_STATS_path(ISA_Cluster_id)      	HRTCAT(ISA_Cluster_id,_Stat_AWB)

//AF Statistics 
#define Acc_AF_STATS_path(ISA_Cluster_id)       	HRTCAT(ISA_Cluster_id,_Stat_AF)

//AE Statistics 
#define Acc_AE_STATS_path(ISA_Cluster_id)       	HRTCAT(ISA_Cluster_id,_Stat_AE)

//Bayer Dpc 
#define Acc_Dpc_path(ISA_Cluster_id)       	        HRTCAT(ISA_Cluster_id,_Bayer_Dpc)

//Bayer Lsc 
#define Acc_Lsc_path(ISA_Cluster_id)            	HRTCAT(ISA_Cluster_id,_Bayer_Lsc)

//Bayer Scaler 
#define Acc_Scaler_path(ISA_Cluster_id)            	HRTCAT(ISA_Cluster_id,_Bayer_Scaler)

//GP regs
#define Isa_Gp_Reg_path(ISA_Cluster_id)            	HRTCAT(ISA_Cluster_id,_isa_gp_reg)


//General Purpose registers related definitions
#define Acc_GP_Regs_port(ACC_GP_REGS_id)                HRTCAT(ACC_GP_REGS_id,_slv_in)

#define Acc_GP_Regs_set_register(ACC_GP_REGS_id, addr, val) \
    _hrt_slave_port_store_32_volatile(Acc_GP_Regs_port(ACC_GP_REGS_id), addr, (val))

#define Acc_GP_Regs_get_register(ACC_GP_REGS_id, addr) \
    _hrt_slave_port_load_32_volatile(Acc_GP_Regs_port(ACC_GP_REGS_id), addr)

#define Acc_GP_Regs_path(ISA_Cluster_id)               HRTCAT(ISA_Cluster_id,_acc_gp_reg)

#endif
