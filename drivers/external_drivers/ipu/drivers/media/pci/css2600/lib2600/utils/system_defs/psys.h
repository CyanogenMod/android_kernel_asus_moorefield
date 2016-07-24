#ifndef _PSYS_H_
#define _PSYS_H_

#include "vied_config_host.h"
#include "processing_system_system.h"
// #include "processing_system_gac_sdk.h"
#include <vied/vied_subsystem_access_initialization.h>
//#include <vied/vied_memory_access_initialization.h>
#include <vied/vied_subsystem_access.h>
#include <vied/shared_memory_access.h>
#include <vied/shared_memory_map.h>

#define SUBSYSTEM_SLAVE   processing_system_unps_logic_mmu_at_system_cfg_bus_sl0
#define SUBSYSTEM_MASTER  testbench_master_rec0_mt

#define MMU0              processing_system_unps_logic_mmu_at_system_mmu0
#define MMU1              processing_system_unps_logic_mmu_at_system_mmu1
#define MMU               MMU0
#define MMU_PAGE_SHIFT    12
#define MMU_PAGE_BYTES    hrt_device_property(MMU,PageBytes)
#define MMU_PAGE_NUMBERS  hrt_device_property(MMU,PageTablesStorePageNumbersOnly)

/////////////////////////////////////////////////////////////////////////////////////

#define HOST              host
#define HOST_MASTER       host_op0

#define XMEM              testbench_ddr
#define XMEM_SLAVE        testbench_ddr_ip0
#define XMEM_BYTES        _hrt_sysmem_size(XMEM)
#define XMEM_ALIGN        32


#define host_ddr_addr()   hrt_master_to_slave_address(HOST_MASTER,   XMEM_SLAVE)
#define mmu_ddr_addr()    hrt_master_to_slave_address(SUBSYSTEM_MASTER, XMEM_SLAVE)

/////////////////////////////////////////////////////////////////////////////////////

/* Global memory related defines */
#define PSYS_GMEM_BASE           _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_gmem_sl_in
#define PSYS_GMEM_BASE_HOST      _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_gmem_sl_in
#define PSYS_GMEM_BASE_DMA       _hrt_master_to_slave_address_processing_system_dma_logic_dma_external0_m0_to_processing_system_unps_logic_gmem_mem_ip0
#define PSYS_GMEM_BASE_S2V       _hrt_master_to_slave_address_processing_system_psa_psa_logic_PSA_Cluster_str2vec_bayer1_mt_to_processing_system_unps_logic_gmem_mem_ip0
#define PSYS_GMEM_BASE_DMA_M0    _hrt_master_to_slave_address_processing_system_dma_logic_dma_external0_m0_to_processing_system_unps_logic_gmem_mem_ip0
#define PSYS_GMEM_BASE_DMA_M2    _hrt_master_to_slave_address_processing_system_dma_logic_dma_external0_m2_to_processing_system_unps_logic_gmem_mem_ip0
#define ISP0_TILE_GMEM_BASE_ADDR _hrt_master_to_slave_address_processing_system_isp_tile0_logic_isp_mt_bmem_op_to_processing_system_unps_logic_gmem_mem_ip0

/////////////////////////////////////////////////////////////////////////////////////
/* ISP local VMEM related defines */
#define ISP0_VMEM_BASE      _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_isp_tile0_logic_isp_sl_vmem_ip
#define ISP0_VMEM_BASE_HOST _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_isp_tile0_logic_isp_sl_vmem_ip
#define ISP0_VMEM_BASE_S2V _hrt_master_to_slave_address_processing_system_psa_psa_logic_PSA_Cluster_str2vec_bayer1_mt_to_processing_system_isp_tile0_logic_isp_sl_vmem_ip
#define ISP0_VMEM_BASE_DMA_M0 _hrt_master_to_slave_address_processing_system_dma_logic_dma_external0_m0_to_processing_system_isp_tile0_logic_isp_sl_vmem_ip
#define ISP0_VMEM_BASE_DMA_M2 _hrt_master_to_slave_address_processing_system_dma_logic_dma_external0_m2_to_processing_system_isp_tile0_logic_isp_sl_vmem_ip

/////////////////////////////////////////////////////////////////////////////////////

/* Interrupt related defines IRQ control + IRQ Register */
#define PSYS_IRQCTR  (_hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_gp_sl_in + _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_gp_mt_out_to_processing_system_unps_logic_gp_devices_irqcontrol_slv_in)
#define PSYS_IRQREG  (_hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_gp_sl_in + _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_gp_mt_out_to_processing_system_unps_logic_gp_devices_irqreg_slv_in)
#define PSYS_SW_INT_START 10

/////////////////////////////////////////////////////////////////////////////////////

#define PSYS_SPC        processing_system_unps_logic_spc_tile_sp
#define PSYS_SPP0       processing_system_unps_logic_spp_tile0_sp
#define PSYS_SPP1       processing_system_unps_logic_spp_tile1_sp
#define PSYS_SPF        processing_system_unps_logic_spf_tile_sp
#define PSYS_ISP0       processing_system_isp_tile0_logic_isp
#define PSYS_ISP1       processing_system_isp_tile1_logic_isp
#define PSYS_ISP2       processing_system_isp_tile2_logic_isp
#define PSYS_ISP3       processing_system_isp_tile3_logic_isp

#define SPC_GMEM_BASE   _hrt_master_to_slave_address_processing_system_unps_logic_spc_tile_sp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define SPF_GMEM_BASE   _hrt_master_to_slave_address_processing_system_unps_logic_spf_tile_sp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define SPP0_GMEM_BASE  _hrt_master_to_slave_address_processing_system_unps_logic_spp_tile0_sp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define SPP1_GMEM_BASE  _hrt_master_to_slave_address_processing_system_unps_logic_spp_tile1_sp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define ISP0_GMEM_BASE  _hrt_master_to_slave_address_processing_system_isp_tile0_logic_isp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define ISP1_GMEM_BASE  _hrt_master_to_slave_address_processing_system_isp_tile1_logic_isp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define ISP2_GMEM_BASE  _hrt_master_to_slave_address_processing_system_isp_tile2_logic_isp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define ISP3_GMEM_BASE  _hrt_master_to_slave_address_processing_system_isp_tile3_logic_isp_cmt_op_to_processing_system_unps_logic_gmem_mem_ip0
#define HOST_GMEM_BASE  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_gmem_sl_in + \
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_gmem_mt_out_to_processing_system_unps_logic_gmem_mem_ip0

#define HOST_SPC_STAT   _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_spc_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_spc_mt_out_to_processing_system_unps_logic_spc_tile_sp_sl_stat_ip
#define HOST_SPF_STAT   _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_spf_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_spf_mt_out_to_processing_system_unps_logic_spf_tile_sp_sl_stat_ip
#define HOST_SPP0_STAT  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_spp0_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_spp0_mt_out_to_processing_system_unps_logic_spp_tile0_sp_sl_stat_ip
#define HOST_SPP1_STAT  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_spp1_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_spp1_mt_out_to_processing_system_unps_logic_spp_tile1_sp_sl_stat_ip
#define HOST_ISP0_STAT  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp0_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp0_mt_out_to_processing_system_isp_tile0_logic_isp_sl_stat_ip
#define HOST_ISP1_STAT  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp1_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp1_mt_out_to_processing_system_isp_tile1_logic_isp_sl_stat_ip
#define HOST_ISP2_STAT  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp2_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp2_mt_out_to_processing_system_isp_tile2_logic_isp_sl_stat_ip
#define HOST_ISP3_STAT  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp3_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp3_mt_out_to_processing_system_isp_tile3_logic_isp_sl_stat_ip

#define HOST_SPP0_DMEM  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_spp0_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_spp0_mt_out_to_processing_system_unps_logic_spp_tile0_sp_sl_dmem_ip
#define HOST_SPP1_DMEM  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_spp1_sl_in +\
                        _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_spp1_mt_out_to_processing_system_unps_logic_spp_tile0_sp_sl_dmem_ip

#define TB_REGS         _hrt_master_to_slave_address_host_op0_to_testbench_tb_regs_slv_in
#define SEC_REGS        _hrt_master_to_slave_address_host_op0_to_testbench_sec_regs_slv_in

#include <mmu.h>
#include "mmu_functions.h"
#include "vied_load_program.h"

/////////////////////////////////////////////////////////////////////////////////////

// To ABs:

  // From HOST to ABs
#define HOST_2_AB_IPFD_ADDR             _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_ipfd_sl_in
#define HOST_2_AB_PSA_ADDR              _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_psa_sl_in
#define HOST_2_AB_PSYS_ISP0_VMEM_ADDR   _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp0_sl_in
#define HOST_2_AB_PSYS_ISP1_VMEM_ADDR   _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp1_sl_in
#define HOST_2_AB_PSYS_ISP2_VMEM_ADDR   _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp2_sl_in
#define HOST_2_AB_PSYS_ISP3_VMEM_ADDR   _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_isp3_sl_in

// From ABs
  // From AB to IPFD
#define AB_2_IPFD_IBUFF_ADDR     _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_ipfd_mt_out_to_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_sl
#define AB_2_IPFD_DMA4_ADDR      _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_ipfd_mt_out_to_processing_system_unps_logic_ipfd_dma4_inst_s0
#define AB_2_IPFD_DMF_ADDR       _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_ipfd_mt_out_to_processing_system_unps_logic_ipfd_dmf_inst_sl_cfg
#define AB_2_IPFD_DMFSYNC_ADDR   _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_ipfd_mt_out_to_processing_system_unps_logic_ipfd_dmf_s_inst_sl_cmd
#define AB_2_IPFD_EQC_ADDR       _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_ipfd_mt_out_to_processing_system_unps_logic_ipfd_eqc_ipfd_sp1
#define AB_2_IPFD_CONFIG_ADDR    _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_ipfd_mt_out_to_processing_system_unps_logic_ipfd_config_bus_inst_sl_ipf
#define AB_2_IPFD_CIO_FIFO_ADDR  _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_ipfd_mt_out_to_processing_system_unps_logic_configbus_ciopipe_mt_ipfd_cio_fifo_sl

//////////////////////////
// From AB to PSA
//////////////////////////
#define AB_2_PSA_ADDR            _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_c_sl
// input slice A psa cluster components
#if 1
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_acc_gp_reg_slv_in 0x7B00
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_PSACfgL0Bus_ctrl_in 0x0
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_PSACfgL1Bus_ctrl_in 0x7000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_PSACfgL2Bus_ctrl_in 0x7D00
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_vec2str_bayer1_sl_cfg 0x7000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_vec2str_bayer2_sl_cfg 0x7100
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_vec2str_yuv1_sl_cfg 0x7200
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_vec2str_rgb1_sl_cfg 0x7400
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_vec2str_rgb2_sl_cfg 0x7300
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_PSA_Irq_Ctrl_slv_in 0x7C00
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_bayer1_sl_cfg 0x7600
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_ack_conv_bayer1_crq_in 0x7D00
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_bayer2_sl_cfg 0x7700
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_ack_conv_bayer2_crq_in 0x7D10
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_yuv1_sl_cfg 0x7800
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_ack_conv_yuv1_crq_in 0x7D20
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_yuv2_sl_cfg 0x7900
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_ack_conv_yuv2_crq_in 0x7D30
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_rgb_sl_cfg 0x7A00
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_str2vec_ack_conv_rgb_crq_in 0x7D40
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_WB_WBACrqBus_ctrl_in 0xB000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_WB_ACB_crq_in 0xB300
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_WB_WBA_crq_in 0xB500
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_WB_BC_crq_in 0xB000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_WB_AckConv_crq_in 0xB400
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_ANR_ANR_CRQBUS_ctrl_in 0xA000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_ANR_ACB_crq_in 0xA000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_ANR_SEARCH_crq_in 0xA200
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_ANR_TRANSFORM_crq_in 0xA300
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_ANR_STITCH_crq_in 0xA900
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_ANR_TILE_crq_in 0xAB00
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Bayer_ANR_AckConv_crq_in 0xA100
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_DmCrqBus_ctrl_in 0xC000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_ACB_crq_in 0xC000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_DM_crq_in 0xC060
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_AckConv_crq_in 0xC050
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_CCM_CCMCrqBus_ctrl_in 0xD000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_CCM_ACB_crq_in 0xD100
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_CCM_CCM_crq_in 0xD600
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_CCM_BDC_crq_in 0xD300
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_CCM_AckConv_crq_in 0xD200
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_GTC_RGBCrqBus_ctrl_in 0x8000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_GTC_ACB_crq_in 0x8800
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_GTC_CSC_CDS_crq_in 0x8A00
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_GTC_GTM_crq_in 0x8000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_GTC_AckConv_crq_in 0x8900
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_YUVAutoCrqBus_ctrl_in 0x9000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_ACB_crq_in 0x9000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_YUV_SPLITTER_crq_in 0x9200
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_IEFD_crq_in 0x9300
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_YDS_crq_in 0x9400
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_TCC_crq_in 0x9500
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_YUV_COLLECTOR_crq_in 0x9800
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_YUV1_Processing_AckConv_crq_in 0x9100
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_DVS_DvsCrqBus_ctrl_in 0x0
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_DVS_ACB_crq_in 0x0
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_DVS_YBIN_crq_in 0x200
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_DVS_DVS_crq_in 0x300
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_DVS_AckConv_crq_in 0x100
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Lace_Stat_L_STAT_CrqBus_ctrl_in 0x5000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Lace_Stat_ACB_crq_in 0x6800
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Lace_Stat_LACE_STAT_crq_in 0x5000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Lace_Stat_AckConv_crq_in 0x6C00

#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_DmCrqBus_ctrl_in 0xC000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_ACB_crq_in 0xC000
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_DM_crq_in 0xC060
#define _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster_Advanced_Demosaic_AckConv_crq_in 0xC050

#define AB_PS_A_PSA_CLUSTER_ADDR    _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_psa_mt_out_to_processing_system_psa_psa_logic_PSA_Cluster

//WBA GA components
#define AB_PS_A_PSA_WB_WBA_ADDR                 HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_WB_WBA_crq_in)
#define AB_PS_A_PSA_WB_ACB_ADDR         HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_WB_ACB_crq_in)
#define AB_PS_A_PSA_WB_BC_ADDR         HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_WB_BC_crq_in)
#define AB_PS_A_PSA_WB_AckConv_ADDR         HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_WB_AckConv_crq_in)

//CCM GA components
#define AB_PS_A_PSA_CCM_ACB_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_CCM_ACB_crq_in)
#define AB_PS_A_PSA_CCM_CCM_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_CCM_CCM_crq_in)
#define AB_PS_A_PSA_CCM_BDC_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_CCM_BDC_crq_in)
#define AB_PS_A_PSA_CCM_AckConv_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_CCM_AckConv_crq_in)
//GTC GA components
#define AB_PS_A_PSA_GTC_ACB_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_GTC_ACB_crq_in)
#define AB_PS_A_PSA_GTC_CSC_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_GTC_CSC_CDS_crq_in)
#define AB_PS_A_PSA_GTC_GTM_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_GTC_GTM_crq_in)
#define AB_PS_A_PSA_GTC_AckConv_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_GTC_AckConv_crq_in)
//DVS GA components
#define AB_PS_A_PSA_DVS_ACB_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_DVS_ACB_crq_in)
#define AB_PS_A_PSA_DVS_YBIN_ADDR               HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_DVS_YBIN_crq_in)
#define AB_PS_A_PSA_DVS_DVS_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_DVS_DVS_crq_in)
#define AB_PS_A_PSA_DVS_AckConv_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_DVS_AckConv_crq_in)
//LACE STAT GA components
#define AB_PS_A_PSA_LACE_STAT_ADDR              HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Lace_Stat_LACE_STAT_crq_in)
#define AB_PS_A_PSA_LACE_ACB_ADDR               HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Lace_Stat_ACB_crq_in)
#define AB_PS_A_PSA_LACE_AckConv_ADDR           HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Lace_Stat_AckConv_crq_in)
//YUV1 GA components
#define AB_PS_A_PSA_YUV1_ACB_ADDR               HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_ACB_crq_in)
#define AB_PS_A_PSA_YUV1_YUV_SPLITTER_ADDR      HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_YUV_SPLITTER_crq_in)
#define AB_PS_A_PSA_YUV1_Y_EE_NR_ADDR        	HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_Y_EE_NR_crq_in)
#define AB_PS_A_PSA_YUV1_YDS_ADDR    		HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_YDS_crq_in)
#define AB_PS_A_PSA_YUV1_TCC_ADDR        	HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_TCC_crq_in)
#define AB_PS_A_PSA_YUV1_YUV_COLLECTOR_ADDR    	HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_YUV_COLLECTOR_crq_in)
#define AB_PS_A_PSA_YUV1_AckConv_ADDR    	HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_AckConv_crq_in)
#define AB_PS_A_PSA_YUV1_IEFD_ADDR              HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_YUV1_Processing_IEFD_crq_in)
//ANR GA components
#define AB_PS_A_PSA_ANR_ACB_ADDR                HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_ANR_ACB_crq_in)
#define AB_PS_A_PSA_ANR_SEARCH_ADDR             HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_ANR_SEARCH_crq_in)
#define AB_PS_A_PSA_ANR_TRANSFORM_ADDR          HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_ANR_TRANSFORM_crq_in)
#define AB_PS_A_PSA_ANR_STITCH_ADDR             HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_ANR_STITCH_crq_in)
#define AB_PS_A_PSA_ANR_TILE_ADDR               HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_ANR_TILE_crq_in)
#define AB_PS_A_PSA_ANR_AckConv_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Bayer_ANR_AckConv_crq_in)
//Demosaic GA components
#define AB_PS_A_PSA_DM_ADDR                     HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Advanced_Demosaic_DM_crq_in)
#define AB_PS_A_PSA_DM_AckConv_ADDR             HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Advanced_Demosaic_AckConv_crq_in)
#define AB_PS_A_PSA_DM_DmCrqBusv_ADDR           HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Advanced_Demosaic_DmCrqBus_ctrl_in)
#define AB_PS_A_PSA_DM_ACB_ADDR                 HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_Advanced_Demosaic_ACB_crq_in)
//Str2Vec
#define AB_PS_A_PSA_STR_2_VEC_BAYER_1_ADDR          HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_bayer1_sl_cfg)
#define AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_1_ADDR  HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_ack_conv_bayer1_crq_in)
#define AB_PS_A_PSA_STR_2_VEC_BAYER_2_ADDR          HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_bayer2_sl_cfg)
#define AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_2_ADDR  HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_ack_conv_bayer2_crq_in)
#define AB_PS_A_PSA_STR_2_VEC_YUV_1_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_yuv1_sl_cfg)
#define AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_1_ADDR    HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_ack_conv_yuv1_crq_in)
#define AB_PS_A_PSA_STR_2_VEC_YUV_2_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_yuv2_sl_cfg)
#define AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_2_ADDR    HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_ack_conv_yuv2_crq_in)
#define AB_PS_A_PSA_STR_2_VEC_RGB_ADDR              HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_rgb_sl_cfg)
#define AB_PS_A_PSA_STR_2_VEC_AckConv_RGB_ADDR      HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_str2vec_ack_conv_rgb_crq_in)
//Vec2Str
#define AB_PS_A_PSA_VEC_2_STR_BAYER_1_ADDR          HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_vec2str_bayer1_sl_cfg)
#define AB_PS_A_PSA_VEC_2_STR_BAYER_2_ADDR          HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_vec2str_bayer2_sl_cfg)
#define AB_PS_A_PSA_VEC_2_STR_YUV_1_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_vec2str_yuv1_sl_cfg)
#define AB_PS_A_PSA_VEC_2_STR_YUV_2_ADDR            HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_vec2str_yuv2_sl_cfg)
#define AB_PS_A_PSA_VEC_2_STR_RGB_ADDR              HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_vec2str_rgb_sl_cfg)
//RGB mux
#define AB_PS_A_PSA_S2V_MUX21_ADDR                  HRTCAT(AB_PS_A_PSA_CLUSTER_ADDR,_acc_gp_reg_slv_in)

// Host to PSA addresses
#define HOST_AB_IS_A_ADDR                 _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_psa_sl_in

//islice psa cluster componentes
  //WBA
#define HOST_AB_PS_A_PSA_WB_WBA_ADDR      (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_WB_WBA_ADDR)
#define HOST_AB_PS_A_PSA_WB_ACB_ADDR      (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_WB_ACB_ADDR)
#define HOST_AB_PS_A_PSA_WB_BC_ADDR       (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_WB_BC_ADDR)
#define HOST_AB_PS_A_PSA_WB_AckConv_ADDR  (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_WB_AckConv_ADDR)

  //CCM
#define HOST_AB_PS_A_PSA_CCM_ACB_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_CCM_ACB_ADDR)
#define HOST_AB_PS_A_PSA_CCM_CCM_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_CCM_CCM_ADDR)
#define HOST_AB_PS_A_PSA_CCM_BDC_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_CCM_BDC_ADDR)
#define HOST_AB_PS_A_PSA_CCM_AckConv_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_CCM_AckConv_ADDR)

  //GTC
#define HOST_AB_PS_A_PSA_GTC_ACB_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_GTC_ACB_ADDR)
#define HOST_AB_PS_A_PSA_GTC_CSC_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_GTC_CSC_ADDR)
#define HOST_AB_PS_A_PSA_GTC_GTM_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_GTC_GTM_ADDR)
#define HOST_AB_PS_A_PSA_GTC_AckConv_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_GTC_AckConv_ADDR)
  //DVS
#define HOST_AB_PS_A_PSA_DVS_ACB_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DVS_ACB_ADDR)
#define HOST_AB_PS_A_PSA_DVS_YBIN_ADDR    (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DVS_YBIN_ADDR)
#define HOST_AB_PS_A_PSA_DVS_DVS_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DVS_DVS_ADDR)
#define HOST_AB_PS_A_PSA_DVS_AckConv_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DVS_AckConv_ADDR)
  //LACE STAT
#define HOST_AB_PS_A_PSA_LACE_STAT_ADDR   (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_LACE_STAT_ADDR)
#define HOST_AB_PS_A_PSA_LACE_ACB_ADDR    (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_LACE_ACB_ADDR)
#define HOST_AB_PS_A_PSA_LACE_AckConv_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_LACE_AckConv_ADDR)
  //YUV1
#define HOST_AB_PS_A_PSA_YUV1_ACB_ADDR          (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_ACB_ADDR)
#define HOST_AB_PS_A_PSA_YUV1_YUV_SPLITTER_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_YUV_SPLITTER_ADDR)
// #define HOST_AB_PS_A_PSA_YUV1_Y_EE_NR_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_Y_EE_NR_ADDR)
#define HOST_AB_PS_A_PSA_YUV1_IEFD_ADDR         (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_IEFD_ADDR)
#define HOST_AB_PS_A_PSA_YUV1_YDS_ADDR          (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_YDS_ADDR)
#define HOST_AB_PS_A_PSA_YUV1_TCC_ADDR          (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_TCC_ADDR)
#define HOST_AB_PS_A_PSA_YUV1_YUV_COLLECTOR_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_YUV_COLLECTOR_ADDR)
#define HOST_AB_PS_A_PSA_YUV1_AckConv_ADDR      (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_YUV1_AckConv_ADDR)

  //ANR
#define HOST_AB_PS_A_PSA_ANR_ACB_ADDR           (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_ANR_ACB_ADDR)
#define HOST_AB_PS_A_PSA_ANR_SEARCH_ADDR        (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_ANR_SEARCH_ADDR)
#define HOST_AB_PS_A_PSA_ANR_TRANSFORM_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_ANR_TRANSFORM_ADDR)
#define HOST_AB_PS_A_PSA_ANR_STITCH_ADDR        (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_ANR_STITCH_ADDR)
#define HOST_AB_PS_A_PSA_ANR_TILE_ADDR          (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_ANR_TILE_ADDR)
#define HOST_AB_PS_A_PSA_ANR_AckConv_ADDR       (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_ANR_AckConv_ADDR)
  //Demosaic
#define HOST_AB_PS_A_PSA_DM_ADDR                (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DM_ADDR)
#define HOST_AB_PS_A_PSA_DM_AckConv_ADDR        (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DM_AckConv_ADDR)
#define HOST_AB_PS_A_PSA_DM_DmCrqBusv_ADDR      (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DM_DmCrqBusv_ADDR)
#define HOST_AB_PS_A_PSA_DM_ACB_ADDR            (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_DM_ACB_ADDR)
// RGB mux
#define HOST_AB_PS_A_PSA_S2V_MUX21_ADDR         (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_S2V_MUX21_ADDR)

  // From PSA to EVQs - of ISP
#define PSA_EQC_ISP0_EVQ_ADDR _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_isp_tile0_logic_evq_s_ip
#define PSA_EQC_ISP1_EVQ_ADDR _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_isp_tile1_logic_evq_s_ip
#define PSA_EQC_ISP2_EVQ_ADDR _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_isp_tile2_logic_evq_s_ip
#define PSA_EQC_ISP3_EVQ_ADDR _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_isp_tile3_logic_evq_s_ip

// From PSA to EVQs - of SP
#define PSA_EQC_SPC_EVQ_ADDR  _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_unps_logic_spc_tile_evq_s_ip
#define PSA_EQC_SPP0_EVQ_ADDR _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_unps_logic_spp_tile0_evq_s_ip
#define PSA_EQC_SPP1_EVQ_ADDR _hrt_master_to_slave_address_processing_system_unps_logic_eqc_psa_mp0_to_processing_system_unps_logic_spp_tile1_evq_s_ip

//Str2Vec
#define HOST_AB_PS_A_PSA_STR_2_VEC_BAYER_1_ADDR         (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_BAYER_1_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_1_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_1_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_BAYER_2_ADDR         (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_BAYER_2_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_2_ADDR (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_AckConv_BAYER_2_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_YUV_1_ADDR           (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_YUV_1_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_1_ADDR   (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_1_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_YUV_2_ADDR           (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_YUV_2_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_2_ADDR   (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_AckConv_YUV_2_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_RGB_ADDR             (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_RGB_ADDR)
#define HOST_AB_PS_A_PSA_STR_2_VEC_AckConv_RGB_ADDR     (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_STR_2_VEC_AckConv_RGB_ADDR)
//Vec2Str
#define HOST_AB_PS_A_PSA_VEC_2_STR_BAYER_1_ADDR         (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_VEC_2_STR_BAYER_1_ADDR)
#define HOST_AB_PS_A_PSA_VEC_2_STR_BAYER_2_ADDR         (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_VEC_2_STR_BAYER_2_ADDR)
#define HOST_AB_PS_A_PSA_VEC_2_STR_YUV_1_ADDR           (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_VEC_2_STR_YUV_1_ADDR)
#define HOST_AB_PS_A_PSA_VEC_2_STR_YUV_2_ADDR           (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_VEC_2_STR_YUV_2_ADDR)
#define HOST_AB_PS_A_PSA_VEC_2_STR_RGB_ADDR             (HOST_AB_IS_A_ADDR + AB_PS_A_PSA_VEC_2_STR_RGB_ADDR)

typedef enum {
    PSA_ACK_WB,
    PSA_ACK_ANR,
    PSA_ACK_DM,
    PSA_ACK_CCM,
    PSA_ACK_GTM,
    PSA_ACK_YUV,
    PSA_ACK_LACE_STAT,
    PSA_ACK_DVS,
    PSA_ACK_STR2VEC_BAYER0,
    PSA_ACK_STR2VEC_BAYER1,
    PSA_ACK_STR2VEC_RGB,
    PSA_ACK_STR2VEC_YUV0,
    PSA_ACK_STR2VEC_YUV1,
    PSA_ACK_NUM_OF_ACKS        //This value must always be last in this enum!!!
}PSA_ACKS;

#endif

/////////////////////////////////////////////////////////////////////////////////////

  // Processing system isa cluster components
// Access Blocker:
#define HOST_AB_PS_ISLC_ADDR  _hrt_master_to_slave_address_host_op0_to_processing_system_unps_logic_configbus_ab_mt_islc_sl_in

#define AB_PS_ISLC_ISA_CLUSTER_ADDR _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_islc_mt_out_to_processing_system_input_slice_light_logic_isa_isa_logic_ISA_Cluster
//Input Corr GA components
#define AB_PS_ISLC_ISA_INPUT_CORR_ACB_ADDR      HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Input_Corr_ACB_crq_in)
#define AB_PS_ISLC_ISA_INPUT_CORR_INL_ADDR      HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Input_Corr_INL_crq_in)
#define AB_PS_ISLC_ISA_INPUT_CORR_GBL_ADDR      HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Input_Corr_GBL_crq_in)
#define AB_PS_ISLC_ISA_INPUT_CORR_PCLN_ADDR     HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Input_Corr_PCLN_crq_in)
#define AB_PS_ISLC_ISA_INPUT_CORR_ACK_CONV_ADDR HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Input_Corr_AckConv_crq_in)
//AWB statistics GA components
#define AB_PS_ISLC_ISA_AWB_ACB_ADDR             HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AWB_ACB_crq_in)
#define AB_PS_ISLC_ISA_AWB_AWRG_ADDR            HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AWB_AWRG_crq_in)
#define AB_PS_ISLC_ISA_AWB_ACK_CONV_ADDR        HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AWB_AckConv_crq_in)
//AF AWB FR GRD statistics GA components
#define AB_PS_ISLC_ISA_AF_AWB_FR_GRD_ACB_ADDR            HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AF_AWB_FR_ACB_crq_in)
#define AB_PS_ISLC_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR  HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AF_AWB_FR_AF_AWB_FR_GRD_crq_in)
#define AB_PS_ISLC_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR       HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AF_AWB_FR_AckConv_crq_in)
//AE statistics GA componnents
#define AB_PS_ISLC_ISA_AE_STATS_ACB_ADDR            HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AE_ACB_crq_in)
#define AB_PS_ISLC_ISA_AE_STATS_WGHT_HIST_ADDR      HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AE_WGHT_HIST_crq_in)
#define AB_PS_ISLC_ISA_AE_STATS_CCM_ADDR            HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AE_AE_CCM_crq_in)
#define AB_PS_ISLC_ISA_AE_STATS_ACK_CONV_ADDR       HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Stat_AE_AckConv_crq_in)
//GDDPC GA componnents
#define AB_PS_ISLC_ISA_GDDPC_ACB_ADDR               HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Dpc_ACB_crq_in)
#define AB_PS_ISLC_ISA_GDDPC_GDDPC_ADDR             HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Dpc_GDDPC_crq_in)
#define AB_PS_ISLC_ISA_GDDPC_ACK_CONV_ADDR          HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Dpc_AckConv_crq_in)
//LSC GA componnents
#define AB_PS_ISLC_ISA_LSC_ACB_ADDR                 HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Lsc_ACB_crq_in)
#define AB_PS_ISLC_ISA_LSC_LSC_ADDR                 HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Lsc_LSC_crq_in)
#define AB_PS_ISLC_ISA_LSC_ACK_CONV_ADDR            HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Lsc_AckConv_crq_in)
//Bayer Scaler componenets
#define AB_PS_ISLC_ISA_SCALER_ACB_ADDR              HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Scaler_ACB_crq_in)
#define AB_PS_ISLC_ISA_SCALER_ACK_CONV_ADDR         HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Scaler_AckConv_crq_in)
#define AB_PS_ISLC_ISA_SCALER_SCALER_ADDR           HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_Bayer_Scaler_SCALER_crq_in)

//ISA GP regs
#define AB_PS_ISLC_ISA_GP_REGS_ADDR                 HRTCAT(AB_PS_ISLC_ISA_CLUSTER_ADDR,_isa_gp_reg_slv_in)

/////////////////////////////////////////////////////////////////////////////////////

//// Host addresses:
//islice isa cluster componentes
//Input Corr GA components
#define HOST_PS_ISLC_ISA_INPUT_CORR_ACB_ADDR        (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_INPUT_CORR_ACB_ADDR)
#define HOST_PS_ISLC_ISA_INPUT_CORR_INL_ADDR        (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_INPUT_CORR_INL_ADDR)
#define HOST_PS_ISLC_ISA_INPUT_CORR_GBL_ADDR        (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_INPUT_CORR_GBL_ADDR)
#define HOST_PS_ISLC_ISA_INPUT_CORR_PCLN_ADDR       (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_INPUT_CORR_PCLN_ADDR)
#define HOST_PS_ISLC_ISA_INPUT_CORR_ACK_CONV_ADDR   (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_INPUT_CORR_ACK_CONV_ADDR)
//WBA  GA components
#define HOST_PS_ISLC_ISA_AWB_ACB_ADDR               (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AWB_ACB_ADDR)
#define HOST_PS_ISLC_ISA_AWB_AWRG_ADDR              (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AWB_AWRG_ADDR)
#define HOST_PS_ISLC_ISA_AWB_ACK_CONV_ADDR          (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AWB_ACK_CONV_ADDR)
//AF AWB FR GRD GA components
#define HOST_PS_ISLC_ISA_AF_AWB_FR_GRD_ACB_ADDR             (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AF_AWB_FR_GRD_ACB_ADDR)
#define HOST_PS_ISLC_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR   (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR)
#define HOST_PS_ISLC_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR        (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR)
//Input Corr GA components
#define HOST_PS_ISLC_ISA_AE_STATS_ACB_ADDR          (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AE_STATS_ACB_ADDR)
#define HOST_PS_ISLC_ISA_AE_STATS_WGHT_HIST_ADDR    (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AE_STATS_WGHT_HIST_ADDR)
#define HOST_PS_ISLC_ISA_AE_STATS_CCM_ADDR          (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AE_STATS_CCM_ADDR)
#define HOST_PS_ISLC_ISA_AE_STATS_ACK_CONV_ADDR     (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_AE_STATS_ACK_CONV_ADDR)
//GDDPC GA components
#define HOST_PS_ISLC_ISA_GDDPC_ACB_ADDR             (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_GDDPC_ACB_ADDR)
#define HOST_PS_ISLC_ISA_GDDPC_GDDPC_ADDR           (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_GDDPC_GDDPC_ADDR)
#define HOST_PS_ISLC_ISA_GDDPC_ACK_CONV_ADDR        (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_GDDPC_ACK_CONV_ADDR)
//LSC GA components
#define HOST_PS_ISLC_ISA_LSC_ACB_ADDR               (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_LSC_ACB_ADDR)
#define HOST_PS_ISLC_ISA_LSC_LSC_ADDR               (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_LSC_LSC_ADDR)
#define HOST_PS_ISLC_ISA_LSC_ACK_CONV_ADDR          (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_LSC_ACK_CONV_ADDR)
//Bayer Scaler GA components
#define HOST_PS_ISLC_ISA_SCALER_ACB_ADDR            (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_SCALER_ACB_ADDR)
#define HOST_PS_ISLC_ISA_SCALER_ACK_CONV_ADDR       (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_SCALER_ACK_CONV_ADDR)
#define HOST_PS_ISLC_ISA_SCALER_SCALER_ADDR         (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_SCALER_SCALER_ADDR)
//ISA GP regs
#define HOST_PS_ISLC_ISA_GP_REGS_ADDR               (HOST_AB_PS_ISLC_ADDR + AB_PS_ISLC_ISA_GP_REGS_ADDR)

/////////////////////////////////////////////////////////////////////////////////////

  // From AB to ISP VMEMs
#define AB_2_PSYS_ISP0_VMEM_ADDR      _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp0_mt_out_to_processing_system_isp_tile0_logic_isp_sl_vmem_ip
#define AB_2_PSYS_ISP1_VMEM_ADDR      _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp0_mt_out_to_processing_system_isp_tile1_logic_isp_sl_vmem_ip
#define AB_2_PSYS_ISP2_VMEM_ADDR      _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp0_mt_out_to_processing_system_isp_tile2_logic_isp_sl_vmem_ip
#define AB_2_PSYS_ISP3_VMEM_ADDR      _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_isp0_mt_out_to_processing_system_isp_tile3_logic_isp_sl_vmem_ip

  // From AB to Global MEM
#define AB_2_GMEM_ADDR           _hrt_master_to_slave_address_processing_system_unps_logic_configbus_ab_mt_gmem_mt_out_to_processing_system_unps_logic_configbus_ab_mt_isp1_sl_in

// From Host to IPFD
#define HOST_2_IPFD_IBUFF_ADDR   ((HOST_2_AB_IPFD_ADDR) + (AB_2_IPFD_IBUFF_ADDR))
#define HOST_2_IPFD_DMA4_ADDR    ((HOST_2_AB_IPFD_ADDR) + (AB_2_IPFD_DMA4_ADDR))
#define HOST_2_IPFD_DMF_ADDR     ((HOST_2_AB_IPFD_ADDR) + (AB_2_IPFD_DMF_ADDR))
#define HOST_2_IPFD_DMFSYNC_ADDR ((HOST_2_AB_IPFD_ADDR) + (AB_2_IPFD_DMFSYNC_ADDR))
#define HOST_2_IPFD_EQC_ADDR     ((HOST_2_AB_IPFD_ADDR) + (AB_2_IPFD_EQC_ADDR))
#define HOST_2_IPFD_CONFIG_ADDR  ((HOST_2_AB_IPFD_ADDR) + (AB_2_IPFD_CONFIG_ADDR))

// From Host to ISP VMEMs
#define HOST_2_PSYS_ISP0_VMEM_ADDR    ((HOST_2_AB_PSYS_ISP0_VMEM_ADDR) + (AB_2_PSYS_ISP0_VMEM_ADDR))
#define HOST_2_PSYS_ISP1_VMEM_ADDR    ((HOST_2_AB_PSYS_ISP1_VMEM_ADDR) + (AB_2_PSYS_ISP1_VMEM_ADDR))
#define HOST_2_PSYS_ISP2_VMEM_ADDR    ((HOST_2_AB_PSYS_ISP2_VMEM_ADDR) + (AB_2_PSYS_ISP2_VMEM_ADDR))
#define HOST_2_PSYS_ISP3_VMEM_ADDR    ((HOST_2_AB_PSYS_ISP3_VMEM_ADDR) + (AB_2_PSYS_ISP3_VMEM_ADDR))

// From IPFD to ...
#define IPFD_2_SP_CTRL_TILE_EVQ_ADDR    _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_eqc_ipfd_mp0_to_processing_system_unps_logic_spc_tile_evq_s_ip
#define IPFD_2_SP_PROXY0_TILE_EVQ_ADDR  _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_eqc_ipfd_mp0_to_processing_system_unps_logic_spp_tile0_evq_s_ip
#define IPFD_2_SP_PROXY1_TILE_EVQ_ADDR  _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_eqc_ipfd_mp0_to_processing_system_unps_logic_spp_tile1_evq_s_ip
#define IPFD_2_SP_FLOATP_TILE_EVQ_ADDR  _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_eqc_ipfd_mp0_to_processing_system_unps_logic_spf_tile_evq_s_ip
#define IPFD_2_ISP_TILE0_EVQ_ADDR       _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_eqc_ipfd_mp0_to_processing_system_isp_tile0_logic_evq_s_ip

#define IPFD_DMA4_MA_2_GMEM_SL1_ADDR    _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_dma4_inst_mA_to_processing_system_unps_logic_databus_bus_P7_gmem_sl1

// From ISP0 Tile to IPFD
#define ISP0_TILE_2_IPFD_SL_VEC_ADDR    _hrt_master_to_slave_address_processing_system_isp_tile0_logic_isp_mt_bmem_op_to_processing_system_unps_logic_ipfd_data_bus_inst_s0

// From H

#endif /* _PSYS_H_ */

