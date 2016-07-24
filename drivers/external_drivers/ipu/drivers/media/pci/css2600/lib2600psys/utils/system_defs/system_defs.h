#ifndef _system_defs_h_
#define _system_defs_h_

#include <input_system_system.h>

#define XMEM_WIDTH                  512
#define ISL_INPUT_WORD_SIZE         64

#define RTL_FREQUENCY			533000000 // 533MHz
#define FPGA_FREQUENCY			25000000 // 25MHz

///////////////////////////
////// MG PARAMETERS //////
#define STYPE					0
#define DTYPE_RAW8_BITS			8
#define DTYPE_RAW10_BITS		10
#define MG_PPC					4
#define LINE_HEADER				8

//////////// ID's /////////
#define MIPI_PKT_GEN0               input_system_unis_logic_mipi_gen_mipi_pkt_gen0
#define MIPI_PKT_GEN1               input_system_unis_logic_mipi_gen_mipi_pkt_gen1

#define CELL                        input_system_unis_logic_sp_control_tile_sp
#define HOST_DDR_ID                 testbench_ddr
//////////////////////////

//////////////////////////
////// DMA PARAMETERS ////
#define DMA_EXT0_NR_GLOBAL_SETS     _hrt_device_input_system_unis_logic_dma_ext0_dma_property_GlobalSets
#define DMA_EXT0_NR_REQUEST_BANKS   _hrt_device_input_system_unis_logic_dma_ext0_dma_property_RequestBanks
#define DMA_EXT0_NR_UNIT_BANKS      _hrt_device_input_system_unis_logic_dma_ext0_dma_property_UnitBanks
#define DMA_EXT0_NR_SPAN_BANKS      _hrt_device_input_system_unis_logic_dma_ext0_dma_property_SpanBanks
#define DMA_EXT0_NR_TERMINAL_BANKS  _hrt_device_input_system_unis_logic_dma_ext0_dma_property_TerminalBanks
#define DMA_EXT0_NR_CHANNEL_BANKS   _hrt_device_input_system_unis_logic_dma_ext0_dma_property_ChannelBanks

#define DMA_EXT0_NR_UNITS           _hrt_device_input_system_unis_logic_dma_ext0_dma_property_Units
#define DMA_EXT0_NR_SPANS           _hrt_device_input_system_unis_logic_dma_ext0_dma_property_Spans
#define DMA_EXT0_NR_TERMINALS       _hrt_device_input_system_unis_logic_dma_ext0_dma_property_Terminals
#define DMA_EXT0_NR_CHANNELS        _hrt_device_input_system_unis_logic_dma_ext0_dma_property_ChannelBanks

#define DMA_EXT1_NR_GLOBAL_SETS     _hrt_device_input_system_unis_logic_dma_ext1_dma_property_GlobalSets
#define DMA_EXT1_NR_REQUEST_BANKS   _hrt_device_input_system_unis_logic_dma_ext1_dma_property_RequestBanks
#define DMA_EXT1_NR_UNIT_BANKS      _hrt_device_input_system_unis_logic_dma_ext1_dma_property_UnitBanks
#define DMA_EXT1_NR_SPAN_BANKS      _hrt_device_input_system_unis_logic_dma_ext1_dma_property_SpanBanks
#define DMA_EXT1_NR_TERMINAL_BANKS  _hrt_device_input_system_unis_logic_dma_ext1_dma_property_TerminalBanks
#define DMA_EXT1_NR_CHANNEL_BANKS   _hrt_device_input_system_unis_logic_dma_ext1_dma_property_ChannelBanks

#define DMA_EXT1_NR_UNITS           _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Units
#define DMA_EXT1_NR_SPANS           _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Spans
#define DMA_EXT1_NR_TERMINALS       _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Terminals
#define DMA_EXT1_NR_CHANNELS        _hrt_device_input_system_unis_logic_dma_ext1_dma_property_Channels

#define DMA_ISL_NR_GLOBAL_SETS      _hrt_device_input_system_is_a_logic_dma_property_GlobalSets
#define DMA_ISL_NR_REQUEST_BANKS    _hrt_device_input_system_is_a_logic_dma_property_RequestBanks
#define DMA_ISL_NR_UNIT_BANKS       _hrt_device_input_system_is_a_logic_dma_property_UnitBanks
#define DMA_ISL_NR_SPAN_BANKS       _hrt_device_input_system_is_a_logic_dma_property_SpanBanks
#define DMA_ISL_NR_TERMINAL_BANKS   _hrt_device_input_system_is_a_logic_dma_property_TerminalBanks
#define DMA_ISL_NR_CHANNEL_BANKS    _hrt_device_input_system_is_a_logic_dma_property_ChannelBanks

#define DMA_ISL_NR_UNITS            _hrt_device_input_system_is_a_logic_dma_property_Units
#define DMA_ISL_NR_SPANS            _hrt_device_input_system_is_a_logic_dma_property_Spans
#define DMA_ISL_NR_TERMINALS        _hrt_device_input_system_is_a_logic_dma_property_Terminals
#define DMA_ISL_NR_CHANNELS         _hrt_device_input_system_is_a_logic_dma_property_Channels

//////////////////////////////////
/////// Str2MMIO PARAMETERS //////
  //csi2
#define S2M_CSI2_A_NR_SIDS          _hrt_device_input_system_csi2_logic_s2m_a_property_NofSids
#define S2M_CSI2_B_NR_SIDS          _hrt_device_input_system_csi2_logic_s2m_b_property_NofSids
#define S2M_CSI2_C_NR_SIDS          _hrt_device_input_system_csi2_logic_s2m_c_property_NofSids
#define S2M_CSI2_D_NR_SIDS          _hrt_device_input_system_csi2_logic_s2m_d_property_NofSids
  //mipi gen
#define S2M_MG0_NR_SIDS             _hrt_device_input_system_unis_logic_mipi_gen_mipi_s2m0_property_NofSids
#define S2M_MG1_NR_SIDS             _hrt_device_input_system_unis_logic_mipi_gen_mipi_s2m1_property_NofSids
  //islice
#define S2M_ISL0_NR_SIDS            _hrt_device_input_system_is_a_logic_pixel_s2m0_property_NofSids
#define S2M_ISL1_NR_SIDS            _hrt_device_input_system_is_a_logic_pixel_s2m1_property_NofSids

//////////////////////////////////////
//////// IBufCntrl PARAMETERS ////////
 // csi2
#define IBC2600_CSI2_NR_SIDS          _hrt_device_input_system_csi2_logic_ibuf_ctrl_property_nr_sid_procs
#define IBC2600_CSI2_NR_IRQ_CHECKS    _hrt_device_input_system_csi2_logic_ibuf_ctrl_property_nr_irq_check_units
#define IBC2600_CSI2_NR_FEEDERS       _hrt_device_input_system_csi2_logic_ibuf_ctrl_property_nr_feeders
#define IBC2600_CSI2_IBUF_DW          _hrt_device_input_system_csi2_logic_ibuf_ctrl_property_ibuf_dw
 // mipi gen
#define IBC2600_MG_NR_SIDS          _hrt_device_input_system_unis_logic_mipi_gen_ibuf_ctrl_property_nr_sid_procs
#define IBC2600_MG_NR_IRQ_CHECKS    _hrt_device_input_system_unis_logic_mipi_gen_ibuf_ctrl_property_nr_irq_check_units
#define IBC2600_MG_NR_FEEDERS       _hrt_device_input_system_unis_logic_mipi_gen_ibuf_ctrl_property_nr_feeders
#define IBC2600_MG_IBUF_DW          _hrt_device_input_system_unis_logic_mipi_gen_ibuf_ctrl_property_ibuf_dw
 // islice
#define IBC2600_ISL_NR_SIDS         _hrt_device_input_system_is_a_logic_ibuf_ctrl_property_nr_sid_procs
#define IBC2600_ISL_NR_IRQ_CHECKS   _hrt_device_input_system_is_a_logic_ibuf_ctrl_property_nr_irq_check_units
#define IBC2600_ISL_NR_FEEDERS      _hrt_device_input_system_is_a_logic_ibuf_ctrl_property_nr_feeders
#define IBC2600_ISL_IBUF_DW         _hrt_device_input_system_is_a_logic_ibuf_ctrl_property_ibuf_dw

//////////////////////////
/////// ADDRESSES ////////
// HOST:

  //testbench
#define HOST_DDR_ADDR               _hrt_master_to_slave_address_host_op0_to_testbench_ddr_ip0
#define HOST_STAT_CTRL_SPC_ADDR		(_hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in +\
									_hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_sp_mt_out_to_input_system_unis_logic_sp_control_tile_sp_sl_stat_ip)

/////////////////////////
// To AB's:

  // host
#define HOST_AB_CSI2_3_ADDR         _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_sl_in
#define HOST_AB_SP_ADDR             _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in
#define HOST_AB_DMA_EXT0_ADDR       _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext0_sl_in
#define HOST_AB_DMA_EXT1_ADDR       _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext1_sl_in
#define HOST_AB_IS_A_ADDR           _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_is_a_sl_in
#define HOST_AB_UNIS_BUS2_ADDR      _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in
#define HOST_AB_MIPI_BUF_ADDR       _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_mipibuf_sl_in
#define HOST_AB_PIX_BUF_ADDR        _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_pixelbuf_sl_in

  // EQC
#define EQC_CSI2_AB_SP_ADDR         _hrt_master_to_slave_address_input_system_csi2_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in
#define EQC_CSI2_AB_DMA_EXT0_ADDR   _hrt_master_to_slave_address_input_system_csi2_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext0_sl_in
#define EQC_CSI2_AB_DMA_EXT1_ADDR   _hrt_master_to_slave_address_input_system_csi2_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext1_sl_in
#define EQC_CSI2_AB_IS_A_ADDR       _hrt_master_to_slave_address_input_system_csi2_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_ab_sl_in

#define EQC_IS_A_AB_SP_ADDR         _hrt_master_to_slave_address_input_system_is_a_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in
#define EQC_IS_A_AB_DMA_EXT0_ADDR   _hrt_master_to_slave_address_input_system_is_a_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext0_sl_in
#define EQC_IS_A_AB_DMA_EXT1_ADDR   _hrt_master_to_slave_address_input_system_is_a_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext1_sl_in
#define EQC_IS_A_AB_CSI2_3_ADDR     _hrt_master_to_slave_address_input_system_is_a_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_sl_in
#define EQC_IS_A_AB_BUS2_ADDR       _hrt_master_to_slave_address_input_system_is_a_logic_loc_bus_bus0_b_mt_ctrl_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in
#define EQC_IS_A_AB_UNIS_BUS2_ADDR  _hrt_master_to_slave_address_input_system_is_a_logic_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in

#define EQC_MG_AB_SP_ADDR           _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in
#define EQC_MG_AB_DMA_EXT0_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext0_sl_in
#define EQC_MG_AB_DMA_EXT1_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext1_sl_in
#define EQC_MG_AB_IS_A_ADDR         _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_is_a_sl_in
#define EQC_MG_AB_UNIS_BUS2_ADDR    _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in
#define EQC_MG_AB_PIX_BUF_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_pixelbuf_sl_in

#define EQC_DMA_EXT0_AB_SP_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_dma_ext0_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in
#define EQC_DMA_EXT0_AB_BUS2_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_dma_ext0_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in
#define EQC_DMA_EXT0_AB_IS_A_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_dma_ext0_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_is_a_sl_in
#define EQC_DMA_EXT0_AB_CSI2_3_ADDR _hrt_master_to_slave_address_input_system_unis_logic_dma_ext0_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_sl_in

#define EQC_DMA_EXT1_AB_SP_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_dma_ext1_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in
#define EQC_DMA_EXT1_AB_BUS2_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_dma_ext1_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in
#define EQC_DMA_EXT1_AB_IS_A_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_dma_ext1_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_is_a_sl_in
#define EQC_DMA_EXT1_AB_CSI2_3_ADDR _hrt_master_to_slave_address_input_system_unis_logic_dma_ext1_eqc_mp0_to_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_sl_in

  // SP:
#define SP_AB_UNIS_BUS2_ADDR        _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_sp_bus_mt0_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in
#define SP_AB_IS_A_ADDR             _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_sp_bus_mt0_to_input_system_unis_logic_ctrl_bus_ab_mt_is_a_sl_in
#define SP_AB_CSI2_3_ADDR           _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_sp_bus_mt0_to_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_sl_in

// PMA:
#define PMA_IF_AB_UNIS_BUS2_ADDR    _hrt_master_to_slave_address_input_system_unis_logic_pma_if_ctrl_out_to_input_system_unis_logic_ctrl_bus_ab_mt_bus2_sl_in
#define PMA_IF_AB_IS_A_ADDR         _hrt_master_to_slave_address_input_system_unis_logic_pma_if_ctrl_out_to_input_system_unis_logic_ctrl_bus_ab_mt_is_a_sl_in
#define PMA_IF_AB_CSI2_3_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_pma_if_ctrl_out_to_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_sl_in

///////////////
// To EQC
  // IBufCtrl
#define IBC2600_CSI2_EQC_ADDR       _hrt_master_to_slave_address_input_system_csi2_logic_ibuf_ctrl_mt_to_input_system_csi2_logic_eqc_sp0
#define IBC2600_MG_EQC_ADDR         _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_ibuf_ctrl_mt_to_input_system_unis_logic_mipi_gen_eqc_sp0
#define IBC2600_IS_A_EQC_ADDR       _hrt_master_to_slave_address_input_system_is_a_logic_ibuf_ctrl_mt_to_input_system_is_a_logic_eqc_sp0

  //ISA
#define ISA_IS_A_EQC_ADDR           _hrt_master_to_slave_address_input_system_is_a_logic_isa_isa_logic_ISA_Cluster_ISAAckBus_ack_bus_out_to_input_system_is_a_logic_eqc_sp0
//////////////
// From AB's:

 // csi2
#define AB_CSI2_3_IBC2600_CSI2_ADDR _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_ibuf_ctrl_sl
#define AB_CSI2_3_S2M_CSI2_A_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_s2m_a_sl_in
#define AB_CSI2_3_S2M_CSI2_B_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_s2m_b_sl_in
#define AB_CSI2_3_S2M_CSI2_C_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_s2m_c_sl_in
#define AB_CSI2_3_S2M_CSI2_D_ADDR   _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_s2m_d_sl_in
#define AB_CSI2_DPHY_RX_A_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_csi_rx_a_ctrl_rx
#define AB_CSI2_DPHY_RX_B_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_csi_rx_b_ctrl_rx
#define AB_CSI2_DPHY_RX_C_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_csi_rx_c_ctrl_rx
#define AB_CSI2_DPHY_RX_D_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_csi2_3ph_csi2_mt_out_to_input_system_csi2_logic_csi_rx_d_ctrl_rx

// input slice A
#define AB_IS_A_IBC2600_ISL_ADDR    _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_ibuf_ctrl_sl
#define AB_IS_A_S2M_ISL0_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_pixel_s2m0_sl_in
#define AB_IS_A_S2M_ISL1_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_pixel_s2m1_sl_in
#define AB_IS_A_DMA_ISL_ADDR        _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_dma_s0
#define AB_IS_A_CIO2STR_ADDR        _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_cio2str_sl_cfg
#define AB_IS_A_PIX_FORM_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_pf_ctrl_in
#define AB_IS_A_MIPI_BE0_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_mipi_be0_ctrl_in
#define AB_IS_A_MIPI_BE1_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_mipi_be1_ctrl_in
#define AB_IS_A_GPREG_ADDR          _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_gpreg_slv_in
  // input slice A isa cluster components
#define AB_IS_A_ISA_CLUSTER_ADDR    _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_is_a_mt_out_to_input_system_is_a_logic_isa_isa_logic_ISA_Cluster
//Input Corr GA componnents
#define AB_IS_A_ISA_INPUT_CORR_ACB_ADDR        HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Input_Corr_ACB_crq_in)
#define AB_IS_A_ISA_INPUT_CORR_INL_ADDR        HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Input_Corr_INL_crq_in)
#define AB_IS_A_ISA_INPUT_CORR_GBL_ADDR        HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Input_Corr_GBL_crq_in)
#define AB_IS_A_ISA_INPUT_CORR_PCLN_ADDR       HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Input_Corr_PCLN_crq_in)
#define AB_IS_A_ISA_INPUT_CORR_ACK_CONV_ADDR   HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Input_Corr_AckConv_crq_in)
//AWB statistics GA componnents
#define AB_IS_A_ISA_AWB_ACB_ADDR  	     HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AWB_ACB_crq_in)
#define AB_IS_A_ISA_AWB_AWRG_ADDR            HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AWB_AWRG_crq_in)
#define AB_IS_A_ISA_AWB_ACK_CONV_ADDR        HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AWB_AckConv_crq_in)
//AF AWB FR GRD statistics GA components
#define AB_IS_A_ISA_AF_AWB_FR_GRD_ACB_ADDR            HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AF_AWB_FR_ACB_crq_in)
#define AB_IS_A_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR  HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AF_AWB_FR_AF_AWB_FR_GRD_crq_in)
#define AB_IS_A_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR       HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AF_AWB_FR_AckConv_crq_in)
//AE statistics GA componnents
#define AB_IS_A_ISA_AE_STATS_ACB_ADDR      	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AE_ACB_crq_in)
#define AB_IS_A_ISA_AE_STATS_WGHT_HIST_ADDR	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AE_WGHT_HIST_crq_in)
#define AB_IS_A_ISA_AE_STATS_CCM_ADDR       	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AE_AE_CCM_crq_in)
#define AB_IS_A_ISA_AE_STATS_ACK_CONV_ADDR  	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Stat_AE_AckConv_crq_in)
//GDDPC GA componnents
#define AB_IS_A_ISA_GDDPC_ACB_ADDR         	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Dpc_ACB_crq_in)
#define AB_IS_A_ISA_GDDPC_GDDPC_ADDR       	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Dpc_GDDPC_crq_in)
#define AB_IS_A_ISA_GDDPC_ACK_CONV_ADDR    	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Dpc_AckConv_crq_in)
//LSC GA componnents
#define AB_IS_A_ISA_LSC_ACB_ADDR           	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Lsc_ACB_crq_in)
#define AB_IS_A_ISA_LSC_LSC_ADDR           	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Lsc_LSC_crq_in)
#define AB_IS_A_ISA_LSC_ACK_CONV_ADDR     	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Lsc_AckConv_crq_in)
//Bayer Scaler componenets
#define AB_IS_A_ISA_SCALER_ACB_ADDR 		HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Scaler_ACB_crq_in)
#define AB_IS_A_ISA_SCALER_ACK_CONV_ADDR 	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Scaler_AckConv_crq_in)
#define AB_IS_A_ISA_SCALER_SCALER_ADDR 		HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_Bayer_Scaler_SCALER_crq_in)

//ISA GP regs
#define AB_IS_A_ISA_GP_REGS_ADDR	   	HRTCAT(AB_IS_A_ISA_CLUSTER_ADDR,_isa_gp_reg_slv_in)




  // unis bus2
#define AB_UNIS_BUS2_IBC2600_MG_ADDR  _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_bus2_mt_out_to_input_system_unis_logic_mipi_gen_ibuf_ctrl_sl
#define AB_UNIS_BUS2_S2M_MG0_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_bus2_mt_out_to_input_system_unis_logic_mipi_gen_mipi_s2m0_sl_in
#define AB_UNIS_BUS2_S2M_MG1_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_bus2_mt_out_to_input_system_unis_logic_mipi_gen_mipi_s2m1_sl_in
#define AB_UNIS_BUS2_PMA_IF_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_bus2_mt_out_to_input_system_unis_logic_pma_if_ctrl_in
         //added for MG   --offset from AB to MG devices --
#define AB_UNIS_BUS2_M_PKT_G0_ADDR    _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_bus2_mt_out_to_input_system_unis_logic_mipi_gen_mipi_pkt_gen0_slv_in
#define AB_UNIS_BUS2_M_PKT_G1_ADDR    _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_bus2_mt_out_to_input_system_unis_logic_mipi_gen_mipi_pkt_gen1_slv_in

  // ext 0
#define AB_DMA_EXT0_DMA_EXT0_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext0_mt_out_to_input_system_unis_logic_dma_ext0_dma_s0

  // ext 1
#define AB_DMA_EXT1_DMA_EXT1_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_dma_ext1_mt_out_to_input_system_unis_logic_dma_ext1_dma_s0

  // sp
#define AB_SP_EVQ_ADDR                _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_sp_mt_out_to_input_system_unis_logic_sp_control_tile_evq_s_ip

  // mipi buf
#define AB_MIPI_BUF_MIPI_BUF_ADDR     _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_mipibuf_mt_out_to_input_system_unis_logic_mipi_buffer_ip0

  // pixel_buf
#define AB_PIX_BUF_PIX_BUF_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_pixelbuf_mt_out_to_input_system_unis_logic_pixel_buffer_ip0

///////////////////////////
/////// COMBINED ADDRESSES
///////////
//// host:

#define HOST_DMEM_SPC_ADDR			(_hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_sp_sl_in + \
									 _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_sp_mt_out_to_input_system_unis_logic_sp_control_tile_sp_sl_dmem_ip)

  //csi2
#define HOST_CSI2_RX_A_ADDR			(HOST_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_A_ADDR)
#define SP_CSI2_RX_A_ADDR			(SP_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_A_ADDR)
#define HOST_CSI2_RX_B_ADDR			(HOST_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_B_ADDR)
#define SP_CSI2_RX_B_ADDR			(SP_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_B_ADDR)
#define HOST_CSI2_RX_C_ADDR			(HOST_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_C_ADDR)
#define SP_CSI2_RX_C_ADDR			(SP_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_C_ADDR)
#define HOST_CSI2_RX_D_ADDR			(HOST_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_D_ADDR)
#define SP_CSI2_RX_D_ADDR			(SP_AB_CSI2_3_ADDR + AB_CSI2_DPHY_RX_D_ADDR)
#define HOST_IBC2600_CSI2_ADDR      (HOST_AB_CSI2_3_ADDR + AB_CSI2_3_IBC2600_CSI2_ADDR)
#define SP_IBC2600_CSI2_ADDR      	(SP_AB_CSI2_3_ADDR + AB_CSI2_3_IBC2600_CSI2_ADDR)
#define HOST_S2M_CSI2_A_ADDR        (HOST_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_A_ADDR)
#define SP_S2M_CSI2_A_ADDR        	(SP_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_A_ADDR)
#define HOST_S2M_CSI2_B_ADDR        (HOST_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_B_ADDR)
#define SP_S2M_CSI2_B_ADDR        	(SP_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_B_ADDR)
#define HOST_S2M_CSI2_C_ADDR        (HOST_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_C_ADDR)
#define SP_S2M_CSI2_C_ADDR        	(SP_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_C_ADDR)
#define HOST_S2M_CSI2_D_ADDR        (HOST_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_D_ADDR)
#define SP_S2M_CSI2_D_ADDR        	(SP_AB_CSI2_3_ADDR + AB_CSI2_3_S2M_CSI2_D_ADDR)

  //unis
#define HOST_IBC2600_MG_ADDR        (HOST_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_IBC2600_MG_ADDR)
#define SP_IBC2600_MG_ADDR        	(SP_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_IBC2600_MG_ADDR)
#define HOST_S2M_MG0_ADDR           (HOST_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_S2M_MG0_ADDR)
#define SP_S2M_MG0_ADDR           	(SP_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_S2M_MG0_ADDR)
#define HOST_S2M_MG1_ADDR           (HOST_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_S2M_MG1_ADDR)
#define SP_S2M_MG1_ADDR           	(SP_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_S2M_MG1_ADDR)
 //added for MG   --MG base addresses from host --
#define HOST_M_PKT_G0_ADDR  		(HOST_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_M_PKT_G0_ADDR)
#define HOST_M_PKT_G1_ADDR  		(HOST_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_M_PKT_G1_ADDR)

#define HOST_DMA_EXT0_ADDR          (HOST_AB_DMA_EXT0_ADDR + AB_DMA_EXT0_DMA_EXT0_ADDR)
#define HOST_DMA_EXT1_ADDR          (HOST_AB_DMA_EXT1_ADDR + AB_DMA_EXT1_DMA_EXT1_ADDR)
#define HOST_MIPI_BUF_ADDR          (HOST_AB_MIPI_BUF_ADDR + AB_MIPI_BUF_MIPI_BUF_ADDR)
#define HOST_PIX_BUF_ADDR           (HOST_AB_PIX_BUF_ADDR  + AB_PIX_BUF_PIX_BUF_ADDR)
#define HOST_SP_QUE_ADDR            (HOST_AB_SP_ADDR + AB_SP_EVQ_ADDR)
#define HOST_PMA_IF_ADDR            (HOST_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_PMA_IF_ADDR)


  //islice
#define HOST_IBC2600_ISL_ADDR       (HOST_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)
#define SP_IBC2600_ISL_ADDR       	(SP_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)
#define HOST_S2M_ISL0_ADDR          (HOST_AB_IS_A_ADDR + AB_IS_A_S2M_ISL0_ADDR)
#define SP_S2M_ISL0_ADDR          	(SP_AB_IS_A_ADDR + AB_IS_A_S2M_ISL0_ADDR)
#define HOST_S2M_ISL1_ADDR          (HOST_AB_IS_A_ADDR + AB_IS_A_S2M_ISL1_ADDR)
#define SP_S2M_ISL1_ADDR          	(SP_AB_IS_A_ADDR + AB_IS_A_S2M_ISL1_ADDR)
#define HOST_DMA_ISL_ADDR           (HOST_AB_IS_A_ADDR + AB_IS_A_DMA_ISL_ADDR)
#define HOST_CIO2STR_ADDR           (HOST_AB_IS_A_ADDR + AB_IS_A_CIO2STR_ADDR)
#define SP_CIO2STR_ADDR           	(SP_AB_IS_A_ADDR + AB_IS_A_CIO2STR_ADDR)
#define HOST_PIX_FORM_ADDR          (HOST_AB_IS_A_ADDR + AB_IS_A_PIX_FORM_ADDR)
#define SP_PIX_FORM_ADDR          	(SP_AB_IS_A_ADDR + AB_IS_A_PIX_FORM_ADDR)
#define HOST_MIPI_BE0_ADDR          (HOST_AB_IS_A_ADDR + AB_IS_A_MIPI_BE0_ADDR)
#define SP_MIPI_BE0_ADDR          	(SP_AB_IS_A_ADDR + AB_IS_A_MIPI_BE0_ADDR)
#define HOST_MIPI_BE1_ADDR          (HOST_AB_IS_A_ADDR + AB_IS_A_MIPI_BE1_ADDR)
#define SP_MIPI_BE1_ADDR          	(SP_AB_IS_A_ADDR + AB_IS_A_MIPI_BE1_ADDR)
#define HOST_IS_GPREG_ADDR          (HOST_AB_IS_A_ADDR + AB_IS_A_GPREG_ADDR)
  //islice isa cluster componentes
//Input Corr GA componnents
#define HOST_IS_A_ISA_INPUT_CORR_ACB_ADDR	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_INPUT_CORR_ACB_ADDR)
#define HOST_IS_A_ISA_INPUT_CORR_INL_ADDR   	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_INPUT_CORR_INL_ADDR)
#define HOST_IS_A_ISA_INPUT_CORR_GBL_ADDR 	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_INPUT_CORR_GBL_ADDR)
#define HOST_IS_A_ISA_INPUT_CORR_PCLN_ADDR   	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_INPUT_CORR_PCLN_ADDR)
#define HOST_IS_A_ISA_INPUT_CORR_ACK_CONV_ADDR	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_INPUT_CORR_ACK_CONV_ADDR)
//AWB statistics  GA componnents
#define HOST_IS_A_ISA_AWB_ACB_ADDR 		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AWB_ACB_ADDR)
#define HOST_IS_A_ISA_AWB_AWRG_ADDR           (HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AWB_AWRG_ADDR)
#define HOST_IS_A_ISA_AWB_ACK_CONV_ADDR		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AWB_ACK_CONV_ADDR)


//AF AWB FR GRD GA components
#define HOST_IS_A_ISA_AF_AWB_FR_GRD_ACB_ADDR	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AF_AWB_FR_GRD_ACB_ADDR)
#define HOST_IS_A_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR 	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AF_AWB_FR_GRD_AF_AWB_FR_GRD_ADDR)
#define HOST_IS_A_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AF_AWB_FR_GRD_ACK_CONV_ADDR)
//AE Statistics GA componnents
#define HOST_IS_A_ISA_AE_STATS_ACB_ADDR  	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AE_STATS_ACB_ADDR)
#define HOST_IS_A_ISA_AE_STATS_WGHT_HIST_ADDR 	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AE_STATS_WGHT_HIST_ADDR)
#define HOST_IS_A_ISA_AE_STATS_CCM_ADDR  	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AE_STATS_CCM_ADDR)
#define HOST_IS_A_ISA_AE_STATS_ACK_CONV_ADDR	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_AE_STATS_ACK_CONV_ADDR)
//GDDPC GA componnents
#define HOST_IS_A_ISA_GDDPC_ACB_ADDR		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_GDDPC_ACB_ADDR)
#define HOST_IS_A_ISA_GDDPC_GDDPC_ADDR 		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_GDDPC_GDDPC_ADDR)
#define HOST_IS_A_ISA_GDDPC_ACK_CONV_ADDR 	(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_GDDPC_ACK_CONV_ADDR)
//LSC GA componnents
#define HOST_IS_A_ISA_LSC_ACB_ADDR 		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_LSC_ACB_ADDR)
#define HOST_IS_A_ISA_LSC_LSC_ADDR 		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_LSC_LSC_ADDR)
#define HOST_IS_A_ISA_LSC_ACK_CONV_ADDR		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_LSC_ACK_CONV_ADDR)
//Bayer Scaler GA components
#define HOST_IS_A_ISA_SCALER_ACB_ADDR         (HOST_AB_IS_A_ADDR + AB_IS_A_ISA_SCALER_ACB_ADDR)
#define HOST_IS_A_ISA_SCALER_ACK_CONV_ADDR    (HOST_AB_IS_A_ADDR + AB_IS_A_ISA_SCALER_ACK_CONV_ADDR)
#define HOST_IS_A_ISA_SCALER_SCALER_ADDR      (HOST_AB_IS_A_ADDR + AB_IS_A_ISA_SCALER_SCALER_ADDR)
//ISA GP regs
#define HOST_IS_A_ISA_GP_REGS_ADDR   		(HOST_AB_IS_A_ADDR + AB_IS_A_ISA_GP_REGS_ADDR)

// SP:
  //csi2
#define SP_IBC2600_CSI2_ADDR        (SP_AB_CSI2_3_ADDR + AB_CSI2_3_IBC2600_CSI2_ADDR)

  //unis
#define SP_IBC2600_MG_ADDR          (SP_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_IBC2600_MG_ADDR)
#define SP_EVQ_ADDR                 _hrt_master_to_slave_address_input_system_unis_logic_sp_control_tile_dmem_bus_mt_dmem_to_input_system_unis_logic_sp_control_tile_evq_s_op
	//added for MG   --MG base addresses from SP --
#define SP_M_PKT_G0_ADDR			(SP_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_M_PKT_G0_ADDR)
#define SP_M_PKT_G1_ADDR			(SP_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_M_PKT_G1_ADDR)
  //islice
#define SP_IBC2600_ISL_ADDR         (SP_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)

// S2M:
  //csi2
#define S2M_CSI2_A_IBC2600_ADDR     _hrt_master_to_slave_address_input_system_csi2_logic_s2m_a_ack_mt_out_to_input_system_csi2_logic_ibuf_ctrl_sl
#define S2M_CSI2_B_IBC2600_ADDR     _hrt_master_to_slave_address_input_system_csi2_logic_s2m_b_ack_mt_out_to_input_system_csi2_logic_ibuf_ctrl_sl
#define S2M_CSI2_C_IBC2600_ADDR     _hrt_master_to_slave_address_input_system_csi2_logic_s2m_c_ack_mt_out_to_input_system_csi2_logic_ibuf_ctrl_sl
#define S2M_CSI2_D_IBC2600_ADDR     _hrt_master_to_slave_address_input_system_csi2_logic_s2m_d_ack_mt_out_to_input_system_csi2_logic_ibuf_ctrl_sl
#define S2M_CSI2_A_MIPI_BUF_ADDR    _hrt_master_to_slave_address_input_system_csi2_logic_s2m_a_mt_out_to_input_system_unis_logic_mipi_buffer_ip0
#define S2M_CSI2_B_MIPI_BUF_ADDR    _hrt_master_to_slave_address_input_system_csi2_logic_s2m_b_mt_out_to_input_system_unis_logic_mipi_buffer_ip0
#define S2M_CSI2_C_MIPI_BUF_ADDR    _hrt_master_to_slave_address_input_system_csi2_logic_s2m_c_mt_out_to_input_system_unis_logic_mipi_buffer_ip0
#define S2M_CSI2_D_MIPI_BUF_ADDR    _hrt_master_to_slave_address_input_system_csi2_logic_s2m_d_mt_out_to_input_system_unis_logic_mipi_buffer_ip0


  //mipi gen
#define S2M_MG0_IBC2600_ADDR        _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_ctrl_bus_bus1_a_mt_mipi_s2m0_to_input_system_unis_logic_mipi_gen_ibuf_ctrl_sl
#define S2M_MG1_IBC2600_ADDR        _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_ctrl_bus_bus1_a_mt_mipi_s2m1_to_input_system_unis_logic_mipi_gen_ibuf_ctrl_sl
#define S2M_MG0_MIPI_BUF_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_mipi_s2m0_mt_out_to_input_system_unis_logic_mipi_buffer_ip0
#define S2M_MG1_MIPI_BUF_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_mipi_s2m1_mt_out_to_input_system_unis_logic_mipi_buffer_ip0
#define S2M_MG0_PIX_BUF_ADDR        _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_mipi_s2m0_mt_out_to_input_system_unis_logic_pixel_buffer_ip0

  //islice
#define S2M_ISL0_IBC2600_ADDR       _hrt_master_to_slave_address_input_system_is_a_logic_pixel_s2m0_ack_mt_out_to_input_system_is_a_logic_ibuf_ctrl_sl
#define S2M_ISL1_IBC2600_ADDR       _hrt_master_to_slave_address_input_system_is_a_logic_pixel_s2m1_ack_mt_out_to_input_system_is_a_logic_ibuf_ctrl_sl
#define S2M_ISL0_PIX_BUF_ADDR       _hrt_master_to_slave_address_input_system_is_a_logic_pixel_s2m0_mt_out_to_input_system_unis_logic_pixel_buffer_ip0
#define S2M_ISL1_PIX_BUF_ADDR       _hrt_master_to_slave_address_input_system_is_a_logic_pixel_s2m1_mt_out_to_input_system_unis_logic_pixel_buffer_ip0


// DMA:
  //ext 0
#define DMA_EXT0_IBC2600_CSI2_ADDR  (EQC_DMA_EXT0_AB_CSI2_3_ADDR + AB_CSI2_3_IBC2600_CSI2_ADDR)
#define DMA_EXT0_IBC2600_MG_ADDR    (EQC_DMA_EXT0_AB_BUS2_ADDR + AB_UNIS_BUS2_IBC2600_MG_ADDR)
#define DMA_EXT0_IBC2600_ISL_ADDR   (EQC_DMA_EXT0_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)
#define DMA_EXT0_MIPI_BUF_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_dma_ext0_dma_m0_to_input_system_unis_logic_mipi_buffer_ip0
#define DMA_EXT0_PIX_BUF_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_dma_ext0_dma_m0_to_input_system_unis_logic_pixel_buffer_ip0

  //ext 1
#define DMA_EXT1_IBC2600_CSI2_ADDR  (EQC_DMA_EXT1_AB_CSI2_3_ADDR + AB_CSI2_3_IBC2600_CSI2_ADDR)
#define DMA_EXT1_IBC2600_MG_ADDR    (EQC_DMA_EXT1_AB_BUS2_ADDR + AB_UNIS_BUS2_IBC2600_MG_ADDR)
#define DMA_EXT1_IBC2600_ISL_ADDR   (EQC_DMA_EXT1_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)
//#define DMA_EXT1_MIPI_BUF_ADDR      _hrt_master_to_slave_address_input_system_unis_logic_dma_ext1_dma_m0_to_input_system_unis_logic_mipi_buffer_ip0
#define DMA_EXT1_MIPI_BUF_ADDR      0x10000
#define DMA_EXT1_PIX_BUF_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_dma_ext1_dma_m0_to_input_system_unis_logic_pixel_buffer_ip0

  // islice
#define DMA_ISL_IBC2600_ISL_ADDR    _hrt_master_to_slave_address_input_system_is_a_logic_dma_m2_to_input_system_is_a_logic_ibuf_ctrl_sl
#define DMA_ISL_MIPI_BUF_ADDR       _hrt_master_to_slave_address_input_system_is_a_logic_dma_m0_to_input_system_unis_logic_mipi_bus_mshield_mt_mipibuf_sp

// ISA
#define ISA_SP_QUE_ADDR                     (ISA_IS_A_EQC_ADDR + EQC_IS_A_AB_SP_ADDR + AB_SP_EVQ_ADDR)

// IBUF:
 // csi2
#define IBC2600_CSI2_SP_QUE_ADDR      (IBC2600_CSI2_EQC_ADDR + EQC_CSI2_AB_SP_ADDR + AB_SP_EVQ_ADDR)
#define IBC2600_CSI2_S2M_CSI2_A_ADDR  _hrt_master_to_slave_address_input_system_csi2_logic_ibuf_ctrl_mt_to_input_system_csi2_logic_s2m_a_sl_in
#define IBC2600_CSI2_S2M_CSI2_B_ADDR  _hrt_master_to_slave_address_input_system_csi2_logic_ibuf_ctrl_mt_to_input_system_csi2_logic_s2m_b_sl_in
#define IBC2600_CSI2_S2M_CSI2_C_ADDR  _hrt_master_to_slave_address_input_system_csi2_logic_ibuf_ctrl_mt_to_input_system_csi2_logic_s2m_c_sl_in
#define IBC2600_CSI2_S2M_CSI2_D_ADDR  _hrt_master_to_slave_address_input_system_csi2_logic_ibuf_ctrl_mt_to_input_system_csi2_logic_s2m_d_sl_in
#define IBC2600_CSI2_DMA_EXT0_ADDR    (IBC2600_CSI2_EQC_ADDR + EQC_CSI2_AB_DMA_EXT0_ADDR + AB_DMA_EXT0_DMA_EXT0_ADDR)
#define IBC2600_CSI2_DMA_EXT1_ADDR    (IBC2600_CSI2_EQC_ADDR + EQC_CSI2_AB_DMA_EXT1_ADDR + AB_DMA_EXT1_DMA_EXT1_ADDR)
#define IBC2600_CSI2_DMA_ISL_ADDR     (IBC2600_CSI2_EQC_ADDR + EQC_CSI2_AB_IS_A_ADDR + AB_IS_A_DMA_ISL_ADDR)
#define IBC2600_CSI2_IBC2600_ISL_ADDR (IBC2600_CSI2_EQC_ADDR + EQC_CSI2_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)
#define IBC2600_CSI2_PMA_IF_ADDR      (IBC2600_CSI2_EQC_ADDR + EQC_IS_A_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_PMA_IF_ADDR)

 // mipi gen
#define IBC2600_MG_SP_QUE_ADDR        (IBC2600_MG_EQC_ADDR + EQC_MG_AB_SP_ADDR + AB_SP_EVQ_ADDR)
#define IBC2600_MG_S2M_MG0_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_ibuf_ctrl_mt_to_input_system_unis_logic_mipi_gen_mipi_s2m0_sl_in
#define IBC2600_MG_S2M_MG1_ADDR       _hrt_master_to_slave_address_input_system_unis_logic_mipi_gen_ibuf_ctrl_mt_to_input_system_unis_logic_mipi_gen_mipi_s2m1_sl_in
#define IBC2600_MG_DMA_EXT0_ADDR      (IBC2600_MG_EQC_ADDR + EQC_MG_AB_DMA_EXT0_ADDR + AB_DMA_EXT0_DMA_EXT0_ADDR)
#define IBC2600_MG_DMA_EXT1_ADDR      (IBC2600_MG_EQC_ADDR + EQC_MG_AB_DMA_EXT1_ADDR + AB_DMA_EXT1_DMA_EXT1_ADDR)
#define IBC2600_MG_DMA_ISL_ADDR       (IBC2600_MG_EQC_ADDR + EQC_MG_AB_IS_A_ADDR + AB_IS_A_DMA_ISL_ADDR)
#define IBC2600_MG_IBC2600_ISL_ADDR   (IBC2600_MG_EQC_ADDR + EQC_MG_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)
#define IBC2600_MG_PMA_IF_ADDR        (IBC2600_MG_EQC_ADDR + EQC_MG_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_PMA_IF_ADDR)
#define IBC2600_MG_PIX_BUF_ADDR       (IBC2600_MG_EQC_ADDR + EQC_MG_AB_PIX_BUF_ADDR   + AB_PIX_BUF_PIX_BUF_ADDR)

 // islice
#define IBC2600_ISL_SP_QUE_ADDR       (IBC2600_IS_A_EQC_ADDR + EQC_IS_A_AB_SP_ADDR + AB_SP_EVQ_ADDR)
#define IBC2600_ISL_S2M_ISL0_ADDR     _hrt_master_to_slave_address_input_system_is_a_logic_ibuf_ctrl_mt_to_input_system_is_a_logic_pixel_s2m0_sl_in
#define IBC2600_ISL_S2M_ISL1_ADDR     _hrt_master_to_slave_address_input_system_is_a_logic_ibuf_ctrl_mt_to_input_system_is_a_logic_pixel_s2m1_sl_in
#define IBC2600_ISL_DMA_EXT0_ADDR     (IBC2600_IS_A_EQC_ADDR + EQC_IS_A_AB_DMA_EXT0_ADDR + AB_DMA_EXT0_DMA_EXT0_ADDR)
#define IBC2600_ISL_DMA_EXT1_ADDR     (IBC2600_IS_A_EQC_ADDR + EQC_IS_A_AB_DMA_EXT1_ADDR + AB_DMA_EXT1_DMA_EXT1_ADDR)
#define IBC2600_ISL_DMA_ISL_ADDR      _hrt_master_to_slave_address_input_system_is_a_logic_ibuf_ctrl_mt_to_input_system_is_a_logic_dma_s0
#define IBC2600_ISL_IBC2600_MG_ADDR   (IBC2600_IS_A_EQC_ADDR + EQC_IS_A_AB_BUS2_ADDR + AB_UNIS_BUS2_IBC2600_MG_ADDR)
#define IBC2600_ISL_IBC2600_CSI2_ADDR (IBC2600_IS_A_EQC_ADDR + EQC_IS_A_AB_CSI2_3_ADDR + AB_CSI2_3_IBC2600_CSI2_ADDR)
#define IBC2600_ISL_PMA_IF_ADDR       (IBC2600_IS_A_EQC_ADDR + EQC_IS_A_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_PMA_IF_ADDR)



//// PMA IF
#define PMA_IF_IBC2600_MG_ADDR        (PMA_IF_AB_UNIS_BUS2_ADDR + AB_UNIS_BUS2_IBC2600_MG_ADDR)
#define PMA_IF_IBC2600_CSI2_ADDR      (PMA_IF_AB_CSI2_3_ADDR + AB_CSI2_3_IBC2600_CSI2_ADDR)
#define PMA_IF_IBC2600_ISL_ADDR       (PMA_IF_AB_IS_A_ADDR + AB_IS_A_IBC2600_ISL_ADDR)
#define PMA_IF_WAKE_DRAIN_DONE_ADDR   _hrt_master_to_slave_address_host_op0_to_testbench_tb_regs_slv_in
//////////////////////////

#endif
