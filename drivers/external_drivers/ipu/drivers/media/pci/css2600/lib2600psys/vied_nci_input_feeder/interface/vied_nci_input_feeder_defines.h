#ifndef _IPF_DEFINES_H_
#define _IPF_DEFINES_H_

#include <psys.h>
#include <master_to_slave_hrt.h>

// IPF Defines

#define IPF_CFG_BUS_DATA_WIDTH 32
#define IPF_CFG_BUS_ADDR_WIDTH 32
#define IPF_VCT_BUS_DATA_WIDTH 512


/*#define IPF_DEV_ID    0
#define IPF_MAX_SIDS  5*/
#define IPF_MAX_EQC   8
// // Color Components IDs
// #define BAYER0_ID     0
// #define BAYER1_ID     1
// #define YUV420_ID     2
// #define RGB0_ID       3
// #define RGB1_ID       4


#define IBUFF_ACK_DESTINATION    _hrt_master_to_slave_address_input_feeder_inst_ibufctrl_2600_inst_mt_to_input_feeder_inst_eqc_ipfd_sp0
// #define EQC_CFG_ADDR             HOST_2_IPFD_EQC_ADDR
#define MEM_CTRL_ACK_ADDR        _hrt_master_to_slave_address_input_feeder_inst_eqc_ipfd_mp0_to_mem_ctrl_ip0

// Addresses
// #define IPF_IBUFF_CTRL_MT_TO_DMA_SLV           _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_mt_to_processing_system_unps_logic_ipfd_dma4_inst_s0
// #define IPF_IBUFF_CTRL_MT_TO_EQC_SLV           _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_mt_to_processing_system_unps_logic_ipfd_eqc_ipfd_sp0
// #define SID0_IPF_IBUFF_CTRL_MT_TO_EVENT_QUEUE  _hrt_master_to_slave_address_input_feeder_inst_ibufctrl_2600_inst_mt_to_mem_ctrl_ip0 + 0x0
// #define SID1_IPF_IBUFF_CTRL_MT_TO_EVENT_QUEUE  _hrt_master_to_slave_address_input_feeder_inst_ibufctrl_2600_inst_mt_to_mem_ctrl_ip0 + 0x4
// #define SID2_IPF_IBUFF_CTRL_MT_TO_EVENT_QUEUE  _hrt_master_to_slave_address_input_feeder_inst_ibufctrl_2600_inst_mt_to_mem_ctrl_ip0 + 0x8
// #define SID3_IPF_IBUFF_CTRL_MT_TO_EVENT_QUEUE  _hrt_master_to_slave_address_input_feeder_inst_ibufctrl_2600_inst_mt_to_mem_ctrl_ip0 + 0xC
// #define SID4_IPF_IBUFF_CTRL_MT_TO_EVENT_QUEUE  _hrt_master_to_slave_address_input_feeder_inst_ibufctrl_2600_inst_mt_to_mem_ctrl_ip0 + 0xF
// #define IPF_IBUFF_CTRL_MT_TO_DMFS_SLV          _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_mt_to_processing_system_unps_logic_ipfd_dmf_s_inst_sl_cmd
#define IPF_IBUFF_CTRL_SLV                     HOST_2_IPFD_IBUFF_ADDR
#define IPF_DMFS_SLV_PORT                      HOST_2_IPFD_DMFSYNC_ADDR
#define IPF_DMFS_MT_TO_IBUFF_SLV               _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_dmf_s_inst_mt_ack_to_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_sl
#define IPF_DMF_SLV_PORT                       HOST_2_IPFD_DMF_ADDR
#define IPF_DMA4_SLV_PORT                      HOST_2_IPFD_DMA4_ADDR



#define NR_BAYER1_VECTORS 4
#define NR_BAYER2_VECTORS 4
#define NR_YUV420_VECTORS 6
#define NR_RGB_VECTORS    12

// DMF Settings
// #define SID0_IPF_DMF_BUFFER_SIZE 8
// #define SID1_IPF_DMF_BUFFER_SIZE 8
// #define SID2_IPF_DMF_BUFFER_SIZE 12
// #define SID3_IPF_DMF_BUFFER_SIZE 18
// #define SID4_IPF_DMF_BUFFER_SIZE 18
// #define SID0_DMF_BURST_TIMER     64 - 1
// #define SID1_DMF_BURST_TIMER     64 - 1
// #define SID2_DMF_BURST_TIMER     32 - 1
// #define SID3_DMF_BURST_TIMER     32 - 1
// #define SID4_DMF_BURST_TIMER     32 - 1

#define IPF_DMF_MAX_FIFO_SIZE 64

// DMA settings
#define IPF_DMA_MAX_SPAN_HEIGHT   4096
#define IPF_DMA_MAX_SPAN_WIDTH    512
#define IPF_DMA_MAX_REGION_STRIDE 65536

// IBuffer CTRL settings
#define CONFIG_DMA_BY_IBUFF_CTRL                1
#define IPF_IBUFF_MAX_NUM_UNITS_PER_FETCH       16383
#define IPF_IBUFF_MAX_UNITS_PER_BUFFER          16383

#define IPF_ID                     input_feeder_inst
#define IPF_DMF_ID                 input_feeder_inst_dmf_inst
#define IPF_DMF_SYNC_ID            input_feeder_inst_dmf_s_inst
#define IPF_DMA_ID                 input_feeder_inst_dma4_inst
#define IPF_IBUF_CNTRL_ID          input_feeder_inst_ibufctrl_2600_inst

// DMA Settings

#endif /* _IPF_DEFINES_H_ */


