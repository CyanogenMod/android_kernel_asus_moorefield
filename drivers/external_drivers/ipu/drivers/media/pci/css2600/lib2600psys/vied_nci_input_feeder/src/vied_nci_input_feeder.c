#include "type_support.h"
#include "misc_support.h"
#include "vied_nci_input_feeder.h"

#include "vied_nci_input_feeder_dmf.h"
#include "vied_nci_input_feeder_dmfs.h"
#include "vied_nci_input_feeder_dma.h"
#include "vied_nci_input_feeder_ibuff_ctrl.h"
#include "vied_nci_eqc.h"

#define IPF_IBUFF_CTRL_MT_TO_DMA_SLV           _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_mt_to_processing_system_unps_logic_ipfd_dma4_inst_s0
#define IPF_IBUFF_CTRL_MT_TO_EQC_SLV           _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_mt_to_processing_system_unps_logic_ipfd_eqc_ipfd_sp0
#define IPF_IBUFF_CTRL_MT_TO_DMFS_SLV          _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_mt_to_processing_system_unps_logic_ipfd_dmf_s_inst_sl_cmd
#define IPF_IBUFF_CTRL_SLV                     HOST_2_IPFD_IBUFF_ADDR
#define IPF_DMFS_SLV_PORT                      HOST_2_IPFD_DMFSYNC_ADDR
#define IPF_DMFS_MT_TO_IBUFF_SLV               _hrt_master_to_slave_address_processing_system_unps_logic_ipfd_dmf_s_inst_mt_ack_to_processing_system_unps_logic_ipfd_ibufctrl_2600_inst_sl
#define IPF_DMF_SLV_PORT                       HOST_2_IPFD_DMF_ADDR

#define NWAYS 32

typedef struct {
  unsigned int   fifo_size           [VIED_NCI_INFEEDER_MAX_SIDS];
  unsigned int   span_width_source   [VIED_NCI_INFEEDER_MAX_SIDS];
  unsigned int   span_height_source  [VIED_NCI_INFEEDER_MAX_SIDS];
  unsigned int   num_units_per_fetch [VIED_NCI_INFEEDER_MAX_SIDS];
  unsigned int   units_per_buffer    [VIED_NCI_INFEEDER_MAX_SIDS];
  unsigned int   start_address       [VIED_NCI_INFEEDER_MAX_SIDS];
  unsigned int   region_stide        [VIED_NCI_INFEEDER_MAX_SIDS];
  unsigned int   ack_address         [VIED_NCI_INFEEDER_MAX_SIDS];
  t_sid_dmfs     p_sid_dmfs          [VIED_NCI_INFEEDER_MAX_SIDS];
  t_s2m_dmfs     p_s2m_dmfs          [VIED_NCI_INFEEDER_MAX_SIDS];
  t_sid_dmf      p_sid_dmf           [VIED_NCI_INFEEDER_MAX_SIDS];
  t_sid_dma      p_sid_dma           [VIED_NCI_INFEEDER_MAX_SIDS];
  ibc2600_ibuf_s p_sids_ibuff;
} t_sid_ipf;

unsigned int ibuff_proc_items_p_unit[VIED_NCI_INFEEDER_MAX_SIDS]         = {NR_BAYER1_VECTORS, NR_BAYER2_VECTORS, NR_YUV420_VECTORS, NR_RGB_VECTORS, NR_RGB_VECTORS};
unsigned int dmf_sid_burst_timer[VIED_NCI_INFEEDER_MAX_SIDS]             = {SID0_DMF_BURST_TIMER, SID1_DMF_BURST_TIMER, SID2_DMF_BURST_TIMER, SID3_DMF_BURST_TIMER, SID4_DMF_BURST_TIMER};

static uint8_t dests_p_proc[VIED_NCI_INFEEDER_MAX_SIDS];
static uint8_t has_2nd_buff[VIED_NCI_INFEEDER_MAX_SIDS];

static t_sid_ipf p_sid_ipf;

// static t_sid_ipf p_sid_ipf;
static bool static_config_done;
static bool infeeder_lock_stream_configuration[VIED_NCI_INFEEDER_MAX_SIDS];

////////////////////////////////////////////////////
// Private helper functions
////////////////////////////////////////////////////

// Set start address of a stream. Results in an INIT event
static vied_nci_infeeder_msg_t vied_nci_infeeder_stream_set_start_address(vied_nci_infeeder_stream_handle_t str_handle, vied_nci_infeeder_address_t start_address) {
  p_sid_ipf.p_sids_ibuff.dest_cfg[str_handle].st_addr = start_address;
  ibc2600_set_dest_cfg_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_DEST_CFG_ST_ADDR, p_sid_ipf.p_sids_ibuff.dest_cfg[str_handle].st_addr);
  ibc2600_set_sid_cmd_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_PROC_CMD_CMD, _IBC_2600_CMD_INIT_VALUE);

  return vied_nci_infeeder_msg_successful;
}

// Set ACK address of a stream
vied_nci_infeeder_msg_t
vied_nci_infeeder_stream_set_ack_address(vied_nci_infeeder_stream_handle_t
					 str_handle,
					 vied_nci_infeeder_address_t
					 ack_address, unsigned int sidpid) {
  unsigned int  mt_cfg_type = 2; // LOG2(nr bytes of cfg data)
  p_sid_ipf.p_sids_ibuff.proc[str_handle].cmd.ack_addr = ((IPF_IBUFF_CTRL_MT_TO_EQC_SLV + ack_address) >> mt_cfg_type);

  ibc2600_set_sid_cmd_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_PROC_CMD_ACK_ADDR, p_sid_ipf.p_sids_ibuff.proc[str_handle].cmd.ack_addr);
  ibc2600_set_sid_cmd_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_PROC_CMD_EVENTQUE_SIDPID, sidpid);

  return vied_nci_infeeder_msg_successful;
}

// Set FIFO size of a stream
static vied_nci_infeeder_msg_t vied_nci_infeeder_stream_set_fifo_size(vied_nci_infeeder_stream_handle_t str_handle, unsigned int fifo_size) {
  // Checks
  if((fifo_size < 1) || (fifo_size > IPF_DMF_MAX_FIFO_SIZE)) {
    return vied_nci_infeeder_msg_fifo_size_exceeded;
  }

  p_sid_ipf.fifo_size[str_handle]                      = fifo_size;
  p_sid_ipf.p_sid_dmf[str_handle].sid_buffer_size      = fifo_size;
  p_sid_ipf.p_sid_dmfs[str_handle].sid_set_buffer_size = fifo_size;

  dmf_set_buffer_size_sid(p_sid_ipf.p_sid_dmf[str_handle]);
  dmfs_set_buffer_size_sid(p_sid_ipf.p_sid_dmfs[str_handle]);

  return vied_nci_infeeder_msg_successful;
}

// Set num_units_per_fetch of a stream
static vied_nci_infeeder_msg_t vied_nci_infeeder_stream_set_num_units_per_fetch(vied_nci_infeeder_stream_handle_t str_handle, unsigned int num_units_per_fetch) {
  // Checks
  if(num_units_per_fetch > IPF_IBUFF_MAX_NUM_UNITS_PER_FETCH) return vied_nci_infeeder_msg_num_units_per_fetch_exceeded;

  p_sid_ipf.p_sids_ibuff.dest_cfg[str_handle].num_items = num_units_per_fetch;

  ibc2600_set_dest_cfg_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_DEST_CFG_DEST_NUM_UNITS, num_units_per_fetch);

  dmfs_set_reg(p_sid_ipf.p_s2m_dmfs[str_handle].base_addr, S2M_NUM_ITEMS_REG, num_units_per_fetch);

//   printf("Set dest cfg num_units_per_fetch: 0x%x\n", p_sid_ipf.p_sids_ibuff.dest_cfg[sid].num_items);

  return vied_nci_infeeder_msg_successful;
}

//Set units_per_buffer of a stream
static vied_nci_infeeder_msg_t vied_nci_infeeder_stream_set_units_per_buffer(vied_nci_infeeder_stream_handle_t str_handle, unsigned int units_per_buffer) {
  // Checks
  if(units_per_buffer > IPF_IBUFF_MAX_UNITS_PER_BUFFER)  return vied_nci_infeeder_msg_units_per_buffer_exceeded;

  p_sid_ipf.p_sids_ibuff.proc[str_handle].cfg.lines_p_frame = units_per_buffer;

  ibc2600_set_sid_cfg_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_PROC_CFG_LINES_P_FRAME, p_sid_ipf.p_sids_ibuff.proc[str_handle].cfg.lines_p_frame);
  ibc2600_set_sid_cfg_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_PROC_CFG_UNITS_P_IBUF , p_sid_ipf.p_sids_ibuff.proc[str_handle].cfg.lines_p_frame);

  return vied_nci_infeeder_msg_successful;
}

// Set span_width_source of a stream
static vied_nci_infeeder_msg_t vied_nci_infeeder_stream_set_span_width_source(vied_nci_infeeder_stream_handle_t str_handle, unsigned int span_width) {
  // Checks
  if(span_width > IPF_DMA_MAX_SPAN_WIDTH)   return vied_nci_infeeder_msg_span_width_exceeded;

  int bank_addr = (SPAN_GROUP_ID << group_id_lsbidx()) + (str_handle << bank_id_lsbidx());

  // Adjust Span Width
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES, span_width - 1);

  // Adjust Region Width
  bank_addr = (TERMINAL_GROUP_ID << group_id_lsbidx()) + (str_handle << bank_id_lsbidx());
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES, (span_width * ibuff_proc_items_p_unit[str_handle] * NWAYS) - 1);

  return vied_nci_infeeder_msg_successful;
}

// Set span_height_source of a stream
static vied_nci_infeeder_msg_t vied_nci_infeeder_stream_set_span_height_source(vied_nci_infeeder_stream_handle_t str_handle, unsigned int span_height) {
  // Checks
  if(span_height > IPF_DMA_MAX_SPAN_HEIGHT) return vied_nci_infeeder_msg_span_height_exceeded;

  // Adjust Span Height
  int bank_addr = (SPAN_GROUP_ID << group_id_lsbidx()) + (str_handle << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES, span_height - 1);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES, span_height - 1);

  return vied_nci_infeeder_msg_successful;
}

// Set region_stride of a stream
static vied_nci_infeeder_msg_t vied_nci_infeeder_stream_set_region_stride(vied_nci_infeeder_stream_handle_t str_handle, unsigned int region_stride) {
  // Checks
  if(region_stride > IPF_DMA_MAX_REGION_STRIDE)               return vied_nci_infeeder_msg_region_stride_exceeded;


  int bank_addr = (TERMINAL_GROUP_ID << group_id_lsbidx()) + (str_handle << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, region_stride );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, region_stride );

  return vied_nci_infeeder_msg_successful;
}

////////////////////////////////////////////////////
// Static configuration layer
////////////////////////////////////////////////////
static vied_nci_infeeder_msg_t vied_nci_infeeder_static_config(vied_nci_infeeder_handle_t handle) {

  unsigned int i;

  unsigned int dmfs_sid_base_addr[VIED_NCI_INFEEDER_MAX_SIDS]              = {0x00, 0x18, 0x30, 0x48, 0x60};
  unsigned int dmfs_sid_ack_response_addr[VIED_NCI_INFEEDER_MAX_SIDS]      = {0x090, 0x290, 0x490, 0x690, 0x890};
  unsigned int dmfs_s2m_base_addr[VIED_NCI_INFEEDER_MAX_SIDS]              = {0x78, 0x94, 0xB0, 0xCC, 0xE8};
  unsigned int dmfs_s2m_ack_response_addr[VIED_NCI_INFEEDER_MAX_SIDS]      = {0x094, 0x294, 0x494, 0x694, 0x894};

  unsigned int dmf_base_addr[VIED_NCI_INFEEDER_MAX_SIDS]                   = {0x00, 0x18, 0x30, 0x48, 0x60};
//  unsigned int dmf_sid_burst_timer[VIED_NCI_INFEEDER_MAX_SIDS]             = {SID0_DMF_BURST_TIMER, SID1_DMF_BURST_TIMER, SID2_DMF_BURST_TIMER, SID3_DMF_BURST_TIMER, SID4_DMF_BURST_TIMER};
  unsigned int dma_channel_descriptor_ack_addr[VIED_NCI_INFEEDER_MAX_SIDS] = {0x0A0, 0x2A0, 0x4A0, 0x6A0, 0x8A0};
  unsigned int dma_terminal_descriptor_region_origin[VIED_NCI_INFEEDER_MAX_SIDS] = {0x000, 0x080, 0x100, 0x180, 0x200};
  unsigned int ibuff_proc_snd_buf_cmd_addr[VIED_NCI_INFEEDER_MAX_SIDS]     = {0x00, 0x18, 0x30, 0x48, 0x60};
  unsigned int ibuff_proc_str2mmio_addr[VIED_NCI_INFEEDER_MAX_SIDS]        = {0x78, 0x94, 0xB0, 0xCC, 0xE8};
//  unsigned int ibuff_proc_items_p_unit[VIED_NCI_INFEEDER_MAX_SIDS]         = {NR_BAYER1_VECTORS, NR_BAYER2_VECTORS, NR_YUV420_VECTORS, NR_RGB_VECTORS, NR_RGB_VECTORS};
  NOT_USED(handle);
  static_config_done = false;

  for(i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    p_sid_ipf.fifo_size          [i] = 1;
    p_sid_ipf.span_width_source  [i] = 1;
    p_sid_ipf.span_height_source [i] = 1;
    p_sid_ipf.num_units_per_fetch[i] = 1;
    p_sid_ipf.units_per_buffer   [i] = 0;
    p_sid_ipf.start_address      [i] = 0x0;
    p_sid_ipf.region_stide       [i] = IPF_VCT_BUS_DATA_WIDTH/8;
    p_sid_ipf.ack_address        [i] = 0;
  }

  //
  // DMF Sync configurations
  //
  for(i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    p_sid_ipf.p_sid_dmfs[i].base_addr             = IPF_DMFS_SLV_PORT + dmfs_sid_base_addr[i];
    p_sid_ipf.p_sid_dmfs[i].sid_cmd_request       = 0;
    p_sid_ipf.p_sid_dmfs[i].sid_ack_response_addr = IPF_DMFS_MT_TO_IBUFF_SLV + dmfs_sid_ack_response_addr[i];
    p_sid_ipf.p_sid_dmfs[i].sid_ack_response      = 1;
    p_sid_ipf.p_sid_dmfs[i].sid_set_buffer_size   = p_sid_ipf.fifo_size[i];
    p_sid_ipf.p_sid_dmfs[i].sid_compute_unit_size = ibuff_proc_items_p_unit[i];
    p_sid_ipf.p_s2m_dmfs[i].base_addr             = IPF_DMFS_SLV_PORT + dmfs_s2m_base_addr[i];
    p_sid_ipf.p_s2m_dmfs[i].s2m_cmd_request       = 0;
    p_sid_ipf.p_s2m_dmfs[i].s2m_num_items         = ibuff_proc_items_p_unit[i];
    p_sid_ipf.p_s2m_dmfs[i].s2m_ack_response_addr = IPF_DMFS_MT_TO_IBUFF_SLV + dmfs_s2m_ack_response_addr[i];
  }

  //
  // DMF Configurations
  //
  unsigned int total_buffer_size = 0;

  for(i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    p_sid_ipf.p_sid_dmf[i].base_addr       = IPF_DMF_SLV_PORT + dmf_base_addr[i];
    p_sid_ipf.p_sid_dmf[i].sid_burst_timer = dmf_sid_burst_timer[i];
    p_sid_ipf.p_sid_dmf[i].sid_buffer_size = p_sid_ipf.fifo_size[i]; //dmf_sid_buffer_size[i];
    total_buffer_size += p_sid_ipf.fifo_size[i];
  }
  if(total_buffer_size > DMF_REG_FILE_SIZE) return -1;

  for(i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    p_sid_ipf.p_sid_dma[i].base_addr                                = 0x0;  // Not used at the moment
    p_sid_ipf.p_sid_dma[i].terminal_A_id                            =                    i;
    p_sid_ipf.p_sid_dma[i].terminal_B_id                            = TERMINAL_BANKS/2 + i;
    p_sid_ipf.p_sid_dma[i].channel_id                               =                    i;
    p_sid_ipf.p_sid_dma[i].unit_id                                  =                    i;
    p_sid_ipf.p_sid_dma[i].span_A_id                                =                    i;
    p_sid_ipf.p_sid_dma[i].span_B_id                                =     SPAN_BANKS/2 + i;
    p_sid_ipf.p_sid_dma[i].request_id                               =                    i;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[0].region_origin     = p_sid_ipf.start_address[i];
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[0].region_width      = (p_sid_ipf.span_width_source[i] * ibuff_proc_items_p_unit[i] * NWAYS) - 1;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[0].region_stride     = p_sid_ipf.region_stide[i];
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[0].element_setup     = ELEMENT_PRECISION_16;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[0].cio_info_setup    = 0;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[0].port_mode         = MMIO_MODE;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[1].region_origin     = dma_terminal_descriptor_region_origin[i];   // Address SID
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[1].region_width      = (ibuff_proc_items_p_unit[i] * NWAYS) - 1;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[1].region_stride     = IPF_VCT_BUS_DATA_WIDTH/8;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[1].element_setup     = ELEMENT_PRECISION_16;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[1].cio_info_setup    = 0;
    p_sid_ipf.p_sid_dma[i].terminal_descriptor[1].port_mode         = FIFO_MODE;
    p_sid_ipf.p_sid_dma[i].channel_descriptor.element_extend_mode   = extend_mode_zero;
    p_sid_ipf.p_sid_dma[i].channel_descriptor.element_init_data     = 0x0;
    p_sid_ipf.p_sid_dma[i].channel_descriptor.padding_mode          = padding_mode_constant;
    p_sid_ipf.p_sid_dma[i].channel_descriptor.sampling_setup        = 0x0;
    p_sid_ipf.p_sid_dma[i].channel_descriptor.global_set_id         = 0;
    p_sid_ipf.p_sid_dma[i].channel_descriptor.ack_mode              = 1;  // Active ACK
    p_sid_ipf.p_sid_dma[i].channel_descriptor.ack_addr              = dma_channel_descriptor_ack_addr[i];  // ACK ADDR in IBUFF CTRL
    p_sid_ipf.p_sid_dma[i].channel_descriptor.ack_data              = 1;
    p_sid_ipf.p_sid_dma[i].channel_descriptor.completed_count       = 0x0;
    p_sid_ipf.p_sid_dma[i].unit_descriptor.unit_width               = NWAYS - 1;
    p_sid_ipf.p_sid_dma[i].unit_descriptor.unit_height              = ibuff_proc_items_p_unit[i] - 1;
    p_sid_ipf.p_sid_dma[i].span_descriptor[0].unit_location         = 0x0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[0].span_width            = p_sid_ipf.span_width_source[i] - 1;
    p_sid_ipf.p_sid_dma[i].span_descriptor[0].span_height           = p_sid_ipf.span_height_source[i] - 1;
    p_sid_ipf.p_sid_dma[i].span_descriptor[0].span_row              = 0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[0].span_column           = 0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[0].span_mode             = span_mode(span_order_column_first, addressing_mode_coordinate_based);
    p_sid_ipf.p_sid_dma[i].span_descriptor[1].unit_location         = 0x0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[1].span_width            = 0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[1].span_height           = 0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[1].span_row              = 0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[1].span_column           = 0;
    p_sid_ipf.p_sid_dma[i].span_descriptor[1].span_mode             = span_mode(span_order_column_first, addressing_mode_coordinate_based);
    p_sid_ipf.p_sid_dma[i].request_descriptor.descriptor_id_setup_1 = descriptor_id_setup_1(p_sid_ipf.p_sid_dma[i].unit_id, p_sid_ipf.p_sid_dma[i].terminal_A_id, p_sid_ipf.p_sid_dma[i].terminal_B_id, p_sid_ipf.p_sid_dma[i].channel_id);
    p_sid_ipf.p_sid_dma[i].request_descriptor.descriptor_id_setup_2 = descriptor_id_setup_2(p_sid_ipf.p_sid_dma[i].span_A_id, p_sid_ipf.p_sid_dma[i].span_B_id);
  }

  //
  // IBuffer Controller Configuration
  //
  unsigned int  mt_cfg_type = 2; // LOG2(nr bytes of cfg data)

  for(i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    dests_p_proc[i] = 1;
    has_2nd_buff[i] = 1;
  }

  p_sid_ipf.p_sids_ibuff.dests_p_proc = dests_p_proc;
  p_sid_ipf.p_sids_ibuff.has_2nd_buff = has_2nd_buff;

  p_sid_ipf.p_sids_ibuff.nr_procs      = VIED_NCI_INFEEDER_MAX_SIDS;
  p_sid_ipf.p_sids_ibuff.nr_feeders    = 0;

  p_sid_ipf.p_sids_ibuff.base_address = IPF_IBUFF_CTRL_SLV;
  p_sid_ipf.p_sids_ibuff.iwake_addr   = 0;   // Not used
  p_sid_ipf.p_sids_ibuff.error_irq_en = 0; // Not used

  ibc2600_setup_ibuf(&p_sid_ipf.p_sids_ibuff);

  for(i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    // IBuff CTRL for SID0 : Bayer1
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].feed_addr                 = 0;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].req_addr                  = (get_dma_request_addr(IPF_IBUFF_CTRL_MT_TO_DMA_SLV, p_sid_ipf.p_sid_dma[i].request_id)) >> mt_cfg_type;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].channel_addr              = (get_dma_channel_addr(IPF_IBUFF_CTRL_MT_TO_DMA_SLV, p_sid_ipf.p_sid_dma[i].channel_id)) >> mt_cfg_type;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].span_a_addr               = (get_dma_span_addr(IPF_IBUFF_CTRL_MT_TO_DMA_SLV, p_sid_ipf.p_sid_dma[i].span_A_id)) >> mt_cfg_type;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].span_b_addr               = (get_dma_span_addr(IPF_IBUFF_CTRL_MT_TO_DMA_SLV, p_sid_ipf.p_sid_dma[i].span_B_id)) >> mt_cfg_type;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].terminal_b_addr           = (get_dma_terminal_addr(IPF_IBUFF_CTRL_MT_TO_DMA_SLV, p_sid_ipf.p_sid_dma[i].terminal_A_id)) >>mt_cfg_type;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].dest_mode.bits.is_feeder  = 0;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].dest_mode.bits.config_dma = CONFIG_DMA_BY_IBUFF_CTRL;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].dest_mode.bits.iwake_en   = 0;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].dest_mode.bits.others     = 0;
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].st_addr                   = p_sid_ipf.start_address[i];  // Frame start of address of DMF SID 0
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].num_items                 = p_sid_ipf.num_units_per_fetch[i]; // nr units at destination before wrapping
    p_sid_ipf.p_sids_ibuff.dest_cfg[i].iwake_threshold           = 0; // not used at the moment
    p_sid_ipf.p_sids_ibuff.proc[i].cmd.sidpid                    = (0xf << 6) | i;
    p_sid_ipf.p_sids_ibuff.proc[i].cmd.ack_addr                  = ((IPF_IBUFF_CTRL_MT_TO_EQC_SLV + IPFD_2_SP_CTRL_TILE_EVQ_ADDR)  >> mt_cfg_type); // ((IPF_IBUFF_CTRL_MT_TO_EQC_SLV + (i * 0x100))  >> mt_cfg_type); // SP CTRL EVQ is ACK by default: Add this in the configuration during device_open
    p_sid_ipf.p_sids_ibuff.proc[i].cmd.snd_buf_cmd_addr          = ((IPF_IBUFF_CTRL_MT_TO_DMFS_SLV + ibuff_proc_snd_buf_cmd_addr[i]) >> mt_cfg_type);
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.str2mmio_addr             = ((IPF_IBUFF_CTRL_MT_TO_DMFS_SLV + ibuff_proc_str2mmio_addr[i]) >> mt_cfg_type);
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.store_cmd                 = 0; // store words
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.items_p_unit              = ibuff_proc_items_p_unit[i]; // X items (words) per unit
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.units_p_line              = 1; // 1 unit is a line, so 1
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.lines_p_frame             = p_sid_ipf.units_per_buffer[i];
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.units_p_ibuf              = p_sid_ipf.p_sids_ibuff.proc[i].cfg.units_p_line * p_sid_ipf.p_sids_ibuff.proc[i].cfg.lines_p_frame; // total amount of units in frame, maybe not needed
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.st_addr                   = 0x000; // not used anymore
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.sync_frame                = 1;
    p_sid_ipf.p_sids_ibuff.proc[i].cfg.dest_en[0]                = 1;
  }

  // Configuring DMF
  unsigned int sid_dmf;
  for(sid_dmf = 0; sid_dmf < VIED_NCI_INFEEDER_MAX_SIDS; sid_dmf++) {
    dmf_static_sid_config(p_sid_ipf.p_sid_dmf[sid_dmf]);
  }

  // Configuring DMF Sync
  unsigned int sid_dmfs;
  for(sid_dmfs = 0; sid_dmfs < VIED_NCI_INFEEDER_MAX_SIDS; sid_dmfs++) {
    dmfs_static_sid_config(p_sid_ipf.p_sid_dmfs[sid_dmfs], p_sid_ipf.p_s2m_dmfs[sid_dmfs]);
  }

  // Configuring DMA
  unsigned int sid_dma;
  dma_global_config();
  for(sid_dma = 0; sid_dma < VIED_NCI_INFEEDER_MAX_SIDS; sid_dma++) {
    dma_static_sid_config(p_sid_ipf.p_sid_dma[sid_dma]);
  }

  /* TODO remove configuring Event Queue Space Checker 
   * and move to platform code configuration phase
   */
  vied_nci_eqc_handle_t handle_eqc;

  handle_eqc = vied_nci_eqc_open(1);
  vied_nci_eqc_config(handle_eqc, p_id_eqc);
  vied_nci_eqc_close(handle_eqc);

  // Configuring IBuffer CTRL
  unsigned int sid_ibuff;
  for(sid_ibuff = 0; sid_ibuff < VIED_NCI_INFEEDER_MAX_SIDS; sid_ibuff++) {
    ibc2600_config_proc(&p_sid_ipf.p_sids_ibuff, sid_ibuff);
  }

  // Initialize the IBuffer CTRL with the configured values
  for (sid_ibuff = 0; sid_ibuff < VIED_NCI_INFEEDER_MAX_SIDS; sid_ibuff++) {
    ibc2600_set_sid_cmd_reg(&p_sid_ipf.p_sids_ibuff, sid_ibuff, _IBC_2600_PROC_CMD_CMD, _IBC_2600_CMD_INIT_VALUE);
  }

  for (i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    infeeder_lock_stream_configuration[i] = false;
  };

  static_config_done = true;

  return vied_nci_infeeder_msg_successful;
}


////////////////////////////////////////////////////
// Dynamic Configuration Layer
////////////////////////////////////////////////////

// Open and Close Input Feeder Cluster functions
vied_nci_infeeder_handle_t vied_nci_infeeder_open(vied_nci_infeeder_dev_id_t handle) {


  // Create the handles to the devices in the Input Feeder
  vied_nci_infeeder_static_config(handle);

  return 0;
}
// Open and Close Input Feeder Cluster functions
vied_nci_infeeder_stream_handle_t vied_nci_infeeder_stream_open(vied_nci_infeeder_dev_id_t handle, vied_nci_infeeder_stream_id_t sid) {

  // Create the handles to the devices in the Input Feeder
  NOT_USED(handle);
  return (vied_nci_infeeder_stream_handle_t)(sid);
}

// Configuration of stream
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_config(vied_nci_infeeder_stream_handle_t str_handle, unsigned int fifo_size, unsigned int num_units_per_fetch, unsigned int units_per_buffer, unsigned int span_width, unsigned int span_height, unsigned int region_stride) {

  int i;
  // Checks
  for(i = 0; i < VIED_NCI_INFEEDER_MAX_SIDS; i++) {
    // All streams should not be locked when doing a configuration
    if (infeeder_lock_stream_configuration[i]) return vied_nci_infeeder_msg_config_locked;
  }

  if((fifo_size < 1) || (fifo_size > IPF_DMF_MAX_FIFO_SIZE))     return vied_nci_infeeder_msg_fifo_size_exceeded;
  if(num_units_per_fetch > IPF_IBUFF_MAX_NUM_UNITS_PER_FETCH) return vied_nci_infeeder_msg_num_units_per_fetch_exceeded;
  if(units_per_buffer > IPF_IBUFF_MAX_UNITS_PER_BUFFER)       return vied_nci_infeeder_msg_units_per_buffer_exceeded;
  if(span_width > IPF_DMA_MAX_SPAN_WIDTH)                     return vied_nci_infeeder_msg_span_width_exceeded;
  if(span_height > IPF_DMA_MAX_SPAN_HEIGHT)                   return vied_nci_infeeder_msg_span_height_exceeded;
  if(region_stride > IPF_DMA_MAX_REGION_STRIDE)               return vied_nci_infeeder_msg_region_stride_exceeded;

  vied_nci_infeeder_stream_set_fifo_size          (str_handle, fifo_size);
  vied_nci_infeeder_stream_set_num_units_per_fetch(str_handle, num_units_per_fetch);
  vied_nci_infeeder_stream_set_units_per_buffer   (str_handle, units_per_buffer);
  vied_nci_infeeder_stream_set_span_width_source  (str_handle, span_width);
  vied_nci_infeeder_stream_set_span_height_source (str_handle, span_height);
  vied_nci_infeeder_stream_set_region_stride      (str_handle, region_stride);


  return vied_nci_infeeder_msg_successful;
}


// Start stream function
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_start(vied_nci_infeeder_stream_handle_t str_handle) {

  infeeder_lock_stream_configuration[str_handle] = true;

  return vied_nci_infeeder_msg_successful;
}



// Run stream function ==> does a buffer transfer
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_run(vied_nci_infeeder_stream_handle_t str_handle, vied_nci_infeeder_address_t start_address, vied_nci_infeeder_address_t ack_address) {
  // Checks
  if(p_sid_ipf.fifo_size[str_handle] == 0)               return vied_nci_infeeder_msg_fifo_size_exceeded;

  // Set ACK Address
  vied_nci_infeeder_stream_set_ack_address(str_handle, ack_address, 0);
  // Set Start Address (results in an Init event)
  vied_nci_infeeder_stream_set_start_address(str_handle, start_address);

  unsigned int cmd_token;

  // Run buffer_tranfers (results in an ACK event)
  cmd_token = (_IBC_2600_CMD_STORE_2ND_BUFFER_MODE << _IBC_2600_CMD_DEST_IDX(0));
  ibc2600_set_sid_cmd_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_PROC_CMD_CMD, cmd_token);

  return vied_nci_infeeder_msg_successful;
}


// Stop stream function
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_stop(vied_nci_infeeder_stream_handle_t str_handle) {
  // Checks
  if(p_sid_ipf.fifo_size[str_handle] == 0)               return vied_nci_infeeder_msg_fifo_size_exceeded;

  // CODE TO BE CHECKED!!!!
//  cmd_token = (_IBC_2600_CMD_DEST_DISABLE << _IBC_2600_CMD_DEST_IDX(0));
//  ibc2600_set_sid_cmd_reg(&p_sid_ipf.p_sids_ibuff, str_handle, _IBC_2600_PROC_CMD_CMD, cmd_token);

  return vied_nci_infeeder_msg_successful;
}

// Flush a stream function
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_flush(vied_nci_infeeder_stream_handle_t str_handle) {

  infeeder_lock_stream_configuration[str_handle] = false;

  return vied_nci_infeeder_msg_successful;
}


// Close a stream function
vied_nci_infeeder_msg_t vied_nci_infeeder_stream_close(vied_nci_infeeder_stream_handle_t str_handle) {

  infeeder_lock_stream_configuration[str_handle] = false;
  // Add code to put the FIFO size to 1 of a stream. All FIFOs should be empty!
  return vied_nci_infeeder_msg_successful;
}


// Close the device function
vied_nci_infeeder_msg_t vied_nci_infeeder_close(vied_nci_infeeder_stream_handle_t str_handle) {

  // Add code to release the handle to the devices in the Input Feeder
  NOT_USED(str_handle);
  return 0;
}

vied_nci_infeeder_msg_t vied_nci_infeeder_decode_event(vied_nci_infeeder_event_t event, vied_nci_infeeder_stream_handle_t *str_handle, vied_nci_infeeder_msg_t *decoded_event) {

  *str_handle = ((event >> 20) & 0x7);

  if(event & 0x1) {  // ACK received
    *decoded_event = vied_nci_infeeder_msg_ack_received;
    if(event & 0x8) {
      return vied_nci_infeeder_msg_event_failure;
    }
  }

  // Add code to release the handle to the devices in the Input Feeder
  return vied_nci_infeeder_msg_successful;
}





