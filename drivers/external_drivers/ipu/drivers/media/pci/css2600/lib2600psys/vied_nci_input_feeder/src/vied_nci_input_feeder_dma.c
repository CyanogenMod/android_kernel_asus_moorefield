#include "vied_nci_input_feeder_dma.h"
#include "print_support.h"

void dma_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value){
  hrt_master_port_store(base_addr + reg * DMA_REG_ALIGNEMENT, &value, IPF_CFG_BUS_DATA_WIDTH/8);
}

unsigned int dma_get_reg(unsigned int base_addr, unsigned int reg) {
  unsigned int value;
  hrt_master_port_load(base_addr + reg * DMA_REG_ALIGNEMENT, &value, IPF_CFG_BUS_DATA_WIDTH/8);
  return value;
}

int get_dma_request_addr (unsigned int base_addr, unsigned int request_id) {
  return base_addr + (REQUEST_GROUP_ID  << group_id_lsbidx()) + (request_id  << bank_id_lsbidx());
}


int get_dma_channel_addr (unsigned int base_addr, unsigned int channel_id) {
  return base_addr + (CHANNEL_GROUP_ID  << group_id_lsbidx()) + (channel_id  << bank_id_lsbidx());
}


int get_dma_span_addr    (unsigned int base_addr, unsigned int span_id) {
  return base_addr + (SPAN_GROUP_ID     << group_id_lsbidx()) + (span_id     << bank_id_lsbidx());
}


int get_dma_terminal_addr(unsigned int base_addr, unsigned int terminal_id){
  return base_addr + (TERMINAL_GROUP_ID << group_id_lsbidx()) + (terminal_id << bank_id_lsbidx());
}


int dma_static_sid_config(t_sid_dma p_sid_dma) {

  unsigned int terminal_A_id = p_sid_dma.terminal_A_id;
  unsigned int terminal_B_id = p_sid_dma.terminal_B_id;
  unsigned int span_A_id     = p_sid_dma.span_A_id;
  unsigned int span_B_id     = p_sid_dma.span_B_id;
  unsigned int channel_id    = p_sid_dma.channel_id;
  unsigned int unit_id       = p_sid_dma.unit_id;
  unsigned int request_id    = p_sid_dma.request_id;

  // Upload Terminals
  upload_terminal_descriptor(p_sid_dma.terminal_descriptor[0], terminal_A_id);
  upload_terminal_descriptor(p_sid_dma.terminal_descriptor[1], terminal_B_id);

  // Upload Spans
  upload_span_descriptor(p_sid_dma.span_descriptor[0], span_A_id);
  upload_span_descriptor(p_sid_dma.span_descriptor[1], span_B_id);

  // Upload Channel
  upload_channel_descriptor (p_sid_dma.channel_descriptor, channel_id);

  // Upload Unit
  upload_unit_descriptor(p_sid_dma.unit_descriptor, unit_id);

  // Upload Request descriptor
  upload_descriptors(p_sid_dma.request_descriptor, request_id);


#if 0
  terminal_descriptor_t   terminal_descriptor_tmp;
  span_descriptor_t       span_descriptor_tmp;

  if(!compare_terminal_descriptors(p_sid_dma.terminal_descriptor [0], download_terminal_descriptor( terminal_A_id))) {
    terminal_descriptor_tmp = download_terminal_descriptor    (     terminal_A_id);
    PERROR("in terminal descriptor A: %d\n", terminal_A_id);

    PRINT("region_origin  : %d - %d\n", p_sid_dma.terminal_descriptor[0].region_origin, terminal_descriptor_tmp.region_origin);
    PRINT("region_width   : %d - %d\n", p_sid_dma.terminal_descriptor[0].region_width, terminal_descriptor_tmp.region_width);
    PRINT("region_stride  : %d - %d\n", p_sid_dma.terminal_descriptor[0].region_stride, terminal_descriptor_tmp.region_stride);
    PRINT("element_setup  : %d - %d\n", p_sid_dma.terminal_descriptor[0].element_setup, terminal_descriptor_tmp.element_setup);
    PRINT("cio_info_setup : %d - %d\n", p_sid_dma.terminal_descriptor[0].cio_info_setup, terminal_descriptor_tmp.cio_info_setup);
    PRINT("port_mode      : %d - %d\n", p_sid_dma.terminal_descriptor[0].port_mode, terminal_descriptor_tmp.port_mode);
  }
  if(!compare_terminal_descriptors(p_sid_dma.terminal_descriptor [1], download_terminal_descriptor( terminal_B_id))) {
    terminal_descriptor_tmp = download_terminal_descriptor    (     terminal_B_id);
    PRINT("Error in terminal descriptor B: %d\n", terminal_B_id);

    PRINT("region_origin  : %d - %d\n", p_sid_dma.terminal_descriptor[1].region_origin, terminal_descriptor_tmp.region_origin);
    PRINT("region_width   : %d - %d\n", p_sid_dma.terminal_descriptor[1].region_width, terminal_descriptor_tmp.region_width);
    PRINT("region_stride  : %d - %d\n", p_sid_dma.terminal_descriptor[1].region_stride, terminal_descriptor_tmp.region_stride);
    PRINT("element_setup  : %d - %d\n", p_sid_dma.terminal_descriptor[1].element_setup, terminal_descriptor_tmp.element_setup);
    PRINT("cio_info_setup : %d - %d\n", p_sid_dma.terminal_descriptor[1].cio_info_setup, terminal_descriptor_tmp.cio_info_setup);
    PRINT("port_mode      : %d - %d\n", p_sid_dma.terminal_descriptor[1].port_mode, terminal_descriptor_tmp.port_mode);
  }
  if(!compare_channel_descriptors (p_sid_dma.channel_descriptor     , download_channel_descriptor (    channel_id))) PERROR("in channel descriptor %d\n", channel_id);
  if(!compare_unit_descriptors    (p_sid_dma.unit_descriptor        , download_unit_descriptor    (       unit_id))) PERROR("in unit descriptor %d\n", unit_id);
  if(!compare_span_descriptors    (p_sid_dma.span_descriptor     [0], download_span_descriptor    (     span_A_id))) {
    span_descriptor_tmp = download_span_descriptor    (     span_A_id);

    PERROR("in span descriptor A: %d\n", span_A_id);
    PRINT("unit  : %d - %d\n", p_sid_dma.span_descriptor[0].unit_location, span_descriptor_tmp.unit_location);
    PRINT("width : %d - %d\n", p_sid_dma.span_descriptor[0].span_width, span_descriptor_tmp.span_width);
    PRINT("height: %d - %d\n", p_sid_dma.span_descriptor[0].span_height, span_descriptor_tmp.span_height);
    PRINT("row   : %d - %d\n", p_sid_dma.span_descriptor[0].span_row, span_descriptor_tmp.span_row);
    PRINT("column: %d - %d\n", p_sid_dma.span_descriptor[0].span_column, span_descriptor_tmp.span_column);
    PRINT("mode  : %d - %d\n", p_sid_dma.span_descriptor[0].span_mode, span_descriptor_tmp.span_mode);
  }
  if(!compare_span_descriptors    (p_sid_dma.span_descriptor     [1], download_span_descriptor    (     span_B_id))) PERROR("in span descriptor B: %d\n", span_B_id);
#endif


  return 0;
}


void dma_global_config(void) {
  global_descriptor_t global_descriptor;
  master_descriptor_t master_descriptor[MASTER_BANKS];

  int global_id;
  int master_id;

  // upload DMA descriptors
  global_descriptor.unit_descriptor_base_addr     = 0x0;
  global_descriptor.span_descriptor_base_addr     = 0x0;
  global_descriptor.terminal_descriptor_base_addr = 0x0;
  global_descriptor.channel_descriptor_base_addr  = 0x0;
  global_descriptor.max_block_height              = MAX_BLOCK_HEIGHT - 1;
  for (global_id = 0; global_id < GLOBAL_SETS; global_id++) {
    global_descriptor.max_1d_block_width[global_id] = MAX_LINEAR_BURST_SIZE - 1;
    global_descriptor.max_2d_block_width[global_id] = MAX_BLOCK_WIDTH - 1;
  }

  for (master_id = 0; master_id < MASTER_BANKS; master_id++) {
    master_descriptor[master_id].srmd_support  = 0;
    master_descriptor[master_id].burst_support = 0;
    master_descriptor[master_id].max_stride    = MAX_STRIDE;
  }


  // Upload Global descriptors
  upload_global_descriptor(global_descriptor);

  // Upload Master descriptors
  for (master_id = 0; master_id < MASTER_BANKS; master_id++) {
    upload_master_descriptor(master_descriptor[master_id], master_id);
  }
}

