#ifndef _IPF_DMA4_H_
#define _IPF_DMA4_H_

#include "vied_nci_input_feeder_defines.h"
#include "dma_utils/api.h"

#define FIFO_MODE  1
#define MMIO_MODE  0

#define ELEMENT_PRECISION_8  0
#define ELEMENT_PRECISION_16 1

#define DMA_REG_ALIGNEMENT ((IPF_CFG_BUS_ADDR_WIDTH)/(8))

typedef struct s_sid_dma {
  unsigned int base_addr;
  unsigned int terminal_A_id;
  unsigned int terminal_B_id;
  unsigned int channel_id;
  unsigned int unit_id;
  unsigned int span_A_id;
  unsigned int span_B_id;
  unsigned int request_id;
  global_descriptor_t   global_descriptor  [GLOBAL_BANKS];
  master_descriptor_t   master_descriptor  [MASTER_BANKS];
  terminal_descriptor_t terminal_descriptor[2];
  channel_descriptor_t  channel_descriptor;
  unit_descriptor_t     unit_descriptor;
  span_descriptor_t     span_descriptor[2];
  request_descriptor_t  request_descriptor;
} t_sid_dma;



void dma_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value);
unsigned int dma_get_reg(unsigned int base_addr, unsigned int reg);

int get_dma_request_addr (unsigned int base_addr, unsigned int request_id);
int get_dma_channel_addr (unsigned int base_addr, unsigned int channel_id);
int get_dma_span_addr    (unsigned int base_addr, unsigned int span_id);
int get_dma_terminal_addr(unsigned int base_addr, unsigned int terminal_id);


int dma_static_sid_config(t_sid_dma p_sid_dma);
void dma_global_config(void);


#endif

