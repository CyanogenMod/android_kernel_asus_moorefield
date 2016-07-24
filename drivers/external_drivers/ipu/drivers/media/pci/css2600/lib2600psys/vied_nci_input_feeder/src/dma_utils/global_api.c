#include "global_api.h"
#include "print_support.h"

/******************************************************
 * global descriptor API functions
 ******************************************************/

bool compare_global_descriptors (
// returns true when descriptors are equal
  global_descriptor_t d1,
  global_descriptor_t d2
)
{
  int global_set_id;
  bool equal = true;
  equal = equal && (d1.unit_descriptor_base_addr     == d2.unit_descriptor_base_addr    );
  equal = equal && (d1.span_descriptor_base_addr     == d2.span_descriptor_base_addr    );
  equal = equal && (d1.terminal_descriptor_base_addr == d2.terminal_descriptor_base_addr);
  equal = equal && (d1.channel_descriptor_base_addr  == d2.channel_descriptor_base_addr );
  equal = equal && (d1.max_block_height              == d2.max_block_height             );
  for (global_set_id = 0; global_set_id < GLOBAL_SETS; global_set_id++) {
    equal = equal && (d1.max_1d_block_width[global_set_id] == d2.max_1d_block_width[global_set_id]);
    equal = equal && (d1.max_2d_block_width[global_set_id] == d2.max_2d_block_width[global_set_id]);
  }
  return equal;
}

void print_global_descriptor(
  global_descriptor_t d
)
{
  int global_set_id;

  PRINT("global descriptor contents:\n");
  PRINT("  unit descriptor base addr     : %d\n", d.unit_descriptor_base_addr    );
  PRINT("  span descriptor base addr     : %d\n", d.span_descriptor_base_addr    );
  PRINT("  terminal descriptor base addr : %d\n", d.terminal_descriptor_base_addr);
  PRINT("  channel descriptor base addr  : %d\n", d.channel_descriptor_base_addr );
  PRINT("  max block height              : %d\n", d.max_block_height             );
  for (global_set_id = 0; global_set_id < GLOBAL_SETS; global_set_id++) {
    PRINT("  max 1D block width[%02d]  : %d\n", global_set_id, d.max_1d_block_width[global_set_id]);
    PRINT("  max 2D block width[%02d]  : %d\n", global_set_id, d.max_2d_block_width[global_set_id]);
  }
  return;
}

void upload_global_descriptor(
  global_descriptor_t global_descriptor
)
{
  int global_set_id;
  int bank_addr;

  bank_addr = (GLOBAL_GROUP_ID << group_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, global_descriptor.unit_descriptor_base_addr    );
//   dma_hrt_store_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES, global_descriptor.span_descriptor_base_addr    );
//   dma_hrt_store_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES, global_descriptor.terminal_descriptor_base_addr);
//   dma_hrt_store_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES, global_descriptor.channel_descriptor_base_addr );
//   dma_hrt_store_value(DMA_ID, bank_addr + 6 * CTRLS_DATA_BYTES, global_descriptor.max_block_height             );
//   for (global_set_id = 0; global_set_id < GLOBAL_SETS; global_set_id++) {
//     dma_hrt_store_value(DMA_ID, bank_addr + (7 + 2*global_set_id) * CTRLS_DATA_BYTES, global_descriptor.max_1d_block_width[global_set_id]);
//     dma_hrt_store_value(DMA_ID, bank_addr + (8 + 2*global_set_id) * CTRLS_DATA_BYTES, global_descriptor.max_2d_block_width[global_set_id]);
//   }

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, global_descriptor.unit_descriptor_base_addr    );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES, global_descriptor.span_descriptor_base_addr    );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES, global_descriptor.terminal_descriptor_base_addr);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES, global_descriptor.channel_descriptor_base_addr );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 6 * CTRLS_DATA_BYTES, global_descriptor.max_block_height             );
  for (global_set_id = 0; global_set_id < GLOBAL_SETS; global_set_id++) {
    dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + (7 + 2*global_set_id) * CTRLS_DATA_BYTES, global_descriptor.max_1d_block_width[global_set_id]);
    dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + (8 + 2*global_set_id) * CTRLS_DATA_BYTES, global_descriptor.max_2d_block_width[global_set_id]);
  }
}

global_descriptor_t download_global_descriptor()
{
  int global_set_id;
  int bank_addr;
  global_descriptor_t global_descriptor;

  bank_addr = (GLOBAL_GROUP_ID << group_id_lsbidx());

//   global_descriptor.unit_descriptor_base_addr     = dma_hrt_load_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES);
//   global_descriptor.span_descriptor_base_addr     = dma_hrt_load_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES);
//   global_descriptor.terminal_descriptor_base_addr = dma_hrt_load_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES);
//   global_descriptor.channel_descriptor_base_addr  = dma_hrt_load_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES);
//   global_descriptor.max_block_height              = dma_hrt_load_value(DMA_ID, bank_addr + 6 * CTRLS_DATA_BYTES);
//   for (global_set_id = 0; global_set_id < GLOBAL_SETS; global_set_id++) {
//     global_descriptor.max_1d_block_width[global_set_id] = dma_hrt_load_value(DMA_ID, bank_addr + (7 + 2*global_set_id) * CTRLS_DATA_BYTES);
//     global_descriptor.max_2d_block_width[global_set_id] = dma_hrt_load_value(DMA_ID, bank_addr + (8 + 2*global_set_id) * CTRLS_DATA_BYTES);
//   }

  global_descriptor.unit_descriptor_base_addr     = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES);
  global_descriptor.span_descriptor_base_addr     = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES);
  global_descriptor.terminal_descriptor_base_addr = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES);
  global_descriptor.channel_descriptor_base_addr  = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES);
  global_descriptor.max_block_height              = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 6 * CTRLS_DATA_BYTES);
  for (global_set_id = 0; global_set_id < GLOBAL_SETS; global_set_id++) {
    global_descriptor.max_1d_block_width[global_set_id] = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + (7 + 2*global_set_id) * CTRLS_DATA_BYTES);
    global_descriptor.max_2d_block_width[global_set_id] = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + (8 + 2*global_set_id) * CTRLS_DATA_BYTES);
  }
  return global_descriptor;
}

