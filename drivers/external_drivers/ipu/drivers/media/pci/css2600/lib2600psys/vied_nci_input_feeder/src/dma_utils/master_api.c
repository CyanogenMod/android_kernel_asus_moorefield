#include "master_api.h"
#include "print_support.h"

/******************************************************
 * master descriptor API functions
 ******************************************************/

bool compare_master_descriptors (
// returns true when descriptors are equal
  master_descriptor_t d1,
  master_descriptor_t d2
)
{
  bool equal = true;
  equal = equal && (d1.srmd_support  == d2.srmd_support );
  equal = equal && (d1.burst_support == d2.burst_support);
  equal = equal && (d1.max_stride    == d2.max_stride   );
  return equal;
}

void print_master_descriptor(
  master_descriptor_t d
)
{
  PRINT("master descriptor contents:\n");
  PRINT("  srmd support  : %d\n", d.srmd_support );
  PRINT("  burst support : %d\n", d.burst_support);
  PRINT("  max stride    : %d\n", d.max_stride   );
  return;
}

void upload_master_descriptor(
  master_descriptor_t master_descriptor,
  int master_id
)
{
  int bank_addr;

  if (master_id >= MASTER_BANKS) {
    PERROR("attempting to upload a descriptor with master id %d to a non-existent descriptor register bank (number of master banks is %d)\n", master_id, MASTER_BANKS);
    exit(1);
  }

  bank_addr = (MASTER_GROUP_ID << group_id_lsbidx()) + (master_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES, master_descriptor.srmd_support );
//   dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, master_descriptor.burst_support);
//   dma_hrt_store_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES, master_descriptor.max_stride   );

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES, master_descriptor.srmd_support );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, master_descriptor.burst_support);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES, master_descriptor.max_stride   );

}

master_descriptor_t download_master_descriptor(
  int master_id
)
{
  int bank_addr;
  master_descriptor_t master_descriptor;

  if (master_id >= MASTER_BANKS) {
    PERROR("attempting to download a descriptor with master id %d from a non-existent descriptor register bank (number of master banks is %d)\n", master_id, MASTER_BANKS);
    exit(1);
  }

  bank_addr = (MASTER_GROUP_ID << group_id_lsbidx()) + (master_id << bank_id_lsbidx());

//   master_descriptor.srmd_support  = dma_hrt_load_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES);
//   master_descriptor.burst_support = dma_hrt_load_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES);
//   master_descriptor.max_stride    = dma_hrt_load_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES);

  master_descriptor.srmd_support  = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES);
  master_descriptor.burst_support = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES);
  master_descriptor.max_stride    = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES);
  return master_descriptor;
}

