#include "unit_api.h"
#include "print_support.h"

/******************************************************
 * unit descriptor API functions
 ******************************************************/
int unit_descriptor_words()
{
  int unit_descriptor_words = (
                                BYTES(UNIT_WIDTH_BITS)  +
                                BYTES(UNIT_HEIGHT_BITS) +
                                CTRLM_DATA_BYTES - 1
                              ) / CTRLM_DATA_BYTES;

  return unit_descriptor_words;
}

void write_unit_descriptor(
  int unit_descriptor_base_addr,
  int unit_id,
  unit_descriptor_t unit_descriptor
)
{
  int base_addr = unit_descriptor_base_addr + unit_id * unit_descriptor_words() * CTRLM_DATA_BYTES;

  store_descriptor_field(&base_addr, unit_descriptor.unit_width,  UNIT_WIDTH_BITS );
  store_descriptor_field(&base_addr, unit_descriptor.unit_height, UNIT_HEIGHT_BITS);
}

unit_descriptor_t read_unit_descriptor(
  int unit_descriptor_base_addr,
  int unit_id
)
{
  unit_descriptor_t unit_descriptor;

  int base_addr = unit_descriptor_base_addr + unit_id * unit_descriptor_words() * CTRLM_DATA_BYTES;

  unit_descriptor.unit_width  = load_descriptor_field(&base_addr, UNIT_WIDTH_BITS );
  unit_descriptor.unit_height = load_descriptor_field(&base_addr, UNIT_HEIGHT_BITS);
  return unit_descriptor;
}

bool compare_unit_descriptors (
// returns true when descriptors are equal
  unit_descriptor_t d1,
  unit_descriptor_t d2
)
{
  bool equal = true;
  equal = equal && (d1.unit_width  == d2.unit_width );
  equal = equal && (d1.unit_height == d2.unit_height);
  return equal;
}

void print_unit_descriptor(
  unit_descriptor_t d
)
{
  PRINT("unit descriptor contents:\n");
  PRINT("  unit width  : %d\n", d.unit_width );
  PRINT("  unit height : %d\n", d.unit_height);
  return;
}

void upload_unit_descriptor(
  unit_descriptor_t unit_descriptor,
  int unit_id
)
{
  int bank_addr;

  if (unit_id >= UNIT_BANKS) {
    PERROR("attempting to upload a descriptor with unit id %d to a non-existent descriptor register bank (number of unit banks is %d)\n", unit_id, UNIT_BANKS);
    exit(1);
  }

  bank_addr = (UNIT_GROUP_ID << group_id_lsbidx()) + (unit_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES, unit_descriptor.unit_width);
//   dma_hrt_store_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES, unit_descriptor.unit_height);

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES, unit_descriptor.unit_width);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES, unit_descriptor.unit_height);

}

unit_descriptor_t download_unit_descriptor(
  int unit_id
)
{
  int bank_addr;
  unit_descriptor_t unit_descriptor;

  if (unit_id >= UNIT_BANKS) {
    PERROR("attempting to download a descriptor with unit id %d from a non-existent descriptor register bank (number of unit banks is %d)\n", unit_id, UNIT_BANKS);
    exit(1);
  }

  bank_addr = (UNIT_GROUP_ID << group_id_lsbidx()) + (unit_id << bank_id_lsbidx());

//   unit_descriptor.unit_width  = dma_hrt_load_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES);
//   unit_descriptor.unit_height = dma_hrt_load_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES);

  unit_descriptor.unit_width  = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES);
  unit_descriptor.unit_height = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES);

  return unit_descriptor;
}

void set_unit_bank_mode(
/* sets the bank mode of bank <unit_bank_id> to <bank_mode> */
  int unit_bank_id,
  bank_mode_t bank_mode
)
{
  int bank_addr;

    if (unit_bank_id >= UNIT_BANKS) {
    PERROR("attempting to download a descriptor with unit id %d from a non-existent descriptor register bank (number of unit banks is %d)\n", unit_bank_id, UNIT_BANKS);
    exit(1);
  }

  bank_addr = (UNIT_GROUP_ID << group_id_lsbidx()) + (unit_bank_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
}
