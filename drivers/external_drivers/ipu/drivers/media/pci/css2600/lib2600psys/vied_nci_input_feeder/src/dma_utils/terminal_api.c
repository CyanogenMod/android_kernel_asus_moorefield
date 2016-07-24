#include "terminal_api.h"
#include "print_support.h"


extern unsigned int get_precision(
/* returns the element precision corresponding to the specified element setup (element precision index) value */
  unsigned int element_setup
)
{
  unsigned int precisions[UNIQUE_PRECISIONS] = UNIQUE_PRECISION_LIST;
  return precisions[element_setup];
}

extern bool is_strict_precision(
/* returns true if the <precision> provided as argument is strict; returns false otherwise. */
  unsigned int precision
)
{
  bool is_pow_of_2      = ((precision & (precision - 1)) == 0);
  bool is_multiple_of_8 = ((precision % 8) == 0);
  return is_pow_of_2 && is_multiple_of_8;
}

/******************************************************
 * terminal descriptor API functions
 ******************************************************/
int terminal_descriptor_words()
{
  int terminal_descriptor_words = (
                                BYTES(REGION_ORIGIN_BITS)  +
                                BYTES(REGION_WIDTH_BITS)   +
                                BYTES(REGION_STRIDE_BITS)  +
                                BYTES(ELEMENT_SETUP_BITS)  +
                                BYTES(CIO_INFO_SETUP_BITS) +
                                BYTES(PORT_MODE_BITS) +
                                CTRLM_DATA_BYTES - 1
                              ) / CTRLM_DATA_BYTES;
  return terminal_descriptor_words;
}

void write_terminal_descriptor(
  int terminal_descriptor_base_addr,
  int terminal_id,
  terminal_descriptor_t terminal_descriptor
)
{
  int base_addr = terminal_descriptor_base_addr + terminal_id * terminal_descriptor_words() * CTRLM_DATA_BYTES;

  store_descriptor_field(&base_addr, terminal_descriptor.region_origin,  REGION_ORIGIN_BITS );
  store_descriptor_field(&base_addr, terminal_descriptor.region_width,   REGION_WIDTH_BITS  );
  store_descriptor_field(&base_addr, terminal_descriptor.region_stride,  REGION_STRIDE_BITS );
  store_descriptor_field(&base_addr, terminal_descriptor.element_setup,  ELEMENT_SETUP_BITS );
  store_descriptor_field(&base_addr, terminal_descriptor.cio_info_setup, CIO_INFO_SETUP_BITS);
  store_descriptor_field(&base_addr, terminal_descriptor.port_mode,      PORT_MODE_BITS);
}

terminal_descriptor_t read_terminal_descriptor(
  int terminal_descriptor_base_addr,
  int terminal_id
)
{
  terminal_descriptor_t terminal_descriptor;

  int base_addr = terminal_descriptor_base_addr + terminal_id * terminal_descriptor_words() * CTRLM_DATA_BYTES;

  terminal_descriptor.region_origin  = load_descriptor_field(&base_addr, REGION_ORIGIN_BITS );
  terminal_descriptor.region_width   = load_descriptor_field(&base_addr, REGION_WIDTH_BITS  );
  terminal_descriptor.region_stride  = load_descriptor_field(&base_addr, REGION_STRIDE_BITS );
  terminal_descriptor.element_setup  = load_descriptor_field(&base_addr, ELEMENT_SETUP_BITS );
  terminal_descriptor.cio_info_setup = load_descriptor_field(&base_addr, CIO_INFO_SETUP_BITS);
  terminal_descriptor.port_mode      = load_descriptor_field(&base_addr, PORT_MODE_BITS     );
  return terminal_descriptor;
}

bool compare_terminal_descriptors (
// returns true when descriptors are equal
  terminal_descriptor_t d1,
  terminal_descriptor_t d2
)
{
  bool equal = true;
  equal = equal && (d1.region_origin  == d2.region_origin );
  equal = equal && (d1.region_width   == d2.region_width  );
  equal = equal && (d1.region_stride  == d2.region_stride );
  equal = equal && (d1.element_setup  == d2.element_setup );
  equal = equal && (d1.cio_info_setup == d2.cio_info_setup);
  equal = equal && (d1.port_mode      == d2.port_mode     );
  return equal;
}

void print_terminal_descriptor(
  terminal_descriptor_t d
)
{
  PRINT("terminal descriptor contents:\n");
  PRINT("  region origin  : %d\n", d.region_origin );
  PRINT("  region width   : %d\n", d.region_width  );
  PRINT("  region stride  : %d\n", d.region_stride );
  PRINT("  element setup  : %d\n", d.element_setup );
  PRINT("  cio info setup : %d\n", d.cio_info_setup);
  PRINT("  port mode      : %d\n", d.port_mode     );
  return;
}

void upload_terminal_descriptor(
  terminal_descriptor_t terminal_descriptor,
  int terminal_id
)
{
  int bank_addr;

  if (terminal_id >= TERMINAL_BANKS) {
    PERROR("attempting to upload a descriptor with terminal id %d to a non-existent descriptor register bank (number of terminal banks is %d)\n", terminal_id, TERMINAL_BANKS);
    exit(1);
  }

  bank_addr = (TERMINAL_GROUP_ID << group_id_lsbidx()) + (terminal_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES, terminal_descriptor.region_origin );
//   dma_hrt_store_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES, terminal_descriptor.region_width  );
//   dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, terminal_descriptor.region_stride );
//   dma_hrt_store_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES, terminal_descriptor.element_setup );
//   dma_hrt_store_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES, terminal_descriptor.cio_info_setup);
//   dma_hrt_store_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES, terminal_descriptor.port_mode     );

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES, terminal_descriptor.region_origin );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES, terminal_descriptor.region_width  );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, terminal_descriptor.region_stride );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES, terminal_descriptor.element_setup );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES, terminal_descriptor.cio_info_setup);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES, terminal_descriptor.port_mode     );
}

terminal_descriptor_t download_terminal_descriptor(
  int terminal_id
)
{
  int bank_addr;
  terminal_descriptor_t terminal_descriptor;

  if (terminal_id >= TERMINAL_BANKS) {
    PERROR("attempting to download a descriptor with terminal id %d from a non-existent descriptor register bank (number of terminal banks is %d)\n", terminal_id, TERMINAL_BANKS);
    exit(1);
  }

  bank_addr = (TERMINAL_GROUP_ID << group_id_lsbidx()) + (terminal_id << bank_id_lsbidx());

//   terminal_descriptor.region_origin  = dma_hrt_load_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES);
//   terminal_descriptor.region_width   = dma_hrt_load_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES);
//   terminal_descriptor.region_stride  = dma_hrt_load_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES);
//   terminal_descriptor.element_setup  = dma_hrt_load_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES);
//   terminal_descriptor.cio_info_setup = dma_hrt_load_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES);
//   terminal_descriptor.port_mode      = dma_hrt_load_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES);

  terminal_descriptor.region_origin  = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES);
  terminal_descriptor.region_width   = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES);
  terminal_descriptor.region_stride  = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES);
  terminal_descriptor.element_setup  = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES);
  terminal_descriptor.cio_info_setup = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES);
  terminal_descriptor.port_mode      = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES);

  return terminal_descriptor;
}

void set_terminal_bank_mode(
/* sets the bank mode of bank <terminal_bank_id> to <bank_mode> */
  int terminal_bank_id,
  bank_mode_t bank_mode
)
{
  int bank_addr;

    if (terminal_bank_id >= TERMINAL_BANKS) {
    PERROR("attempting to download a descriptor with terminal id %d from a non-existent descriptor register bank (number of terminal banks is %d)\n", terminal_bank_id, TERMINAL_BANKS);
    exit(1);
  }

  bank_addr = (TERMINAL_GROUP_ID << group_id_lsbidx()) + (terminal_bank_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
}
