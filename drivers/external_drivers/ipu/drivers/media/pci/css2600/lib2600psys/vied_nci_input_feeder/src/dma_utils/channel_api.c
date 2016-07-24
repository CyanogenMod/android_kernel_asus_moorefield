#include "channel_api.h"
#include "print_support.h"

extern char* get_padding_mode_name(
/* returns a string reflecting the name of the <padding_mode> */
  padding_mode_t padding_mode
)
{
  switch (padding_mode) {
    case padding_mode_constant : return "constant"; break;
    case padding_mode_clone    : return "clone";    break;
    case padding_mode_mirror   : return "mirror";   break;
    case padding_mode_append   : return "append";   break;
    case padding_mode_truncate : return "truncate"; break;
  }
  return "unknown";
}

unsigned int get_subsampling_factor(
/* returns the subsampling factor corresponding to the specified sampling setup (subsampling index) value */
  unsigned int sampling_setup
)
{
  unsigned int factors[UNIQUE_SUBSAMPLING_FACTORS] = UNIQUE_SUBSAMPLING_FACTOR_LIST;
  return factors[sampling_setup];
}

/******************************************************
 * channel descriptor API functions
 ******************************************************/
int channel_descriptor_words()
{
  int channel_descriptor_words = (
                                   BYTES(ELEMENT_EXTEND_MODE_BITS) +
                                   BYTES(ELEMENT_INIT_DATA_BITS)   +
                                   BYTES(PADDING_MODE_BITS)        +
                                   BYTES(SAMPLING_SETUP_BITS)      +
                                   BYTES(GLOBAL_SET_ID_BITS)       +
                                   BYTES(ACK_MODE_BITS)            +
                                   BYTES(ACK_ADDR_BITS)            +
                                   BYTES(ACK_DATA_BITS)            +
                                   BYTES(COMPLETED_COUNT_BITS)     +
                                   CTRLM_DATA_BYTES - 1
                                 ) / CTRLM_DATA_BYTES;

  return channel_descriptor_words;
}

void write_channel_descriptor(
  int channel_descriptor_base_addr,
  int channel_id,
  channel_descriptor_t channel_descriptor
)
{
  int base_addr = channel_descriptor_base_addr + channel_id * channel_descriptor_words() * CTRLM_DATA_BYTES;

  store_descriptor_field(&base_addr, channel_descriptor.element_extend_mode, ELEMENT_EXTEND_MODE_BITS);
  store_descriptor_field(&base_addr, channel_descriptor.element_init_data,   ELEMENT_INIT_DATA_BITS  );
  store_descriptor_field(&base_addr, channel_descriptor.padding_mode,        PADDING_MODE_BITS       );
  store_descriptor_field(&base_addr, channel_descriptor.sampling_setup,      SAMPLING_SETUP_BITS     );
  store_descriptor_field(&base_addr, channel_descriptor.global_set_id,       GLOBAL_SET_ID_BITS      );
  store_descriptor_field(&base_addr, channel_descriptor.ack_mode,            ACK_MODE_BITS           );
  store_descriptor_field(&base_addr, channel_descriptor.ack_addr,            ACK_ADDR_BITS           );
  store_descriptor_field(&base_addr, channel_descriptor.ack_data,            ACK_DATA_BITS           );
  store_descriptor_field(&base_addr, channel_descriptor.completed_count,     COMPLETED_COUNT_BITS    );
}

channel_descriptor_t read_channel_descriptor(
  int channel_descriptor_base_addr,
  int channel_id
)
{
  channel_descriptor_t channel_descriptor;

  int base_addr = channel_descriptor_base_addr + channel_id * channel_descriptor_words() * CTRLM_DATA_BYTES;

  channel_descriptor.element_extend_mode = load_descriptor_field(&base_addr, ELEMENT_EXTEND_MODE_BITS);
  channel_descriptor.element_init_data   = load_descriptor_field(&base_addr, ELEMENT_INIT_DATA_BITS  );
  channel_descriptor.padding_mode        = load_descriptor_field(&base_addr, PADDING_MODE_BITS       );
  channel_descriptor.sampling_setup      = load_descriptor_field(&base_addr, SAMPLING_SETUP_BITS     );
  channel_descriptor.global_set_id       = load_descriptor_field(&base_addr, GLOBAL_SET_ID_BITS      );
  channel_descriptor.ack_mode            = load_descriptor_field(&base_addr, ACK_MODE_BITS           );
  channel_descriptor.ack_addr            = load_descriptor_field(&base_addr, ACK_ADDR_BITS           );
  channel_descriptor.ack_data            = load_descriptor_field(&base_addr, ACK_DATA_BITS           );
  channel_descriptor.completed_count     = load_descriptor_field(&base_addr, COMPLETED_COUNT_BITS    );
  return channel_descriptor;
}

bool compare_channel_descriptors (
// returns true when descriptors are equal
  channel_descriptor_t d1,
  channel_descriptor_t d2
)
{
  bool equal = true;
  equal = equal && (d1.element_extend_mode == d2.element_extend_mode);
  equal = equal && (d1.element_init_data   == d2.element_init_data  );
  equal = equal && (d1.padding_mode        == d2.padding_mode       );
  equal = equal && (d1.sampling_setup      == d2.sampling_setup     );
  equal = equal && (d1.global_set_id       == d2.global_set_id      );
  equal = equal && (d1.ack_mode            == d2.ack_mode           );
  equal = equal && (d1.ack_addr            == d2.ack_addr           );
  equal = equal && (d1.ack_data            == d2.ack_data           );
  equal = equal && (d1.completed_count     == d2.completed_count    );
  return equal;
}

void print_channel_descriptor(
  channel_descriptor_t d
)
{
  PRINT("channel descriptor contents:\n");
  PRINT("  element extend mode    : %d\n", d.element_extend_mode);
  PRINT("  element init data bits : %d\n", d.element_init_data  );
  PRINT("  padding mode           : %d\n", d.padding_mode       );
  PRINT("  sampling setup         : %d\n", d.sampling_setup     );
  PRINT("  global set id          : %d\n", d.global_set_id      );
  PRINT("  ack mode               : %d\n", d.ack_mode           );
  PRINT("  ack addr               : %d\n", d.ack_addr           );
  PRINT("  ack data               : %d\n", d.ack_data           );
  PRINT("  completed count        : %d\n", d.completed_count    );
  return;
}

void upload_channel_descriptor(
  channel_descriptor_t channel_descriptor,
  int channel_id
)
{
  int bank_addr;

  if (channel_id >= CHANNEL_BANKS) {
    PERROR("attempting to upload a descriptor with channel id %d to a non-existent descriptor register bank (number of channel banks is %d)\n", channel_id, CHANNEL_BANKS);
    exit(1);
  }

  bank_addr = (CHANNEL_GROUP_ID << group_id_lsbidx()) + (channel_id << bank_id_lsbidx());

/*  dma_hrt_store_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES, channel_descriptor.element_extend_mode);
  dma_hrt_store_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES, channel_descriptor.element_init_data  );
  dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, channel_descriptor.padding_mode       );
  dma_hrt_store_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES, channel_descriptor.sampling_setup     );
  dma_hrt_store_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES, channel_descriptor.global_set_id      );
  dma_hrt_store_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES, channel_descriptor.ack_mode           );
  dma_hrt_store_value(DMA_ID, bank_addr + 6 * CTRLS_DATA_BYTES, channel_descriptor.ack_addr           );
  dma_hrt_store_value(DMA_ID, bank_addr + 7 * CTRLS_DATA_BYTES, channel_descriptor.ack_data           );*/
  // purposely do not upload anything to the completed counter, as this register is read-only

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES, channel_descriptor.element_extend_mode);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES, channel_descriptor.element_init_data  );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, channel_descriptor.padding_mode       );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES, channel_descriptor.sampling_setup     );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES, channel_descriptor.global_set_id      );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES, channel_descriptor.ack_mode           );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 6 * CTRLS_DATA_BYTES, channel_descriptor.ack_addr           );
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 7 * CTRLS_DATA_BYTES, channel_descriptor.ack_data           );



}

channel_descriptor_t download_channel_descriptor(
  int channel_id
)
{
  int bank_addr;
  channel_descriptor_t channel_descriptor;

  if (channel_id >= CHANNEL_BANKS) {
    PERROR("attempting to download a descriptor with channel id %d from a non-existent descriptor register bank (number of channel banks is %d)\n", channel_id, CHANNEL_BANKS);
    exit(1);
  }

  bank_addr = (CHANNEL_GROUP_ID << group_id_lsbidx()) + (channel_id << bank_id_lsbidx());

//   channel_descriptor.element_extend_mode = dma_hrt_load_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES);
//   channel_descriptor.element_init_data   = dma_hrt_load_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES);
//   channel_descriptor.padding_mode        = dma_hrt_load_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES);
//   channel_descriptor.sampling_setup      = dma_hrt_load_value(DMA_ID, bank_addr + 3 * CTRLS_DATA_BYTES);
//   channel_descriptor.global_set_id       = dma_hrt_load_value(DMA_ID, bank_addr + 4 * CTRLS_DATA_BYTES);
//   channel_descriptor.ack_mode            = dma_hrt_load_value(DMA_ID, bank_addr + 5 * CTRLS_DATA_BYTES);
//   channel_descriptor.ack_addr            = dma_hrt_load_value(DMA_ID, bank_addr + 6 * CTRLS_DATA_BYTES);
//   channel_descriptor.ack_data            = dma_hrt_load_value(DMA_ID, bank_addr + 7 * CTRLS_DATA_BYTES);
//   channel_descriptor.completed_count     = dma_hrt_load_value(DMA_ID, bank_addr + 11 * CTRLS_DATA_BYTES);

  channel_descriptor.element_extend_mode = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES);
  channel_descriptor.element_init_data   = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES);
  channel_descriptor.padding_mode        = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES);
  channel_descriptor.sampling_setup      = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 3 * CTRLS_DATA_BYTES);
  channel_descriptor.global_set_id       = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 4 * CTRLS_DATA_BYTES);
  channel_descriptor.ack_mode            = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 5 * CTRLS_DATA_BYTES);
  channel_descriptor.ack_addr            = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 6 * CTRLS_DATA_BYTES);
  channel_descriptor.ack_data            = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 7 * CTRLS_DATA_BYTES);
  channel_descriptor.completed_count     = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 11 * CTRLS_DATA_BYTES);

  return channel_descriptor;
}

unsigned int read_completed_counter (
/* returns the value of the completed counter in channel bank <channel_bank_id> */
  int channel_bank_id
)
{
  int bank_addr;

  if (channel_bank_id >= CHANNEL_BANKS) {
    PERROR("attempting to read the completed counter valued using a channel bank id %d that points to a non-existent channel register bank (number of channel banks is %d)\n", channel_bank_id, CHANNEL_BANKS);
    exit(1);
  }

  bank_addr = (CHANNEL_GROUP_ID << group_id_lsbidx()) + (channel_bank_id << bank_id_lsbidx());

  PRINT("Bank addr: 0x%x\n", bank_addr);
//   return dma_hrt_load_value(DMA_ID, bank_addr + 11 * CTRLS_DATA_BYTES);
  return dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 11 * CTRLS_DATA_BYTES);
}

void set_channel_bank_mode(
/* sets the bank mode of bank <channel_bank_id> to <bank_mode> */
  int channel_bank_id,
  bank_mode_t bank_mode
)
{
  int bank_addr;

    if (channel_bank_id >= CHANNEL_BANKS) {
    PERROR("attempting to download a descriptor with channel id %d from a non-existent descriptor register bank (number of channel banks is %d)\n", channel_bank_id, CHANNEL_BANKS);
    exit(1);
  }

  bank_addr = (CHANNEL_GROUP_ID << group_id_lsbidx()) + (channel_bank_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 15 * CTRLS_DATA_BYTES, (unsigned int) bank_mode);
}

int wait_for_acknowledge (
  channel_descriptor_t channel_descriptor,
  int channel_id
)
{
  int completed;
  unsigned int word;

  if (channel_descriptor.ack_mode == 0) { /* passive acknowledge */
    /* read the completed counter register */
    do {
      hrt_sleep();
      completed = read_completed_counter(channel_id);
      PRINT("polling for passive acknowledge by reading completed requests on channel %d: read 0x%x.\n", channel_id, completed);
    } while (completed == 0);
  }
  else { /* ensure active acknowledge mode == number of executed commands */
    do {
      hrt_sleep();
//       word = _hrt_slave_port_load_32(HRTCAT(MEMORY1,_ip0), channel_descriptor.ack_addr - MEMORY1_BASE_ADDR);
      word =channel_descriptor.ack_data; //FB: dummy
      PRINT("polling for active acknowledge by reading data at memory1 address 0x%x: read 0x%x.\n", channel_descriptor.ack_addr, word);
    } while (word != channel_descriptor.ack_data);
    completed = channel_descriptor.ack_mode;
  }
  return completed;
}
