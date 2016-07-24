#include "request_api.h"
#include "print_support.h"

transfer_direction_t get_transfer_direction(
/* returns the transfer direction encoded in an 'execute' instruction */
  unsigned int instruction
)
{
  return (instruction >> 3) & 0x1;
}

extern int get_macro_size(
/* returns the macro size encoded in an 'execute' instruction */
  unsigned int instruction
)
{
  return (instruction >> 11) & ~((-1) << MACRO_SIZE_BITS);
}

bool is_execute_instruction(
/* returns true if the instruction is in 'execute' format; false otherwise */
  unsigned int instruction
)
{
  return ((instruction & 0x1) == 0);
}

unsigned int execute_instruction(
/* creates an instruction field out of the various field components */
  bool                 transfer_ack,
  bool                 transfer_flush,
  transfer_direction_t transfer_direction,
  transfer_kind_t      transfer_kind,
  bool                 unit_invalidate,
  bool                 span_a_invalidate,
  bool                 span_b_invalidate,
  bool                 terminal_a_invalidate,
  bool                 terminal_b_invalidate,
  bool                 channel_invalidate,
  unsigned int         macro_size
)
{
  unsigned int instruction;

  unsigned int format = 0;

  instruction =                       macro_size            & ~((-1) << MACRO_SIZE_BITS);
  instruction = (instruction << 1) | (channel_invalidate    & 0x1);
  instruction = (instruction << 1) | (terminal_b_invalidate & 0x1);
  instruction = (instruction << 1) | (terminal_a_invalidate & 0x1);
  instruction = (instruction << 1) | (span_b_invalidate     & 0x1);
  instruction = (instruction << 1) | (span_a_invalidate     & 0x1);
  instruction = (instruction << 1) | (unit_invalidate       & 0x1);
  instruction = (instruction << 1) | (transfer_kind         & 0x1);
  instruction = (instruction << 1) | (transfer_direction    & 0x1);
  instruction = (instruction << 1) | (transfer_flush        & 0x1);
  instruction = (instruction << 1) | (transfer_ack          & 0x1);
  instruction = (instruction << 1) | (format                & 0x1);
  return instruction;
}

unsigned int invalidate_instruction(
/* creates an instruction field in 'execute' format out of the various field components */
  descriptor_kind_t    descriptor_kind,
  unsigned int         lower_id,
  unsigned int         upper_id
)
{
  unsigned int instruction;

  unsigned int format = 1;

  instruction =                                          upper_id        & ~((-1) << DESCRIPTOR_ID_BITS  );
  instruction = (instruction << DESCRIPTOR_ID_BITS  ) | (lower_id        & ~((-1) << DESCRIPTOR_ID_BITS  ));
  instruction = (instruction << DESCRIPTOR_KIND_BITS) | (descriptor_kind & ~((-1) << DESCRIPTOR_KIND_BITS));
  instruction = (instruction << 1)                    | (format          & 0x1);
  return instruction;
}

unsigned int descriptor_id_setup_1(
/* creates a descriptor id setup 1 field out of the various field components */
  unsigned int unit_id,
  unsigned int terminal_a_id,
  unsigned int terminal_b_id,
  unsigned int channel_id
)
{
  unsigned int field;

  field =                                channel_id    & ~((-1) << _log2(CHANNELS));
  field = (field << _log2(TERMINALS)) | (terminal_b_id & ~((-1) << _log2(TERMINALS)));
  field = (field << _log2(TERMINALS)) | (terminal_a_id & ~((-1) << _log2(TERMINALS)));
  field = (field << _log2(UNITS))     | (unit_id       & ~((-1) << _log2(UNITS)));
  return field;
}

extern unsigned int descriptor_id_setup_2(
/* creates a descriptor id setup 2 field out of the various field components */
  unsigned int span_a_id,
  unsigned int span_b_id
)
{
  unsigned int field;

  field =                            span_b_id & ~((-1) << _log2(SPANS));
  field = (field << _log2(SPANS)) | (span_a_id & ~((-1) << _log2(SPANS)));
  return field;
}


/******************************************************
 * request descriptor API functions
 ******************************************************/
bool compare_request_descriptors (
// returns true when descriptors are equal
  request_descriptor_t d1,
  request_descriptor_t d2
)
{
  bool equal = true;
  equal = equal && (d1.instruction           == d2.instruction          );
  equal = equal && (d1.descriptor_id_setup_1 == d2.descriptor_id_setup_1);
  equal = equal && (d1.descriptor_id_setup_2 == d2.descriptor_id_setup_2);
  return equal;
}

void print_request_descriptor(
  request_descriptor_t d
)
{
  PRINT("request descriptor contents:\n");
  PRINT("  instruction           : %d\n", d.instruction          );
  PRINT("  descriptor id setup 1 : %d\n", d.descriptor_id_setup_1);
  PRINT("  descriptor id setup 2 : %d\n", d.descriptor_id_setup_2);
  return;
}

void upload_request_descriptor(
  request_descriptor_t request_descriptor,
  int request_id
)
{
  int bank_addr;

  if (request_id >= REQUEST_BANKS) {
    PERROR("attempting to upload a descriptor with request id %d to a non-existent descriptor register bank (number of request banks is %d)\n", request_id, REQUEST_BANKS);
    exit(1);
  }

  bank_addr = (REQUEST_GROUP_ID << group_id_lsbidx()) + (request_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_1);
//   dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_2);
//   /* write the instruction last, as this validates the request and temporarily blocks further access to the request register bank */
//   dma_hrt_store_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES, request_descriptor.instruction          );

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_1);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_2);
  /* write the instruction last, as this validates the request and temporarily blocks further access to the request register bank */
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES, request_descriptor.instruction          );

}

void upload_descriptors(
    request_descriptor_t request_descriptor,
  int request_id
                              )
{
  int bank_addr;

  if (request_id >= REQUEST_BANKS) {
    PERROR("attempting to upload a descriptor with request id %d to a non-existent descriptor register bank (number of request banks is %d)\n", request_id, REQUEST_BANKS);
    exit(1);
  }

  bank_addr = (REQUEST_GROUP_ID << group_id_lsbidx()) + (request_id << bank_id_lsbidx());

//   dma_hrt_store_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_1);
//   dma_hrt_store_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_2);

  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_1);
  dma_hrt_store_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_2);

//   printf("Requester %d: addr : 0x%08x, data: 0x%08x\n", request_id, bank_addr + 1 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_1);
//   printf("Requester %d: addr : 0x%08x, data: 0x%08x\n", request_id, bank_addr + 2 * CTRLS_DATA_BYTES, request_descriptor.descriptor_id_setup_2);
}

request_descriptor_t download_request_descriptor(
  int request_id
)
{
  int bank_addr;
  request_descriptor_t request_descriptor;

  if (request_id >= REQUEST_BANKS) {
    PERROR("attempting to download a descriptor with request id %d from a non-existent descriptor register bank (number of request banks is %d)\n", request_id, REQUEST_BANKS);
    exit(1);
  }

  bank_addr = (REQUEST_GROUP_ID << group_id_lsbidx()) + (request_id << bank_id_lsbidx());

//   request_descriptor.instruction           = dma_hrt_load_value(DMA_ID, bank_addr + 0 * CTRLS_DATA_BYTES);
//   request_descriptor.descriptor_id_setup_1 = dma_hrt_load_value(DMA_ID, bank_addr + 1 * CTRLS_DATA_BYTES);
//   request_descriptor.descriptor_id_setup_2 = dma_hrt_load_value(DMA_ID, bank_addr + 2 * CTRLS_DATA_BYTES);

  request_descriptor.instruction           = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 0 * CTRLS_DATA_BYTES);
  request_descriptor.descriptor_id_setup_1 = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 1 * CTRLS_DATA_BYTES);
  request_descriptor.descriptor_id_setup_2 = dma_hrt_load_value(DMA_BASE_ADDR, bank_addr + 2 * CTRLS_DATA_BYTES);

  return request_descriptor;
}

