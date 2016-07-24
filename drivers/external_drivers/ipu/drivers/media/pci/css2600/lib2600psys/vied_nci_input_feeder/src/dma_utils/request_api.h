#ifndef _request_api_h_
#define _request_api_h_

#include "defines.h"
#include "base_api.h"


typedef struct {
  unsigned int instruction          ;
  unsigned int descriptor_id_setup_1;
  unsigned int descriptor_id_setup_2;
} request_descriptor_t;

typedef enum {
  instruction_format_execute,
  instruction_format_invalidate
} instruction_format_t;

typedef enum {
  transfer_direction_ab,
  transfer_direction_ba
} transfer_direction_t;

typedef enum {
  transfer_kind_init,
  transfer_kind_move
} transfer_kind_t;

typedef enum {
  descriptor_kind_channel,
  descriptor_kind_terminal,
  descriptor_kind_unit,
  descriptor_kind_span,
} descriptor_kind_t;

extern transfer_direction_t get_transfer_direction(
/* returns the transfer direction encoded in an 'execute' instruction */
  unsigned int instruction
);

extern int get_macro_size(
/* returns the macro size encoded in an 'execute' instruction */
  unsigned int instruction
);

extern bool is_execute_instruction(
/* returns true if the instruction is in 'execute' format; false otherwise */
  unsigned int instruction
);

extern unsigned int execute_instruction(
/* creates an instruction field in 'execute' format out of the various field components */
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
);

extern unsigned int invalidate_instruction(
/* creates an instruction field in 'execute' format out of the various field components */
  descriptor_kind_t    descriptor_kind,
  unsigned int         lower_id,
  unsigned int         upper_id
);

extern unsigned int descriptor_id_setup_1(
/* creates a descriptor id setup 1 field out of the various field components */
  unsigned int unit_id,
  unsigned int terminal_a_id,
  unsigned int terminal_b_id,
  unsigned int channel_id
);

extern unsigned int descriptor_id_setup_2(
/* creates a descriptor id setup 2 field out of the various field components */
  unsigned int span_a_id,
  unsigned int span_b_id
);

extern bool compare_request_descriptors(
/* compares two request descriptors and returns true when the descriptors are equal, false if they are not equal */
  request_descriptor_t d1,
  request_descriptor_t d2
);

extern void print_request_descriptor(
/* prints the contents of a request descriptor */
  request_descriptor_t d
);

extern void upload_request_descriptor(
/* uploads a request descriptor to DMA request descriptor register bank with id <request_id> */
  request_descriptor_t request_descriptor,
  int request_id
);

extern void upload_descriptors(
  /* uploads a request descriptor to DMA request descriptor register bank with id <request_id> */
  request_descriptor_t request_descriptor,
  int request_id
);

extern request_descriptor_t download_request_descriptor(
/* downloads a request descriptor from DMA request descriptor register bank with id <request_id> */
  int request_id
);


#endif
