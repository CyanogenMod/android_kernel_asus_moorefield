#ifndef _model_h_
#define _model_h_

#include "base_api.h"
#include "request_api.h"
#include "unit_api.h"
#include "span_api.h"
#include "terminal_api.h"
#include "channel_api.h"

/*extern void load_block(
  char ***elem,
  int  master_id,
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor,
  terminal_descriptor_t terminal_descriptor,
  channel_descriptor_t  channel_descriptor,
  bool write_enable
);
*/
extern int check_operation_transfer (
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor_a,
  span_descriptor_t     span_descriptor_b,
  terminal_descriptor_t terminal_descriptor_a,
  terminal_descriptor_t terminal_descriptor_b,
  channel_descriptor_t  channel_descriptor
);

extern int check_instruction_transfer (
  request_descriptor_t  request_descriptor,
  unit_descriptor_t     unit_descriptor,
  span_descriptor_t     span_descriptor_a,
  span_descriptor_t     span_descriptor_b,
  terminal_descriptor_t terminal_descriptor_a,
  terminal_descriptor_t terminal_descriptor_b,
  channel_descriptor_t  channel_descriptor
);

#endif
