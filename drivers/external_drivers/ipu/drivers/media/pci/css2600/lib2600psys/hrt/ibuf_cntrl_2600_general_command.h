#ifndef _ibuf_cntrl_2600_general_command_h_
#define _ibuf_cntrl_2600_general_command_h_

#include <ibuf_cntrl_2600_defs.h>

typedef enum {
  disable = _IBC_2600_CMD_DEST_DISABLE,
  addr_next_token = _IBC_2600_CMD_STORE_ADDR_NEXT_TOKEN,
  addr_register = _IBC_2600_CMD_STORE_ADDR_CONFIG,
  second_buf_mode = _IBC_2600_CMD_STORE_2ND_BUFFER_MODE
} ibc2600_cmd_type;

typedef struct {
  ibc2600_cmd_type cmd;
  unsigned int address;
} ibc2600_dest_cmd_s;

typedef struct {
  unsigned char nr_dests;
  ibc2600_dest_cmd_s *dest;
} ibc2600_cmd_s;

typedef struct {
  unsigned int cmd_address;
  unsigned int cmd_token;
  unsigned char nr_nxt_addr;
  unsigned int *addresses;
} ibc2600_cmd_encoded_s;

#endif
