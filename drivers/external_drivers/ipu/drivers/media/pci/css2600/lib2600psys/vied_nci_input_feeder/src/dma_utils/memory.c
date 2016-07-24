#include "memory.h"

#include "defines.h"

#include <hrt/api.h>
#include <hrt/system.h>
#include <host.h>

#define MEMORY  global_mem
#define MEMORY_BASE_ADDR mem_ip0_master_port_address

void initialize_memory_block(block_t block)
{
  int x, y, addr, row_addr;
  char row_data, data;

  row_data = 0x0;
  row_addr = block.start_addr;
  for (y = 0; y < block.height; y++) {
    addr = row_addr;
    data = row_data;
    for (x = 0; x < block.width; x++) {
      if (block.memory_id == 0) {
        _hrt_slave_port_store_8(HRTCAT(MEMORY,_ip0), addr, data);
      }
      addr += 1;
      data += 1;
    }
    row_addr += block.stride;
    row_data ++;
  }
}

void initialize_memory_block_32bits(block_t block)
{
  int x, y, i, addr, row_addr;
  char row_data, data_byte;
  unsigned int data;

  data = 0x0;
  row_data = 0x0;
  row_addr = block.start_addr;
  for (y = 0; y < block.height; y++) {
    data = 0x0;
    addr = row_addr;
    for (x = 0; x < block.width/4; x++) {
      for(i = 0; i < 32/8; i++) {
        data = data | (row_data << 8*i);
        row_data = (row_data + 1) % 0x7F;
      }
      if (block.memory_id == 0) {
        _hrt_slave_port_store_32(HRTCAT(MEMORY,_ip0), addr, data);
      }
      data = 0x0;
      addr += 4;
    }
    row_addr += block.stride;
  }
}

