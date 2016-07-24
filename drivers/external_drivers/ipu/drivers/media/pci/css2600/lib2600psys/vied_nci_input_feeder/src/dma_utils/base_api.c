#include "base_api.h"
#include "print_support.h"


/*
int _log2(int n) {
  return (int) ceil(log((double)n) / log(2));
}
*/

int _log2(int a)
{
        int b = 1;
        int c = 0;
        for (; b<a; b<<=1, c++);
        return c;
}


 /**********************************
  * slave address decomposition
  **********************************/
int bank_id_lsbidx()
{
  int byte_index_bits  = _log2(BYTES(CTRLM_DATA_BITS));
  int max_regs = MAX(REQUEST_REGS, MAX(UNIT_REGS, MAX(SPAN_REGS, MAX(TERMINAL_REGS, MAX(CHANNEL_REGS, MAX(MASTER_REGS, GLOBAL_REGS))))));
  int register_index_bits = _log2(max_regs);
  int lsbidx = register_index_bits + byte_index_bits;
  return lsbidx;
}

int group_id_lsbidx()
{
  int max_banks = MAX(REQUEST_BANKS, MAX(UNIT_BANKS, MAX(SPAN_BANKS, MAX(TERMINAL_BANKS, MAX(CHANNEL_BANKS, MAX(MASTER_BANKS, GLOBAL_BANKS))))));
  int bank_index_bits  = _log2(max_banks);
  int lsbidx = bank_index_bits + bank_id_lsbidx();
  return lsbidx;
}


void store_descriptor_field(int *addr, unsigned int value, int bits)
{
  int b;
  int bytes = (bits + 7) / 8;
  unsigned int byte = 0;

  for (b = 0; b < bytes; b++) {
    byte = (value >> (b*8)) & 0xFF;
    (*addr)++;
  }
}

unsigned int load_descriptor_field(int *addr, int bits)
{
  unsigned int value = 0;
  int b;
  int bytes = (bits + 7) / 8;
  unsigned int byte = 0;

  for (b = 0; b < bytes; b++) {
    value |= (byte << (b*8));
    (*addr)++;
  }
  return value;
}


