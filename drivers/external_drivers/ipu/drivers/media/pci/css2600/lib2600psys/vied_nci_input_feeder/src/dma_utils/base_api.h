#ifndef _base_api_h_
#define _base_api_h_

#include <hrt/api.h>
#include <hrt/system.h>
#include "type_support.h"
#include <host.h>
// #include <input_feeder_test_system_defs.h>
#include "vied_nci_input_feeder_defines.h"

#define DMA_BASE_ADDR IPF_DMA4_SLV_PORT

#define dma_hrt_store_value(base_addr, reg_addr, val) \
  _hrt_master_port_store_32_volatile((base_addr) + (reg_addr), val);

#define dma_hrt_load_value(base_addr, reg_addr) \
  _hrt_master_port_load_32_volatile((base_addr) + (reg_addr))


/*

#define dma_hrt_store_value(dev_id, addr, val) \
  _hrt_slave_port_store_32_volatile(HRTCAT(dev_id,_s0), (addr), (val))

#define dma_hrt_load_value(dev_id, addr) \
  _hrt_slave_port_load_32(HRTCAT(dev_id,_s0), (addr))

#define _hrt_store_value(dev_id, addr, val) \
  _hrt_slave_port_store_32_volatile(HRTCAT(dev_id,_slv_in), (addr), (val))

#define _hrt_load_value(dev_id, addr) \
  _hrt_slave_port_load_32(HRTCAT(dev_id,_slv_in), (addr))*/

#include "defines.h"

#define BYTES(bits) ((bits + 7) / 8)

//#define MIN(a,b) (a<b?a:b)
#define MAX(a,b) (a>b?a:b)

#define CTRLS_DATA_BYTES BYTES(CTRLS_DATA_BITS)
#define CTRLM_DATA_BYTES BYTES(CTRLM_DATA_BITS)

// typedef int bool;
#define true  1
#define false 0

typedef enum {
  bank_mode_non_cached,
  bank_mode_cached
} bank_mode_t;

extern int _log2(int n);

extern int group_id_lsbidx(void);
extern int bank_id_lsbidx(void);


extern void store_descriptor_field(
/* stores a single descriptor field with <value> encoded in specified number of <bits> in memory at specified <addr>ess */
  int *addr,
  unsigned int value,
  int bits
);

extern unsigned int load_descriptor_field(
/* loads a single descriptor field encoded in specified number of <bits> from memory at specified <addr>ess */
  int *addr,
  int bits
);

#endif
