#include <hrt/api.h>
#include <hrt/system.h>

#include "vied_nci_input_feeder_defines.h"
#include "vied_nci_input_feeder_dmf.h"



void dmf_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value){
  hrt_master_port_store(base_addr + reg * DMF_REG_ALIGNEMENT, &value, IPF_CFG_BUS_DATA_WIDTH/8);
}

unsigned int dmf_get_reg(unsigned int base_addr, unsigned int reg) {
  unsigned int value;
  hrt_master_port_load(base_addr + reg * DMF_REG_ALIGNEMENT, &value, IPF_CFG_BUS_DATA_WIDTH/8);
  return value;
}

void dmf_static_sid_config(t_sid_dmf p_sid_dmf) {
  // Configure SID of DMF
  dmf_set_reg(p_sid_dmf.base_addr, SID_BURST_TIMER_REG, p_sid_dmf.sid_burst_timer);
  dmf_set_reg(p_sid_dmf.base_addr, SID_BUFFER_SIZE_REG, p_sid_dmf.sid_buffer_size);
}

void dmf_set_burst_timer_sid(t_sid_dmf p_sid_dmf) {
  dmf_set_reg(p_sid_dmf.base_addr, SID_BURST_TIMER_REG, p_sid_dmf.sid_burst_timer);
}

void dmf_set_buffer_size_sid(t_sid_dmf p_sid_dmf) {
//  printf("Fifo size 2: %d, addr: 0x%x\n", p_sid_dmf.sid_buffer_size, p_sid_dmf.base_addr);
  dmf_set_reg(p_sid_dmf.base_addr, SID_BUFFER_SIZE_REG, p_sid_dmf.sid_buffer_size);
}

