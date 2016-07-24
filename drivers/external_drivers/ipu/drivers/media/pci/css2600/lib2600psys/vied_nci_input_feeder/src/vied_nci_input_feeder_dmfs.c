#include <hrt/api.h>
#include <hrt/system.h>

// #include "vied_nci_input_feeder_defines.h"
#include "vied_nci_input_feeder_dmfs.h"

void dmfs_set_reg(unsigned int base_addr, unsigned int reg, unsigned int value){
  hrt_master_port_store(base_addr + reg * DMFS_REG_ALIGNEMENT, &value, DMFS_CFG_BUS_DATA_WIDTH/8);
}

unsigned int dmfs_get_reg(unsigned int base_addr, unsigned int reg) {
  unsigned int value;
  hrt_master_port_load(base_addr + reg * DMFS_REG_ALIGNEMENT, &value, DMFS_CFG_BUS_DATA_WIDTH/8);
  return value;
}

void dmfs_static_sid_config(t_sid_dmfs p_sid_dmfs, t_s2m_dmfs p_s2m_dmfs) {


  // Configure SID of DMF Sync
  dmfs_set_reg(p_sid_dmfs.base_addr, SID_ACK_RESPONSE_ADDR_REG, p_sid_dmfs.sid_ack_response_addr);
  dmfs_set_reg(p_sid_dmfs.base_addr,      SID_ACK_RESPONSE_REG, p_sid_dmfs.sid_ack_response);
  dmfs_set_reg(p_sid_dmfs.base_addr,   SID_SET_BUFFER_SIZE_REG, p_sid_dmfs.sid_set_buffer_size);
  dmfs_set_reg(p_sid_dmfs.base_addr, SID_AVAIL_BUFFER_SIZE_REG, p_sid_dmfs.sid_compute_unit_size);

  // Configure s2M of DMF Sync
  dmfs_set_reg(p_s2m_dmfs.base_addr, S2M_ACK_RESPONSE_ADDR_REG, p_s2m_dmfs.s2m_ack_response_addr);
  dmfs_set_reg(p_s2m_dmfs.base_addr,         S2M_NUM_ITEMS_REG, p_s2m_dmfs.s2m_num_items);

}

void dmfs_set_buffer_size_sid(t_sid_dmfs p_sid_dmfs) {
  dmfs_set_reg(p_sid_dmfs.base_addr, SID_SET_BUFFER_SIZE_REG, p_sid_dmfs.sid_set_buffer_size);
}

