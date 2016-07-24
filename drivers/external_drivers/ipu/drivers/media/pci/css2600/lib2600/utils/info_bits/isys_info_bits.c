#include <isys.h>
#include "isys_tb.h"

void set_info_bits_mmu_isys(unsigned int info_bit) {
  vied_subsystem_store_32(isys0, input_system_unis_logic_mmu_at_system_mmu0_ctrl_in_master_port_address+0x8, info_bit);
  vied_subsystem_store_32(isys0, input_system_unis_logic_mmu_at_system_mmu1_ctrl_in_master_port_address+0x8, info_bit);
}

void set_info_bits_spc_isys(unsigned int info_bit) {
  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x14, info_bit);  /* info bits for cache master */

  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x2c, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x38, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x44, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x50, info_bit);  /* info bits cmem master bus  */

  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x5c, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x68, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x74, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(isys0, input_system_unis_logic_sp_control_tile_cfg_bus_sl_master_port_address+0x80, info_bit);  /* info bits xmem master bus  */
}



void set_info_bits_isys(unsigned int info_bit) {
  set_info_bits_mmu_isys(info_bit);
  set_info_bits_spc_isys(info_bit);
}
