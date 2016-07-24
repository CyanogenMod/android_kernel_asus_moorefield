#include "psys.h"
#include "psys_tb.h"

void set_info_bits_mmu(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_unps_logic_mmu_at_system_mmu0_ctrl_in_master_port_address+0x8, info_bit);
}

void set_info_bits_spc(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x14, info_bit);  /* info bits for cache master */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x2c, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x38, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x44, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x50, info_bit);  /* info bits cmem master bus  */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x5c, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x68, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x74, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spc_tile_cfg_bus_sl_master_port_address+0x80, info_bit);  /* info bits xmem master bus  */
}

void set_info_bits_spp0(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x14, info_bit);  /* info bits for cache master */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x2c, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x38, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x44, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x50, info_bit);  /* info bits cmem master bus  */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x5c, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x68, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x74, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile0_cfg_bus_sl_master_port_address+0x80, info_bit);  /* info bits xmem master bus  */
}

void set_info_bits_spp1(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x14, info_bit);  /* info bits for cache master */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x2c, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x38, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x44, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x50, info_bit);  /* info bits cmem master bus  */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x5c, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x68, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x74, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spp_tile1_cfg_bus_sl_master_port_address+0x80, info_bit);  /* info bits xmem master bus  */
}

void set_info_bits_spf(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x14, info_bit);  /* info bits for cache master */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x2c, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x38, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x44, info_bit);  /* info bits cmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x50, info_bit);  /* info bits cmem master bus  */

  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x5c, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x68, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x74, info_bit);  /* info bits xmem master bus  */
  vied_subsystem_store_32(psys0, processing_system_unps_logic_spf_tile_cfg_bus_sl_master_port_address+0x80, info_bit);  /* info bits xmem master bus  */
}


void set_info_bits_isp_tile0(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_isp_tile0_logic_cfg_bus_sl_master_port_address+0x14, info_bit); /* ISP ICACHE */

  vied_subsystem_store_32(psys0, processing_system_isp_tile0_logic_cfg_bus_sl_master_port_address+0x8c, info_bit); /* Data Master segment 0 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile0_logic_cfg_bus_sl_master_port_address+0x98, info_bit); /* Data Master segment 1 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile0_logic_cfg_bus_sl_master_port_address+0xA4, info_bit); /* Data Master segment 2 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile0_logic_cfg_bus_sl_master_port_address+0xB0, info_bit); /* Data Master segment 3 */
}

void set_info_bits_isp_tile1(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_isp_tile1_logic_cfg_bus_sl_master_port_address+0x14, info_bit); /* ISP ICACHE */

  vied_subsystem_store_32(psys0, processing_system_isp_tile1_logic_cfg_bus_sl_master_port_address+0x8c, info_bit); /* Data Master segment 0 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile1_logic_cfg_bus_sl_master_port_address+0x98, info_bit); /* Data Master segment 1 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile1_logic_cfg_bus_sl_master_port_address+0xA4, info_bit); /* Data Master segment 2 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile1_logic_cfg_bus_sl_master_port_address+0xB0, info_bit); /* Data Master segment 3 */
}

void set_info_bits_isp_tile2(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_isp_tile2_logic_cfg_bus_sl_master_port_address+0x14, info_bit); /* ISP ICACHE */

  vied_subsystem_store_32(psys0, processing_system_isp_tile2_logic_cfg_bus_sl_master_port_address+0x8c, info_bit); /* Data Master segment 0 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile2_logic_cfg_bus_sl_master_port_address+0x98, info_bit); /* Data Master segment 1 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile2_logic_cfg_bus_sl_master_port_address+0xA4, info_bit); /* Data Master segment 2 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile2_logic_cfg_bus_sl_master_port_address+0xB0, info_bit); /* Data Master segment 3 */
}

void set_info_bits_isp_tile3(unsigned int info_bit) {
  vied_subsystem_store_32(psys0, processing_system_isp_tile3_logic_cfg_bus_sl_master_port_address+0x14, info_bit); /* ISP ICACHE */

  vied_subsystem_store_32(psys0, processing_system_isp_tile3_logic_cfg_bus_sl_master_port_address+0x8c, info_bit); /* Data Master segment 0 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile3_logic_cfg_bus_sl_master_port_address+0x98, info_bit); /* Data Master segment 1 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile3_logic_cfg_bus_sl_master_port_address+0xA4, info_bit); /* Data Master segment 2 */
  vied_subsystem_store_32(psys0, processing_system_isp_tile3_logic_cfg_bus_sl_master_port_address+0xB0, info_bit); /* Data Master segment 3 */
}



void set_info_bits(unsigned int info_bit) {
  set_info_bits_mmu(info_bit);
  set_info_bits_spc(info_bit);
  set_info_bits_spp0(info_bit);
  set_info_bits_spp1(info_bit);
  set_info_bits_spf(info_bit);
  set_info_bits_isp_tile0(info_bit);
  set_info_bits_isp_tile1(info_bit);
  set_info_bits_isp_tile2(info_bit);
  set_info_bits_isp_tile3(info_bit);
}
