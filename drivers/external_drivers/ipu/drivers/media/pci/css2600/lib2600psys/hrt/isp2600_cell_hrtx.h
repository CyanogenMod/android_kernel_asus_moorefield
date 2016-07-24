#ifndef _isp2600_cell_hrtx_h
#define _isp2600_cell_hrtx_h

/* Identifiers to be used as arguments for hrt functions*/
enum isp2600_memories {
  isp2600_base_config_mem,
  isp2600_base_dmem,
  isp2600_qmem,
  isp2600_cmem,
  isp2600_xmem,
  isp2600_simd_bamem,
  isp2600_simd_vmem,
  isp2600_simd_vec_master,
  isp2600_num_memories
};

enum isp2600_slave_ports {
  isp2600_sl_stat_ip_cell_port,
  isp2600_sl_pmem_ip_cell_port,
  isp2600_sl_dmem_ip_cell_port,
  isp2600_sl_bmem_ip_cell_port,
  isp2600_sl_vmem_ip_cell_port,
  isp2600_num_slave_ports,
  isp2600_unreachable_cell_port
};

enum isp2600_master_interfaces {
  isp2600_base_config_mem_master,
  isp2600_qmem_isp_std_master,
  isp2600_cmem_isp_std_master,
  isp2600_xmem_isp_std_master,
  isp2600_simd_vec_master_isp_vec_master,
  isp2600_num_master_interfaces
};

enum isp2600_arbiters {
  isp2600_base_config_mem_Arb_prg_mem_wp,
  isp2600_base_dmem_Arb_data_mem_wp0,
  isp2600_simd_vmem_Arb_vec_dmem_wp0,
  isp2600_num_arbiters
};

enum isp2600_arbiter_contenders {
  isp2600_base_config_mem_Arb_prg_mem_wp_source1,
  isp2600_base_config_mem_Arb_prg_mem_wp_source0,
  isp2600_base_dmem_Arb_data_mem_wp0_source1,
  isp2600_base_dmem_Arb_data_mem_wp0_source0,
  isp2600_simd_vmem_Arb_vec_dmem_wp0_source1,
  isp2600_simd_vmem_Arb_vec_dmem_wp0_source0,
  isp2600_num_arbiter_contenders
};

#endif /* _isp2600_cell_hrtx_h */
