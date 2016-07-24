#ifndef _sp2600_fp_cell_hrtx_h
#define _sp2600_fp_cell_hrtx_h

/* Identifiers to be used as arguments for hrt functions*/
enum sp2600_fp_memories {
  sp2600_fp_config_icache,
  sp2600_fp_dmem,
  sp2600_fp_qmem,
  sp2600_fp_cmem,
  sp2600_fp_xmem,
  sp2600_fp_dmem1,
  sp2600_fp_num_memories
};

enum sp2600_fp_slave_ports {
  sp2600_fp_sl_stat_ip_cell_port,
  sp2600_fp_sl_dmem_ip0_cell_port,
  sp2600_fp_sl_dmem_ip1_cell_port,
  sp2600_fp_num_slave_ports,
  sp2600_fp_unreachable_cell_port
};

enum sp2600_fp_master_interfaces {
  sp2600_fp_config_icache_master,
  sp2600_fp_qmem_master_int,
  sp2600_fp_cmem_master_int,
  sp2600_fp_xmem_master_int,
  sp2600_fp_num_master_interfaces
};

enum sp2600_fp_arbiters {
  sp2600_fp_dmem_Arb_mem_wp,
  sp2600_fp_dmem1_Arb_mem_wp,
  sp2600_fp_num_arbiters
};

enum sp2600_fp_arbiter_contenders {
  sp2600_fp_dmem_Arb_mem_wp_source1,
  sp2600_fp_dmem_Arb_mem_wp_source0,
  sp2600_fp_dmem1_Arb_mem_wp_source1,
  sp2600_fp_dmem1_Arb_mem_wp_source0,
  sp2600_fp_num_arbiter_contenders
};

#endif /* _sp2600_fp_cell_hrtx_h */
