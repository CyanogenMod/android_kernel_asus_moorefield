#ifndef _sp2600_control_cell_hrtx_h
#define _sp2600_control_cell_hrtx_h

/* Identifiers to be used as arguments for hrt functions*/
enum sp2600_control_memories {
  sp2600_control_config_icache,
  sp2600_control_dmem,
  sp2600_control_qmem,
  sp2600_control_cmem,
  sp2600_control_xmem,
  sp2600_control_num_memories
};

enum sp2600_control_slave_ports {
  sp2600_control_sl_stat_ip_cell_port,
  sp2600_control_sl_dmem_ip_cell_port,
  sp2600_control_num_slave_ports,
  sp2600_control_unreachable_cell_port
};

enum sp2600_control_master_interfaces {
  sp2600_control_config_icache_master,
  sp2600_control_qmem_master_int,
  sp2600_control_cmem_master_int,
  sp2600_control_xmem_master_int,
  sp2600_control_num_master_interfaces
};

enum sp2600_control_arbiters {
  sp2600_control_dmem_Arb_mem_wp,
  sp2600_control_num_arbiters
};

enum sp2600_control_arbiter_contenders {
  sp2600_control_dmem_Arb_mem_wp_source1,
  sp2600_control_dmem_Arb_mem_wp_source0,
  sp2600_control_num_arbiter_contenders
};

#endif /* _sp2600_control_cell_hrtx_h */
