#ifndef _sp2600_proxy_cell_hrtx_h
#define _sp2600_proxy_cell_hrtx_h

/* Identifiers to be used as arguments for hrt functions*/
enum sp2600_proxy_memories {
  sp2600_proxy_config_icache,
  sp2600_proxy_dmem,
  sp2600_proxy_qmem,
  sp2600_proxy_cmem,
  sp2600_proxy_xmem,
  sp2600_proxy_num_memories
};

enum sp2600_proxy_slave_ports {
  sp2600_proxy_sl_stat_ip_cell_port,
  sp2600_proxy_sl_dmem_ip_cell_port,
  sp2600_proxy_num_slave_ports,
  sp2600_proxy_unreachable_cell_port
};

enum sp2600_proxy_master_interfaces {
  sp2600_proxy_config_icache_master,
  sp2600_proxy_qmem_master_int,
  sp2600_proxy_cmem_master_int,
  sp2600_proxy_xmem_master_int,
  sp2600_proxy_num_master_interfaces
};

enum sp2600_proxy_arbiters {
  sp2600_proxy_dmem_Arb_mem_wp,
  sp2600_proxy_num_arbiters
};

enum sp2600_proxy_arbiter_contenders {
  sp2600_proxy_dmem_Arb_mem_wp_source1,
  sp2600_proxy_dmem_Arb_mem_wp_source0,
  sp2600_proxy_num_arbiter_contenders
};

#endif /* _sp2600_proxy_cell_hrtx_h */
