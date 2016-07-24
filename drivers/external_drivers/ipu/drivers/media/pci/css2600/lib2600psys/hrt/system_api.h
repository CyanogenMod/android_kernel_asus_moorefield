#ifndef _HRT_SYSTEM_API_H
#define _HRT_SYSTEM_API_H

#define HRT_PROC_TYPE(proc)            HRTCAT(proc,_type)
#define HRT_PROC_TYPE_PROP(proc, prop) HRTCAT(HRT_PROC_TYPE(proc),prop)
#define HRT_PROC_UNIT_PROP(unit,prop)  HRTCAT(unit, prop)
#define HRT_TYPE_BITS(cell, type)      HRT_PROC_TYPE_PROP(cell, HRTCAT3(_,type,_bits))
/* hrt internal api, used to implement the official hrt api */

/* hive processor support */
#define _hrt_cell_stat_ctrl_slave_port(p)            HRT_PROC_TYPE_PROP(p, _stat_ctrl_slave_port)
#define _hrt_cell_stat_ctrl_base(p)                  HRT_PROC_TYPE_PROP(p, _stat_ctrl_base)
#define _hrt_cell_prog_mem_slave_port(p)             HRT_PROC_TYPE_PROP(p, _prog_mem_slave_port)
#define _hrt_cell_prog_mem_base(p)                   HRT_PROC_TYPE_PROP(p, _prog_mem_base)
#define _hrt_cell_prog_mem_width(p)                  HRT_PROC_TYPE_PROP(p, _prog_mem_width) 
#define _hrt_cell_view_table_slave_port(p)           _hrt_cell_prog_mem_slave_port(p)
#define _hrt_cell_view_table_base(p)                 HRT_PROC_TYPE_PROP(p, _view_table_base)
#define _hrt_cell_default_mem(p)                     HRT_PROC_TYPE_PROP(p, _default_mem)
#define _hrt_cell_num_views(p)                       HRT_PROC_TYPE_PROP(p, _num_views) 
#define _hrt_cell_int_size(p)                        HRT_PROC_TYPE_PROP(p, _int_size)
#define _hrt_cell_start_address_register(p)          HRT_PROC_TYPE_PROP(p, _start_address_register)
#define _hrt_cell_break_address_register(p)          HRT_PROC_TYPE_PROP(p, _break_address_register)
#define _hrt_cell_debug_pc_register(p)               HRT_PROC_TYPE_PROP(p, _debug_pc_register)
#define _hrt_cell_reset_flag_register(p)             HRT_PROC_TYPE_PROP(p, _reset_flag_register)
#define _hrt_cell_reset_flag_bit(p)                  HRT_PROC_TYPE_PROP(p, _reset_flag_bit)
#define _hrt_cell_start_flag_register(p)             HRT_PROC_TYPE_PROP(p, _start_flag_register)
#define _hrt_cell_start_flag_bit(p)                  HRT_PROC_TYPE_PROP(p, _start_flag_bit)
#define _hrt_cell_break_flag_register(p)             HRT_PROC_TYPE_PROP(p, _break_flag_register)
#define _hrt_cell_break_flag_bit(p)                  HRT_PROC_TYPE_PROP(p, _break_flag_bit)
#define _hrt_cell_run_flag_register(p)               HRT_PROC_TYPE_PROP(p, _run_flag_register)
#define _hrt_cell_run_flag_bit(p)                    HRT_PROC_TYPE_PROP(p, _run_flag_bit)
#define _hrt_cell_broken_flag_register(p)            HRT_PROC_TYPE_PROP(p, _broken_flag_register)
#define _hrt_cell_broken_flag_bit(p)                 HRT_PROC_TYPE_PROP(p, _broken_flag_bit)
#define _hrt_cell_ready_flag_register(p)             HRT_PROC_TYPE_PROP(p, _ready_flag_register)
#define _hrt_cell_ready_flag_bit(p)                  HRT_PROC_TYPE_PROP(p, _ready_flag_bit)
#define _hrt_cell_sleeping_flag_register(p)          HRT_PROC_TYPE_PROP(p, _sleeping_flag_register)
#define _hrt_cell_sleeping_flag_bit(p)               HRT_PROC_TYPE_PROP(p, _sleeping_flag_bit)
#define _hrt_cell_stalling_flag_register(p)          HRT_PROC_TYPE_PROP(p, _stalling_flag_register)
#define _hrt_cell_stalling_flag_bit(p)               HRT_PROC_TYPE_PROP(p, _stalling_flag_bit)
#define _hrt_cell_irq_clr_flag_register(p)           HRT_PROC_TYPE_PROP(p, _irq_clr_flag_register)
#define _hrt_cell_irq_clr_flag_bit(p)                HRT_PROC_TYPE_PROP(p, _irq_clr_flag_bit)
#define _hrt_cell_broken_irq_mask_flag_register(p)   HRT_PROC_TYPE_PROP(p, _broken_irq_mask_flag_register)
#define _hrt_cell_broken_irq_mask_flag_bit(p)        HRT_PROC_TYPE_PROP(p, _broken_irq_mask_flag_bit)
#define _hrt_cell_ready_irq_mask_flag_register(p)    HRT_PROC_TYPE_PROP(p, _ready_irq_mask_flag_register)
#define _hrt_cell_ready_irq_mask_flag_bit(p)         HRT_PROC_TYPE_PROP(p, _ready_irq_mask_flag_bit)
#define _hrt_cell_sleeping_irq_mask_flag_register(p) HRT_PROC_TYPE_PROP(p, _sleeping_irq_mask_flag_register)
#define _hrt_cell_sleeping_irq_mask_flag_bit(p)      HRT_PROC_TYPE_PROP(p, _sleeping_irq_mask_flag_bit)
#define _hrt_cell_debug_step_flag_register(p)        HRT_PROC_TYPE_PROP(p, _debug_step_flag_register)
#define _hrt_cell_debug_step_flag_bit(p)             HRT_PROC_TYPE_PROP(p, _debug_step_flag_bit)
#define _hrt_cell_loop_cache_invalidate_flag_register(p)    HRT_PROC_TYPE_PROP(p, _loop_cache_invalidate_flag_register)
#define _hrt_cell_loop_cache_invalidate_flag_bit(p)         HRT_PROC_TYPE_PROP(p, _loop_cache_invalidate_flag_bit)
#define _hrt_cell_pmem_slave_access_flag_register(p)    HRT_PROC_TYPE_PROP(p, _pmem_slave_access_flag_register)
#define _hrt_cell_pmem_slave_access_flag_bit(p)         HRT_PROC_TYPE_PROP(p, _pmem_slave_access_flag_bit)
#define _hrt_cell_icache(p)                          HRT_PROC_TYPE_PROP(p, _icache)
#define _hrt_cell_icache_master_interface(p)         HRT_PROC_TYPE_PROP(p, _icache_master_interface)
#define _hrt_cell_icache_segment_size(p)             HRT_PROC_TYPE_PROP(p, _icache_segment_size)

/* CSim specific */
#define _hrt_cell_program_filename_register(p)          HRT_PROC_TYPE_PROP(p, _program_filename_register)

/* master interface support */
#define _hrt_cell_burst_size_register(p,mt_int)          HRT_PROC_UNIT_PROP(mt_int,_burst_size_register)
#define _hrt_cell_burst_size_lsb(p, mt_int)              HRT_PROC_UNIT_PROP(mt_int,_burst_size_lsb)
#define _hrt_cell_burst_size_num_bits(p,mt_int)          HRT_PROC_UNIT_PROP(mt_int,_burst_size_num_bits)
#define _hrt_cell_burst_timeout_register(p,mt_int)       HRT_PROC_UNIT_PROP(mt_int,_burst_timeout_register)
#define _hrt_cell_burst_timeout_lsb(p,mt_int)            HRT_PROC_UNIT_PROP(mt_int,_burst_timeout_lsb)
#define _hrt_cell_burst_timeout_num_bits(p,mt_int)       HRT_PROC_UNIT_PROP(mt_int,_burst_timeout_num_bits)
#define _hrt_cell_base_address_lowest_register(p,mt_int) HRT_PROC_UNIT_PROP(mt_int,_base_address_lowest_register)

/* msink support */
#define _hrt_cell_msink_stalling_flag_register(p,msink) HRT_PROC_UNIT_PROP(msink, _stalling_flag_register)
#define _hrt_cell_msink_stalling_flag_bit(p,msink)      HRT_PROC_UNIT_PROP(msink, _stalling_flag_bit)

/* arbiter support */
#define _hrt_cell_arbiter_period_register(p,arb) HRT_PROC_UNIT_PROP(arb,_period_register)
#define _hrt_cell_arbiter_period_lsb(p,arb)      HRT_PROC_UNIT_PROP(arb,_period_lsb)
#define _hrt_cell_arbiter_period_num_bits(p,arb) HRT_PROC_UNIT_PROP(arb,_period_num_bits)
#define _hrt_cell_arbiter_num_contenders(p,arb)  HRT_PROC_UNIT_PROP(arb,_num_contenders)
#define _hrt_cell_arbiter_first_contender(p,arb) HRT_PROC_UNIT_PROP(arb,_first_contender)
#define _hrt_cell_arbiter_contender_bandwidth_register(p,cont) HRT_PROC_UNIT_PROP(cont,_bandwidth_register)
#define _hrt_cell_arbiter_contender_bandwidth_lsb(p,cont)      HRT_PROC_UNIT_PROP(cont,_bandwidth_lsb)
#define _hrt_cell_arbiter_contender_bandwidth_num_bits(p,cont) HRT_PROC_UNIT_PROP(cont,_bandwidth_num_bits)

/* legacy functions to use hrt on hardware generated with Adalia hw tools: */
#define _hrt_cell_arbiter_mode_lsb(p)                    HRT_PROC_TYPE_PROP(p, _arbiter_mode_lsb)
#define _hrt_cell_arbiter_mode_register(p)               HRT_PROC_TYPE_PROP(p, _arbiter_mode_register)

#define _hrt_cell_cache_invalidate_flag_register(p,cache)      HRT_PROC_TYPE_PROP(p, HRTCAT3(_,cache,_invalidate_flag_register))
#define _hrt_cell_cache_invalidate_flag_bit(p,cache)           HRT_PROC_TYPE_PROP(p, HRTCAT3(_,cache,_invalidate_flag_bit))
#define _hrt_cell_cache_prefetch_enable_flag_register(p,cache) HRT_PROC_TYPE_PROP(p, HRTCAT3(_,cache,_prefetch_enable_flag_register))
#define _hrt_cell_cache_prefetch_enable_flag_bit(p,cache)      HRT_PROC_TYPE_PROP(p, HRTCAT3(_,cache,_prefetch_enable_flag_bit))

/* system specific part */
#define _hrt_cell_mem_size(p,m)                                 HRT_PROC_UNIT_PROP(m, _size)
#define _hrt_cell_mem_physical_size(p,m)                        HRT_PROC_UNIT_PROP(m, _physical_size)
#define _hrt_cell_slave_port(p,cell_port)                       HRTCAT3(p,cell_port,_bus_slave_port)
#define _hrt_cell_slave_port_address(p,cell_port)               HRTCAT3(p,cell_port,_bus_slave_port_address)
#define _hrt_cell_mem_first_slave_port(p,m)                     HRTCAT(m, _first_slave_port)
#define _hrt_cell_connected_slave_port(p,s,m)                   HRTCAT3(p,s,_connected_cell_port)(m)
#define _hrt_cell_connected_slave_port_address(p,s,m)           HRTCAT3(p,s,_connected_cell_port_address)(m)
#define _hrt_cell_connected_slave_port_message(p,s,m)           HRTCAT3(p,s,_connected_cell_port_message)(m)
#define _hrt_cell_mem_slave_port(p,m)                           _hrt_cell_slave_port(p, _hrt_cell_connected_slave_port(p, _hrt_cell_mem_first_slave_port(p,m), m))
#define _hrt_cell_mem_error_message(p,m)                        _hrt_cell_connected_slave_port_message(p, _hrt_cell_mem_first_slave_port(p,m), m)
#define _hrt_cell_mem_master_to_slave_address(p,m,a)            (_hrt_cell_connected_slave_port_address(p, _hrt_cell_mem_first_slave_port(p,m), m) + (a))
#define hrt_master_to_slave_address(master, slave)              HRTCAT4(_hrt_master_to_slave_address_,master,_to_,slave)

/* system memory part */
#define _hrt_sysmem_size(m)       HRTCAT(m, _size) 

#define _hrt_fifo_adapter_slave_port(fa) HRTCAT(fa, _slave_port)

#define _hrt_unreachable_mem_port                       _hrt_unreachable_cell_port
#define _hrt_unreachable_cell_port_bus_slave_port       _hrt_unreachable_slave_port
#define _hrt_unreachable_slave_port_master_port_address 0xFFFFFFFF
#define _hrt_unreachable_slave_port_error_message       "slave port is not connected to the bus"
#define _hrt_unreachable_slave_port_master_port         _hrt_unreachable_master_port
#define _hrt_master_port_data_width(p)                  HRTCAT(p, _data_width)
#define _hrt_slave_port_master_port_address(p)          HRTCAT(p, _master_port_address)
#define _hrt_slave_port_master_port_id(p)               HRTCAT(p, _master_port_id)
#define _hrt_slave_port_error_message(p)                HRTCAT(p, _error_message)

/* device properties */
#define hrt_device_property(device, property)              HRTCAT4(_hrt_device_,device,_property_,property)

#endif /* _HRT_SYSTEM_API_H */
