#ifndef _HRT_STAT_CTRL_H
#define _HRT_STAT_CTRL_H

#include "version.h"
#include "hive_types.h"
#include "memory.h"
#include "bits.h"
#include "system_api.h"

typedef enum hrt_arbiter_modes {
  hrt_core_has_priority   = 0x0,
  hrt_system_has_priority = 0x1,
  hrt_round_robin_arbiter = 0x2
} hive_arbiter_mode;
#define _hrt_cell_arbiter_mode_num_bits(cell) 0x2

#define _hrt_sc_slave(cell) _hrt_cell_slave_port(cell, _hrt_cell_stat_ctrl_slave_port(cell))
#define _hrt_sc_get_reg(cell, reg)      _hrt_slave_port_load_32( _hrt_sc_slave(cell), _hrt_cell_stat_ctrl_base(cell) + 4*(reg))
#define _hrt_sc_set_reg(cell, reg, val) \
  do { \
    _hrt_slave_port_store_32(_hrt_sc_slave(cell), _hrt_cell_stat_ctrl_base(cell) + 4*(reg), val); \
    hrt_sleep(); \
  } while (0)
#define _hrt_sc_set_bit(cell, reg, bit, val) _hrt_sc_set_reg(cell, reg, _hrt_set_bit(_hrt_sc_get_reg(cell, reg), bit, val))
#define _hrt_sc_get_bit(cell, reg, bit) _hrt_get_bit(_hrt_sc_get_reg(cell, reg), bit)
#define _hrt_sc_set_bits(cell, reg, lsb, num, val) \
  _hrt_sc_set_reg(cell, \
                  reg, \
                  _hrt_set_bits( _hrt_sc_get_reg(cell, reg), lsb, num, val) \
                 )

/* Implementation of hrt stat_ctrl interface: */

#if defined(C_RUN) || defined(HRT_UNSCHED) || defined (TM_RUN)
#define hrt_ctl_reset(cell) \
  hrt_error("hrt_ctl_reset is not supported in c-run and unscheduled run\n")
#else
#define hrt_ctl_reset(cell) \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_true); \
  hrt_sleep ();                                                                                          \
  _hrt_sc_set_bit(cell, _hrt_cell_reset_flag_register(cell), _hrt_cell_reset_flag_bit(cell), hive_false)
#endif

#define hrt_ctl_start(cell) \
  _hrt_sc_set_bit(cell, _hrt_cell_start_flag_register(cell), _hrt_cell_start_flag_bit(cell), hive_true)

#define hrt_ctl_break(cell, enable) \
  _hrt_sc_set_bit(cell, _hrt_cell_break_flag_register(cell), _hrt_cell_break_flag_bit(cell), enable)

#define hrt_ctl_run(cell, enable) \
  _hrt_sc_set_bit(cell, _hrt_cell_run_flag_register(cell), _hrt_cell_run_flag_bit(cell), enable)

#define hrt_ctl_is_broken(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_broken_flag_register(cell), _hrt_cell_broken_flag_bit(cell))

#define hrt_ctl_is_ready(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_ready_flag_register(cell), _hrt_cell_ready_flag_bit(cell))

#define hrt_ctl_is_sleeping(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_sleeping_flag_register(cell), _hrt_cell_sleeping_flag_bit(cell))

#define hrt_ctl_is_stalling(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_stalling_flag_register(cell), _hrt_cell_stalling_flag_bit(cell))

#define hrt_ctl_clear_irqs(cell) \
  _hrt_sc_set_bit(cell, _hrt_cell_irq_clr_flag_register(cell), _hrt_cell_irq_clr_flag_bit(cell), hive_true)

#define hrt_ctl_set_irq_broken(cell, enable) \
  _hrt_sc_set_bit(cell, _hrt_cell_broken_irq_mask_flag_register(cell), _hrt_cell_broken_irq_mask_flag_bit(cell), enable)

#define hrt_ctl_get_irq_broken(cell) \
 _hrt_sc_get_bit(cell, _hrt_cell_broken_irq_mask_flag_register(cell), _hrt_cell_broken_irq_mask_flag_bit(cell))

#define hrt_ctl_set_irq_ready(cell, enable) \
  _hrt_sc_set_bit(cell, _hrt_cell_ready_irq_mask_flag_register(cell), _hrt_cell_ready_irq_mask_flag_bit(cell), enable)

#define hrt_ctl_get_irq_ready(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_ready_irq_mask_flag_register(cell), _hrt_cell_ready_irq_mask_flag_bit(cell))

#define hrt_ctl_set_irq_sleeping(cell, enable) \
  _hrt_sc_set_bit(cell, _hrt_cell_sleeping_irq_mask_flag_register(cell), _hrt_cell_sleeping_irq_mask_flag_bit(cell), enable)

#define hrt_ctl_get_irq_sleeping(cell) \
  _hrt_sc_get_bit(cell, _hrt_cell_sleeping_irq_mask_flag_register(cell), _hrt_cell_sleeping_irq_mask_flag_bit(cell))

#define hrt_ctl_set_single_step(cell, enable) \
  _hrt_sc_set_bit(cell, _hrt_cell_debug_step_flag_register(cell), _hrt_cell_debug_step_flag_bit(cell), enable)

#define hrt_ctl_set_start_address(cell, addr) \
  _hrt_sc_set_reg(cell, _hrt_cell_start_address_register(cell), addr)

#define hrt_ctl_set_break_address(cell, addr) \
  _hrt_sc_set_reg(cell, _hrt_cell_break_address_register(cell), addr)

#define hrt_ctl_program_counter(cell) \
  _hrt_sc_get_reg(cell, _hrt_cell_debug_pc_register(cell))

#define hrt_ctl_set_ext_base_address(cell, mt_int, segment, addr) \
  _hrt_sc_set_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment, addr)

#define hrt_ctl_get_ext_base_address(cell, mt_int, segment) \
  _hrt_sc_get_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment)

#define hrt_ctl_set_ext_base_info(cell, mt_int, segment, info) \
  _hrt_sc_set_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment+1, info)

#define hrt_ctl_get_ext_base_info(cell, mt_int, segment) \
  _hrt_sc_get_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment+1)

#define hrt_ctl_set_ext_base_info_override(cell, mt_int, segment, info_override) \
  _hrt_sc_set_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment+2, info_override)

#define hrt_ctl_get_ext_base_info_override(cell, mt_int, segment) \
  _hrt_sc_get_reg(cell, _hrt_cell_base_address_lowest_register(cell, mt_int) + 3*segment+2)

#define hrt_ctl_debug_msink_stall(cell,msink) \
  _hrt_sc_get_bit(cell, _hrt_cell_msink_stalling_flag_register(cell, msink), _hrt_cell_msink_stalling_flag_bit(cell, msink))

#define hrt_ctl_set_arbiter_period(cell, arb, val) \
  _hrt_sc_set_bits(cell, \
                   _hrt_cell_arbiter_period_register(cell, arb), \
                   _hrt_cell_arbiter_period_lsb(cell, arb), \
                   _hrt_cell_arbiter_period_num_bits(cell, arb), \
                   val)

#define hrt_ctl_set_arbiter_contender_bandwidth(cell, cont, val) \
  _hrt_sc_set_bits(cell, \
                   _hrt_cell_arbiter_contender_bandwidth_register(cell, cont), \
                   _hrt_cell_arbiter_contender_bandwidth_lsb(cell, cont), \
                   _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, cont), \
                   val)

#define hrt_ctl_set_arbiter(cell, arbiter, period, bandwidth, length) \
  HRTCAT(_hrt_ctl_set_arbiter_, arbiter)(cell, period, bandwidth, length)


#if defined HRT_CSIM || defined HRT_VP
/* the startus&control should be in special mode to store a program
   char */
#define _hrt_ctl_store_program_char(cell, c) \
{ \
  _hrt_sc_set_reg(cell, _hrt_cell_program_filename_register(cell), (unsigned int)c); \
}

#define _hrt_ctl_enter_special_mode(cell) \
{ \
  unsigned int val = _hrt_sc_get_reg(cell, _hrt_cell_start_flag_register(cell)); \
  val =_hrt_set_bit(val,_hrt_cell_start_flag_bit(cell),hive_true); \
  val =_hrt_set_bit(val,_hrt_cell_run_flag_bit(cell),hive_false); \
  _hrt_sc_set_reg(cell, _hrt_cell_program_filename_register(cell), val); \
}
#endif

/* This function is used to set the arbiter mode on adalia hardware and
   should only be used for that. For newer hardware, use hrt_ctl_set_arbiter) */
#define hrt_ctl_set_arbiter_mode(cell, mode) \
  _hrt_sc_set_bits(cell, \
                   _hrt_cell_arbiter_mode_register(cell), \
                   _hrt_cell_arbiter_mode_lsb(cell), \
                   _hrt_cell_arbiter_mode_num_bits(cell), \
                   mode)
  
#define hrt_ctl_set_burst_length(cell, mt_int, val) \
  _hrt_sc_set_bits(cell, \
                   _hrt_cell_burst_size_register(cell, mt_int), \
                   _hrt_cell_burst_size_lsb(cell, mt_int), \
                   _hrt_cell_burst_size_num_bits(cell, mt_int), \
                   val)
      
#define hrt_ctl_set_burst_interval(cell, mt_int, val) \
  _hrt_sc_set_bits(cell, \
                   _hrt_cell_burst_timeout_register(cell, mt_int), \
                   _hrt_cell_burst_timeout_lsb(cell, mt_int), \
                   _hrt_cell_burst_timeout_num_bits(cell, mt_int), \
                   val)

#define hrt_ctl_invalidate_loop_cache(cell) \
  _hrt_sc_set_bit(cell, _hrt_cell_loop_cache_invalidate_flag_register(cell), _hrt_cell_loop_cache_invalidate_flag_bit(cell), 1)

#define hrt_ctl_invalidate_loop_cache_off(cell) \
  _hrt_sc_set_bit(cell, _hrt_cell_loop_cache_invalidate_flag_register(cell), _hrt_cell_loop_cache_invalidate_flag_bit(cell), 0)

#define hrt_ctl_pmem_slave_access_on(cell) \
  _hrt_sc_set_bit(cell, _hrt_cell_pmem_slave_access_flag_register(cell), _hrt_cell_pmem_slave_access_flag_bit(cell), 1)

#define hrt_ctl_pmem_slave_access_off(cell) \
  _hrt_sc_set_bit(cell, _hrt_cell_pmem_slave_access_flag_register(cell), _hrt_cell_pmem_slave_access_flag_bit(cell), 0)

#define hrt_ctl_invalidate_cache(cell, cache) \
  _hrt_sc_set_bit(cell, _hrt_cell_cache_invalidate_flag_register(cell, cache), _hrt_cell_cache_invalidate_flag_bit(cell, cache), 1)

#define hrt_ctl_set_cache_prefetch_enable(cell, cache, enable) \
  _hrt_sc_set_bit(cell, _hrt_cell_cache_prefetch_enable_flag_register(cell, cache), _hrt_cell_cache_prefetch_enable_flag_bit(cell, cache), enable)

#endif /* _HRT_STAT_CTRL_H */
