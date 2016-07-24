#ifndef _isp2600_cell_h
#define _isp2600_cell_h

#include <hrt/api.h>

#define isp2600_prog_mem_slave_port  isp2600_sl_pmem_ip_cell_port
#define isp2600_prog_mem_width       512
#define isp2600_prog_mem_base        0x0
#define isp2600_prog_mem_end         0x1FFFF
#define isp2600_prog_mem_size        0x500
#define isp2600_stat_ctrl_slave_port isp2600_sl_stat_ip_cell_port
#define isp2600_stat_ctrl_base       0x0
#define isp2600_view_table_base      0x0
#define isp2600_num_views            1
#define isp2600_default_mem          isp2600_base_dmem
#define isp2600_int_size             4
#define isp2600_char_bits            8
#define isp2600_uchar_bits           isp2600_char_bits
#define isp2600_short_bits           16
#define isp2600_ushort_bits           isp2600_short_bits
#define isp2600_int_bits             32
#define isp2600_uint_bits            isp2600_int_bits
#define isp2600_long_bits            32
#define isp2600_ulong_bits           isp2600_long_bits
#define isp2600_ptr_bits             32

#define isp2600_start_address_register          0x1
#define isp2600_break_address_register          0x2
#define isp2600_debug_pc_register               0x4C
#define isp2600_reset_flag_register             0x0
#define isp2600_reset_flag_bit                  0x0
#define isp2600_start_flag_register             0x0
#define isp2600_start_flag_bit                  0x1
#define isp2600_break_flag_register             0x0
#define isp2600_break_flag_bit                  0x2
#define isp2600_run_flag_register               0x0
#define isp2600_run_flag_bit                    0x3
#define isp2600_broken_flag_register            0x0
#define isp2600_broken_flag_bit                 0x4
#define isp2600_ready_flag_register             0x0
#define isp2600_ready_flag_bit                  0x5
#define isp2600_sleeping_flag_register          0x0
#define isp2600_sleeping_flag_bit               0x6
#define isp2600_stalling_flag_register          0x0
#define isp2600_stalling_flag_bit               0x7
#define isp2600_irq_clr_flag_register           0x0
#define isp2600_irq_clr_flag_bit                0x8
#define isp2600_broken_irq_mask_flag_register   0x0
#define isp2600_broken_irq_mask_flag_bit        0x9
#define isp2600_ready_irq_mask_flag_register    0x0
#define isp2600_ready_irq_mask_flag_bit         0xA
#define isp2600_sleeping_irq_mask_flag_register 0x0
#define isp2600_sleeping_irq_mask_flag_bit      0xB
#define isp2600_debug_step_flag_register        0x4B
#define isp2600_debug_step_flag_bit             0x0
#define isp2600_loop_cache_invalidate_flag_register    0x4E
#define isp2600_loop_cache_invalidate_flag_bit         0x0
#define isp2600_pmem_slave_access_flag_register 0x4F
#define isp2600_pmem_slave_access_flag_bit      0x0
#define isp2600_program_filename_register       isp2600_start_flag_register
#define isp2600_base_config_mem_iam_op0_stalling_flag_register 0x4D
#define isp2600_base_config_mem_iam_op0_stalling_flag_bit 0x0
#define isp2600_base_config_mem_iam_op1_stalling_flag_register 0x4D
#define isp2600_base_config_mem_iam_op1_stalling_flag_bit 0x1
#define isp2600_base_dmem_loc_mt_am_inst_0_op0_stalling_flag_register 0x4D
#define isp2600_base_dmem_loc_mt_am_inst_0_op0_stalling_flag_bit 0x2
#define isp2600_qmem_loc_mt_am_inst_1_op0_stalling_flag_register 0x4D
#define isp2600_qmem_loc_mt_am_inst_1_op0_stalling_flag_bit 0x3
#define isp2600_cmem_loc_mt_am_inst_2_op0_stalling_flag_register 0x4D
#define isp2600_cmem_loc_mt_am_inst_2_op0_stalling_flag_bit 0x4
#define isp2600_xmem_loc_mt_am_inst_3_op0_stalling_flag_register 0x4D
#define isp2600_xmem_loc_mt_am_inst_3_op0_stalling_flag_bit 0x5
#define isp2600_simd_bamem_loc_mt_am_inst_4_op0_stalling_flag_register 0x4D
#define isp2600_simd_bamem_loc_mt_am_inst_4_op0_stalling_flag_bit 0x6
#define isp2600_simd_vmem_loc_mt_am_inst_5_op0_stalling_flag_register 0x4D
#define isp2600_simd_vmem_loc_mt_am_inst_5_op0_stalling_flag_bit 0x7
#define isp2600_simd_vec_master_loc_mt_am_inst_6_op0_stalling_flag_register 0x4D
#define isp2600_simd_vec_master_loc_mt_am_inst_6_op0_stalling_flag_bit 0x8

#define isp2600_icache base_config_mem_icache
#define isp2600_icache_segment_size 2097152
#define isp2600_base_config_mem_master_base_address_lowest_register 0x4
#define isp2600_base_config_mem_master_burst_size_register          0x46
#define isp2600_base_config_mem_master_burst_size_lsb               0x0
#define isp2600_base_config_mem_master_burst_size_num_bits          0x3
#define isp2600_base_config_mem_master_burst_timeout_register       0x46
#define isp2600_base_config_mem_master_burst_timeout_lsb            0x3
#define isp2600_base_config_mem_master_burst_timeout_num_bits       0x4
#define isp2600_icache_master_interface isp2600_base_config_mem_master
#define isp2600_qmem_isp_std_master_base_address_lowest_register 0x7
#define isp2600_qmem_isp_std_master_burst_size_register          0x0
#define isp2600_qmem_isp_std_master_burst_size_lsb               0x0
#define isp2600_qmem_isp_std_master_burst_size_num_bits          0x0
#define isp2600_qmem_isp_std_master_burst_timeout_register       0x0
#define isp2600_qmem_isp_std_master_burst_timeout_lsb            0x0
#define isp2600_qmem_isp_std_master_burst_timeout_num_bits       0x0
#define isp2600_cmem_isp_std_master_base_address_lowest_register 0xA
#define isp2600_cmem_isp_std_master_burst_size_register          0x0
#define isp2600_cmem_isp_std_master_burst_size_lsb               0x0
#define isp2600_cmem_isp_std_master_burst_size_num_bits          0x0
#define isp2600_cmem_isp_std_master_burst_timeout_register       0x0
#define isp2600_cmem_isp_std_master_burst_timeout_lsb            0x0
#define isp2600_cmem_isp_std_master_burst_timeout_num_bits       0x0
#define isp2600_xmem_isp_std_master_base_address_lowest_register 0x22
#define isp2600_xmem_isp_std_master_burst_size_register          0x0
#define isp2600_xmem_isp_std_master_burst_size_lsb               0x0
#define isp2600_xmem_isp_std_master_burst_size_num_bits          0x0
#define isp2600_xmem_isp_std_master_burst_timeout_register       0x0
#define isp2600_xmem_isp_std_master_burst_timeout_lsb            0x0
#define isp2600_xmem_isp_std_master_burst_timeout_num_bits       0x0
#define isp2600_simd_vec_master_isp_vec_master_base_address_lowest_register 0x2E
#define isp2600_simd_vec_master_isp_vec_master_burst_size_register          0x0
#define isp2600_simd_vec_master_isp_vec_master_burst_size_lsb               0x0
#define isp2600_simd_vec_master_isp_vec_master_burst_size_num_bits          0x0
#define isp2600_simd_vec_master_isp_vec_master_burst_timeout_register       0x0
#define isp2600_simd_vec_master_isp_vec_master_burst_timeout_lsb            0x0
#define isp2600_simd_vec_master_isp_vec_master_burst_timeout_num_bits       0x0

#define isp2600_base_config_mem_Arb_prg_mem_wp_period_register 0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_period_lsb      0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_period_num_bits 0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_source1_bandwidth_register 0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_source1_bandwidth_lsb      0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_source1_bandwidth_num_bits 0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_source0_bandwidth_register 0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_source0_bandwidth_lsb      0x0
#define isp2600_base_config_mem_Arb_prg_mem_wp_source0_bandwidth_num_bits 0x0
#define _hrt_ctl_set_arbiter_isp2600_base_config_mem_Arb_prg_mem_wp(cell, period, bandwidth, length) \
  { \
    hrt_ctl_set_arbiter_period(cell, isp2600_base_config_mem_Arb_prg_mem_wp, period); \
    if (0<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source1, bandwidth[0]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source1, 0);\
    if (1<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source0, bandwidth[1]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source0, 0);\
  }

#define isp2600_base_dmem_Arb_data_mem_wp0_period_register 0x0
#define isp2600_base_dmem_Arb_data_mem_wp0_period_lsb      0xE
#define isp2600_base_dmem_Arb_data_mem_wp0_period_num_bits 0x1
#define isp2600_base_dmem_Arb_data_mem_wp0_source1_bandwidth_register 0x0
#define isp2600_base_dmem_Arb_data_mem_wp0_source1_bandwidth_lsb      0x10
#define isp2600_base_dmem_Arb_data_mem_wp0_source1_bandwidth_num_bits 0x1
#define isp2600_base_dmem_Arb_data_mem_wp0_source0_bandwidth_register 0x0
#define isp2600_base_dmem_Arb_data_mem_wp0_source0_bandwidth_lsb      0xF
#define isp2600_base_dmem_Arb_data_mem_wp0_source0_bandwidth_num_bits 0x1
#define _hrt_ctl_set_arbiter_isp2600_base_dmem_Arb_data_mem_wp0(cell, period, bandwidth, length) \
  { \
    hrt_ctl_set_arbiter_period(cell, isp2600_base_dmem_Arb_data_mem_wp0, period); \
    if (0<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source1, bandwidth[0]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source1, 0);\
    if (1<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source0, bandwidth[1]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source0, 0);\
  }

#define isp2600_simd_vmem_Arb_vec_dmem_wp0_period_register 0x0
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_period_lsb      0x11
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_period_num_bits 0x1
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_source1_bandwidth_register 0x0
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_source1_bandwidth_lsb      0x13
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_source1_bandwidth_num_bits 0x1
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_source0_bandwidth_register 0x0
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_source0_bandwidth_lsb      0x12
#define isp2600_simd_vmem_Arb_vec_dmem_wp0_source0_bandwidth_num_bits 0x1
#define _hrt_ctl_set_arbiter_isp2600_simd_vmem_Arb_vec_dmem_wp0(cell, period, bandwidth, length) \
  { \
    hrt_ctl_set_arbiter_period(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0, period); \
    if (0<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source1, bandwidth[0]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source1, 0);\
    if (1<length) hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source0, bandwidth[1]);\
    else          hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source0, 0);\
  }


#define isp2600_arbiter_mode_lsb 0xE
#define isp2600_arbiter_mode_register 0x0
#define isp2600_base_config_mem_icache_invalidate_flag_register      0x0
#define isp2600_base_config_mem_icache_invalidate_flag_bit           0xC
#define isp2600_base_config_mem_icache_prefetch_enable_flag_register 0x0
#define isp2600_base_config_mem_icache_prefetch_enable_flag_bit      0xD

#define isp2600_base_dmem_size 0x4000
#define isp2600_base_dmem_physical_size 0x4000
#define isp2600_base_dmem_first_slave_port isp2600_sl_dmem_ip_cell_port
#define isp2600_base_dmem_sl_dmem_ip_next_cell_port isp2600_unreachable_cell_port
#define isp2600_sl_dmem_ip_isp2600_base_dmem_address 0x0
#define _hrt_mem_load_8_isp2600_base_dmem(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_uload_8_isp2600_base_dmem(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_load_16_isp2600_base_dmem(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_uload_16_isp2600_base_dmem(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_load_32_isp2600_base_dmem(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_uload_32_isp2600_base_dmem(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_store_8_isp2600_base_dmem(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_store_16_isp2600_base_dmem(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_store_32_isp2600_base_dmem(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_load_isp2600_base_dmem(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_store_isp2600_base_dmem(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define _hrt_mem_set_isp2600_base_dmem(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2600_base_dmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_base_dmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_base_dmem))
#define isp2600_simd_bamem_size 0x800000
#define isp2600_simd_bamem_physical_size 0x7FFFC0
#define isp2600_simd_bamem_first_slave_port isp2600_sl_bmem_ip_cell_port
#define isp2600_simd_bamem_sl_bmem_ip_next_cell_port isp2600_unreachable_cell_port
#define isp2600_sl_bmem_ip_isp2600_simd_bamem_address 0x0
#define _hrt_mem_load_8_isp2600_simd_bamem(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_uload_8_isp2600_simd_bamem(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_load_16_isp2600_simd_bamem(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_uload_16_isp2600_simd_bamem(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_load_32_isp2600_simd_bamem(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_uload_32_isp2600_simd_bamem(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_store_8_isp2600_simd_bamem(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_store_16_isp2600_simd_bamem(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_store_32_isp2600_simd_bamem(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_load_isp2600_simd_bamem(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_store_isp2600_simd_bamem(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define _hrt_mem_set_isp2600_simd_bamem(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_bamem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_bamem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_simd_bamem))
#define isp2600_simd_vmem_size 0x20000
#define isp2600_simd_vmem_physical_size 0x20000
#define isp2600_simd_vmem_first_slave_port isp2600_sl_vmem_ip_cell_port
#define isp2600_simd_vmem_sl_vmem_ip_next_cell_port isp2600_unreachable_cell_port
#define isp2600_sl_vmem_ip_isp2600_simd_vmem_address 0x0
#define _hrt_mem_load_8_isp2600_simd_vmem(cell, addr) \
        _hrt_slave_port_load_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_uload_8_isp2600_simd_vmem(cell, addr) \
        _hrt_slave_port_uload_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_load_16_isp2600_simd_vmem(cell, addr) \
        _hrt_slave_port_load_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_uload_16_isp2600_simd_vmem(cell, addr) \
        _hrt_slave_port_uload_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_load_32_isp2600_simd_vmem(cell, addr) \
        _hrt_slave_port_load_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_uload_32_isp2600_simd_vmem(cell, addr) \
        _hrt_slave_port_uload_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                                _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_store_8_isp2600_simd_vmem(cell, addr, data) \
        _hrt_slave_port_store_8_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_store_16_isp2600_simd_vmem(cell, addr, data) \
        _hrt_slave_port_store_16_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_store_32_isp2600_simd_vmem(cell, addr, data) \
        _hrt_slave_port_store_32_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                               _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), data, \
                               _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_load_isp2600_simd_vmem(cell, addr, data, size) \
        _hrt_slave_port_load_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_store_isp2600_simd_vmem(cell, addr, data, size) \
        _hrt_slave_port_store_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_set_isp2600_simd_vmem(cell, addr, data, size) \
        _hrt_slave_port_set_msg(_hrt_cell_mem_slave_port(cell, isp2600_simd_vmem), \
                             _hrt_cell_mem_master_to_slave_address(cell, isp2600_simd_vmem, (addr)), \
                             data, size, _hrt_cell_mem_error_message(cell, isp2600_simd_vmem))
#define _hrt_mem_load_isp2600_char(cell, mem, addr)   HRTCAT(hrt_mem_load_, isp2600_char_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_uchar(cell, mem, addr)  HRTCAT(hrt_mem_uload_, isp2600_uchar_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_short(cell, mem, addr)  HRTCAT(hrt_mem_load_, isp2600_short_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_ushort(cell, mem, addr) HRTCAT(hrt_mem_uload_, isp2600_ushort_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_int(cell, mem, addr)    HRTCAT(hrt_mem_load_, isp2600_int_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_uint(cell, mem, addr)   HRTCAT(hrt_mem_uload_, isp2600_uint_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_long(cell, mem, addr)   HRTCAT(hrt_mem_load_, isp2600_long_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_ulong(cell, mem, addr)  HRTCAT(hrt_mem_uload_, isp2600_ulong_bits)(cell, mem, addr)
#define _hrt_mem_load_isp2600_ptr(cell, mem, addr)    HRTCAT(hrt_mem_uload_, isp2600_ptr_bits)(cell, mem, addr)
#define _hrtx_memid_load_CASE_isp2600(cell, mem, addr, data, bytes) \
  switch(mem) { \
    case isp2600_base_dmem: \
      _hrt_mem_load_isp2600_base_dmem(cell, addr, data, bytes); \
      break;\
    case isp2600_simd_bamem: \
      _hrt_mem_load_isp2600_simd_bamem(cell, addr, data, bytes); \
      break;\
    case isp2600_simd_vmem: \
      _hrt_mem_load_isp2600_simd_vmem(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_memid_store_CASE_isp2600(cell, mem, addr, data, bytes) \
  switch(mem) {\
    case isp2600_base_dmem: \
      _hrt_mem_store_isp2600_base_dmem(cell, addr, data, bytes); \
      break;\
    case isp2600_simd_bamem: \
      _hrt_mem_store_isp2600_simd_bamem(cell, addr, data, bytes); \
      break;\
    case isp2600_simd_vmem: \
      _hrt_mem_store_isp2600_simd_vmem(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_memid_set_CASE_isp2600(cell, mem, addr, data, bytes) \
  switch(mem) {\
    case isp2600_base_dmem: \
      _hrt_mem_set_isp2600_base_dmem(cell, addr, data, bytes); \
      break;\
    case isp2600_simd_bamem: \
      _hrt_mem_set_isp2600_simd_bamem(cell, addr, data, bytes); \
      break;\
    case isp2600_simd_vmem: \
      _hrt_mem_set_isp2600_simd_vmem(cell, addr, data, bytes); \
      break;\
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_cellid_mem_size_CASE_isp2600(cell, mem) \
  switch(mem) {\
    case isp2600_base_dmem: \
      return _hrt_cell_mem_size(cell, isp2600_base_dmem); \
    case isp2600_simd_bamem: \
      return _hrt_cell_mem_size(cell, isp2600_simd_bamem); \
    case isp2600_simd_vmem: \
      return _hrt_cell_mem_size(cell, isp2600_simd_vmem); \
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_cellid_mem_physical_size_CASE_isp2600(cell, mem) \
  switch(mem) {\
    case isp2600_base_dmem: \
      return _hrt_cell_mem_physical_size(cell, isp2600_base_dmem); \
    case isp2600_simd_bamem: \
      return _hrt_cell_mem_physical_size(cell, isp2600_simd_bamem); \
    case isp2600_simd_vmem: \
      return _hrt_cell_mem_physical_size(cell, isp2600_simd_vmem); \
    default:\
      hrt_error("unknown memory identifier (%d)", mem);\
  }
#define _hrtx_ctlid_set_arbiter_period_CASE_isp2600(cell, arbiter, period) \
  switch(arbiter) {\
    case isp2600_base_config_mem_Arb_prg_mem_wp: \
      hrt_ctl_set_arbiter_period(cell, isp2600_base_config_mem_Arb_prg_mem_wp, period); \
      break;\
    case isp2600_base_dmem_Arb_data_mem_wp0: \
      hrt_ctl_set_arbiter_period(cell, isp2600_base_dmem_Arb_data_mem_wp0, period); \
      break;\
    case isp2600_simd_vmem_Arb_vec_dmem_wp0: \
      hrt_ctl_set_arbiter_period(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0, period); \
      break;\
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_get_arbiter_period_CASE_isp2600(cell, arbiter) \
  switch(arbiter) {\
    case isp2600_base_config_mem_Arb_prg_mem_wp: \
      return hrtx_ctl_get_arbiter_period(cell, isp2600_base_config_mem_Arb_prg_mem_wp); \
    case isp2600_base_dmem_Arb_data_mem_wp0: \
      return hrtx_ctl_get_arbiter_period(cell, isp2600_base_dmem_Arb_data_mem_wp0); \
    case isp2600_simd_vmem_Arb_vec_dmem_wp0: \
      return hrtx_ctl_get_arbiter_period(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0); \
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_get_arbiter_period_num_bits_CASE_isp2600(cell, arbiter) \
  switch(arbiter) {\
    case isp2600_base_config_mem_Arb_prg_mem_wp: \
      return _hrt_cell_arbiter_period_num_bits(cell, isp2600_base_config_mem_Arb_prg_mem_wp); \
    case isp2600_base_dmem_Arb_data_mem_wp0: \
      return _hrt_cell_arbiter_period_num_bits(cell, isp2600_base_dmem_Arb_data_mem_wp0); \
    case isp2600_simd_vmem_Arb_vec_dmem_wp0: \
      return _hrt_cell_arbiter_period_num_bits(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0); \
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_set_arbiter_CASE_isp2600(cell, arbiter, period, bandwidth, length) \
  switch(arbiter) {\
    case isp2600_base_config_mem_Arb_prg_mem_wp: \
      hrt_ctl_set_arbiter(cell, isp2600_base_config_mem_Arb_prg_mem_wp, period, bandwidth, length); \
      break;\
    case isp2600_base_dmem_Arb_data_mem_wp0: \
      hrt_ctl_set_arbiter(cell, isp2600_base_dmem_Arb_data_mem_wp0, period, bandwidth, length); \
      break;\
    case isp2600_simd_vmem_Arb_vec_dmem_wp0: \
      hrt_ctl_set_arbiter(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0, period, bandwidth, length); \
      break;\
    default:\
      hrt_error("unknown arbiter identifier (%d)", arbiter);\
  }
#define _hrtx_ctlid_set_arbiter_contender_bandwidth_CASE_isp2600(cell, contender, bandwidth) \
  switch(contender) {\
    case isp2600_base_config_mem_Arb_prg_mem_wp_source1: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source1, bandwidth); \
      break;\
    case isp2600_base_config_mem_Arb_prg_mem_wp_source0: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source0, bandwidth); \
      break;\
    case isp2600_base_dmem_Arb_data_mem_wp0_source1: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source1, bandwidth); \
      break;\
    case isp2600_base_dmem_Arb_data_mem_wp0_source0: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source0, bandwidth); \
      break;\
    case isp2600_simd_vmem_Arb_vec_dmem_wp0_source1: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source1, bandwidth); \
      break;\
    case isp2600_simd_vmem_Arb_vec_dmem_wp0_source0: \
      hrt_ctl_set_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source0, bandwidth); \
      break;\
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_get_arbiter_contender_bandwidth_CASE_isp2600(cell, contender) \
  switch(contender) {\
    case isp2600_base_config_mem_Arb_prg_mem_wp_source1: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source1); \
    case isp2600_base_config_mem_Arb_prg_mem_wp_source0: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source0); \
    case isp2600_base_dmem_Arb_data_mem_wp0_source1: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source1); \
    case isp2600_base_dmem_Arb_data_mem_wp0_source0: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2600_base_dmem_Arb_data_mem_wp0_source0); \
    case isp2600_simd_vmem_Arb_vec_dmem_wp0_source1: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source1); \
    case isp2600_simd_vmem_Arb_vec_dmem_wp0_source0: \
      return hrtx_ctl_get_arbiter_contender_bandwidth(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source0); \
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_get_arbiter_contender_bandwidth_num_bits_CASE_isp2600(cell, contender) \
  switch(contender) {\
    case isp2600_base_config_mem_Arb_prg_mem_wp_source1: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source1); \
    case isp2600_base_config_mem_Arb_prg_mem_wp_source0: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2600_base_config_mem_Arb_prg_mem_wp_source0); \
    case isp2600_base_dmem_Arb_data_mem_wp0_source1: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2600_base_dmem_Arb_data_mem_wp0_source1); \
    case isp2600_base_dmem_Arb_data_mem_wp0_source0: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2600_base_dmem_Arb_data_mem_wp0_source0); \
    case isp2600_simd_vmem_Arb_vec_dmem_wp0_source1: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source1); \
    case isp2600_simd_vmem_Arb_vec_dmem_wp0_source0: \
      return _hrt_cell_arbiter_contender_bandwidth_num_bits(cell, isp2600_simd_vmem_Arb_vec_dmem_wp0_source0); \
    default:\
      hrt_error("unknown arbiter contender identifier (%d)", contender);\
  }
#define _hrtx_ctlid_set_burst_length_CASE_isp2600(cell, mt_int, length) \
  switch(mt_int) {\
    case isp2600_base_config_mem_master: \
      hrt_ctl_set_burst_length(cell, isp2600_base_config_mem_master, length); \
      break;\
    case isp2600_qmem_isp_std_master: \
      hrt_ctl_set_burst_length(cell, isp2600_qmem_isp_std_master, length); \
      break;\
    case isp2600_cmem_isp_std_master: \
      hrt_ctl_set_burst_length(cell, isp2600_cmem_isp_std_master, length); \
      break;\
    case isp2600_xmem_isp_std_master: \
      hrt_ctl_set_burst_length(cell, isp2600_xmem_isp_std_master, length); \
      break;\
    case isp2600_simd_vec_master_isp_vec_master: \
      hrt_ctl_set_burst_length(cell, isp2600_simd_vec_master_isp_vec_master, length); \
      break;\
    default:\
      length = length; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_burst_length_CASE_isp2600(cell, mt_int) \
  switch(mt_int) {\
    case isp2600_base_config_mem_master: \
      return hrtx_ctl_get_burst_length(cell, isp2600_base_config_mem_master); \
    case isp2600_qmem_isp_std_master: \
      return hrtx_ctl_get_burst_length(cell, isp2600_qmem_isp_std_master); \
    case isp2600_cmem_isp_std_master: \
      return hrtx_ctl_get_burst_length(cell, isp2600_cmem_isp_std_master); \
    case isp2600_xmem_isp_std_master: \
      return hrtx_ctl_get_burst_length(cell, isp2600_xmem_isp_std_master); \
    case isp2600_simd_vec_master_isp_vec_master: \
      return hrtx_ctl_get_burst_length(cell, isp2600_simd_vec_master_isp_vec_master); \
    default:\
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_set_burst_interval_CASE_isp2600(cell, mt_int, interval) \
  switch(mt_int) {\
    case isp2600_base_config_mem_master: \
      hrt_ctl_set_burst_interval(cell, isp2600_base_config_mem_master, interval); \
      break;\
    case isp2600_qmem_isp_std_master: \
      hrt_ctl_set_burst_interval(cell, isp2600_qmem_isp_std_master, interval); \
      break;\
    case isp2600_cmem_isp_std_master: \
      hrt_ctl_set_burst_interval(cell, isp2600_cmem_isp_std_master, interval); \
      break;\
    case isp2600_xmem_isp_std_master: \
      hrt_ctl_set_burst_interval(cell, isp2600_xmem_isp_std_master, interval); \
      break;\
    case isp2600_simd_vec_master_isp_vec_master: \
      hrt_ctl_set_burst_interval(cell, isp2600_simd_vec_master_isp_vec_master, interval); \
      break;\
    default:\
      interval = interval; \
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_burst_interval_CASE_isp2600(cell, mt_int) \
  switch(mt_int) {\
    case isp2600_base_config_mem_master: \
      return hrtx_ctl_get_burst_interval(cell, isp2600_base_config_mem_master); \
    case isp2600_qmem_isp_std_master: \
      return hrtx_ctl_get_burst_interval(cell, isp2600_qmem_isp_std_master); \
    case isp2600_cmem_isp_std_master: \
      return hrtx_ctl_get_burst_interval(cell, isp2600_cmem_isp_std_master); \
    case isp2600_xmem_isp_std_master: \
      return hrtx_ctl_get_burst_interval(cell, isp2600_xmem_isp_std_master); \
    case isp2600_simd_vec_master_isp_vec_master: \
      return hrtx_ctl_get_burst_interval(cell, isp2600_simd_vec_master_isp_vec_master); \
    default:\
      hrt_error("unknown master interface identifier (%d)", mt_int);\
  }
#define _hrtx_ctlid_get_ext_base_address_CASE_isp2600(cell, mt_int, segment) \
  switch(mt_int) { \
    case isp2600_base_config_mem_master: \
      return hrtx_ctl_get_ext_base_address(cell, isp2600_base_config_mem_master, segment); \
    case isp2600_qmem_isp_std_master: \
      return hrtx_ctl_get_ext_base_address(cell, isp2600_qmem_isp_std_master, segment); \
    case isp2600_cmem_isp_std_master: \
      return hrtx_ctl_get_ext_base_address(cell, isp2600_cmem_isp_std_master, segment); \
    case isp2600_xmem_isp_std_master: \
      return hrtx_ctl_get_ext_base_address(cell, isp2600_xmem_isp_std_master, segment); \
    case isp2600_simd_vec_master_isp_vec_master: \
      return hrtx_ctl_get_ext_base_address(cell, isp2600_simd_vec_master_isp_vec_master, segment); \
    default: \
      segment = segment; \
      hrt_error("unknown master interface identifier (%d)", mt_int); \
  }
#define _hrtx_ctlid_set_ext_base_address_CASE_isp2600(cell, mt_int, segment, addr) \
  switch(mt_int) { \
    case isp2600_base_config_mem_master: \
      hrt_ctl_set_ext_base_address(cell, isp2600_base_config_mem_master, segment*3, addr); \
      break;\
    case isp2600_qmem_isp_std_master: \
      hrt_ctl_set_ext_base_address(cell, isp2600_qmem_isp_std_master, segment*3, addr); \
      break;\
    case isp2600_cmem_isp_std_master: \
      hrt_ctl_set_ext_base_address(cell, isp2600_cmem_isp_std_master, segment*3, addr); \
      break;\
    case isp2600_xmem_isp_std_master: \
      hrt_ctl_set_ext_base_address(cell, isp2600_xmem_isp_std_master, segment*3, addr); \
      break;\
    case isp2600_simd_vec_master_isp_vec_master: \
      hrt_ctl_set_ext_base_address(cell, isp2600_simd_vec_master_isp_vec_master, segment*3, addr); \
      break;\
    default: \
      segment = segment; \
      addr = addr; \
      hrt_error("unknown master interface identifier (%d)", mt_int); \
  }

#define _jtag_isp2600_base_config_mem_master_width 512
#define _jtag_isp2600_base_config_mem_master_capacity 32768
#define _jtag_isp2600_base_config_mem_master_type 1
#define _jtag_isp2600_base_dmem_data_mem_width 64
#define _jtag_isp2600_base_dmem_data_mem_capacity 2048
#define _jtag_isp2600_base_dmem_data_mem_type 1
#define _jtag_isp2600_qmem_isp_std_master_width 32
#define _jtag_isp2600_qmem_isp_std_master_capacity 1073741824
#define _jtag_isp2600_qmem_isp_std_master_type 1
#define _jtag_isp2600_cmem_isp_std_master_width 32
#define _jtag_isp2600_cmem_isp_std_master_capacity 1073741824
#define _jtag_isp2600_cmem_isp_std_master_type 1
#define _jtag_isp2600_xmem_isp_std_master_width 32
#define _jtag_isp2600_xmem_isp_std_master_capacity 1073741824
#define _jtag_isp2600_xmem_isp_std_master_type 1
#define _jtag_isp2600_simd_bamem_bamem_width 512
#define _jtag_isp2600_simd_bamem_bamem_capacity 131071
#define _jtag_isp2600_simd_bamem_bamem_type 1
#define _jtag_isp2600_simd_vmem_vec_dmem_width 512
#define _jtag_isp2600_simd_vmem_vec_dmem_capacity 2048
#define _jtag_isp2600_simd_vmem_vec_dmem_type 1
#define _jtag_isp2600_simd_vec_master_isp_vec_master_width 512
#define _jtag_isp2600_simd_vec_master_isp_vec_master_capacity 67108864
#define _jtag_isp2600_simd_vec_master_isp_vec_master_type 1
#define _jtag_isp2600_rf_g_width 1
#define _jtag_isp2600_rf_g_capacity 16
#define _jtag_isp2600_rf_g_type 2
#define _jtag_isp2600_base_rf1_width 32
#define _jtag_isp2600_base_rf1_capacity 64
#define _jtag_isp2600_base_rf1_type 2
#define _jtag_isp2600_base_rf2_width 32
#define _jtag_isp2600_base_rf2_capacity 16
#define _jtag_isp2600_base_rf2_type 2
#define _jtag_isp2600_simd_rf1_width 32
#define _jtag_isp2600_simd_rf1_capacity 16
#define _jtag_isp2600_simd_rf1_type 2
#define _jtag_isp2600_simd_rf2_width 32
#define _jtag_isp2600_simd_rf2_capacity 16
#define _jtag_isp2600_simd_rf2_type 2
#define _jtag_isp2600_simd_rf3_width 32
#define _jtag_isp2600_simd_rf3_capacity 32
#define _jtag_isp2600_simd_rf3_type 2
#define _jtag_isp2600_simd_rf4_width 32
#define _jtag_isp2600_simd_rf4_capacity 16
#define _jtag_isp2600_simd_rf4_type 2
#define _jtag_isp2600_simd_rf5_width 32
#define _jtag_isp2600_simd_rf5_capacity 16
#define _jtag_isp2600_simd_rf5_type 2
#define _jtag_isp2600_simd_rf6_width 32
#define _jtag_isp2600_simd_rf6_capacity 16
#define _jtag_isp2600_simd_rf6_type 2
#define _jtag_isp2600_simd_rf7_width 32
#define _jtag_isp2600_simd_rf7_capacity 16
#define _jtag_isp2600_simd_rf7_type 2
#define _jtag_isp2600_simd_vrf1_width 512
#define _jtag_isp2600_simd_vrf1_capacity 24
#define _jtag_isp2600_simd_vrf1_type 2
#define _jtag_isp2600_simd_vrf2_width 512
#define _jtag_isp2600_simd_vrf2_capacity 24
#define _jtag_isp2600_simd_vrf2_type 2
#define _jtag_isp2600_simd_vrf3_width 512
#define _jtag_isp2600_simd_vrf3_capacity 24
#define _jtag_isp2600_simd_vrf3_type 2
#define _jtag_isp2600_simd_vrf4_width 512
#define _jtag_isp2600_simd_vrf4_capacity 24
#define _jtag_isp2600_simd_vrf4_type 2
#define _jtag_isp2600_simd_vrf5_width 512
#define _jtag_isp2600_simd_vrf5_capacity 24
#define _jtag_isp2600_simd_vrf5_type 2
#define _jtag_isp2600_simd_vrf6_width 512
#define _jtag_isp2600_simd_vrf6_capacity 24
#define _jtag_isp2600_simd_vrf6_type 2
#define _jtag_isp2600_simd_vrf7_width 512
#define _jtag_isp2600_simd_vrf7_capacity 24
#define _jtag_isp2600_simd_vrf7_type 2
#define _jtag_isp2600_simd_srf2_width 128
#define _jtag_isp2600_simd_srf2_capacity 16
#define _jtag_isp2600_simd_srf2_type 2
#define _jtag_isp2600_simd_srf3_width 128
#define _jtag_isp2600_simd_srf3_capacity 16
#define _jtag_isp2600_simd_srf3_type 2
#define _jtag_isp2600_simd_srf4_width 128
#define _jtag_isp2600_simd_srf4_capacity 16
#define _jtag_isp2600_simd_srf4_type 2
#define _jtag_isp2600_PC_width 16
#define _jtag_isp2600_PC_capacity 1
#define _jtag_isp2600_PC_type 2
#define _jtag_isp2600_SR_width 9
#define _jtag_isp2600_SR_capacity 1
#define _jtag_isp2600_SR_type 2

#endif /* _isp2600_cell_h */
