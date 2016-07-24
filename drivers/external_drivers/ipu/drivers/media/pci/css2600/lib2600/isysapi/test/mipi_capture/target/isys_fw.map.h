#ifndef _isys_fw_map_h_
#define _isys_fw_map_h_

#include "./isys_fw.blob.h"
#include "./isys_fw.transfer.h"

#ifndef _hrt_dummy_use_blob_isys_fw
#define _hrt_dummy_use_blob_isys_fw()
#endif

#define _hrt_cell_load_program_isys_fw(proc) _hrt_cell_load_program_embedded(proc, isys_fw)

/* function prop_get_terminal_desc_bits: 25A3 */

/* function ia_css_syscom_send_port_transfer: 120 */

/* function ia_css_syscom_open: 1C4 */

/* function isa_adi_configure_awb_ff_ctrl: 2AFF */

/* function vied_nci_eq_set_trace_addr_d: 16A5 */

/* function ia_css_syscom_recv_port_open: 18D */

/* function vied_nci_dma_terminal_set_bank_mode: 1F27 */

/* function prop_get_channel_desc_bits: 241C */

/* function vied_nci_eq_get_sdp: 1832 */

/* function vied_nci_acc_isa_configure_ff_inl_param: 2CAC */

/* function vied_nci_ibufctrl_set_dest_cfg_reg: 1524 */

/* function vied_nci_dma_chan_configure: 19A4 */

/* function prop_get_span_desc_bits: 24D5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_expected_init_mask
#define HIVE_MEM_expected_init_mask sp2600_control_dmem
#define HIVE_ADDR_expected_init_mask 0x1CAC
#define HIVE_SIZE_expected_init_mask 48
#else
#error Symbol expected_init_mask occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_expected_init_mask sp2600_control_dmem
#define HIVE_ADDR_isys_fw_expected_init_mask 0x1CAC
#define HIVE_SIZE_isys_fw_expected_init_mask 48

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_init_mask
#define HIVE_MEM_event_init_mask sp2600_control_dmem
#define HIVE_ADDR_event_init_mask 0x1CDC
#define HIVE_SIZE_event_init_mask 48
#else
#error Symbol event_init_mask occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_event_init_mask sp2600_control_dmem
#define HIVE_ADDR_isys_fw_event_init_mask 0x1CDC
#define HIVE_SIZE_isys_fw_event_init_mask 48

/* function vied_nci_eq_device_flush: 173B */

/* function buffer_store: 1012 */

/* function vied_nci_dma_span_desc_mem_store: 222C */

/* function vied_nci_eq_set_fwtrace_last: 1614 */

/* function vied_nci_eq_get_timer_inc: 17CB */

/* function __errno: 2DE8 */

/* function str2mmio_store_reg_lite: 117C */

/* function vied_nci_eq_get_sid: 1725 */

/* function vied_nci_eq_enable_wakeup_low: 1705 */

/* function istream_configure: 8B1 */

/* function vied_nci_ibufctrl_config_dest: 12C3 */

/* function isa_adi_configure_lsc_ff_params: 2B42 */

/* function vied_nci_eq_set_fwtrace_first: 1634 */

/* function vied_nci_eq_device_close: 160D */

/* function vied_nci_str2mmio_enable_sid: 1097 */

/* function recv_port_open: 108 */

/* function vied_nci_str2mmio_open_dev: 103F */

/* function istream_capture: 7EA */

/* function prop_get_request_desc_bits: 2307 */

/* function vied_nci_ibufctrl_set_frame_check_reg: 1512 */

/* function vied_nci_ibufctrl_config_frame_check: 134D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_frames_done
#define HIVE_MEM_frames_done sp2600_control_dmem
#define HIVE_ADDR_frames_done 0x1C94
#define HIVE_SIZE_frames_done 24
#else
#error Symbol frames_done occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_frames_done sp2600_control_dmem
#define HIVE_ADDR_isys_fw_frames_done 0x1C94
#define HIVE_SIZE_isys_fw_frames_done 24

/* function memcpy: 2E72 */

/* function vied_nci_ibufctrl_config_proc: 1260 */

/* function __divu: 2DF2 */

/* function isa_adi_configure_af_ff_ctrl: 2AF4 */

/* function vied_nci_isldevice_mipi_backend_open_dev: 27CC */

/* function vied_nci_str2mmio_cfg_sid: 10E0 */

/* function ia_css_output_buffer_css_store: FE5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM___dummy
#define HIVE_MEM___dummy sp2600_control_dmem
#define HIVE_MEM___dummy_sp2600_control_dmem
#define HIVE_ADDR___dummy 0x0
#define HIVE_SIZE___dummy 4
#else
#if !defined (HIVE_MEM___dummy_sp2600_control_dmem) || HIVE_ADDR___dummy != 0x0 || HIVE_SIZE___dummy != 4
#error Symbol __dummy occurs in multiple mapfiles differently, please rename or split host code into separate modules
#endif
#endif
#endif
#define HIVE_MEM_isys_fw___dummy sp2600_control_dmem
#define HIVE_ADDR_isys_fw___dummy 0x0
#define HIVE_SIZE_isys_fw___dummy 4

/* function fw_memcpy: 798 */

/* function ia_css_syscom_recv_port_available: 173 */

/* function prop_get_dma_register_id_idx: 26DC */

/* function vied_nci_eq_get_pid: 172C */

/* function vied_nci_eq_get_pidend: 180B */

/* function vied_nci_dma_chan_open: 1ABF */

/* function vied_nci_ibufctrl_setup_ibuf: 11FB */

/* function vied_nci_acc_isa_configure_ff_inl_ctrl: 2CDA */

/* function istream_open: 882 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_istr_ctx
#define HIVE_MEM_istr_ctx sp2600_control_dmem
#define HIVE_ADDR_istr_ctx 0xAE98
#define HIVE_SIZE_istr_ctx 4368
#else
#error Symbol istr_ctx occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_istr_ctx sp2600_control_dmem
#define HIVE_ADDR_isys_fw_istr_ctx 0xAE98
#define HIVE_SIZE_isys_fw_istr_ctx 4368

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sharedmemorysubsystems
#define HIVE_MEM_sharedmemorysubsystems sp2600_control_dmem
#define HIVE_ADDR_sharedmemorysubsystems 0xBFA8
#define HIVE_SIZE_sharedmemorysubsystems 4
#else
#error Symbol sharedmemorysubsystems occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_sharedmemorysubsystems sp2600_control_dmem
#define HIVE_ADDR_isys_fw_sharedmemorysubsystems 0xBFA8
#define HIVE_SIZE_isys_fw_sharedmemorysubsystems 4

/* function vied_nci_eq_clear_wakeup_low: 16E5 */

/* function vied_nci_isldevice_pixel_formatter_flush: 2827 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pin_from_stream_and_sid_plus1
#define HIVE_MEM_pin_from_stream_and_sid_plus1 sp2600_control_dmem
#define HIVE_ADDR_pin_from_stream_and_sid_plus1 0x1D0C
#define HIVE_SIZE_pin_from_stream_and_sid_plus1 384
#else
#error Symbol pin_from_stream_and_sid_plus1 occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_pin_from_stream_and_sid_plus1 sp2600_control_dmem
#define HIVE_ADDR_isys_fw_pin_from_stream_and_sid_plus1 0x1D0C
#define HIVE_SIZE_isys_fw_pin_from_stream_and_sid_plus1 384

/* function isa_adi_configure_input_correction_ff_ctrl: 2C47 */

/* function vied_nci_isldevice_mipi_backend_set_alignment_config: 2775 */

/* function vied_nci_eq_send: 1842 */

/* function vied_nci_isldevice_pixel_formatter_set_param_bank: 27DD */

/* function vied_nci_eq_set_trace_addr_c: 16B5 */

/* function vied_nci_ibufctrl_set_sid_cmd_reg: 1536 */

/* function vied_nci_eq_available: 176C */

/* function ia_css_syscom_get_specific: 1A2 */

/* function vied_nci_dma_channel_set_bank_mode: 2038 */

/* function isa_adi_configure_input_correction_ff_params: 2BE8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_state
#define HIVE_MEM_state sp2600_control_dmem
#define HIVE_ADDR_state 0x28F0
#define HIVE_SIZE_state 24
#else
#error Symbol state occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_state sp2600_control_dmem
#define HIVE_ADDR_isys_fw_state 0x28F0
#define HIVE_SIZE_isys_fw_state 24

/* function vied_nci_acc_isa_configure_ff_gbl_ctrl: 2D1A */

/* function ia_css_shared_buffer_css_load: 1003 */

/* function vied_nci_dma_chan_stop: 1928 */

/* function vied_nci_dma_terminal_desc_reg_store: 1F40 */

/* function vied_nci_eq_recv: 174C */

/* function vied_nci_isldevice_mipi_backend_set_raw18_config: 274B */

/* function isys_fw: 214 */

/* function vied_nci_eq_recv_port_open: 177C */

/* function send_port_available: 5D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ctx
#define HIVE_MEM_ctx sp2600_control_dmem
#define HIVE_ADDR_ctx 0x2908
#define HIVE_SIZE_ctx 4
#else
#error Symbol ctx occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_ctx sp2600_control_dmem
#define HIVE_ADDR_isys_fw_ctx 0x2908
#define HIVE_SIZE_isys_fw_ctx 4

/* function vied_nci_eq_set_fwtrace_middle: 1624 */

/* function vied_nci_dma_unit_set_bank_mode: 1E8F */

/* function vied_nci_isldevice_mipi_backend_set_comp_format_reg0: 2767 */

/* function vied_nci_str2mmio_reg_addr_with_base: 1156 */

/* function vied_nci_acc_isa_configure_ff_lsc_param_dynamic: 2D82 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_token_info
#define HIVE_MEM_token_info sp2600_control_dmem
#define HIVE_ADDR_token_info 0x2910
#define HIVE_SIZE_token_info 920
#else
#error Symbol token_info occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_token_info sp2600_control_dmem
#define HIVE_ADDR_isys_fw_token_info 0x2910
#define HIVE_SIZE_isys_fw_token_info 920

/* function vied_nci_ibufctrl_send_cmd: 15BA */

/* function vied_nci_dma_chan_close: 1920 */

/* function isa_ctrl_trigger_update: 2858 */

/* function vied_nci_dma_get_chan_res: 189B */

/* function vied_nci_eq_device_open: 1783 */

/* function vied_nci_dma_open: 1CCA */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buffer_write_token_flush_stop_ack
#define HIVE_MEM_buffer_write_token_flush_stop_ack sp2600_control_dmem
#define HIVE_ADDR_buffer_write_token_flush_stop_ack 0x2CA8
#define HIVE_SIZE_buffer_write_token_flush_stop_ack 432
#else
#error Symbol buffer_write_token_flush_stop_ack occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_buffer_write_token_flush_stop_ack sp2600_control_dmem
#define HIVE_ADDR_isys_fw_buffer_write_token_flush_stop_ack 0x2CA8
#define HIVE_SIZE_isys_fw_buffer_write_token_flush_stop_ack 432

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dev_prop
#define HIVE_MEM_dev_prop sp2600_control_dmem
#define HIVE_ADDR_dev_prop 0x190
#define HIVE_SIZE_dev_prop 400
#else
#error Symbol dev_prop occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_dev_prop sp2600_control_dmem
#define HIVE_ADDR_isys_fw_dev_prop 0x190
#define HIVE_SIZE_isys_fw_dev_prop 400

/* function vied_nci_ibufctrl_config_ibuf: 122B */

/* function vied_nci_dma_span_set_bank_mode: 21D4 */

/* function vied_nci_isldevice_pixel_formatter_set_int: 2819 */

/* function vied_nci_ibufctrl_config_proc_cmd: 142E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_regmem
#define HIVE_MEM_regmem sp2600_control_dmem
#define HIVE_ADDR_regmem 0x28AC
#define HIVE_SIZE_regmem 64
#else
#error Symbol regmem occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_regmem sp2600_control_dmem
#define HIVE_ADDR_isys_fw_regmem 0x28AC
#define HIVE_SIZE_isys_fw_regmem 64

/* function isa_adi_configure_ae_ff_ctrl: 2AE9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_invalid_pin
#define HIVE_MEM_invalid_pin sp2600_control_dmem
#define HIVE_ADDR_invalid_pin 0x20
#define HIVE_SIZE_invalid_pin 1
#else
#error Symbol invalid_pin occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_invalid_pin sp2600_control_dmem
#define HIVE_ADDR_isys_fw_invalid_pin 0x20
#define HIVE_SIZE_isys_fw_invalid_pin 1

/* function vied_nci_ibufctrl_open_dev: 11EC */

/* function vied_nci_eq_set_trace_pc_timer: 1685 */

/* function vied_nci_eq_pack: 1852 */

/* function vied_nci_ibufctrl_set_feeder_reg: 1500 */

/* function vied_nci_dma_request_desc_run: 21BB */

/* function event_queue_ip_reg_load: 188A */

/* function vied_nci_acc_isa_configure_ff_gbl_params: 2CF2 */

/* function vied_nci_eq_set_trace_header: 1675 */

/* function regmem_load_32: 7 */

/* function ia_css_shared_buffer_css_store: FF4 */

/* function vied_nci_eq_all_available: 175C */

/* function pixel_str2mmio_sid_addr: 11BA */

/* function vied_nci_dma_terminal_desc_mem_store: 1F77 */

/* function isys_fw_entry: 20B */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_isys_fw_entry
#error Symbol isys_fw_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_isys_fw_entry 0x20B
#endif
#define HIVE_ADDR_isys_fw_isys_fw_entry 0x20B

/* function vied_nci_dma_request_desc_reg_store: 219E */

/* function vied_nci_isldevice_mipi_backend_set_raw16_config: 2759 */

/* function istream_sync: 7DA */

/* function ia_css_input_buffer_css_load: FD6 */

/* function vied_nci_ibufctrl_send_init: 159D */

/* function vied_nci_isldevice_pixel_formatter_clr_int: 280B */

/* function ia_css_syscom_recv_port_transfer: 161 */

/* function vied_nci_acc_isa_configure_ff_lsc_ctrl: 2DC3 */

/* function __modu: 2E34 */

/* function regmem_store_32: 0 */

/* function prop_get_dma_group_id_idx: 2638 */

/* function prop_get_unit_desc_bits: 23B2 */

/* function vied_nci_ibufctrl_set_shared_reg: 155A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isa_adi_configure_ff_params
#define HIVE_MEM_isa_adi_configure_ff_params sp2600_control_dmem
#define HIVE_ADDR_isa_adi_configure_ff_params 0x354
#define HIVE_SIZE_isa_adi_configure_ff_params 28
#else
#error Symbol isa_adi_configure_ff_params occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_isa_adi_configure_ff_params sp2600_control_dmem
#define HIVE_ADDR_isys_fw_isa_adi_configure_ff_params 0x354
#define HIVE_SIZE_isys_fw_isa_adi_configure_ff_params 28

/* function ia_css_syscom_send_port_close: 145 */

/* function memset: 2EBD */

/* function event_queue_op_reg_load: 187C */

/* function vied_nci_dma_unit_desc_mem_store: 1EC6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_subsystems
#define HIVE_MEM_subsystems sp2600_control_dmem
#define HIVE_ADDR_subsystems 0xBFAC
#define HIVE_SIZE_subsystems 4
#else
#error Symbol subsystems occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_subsystems sp2600_control_dmem
#define HIVE_ADDR_isys_fw_subsystems 0xBFAC
#define HIVE_SIZE_isys_fw_subsystems 4

/* function send_port_open: 81 */

/* function vied_nci_isldevice_cio2stream_set_port_cfg: 2704 */

/* function vied_nci_dma_specify_prop: 1CF0 */

/* function vied_nci_ibufctrl_reg_addr_with_base: 1424 */

/* function vied_nci_eq_set_timer_inc: 17BB */

/* function vied_nci_eq_set_wake_prio: 17DB */

/* function recv_port_transfer: 99 */

/* function prop_get_dma_bank_id_idx: 2698 */

/* function event_queue_ip_reg_store: 1893 */

/* function vied_nci_dma_chan_start: 199D */

/* function vied_nci_dma_channel_desc_reg_store: 2051 */

/* function mipi_str2mmio_sid_addr: 11B1 */

/* function vied_nci_eq_get_lost_packets: 1655 */

/* function ia_css_syscom_recv_port_close: 186 */

/* function isa_ctrl_init: 2AC5 */

/* function vied_nci_dma_read_request_valid_register: 2184 */

/* function vied_nci_isldevice_pixel_formatter_open_dev: 2847 */

/* function vied_nci_dma_span_desc_reg_store: 21ED */

/* function vied_nci_eq_set_trace_addr_b: 16C5 */

/* function vied_nci_dma_master_desc_reg_store: 1E71 */

/* function recv_port_available: E5 */

/* function vied_nci_dma_configure: 1B63 */

/* function vied_nci_isldevice_mipi_backend_set_force_raw8: 273D */

/* function vied_nci_eq_get_properties: 1875 */

/* function str2mmio_sid_addr: 1193 */

/* function isa_adi_configure_lsc_ff_ctrl: 2B90 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_que
#define HIVE_MEM_que sp2600_control_dmem
#define HIVE_ADDR_que 0x1C60
#define HIVE_SIZE_que 4
#else
#error Symbol que occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_que sp2600_control_dmem
#define HIVE_ADDR_isys_fw_que 0x1C60
#define HIVE_SIZE_isys_fw_que 4

/* function vied_nci_dma_chan_next: 192F */

/* function vied_nci_ibufctrl_set_sid_cfg_reg: 1548 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM__impure_ptr
#define HIVE_MEM__impure_ptr sp2600_control_dmem
#define HIVE_ADDR__impure_ptr 0x1644
#define HIVE_SIZE__impure_ptr 4
#else
#error Symbol _impure_ptr occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw__impure_ptr sp2600_control_dmem
#define HIVE_ADDR_isys_fw__impure_ptr 0x1644
#define HIVE_SIZE_isys_fw__impure_ptr 4

/* function vied_nci_eq_set_trace_addr_a: 16D5 */

/* function vied_nci_isldevice_pixel_formatter_set_cfg_reg: 283A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_nci_isa_acc_dev
#define HIVE_MEM_nci_isa_acc_dev sp2600_control_dmem
#define HIVE_ADDR_nci_isa_acc_dev 0x15D0
#define HIVE_SIZE_nci_isa_acc_dev 112
#else
#error Symbol nci_isa_acc_dev occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_nci_isa_acc_dev sp2600_control_dmem
#define HIVE_ADDR_isys_fw_nci_isa_acc_dev 0x15D0
#define HIVE_SIZE_isys_fw_nci_isa_acc_dev 112

/* function send_response: 6F5 */

/* function str2mmio_load_reg_lite: 116C */

/* function vied_nci_isldevice_mipi_backend_set_irq_clear: 272F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_perform_poll
#define HIVE_MEM_perform_poll sp2600_control_dmem
#define HIVE_ADDR_perform_poll 0x8
#define HIVE_SIZE_perform_poll 24
#else
#error Symbol perform_poll occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_perform_poll sp2600_control_dmem
#define HIVE_ADDR_isys_fw_perform_poll 0x8
#define HIVE_SIZE_isys_fw_perform_poll 24

/* function vied_nci_eq_send_port_open: 186B */

/* function ia_css_syscom_send_port_open: 14C */

/* function __mod: 2E22 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buffer_write_token_pin_ready
#define HIVE_MEM_buffer_write_token_pin_ready sp2600_control_dmem
#define HIVE_ADDR_buffer_write_token_pin_ready 0x2E58
#define HIVE_SIZE_buffer_write_token_pin_ready 31104
#else
#error Symbol buffer_write_token_pin_ready occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_buffer_write_token_pin_ready sp2600_control_dmem
#define HIVE_ADDR_isys_fw_buffer_write_token_pin_ready 0x2E58
#define HIVE_SIZE_isys_fw_buffer_write_token_pin_ready 31104

/* function str2mmio_common_reg_addr: 118C */

/* function vied_nci_dma_global_desc_reg_store: 1E3F */

/* function vied_nci_eq_get_wakup_stat_low: 1715 */

/* function event_queue_op_reg_store: 1883 */

/* function vied_nci_isldevice_cio2stream_open_dev: 271E */

/* function vied_nci_eq_set_pidend: 17FB */

/* function vied_nci_eq_reserve: 185B */

/* function vied_nci_eq_set_wakeup_low: 16F5 */

/* function vied_nci_eq_get_msg: 1733 */

/* function vied_nci_acc_isa_configure_ff_pcln_ctrl: 2D6A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_buffer_write_token_capture_done
#define HIVE_MEM_buffer_write_token_capture_done sp2600_control_dmem
#define HIVE_ADDR_buffer_write_token_capture_done 0xA7D8
#define HIVE_SIZE_buffer_write_token_capture_done 1728
#else
#error Symbol buffer_write_token_capture_done occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_buffer_write_token_capture_done sp2600_control_dmem
#define HIVE_ADDR_isys_fw_buffer_write_token_capture_done 0xA7D8
#define HIVE_SIZE_isys_fw_buffer_write_token_capture_done 1728

/* function vied_nci_eq_clear_lost_packets: 1644 */

/* function vied_nci_isldevice_pixel_formatter_set_int_enable: 27FD */

/* function vied_nci_acc_isa_configure_ff_pcln_param: 2D42 */

/* function poll_event_queue: 729 */

/* function ia_css_syscom_send_port_available: 132 */

/* function vied_nci_ibufctrl_config_feeder: 13B3 */

/* function vied_nci_ibufctrl_config_proc_cfg: 1471 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_cmds_sent
#define HIVE_MEM_cmds_sent sp2600_control_dmem
#define HIVE_ADDR_cmds_sent 0x1C7C
#define HIVE_SIZE_cmds_sent 24
#else
#error Symbol cmds_sent occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_cmds_sent sp2600_control_dmem
#define HIVE_ADDR_isys_fw_cmds_sent 0x1C7C
#define HIVE_SIZE_isys_fw_cmds_sent 24

/* function vied_nci_eq_set_trace_mode: 1665 */

/* function buffer_load: 1028 */

/* function ia_css_syscom_close: 1BD */

/* function isa_adi_configure_gddpc_ff_ctrl: 2B15 */

/* function vied_nci_isldevice_mipi_backend_set_cfg: 2783 */

/* function vied_nci_dma_close: 1CC3 */

/* function vied_nci_dma_channel_desc_mem_store: 207B */

/* function vied_nci_eq_set_sdp: 181B */

/* function vied_nci_str2mmio_disable_sid: 104E */

/* function vied_nci_dma_unit_desc_reg_store: 1EA8 */

/* function poll_msg_queue: 759 */

/* function isa_adi_configure_dwnsclr_ff_ctrl: 2B0A */

/* function vied_nci_eq_get_wakeup_prio: 17EB */

/* function vied_nci_eq_enable_trace: 1695 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isa_adi_configure_ff_ctrl
#define HIVE_MEM_isa_adi_configure_ff_ctrl sp2600_control_dmem
#define HIVE_ADDR_isa_adi_configure_ff_ctrl 0x338
#define HIVE_SIZE_isa_adi_configure_ff_ctrl 28
#else
#error Symbol isa_adi_configure_ff_ctrl occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_isa_adi_configure_ff_ctrl sp2600_control_dmem
#define HIVE_ADDR_isys_fw_isa_adi_configure_ff_ctrl 0x338
#define HIVE_SIZE_isys_fw_isa_adi_configure_ff_ctrl 28

/* function str2mmio_dev_configuration: 11C3 */

/* function ibufctrl_dev_configuration: 156C */

/* function send_port_transfer: 11 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pistr_ctx
#define HIVE_MEM_pistr_ctx sp2600_control_dmem
#define HIVE_ADDR_pistr_ctx 0x1C64
#define HIVE_SIZE_pistr_ctx 24
#else
#error Symbol pistr_ctx occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_pistr_ctx sp2600_control_dmem
#define HIVE_ADDR_isys_fw_pistr_ctx 0x1C64
#define HIVE_SIZE_isys_fw_pistr_ctx 24


#endif /* _isys_fw_map_h_ */
