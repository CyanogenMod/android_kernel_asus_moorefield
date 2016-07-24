#ifndef _devproxy_map_h_
#define _devproxy_map_h_

#include "./devproxy.blob.h"
#include "./devproxy.transfer.h"

#ifndef _hrt_dummy_use_blob_devproxy
#define _hrt_dummy_use_blob_devproxy()
#endif

#define _hrt_cell_load_program_devproxy(proc) _hrt_cell_load_program_embedded(proc, devproxy)

/* function vied_nci_acc_get_ff_params_space: 15C */

/* function vied_nci_acc_queue_process_fragment: 1CB */

/* function devproxy_service_dm_function_ack_handle: C71 */

/* function devproxy_service_ipf_function_done: F10 */

/* function vied_nci_eq_set_trace_addr_d: 1065 */

/* function vied_nci_acc_routing_previous_in: E7 */

/* function vied_nci_eq_get_sdp: 112C */

/* function vied_nci_eq_device_flush: 10A2 */

/* function vied_nci_eq_set_fwtrace_last: 101D */

/* function vied_nci_acc_device_get_ff_params_space_base: 278 */

/* function vied_nci_eq_get_timer_inc: 10EA */

/* function vied_nci_eq_enable_wakeup_low: 1093 */

/* function devproxy_service_ipf_function_ack_handle: EAD */

/* function vied_nci_acc_config_resolution: 59 */

/* function devproxy_service_dm_send_ack: D02 */

/* function devproxy_service_wba_function_init: B64 */

/* function devproxy_service_ipf_function_init: F3B */

/* function vied_nci_eq_set_fwtrace_first: 102B */

/* function vied_nci_eq_device_close: 1016 */

/* function ia_css_tproxy_chan_open: 11EE */

/* function devproxy_service_ccm_function_init: D9E */

/* function devproxy_service_ccm_send_ack: DC1 */

/* function vied_nci_acc_device_get_ack_register: 2A1 */

/* function devproxy_service_gtc_function_ack_handle: DEE */

/* function dummy_simulate_nci_queue_process_fragment: 97F */

/* function vied_nci_eq_get_pidend: 1111 */

/* function devproxy_service_wba_send_ack: B87 */

/* function devproxy_service_dm_function_done: CBE */

/* function vied_nci_acc_load_from_ff: 1A3 */

/* function vied_nci_acc_get_ff_buffer_space: 13B */

/* function dummy_simulate_nci_queue_idle_state: 990 */

/* function vied_nci_eq_clear_wakeup_low: 1085 */

/* function devproxy_entry: 2CF */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_devproxy_entry
#error Symbol devproxy_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_devproxy_entry 0x2CF
#endif
#define HIVE_ADDR_devproxy_devproxy_entry 0x2CF

/* function devproxy_service_dummy_handle_device_event: 7F8 */

/* function devproxy_service_wba_function_next: B15 */

/* function test_service_dummy_send_ack: A9D */

/* function vied_nci_acc_device_set_ack_register: 2AE */

/* function vied_nci_eq_set_trace_addr_c: 106D */

/* function devproxy_test_service_init: A8E */

/* function devproxy_service_ccm_function_done: D7D */

/* function vied_nci_infeeder_stream_run_next: 1234 */

/* function devproxy_service_ipf_function_next: EE2 */

/* function vied_nci_acc_device_get_acb_register: 2B8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM__hrt_host_0_present
#define HIVE_MEM__hrt_host_0_present sp2600_proxy_dmem
#define HIVE_ADDR__hrt_host_0_present 0xBFC
#define HIVE_SIZE__hrt_host_0_present 4
#else
#error Symbol _hrt_host_0_present occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_devproxy__hrt_host_0_present sp2600_proxy_dmem
#define HIVE_ADDR_devproxy__hrt_host_0_present 0xBFC
#define HIVE_SIZE_devproxy__hrt_host_0_present 4

/* function ia_css_tproxy_chan_next: 116A */

/* function devproxy_service_dm_init: D1F */

/* function vied_nci_eq_set_fwtrace_middle: 1024 */

/* function test_service_function_init: A77 */

/* function vied_nci_acc_device_get_ff_buffer_space_base: 26D */

/* function ia_css_tproxy_get_chan_id: 122D */

/* function devproxy_service_gtc_send_ack: E80 */

/* function devproxy_service_gtc_init: E9D */

/* function vied_nci_eq_device_open: 10AA */

/* function devproxy_service_gtc_function_init: E5D */

/* function vied_nci_eq_set_trace_pc_timer: 1055 */

/* function vied_nci_acc_device_get_ff_partition_params_space_base: 262 */

/* function vied_nci_acc_get_part_params: 0 */

/* function devproxy_service_gtc_function_done: E3C */

/* function vied_nci_eq_set_trace_header: 104D */

/* function devproxy_service_dummy_ctrl_next: 8E6 */

/* function vied_nci_acc_device_set_ff_part_register: 220 */

/* function vied_nci_acc_routing_in: FF */

/* function devproxy: 2D8 */

/* function test_service_dummy_dma_copy_data: AD8 */

/* function devproxy_service_dm_function_init: CDF */

/* function devproxy_service_ipf_send_ack: FDC */

/* function devproxy_service_dummy_create: 967 */

/* function __modu: 1285 */

/* function test_service_function_done: A56 */

/* function test_service_function_ack_handle: 9E8 */

/* function devproxy_service_dummy_ctrl_init: 940 */

/* function ia_css_tproxy_chan_nextN: 117A */

/* function vied_nci_acc_queue_idle_state: 1DE */

/* function vied_nci_acc_close: 1EF */

/* function vied_nci_acc_device_set_mux_register: 204 */

/* function ia_css_tproxy_chan_close: 1137 */

/* function vied_nci_acc_routing_next_out: B7 */

/* function vied_nci_acc_device_get_ff_meta_data_register: 22E */

/* function vied_nci_eq_set_timer_inc: 10E2 */

/* function devproxy_service_ccm_function_ack_handle: D2F */

/* function vied_nci_eq_set_wake_prio: 10F4 */

/* function devproxy_service_anr_function_done: C00 */

/* function devproxy_service_dummy_ctrl_done: 891 */

/* function vied_nci_eq_get_lost_packets: 103B */

/* function devproxy_service_dm_function_next: C90 */

/* function devproxy_service_gtc_function_next: E0D */

/* function vied_nci_eq_set_trace_addr_b: 1075 */

/* function vied_nci_acc_device_set_ff_register: 293 */

/* function vied_nci_acc_routing_out: 77 */

/* function devproxy_service_wba_function_done: B43 */

/* function vied_nci_eq_set_trace_addr_a: 107D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_DUMMY_ACK_PROCESS_FRAGMENT
#define HIVE_MEM_DUMMY_ACK_PROCESS_FRAGMENT sp2600_proxy_dmem
#define HIVE_ADDR_DUMMY_ACK_PROCESS_FRAGMENT 0xA58
#define HIVE_SIZE_DUMMY_ACK_PROCESS_FRAGMENT 4
#else
#error Symbol DUMMY_ACK_PROCESS_FRAGMENT occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_devproxy_DUMMY_ACK_PROCESS_FRAGMENT sp2600_proxy_dmem
#define HIVE_ADDR_devproxy_DUMMY_ACK_PROCESS_FRAGMENT 0xA58
#define HIVE_SIZE_devproxy_DUMMY_ACK_PROCESS_FRAGMENT 4

/* function ia_css_tproxy_chan_configure: 11AF */

/* function devproxy_service_ccm_function_next: D4E */

/* function vied_nci_acc_device_set_pif_conv_register: 1FD */

/* function devproxy_service_anr_send_ack: C44 */

/* function vied_nci_eq_get_wakup_stat_low: 109A */

/* function devproxy_service_ipf_init: FF9 */

/* function vied_nci_acc_set_part_params: 2B */

/* function vied_nci_acc_get_ff_partition_params_space: 11A */

/* function vied_nci_acc_device_get_ff_buffer_space_end: 24C */

/* function vied_nci_eq_set_pidend: 1108 */

/* function dummy_simulate_nci_acknowledge: 9A1 */

/* function devproxy_service_anr_init: C61 */

/* function vied_nci_acc_device_get_ff_params_space_end: 257 */

/* function vied_nci_eq_set_wakeup_low: 108C */

/* function vied_nci_acc_device_get_ff_part_register: 210 */

/* function vied_nci_acc_store_to_ff: 178 */

/* function devproxy_service_wba_function_ack_handle: AF6 */

/* function vied_nci_acc_open: 1F6 */

/* function vied_nci_acc_device_get_ff_partition_params_space_end: 23E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_DUMMY_ACK_IDLE_STATE
#define HIVE_MEM_DUMMY_ACK_IDLE_STATE sp2600_proxy_dmem
#define HIVE_ADDR_DUMMY_ACK_IDLE_STATE 0xBF8
#define HIVE_SIZE_DUMMY_ACK_IDLE_STATE 4
#else
#error Symbol DUMMY_ACK_IDLE_STATE occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_devproxy_DUMMY_ACK_IDLE_STATE sp2600_proxy_dmem
#define HIVE_ADDR_devproxy_DUMMY_ACK_IDLE_STATE 0xBF8
#define HIVE_SIZE_devproxy_DUMMY_ACK_IDLE_STATE 4

/* function devproxy_service_anr_function_next: BD3 */

/* function vied_nci_eq_clear_lost_packets: 1032 */

/* function devproxy_service_anr_function_ack_handle: BB4 */

/* function devproxy_service_anr_function_init: C21 */

/* function vied_nci_eq_set_trace_mode: 1045 */

/* function devproxy_service_ccm_init: DDE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_devproxy_base
#define HIVE_MEM_devproxy_base sp2600_proxy_dmem
#define HIVE_MEM_devproxy_base_sp2600_proxy_dmem
#define HIVE_ADDR_devproxy_base 0x124
#define HIVE_SIZE_devproxy_base 1488
#else
#if !defined (HIVE_MEM_devproxy_base_sp2600_proxy_dmem) || HIVE_ADDR_devproxy_base != 0x124 || HIVE_SIZE_devproxy_base != 1488
#error Symbol devproxy_base occurs in multiple mapfiles differently, please rename or split host code into separate modules
#endif
#endif
#endif
#define HIVE_MEM_devproxy_devproxy_base sp2600_proxy_dmem
#define HIVE_ADDR_devproxy_devproxy_base 0x124
#define HIVE_SIZE_devproxy_devproxy_base 1488

/* function vied_nci_acc_device_get_ff_register: 283 */

/* function test_service_dummy_nci_process_fragment: ABA */

/* function vied_nci_eq_set_sdp: 111C */

/* function vied_nci_acc_device_set_acb_register: 2C5 */

/* function vied_nci_eq_get_wakeup_prio: 10FD */

/* function vied_nci_eq_enable_trace: 105D */

/* function devproxy_service_wba_init: BA4 */

/* function test_service_function_next: A05 */


#endif /* _devproxy_map_h_ */
