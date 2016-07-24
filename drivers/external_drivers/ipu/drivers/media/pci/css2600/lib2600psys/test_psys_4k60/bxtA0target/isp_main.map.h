#ifndef _isp_main_map_h_
#define _isp_main_map_h_

#include "./isp_main.blob.h"
#include "./isp_main.transfer.h"

#ifndef _hrt_dummy_use_blob_isp_main
#define _hrt_dummy_use_blob_isp_main()
#endif

#define _hrt_cell_load_program_isp_main(proc) _hrt_cell_load_program_embedded(proc, isp_main)

/* function vied_nci_acc_get_ff_params_space: 996D */

/* function vied_nci_acc_queue_process_fragment: 9A02 */

/* function vied_nci_eq_set_trace_addr_d: 86AC */

/* function vied_nci_acc_routing_previous_in: 98CE */

/* function isp_main_entry: 8000 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_isp_main_entry
#error Symbol isp_main_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_isp_main_entry 0x8000
#endif
#define HIVE_ADDR_isp_main_isp_main_entry 0x8000

/* function ia_css_devproxy_service_ipf_init_ref: 8C05 */

/* function vied_nci_eq_get_sdp: 8800 */

/* function ia_css_devproxy_service_ccm_route_out: 92F3 */

/* function vied_nci_eq_device_flush: 8720 */

/* function buffer_store: 9D53 */

/* function vied_nci_eq_set_fwtrace_last: 8628 */

/* function vied_nci_acc_device_get_ff_params_space_base: 9B1A */

/* function vied_nci_eq_get_timer_inc: 878F */

/* function ia_css_devproxy_service_ccm_init_ref: 93A4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_addr_frame_buffer
#define HIVE_MEM_addr_frame_buffer isp2600_base_dmem
#define HIVE_ADDR_addr_frame_buffer 0xD24
#define HIVE_SIZE_addr_frame_buffer 4
#else
#error Symbol addr_frame_buffer occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main_addr_frame_buffer isp2600_base_dmem
#define HIVE_ADDR_isp_main_addr_frame_buffer 0xD24
#define HIVE_SIZE_isp_main_addr_frame_buffer 4

/* function ia_css_devproxy_service_ipf_dma_config: 8A99 */

/* function vied_nci_s2v_start: 9744 */

/* function vied_nci_eq_enable_wakeup_low: 8704 */

/* function vied_nci_acc_config_resolution: 9815 */

/* function ia_css_devproxy_service_gtc_init_ref: 958D */

/* function ia_css_devproxy_service_dm_init_ref: 91BB */

/* function vied_nci_eq_set_fwtrace_first: 8644 */

/* function vied_nci_eq_device_close: 861A */

/* function vied_nci_s2v_run: 972B */

/* function ia_css_tproxy_chan_open: 9C99 */

/* function ia_css_devproxy_service_anr_route_out: 8F21 */

/* function ia_css_devproxy_service_dm_route_out: 910A */

/* function __divu: 9D91 */

/* function vied_nci_acc_device_get_ack_register: 9B5F */

/* function ia_css_output_buffer_css_store: 9D0B */

/* function ia_css_devproxy_service_dm_config: 9071 */

/* function vied_nci_eq_get_pidend: 87D2 */

/* function vied_nci_acc_load_from_ff: 99CA */

/* function vied_nci_acc_get_ff_buffer_space: 9942 */

/* function ia_css_devproxy_service_ccm_config: 925A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_addr_isp_params
#define HIVE_MEM_addr_isp_params isp2600_base_dmem
#define HIVE_ADDR_addr_isp_params 0xD28
#define HIVE_SIZE_addr_isp_params 4
#else
#error Symbol addr_isp_params occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main_addr_isp_params isp2600_base_dmem
#define HIVE_ADDR_isp_main_addr_isp_params 0xD28
#define HIVE_SIZE_isp_main_addr_isp_params 4

/* function vied_nci_eq_clear_wakeup_low: 86E8 */

/* function ia_css_devproxy_service_wba_init_ref: 8DE9 */

/* function vied_nci_acc_device_set_ack_register: 9B76 */

/* function vied_nci_eq_set_trace_addr_c: 86BB */

/* function vied_nci_s2v_set_ack_register: 9777 */

/* function vied_nci_acc_device_get_acb_register: 9B8B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM__hrt_host_0_present
#define HIVE_MEM__hrt_host_0_present isp2600_base_dmem
#define HIVE_ADDR__hrt_host_0_present 0xD2C
#define HIVE_SIZE__hrt_host_0_present 4
#else
#error Symbol _hrt_host_0_present occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main__hrt_host_0_present isp2600_base_dmem
#define HIVE_ADDR_isp_main__hrt_host_0_present 0xD2C
#define HIVE_SIZE_isp_main__hrt_host_0_present 4

/* function ia_css_shared_buffer_css_load: 9D3B */

/* function ia_css_tproxy_chan_next: 9BF5 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_s2v_handle
#define HIVE_MEM_s2v_handle isp2600_base_dmem
#define HIVE_ADDR_s2v_handle 0xD30
#define HIVE_SIZE_s2v_handle 20
#else
#error Symbol s2v_handle occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main_s2v_handle isp2600_base_dmem
#define HIVE_ADDR_isp_main_s2v_handle 0xD30
#define HIVE_SIZE_isp_main_s2v_handle 20

/* function vied_nci_eq_set_fwtrace_middle: 8636 */

/* function vied_nci_s2v_config_ack: 962C */

/* function vied_nci_acc_device_get_ff_buffer_space_base: 9B07 */

/* function ia_css_devproxy_service_ipf_route_out: 8B58 */

/* function ia_css_tproxy_get_chan_id: 9CE5 */

/* function vied_nci_eq_device_open: 872E */

/* function ia_css_devproxy_service_wba_route_out: 8D38 */

/* function vied_nci_eq_set_trace_pc_timer: 868E */

/* function vied_nci_acc_device_get_ff_partition_params_space_base: 9AF4 */

/* function vied_nci_acc_get_part_params: 97A1 */

/* function vied_nci_s2v_config: 9659 */

/* function vied_nci_eq_set_trace_header: 867F */

/* function ia_css_shared_buffer_css_store: 9D23 */

/* function isp_main: 800C */

/* function ia_css_devproxy_ctrl_shutdown: 8813 */

/* function vied_nci_s2v_set_register: 978C */

/* function vied_nci_acc_device_set_ff_part_register: 9A88 */

/* function vied_nci_acc_routing_in: 98F0 */

/* function ia_css_input_buffer_css_load: 9CF3 */

/* function ia_css_devproxy_service_anr_config: 8E88 */

/* function ia_css_devproxy_ctrl_startup: 883C */

/* function ia_css_tproxy_chan_nextN: 9C0D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_nof_runs_s2v
#define HIVE_MEM_nof_runs_s2v isp2600_base_dmem
#define HIVE_ADDR_nof_runs_s2v 0xD44
#define HIVE_SIZE_nof_runs_s2v 20
#else
#error Symbol nof_runs_s2v occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main_nof_runs_s2v isp2600_base_dmem
#define HIVE_ADDR_isp_main_nof_runs_s2v 0xD44
#define HIVE_SIZE_isp_main_nof_runs_s2v 20

/* function vied_nci_acc_queue_idle_state: 9A1C */

/* function vied_nci_acc_close: 9A34 */

/* function vied_nci_acc_device_set_mux_register: 9A5C */

/* function ia_css_tproxy_chan_close: 9BB7 */

/* function vied_nci_acc_routing_next_out: 9891 */

/* function vied_nci_acc_device_get_ff_meta_data_register: 9AA0 */

/* function vied_nci_eq_set_timer_inc: 8780 */

/* function vied_nci_eq_set_wake_prio: 879F */

/* function ia_css_devproxy_service_anr_init_ref: 8FD2 */

/* function ia_css_devproxy_service_gtc_config: 9443 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_copy_mode
#define HIVE_MEM_copy_mode isp2600_base_dmem
#define HIVE_ADDR_copy_mode 0xD58
#define HIVE_SIZE_copy_mode 4
#else
#error Symbol copy_mode occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main_copy_mode isp2600_base_dmem
#define HIVE_ADDR_isp_main_copy_mode 0xD58
#define HIVE_SIZE_isp_main_copy_mode 4

/* function vied_nci_eq_get_lost_packets: 8660 */

/* function vied_nci_s2v_close: 975C */

/* function vied_nci_eq_set_trace_addr_b: 86CA */

/* function ia_css_devproxy_test_service_route_out: 8854 */

/* function vied_nci_acc_device_set_ff_register: 9B47 */

/* function vied_nci_acc_routing_out: 983E */

/* function vied_nci_eq_set_trace_addr_a: 86D9 */

/* function ia_css_tproxy_chan_configure: 9C4D */

/* function vied_nci_s2v_open: 9769 */

/* function vied_nci_acc_device_set_pif_conv_register: 9A4F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_vmem
#define HIVE_MEM_vmem isp2600_base_dmem
#define HIVE_ADDR_vmem 0xCAC
#define HIVE_SIZE_vmem 4
#else
#error Symbol vmem occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main_vmem isp2600_base_dmem
#define HIVE_ADDR_isp_main_vmem 0xCAC
#define HIVE_SIZE_isp_main_vmem 4

/* function vied_nci_eq_get_wakup_stat_low: 8712 */

/* function vied_nci_acc_set_part_params: 97DB */

/* function vied_nci_acc_get_ff_partition_params_space: 9917 */

/* function vied_nci_acc_device_get_ff_buffer_space_end: 9ACE */

/* function vied_nci_eq_set_pidend: 87C2 */

/* function vied_nci_acc_device_get_ff_params_space_end: 9AE1 */

/* function vied_nci_eq_set_wakeup_low: 86F6 */

/* function vied_nci_acc_device_get_ff_part_register: 9A6E */

/* function vied_nci_acc_store_to_ff: 9994 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_p_s2v_dataout
#define HIVE_MEM_p_s2v_dataout isp2600_base_dmem
#define HIVE_ADDR_p_s2v_dataout 0xC44
#define HIVE_SIZE_p_s2v_dataout 4
#else
#error Symbol p_s2v_dataout occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isp_main_p_s2v_dataout isp2600_base_dmem
#define HIVE_ADDR_isp_main_p_s2v_dataout 0xC44
#define HIVE_SIZE_isp_main_p_s2v_dataout 4

/* function __mul: 9DE3 */

/* function vied_nci_acc_open: 9A41 */

/* function vied_nci_acc_device_get_ff_partition_params_space_end: 9ABA */

/* function vied_nci_eq_clear_lost_packets: 8652 */

/* function ia_css_devproxy_service_wba_config: 8C9F */

/* function ia_css_devproxy_service_ipf_buffer_config: 89A4 */

/* function vied_nci_eq_set_trace_mode: 8670 */

/* function buffer_load: 9D72 */

/* function ia_css_devproxy_test_service_init_ref: 8905 */

/* function vied_nci_acc_device_get_ff_register: 9B2D */

/* function vied_nci_eq_set_sdp: 87E5 */

/* function vied_nci_acc_device_set_acb_register: 9BA2 */

/* function vied_nci_eq_get_wakeup_prio: 87AF */

/* function vied_nci_eq_enable_trace: 869D */

/* function ia_css_devproxy_service_gtc_route_out: 94DC */


#endif /* _isp_main_map_h_ */
