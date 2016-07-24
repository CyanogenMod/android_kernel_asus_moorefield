#ifndef _test_infra_spc_main_map_h_
#define _test_infra_spc_main_map_h_

#include "./test_infra_spc_main.blob.h"
#include "./test_infra_spc_main.transfer.h"

#ifndef _hrt_dummy_use_blob_test_infra_spc_main
#define _hrt_dummy_use_blob_test_infra_spc_main()
#endif

#define _hrt_cell_load_program_test_infra_spc_main(proc) _hrt_cell_load_program_embedded(proc, test_infra_spc_main)

/* function vied_nci_eq_set_trace_addr_d: EC */

/* function vied_nci_eq_get_sdp: 22C */

/* function vied_nci_eq_device_flush: 16C */

/* function vied_nci_eq_set_fwtrace_last: 5B */

/* function vied_nci_eq_get_timer_inc: 1C5 */

/* function vied_nci_eq_get_sid: 23C */

/* function vied_nci_eq_enable_wakeup_low: 14C */

/* function vied_nci_eq_set_fwtrace_first: 7B */

/* function vied_nci_eq_device_close: 54 */

/* function test_infra_spc_main_entry: 0 */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_test_infra_spc_main_entry
#error Symbol test_infra_spc_main_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_test_infra_spc_main_entry 0x0
#endif
#define HIVE_ADDR_test_infra_spc_main_test_infra_spc_main_entry 0x0

/* function vied_nci_eq_get_pid: 243 */

/* function vied_nci_eq_get_pidend: 205 */

/* function vied_nci_eq_clear_wakeup_low: 12C */

/* function vied_nci_eq_send: 289 */

/* function vied_nci_eq_set_trace_addr_c: FC */

/* function vied_nci_eq_available: 272 */

/* function vied_nci_eq_recv: 252 */

/* function vied_nci_eq_recv_port_open: 282 */

/* function vied_nci_eq_set_fwtrace_middle: 6B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_wait_count
#define HIVE_MEM_wait_count sp2600_control_dmem
#define HIVE_ADDR_wait_count 0x54
#define HIVE_SIZE_wait_count 4
#else
#error Symbol wait_count occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_test_infra_spc_main_wait_count sp2600_control_dmem
#define HIVE_ADDR_test_infra_spc_main_wait_count 0x54
#define HIVE_SIZE_test_infra_spc_main_wait_count 4

/* function test_infra_spc_main: 9 */

/* function vied_nci_eq_device_open: 17D */

/* function vied_nci_eq_set_trace_pc_timer: CC */

/* function vied_nci_eq_pack: 299 */

/* function event_queue_ip_reg_load: 2D1 */

/* function vied_nci_eq_set_trace_header: BC */

/* function vied_nci_eq_all_available: 262 */

/* function event_queue_op_reg_load: 2C3 */

/* function vied_nci_eq_set_timer_inc: 1B5 */

/* function vied_nci_eq_set_wake_prio: 1D5 */

/* function event_queue_ip_reg_store: 2DA */

/* function vied_nci_eq_get_lost_packets: 9C */

/* function vied_nci_eq_set_trace_addr_b: 10C */

/* function vied_nci_eq_get_properties: 2BC */

/* function vied_nci_eq_set_trace_addr_a: 11C */

/* function vied_nci_eq_send_port_open: 2B2 */

/* function vied_nci_eq_get_wakup_stat_low: 15C */

/* function event_queue_op_reg_store: 2CA */

/* function vied_nci_eq_set_pidend: 1F5 */

/* function vied_nci_eq_reserve: 2A2 */

/* function vied_nci_eq_set_wakeup_low: 13C */

/* function vied_nci_eq_get_msg: 24A */

/* function vied_nci_eq_clear_lost_packets: 8B */

/* function vied_nci_eq_set_trace_mode: AC */

/* function vied_nci_eq_set_sdp: 215 */

/* function vied_nci_eq_get_wakeup_prio: 1E5 */

/* function vied_nci_eq_enable_trace: DC */


#endif /* _test_infra_spc_main_map_h_ */
