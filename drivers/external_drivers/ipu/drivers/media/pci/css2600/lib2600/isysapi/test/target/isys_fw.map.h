#ifndef _isys_fw_map_h_
#define _isys_fw_map_h_

#include "./isys_fw.blob.h"
#include "./isys_fw.transfer.h"

#ifndef _hrt_dummy_use_blob_isys_fw
#define _hrt_dummy_use_blob_isys_fw()
#endif

#define _hrt_cell_load_program_isys_fw(proc) _hrt_cell_load_program_embedded(proc, isys_fw)

/* function ia_css_syscom_send_port_transfer: 120 */

/* function ia_css_syscom_open: 1C4 */

/* function ia_css_syscom_recv_port_open: 18D */

/* function buffer_store: 71E */

/* function istream_configure: 6BC */

/* function recv_port_open: 108 */

/* function istream_capture: 694 */

/* function memcpy: 74B */

/* function ia_css_output_buffer_css_store: 6F1 */

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

/* function fw_memcpy: 642 */

/* function ia_css_syscom_recv_port_available: 173 */

/* function istream_open: 6AC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_istr_ctx
#define HIVE_MEM_istr_ctx sp2600_control_dmem
#define HIVE_ADDR_istr_ctx 0x720
#define HIVE_SIZE_istr_ctx 4368
#else
#error Symbol istr_ctx occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_istr_ctx sp2600_control_dmem
#define HIVE_ADDR_isys_fw_istr_ctx 0x720
#define HIVE_SIZE_isys_fw_istr_ctx 4368

/* function ia_css_syscom_get_specific: 1A2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_state
#define HIVE_MEM_state sp2600_control_dmem
#define HIVE_ADDR_state 0x368
#define HIVE_SIZE_state 24
#else
#error Symbol state occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_state sp2600_control_dmem
#define HIVE_ADDR_isys_fw_state 0x368
#define HIVE_SIZE_isys_fw_state 24

/* function ia_css_shared_buffer_css_load: 70F */

/* function isys_fw: 214 */

/* function send_port_available: 5D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ctx
#define HIVE_MEM_ctx sp2600_control_dmem
#define HIVE_ADDR_ctx 0x380
#define HIVE_SIZE_ctx 4
#else
#error Symbol ctx occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_ctx sp2600_control_dmem
#define HIVE_ADDR_isys_fw_ctx 0x380
#define HIVE_SIZE_isys_fw_ctx 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_token_info
#define HIVE_MEM_token_info sp2600_control_dmem
#define HIVE_ADDR_token_info 0x388
#define HIVE_SIZE_token_info 920
#else
#error Symbol token_info occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_token_info sp2600_control_dmem
#define HIVE_ADDR_isys_fw_token_info 0x388
#define HIVE_SIZE_isys_fw_token_info 920

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_regmem
#define HIVE_MEM_regmem sp2600_control_dmem
#define HIVE_ADDR_regmem 0x328
#define HIVE_SIZE_regmem 64
#else
#error Symbol regmem occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_regmem sp2600_control_dmem
#define HIVE_ADDR_isys_fw_regmem 0x328
#define HIVE_SIZE_isys_fw_regmem 64

/* function regmem_load_32: 7 */

/* function ia_css_shared_buffer_css_store: 700 */

/* function isys_fw_entry: 20B */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_isys_fw_entry
#error Symbol isys_fw_entry occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#define HIVE_ADDR_isys_fw_entry 0x20B
#endif
#define HIVE_ADDR_isys_fw_isys_fw_entry 0x20B

/* function istream_sync: 684 */

/* function ia_css_input_buffer_css_load: 6E2 */

/* function ia_css_syscom_recv_port_transfer: 161 */

/* function regmem_store_32: 0 */

/* function ia_css_syscom_send_port_close: 145 */

/* function memset: 796 */

/* function send_port_open: 81 */

/* function recv_port_transfer: 99 */

/* function ia_css_syscom_recv_port_close: 186 */

/* function recv_port_available: E5 */

/* function send_response: 5C4 */

/* function ia_css_syscom_send_port_open: 14C */

/* function poll_event_queue: 5F8 */

/* function ia_css_syscom_send_port_available: 132 */

/* function buffer_load: 734 */

/* function ia_css_syscom_close: 1BD */

/* function poll_msg_queue: 603 */

/* function send_port_transfer: 11 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pistr_ctx
#define HIVE_MEM_pistr_ctx sp2600_control_dmem
#define HIVE_ADDR_pistr_ctx 0x310
#define HIVE_SIZE_pistr_ctx 24
#else
#error Symbol pistr_ctx occurs in multiple mapfiles, please rename or split host code into separate modules
#endif
#endif
#define HIVE_MEM_isys_fw_pistr_ctx sp2600_control_dmem
#define HIVE_ADDR_isys_fw_pistr_ctx 0x310
#define HIVE_SIZE_isys_fw_pistr_ctx 24


#endif /* _isys_fw_map_h_ */
