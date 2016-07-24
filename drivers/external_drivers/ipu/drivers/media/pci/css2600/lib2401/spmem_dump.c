/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef _sp_map_h_
#define _sp_map_h_


#ifndef _hrt_dummy_use_blob_sp
#define _hrt_dummy_use_blob_sp()
#endif

#define _hrt_cell_load_program_sp(proc) _hrt_cell_load_program_embedded(proc, sp)

/* function longjmp: 4357 */

/* function ia_css_dmaproxy_sp_set_addr_B: 1BD8 */

/* function debug_buffer_set_ddr_addr: EE */

/* function ia_css_queue_get_size: 3368 */

/* function ia_css_queue_load: 3A28 */

/* function setjmp: 4360 */

/* function __dmaproxy_sp_read_write_text: 1CF0 */

/* function ia_css_dmaproxy_sp_wait_for_ack: 4763 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_N_CSI_RX_BE_SID_WIDTH
#define HIVE_MEM_N_CSI_RX_BE_SID_WIDTH scalar_processor_2400_dmem
#define HIVE_ADDR_N_CSI_RX_BE_SID_WIDTH 0x1A4
#define HIVE_SIZE_N_CSI_RX_BE_SID_WIDTH 12
#else
#endif
#endif
#define HIVE_MEM_sp_N_CSI_RX_BE_SID_WIDTH scalar_processor_2400_dmem
#define HIVE_ADDR_sp_N_CSI_RX_BE_SID_WIDTH 0x1A4
#define HIVE_SIZE_sp_N_CSI_RX_BE_SID_WIDTH 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_stage
#define HIVE_MEM_isp_stage scalar_processor_2400_dmem
#define HIVE_ADDR_isp_stage 0x5EB8
#define HIVE_SIZE_isp_stage 824
#else
#endif
#endif
#define HIVE_MEM_sp_isp_stage scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isp_stage 0x5EB8
#define HIVE_SIZE_sp_isp_stage 824

/* function pocapp_register_isr: 683 */

/* function ia_css_queue_item_store: 36FE */

/* function input_system_reset: 18D7 */

/* function sp_start_isp: 419 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_binary_group
#define HIVE_MEM_sp_binary_group scalar_processor_2400_dmem
#define HIVE_ADDR_sp_binary_group 0x6298
#define HIVE_SIZE_sp_binary_group 32
#else
#endif
#endif
#define HIVE_MEM_sp_sp_binary_group scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_binary_group 0x6298
#define HIVE_SIZE_sp_sp_binary_group 32

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sw_state
#define HIVE_MEM_sp_sw_state scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sw_state 0x6538
#define HIVE_SIZE_sp_sw_state 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sw_state scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_sw_state 0x6538
#define HIVE_SIZE_sp_sp_sw_state 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_psyspoc_host2sp_psys_cmd_queue_handle
#define HIVE_MEM_psyspoc_host2sp_psys_cmd_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_psyspoc_host2sp_psys_cmd_queue_handle 0x5550
#define HIVE_SIZE_psyspoc_host2sp_psys_cmd_queue_handle 12
#else
#endif
#endif
#define HIVE_MEM_sp_psyspoc_host2sp_psys_cmd_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_psyspoc_host2sp_psys_cmd_queue_handle 0x5550
#define HIVE_SIZE_sp_psyspoc_host2sp_psys_cmd_queue_handle 12

/* function ia_css_thread_sp_main: 4055 */

/* function ia_css_ispctrl_sp_init_internal_buffers: 1F79 */

/* function pixelgen_unit_test: 1533 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_psys_cmd
#define HIVE_MEM_psys_cmd scalar_processor_2400_dmem
#define HIVE_ADDR_psys_cmd 0x56A0
#define HIVE_SIZE_psys_cmd 1696
#else
#endif
#endif
#define HIVE_MEM_sp_psys_cmd scalar_processor_2400_dmem
#define HIVE_ADDR_sp_psys_cmd 0x56A0
#define HIVE_SIZE_sp_psys_cmd 1696

/* function ibuf_ctrl_sync: 1304 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_stop_copy_preview
#define HIVE_MEM_sp_stop_copy_preview scalar_processor_2400_dmem
#define HIVE_ADDR_sp_stop_copy_preview 0x2068
#define HIVE_SIZE_sp_stop_copy_preview 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_stop_copy_preview scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_stop_copy_preview 0x2068
#define HIVE_SIZE_sp_sp_stop_copy_preview 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sem_for_sp2host_event_queue
#define HIVE_MEM_pocapp_sem_for_sp2host_event_queue scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sem_for_sp2host_event_queue 0x2054
#define HIVE_SIZE_pocapp_sem_for_sp2host_event_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sem_for_sp2host_event_queue scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sem_for_sp2host_event_queue 0x2054
#define HIVE_SIZE_sp_pocapp_sem_for_sp2host_event_queue 20

/* function ia_css_queue_store: 38A1 */

/* function ia_css_ispctrl_sp_end_binary: 1D7C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sp_threads_stack_size
#define HIVE_MEM_pocapp_sp_threads_stack_size scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sp_threads_stack_size 0xE0
#define HIVE_SIZE_pocapp_sp_threads_stack_size 12
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sp_threads_stack_size scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sp_threads_stack_size 0xE0
#define HIVE_SIZE_sp_pocapp_sp_threads_stack_size 12

/* function pocapp_post_signalling_event: 698 */

/* function pixelgen_tpg_run: 15E9 */

/* function ia_css_isys_stream_find_stop_cmd_in_queue: C3C */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_is_pending_mask
#define HIVE_MEM_event_is_pending_mask scalar_processor_2400_dmem
#define HIVE_ADDR_event_is_pending_mask 0x5C
#define HIVE_SIZE_event_is_pending_mask 44
#else
#endif
#endif
#define HIVE_MEM_sp_event_is_pending_mask scalar_processor_2400_dmem
#define HIVE_ADDR_sp_event_is_pending_mask 0x5C
#define HIVE_SIZE_sp_event_is_pending_mask 44

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_com
#define HIVE_MEM_host_sp_com scalar_processor_2400_dmem
#define HIVE_ADDR_host_sp_com 0x206C
#define HIVE_SIZE_host_sp_com 220
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_com scalar_processor_2400_dmem
#define HIVE_ADDR_sp_host_sp_com 0x206C
#define HIVE_SIZE_sp_host_sp_com 220

/* function ia_css_queue_get_free_space: 34C0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_init_dmem_data
#define HIVE_MEM_sp_init_dmem_data scalar_processor_2400_dmem
#define HIVE_ADDR_sp_init_dmem_data 0x653C
#define HIVE_SIZE_sp_init_dmem_data 24
#else
#endif
#endif
#define HIVE_MEM_sp_sp_init_dmem_data scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_init_dmem_data 0x653C
#define HIVE_SIZE_sp_sp_init_dmem_data 24

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sp_threads_stack
#define HIVE_MEM_pocapp_sp_threads_stack scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sp_threads_stack 0xD4
#define HIVE_SIZE_pocapp_sp_threads_stack 12
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sp_threads_stack scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sp_threads_stack 0xD4
#define HIVE_SIZE_sp_pocapp_sp_threads_stack 12

/* function ia_css_isys_stream_dequeue_message: CE5 */

/* function ia_css_psyspoc_execute_stage: E95 */

/* function is_isp_debug_buffer_full: 3B2 */

/* function ia_css_dmaproxy_sp_configure_channel_from_info: 1B55 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_N_IBUF_CTRL_PROCS
#define HIVE_MEM_N_IBUF_CTRL_PROCS scalar_processor_2400_dmem
#define HIVE_ADDR_N_IBUF_CTRL_PROCS 0x1D8
#define HIVE_SIZE_N_IBUF_CTRL_PROCS 12
#else
#endif
#endif
#define HIVE_MEM_sp_N_IBUF_CTRL_PROCS scalar_processor_2400_dmem
#define HIVE_ADDR_sp_N_IBUF_CTRL_PROCS 0x1D8
#define HIVE_SIZE_sp_N_IBUF_CTRL_PROCS 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_per_frame_data
#define HIVE_MEM_sp_per_frame_data scalar_processor_2400_dmem
#define HIVE_ADDR_sp_per_frame_data 0x2148
#define HIVE_SIZE_sp_per_frame_data 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_per_frame_data scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_per_frame_data 0x2148
#define HIVE_SIZE_sp_sp_per_frame_data 4

/* function ia_css_isys_stream_set_state: D97 */

/* function pocapp_generate_sw_interrupt: 6ED */

/* function memcpy: 4400 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_N_ISYS2401_DMA_CHANNEL_PROCS
#define HIVE_MEM_N_ISYS2401_DMA_CHANNEL_PROCS scalar_processor_2400_dmem
#define HIVE_ADDR_N_ISYS2401_DMA_CHANNEL_PROCS 0x1FC
#define HIVE_SIZE_N_ISYS2401_DMA_CHANNEL_PROCS 4
#else
#endif
#endif
#define HIVE_MEM_sp_N_ISYS2401_DMA_CHANNEL_PROCS scalar_processor_2400_dmem
#define HIVE_ADDR_sp_N_ISYS2401_DMA_CHANNEL_PROCS 0x1FC
#define HIVE_SIZE_sp_N_ISYS2401_DMA_CHANNEL_PROCS 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GP_DEVICE_BASE
#define HIVE_MEM_GP_DEVICE_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_GP_DEVICE_BASE 0x324
#define HIVE_SIZE_GP_DEVICE_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GP_DEVICE_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_GP_DEVICE_BASE 0x324
#define HIVE_SIZE_sp_GP_DEVICE_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_thread_sp_ready_queue
#define HIVE_MEM_ia_css_thread_sp_ready_queue scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_thread_sp_ready_queue 0x308
#define HIVE_SIZE_ia_css_thread_sp_ready_queue 12
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_thread_sp_ready_queue scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_thread_sp_ready_queue 0x308
#define HIVE_SIZE_sp_ia_css_thread_sp_ready_queue 12

/* function sp_dma_proxy_set_width_ab: 19F2 */

/* function ia_css_uds_sp_scale_params: 315B */

/* function ia_css_circbuf_increase_size: 413A */

/* function __divu: 437E */

/* function ia_css_thread_sp_get_state: 3F7D */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_N_SHORT_PACKET_LUT_ENTRIES
#define HIVE_MEM_N_SHORT_PACKET_LUT_ENTRIES scalar_processor_2400_dmem
#define HIVE_ADDR_N_SHORT_PACKET_LUT_ENTRIES 0x180
#define HIVE_SIZE_N_SHORT_PACKET_LUT_ENTRIES 12
#else
#endif
#endif
#define HIVE_MEM_sp_N_SHORT_PACKET_LUT_ENTRIES scalar_processor_2400_dmem
#define HIVE_ADDR_sp_N_SHORT_PACKET_LUT_ENTRIES 0x180
#define HIVE_SIZE_sp_N_SHORT_PACKET_LUT_ENTRIES 12

/* function thread_fiber_sp_main: 4133 */

/* function ia_css_spctrl_sp_set_state: 3BB6 */

/* function ia_css_thread_sem_sp_signal: 4938 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_IRQ_BASE
#define HIVE_MEM_IRQ_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_IRQ_BASE 0x2C
#define HIVE_SIZE_IRQ_BASE 16
#else
#endif
#endif
#define HIVE_MEM_sp_IRQ_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_IRQ_BASE 0x2C
#define HIVE_SIZE_sp_IRQ_BASE 16

/* function ia_css_virtual_isys_sp_isr_init: 3C47 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_TIMED_CTRL_BASE
#define HIVE_MEM_TIMED_CTRL_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_TIMED_CTRL_BASE 0x40
#define HIVE_SIZE_TIMED_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_TIMED_CTRL_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_TIMED_CTRL_BASE 0x40
#define HIVE_SIZE_sp_TIMED_CTRL_BASE 4

/* function ia_css_thread_sem_sp_init: 4A0B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_is_isp_requested
#define HIVE_MEM_is_isp_requested scalar_processor_2400_dmem
#define HIVE_ADDR_is_isp_requested 0x1D64
#define HIVE_SIZE_is_isp_requested 4
#else
#endif
#endif
#define HIVE_MEM_sp_is_isp_requested scalar_processor_2400_dmem
#define HIVE_ADDR_sp_is_isp_requested 0x1D64
#define HIVE_SIZE_sp_is_isp_requested 4

/* function ia_css_dmaproxy_sp_execute: 1AAD */

/* function csi_rx_backend_rst: 1259 */

/* function ia_css_queue_is_empty: 33A3 */

/* function ia_css_circbuf_extract: 4248 */

/* function ia_css_isys_stream_get_state: DB4 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sp_threads_fiber
#define HIVE_MEM_pocapp_sp_threads_fiber scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sp_threads_fiber 0xEC
#define HIVE_SIZE_pocapp_sp_threads_fiber 12
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sp_threads_fiber scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sp_threads_fiber 0xEC
#define HIVE_SIZE_sp_pocapp_sp_threads_fiber 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_sp_thread
#define HIVE_MEM_current_sp_thread scalar_processor_2400_dmem
#define HIVE_ADDR_current_sp_thread 0x304
#define HIVE_SIZE_current_sp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_sp_thread scalar_processor_2400_dmem
#define HIVE_ADDR_sp_current_sp_thread 0x304
#define HIVE_SIZE_sp_current_sp_thread 4

/* function ia_css_spctrl_sp_get_spid: 3BBD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_psyspoc_sp2host_psys_event_queue_handle
#define HIVE_MEM_psyspoc_sp2host_psys_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_psyspoc_sp2host_psys_event_queue_handle 0x555C
#define HIVE_SIZE_psyspoc_sp2host_psys_event_queue_handle 12
#else
#endif
#endif
#define HIVE_MEM_sp_psyspoc_sp2host_psys_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_psyspoc_sp2host_psys_event_queue_handle 0x555C
#define HIVE_SIZE_sp_psyspoc_sp2host_psys_event_queue_handle 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_psysdebug_execute_start_cnt
#define HIVE_MEM_psysdebug_execute_start_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_psysdebug_execute_start_cnt 0x1164
#define HIVE_SIZE_psysdebug_execute_start_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_psysdebug_execute_start_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_psysdebug_execute_start_cnt 0x1164
#define HIVE_SIZE_sp_psysdebug_execute_start_cnt 4

/* function ia_css_dmaproxy_sp_read_byte_addr: 4794 */

/* function ia_css_virtual_isys_sync_all: 3C6B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_N_STREAM2MMIO_SID_PROCS
#define HIVE_MEM_N_STREAM2MMIO_SID_PROCS scalar_processor_2400_dmem
#define HIVE_ADDR_N_STREAM2MMIO_SID_PROCS 0x200
#define HIVE_SIZE_N_STREAM2MMIO_SID_PROCS 12
#else
#endif
#endif
#define HIVE_MEM_sp_N_STREAM2MMIO_SID_PROCS scalar_processor_2400_dmem
#define HIVE_ADDR_sp_N_STREAM2MMIO_SID_PROCS 0x200
#define HIVE_SIZE_sp_N_STREAM2MMIO_SID_PROCS 12

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_isp_stop_req
#define HIVE_MEM_pocapp_isp_stop_req scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_isp_stop_req 0x330
#define HIVE_SIZE_pocapp_isp_stop_req 4
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_isp_stop_req scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_isp_stop_req 0x330
#define HIVE_SIZE_sp_pocapp_isp_stop_req 4

/* function ia_css_circbuf_peek: 4227 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pipeline_sp_curr_binary_id
#define HIVE_MEM_pipeline_sp_curr_binary_id scalar_processor_2400_dmem
#define HIVE_ADDR_pipeline_sp_curr_binary_id 0xF8
#define HIVE_SIZE_pipeline_sp_curr_binary_id 4
#else
#endif
#endif
#define HIVE_MEM_sp_pipeline_sp_curr_binary_id scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pipeline_sp_curr_binary_id 0xF8
#define HIVE_SIZE_sp_pipeline_sp_curr_binary_id 4

/* function ia_css_queue_get_used_space: 3472 */

/* function ia_css_ispctrl_sp_output_compute_dma_info: 271A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_bufq_sp_pipe_private_s3a_bufs
#define HIVE_MEM_ia_css_bufq_sp_pipe_private_s3a_bufs scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_bufq_sp_pipe_private_s3a_bufs 0x5D40
#define HIVE_SIZE_ia_css_bufq_sp_pipe_private_s3a_bufs 60
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_bufq_sp_pipe_private_s3a_bufs scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_bufq_sp_pipe_private_s3a_bufs 0x5D40
#define HIVE_SIZE_sp_ia_css_bufq_sp_pipe_private_s3a_bufs 60

/* function ia_css_queue_is_full: 350F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_event_thread_loop_cnt
#define HIVE_MEM_debug_event_thread_loop_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_debug_event_thread_loop_cnt 0x334
#define HIVE_SIZE_debug_event_thread_loop_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_event_thread_loop_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_debug_event_thread_loop_cnt 0x334
#define HIVE_SIZE_sp_debug_event_thread_loop_cnt 4

/* function debug_buffer_init_isp: FB */

/* function pocapp_sp_event_proxy_init: 608 */

/* function ibuf_ctrl_run: 1354 */

/* function ia_css_isys_stream_container_reset: DBE */

/* function ia_css_thread_sp_yield: 48AE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_dmaproxy_sp_invalidate_tlb
#define HIVE_MEM_ia_css_dmaproxy_sp_invalidate_tlb scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_dmaproxy_sp_invalidate_tlb 0x5EAC
#define HIVE_SIZE_ia_css_dmaproxy_sp_invalidate_tlb 4
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_dmaproxy_sp_invalidate_tlb scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_dmaproxy_sp_invalidate_tlb 0x5EAC
#define HIVE_SIZE_sp_ia_css_dmaproxy_sp_invalidate_tlb 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_psys_thread
#define HIVE_MEM_pocapp_psys_thread scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_psys_thread 0x1F88
#define HIVE_SIZE_pocapp_psys_thread 68
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_psys_thread scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_psys_thread 0x1F88
#define HIVE_SIZE_sp_pocapp_psys_thread 68

/* function ia_css_thread_sp_fork: 400A */

/* function pocapp_sp_event_proxy_func: 5BC */

/* function ia_css_dmaproxy_sp_vmem_read: 1A36 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_N_LONG_PACKET_LUT_ENTRIES
#define HIVE_MEM_N_LONG_PACKET_LUT_ENTRIES scalar_processor_2400_dmem
#define HIVE_ADDR_N_LONG_PACKET_LUT_ENTRIES 0x18C
#define HIVE_SIZE_N_LONG_PACKET_LUT_ENTRIES 12
#else
#endif
#endif
#define HIVE_MEM_sp_N_LONG_PACKET_LUT_ENTRIES scalar_processor_2400_dmem
#define HIVE_ADDR_sp_N_LONG_PACKET_LUT_ENTRIES 0x18C
#define HIVE_SIZE_sp_N_LONG_PACKET_LUT_ENTRIES 12

/* function ia_css_thread_sp_init: 4036 */

/* function ia_css_ispctrl_sp_set_stream_base_addr: 2EE0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_DMEM_BASE
#define HIVE_MEM_ISP_DMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_ISP_DMEM_BASE 0x10
#define HIVE_SIZE_ISP_DMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_DMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ISP_DMEM_BASE 0x10
#define HIVE_SIZE_sp_ISP_DMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_DMEM_BASE
#define HIVE_MEM_SP_DMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_SP_DMEM_BASE 0x4
#define HIVE_SIZE_SP_DMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_SP_DMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_SP_DMEM_BASE 0x4
#define HIVE_SIZE_sp_SP_DMEM_BASE 4

/* function ibuf_ctrl_transfer: 1316 */

/* function ia_css_dmaproxy_sp_read: 1ACE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_raw_copy_line_count
#define HIVE_MEM_raw_copy_line_count scalar_processor_2400_dmem
#define HIVE_ADDR_raw_copy_line_count 0xFC
#define HIVE_SIZE_raw_copy_line_count 4
#else
#endif
#endif
#define HIVE_MEM_sp_raw_copy_line_count scalar_processor_2400_dmem
#define HIVE_ADDR_sp_raw_copy_line_count 0xFC
#define HIVE_SIZE_sp_raw_copy_line_count 4

/* function ia_css_queue_peek: 33E7 */

/* function ia_css_isys_stream_block_handler: AA6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_can_send_token_mask
#define HIVE_MEM_event_can_send_token_mask scalar_processor_2400_dmem
#define HIVE_ADDR_event_can_send_token_mask 0x88
#define HIVE_SIZE_event_can_send_token_mask 44
#else
#endif
#endif
#define HIVE_MEM_sp_event_can_send_token_mask scalar_processor_2400_dmem
#define HIVE_ADDR_sp_event_can_send_token_mask 0x88
#define HIVE_SIZE_sp_event_can_send_token_mask 44

/* function csi_rx_frontend_stop: 1162 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isp_thread
#define HIVE_MEM_isp_thread scalar_processor_2400_dmem
#define HIVE_ADDR_isp_thread 0x61F0
#define HIVE_SIZE_isp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_isp_thread scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isp_thread 0x61F0
#define HIVE_SIZE_sp_isp_thread 4

/* function is_ddr_debug_buffer_full: 334 */

/* function ia_css_isyspoc_isys_thread_func: 760 */

/* function sp_dma_proxy_isp_write_addr: 1A4E */

/* function ia_css_psyspoc_psys_thread_func: DCD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_psysdebug_msg_cnt
#define HIVE_MEM_psysdebug_msg_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_psysdebug_msg_cnt 0x1160
#define HIVE_SIZE_psysdebug_msg_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_psysdebug_msg_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_psysdebug_msg_cnt 0x1160
#define HIVE_SIZE_sp_psysdebug_msg_cnt 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_isys_stream_container
#define HIVE_MEM_ia_css_isys_stream_container scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_isys_stream_container 0x39DC
#define HIVE_SIZE_ia_css_isys_stream_container 7008
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_isys_stream_container scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_isys_stream_container 0x39DC
#define HIVE_SIZE_sp_ia_css_isys_stream_container 7008

/* function debug_enqueue_ddr: 10A */

/* function ia_css_isys_stream_ack_send_event: B41 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isyspoc_sem_for_isys_sp2host_event_queue
#define HIVE_MEM_isyspoc_sem_for_isys_sp2host_event_queue scalar_processor_2400_dmem
#define HIVE_ADDR_isyspoc_sem_for_isys_sp2host_event_queue 0x3960
#define HIVE_SIZE_isyspoc_sem_for_isys_sp2host_event_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_isyspoc_sem_for_isys_sp2host_event_queue scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isyspoc_sem_for_isys_sp2host_event_queue 0x3960
#define HIVE_SIZE_sp_isyspoc_sem_for_isys_sp2host_event_queue 20

/* function dmaproxy_sp_read_write: 4835 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_dmaproxy_isp_dma_cmd_buffer
#define HIVE_MEM_ia_css_dmaproxy_isp_dma_cmd_buffer scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_dmaproxy_isp_dma_cmd_buffer 0x5EB0
#define HIVE_SIZE_ia_css_dmaproxy_isp_dma_cmd_buffer 4
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_dmaproxy_isp_dma_cmd_buffer scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_dmaproxy_isp_dma_cmd_buffer 0x5EB0
#define HIVE_SIZE_sp_ia_css_dmaproxy_isp_dma_cmd_buffer 4

/* function ia_css_dmaproxy_sp_ack: 44AC */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_sp2host_event_cnt
#define HIVE_MEM_debug_sp2host_event_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_debug_sp2host_event_cnt 0x115C
#define HIVE_SIZE_debug_sp2host_event_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_sp2host_event_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_debug_sp2host_event_cnt 0x115C
#define HIVE_SIZE_sp_debug_sp2host_event_cnt 4

/* function ia_css_dmaproxy_sp_process: 44DB */

/* function ia_css_ispctrl_sp_init_cs: 1E82 */

/* function ia_css_spctrl_sp_init: 3BCB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_psyspoc_sem_for_psys_host2sp_cmd_queue
#define HIVE_MEM_psyspoc_sem_for_psys_host2sp_cmd_queue scalar_processor_2400_dmem
#define HIVE_ADDR_psyspoc_sem_for_psys_host2sp_cmd_queue 0x5568
#define HIVE_SIZE_psyspoc_sem_for_psys_host2sp_cmd_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_psyspoc_sem_for_psys_host2sp_cmd_queue scalar_processor_2400_dmem
#define HIVE_ADDR_sp_psyspoc_sem_for_psys_host2sp_cmd_queue 0x5568
#define HIVE_SIZE_sp_psyspoc_sem_for_psys_host2sp_cmd_queue 20

/* function input_system_input_port_close: 1766 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_output
#define HIVE_MEM_sp_output scalar_processor_2400_dmem
#define HIVE_ADDR_sp_output 0x214C
#define HIVE_SIZE_sp_output 16
#else
#endif
#endif
#define HIVE_MEM_sp_sp_output scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_output 0x214C
#define HIVE_SIZE_sp_sp_output 16

/* function pixelgen_prbs_config: 155E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_CTRL_BASE
#define HIVE_MEM_ISP_CTRL_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_ISP_CTRL_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_CTRL_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ISP_CTRL_BASE 0x8
#define HIVE_SIZE_sp_ISP_CTRL_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_isys_stream_state_lock
#define HIVE_MEM_ia_css_isys_stream_state_lock scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_isys_stream_state_lock 0x553C
#define HIVE_SIZE_ia_css_isys_stream_state_lock 20
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_isys_stream_state_lock scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_isys_stream_state_lock 0x553C
#define HIVE_SIZE_sp_ia_css_isys_stream_state_lock 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_INPUT_FORMATTER_BASE
#define HIVE_MEM_INPUT_FORMATTER_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_INPUT_FORMATTER_BASE 0x4C
#define HIVE_SIZE_INPUT_FORMATTER_BASE 16
#else
#endif
#endif
#define HIVE_MEM_sp_INPUT_FORMATTER_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_INPUT_FORMATTER_BASE 0x4C
#define HIVE_SIZE_sp_INPUT_FORMATTER_BASE 16

/* function psyspoc_send_psys_event: E4A */

/* function sp_dma_proxy_reset_channels: 1D36 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_bufq_host_sp_queue
#define HIVE_MEM_ia_css_bufq_host_sp_queue scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_bufq_host_sp_queue 0x1DA4
#define HIVE_SIZE_ia_css_bufq_host_sp_queue 484
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_bufq_host_sp_queue scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_bufq_host_sp_queue 0x1DA4
#define HIVE_SIZE_sp_ia_css_bufq_host_sp_queue 484

/* function thread_fiber_sp_create: 40A2 */

/* function ia_css_dmaproxy_sp_set_increments: 1BC3 */

/* function pixelgen_tpg_is_done: 15D8 */

/* function ia_css_isys_stream_capture_indication: 3D3A */

/* function sp_start_isp_entry: 40F */
#ifndef HIVE_MULTIPLE_PROGRAMS
#ifdef HIVE_ADDR_sp_start_isp_entry
#endif
#define HIVE_ADDR_sp_start_isp_entry 0x40F
#endif
#define HIVE_ADDR_sp_sp_start_isp_entry 0x40F

/* function ia_css_dmaproxy_sp_channel_acquire: 1D65 */

/* function ibuf_ctrl_config: 1374 */

/* function ia_css_isys_stream_stop: 3DB2 */

/* function __ia_css_dmaproxy_sp_wait_for_ack_text: 19E9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_isys_thread
#define HIVE_MEM_pocapp_isys_thread scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_isys_thread 0x1FCC
#define HIVE_SIZE_pocapp_isys_thread 68
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_isys_thread scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_isys_thread 0x1FCC
#define HIVE_SIZE_sp_pocapp_isys_thread 68

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_group
#define HIVE_MEM_sp_group scalar_processor_2400_dmem
#define HIVE_ADDR_sp_group 0x2160
#define HIVE_SIZE_sp_group 6120
#else
#endif
#endif
#define HIVE_MEM_sp_sp_group scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_group 0x2160
#define HIVE_SIZE_sp_sp_group 6120

/* function ia_css_thread_sp_kill: 3FD0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sp_output
#define HIVE_MEM_pocapp_sp_output scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sp_output 0x1D68
#define HIVE_SIZE_pocapp_sp_output 16
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sp_output scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sp_output 0x1D68
#define HIVE_SIZE_sp_pocapp_sp_output 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_MMU_BASE
#define HIVE_MEM_MMU_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_MMU_BASE 0x24
#define HIVE_SIZE_MMU_BASE 8
#else
#endif
#endif
#define HIVE_MEM_sp_MMU_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_MMU_BASE 0x24
#define HIVE_SIZE_sp_MMU_BASE 8

/* function ia_css_dmaproxy_sp_channel_release: 1D4E */

/* function pixelgen_prbs_run: 154C */

/* function ia_css_dmaproxy_sp_is_idle: 1D21 */

/* function isp_hmem_load: 10AF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_isys_sp_error_cnt
#define HIVE_MEM_ia_css_isys_sp_error_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_isys_sp_error_cnt 0x3948
#define HIVE_SIZE_ia_css_isys_sp_error_cnt 16
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_isys_sp_error_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_isys_sp_error_cnt 0x3948
#define HIVE_SIZE_sp_ia_css_isys_sp_error_cnt 16

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_buffer_ddr_address
#define HIVE_MEM_debug_buffer_ddr_address scalar_processor_2400_dmem
#define HIVE_ADDR_debug_buffer_ddr_address 0xBC
#define HIVE_SIZE_debug_buffer_ddr_address 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_buffer_ddr_address scalar_processor_2400_dmem
#define HIVE_ADDR_sp_debug_buffer_ddr_address 0xBC
#define HIVE_SIZE_sp_debug_buffer_ddr_address 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_host2sp_event_queue_handle
#define HIVE_MEM_pocapp_host2sp_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_host2sp_event_queue_handle 0x1D78
#define HIVE_SIZE_pocapp_host2sp_event_queue_handle 12
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_host2sp_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_host2sp_event_queue_handle 0x1D78
#define HIVE_SIZE_sp_pocapp_host2sp_event_queue_handle 12

/* function ia_css_thread_sp_set_priority: 3FC8 */

/* function sizeof_hmem: 115A */

/* function input_system_channel_open: 189A */

/* function pixelgen_tpg_stop: 15C6 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sp_threads
#define HIVE_MEM_pocapp_sp_threads scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sp_threads 0xCC
#define HIVE_SIZE_pocapp_sp_threads 8
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sp_threads scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sp_threads 0xCC
#define HIVE_SIZE_sp_pocapp_sp_threads 8

/* function __ia_css_dmaproxy_sp_process_text: 1935 */

/* function ia_css_dmaproxy_sp_set_width_exception: 1BAD */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_param_handle
#define HIVE_MEM_param_handle scalar_processor_2400_dmem
#define HIVE_ADDR_param_handle 0x5D7C
#define HIVE_SIZE_param_handle 12
#else
#endif
#endif
#define HIVE_MEM_sp_param_handle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_param_handle 0x5D7C
#define HIVE_SIZE_sp_param_handle 12

/* function __modu: 43C4 */

/* function ia_css_dmaproxy_sp_init_isp_vector: 1A08 */

/* function input_system_channel_transfer: 1883 */

/* function isp_vamem_store: 0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GDC_BASE
#define HIVE_MEM_GDC_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_GDC_BASE 0x44
#define HIVE_SIZE_GDC_BASE 8
#else
#endif
#endif
#define HIVE_MEM_sp_GDC_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_GDC_BASE 0x44
#define HIVE_SIZE_sp_GDC_BASE 8

/* function ia_css_queue_local_init: 36D8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sp2host_event_queue_handle
#define HIVE_MEM_pocapp_sp2host_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sp2host_event_queue_handle 0x1D84
#define HIVE_SIZE_pocapp_sp2host_event_queue_handle 12
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sp2host_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sp2host_event_queue_handle 0x1D84
#define HIVE_SIZE_sp_pocapp_sp2host_event_queue_handle 12

/* function ia_css_dmaproxy_sp_deregister_channel_from_port: 19D0 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_thread_sp_num_ready_threads
#define HIVE_MEM_ia_css_thread_sp_num_ready_threads scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_thread_sp_num_ready_threads 0x6558
#define HIVE_SIZE_ia_css_thread_sp_num_ready_threads 4
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_thread_sp_num_ready_threads scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_thread_sp_num_ready_threads 0x6558
#define HIVE_SIZE_sp_ia_css_thread_sp_num_ready_threads 4

/* function ia_css_ispctrl_sp_isp_done_row_striping: 2700 */

/* function __ia_css_virtual_isys_sp_isr_text: 3C0A */

/* function ia_css_queue_dequeue: 3557 */

/* function ia_css_dmaproxy_sp_configure_channel: 47AB */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isyspoc_sem_for_isys_host2sp_cmd_queue
#define HIVE_MEM_isyspoc_sem_for_isys_host2sp_cmd_queue scalar_processor_2400_dmem
#define HIVE_ADDR_isyspoc_sem_for_isys_host2sp_cmd_queue 0x3974
#define HIVE_SIZE_isyspoc_sem_for_isys_host2sp_cmd_queue 20
#else
#endif
#endif
#define HIVE_MEM_sp_isyspoc_sem_for_isys_host2sp_cmd_queue scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isyspoc_sem_for_isys_host2sp_cmd_queue 0x3974
#define HIVE_SIZE_sp_isyspoc_sem_for_isys_host2sp_cmd_queue 20

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_current_thread_fiber_sp
#define HIVE_MEM_current_thread_fiber_sp scalar_processor_2400_dmem
#define HIVE_ADDR_current_thread_fiber_sp 0x6560
#define HIVE_SIZE_current_thread_fiber_sp 4
#else
#endif
#endif
#define HIVE_MEM_sp_current_thread_fiber_sp scalar_processor_2400_dmem
#define HIVE_ADDR_sp_current_thread_fiber_sp 0x6560
#define HIVE_SIZE_sp_current_thread_fiber_sp 4

/* function ia_css_circbuf_pop: 42E0 */

/* function memset: 4443 */

/* function irq_raise_set_token: B7 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_GPIO_BASE
#define HIVE_MEM_GPIO_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_GPIO_BASE 0x3C
#define HIVE_SIZE_GPIO_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_GPIO_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_GPIO_BASE 0x3C
#define HIVE_SIZE_sp_GPIO_BASE 4

/* function pixelgen_prbs_stop: 153A */

/* function ia_css_ispctrl_sp_init_ds: 2020 */

/* function get_xmem_base_addr_raw: 2384 */

/* function pixelgen_tpg_config: 15FB */

/* function ia_css_circbuf_create: 432A */

/* function csi_rx_frontend_run: 1173 */

/* function ia_css_isys_stream_open: 3E46 */

/* function input_system_channel_configure: 18AE */

/* function isp_hmem_clear: 1076 */

/* function stream2mmio_config: 14E6 */

/* function ia_css_ispctrl_sp_start_binary: 1E60 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_callout_cnt
#define HIVE_MEM_debug_callout_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_debug_callout_cnt 0x338
#define HIVE_SIZE_debug_callout_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_callout_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_debug_callout_cnt 0x338
#define HIVE_SIZE_sp_debug_callout_cnt 4

/* function csi_rx_frontend_config: 11CB */

/* function ia_css_rmgr_sp_rel_gen: 3E97 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_event_any_pending_mask
#define HIVE_MEM_event_any_pending_mask scalar_processor_2400_dmem
#define HIVE_ADDR_event_any_pending_mask 0x328
#define HIVE_SIZE_event_any_pending_mask 8
#else
#endif
#endif
#define HIVE_MEM_sp_event_any_pending_mask scalar_processor_2400_dmem
#define HIVE_ADDR_sp_event_any_pending_mask 0x328
#define HIVE_SIZE_sp_event_any_pending_mask 8

/* function ia_css_pipeline_sp_get_pipe_io_status: 703 */

/* function sh_css_decode_tag_descr: 3CD */

/* function debug_enqueue_isp: 2DF */

/* function ia_css_spctrl_sp_uninit: 3BC4 */

/* function csi_rx_backend_run: 11B9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_bufq_sp_pipe_private_dis_bufs
#define HIVE_MEM_ia_css_bufq_sp_pipe_private_dis_bufs scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_bufq_sp_pipe_private_dis_bufs 0x5D88
#define HIVE_SIZE_ia_css_bufq_sp_pipe_private_dis_bufs 140
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_bufq_sp_pipe_private_dis_bufs scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_bufq_sp_pipe_private_dis_bufs 0x5D88
#define HIVE_SIZE_sp_ia_css_bufq_sp_pipe_private_dis_bufs 140

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sem_for_isp_idle
#define HIVE_MEM_sem_for_isp_idle scalar_processor_2400_dmem
#define HIVE_ADDR_sem_for_isp_idle 0x1D90
#define HIVE_SIZE_sem_for_isp_idle 20
#else
#endif
#endif
#define HIVE_MEM_sp_sem_for_isp_idle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sem_for_isp_idle 0x1D90
#define HIVE_SIZE_sp_sem_for_isp_idle 20

/* function ia_css_dmaproxy_sp_write_byte_addr: 1A7C */

/* function ia_css_dmaproxy_sp_init: 19A2 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_VAMEM_BASE
#define HIVE_MEM_ISP_VAMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_ISP_VAMEM_BASE 0x14
#define HIVE_SIZE_ISP_VAMEM_BASE 12
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_VAMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ISP_VAMEM_BASE 0x14
#define HIVE_SIZE_sp_ISP_VAMEM_BASE 12

/* function input_system_channel_sync: 186F */

/* function pocapp_sp_event_proxy_callout_func: 447E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_bufq_sp_pipe_private_exp_ids
#define HIVE_MEM_ia_css_bufq_sp_pipe_private_exp_ids scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_bufq_sp_pipe_private_exp_ids 0x5E14
#define HIVE_SIZE_ia_css_bufq_sp_pipe_private_exp_ids 70
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_bufq_sp_pipe_private_exp_ids scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_bufq_sp_pipe_private_exp_ids 0x5E14
#define HIVE_SIZE_sp_ia_css_bufq_sp_pipe_private_exp_ids 70

/* function ia_css_queue_item_load: 37ED */

/* function ia_css_spctrl_sp_get_state: 3BAF */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_callout_sp_thread
#define HIVE_MEM_callout_sp_thread scalar_processor_2400_dmem
#define HIVE_ADDR_callout_sp_thread 0x6554
#define HIVE_SIZE_callout_sp_thread 4
#else
#endif
#endif
#define HIVE_MEM_sp_callout_sp_thread scalar_processor_2400_dmem
#define HIVE_ADDR_sp_callout_sp_thread 0x6554
#define HIVE_SIZE_sp_callout_sp_thread 4

/* function ia_css_isys_stream_update_block: B59 */

/* function thread_fiber_sp_init: 4129 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_SP_PMEM_BASE
#define HIVE_MEM_SP_PMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_SP_PMEM_BASE 0x0
#define HIVE_SIZE_SP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_SP_PMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_SP_PMEM_BASE 0x0
#define HIVE_SIZE_sp_SP_PMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_input_stream_format
#define HIVE_MEM_sp_isp_input_stream_format scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isp_input_stream_format 0x5E5C
#define HIVE_SIZE_sp_isp_input_stream_format 20
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_input_stream_format scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_isp_input_stream_format 0x5E5C
#define HIVE_SIZE_sp_sp_isp_input_stream_format 20

/* function __mod: 43B0 */

/* function ia_css_dmaproxy_sp_init_dmem_channel: 1AE8 */

/* function ia_css_thread_sp_join: 3FF9 */

/* function ia_css_dmaproxy_sp_add_command: 4877 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_dmaproxy_sp_proxy_status
#define HIVE_MEM_dmaproxy_sp_proxy_status scalar_processor_2400_dmem
#define HIVE_ADDR_dmaproxy_sp_proxy_status 0x25C
#define HIVE_SIZE_dmaproxy_sp_proxy_status 4
#else
#endif
#endif
#define HIVE_MEM_sp_dmaproxy_sp_proxy_status scalar_processor_2400_dmem
#define HIVE_ADDR_sp_dmaproxy_sp_proxy_status 0x25C
#define HIVE_SIZE_sp_dmaproxy_sp_proxy_status 4

/* function ia_css_pipeline_sp_wait_for_isys_stream_N: 3CE5 */

/* function ia_css_circbuf_peek_from_start: 4206 */

/* function ia_css_thread_sp_run: 406C */

/* function ia_css_sp_isp_param_init_isp_memories: 3026 */

/* function pocapp_init_psys_event_queues: E6B */

/* function irq_raise: C9 */

/* function ia_css_dmaproxy_sp_mmu_invalidate: 18F0 */

/* function csi_rx_backend_disable: 1185 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_N_CSI_RX_FE_CTRL_DLANES
#define HIVE_MEM_N_CSI_RX_FE_CTRL_DLANES scalar_processor_2400_dmem
#define HIVE_ADDR_N_CSI_RX_FE_CTRL_DLANES 0x198
#define HIVE_SIZE_N_CSI_RX_FE_CTRL_DLANES 12
#else
#endif
#endif
#define HIVE_MEM_sp_N_CSI_RX_FE_CTRL_DLANES scalar_processor_2400_dmem
#define HIVE_ADDR_sp_N_CSI_RX_FE_CTRL_DLANES 0x198
#define HIVE_SIZE_sp_N_CSI_RX_FE_CTRL_DLANES 12

/* function ia_css_dmaproxy_sp_read_byte_addr_mmio: 477D */

/* function ia_css_ispctrl_sp_done_ds: 2003 */

/* function csi_rx_backend_config: 11DC */

/* function ia_css_sp_isp_param_get_mem_inits: 3001 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_HMEM_BASE
#define HIVE_MEM_ISP_HMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_ISP_HMEM_BASE 0x20
#define HIVE_SIZE_ISP_HMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_HMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ISP_HMEM_BASE 0x20
#define HIVE_SIZE_sp_ISP_HMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_bufq_sp_pipe_private_frames
#define HIVE_MEM_ia_css_bufq_sp_pipe_private_frames scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_bufq_sp_pipe_private_frames 0x5E70
#define HIVE_SIZE_ia_css_bufq_sp_pipe_private_frames 60
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_bufq_sp_pipe_private_frames scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_bufq_sp_pipe_private_frames 0x5E70
#define HIVE_SIZE_sp_ia_css_bufq_sp_pipe_private_frames 60

/* function ia_css_ispctrl_sp_init_isp_vars: 2DAE */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_pocapp_sp_event_proxy_thread
#define HIVE_MEM_pocapp_sp_event_proxy_thread scalar_processor_2400_dmem
#define HIVE_ADDR_pocapp_sp_event_proxy_thread 0x2010
#define HIVE_SIZE_pocapp_sp_event_proxy_thread 68
#else
#endif
#endif
#define HIVE_MEM_sp_pocapp_sp_event_proxy_thread scalar_processor_2400_dmem
#define HIVE_ADDR_sp_pocapp_sp_event_proxy_thread 0x2010
#define HIVE_SIZE_sp_pocapp_sp_event_proxy_thread 68

/* function ia_css_isys_stream_start: 3D85 */

/* function ia_css_dmaproxy_sp_write: 1A93 */

/* function ia_css_isys_stream_start_async: 3DE9 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_irq_sw_interrupt_token
#define HIVE_MEM_irq_sw_interrupt_token scalar_processor_2400_dmem
#define HIVE_ADDR_irq_sw_interrupt_token 0x1D60
#define HIVE_SIZE_irq_sw_interrupt_token 4
#else
#endif
#endif
#define HIVE_MEM_sp_irq_sw_interrupt_token scalar_processor_2400_dmem
#define HIVE_ADDR_sp_irq_sw_interrupt_token 0x1D60
#define HIVE_SIZE_sp_irq_sw_interrupt_token 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_isp_addresses
#define HIVE_MEM_sp_isp_addresses scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isp_addresses 0x61F4
#define HIVE_SIZE_sp_isp_addresses 164
#else
#endif
#endif
#define HIVE_MEM_sp_sp_isp_addresses scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_isp_addresses 0x61F4
#define HIVE_SIZE_sp_sp_isp_addresses 164

/* function ia_css_rmgr_sp_acq_gen: 3EBB */

/* function input_system_input_port_open: 17B8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_host_sp_queues_initialized
#define HIVE_MEM_host_sp_queues_initialized scalar_processor_2400_dmem
#define HIVE_ADDR_host_sp_queues_initialized 0x3958
#define HIVE_SIZE_host_sp_queues_initialized 4
#else
#endif
#endif
#define HIVE_MEM_sp_host_sp_queues_initialized scalar_processor_2400_dmem
#define HIVE_ADDR_sp_host_sp_queues_initialized 0x3958
#define HIVE_SIZE_sp_host_sp_queues_initialized 4

/* function ia_css_queue_uninit: 3696 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ia_css_ispctrl_sp_isp_started
#define HIVE_MEM_ia_css_ispctrl_sp_isp_started scalar_processor_2400_dmem
#define HIVE_ADDR_ia_css_ispctrl_sp_isp_started 0x5EB4
#define HIVE_SIZE_ia_css_ispctrl_sp_isp_started 4
#else
#endif
#endif
#define HIVE_MEM_sp_ia_css_ispctrl_sp_isp_started scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ia_css_ispctrl_sp_isp_started 0x5EB4
#define HIVE_SIZE_sp_ia_css_ispctrl_sp_isp_started 4

/* function ia_css_dmaproxy_sp_set_height_exception: 1B9B */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_psysdebug_execute_done_cnt
#define HIVE_MEM_psysdebug_execute_done_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_psysdebug_execute_done_cnt 0x1168
#define HIVE_SIZE_psysdebug_execute_done_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_psysdebug_execute_done_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_psysdebug_execute_done_cnt 0x1168
#define HIVE_SIZE_sp_psysdebug_execute_done_cnt 4

/* function ia_css_dmaproxy_sp_init_vmem_channel: 1B1E */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isyspoc_host2sp_isys_cmd_queue_handle
#define HIVE_MEM_isyspoc_host2sp_isys_cmd_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_isyspoc_host2sp_isys_cmd_queue_handle 0x3988
#define HIVE_SIZE_isyspoc_host2sp_isys_cmd_queue_handle 72
#else
#endif
#endif
#define HIVE_MEM_sp_isyspoc_host2sp_isys_cmd_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isyspoc_host2sp_isys_cmd_queue_handle 0x3988
#define HIVE_SIZE_sp_isyspoc_host2sp_isys_cmd_queue_handle 72

/* function csi_rx_backend_stop: 11A8 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_num_ready_threads
#define HIVE_MEM_num_ready_threads scalar_processor_2400_dmem
#define HIVE_ADDR_num_ready_threads 0x655C
#define HIVE_SIZE_num_ready_threads 4
#else
#endif
#endif
#define HIVE_MEM_sp_num_ready_threads scalar_processor_2400_dmem
#define HIVE_ADDR_sp_num_ready_threads 0x655C
#define HIVE_SIZE_sp_num_ready_threads 4

/* function ia_css_dmaproxy_sp_write_byte_addr_mmio: 1A65 */

/* function ia_css_queue_enqueue: 35E1 */

/* function ia_css_dmaproxy_sp_vmem_write: 1A1F */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_debug_host2sp_event_cnt
#define HIVE_MEM_debug_host2sp_event_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_debug_host2sp_event_cnt 0x33C
#define HIVE_SIZE_debug_host2sp_event_cnt 4
#else
#endif
#endif
#define HIVE_MEM_sp_debug_host2sp_event_cnt scalar_processor_2400_dmem
#define HIVE_ADDR_sp_debug_host2sp_event_cnt 0x33C
#define HIVE_SIZE_sp_debug_host2sp_event_cnt 4

/* function csi_rx_backend_enable: 1196 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_data
#define HIVE_MEM_sp_data scalar_processor_2400_dmem
#define HIVE_ADDR_sp_data 0x62B8
#define HIVE_SIZE_sp_data 640
#else
#endif
#endif
#define HIVE_MEM_sp_sp_data scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_data 0x62B8
#define HIVE_SIZE_sp_sp_data 640

/* function input_system_input_port_configure: 180A */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_BAMEM_BASE
#define HIVE_MEM_ISP_BAMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_ISP_BAMEM_BASE 0x320
#define HIVE_SIZE_ISP_BAMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_BAMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ISP_BAMEM_BASE 0x320
#define HIVE_SIZE_sp_ISP_BAMEM_BASE 4

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_mem_map
#define HIVE_MEM_mem_map scalar_processor_2400_dmem
#define HIVE_ADDR_mem_map 0x557C
#define HIVE_SIZE_mem_map 284
#else
#endif
#endif
#define HIVE_MEM_sp_mem_map scalar_processor_2400_dmem
#define HIVE_ADDR_sp_mem_map 0x557C
#define HIVE_SIZE_sp_mem_map 284

/* function isys2401_dma_config_legacy: 14BC */

/* function ia_css_virtual_isys_sp_isr: 4893 */

/* function thread_sp_queue_print: 4089 */

/* function ia_css_circbuf_destroy: 4321 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_ISP_PMEM_BASE
#define HIVE_MEM_ISP_PMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_ISP_PMEM_BASE 0xC
#define HIVE_SIZE_ISP_PMEM_BASE 4
#else
#endif
#endif
#define HIVE_MEM_sp_ISP_PMEM_BASE scalar_processor_2400_dmem
#define HIVE_ADDR_sp_ISP_PMEM_BASE 0xC
#define HIVE_SIZE_sp_ISP_PMEM_BASE 4

/* function ia_css_sp_isp_param_mem_load: 2F89 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_isyspoc_sp2host_isys_event_queue_handle
#define HIVE_MEM_isyspoc_sp2host_isys_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_isyspoc_sp2host_isys_event_queue_handle 0x39D0
#define HIVE_SIZE_isyspoc_sp2host_isys_event_queue_handle 12
#else
#endif
#endif
#define HIVE_MEM_sp_isyspoc_sp2host_isys_event_queue_handle scalar_processor_2400_dmem
#define HIVE_ADDR_sp_isyspoc_sp2host_isys_event_queue_handle 0x39D0
#define HIVE_SIZE_sp_isyspoc_sp2host_isys_event_queue_handle 12

/* function __div: 4368 */

/* function ia_css_thread_sem_sp_wait: 4983 */

#ifndef HIVE_MULTIPLE_PROGRAMS
#ifndef HIVE_MEM_sp_sleep_mode
#define HIVE_MEM_sp_sleep_mode scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sleep_mode 0x395C
#define HIVE_SIZE_sp_sleep_mode 4
#else
#endif
#endif
#define HIVE_MEM_sp_sp_sleep_mode scalar_processor_2400_dmem
#define HIVE_ADDR_sp_sp_sleep_mode 0x395C
#define HIVE_SIZE_sp_sp_sleep_mode 4

/* function mmu_invalidate_cache: E0 */

/* function ia_css_dmaproxy_sp_register_channel_to_port: 19DD */

/* function ia_css_queue_remote_init: 36B8 */

/* function ia_css_isys_invalidate_msgs: BB0 */

#define HIVE_ICACHE_sp_critical_SEGMENT_START 0
#define HIVE_ICACHE_sp_critical_NUM_SEGMENTS  1

#endif /* _sp_map_h_ */
extern void sh_css_dump_sp_dmem(void);
void sh_css_dump_sp_dmem(void)
{
}
