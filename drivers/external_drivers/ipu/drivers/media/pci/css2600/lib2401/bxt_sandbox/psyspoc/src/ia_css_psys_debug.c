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

#include "ia_css_psys_debug.h"

static void
psys_debug_frame_info(
	const struct ia_css_frame_sp_info *frame_info){
	assert(frame_info != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info Res - Width: %u\n", frame_info->res.width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info res - Height: %u\n", frame_info->res.height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info padded_width: %u\n", frame_info->padded_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info format: 0x%x\n", frame_info->format);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info raw_bit_depth: 0x%x\n", frame_info->raw_bit_depth);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info raw_bayer_order: 0x%x\n", frame_info->raw_bayer_order);
}

static void
psys_debug_buf_info(
	const struct ia_css_buffer_sp *buf_info){
	assert(buf_info != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info BUF - XMEM_ADDR: %p\n", buf_info->buf_src.xmem_addr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info BUF - Queue ID: %d\n", buf_info->buf_src.queue_id);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames Info BUF - Type: %d\n", buf_info->buf_type);
}

static void
psys_debug_uds_crop_pos(
	const struct sh_css_crop_pos *crop_pos){
	assert(crop_pos != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS - Crop_Pos x: %u\n",crop_pos->x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS - Crop_Pos y: %u\n",crop_pos->y);
}

static void
psys_debug_uds_info(
	const struct sh_css_uds_info *uds_info){
	assert(uds_info != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS - Curr_dx: %u\n",uds_info->curr_dx);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS - Curr_dy: %u\n",uds_info->curr_dy);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS - xc: %u\n",uds_info->xc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS - yc: %u\n",uds_info->yc);
}

static void
psys_debug_isp_param_info(
	const struct ia_css_isp_parameter_set_info *param_info) {
	int i, j;
	assert(param_info != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : isp_parameters_id: %u\n",param_info->isp_parameters_id);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : output_frame_ptr: 0x%x\n",param_info->output_frame_ptr);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : isp_param: 0x%x\n",param_info->mem_map.isp_param);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : macc_tbl: 0x%x\n",param_info->mem_map.macc_tbl);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : fpn_tbl: 0x%x\n",param_info->mem_map.fpn_tbl);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : sc_tbl: 0x%x\n",param_info->mem_map.sc_tbl);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_r_x: 0x%x\n",param_info->mem_map.tetra_r_x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_r_y: 0x%x\n",param_info->mem_map.tetra_r_y);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_gr_x: 0x%x\n",param_info->mem_map.tetra_gr_x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_gb_x: 0x%x\n",param_info->mem_map.tetra_gb_x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_gb_y: 0x%x\n",param_info->mem_map.tetra_gb_y);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_b_x: 0x%x\n",param_info->mem_map.tetra_b_x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_b_y: 0x%x\n",param_info->mem_map.tetra_b_y);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_ratb_x: 0x%x\n",param_info->mem_map.tetra_ratb_x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_ratb_y: 0x%x\n",param_info->mem_map.tetra_ratb_y);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_batr_x: 0x%x\n",param_info->mem_map.tetra_batr_x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : tetra_batr_y: 0x%x\n",param_info->mem_map.tetra_batr_y);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : dvs_6axis_params_y: 0x%x\n",param_info->mem_map.dvs_6axis_params_y);

	for(i = 0; i < SH_CSS_MAX_STAGES; i++){
		for(j = 0; j < IA_CSS_NUM_MEMORIES; j++)
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM INFO : isp_mem_param[%d][%d]: 0x%x\n",i, j, param_info->mem_map.isp_mem_param[i][j]);
	}
}


static void
psys_debug_print_data(
		const struct ia_css_data data){
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, " address: 0x%x, size %u\n",data.address, data.size);
}

static void
psys_debug_print_isp_mem_initializers(
	const struct ia_css_isp_param_css_segments *segments){
	int i, j;
	assert(segments != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM CSS Segments\n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM CSS Segments start address: 0x%x\n",segments);
	for(i = 0; i < IA_CSS_NUM_PARAM_CLASSES; i++){
		for (j = 0; j <IA_CSS_NUM_MEMORIES; j++){
			ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM CSS Segments data[%d][%d]",i, j);
			psys_debug_print_data(segments->params[i][j]);
		}
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP PARAM CSS Segments: 0x%x\n", segments);
}

static void
psys_debug_print_sp_stage(
	const ia_css_psysapi_cmd_t *cmd){
	assert(cmd != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage: \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - num: %u\n", cmd->sp_stage.num);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - isp_online: %u\n", cmd->sp_stage.isp_online);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - isp_copy_vf: %u\n", cmd->sp_stage.isp_copy_vf);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - isp_copy_output: %u\n", cmd->sp_stage.isp_copy_output);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - sp_enable_xnr: %u\n", cmd->sp_stage.sp_enable_xnr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - isp_deci_log_factor: %u\n", cmd->sp_stage.isp_deci_log_factor);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - isp_vf_downscale_bits: %u\n", cmd->sp_stage.isp_vf_downscale_bits);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - deinterleaved: %u\n", cmd->sp_stage.deinterleaved);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - program_input_circuit: %u\n", cmd->sp_stage.program_input_circuit);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - func: %u\n", cmd->sp_stage.func);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - stage_type: %u\n", cmd->sp_stage.stage_type);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - num_stripes: %u\n", cmd->sp_stage.num_stripes);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - isp_stage_addr: 0x%x\n", cmd->sp_stage.isp_stage_addr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - xmem_bin_addr: 0x%x\n", cmd->sp_stage.xmem_bin_addr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - xmem_map_addr: 0x%x\n", cmd->sp_stage.xmem_map_addr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - top_cropping: %u\n", cmd->sp_stage.top_cropping);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - row_stripes_height: %u\n", cmd->sp_stage.row_stripes_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - row_stripes_overlap_lines: %u\n", cmd->sp_stage.row_stripes_overlap_lines);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - if_config_index: %u\n", cmd->sp_stage.if_config_index);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Crop Position: \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Out Crop Position - x: %u\n", cmd->sp_stage.sp_out_crop_pos.x);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Out Crop Position - y: %u\n", cmd->sp_stage.sp_out_crop_pos.y);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - DVS Envelope: \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - DVS Envelope - width: %u\n", cmd->sp_stage.dvs_envelope.width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - DVS Envelope - height: %u\n", cmd->sp_stage.dvs_envelope.height);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - UDS Info: \n");
	psys_debug_uds_info(&(cmd->sp_stage.uds));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - IN \n");
	psys_debug_frame_info(&(cmd->sp_stage.frames.in.info));
	psys_debug_buf_info(&(cmd->sp_stage.frames.in.buf_attr));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - OUT:0\n");
	psys_debug_frame_info(&(cmd->sp_stage.frames.out[0].info));
	psys_debug_buf_info(&(cmd->sp_stage.frames.out[0].buf_attr));
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - OUT:1\n");
	psys_debug_frame_info(&(cmd->sp_stage.frames.out[1].info));
	psys_debug_buf_info(&(cmd->sp_stage.frames.out[1].buf_attr));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - OUT_VF\n");
	psys_debug_frame_info(&(cmd->sp_stage.frames.out_vf.info));
	psys_debug_buf_info(&(cmd->sp_stage.frames.out_vf.buf_attr));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - S3A BUF\n");
	psys_debug_buf_info(&(cmd->sp_stage.frames.s3a_buf));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - DVS BUF\n");
	psys_debug_buf_info(&(cmd->sp_stage.frames.dvs_buf));
#if defined(IS_ISP_2500_SYSTEM)
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - LACE BUF\n");
	psys_debug_buf_info(&(cmd->sp_stage.frames.lace_buf));
#endif
#if defined SH_CSS_ENABLE_METADATA
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - METADATA BUF\n");
	psys_debug_buf_info(&(cmd->sp_stage.frames.metadata_buf));
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - effective_in_res\n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - effective_in_res - Width: %u\n", cmd->sp_stage.frames.effective_in_res.width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage - Frames - effective_in_res - height: %u\n", cmd->sp_stage.frames.effective_in_res.height);
}

static void
psys_debug_print_isp_stage_blob_info_memory_offsets(
	const ia_css_psysapi_cmd_t *cmd){
	int i;
	assert(cmd != NULL);

	for(i = 0; i < IA_CSS_NUM_PARAM_CLASSES; i++){
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Memory Offset[%d]: %u\n",i, cmd->isp_stage.blob_info.memory_offsets.offsets[i]);
	}

}

static void
psys_debug_print_isp_stage_blob_info(
	const ia_css_psysapi_cmd_t *cmd){

	assert(cmd != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info: \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info addr: 0x%x\n", &cmd->isp_stage.blob_info);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Offset: %u\n", cmd->isp_stage.blob_info.offset);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Memory Offsets:\n");
	psys_debug_print_isp_stage_blob_info_memory_offsets(cmd);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Prog Name Offset: %u\n", cmd->isp_stage.blob_info.prog_name_offset);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Size: %u\n", cmd->isp_stage.blob_info.size);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Padding Size: %u\n", cmd->isp_stage.blob_info.padding_size);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Icache Source: %u\n", cmd->isp_stage.blob_info.icache_source);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Icache Source Size: %u\n", cmd->isp_stage.blob_info.icache_size);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Icache Padding: %u\n", cmd->isp_stage.blob_info.icache_padding);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Text Source: %u\n", cmd->isp_stage.blob_info.text_source);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Text Source Size: %u\n", cmd->isp_stage.blob_info.text_size);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Text Padding: %u\n", cmd->isp_stage.blob_info.text_padding);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Data Source: %u\n", cmd->isp_stage.blob_info.data_source);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Data Source Size: %u\n", cmd->isp_stage.blob_info.data_size);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info Data Padding: %u\n", cmd->isp_stage.blob_info.data_padding);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info BSS Target: %u\n", cmd->isp_stage.blob_info.bss_target);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Blob Info BSS Size: %u\n", cmd->isp_stage.blob_info.bss_size);
}

static void
psys_debug_print_isp_stage_binary_info(
	const ia_css_psysapi_cmd_t *cmd){
	assert(cmd != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "\nISP Stage - Binary Info \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info ID: %u\n", cmd->isp_stage.binary_info.id);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Mode %u\n", cmd->isp_stage.binary_info.pipeline.mode);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Supported BDS Factor: %u\n", cmd->isp_stage.binary_info.bds.supported_bds_factors);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Min IP Width: %u\n", cmd->isp_stage.binary_info.input.min_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Min IP Height: %u\n", cmd->isp_stage.binary_info.input.min_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max IP Width: %u\n", cmd->isp_stage.binary_info.input.max_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max IP Height: %u\n", cmd->isp_stage.binary_info.input.max_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Min OP Width: %u\n", cmd->isp_stage.binary_info.output.min_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Min OP Height: %u\n", cmd->isp_stage.binary_info.output.min_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max OP Width: %u\n", cmd->isp_stage.binary_info.output.max_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max OP Height: %u\n", cmd->isp_stage.binary_info.output.max_height);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max Internal Width: %u\n", cmd->isp_stage.binary_info.internal.max_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max Internal Height: %u\n", cmd->isp_stage.binary_info.internal.max_height);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max DVS Envelope Width: %u\n", cmd->isp_stage.binary_info.dvs.max_envelope_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Max DVS Envelope Height: %u\n", cmd->isp_stage.binary_info.dvs.max_envelope_height);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Variable Resolution: %u\n", cmd->isp_stage.binary_info.pipeline.variable_resolution);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Variable Output Format: %u\n", cmd->isp_stage.binary_info.output.variable_format);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Variable VF Veceven: %u\n", cmd->isp_stage.binary_info.vf_dec.is_variable);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Variable Max VF Log Downscale: %u\n", cmd->isp_stage.binary_info.vf_dec.max_log_downscale);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Top Cropping: %u\n", cmd->isp_stage.binary_info.pipeline.top_cropping);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Left Cropping: %u\n", cmd->isp_stage.binary_info.pipeline.left_cropping);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info S3A Table Use Dmem: %u\n", cmd->isp_stage.binary_info.s3a.s3atbl_use_dmem);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Input: %u\n", cmd->isp_stage.binary_info.input);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info C Sub-Sampling: %u\n", cmd->isp_stage.binary_info.pipeline.c_subsampling);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info output_num_chunks: %u\n", cmd->isp_stage.binary_info.output.num_chunks);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info num_stripes: %u\n", cmd->isp_stage.binary_info.iterator.num_stripes);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info row_stripes_height: %u\n", cmd->isp_stage.binary_info.iterator.row_stripes_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info row_stripes_overlap_lines: %u\n", cmd->isp_stage.binary_info.iterator.row_stripes_overlap_lines);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info pipelining: %u\n", cmd->isp_stage.binary_info.pipeline.pipelining);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info fixed_s3a_deci_log: %u\n", cmd->isp_stage.binary_info.s3a.fixed_s3a_deci_log);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info isp_addresses: %u\n", cmd->isp_stage.binary_info.addresses.isp_addresses);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info main_entry: %u\n", cmd->isp_stage.binary_info.addresses.main_entry);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info in_frame: %u\n", cmd->isp_stage.binary_info.addresses.in_frame);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info out_frame: %u\n", cmd->isp_stage.binary_info.addresses.out_frame);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info in_data: %u\n", cmd->isp_stage.binary_info.addresses.in_data);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info out_data: %u\n", cmd->isp_stage.binary_info.addresses.out_data);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info block_width: %u\n", cmd->isp_stage.binary_info.block.block_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info block_height: %u\n", cmd->isp_stage.binary_info.block.block_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info output_block_height: %u\n", cmd->isp_stage.binary_info.block.output_block_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info dvs_in_block_width: %u\n", cmd->isp_stage.binary_info.block.block_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info dvs_in_block_height: %u\n", cmd->isp_stage.binary_info.block.block_height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info sh_dma_cmd_ptr: %u\n", cmd->isp_stage.binary_info.addresses.sh_dma_cmd_ptr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info isp_pipe_version: %u\n", cmd->isp_stage.binary_info.pipeline.isp_pipe_version);
}

static void
psys_debug_print_binary_info_enable(
	const ia_css_psysapi_cmd_t *cmd){
	assert(cmd != NULL);

#if defined(IS_ISP_2500_SYSTEM)
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable\n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - input_feeder: %u\n", cmd->isp_stage.binary_info.enable.input_feeder);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - lin: %u\n", cmd->isp_stage.binary_info.enable.lin);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - dpc_acc: %u\n", cmd->isp_stage.binary_info.enable.dpc_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - bds_acc: %u\n", cmd->isp_stage.binary_info.enable.bds_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - shd_acc: %u\n", cmd->isp_stage.binary_info.enable.shd_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - shd_ff: %u\n", cmd->isp_stage.binary_info.enable.shd_ff);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - stats_3a_raw_buffer: %u\n", cmd->isp_stage.binary_info.enable.stats_3a_raw_buffer);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - acc_bayer_denoise: %u\n", cmd->isp_stage.binary_info.enable.acc_bayer_denoise);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - bnr_ff: %u\n", cmd->isp_stage.binary_info.enable.bnr_ff);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - awb_acc: %u\n", cmd->isp_stage.binary_info.enable.awb_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - awb_fr_acc: %u\n", cmd->isp_stage.binary_info.enable.awb_fr_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - anr_acc: %u\n", cmd->isp_stage.binary_info.enable.anr_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - rgbpp_acc: %u\n", cmd->isp_stage.binary_info.enable.rgbpp_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - demosaic_acc: %u\n", cmd->isp_stage.binary_info.enable.demosaic_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - dvs_stats: %u\n", cmd->isp_stage.binary_info.enable.dvs_stats);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - lace_stats: %u\n", cmd->isp_stage.binary_info.enable.lace_stats);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - yuvp1_acc: %u\n", cmd->isp_stage.binary_info.enable.yuvp1_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - yuvp2_acc: %u\n", cmd->isp_stage.binary_info.enable.yuvp2_acc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - ae: %u\n", cmd->isp_stage.binary_info.enable.ae);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - af: %u\n", cmd->isp_stage.binary_info.enable.af);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - dergb: %u\n", cmd->isp_stage.binary_info.enable.dergb);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - rgb2yuv: %u\n", cmd->isp_stage.binary_info.enable.rgb2yuv);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - rgb2yuv: %u\n", cmd->isp_stage.binary_info.enable.rgb2yuv);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - high_quality: %u\n", cmd->isp_stage.binary_info.enable.high_quality);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - kerneltest: %u\n", cmd->isp_stage.binary_info.enable.kerneltest);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - routing_bnr_to_anr: %u\n", cmd->isp_stage.binary_info.enable.routing_bnr_to_anr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - routing_anr_to_de: %u\n", cmd->isp_stage.binary_info.enable.routing_anr_to_de);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - routing_rgb_to_yuvp1: %u\n", cmd->isp_stage.binary_info.enable.routing_rgb_to_yuvp1);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - routing_yuvp1_to_yuvp2: %u\n", cmd->isp_stage.binary_info.enable.routing_yuvp1_to_yuvp2);
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - reduced_pipe: %u\n", cmd->isp_stage.binary_info.enable.reduced_pipe);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - vf_veceven: %u\n", cmd->isp_stage.binary_info.enable.vf_veceven);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - dis: %u\n", cmd->isp_stage.binary_info.enable.dis);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - dvs_envelope: %u\n", cmd->isp_stage.binary_info.enable.dvs_envelope);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - uds: %u\n", cmd->isp_stage.binary_info.enable.uds);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - dvs_6axis: %u\n", cmd->isp_stage.binary_info.enable.dvs_6axis);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - block_output: %u\n", cmd->isp_stage.binary_info.enable.block_output);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - streaming_dma: %u\n", cmd->isp_stage.binary_info.enable.streaming_dma);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - ds: %u\n", cmd->isp_stage.binary_info.enable.ds);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - bayer_fir_6db: %u\n", cmd->isp_stage.binary_info.enable.bayer_fir_6db);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - raw_binning: %u\n", cmd->isp_stage.binary_info.enable.raw_binning);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - continuous: %u\n", cmd->isp_stage.binary_info.enable.continuous);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - s3a: %u\n", cmd->isp_stage.binary_info.enable.s3a);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - fpnr: %u\n", cmd->isp_stage.binary_info.enable.fpnr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - macc: %u\n", cmd->isp_stage.binary_info.enable.macc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - output: %u\n", cmd->isp_stage.binary_info.enable.output);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - ref_frame: %u\n", cmd->isp_stage.binary_info.enable.ref_frame);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - tnr: %u\n", cmd->isp_stage.binary_info.enable.tnr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - xnr: %u\n", cmd->isp_stage.binary_info.enable.xnr);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - params: %u\n", cmd->isp_stage.binary_info.enable.params);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - ca_gdc: %u\n", cmd->isp_stage.binary_info.enable.ca_gdc);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - isp_addressesc: %u\n", cmd->isp_stage.binary_info.enable.isp_addresses);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - in_frame: %u\n", cmd->isp_stage.binary_info.enable.in_frame);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - out_frame: %u\n", cmd->isp_stage.binary_info.enable.out_frame);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info Enable - high_speed: %u\n", cmd->isp_stage.binary_info.enable.high_speed);
}

static void
psys_debug_print_binary_info_dma(
	const ia_css_psysapi_cmd_t *cmd) {
	assert(cmd != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - ref_y_channel %u\n", cmd->isp_stage.binary_info.dma.ref_y_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - ref_c_channel %u\n", cmd->isp_stage.binary_info.dma.ref_c_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - tnr_channel %u\n", cmd->isp_stage.binary_info.dma.tnr_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - tnr_out_channel %u\n", cmd->isp_stage.binary_info.dma.tnr_out_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - dvs_coords_channel %u\n", cmd->isp_stage.binary_info.dma.dvs_coords_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - output_channel %u\n", cmd->isp_stage.binary_info.dma.output_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - c_channel %u\n", cmd->isp_stage.binary_info.dma.c_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - vfout_channel %u\n", cmd->isp_stage.binary_info.dma.vfout_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - vfout_c_channel %u\n", cmd->isp_stage.binary_info.dma.vfout_c_channel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - vfdec_bits_per_pixel %u\n", cmd->isp_stage.binary_info.dma.vfdec_bits_per_pixel);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info DMA - claimed_by_isp %u\n", cmd->isp_stage.binary_info.dma.claimed_by_isp);
}

static void
psys_debug_print_binary_info_uds(
	const ia_css_psysapi_cmd_t *cmd) {
	assert(cmd != NULL);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - bpp: %u\n", cmd->isp_stage.binary_info.uds.bpp);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - use_bci: %u\n", cmd->isp_stage.binary_info.uds.use_bci);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - use_str: %u\n", cmd->isp_stage.binary_info.uds.use_str);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - woix: %u\n", cmd->isp_stage.binary_info.uds.woix);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - woiy: %u\n", cmd->isp_stage.binary_info.uds.woiy);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - extra_out_vecs: %u\n", cmd->isp_stage.binary_info.uds.extra_out_vecs);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - vectors_per_line_in: %u\n", cmd->isp_stage.binary_info.uds.vectors_per_line_in);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - vectors_per_line_out: %u\n", cmd->isp_stage.binary_info.uds.vectors_per_line_out);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - vmem_gdc_in_block_height_y: %u\n", cmd->isp_stage.binary_info.uds.vmem_gdc_in_block_height_y);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Info UDS - vmem_gdc_in_block_height_c: %u\n", cmd->isp_stage.binary_info.uds.vmem_gdc_in_block_height_c);
}

static void
psys_debug_print_pipeline(
	const ia_css_psysapi_cmd_t *cmd) {
	int i;
	assert(cmd != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline\n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline pipe_id: %u\n", cmd->sp_pipeline.pipe_id);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline pipe_num: %u\n", cmd->sp_pipeline.pipe_num);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline thread_id: %u\n", cmd->sp_pipeline.thread_id);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline pipe_config: %u\n", cmd->sp_pipeline.pipe_config);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline inout_port_config: %u\n", cmd->sp_pipeline.inout_port_config);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline required_bds_factor: %u\n", cmd->sp_pipeline.required_bds_factor);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline dvs_frame_delay: %u\n", cmd->sp_pipeline.dvs_frame_delay);
#if !defined(HAS_NO_INPUT_SYSTEM)
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline input_system_mode: %u\n", cmd->sp_pipeline.input_system_mode);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline port_id: %u\n", cmd->sp_pipeline.port_id);
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline num_stages: %u\n", cmd->sp_pipeline.num_stages);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline running: %u\n", cmd->sp_pipeline.running);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline port_id: %u\n", cmd->sp_pipeline.port_id);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline num_execs: %u\n", cmd->sp_pipeline.num_execs);
#if defined(SH_CSS_ENABLE_PER_FRAME_PARAMS)
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline output_frame_queue_id: %u\n", cmd->sp_pipeline.output_frame_queue_id);
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline copy - bin - bytes_available: %u\n", cmd->sp_pipeline.copy.bin.bytes_available);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline copy - raw - height: %u\n", cmd->sp_pipeline.copy.raw.height);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline copy - raw - width: %u\n", cmd->sp_pipeline.copy.raw.width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline copy - raw - padded_width: %u\n", cmd->sp_pipeline.copy.raw.padded_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline copy - raw - max_input_width: %u\n", cmd->sp_pipeline.copy.raw.max_input_width);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline copy - raw - raw_bit_depth: %u\n", cmd->sp_pipeline.copy.raw.raw_bit_depth);

	for(i = 0; i < SH_CSS_MAX_STAGES; i++)
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Pipeline SP Stage Addr: Stage: [%d] - 0x%x\n", i, cmd->sp_pipeline.sp_stage_addr[i]);
}

static void
psys_debug_print_isp_stage(
	const ia_css_psysapi_cmd_t *cmd) {
	assert(cmd != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "\nISP Stage: \n");
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Binary Name: %s\n", cmd->isp_stage.binary_name);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage - Mem Initializers: 0x%x\n", cmd->isp_stage.mem_initializers);

	psys_debug_print_isp_mem_initializers(&(cmd->isp_stage.mem_initializers));
	psys_debug_print_isp_stage_blob_info(cmd);
	psys_debug_print_isp_stage_binary_info(cmd);
	psys_debug_print_binary_info_enable(cmd);
	psys_debug_print_binary_info_dma(cmd);
	psys_debug_print_binary_info_uds(cmd);
}

void
ia_css_debug_psys_cmd_print(
	const ia_css_psysapi_cmd_t *cmd) {
	int i;
	assert(cmd != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "%s - Enter\n", __FUNCTION__);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "cmd: 0x%x\n", cmd);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Stage addr: 0x%x\n", &cmd->isp_stage);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Stage addr: 0x%x\n", cmd->sp_stage);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "SP Pipeline addr: 0x%x\n", &cmd->sp_pipeline);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS Params addr: 0x%x\n", cmd->uds_params);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Param Info addr: 0x%x\n", cmd->isp_param_info);
	psys_debug_print_isp_stage(cmd);
	psys_debug_print_sp_stage(cmd);
	psys_debug_print_pipeline(cmd);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "UDS Params: \n");
	for(i = 0; i < SH_CSS_MAX_STAGES; i++) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Stage: %d\n", i);
		psys_debug_uds_crop_pos(&(cmd->uds_params[i].crop_pos));
		psys_debug_uds_info(&(cmd->uds_params[i].uds));
	}
#if !defined(IS_ISP_2500_SYSTEM)
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ISP Param Info: \n");
	psys_debug_isp_param_info(&(cmd->isp_param_info));
#endif
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "Psys Event: %d\n", cmd->psys_event);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "%s - Leave\n", __FUNCTION__);
}


