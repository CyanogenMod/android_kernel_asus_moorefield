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

#include "ia_css_psys_sppipeline.h"
#include "ia_css_psys_cmd_comm.h"
#include "sh_css_internal.h"
#include "ia_css_memory_access.h"
#include "assert_support.h"
#include "ia_css_pg_param_internal.h"
#include "ia_css_psys_frameadapter.h"
#include "ia_css_psys_process_group.h"
#include "ia_css_psys_program_group_manifest.h"
#include "ia_css_psys_process_group.h"
#include "ia_css_psys_terminal.h"
#include "ia_css_binary.h"
#include "ia_css_frame.h"
#include "ia_css_debug.h"
#include "ia_css_isp_param.h"
#include "ia_css_bufq_comm.h"
#include "ia_css_psys_pgpoc_context.h"

#include "memory_access.h"

#define IA_CSS_INCLUDE_CONFIGURATIONS
#include "ia_css_isp_configs.h"
#define IA_CSS_INCLUDE_STATES
#include "ia_css_isp_states.h"
#include "isp/modes/interface/isp_const.h"

#include "ia_css_3a.h"
#include "ia_css_psyspoc_params.h"
#include "ia_css_psys_terminal.hsys.user.h"
#include "psys_params_sections.h"  /* get_cached_param_data()*/

static void psys_init_frame_info_from_terminals(
	ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context);

static void psys_terminal_get_frame_info(
	struct ia_css_frame_info *info,
	const ia_css_frame_descriptor_t *desc);

static void psys_terminal_get_frame_info(
	struct ia_css_frame_info *info,
	const ia_css_frame_descriptor_t *desc)
{
	assert(info != NULL && desc != NULL);
	memset(info, 0, sizeof(struct ia_css_frame_info));
	info->res.width = desc->dimension[IA_CSS_COL_DIMENSION];
	info->res.height = desc->dimension[IA_CSS_ROW_DIMENSION];
	/* convert from psys data formats to css formats. */
	info->format = ia_css_psys_helper_css_frame_format(
		desc->frame_format_type);
	info->raw_bit_depth = desc->bpp;
	/* default: grbg */
	info->raw_bayer_order = 0;
	ia_css_frame_info_set_width(info,
		desc->dimension[IA_CSS_COL_DIMENSION], 0);
}

static struct ia_css_binary_xinfo *get_binary_xinfo(
	const ia_css_process_group_t *process_group)
{
	struct ia_css_binary_xinfo *all_binaries = NULL;
	struct ia_css_binary_xinfo *binary_xinfo = NULL;
	uint32_t pgid, no_of_binaries, i;

	assert(process_group != NULL);

	pgid = (uint32_t) ia_css_process_group_get_program_group_ID(process_group);
	ia_css_binary_get_isp_binaries(&all_binaries, &no_of_binaries);
	assert(all_binaries != NULL);

	for(i = 0; i < no_of_binaries; i++) {
		binary_xinfo = &all_binaries[i];
		assert(binary_xinfo != NULL);
		if(binary_xinfo->sp.id == pgid)
			break;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"get_binary_xinfo(): binary id(%u) info(0x%x)\n", pgid, binary_xinfo);

	return binary_xinfo;
}

static void
copy_buffer_to_spframe(
	struct sh_css_sp_stage *sp_stage,
	struct ia_css_frame_sp *sp_frame_out,
	const struct ia_css_frame_info *frame_info_in,
	const hrt_vaddress data,
	enum ia_css_buffer_type type)
{
	struct ia_css_frame frame_local;
	assert(sp_stage != NULL && sp_frame_out != NULL && data != mmgr_NULL);

	sp_frame_out->buf_attr.buf_src.xmem_addr = data;
	sp_frame_out->buf_attr.buf_type = type;

	/* frame_info(css)->sp_frame_info(css) */
	ia_css_frame_info_to_frame_sp_info(&sp_frame_out->info, frame_info_in);

	/* only to get plane offsets */
	frame_local.info = *frame_info_in;
	ia_css_frame_init_planes(&frame_local);

	switch (frame_local.info.format) {
	case IA_CSS_FRAME_FORMAT_RAW_PACKED:
	case IA_CSS_FRAME_FORMAT_RAW:
		sp_frame_out->planes.raw.offset = frame_local.planes.raw.offset;
		break;
	case IA_CSS_FRAME_FORMAT_RGB565:
	case IA_CSS_FRAME_FORMAT_RGBA888:
		sp_frame_out->planes.rgb.offset = frame_local.planes.rgb.offset;
		break;
	case IA_CSS_FRAME_FORMAT_PLANAR_RGB888:
		sp_frame_out->planes.planar_rgb.r.offset =
			frame_local.planes.planar_rgb.r.offset;
		sp_frame_out->planes.planar_rgb.g.offset =
			frame_local.planes.planar_rgb.g.offset;
		sp_frame_out->planes.planar_rgb.b.offset =
			frame_local.planes.planar_rgb.b.offset;
		break;
	case IA_CSS_FRAME_FORMAT_YUYV:
	case IA_CSS_FRAME_FORMAT_UYVY:
	case IA_CSS_FRAME_FORMAT_CSI_MIPI_YUV420_8:
	case IA_CSS_FRAME_FORMAT_YUV_LINE:
		sp_frame_out->planes.yuyv.offset = frame_local.planes.yuyv.offset;
		break;
	case IA_CSS_FRAME_FORMAT_NV11:
	case IA_CSS_FRAME_FORMAT_NV12:
	case IA_CSS_FRAME_FORMAT_NV21:
	case IA_CSS_FRAME_FORMAT_NV16:
	case IA_CSS_FRAME_FORMAT_NV61:
		sp_frame_out->planes.nv.y.offset =
			frame_local.planes.nv.y.offset;
		sp_frame_out->planes.nv.uv.offset =
			frame_local.planes.nv.uv.offset;
		break;
	case IA_CSS_FRAME_FORMAT_YUV420:
	case IA_CSS_FRAME_FORMAT_YUV422:
	case IA_CSS_FRAME_FORMAT_YUV444:
	case IA_CSS_FRAME_FORMAT_YUV420_16:
	case IA_CSS_FRAME_FORMAT_YUV422_16:
	case IA_CSS_FRAME_FORMAT_YV12:
	case IA_CSS_FRAME_FORMAT_YV16:
		sp_frame_out->planes.yuv.y.offset =
			frame_local.planes.yuv.y.offset;
		sp_frame_out->planes.yuv.u.offset =
			frame_local.planes.yuv.u.offset;
		sp_frame_out->planes.yuv.v.offset =
			frame_local.planes.yuv.v.offset;
		break;
	case IA_CSS_FRAME_FORMAT_QPLANE6:
		sp_frame_out->planes.plane6.r.offset =
			frame_local.planes.plane6.r.offset;
		sp_frame_out->planes.plane6.r_at_b.offset =
			frame_local.planes.plane6.r_at_b.offset;
		sp_frame_out->planes.plane6.gr.offset =
			frame_local.planes.plane6.gr.offset;
		sp_frame_out->planes.plane6.gb.offset =
			frame_local.planes.plane6.gb.offset;
		sp_frame_out->planes.plane6.b.offset =
			frame_local.planes.plane6.b.offset;
		sp_frame_out->planes.plane6.b_at_r.offset =
			frame_local.planes.plane6.b_at_r.offset;
		break;
	case IA_CSS_FRAME_FORMAT_BINARY_8:
		sp_frame_out->planes.binary.data.offset =
			frame_local.planes.binary.data.offset;
		break;
	default:
		/* This should not happen, but in case it does,
		 * nullify the planes
		 */
		memset(&sp_frame_out->planes, 0, sizeof(sp_frame_out->planes));
		break;
	}
}

static hrt_vaddress get_data_of_terminal(
	ia_css_process_group_t *process_group,
	ia_css_frame_format_type_t format_type)
{
	uint32_t i, terminal_count;
	ia_css_terminal_t *terminal;
	ia_css_frame_descriptor_t *desc;

	assert(process_group != NULL);

	terminal_count = ia_css_process_group_get_terminal_count(process_group);
	for (i = 0; i < terminal_count; i++) {
		terminal = ia_css_process_group_get_terminal(process_group, i);
		if(terminal != NULL) {
			desc = ia_css_data_terminal_get_frame_descriptor(
					(ia_css_data_terminal_t *)terminal);
			if (format_type == desc->frame_format_type) {
				ia_css_frame_t *psys_frame;
				/* TODO: Terminals must have generic buffers, not frames. */
				/* TODO: check for num planes to be 1 if possible */
				psys_frame = ia_css_data_terminal_get_frame(
							(ia_css_data_terminal_t *)terminal);
				assert(psys_frame != NULL);
				return (hrt_vaddress) psys_frame->data;
			}
		}
	}

	return 0;
}

static void
initialize_frame_buffer_attribute(struct ia_css_buffer_sp *buf_attr)
{
	buf_attr->buf_src.queue_id = SH_CSS_INVALID_QUEUE_ID;
	buf_attr->buf_type = IA_CSS_BUFFER_TYPE_INVALID;
}

static void spstage_set_static_frame_defaults(struct sh_css_sp_stage *sp_stage)
{
	uint32_t i;

	assert(sp_stage != NULL);

	initialize_frame_buffer_attribute(&sp_stage->frames.in.buf_attr);
	for (i = 0; i < IA_CSS_BINARY_MAX_OUTPUT_PORTS; i++) {
		initialize_frame_buffer_attribute(&sp_stage->frames.out[i].buf_attr);
	}
	initialize_frame_buffer_attribute(&sp_stage->frames.out_vf.buf_attr);
	initialize_frame_buffer_attribute(&sp_stage->frames.s3a_buf);
	initialize_frame_buffer_attribute(&sp_stage->frames.dvs_buf);
#if defined(IS_ISP_2500_SYSTEM)
	initialize_frame_buffer_attribute(&sp_stage->frames.lace_buf);
#endif
#if defined SH_CSS_ENABLE_METADATA
	initialize_frame_buffer_attribute(&sp_stage->frames.metadata_buf);
#endif
}

/* Populate frame info from process terminal properties. Terminal properties are
 * populated during process creation via terminal params.
 * TODO: Handle state terminals and param terminals as needed. */
static void psys_init_frame_info_from_terminals(
	ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context)
{
	uint32_t i, terminal_count;
	ia_css_terminal_type_t type;
	ia_css_terminal_t *terminal;
	ia_css_frame_descriptor_t *desc;
	struct ia_css_frame_info *info, *out_info = NULL;

	assert(process_group != NULL && context != NULL);

	context->enable_vf_output = 0;
	terminal_count = ia_css_process_group_get_terminal_count(process_group);
	for (i = 0; i < terminal_count; i++) {
		terminal = ia_css_process_group_get_terminal(process_group, i);
		if(terminal != NULL) {
			ia_css_frame_format_type_t frame_format;
			desc = ia_css_data_terminal_get_frame_descriptor(
					(ia_css_data_terminal_t *)terminal);
			type = ia_css_terminal_get_type(terminal);
			frame_format = desc->frame_format_type;
			/* TODO: make sure we don't fill frame info for parameter data */
			if(type == IA_CSS_TERMINAL_TYPE_DATA_IN) {
				info = &context->allocated_in_info;
			} else if(type == IA_CSS_TERMINAL_TYPE_DATA_OUT) {
				/* skip conversion for 3A. TODO: maybe more such types? */
				if ((frame_format == IA_CSS_DATA_S3A_STATISTICS_HI) ||
					(frame_format == IA_CSS_DATA_S3A_STATISTICS_LO) ||
					(frame_format == IA_CSS_DATA_S3A_HISTOGRAM))
					continue;

				if(out_info != NULL) {
					info = &context->out_vf_info;
					context->enable_vf_output = 1;;
				} else {
					out_info = info = &context->out_info;
				}
			} else {
				continue;
			}

			psys_terminal_get_frame_info(info, desc);
		}
	}
}

static void store_spframes_in_spstage(
	struct sh_css_sp_stage *sp_stage,
	const ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context)
{
	uint32_t i, terminal_count;
	ia_css_terminal_type_t type;
	ia_css_terminal_t *terminal;
	bool primary_out_buffer_recvd =false;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"store_spframes_in_spstage(): enter\n");

	assert(sp_stage);
	assert(process_group != NULL && context != NULL);

	spstage_set_static_frame_defaults(sp_stage);
	terminal_count = ia_css_process_group_get_terminal_count(process_group);
	for (i = 0; i < terminal_count; i++) {
		hrt_vaddress data;
		ia_css_frame_t *psys_frame;
		terminal = ia_css_process_group_get_terminal(process_group, i);
		assert(terminal != NULL);
		if(terminal == NULL) {
			return;  /* TODO: Communicate error to higher layers.*/
		}
		if (ia_css_is_terminal_parameter_terminal(terminal)) {
			continue;
		}
		type = ia_css_terminal_get_type(terminal);
		psys_frame = ia_css_data_terminal_get_frame(
				(ia_css_data_terminal_t *)terminal);
		assert(psys_frame != NULL);
		data = psys_frame->data;
		/* TODO: raw bayer order is not signalled yet */
		if(type == IA_CSS_TERMINAL_TYPE_DATA_IN) {
			copy_buffer_to_spframe(sp_stage,
				&sp_stage->frames.in,
				&context->allocated_in_info,
				data,
				IA_CSS_BUFFER_TYPE_INPUT_FRAME);
		} else if(type == IA_CSS_TERMINAL_TYPE_DATA_OUT) {
			ia_css_frame_descriptor_t * frame_descriptor =
				ia_css_data_terminal_get_frame_descriptor(
						(ia_css_data_terminal_t *)terminal);
			ia_css_frame_format_type_t frame_format =
				frame_descriptor->frame_format_type;

			/* 3A buffers are passed directly into psys command structure.*/
			if ((frame_format == IA_CSS_DATA_S3A_STATISTICS_HI) ||
				(frame_format == IA_CSS_DATA_S3A_STATISTICS_LO) ||
				(frame_format == IA_CSS_DATA_S3A_HISTOGRAM))
				continue;

			/*Assumption: 2nd output buffer of type IA_CSS_TERMINAL_TYPE_DATA_OUT
			should be assigned to .vf_output, applies to capture binary only*/
			if (primary_out_buffer_recvd){
				assert(context->enable_vf_output);
				copy_buffer_to_spframe(sp_stage,
					&sp_stage->frames.out_vf,
					&context->out_vf_info,
					data,
					IA_CSS_BUFFER_TYPE_VF_OUTPUT_FRAME);
			} else {
				/* first output buffer*/
				copy_buffer_to_spframe(sp_stage,
						/* using port 0 only for now. */
						&sp_stage->frames.out[0],
						&context->out_info,
						data,
						IA_CSS_BUFFER_TYPE_OUTPUT_FRAME);
					primary_out_buffer_recvd = true;
			}
		}
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"store_spframes_in_spstage(): exit\n");
}

static void construct_sppipeline(
	struct sh_css_sp_pipeline *sp_pipeline,
	hrt_vaddress sp_stage_addr,
	uint32_t required_bds_factor,
	uint32_t dvs_frame_delay)
{
	struct sh_css_sp_pipeline *this_pipeline = sp_pipeline;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "construct_sppipeline(): enter\n");
	assert(this_pipeline != NULL);

	memset(this_pipeline, 0 , sizeof(struct sh_css_sp_pipeline));
	this_pipeline->num_stages = 1;
	this_pipeline->pipe_qos_config = QOS_INVALID;
	SH_CSS_PIPE_PORT_CONFIG_SET(this_pipeline->inout_port_config,
		(uint8_t)SH_CSS_PORT_INPUT,
		(uint8_t)SH_CSS_HOST_TYPE,1);
	SH_CSS_PIPE_PORT_CONFIG_SET(this_pipeline->inout_port_config,
		(uint8_t)SH_CSS_PORT_OUTPUT,
		(uint8_t)SH_CSS_HOST_TYPE,1);
	this_pipeline->required_bds_factor = required_bds_factor;
	this_pipeline->sp_stage_addr[0] = sp_stage_addr;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "construct_sppipeline(): exit\n");
	this_pipeline->dvs_frame_delay = dvs_frame_delay;
}

static void construct_spstage(
	ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context,
	struct sh_css_sp_stage *sp_stage,
	hrt_vaddress isp_stage_addr,
	hrt_vaddress program_addr,
	hrt_vaddress param_addr,
	struct ia_css_binary *binary,
	const struct ia_css_pg_param *params)
{
	struct sh_css_sp_stage *this_stage = sp_stage;
	const struct ia_css_binary_info *binary_info;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "construct_spstage(): enter\n");

	assert(this_stage != NULL);
	assert(process_group != NULL && context != NULL);
	assert(binary != NULL && params != NULL);

	binary_info = &binary->info->sp;

	memset(this_stage, 0 , sizeof(struct sh_css_sp_stage));
	this_stage->if_config_index = SH_CSS_IF_CONFIG_NOT_NEEDED;
	this_stage->stage_type = SH_CSS_ISP_STAGE_TYPE;
	this_stage->isp_stage_addr = isp_stage_addr;
	this_stage->xmem_bin_addr = program_addr;
	this_stage->xmem_map_addr = param_addr;

	this_stage->sp_enable_xnr = params->xnr;
	this_stage->enable.vf_output = params->enable_vf_output;
	this_stage->isp_pipe_version = params->isp_pipe_version;
	this_stage->isp_deci_log_factor = params->isp_deci_log_factor;
	this_stage->isp_vf_downscale_bits = params->isp_vf_downscale_bits;
	this_stage->uds = params->uds;
	this_stage->dvs_envelope.width =
		params->dvs_envelope[IA_CSS_COL_DIMENSION];
	this_stage->dvs_envelope.height =
		params->dvs_envelope[IA_CSS_ROW_DIMENSION];
	this_stage->sp_out_crop_pos = params->sp_out_crop_pos;
	/* checkif: effective dimension = in - cropping? */
	this_stage->frames.effective_in_res.width =
		context->allocated_in_info.res.width;
	this_stage->frames.effective_in_res.height =
		context->allocated_in_info.res.height;

	ia_css_frame_info_to_frame_sp_info(&this_stage->frames.internal_frame_info,
		&binary->internal_frame_info);

	/*These Params come from binary info*/
	this_stage->top_cropping = binary_info->pipeline.top_cropping;
	this_stage->enable.sdis = binary_info->enable.dis;
	this_stage->enable.s3a = binary_info->enable.s3a;

	/* ASK: should this also come from parameters */
	this_stage->num_stripes = binary_info->iterator.num_stripes;

	/*NEW ADDITIONS FOR PREVIEW BINARY*/
	this_stage->isp_copy_output = params->copy_output;

	store_spframes_in_spstage(
		sp_stage,
		process_group,
		context);

#if 0 /* TODO: Enable this for downscaling */
	if (binary->info->sp.pipeline.mode == IA_CSS_BINARY_MODE_PREVIEW &&
		(binary->vf_downscale_log2 > 0)) {
		/* TODO: Remove this after preview output decimation is fixed
		 * by configuring out&vf info fiels properly */
		this_stage->frames.out[0].info.padded_width
			<<= binary->vf_downscale_log2;
		this_stage->frames.out[0].info.res.width
			<<= binary->vf_downscale_log2;
		this_stage->frames.out[0].info.res.height
			<<= binary->vf_downscale_log2;
	}
#endif

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "construct_spstage(): exit\n");
	return;
}

static enum ia_css_stream_format get_stream_format(
	struct ia_css_frame_info *in_info)
{
	assert(in_info != NULL);
	switch(in_info->format) {
		case IA_CSS_FRAME_FORMAT_RAW:
		case IA_CSS_FRAME_FORMAT_RAW_PACKED:
		if(in_info->raw_bit_depth == 10)
			return IA_CSS_STREAM_FORMAT_RAW_10;
		break;

		default:
		return IA_CSS_STREAM_FORMAT_YUV420_8;

	}
	return IA_CSS_STREAM_FORMAT_YUV420_8;
}

static void construct_ispstage(
	struct sh_css_sp_pipeline *pipeline,
	struct sh_css_isp_stage *isp_stage,
	ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context,
	struct ia_css_binary *binary,
	struct ia_css_binary_xinfo *binary_xinfo,
	const struct ia_css_pg_param *params,
	struct ia_css_frame **delay_frames,
	struct ia_css_frame **tnr_frames)
{
	struct sh_css_isp_stage *this_isp_stage = isp_stage;
	struct ia_css_isp_param_css_segments mem_if;
	struct ia_css_frame_info effective_in_info;
	const struct ia_css_frame_info *ptr_out_info[IA_CSS_BINARY_MAX_OUTPUT_PORTS],
		*vf_info, *bds_info;
	struct ia_css_frame_info *allocated_in_info, *out_info, *out_vf_info;
	struct ia_css_frame_info ref_info, tnr_info;
	enum ia_css_stream_format stream_format = 0;
	struct ia_css_resolution dvs_env;
	bool  two_ppc = true, deinterleaved = false;
	enum ia_css_err err;
	uint32_t i = 0;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"construct_ispstage(): enter\n");

	assert(this_isp_stage != NULL);
	assert(process_group != NULL && context != NULL);
	assert(binary != NULL);
	assert(binary_xinfo != NULL);
	assert(params != NULL);

	allocated_in_info = &context->allocated_in_info;
	out_info = &context->out_info;
	out_vf_info = &context->out_vf_info;
	vf_info = NULL;
	bds_info = NULL;

	this_isp_stage->blob_info = binary_xinfo->blob->header.blob;
	this_isp_stage->binary_info = binary_xinfo->sp;

	if(binary_xinfo->blob->name) {
		memcpy(this_isp_stage->binary_name, binary_xinfo->blob->name,
			strlen(binary_xinfo->blob->name)+1);
	}

	/* TODO Move this to user space? */
	memset(&mem_if, 0, sizeof(struct ia_css_isp_param_css_segments));
	dvs_env.width = params->dvs_envelope[IA_CSS_COL_DIMENSION];
	dvs_env.height = params->dvs_envelope[IA_CSS_ROW_DIMENSION];

	effective_in_info = *allocated_in_info;
	if ( allocated_in_info->format == IA_CSS_FRAME_FORMAT_RAW ) {
		/*CSS expects raw buffers size less by left/top cropping*/
		effective_in_info.res.width -=
			(binary_xinfo->sp.pipeline.left_cropping + dvs_env.width);
		effective_in_info.res.height -=
			(binary_xinfo->sp.pipeline.top_cropping + dvs_env.height);
		effective_in_info.padded_width =
			CEIL_MUL(effective_in_info.res.width, 2 * ISP_VEC_NELEMS);

		/* Binary cropping requirement is known to be 12.
		 * Assert if it changes */
		assert(binary_xinfo->sp.pipeline.left_cropping+dvs_env.width==12);
		assert(binary_xinfo->sp.pipeline.top_cropping+dvs_env.height==12);
	}

	ptr_out_info[i] = out_info;
	for (i = 1; i < IA_CSS_BINARY_MAX_OUTPUT_PORTS; i++) {
		ptr_out_info[i] = NULL;
	}
	stream_format = get_stream_format(allocated_in_info);
	/* TODO: Fix this for other binaries */
	if ((binary_xinfo->sp.pipeline.mode == IA_CSS_BINARY_MODE_PREVIEW) ||
	   (binary_xinfo->sp.pipeline.mode == IA_CSS_BINARY_MODE_VIDEO)) {
		vf_info = ptr_out_info[0];
		/* assuming raw_binning is false */
		bds_info = &effective_in_info;
	}
	else if (binary_xinfo->sp.pipeline.mode == IA_CSS_BINARY_MODE_PRIMARY) {
		vf_info = out_vf_info;
		assert(context->enable_vf_output);
	}

	/* destroy any existing isp_param structures, just in case */
	ia_css_binary_destroy_isp_parameters(binary);
	/* isp params are allocated here. We only need
	 * ia_css_isp_param_allocate_isp_parameters but some of the isp host
	 * configure functions take a binary, so we might as well use it. */
	err = ia_css_binary_fill_info(binary_xinfo, false, two_ppc,
		stream_format, &effective_in_info, bds_info, ptr_out_info, vf_info,
		binary, &dvs_env, -1, false);
	assert(err == IA_CSS_SUCCESS);
	ia_css_init_memory_interface(&mem_if, &binary->mem_params, &binary->css_params);

	/* Configure ISP via ISP specific host side functions.
	TODO: Add other host configure's here. */
	ia_css_fpn_configure   (binary,  &binary->in_frame_info);
	ia_css_output0_configure(binary, out_info);
	ia_css_output1_configure(binary, out_vf_info);
	ia_css_copy_output_configure(binary, params->copy_output);
	ia_css_output0_configure(binary, out_info);
	ia_css_iterator_configure (binary, allocated_in_info);
	ia_css_output_configure(binary, out_info);
	ia_css_raw_configure(pipeline, binary,
		allocated_in_info, &binary->in_frame_info,
		two_ppc, deinterleaved);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"construct_ispstage(): init state configuration\n");
	if (binary_xinfo->sp.pipeline.mode == IA_CSS_BINARY_MODE_VIDEO) {
		ref_info = binary->internal_frame_info;
		ref_info.format = IA_CSS_FRAME_FORMAT_YUV420;
		ref_info.raw_bit_depth = SH_CSS_REF_BIT_DEPTH;

		for (i = 0; i < NUM_VIDEO_DELAY_FRAMES ; i++){
			if (delay_frames[i]) {
				ia_css_frame_free(delay_frames[i]);
				delay_frames[i] = NULL;
			}
		ia_css_frame_allocate_from_info(
			&delay_frames[i], &ref_info);
#ifdef HRT_CSIM
		ia_css_frame_zero(delay_frames[i]);
#endif
		}

		ia_css_ref_configure (binary,
			(const struct ia_css_frame **)delay_frames,
			pipeline->dvs_frame_delay);

		if (binary_xinfo->sp.enable.block_output){
			tnr_info = binary->out_frame_info[0];
			/* Make tnr reference buffers output block height align */
			tnr_info.res.height = CEIL_MUL(tnr_info.res.height,
				binary_xinfo->sp.block.output_block_height);
		} else {
			tnr_info = binary->internal_frame_info;
		}

		tnr_info.format = IA_CSS_FRAME_FORMAT_YUV_LINE;
		tnr_info.raw_bit_depth = SH_CSS_TNR_BIT_DEPTH;

		for (i = 0; i < NUM_VIDEO_TNR_FRAMES; i++) {
			if (tnr_frames[i]) {
				ia_css_frame_free(tnr_frames[i]);
				tnr_frames[i] = NULL;
			}
		ia_css_frame_allocate_from_info(
					&tnr_frames[i],
					&tnr_info);
#ifdef HRT_CSIM
		ia_css_frame_zero(tnr_frames[i]);
#endif
		}
		ia_css_tnr_configure(binary,
			(const struct ia_css_frame **)tnr_frames);
	}

	for (i = 0; i < IA_CSS_NUM_STATE_IDS; i++) {
		ia_css_kernel_init_state[i](binary);
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"construct_ispstage(): commit config and state to iunit address\n");

	err = ia_css_isp_param_copy_isp_mem_if_to_ddr(
		&binary->css_params,
		&binary->mem_params,
		IA_CSS_PARAM_CLASS_CONFIG);

	assert(err == IA_CSS_SUCCESS);

	err = ia_css_isp_param_copy_isp_mem_if_to_ddr(
		&binary->css_params,
		&binary->mem_params,
		IA_CSS_PARAM_CLASS_STATE);

	assert(err == IA_CSS_SUCCESS);
	this_isp_stage->mem_initializers = mem_if;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"construct_ispstage(): exit\n");
	return;
}

static void handle_dis (ia_css_psys_pgpoc_context_t *context)
{
	struct ia_css_dvs_grid_info *dvs_grid = NULL;
	struct ia_css_grid_info grid;
	hrt_vaddress dvs_stats_vaddr;
	ia_css_psysapi_cmd_t *host_cmd;

	assert(context != NULL);
	host_cmd = &context->host_cmd;
	assert(host_cmd != NULL);

	dvs_stats_vaddr =
		host_cmd->sp_stage.frames.dvs_buf.buf_src.xmem_addr;
	/* reuse dvs buffers until process group is destroyed. */
	if(dvs_stats_vaddr == mmgr_NULL || dvs_stats_vaddr == mmgr_EXCEPTION) {
		ia_css_binary_dvs_grid_info(&context->binary,
				&grid);
		dvs_grid = &grid.dvs_grid;
		context->dvs_stats = ia_css_isp_dvs2_statistics_allocate(dvs_grid);
		assert(context->dvs_stats != NULL);

		dvs_stats_vaddr = mmgr_malloc(sizeof(struct ia_css_isp_dvs_statistics));
		assert(dvs_stats_vaddr != 0);
		host_cmd->sp_stage.frames.dvs_buf.buf_src.xmem_addr =
			dvs_stats_vaddr;
		mmgr_store(dvs_stats_vaddr,
				context->dvs_stats,
				sizeof(struct ia_css_isp_dvs_statistics));
	}
	return;

}

void ia_css_psys_sppipeline_cmd_create(
	ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context)
{
	struct ia_css_binary_xinfo *binary_xinfo = NULL;
	struct ia_css_pg_param param;
	hrt_vaddress isp_stage_addr, sp_stage_addr, param_addr;
	struct ia_css_binary* binary;
	ia_css_psysapi_cmd_t *psys_cmd = NULL;
	struct ia_css_frame **ref_frames;
	struct ia_css_frame **tnr_frames;
	int retval = -1;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_psys_sppipeline_cmd_create(): enter\n ");

	assert(process_group && context != NULL);

	psys_cmd = &context->host_cmd;
	isp_stage_addr = sp_stage_addr = param_addr = 0;
	ref_frames = context->delay_frames;
	tnr_frames = context->tnr_frames;

	memset((void*) psys_cmd, 0, sizeof(ia_css_psysapi_cmd_t));

	binary_xinfo = get_binary_xinfo(process_group);
	assert(binary_xinfo != NULL);
	if (binary_xinfo == NULL) {
		return; /* should not happen; silences KW issue */
	}
	psys_init_frame_info_from_terminals(process_group, context);

	binary = &context->binary;

	COMPILATION_ERROR_IF(SIZE_OF_IA_CSS_PG_PARAM_STRUCT_IN_BITS
				!= (CHAR_BIT * sizeof(ia_css_pg_param_t)));

	retval = psyspoc_get_pg_param_from_sections((const ia_css_process_group_t *)process_group,
					(const ia_css_program_group_manifest_t *)context->pg_manifest,
					&param);
	assert(retval == 0);

	if (retval != 0){
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_psys_sppipeline_cmd_create(): failed to get pg_params from sections \n ");
		return ;
	}

	/* Create SP pipeline */
	construct_sppipeline(
		&psys_cmd->sp_pipeline,
		sp_stage_addr,
		param.required_bds_factor,
		param.dvs_frame_delay);


	/* Create ISP stage */
	construct_ispstage(
		&psys_cmd->sp_pipeline,
		&psys_cmd->isp_stage,
		process_group,
		context,
		binary,
		binary_xinfo,
		&param,
		ref_frames,
		tnr_frames);

	/* Create SP stage */
	construct_spstage(
		process_group,
		context,
		&psys_cmd->sp_stage,
		isp_stage_addr,
		binary_xinfo->xmem_addr,
		param_addr,
		binary,
		&param);

	psys_cmd->uds_params[0].uds = param.uds;
	psys_cmd->uds_params[0].crop_pos.x = binary_xinfo->sp.pipeline.left_cropping;
	psys_cmd->uds_params[0].crop_pos.y = binary_xinfo->sp.pipeline.top_cropping;
	psys_cmd->token = ia_css_process_group_get_token(process_group);
	psys_cmd->current_stripe = 0;
	psys_cmd->stripe_limit = psys_cmd->sp_stage.num_stripes;

	if (binary_xinfo->sp.enable.s3a) {
		psys_cmd->s3a_stats_buffer_hi =
			(uint32_t)get_data_of_terminal(process_group, IA_CSS_DATA_S3A_STATISTICS_HI);
		psys_cmd->s3a_stats_buffer_lo =
			(uint32_t)get_data_of_terminal(process_group, IA_CSS_DATA_S3A_STATISTICS_LO);
		psys_cmd->s3a_hist_buffer =
			(uint32_t)get_data_of_terminal(process_group, IA_CSS_DATA_S3A_HISTOGRAM);
	}

	ia_css_psyspoc_handle_params(process_group, context);
	/* TODO: remove this dis should come from terminals. */
	if(binary_xinfo->sp.enable.dis)
		handle_dis(context);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_psys_sppipeline_cmd_create(): exit\n ");
	return;
}

void ia_css_psys_sppipeline_cmd_free(
	ia_css_process_group_t *process_group,
	ia_css_psys_pgpoc_context_t *context)
{
	unsigned mem;
	struct sh_css_ddr_address_map *mem_map;
	hrt_vaddress dvs_stats_vaddr;
	ia_css_psysapi_cmd_t *host_cmd = NULL;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_psys_sppipeline_cmd_free(): enter\n ");

	assert(process_group != NULL && context != 0);

	host_cmd = &context->host_cmd;

	/* TODO: binary needs to be part of host side cookie */
	ia_css_binary_destroy_isp_parameters(&context->binary);

	ia_css_frame_free_multiple(NUM_VIDEO_DELAY_FRAMES,
						context->delay_frames);
	ia_css_frame_free_multiple(NUM_VIDEO_TNR_FRAMES,
						context->tnr_frames);

	mem_map = &host_cmd->isp_param_info.mem_map;
	for (mem = 0; mem < N_IA_CSS_MEMORIES; mem++) {
		hrt_vaddress ddr_mem_ptr =
			mem_map->isp_mem_param[0][mem];
		if(ddr_mem_ptr)
			mmgr_free(ddr_mem_ptr);
		mem_map->isp_mem_param[0][mem] = mmgr_NULL;
	}

	 /* TODO: remove DVS logic */
	dvs_stats_vaddr =
		host_cmd->sp_stage.frames.dvs_buf.buf_src.xmem_addr;
	if(dvs_stats_vaddr != mmgr_NULL && dvs_stats_vaddr != mmgr_EXCEPTION) {
		ia_css_isp_dvs_statistics_free(context->dvs_stats);
		mmgr_free(dvs_stats_vaddr);
	}

	/* TODO: Tables need to be part of host side cookie */
	if(mem_map->sc_tbl != mmgr_NULL && mem_map->sc_tbl != mmgr_EXCEPTION) {
		mmgr_free(mem_map->sc_tbl);
		mem_map->sc_tbl = mmgr_NULL;
	}

	if(mem_map->macc_tbl != mmgr_NULL && mem_map->macc_tbl != mmgr_EXCEPTION) {
		mmgr_free(mem_map->macc_tbl);
		mem_map->macc_tbl = mmgr_NULL;
	}

	if(mem_map->dvs_6axis_params_y != mmgr_NULL && mem_map->dvs_6axis_params_y != mmgr_EXCEPTION) {
		mmgr_free(mem_map->dvs_6axis_params_y);
		mem_map->dvs_6axis_params_y = mmgr_NULL;
	}

	spstage_set_static_frame_defaults(&host_cmd->sp_stage);
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_psys_sppipeline_cmd_free(): exit\n ");
}
