
#include "ia_css_isys_private.h"
/* The following is needed for the contained data types */
#include "ia_css_isys_fw_bridged_types.h"
#include "ia_css_isysapi_types.h"
#include "ia_css_syscom_config.h"
/* The following header file is needed for the
 * stddef.h (NULL),
 * limits.h (CHAR_BIT definition).
 */
#include "type_support.h"
#include "error_support.h"
#include "assert_support.h"

#include "ia_css_input_buffer_cpu.h"


static inline void resolution_host_to_css(const struct ia_css_isys_resolution *resolution_host, struct ia_css_isys_resolution_comm *resolution_css) {
	resolution_css->width = resolution_host->width;
	resolution_css->height = resolution_host->height;
}

static inline void buffer_partition_host_to_css(const struct ia_css_isys_buffer_partition *buffer_partition_host, struct ia_css_isys_buffer_partition_comm *buffer_partition_css) {
	int i;
	for (i=0; i<STREAM_ID_MAX; i++) {
		buffer_partition_css->block_size[i] = buffer_partition_host->block_size[i];
	}
	buffer_partition_css->nof_blocks = buffer_partition_host->nof_blocks;
}

static inline void output_pin_payload_host_to_css(const struct ia_css_isys_output_pin_payload *output_pin_payload_host, struct ia_css_isys_output_pin_payload_comm *output_pin_payload_css) {
	output_pin_payload_css->out_buf_id = output_pin_payload_host->out_buf_id;
	output_pin_payload_css->addr = output_pin_payload_host->addr;
}

static inline void output_pin_info_host_to_css(const struct ia_css_isys_output_pin_info *output_pin_info_host, struct ia_css_isys_output_pin_info_comm *output_pin_info_css) {
	output_pin_info_css->pt = output_pin_info_host->pt;
	switch (output_pin_info_host->pt) {
	case IA_CSS_ISYS_PIN_TYPE_MIPI:
		output_pin_info_css->type_specifics.dt = output_pin_info_host->type_specifics.dt;
		break;
	case IA_CSS_ISYS_PIN_TYPE_RAW_NS:
	case IA_CSS_ISYS_PIN_TYPE_RAW_S:
		output_pin_info_css->type_specifics.ft = output_pin_info_host->type_specifics.ft;
		break;
	default:
		break;
	}
	resolution_host_to_css(&output_pin_info_host->output_res, &output_pin_info_css->output_res);
	output_pin_info_css->watermark_in_lines = output_pin_info_host->watermark_in_lines;
	output_pin_info_css->send_irq = output_pin_info_host->send_irq;
	output_pin_info_css->input_pin_index = output_pin_info_host->input_pin_index;
}

static inline void input_pin_host_to_css(const struct ia_css_isys_input_pin *input_pin_host, struct ia_css_isys_input_pin_comm *input_pin_css) {
	input_pin_css->in_buf_id = input_pin_host->in_buf_id;
	input_pin_css->addr = input_pin_host->addr;
}

static inline void input_pin_info_host_to_css(const struct ia_css_isys_input_pin_info *input_pin_info_host, struct ia_css_isys_input_pin_info_comm *input_pin_info_css) {
	input_pin_info_css->dt = input_pin_info_host->dt;
	resolution_host_to_css(&input_pin_info_host->input_res, &input_pin_info_css->input_res);
	input_pin_info_css->crop.top_offset = input_pin_info_host->crop.top_offset;
	input_pin_info_css->crop.left_offset = input_pin_info_host->crop.left_offset;
	input_pin_info_css->crop.bottom_offset = input_pin_info_host->crop.bottom_offset;
	input_pin_info_css->crop.right_offset = input_pin_info_host->crop.right_offset;
}

static inline void output_pin_host_to_css(const struct ia_css_isys_output_pin *output_pin_host, struct ia_css_isys_output_pin_comm *output_pin_css) {
	output_pin_payload_host_to_css(&output_pin_host->payload, &output_pin_css->payload);
	output_pin_info_host_to_css(&output_pin_host->info, &output_pin_css->info);
}

static inline void isl_cfg_host_to_css(const struct ia_css_isys_isl_cfg *isl_cfg_host, struct ia_css_isys_isl_cfg_comm *isl_cfg_css) {
	isl_cfg_css->isl_input_pin_index0 = isl_cfg_host->isl_input_pin_index0;
	isl_cfg_css->isl_input_pin_index1 = isl_cfg_host->isl_input_pin_index1;
}

static inline void isa_cfg_host_to_css(const struct ia_css_isys_isa_cfg *isa_cfg_host, struct ia_css_isys_isa_cfg_comm *isa_cfg_css) {
	isa_cfg_css->acc_enabled_set.blc_enabled = isa_cfg_host->acc_enabled_set.blc_enabled;
	isa_cfg_css->acc_enabled_set.lsc_enabled = isa_cfg_host->acc_enabled_set.lsc_enabled;
	isa_cfg_css->acc_enabled_set.dpc_enabled = isa_cfg_host->acc_enabled_set.dpc_enabled;
	isa_cfg_css->acc_enabled_set.downscaler_enabled = isa_cfg_host->acc_enabled_set.downscaler_enabled;
	isa_cfg_css->acc_enabled_set.awb_enabled = isa_cfg_host->acc_enabled_set.awb_enabled;
	isa_cfg_css->acc_enabled_set.af_enabled = isa_cfg_host->acc_enabled_set.af_enabled;
	isa_cfg_css->acc_enabled_set.ae_enabled = isa_cfg_host->acc_enabled_set.ae_enabled;
	isa_cfg_css->isa_input_pin_index = isa_cfg_host->isa_input_pin_index;
}

static inline void stream_cfg_data_host_to_css(const struct ia_css_isys_stream_cfg_data *stream_cfg_data_host, struct ia_css_isys_stream_cfg_data_comm *stream_cfg_data_css) {
	int i;
	stream_cfg_data_css->isl_use = stream_cfg_data_host->isl_use;
	switch (stream_cfg_data_host->isl_use) {
	case IA_CSS_ISYS_USE_SINGLE_DUAL_ISL:
		isl_cfg_host_to_css(&stream_cfg_data_host->isl_isa_cfg.isl_cfg, &stream_cfg_data_css->isl_isa_cfg.isl_cfg);
		break;
	case IA_CSS_ISYS_USE_SINGLE_ISA:
		isa_cfg_host_to_css(&stream_cfg_data_host->isl_isa_cfg.isa_cfg, &stream_cfg_data_css->isl_isa_cfg.isa_cfg);
		break;
	case IA_CSS_ISYS_USE_NO_ISL_NO_ISA:
		break;
	default:
		break;
	}
	stream_cfg_data_css->src = stream_cfg_data_host->src;
	stream_cfg_data_css->vc = stream_cfg_data_host->vc;
	stream_cfg_data_css->nof_input_pins = stream_cfg_data_host->nof_input_pins;
	for (i=0; i<MAX_IPINS; i++) {
		input_pin_info_host_to_css(&stream_cfg_data_host->input_pins[i], &stream_cfg_data_css->input_pins[i]);
	}
	stream_cfg_data_css->nof_output_pins = stream_cfg_data_host->nof_output_pins;
	for (i=0; i<MAX_OPINS; i++) {
		output_pin_info_host_to_css(&stream_cfg_data_host->output_pins[i], &stream_cfg_data_css->output_pins[i]);
	}
}

static inline void frame_buff_set_host_to_css(const struct ia_css_isys_frame_buff_set *frame_buff_set_host, struct ia_css_isys_frame_buff_set_comm *frame_buff_set_css) {
	int i;
	for (i=0; i<MAX_OPINS; i++) {
		output_pin_host_to_css(&frame_buff_set_host->output_pins[i], &frame_buff_set_css->output_pins[i]);
	}
	{
		input_pin_host_to_css(&frame_buff_set_host->input_pin_set.blc_param, &frame_buff_set_css->input_pin_set.blc_param);
		input_pin_host_to_css(&frame_buff_set_host->input_pin_set.lsc_param, &frame_buff_set_css->input_pin_set.lsc_param);
		input_pin_host_to_css(&frame_buff_set_host->input_pin_set.dpc_param, &frame_buff_set_css->input_pin_set.dpc_param);
	}
	frame_buff_set_css->send_irq_sof = frame_buff_set_host->send_irq_sof;
	frame_buff_set_css->send_irq_eof = frame_buff_set_host->send_irq_eof;
}


static inline void resolution_css_to_host(const struct ia_css_isys_resolution_comm *resolution_css, struct ia_css_isys_resolution *resolution_host) {
	resolution_host->width = resolution_css->width;
	resolution_host->height = resolution_css->height;
}

static inline void output_pin_payload_css_to_host(const struct ia_css_isys_output_pin_payload_comm *output_pin_payload_css, struct ia_css_isys_output_pin_payload *output_pin_payload_host) {
	output_pin_payload_host->out_buf_id = output_pin_payload_css->out_buf_id;
	output_pin_payload_host->addr = output_pin_payload_css->addr;
}

static inline void output_pin_info_css_to_host(const struct ia_css_isys_output_pin_info_comm *output_pin_info_css, struct ia_css_isys_output_pin_info *output_pin_info_host) {
	output_pin_info_host->pt = output_pin_info_css->pt;
	switch (output_pin_info_css->pt) {
	case IA_CSS_ISYS_PIN_TYPE_MIPI:
		output_pin_info_host->type_specifics.dt = output_pin_info_css->type_specifics.dt;
		break;
	case IA_CSS_ISYS_PIN_TYPE_RAW_NS:
	case IA_CSS_ISYS_PIN_TYPE_RAW_S:
		output_pin_info_host->type_specifics.ft = output_pin_info_css->type_specifics.ft;
		break;
	default:
		break;
	}
	resolution_css_to_host(&output_pin_info_css->output_res, &output_pin_info_host->output_res);
	output_pin_info_host->watermark_in_lines = output_pin_info_css->watermark_in_lines;
	output_pin_info_host->send_irq = output_pin_info_css->send_irq;
	output_pin_info_host->input_pin_index = output_pin_info_css->input_pin_index;
}

static inline void output_pin_css_to_host(const struct ia_css_isys_output_pin_comm *output_pin_css, struct ia_css_isys_output_pin *output_pin_host) {
	output_pin_payload_css_to_host(&output_pin_css->payload, &output_pin_host->payload);
	output_pin_info_css_to_host(&output_pin_css->info, &output_pin_host->info);
}

static inline void resp_info_css_to_host(const struct ia_css_isys_resp_info_comm *resp_info_css, struct ia_css_isys_resp_info *resp_info_host) {
	resp_info_host->type = resp_info_css->type;
	resp_info_host->timestamp[0] = resp_info_css->timestamp[0];
	resp_info_host->timestamp[1] = resp_info_css->timestamp[1];
	resp_info_host->stream_handle = resp_info_css->stream_handle;
	switch (resp_info_css->type) {
	case IA_CSS_ISYS_RESP_TYPE_PIN_DATA_READY:
		output_pin_css_to_host(&resp_info_css->resp_data.pin, &resp_info_host->resp_data.pin);
		break;
	case IA_CSS_ISYS_RESP_TYPE_STREAM_OPEN_DONE:
	case IA_CSS_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE:
	case IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE:
		resp_info_host->resp_data.buf_id = resp_info_css->resp_data.buf_id;
		break;
	default:
		break;
	}
	resp_info_host->error = resp_info_css->error;
}


/**
 * ia_css_isys_constr_fw_stream_cfg()
 */
int ia_css_isys_constr_fw_stream_cfg(
	ia_css_input_buffer_css_address *pstream_cfg_fw,
	ia_css_input_buffer *pbuf_stream_cfg_id,
	const struct ia_css_isys_stream_cfg_data *stream_cfg
) {
	ia_css_input_buffer_cpu_address stream_cfg_cpu_addr;
	ia_css_input_buffer_css_address stream_cfg_css_addr;

	assert(pstream_cfg_fw != NULL);
	assert(pbuf_stream_cfg_id != NULL);
	assert(stream_cfg != NULL);

	*pbuf_stream_cfg_id = ia_css_input_buffer_alloc(sizeof(struct ia_css_isys_stream_cfg_data_comm));
	stream_cfg_cpu_addr = ia_css_input_buffer_cpu_map(*pbuf_stream_cfg_id);
	verifret(stream_cfg_cpu_addr != (ia_css_input_buffer_cpu_address)NULL, EFAULT);

	stream_cfg_data_host_to_css(stream_cfg, stream_cfg_cpu_addr);

	ia_css_input_buffer_cpu_unmap(*pbuf_stream_cfg_id);

	stream_cfg_css_addr = ia_css_input_buffer_css_map(*pbuf_stream_cfg_id);
	*pstream_cfg_fw = stream_cfg_css_addr;
	return 0;
}


/**
 * ia_css_isys_constr_fw_next_frame()
 */
int ia_css_isys_constr_fw_next_frame(
	ia_css_input_buffer_css_address *pnext_frame_fw,
	ia_css_input_buffer *pbuf_next_frame_id,
	const struct ia_css_isys_frame_buff_set *next_frame
) {
	ia_css_input_buffer_cpu_address next_frame_cpu_addr;
	ia_css_input_buffer_css_address next_frame_css_addr;

	assert(pnext_frame_fw != (ia_css_input_buffer_css_address *)NULL);
	assert(next_frame != NULL);
	//alloc input buffer
	*pbuf_next_frame_id = ia_css_input_buffer_alloc(sizeof(struct ia_css_isys_frame_buff_set_comm));
	//map it in cpu
	next_frame_cpu_addr = ia_css_input_buffer_cpu_map(*pbuf_next_frame_id);
	verifret(next_frame_cpu_addr != (ia_css_input_buffer_cpu_address)NULL, EFAULT);

	frame_buff_set_host_to_css(next_frame, next_frame_cpu_addr);

	//unmap the buffer from cpu
	ia_css_input_buffer_cpu_unmap(*pbuf_next_frame_id);

	//map it to css
	next_frame_css_addr = ia_css_input_buffer_css_map(*pbuf_next_frame_id);

	*pnext_frame_fw = next_frame_css_addr;

	return 0;
}


/**
 * ia_css_isys_extract_fw_response()
 */
int ia_css_isys_extract_fw_response(
	const struct resp_queue_token *token,
	struct ia_css_isys_resp_info *received_response
) {
	assert(received_response != NULL);

	resp_info_css_to_host(&(token->resp_info), received_response);

	return 0;
}


/**
 * ia_css_isys_prepare_param()
 */
int ia_css_isys_prepare_param(
	struct is_css_isys_fw_config *isys_fw_cfg,
	const struct ia_css_isys_buffer_partition *pmipi,
	const struct ia_css_isys_buffer_partition *ppixel
) {
	assert(isys_fw_cfg != NULL);
	assert(pmipi != NULL);
	assert(ppixel != NULL);

	buffer_partition_host_to_css(pmipi, &isys_fw_cfg->mipi);
	buffer_partition_host_to_css(ppixel, &isys_fw_cfg->pixel);

	return 0;
}

