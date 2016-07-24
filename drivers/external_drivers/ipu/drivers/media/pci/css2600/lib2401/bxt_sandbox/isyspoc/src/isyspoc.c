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



/* TODO: REMOVE --> START IF EXTERNALLY INCLUDED/DEFINED */
/* These are temporary, the correct numbers need to be inserted/linked */
/* Until this happens, the following definitions stay here             */
#define input_mipi_min_width        2
#define input_mipi_max_width        16384
#define input_mipi_min_height       2
#define input_mipi_max_height       16384
#define output_min_width			2
#define output_max_width			16384
#define output_min_height			2
#define output_max_height			16384
/*       REMOVE --> END   IF EXTERNALLY INCLUDED/DEFINED */

#include "input_system.h"
#include "ia_css_isys.h"
#include "memory_access.h"
#include "ia_css_isys_ext_public.h"
/* The FW bridged types are included through the following */
#include "ia_css_isysapi.h"
#include "ia_css_isys_private.h"
/* The following provides the isys-sys context */
#include "ia_css_fwctrl_public.h"
#include "ia_css_fwctrl.h"
#include "ia_css_isyspoc_comm.h"
#include "ia_css_debug.h"

#include "isyspoc_2401.h"

#include "type_support.h"
#include "assert_support.h"
#include "error_support.h"
#include "sh_css_internal.h"

/* Change ISYSPOC_HW_DEBUG to 1 to enable debugging on HW*/
#define ISYSPOC_HW_DEBUG	(0)

#if (ISYSPOC_HW_DEBUG)
#include "spmem_dump.c"
#endif /*ISYSPOC_HW_DEBUG*/

#define HOST_MALLOC        sh_css_malloc
#define HOST_FREE          sh_css_free

#define	ISYS_IP_PIN0	0

static int isyspoc_create_reponse(
	struct ia_css_isys_context *ctx,
	struct ia_css_isys_resp_info *received_response,
	ia_css_isyspoc_cmd_msg_t *isys_msg
);

static void isyspoc_update_isys_buf(
	const uint32_t buf_id,
	const struct ia_css_isys_frame_buff_set *next_frame,
	ia_css_isyspoc_buffer *isys_buf);


#if (ISYSPOC_HW_DEBUG)
static void isyspoc_dump_sp_stream_state(unsigned int stream_handle);
#endif /*ISYSPOC_HW_DEBUG*/
static int isyspoc_validate_output_pin(const struct ia_css_isys_output_pin *output_pin);
static void isyspoc_dump_isys_stream_cfg(const ia_css_isys_stream_cfg_t *isys_stream_cfg);
static void isyspoc_dump_virtual_isys_handle(const virtual_input_system_stream_t *virt_isys);

/**
 * ia_css_isys_device_open() - configure ISYS device
 */
int ia_css_isys_device_open(
	HANDLE *context,
	struct ia_css_isys_device_cfg_data *config
)
{
	unsigned int stream_handle;
	struct ia_css_isys_context *ctx;
	struct ia_css_fwctrl_devconfig device_config;
	int retval = 0;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_isys_device_open() enter: void\n");
	assert(config != NULL);

	/*Make sure that the size of the cmd struct is as expected */
	COMPILATION_ERROR_IF( SIZE_OF_ISYSPOC_CMD_MSG_STRUCT != sizeof(ia_css_isyspoc_cmd_msg_t));

	assert(config->mipi.nof_blocks <= STREAM_ID_MAX);
	for (stream_handle = 0; stream_handle < config->mipi.nof_blocks; stream_handle++) {
		assert(config->mipi.block_size[stream_handle] > 0);
	}

	assert(config->pixel.nof_blocks <= STREAM_ID_MAX);
	for (stream_handle = 0; stream_handle < config->pixel.nof_blocks; stream_handle++) {
		assert(config->pixel.block_size[stream_handle] > 0);
	}

	ctx = (struct ia_css_isys_context *)HOST_MALLOC(sizeof(struct ia_css_isys_context));
	assert(ctx != NULL);
	if(ctx == NULL) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"ia_css_isys_device_open(): Failed to allocate ctx memory\n");
		return ENOMEM;
	}
	memset(ctx, 0, sizeof(struct ia_css_isys_context));
	*context = (HANDLE)ctx;

	for (stream_handle = 0; stream_handle < STREAM_ID_MAX; stream_handle++) {
		ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_IDLE;
		ctx->stream_nof_output_pins[stream_handle] = 0;
	}

	device_config.firmware_address = config->driver_sys.firmware_address;
	retval = ia_css_fwctrl_device_open(&device_config);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_isys_device_open() return: return_err=%d\n", retval);
	return retval;
}

 /**
 * ia_css_isys_stream_open() - open and configure a virtual stream
 */
int ia_css_isys_stream_open(
	HANDLE context,
	unsigned int stream_handle,
	const struct ia_css_isys_stream_cfg_data *stream_cfg
)
{
	bool rc;
	ia_css_isys_descr_t			isys_stream_descr;
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	ia_css_isyspoc_cmd_msg_t isys_msg;
	int retval = -1;

	assert(stream_handle < STREAM_ID_MAX);

	assert(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_IDLE);

	assert(stream_cfg != NULL);

	/* Currently, we only support frame buffers */
	assert(stream_cfg->nof_output_pins == ISYSPOC_OUTPUT_PIN_MAX);

	/* initialize the pending frames array */
	retval = isyspoc_init_pending_frame(ctx, stream_handle);
	assert(retval == 0);

	/* Initialize the message to 0*/
	memset(&isys_msg, 0, sizeof(isys_msg));

	/* validate stream's source*/
	assert(stream_cfg->src < N_IA_CSS_ISYS_STREAM_SRC);

	if (stream_cfg->src >= N_IA_CSS_ISYS_STREAM_SRC) {
		return EINVAL;
	}

	/* Note: This should be done first.
	 * as it will overwrite entire contents of isys_stream_descr*/
	memcpy(&isys_stream_descr,
		&(ctx->port_cfg[stream_cfg->src]),
		sizeof(input_system_cfg_t));

	rc = ia_css_isys_translate_stream_cfg_to_isys_stream_descr(
				stream_cfg,
				&isys_stream_descr,
				ISYS_IP_PIN0);
	if(true != rc)
		return 1;      /*This will be changed to an appropriate error code*/

	/* create the virtual Input System (2401) */
	rc =  ia_css_isys_stream_create(
			&(isys_stream_descr),
			&(ctx->virtual_input_system[stream_handle]));
	if (rc != true)
		return 1;      /*This will be changed to an appropriate error code*/

	/* calculate the configuration of the virtual Input System (2401) */
	memset(&isys_msg.payload.virtual_input_system_cfg, 0, sizeof(ia_css_isys_stream_cfg_t));
	rc = ia_css_isys_stream_calculate_cfg(
			&(ctx->virtual_input_system[stream_handle]),
			&(isys_stream_descr),
			&(isys_msg.payload.virtual_input_system_cfg));
	if (rc != true) {
		ia_css_isys_stream_destroy(&(ctx->virtual_input_system[stream_handle]));
		return 1;      /*This will be changed to an appropriate error code*/
	}

	isys_msg.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_OPEN;
	isys_msg.stream_handle = stream_handle;
	isys_msg.ret = 0;
	isys_msg.virtual_input_system = ctx->virtual_input_system[stream_handle];
	isys_msg.num_child_msgs = 0;
	isyspoc_dump_virtual_isys_handle(&isys_msg.virtual_input_system);
	isyspoc_dump_isys_stream_cfg(&isys_msg.payload.virtual_input_system_cfg);

	if ((retval =  ia_css_fwctrl_isys_stream_send_msg(stream_handle, &isys_msg)) == 0) {
		ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_OPENED;
		ctx->stream_nof_output_pins[stream_handle] = stream_cfg->nof_output_pins;
	}

	return retval;
}

/**
 * ia_css_isys_stream_close() - close virtual stream
 */
int ia_css_isys_stream_close(
	HANDLE context,
	unsigned int stream_handle
)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	ia_css_isyspoc_cmd_msg_t isys_msg;
	int retval = 0;

	assert(stream_handle < STREAM_ID_MAX);
	assert(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_OPENED);

	/* Initialize the message to 0*/
	memset(&isys_msg, 0, sizeof(isys_msg));

	isys_msg.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_CLOSE;
	isys_msg.stream_handle = stream_handle;
	isys_msg.ret = 0;
	isys_msg.virtual_input_system = ctx->virtual_input_system[stream_handle];
	isys_msg.num_child_msgs = 0;
	isyspoc_dump_virtual_isys_handle(&isys_msg.virtual_input_system);

	if ((retval =  ia_css_fwctrl_isys_stream_send_msg(stream_handle, &isys_msg)) == 0) {
		ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_IDLE;
		ctx->stream_nof_output_pins[stream_handle] = 0;
	}

	ia_css_isys_stream_destroy(&(ctx->virtual_input_system[stream_handle]));

	return retval;
}

/**
 * ia_css_isys_stream_start() - starts handling a mipi virtual stream
 */
int ia_css_isys_stream_start(
		HANDLE context,
		unsigned int stream_handle,
		const struct ia_css_isys_frame_buff_set *next_frame
		)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	ia_css_isyspoc_cmd_msg_t isys_msg;
	unsigned int i;
	int retval = 0;

	assert(stream_handle < STREAM_ID_MAX);
	assert(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_OPENED);

	if (next_frame != NULL) {
		assert(ctx->stream_nof_output_pins[stream_handle] < MAX_OPINS);
		for (i=0; i<ctx->stream_nof_output_pins[stream_handle]; i++) {
			/* Verify output pin */
			if (0 != isyspoc_validate_output_pin(&next_frame->output_pins[i])) {
				ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
					"ia_css_isys_stream_start(): invalid outputr_pin %d\n",
					i);
				return EINVAL;
			}
		}
	}

	/* Initialize the message to 0*/
	memset(&isys_msg, 0, sizeof(isys_msg));

	isys_msg.stream_handle = stream_handle;
	isys_msg.ret = 0;
	isys_msg.virtual_input_system = ctx->virtual_input_system[stream_handle];
	isyspoc_dump_virtual_isys_handle(&isys_msg.virtual_input_system);

	if (next_frame != NULL) {
		uint32_t buf_id;
		isys_msg.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_START_AND_CAPTURE;

		/* Add the frameset to the pending frames*/
		retval = isyspoc_add_to_pending_frame(ctx,
				stream_handle, next_frame, &buf_id);
		if (0 != retval) {
			/* Adding to pending frames failed */
			return retval;
		}

		/* Update the cmd*/
		isyspoc_update_isys_buf(buf_id, next_frame,
			&isys_msg.payload.isys_output);

		isys_msg.num_child_msgs = MIN_ISYS_CAPTURE_RESP + ctx->stream_nof_output_pins[stream_handle] - 1;
	}
	else {
		isys_msg.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_START;
		isys_msg.num_child_msgs = 0;
	}

	if ((retval =  ia_css_fwctrl_isys_stream_send_msg(stream_handle, &isys_msg)) == 0) {
        	ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_STARTED;
	}

	return retval;
}

/**
 * ia_css_isys_stream_stop() - Stops a mipi virtual stream
 */
  int ia_css_isys_stream_stop(
	HANDLE context,
	unsigned int stream_handle
)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	ia_css_isyspoc_cmd_msg_t isys_msg;
	int retval = 0;

	assert(stream_handle < STREAM_ID_MAX);
	assert(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_STARTED);

	/* Initialize the message to 0*/
	memset(&isys_msg, 0, sizeof(isys_msg));

	isys_msg.stream_handle = stream_handle;
	isys_msg.ret = 0;
	isys_msg.virtual_input_system = ctx->virtual_input_system[stream_handle];
	isys_msg.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_STOP;
	isys_msg.num_child_msgs = 0;
	isyspoc_dump_virtual_isys_handle(&isys_msg.virtual_input_system);

	if ((retval =  ia_css_fwctrl_isys_stream_send_msg(stream_handle, &isys_msg)) == 0) {
		ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_OPENED;
	}

	return retval;
}

/**
 * ia_css_isys_stream_flush() - stops a mipi virtual stream but completes processing cmd backlog
 */
 int ia_css_isys_stream_flush(
	HANDLE context,
	unsigned int stream_handle
)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	ia_css_isyspoc_cmd_msg_t isys_msg;
	int retval = 0;

	assert(stream_handle < STREAM_ID_MAX);
	assert(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_STARTED);

	/* Initialize the message to 0*/
	memset(&isys_msg, 0, sizeof(isys_msg));

	isys_msg.stream_handle = stream_handle;
	isys_msg.ret = 0;
	isys_msg.virtual_input_system = ctx->virtual_input_system[stream_handle];
	isys_msg.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_FLUSH;
	isys_msg.num_child_msgs = 0;
	isyspoc_dump_virtual_isys_handle(&isys_msg.virtual_input_system);

	if ((retval =  ia_css_fwctrl_isys_stream_send_msg(stream_handle, &isys_msg)) == 0) {
		ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_OPENED;
	}

	return retval;
}

/**
 * ia_css_isys_stream_capture_indication() - captures "next frame" on stream_handle
 */
int ia_css_isys_stream_capture_indication(
	HANDLE context,
	unsigned int stream_handle,
	const struct ia_css_isys_frame_buff_set *next_frame
)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	ia_css_isyspoc_cmd_msg_t isys_msg;
	unsigned int i;
	int retval = 0;
	uint32_t buf_id;

	assert(stream_handle < STREAM_ID_MAX);
	assert(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_STARTED);

	assert(next_frame != NULL);
	assert(ctx->stream_nof_output_pins[stream_handle] < MAX_OPINS);
	for (i=0; i<ctx->stream_nof_output_pins[stream_handle]; i++) {
		/* Verify output pin */
		if (0 != isyspoc_validate_output_pin(&next_frame->output_pins[i])) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
				"ia_css_isys_stream_capture_indication(): invalid output_pin %d\n",
				i);
			return EINVAL;
		}
	}

	/* Initialize the message to 0*/
	memset(&isys_msg, 0, sizeof(isys_msg));

	isys_msg.stream_handle = stream_handle;
	isys_msg.ret = 0;
	isys_msg.virtual_input_system = ctx->virtual_input_system[stream_handle];
	isys_msg.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_CAPTURE;

	/* Add the frameset to the pending frames*/
	retval = isyspoc_add_to_pending_frame(ctx,
			stream_handle, next_frame, &buf_id);
	if (0 != retval) {
		/* Adding to pending frames failed */
		return retval;
	}

	/* Update the cmd*/
	isyspoc_update_isys_buf(buf_id, next_frame,
		&isys_msg.payload.isys_output);

	isys_msg.num_child_msgs = MIN_ISYS_CAPTURE_RESP + ctx->stream_nof_output_pins[stream_handle] - 1;

	isyspoc_dump_virtual_isys_handle(&isys_msg.virtual_input_system);

	retval =  ia_css_fwctrl_isys_stream_send_msg(stream_handle, &isys_msg);

	return retval;
}

/**
 * ia_css_isys_stream_handle_response() - handle ISYS responses
 */
 int ia_css_isys_stream_handle_response(
	HANDLE context,
	struct ia_css_isys_resp_info *received_response
)
{
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	ia_css_isyspoc_cmd_msg_t isys_msg;
	int ret = 0;

	assert(received_response != NULL);
	assert(ctx != NULL);

	do {
		ret =  ia_css_fwctrl_isys_receive_msg(&isys_msg);
		if (ret != 0)
			return ret;
	} while(isys_msg.resp_type == N_IA_CSS_ISYS_RESP_TYPE);

	if (0 == ret) {
		ret = isyspoc_create_reponse(ctx, received_response, &isys_msg);
	}

	return ret;
}

int ia_css_isys_device_close(
	HANDLE context,
	unsigned int nof_streams
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	unsigned int i;
	int ret = 0;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_isys_device_close() enter: void\n");

	for (i=0; i < nof_streams; i++) {
		if (ctx->stream_state_array[i] != IA_CSS_ISYS_STREAM_STATE_IDLE) {
			return EPERM;
		}
	}

	ret = ia_css_fwctrl_device_close();
	HOST_FREE(context);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_isys_device_close() return: return_err=%d\n", ret);
	return ret;
}

static void isyspoc_update_isys_buf(
	const uint32_t buf_id,
	const struct ia_css_isys_frame_buff_set *next_frame,
	ia_css_isyspoc_buffer *isys_buf)
{
	assert(next_frame);
	assert(isys_buf);

	isys_buf->buf_id = buf_id;
	isys_buf->buf_addr = next_frame->output_pins[ISYSPOC_OUTPUT_PIN_MAX-1].payload.addr;
}


static int isyspoc_validate_output_pin(const struct ia_css_isys_output_pin *output_pin)
{
	assert(output_pin);

	/* Verify output pin */
	if (output_pin->info.pt >= N_IA_CSS_ISYS_PIN_TYPE) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"isyspoc_validate_output_pin(): invalid pintype %d\n",
			output_pin->info.pt);
		return EINVAL;
	}

	if (output_pin->info.pt == IA_CSS_ISYS_PIN_TYPE_MIPI) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"isyspoc_validate_output_pin(): mipi pintype not supported\n");
		return EINVAL;
	}

	if (output_pin->info.pt == IA_CSS_ISYS_PIN_TYPE_RAW_S) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"isyspoc_validate_output_pin(): raws pintype not supported\n");
		return EINVAL;
	}

	if (output_pin->info.pt == IA_CSS_ISYS_PIN_TYPE_RAW_NS) {
		if (output_pin->info.type_specifics.ft >= N_IA_CSS_ISYS_FRAME_FORMAT) {
			ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
				"isyspoc_validate_output_pin(): invalid frametype %d\n",
				output_pin->info.type_specifics.ft);
			return EINVAL;
		}
	}

	if ((output_pin->info.output_res.width < output_min_width)
	     && (output_pin->info.output_res.width > output_max_width)) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"isyspoc_validate_output_pin(): unsupported width=%d\n",
			output_pin->info.output_res.width);
	}

	if ((output_pin->info.output_res.height < output_min_height)
	     && (output_pin->info.output_res.height > output_max_height)) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"isyspoc_validate_output_pin(): unsupported res height=%d\n",
			output_pin->info.output_res.height);
		return EINVAL;
	}

	if(output_pin->payload.addr == (hrt_vaddress)NULL) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR,
			"isyspoc_validate_output_pin(): invalid payload.addr\n");
		return EINVAL;
	}

	return 0;
}

static int isyspoc_create_reponse(
	struct ia_css_isys_context *ctx,
	struct ia_css_isys_resp_info *received_response,
	ia_css_isyspoc_cmd_msg_t *isys_msg
) {
	int ret = 0;
	struct ia_css_isys_frame_buff_set frame_set;

	received_response->error = isys_msg->ret;
	received_response->timestamp[0] = isys_msg->timestamp[0];
	received_response->timestamp[1] = isys_msg->timestamp[1];
	received_response->stream_handle = isys_msg->stream_handle;
	received_response->type = isys_msg->resp_type;
	isyspoc_dump_virtual_isys_handle(&isys_msg->virtual_input_system);

	switch(isys_msg->resp_type) {
		case IA_CSS_ISYS_RESP_TYPE_STREAM_START_ACK:
		case IA_CSS_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_ACK:
			ia_css_debug_dump_isys_state();
			break;

		case IA_CSS_ISYS_RESP_TYPE_STREAM_OPEN_DONE:
		case IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_ACK:
		case IA_CSS_ISYS_RESP_TYPE_STREAM_STOP_ACK:
		case IA_CSS_ISYS_RESP_TYPE_STREAM_FLUSH_ACK:
		case IA_CSS_ISYS_RESP_TYPE_STREAM_CLOSE_ACK:

			break;


		case IA_CSS_ISYS_RESP_TYPE_PIN_DATA_READY:
		case IA_CSS_ISYS_RESP_TYPE_PIN_DATA_WATERMARK:
		case IA_CSS_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE:
		case IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE:
			/* Get the frame buffer from index*/
			ret = isyspoc_get_pending_frame(ctx,
				isys_msg->stream_handle,
				isys_msg->payload.isys_output.buf_id,
				&frame_set);
			assert(ret==0);

			received_response->resp_data.pin = frame_set.output_pins[ISYSPOC_OUTPUT_PIN_MAX-1];
			if ((isys_msg->resp_type == IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE)
				|| (isys_msg->resp_type == IA_CSS_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE)) {
				/*Remove the pending frame*/
				ret = isyspoc_remove_from_pending_frame(ctx,
					isys_msg->stream_handle,
					isys_msg->payload.isys_output.buf_id);
				assert(ret==0);
			}
			break;

		/* Need to find out if these events are handled
		 in software for CHV 2401 system */
		case IA_CSS_ISYS_RESP_TYPE_FRAME_SOF:
		case IA_CSS_ISYS_RESP_TYPE_FRAME_EOF:

			break;

		default:
			ret = -1;
			break;
	}

	return ret;
}

static void isyspoc_dump_isys_stream_cfg(const ia_css_isys_stream_cfg_t *isys_stream_cfg)
{
	const input_system_input_port_cfg_t *input_port_cfg;
	const input_system_channel_cfg_t *channel_cfg;
	if (NULL == isys_stream_cfg) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR, "isyspoc_dump_isys_stream_cfg(): Invalid argument");
		return;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"isys_stream_cfg: addr=0x%x enable_metadata=%d valid=%d\n",
		isys_stream_cfg, isys_stream_cfg->enable_metadata, isys_stream_cfg->valid);

	input_port_cfg = &isys_stream_cfg->input_port_cfg;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"input_port_cfg: csi_rx.fe.active_lanes=%d csi_rx.be.lut_entry.lpe=%d "
		"csi_rx.be.lut_entry.spe=%d csi_rx.be.csi_mipi_packet_type=%d "
		"csi_rx.be.mipi_cfg.comp_enable=%d csi_rx.be.mipi_cfg.virtual_channel=%d "
		"csi_rx.be.mipi_cfg.data_type=%d csi_rx.be.mipi_cfg.comp_scheme=%d "
		"csi_rx.be.mipi_cfg.comp_predictor=%d csi_rx.be.mipi_cfg.comp_bit_idx=%d\n",
		input_port_cfg->csi_rx_cfg.frontend_cfg.active_lanes,
		input_port_cfg->csi_rx_cfg.backend_cfg.lut_entry.long_packet_entry,
		input_port_cfg->csi_rx_cfg.backend_cfg.lut_entry.short_packet_entry,
		input_port_cfg->csi_rx_cfg.backend_cfg.csi_mipi_packet_type,
		input_port_cfg->csi_rx_cfg.backend_cfg.csi_mipi_cfg.comp_enable,
		input_port_cfg->csi_rx_cfg.backend_cfg.csi_mipi_cfg.virtual_channel,
		input_port_cfg->csi_rx_cfg.backend_cfg.csi_mipi_cfg.data_type,
		input_port_cfg->csi_rx_cfg.backend_cfg.csi_mipi_cfg.comp_scheme,
		input_port_cfg->csi_rx_cfg.backend_cfg.csi_mipi_cfg.comp_predictor,
		input_port_cfg->csi_rx_cfg.backend_cfg.csi_mipi_cfg.comp_bit_idx);

	channel_cfg = &isys_stream_cfg->channel_cfg;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"channel_cfg: mmio.bits_per_pixel=%d mmio.enable_blocking=%d "
		"ibuf.online=%d ibuf.dma_cfg.channel=%d ibuf.dma_cfg.cmd=%d "
		"ibuf.dma_cfg.shift_returned_items=%d "
		"ibuf.dma_cfg.elems_per_word_in_ibuf=%d ibuf.dma_cfg.elems_per_word_in_dest=%d "
		"ibuf.ib_buffer.start_addr=%d ibuf.ib_buffer.stride=%d ibuf.ib_buffer.lines=%d "
		"ibuf.dest_buf_cfg.stride=%d ibuf.dest_buf_cfg.start_addr=%d "
		"ibuf.dest_buf_cfg.lines=%d ibuf.items_per_store=%d ibuf.stores_per_frame=%d "
		"ibuf.stream2mmio_cfg.sync_cmd=%d ibuf.stream2mmio_cfg.store_cmd=%d "
		"dma_cfg.channel=%d dma_cfg.connection=%d dma_cfg.extension=%d dma_cfg.height=%d "
		"dma_src_port_cfg.stride=%d dma_src_port_cfg.elements=%d dma_src_port_cfg.cropping=%d "
		"dma_src_port_cfg.width=%d dma_dest_port_cfg.stride=%d dma_dest_port_cfg.elements=%d "
		"dma_dest_port_cfg.cropping=%d dma_dest_port_cfg.width=%d\n",
		channel_cfg->stream2mmio_cfg.bits_per_pixel, channel_cfg->stream2mmio_cfg.enable_blocking,
		channel_cfg->ibuf_ctrl_cfg.online, channel_cfg->ibuf_ctrl_cfg.dma_cfg.channel,
		channel_cfg->ibuf_ctrl_cfg.dma_cfg.cmd, channel_cfg->ibuf_ctrl_cfg.dma_cfg.shift_returned_items,
		channel_cfg->ibuf_ctrl_cfg.dma_cfg.elems_per_word_in_ibuf,
		channel_cfg->ibuf_ctrl_cfg.dma_cfg.elems_per_word_in_dest,
		channel_cfg->ibuf_ctrl_cfg.ib_buffer.start_addr, channel_cfg->ibuf_ctrl_cfg.ib_buffer.stride,
		channel_cfg->ibuf_ctrl_cfg.ib_buffer.lines, channel_cfg->ibuf_ctrl_cfg.dest_buf_cfg.stride,
		channel_cfg->ibuf_ctrl_cfg.dest_buf_cfg.start_addr, channel_cfg->ibuf_ctrl_cfg.dest_buf_cfg.lines,
		channel_cfg->ibuf_ctrl_cfg.items_per_store, channel_cfg->ibuf_ctrl_cfg.stores_per_frame,
		channel_cfg->ibuf_ctrl_cfg.stream2mmio_cfg.sync_cmd,
		channel_cfg->ibuf_ctrl_cfg.stream2mmio_cfg.store_cmd, channel_cfg->dma_cfg.channel,
		channel_cfg->dma_cfg.connection, channel_cfg->dma_cfg.extension, channel_cfg->dma_cfg.height,
		channel_cfg->dma_src_port_cfg.stride, channel_cfg->dma_src_port_cfg.elements,
		channel_cfg->dma_src_port_cfg.cropping, channel_cfg->dma_src_port_cfg.width,
		channel_cfg->dma_dest_port_cfg.stride, channel_cfg->dma_dest_port_cfg.elements,
		channel_cfg->dma_dest_port_cfg.cropping, channel_cfg->dma_dest_port_cfg.width);
}
static void isyspoc_dump_virtual_isys_handle(const virtual_input_system_stream_t *visys)
{
	const input_system_input_port_t *input_port;
	const input_system_channel_t *channel;
	if (NULL == visys) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_ERROR, "isyspoc_dump_virtual_isys_handle(): Invalid argument");
		return;
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"visys: addr=0x%x enable_metadata=%d online=%d linked_isys_stream_id=%d valid=%d\n",
		visys, visys->enable_metadata, visys->online, visys->linked_isys_stream_id, visys->valid);

	/* dump the input_port */
	input_port=&visys->input_port;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"input_port: src_type=%d pixelgen.id=%d "
		"csi_rx.fe_id=%d csi_rx.be_id=%d csi_rx.packet_type=%d "
		"csi_rx.be_lut.lpe=0x%x csi_rx.be_lut.spe=0x%x\n",
		input_port->source_type, input_port->pixelgen.pixelgen_id,
		input_port->csi_rx.frontend_id, input_port->csi_rx.backend_id,
		input_port->csi_rx.packet_type,
		input_port->csi_rx.backend_lut_entry.long_packet_entry,
		input_port->csi_rx.backend_lut_entry.short_packet_entry);

	/* dump the channel*/
	channel = &visys->channel;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"channel: mmio_id=%d mmio_sid_id=%d ibuf_ctrl_id=%d "
		"ibuf.addr=0x%x ibuf.stride=%d ibuf.lines=%d "
		"dma_id=%d dma_chan=%d\n",
		channel->stream2mmio_id, channel->stream2mmio_sid_id,
		channel->ibuf_ctrl_id, channel->ib_buffer.start_addr,
		channel->ib_buffer.stride, channel->ib_buffer.lines,
		channel->dma_id, channel->dma_channel);
}

#if (ISYSPOC_HW_DEBUG)
static void isyspoc_dump_sp_stream_state(unsigned int stream_handle)
{
	if (INPUT_SYSTEM_N_STREAM_ID >= stream_handle) {
		unsigned int offset = stream_handle*sizeof(ia_css_isys_stream_holder_t);
		ia_css_isys_stream_holder_t isys_stream_state;
		sp_dmem_load(SP0_ID,
				(unsigned int) HIVE_ADDR_ia_css_isys_stream_container + offset,
				&isys_stream_state, sizeof(ia_css_isys_stream_holder_t));

		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
				"sizeof(cmd)=%d sizeof(virtual_input_system_stream_t)=%d "
				"sizeof(ia_css_isys_stream_cfg_t)=%d sizeof(ia_css_isys_frame_buff_set)=%d\n",
				sizeof(ia_css_isyspoc_cmd_msg_t),
				sizeof(virtual_input_system_stream_t),
				sizeof(ia_css_isys_stream_cfg_t),
				sizeof(struct ia_css_isys_frame_buff_set));

		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
				"sp.stream_handle=%d sp.stream.state=%d "
				"sp.stream.msg_addr=%x sp.stream.msg_cmd=%d sp.stream.msg_resp=%d "
				"sp.stream.stream_num=%d sp.stream.err=%d\n",
				stream_handle, isys_stream_state.state,
				isys_stream_state.msg_addr, isys_stream_state.msg.send_type,
				isys_stream_state.msg.resp_type,
				isys_stream_state.msg.stream_handle,
				isys_stream_state.msg.ret);

		isyspoc_dump_virtual_isys_handle(&isys_stream_state.msg.virtual_input_system);
		isyspoc_dump_isys_stream_cfg(&isys_stream_state.msg.payload.virtual_input_system_cfg);
	}
}
#endif /*ISYSPOC_HW_DEBUG*/

#undef HOST_MALLOC
#undef HOST_FREE
