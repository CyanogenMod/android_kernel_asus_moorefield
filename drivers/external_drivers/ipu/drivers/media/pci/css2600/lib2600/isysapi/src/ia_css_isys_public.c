

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


/* The FW bridged types are included through the following */
#include "ia_css_isysapi.h"
/* The following provides the isys-sys context */
#include "ia_css_isys_private.h"
/* The following provides the sys layer functions */
#include "ia_css_syscom.h"

#include "ia_css_input_buffer_cpu.h"
/* The following is needed for the
 * stddef.h (NULL),
 * limits.h (CHAR_BIT definition).
 */
#include "type_support.h"
#include "error_support.h"
#include "assert_support.h"
#include "cpu_mem_support.h"


/**
 * ia_css_isys_device_open() - open and configure ISYS device
 */
int ia_css_isys_device_open(
	HANDLE *context,
	struct ia_css_isys_device_cfg_data *config
) {
	unsigned int stream_handle;
	int retval = 0;
	struct ia_css_isys_context *ctx;
	struct ia_css_syscom_config sys;
	struct is_css_isys_fw_config isys_fw_cfg;

	verifret(config != NULL, EFAULT);

	verifret(config->mipi.nof_blocks <= STREAM_ID_MAX, EINVAL);
	for (stream_handle = 0; stream_handle < config->mipi.nof_blocks; stream_handle++) {
		verifret(config->mipi.block_size[stream_handle] > 0, EINVAL);
	}

	verifret(config->pixel.nof_blocks <= STREAM_ID_MAX, EINVAL);
	for (stream_handle = 0; stream_handle < config->pixel.nof_blocks; stream_handle++) {
		verifret(config->pixel.block_size[stream_handle] > 0, EINVAL);
	}

	ctx = (struct ia_css_isys_context *)ia_css_cpu_mem_alloc(sizeof(struct ia_css_isys_context));
	verifret(ctx != NULL, EFAULT);
	*context = (HANDLE)ctx;

	for (stream_handle = 0; stream_handle < STREAM_ID_MAX; stream_handle++) {
		ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_IDLE;
		ctx->stream_nof_output_pins[stream_handle] = 0;
	}

	/* Copy to the sys config the driver_sys config, and add the internal info (token sizes) */
	sys.num_input_queues = config->driver_sys.num_send_queues;
	sys.num_output_queues = config->driver_sys.num_recv_queues;
	sys.input_queue_size = config->driver_sys.send_queue_size;
	sys.output_queue_size = config->driver_sys.recv_queue_size;
	sys.input_token_size = sizeof(struct send_queue_token);
	sys.output_token_size = sizeof(struct resp_queue_token);

	/* Prepare the param */
	retval = ia_css_isys_prepare_param(&isys_fw_cfg, &config->mipi, &config->pixel);
	verifret(retval == 0, retval);

	sys.specific_addr = &isys_fw_cfg;        /* parameter struct to be passed to fw */
	sys.specific_size = sizeof(isys_fw_cfg); /* parameters size */

	/* The allocation of the queues will take place within this call and info will be stored in sys_context output */
	ctx->sys = ia_css_syscom_open(&sys);
	verifret(ctx->sys, EFAULT);

#if (VERIFY_DEVSTATE != 0)
	ctx->dev_state = IA_CSS_ISYS_DEVICE_STATE_CONFIGURED;
#endif /* VERIFY_DEVSTATE */

	return 0;
}


 /**
 * ia_css_isys_stream_open() - open and configure a virtual stream
 */
 int ia_css_isys_stream_open(
	HANDLE context,
	unsigned int stream_handle,
	const struct ia_css_isys_stream_cfg_data *stream_cfg
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	unsigned int i;
	int retval = 0;
	unsigned int packets;
	struct send_queue_token token;
	ia_css_input_buffer_css_address stream_cfg_fw = 0;
	ia_css_input_buffer buf_stream_cfg_id = (ia_css_input_buffer)NULL;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	verifret(stream_handle < STREAM_ID_MAX, EINVAL);

	verifret(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_IDLE, EPERM);

	verifret(stream_cfg != NULL, EFAULT);
	verifret(stream_cfg->src < N_IA_CSS_ISYS_STREAM_SRC, EINVAL);
	verifret(stream_cfg->vc < N_IA_CSS_ISYS_MIPI_VC, EINVAL);
	verifret(stream_cfg->nof_input_pins < MAX_IPINS, EINVAL);
	for (i=0; i<stream_cfg->nof_input_pins; i++) {
		/* Verify input pin */
		verifret(stream_cfg->input_pins[i].dt < N_IA_CSS_ISYS_MIPI_DATA_TYPE, EINVAL);
		verifret(
			stream_cfg->input_pins[i].input_res.width >= input_mipi_min_width &&
			stream_cfg->input_pins[i].input_res.width <= input_mipi_max_width &&
			stream_cfg->input_pins[i].input_res.height >= input_mipi_min_height &&
			stream_cfg->input_pins[i].input_res.height <= input_mipi_max_height, EINVAL);
		verifret(
			stream_cfg->input_pins[i].crop.right_offset - stream_cfg->input_pins[i].crop.left_offset <= stream_cfg->input_pins[i].input_res.width &&
			stream_cfg->input_pins[i].crop.bottom_offset - stream_cfg->input_pins[i].crop.top_offset <= stream_cfg->input_pins[i].input_res.height, EINVAL);
	}
	verifret(stream_cfg->nof_output_pins < MAX_OPINS, EINVAL);
	for (i=0; i<stream_cfg->nof_output_pins; i++) {
		/* Verify output pin */
		verifret(stream_cfg->output_pins[i].pt < N_IA_CSS_ISYS_PIN_TYPE, EINVAL);
		switch(stream_cfg->output_pins[i].pt) {
		case IA_CSS_ISYS_PIN_TYPE_MIPI:
			verifret(stream_cfg->output_pins[i].type_specifics.dt < N_IA_CSS_ISYS_MIPI_DATA_TYPE, EINVAL);
			break;
		case IA_CSS_ISYS_PIN_TYPE_RAW_NS:
		case IA_CSS_ISYS_PIN_TYPE_RAW_S:
			verifret(stream_cfg->output_pins[i].type_specifics.ft < N_IA_CSS_ISYS_FRAME_FORMAT, EINVAL);
			break;
		default:
			break;
		}
		verifret(
			stream_cfg->output_pins[i].output_res.width >= output_min_width &&
			stream_cfg->output_pins[i].output_res.width <= output_max_width &&
			stream_cfg->output_pins[i].output_res.height >= output_min_height &&
			stream_cfg->output_pins[i].output_res.height <= output_max_height, EINVAL);
	}

	/* open 1 send queue/stream and a single receive queue if not existing */
	retval = ia_css_syscom_send_port_open(ctx->sys, stream_handle);
	verifret(retval == 0, retval);
	for (i=0; i < STREAM_ID_MAX; i++) {
		if (ctx->stream_state_array[i] != IA_CSS_ISYS_STREAM_STATE_IDLE) {
			break;
		}
	}
	if (i == STREAM_ID_MAX) {
		retval = ia_css_syscom_recv_port_open(ctx->sys, 0);
		verifret(retval == 0, retval);
	}

	//assert((ia_css_syscom_send_port_available(ctx->sys, stream_handle, &packets) == 0) && packets > 0);
	retval = ia_css_syscom_send_port_available(ctx->sys, stream_handle, &packets);
	verifret(retval == 0, retval);
	verifret(packets > 0, EPERM);
	token.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_OPEN;
	retval = ia_css_isys_constr_fw_stream_cfg(&stream_cfg_fw, &buf_stream_cfg_id, stream_cfg);
	verifret(retval == 0, retval);
	assert(stream_cfg_fw != 0);
	token.payload = stream_cfg_fw;
	token.buf_handle = HOST_ADDRESS(buf_stream_cfg_id);
	retval = ia_css_syscom_send_port_transfer(ctx->sys, stream_handle, &token);
	verifret(retval == 0, retval);

	ctx->stream_nof_output_pins[stream_handle] = stream_cfg->nof_output_pins;
	ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_OPENED;

	return 0;
}


/**
 * ia_css_isys_stream_close() - close virtual stream
 */
int ia_css_isys_stream_close(
	HANDLE context,
	unsigned int stream_handle
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	int retval = 0;
	unsigned int i;
	unsigned int packets;
	struct send_queue_token token;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	verifret(stream_handle < STREAM_ID_MAX, EINVAL);
	verifret(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_OPENED, EPERM);

	retval = ia_css_syscom_send_port_available(ctx->sys, stream_handle, &packets);
	verifret(retval == 0, retval);
	verifret(packets > 0, EPERM);
	token.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_CLOSE;
	token.payload = 0;
	token.buf_handle = 0;
	retval = ia_css_syscom_send_port_transfer(ctx->sys, stream_handle, &token);
	verifret(retval == 0, retval);

	/* close 1 send queue/stream and the single receive queue if none is using it */
	retval = ia_css_syscom_send_port_close(ctx->sys,stream_handle);
	verifret(retval == 0, retval);
	for (i=0; i < STREAM_ID_MAX; i++) {
		if (ctx->stream_state_array[i] != IA_CSS_ISYS_STREAM_STATE_IDLE) {
			break;
		}
	}
	if (i == STREAM_ID_MAX) {
		retval = ia_css_syscom_recv_port_close(ctx->sys, 0);
		verifret(retval == 0, retval);
	}

	ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_CLOSED;
	ctx->stream_nof_output_pins[stream_handle] = 0;

	return 0;
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
	unsigned int i;
	int retval = 0;
	unsigned int packets;
	struct send_queue_token token;
	ia_css_input_buffer_css_address next_frame_fw = 0;
	ia_css_input_buffer buf_next_frame_id = (ia_css_input_buffer)NULL;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	verifret(stream_handle < STREAM_ID_MAX, EINVAL);
	verifret(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_OPENED, EPERM);

	if (next_frame != NULL) {
		assert(ctx->stream_nof_output_pins[stream_handle] < MAX_OPINS);
		for (i=0; i<ctx->stream_nof_output_pins[stream_handle]; i++) {
			/* Verify output pin */
			verifret(next_frame->output_pins[i].info.pt < N_IA_CSS_ISYS_PIN_TYPE, EINVAL);
			switch(next_frame->output_pins[i].info.pt) {
			case IA_CSS_ISYS_PIN_TYPE_MIPI:
				verifret(next_frame->output_pins[i].info.type_specifics.dt < N_IA_CSS_ISYS_MIPI_DATA_TYPE, EINVAL);
				break;
			case IA_CSS_ISYS_PIN_TYPE_RAW_NS:
			case IA_CSS_ISYS_PIN_TYPE_RAW_S:
				verifret(next_frame->output_pins[i].info.type_specifics.ft < N_IA_CSS_ISYS_FRAME_FORMAT, EINVAL);
				break;
			default:
				break;
			}
			verifret(
				next_frame->output_pins[i].info.output_res.width >= output_min_width &&
				next_frame->output_pins[i].info.output_res.width <= output_max_width &&
				next_frame->output_pins[i].info.output_res.height >= output_min_height &&
				next_frame->output_pins[i].info.output_res.height <= output_max_height, EINVAL);
			verifret(next_frame->output_pins[i].payload.addr != 0, EFAULT);
		}
		verifret(next_frame->input_pin_set.lsc_param.addr != 0, EFAULT);
		verifret(next_frame->input_pin_set.dpc_param.addr != 0, EFAULT);
		verifret(next_frame->input_pin_set.blc_param.addr != 0, EFAULT);
	}

	retval = ia_css_syscom_send_port_available(ctx->sys, stream_handle, &packets);
	verifret(retval == 0, retval);
	verifret(packets > 0, EPERM);
	if (next_frame != NULL) {
		token.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_START_AND_CAPTURE;
		retval = ia_css_isys_constr_fw_next_frame(&next_frame_fw, &buf_next_frame_id, next_frame);
		verifret(retval == 0, retval);
		assert(next_frame_fw != 0);
		token.payload = next_frame_fw;
		token.buf_handle = HOST_ADDRESS(buf_next_frame_id);
	}
	else {
		token.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_START;
		token.payload = 0;
		token.buf_handle = 0;
	}
	retval = ia_css_syscom_send_port_transfer(ctx->sys, stream_handle, &token);
	verifret(retval == 0, retval);

	ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_STARTED;

	return 0;
}


/**
 * ia_css_isys_stream_stop() - Stops a mipi virtual stream
 */
  int ia_css_isys_stream_stop(
	HANDLE context,
	unsigned int stream_handle
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	int retval = 0;
	unsigned int packets;
	struct send_queue_token token;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	verifret(stream_handle < STREAM_ID_MAX, EINVAL);
	verifret(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_STARTED, EPERM);

	retval = ia_css_syscom_send_port_available(ctx->sys, stream_handle, &packets);
	verifret(retval == 0, retval);
	verifret(packets > 0, EPERM);
	token.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_STOP;	/* In practice this will go to different queue or direct link */
	token.payload = 0;
	token.buf_handle = 0;
	retval = ia_css_syscom_send_port_transfer(ctx->sys, stream_handle, &token);
	verifret(retval == 0, retval);

	ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_OPENED;

	return 0;
}


/**
 * ia_css_isys_stream_flush() - stops a mipi virtual stream but completes processing cmd backlog
 */
 int ia_css_isys_stream_flush(
	HANDLE context,
	unsigned int stream_handle
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	int retval = 0;
	unsigned int packets;
	struct send_queue_token token;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	verifret(stream_handle < STREAM_ID_MAX, EINVAL);
	verifret(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_STARTED, EPERM);

	retval = ia_css_syscom_send_port_available(ctx->sys, stream_handle, &packets);
	verifret(retval == 0, retval);
	verifret(packets > 0, EPERM);
	token.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_FLUSH;
	token.payload = 0;
	token.buf_handle = 0;

	retval = ia_css_syscom_send_port_transfer(ctx->sys, stream_handle, &token);
	verifret(retval == 0, retval);

	ctx->stream_state_array[stream_handle] = IA_CSS_ISYS_STREAM_STATE_OPENED;

	return 0;
}


/**
 * ia_css_isys_stream_capture_indication() - captures "next frame" on stream_handle
 */
int ia_css_isys_stream_capture_indication(
	HANDLE context,
	unsigned int stream_handle,
	const struct ia_css_isys_frame_buff_set *next_frame
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	unsigned int i;
	int retval = 0;
	unsigned int packets;
	struct send_queue_token token;
	ia_css_input_buffer_css_address next_frame_fw = 0;
	ia_css_input_buffer buf_next_frame_id = (ia_css_input_buffer)NULL;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	verifret(stream_handle < STREAM_ID_MAX, EINVAL);
	verifret(ctx->stream_state_array[stream_handle] == IA_CSS_ISYS_STREAM_STATE_STARTED, EPERM);

	verifret(next_frame != NULL, EFAULT);

	{
		assert(ctx->stream_nof_output_pins[stream_handle] < MAX_OPINS);
		for (i=0; i<ctx->stream_nof_output_pins[stream_handle]; i++) {
			/* Verify output pin */
			verifret(next_frame->output_pins[i].info.pt < N_IA_CSS_ISYS_PIN_TYPE, EINVAL);
			switch(next_frame->output_pins[i].info.pt) {
			case IA_CSS_ISYS_PIN_TYPE_MIPI:
				verifret(next_frame->output_pins[i].info.type_specifics.dt < N_IA_CSS_ISYS_MIPI_DATA_TYPE, EINVAL);
				break;
			case IA_CSS_ISYS_PIN_TYPE_RAW_NS:
			case IA_CSS_ISYS_PIN_TYPE_RAW_S:
				verifret(next_frame->output_pins[i].info.type_specifics.ft < N_IA_CSS_ISYS_FRAME_FORMAT, EINVAL);
				break;
			default:
				break;
			}
			verifret(
				next_frame->output_pins[i].info.output_res.width >= output_min_width &&
				next_frame->output_pins[i].info.output_res.width <= output_max_width &&
				next_frame->output_pins[i].info.output_res.height >= output_min_height &&
				next_frame->output_pins[i].info.output_res.height <= output_max_height, EINVAL);
			verifret(next_frame->output_pins[i].payload.addr != 0, EFAULT);
		}
		verifret(next_frame->input_pin_set.lsc_param.addr != 0, EFAULT);
		verifret(next_frame->input_pin_set.dpc_param.addr != 0, EFAULT);
		verifret(next_frame->input_pin_set.blc_param.addr != 0, EFAULT);
	}

	retval = ia_css_syscom_send_port_available(ctx->sys, stream_handle, &packets);
	verifret(retval == 0, retval);
	verifret(packets > 0, EPERM);
	{
		token.send_type = IA_CSS_ISYS_SEND_TYPE_STREAM_CAPTURE;
		retval = ia_css_isys_constr_fw_next_frame(&next_frame_fw, &buf_next_frame_id, next_frame);
		verifret(retval == 0, retval);
		assert(next_frame_fw != 0);
		token.payload = next_frame_fw;
		token.buf_handle = HOST_ADDRESS(buf_next_frame_id);
	}
	retval = ia_css_syscom_send_port_transfer(ctx->sys, stream_handle, &token);
	verifret(retval == 0, retval);

	return 0;
}


/**
 * ia_css_isys_stream_handle_response() - handle ISYS responses
 */
 int ia_css_isys_stream_handle_response(
	HANDLE context,
	struct ia_css_isys_resp_info *received_response
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	int retval = 0;
	unsigned int packets;
	struct resp_queue_token token;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	verifret(received_response != NULL, EFAULT);

	retval = ia_css_syscom_recv_port_available(ctx->sys, 0, &packets);
	verifret(retval == 0, retval);
	verifret(packets > 0, EPERM);
	retval = ia_css_syscom_recv_port_transfer(ctx->sys, 0, &token);
	verifret(retval == 0, retval);
	retval = ia_css_isys_extract_fw_response(&token, received_response);
	verifret(retval == 0, retval);

	verifret(received_response->type < N_IA_CSS_ISYS_RESP_TYPE, EINVAL);
	verifret(received_response->stream_handle < STREAM_ID_MAX, EINVAL);
	if (received_response->type == IA_CSS_ISYS_RESP_TYPE_STREAM_OPEN_DONE ||
		received_response->type == IA_CSS_ISYS_RESP_TYPE_STREAM_START_AND_CAPTURE_DONE ||
		received_response->type == IA_CSS_ISYS_RESP_TYPE_STREAM_CAPTURE_DONE)
	{
		ia_css_input_buffer_css_unmap((ia_css_input_buffer)HOST_ADDRESS(received_response->resp_data.buf_id));
		ia_css_input_buffer_free((ia_css_input_buffer)HOST_ADDRESS(received_response->resp_data.buf_id));
	}
	if (received_response->type == IA_CSS_ISYS_RESP_TYPE_PIN_DATA_READY ||
		received_response->type == IA_CSS_ISYS_RESP_TYPE_PIN_DATA_WATERMARK)
	{
		verifret(received_response->resp_data.pin.payload.addr != 0, EFAULT);
		verifret(received_response->resp_data.pin.payload.out_buf_id != 0, EFAULT);
		verifret(received_response->resp_data.pin.info.pt < N_IA_CSS_ISYS_PIN_TYPE, EINVAL);
		switch(received_response->resp_data.pin.info.pt) {
		case IA_CSS_ISYS_PIN_TYPE_MIPI:
				verifret(received_response->resp_data.pin.info.type_specifics.dt < N_IA_CSS_ISYS_MIPI_DATA_TYPE, EINVAL);
			break;
		case IA_CSS_ISYS_PIN_TYPE_RAW_NS:
		case IA_CSS_ISYS_PIN_TYPE_RAW_S:
				verifret(received_response->resp_data.pin.info.type_specifics.ft < N_IA_CSS_ISYS_FRAME_FORMAT, EINVAL);
			break;
		default:
			return 0;
		}
		verifret(
			received_response->resp_data.pin.info.output_res.width >= output_min_width &&
			received_response->resp_data.pin.info.output_res.width <= output_max_width &&
			received_response->resp_data.pin.info.output_res.height >= output_min_height &&
			received_response->resp_data.pin.info.output_res.height <= output_max_height, EINVAL);
	}

	return 0;
}

/**
 * ia_css_isys_device_close() - close ISYS device
 */
int ia_css_isys_device_close(
	HANDLE context,
	unsigned int nof_streams
) {
	struct ia_css_isys_context *ctx = (struct ia_css_isys_context *)context;
	unsigned int i;

#if (VERIFY_DEVSTATE != 0)
	verifret(ctx->dev_state == IA_CSS_ISYS_DEVICE_STATE_CONFIGURED, EPERM);
#endif /* VERIFY_DEVSTATE */

	for (i=0; i < nof_streams; i++) {
		verifret(ctx->stream_state_array[i] == IA_CSS_ISYS_STREAM_STATE_CLOSED, EPERM);
	}
	ia_css_syscom_close(ctx->sys);
	ia_css_cpu_mem_free(context);

	return 0;
}

