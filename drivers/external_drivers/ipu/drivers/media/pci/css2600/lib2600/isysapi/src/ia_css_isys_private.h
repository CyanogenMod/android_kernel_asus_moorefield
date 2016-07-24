#ifndef __IA_CSS_ISYS_PRIVATE_H__
#define __IA_CSS_ISYS_PRIVATE_H__


#include "type_support.h"
/* Needed for the structure member ia_css_sys_context * sys */
#include "ia_css_syscom.h"
/* Needed for the definitions of STREAM_ID_MAX */
#include "ia_css_isysapi.h"
/* The following is needed for the function arguments */
#include "ia_css_isys_fw_bridged_types.h"

#include "ia_css_input_buffer.h"


/* Set any of the 2 for the respective error handling */
#define VERIFY_DEVSTATE 1
#define VERIFY_STRSTATE 1

#if (VERIFY_DEVSTATE != 0)
/**
 * enum device_state
 */
enum device_state {
	IA_CSS_ISYS_DEVICE_STATE_IDLE = 0,
	IA_CSS_ISYS_DEVICE_STATE_CONFIGURED
};
#endif /* VERIFY_DEVSTATE */

/**
 * enum stream_state
 */
enum stream_state {
	IA_CSS_ISYS_STREAM_STATE_IDLE = 0,
	IA_CSS_ISYS_STREAM_STATE_OPENED,
	IA_CSS_ISYS_STREAM_STATE_STARTED,
	IA_CSS_ISYS_STREAM_STATE_CLOSED
};



struct ia_css_isys_context {
    struct ia_css_syscom_context *sys;
    /*add here any isys specific members that need
	  to be passed into the isys api functions as input */
	unsigned int stream_nof_output_pins[STREAM_ID_MAX];
#if (VERIFY_DEVSTATE != 0)
	enum device_state dev_state;
#endif /* VERIFY_DEVSTATE */
	enum stream_state stream_state_array[STREAM_ID_MAX];
};

/**
 * ia_css_isys_constr_fw_stream_cfg()
 */
extern int ia_css_isys_constr_fw_stream_cfg(
	ia_css_input_buffer_css_address *pstream_cfg_fw,
	ia_css_input_buffer *pbuf_stream_cfg_id,
	const struct ia_css_isys_stream_cfg_data *stream_cfg
);

/**
 * ia_css_isys_constr_fw_next_frame()
 */
extern int ia_css_isys_constr_fw_next_frame(
	ia_css_input_buffer_css_address *pnext_frame_fw,
	ia_css_input_buffer *pbuf_next_frame_id,
	const struct ia_css_isys_frame_buff_set *next_frame
);

/**
 * ia_css_isys_extract_fw_response()
 */
extern int ia_css_isys_extract_fw_response(
	const struct resp_queue_token *token,
	struct ia_css_isys_resp_info *received_response
);

/**
 * ia_css_isys_prepare_param()
 */
extern int ia_css_isys_prepare_param(
	struct is_css_isys_fw_config *isys_fw_cfg,
	const struct ia_css_isys_buffer_partition *pmipi,
	const struct ia_css_isys_buffer_partition *ppixel
);

#endif /*__IA_CSS_ISYS_PRIVATE_H_INCLUDED__*/
