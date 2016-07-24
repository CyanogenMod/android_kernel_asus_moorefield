/*
 * INTEL CONFIDENTIAL
 *
 * Copyright (C) 2010 - 2014 Intel Corporation.
 * All Rights Reserved.
 *
 * The source code contained or described herein and all documents
 * related to the source code ("Material") are owned by Intel Corporation
 * or licensors. Title to the Material remains with Intel
 * Corporation or its licensors. The Material contains trade
 * secrets and proprietary and confidential information of Intel or its
 * licensors. The Material is protected by worldwide copyright
 * and trade secret laws and treaty provisions. No part of the Material may
 * be used, copied, reproduced, modified, published, uploaded, posted,
 * transmitted, distributed, or disclosed in any way without Intel's prior
 * express written permission.
 *
 * No License under any patent, copyright, trade secret or other intellectual
 * property right is granted to or conferred upon you by disclosure or
 * delivery of the Materials, either expressly, by implication, inducement,
 * estoppel or otherwise. Any license under such intellectual property rights
 * must be express and approved by Intel in writing.
 */

#ifndef _IA_CSS_ISYSPOC_COMM_H
#define _IA_CSS_ISYSPOC_COMM_H

#include "ia_css_isys_comm.h"    /* ia_css_isys_stream_cfg_t */

#define ISYSPOC_OUTPUT_PIN_MAX	(1)
#define MIN_ISYS_CAPTURE_RESP	(2)
#define MAX_ISYS_MSG_RESP		(ISYSPOC_OUTPUT_PIN_MAX + MIN_ISYS_CAPTURE_RESP)

enum ia_css_isys_stream_state {
	IA_CSS_ISYS_STREAM_AVAILABLE = 0,
	IA_CSS_ISYS_STREAM_START_WAITING,
	IA_CSS_ISYS_STREAM_START_CAPTURE_WAITING,
	IA_CSS_ISYS_STREAM_CAPTURE_WAITING,
	IA_CSS_ISYS_STREAM_START_CAPTURE_PARTIAL_DONE,
    IA_CSS_ISYS_STREAM_CAPTURE_DONE,
};

typedef struct {
	hrt_vaddress buf_addr;
	uint32_t buf_id;
} ia_css_isyspoc_buffer;

#define SIZE_OF_VISYS_STRUCT	sizeof(ia_css_isys_stream_cfg_t)
#define SIZE_OF_POCBUFF_STRUCT	sizeof(ia_css_isyspoc_buffer)

#define SIZE_OF_ISYSPOC_PAYLOAD_UNION				\
	(MAX(SIZE_OF_VISYS_STRUCT, SIZE_OF_POCBUFF_STRUCT))

#define SIZE_OF_ISYSPOC_CMD_MSG_STRUCT	\
	((7 * sizeof(uint32_t)) +			\
	((MAX_ISYS_MSG_RESP-1) * sizeof(hrt_vaddress)) +	\
	sizeof(virtual_input_system_stream_t) +	\
	SIZE_OF_ISYSPOC_PAYLOAD_UNION)

typedef struct {
	uint32_t send_type;
	uint32_t resp_type;
	uint32_t timestamp[2];
	uint32_t stream_handle;
	uint32_t ret;
	uint32_t num_child_msgs;
	hrt_vaddress child_msg_ptrs[MAX_ISYS_MSG_RESP-1];
	virtual_input_system_stream_t virtual_input_system;
	union {
		ia_css_isys_stream_cfg_t virtual_input_system_cfg;
		ia_css_isyspoc_buffer isys_output;
	} payload;
} ia_css_isyspoc_cmd_msg_t;

/* This is shared to enable debugging */
typedef struct {
	enum ia_css_isys_stream_state state;
	uint32_t msg_addr;
	ia_css_isyspoc_cmd_msg_t msg;
	uint32_t cur_msg_addr;
	ia_css_isyspoc_cmd_msg_t cur_resp_msg;
	uint32_t child_msg_resps;
} ia_css_isys_stream_holder_t;

#endif				/* _IA_CSS_ISYSPOC_COMM_H */
