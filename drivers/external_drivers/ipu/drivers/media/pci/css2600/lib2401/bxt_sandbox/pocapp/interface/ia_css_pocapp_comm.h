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

#ifndef _IA_CSS_POCAPP_COMM_H
#define _IA_CSS_POCAPP_COMM_H

#include "ia_css_circbuf_comm.h" /* ia_css_circbuf_desc_t*/
#include <input_system.h>

#define BXTPOC_COMM_QUEUE_SIZE	(10)

struct bxt_poc_host_sp_queues {
	/*
	 * generic event queues.
	 */
	ia_css_circbuf_desc_t host2sp_event_queue_desc;
	ia_css_circbuf_elem_t host2sp_event_queue_elems[BXTPOC_COMM_QUEUE_SIZE];
	ia_css_circbuf_desc_t sp2host_event_queue_desc;
	ia_css_circbuf_elem_t sp2host_event_queue_elems[BXTPOC_COMM_QUEUE_SIZE];

	/*
	 * ISYS event queues.
	 */
	ia_css_circbuf_desc_t host2sp_isys_cmd_queue_desc[INPUT_SYSTEM_N_STREAM_ID];
	ia_css_circbuf_elem_t host2sp_isys_cmd_queue_elems[INPUT_SYSTEM_N_STREAM_ID][BXTPOC_COMM_QUEUE_SIZE];
	ia_css_circbuf_desc_t sp2host_isys_event_queue_desc;
	ia_css_circbuf_elem_t sp2host_isys_event_queue_elems[BXTPOC_COMM_QUEUE_SIZE];

	/*
	 * PSYS event queues.
	*/
	ia_css_circbuf_desc_t host2sp_psys_cmd_queue_desc;
	ia_css_circbuf_elem_t host2sp_psys_cmd_queue_elems[BXTPOC_COMM_QUEUE_SIZE];
	ia_css_circbuf_desc_t sp2host_psys_event_queue_desc;
	ia_css_circbuf_elem_t sp2host_psys_event_queue_elems[BXTPOC_COMM_QUEUE_SIZE];

};
/************************************************************/
/*	QUEUE DATA STRUCUTRES ARE ALSO ALLOCATED HERE       */
/************************************************************/
/*Note: Intentionally gave name , in order to avoid touching mk_firmware_sp.c
 * for host-sp comm*/
extern volatile struct bxt_poc_host_sp_queues ia_css_bufq_host_sp_queue;

/**************************************************************
 *
 *   EVENT IDS FOR BXT POC
 *
 **************************************************************/
#define SP_SW_EVENT_ISYS  7
#define SP_SW_EVENT_PSYS  8

#endif  /* _IA_CSS_POCAPP_COMM_H */
