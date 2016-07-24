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

#include "ia_css_psys_device.h"
#include "ia_css_psys_device_internal.h"

#include <error_support.h>
#include <assert_support.h>
#include <print_support.h>
#include <misc_support.h>
#include "ia_css_circbuf_comm.h" /* ia_css_circbuf_desc_t*/
#include "ia_css_queue.h"        /* ia_css_queue_t */
#include "ia_css_fwctrl_public.h"
#include "ia_css_fwctrl.h"
#include "ia_css_buffer_pool.h"
#include "ia_css_isyspoc_comm.h"
#include "ia_css_psys_cmd_comm.h"
#include "ia_css_debug.h"
#include "ia_css_psys_device.h"
#include "ia_css_psys_debug.h"
#include "ia_css_psyspoc_ctx_pool.h"

/* Change PSYSPOC_HW_DEBUG to 1 to enable debugging on HW*/
#define PSYSPOC_HW_DEBUG	(0)

#if (PSYSPOC_HW_DEBUG)
#include "spmem_dump.c"
#endif /*PSYSPOC_HW_DEBUG*/

struct ia_css_syscom_context {
	struct ia_css_syscom_config config;
};

#define MAX_PSYS_MSGS 50

static struct ia_css_syscom_config	psyspoc_syscom_config = {0, 0, 0, 0, 0, 0, 0, 0, 0, NULL, 0};
static bool	external_alloc = true;

static struct ia_css_syscom_context context;
static struct ia_css_syscom_context *ptr_context = NULL;
static ia_css_psysapi_cmd_t return_cmd;

static unsigned int ia_css_psys_cmd_msg_size[IA_CSS_N_PSYS_CMD_QUEUE_ID] = {
	sizeof(ia_css_psysapi_cmd_t), 0};
static unsigned int ia_css_psys_event_msg_size[IA_CSS_N_PSYS_EVENT_QUEUE_ID] = {
	sizeof(ia_css_psysapi_cmd_t)};
#if (PSYSPOC_HW_DEBUG)
static void psyspoc_dump_sp_stream_state(const char *msg);
#else
#define psyspoc_dump_sp_stream_state(msg)
#endif /*PSYSPOC_HW_DEBUG*/

struct ia_css_syscom_context *psys_device_get_global_syscom_context(void)
{
	/* return open only if psys_open has been called. */
	if(ptr_context != NULL)
		return ptr_context;

	return NULL;
}

struct ia_css_syscom_config *ia_css_psys_specify(void)
{
	struct ia_css_syscom_config	*config = &psyspoc_syscom_config;

	/* Not really used yet */
	config->num_input_queues = 1;
	config->num_output_queues = IA_CSS_N_PSYS_EVENT_QUEUE_ID;
	config->input_token_size = sizeof(ia_css_psysapi_cmd_t);
	config->output_token_size = sizeof(ia_css_psysapi_cmd_t);

	return config;
}

struct ia_css_syscom_context* ia_css_psys_open(
	const struct ia_css_psys_buffer_s		*buffer,
	struct ia_css_syscom_config				*config)
{
	struct ia_css_fwctrl_devconfig device_config;
	int ret;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_psys_open() entered \n ");
	verifexit(config != NULL, EINVAL);

	/* allowing only one context at a time => only one psys_open. Repeated calls
	 * will fail here. */
	assert(ptr_context == NULL);

	if (buffer == NULL) {
/* Allocate locally */
		external_alloc = false;
	}

	/* We don't know how to handle anything else. */
	assert(buffer == NULL);

	context.config = *config;
	device_config.firmware_address = config->specific_addr;
	ret = ia_css_fwctrl_device_open(&device_config);
	if(ret != 0)
		goto EXIT;

	ret = ia_css_psyspoc_ctx_pool_init();
	if(ret != 0)
		goto EXIT;

	ptr_context = &context;
EXIT:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_psys_open() exit \n ");
	return ptr_context;
}

struct ia_css_syscom_context* ia_css_psys_close(
	struct ia_css_syscom_context			*context)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_psys_close() entered \n ");

	verifexit(context != NULL, EINVAL);

	ptr_context = NULL;
	if (external_alloc) {
/*		memset(); */
	} else {
/* Free local allocations */
/* Reset */
		external_alloc = true;
		ia_css_psyspoc_ctx_pool_uninit();
		ia_css_fwctrl_device_close();
	}
EXIT:
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_psys_close() exit \n ");
	return ptr_context;
}

int ia_css_psys_cmd_queue_send(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_cmd_queue_ID_t		id,
	const void								*cmd_msg_buffer)
{
	int	count = 0, ret;

	verifexit(context != NULL, EINVAL);
/* The ~full check fails on receive queues */
	verifexit(cmd_msg_buffer != NULL, EINVAL);

	assert(id == IA_CSS_PSYS_CMD_QUEUE_EXECUTE_ID);

	psyspoc_dump_sp_stream_state("before sending msg");
	ret = ia_css_fwctrl_psys_send_msg(
		(const ia_css_psysapi_cmd_t *)cmd_msg_buffer);
	assert(ret == 0);

	count = 1;
EXIT:
	return count;
}

int ia_css_psys_event_queue_receive(
	struct ia_css_syscom_context			*context,
	ia_css_psys_event_queue_ID_t			id,
	void									*event_msg_buffer)
{
	int	count = 0, ret;
	struct ia_css_psys_event_s *msg;

	verifexit(context != NULL, EINVAL);
/* The ~empty check fails on send queues */
	verifexit(event_msg_buffer != NULL, EINVAL);

	assert(id == IA_CSS_PSYS_EVENT_QUEUE_MAIN_ID);

	ret = ia_css_fwctrl_psys_receive_msg(
		(ia_css_psysapi_cmd_t *)&return_cmd);
	if(ret != 0)
		goto EXIT;

	msg = (struct ia_css_psys_event_s *)event_msg_buffer;
	memset(msg, 0, sizeof(struct ia_css_psys_event_s));
	msg->status = (uint16_t) return_cmd.psys_event;

	/* In poc, we have no use for iunit process group address. User has to
	 * identify his process group from the token */
	msg->token = (uint64_t) return_cmd.token;

	count = 1;
	psyspoc_dump_sp_stream_state("received msg");
EXIT:
	return count;
}

size_t ia_css_psys_get_cmd_msg_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id)
{
	size_t	msg_size = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	NOT_USED(id);
	msg_size = ia_css_psys_cmd_msg_size[0];
EXIT:
	return msg_size;
}

size_t ia_css_psys_get_event_msg_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id)
{
	size_t	msg_size = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	NOT_USED(id);
	msg_size = ia_css_psys_event_msg_size[0];
EXIT:
	return msg_size;
}

#if (PSYSPOC_HW_DEBUG)
static void psyspoc_dump_sp_stream_state(const char *msg)
{
	uint32_t psysdebug_msg_cnt;
	uint32_t psysdebug_execute_start_cnt;
	uint32_t psysdebug_execute_done_cnt;
	uint32_t debug_host2sp_event_cnt;
	uint32_t debug_event_thread_loop_cnt;
	uint32_t debug_callout_cnt;
	uint32_t debug_sp2host_event_cnt;
	ia_css_psysapi_cmd_t psys_cmd;

	if (msg) {
		/*Helper messages*/
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, msg);
	}

	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(debug_host2sp_event_cnt),
		&debug_host2sp_event_cnt, sizeof(uint32_t));

	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(debug_event_thread_loop_cnt),
		&debug_event_thread_loop_cnt, sizeof(uint32_t));

	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(debug_callout_cnt),
		&debug_callout_cnt, sizeof(uint32_t));

	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(debug_sp2host_event_cnt),
		&debug_sp2host_event_cnt, sizeof(uint32_t));

	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(psysdebug_msg_cnt),
		&psysdebug_msg_cnt, sizeof(uint32_t));

	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(psysdebug_execute_start_cnt),
		&psysdebug_execute_start_cnt, sizeof(uint32_t));

	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(psysdebug_execute_done_cnt),
		&psysdebug_execute_done_cnt, sizeof(uint32_t));

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"event_thread_cnt=%d callout_cnt=%d host2sp_event_cnt=%d sp2host_event_cnt=%d\n"
		"PSYS: msg_cnt=%d execute_start=%d execute_done=%d\n",
		debug_event_thread_loop_cnt, debug_callout_cnt, debug_host2sp_event_cnt,
		debug_sp2host_event_cnt,
		psysdebug_msg_cnt, psysdebug_execute_start_cnt, psysdebug_execute_done_cnt);

	/* Dump the command that SP has seen*/
	sp_dmem_load(SP0_ID,
		(unsigned int)sp_address_of(psys_cmd),
		&psys_cmd, sizeof(psys_cmd));
	ia_css_debug_psys_cmd_print(&psys_cmd);
	ia_css_debug_dump_sp_state();
}
#endif /*PSYSPOC_HW_DEBUG*/
