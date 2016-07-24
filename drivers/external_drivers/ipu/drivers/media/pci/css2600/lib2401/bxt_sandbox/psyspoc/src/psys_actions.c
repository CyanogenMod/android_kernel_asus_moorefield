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

#include <ia_css_psys_process_group_cmd_impl.h>
#include <ia_css_psys_process_group.h>
#include "ia_css_psys_state_change_actions.h"
#include "ia_css_psys_sppipeline.h"
#include "ia_css_psys_device_internal.h"
#include "assert_support.h"
#include "ia_css_fwctrl.h"
#include "ia_css_psys_device.h"
#include "ia_css_psyspoc_ctx_pool.h"
#include "ia_css_psys_pgpoc_context.h"
#include "ia_css_debug.h"
#include "misc_support.h"

void ia_css_process_group_state_change(
	ia_css_process_group_t *process_group)
{
	int count;
	ia_css_psys_pgpoc_context_t *context;

	assert(process_group != NULL);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_process_group_state_change(): enter curr_state(%u)\n ",
		process_group->state);

	switch(process_group->state) {
		case IA_CSS_PROCESS_GROUP_STARTED:

		/* retrieve context from token*/
		context = (ia_css_psys_pgpoc_context_t *)
				ia_css_psyspoc_ctx_pool_buffer_map(
				ia_css_process_group_get_private_token(process_group));
		assert(context != NULL);
		if (context == NULL) {
			return;
		}

		ia_css_psys_sppipeline_cmd_create(process_group, context);

		count = ia_css_psys_cmd_queue_send(
			psys_device_get_global_syscom_context(),
			IA_CSS_PSYS_CMD_QUEUE_EXECUTE_ID,
			(const void *)&context->host_cmd);
		/* Assert that the command has been queued. */
		assert(count == 1);

		break;

		default:
		break;
	}

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_process_group_state_change(): exit\n ");

	return;
}

/* ia_css_process_group_create will emit these after creation. manifest and
 * param can stores in poc context or as needed. */
int ia_css_process_group_on_create(
	ia_css_process_group_t					*process_group,
	const ia_css_program_group_manifest_t			*program_group_manifest,
	const ia_css_program_group_param_t			*program_group_param)
{
	ia_css_psys_pgpoc_context_t *context;
	int ret;

	assert(process_group != NULL);
	assert(program_group_manifest != NULL);
	assert(program_group_param != NULL);
	if (process_group == NULL ||
	    program_group_manifest == NULL ||
	    program_group_param ==  NULL) {
		return -1;
	}
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE, "ia_css_process_group_on_create(): enter \n ");

	/* Acquire a psyspoc context */
	context = (ia_css_psys_pgpoc_context_t *)
		ia_css_psyspoc_ctx_pool_acquire_buffer();
	assert(context != NULL);
	if (context == NULL) {
		return -1;
	}
	memset(context, 0, sizeof(ia_css_psys_pgpoc_context_t));
	ret = ia_css_process_group_set_private_token(process_group,
		ia_css_psyspoc_ctx_pool_token_map(context));

	/*Store manifest inside context.*/
	context->pg_manifest = program_group_manifest;
	context->param = program_group_param;
	assert(ret == 0);
	return 0;
}

/* Emitted before ia_css_process_group_destroy is called. clean up poc context
 * here. */
int ia_css_process_group_on_destroy(
	ia_css_process_group_t					*process_group)
{
	uint64_t private_token;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_process_group_on_destroy(): enter\n ");

	assert(process_group != NULL);

	private_token = ia_css_process_group_get_private_token(process_group);
	assert(private_token != 0);

	ia_css_psys_sppipeline_cmd_free(process_group,
		ia_css_psyspoc_ctx_pool_buffer_map(private_token));
	ia_css_psyspoc_ctx_pool_release_token(private_token);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_process_group_on_destroy(): exit\n ");

	return 0;
}
