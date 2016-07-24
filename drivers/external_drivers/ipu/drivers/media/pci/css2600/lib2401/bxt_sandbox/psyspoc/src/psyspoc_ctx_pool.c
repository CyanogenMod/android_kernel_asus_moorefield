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

#include "ia_css_host_buffer_pool.h"
#include "ia_css_psyspoc_ctx_pool.h"
#include "ia_css_psys_pgpoc_context.h"
#include "ia_css_debug.h"

/* psyspoc needs a context to store poc specific information, mostly related to
 * css. This information is stored in a private token field in process group.
 * BXT might have its own context, if one is needed. The buffer pools we use for
 * fwctrl are reused here. ctx pool does not belong to fwctrl and hence the
 * separate source files. MAX_PSYSPOC_CTX_MSGS is same as MAX_PSYS_MSGS in
 * fwctrl and the number of process groups. */

#define MAX_PSYSPOC_CTX_MSGS 100

static struct ia_css_host_buffer_pool psyspoc_ctx_buf_pool;
static bool ctx_pool_init = false;

int32_t ia_css_psyspoc_ctx_pool_init(void)
{
	size_t size;
	int32_t ret;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
	"ia_css_psyspoc_ctx_pool_init(): enter\n");

	if(ctx_pool_init) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_init(): context pool is already init\n");
		return 0;
	}

	size = sizeof(ia_css_psys_pgpoc_context_t);
	ia_css_create_host_bufpool(&psyspoc_ctx_buf_pool);
	ret = psyspoc_ctx_buf_pool.init(&psyspoc_ctx_buf_pool, size,
		MAX_PSYSPOC_CTX_MSGS);
	if(ret != 0) {
		ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
			"ia_css_psyspoc_ctx_pool_init(): error ret-%d\n", ret);
		return ret;
	}

	ctx_pool_init = true;

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_init(): exit\n");
	return 0;
}

void ia_css_psyspoc_ctx_pool_uninit(void)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_uninit(): enter ctx_pool_init(%u)\n", ctx_pool_init);

	if(ctx_pool_init)
		psyspoc_ctx_buf_pool.uninit(&psyspoc_ctx_buf_pool);

	ctx_pool_init = false;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_uninit(): exit\n");
}

void *ia_css_psyspoc_ctx_pool_acquire_buffer(void)
{
	void *acq_buf = NULL;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_acquire_buffer(): enter ctx_pool_init(%u)\n",
		ctx_pool_init);

	if(ctx_pool_init)
		acq_buf = (void *)psyspoc_ctx_buf_pool.acquire_host_buf(
			&psyspoc_ctx_buf_pool);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_acquire_buffer(): exit acq_buf(0x%x)\n",
		acq_buf);
	return acq_buf;
}

void ia_css_psyspoc_ctx_pool_release_token(uint64_t token)
{
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_release_token(): enter ctx_pool_init(%u) token(0x%x)\n",
		ctx_pool_init, token);

	if(ctx_pool_init && token)
		psyspoc_ctx_buf_pool.release_host_token(&psyspoc_ctx_buf_pool,
			token);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_release_token(): exit\n");
	return;
}

void *ia_css_psyspoc_ctx_pool_buffer_map(uint64_t token)
{
	void *host_buf = NULL;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_buffer_map(): enter ctx_pool_init(%u) token(0x%x)\n",
		ctx_pool_init, token);

	if(ctx_pool_init && token)
		host_buf = psyspoc_ctx_buf_pool.host_buf_map(&psyspoc_ctx_buf_pool,
			token);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_buffer_map(): exit host_buf(0x%x)\n", host_buf);
	return host_buf;
}

uint64_t ia_css_psyspoc_ctx_pool_token_map(void *host_buf)
{
	uint64_t token = 0;
	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_token_map(): enter ctx_pool_init(%u) host_buf(0x%x)\n",
		ctx_pool_init, host_buf);

	if(ctx_pool_init && (host_buf != NULL) )
		token = psyspoc_ctx_buf_pool.token_buf_map(&psyspoc_ctx_buf_pool,
			host_buf);

	ia_css_debug_dtrace(IA_CSS_DEBUG_TRACE,
		"ia_css_psyspoc_ctx_pool_token_map(): exit token(0x%x)\n", token);
	return token;
}
