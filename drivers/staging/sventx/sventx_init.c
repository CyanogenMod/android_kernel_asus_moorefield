/*

  This file is provided under a dual BSD/GPLv2 license.  When using or
  redistributing this file, you may do so under either license.

  GPL LICENSE SUMMARY

  Copyright(c) 2012-2014 Intel Corporation. All rights reserved.

  This program is free software; you can redistribute it and/or modify
  it under the terms of version 2 of the GNU General Public License as
  published by the Free Software Foundation.

  This program is distributed in the hope that it will be useful, but
  WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
  General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program; if not, write to the Free Software
  Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
  The full GNU General Public License is included in this distribution
  in the file called license.txt.

  Contact Information:
    Intel Corporation
    2200 Mission College Blvd.
    Santa Clara, CA  97052

  BSD LICENSE

  Copyright(c) 2012-2014 Intel Corporation. All rights reserved.
  All rights reserved.

  Redistribution and use in source and binary forms, with or without
  modification, are permitted provided that the following conditions
  are met:

    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in
      the documentation and/or other materials provided with the
      distribution.
    * Neither the name of Intel Corporation nor the names of its
      contributors may be used to endorse or promote products derived
      from this software without specific prior written permission.

  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*/

#include "sventx.h"
#include "sventx/event.h"

/**
 * SVEN global state
 */
static sven_header_t sven_hdr = { 0 };

static union u_sven_null_init {
	sven_handle_t handle;
	sven_header_t header;
} zero = { {0} }; /* for initializations */

/**
 * This handle is used in case if no valid handle could be obtained.
 * Accessing the nullHandle is basically a NOP. It simplifies the
 * rest of the code from testing for null handles, which is often as
 * expensive as the entire operation.
 */
sven_handle_t sventx_null_handle;

#if !defined(SVEN_PCFG_ENABLE_DEFAULT_SCATTER_WRITE)
/**
 * null-device style default output function
 */
static void nullWriter(psven_handle_t svenh,
		       psven_scatter_prog_t scatterprog, const void *pdesc)
{
}
#endif

/**
 * Initialize the SVENTX library.
 *
 * This function must be called during the start of the platform before any
 * other instrumentation library call. The function initializes the global
 * state data necessary for the library to execute. Passing NULL as state
 * means using the shared global state singleton. Passing a valid pointer
 * allows using multiple SVENTX state context structures in parallel.
 *
 * @param header Pointer to SVENTX global state structure or NULL for default.
 * @param pfinit Pointer to platform initialization function or 0 if not used.
 * @param init_param Value passed to the the platform init hook function.
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_init(psven_header_t header, sven_inithook_t pfinit,
	    const void *init_param)
{
	if (0 == header) {
		/* No user supplied global state storage,
		 * use internal default state
		 */
		header = &sven_hdr;
	}

	*header = zero.header;
	header->svh_version = SVEN_VERSION_CODE;
#if defined(SVEN_PCFG_ENABLE_HOT_GATE)
	header->svh_hot = SVEN_HOT_ENABLE_DEFAULT;
#endif

#if defined(SVEN_PCFG_ENABLE_DEFAULT_SCATTER_WRITE)
	header->svh_writer = sventx_sth_scatter_write;
#else
	header->svh_writer = nullWriter;
#endif

	sventx_null_handle = zero.handle;

	/* call platform state initialization hook if defined
	 */
	if (pfinit != (sven_inithook_t) 0)
		(*pfinit) (header, init_param);
}

/**
 * Destroy the SVENTX library state.
 *
 * This function must be called during shutdown of the platform to release
 * any SVENTX resources.
 *
 * @param header Pointer to library state or NULL to use shared default.
 * @param pfdestroy Pointer to platform state destroy function or 0
 *                  if not used.
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_destroy(psven_header_t header, sven_destroyhook_t pfdestroy)
{
	if (0 == header) {
		/* No user supplied global state storage,
		 * use internal default state
		 */
		header = &sven_hdr;
	}

	/* call platform state destroy hook first, if defined
	 */
	if ((sven_destroyhook_t) 0 != pfdestroy)
		(*pfdestroy) (header);
}

/**
 *  Initialize a SVEN handle.
 *
 * @param header Pointer to library state or NULL to use shared default.
 * @param svh Pointer to new/uninitialized SVEN handle
 * @param init_param Value passed to the the platform handle init function
 * @param fromheap 1 of heap allocated handle, 0 otherwise
 */
SVEN_EXPORT psven_handle_t SVEN_CALLCONV
sventx_init_handle(psven_header_t header,
		  psven_handle_t svh,
		  const void *init_param, sven_u32_t fromHeap)
{
	if ((psven_handle_t) 0 == svh)
		return &sventx_null_handle;


	if (0 == header) {
		/* No user supplied global state storage,
		 * use internal default state
		 */
		header = &sven_hdr;
	}

	*svh = zero.handle;

	svh->svh_header = header;
	svh->svh_flags.shf_alloc = fromHeap ? 1 : 0;

#if defined(SVEN_PCFG_ENABLE_SEQUENCE_COUNT)
	svh->svh_sequence_count = 0;
#endif

	/* call platform handle initialization hook if defined
	 */
#if defined(SVEN_PCFG_ENABLE_PLATFORM_HANDLE_DATA)
	if ((sven_inithandle_hook_t) 0 != svh->svh_header->svh_inith)
		svh->svh_header->svh_inith(svh, init_param);
#endif
	return svh;
}

/**
 *  Release a SVEN handle.
 *
 * @param svh Pointer to initialized SVEN handle
 */
SVEN_EXPORT void SVEN_CALLCONV sventx_delete_handle(psven_handle_t svh)
{
	if (&sventx_null_handle != svh && (psven_handle_t) 0 != svh) {

#if defined(SVEN_PCFG_ENABLE_PLATFORM_HANDLE_DATA)
		/* call platform handle release hook if defined
		 */
		if ((sven_releasehandle_hook_t) 0 !=
		    svh->svh_header->svh_releaseh)
			svh->svh_header->svh_releaseh(svh);
#endif

#if defined(SVEN_PCFG_ENABLE_HEAP_MEMORY)
		if (0 != svh->svh_flags.shf_alloc) {
			SVEN_HEAP_FREE(svh);
		} else
#endif
		{
			*svh = zero.handle;
		}
	}
}
