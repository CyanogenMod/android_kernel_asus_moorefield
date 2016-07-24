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

/* SVEN Instrumentation API implementation
 */

#ifndef SVEN_API_INCLUDED
#define SVEN_API_INCLUDED

#ifndef SVEN_H_INCLUDED
#include "sventx.h"
#endif

#ifdef __cplusplus
extern "C" {
#endif

/* Define away all instrumentation calls if one if the following global
 * disable switches is set
 */
#if !defined(SVEN_DEVH_DISABLE_SVEN) && !defined(SVEN_DISABLE_SVEN)

/**
 * @defgroup ApiSets API Sets
 *
 * SVENTX provided Instrumentation API sets
 *
 * SVENTX provided different API sets. Most sets can be individually enabled
 * or disabled using the @ref PCFG_ApiSet platform feature defines.
 */

/**
 * @defgroup API_global State and Lifetime handling macros
 * @ingroup ApiSets
 *
 * State and handle lifetime related macros
 * @{
 */

/**
 * SVENTX platform initialization with user provided state structure
 *
 * This function must be called during system startup to initialize the SVENTX
 * execution environment. This function expects a user provided SVENTX state
 * structure pointer. This call supports environments with different
 * library states at the same time. To initialize SVENTX for using a global
 * shared state, call #SVEN_INIT(f,p) instead.
 *
 * @param s Pointer to SVENTX state header variable
 * @param f Pointer to platform initialization hook function
 * @param p Pointer value that gets passed to the initialization hook function
 */
#define SVEN_INIT_STATE(s, f, p) \
	sventx_init((s), (f), (p))

/**
 * SVENTX platform shutdown with user provided state structure
 *
 * This function expect a user provided SVENTX state
 * structure pointer. This call supports environments with different
 * library states at the same time.To shutdown SVENTX using a global
 * shared state, call #SVEN_SHUTDOWN(f) instead.
 *
 * @param s Pointer to SVENTX state header variable
 * @param f pointer to platform resource destruction hook function
 */
#define SVEN_SHUTDOWN_STATE(s, f) \
	sventx_destroy((s), (f))

/**
 * SVENTX global platform initialization
 *
 * This function must be called during system startup to initialize the SVENTX
 * execution environment.
 *
 * @param f pointer to platform initialization hook function
 * @param p pointer value that gets passed to the initialization hook function
 */
#define SVEN_INIT(f, p) \
	SVEN_INIT_STATE((psven_header_t)0, (f), (p))

/**
 * SVENTX global platform shutdown
 *
 * @param f pointer to platform resource destruction hook function
 */
#define SVEN_SHUTDOWN(f) \
	SVEN_SHUTDOWN_STATE((psven_header_t)0, (f))

/**
 * Initialize non heap SVENTX handle with custom global state
 *
 * This function is used in platforms that don't support heap allocations.
 * The caller has to provide a pointer to a memory location that can hold
 * a #sven_handle_t data structure. This function expect a user provided
 * SVENTX state structure pointer as its first parameter. To create a
 * handle for the shared global state, call #SVEN_INIT_HANDLE(h,p) instead.
 *
 * @param s Pointer to SVENTX state header variable
 * @param h Pointer to handle data structure on the stack or data segment.
 * @param p Pointer to data that get passed to the platform handle init hook
 *          function.
 *
 * Example
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * extern sven_header_t svh_header;
 * static sven_handle_t svh_data;
 *
 * void foo()
 * {
 *      psven_handle_t svh;
 *
 *      svh = SVEN_INIT_HANDLE_STATE(&svh_header, &svh_data, NULL);
 *
 *      ...
 *  }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_INIT_HANDLE_STATE(s, h, p) \
	sventx_init_handle((s), (h), (p), 0)

/**
 * Initialize non heap SVENTX handle
 *
 * This function is used in platforms that don't support heap allocations.
 * The caller has to provide a pointer to a memory location that can hold
 * a #sven_handle_t data structure.
 *
 * @param h Pointer to handle data structure on the stack or data segment.
 * @param p Pointer to data that get passed to the platform handle init hook
 *          function
 *
 * Example
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * static sven_handle_t svh_data;
 *
 * void foo()
 * {
 *      psven_handle_t svh = SVEN_INIT_HANDLE(&svh_data, NULL);
 *
 *      ...
 *  }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_INIT_HANDLE(h, p) \
	SVEN_INIT_HANDLE_STATE((psven_header_t)0, (h), (p))

#if defined(SVEN_PCFG_ENABLE_HEAP_MEMORY)
/**
 * Heap allocate and initialize a new SVENTX handle for a custom global state
 *
 * This function is only supported if the platform supports the
 * feature #SVEN_PCFG_ENABLE_HEAP_MEMORY. Use
 * #SVEN_INIT_HANDLE if no heap support is enabled.
 *
 * @param s Pointer to SVENTX state header variable
 * @param p Pointer to data that get passed to the platform handle init hook
 *          function.
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * extern sven_header_t svh_header;
 *
 * void foo()
 * {
 *      psven_handle_t svh;
 *
 *      svh = SVEN_ALLOC_HANDLE_STATE(&svh_header, NULL);
 *
 *      ...
 *  }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_ALLOC_HANDLE_STATE(s, p) \
	sventx_init_handle(\
		(s),\
		(psven_handle_t)SVEN_HEAP_MALLOC(sizeof(sven_handle_t)),\
		(p), 1)

/**
 * Heap allocate and initialize a new SVENTX handle
 * This function is only supported if the platform supports the
 * feature #SVEN_PCFG_ENABLE_HEAP_MEMORY. Use
 * #SVEN_INIT_HANDLE if no heap support is enabled.
 *
 * @param p pointer to data that get passed to the platform handle init hook
 *          function
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * void foo()
 * {
 *      psven_handle_t svh = SVEN_ALLOC_HANDLE(NULL);
 *
 *      ...
 *  }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_ALLOC_HANDLE(p) \
	SVEN_ALLOC_HANDLE_STATE((psven_header_t)0, (p))

#else
#define SVEN_ALLOC_HANDLE(p) \
SVENTX_CFG_ERROR_SVEN_ALLOC_HANDLE_CALLED_WITHOUT_SVEN_PCFG_ENABLE_HEAP_MEMORY
#define SVEN_ALLOC_HANDLE_STATE(s, p) \
SVENTX_CFG_ERROR_SVEN_ALLOC_HANDLE_CALLED_WITHOUT_SVEN_PCFG_ENABLE_HEAP_MEMORY
#endif

#if defined(SVEN_PCFG_ENABLE_CHECKSUM)
/**
 * Enable or disable checksum generation over the given SVENTX handle
 *
 * @param h SVENTX handle from #SVEN_INIT_HANDLE or #SVEN_ALLOC_HANDLE
 * @param v 0 disable checksum, otherwise enable checksum
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * psven_handle_t svh = SVEN_ALLOC_HANDLE(NULL);
 *
 * // enable checksums
 * //
 * SVEN_ENABLE_HANDLE_CHECKSUM(svh, 1);
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_ENABLE_HANDLE_CHECKSUM(h, v) \
	(h)->svh_tag.et_chksum = (v) ? 1 : 0
#endif

#if defined(SVEN_PCFG_ENABLE_SEQUENCE_COUNT)
/**
 * Enable or disable sequence count generation over the given SVENTX handle.
 *
 * @param h SVENTX handle from #SVEN_INIT_HANDLE or #SVEN_ALLOC_HANDLE
 * @param v 0 disable counter, otherwise enable counter
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * psven_handle_t svh = SVEN_ALLOC_HANDLE(NULL);
 *
 * // enable handle sequence counter
 * //
 * SVEN_ENABLE_HANDLE_COUNTER(svh, 1);
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_ENABLE_HANDLE_COUNTER(h, v) \
	(h)->svh_tag.et_sequence = (v) ? 1 : 0
#endif

/**
 * Specify module and unit ID of the given SVENTX handle.
 *
 * This function is used for backward compatibility with the
 * SVEN 1.0 protocol or in scenarios where 128 bit module GUID
 * values are not supported. New code should use #SVEN_SET_HANDLE_GUID_UNIT
 * to generated output with unique origin GUID tags.
 *
 * @param h SVENTX handle from #SVEN_INIT_HANDLE or #SVEN_ALLOC_HANDLE
 * @param m module id (0..0xFE)
 * @param u unit id (0..15)
 *
 * @see #SVEN_SET_HANDLE_GUID_UNIT #sven_event_tag_t
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * psven_handle_t svh = SVEN_ALLOC_HANDLE(NULL);
 *
 * // tag events with 5:1 as module:unit id pair
 * //
 * SVEN_SET_MODULE_UNIT(svh, 5, 1);
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_SET_HANDLE_MODULE_UNIT(h, m, u) \
	((h)->svh_tag.et_module = (m), \
	 (h)->svh_tag.et_unit = (u))

#if defined(SVEN_PCFG_ENABLE_ORIGIN_GUID)

/**
 * Specify GUID and unit ID of the given SVENTX handle.
 *
 * @param h SVENTX handle from #SVEN_INIT_HANDLE or #SVEN_ALLOC_HANDLE
 * @param m #sven_guid_t data structure
 * @param u unit id (0..15)
 *
 * @see #sven_event_tag_t
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * // GUID generated for example by this tool:
 * // <http://msdn.microsoft.com/en-us/library/kw069h38.aspx>
 * //
 * static const sven_guid_t guid = {
 *     0x8887160a, 0xc965, 0x463b,
 *     { 0x9f, 0x43, 0x1e, 0xfe, 0x9f, 0xdf, 0xe3, 0xf9 }
 * };
 *
 * void foo()
 * {
 *      psven_handle_t svh = SVEN_ALLOC_HANDLE(NULL);
 *
 *      // tag events with our GUID and 1 as unit id
 *      //
 *      SVEN_SET_HANDLE_GUID_UNIT(svh, guid, 1);
 *  }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_SET_HANDLE_GUID_UNIT(h, g, u) \
	((h)->svh_tag.et_module = SVEN_MODULE_IS_GUID, \
	 (h)->svh_tag.et_unit = (u), \
	 (h)->svh_guid  = (g))
#else
#define SVEN_SET_HANDLE_GUID_UNIT(p, g, u) \
SVENTX_CFG_ERROR_SET_HANDLE_GUID_UNIT_WITHOUT_SVEN_PCFG_ENABLE_ORIGIN_GUID
#endif

/**
 * Delete a SVENTX handle
 *
 * @param h SVENTX handle from #SVEN_INIT_HANDLE or #SVEN_ALLOC_HANDLE
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * void foo()
 * {
 *      psven_handle_t svh = SVEN_ALLOC_HANDLE(NULL);
 *
 *      // ...
 *
 *      SVEN_DELETE_HANDLE(svh);
 *  }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_DELETE_HANDLE(h) \
	sventx_delete_handle(h)

/** @} */

/**
 * Pass null to instrumentation API to skip location information generation.
 */
#define SVEN_NOLOCATION  (struct s_sven_eventlocation *)0

/**
 * @defgroup WriteAPI Raw Data Writing Macros
 * @ingroup ApiSets
 *
 * Raw data writing macros:
 * @{
 */

/**
 * Send short data message with 32bit payload.<BR>
 *
 * This API is indented for space and speed restricted environments that
 * cannot support the more complex message types.
 * @param svh #psven_handle_t SVEN handle
 * @param v 32 bit output value
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * #define INIT_START 1
 * #define INIT_DONE  2
 *
 * foo()
 * {
 *      SVEN_SHORT32(svh, INIT_START);
 *      // processing ..
 *      //
 *
 *      SVEN_SHORT32(svh, INIT_DONE);
 *      // ..
 * }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_SHORT32(h, v) SVEN_STH_OUT_D32MTS(h, v)

#if defined(SVEN_PCFG_ENABLE_WRITE_API)

/**
 * Send raw data message with user defined payload.<BR>
 *
 * @param svh #psven_handle_t SVEN handle
 * @param severity #sven_severity_t severity level (0..7)
 * @param protocol 8 bit message protocol ID. This ID is used to identify
 *            the data protocol in the payload of this message for pretty
 *            printing in trace decoders. The protocol ID must uniquely
 *            identify the contents within the SVEN handle's
 *            #s_sven_eventag.et_module module ID or
 *            #s_sven_handle.svh_guid GUID. The SVEN protocol decoder for
 *            the trace decode engine (TDE) uses this value to route data
 *            from `SVEN_WRITE()` calls to other cascaded decoders to actually
 *            process the contents.
 * @param p pointer to raw data to send
 * @param len 16 bit length of data to send
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 *
 * // emit raw data under user defined decode ID 5
 * //
 * unsigned char data[] = { 0,1,2,3,4,5,6,7,8,9 };
 *
 * SVEN_WRITE(svenh, SVEN_SEVERITY_NORMAL, 5, data, sizeof(data));
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_WRITE(h, sev, id, p, len) \
	sventx_write_raw_message((h), \
		SVEN_NOLOCATION, (sev), (id), (p), (len))

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)

/** @copydoc SVEN_WRITE(h, sev, id, p, len) */
#define SVEN_WRITE_LOCADDR(h, sev, id, p, len) \
	sventx_write_raw_message((h),\
		sventx_make_address_location((h), sventx_return_addr()),\
		(sev), (id), (p), (len))

#endif				/* defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS) */

/** @copydoc SVEN_WRITE(h, sev, id, p, len) */
#define SVEN_WRITE_LOC16(h, sev, file, id,  p, len) \
	sventx_write_raw_message((h),\
		sventx_make_file_location32((h), \
				(sven_u16_t)(file), \
				(sven_u16_t)SVEN_LINE),\
				(sev), (id), (p), (len))

/** @copydoc SVEN_WRITE(h, sev, id, p, len) */
#define SVEN_WRITE_LOC32(h, sev, file, id, p, len) \
	sventx_write_raw_message((h),\
		sventx_make_file_location64((h), \
				(sven_u32_t)(file), \
				(sven_u32_t)SVEN_LINE),\
				(sev), (id), (p), (len))

#endif				/* defined(SVEN_PCFG_ENABLE_LOCATION_RECORD) */

/** @} */

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_raw_message(const psven_handle_t svh,
		psven_eventlocation_t loc,
		sven_severity_t severity,
		sven_u8_t subtype,
		const void *data, sven_u16_t length);

#endif				/* #if defined(SVEN_PCFG_ENABLE_WRITE_API) */

/* public API prototypes */

SVEN_EXPORT void SVEN_CALLCONV
sventx_init(psven_header_t header, sven_inithook_t pfinit,
		const void *init_param);
SVEN_EXPORT void SVEN_CALLCONV sventx_destroy(psven_header_t
		header,
		sven_destroyhook_t
		pfdestroy);
SVEN_EXPORT psven_handle_t SVEN_CALLCONV
sventx_init_handle(psven_header_t header, psven_handle_t svh,
		const void *init_param, sven_u32_t heap);
SVEN_EXPORT void SVEN_CALLCONV sventx_delete_handle(psven_handle_t
		svh);

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)
SVEN_EXPORT void *SVEN_CALLCONV sventx_return_addr(void);
#endif

/* Inline helper functions
 * These functions are only attempted to be inlined if the define
 * SVEN_PCFG_ENABLE_INLINE is defined.
 * Undefining it causes true functions calls to be used instead.
 * This is useful if saving code space matters more then speed.
 */
#if defined(SVEN_PCFG_ENABLE_INLINE)
#include "./inline.h"
#else
SVEN_INLINE psven_eventlocation_t SVEN_CALLCONV
sventx_make_file_location32(psven_handle_t h, sven_u16_t f,
		sven_u16_t l);
SVEN_INLINE psven_eventlocation_t SVEN_CALLCONV
sventx_make_file_location64(psven_handle_t h, sven_u32_t f,
		sven_u32_t l);
SVEN_INLINE psven_eventlocation_t SVEN_CALLCONV
sventx_make_address_location(psven_handle_t h, void *p);

SVEN_INLINE void SVEN_CALLCONV sventx_make_param0(psven_handle_t h);
SVEN_INLINE void SVEN_CALLCONV
sventx_make_param1(psven_handle_t h, sven_u32_t p1);
SVEN_INLINE void SVEN_CALLCONV
sventx_make_param2(psven_handle_t h, sven_u32_t p1,
		sven_u32_t p2);
SVEN_INLINE void SVEN_CALLCONV sventx_make_param3(psven_handle_t h,
		sven_u32_t p1,
		sven_u32_t p2,
		sven_u32_t p3);
SVEN_INLINE void SVEN_CALLCONV sventx_make_param4(psven_handle_t h,
		sven_u32_t p1,
		sven_u32_t p2,
		sven_u32_t p3,
		sven_u32_t p4);
SVEN_INLINE void SVEN_CALLCONV sventx_make_param5(psven_handle_t h,
		sven_u32_t p1,
		sven_u32_t p2,
		sven_u32_t p3,
		sven_u32_t p4,
		sven_u32_t p5);
SVEN_INLINE void SVEN_CALLCONV sventx_make_param6(psven_handle_t h,
		sven_u32_t p1,
		sven_u32_t p2,
		sven_u32_t p3,
		sven_u32_t p4,
		sven_u32_t p5,
		sven_u32_t p6);
#endif

#if defined(SVEN_PCFG_ENABLE_TRIGGER_API)
/**
 * @defgroup TriggerAPI SW initiated Trigger functions
 * @ingroup ApiSets
 *
 * Macros to emit special trace packets for triggering purposes:
 * @{
 */

/**
 * Send TRIGGER message with 7bit payload.<BR>
 *
 * This API generates a MIPI USER_TS package with an
 * 8 bit payload. The purpose of this message is to
 * create a trigger pattern for the NPK trigger unit
 * with up to 128 different states. A decode SW stack
 * usually ignores this message.
 * The highest bit of the payload is reserved and forced
 * to zero.

 * @param payload software defined trigger value
 * @since 1.4.0
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * #define TRIGGER_INITSTART 1
 * #define TRIGGER_INITDONE  2
 *
 * foo()
 * {
 *      SVEN_TRIGGER8(svh, TRIGGER_INITSTART);
 *      // processing ..
 *      //
 *
 *      SVEN_TRIGGER8(svh, TRIGGER_INITDONE);
 *      // ..
 * }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_TRIGGER8(h, v) \
	SVEN_STH_OUT_USER8TS(h, ((v) & 0x7F))

/** @} */
#endif /* defined(SVEN_PCFG_ENABLE_TRIGGER_API) */

#if defined(SVEN_PCFG_ENABLE_STRING_API)

#define _SVEN_ASSERT_DEBUG_STRING SVEN_FILE ":" _SVEN_CPP_TOSTR(SVEN_LINE)

/**
 * @defgroup StringAPI String Generating Macros
 * @ingroup ApiSets
 *
 * String generating macros:
 * @{
 */

/**
 * Send A UTF-8 character string with given severity and length.<BR>
 * A length of 0 results in a fixed 32byte character long output.Larger
 * strings get truncated, smaller ones zero filled. The string is not
 * zero terminated if it is larger then length.
 *
 * @param svh #psven_handle_t SVEN handle
 * @param severity #sven_severity_t severity level (0..7)
 * @param str const sven_u8_t * pointer to UTF-8 string bytes
 * @param len sven_u16_t number of bytes to emit or 0 to send fixed size 32bytes
 * @param file 16bit or 32bit user defined file ID (for _LOC16/32 variants)
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * #define FILE_ID 0xabdc // this file id
 * char * str = "Hello World";
 *
 * SVEN_DEBUG(svh, SVEN_SEVERITY_NORMAL, str, strlen(str));
 * SVEN_DEBUG_LOCADDR(svh, SVEN_SEVERITY_NORMAL, str, strlen(str));
 * SVEN_DEBUG_LOC16(svh, SVEN_SEVERITY_NORMAL, FILE_ID, str, strlen(str));
 * SVEN_DEBUG_LOC32(svh, SVEN_SEVERITY_NORMAL, FILE_ID, str, strlen(str));
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_DEBUG(svh, severity, str, len) \
	sventx_write_debug_string((svh), SVEN_NOLOCATION, \
			SVEN_DEBUGSTR_Generic, \
			(severity), (len), (str))

/**
 * Send function enter string event.
 *
 * @param svh #psven_handle_t SVEN handle
 * @param severity #sven_severity_t severity level (0..7)
 * @param file 16bit or 32bit user defined file ID (for _LOC16/32 variants)
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * void foo()
 * {
 *    SVEN_FUNC_ENTER(svenh, SVEN_SEVERITY_NORMAL);
 *    // body
 *    SVEN_FUNC_EXIT(svenh, SVEN_SEVERITY_NORMAL);
 * }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_FUNC_ENTER(svh, severity) \
	sventx_write_debug_string((svh), SVEN_NOLOCATION, \
			SVEN_DEBUGSTR_FunctionEntered, \
			(severity), sizeof(SVEN_FUNCTION_NAME),\
			SVEN_FUNCTION_NAME)

/**
 * Send function exit string event. The playload is the UTF-8 function name
 * of the surrounding function
 *
 * @copydetails SVEN_FUNC_ENTER(svh, severity)
 */
#define SVEN_FUNC_EXIT(svh, severity) \
	sventx_write_debug_string((svh), SVEN_NOLOCATION, \
			SVEN_DEBUGSTR_FunctionExited, \
			(severity), sizeof(SVEN_FUNCTION_NAME),\
			SVEN_FUNCTION_NAME)

/**
 * Send [file]:[line] assertion notice string event if the passed
 * condition is false.
 *
 * @param svh #psven_handle_t SVEN handle
 * @param severity #sven_severity_t severity level (0..7)
 * @param file 16bit or 32bit user defined file ID (for _LOC16/32 variants)
 * @param cond boolean condition to verify
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * void foo(void * p)
 * {
 *    SVEN_DEBUG_ASSERT(svenh, SVEN_SEVERITY_ERROR, p != NULL);
 * }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_DEBUG_ASSERT(svh, severity, cond) \
	{\
	if (!(cond)) {\
		sventx_write_debug_string((svh),\
				SVEN_NOLOCATION,\
				SVEN_DEBUGSTR_Assert, \
				(severity), sizeof(_SVEN_ASSERT_DEBUG_STRING),\
				_SVEN_ASSERT_DEBUG_STRING);\
		} \
	}


#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)

/** @copydoc SVEN_DEBUG */
#define SVEN_DEBUG_LOC16(svh, severity, file, str, len) \
	sventx_write_debug_string((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(file), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_DEBUGSTR_Generic, \
		(severity), (len), (str))

/** @copydoc SVEN_DEBUG */
#define SVEN_DEBUG_LOC32(svh, severity, file, str, len) \
	sventx_write_debug_string((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(file), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_DEBUGSTR_Generic, \
		(severity), (len), (str))

/** @copydoc SVEN_FUNC_ENTER(svh, severity) */
#define SVEN_FUNC_ENTER_LOC16(svh, severity, file) \
	sventx_write_debug_string((svh),\
		sventx_make_file_location32((svh), \
			(sven_u16_t)(file), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_DEBUGSTR_FunctionEntered, \
		(severity), sizeof(SVEN_FUNCTION_NAME),\
		SVEN_FUNCTION_NAME)

/** @copydoc SVEN_FUNC_EXIT(svh, severity) */
#define SVEN_FUNC_EXIT_LOC16(svh, severity, file) \
	sventx_write_debug_string((svh),\
		sventx_make_file_location32((svh), \
			(sven_u16_t)(file), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_DEBUGSTR_FunctionExited, \
		(severity), sizeof(SVEN_FUNCTION_NAME),\
		SVEN_FUNCTION_NAME)

/** @copydoc  SVEN_DEBUG_ASSERT(svh, severity) */
#define SVEN_DEBUG_ASSERT_LOC16(svh, severity, file, cond) \
	{\
	if (!(cond)) {\
		sventx_write_debug_string((svh),\
			sventx_make_file_location32((svh), \
					(sven_u16_t)(file), \
					(sven_u16_t)SVEN_LINE),\
			SVEN_DEBUGSTR_Assert, \
			(severity), sizeof(_SVEN_ASSERT_DEBUG_STRING),\
			_SVEN_ASSERT_DEBUG_STRING);\
		} \
	}

/** @copydoc SVEN_FUNC_ENTER(svh, severity) */
#define SVEN_FUNC_ENTER_LOC32(svh, severity, file) \
	sventx_write_debug_string((svh),\
		sventx_make_file_location64((svh), \
				(sven_u32_t)(file), \
				(sven_u32_t)SVEN_LINE),\
		SVEN_DEBUGSTR_FunctionEntered, \
		(severity), sizeof(SVEN_FUNCTION_NAME),\
		SVEN_FUNCTION_NAME)

/** @copydoc SVEN_FUNC_EXIT(svh, severity) */
#define SVEN_FUNC_EXIT_LOC32(svh, severity, file) \
	sventx_write_debug_string((svh),\
		sventx_make_file_location64((svh), \
				(sven_u32_t)(file), \
				(sven_u32_t)SVEN_LINE),\
		SVEN_DEBUGSTR_FunctionExited, \
		(severity), sizeof(SVEN_FUNCTION_NAME),\
		SVEN_FUNCTION_NAME)

/** @copydoc SVEN_DEBUG_ASSERT(svh, severity) */
#define SVEN_DEBUG_ASSERT_LOC32(svh, severity, file, cond) \
	{ \
	if (!(cond)) {\
		sventx_write_debug_string((svh),\
			sventx_make_file_location64((svh), \
					(sven_u32_t)(file), \
					(sven_u32_t)SVEN_LINE),\
			SVEN_DEBUGSTR_Assert, \
			(severity), sizeof(_SVEN_ASSERT_DEBUG_STRING),\
			_SVEN_ASSERT_DEBUG_STRING);\
		} \
	}

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)

/** @copydoc SVEN_DEBUG */
#define SVEN_DEBUG_LOCADDR(svh, severity, str, len) \
	sventx_write_debug_string((svh), \
		sventx_make_address_location((svh),\
			sventx_return_addr()),\
		SVEN_DEBUGSTR_Generic, \
		(severity), (len), (str))

/** @copydoc SVEN_FUNC_ENTER(svh, severity) */
#define SVEN_FUNC_ENTER_LOCADDR(svh, severity) \
	sventx_write_debug_string((svh),\
		sventx_make_address_location((svh),\
			sventx_return_addr()),\
		SVEN_DEBUGSTR_FunctionEntered, \
		(severity), sizeof(SVEN_FUNCTION_NAME),\
		SVEN_FUNCTION_NAME)

/** @copydoc SVEN_FUNC_EXIT(svh, severity) */
#define SVEN_FUNC_EXIT_LOCADDR(svh, severity) \
	sventx_write_debug_string((svh),\
		sventx_make_address_location((svh),\
			sventx_return_addr()),\
		SVEN_DEBUGSTR_FunctionExited, \
		(severity), sizeof(SVEN_FUNCTION_NAME),\
		SVEN_FUNCTION_NAME)

/** @copydoc  SVEN_DEBUG_ASSERT(svh, severity) */
#define SVEN_DEBUG_ASSERT_LOCADDR(svh, severity, cond) \
	{ \
	if (!(cond)) {\
		sventx_write_debug_string((svh),\
		sventx_make_address_location((svh),\
			sventx_return_addr()),\
		SVEN_DEBUGSTR_Assert, \
		(severity), sizeof(_SVEN_ASSERT_DEBUG_STRING),\
		 _SVEN_ASSERT_DEBUG_STRING);\
		} \
	}

#endif		/* defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS) */

#endif		/* defined(SVEN_PCFG_ENABLE_LOCATION_RECORD) */
/** @} */

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_debug_string(psven_handle_t svh,
		psven_eventlocation_t loc,
		sven_eventtype_debugstr_t type,
		sven_severity_t severity,
		sven_u16_t len, const char *str);

#endif		/* defined (SVEN_PCFG_ENABLE_STRING_API) */

/**
 * @defgroup CatAPI64 64 Bit Catalog Message Macros
 * @ingroup ApiSets
 *
 * SVEN catalog message instrumentation API
 *
 * Note: This API set is enabled or disabled by the
 * #SVEN_PCFG_ENABLE_CATID64_API and/or #SVEN_PCFG_ENABLE_CATID32_API
 * platform feature defines.
 * @{
 */

#if defined(SVEN_PCFG_ENABLE_CATID64_API)

/**
 * Send catalog message with 0-6 parameters.<BR>
 * This family of Macros is used to send 32 or 64 bit wide catalog
 * message IDs with up to six 32 bit wide parameters into the trace stream.
 *
 * The macro names are encoded in the following way:
 * SVEN_CATALOG{ID-WIDTH]_{PARAMETER-COUNT}
 *
 * @param svh #psven_handle_t SVEN handle
 * @param severity #sven_severity_t severity level (0..7)
 * @param catid 32/64 bit wide catalog ID
 * @param file 16/32 bit wide user defined file ID (for _LOC16/32 variants)
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 *
 * // emit plain catalog id (no parameter)
 * //
 * SVEN_CATALOG32_0(svh, SVEN_SEVERITY_ERROR, MSGID_INIT_FAIL);
 *
 * // catalog id with one parameter
 * //
 * SVEN_CATALOG32_1(svh, SVEN_SEVERITY_NORMAL, MSGID_SETLEVEL, 0x3);
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_CATALOG64_0(svh, sev, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog64_message((svh), SVEN_NOLOCATION, (sev), (id)))
#define SVEN_CATALOG64_1(svh, sev, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog64_message((svh), SVEN_NOLOCATION, (sev), (id)))
#define SVEN_CATALOG64_2(svh, sev, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog64_message((svh), SVEN_NOLOCATION, (sev), (id)))
#define SVEN_CATALOG64_3(svh, sev, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog64_message((svh), SVEN_NOLOCATION, (sev), (id)))
#define SVEN_CATALOG64_4(svh, sev, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog64_message((svh), SVEN_NOLOCATION, (sev), (id)))
#define SVEN_CATALOG64_5(svh, sev, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog64_message((svh), SVEN_NOLOCATION, (sev), (id)))
#define SVEN_CATALOG64_6(svh, sev, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog64_message((svh), SVEN_NOLOCATION, (sev), (id)))

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)

/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_0_LOCADDR(svh, sev, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_1_LOCADDR(svh, sev, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_2_LOCADDR(svh, sev, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_3_LOCADDR(svh, sev, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_4_LOCADDR(svh, sev, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_5_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_6_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))

#endif				/* defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS) */

/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_0_LOC16(svh, sev, file, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_1_LOC16(svh, sev, file, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_2_LOC16(svh, sev, file, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_3_LOC16(svh, sev, file, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_4_LOC16(svh, sev, file, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_5_LOC16(svh, sev, file, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_6_LOC16(svh, sev, file, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))

/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_0_LOC32(svh, sev, file, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_1_LOC32(svh, sev, file, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_2_LOC32(svh, sev, file, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_3_LOC32(svh, sev, file, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_4_LOC32(svh, sev, file, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_5_LOC32(svh, sev, file, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG64_6_LOC32(svh, sev, file, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog64_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))

#endif		/* defined(SVEN_PCFG_ENABLE_LOCATION_RECORD) */
/** @} */

/* API function prototypes */

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_catalog64_message(const psven_handle_t svh,
				 psven_eventlocation_t loc,
				 sven_severity_t severity,
				 sven_u64_t catid);

#endif		/* defined(SVEN_PCFG_ENABLE_CATID64_API) */

#if defined(SVEN_PCFG_ENABLE_CATID32_API)
/**
 * @defgroup CatAPI32 32 Bit Catalog Message Macros
 * @ingroup ApiSets
 *
 * SVEN catalog message instrumentation API
 * @{
 */

/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_0(svh, sev, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog32_message((svh), SVEN_NOLOCATION, (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_1(svh, sev, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog32_message((svh), SVEN_NOLOCATION, (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_2(svh, sev, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog32_message((svh), SVEN_NOLOCATION, (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_3(svh, sev, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog32_message((svh), SVEN_NOLOCATION, (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_4(svh, sev, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog32_message((svh), SVEN_NOLOCATION, (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_5(svh, sev, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog32_message((svh), SVEN_NOLOCATION, (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_6(svh, sev, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog32_message((svh), SVEN_NOLOCATION, (sev), (id)))

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)

/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_0_LOCADDR(svh, sev, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_1_LOCADDR(svh, sev, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_2_LOCADDR(svh, sev, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_3_LOCADDR(svh, sev, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_4_LOCADDR(svh, sev, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_5_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_6_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (id)))

#endif		/* defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS) */

/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_0_LOC16(svh, sev, file, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_1_LOC16(svh, sev, file, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_2_LOC16(svh, sev, file, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_3_LOC16(svh, sev, file, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_4_LOC16(svh, sev, file, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_5_LOC16(svh, sev, file, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_6_LOC16(svh, sev, file, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (id)))

/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_0_LOC32(svh, sev, file, id)\
	(sventx_make_param0(svh),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_1_LOC32(svh, sev, file, id, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_2_LOC32(svh, sev, file, id, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_3_LOC32(svh, sev, file, id, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_4_LOC32(svh, sev, file, id, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_5_LOC32(svh, sev, file, id, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))
/** @copydoc SVEN_CATALOG64_0(svh, sev) */
#define SVEN_CATALOG32_6_LOC32(svh, sev, file, id, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_catalog32_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (id)))

#endif				/* defined(SVEN_PCFG_ENABLE_LOCATION_RECORD) */
/** @} */

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_catalog32_message(const psven_handle_t svh,
				 psven_eventlocation_t loc,
				 sven_severity_t severity,
				 sven_u32_t catid);

#endif				/* defined(SVEN_PCFG_ENABLE_CATID32_API) */

#if defined(SVEN_PCFG_ENABLE_API_API)
/**
 * @defgroup ApiAPI API Function Call and Return Macros
 * @ingroup ApiSets
 *
 * SVEN API Call/Return message instrumentation macros:
 *
 * Note: This API set is enabled or disabled by the
 * #SVEN_PCFG_ENABLE_API_API platform feature define.
 * @{
 */

/**
 * Send API call message with 0-6 parameters.<BR>
 * This family of Macros is used to send API call events
 *  with up to 6 32 bit wide parameters into the trace stream.
 *
 * The macro names are encoded in the following way:
 * SVEN_API_CALL_{PARAMETER-COUNT}
 *
 * @param svh #psven_handle_t SVEN handle
 * @param severity #sven_severity_t severity level (0..7)
 * @param api user defined API set number
 * @param function user defined function id inside API set
 *
 * @see SVEN_API_RETURN
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * #define APISET_HEAP 0x10
 * #define API_MALLOC  0x0
 * #define API_CALLOC  0x1
 * #define API_FREE    0x2
 *
 * void * malloc(size_t len)
 * {
 *      SVEN_API_CALL_1(svh, SVEN_SEVERITY_NORMAL,
 *                     APISET_HEAP, API_MALLOC, len);
 *
 *      // ...
 *
 *      SVEN_API_RETURN(svh, SVEN_SEVERITY_NORMAL,
 *                      APISET_HEAP, API_MALLOC, p);
 *      return p;
 *  }
 * void * calloc(size_t nmemb, size_t len)
 * {
 *      SVEN_API_CALL_2(svh, SVEN_SEVERITY_NORMAL,
 *                      APISET_HEAP, API_CALLOC, nmemb, len);
 *
 *      // ...
 *
 *      SVEN_API_RETURN(svh, SVEN_SEVERITY_NORMAL,
 *                      APISET_HEAP, API_MALLOC, p);
 *      return p;
 *  }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_API_CALL_0(svh, sev, api, func)\
	(sventx_make_param0(svh),\
	sventx_write_api_call_message(\
		(svh), SVEN_NOLOCATION, (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_1(svh, sev, api, func, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_api_call_message(\
		(svh), SVEN_NOLOCATION, (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_2(svh, sev, api, func, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_api_call_message(\
		(svh), SVEN_NOLOCATION, (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_3(svh, sev, api, func, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_api_call_message(\
		(svh), SVEN_NOLOCATION, (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_4(svh, sev, api, func, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_api_call_message(\
		(svh), SVEN_NOLOCATION, (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_5(svh, sev, api, func, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_api_call_message(\
		(svh), SVEN_NOLOCATION, (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_6(svh, sev, api, func, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_api_call_message(\
	(svh), SVEN_NOLOCATION, (sev), (api), (func)))

/**
 * Send API return message with return value.<BR>
 *
 * @param svh #psven_handle_t SVEN handle
 * @param severity #sven_severity_t severity level (0..7)
 * @param api user defined API set number
 * @param function user defined function id inside API set
 * @param ret return value of API
 *
 * @see SVEN_API_CALL_0
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * #define APISET_HEAP 0x10
 * #define API_MALLOC  0x0
 * #define API_CALLOC  0x1
 * #define API_FREE    0x2
 *
 * void * malloc(size_t len)
 * {
 *      // ...
 *
 *      SVEN_API_RETURN(svh, SVEN_SEVERITY_NORMAL, APISET_HEAP, API_MALLOC, p);
 *      return p;
 *  }

 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_API_RETURN(svh, sev, api, func, ret)\
	sventx_write_api_return_message((svh), \
		SVEN_NOLOCATION, (sev), (api), (func), (ret))

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)

/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_0_LOCADDR(svh, sev, api, func)\
	(sventx_make_param0(svh),\
	sventx_write_api_call_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_1_LOCADDR(svh, sev, api, func, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_api_call_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_2_LOCADDR(svh, sev, api, func, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_api_call_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_3_LOCADDR(svh, sev, api, func, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_api_call_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_4_LOCADDR(svh, sev, api, func, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_api_call_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_5_LOCADDR(svh, sev, api, func, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_api_call_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_6_LOCADDR(svh, sev, api, func, p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_api_call_message((svh),\
	       sventx_make_address_location((svh), sventx_return_addr()),\
	       (sev), (api), (func)))

/** @copydoc SVEN_API_RETURN(svh, sev, api, func) */
#define SVEN_API_RETURN_LOCADDR(svh, sev, api, func, ret)\
	sventx_write_api_return_message((svh),\
		sventx_make_address_location((svh), sventx_return_addr()),\
		(sev), (api), (func), (ret))

#endif		/* defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS) */

/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_0_LOC16(svh, sev, file, api, func)\
	(sventx_make_param0(svh),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_1_LOC16(svh, sev, file, api, func, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_2_LOC16(svh, sev, file, api, func, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_3_LOC16(svh, sev, file, api, func, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_4_LOC16(svh, sev, file, api, func, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_5_LOC16(svh, sev, file, api, func, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_6_LOC16(svh, sev, file, api, func,\
		p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location32((svh), \
		   (sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
	       (sev), (api), (func)))

/** @copydoc SVEN_API_RETURN(svh, sev, api, func) */
#define SVEN_API_RETURN_LOC16(svh, sev, file, api, func, ret)\
	sventx_write_api_return_message((svh),\
		sventx_make_file_location32((svh), \
			(sven_u16_t)(file), (sven_u16_t)SVEN_LINE),\
		(sev), (api), (func), (ret))

/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_0_LOC32(svh, sev, file, api, func)\
	(sventx_make_param0(svh),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_1_LOC32(svh, sev, file, api, func, p1)\
	(sventx_make_param1(svh, p1),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_2_LOC32(svh, sev, file, api, func, p1, p2)\
	(sventx_make_param2(svh, p1, p2),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_3_LOC32(svh, sev, file, api, func, p1, p2, p3)\
	(sventx_make_param3(svh, p1, p2, p3),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_4_LOC32(svh, sev, file, api, func, p1, p2, p3, p4)\
	(sventx_make_param4(svh, p1, p2, p3, p4),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_5_LOC32(svh, sev, file, api, func, p1, p2, p3, p4, p5)\
	(sventx_make_param5(svh, p1, p2, p3, p4, p5),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (api), (func)))
/** @copydoc SVEN_API_CALL_0(svh, sev, api, func) */
#define SVEN_API_CALL_6_LOC32(svh, sev, file, api, func,\
		p1, p2, p3, p4, p5, p6)\
	(sventx_make_param6(svh, p1, p2, p3, p4, p5, p6),\
	sventx_write_api_call_message((svh),\
	       sventx_make_file_location64((svh), \
		   (sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
	       (sev), (api), (func)))

/** @copydoc SVEN_API_RETURN(svh, sev, api, func) */
#define SVEN_API_RETURN_LOC32(svh, sev, file, api, func, ret)\
	sventx_write_api_return_message((svh),\
		sventx_make_file_location64((svh), \
				(sven_u32_t)(file), (sven_u32_t)SVEN_LINE),\
		(sev), (api), (func), (ret))
#endif				/* defined(SVEN_PCFG_ENABLE_LOCATION_RECORD) */

/** @} */

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_api_call_message(const psven_handle_t svh,
			psven_eventlocation_t loc,
			sven_severity_t severity,
			sven_u32_t api,
			sven_u32_t function);

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_api_return_message(const psven_handle_t svh,
			psven_eventlocation_t loc,
			sven_severity_t severity,
			sven_u32_t api,
			sven_u32_t function,
			sven_u64_t ret);

#endif		/* defined(SVEN_PCFG_ENABLE_API_API) */

#if defined(SVEN_PCFG_ENABLE_REGISTER_API)

/**
 * @defgroup RegAPI Register Access Macros
 * @ingroup ApiSets

 * Register access reporting macros
 *
 * Note: These API set is enabled or disabled by the
 * #SVEN_PCFG_ENABLE_REGISTER_API platform feature define.
 *
 * @{
 */

#if defined(SVEN_PCFG_ENABLE_64BIT_ADDR)
	typedef sven_u64_t sven_phyaddr_t;
#else
	typedef sven_u32_t sven_phyaddr_t;
#endif

/**
 * Send MMIO register read or write event
 * This family of Macros is used to send 8, 16, 32 or 64 bit register
 * access messages into the trace stream.
 *
 * The macro names are encoded in the following way:
 * SVEN_WRITE_REG{WIDTH} or SVEN_READ_REG{WIDTH}
 *
 * @param svh #psven_handle_t SVEN handle
 * @param a physical address of MMIO register
 * @param v the value read or written
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 *#define PHYADDR_DEVx  0xA5000000
 * unsigned read_reg()
 *{
 *    volatile unsigned * pdev = MAP_PHYADDR(PHYADDR_DEVx);
 *    unsigned val;
 *
 *    val = *pdev;
 *    SVEN_READ_REG32(svh, PHYADDR_DEVx, val);
 *
 *    *pdev = val;
 *    SVEN_WRITE_REG32(svh, PHYADDR_DEVx, val);
 *
 *    return val;
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_WRITE_REG64(svh, a, v) \
	sventx_write_64bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo64_Write, \
			    (a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG64(svh, a, v) \
	sventx_write_64bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo64_Read, \
			    (a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG32(svh, a, v) \
	sventx_write_32bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo32_Write, \
			    (a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG32(svh, a, v) \
	sventx_write_32bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo32_Read, \
			    (a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG16(svh, a, v) \
	sventx_write_32bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo16_Write, \
			    (a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG16(svh, a, v) \
	sventx_write_32bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo16_Read, \
			    (a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG8(svh, a, v) \
	sventx_write_32bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo8_Write, \
			    (a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG8(svh, a, v) \
	sventx_write_32bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIo8_Read, \
			    (a), (v), 0)

/**
 * Send Model specific register read or write event
 *
 * The macro names are encoded in the following way:
 * SVEN_WRITE_REG{WIDTH} or SVEN_READ_REG{WIDTH}
 *
 * @param svh #psven_handle_t SVEN handle
 * @param i MSR register index
 * @param v the 64 bit value read or written
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 * foo()
 * {
 *    unsigned long long v = __readmsr(15);
 *    SVEN_READ_MSR(svh, 15, v);
 *
 *    __writemsr(15, 0x12345678);
 *    SVEN_WRITE_MSR(svh, 15, 0x12345678);
 * }
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_READ_MSR(svh, a, v) \
	sventx_write_64bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIoMSR_Read, \
			    (a), (v), 0)

/** @copydoc SVEN_READ_MSR(svh, a, v) */
#define SVEN_WRITE_MSR(svh, a, v) \
	sventx_write_64bit_regio_message((svh), SVEN_NOLOCATION, \
			    SVEN_EV_RegIoMSR_Write, \
			    (a), (v), 0)

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)

#if defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS)

/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG64_LOCADDR(svh, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_address_location((svh),\
			sventx_return_addr()),\
		SVEN_EV_RegIo64_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG64_LOCADDR(svh, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_address_location((svh),\
		    sventx_return_addr()),\
		SVEN_EV_RegIo64_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG32_LOCADDR(svh, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_address_location((svh),\
		    sventx_return_addr()),\
		SVEN_EV_RegIo32_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG32_LOCADDR(svh, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_address_location((svh),\
		    sventx_return_addr()),\
		SVEN_EV_RegIo32_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG16_LOCADDR(svh, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_address_location((svh),\
		    sventx_return_addr()),\
		SVEN_EV_RegIo16_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG16_LOCADDR(svh, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_address_location((svh),\
		    sventx_return_addr()),\
		SVEN_EV_RegIo16_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG8_LOCADDR(svh, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_address_location((svh),\
		    sventx_return_addr()),\
		SVEN_EV_RegIo8_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG8_LOCADDR(svh, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_address_location((svh),\
		    sventx_return_addr()),\
		SVEN_EV_RegIo8_Read, \
		(a), (v), 0)

/** @copydoc SVEN_READ_MSR(svh, a, v) */
#define SVEN_READ_MSR_LOCADDR(svh, a, v) \
	sventx_write_64bit_regio_message((svh),\
		sventx_make_address_location((svh),\
			sventx_return_addr()),\
		SVEN_EV_RegIoMSR_Read, \
		(a), (v), 0)

/** @copydoc SVEN_READ_MSR(svh, a, v) */
#define SVEN_WRITE_MSR_LOCADDR(svh, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_address_location((svh),\
			sventx_return_addr()), \
		SVEN_EV_RegIoMSR_Write, \
		(a), (v), 0)
#endif		/* defined(SVEN_PCFG_ENABLE_LOCATION_ADDRESS) */

/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG64_LOC16(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo64_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG64_LOC16(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo64_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG32_LOC16(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo32_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG32_LOC16(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo32_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG16_LOC16(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo16_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG16_LOC16(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo16_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG8_LOC16(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo8_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG8_LOC16(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
		    (sven_u16_t)(f), \
		    (sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIo8_Read, \
		(a), (v), 0)

/** @copydoc SVEN_READ_MSR(svh, a, v) */
#define SVEN_READ_MSR_LOC16(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh),\
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIoMSR_Read, \
		(a), (v), 0)

/** @copydoc SVEN_READ_MSR(svh, a, v) */
#define SVEN_WRITE_MSR_LOC16(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_file_location32((svh), \
			(sven_u16_t)(f), \
			(sven_u16_t)SVEN_LINE),\
		SVEN_EV_RegIoMSR_Write, \
		(a), (v), 0)

/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG64_LOC32(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo64_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG64_LOC32(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo64_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG32_LOC32(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo32_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG32_LOC32(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo32_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG16_LOC32(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo16_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG16_LOC32(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo16_Read, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_WRITE_REG8_LOC32(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo8_Write, \
		(a), (v), 0)
/** @copydoc SVEN_WRITE_REG64(svh, a, v) */
#define SVEN_READ_REG8_LOC32(svh, f, a, v) \
	sventx_write_32bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
		    (sven_u32_t)(f), \
		    (sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIo8_Read, \
		(a), (v), 0)

/** @copydoc SVEN_READ_MSR(svh, a, v) */
#define SVEN_READ_MSR_LOC32(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh),\
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIoMSR_Read, \
		(a), (v), 0)

/** @copydoc SVEN_READ_MSR(svh, a, v) */
#define SVEN_WRITE_MSR_LOC32(svh, f, a, v) \
	sventx_write_64bit_regio_message((svh), \
		sventx_make_file_location64((svh), \
			(sven_u32_t)(f), \
			(sven_u32_t)SVEN_LINE),\
		SVEN_EV_RegIoMSR_Write, \
		(a), (v), 0)

#endif		/* defined(SVEN_PCFG_ENABLE_LOCATION_RECORD) */
/** @} */

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_32bit_regio_message(const psven_handle_t svh,
			psven_eventlocation_t loc,
			sven_eventtype_regio_t reg_io_type,
			sven_phyaddr_t reg_addr,
			sven_u32_t reg_value,
			sven_u32_t reg_mask);

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_64bit_regio_message(const psven_handle_t svh,
			psven_eventlocation_t loc,
			sven_eventtype_regio_t reg_io_type,
			sven_phyaddr_t reg_addr,
			sven_u64_t reg_value,
			sven_u64_t reg_mask);

#endif	/* defined (SVEN_PCFG_ENABLE_REGISTER_API) */

#endif	/* ! SVEN_DEVH_DISABLE_SVEN &&  !SVEN_DISABLE_SVEN */

/* Define undefined API's to nothing. This ensures source compatibility
 * independent of the SVENTX feature configuration.
 */
#ifndef SVEN_INIT
#define SVEN_INIT(f, p)
#endif

#ifndef SVEN_SHUTDOWN
#define SVEN_SHUTDOWN(x)
#endif

#ifndef SVEN_INIT_HANDLE
#define SVEN_INIT_HANDLE(h, p) (psven_handle_t)0
#endif

#ifndef SVEN_ENABLE_HANDLE_CHECKSUM
#define SVEN_ENABLE_HANDLE_CHECKSUM(h, v)
#endif

#ifndef SVEN_ENABLE_HANDLE_COUNTER
#define SVEN_ENABLE_HANDLE_COUNTER(h, v)
#endif

#ifndef SVEN_SET_HANDLE_MODULE_UNIT
#define SVEN_SET_HANDLE_MODULE_UNIT(p, m, u)
#endif

#ifndef SVEN_SET_HANDLE_GUID_UNIT
#define SVEN_SET_HANDLE_GUID_UNIT(p, g, u)
#endif

#ifndef SVEN_INIT
#define SVEN_INIT(f, p)
#endif

#ifndef SVEN_SHUTDOWN
#define SVEN_SHUTDOWN()
#endif

#ifndef SVEN_INIT_HANDLE
#define SVEN_INIT_HANDLE(h, p)
#endif

#ifndef SVEN_ALLOC_HANDLE
#define SVEN_ALLOC_HANDLE(p) (psven_handle_t)0
#endif

#ifndef SVEN_ENABLE_HANDLE_CHECKSUM
#define SVEN_ENABLE_HANDLE_CHECKSUM(h)
#endif

#ifndef SVEN_ENABLE_HANDLE_COUNTER
#define SVEN_ENABLE_HANDLE_COUNTER(h)
#endif

#ifndef SVEN_SET_HANDLE_MODULE_UNIT
#define SVEN_SET_HANDLE_MODULE_UNIT(p, m, u)
#endif

#ifndef SVEN_SET_HANDLE_GUID_UNIT
#define SVEN_SET_HANDLE_GUID_UNIT(p, o, u)
#endif

#ifndef SVEN_SHORT32
#define SVEN_SHORT32(h, v)
#endif

#ifndef SVEN_WRITE
#define SVEN_WRITE(h, sev, id, p, len)
#endif
#ifndef SVEN_WRITE_LOCADDR
#define SVEN_WRITE_LOCADDR(h, sev, id,  p, len)
#endif
#ifndef SVEN_WRITE_LOC16
#define SVEN_WRITE_LOC16(h, sev, f, id,  p, len)
#endif
#ifndef SVEN_WRITE_LOC32
#define SVEN_WRITE_LOC32(h, sev, f, id,  p, len)
#endif

#ifndef SVEN_DELETE_HANDLE
#define SVEN_DELETE_HANDLE(h)
#endif

#ifndef SVEN_DEBUG
#define SVEN_DEBUG(svh, sev, str, len)
#endif
#ifndef SVEN_DEBUG_LOCADDR
#define SVEN_DEBUG_LOCADDR(svh, sev, str, len)
#endif
#ifndef SVEN_DEBUG_LOC16
#define SVEN_DEBUG_LOC16(svh, sev, file, str, len)
#endif
#ifndef SVEN_DEBUG_LOC32
#define SVEN_DEBUG_LOC32(svh, sev, file, str, len)
#endif
#ifndef SVEN_FUNC_ENTER
#define SVEN_FUNC_ENTER(svh, sev)
#endif
#ifndef SVEN_FUNC_ENTER_LOCADDR
#define SVEN_FUNC_ENTER_LOCADDR(svh, sev)
#endif
#ifndef SVEN_FUNC_ENTER_LOC16
#define SVEN_FUNC_ENTER_LOC16(svh, sev, file)
#endif
#ifndef SVEN_FUNC_ENTER_LOC32
#define SVEN_FUNC_ENTER_LOC32(svh, sev, file)
#endif
#ifndef SVEN_FUNC_EXIT
#define SVEN_FUNC_EXIT(svh, sev)
#endif
#ifndef SVEN_FUNC_EXIT_LOCADDR
#endif
#ifndef SVEN_FUNC_EXIT_LOCADDR
#define SVEN_FUNC_EXIT_LOCADDR(svh, sev)
#endif
#ifndef SVEN_FUNC_EXIT_LOC16
#define SVEN_FUNC_EXIT_LOC16(svh, sev, file)
#endif
#ifndef SVEN_FUNC_EXIT_LOC32
#define SVEN_FUNC_EXIT_LOC32(svh, sev, file)
#endif
#ifndef SVEN_DEBUG_ASSERT
#define SVEN_DEBUG_ASSERT(svh, sev, cond)
#endif
#ifndef SVEN_DEBUG_ASSERT_LOC16
#define SVEN_DEBUG_ASSERT_LOC16(svh, sev, file, cond)
#endif
#ifndef SVEN_DEBUG_ASSERT_LOC32
#define SVEN_DEBUG_ASSERT_LOC32(svh, sev, file, cond)
#endif
#ifndef SVEN_DEBUG_ASSERT_LOCADDR
#define SVEN_DEBUG_ASSERT_LOCADDR(svh, sev, cond)
#endif

#ifndef SVEN_CATALOG32_0
#define SVEN_CATALOG32_0(svh, sev, id)
#endif
#ifndef SVEN_CATALOG32_1
#define SVEN_CATALOG32_1(svh, sev, id, p1)
#endif
#ifndef SVEN_CATALOG32_2
#define SVEN_CATALOG32_2(svh, sev, id, p1, p2)
#endif
#ifndef SVEN_CATALOG32_3
#define SVEN_CATALOG32_3(svh, sev, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG32_4
#define SVEN_CATALOG32_4(svh, sev, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG32_5
#define SVEN_CATALOG32_5(svh, sev, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG32_6
#define SVEN_CATALOG32_6(svh, sev, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_CATALOG32_0_LOCADDR
#define SVEN_CATALOG32_0_LOCADDR(svh, sev, id)
#endif
#ifndef SVEN_CATALOG32_1_LOCADDR
#define SVEN_CATALOG32_1_LOCADDR(svh, sev, id, p1)
#endif
#ifndef SVEN_CATALOG32_2_LOCADDR
#define SVEN_CATALOG32_2_LOCADDR(svh, sev, id, p1, p2)
#endif
#ifndef SVEN_CATALOG32_3_LOCADDR
#define SVEN_CATALOG32_3_LOCADDR(svh, sev, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG32_4_LOCADDR
#define SVEN_CATALOG32_4_LOCADDR(svh, sev, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG32_5_LOCADDR
#define SVEN_CATALOG32_5_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG32_6_LOCADDR
#define SVEN_CATALOG32_6_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_CATALOG32_0_LOC16
#define SVEN_CATALOG32_0_LOC16(svh, sev, f, id)
#endif
#ifndef SVEN_CATALOG32_1_LOC16
#define SVEN_CATALOG32_1_LOC16(svh, sev, f, id, p1)
#endif
#ifndef SVEN_CATALOG32_2_LOC16
#define SVEN_CATALOG32_2_LOC16(svh, sev, f, id, p1, p2)
#endif
#ifndef SVEN_CATALOG32_3_LOC16
#define SVEN_CATALOG32_3_LOC16(svh, sev, f, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG32_4_LOC16
#define SVEN_CATALOG32_4_LOC16(svh, sev, f, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG32_5_LOC16
#define SVEN_CATALOG32_5_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG32_6_LOC16
#define SVEN_CATALOG32_6_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_CATALOG32_0_LOC32
#define SVEN_CATALOG32_0_LOC32(svh, sev, f, id)
#endif
#ifndef SVEN_CATALOG32_1_LOC32
#define SVEN_CATALOG32_1_LOC32(svh, sev, f, id, p1)
#endif
#ifndef SVEN_CATALOG32_2_LOC32
#define SVEN_CATALOG32_2_LOC32(svh, sev, f, id, p1, p2)
#endif
#ifndef SVEN_CATALOG32_3_LOC32
#define SVEN_CATALOG32_3_LOC32(svh, sev, f, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG32_4_LOC32
#define SVEN_CATALOG32_4_LOC32(svh, sev, f, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG32_5_LOC32
#define SVEN_CATALOG32_5_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG32_6_LOC32
#define SVEN_CATALOG32_6_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_CATALOG64_0
#define SVEN_CATALOG64_0(svh, sev, id)
#endif
#ifndef SVEN_CATALOG64_1
#define SVEN_CATALOG64_1(svh, sev, id, p1)
#endif
#ifndef SVEN_CATALOG64_2
#define SVEN_CATALOG64_2(svh, sev, id, p1, p2)
#endif
#ifndef SVEN_CATALOG64_3
#define SVEN_CATALOG64_3(svh, sev, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG64_4
#define SVEN_CATALOG64_4(svh, sev, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG64_5
#define SVEN_CATALOG64_5(svh, sev, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG64_6
#define SVEN_CATALOG64_6(svh, sev, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_CATALOG64_0_LOCADDR
#define SVEN_CATALOG64_0_LOCADDR(svh, sev, id)
#endif
#ifndef SVEN_CATALOG64_1_LOCADDR
#define SVEN_CATALOG64_1_LOCADDR(svh, sev, id, p1)
#endif
#ifndef SVEN_CATALOG64_2_LOCADDR
#define SVEN_CATALOG64_2_LOCADDR(svh, sev, id, p1, p2)
#endif
#ifndef SVEN_CATALOG64_3_LOCADDR
#define SVEN_CATALOG64_3_LOCADDR(svh, sev, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG64_4_LOCADDR
#define SVEN_CATALOG64_4_LOCADDR(svh, sev, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG64_5_LOCADDR
#define SVEN_CATALOG64_5_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG64_6_LOCADDR
#define SVEN_CATALOG64_6_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_CATALOG64_0_LOC16
#define SVEN_CATALOG64_0_LOC16(svh, sev, f, id)
#endif
#ifndef SVEN_CATALOG64_1_LOC16
#define SVEN_CATALOG64_1_LOC16(svh, sev, f, id, p1)
#endif
#ifndef SVEN_CATALOG64_2_LOC16
#define SVEN_CATALOG64_2_LOC16(svh, sev, f, id, p1, p2)
#endif
#ifndef SVEN_CATALOG64_3_LOC16
#define SVEN_CATALOG64_3_LOC16(svh, sev, f, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG64_4_LOC16
#define SVEN_CATALOG64_4_LOC16(svh, sev, f, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG64_5_LOC16
#define SVEN_CATALOG64_5_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG64_6_LOC16
#define SVEN_CATALOG64_6_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_CATALOG64_0_LOC32
#define SVEN_CATALOG64_0_LOC32(svh, sev, f, id)
#endif
#ifndef SVEN_CATALOG64_1_LOC32
#define SVEN_CATALOG64_1_LOC32(svh, sev, f, id, p1)
#endif
#ifndef SVEN_CATALOG64_2_LOC32
#define SVEN_CATALOG64_2_LOC32(svh, sev, f, id, p1, p2)
#endif
#ifndef SVEN_CATALOG64_3_LOC32
#define SVEN_CATALOG64_3_LOC32(svh, sev, f, id, p1, p2, p3)
#endif
#ifndef SVEN_CATALOG64_4_LOC32
#define SVEN_CATALOG64_4_LOC32(svh, sev, f, id, p1, p2, p3, p4)
#endif
#ifndef SVEN_CATALOG64_5_LOC32
#define SVEN_CATALOG64_5_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_CATALOG64_6_LOC32
#define SVEN_CATALOG64_6_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_API_CALL_0
#define SVEN_API_CALL_0(svh, sev, api, func)
#endif
#ifndef SVEN_API_CALL_1
#define SVEN_API_CALL_1(svh, sev, api, func, p1)
#endif
#ifndef SVEN_API_CALL_2
#define SVEN_API_CALL_2(svh, sev, api, func, p1, p2)
#endif
#ifndef SVEN_API_CALL_3
#define SVEN_API_CALL_3(svh, sev, api, func, p1, p2, p3)
#endif
#ifndef SVEN_API_CALL_4
#define SVEN_API_CALL_4(svh, sev, api, func, p1, p2, p3, p4)
#endif
#ifndef SVEN_API_CALL_5
#define SVEN_API_CALL_5(svh, sev, api, func, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_API_CALL_6
#define SVEN_API_CALL_6(svh, sev, api, func, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_API_CALL_0_LOCADDR
#define SVEN_API_CALL_0_LOCADDR(svh, sev, api, func)
#endif
#ifndef SVEN_API_CALL_1_LOCADDR
#define SVEN_API_CALL_1_LOCADDR(svh, sev, api, func, p1)
#endif
#ifndef SVEN_API_CALL_2_LOCADDR
#define SVEN_API_CALL_2_LOCADDR(svh, sev, api, func, p1, p2)
#endif
#ifndef SVEN_API_CALL_3_LOCADDR
#define SVEN_API_CALL_3_LOCADDR(svh, sev, api, func, p1, p2, p3)
#endif
#ifndef SVEN_API_CALL_4_LOCADDR
#define SVEN_API_CALL_4_LOCADDR(svh, sev, api, func, p1, p2, p3, p4)
#endif
#ifndef SVEN_API_CALL_5_LOCADDR
#define SVEN_API_CALL_5_LOCADDR(svh, sev, api, func, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_API_CALL_6_LOCADDR
#define SVEN_API_CALL_6_LOCADDR(svh, sev, api, func, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_API_CALL_0_LOC16
#define SVEN_API_CALL_0_LOC16(svh, sev, f, api, func)
#endif
#ifndef SVEN_API_CALL_1_LOC16
#define SVEN_API_CALL_1_LOC16(svh, sev, f, api, func, p1)
#endif
#ifndef SVEN_API_CALL_2_LOC16
#define SVEN_API_CALL_2_LOC16(svh, sev, f, api, func, p1, p2)
#endif
#ifndef SVEN_API_CALL_3_LOC16
#define SVEN_API_CALL_3_LOC16(svh, sev, f, api, func, p1, p2, p3)
#endif
#ifndef SVEN_API_CALL_4_LOC16
#define SVEN_API_CALL_4_LOC16(svh, sev, f, api, func, p1, p2, p3, p4)
#endif
#ifndef SVEN_API_CALL_5_LOC16
#define SVEN_API_CALL_5_LOC16(svh, sev, f, api, func, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_API_CALL_6_LOC16
#define SVEN_API_CALL_6_LOC16(svh, sev, f, api, func, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_API_CALL_0_LOC32
#define SVEN_API_CALL_0_LOC32(svh, sev, f, api, func)
#endif
#ifndef SVEN_API_CALL_1_LOC32
#define SVEN_API_CALL_1_LOC32(svh, sev, f, api, func, p1)
#endif
#ifndef SVEN_API_CALL_2_LOC32
#define SVEN_API_CALL_2_LOC32(svh, sev, f, api, func, p1, p2)
#endif
#ifndef SVEN_API_CALL_3_LOC32
#define SVEN_API_CALL_3_LOC32(svh, sev, f, api, func, p1, p2, p3)
#endif
#ifndef SVEN_API_CALL_4_LOC32
#define SVEN_API_CALL_4_LOC32(svh, sev, f, api, func, p1, p2, p3, p4)
#endif
#ifndef SVEN_API_CALL_5_LOC32
#define SVEN_API_CALL_5_LOC32(svh, sev, f, api, func, p1, p2, p3, p4, p5)
#endif
#ifndef SVEN_API_CALL_6_LOC32
#define SVEN_API_CALL_6_LOC32(svh, sev, f, api, func, p1, p2, p3, p4, p5, p6)
#endif

#ifndef SVEN_API_RETURN
#define SVEN_API_RETURN(svh, sev, api, func, ret)
#endif
#ifndef SVEN_API_RETURN_LOCADDR
#define SVEN_API_RETURN_LOCADDR(svh, sev, api, func, ret)
#endif
#ifndef SVEN_API_RETURN_LOC16
#define SVEN_API_RETURN_LOC16(svh, sev, f, api, func, ret)
#endif
#ifndef SVEN_API_RETURN_LOC32
#define SVEN_API_RETURN_LOC32(svh, sev, f, api, func, ret)
#endif

#ifndef SVEN_WRITE_REG8
#define SVEN_WRITE_REG8(h, a, v)
#endif
#ifndef SVEN_WRITE_REG8_LOCADDR
#define SVEN_WRITE_REG8_LOCADDR(h, a, v)
#endif
#ifndef SVEN_WRITE_REG8_LOC16
#define SVEN_WRITE_REG8_LOC16(h, f, a, v)
#endif
#ifndef SVEN_WRITE_REG8_LOC32
#define SVEN_WRITE_REG8_LOC32(h, f, a, v)
#endif
#ifndef SVEN_WRITE_REG16
#define SVEN_WRITE_REG16(h, a, v)
#endif
#ifndef SVEN_WRITE_REG16_LOCADDR
#define SVEN_WRITE_REG16_LOCADDR(h, a, v)
#endif
#ifndef SVEN_WRITE_REG16_LOC16
#define SVEN_WRITE_REG16_LOC16(h, f, a, v)
#endif
#ifndef SVEN_WRITE_REG16_LOC32
#define SVEN_WRITE_REG16_LOC32(h, f, a, v)
#endif
#ifndef SVEN_WRITE_REG32
#define SVEN_WRITE_REG32(h, a, v)
#endif
#ifndef SVEN_WRITE_REG32_LOCADDR
#define SVEN_WRITE_REG32_LOCADDR(h, a, v)
#endif
#ifndef SVEN_WRITE_REG32_LOC16
#define SVEN_WRITE_REG32_LOC16(h, f, a, v)
#endif
#ifndef SVEN_WRITE_REG32_LOC32
#define SVEN_WRITE_REG32_LOC32(h, f, a, v)
#endif
#ifndef SVEN_WRITE_REG64
#define SVEN_WRITE_REG64(h, a, v)
#endif
#ifndef SVEN_WRITE_REG64_LOCADDR
#define SVEN_WRITE_REG64_LOCADDR(h, a, v)
#endif
#ifndef SVEN_WRITE_REG64_LOC16
#define SVEN_WRITE_REG64_LOC16(h, f, a, v)
#endif
#ifndef SVEN_WRITE_REG64_LOC32
#define SVEN_WRITE_REG64_LOC32(h, f, a, v)
#endif

#ifndef SVEN_READ_REG8
#define SVEN_READ_REG8(h, a , v)
#endif
#ifndef SVEN_READ_REG8_LOCADDR
#define SVEN_READ_REG8_LOCADDR(h, a, v)
#endif
#ifndef SVEN_READ_REG8_LOC16
#define SVEN_READ_REG8_LOC16(h, f, a, v)
#endif
#ifndef SVEN_READ_REG8_LOC32
#define SVEN_READ_REG8_LOC32(h, f, a, v)
#endif
#ifndef SVEN_READ_REG16
#define SVEN_READ_REG16(h, a, v)
#endif
#ifndef SVEN_READ_REG16_LOCADDR
#define SVEN_READ_REG16_LOCADDR(h, a, v)
#endif
#ifndef SVEN_READ_REG16_LOC16
#define SVEN_READ_REG16_LOC16(h, f, a, v)
#endif
#ifndef SVEN_READ_REG16_LOC32
#define SVEN_READ_REG16_LOC32(h, f, a, v)
#endif
#ifndef SVEN_READ_REG32
#define SVEN_READ_REG32(h, a, v)
#endif
#ifndef SVEN_READ_REG32_LOCADDR
#define SVEN_READ_REG32_LOCADDR(h, a, v)
#endif
#ifndef SVEN_READ_REG32_LOC16
#define SVEN_READ_REG32_LOC16(h, f, a, v)
#endif
#ifndef SVEN_READ_REG32_LOC32
#define SVEN_READ_REG32_LOC32(h, f, a, v)
#endif
#ifndef SVEN_READ_REG64
#define SVEN_READ_REG64(h, a, v)
#endif
#ifndef SVEN_READ_REG64_LOCADDR
#define SVEN_READ_REG64_LOCADDR(h, a, v)
#endif
#ifndef SVEN_READ_REG64_LOC16
#define SVEN_READ_REG64_LOC16(h, f, a, v)
#endif
#ifndef SVEN_READ_REG64_LOC32
#define SVEN_READ_REG64_LOC32(h, f, a, v)
#endif
#ifndef SVEN_READ_MSR
#define SVEN_READ_MSR(h, a, v)
#endif
#ifndef SVEN_READ_MSR_LOCADDR
#define SVEN_READ_MSR_LOCADDR(h, a, v)
#endif
#ifndef SVEN_READ_MSR_LOC16
#define SVEN_READ_MSR_LOC16(h, f, a, v)
#endif
#ifndef SVEN_READ_MSR_LOC32
#define SVEN_READ_MSR_LOC32(h, f, a, v)
#endif
#ifndef SVEN_TRIGGER8
#define SVEN_TRIGGER8(h, v)
#endif

/* Map CATPRINTF calls to their corresponding catalog APIs
 * by dropping the format parameter.
 */
 /**
 * @defgroup PrintfApi Printf style catalog Message Macros
 * @ingroup ApiSets
 *
 * SVEN printf style catalog message instrumentation API wrappers.
 * The below `CATPRINTF` macros calls to their corresponding catalog API
 * calls by dropping the format parameter. The API is intended to have
 * printf style instrumentation inside the sources, but drop the format
 * string part during compilation and replace it with a catalog API call
 * instead. Source scanning tools are used to extract the format strings
 * from sources into catalog dictionary files. The dictionalry files can
 * then be used to reconstruct the printf formatting during trace decode
 * time. This saves both space and bandwidth in the resulting application.
 *
 *  This API set is enabled or disabled by the
 * #SVEN_PCFG_ENABLE_CATID64_API and/or #SVEN_PCFG_ENABLE_CATID32_API
 * platform feature defines.
 * @{
 */
#define SVEN_CATPRINTF32_0(svh, sev, id, fmt)\
	SVEN_CATALOG32_0(svh, sev, id)
#define SVEN_CATPRINTF32_1(svh, sev, id, fmt, p1)\
	SVEN_CATALOG32_1(svh, sev, id, p1)
#define SVEN_CATPRINTF32_2(svh, sev, id, fmt, p1, p2)\
	SVEN_CATALOG32_2(svh, sev, id, p1, p2)
#define SVEN_CATPRINTF32_3(svh, sev, id, fmt, p1, p2, p3)\
	SVEN_CATALOG32_3(svh, sev, id, p1, p2, p3)
#define SVEN_CATPRINTF32_4(svh, sev, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG32_4(svh, sev, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF32_5(svh, sev, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG32_5(svh, sev, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF32_6(svh, sev, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG32_6(svh, sev, id, p1, p2, p3, p4, p5, p6)
#define SVEN_CATPRINTF32_0_LOCADDR(svh, sev, id, fmt)\
	SVEN_CATALOG32_0_LOCADDR(svh, sev, id)
#define SVEN_CATPRINTF32_1_LOCADDR(svh, sev, id, fmt, p1)\
	SVEN_CATALOG32_1_LOCADDR(svh, sev, id, p1)
#define SVEN_CATPRINTF32_2_LOCADDR(svh, sev, id, fmt, p1, p2)\
	SVEN_CATALOG32_2_LOCADDR(svh, sev, id, p1, p2)
#define SVEN_CATPRINTF32_3_LOCADDR(svh, sev, id, fmt, p1, p2, p3)\
	SVEN_CATALOG32_3_LOCADDR(svh, sev, id, p1, p2, p3)
#define SVEN_CATPRINTF32_4_LOCADDR(svh, sev, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG32_4_LOCADDR(svh, sev, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF32_5_LOCADDR(svh, sev, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG32_5_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF32_6_LOCADDR(svh, sev, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG32_6_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5, p6)
#define SVEN_CATPRINTF32_0_LOC16(svh, sev, f, id, fmt)\
	SVEN_CATALOG32_0_LOC16(svh, sev, f, id)
#define SVEN_CATPRINTF32_1_LOC16(svh, sev, f, id, fmt, p1)\
	SVEN_CATALOG32_1_LOC16(svh, sev, f, id, p1)
#define SVEN_CATPRINTF32_2_LOC16(svh, sev, f, id, fmt, p1, p2)\
	SVEN_CATALOG32_2_LOC16(svh, sev, f, id, p1, p2)
#define SVEN_CATPRINTF32_3_LOC16(svh, sev, f, id, fmt, p1, p2, p3)\
	SVEN_CATALOG32_3_LOC16(svh, sev, f, id, p1, p2, p3)
#define SVEN_CATPRINTF32_4_LOC16(svh, sev, f, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG32_4_LOC16(svh, sev, f, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF32_5_LOC16(svh, sev, f, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG32_5_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF32_6_LOC16(svh, sev, f, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG32_6_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
#define SVEN_CATPRINTF32_0_LOC32(svh, sev, f, id, fmt)\
	SVEN_CATALOG32_0_LOC32(svh, sev, f, id)
#define SVEN_CATPRINTF32_1_LOC32(svh, sev, f, id, fmt, p1)\
	SVEN_CATALOG32_1_LOC32(svh, sev, f, id, p1)
#define SVEN_CATPRINTF32_2_LOC32(svh, sev, f, id, fmt, p1, p2)\
	SVEN_CATALOG32_2_LOC32(svh, sev, f, id, p1, p2)
#define SVEN_CATPRINTF32_3_LOC32(svh, sev, f, id, fmt, p1, p2, p3)\
	SVEN_CATALOG32_3_LOC32(svh, sev, f, id, p1, p2, p3)
#define SVEN_CATPRINTF32_4_LOC32(svh, sev, f, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG32_4_LOC32(svh, sev, f, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF32_5_LOC32(svh, sev, f, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG32_5_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF32_6_LOC32(svh, sev, f, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG32_6_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
#define SVEN_CATPRINTF64_0(svh, sev, id, fmt)\
	SVEN_CATALOG64_0(svh, sev, id)
#define SVEN_CATPRINTF64_1(svh, sev, id, fmt, p1)\
	SVEN_CATALOG64_1(svh, sev, id, p1)
#define SVEN_CATPRINTF64_2(svh, sev, id, fmt, p1, p2)\
	SVEN_CATALOG64_2(svh, sev, id, p1, p2)
#define SVEN_CATPRINTF64_3(svh, sev, id, fmt, p1, p2, p3)\
	SVEN_CATALOG64_3(svh, sev, id, p1, p2, p3)
#define SVEN_CATPRINTF64_4(svh, sev, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG64_4(svh, sev, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF64_5(svh, sev, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG64_5(svh, sev, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF64_6(svh, sev, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG64_6(svh, sev, id, p1, p2, p3, p4, p5, p6)
#define SVEN_CATPRINTF64_0_LOCADDR(svh, sev, id, fmt)\
	SVEN_CATALOG64_0_LOCADDR(svh, sev, id)
#define SVEN_CATPRINTF64_1_LOCADDR(svh, sev, id, fmt, p1)\
	SVEN_CATALOG64_1_LOCADDR(svh, sev, id, p1)
#define SVEN_CATPRINTF64_2_LOCADDR(svh, sev, id, fmt, p1, p2)\
	SVEN_CATALOG64_2_LOCADDR(svh, sev, id, p1, p2)
#define SVEN_CATPRINTF64_3_LOCADDR(svh, sev, id, fmt, p1, p2, p3)\
	SVEN_CATALOG64_3_LOCADDR(svh, sev, id, p1, p2, p3)
#define SVEN_CATPRINTF64_4_LOCADDR(svh, sev, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG64_4_LOCADDR(svh, sev, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF64_5_LOCADDR(svh, sev, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG64_5_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF64_6_LOCADDR(svh, sev, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG64_6_LOCADDR(svh, sev, id, p1, p2, p3, p4, p5, p6)
#define SVEN_CATPRINTF64_0_LOC16(svh, sev, f, id, fmt)\
	SVEN_CATALOG64_0_LOC16(svh, sev, f, id)
#define SVEN_CATPRINTF64_1_LOC16(svh, sev, f, id, fmt, p1)\
	SVEN_CATALOG64_1_LOC16(svh, sev, f, id, p1)
#define SVEN_CATPRINTF64_2_LOC16(svh, sev, f, id, fmt, p1, p2)\
	SVEN_CATALOG64_2_LOC16(svh, sev, f, id, p1, p2)
#define SVEN_CATPRINTF64_3_LOC16(svh, sev, f, id, fmt, p1, p2, p3)\
	SVEN_CATALOG64_3_LOC16(svh, sev, f, id, p1, p2, p3)
#define SVEN_CATPRINTF64_4_LOC16(svh, sev, f, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG64_4_LOC16(svh, sev, f, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF64_5_LOC16(svh, sev, f, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG64_5_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF64_6_LOC16(svh, sev, f, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG64_6_LOC16(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
#define SVEN_CATPRINTF64_0_LOC32(svh, sev, f, id, fmt)\
	SVEN_CATALOG64_0_LOC32(svh, sev, f, id)
#define SVEN_CATPRINTF64_1_LOC32(svh, sev, f, id, fmt, p1)\
	SVEN_CATALOG64_1_LOC32(svh, sev, f, id, p1)
#define SVEN_CATPRINTF64_2_LOC32(svh, sev, f, id, fmt, p1, p2)\
	SVEN_CATALOG64_2_LOC32(svh, sev, f, id, p1, p2)
#define SVEN_CATPRINTF64_3_LOC32(svh, sev, f, id, fmt, p1, p2, p3)\
	SVEN_CATALOG64_3_LOC32(svh, sev, f, id, p1, p2, p3)
#define SVEN_CATPRINTF64_4_LOC32(svh, sev, f, id, fmt, p1, p2, p3, p4)\
	SVEN_CATALOG64_4_LOC32(svh, sev, f, id, p1, p2, p3, p4)
#define SVEN_CATPRINTF64_5_LOC32(svh, sev, f, id, fmt, p1, p2, p3, p4, p5)\
	SVEN_CATALOG64_5_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5)
#define SVEN_CATPRINTF64_6_LOC32(svh, sev, f, id, fmt, p1, p2, p3, p4, p5, p6)\
	SVEN_CATALOG64_6_LOC32(svh, sev, f, id, p1, p2, p3, p4, p5, p6)
/** @} */

#ifdef __cplusplus
}	/* extern C */
#endif
#endif
