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

#if defined(SVEN_UNIT_TEST)
#define ASSERT_CHECK(x) ASSERT_EQ(x, true)
#else
#define ASSERT_CHECK(x)
#endif

#if !defined(SVEN_SCATTER_WRITE)
/**
 * Default writer access is to call global state svh_writer pointer.
 * Redefine this if you can avoid the function pointer overhead.
 */
#define SVEN_SCATTER_WRITE(sven_handle, scatter_prog, data_ptr) \
	{ \
		(sven_handle)->svh_header->svh_writer( \
				(sven_handle), (scatter_prog), (data_ptr));\
	}
#endif

/**
 * predefined write scatter instructions defined in scatter_op table
 */
enum sven_scatter_ops {
#if defined(SVEN_PCFG_ENABLE_ORIGIN_GUID)
	SCATTER_OP_GUID,
#endif

#if defined(SVEN_PCFG_ENABLE_SEQUENCE_COUNT)
	SCATTER_OP_SEQUENCE,
#endif
#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)
	SCATTER_OP_LOC_FMT,
	SCATTER_OP_LOC_32,
	SCATTER_OP_LOC_64,
#endif
	SCATTER_OP_LENGTH,
	SCATTER_OP_PAYLD_FIX,
	SCATTER_OP_PAYLD_VAR,
	SCATTER_OP_CHECKSUM,
	SCATTER_OP_CATID_32,
	SCATTER_OP_CATID_64,
	SCATTER_OP_CATID_ARGS,
	SCATTER_OP_API,
	SCATTER_OP_API_RET,
	SCATTER_OP_API_ARGS,

	SCATTER_OP_END
};

/**
 * Scatter instruction that describe the writing of SVEN event descriptor
 * members by the scatter write algorithm.
 */
static const sven_scatter_prog_t scatter_ops[] = {
#if defined(SVEN_PCFG_ENABLE_ORIGIN_GUID)

#if defined(SVEN_PCFG_ENABLE_64BIT_IO)
	{			/* SCATTER_OP_GUID */
	 SVEN_SCATTER_OP_64BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_guid),
	 2},
#else
	{			/* SCATTER_OP_GUID */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_guid),
	 4},
#endif
#endif

#if defined(SVEN_PCFG_ENABLE_SEQUENCE_COUNT)
	{			/* SCATTER_OP_SEQUENCE */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_seq),
	 1},
#endif

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)
	{			/* SCATTER_OP_LOC_FMT */
	 SVEN_SCATTER_OP_8BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_loc.el_format),
	 1},
	{			/* SCATTER_OP_LOC_32 */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_loc.el_u),
	 1},
#if defined(SVEN_PCFG_ENABLE_64BIT_IO)
	{			/* SCATTER_OP_LOC_64 */
	 SVEN_SCATTER_OP_64BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_loc.el_u),
	 1},
#else
	{			/* SCATTER_OP_LOC_64 */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_loc.el_u),
	 2},
#endif
#endif

	{			/* SCATTER_OP_LENGTH */
	 SVEN_SCATTER_OP_16BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_len),
	 1},
#if defined(SVEN_PCFG_ENABLE_64BIT_IO)
	{			/* SCATTER_OP_PAYLD_FIX */
	 SVEN_SCATTER_OP_64BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld),
	 SVEN_EVENT_PAYLOADSIZE / sizeof(sven_u64_t)
	 }
	,
#else
	{			/* SCATTER_OP_PAYLD_FIX */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld),
	 SVEN_EVENT_PAYLOADSIZE / sizeof(sven_u32_t)
	 }
	,
#endif
	{			/* SCATTER_OP_PAYLD_VAR */
	 SVEN_SCATTER_OP_BLOB,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_var),
	 0}
	,
	{			/* SCATTER_OP_PAYLD_CHECKSUM */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_chk),
	 1}
	,
	{			/* SCATTER_OP_CATID_32 */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_catid.id.sci_32),
	 1}
	,
#if defined(SVEN_PCFG_ENABLE_64BIT_IO)
	{			/* SCATTER_OP_CATID_64 */
	 SVEN_SCATTER_OP_64BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_catid.id.sci_64),
	 1}
	,
#else
	{			/* SCATTER_OP_CATID_64 */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_catid.id.sci_64),
	 2}
	,
#endif
	{			/* SCATTER_OP_CATID_ARGS */
	 SVEN_SCATTER_OP_BLOB,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_catid.param),
	 0}
	,
#if defined(SVEN_PCFG_ENABLE_64BIT_IO)
	{			/* SCATTER_OP_API */
	 SVEN_SCATTER_OP_64BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_api.api),
	 1}
	,
	{			/* SCATTER_OP_API_RET */
	 SVEN_SCATTER_OP_64BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_api.ret),
	 1}
	,
#else
	{			/* SCATTER_OP_API */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_api.api),
	 2}
	,
	{			/* SCATTER_OP_API_RET */
	 SVEN_SCATTER_OP_32BIT,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_api.ret),
	 2}
	,
#endif
	{			/* SCATTER_OP_API_ARGS */
	 SVEN_SCATTER_OP_BLOB,
	 SVEN_EVDSC_MEMBER_OFF(ed_pld.data_api.param),
	 0}
	,
	{			/* SCATTER_OP_END */
	 SVEN_SCATTER_OP_END,
	 0,
	 0}
	,
};

#if defined(SVEN_PCFG_ENABLE_CATID32_API) ||\
	defined(SVEN_PCFG_ENABLE_CATID64_API) || \
	defined(SVEN_PCFG_ENABLE_STRING_API) || \
	defined(SVEN_PCFG_ENABLE_WRITE_API) || \
	defined(SVEN_PCFG_ENABLE_API_API) || \
	defined(SVEN_PCFG_ENABLE_DEVH_API) || \
	defined(SVEN_PCFG_ENABLE_REGISTER_API)

/**
 * Add optional event components to the event descriptor
 */
static void
#if defined(SVEN_PCFG_ENABLE_INLINE)
inline
#endif
insert_optional_event_components(psven_handle_t svh,
				 psven_eventlocation_t loc,
				 psven_eventdsc_t desc,
				 psven_scatter_prog_t *prog_ptr)
{
	psven_scatter_prog_t prog = *prog_ptr;

#if defined(SVEN_PCFG_ENABLE_ORIGIN_GUID)
	/* origin GUID  ? */
	if (desc->ed_tag.et_module == SVEN_MODULE_IS_GUID) {
		desc->ed_guid = svh->svh_guid;
		*prog++ = scatter_ops[SCATTER_OP_GUID];
	}
#endif

#if defined(SVEN_PCFG_ENABLE_SEQUENCE_COUNT)
	/* Sequence # */
	if (desc->ed_tag.et_sequence) {
		desc->ed_seq = svh->svh_sequence_count++;
		*prog++ = scatter_ops[SCATTER_OP_SEQUENCE];
	}
#endif

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)
	/* location information ? */
	if ((psven_eventlocation_t) 0 != loc) {
		desc->ed_loc = *loc;
		desc->ed_tag.et_location = 1;

		*prog++ = scatter_ops[SCATTER_OP_LOC_FMT];
		if (desc->ed_loc.el_format.elt_size)
			*prog++ = scatter_ops[SCATTER_OP_LOC_64];
		else
			*prog++ = scatter_ops[SCATTER_OP_LOC_32];
	}
#endif

	*prog_ptr = prog;
}
#endif


#if defined(SVEN_PCFG_ENABLE_STRING_API)

/**
 * Write a string output SVEN event
 *
 * @param svh SVEN handle
 * @param loc Pointer to instrumentation location or null if no location
 * @param type string event subtype
 * @param severity severity level (0..7)
 * @param len number of bytes to emit or 0 to send fixed size 32bytes
 * @param str  pointer to UTF-8 string bytes
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_write_debug_string(psven_handle_t svh,
			  psven_eventlocation_t loc,
			  sven_eventtype_debugstr_t type,
			  sven_severity_t severity,
			  sven_u16_t len, const char *str)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_type = SVEN_event_type_debug_string;
	desc.ed_tag.et_subtype = type;
	desc.ed_tag.et_severity = severity;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	/* fixed or variable sized payload */
	if (0 == len) {
		/* `strncpy' copies not more than LENGTH characters from the
		 * the string pointed to by SRC (including the terminating null
		 * character) to the array pointed to by DST.  If the string
		 * pointed to by SRC is shorter than LENGTH characters, null
		 * characters are appended to the destination array until a
		 * total of LENGTH characters have been written.
		 */
		while (len < SVEN_EVENT_PAYLOADSIZE) {
			desc.ed_pld.data_u8[len] = str[len];
			if ('\0' == desc.ed_pld.data_u8[len])
				break;
			++len;
		}
		while (len < SVEN_EVENT_PAYLOADSIZE)
			desc.ed_pld.data_u8[len++] = 0;

		*prog_ptr++ = scatter_ops[SCATTER_OP_PAYLD_FIX];

	} else {
		/* variable sized string payload */
		desc.ed_tag.et_length = 1;
		desc.ed_len = len;
		*prog_ptr++ = scatter_ops[SCATTER_OP_LENGTH];

		desc.ed_pld.data_var = (const sven_u8_t *) str;
		*prog_ptr = scatter_ops[SCATTER_OP_PAYLD_VAR];
		prog_ptr->sso_length = len;
		++prog_ptr;
	}

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	/* call IO routine to dump out the event */
	SVEN_SCATTER_WRITE(svh, prog, &desc);
}
#endif				/* #if defined(SVEN_PCFG_ENABLE_STRING_API) */

#if defined(SVEN_PCFG_ENABLE_CATID64_API)

/**
 * Write catalog message SVEN event
 *
 * @param svh  SVEN handle
 * @param loc  Pointer to instrumentation location or null
 * @param severity message severity level (0..7)
 * @param catid catalog ID
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_write_catalog64_message(const psven_handle_t svh,
			       psven_eventlocation_t loc,
			       sven_severity_t severity, sven_u64_t catid)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;
	sven_u16_t paramlen =
	    svh->svh_param_count * (sven_u16_t) sizeof(sven_u32_t);

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_type = SVEN_event_type_catalog_msg;
	desc.ed_tag.et_severity = severity;
	desc.ed_tag.et_length = 1;
	desc.ed_tag.et_subtype = SVEN_CATID_64;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	desc.ed_len = sizeof(catid) + paramlen;
	*prog_ptr++ = scatter_ops[SCATTER_OP_LENGTH];

	/* cat ID */
	desc.ed_pld.data_catid.id.sci_64 = catid;
	*prog_ptr++ = scatter_ops[SCATTER_OP_CATID_64];

	/* parameters (if any) */

	if (0 != paramlen) {
		desc.ed_pld.data_catid.param = svh->svh_param;
		*prog_ptr = scatter_ops[SCATTER_OP_CATID_ARGS];
		prog_ptr->sso_length = paramlen;
		++prog_ptr;
	}

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	/* call IO routine to dump out the event */
	SVEN_SCATTER_WRITE(svh, prog, &desc);
}

#endif				/* #if defined(SVEN_PCFG_ENABLE_CATID64_API) */

#if defined(SVEN_PCFG_ENABLE_CATID32_API)

/**
 * Write catalog message SVEN event
 *
 * @param svh  SVEN handle
 * @param loc  Pointer to instrumentation location or null
 * @param severity message severity level (0..7)
 * @param catid catalog ID
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_write_catalog32_message(const psven_handle_t svh,
			       psven_eventlocation_t loc,
			       sven_severity_t severity, sven_u32_t catid)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;
	sven_u16_t paramlen =
	    svh->svh_param_count * (sven_u16_t) sizeof(sven_u32_t);

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_type = SVEN_event_type_catalog_msg;
	desc.ed_tag.et_severity = severity;
	desc.ed_tag.et_length = 1;
	desc.ed_tag.et_subtype = SVEN_CATID_32;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	desc.ed_len = sizeof(catid) + paramlen;
	*prog_ptr++ = scatter_ops[SCATTER_OP_LENGTH];

	/* cat ID */
	desc.ed_pld.data_catid.id.sci_32 = catid;
	*prog_ptr++ = scatter_ops[SCATTER_OP_CATID_32];

	/* parameters (if any) */

	if (0 != paramlen) {
		desc.ed_pld.data_catid.param = svh->svh_param;
		*prog_ptr = scatter_ops[SCATTER_OP_CATID_ARGS];
		prog_ptr->sso_length = paramlen;
		++prog_ptr;
	}

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	/* call IO routine to dump out the event */
	SVEN_SCATTER_WRITE(svh, prog, &desc);
}

#endif				/* #if defined(SVEN_PCFG_ENABLE_CATID32_API) */


#if defined(SVEN_PCFG_ENABLE_WRITE_API)

/**
 * Write raw data SVEN event
 *
 * @param svh  SVEN handle
 * @param loc  pointer to instrumentation location or null
 * @param severity message severity level (0..7)
 * @param protocol content protocol ID
 * @param data pointer to raw data
 * @param length number of bytes to send
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_write_raw_message(const psven_handle_t svh,
			 psven_eventlocation_t loc,
			 sven_severity_t severity,
			 sven_u8_t protocol,
			 const void *data, sven_u16_t length)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_type = SVEN_event_type_module_specific;
	desc.ed_tag.et_length = 1;
	desc.ed_tag.et_severity = severity;
	desc.ed_tag.et_subtype = protocol;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	desc.ed_len = length;
	*prog_ptr++ = scatter_ops[SCATTER_OP_LENGTH];

	desc.ed_pld.data_var = data;
	*prog_ptr = scatter_ops[SCATTER_OP_PAYLD_VAR];
	prog_ptr->sso_length = length;
	++prog_ptr;

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	/* call IO routine to dump out the event */
	SVEN_SCATTER_WRITE(svh, prog, &desc);
}
#endif				/* defined(SVEN_PCFG_ENABLE_WRITE_API) */

#if defined(SVEN_PCFG_ENABLE_API_API)

/**
 * Write API call messages with up to 6 parameters
 *
 * @param svh  SVEN handle
 * @param loc  pointer to instrumentation location or null
 * @param severity message severity level (0..7)
 * @param api API set number
 * @param function  function id inside API
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_write_api_call_message(const psven_handle_t svh,
			      psven_eventlocation_t loc,
			      sven_severity_t severity,
			      sven_u32_t api, sven_u32_t function)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;
	sven_u16_t paramlen =
	    svh->svh_param_count * (sven_u16_t) sizeof(sven_u32_t);

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_severity = severity;
	desc.ed_tag.et_type = SVEN_event_type_API;
	desc.ed_tag.et_length = 1;
	desc.ed_tag.et_subtype = SVEN_EV_API_FunctionCalled;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	desc.ed_len = sizeof(sven_u64_t) + paramlen;
	*prog_ptr++ = scatter_ops[SCATTER_OP_LENGTH];

	desc.ed_pld.data_api.api[0] = api;
	desc.ed_pld.data_api.api[1] = function;
	*prog_ptr++ = scatter_ops[SCATTER_OP_API];

	/* parameters (if any) */

	if (0 != paramlen) {
		desc.ed_pld.data_api.param = svh->svh_param;
		*prog_ptr = scatter_ops[SCATTER_OP_API_ARGS];
		prog_ptr->sso_length = paramlen;
		++prog_ptr;
	}

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	/* call IO routine to dump out the event */
	SVEN_SCATTER_WRITE(svh, prog, &desc);
}

/**
 * Write API return messages
 *
 * @param svh  SVEN handle
 * @param loc  Pointer to instrumentation location or null
 * @param severity message severity level (0..7)
 * @param api API set number
 * @param function function id inside API
 * @param ret return value of API
 */
SVEN_EXPORT void SVEN_CALLCONV
sventx_write_api_return_message(const psven_handle_t svh,
				psven_eventlocation_t loc,
				sven_severity_t severity,
				sven_u32_t api,
				sven_u32_t function, sven_u64_t ret)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_type = SVEN_event_type_API;
	desc.ed_tag.et_subtype = SVEN_EV_API_FunctionReturned;
	desc.ed_tag.et_length = 1;
	desc.ed_tag.et_severity = severity;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	desc.ed_len = 2 * sizeof(sven_u64_t);
	*prog_ptr++ = scatter_ops[SCATTER_OP_LENGTH];

	desc.ed_pld.data_api.api[0] = api;
	desc.ed_pld.data_api.api[1] = function;
	*prog_ptr++ = scatter_ops[SCATTER_OP_API];

	desc.ed_pld.data_api.ret = ret;
	*prog_ptr++ = scatter_ops[SCATTER_OP_API_RET];

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	/* call IO routine to dump out the event */
	SVEN_SCATTER_WRITE(svh, prog, &desc);
}
#endif				/* defined(SVEN_PCFG_ENABLE_WRITE_API) */

#if defined(SVEN_PCFG_ENABLE_REGISTER_API)

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_32bit_regio_message(const psven_handle_t svh,
				 psven_eventlocation_t loc,
				 sven_eventtype_regio_t reg_io_type,
				 sven_phyaddr_t reg_addr,
				 sven_u32_t reg_value, sven_u32_t reg_mask)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_subtype = reg_io_type;
	desc.ed_tag.et_type = SVEN_event_type_register_io;
	desc.ed_tag.et_length = 0;
	desc.ed_tag.et_severity = SVEN_SEVERITY_NONE;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	desc.ed_pld.data_u64[0] = reg_addr;
	desc.ed_pld.data_u64[1] = reg_value;
	desc.ed_pld.data_u64[2] = reg_mask;
	*prog_ptr++ = scatter_ops[SCATTER_OP_PAYLD_FIX];

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	SVEN_SCATTER_WRITE(svh, prog, &desc);
}

SVEN_EXPORT void SVEN_CALLCONV
sventx_write_64bit_regio_message(const psven_handle_t svh,
				 psven_eventlocation_t loc,
				 sven_eventtype_regio_t reg_io_type,
				 sven_phyaddr_t reg_addr,
				 sven_u64_t reg_value, sven_u64_t reg_mask)
{
	sven_eventdsc_t desc;
	sven_scatter_prog_t prog[SVEN_SCATTER_PROG_LEN];
	psven_scatter_prog_t prog_ptr = prog;

	if (&sventx_null_handle == svh)
		return;

	/* assign tag */
	desc.ed_tag = svh->svh_tag;
	desc.ed_tag.et_subtype = reg_io_type;
	desc.ed_tag.et_type = SVEN_event_type_register_io;
	desc.ed_tag.et_length = 0;
	desc.ed_tag.et_severity = SVEN_SEVERITY_NONE;

	insert_optional_event_components(svh, loc, &desc, &prog_ptr);

	desc.ed_pld.data_u64[0] = reg_addr;
	desc.ed_pld.data_u64[1] = reg_value;
	desc.ed_pld.data_u64[2] = reg_mask;
	*prog_ptr++ = scatter_ops[SCATTER_OP_PAYLD_FIX];

	*prog_ptr = scatter_ops[SCATTER_OP_END];

	ASSERT_CHECK(prog_ptr < &prog[SVEN_SCATTER_PROG_LEN]);

	SVEN_SCATTER_WRITE(svh, prog, &desc);
}

#endif				/* defined (SVEN_PCFG_ENABLE_REGISTER_API) */
