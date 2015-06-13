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

#ifndef SVEN_EVENT_INCLUDED
#define SVEN_EVENT_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/**
 * Catalog ID container
 */
typedef union u_sven_catid {
	sven_u32_t sci_32;
	sven_u64_t sci_64;
} sven_catid_t;

/**
 * SVEN event descriptor
 *
 *  This structure stores a SVEN event in "logical" memory format.
 *  Logical means that all optional fields are present but not necessarily
 *  used. Variable length payloads are addressed through a pointer and
 *  are not copied into the structure.
 */
typedef struct s_sven_eventdsc {
	sven_event_tag_t ed_tag;
			     /**< 32bit event tag  (mandatory)      */
#if defined(SVEN_PCFG_ENABLE_ORIGIN_GUID)
	sven_guid_t ed_guid;/**< origin GUID 128 bit ID (optional)  */
#endif
#if defined(SVEN_PCFG_ENABLE_SEQUENCE_COUNT)
	sven_u32_t ed_seq;   /**< event sequence number (optional)  */
#endif
#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)
	sven_eventlocation_t ed_loc;
			     /**< event source location (optional)  */
#endif
	sven_u16_t ed_len;   /**< variable payload length (optional)*/

#define SVEN_EVENT_PAYLOADSIZE  24   /**< SVEN 1.0 payload length */
	/** event payload (event type != SVEN_Event_type_short) */
	union {
		sven_u8_t data_u8[SVEN_EVENT_PAYLOADSIZE /
				  sizeof(sven_u8_t)];
		sven_u32_t data_u32[SVEN_EVENT_PAYLOADSIZE /
				    sizeof(sven_u32_t)];
		sven_u64_t data_u64[SVEN_EVENT_PAYLOADSIZE /
				    sizeof(sven_u64_t)];

		struct {
			sven_u32_t api[2];
			sven_u64_t ret;
			sven_u32_t *param;
		} data_api;

		struct {
			sven_catid_t id;
			sven_u32_t *param;
		} data_catid;

		const void *data_var;
			     /**< variable length payload           */
	} ed_pld;
	sven_u32_t ed_chk;   /**< event checksum (optional)         */
} sven_eventdsc_t, *psven_eventdsc_t;

#if defined(SVEN_PCFG_ENABLE_64BIT_ADDR)
#define SVEN_EVDSC_MEMBER_OFF(m)\
	((sven_u16_t)(sven_u64_t)&(((psven_eventdsc_t)0)->m))
#else
#define SVEN_EVDSC_MEMBER_OFF(m)\
	((sven_u16_t)(sven_u32_t)&(((psven_eventdsc_t)0)->m))
#endif

/**
 * SVEN event scatter write operations
 */
enum u_sven_scatter_op {
	SVEN_SCATTER_OP_SKIP = 0x00,
				/**< skip sso_length bytes          */

	SVEN_SCATTER_OP_8BIT = 0x01,
				/**< write sso_length times 8 bit   */
	SVEN_SCATTER_OP_16BIT = 0x02,
				/**< write sso_length times 16 bit  */
	SVEN_SCATTER_OP_32BIT = 0x04,
				/**< write sso_length times 32 bit  */
	SVEN_SCATTER_OP_64BIT = 0x08,
				/**< write sso_length times 64 bit  */

	SVEN_SCATTER_OP_BLOB = 0x10,
				/**< write sso_length bytes that are
				  *  accessed through a pointer   */
	SVEN_SCATTER_OP_END = 0xFF
				/**< end of scatter writer program  */
};

/**
 * SVEN event scatter write instruction definition
 */
typedef struct s_sven_scatter_prog {
	sven_u8_t sso_opcode;	/**< scatter write operation
				 *   @see u_sven_scatter_op            */
	sven_u8_t sso_offset;	/**< data offset in SVEN descriptor    */
	sven_u16_t sso_length;
				/**< repeat count for sso_opcode       */
} sven_scatter_prog_t, *psven_scatter_prog_t;

#define SVEN_SCATTER_PROG_LEN   10    /**< maximum needed scatter prog size   */

#if defined(SVEN_PCFG_ENABLE_DEFAULT_SCATTER_WRITE)
/* default scatter write routine */
extern void sventx_sth_scatter_write(psven_handle_t svenh,
		psven_scatter_prog_t scatterprog,
		const void *pdesc);
#endif

#ifdef __cplusplus
}	/* extern C */
#endif
#endif
