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
#include "sventx/crc32.h"

#if defined(SVEN_PCFG_ENABLE_DEFAULT_SCATTER_WRITE)

/** SVEN scatter write routine
 *
 * This function implements the scatter write algorithm that translates
 * the logical SVEN memory descriptor representation into output
 * requests to the STH. The actual output routines are defined through
 * the SVEN_STH_OUT_* definitions from the platform header file.
 *
 * @param svenh used sven handle
 * @param scatterprog event content write instructions
 * @param pdesc pointer to memory area with event data
 */
void sventx_sth_scatter_write(psven_handle_t svenh,
		      psven_scatter_prog_t scatterprog, const void *pdesc)
{
	int repeat;

	/* Define an "any" size integer pointer to avoid casts and to simplify
	 * type based incrementing
	 */
	union {
		const void *vp;
		const sven_u8_t *bp;
		const sven_u16_t *hp;
		const sven_u32_t *wp;
		const sven_u64_t *dp;
	} data_ptr;

#if defined(SVEN_PCFG_ENABLE_CHECKSUM)
#define IFDO(a, b)  { if (a) do { b; } while (0); }

	sven_u32_t crc;
	int use_crc;

	use_crc = svenh->svh_tag.et_chksum;
	crc = SVEN_CRC32_INIT(0);
#else

#define IFDO(a, b)

#endif

	/* Write the "always" present tag field as a time-stamped D32 */
	SVEN_STH_OUT_D32TS(svenh, *(sven_u32_t *) pdesc);

	IFDO(use_crc, SVEN_CRC32_U32(crc, *(sven_u32_t *) pdesc));

	/* Run the event scatter write program to dump the event contents
	 */
	while (scatterprog->sso_opcode != SVEN_SCATTER_OP_END) {
		repeat = scatterprog->sso_length;
		data_ptr.vp = pdesc;
		data_ptr.bp += scatterprog->sso_offset;

		switch (scatterprog->sso_opcode) {

		case SVEN_SCATTER_OP_8BIT:
			do {
				SVEN_STH_OUT_D8(svenh, *data_ptr.bp);
				IFDO(use_crc,
				     SVEN_CRC32_U8(crc, *data_ptr.bp));
				++data_ptr.bp;
			} while (--repeat);
			break;

		case SVEN_SCATTER_OP_16BIT:
			do {
				SVEN_STH_OUT_D16(svenh, *data_ptr.hp);
				IFDO(use_crc,
				     SVEN_CRC32_U16(crc, *data_ptr.hp));
				++data_ptr.hp;
			} while (--repeat);
			break;

		case SVEN_SCATTER_OP_32BIT:
			do {
				SVEN_STH_OUT_D32(svenh, *data_ptr.wp);
				IFDO(use_crc,
				     SVEN_CRC32_U32(crc, *data_ptr.wp));
				++data_ptr.wp;
			} while (--repeat);
			break;

#if defined(SVEN_PCFG_ENABLE_64BIT_IO)

		case SVEN_SCATTER_OP_64BIT:
			do {
				SVEN_STH_OUT_D64(svenh, *data_ptr.dp);
				IFDO(use_crc,
				     SVEN_CRC32_U64(crc, *data_ptr.dp));
				++data_ptr.dp;
			} while (--repeat);
			break;
#endif

		case SVEN_SCATTER_OP_BLOB:
			{
				/* data location is pointer to real data,
				 * not data itself
				 */
				data_ptr.vp = *(void **) data_ptr.vp;

#if defined(SVEN_PCFG_ENABLE_64BIT_IO)

				while (repeat >= sizeof(sven_u64_t)) {
					SVEN_STH_OUT_D64(svenh,
							 *data_ptr.dp);
					IFDO(use_crc,
					     SVEN_CRC32_U64(crc,
							    *data_ptr.dp));
					++data_ptr.dp;
					repeat -= sizeof(sven_u64_t);
				}

				if (repeat >= sizeof(sven_u32_t)) {
					SVEN_STH_OUT_D32(svenh,
							 *data_ptr.wp);
					IFDO(use_crc,
					     SVEN_CRC32_U32(crc,
							    *data_ptr.wp));
					++data_ptr.wp;
					repeat -= sizeof(sven_u32_t);
				}
#else
				while (repeat >= sizeof(sven_u32_t)) {
					SVEN_STH_OUT_D32(svenh,
							 *data_ptr.wp);
					IFDO(use_crc,
					     SVEN_CRC32_U32(crc,
							    *data_ptr.wp));
					++data_ptr.wp;
					repeat -= sizeof(sven_u32_t);
				}
#endif
				if (repeat >= sizeof(sven_u16_t)) {
					SVEN_STH_OUT_D16(svenh,
							 *data_ptr.hp);
					IFDO(use_crc,
					     SVEN_CRC32_U16(crc,
							    *data_ptr.hp));
					++data_ptr.hp;
					repeat -= sizeof(sven_u16_t);
				}

				if (repeat) {
					SVEN_STH_OUT_D8(svenh,
							*data_ptr.bp);
					IFDO(use_crc,
					     SVEN_CRC32_U8(crc,
							   *data_ptr.bp));
				}
			}
			break;

		}
		++scatterprog;
	}

#if defined(SVEN_PCFG_ENABLE_CHECKSUM)
	if (use_crc) {
		crc = SVEN_CRC32_GET(crc);
		SVEN_STH_OUT_D32(svenh, crc);
	}
#endif

	/* EVENT end of record mark */
	SVEN_STH_OUT_FLAG(svenh);
}

#endif /* defined(SVEN_PCFG_ENABLE_DEFAULT_SCATTER_WRITE) */
