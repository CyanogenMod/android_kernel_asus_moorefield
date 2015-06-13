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

/* This CRC32 algorithm is an altered version of the ZLIB CRC32 code from
 *
 *     http://www.zlib.net/
 *
 * The original code has been modified to meet the needs of the
 * sventx library while still producing the same results.
 * The original ZLIB copyright notice follows this comment:
 */

/* crc32.c -- compute the CRC-32 of a data stream
 * Copyright (C) 1995-2006, 2010, 2011, 2012 Mark Adler
 * For conditions of distribution and use, see copyright notice in zlib.h
 *
 * zlib.h -- interface of the 'zlib' general purpose compression library
  version 1.2.7, May 2nd, 2012

  Copyright (C) 1995-2012 Jean-loup Gailly and Mark Adler

  This software is provided 'as-is', without any express or implied
  warranty.  In no event will the authors be held liable for any damages
  arising from the use of this software.

  Permission is granted to anyone to use this software for any purpose,
  including commercial applications, and to alter it and redistribute it
  freely, subject to the following restrictions:

  1. The origin of this software must not be misrepresented; you must not
     claim that you wrote the original software. If you use this software
     in a product, an acknowledgment in the product documentation would be
     appreciated but is not required.
  2. Altered source versions must be plainly marked as such, and must not be
     misrepresented as being the original software.
  3. This notice may not be removed or altered from any source distribution.

  Jean-loup Gailly        Mark Adler
  jloup@gzip.org          madler@alumni.caltech.edu


  The data format used by the zlib library is described by RFCs (Request for
  Comments) 1950 to 1952 in the files http://tools.ietf.org/html/rfc1950
  (zlib format), rfc1951 (deflate format) and rfc1952 (gzip format).
*/
#ifndef SVEN_CRC32_INCLUDED
#define SVEN_CRC32_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
	extern const sven_u32_t crc_table[256];

#if !defined(SVEN_PCFG_ENABLE_CHECKSUM)

#define SVEN_CRC32_INIT(v)
#define SVEN_CRC32_GET(v)

#define SVEN_CRC32_U8(crc, v)
#define SVEN_CRC32_U16(crc, v)
#define SVEN_CRC32_U32(crc, v)
#define SVEN_CRC32_U64(crc, v)

#else

#define SVEN_CRC32_INIT(v) ((v)^0xffffffff)
#define SVEN_CRC32_GET(v)  ((v)^0xffffffff)

#define SVEN_CRC32_U8(crc, v)   { crc = sventx_crc32_8((crc), (v));  }
#define SVEN_CRC32_U16(crc, v)  { crc = sventx_crc32_16((crc), (v)); }
#define SVEN_CRC32_U32(crc, v)  { crc = sventx_crc32_32((crc), (v)); }
#define SVEN_CRC32_U64(crc, v)  { crc = sventx_crc32_64((crc), (v)); }

#if !defined(SVEN_PCFG_ENABLE_INLINE)

SVEN_INLINE sven_u32_t sventx_crc32_8(sven_u32_t crc, sven_u8_t b);
SVEN_INLINE sven_u32_t sventx_crc32_16(sven_u32_t crc, sven_u16_t hw);
SVEN_INLINE sven_u32_t sventx_crc32_32(sven_u32_t crc, sven_u32_t hw);
SVEN_INLINE sven_u32_t sventx_crc32_64(sven_u32_t crc, sven_u64_t hw);

#else

SVEN_INLINE sven_u32_t sventx_crc32_8(sven_u32_t crc, sven_u8_t b)
{
	return crc_table[((int) crc ^ b) & 0xff] ^ (crc >> 8);
}

SVEN_INLINE sven_u32_t sventx_crc32_16(sven_u32_t crc, sven_u16_t hw)
{
	crc = sventx_crc32_8(crc, (sven_u8_t) hw);
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 8));

	return crc;
}

SVEN_INLINE sven_u32_t sventx_crc32_32(sven_u32_t crc, sven_u32_t hw)
{
	crc = sventx_crc32_8(crc, (sven_u8_t) hw);
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 8));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 16));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 24));

	return crc;
}

SVEN_INLINE sven_u32_t sventx_crc32_64(sven_u32_t crc, sven_u64_t hw)
{
	crc = sventx_crc32_8(crc, (sven_u8_t) hw);
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 8));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 16));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 24));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 32));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 40));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 48));
	crc = sventx_crc32_8(crc, (sven_u8_t) (hw >> 56));

	return crc;
}
#endif

#endif		/* defined(SVEN_PCFG_ENABLE_CHECKSUM) */

#ifdef __cplusplus
}		/* extern C */
#endif

#endif
