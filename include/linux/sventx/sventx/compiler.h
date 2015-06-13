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

/*  Common defines used inside the SVENTX instrumentation library */

#ifndef SVEN_COMPILER_INCLUDED
#define SVEN_COMPILER_INCLUDED

#if defined(__cplusplus)
extern "C" {
#endif


#if defined(_WIN32)		/* MSVC Compiler section */

/* basic integer types
 */
	typedef __int8 sven_s8_t;
	typedef __int16 sven_s16_t;
	typedef __int32 sven_s32_t;
	typedef __int64 sven_s64_t;

	typedef unsigned __int8 sven_u8_t;
	typedef unsigned __int16 sven_u16_t;
	typedef unsigned __int32 sven_u32_t;
	typedef unsigned __int64 sven_u64_t;

/* shared library import/export
 */
#if defined(SVENTX_EXPORTS)
#define SVEN_EXPORT   __declspec(dllexport)
#else
#define SVEN_EXPORT   __declspec(dllimport)
#endif
#define SVEN_CALLCONV __stdcall

/* Caution: Windows doesn't support attribute based shared library
 * life time functions. Add the calls into a dllmain routine
 * instead.
 */
#define SVEN_SHAREDLIB_CONSTRUCTOR
#define SVEN_SHAREDLIB_DESTRUCTOR

#define SVEN_FUNCTION_NAME __FUNCTION__
#define SVEN_LINE          __LINE__
#define SVEN_FILE          __FILE__

#elif defined(__GNUC__)		/* GNU-C Compiler section */

/* basic integer types
 */
	typedef char sven_s8_t;
	typedef short sven_s16_t;
	typedef int sven_s32_t;
	typedef long long sven_s64_t;

	typedef unsigned char sven_u8_t;
	typedef unsigned short sven_u16_t;
	typedef unsigned int sven_u32_t;
	typedef unsigned long long sven_u64_t;

/* shared library related
 */
#define SVEN_EXPORT
#define SVEN_CALLCONV

#define SVEN_SHAREDLIB_CONSTRUCTOR __attribute__((constructor))
#define SVEN_SHAREDLIB_DESTRUCTOR  __attribute__((destructor))

#define SVEN_FUNCTION_NAME __PRETTY_FUNCTION__
#define SVEN_LINE          __LINE__
#define SVEN_FILE          __FILE__

#else

#error unknown compiler, copy and adapt one of the sections above

#endif

#ifdef __cplusplus
}				/* extern C */
#endif
#endif
