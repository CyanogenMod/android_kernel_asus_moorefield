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

/* Example platform specific extensions
 * This "platform" shows how to use the SVEN platform modules
 * to configure the SVEN feature set and add platform specific
 * customizations.
 */

#ifndef SVEN_PLATFORM_CONFIG_INCLUDED
#define SVEN_PLATFORM_CONFIG_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @defgroup PCFG_Config  Platform Feature Configuration Defines
 *
 * Defines to customize the SVENTX feature set to match the platform needs.
 *
 * Each optional library feature can be disabled by not defining the related
 * PCFG_ENABLE define. Removing unused features reduces both memory footprint
 * and runtime overhead of SVENTX.
 */

/**
 * @defgroup PCFG_Global Platform wide configuration
 * @ingroup  PCFG_Config
 *
 * These defines enable global features in the SVENTX library.
 * @{
 */

/**
 * Extend Platform global SVENTX data state
 *
 * This define extends the global SVENTX state singleton data structure
 * #sven_header_t with platform private content. A platform typically
 * stores data for SVEN handle creation processing in this structure.
 *
 * Note: This data is not touched by SVENTX itself, but typically is during
 * platform handle creation and destruction hook function calls. **These
 * calls are not lock protected and may happen concurrently!**. The hook
 * functions need to implement locking if they modify the platform state data.
 *
 * The platform example uses #sven_platform_state_t as data state extension.
 */
#if !defined(__KERNEL__)
#define SVEN_PCFG_ENABLE_PLATFORM_STATE_DATA
#endif
/**
 * Extend SVENTX handle data state
 *
 * This define extends the SVENTX handle state data structure
 * #sven_handle_t with platform private content. A platform typically
 * stores data for fast STH access into the handle data, for example a
 * volatile pointer to the STH MMIO space.
 *
 * The platform example uses #sven_platform_handle_t as handle state extension.
 */
#if !defined(__KERNEL__)
#define SVEN_PCFG_ENABLE_PLATFORM_HANDLE_DATA
#endif
/**
 * Use SVENTX STH scatter write output function
 *
 * SVENTX comes with a output routine that is optimized to write data out to
 * the STH MMIO space. It simplifies a SVENTX platform integration as
 * only low level STH access macros must be provided for outputting data.
 *
 * These low level output macros are:
 * #SVEN_STH_OUT_D32MTS, SVEN_STH_OUT_D32TS, #SVEN_STH_OUT_D64,
 * #SVEN_STH_OUT_D32, #SVEN_STH_OUT_D16, #SVEN_STH_OUT_D8 and
 * #SVEN_STH_OUT_FLAG
 */
#define SVEN_PCFG_ENABLE_DEFAULT_SCATTER_WRITE

/**
 * Enable HEAP usage for handle generation
 *
 * This macro tells the SVENTX library to enable the heap allocation handle
 * creation API #SVEN_ALLOC_HANDLE.
 * The platform must provide the macros #SVEN_HEAP_MALLOC and #SVEN_HEAP_FREE
 * to point SVENTX to the platform malloc and free functions.
 *
 * Note: You must use unpaged memory allocation functions for OS kernel space
 * environments.
 */
#define SVEN_PCFG_ENABLE_HEAP_MEMORY

/* MSVC and GNU compiler 64bit mode */
#if defined(_WIN64) || defined(__x86_64__)
/**
 * Enable 64bit instruction addresses
 *
 * Set this define if running in 64bit code address space.
 */
#define SVEN_PCFG_ENABLE_64BIT_ADDR
#endif
/**
 * Enable atomic 64bit write operations
 *
 * Set this define if your platform supports an atomic 64 Bit data write
 * operation. This results in fewer IO accesses to the STH.The SVENTX library
 * defaults to 2 consecutive 32 Bit writes otherwise.
 */
#if defined(_WIN64) || defined(__x86_64__)
#define SVEN_PCFG_ENABLE_64BIT_IO
#endif

/**
 * Enable helper function code inlining
 *
 * Set this define if speed is more important then code size on your platform.
 * It causes several helper function to get inlined, producing faster, but
 * also larger code.
 */
#define SVEN_PCFG_ENABLE_INLINE

#define SVEN_PCFG_ENABLE_HOT_GATE	/* experimental, not used right now */

/** @} */

/**
 * @defgroup PCFG_ApiSet Supported API sets
 * @ingroup  PCFG_Config
 *
 * These defines enable API sets in the SVENTX library.
 * @{
 */

/**
 * Enable the Catalog API for 32 Bit Catalog IDs.
 */
#define SVEN_PCFG_ENABLE_CATID32_API

/**
 * Enable the Catalog API for 64 Bit Catalog IDs.
 */
#define SVEN_PCFG_ENABLE_CATID64_API	    /**< 64 bit catid API support   */

/**
 * Enable plain UTF-8 string output APIs.
 */
#define SVEN_PCFG_ENABLE_STRING_API

/**
 * Enable support for legacy SVEN 1.0 APIs
 */
#define SVEN_PCFG_ENABLE_DEVH_API

/**
 * Enable raw data output APIs
 */
#define SVEN_PCFG_ENABLE_WRITE_API

/**
 * Enable API function call/return reporting APIs
 */
#define SVEN_PCFG_ENABLE_API_API

/**
 * Enable register IO API's
 */
#define SVEN_PCFG_ENABLE_REGISTER_API

/**
 * Enable Trigger APIs
 */
#define SVEN_PCFG_ENABLE_TRIGGER_API

/* @} */

/**
 * @defgroup PCFG_Event Optional event attributes
 * @ingroup  PCFG_Config
 *
 * These defines enable optional event components.
 * @{
 */

/**
 * Enable per handle sequence counting.
 */
#define SVEN_PCFG_ENABLE_SEQUENCE_COUNT

/**
 * Enable 128 Bit origin GUID support.
 */
#define SVEN_PCFG_ENABLE_ORIGIN_GUID

/**
 * Enable <file>:<line> ID pair location record sending API variants.
 */
#define SVEN_PCFG_ENABLE_LOCATION_RECORD

/**
 * Enable the instrumentation location address sending API variant.
 *
 * This API requires #SVEN_PCFG_ENABLE_LOCATION_RECORD to be set as well.
 * It uses its own define as it additionally requires the function
 * @ref sventx_return_addr() to be implemented for your platform.
 */
#define SVEN_PCFG_ENABLE_LOCATION_ADDRESS

/**
 * Enable event data CRC32 generation.
 */
#define SVEN_PCFG_ENABLE_CHECKSUM

/** @} */

#ifdef __cplusplus
}				/* extern C */
#endif
#endif
