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

#ifndef SVEN_H_INCLUDED
#define SVEN_H_INCLUDED

/* SVENTX library version information
 */
#define SVEN_VERSION_MAJOR 1   /**< Major version, incremented if API changes */
#define SVEN_VERSION_MINOR 6   /**< Minor version, incremented on compatible extensions */
#define SVEN_VERSION_PATCH 0   /**< Patch for existing major, minor, usually 0 */

/** Compute SVENTX version value
 *
 * Used to compare SVEN Major.Minor.patch versions numerically at runtime.
 *
 * @param ma major version number
 * @param mi minor version number
 * @param p patch version number
 *
 * Example:
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~{.c}
 *
 * #if  SVEN_VERSION_CODE >= SVEN_MAKE_VERSION_CODE(1,5,0)
 *     // do what only >= 1.5.x supports
 * #endif
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */
#define SVEN_MAKE_VERSION_CODE(ma, mi, p) (((ma) << 16) | ((mi)<<8) | (p))

/** Numeric SVEN version code */
#define SVEN_VERSION_CODE SVEN_MAKE_VERSION_CODE(\
	SVEN_VERSION_MAJOR,\
	SVEN_VERSION_MINOR,\
	SVEN_VERSION_PATCH)

/* Macros to trick numeric values like __LINE__ into a string
 */
#define _SVEN_STRINGIFY(x) #x
#define _SVEN_CPP_TOSTR(x) _SVEN_STRINGIFY(x)

#define _SVEN_VERSION_STRING(a, b, c)\
	_SVEN_CPP_TOSTR(a)"."_SVEN_CPP_TOSTR(b)"."_SVEN_CPP_TOSTR(c)

/** Textual version string */
#define SVEN_VERSION_STRING \
	_SVEN_VERSION_STRING(\
		SVEN_VERSION_MAJOR,\
		SVEN_VERSION_MINOR,\
		SVEN_VERSION_PATCH)

#ifndef SVEN_COMPILER_INCLUDED
#include "sventx/compiler.h"
#endif

#if defined(__cplusplus)
extern "C" {
#endif

/** SVEN Major event Types
 */
enum e_sven_eventtype_t {
	SVEN_event_type_invalid = 0,        /**< NO ZEROES ALLOWED         */
	SVEN_Event_type_short = 1,          /**< tag only event            */
	SVEN_event_type_debug_string = 2,   /**< text message output       */
	SVEN_event_type_catalog_msg = 3,    /**< catalog message output    */
	SVEN_event_type_register_io = 4,    /**< MMIO register access      */
	SVEN_event_type_port_io = 5,        /**< port space access         */
	SVEN_event_type_module_specific = 6,/**< custom module payload     */
	SVEN_event_type_API = 7,            /**< API call/return           */

	SVEN_event_type_MAX
};

/** SVEN_event_type_debug_string Sub-Types
 */
typedef enum e_sven_eventtype_debugstr {
	SVEN_DEBUGSTR_invalid = 0,        /**< no zeroes allowed            */
	SVEN_DEBUGSTR_Generic = 1,        /**< string generic debug         */
	SVEN_DEBUGSTR_FunctionEntered = 2,/**< string is function name      */
	SVEN_DEBUGSTR_FunctionExited = 3, /**< string is function name      */
	SVEN_DEBUGSTR_AutoTrace = 4,      /**< string is __FILE__:__LINE__  */
	SVEN_DEBUGSTR_InvalidParam = 5,   /**< Invalid parameter passed     */
	SVEN_DEBUGSTR_Checkpoint = 6,     /**< Execution Checkpoint string  */
	SVEN_DEBUGSTR_Assert = 7,         /**< Software Assert: failure     */
	SVEN_DEBUGSTR_Warning = 8,        /**< Warning: text description    */
	SVEN_DEBUGSTR_FatalError = 9,     /**< Error: text description      */
	SVEN_DEBUGSTR_LogCat = 10,        /**< Android LogCat style string  */

	SVEN_DEBUGSTR_MAX
} sven_eventtype_debugstr_t;

/** SVEN_event_type_debug_string Sub-Types
 */
typedef enum e_sven_eventtype_catid {
	SVEN_CATID_invalid = 0,         /**< no zeroes allowed      */
	SVEN_CATID_32 = 1,              /**< 32 bit catalog ID      */
	SVEN_CATID_64 = 2,              /**< 64 bit catalog ID      */
	SVEN_CATID_32_PARALGIN = 3,     /**< 32 bit ID with gap for param alignment */

	SVEN_CATID_MAX
} sven_eventtype_catid_t;

typedef enum e_sven_eventtype_api {
	SVEN_EV_API_INVALID,           /**< no zeroes allowed              */
	SVEN_EV_API_FunctionCalled,    /**< API call event with parameters */
	SVEN_EV_API_FunctionReturned,  /**< API return event               */

	SVEN_EV_API_MAX
} e_sven_eventtype_api_t;

/**
 *  SVEN_event_type_register_io SUBTypes
 */
typedef enum e_sven_eventtype_regio {
	SVEN_EV_RegIo_invalid, /* no zeroes allowed */

	SVEN_EV_RegIo32_Read,
	SVEN_EV_RegIo32_Write,
	SVEN_EV_RegIo32_OrBits,
	SVEN_EV_RegIo32_AndBits,
	SVEN_EV_RegIo32_SetMasked, /* Clear, then set register bits */

	SVEN_EV_RegIo16_Read,
	SVEN_EV_RegIo16_Write,
	SVEN_EV_RegIo16_OrBits,
	SVEN_EV_RegIo16_AndBits,
	SVEN_EV_RegIo16_SetMasked, /* Clear, then set register bits */

	SVEN_EV_RegIo8_Read,
	SVEN_EV_RegIo8_Write,
	SVEN_EV_RegIo8_OrBits,
	SVEN_EV_RegIo8_AndBits,
	SVEN_EV_RegIo8_SetMasked, /* Clear, then set register bits */

	SVEN_EV_RegIo64_Read,
	SVEN_EV_RegIo64_Write,
	SVEN_EV_RegIo64_OrBits,
	SVEN_EV_RegIo64_AndBits,
	SVEN_EV_RegIo64_SetMasked, /* Clear, then set register bits */

	SVEN_EV_RegIoMSR_Read,
	SVEN_EV_RegIoMSR_Write,
	SVEN_EV_RegIoMSR_OrBits,
	SVEN_EV_RegIoMSR_AndBits,
	SVEN_EV_RegIoMSR_SetMasked, /* Clear, then set register bits */

	SVEN_EV_RegIo_MAX
} sven_eventtype_regio_t;

struct s_sven_header;
struct s_sven_handle;
struct s_sven_scatter_prog;

/** 128 bit GUID style event origin ID */
typedef struct s_sven_guid {
	sven_u32_t l;
	sven_u16_t w1;
	sven_u16_t w2;
	sven_u8_t b[8];
} sven_guid_t;

/**
 * SVEN global state initialization hook definition
 *
 * This function gets called in the context of the sventx_init() SVEN API
 * function after the generic state members of the global SVEN state
 * structure sven_hdr have been setup. It's purpose is to initialize the
 * platform dependent portion of the SVEN state and other necessary
 * platform specific initialization steps.
 *
 * @param svenh Pointer to SVEN global state structure
 * @param p user defined value or pointer to data
 * @see sven_hdr sven_header_t
 */
typedef void (SVEN_CALLCONV * sven_inithook_t)(struct s_sven_header *svenh,
		const void *p);

/**
 * SVEN global state destroy hook definition
 *
 * This function gets called in the context of the sventx_destroy() SVEN API
 * function before the generic state members of the global SVEN state
 * structure sven_hdr have been destroyed. Its purpose is to free resources
 * used by the platform dependent portion of the SVEN state.
 *
 * @param svenh Pointer to SVEN global state structure
 */
typedef void (SVEN_CALLCONV * sven_destroyhook_t)(struct s_sven_header *svenh);

/**
 * SVEN handle state initialization hook definition
 *
 * This function gets called in the context of SVEN IO handle generation.
 *It's purpose is to initialize the
 * platform dependent portion of the SVEN handle and other necessary
 * platform specific initialization steps.
 *
 * @param svenh Pointer to new SVEN handle
 * @param p user defined value or pointer to data
 * @see sven_handle_t
 */
typedef void (*sven_inithandle_hook_t)(struct s_sven_handle *svenh,
		const void *p);

/**
 * SVEN handle state release hook definition
 *
 * This function gets called when a SVEN handle is about to be destroyed..
 * It's purpose is to free any resources allocated during the handle
 * generation.
 *
 * @param svenh Pointer to SVEN handle the is destroyed
 * @see sven_handle_t
 */
typedef void (*sven_releasehandle_hook_t)(struct s_sven_handle *svenh);

/**
 * SVEN low level event write routine definition
 *
 * This function is called at the end of an instrumentation API to output
 * the raw event data.
 *
 * @param svenh pointer to a SVEN handle structure used in the SVEN API call,
 * @param scatterprog pointer to a list of scatter write instructions that
 *                    encodes how to convert the SVEN descriptor pointer by
 *                    pdesc into raw binary data. This list doesn't include
 *                    the mandatory first 32 tag byte value pointed by pdesc.
 * @param pdesc pointer to a sven descriptor, which containing at least
 *              the 32bit event tag data
 */
typedef void (*sven_event_write_t)(struct s_sven_handle *svenh,
		struct s_sven_scatter_prog *scatterprog, const void *pdesc);

#ifdef __cplusplus
} /* extern C */
#endif
#ifndef SVEN_PLATFORM_INCLUDED
#include "sventx_platform.h"
#endif
#ifdef __cplusplus
extern "C" {
#endif

#if defined(SVEN_PCFG_ENABLE_DEVH_API)
/* force implicit enabling of certain API's for legacy support
 */
#if !defined(SVEN_PCFG_ENABLE_STRING_API)
#define SVEN_PCFG_ENABLE_STRING_API
#endif
#endif

#if !defined(SVEN_HOT_ENABLE_DEFAULT)
/** Default value of HOT ENABLE 32bt mask
 *  This value should be defined inside sven_platform.h
 */
#define SVEN_HOT_ENABLE_DEFAULT 0xFFFFFFFF
#endif

#if defined(SVEN_PCFG_ENABLE_INLINE)
#define SVEN_INLINE static inline
#else
#define SVEN_INLINE SVEN_EXPORT
#endif

/** SVEN global state structure.
 * This structure is a singleton, holding the global SVENTX library state
 */
typedef struct s_sven_header {
	sven_u32_t svh_version; /**< SVEN version ID            */

#if defined(SVEN_PCFG_ENABLE_HOT_GATE)
	sven_u32_t svh_hot;     /**< event category hot gate    */
#endif

#if defined(SVEN_PCFG_ENABLE_PLATFORM_HANDLE_DATA)
	sven_inithandle_hook_t svh_inith;       /**< handle init hook function*/
	sven_releasehandle_hook_t svh_releaseh; /**< handle release hook      */
#endif

	sven_event_write_t svh_writer;         /**< event output routine      */

#if defined(SVEN_PCFG_ENABLE_PLATFORM_STATE_DATA)
	sven_platform_state_t svh_platform;
	/**< platform specific state    */
#endif
} sven_header_t, *psven_header_t;

/**
 * SVEN event data tag definition
 *
 * Each SVEN event starts with a 32bit event tag. The tag defines the
 * event originator and decoding information for the data following
 * the tag
 */
typedef struct s_sven_eventag {
	sven_u32_t et_type:4;     /**< SVEN event type ID             */
	sven_u32_t et_severity:3; /**< severity level of message      */

	sven_u32_t et_sequence:1; /**< indicate 32bit sequence number */
	sven_u32_t et_location:1; /**< indicate location information  */
	sven_u32_t et_length:1;   /**< indicate variable length event */
	sven_u32_t et_chksum:1;   /**< indicate 32bit CRC             */
	sven_u32_t et_reserved:1;
	sven_u32_t et_unit:4;     /**< module instance ID value       */
#define SVEN_MODULE_IS_GUID    0xFF
	sven_u32_t et_module:8;   /**< originator module ID           */
				  /**< 0xFF adds 128bit module GUID   */
				  /**< @see enum SVEN_MajorEventType_t*/

	sven_u32_t et_subtype:8;  /**< type dependend sub category    */
} sven_event_tag_t;

/**
 * Message severity level enumeration
 */
typedef enum e_sven_severity {
	SVEN_SEVERITY_NONE = 0,   /**< undefined severity         */
	SVEN_SEVERITY_FATAL = 1,  /**< critical error level       */
	SVEN_SEVERITY_ERROR = 2,  /**< error message level        */
	SVEN_SEVERITY_WARNING = 3,/**< warning message level      */
	SVEN_SEVERITY_NORMAL = 4, /**< normal message level       */
	SVEN_SEVERITY_USER1 = 5,  /**< user defined level 5       */
	SVEN_SEVERITY_USER2 = 6,  /**< user defined level 6       */
	SVEN_SEVERITY_USER3 = 7   /**< user defined level 7       */
} sven_severity_t;

/**
 * Specify format of used event location information that is added to the
 * event data if tag.et_location == 1
 */
typedef struct s_even_location_type {
	sven_s8_t elt_size:1; /**< 0 = 32bit, 1 = 64bit       */
	sven_s8_t elt_addr:1; /**< 0 = file/line, 1 = address */
	sven_s8_t elt_reserved:6;
} sven_location_type_t;

/**
 * Location information inside an event (64 bit format)
 * Location is either the source position of the instrumentation call, or
 * the call instruction pointer value.
 */
typedef union u_sven_eventlocation32 {
	struct {
		sven_u32_t etls_fileID:16;
		/**< ID of instrumented file   */
		sven_u32_t etls_lineNo:16;
		/**< line number in file       */
	} etls_source_location;

	sven_u32_t etls_code_location:32;
	/**< instruction pointer value */
} sven_eventlocation32_t;

/**
 * Location information inside an event (32 bit format)
 * Location is either the source position of the instrumentation call, or
 * the call instruction pointer value.
 */
typedef union s_sven_eventlocation64 {
	struct {
		sven_u64_t etls_fileID:32;
		/**< ID of instrumented file   */
		sven_u64_t etls_lineNo:32;
		/**< line number in file       */
	} etls_source_location;
	sven_u64_t etls_code_location:64;
	/**< instruction pointer value */
} sven_eventlocation64_t;

/**
 * Location information record descriptor
 */
typedef struct s_sven_eventlocation {
	sven_location_type_t el_format;
	/**< information size and type */

	union {
		sven_eventlocation32_t loc32; /**< data for 32 bit variant   */
		sven_eventlocation64_t loc64; /**< data for 64 bit variant   */
	} el_u;
} sven_eventlocation_t, *psven_eventlocation_t;

/** SVEN internal handle state flags
 */
typedef struct s_sven_handle_flags {
	sven_u32_t shf_alloc:1; /**< set if heap allocated     */
} sven_handle_flags_t;

/** SVEN connection handle state structure
 * This structure connects the instrumentation API with the underlying SVEN
 * Infrastructure. It plays a similar role as a FILE * in traditional
 * C file IO.
 */
typedef struct s_sven_handle {
	psven_header_t svh_header;    /**< global state             */
	sven_handle_flags_t svh_flags;/**< handle state             */
	sven_event_tag_t svh_tag;     /**< tag flags                */

#if defined(SVEN_PCFG_ENABLE_ORIGIN_GUID)
	sven_guid_t svh_guid; /**< module GUID              */
#endif

#if defined(SVEN_PCFG_ENABLE_SEQUENCE_COUNT)
	sven_u32_t svh_sequence_count; /**< event sequence counter  */
#endif

#if defined(SVEN_PCFG_ENABLE_LOCATION_RECORD)
	sven_eventlocation_t svh_location; /**< location record     */
#endif

	sven_u32_t svh_param_count; /**< number of parameters     */
	sven_u32_t svh_param[6];    /**< catalog msg parameters   */

#if defined(SVEN_PCFG_ENABLE_PLATFORM_HANDLE_DATA)
	sven_platform_handle_t svh_platform; /**< platform specific state  */
#endif
} sven_handle_t, *psven_handle_t;

#ifdef __cplusplus
} /* extern C */
#endif
#ifndef SVEN_API_INCLUDED
#include "sventx/api.h"
#endif

extern sven_handle_t sventx_null_handle;

#endif
