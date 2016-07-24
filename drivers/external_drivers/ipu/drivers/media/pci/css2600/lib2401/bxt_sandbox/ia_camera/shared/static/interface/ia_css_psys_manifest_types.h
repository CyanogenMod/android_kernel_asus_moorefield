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

#ifndef __IA_CSS_PSYS_MANIFEST_TYPES_H_INCLUDED__
#define __IA_CSS_PSYS_MANIFEST_TYPES_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_manifest_types.h
 *
 * The types belonging to the terminal/program/program group manifest static module
 */

#include <vied_nci_psys_system_global.h>	/* vied_nci_resource_id_t, vied_nci_resource_size_t */

#include <type_support.h>

#define IA_CSS_TERMINAL_TYPE_BITS				32
typedef enum ia_css_terminal_type {
	IA_CSS_TERMINAL_TYPE_DATA_IN = 0,		/**< Data input */
	IA_CSS_TERMINAL_TYPE_DATA_OUT,			/**< Data output */
	IA_CSS_TERMINAL_TYPE_PARAM_STREAM,		/**< Type 6 parameter input */
	IA_CSS_TERMINAL_TYPE_PARAM_CACHED,		/**< Type 1-5 parameter input */
	IA_CSS_TERMINAL_TYPE_STATE_IN,			/**< State (private data) input */
	IA_CSS_TERMINAL_TYPE_STATE_OUT,			/**< State (private data) output */
	IA_CSS_N_TERMINAL_TYPES
} ia_css_terminal_type_t;

/*
 * Connection type defining the interface source/sink
 *
 * Note that the connection type does not define the
 * real-time configuration of the system, i.e. it
 * does not describe whether a source and sink
 * program group or sub-system operate synchronously
 * that is a program script property {online, offline}
 * (see FAS 5.16.3)
 */
#define IA_CSS_CONNECTION_BITMAP_BITS			8
typedef uint8_t									ia_css_connection_bitmap_t;

typedef enum ia_css_connection_type {
	IA_CSS_CONNECTION_MEMORY = 0,			/**< The terminal is in DDR */
	IA_CSS_CONNECTION_MEMORY_STREAM,		/**< The terminal is a (watermark) queued stream over DDR */
	IA_CSS_CONNECTION_STREAM,				/**< The terminal is a device port */
	IA_CSS_N_CONNECTION_TYPES
} ia_css_connection_type_t;

#define IA_CSS_PROGRAM_TYPE_BITS				32
typedef enum ia_css_program_type {
	IA_CSS_PROGRAM_TYPE_SINGULAR = 0,
	IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB,
	IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER,
	IA_CSS_PROGRAM_TYPE_PARALLEL_SUB,
	IA_CSS_PROGRAM_TYPE_PARALLEL_SUPER,
/*
 * Future extension; A bitmap coding starts making more sense
 *
	IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB_PARALLEL_SUB,
	IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB_PARALLEL_SUPER,
	IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER_PARALLEL_SUB,
	IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUPER_PARALLEL_SUPER,
 */
	IA_CSS_N_PROGRAM_TYPES
} ia_css_program_type_t;

#define IA_CSS_PROGRAM_GROUP_ID_BITS			32
typedef uint32_t								ia_css_program_group_ID_t;
#define IA_CSS_PROGRAM_ID_BITS					32
typedef uint32_t								ia_css_program_ID_t;
#define IA_CSS_TERMINAL_ID_BITS					8
typedef uint8_t									ia_css_terminal_ID_t;

typedef struct ia_css_program_group_manifest_s	ia_css_program_group_manifest_t;
typedef struct ia_css_program_manifest_s		ia_css_program_manifest_t;
typedef struct ia_css_terminal_manifest_s		ia_css_terminal_manifest_t;
typedef struct ia_css_data_terminal_manifest_s	ia_css_data_terminal_manifest_t;
typedef struct ia_css_param_terminal_manifest_s	ia_css_param_terminal_manifest_t;


#define IA_CSS_PARAM_MANIFEST_BITS				48
typedef struct ia_css_parameter_manifest_s		ia_css_parameter_manifest_t;

struct ia_css_parameter_manifest_s {
	vied_nci_resource_id_t				kernel_id;										/**< Indication of the kernel this parameter belongs to */
	vied_nci_resource_id_t				mem_type_id;									/**< Indication of the memory type to be used by this parameter */
	vied_nci_resource_size_t			mem_size;										/**< Memory allocation size needs of this parameter */
	vied_nci_resource_size_t			mem_offset;										/**< Offset of the parameter allocation in memory */
};

#endif /* __IA_CSS_PSYS_MANIFEST_TYPES_H_INCLUDED__  */
