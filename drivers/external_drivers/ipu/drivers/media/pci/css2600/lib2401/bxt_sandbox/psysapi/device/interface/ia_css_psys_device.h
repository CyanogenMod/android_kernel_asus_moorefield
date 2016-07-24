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

#ifndef __IA_CSS_PSYS_DEVICE_H_INCLUDED__
#define __IA_CSS_PSYS_DEVICE_H_INCLUDED__

/*! \file */

/** @file ia_css_psys_device.h
 *
 * Define the interface to open the psys specific communication layer
 * instance
 */

#include <vied_nci_psys_system_global.h>	/* vied_vaddress_t */

#include <type_support.h>
#include <print_support.h>

#include <ia_css_syscom_config.h>
#include <ia_css_syscom.h>

/*
 * The ID's of the Psys specific queues.
 */
typedef enum ia_css_psys_cmd_queues {
	IA_CSS_PSYS_CMD_QUEUE_EXECUTE_ID = 0,		/**< The in-order queue for scheduled proces groups */
	IA_CSS_PSYS_CMD_QUEUE_COMMAND_ID,			/**< The in-order queue for comamnds changing psys or process group state */
	IA_CSS_N_PSYS_CMD_QUEUE_ID
} ia_css_psys_cmd_queue_ID_t;

typedef enum ia_css_psys_event_queues {
	IA_CSS_PSYS_EVENT_QUEUE_MAIN_ID,			/**< The in-order queue for event returns */
	IA_CSS_N_PSYS_EVENT_QUEUE_ID
} ia_css_psys_event_queue_ID_t;


#define IA_CSS_PSYS_CMD_BITS					64
struct ia_css_psys_cmd_s {
	uint16_t			command;				/**< The command issued to the process group */
	uint16_t			msg;					/**< Message field of the command */
	uint32_t			process_group;			/**< The process group reference */
};

#define IA_CSS_PSYS_EVENT_BITS					128
struct ia_css_psys_event_s {
	uint16_t			status;					/**< The (return) status of the command issued to the process group this event refers to */
	uint16_t			command;				/**< The command issued to the process group this event refers to */
	uint32_t			process_group;			/**< The process group reference */
	uint64_t			token;					/**< This token (size) must match the token registered in a process group */
};

struct ia_css_psys_buffer_s {
	void				*host_buffer;			/**< The in-order queue for scheduled proces groups */
	vied_vaddress_t		*isp_buffer;
};

/*! Print the syscom creation descriptor to file/stream

 @param	config[in]				Psys syscom descriptor
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_psys_config_print(
	const struct ia_css_syscom_config		*config,
	void									*fid);

/*! Print the Psys syscom object to file/stream

 @param	context[in]				Psys syscom object
 @param	fid[out]				file/stream handle

 @return < 0 on error
 */
extern int ia_css_psys_print(
	const struct ia_css_syscom_context		*context,
	void									*fid);

/*! Create the syscom creation descriptor

 @return NULL on error
 */
extern struct ia_css_syscom_config *ia_css_psys_specify(void);

/*! Compute the size of storage required for allocating the Psys syscom object

 @param	config[in]				Psys syscom descriptor

 @return 0 on error
 */
extern size_t ia_css_sizeof_psys(
	struct ia_css_syscom_config				*config);

/*! Open (and map the storage for) the Psys syscom object

 @param	buffer[in]				storage buffers for the syscom object
								in the kernel virtual memory space and
								its Psys mapped version
 @param	config[in]				Psys syscom descriptor

 Precondition(1): The buffer must be large enough to hold the syscom object.
 Its size must be computed with the function "ia_css_sizeof_psys()". The
 buffer must be created in the kernel memory space.

 Precondition(2):  If buffer == NULL, the storage allocations and mapping
 is performed in this function. Config must hold the handle to the Psys
 virtual memory space

 Postcondition: The context is initialised in the provided/created buffer.
 The syscom context pointer is the kernel space handle to the syscom object

 @return NULL on error
 */
extern struct ia_css_syscom_context *ia_css_psys_open(
	const struct ia_css_psys_buffer_s		*buffer,
	struct ia_css_syscom_config				*config);

/*! close (and unmap the storage of) the Psys syscom object

 @param	context[in]				Psys syscom object

 @return NULL
 */
extern struct ia_css_syscom_context *ia_css_psys_close(
	struct ia_css_syscom_context			*context);

/*!Indicate if the designated cmd queue in the Psys syscom object is full

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom cmd queue ID

 @return false if the cmd queue is not full or on error
 */
extern bool ia_css_is_psys_cmd_queue_full(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id);

/*!Indicate if the designated cmd queue in the Psys syscom object is notfull

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom cmd queue ID

 @return false if the cmd queue is full on error
 */
extern bool ia_css_is_psys_cmd_queue_not_full(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id);

/*!Indicate if the designated cmd queue in the Psys syscom object holds N space

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom cmd queue ID
 @param	N[in]					Number of messages

 @return false if the cmd queue space is unavailable or on error
 */
extern bool ia_css_has_psys_cmd_queue_N_space(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id,
	const unsigned int						N);

/*!Return the free space count in the designated cmd queue in the Psys syscom object

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom cmd queue ID

 @return the space, < 0 on error
 */
extern int ia_css_psys_cmd_queue_get_available_space(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id);

/*!Indicate if there are any messages pending in the Psys syscom object event queues

 @param	context[in]				Psys syscom object

 @return false if there are no messages or on error
 */
extern bool ia_css_any_psys_event_queue_not_empty(
	const struct ia_css_syscom_context		*context);

/*!Indicate if the designated event queue in the Psys syscom object is empty

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom event queue ID

 @return false if the event queue is not empty or on error
 */
extern bool ia_css_is_psys_event_queue_empty(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id);

/*!Indicate if the designated event queue in the Psys syscom object is not empty

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom event queue ID

 @return false if the receive queue is empty or on error
 */
extern bool ia_css_is_psys_event_queue_not_empty(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id);

/*!Indicate if the designated event queue in the Psys syscom object holds N items

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom event queue ID
 @param	N[in]					Number of messages

 @return false if the event queue has insufficient messages available or on error
 */
extern bool ia_css_has_psys_event_queue_N_msgs(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id,
	const unsigned int						N);

/*!Return the message count in the designated event queue in the Psys syscom object

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom event queue ID

 @return the messages, < 0 on error
 */
extern int ia_css_psys_event_queue_get_available_msgs(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id);

/*! Send (pass by value) a command on a queue in the Psys syscom object

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom cmd queue ID
 @param	cmd_msg_buffer[in]		pointer to the command message buffer

 Precondition: The command message buffer must be large enough to hold the command

 Postcondition: Either 0 or 1 commands have been sent

 Note: The message size is fixed and determined on creation

 @return the number of sent commands (1), <= 0 on error
 */
extern int ia_css_psys_cmd_queue_send(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_cmd_queue_ID_t		id,
	const void								*cmd_msg_buffer);

/*! Send (pass by value) N commands on a queue in the Psys syscom object

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom cmd queue ID
 @param	cmd_msg_buffer[in]		pointer to the command message buffer
 @param	N[in]					Number of commands

 Precondition: The command message buffer must be large enough to hold the commands

 Postcondition: Either 0 or up to and including N commands have been sent

 Note: The message size is fixed and determined on creation

 @return the number of sent commands, <= 0 on error
 */
extern int ia_css_psys_cmd_queue_send_N(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_cmd_queue_ID_t		id,
	const void								*cmd_msg_buffer,
	const unsigned int						N);

/*! Receive (pass by value) an event from an event queue in the Psys syscom object

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom event queue ID
 @param	event_msg_buffer[out]	pointer to the event message buffer

 Precondition: The event message buffer must be large enough to hold the event

 Postcondition: Either 0 or 1 events have been received

 Note: The event size is fixed and determined on creation

 @return the number of received events (1), <= 0 on error
 */
extern int ia_css_psys_event_queue_receive(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_event_queue_ID_t		id,
	void									*event_msg_buffer);

/*! Receive (pass by value) N events from an event queue in the Psys syscom object

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom event queue ID
 @param	event_msg_buffer[out]	pointer to the event message buffer
 @param	N[in]					Number of events

 Precondition: The event buffer must be large enough to hold the events

 Postcondition: Either 0 or up to and including N events have been received

 Note: The message size is fixed and determined on creation

 @return the number of received event messages, <= 0 on error
 */
extern int ia_css_psys_event_queue_receive_N(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_event_queue_ID_t		id,
	void									*event_msg_buffer,
	const unsigned int						N);


/*
 * Access functions to query the object stats
 */


/*!Return the size of the Psys syscom object

 @param	context[in]				Psys syscom object

 @return 0 on error
 */
extern size_t ia_css_psys_get_size(
	const struct ia_css_syscom_context		*context);

/*!Return the number of cmd queues in the Psys syscom object

 @param	context[in]				Psys syscom object

 @return 0 on error
 */
extern unsigned int ia_css_psys_get_cmd_queue_count(
	const struct ia_css_syscom_context		*context);

/*!Return the number of event queues in the Psys syscom object

 @param	context[in]				Psys syscom object

 @return 0 on error
 */
extern unsigned int ia_css_psys_get_event_queue_count(
	const struct ia_css_syscom_context		*context);

/*!Return the size of the indicated Psys command queue

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom cmd queue ID

 Note: The queue size is expressed in the number of fields

 @return 0 on error
 */
extern size_t ia_css_psys_get_cmd_queue_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id);

/*!Return the size of the indicated Psys event queue

 @param	context[in]				Psys syscom object
 @param	id[in]					Psys syscom event queue ID

 Note: The queue size is expressed in the number of fields

 @return 0 on error
 */
extern size_t ia_css_psys_get_event_queue_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id);

/*!Return the command message size of the indicated Psys command queue

 @param	context[in]				Psys syscom object

 Note: The message size is expressed in uint8_t

 @return 0 on error
 */
extern size_t ia_css_psys_get_cmd_msg_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id);

/*!Return the event message size of the indicated Psys event queue

 @param	context[in]				Psys syscom object

 Note: The message size is expressed in uint8_t

 @return 0 on error
 */
extern size_t ia_css_psys_get_event_msg_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id);

#endif /* __IA_CSS_PSYS_DEVICE_H_INCLUDED__  */
