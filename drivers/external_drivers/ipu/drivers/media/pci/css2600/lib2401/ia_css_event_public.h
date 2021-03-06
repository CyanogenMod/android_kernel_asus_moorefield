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

#ifndef __IA_CSS_EVENT_PUBLIC_H
#define __IA_CSS_EVENT_PUBLIC_H

/** @file
 * This file contains CSS-API events functionality
 */

#include <type_support.h>	/* uint8_t */
#include <ia_css_err.h>		/* ia_css_err */
#include <ia_css_types.h>	/* ia_css_pipe */

/** The event type, distinguishes the kind of events that
 * can are generated by the CSS system.
 *
 * !IMPORTANT! KEEP THE FOLLOWING 3 ENUMS IN SYNC:
 * 1) "enum ia_css_event_type event_id_2_event_mask" (file event.sp.c)
 * 2) "enum ia_css_event_type convert_event_sp_to_host_domain" (file sh_css.c)
 * 3) "enum sh_css_sp_event_type" (file sh_css_internal.h)
 */
enum ia_css_event_type {
	IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE	= 1 << 0, /**< Output frame ready. */
	IA_CSS_EVENT_TYPE_SECOND_OUTPUT_FRAME_DONE	= 1 << 1, /**< Second output frame ready. */
	IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE	= 1 << 2, /**< Viewfinder Output frame ready. */
	IA_CSS_EVENT_TYPE_SECOND_VF_OUTPUT_FRAME_DONE	= 1 << 3, /**< Second viewfinder Output frame ready. */
	IA_CSS_EVENT_TYPE_3A_STATISTICS_DONE	= 1 << 4, /**< Indication that 3A statistics are available. */
	IA_CSS_EVENT_TYPE_DIS_STATISTICS_DONE	= 1 << 5, /**< Indication that DIS statistics are available. */
	IA_CSS_EVENT_TYPE_PIPELINE_DONE		= 1 << 6, /**< Pipeline Done event, sent after last pipeline stage. */
	IA_CSS_EVENT_TYPE_FRAME_TAGGED		= 1 << 7, /**< Frame tagged. */
	IA_CSS_EVENT_TYPE_INPUT_FRAME_DONE	= 1 << 8, /**< Input frame ready. */
	IA_CSS_EVENT_TYPE_METADATA_DONE		= 1 << 9, /**< Metadata ready. */
	IA_CSS_EVENT_TYPE_LACE_STATISTICS_DONE	= 1 << 10, /**< Indication that LACE statistics are available. */
	IA_CSS_EVENT_TYPE_ACC_STAGE_COMPLETE	= 1 << 11, /**< Extension stage complete. */
	/* NOTE: Add new enums before IA_CSS_EVENT_TYPE_PORT_EOF MUST, see comment below */
	IA_CSS_EVENT_TYPE_PORT_EOF		= 1 << 12, /**< End Of Frame event, sent when in buffered sensor mode.
								This enum MUST be last */
};

#define IA_CSS_EVENT_TYPE_NONE 0

/** IA_CSS_EVENT_TYPE_ALL is a mask for alle events _EXCEPT_ IA_CSS_EVENT_TYPE_PORT_EOF
 * For this reason IA_CSS_EVENT_TYPE_PORT_EOF must be last in the enum above.
 * we create it by sbutracting 1 from IA_CSS_EVENT_TYPE_PORT_EOF
 * alternate implementation could be or-ing of all the events.
 * (IA_CSS_EVENT_TYPE_OUTPUT_FRAME_DONE | IA_CSS_EVENT_TYPE_VF_OUTPUT_FRAME_DONE | ... |
 *  IA_CSS_EVENT_TYPE_INPUT_FRAME_DONE)
 */
#define IA_CSS_EVENT_TYPE_ALL  (IA_CSS_EVENT_TYPE_PORT_EOF - 1)

/** The event struct, container for the event type and its related values.
 * Depending on the event type, either pipe or port will be filled.
 * Pipeline related events (like buffer/frame events) will return a valid and filled pipe handle.
 * For non pipeline related events (but i.e. stream specific, like EOF event), the port will be
 * filled.
 */
struct ia_css_event {
	struct ia_css_pipe    *pipe;   /**< Pipe handle on which event happened, NULL for non pipe related events. */
	enum ia_css_event_type type;   /**< Type of Event, always valid/filled. */
	uint8_t                port;   /**< Port number for EOF event (not valid for other events). */
	uint8_t                exp_id; /**< Exposure id for EOF/TAGGED_FRAME event (not valid for other events). */
	uint32_t               fw_handle; /**< Firmware Handle for ACC_STAGE_COMPLETE event (not valid for other events). */
};

/** @brief Dequeue a PSYS event from the CSS system.
 *
 * @param[out]	event   Pointer to the event struct which will be filled by
 *                      this function if an event is available.
 * @return		IA_CSS_ERR_QUEUE_IS_EMPTY if no events are
 *			available or
 *			IA_CSS_SUCCESS otherwise.
 *
 * This function dequeues an event from the PSYS event queue. The queue is
 * between the Host CPU and the CSS system. This function can be
 * called after an interrupt has been generated that signalled that a new event
 * was available and can be used in a polling-like situation where the NO_EVENT
 * return value is used to determine whether an event was available or not.
 */
enum ia_css_err
ia_css_dequeue_psys_event(struct ia_css_event *event);

/** @brief Dequeue an event from the CSS system.
 *
 * @param[out]	event   Pointer to the event struct which will be filled by
 *                      this function if an event is available.
 * @return		IA_CSS_ERR_QUEUE_IS_EMPTY if no events are
 *			available or
 *			IA_CSS_SUCCESS otherwise.
 *
 * deprecated{Use ia_css_dequeue_psys_event instead}.
 * Unless the isys event queue is explicitly enabled, this function will
 * dequeue both isys (EOF) and psys events (all others).
 */
enum ia_css_err
ia_css_dequeue_event(struct ia_css_event *event);

/** @brief Dequeue an ISYS event from the CSS system.
 *
 * @param[out]	event   Pointer to the event struct which will be filled by
 *                      this function if an event is available.
 * @return		IA_CSS_ERR_QUEUE_IS_EMPTY if no events are
 *			available or
 *			IA_CSS_SUCCESS otherwise.
 *
 * This function dequeues an event from the ISYS event queue. The queue is
 * between host and the CSS system.
 * Unlike the ia_css_dequeue_event() function, this function can be called
 * directly from an interrupt service routine (ISR) and it is safe to call
 * this function in parallel with other CSS API functions (but only one
 * call to this function should be in flight at any point in time).
 *
 * The reason for having the ISYS events separate is to prevent them from
 * incurring additional latency due to locks being held by other CSS API
 * functions.
 */
enum ia_css_err
ia_css_dequeue_isys_event(struct ia_css_event *event);

#endif /* __IA_CSS_EVENT_PUBLIC_H */
