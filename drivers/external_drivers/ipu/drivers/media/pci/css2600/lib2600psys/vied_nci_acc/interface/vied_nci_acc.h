/*
 * vied_nci_acc.h
 *
 *  Created on: Jan 15, 2014
 *     Authors: mmarkov1
 *              vilic
 */

/*
 * Description : Declarations of NCI functions used both for PSYS and ISYS clusters...
 */

#ifndef __VIED_NCI_ACC_H_INCLUDED__
#define __VIED_NCI_ACC_H_INCLUDED__

#include <hrt/api.h>
#include <type_support.h>
#include "vied_nci_acc_storage_class.h"

#ifdef PSYS
	#include "vied_nci_acc_psys_defs.h"
	#include "vied_nci_acc_psys_local_defs.h"
#else
	#include "vied_nci_acc_isys_defs.h"
	#include "vied_nci_acc_isys_local_defs.h"
#endif

/*
 * Load time interface
 */

VIED_NCI_ACC_STORAGE_CLASS_H
enum vied_nci_acc_id vied_nci_acc_open(
	enum vied_nci_acc_id device_id
);


VIED_NCI_ACC_STORAGE_CLASS_H
enum vied_nci_err vied_nci_acc_close(
	enum vied_nci_acc_id acc_handle
);


enum vied_nci_err vied_nci_acc_store_to_ff(
	enum vied_nci_acc_id acc_handle,
	enum vied_nci_ff_id fixed_function_id,
	void *ff_params,
	uint32_t ff_param_size
);


enum vied_nci_err vied_nci_acc_load_from_ff(
	enum vied_nci_acc_id acc_handle,
	enum vied_nci_ff_id fixed_function_id,
	void *buffer_address,
	uint32_t buffer_size
);


enum vied_nci_err vied_nci_acc_get_ff_params_space(enum vied_nci_acc_id acc_handle,
						enum vied_nci_ff_id fixed_function_id,
						uint32_t *base, uint32_t *end);


enum vied_nci_err vied_nci_acc_get_ff_buffer_space(enum vied_nci_acc_id acc_handle,
						enum vied_nci_ff_id fixed_function_id,
						uint32_t set_id, uint32_t *base, uint32_t *end);


enum vied_nci_err vied_nci_acc_get_ff_partition_params_space(enum vied_nci_acc_id acc_handle,
							enum vied_nci_ff_id fixed_function_id,
							uint32_t set_id, uint32_t *base, uint32_t *end);


enum vied_nci_err vied_nci_acc_routing_in(
	enum vied_nci_acc_id acc_handle
);


enum vied_nci_err vied_nci_acc_routing_previous_in(
	enum vied_nci_acc_id acc_handle
);


enum vied_nci_err vied_nci_acc_routing_next_out(
	enum vied_nci_acc_id acc_handle,
	uint32_t ack_address,
	uint32_t ack_sid,
	uint32_t ack_pid,
	uint32_t ack_msg
);


enum vied_nci_err vied_nci_acc_routing_out(
	enum vied_nci_acc_id acc_handle,
	uint32_t ack_address,
	uint32_t ack_sid,
	uint32_t ack_pid,
	uint32_t ack_msg
);


/*
 * Configuration time interface
 */

void vied_nci_acc_config_resolution(
	enum vied_nci_acc_id acc_handle,
	uint32_t fragment_width,
	uint32_t fragment_height,
	uint32_t scale_factor_mult,
	uint32_t scale_factor_nf
);

VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_queue_idle_state(
	enum vied_nci_acc_id acc_handle
);


/*
 * Run time interface
 */

void vied_nci_acc_set_part_params(
	enum vied_nci_acc_id acc_handle,
	enum vied_nci_ff_id fixed_function_id,
	uint32_t part_param_set_id,
	void *part_params,
	uint32_t part_param_size
);

void vied_nci_acc_get_part_params(
	enum vied_nci_acc_id acc_handle,
	enum vied_nci_ff_id fixed_function_id,
	uint32_t part_param_set_id,
	void *buffer_address,
	uint32_t buffer_size
);

VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_queue_process_fragment(
	enum vied_nci_acc_id acc_handle,
	uint32_t fragment_part_height,
	uint32_t part_param_set_id
);

void vied_nci_acc_flush_meta_data(
	enum vied_nci_acc_id acc_handle,
	enum vied_nci_ff_id fixed_function_id,
	uint32_t set_id,
	void *buffer_address,
	uint32_t buffer_size
);

#ifdef _INLINE_VIED_NCI_ACC
#include "vied_nci_acc_inline.h"
#endif

#endif /* __VIED_NCI_ACC_H_INCLUDED__ */

