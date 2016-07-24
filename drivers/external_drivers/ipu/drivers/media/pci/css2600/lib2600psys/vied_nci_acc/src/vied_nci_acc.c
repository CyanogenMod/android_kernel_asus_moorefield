/*
 * vied_nci_acc.c
 *
 *  Created on: Jan 15, 2014
 *     Authors: mmarkov1
 *              vilic
 */

#include <hrt/api.h>
#include <ga_acb_api.h>
#include <type_support.h>
#include "misc_support.h"

#ifdef PSYS
  #include "vied_nci_acc_psys_defs.h"
  #include "vied_nci_acc_psys_local_defs.h"
#else
  #include "vied_nci_acc_isys_defs.h"
  #include "vied_nci_acc_isys_local_defs.h"
#endif

#include "vied_nci_acc_storage_class.h"
#include "vied_nci_acc_reg_access.h"

#ifndef _INLINE_VIED_NCI_ACC
#include "vied_nci_acc_inline.h"
#else
#ifdef _VIED_NCI_ACC_ON_CELL
#include "vied_nci_acc_cell_reg_access.h"
#endif
#endif


/* Implementation of the PSA API depends on NCIs
 * For the time being use stub functions
 * TODO: Replace stub functions with code from the HW integration tests
 * which implement the NCIs with the hrt calls
 */


/*
 * Load time interface
 */

enum vied_nci_err vied_nci_acc_load_from_ff(enum vied_nci_acc_id acc_handle,
					enum vied_nci_ff_id fixed_function_id,
					void *buffer_address,
					uint32_t buffer_size)
{
	uint32_t offset;
	uint32_t *param;

	NOT_USED(acc_handle);

	param = (uint32_t *)buffer_address;
	for (offset = 0; offset < buffer_size; offset += VIED_NCI_REGISTER_SIZE_BYTES) {
		*param = vied_nci_acc_device_get_ff_register(fixed_function_id, offset);
		param++;
	}

	return VIED_NCI_SUCCESS;
}


enum vied_nci_err vied_nci_acc_store_to_ff(enum vied_nci_acc_id acc_handle,
					enum vied_nci_ff_id fixed_function_id,
					void *ff_params,
					uint32_t ff_param_size)
{
	uint32_t offset;
	uint32_t *param;

	NOT_USED(acc_handle);

	param = (uint32_t *)ff_params;
	for (offset = 0; offset < ff_param_size; offset += VIED_NCI_REGISTER_SIZE_BYTES) {
		vied_nci_acc_device_set_ff_register(fixed_function_id, offset, (*param));
		param++;
	}

	return VIED_NCI_SUCCESS;
}


enum vied_nci_err vied_nci_acc_get_ff_params_space(enum vied_nci_acc_id acc_handle,
						enum vied_nci_ff_id fixed_function_id,
						uint32_t *base, uint32_t *end)
{
	NOT_USED(acc_handle);

	*base = vied_nci_acc_device_get_ff_params_space_base(fixed_function_id);
	*end  = vied_nci_acc_device_get_ff_params_space_end(fixed_function_id);

	return VIED_NCI_SUCCESS;
}


enum vied_nci_err vied_nci_acc_get_ff_buffer_space(enum vied_nci_acc_id acc_handle,
						enum vied_nci_ff_id fixed_function_id,
						uint32_t set_id, uint32_t *base, uint32_t *end)
{
	NOT_USED(acc_handle);

	*base = vied_nci_acc_device_get_ff_buffer_space_base(fixed_function_id, set_id);
	*end  = vied_nci_acc_device_get_ff_buffer_space_end(fixed_function_id, set_id);

	return VIED_NCI_SUCCESS;
}


enum vied_nci_err vied_nci_acc_get_ff_partition_params_space(enum vied_nci_acc_id acc_handle,
							enum vied_nci_ff_id fixed_function_id,
							uint32_t set_id, uint32_t *base, uint32_t *end)
{
	NOT_USED(acc_handle);

	*base = vied_nci_acc_device_get_ff_partition_params_space_base(fixed_function_id, set_id);
	*end  = vied_nci_acc_device_get_ff_partition_params_space_end(fixed_function_id, set_id);

	return VIED_NCI_SUCCESS;
}


enum vied_nci_err vied_nci_acc_routing_in(enum vied_nci_acc_id acc_handle)
{
	uint32_t reg_value;

	reg_value = vied_nci_acc_device_get_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR);
	vied_nci_acc_device_set_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR, (reg_value & 0xfffffffe));

	return VIED_NCI_SUCCESS;
}


enum vied_nci_err vied_nci_acc_routing_previous_in(enum vied_nci_acc_id acc_handle)
{
	uint32_t reg_value;

	reg_value = vied_nci_acc_device_get_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR);
	vied_nci_acc_device_set_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR, reg_value | 1);

	return VIED_NCI_SUCCESS;
}


enum vied_nci_err vied_nci_acc_routing_next_out(enum vied_nci_acc_id acc_handle,
						uint32_t ack_address,
						uint32_t ack_sid,
						uint32_t ack_pid,
						uint32_t ack_msg)
{
	uint32_t reg_value;

	reg_value = vied_nci_acc_device_get_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR);
	vied_nci_acc_device_set_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR, reg_value | 2);

	vied_nci_acc_device_set_ack_register(acc_handle, VIED_NCI_ACC_ACK_ADDR_ADDR, (ack_address));
	vied_nci_acc_device_set_ack_register(acc_handle, VIED_NCI_ACC_ACK_CMD_ADDR,
					((ack_sid << 26) | (ack_pid << 20) | ack_msg));

	return VIED_NCI_SUCCESS;
}

enum vied_nci_err vied_nci_acc_routing_out(enum vied_nci_acc_id acc_handle,
					uint32_t ack_address,
					uint32_t ack_sid,
					uint32_t ack_pid,
					uint32_t ack_msg)
{
	uint32_t reg_value;

	reg_value = vied_nci_acc_device_get_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR);
	vied_nci_acc_device_set_acb_register(acc_handle, GA_ACB_BASE_CTRL_ADDR, reg_value & 0xfffffffd);

	vied_nci_acc_device_set_ack_register(acc_handle, VIED_NCI_ACC_ACK_ADDR_ADDR, (ack_address));
	vied_nci_acc_device_set_ack_register(acc_handle, VIED_NCI_ACC_ACK_CMD_ADDR,
					((ack_sid << 26) | (ack_pid << 20) | ack_msg));

	if (acc_handle >= VIED_NCI_ICA_ID)
		vied_nci_acc_device_set_mux_register(acc_handle);

	return VIED_NCI_SUCCESS;
}


/*
 * Configuration time interface
 */


void vied_nci_acc_config_resolution(enum vied_nci_acc_id acc_handle,
				uint32_t fragment_width,
				uint32_t fragment_height,
				uint32_t scale_factor_mult,
				uint32_t scale_factor_nf)
{
	vied_nci_acc_device_set_acb_register(acc_handle, GA_ACB_INPUT_FRAME_SIZE_ADDR,
					((fragment_height << 16) | fragment_width));
	vied_nci_acc_device_set_acb_register(acc_handle, GA_ACB_SCALE_ADDR,
					((scale_factor_nf << 4) | scale_factor_mult));

	return;
}


/*
 * Run time interface
 */

void vied_nci_acc_set_part_params(enum vied_nci_acc_id acc_handle,
				enum vied_nci_ff_id fixed_function_id,
				uint32_t part_param_set_id,
				void *part_params,
				uint32_t part_param_size)
{
	uint32_t offset;
	uint32_t *param;

	NOT_USED(acc_handle);

	param = (uint32_t *)part_params;
	for (offset = 0; offset < part_param_size; offset += VIED_NCI_REGISTER_SIZE_BYTES) {
		vied_nci_acc_device_set_ff_part_register(fixed_function_id, part_param_set_id, offset, (*param));
		param++;
	}

	return;
}


void vied_nci_acc_get_part_params(enum vied_nci_acc_id acc_handle,
				enum vied_nci_ff_id fixed_function_id,
				uint32_t part_param_set_id,
				void *buffer_address,
				uint32_t buffer_size)
{
	uint32_t offset;
	uint32_t *reg;

	NOT_USED(acc_handle);

	reg = (uint32_t *)buffer_address;
	for (offset = 0; offset < buffer_size; offset += VIED_NCI_REGISTER_SIZE_BYTES) {
		*reg = vied_nci_acc_device_get_ff_part_register(fixed_function_id, part_param_set_id, offset);
		reg++;
	}

	return;
}

