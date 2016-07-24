/*! \file */

#ifndef _request_descriptor_h_
#define _request_descriptor_h_

#include "type_support.h"
#include "vied_nci_dma.h"
#include "vied_nci_dma_local.h"
#include "vied_bit.h"

/** Creates an instruction field in 'execute' format out of the various field components.
 */
STORAGE_CLASS_INLINE uint32_t vied_nci_dma_create_execute_instruction(
		const struct vied_nci_dma_dev_t * const dev,
		const enum vied_nci_dma_command_t command,
		const uint32_t macro_size)
{
	const struct vied_nci_dma_request_desc_bits_t *const reg =
	    &(dev->request_desc_bits);

	uint32_t instruction = vied_bit_slice(macro_size - 1,
			0, reg->macro_size_bits);
	instruction = vied_bit_shift_OR(instruction,
			EXECUTION_INVAL_MODIFIER_BITS, 0);
	instruction = vied_bit_shift_OR(instruction,
			EXECUTION_COMMAND_BITS, command);
	instruction = vied_bit_shift_OR(instruction,
			EXECUTION_FORMAT_BITS, REQUEST_EXECUTION_FORMAT);

	return instruction;
}

/** Creates an instruction field in 'execute' format out of the various field components.
 */
STORAGE_CLASS_INLINE uint32_t vied_nci_dma_create_invalidate_instruction(
		const struct vied_nci_dma_dev_t * const dev,
		const enum vied_nci_dma_descriptor_kind_t descriptor_kind,
		const uint32_t lower_id,
		const uint32_t upper_id)
{
	const struct vied_nci_dma_request_desc_bits_t *const reg =
		&(dev->request_desc_bits);

	uint32_t instruction = vied_bit_slice(upper_id,
			0, reg->descriptor_id_bits);
	instruction = vied_bit_shift_OR(instruction,
			reg->descriptor_id_bits, lower_id);
	instruction = vied_bit_shift_OR(instruction,
			DESCRIPTOR_KIND_BITS, descriptor_kind);
	instruction = vied_bit_shift_OR(instruction,
			EXECUTION_FORMAT_BITS, REQUEST_INVALIDATION_FORMAT);

	return instruction;
}

/** Creates a descriptor id setup 1 field out of the various field components.
 */
STORAGE_CLASS_INLINE uint32_t vied_nci_dma_descriptor_id_setup_1(
		const struct vied_nci_dma_dev_t * const dev,
		const uint32_t unit_id,
		const uint32_t terminal_a_id,
		const uint32_t terminal_b_id,
		const uint32_t channel_id)
{
	const struct vied_nci_dma_request_desc_bits_t *const reg =
	    &(dev->request_desc_bits);

	uint32_t field = vied_bit_slice(channel_id, 0, reg->channel_bits);
	field = vied_bit_shift_OR(field, reg->terminal_bits, terminal_b_id);
	field = vied_bit_shift_OR(field, reg->terminal_bits, terminal_a_id);
	field = vied_bit_shift_OR(field, reg->unit_bits, unit_id);

	return field;
}

/** Creates a descriptor id setup 2 field out of the various field components.
 */
STORAGE_CLASS_INLINE uint32_t vied_nci_dma_descriptor_id_setup_2(
		const struct vied_nci_dma_dev_t * const dev,
		const uint32_t span_a_id,
		const uint32_t span_b_id)
{
	const struct vied_nci_dma_request_desc_bits_t *const reg =
	    &(dev->request_desc_bits);

	uint32_t field = vied_bit_slice(span_b_id, 0, reg->span_bits);
	field = vied_bit_shift_OR(field, reg->span_bits, span_a_id);

	return field;
}

/** Write \p instruction to the equest_descriptor identified with \p request_id,
 *  in the request descriptors register bank.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_request_desc_run(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t request_id,
		const uint32_t instruction);

/** Uploads the contents of the \p request_descriptor identified with \p request_id,
 *  to its corresponding location in the request descriptors register bank.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_request_desc_reg_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t request_id,
		const struct vied_nci_dma_request_desc_t *request_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Downloads the request descriptor identified \p request_id from its corresponding
 *  location in the request descriptors register bank.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_request_desc_t
vied_nci_dma_request_desc_reg_load(
		const struct vied_nci_dma_dev_t *dev,
		uint32_t request_id);
#endif

STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_read_request_valid_register(
		const struct vied_nci_dma_dev_t *dev,
		uint32_t request_id);

#endif
