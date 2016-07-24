#include "request_descriptor.h"
#include "vied_nci_dma_local.h"
#include "vied_bit.h"
#include "vied_nci_dma_dev_access.h"

/** Returns the transfer direction encoded in an 'execute' instruction.
*/
STORAGE_CLASS_INLINE enum vied_nci_dma_transfer_direction_t
get_transfer_direction(
		const uint32_t instruction)
{
	return vied_bit_slice(instruction, REQUEST_TRANSFER_DIRECTION_BIT, 1);
}

/** Returns the macro size encoded in an 'execute' instruction.
 */
STORAGE_CLASS_INLINE uint32_t get_macro_size(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t instruction)
{
	const struct vied_nci_dma_request_desc_bits_t *const reg =
	    &(dev->request_desc_bits);

	return vied_bit_slice(instruction, REQUEST_MACRO_SIZE_BIT, reg->macro_size_bits);
}

STORAGE_CLASS_INLINE uint32_t get_instruction_format(
		const uint32_t instruction)
{
	return vied_bit_slice(instruction, REQUEST_FORMAT_BIT, 1);
}

/******************************************************
 * request descriptor API functions
 ******************************************************/
uint32_t vied_nci_dma_request_desc_run(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t request_id,
		const uint32_t instruction)
{
	if (request_id >= dev->num_of_requestors) {
		return EINVAL;
	}

	/* write the instruction last, as this validates the request and temporarily
	 * blocks further access to the request register bank *
	 */
	vied_nci_dma_reg_store(dev, REQUEST_GROUP_ID, request_id,
			REG_INSTRUCTION, instruction);

	return 0;
}


uint32_t vied_nci_dma_request_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t request_id,
		const struct vied_nci_dma_request_desc_t *request_descriptor)
{
	if (request_id >= dev->num_of_requestors) {
		return EINVAL;
	}

	/* Required only for execution format */
	vied_nci_dma_reg_store(dev, REQUEST_GROUP_ID, request_id,
			REG_DESC_ID_SETUP1,
			request_descriptor->descriptor_id_setup_1);
	vied_nci_dma_reg_store(dev, REQUEST_GROUP_ID, request_id,
			REG_DESC_ID_SETUP2,
			request_descriptor->descriptor_id_setup_2);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_request_desc_t vied_nci_dma_request_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t request_id)
{
	struct vied_nci_dma_request_desc_t request_descriptor = {0};

	if (request_id >= dev->num_of_requestors) {
		return request_descriptor;
	}
	request_descriptor.instruction =
		vied_nci_dma_reg_load(dev, REQUEST_GROUP_ID, request_id,
		REG_INSTRUCTION);
	request_descriptor.descriptor_id_setup_1 =
		vied_nci_dma_reg_load(dev, REQUEST_GROUP_ID, request_id,
		REG_DESC_ID_SETUP1);
	request_descriptor.descriptor_id_setup_2 =
		vied_nci_dma_reg_load(dev, REQUEST_GROUP_ID, request_id,
		REG_DESC_ID_SETUP2);

	return request_descriptor;
}
#endif

uint32_t vied_nci_dma_read_request_valid_register(
		const struct vied_nci_dma_dev_t * const dev,
		const uint32_t request_id)
{
	if (request_id >= dev->num_of_requestors) {
		return 0;
	}

	return vied_nci_dma_reg_load(dev, REQUEST_GROUP_ID, request_id,
			REG_REQUEST_VALID);
}
