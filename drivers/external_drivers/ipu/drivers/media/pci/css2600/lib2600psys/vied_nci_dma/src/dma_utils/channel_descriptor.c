#include <errno.h>
#include <assert_support.h>
#include "channel_descriptor.h"
#include "vied_nci_dma_local.h"
#include "vied_nci_dma_dev_access.h"

/******************************************************
 * channel descriptor API functions
 ******************************************************/
uint32_t vied_nci_dma_channel_desc_mem_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t channel_id,
		const struct vied_nci_dma_channel_desc_t *channel_descriptor)
{
	uint32_t base_addr;
	const struct vied_nci_dma_channel_desc_bits_t *const reg =
	    &(dev->channel_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;

	OP___assert(reg->desc_words <= MAX_DESC_WORDS);
	if ((channel_id < dev->num_of_channel_banks)
		|| (channel_id > dev->num_of_channels)) {
		return EINVAL;
	}

	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->element_extend_mode,
		reg->element_extend_mode_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->element_init_data,
		reg->element_init_data_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->padding_mode,
		reg->padding_mode_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->sampling_setup,
		reg->sampling_setup_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->global_set_id,
		reg->global_set_id_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->ack_mode,
		reg->ack_mode_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->ack_addr,
		reg->ack_addr_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->ack_data,
		reg->ack_data_bits);
	vied_nci_dma_desc_pack(&pbuf, channel_descriptor->completed_count,
		reg->completed_count_bits);

	base_addr = vied_nci_dma_desc_mem_addr(dev->channel_desc_base_addr,
			(channel_id - dev->num_of_channel_banks), reg->desc_words);
	vied_nci_dma_desc_mem_store(base_addr, buf, reg->desc_words);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_channel_desc_t vied_nci_dma_channel_desc_mem_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t channel_id)
{
	uint32_t base_addr;
	struct vied_nci_dma_channel_desc_t channel_descriptor = {0};
	const struct vied_nci_dma_channel_desc_bits_t *const reg =
	    &(dev->channel_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;

	if (channel_id < dev->num_of_channel_banks
		|| channel_id > dev->num_of_channels) {
		return channel_descriptor;
	}

	base_addr = vied_nci_dma_desc_mem_addr(dev->channel_desc_base_addr,
			(channel_id - dev->num_of_channel_banks), reg->desc_words);
	vied_nci_dma_desc_mem_load(base_addr, buf, reg->desc_words);

	channel_descriptor.element_extend_mode =
		vied_nci_dma_desc_unpack(&pbuf, reg->element_extend_mode_bits);
	channel_descriptor.element_init_data =
		vied_nci_dma_desc_unpack(&pbuf, reg->element_init_data_bits);
	channel_descriptor.padding_mode =
		vied_nci_dma_desc_unpack(&pbuf, reg->padding_mode_bits);
	channel_descriptor.sampling_setup =
		vied_nci_dma_desc_unpack(&pbuf, reg->sampling_setup_bits);
	channel_descriptor.global_set_id =
		vied_nci_dma_desc_unpack(&pbuf, reg->global_set_id_bits);
	channel_descriptor.ack_mode =
		vied_nci_dma_desc_unpack(&pbuf, reg->ack_mode_bits);
	channel_descriptor.ack_addr =
		vied_nci_dma_desc_unpack(&pbuf, reg->ack_addr_bits);
	channel_descriptor.ack_data =
		vied_nci_dma_desc_unpack(&pbuf, reg->ack_data_bits);
	channel_descriptor.completed_count =
		vied_nci_dma_desc_unpack(&pbuf, reg->completed_count_bits);

	return channel_descriptor;
}
#endif

uint32_t vied_nci_dma_channel_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t channel_id,
		const struct vied_nci_dma_channel_desc_t *channel_descriptor)
{
	if (channel_id >= dev->num_of_channel_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_ELEMENT_EXTEND_MODE,
			channel_descriptor->element_extend_mode);
	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_ELEMENT_INIT_DATA,
			channel_descriptor->element_init_data);
	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_PADDING_MODE,
			channel_descriptor->padding_mode);
	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_SAMPLING_SETUP,
			channel_descriptor->sampling_setup);
	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_GLOBAL_SET_ID,
			channel_descriptor->global_set_id);
	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_ACK_MODE,
			channel_descriptor->ack_mode);
	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_ACK_ADDRESS,
			channel_descriptor->ack_addr);
	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_id,
			REG_ACK_DATA,
			channel_descriptor->ack_data);

	// purposely do not upload anything to the completed counter, as this register is read-only
	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_channel_desc_t vied_nci_dma_channel_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t channel_id)
{
	struct vied_nci_dma_channel_desc_t channel_descriptor = {0};

	if (channel_id >= dev->num_of_channel_banks) {
		return channel_descriptor;
	}

	channel_descriptor.element_extend_mode =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_ELEMENT_EXTEND_MODE);
	channel_descriptor.element_init_data =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_ELEMENT_INIT_DATA);
	channel_descriptor.padding_mode =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_PADDING_MODE);
	channel_descriptor.sampling_setup =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_SAMPLING_SETUP);
	channel_descriptor.global_set_id =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_GLOBAL_SET_ID);
	channel_descriptor.ack_mode =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_ACK_MODE);
	channel_descriptor.ack_addr =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_ACK_ADDRESS);
	channel_descriptor.ack_data =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_ACK_DATA);
	channel_descriptor.completed_count =
		vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_id,
		REG_COMPLETED_COUNTER);

	return channel_descriptor;
}
#endif

#ifdef VIED_NCI_DMA_DEBUG
uint32_t vied_nci_dma_read_completed_counter(
		const struct vied_nci_dma_dev_t * const dev,
		const uint32_t channel_bank_id)
{

	if (channel_bank_id >= dev->num_of_channel_banks) {
		return 0;
	}
	return vied_nci_dma_reg_load(dev, CHANNEL_GROUP_ID, channel_bank_id,
			REG_COMPLETED_COUNTER);
}
#endif

uint32_t vied_nci_dma_channel_set_bank_mode(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t channel_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode)
{
	if (channel_bank_id >= dev->num_of_channel_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, CHANNEL_GROUP_ID, channel_bank_id,
			REG_BANK_MODE, bank_mode);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
uint32_t vied_nci_dma_wait_for_acknowledge(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_id,
		const struct vied_nci_dma_channel_desc_t *channel_descriptor)
{
	int completed = ETIME;
	int word = 0;
	int n = 100;

	NOT_USED(dev);
	NOT_USED(channel_id);
	/* passive acknowledge not supported*/
	if (channel_descriptor->ack_mode) {
		do {
			word = vied_nci_dma_load_32(channel_descriptor->ack_addr);
		} while (word != channel_descriptor->ack_data && (n--) > 0);
	} else {
		return EINVAL;
	}

	if (n > 0) {
		completed = 0;
	} else {
		completed = ETIME;
	}
	return completed;
}
#endif
