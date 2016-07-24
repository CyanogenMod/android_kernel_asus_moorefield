#include <assert_support.h>
#include "terminal_descriptor.h"
#include "vied_nci_dma_local.h"
#include "vied_nci_dma_dev_access.h"

/******************************************************
 * terminal descriptor API functions
 ******************************************************/
uint32_t vied_nci_dma_terminal_desc_mem_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t terminal_id,
		const struct vied_nci_dma_terminal_desc_t *terminal_descriptor)
{

	uint32_t base_addr;
	const struct vied_nci_dma_terminal_desc_bits_t *const reg =
	    &(dev->terminal_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;

	OP___assert(reg->desc_words <= MAX_DESC_WORDS);
	if ((terminal_id < dev->num_of_terminal_banks)
		|| (terminal_id > dev->num_of_terminals)) {
		return EINVAL;
	}

	vied_nci_dma_desc_pack(&pbuf, terminal_descriptor->region_origin,
		reg->region_origin_bits);
	vied_nci_dma_desc_pack(&pbuf, terminal_descriptor->region_width,
		reg->region_width_bits);
	vied_nci_dma_desc_pack(&pbuf, terminal_descriptor->region_stride,
		reg->region_stride_bits);
	vied_nci_dma_desc_pack(&pbuf, terminal_descriptor->element_setup,
		reg->element_setup_bits);
	vied_nci_dma_desc_pack(&pbuf, terminal_descriptor->cio_info_setup,
		reg->cio_info_setup_bits);
	vied_nci_dma_desc_pack(&pbuf, terminal_descriptor->port_mode,
		reg->port_mode_bits);

	base_addr = vied_nci_dma_desc_mem_addr(dev->terminal_desc_base_addr,
			(terminal_id - dev->num_of_terminal_banks), reg->desc_words);
	vied_nci_dma_desc_mem_store(base_addr, buf, reg->desc_words);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_terminal_desc_t vied_nci_dma_terminal_desc_mem_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t terminal_id)
{
	uint32_t base_addr;
	struct vied_nci_dma_terminal_desc_t terminal_descriptor = {0};
	const struct vied_nci_dma_terminal_desc_bits_t *const reg =
	    &(dev->terminal_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;

	if ((terminal_id < dev->num_of_terminal_banks)
		|| (terminal_id > dev->num_of_terminals)) {
		return terminal_descriptor;
	}

	base_addr = vied_nci_dma_desc_mem_addr(dev->terminal_desc_base_addr,
			(terminal_id - dev->num_of_terminal_banks), reg->desc_words);
	vied_nci_dma_desc_mem_load(base_addr, buf, reg->desc_words);

	terminal_descriptor.region_origin =
	    vied_nci_dma_desc_unpack(&pbuf, reg->region_origin_bits);
	terminal_descriptor.region_width =
		vied_nci_dma_desc_unpack(&pbuf, reg->region_width_bits);
	terminal_descriptor.region_stride =
		vied_nci_dma_desc_unpack(&pbuf, reg->region_stride_bits);
	terminal_descriptor.element_setup =
		vied_nci_dma_desc_unpack(&pbuf, reg->element_setup_bits);
	terminal_descriptor.cio_info_setup =
	    vied_nci_dma_desc_unpack(&pbuf, reg->cio_info_setup_bits);
	terminal_descriptor.port_mode =
	    vied_nci_dma_desc_unpack(&pbuf, reg->port_mode_bits);

	return terminal_descriptor;
}
#endif

uint32_t vied_nci_dma_terminal_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t terminal_id,
		const struct vied_nci_dma_terminal_desc_t *terminal_descriptor)
{
	if (terminal_id >= dev->num_of_terminal_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, TERMINAL_GROUP_ID, terminal_id,
			REG_REGION_ORIGIN,
			terminal_descriptor->region_origin);
	vied_nci_dma_reg_store(dev, TERMINAL_GROUP_ID, terminal_id,
			REG_REGION_WIDTH,
			terminal_descriptor->region_width);
	if (dev->region_stride_reg_available) {
		vied_nci_dma_reg_store(dev, TERMINAL_GROUP_ID, terminal_id,
			REG_REGION_STRIDE,
			terminal_descriptor->region_stride);
	}
	vied_nci_dma_reg_store(dev, TERMINAL_GROUP_ID, terminal_id,
			REG_ELEMENT_SETUP,
			terminal_descriptor->element_setup);
	vied_nci_dma_reg_store(dev, TERMINAL_GROUP_ID, terminal_id,
			REG_CIO_INFO,
			terminal_descriptor->cio_info_setup);
	vied_nci_dma_reg_store(dev, TERMINAL_GROUP_ID, terminal_id,
			REG_PORT_MODE,
			terminal_descriptor->port_mode);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_terminal_desc_t vied_nci_dma_terminal_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t terminal_id)
{
	struct vied_nci_dma_terminal_desc_t terminal_descriptor = {0};

	if (terminal_id >= dev->num_of_terminal_banks) {
		return terminal_descriptor;
	}

	terminal_descriptor.region_origin =
		vied_nci_dma_reg_load(dev, TERMINAL_GROUP_ID, terminal_id,
		REG_REGION_ORIGIN);
	terminal_descriptor.region_width =
		vied_nci_dma_reg_load(dev, TERMINAL_GROUP_ID, terminal_id,
		REG_REGION_WIDTH);
	if (dev->region_stride_reg_available) {
		terminal_descriptor.region_stride =
			vied_nci_dma_reg_load(dev, TERMINAL_GROUP_ID, terminal_id,
				REG_REGION_STRIDE);
	}
	terminal_descriptor.element_setup =
		vied_nci_dma_reg_load(dev, TERMINAL_GROUP_ID, terminal_id,
		REG_ELEMENT_SETUP);
	terminal_descriptor.cio_info_setup =
		vied_nci_dma_reg_load(dev, TERMINAL_GROUP_ID, terminal_id,
		REG_CIO_INFO);
	terminal_descriptor.port_mode =
		vied_nci_dma_reg_load(dev, TERMINAL_GROUP_ID, terminal_id,
		REG_PORT_MODE);

	return terminal_descriptor;
}
#endif

uint32_t vied_nci_dma_terminal_set_bank_mode(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t terminal_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode)
{
	if (terminal_bank_id >= dev->num_of_terminal_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, TERMINAL_GROUP_ID, terminal_bank_id,
			REG_BANK_MODE, bank_mode);

	return 0;
}
