#include <assert_support.h>
#include "unit_descriptor.h"
#include "vied_nci_dma_local.h"
#include "vied_nci_dma_dev_access.h"

/******************************************************
 * unit descriptor API functions
 ******************************************************/
uint32_t vied_nci_dma_unit_desc_mem_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t unit_id,
		const struct vied_nci_dma_unit_desc_t *unit_descriptor)
{
	uint32_t base_addr;
	const struct vied_nci_dma_unit_desc_bits_t *const reg =
	    &(dev->unit_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;

	OP___assert(reg->desc_words <= MAX_DESC_WORDS);
	if ((unit_id < dev->num_of_unit_banks) || (unit_id > dev->num_of_units)) {
		return EINVAL;
	}

	vied_nci_dma_desc_pack(&pbuf, unit_descriptor->unit_width,
		reg->unit_width_bits);
	vied_nci_dma_desc_pack(&pbuf, unit_descriptor->unit_height,
		reg->unit_height_bits);

	base_addr = vied_nci_dma_desc_mem_addr(dev->unit_desc_base_addr,
			(unit_id - dev->num_of_unit_banks), reg->desc_words);
	vied_nci_dma_desc_mem_store(base_addr, buf, reg->desc_words);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_unit_desc_t vied_nci_dma_unit_desc_mem_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t unit_id)
{
	uint32_t base_addr;
	struct vied_nci_dma_unit_desc_t unit_descriptor = {0};
	const struct vied_nci_dma_unit_desc_bits_t *const reg =
	    &(dev->unit_desc_bits);
	uint32_t buf[MAX_DESC_WORDS];
	uint8_t *pbuf = (uint8_t *)buf;

	if ((unit_id < dev->num_of_unit_banks) || (unit_id > dev->num_of_units)) {
		return unit_descriptor;
	}

	base_addr = vied_nci_dma_desc_mem_addr(dev->unit_desc_base_addr,
			(unit_id - dev->num_of_unit_banks), reg->desc_words);
	vied_nci_dma_desc_mem_load(base_addr, buf, reg->desc_words);

	unit_descriptor.unit_width =
	    vied_nci_dma_desc_unpack(&pbuf, reg->unit_width_bits);
	unit_descriptor.unit_height =
	    vied_nci_dma_desc_unpack(&pbuf, reg->unit_height_bits);

	return unit_descriptor;
}
#endif

uint32_t vied_nci_dma_unit_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t unit_id,
		const struct vied_nci_dma_unit_desc_t *unit_descriptor)
{
	if (unit_id >= dev->num_of_unit_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, UNIT_GROUP_ID, unit_id, REG_UNIT_WIDTH,
			unit_descriptor->unit_width);
	vied_nci_dma_reg_store(dev, UNIT_GROUP_ID, unit_id, REG_UNIT_HEIGHT,
			unit_descriptor->unit_height);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_unit_desc_t vied_nci_dma_unit_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t unit_id)
{
	struct vied_nci_dma_unit_desc_t unit_descriptor = {0};

	if (unit_id >= dev->num_of_unit_banks) {
		return unit_descriptor;
	}

	unit_descriptor.unit_width =
		vied_nci_dma_reg_load(dev, UNIT_GROUP_ID, unit_id, REG_UNIT_WIDTH);
	unit_descriptor.unit_height =
		vied_nci_dma_reg_load(dev, UNIT_GROUP_ID, unit_id, REG_UNIT_HEIGHT);

	return unit_descriptor;
}
#endif

uint32_t vied_nci_dma_unit_set_bank_mode(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t unit_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode)
{
	if (unit_bank_id >= dev->num_of_unit_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, UNIT_GROUP_ID, unit_bank_id,
			REG_BANK_MODE, bank_mode);

	return 0;
}
