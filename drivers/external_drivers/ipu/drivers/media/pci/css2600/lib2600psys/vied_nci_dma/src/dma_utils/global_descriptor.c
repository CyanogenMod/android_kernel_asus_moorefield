#include "global_descriptor.h"
#include "vied_nci_dma_local.h"
#include "vied_nci_dma_dev_access.h"

/******************************************************
 * global descriptor API functions
 ******************************************************/
void vied_nci_dma_global_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const struct vied_nci_dma_global_desc_t *const global_descriptor)
{
	uint32_t global_set_id;

	vied_nci_dma_reg_store(dev, GLOBAL_GROUP_ID, 0, REG_UNIT_BASE_ADDR,
			    global_descriptor->unit_descriptor_base_addr);
	vied_nci_dma_reg_store(dev, GLOBAL_GROUP_ID, 0, REG_SPAN_BASE_ADDR,
			    global_descriptor->span_descriptor_base_addr);
	vied_nci_dma_reg_store(dev, GLOBAL_GROUP_ID, 0, REG_TERMINAL_BASE_ADDR,
			    global_descriptor->terminal_descriptor_base_addr);
	vied_nci_dma_reg_store(dev, GLOBAL_GROUP_ID, 0, REG_CHANNEL_BASE_ADDR,
			    global_descriptor->channel_descriptor_base_addr);
	vied_nci_dma_reg_store(dev, GLOBAL_GROUP_ID, 0, REG_MAX_BLOCK_HEIGHT,
			    global_descriptor->max_block_height);

	for (global_set_id = 0; global_set_id < dev->global_sets; global_set_id++) {
		vied_nci_dma_reg_store(dev, GLOBAL_GROUP_ID, 0,
				REG_MAX_BLOCK_WIDTH_1DBURST(global_set_id),
				global_descriptor->max_1d_block_width[global_set_id]);
		vied_nci_dma_reg_store(dev, GLOBAL_GROUP_ID, 0,
				REG_MAX_BLOCK_WIDTH_2DBURST(global_set_id),
				global_descriptor->max_2d_block_width[global_set_id]);
	}
}
#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_global_desc_t vied_nci_dma_global_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev)
{
	uint32_t global_set_id;
	struct vied_nci_dma_global_desc_t global_descriptor = { 0 };

	global_descriptor.unit_descriptor_base_addr =
		vied_nci_dma_reg_load(dev, GLOBAL_GROUP_ID, 0, REG_UNIT_BASE_ADDR);
	global_descriptor.span_descriptor_base_addr =
		vied_nci_dma_reg_load(dev, GLOBAL_GROUP_ID, 0, REG_SPAN_BASE_ADDR);
	global_descriptor.terminal_descriptor_base_addr =
		vied_nci_dma_reg_load(dev, GLOBAL_GROUP_ID, 0, REG_TERMINAL_BASE_ADDR);
	global_descriptor.channel_descriptor_base_addr =
		vied_nci_dma_reg_load(dev, GLOBAL_GROUP_ID, 0, REG_CHANNEL_BASE_ADDR);
	global_descriptor.max_block_height =
		vied_nci_dma_reg_load(dev, GLOBAL_GROUP_ID, 0, REG_MAX_BLOCK_HEIGHT);

	for (global_set_id = 0; global_set_id < dev->global_sets; global_set_id++) {
		global_descriptor.max_1d_block_width[global_set_id] =
			vied_nci_dma_reg_load(dev, GLOBAL_GROUP_ID, 0,
			REG_MAX_BLOCK_WIDTH_1DBURST(global_set_id));
		global_descriptor.max_2d_block_width[global_set_id] =
			vied_nci_dma_reg_load(dev, GLOBAL_GROUP_ID, 0,
			REG_MAX_BLOCK_WIDTH_2DBURST(global_set_id));
	}

	return global_descriptor;
}
#endif
