#include "master_descriptor.h"
#include "vied_nci_dma_local.h"
#include "vied_nci_dma_dev_access.h"

/******************************************************
 * master descriptor API functions
 ******************************************************/
uint32_t vied_nci_dma_master_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const struct vied_nci_dma_master_desc_t *const master_descriptor,
		const uint32_t master_id)
{
	if (master_id >= dev->master_banks) {
		return EINVAL;
	}

	vied_nci_dma_reg_store(dev, MASTER_GROUP_ID, master_id, REG_SRMD_SUPPORT,
			    master_descriptor->srmd_support);
	vied_nci_dma_reg_store(dev, MASTER_GROUP_ID, master_id, REG_BURST_SUPPORT,
			    master_descriptor->burst_support);

	return 0;
}

#ifdef VIED_NCI_DMA_DEBUG
struct vied_nci_dma_master_desc_t vied_nci_dma_master_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t master_id)
{
	struct vied_nci_dma_master_desc_t master_descriptor = {0};

	if (master_id >= dev->master_banks) {
		return master_descriptor;
	}

	master_descriptor.srmd_support =
		vied_nci_dma_reg_load(dev, MASTER_GROUP_ID, master_id,
		REG_SRMD_SUPPORT);
	master_descriptor.burst_support =
		vied_nci_dma_reg_load(dev, MASTER_GROUP_ID, master_id,
		REG_BURST_SUPPORT);

	return master_descriptor;
}
#endif
