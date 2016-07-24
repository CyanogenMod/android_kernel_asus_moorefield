/*! \file */

#ifndef _master_descriptor_h_
#define _master_descriptor_h_

#include "type_support.h"
#include "vied_nci_dma.h"

/** Uploads the contents of the \p master_desc identified with \p master_id,
 *  to its corresponding location in the master descriptors register bank.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_master_desc_reg_store(
		const struct vied_nci_dma_dev_t *const dev,
		const struct vied_nci_dma_master_desc_t *const master_descriptor,
		const uint32_t master_id);

#ifdef VIED_NCI_DMA_DEBUG
/** Downloads the master descriptor identified \p master_id from its corresponding
 *  location in the master descriptors register bank.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_master_desc_t
vied_nci_dma_master_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		uint32_t master_id);
#endif

#endif
