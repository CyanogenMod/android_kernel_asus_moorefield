/*! \file */

#ifndef _global_descriptor_h_
#define _global_descriptor_h_

#include "type_support.h"
#include "vied_nci_dma.h"

/** Uploads a global descriptor \p global_desc to DMA global registers.
 */
STORAGE_CLASS_EXTERN void vied_nci_dma_global_desc_reg_store(
	const struct vied_nci_dma_dev_t *const dev,
	const struct vied_nci_dma_global_desc_t *const global_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Downloads a global descriptor from DMA global registers.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_global_desc_t
vied_nci_dma_global_desc_reg_load(
	const struct vied_nci_dma_dev_t *const dev);
#endif

#endif
