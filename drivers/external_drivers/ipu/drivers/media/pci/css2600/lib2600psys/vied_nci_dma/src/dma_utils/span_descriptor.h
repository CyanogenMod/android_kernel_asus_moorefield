/*! \file */

#ifndef _span_descriptor_h_
#define _span_descriptor_h_

#include "type_support.h"
#include "vied_nci_dma.h"

/** Writes the contents of the \p span_descriptor identified with \p span_id,
 *  to its corresponding location in the list of span descriptors starting at the
 *  memory location specified by the \p span_desc_base_addr member of
 *  DMA device \p dev.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_span_desc_mem_store(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_id,
		const struct vied_nci_dma_span_desc_t *span_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Reads the span descriptor identified with \p span_id, from its
 *  corresponding location in the list of span descriptors starting at
 *  the memory location specified by the \p span_desc_base_addr member
 *  of DMA device \p dev.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_span_desc_t vied_nci_dma_span_desc_mem_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_id);
#endif

/** Uploads the contents of the \p span_descriptor identified with \p span_id,
 *  to its corresponding location in the span descriptors register bank.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_span_desc_reg_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t span_id,
		const struct vied_nci_dma_span_desc_t *span_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Downloads the span descriptor identified \p span_id from its corresponding
 *  location in the span descriptors register bank.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_span_desc_t vied_nci_dma_span_desc_reg_load(
		const struct vied_nci_dma_dev_t *const dev,
		const uint32_t span_id);
#endif

/** Sets the bank mode of bank \p span_bank_id to \p bank_mode.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_span_set_bank_mode(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t span_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode);

#endif
