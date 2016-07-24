/*! \file */

#ifndef _unit_descriptor_h_
#define _unit_descriptor_h_

#include <type_support.h>
#include "vied_nci_dma.h"

/** Writes the contents of the \p unit_descriptor identified with \p unit_id,
 *  to its corresponding location in the list of uint descriptors starting at the
 *  memory location specified by the \p uint_desc_base_addr member of
 *  DMA device \p dev.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_unit_desc_mem_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t unit_id,
		const struct vied_nci_dma_unit_desc_t *unit_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Reads the uint descriptor identified with \p uint_id, from its
 *  corresponding location in the list of uint descriptors starting at
 *  the memory location specified by the \p uint_desc_base_addr member
 *  of DMA device \p dev.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_unit_desc_t
vied_nci_dma_unit_desc_mem_load(
		const struct vied_nci_dma_dev_t *dev,
		uint32_t unit_id);
#endif

/** Uploads the contents of the \p uint_descriptor identified with \p uint_id,
 *  to its corresponding location in the uint descriptors register bank.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_unit_desc_reg_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t unit_id,
		const struct vied_nci_dma_unit_desc_t *unit_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Downloads the uint descriptor identified \p uint_id from its corresponding
 *  location in the uint descriptors register bank.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_unit_desc_t vied_nci_dma_unit_desc_reg_load(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t unit_id);
#endif

/** Sets the bank mode of bank \p uint_bank_id to \p bank_mode.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_unit_set_bank_mode(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t unit_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode);
#endif
