/*! \file */

#ifndef _terminal_descriptor_h_
#define _terminal_descriptor_h_

#include "type_support.h"
#include "vied_nci_dma.h"

/** Writes the contents of the \p terminal_descriptor identified with \p terminal_id,
 *  to its corresponding location in the list of terminal descriptors starting at the
 *  memory location specified by the \p terminal_desc_base_addr member of
 *  DMA device \p dev.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_terminal_desc_mem_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t terminal_id,
		const struct vied_nci_dma_terminal_desc_t *terminal_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Reads the terminal descriptor identified with \p terminal_id, from its
 *  corresponding location in the list of terminal descriptors starting at
 *  the memory location specified by the \p terminal_desc_base_addr member
 *  of DMA device \p dev.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_terminal_desc_t
vied_nci_dma_terminal_desc_mem_load(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t terminal_id);
#endif

/** Uploads the contents of the \p terminal_descriptor identified with \p terminal_id,
 *  to its corresponding location in the terminal descriptors register bank.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_terminal_desc_reg_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t terminal_id,
		const struct vied_nci_dma_terminal_desc_t *terminal_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Downloads the terminal descriptor identified \p terminal_id from its corresponding
 *  location in the terminal descriptors register bank.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_terminal_desc_t
vied_nci_dma_terminal_desc_reg_load(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t terminal_id);
#endif

/** Sets the bank mode of bank \p terminal_bank_id to \p bank_mode.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_terminal_set_bank_mode(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t terminal_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode);

#endif
