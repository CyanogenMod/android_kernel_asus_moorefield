/*! \file */

#ifndef _channel_descriptor_h_
#define _channel_descriptor_h_

#include "type_support.h"
#include "vied_nci_dma.h"

/** Writes the contents of the \p channel_descriptor identified with \p channel_id,
 *  to its corresponding location in the list of channel descriptors starting at the
 *  memory location specified by the \p channel_desc_base_addr member of
 *  DMA device \p dev.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_channel_desc_mem_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_id,
		const struct vied_nci_dma_channel_desc_t *channel_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Reads the channel descriptor identified with \p channel_id, from its
 *  corresponding location in the list of channel descriptors starting at
 *  the memory location specified by the \p channel_desc_base_addr member
 *  of DMA device \p dev.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_channel_desc_t
vied_nci_dma_channel_desc_mem_load(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_id);
#endif

/** Uploads the contents of the \p channel_descriptor identified with \p channel_id,
 *  to its corresponding location in the channel descriptors register bank.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_channel_desc_reg_store(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_id,
		const struct vied_nci_dma_channel_desc_t *channel_descriptor);

#ifdef VIED_NCI_DMA_DEBUG
/** Downloads the channel descriptor identified \p channel_id from its corresponding
 *  location in the channel descriptors register bank.
 */
STORAGE_CLASS_EXTERN struct vied_nci_dma_channel_desc_t
vied_nci_dma_channel_desc_reg_load(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_id);
#endif

#ifdef VIED_NCI_DMA_DEBUG
/** Returns the value of the \b completed counter in the channel bank \p channel_bank_id.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_read_completed_counter(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_bank_id);
#endif

/** Sets the bank mode of bank \p channel_bank_id to \p bank_mode.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_channel_set_bank_mode(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_bank_id,
		const enum vied_nci_dma_bank_mode_t bank_mode);

#ifdef VIED_NCI_DMA_DEBUG
/** Waits for the passive or active acknowledge message to arrive for the
 *  \p channel_descriptor identified with \p channel_id.
 */
STORAGE_CLASS_EXTERN uint32_t vied_nci_dma_wait_for_acknowledge(
		const struct vied_nci_dma_dev_t *dev,
		const uint32_t channel_id,
		const struct vied_nci_dma_channel_desc_t *channel_descriptor);
#endif

#endif
