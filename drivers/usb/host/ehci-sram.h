/*
 * EHCI HCD (Host Controller Driver) SRAM Glue.
 *
 */
#ifndef __LINUX_EHCI_SRAM_H
#define __LINUX_EHCI_SRAM_H

#ifdef CONFIG_HAVE_GENERIC_DMA_COHERENT
#define ehci_sram_declare dma_declare_coherent_memory
#define ehci_sram_release dma_release_declared_memory
#else
int ehci_sram_declare(struct device *dev, dma_addr_t bus_addr,
		      dma_addr_t device_addr, size_t size, int flags);
void ehci_sram_release(struct device *dev);
#endif /* CONFIG_HAVE_GENERIC_DMA_COHERENT */

#endif /* __LINUX_EHCI_SRAM_H */
