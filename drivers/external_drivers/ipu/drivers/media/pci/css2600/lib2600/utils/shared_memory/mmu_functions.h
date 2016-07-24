#ifndef _MMU_FUNCTION_H_
#define _MMU_FUNCTION_H_

#include "subsystem_mmu_defs.h"

#define SUBSYSTEM_ID					0
#define MMU_TLB_INVALIDATE_REG_OFFSET	0x0
#define MMU_PAGETABLE_REG_OFFSET		0x4
#define MMU_BYTES_PER_PAGE				0x1000

static inline void invalidate_tlb(void)
{
	/* HSS: PSYS=ISYS=0. TODO: for common use, pass the ssid to the invalifate_tlb() */
	vied_subsystem_store_32(SUBSYSTEM_ID, HOST_MMU0_ADDR + MMU_TLB_INVALIDATE_REG_OFFSET, 1);
	vied_subsystem_store_32(SUBSYSTEM_ID, HOST_MMU1_ADDR + MMU_TLB_INVALIDATE_REG_OFFSET, 1);
}

static inline void set_L1_base_address(vied_physical_address_t address)
{
	vied_physical_address_t mmu_base = address;
	/* HSS: PSYS=ISYS=0. TODO: to use both in a platform, pass the ssid to the set_L1_base_address() */
	vied_subsystem_store_32(SUBSYSTEM_ID, HOST_MMU0_ADDR + MMU_PAGETABLE_REG_OFFSET, mmu_base/MMU_BYTES_PER_PAGE);
	vied_subsystem_store_32(SUBSYSTEM_ID, HOST_MMU1_ADDR + MMU_PAGETABLE_REG_OFFSET, mmu_base/MMU_BYTES_PER_PAGE);
}
#endif /* _MMU_FUNCTION_H_ */
