#ifndef _mmu_functions_h_
#define _mmu_functions_h_

#include <mmu.h>

#define _hrt_mmu_set_register_base_addr(base, reg, val) \
          _hrt_master_port_store_32_volatile((base + hrt_mmu_register_address(reg)), (val))

#define _hrt_mmu_get_register_base_addr(base, reg) \
          _hrt_master_port_load_32_volatile((base + hrt_mmu_register_address(reg)))

#define hrt_mmu_invalidate_TLB_with_base(base) \
          _hrt_mmu_set_register_base_addr(base, _HRT_MMU_INVALIDATE_TLB_REG_IDX, 1)

#define hrt_mmu_set_page_table_base_address_with_base(base, addr) \
          _hrt_mmu_set_register_base_addr(base, _HRT_MMU_PAGE_TABLE_BASE_ADDRESS_REG_IDX, addr)

#define hrt_mmu_get_page_table_base_address_with_base(base) \
          _hrt_mmu_get_register_base_addr(base, _HRT_MMU_PAGE_TABLE_BASE_ADDRESS_REG_IDX)

#endif
