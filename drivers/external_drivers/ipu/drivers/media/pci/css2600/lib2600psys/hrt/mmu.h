#ifndef _hrt_mmu_h
#define _hrt_mmu_h

#include <hrt/api.h>
#include "mmu_defs.h"

#define hrt_mmu_slave_port(mmu_id)    HRTCAT(mmu_id,_ctrl_in)
#define hrt_mmu_register_address(reg) (_HRT_MMU_REG_ALIGN * (reg))

#define _hrt_mmu_set_register(mmu_id, reg, val) \
  _hrt_slave_port_store_32_volatile(hrt_mmu_slave_port(mmu_id), hrt_mmu_register_address(reg), (val))

#define _hrt_mmu_get_register(mmu_id, reg) \
  _hrt_slave_port_load_32_volatile(hrt_mmu_slave_port(mmu_id), hrt_mmu_register_address(reg))

/* register addresses */
#define hrt_mmu_invalidate_TLB_register_address()          hrt_mmu_register_address(_HRT_MMU_INVALIDATE_TLB_REG_IDX)
#define hrt_mmu_page_table_base_address_register_address() hrt_mmu_register_address(_HRT_MMU_PAGE_TABLE_BASE_ADDRESS_REG_IDX)
#define hrt_mmu_page_table_tag_register_address()          hrt_mmu_register_address(_HRT_MMU_PAGE_TABLE_TAG_REG_IDX)

#define hrt_mmu_invalidate_TLB(mmu_id) \
  _hrt_mmu_set_register(mmu_id, _HRT_MMU_INVALIDATE_TLB_REG_IDX, 1)

#define _hrt_page_size_or_one(mmu_id) \
   (hrt_device_property(mmu_id,PageTablesStorePageNumbersOnly) ? (hrt_device_property(mmu_id,PageBytes)) : 1)


#define hrt_mmu_set_page_table_base_address(mmu_id, addr) \
  _hrt_mmu_set_register(mmu_id, _HRT_MMU_PAGE_TABLE_BASE_ADDRESS_REG_IDX, addr/_hrt_page_size_or_one(mmu_id))

#define hrt_mmu_get_page_table_base_address(mmu_id) \
  ((unsigned long long)_hrt_mmu_get_register(mmu_id, _HRT_MMU_PAGE_TABLE_BASE_ADDRESS_REG_IDX) * _hrt_page_size_or_one(mmu_id))


#define hrt_mmu_set_page_table_tag(mmu_id, info) \
  _hrt_mmu_set_register(mmu_id, _HRT_MMU_PAGE_TABLE_TAG_REG_IDX, info)

#define hrt_mmu_get_page_table_tag(mmu_id) \
  _hrt_mmu_get_register(mmu_id, _HRT_MMU_PAGE_TABLE_TAG_REG_IDX)

#endif /* _hrt_mmu_h */
