#include "psys.h"

#include "hrt_mm.h"


/***********************************************************************************************/
/* #include <mmu.h> */ /* Changes due to AB instantiation */

#include <hrt/api.h>
#include <mmu_defs.h>

typedef enum {
	processing_system_unps_logic_mmu_at_system_mmu0_id,
	processing_system_unps_logic_mmu_at_system_mmu1_id,
	mmu_num_devices
} mmu_t;


static const uint32_t mmu_address[mmu_num_devices] =
{
	0x3C0000,
	0x3C0100
};


#define hrt_mmu_slave_port(mmu_id)    HRTCAT(mmu_id,_id)
#define hrt_mmu_register_address(reg) (_HRT_MMU_REG_ALIGN * (reg))

#define _hrt_mmu_set_register(mmu_id, reg, val) \
  _hrt_master_port_store_32_volatile_msg(mmu_address[hrt_mmu_slave_port(mmu_id)]+ hrt_mmu_register_address(reg), (val), NULL)
/*  _hrt_slave_port_store_32_volatile(hrt_mmu_slave_port(mmu_id), hrt_mmu_register_address(reg), (val)) */

#define _hrt_mmu_get_register(mmu_id, reg) \
  _hrt_master_port_load_32_volatile_msg(mmu_address[hrt_mmu_slave_port(mmu_id)] + hrt_mmu_register_address(reg), NULL)
/*  _hrt_slave_port_load_32_volatile(hrt_mmu_slave_port(mmu_id), hrt_mmu_register_address(reg))*/

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

/***********************************************************************************************/


#define mmu_ddr_addr()        hrt_master_to_slave_address(SUBSYSTEM_MASTER, XMEM_SLAVE)
#define host_ddr_addr()       hrt_master_to_slave_address(HOST_MASTER,   XMEM_SLAVE)

/* we use malloc/free if the custom host is enabled and the
   NO_OS is not enabled. When NO_OS is enabled, we're running on the SoC
   with no operating system, where malloc returns a physical address, so we
   need hmm to do the virtual memory management. */
#if defined(C_RUN)
#define USE_HMM 0
#elif defined(HRT_HW) && defined(HRT_CUSTOM_HOST) && !defined(HRT_NO_OS)
#define USE_HMM 0
#else
#define USE_HMM 1
#endif

#if USE_HMM
static unsigned init_done = 0;

static void inv_tlb(void)
{
  hrt_mmu_invalidate_TLB(MMU0);
  hrt_mmu_invalidate_TLB(MMU1);
}
static hmm_config ddr_config      ;

#define hrt_mm_init(void) { if (!init_done) hrt_mm_do_init();}

static void hrt_mm_do_init(void)
{
  hmm_pptr mmu_base = 0;
  if (init_done) return;
  ddr_config       = hmm_default_config();

  /* We initialize the memory manager with the physical memory and
     enable the virtual memory system. This gives us the page used
     for the first level of pages which is programmed into the MMU.
     To prevent the DMA overwriting data that immediately follows target,
     we make sure every allocated piece of memory is aligned to the width
     in bytes of a system word as seen by the DMA */
  hmm_init_virt_mem(MMU_PAGE_SHIFT, inv_tlb, XMEM_ALIGN, MMU_PAGE_NUMBERS);
#if defined (HRT_CSIM)
  hmm_init_l_pages(1); /* prevent mmu from generating uninitialized read warnings
                          when fetching a block of L1 or L2 data */
#endif

  printf("host ddr address: %llX\n", host_ddr_addr());
  printf("mmu  ddr address: %llX\n", mmu_ddr_addr());

#if defined (HRT_RTL)
  hmm_config_register_zone(ddr_config, host_ddr_addr(), mmu_ddr_addr(), XMEM_BYTES);
#elif defined (HRT_NO_OS)
  {
    /* this is the case for the IA SOC with no OS, we allocate a piece of
       physical memory and assign that to the hmm library. */
    unsigned int bytes = 128*1024*1024; /* 128MB */
    void *phys_mem = malloc(bytes);
    hmm_register_zone((unsigned int)phys_mem, (unsigned int)phys_mem, bytes);
  }
#else
  hmm_config_register_zone(ddr_config, host_ddr_addr(), mmu_ddr_addr(), XMEM_BYTES);
#endif

  mmu_base = hmm_l1_page_mmu_address();
  hrt_mmu_set_page_table_base_address(MMU0, mmu_base);
  hrt_mmu_set_page_table_base_address(MMU1, mmu_base);
  inv_tlb();
  init_done = 1;
}
#else
#define hrt_mm_init()
#include <stdlib.h>
#endif /* USE_HMM */

void hrt_mm_free(hmm_ptr virt_addr)
{
#if USE_HMM
  hmm_free(virt_addr);
#else
  hmm_ptr original_ptr_addr = virt_addr-XMEM_ALIGN;
  hmm_ptr original_ptr = *(hmm_ptr*) original_ptr_addr;
  free((void *)original_ptr );
#endif
}

hmm_ptr hrt_mm_config_alloc(hmm_config config, size_t bytes) {
#if USE_HMM
  hrt_mm_init();
  return hmm_config_alloc(config, bytes);
#else
  (void)config;
  return (hmm_ptr)malloc(bytes);
#endif
}

hmm_ptr hrt_mm_config_calloc(hmm_config config, size_t bytes)
{
#if USE_HMM
  hrt_mm_init();
  return hmm_config_calloc(config, bytes);
#else
  (void)config;
  return (hmm_ptr)calloc(bytes, 1);
#endif
}

hmm_ptr hrt_mm_config_alloc_contiguous(hmm_config config, size_t bytes)
{
#if USE_HMM
  hrt_mm_init();
  return hmm_config_alloc_contiguous(config, bytes);
#else
  (void)config;
  return (hmm_ptr)malloc(bytes);
#endif
}

hmm_ptr hrt_mm_config_calloc_contiguous(hmm_config config, size_t bytes)
{
#if USE_HMM
  hrt_mm_init();
  return hmm_config_calloc_contiguous(config, bytes);
#else
  (void)config;
  return (hmm_ptr)calloc(bytes, 1);
#endif
}

hmm_ptr hrt_mm_alloc(size_t bytes)
{
#if USE_HMM
  hrt_mm_init();
  return hrt_mm_config_alloc(ddr_config, bytes);
#else
  hmm_ptr original_ptr, aligned_ptr;
  /* Allocate extra bytes to skip the first sub-word and to store the original pointer */
  original_ptr             = (hmm_ptr) malloc( bytes + XMEM_ALIGN + XMEM_ALIGN-1 );
  aligned_ptr              = (original_ptr + XMEM_ALIGN-1) & ~(XMEM_ALIGN-1);
  /* Store original pointer, such that it can be freed later */
  *(hmm_ptr*)(aligned_ptr) = original_ptr;
  aligned_ptr             += XMEM_ALIGN;
  return( aligned_ptr );
#endif
}

hmm_ptr hrt_mm_calloc(size_t bytes)
{
#if USE_HMM
  hrt_mm_init();
  return hrt_mm_config_calloc(ddr_config, bytes);
#else
  hmm_ptr original_ptr, aligned_ptr;
  /* Allocate extra bytes to skip the first sub-word and to store the original pointer */
  original_ptr             = (hmm_ptr) calloc( bytes + XMEM_ALIGN + XMEM_ALIGN-1, 1 );
  aligned_ptr              = (original_ptr + XMEM_ALIGN-1) & ~(XMEM_ALIGN-1);
  /* Store original pointer, such that it can be freed later */
  *(hmm_ptr*)(aligned_ptr) = original_ptr;
  aligned_ptr             += XMEM_ALIGN;
  return( aligned_ptr );
#endif
}

hmm_ptr hrt_mm_alloc_contiguous(size_t bytes) {
#if USE_HMM
  hrt_mm_init();
  return hrt_mm_config_alloc_contiguous(ddr_config, bytes);
#else
  return hrt_mm_alloc(bytes);
#endif
}

hmm_ptr hrt_mm_calloc_contiguous(size_t bytes) {
#if USE_HMM
  hrt_mm_init();
  return hrt_mm_config_calloc_contiguous(ddr_config, bytes);
#else
  return hrt_mm_calloc(bytes);
#endif
}

//////////////////////////////////////////////////////////////

void hrt_mm_load(hmm_ptr virt_addr, void *data, size_t bytes)
{
#if USE_HMM
  hmm_load(virt_addr, data, bytes);
#else
  memcpy(data, (void*)virt_addr, bytes);
#endif
}

void hrt_mm_store(hmm_ptr virt_addr, const void *data, size_t bytes)
{
#if USE_HMM
  hmm_store(virt_addr, data, bytes);
#else
  memcpy((void*)virt_addr, data, bytes);
#endif
}

void hrt_mm_set(hmm_ptr virt_addr, int data, size_t bytes)
{
#if USE_HMM
  hmm_set(virt_addr, data, bytes);
#else
  memset((void*)virt_addr, data, bytes);
#endif
}

int hrt_mm_load_int(hmm_ptr virt_addr)
{
#if USE_HMM
  return hmm_load_32(virt_addr);
#else
  return *(int*)virt_addr;
#endif
}

short hrt_mm_load_short(hmm_ptr virt_addr)
{
#if USE_HMM
  return hmm_load_16(virt_addr);
#else
  return *(short*)virt_addr;
#endif
}

char hrt_mm_load_char(hmm_ptr virt_addr)
{
#if USE_HMM
  return hmm_load_8(virt_addr);
#else
  return *(char*)virt_addr;
#endif
}

void hrt_mm_store_int(hmm_ptr virt_addr, int data)
{
#if USE_HMM
  hmm_store_32(virt_addr, data);
#else
  *(int*)virt_addr = data;
#endif
}

void hrt_mm_store_short(hmm_ptr virt_addr, short data)
{
#if USE_HMM
  hmm_store_16(virt_addr, data);
#else
  *(short*)virt_addr = data;
#endif
}

void hrt_mm_store_char(hmm_ptr virt_addr, char data)
{
#if USE_HMM
  hmm_store_8(virt_addr, data);
#else
  *(char*)virt_addr = data;
#endif
}

hmm_ptr hrt_mm_virt_to_phys(hmm_ptr virt_addr)
{
#if USE_HMM
  return hmm_virt_to_phys(virt_addr);
#else
  return virt_addr; /* not sure what to do here, Intel needs to decide */
#endif
}
