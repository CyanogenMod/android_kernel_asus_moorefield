#include "processing_system_system.h"

#define SUBSYSTEM_SLAVE   processing_system_unps_logic_mmu_at_system_ciopipe_mt_cio_fifo_info_sl
#define SUBSYSTEM_MASTER  testbench_master_rec0_mt

#define MMU0              processing_system_unps_logic_mmu_at_system_mmu0
#define MMU1              processing_system_unps_logic_mmu_at_system_mmu1
#define MMU               MMU0
#define MMU_PAGE_SHIFT    12
#define MMU_PAGE_BYTES    hrt_device_property(MMU,PageBytes)
#define MMU_PAGE_NUMBERS  hrt_device_property(MMU,PageTablesStorePageNumbersOnly)

/////////////////////////////////////////////////////////////////////////////////////

#define HOST              host
#define HOST_MASTER       host_op0
#ifndef HRT_ADDRESS_WIDTH
#define HRT_ADDRESS_WIDTH 64
#endif

#define XMEM              testbench_ddr
#define XMEM_SLAVE        testbench_ddr_ip0
#define XMEM_BYTES        _hrt_sysmem_size(XMEM)
#define XMEM_ALIGN        32


