#include "input_system_system.h"

#define SUBSYSTEM_SLAVE   input_system_unis_logic_mmu_at_system_ciopipe_mt_cio_fifo_info_sl
#define SUBSYSTEM_MASTER  testbench_master_rec0_mt

#define MMU0              input_system_unis_logic_mmu_at_system_mmu0
#define MMU1              input_system_unis_logic_mmu_at_system_mmu1
#define MMU               MMU0

#define HOST_AB_MMU_ADDR  _hrt_master_to_slave_address_host_op0_to_input_system_unis_logic_ctrl_bus_ab_mt_mmu_sl_in
#define AB_MMU_MMU0_ADDR  _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_mmu_mt_out_to_input_system_unis_logic_mmu_at_system_mmu0_ctrl_in
#define AB_MMU_MMU1_ADDR  _hrt_master_to_slave_address_input_system_unis_logic_ctrl_bus_ab_mt_mmu_mt_out_to_input_system_unis_logic_mmu_at_system_mmu1_ctrl_in

#define HOST_MMU0_ADDR    (HOST_AB_MMU_ADDR + AB_MMU_MMU0_ADDR)
#define HOST_MMU1_ADDR    (HOST_AB_MMU_ADDR + AB_MMU_MMU1_ADDR)

#define MMU_PAGE_SHIFT    12
#define MMU_PAGE_BYTES    hrt_device_property(MMU,PageBytes)
#define MMU_PAGE_NUMBERS  hrt_device_property(MMU,PageTablesStorePageNumbersOnly)

/////////////////////////////////////////////////////////////////////////////////

#define HOST              host
#define HOST_MASTER       host_op0
#define HRT_ADDRESS_WIDTH 64

#define XMEM              testbench_ddr
#define XMEM_SLAVE        testbench_ddr_ip0
#define XMEM_BYTES        _hrt_sysmem_size(XMEM)
#define XMEM_ALIGN        32

#if defined(HRT_HW) && (!defined(HRT_CUSTOM_HOST) || defined(HRT_NO_OS))
extern unsigned long long vied_get_ddr_address (void);
#endif
