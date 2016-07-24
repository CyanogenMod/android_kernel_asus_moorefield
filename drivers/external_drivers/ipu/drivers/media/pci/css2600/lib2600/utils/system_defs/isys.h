#include "input_system_system.h"
#include <vied/vied_subsystem_access_initialization.h>
#include <vied/vied_subsystem_access.h>
#include <vied/shared_memory_access.h>
#include <vied/shared_memory_map.h>

//#define SUBSYSTEM_SLAVE   input_system_unis_logic_mmu_at_system_cfg_bus_sl0
#define SUBSYSTEM_SLAVE   input_system_unis_logic_mmu_at_system_ciopipe_mt_cio_fifo_info_sl
#define SUBSYSTEM_MASTER  testbench_master_rec0_mt

#define MMU0              input_system_unis_logic_mmu_at_system_mmu0
#define MMU1              input_system_unis_logic_mmu_at_system_mmu1
#define MMU               MMU0
#define MMU_PAGE_SHIFT    12
#define MMU_PAGE_BYTES    hrt_device_property(MMU,PageBytes)
#define MMU_PAGE_NUMBERS  hrt_device_property(MMU,PageTablesStorePageNumbersOnly)

/////////////////////////////////////////////////////////////////////////////////////

#define HOST              host
#define HOST_MASTER       host_op0

#define XMEM              testbench_ddr
#define XMEM_SLAVE        testbench_ddr_ip0
#define XMEM_BYTES        _hrt_sysmem_size(XMEM)
#define XMEM_ALIGN        32


#define host_ddr_addr()   hrt_master_to_slave_address(HOST_MASTER,   XMEM_SLAVE)
#define mmu_ddr_addr()    hrt_master_to_slave_address(SUBSYSTEM_MASTER, XMEM_SLAVE)

/////////////////////////////////////////////////////////////////////////////////////

#define UNIS_PIXEL_BUFFER_BASE input_system_unis_logic_pixel_buffer_ip0_master_port_address
#define UNIS_GPREG_DMA         input_system_unis_logic_fw_dma_eqc_sp1_master_port_address
#define UNIS_GPREG_HOST        input_system_unis_logic_gpreg_slv_in_master_port_address

#include <mmu.h>
#include "mmu_functions.h"

#define TB_REGS   _hrt_master_to_slave_address_host_op0_to_testbench_tb_regs_slv_in
#define SEC_REGS  _hrt_master_to_slave_address_host_op0_to_testbench_sec_regs_slv_in
