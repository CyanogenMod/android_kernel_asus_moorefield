/*
* This module presents host code to load and run the devproxy
*/

#ifdef C_RUN
#include "test_infra_loader.spc.h"
#else
#include "test_infra_spc_main.map.h"
#endif

#include "psys.h"

#define PSYS_SP_CONTROL processing_system_unps_logic_spc_tile_sp

void test_infra_spc_main_program_load(void)
{
#ifdef C_RUN
	hrt_cell_load_program_id(PSYS_SP_CONTROL, test_infra_spc_main);
#else
        host_virtual_address_t prog_addr = vied_alloc_program(test_infra_spc_main);
        vied_virtual_address_t cell_prog_addr = shared_memory_map(0, 0, prog_addr);
        vied_load_program(PSYS_SP_CONTROL, test_infra_spc_main, cell_prog_addr, prog_addr);
#endif
}

void test_infra_spc_main_program_start(void)
{
	hrt_cell_start_function(PSYS_SP_CONTROL, test_infra_spc_main);
}

void test_infra_spc_main_program_wait(void)
{
	hrt_cell_wait(PSYS_SP_CONTROL);
}

void test_infra_spc_main_program_config(int n)
{
	hrt_scalar_store(PSYS_SP_CONTROL, uint, wait_count, (uint32_t)n);
}

