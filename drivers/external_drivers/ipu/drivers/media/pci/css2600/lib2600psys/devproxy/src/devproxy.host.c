/*
* This module presents host code to load and run the devproxy
*/

#ifdef C_RUN
#include "ia_css_devproxy.h"
#else
/* next name of header/map file is dependent on name of program in makefile,
   not sure how to share this yet */
#include "devproxy.map.h"
#endif

#include "psys.h"

void ia_css_devproxy_program_load(void)
{
#if defined(C_RUN)
	hrt_cell_load_program_id(PSYS_SPP0, devproxy);
#else
	const int id = 0;
	const int idm = 0;
        host_virtual_address_t prog_addr = vied_alloc_program(devproxy);
        vied_virtual_address_t cell_prog_addr = shared_memory_map(id, idm, prog_addr);
        vied_load_program(PSYS_SPP0, devproxy, cell_prog_addr, prog_addr);
#endif
}

void ia_css_devproxy_program_start(void)
{
	hrt_cell_start_function(PSYS_SPP0, devproxy);
}
