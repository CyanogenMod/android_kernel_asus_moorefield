#ifndef _sp_hrt_h_
#define _sp_hrt_h_

#define hrt_sp_dmem(cell) HRT_PROC_TYPE_PROP(cell, _dmem)

#define hrt_sp_dmem_master_port_address(cell) hrt_mem_master_port_address(cell, hrt_sp_dmem(cell))

#endif /* _sp_hrt_h_ */
