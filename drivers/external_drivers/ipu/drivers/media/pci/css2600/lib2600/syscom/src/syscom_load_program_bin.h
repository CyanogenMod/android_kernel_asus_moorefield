#ifndef _syscom_load_program_bin_h
#define _syscom_load_program_bin_h

#include "syscom_sp.h"
#include "type_support.h" /* workaround: needed because <vied/shared_memory_map.h> uses size_t
				but does not include header file */
#include <vied/shared_memory_access.h>

/* align on the SPC icache block size, or, preferrably page size */
#define _SP_ICACHE_ALIGN 512

#define _syscom_align(address, bytes) ((((unsigned long long)(address)) + ((bytes)-1)) & ~((bytes)-1))

#define _syscom_program_blob(prog)			(_hrt_blob_ ## prog .data)

#define _syscom_program_icache_target(prog)		(_hrt_icache_target_of_ ## prog)
#define _syscom_program_icache_source(prog)		(_hrt_icache_source_of_ ## prog)
#define _syscom_program_icache_size(prog)		(_hrt_icache_size_of_   ## prog)

#define _syscom_program_data_target(prog)		(_hrt_data_target_of_   ## prog)
#define _syscom_program_data_source(prog)		(_hrt_data_source_of_   ## prog)
#define _syscom_program_data_size(prog)			(_hrt_data_size_of_     ## prog)

#define _syscom_program_bss_target(prog)		(_hrt_bss_target_of_    ## prog)
#define _syscom_program_bss_size(prog)			(_hrt_bss_size_of_      ## prog)


#define syscom_load_program(cell, prog, ddr_addr, host_ddr_addr)\
{ \
	const char *blob = _syscom_program_blob(prog);\
	unsigned int aligned_ddr_addr             = ddr_addr;\
	unsigned long long aligned_host_ddr_addr  = host_ddr_addr;\
	\
	sp_invalidate_icache();\
	sp_set_icache_base_address(aligned_ddr_addr + _syscom_program_icache_target(prog));\
	shared_memory_store(0, aligned_host_ddr_addr + _syscom_program_icache_target(prog),\
		&blob[_syscom_program_icache_source(prog)],\
		_syscom_program_icache_size(prog));\
	sp_dmem_store(_syscom_program_data_target(prog), &blob[_syscom_program_data_source(prog)],\
		_syscom_program_data_size(prog));\
	sp_dmem_zero(_syscom_program_bss_target(prog), _syscom_program_bss_size(prog));\
}

#define syscom_alloc_program(prog) \
	_syscom_align(shared_memory_alloc(0, _syscom_program_icache_size(prog) + _SP_ICACHE_ALIGN), _SP_ICACHE_ALIGN)

#endif /* _syscom_load_program_bin_h */
