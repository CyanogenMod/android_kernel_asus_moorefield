#ifndef _vied_load_program_hrt_h
#define _vied_load_program_hrt_h

#define _ICACHE_ALIGN 512

#define _vied_align(address, bytes) ((((host_virtual_address_t)(address)) + ((bytes)-1)) & ~((bytes)-1))

#define _hrt_vied_set_icache_master_icache(cell, segment, offset) \
{\
  vied_virtual_address_t addr = aligned_ddr_addr;\
  hrt_cell_set_icache_segment_address(cell, segment, addr + (offset)); \
}

#define _hrt_vied_set_icache_master(section, segment, offset, cell) \
  HRTCAT(_hrt_vied_set_icache_master_, section)(cell, segment, offset)

#define _hrt_vied_store_code_section_text(to, from, size, cell)\
  _hrt_cell_prog_mem_store(cell, to, &blob[from], size)

#define _hrt_vied_store_code_section_pmem(to, from, size, cell) \
  _hrt_cell_prog_mem_store(cell, to, &blob[from], size)

#define _hrt_vied_store_code_section_icache(to, from, size, cell) \
{ \
  shared_memory_store(0, aligned_host_ddr_addr + to, &blob[from], size); \
  hrt_cell_invalidate_icache(cell); \
}

#define _hrt_vied_store_code_section(section, to, from, size, cell) \
  HRTCAT(_hrt_vied_store_code_section_, section)(to, from, size, cell)

#define _hrt_vied_store_code_empty(section, to, from, size, cell)

#if defined(C_RUN) || defined(HRT_UNSCHED)

#define vied_load_program_data(cell, prog) \
  hrt_cell_load_program_id(cell, prog)

#define vied_load_program_code(cell, prog, ddr_addr, host_ddr_addr) \
{ \
  vied_virtual_address_t da  = (vied_virtual_address_t)ddr_addr; \
  host_virtual_address_t hda = (host_virtual_address_t)host_ddr_addr; \
  da = da; \
  hda = hda; \
}

#define vied_alloc_program(prog) \
  shared_memory_alloc(0, 4)

#elif defined(HRT_CSIM)

#define vied_load_program_code(cell, prog, ddr_addr, host_ddr_addr)  \
{ \
  const char *blob = _hrt_program_blob(prog); \
  vied_virtual_address_t aligned_ddr_addr      = ddr_addr; \
  host_virtual_address_t aligned_host_ddr_addr = host_ddr_addr; \
  _hrt_program_transfer_func(prog)(_hrt_vied_store_code_section, _hrt_cell_store_data_empty, \
                                   _hrt_cell_zero_data_empty, _hrt_cell_write_view_table_empty, \
                                   _hrt_vied_set_icache_master, cell); \
}

#define vied_load_program_data(cell, prog) \
  hrt_cell_load_program_id(cell, prog)

#define vied_alloc_program(prog) \
  _vied_align(shared_memory_alloc(0, hrt_embedded_section_size(prog, icache) + _ICACHE_ALIGN), _ICACHE_ALIGN)

#else

#define vied_load_program_code(cell, prog, ddr_addr, host_ddr_addr)\
{ \
  const char *blob = _hrt_program_blob(prog); \
  vied_virtual_address_t aligned_ddr_addr       = ddr_addr; \
  host_virtual_address_t aligned_host_ddr_addr  = host_ddr_addr; \
  _hrt_program_transfer_func(prog)(_hrt_vied_store_code_section, _hrt_cell_store_data_empty,\
                                   _hrt_cell_zero_data_empty, _hrt_cell_write_view_table, \
                                   _hrt_vied_set_icache_master, cell);\
}

#define vied_load_program_data(cell, prog) \
{ \
  const char *blob = _hrt_program_blob(prog); \
  blob = blob; \
  _hrt_program_transfer_func(prog)(_hrt_vied_store_code_empty, _hrt_cell_store_data,\
                                   _hrt_cell_zero_data, _hrt_cell_write_view_table_empty, \
                                   _hrt_cell_set_icache_master_empty, cell);\
}

#define vied_alloc_program(prog) \
  _vied_align(shared_memory_alloc(0, hrt_embedded_section_size(prog, icache) + _ICACHE_ALIGN), _ICACHE_ALIGN)

#endif

#define vied_load_program(cell, prog, ddr_addr, host_ddr_addr) \
{ \
  vied_load_program_data(cell, prog); \
  vied_load_program_code(cell, prog, ddr_addr, host_ddr_addr); \
}

#endif /* _vied_load_program_hrt_h */
