#ifndef HRT_CELL_H
#define HRT_CELL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "version.h"
#include "hive_types.h"
#include "stat_ctrl.h"
#include "host.h"
#ifndef HRT_VP
#include "syscall.h"
#endif
#include "memory.h"
#include "embed.h"

#if defined(CRUN) || defined(HRT_CSIM)
#include <hrt/csim/csim_interface.h>
#endif

#define _HRT_CRUN_EXE_PREFIX    "crun_executable="
#define _HRT_CRUN_DLIB_PREFIX   "crun_dlib="
#define _HRT_CSIM_EXE_PREFIX    "csim_executable="
#define _HRT_VP_EXE_PREFIX    "csim_executable="

#if defined(HRT_RTL)
unsigned int _hrt_cell_wait_poll_interval;
#endif

#if defined(C_RUN)
#define _hrt_ctl_reset_loop_cache(c)
#else
#define _hrt_ctl_reset_loop_cache(c) \
	do { \
		hrt_ctl_invalidate_loop_cache(c); \
		hrt_sleep(); \
		hrt_ctl_invalidate_loop_cache_off(c); \
	} while (0)
#endif

#define _hrt_cell_load_program_csim(c, p) \
	_hrt_cell_store_program_filename(c, p, _HRT_CSIM_EXE_PREFIX)

#define _hrt_cell_load_program_id_csim(c, p) \
	do { \
		_hrt_cell_dummy_use_blob(p); \
		_hrt_cell_load_program_csim(c, HRTSTR(p)); \
	} while (0)

#define _hrt_cell_load_program_vp(c, p) \
{ \
	 _hrt_cell_store_program_filename(c, p, _HRT_VP_EXE_PREFIX);   \
}

#define _hrt_cell_load_program_id_vp(c, p) \
	do { \
		_hrt_cell_dummy_use_blob(p); \
		_hrt_cell_load_program_vp(c, HRTSTR(p)); \
	} while (0)

#if defined(C_RUN) && defined(C_RUN_DYNAMIC_LINK_PROGRAMS)
#define _HRT_CRUN_PROGRAM_PREFIX _HRT_CRUN_DLIB_PREFIX
#else
#define _HRT_CRUN_PROGRAM_PREFIX _HRT_CRUN_EXE_PREFIX
#endif

#define _hrt_cell_load_program_crun(cell, file_name) \
{ \
  _hrt_cell_store_program_filename(cell,file_name, _HRT_CRUN_PROGRAM_PREFIX); \
}


#define _hrt_cell_load_program_elf(c,p) \
{ \
	HRTCAT(_hrt_cell_load_program_, p)(c); \
}

#if defined (C_RUN)
#  define _hrt_cell_load_program(c,p) \
	_hrt_cell_load_program_crun(c,p)
#  define _hrt_cell_load_program_id(c,p) \
	_hrt_cell_load_program(c, HRTSTR(p))
#  define _hrt_cell_load_program_for_icache(c,p,addr) \
	_hrt_cell_load_program_id(c,p)
#elif defined (HRT_CSIM)
#  define _hrt_cell_load_program(c,p) \
	_hrt_cell_load_program_csim (c,p)
#  define _hrt_cell_load_program_id(c,p) \
	_hrt_cell_load_program_id_csim(c, p)
#  define _hrt_cell_load_program_for_icache(c,p,addr) \
	_hrt_cell_load_program_csim_for_icache(c,p,addr)
#elif defined (HRT_VP)
#  define _hrt_cell_load_program(c,p) \
	_hrt_cell_load_program_vp (c,p)
#  define _hrt_cell_load_program_id(c,p) \
	_hrt_cell_load_program_id_vp(c, p)
#  define _hrt_cell_load_program_for_icache(c,p,addr) \
	_hrt_cell_load_program_csim_for_icache(c,p,addr)
#else
#  define _hrt_cell_load_program(c,p) \
	_hrt_cell_load_program_from_elf_file(c, p)
#  define _hrt_cell_load_program_id(c,p) \
	_hrt_cell_load_program_elf(c,p)
#  define _hrt_cell_load_program_for_icache(c,p,addr) \
	_hrt_cell_load_program_elf_for_icache(c,p,addr)
#endif /* HRT_CSIM */

#if defined (C_RUN) || defined(HRT_UNSCHED)
#define hrt_embedded_section_size(prog, section) 0
#else
#define hrt_embedded_section_size(prog, section) \
	HRTCAT(HRTCAT(_hrt_section_size_, prog), HRTCAT(_, section))
#endif
#define _hrt_section_target_address(section) \
	HRTCAT(section,_target_address)

#define _hrt_cell_store_code_section_text(to, from, size, cell) \
	_hrt_cell_prog_mem_store(cell, to, &blob[from], size)
#define _hrt_cell_store_code(section, to, from, size, cell) \
	HRTCAT(_hrt_cell_store_code_section_, section)(to, from, size, cell)
#define _hrt_cell_store_data(mem, mem_offset, from, size, cell) \
	hrt_mem_store(cell, mem, mem_offset, &blob[from], size)
#define _hrt_cell_zero_data(mem, mem_offset, size, cell) \
	hrt_mem_zero(cell, mem, mem_offset, size)
#define _hrt_cell_write_view_table(from, size, cell) \
	_hrt_cell_write_view_table_word(cell, &blob[from], size)

/* Empty version of the section transfer functions */
#define _hrt_cell_store_code_empty(to, from, size, cell)
#define _hrt_cell_store_data_empty(mem, mem_offset, from, size, cell)
#define _hrt_cell_zero_data_empty(mem, mem_offset, size, cell)
#define _hrt_cell_write_view_table_empty(from, size, cell)
#define _hrt_cell_set_icache_master_empty(section, reg, offset, cell)

#define _hrt_cell_store_code_for_icache(section, to, from, size, cell) \
	hrt_master_port_store(target_addr + to, &blob[from], size)

#define _hrt_cell_store_code_for_csim_icache(section, to, from, size, cell) \
	hrt_master_port_store(target_addr + to, &blob[from], size)

#define _hrt_cell_load_program_from_blob(cell, p, code_func, store_func, zero_func, table_func, icache_master_func) \
	do { \
		const char *blob = _hrt_program_blob(p); \
		_hrt_program_transfer_func(p)(code_func, store_func, zero_func, table_func, icache_master_func, cell); \
	} while (0)

#define _hrt_cell_load_program_embedded(cell, p) \
 	_hrt_cell_load_program_from_blob(cell, p, \
        				 _hrt_cell_store_code, \
					 _hrt_cell_store_data, \
					 _hrt_cell_zero_data, \
					 _hrt_cell_write_view_table, \
					 _hrt_cell_set_icache_master_empty)

/* upload the program to an address, the icache will fetch it from there */
#define _hrt_cell_load_program_elf_for_icache(cell, prog, addr) \
	do { \
		hive_address target_addr = (hive_address)addr; \
		_hrt_cell_load_program_from_blob( \
                		cell, \
				prog, \
				_hrt_cell_store_code_for_icache, \
				_hrt_cell_store_data, \
				_hrt_cell_zero_data, \
				_hrt_cell_write_view_table, \
				_hrt_cell_set_icache_master_empty); \
	} while (0)

/* For csim, we only use the blob to initialize the icache memory width some data to prevent
   a slur of uninitialized read warnings. */
#define _hrt_cell_load_program_csim_for_icache(cell, prog, addr) \
	do { \
		hive_address target_addr = (hive_address)addr; \
		_hrt_cell_load_program_id(cell, prog); \
		_hrt_cell_load_program_from_blob( \
				cell, \
				prog, \
				_hrt_cell_store_code_for_csim_icache, \
				_hrt_cell_store_data_empty, \
				_hrt_cell_zero_data_empty, \
				_hrt_cell_write_view_table_empty, \
				_hrt_cell_set_icache_master_empty); \
	} while (0)

#define _hrt_section_num_regs(section)   HRTCAT3(HIVE_ICACHE_,section,_NUM_SEGMENTS)
#define _hrt_section_start_reg(section)  HRTCAT3(HIVE_ICACHE_,section,_SEGMENT_START)

#define hrt_cell_set_section_base_addresses(cell, section, addr) \
	do {\
		int i; \
		for (i=0; i < _hrt_section_num_regs(section); i++) {\
			hrt_ctl_set_ext_base_address( \
					cell,  \
					_hrt_cell_icache_master_interface(cell), \
					_hrt_section_start_reg(section) + i, \
					(addr) + i * _hrt_cell_icache_segment_size(cell)); \
		} \
	} while (0)

#define hrt_cell_set_icache_segment_address(cell, segment, addr) \
  hrt_ctl_set_ext_base_address(cell, _hrt_cell_icache_master_interface(cell), segment, addr)

#define hrt_cell_set_icache_segment_info(cell, segment, info) \
  hrt_ctl_set_ext_base_info(cell, _hrt_cell_icache_master_interface(cell), segment, info)

#define hrt_cell_set_icache_base_address(cell, addr) \
  hrt_ctl_set_ext_base_address(cell, _hrt_cell_icache_master_interface(cell), 0, addr)

#define hrt_cell_get_icache_base_address(cell) \
  hrt_ctl_get_ext_base_address(cell, _hrt_cell_icache_master_interface(cell), 0)

#define hrt_cell_set_icache_base_info(cell, info) \
  hrt_ctl_set_ext_base_info(cell, _hrt_cell_icache_master_interface(cell), 0, info)

#define hrt_cell_get_icache_base_info(cell) \
  hrt_ctl_get_ext_base_info(cell, _hrt_cell_icache_master_interface(cell), 0)

#define hrt_cell_icache_prefetch_enable(cell) \
  hrt_ctl_set_cache_prefetch_enable(cell, _hrt_cell_icache(cell), 1)

#define hrt_cell_icache_prefetch_disable(cell) \
  hrt_ctl_set_cache_prefetch_enable(cell, _hrt_cell_icache(cell), 0)

#define hrt_cell_invalidate_icache(cell) \
  hrt_ctl_invalidate_cache(cell, _hrt_cell_icache(cell))

#define _hrt_cell_prog_mem_store_8(cell, offset, data) \
  _hrt_slave_port_store_8(_hrt_cell_slave_port(cell, _hrt_cell_prog_mem_slave_port(cell)), \
                          _hrt_cell_prog_mem_base(cell) + (offset), \
                          data)

#define _hrt_cell_prog_mem_store(cell, offset, data, size) \
  _hrt_slave_port_store(_hrt_cell_slave_port(cell, _hrt_cell_prog_mem_slave_port(cell)), \
                        _hrt_cell_prog_mem_base(cell) + (offset), \
                        data, size )

#define _hrt_cell_write_view_table_word(cell, data, size) \
  _hrt_slave_port_store(_hrt_cell_slave_port(cell, _hrt_cell_prog_mem_slave_port(cell)), \
                        _hrt_cell_view_table_base(cell), data, size )

#if (defined HRT_CSIM) || (defined HRT_VP)
/* Upload the name of the shared library to the status and control memory.
   Csim then uses that to open the file. We do not use malloc and
   strcpy because that would require libc which we do not always have
   on a Silicon Hive processor (which can also use this function).
 */
#define _hrt_cell_store_program_filename(cell, csim_file_name, prefix_label)\
	do { \
		const char *in_ptr = prefix_label; \
		_hrt_ctl_enter_special_mode(cell); \
		while(*in_ptr != '\0') { \
			_hrt_ctl_store_program_char(cell, *in_ptr); \
			in_ptr++; \
		} \
		in_ptr = csim_file_name;  \
		while(*in_ptr != '\0') { \
			_hrt_ctl_store_program_char(cell, *in_ptr); \
			in_ptr++; \
		} \
		_hrt_ctl_store_program_char(cell, '\0'); \
	} while (0)
#endif

#define _hrt_cell_start_function_crun(p,f) \
  do { \
    csim_processor_set_crun_func(p,HRTSTR(f)); \
    hrt_ctl_run(p, hive_true); \
    hrt_ctl_start(p); \
    hrt_sleep(); \
  } while (0)

/* compiled simulator specific calls because no elf file is used: */
#if defined(C_RUN)
extern void csim_processor_set_crun_func(hive_proc_id p, const char *func);
#  define hrt_cell_start_function(p,f) _hrt_cell_start_function_crun(p,f)
#  define hrt_cell_exit_value(p)       0
#else // C_RUN
#  define hrt_cell_start_function(p,f) _hrt_cell_start(p, HRTCAT3(HIVE_ADDR_,f,_entry))
#  define hrt_cell_exit_value(p)       hrt_scalar_load(p, int, __exit_value)
#endif // C_RUN

#define hrt_cell_load_program(c, p) \
{ \
  _hrt_cell_load_program(c, p); \
  _hrt_ctl_reset_loop_cache(c); \
}

#define hrt_cell_load_program_id(c,p) \
	{ \
		_hrt_cell_load_program_id(c, p); \
		_hrt_ctl_reset_loop_cache(c); \
	}

#define hrt_cell_load_program_for_icache(c, p, addr) \
	{ \
		_hrt_cell_load_program_for_icache(c, p, addr); \
		_hrt_ctl_reset_loop_cache(c); \
	}

#define hrt_cell_run_function(p,f) \
	do { \
		hrt_cell_start_function(p,f); \
		hrt_cell_wait(p); \
	} while(0)

#define _hrt_cell_start(proc, start_address) \
 do {\
   hrt_ctl_set_start_address(proc, start_address); \
   hrt_ctl_run(proc, hive_true); \
   hrt_ctl_start(proc); \
   hrt_sleep(); \
 } while (0)

#if defined(HRT_HW) || defined(HRT_MT_INT) || defined(HRT_RTL)
/* On the hardware, the host does not handle
   the system calls from the hive processor. Also,
   on Silicon Hive processors in crun, we also do not handle
   system calls. */
#define hrt_cell_wait_with_interval(proc, interval) \
	do {\
		unsigned int _i; \
		for(_i = 0; _i < (interval); _i++) hrt_sleep(); \
	} while (!hrt_ctl_is_ready(proc))
#else
#define hrt_cell_wait_with_interval(proc, interval) \
	do {\
		if (hrt_ctl_is_sleeping(proc)) {\
			hrt_syscall_server (proc);\
			hrt_ctl_start(proc);\
		}\
		hrt_sleep();\
	} while (!hrt_ctl_is_ready(proc));
#endif /* HRT_HW */

#if defined(HRT_RTL)
#define hrt_cell_wait(proc) \
  hrt_cell_wait_with_interval(proc, 1 + _hrt_cell_wait_poll_interval)
#else
#define hrt_cell_wait(proc) \
  hrt_cell_wait_with_interval(proc, 1)
#endif

#if defined(HRT_RTL)
#define hrt_set_cell_wait_poll_interval(interval) \
  _hrt_cell_wait_poll_interval = (interval) - 1
#else
#define hrt_set_cell_wait_poll_interval(interval) \
  (void)(interval)
#endif

#define hrt_cell_init(cell) HRTCAT(hrt_cell_init_,cell)()
#define hrt_cell_wait_timeout(cell, max_loops) HRTCAT(hrt_cell_wait_timeout_, cell)(max_loops)

#ifdef __cplusplus
}
#endif

#endif /* HRT_CELL_H */
