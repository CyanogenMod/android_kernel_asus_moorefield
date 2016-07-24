#ifndef _HRT_VECTOR_LDST_H_
#define _HRT_VECTOR_LDST_H_

#include "vector.h"
#include "memory.h"
#include "system_api.h"

#define _HRT_VEC_ELEM_BITS(cell) HRT_PROC_TYPE_PROP(cell, _vector_elem_bits)
#define _HRT_VEC_ELEM_PRECISION(cell) \
	HRT_PROC_TYPE_PROP(cell, _vector_elem_precision)
#define _HRT_VEC_NUM_ELEMS(cell) HRT_PROC_TYPE_PROP(cell, _vector_num_elems)
#define _HRT_VEC_ALIGN(cell)     HRT_PROC_TYPE_PROP(cell, _vector_alignment)
#define _HRT_VEC_BITS(cell) \
	(_HRT_VEC_ELEM_BITS(cell) * _HRT_VEC_NUM_ELEMS(cell))
#define _HRT_VEC_BYTES(cell)     _hrt_ceil_div(_HRT_VEC_BITS(cell), 8)

#ifdef C_RUN
#ifdef  C_RUN_DYNAMIC_LINK_PROGRAMS
extern void *csim_processor_get_crun_symbol(hive_proc_id p, const char *sym);
#define _HRT_VEC_ADDR(cell,vec)   ((char*)csim_processor_get_crun_symbol(cell,HRTSTR(vec)))
#else
#define _HRT_VEC_ADDR(cell,vec)   ((char*)(vec))
#endif
#define _HRT_VEC_MEM(vec)    memory_identifier_is_not_used_in_crun
#define _hrt_store_vector(cell, mem, target, source, vector_bytes) \
	memcpy(target, source, vector_bytes)
#define _hrt_load_vector(cell, mem, source, target, vector_bytes) \
	memcpy(target, source, vector_bytes)
#else
#define _HRT_VEC_ADDR(cell,vec)   HRTCAT(HIVE_ADDR_, vec)
#define _HRT_VEC_MEM(vec)    HRTCAT(HIVE_MEM_, vec)
#define _hrt_store_vector(cell, mem, target, source, vector_bytes) \
	hrt_mem_store(cell, mem, target, source, vector_bytes)
#define _hrt_load_vector(cell, mem, source, target, vector_bytes) \
	hrt_mem_load(cell, mem, source, target, vector_bytes)
#endif

#define _hrt_indexed_store_vector(cell, source, target, \
		target_idx, sign_extend) \
{ \
	char vc[_HRT_VEC_BYTES(cell)]; \
	int idx = target_idx; \
	_hrt_encode_vector(source, \
			   vc, \
			   _HRT_VEC_ELEM_BITS(cell), \
			   _HRT_VEC_ELEM_PRECISION(cell), \
			   _HRT_VEC_NUM_ELEMS(cell), \
			   sign_extend); \
	_hrt_store_vector(cell, \
			  _HRT_VEC_MEM(target), \
			  _HRT_VEC_ADDR(cell,target) + (idx*_HRT_VEC_ALIGN(cell)), \
			  vc, \
			  _HRT_VEC_BYTES(cell)); \
}

#define _hrt_indexed_load_vector(cell, source, source_idx, \
		target, sign_extend) \
{ \
	char vc[_HRT_VEC_BYTES(cell)]; \
	int idx = source_idx; \
	_hrt_load_vector(cell, _HRT_VEC_MEM(source), \
			_HRT_VEC_ADDR(cell,source) + (idx*_HRT_VEC_ALIGN(cell)), \
			vc, _HRT_VEC_BYTES(cell)); \
	_hrt_decode_vector(vc, target, _HRT_VEC_ELEM_BITS(cell), \
			_HRT_VEC_ELEM_PRECISION(cell), \
			_HRT_VEC_NUM_ELEMS(cell), sign_extend); \
}

#define hrt_indexed_store_signed_vector(cell, source, target, target_idx) \
	_hrt_indexed_store_vector(cell, source, target, target_idx, 1)
#define hrt_indexed_store_unsigned_vector(cell, source, target, target_idx) \
	_hrt_indexed_store_vector(cell, source, target, target_idx, 0)
#define hrt_indexed_load_signed_vector(cell, source, source_idx, target) \
	_hrt_indexed_load_vector(cell, source, source_idx, target, 1)
#define hrt_indexed_load_unsigned_vector(cell, source, source_idx, target) \
	_hrt_indexed_load_vector(cell, source, source_idx, target, 0)

#define hrt_scalar_store_signed_vector(cell, source, target) \
	hrt_indexed_store_signed_vector(cell, source, target, 0)
#define hrt_scalar_store_unsigned vector(cell, source, target) \
	hrt_indexed_store_unsigned_vector(cell, source, target, 0)
#define hrt_scalar_load_signed_vector(cell, source, target) \
	hrt_indexed_load_signed_vector(cell, source, 0, target)
#define hrt_scalar_load_unsigned_vector(cell, source, target) \
	hrt_indexed_load_unsigned_vector(cell, source, 0, target)

#endif /* _HRT_VECTOR_LDST_H_ */
