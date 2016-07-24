#ifndef _hrt_vector_h
#define _hrt_vector_h

/* This file contains support to convert between vectors represented
   as arrays of int (on the host) and real packed vectors on a target
   processor.
   It also contains functions to transfer these vectors between a host
   processor and a target (vector) processor.
   Note that this file is not to be included directly by application code,
   but only through a processor specific header file. The reason for this
   is this set of macros has to be defined for a given processor of type
   <proc_type>:
   <proc_type>_vector_alignment:      the alignment in bytes of an entire vector in memory
   <proc_type>_vector_elem_bits:      number of bits per vector element in memory
   <proc_type>_vector_elem_precision: number of valid bits per vector element, this can be smaller
                                      than the vector_elem_bits if the width of a vector element
                                      differs between the vector memory and the datapath
   <proc_type>_vector_num_elems:      the number of elements per vector
 */
#include "error.h"
#include "bits.h"
#include "memory.h"
#include "system_api.h"

#define _hrt_vector_set_bit(a,b,c) ((a)|((b)<<(c)))

#define _hrt_ceil_div(a,b) (((a)+(b)-1)/(b))
#define _hrt_extend(a, bit) (((a)<<(32-bit))>>(32-bit))
#define _hrt_sign_extend(a, bit) _hrt_extend((int)(a), (bit))
#define _hrt_zero_extend(a, bit) _hrt_extend((unsigned)(a), (bit))

static inline void _hrt_decode_vector (char *vec_in, int *vec_out, int elem_bits, int elem_precision, int num_elems, int sign_extend)
{
  int i, j, ptr = 0, pos = 0;

  for (i=0; i<num_elems; i++) {
    unsigned int value = 0;
    for (j=0; j<elem_bits; j++) {
      unsigned int b = _hrt_get_bit (vec_in[ptr], pos);
      value = _hrt_vector_set_bit (value, b, j);
      pos++;
      if (pos==8) {
        ptr++;
        pos = 0;
      }
    }
    if (sign_extend) vec_out[i] = _hrt_sign_extend(value, elem_precision);
    else             vec_out[i] = _hrt_zero_extend(value, elem_precision);
  }
}

static inline void _hrt_encode_vector (int *vec_in, char *vec_out, int elem_bits, int elem_precision, int num_elems, int sign_extend)
{
  unsigned char v = 0;
  int i, j, ptr = 0, pos = 0;

  for (i=0; i<num_elems; i++) {
    unsigned int value;
    if (sign_extend) value = _hrt_sign_extend(vec_in[i], elem_precision);
    else             value = _hrt_zero_extend(vec_in[i], elem_precision);
    if (value != (unsigned int)vec_in[i]) {
      hrt_error("_hrt_encode_vector: value %d does not fit in a vector element (elements are %d bits)", vec_in[i], elem_bits);
    }
    for (j=0; j<elem_bits; j++) {
      unsigned int b = _hrt_get_bit (value, j);
      v = _hrt_vector_set_bit (v, b, pos);
      pos++;

      if (pos==8) {
        vec_out[ptr++] = v;
        pos = 0;
        v = 0;
      }
    }
  }
  /* if we started writing a byte, but did not finish it, finish it here.
     This happens when num_elems*elem_bits is not a multiple of 8 */
  if (pos && pos != 8) vec_out[ptr] = v;
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

#endif /* _hrt_vector_h */
