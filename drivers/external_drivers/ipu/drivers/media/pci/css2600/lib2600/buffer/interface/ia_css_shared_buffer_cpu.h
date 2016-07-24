#ifndef __IA_CSS_SHARED_BUFFER_CPU_H__
#define __IA_CSS_SHARED_BUFFER_CPU_H__

#include "ia_css_shared_buffer.h"

extern ia_css_shared_buffer
ia_css_shared_buffer_alloc(unsigned int size);

extern void
ia_css_shared_buffer_free(ia_css_shared_buffer i);

extern ia_css_shared_buffer_cpu_address
ia_css_shared_buffer_cpu_map(ia_css_shared_buffer i);

extern void
ia_css_shared_buffer_cpu_unmap(ia_css_shared_buffer i);

extern ia_css_shared_buffer_css_address
ia_css_shared_buffer_css_map(ia_css_shared_buffer i);

extern void
ia_css_shared_buffer_css_unmap(ia_css_shared_buffer i);

extern void
ia_css_shared_buffer_cpu_store(ia_css_shared_buffer_cpu_address dst, const void* src, unsigned int size);

extern void
ia_css_shared_buffer_cpu_load(ia_css_shared_buffer_cpu_address src, void* dst, unsigned int size);

extern void
ia_css_shared_buffer_css_update(ia_css_shared_buffer i);

extern void
ia_css_shared_buffer_cpu_update(ia_css_shared_buffer i);

#endif /*__IA_CSS_SHARED_BUFFER_CPU_H__*/
