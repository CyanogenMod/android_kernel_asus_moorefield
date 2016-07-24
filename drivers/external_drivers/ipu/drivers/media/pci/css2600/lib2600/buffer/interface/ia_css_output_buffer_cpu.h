#ifndef __IA_CSS_OUTPUT_BUFFER_CPU_H__
#define __IA_CSS_OUTPUT_BUFFER_CPU_H__

#include "ia_css_output_buffer.h"

ia_css_output_buffer			ia_css_output_buffer_alloc(unsigned int size);
void					ia_css_output_buffer_free(ia_css_output_buffer buf);

// acquire and release write access for CSS
ia_css_output_buffer_css_address	ia_css_output_buffer_css_map(ia_css_output_buffer buf);
void					ia_css_output_buffer_css_unmap(ia_css_output_buffer buf);

// acquire and release read access for CPU
ia_css_output_buffer_cpu_address	ia_css_output_buffer_cpu_map(ia_css_output_buffer buf);
void				 	ia_css_output_buffer_cpu_unmap(ia_css_output_buffer buf);

// indirect to output buffer (cpu-mapped)
void		ia_css_output_buffer_cpu_load(ia_css_output_buffer_cpu_address dst,
				 void* src, unsigned int bytes);

#endif /* __IA_CSS_OUTPUT_BUFFER_CPU_H__ */
