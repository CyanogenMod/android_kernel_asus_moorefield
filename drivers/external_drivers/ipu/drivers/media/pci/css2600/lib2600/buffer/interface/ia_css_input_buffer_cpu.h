#ifndef __IA_CSS_INPUT_BUFFER_CPU_H__
#define __IA_CSS_INPUT_BUFFER_CPU_H__

#include "ia_css_input_buffer.h"

// input buffer allocation
ia_css_input_buffer		ia_css_input_buffer_alloc(unsigned int size);
void				ia_css_input_buffer_free(ia_css_input_buffer buf);


// Map an input buffer to the CPU address space, and acquire write access for the CPU
ia_css_input_buffer_cpu_address	ia_css_input_buffer_cpu_map(ia_css_input_buffer buf);
// unmap input buffer from CPU address space, and release write access
void				ia_css_input_buffer_cpu_unmap(ia_css_input_buffer buf);


// Map an input buffer to CSS address space, and acquire read access for CSS
ia_css_input_buffer_css_address	ia_css_input_buffer_css_map(ia_css_input_buffer buf);
// unmap input buffer from CSS address space, and release read access
void				ia_css_input_buffer_css_unmap(ia_css_input_buffer buf);


// indirect access to input buffer by cpu (on cpu-mapped input buffer)
void	ia_css_input_buffer_cpu_store(ia_css_input_buffer_cpu_address dst,
				const void* src, unsigned int bytes);

#endif /* __IA_CSS_INPUT_BUFFER_CPU_H__ */
