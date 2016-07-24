#ifndef __IA_CSS_OUTPUT_BUFFER_H__
#define __IA_CSS_OUTPUT_BUFFER_H__

/* Output Buffers */
/* A CSS output buffer a buffer in DDR that can be written by CSS hardware
 * and that can be read by the host, after the buffer has been handed over
 * Examples: output frame buffer
 */

#include "ia_css_buffer_address.h"

typedef struct ia_css_buffer_s*	ia_css_output_buffer;
typedef const void*		ia_css_output_buffer_cpu_address;
typedef ia_css_buffer_address	ia_css_output_buffer_css_address;

#endif /* __IA_CSS_OUTPUT_BUFFER_H__ */

