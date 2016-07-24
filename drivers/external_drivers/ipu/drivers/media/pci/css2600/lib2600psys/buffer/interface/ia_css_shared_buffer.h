#ifndef __IA_CSS_SHARED_BUFFER_H__
#define __IA_CSS_SHARED_BUFFER_H__
/* Shared Buffers */

/* A CSS shared buffer is a buffer in DDR that can be read and written by the CPU and CSS.
 * Both the CPU and CSS can have the buffer mapped simultaneously.
 * Access rights are not managed by this interface, this could be done by means
 * the read and write pointer of a queue, for example.
 */

#include "ia_css_buffer_address.h"

typedef struct ia_css_buffer_s*	ia_css_shared_buffer;
typedef void*			ia_css_shared_buffer_cpu_address;
typedef ia_css_buffer_address	ia_css_shared_buffer_css_address;

#endif /*__IA_CSS_SHARED_BUFFER_H__*/
