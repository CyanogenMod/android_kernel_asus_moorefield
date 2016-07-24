#ifndef __IA_CSS_PRIVATE_BUFFER_H__
#define __IA_CSS_PRIVATE_BUFFER_H__

/* A CSS private buffer is a buffer in DDR that can only be read and written by CSS hardware
   and cannot be accessed from the host.
   Example: state saving buffers, intermediate data buffers, going from one psys task on the
   subsystem to another.
   How about buffers going from ISYS to PSYS?
 */

typedef struct ia_css_buffer_s*	ia_css_private_buffer; // impl dependent
typedef unsigned int 		ia_css_private_buffer_address;

ia_css_private_buffer		ia_css_private_buffer_alloc(unsigned int size);
void  				ia_css_private_buffer_free(ia_css_private_buffer buf);

// acquire/release read and write access
ia_css_private_buffer_address	ia_css_private_buffer_acquire(ia_css_private_buffer buf);
void 				ia_css_private_buffer_release(ia_css_private_buffer);

// access: indirect access, indirect: ia_css_buffer_load and store
#endif /*__IA_CSS_BUFFER_H__*/

