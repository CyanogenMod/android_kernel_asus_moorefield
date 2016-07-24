#ifndef __IA_CSS_SHARED_BUFFER_CSS_H__
#define __IA_CSS_SHARED_BUFFER_CSS_H__


#include "ia_css_shared_buffer.h"

// Access to shared buffers from CSS

void
ia_css_shared_buffer_css_store(ia_css_shared_buffer_css_address addr, const void* data, unsigned int size);

void
ia_css_shared_buffer_css_load(ia_css_shared_buffer_css_address addr, void* data, unsigned int size);

#endif /*__IA_CSS_SHARED_BUFFER_CSS_H__*/
