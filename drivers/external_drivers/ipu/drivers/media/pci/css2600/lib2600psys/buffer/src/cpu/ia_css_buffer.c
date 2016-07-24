/* provided interface */
#include "ia_css_buffer.h"

/* used interfaces */
#include "vied/shared_memory_access.h"
#include "vied/shared_memory_map.h"
#include "cpu_mem_support.h"

ia_css_buffer_t
ia_css_buffer_alloc(unsigned int size)
{
	ia_css_buffer_t b;

	b = ia_css_cpu_mem_alloc(sizeof(*b));
	if (b == NULL) {
		return NULL;
	}

	b->mem = shared_memory_alloc(mid, size);
	if (b->mem == 0) {
		ia_css_cpu_mem_free(b);
		return NULL;
	}

	b->css_address = shared_memory_map(sid, mid, b->mem);
	b->size	= size;
	return b;
}


void
ia_css_buffer_free(ia_css_buffer_t b)
{
	shared_memory_unmap(sid, mid, b->css_address);
	shared_memory_free(mid, b->mem);
	ia_css_cpu_mem_free(b);
}

