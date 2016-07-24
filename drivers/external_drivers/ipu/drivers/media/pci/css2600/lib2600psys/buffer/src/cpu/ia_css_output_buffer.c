#include "ia_css_output_buffer_cpu.h"

// used interfaces
#include "ia_css_buffer.h"
#include "vied/shared_memory_access.h"
#include "vied/shared_memory_map.h"
#include "cpu_mem_support.h"
#include "assert_support.h"


ia_css_output_buffer
ia_css_output_buffer_alloc(unsigned int size)
{
	ia_css_output_buffer b;

	b = ia_css_cpu_mem_alloc(sizeof(*b));
	if (b == NULL) {
		return NULL;
	}

	b->cpu_address = ia_css_cpu_mem_alloc_page_aligned(size);
	if (b->cpu_address == NULL) {
		ia_css_cpu_mem_free(b);
		return NULL;
	}

	b->mem = shared_memory_alloc(mid, size);
	if (b->mem == 0) {
		ia_css_cpu_mem_free(b->cpu_address);
		ia_css_cpu_mem_free(b);
		return NULL;
	}

	b->css_address = shared_memory_map(sid, mid, b->mem);
	/* initialize the buffer to avoid warnings when copying */
	shared_memory_zero(mid, b->mem, size);

	b->size	= size;
	b->state = buffer_unmapped;

	return b;
}


void
ia_css_output_buffer_free(ia_css_output_buffer b)
{
	assert(b->state == buffer_unmapped);

	ia_css_cpu_mem_free(b->cpu_address);
	shared_memory_unmap(sid, mid, b->css_address);
	shared_memory_free(mid, b->mem);
	ia_css_cpu_mem_free(b);
}

///// CSS map, unmap

ia_css_output_buffer_css_address
ia_css_output_buffer_css_map(ia_css_output_buffer b)
{
	// map output buffer to CSS address space, acquire write access

	assert(b->state == buffer_unmapped);
	b->state = buffer_write;
	return (ia_css_output_buffer_css_address)b->css_address;
}

void
ia_css_output_buffer_css_unmap(ia_css_output_buffer b)
{
	// unmap output buffer from CSS address space, release write access

	assert(b->state == buffer_write);
	b->state = buffer_unmapped;
}

//// CPU map, load, unmap

ia_css_output_buffer_cpu_address
ia_css_output_buffer_cpu_map(ia_css_output_buffer b)
{
	// map output buffer to CPU address space, acquire read access

	assert(b->state == buffer_unmapped);
	b->state = buffer_read;

	// copy data from CSS address space to CPU address space
	shared_memory_load(mid, b->mem, b->cpu_address, b->size);

	return b->cpu_address;
}

void
ia_css_output_buffer_cpu_load(ia_css_output_buffer_cpu_address src, void* dst, unsigned int size)
{
	ia_css_cpu_mem_copy(dst, src, size);
}

void
ia_css_output_buffer_cpu_unmap(ia_css_output_buffer b)
{
	// unmap output buffer from CPU address space, release read access

	assert(b->state == buffer_read);
	b->state = buffer_unmapped;
}

