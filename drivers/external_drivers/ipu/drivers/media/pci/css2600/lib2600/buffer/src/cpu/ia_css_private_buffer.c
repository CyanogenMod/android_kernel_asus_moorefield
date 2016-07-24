#include "ia_css_private_buffer.h"

#include "ia_css_buffer.h"
#include "vied/shared_memory_access.h"
#include "vied/shared_memory_map.h"
#include "assert_support.h"


ia_css_private_buffer
ia_css_private_buffer_alloc(unsigned int size)
{
	return ia_css_buffer_alloc(size);
}

void
ia_css_private_buffer_free(ia_css_private_buffer b)
{
	return ia_css_buffer_free(b);
}

ia_css_private_buffer_address
ia_css_private_buffer_acquire(ia_css_private_buffer b)
{
	assert(b->state == buffer_unmapped);
	b->state = buffer_css;
	return b->css_address;
}

void
ia_css_private_buffer_release(ia_css_private_buffer b)
{
	assert(b->state == buffer_css);
	b->state = buffer_unmapped;
}
