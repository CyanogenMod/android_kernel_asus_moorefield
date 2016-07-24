// implementation of buffer access from the CPU
// using shared_memory interface

#include "buffer_access.h"
#include "vied/shared_memory_access.h"
// #include <stdio.h>

static const int mid = 0;

void
buffer_load(buffer_address address, void *data, unsigned int bytes, unsigned int mm_id)
{
	shared_memory_load(mm_id, address, data, bytes);
}

void
buffer_store(buffer_address address, const void *data, unsigned int bytes, unsigned int mm_id)
{
	shared_memory_store(mm_id, address, data, bytes);
}

