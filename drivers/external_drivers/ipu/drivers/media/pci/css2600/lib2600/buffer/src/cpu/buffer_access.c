// implementation of buffer access from the CPU
// using shared_memory interface

#include "buffer_access.h"
#include "vied/shared_memory_access.h"
// #include <stdio.h>

static const int mid = 0;

void
buffer_load(buffer_address address, void* data, unsigned int bytes)
{
	shared_memory_load(mid, address, data, bytes);
	// printf("Host load 0x%llx, %d\n", address, *(unsigned int*)data);
}

void
buffer_store(buffer_address address, const void* data, unsigned int bytes)
{
	shared_memory_store(mid, address, data, bytes);
	// printf("Host store 0x%llx, %d\n", address, *(unsigned int*)data);
}

