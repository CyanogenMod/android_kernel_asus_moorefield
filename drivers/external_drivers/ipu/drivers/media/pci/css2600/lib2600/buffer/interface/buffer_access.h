#ifndef __BUFFER_ACCESS_H_INCLUDED__
#define __BUFFER_ACCESS_H_INCLUDED__

#include "buffer_type.h"

void
buffer_load(buffer_address address, void* data, unsigned int size);

void
buffer_store(buffer_address address, const void* data, unsigned int size);

#endif /* __BUFFER_ACCESS_H_INCLUDED__ */
