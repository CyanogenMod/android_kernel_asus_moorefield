#ifndef __BUFFER_ACCESS_H_INCLUDED__
#define __BUFFER_ACCESS_H_INCLUDED__

#include "buffer_type.h"
/* #def to keep consistent the buffer load interfaces for host and css */
#define IDM				0

void
buffer_load(buffer_address address, void *data, unsigned int size, unsigned int mm_id);

void
buffer_store(buffer_address address, const void *data, unsigned int size, unsigned int mm_id);

#endif /* __BUFFER_ACCESS_H_INCLUDED__ */
