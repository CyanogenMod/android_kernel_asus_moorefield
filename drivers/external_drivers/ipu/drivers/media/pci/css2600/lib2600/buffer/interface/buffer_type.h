#ifndef _BUFFER_TYPE_
#define _BUFFER_TYPE_

/* portable access to buffers in DDR */

#ifdef __VIED_CELL
typedef unsigned int buffer_address;
#else
#include "type_support.h" // workaround needed because shared_memory_access.h uses size_t
#include "vied/shared_memory_access.h"
typedef host_virtual_address_t buffer_address;
#endif

#endif
