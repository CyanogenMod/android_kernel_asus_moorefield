#ifndef _HRT_VIED_MEMORY_ACCESS_TYPES_H
#define _HRT_VIED_MEMORY_ACCESS_TYPES_H

/** Types for the VIED memory access interface */

#include "vied_types.h"

/**
 * \brief An identifier for a system memory.
 *
 * This identifier must be a compile-time constant. It is used in
 * access to system memory.
 */
typedef unsigned int    vied_memory_t;

#ifndef __HIVECC
/**
 * \brief The type for a physical address
 */
#if  defined(C_RUN) || defined(crun_hostlib)
typedef int* vied_physical_address_t;
#else
typedef unsigned long long    vied_physical_address_t;
#endif
#endif

#endif /* _HRT_VIED_MEMORY_ACCESS_TYPES_H */
