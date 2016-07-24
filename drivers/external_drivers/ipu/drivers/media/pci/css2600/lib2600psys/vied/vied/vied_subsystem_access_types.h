#ifndef _HRT_VIED_SUBSYSTEM_ACCESS_TYPES_H
#define _HRT_VIED_SUBSYSTEM_ACCESS_TYPES_H

/** Types for the VIED subsystem access interface */
#include <type_support.h>

/** \brief An identifier for a VIED subsystem.
 *
 * This identifier must be a compile-time constant. It is used in
 * access to a VIED subsystem.
 */
typedef  unsigned int   vied_subsystem_t;


/** \brief An address within a VIED subsystem */
typedef  uint32_t    vied_subsystem_address_t;

#endif /* _HRT_VIED_SUBSYSTEM_ACCESS_TYPES_H */
