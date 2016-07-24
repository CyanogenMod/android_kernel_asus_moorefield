#ifndef _HRT_SYSTEM_H
#define _HRT_SYSTEM_H

#include "hive_types.h"

#if defined(SYSTEM) && defined(ONLY_CELL)
#include HRTSTR(SYSTEM.h)
#define CELL ONLY_CELL
#endif

#endif /* _HRT_SYSTEM_H */
