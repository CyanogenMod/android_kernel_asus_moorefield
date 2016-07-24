#ifndef _HRT_API_H
#define _HRT_API_H

/* HRT configurations for specific compilers go here */
#ifdef __HIVECC
#define _HRT_NO_ELF_SUPPORT
#define _HRT_NO_OVERRIDE_MEM_ATTRS
#ifndef HRT_HW
#define HRT_HW
#endif
#endif

#if defined(HRT_HW)
#define hrt_main main
#endif

/* some compilers do not provide inline, only __inline */
#if defined(__arm) || defined(_MSC_VER)
#define inline __inline
#endif
#define HRT_INLINE static inline

#include "version.h"
#include "defs.h"
#include "hive_types.h"
#include "cell.h"
#include "var.h"
#include "memory.h"
#include "system_memory.h"
#include "fifo.h"
#include "fifo_nb.h"
#include "syscall.h"
#include "master_port.h"
#include "stat_ctrl.h"
#include "host.h"
#include "thread.h"
#include "error.h"
#include "vector.h"
#include "vector_ldst.h"

#endif /* _HRT_API_H */
