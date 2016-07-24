#ifndef __TYPE_SUPPORT_H_INCLUDED__
#define __TYPE_SUPPORT_H_INCLUDED__

/* Per the DLI spec, types are in "type_support.h" and
 * "platform_support.h" is for uncalssified/to be refactored
 * platform specific definitions.
 */
#define IA_CSS_UINT8_T_BITS						8
#define IA_CSS_UINT16_T_BITS					16
#define IA_CSS_UINT32_T_BITS					32
#define IA_CSS_INT32_T_BITS						32
#define IA_CSS_UINT64_T_BITS					64


#if defined(_MSC_VER)
#include <stdint.h>
//#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#if defined(_M_X64)
#define HOST_ADDRESS(x) (unsigned long long)(x)
#else
#define HOST_ADDRESS(x) (unsigned long)(x)
#endif

#elif defined(__HIVECC)
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#define HOST_ADDRESS(x) (unsigned long)(x)

#elif defined(__KERNEL__)
#include <linux/types.h>
#include <linux/limits.h>

#define CHAR_BIT (8)
#define HOST_ADDRESS(x) (unsigned long)(x)

#elif defined(__GNUC__)
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#define HOST_ADDRESS(x) (unsigned long)(x)

#else /* default is for the FIST environment */
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include <limits.h>
#define HOST_ADDRESS(x) (unsigned long)(x)

#endif

typedef void* HANDLE;

#endif /* __TYPE_SUPPORT_H_INCLUDED__ */
