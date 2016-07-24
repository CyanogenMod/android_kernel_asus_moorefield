#ifndef __ERROR_SUPPORT_H_INCLUDED__
#define __ERROR_SUPPORT_H_INCLUDED__

#if defined(__KERNEL__)
#include <linux/errno.h>
#else
#include <errno.h>
#endif

// OS-independent definition of IA_CSS errno values
// #define IA_CSS_EINVAL 1
// #define IA_CSS_EFAULT 2



#define verifret(cond,error_type)   \
do {                                \
	if (!(cond)){                   \
		return error_type;          \
	}                               \
} while(0)

#define verifjmp(cond,error_tag)    \
do {                                \
	if (!(cond)){                   \
		goto error_tag;             \
	}                               \
} while(0)

#endif /* __ERROR_SUPPORT_H_INCLUDED__ */
