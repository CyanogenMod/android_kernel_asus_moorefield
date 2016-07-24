#ifndef __ERROR_SUPPORT_H_INCLUDED__
#define __ERROR_SUPPORT_H_INCLUDED__

#if defined(__KERNEL__)
#include <linux/errno.h>

/* TODO: update error return mechanism to a more sophisticated method/definition */
/* For now this workaround is employed to uild in kernel space */
static int errno;

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

#define verifexit(cond,error_tag)  \
do {                               \
	if (!(cond)){              \
		errno = error_tag; \
		goto EXIT;         \
	}                          \
} while(0)

#define verifjmpexit(cond)         \
do {                               \
	if (!(cond)){              \
		goto EXIT;         \
	}                          \
} while(0)

#endif /* __ERROR_SUPPORT_H_INCLUDED__ */
