#ifndef __PRINT_SUPPORT_H_INCLUDED__
#define __PRINT_SUPPORT_H_INCLUDED__

#if defined(_MSC_VER) || defined(CRUN)
#include <stdio.h>

#define PWARN(format, arguments...)		printf("warning: ", ##arguments)
#define PRINT(format, arguments...)		printf(format, ##arguments)
#define PERROR(format, arguments...)		printf("error: " format, ##arguments)
#define PDEBUG(format, arguments...)		printf("debug: " format, ##arguments)

#elif defined(__HIVECC)
#include <hive/support.h>
/* To be revised

#define PWARN(format)
#define PRINT(format)				OP___printstring(format)
#define PERROR(variable)			OP___dump(9999, arguments)
#define PDEBUG(variable)			OP___dump(__LINE__, arguments)

*/

#elif defined(__KERNEL__)
#include <linux/kernel.h>
#include <linux/printk.h>

/* TODO : These defines ease the Broxton Android build */
/*	Bind them with appropriate tracing mechanism that can be used
 *	in kernel space like printk(), ftrace_printk() etc
 */
#define	fprintf				{}
#define	exit(var)
#define	FILE				void

#define PWARN(format, arguments...)		printk(KERN_WARNING format, ##arguments)
#define PRINT(format, arguments...)		printk(KERN_INFO format, ##arguments)
#define PERROR(format, arguments...)		printk(KERN_ERR format, ##arguments)
#define PDEBUG(format, arguments...)		printk(KERN_DEBUG format, ##arguments)

#else
#include <stdio.h>

#define PWARN(format, arguments...)		printf("warning: ", ##arguments)
#define PRINT(format, arguments...)		printf(format, ##arguments)
#define PERROR(format, arguments...)		printf("error: " format, ##arguments)
#define PDEBUG(format, arguments...)		printf("debug: " format, ##arguments)

#endif

#endif /* __PRINT_SUPPORT_H_INCLUDED__ */
