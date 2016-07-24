#ifndef _HRT_ERROR_H
#define _HRT_ERROR_H

#if defined(HRT_HW) || defined(HRT_KERNEL)
#define hrt_error(fmt, ...) {}
#define hrt_warning(fmt, ...) {}
#elif defined(HRT_HSS)
#include "master_port.h"
extern void _hrt_host_error(const char *fmt, ...);
extern void _hrt_host_warning(const char *fmt, ...);
extern void _hrt_set_source_info(const char *file, unsigned int line);
#define hrt_error(fmt, s...) \
	(_hrt_set_source_info(__FILE__, __LINE__), _hrt_host_error(fmt, ## s))
#define hrt_warning(fmt, s...) \
	(_hrt_set_source_info(__FILE__, __LINE__), _hrt_host_warning(fmt, ## s))
#else
#include <stdarg.h>
#include "defs.h"
static inline void
hrt_error(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	fprintf(stderr, "hrt: error: ");
	vfprintf(stderr, fmt, args);
	fprintf(stderr, "\n");
	va_end(args);
	exit(1);
}

static inline void
hrt_warning(const char *fmt, ...)
{
	va_list args;
	va_start(args, fmt);
	fprintf(stderr, "hrt: warning: ");
	vfprintf(stderr, fmt, args);
	fprintf(stderr, "\n");
	va_end(args);
}
#endif

#endif /* _HRT_ERROR_H */
