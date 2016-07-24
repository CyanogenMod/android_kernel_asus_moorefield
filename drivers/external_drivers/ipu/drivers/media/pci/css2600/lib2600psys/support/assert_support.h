#ifndef __ASSERT_SUPPORT_H_INCLUDED__
#define __ASSERT_SUPPORT_H_INCLUDED__

#ifdef __KLOCWORK__
/* Klocwork does not see that assert will lead to abortion
 * as there is no good way to tell this to KW and the code
 * should not depend on assert to function (actually the assert
 * could be disabled in a release build) it was decided to
 * disable the assert for KW scans (by defining NDEBUG)
 * see also: http://www.klocwork.com/products/documentation/current/Tuning_C/C%2B%2B_analysis#Assertions
 */
#define NDEBUG
#endif /* __KLOCWORK__ */

#ifdef NDEBUG

#define assert(cnd) ((void)0)

#else

#if defined(_MSC_VER)
//#include <wdm.h>
#include <assert.h>
//#define assert(cnd) ASSERT(cnd)

#define OP___assert(cnd) assert(cnd)
#elif defined(__HIVECC)

/*
 * Enabling assert on cells has too many side effects, it should
 * by default be limited to the unsched CSIM mode, or to only
 * controller type processors. Presently there are not controls
 * in place for that
 */
/* #define OP___assert(cnd) OP___csim_assert(cnd) */
//#if defined(__SP)
//#define OP___assert(cnd) OP___csim_assert(cnd)
//#else
#define OP___assert(cnd) ((void)0)
//#endif

#elif defined(__KERNEL__) /* a.o. Android builds */
#include <linux/bug.h>

/*Workaround: removed ia_css_debug_dtrace
 *to avoid circular dependency of ia_css_debug.h
 *need to find a better solution
 */

#define assert(cnd)							\
	do {								\
		if (!(cnd)) {						\
			BUG();						\
		}							\
	} while (0)

#define OP___assert(cnd) assert(cnd)

#elif defined(__FIST__)

#include "assert.h"
#define OP___assert(cnd) assert(cnd)

#elif defined(__GNUC__)
#include "assert.h"
#define OP___assert(cnd) assert(cnd)
#else /* default is for unknown environments */
#define assert(cnd) ((void)0)
#endif

/**
 * The following macro can help to test the size of a struct at compile
 * time rather than at run-time. It does not work for all compilers; see
 * below.
 *
 * Depending on the value of 'condition', the following macro is expanded to:
 * - condition==true:
 *     an expression containing an array declaration with negative size,
 *     usually resulting in a compilation error
 * - condition==false:
 *     (void) 1; // C statement with no effect
 *
 * example:
 *  COMPILATION_ERROR_IF( sizeof(struct host_sp_queues) != SIZE_OF_HOST_SP_QUEUES_STRUCT);
 *
 * verify that the macro indeed triggers a compilation error with your compiler:
 *  COMPILATION_ERROR_IF( sizeof(struct host_sp_queues) != (sizeof(struct host_sp_queues)+1) );
 *
 * Not all compilers will trigger an error with this macro; use a search engine to search for
 * BUILD_BUG_ON to find other methods.
 */
#define COMPILATION_ERROR_IF(condition) ((void)sizeof(char[1 - 2*!!(condition)]))

#endif /* NDEBUG */

#endif /* __ASSERT_SUPPORT_H_INCLUDED__ */
