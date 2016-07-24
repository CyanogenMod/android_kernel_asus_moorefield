#ifndef _IA_CSS_TPROXY_DEBUG_H_
#define _IA_CSS_TPROXY_DEBUG_H_

#include <assert_support.h>

#ifndef __DEBUG_TPROXY__

#define IA_CSS_TPROXY_DEBUG_INFO(msg)
#define IA_CSS_TPROXY_DEBUG_DUMP(val)
#define IA_CSS_TPROXY_DEBUG_DUMP2(val1, val2)
#define IA_CSS_TPROXY_DEBUG_ASSERT(val)

#if defined(__HIVECC)
#define IA_CSS_TPROXY_DEBUG_ERROR(msg)                  \
        OP___printstring("Error:");                     \
        OP___printstring(msg);                          \
        OP___dump(__LINE__, errno)

#define IA_CSS_TPROXY_DEBUG_ASSERT_MSG(cnd, msg)        \
        if (!cnd) OP___printstring(msg)

#elif defined(__GNUC__) && !defined(__KERNEL__)
#include <stdio.h>

#define IA_CSS_TPROXY_DEBUG_ERROR(msg)                  \
        printf("Error: %s (%d)", msg, errno)

#define IA_CSS_TPROXY_DEBUG_ASSERT_MSG(cnd, msg)        \
        if (!cnd) printf("Error: %s\n", msg)

#elif defined(__KERNEL__)
#include <linux/kernel.h>
#include <linux/printk.h>

#define IA_CSS_TPROXY_DEBUG_ERROR(msg)                  \
        printk(KERN_ERR "%s (%d)", msg, errno)

#define IA_CSS_TPROXY_DEBUG_ASSERT_MSG(cnd, msg)        \
        if (!cnd) printk(KERN_ERR "%s\n", msg)

#endif /* Compiler */

#else /* __DEBUG_TPROXY__ */

/*
  Info, Error, Dump, and Dump2.
*/

#if defined(TPROXY_ENABLE_TRACING)
#include <vied_nci_tunit.h>

#define IA_CSS_TPROXY_DEBUG_INFO(msg)                   \
        vied_nci_tunit_print(                           \
                0,                                      \
                VIED_NCI_TUNIT_MSG_SEVERITY_NORMAL)

#define IA_CSS_TPROXY_DEBUG_ERROR(msg)                  \
        vied_nci_tunit_print(                           \
                0,                                      \
                VIED_NCI_TUNIT_MSG_SEVERITY_ERROR)

#define IA_CSS_TPROXY_DEBUG_DUMP(val)                   \
        vied_nci_tunit_print1i(                         \
                0,                                      \
                VIED_NCI_TUNIT_MSG_SEVERITY_NORMAL,     \
                (uint32_t)val)

#define IA_CSS_TPROXY_DEBUG_DUMP2(val1, val2)           \
        vied_nci_tunit_print2i(                         \
                0,                                      \
                VIED_NCI_TUNIT_MSG_SEVERITY_NORMAL,     \
                (uint32_t)val1,                         \
                (uint32_t)val2)

#elif defined(__HIVECC)
#define IA_CSS_TPROXY_DEBUG_INFO(msg)                   \
        OP___printstring(msg)

#define IA_CSS_TPROXY_DEBUG_ERROR(msg)                  \
        OP___printstring("Error:");                     \
        OP___printstring(msg);                          \
        OP___dump(__LINE__, errno)

#define IA_CSS_TPROXY_DEBUG_DUMP(val)                   \
        OP___dump(__LINE__, val)

#define IA_CSS_TPROXY_DEBUG_DUMP2(val1, val2)           \
        OP___dump(val1, val2)

#elif defined(__GNUC__) && !defined(__KERNEL__)
#include <stdio.h>

#define IA_CSS_TPROXY_DEBUG_INFO(msg)                   \
        printf("%s", msg)

#define IA_CSS_TPROXY_DEBUG_ERROR(msg)                  \
        printf("Error: %s (%d)", msg, errno);

#define IA_CSS_TPROXY_DEBUG_DUMP(val)                   \
        printf("%d\n", val)

#define IA_CSS_TPROXY_DEBUG_DUMP2(val1, val2)           \
        printf("%d, %d\n", (val1), (val2))

#endif /* Tracing & Compiler */

/*
  Assert, and Assert MSG.
*/

#if defined(__HIVECC)
#define IA_CSS_TPROXY_DEBUG_ASSERT(cnd)                 \
        OP___assert(cnd)

#define IA_CSS_TPROXY_DEBUG_ASSERT_MSG(cnd, msg)        \
        if (!cnd) OP___printstring(msg);                \
        OP___assert(cnd)

#elif defined(__GNUC__) && !defined(__KERNEL__)
#define IA_CSS_TPROXY_DEBUG_ASSERT(cnd)                 \
        assert(cnd)

#define IA_CSS_TPROXY_DEBUG_ASSERT_MSG(cnd, msg)        \
        if (!cnd) printf("Assert: %s", msg);            \
        assert(cnd)

#endif /* Compiler */

#endif /* __DEBUG_TPROXY__ */

#endif /*_IA_CSS_TPROXY_DEBUG_H_*/

