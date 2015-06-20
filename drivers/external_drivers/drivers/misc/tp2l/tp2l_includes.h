/* Include here all the tracepoint header files that
 * TP2L shall support.
 *
 * Note that most of the tracepoint header files are
 * already listed below, but some of them or commented.
 * This is done on purpose since there are some compilation
 * issues when including some tracepoint header files. This
 * is mainly due to some header file inclusion missing in the
 * tracepoint header files. This will be fixed in a second step.
 */



#ifdef TP2L_TEST
#define TP2L_INCLUDE_FILE "tp2l_test_trace.h"
#include "tp2l.h"
#endif




/* #define TP2L_INCLUDE_FILE "trace/events/9p.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/asoc.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/bcache.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/block.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/btrfs.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/compaction.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/cpufreq_interactive.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/ext3.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/ext4.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/f2fs.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/filemap.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/gfpflags.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/gfx_idle.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/gpio.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/gpu.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/host1x.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/irq.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/jbd2.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/jbd.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/kmem.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/kvm.h" */
/* #include "tp2l.h" */

/* It is not recommended to include lock.h here since
 * TP2L uses spinlock, and this would result in recursivity
 * issues.
 */
/* #define TP2L_INCLUDE_FILE "trace/events/lock.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/mce.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/migrate.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/mmc.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/module.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/napi.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/net.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/oom.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/power.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/printk.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/random.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/ras.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/rcu.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/regmap.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/regulator.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/rpm.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/sched.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/scsi.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/signal.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/skb.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/sock.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/sunrpc.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/syscalls.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "trace/events/task.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/timer.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/udp.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "trace/events/vmscan.h"
#include "tp2l.h"

/* #define TP2L_INCLUDE_FILE "trace/events/workqueue.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/writeback.h" */
/* #include "tp2l.h" */

/* #define TP2L_INCLUDE_FILE "trace/events/xen.h" */
/* #include "tp2l.h" */

#define TP2L_INCLUDE_FILE "net/wireless/trace.h"
#include "tp2l.h"

#define TP2L_INCLUDE_FILE "net/mac80211/trace.h"
#include "tp2l.h"

#ifdef CONFIG_IWLWIFI_DEVICE_TRACING
#define TP2L_INCLUDE_FILE "drivers/net/wireless/iwlwifi/iwl-devtrace.h"
#include "tp2l.h"
#endif

