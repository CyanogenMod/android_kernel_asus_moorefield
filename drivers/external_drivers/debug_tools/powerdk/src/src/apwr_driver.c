/* ***********************************************************************************************

   This file is provided under a dual BSD/GPLv2 license.  When using or 
   redistributing this file, you may do so under either license.

   GPL LICENSE SUMMARY

   Copyright(c) 2011 Intel Corporation. All rights reserved.

   This program is free software; you can redistribute it and/or modify 
   it under the terms of version 2 of the GNU General Public License as
   published by the Free Software Foundation.

   This program is distributed in the hope that it will be useful, but 
   WITHOUT ANY WARRANTY; without even the implied warranty of 
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU 
   General Public License for more details.

   You should have received a copy of the GNU General Public License 
   along with this program; if not, write to the Free Software 
   Foundation, Inc., 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
   The full GNU General Public License is included in this distribution 
   in the file called LICENSE.GPL.

   Contact Information:
   Gautam Upadhyaya <gautam.upadhyaya@intel.com>
   1906 Fox Drive, Champaign, IL - 61820, USA

   BSD LICENSE 

   Copyright(c) 2011 Intel Corporation. All rights reserved.
   All rights reserved.

   Redistribution and use in source and binary forms, with or without 
   modification, are permitted provided that the following conditions 
   are met:

   * Redistributions of source code must retain the above copyright 
   notice, this list of conditions and the following disclaimer.
   * Redistributions in binary form must reproduce the above copyright 
   notice, this list of conditions and the following disclaimer in 
   the documentation and/or other materials provided with the 
   distribution.
   * Neither the name of Intel Corporation nor the names of its 
   contributors may be used to endorse or promote products derived 
   from this software without specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS 
   "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT 
   LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR 
   A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT 
   OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, 
   SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT 
   LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, 
   DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY 
   THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT 
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE 
   OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
   ***********************************************************************************************
   */

/**
 * apwr_driver.c: Prototype kernel module to trace the following
 * events that are relevant to power:
 *	- entry into a C-state
 *	- change of processor frequency
 *	- interrupts and timers
 */

#define MOD_AUTHOR "Gautam Upadhyaya <gautam.upadhyaya@intel.com>"
#define MOD_DESC "Power driver for Piersol power tool. Adapted from Romain Cledat's codebase."

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/time.h>
#include <linux/smp.h> // For smp_call_function

#include <asm/local.h>
#include <asm/cputime.h> // For ktime
#include <asm/io.h> // For ioremap, read, and write

#include <trace/events/timer.h>
#include <trace/events/power.h>
#include <trace/events/irq.h>
#include <trace/events/sched.h>
#include <trace/events/syscalls.h>
#include <trace/events/workqueue.h>

#include <linux/hardirq.h> // for "in_interrupt"
#include <linux/interrupt.h> // for "TIMER_SOFTIRQ, HRTIMER_SOFTIRQ"

#include <linux/kallsyms.h>
#include <linux/stacktrace.h>
#include <linux/hash.h>
#include <linux/poll.h>
#include <linux/list.h>
#include <linux/cdev.h>
#include <linux/device.h>
#include <linux/cpufreq.h>
#include <linux/version.h> // for "LINUX_VERSION_CODE"
#include <asm/unistd.h> // for "__NR_execve"
#include <asm/delay.h> // for "udelay"
#include <linux/suspend.h> // for "pm_notifier"
#include <linux/pci.h>

#if DO_ANDROID
#include <asm/intel_scu_ipc.h>
#endif

#if DO_WAKELOCK_SAMPLE 
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#include <trace/events/wakelock.h> // Works for the custom kernel enabling wakelock tracepoint event
#endif
#endif

#ifndef __arm__
#include <asm/timer.h> // for "CYC2NS_SCALE_FACTOR"
#endif

#include "pw_lock_defs.h"
#include "pw_mem.h" // internally includes "pw_lock_defs.h"
#include "pw_data_structs.h"
#include "pw_output_buffer.h"
#include "pw_defines.h"

/**** CONFIGURATION ****/

typedef enum {
    NON_ATOM=0,
    MFD,
    LEX,
    CLV
} atom_arch_type_t;

#define APWR_VERSION_CODE LINUX_VERSION_CODE

static __read_mostly atom_arch_type_t pw_is_atm = NON_ATOM;
static __read_mostly bool pw_is_any_thread_set = false;
static __read_mostly bool pw_is_auto_demote_enabled = false;
static __read_mostly unsigned long pw_msr_fsb_freq_value = false;

/* Controls the amount of printks that happen. Levels are:
 *	- 0: no output save for errors and status at end
 *	- 1: single line for each hit (tracepoint, idle notifier...)
 *	- 2: more details
 *	- 3: user stack and kernel stack info
 */
static unsigned int verbosity = 0;

module_param(verbosity, uint, 0);
MODULE_PARM_DESC(verbosity, "Verbosity of output. From 0 to 3 with 3 the most verbose [default=0]");

/*
 * Controls whether we should be probing on
 * syscall enters and exits.
 * Useful for:
 * (1) Fork <-> Exec issues.
 * (2) Userspace <-> Kernelspace timer discrimination.
 */
static unsigned int probe_on_syscalls=0;
module_param(probe_on_syscalls, uint, 0);
MODULE_PARM_DESC(probe_on_syscalls, "Should we probe on syscall enters and exits? 1 ==> YES, 0 ==> NO (Default NO)");

/*
 * For measuring collection times.
 */
static unsigned long startJIFF, stopJIFF;

#define SUCCESS 0
#define ERROR 1

/*
 * Compile-time flags -- these affect
 * which parts of the driver get
 * compiled in.
 */
/*
 * Do we allow blocking reads?
 */
#define ALLOW_BLOCKING_READ 1
/*
 * Control whether the 'OUTPUT' macro is enabled.
 * Set to: "1" ==> 'OUTPUT' is enabled.
 *         "0" ==> 'OUTPUT' is disabled.
 */
#define DO_DEBUG_OUTPUT 0
/*
 * Control whether to output driver ERROR messages.
 * These are independent of the 'OUTPUT' macro
 * (which controls debug messages).
 * Set to '1' ==> Print driver error messages (to '/var/log/messages')
 *        '0' ==> Do NOT print driver error messages
 */
#define DO_PRINT_DRIVER_ERROR_MESSAGES 1
/*
 * Do we read the TSC MSR directly to determine
 * TSC (as opposed to using a kernel
 * function call -- e.g. rdtscll)?
 */
#define READ_MSR_FOR_TSC 1
/*
 * Do we support stats collection
 * for the 'PW_IOCTL_STATUS' ioctl?
 */
#define DO_IOCTL_STATS 0
/*
 * Do we check if the special 'B0' MFLD
 * microcode patch has been installed?
 * '1' ==> YES, perform the check.
 * '0' ==> NO, do NOT perform the check.
 */
#define DO_CHECK_BO_MICROCODE_PATCH 1
/*
 * Do we conduct overhead measurements?
 * '1' == > YES, conduct measurements.
 * '0' ==> NO, do NOT conduct measurements.
 */
#define DO_OVERHEAD_MEASUREMENTS 1
/*
 * Should we print some stats at the end of a collection?
 * '1' ==> YES, print stats
 * '0' ==> NO, do NOT print stats
 */
#define DO_PRINT_COLLECTION_STATS 0
/*
 * Do we keep track of IRQ # <--> DEV name mappings?
 * '1' ==> YES, cache mappings.
 * '0' ==> NO, do NOT cache mappings.
 */
#define DO_CACHE_IRQ_DEV_NAME_MAPPINGS 1
/*
 * Do we allow multiple device (names) to
 * map to the same IRQ number? Setting
 * to true makes the driver slower, if
 * more accurate.
 * '1' ==> YES, allow multi-device IRQs
 * '0' ==> NO, do NOT allow.
 */
#define DO_ALLOW_MULTI_DEV_IRQ 0
/*
 * Do we use a constant poll for wakelock names?
 * '1' ==> YES, use a constant pool.
 * '0' ==> NO, do NOT use a constant pool.
 */
#define DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES 1
/*
 * Do we use APERF, MPERF for
 * dynamic freq calculations?
 * '1' ==> YES, use APERF, MPERF
 * '0' ==> NO, use IA32_FIXED_CTR{1,2}
 */
#define USE_APERF_MPERF_FOR_DYNAMIC_FREQUENCY 0
/*
 * MSR used to toggle C-state auto demotions.
 */
#define AUTO_DEMOTE_MSR 0xe2
/*
 * Bit positions to toggle auto-demotion on NHM, ATM
 */
#define NHM_C3_AUTO_DEMOTE (1UL << 25)
#define NHM_C1_AUTO_DEMOTE (1UL << 26)
#define ATM_C6_AUTO_DEMOTE (1UL << 25)
#define AUTO_DEMOTE_FLAGS() ( pw_is_atm ? ATM_C6_AUTO_DEMOTE : (NHM_C3_AUTO_DEMOTE | NHM_C1_AUTO_DEMOTE) )
#define IS_AUTO_DEMOTE_ENABLED(msr) ( pw_is_atm ? (msr) & ATM_C6_AUTO_DEMOTE : (msr) & (NHM_C3_AUTO_DEMOTE | NHM_C1_AUTO_DEMOTE) )
/*
 * PERF_STATUS MSR addr -- bits 12:8, multiplied by the
 * bus clock freq, give the freq the H/W is currently
 * executing at.
 */
#define IA32_PERF_STATUS_MSR_ADDR 0x198
/*
 * Do we use the cpufreq notifier
 * for p-state transitions?
 * Useful on MFLD, where the default
 * TPF seems to be broken.
 */
#define DO_CPUFREQ_NOTIFIER 1
/*
 * Collect S state residency counters
 * The S state residency counters are only available 
 * for MFLD now.
 */
// #define DO_S_RESIDENCY_SAMPLE 1
#define DO_S_RESIDENCY_SAMPLE 1
/*
 * Collect S states
 * By default, S-state sample collection is intentionally 
 * disabled because it will be always S0 state because the collection
 * will always work when CPU is running.
 */
#define DO_S_STATE_SAMPLE 0
/*
 * Collect D state residency counters in south complex
 * The D state residency counters are only available 
 * for MFLD now.
 */
// #define DO_D_SC_RESIDENCY_SAMPLE 1
#define DO_D_SC_RESIDENCY_SAMPLE 1
/*
 * Collect D states in north complex
 */
// #define DO_D_NC_STATE_SAMPLE 1
#define DO_D_NC_STATE_SAMPLE 1
/*
 * Collect D states in south complex
 * By default, South complex D-state sample collection is 
 * intentionally disabled because we can obtain the same 
 * information from South complex D-state residency counters 
 * more accurately.
 */
// #define DO_D_SC_STATE_SAMPLE 1
#define DO_D_SC_STATE_SAMPLE 0
/* 
 * Use the predefined SCU IPC functions to access to SCU 
 * The intel_scu_ipc driver uses mutex lock which makes the system hanging 
 * when used in sched_switch. So, disable it for now.
 */
#define USE_PREDEFINED_SCU_IPC 0
#if USE_PREDEFINED_SCU_IPC
    #include <asm/intel_scu_ipc.h> // For intel_scu_ipc_command
#endif

/* 
 * Run the p-state sample generation in parallel for all CPUs
 * at the beginning and the end to avoid any delay 
 * due to serial execution
 */
#define DO_GENERATE_CURRENT_FREQ_IN_PARALLEL 0

/*
 * Compile-time constants and
 * other macros.
 */

#define NUM_MAP_BUCKETS_BITS 9
#define NUM_MAP_BUCKETS (1UL << NUM_MAP_BUCKETS_BITS)

// 32 locks for the hash table
#define HASH_LOCK_BITS 5
#define NUM_HASH_LOCKS (1UL << HASH_LOCK_BITS)
#define HASH_LOCK_MASK (NUM_HASH_LOCKS - 1)

#define HASH_LOCK(i) LOCK(hash_locks[(i) & HASH_LOCK_MASK])
#define HASH_UNLOCK(i) UNLOCK(hash_locks[(i) & HASH_LOCK_MASK])

#define NUM_TIMER_NODES_PER_BLOCK 20

#define TIMER_HASH_FUNC(a) hash_ptr((void *)a, NUM_MAP_BUCKETS_BITS)

/* Macro for printk based on verbosity */
#if DO_DEBUG_OUTPUT
#define OUTPUT(level, ...) do { if(unlikely(level <= verbosity)) printk(__VA_ARGS__); } while(0);
#else
#define OUTPUT(level, ...)
#endif // DO_DEBUG_OUTPUT
/*
 * Macro for driver error messages.
 */
#if DO_PRINT_DRIVER_ERROR_MESSAGES
    #define pw_pr_error(...) printk(KERN_ERR __VA_ARGS__)
#else
    #define pw_pr_error(...)
#endif

#define CPU() (smp_processor_id())
#define RAW_CPU() (raw_smp_processor_id())
#define TID() (current->pid)
#define PID() (current->tgid)
#define NAME() (current->comm)
#define PKG(c) ( cpu_data(c).phys_proc_id )
#define IT_REAL_INCR() (current->signal->it_real_incr.tv64)

#define GET_BOOL_STRING(b) ( (b) ? "TRUE" : "FALSE" )

#define BEGIN_IRQ_STATS_READ(p, c) do{		\
    p = &per_cpu(irq_stat, (c));

#define END_IRQ_STATS_READ(p, c)		\
    }while(0)

#define BEGIN_LOCAL_IRQ_STATS_READ(p) do{	\
    p = &__get_cpu_var(irq_stat);

#define END_LOCAL_IRQ_STATS_READ(p)		\
    }while(0)


/*
 * For now, we limit kernel-space backtraces to 20 entries.
 * This decision will be re-evaluated in the future.
 */
// #define MAX_BACKTRACE_LENGTH 20
#define MAX_BACKTRACE_LENGTH TRACE_LEN
/*
 * Is this a "root" timer?
 */
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
    #define IS_ROOT_TIMER(tid) ( (tid) == 0 || !is_tid_in_sys_list(tid) )
#else
    #define IS_ROOT_TIMER(tid) ( (tid) == 0 )
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT
/*
 * 64bit Compare-and-swap.
 */
#define CAS64(p, o, n) cmpxchg64((p), (o), (n)) == (o)
/*
 * Local compare-and-swap.
 */
#define LOCAL_CAS(l, o, n) local_cmpxchg((l), (o), (n)) == (o)
/*
 * Record a wakeup cause (but only if we're the first non-{TPS,TPE}
 * event to occur after a wakeup.
 * @tsc: the TSC when the event occurred
 * @type: the wakeup type: one of c_break_type_t vals
 * @value: a domain-specific value
 * @cpu: the logical CPU on which the timer was initialized; specific ONLY to wakeups caused by timers!
 * @pid: PID of the process that initialized the timer for a timer-wakeup (or -1 for other wakeup events).
 * @tid: TID of the task that initialized the timer for a timer-wakeup (or -1 for other wakeup events).
 */
#define record_wakeup_cause(tsc, type, value, cpu, pid, tid) do { \
    struct wakeup_event *wu_event = &get_cpu_var(wakeup_event_counter); \
    bool is_first_wakeup_event = CAS64(&wu_event->event_tsc, 0, (tsc)); \
    if (is_first_wakeup_event) { \
        wu_event->event_val = (value); \
        wu_event->init_cpu = (cpu); \
        wu_event->event_type = (type); \
        wu_event->event_tid = (tid); \
        wu_event->event_pid = (pid); \
    } \
    put_cpu_var(wakeup_event_counter); \
} while(0)

/*
 * For NHM etc.: Base operating frequency
 * ratio is encoded in 'PLATFORM_INFO' MSR.
 */
#define PLATFORM_INFO_MSR_ADDR 0xCE
/*
 * For MFLD -- base operating frequency
 * ratio is encoded in 'CLOCK_CR_GEYSIII_STAT'
 * MSR (internal communication with Peggy Irelan)
 */
#define CLOCK_CR_GEYSIII_STAT_MSR_ADDR 0x198 // '408 decimal'
/*
 * Standard Bus frequency. Valid for
 * NHM/WMR.
 * TODO: frequency for MFLD?
 */
#define BUS_CLOCK_FREQ_KHZ_NHM 133000 /* For NHM/WMR. SNB has 100000 */
#define BUS_CLOCK_FREQ_KHZ_MFLD 100000 /* For MFLD. SNB has 100000 */
/*
 * For core and later, Bus freq is encoded in 'MSR_FSB_FREQ'
 */
#define MSR_FSB_FREQ_ADDR 0xCD
/*
 * Try and determine the bus frequency.
 * Used ONLY if the user-program passed
 * us an invalid clock frequency.
 */
#define DEFAULT_BUS_CLOCK_FREQ_KHZ() ({u32 __tmp = (pw_is_atm) ? BUS_CLOCK_FREQ_KHZ_MFLD : BUS_CLOCK_FREQ_KHZ_NHM; __tmp;})
/*
 * MSRs required to enable CPU_CLK_UNHALTED.REF
 * counting.
 */
#define IA32_PERF_GLOBAL_CTRL_ADDR 0x38F
#define IA32_FIXED_CTR_CTL_ADDR 0x38D
/*
 * Standard APERF/MPERF addresses.
 * Required for dynamic freq
 * measurement.
 */
#define MPERF_MSR_ADDR 0xe7
#define APERF_MSR_ADDR 0xe8
/*
 * Fixed counter addresses.
 * Required for dynamic freq
 * measurement.
 */
#define IA32_FIXED_CTR1_ADDR 0x30A
#define IA32_FIXED_CTR2_ADDR 0x30B
/*
 * Bit positions for 'AnyThread' bits for the two
 * IA_32_FIXED_CTR{1,2} MSRs. Always '2 + 4*N'
 * where N == 1 => CTR1, N == 2 => CTR2
 */
#define IA32_FIXED_CTR1_ANYTHREAD_POS (1UL << 6)
#define IA32_FIXED_CTR2_ANYTHREAD_POS (1UL << 10)
#define ENABLE_FIXED_CTR_ANY_THREAD_MASK (IA32_FIXED_CTR1_ANYTHREAD_POS | IA32_FIXED_CTR2_ANYTHREAD_POS)
#define DISABLE_FIXED_CTR_ANY_THREAD_MASK ~ENABLE_FIXED_CTR_ANY_THREAD_MASK
#define IS_ANY_THREAD_SET(msr) ( (msr) & ENABLE_FIXED_CTR_ANY_THREAD_MASK )
/*
 * Toggle between APERF,MPERF and
 * IA32_FIXED_CTR{1,2} for Turbo.
 */
#if USE_APERF_MPERF_FOR_DYNAMIC_FREQUENCY
    #define CORE_CYCLES_MSR_ADDR APERF_MSR_ADDR
    #define REF_CYCLES_MSR_ADDR MPERF_MSR_ADDR
#else // !USE_APERF_MPERF_FOR_DYNAMIC_FREQUENCY
    #define CORE_CYCLES_MSR_ADDR IA32_FIXED_CTR1_ADDR
    #define REF_CYCLES_MSR_ADDR IA32_FIXED_CTR2_ADDR
#endif

/*
 * Size of each 'bucket' for a 'cpu_bitmap'
 */
#define NUM_BITS_PER_BUCKET (sizeof(unsigned long) * 8)
/*
 * Num 'buckets' for each 'cpu_bitmap' in the
 * 'irq_node' struct.
 */
#define NUM_BITMAP_BUCKETS ( (pw_max_num_cpus / NUM_BITS_PER_BUCKET) + 1 )
/*
 * 'cpu_bitmap' manipulation macros.
 */
#define IS_BIT_SET(bit,map) ( test_bit( (bit), (map) ) != 0 )
#define SET_BIT(bit,map) ( test_and_set_bit( (bit), (map) ) )
/*
 * Timer stats accessor macros.
 */
#ifdef CONFIG_TIMER_STATS
	#define TIMER_START_PID(t) ( (t)->start_pid )
	#define TIMER_START_COMM(t) ( (t)->start_comm )
#else
	#define TIMER_START_PID(t) (-1)
	#define TIMER_START_COMM(t) ( "UNKNOWN" )
#endif
/*
 * Helper macro to return time in usecs.
 */
#define CURRENT_TIME_IN_USEC() ({struct timeval tv; \
		do_gettimeofday(&tv);		\
		(unsigned long long)tv.tv_sec*1000000ULL + (unsigned long long)tv.tv_usec;})

/*
 * IPC communication definitions for coulomb counter, 
 * S and D state residencies
 */
// IPC register offsets
#define IPC_BASE_ADDRESS                0xFF11C000
#define IPC_CMD_OFFSET                  0x00000000
#define IPC_STS_OFFSET                  0x00000004
#define IPC_SPTR_OFFSET                 0x00000008
#define IPC_DPTR_OFFSET                 0x0000000C
#define IPC_WBUF_OFFSET                 0x00000080
#define IPC_RBUF_OFFSET                 0x00000090
#define IPC_MAX_ADDR                    0x100

// Write 3bytes in IPC_WBUF (2bytes for address and 1byte for value)
#define IPC_ADC_WRITE_1                 0x000300FF
// Write 2bytes in IPC_WBUF (2bytes for address) and read 1byte from IPC_RBUF
#define IPC_ADC_READ_1                  0x000210FF

// IPC commands
#define IPC_MESSAGE_MSIC                0xFF
#define IPC_MESSAGE_CC                  0xEF
#define IPC_MESSAGE_D_RESIDENCY         0xEA
#define IPC_MESSAGE_S_RESIDENCY         0xEB
 
// IPC subcommands
#define IPC_COMMAND_WRITE               0x0
#define IPC_COMMAND_READ                0x1
#define IPC_COMMAND_START_RESIDENCY     0x0
#define IPC_COMMAND_STOP_RESIDENCY      0x1
#define IPC_COMMAND_DUMP_RESIDENCY      0x2
#define IPC_COMMAND_OTHER               0x3
 
// Address for S state residency counters
#define S_RESIDENCY_BASE_ADDRESS        0xFFFF71E0
#define S_RESIDENCY_MAX_COUNTERS        0x4
// Address for D state residency counters
#define D_RESIDENCY_BASE_ADDRESS        0xFFFF7000
#define D_RESIDENCY_MAX_COUNTERS        0x78    // 40 LSS * 3 D states = 120
// Address for cumulative residency counter
#define CUMULATIVE_RESIDENCY_ADDRESS    0xFFFF71EC
#define DEV_UNDEFINED_ADDRESS           0xFF11C090

// PCI communication
#define MTX_ENABLE_PCI                      0x80000000
#define MTX_PCI_MSG_CTRL_REG                0x000000D0
#define MTX_PCI_MSG_DATA_REG                0x000000D4
#define MTX_PCI_ADDR(Bus, Device, Function, Offset) ((0x01 << 31) |(((Bus) &0xFF) <<     16)|   \ (((Device)&0x1F) << 11)|(((Function)&0x07) << 8)| (((Offset)&(~0x3)) << 0))
#define MTX_PCI_ADDR_IO                    0x00000CF8
#define MTX_PCI_DATA_IO                    0x00000CFC

// Power management base address for read/control subsystem status
#define PM_BASE_ADDRESS                    0xFF11D000
#define PM_MAX_ADDR                        0x3F

#if DO_D_NC_STATE_SAMPLE 
#define NC_APM_STS_ADDR                    0x4
#define NC_PM_SSS_ADDR                     0x30
#define NC_APM_STS_REG                     0x10047AF0
#define NC_PM_SSS_REG                      0x10047800
static unsigned long pci_apm_sts_mem_addr;
static unsigned long pci_pm_sss_mem_addr;
#endif

#if DO_S_STATE_SAMPLE || DO_D_SC_STATE_SAMPLE
// memory mapped io base address for subsystem power management
static void *mmio_pm_base = NULL;
#endif
#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
// memory mapped io base address for IPC-1
static void *mmio_ipc1_base = NULL;
// memory mapped io base address for Cumulative Residency counter
static void *mmio_cumulative_residency_base = NULL;
#endif
#if DO_S_RESIDENCY_SAMPLE 
// memory mapped io base address for S0ix Residency counters
static void *mmio_s_residency_base = NULL;
#endif
#if DO_D_SC_RESIDENCY_SAMPLE 
// memory mapped io base address for D0ix Residency counters
static void *mmio_d_residency_base = NULL;
#endif

// Lock used to prevent multiple call to SCU
#if !USE_PREDEFINED_SCU_IPC
static DEFINE_SPINLOCK(ipclock);
#endif

// Required to calculate S0i0 residency counter from non-zero S state counters
#if DO_S_RESIDENCY_SAMPLE 
static u64 startJIFF_s_residency;
static u64 startTSC_s_residency;
#endif
#if DO_S_RESIDENCY_SAMPLE 
static u64 startJIFF_d_sc_residency;
#endif

static u64 undefined_device_list = 0xFFFFFFFFFFFFFFFFULL;

// Keep track of jiffies when previously D-state sampled
static unsigned long long prev_sample_usec = 0;

// Hold bits for interesting device residency counters 
// The first LSB indicates that the residency counter for LSS0 is collected.
static u64 d_sc_mask = 0xFFFFFFFFULL;
// By default, 31 south complex LSSs are available in MFD now.
static u32 d_sc_device_num = MAX_LSS_NUM_IN_SC;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 27)
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)    smp_call_function((func),(ctx),(wait))
#else
#define SMP_CALL_FUNCTION(func,ctx,retry,wait)    smp_call_function((func),(ctx),(retry),(wait))
#endif


#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 9, 0)
    #define PW_HLIST_FOR_EACH_ENTRY(tpos, pos, head, member) hlist_for_each_entry(tpos, pos, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_SAFE(tpos, pos, n, head, member) hlist_for_each_entry_safe(tpos, pos, n, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_RCU(tpos, pos, head, member) hlist_for_each_entry_rcu(tpos, pos, head, member)
#else // >= 3.9.0
    #define PW_HLIST_FOR_EACH_ENTRY(tpos, pos, head, member) hlist_for_each_entry(tpos, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_SAFE(tpos, pos, n, head, member) hlist_for_each_entry_safe(tpos, n, head, member)
    #define PW_HLIST_FOR_EACH_ENTRY_RCU(tpos, pos, head, member) hlist_for_each_entry_rcu(tpos, head, member)
#endif


/*
 * Data structure definitions.
 */

typedef struct tnode tnode_t;
struct tnode{
    struct hlist_node list;
    unsigned long timer_addr;
    pid_t tid, pid;
    u64 tsc;
    s32 init_cpu;
    u16 is_root_timer : 1;
    u16 trace_sent : 1;
    u16 trace_len : 14;
    unsigned long *trace;
};

typedef struct hnode hnode_t;
struct hnode{
    struct hlist_head head;
};

typedef struct tblock tblock_t;
struct tblock{
    struct tnode *data;
    tblock_t *next;
};

typedef struct per_cpu_mem per_cpu_mem_t;
struct per_cpu_mem{
    tblock_t *block_list;
    hnode_t free_list_head;
};

#define GET_MEM_VARS(cpu) &per_cpu(per_cpu_mem_vars, (cpu))
#define GET_MY_MEM_VARS(cpu) &__get_cpu_var(per_cpu_mem_vars)

/*
 * For IRQ # <--> DEV NAME mappings.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

typedef struct irq_node irq_node_t;
struct irq_node{
    struct hlist_node list;
    struct rcu_head rcu;
    int irq;
    char *name;
    /*
     * We send IRQ # <-> DEV name
     * mappings to Ring-3 ONCE PER
     * CPU. We need a bitmap to let
     * us know which cpus have
     * already had this info sent.
     *
     * FOR NOW, WE ASSUME A MAX OF 64 CPUS!
     * (This assumption is enforced in
     * 'init_data_structures()')
     */
    unsigned long *cpu_bitmap;
};
#define PWR_CPU_BITMAP(node) ( (node)->cpu_bitmap )

typedef struct irq_hash_node irq_hash_node_t;
struct irq_hash_node{
    struct hlist_head head;
};
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS


#define NUM_IRQ_MAP_BITS 6
#define NUM_IRQ_MAP_BUCKETS (1UL << NUM_IRQ_MAP_BITS)
#define IRQ_MAP_HASH_MASK (NUM_IRQ_MAP_BITS - 1)
// #define IRQ_MAP_HASH_FUNC(num) (num & IRQ_MAP_HASH_MASK)
#define IRQ_MAP_HASH_FUNC(a) hash_long((u32)a, NUM_IRQ_MAP_BITS)

#define IRQ_LOCK_MASK HASH_LOCK_MASK

#define IRQ_LOCK(i) LOCK(irq_map_locks[(i) & IRQ_LOCK_MASK])
#define IRQ_UNLOCK(i) UNLOCK(irq_map_locks[(i) & IRQ_LOCK_MASK])

#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

typedef struct wlock_node wlock_node_t;
struct wlock_node{
    struct hlist_node list;
    struct rcu_head rcu;
    int constant_pool_index;
    unsigned long hash_val;
    size_t wakelock_name_len;
    char *wakelock_name;
};

typedef struct wlock_hash_node wlock_hash_node_t;
struct wlock_hash_node{
    struct hlist_head head;
};

#define NUM_WLOCK_MAP_BITS 6
#define NUM_WLOCK_MAP_BUCKETS (1UL << NUM_WLOCK_MAP_BITS)
#define WLOCK_MAP_HASH_MASK (NUM_WLOCK_MAP_BUCKETS - 1) /* Used for modulo: x % y == x & (y-1) iff y is pow-of-2 */
#define WLOCK_MAP_HASH_FUNC(n) pw_hash_string(n)

#define WLOCK_LOCK_MASK HASH_LOCK_MASK

#define WLOCK_LOCK(i) LOCK(wlock_map_locks[(i) & WLOCK_LOCK_MASK])
#define WLOCK_UNLOCK(i) UNLOCK(wlock_map_locks[(i) & WLOCK_LOCK_MASK])

#endif // DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES 

/*
 * For tracking device driver loads
 * and removals i.e. insmods and rmmods.
 */
typedef struct mod_node mod_node_t;
struct mod_node{
    struct list_head list;
    struct rcu_head rcu;
    pid_t tid, pid;
    const char *name;
};

/*
 * For syscall nodes
 */
typedef struct sys_node sys_node_t;
struct sys_node{
    struct hlist_node list;
    pid_t tid, pid;
    int ref_count, weight;
};

#define SYS_MAP_BUCKETS_BITS 9
#define NUM_SYS_MAP_BUCKETS (1UL << SYS_MAP_BUCKETS_BITS) // MUST be pow-of-2
#define SYS_MAP_LOCK_BITS 4
#define NUM_SYS_MAP_LOCKS (1UL << SYS_MAP_LOCK_BITS) // MUST be pow-of-2

#define SYS_MAP_NODES_HASH(t) hash_32(t, SYS_MAP_BUCKETS_BITS)
#define SYS_MAP_LOCK_HASH(t) ( (t) & (SYS_MAP_LOCK_BITS - 1) ) // pow-of-2 modulo

#define SYS_MAP_LOCK(index) LOCK(apwr_sys_map_locks[index])
#define SYS_MAP_UNLOCK(index) UNLOCK(apwr_sys_map_locks[index])

#define GET_SYS_HLIST(index) (apwr_sys_map + index)


/*
 * Function declarations (incomplete).
 */
bool is_sleep_syscall_i(long id) __attribute__((always_inline));
void sys_enter_helper_i(long id, pid_t tid, pid_t pid) __attribute__((always_inline));
void sys_exit_helper_i(long id, pid_t tid, pid_t pid) __attribute__((always_inline));
void sched_wakeup_helper_i(struct task_struct *task) __attribute__((always_inline));
static int pw_device_open(struct inode *inode, struct file *file);
static int pw_device_release(struct inode *inode, struct file *file);
static ssize_t pw_device_read(struct file *file, char __user * buffer, size_t length, loff_t * offset);
static long pw_device_unlocked_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param);
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
static long pw_device_compat_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param);
#endif
static long pw_unlocked_handle_ioctl_i(unsigned int ioctl_num, struct PWCollector_ioctl_arg *remote_args, unsigned long ioctl_param);
static unsigned int pw_device_poll(struct file *filp, poll_table *wait);
static int pw_device_mmap(struct file *filp, struct vm_area_struct *vma);
static int pw_register_dev(void);
static void pw_unregister_dev(void);
static int pw_read_msr_set_i(struct msr_set *msr_set, int *which_cx, u64 *cx_val);
#if DO_WAKELOCK_SAMPLE
static unsigned long pw_hash_string(const char *data);
#endif // DO_WAKELOCK_SAMPLE
static int pw_init_data_structures(void);
static void pw_destroy_data_structures(void);

/*
 * Variable declarations.
 */

/*
 * Names for SOFTIRQs.
 * These are taken from "include/linux/interrupt.h"
 */
static const char *pw_softirq_to_name[] = {"HI_SOFTIRQ", "TIMER_SOFTIRQ", "NET_TX_SOFTIRQ", "NET_RX_SOFTIRQ", "BLOCK_SOFTIRQ", "BLOCK_IOPOLL_SOFTIRQ", "TASKLET_SOFTIRQ", "SCHED_SOFTIRQ", "HRTIMER_SOFTIRQ", "RCU_SOFTIRQ"};

/*
 * For microcode PATCH version.
 * ONLY useful for MFLD!
 */
static u32 __read_mostly micro_patch_ver = 0x0;

/* 
 * Is the device open right now? Used to prevent
 * concurent access into the same device.
 */
#define DEV_IS_OPEN 0 // see if device is in use
static volatile unsigned long dev_status;

static struct hnode timer_map[NUM_MAP_BUCKETS];

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
static PWCollector_irq_mapping_t *irq_mappings_list = NULL;
static irq_hash_node_t irq_map[NUM_IRQ_MAP_BUCKETS];
static int total_num_irq_mappings = 0;
#endif //  DO_CACHE_IRQ_DEV_NAME_MAPPINGS

#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES
static wlock_hash_node_t wlock_map[NUM_WLOCK_MAP_BUCKETS];
static int total_num_wlock_mappings = 0;
#define GET_NEXT_CONSTANT_POOL_INDEX() total_num_wlock_mappings++
#endif // DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

DEFINE_PER_CPU(per_cpu_t, per_cpu_counts);

DEFINE_PER_CPU(stats_t, per_cpu_stats);

DEFINE_PER_CPU(CTRL_values_t, CTRL_data_values);

#ifdef __arm__
DEFINE_PER_CPU(u64, trace_power_prev_time) = 0;
#endif

static DEFINE_PER_CPU(per_cpu_mem_t, per_cpu_mem_vars);

static DEFINE_PER_CPU(u64, num_local_apic_timer_inters) = 0;

static DEFINE_PER_CPU(u32, pcpu_prev_req_freq) = 0;

static DEFINE_PER_CPU(struct msr_set, pw_pcpu_msr_sets);


/*
 * TPS helper -- required for overhead
 * measurements.
 */
#if DO_IOCTL_STATS
static DEFINE_PER_CPU(u64, num_inters) = 0;
#endif

/*
 * Macro to add newly allocated timer
 * nodes to individual free lists.
 */
#define LINK_FREE_TNODE_ENTRIES(nodes, size, free_head) do{		\
	int i=0;							\
	for(i=0; i<(size); ++i){					\
	    tnode_t *__node = &((nodes)[i]);				\
	    hlist_add_head(&__node->list, &((free_head)->head));	\
	}								\
    }while(0)


/*
 * Hash locks.
 */
static spinlock_t hash_locks[NUM_HASH_LOCKS];
/*
 * IRQ Map locks.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
static spinlock_t irq_map_locks[NUM_HASH_LOCKS];
#endif
/*
 * Wakelock map locks
 */
#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES
static spinlock_t wlock_map_locks[NUM_HASH_LOCKS];
#endif

/*
 * Base operating frequency -- required if
 * checking turbo frequencies.
 */
static __read_mostly u32 base_operating_freq_khz = 0x0;
/*
 * A table of available frequencies -- basically
 * the same as what's displayed in the 
 * 'scaling_available_frequencies' sysfs file.
 */
static __read_mostly u32 *apwr_available_frequencies = NULL;
/*
 * Character device file MAJOR
 * number -- we're now obtaining
 * this dynamically.
 */
static int apwr_dev_major_num = -1;
/*
 * Atomic counter used to synchronize TPS probes and
 * sched wakeups on other cores.
 */
#if DO_TPS_EPOCH_COUNTER
static atomic_t tps_epoch = ATOMIC_INIT(0);
#endif // DO_TPS_EPOCH_COUNTER

/*
 * Variables to create the character device file
 */
static dev_t apwr_dev;
static struct cdev *apwr_cdev;
static struct class *apwr_class = NULL;


#if DO_OVERHEAD_MEASUREMENTS
/*
 * Counter to count # of entries
 * in the timer hash map -- used
 * for debugging.
 */
static atomic_t num_timer_entries = ATOMIC_INIT(0);
#endif
/*
 * SORTED list of all device drivers loaded into
 * the kernel since the power driver was loaded.
 */
LIST_HEAD(apwr_mod_list);
/*
 * Spinlock to guard updates to mod list.
 */
static DEFINE_SPINLOCK(apwr_mod_list_lock);
/*
 * The sys map. Individual buckets are unordered.
 */
static struct hlist_head apwr_sys_map[NUM_SYS_MAP_BUCKETS];
/*
 * Spinlock to guard updates to sys map.
 */
static spinlock_t apwr_sys_map_locks[NUM_SYS_MAP_LOCKS];
/*
 * These are used for the 'hrtimer_start(...)'
 * hack.
 */
static u32 tick_count = 0;
static DEFINE_SPINLOCK(tick_count_lock);
static bool should_probe_on_hrtimer_start = true;

DEFINE_PER_CPU(local_t, sched_timer_found) = LOCAL_INIT(0);

static DEFINE_PER_CPU(local_t, num_samples_produced) = LOCAL_INIT(0);
static DEFINE_PER_CPU(local_t, num_samples_dropped) = LOCAL_INIT(0);
/*
 * Collection time, in seconds. Specified by the user via the 'PW_IOCTL_COLLECTION_TIME'
 * ioctl. Used ONLY to decide if we should wake up the power collector after resuming
 * from an S3 (suspend) state.
 */
unsigned long pw_collection_time_secs = 0;
/*
 * Collection time, in clock ticks. Specified by the user via the 'PW_IOCTL_COLLECTION_TIME'
 * ioctl. Used ONLY to decide if we should wake up the power collector after resuming
 * from an S3 (suspend) state.
 */
u64 pw_collection_time_ticks = 0;
/*
 * Snapshot of 'TSC' time on collection START.
 */
u64 pw_collection_start_tsc = 0;
/*
 * Suspend {START, STOP} TSC ticks.
 */
u64 pw_suspend_start_tsc = 0, pw_suspend_stop_tsc = 0;
/*
 * Suspend {START, STOP} S0i3 values.
 */
u64 pw_suspend_start_s0i3 = 0, pw_suspend_stop_s0i3 = 0;
/*
 * The power collector task. Used ONLY to decide whom to send a 'SIGINT' to.
 */
struct task_struct *pw_power_collector_task = NULL;
/*
 * Timer used to defer sending SIGINT.
 * Used ONLY if the device entered ACPI S3 (aka "Suspend-To-Ram") during the
 * collection.
 */
static struct hrtimer pw_acpi_s3_hrtimer;
/*
 * Used to record which wakeup event occured first.
 * Reset on every TPS.
 */
static DEFINE_PER_CPU_SHARED_ALIGNED(struct wakeup_event, wakeup_event_counter) = {0, 0, PW_BREAK_TYPE_U, -1, -1};
/*
 * Did the user mmap our buffers?
 */
static bool pw_did_mmap = false;


/*
 * MACRO helpers to measure function call
 * times.
 */
#if DO_OVERHEAD_MEASUREMENTS

#include "pw_overhead_measurements.h"

/*
 * For each function that you want to profile, 
 * do the following (e.g. function 'foo'):
 * **************************************************
 * DECLARE_OVERHEAD_VARS(foo);
 * **************************************************
 * This will declare the two variables required
 * to keep track of overheads incurred in 
 * calling/servicing 'foo'. Note that the name
 * that you declare here *MUST* match the function name!
 */

DECLARE_OVERHEAD_VARS(timer_init); // for the "timer_init" family of probes
DECLARE_OVERHEAD_VARS(timer_expire); // for the "timer_expire" family of probes
DECLARE_OVERHEAD_VARS(tps); // for TPS
DECLARE_OVERHEAD_VARS(tpf); // for TPF
DECLARE_OVERHEAD_VARS(timer_insert); // for "timer_insert"
DECLARE_OVERHEAD_VARS(timer_delete); // for "timer_delete"
DECLARE_OVERHEAD_VARS(exit_helper); // for "exit_helper"
DECLARE_OVERHEAD_VARS(map_find_unlocked_i); // for "map_find_i"
DECLARE_OVERHEAD_VARS(get_next_free_node_i); // for "get_next_free_node_i"
DECLARE_OVERHEAD_VARS(ti_helper); // for "ti_helper"
DECLARE_OVERHEAD_VARS(inter_common); // for "inter_common"
DECLARE_OVERHEAD_VARS(irq_insert); // for "irq_insert"
DECLARE_OVERHEAD_VARS(find_irq_node_i); // for "find_irq_node_i"
DECLARE_OVERHEAD_VARS(wlock_insert); // for "wlock_insert"
DECLARE_OVERHEAD_VARS(find_wlock_node_i); // for "find_wlock_node_i"
DECLARE_OVERHEAD_VARS(sys_enter_helper_i);
DECLARE_OVERHEAD_VARS(sys_exit_helper_i);
DECLARE_OVERHEAD_VARS(pw_produce_tps_msg);

/*
 * Macros to measure overheads
 */
#define DO_PER_CPU_OVERHEAD_FUNC(func, ...) do{		\
	u64 *__v = &__get_cpu_var(func##_elapsed_time);	\
	u64 tmp_1 = 0, tmp_2 = 0;			\
	local_inc(&__get_cpu_var(func##_num_iters));	\
	tscval(&tmp_1);					\
	{						\
	    func(__VA_ARGS__);				\
	}						\
	tscval(&tmp_2);					\
	*(__v) += (tmp_2 - tmp_1);			\
    }while(0)

#define DO_PER_CPU_OVERHEAD_FUNC_RET(ret, func, ...) do{	\
	u64 *__v = &__get_cpu_var(func##_elapsed_time);		\
	u64 tmp_1 = 0, tmp_2 = 0;				\
	local_inc(&__get_cpu_var(func##_num_iters));		\
	tscval(&tmp_1);						\
	{							\
	    ret = func(__VA_ARGS__);				\
	}							\
	tscval(&tmp_2);						\
	*(__v) += (tmp_2 - tmp_1);				\
    }while(0)


#else // DO_OVERHEAD_MEASUREMENTS

#define DO_PER_CPU_OVERHEAD(v, func, ...) func(__VA_ARGS__)
#define DO_PER_CPU_OVERHEAD_FUNC(func, ...) func(__VA_ARGS__)
#define DO_PER_CPU_OVERHEAD_FUNC_RET(ret, func, ...) ret = func(__VA_ARGS__)

#endif // DO_OVERHEAD_MEASUREMENTS

/*
 * File operations exported by the driver.
 */
struct file_operations Fops = {
    .open = &pw_device_open,
    .read = &pw_device_read,
    .poll = &pw_device_poll,
    // .ioctl = device_ioctl,
    .unlocked_ioctl = &pw_device_unlocked_ioctl,
#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
    .compat_ioctl = &pw_device_compat_ioctl,
#endif // COMPAT && x64
    .mmap = &pw_device_mmap,
    .release = &pw_device_release,
};

/*
 * Functions.
 */

/* Helper function to get TSC */
static inline void tscval(u64 *v)
{
#ifndef __arm__
#if READ_MSR_FOR_TSC
    u64 res;
    rdmsrl(0x10, res);
    *v = res;
#else
    unsigned int aux; 
    rdtscpll(*v, aux);
#endif // READ_MSR_FOR_TSC
#else
    struct timespec ts;
    ktime_get_ts(&ts);
    *v = (u64)ts.tv_sec * 1000000000ULL + (u64)ts.tv_nsec;
#endif // not def __arm__
};

/*
 * PCI io communication functions to read D states in north complex
 */
#if DO_D_NC_STATE_SAMPLE 
static int get_D_NC_states (unsigned long *states)  {
    if (states == NULL) {
        return -ERROR;
    }
    states[0] = inl(pci_apm_sts_mem_addr + NC_APM_STS_ADDR);
    states[1] = inl(pci_pm_sss_mem_addr + NC_PM_SSS_ADDR);
 
    return SUCCESS;
};
#endif


#if DO_S_STATE_SAMPLE 
static int get_S_state (u32 *state)  {
    if (mmio_pm_base != NULL && state != NULL) {
        *state = readl(mmio_pm_base + 0x4);

        return SUCCESS;
    }

    return -ERROR;
};
#endif


#if DO_D_SC_STATE_SAMPLE 
static int get_D_SC_states (u32 *states)  {
    if (mmio_pm_base != NULL && states != NULL) {
        states[0] = readl(mmio_pm_base + 0x30);
        states[1] = readl(mmio_pm_base + 0x34);
        states[2] = readl(mmio_pm_base + 0x38);
        states[3] = readl(mmio_pm_base + 0x3C);

        return SUCCESS;
    }

    return -ERROR;
};
#endif


/*
 * IPC communication functions to SCU 
 */
#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
// Send IPC command
static inline void ipc_command(u32 cmd)
{
    if (mmio_ipc1_base != NULL)
        writel(cmd, mmio_ipc1_base);
    else
        pw_pr_error("mmio_ipc1_base is NULL!\n");
};
 
// Read IPC status register
static inline u32 ipc_status(void)
{
    if (mmio_ipc1_base != NULL)
        return readl(mmio_ipc1_base + IPC_STS_OFFSET);

    pw_pr_error("mmio_ipc1_base is NULL!\n");
    return ERROR;
};
 
// Wait till scu status is busy
static inline int busy_loop(void)
{
    u32 status = 0;
    u32 count = 0;
 
    status = ipc_status();
    // Check the busy bit
    while (status & 1) {
        udelay(1); 
        status = ipc_status();
        count++;

        // SCU is still busy after 1 msec
        if (count > 1000) {
            OUTPUT(0, KERN_INFO "[APWR] IPC is busy.\n");
            return -ERROR;;
        }
    }
 
    // Check the error bit
    if ((status >> 1) & 1) {
        return -ERROR;
    }
 
    return SUCCESS;
};
#endif

#if DO_S_RESIDENCY_SAMPLE 
// Start S state residency counting
static int start_s_residency_counter(void) 
{
    int ret = 0;
 
    OUTPUT(0, KERN_INFO "[APWR] Start S0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_START_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);
 
    ipc_command((IPC_COMMAND_START_RESIDENCY << 12) | IPC_MESSAGE_S_RESIDENCY); 

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif
 
    return ret;
};

// Stop S state residency counting
static int stop_s_residency_counter(void) 
{
    int ret = 0;
 
    OUTPUT(0, KERN_INFO "[APWR] Stop S0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_STOP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);
 
    ipc_command((IPC_COMMAND_STOP_RESIDENCY << 12) | IPC_MESSAGE_S_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif
 
    return ret;
};

// Dump S state residency counting
static u64 dump_s_residency_counter(void) 
{
    int ret;
    u64 delta_usec = 0;
 
    OUTPUT(0, KERN_INFO "[APWR] Dump S0ix residency counter\n");
 
#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_S_RESIDENCY, IPC_COMMAND_DUMP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);
 
    ipc_command((IPC_COMMAND_DUMP_RESIDENCY << 12) | IPC_MESSAGE_S_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    if(ret != SUCCESS) {
        OUTPUT(0, KERN_ERR "Error: dump_s_residency_counter!\n");
    }

#if 0
    if(stop_s_residency_counter() != SUCCESS)
        pw_pr_error("Error: stop_s_residency_counter!\n");
    stopJIFF_s_residency = jiffies;
    if(start_s_residency_counter() != SUCCESS)
        pw_pr_error("Error: start_s_residency_counter!\n");
    startJIFF_s_residency = jiffies;
#endif

    delta_usec = CURRENT_TIME_IN_USEC() - startJIFF_s_residency;
 
    return delta_usec;
};
#endif

#if DO_D_SC_RESIDENCY_SAMPLE 
// Start D residency counting in south complex
static int start_d_sc_residency_counter(void) {
    int ret = 0;

    OUTPUT(0, KERN_INFO "[APWR] Start D0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_START_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_START_RESIDENCY << 12) | IPC_MESSAGE_D_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    return ret;
};

// Stop D residency counting in south complex
static int stop_d_sc_residency_counter(void) {
    int ret = 0;

    OUTPUT(0, KERN_INFO "[APWR] Stop D0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_STOP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_STOP_RESIDENCY << 12) | IPC_MESSAGE_D_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    return ret;
};

// Dump D residency counting in south complex
static u64 dump_d_sc_residency_counter(void) {
    int ret;
    u64 delta_usec = 0;

    OUTPUT(0, KERN_INFO "[APWR] Dump D0ix residency counter\n");

#if USE_PREDEFINED_SCU_IPC
    ret = intel_scu_ipc_simple_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_DUMP_RESIDENCY, NULL, 0, NULL, 0);
#else
    LOCK(ipclock);

    ipc_command((IPC_COMMAND_DUMP_RESIDENCY << 12) | IPC_MESSAGE_D_RESIDENCY);

    ret = busy_loop();
#endif

#if !USE_PREDEFINED_SCU_IPC
    UNLOCK(ipclock);
#endif

    if(ret != SUCCESS) {
        OUTPUT(0, KERN_ERR "Error: dump_d_sc_residency_counter!\n");
    }


    delta_usec = CURRENT_TIME_IN_USEC() - startJIFF_d_sc_residency;

    return delta_usec;
};
#endif


/*
 * Initialization and termination routines.
 */
static void destroy_timer_map(void)
{
    /*
     * NOP: nothing to free here -- timer nodes
     * are freed when their corresponding
     * (per-cpu) blocks are freed.
     */
};

static int init_timer_map(void)
{
    int i=0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i){
        INIT_HLIST_HEAD(&timer_map[i].head);
    }

    for (i=0; i<NUM_HASH_LOCKS; ++i) {
        spin_lock_init(&hash_locks[i]);
    }

    return SUCCESS;
};

#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

static int init_wlock_map(void)
{
    int i=0;

    for(i=0; i<NUM_WLOCK_MAP_BUCKETS; ++i){
        INIT_HLIST_HEAD(&wlock_map[i].head);
    }

    /*
     * Init locks
     */
    for(i=0; i<NUM_HASH_LOCKS; ++i){
        spin_lock_init(&wlock_map_locks[i]);
    }

    total_num_wlock_mappings = 0;

    return SUCCESS;
};

static void wlock_destroy_node(struct wlock_node *node)
{
    if(node->wakelock_name){
	pw_kfree(node->wakelock_name);
	node->wakelock_name = NULL;
    }
    pw_kfree(node);
};

static void wlock_destroy_callback(struct rcu_head *head)
{
    struct wlock_node *node = container_of(head, struct wlock_node, rcu);

    wlock_destroy_node(node);
};

static void destroy_wlock_map(void)
{
    int i=0;

    for(i=0; i<NUM_WLOCK_MAP_BUCKETS; ++i){
        struct hlist_head *head = &wlock_map[i].head;
        while(!hlist_empty(head)){
            struct wlock_node *node = hlist_entry(head->first, struct wlock_node, list);
            hlist_del(&node->list);
            wlock_destroy_callback(&node->rcu);
        }
    }
    total_num_wlock_mappings  = 0;
};

#endif // DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS

static int init_irq_map(void)
{
    int i=0;

    for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
        INIT_HLIST_HEAD(&irq_map[i].head);
    }

    /*
     * Init locks
     */
    for(i=0; i<NUM_HASH_LOCKS; ++i){
        spin_lock_init(&irq_map_locks[i]);
    }

    total_num_irq_mappings = 0;

    return SUCCESS;
};

static void irq_destroy_callback(struct rcu_head *head)
{
    struct irq_node *node = container_of(head, struct irq_node, rcu);
   
    if(node->name){
	pw_kfree(node->name);
	node->name = NULL;
    }
    if (node->cpu_bitmap) {
        pw_kfree(node->cpu_bitmap);
        node->cpu_bitmap = NULL;
    }
    pw_kfree(node);
};

static void destroy_irq_map(void)
{
    int i=0;

    for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i){
        struct hlist_head *head = &irq_map[i].head;
        while(!hlist_empty(head)){
            struct irq_node *node = hlist_entry(head->first, struct irq_node, list);
            hlist_del(&node->list);
            irq_destroy_callback(&node->rcu);
        }
    }

    if(irq_mappings_list){
        pw_kfree(irq_mappings_list);
        irq_mappings_list = NULL;
    }
};

#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS


static void free_timer_block(tblock_t *block)
{
    if(!block){
	return;
    }
    if(block->data){
	int i=0;
	for(i=0; i<NUM_TIMER_NODES_PER_BLOCK; ++i){
            /*
             * Check trace, just to be sure
             * (We shouldn't need this -- 'timer_destroy()'
             * explicitly checks and frees call trace
             * arrays).
             */
	    if(block->data[i].trace)
		pw_kfree(block->data[i].trace);
        }
	pw_kfree(block->data);
    }
    free_timer_block(block->next);
    pw_kfree(block);
    return;
};

static tblock_t *allocate_new_timer_block(struct hnode *free_head)
{
    tblock_t *block = pw_kmalloc(sizeof(tblock_t), GFP_ATOMIC);
    if(!block){
	return NULL;
    }
    block->data = pw_kmalloc(sizeof(tnode_t) * NUM_TIMER_NODES_PER_BLOCK, GFP_ATOMIC);
    if(!block->data){
	pw_kfree(block);
	return NULL;
    }
    memset(block->data, 0, sizeof(tnode_t) * NUM_TIMER_NODES_PER_BLOCK);
    if(free_head){
	LINK_FREE_TNODE_ENTRIES(block->data, NUM_TIMER_NODES_PER_BLOCK, free_head);
    }
    block->next = NULL;
    return block;
};

static void destroy_per_cpu_timer_blocks(void)
{
    int cpu = -1;

    for_each_online_cpu(cpu){
	per_cpu_mem_t *pcpu_mem = GET_MEM_VARS(cpu);
	tblock_t *blocks = pcpu_mem->block_list;
	free_timer_block(blocks);
    }
};

static int init_per_cpu_timer_blocks(void)
{
    int cpu = -1;

    for_each_online_cpu(cpu){
	per_cpu_mem_t *pcpu_mem = GET_MEM_VARS(cpu);
	struct hnode *free_head = &pcpu_mem->free_list_head;
	INIT_HLIST_HEAD(&free_head->head);
	if(!(pcpu_mem->block_list = allocate_new_timer_block(free_head))){
	    return -ERROR;
        }
    }

    return SUCCESS;
};


void free_sys_node_i(sys_node_t *node)
{
    if (!node) {
	return;
    }
    pw_kfree(node);
};

sys_node_t *alloc_new_sys_node_i(pid_t tid, pid_t pid)
{
    sys_node_t *node = pw_kmalloc(sizeof(sys_node_t), GFP_ATOMIC);
    if (!node) {
	pw_pr_error("ERROR: could NOT allocate new sys node!\n");
	return NULL;
    }
    node->tid = tid; node->pid = pid;
    node->ref_count = node->weight = 1;
    INIT_HLIST_NODE(&node->list);
    return node;
};

int destroy_sys_list(void)
{
    int size = 0, i=0;

    for (i=0; i<NUM_SYS_MAP_BUCKETS; ++i) {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(i);
	int tmp_size = 0;
	while (!hlist_empty(apwr_sys_list)) {
	    sys_node_t *node = hlist_entry(apwr_sys_list->first, struct sys_node, list);
	    hlist_del(&node->list);
	    ++tmp_size;
	    free_sys_node_i(node);
	    ++size;
	}
	if (tmp_size) {
	    OUTPUT(3, KERN_INFO "[%d] --> %d\n", i, tmp_size);
	}
    }

#if DO_PRINT_COLLECTION_STATS
    printk(KERN_INFO "SYS_LIST_SIZE = %d\n", size);
#endif

    return SUCCESS;
};

int init_sys_list(void)
{
    int i=0;

    for (i=0; i<NUM_SYS_MAP_BUCKETS; ++i) {
	INIT_HLIST_HEAD(GET_SYS_HLIST(i));
    }

    for (i=0; i<NUM_SYS_MAP_LOCKS; ++i) {
	spin_lock_init(apwr_sys_map_locks + i);
    }

    return SUCCESS;
};

void destroy_mod_list(void)
{
    int size = 0;
    while (!list_empty(&apwr_mod_list)) {
        struct mod_node *node = list_first_entry(&apwr_mod_list, struct mod_node, list);
        list_del(&node->list);
        pw_kfree(node);
        ++size;
    }
#if DO_PRINT_COLLECTION_STATS
    printk(KERN_INFO "MOD_LIST_SIZE = %d\n", size);
#endif
};

int init_mod_list(void)
{
    // NOP
    return SUCCESS;
};


static void pw_destroy_data_structures(void)
{
    destroy_timer_map();

    destroy_per_cpu_timer_blocks();

#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES
    destroy_wlock_map();
#endif // DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
    destroy_irq_map();
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS


    destroy_mod_list();
    destroy_sys_list();


    if (apwr_available_frequencies) {
        pw_kfree(apwr_available_frequencies);
        OUTPUT(3, KERN_INFO "FREED AVAILABLE FREQUENCIES!\n");
        apwr_available_frequencies = NULL;
    }

    pw_destroy_per_cpu_buffers();

    {
        /*
         * Print some stats about # samples produced and # dropped.
         */
#if DO_PRINT_COLLECTION_STATS
        printk(KERN_INFO "DEBUG: There were %llu / %llu dropped samples!\n", pw_num_samples_dropped, pw_num_samples_produced);
#endif
    }
};

static int pw_init_data_structures(void)
{
    /*
     * Find the # CPUs in this system.
     */
    // pw_max_num_cpus = num_online_cpus();
    pw_max_num_cpus = num_possible_cpus();

    /*
     * Init the (per-cpu) free lists
     * for timer mappings.
     */
    if(init_per_cpu_timer_blocks()){
        pw_pr_error("ERROR: could NOT initialize the per-cpu timer blocks!\n");
        pw_destroy_data_structures();
        return -ERROR;
    }

    if(init_timer_map()){
        pw_pr_error("ERROR: could NOT initialize timer map!\n");
        pw_destroy_data_structures();
        return -ERROR;
    }

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
    if(init_irq_map()){
        pw_pr_error("ERROR: could NOT initialize irq map!\n");
        pw_destroy_data_structures();
        return -ERROR;
    }
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS

#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES
    if (init_wlock_map()) {
        pw_pr_error("ERROR: could NOT initialize wlock map!\n");
        pw_destroy_data_structures();
        return -ERROR;
    }
#endif // DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

    if (init_mod_list()) {
        pw_pr_error("ERROR: could NOT init mod list!\n");
        pw_destroy_data_structures();
        return -ERROR;
    }

    if (init_sys_list()) {
        pw_pr_error("ERROR: could NOT initialize syscall map!\n");
        pw_destroy_data_structures();
        return -ERROR;
    }

    if (pw_init_per_cpu_buffers()) {
        pw_pr_error("ERROR initializing per-cpu output buffers\n");
        pw_destroy_data_structures();
        return -ERROR;
    }

    return SUCCESS;
};

/*
 * Free list manipulation routines.
 */

static int init_tnode_i(tnode_t *node, unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, s32 init_cpu, int trace_len, unsigned long *trace)
{

    if(node->trace){
        pw_kfree(node->trace);
        node->trace = NULL;
    }

    node->timer_addr = timer_addr; node->tsc = tsc; node->tid = tid; node->pid = pid; node->init_cpu = init_cpu; node->trace_sent = 0; node->trace_len = trace_len;

    if(trace_len >  0){
        /*
         * Root timer!
         */
        node->is_root_timer = 1;
        node->trace = pw_kmalloc(sizeof(unsigned long) * trace_len, GFP_ATOMIC);
        if(!node->trace){
            pw_pr_error("ERROR: could NOT allocate memory for backtrace!\n");
            // pw_kfree(node);
            return -ERROR;
        }
        memcpy(node->trace, trace, sizeof(unsigned long) * trace_len); // dst, src
    }

    /*
     * Ensure everyone sees this...
     */
    smp_mb();

    return SUCCESS;
};

static tnode_t *get_next_free_tnode_i(unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, s32 init_cpu, int trace_len, unsigned long *trace)
{
    per_cpu_mem_t *pcpu_mem = GET_MY_MEM_VARS();
    struct hnode *free_head = &pcpu_mem->free_list_head;
    struct hlist_head *head = &free_head->head;

    if(hlist_empty(head)){
	tblock_t *block = allocate_new_timer_block(free_head);
	if(block){
	    block->next = pcpu_mem->block_list;
	    pcpu_mem->block_list = block;
	}
	OUTPUT(3, KERN_INFO "[%d]: ALLOCATED A NEW TIMER BLOCK!\n", CPU());
    }

    if(!hlist_empty(head)){
	struct tnode *node = hlist_entry(head->first, struct tnode, list);
	hlist_del(&node->list);
	/*
	 * 'kmalloc' doesn't zero out memory -- set
	 * 'trace' to NULL to avoid an invalid
	 * 'free' in 'init_tnode_i(...)' just to
	 * be sure (Shouldn't need to have to 
	 * do this -- 'destroy_timer()' *should*
	 * have handled it for us).
	 */
	node->trace = NULL;

	if(init_tnode_i(node, timer_addr, tid, pid, tsc, init_cpu, trace_len, trace)){
	    /*
	     * Backtrace couldn't be inited -- re-enqueue
	     * onto the free-list.
	     */
	    node->trace = NULL;
	    hlist_add_head(&node->list, head);
	    return NULL;
	}
	return node;
    }
    return NULL;
};

static void timer_destroy(struct tnode *node)
{
    per_cpu_mem_t *pcpu_mem = GET_MY_MEM_VARS();
    struct hnode *free_head = &pcpu_mem->free_list_head;

    OUTPUT(3, KERN_INFO "DESTROYING %p\n", node);

    if(node->trace){
	pw_kfree(node->trace);
	node->trace = NULL;
    }

    hlist_add_head(&node->list, &((free_head)->head));
};

/*
 * Hash map routines.
 */

static tnode_t *timer_find(unsigned long timer_addr, pid_t tid)
{
    int idx = TIMER_HASH_FUNC(timer_addr);
    tnode_t *node = NULL, *retVal = NULL;
    struct hlist_node *curr = NULL;
    struct hlist_head *head = NULL;

    HASH_LOCK(idx);
    {
	head = &timer_map[idx].head;

	PW_HLIST_FOR_EACH_ENTRY(node, curr, head, list) {
	    if(node->timer_addr == timer_addr && (node->tid == tid || tid < 0)){
		retVal = node;
		break;
	    }
	}
    }
    HASH_UNLOCK(idx);

    return retVal;
};


static void timer_insert(unsigned long timer_addr, pid_t tid, pid_t pid, u64 tsc, s32 init_cpu, int trace_len, unsigned long *trace)
{
    int idx = TIMER_HASH_FUNC(timer_addr);
    struct hlist_node *curr = NULL;
    struct hlist_head *head = NULL;
    struct tnode *node = NULL, *new_node = NULL;
    bool found = false;

    HASH_LOCK(idx);
    {
        head = &timer_map[idx].head;

        PW_HLIST_FOR_EACH_ENTRY(node, curr, head, list){
            if(node->timer_addr == timer_addr){
                /*
                 * Update-in-place.
                 */
                OUTPUT(3, KERN_INFO "Timer %p UPDATING IN PLACE! Node = %p, Trace = %p\n", (void *)timer_addr, node, node->trace);
                init_tnode_i(node, timer_addr, tid, pid, tsc, init_cpu, trace_len, trace);
                found = true;
                break;
            }
        }

        if(!found){
            /*
             * Insert a new entry here.
             */
	    new_node = get_next_free_tnode_i(timer_addr, tid, pid, tsc, init_cpu, trace_len, trace);
            if(likely(new_node)){
                hlist_add_head(&new_node->list, &timer_map[idx].head);
#if DO_OVERHEAD_MEASUREMENTS
                {
                    smp_mb();
                    atomic_inc(&num_timer_entries);
                }
#endif
            }else{ // !new_node
                pw_pr_error("ERROR: could NOT allocate new timer node!\n");
            }
        }
    }
    HASH_UNLOCK(idx);

    return;
};

static int timer_delete(unsigned long timer_addr, pid_t tid)
{
    int idx = TIMER_HASH_FUNC(timer_addr);
    tnode_t *node = NULL, *found_node = NULL;
    struct hlist_node *curr = NULL, *next = NULL;
    struct hlist_head *head = NULL;
    int retVal = -ERROR;

    HASH_LOCK(idx);
    {
	head = &timer_map[idx].head;

	PW_HLIST_FOR_EACH_ENTRY_SAFE(node, curr, next, head, list){
	    // if(node->timer_addr == timer_addr && node->tid == tid){
            if(node->timer_addr == timer_addr) {
                if (node->tid != tid){
                    OUTPUT(0, KERN_INFO "WARNING: stale timer tid value? node tid = %d, task tid = %d\n", node->tid, tid);
		}
		hlist_del(&node->list);
		found_node = node;
		retVal = SUCCESS;
		OUTPUT(3, KERN_INFO "[%d]: TIMER_DELETE FOUND HRT = %p\n", tid, (void *)timer_addr);
		break;
	    }
	}
    }
    HASH_UNLOCK(idx);

    if(found_node){
	timer_destroy(found_node);
    }

    return retVal;
};

static void delete_all_non_kernel_timers(void)
{
    struct tnode *node = NULL;
    struct hlist_node *curr = NULL, *next = NULL;
    int i=0, num_timers = 0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	{
	    HASH_LOCK(i);
	    {
		PW_HLIST_FOR_EACH_ENTRY_SAFE(node, curr, next, &timer_map[i].head, list){
                    if (node->is_root_timer == 0) {
			++num_timers;
			OUTPUT(3, KERN_INFO "[%d]: Timer %p (Node %p) has TRACE = %p\n", node->tid, (void *)node->timer_addr, node, node->trace);
			hlist_del(&node->list);
			timer_destroy(node);
		    }
		}
	    }
	    HASH_UNLOCK(i);
	}
};


static void delete_timers_for_tid(pid_t tid)
{
    struct tnode *node = NULL;
    struct hlist_node *curr = NULL, *next = NULL;
    int i=0, num_timers = 0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	{
	    HASH_LOCK(i);
	    {
		PW_HLIST_FOR_EACH_ENTRY_SAFE(node, curr, next, &timer_map[i].head, list){
		    if(node->is_root_timer == 0 && node->tid == tid){
			++num_timers;
			OUTPUT(3, KERN_INFO "[%d]: Timer %p (Node %p) has TRACE = %p\n", tid, (void *)node->timer_addr, node, node->trace);
			hlist_del(&node->list);
			timer_destroy(node);
		    }
		}
	    }
	    HASH_UNLOCK(i);
	}

    OUTPUT(3, KERN_INFO "[%d]: # timers = %d\n", tid, num_timers);
};

static int get_num_timers(void)
{
    tnode_t *node = NULL;
    struct hlist_node *curr = NULL;
    int i=0, num=0;


    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	PW_HLIST_FOR_EACH_ENTRY(node, curr, &timer_map[i].head, list){
	    ++num;
	    OUTPUT(3, KERN_INFO "[%d]: %d --> %p\n", i, node->tid, (void *)node->timer_addr);
	}

    return num;
};

#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

#if DO_WAKELOCK_SAMPLE 
static unsigned long pw_hash_string(const char *data)
{
    unsigned long hash = 0;
    unsigned char c;
    char *str = (char *)data;
    while ((c = *str++)) {
        hash = c + (hash << 6) + (hash << 16) - hash;
    }
    return hash;
};

static wlock_node_t *get_next_free_wlock_node_i(unsigned long hash, size_t wlock_name_len, const char *wlock_name)
{
    wlock_node_t *node = pw_kmalloc(sizeof(wlock_node_t), GFP_ATOMIC);

    if (likely(node)) {
	memset(node, 0, sizeof(wlock_node_t));
	node->hash_val = hash;

	INIT_HLIST_NODE(&node->list);

	if( !(node->wakelock_name = pw_kstrdup(wlock_name, GFP_ATOMIC))){
	    pw_pr_error("ERROR: could NOT kstrdup wlock device name: %s\n", wlock_name);
	    pw_kfree(node);
	    node = NULL;
	} else {
            node->wakelock_name_len = wlock_name_len;
        }
    } else {
	pw_pr_error("ERROR: could NOT allocate new wlock node!\n");
    }

    return node;
};

/*
 * Check if the given wlock # <-> DEV Name mapping exists and, if
 * it does, whether this mapping was sent for the given 'cpu'
 * (We need to send each such mapping ONCE PER CPU to ensure it is
 * received BEFORE a corresponding wlock C-state wakeup).
 */
static int find_wlock_node_i(unsigned long hash, size_t wlock_name_len, const char *wlock_name)
{
    wlock_node_t *node = NULL;
    struct hlist_node *curr = NULL;
    int idx = hash & WLOCK_MAP_HASH_MASK;
    int cp_index = -1;

    rcu_read_lock();
    {
        PW_HLIST_FOR_EACH_ENTRY_RCU (node, curr, &wlock_map[idx].head, list) {
            //printk(KERN_INFO "hash_val = %lu, name = %s, cp_index = %d\n", node->hash_val, node->wakelock_name, node->constant_pool_index);
            if (node->hash_val == hash && node->wakelock_name_len == wlock_name_len && !strcmp(node->wakelock_name, wlock_name)) {
                cp_index = node->constant_pool_index;
                break;
            }
        }
    }
    rcu_read_unlock();

    return cp_index;
};

static pw_mapping_type_t wlock_insert(size_t wlock_name_len, const char *wlock_name, int *cp_index)
{
    wlock_node_t *node = NULL;
    unsigned long hash = WLOCK_MAP_HASH_FUNC(wlock_name);
    pw_mapping_type_t retVal = PW_MAPPING_ERROR;

    *cp_index = find_wlock_node_i(hash, wlock_name_len, wlock_name);

    //printk(KERN_INFO "wlock_insert: cp_index = %d, name = %s\n", *cp_index, wlock_name);

    if (*cp_index >= 0) {
        /*
         * Mapping FOUND!
         */
        //printk(KERN_INFO "OK: mapping already exists for %s (cp_index = %d)\n", wlock_name, *cp_index);
        return PW_MAPPING_EXISTS;
    }

    node = get_next_free_wlock_node_i(hash, wlock_name_len, wlock_name);

    if (unlikely(node == NULL)) {
	pw_pr_error("ERROR: could NOT allocate node for wlock insertion!\n");
	return PW_MAPPING_ERROR;
    }

    WLOCK_LOCK(hash);
    {
        int idx = hash & WLOCK_MAP_HASH_MASK;
        wlock_node_t *old_node = NULL;
        struct hlist_node *curr = NULL;
        /*
         * It is THEORETICALLY possible that the same wakelock name was passed to 'acquire' twice and that
         * a different process inserted an entry into the wakelock after our check and before we could insert
         * (i.e. a race condition). Check for that first.
         */
        PW_HLIST_FOR_EACH_ENTRY(old_node, curr, &wlock_map[idx].head, list) {
            if (old_node->hash_val == hash && old_node->wakelock_name_len == wlock_name_len && !strcmp(old_node->wakelock_name, wlock_name)) {
                *cp_index = old_node->constant_pool_index;
                //printk(KERN_INFO "wlock mapping EXISTS: cp_index = %d, name = %s\n", *cp_index, wlock_name);
                break;
            }
        }
        if (likely(*cp_index < 0)) {
            /*
             * OK: insert a new node.
             */
            *cp_index = node->constant_pool_index = GET_NEXT_CONSTANT_POOL_INDEX();
	    hlist_add_head_rcu(&node->list, &wlock_map[idx].head);
            retVal = PW_NEW_MAPPING_CREATED;
            //printk(KERN_INFO "CREATED new wlock mapping: cp_index = %d, name = %s\n", *cp_index, wlock_name);
        } else {
            /*
             * Hmnnn ... a race condition. Warn because this is very unlikely!
             */
            //printk(KERN_INFO "WARNING: race condition detected for wlock insert for node %s\n", wlock_name);
            wlock_destroy_node(node);
            retVal = PW_MAPPING_EXISTS;
        }
    }
    WLOCK_UNLOCK(hash);

    return retVal;
};
#endif // DO_WAKELOCK_SAMPLE

/*
 * INTERNAL HELPER: retrieve number of
 * mappings in the wlock mappings list.
 */
#ifndef __arm__
static int get_num_wlock_mappings(void)
{
    int retVal = 0;
    int i=0;
    wlock_node_t *node = NULL;
    struct hlist_node *curr = NULL;

    for(i=0; i<NUM_WLOCK_MAP_BUCKETS; ++i)
	PW_HLIST_FOR_EACH_ENTRY(node, curr, &wlock_map[i].head, list){
	    ++retVal;
	    OUTPUT(0, KERN_INFO "[%d]: wlock Num=%d, Dev=%s\n", i, node->wlock, node->name);
	}

    return retVal;

};
#endif
#endif // DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES

/*
 * IRQ list manipulation routines.
 */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS


static irq_node_t *get_next_free_irq_node_i(int cpu, int irq_num, const char *irq_name)
{
    irq_node_t *node = pw_kmalloc(sizeof(irq_node_t), GFP_ATOMIC);

    if(likely(node)){
	memset(node, 0, sizeof(irq_node_t));
	node->irq = irq_num;
	/*
	 * Set current CPU bitmap.
	 */
        node->cpu_bitmap = pw_kmalloc(sizeof(unsigned long) * NUM_BITMAP_BUCKETS, GFP_ATOMIC);
        if (unlikely(!node->cpu_bitmap)) {
            pw_pr_error("ERROR: could NOT allocate a bitmap for the new irq_node!\n");
            pw_kfree(node);
            return NULL;
        }
        memset(node->cpu_bitmap, 0, sizeof(unsigned long) * NUM_BITMAP_BUCKETS);
        SET_BIT(cpu, PWR_CPU_BITMAP(node));

	INIT_HLIST_NODE(&node->list);

	if( !(node->name = pw_kstrdup(irq_name, GFP_ATOMIC))){
	    pw_pr_error("ERROR: could NOT kstrdup irq device name: %s\n", irq_name);
            pw_kfree(node->cpu_bitmap);
	    pw_kfree(node);
	    node = NULL;
	}
    }else{
	pw_pr_error("ERROR: could NOT allocate new irq node!\n");
    }

    return node;

};

/*
 * Check if the given IRQ # <-> DEV Name mapping exists and, if
 * it does, whether this mapping was sent for the given 'cpu'
 * (We need to send each such mapping ONCE PER CPU to ensure it is
 * received BEFORE a corresponding IRQ C-state wakeup).
 */
static bool find_irq_node_i(int cpu, int irq_num, const char *irq_name, int *index, bool *was_mapping_sent)
{
    irq_node_t *node = NULL;
    struct hlist_node *curr = NULL;
    int idx = IRQ_MAP_HASH_FUNC(irq_num);

    *index = idx;

    rcu_read_lock();

    PW_HLIST_FOR_EACH_ENTRY_RCU(node, curr, &irq_map[idx].head, list){
	if(node->irq == irq_num
#if DO_ALLOW_MULTI_DEV_IRQ
	   && !strcmp(node->name, irq_name)
#endif // DO_ALLOW_MULTI_DEV_IRQ
	   )
	    {
		/*
		 * OK, so the maping exists. But each
		 * such mapping must be sent ONCE PER
		 * CPU to Ring-3 -- have we done so
		 * for this cpu?
		 */
		// *was_mapping_sent = (node->cpu_bitmap & (1 << cpu)) ? true : false;
                *was_mapping_sent = (IS_BIT_SET(cpu, PWR_CPU_BITMAP(node))) ? true : false;
		rcu_read_unlock();
		return true;
	    }
    }

    rcu_read_unlock();
    return false;
};

/*
 * Check to see if a given IRQ # <-> DEV Name mapping exists
 * in our list of such mappings and, if it does, whether this
 * mapping has been sent to Ring-3. Take appropriate actions
 * if any of these conditions is not met.
 */
static irq_mapping_types_t irq_insert(int cpu, int irq_num, const char *irq_name)
{
    irq_node_t *node = NULL;
    int idx = -1;
    bool found_mapping = false, mapping_sent = false;
    // int idx = IRQ_MAP_HASH_FUNC(irq_num);

    /*
     * Protocol:
     * (a) if mapping FOUND: return "OK_IRQ_MAPPING_EXISTS"
     * (b) if new mapping CREATED: return "OK_NEW_IRQ_MAPPING_CREATED"
     * (c) if ERROR: return "ERROR_IRQ_MAPPING"
     */

    found_mapping = find_irq_node_i(cpu, irq_num, irq_name, &idx, &mapping_sent);
    if(found_mapping && mapping_sent){
	/*
	 * OK, mapping exists AND we've already
	 * sent the mapping for this CPU -- nothing
	 * more to do.
	 */
	return OK_IRQ_MAPPING_EXISTS;
    }

    /*
     * Either this mapping didn't exist at all, 
     * or the mapping wasn't sent for this CPU.
     * In either case, because we're using RCU,
     * we'll have to allocate a new node.
     */

    node = get_next_free_irq_node_i(cpu, irq_num, irq_name);

    if(unlikely(node == NULL)){
	pw_pr_error("ERROR: could NOT allocate node for irq insertion!\n");
	return ERROR_IRQ_MAPPING;
    }

    IRQ_LOCK(idx);
    {
	/*
	 * It is *THEORETICALLY* possible that
	 * a different CPU added this IRQ entry
	 * to the 'irq_map'. For now, disregard
	 * the possiblility (at worst we'll have
	 * multiple entries with the same mapping,
	 * which is OK).
	 */
	bool found = false;
	irq_node_t *old_node = NULL;
	struct hlist_node *curr = NULL;
	if(found_mapping){
	    PW_HLIST_FOR_EACH_ENTRY(old_node, curr, &irq_map[idx].head, list){
		if(old_node->irq == irq_num
#if DO_ALLOW_MULTI_DEV_IRQ
		   && !strcmp(old_node->name, irq_name)
#endif // DO_ALLOW_MULTI_DEV_IRQ
		   )
		    {
			/*
			 * Found older entry -- copy the 'cpu_bitmap'
			 * field over to the new entry (no need to set this
			 * CPU's entry -- 'get_next_free_irq_node_i() has
			 * already done that. Instead, do a BITWISE OR of
			 * the old and new bitmaps)...
			 */
			OUTPUT(0, KERN_INFO "[%d]: IRQ = %d, OLD bitmap = %lu\n", cpu, irq_num, *(old_node->cpu_bitmap));
			// node->cpu_bitmap |= old_node->cpu_bitmap;
                        /*
                         * UPDATE: new 'bitmap' scheme -- copy over the older
                         * bitmap array...
                         */
                        memcpy(node->cpu_bitmap, old_node->cpu_bitmap, sizeof(unsigned long) * NUM_BITMAP_BUCKETS); // dst, src
                        /*
                         * ...then set the current CPU's pos in the 'bitmap'
                         */
                        SET_BIT(cpu, node->cpu_bitmap);
			/*
			 * ...and then replace the old node with
			 * the new one.
			 */
			hlist_replace_rcu(&old_node->list, &node->list);
			call_rcu(&old_node->rcu, &irq_destroy_callback);
			/*
			 * OK -- everything done.
			 */
			found = true;
			break;
		    }
	    }
	    if(!found){
		pw_pr_error("ERROR: CPU = %d, IRQ = %d, mapping_found but not found!\n", cpu, irq_num);
	    }
	}else{
	    hlist_add_head_rcu(&node->list, &irq_map[idx].head);
	    /*
	     * We've added a new mapping.
	     */
	    ++total_num_irq_mappings;
	}
    }
    IRQ_UNLOCK(idx);
    /*
     * Tell caller that this mapping
     * should be sent to Ring-3.
     */
    return OK_NEW_IRQ_MAPPING_CREATED;
};

/*
 * INTERNAL HELPER: retrieve number of
 * mappings in the IRQ mappings list.
 */
static int get_num_irq_mappings(void)
{
    int retVal = 0;
    int i=0;
    irq_node_t *node = NULL;
    struct hlist_node *curr = NULL;

    for(i=0; i<NUM_IRQ_MAP_BUCKETS; ++i)
	PW_HLIST_FOR_EACH_ENTRY(node, curr, &irq_map[i].head, list){
	    ++retVal;
	    OUTPUT(0, KERN_INFO "[%d]: IRQ Num=%d, Dev=%s\n", i, node->irq, node->name);
	}

    return retVal;

};

#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS

/*
 * MOD list manipulation routines.
 */

void mod_node_destroy_callback_i(struct rcu_head *head)
{
    struct mod_node *node = container_of(head, struct mod_node, rcu);
    if (node) {
        pw_kfree(node);
    }
};
/*
 * Helper function to check if a given TID corresponds
 * to that of a device driver (which inherits pid,tid of
 * the underlying 'insmod' process).
 */
bool is_tid_in_mod_list(pid_t tid)
{
    mod_node_t *node;
    bool retVal = false;
    rcu_read_lock();
    {
        list_for_each_entry_rcu(node, &apwr_mod_list, list){
            if (node->tid >= tid) {
                retVal = node->tid == tid;
                break;
            }
        }
    }
    rcu_read_unlock();
    return retVal;
};

/*
 * Helper function to add a new device driver record to our
 * list of drivers.
 */
int add_new_module_to_mod_list(pid_t tid, pid_t pid, const struct module *mod)
{
    mod_node_t *node = NULL;

    if (mod == NULL) {
        pw_pr_error("ERROR: CANNOT add NULL module to list!\n");
        return -ERROR;
    }

    node = pw_kmalloc(sizeof(mod_node_t), GFP_KERNEL);
    if (node == NULL) {
        pw_pr_error("ERROR: could NOT allocate a new mod node!\n");
        return -ERROR;
    }
    node->tid = tid; node->pid = pid;
    node->name = mod->name;

    LOCK(apwr_mod_list_lock);
    {
        struct mod_node *__tmp_node = NULL;
        bool inserted = false;
        list_for_each_entry(__tmp_node, &apwr_mod_list, list){
            if (__tmp_node->tid > tid) {
                list_add_tail_rcu(&node->list, &__tmp_node->list);
                inserted = true;
                break;
            }
        }
        if (!inserted) {
            list_add_tail_rcu(&node->list, &apwr_mod_list);
        }
    }
    UNLOCK(apwr_mod_list_lock);
    return SUCCESS;
};

/*
 * Helper function to remove a device driver record from
 * our list of device drivers.
 */
int remove_module_from_mod_list(const struct module *mod)
{
    const char *name = mod->name;
    struct mod_node *node = NULL;
    struct mod_node *tmp_node = NULL;
    bool removed = false;

    LOCK(apwr_mod_list_lock);
    {
        list_for_each_entry_safe(node, tmp_node, &apwr_mod_list, list){
            if (strcmp(node->name, name) == 0) {
                list_del_rcu(&node->list);
                // call_rcu(&node->rcu, &mod_node_destroy_callback_i);
                removed = true;
                break;
            }
        }
    }
    UNLOCK(apwr_mod_list_lock);
    if (!removed) {
        pw_pr_error("ERROR: could NOT remove module = %s from list!\n", name);
        return -ERROR;
    } else {
        synchronize_rcu();
        pw_kfree(node);
    }
    return SUCCESS;
};

/*
 * SYS map manipulation routines.
 */

inline bool is_tid_in_sys_list(pid_t tid)
{
    sys_node_t *node = NULL;
    struct hlist_node *curr = NULL;
    bool found = false;

    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        PW_HLIST_FOR_EACH_ENTRY (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid) {
		found = true;
		break;
	    }
	}
    }
    SYS_MAP_UNLOCK(lindex);

    return found;
};

inline int check_and_remove_proc_from_sys_list(pid_t tid, pid_t pid)
{
    sys_node_t *node = NULL;
    struct hlist_node *curr = NULL;
    bool found = false;
    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        PW_HLIST_FOR_EACH_ENTRY (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid && node->ref_count > 0) {
		found = true;
		--node->ref_count;
		break;
	    }
	}
    }
    SYS_MAP_UNLOCK(lindex);

    if (!found) {
	return -ERROR;
    }
    return SUCCESS;
};

inline int check_and_delete_proc_from_sys_list(pid_t tid, pid_t pid)
{
    sys_node_t *node = NULL;
    bool found = false;
    struct hlist_node *curr = NULL;
    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        PW_HLIST_FOR_EACH_ENTRY (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid) {
		found = true;
		hlist_del(&node->list);
                OUTPUT(3, KERN_INFO "CHECK_AND_DELETE: successfully deleted node: tid = %d, ref_count = %d, weight = %d\n", tid, node->ref_count, node->weight);
		free_sys_node_i(node);
		break;
	    }
	}
    }
    SYS_MAP_UNLOCK(lindex);

    if (!found) {
	return -ERROR;
    }
    return SUCCESS;
};

inline int check_and_add_proc_to_sys_list(pid_t tid, pid_t pid)
{
    sys_node_t *node = NULL;
    bool found = false;
    int retVal = SUCCESS;
    struct hlist_node *curr = NULL;
    int hindex = SYS_MAP_NODES_HASH(tid);
    int lindex = SYS_MAP_LOCK_HASH(tid);

    SYS_MAP_LOCK(lindex);
    {
	struct hlist_head *apwr_sys_list = GET_SYS_HLIST(hindex);
        PW_HLIST_FOR_EACH_ENTRY (node, curr, apwr_sys_list, list) {
	    if (node->tid == tid) {
		found = true;
		++node->ref_count;
		++node->weight;
		break;
	    }
	}
        if (!found){
	    node = alloc_new_sys_node_i(tid, pid);
	    if (!node) {
		pw_pr_error("ERROR: could NOT allocate new node!\n");
		retVal = -ERROR;
	    } else {
		hlist_add_head(&node->list, apwr_sys_list);
	    }
        }
    }
    SYS_MAP_UNLOCK(lindex);
    return retVal;
};


void print_sys_node_i(sys_node_t *node)
{
    printk(KERN_INFO "SYS_NODE: %d -> %d, %d\n", node->tid, node->ref_count, node->weight);
};


/*
 * HELPER template function to illustrate
 * how to 'produce' data into the
 * (per-cpu) output buffers.
 */
static inline void producer_template(int cpu)
{
    /*
     * Template for any of the 'produce_XXX_sample(...)'
     * functions.
     */
    struct PWCollector_msg msg;
    bool should_wakeup = true; // set to FALSE if calling from scheduling context (e.g. from "sched_wakeup()")
    msg.data_len = 0;

    // Populate 'sample' fields in a domain-specific
    // manner. e.g.:
    // sample.foo = bar
    /*
     * OK, computed 'sample' fields. Now
     * write sample into the output buffer.
     */
    pw_produce_generic_msg(&msg, should_wakeup);
};


#if DO_S_RESIDENCY_SAMPLE 
/*
 * Insert a S Residency counter sample into a (per-cpu) output buffer.
 */
static inline void produce_s_residency_sample(u64 usec)
{
    u64 tsc;
    int cpu = raw_smp_processor_id();

    PWCollector_msg_t msg;
    s_residency_sample_t sres;

    /*
     * No residency counters available  
     */
    tscval(&tsc);
    msg.data_type = S_RESIDENCY;
    msg.cpuidx = cpu;
    msg.tsc = tsc;
    msg.data_len = sizeof(sres);

    if (startTSC_s_residency == 0) {
        startTSC_s_residency = tsc;
    }

    // sres.data[0] = usec;
    sres.data[0] = tsc - startTSC_s_residency;

    if (usec) {
        int i=0;
        for (i=0; i<3; ++i) {
            sres.data[i+1] = ((u32 *)mmio_s_residency_base)[i];
        }
    } else {
        memset(&sres.data[1], 0, sizeof(u64) * 3);
    }

    sres.data[4] = sres.data[5] = 0;

    msg.p_data = (u64)((unsigned long)(&sres));

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&msg, true); // "true" ==> allow wakeups
};
#endif


#if DO_D_SC_RESIDENCY_SAMPLE 
/*
 * Insert a D Residency counter sample into a (per-cpu) output buffer.
 */
static inline void produce_d_sc_residency_sample(u64 usec)
{
    u64 tsc, msec;
    int cpu = raw_smp_processor_id();
    int i;
    int num = 0;

    PWCollector_msg_t msg;
    d_residency_msg_t dres;

    tscval(&tsc);
    msec = readl(mmio_cumulative_residency_base);

    memset(&msg, 0, sizeof(msg));
    memset(&dres, 0, sizeof(dres));

    msg.cpuidx = cpu;
    msg.tsc = tsc;
    msg.data_type = D_RESIDENCY;
    msg.data_len = sizeof(dres);

    dres.device_type = PW_SOUTH_COMPLEX;

    for (i=0; i<d_sc_device_num; i++) {
        if ((d_sc_mask >> i) & 0x1) {
            dres.mask |= (1 << i); // turn ON bit # 'i'
            dres.d_residency_counters[i].data[0] = (!usec) ? 0 : msec * 1000;
            dres.d_residency_counters[i].data[1] = (!usec) ? 0 : readl(mmio_d_residency_base + sizeof(u32) * i);
            dres.d_residency_counters[i].data[2] = (!usec) ? 0 : readl(mmio_d_residency_base + 40*4 + sizeof(u32) * i);
            dres.d_residency_counters[i].data[3] = (!usec) ? 0 : readl(mmio_d_residency_base + 40*4*2 + sizeof(u32) * i);
            ++num;
        }
    }
    dres.num_sampled = num;

    msg.p_data = (u64)((unsigned long)&dres);
    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&msg, true); // "true" ==> allow wakeups

};
#endif


#if DO_S_STATE_SAMPLE 
/*
 * Insert a S state sample into a (per-cpu) output buffer.
 */
static inline void produce_s_state_sample(void)
{
    u64 tsc;
    int cpu = CPU();
    s_state_sample_t ss;
    PWCollector_msg_t msg;

    tscval(&tsc);

    if (get_S_state(&ss.state)) {
        return;
    }

    msg.cpuidx = cpu;
    msg.tsc = tsc;

    msg.data_type = S_STATE;
    msg.data_len = sizeof(ss);
    msg.p_data = (u64)(unsigned long)&ss;
    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&msg, true); // "true" ==> allow wakeups
};
#endif


#if DO_D_NC_STATE_SAMPLE 
/*
 * Insert a north complex D state sample into a (per-cpu) output buffer.
 */
static inline int produce_d_nc_state_sample(void)
{
    u64 tsc;
    int cpu = RAW_CPU();
    // unsigned long ncstates = 0;
    unsigned long ncstates[2];
    PWCollector_msg_t sample;
    d_state_sample_t d_sample;

    tscval(&tsc);

    if (get_D_NC_states(ncstates)) {
        return -ERROR;
    }
    
    sample.cpuidx = cpu;
    sample.tsc = tsc;
    d_sample.device_type = PW_NORTH_COMPLEX;
    // d_sample.states[0] = ncstates;
    memcpy(d_sample.states, ncstates, sizeof(unsigned long) * 2);

    sample.data_type = D_STATE;
    sample.data_len = sizeof(d_sample);
    sample.p_data = (u64)((unsigned long)&d_sample);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required 

    pw_pr_debug("OK: produced NC D-state sample at TSC = %llu\n", tsc);

    return SUCCESS;
};
#endif


#if DO_D_SC_STATE_SAMPLE 
/*
 * Insert a south complex D state sample into a (per-cpu) output buffer.
 */
static inline void produce_d_sc_state_sample(void)
{
    u64 tsc;
    int cpu = raw_smp_processor_id();
    u32 scstates[4];
    PWCollector_msg_t sample;
    d_state_sample_t ds;

    tscval(&tsc);

    if (get_D_SC_states(scstates)) {
        return;
    }

    sample.cpuidx = cpu;
    sample.tsc = tsc;

    ds.device_type = PW_SOUTH_COMPLEX;
    memcpy(ds.states, scstates, sizeof(u32) * 4);

    sample.data_type = D_STATE;
    sample.data_len = sizeof(ds);
    sample.p_data = (u64)(unsigned long)&ds;

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
};
#endif


#if DO_WAKELOCK_SAMPLE 
/*
 * Insert a Wakelock sample into a (per-cpu) output buffer.
 */
static inline void produce_w_sample(int cpu, u64 tsc, w_sample_type_t type, pid_t tid, pid_t pid, const char *wlname, const char *pname, u64 timeout)
{
    PWCollector_msg_t sample;
    w_wakelock_msg_t w_msg;
    int cp_index = -1;
    size_t len = strlen(wlname);
    size_t msg_len = 0;
    constant_pool_msg_t *cp_msg = NULL;

    pw_mapping_type_t map_type = wlock_insert(len, wlname, &cp_index);

    sample.cpuidx = cpu;

    if (unlikely(map_type == PW_MAPPING_ERROR)) {
        printk(KERN_INFO "ERROR: could NOT insert wlname = %s into constant pool!\n", wlname);
        return;
    }
    /*
     * Preallocate any memory we might need, BEFORE disabling interrupts!
     */
    if (unlikely(map_type == PW_NEW_MAPPING_CREATED)) {
        msg_len = PW_CONSTANT_POOL_MSG_HEADER_SIZE + len + 1;
        cp_msg = pw_kmalloc(msg_len, GFP_ATOMIC);
        if (unlikely(cp_msg == NULL)) {
            /*
             * Hmnnnnn ... we'll need to destroy the newly created node. For now, don't handle this!!!
             * TODO: handle this case!
             */
            printk(KERN_INFO "ERROR: could NOT allocate a new node for a constant-pool mapping: WILL LEAK MEMORY and THIS MAPPING WILL BE MISSING FROM YOUR END RESULTS!");
            return;
        }
    }
    // get_cpu();
    {
        if (unlikely(map_type == PW_NEW_MAPPING_CREATED)) {
            /*
             * We've inserted a new entry into our kernel wakelock constant pool. Tell wuwatch
             * about it.
             */
            cp_msg->entry_type = W_STATE; // This is a KERNEL walock constant pool mapping
            cp_msg->entry_len = len;
            cp_msg->entry_index = cp_index;
            memcpy(cp_msg->entry, wlname, len+1);

            sample.tsc = tsc;
            sample.data_type = CONSTANT_POOL_ENTRY;
            sample.data_len = msg_len;
            sample.p_data = (u64)((unsigned long)cp_msg);

            pw_produce_generic_msg(&sample, false); // "false" ==> do NOT wakeup any sleeping readers
        }
        /*
         * OK, now send the actual wakelock sample.
         */
        w_msg.type = type;
        w_msg.expires = timeout;
        w_msg.tid = tid;
        w_msg.pid = pid;
        w_msg.constant_pool_index = cp_index;
        memcpy(w_msg.proc_name, pname, PW_MAX_PROC_NAME_SIZE); // process name

        sample.tsc = tsc;
        sample.data_type = W_STATE;
        sample.data_len = sizeof(w_msg);
        sample.p_data = (u64)(unsigned long)&w_msg;
        /*
         * OK, everything computed. Now copy
         * this sample into an output buffer
         */
        pw_produce_generic_msg(&sample, false); // "false" ==> do NOT wakeup any sleeping readers
    }
    // put_cpu();

    if (unlikely(cp_msg)) {
        //printk(KERN_INFO "OK: sent wakelock mapping: cp_index = %d, name = %s\n", cp_index, wlname);
        pw_kfree(cp_msg);
    }
    //printk(KERN_INFO "OK: sent wakelock msg for wlname = %s\n", wlname);
    return;
};
#endif


/*
 * Insert a P-state transition sample into a (per-cpu) output buffer.
 */
static inline void produce_p_sample(int cpu, unsigned long long tsc, u32 req_freq, u32 perf_status, u8 is_boundary_sample, u64 aperf, u64 mperf)
{
    struct PWCollector_msg sample;
    p_msg_t p_msg;

    sample.cpuidx = cpu;
    sample.tsc = tsc;

    p_msg.prev_req_frequency = req_freq;
    p_msg.perf_status_val = (u16)perf_status;
    p_msg.is_boundary_sample = is_boundary_sample;

    sample.data_type = P_STATE;
    sample.data_len = sizeof(p_msg);
    sample.p_data = (u64)((unsigned long)&p_msg);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
};

/*
 * Insert a K_CALL_STACK sample into a (per-cpu) output buffer.
 */
static inline void produce_k_sample(int cpu, const tnode_t *tentry)
{
    struct PWCollector_msg sample;
    k_sample_t k_sample;

    sample.cpuidx = cpu;
    sample.tsc = tentry->tsc;

    k_sample.tid = tentry->tid;
    k_sample.trace_len = tentry->trace_len;
    /*
     * Generate the "entryTSC" and "exitTSC" values here.
     */
    {
	k_sample.entry_tsc = tentry->tsc - 1;
	k_sample.exit_tsc = tentry->tsc + 1;
    }
    /*
     * Also populate the trace here!
     */
    if(tentry->trace_len){
	int num = tentry->trace_len;
	int i=0;
	u64 *trace = k_sample.trace;
	if(tentry->trace_len >= PW_TRACE_LEN){
	    OUTPUT(0, KERN_ERR "Warning: kernel trace len = %d > TRACE_LEN = %d! Will need CHAINING!\n", num, PW_TRACE_LEN);
	    num = PW_TRACE_LEN;
	}
	/*
	 * Can't 'memcpy()' -- individual entries in
	 * the 'k_sample_t->trace[]' array are ALWAYS
	 * 64 bits wide, REGARDLESS OF THE UNDERLYING
	 * ARCHITECTURE!
	 */
	for(i=0; i<num; ++i){
	    trace[i] = tentry->trace[i];
	}
    }
    OUTPUT(3, KERN_INFO "KERNEL-SPACE mapping!\n");

    sample.data_type = K_CALL_STACK;
    sample.data_len = sizeof(k_sample);
    sample.p_data = (u64)((unsigned long)&k_sample);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
};

/*
 * Insert an IRQ_MAP sample into a (per-cpu) output buffer.
 */
static inline void produce_i_sample(int cpu, int num, const char *name)
{
    struct PWCollector_msg sample;
    i_sample_t i_sample;
    u64 tsc;

    tscval(&tsc);

    sample.cpuidx = cpu;
    sample.tsc = tsc;

    i_sample.irq_num = num;
    memcpy(i_sample.irq_name, name, PW_IRQ_DEV_NAME_LEN); // dst, src

    sample.data_type = IRQ_MAP;
    sample.data_len = sizeof(i_sample);
    sample.p_data = (u64)((unsigned long)&i_sample);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
};

/*
 * Insert a PROC_MAP sample into a (per-cpu) output buffer.
 */
static inline void produce_r_sample(int cpu, u64 tsc, r_sample_type_t type, pid_t tid, pid_t pid, const char *name)
{
    struct PWCollector_msg sample;
    r_sample_t r_sample;

    sample.cpuidx = cpu;
    sample.tsc = tsc;

    r_sample.type = type;
    r_sample.tid = tid;
    r_sample.pid = pid;
    memcpy(r_sample.proc_name, name, PW_MAX_PROC_NAME_SIZE); // dst, src

    sample.data_type = PROC_MAP;
    sample.data_len = sizeof(r_sample);
    sample.p_data = (u64)((unsigned long)&r_sample);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
};

/*
 * Insert an M_MAP sample into a (per-cpu) output buffer.
 */
static inline void produce_m_sample(int cpu, const char *name, unsigned long long begin, unsigned long long sz)
{
    struct PWCollector_msg sample;
    m_sample_t m_sample;
    u64 tsc;

    tscval(&tsc);

    sample.cpuidx = cpu;
    sample.tsc = tsc;

    m_sample.start = begin;
    m_sample.end = (begin+sz);
    m_sample.offset = 0;
    memcpy(m_sample.name, name, PW_MODULE_NAME_LEN); // dst, src

    sample.data_type = M_MAP;
    sample.data_len = sizeof(m_sample);
    sample.p_data = (u64)((unsigned long)&m_sample);

    /*
     * OK, everything computed. Now copy
     * this sample into an output buffer
     */
    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required

};


/*
 * Probe functions (and helpers).
 */


/*
 * Generic method to generate a kernel-space call stack.
 * Utilizes the (provided) "save_stack_trace()" function.
 */
int __get_kernel_timerstack(unsigned long buffer[], int len)
{
    struct stack_trace strace;

    strace.max_entries = len; // MAX_BACKTRACE_LENGTH;
    strace.nr_entries = 0;
    strace.entries = buffer;
    strace.skip = 3;

    save_stack_trace(&strace);

    OUTPUT(0, KERN_INFO "[%d]: KERNEL TRACE: nr_entries = %d\n", TID(), strace.nr_entries);

    return strace.nr_entries;
};

/*
 * Generate a kernel-space call stack.
 * Requires the kernel be compiler with frame pointers ON.
 *
 * Returns number of return addresses in the call stack
 * or ZERO, to indicate no stack.
 */
int get_kernel_timerstack(unsigned long buffer[], int len)
{
    return __get_kernel_timerstack(buffer, len);
};


static void timer_init(void *timer_addr)
{
    pid_t tid = TID();
    pid_t pid = PID();
    u64 tsc = 0;
    int trace_len = 0;
    unsigned long trace[MAX_BACKTRACE_LENGTH];
    bool is_root_timer = false;
    s32 init_cpu = RAW_CPU();

    tscval(&tsc);

    /*
     * For accuracy, we ALWAYS collect
     * kernel call stacks.
     */
    if ( (is_root_timer = IS_ROOT_TIMER(tid)) ) {
	/*
	 * get kernel timerstack here.
	 * Requires the kernel be compiled with
	 * frame_pointers on.
	 */
	if (INTERNAL_STATE.have_kernel_frame_pointers) {
	    trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	}
	else{
	    trace_len = 0;
	}
	OUTPUT(0, KERN_INFO "KERNEL-SPACE timer init! Timer_addr = %p, tid = %d, pid = %d\n", timer_addr, tid, pid);
    } else {
        trace_len = 0;
    }
    /*
     * Store the timer if:
     * (a) called for a ROOT process (tid == 0) OR
     * (b) we're actively COLLECTING.
     */
    if (is_root_timer || IS_COLLECTING()) {
	DO_PER_CPU_OVERHEAD_FUNC(timer_insert, (unsigned long)timer_addr, tid, pid, tsc, init_cpu, trace_len, trace);
    }
};

// #if (KERNEL_VER < 35)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_init(struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#else
    static void probe_hrtimer_init(void *ignore, struct hrtimer *timer, clockid_t clockid, enum hrtimer_mode mode)
#endif
{
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_timer_init(struct timer_list *timer)
#else
    static void probe_timer_init(void *ignore, struct timer_list *timer)
#endif
{
    /*
     * Debugging ONLY!
     */
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};

/*
 * Interval timer state probe.
 * Fired on interval timer initializations
 * (from "setitimer(...)")
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_itimer_state(int which, const struct itimerval *const value, cputime_t expires)
#else
    static void probe_itimer_state(void *ignore, int which, const struct itimerval *const value, cputime_t expires)
#endif
{
    struct hrtimer *timer = &current->signal->real_timer;

    OUTPUT(3, KERN_INFO "[%d]: ITIMER STATE: timer = %p\n", TID(), timer);
    DO_PER_CPU_OVERHEAD_FUNC(timer_init, timer);
};


#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_start(struct hrtimer *hrt)
#else
    static void probe_hrtimer_start(void *ignore, struct hrtimer *hrt)
#endif
{
    int cpu = CPU();
    pid_t tid = TID();
    pid_t pid = PID();
    u64 tsc = 0;
    /* const char *name = TIMER_START_COMM(hrt); */
    int i, trace_len;
    char symname[KSYM_NAME_LEN];
    unsigned long trace[MAX_BACKTRACE_LENGTH];
    void *sched_timer_addr = NULL;
    per_cpu_t *pcpu = NULL;
    bool should_unregister = false;

    if(!should_probe_on_hrtimer_start){
	OUTPUT(3, KERN_INFO "HRTIMER_START: timer = %p\n", hrt);
	return;
    }

    /*
     * Not sure if "save_stack_trace" or "sprint_symbol" can
     * sleep. To be safe, use the "__get_cpu_var" variants
     * here. Note that it's OK if they give us stale values -- we're
     * not looking for an exact match.
     */
    if(tid || local_read(&__get_cpu_var(sched_timer_found)))
	return;

    /*
     * Basic algo: generate a backtrace for this hrtimer_start
     * tracepoint. Then generate symbolic information for each 
     * entry in the backtrace array. Check these symbols.
     * If any one of these symbols is equal to "cpu_idle" then
     * we know that this timer is the "tick" timer for this
     * CPU -- store the address (and the backtrace) in
     * the trace map (and also note that we have, in fact, found
     * the tick timer so that we don't repeat this process again).
     */

    if(INTERNAL_STATE.have_kernel_frame_pointers){
	trace_len = get_kernel_timerstack(trace, MAX_BACKTRACE_LENGTH);
	OUTPUT(0, KERN_INFO "[%d]: %.20s TIMER_START for timer = %p. trace_len = %d\n", tid, TIMER_START_COMM(hrt), hrt, trace_len);
	for(i=0; i<trace_len; ++i){
	    sprint_symbol(symname, trace[i]);
	    OUTPUT(3, KERN_INFO "SYM MAPPING: 0x%lx --> %s\n", trace[i], symname);
	    if(strstr(symname, "cpu_idle")){
		OUTPUT(0, KERN_INFO "FOUND CPU IDLE for cpu = %d . TICK SCHED TIMER = %p\n", cpu, hrt);
		local_inc(&__get_cpu_var(sched_timer_found));
		// *timer_found = true;
		sched_timer_addr = hrt;
	    }
	}
    }else{
	OUTPUT(0, KERN_INFO "NO TIMER STACKS!\n");
    }

    if(sched_timer_addr){
	/*
	 * OK, use the safer "get_cpu_var(...)" variants
	 * here. These disable interrupts.
	 */
	pcpu = &get_cpu_var(per_cpu_counts);
	{
	    cpu = CPU();
	    /*
	     * Races should *NOT* happen. Still, check
	     * to make sure.
	     */
	    if(!pcpu->sched_timer_addr){
		pcpu->sched_timer_addr = sched_timer_addr;

		tsc = 0x1 + cpu;

		timer_insert((unsigned long)sched_timer_addr, tid, pid, tsc, cpu, trace_len, trace);
		/*
		 * Debugging
		 */
		if(!timer_find((unsigned long)sched_timer_addr, tid)){
		    pw_pr_error("ERROR: could NOT find timer %p in hrtimer_start!\n", sched_timer_addr);
		}
	    }
	}
	put_cpu_var(pcpu);

	LOCK(tick_count_lock);
	{
	    if( (should_unregister = (++tick_count == pw_max_num_cpus))){
		OUTPUT(0, KERN_INFO "[%d]: ALL TICK TIMERS accounted for -- removing hrtimer start probe!\n", cpu);
		should_probe_on_hrtimer_start = false;
	    }
	}
	UNLOCK(tick_count_lock);
    }
};

/*
 * Common function to perform some bookkeeping on
 * IRQ-related wakeups (including (HR)TIMER_SOFTIRQs).
 * Records hits and (if necessary) sends i-sample
 * messages to Ring 3.
 */
static void handle_irq_wakeup_i(int cpu, int irq_num, const char *irq_name, bool was_hit)
{
    /*
     * Send a sample to Ring-3
     * (but only if collecting).
     */
    if (IS_COLLECTING()) {
        u64 sample_tsc;
        tscval(&sample_tsc);
        record_wakeup_cause(sample_tsc, PW_BREAK_TYPE_I, irq_num, -1, -1, -1);
    }
    /*
     * Then send an i-sample instance
     * to Ring 3 (but only if so configured
     * and if this is first time this
     * particular IRQ was seen on the
     * current CPU).
     */
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
    {
        int __ret = -1;
        /*
         * We only cache device names if they
         * actually caused a C-state
         * wakeup.
         */
        if(was_hit){
            DO_PER_CPU_OVERHEAD_FUNC_RET(__ret, irq_insert, cpu, irq_num, irq_name);
            /*
             * Protocol:
             * (a) if mapping FOUND (and already SENT for THIS CPU): irq_insert returns "OK_IRQ_MAPPING_EXISTS"
             * (b) if new mapping CREATED (or mapping exists, but NOT SENT for THIS CPU): irq_insert returns "OK_NEW_IRQ_MAPPING_CREATED"
             * (c) if ERROR: irq_insert returns "ERROR_IRQ_MAPPING"
             */
            if(__ret == OK_NEW_IRQ_MAPPING_CREATED && IS_COLLECTING()) {
                /*
                 * Send mapping info to Ring-3.
                 */
                produce_i_sample(cpu, irq_num, irq_name);
            }else if(__ret == ERROR_IRQ_MAPPING){
                pw_pr_error("ERROR: could NOT insert [%d,%s] into irq list!\n", irq_num, irq_name);
            }
        }
    }
#endif // DO_CACHE_IRQ_DEV_NAME_MAPPINGS
};

#define TRACK_TIMER_EXPIRES 1

static void timer_expire(void *timer_addr, pid_t tid)
{
    int cpu = -1;
    pid_t pid = -1;
    tnode_t *entry = NULL;
    u64 tsc = 0;
    bool found = false;
    bool was_hit = false;
    bool is_root = false;
    int irq_num = -1;
    s32 init_cpu = -1;

    /*
     * Reduce overhead -- do NOT run
     * if user specifies NO C-STATES.
     */
    if (unlikely(!IS_C_STATE_MODE())) {
	return;
    }

#if !TRACK_TIMER_EXPIRES
    {
        if (IS_COLLECTING()) {
            u64 sample_tsc;
            tscval(&sample_tsc);
            record_wakeup_cause(sample_tsc, PW_BREAK_TYPE_T, 0, -1, PID(), TID());
        }

        return;
    }
#endif

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif
    /*
     * Atomic context => use __get_cpu_var(...) instead of get_cpu_var(...)
     */
    irq_num = (&__get_cpu_var(per_cpu_counts))->was_timer_hrtimer_softirq;

    // was_hit = local_read(&__get_cpu_var(is_first_event)) == 1;
    was_hit = __get_cpu_var(wakeup_event_counter).event_tsc == 0;

#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
#endif // DO_IOCTL_STATS

    cpu = CPU();

    if ( (entry = (tnode_t *)timer_find((unsigned long)timer_addr, tid))) {
	pid = entry->pid;
	tsc = entry->tsc;
        init_cpu = entry->init_cpu;
	found = true;
        is_root = entry->is_root_timer;
    } else {
	/*
	 * Couldn't find timer entry -- PID defaults to TID.
	 */
	pid = tid;
	tsc = 0x1;
	OUTPUT(3, KERN_INFO "Warning: [%d]: timer %p NOT found in list!\n", pid, timer_addr);
        is_root = pid == 0;
    }

    if (!found) {
	// tsc = pw_max_num_cpus + 1;
        tsc = 0x0;
	if (tid < 0) {
	    /*
	     * Yes, this is possible, especially if
	     * the timer was fired because of a TIMER_SOFTIRQ.
	     * Special case that here.
	     */
            if (irq_num > 0) {
		/*
		 * Basically, fall back on the SOFTIRQ
		 * option because we couldn't quite figure
		 * out the process that is causing this
		 * wakeup. This is a duplicate of the
		 * equivalent code in "inter_common(...)".
		 */
		const char *irq_name = pw_softirq_to_name[irq_num];
		OUTPUT(3, KERN_INFO "WARNING: could NOT find TID in timer_expire for Timer = %p: FALLING BACK TO TIMER_SOFTIRQ OPTION! was_hit = %s\n", timer_addr, GET_BOOL_STRING(was_hit));
		handle_irq_wakeup_i(cpu, irq_num, irq_name, was_hit);
		/*
		 * No further action is required.
		 */
		return;
	    }
	    else {
		/*
		 * tid < 0 but this was NOT caused
		 * by a TIMER_SOFTIRQ.
		 * UPDATE: this is also possible if
		 * the kernel wasn't compiled with the
		 * 'CONFIG_TIMER_STATS' option set.
		 */
		OUTPUT(0, KERN_INFO "WARNING: NEGATIVE tid in timer_expire!\n");
	    }
	}
    } else {
	/*
	 * OK, found the entry. But timers fired
	 * because of 'TIMER_SOFTIRQ' will have
	 * tid == -1. Guard against that
	 * by checking the 'tid' value. If < 0
	 * then replace with entry->tid
	 */
	if(tid < 0){
	    tid = entry->tid;
	}
    }
    /*
     * Now send a sample to Ring-3.
     * (But only if collecting).
     */
    if (IS_COLLECTING()) {
        u64 sample_tsc;

        tscval(&sample_tsc);
        record_wakeup_cause(sample_tsc, PW_BREAK_TYPE_T, tsc, init_cpu, pid, tid);
    }

    /*
     * OK, send the TIMER::TSC mapping & call stack to the user
     * (but only if this is for a kernel-space call stack AND the
     * user wants kernel call stack info).
     */
    if (is_root && (IS_COLLECTING() || IS_SLEEPING()) && IS_KTIMER_MODE() && found && !entry->trace_sent) {
	produce_k_sample(cpu, entry);
	entry->trace_sent = 1;
    }
};

/*
 * High resolution timer (hrtimer) expire entry probe.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_expire_entry(struct hrtimer *hrt, ktime_t *now)
#else
    static void probe_hrtimer_expire_entry(void *ignore, struct hrtimer *hrt, ktime_t *now)
#endif
{
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, hrt, TIMER_START_PID(hrt));
};

/*
 * Macro to determine if the given
 * high resolution timer is periodic.
 */
#define IS_INTERVAL_TIMER(hrt) ({					\
	    bool __tmp = false;						\
	    pid_t pid = TIMER_START_PID(hrt);				\
	    ktime_t rem_k = hrtimer_expires_remaining(hrt);		\
	    s64 remaining = rem_k.tv64;					\
	    /* We first account for timers that */			\
	    /* are explicitly re-enqueued. For these */			\
	    /* we check the amount of time 'remaining' */		\
	    /* for the timer i.e.  how much time until */		\
	    /* the timer expires. If this is POSITIVE ==> */		\
	    /* the timer will be re-enqueued onto the */		\
	    /* timer list and is therefore PERIODIC */			\
	    if(remaining > 0){						\
		__tmp = true;						\
	    }else{							\
		/* Next, check for 'itimers' -- these INTERVAL TIMERS are */ \
		/* different in that they're only re-enqueued when their */ \
		/* signal (i.e. SIGALRM) is DELIVERED. Accordingly, we */ \
		/* CANNOT check the 'remaining' time for these timers. Instead, */ \
		/* we compare them to an individual task's 'REAL_TIMER' address.*/ \
		/* N.B.: Call to 'pid_task(...)' influenced by SEP driver code */ \
		struct task_struct *tsk = pid_task(find_pid_ns(pid, &init_pid_ns), PIDTYPE_PID); \
		__tmp = (tsk && ( (hrt) == &tsk->signal->real_timer));	\
	    }								\
	    __tmp; })


/*
 * High resolution timer (hrtimer) expire exit probe.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_hrtimer_expire_exit(struct hrtimer *hrt)
#else
    static void probe_hrtimer_expire_exit(void *ignore, struct hrtimer *hrt)
#endif
{
    if(!IS_INTERVAL_TIMER(hrt)){
        /*
         * timers are run from hardirq context -- no need
         * for expensive 'get_cpu_var(...)' variants.
         */
        per_cpu_t *pcpu = &__get_cpu_var(per_cpu_counts);
        /*
         * REMOVE the timer from
         * our timer map here (but
         * only if this isn't a 'sched_tick'
         * timer!)
         */
        if((void *)hrt != pcpu->sched_timer_addr){
            int ret = -1;
            DO_PER_CPU_OVERHEAD_FUNC_RET(ret, timer_delete, (unsigned long)hrt, TIMER_START_PID(hrt));
            if(ret){
                OUTPUT(0, KERN_INFO "WARNING: could NOT delete timer mapping for HRT = %p, TID = %d, NAME = %.20s\n", hrt, TIMER_START_PID(hrt), TIMER_START_COMM(hrt));
            }else{
                OUTPUT(3, KERN_INFO "OK: DELETED timer mapping for HRT = %p, TID = %d, NAME = %.20s\n", hrt, TIMER_START_PID(hrt), TIMER_START_COMM(hrt));
                // debugging ONLY!
                if(timer_find((unsigned long)hrt, TIMER_START_PID(hrt))){
                    OUTPUT(0, KERN_INFO "WARNING: TIMER_FIND reports TIMER %p STILL IN MAP!\n", hrt);
                }
            }
        }
    }
};

#define DEFERRABLE_FLAG (0x1)
#define IS_TIMER_DEFERRABLE(t) ( (unsigned long)( (t)->base) & DEFERRABLE_FLAG )


/*
 * Timer expire entry probe.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_timer_expire_entry(struct timer_list *t)
#else
    static void probe_timer_expire_entry(void *ignore, struct timer_list *t)
#endif
{
    DO_PER_CPU_OVERHEAD_FUNC(timer_expire, t, TIMER_START_PID(t));
};


/*
 * Function common to all interrupt tracepoints.
 */
static void inter_common(int irq_num, const char *irq_name)
{
    per_cpu_t *pcpu = NULL;

    bool was_hit = false;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    /*
     * Reduce overhead -- do NOT run
     * if user specifies NO C-STATES.
     */
    if(unlikely(!IS_C_STATE_MODE())){
        return;
    }

    /*
     * Debugging: make sure we're in
     * interrupt context!
     */
    if(!in_interrupt()){
        printk(KERN_ERR "BUG: inter_common() called from a NON-INTERRUPT context! Got irq: %lu and soft: %lu\n", in_irq(), in_softirq());
        return;
    }

    /*
     * Interrupt context: no need for expensive "get_cpu_var(...)" version.
     */
    pcpu = &__get_cpu_var(per_cpu_counts);

    /*
     * If this is a TIMER or an HRTIMER SOFTIRQ then
     * DO NOTHING (let the 'timer_expire(...)'
     * function handle this for greater accuracy).
     */
    if(false && (irq_num == TIMER_SOFTIRQ || irq_num == HRTIMER_SOFTIRQ)){
	pcpu->was_timer_hrtimer_softirq = irq_num;
#if DO_IOCTL_STATS
	/*
	 * Increment counter for timer interrupts as well.
	 */
	local_inc(&pstats->num_timers);
#endif // DO_IOCTL_STATS
	OUTPUT(3, KERN_INFO "(HR)TIMER_SOFTIRQ: # = %d\n", irq_num);
	return;
    }

#if DO_IOCTL_STATS
    pstats = &__get_cpu_var(per_cpu_stats);
    local_inc(&pstats->num_inters);

    /*
     * Increment counter for timer interrupts as well.
     */
    if(in_softirq() && (irq_num == TIMER_SOFTIRQ || irq_num == HRTIMER_SOFTIRQ))
        local_inc(&pstats->num_timers);
#endif

    /*
     * Check if this interrupt caused a C-state
     * wakeup (we'll use that info to decide
     * whether to cache this IRQ # <-> DEV name
     * mapping).
     */
    // was_hit = local_read(&__get_cpu_var(is_first_event)) == 1;
    was_hit = __get_cpu_var(wakeup_event_counter).event_tsc == 0;

    /*
     * OK, record a 'hit' (if applicable) and
     * send an i-sample message to Ring 3.
     */
    handle_irq_wakeup_i(CPU(), irq_num, irq_name, was_hit);
};

/*
 * IRQ tracepoint.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_irq_handler_entry(int irq, struct irqaction *action)
#else
static void probe_irq_handler_entry(void *ignore, int irq, struct irqaction *action)
#endif
{
    const char *name = action->name;
    OUTPUT(3, KERN_INFO "NUM: %d\n", irq);
    // inter_common(irq);
    DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
};

/*
 * soft IRQ tracepoint.
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_softirq_entry(struct softirq_action *h, struct softirq_action *vec)
#else
    static void probe_softirq_entry(void *ignore, struct softirq_action *h, struct softirq_action *vec)
#endif
{
    int irq = -1;
    const char *name = NULL;
    irq = (int)(h-vec);
    name = pw_softirq_to_name[irq];

    OUTPUT(3, KERN_INFO "NUM: %d\n", irq);

    DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
};
#else // >= 2.6.38
static void probe_softirq_entry(void *ignore, unsigned int vec_nr)
{
	int irq = (int)vec_nr;
	const char *name = pw_softirq_to_name[irq];

	DO_PER_CPU_OVERHEAD_FUNC(inter_common, irq, name);
};
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_workqueue_execution(struct task_struct *wq_thread, struct work_struct *work)
#else
static void probe_workqueue_execution(void * ignore, struct task_struct *wq_thread, struct work_struct *work)
#endif // < 2.6.35
{
    if (IS_COLLECTING()) {
        u64 tsc;
        tscval(&tsc);

        record_wakeup_cause(tsc, PW_BREAK_TYPE_W, 0, -1, -1, -1);
    }
};
#else // >= 2.6.36
static void probe_workqueue_execute_start(void *ignore, struct work_struct *work)
{
    if (IS_COLLECTING()) {
        u64 tsc;
        tscval(&tsc);

        record_wakeup_cause(tsc, PW_BREAK_TYPE_W, 0, -1, -1, -1);
    }
};
#endif // < 2.6.36

/*
 * Basically the same as arch/x86/kernel/irq.c --> "arch_irq_stat_cpu(cpu)"
 */

static u64 my_local_arch_irq_stats_cpu(void)
{
    u64 sum = 0;
    irq_cpustat_t *stats;
#ifdef __arm__
    int i=0;
#endif
    BEGIN_LOCAL_IRQ_STATS_READ(stats);
    {
#ifndef __arm__
// #ifdef CONFIG_X86_LOCAL_APIC
        sum += stats->apic_timer_irqs;
// #endif
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 34)
        sum += stats->x86_platform_ipis;
#endif // 2,6,34
        sum += stats->apic_perf_irqs;
#ifdef CONFIG_SMP
        sum += stats->irq_call_count;
        sum += stats->irq_resched_count;
        sum += stats->irq_tlb_count;
#endif
#ifdef CONFIG_X86_THERMAL_VECTOR
        sum += stats->irq_thermal_count;
#endif
        sum += stats->irq_spurious_count; // should NEVER be non-zero!!!
#else
        sum += stats->__softirq_pending;
#ifdef CONFIG_SMP
        for (i=0; i<NR_IPI; ++i) {
            sum += stats->ipi_irqs[i];
        }
#endif
#endif
    }
    END_LOCAL_IRQ_STATS_READ(stats);
    return sum;
};

static DEFINE_PER_CPU(u64, prev_c6_val) = 0;

/*
 * TPS epoch manipulation functions.
 */
#if DO_TPS_EPOCH_COUNTER

int inc_tps_epoch_i(void)
{
    int retVal = -1;
    /*
     * From "Documentation/memory-barriers.txt": "atomic_inc_return()"
     * has IMPLICIT BARRIERS -- no need to add explicit barriers
     * here!
     */
    retVal = atomic_inc_return(&tps_epoch);
    return retVal;
};

int read_tps_epoch_i(void)
{
    /*
     * Make sure TPS updates have propagated
     */
    smp_mb();
    return atomic_read(&tps_epoch);
};
#endif // DO_TPS_EPOCH_COUNTER

static int pw_read_msr_set_i(struct msr_set *msr_set, int *which_cx, u64 *cx_val)
{
    int num_res = 0;
#ifndef __arm__
    int i=0;
    u64 val = 0;
    s32 msr_addr = -1;

    *which_cx = APERF;

    // for (i=C2; i<MAX_MSR_ADDRESSES; ++i) {
    for (i=C9; i>= C2; --i) { // iterate backwards!
        msr_set->curr_msr_count[i] = 0;
        msr_addr = INTERNAL_STATE.coreResidencyMSRAddresses[i];
        // msr_addr = INTERNAL_STATE.pkgResidencyMSRAddresses[i];
        if (msr_addr <= 0) {
            continue;
        }
        rdmsrl(msr_addr, val);
        if (msr_set->prev_msr_vals[i] != 0 && msr_set->prev_msr_vals[i] != val) {
            *which_cx = i; *cx_val = val;
            ++num_res;
            msr_set->curr_msr_count[i] = val;
        }
        msr_set->prev_msr_vals[i] = val;
    }
#else
    // probe_power_end fills in these statistics when it is called
    // so we just grab what is set here.  On x86 we grab and set them above
    *which_cx = msr_set->prev_req_cstate;
    *cx_val = msr_set->curr_msr_count[msr_set->prev_req_cstate];
    num_res = 1;
#endif // ifndef __arm__
    return num_res;
};

static void tps(unsigned int type, unsigned int state)
{
    int cpu = CPU(), epoch = 0;
    u64 tsc = 0;
    PWCollector_msg_t sample;
    int which_cx = APERF;
    u64 cx_val = 0;
    msr_set_t *set = NULL;
    bool local_apic_fired = false;
    u32 prev_req_cstate = 0;
    u8 init_msr_set_sent = 1;
    int num_cx = -1;
    u64 msr_vals[MAX_MSR_ADDRESSES];
    u64 msr_set[MAX_MSR_ADDRESSES];
    PWC_tps_msg_t c_msg;
    c_msg_t *cm = &c_msg.data;

    const char *wakeup_reasons[] = {"IRQ", "TIM", "SCHED", "IPI", "WRQ", "BEGIN", "NOT", "ABRT", "?"};
#ifdef __arm__
    u64 *prev_tsc = NULL;
    u64 c0_time = 0;
#endif

    memset(msr_vals, 0, sizeof(u64) * MAX_MSR_ADDRESSES);

    tscval(&tsc);

    // trace_printk("APWR: [%d] TPS at %llu\n", cpu, tsc);

    /*
     * Read all C-state MSRs.
     */
    set = &get_cpu_var(pw_pcpu_msr_sets);
#ifdef __arm__
    ++state;  // on ARM (Nexus 7 at least) states start with LP3 and no C0
    prev_tsc = &get_cpu_var(trace_power_prev_time);
    c0_time = tsc - *prev_tsc;
    *prev_tsc = tsc;
    put_cpu_var(trace_power_prev_time);
    set->prev_msr_vals[MPERF] += c0_time;
    c0_time = set->prev_msr_vals[MPERF];
#endif // __arm__
    {
        init_msr_set_sent = set->init_msr_set_sent;
        prev_req_cstate = set->prev_req_cstate;
        num_cx = pw_read_msr_set_i(set, &which_cx, &cx_val);
        set->prev_req_cstate = (u32)state; // must be after pw_read_msr_set_i for ARM changes
        if (unlikely(init_msr_set_sent == 0)) {
            memcpy(msr_set, set->prev_msr_vals, sizeof(u64) * MAX_MSR_ADDRESSES);
            set->init_msr_set_sent = 1;
        }
        if (unlikely(num_cx > 1)) {
            memcpy(msr_vals, set->curr_msr_count, sizeof(u64) * MAX_MSR_ADDRESSES);
        }
    }
    put_cpu_var(pw_pcpu_msr_sets);

    /*
     * Check if the local APIC timer raised interrupts.
     * Only required if we're capturing C-state samples.
     */
    if (IS_C_STATE_MODE()) {
        u64 event_tsc = 0, event_val = 0;
        pid_t event_tid = -1, event_pid = -1;
        c_break_type_t event_type = PW_BREAK_TYPE_U;
        s32 event_init_cpu = -1;
        /*
         * See if we can get a wakeup cause, along
         * with associated data.
         * We use "__get_cpu_var()" instead of "get_cpu_var()" because
         * it is OK for us to be preempted out at any time. Also, not
         * disabling preemption saves us about 500 cycles per TPS.
         */
        {
            struct wakeup_event *wu_event = &get_cpu_var(wakeup_event_counter);
            if (wu_event->event_tsc > 0) {
                event_type = wu_event->event_type;
                event_val = wu_event->event_val;
                event_tsc = wu_event->event_tsc;
                event_pid = wu_event->event_pid;
                event_tid = wu_event->event_tid;
                event_init_cpu = wu_event->init_cpu;
                wu_event->event_tsc = 0; // reset for the next wakeup event.
            }
            put_cpu_var(wakeup_event_counter);
        }
        /*
         * Check if the local APIC timer raised interrupts.
         */
        {
            u64 curr_num_local_apic = my_local_arch_irq_stats_cpu();
            u64 *old_num_local_apic = &__get_cpu_var(num_local_apic_timer_inters);
            if (*old_num_local_apic && (*old_num_local_apic != curr_num_local_apic)) {
                local_apic_fired = true;
            }
            *old_num_local_apic = curr_num_local_apic;
        }

        if (event_type == PW_BREAK_TYPE_U && local_apic_fired && IS_COLLECTING()) {
            event_type = PW_BREAK_TYPE_IPI;
            /*
             * We need a 'TSC' for this IPI sample but we don't know
             * WHEN the local APIC timer interrupt was raised. Fortunately, it doesn't
             * matter, because we only need to ensure this sample lies
             * BEFORE the corresponding 'C_STATE' sample in a sorted timeline.
             * We therefore simply subtract one from the C_STATE sample TSC to get
             * the IPI sample TSC.
             */
            event_tsc = tsc - 1;
            event_tid = event_pid = -1;
            event_val = 0;
        }

        if (unlikely(init_msr_set_sent == 0)) {
            /*
             * OK, this is the first TPS for this thread during the current collection. Send two messages:
             * 1. First, a "POSIX_TIME_SYNC" message to allow Ring-3 to correlate the TSC used by wuwatch with 
             * the "clock_gettime()" used by TPSS.
             */
            {
                struct timespec ts;
                tsc_posix_sync_msg_t tsc_msg;
                u64 tmp_tsc = 0, tmp_nsecs = 0;
                ktime_get_ts(&ts);
                tscval(&tmp_tsc);
                tmp_nsecs = (u64)ts.tv_sec * 1000000000ULL + (u64)ts.tv_nsec;
                tsc_msg.tsc_val = tmp_tsc; tsc_msg.posix_mono_val = tmp_nsecs;

                sample.tsc = tsc;
                sample.cpuidx = cpu;
                sample.data_type = TSC_POSIX_MONO_SYNC;
                sample.data_len = sizeof(tsc_msg);
                sample.p_data = (u64)((unsigned long)&tsc_msg);

                pw_produce_generic_msg(&sample, true);
                pw_pr_debug(KERN_INFO "[%d]: SENT POSIX_TIME_SYNC\n", cpu);
                printk(KERN_INFO "[%d]: tsc = %llu posix mono = %llu\n", cpu, tmp_tsc, tmp_nsecs);
            }
            /*
             * 2. Second, the initial MSR 'set' for this (logical) CPU.
             */
            {
                sample.tsc = tsc;
                sample.cpuidx = cpu;
                sample.data_type = C_STATE_MSR_SET;
                sample.data_len = sizeof(msr_set);
                sample.p_data = (u64)((unsigned long)msr_set);

                // Why "true"? Document!
                pw_produce_generic_msg(&sample, true);
                pw_pr_debug(KERN_INFO "[%d]: SENT init msr set\n", cpu);
            }
        }

        c_msg.cpuidx = cpu;
        c_msg.tsc = tsc;

#ifndef __arm__
        {
            u64 mperf = 0;
            rdmsrl(INTERNAL_STATE.coreResidencyMSRAddresses[MPERF], mperf);
            cm->mperf = mperf;
        }
#else
        cm->mperf = c0_time;
#endif // ifndef __arm__
        // cm->req_state = (u8)prev_req_cstate;
        cm->req_state = (u8)state;
        cm->act_state = (u8)which_cx;
        cm->cx_msr_val = cx_val;
 
	cm->wakeup_tsc = event_tsc;
	cm->wakeup_data = event_val;
        cm->timer_init_cpu = event_init_cpu;
	cm->wakeup_pid = event_pid;
	cm->wakeup_tid = event_tid;
	cm->wakeup_type = event_type;

        // trace_printk("APWR: TPS at %llu has wakeup reason = %s\n", tsc, wakeup_reasons[event_type]);


        c_msg.data_type = C_STATE;
        c_msg.data_len = sizeof(*cm);

#if DO_TPS_EPOCH_COUNTER
        /*
         * We're entering a new TPS "epoch".
         * Increment our counter.
         */
        epoch = inc_tps_epoch_i();
        cm->tps_epoch = epoch;
#endif // DO_TPS_EPOCH_COUNTER

        if (IS_COLLECTING()) {
            DO_PER_CPU_OVERHEAD_FUNC(pw_produce_tps_msg, &c_msg);
#ifndef __arm__
            if (unlikely(num_cx > 1)) {
                // pw_pr_warn("WARNING: [%d] has %d cx residencies!\n", cpu, num_cx);
                // printk(KERN_INFO "WARNING: [%d] has %d cx residencies at TSC = %llu! Prev = %d, val = %llu\n", cpu, num_cx, tsc, which_cx, cx_val);
                for (--num_cx; num_cx > 0; --num_cx) {
                    /*
                     * Subsequent samples have the same 'header' information (i.e. same TSC, same cpuidx etc.),
                     * and also the same 'mperf' and 'wakeup' information. The ONLY things that are different
                     * are the 'which_cx' and 'cx_msr_val' fields.
                     */
                    int i=0;
                    for (i=C2; i<MAX_MSR_ADDRESSES; ++i) {
                        if (i == which_cx || msr_vals[i] == 0) {
                            continue;
                        }
                        pw_pr_debug("[%d]: cx = %d\n", cpu, i);
                        cm->act_state = i; cm->cx_msr_val = msr_vals[i];
                        DO_PER_CPU_OVERHEAD_FUNC(pw_produce_tps_msg, &c_msg);
                    }
                }
            }
#endif // __arm__
        }

        /*
         * Reset the "first-hit" variable.
         */
        {
            __get_cpu_var(wakeup_event_counter).event_tsc = 0;
        }
    } // IS_C_STATE_MODE()

    // Collect S and D state / residency counter samples on CPU0
    if (cpu != 0 || !IS_COLLECTING()) {
        return;
    }

    /* 
     * Controls the sampling frequency to reduce data collection overheads
     * UPDATE: disabled for now. Sampling of NC D-states is now being driven by
     * a Ring-3 timer.
     */
#if 0
    if ((CURRENT_TIME_IN_USEC() - prev_sample_usec) > INTERNAL_STATE.d_state_sample_interval*1000) {
        prev_sample_usec = CURRENT_TIME_IN_USEC();

#if DO_S_STATE_SAMPLE 
        if (pw_is_atm && IS_S_STATE_MODE()) {
            if (INTERNAL_STATE.write_to_buffers) {
                produce_s_state_sample();
            }
        }
#endif

#if DO_D_NC_STATE_SAMPLE 
        if (pw_is_atm && IS_D_NC_STATE_MODE()) {
            if (INTERNAL_STATE.write_to_buffers) {
                produce_d_nc_state_sample();
            }
        }
#endif

#if DO_D_SC_STATE_SAMPLE 
        if (pw_is_atm && IS_D_SC_STATE_MODE()) {
            if (INTERNAL_STATE.write_to_buffers) {
                produce_d_sc_state_sample();
            }
        }
#endif
    }
#endif // if 0

};

/*
 * C-state break.
 * Read MSR residencies.
 * Also gather information on what caused C-state break.
 * If so configured, write C-sample information to (per-cpu)
 * output buffer.
 */
#ifdef __arm__
/* 
 * TODO: we may want to change this to call receive_wakeup_cause()
 * and put the logic for the ARM stuff in there.  The reason
 * we don't do it now is we want to make sure this isn't called
 * before the timer or interrupt trace calls or we will attribute
 * the cause of the wakeup as this function instead of an interrupt or
 * timer.  We can probably fix that by cleaning up the logic a bit
 * but for the merge we ignore that for now
 */
static void probe_power_end(void *ignore)
{
    u64 tsc = 0;
    
    msr_set_t *set = NULL;
    u64 *prev_tsc = NULL;
    u64 trace_time = 0;

    tscval(&tsc);
    prev_tsc = &get_cpu_var(trace_power_prev_time);
    trace_time = tsc - *prev_tsc;

    *prev_tsc = tsc;
    put_cpu_var(trace_power_prev_time);

    /*
     * Set all C-state MSRs.
     */
    set = &get_cpu_var(pw_pcpu_msr_sets);
    {
        set->prev_msr_vals[set->prev_req_cstate] += trace_time;

        memset(set->curr_msr_count, 0, sizeof(u64) * MAX_MSR_ADDRESSES);
        set->curr_msr_count[set->prev_req_cstate] =
            set->prev_msr_vals[set->prev_req_cstate];
    }
    put_cpu_var(pw_pcpu_msr_sets);
}
#endif // __arm__

#if APWR_RED_HAT
/*
 * Red Hat back ports SOME changes from 2.6.37 kernel
 * into 2.6.32 kernel. Special case that here.
 */
static void probe_power_start(unsigned int type, unsigned int state, unsigned int cpu_id)
{
    // tps_i(type, state);
    DO_PER_CPU_OVERHEAD_FUNC(tps, type, state);
};
#else
#if LINUX_VERSION_CODE  < KERNEL_VERSION(2,6,38)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_power_start(unsigned int type, unsigned int state)
#elif LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
static void probe_power_start(void *ignore, unsigned int type, unsigned int state)
#else // 2.6.36 <= version < 2.6.38
static void probe_power_start(void *ignore, unsigned int type, unsigned int state, unsigned int cpu_id)
#endif
{
    // tps_i(type, state);
    DO_PER_CPU_OVERHEAD_FUNC(tps, type, state);
};
#else // version >= 2.6.38
static void probe_cpu_idle(void *ignore, unsigned int state, unsigned int cpu_id)
{
   if (state == PWR_EVENT_EXIT) {
#ifdef __arm__
       probe_power_end(NULL);
#endif
       return;
   }

    // tps_i(type, state);
   DO_PER_CPU_OVERHEAD_FUNC(tps, 0 /*type*/, state);
};
#endif // version
#endif // APWR_RED_HAT
#ifdef __arm__
#if TRACE_CPU_HOTPLUG
static void probe_cpu_hotplug(void *ignore, unsigned int state, int cpu_id)
{
        PWCollector_msg_t output_sample;
        event_sample_t event_sample;
        u64 sample_tsc;

        tscval(&sample_tsc);
        output_sample.cpuidx = cpu_id;
        output_sample.tsc = sample_tsc;

        event_sample.data[0] = cpu_id;
        event_sample.data[1] = state;

#if DO_TPS_EPOCH_COUNTER
        event_sample.data[2] = read_tps_epoch_i();
#endif // DO_TPS_EPOCH_COUNTER

        output_sample.data_type = CPUHOTPLUG_SAMPLE;
        output_sample.data_len = sizeof(event_sample);
        output_sample.p_data = (u64)((unsigned long)&event_sample);

        pw_produce_generic_msg(&output_sample, false); // "false" ==> don't wake any sleeping readers (required from scheduling context)
}
#endif // TRACE_CPU_HOTPLUG
#endif // __arm__

/*
 * Tokenize the Frequency table string
 * to extract individual frequency 'steps'
 */
int extract_valid_frequencies(const char *buffer, ssize_t len)
{
    const char *str = NULL;
    char tmp[10];
    int num_toks = 0, i=0, j=0, tmp_len = 0;
    unsigned long freq = 0;

    if(len <= 0 || !buffer)
	return -ERROR;

    /*
     * Step-1: find out number of
     * frequency steps
     */
    for (i=0; i<len; ++i) {
	if (buffer[i] == ' ') {
	    ++num_toks;
        }
    }

    /*
     * We don't keep a separate "len"
     * field to indicate # of freq
     * steps. Instead, we write a
     * ZERO in the last entry of the
     * 'apwr_available_frequencies' table.
     * Increase the size of the table to
     * accommodate this.
     */
    ++num_toks;

    /*
     * Step-2: allocate memory etc.
     */
    if(!(apwr_available_frequencies = pw_kmalloc(sizeof(u32) * num_toks, GFP_KERNEL))){
	pw_pr_error("ERROR: could NOT allocate memory for apwr_available_frequencies array!\n");
	return -ERROR;
    }

    /*
     * Step-3: extract the actual frequencies.
     */
    for (i=0, j=0, str = buffer; i<len; ++i) {
	if (buffer[i] == ' ') {
	    memset(tmp, 0, sizeof(tmp));
	    ++num_toks;
            tmp_len = (buffer+i) - str;
            if (tmp_len > sizeof(tmp)) {
		// ERROR!
		return -ERROR;
	    }
	    strncpy(tmp, str, tmp_len);
	    // fprintf(stderr, "TOKEN = %d\n", atoi(tmp));
	    freq = simple_strtoul(tmp, NULL, 10);
	    apwr_available_frequencies[j++] = (u32)freq;
	    OUTPUT(0, KERN_INFO "FREQ-STEP: %lu\n", freq);
	    str = buffer + i + 1;
	}
    }
    /*
     * Now write the ZERO entry 
     * denoting EOF
     */
    apwr_available_frequencies[j] = 0x0;
    /*
     * Special support for MFLD -- if we
     * couldn't get the 'base_operating_freq_khz'
     * then set that now.
     */
    if (!base_operating_freq_khz) {
	OUTPUT(0, KERN_INFO "WARNING: no \"base_operating_frequency\" set -- using %u\n", apwr_available_frequencies[0]);
	base_operating_freq_khz = apwr_available_frequencies[0];
    }
    return SUCCESS;
};

#define IS_BRACKETED_BY(upper, lower, freq) ({bool __tmp = ( (upper) >= (freq) && (freq) >= (lower) ) ? true : false; __tmp;})
#define GET_CORRECT_FREQ_BOUND(upper, lower, freq) ({u32 __delta_upper = (upper) - (freq), __delta_lower = (freq) - (lower), __tmp = (__delta_upper < __delta_lower) ? (upper) : (lower); __tmp;})

/*
 * Given a frequency calculated using the
 * 'aperf/mperf' ratio, try and figure out
 * what the ACTUAL frequency was.
 */
unsigned long get_actual_frequency(unsigned long avg_freq)
{
    /*
     * Basic algo: iterate through array of
     * available frequencies, comparing 'avg_freq'
     * to each. Stop at FIRST frequency which
     * is LESS than 'avg_freq' (i.e. we're ROUNDING
     * DOWN the 'avg_freq').
     *
     * Note that we should probably be trying to
     * find which of the 'available' frequencies
     * is CLOSEST to 'avg_freq'. For now,
     * we rely on this more primitive method.
     *
     * UPDATE: we're now finding the CLOSEST
     * frequency to the 'avg_freq' value.
     */
    int i=0;

    if(!apwr_available_frequencies || apwr_available_frequencies[0] == 0)
	return 0;
    /*
     * SPECIAL CASE FOR TURBO frequencies:
     * If 'avg_freq' > FIRST entry in array
     * (i.e. the TURBO entry) then just
     * return 'avg_freq'
     * UPDATE: NO -- for now we return the
     * TSC freq + 1 MHz i.e. the first TURBO
     * freq
     */
    /*
    if(avg_freq > apwr_available_frequencies[0])
	return avg_freq;
        */
    if (avg_freq >= apwr_available_frequencies[0]) {
        return apwr_available_frequencies[0]; // in Hz
    }

    /*
     * Try and find the frequency STEP that's
     * closest to 'avg_freq'
     */
    for(i=1; apwr_available_frequencies[i] != 0; ++i){
	u32 upper = apwr_available_frequencies[i-1], lower = apwr_available_frequencies[i];
	if(IS_BRACKETED_BY(upper, lower, avg_freq)){
	    u32 act_freq = GET_CORRECT_FREQ_BOUND(upper, lower, avg_freq);
	    OUTPUT(3, KERN_INFO "AVG = %lu, UPPER = %u, LOWER = %u, CLOSEST = %u, \n", avg_freq, upper, lower, act_freq);
	    return act_freq;
	}
    }

    /*
     * OK, couldn't find a suitable freq -- just
     * return the 'avg_freq'.
     * UPDATE: reaching here implies a frequency
     * LOWER than the LOWEST frequency step. In this
     * case, return this LOWEST frequency.
     */
    return apwr_available_frequencies[--i];
};


/*
 * New methodology -- We're now using APERF/MPERF
 * collected within the TPS probe to calculate actual 
 * frequencies. Only calculate TSC values within
 * the 'power_frequency' tracepoint.
 */
#ifndef __arm__
static void tpf(int cpu, unsigned int type, u32 req_freq)
{
    u64 tsc = 0, aperf = 0, mperf = 0;
    u32 prev_req_freq = 0;
    u32 perf_status = 0;

#if DO_IOCTL_STATS
    stats_t *pstats = NULL;
#endif

    /*
     * We're not guaranteed that 'cpu' (which is the CPU on which the frequency transition is occuring) is
     * the same as the cpu on which the callback i.e. the 'TPF' probe is executing. This is why we use 'rdmsr_safe_on_cpu()'
     * to read the various MSRs.
     */
    /*
     * Read TSC value
     */
    u32 l=0, h=0;
    WARN_ON(rdmsr_safe_on_cpu(cpu, 0x10, &l, &h));
    tsc = (u64)h << 32 | (u64)l;
    /*
     * Read CPU_CLK_UNHALTED.REF and CPU_CLK_UNHALTED.CORE. These required ONLY for AXE import
     * backward compatibility!
     */
#if 1
    {
        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[APERF], &l, &h));
        aperf = (u64)h << 32 | (u64)l;

        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[MPERF], &l, &h));
        mperf = (u64)h << 32 | (u64)l;
    }
#endif

    /*
     * Read the IA32_PERF_STATUS MSR. Bits 12:8 (on Atom ) or 15:0 (on big-core) of this determines 
     * the frequency the H/W is currently running at.
     * We delegate the actual frequency computation to Ring-3 because the PERF_STATUS encoding is
     * actually model-specific.
     */
    WARN_ON(rdmsr_safe_on_cpu(cpu, IA32_PERF_STATUS_MSR_ADDR, &l, &h));
    perf_status = l; // We're only interested in the lower 16 bits!

    /*
     * Retrieve the previous requested frequency, if any. 
     */
    prev_req_freq = per_cpu(pcpu_prev_req_freq, cpu);
    per_cpu(pcpu_prev_req_freq, cpu) = req_freq;

    produce_p_sample(cpu, tsc, prev_req_freq, perf_status, 0 /* boundary */, aperf, mperf); // "0" ==> NOT a boundary sample

    OUTPUT(0, KERN_INFO "[%d]: TSC = %llu, OLD_req_freq = %u, NEW_REQ_freq = %u, perf_status = %u\n", cpu, tsc, prev_req_freq, req_freq, perf_status); 

#if DO_IOCTL_STATS
    {
	pstats = &get_cpu_var(per_cpu_stats);
	local_inc(&pstats->p_trans);
	put_cpu_var(pstats);
    }
#endif // DO_IOCTL_STATS
};
#endif // not def __arm__

#if DO_CPUFREQ_NOTIFIER
/*
 * CPUFREQ notifier callback function.
 * Used in cases where the default
 * power frequency tracepoint mechanism
 * is broken (e.g. MFLD).
 */
static int apwr_cpufreq_notifier(struct notifier_block *block, unsigned long val, void *data)
{
    struct cpufreq_freqs *freq = data;
    u32 state = freq->new; // "state" is frequency CPU is ABOUT TO EXECUTE AT
    int cpu = freq->cpu;

    if (unlikely(!IS_FREQ_MODE())) {
	return SUCCESS;
    }

    if (val == CPUFREQ_PRECHANGE) {
#ifndef __arm__
        DO_PER_CPU_OVERHEAD_FUNC(tpf, cpu, 2, state);
#else
        u64 tsc;
        u32 prev_req_freq = 0;
        u32 perf_status = 0;
        
        tscval(&tsc);

        prev_req_freq = per_cpu(pcpu_prev_req_freq, cpu);
        per_cpu(pcpu_prev_req_freq, cpu) = state;

        perf_status = prev_req_freq / INTERNAL_STATE.bus_clock_freq_khz;
        produce_p_sample(cpu, tsc, prev_req_freq, perf_status, 0 /* boundary */, 0, 0); // "0" ==> NOT a boundary sample
#endif // not def __arm__
    }
    return SUCCESS;
};

static struct notifier_block apwr_cpufreq_notifier_block = {
    .notifier_call = &apwr_cpufreq_notifier
};

#else // DO_CPUFREQ_NOTIFIER

/*
 * P-state transition probe.
 *
 * "type" is ALWAYS "2" (i.e. "POWER_PSTATE", see "include/trace/power.h")
 * "state" is the NEXT frequency range the CPU is going to enter (see "arch/x86/kernel/cpu/cpufreq/acpi-cpufreq.c")
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_power_frequency(unsigned int type, unsigned int state)
#else
static void probe_power_frequency(void *ignore, unsigned int type, unsigned int state)
#endif
{
    if(unlikely(!IS_FREQ_MODE())){
        return;
    }
    DO_PER_CPU_OVERHEAD_FUNC(tpf, CPU(), type, state);
};

#endif // DO_CPUFREQ_NOTIFIER


/*
 * Helper function for "probe_sched_exit"
 * Useful for overhead measurements.
 */
static void exit_helper(struct task_struct *task)
{
    pid_t tid = task->pid, pid = task->tgid;

    OUTPUT(3, KERN_INFO "[%d]: SCHED_EXIT\n", tid);
    /*
     * Delete all (non-Kernel) timer mappings created
     * for this thread.
     */
    delete_timers_for_tid(tid);
    /*
     * Delete any sys-node mappings created on behalf
     * of this thread.
     */
    check_and_delete_proc_from_sys_list(tid, pid);

};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sched_process_exit(struct task_struct *task)
#else
    static void probe_sched_process_exit(void *ignore, struct task_struct *task)
#endif
{
    pid_t tid = task->pid, pid = task->tgid;
    const char *name = task->comm;
    u64 tsc;


    OUTPUT(3, KERN_INFO "[%d, %d]: %s exitting\n", tid, pid, name);

    DO_PER_CPU_OVERHEAD_FUNC(exit_helper, task);

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING!
     * UPDATE: track if COLLECTION ONGOING OR
     * IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
	return;
    }

    tscval(&tsc);

    produce_r_sample(CPU(), tsc, PW_PROC_EXIT, tid, pid, name);
};

void __attribute__((always_inline)) sched_wakeup_helper_i(struct task_struct *task)
{
    int target_cpu = task_cpu(task), source_cpu = CPU();
    /*
     * "Self-sched" samples are "don't care".
     */
    if (target_cpu != source_cpu) {

        PWCollector_msg_t output_sample;
        event_sample_t event_sample;
        u64 sample_tsc;

        tscval(&sample_tsc);
        output_sample.cpuidx = source_cpu;
        output_sample.tsc = sample_tsc;

        event_sample.data[0] = source_cpu;
        event_sample.data[1] = target_cpu;

#if DO_TPS_EPOCH_COUNTER
        event_sample.data[2] = read_tps_epoch_i();
#endif // DO_TPS_EPOCH_COUNTER

        output_sample.data_type = SCHED_SAMPLE;
        output_sample.data_len = sizeof(event_sample);
        output_sample.p_data = (u64)((unsigned long)&event_sample);

        pw_produce_generic_msg(&output_sample, false); // "false" ==> don't wake any sleeping readers (required from scheduling context)
    }
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sched_wakeup(struct rq *rq, struct task_struct *task, int success)
#else
static void probe_sched_wakeup(void *ignore, struct task_struct *task, int success)
#endif
{
    if (likely(IS_COLLECTING())) {
        sched_wakeup_helper_i(task); 
    }
};


bool __attribute__((always_inline)) is_sleep_syscall_i(long id) 
{
    switch (id) {
        case __NR_poll: // 7
        case __NR_select: // 23
        case __NR_nanosleep: // 35
        case __NR_alarm: // 37
        case __NR_setitimer: // 38
        case __NR_rt_sigtimedwait: // 128
        case __NR_futex: // 202
        case __NR_timer_settime: // 223
        case __NR_clock_nanosleep: // 230
        case __NR_epoll_wait: // 232
        case __NR_pselect6: // 270
        case __NR_ppoll: // 271
        case __NR_epoll_pwait: // 281
        case __NR_timerfd_settime: // 286
            return true;
        default:
            break;
    }
    return false;
};

void  __attribute__((always_inline)) sys_enter_helper_i(long id, pid_t tid, pid_t pid)
{
    if (check_and_add_proc_to_sys_list(tid, pid)) {
        pw_pr_error("ERROR: could NOT add proc to sys list!\n");
    }
    return;
};

void  __attribute__((always_inline)) sys_exit_helper_i(long id, pid_t tid, pid_t pid)
{
    check_and_remove_proc_from_sys_list(tid, pid);
};


#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sys_enter(struct pt_regs *regs, long ret)
#else
static void probe_sys_enter(void *ignore, struct pt_regs *regs, long ret)
#endif
{
    long id = syscall_get_nr(current, regs);
    pid_t tid = TID(), pid = PID();

    if (is_sleep_syscall_i(id)) {
        DO_PER_CPU_OVERHEAD_FUNC(sys_enter_helper_i, id, tid, pid);
    }
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sys_exit(struct pt_regs *regs, long ret)
#else
static void probe_sys_exit(void *ignore, struct pt_regs *regs, long ret)
#endif
{
    long id = syscall_get_nr(current, regs);
    pid_t tid = TID(), pid = PID();

    DO_PER_CPU_OVERHEAD_FUNC(sys_exit_helper_i, id, tid, pid);

    if(id == __NR_execve && IS_COLLECTING()){
        u64 tsc;

        tscval(&tsc);
        OUTPUT(3, KERN_INFO "[%d]: EXECVE ENTER! TID = %d, NAME = %.20s\n", CPU(), TID(), NAME());
        produce_r_sample(CPU(), tsc, PW_PROC_EXEC, TID(), PID(), NAME());
    }
};
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_sched_process_fork(struct task_struct *parent, struct task_struct *child)
#else
    static void probe_sched_process_fork(void *ignore, struct task_struct *parent, struct task_struct *child)
#endif
{
    const char *cname = child->comm;
    pid_t ctid = child->pid, cpid = child->tgid;
    u64 tsc;

    tscval(&tsc);

    OUTPUT(3, KERN_INFO "DEBUG: PROCESS_FORK: %d (%.20s) --> %d (%.20s) \n", parent->pid, parent->comm, child->pid, cname);

    if (IS_COLLECTING() || IS_SLEEPING()) {
        produce_r_sample(CPU(), tsc, PW_PROC_FORK, ctid, cpid, cname);
    }
};

/*
 * Notifier for module loads and frees.
 * We register module load and free events -- extract memory bounds for
 * the module (on load). Also track TID, NAME for tracking device driver timers.
 */
int apwr_mod_notifier(struct notifier_block *block, unsigned long val, void *data)
{
    struct module *mod = data;
    int cpu = CPU();
    const char *name = mod->name;
    unsigned long module_core = (unsigned long)mod->module_core;
    unsigned long core_size = mod->core_size;

    if (IS_COLLECTING() || IS_SLEEPING()) {
        if (val == MODULE_STATE_COMING) {
            OUTPUT(0, KERN_INFO "COMING: tid = %d, pid = %d, name = %s, module_core = %lu\n", TID(), PID(), name, module_core);
            produce_m_sample(cpu, name, module_core, core_size);
            // return add_new_module_to_mod_list(TID(), PID(), mod);
        } else if (val == MODULE_STATE_GOING) {
            OUTPUT(0, KERN_INFO "GOING: tid = %d, pid = %d, name = %s\n", TID(), PID(), name);
            // return remove_module_from_mod_list(mod);
        }
    }
    return SUCCESS;
};

static struct notifier_block apwr_mod_notifier_block = {
    .notifier_call = &apwr_mod_notifier
};


#if DO_WAKELOCK_SAMPLE 
/*
 * Wakelock hooks
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_wake_lock(struct wake_lock *lock)
#else
static void probe_wake_lock(void *ignore, struct wake_lock *lock)
#endif
{
    u64 tsc;
    u64 timeout = 0;
    w_sample_type_t wtype;

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING OR IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
	return;
    }

    tscval(&tsc);
    if (lock->flags & (1U << 10)){   // Check if WAKE_LOCK_AUTO_EXPIRE is flagged
        wtype = PW_WAKE_LOCK_TIMEOUT;
        timeout = jiffies_to_msecs(lock->expires - jiffies);
    }else{
        wtype = PW_WAKE_LOCK;
    }

    produce_w_sample(CPU(), tsc, wtype , TID(), PID(), lock->name, NAME(), timeout);

    OUTPUT(0, "wake_lock: type=%d, name=%s, timeout=%llu (msec), CPU=%d, PID=%d, TSC=%llu\n", wtype, lock->name, timeout, CPU(), PID(), tsc);
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
static void probe_wake_unlock(struct wake_lock *lock)
#else
static void probe_wake_unlock(void *ignore, struct wake_lock *lock)
#endif
{
    u64 tsc;

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING OR IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
        return;
    }

    tscval(&tsc);

    produce_w_sample(CPU(), tsc, PW_WAKE_UNLOCK, TID(), PID(), lock->name, NAME(), 0);

    OUTPUT(0, "wake_unlock: name=%s, CPU=%d, PID=%d, TSC=%llu\n", lock->name, CPU(), PID(), tsc);
};

#else
static void probe_wakeup_source_activate(void *ignore, const char *name, unsigned int state)
{
    u64 tsc;
    u64 timeout = 0;
    w_sample_type_t wtype;

    if (name == NULL) {
        printk("wake_lock: name=UNKNOWNs, state=%u\n", state);
        return; 
    }

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING OR IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
	return;
    }

    tscval(&tsc);
    wtype = PW_WAKE_LOCK;

    produce_w_sample(CPU(), tsc, wtype , TID(), PID(), name, NAME(), timeout);

    OUTPUT(0, "wake_lock: type=%d, name=%s, timeout=%llu (msec), CPU=%d, PID=%d, TSC=%llu\n", wtype, name, timeout, CPU(), PID(), tsc);
};

static void probe_wakeup_source_deactivate(void *ignore, const char *name, unsigned int state)
{
    u64 tsc;

    if (name == NULL) {
        printk("wake_unlock: name=UNKNOWNs, state=%u\n", state);
        return; 
    }

    /*
     * Track task exits ONLY IF COLLECTION
     * ONGOING OR IF IN PAUSED STATE!
     */
    if(!IS_COLLECTING() && !IS_SLEEPING()){
        return;
    }

    tscval(&tsc);

    produce_w_sample(CPU(), tsc, PW_WAKE_UNLOCK, TID(), PID(), name, NAME(), 0);

    OUTPUT(0, "wake_unlock: name=%s, CPU=%d, PID=%d, TSC=%llu\n", name, CPU(), PID(), tsc);
};
#endif
#endif

static int register_timer_callstack_probes(void)
{
    int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    {
	OUTPUT(0, KERN_INFO "\tTIMER_INIT_EVENTS");
	ret = register_trace_hrtimer_init(probe_hrtimer_init);
	WARN_ON(ret);
	ret = register_trace_timer_init(probe_timer_init);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tITIMER_STATE_EVENTS");
	ret = register_trace_itimer_state(probe_itimer_state);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tTIMER_START_EVENTS");
	ret = register_trace_hrtimer_start(probe_hrtimer_start);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tSCHED_EXIT_EVENTS");
	ret = register_trace_sched_process_exit(probe_sched_process_exit);
	WARN_ON(ret);
    }

#else

    {
	OUTPUT(0, KERN_INFO "\tTIMER_INIT_EVENTS");
	ret = register_trace_hrtimer_init(probe_hrtimer_init, NULL);
	WARN_ON(ret);
	ret = register_trace_timer_init(probe_timer_init, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tITIMER_STATE_EVENTS");
	ret = register_trace_itimer_state(probe_itimer_state, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tTIMER_START_EVENTS");
	ret = register_trace_hrtimer_start(probe_hrtimer_start, NULL);
	WARN_ON(ret);
    }

    {
	OUTPUT(0, KERN_INFO "\tSCHED_EVENTS");
	ret = register_trace_sched_process_exit(probe_sched_process_exit, NULL);
	WARN_ON(ret);
    }

#endif // KERNEL_VER
    return SUCCESS;
};

static void unregister_timer_callstack_probes(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    {
	unregister_trace_hrtimer_init(probe_hrtimer_init);
	unregister_trace_timer_init(probe_timer_init);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_itimer_state(probe_itimer_state);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_hrtimer_start(probe_hrtimer_start);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_sched_process_exit(probe_sched_process_exit);

	tracepoint_synchronize_unregister();
    }

#else

    {
	unregister_trace_hrtimer_init(probe_hrtimer_init, NULL);
	unregister_trace_timer_init(probe_timer_init, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_itimer_state(probe_itimer_state, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_hrtimer_start(probe_hrtimer_start, NULL);

	tracepoint_synchronize_unregister();
    }

    {
	unregister_trace_sched_process_exit(probe_sched_process_exit, NULL);

	tracepoint_synchronize_unregister();
    }

#endif // KERNEL_VER
};

/*
 * Register all probes which should be registered
 * REGARDLESS OF COLLECTION STATUS.
 */
static int register_permanent_probes(void)
{
    if (probe_on_syscalls) {
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
        {
            WARN_ON(register_trace_sys_enter(probe_sys_enter));
            WARN_ON(register_trace_sys_exit(probe_sys_exit));
        }
#else // LINUX_VERSION
        {
            WARN_ON(register_trace_sys_enter(probe_sys_enter, NULL));
            WARN_ON(register_trace_sys_exit(probe_sys_exit, NULL));
        }
#endif // LINUX_VERSION
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT
    }
    return register_timer_callstack_probes();
};

static void unregister_permanent_probes(void)
{
    if (probe_on_syscalls) {
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
        {
            unregister_trace_sys_enter(probe_sys_enter);
            unregister_trace_sys_exit(probe_sys_exit);

            tracepoint_synchronize_unregister();
        }
#else // LINUX_VERSION
        {
            unregister_trace_sys_enter(probe_sys_enter, NULL);
            unregister_trace_sys_exit(probe_sys_exit, NULL);

            tracepoint_synchronize_unregister();
        }
#endif // LINUX_VERSION
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT
    }

    unregister_timer_callstack_probes();
};

/*
 * Register all probes which should be registered
 * ONLY FOR AN ONGOING, NON-PAUSED COLLECTION.
 */
static int register_non_pausable_probes(void)
{
    // timer expire
    // irq
    // tps
    // tpf
    int ret = 0;

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
            ret = register_trace_timer_expire_entry(probe_timer_expire_entry);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit);
            WARN_ON(ret);
            ret = register_trace_irq_handler_entry(probe_irq_handler_entry);
            WARN_ON(ret);
            ret = register_trace_softirq_entry(probe_softirq_entry);
            WARN_ON(ret);
            ret = register_trace_sched_wakeup(probe_sched_wakeup);
            WARN_ON(ret);
        }
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_power_start(probe_power_start);
            WARN_ON(ret);
        }
#ifdef __arm__
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_power_end(probe_power_end);
            WARN_ON(ret);
        }
#endif // __arm__
        {
            ret = register_trace_workqueue_execution(probe_workqueue_execution);
            WARN_ON(ret);
        }
    }


#else // KERNEL_VER

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            OUTPUT(0, KERN_INFO "\tTRACE_BREAK_EVENTS");
            ret = register_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit, NULL);
            WARN_ON(ret);
            ret = register_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_softirq_entry(probe_softirq_entry, NULL);
            WARN_ON(ret);
            ret = register_trace_sched_wakeup(probe_sched_wakeup, NULL);
            WARN_ON(ret);
        }
        /*
         * ONLY required for "SLEEP" mode i.e. C-STATES
         */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_power_start(probe_power_start, NULL);
            WARN_ON(ret);
        }
#ifdef __arm__
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_power_end(probe_power_end, NULL);
            WARN_ON(ret);
        }
#endif // __arm__
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
        {
            ret = register_trace_workqueue_execution(probe_workqueue_execution, NULL);
            WARN_ON(ret);
        }
#else // 2.6.36 <= version < 2.6.38
        {
            ret = register_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);
            WARN_ON(ret);
        }
#endif // version < 2.6.36
#else // version >= 2.6.38
        {
            OUTPUT(0, KERN_INFO "\tCSTATE_EVENTS");
            ret = register_trace_cpu_idle(probe_cpu_idle, NULL);
            WARN_ON(ret);
        }
#ifdef __arm__
#if TRACE_CPU_HOTPLUG
        {
            OUTPUT(0, KERN_INFO "\tCPU_ON_OFF_EVENTS");
            ret = register_trace_cpu_hotplug(probe_cpu_hotplug, NULL);
            WARN_ON(ret);
        }
#endif // TRACE_CPU_HOTPLUG
#endif // __arm__
        {
            ret = register_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);
            WARN_ON(ret);
        }
#endif // LINUX_VERSION_CODE < 2.6.38
    }

#endif // KERNEL_VER
    return SUCCESS;
};

static void unregister_non_pausable_probes(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    // #if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            unregister_trace_timer_expire_entry(probe_timer_expire_entry);
            unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry);
            unregister_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit);
            unregister_trace_irq_handler_entry(probe_irq_handler_entry);
            unregister_trace_softirq_entry(probe_softirq_entry);
            unregister_trace_sched_wakeup(probe_sched_wakeup);

            tracepoint_synchronize_unregister();
        }
        /*
         * ONLY required for "SLEEP" mode i.e. C-STATES
         */
        {
            unregister_trace_power_start(probe_power_start);
#ifdef __arm__
            unregister_trace_power_end(probe_power_end);
#endif // __arm__
            tracepoint_synchronize_unregister();
        }

        {
            unregister_trace_workqueue_execution(probe_workqueue_execution);

            tracepoint_synchronize_unregister();
        }
    }


#else // KERNEL_VER

    /*
     * ONLY required for "SLEEP" mode i.e. C-STATES
     */
    if(IS_SLEEP_MODE()){
        OUTPUT(0, KERN_INFO "SLEEP MODE REQUESTED\n");
        {
            unregister_trace_timer_expire_entry(probe_timer_expire_entry, NULL);
            unregister_trace_hrtimer_expire_entry(probe_hrtimer_expire_entry, NULL);
            unregister_trace_hrtimer_expire_exit(probe_hrtimer_expire_exit, NULL);
            unregister_trace_irq_handler_entry(probe_irq_handler_entry, NULL);
            unregister_trace_softirq_entry(probe_softirq_entry, NULL);
            unregister_trace_sched_wakeup(probe_sched_wakeup, NULL);

            tracepoint_synchronize_unregister();
        }
        /*
         * ONLY required for "SLEEP" mode i.e. C-STATES
         */
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,38)
        {
            unregister_trace_power_start(probe_power_start, NULL);
#ifdef __arm__
            unregister_trace_power_end(probe_power_end, NULL);
#endif // __arm__
            tracepoint_synchronize_unregister();
        }
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,36)
        {
            unregister_trace_workqueue_execution(probe_workqueue_execution, NULL);

            tracepoint_synchronize_unregister();
        }
#else // 2.6.36 <= version < 2.6.38
        {
            unregister_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);

            tracepoint_synchronize_unregister();
        }
#endif // version < 2.6.36
#else // version >= 2.6.38
        {
            unregister_trace_cpu_idle(probe_cpu_idle, NULL);

            tracepoint_synchronize_unregister();
        }
#ifdef __arm__
#if TRACE_CPU_HOTPLUG
        {
            unregister_trace_cpu_hotplug(probe_cpu_hotplug, NULL);

            tracepoint_synchronize_unregister();
        }
#endif // TRACE_CPU_HOTPLUG
#endif // __arm__
        {
            unregister_trace_workqueue_execute_start(probe_workqueue_execute_start, NULL);

            tracepoint_synchronize_unregister();
        }
#endif // LINUX_VERSION_CODE < 2.6.38
    }

#endif // KERNEL_VER
};

/*
 * Register all probes which must be registered
 * ONLY FOR AN ONGOING (i.e. START/PAUSED) COLLECTION.
 */
static int register_pausable_probes(void)
{
    int ret = 0;
    // sys_exit
    // sched_fork
    // module_notifier
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)
    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tMOD_NOTIFIER_EVENTS");
        register_module_notifier(&apwr_mod_notifier_block);
    }
    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tSCHED_FORK_EVENTS");
	ret = register_trace_sched_process_fork(probe_sched_process_fork);
	WARN_ON(ret);
    }

    /*
     * ALWAYS required.
     */
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
    
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT

    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_register_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    ret = register_trace_power_frequency(probe_power_frequency);
	    WARN_ON(ret);
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
        OUTPUT(0, KERN_INFO "\tWAKELOCK_EVENTS");
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        ret = register_trace_wake_lock(probe_wake_lock);
#else
        ret = register_trace_wakeup_source_activate(probe_wakeup_source_activate);
#endif
        WARN_ON(ret);

        OUTPUT(0, KERN_INFO "\tWAKEUNLOCK_EVENTS");
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        ret = register_trace_wake_unlock(probe_wake_unlock);
#else
        ret = register_trace_wakeup_source_deactivate(probe_wakeup_source_deactivate);
#endif
        WARN_ON(ret);
#endif
    }

#else // KERNEL_VER

    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tMOD_NOTIFIER_EVENTS");
        register_module_notifier(&apwr_mod_notifier_block);
    }

    /*
     * ALWAYS required.
     */
    {
	OUTPUT(0, KERN_INFO "\tSCHED_FORK_EVENTS");
	ret = register_trace_sched_process_fork(probe_sched_process_fork, NULL);
	WARN_ON(ret);
    }

    /*
     * ALWAYS required.
     */
#if DO_PROBE_ON_SYSCALL_ENTER_EXIT
    
#endif // DO_PROBE_ON_SYSCALL_ENTER_EXIT

    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED!\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_register_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    ret = register_trace_power_frequency(probe_power_frequency, NULL);
	    WARN_ON(ret);
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
        OUTPUT(0, KERN_INFO "\tWAKELOCK_EVENTS");
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        ret = register_trace_wake_lock(probe_wake_lock, NULL);
#else
        ret = register_trace_wakeup_source_activate(probe_wakeup_source_activate, NULL);
#endif
        WARN_ON(ret);

        OUTPUT(0, KERN_INFO "\tWAKEUNLOCK_EVENTS");
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        ret = register_trace_wake_unlock(probe_wake_unlock, NULL);
#else
        ret = register_trace_wakeup_source_deactivate(probe_wakeup_source_deactivate, NULL);
#endif
        WARN_ON(ret);
#endif
    }

#endif // KERNEL_VER

    return SUCCESS;
};

static void unregister_pausable_probes(void)
{
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35)

    /*
     * ALWAYS required.
     */
    {
        unregister_module_notifier(&apwr_mod_notifier_block);
    }

    /*
     * ALWAYS required.
     */
    {
	unregister_trace_sched_process_fork(probe_sched_process_fork);

	tracepoint_synchronize_unregister();
    }

    /*
     * ALWAYS required.
     */
    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED!\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_unregister_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    unregister_trace_power_frequency(probe_power_frequency);

	    tracepoint_synchronize_unregister();
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        unregister_trace_wake_lock(probe_wake_lock);
        unregister_trace_wake_unlock(probe_wake_unlock);
#else
        unregister_trace_wakeup_source_activate(probe_wakeup_source_activate);
        unregister_trace_wakeup_source_deactivate(probe_wakeup_source_deactivate);
#endif

        tracepoint_synchronize_unregister();
#endif
    }

#else // KERNEL_VER

    /*
     * ALWAYS required.
     */
    {
        unregister_module_notifier(&apwr_mod_notifier_block);
    }

    /*
     * ALWAYS required.
     */
    {
	unregister_trace_sched_process_fork(probe_sched_process_fork, NULL);

	tracepoint_synchronize_unregister();
    }

    /*
     * ALWAYS required.
     */

    /*
     * ONLY required for "FREQ" mode i.e. P-STATES
     */
    if(IS_FREQ_MODE()){
	OUTPUT(0, KERN_INFO "FREQ MODE REQUESTED\n");
#if DO_CPUFREQ_NOTIFIER
	{
	    OUTPUT(0, KERN_INFO "\tPSTATE_EVENTS\n");
	    cpufreq_unregister_notifier(&apwr_cpufreq_notifier_block, CPUFREQ_TRANSITION_NOTIFIER);
	}
#else // DO_CPUFREQ_NOTIFIER
	{
	    unregister_trace_power_frequency(probe_power_frequency, NULL);

	    tracepoint_synchronize_unregister();
	}
#endif // DO_CPUFREQ_NOTIFIER
    }

    if(IS_WAKELOCK_MODE()){
#if DO_WAKELOCK_SAMPLE
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,4,0)
        unregister_trace_wake_lock(probe_wake_lock, NULL);
        unregister_trace_wake_unlock(probe_wake_unlock, NULL);
#else
        unregister_trace_wakeup_source_activate(probe_wakeup_source_activate, NULL);
        unregister_trace_wakeup_source_deactivate(probe_wakeup_source_deactivate, NULL);
#endif

        tracepoint_synchronize_unregister();
#endif
    }

#endif // KERNEL_VER
};

/*
 * Service a "read(...)" call from user-space.
 * 
 * Returns sample information back to the user.
 * When a user calls the "read" function, the device
 * driver first checks if any (per-cpu) output buffers are full.
 * If yes, then the entire contents of that buffer are
 * copied to the user. If not, then the user blocks, until the
 * buffer-full condition is met.
 */
static ssize_t pw_device_read(struct file *file, char __user *buffer, size_t length, loff_t *offset)
{
    u32 val = 0;
    bool is_flush_mode = INTERNAL_STATE.drain_buffers;

#if 0
    while (pw_any_seg_full(&val, is_flush_mode) == false) {
        if (val == PW_ALL_WRITES_DONE_MASK) {
            BUG_ON(IS_COLLECTING());
            return 0; // "0" ==> EOF
        }
        val = PW_ALL_WRITES_DONE_MASK;
        if (wait_event_interruptible(pw_reader_queue, ( (!IS_COLLECTING() && !IS_SLEEPING()) || pw_any_seg_full(&val, false /* is flush mode */)))) {
            pw_pr_error("wait_event_interruptible error\n");
            return -ERESTARTSYS;
        }
        /*
         * OK, we were woken up. This can be because we have a full buffer or
         * because a 'STOP/CANCEL' cmd was issued. In the first case, we will have a valid
         * value for 'val' so check for that here.
         */
        if (val != PW_ALL_WRITES_DONE_MASK) {
            // we have a full buffer to return
            break;
        }
        /*
         * No full buffer exists; we may have been woken up because of a 'STOP' cmd. Loop
         * back and check.
         */
        // is_flush_mode = !IS_COLLECTING() && !IS_SLEEPING();
        is_flush_mode = INTERNAL_STATE.drain_buffers;
    }
#else // if 1
    do {
        val = PW_ALL_WRITES_DONE_MASK; is_flush_mode = INTERNAL_STATE.drain_buffers;
        pw_pr_debug(KERN_INFO "Waiting, flush = %s\n", GET_BOOL_STRING(is_flush_mode));
        if (wait_event_interruptible(pw_reader_queue, (pw_any_seg_full(&val, &INTERNAL_STATE.drain_buffers) || (!IS_COLLECTING() && !IS_SLEEPING())))) {
            pw_pr_error("wait_event_interruptible error\n");
            return -ERESTARTSYS;
        }
        pw_pr_debug(KERN_INFO "After wait: val = %u\n", val);
    } while (val == PW_NO_DATA_AVAIL_MASK);
#endif // if 0
    /*
     * Are we done producing/consuming?
     */
    if (val == PW_ALL_WRITES_DONE_MASK) {
        return 0; // "0" ==> EOF
    }
    /*
     * 'mmap' unsupported, for now
     */
    if (false && pw_did_mmap) {
        if (put_user(val, (u32 *)buffer)) {
            pw_pr_error("ERROR in put_user\n");
            return -ERROR;
        }
        return sizeof(val); // 'read' returns # of bytes actually read
    } else {
        /*
         * Copy the buffer contents into userspace.
         */
        size_t bytes_read = 0;
        unsigned long bytes_not_copied = pw_consume_data(val, buffer, length, &bytes_read); // 'read' returns # of bytes actually read
        pw_pr_debug(KERN_INFO "OK: returning %d\n", bytes_read);
        if (unlikely(bytes_not_copied)) {
            return -ERROR;
        }
        return bytes_read;
    }
};

static unsigned int pw_device_poll(struct file *filp, poll_table *wait)
{
    unsigned int mask = 0;
    u32 dummy = 0;

    poll_wait(filp, &pw_reader_queue, wait);

    if (!IS_COLLECTING() || pw_any_seg_full(&dummy, &INTERNAL_STATE.drain_buffers)) { // device is readable if: (a) NOT collecting or (b) any buffers is full
	mask = (POLLIN | POLLRDNORM);
    }

    return mask;
};

/*
 * 'mmap' unsupported, for now
 */
static int pw_device_mmap(struct file *filp, struct vm_area_struct *vma)
{
    long length = vma->vm_end - vma->vm_start;
    unsigned long total_size = 0;

    pw_pr_debug("MMAP received!\n");

    if (true) {
        return -ERROR;
    }

    /*
     * Check size restrictions.
     */
    if (length != pw_buffer_alloc_size) {
        pw_pr_error("ERROR: requested mapping size %ld bytes, MUST be %lu\n", length, pw_buffer_alloc_size);
        return -ERROR;
    }

    if (pw_map_per_cpu_buffers(vma, &total_size)) {
        pw_pr_error("ERROR mapping per-cpu buffers to userspace!\n");
        return -ERROR;
    }

    /*
     * Sanity!
     */
    if (total_size != length) {
        pw_pr_warn("WARNING: mmap: total size = %lu, length = %lu\n", total_size, length);
    } else {
        pw_pr_debug("OK: mmap total size = %lu, length = %lu\n", total_size, length);
    }

    pw_did_mmap = true;

    return SUCCESS;
};


// "copy_from_user" ==> dst, src
#define EXTRACT_LOCAL_ARGS(l,u) copy_from_user((l), (u), sizeof(struct PWCollector_ioctl_arg))

/*
 * Check if command is valid, given current state.
 */
static inline bool is_cmd_valid(PWCollector_cmd_t cmd)
{
    bool is_collecting = IS_COLLECTING(), is_sleeping = IS_SLEEPING();

    if(is_sleeping){
	/*
	 * If currently PAUSEd, the ONLY command
	 * that's NOT allowed is a subsequent PAUSE.
	 */
	if(cmd == PW_PAUSE)
	    return false;
    }
    else if(is_collecting && (cmd == PW_START || cmd == PW_RESUME))
    	return false;
    else if(!is_collecting && (cmd == PW_STOP || cmd == PW_PAUSE || cmd == PW_CANCEL))
    	return false;

    return true;
};

/*
 * Get list of available frequencies
 */
void get_frequency_steps(void)
{
    /*
     * Try and determine the 'legal' frequencies
     * i.e. the various (discrete) frequencies
     * the processor could possibly execute at.
     * This must be done ONCE PER COLLECTION (and
     * only at the START of the collection)!
     * Update: do this ONLY if we're computing
     * frequencies dynamically!
     */
    int cpu = 0;
    struct cpufreq_policy *policy;
    ssize_t buffer_len = -1;
    struct freq_attr *freq_attrs = &cpufreq_freq_attr_scaling_available_freqs;
    char buffer[512]; // size is probably overkill -- but we can't be sure how many freq states there are!
    /*
     * (1) Get the current CPUFREQ policy...
     */
    if( (policy = cpufreq_cpu_get(cpu)) == NULL){
	OUTPUT(0, KERN_INFO "WARNING: cpufreq_cpu_get error for CPU = %d\n", cpu);
    }
    else{
	/*
	 * (2) Get the (string) representation of the
	 * various frequencies. The string contains
	 * a number of (space-separated) frequencies (in KHz).
	 */
	if( (buffer_len = freq_attrs->show(policy, buffer)) == -ENODEV){
	    OUTPUT(0, KERN_INFO "WARNING: cpufreq_attrs->show(...) error for CPU = %d\n", cpu);
	}else{
            if (buffer_len >= 512) {
                pw_pr_error("Error: buffer_len (%d) >= 512\n", buffer_len);
                return;
            }
	    /*
	     * (3) Tokenize the string to extract the
	     * actual frequencies. At this point
	     * we can confidently state we've found
	     * the frequencies and we don't need to
	     * repeat this procedure.
	     */
	    OUTPUT(0, KERN_INFO "[%d]: buffer_len = %d, BUFFER = %s\n", cpu, (int)buffer_len, buffer);
	    if (extract_valid_frequencies(buffer, buffer_len)) {
		pw_pr_error("ERROR: could NOT determine frequency table!\n");
	    }
	}
        cpufreq_cpu_put(policy);
    }
};


/*
 * Retrieve the base operating frequency
 * for this CPU. The base frequency acts
 * as a THRESHOLD indicator for TURBO -- frequencies
 * ABOVE this are considered TURBO.
 */
static inline void get_base_operating_frequency(void)
{
#ifndef __arm__
    u64 res;
    u16 ratio = 0;

    if(!INTERNAL_STATE.bus_clock_freq_khz){
        pw_pr_error("ERROR: cannot set base_operating_frequency until we have a bus clock frequency!\n");
        return;
    }

    /*
     * Algo:
     * (1) If NHM/WMR/SNB -- read bits 15:8 of 'PLATFORM_INFO_MSR_ADDR'
     * (2) If MFLD/ATM -- read bits 44:40 of 'CLOCK_CR_GEYSIII_STAT' MSR
     * to extract the 'base operating ratio'.
     * To get actual TSC frequency, multiply this ratio
     * with the bus clock frequency.
     */
    if(pw_is_atm){
	rdmsrl(CLOCK_CR_GEYSIII_STAT_MSR_ADDR, res);
	/*
	 * Base operating Freq ratio is
	 * bits 44:40
	 */
	ratio = (res >> 40) & 0x1f;
    }else{
	rdmsrl(PLATFORM_INFO_MSR_ADDR, res);
	/*
	 * Base Operating Freq ratio is
	 * bits 15:8
	 */
	ratio = (res >> 8) & 0xff;
    }

    // base_operating_freq = ratio * BUS_CLOCK_FREQ_KHZ;
    base_operating_freq_khz = ratio * INTERNAL_STATE.bus_clock_freq_khz;
    OUTPUT(3, KERN_INFO "RATIO = 0x%x, BUS_FREQ = %u, FREQ = %u, (RES = %llu)\n", (u32)ratio, (u32)INTERNAL_STATE.bus_clock_freq_khz, base_operating_freq_khz, res);
#else
    struct cpufreq_policy *policy;
    if( (policy = cpufreq_cpu_get(0)) == NULL){
        base_operating_freq_khz = policy->max;
    }
#endif // ifndef __arm__
};

/*
 * Set initial config params.
 * These include MSR addresses, and power
 * collection switches.
 */
int set_config(struct PWCollector_config *remote_config, int size)
{
    int i=0;
    struct PWCollector_config local_config;

    if( (i = copy_from_user(&local_config, remote_config, sizeof(local_config) /*size*/))) // "copy_from_user" returns number of bytes that COULD NOT be copied
	return i;
    /*
     * Copy Core/Pkg MSR addresses
     */
    memcpy(INTERNAL_STATE.coreResidencyMSRAddresses, local_config.info.coreResidencyMSRAddresses, sizeof(int) * MAX_MSR_ADDRESSES);
    memcpy(INTERNAL_STATE.pkgResidencyMSRAddresses, local_config.info.pkgResidencyMSRAddresses, sizeof(int) * MAX_MSR_ADDRESSES);

    if(true){
	    printk(KERN_INFO "CORE addrs...\n");
	    for(i=0; i<MAX_MSR_ADDRESSES; ++i) {
		    printk(KERN_INFO "C%d: %d\n", i, INTERNAL_STATE.coreResidencyMSRAddresses[i]);
	    }
	    printk(KERN_INFO "PKG addrs...\n");
	    for(i=0; i<MAX_MSR_ADDRESSES; ++i) {
		    printk(KERN_INFO "C%d: %d\n", i, INTERNAL_STATE.pkgResidencyMSRAddresses[i]);
	    }
    }
    /*
     * Set C-state clock multiplier.
     */
    INTERNAL_STATE.residency_count_multiplier = local_config.info.residency_count_multiplier;

    /*
     * Make sure we've got a valid multiplier!
     */
    if((int)INTERNAL_STATE.residency_count_multiplier <= 0)
	INTERNAL_STATE.residency_count_multiplier = 1;

    if(true){
	OUTPUT(0, KERN_INFO "DEBUG: C-state clock multiplier = %u\n", INTERNAL_STATE.residency_count_multiplier);
    }

    /*
     * Set bus clock frequency -- required for
     * Turbo threshold determination / calculation.
     */
    INTERNAL_STATE.bus_clock_freq_khz = local_config.info.bus_clock_freq_khz;
    /*
     * Check if we've got a valid bus clock frequency -- default to
     * BUS_CLOCK_FREQ_KHZ if not.
     */
    if((int)INTERNAL_STATE.bus_clock_freq_khz <= 0)
	// INTERNAL_STATE.bus_clock_freq_khz = BUS_CLOCK_FREQ_KHZ;
	INTERNAL_STATE.bus_clock_freq_khz = DEFAULT_BUS_CLOCK_FREQ_KHZ();

    OUTPUT(0, KERN_INFO "DEBUG: Bus clock frequency = %u KHz\n", INTERNAL_STATE.bus_clock_freq_khz);

    /*
     * The base operating frequency requires the
     * bus frequency -- set it here.
     */
    get_base_operating_frequency();
    /*
     * Get a list of frequencies that
     * the processors may execute at.
     * (But only if the steps haven't previously
     * been determined).
     */
    if (!apwr_available_frequencies) {
        get_frequency_steps();
    }

    /*
     * Set power switches.
     */
    INTERNAL_STATE.collection_switches = local_config.data;
    printk(KERN_INFO "\tCONFIG collection switches = %d\n", INTERNAL_STATE.collection_switches);

    INTERNAL_STATE.d_state_sample_interval = local_config.d_state_sample_interval;
    OUTPUT(0, KERN_INFO "\tCONFIG D-state collection interval (msec) = %d\n", INTERNAL_STATE.d_state_sample_interval);

    return SUCCESS;
};

int check_platform(struct PWCollector_check_platform *remote_check, int size)
{
    struct PWCollector_check_platform *local_check;
    const char *unsupported = "UNSUPPORTED_T1, UNSUPPORTED_T2"; // for debugging ONLY
    int len = strlen(unsupported);
    int max_size = sizeof(struct PWCollector_check_platform);
    int retVal = SUCCESS;

    local_check = pw_kmalloc(max_size, GFP_KERNEL);

    if(!local_check){
	pw_pr_error("ERROR: could NOT allocate memory in check_platform!\n");
	return -ERROR;
    }

    memset(local_check, 0, max_size);

    /*
     * Populate "local_check.unsupported_tracepoints" with a (comma-separated)
     * list of unsupported tracepoints. For now, we just leave this
     * blank, reflecting the fact that, on our development systems,
     * every tracepoints is supported.
     *
     * Update: for debugging, write random data here.
     */
    memcpy(local_check->unsupported_tracepoints, unsupported, len);
    /*
     * UPDATE: we're borrowing one of the 'reserved' 64bit values
     * to document the following:
     * (1) Kernel call stacks supported?
     * (2) Kernel compiled with CONFIG_TIMER_STATS?
     * (3) Wakelocks supported?
     */
#ifdef CONFIG_FRAME_POINTER
    local_check->supported_kernel_features |= PW_KERNEL_SUPPORTS_CALL_STACKS;
#endif
#ifdef CONFIG_TIMER_STATS
    local_check->supported_kernel_features |= PW_KERNEL_SUPPORTS_CONFIG_TIMER_STATS;
#endif
#if DO_WAKELOCK_SAMPLE
    local_check->supported_kernel_features |= PW_KERNEL_SUPPORTS_WAKELOCK_PATCH;
#endif
    /*
     * Also update information on the underlying CPU:
     * (1) Was the 'ANY-THREAD' bit set?
     * (2) Was 'Auto-Demote' enabled?
     */
    if (pw_is_any_thread_set) {
        local_check->supported_arch_features |= PW_ARCH_ANY_THREAD_SET;
    }
    if (pw_is_auto_demote_enabled) {
        local_check->supported_arch_features |= PW_ARCH_AUTO_DEMOTE_ENABLED;
    }

    /*
     * Copy everything back to user address space.
     */
    if( (retVal = copy_to_user(remote_check, local_check, size))) // returns number of bytes that COULD NOT be copied
	retVal = -ERROR;

    pw_kfree(local_check);
    return retVal; // all unsupported tracepoints documented
};

/*
 * Return the TURBO frequency threshold
 * for this CPU.
 */
int get_turbo_threshold(struct PWCollector_turbo_threshold *remote_thresh, int size)
{
    struct PWCollector_turbo_threshold local_thresh;

    if (!base_operating_freq_khz) {
        pw_pr_error("ERROR: retrieving turbo threshold without specifying base operating freq?!\n");
	return -ERROR;
    }

    local_thresh.threshold_frequency = base_operating_freq_khz;

    if(copy_to_user(remote_thresh, &local_thresh, size)) // returns number of bytes that could NOT be copied.
	return -ERROR;

    return SUCCESS;
};

/*
 * Retrieve device driver version
 */
int get_version(struct PWCollector_version_info *remote_version, int size)
{
    struct PWCollector_version_info local_version;

    local_version.version = PW_VERSION_VERSION;
    local_version.inter = PW_VERSION_INTERFACE;
    local_version.other = PW_VERSION_OTHER;

    /*
     * Copy everything back to user address space.
     */
    return copy_to_user(remote_version, &local_version, size); // returns number of bytes that could NOT be copiled
};

/*
 * Retrieve microcode patch version.
 * Only useful for MFLD
 */
int get_micro_patch_ver(int *remote_ver, int size)
{
    int local_ver = micro_patch_ver;

    /*
     * Copy everything back to user address space.
     */
    return copy_to_user(remote_ver, &local_ver, size); // returns number of bytes that could NOT be copiled
};

/*
 * Retrieve available device list
 */
int get_available_devices(struct PWCollector_device_info *remote_devices, int size)
{
    struct PWCollector_device_info local_devices;

    local_devices.undefined_devices = undefined_device_list;

    /*
     * Copy everything back to user address space.
     */
    return copy_to_user(remote_devices, &local_devices, size); // returns number of bytes that could NOT be copiled
};

int get_status(struct PWCollector_status *remote_status, int size)
{
    struct PWCollector_status local_status;
    int cpu, retVal = SUCCESS;
    stats_t *pstats = NULL;
    unsigned long statusJIFF, elapsedJIFF = 0;

    memset(&local_status, 0, sizeof(local_status));

    /*
     * Set # cpus.
     */
    // local_status.num_cpus = pw_max_num_cpus;
    local_status.num_cpus = num_online_cpus();

    /*
     * Set total collection time elapsed.
     */
    {
	statusJIFF = jiffies;
	if(statusJIFF < INTERNAL_STATE.collectionStartJIFF){
	    OUTPUT(0, KERN_INFO "WARNING: jiffies counter has WRAPPED AROUND!\n");
	    elapsedJIFF = 0; // avoid messy NAN when dividing
	}else{
	    // elapsedJIFF = statusJIFF - startJIFF;
	    elapsedJIFF = statusJIFF - INTERNAL_STATE.collectionStartJIFF;
	}
	OUTPUT(0, KERN_INFO "start = %lu, stop = %lu, elapsed = %lu\n", INTERNAL_STATE.collectionStartJIFF, statusJIFF, elapsedJIFF);
    }
    local_status.time = jiffies_to_msecs(elapsedJIFF);

    /*
     * Set # c-breaks etc.
     * Note: aggregated over ALL cpus,
     * per spec document.
     */
    for_each_online_cpu(cpu){
	pstats = &per_cpu(per_cpu_stats, cpu);
	local_status.c_breaks += local_read(&pstats->c_breaks);
	local_status.timer_c_breaks += local_read(&pstats->timer_c_breaks);
	local_status.inters_c_breaks += local_read(&pstats->inters_c_breaks);
	local_status.p_trans += local_read(&pstats->p_trans);
	local_status.num_inters += local_read(&pstats->num_inters);
	local_status.num_timers += local_read(&pstats->num_timers);
    }

    /*
     * Now copy everything to user-space.
     */
    retVal = copy_to_user(remote_status, &local_status, sizeof(local_status)); // returns number of bytes that COULD NOT be copied

    return retVal;
};

long get_available_frequencies(struct PWCollector_available_frequencies *remote_freqs, int size)
{
    int i=0;
    struct PWCollector_available_frequencies local_freqs;

    if(!apwr_available_frequencies){
	pw_pr_error("ERROR: trying to get list of available frequencies WITHOUT setting config?!\n");
	return -ERROR;
    }

    memset(local_freqs.frequencies, 0, sizeof(u32) * PW_MAX_NUM_AVAILABLE_FREQUENCIES);

    for(i = 0, local_freqs.num_freqs = 0; apwr_available_frequencies[i] != 0; ++i, ++local_freqs.num_freqs){
	local_freqs.frequencies[i] = apwr_available_frequencies[i];
    }

    if(copy_to_user(remote_freqs, &local_freqs, size)) // returns number of bytes that could NOT be copied.
	return -ERROR;

    return SUCCESS;
};

/*
 * Reset all statistics collected so far.
 * Called from a non-running collection context.
 */
static inline void reset_statistics(void)
{
    int cpu;
    stats_t *pstats = NULL;

    /*
     * Note: no need to lock, since we're only
     * going to be called from a non-running
     * collection, and tracepoints are inserted
     * (just) before a collection starts, and removed
     * (just) after a collection ends.
     */
    for_each_online_cpu(cpu){
	/*
	 * Reset the per cpu stats
	 */
	{
	    pstats = &per_cpu(per_cpu_stats, cpu);
	    local_set(&pstats->c_breaks, 0);
	    local_set(&pstats->timer_c_breaks, 0);
	    local_set(&pstats->inters_c_breaks, 0);
	    local_set(&pstats->p_trans, 0);
	    local_set(&pstats->num_inters, 0);
	    local_set(&pstats->num_timers, 0);
	}
    }
};

/*
 * Reset the (PER-CPU) structs containing
 * MSR residency information (amongst
 * other fields).
 */
void reset_per_cpu_msr_residencies(void)
{
    int cpu;

    for_each_online_cpu(cpu) {
        /*
         * Reset the per-cpu residencies
         */
        *(&per_cpu(prev_c6_val, cpu)) = 0;
        /*
         * Reset the "first-hit" variable.
         */
        // local_set(&per_cpu(is_first_event, cpu), 1);
        per_cpu(wakeup_event_counter, cpu).event_tsc = 0;
        /*
         * Reset the 'init_msr_sent' variable.
         */
        memset((&per_cpu(pw_pcpu_msr_sets, cpu)), 0, sizeof(msr_set_t));
        /*
         * Reset stats on # samples produced and # dropped.
         */
        local_set(&per_cpu(num_samples_produced, cpu), 0);
        local_set(&per_cpu(num_samples_dropped, cpu), 0);
    }
    /*
     * Reset the TPS atomic count value.
     */
#if DO_TPS_EPOCH_COUNTER
    atomic_set(&tps_epoch, 0);
#endif
    /*
     * Ensure updates are propagated.
     */
    smp_mb();
};


static void reset_trace_sent_fields(void)
{
    struct tnode *node = NULL;
    struct hlist_node *curr = NULL;
    int i=0;

    for(i=0; i<NUM_MAP_BUCKETS; ++i)
	PW_HLIST_FOR_EACH_ENTRY(node, curr, &timer_map[i].head, list){
	    node->trace_sent = 0;
	}
};


/*
 * Run the generation of current p-state sample
 * for all cpus in parallel to avoid the delay 
 * due to a serial execution.
 */
static void generate_cpu_frequency_per_cpu(int cpu, bool is_start)
{
    u64 tsc = 0, aperf = 0, mperf = 0;
    u32 perf_status = 0;
    u8 is_boundary = (is_start) ? 1 : 2; // "0" ==> NOT boundary, "1" ==> START boundary, "2" ==> STOP boundary
    u32 prev_req_freq = 0;

#ifndef __arm__
    u32 l=0, h=0;
    {
        int ret = rdmsr_safe_on_cpu(cpu, 0x10, &l, &h);
        if(ret){
            OUTPUT(0, KERN_INFO "WARNING: rdmsr of TSC failed with code %d\n", ret);
        }
        tsc = h;
        tsc <<= 32;
        tsc += l;
    }

    /*
     * Read the IA32_PERF_STATUS MSR. We delegate the actual frequency computation to Ring-3 because
     * the PERF_STATUS encoding is actually model-specific.
     */
    {
        WARN_ON(rdmsr_safe_on_cpu(cpu, IA32_PERF_STATUS_MSR_ADDR, &l, &h));
        perf_status = l; // We're only interested in the lower 16 bits!
    }


    /*
     * Retrieve the previous requested frequency.
     */
    if (is_start == false) {
        prev_req_freq = per_cpu(pcpu_prev_req_freq, cpu);
    } else {
        /*
         * Collection START: make sure we reset the requested frequency!
         */
        per_cpu(pcpu_prev_req_freq, cpu) = 0;
    }
    // per_cpu(pcpu_prev_req_freq, cpu) = perf_status;
    /*
     * Also read CPU_CLK_UNHALTED.REF and CPU_CLK_UNHALTED.CORE. These required ONLY for AXE import
     * backward compatibility!
     */
#if 1
    {
        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[APERF], &l, &h));
        aperf = (u64)h << 32 | (u64)l;

        WARN_ON(rdmsr_safe_on_cpu(cpu, INTERNAL_STATE.coreResidencyMSRAddresses[MPERF], &l, &h));
        mperf = (u64)h << 32 | (u64)l;
    }
#endif

#else
    msr_set_t *set = NULL;
    set = &get_cpu_var(pw_pcpu_msr_sets);

    tscval(&tsc);
    aperf = set->prev_msr_vals[APERF];
    mperf = set->prev_msr_vals[MPERF];
    put_cpu_var(pw_pcpu_msr_sets);

    perf_status = cpufreq_quick_get(cpu) / INTERNAL_STATE.bus_clock_freq_khz;

    /*
     * Retrieve the previous requested frequency.
     */
    // TODO CAN WE MERGE THIS CODE WITH ifndef __arm__ code above and put it
    // after this code?
    if (is_start == false) {
        prev_req_freq = per_cpu(pcpu_prev_req_freq, cpu);
    }
#endif // ifndef __arm__

    produce_p_sample(cpu, tsc, prev_req_freq, perf_status, is_boundary, aperf, mperf);

};

static void generate_cpu_frequency(void *start)
{
    int cpu = raw_smp_processor_id();
    bool is_start = *((bool *)start);

    generate_cpu_frequency_per_cpu(cpu, is_start);
}

/*
 * Measure current CPU operating
 * frequency, and push 'P-samples'
 * onto the (per-cpu) O/P buffers.
 * Also determine the various
 * discrete frequencies the processor
 * is allowed to execute at (basically
 * the various frequencies present
 * in the 'scaling_available_frequencies'
 * sysfs file).
 *
 * REQUIRES CPUFREQ DRIVER!!!
 */
static void get_current_cpu_frequency(bool is_start)
{

#if DO_GENERATE_CURRENT_FREQ_IN_PARALLEL
    SMP_CALL_FUNCTION(&generate_cpu_frequency, (void *)&is_start, 0, 1);
    // smp_call_function is executed for all other CPUs except itself. 
    // So, run it for itself.
    generate_cpu_frequency((void *)&is_start);
#else
    int cpu = 0;
    for_each_online_cpu(cpu){
        generate_cpu_frequency_per_cpu(cpu, is_start);
    }
#endif
};

static void generate_end_tps_sample_per_cpu(void *tsc)
{
    int cpu = raw_smp_processor_id();
    PWC_tps_msg_t c_msg;

    /*
     * UPDATE: do this ONLY if we've sent at least one tps sample in the past!
     */
    if (unlikely(__get_cpu_var(pw_pcpu_msr_sets).init_msr_set_sent == 0x0)) {
        return;
    }

    memset(&c_msg, 0, sizeof(c_msg));

    c_msg.cpuidx = cpu;
    c_msg.tsc = *((u64 *)tsc);
    c_msg.data_type = C_STATE;
    c_msg.data_len = sizeof(c_msg.data);

    DO_PER_CPU_OVERHEAD_FUNC(pw_produce_tps_msg, &c_msg);
};

static void generate_end_tps_samples(void)
{
    u64 tsc = 0;

    tscval(&tsc);

    SMP_CALL_FUNCTION(&generate_end_tps_sample_per_cpu, (void *)&tsc, 0, 1);
    generate_end_tps_sample_per_cpu((void *)&tsc);
};

/*
 * START/RESUME a collection.
 *
 * (a) (For START ONLY): ZERO out all (per-cpu) O/P buffers.
 * (b) Reset all statistics.
 * (c) Register all tracepoints.
 */
int start_collection(PWCollector_cmd_t cmd)
{
    switch(cmd){
    case PW_START:
	/*
	 * Reset the O/P buffers.
	 *
	 * START ONLY
	 */
        pw_reset_per_cpu_buffers();
	/*
	 * Reset the 'trace_sent' fields
	 * for all trace entries -- this
	 * ensures we send backtraces
	 * once per collection, as
	 * opposed to once per 'insmod'.
	 *
	 * START ONLY
	 */
	{
	    reset_trace_sent_fields();
	}

#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
	/*
	 * Reset the list of vars required
	 * to transfer IRQ # <-> Name info.
	 * UPDATE: the 'irq_map' should contain
	 * mappings for only those
	 * devices that actually caused C-state
	 * wakeups DURING THE CURRENT COLLECTION.
	 * We therefore reset the map before
	 * every collection (this also auto resets
	 * the "irq_mappings_list" data structure).
	 *
	 * START ONLY
	 */
	{
	    destroy_irq_map();
	    if(init_irq_map()){
		// ERROR
		pw_pr_error("ERROR: could NOT initialize irq map in start_collection!\n");
		return -ERROR;
	    }
	}
#endif

#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES
        {
            destroy_wlock_map();
            if (init_wlock_map()) {
                pw_pr_error("ERROR: could NOT initialize wlock map in start_collection1\n");
                return -ERROR;
            }
        }
#endif
	/*
	 * Reset collection stats
	 *
	 * START ONLY
	 */
#if DO_IOCTL_STATS
	{
	    reset_statistics();
	}
#endif

    case PW_RESUME: // fall through
	break;
    default: // should *NEVER* happen!
	printk(KERN_ERR "Error: invalid cmd=%d in start collection!\n", cmd);
	return -ERROR;
    }
    /*
     * Reset the (per-cpu) "per_cpu_t" structs that hold MSR residencies
     *
     * START + RESUME
     */
    {
	reset_per_cpu_msr_residencies();
    }

    /*
     * Get START P-state samples.
     *
     * UPDATE: do this ONLY IF
     * USER SPECIFIES FREQ-mode!
     *
     * START + RESUME???
     */
    if(likely(IS_FREQ_MODE())){
	get_current_cpu_frequency(true); // "true" ==> collection START
    }

    /*
     * Take a snapshot of the TSC on collection start -- required for ACPI S3 support.
     */
    tscval(&pw_collection_start_tsc);

    INTERNAL_STATE.collectionStartJIFF = jiffies;
    INTERNAL_STATE.write_to_buffers = true;

    /*
     * OK, all setup completed. Now
     * register the tracepoints.
     */
    switch(cmd){
    case PW_START:
	register_pausable_probes();
    case PW_RESUME: // fall through
	register_non_pausable_probes();
	break;
    default: // should *NEVER* happen!
	printk(KERN_ERR "Error: invalid cmd=%d in start collection!\n", cmd);
	return -ERROR;
    }

#if DO_S_RESIDENCY_SAMPLE 
    //struct timeval cur_time;
    if(pw_is_atm && IS_S_RESIDENCY_MODE()){
	    start_s_residency_counter();
            startTSC_s_residency = 0;
            //do_gettimeofday(cur_time); 
            produce_s_residency_sample(0);
            startJIFF_s_residency = CURRENT_TIME_IN_USEC();
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE 
    if(pw_is_atm && IS_D_SC_RESIDENCY_MODE()){
	    start_d_sc_residency_counter();
            startJIFF_d_sc_residency = CURRENT_TIME_IN_USEC();
            prev_sample_usec = CURRENT_TIME_IN_USEC();
            produce_d_sc_residency_sample(0);
    }
#endif

    return SUCCESS;
};

/*
 * STOP/PAUSE/CANCEL a (running) collection.
 *
 * (a) Unregister all tracepoints.
 * (b) Reset all stats.
 * (c) Wake any process waiting for full buffers.
 */
int stop_collection(PWCollector_cmd_t cmd)
{
    /*
     * Reset the power collector task.
     */
    pw_power_collector_task = NULL;
    /*
     * Reset the collection start TSC.
     */
    pw_collection_start_tsc = 0;
    /*
     * Reset the collection time.
     */
    pw_collection_time_ticks = 0;
    /*
     * Was the ACPI S3 hrtimer active? If so, cancel it.
     */
    if (hrtimer_active(&pw_acpi_s3_hrtimer)) {
        printk(KERN_INFO "WARNING: active ACPI S3 timer -- trying to cancel!\n");
        hrtimer_try_to_cancel(&pw_acpi_s3_hrtimer);
    }
    /*
     * Get S and D state residency counter samples
     */
#if DO_S_RESIDENCY_SAMPLE 
    if(pw_is_atm && IS_S_RESIDENCY_MODE()){
        u64 usec = dump_s_residency_counter();
        if (usec > 0) {
            produce_s_residency_sample(usec);
        }
        stop_s_residency_counter();
    }
#endif

#if DO_S_STATE_SAMPLE 
    if(pw_is_atm && IS_S_STATE_MODE()){
        if(INTERNAL_STATE.write_to_buffers) {
            produce_s_state_sample();
        }
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE 
    if(pw_is_atm && IS_D_SC_RESIDENCY_MODE()){
        u64 usec = dump_d_sc_residency_counter();
        if (usec > 0) {
            produce_d_sc_residency_sample(usec);
        } 
        stop_d_sc_residency_counter();
    }
#endif

#if DO_D_NC_STATE_SAMPLE 
    if(pw_is_atm && IS_D_NC_STATE_MODE()){
        if(INTERNAL_STATE.write_to_buffers) {
            produce_d_nc_state_sample();
        }
    }
#endif

#if DO_D_SC_STATE_SAMPLE 
    if(pw_is_atm && IS_D_SC_STATE_MODE()){
        if(INTERNAL_STATE.write_to_buffers) {
            produce_d_sc_state_sample();
        }
    }
#endif

    INTERNAL_STATE.collectionStopJIFF = jiffies;
    INTERNAL_STATE.write_to_buffers = false;
    {
	if(true && cmd == PW_PAUSE){
	    u64 tmp_tsc = 0;
	    tscval(&tmp_tsc);
	    OUTPUT(0, KERN_INFO "RECEIVED PAUSE at tsc = %llu\n", tmp_tsc);
	}
    }

    {
	unregister_non_pausable_probes();
    }

    if(cmd == PW_STOP || cmd == PW_CANCEL) {
        unregister_pausable_probes();
        /*
         * Get STOP P-state samples
         */
        if (likely(IS_FREQ_MODE())) {
            get_current_cpu_frequency(false); // "false" ==> collection STOP
        }
        /*
         * Get STOP C-state samples.
         */
        if (likely(IS_C_STATE_MODE())) {
            generate_end_tps_samples();
        }
        /*
         * Gather some stats on # of samples produced and dropped.
         */
        {
            pw_count_samples_produced_dropped();
        }
    }

    

    // Reset the (per-cpu) "per_cpu_t" structs that hold MSR residencies
    {
        reset_per_cpu_msr_residencies();
    }

    /*
     * Reset IOCTL stats
     *
     * STOP/CANCEL ONLY
     */
#if DO_IOCTL_STATS
    if(cmd == PW_STOP || cmd == PW_CANCEL) {
        reset_statistics();
    }
#endif

    /*
     * Tell consumers to 'flush' all buffers. We need to
     * defer this as long as possible because it needs to be
     * close to the 'wake_up_interruptible', below.
     */
    {
	INTERNAL_STATE.drain_buffers = true;
        smp_mb();
    }

    /*
     * There might be a reader thread blocked on a read: wake
     * it up to give it a chance to respond to changed
     * conditions.
     */
    {
        wake_up_interruptible(&pw_reader_queue);
    }

    /*
     * Delete all non-kernel timers.
     * Also delete the wakelock map.
     *
     * STOP/CANCEL ONLY
     */
    if (cmd == PW_STOP || cmd == PW_CANCEL) {
        delete_all_non_kernel_timers();
#if DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES
        destroy_wlock_map();
#endif // DO_USE_CONSTANT_POOL_FOR_WAKELOCK_NAMES
    }

    OUTPUT(0, KERN_INFO "\tUNREGISTERED all probes!\n");
    return SUCCESS;
};

long handle_cmd(PWCollector_cmd_t cmd)
{
    PWCollector_cmd_t prev_cmd;
    /*
     * Sanity check cmd range.
     */
    if(cmd < PW_START || cmd > PW_MARK){
	pw_pr_error("Error: UNSUPPORTED cmd=%d\n", cmd);
	return -ERROR;
    }
    /*
     * Check to see if there are any invalid
     * command combinations (e.g. START -> START etc.)
     */
    if(!is_cmd_valid(cmd)){
	pw_pr_error("Error: INVALID requested cmd=%d, CURRENT cmd=%d\n", cmd, INTERNAL_STATE.cmd);
	return -ERROR;
    }
    /*
     * OK, we've gotten a valid command.
     * Store it.
     */
    prev_cmd = INTERNAL_STATE.cmd;
    INTERNAL_STATE.cmd = cmd;
    /*
     * Actions based on specific commands here...
     */
    switch(cmd){
    case PW_START:
    case PW_RESUME:
	INTERNAL_STATE.drain_buffers = false;
	// startJIFF = jiffies;
	{
	    if(start_collection(cmd))
		return -ERROR;
	}
	break;
    case PW_STOP:
	// INTERNAL_STATE.drain_buffers = true;
    case PW_PAUSE:
    case PW_CANCEL:
	// stopJIFF = jiffies;
	{
	    stop_collection(cmd);
	}
	break;
    default:
	pw_pr_error("Error: UNSUPPORTED cmd=%d\n", cmd);
	/*
	 * Reset "cmd" state to what it was before
	 * this ioctl.
	 */
	INTERNAL_STATE.cmd = prev_cmd;
	return -ERROR;
    }
    OUTPUT(3, KERN_INFO "Debug: Successfully switched mode from %d to %d: IS_COLLECTING = %d\n", prev_cmd, cmd, IS_COLLECTING());
    return SUCCESS;
};

long do_cmd(PWCollector_cmd_t cmd, u64 *remote_output_args, int size)
{
    int retVal = SUCCESS;
    /*
     * Handle the command itself.
     */
    if (handle_cmd(cmd)) {
        return -ERROR;
    }
    /*
     * Then check if the user requested some collection stats.
     */
#if DO_COUNT_DROPPED_SAMPLES
    if (cmd == PW_STOP || cmd == PW_CANCEL) {
        // u64 local_args[2] = {total_num_samples_produced, total_num_samples_dropped};
        u64 local_args[2] = {pw_num_samples_produced, pw_num_samples_dropped};
        // u64 local_args[2] = {100, 10}; // for debugging!
        if (copy_to_user(remote_output_args, local_args, size)) // returns number of bytes that could NOT be copied
            retVal = -ERROR;
    }
#endif // DO_COUNT_DROPPED_SAMPLES

    return retVal;
};

/*
 * Callback from Power Manager SUSPEND/RESUME events. Useful if the device was suspended
 * (i.e. entered ACPI 'S3') during the collection.
 */
int pw_alrm_suspend_notifier_callback_i(struct notifier_block *block, unsigned long state, void *dummy)
{
    u64 tsc_suspend_time_ticks = 0;
    u64 suspend_time_ticks = 0;
    u64 usec = 0;
    u64 suspend_time_usecs = 0;
    u64 base_operating_freq_mhz = base_operating_freq_khz / 1000;

    if (!pw_is_atm) {
        return NOTIFY_DONE;
    }
    switch (state) {
        case PM_SUSPEND_PREPARE:
            /*
             * Entering SUSPEND -- nothing to do.
             */
            tscval(&pw_suspend_start_tsc);
            printk(KERN_INFO "pw: SUSPEND PREPARE: tsc = %llu\n", pw_suspend_start_tsc);
            if (likely(IS_COLLECTING())) {
                if (IS_S_RESIDENCY_MODE()) {
                    /*
                     * Use the s-residency counters instead of the TSC/RTC hack.
                     */
                    int cpu = RAW_CPU();
                    PWCollector_msg_t msg;
                    s_residency_sample_t sres;

                    usec = dump_s_residency_counter();
                    pw_suspend_start_s0i3 = (u64)(((u32 *)mmio_s_residency_base)[2]);

                    /*
                     * No residency counters available  
                     */
                    msg.data_type = S_RESIDENCY;
                    msg.cpuidx = cpu;
                    msg.tsc = pw_suspend_start_tsc;
                    msg.data_len = sizeof(sres);

                    sres.data[0] = pw_suspend_start_tsc - startTSC_s_residency;

                    if (usec) {
                        int i=0;
                        for (i=0; i<3; ++i) {
                            sres.data[i+1] = ((u32 *)mmio_s_residency_base)[i];
                        }
                    } else {
                        memset(&sres.data[1], 0, sizeof(u64) * 3);
                    }

                    sres.data[4] = sres.data[5] = 0;

                    msg.p_data = (u64)((unsigned long)(&sres));

                    /*
                     * OK, everything computed. Now copy
                     * this sample into an output buffer
                     */
                    pw_produce_generic_msg(&msg, true); // "true" ==> allow wakeups
                }

                /*
                 * And finally, the special 'broadcast' wakelock sample.
                 */
                if (IS_WAKELOCK_MODE()) {
                    PWCollector_msg_t sample;
                    w_sample_t w_msg;

                    memset(&w_msg, 0, sizeof(w_msg));

                    sample.cpuidx = RAW_CPU();
                    sample.tsc = pw_suspend_start_tsc;

                    w_msg.type = PW_WAKE_UNLOCK_ALL;

                    sample.data_type = W_STATE;
                    sample.data_len = sizeof(w_msg);
                    sample.p_data = (u64)(unsigned long)&w_msg;
                    /*
                     * OK, everything computed. Now copy
                     * this sample into an output buffer
                     */
                    pw_produce_generic_msg(&sample, true); // "true" ==> wakeup sleeping readers, if required
                }

                printk(KERN_INFO "SUSPEND PREPARE s0i3 = %llu\n", pw_suspend_start_s0i3);
            }
            break;
        case PM_POST_SUSPEND:
            /*
             * Exitted SUSPEND -- check to see if we've been in suspend
             * for longer than the collection time specified by the user.
             * If so, send the use a SIGINT -- that will force it to
             * stop collecting.
             */
            tscval(&pw_suspend_stop_tsc);
            printk(KERN_INFO "pw: POST SUSPEND: tsc = %llu\n", pw_suspend_stop_tsc);
            BUG_ON(pw_suspend_start_tsc == 0);

            if (likely(IS_COLLECTING()) && likely(IS_S_RESIDENCY_MODE())) {
                /*
                 * Use the s-residency counters instead of the TSC/RTC hack.
                 */
                usec = dump_s_residency_counter();
                pw_suspend_stop_s0i3 = (u64)(((u32 *)mmio_s_residency_base)[2]);
                suspend_time_usecs = (pw_suspend_stop_s0i3 - pw_suspend_start_s0i3); 
                suspend_time_ticks = suspend_time_usecs * base_operating_freq_mhz;
                printk(KERN_INFO "BASE operating freq_mhz = %llu\n", base_operating_freq_mhz);
                printk(KERN_INFO "POST SUSPEND s0i3 = %llu, S3 RESIDENCY = %llu (%llu ticks)\n", pw_suspend_stop_s0i3, suspend_time_usecs, suspend_time_ticks);

                /*
                 * We need to an 'S_RESIDENCY' sample detailing the actual supend 
                 * statistics (when did the device get suspended; for how long 
                 * was it suspended etc.). 
                 */
                {
                    PWCollector_msg_t msg;
                    s_residency_sample_t sres;
                    
                    msg.data_type = S_RESIDENCY;
                    msg.cpuidx = RAW_CPU();
                    msg.tsc = pw_suspend_stop_tsc;
                    msg.data_len = sizeof(sres);

                    sres.data[0] = pw_suspend_stop_tsc - startTSC_s_residency;

                    if (usec) {
                        int i=0;
                        for (i=0; i<3; ++i) {
                            sres.data[i+1] = ((u32 *)mmio_s_residency_base)[i];
                        }
                    } else {
                        memset(&sres.data[1], 0, sizeof(u64) * 3);
                    }

                    sres.data[4] = suspend_time_ticks;
                    sres.data[5] = pw_suspend_start_tsc;

                    msg.p_data = (u64)((unsigned long)(&sres));

                    /*
                     * OK, everything computed. Now copy
                     * this sample into an output buffer
                     */
                    pw_produce_generic_msg(&msg, true); // "true" ==> allow wakeups
                }
            } else {
                tsc_suspend_time_ticks = (pw_suspend_stop_tsc - pw_suspend_start_tsc);
                suspend_time_ticks = tsc_suspend_time_ticks;
            }
            printk(KERN_INFO "OK: suspend time ticks = %llu\n", suspend_time_ticks);

            break;
        default:
            pw_pr_error("pw: unknown = %lu\n", state);
    }
    return NOTIFY_DONE;
};
/*
 * PM notifier.
 */
struct notifier_block pw_alrm_pm_suspend_notifier = {
    .notifier_call = &pw_alrm_suspend_notifier_callback_i,
};

static inline int get_arg_lengths(unsigned long ioctl_param, int *in_len, int *out_len)
{
    ioctl_args_stub_t local_stub, *remote_stub;

    remote_stub = (ioctl_args_stub_t *)ioctl_param;
    if(copy_from_user(&local_stub, remote_stub, sizeof(ioctl_args_stub_t))){
	pw_pr_error("ERROR: could NOT extract local stub!\n");
	return -ERROR;
    }
    OUTPUT(0, KERN_INFO "OK: in_len = %d, out_len = %d\n", local_stub.in_len, local_stub.out_len);
    *in_len = local_stub.in_len; *out_len = local_stub.out_len;
    return SUCCESS;
};


#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
#define MATCH_IOCTL(num, pred) ( (num) == (pred) || (num) == (pred##32) )
#else
#define MATCH_IOCTL(num, pred) ( (num) == (pred) )
#endif

/*
 * Service IOCTL calls from user-space.
 * Handles both 32b and 64b calls.
 */
long pw_unlocked_handle_ioctl_i(unsigned int ioctl_num, struct PWCollector_ioctl_arg *remote_args, unsigned long ioctl_param)
{
    int local_in_len, local_out_len;
    PWCollector_cmd_t cmd;
    int tmp = -1;


    /*
     * (1) Sanity check:
     * Before doing anything, double check to
     * make sure this IOCTL was really intended
     * for us!
     */
    if(_IOC_TYPE(ioctl_num) != APWR_IOCTL_MAGIC_NUM){
	pw_pr_error("ERROR: requested IOCTL TYPE (%d) != APWR_IOCTL_MAGIC_NUM (%d)\n", _IOC_TYPE(ioctl_num), APWR_IOCTL_MAGIC_NUM);
	return -ERROR;
    }
    /*
     * (2) Extract arg lengths.
     */
    if(get_arg_lengths(ioctl_param, &local_in_len, &local_out_len)){
	return -ERROR;
    }
    /*
     * (3) Service individual IOCTL requests.
     */
    if(MATCH_IOCTL(ioctl_num, PW_IOCTL_CONFIG)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_CONFIG\n");
	return set_config((struct PWCollector_config *)remote_args->in_arg, local_in_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_CMD)){
	if(get_user(cmd, ((PWCollector_cmd_t *)remote_args->in_arg))){
	    pw_pr_error("ERROR: could NOT extract cmd value!\n");
	    return -ERROR;
	}
	OUTPUT(0, KERN_INFO "PW_IOCTL_CMD: cmd=%d\n", cmd);
	// return handle_cmd(cmd);
        return do_cmd(cmd, (u64 *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_STATUS)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_STATUS\n");
	/*
	 * For now, we assume STATUS information can only
	 * be retrieved for an ACTIVE collection.
	 */
	if(!IS_COLLECTING()){
	    pw_pr_error("\tError: status information requested, but NO COLLECTION ONGOING!\n");
	    return -ERROR;
	}
#if DO_IOCTL_STATS
	return get_status((struct PWCollector_status *)remote_args->out_arg, local_out_len);
#else
	return -ERROR;
#endif
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_CHECK_PLATFORM)) {
	OUTPUT(0, KERN_INFO "PW_IOCTL_CHECK_PLATFORM\n");
	if( (tmp = check_platform((struct PWCollector_check_platform *)remote_args->out_arg, local_out_len)))
	    if(tmp < 0) // ERROR
		return 2; // for PW_IOCTL_CHECK_PLATFORM: >= 2 ==> Error; == 1 => SUCCESS, but not EOF; 0 ==> SUCCESS, EOF
	return tmp;
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_VERSION)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_VERSION\n");
	OUTPUT(3, KERN_INFO "OUT len = %d\n", local_out_len);
	return get_version((struct PWCollector_version_info *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_DEVICES)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_DEVICES\n");
	OUTPUT(3, KERN_INFO "OUT len = %d\n", local_out_len);
	return get_available_devices((struct PWCollector_device_info *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_MICRO_PATCH)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_MICRO_PATCH\n");
	return get_micro_patch_ver((int *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_TURBO_THRESHOLD)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_TURBO_THRESHOLD\n");
	return get_turbo_threshold((struct PWCollector_turbo_threshold *)remote_args->out_arg, local_out_len);
    }
    else if(MATCH_IOCTL(ioctl_num, PW_IOCTL_AVAILABLE_FREQUENCIES)){
	OUTPUT(0, KERN_INFO "PW_IOCTL_AVAILABLE_FREQUENCIES\n");
	return get_available_frequencies((struct PWCollector_available_frequencies *)remote_args->out_arg, local_out_len);
    }
    else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_COLLECTION_TIME)) {
        /*
         * Only supported on Android/Moorestown!!!
         */
        OUTPUT(0, KERN_INFO "PW_IOCTL_COLLECTION_TIME\n");
// #ifdef CONFIG_X86_MRST
        {
            unsigned int local_collection_time_secs = 0;
            if (get_user(local_collection_time_secs, (unsigned long *)remote_args->in_arg)) {
                pw_pr_error("ERROR extracting local collection time!\n");
                return -ERROR;
            }
            printk(KERN_INFO "OK: received local collection time = %u seconds\n", local_collection_time_secs);
            /*
             * Get (and set) collection START time...
             */
            {
                // pw_rtc_time_start = pw_get_current_rtc_time_seconds();
            }
            /*
             * ...and the total collection time...
             */
            {
                pw_collection_time_secs = local_collection_time_secs;
                pw_collection_time_ticks = (u64)local_collection_time_secs * (u64)base_operating_freq_khz * 1000;
            }
            /*
             * ...and the client task.
             */
            {
                pw_power_collector_task = current;
            }
        }
// #endif // CONFIG_X86_MRST
        return SUCCESS;
    }
    else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_MMAP_SIZE)) {
        pw_pr_debug("MMAP_SIZE received!\n");
        if(put_user(pw_buffer_alloc_size, (unsigned long *)remote_args->out_arg)) {
            pw_pr_error("ERROR transfering buffer size!\n");
            return -ERROR;
        }
        return SUCCESS;
    }
    else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_BUFFER_SIZE)) {
        unsigned long buff_size = pw_get_buffer_size();
        pw_pr_debug("BUFFER_SIZE received!\n");
        if(put_user(buff_size, (unsigned long *)remote_args->out_arg)) {
            pw_pr_error("ERROR transfering buffer size!\n");
            return -ERROR;
        }
        return SUCCESS;
    }
    else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_DO_D_NC_READ)) {
        pw_pr_debug("PW_IOCTL_DO_D_NC_READ  received!\n");
        // printk(KERN_INFO "PW_IOCTL_DO_D_NC_READ  received!\n");
#if DO_D_NC_STATE_SAMPLE 
        if (produce_d_nc_state_sample()) {
            pw_pr_error("ERROR taking NC D-state sample!\n");
            return -ERROR;
        }
#endif
        return SUCCESS;
    }
    else if (MATCH_IOCTL(ioctl_num, PW_IOCTL_FSB_FREQ)) {
        pw_pr_debug("PW_IOCTL_FSB_FREQ  received!\n");
        // printk(KERN_INFO "PW_IOCTL_FSB_FREQ  received!\n");
        if (put_user(pw_msr_fsb_freq_value, (unsigned long *)remote_args->out_arg)) {
            pw_pr_error("ERROR transfering FSB_FREQ MSR value!\n");
            return -ERROR;
        }
        return SUCCESS;
    }
    else{
	// ERROR!
	pw_pr_error("Invalid IOCTL command = %u\n", ioctl_num);
	return -ERROR;
    }
    /*
     * Should NEVER reach here!
     */
    return -ERROR;
};

/*
 * (1) Handle 32b IOCTLs in 32b kernel-space.
 * (2) Handle 64b IOCTLs in 64b kernel-space.
 */
long pw_device_unlocked_ioctl(struct file *filp, unsigned int ioctl_num, unsigned long ioctl_param)
{
    OUTPUT(3, KERN_INFO "64b transfering to handler!\n");
    return pw_unlocked_handle_ioctl_i(ioctl_num, (struct PWCollector_ioctl_arg *)ioctl_param, ioctl_param);
};

#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
/*
 * Handle 32b IOCTLs in 64b kernel-space.
 */
long pw_device_compat_ioctl(struct file *file, unsigned int ioctl_num, unsigned long ioctl_param)
{
    struct PWCollector_ioctl_arg remote_args;
    struct PWCollector_ioctl_arg32 *remote_args32 = (struct PWCollector_ioctl_arg32 *)ioctl_param;

    remote_args.in_len = remote_args32->in_len;
    remote_args.in_arg = (char *)((unsigned long)remote_args32->in_arg);
    remote_args.out_len = remote_args32->out_len;
    remote_args.out_arg = (char *)((unsigned long)remote_args32->out_arg);
    /*
    remote_args.in_len = remote_args32->in_len; remote_args.in_arg = remote_args32->in_arg;
    remote_args.out_len = remote_args32->out_len; remote_args.out_arg = remote_args32->out_arg;
    */

    OUTPUT(3, KERN_INFO "32b transfering to handler!\n");

    return pw_unlocked_handle_ioctl_i(ioctl_num, &remote_args, ioctl_param);
};
#endif // COMPAT && x64

/*
 * Service an "open(...)" call from user-space.
 */
static int pw_device_open(struct inode *inode, struct file *file)
{
    /* 
     * We don't want to talk to two processes at the same time 
     */
    if(test_and_set_bit(DEV_IS_OPEN, &dev_status)){
	// Device is busy
	return -EBUSY;
    }

    try_module_get(THIS_MODULE);
    return SUCCESS;
};

/*
 * Service a "close(...)" call from user-space.
 */
static int pw_device_release(struct inode *inode, struct file *file)
{
    OUTPUT(3, KERN_INFO "Debug: Device Release!\n");
    /*
     * Did the client just try to zombie us?
     */
    if(IS_COLLECTING()){
	pw_pr_error("ERROR: Detected ongoing collection on a device release!\n");
	INTERNAL_STATE.cmd = PW_CANCEL;
	stop_collection(PW_CANCEL);
    }
    module_put(THIS_MODULE);
    /* 
     * We're now ready for our next caller 
     */
    clear_bit(DEV_IS_OPEN, &dev_status);
    return SUCCESS;
};


int pw_register_dev(void)
{
    int ret;

    /* 
     * Create the character device
     */
    ret = alloc_chrdev_region(&apwr_dev, 0, 1, PW_DEVICE_NAME);
    apwr_dev_major_num = MAJOR(apwr_dev);
    apwr_class = class_create(THIS_MODULE, "apwr");
    if(IS_ERR(apwr_class))
        printk(KERN_ERR "Error registering apwr class\n");

    device_create(apwr_class, NULL, apwr_dev, NULL, PW_DEVICE_NAME);
    apwr_cdev = cdev_alloc();
    if (apwr_cdev == NULL) {
        printk("Error allocating character device\n");
        return ret;
    }
    apwr_cdev->owner = THIS_MODULE;
    apwr_cdev->ops = &Fops;
    if( cdev_add(apwr_cdev, apwr_dev, 1) < 0 )  {
        printk("Error registering device driver\n");
        return ret;
    }

    return ret;
};

void pw_unregister_dev(void)
{
    /* 
     * Remove the device 
     */
    unregister_chrdev(apwr_dev_major_num, PW_DEVICE_NAME);
    device_destroy(apwr_class, apwr_dev);
    class_destroy(apwr_class);
    unregister_chrdev_region(apwr_dev, 1);
    cdev_del(apwr_cdev);
};

#ifndef __arm__
static void disable_auto_demote(void *dummy)
{
    unsigned long long auto_demote_disable_flags = AUTO_DEMOTE_FLAGS();
    unsigned long long msr_addr = AUTO_DEMOTE_MSR;
    unsigned long long msr_bits = 0, old_msr_bits = 0;

    rdmsrl(msr_addr, msr_bits);
    old_msr_bits = msr_bits;
    msr_bits &= ~auto_demote_disable_flags;
    wrmsrl(msr_addr, msr_bits);

    if (true) {
        printk(KERN_INFO "[%d]: old_msr_bits = %llu, was auto enabled = %s, DISABLED auto-demote\n", RAW_CPU(), old_msr_bits, GET_BOOL_STRING(IS_AUTO_DEMOTE_ENABLED(old_msr_bits)));
    }
};

static void enable_auto_demote(void *dummy)
{
    unsigned long long auto_demote_disable_flags = AUTO_DEMOTE_FLAGS();
    unsigned long long msr_addr = AUTO_DEMOTE_MSR;
    unsigned long long msr_bits = 0, old_msr_bits = 0;

    rdmsrl(msr_addr, msr_bits);
    old_msr_bits = msr_bits;
    msr_bits |= auto_demote_disable_flags;
    wrmsrl(msr_addr, msr_bits);

    if (true) {
        printk(KERN_INFO "[%d]: OLD msr_bits = %llu, NEW msr_bits = %llu\n", raw_smp_processor_id(), old_msr_bits, msr_bits);
    }
};
#endif // ifndef __arm__

static bool check_auto_demote_flags(int cpu)
{
#ifndef __arm__
    u32 l=0, h=0;
    u64 msr_val = 0;
    WARN_ON(rdmsr_safe_on_cpu(cpu, AUTO_DEMOTE_MSR, &l, &h));
    msr_val = (u64)h << 32 | (u64)l;
    return IS_AUTO_DEMOTE_ENABLED(msr_val);
#else
    return false;
#endif // ifndef __arm__
};

static bool check_any_thread_flags(int cpu)
{
#ifndef __arm__
    u32 l=0, h=0;
    u64 msr_val = 0;
    WARN_ON(rdmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, &l, &h));
    msr_val = (u64)h << 32 | (u64)l;
    return IS_ANY_THREAD_SET(msr_val);
#else
    return false;
#endif // ifndef __arm__
};

static void check_arch_flags(void)
{
    int cpu = 0;
    /*
     * It is ASSUMED that auto-demote and any-thread will either be set on ALL Cpus or on
     * none!
     */
    pw_is_any_thread_set = check_any_thread_flags(cpu);
    pw_is_auto_demote_enabled = check_auto_demote_flags(cpu);

    OUTPUT(0, KERN_INFO "any thread set = %s, auto demote enabled = %s\n", GET_BOOL_STRING(pw_is_any_thread_set), GET_BOOL_STRING(pw_is_auto_demote_enabled));
    return;
};

#ifndef __arm__
/*
 * Enable CPU_CLK_UNHALTED.REF counting
 * by setting bits 8,9 in MSR_PERF_FIXED_CTR_CTRL
 * MSR (addr == 0x38d). Also store the previous
 * value of the MSR.
 */
static void enable_ref(void)
{
    int cpu;
    u64 res;
    int ret;

    u32 *data_copy;// [2];
    u32 data[2];

    for_each_online_cpu(cpu){
        /*
         * (1) Do for IA32_FIXED_CTR_CTL
         */
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->fixed_data;
            ret = rdmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, &data[0], &data[1]);
            WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
            memcpy(data_copy, data, sizeof(u32) * 2);
            /*
             * Turn on CPU_CLK_UNHALTED.REF counting.
             *
             * UPDATE: also turn on CPU_CLK_UNHALTED.CORE counting.
             */
            // data[0] |= 0x300;
            data[0] |= 0x330;

            ret = wrmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, data[0], data[1]);
        }
        /*
         * (2) Do for IA32_PERF_GLOBAL_CTRL_ADDR
         */
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->perf_data;
            ret = rdmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, &data[0], &data[1]);
            WARN(ret, KERN_WARNING "rdmsr failed with code %d\n", ret);
            memcpy(data_copy, data, sizeof(u32) * 2);
            res = data[1];
            res <<= 32;
            res += data[0];
            OUTPUT(0, KERN_INFO "[%d]: READ res = 0x%llx\n", cpu, res);
            /*
             * Turn on CPU_CLK_UNHALTED.REF counting.
             *
             * UPDATE: also turn on CPU_CLK_UNHALTED.CORE counting.
             * Set bits 33, 34
             */
            // data[0] |= 0x330;
            data[1] |= 0x6;
            // data[0] = data[1] = 0x0;

            ret = wrmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, data[0], data[1]);
        }
    }
};

static void restore_ref(void)
{
    int cpu;
    u64 res;
    int ret;

    u32 *data_copy;
    u32 data[2];

    memset(data, 0, sizeof(u32) * 2);

    for_each_online_cpu(cpu){
        /*
         * (1) Do for IA32_FIXED_CTR_CTL
         */
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->fixed_data;
            memcpy(data, data_copy, sizeof(u32) * 2);

            res = data[1];
            res <<= 32;
            res += data[0];

            OUTPUT(3, KERN_INFO "[%d]: PREV res = 0x%llx\n", cpu, res);
            if( (ret = wrmsr_safe_on_cpu(cpu, IA32_FIXED_CTR_CTL_ADDR, data[0], data[1]))){
                pw_pr_error("ERROR writing PREVIOUS IA32_FIXED_CTR_CLT_ADDR values for CPU = %d!\n", cpu);
            }
        }
        /*
         * (2) Do for IA32_PERF_GLOBAL_CTRL_ADDR
         */
        {
            data_copy = (&per_cpu(CTRL_data_values, cpu))->perf_data;
            memcpy(data, data_copy, sizeof(u32) * 2);

            res = data[1];
            res <<= 32;
            res += data[0];

            OUTPUT(3, KERN_INFO "[%d]: PREV res = 0x%llx\n", cpu, res);
            if( (ret = wrmsr_safe_on_cpu(cpu, IA32_PERF_GLOBAL_CTRL_ADDR, data[0], data[1]))){
                pw_pr_error("ERROR writing PREVIOUS IA32_PERF_GLOBAL_CTRL_ADDR values for CPU = %d!\n", cpu);
            }
        }
    }
};
#endif // ifndef __arm__
/*
 * Check if we're running on ATM.
 */
atom_arch_type_t is_atm(void)
{
#ifndef __arm__
    unsigned int ecx, edx;
    unsigned int fms, family, model, stepping;

    asm("cpuid" : "=a" (fms), "=c" (ecx), "=d" (edx) : "a" (1) : "ebx");

    family = (fms >> 8) & 0xf;
    model = (fms >> 4) & 0xf;
    stepping = fms & 0xf;

    if (family == 6 || family == 0xf){
        model += ((fms >> 16) & 0xf) << 4;
    }
    OUTPUT(0, KERN_INFO "FMS = 0x%x:%x:%x (%d:%d:%d)\n", family, model, stepping, family, model, stepping);
    /*
     * This check below will need to
     * be updated for each new
     * architecture type!!!
     */
    if (family == 0x6) {
        switch (model) {
            case 0x27:
                switch (stepping) {
                    case 0x1:
                        return MFD;
                    case 0x2:
                        return LEX;
                }
                break;
            case 0x35:
                return CLV;
        }
    }
#endif // ifndef __arm__
    return NON_ATOM;
};

static int __init init_hooks(void)
{
    int ret = SUCCESS;

    if (false) {
#ifndef __arm__
        printk(KERN_INFO "F.M = %u.%u\n", boot_cpu_data.x86, boot_cpu_data.x86_model);
#endif // ifndef __arm__
        return -ERROR;
    }


    OUTPUT(0, KERN_INFO "# IRQS = %d\n", NR_IRQS);

    OUTPUT(0, KERN_INFO "Sizeof PWCollector_sample_t = %lu, Sizeof k_sample_t = %lu\n", sizeof(PWCollector_sample_t), sizeof(k_sample_t));

    /*
     * We first check to see if
     * TRACEPOINTS are ENABLED in the kernel.
     * If not, EXIT IMMEDIATELY!
     */
#ifdef CONFIG_TRACEPOINTS
    OUTPUT(0, KERN_INFO "Tracepoints ON!\n");
#else
    pw_pr_error("ERROR: TRACEPOINTS NOT found on system!!!\n");
    return -ERROR;
#endif

    /*
     * Check if we're running on ATM.
     */
    pw_is_atm = is_atm();

    /*
     * For MFLD, we also check
     * if the required microcode patches
     * have been installed. If
     * not then EXIT IMMEDIATELY!
     */
#if DO_CHECK_BO_MICROCODE_PATCH
    {
        /*
         * Read MSR 0x8b -- if microcode patch
         * has been applied then the first 12 bits
         * of the higher order 32 bits should be
         * >= 0x102.
         *
         * THIS CHECK VALID FOR ATM ONLY!!!
         */
        /*
         * Do check ONLY if we're ATM!
         */
        if(pw_is_atm){
#ifndef __arm__
            u64 res;
            u32 patch_val;

            rdmsrl(0x8b, res);
            patch_val = (res >> 32) & 0xfff;
            if(patch_val < 0x102){
                pw_pr_error("ERROR: B0 micro code path = 0x%x: REQUIRED >= 0x102!!!\n", patch_val);
                return -ERROR;
            }
            micro_patch_ver = patch_val;
            OUTPUT(3, KERN_INFO "patch ver = %u\n", micro_patch_ver);
#endif // ifndef __arm__
        }else{
            OUTPUT(0, KERN_INFO "DEBUG: SKIPPING MICROCODE PATCH check -- NON ATM DETECTED!\n");
        }
    }
#endif

    /*
     * Read the 'FSB_FREQ' MSR to determine bus clock freq multiplier.
     * Update: ONLY if saltwell!
     */
    if (pw_is_atm) {
        u64 res;

        rdmsrl(MSR_FSB_FREQ_ADDR, res);
        memcpy(&pw_msr_fsb_freq_value, &res, sizeof(unsigned long));
        printk(KERN_INFO "MSR_FSB_FREQ value = %lu\n", pw_msr_fsb_freq_value);
    }

    OUTPUT(3, KERN_INFO "Sizeof node = %lu\n", sizeof(tnode_t));
    OUTPUT(3, KERN_INFO "Sizeof per_cpu_t = %lu\n", sizeof(per_cpu_t));

    startJIFF = jiffies;

    if (pw_init_data_structures()) {
        return -ERROR;
    }

#if 0
    {
        get_base_operating_frequency();
    }
#endif

    /*
    {
        disable_auto_demote(NULL);
        smp_call_function(disable_auto_demote, NULL, 1);
        printk(KERN_INFO "DISABLED AUTO-DEMOTE!\n");
    }
    */
    /*
     * Check Arch flags (ANY_THREAD, AUTO_DEMOTE etc.)
     */
    {
        check_arch_flags();
    }
    {
        // enable_ref();
    }

#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
    if(pw_is_atm){
        void *mmio_d_map_base = NULL;
        switch(pw_is_atm) {
            case MFD:
            case LEX:
                d_sc_device_num = MFD_MAX_LSS_NUM_IN_SC;
                break;
            case CLV:
                d_sc_device_num = CLV_MAX_LSS_NUM_IN_SC;
                break;
            case NON_ATOM:
                break;
        }
        // Map the bus memory into CPU space for 6 IPC registers
        mmio_ipc1_base = ioremap_nocache(IPC_BASE_ADDRESS, IPC_MAX_ADDR);

        if (mmio_ipc1_base == NULL) {
            printk(KERN_ERR "ioremap_nocache returns NULL! IPC1 is NOT available\n");
            ret = -ERROR;
            goto err_ret_post_init;
        }

#if DO_S_RESIDENCY_SAMPLE
        // Map the bus memory into CPU space for 4 S state residency counters
        mmio_s_residency_base = ioremap_nocache(S_RESIDENCY_BASE_ADDRESS, S_RESIDENCY_MAX_COUNTERS*4);

        if (mmio_s_residency_base == NULL) {
            printk(KERN_ERR "ioremap_nocache returns NULL! S Residency counter is NOT available\n");
            ret = -ERROR;
            goto err_ret_post_init;
        }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE
        // Map the bus memory into CPU space for 4(bytes) * 3(states) * 40(LSS) D state residency counters
        mmio_d_residency_base = ioremap_nocache(D_RESIDENCY_BASE_ADDRESS, D_RESIDENCY_MAX_COUNTERS*4);

        if (mmio_d_residency_base == NULL) {
            printk(KERN_ERR "ioremap_nocache returns NULL! D Residency counter is NOT available\n");
            ret = -ERROR;
            goto err_ret_post_init;
        }

        // Map the bus memory into CPU space for 4(bytes) * 2 for available devices 
        if(pw_is_atm == CLV) {
#if USE_PREDEFINED_SCU_IPC
            ret = intel_scu_ipc_simple_command(IPC_MESSAGE_D_RESIDENCY, IPC_COMMAND_OTHER, NULL, 0, NULL, 0);
#else
            LOCK(ipclock);
            ipc_command((IPC_COMMAND_OTHER << 12) | IPC_MESSAGE_D_RESIDENCY);
            ret = busy_loop();
#endif
#if !USE_PREDEFINED_SCU_IPC
            UNLOCK(ipclock);
#endif
            mmio_d_map_base = ioremap_nocache(DEV_UNDEFINED_ADDRESS, 8);

            if (mmio_d_map_base == NULL) {
                printk(KERN_INFO "Device availability check failed! # of devices is set to %d\n", CLV_MAX_LSS_NUM_IN_SC);
            } else {
                int i, count = 0;
                u32 val_lo, val_hi;
                val_lo = readl(mmio_d_map_base);
                val_hi = readl(mmio_d_map_base + 4);
                OUTPUT(3, KERN_INFO "[apwr] LO = 0x%x, 0x%x\n", val_lo, val_hi);
                undefined_device_list = (((u64)val_hi << 32) | val_lo);
                iounmap(mmio_d_map_base);
                for (i=0; i<64; i++) {
                    if (!(undefined_device_list & ((u64)0x1 << i))) {
                        count++;
                    }
                } 
                d_sc_device_num = count;
                OUTPUT(3, KERN_INFO "[apwr] Undefined devices = 0x%llx, # of devices = %d\n", undefined_device_list, d_sc_device_num);
            }
#endif
        }

        mmio_cumulative_residency_base = ioremap_nocache(CUMULATIVE_RESIDENCY_ADDRESS, 4);

        if (mmio_cumulative_residency_base == NULL) {
            printk(KERN_ERR "ioremap_nocache returns NULL! Cumulative Residency counter is NOT available\n");
            ret = -ERROR;
            goto err_ret_post_init;
        }
    }
#endif

#if DO_S_STATE_SAMPLE || DO_D_SC_STATE_SAMPLE
    if(pw_is_atm){
        // Map the bus memory into CPU space for power management
        mmio_pm_base = ioremap_nocache(PM_BASE_ADDRESS, PM_MAX_ADDR);

        if (mmio_pm_base == NULL) {
            printk(KERN_ERR "ioremap_nocache returns NULL! PM is NOT available\n");
            ret = -ERROR;
            goto err_ret_post_init;
        }
    }
#endif

#if DO_D_NC_STATE_SAMPLE && DO_ANDROID
     {
        u32 read_value = 0;
        read_value = intel_mid_msgbus_read32_raw(NC_APM_STS_REG);
        pci_apm_sts_mem_addr = read_value & 0xFFFF;
       
        read_value = intel_mid_msgbus_read32_raw(NC_PM_SSS_REG);
        pci_pm_sss_mem_addr = read_value & 0xFFFF;
    } 
#endif

    {
        /*
         * Check if kernel-space call stack generation
         * is possible.
         */
#ifdef CONFIG_FRAME_POINTER
        OUTPUT(0, KERN_INFO "Frame pointer ON!\n");
        INTERNAL_STATE.have_kernel_frame_pointers = true;
#else
        printk(KERN_INFO "**********************************************************************************************************\n");
        printk(KERN_INFO "Error: kernel NOT compiled with frame pointers -- NO KERNEL-SPACE TIMER CALL TRACES WILL BE GENERATED!\n");
        printk(KERN_INFO "**********************************************************************************************************\n");
        INTERNAL_STATE.have_kernel_frame_pointers = false;
#endif
    }

    /*
     * "Register" the device-specific special character file here.
     */
    {
        if( (ret = pw_register_dev()) < 0) {
            goto err_ret_post_init;
        }
    }

    /*
     * Probes required to cache (kernel) timer 
     * callstacks need to be inserted, regardless 
     * of collection status.
     */
    {
        // register_timer_callstack_probes();
        register_permanent_probes();
    }
    /*
     * Register SUSPEND/RESUME notifier.
     */
    {
        register_pm_notifier(&pw_alrm_pm_suspend_notifier);
    }

#if 0
    {
        register_all_probes();
    }
#endif

    printk(KERN_INFO "\n--------------------------------------------------------------------------------------------\n");
    printk(KERN_INFO "START Initialized the PWR DRIVER\n");
    printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");

    return SUCCESS;

err_ret_post_init:
    pw_destroy_data_structures();
    // restore_ref();
    /*
    {
        enable_auto_demote(NULL);
        smp_call_function(enable_auto_demote, NULL, 1);
        printk(KERN_INFO "ENABLED AUTO-DEMOTE!\n");
    }
    */

    return ret;
};

static void __exit cleanup_hooks(void)
{
    unsigned long elapsedJIFF = 0, collectJIFF = 0;
    int num_timers = 0, num_irqs = 0;

    {
        pw_unregister_dev();
    }

#if DO_S_RESIDENCY_SAMPLE 
    if(mmio_s_residency_base != NULL && pw_is_atm){
        stop_s_residency_counter();

        iounmap(mmio_s_residency_base);
    }
#endif

#if DO_D_SC_RESIDENCY_SAMPLE 
    if(mmio_d_residency_base != NULL && pw_is_atm){
        stop_d_sc_residency_counter();

        iounmap(mmio_d_residency_base);
    }
#endif

#if DO_S_RESIDENCY_SAMPLE || DO_D_SC_RESIDENCY_SAMPLE
    if(mmio_ipc1_base != NULL && pw_is_atm){
        iounmap(mmio_ipc1_base);
        iounmap(mmio_cumulative_residency_base);
    }
#endif

#if DO_S_STATE_SAMPLE || DO_D_SC_STATE_SAMPLE
    if(mmio_pm_base != NULL && pw_is_atm){
        iounmap(mmio_pm_base);
    }
#endif

    /*
     * Unregister the suspend notifier.
     */
    {
        unregister_pm_notifier(&pw_alrm_pm_suspend_notifier);
    }

    /*
     * Probes required to cache (kernel) timer 
     * callstacks need to be removed, regardless 
     * of collection status.
     */
    {
        // unregister_timer_callstack_probes();
        unregister_permanent_probes();
    }

#if 1
    if(IS_COLLECTING()){
        // unregister_all_probes();
        unregister_non_pausable_probes();
        unregister_pausable_probes();
    }
    else if(IS_SLEEPING()){
        unregister_pausable_probes();
    }
#else
    /*
     * Forcibly unregister -- used in debugging.
     */
    {
        unregister_all_probes();
    }
#endif


    {
        num_timers = get_num_timers();
#if DO_CACHE_IRQ_DEV_NAME_MAPPINGS
        num_irqs = get_num_irq_mappings();
#endif
    }

    {
        pw_destroy_data_structures();
    }

    {
        // restore_ref();
    }
    /*
    {
        enable_auto_demote(NULL);
        smp_call_function(enable_auto_demote, NULL, 1);
        printk(KERN_INFO "ENABLED AUTO-DEMOTE!\n");
    }
    */

    /*
     * Collect some statistics: total execution time.
     */
    stopJIFF = jiffies;
    if(stopJIFF < startJIFF){
        OUTPUT(0, KERN_INFO "WARNING: jiffies counter has WRAPPED AROUND!\n");
        elapsedJIFF = 0; // avoid messy NAN when dividing
    }else{
        elapsedJIFF = stopJIFF - startJIFF;
    }

    /*
     * Collect some collection statistics: total collection time.
     */
    if(INTERNAL_STATE.collectionStopJIFF < INTERNAL_STATE.collectionStartJIFF){
        OUTPUT(0, KERN_INFO "WARNING: jiffies counter has WRAPPED AROUND!\n");
        collectJIFF = 0;
    }else{
        collectJIFF = INTERNAL_STATE.collectionStopJIFF - INTERNAL_STATE.collectionStartJIFF;
    }

    printk(KERN_INFO "\n--------------------------------------------------------------------------------------------\n");

    printk(KERN_INFO "STOP Terminated the PWR Driver.\n");
#if DO_PRINT_COLLECTION_STATS
    printk(KERN_INFO "Total time elapsed = %u msecs, Total collection time = %u msecs\n", jiffies_to_msecs(elapsedJIFF), jiffies_to_msecs(collectJIFF));

    printk(KERN_INFO "Total # timers = %d, Total # irq mappings = %d\n", num_timers, num_irqs);

#if DO_OVERHEAD_MEASUREMENTS
    {
        timer_init_print_cumulative_overhead_params("TIMER_INIT");
        timer_expire_print_cumulative_overhead_params("TIMER_EXPIRE");
        timer_insert_print_cumulative_overhead_params("TIMER_INSERT");
        tps_print_cumulative_overhead_params("TPS");
        tpf_print_cumulative_overhead_params("TPF");
        inter_common_print_cumulative_overhead_params("INTER_COMMON");
        irq_insert_print_cumulative_overhead_params("IRQ_INSERT");
        find_irq_node_i_print_cumulative_overhead_params("FIND_IRQ_NODE_I");
        exit_helper_print_cumulative_overhead_params("EXIT_HELPER");
        timer_delete_print_cumulative_overhead_params("TIMER_DELETE");
        sys_enter_helper_i_print_cumulative_overhead_params("SYS_ENTER_HELPER_I");
        sys_exit_helper_i_print_cumulative_overhead_params("SYS_EXIT_HELPER_I");
        /*
         * Also print stats on timer entries.
         */
        printk(KERN_INFO "# TIMER ENTRIES = %d\n", atomic_read(&num_timer_entries));
        /*
         * And some mem debugging stats.
         */
        printk(KERN_INFO "TOTAL # BYTES ALLOCED = %llu, CURR # BYTES ALLOCED = %llu, MAX # BYTES ALLOCED = %llu\n", TOTAL_NUM_BYTES_ALLOCED(), CURR_NUM_BYTES_ALLOCED(), MAX_NUM_BYTES_ALLOCED());
    }
#endif // DO_OVERHEAD_MEASUREMENTS
#endif // DO_PRINT_COLLECTION_STATS

    printk(KERN_INFO "--------------------------------------------------------------------------------------------\n");
};

module_init(init_hooks);
module_exit(cleanup_hooks);

MODULE_LICENSE("GPL");
MODULE_AUTHOR(MOD_AUTHOR);
MODULE_DESCRIPTION(MOD_DESC);
