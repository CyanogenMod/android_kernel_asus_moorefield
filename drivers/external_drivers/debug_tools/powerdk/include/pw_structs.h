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

/*
 * Description: file containing data structures used by the
 * power driver.
 */

#ifndef _DATA_STRUCTURES_H_
#define _DATA_STRUCTURES_H_ 1

#include "pw_types.h"

/*
 * Should we probe on syscall enters and exits?
 * We require this functionality to handle certain
 * device-driver related timers.
 * ********************************************************
 * WARNING: SETTING TO 1 will INVOLVE HIGH OVERHEAD!!!
 * ********************************************************
 */
#define DO_PROBE_ON_SYSCALL_ENTER_EXIT 0
#define DO_PROBE_ON_EXEC_SYSCALL DO_PROBE_ON_SYSCALL_ENTER_EXIT
/*
 * Do we use an RCU-based mechanism
 * to determine which output buffers
 * to write to?
 * Set to: "1" ==> YES
 *         "0" ==> NO
 * ************************************
 * CAUTION: RCU-based output buffer
 * selection is EXPERIMENTAL ONLY!!!
 * ************************************
 */
#define DO_RCU_OUTPUT_BUFFERS 0
/*
 * Do we force the device driver to
 * (periodically) flush its buffers?
 * Set to: "1" ==> YES
 *       : "0" ==> NO
 * ***********************************
 * UPDATE: This value is now tied to the
 * 'DO_RCU_OUTPUT_BUFFERS' flag value
 * because, for proper implementations
 * of buffer flushing, we MUST have
 * an RCU-synchronized output buffering
 * mechanism!!!
 * ***********************************
 */
#define DO_PERIODIC_BUFFER_FLUSH DO_RCU_OUTPUT_BUFFERS
/*
 * Do we use a TPS "epoch" counter to try and
 * order SCHED_WAKEUP samples and TPS samples?
 * (Required on many-core architectures that don't have
 * a synchronized TSC).
 */
#define DO_TPS_EPOCH_COUNTER 1
/*
 * Should the driver count number of dropped samples?
 */
#define DO_COUNT_DROPPED_SAMPLES 1
/*
 * Should we allow the driver to terminate the wuwatch userspace
 * application dynamically?
 * Used ONLY by the 'suspend_notifier' in cases when the driver
 * detects the application should exit because the device
 * was in ACPI S3 for longer than the collection time.
 * DISABLED, FOR NOW
 */
#define DO_ALLOW_DRIVER_TERMINATION_OF_WUWATCH 0


#define NUM_SAMPLES_PER_SEG 512
#define SAMPLES_PER_SEG_MASK 511 /* MUST be (NUM_SAMPLES_PER_SEG - 1) */
#if 1
    #define NUM_SEGS_PER_BUFFER 2 /* MUST be POW-of-2 */
    #define NUM_SEGS_PER_BUFFER_MASK 1 /* MUST be (NUM_SEGS_PER_BUFFER - 1) */
#else
    #define NUM_SEGS_PER_BUFFER 4 /* MUST be POW-of-2 */
    #define NUM_SEGS_PER_BUFFER_MASK 3 /* MUST be (NUM_SEGS_PER_BUFFER - 1) */
#endif

#define SEG_SIZE (NUM_SAMPLES_PER_SEG * sizeof(PWCollector_sample_t))

#ifndef PAGE_SIZE
#define PAGE_SIZE 4096
#endif

/*
 * The MAX number of entries in the "trace" array of the "k_sample_t" structure.
 * If the actual backtrace is longer, multiple
 * k_sample structs need to be chained together (see "sample_len" in
 * the "sample" struct).
 */
#define PW_TRACE_LEN 11
#define TRACE_LEN PW_TRACE_LEN // required by PERFRUN
/*
 * Max size of a module name. Ideally we'd
 * like to directly include "module.h" (which
 * defines this value), but this code
 * will be shared with Ring-3 code, which is
 * why we redefine it here.
 */
#define PW_MODULE_NAME_LEN (64 - sizeof(unsigned long))
/*
 * MAX size of each irq name (bytes).
 */
#define PW_IRQ_DEV_NAME_LEN	100
/*
 * MAX size of each proc name.
 */
#define PW_MAX_PROC_NAME_SIZE 16
/*
 * MAX number of logical subsystems in south complex.
 * for Medfield platform
 */
#define MFD_MAX_LSS_NUM_IN_SC 31
/*
 * MAX number of logical subsystems in south complex.
 * for Clovertrail platform.
 */
#define CLV_MAX_LSS_NUM_IN_SC 25
/*
 * MAX number of logical subsystems.
 * Choose whichever is the maximum among available platforms
 * defined above
 */
#define MAX_LSS_NUM_IN_SC 31

/*
 * MAX number of logical subsystems in north complex.
 * for Medfield platform
 */
#define MFD_MAX_LSS_NUM_IN_NC 9
/*
 * MAX number of logical subsystems in north complex.
 * for Clovertrail platform
 */
#define CLV_MAX_LSS_NUM_IN_NC 7
/*
 * MAX number of logical subsystems.
 * Choose whichever is the maximum among available platforms
 * defined above
 */
#define MAX_LSS_NUM_IN_NC 9

/*
 * MAX size of each wakelock name.
 */
#define PW_MAX_WAKELOCK_NAME_SIZE 76
/*
 * Device {short, long} names.
 * Used for MFLD.
 */
#define PW_MAX_DEV_SHORT_NAME_SIZE 10
#define PW_MAX_DEV_LONG_NAME_SIZE 80
/*
 * Package names used for Android OS.
 */
#define PW_MAX_PKG_NAME_SIZE 80
/*
 * Max # of 'd_residency' counters present
 * in a single 'd_residency_sample' instance.
 */
#define PW_MAX_DEVICES_PER_SAMPLE 2
/*
 * MAX number of mappings per block.
 */
#define PW_MAX_NUM_IRQ_MAPPINGS_PER_BLOCK 16
/*
 * MAX size of each irq name (bytes).
 */
#define PW_MAX_IRQ_NAME_SIZE 32
/*
 * Max # of available frequencies.
 */
#define PW_MAX_NUM_AVAILABLE_FREQUENCIES 16 // should be enough!
/*
 * MAX number of mappings per block.
 */
#define PW_MAX_NUM_PROC_MAPPINGS_PER_BLOCK 32

/*
 * MSR counter stuff.
 *
 * Ultimately the list of MSRs to read (and the core MSR residency addresses)
 * will be specified by the "runss" tool (via the "PW_IOCTL_CONFIG" ioctl).
 *
 * For now, hardcoded to values for NHM.
 */
typedef enum {
    MPERF=0, // C0
    APERF, // C1
    C2,
    C3,
    C4,
    C5,
    C6,
    C7,
    C8,
    C9,
    /* C10, */
    /* C11, */
    MAX_MSR_ADDRESSES
} c_state_t;

/*
 * Enumeration of possible sample types.
 */
typedef enum {
    FREE_SAMPLE=0, /* Used (internally) to indicate a FREE entry */
    C_STATE, /* Used for c-state samples */
    P_STATE, /* Used for p-state samples */
    K_CALL_STACK, /* Used for kernel-space call trace entries */
    M_MAP, /* Used for module map info samples */
    IRQ_MAP, /* Used for IRQ # <-> DEV name mapping samples */
    PROC_MAP, /* Used for PID <-> PROC name mapping samples */
    S_RESIDENCY, /* Used for S residency counter samples */
    S_STATE, /* Used for S state samples */
    D_RESIDENCY, /* Used for D residency counter samples */
    D_STATE, /* Used for D state samples in north or south complex */
    TIMER_SAMPLE,
    IRQ_SAMPLE,
    WORKQUEUE_SAMPLE,
    SCHED_SAMPLE,
    IPI_SAMPLE,
    TPE_SAMPLE, /*  Used for 'trace_power_end' samples */
    W_STATE, /* Used for kernel wakelock samples */
    DEV_MAP, /* Used for NC and SC device # <-> DEV name mapping samples */
    C_STATE_MSR_SET, /* Used to send an initial snapshot of the various C-state MSRs */
    U_STATE, /* Used for user wakelock samples */
    TSC_POSIX_MONO_SYNC, /* Used to sync TSC <-> posix CLOCK_MONOTONIC timers; REQUIRED for AXE support */ 
    CONSTANT_POOL_ENTRY, /* Used to send constant pool information */
    PKG_MAP, /* Used to send UID and package name mappings used for Android */
    CPUHOTPLUG_SAMPLE, /* Used to note when a CPU goes online or offline in ARM systems */
    SAMPLE_TYPE_END
} sample_type_t;

/*
 * Enumeration of possible C-state sample
 * types.
 */

typedef enum{
    PW_BREAK_TYPE_I=0, // interrupt
    PW_BREAK_TYPE_T, // timer
    PW_BREAK_TYPE_S, // sched-switch
    PW_BREAK_TYPE_IPI, // {LOC, RES, CALL, TLB}
    PW_BREAK_TYPE_W, // workqueue
    PW_BREAK_TYPE_B, // begin
    PW_BREAK_TYPE_N, // Not-a-break: used exclusively for CLTP support: DEBUGGING ONLY!
    PW_BREAK_TYPE_A, // Abort
    PW_BREAK_TYPE_U // unknown
}c_break_type_t;

#pragma pack(push) /* Store current alignment */
#pragma pack(2) /* Set new alignment -- 2 byte boundaries */

/*
 * A c-state msg.
 */
typedef struct c_msg {
    u64 mperf;
    u64 cx_msr_val; // The value in the C-state MSR pointed to by 'act_state'
    u64 wakeup_tsc; // The TSC when the wakeup event was handled.
    u64 wakeup_data; // Domain-specific wakeup data. Corresponds to "c_data" under old scheme.
    /*
     * In cases of timer-related wakeups, encode the CPU on which the timer was
     * initialized (and whose init TSC is encoded in the 'wakeup_data' field).
     */
    s32 timer_init_cpu;
    /*
     * 's32' for wakeup_{pid,tid} is overkill: '/proc/sys/kernel/pid_max' is almost always
     * 32768, which will fit in 's16'. However, it IS user-configurable, so we must 
     * accomodate larger pids.
     */
    s32 wakeup_pid;
    s32 wakeup_tid;
    u32 tps_epoch;
    u8 wakeup_type; // instance of 'c_break_type_t'
    u8 req_state; // State requested by OS: "HINT" parameter passed to TPS probe
    u8 act_state; // State granted by hardware: the MSR that counted, and whose residency is encoded in the 'cx_res' field
} c_msg_t;

/*
 * A p-state sample: MUST be 54 bytes EXACTLY!!!
 */
typedef struct p_msg {
    /*
     * The frequency the OS requested during the LAST TPF, in KHz.
     */
    u32 prev_req_frequency;
    /*
     * The value of the IA32_PERF_STATUS register: multiply bits 12:8 (Atom) or 15:0 (big-core)
     * with the BUS clock frequency to get actual frequency the HW was executing at.
     */
    u16 perf_status_val;
    /*
     * We encode the frequency at the start
     * and end of a collection in 'boundary'
     * messages. This flag is set for such
     * messages.
     */
    u16 is_boundary_sample;
} p_msg_t;

/*
 * The 'type' of the associated
 * 'u_sample'.
 */
typedef enum{
    PW_WAKE_ACQUIRE, // Wake lock
    PW_WAKE_RELEASE // Wake unlock
}u_sample_type_t;

/*
 * The 'type' of the associated
 * 'u_sample'.
 */
typedef enum{
    PW_WAKE_PARTIAL, // PARTIAL_WAKE_LOCK 
    PW_WAKE_FULL, // FULL_WAKE_LOCK
    PW_WAKE_SCREEN_DIM, // SCREEN_DIM_WAKE_LOCK
    PW_WAKE_SCREEN_BRIGHT, // SCREEN_BRIGHT_WAKE_LOCK
    PW_WAKE_PROXIMITY_SCREEN_OFF  // PROXIMITY_SCREEN_OFF_WAKE_LOCK
}u_sample_flag_t;

/*
 * Wakelock sample
 */
typedef struct{
    u_sample_type_t type;   // Either WAKE_ACQUIRE or WAKE_RELEASE 
    u_sample_flag_t flag;   // Wakelock flag
    pid_t pid, uid;
    u32 count;
    char tag[PW_MAX_WAKELOCK_NAME_SIZE]; // Wakelock tag
}u_sample_t;

/*
 * Generic "event" sample.
 */
typedef struct event_sample {
    u64 data[6];
} event_sample_t;

/*
 * The 'type' of the associated
 * 'd_residency_sample'.
 */
typedef enum {
    PW_NORTH_COMPLEX, // North complex
    PW_SOUTH_COMPLEX  // South complex
} device_type_t;

/*
 * 'D-residency' information.
 */
typedef struct event_sample d_residency_data_t;

/*
 * Device state (a.k.a. D state) residency counter sample
 * For now, ASSUME we have 40 devices
 */
typedef struct d_residency_msg {
    u8 device_type; // One of 'device_type_t'
    u8 num_sampled;
    // u8 mask; // Each bit indicates whether LSS residency is counted or not.
    u64 mask; // Each bit indicates whether LSS residency is counted or not.
             // 1 means "counted", 0 means "not counted"
    d_residency_data_t d_residency_counters[40]; // "40" ==> We have 40 SC devices
} d_residency_msg_t;

/*
 * Kernel wakelock information.
 */
typedef struct constant_pool_msg {
    u16 entry_type; // one of 'W_STATE' for kernel mapping or 'U_STATE' for userspace mapping
    u16 entry_len;
    /*
     * We need to differentiate between the two types of 'W_STATE' constant-pool entries:
     * 1. Entries generated in Ring-3 (as a result of parsing the "/proc/wakelocks" file). These are generated at
     *    the START of a collection and have a 'w_sample_type_t' value of 'PW_WAKE_LOCK_INITAL'.
     * 2. Entries generated in Ring-0 DURING the collection.
     * All examples of (1) will have the MSB set to '1'. Examples of (2) will not be bitmasked in any way.
     */
    u32 entry_index;
    char entry[1]; // MUST be LAST entry in struct!
} constant_pool_msg_t;
#define PW_CONSTANT_POOL_MSG_HEADER_SIZE (sizeof(constant_pool_msg_t) - sizeof(char[1]))
#define PW_CONSTANT_POOL_INIT_ENTRY_MASK (1U << 31)
#define PW_SET_INITIAL_W_STATE_MAPPING_MASK(idx) ( (idx) | PW_CONSTANT_POOL_INIT_ENTRY_MASK )
#define PW_HAS_INITIAL_W_STATE_MAPPING_MASK(idx) ( (idx) & PW_CONSTANT_POOL_INIT_ENTRY_MASK ) /* MSB will be SET if 'PW_WAKE_LOCK_INITIAL' mapping */
#define PW_STRIP_INITIAL_W_STATE_MAPPING_MASK(idx) ( (idx) & ~PW_CONSTANT_POOL_INIT_ENTRY_MASK )

typedef struct w_wakelock_msg {
    u16 type; // one of 'w_sample_type_t'
    pid_t tid, pid;
    u32 constant_pool_index;
    u64 expires;
    char proc_name[PW_MAX_PROC_NAME_SIZE];
} w_wakelock_msg_t;

typedef struct u_wakelock_msg {
    u16 type; // One of 'u_sample_type_t'
    u16 flag; // One of 'u_sample_flag_t'
    pid_t pid, uid;
    u32 count;
    u32 constant_pool_index;
} u_wakelock_msg_t;

/*
 * TSC_POSIX_MONO_SYNC
 * TSC <-> Posix clock_gettime() sync messages.
 */
typedef struct tsc_posix_sync_msg {
    pw_u64_t tsc_val;
    pw_u64_t posix_mono_val;
} tsc_posix_sync_msg_t;

/*
 * The main PWCollector_sample structure. ALL Ring 0 --> Ring 3 (data) messages
 * are encoded in these.
 */
typedef struct PWCollector_msg PWCollector_msg_t;
struct PWCollector_msg {
    u64 tsc; /* The time at which the message was enqueued */
    u16 data_len; /* The size of the payload i.e. the size of the message pointed to by 'p_data' */
    u8 cpuidx; /* The (logical) processor which sent the message */
    u8 data_type; /* The message type, one of sample_type_t */
    u64 p_data; /* The actual payload */
};

#define PW_MSG_HEADER_SIZE ( sizeof(PWCollector_msg_t) - sizeof(u64) )

#pragma pack(pop) /* Restore previous alignment */




/*
 * Structure used to encode C-state sample information.
 */
typedef struct c_sample {
    u16 break_type; // instance of 'c_break_type_t'
    u16 prev_state; // "HINT" parameter passed to TPS probe
    pid_t pid; // PID of process which caused the C-state break.
    pid_t tid; // TID of process which caused the C-state break.
    u32 tps_epoch; // Used to sync with SCHED_SAMPLE events
    /*
     * "c_data" is one of the following:
     * (1) If "c_type" == 'I' ==> "c_data" is the IRQ of the interrupt
     * that caused the C-state break.
     * (2) If "c_type" == 'D' || 'N' => "c_data" is the TSC that maps to the 
     * user-space call trace ID for the process which caused the C-state break.
     * (3) If "c_type" == 'U' ==> "c_data" is undefined.
     */
    u64 c_data;
    u64 c_state_res_counts[MAX_MSR_ADDRESSES];
} c_sample_t;

#define RES_COUNT(s,i) ( (s).c_state_res_counts[(i)] )

/*
 * Structure used to encode P-state transition information.
 *
 * UPDATE: For TURBO: for now, we only encode WHETHER the CPU is
 * about to TURBO-up; we don't include information on WHICH Turbo
 * frequency the CPU will execute at. See comments in struct below
 * for an explanation on why the 'frequency' field values are
 * unreliable in TURBO mode.
 */
typedef struct p_sample {
    /*
     * Field to encode the frequency
     * the CPU was ACTUALLY executing
     * at DURING THE PREVIOUS 
     * P-QUANTUM.
     */
    u32 frequency;
    /*
     * Field to encode the frequency
     * the OS requested DURING THE
     * PREVIOUS P-QUANTUM.
     */
    u32 prev_req_frequency;
    /*
     * We encode the frequency at the start
     * and end of a collection in 'boundary'
     * messages. This flag is set for such
     * messages.
     */
    u32 is_boundary_sample;
    u32 padding;
    /*
     * The APERF and MPERF values.
     */
    u64 unhalted_core_value, unhalted_ref_value;
} p_sample_t;


/*
 * Structure used to encode kernel-space call trace information.
 */
typedef struct k_sample {
    /*
     * "trace_len" indicates the number of entries in the "trace" array.
     * Note that the actual backtrace may be larger -- in which case the "sample_len"
     * field of the enclosing "struct PWCollector_sample" will be greater than 1.
     */
    u32 trace_len;
    /*
     * We can have root timers with non-zero tids.
     * Account for that possibility here.
     */
    pid_t tid;
    /*
     * The entry and exit TSC values for this kernel call stack.
     * MUST be equal to "[PWCollector_sample.tsc - 1, PWCollector_sample.tsc + 1]" respectively!
     */
    u64 entry_tsc, exit_tsc;
    /*
     * "trace" contains the kernel-space call trace.
     * Individual entries in the trace correspond to the various
     * return addresses in the call trace, shallowest address first.
     * For example: if trace is: "0x10 0x20 0x30 0x40" then 
     * the current function has a return address of 0x10, its calling function
     * has a return address of 0x20 etc.
     */
    u64 trace[TRACE_LEN];
} k_sample_t;


/*
 * Structure used to encode kernel-module map information.
 */
typedef struct m_sample {
    /*
     * Offset of current chunk, in case a kernel module is 
     * mapped in chunks. DEFAULTS TO ZERO!
     */
    u32 offset;
    /*
     * Compiler would have auto-padded this for us, but
     * we make that padding explicit just in case.
     */
    u32 padding_64b;
    /*
     * The starting addr (in HEX) for this module.
     */
    u64 start;
    /*
     * The ending addr (in HEX) for this module.
     */
    u64 end;
    /*
     * Module NAME. Note that this is NOT the full
     * path name. There currently exists no way
     * of extracting path names from the module
     * structure.
     */
    char name[PW_MODULE_NAME_LEN];
} m_sample_t;


/*
 * Structure used to encode IRQ # <-> DEV name
 * mapping information.
 */
typedef struct i_sample {
    /*
     * The IRQ #
     */
    int irq_num;
    /*
     * Device name corresponding
     * to 'irq_num'
     */
    char irq_name[PW_IRQ_DEV_NAME_LEN];
} i_sample_t;

/*
 * The 'type' of the associated
 * 'r_sample'.
 */
typedef enum r_sample_type {
    PW_PROC_FORK, /* Sample encodes a process FORK */
    PW_PROC_EXIT, /* Sample encodes a process EXIT */
    PW_PROC_EXEC /* Sample encodes an EXECVE system call */
} r_sample_type_t;

typedef struct r_sample {
    u32 type;
    pid_t tid, pid;
    char proc_name[PW_MAX_PROC_NAME_SIZE];
} r_sample_t;

/*
 * Platform state (a.k.a. S state) residency counter sample
 */
typedef struct event_sample s_residency_sample_t;
/*
 * Platform state (a.k.a. S state) sample
 */
typedef struct s_state_sample {
    u32 state; // S-state
} s_state_sample_t;


typedef struct event_sample d_residency_t;

/*
 * Device state (a.k.a. D state) residency counter sample
 */
typedef struct d_residency_sample {
    u16 device_type;     // Either NORTH_COMPLEX or SOUTH_COMPLEX
    u16 num_sampled;
    u16 mask[PW_MAX_DEVICES_PER_SAMPLE]; // Each bit indicates whether LSS residency is counted or not.
                // 1 means "counted", 0 means "not counted"
                // The last byte indicates the number of LSSes sampled 
    d_residency_t d_residency_counters[PW_MAX_DEVICES_PER_SAMPLE]; // we can fit at most '2' samples in every 'PWCollector_sample_t'
} d_residency_sample_t;

/*
 * Device state (a.k.a. D state) sample from north or south complex
 */
typedef struct d_state_sample {
    char device_type;     // Either NORTH_COMPLEX or SOUTH_COMPLEX
    u32 states[4]; // Each device state is represented in 2 bits
} d_state_sample_t;

/*
 * The 'type' of the associated
 * 'w_sample'.
 */
typedef enum w_sample_type {
    PW_WAKE_LOCK, // Wake lock
    PW_WAKE_UNLOCK, // Wake unlock
    PW_WAKE_LOCK_TIMEOUT, // Wake lock with timeout
    PW_WAKE_LOCK_INITIAL, // Wake locks acquired before collection
    PW_WAKE_UNLOCK_ALL // All previously held wakelocks have been unlocked -- used in ACPI S3 notifications
} w_sample_type_t;

/*
 * Wakelock sample
 */
typedef struct w_sample {
    w_sample_type_t type;   // Wakelock type
    pid_t tid, pid;
    char name[PW_MAX_WAKELOCK_NAME_SIZE]; // Wakelock name
    u64 expires; // wakelock timeout in tsc if type is equal to PW_WAKE_LOCK_TIMEOUT,
                 // otherwise 0
    char proc_name[PW_MAX_PROC_NAME_SIZE]; // process name
} w_sample_t;

/*
 * Structure to return Dev # <-> Dev Name mappings.
 */
typedef struct dev_sample {
    u32 dev_num;
    u32 dev_type; // one of "device_type_t"
    char dev_short_name[PW_MAX_DEV_SHORT_NAME_SIZE];
    char dev_long_name[PW_MAX_DEV_LONG_NAME_SIZE];
} dev_sample_t;

/*
 * Structure to return UID # <-> Package Name mappings.
 */
typedef struct pkg_sample {
    u32 uid;
    char pkg_name[PW_MAX_PKG_NAME_SIZE];
} pkg_sample_t;

/*
 * The C/P/K/S sample structure.
 */
typedef struct PWCollector_sample {
    u32 cpuidx;
    u16 sample_type; // The type of the sample: one of "sample_type_t"
    /*
     * "sample_len" is useful when stitching together 
     * multiple PWCollector_sample instances.
     * This is used in cases where the kernel-space call 
     * trace is very large, and cannot fit within one K-sample.
     * We can stitch together a MAX of
     * 256 K-samples.
     */
    u16 sample_len;
    u64 tsc; // The TSC at which the measurement was taken
    union {
	c_sample_t c_sample;
	p_sample_t p_sample;
	k_sample_t k_sample;
	m_sample_t m_sample;
	i_sample_t i_sample;
	r_sample_t r_sample;
	s_residency_sample_t s_residency_sample;
	s_state_sample_t s_state_sample;
	d_state_sample_t d_state_sample;
	d_residency_sample_t d_residency_sample;
	w_sample_t w_sample;
	u_sample_t u_sample;
	event_sample_t e_sample;
        dev_sample_t dev_sample;
        pkg_sample_t pkg_sample;
    };
} PWCollector_sample_t;


typedef enum PWCollector_cmd {
    PW_START=1,
    PW_DETACH,
    PW_PAUSE,
    PW_RESUME,
    PW_STOP,
    PW_CANCEL,
    PW_SNAPSHOT,
    PW_STATUS,
    PW_MARK
} PWCollector_cmd_t;

typedef enum power_data {
    PW_SLEEP=0, /* DD should register all timer and sleep-related tracepoints */
    PW_KTIMER, /* DD should collect kernel call stacks */
    PW_FREQ, /* DD should collect P-state transition information */
    PW_PLATFORM_RESIDENCY, /* DD should collect S-state residency information */
    PW_PLATFORM_STATE, /* DD should collect S-state samples */
    PW_DEVICE_SC_RESIDENCY, /* DD should collect South-Complex D-state residency information */
    PW_DEVICE_NC_STATE, /* DD should collect North-Complex D-state samples */
    PW_DEVICE_SC_STATE, /* DD should collect South-Complex D-state samples */
    PW_WAKELOCK_STATE, /* DD should collect wakelock samples */
    PW_POWER_C_STATE, /* DD should collect C-state samples */
    PW_MAX_POWER_DATA_MASK /* Marker used to indicate MAX valid 'power_data_t' enum value -- NOT used by DD */
} power_data_t;

#define POWER_SLEEP_MASK (1 << PW_SLEEP)
#define POWER_KTIMER_MASK (1 << PW_KTIMER)
#define POWER_FREQ_MASK (1 << PW_FREQ)
#define POWER_S_RESIDENCY_MASK (1 << PW_PLATFORM_RESIDENCY)
#define POWER_S_STATE_MASK (1 << PW_PLATFORM_STATE)
#define POWER_D_SC_RESIDENCY_MASK (1 << PW_DEVICE_SC_RESIDENCY)
#define POWER_D_SC_STATE_MASK (1 << PW_DEVICE_SC_STATE)
#define POWER_D_NC_STATE_MASK (1 << PW_DEVICE_NC_STATE)
#define POWER_WAKELOCK_MASK (1 << PW_WAKELOCK_STATE)
#define POWER_C_STATE_MASK ( 1 << PW_POWER_C_STATE )

#define SET_COLLECTION_SWITCH(m,s) ( (m) |= (1 << (s) ) )
#define RESET_COLLECTION_SWITCH(m,s) ( (m) &= ~(1 << (s) ) )
#define WAS_COLLECTION_SWITCH_SET(m, s) ( (m) & (1 << (s) ) )

/*
 * Platform-specific config struct.
 */
typedef struct platform_info {
    int residency_count_multiplier;
    int bus_clock_freq_khz;
    int coreResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    int pkgResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    u64 reserved[3];
} platform_info_t;

/*
 * Config Structure. Includes platform-specific
 * stuff and power switches.
 */
struct PWCollector_config {
    int data;
    u32 d_state_sample_interval;  // This is the knob to control the frequency of D-state data sampling 
    // to adjust their collection overhead in the unit of msec.
    platform_info_t info;
};

/*
 * Some constants used to describe kernel features
 * available to the power driver.
 */
#define PW_KERNEL_SUPPORTS_CALL_STACKS (1 << 0)
#define PW_KERNEL_SUPPORTS_CONFIG_TIMER_STATS (1 << 1)
#define PW_KERNEL_SUPPORTS_WAKELOCK_PATCH (1 << 2)
/*
 * Some constants used to describe arch features enabled
 */
#define PW_ARCH_ANY_THREAD_SET (1 << 0)
#define PW_ARCH_AUTO_DEMOTE_ENABLED (1 << 1)

/*
 * Structure to encode unsupported tracepoints and
 * kernel features that enable power collection.
 */
struct PWCollector_check_platform {
    char unsupported_tracepoints[4096];
    /*
     * Bitwise 'OR' of zero or more of the
     * 'PW_KERNEL_SUPPORTS_' constants described
     * above.
     */
    u32 supported_kernel_features;
    u32 supported_arch_features;
    u64 reserved[3];
};

/*
 * Structure to return status information.
 */
struct PWCollector_status {
    u32 num_cpus;
    u32 time;
    u32 c_breaks;
    u32 timer_c_breaks;
    u32 inters_c_breaks;
    u32 p_trans;
    u32 num_inters;
    u32 num_timers;
};

/*
 * Structure to return version information.
 */
struct PWCollector_version_info {
    int version;
    int inter;
    int other;
};

/*
 * Structure to return device information.
 */
struct PWCollector_device_info {
    u64 undefined_devices;
};

/*
 * Structure to return specific microcode
 * patch -- for MFLD development steppings.
 */
struct PWCollector_micro_patch_info {
    u32 patch_version;
};

/*
 * Helper struct for IRQ <-> DEV name mappings.
 */
typedef struct PWCollector_irq_mapping {
	int irq_num;
	char irq_name[PW_MAX_IRQ_NAME_SIZE];
} PWCollector_irq_mapping_t;
/*
 * Structure to return IRQ <-> DEV name mappings.
 */
struct PWCollector_irq_mapping_block {
	/*
	 * INPUT param: if >= 0 ==> indicates
	 * the client wants information for
	 * a SPECIFIC IRQ (and does not want
	 * ALL mappings).
	 */
	int requested_irq_num;
	/*
	 * OUTPUT param: records number of
	 * valid entries in the 'mappings'
	 * array.
	 */
	int size;
	/*
	 * INPUT/OUTPUT param: records from which IRQ
	 * entry the client wants mapping info.
	 * Required because the driver
	 * may return LESS than the total number
	 * of IRQ mappings, in which case the client
	 * is expected to call this IOCTL
	 * again, specifying the offset.
	 */
	int offset;
	/*
	 * The array of mappings.
	 */
	PWCollector_irq_mapping_t mappings[PW_MAX_NUM_IRQ_MAPPINGS_PER_BLOCK];
};


typedef struct PWCollector_proc_mapping {
    pid_t pid, tid;
    char name[PW_MAX_PROC_NAME_SIZE];
} PWCollector_proc_mapping_t;
/*
 * Structure to return PID <-> PROC name mappings.
 */
struct PWCollector_proc_mapping_block {
    /*
     * OUTPUT param: records number of
     * valid entries in the 'mappings'
     * array.
     */
    int size;
    /*
     * INPUT/OUTPUT param: records from which PROC
     * entry the client wants mapping info.
     * may return LESS than the total number
     * of PROC mappings, in which case the client
     * is expected to call this IOCTL
     * again, specifying the offset.
     */
    int offset;
    /*
     * The array of mappings.
     */
    PWCollector_proc_mapping_t mappings[PW_MAX_NUM_PROC_MAPPINGS_PER_BLOCK];
};


/*
 * Structure to return TURBO frequency
 * threshold.
 */
struct PWCollector_turbo_threshold {
    u32 threshold_frequency;
};

/*
 * Structure to return 'available
 * frequencies' i.e. the list of
 * frequencies the processor
 * may execute at.
 */
struct PWCollector_available_frequencies {
    /*
     * Number of valid entries in the
     * 'frequencies' array -- supplied
     * by the DD.
     */
    u32 num_freqs;
    /*
     * List of available frequencies, in kHz.
     */
    u32 frequencies[PW_MAX_NUM_AVAILABLE_FREQUENCIES];
};

/*
 * Wrapper for ioctl arguments.
 * EVERY ioctl MUST use this struct!
 */
struct PWCollector_ioctl_arg {
    int in_len;
    int out_len;
    const char *in_arg;
    char *out_arg;
};

#endif // _DATA_STRUCTURES_H_
