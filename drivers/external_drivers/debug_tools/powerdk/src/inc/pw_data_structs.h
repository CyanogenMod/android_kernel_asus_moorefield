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
 * Description: file containing internal data structures used by the
 * power driver.
 */

#ifndef _PW_DATA_STRUCTS_H_
#define _PW_DATA_STRUCTS_H_ 1

/*
 * We're the PWR kernel device driver.
 * Flag needs to be set BEFORE
 * including 'pw_ioctl.h'
 */
#define PW_KERNEL_MODULE 1

#include "pw_ioctl.h" // For IOCTL mechanism
#include <linux/fs.h>
#include <linux/bitops.h> // for "test_and_set_bit(...)" atomic functionality


enum{
    EMPTY=0,
    FULL
};

enum{
    IRQ=0,
    TIMER,
    WORKQUEUE,
    SCHED,
    BREAK_TYPE_END
};

/*
 * Used to indicate whether
 * a new IRQ mapping was
 * created.
 */
typedef enum {
	OK_IRQ_MAPPING_EXISTS,
	OK_NEW_IRQ_MAPPING_CREATED,
	ERROR_IRQ_MAPPING
} irq_mapping_types_t;

typedef enum {
    PW_MAPPING_EXISTS,
    PW_NEW_MAPPING_CREATED,
    PW_MAPPING_ERROR
} pw_mapping_type_t;

/*
 * Structure to hold current CMD state
 * of the device driver. Constantly evolving, but
 * that's OK -- this is internal to the driver
 * and is NOT exported.
 */
typedef struct {
    PWCollector_cmd_t cmd; // indicates which command was specified last e.g. START, STOP etc.
    /*
     * Should we write to our per-cpu output buffers?
     * YES if we're actively collecting.
     * NO if we're not.
     */
    bool write_to_buffers;
    /*
     * Should we "drain/flush" the per-cpu output buffers?
     * (See "device_read" for an explanation)
     */
    bool drain_buffers;
    /*
     * Current methodology for generating kernel-space call
     * stacks relies on following frame pointer: has
     * the kernel been compiled with frame pointers?
     */
    bool have_kernel_frame_pointers;
    /*
     * On some archs, C-state residency MSRS do NOT count at TSC frequency. 
     * For these, we need to apply a "clock multiplier". Record that
     * here.
     */
    unsigned int residency_count_multiplier;
    /*
     * Store the bus clock frequency.
     */
    unsigned int bus_clock_freq_khz;
    /*
     * Core/Pkg MSR residency addresses
     */
    unsigned int coreResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    unsigned int pkgResidencyMSRAddresses[MAX_MSR_ADDRESSES];
    /*
     * What switches should the device driver collect?
     * Note: different from interface spec:
     * We're moving from bitwise OR to bitwise OR of (1 << switch) values.
     * Use the "POWER_XXX_MASK" masks to set/test switch residency.
     */
    int collection_switches;
    /*
     * Total time elapsed for
     * all collections.
     * Aggregated over all collections -- useful
     * in multiple PAUSE/RESUME scenarios
     */
    unsigned long totalCollectionTime;
    /*
     * Start and stop jiffy values for
     * the current collection.
     */
    unsigned long collectionStartJIFF, collectionStopJIFF;
    /*
     * This is the knob to control the frequency of D-state data sampling 
     * to adjust their collection overhead. By default, they are sampled 
     * in power_start traceevent after 100 msec is passed from the previous sample.
     */
    u32 d_state_sample_interval;

    // Others...
}internal_state_t;

static internal_state_t INTERNAL_STATE;

#define IS_COLLECTING() (INTERNAL_STATE.cmd == PW_START || INTERNAL_STATE.cmd == PW_RESUME)
#define IS_SLEEPING() (INTERNAL_STATE.cmd == PW_PAUSE)
#define IS_SLEEP_MODE() (INTERNAL_STATE.collection_switches & POWER_SLEEP_MASK)
#define IS_FREQ_MODE() (INTERNAL_STATE.collection_switches & POWER_FREQ_MASK)
#define IS_KTIMER_MODE() (INTERNAL_STATE.collection_switches & POWER_KTIMER_MASK)
#define IS_NON_PRECISE_MODE() (INTERNAL_STATE.collection_switches & POWER_SYSTEM_MASK)
#define IS_S_RESIDENCY_MODE() (INTERNAL_STATE.collection_switches & POWER_S_RESIDENCY_MASK)
#define IS_S_STATE_MODE() (INTERNAL_STATE.collection_switches & POWER_S_STATE_MASK)
#define IS_D_SC_RESIDENCY_MODE() (INTERNAL_STATE.collection_switches & POWER_D_SC_RESIDENCY_MASK)
#define IS_D_SC_STATE_MODE() (INTERNAL_STATE.collection_switches & POWER_D_SC_STATE_MASK)
#define IS_D_NC_STATE_MODE() (INTERNAL_STATE.collection_switches & POWER_D_NC_STATE_MASK)
#define IS_WAKELOCK_MODE() (INTERNAL_STATE.collection_switches & POWER_WAKELOCK_MASK)
/*
 * Special check to see if we should produce c-state samples.
 * Required to support S/D-state use of TPS probe (which requires "SLEEP_MASK") without
 * producing any C-state samples.
 */
#define IS_C_STATE_MODE() ( INTERNAL_STATE.collection_switches & POWER_C_STATE_MASK )


/*
 * Per-cpu structure holding MSR residency counts,
 * timer-TSC values etc.
 */
typedef struct per_cpu_struct {
	u32 was_timer_hrtimer_softirq; // 4 bytes
	void *sched_timer_addr; // 4/8 bytes (arch dependent)
}per_cpu_t;

/*
 * Per-cpu structure holding wakeup event causes, tscs
 * etc. Set by the first non-{TPS, TPE} event to occur
 * after a processor wakes up.
 */
struct wakeup_event {
    u64 event_tsc; // TSC at which the event occurred
    u64 event_val; // Event value -- domain-specific
    s32 init_cpu; // CPU on which a timer was initialized; valid ONLY for wakeups caused by timers!
    u32 event_type; // one of c_break_type_t enum values
    pid_t event_tid, event_pid;
};

/*
 * Convenience macros for accessing per-cpu residencies
 */
#define RESIDENCY(p,i) ( (p)->residencies[(i)] )
#define PREV_MSR_VAL(p,i) ( (p)->prev_msr_vals[(i)] )

/*
 * Per-cpu structure holding stats information.
 * Eventually, we may want to incorporate these fields within 
 * the "per_cpu_t" structure.
 */
typedef struct{
    local_t c_breaks, timer_c_breaks, inters_c_breaks;
    local_t p_trans;
    local_t num_inters, num_timers;
}stats_t;

/*
 * Per-cpu structure holding C-state MSR values.
 */
typedef struct msr_set msr_set_t;
struct msr_set {
    u64 prev_msr_vals[MAX_MSR_ADDRESSES];
    /*
     * Which 'Cx' MSRs counted during the previous C-state quantum(s)?
     * (We could have more than one in an HT environment -- it is NOT
     * guaranteed that a core wakeup will cause both threads to wakeup.)
     */
    u64 curr_msr_count[MAX_MSR_ADDRESSES];
    /*
     * What was the last C-state the OS requested?
     */
    u32 prev_req_cstate;
    /*
     * Have we sent the boundary C-state sample?
     * Required for an initial MSR set snapshot.
     */
    u8 init_msr_set_sent;
};

/*
 * Struct to hold old IA32_FIXED_CTR_CTRL MSR 
 * values (to enable restoring
 * after pw driver terminates). These are
 * used to enable/restore/disable CPU_CLK_UNHALTED.REF
 * counting.
 *
 * UPDATE: also store old IA32_PERF_GLOBAL_CTRL values..
 */
typedef struct {
    u32 fixed_data[2], perf_data[2];
} CTRL_values_t;

/*
 * Helper struct to extract (user-supplied)
 * IOCTL input and output lengths.
 */
typedef struct {
	int in_len, out_len;
	char data[1];
} ioctl_args_stub_t;

#if defined(HAVE_COMPAT_IOCTL) && defined(CONFIG_X86_64)
/*
 * Helper struct for use in translating
 * IOCTLs from 32b user programs in 64b
 * kernels.
 */
struct PWCollector_ioctl_arg32{
	int in_len, out_len;
	u32 in_arg, out_arg;
};
#endif // COMPAT && x64

#endif // _PW_DATA_STRUCTS_H_
