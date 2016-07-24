/* i915_debugfs.h -- Private header for the I915 driver debugfs interface -*-
 */
/*
 * Copyright 2013 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Author:
 * Deepak S <deepak.s@intel.com>
 */

#ifndef _I915_DEBUGFS_H_
#define _I915_DEBUGFS_H_

/* Header file for Color management through debugfs */
#include "intel_clrmgr.h"

/* Operations supported
*/
#define MAX_BUFFER_STR_LEN	200

#define READ_TOKEN	        "READ"
#define WRITE_TOKEN	        "WRITE"

#define STATUS_TOKEN		"STATUS"
#define ENABLE_TOKEN		"ENABLE"
#define DISABLE_TOKEN		"DISABLE"
#define DETAILS_TOKEN		"DETAILS"

#define IOSF_FUSE_TOKEN	"FUSE"
#define IOSF_PUNIT_TOKEN	"PUNIT"
#define IOSF_CCU_TOKEN	"CCU"

/* RC6 Operations */
#define RC6_POWER_TOKEN	"PWRWELLS"
#define RC6_SINGLETHREAD_TOKEN	"SINGLETHREAD"
#define MULTITHREAD_TOKEN	"MULTITHREAD"
#define READ_COUNTER_0_TOKEN	"COUNTER_0"
#define READ_COUNTER_1_TOKEN	"COUNTER_1"
#define READ_COUNTER_6_TOKEN	"COUNTER_6"

/* RP(Turbo) Operations */
#define RP_MAXFREQ_TOKEN	"SETMAXFREQ"
#define RP_MINFREQ_TOKEN        "SETMINFREQ"

/* DPST Operations */
#define DPST_DUMP_REG_TOKEN     "DUMP_REG"
#define DPST_FACTOR_TOKEN	"BACKLIGHT_FACTOR"
#define DPST_LEVEL_TOKEN	"CUR_LEVEL"
#define DPST_GET_BIN_DATA_TOKEN	"GET_BIN_DATA"
#define DPST_GET_LUMA_DATA_TOKEN	"GET_LUMA_DATA"
#define DPST_IRQ_COUNT_TOKEN	"INTERRUPT_COUNT"

/* RPM(S0iX) Operations */
#define RPM_DISPLAY_RUNTIME_SUSPEND_TOKEN       "DISPLAY_SUSPEND"
#define RPM_DISPLAY_RUNTIME_RESUME_TOKEN        "DISPLAY_RESUME"

/* DebugFS Variable declaration */
struct debugfs_mmio_vars {
	char mmio_vars[MAX_BUFFER_STR_LEN];
	u32 mmio_input;
};

struct debugfs_iosf_vars {
	char iosf_vars[MAX_BUFFER_STR_LEN];
	u32 iosf_input;
};

struct debugfs_rc6_vars {
	char rc6_vars[MAX_BUFFER_STR_LEN];
	u32 rc6_input;
};

struct debugfs_rpm_vars {
	char rpm_vars[MAX_BUFFER_STR_LEN];
	u32 rpm_input;
};

struct debugfs_turbo_vars {
	char turbo_vars[MAX_BUFFER_STR_LEN];
	u32 turbo_input;
};

struct debugfs_dpst_vars {
	char dpst_vars[MAX_BUFFER_STR_LEN];
	u32 dpst_input;
};

union {
	struct debugfs_mmio_vars mmio;
	struct debugfs_iosf_vars iosf;
	struct debugfs_rc6_vars rc6;
	struct debugfs_rpm_vars rpm;
	struct debugfs_turbo_vars turbo;
	struct debugfs_dpst_vars dpst;
} i915_debugfs_vars;

enum {
	ACTIVE_LIST,
	INACTIVE_LIST,
	PINNED_LIST,
};

#endif /* _I915_DEBUGFS_H_ */
