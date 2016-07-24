/*
 * cyttsp5_mt_common.h
 * Cypress TrueTouch(TM) Standard Product V5 Multi-Touch Reports Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <linux/cyttsp5_bus.h>

#include <linux/delay.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#endif

#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_mt.h>
#include "cyttsp5_regs.h"

struct cyttsp5_mt_data;
struct cyttsp5_mt_function {
	int (*mt_release)(struct cyttsp5_device *ttsp);
	int (*mt_probe)(struct cyttsp5_device *ttsp,
			struct cyttsp5_mt_data *md);
	void (*report_slot_liftoff)(struct cyttsp5_mt_data *md, int max_slots);
	void (*input_sync)(struct input_dev *input);
	void (*input_report)(struct input_dev *input, int sig,
			int t, int type);
	void (*final_sync)(struct input_dev *input, int max_slots,
			int mt_sync_count, unsigned long *ids);
	int (*input_register_device)(struct input_dev *input, int max_slots);
};

struct gesture_state {
	int event_count;
	int min_x;
	int max_x;
	int min_y;
	int max_y;
	int max_touches;
	int start_time;
	int end_time;
};

struct cyttsp5_mt_data {
	struct cyttsp5_device *ttsp;
	struct cyttsp5_mt_platform_data *pdata;
	struct cyttsp5_sysinfo *si;
	struct input_dev *input;
	struct cyttsp5_mt_function mt_function;
	struct mutex mt_lock;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend es;
	bool is_suspended;
#endif
	bool input_device_registered;
	bool volume_enabled;
	int volume_keepout;
	char phys[NAME_MAX];
	int num_prv_tch;
#ifdef VERBOSE_DEBUG
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
#endif
	struct gesture_state gs;
#ifdef CONFIG_WAKELOCK
	struct wake_lock wakelock;
#endif
};

void cyttsp5_init_function_ptrs(struct cyttsp5_mt_data *md);
extern struct cyttsp5_driver cyttsp5_mt_driver;

