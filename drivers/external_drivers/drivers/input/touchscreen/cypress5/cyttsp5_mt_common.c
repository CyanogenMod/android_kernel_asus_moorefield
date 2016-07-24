/*
 * cyttsp5_mt_common.c
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

#include "cyttsp5_mt_common.h"

#ifdef CONFIG_WAKELOCK
#include <linux/wakelock.h>
#define WAKELOCK_TIMEOUT_IN_MS 2000
#endif

/* Time period in which a gesture tap may be detected in ms. */
static int gesture_tap_time_min = 5;
static int gesture_tap_time_max = 300;

#define GESTURE_ID_VOLUME_DOWN 0xa0
#define GESTURE_ID_VOLUME_UP   0xa1

extern struct mutex cyttsp5_sleep_lock;

static void gesture_accumulate(struct input_dev *input,
			       struct cyttsp5_mt_data *md,
			       struct cyttsp5_touch *tch, int num_cur_tch);

static void cyttsp5_mt_lift_all(struct cyttsp5_mt_data *md)
{
	int max = md->si->tch_abs[CY_TCH_T].max;

	gesture_accumulate(md->input, md, NULL, 0);

	if (md->num_prv_tch != 0) {
		if (md->mt_function.report_slot_liftoff)
			md->mt_function.report_slot_liftoff(md, max);
		input_sync(md->input);
		md->num_prv_tch = 0;
	}
}

static void cyttsp5_get_touch_axis(struct cyttsp5_mt_data *md,
	int *axis, int size, int max, u8 *xy_data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		dev_vdbg(&md->ttsp->dev,
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
			" xy_data[%d]=%02X(%d) bofs=%d\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next], bofs);
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;

	dev_vdbg(&md->ttsp->dev,
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
		" xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void cyttsp5_get_touch_hdr(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_mode)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_hdr hdr;

	for (hdr = CY_TCH_TIME; hdr < CY_TCH_NUM_HDR; hdr++) {
		if (!si->tch_hdr[hdr].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->hdr[hdr],
			si->tch_hdr[hdr].size,
			si->tch_hdr[hdr].max,
			xy_mode + si->tch_hdr[hdr].ofs,
			si->tch_hdr[hdr].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_hdr_string[hdr],
			touch->hdr[hdr], touch->hdr[hdr]);
	}

	dev_dbg(dev,
		"%s: time=%X tch_num=%d lo=%d noise=%d counter=%d\n",
		__func__,
		touch->hdr[CY_TCH_TIME],
		touch->hdr[CY_TCH_NUM],
		touch->hdr[CY_TCH_LO],
		touch->hdr[CY_TCH_NOISE],
		touch->hdr[CY_TCH_COUNTER]);
}

static void cyttsp5_get_touch(struct cyttsp5_mt_data *md,
	struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	enum cyttsp5_tch_abs abs;
	int tmp;
	bool flipped;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++) {
		if (!si->tch_abs[abs].report)
			continue;
		cyttsp5_get_touch_axis(md, &touch->abs[abs],
			si->tch_abs[abs].size,
			si->tch_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->tch_abs[abs].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_abs_string[abs],
			touch->abs[abs], touch->abs[abs]);
	}

	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		tmp = touch->abs[CY_TCH_X];
		touch->abs[CY_TCH_X] = touch->abs[CY_TCH_Y];
		touch->abs[CY_TCH_Y] = tmp;
		flipped = true;
	} else
		flipped = false;

	if (md->pdata->flags & CY_MT_FLAG_INV_X) {
		if (flipped)
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_X];
		else
			touch->abs[CY_TCH_X] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_X];
	}
	if (md->pdata->flags & CY_MT_FLAG_INV_Y) {
		if (flipped)
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_x -
				touch->abs[CY_TCH_Y];
		else
			touch->abs[CY_TCH_Y] = si->sensing_conf_data.res_y -
				touch->abs[CY_TCH_Y];
	}

	dev_vdbg(dev, "%s: flip=%s inv-x=%s inv-y=%s x=%04X(%d) y=%04X(%d)\n",
		__func__, flipped ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_X ? "true" : "false",
		md->pdata->flags & CY_MT_FLAG_INV_Y ? "true" : "false",
		touch->abs[CY_TCH_X], touch->abs[CY_TCH_X],
		touch->abs[CY_TCH_Y], touch->abs[CY_TCH_Y]);
}

/* Reset the gesture detector accumulator */
static void gesture_reset(struct gesture_state *gs)
{
	printk("[cyttsp5_mt] Touch started\n");
	memset(gs, 0, sizeof(*gs));
	/* Set max and min to extreme initialized values. */
	gs->max_x = INT_MIN;
	gs->max_y = INT_MIN;
	gs->min_x = INT_MAX;
	gs->min_y = INT_MAX;
}

/* Check for valid gesture */
static int gesture_valid(struct input_dev *input,
			   struct gesture_state *gs)
{
	u16 delta_msec;		/* u16 to handle timer wrap */
	printk("[cyttsp5_mt] Touch ended, event#=%d, finger#=%d, time=[%d, %d]"
	       " width=%d, height=%d\n",
	       gs->event_count, gs->max_touches, gs->start_time, gs->end_time,
	       gs->max_x-gs->min_x, gs->max_y-gs->min_y);
	/* timestamps are in ms */
	delta_msec = gs->end_time - gs->start_time;

	return (gs->event_count > 1)
		&& (gs->max_touches == 1)
		&& (delta_msec > gesture_tap_time_min)
		&& (delta_msec < gesture_tap_time_max);
}

static void gesture_accumulate(struct input_dev *input,
			       struct cyttsp5_mt_data *md,
			       struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct gesture_state *gs = &md->gs;
	if (md->num_prv_tch == 0 && num_cur_tch == 1) {
		/* First finger landed, begin gesture */
		gesture_reset(gs);
		gs->start_time = tch->hdr[CY_TCH_TIME];
	} else if (md->num_prv_tch && num_cur_tch == 0) {
		/* Last finger lifted, see if we captured a gesture.
		 * We don't send events unless the screen is off.
		 * Screen state is captured in poorly named
		 * is_suspended. */
		int valid = gesture_valid(input, gs);
		printk(KERN_DEBUG "cyttsp5 is_suspended %d tap_detected %d\n",
		       md->is_suspended, valid);
		if (md->is_suspended && valid) {
			/* Set a wakelock to prevent gesture from
			 * getting stuck in input layer during a
			 * suspend cycle. */
			wake_lock_timeout(&md->wakelock,
					  msecs_to_jiffies(WAKELOCK_TIMEOUT_IN_MS));
		}
	}

	if (!num_cur_tch)
		return;

	gs->event_count++;
	gs->min_x = min(gs->min_x, tch->abs[CY_TCH_X]);
	gs->max_x = max(gs->max_x, tch->abs[CY_TCH_X]);
	gs->min_y = min(gs->min_y, tch->abs[CY_TCH_Y]);
	gs->max_y = max(gs->max_y, tch->abs[CY_TCH_Y]);
	gs->max_touches = max(gs->max_touches, num_cur_tch);
	gs->end_time = tch->hdr[CY_TCH_TIME];
}

static void cyttsp5_get_mt_touches(struct cyttsp5_mt_data *md,
		struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	int sig;
	int i, j, t = 0;
	DECLARE_BITMAP(ids, MAX_TOUCH_NUMBER);
	int mt_sync_count = 0;

	bitmap_zero(ids, MAX_TOUCH_NUMBER);
	memset(tch->abs, 0, sizeof(tch->abs));

	for (i = 0; i < num_cur_tch; i++) {
		cyttsp5_get_touch(md, tch, si->xy_data +
			(i * si->desc.tch_record_size));

		/*  Discard proximity event */
		if (tch->abs[CY_TCH_O] == CY_OBJ_PROXIMITY) {
			dev_vdbg(dev, "%s: Discarding proximity event\n",
					__func__);
			continue;
		} else if (tch->abs[CY_TCH_O] == CY_OBJ_HOVER) {
			tch->abs[CY_TCH_P] = 0;
		}

		/*  Discard volume slider event */
		if ((md->num_prv_tch == 0) &&
		    (tch->abs[CY_TCH_Y] < md->volume_keepout)) {
			dev_vdbg(dev, "%s: Discarding volume slider event\n",
				 __func__);
			return;
		}

		if ((tch->abs[CY_TCH_T] < md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MIN_OST]) ||
			(tch->abs[CY_TCH_T] > md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MAX_OST])) {
			dev_err(dev, "%s: tch=%d -> bad trk_id=%d max_id=%d\n",
				__func__, i, tch->abs[CY_TCH_T],
				md->pdata->frmwrk->abs[(CY_ABS_ID_OST *
				CY_NUM_ABS_SET) + CY_MAX_OST]);
			if (md->mt_function.input_sync)
				md->mt_function.input_sync(md->input);
			mt_sync_count++;
			continue;
		}

		/* use 0 based track id's */
		sig = md->pdata->frmwrk->abs
			[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + 0];
		if (sig != CY_IGNORE_VALUE) {
			t = tch->abs[CY_TCH_T] - md->pdata->frmwrk->abs
				[(CY_ABS_ID_OST * CY_NUM_ABS_SET) + CY_MIN_OST];
			if (tch->abs[CY_TCH_E] == CY_EV_LIFTOFF) {
				dev_dbg(dev, "%s: t=%d e=%d lift-off\n",
					__func__, t, tch->abs[CY_TCH_E]);
				goto cyttsp5_get_mt_touches_pr_tch;
			}
			if (md->mt_function.input_report)
				md->mt_function.input_report(md->input, sig,
						t, tch->abs[CY_TCH_O]);
			__set_bit(t, ids);
		}

		/* all devices: position and pressure fields */
		for (j = 0; j <= CY_ABS_W_OST; j++) {
			if (!si->tch_abs[j].report)
				continue;
			sig = md->pdata->frmwrk->abs[((CY_ABS_X_OST + j) *
				CY_NUM_ABS_SET) + 0];
			if (sig != CY_IGNORE_VALUE) {
				int coord = tch->abs[CY_TCH_X + j];
				/* Slight tweak */
				if (si->quirks & CY_RES_Y_QUIRK &&
				    sig == CY_ABS_Y_OST)
					coord -= (CY_FW_REPORTED_RES_Y -
						  CY_CORRECTED_RES_Y);
				input_report_abs(md->input, sig, coord);
			}
		}

		/* Get the extended touch fields */
		for (j = 0; j < CY_NUM_EXT_TCH_FIELDS; j++) {
			if (!si->tch_abs[j].report)
				continue;
			sig = md->pdata->frmwrk->abs
				[((CY_ABS_MAJ_OST + j) *
				CY_NUM_ABS_SET) + 0];
			if (sig != CY_IGNORE_VALUE)
				input_report_abs(md->input, sig,
					tch->abs[CY_TCH_MAJ + j]);
		}
		if (md->mt_function.input_sync)
			md->mt_function.input_sync(md->input);
		mt_sync_count++;

cyttsp5_get_mt_touches_pr_tch:
		if (i == 0)
			gesture_accumulate(md->input, md, tch, num_cur_tch);


		dev_dbg(dev,
			"%s: t=%d x=%d y=%d z=%d M=%d m=%d o=%d e=%d obj=%d tip=%d\n",
			__func__, t,
			tch->abs[CY_TCH_X],
			tch->abs[CY_TCH_Y],
			tch->abs[CY_TCH_P],
			tch->abs[CY_TCH_MAJ],
			tch->abs[CY_TCH_MIN],
			tch->abs[CY_TCH_OR],
			tch->abs[CY_TCH_E],
			tch->abs[CY_TCH_O],
			tch->abs[CY_TCH_TIP]);
	}

	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, MAX_TOUCH_NUMBER,
				mt_sync_count, ids);

	md->num_prv_tch = num_cur_tch;

	return;
}

/* read xy_data for all current touches */
static int cyttsp5_xy_worker(struct cyttsp5_mt_data *md)
{
	struct device *dev = &md->ttsp->dev;
	struct cyttsp5_sysinfo *si = md->si;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;

	cyttsp5_get_touch_hdr(md, &tch, si->xy_mode + 3);

	num_cur_tch = tch.hdr[CY_TCH_NUM];
	if (num_cur_tch > MAX_TOUCH_NUMBER) {
		dev_err(dev, "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = MAX_TOUCH_NUMBER;
	}

	if (tch.hdr[CY_TCH_LO]) {
		dev_dbg(dev, "%s: Large area detected\n", __func__);
		if (md->pdata->flags & CY_MT_FLAG_NO_TOUCH_ON_LO)
			num_cur_tch = 0;
	}

	/* extract xy_data for all currently reported touches */
	dev_vdbg(dev, "%s: extract data num_cur_tch=%d\n", __func__,
		num_cur_tch);
	if (num_cur_tch)
		cyttsp5_get_mt_touches(md, &tch, num_cur_tch);
	else
		cyttsp5_mt_lift_all(md);

	return 0;
}

static void cyttsp5_gesture_worker(struct cyttsp5_mt_data *md)
{
	struct cyttsp5_sysinfo *si = md->si;

	if (si->gesture_id && md->volume_enabled) {
		printk(KERN_DEBUG "%s gesture_id %02x\n", __func__,
		       si->gesture_id);
		if (si->gesture_id == GESTURE_ID_VOLUME_DOWN) {
			input_report_key(md->input, KEY_VOLUMEDOWN, 1);
			input_sync(md->input);
			input_report_key(md->input, KEY_VOLUMEDOWN, 0);
			input_sync(md->input);
		}
		if (si->gesture_id == GESTURE_ID_VOLUME_UP) {
			input_report_key(md->input, KEY_VOLUMEUP, 1);
			input_sync(md->input);
			input_report_key(md->input, KEY_VOLUMEUP, 0);
			input_sync(md->input);
		}
	}
}

static void cyttsp5_mt_send_dummy_event(struct cyttsp5_mt_data *md)
{
	unsigned long ids = 0;

	/* for easy wakeup */
	if (md->mt_function.input_report)
		md->mt_function.input_report(md->input, ABS_MT_TRACKING_ID,
			0, CY_OBJ_STANDARD_FINGER);
	if (md->mt_function.input_sync)
		md->mt_function.input_sync(md->input);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 0, 1, &ids);
	if (md->mt_function.report_slot_liftoff)
		md->mt_function.report_slot_liftoff(md, 1);
	if (md->mt_function.final_sync)
		md->mt_function.final_sync(md->input, 1, 1, &ids);
}

static int cyttsp5_mt_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc;

	if (md->si->xy_mode[2] !=  md->si->desc.tch_report_id)
		return 0;

	/* core handles handshake */
	mutex_lock(&md->mt_lock);
	rc = cyttsp5_xy_worker(md);
	cyttsp5_gesture_worker(md);
	mutex_unlock(&md->mt_lock);
	if (rc < 0)
		dev_err(dev, "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static int cyttsp5_mt_wake_attention(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(&ttsp->dev);

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_send_dummy_event(md);
	mutex_unlock(&md->mt_lock);
	return 0;
}

static int cyttsp5_startup_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc = 0;

	mutex_lock(&md->mt_lock);
	cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return rc;
}

static int cyttsp5_mt_open(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);

	pm_runtime_get_sync(dev);

	dev_vdbg(dev, "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	/* set up startup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

	/* set up wakeup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_WAKE,
		cyttsp5_mt_wake_attention, 0);

	return 0;
}

static void cyttsp5_mt_close(struct input_dev *input)
{
	struct device *dev = input->dev.parent;
	struct cyttsp5_device *ttsp =
		container_of(dev, struct cyttsp5_device, dev);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_mt_attention, CY_MODE_OPERATIONAL);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

	pm_runtime_put(dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void cyttsp5_mt_early_suspend(struct early_suspend *h)
{
	struct cyttsp5_mt_data *md =
		container_of(h, struct cyttsp5_mt_data, es);

	printk(KERN_DEBUG "%s\n", __func__);
	mutex_lock(&md->mt_lock);
	md->is_suspended = true;
	mutex_unlock(&md->mt_lock);
}

static void cyttsp5_mt_late_resume(struct early_suspend *h)
{
	struct cyttsp5_mt_data *md =
		container_of(h, struct cyttsp5_mt_data, es);

	printk(KERN_DEBUG "%s\n", __func__);
	mutex_lock(&md->mt_lock);
	md->is_suspended = false;
	mutex_unlock(&md->mt_lock);
}

static void cyttsp5_setup_early_suspend(struct cyttsp5_mt_data *md)
{
	md->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	md->es.suspend = cyttsp5_mt_early_suspend;
	md->es.resume = cyttsp5_mt_late_resume;

	register_early_suspend(&md->es);
}
#endif

#if defined(CONFIG_PM_RUNTIME)
static int cyttsp5_mt_rt_suspend(struct device *dev)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	mutex_lock(&md->mt_lock);
	if (md->si)
		cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_rt_resume(struct device *dev)
{
	return 0;
}
#endif

#if defined(CONFIG_PM_SLEEP)
static int cyttsp5_mt_suspend(struct device *dev)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	mutex_lock(&md->mt_lock);
	if (md->si)
		cyttsp5_mt_lift_all(md);
	mutex_unlock(&md->mt_lock);

	return 0;
}

static int cyttsp5_mt_resume(struct device *dev)
{
	return 0;
}

struct device *cyttsp5_mt_instance;

int cyttsp5_mt_sleep_devices(void)
{
	/* TODO(ctalbott) Maybe should remember that we're supposed to
	 * be asleep, and then go to sleep when we attempt to assign
	 * to cyttsp5_mt_instance */
	if (cyttsp5_mt_instance == NULL)
		return 0;

	return cyttsp5_mt_suspend(cyttsp5_mt_instance);
}

int cyttsp5_mt_wake_devices(void)
{
	if (cyttsp5_mt_instance == NULL)
		return 0;

	return cyttsp5_mt_resume(cyttsp5_mt_instance);
}

#endif

static const struct dev_pm_ops cyttsp5_mt_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(cyttsp5_mt_suspend, cyttsp5_mt_resume)
	SET_RUNTIME_PM_OPS(cyttsp5_mt_rt_suspend, cyttsp5_mt_rt_resume, NULL)
};

static int cyttsp5_setup_input_device(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int signal = CY_IGNORE_VALUE;
	int max_x, max_y, max_p, min, max;
	int max_x_tmp, max_y_tmp;
	int i;
	int rc;

	dev_vdbg(dev, "%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, md->input->evbit);
	__set_bit(EV_REL, md->input->evbit);
	__set_bit(EV_KEY, md->input->evbit);
#ifdef INPUT_PROP_DIRECT
	__set_bit(INPUT_PROP_DIRECT, md->input->propbit);
#endif

	/* If virtualkeys enabled, don't use all screen */
	if (md->pdata->flags & CY_MT_FLAG_VKEYS) {
		max_x_tmp = md->pdata->vkeys_x;
		max_y_tmp = md->pdata->vkeys_y;
	} else {
		max_x_tmp = md->si->sensing_conf_data.res_x;
		max_y_tmp = md->si->sensing_conf_data.res_y;
	}

	/* get maximum values from the sysinfo data */
	if (md->pdata->flags & CY_MT_FLAG_FLIP) {
		max_x = max_y_tmp - 1;
		max_y = max_x_tmp - 1;
	} else {
		max_x = max_x_tmp - 1;
		max_y = max_y_tmp - 1;
	}
	max_p = md->si->sensing_conf_data.max_z;

	/* set event signal capabilities */
	for (i = 0; i < (md->pdata->frmwrk->size / CY_NUM_ABS_SET); i++) {
		signal = md->pdata->frmwrk->abs
			[(i * CY_NUM_ABS_SET) + CY_SIGNAL_OST];
		if (signal != CY_IGNORE_VALUE) {
			__set_bit(signal, md->input->absbit);
			min = md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MIN_OST];
			max = md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MAX_OST];
			if (i == CY_ABS_ID_OST) {
				/* shift track ids down to start at 0 */
				max = max - min;
				min = min - min;
			} else if (i == CY_ABS_X_OST)
				max = max_x;
			else if (i == CY_ABS_Y_OST)
				max = max_y;
			else if (i == CY_ABS_P_OST)
				max = max_p;
			input_set_abs_params(md->input, signal, min, max,
				md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FUZZ_OST],
				md->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FLAT_OST]);
			dev_dbg(dev, "%s: register signal=%02X min=%d max=%d\n",
				__func__, signal, min, max);
		}
	}

	rc = md->mt_function.input_register_device(md->input,
			md->si->tch_abs[CY_TCH_T].max);
	if (rc < 0)
		dev_err(dev, "%s: Error, failed register input device r=%d\n",
			__func__, rc);
	else
		md->input_device_registered = true;

	return rc;
}

static int cyttsp5_setup_input_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	int rc;

	md->si = cyttsp5_request_sysinfo(ttsp);
	if (!md->si)
		return -EINVAL;

	rc = cyttsp5_setup_input_device(ttsp);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_setup_input_attention, 0);

	return rc;
}

static ssize_t cyttsp5_early_suspend_status_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&md->mt_lock);
	if (md->is_suspended)
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "off\n");
	else
		ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "on\n");
	mutex_unlock(&md->mt_lock);

	return ret;
}

static ssize_t cyttsp5_enable_volume_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&md->mt_lock);
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "%d\n", md->volume_enabled);
	mutex_unlock(&md->mt_lock);

	return ret;
}

static ssize_t cyttsp5_enable_volume_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	unsigned long value;
	int rc;

	mutex_lock(&md->mt_lock);
	rc = kstrtoul(buf, 10, &value);
	if (rc < 0 || value & ~1) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		mutex_unlock(&md->mt_lock);
		return -EINVAL;
	}
	md->volume_enabled = value;
	mutex_unlock(&md->mt_lock);
	return size;
}

static ssize_t cyttsp5_volume_keepout_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	ssize_t ret;

	mutex_lock(&md->mt_lock);
	ret = snprintf(buf, CY_MAX_PRBUF_SIZE, "%d\n", md->volume_keepout);
	mutex_unlock(&md->mt_lock);

	return ret;
}

static ssize_t cyttsp5_volume_keepout_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);
	unsigned long value;
	int rc;

	mutex_lock(&md->mt_lock);
	rc = kstrtoul(buf, 10, &value);
	if (rc < 0) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		mutex_unlock(&md->mt_lock);
		return -EINVAL;
	}
	md->volume_keepout = value;
	mutex_unlock(&md->mt_lock);
	return size;
}

static struct device_attribute attributes[] = {
	__ATTR(early_suspend_status, S_IRUGO,
	       cyttsp5_early_suspend_status_show, NULL),
	__ATTR(enable_volume, S_IWUSR|S_IRUSR,
	       cyttsp5_enable_volume_show, cyttsp5_enable_volume_store),
	__ATTR(volume_keepout, S_IWUSR|S_IRUSR,
	       cyttsp5_volume_keepout_show, cyttsp5_volume_keepout_store),
};

static int add_sysfs_interfaces(struct device *dev)
{
	int i;

	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		if (device_create_file(dev, attributes + i))
			goto undo;
	return 0;
undo:
	for (i--; i >= 0; i--)
		device_remove_file(dev, attributes + i);
	dev_err(dev, "%s: failed to create sysfs interface\n", __func__);
	return -ENODEV;
}

static void remove_sysfs_interfaces(struct device *dev)
{
	u32 i;
	for (i = 0; i < ARRAY_SIZE(attributes); i++)
		device_remove_file(dev, attributes + i);
}

static int cyttsp5_mt_release(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md = dev_get_drvdata(dev);

	mutex_lock(&cyttsp5_sleep_lock);
	cyttsp5_mt_instance = NULL;
	mutex_unlock(&cyttsp5_sleep_lock);

	remove_sysfs_interfaces(dev);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&md->es);
#endif

	if (md->input_device_registered) {
		input_unregister_device(md->input);
	} else {
		input_free_device(md->input);
		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);

	dev_set_drvdata(dev, NULL);
	kfree(md);
	return 0;
}

static int cyttsp5_mt_probe(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_mt_data *md;
	struct cyttsp5_mt_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	if (pdata == NULL) {
		dev_err(dev, "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	md = kzalloc(sizeof(*md), GFP_KERNEL);
	if (md == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	cyttsp5_init_function_ptrs(md);

#ifdef CONFIG_WAKELOCK
	wake_lock_init(&md->wakelock, WAKE_LOCK_SUSPEND, "touchpad_wakelock");
#endif
	mutex_init(&md->mt_lock);
	md->ttsp = ttsp;
	md->pdata = pdata;
	dev_set_drvdata(dev, md);
	/* Create the input device and register it. */
	dev_vdbg(dev, "%s: Create the input device and register it\n",
		__func__);
	md->input = input_allocate_device();
	if (md->input == NULL) {
		dev_err(dev, "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENOSYS;
		goto error_alloc_failed;
	}

	if (pdata->inp_dev_name)
		md->input->name = pdata->inp_dev_name;
	else
		md->input->name = ttsp->name;
	scnprintf(md->phys, sizeof(md->phys)-1, "%s", dev_name(dev));
	md->input->phys = md->phys;
	md->input->dev.parent = &md->ttsp->dev;
	md->input->open = cyttsp5_mt_open;
	md->input->close = cyttsp5_mt_close;
	input_set_drvdata(md->input, md);

	pm_runtime_enable(dev);

	/* get sysinfo */
	md->si = cyttsp5_request_sysinfo(ttsp);

	if (md->si) {
		rc = cyttsp5_setup_input_device(ttsp);
		if (rc)
			goto error_init_input;
	} else {
		dev_err(dev, "%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, md->si);
		cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

#ifdef CONFIG_HAS_EARLYSUSPEND
	cyttsp5_setup_early_suspend(md);
#endif

	md->volume_enabled = 1;
	md->volume_keepout = 25;
	dev_dbg(dev, "%s: add sysfs interfaces\n", __func__);
	rc = add_sysfs_interfaces(dev);
	if (rc < 0) {
		dev_err(dev, "%s: Error, fail sysfs init\n", __func__);
		goto error_attr_create;
	}
	mutex_lock(&cyttsp5_sleep_lock);
	/* We only handle a single touchpad instance for simplicity */
	WARN_ON(cyttsp5_mt_instance != NULL);
	if (cyttsp5_mt_instance == NULL)
		cyttsp5_mt_instance = dev;
	mutex_unlock(&cyttsp5_sleep_lock);

	return 0;

error_attr_create:
	unregister_early_suspend(&md->es);
error_init_input:
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	input_free_device(md->input);
error_alloc_failed:
	dev_set_drvdata(dev, NULL);
#ifdef CONFIG_WAKELOCK
	wake_lock_destroy(&md->wakelock);
#endif
	kfree(md);
error_alloc_data_failed:
error_no_pdata:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

struct cyttsp5_driver cyttsp5_mt_driver = {
	.probe = cyttsp5_mt_probe,
	.remove = cyttsp5_mt_release,
	.driver = {
		.name = CYTTSP5_MT_NAME,
		.bus = &cyttsp5_bus_type,
		.pm = &cyttsp5_mt_pm_ops,
	},
};

