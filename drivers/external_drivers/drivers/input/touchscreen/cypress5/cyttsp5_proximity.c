/*
 * cyttsp5_proximity.c
 * Cypress TrueTouch(TM) Standard Product V5 Proximity Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2013 Cypress Semiconductor
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
#include <linux/gpio.h>
#include <linux/input.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/limits.h>
#include <linux/module.h>
#include <linux/pm_runtime.h>
#include <linux/slab.h>
#include <linux/workqueue.h>

#include <linux/cyttsp5_core.h>
#include <linux/cyttsp5_mt.h>
#include <linux/cyttsp5_proximity.h>
#include "cyttsp5_regs.h"

/* Timeout value in ms. */
#define CY_PROXIMITY_REQUEST_EXCLUSIVE_TIMEOUT		1000

#define CY_PROXIMITY_ON 0
#define CY_PROXIMITY_OFF 1

static int cyttsp5_proximity_attention(struct cyttsp5_device *ttsp);
static int cyttsp5_startup_attention(struct cyttsp5_device *ttsp);

struct cyttsp5_proximity_data {
	struct cyttsp5_device *ttsp;
	struct cyttsp5_proximity_platform_data *pdata;
	struct cyttsp5_sysinfo *si;
	struct input_dev *input;
	struct mutex report_lock;
	struct mutex sysfs_lock;
	int enable_count;
	bool is_suspended;
	bool input_device_registered;
	char phys[NAME_MAX];
	u8 pr_buf[CY_MAX_PRBUF_SIZE];
};

static void cyttsp5_report_proximity(struct cyttsp5_proximity_data *pd,
	bool on)
{
	int val = on ? CY_PROXIMITY_ON : CY_PROXIMITY_OFF;

	input_report_abs(pd->input, ABS_DISTANCE, val);
	input_sync(pd->input);
}

static void cyttsp5_get_touch_axis(struct cyttsp5_proximity_data *pd,
	int *axis, int size, int max, u8 *xy_data, int bofs)
{
	int nbyte;
	int next;

	for (nbyte = 0, *axis = 0, next = 0; nbyte < size; nbyte++) {
		dev_vdbg(&pd->ttsp->dev,
			"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
			" xy_data[%d]=%02X(%d) bofs=%d\n",
			__func__, *axis, *axis, size, max, xy_data, next,
			xy_data[next], xy_data[next], bofs);
		*axis = *axis + ((xy_data[next] >> bofs) << (nbyte * 8));
		next++;
	}

	*axis &= max - 1;

	dev_vdbg(&pd->ttsp->dev,
		"%s: *axis=%02X(%d) size=%d max=%08X xy_data=%p"
		" xy_data[%d]=%02X(%d)\n",
		__func__, *axis, *axis, size, max, xy_data, next,
		xy_data[next], xy_data[next]);
}

static void cyttsp5_get_touch_hdr(struct cyttsp5_proximity_data *pd,
	struct cyttsp5_touch *touch, u8 *xy_mode)
{
	struct device *dev = &pd->ttsp->dev;
	struct cyttsp5_sysinfo *si = pd->si;
	enum cyttsp5_tch_hdr hdr;

	for (hdr = CY_TCH_TIME; hdr < CY_TCH_NUM_HDR; hdr++) {
		if (!si->tch_hdr[hdr].report)
			continue;
		cyttsp5_get_touch_axis(pd, &touch->hdr[hdr],
			si->tch_hdr[hdr].size,
			si->tch_hdr[hdr].max,
			xy_mode + si->tch_hdr[hdr].ofs,
			si->tch_hdr[hdr].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_hdr_string[hdr],
			touch->hdr[hdr], touch->hdr[hdr]);
	}
}

static void cyttsp5_get_touch(struct cyttsp5_proximity_data *pd,
	struct cyttsp5_touch *touch, u8 *xy_data)
{
	struct device *dev = &pd->ttsp->dev;
	struct cyttsp5_sysinfo *si = pd->si;
	enum cyttsp5_tch_abs abs;

	for (abs = CY_TCH_X; abs < CY_TCH_NUM_ABS; abs++) {
		if (!si->tch_abs[abs].report)
			continue;
		cyttsp5_get_touch_axis(pd, &touch->abs[abs],
			si->tch_abs[abs].size,
			si->tch_abs[abs].max,
			xy_data + si->tch_abs[abs].ofs,
			si->tch_abs[abs].bofs);
		dev_vdbg(dev, "%s: get %s=%04X(%d)\n", __func__,
			cyttsp5_tch_abs_string[abs],
			touch->abs[abs], touch->abs[abs]);
	}

	dev_vdbg(dev, "%s: x=%04X(%d) y=%04X(%d)\n", __func__,
		touch->abs[CY_TCH_X], touch->abs[CY_TCH_X],
		touch->abs[CY_TCH_Y], touch->abs[CY_TCH_Y]);
}

static void cyttsp5_get_proximity_touch(struct cyttsp5_proximity_data *pd,
		struct cyttsp5_touch *tch, int num_cur_tch)
{
	struct cyttsp5_sysinfo *si = pd->si;
	int i;

	for (i = 0; i < num_cur_tch; i++) {
		cyttsp5_get_touch(pd, tch, si->xy_data +
			(i * si->desc.tch_record_size));

		/* Check for proximity event */
		if (tch->abs[CY_TCH_O] == CY_OBJ_PROXIMITY) {
			if (tch->abs[CY_TCH_E] == CY_EV_TOUCHDOWN)
				cyttsp5_report_proximity(pd, true);
			else if (tch->abs[CY_TCH_E] == CY_EV_LIFTOFF)
				cyttsp5_report_proximity(pd, false);
			break;
		}
	}
}

/* read xy_data for all current touches */
static int cyttsp5_xy_worker(struct cyttsp5_proximity_data *pd)
{
	struct device *dev = &pd->ttsp->dev;
	struct cyttsp5_sysinfo *si = pd->si;
	struct cyttsp5_touch tch;
	u8 num_cur_tch;

	cyttsp5_get_touch_hdr(pd, &tch, si->xy_mode + 3);

	num_cur_tch = tch.hdr[CY_TCH_NUM];
	if (num_cur_tch > MAX_TOUCH_NUMBER) {
		dev_err(dev, "%s: Num touch err detected (n=%d)\n",
			__func__, num_cur_tch);
		num_cur_tch = MAX_TOUCH_NUMBER;
	}

	if (tch.hdr[CY_TCH_LO])
		dev_dbg(dev, "%s: Large area detected\n", __func__);

	/* extract xy_data for all currently reported touches */
	dev_vdbg(dev, "%s: extract data num_cur_rec=%d\n", __func__,
		num_cur_tch);
	if (num_cur_tch)
		cyttsp5_get_proximity_touch(pd, &tch, num_cur_tch);
	else
		cyttsp5_report_proximity(pd, false);

	return 0;
}

static int _cyttsp5_set_proximity(struct cyttsp5_proximity_data *pd,
		bool enable)
{
	struct cyttsp5_device *ttsp = pd->ttsp;
	int touchmode;
	int rc;

	rc = cyttsp5_request_nonhid_get_param(ttsp, 0,
			CY_RAM_ID_TOUCHMODE_ENABLED, &touchmode);
	if (rc)
		return rc;

	if (enable)
		touchmode |= 0x80;
	else
		touchmode &= 0x7F;

	rc = cyttsp5_request_nonhid_set_param(ttsp, 0,
			CY_RAM_ID_TOUCHMODE_ENABLED, touchmode);
	return rc;
}

static int _cyttsp5_proximity_enable(struct cyttsp5_proximity_data *pd)
{
	struct cyttsp5_device *ttsp = pd->ttsp;
	struct device *dev = &ttsp->dev;
	int rc = 0;

	dev_dbg(dev, "%s\n", __func__);

	/* We use pm_runtime_get_sync to activate
	 * the core device until it is disabled back
	 */
	pm_runtime_get_sync(dev);

	rc = cyttsp5_request_exclusive(ttsp,
			CY_PROXIMITY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto exit;
	}

	rc = _cyttsp5_set_proximity(pd, true);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request enable proximity scantype r=%d\n",
				__func__, rc);
		goto exit_release;
	}

	dev_vdbg(dev, "%s: setup subscriptions\n", __func__);

	/* set up touch call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_IRQ,
		cyttsp5_proximity_attention, CY_MODE_OPERATIONAL);

	/* set up startup call back */
	cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_startup_attention, 0);

exit_release:
	cyttsp5_release_exclusive(ttsp);
exit:
	return rc;
}

static int _cyttsp5_proximity_disable(struct cyttsp5_proximity_data *pd,
		bool force)
{
	struct cyttsp5_device *ttsp = pd->ttsp;
	struct device *dev = &ttsp->dev;
	int rc = 0;

	dev_dbg(dev, "%s\n", __func__);

	rc = cyttsp5_request_exclusive(ttsp,
			CY_PROXIMITY_REQUEST_EXCLUSIVE_TIMEOUT);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request exclusive r=%d\n",
				__func__, rc);
		goto exit;
	}

	rc = _cyttsp5_set_proximity(pd, false);
	if (rc < 0) {
		dev_err(dev, "%s: Error on request disable proximity scan r=%d\n",
				__func__, rc);
		goto exit_release;
	}

exit_release:
	cyttsp5_release_exclusive(ttsp);

exit:
	if (!rc || force) {
		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_IRQ,
			cyttsp5_proximity_attention, CY_MODE_OPERATIONAL);

		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_startup_attention, 0);

		pm_runtime_put(dev);
	}

	return rc;
}

static ssize_t cyttsp5_proximity_enable_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);
	int val = 0;

	mutex_lock(&pd->sysfs_lock);
	val = pd->enable_count;
	mutex_unlock(&pd->sysfs_lock);

	return scnprintf(buf, CY_MAX_PRBUF_SIZE, "%d\n", val);
}

static ssize_t cyttsp5_proximity_enable_store(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);
	unsigned long value;
	int rc;

	rc = kstrtoul(buf, 10, &value);
	if (rc < 0 || (value != 0 && value != 1)) {
		dev_err(dev, "%s: Invalid value\n", __func__);
		return -EINVAL;
	}

	mutex_lock(&pd->sysfs_lock);
	if (value) {
		if (pd->enable_count++) {
			dev_vdbg(dev, "%s: '%s' already enabled\n", __func__,
				pd->ttsp->name);
		} else {
			rc = _cyttsp5_proximity_enable(pd);
			if (rc)
				pd->enable_count--;
		}
	} else {
		if (--pd->enable_count) {
			if (pd->enable_count < 0) {
				dev_err(dev, "%s: '%s' unbalanced disable\n",
					__func__, pd->ttsp->name);
				pd->enable_count = 0;
			}
		} else {
			rc = _cyttsp5_proximity_disable(pd, false);
			if (rc)
				pd->enable_count++;
		}
	}
	mutex_unlock(&pd->sysfs_lock);

	if (rc)
		return rc;

	return size;
}

static DEVICE_ATTR(enable, S_IRUSR | S_IWUSR,
		cyttsp5_proximity_enable_show,
		cyttsp5_proximity_enable_store);

static int cyttsp5_proximity_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);
	int rc = 0;

	if (pd->si->xy_mode[2] !=  pd->si->desc.tch_report_id)
		return 0;

	mutex_lock(&pd->report_lock);
	if (!pd->is_suspended) {
		/* core handles handshake */
		rc = cyttsp5_xy_worker(pd);
	} else {
		dev_vdbg(dev, "%s: Ignoring report while suspended\n",
			__func__);
	}
	mutex_unlock(&pd->report_lock);
	if (rc < 0)
		dev_err(dev, "%s: xy_worker error r=%d\n", __func__, rc);

	return rc;
}

static int cyttsp5_startup_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);

	mutex_lock(&pd->report_lock);
	cyttsp5_report_proximity(pd, false);
	mutex_unlock(&pd->report_lock);

	return 0;
}

#ifdef CONFIG_PM_RUNTIME
static int cyttsp5_proximity_suspend(struct device *dev)
{
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);

	mutex_lock(&pd->report_lock);
	pd->is_suspended = true;
	cyttsp5_report_proximity(pd, false);
	mutex_unlock(&pd->report_lock);

	return 0;
}

static int cyttsp5_proximity_resume(struct device *dev)
{
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);

	mutex_lock(&pd->report_lock);
	pd->is_suspended = false;
	mutex_unlock(&pd->report_lock);

	return 0;
}
#endif

static const struct dev_pm_ops cyttsp5_proximity_pm_ops = {
	SET_RUNTIME_PM_OPS(cyttsp5_proximity_suspend,
		cyttsp5_proximity_resume, NULL)
};

static int cyttsp5_setup_input_device_and_sysfs(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);
	int signal = CY_IGNORE_VALUE;
	int min, max;
	int i;
	int rc;

	rc = device_create_file(dev, &dev_attr_enable);
	if (rc) {
		dev_err(dev, "%s: Error, could not create enable\n",
				__func__);
		goto exit;
	}

	dev_vdbg(dev, "%s: Initialize event signals\n", __func__);
	__set_bit(EV_ABS, pd->input->evbit);

	/* set event signal capabilities */
	for (i = 0; i < (pd->pdata->frmwrk->size / CY_NUM_ABS_SET); i++) {
		signal = pd->pdata->frmwrk->abs
			[(i * CY_NUM_ABS_SET) + CY_SIGNAL_OST];
		if (signal != CY_IGNORE_VALUE) {
			min = pd->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MIN_OST];
			max = pd->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_MAX_OST];
			input_set_abs_params(pd->input, signal, min, max,
				pd->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FUZZ_OST],
				pd->pdata->frmwrk->abs
				[(i * CY_NUM_ABS_SET) + CY_FLAT_OST]);
		}
	}

	rc = input_register_device(pd->input);
	if (rc) {
		dev_err(dev, "%s: Error, failed register input device r=%d\n",
			__func__, rc);
		goto unregister_enable;
	}

	pd->input_device_registered = true;
	return rc;

unregister_enable:
	device_remove_file(dev, &dev_attr_enable);
exit:
	return rc;
}

static int cyttsp5_setup_input_attention(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);
	int rc;

	pd->si = cyttsp5_request_sysinfo(ttsp);
	if (!pd->si)
		return -EINVAL;

	rc = cyttsp5_setup_input_device_and_sysfs(ttsp);

	cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
		cyttsp5_setup_input_attention, 0);

	return rc;
}

static int cyttsp5_proximity_probe(struct cyttsp5_device *ttsp)
{
	struct cyttsp5_proximity_data *pd;
	struct device *dev = &ttsp->dev;
	struct cyttsp5_proximity_platform_data *pdata = dev_get_platdata(dev);
	int rc = 0;

	if (pdata == NULL) {
		dev_err(dev, "%s: Missing platform data\n", __func__);
		rc = -ENODEV;
		goto error_no_pdata;
	}

	pd = kzalloc(sizeof(*pd), GFP_KERNEL);
	if (pd == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	mutex_init(&pd->report_lock);
	mutex_init(&pd->sysfs_lock);
	pd->ttsp = ttsp;
	pd->pdata = pdata;
	dev_set_drvdata(dev, pd);
	/* Create the input device and register it. */
	dev_vdbg(dev, "%s: Create the input device and register it\n",
		__func__);
	pd->input = input_allocate_device();
	if (pd->input == NULL) {
		dev_err(dev, "%s: Error, failed to allocate input device\n",
			__func__);
		rc = -ENOSYS;
		goto error_alloc_failed;
	}

	if (pdata->inp_dev_name)
		pd->input->name = pdata->inp_dev_name;
	else
		pd->input->name = ttsp->name;
	scnprintf(pd->phys, sizeof(pd->phys)-1, "%s", dev_name(dev));
	pd->input->phys = pd->phys;
	pd->input->dev.parent = &pd->ttsp->dev;
	input_set_drvdata(pd->input, pd);

	pm_runtime_enable(dev);
	pm_runtime_get_sync(dev);

	/* get sysinfo */
	pd->si = cyttsp5_request_sysinfo(ttsp);
	pm_runtime_put(dev);

	if (pd->si) {
		rc = cyttsp5_setup_input_device_and_sysfs(ttsp);
		if (rc)
			goto error_init_input;
	} else {
		dev_err(dev, "%s: Fail get sysinfo pointer from core p=%p\n",
			__func__, pd->si);
		cyttsp5_subscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

	rc = _cyttsp5_set_proximity(pd, false);
	return 0;

error_init_input:
	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);
	input_free_device(pd->input);
error_alloc_failed:
	dev_set_drvdata(dev, NULL);
	kfree(pd);
error_alloc_data_failed:
error_no_pdata:
	dev_err(dev, "%s failed.\n", __func__);
	return rc;
}

static int cyttsp5_proximity_release(struct cyttsp5_device *ttsp)
{
	struct device *dev = &ttsp->dev;
	struct cyttsp5_proximity_data *pd = dev_get_drvdata(dev);

	dev_dbg(dev, "%s\n", __func__);

	if (pd->input_device_registered) {
		/* Disable proximity sensing */
		mutex_lock(&pd->sysfs_lock);
		if (pd->enable_count)
			_cyttsp5_proximity_disable(pd, true);
		mutex_unlock(&pd->sysfs_lock);
		device_remove_file(dev, &dev_attr_enable);
		input_unregister_device(pd->input);
	} else {
		input_free_device(pd->input);
		cyttsp5_unsubscribe_attention(ttsp, CY_ATTEN_STARTUP,
			cyttsp5_setup_input_attention, 0);
	}

	pm_runtime_suspend(dev);
	pm_runtime_disable(dev);

	dev_set_drvdata(dev, NULL);
	kfree(pd);
	return 0;
}

static struct cyttsp5_driver cyttsp5_proximity_driver = {
	.probe = cyttsp5_proximity_probe,
	.remove = cyttsp5_proximity_release,
	.driver = {
		.name = CYTTSP5_PROXIMITY_NAME,
		.bus = &cyttsp5_bus_type,
		.owner = THIS_MODULE,
		.pm = &cyttsp5_proximity_pm_ops,
	},
};

static int __init cyttsp5_proximity_init(void)
{
	int rc = 0;
	rc = cyttsp5_register_driver(&cyttsp5_proximity_driver);
	pr_info("%s: Cypress TTSP v5 Proximity Driver (Built %s), rc=%d\n",
		 __func__, CY_DRIVER_DATE, rc);
	return rc;
}
module_init(cyttsp5_proximity_init);

static void __exit cyttsp5_proximity_exit(void)
{
	cyttsp5_unregister_driver(&cyttsp5_proximity_driver);
}
module_exit(cyttsp5_proximity_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product Proximity Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
