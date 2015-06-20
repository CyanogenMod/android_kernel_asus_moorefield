/**************************************************************************
 * Copyright (c) 2012, Intel Corporation.
 * All Rights Reserved.

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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *   Vinil Cheeramvelill <vinil.cheeramvelil@intel.com>
 */

#include "displayclass_interface.h"
#include "dc_maxfifo.h"
#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "mdfld_dsi_output.h"

#include <linux/spinlock.h>
#include <linux/device.h>
#include <linux/intel_mid_pm.h>

/* When the Display controller buffer is above the high watermark
 * the display controller has enough data and does not need to
 * fetch any more data from memory till the low watermark point
 * is reached
 * Register layout
 *	Bits  9:0	- Low watermark
 *	Bits 19:10	- High watermark
 * The Watermark values represent 64 bytes of space. So a high
 * watermark value of 0x200 in bits 19:10 is
 * 0x200 X 64 = 512 X 64 = 32768 bytes
 */
#define MAXFIFO_TNG_SYSFS_GROUP_NAME	"dc_maxfifo"

#define MAXFIFO_HIGH_WATERMARK		(0x200<<10)
#define MAXFIFO_LOW_WATEMARK		(0x100<<0)

#define TNG_DSPSRCTRL_DEFAULT	(MAXFIFO_LOW_WATEMARK |\
				MAXFIFO_HIGH_WATERMARK |\
				DSPSRCTRL_MAXFIFO_MODE_ALWAYS_MAXFIFO |\
				DSPSRCTRL_MAXFIFO_ENABLE)

#define DC_MAXFIFO_REGSTOSET_DSPSRCTRL_ENABLE	0x1
#define DC_MAXFIO_REGSTOSET_DSPSSM_S0i1_DISP	0x2
#define DC_MAXFIFO_REGSTOSET_DSPSRCTRL_MAXFIFO	0x4

#define TNG_MAXFIFO_REGS_TO_SET_DEFAULT  (DC_MAXFIFO_REGSTOSET_DSPSRCTRL_ENABLE |\
		DC_MAXFIFO_REGSTOSET_DSPSRCTRL_MAXFIFO |\
		DC_MAXFIO_REGSTOSET_DSPSSM_S0i1_DISP )


static int maxfifo_entry_delay = 80;
EXPORT_SYMBOL(maxfifo_entry_delay);
module_param_named(maxfifo_delay, maxfifo_entry_delay, int, 0600);

static void maxfifo_report_repeat_frame_interrupt(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;

	schedule_work(&maxfifo_info->repeat_frame_interrupt_work);
}

static void maxfifo_timer_func(unsigned long data)
{
	struct drm_device *dev = (struct drm_device *)data;
	maxfifo_report_repeat_frame_interrupt(dev);
}

static int maxfifo_timer_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct timer_list *maxfifo_timer = &dev_priv->maxfifo_timer;

	init_timer(maxfifo_timer);

	maxfifo_timer->data = (unsigned long)dev;
	maxfifo_timer->function = maxfifo_timer_func;

	return 0;
}

void maxfifo_timer_start(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct timer_list *maxfifo_timer = &dev_priv->maxfifo_timer;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;

	if (!maxfifo_info)
		return;

	mod_timer(maxfifo_timer,
		  jiffies + msecs_to_jiffies(maxfifo_entry_delay));
}

void maxfifo_timer_stop(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;

	if (!maxfifo_info)
		return;

	del_timer(&dev_priv->maxfifo_timer);
}

static void maxfifo_send_hwc_uevent(struct drm_device * dev)
{
	char *event_string = "REPEATED_FRAME";
	char *envp[] = { event_string, NULL };

	PSB_DEBUG_MAXFIFO("maxfifo: sending uevent to HWC\n");
	kobject_uevent_env(&dev->primary->kdev.kobj, KOBJ_CHANGE, envp);
}

static void maxfifo_send_hwc_event_work(struct work_struct *work)
{
	struct dc_maxfifo *maxfifo_info = container_of(work,
				struct dc_maxfifo, repeat_frame_interrupt_work);

	/* Network streaming video playback FPS may be as low as 10fps. We do not want
	 * to trigger layers composition for every frame. So we set maxFIFO latecy
	 * longer in such scenario, but still use shorter latecy for other scenario. */
	if ((jiffies - maxfifo_info->last_jiffies) < msecs_to_jiffies(200))
		maxfifo_entry_delay = 300;
	else
		maxfifo_entry_delay = 60;

	PSB_DEBUG_MAXFIFO("last jiffies %ld, now: %ld, set timer delay: %dms\n",
		maxfifo_info->last_jiffies, jiffies, maxfifo_entry_delay);

	/* Do not adjust maxfifo latency if we just adjust it to avoid it swing between
	 * long and short */
	if (!maxfifo_info->jiffies_record) {
		maxfifo_info->last_jiffies = jiffies;
		maxfifo_info->jiffies_record = 1;
	} else
		maxfifo_info->jiffies_record = 0;

	maxfifo_send_hwc_uevent(maxfifo_info->dev_drm);
}

bool can_enter_maxfifo_s0i1_display(struct drm_device *dev, int mode)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;

	if (!maxfifo_info)
		return false;

	if (maxfifo_info->req_mode != mode) {
		/* new mode, restart entry count */
		maxfifo_info->maxfifo_try_count = 0;
		maxfifo_info->req_mode = mode;
		/* Stop idle timer for overlay only case */
		if (2 == mode)
			maxfifo_timer_stop(dev);
		return false;
	}

	/* enter into maxfifo if we get 20 consecutive same mode requests */
	if (maxfifo_info->maxfifo_try_count++ > 20)
		return true;

	return false;
}

/*
 * can be called from mid_enable_pipe_event() with dev_priv->irqmask_lock held
 * so lock acquire sequence is:
 * 	dev_priv.irqmask_lock -> maxfifo_info.lock
 * can't call dpst_irq enable/disable with maxfifo_info.lock held,
 * it will cause ABBA deadlock.
 */
bool enter_s0i1_display_mode(struct drm_device *dev, bool from_playback)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;
	unsigned long flags;
	int ret;

	if (!maxfifo_info)
		return false;

	/*
	 * enter s0i1-disp mode after vblank is disabled,
	 * need to cancel hrt timer to avoid exiting from s0i1-disp mode
	 */
	if (!from_playback)
		hrtimer_cancel(&dev_priv->vsync_timer);

	spin_lock_irqsave(&maxfifo_info->lock, flags);

	if (maxfifo_info->s0i1_disp_state != S0i1_DISP_STATE_READY) {
		ret = false;
		goto out;
	}

	ret = true;
	psb_disable_pipestat(dev_priv, 0, PIPE_VBLANK_INTERRUPT_ENABLE);
	pmu_set_s0i1_disp_vote(true);
	maxfifo_info->s0i1_disp_state = S0i1_DISP_STATE_ENTERED;
	PSB_DEBUG_MAXFIFO("maxfifo: enter s0i1-display playback:%d\n",
			  from_playback);
	if (dev_priv->psb_dpst_state)
		psb_irq_turn_off_dpst_no_lock(dev);

out:
	spin_unlock_irqrestore(&maxfifo_info->lock, flags);
	return ret;
}

static void __exit_s0i1_display_mode(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;

	if (maxfifo_info->s0i1_disp_state != S0i1_DISP_STATE_ENTERED) {
		PSB_DEBUG_MAXFIFO("maxfifo: not in s0i1 display mode\n");
		return;
	}

	pmu_set_s0i1_disp_vote(false);
	maxfifo_info->s0i1_disp_state = S0i1_DISP_STATE_READY;

	if (dev_priv->psb_dpst_state)
		psb_irq_turn_on_dpst_no_lock(dev);

	psb_enable_pipestat(dev_priv, 0, PIPE_VBLANK_INTERRUPT_ENABLE);
	PSB_DEBUG_MAXFIFO("maxfifo: exit s0i1-display\n");
}

bool exit_s0i1_display_mode(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;
	unsigned long flags;

	if (!maxfifo_info)
		return false;

	spin_lock_irqsave(&maxfifo_info->lock, flags);
	__exit_s0i1_display_mode(dev);
	spin_unlock_irqrestore(&maxfifo_info->lock, flags);

	return true;
}

bool enter_maxfifo_mode(struct drm_device *dev, int mode)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct dc_maxfifo * maxfifo_info = dev_priv->dc_maxfifo_info;
	unsigned long flags;
	u32 dspsrctrl_val = MAXFIFO_LOW_WATEMARK | MAXFIFO_HIGH_WATERMARK;
	u32 regs_to_set;

	if (!maxfifo_info)
		return false;

	maxfifo_timer_stop(dev);

	if (power_island_get(OSPM_DISPLAY_A)) {
		spin_lock_irqsave(&maxfifo_info->lock, flags);

		regs_to_set = maxfifo_info->regs_to_set;
		if (regs_to_set & DC_MAXFIFO_REGSTOSET_DSPSRCTRL_ENABLE) {
			dspsrctrl_val |= DSPSRCTRL_MAXFIFO_ENABLE;
			dspsrctrl_val |= mode << 24;
			if (regs_to_set & DC_MAXFIFO_REGSTOSET_DSPSRCTRL_MAXFIFO)
				dspsrctrl_val |= DSPSRCTRL_MAXFIFO_MODE_ALWAYS_MAXFIFO;
			maxfifo_info->ddl1 = PSB_RVDC32(DDL1);
			maxfifo_info->ddl2 = PSB_RVDC32(DDL2);
			maxfifo_info->ddl3 = PSB_RVDC32(DDL3);
			maxfifo_info->ddl4 = PSB_RVDC32(DDL4);
			PSB_WVDC32(dspsrctrl_val, DSPSRCTRL_REG);
			/* As SV suggestion, we need to set DDL as 0 in maxfifo mode */
			PSB_WVDC32(0, DDL1);
			PSB_WVDC32(0, DDL2);
			PSB_WVDC32(0, DDL3);
			PSB_WVDC32(0, DDL4);
			maxfifo_info->maxfifo_current_state = mode;
		}

		if (regs_to_set & DC_MAXFIO_REGSTOSET_DSPSSM_S0i1_DISP)
			maxfifo_info->s0i1_disp_state = S0i1_DISP_STATE_READY;

		PSB_DEBUG_MAXFIFO("maxfifo: enter into maxfifo mode %d, "
				  "DSPSRCTRL = %08x, DSP_SS_PM = %08x\n",
				  mode, PSB_RVDC32(DSPSRCTRL_REG),
				intel_mid_msgbus_read32(PUNIT_PORT, DSP_SS_PM));

		spin_unlock_irqrestore(&maxfifo_info->lock, flags);
		power_island_put(OSPM_DISPLAY_A);
	}

	return true;
}

bool exit_maxfifo_mode(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;
	u32 dspsrctrl_val = MAXFIFO_LOW_WATEMARK | MAXFIFO_HIGH_WATERMARK;
	unsigned long flags;

	if (!maxfifo_info)
		return false;

	if (power_island_get(OSPM_DISPLAY_A)) {
		spin_lock_irqsave(&maxfifo_info->lock, flags);

		__exit_s0i1_display_mode(dev);

		PSB_DEBUG_MAXFIFO("maxfifo: exit maxfifo mode %d, "
				  "DSPSRCTRL = %08x, DSP_SS_PM = %08x\n",
				  maxfifo_info->maxfifo_current_state,
				  PSB_RVDC32(DSPSRCTRL_REG),
				intel_mid_msgbus_read32(PUNIT_PORT, DSP_SS_PM));

		/* Set DDL back to original values when leaving maxfifo */
		PSB_WVDC32(maxfifo_info->ddl1, DDL1);
		PSB_WVDC32(maxfifo_info->ddl2, DDL2);
		PSB_WVDC32(maxfifo_info->ddl3, DDL3);
		PSB_WVDC32(maxfifo_info->ddl4, DDL4);
		PSB_WVDC32(dspsrctrl_val, DSPSRCTRL_REG);
		maxfifo_info->maxfifo_current_state = -1;
		maxfifo_info->req_mode = -1;
		maxfifo_info->maxfifo_try_count = 0;
		maxfifo_info->s0i1_disp_state = S0i1_DISP_STATE_NOT_READY;

		spin_unlock_irqrestore(&maxfifo_info->lock, flags);
		power_island_put(OSPM_DISPLAY_A);
	}

	return true;
}

/*
 * Sysfs Entries for Maxfifo mode
 */
static ssize_t _show_sysfs_enable (struct device *kdev,
				   struct device_attribute *attr, char *buf)
{
	int enabled = 0;

	struct drm_minor *minor = container_of(kdev, struct drm_minor, kdev);
	struct drm_device *dev = minor->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct dc_maxfifo *maxfifo_info = dev_priv->dc_maxfifo_info;

	if (maxfifo_info)
		enabled = maxfifo_info->regs_to_set;

	return sprintf(buf, "%d\n", enabled);

}

static ssize_t _store_sysfs_enable(struct device *kdev,
				   struct device_attribute *attr,
				   const char *buf, size_t count)
{
	int enable;

	struct drm_minor *minor = container_of(kdev, struct drm_minor, kdev);
	struct drm_device *dev = minor->dev;
	struct drm_psb_private * dev_priv = dev->dev_private;
	struct dc_maxfifo * maxfifo_info = dev_priv->dc_maxfifo_info;

	if (maxfifo_info) {
		sscanf(buf, "%d", &enable);

		spin_lock_irq(&maxfifo_info->lock);
		maxfifo_info->regs_to_set = enable;
		spin_unlock_irq(&maxfifo_info->lock);

		if (enable & 0x8)
			enter_maxfifo_mode(dev, 0);
		if (enable & 0x10)
			exit_maxfifo_mode(dev);
		if (enable & 0x20)
			maxfifo_report_repeat_frame_interrupt(dev);
		if (enable & 0x80)
			maxfifo_send_hwc_uevent(dev);
	}

	return count;
}

static ssize_t _show_sysfs_state (struct device *kdev,
				struct device_attribute *attr, char *buf)
{
	int ret = 0;

	struct drm_minor *minor = container_of(kdev, struct drm_minor, kdev);
	struct drm_device *dev = minor->dev;
	struct drm_psb_private * dev_priv = dev->dev_private;
	struct dc_maxfifo * maxfifo_info = dev_priv->dc_maxfifo_info;

	if (maxfifo_info) {
		spin_lock_irq(&maxfifo_info->lock);
		ret = sprintf(buf,
			      "S0i1-Display-Status: Reg DSPSRCTRL = %08x, "
			      "DSP_SS_PM = %08x\n",
			      PSB_RVDC32(DSPSRCTRL_REG),
			      intel_mid_msgbus_read32(PUNIT_PORT, DSP_SS_PM));
		spin_unlock_irq(&maxfifo_info->lock);
	}
	return ret;
}

static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR,
		   _show_sysfs_enable, _store_sysfs_enable);
static DEVICE_ATTR(state, S_IRUGO, _show_sysfs_state, NULL);

static struct attribute *tng_maxfifo_attrs[] = {
	&dev_attr_enable.attr,
	&dev_attr_state.attr,
	NULL
};

static struct attribute_group tng_maxfifo_attr_group = {
	.name = MAXFIFO_TNG_SYSFS_GROUP_NAME,
	.attrs = tng_maxfifo_attrs
};

static bool maxfifo_create_sysfs_entries(struct drm_device * dev)
{
	int ret;

	ret = sysfs_create_group(&dev->primary->kdev.kobj,
				&tng_maxfifo_attr_group);
	if (ret)
		DRM_ERROR("Maxfifo sysfs setup failed\n");
	return ret;
}

int dc_maxfifo_init(struct drm_device *dev)
{
	struct dc_maxfifo *maxfifo_info;
	struct drm_psb_private *dev_priv = dev->dev_private;

	/* don't support s0i1-disp with command mode panel */
	if (is_panel_vid_or_cmd(dev) == MDFLD_DSI_ENCODER_DBI)
		return 0;

	dev_priv->dc_maxfifo_info =
		kzalloc(sizeof(struct dc_maxfifo), GFP_KERNEL);

	if (!dev_priv->dc_maxfifo_info) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}
	maxfifo_info = dev_priv->dc_maxfifo_info;

	spin_lock_init(&maxfifo_info->lock);

	maxfifo_info->maxfifo_current_state = -1;
	maxfifo_info->req_mode = -1;
	maxfifo_info->dev_drm = dev;
	maxfifo_info->regs_to_set = TNG_MAXFIFO_REGS_TO_SET_DEFAULT;
	maxfifo_info->s0i1_disp_state = S0i1_DISP_STATE_NOT_READY;
	maxfifo_info->jiffies_record = 0;

	INIT_WORK(&maxfifo_info->repeat_frame_interrupt_work,
			maxfifo_send_hwc_event_work);

	maxfifo_info->repeat_frame_interrupt_on = false;
	maxfifo_timer_init(dev);

	/*Initialize the sysfs entries*/
	maxfifo_create_sysfs_entries(dev);

	return 0;
}

int dc_maxfifo_uninit(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;

	if (!dev_priv->dc_maxfifo_info)
		return true;

	kfree(dev_priv->dc_maxfifo_info);
	return 0;
}
