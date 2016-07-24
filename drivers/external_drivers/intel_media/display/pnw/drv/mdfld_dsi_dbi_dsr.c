/*
 * Copyright © 2012 Intel Corporation
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
 * Authors:
 * Jackie Li<yaodong.li@intel.com>
 */
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dsr.h"
#include "mdfld_dsi_pkg_sender.h"

#define DSR_COUNT 15

static int exit_dsr_locked(struct mdfld_dsi_config *dsi_config)
{
	int err = 0;
	struct drm_device *dev;
	struct mdfld_dsi_pkg_sender *sender;
	if (!dsi_config)
		return -EINVAL;

	sender = mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("Failed to get dsi sender\n");
		return -EINVAL;
	}

	dev = dsi_config->dev;
	err =  __dbi_power_on(dsi_config);
	if (!err)
		/*enable TE, will need it in panel power on*/
		mdfld_enable_te(dev, dsi_config->pipe);

	mdfld_dsi_status_check(sender);
	return err;
}

static int enter_dsr_locked(struct mdfld_dsi_config *dsi_config, int level)
{
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	struct mdfld_dsi_pkg_sender *sender;
	int err;
	pm_message_t state;

	PSB_DEBUG_ENTRY("mdfld_dsi_dsr: enter dsr\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	sender = mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("Failed to get dsi sender\n");
		return -EINVAL;
	}

	if (level < DSR_EXITED) {
		DRM_ERROR("Why to do this?");
		return -EINVAL;
	}

	if (level > DSR_ENTERED_LEVEL0) {
		/**
		 * TODO: require OSPM interfaces to tell OSPM module that
		 * display controller is ready to be power gated.
		 * OSPM module needs to response this request ASAP.
		 * NOTE: it makes no sense to have display controller islands
		 * & pci power gated here directly. OSPM module is the only one
		 * who can power gate/ungate power islands.
		 * FIXME: since there's no ospm interfaces for acquiring
		 * suspending DSI related power islands, we have to call OSPM
		 * interfaces to power gate display islands and pci right now,
		 * which should NOT happen in this way!!!
		 */
		if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
			OSPM_UHB_FORCE_POWER_ON)) {
			DRM_ERROR("Failed power on display island\n");
			return -EINVAL;
		}

		PSB_DEBUG_ENTRY("mdfld_dsi_dsr: entering DSR level 1\n");

		/*Disable TE, don't need it anymore*/
		mdfld_disable_te(dev, dsi_config->pipe);
		err = mdfld_dsi_wait_for_fifos_empty(sender);
		if (err) {
			DRM_ERROR("mdfld_dsi_dsr: FIFO not empty\n");
			mdfld_enable_te(dev, dsi_config->pipe);
			ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
			return err;
		}
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

		/*suspend whole PCI host and related islands
		** if failed at this try, revive te for another chance
		*/
		state.event = 0;
		if (ospm_power_suspend(gpDrmDevice->pdev, state)) {
			/* Only display island is powered off then
			 ** need revive the whole TE
			 */
			if (!ospm_power_is_hw_on(OSPM_DISPLAY_ISLAND))
				exit_dsr_locked(dsi_config);
			else
				mdfld_enable_te(dev, dsi_config->pipe);
			return -EINVAL;
		}
		/*
		 *suspend pci
		 *FIXME: should I do it here?
		 *how about decoder/encoder is working??
		 *OSPM should check the refcout of each islands before
		 *actually power off PCI!!!
		 *need invoke this in the same context, we need deal with
		 *DSR lock later for suspend PCI may go to sleep!!!
		 */
		/*ospm_suspend_pci(dev->pdev);*/

		PSB_DEBUG_ENTRY("mdfld_dsi_dsr: entered\n");
		return 0;
	}

	/*
	 * if DSR_EXITED < level < DSR_ENTERED_LEVEL1, we only have the display
	 * controller components turned off instead of power gate them.
	 * this is useful for HDMI & WIDI.
	 */
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
		OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("Failed power on display island\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("mdfld_dsi_dsr: entering DSR level 0\n");

	/*Disable TE, don't need it anymore*/
	mdfld_disable_te(dev, dsi_config->pipe);
	err = mdfld_dsi_wait_for_fifos_empty(sender);
	if (err) {
		DRM_ERROR("mdfld_dsi_dsr: FIFO not empty\n");
		mdfld_enable_te(dev, dsi_config->pipe);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		return err;
	}

	/*turn off dbi interface put in ulps*/
	__dbi_power_off(dsi_config);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	PSB_DEBUG_ENTRY("entered\n");
	return 0;
}

int mdfld_dsi_dsr_update_panel_fb(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	struct mdfld_dsi_pkg_sender *sender;
	int err = 0;

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config->dsi_hw_context.panel_on) {
		PSB_DEBUG_ENTRY(
		"if screen off, update fb is not allowed\n");
		goto update_fb_out;
	}

	/*no pending fb updates, go ahead to send out write_mem_start*/
	PSB_DEBUG_ENTRY("send out write_mem_start\n");
	sender = mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("No sender\n");
		err = -EINVAL;
		goto update_fb_out;
	}
	/*some time the panel will randomly in
	*idle/invert mode. so here, make sure
	*exit idle/invert mode for each frame
	*/
	err = mdfld_dsi_send_dcs(sender,
		 exit_idle_mode,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("faild to exit idle mode\n");
		err = -EINVAL;
		goto update_fb_out;
	}

	err = mdfld_dsi_send_dcs(sender,
		 exit_invert_mode,
		 NULL,
		 0,
		 CMD_DATA_SRC_SYSTEM_MEM,
		 MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_ERROR("faild to exit invert mode\n");
		err = -EINVAL;
		goto update_fb_out;
	}

	err = mdfld_dsi_send_dcs(sender, write_mem_start,
				NULL, 0, CMD_DATA_SRC_PIPE,
				MDFLD_DSI_SEND_PACKAGE);
	if (err) {
		DRM_DEBUG("Failed to send write_mem_start");
		err = -EINVAL;
		goto update_fb_out;
	}

	/*clear free count*/
	dsr->free_count = 0;
update_fb_out:
	return err;
}

int mdfld_dsi_dsr_report_te(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int err = 0;
	int dsr_level;

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	/*
	 * TODO: check HDMI & WIDI connection state here, then setup
	 * dsr_level accordingly.
	 */
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	/*
	 * FIXME: when hdmi connected with no audio output, we still can
	 * power gate DSI related islands, how to check whether HDMI audio
	 * is active or not.
	 * Currently, we simply enter DSR LEVEL0 when HDMI is connected
	 */
	if (dev_priv->bhdmiconnected)
		dsr_level = DSR_ENTERED_LEVEL0;
	else
		dsr_level = DSR_ENTERED_LEVEL1;

	mutex_lock(&dsi_config->context_lock);

	if (!dsr->dsr_enabled)
		goto report_te_out;

	/*if panel is off, then forget it*/
	if (!dsi_config->dsi_hw_context.panel_on)
		goto report_te_out;

	if (dsr_level <= dsr->dsr_state)
		goto report_te_out;
	else if (++dsr->free_count > DSR_COUNT && !dsr->ref_count) {
		/*reset free count*/
		dsr->free_count = 0;
		if (drm_psb_use_cases_control & PSB_DSR_ENABLE) {
			/*enter dsr*/
			err = enter_dsr_locked(dsi_config, dsr_level);
			if (err) {
				PSB_DEBUG_ENTRY("Failed to enter DSR\n");
				goto report_te_out;
			}
			dsr->dsr_state = dsr_level;
		}
	}
report_te_out:
	mutex_unlock(&dsi_config->context_lock);
	return err;
}

int mdfld_dsi_dsr_forbid_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	int err = 0;

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	/*exit dsr if necessary*/
	if (!dsr->dsr_enabled)
		goto forbid_out;

	PSB_DEBUG_ENTRY("\n");

	/*if reference count is not 0, it means dsr was forbidden*/
	if (dsr->ref_count) {
		dsr->ref_count++;
		goto forbid_out;
	}

	/*exited dsr if current dsr state is DSR_ENTERED*/
	if (dsr->dsr_state > DSR_EXITED) {
		err = exit_dsr_locked(dsi_config);
		if (err) {
			DRM_ERROR("Failed to exit DSR\n");
			goto forbid_out;
		}
		dsr->dsr_state = DSR_EXITED;
	}
	dsr->ref_count++;
forbid_out:
	return err;
}

int mdfld_dsi_dsr_forbid(struct mdfld_dsi_config *dsi_config)
{
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	mutex_lock(&dsi_config->context_lock);

	err = mdfld_dsi_dsr_forbid_locked(dsi_config);

	mutex_unlock(&dsi_config->context_lock);

	return err;
}

int mdfld_dsi_dsr_allow_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return 0;

	if (!dsr->dsr_enabled)
		goto allow_out;


	if (!dsr->ref_count)
		goto allow_out;

	PSB_DEBUG_ENTRY("\n");

	dsr->ref_count--;
allow_out:
	return 0;
}

int mdfld_dsi_dsr_allow(struct mdfld_dsi_config *dsi_config)
{
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	mutex_lock(&dsi_config->context_lock);

	err = mdfld_dsi_dsr_allow_locked(dsi_config);

	mutex_unlock(&dsi_config->context_lock);

	return err;
}

void mdfld_dsi_dsr_enable(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		return;

	/*lock dsr*/
	mutex_lock(&dsi_config->context_lock);

	dsr->dsr_enabled = 1;
	dsr->dsr_state = DSR_EXITED;

	mutex_unlock(&dsi_config->context_lock);
}

int mdfld_dsi_dsr_in_dsr_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;
	int in_dsr = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		goto get_state_out;
	}

	dsr = dsi_config->dsr;

	/*if no dsr attached, return 0*/
	if (!dsr)
		goto get_state_out;

	if (dsr->dsr_state > DSR_EXITED)
		in_dsr = 1;

get_state_out:
	return in_dsr;
}

/**
 * init dsr structure
 */
int mdfld_dsi_dsr_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}

	/*check panel type*/
	if (dsi_config->type == MDFLD_DSI_ENCODER_DPI) {
		DRM_INFO("%s: Video mode panel, disabling DSR\n", __func__);
		return 0;
	}

	dsr = kzalloc(sizeof(struct mdfld_dsi_dsr), GFP_KERNEL);
	if (!dsr) {
		DRM_ERROR("No memory\n");
		return -ENOMEM;
	}

	/*init reference count*/
	dsr->ref_count = 0;

	/*init free count*/
	dsr->free_count = 0;

	/*init dsr enabled*/
	dsr->dsr_enabled = 0;

	/*set dsr state*/
	dsr->dsr_state = DSR_INIT;

	/*init dsi config*/
	dsr->dsi_config = dsi_config;

	dsi_config->dsr = dsr;

	PSB_DEBUG_ENTRY("successfully\n");

	return 0;
}

/**
 * destroy dsr structure
 */
void mdfld_dsi_dsr_destroy(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_dsr *dsr;

	PSB_DEBUG_ENTRY("\n");

	dsr = dsi_config->dsr;

	if (!dsr)
		kfree(dsr);

	dsi_config->dsr = 0;
}
