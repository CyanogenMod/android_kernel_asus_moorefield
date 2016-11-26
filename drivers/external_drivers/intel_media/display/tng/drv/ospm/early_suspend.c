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
 *    Hitesh K. Patel <hitesh.k.patel@intel.com>
 */

#ifdef CONFIG_HAS_EARLYSUSPEND

#include <linux/earlysuspend.h>
#include <linux/mutex.h>
#include <linux/notifier.h>
#include <linux/reboot.h>
#include <asm/intel_scu_pmic.h>
#include <linux/lnw_gpio.h>
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>
#include "psb_drv.h"
#include "early_suspend.h"
#include "android_hdmi.h"
#include "gfx_rtpm.h"
#include "dc_maxfifo.h"

static struct drm_device *g_dev;

#define PULL_DOWN_EN			BIT(9)
#define PULL_UP_EN			BIT(8)
#define BL_EN_REG 0xFF0C2530

static void gfx_early_suspend(struct early_suspend *h)
{
	struct drm_psb_private *dev_priv = g_dev->dev_private;
	struct drm_device *dev = dev_priv->dev;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *enc_funcs;

	PSB_DEBUG_PM("%s\n", __func__);

	flush_workqueue(dev_priv->power_wq);

	/*
	 * exit s0i1-disp mode to avoid keeing system in s0i1-disp mode,
	 * otherwise, we may block system entering into other s0i1 mode
	 * after screen off
	 */
	maxfifo_timer_stop(dev);
	exit_maxfifo_mode(dev);

	if (dev_priv->psb_dpst_state)
		psb_irq_disable_dpst(dev);
	/* protect early_suspend with dpms and mode config */
	mutex_lock(&dev->mode_config.mutex);

	list_for_each_entry(encoder,
			&dev->mode_config.encoder_list,
			head) {
		enc_funcs = encoder->helper_private;
		if (!drm_helper_encoder_in_use(encoder))
			continue;
		if (enc_funcs && enc_funcs->save)
			enc_funcs->save(encoder);

		if (encoder->encoder_type == DRM_MODE_ENCODER_TMDS) {
			DCLockMutex();
			drm_handle_vblank(dev, 1);

			/* Turn off vsync interrupt. */
			drm_vblank_off(dev, 1);

			/* Make the pending flip request as completed. */
			DCUnAttachPipe(1);
			DC_MRFLD_onPowerOff(1);
			DCUnLockMutex();
		}
	}

	/* Suspend hdmi
	 * Note: hotplug detection is disabled if audio is not playing
	 */
	android_hdmi_suspend_display(dev);

	ospm_power_suspend();
	dev_priv->early_suspended = true;

	mutex_unlock(&dev->mode_config.mutex);
	psb_dpst_notify_change_um(DPST_EVENT_HIST_INTERRUPT,
						  dev_priv->psb_dpst_state);
}

static void gfx_late_resume(struct early_suspend *h)
{
	struct drm_psb_private *dev_priv = g_dev->dev_private;
	struct drm_device *dev = dev_priv->dev;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *enc_funcs;

	PSB_DEBUG_PM("%s\n", __func__);

	/* protect early_suspend with dpms and mode config */
	mutex_lock(&dev->mode_config.mutex);

	dev_priv->early_suspended = false;
	ospm_power_resume();

	list_for_each_entry(encoder,
			&dev->mode_config.encoder_list,
			head) {
		enc_funcs = encoder->helper_private;
		if (!drm_helper_encoder_in_use(encoder))
			continue;
		if (enc_funcs && enc_funcs->save)
			enc_funcs->restore(encoder);
	}

	/* Resume HDMI */
	android_hdmi_resume_display(dev);

	/*
	 * Devices connect status will be changed
	 * when system suspend,re-detect once here.
	 */
	if (android_hdmi_is_connected(dev)) {
		DCLockMutex();
		DCAttachPipe(1);
		DC_MRFLD_onPowerOn(1);
		mid_hdmi_audio_resume(dev);
		DCUnLockMutex();
	}

	mutex_unlock(&dev->mode_config.mutex);
}

static struct early_suspend intel_media_early_suspend = {
	.level = EARLY_SUSPEND_LEVEL_DISABLE_FB,
	.suspend = gfx_early_suspend,
	.resume = gfx_late_resume,
};

static int display_reboot_notifier_call(struct notifier_block *this, unsigned long event, void *ptr)
{
	struct drm_psb_private *dev_priv = g_dev->dev_private;
	struct drm_device *dev = dev_priv->dev;
	struct drm_encoder *encoder;
	struct drm_encoder_helper_funcs *enc_funcs;
	static void __iomem *bl_en_mmio;
	u8 addr, value;
	addr = 0xae;

	if (!bl_en_mmio)
		bl_en_mmio = ioremap_nocache(BL_EN_REG, 4);

	switch (event) {
	case SYS_RESTART:
	case SYS_HALT:
	case SYS_POWER_OFF:
		pr_info("%s+\n", __func__);
		//gfx_early_suspend(NULL);
		/* protect early_suspend with dpms and mode config */
		if (dev_priv->early_suspended)
			return;

		mutex_lock(&dev->mode_config.mutex);
		/*
		* We borrow the early_suspended to avoid entering flip path after
		* shutdown is called
		*/
		dev_priv->early_suspended = true;

		/* wait for the previous flip to be finished */
		list_for_each_entry(encoder, &dev->mode_config.encoder_list, head) {
			enc_funcs = encoder->helper_private;
			if (!drm_helper_encoder_in_use(encoder))
				continue;
			if (enc_funcs && enc_funcs->save)
				enc_funcs->save(encoder);
		}
		mutex_unlock(&dev->mode_config.mutex);

		usleep_range(60000, 60600);

		if (Read_PROJ_ID() != PROJ_ID_ZS550ML) {
			printk("[DISP] turn off the HW reset!\n");
			gpio_set_value_cansleep(190, 0);
			usleep_range(5000, 5100);
		}

		printk("[DISP] turn off the power 2v8!\n");
		intel_scu_ipc_ioread8(addr, &value);
		value &= ~0x1;
		intel_scu_ipc_iowrite8(addr, value);

		writel((readl(bl_en_mmio) | PULL_DOWN_EN) & (~PULL_UP_EN), bl_en_mmio);
		printk("[DISP] PULL DOWN the BL_EN gpio pin! %x\n", readl(bl_en_mmio));

		pr_info("%s-\n", __func__);
		break;
	}
}

static struct notifier_block display_reboot_notifier = {
	.notifier_call = display_reboot_notifier_call,
	.priority = 2,
};

void intel_media_early_suspend_init(struct drm_device *dev)
{
	g_dev = dev;
	register_early_suspend(&intel_media_early_suspend);

	if (register_reboot_notifier(&display_reboot_notifier))
		DRM_ERROR("%s: unable to register display reboot notifier\n", __func__);
}

void intel_media_early_suspend_uninit(void)
{
	unregister_early_suspend(&intel_media_early_suspend);

	if (unregister_reboot_notifier(&display_reboot_notifier))
		DRM_ERROR("%s: unable to unregister display reboot notifier\n", __func__);
}

#endif
