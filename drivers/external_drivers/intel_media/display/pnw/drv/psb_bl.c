/*
 *  psb backlight using HAL
 *
 * Copyright (c) 2009, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc., 
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * Authors: Eric Knopp
 *
 */

#include <linux/backlight.h>
#include <linux/version.h>
#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_powermgmt.h"
#include "mdfld_dsi_dbi.h"

#define MRST_BLC_MAX_PWM_REG_FREQ	    0xFFFF
#define BLC_PWM_PRECISION_FACTOR 100	/* 10000000 */
#define BLC_PWM_FREQ_CALC_CONSTANT 32
#define MHz 1000000
#define BRIGHTNESS_MIN_LEVEL 1
#define BRIGHTNESS_INIT_LEVEL	50
#define BRIGHTNESS_MAX_LEVEL 100
#define BRIGHTNESS_MASK	0xFF
#define BLC_POLARITY_NORMAL 0
#define BLC_POLARITY_INVERSE 1
#define BLC_ADJUSTMENT_MAX 100

#define PSB_BLC_PWM_PRECISION_FACTOR    10
#define PSB_BLC_MAX_PWM_REG_FREQ        0xFFFE
#define PSB_BLC_MIN_PWM_REG_FREQ        0x2

#define PSB_BACKLIGHT_PWM_POLARITY_BIT_CLEAR (0xFFFE)
#define PSB_BACKLIGHT_PWM_CTL_SHIFT	(16)

int psb_brightness;
static struct backlight_device *psb_backlight_device;
u8 blc_pol;
u8 blc_type;

int lastFailedBrightness = -1;

int psb_set_brightness(struct backlight_device *bd)
{
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_config **dsi_configs;
	int level;
	u32 blc_pwm_ctl;
	u32 max_pwm_blc;
	bool panel_on1 = false, panel_on2 = false;

	if (!(drm_psb_use_cases_control & PSB_BRIGHTNESS_ENABLE))
		return 0;


	dev = (struct drm_device *)bl_get_data(psb_backlight_device);
	dev_priv = (struct drm_psb_private *)dev->dev_private;
	dsi_configs = dev_priv->dsi_configs;

	if (dsi_configs[0])
		panel_on1 = dsi_configs[0]->dsi_hw_context.panel_on;
	if (dsi_configs[1])
		panel_on2 = dsi_configs[1]->dsi_hw_context.panel_on;

	if(bd != NULL)
		level = bd->props.brightness;
	else
		level = lastFailedBrightness;

	/* Perform value bounds checking */
	if (level < BRIGHTNESS_MIN_LEVEL)
		level = BRIGHTNESS_MIN_LEVEL;

	if(!gbdispstatus){
		PSB_DEBUG_ENTRY("[DISPLAY]: already OFF ignoring brighness " \
				"request\n");
		/* there may exist concurrent racing, the gbdispstatus may
		 * haven't been set in gfx_late_resume yet. record here, and
		 * we may call brightness setting at the end of gfx_late_resume
		 */
		lastFailedBrightness = level;
		return 0;
	}

	lastFailedBrightness = -1;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON)) {
		if (IS_MRST(dev)) {
			/* Calculate and set the brightness value */
			max_pwm_blc = REG_READ(BLC_PWM_CTL) >> MRST_BACKLIGHT_MODULATION_FREQ_SHIFT;
			blc_pwm_ctl = level * max_pwm_blc / BRIGHTNESS_MAX_LEVEL;

			/* Adjust the backlight level with the percent in
			 * dev_priv->blc_adj1;
			 */
			blc_pwm_ctl = blc_pwm_ctl * dev_priv->blc_adj1;
			blc_pwm_ctl = blc_pwm_ctl / BLC_ADJUSTMENT_MAX;

			/* Adjust the backlight level with the percent in
			 * dev_priv->blc_adj2;
			 */
			blc_pwm_ctl = blc_pwm_ctl * dev_priv->blc_adj2;
			blc_pwm_ctl = blc_pwm_ctl / BLC_ADJUSTMENT_MAX;


			if (blc_pol == BLC_POLARITY_INVERSE)
				blc_pwm_ctl = max_pwm_blc - blc_pwm_ctl;

			/* force PWM bit on */
			REG_WRITE(BLC_PWM_CTL2, (0x80000000 | REG_READ(BLC_PWM_CTL2)));
			REG_WRITE(BLC_PWM_CTL, (max_pwm_blc << MRST_BACKLIGHT_MODULATION_FREQ_SHIFT) |
				  blc_pwm_ctl);
		} else if (IS_MDFLD(dev)) {
			u32 adjusted_level = 0;

			/* Adjust the backlight level with the percent in
			 * dev_priv->blc_adj2;
			 */
			adjusted_level = level * dev_priv->blc_adj2;
			adjusted_level =
				adjusted_level / BLC_ADJUSTMENT_MAX / 100;
			dev_priv->brightness_adjusted = adjusted_level;

#ifndef CONFIG_MDFLD_DSI_DPU
			if (!(dev_priv->dsr_fb_update & MDFLD_DSR_MIPI_CONTROL)
			    && dev_priv->b_dsr_enable
			    && is_cmd_mode_panel(dev)
			    && (panel_on1 || panel_on2)) {
				dev_priv->exit_idle(dev,
						MDFLD_DSR_MIPI_CONTROL,
						0, 0);
				PSB_DEBUG_ENTRY("Out of DSR before set " \
						"brightness to %d.\n",
						adjusted_level);
			}
#endif
			/* support the main panel brightness control */
			mdfld_dsi_brightness_control(dev, 0, adjusted_level);
			/* support the second panel brightness control */
			if (dev_priv->panel_desc == DISPLAY_C)
				mdfld_dsi_brightness_control(dev, 2, adjusted_level);
		}
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}

	/* cache the brightness for later use */
	psb_brightness = level;
	return 0;
}

int psb_get_brightness(struct backlight_device *bd)
{
	DRM_DEBUG_DRIVER("brightness = 0x%x \n", psb_brightness);

	/* return locally cached var instead of HW read (due to DPST etc.) */
	return psb_brightness;
}

const struct backlight_ops psb_ops = {
	.get_brightness = psb_get_brightness,
	.update_status  = psb_set_brightness,
};

static int device_backlight_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = (struct drm_psb_private *) dev->dev_private;

	dev_priv->blc_adj1 = BLC_ADJUSTMENT_MAX;
	dev_priv->blc_adj2 = BLC_ADJUSTMENT_MAX * 100;
	return 0;
}

int psb_backlight_init(struct drm_device *dev)
{
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	int ret = 0;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34))
	psb_backlight_device = backlight_device_register("psb-bl", NULL, (void *)dev, &psb_ops);
#else
	struct backlight_properties props;
	memset(&props, 0, sizeof(struct backlight_properties));
	props.type = BACKLIGHT_PLATFORM;
	props.max_brightness = BRIGHTNESS_MAX_LEVEL;

	psb_backlight_device = backlight_device_register("psb-bl", NULL, (void *)dev, &psb_ops, &props);
#endif
	if (IS_ERR(psb_backlight_device))
		return PTR_ERR(psb_backlight_device);

	if ((ret = device_backlight_init(dev)) != 0)
		return ret;

	psb_backlight_device->props.brightness = BRIGHTNESS_INIT_LEVEL;
	psb_backlight_device->props.max_brightness = BRIGHTNESS_MAX_LEVEL;
	backlight_update_status(psb_backlight_device);
#endif
	return 0;
}

void psb_backlight_exit(void)
{
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	psb_backlight_device->props.brightness = 0;
	backlight_update_status(psb_backlight_device);
	backlight_device_unregister(psb_backlight_device);
#endif
	return;
}

struct backlight_device * psb_get_backlight_device(void)
{
#ifdef CONFIG_BACKLIGHT_CLASS_DEVICE
	return psb_backlight_device;
#endif
	return NULL;
}
