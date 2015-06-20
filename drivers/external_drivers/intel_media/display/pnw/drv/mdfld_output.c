/*
 * Copyright (c)  2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicensen
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
 * Thomas Eaton <thomas.g.eaton@intel.com>
 * Scott Rowe <scott.m.rowe@intel.com>
*/

#include <linux/init.h>
#include <linux/kernel.h>
#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_output.h"
#include "android_hdmi.h"
#include "dispmgrnl.h"
#include "psb_dpst_func.h"
#include "mdfld_dsi_dbi_dsr.h"
#include "displays/hdmi.h"
#include "psb_drv.h"

#ifdef CONFIG_GFX_RTPM
#include <linux/pm_runtime.h>
#endif

#define TMD_6X10_PANEL_NAME	"TMD BB PRx"
bool is_tmd_6x10_panel(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	int len = 0;

	if (unlikely(!dev_priv))
		return -EINVAL;

	len = strnlen(TMD_6X10_PANEL_NAME, PANEL_NAME_MAX_LEN);
	if (!strncmp(TMD_6X10_PANEL_NAME, dev_priv->panel_info.name, len))
		return true;
	else
		return false;
}

int get_panel_mode(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv;

	if (unlikely(!dev))
		return -EINVAL;

	dev_priv = dev->dev_private;
	if (unlikely(!dev))
		return -EINVAL;

	return dev_priv->panel_info.mode;
}

bool is_cmd_mode_panel(struct drm_device *dev)
{
	int mode = get_panel_mode(dev);

	return (mode == MDFLD_DSI_ENCODER_DBI) ? true : false;
}

static void init_panel(struct drm_device *dev, int pipe)
{
#ifdef CONFIG_SUPPORT_HDMI
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *) dev->dev_private;
#endif
	int ret = 0;

#ifdef CONFIG_SUPPORT_HDMI
	if (pipe == 1) {
		PSB_DEBUG_ENTRY( "GFX: Initializing HDMI");
		android_hdmi_driver_init(dev, &dev_priv->mode_dev);
		return;
	}
#endif

	ret = mdfld_dsi_output_init(dev, pipe);
	if (ret)
		DRM_ERROR("%s, dsi_output_init error!\n", __func__);
}

/*
 * use to overwrite fw setting
 */
static void Overwrite_fw_setting(struct mdfld_dsi_config *dsi_config)
{
	return;
}
/*
* In some case Fw has initialized different with driver requirement.
* so driver needs to check whether reusable or not.
* if can reuse, driver can overwrite some setting according driver.
* if can not reuse, driver need reset-panel and display controller.
*/
bool Check_fw_initilized_reusable(struct mdfld_dsi_config *dsi_config,
				struct panel_funcs *p_funcs)
{
	struct drm_device *dev = NULL;
	struct mdfld_dsi_hw_registers *regs = NULL;
	int fw_type = 0xff;
	bool b_reuseable = true;
	u32 pipe_config = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!p_funcs || !dsi_config) {
		DRM_ERROR("invalid parameter!\n");
		return false;
	}

	dev = dsi_config->dev;
	regs = &dsi_config->regs;
	pipe_config = REG_READ(regs->pipeconf_reg);

	if (!(pipe_config & PIPEACONF_ENABLE))
		return false;

	/*check fw initialized command/vide mode*/
	if (pipe_config & PIPEACONF_DSR)
		fw_type = MDFLD_DSI_ENCODER_DBI;
	else
		fw_type = MDFLD_DSI_ENCODER_DPI;

	/*check whether the same as driver requirment*/
	if (dsi_config->type == fw_type) {
		b_reuseable = true;
		/*overwrite or totall reuse*/
		Overwrite_fw_setting(dsi_config);
	} else {
		DRM_INFO("can not reuse fw panel setting,do reset\n") ;
		/*reset dispaly controller*/
		acquire_ospm_lock();
		ospm_power_island_down(OSPM_DISPLAY_ISLAND);
		ospm_power_island_up(OSPM_DISPLAY_ISLAND);
		release_ospm_lock();
		b_reuseable = false;
	}

	return b_reuseable;
}

void mdfld_output_init(struct drm_device *dev)
{
	/* initialization PIPEA */
	init_panel(dev, 0);

#ifdef CONFIG_MDFD_DUAL_MIPI
	/* initialization for PIPEC */
	init_panel(dev, 2);
#endif

#ifdef CONFIG_SUPPORT_HDMI
	/* initialization for PIPEB */
	init_panel(dev, 1);
#endif
}

void intel_mid_panel_register(
		void (*panel_init)(struct drm_device *, struct panel_funcs *))
{
	struct drm_device *dev = g_drm_dev;
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_dpi_output *dpi_output = NULL;
	struct mdfld_dsi_config *dsi_config = NULL;
	struct panel_funcs *p_funcs = NULL;
	struct panel_info dsi_panel_info = {0, 0};
	struct drm_connector *connector = NULL;
	struct mdfld_dsi_connector *dsi_connector = NULL;
	int ret = 0;

	PSB_DEBUG_ENTRY("\n");

	if (likely(dev)) {
		dev_priv = dev->dev_private;
	} else {
		pr_err("%s, drm dev is NULL!\n", __func__);
		return;
	}

	if (likely(dev_priv)) {
		dbi_output = dev_priv->dbi_output;
		dpi_output = dev_priv->dpi_output;
		dsi_config = dev_priv->dsi_configs[0];
	} else {
		pr_err("%s, dev private is NULL!\n", __func__);
		return;
	}

	if (unlikely(!dsi_config)) {
		pr_err("%s, dbi_output or dsi_config is NULL!\n", __func__);
		return;
	}

	p_funcs = kzalloc(sizeof(struct panel_funcs), GFP_KERNEL);
	if (unlikely(!p_funcs)) {
		pr_err("%s, faild to allock panel_funcs\n", __func__);
		return;
	}
	if (dsi_config->type == MDFLD_DSI_ENCODER_DBI)
		dbi_output->p_funcs = p_funcs;
	else
		dpi_output->p_funcs = p_funcs;

	connector = &dsi_config->connector->base.base;
	dsi_connector = dsi_config->connector;

	/*register panel callbacks*/
	(*panel_init)(dev, p_funcs);

	if (p_funcs->get_config_mode)
		dsi_config->fixed_mode = p_funcs->get_config_mode();
	if (p_funcs->get_panel_info)
		p_funcs->get_panel_info(0, &dsi_panel_info);

	connector->display_info.width_mm = dsi_panel_info.width_mm;
	connector->display_info.height_mm = dsi_panel_info.height_mm;

	if (p_funcs->detect) {
		ret = p_funcs->detect(dsi_config);
		if (ret) {
			pr_warn("%s, fail to detect panel\n", __func__);
			dsi_connector->status =
				connector_status_disconnected;
		} else {
			dsi_connector->status = connector_status_connected;
		}
	} else {
		dsi_connector->status = connector_status_disconnected;
	}

	if (dsi_connector->status == connector_status_connected)
		dev_priv->panel_desc |= DISPLAY_A;

	if (p_funcs->dsi_controller_init)
		p_funcs->dsi_controller_init(dsi_config);

	if (drm_psb_no_fb == 0) {
		/*register fb device*/
		psb_fbdev_init(dev);
		drm_kms_helper_poll_init(dev);
	}

	/*must be after mrst_get_fuse_settings()*/
	psb_backlight_init(dev);

	/* Post OSPM init */
	ospm_post_init(dev);

#ifdef CONFIG_CTP_DPST
	/* init display manager */
	dpst_init(dev, AGGRESSIVE_LEVEL_DEFAULT);
#endif

	mdfld_dsi_dsr_enable(dsi_config);

#ifdef CONFIG_GFX_RTPM
	/*enable runtime pm at last*/
	pm_runtime_put_noidle(&dev->pdev->dev);
#endif
}
