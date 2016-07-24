/*
 * Copyright © 2010 Intel Corporation
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
 * jim liu <jim.liu@intel.com>
 * Jackie Li<yaodong.li@intel.com>
 */

#include "mdfld_dsi_dpi.h"
#include "mdfld_output.h"
#include "mdfld_dsi_pkg_sender.h"
#include "psb_drv.h"
#include "mdfld_csc.h"
#include "psb_irq.h"
#include "dispmgrnl.h"

static
u16 mdfld_dsi_dpi_to_byte_clock_count(int pixel_clock_count,
		int num_lane, int bpp)
{
	return (u16)((pixel_clock_count * bpp) / (num_lane * 8)); 
}

/*
 * Calculate the dpi time basing on a given drm mode @mode
 * return 0 on success.
 * FIXME: I was using proposed mode value for calculation, may need to 
 * use crtc mode values later 
 */
int mdfld_dsi_dpi_timing_calculation(struct drm_display_mode *mode,
				struct mdfld_dsi_dpi_timing *dpi_timing,
				int num_lane, int bpp)
{
	int pclk_hsync, pclk_hfp, pclk_hbp, pclk_hactive;
	int pclk_vsync, pclk_vfp, pclk_vbp, pclk_vactive;
	
	if(!mode || !dpi_timing) {
		DRM_ERROR("Invalid parameter\n");
		return -EINVAL;
	}
	
	PSB_DEBUG_ENTRY("pclk %d, hdisplay %d, hsync_start %d, hsync_end %d, htotal %d\n", 
			mode->clock, mode->hdisplay, mode->hsync_start,
			mode->hsync_end, mode->htotal);
	PSB_DEBUG_ENTRY("vdisplay %d, vsync_start %d, vsync_end %d, vtotal %d\n", 
			mode->vdisplay, mode->vsync_start,
			mode->vsync_end, mode->vtotal);
	
	pclk_hactive = mode->hdisplay;
	pclk_hfp = mode->hsync_start - mode->hdisplay;
	pclk_hsync = mode->hsync_end - mode->hsync_start;
	pclk_hbp = mode->htotal - mode->hsync_end;
	
	pclk_vactive = mode->vdisplay;
	pclk_vfp = mode->vsync_start - mode->vdisplay;
	pclk_vsync = mode->vsync_end - mode->vsync_start;
	pclk_vbp = mode->vtotal - mode->vsync_end;

	/*
	 * byte clock counts were calculated by following formula
	 * bclock_count = pclk_count * bpp / num_lane / 8
	 */
	dpi_timing->hsync_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hsync, num_lane, bpp);
	dpi_timing->hbp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hbp, num_lane, bpp);
	dpi_timing->hfp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hfp, num_lane, bpp);
	dpi_timing->hactive_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_hactive, num_lane, bpp);

	dpi_timing->vsync_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_vsync, num_lane, bpp);
	dpi_timing->vbp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_vbp, num_lane, bpp);
	dpi_timing->vfp_count = mdfld_dsi_dpi_to_byte_clock_count(pclk_vfp, num_lane, bpp);

	PSB_DEBUG_ENTRY("DPI timings: %d, %d, %d, %d, %d, %d, %d\n", 
			dpi_timing->hsync_count, dpi_timing->hbp_count,
			dpi_timing->hfp_count, dpi_timing->hactive_count,
			dpi_timing->vsync_count, dpi_timing->vbp_count,
			dpi_timing->vfp_count);

	return 0; 
}

void mdfld_dsi_dpi_set_color_mode(struct mdfld_dsi_config *dsi_config , bool on)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	int err;
	u32  spk_pkg =  (on == true) ? MDFLD_DSI_DPI_SPK_COLOR_MODE_ON :
		MDFLD_DSI_DPI_SPK_COLOR_MODE_OFF;


	PSB_DEBUG_ENTRY("Turn  color mode %s  pkg value= %d...\n",
		(on ? "on" : "off"), spk_pkg);

	if (!sender) {
		DRM_ERROR("Failed to get DSI packet sender\n");
		return ;
	}

	/*send turn on/off color mode packet*/
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
				spk_pkg);
	if (err) {
		DRM_ERROR("Failed to send turn on packet\n");
		return ;
	}
	PSB_DEBUG_ENTRY("Turn  color mode %s successful.\n",
			(on ? "on" : "off"));
	return;
}

static int __dpi_enter_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	if (ctx->device_ready & DSI_POWER_STATE_ULPS_MASK) {
		DRM_ERROR("Broken ULPS states\n");
		return -EINVAL;
	}

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	/*wait for all FIFOs empty*/
	mdfld_dsi_wait_for_fifos_empty(sender);

	/*inform DSI host is to be put on ULPS*/
	ctx->device_ready |= DSI_POWER_STATE_ULPS_ENTER;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	DRM_INFO("%s: entered ULPS state\n", __func__);
	return 0;
}

static int __dpi_exit_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	/*enter ULPS EXIT state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	ctx->device_ready |= DSI_POWER_STATE_ULPS_EXIT;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	/*wait for 1ms as spec suggests*/
	mdelay(1);

	/*clear ULPS state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);
	return 0;
}

/**
 * Power on sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dpi_panel_power_on(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int retry, reset_count = 10;
	int i;
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;
reset_recovery:
	--reset_count;
	/*HW-Reset*/
	if (p_funcs && p_funcs->reset)
		p_funcs->reset(dsi_config);

	/*Enable DSI PLL*/
	if (!(REG_READ(regs->dpll_reg) & BIT31)) {
		if(ctx->pll_bypass_mode) {
			uint32_t dpll = 0;

			REG_WRITE(regs->dpll_reg, dpll);
			if(ctx->cck_div)
				dpll = dpll | BIT11;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT12;
			REG_WRITE(regs->dpll_reg, dpll);
			udelay(2);
			dpll = dpll | BIT13;
			REG_WRITE(regs->dpll_reg, dpll);
			dpll = dpll | BIT31;
			REG_WRITE(regs->dpll_reg, dpll);
		} else {
			REG_WRITE(regs->dpll_reg, 0x0);
			REG_WRITE(regs->fp_reg, 0x0);
			REG_WRITE(regs->fp_reg, ctx->fp);
			REG_WRITE(regs->dpll_reg, ((ctx->dpll) & ~BIT30));

			udelay(2);
			val = REG_READ(regs->dpll_reg);
			REG_WRITE(regs->dpll_reg, (val | BIT31));

			/*wait for PLL lock on pipe*/
			retry = 10000;
			while (--retry && !(REG_READ(PIPEACONF) & BIT29))
				udelay(3);
			if (!retry) {
				DRM_ERROR("PLL failed to lock on pipe\n");
				err = -EAGAIN;
				goto power_on_err;
			}
		}
	}

	/*D-PHY parameter*/
	REG_WRITE(regs->dphy_param_reg, ctx->dphy_param);

	/*Configure DSI controller*/
	REG_WRITE(regs->mipi_control_reg, ctx->mipi_control);
	REG_WRITE(regs->intr_en_reg, ctx->intr_en);
	REG_WRITE(regs->hs_tx_timeout_reg, ctx->hs_tx_timeout);
	REG_WRITE(regs->lp_rx_timeout_reg, ctx->lp_rx_timeout);
	REG_WRITE(regs->turn_around_timeout_reg,
		ctx->turn_around_timeout);
	REG_WRITE(regs->device_reset_timer_reg,
		ctx->device_reset_timer);
	REG_WRITE(regs->high_low_switch_count_reg,
		ctx->high_low_switch_count);
	REG_WRITE(regs->init_count_reg, ctx->init_count);
	REG_WRITE(regs->eot_disable_reg, ctx->eot_disable);
	REG_WRITE(regs->lp_byteclk_reg, ctx->lp_byteclk);
	REG_WRITE(regs->clk_lane_switch_time_cnt_reg,
		ctx->clk_lane_switch_time_cnt);
	REG_WRITE(regs->video_mode_format_reg, ctx->video_mode_format);
	REG_WRITE(regs->dsi_func_prg_reg, ctx->dsi_func_prg);

	/*DSI timing*/
	REG_WRITE(regs->dpi_resolution_reg, ctx->dpi_resolution);
	REG_WRITE(regs->hsync_count_reg, ctx->hsync_count);
	REG_WRITE(regs->hbp_count_reg, ctx->hbp_count);
	REG_WRITE(regs->hfp_count_reg, ctx->hfp_count);
	REG_WRITE(regs->hactive_count_reg, ctx->hactive_count);
	REG_WRITE(regs->vsync_count_reg, ctx->vsync_count);
	REG_WRITE(regs->vbp_count_reg, ctx->vbp_count);
	REG_WRITE(regs->vfp_count_reg, ctx->vfp_count);

	/*Setup pipe timing*/
	REG_WRITE(regs->htotal_reg, ctx->htotal);
	REG_WRITE(regs->hblank_reg, ctx->hblank);
	REG_WRITE(regs->hsync_reg, ctx->hsync);
	REG_WRITE(regs->vtotal_reg, ctx->vtotal);
	REG_WRITE(regs->vblank_reg, ctx->vblank);
	REG_WRITE(regs->vsync_reg, ctx->vsync);
	REG_WRITE(regs->pipesrc_reg, ctx->pipesrc);

	REG_WRITE(regs->dsppos_reg, ctx->dsppos);
	REG_WRITE(regs->dspstride_reg, ctx->dspstride);

	/*Setup plane*/
	REG_WRITE(regs->dspsize_reg, ctx->dspsize);
	REG_WRITE(regs->dspsurf_reg, ctx->dspsurf);
	REG_WRITE(regs->dsplinoff_reg, ctx->dsplinoff);
	REG_WRITE(regs->vgacntr_reg, ctx->vgacntr);

	/*restore color_coef (chrome) */
	for (i = 0; i < 6; i++) {
		REG_WRITE(regs->color_coef_reg + (i<<2), ctx->color_coef[i]);
	}

	/* restore palette (gamma) */
	for (i = 0; i < 256; i++)
		REG_WRITE(regs->palette_reg + (i<<2), ctx->palette[i]);

#ifdef CONFIG_CTP_DPST
	/* restore dpst setting */
	if (dev_priv->psb_dpst_state) {
		dpstmgr_reg_restore_locked(dsi_config);
		psb_enable_pipestat(dev_priv, 0, PIPE_DPST_EVENT_ENABLE);
	}
#endif

	/*exit ULPS state*/
	__dpi_exit_ulps_locked(dsi_config);

	/*Enable DSI Controller*/
	REG_WRITE(regs->device_ready_reg, ctx->device_ready | BIT0);

	/*set low power output hold*/
	REG_WRITE(regs->mipi_reg, (ctx->mipi | BIT16));

	/**
	 * Different panel may have different ways to have
	 * drvIC initialized. Support it!
	 */
	if (p_funcs && p_funcs->drv_ic_init) {
		if (p_funcs->drv_ic_init(dsi_config)) {
			if (!reset_count) {
				err = -EAGAIN;
				goto power_on_err;
			}
			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_DOWN, OSPM_REG_TYPE);

			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_UP, OSPM_REG_TYPE);

			DRM_ERROR("Failed to init dsi controller, reset it!\n");
			goto reset_recovery;
		}
	}

	/**
	 * Different panel may have different ways to have
	 * panel turned on. Support it!
	 */
	if (p_funcs && p_funcs->power_on)
		if (p_funcs->power_on(dsi_config)) {
			DRM_ERROR("Failed to power on panel\n");
			err = -EAGAIN;
			goto power_on_err;
		}

	/*Enable MIPI Port*/
	REG_WRITE(regs->mipi_reg, (ctx->mipi | BIT31));

	/*Enable pipe*/
	val = ctx->pipeconf;
	val &= ~0x000c0000;
	val |= BIT31;

	/* disable gamma if needed */
	if (drm_psb_enable_color_conversion == 0)
		 val &= ~(PIPEACONF_COLOR_MATRIX_ENABLE);


	REG_WRITE(regs->pipeconf_reg, val);
	REG_WRITE(regs->pipestat_reg, ctx->pipestat |
			PIPE_VBLANK_INTERRUPT_ENABLE);

	/*Wait for pipe enabling,when timing generator
	is wroking */
	if (REG_READ(regs->mipi_reg) & BIT31) {
		retry = 10000;
		while (--retry && !(REG_READ(regs->pipeconf_reg) & BIT30))
			udelay(3);

		if (!retry) {
			DRM_ERROR("Failed to enable pipe\n");
			err = -EAGAIN;
			goto power_on_err;
		}
	}
	/*enable plane*/
	val = ctx->dspcntr | BIT31;

	/* disable gamma if needed */
	if (drm_psb_enable_gamma == 0)
		 val &= ~(PIPEACONF_GAMMA);

	REG_WRITE(regs->dspcntr_reg, val);

	/*Notify PVR module that screen is on*/
	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 1);

	if (p_funcs && p_funcs->set_brightness)
		if (p_funcs->set_brightness(dsi_config, ctx->lastbrightnesslevel))
			DRM_ERROR("Failed to set panel brightness\n");

power_on_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Power off sequence for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dpi_panel_power_off(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	u32 val = 0;
	u32 tmp = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	int retry;
	int i;
	int pipe0_enabled;
	int pipe2_enabled;
	int err = 0;
	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs && p_funcs->set_brightness)
		if (p_funcs->set_brightness(dsi_config, 0))
			DRM_ERROR("Failed to set panel brightness\n");

	/*Notify PVR module that screen is off*/
	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);

	/*save the plane informaton, for it will updated*/
	ctx->dspsurf = REG_READ(regs->dspsurf_reg);
	ctx->dsplinoff = REG_READ(regs->dsplinoff_reg);
	ctx->dspsize = REG_READ(regs->dspsize_reg);
	ctx->pipestat = REG_READ(regs->pipestat_reg);
	ctx->dspcntr = REG_READ(regs->dspcntr_reg);
	ctx->dspstride= REG_READ(regs->dspstride_reg);

	tmp = REG_READ(regs->pipeconf_reg);

	/*save color_coef (chrome) */
	for (i = 0; i < 6; i++) {
		ctx->color_coef[i] = REG_READ(regs->color_coef_reg + (i<<2));
	}

	/* save palette (gamma) */
	for (i = 0; i < 256; i++)
		ctx->palette[i] = REG_READ(regs->palette_reg + (i<<2));

	/*
	 * Couldn't disable the pipe until DRM_WAIT_ON signaled by last
	 * vblank event when playing video, otherwise the last vblank event
	 * will lost when pipe disabled before vblank interrupt coming sometimes.
	 */

	/*Disable panel*/
	val = ctx->dspcntr;
	val &= ~(PIPEACONF_COLOR_MATRIX_ENABLE | PIPEACONF_GAMMA);
	REG_WRITE(regs->dspcntr_reg, (val & ~BIT31));
	/*Disable overlay & cursor panel assigned to this pipe*/
	REG_WRITE(regs->pipeconf_reg, (tmp | (0x000c0000)));

	/*Disable pipe*/
	val = REG_READ(regs->pipeconf_reg);
	ctx->pipeconf = val;
	REG_WRITE(regs->pipeconf_reg, (val & ~BIT31));

	/*wait for pipe disabling,
	pipe synchronization plus , only avaiable when
	timer generator is working*/
	if (REG_READ(regs->mipi_reg) & BIT31) {
		retry = 100000;
		while (--retry && (REG_READ(regs->pipeconf_reg) & BIT30))
			udelay(5);

		if (!retry) {
			DRM_ERROR("Failed to disable pipe\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}
	/*Disable MIPI port*/
	REG_WRITE(regs->mipi_reg, (REG_READ(regs->mipi_reg) & ~BIT31));

	/**
	 * Different panel may have different ways to have
	 * panel turned off. Support it!
	 */
	if (p_funcs && p_funcs->power_off) {
		if (p_funcs->power_off(dsi_config)) {
			DRM_ERROR("Failed to power off panel\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}

	/*clear Low power output hold*/
	REG_WRITE(regs->mipi_reg, (REG_READ(regs->mipi_reg) & ~BIT16));

	/*Disable DSI controller*/
	REG_WRITE(regs->device_ready_reg, (ctx->device_ready & ~BIT0));

	/*enter ULPS*/
	__dpi_enter_ulps_locked(dsi_config);

	/*Disable DSI PLL*/
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled) {
		REG_WRITE(regs->dpll_reg , 0x0);
		/*power gate pll*/
		REG_WRITE(regs->dpll_reg, BIT30);
	}

power_off_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Send TURN_ON package to dpi panel to turn it on
 */
static int mdfld_dsi_dpi_panel_turn_on(struct mdfld_dsi_config *dsi_config,
		struct panel_funcs *p_funcs)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	struct mdfld_dsi_hw_context *ctx;
	int err = 0;

	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_TURN_ON);
	/*According HW DSI spec, here need wait for 100ms*/
	/*To optimize dpms flow, move sleep out of mutex*/
	/* msleep(100); */

	ctx = &dsi_config->dsi_hw_context;
	if (p_funcs->set_brightness(dsi_config, ctx->lastbrightnesslevel))
		DRM_ERROR("Failed to set panel brightness\n");

	return err;
}

/**
 * Send SHUT_DOWN package to dpi panel to turn if off
 */
static int mdfld_dsi_dpi_panel_shut_down(struct mdfld_dsi_config *dsi_config,
		struct panel_funcs *p_funcs)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_get_pkg_sender(dsi_config);
	struct mdfld_dsi_hw_context *ctx;
	int err = 0;

	ctx = &dsi_config->dsi_hw_context;
	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs->set_brightness(dsi_config, 0))
		DRM_ERROR("Failed to set panel brightness\n");

	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
	/*According HW DSI spec, here need wait for 100ms*/
	/*To optimize dpms flow, move sleep out of mutex*/
	/* msleep(100); */

	return err;
}

/**
 * Setup Display Controller to turn on/off a video mode panel.
 * Most of the video mode MIPI panel should follow the power on/off
 * sequence in this function.
 * NOTE: do NOT modify this function for new panel Enabling. Register
 * new panel function callbacks to make this function available for a
 * new video mode panel
 */
static int __mdfld_dsi_dpi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_connector *dsi_connector;
	struct mdfld_dsi_dpi_output *dpi_output;
	struct mdfld_dsi_config *dsi_config;
	struct panel_funcs *p_funcs;
	int pipe;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	static int last_ospm_suspend = -1;

	if (!encoder) {
		DRM_ERROR("Invalid encoder\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("%s, last_ospm_suspend = %s\n", (on ? "on" : "off"),
			(last_ospm_suspend ? "true" : "false"));

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	p_funcs = dpi_output->p_funcs;
	pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);
	dsi_connector = mdfld_dsi_encoder_get_connector(dsi_encoder);
	if (!dsi_connector) {
		DRM_ERROR("Invalid connector\n");
		return -EINVAL;
	}
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (dsi_connector->status != connector_status_connected)
		return 0;

	mutex_lock(&dsi_config->context_lock);

	if (last_ospm_suspend == -1)
		last_ospm_suspend = false;

	if (dpi_output->first_boot && dsi_config->dsi_hw_context.panel_on) {

		if (Check_fw_initilized_reusable(dsi_config, p_funcs)) {
			DRM_INFO("skip panle power setting for first boot!"
					" panel is already powered on\n");
			goto fun_exit;
		} else
			dsi_config->dsi_hw_context.panel_on = 0;
	}

	/**
	 * if ospm has turned panel off, but dpms tries to turn panel on, skip
	 */
	if (dev_priv->dpms_on_off && on && last_ospm_suspend)
		goto fun_exit;

	switch (on) {
	case true:
		/* panel is already on */
		if (dsi_config->dsi_hw_context.panel_on)
			goto fun_exit;
		/* For DPMS case, just turn on/off panel */
		if (dev_priv->dpms_on_off) {
			if (mdfld_dsi_dpi_panel_turn_on(dsi_config, p_funcs)) {
				DRM_ERROR("Faild to turn on panel\n");
				goto set_power_err;
			}
		} else {
			if (__dpi_panel_power_on(dsi_config, p_funcs)) {
				DRM_ERROR("Faild to turn on panel\n");
				goto set_power_err;
			}
		}
		/**
		 * If power on, turn off color mode by default,
		 * let panel in full color mode
		 */
		mdfld_dsi_dpi_set_color_mode(dsi_config, false);

		dsi_config->dsi_hw_context.panel_on = 1;
		last_ospm_suspend = false;
		break;
	case false:
		if (dev_priv->dpms_on_off &&
				dsi_config->dsi_hw_context.panel_on) {
			if (mdfld_dsi_dpi_panel_shut_down(dsi_config, p_funcs))
				DRM_ERROR("Faild to shutdown panel\n");

			last_ospm_suspend = false;
		} else if (!dev_priv->dpms_on_off && !last_ospm_suspend) {
			if (__dpi_panel_power_off(dsi_config, p_funcs)) {
				DRM_ERROR("Faild to turn off panel\n");
				goto set_power_err;
			}
			/* ospm suspend called? */
			last_ospm_suspend = true;
		}
		dsi_config->dsi_hw_context.panel_on = 0;
		break;
	default:
		break;
	}

fun_exit:
	mutex_unlock(&dsi_config->context_lock);
	/*To optimize dpms with context lock, move panel sleep out of mutex*/
	if (dev_priv->dpms_on_off)
		msleep(100);
	PSB_DEBUG_ENTRY("successfully\n");
	return 0;
set_power_err:
	mutex_unlock(&dsi_config->context_lock);
	PSB_DEBUG_ENTRY("unsuccessfully!!!!\n");
	return -EAGAIN;
}

void mdfld_dsi_dpi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	int pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_dpi_output *dpi_output = NULL;
	u32 mipi_reg = MIPI;
	u32 pipeconf_reg = PIPEACONF;

	PSB_DEBUG_ENTRY("set power %s on pipe %d\n", on ? "On" : "Off", pipe);

	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);

	if (pipe)
		if (!(dev_priv->panel_desc & DISPLAY_B) ||
				!(dev_priv->panel_desc & DISPLAY_C))
			return;

	if (pipe) {
		mipi_reg = MIPI_C;
		pipeconf_reg = PIPECCONF;
	}

	/*start up display island if it was shutdown*/
	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return;

	/**
	 * if TMD panel call new power on/off sequences instead.
	 * NOTE: refine TOSHIBA panel code later
	 */
	__mdfld_dsi_dpi_set_power(encoder, on);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static
void mdfld_dsi_dpi_dpms(struct drm_encoder *encoder, int mode)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_config *dsi_config;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;

	if (!(drm_psb_use_cases_control & PSB_DPMS_ENABLE))
		return;

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY(
			"%s\n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));
	if (!gbdispstatus) {
		PSB_DEBUG_ENTRY(
		"panel in suspend status, skip turn on/off from DMPS");
		return ;
	}

	mutex_lock(&dev_priv->dpms_mutex);
	dev_priv->dpms_on_off = true;
	if (mode == DRM_MODE_DPMS_ON)
		mdfld_dsi_dpi_set_power(encoder, true);
	else
		mdfld_dsi_dpi_set_power(encoder, false);
	dev_priv->dpms_on_off = false;
	mutex_unlock(&dev_priv->dpms_mutex);
}

static
bool mdfld_dsi_dpi_mode_fixup(struct drm_encoder *encoder,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_display_mode *fixed_mode = dsi_config->fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	if(fixed_mode) {
		adjusted_mode->hdisplay = fixed_mode->hdisplay;
		adjusted_mode->hsync_start = fixed_mode->hsync_start;
		adjusted_mode->hsync_end = fixed_mode->hsync_end;
		adjusted_mode->htotal = fixed_mode->htotal;
		adjusted_mode->vdisplay = fixed_mode->vdisplay;
		adjusted_mode->vsync_start = fixed_mode->vsync_start;
		adjusted_mode->vsync_end = fixed_mode->vsync_end;
		adjusted_mode->vtotal = fixed_mode->vtotal;
		adjusted_mode->clock = fixed_mode->clock;
		drm_mode_set_crtcinfo(adjusted_mode, CRTC_INTERLACE_HALVE_V);
	}
	
	return true;
}

static
void mdfld_dsi_dpi_prepare(struct drm_encoder *encoder)
{
	PSB_DEBUG_ENTRY("\n");
}

static
void mdfld_dsi_dpi_commit(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_dpi_output *dpi_output;
	struct mdfld_dsi_hw_context *ctx;
	struct mdfld_dsi_config *dsi_config;
	struct drm_device *dev;
	u32 temp_val = 0;

	PSB_DEBUG_ENTRY("\n");

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dpi_output = MDFLD_DSI_DPI_OUTPUT(dsi_encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dev = dsi_config->dev;
	ctx = &dsi_config->dsi_hw_context;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND, OSPM_UHB_FORCE_POWER_ON))
		return;
	temp_val = REG_READ(PIPEACONF);
	temp_val &= ~(BIT27 | BIT28);
	/* Setup pipe configuration for different panels*/
	REG_WRITE(PIPEACONF, temp_val | (ctx->pipeconf & (BIT27 | BIT28)));
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	/*Everything is ready, commit DSI hw context to HW*/
	__mdfld_dsi_dpi_set_power(encoder, true);
	dpi_output->first_boot = 0;
}

/**
 * Setup DPI timing for video mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static void __mdfld_dsi_dpi_set_timing(struct mdfld_dsi_config *config,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_dpi_timing dpi_timing;
	struct mdfld_dsi_hw_context *ctx;

	if (!config) {
		DRM_ERROR("Invalid parameters\n");
		return;
	}

	mode = adjusted_mode;
	ctx = &config->dsi_hw_context;

	mutex_lock(&config->context_lock);

	/*dpi resolution*/
	ctx->dpi_resolution = (mode->vdisplay << 16 | mode->hdisplay);

	/*Calculate DPI timing*/
	mdfld_dsi_dpi_timing_calculation(mode, &dpi_timing,
					config->lane_count,
					config->bpp);

	/*update HW context with new DPI timings*/
	ctx->hsync_count = dpi_timing.hsync_count;
	ctx->hbp_count = dpi_timing.hbp_count;
	ctx->hfp_count = dpi_timing.hfp_count;
	ctx->hactive_count = dpi_timing.hactive_count;
	ctx->vsync_count = dpi_timing.vsync_count;
	ctx->vbp_count = dpi_timing.vbp_count;
	ctx->vfp_count = dpi_timing.vfp_count;

	mutex_unlock(&config->context_lock);
}

static
void mdfld_dsi_dpi_mode_set(struct drm_encoder *encoder,
				   struct drm_display_mode *mode,
				   struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	int pipe = mdfld_dsi_encoder_get_pipe(dsi_encoder);


	PSB_DEBUG_ENTRY("set mode %dx%d on pipe %d",
			mode->hdisplay, mode->vdisplay, pipe);

	/**
	 * if TMD panel call new power on/off sequences instead.
	 * NOTE: refine TOSHIBA panel code later
	 */
	__mdfld_dsi_dpi_set_timing(dsi_config, mode, adjusted_mode);
}

static
void mdfld_dsi_dpi_save(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	PSB_DEBUG_ENTRY("\n");

	__mdfld_dsi_dpi_set_power(encoder, false);
}

static
void mdfld_dsi_dpi_restore(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	PSB_DEBUG_ENTRY("\n");

	__mdfld_dsi_dpi_set_power(encoder, true);
}

static const
struct drm_encoder_helper_funcs dsi_dpi_generic_encoder_helper_funcs = {
	.save = mdfld_dsi_dpi_save,
	.restore = mdfld_dsi_dpi_restore,
	.dpms = mdfld_dsi_dpi_dpms,
	.mode_fixup = mdfld_dsi_dpi_mode_fixup,
	.prepare = mdfld_dsi_dpi_prepare,
	.mode_set = mdfld_dsi_dpi_mode_set,
	.commit = mdfld_dsi_dpi_commit,
};

static const
struct drm_encoder_funcs dsi_dpi_generic_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

/**
 * Exit from DSR
 */
void mdfld_dsi_dpi_exit_idle(struct drm_device *dev,
				u32 update_src,
				void *p_surfaceAddr,
				bool check_hw_on_only)
{
	struct drm_psb_private * dev_priv = dev->dev_private;
	unsigned long irqflags;

	/* PSB_DEBUG_ENTRY("\n"); */

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
		OSPM_UHB_ONLY_IF_ON)) {
		DRM_ERROR("display island is in off state\n");
		return;
	}

	spin_lock_irqsave(&dev_priv->irqmask_lock, irqflags);
	/* update the surface base address. */
	if (p_surfaceAddr) {
		REG_WRITE(DSPASURF, *((u32 *)p_surfaceAddr));
#if defined(CONFIG_MDFD_DUAL_MIPI)
		REG_WRITE(DSPCSURF, *((u32 *)p_surfaceAddr));
#endif
	}

	mid_enable_pipe_event(dev_priv, 0);
	psb_enable_pipestat(dev_priv, 0, PIPE_VBLANK_INTERRUPT_ENABLE);

	spin_unlock_irqrestore(&dev_priv->irqmask_lock, irqflags);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/*
 * Init DSI DPI encoder. 
 * Allocate an mdfld_dsi_encoder and attach it to given @dsi_connector
 * return pointer of newly allocated DPI encoder, NULL on error
 */ 
struct mdfld_dsi_encoder *mdfld_dsi_dpi_init(struct drm_device *dev,
		struct mdfld_dsi_connector *dsi_connector)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dsi_dpi_output *dpi_output = NULL;
	struct mdfld_dsi_config *dsi_config;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL;
	int pipe;

	PSB_DEBUG_ENTRY("\n");
 
	if (!dsi_connector) {
		DRM_ERROR("Invalid parameters\n");
		return NULL;
	}
	dsi_config = mdfld_dsi_get_config(dsi_connector);
	pipe = dsi_connector->pipe;
	dsi_connector->status = connector_status_connected;

	dpi_output = kzalloc(sizeof(struct mdfld_dsi_dpi_output), GFP_KERNEL);
	if (!dpi_output) {
		DRM_ERROR("No memory\n");
		return NULL;
	}

	dpi_output->dev = dev;
	dpi_output->first_boot = 1;

	if (pipe)
		dev_priv->dpi_output2 = dpi_output;
	else
		dev_priv->dpi_output = dpi_output;

	/*create drm encoder object*/
	connector = &dsi_connector->base.base;
	encoder = &dpi_output->base.base;
	drm_encoder_init(dev,
			encoder,
			&dsi_dpi_generic_encoder_funcs,
			DRM_MODE_ENCODER_DSI);
	drm_encoder_helper_add(encoder,
			&dsi_dpi_generic_encoder_helper_funcs);
	
	/*attach to given connector*/
	drm_mode_connector_attach_encoder(connector, encoder);
	connector->encoder = encoder;
	
	/*set possible crtcs and clones*/
	if(dsi_connector->pipe) {
		encoder->possible_crtcs = (1 << 2);
		encoder->possible_clones = (1 << 1);
	} else {
		encoder->possible_crtcs = (1 << 0);
		encoder->possible_clones = (1 << 0);
	}

	dev_priv->dsr_fb_update = 0;
	dev_priv->b_dsr_enable = false;
	dev_priv->exit_idle = mdfld_dsi_dpi_exit_idle;
#if defined(CONFIG_MDFLD_DSI_DPU) || defined(CONFIG_MDFLD_DSI_DSR)
	dev_priv->b_dsr_enable_config = true;
#endif /*CONFIG_MDFLD_DSI_DSR*/

	PSB_DEBUG_ENTRY("successfully\n");

	return &dpi_output->base;
}
