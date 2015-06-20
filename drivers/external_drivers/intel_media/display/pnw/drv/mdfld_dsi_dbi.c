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
 *  jim liu <jim.liu@intel.com>
 *  Jackie Li<yaodong.li@intel.com>
 */

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dbi_dpu.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"
#include "psb_powermgmt.h"
#include "mdfld_dsi_dbi_dsr.h"
#include "dispmgrnl.h"

/**
 * Enter DSR 
 */
void mdfld_dsi_dbi_enter_dsr (struct mdfld_dsi_dbi_output * dbi_output, int pipe)
{
	return;
}

#ifndef CONFIG_MDFLD_DSI_DPU

int mdfld_dsi_dbi_async_check_fifo_empty(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output **dbi_outputs = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_pkg_sender *sender = NULL;
	int err = 0;

	dbi_outputs = dsr_info->dbi_outputs;
	dbi_output = 0 ? dbi_outputs[1] : dbi_outputs[0];
	if (!dbi_output)
		return 0;

	sender = mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	err = mdfld_dsi_check_fifo_empty(sender);
	return err;
}

/*
 * use hw te to update fb
 */
int mdfld_dsi_dbi_async_flip_fb_update(struct drm_device *dev, int pipe)
{
	return 0;
}

/**
 * Exit from DSR
 */
void mdfld_dsi_dbi_exit_dsr(struct drm_device *dev,
		u32 update_src,
		void *p_surfaceAddr,
		bool check_hw_on_only)
{
}


static
void intel_dsi_dbi_update_fb(struct mdfld_dsi_dbi_output *dbi_output)
{
	struct mdfld_dsi_pkg_sender *sender =
		mdfld_dsi_encoder_get_pkg_sender(&dbi_output->base);
	struct drm_device *dev = dbi_output->dev;
	struct drm_crtc *crtc = dbi_output->base.base.crtc;
	struct psb_intel_crtc *psb_crtc =
		(crtc) ? to_psb_intel_crtc(crtc) : NULL;
	int pipe = dbi_output->channel_num ? 2 : 0;
	u32 dpll_reg = MRST_DPLL_A;
	u32 dspcntr_reg = DSPACNTR;
	u32 pipeconf_reg = PIPEACONF;
	u32 dsplinoff_reg = DSPALINOFF;
	u32 dspsurf_reg = DSPASURF;

	/* if mode setting on-going, back off */

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return;
	}

	if ((dbi_output->mode_flags & MODE_SETTING_ON_GOING) ||
	    (psb_crtc && (psb_crtc->mode_flags & MODE_SETTING_ON_GOING)) ||
	    !(dbi_output->mode_flags & MODE_SETTING_ENCODER_DONE))
		return;

	if (pipe == 2) {
		dspcntr_reg = DSPCCNTR;
		pipeconf_reg = PIPECCONF;
		dsplinoff_reg = DSPCLINOFF;
		dspsurf_reg = DSPCSURF;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_FORCE_POWER_ON)) {
		DRM_ERROR("hw begin failed\n");
		return;
	}

	/* check DBI FIFO status */
	if (!(REG_READ(dpll_reg) & DPLL_VCO_ENABLE) ||
	   !(REG_READ(dspcntr_reg) & DISPLAY_PLANE_ENABLE) ||
	   !(REG_READ(pipeconf_reg) & DISPLAY_PLANE_ENABLE))
		goto update_fb_out0;

	/* refresh plane changes */

	REG_WRITE(dsplinoff_reg, REG_READ(dsplinoff_reg));
	REG_WRITE(dspsurf_reg, REG_READ(dspsurf_reg));
	REG_READ(dspsurf_reg);

	mdfld_dsi_send_dcs(sender,
			   write_mem_start,
			   NULL,
			   0,
			   CMD_DATA_SRC_PIPE,
			   MDFLD_DSI_SEND_PACKAGE);
	dbi_output->dsr_fb_update_done = true;
	mdfld_dsi_cmds_kick_out(sender);

update_fb_out0:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

/* Perodically update dbi panel */
void mdfld_dbi_update_panel(struct drm_device *dev, int pipe)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;
	struct mdfld_dsi_dbi_output **dbi_outputs;
	struct mdfld_dsi_dbi_output *dbi_output;
	struct mdfld_dsi_config *dsi_config;
	struct mdfld_dsi_hw_context *ctx;

	if (!dsr_info)
		return;

	dbi_outputs = dsr_info->dbi_outputs;
	dbi_output = pipe ? dbi_outputs[1] : dbi_outputs[0];
	dsi_config = pipe ? dev_priv->dsi_configs[1] : dev_priv->dsi_configs[0];

	if (!dbi_output || !dsi_config || (pipe == 1))
		return;

	ctx = &dsi_config->dsi_hw_context;

	/*lock dsi config*/
	mutex_lock(&dsi_config->context_lock);

	/*if FB is damaged and panel is on update on-panel FB*/
	if (!ctx->panel_on)
		goto update_out;

	intel_dsi_dbi_update_fb(dbi_output);

update_out:
	mutex_unlock(&dsi_config->context_lock);
}

int mdfld_dbi_dsr_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct mdfld_dbi_dsr_info *dsr_info = dev_priv->dbi_dsr_info;

	PSB_DEBUG_ENTRY("\n");

	if (!dsr_info || IS_ERR(dsr_info)) {
		dsr_info = kzalloc(sizeof(struct mdfld_dbi_dsr_info),
				   GFP_KERNEL);
		if (!dsr_info) {
			DRM_ERROR("No memory\n");
			return -ENOMEM;
		}

		dev_priv->dbi_dsr_info = dsr_info;
	}

	return 0;
}
#endif

static int mdfld_initia_panel_inside_fb(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct drm_device *dev = dsi_config->dev;
	struct drm_psb_private *dev_priv;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	int err = 0;
	PSB_DEBUG_ENTRY("\n");
	dev_priv = dev->dev_private;
	/*if recovery from suspend case, set black frame buffer*/
	if (!gbdispstatus) {
		REG_WRITE(regs->dspstride_reg, dev_priv->init_screen_stride);
		REG_WRITE(regs->dspsize_reg, dev_priv->init_screen_size);
		REG_WRITE(regs->dsplinoff_reg, dev_priv->init_screen_offset);
		REG_WRITE(regs->dspsurf_reg, dev_priv->init_screen_start);
	}
	/*clean on-panel FB*/
	/*re-initizlize the te_seq count & screen update*/
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}
	atomic64_set(&sender->last_screen_update, 0);
	atomic64_set(&sender->te_seq, 1);
	/*for initia clear directly write out,
	  so temp disable TE trigger */
	REG_WRITE(regs->mipi_reg,
		(REG_READ(regs->mipi_reg) & (~(TE_TRIGGER_GPIO_PIN |
				 TE_TRIGGER_DSI_PROTOCOL))));
	err = mdfld_dsi_send_dcs(sender,
			write_mem_start,
			NULL,
			0,
			CMD_DATA_SRC_PIPE,
			MDFLD_DSI_SEND_PACKAGE);
	if (err)
		DRM_ERROR("%s - sent write_mem_start faild\n", __func__);
	/*wait for updata completly */
	mdfld_dsi_wait_for_fifos_empty(sender);
	/*restor back FB update by TE trigger*/
	REG_WRITE(regs->mipi_reg,
			(REG_READ(regs->mipi_reg) | TE_TRIGGER_GPIO_PIN));
	return err;
}
static int __dbi_enter_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	if (ctx->device_ready & DSI_POWER_STATE_ULPS_MASK) {
		DRM_ERROR("Broken ULPS states\n");
		return -EINVAL;
	}

	/*wait for all FIFOs empty*/
	mdfld_dsi_wait_for_fifos_empty(sender);

	/*inform DSI host is to be put on ULPS*/
	ctx->device_ready |= (DSI_POWER_STATE_ULPS_ENTER |
				 DSI_DEVICE_READY);
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);
	mdelay(1);

	/* set AFE hold value*/
	REG_WRITE(regs->mipi_reg,
	     REG_READ(regs->mipi_reg) & (~PASS_FROM_SPHY_TO_AFE));

	PSB_DEBUG_ENTRY("%s: entered ULPS state\n", __func__);
	return 0;
}
static bool mdfld_get_data_line_LP11_status(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = NULL;
	struct drm_device *dev = NULL;
	int    retry = 1000;
	bool   ret = false;

	if (!dsi_config) {
		ret = false;
		goto __fun_exit;
	}

	dev = dsi_config->dev;
	regs = &dsi_config->regs;
	/*this check is depended clock stop feature at b05c[1]
	if clock stop feature enabled, the clock line is synchronous
	with data line, which means no data transferring, the clock
	will stop and keep at LP11*/
	while (!(REG_READ(regs->mipi_reg) & BIT17) && retry) {
		mdelay(1);
		retry--;
	}

	if (retry == 0) {
		DRM_INFO("Can not get data line LP11\n");
		ret = false;
	}

	ret = true;
__fun_exit:
	return ret;

}
static int __dbi_exit_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	/*inform DSI host is to be put on ULPS*/
	ctx->device_ready |= (DSI_POWER_STATE_ULPS_ENTER |
				 DSI_DEVICE_READY);
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	mdelay(1);
	/* clear AFE hold value*/
	REG_WRITE(regs->mipi_reg,
		REG_READ(regs->mipi_reg) | PASS_FROM_SPHY_TO_AFE);

	/*enter ULPS EXIT state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	ctx->device_ready |= (DSI_POWER_STATE_ULPS_EXIT |
			DSI_DEVICE_READY);
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	/*wait for 1ms as spec suggests*/
	mdelay(1);

	/*clear ULPS state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	ctx->device_ready |= DSI_DEVICE_READY;
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	mdelay(1);

	PSB_DEBUG_ENTRY("%s: exited ULPS state\n", __func__);
	return 0;
}
/* dbi interface power on*/
int __dbi_power_on(struct mdfld_dsi_config *dsi_config)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int retry;
	int i;
	int err = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	/*Enable DSI PLL*/
	if (!(REG_READ(regs->dpll_reg) & BIT31)) {
		if (ctx->pll_bypass_mode) {
			uint32_t dpll = 0;

			REG_WRITE(regs->dpll_reg, dpll);
			if (ctx->cck_div)
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

	/*exit ULPS*/
	if (__dbi_exit_ulps_locked(dsi_config)) {
		DRM_ERROR("Failed to exit ULPS\n");
		goto power_on_err;
	}
	/*update MIPI port config*/
	REG_WRITE(regs->mipi_reg, ctx->mipi |
			 REG_READ(regs->mipi_reg));

	/*unready dsi adapter for re-programming*/
	REG_WRITE(regs->device_ready_reg,
		REG_READ(regs->device_ready_reg) & ~(DSI_DEVICE_READY));

	/*according spec, if clock stop feature enable b05c[1],
	* then un-ready MIPI adapter need wait HS speed mode to
	* low speed mode. per calculation 1us is enough
	*/
	udelay(1);

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
	REG_WRITE(regs->hs_ls_dbi_enable_reg, ctx->hs_ls_dbi_enable);
	REG_WRITE(regs->dsi_func_prg_reg, ctx->dsi_func_prg);

	/*DBI bw ctrl*/
	REG_WRITE(regs->dbi_bw_ctrl_reg, ctx->dbi_bw_ctrl);

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

	/*restore color_coef (chrome) */
	for (i = 0; i < 6; i++)
		REG_WRITE(regs->color_coef_reg + (i<<2), ctx->color_coef[i]);

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

	/*Setup plane*/
	REG_WRITE(regs->dspsize_reg, ctx->dspsize);
	REG_WRITE(regs->dspsurf_reg, ctx->dspsurf);
	REG_WRITE(regs->dsplinoff_reg, ctx->dsplinoff);
	REG_WRITE(regs->vgacntr_reg, ctx->vgacntr);

	/*enable plane*/
	val = ctx->dspcntr | BIT31;
	REG_WRITE(regs->dspcntr_reg, val);

	/*ready dsi adapter*/
	REG_WRITE(regs->device_ready_reg,
		REG_READ(regs->device_ready_reg) | DSI_DEVICE_READY);
	mdelay(1);

	/*Enable pipe*/
	val = ctx->pipeconf;
	val &= ~0x000c0000;
	val |= BIT31 | PIPEACONF_DSR;
	REG_WRITE(regs->pipeconf_reg, val);

	/*Wait for pipe enabling,when timing generator is working */
	retry = 10000;
	while (--retry && !(REG_READ(regs->pipeconf_reg) & BIT30))
		udelay(3);

	if (!retry) {
		DRM_ERROR("Failed to enable pipe\n");
		err = -EAGAIN;
		goto power_on_err;
	}

power_on_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Power on sequence for command mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dbi_panel_power_on(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	struct drm_device *dev;
	int reset_count = 10;
	int err = 0;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("\n");

	if (!dsi_config)
		return -EINVAL;

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	mdfld_dsi_dsr_forbid_locked(dsi_config);
reset_recovery:
	--reset_count;
	err = 0;

	if (__dbi_power_on(dsi_config)) {
		DRM_ERROR("Failed to init display controller!\n");
		err = -EAGAIN;
		goto power_on_err;
	}

	/*enable TE, will need it in panel power on*/
	mdfld_enable_te(dev, dsi_config->pipe);

	/*according panel vender , panel reset prefer happens at LP11*/
	mdfld_get_data_line_LP11_status(dsi_config);

	/*after entering dstb mode, need reset*/
	if (p_funcs && p_funcs->reset)
		p_funcs->reset(dsi_config);

	/* after reset keep in LP11 8ms */
	mdfld_get_data_line_LP11_status(dsi_config);
	mdelay(8);

	/**
	 * Different panel may have different ways to have
	 * drvIC initialized. Support it!
	 */
	if (p_funcs && p_funcs->drv_ic_init) {
		if (p_funcs->drv_ic_init(dsi_config)) {
			DRM_ERROR("Failed to init dsi controller!\n");
			err = -EAGAIN;
			goto power_on_err;
		}
	}
	/* panel insider fb is random after exit deep standby mode
	*re-initia panel insider fb before turn on
	*/
	mdfld_initia_panel_inside_fb(dsi_config);

	/* keep in LP11 for 8 ms before power on */
	mdfld_get_data_line_LP11_status(dsi_config);
	mdelay(8);

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

	if (p_funcs && p_funcs->set_brightness)
		if (p_funcs->set_brightness(dsi_config,
					ctx->lastbrightnesslevel))
			DRM_ERROR("Failed to set panel brightness\n");

	/*wait for all FIFOs empty*/
	mdfld_dsi_wait_for_fifos_empty(sender);

power_on_err:
	if (err && reset_count) {
		if (dev_priv->bhdmiconnected) {
			/*if hdmi connected, turn off display A will
			* influnce HDMI, so only turn off MIPI island
			*/
			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_DOWN, OSPM_REG_TYPE);

			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_UP, OSPM_REG_TYPE);
		} else {
			ospm_power_island_down(OSPM_DISPLAY_A_ISLAND);
			ospm_power_island_up(OSPM_DISPLAY_A_ISLAND);
		}
		DRM_ERROR("Failed to init panel, try  reset it again!\n");
		goto reset_recovery;
	}
	mdfld_dsi_dsr_allow_locked(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}
/**
 * Power off sequence for DBI interface
*/
int __dbi_power_off(struct mdfld_dsi_config *dsi_config)
{
	u32 val = 0;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	int pipe0_enabled;
	int pipe2_enabled;
	int err = 0;
	int i;

	if (!dsi_config)
		return -EINVAL;

	PSB_DEBUG_ENTRY("\n");

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	/*save the plane informaton, for it will updated*/
	ctx->dspsurf	= REG_READ(regs->dspsurf_reg);
	ctx->dsplinoff	= REG_READ(regs->dsplinoff_reg);
	ctx->dspstride	= REG_READ(regs->dspstride_reg);
	ctx->dspsize	= REG_READ(regs->dspsize_reg);
	ctx->pipestat	= REG_READ(regs->pipestat_reg);
	ctx->dspcntr	= REG_READ(regs->dspcntr_reg);
	ctx->pipeconf	= REG_READ(regs->pipeconf_reg);

	/*save color_coef (chrome) */
	for (i = 0; i < 6; i++)
		ctx->color_coef[i] = REG_READ(regs->color_coef_reg + (i<<2));

	/* save palette (gamma) */
	for (i = 0; i < 256; i++)
		ctx->palette[i] = REG_READ(regs->palette_reg + (i<<2));

	/*Disable plane*/
	val = ctx->dspcntr;
	REG_WRITE(regs->dspcntr_reg, (val & ~BIT31));

	/*Disable pipe and overlay & cursor panel assigned to this pipe*/
	val = ctx->pipeconf;
	REG_WRITE(regs->pipeconf_reg, ((val | 0x000c0000) & ~BIT31));

	/*Disable DSI PLL*/
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled) {
		REG_WRITE(regs->dpll_reg , 0x0);
		/*power gate pll*/
		REG_WRITE(regs->dpll_reg, BIT30);
	}
	/*enter ulps*/
	if (__dbi_enter_ulps_locked(dsi_config)) {
		DRM_ERROR("Failed to enter ULPS\n");
		goto power_off_err;
	}

	REG_WRITE(regs->mipi_reg,
	      REG_READ(regs->mipi_reg) & (~PASS_FROM_SPHY_TO_AFE));
power_off_err:
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/**
 * Power off sequence for command mode MIPI panel.
 * NOTE: do NOT modify this function
 */
static int __dbi_panel_power_off(struct mdfld_dsi_config *dsi_config,
			struct panel_funcs *p_funcs)
{
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	int err = 0;

	if (!dsi_config)
		return -EINVAL;

	PSB_DEBUG_ENTRY("\n");

	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON))
		return -EAGAIN;

	mdfld_dsi_dsr_forbid_locked(dsi_config);

	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs && p_funcs->set_brightness)
		if (p_funcs->set_brightness(dsi_config, 0))
			DRM_ERROR("Failed to set panel brightness\n");

	/*wait for two TE, let pending PVR flip complete*/
	msleep(32);

	/*Disable TE, don't need it anymore*/
	mdfld_disable_te(dev, dsi_config->pipe);

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

	/*power off dbi interface*/
	__dbi_power_off(dsi_config);

power_off_err:
	mdfld_dsi_dsr_allow_locked(dsi_config);
	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	return err;
}

/* generic dbi function */
static
int mdfld_generic_dsi_dbi_set_power(struct drm_encoder *encoder, bool on)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_dbi_output *dbi_output;
	struct mdfld_dsi_connector *dsi_connector;
	struct mdfld_dsi_config *dsi_config;
	struct panel_funcs *p_funcs;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	static bool last_ospm_suspend;
	int pipe = 0;

	if (!encoder) {
		DRM_ERROR("Invalid encoder\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("%s\n", (on ? "on" : "off"));

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dbi_output = MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dsi_connector = mdfld_dsi_encoder_get_connector(dsi_encoder);
	if (!dsi_connector) {
		DRM_ERROR("Invalid connector\n");
		return -EINVAL;
	}
	p_funcs = dbi_output->p_funcs;
	dev = encoder->dev;
	dev_priv = dev->dev_private;
	pipe = dsi_config->pipe;

	/*Notify PVR module screen is off before power off*/
	if (!on && dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);

	mutex_lock(&dsi_config->context_lock);

	if (dsi_connector->status != connector_status_connected)
		goto set_power_err;

	if (dbi_output->first_boot &&
	    dsi_config->dsi_hw_context.panel_on) {
		if (Check_fw_initilized_reusable(dsi_config, p_funcs)) {
			DRM_INFO("skip panle power setting for first boot!"
					" panel is already powered on\n");
			if (on) {
				/* When using smooth transition, enable TE
				 * and wake up ESD detection thread.
				 */
				mdfld_enable_te(dev, pipe);
				mdfld_dsi_error_detector_wakeup(dsi_connector);
			}
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

		if (__dbi_panel_power_on(dsi_config, p_funcs)) {
			DRM_ERROR("Faild to turn on panel\n");
			goto set_power_err;
		}

		dsi_config->dsi_hw_context.panel_on = 1;
		dbi_output->dbi_panel_on = 1;
		mdfld_dsi_error_detector_wakeup(dsi_connector);
		last_ospm_suspend = false;

		break;
	case false:
		if (!dsi_config->dsi_hw_context.panel_on &&
		    !dbi_output->first_boot)
			goto fun_exit;

		if (__dbi_panel_power_off(dsi_config, p_funcs)) {
			DRM_ERROR("Faild to turn off panel\n");
			goto set_power_err;
		}

		if (!dev_priv->dpms_on_off)
			last_ospm_suspend = true;
		else
			last_ospm_suspend = false;

		dsi_config->dsi_hw_context.panel_on = 0;
		dbi_output->dbi_panel_on = 0;
		break;
	default:
		break;
	}

fun_exit:
	mutex_unlock(&dsi_config->context_lock);
	if (dsi_config->dsi_hw_context.panel_on) {
		/*Notify PVR module screen is on if success power on*/
		if (dev_priv->pvr_screen_event_handler)
			dev_priv->pvr_screen_event_handler(dev, 1);
	}
	PSB_DEBUG_ENTRY("successfully\n");
	return 0;

set_power_err:
	mutex_unlock(&dsi_config->context_lock);
	/* if err occurs, modify the state back to its previous state */
	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, on ? 0 : 1);
	PSB_DEBUG_ENTRY("unsuccessfully!\n");
	return -EAGAIN;
}

static
void mdfld_generic_dsi_dbi_mode_set(struct drm_encoder *encoder,
		struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	return;
}

static
void mdfld_generic_dsi_dbi_prepare(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);

	PSB_DEBUG_ENTRY("\n");

	dbi_output->mode_flags |= MODE_SETTING_IN_ENCODER;
	dbi_output->mode_flags &= ~MODE_SETTING_ENCODER_DONE;

	mdfld_generic_dsi_dbi_set_power(encoder, false);
	gbdispstatus = false;
}

static
void mdfld_generic_dsi_dbi_commit(struct drm_encoder *encoder)
{
	struct mdfld_dsi_encoder *dsi_encoder =
		MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_dbi_output *dbi_output =
		MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	struct drm_device *dev = dbi_output->dev;
	struct drm_psb_private *dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("\n");

	mdfld_generic_dsi_dbi_set_power(encoder, true);
	gbdispstatus = true;

	dbi_output->mode_flags &= ~MODE_SETTING_IN_ENCODER;
	if (dbi_output->channel_num == 1)
		dev_priv->dsr_fb_update |= MDFLD_DSR_2D_3D_2;
	else
		dev_priv->dsr_fb_update |= MDFLD_DSR_2D_3D_0;
	dbi_output->mode_flags |= MODE_SETTING_ENCODER_DONE;

	dbi_output->first_boot = false;
}

static
void mdfld_generic_dsi_dbi_dpms(struct drm_encoder *encoder, int mode)
{
	struct mdfld_dsi_encoder *dsi_encoder;
	struct mdfld_dsi_dbi_output *dbi_output;
	struct drm_device *dev;
	struct mdfld_dsi_config *dsi_config;
	struct drm_psb_private *dev_priv;

	if (!(drm_psb_use_cases_control & PSB_DPMS_ENABLE))
		return;

	dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	dsi_config = mdfld_dsi_encoder_get_config(dsi_encoder);
	dbi_output = MDFLD_DSI_DBI_OUTPUT(dsi_encoder);
	dev = dsi_config->dev;
	dev_priv = dev->dev_private;

	PSB_DEBUG_ENTRY("%s\n", (mode == DRM_MODE_DPMS_ON ? "on" : "off"));

	if (!gbdispstatus) {
		DRM_INFO("panel in suspend status, " \
			 "skip turn on/off from DMPS");
		return;
	}

	mutex_lock(&dev_priv->dpms_mutex);
	dev_priv->dpms_on_off = true;

	if (mode == DRM_MODE_DPMS_ON)
		mdfld_generic_dsi_dbi_set_power(encoder, true);
	else
		mdfld_generic_dsi_dbi_set_power(encoder, false);

	dev_priv->dpms_on_off = false;
	mutex_unlock(&dev_priv->dpms_mutex);
}

static
void mdfld_generic_dsi_dbi_save(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	mdfld_generic_dsi_dbi_set_power(encoder, false);
}

static
void mdfld_generic_dsi_dbi_restore(struct drm_encoder *encoder)
{
	if (!encoder)
		return;

	mdfld_generic_dsi_dbi_set_power(encoder, true);
}

static
bool mdfld_generic_dsi_dbi_mode_fixup(struct drm_encoder *encoder,
		const struct drm_display_mode *mode,
		struct drm_display_mode *adjusted_mode)
{
	struct mdfld_dsi_encoder *dsi_encoder = MDFLD_DSI_ENCODER(encoder);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_display_mode *fixed_mode = dsi_config->fixed_mode;

	PSB_DEBUG_ENTRY("\n");

	if (fixed_mode) {
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
struct drm_encoder_funcs dsi_dbi_generic_encoder_funcs = {
	.destroy = drm_encoder_cleanup,
};

static
struct drm_encoder_helper_funcs dsi_dbi_generic_encoder_helper_funcs = {
	.save = mdfld_generic_dsi_dbi_save,
	.restore = mdfld_generic_dsi_dbi_restore,
	.dpms = mdfld_generic_dsi_dbi_dpms,
	.mode_fixup = mdfld_generic_dsi_dbi_mode_fixup,
	.prepare = mdfld_generic_dsi_dbi_prepare,
	.mode_set = mdfld_generic_dsi_dbi_mode_set,
	.commit = mdfld_generic_dsi_dbi_commit,
};

/*
 * Init DSI DBI encoder.
 * Allocate an mdfld_dsi_encoder and attach it to given @dsi_connector
 * return pointer of newly allocated DBI encoder, NULL on error
 */
struct mdfld_dsi_encoder *mdfld_dsi_dbi_init(struct drm_device *dev,
		struct mdfld_dsi_connector *dsi_connector)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct mdfld_dsi_config *dsi_config;
	struct drm_connector *connector = NULL;
	struct drm_encoder *encoder = NULL;
	struct psb_gtt *pg = dev_priv ? (dev_priv->pg) : NULL;

#ifdef CONFIG_MDFLD_DSI_DPU
	struct mdfld_dbi_dpu_info *dpu_info =
		dev_priv ? (dev_priv->dbi_dpu_info) : NULL;
#else
	struct mdfld_dbi_dsr_info *dsr_info =
		dev_priv ? (dev_priv->dbi_dsr_info) : NULL;
#endif
	int pipe;

	PSB_DEBUG_ENTRY("\n");

	if (!pg || !dsi_connector) {
		DRM_ERROR("Invalid parameters\n");
		return NULL;
	}

	dsi_config = mdfld_dsi_get_config(dsi_connector);
	pipe = dsi_connector->pipe;
	dsi_connector->status = connector_status_connected;

	/* TODO: get panel info from DDB */
	dbi_output = kzalloc(sizeof(struct mdfld_dsi_dbi_output), GFP_KERNEL);
	if (unlikely(!dbi_output)) {
		DRM_ERROR("No memory\n");
		return NULL;
	}

	if (dsi_connector->pipe == 0) {
		dbi_output->channel_num = 0;
		dev_priv->dbi_output = dbi_output;
	} else if (dsi_connector->pipe == 2) {
		dbi_output->channel_num = 1;
		dev_priv->dbi_output2 = dbi_output;
	} else {
		DRM_ERROR("only support 2 DSI outputs\n");
		goto out_err1;
	}

	dbi_output->dev = dev;
	dbi_output->p_funcs = NULL;

	/*create drm encoder object*/
	connector = &dsi_connector->base.base;
	encoder = &dbi_output->base.base;
	drm_encoder_init(dev,
			encoder,
			&dsi_dbi_generic_encoder_funcs,
			DRM_MODE_ENCODER_DSI);
	drm_encoder_helper_add(encoder,
			&dsi_dbi_generic_encoder_helper_funcs);

	/*attach to given connector*/
	drm_mode_connector_attach_encoder(connector, encoder);
	connector->encoder = encoder;

	/*set possible crtcs and clones*/
	if (dsi_connector->pipe) {
		encoder->possible_crtcs = (1 << 2);
		encoder->possible_clones = (1 << 1);
	} else {
		encoder->possible_crtcs = (1 << 0);
		encoder->possible_clones = (1 << 0);
	}

	dev_priv->dsr_fb_update = 0;
	dev_priv->b_dsr_enable = false;
	dev_priv->b_async_flip_enable = false;
	dev_priv->exit_idle = mdfld_dsi_dbi_exit_dsr;
	dev_priv->async_flip_update_fb = mdfld_dsi_dbi_async_flip_fb_update;
	dev_priv->async_check_fifo_empty = mdfld_dsi_dbi_async_check_fifo_empty;

#if defined(CONFIG_MDFLD_DSI_DPU) || defined(CONFIG_MDFLD_DSI_DSR)
	dev_priv->b_dsr_enable_config = true;
#endif /*CONFIG_MDFLD_DSI_DSR*/

	dbi_output->first_boot = true;
	dbi_output->mode_flags = MODE_SETTING_IN_ENCODER;

#ifdef CONFIG_MDFLD_DSI_DPU
	/*add this output to dpu_info*/
	if (dsi_connector->status == connector_status_connected) {
		if (dsi_connector->pipe == 0)
			dpu_info->dbi_outputs[0] = dbi_output;
		else
			dpu_info->dbi_outputs[1] = dbi_output;

		dpu_info->dbi_output_num++;
	}

#else /*CONFIG_MDFLD_DSI_DPU*/
	if (dsi_connector->status == connector_status_connected) {
		/*add this output to dsr_info*/
		if (dsi_connector->pipe == 0)
			dsr_info->dbi_outputs[0] = dbi_output;
		else
			dsr_info->dbi_outputs[1] = dbi_output;

		dsr_info->dbi_output_num++;
	}
#endif

	PSB_DEBUG_ENTRY("successfully\n");

	return &dbi_output->base;

out_err1:
	kfree(dbi_output);

	return NULL;
}


void mdfld_reset_panel_handler_work(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
		container_of(work, struct drm_psb_private, reset_panel_work);
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_dbi_output *dbi_output = NULL;
	struct panel_funcs *p_funcs  = NULL;
	struct drm_device *dev = NULL;
	struct mdfld_dsi_connector *dsi_connector;

	dbi_output = dev_priv->dbi_output;
	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config || !dbi_output)
		return;

	dev = dsi_config->dev;
	dsi_connector = mdfld_dsi_encoder_get_connector(dsi_config->encoder);

	/*disable ESD when HDMI connected*/
	if (hdmi_state)
		return;

	PSB_DEBUG_ENTRY("\n");

	p_funcs = dbi_output->p_funcs;
	if (p_funcs) {
		mutex_lock(&dsi_config->context_lock);

		if (dev_priv->pvr_screen_event_handler)
			dev_priv->pvr_screen_event_handler(dev, 0);

		if (__dbi_panel_power_off(dsi_config, p_funcs)) {
			mutex_unlock(&dsi_config->context_lock);
			return;
		}

		acquire_ospm_lock();
		ospm_power_island_down(OSPM_DISPLAY_ISLAND);
		ospm_power_island_up(OSPM_DISPLAY_ISLAND);
		release_ospm_lock();

		if (__dbi_panel_power_on(dsi_config, p_funcs)) {
			mutex_unlock(&dsi_config->context_lock);
			return;
		}

		if (dev_priv->pvr_screen_event_handler)
			dev_priv->pvr_screen_event_handler(dev, 1);

		mutex_unlock(&dsi_config->context_lock);
		DRM_INFO("%s: End panel reset\n", __func__);
	} else {
		DRM_INFO("%s invalid panel init\n", __func__);
	}
}
