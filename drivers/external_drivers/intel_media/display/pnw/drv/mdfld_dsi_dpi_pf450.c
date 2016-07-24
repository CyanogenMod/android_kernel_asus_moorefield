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
#include "mdfld_dsi_esd.h"

static
u16 mdfld_dsi_dpi_to_byte_clock_count(int pixel_clock_count,
		int num_lane, int bpp)
{
	return (u16)((pixel_clock_count*bpp)/(num_lane*8));
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

	if (!mode || !dpi_timing) {
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
static int intel_dc_dsi_check_latch_out(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = NULL;
	struct drm_device *dev = NULL;
	int retry = 10, ret = -1;

	if (!dsi_config)
		goto exit;

	dev = dsi_config->dev;
	regs = &dsi_config->regs;

	if (!dev)
		goto exit;

	while (retry) {
		if ((REG_READ(regs->mipi_reg) & BIT17)) {
			DRM_DEBUG("%s: reg 61190 = 0x%08x - lp11 in %d retries\n",
					__func__, REG_READ(regs->mipi_reg), 10-retry);
		} else {
			DRM_DEBUG("%s: reg 61190 = 0x%08x - lp00 in %d retries\n",
					__func__, REG_READ(regs->mipi_reg), 10-retry);
		}

		udelay(10);
		retry--;
	}

exit:
	return ret;
}

static int intel_dc_dsi_wait_for_lp00(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = NULL;
	struct drm_device *dev = NULL;
	int retry = 1000, ret = -1;

	if (!dsi_config)
		goto exit;

	dev = dsi_config->dev;
	regs = &dsi_config->regs;

	if (!dev)
		goto exit;

	while (retry) {
		if ((REG_READ(regs->mipi_reg) & BIT17)) {
			udelay(10);
			retry--;
		} else {
			ret = 0;
			break;
		}
	}

	DRM_INFO("%s: retry %d times\n", __func__, 1000 - retry);

exit:
	return ret;
}
static int __dpi_exit_ulps_locked(struct mdfld_dsi_config *dsi_config);

static int __dpi_enter_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);

	u32 mipi_val = 0;
	int retry = 5;

ulps_recovery:

	PSB_DEBUG_ENTRY("\n");

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	if (ctx->device_ready & DSI_POWER_STATE_ULPS_MASK) {
		DRM_ERROR("Broken ULPS states\n");
		return -EINVAL;
	}

	if (!sender) {
		DRM_ERROR("Cannot get sender\n");
		return -EINVAL;
	}

	/* check latch out, expected to be lp11 */
	intel_dc_dsi_check_latch_out(dsi_config);

	/*wait for all FIFOs empty*/
	mdfld_dsi_wait_for_fifos_empty(sender);

	/*inform DSI host is to be put on ULPS*/
	ctx->device_ready |= (DSI_POWER_STATE_ULPS_ENTER | DSI_DEVICE_READY);
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);
	mdelay(1);

	/* check latch out, expected to be lp00 */
	intel_dc_dsi_check_latch_out(dsi_config);

	if (intel_dc_dsi_wait_for_lp00(dsi_config)) {
		DRM_ERROR("faild to wait for lp00 status\n");
		DRM_ERROR("reg[0x%x] = 0x%x\n", regs->mipi_reg, REG_READ(regs->mipi_reg));

		if (retry--) {
		    DRM_INFO("%s: retry enter ulps (times=%d)\n", __func__, 5-retry);
		    if (__dpi_exit_ulps_locked(dsi_config)) {
			 DRM_ERROR("Failed to exit ULPS\n");
		    }
		    goto ulps_recovery;
		}
	}

	/* set AFE hold value */
	mipi_val = REG_READ(regs->mipi_reg);
	REG_WRITE(regs->mipi_reg, (mipi_val & (~PASS_FROM_SPHY_TO_AFE)));

	return 0;
}

static int __dpi_exit_ulps_locked(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	struct mdfld_dsi_hw_context *ctx = &dsi_config->dsi_hw_context;
	struct drm_device *dev = dsi_config->dev;

	ctx->device_ready = REG_READ(regs->device_ready_reg);

	/*inform DSI host is to be put on ULPS*/
	ctx->device_ready = (DSI_POWER_STATE_ULPS_ENTER | DSI_DEVICE_READY);
	REG_WRITE(regs->device_ready_reg, ctx->device_ready);

	mdelay(1);
	/* clear AFE hold value*/
	REG_WRITE(regs->mipi_reg, PASS_FROM_SPHY_TO_AFE);
	mdelay(1);

	/*enter ULPS EXIT state*/
	ctx->device_ready &= ~DSI_POWER_STATE_ULPS_MASK;
	ctx->device_ready |= (DSI_POWER_STATE_ULPS_EXIT | DSI_DEVICE_READY);
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

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
static void dds_dump_hw_context(struct mdfld_dsi_hw_context *ctx)
{
	printk("ctx->vgacntr = %8x\n", ctx->vgacntr);

	/*plane*/
	printk("ctx->dspcntr = %8x\n", ctx->dspcntr);
	printk("ctx->dspsize = %8x\n", ctx->dspsize);
	printk("ctx->dspsurf = %8x\n", ctx->dspsurf);
	printk("ctx->dsppos = %8x\n", ctx->dsppos);
	printk("ctx->dspstride = %8x\n", ctx->dspstride);
	printk("ctx->dsplinoff = %8x\n", ctx->dsplinoff);

	/*overlay*/
	printk("ctx->ovaadd = %8x\n", ctx->ovaadd);
	printk("ctx->ovcadd = %8x\n", ctx->ovcadd);

	/* gamma and csc */
	/*printk("ctx->vgacntr = %8x\n", ctx->palette[256];
	printk("ctx->vgacntr = %8x\n", ctx->color_coef[6];*/

	/*pipe regs*/
	printk("ctx->htotal = %8x\n", ctx->htotal);
	printk("ctx->hblank = %8x\n", ctx->hblank);
	printk("ctx->hsync = %8x\n", ctx->hsync);
	printk("ctx->vtotal = %8x\n", ctx->vtotal);
	printk("ctx->vblank = %8x\n", ctx->vblank);
	printk("ctx->vsync = %8x\n", ctx->vsync);
	printk("ctx->pipestat = %8x\n", ctx->pipestat);

	printk("ctx->pipesrc = %8x\n", ctx->pipesrc);

	printk("ctx->dpll = %8x\n", ctx->dpll);
	printk("ctx->fp = %8x\n", ctx->fp);
	printk("ctx->pipeconf = %8x\n", ctx->pipeconf);

	/*mipi port*/
	printk("ctx->mipi = %8x\n", ctx->mipi);

	/*DSI controller regs*/
	printk("ctx->device_ready = %8x\n", ctx->device_ready);
	printk("ctx->intr_stat = %8x\n", ctx->intr_stat);
	printk("ctx->intr_en = %8x\n", ctx->intr_en);
	printk("ctx->dsi_func_prg = %8x\n", ctx->dsi_func_prg);
	printk("ctx->hs_tx_timeout = %8x\n", ctx->hs_tx_timeout);
	printk("ctx->lp_rx_timeout = %8x\n", ctx->lp_rx_timeout);
	printk("ctx->turn_around_timeout = %8x\n", ctx->turn_around_timeout);
	printk("ctx->device_reset_timer = %8x\n", ctx->device_reset_timer);
	printk("ctx->dpi_resolution = %8x\n", ctx->dpi_resolution);
	printk("ctx->dbi_fifo_throttle = %8x\n", ctx->dbi_fifo_throttle);
	printk("ctx->hsync_count = %8x\n", ctx->hsync_count);
	printk("ctx->hbp_count = %8x\n", ctx->hbp_count);
	printk("ctx->hfp_count = %8x\n", ctx->hfp_count);
	printk("ctx->hactive_count = %8x\n", ctx->hactive_count);
	printk("ctx->vsync_count = %8x\n", ctx->vsync_count);
	printk("ctx->vbp_count = %8x\n", ctx->vbp_count);
	printk("ctx->vfp_count = %8x\n", ctx->vfp_count);
	printk("ctx->high_low_switch_count = %8x\n", ctx->high_low_switch_count);
	printk("ctx->dpi_control = %8x\n", ctx->dpi_control);
	printk("ctx->dpi_data = %8x\n", ctx->dpi_data);
	printk("ctx->init_count = %8x\n", ctx->init_count);
	printk("ctx->max_return_pack_size = %8x\n", ctx->max_return_pack_size);
	printk("ctx->video_mode_format = %8x\n", ctx->video_mode_format);
	printk("ctx->eot_disable = %8x\n", ctx->eot_disable);
	printk("ctx->lp_byteclk = %8x\n", ctx->lp_byteclk);
	printk("ctx->lp_gen_data = %8x\n", ctx->lp_gen_data);
	printk("ctx->hs_gen_data = %8x\n", ctx->hs_gen_data);
	printk("ctx->lp_gen_ctrl = %8x\n", ctx->lp_gen_ctrl);
	printk("ctx->hs_gen_ctrl = %8x\n", ctx->hs_gen_ctrl);
	printk("ctx->gen_fifo_stat = %8x\n", ctx->gen_fifo_stat);
	printk("ctx->hs_ls_dbi_enable = %8x\n", ctx->hs_ls_dbi_enable);
	printk("ctx->dphy_param = %8x\n", ctx->dphy_param);
	printk("ctx->dbi_bw_ctrl = %8x\n", ctx->dbi_bw_ctrl);
	printk("ctx->clk_lane_switch_time_cnt = %8x\n", ctx->clk_lane_switch_time_cnt);

	/*MIPI adapter regs*/
	printk("ctx->mipi_control = %8x\n", ctx->mipi_control);
	printk("ctx->mipi_data_addr = %8x\n", ctx->mipi_data_addr);
	printk("ctx->mipi_data_len = %8x\n", ctx->mipi_data_len);
	printk("ctx->mipi_cmd_addr = %8x\n", ctx->mipi_cmd_addr);
	printk("ctx->mipi_cmd_len = %8x\n", ctx->mipi_cmd_len);

	/*panel status*/
	printk("ctx->panel_on = %8x\n", ctx->panel_on);
	printk("ctx->backlight_level = %8x\n", ctx->backlight_level);

	printk("ctx->pll_bypass_mode = %8x\n", ctx->pll_bypass_mode);
	printk("ctx->cck_div = %8x\n", ctx->cck_div);
	/*brightness*/
	printk("ctx->lastbrightnesslevel = %8x\n", ctx->lastbrightnesslevel);

	/*dpst register values*/
	printk("ctx->histogram_intr_ctrl = %8x\n", ctx->histogram_intr_ctrl);
	printk("ctx->histogram_logic_ctrl = %8x\n", ctx->histogram_logic_ctrl);
	printk("ctx->aimg_enhance_bin = %8x\n", ctx->aimg_enhance_bin);
	printk("ctx->lvds_port_ctrl = %8x\n", ctx->lvds_port_ctrl);

}
#if defined(CONFIG_EEPROM_PADSTATION)
extern void reportPadStationI2CFail(char *devname);
#endif
extern int asus_panel_id;/*austin+++*/
#endif
/*ASUS_BSP: [DDS] ---*/

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

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH

	/*dds_dump_hw_context(ctx);*/

	if (p_funcs->dsi_controller_init)
		p_funcs->dsi_controller_init(dsi_config);

	if (p_funcs->get_config_mode){
		kfree(dsi_config->fixed_mode);
		dsi_config->fixed_mode = p_funcs->get_config_mode();
	}

	printk("[DISPLAY][DDS] %s: panel_id = %d, panel_turn_on = %d\n", __func__, panel_id, panel_turn_on);

	if (panel_id == 0) {
		/*480x854 austin+++*/
		if (asus_panel_id == OFILM_PANEL) {
			ctx->dpll = 0x400000;
			ctx->fp = 0x1ca;
			ctx->htotal = 0x25f01df;
			ctx->hblank = 0x25f01df;
			ctx->hsync = 0x223021f;
			ctx->vtotal = 0x3970355;
			ctx->vblank = 0x3970355;
			ctx->vsync = 0x3790375;

			/*pipe source*/
			ctx->pipesrc = 0x1df0355;

			/*__mdfld_dsi_dpi_set_timing()*/
			ctx->dpi_resolution =  0x35601e0;

			ctx->hsync_count = 0x6;
			ctx->hbp_count = 0x5a;
			ctx->hfp_count = 0x60;
			ctx->hactive_count = 0x2d0;
			ctx->vsync_count = 0x6;
			ctx->vbp_count = 0x2d;
			ctx->vfp_count = 0x30;

			ctx->pipeconf = 0xf91c0000;

			ctx->dspsize =  0x35501df;
			ctx->dspstride = 0xc80;     /*ystride: 480*4, droidboot have to use pad ystride:800*4*/

			ctx->cck_div = 1;
			ctx->pll_bypass_mode = 0;
		} else if (asus_panel_id == WINTEK_PANEL_NT35510S) {
			DRM_INFO("[DISPLAY] %s: asus_panel_id = WINTEK_PANEL_NT35510S\n", __func__);
			ctx->dpll = 0x200000 ;
			ctx->fp = 0x93 ;
			ctx->htotal = 0x24d01df;
			ctx->hblank = 0x24d01df;
			ctx->hsync = 0x21b0211;
			ctx->vtotal = 0x3c30355;
			ctx->vblank = 0x3c30355;
			ctx->vsync = 0x3910387;

			/*pipe source*/
			ctx->pipesrc = 0x1df0355;

			/*__mdfld_dsi_dpi_set_timing()*/
			ctx->dpi_resolution =  0x35601e0;

			ctx->hsync_count = 0xf;
			ctx->hbp_count = 0x4b;
			ctx->hfp_count = 0x4b;
			ctx->hactive_count = 0x2d0;
			ctx->vsync_count = 0xf;
			ctx->vbp_count = 0x4b;
			ctx->vfp_count = 0x4b;

			ctx->pipeconf = 0xf81c0000;

			ctx->dspsize =  0x35501df;
			ctx->dspstride = 0xc80;     /*ystride: 480*4, droidboot have to use pad ystride:800*4*/

			ctx->cck_div = 1;
			ctx->pll_bypass_mode = 0;
		} else {
			ctx->dpll = 0x400000 ;
			ctx->fp = 0x75 ;
			ctx->htotal = 0x26901df;
			ctx->hblank = 0x26901df;
			ctx->hsync = 0x22d021b;
			ctx->vtotal = 0x3c70355;
			ctx->vblank = 0x3c70355;
			ctx->vsync = 0x3950387;

			/*pipe source*/
			ctx->pipesrc = 0x1df0355;

			/*__mdfld_dsi_dpi_set_timing()*/
			ctx->dpi_resolution =  0x35601e0;

			ctx->hsync_count = 0x1b;
			ctx->hbp_count = 0x5a;
			ctx->hfp_count = 0x5a;
			ctx->hactive_count = 0x2d0;
			ctx->vsync_count = 0x15;
			ctx->vbp_count = 0x4b;
			ctx->vfp_count = 0x4b;

			ctx->pipeconf = 0xf81c0000;

			ctx->dspsize =  0x35501df;
			ctx->dspstride = 0xc80;     /*ystride: 480*4, droidboot have to use pad ystride:800*4*/

			ctx->cck_div = 1;
			ctx->pll_bypass_mode = 0;
		}
		/*austin---*/
	} else {
		/*800x1280*/
		ctx->dpll = 0x40000 ;
		ctx->fp = 0xd2 ;
		ctx->htotal = 0x369031f;
		ctx->hblank = 0x369031f;
		ctx->hsync = 0x3460342;
		ctx->vtotal = 0x52304ff;
		ctx->vblank = 0x52304ff;
		ctx->vsync = 0x51b0517;

		/*pipe source*/
		ctx->pipesrc = 0x31f04ff;

		/*__mdfld_dsi_dpi_set_timing()*/
		ctx->dpi_resolution =  0x5000320;

		ctx->hsync_count = 0x6;
		ctx->hbp_count = 0x34;
		ctx->hfp_count = 0x34;
		ctx->hactive_count = 0x4b0;
		ctx->vsync_count = 0x6;
		ctx->vbp_count = 0x24;
		ctx->vfp_count = 0x24;

		ctx->pipeconf = 0xf81c0000;

		ctx->dspsize =  0x4ff031f;
		ctx->dspstride = 0xc80;

		ctx->cck_div = 0;
		ctx->pll_bypass_mode = 0;
	}

	/*dds_dump_hw_context(ctx);*/

#endif
/*ASUS_BSP: [DDS] ---*/

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
		if (ctx->pll_bypass_mode) {
			uint32_t dpll = 0;

			REG_WRITE(regs->dpll_reg, dpll);
			if	(ctx->cck_div) {
				dpll = dpll | BIT11;
				REG_WRITE(regs->dpll_reg, dpll);
				udelay(1);
			}
			dpll = dpll | BIT12;
			REG_WRITE(regs->dpll_reg, dpll);
			mdelay(1);
			dpll = dpll | BIT13;
			REG_WRITE(regs->dpll_reg, dpll);
			mdelay(1);
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

	REG_WRITE(regs->eot_disable_reg,
			(REG_READ(regs->eot_disable_reg) & ~BIT1));
	REG_WRITE(regs->device_ready_reg, ~BIT0);
	REG_WRITE(regs->device_ready_reg, BIT0);
	mdelay(1);

	/*exit ULPS*/
	if (__dpi_exit_ulps_locked(dsi_config)) {
		DRM_ERROR("Failed to exit ULPS\n");
		goto power_on_err;
	}

	/*update MIPI port config*/
	REG_WRITE(regs->mipi_reg, (ctx->mipi | REG_READ(regs->mipi_reg)));

	/*unready dsi adapter for re-programming*/
	REG_WRITE(regs->device_ready_reg,
			REG_READ(regs->device_ready_reg) & ~(DSI_DEVICE_READY));

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
	REG_WRITE(regs->dsi_func_prg_reg, ctx->dsi_func_prg);
	if	(ctx->pll_bypass_mode)	{
		/*Force using non-burst pulse event mode here*/
		REG_WRITE(regs->video_mode_format_reg, 0x1);
	} else{
		REG_WRITE(regs->video_mode_format_reg, ctx->video_mode_format);
	}

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

	/*Enable DSI Controller*/
	REG_WRITE(regs->device_ready_reg, (REG_READ(regs->device_ready_reg) | BIT0));

	/**
	 * Different panel may have different ways to have
	 * drvIC initialized. Support it!
	 */
	if (p_funcs && p_funcs->drv_ic_init) {
		if (p_funcs->drv_ic_init(dsi_config)) {
			if (!reset_count) {
/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
				if (panel_id == DDS_PAD) {
					#if defined(CONFIG_EEPROM_PADSTATION)
					reportPadStationI2CFail("DISPLAY");
					#endif
					DRM_INFO("[DISPLAY] [DDS] %s: Initial panel_id %d failed, do reportPadStationI2CFail().\n", __func__, panel_id);

				}
#endif
/*ASUS_BSP: [DDS] ---*/
				goto reset_err_bypass;
			}
			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_DOWN, OSPM_REG_TYPE);

			pmu_nc_set_power_state(OSPM_MIPI_ISLAND,
					OSPM_ISLAND_UP, OSPM_REG_TYPE);

			DRM_ERROR("Failed to init dsi controller, reset it!\n");
			goto reset_recovery;
		}
	}

reset_err_bypass:
	if (ctx->pll_bypass_mode) {
		/*Reprogram to use pll clock*/
		REG_WRITE(regs->dpll_reg, 0x0);
		REG_WRITE(regs->fp_reg, 0x0);
		REG_WRITE(regs->fp_reg, ctx->fp);
		REG_WRITE(regs->dpll_reg, ((ctx->dpll) & ~BIT30));

		udelay(2);
		REG_WRITE(regs->dpll_reg, 0x2 << 17);
		mdelay(1);
		REG_WRITE(regs->dpll_reg, (0x2 << 17 | BIT31));

		/*wait for PLL lock on pipe*/
		retry = 10000;
		while (--retry && !(REG_READ(PIPEACONF) & BIT29))
			udelay(3);
		if (!retry) {
			DRM_ERROR("PLL failed to lock on pipe\n");
			err = -EAGAIN;
			goto power_on_err;
		}

		/*Switch back to burst mode*/
		REG_WRITE(regs->video_mode_format_reg, ctx->video_mode_format);

		/*Clear device ready reg*/
		REG_WRITE(regs->device_ready_reg, REG_READ(regs->device_ready_reg) & ~DSI_DEVICE_READY);
		msleep(1);
		/*Enable DSI Controller*/
		REG_WRITE(regs->device_ready_reg, REG_READ(regs->device_ready_reg) | DSI_DEVICE_READY);
		msleep(1);
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

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	printk("[DISPLAY][DDS] %s: panel_id = %d, panel_turn_on = %d\n", __func__, panel_id, panel_turn_on);
#endif
/*ASUS_BSP: [DDS] ---*/

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
	ctx->dspstride = REG_READ(regs->dspstride_reg);

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

	/* check latch out, expected to be lp00 */
	intel_dc_dsi_check_latch_out(dsi_config);

	/*Set clock stopping*/
	REG_WRITE(regs->eot_disable_reg, (REG_READ(regs->eot_disable_reg) | BIT1));

	/* clear device ready and reset device ready
	 * to make clock stopping setting take effects.
	 */
	REG_WRITE(regs->device_ready_reg, (REG_READ(regs->device_ready_reg) & ~BIT0));
	REG_WRITE(regs->device_ready_reg, (REG_READ(regs->device_ready_reg) | BIT0));

	/* check latch out, expected to be lp11 */
	intel_dc_dsi_check_latch_out(dsi_config);

	if (__dpi_enter_ulps_locked(dsi_config)) {
		DRM_ERROR("Faild to enter ULPS\n");
		goto power_off_err;
	}

       /*workaround for MIPI D0~D3 power leakage*/
       __dpi_exit_ulps_locked(dsi_config);
       __dpi_enter_ulps_locked(dsi_config);

	/*Disable DSI PLL*/
	pipe0_enabled = (REG_READ(PIPEACONF) & BIT31) ? 1 : 0;
	pipe2_enabled = (REG_READ(PIPECCONF) & BIT31) ? 1 : 0;

	if (!pipe0_enabled && !pipe2_enabled) {
		REG_WRITE(regs->dpll_reg , 0x0);
		/*power gate pll*/
		REG_WRITE(regs->dpll_reg, BIT30);
	}

/*ASUS_BSP: [DDS] +++*/
#if defined(CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY)
	if (p_funcs && p_funcs->gpio_power_off) {
		if (p_funcs->gpio_power_off(dsi_config)) {
			DRM_ERROR("Failed to power off panel\n");
			err = -EAGAIN;
			goto power_off_err;
		}
	}
#endif
/*ASUS_BSP: [DDS] ---*/

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

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	printk("[DISPLAY][DDS] %s: panel_id = %d, panel_turn_on = %d\n", __func__, panel_id, panel_turn_on);
#endif

#ifndef CONFIG_SUPPORT_P73L_TWO_LANE
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
			MDFLD_DSI_DPI_SPK_TURN_ON);
#else
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_TURN_ON);
#endif
#endif
/*ASUS_BSP: [DDS] ---*/

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

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	printk("[DISPLAY][DDS] %s: panel_id = %d, panel_turn_on = %d\n", __func__, panel_id, panel_turn_on);
#endif
/*ASUS_BSP: [DDS] ---*/

	ctx = &dsi_config->dsi_hw_context;
	ctx->lastbrightnesslevel = psb_brightness;
	if (p_funcs->set_brightness(dsi_config, 0))
		DRM_ERROR("Failed to set panel brightness\n");

/*ASUS_BSP: [DDS] +++*/
#ifndef CONFIG_SUPPORT_P73L_TWO_LANE
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	err = mdfld_dsi_send_dpi_spk_pkg_lp(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
#else
	err = mdfld_dsi_send_dpi_spk_pkg_hs(sender,
			MDFLD_DSI_DPI_SPK_SHUT_DOWN);
#endif
#endif
/*ASUS_BSP: [DDS] ---*/

	/*According HW DSI spec, here need wait for 100ms*/
	/*To optimize dpms flow, move sleep out of mutex*/
	/* msleep(100); */

	return err;
}

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
void mdfld_reset_dpi_panel(struct drm_psb_private *dev_priv, int connected)
{
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	struct mdfld_dsi_dpi_output *dpi_output = dev_priv->dpi_output;
	struct panel_funcs *p_funcs;
	struct drm_device *dev;
	/* ASUS_BSP: Louis +++	*/
	struct mdfld_dsi_hw_context *ctx;
	struct mdfld_dsi_hw_registers *regs;
	/* ASUS_BSP: Louis ---	*/

	if (!dsi_config || !dpi_output) {
		if (!dsi_config)
			DRM_INFO("%s: dsi_config NULL\n", __func__);
		else if (!dpi_output)
			DRM_INFO("%s: dpi_output NULL\n", __func__);
		return;
	}

	dev = dsi_config->dev;
	/* ASUS_BSP: Louis +++	*/
	regs = &dsi_config->regs;
	ctx = &dsi_config->dsi_hw_context;
	/* ASUS_BSP: Louis ---	*/

	/*disable ESD when HDMI connected*/
	if (hdmi_state) {
		DRM_INFO("%s: hdmi_state %d\n", __func__, hdmi_state);
		return;
	}

	DRM_INFO("[DISPLAY] [DDS] %s: panel_turn_on = %d, dsi_hw_context.panel_on =%d.\n",
							__func__, panel_turn_on, dsi_config->dsi_hw_context.panel_on);

	mutex_lock(&dsi_config->context_lock);

	if (panel_turn_on == DDS_NONE) {
		panel_id = connected;
		DRM_INFO("[DISPLAY] [DDS] %s: change panel_id =%d , skip reset.\n", __func__, panel_id);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	if (panel_turn_on == connected) {
		DRM_INFO("[DISPLAY] [DDS] %s: panel_id = %d ,panel_turn_on = %d, but do not skip reset.\n", __func__, panel_id, panel_turn_on);
		/*mutex_unlock(&dsi_config->context_lock);*/
		/*return;*/
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs = dpi_output->p_funcs;
	if (!p_funcs) {
		DRM_ERROR("%s: invalid panel function table\n", __func__);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);

	/* ASUS_BSP: Louis, reset display panel flip while doing DDS +++	*/
	ctx->dspsurf = 0x0;
	ctx->dsplinoff = 0x0;
	REG_WRITE(regs->dspsurf_reg, ctx->dspsurf);
	REG_WRITE(regs->dsplinoff_reg, ctx->dsplinoff);
	/* ASUS_BSP: Louis, reset display panel flip while doing DDS ---	*/

	if (__dpi_panel_power_off(dsi_config, p_funcs)) {
		DRM_INFO("%s: off failed\n", __func__);
		/*mutex_unlock(&dsi_config->context_lock);*/
		/*return;*/
	}

	acquire_ospm_lock();

	/*ospm_power_island_down(OSPM_DISPLAY_ISLAND);*/
	ospm_suspend_display(gpDrmDevice);

	panel_id = connected;
	DRM_INFO("[DISPLAY] [DDS] %s: Enter, change panel_id =%d\n", __func__, panel_id);

/*	ospm_power_island_up(OSPM_DISPLAY_ISLAND);*/
	ospm_resume_display(gpDrmDevice->pdev);

	release_ospm_lock();

	if (__dpi_panel_power_on(dsi_config, p_funcs)) {
		DRM_INFO("%s: on failed\n", __func__);
		/*psb_enable_vblank(dev, 0);*/
		/*mutex_unlock(&dsi_config->context_lock);*/
		/*return;*/
	}

	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 1);

	mutex_unlock(&dsi_config->context_lock);
	/*leve1_cnt = 0;
	vsync_timeout_cnt = 0;*/
	DRM_INFO("%s: End panel reset\n", __func__);
}

void mdfld_reset_same_dpi_panel_work(struct work_struct *work)
{
	struct drm_psb_private *dev_priv =
		container_of(work, struct drm_psb_private, reset_panel_work);

	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	struct mdfld_dsi_dpi_output *dpi_output = dev_priv->dpi_output;
	struct panel_funcs *p_funcs;
	struct drm_device *dev;

	if (!dsi_config || !dpi_output) {
		if (!dsi_config)
			DRM_INFO("%s: dsi_config NULL\n",  __func__);
		else if (!dpi_output)
			DRM_INFO("%s: dpi_output NULL\n", __func__);
		return;
	}

	dev = dsi_config->dev;

	/*disable ESD when HDMI connected*/
	if (hdmi_state) {
		DRM_INFO("%s: hdmi_state %d\n", __func__, hdmi_state);
		return;
	}

	DRM_INFO("[DISPLAY] [DDS] %s: Enter\n", __func__);

	mutex_lock(&dsi_config->context_lock);

	if ((panel_turn_on == DDS_NONE) || (dsi_config->dsi_hw_context.panel_on == 0)) {
		DRM_INFO("[DISPLAY] [DDS] %s:  panel_turn_on=%d , panel_on=%d, skip reset.\n",
							__func__, panel_turn_on, dsi_config->dsi_hw_context.panel_on);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs = dpi_output->p_funcs;
	if (!p_funcs) {
		DRM_ERROR("%s: invalid panel function table\n", __func__);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);

	if (__dpi_panel_power_off(dsi_config, p_funcs)) {
		DRM_INFO("%s: off failed\n", __func__);
		/*mutex_unlock(&dsi_config->context_lock);
		return;*/
	}

	acquire_ospm_lock();

/*	ospm_power_island_down(OSPM_DISPLAY_ISLAND);*/
	ospm_suspend_display(gpDrmDevice);

/*	ospm_power_island_up(OSPM_DISPLAY_ISLAND);*/
	ospm_resume_display(gpDrmDevice->pdev);

	release_ospm_lock();

	if (__dpi_panel_power_on(dsi_config, p_funcs)) {
		DRM_INFO("%s: on failed\n", __func__);
		/*psb_enable_vblank(dev, 0);
		mutex_unlock(&dsi_config->context_lock);
		return;*/
	}

	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 1);

	mutex_unlock(&dsi_config->context_lock);
	/*leve1_cnt = 0;
	vsync_timeout_cnt = 0;*/
	DRM_INFO("%s: End panel reset\n", __func__);
}

void mdfld_reset_same_dpi_panel(struct drm_psb_private *dev_priv)
{
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	struct mdfld_dsi_dpi_output *dpi_output = dev_priv->dpi_output;
	struct panel_funcs *p_funcs;
	struct drm_device *dev;

	if (!dsi_config || !dpi_output) {
		if (!dsi_config)
			DRM_INFO("%s: dsi_config NULL\n", __func__);
		else if (!dpi_output)
			DRM_INFO("%s: dpi_output NULL\n", __func__);
		return;
	}

	dev = dsi_config->dev;

	/*disable ESD when HDMI connected*/
	if (hdmi_state) {
		DRM_INFO("%s: hdmi_state %d\n", __func__, hdmi_state);
		return;
	}

	DRM_INFO("[DISPLAY] [DDS] %s: Enter\n", __func__);

	mutex_lock(&dsi_config->context_lock);

	if ((panel_turn_on == DDS_NONE) || (dsi_config->dsi_hw_context.panel_on == 0)) {
		DRM_INFO("[DISPLAY] [DDS] %s:  panel_turn_on=%d , panel_on=%d, skip reset.\n",
							__func__, panel_turn_on, dsi_config->dsi_hw_context.panel_on);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs = dpi_output->p_funcs;
	if (!p_funcs) {
		DRM_ERROR("%s: invalid panel function table\n", __func__);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);

	if (__dpi_panel_power_off(dsi_config, p_funcs)) {
		DRM_INFO("%s: off failed\n", __func__);
		/*mutex_unlock(&dsi_config->context_lock);
		return;*/
	}

	acquire_ospm_lock();

/*	ospm_power_island_down(OSPM_DISPLAY_ISLAND);*/
	ospm_suspend_display(gpDrmDevice);

/*	ospm_power_island_up(OSPM_DISPLAY_ISLAND);*/
	ospm_resume_display(gpDrmDevice->pdev);

	release_ospm_lock();

	if (__dpi_panel_power_on(dsi_config, p_funcs)) {
		DRM_INFO("%s: on failed\n", __func__);
		/*psb_enable_vblank(dev, 0);
		mutex_unlock(&dsi_config->context_lock);
		return;*/
	}

	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 1);

	mutex_unlock(&dsi_config->context_lock);
	/*leve1_cnt = 0;
	vsync_timeout_cnt = 0;*/
	DRM_INFO("%s: End panel reset\n", __func__);
}

#endif

#if defined(CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY)
void mdfld_shutdown_panel(struct drm_psb_private *dev_priv)
{
	struct mdfld_dsi_config *dsi_config = dev_priv->dsi_configs[0];
	struct mdfld_dsi_dpi_output *dpi_output = dev_priv->dpi_output;
	struct panel_funcs *p_funcs;
	struct drm_device *dev;

	if (!dsi_config || !dpi_output) {
		if (!dsi_config)
			DRM_INFO("%s: dsi_config NULL\n", __func__);
		else if (!dpi_output)
			DRM_INFO("%s: dpi_output NULL\n", __func__);
		return;
	}

	dev = dsi_config->dev;

	/*disable ESD when HDMI connected*/
	if (hdmi_state) {
		DRM_INFO("%s: hdmi_state %d\n", __func__, hdmi_state);
		return;
	}

	DRM_INFO("[DISPLAY] [DDS] %s: Enter\n", __func__);

	mutex_lock(&dsi_config->context_lock);

	if ((panel_turn_on == DDS_NONE) || (dsi_config->dsi_hw_context.panel_on == 0)) {
		DRM_INFO("[DISPLAY] [DDS] %s:  panel_turn_on=%d , panel_on=%d, skip reset.\n",
							__func__, panel_turn_on, dsi_config->dsi_hw_context.panel_on);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	PSB_DEBUG_ENTRY("\n");

	p_funcs = dpi_output->p_funcs;
	if (!p_funcs) {
		DRM_ERROR("%s: invalid panel function table\n", __func__);
		mutex_unlock(&dsi_config->context_lock);
		return;
	}

	if (dev_priv->pvr_screen_event_handler)
		dev_priv->pvr_screen_event_handler(dev, 0);

	if (__dpi_panel_power_off(dsi_config, p_funcs)) {
		DRM_INFO("%s: off failed\n", __func__);
		/*mutex_unlock(&dsi_config->context_lock);
		return;*/
	}

	mutex_unlock(&dsi_config->context_lock);
	/*leve1_cnt = 0;
	vsync_timeout_cnt = 0;*/
	DRM_INFO("%s: End shutdown\n", __func__);
}
#endif
/*ASUS_BSP: [DDS] ---*/

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
	int pipe = 0;
	struct drm_device *dev;
	struct drm_psb_private *dev_priv;
	static int last_ospm_suspend = -1;

	if (!encoder) {
		DRM_ERROR("Invalid encoder\n");
		return -EINVAL;
	}

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	DRM_INFO("[DISPLAY][DDS] %s: panel_id = %d, panel_turn_on = %d\n", __func__, panel_id, panel_turn_on);
	DRM_INFO("[DISPLAY][DDS] mdfld_dsi_dpi_set_power: set power %s on pipe %d\n", on ? "On" : "Off", pipe);
#else
	PSB_DEBUG_ENTRY("%s, last_ospm_suspend = %s\n", (on ? "on" : "off"),
			(last_ospm_suspend ? "true" : "false"));
#endif
/*ASUS_BSP: [DDS] ---*/

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
/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY

			if (on && (panel_turn_on == DDS_PHONE) && (asus_panel_id == OFILM_PANEL))
				mdfld_dsi_error_detector_wakeup(dsi_connector);
#endif
/*ASUS_BSP: [DDS] ---*/

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
#ifdef CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY
		esd_thread_enable = true;
#endif
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

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY
		/*don't care color mode*/
#else
		if (!strncmp("NT35521", dev_priv->panel_info.name, 7)) {
		    /*NT35521 don't care color mode*/
		    ;
		} else{
		    /**
		     * If power on, turn off color mode by default,
		     * let panel in full color mode
		     */
		    mdfld_dsi_dpi_set_color_mode(dsi_config, false);
		}
#endif
		dsi_config->dsi_hw_context.panel_on = 1;
/*ASUS_BSP: [DDS] ---*/

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY

		if ((panel_turn_on == DDS_PHONE) && (asus_panel_id == OFILM_PANEL))
			mdfld_dsi_error_detector_wakeup(dsi_connector);
#endif
/*ASUS_BSP: [DDS] ---*/

		last_ospm_suspend = false;
		break;
	case false:
#ifdef CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY
		esd_thread_enable = false;
		msleep(10);
#endif
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

/*ASUS_BSP: [DDS] +++*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	DRM_INFO("[DISPLAY][DDS] %s: panel_id = %d, panel_turn_on = %d\n", __func__, panel_id, panel_turn_on);
	DRM_INFO("[DISPLAY][DDS] mdfld_dsi_dpi_set_power: set power %s on pipe %d\n", on ? "On" : "Off", pipe);
#else
	PSB_DEBUG_ENTRY("set power %s on pipe %d\n", on ? "On" : "Off", pipe);
#endif
/*ASUS_BSP: [DDS] ---*/

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
	struct drm_psb_private *dev_priv = dev->dev_private;
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
	if (dsi_connector->pipe) {
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
