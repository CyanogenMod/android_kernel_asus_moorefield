/*
 * Copyright Â© 2006-2007 Intel Corporation
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
 *	Eric Anholt <eric@anholt.net>
 */

#include <linux/i2c.h>
#include <linux/pm_runtime.h>
#include <drm/drmP.h>

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_dpi.h"
#include "mdfld_csc.h"

#ifdef CONFIG_MDFLD_DSI_DPU
#include "mdfld_dsi_dbi_dpu.h"
#endif

#include "drmlfb.h"
#include "psb_fb.h"
#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_powermgmt.h"

#include "android_hdmi.h"
#include "mdfld_dsi_dbi_dsr.h"

#ifdef MIN
#undef MIN
#endif

#define MIN(x, y) (((x) < (y)) ? (x) : (y))
#define MAX_GAMMA			0x10000

struct mrst_clock_t {
	/* derived values */
	int dot;
	int m;
	int p1;
};

struct psb_intel_range_t {
	int min, max;
};

struct mrst_limit_t {
	struct psb_intel_range_t dot, m, p1;
};

static const struct drm_crtc_helper_funcs mdfld_helper_funcs;
static const struct drm_crtc_funcs mdfld_intel_crtc_funcs;

void psb_intel_crtc_init(struct drm_device *dev, int pipe,
		     struct psb_intel_mode_device *mode_dev)
{
	struct drm_psb_private *dev_priv = dev->dev_private;
	struct psb_intel_crtc *psb_intel_crtc;
	int i;
	uint16_t *r_base, *g_base, *b_base;

	PSB_DEBUG_ENTRY("\n");

	/* We allocate a extra array of drm_connector pointers
	 * for fbdev after the crtc */
	psb_intel_crtc =
	    kzalloc(sizeof(struct psb_intel_crtc) +
		    (INTELFB_CONN_LIMIT * sizeof(struct drm_connector *)),
		    GFP_KERNEL);
	if (psb_intel_crtc == NULL)
		return;

	drm_crtc_init(dev, &psb_intel_crtc->base, &mdfld_intel_crtc_funcs);

	drm_mode_crtc_set_gamma_size(&psb_intel_crtc->base, 256);
	psb_intel_crtc->pipe = pipe;
	psb_intel_crtc->plane = pipe;

	r_base = psb_intel_crtc->base.gamma_store;
	g_base = r_base + 256;
	b_base = g_base + 256;
	for (i = 0; i < 256; i++) {
		psb_intel_crtc->lut_r[i] = i;
		psb_intel_crtc->lut_g[i] = i;
		psb_intel_crtc->lut_b[i] = i;
		r_base[i] = i << 8;
		g_base[i] = i << 8;
		b_base[i] = i << 8;

		psb_intel_crtc->lut_adj[i] = 0;
	}

	psb_intel_crtc->mode_dev = mode_dev;
	psb_intel_crtc->cursor_addr = 0;

	drm_crtc_helper_add(&psb_intel_crtc->base, &mdfld_helper_funcs);

	/* Setup the array of drm_connector pointer array */
	psb_intel_crtc->mode_set.crtc = &psb_intel_crtc->base;
	BUG_ON(pipe >= ARRAY_SIZE(dev_priv->plane_to_crtc_mapping) ||
	       dev_priv->plane_to_crtc_mapping[psb_intel_crtc->plane] != NULL);
	dev_priv->plane_to_crtc_mapping[psb_intel_crtc->plane] =
		&psb_intel_crtc->base;
	dev_priv->pipe_to_crtc_mapping[psb_intel_crtc->pipe] =
		&psb_intel_crtc->base;
	psb_intel_crtc->mode_set.connectors =
	    (struct drm_connector **) (psb_intel_crtc + 1);
	psb_intel_crtc->mode_set.num_connectors = 0;
}


void psb_intel_wait_for_vblank(struct drm_device *dev)
{
	/* Wait for 20ms, i.e. one cycle at 50hz. */
	/* Between kernel 3.0 and 3.3, udelay was made to complain at
	   compile time for argument == 20000 or more.
	   Therefore, reduce it from 20000 to 19999. */
	udelay(19999);
}

static int mdfld_intel_crtc_cursor_set(struct drm_crtc *crtc,
				 struct drm_file *file_priv,
				 uint32_t handle,
				 uint32_t width, uint32_t height)
{
	struct drm_device *dev = crtc->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct psb_intel_mode_device *mode_dev = psb_intel_crtc->mode_dev;
	int pipe = psb_intel_crtc->pipe;
	uint32_t control = CURACNTR;
	uint32_t base = CURABASE;
	uint32_t temp;
	size_t addr = 0;
	uint32_t page_offset;
	size_t size;
	void *bo;
	int ret;

	PSB_DEBUG_ENTRY("\n");

	switch (pipe) {
	case 0:
		break;
	case 1:
		control = CURBCNTR;
		base = CURBBASE;
		break;
	case 2:
		control = CURCCNTR;
		base = CURCBASE;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");
		return -EINVAL;
	}

	/* Can't enalbe HW cursor on plane B/C. */
	if (pipe != 0)
		return 0;

	/* if we want to turn of the cursor ignore width and height */
	if (!handle) {
		DRM_DEBUG("cursor off\n");
		/* turn off the cursor */
		temp = 0;
		temp |= CURSOR_MODE_DISABLE;

		if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					      OSPM_UHB_ONLY_IF_ON)) {
			REG_WRITE(control, temp);
			REG_WRITE(base, 0);
			ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
		}

		/* unpin the old bo */
		if (psb_intel_crtc->cursor_bo) {
			mode_dev->bo_unpin_for_scanout(dev,
						       psb_intel_crtc->
						       cursor_bo);
			psb_intel_crtc->cursor_bo = NULL;
		}
		return 0;
	}

	/* Currently we only support 64x64 cursors */
	if (width != 64 || height != 64) {
		DRM_ERROR("we currently only support 64x64 cursors\n");
		return -EINVAL;
	}

	bo = mode_dev->bo_from_handle(dev, file_priv, handle);
	if (!bo)
		return -ENOENT;

	ret = mode_dev->bo_pin_for_scanout(dev, bo);
	if (ret)
		return ret;
	size = mode_dev->bo_size(dev, bo);
	if (size < width * height * 4) {
		DRM_ERROR("buffer is to small\n");
		return -ENOMEM;
	}

	/*insert this bo into gtt*/
	ret = psb_gtt_map_meminfo(dev, (IMG_HANDLE)handle, 0, &page_offset);
	if (ret) {
		DRM_ERROR("Can not map meminfo to GTT. handle 0x%x\n", handle);
		return ret;
	}

	addr = page_offset << PAGE_SHIFT;

	if (IS_POULSBO(dev))
		addr += pg->stolen_base;

	psb_intel_crtc->cursor_addr = addr;

	temp = 0;
	/* set the pipe for the cursor */
	temp |= (pipe << 28);
	temp |= CURSOR_MODE_64_ARGB_AX | MCURSOR_GAMMA_ENABLE;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				OSPM_UHB_ONLY_IF_ON)) {
		REG_WRITE(control, temp);
		REG_WRITE(base, addr);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}

	/* unpin the old bo */
	if (psb_intel_crtc->cursor_bo && psb_intel_crtc->cursor_bo != bo) {
		mode_dev->bo_unpin_for_scanout(dev, psb_intel_crtc->cursor_bo);
		psb_intel_crtc->cursor_bo = bo;
	}

	return 0;
}

static int mdfld_intel_crtc_cursor_move(struct drm_crtc *crtc, int x, int y)
{
	struct drm_device *dev = crtc->dev;
#ifndef CONFIG_MDFLD_DSI_DPU
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
#else
	struct psb_drm_dpu_rect rect;
#endif
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	uint32_t pos = CURAPOS;
	uint32_t base = CURABASE;
	uint32_t temp = 0;
	uint32_t addr;

	switch (pipe) {
	case 0:
#ifndef CONFIG_MDFLD_DSI_DPU
		if (!(dev_priv->dsr_fb_update & MDFLD_DSR_CURSOR_0))
			mdfld_dsi_dbi_exit_dsr(dev, MDFLD_DSR_CURSOR_0, 0, 0);
#else /*CONFIG_MDFLD_DSI_DPU*/
		rect.x = x;
		rect.y = y;

		mdfld_dbi_dpu_report_damage(dev, MDFLD_CURSORA, &rect);
		mdfld_dpu_exit_dsr(dev);
#endif
		break;
	case 1:
		pos = CURBPOS;
		base = CURBBASE;
		break;
	case 2:
#ifndef CONFIG_MDFLD_DSI_DPU
		if (!(dev_priv->dsr_fb_update & MDFLD_DSR_CURSOR_2))
			mdfld_dsi_dbi_exit_dsr(dev, MDFLD_DSR_CURSOR_2, 0, 0);
#else /*CONFIG_MDFLD_DSI_DPU*/
		mdfld_dbi_dpu_report_damage(dev, MDFLD_CURSORC, &rect);
		mdfld_dpu_exit_dsr(dev);
#endif
		pos = CURCPOS;
		base = CURCBASE;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");
		return -EINVAL;
	}

	/* Can't enalbe HW cursor on plane B/C. */
	if (pipe != 0)
		return 0;

	if (x < 0) {
		temp |= (CURSOR_POS_SIGN << CURSOR_X_SHIFT);
		x = -x;
	}
	if (y < 0) {
		temp |= (CURSOR_POS_SIGN << CURSOR_Y_SHIFT);
		y = -y;
	}

	temp |= ((x & CURSOR_POS_MASK) << CURSOR_X_SHIFT);
	temp |= ((y & CURSOR_POS_MASK) << CURSOR_Y_SHIFT);

	addr = psb_intel_crtc->cursor_addr;

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_ONLY_IF_ON)) {
		REG_WRITE(pos, temp);
		REG_WRITE(base, addr);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	}

	return 0;
}

/** Loads the palette/gamma unit for the CRTC with the prepared values */
void psb_intel_crtc_load_lut(struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct mdfld_dsi_config *dsi_config = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	int palreg = PALETTE_A;
	int i;

	/* The clocks have to be on to load the palette. */
	if (!crtc->enabled || !dev_priv)
		return;

	dsi_config = dev_priv->dsi_configs[0];
	if (!dsi_config)
		return;
	ctx = &dsi_config->dsi_hw_context;

	switch (psb_intel_crtc->pipe) {
	case 0:
		break;
	case 1:
		palreg = PALETTE_B;
		break;
	case 2:
		palreg = PALETTE_C;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");
		return;
	}

	if (ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				      OSPM_UHB_FORCE_POWER_ON)) {
		mdfld_dsi_dsr_forbid(dsi_config);
		for (i = 0; i < 256; i++) {
			ctx->palette[i] =
				  ((psb_intel_crtc->lut_r[i] +
				  psb_intel_crtc->lut_adj[i]) << 16) |
				  ((psb_intel_crtc->lut_g[i] +
				  psb_intel_crtc->lut_adj[i]) << 8) |
				  (psb_intel_crtc->lut_b[i] +
				  psb_intel_crtc->lut_adj[i]);
			REG_WRITE((palreg + 4 * i), ctx->palette[i]);
		}
		mdfld_dsi_dsr_allow(dsi_config);
		ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
	} else {
		for (i = 0; i < 256; i++) {
			dev_priv->save_palette_a[i] =
				  ((psb_intel_crtc->lut_r[i] +
				  psb_intel_crtc->lut_adj[i]) << 16) |
				  ((psb_intel_crtc->lut_g[i] +
				  psb_intel_crtc->lut_adj[i]) << 8) |
				  (psb_intel_crtc->lut_b[i] +
				  psb_intel_crtc->lut_adj[i]);
		}

	}
}

static void psb_intel_crtc_gamma_set(struct drm_crtc *crtc, u16 *red,
				u16 *green, u16 *blue,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,3,0))
				uint32_t start,
#endif
				uint32_t size)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int i;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0))
	int start = 0;
	int brk = 256;
#else
	int brk = (start + size > 256) ? 256 : start + size;
#endif
	for (i = start; i < brk; i++) {
		psb_intel_crtc->lut_r[i] = red[i] >> 8;
		psb_intel_crtc->lut_g[i] = green[i] >> 8;
		psb_intel_crtc->lut_b[i] = blue[i] >> 8;
	}

	psb_intel_crtc_load_lut(crtc);
}

static void psb_intel_crtc_destroy(struct drm_crtc *crtc)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);

#ifndef CONFIG_X86_MDFLD
	kfree(psb_intel_crtc->crtc_state);
#endif
	drm_crtc_cleanup(crtc);
	kfree(psb_intel_crtc);
}

/**
 * @dev: DRM device
 * @pipe: wait on which pipe
 * @state: wait for pipe status.
 *
 * return: void
 *
 * wait for pipe enable/disable according to state param, and return
 * corresponding result.
 */
inline
void intel_wait_for_pipe_enable_disable(struct drm_device *dev,
		int pipe, bool state)
{
	int retry = 10000;
	u32 pipeconf_reg = 0;
	u32 pipe_state = 0;

	switch (pipe) {
	case 0:
		pipeconf_reg = PIPEACONF;
		break;
	case 1:
		pipeconf_reg = PIPEBCONF;
		break;
	case 2:
		pipeconf_reg = PIPECCONF;
		break;
	default:
		DRM_ERROR("wrong pipe number!\n");
		return;
	}

	while (--retry) {
		pipe_state = REG_READ(pipeconf_reg) & BIT30;
		if ((pipe_state >> 30) == state)
			break;
		udelay(2);
	}

	if (!retry)
		DRM_ERROR("pipe %d faild to change to state %d\n", pipe, state);
}

/*
 * set display controller side palette, it will influence
 * brightness , saturation , contrast.
 * KAI1
*/
int mdfld_intel_crtc_set_gamma(struct drm_device *dev,
				struct gamma_setting *setting_data)
{
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_config *dsi_config = NULL;
	int ret = 0;
	int pipe = 0;
	u32 val = 0;

	PSB_DEBUG_ENTRY("\n");

	if (!dev || !setting_data) {
		ret = -EINVAL;
		return ret;
	}

	if (!(setting_data->type &
		(GAMMA_SETTING|GAMMA_INITIA|GAMMA_REG_SETTING))) {
		ret = -EINVAL;
		return ret;
	}
	if ((setting_data->type == GAMMA_SETTING &&
		setting_data->data_len != GAMMA_10_BIT_TABLE_COUNT) ||
		(setting_data->type == GAMMA_REG_SETTING &&
		setting_data->data_len != GAMMA_10_BIT_TABLE_COUNT)) {
		ret = -EINVAL;
		return ret;
	}

	dev_priv = dev->dev_private;
	pipe = setting_data->pipe;

	if (pipe == 0)
		dsi_config = dev_priv->dsi_configs[0];
	else if (pipe == 2)
		dsi_config = dev_priv->dsi_configs[1];
	else if (pipe == 1) {
		PSB_DEBUG_ENTRY("/KAI1 palette no implement for HDMI\n"
				"do it later\n");
		return -EINVAL;
	} else
		return -EINVAL;

	mutex_lock(&dev_priv->gamma_csc_lock);

	ctx = &dsi_config->dsi_hw_context;
	regs = &dsi_config->regs;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
		ret = -EAGAIN;
		goto _fun_exit;
	}

	/*forbid dsr which will restore regs*/
	mdfld_dsi_dsr_forbid(dsi_config);

	/*enable gamma*/
	if (drm_psb_enable_gamma && setting_data->enable_state) {
		int i = 0, temp = 0;
		u32 integer_part = 0, fraction_part = 0, even_part = 0,
		    odd_part = 0;
		u32 int_red_9_2 = 0, int_green_9_2 = 0, int_blue_9_2 = 0;
		u32 int_red_1_0 = 0, int_green_1_0 = 0, int_blue_1_0 = 0;
		u32 fra_red = 0, fra_green = 0, fra_blue = 0;
		int j = 0;
		/*here set r/g/b the same curve*/
		for (i = 0; i <= 1024; i = i + 8) {
			if (setting_data->type == GAMMA_INITIA) {
				switch (setting_data->initia_mode) {
				case GAMMA_05:
					/* gamma 0.5 */
					temp = 32 * int_sqrt(i * 10000);
					printk(KERN_ALERT "gamma 0.5\n");
					break;
				case GAMMA_20:
					/* gamma 2 */
					temp = (i * i * 100) / 1024;
					printk(KERN_ALERT "gamma 2\n");
					break;
				case GAMMA_05_20:
					/*
					 * 0 ~ 511 gamma 0.5
					 * 512 ~1024 gamma 2
					 */
					if (i < 512)
						temp = int_sqrt(i * 512 *
								10000);
					else
						temp = (i - 512) * (i - 512) *
							100 / 512  + 512 * 100;
					printk(KERN_ALERT "gamma 0.5 + gamma 2\n");
					break;
				case GAMMA_20_05:
					/*
					 * 0 ~ 511 gamma 2
					 * 512 ~1024 gamma 0.5
					 */
					if (i < 512)
						temp = i * i * 100 / 512;
					else
						temp = int_sqrt((i - 512) *
								512 * 10000)
							+ 512 * 100;
					printk(KERN_ALERT "gamma 2 + gamma 0.5\n");
					break;
				case GAMMA_10:
					/* gamma 1 */
					temp = i * 100;
					printk(KERN_ALERT "gamma 1\n");
					break;
				default:
					/* gamma 0.5 */
					temp = 32 * int_sqrt(i *  10000);
					printk(KERN_ALERT "gamma 0.5\n");
					break;
				}
			} else {
				temp = setting_data->gamma_tableX100[i / 8];
			}

			if (setting_data->type == GAMMA_REG_SETTING) {
				if (i != 1024) {
					REG_WRITE(regs->palette_reg + j, 0);
					ctx->palette[(i / 8) * 2] = 0;
					REG_WRITE(regs->palette_reg + j + 4, temp);
					ctx->palette[(i / 8) * 2 + 1] = temp;
				} else {
#ifndef CONFIG_A500CG
					REG_WRITE(regs->gamma_red_max_reg, MAX_GAMMA);
					REG_WRITE(regs->gamma_green_max_reg, MAX_GAMMA);
					REG_WRITE(regs->gamma_blue_max_reg, MAX_GAMMA);
#else
					//AP team will assign MAX_GAMMA
					REG_WRITE(regs->gamma_red_max_reg, setting_data->gamma_tableX100[128]);
					REG_WRITE(regs->gamma_green_max_reg, setting_data->gamma_tableX100[129]);
					REG_WRITE(regs->gamma_blue_max_reg, setting_data->gamma_tableX100[130]);
#endif
				}
			} else {
				if (temp < 0)
					temp = 0;
				if (temp > 1024 * 100)
					temp = 1024 * 100;

				integer_part = temp / 100;
				fraction_part = (temp - integer_part * 100);
				/*get r/g/b each channel*/
				int_blue_9_2 = integer_part >> 2;
				int_green_9_2 = int_blue_9_2 << 8;
				int_red_9_2 = int_blue_9_2 << 16;
				int_blue_1_0 = (integer_part & 0x3) << 6;
				int_green_1_0 = int_blue_1_0 << 8;
				int_red_1_0 = int_blue_1_0 << 16;
				fra_blue = fraction_part*64/100;
				fra_green = fra_blue << 8;
				fra_red = fra_blue << 16;
				/*get even and odd part*/
				odd_part = int_red_9_2 | int_green_9_2 | int_blue_9_2;
				even_part = int_red_1_0 | fra_red | int_green_1_0 |
					fra_green | int_blue_1_0 | fra_blue;
				if (i != 1024) {
					REG_WRITE(regs->palette_reg + j, even_part);
					REG_WRITE(regs->palette_reg + j + 4, odd_part);
				} else {
					REG_WRITE(regs->gamma_red_max_reg,
							(integer_part << 6) |
							(fraction_part));
					REG_WRITE(regs->gamma_green_max_reg,
							(integer_part << 6) |
							(fraction_part));
					REG_WRITE(regs->gamma_blue_max_reg,
							(integer_part << 6) |
							(fraction_part));
					printk(KERN_ALERT
							"max (red %x, green 0x%x, blue 0x%x)\n",
						REG_READ(regs->gamma_red_max_reg),
						REG_READ(regs->gamma_green_max_reg),
						REG_READ(regs->gamma_blue_max_reg));
				}
			}

			j = j + 8;
		}
		/*enable*/
		val = REG_READ(regs->pipeconf_reg);
		val |= (PIPEACONF_GAMMA);
		REG_WRITE(regs->pipeconf_reg, val);
		ctx->pipeconf = val;
		REG_WRITE(regs->dspcntr_reg, REG_READ(regs->dspcntr_reg) |
				DISPPLANE_GAMMA_ENABLE);
		ctx->dspcntr = REG_READ(regs->dspcntr_reg) | DISPPLANE_GAMMA_ENABLE;
		REG_READ(regs->dspcntr_reg);
	} else {
		drm_psb_enable_gamma = 0;
		/*disable */
		val = REG_READ(regs->pipeconf_reg);
		val &= ~(PIPEACONF_GAMMA);
		REG_WRITE(regs->pipeconf_reg, val);
		ctx->pipeconf = val;
		REG_WRITE(regs->dspcntr_reg,
				REG_READ(regs->dspcntr_reg) &
				~(DISPPLANE_GAMMA_ENABLE));
		ctx->dspcntr = REG_READ(regs->dspcntr_reg) & (~DISPPLANE_GAMMA_ENABLE);
		REG_READ(regs->dspcntr_reg);
	}

	mdfld_dsi_dsr_update_panel_fb(dsi_config);
	/*allow entering dsr*/
	mdfld_dsi_dsr_allow(dsi_config);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

_fun_exit:
	mutex_unlock(&dev_priv->gamma_csc_lock);
	return ret;
}

/*
 * set display controller side color conversion
 * KAI1
*/
int mdfld_intel_crtc_set_color_conversion(struct drm_device *dev,
					struct csc_setting *setting_data)
{
	struct drm_psb_private *dev_priv = NULL;
	struct mdfld_dsi_hw_context *ctx = NULL;
	struct mdfld_dsi_hw_registers *regs;
	struct mdfld_dsi_config *dsi_config = NULL;
	int ret = 0;
	int i = 0;
	int pipe = 0;
	u32 val = 0;
	/*Rx, Ry, Gx, Gy, Bx, By, Wx, Wy*/
	/*sRGB color space*/
	uint32_t chrom_input[8] = {	6400, 3300,
		3000, 6000,
		1500, 600,
		3127, 3290 };
	/* PR3 color space*/
	uint32_t chrom_output[8] = { 6382, 3361,
		2979, 6193,
		1448, 478,
		3000, 3236 };

	PSB_DEBUG_ENTRY("\n");

	if (!dev) {
		ret = -EINVAL;
		return ret;
	}

	if (!(setting_data->type &
		(CSC_CHROME_SETTING | CSC_INITIA | CSC_SETTING | CSC_REG_SETTING))) {
		ret = -EINVAL;
		return ret;
	}
	if ((setting_data->type == CSC_SETTING &&
		setting_data->data_len != CSC_COUNT) ||
		(setting_data->type == CSC_CHROME_SETTING &&
		setting_data->data_len != CHROME_COUNT) ||
		(setting_data->type == CSC_REG_SETTING &&
		setting_data->data_len != CSC_REG_COUNT)) {
		ret = -EINVAL;
		return ret;
	}

	dev_priv = dev->dev_private;
	pipe = setting_data->pipe;

	if (pipe == 0)
		dsi_config = dev_priv->dsi_configs[0];
	else if (pipe == 2)
		dsi_config = dev_priv->dsi_configs[1];
	else if (pipe == 1) {
		PSB_DEBUG_ENTRY("/KAI1 color conversion no implement for HDMI\n"
				"do it later\n");
		return -EINVAL;
	} else
		return -EINVAL;

	mutex_lock(&dev_priv->gamma_csc_lock);

	ctx = &dsi_config->dsi_hw_context;
	regs = &dsi_config->regs;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
					OSPM_UHB_FORCE_POWER_ON)) {
		ret = -EAGAIN;
		goto _fun_exit;
	}

	/*forbid dsr which will restore regs*/
	mdfld_dsi_dsr_forbid(dsi_config);

	if (drm_psb_enable_color_conversion && setting_data->enable_state) {
		if (setting_data->type == CSC_INITIA) {
			/*initialize*/
			csc(dev, &chrom_input[0], &chrom_output[0], pipe);
		} else if (setting_data->type == CSC_CHROME_SETTING) {
			/*use chrome to caculate csc*/
			memcpy(chrom_input, setting_data->data.chrome_data,
					8 * sizeof(int));
			memcpy(chrom_output, setting_data->data.chrome_data + 8,
					8 * sizeof(int));
			csc(dev, &chrom_input[0], &chrom_output[0], pipe);
		} else if (setting_data->type == CSC_SETTING) {
			/*use user space csc*/
			csc_program_DC(dev, &setting_data->data.csc_data[0],
					pipe);
		} else if (setting_data->type == CSC_REG_SETTING) {
			/*use user space csc regiseter setting*/
			for (i = 0; i < 6; i++) {
				REG_WRITE(regs->color_coef_reg + (i<<2), setting_data->data.csc_reg_data[i]);
				ctx->color_coef[i] = setting_data->data.csc_reg_data[i];
			}
		}

		/*enable*/
		val = REG_READ(regs->pipeconf_reg);
		val |= (PIPEACONF_COLOR_MATRIX_ENABLE);
		REG_WRITE(regs->pipeconf_reg, val);
		ctx->pipeconf = val;
		val = REG_READ(regs->dspcntr_reg);
		REG_WRITE(regs->dspcntr_reg, val);
	} else {
		drm_psb_enable_color_conversion = 0;
		/*disable*/
		val = REG_READ(regs->pipeconf_reg);
		val &= ~(PIPEACONF_COLOR_MATRIX_ENABLE);
		REG_WRITE(regs->pipeconf_reg, val);
		ctx->pipeconf = val;
		val = REG_READ(regs->dspcntr_reg);
		REG_WRITE(regs->dspcntr_reg, val);
	}

	mdfld_dsi_dsr_update_panel_fb(dsi_config);
	/*allow entering dsr*/
	mdfld_dsi_dsr_allow(dsi_config);

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

_fun_exit:
	mutex_unlock(&dev_priv->gamma_csc_lock);
	return ret;
}

static void pfit_landscape(struct drm_device *dev,
				int hsrc_sz, int vsrc_sz,
				int hdst_sz, int vdst_sz)
{
	int hscale = 0, vscale = 0;

	REG_WRITE(PFIT_CONTROL, PFIT_ENABLE | PFIT_PIPE_SELECT_B |
			PFIT_SCALING_MODE_PROGRAM);

	hscale = PFIT_FRACTIONAL_VALUE * (hsrc_sz + 1) / (hdst_sz + 1);
	vscale = PFIT_FRACTIONAL_VALUE * (vsrc_sz + 1) / (vdst_sz + 1);

	PSB_DEBUG_ENTRY("hscale = 0x%X, vscale = 0X%X\n", hscale, vscale);

	REG_WRITE(PFIT_PGM_RATIOS,
			hscale << PFIT_HORIZ_SCALE_SHIFT |
			vscale << PFIT_VERT_SCALE_SHIFT);
}

static int
mdfld_intel_set_scaling_property(struct drm_crtc *crtc, int x, int y, int pipe)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct drm_framebuffer *fb = crtc->fb;
	struct drm_display_mode *adjusted_mode =
		&psb_intel_crtc->saved_adjusted_mode;
	uint64_t scalingType = psb_intel_crtc->scaling_type;
	int pipesrc_reg = PIPEASRC;
	int dspsize_reg = DSPASIZE;
	int dsppos_reg = DSPAPOS;
	int sprite_pos_x = 0, sprite_pos_y = 0;
	int sprite_width = 0, sprite_height = 0;
	int src_image_hor = 0, src_image_vert = 0;

	switch (pipe) {
	case 0:
		break;
	case 1:
		pipesrc_reg = PIPEBSRC;
		dspsize_reg = DSPBSIZE;
		dsppos_reg = DSPBPOS;
		break;
	case 2:
		pipesrc_reg = PIPECSRC;
		dspsize_reg = DSPCSIZE;
		dsppos_reg = DSPCPOS;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");
		return -EINVAL;
	}

	PSB_DEBUG_ENTRY("scalingType %llu\n", scalingType);
	/* pipesrc and dspsize control the size that is scaled from,
	 * which should always be the user's requested size.
	 */
	if (pipe != 1) {
		REG_WRITE(dspsize_reg, ((adjusted_mode->vdisplay - 1) << 16) |
				(adjusted_mode->hdisplay - 1));
		REG_WRITE(pipesrc_reg, ((adjusted_mode->hdisplay - 1) << 16) |
				(adjusted_mode->vdisplay - 1));
		REG_WRITE(dsppos_reg, 0);

		return 0;
	}

	/*
	 * Frame buffer size may beyond active region in case of
	 * panning mode.
	 */
	sprite_width = MIN(fb->width, adjusted_mode->hdisplay);
	sprite_height = MIN(fb->height, adjusted_mode->vdisplay);

	switch (scalingType) {
	case DRM_MODE_SCALE_NONE:
	case DRM_MODE_SCALE_CENTER:
		/*
		 * This mode is used to support centering the screen
		 * by setting reg in DISPLAY controller
		 */
		src_image_hor = adjusted_mode->hdisplay;
		src_image_vert = adjusted_mode->vdisplay;
		sprite_pos_x = (src_image_hor - sprite_width) / 2;
		sprite_pos_y = (src_image_vert - sprite_height) / 2;

		REG_WRITE(PFIT_CONTROL,
				REG_READ(PFIT_CONTROL) & ~PFIT_ENABLE);

		break;

	case DRM_MODE_SCALE_FULLSCREEN:
		src_image_hor = sprite_width;
		src_image_vert = sprite_height;
		sprite_pos_x = 0;
		sprite_pos_y = 0;

		if ((adjusted_mode->hdisplay > sprite_width) ||
				(adjusted_mode->vdisplay > sprite_height))
			REG_WRITE(PFIT_CONTROL,
					PFIT_ENABLE |
					PFIT_PIPE_SELECT_B |
					PFIT_SCALING_MODE_AUTO);
		break;

	case DRM_MODE_SCALE_ASPECT:
		sprite_pos_x = 0;
		sprite_pos_y = 0;
		sprite_height = fb->height;
		sprite_width = fb->width;
		src_image_hor = fb->width;
		src_image_vert = fb->height;

		/* Use panel fitting when the display does not match
		 * with the framebuffer size */
		if ((adjusted_mode->hdisplay != fb->width) ||
		    (adjusted_mode->vdisplay != fb->height)) {
			if (fb->width > fb->height) {
				pr_debug("[hdmi]: Landscape mode...\n");
				/* Landscape mode: program ratios is
				 * used because 480p does not work with
				 * auto */
				if (adjusted_mode->vdisplay == 480)
					pfit_landscape(dev,
							sprite_width,
							sprite_height,
							adjusted_mode->hdisplay,
							adjusted_mode->vdisplay
						      );
				else
					REG_WRITE(PFIT_CONTROL,
							PFIT_ENABLE |
							PFIT_PIPE_SELECT_B |
							PFIT_SCALING_MODE_AUTO);
			} else {
				/* Portrait mode */
				pr_debug("[hdmi]: Portrait mode...\n");
				if (adjusted_mode->vdisplay == 768 &&
						adjusted_mode->hdisplay ==
						1024) {
					src_image_hor =
						adjusted_mode->hdisplay *
						fb->height /
						adjusted_mode->vdisplay;
					src_image_vert = fb->height;
					sprite_pos_x =
						(src_image_hor - fb->width) / 2;
					REG_WRITE(PFIT_CONTROL,
							PFIT_ENABLE |
							PFIT_PIPE_SELECT_B |
							PFIT_SCALING_MODE_AUTO);
				} else
					REG_WRITE(PFIT_CONTROL,
						PFIT_ENABLE |
						PFIT_PIPE_SELECT_B |
						PFIT_SCALING_MODE_PILLARBOX);
			}
		} else {
			/* Disable panel fitting */
			REG_WRITE(PFIT_CONTROL, 0);
		}
		break;

	default:
		/* Android will not change mode, however ,we have tools
		   to change HDMI timing so there is some cases frame
		   buffer no change ,but timing changed mode setting, in
		   this case. mode information for source size is not
		   right, so here use fb information for source/sprite
		   size*/

		/* The defined sprite rectangle must always be
		   completely contained within the displayable area of the
		   screen image (frame buffer). */
		sprite_pos_x = 0;
		sprite_pos_y = 0;
		sprite_height = fb->height;
		sprite_width = fb->width;
		src_image_hor = fb->width;
		src_image_vert = fb->height;
		if ((adjusted_mode->hdisplay != fb->width) ||
				(adjusted_mode->vdisplay != fb->height))
			REG_WRITE(PFIT_CONTROL, PFIT_ENABLE |
					PFIT_PIPE_SELECT_B);

		break;
	}

	PSB_DEBUG_ENTRY("Sprite position: (%d, %d)\n", sprite_pos_x,
			sprite_pos_y);
	PSB_DEBUG_ENTRY("Sprite size: %d x %d\n", sprite_width,
			sprite_height);
	PSB_DEBUG_ENTRY("Pipe source image size: %d x %d\n",
			src_image_hor, src_image_vert);

	REG_WRITE(dsppos_reg, (sprite_pos_y << 16) | sprite_pos_x);
	REG_WRITE(dspsize_reg, ((sprite_height - 1) << 16) |
			(sprite_width - 1));
	REG_WRITE(pipesrc_reg, ((src_image_hor - 1) << 16) |
			(src_image_vert - 1));

	return 0;
}

static int mdfld__intel_pipe_set_base(struct drm_crtc *crtc, int x, int y,
		struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	/* struct drm_i915_master_private *master_priv; */
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct psb_framebuffer *psbfb = to_psb_fb(crtc->fb);
	struct psb_intel_mode_device *mode_dev = psb_intel_crtc->mode_dev;
	int pipe = psb_intel_crtc->pipe;
	unsigned long Start, Size, Offset;
	int dsplinoff = DSPALINOFF;
	int dspsurf = DSPASURF;
	int dspstride = DSPASTRIDE;
	int dspcntr_reg = DSPACNTR;
	u32 dspcntr;
	int ret = 0;
	MRST_ERROR eError = MRST_ERROR_GENERIC;

	PSB_DEBUG_ENTRY("pipe = 0x%x.\n", pipe);

	/* no fb bound */
	if (!crtc->fb) {
		PSB_DEBUG_ENTRY("No FB bound\n");
		return 0;
	}

	switch (pipe) {
	case 0:
		if (IS_MID(dev))
			dsplinoff = DSPALINOFF;
		break;
	case 1:
		dsplinoff = DSPBLINOFF;
		dspsurf = DSPBSURF;
		dspstride = DSPBSTRIDE;
		dspcntr_reg = DSPBCNTR;
		break;
	case 2:
		dsplinoff = DSPCLINOFF;
		dspsurf = DSPCSURF;
		dspstride = DSPCSTRIDE;
		dspcntr_reg = DSPCCNTR;
		break;
	default:
		DRM_ERROR("Illegal Pipe Number.\n");
		return -EINVAL;
	}

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return 0;

	if (pipe == 1)
		android_hdmi_set_scaling_property(crtc);
	else
		mdfld_intel_set_scaling_property(crtc, x, y, pipe);

	Start = mode_dev->bo_offset(dev, psbfb);
	if (psbfb->pvrBO == NULL)
		Size = psbfb->size;
	else
		Size = mode_dev->bo_size(dev, psbfb->pvrBO);
	Offset = y * crtc->fb->pitches[0] + x * (crtc->fb->bits_per_pixel / 8);

	/* Try to attach/de-attach Plane B to an existing swap chain,
	 * especially with another frame buffer inserted into GTT. */
	/* TODO: remove it since there's no swap chain anymore*/
#if 0
	eError = MRSTLFBChangeSwapChainProperty(&Start, Size, pipe);
	if ((eError != MRST_OK) && (eError != MRST_ERROR_INIT_FAILURE)) {
		DRM_ERROR("Failed to attach/de-attach pipe %d.\n", pipe);
		ret = -EINVAL;
		goto psb_intel_pipe_set_base_exit;
	}
#endif

	REG_WRITE(dspstride, crtc->fb->pitches[0]);
	dspcntr = REG_READ(dspcntr_reg);
	dspcntr &= ~DISPPLANE_PIXFORMAT_MASK;

	switch (crtc->fb->bits_per_pixel) {
	case 8:
		dspcntr |= DISPPLANE_8BPP;
		break;
	case 16:
		if (crtc->fb->depth == 15)
			dspcntr |= DISPPLANE_15_16BPP;
		else
			dspcntr |= DISPPLANE_16BPP;
		break;
	case 24:
	case 32:
		dspcntr |= DISPPLANE_32BPP_NO_ALPHA;
		break;
	default:
		DRM_ERROR("Unknown color depth\n");
		ret = -EINVAL;
		goto psb_intel_pipe_set_base_exit;
	}
	REG_WRITE(dspcntr_reg, dspcntr);

	PSB_DEBUG_ENTRY("Writing base %08lX %08lX %d %d\n",
			Start, Offset, x, y);

	if (IS_I965G(dev) || IS_MID(dev)) {
		REG_WRITE(dsplinoff, Offset);
		REG_READ(dsplinoff);
		REG_WRITE(dspsurf, Start);
		REG_READ(dspsurf);
	} else {
		REG_WRITE(dsplinoff, Start + Offset);
		REG_READ(dsplinoff);
	}

psb_intel_pipe_set_base_exit:

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);

	return ret;
}

/**
 * Disable the pipe, plane and pll.
 */
void mdfld_disable_crtc(struct drm_device *dev, int pipe)
{
	int dpll_reg = DPLL_B;
	int dspcntr_reg = DSPBCNTR;
	int dspbase_reg = MRST_DSPBBASE;
	int pipeconf_reg = PIPEBCONF;
	u32 temp;

	PSB_DEBUG_ENTRY("pipe = %d\n", pipe);

	/*only for HDMI pipe*/
	if (pipe != 1)
		return;

	/*Disable display plane*/
	temp = REG_READ(dspcntr_reg);
	if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
		REG_WRITE(dspcntr_reg,
			  temp & ~DISPLAY_PLANE_ENABLE);
		/* Flush the plane changes */
		REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
		REG_READ(dspbase_reg);
	}

	/* Next, disable display pipes */
	temp = REG_READ(pipeconf_reg);
	if ((temp & PIPEACONF_ENABLE) != 0) {
		temp &= ~PIPEBCONF_ENABLE;
		temp |= PIPECONF_PLANE_OFF | PIPECONF_CURSOR_OFF;
		REG_WRITE(pipeconf_reg, temp);
		REG_READ(pipeconf_reg);

		/* Wait for for the pipe disable to take effect. */
		intel_wait_for_pipe_enable_disable(dev, pipe, false);
	}

	temp = REG_READ(dpll_reg);
	if (temp & DPLL_VCO_ENABLE) {
		temp &= ~(DPLL_VCO_ENABLE);
		REG_WRITE(dpll_reg, temp);
		REG_READ(dpll_reg);

		if (!(temp & MDFLD_PWR_GATE_EN)) {
			REG_WRITE(dpll_reg, temp | MDFLD_PWR_GATE_EN);
			/*need wait 0.5us before enable VCO*/
			udelay(1);
		}
	}
}

/**
 * Sets the power management mode of the pipe and plane.
 *
 * This code should probably grow support for turning the cursor off and back
 * on appropriately at the same time as we're turning the pipe off/on.
 */
static void mdfld_crtc_dpms(struct drm_crtc *crtc, int mode)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	int pipe = psb_intel_crtc->pipe;
	int dpll_reg = MDFLD_DPLL_B;
	int dspcntr_reg = DSPBCNTR;
	int dspbase_reg = DSPBBASE;
	int pipeconf_reg = PIPEBCONF;
	u32 temp;
	bool enabled;
	int timeout = 0;

	PSB_DEBUG_ENTRY("mode = %d, pipe = %d\n", mode, pipe);

	if (pipe != 1)
		return;

	if (gbSuspended)
		return;

	if (!ospm_power_using_hw_begin(OSPM_DISPLAY_ISLAND,
				       OSPM_UHB_FORCE_POWER_ON))
		return;

	/* XXX: When our outputs are all unaware of DPMS modes other than off
	 * and on, we should map those modes to DRM_MODE_DPMS_OFF in the CRTC.
	 */
	switch (mode) {
	case DRM_MODE_DPMS_ON:
	case DRM_MODE_DPMS_STANDBY:
	case DRM_MODE_DPMS_SUSPEND:
		/* If HDMI is connected, Enable the DPLL */
		if (android_hdmi_is_connected(dev) == true) {
			temp = REG_READ(dpll_reg);
			if ((temp & DPLL_VCO_ENABLE) == 0) {
				/*
				 * When ungating power of DPLL, needs to wait 0.5us
				 * before enable the VCO
				 */
				if (temp & MDFLD_PWR_GATE_EN) {
					temp &= ~MDFLD_PWR_GATE_EN;
					REG_WRITE(dpll_reg, temp);
					udelay(1);
				}

				REG_WRITE(dpll_reg, temp);
				REG_READ(dpll_reg);
				udelay(500);

				REG_WRITE(dpll_reg, temp | DPLL_VCO_ENABLE);
				REG_READ(dpll_reg);

				/**
				 * wait for DSI PLL to lock
				 * NOTE: only need to poll status of pipe 0 and pipe 1,
				 * since both MIPI pipes share the same PLL.
				 */
				while ((pipe != 2) && (timeout < 20000) &&
						!(REG_READ(pipeconf_reg) &
							PIPECONF_DSIPLL_LOCK)) {
					udelay(150);
					timeout++;
				}
			}

			/* Enable the pipe */
			temp = REG_READ(pipeconf_reg);
			if ((temp & PIPEBCONF_ENABLE) == 0) {
				/* Enable Pipe */
				temp |= PIPEACONF_ENABLE;
				/* Enable Display/Overplay Planes */
				temp &= ~PIPECONF_PLANE_OFF;
				/* Enable Cursor Planes */
				temp &= ~PIPECONF_CURSOR_OFF;
				REG_WRITE(pipeconf_reg, temp);
				REG_READ(pipeconf_reg);

				/* Wait for for the pipe enable to take effect. */
				intel_wait_for_pipe_enable_disable(dev, pipe, true);
			}

			/* Enable the plane */
			temp = REG_READ(dspcntr_reg);
			if ((temp & DISPLAY_PLANE_ENABLE) == 0) {
				REG_WRITE(dspcntr_reg,
					temp | DISPLAY_PLANE_ENABLE);
				/* Flush the plane changes */
				REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
			}

			psb_intel_crtc_load_lut(crtc);
		}

		break;
	case DRM_MODE_DPMS_OFF:
		/* Disable the VGA plane that we never use */
		REG_WRITE(VGACNTRL, VGA_DISP_DISABLE);

		/* Disable display plane */
		temp = REG_READ(dspcntr_reg);
		if ((temp & DISPLAY_PLANE_ENABLE) != 0) {
			REG_WRITE(dspcntr_reg,
				  temp & ~DISPLAY_PLANE_ENABLE);
			/* Flush the plane changes */
			REG_WRITE(dspbase_reg, REG_READ(dspbase_reg));
			REG_READ(dspbase_reg);
		}

		/* Next, disable display pipes */
		temp = REG_READ(pipeconf_reg);
		if ((temp & PIPEBCONF_ENABLE) != 0) {
			temp &= ~PIPEBCONF_ENABLE;
			temp |= PIPECONF_PLANE_OFF | PIPECONF_CURSOR_OFF;
			REG_WRITE(pipeconf_reg, temp);
			REG_READ(pipeconf_reg);

			/* Wait for for the pipe disable to take effect. */
			intel_wait_for_pipe_enable_disable(dev, pipe, false);
		}

		temp = REG_READ(dpll_reg);
		if (temp & DPLL_VCO_ENABLE) {
			temp &= ~(DPLL_VCO_ENABLE);
			REG_WRITE(dpll_reg, temp);
			REG_READ(dpll_reg);
			/* Wait for the clocks to turn off. */
			/* FIXME_MDFLD PO may need more delay */
			udelay(500);
		}
		break;
	}

	enabled = crtc->enabled && mode != DRM_MODE_DPMS_OFF;

	ospm_power_using_hw_end(OSPM_DISPLAY_ISLAND);
}

static bool psb_intel_crtc_mode_fixup(struct drm_crtc *crtc,
				  const struct drm_display_mode *mode,
				  struct drm_display_mode *adjusted_mode)
{
	return true;
}

#define MDFLD_LIMT_DPLL_19	    0
#define MDFLD_LIMT_DPLL_25	    1
#define MDFLD_LIMT_DPLL_38	    2
#define MDFLD_LIMT_DPLL_83	    3
#define MDFLD_LIMT_DPLL_100	    4
#define MDFLD_LIMT_DSIPLL_19	    5
#define MDFLD_LIMT_DSIPLL_25	    6
#define MDFLD_LIMT_DSIPLL_38	    7
#define MDFLD_LIMT_DSIPLL_83	    8
#define MDFLD_LIMT_DSIPLL_100	    9

#define MDFLD_DOT_MIN		  19750
#define MDFLD_DOT_MAX		  120000
#define MDFLD_DPLL_M_MIN_19	    113
#define MDFLD_DPLL_M_MAX_19	    155
#define MDFLD_DPLL_P1_MIN_19	    2
#define MDFLD_DPLL_P1_MAX_19	    10
#define MDFLD_DPLL_M_MIN_25	    101
#define MDFLD_DPLL_M_MAX_25	    130
#define MDFLD_DPLL_P1_MIN_25	    2
#define MDFLD_DPLL_P1_MAX_25	    10
#define MDFLD_DPLL_M_MIN_38        113
#define MDFLD_DPLL_M_MAX_38        155
#define MDFLD_DPLL_P1_MIN_38       2
#define MDFLD_DPLL_P1_MAX_38       10
#define MDFLD_DPLL_M_MIN_83	    64
#define MDFLD_DPLL_M_MAX_83	    64
#define MDFLD_DPLL_P1_MIN_83	    2
#define MDFLD_DPLL_P1_MAX_83	    2
#define MDFLD_DPLL_M_MIN_100	    64
#define MDFLD_DPLL_M_MAX_100	    64
#define MDFLD_DPLL_P1_MIN_100	    2
#define MDFLD_DPLL_P1_MAX_100	    2
#define MDFLD_DSIPLL_M_MIN_19	    64
#define MDFLD_DSIPLL_M_MAX_19	    175
#define MDFLD_DSIPLL_P1_MIN_19	    3
#define MDFLD_DSIPLL_P1_MAX_19	    8
#define MDFLD_DSIPLL_M_MIN_25	    97
#define MDFLD_DSIPLL_M_MAX_25	    140
#define MDFLD_DSIPLL_P1_MIN_25	    3
#define MDFLD_DSIPLL_P1_MAX_25	    9
#define MDFLD_DSIPLL_M_MIN_38      66
#define MDFLD_DSIPLL_M_MAX_38      87
#define MDFLD_DSIPLL_P1_MIN_38     3
#define MDFLD_DSIPLL_P1_MAX_38     8
#define MDFLD_DSIPLL_M_MIN_83	    33
#define MDFLD_DSIPLL_M_MAX_83	    92
#define MDFLD_DSIPLL_P1_MIN_83	    2
#define MDFLD_DSIPLL_P1_MAX_83	    3
#define MDFLD_DSIPLL_M_MIN_100	    97
#define MDFLD_DSIPLL_M_MAX_100	    140
#define MDFLD_DSIPLL_P1_MIN_100	    3
#define MDFLD_DSIPLL_P1_MAX_100	    9
#define VCO_MAX                     3200000

static const struct mrst_limit_t mdfld_limits[] = {
	{			/* MDFLD_LIMT_DPLL_19 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_19, .max = MDFLD_DPLL_M_MAX_19},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_19, .max = MDFLD_DPLL_P1_MAX_19},
	 },
	{			/* MDFLD_LIMT_DPLL_25 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_25, .max = MDFLD_DPLL_M_MAX_25},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_25, .max = MDFLD_DPLL_P1_MAX_25},
	 },
	{			/* MDFLD_LIMT_DPLL_38 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_38, .max = MDFLD_DPLL_M_MAX_38},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_38, .max = MDFLD_DPLL_P1_MAX_38},
	 },
	{			/* MDFLD_LIMT_DPLL_83 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_83, .max = MDFLD_DPLL_M_MAX_83},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_83, .max = MDFLD_DPLL_P1_MAX_83},
	 },
	{			/* MDFLD_LIMT_DPLL_100 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DPLL_M_MIN_100, .max = MDFLD_DPLL_M_MAX_100},
	 .p1 = {.min = MDFLD_DPLL_P1_MIN_100, .max = MDFLD_DPLL_P1_MAX_100},
	 },
	{			/* MDFLD_LIMT_DSIPLL_19 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_19, .max = MDFLD_DSIPLL_M_MAX_19},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_19, .max = MDFLD_DSIPLL_P1_MAX_19},
	 },
	{			/* MDFLD_LIMT_DSIPLL_25 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_25, .max = MDFLD_DSIPLL_M_MAX_25},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_25, .max = MDFLD_DSIPLL_P1_MAX_25},
	 },
	{			/* MDFLD_LIMT_DSIPLL_38 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_38, .max = MDFLD_DSIPLL_M_MAX_38},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_38, .max = MDFLD_DSIPLL_P1_MAX_38},
	 },
	{			/* MDFLD_LIMT_DSIPLL_83 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_83, .max = MDFLD_DSIPLL_M_MAX_83},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_83, .max = MDFLD_DSIPLL_P1_MAX_83},
	 },
	{			/* MDFLD_LIMT_DSIPLL_100 */
	 .dot = {.min = MDFLD_DOT_MIN, .max = MDFLD_DOT_MAX},
	 .m = {.min = MDFLD_DSIPLL_M_MIN_100, .max = MDFLD_DSIPLL_M_MAX_100},
	 .p1 = {.min = MDFLD_DSIPLL_P1_MIN_100, .max = MDFLD_DSIPLL_P1_MAX_100},
	 },
};

#define MDFLD_M_MIN	    21
#define MDFLD_M_MAX	    180
static const u32 mdfld_m_converts[] = {
/* M configuration table from 9-bit LFSR table */
	224, 368, 440, 220, 366, 439, 219, 365, 182, 347, /* 21 - 30 */
	173, 342, 171, 85, 298, 149, 74, 37, 18, 265,   /* 31 - 40 */
	388, 194, 353, 432, 216, 108, 310, 155, 333, 166, /* 41 - 50 */
	83, 41, 276, 138, 325, 162, 337, 168, 340, 170, /* 51 - 60 */
	341, 426, 469, 234, 373, 442, 221, 110, 311, 411, /* 61 - 70 */
	461, 486, 243, 377, 188, 350, 175, 343, 427, 213, /* 71 - 80 */
	106, 53, 282, 397, 354, 227, 113, 56, 284, 142, /* 81 - 90 */
	71, 35, 273, 136, 324, 418, 465, 488, 500, 506, /* 91 - 100 */
	253, 126, 63, 287, 399, 455, 483, 241, 376, 444, /* 101 - 110 */
	478, 495, 503, 251, 381, 446, 479, 239, 375, 443, /* 111 - 120 */
	477, 238, 119, 315, 157, 78, 295, 147, 329, 420, /* 121 - 130 */
	210, 105, 308, 154, 77, 38, 275, 137, 68, 290, /* 131 - 140 */
	145, 328, 164, 82, 297, 404, 458, 485, 498, 249, /* 141 - 150 */
	380, 190, 351, 431, 471, 235, 117, 314, 413, 206, /* 151 - 160 */
	103, 51, 25, 12, 262, 387, 193, 96, 48, 280, /* 161 - 170 */
	396, 198, 99, 305, 152, 76, 294, 403, 457, 228, /* 171 - 180 */
};

/**
 * Returns whether any output on the specified pipe is of the specified type
 */
static bool psb_intel_pipe_has_type(struct drm_crtc *crtc, int type)
{
	struct drm_device *dev = crtc->dev;
	struct drm_mode_config *mode_config = &dev->mode_config;
	struct drm_connector *l_entry;

	list_for_each_entry(l_entry, &mode_config->connector_list, head) {
		if (l_entry->encoder && l_entry->encoder->crtc == crtc) {
			struct psb_intel_output *psb_intel_output =
			    to_psb_intel_output(l_entry);
			if (psb_intel_output->type == type)
				return true;
		}
	}
	return false;
}

static const struct mrst_limit_t *mdfld_limit(struct drm_crtc *crtc)
{
	const struct mrst_limit_t *limit = NULL;
	struct drm_device *dev = crtc->dev;
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;

	if (psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_MIPI)
	    || psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_MIPI2)) {
		if ((dev_priv->ksel == KSEL_CRYSTAL_19) ||
				(dev_priv->ksel == KSEL_BYPASS_19))
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_19];
		else if (dev_priv->ksel == KSEL_BYPASS_25)
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_25];
		else if (dev_priv->ksel == KSEL_CRYSTAL_38)
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_38];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
				(dev_priv->core_freq == 166))
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_83];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
			 (dev_priv->core_freq == 100 ||
			  dev_priv->core_freq == 200))
			limit = &mdfld_limits[MDFLD_LIMT_DSIPLL_100];
	} else if (psb_intel_pipe_has_type(crtc, INTEL_OUTPUT_HDMI)) {
		if ((dev_priv->ksel == KSEL_CRYSTAL_19) ||
				(dev_priv->ksel == KSEL_BYPASS_19))
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_19];
		else if (dev_priv->ksel == KSEL_BYPASS_25)
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_25];
		else if (dev_priv->ksel == KSEL_CRYSTAL_38)
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_38];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
				(dev_priv->core_freq == 166))
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_83];
		else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
			 (dev_priv->core_freq == 100 ||
			  dev_priv->core_freq == 200))
			limit = &mdfld_limits[MDFLD_LIMT_DPLL_100];
	} else {
		limit = NULL;
		PSB_DEBUG_ENTRY("mdfld_limit Wrong display type.\n");
	}

	return limit;
}

/** Derive the pixel clock for the given refclk and divisors for 8xx chips. */
static void mdfld_clock(int refclk, struct mrst_clock_t *clock)
{
	clock->dot = (refclk * clock->m) / clock->p1;
}

/** Derive the vco clock for the given refclk and divisors for 8xx chips. */
static int mdfld_vco_clock(int refclk, struct mrst_clock_t *clock)
{
	return refclk * clock->m;
}

/**
 * Returns a set of divisors for the desired target clock with the given refclk,
 * or FALSE.  Divisor values are the actual divisors for
 */
static bool
mdfldFindBestPLL(struct drm_crtc *crtc, int target, int refclk,
		struct mrst_clock_t *best_clock)
{
	struct mrst_clock_t clock;
	const struct mrst_limit_t *limit = mdfld_limit(crtc);
	int err = target;
	int vco_clock;

	memset(best_clock, 0, sizeof(*best_clock));
	if (!limit) {
		DRM_ERROR("Invalid limit\n");
		return false;
	}

	PSB_DEBUG_ENTRY("%s: target = %d, m = [%d, %d], p = [%d, %d].\n",
			__func__, target, limit->m.min, limit->m.max,
			limit->p1.min, limit->p1.max);

	for (clock.m = limit->m.min; clock.m <= limit->m.max; clock.m++) {
		for (clock.p1 = limit->p1.min; clock.p1 <= limit->p1.max;
		     clock.p1++) {
			int this_err;

			mdfld_clock(refclk, &clock);
			vco_clock = mdfld_vco_clock(refclk, &clock);

			this_err = abs(clock.dot - target);
			if (this_err <= err && vco_clock < VCO_MAX) {
				*best_clock = clock;
				err = this_err;
			}
		}
	}
	PSB_DEBUG_ENTRY("mdfldFindBestPLL target = %d, m = %d, p = %d.\n",
			target, best_clock->m, best_clock->p1);
	PSB_DEBUG_ENTRY("mdfldFindBestPLL err = %d.\n", err);

	return err != target;
}

static int mdfld_crtc_dsi_pll_calc(struct drm_crtc *crtc,
				struct mdfld_dsi_config *dsi_config,
				struct drm_device *dev,
				u32 *out_dpll,
				u32 *out_fp,
				struct drm_display_mode *adjusted_mode)
{
	DRM_DRIVER_PRIVATE_T *dev_priv = dev->dev_private;
	struct mrst_clock_t clock;
	u32 dpll = 0, fp = 0;
	int refclk = 0;
	int clk_n = 0, clk_p2 = 0, clk_byte = 1, clk = 0,
	    m_conv = 0, clk_tmp = 0;
	bool ok;

	if ((dev_priv->ksel == KSEL_CRYSTAL_19) ||
			(dev_priv->ksel == KSEL_BYPASS_19)) {
		refclk = 19200;
		clk_n = 1, clk_p2 = 8;
	} else if (dev_priv->ksel == KSEL_BYPASS_25) {
		refclk = 25000;
		clk_n = 1, clk_p2 = 8;
	} else if (dev_priv->ksel == KSEL_CRYSTAL_38) {
		refclk = 38400;
		clk_n = 1, clk_p2 = 8;
	} else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
			(dev_priv->core_freq == 166)) {
		refclk = 83000;
		clk_n = 4, clk_p2 = 8;
	} else if ((dev_priv->ksel == KSEL_BYPASS_83_100) &&
		   (dev_priv->core_freq == 100 ||
		    dev_priv->core_freq == 200)) {
		refclk = 100000;
		clk_n = 4, clk_p2 = 8;
	} else {
		refclk = 19200;
		clk_n = 1, clk_p2 = 8;
	}

	dev_priv->bpp = 24;
	clk_byte = dev_priv->bpp / 8;

	if (dsi_config->lane_count)
		clk = adjusted_mode->clock / dsi_config->lane_count;
	else
		clk = adjusted_mode->clock;

	clk_tmp = clk * clk_n * clk_p2 * clk_byte;

	PSB_DEBUG_ENTRY("ref_clk: %d, clk = %d, clk_n = %d, clk_p2 = %d.\n",
			refclk, clk, clk_n, clk_p2);
	PSB_DEBUG_ENTRY("adjusted_mode->clock = %d, clk_tmp = %d.\n",
			adjusted_mode->clock, clk_tmp);

	ok = mdfldFindBestPLL(crtc, clk_tmp, refclk, &clock);
	dev_priv->tmds_clock_khz = clock.dot / (clk_n * clk_p2 * clk_byte);

	if (!ok)
		DRM_ERROR("mdfldFindBestPLL fail in mdfld_crtc_mode_set.\n");
	else {
		m_conv = mdfld_m_converts[(clock.m - MDFLD_M_MIN)];
		PSB_DEBUG_ENTRY("dot clock: %d, m: %d, p1: %d, m_conv: %d.\n",
				clock.dot, clock.m, clock.p1, m_conv);
	}

	dpll = 0x00000000;
	fp = (clk_n / 2) << 16;
	fp |= m_conv;

	/* compute bitmask from p1 value */
	dpll |= (1 << (clock.p1 - 2)) << 17;

	*(out_dpll) = dpll;
	*(out_fp) = fp;

	PSB_DEBUG_ENTRY("dsi dpll = 0x%x  fp = 0x%x\n", dpll, fp);
	return 0;
}

static int mdfld_crtc_dsi_mode_set(struct drm_crtc *crtc,
				struct mdfld_dsi_config *dsi_config,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode,
				int x, int y,
				struct drm_framebuffer *old_fb)
{
	struct drm_device *dev;
	struct psb_intel_crtc *mdfld_dsi_crtc;
	struct psb_framebuffer *mdfld_fb;
	struct psb_intel_mode_device *mode_dev;
	struct mdfld_dsi_hw_context *ctx;
	struct drm_psb_private *dev_priv;
	int fb_bpp;
	int fb_pitch;
	int fb_depth;
	int hdelay;
	static int init_flag = 1;   /*bootstrap flag*/

	if (!crtc || !crtc->fb) {
		DRM_ERROR("Invalid CRTC\n");
		return -EINVAL;
	}

	if (!dsi_config) {
		DRM_ERROR("Invalid DSI config\n");
		return -EINVAL;
	}

	mdfld_dsi_crtc = to_psb_intel_crtc(crtc);
	mdfld_fb = to_psb_fb(crtc->fb);
	mode_dev = mdfld_dsi_crtc->mode_dev;
	mode = adjusted_mode;
	ctx = &dsi_config->dsi_hw_context;
	fb_bpp = crtc->fb->bits_per_pixel;
	fb_pitch = crtc->fb->pitches[0];
	fb_depth = crtc->fb->depth;
	dev = crtc->dev;
	dev_priv = (struct drm_psb_private *)dev->dev_private;

	PSB_DEBUG_ENTRY("mode %dx%d, bpp %d, pitch %d", mode->crtc_hdisplay,
			mode->crtc_vdisplay, fb_bpp, fb_pitch);

	mutex_lock(&dsi_config->context_lock);

	ctx->vgacntr = 0x80000000;

	/*setup pll*/
	mdfld_crtc_dsi_pll_calc(crtc, dsi_config, dev,
				 &ctx->dpll,
				 &ctx->fp,
				 adjusted_mode);

	/*set up pipe timings*/
	ctx->htotal = (mode->crtc_hdisplay - 1) |
		((mode->crtc_htotal - 1) << 16);
	ctx->hblank = (mode->crtc_hblank_start - 1) |
		((mode->crtc_hblank_end - 1) << 16);
	ctx->hsync = (mode->crtc_hsync_start - 1) |
		((mode->crtc_hsync_end - 1) << 16);
	ctx->vtotal = (mode->crtc_vdisplay - 1) |
		((mode->crtc_vtotal - 1) << 16);
	ctx->vblank = (mode->crtc_vblank_start - 1) |
		((mode->crtc_vblank_end - 1) << 16);
	ctx->vsync = (mode->crtc_vsync_start - 1) |
		((mode->crtc_vsync_end - 1) << 16);

	/*pipe source*/
	ctx->pipesrc = ((mode->crtc_hdisplay - 1) << 16) |
		(mode->crtc_vdisplay - 1);

	/*setup dsp plane*/
	ctx->dsppos = 0;
	/* PR2 panel has 200 pixel dummy clocks,
	* So the display timing should be 800x1024, and surface
	* is 608x1024(64 bits align), then the information between android
	* and Linux frame buffer is not consistent.
	*/
//ASUS_BSP: [DDS] +++
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH
	if (0)
		ctx->dspsize = (800 - 1) | ((1280 - 1) << 16);
	else
		ctx->dspsize = (480 - 1) | ((854 - 1) << 16);
#else
	if (is_tmd_6x10_panel(dev, 0))
		ctx->dspsize = ((mode->crtc_vdisplay - 1) << 16) |
			(mode->crtc_hdisplay - 200  - 1);
	else
		ctx->dspsize = ((mode->crtc_vdisplay - 1) << 16) |
			(mode->crtc_hdisplay - 1);
#endif
//ASUS_BSP: [DDS] ---

	ctx->dspstride = fb_pitch;
	ctx->dspsurf = mode_dev->bo_offset(dev, mdfld_fb);
	ctx->dsplinoff = y * fb_pitch + x * (fb_bpp / 8);

	if (init_flag == 1) {
		printk(KERN_DEBUG"%s: dspsurf = 0x%x, dsplinoff = 0x%x\n",
				__func__, ctx->dsplinoff, ctx->dspsurf);
		dev_priv->init_screen_start = ctx->dspsurf;
		dev_priv->init_screen_offset = ctx->dsplinoff;
		dev_priv->init_screen_size = ctx->dspsize;
		dev_priv->init_screen_stride = ctx->dspstride;
		init_flag = 0;
	}

	ctx->dspcntr &= ~DISPPLANE_PIXFORMAT_MASK;
	switch (fb_bpp) {
	case 8:
		ctx->dspcntr |= DISPPLANE_8BPP;
		break;
	case 16:
		if (fb_depth == 15)
			ctx->dspcntr |= DISPPLANE_15_16BPP;
		else
			ctx->dspcntr |= DISPPLANE_16BPP;
		break;
	case 24:
	case 32:
		ctx->dspcntr |= DISPPLANE_32BPP_NO_ALPHA;
		break;
	default:
		DRM_ERROR("Unknown color depth\n");
		mutex_unlock(&dsi_config->context_lock);
		return -EINVAL;
	}

	if (dsi_config->pipe == 2)
		ctx->dspcntr |= (0x2 << 24);

	/*
	 * Setup pipe configuration for different panels
	 * The formula recommended from hw team is as below:
	 * (htotal * 5ns * hdelay) >= 8000ns
	 * hdelay is the count of delayed HBLANK scan lines
	 * And the max hdelay is 4
	 * by programming of PIPE(A/C) CONF bit 28:27:
	 * 00 = 1 scan line, 01 = 2 scan line,
	 * 02 = 3 scan line, 03 = 4 scan line
	 */
	ctx->pipeconf &= ~(BIT27 | BIT28);

	hdelay = 8000 / mode->crtc_htotal / 5;
	if (8000 % (mode->crtc_htotal * 5) > 0)
		hdelay += 1;

	if (hdelay > 4) {
		DRM_ERROR("Do not support such panel setting yet\n");
		hdelay = 4; /* Use the max hdelay instead*/
	}

	/*
	* Overlay issues DMA to load register buffer between vblank start
	* and frame start. If this can't be finished before frame start,
	* overlay will crash.
	* Maxmizing frame start delay to 4 scan lines to minimize overlay
	* crash.
	*/
	hdelay = 4;

	ctx->pipeconf |= ((hdelay - 1) << 27);

	mutex_unlock(&dsi_config->context_lock);
	return 0;
}

static void psb_intel_crtc_prepare(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_OFF);
}

static void psb_intel_crtc_commit(struct drm_crtc *crtc)
{
	struct drm_crtc_helper_funcs *crtc_funcs = crtc->helper_private;
	crtc_funcs->dpms(crtc, DRM_MODE_DPMS_ON);
}

/**
 * Return the pipe currently connected to the panel fitter,
 * or -1 if the panel fitter is not present or not in use
 */
int psb_intel_panel_fitter_pipe(struct drm_device *dev)
{
	u32 pfit_control;

	/* i830 doesn't have a panel fitter */
	if (IS_I830(dev))
		return -1;

	pfit_control = REG_READ(PFIT_CONTROL);

	/* See if the panel fitter is in use */
	if ((pfit_control & PFIT_ENABLE) == 0)
		return -1;

	/* 965 can place panel fitter on either pipe */
	if (IS_I965G(dev) || IS_MID(dev))
		return (pfit_control >> 29) & 0x3;

	/* older chips can only use pipe 1 */
	return 1;
}

static int mdfld_crtc_mode_set(struct drm_crtc *crtc,
				struct drm_display_mode *mode,
				struct drm_display_mode *adjusted_mode,
				int x, int y,
				struct drm_framebuffer *old_fb)
{
	struct drm_device *dev = crtc->dev;
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	struct mdfld_dsi_config *dsi_config = NULL;
	int pipe = psb_intel_crtc->pipe;

	PSB_DEBUG_ENTRY("pipe = 0x%x\n", pipe);

	if (pipe != 1) {
		if (pipe == 0)
			dsi_config = dev_priv->dsi_configs[0];
		else if (pipe == 2)
			dsi_config = dev_priv->dsi_configs[1];

		return mdfld_crtc_dsi_mode_set(crtc, dsi_config, mode,
				adjusted_mode, x, y, old_fb);
	 } else {
		 android_hdmi_crtc_mode_set(crtc, mode, adjusted_mode,
				 x, y, old_fb);

		 return 0;
	}
}


static const struct drm_crtc_funcs mdfld_intel_crtc_funcs = {
	.save = NULL,
	.restore = NULL,
	.cursor_set = mdfld_intel_crtc_cursor_set,
	.cursor_move = mdfld_intel_crtc_cursor_move,
	.gamma_set = psb_intel_crtc_gamma_set,
	.set_config = drm_crtc_helper_set_config,
	.destroy = psb_intel_crtc_destroy,
};

static const struct drm_crtc_helper_funcs mdfld_helper_funcs = {
	.dpms = mdfld_crtc_dpms,
	.mode_fixup = psb_intel_crtc_mode_fixup,
	.mode_set = mdfld_crtc_mode_set,
	.mode_set_base = mdfld__intel_pipe_set_base,
	.prepare = psb_intel_crtc_prepare,
	.commit = psb_intel_crtc_commit,
};
