/*
 * Copyright © 2011 Intel Corporation
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
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 *
 * Authors:
 *   Jesse Barnes <jbarnes@virtuousgeek.org>
 *
 * New plane/sprite handling.
 *
 * The older chips had a separate interface for programming plane related
 * registers; newer ones are much simpler and we can use the new DRM plane
 * support.
 */
#include <drm/drmP.h>
#include <drm/drm_crtc.h>
#include <drm/drm_fourcc.h>
#include <drm/drm_rect.h>
#include "intel_drv.h"
#include <drm/i915_drm.h>
#include "i915_drv.h"

void
__alpha_setting_noncursor(u32 pixformat, int plane, u32 *dspcntr, int alpha)
{
	/* For readability, can split to individual cases */
	/* 5 no alphas, 6-9 common, a-d reserved for sprite, e-f common */
	switch (pixformat) {
	case DISPPLANE_RGBA888:
		if (alpha)
			*dspcntr |= DISPPLANE_RGBA888;
		else
			*dspcntr |= DISPPLANE_RGBX888;
		break;
	case DISPPLANE_BGRA888:
		if (alpha)
			*dspcntr |= DISPPLANE_BGRA888;
		else
			*dspcntr |= DISPPLANE_BGRX888;
		break;
	case DISPPLANE_RGBA101010:
		if (alpha)
			*dspcntr |= DISPPLANE_RGBA101010;
		else
			*dspcntr |= DISPPLANE_RGBX101010;
		break;
	case DISPPLANE_BGRA101010:
		if (alpha)
			*dspcntr |= DISPPLANE_BGRA101010;
		else
			*dspcntr |= DISPPLANE_BGRX101010;
		break;
	case DISPPLANE_RGBA161616:
		if ((plane == PLANEA) || (plane == PLANEB)) {
			if (alpha)
				*dspcntr |= DISPPLANE_RGBA161616;
			else
				*dspcntr |= DISPPLANE_RGBX161616;
		}
		break;
	case DISPPLANE_RGBX888:
	case DISPPLANE_BGRX888:
	case DISPPLANE_RGBX101010:
	case DISPPLANE_BGRX101010:
	case DISPPLANE_RGBX161616:
	default:
		DRM_DEBUG("Alpha not supported for 0x%08x\n", pixformat);
		break;
	}
}

void
__alpha_setting_cursor(u32 pixformat, int plane, u32 *dspcntr, int alpha)
{
	/* For readability, can split to individual cases */
	switch (pixformat) {
	case CURSOR_MODE_128_32B_AX:
	case CURSOR_MODE_128_ARGB_AX:
		if (alpha)
			*dspcntr |= CURSOR_MODE_128_ARGB_AX;
		else
			*dspcntr |= CURSOR_MODE_128_32B_AX;
		break;

	case CURSOR_MODE_256_ARGB_AX:
	case CURSOR_MODE_256_32B_AX:
		if (alpha)
			*dspcntr |= CURSOR_MODE_256_ARGB_AX;
		else
			*dspcntr |= CURSOR_MODE_256_32B_AX;
		break;

	case CURSOR_MODE_64_ARGB_AX:
	case CURSOR_MODE_64_32B_AX:
		if (alpha)
			*dspcntr |= CURSOR_MODE_64_ARGB_AX;
		else
			*dspcntr |= CURSOR_MODE_64_32B_AX;
		break;
	default:
		DRM_DEBUG("Alpha not supported for 0x%08x\n", pixformat);
		break;
	}
}

/*
 * enable/disable alpha for planes
 */
int
i915_set_plane_alpha(struct drm_device *dev, void *data, struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct drm_i915_set_plane_alpha *alphadata = data;
	int plane = alphadata->plane;
	bool alpha = alphadata->alpha;
	bool IsCursor = false;
	u32 dspcntr;
	u32 reg;
	u32 pixformat;
	u32 mask = DISPPLANE_PIXFORMAT_MASK;

	DRM_DEBUG_DRIVER("In i915_set_plane_alpha\n");

	switch (plane) {
	case PLANEA:
		reg = DSPCNTR(0);
		break;
	case PLANEB:
		reg = DSPCNTR(1);
		break;
	case SPRITEA:
		reg = SPCNTR(0, 0);
		break;
	case SPRITEB:
		reg = SPCNTR(0, 1);
		break;
	case SPRITEC:
		reg = SPCNTR(1, 0);
		break;
	case SPRITED:
		reg = SPCNTR(1, 1);
		break;
	case CURSORA:
		reg = CURCNTR(0);
		mask = CURSOR_MODE;
		IsCursor = true;
		break;
	case CURSORB:
		reg = CURCNTR(1);
		mask = CURSOR_MODE;
		IsCursor = true;
		break;
	default:
		DRM_ERROR("No plane selected properly\n");
		return -EINVAL;
	}

	dspcntr = I915_READ(reg);
	DRM_DEBUG_DRIVER("dspcntr = %x\n", dspcntr);

	pixformat = dspcntr & mask;
	dspcntr &= ~mask;
	DRM_DEBUG_DRIVER("pixformat = %x, alpha = %x\n", pixformat, alpha);

	if (pixformat) {
		if (!IsCursor)
			__alpha_setting_noncursor(pixformat, plane,
						&dspcntr, alpha);
		else
			__alpha_setting_cursor(pixformat, plane,
						&dspcntr, alpha);

		DRM_DEBUG_DRIVER("Reg should be written with = %x\n", dspcntr);

		if (pixformat != (dspcntr & mask)) {
			I915_WRITE(reg, dspcntr);
			DRM_DEBUG_DRIVER("Reg written with = %x\n", dspcntr);
		}
	} else
		DRM_DEBUG_DRIVER("Plane might not be enabled/configured!\n");

	return 0;
}

int i915_set_plane_zorder(struct drm_device *dev, void *data,
			  struct drm_file *file)
{
	struct drm_i915_private *dev_priv = dev->dev_private;
	u32 val = 0;
	struct drm_i915_set_plane_zorder *zorder = data;
	u32 order = zorder->order;
	int s1_zorder, s1_bottom, s2_zorder, s2_bottom;
	int pipe = (order >> 31) & 0x1;
	struct intel_crtc *intel_crtc =
			to_intel_crtc(dev_priv->plane_to_crtc_mapping[pipe]);

	s1_zorder = (order >> 3) & 0x1;
	s1_bottom = (order >> 2) & 0x1;
	s2_zorder = (order >> 1) & 0x1;
	s2_bottom = (order >> 0) & 0x1;

	if (dev_priv->atomic_update)
		goto calc_zorder;
	/* Clear the older Z-order */
	val = I915_READ(SPCNTR(pipe, 0));
	/*
	 * Re-Visit: Disable maxfifo when we are moving from a single plane
	 * scenario to a multiple plane. Before even enabling plane or the
	 * z-order maxfifo should be disabled.
	 */
	if (dev_priv->maxfifo_enabled && !(val & SPRITE_ZORDER_ENABLE)) {
		I915_WRITE(FW_BLC_SELF_VLV, ~FW_CSPWRDWNEN);
		dev_priv->maxfifo_enabled = false;
		intel_wait_for_vblank(dev, pipe);
	}
	val &= ~(SPRITE_FORCE_BOTTOM | SPRITE_ZORDER_ENABLE);
	I915_WRITE(SPCNTR(pipe, 0), val);

	val = I915_READ(SPCNTR(pipe, 1));
	/*
	 * Re-Visit: Disable maxfifo when we are moving from a single plane
	 * scenario to a multiple plane. Before even enabling plane or the
	 * z-order maxfifo should be disabled.
	 */
	if (dev_priv->maxfifo_enabled && !(val & SPRITE_ZORDER_ENABLE)) {
		I915_WRITE(FW_BLC_SELF_VLV, ~FW_CSPWRDWNEN);
		dev_priv->maxfifo_enabled = false;
		intel_wait_for_vblank(dev, pipe);
	}
	val &= ~(SPRITE_FORCE_BOTTOM | SPRITE_ZORDER_ENABLE);
	I915_WRITE(SPCNTR(pipe, 1), val);

calc_zorder:
	/* Program new Z-order */
	if (!dev_priv->atomic_update)
		val = I915_READ(SPCNTR(pipe, 0));
	if (s1_zorder)
		val |= SPRITE_ZORDER_ENABLE;
	if (s1_bottom)
		val |= SPRITE_FORCE_BOTTOM;
	if (dev_priv->atomic_update)
		intel_crtc->reg.spacntr = val;
	else
		I915_WRITE(SPCNTR(pipe, 0), val);

	if (dev_priv->atomic_update)
		val = 0;
	else
		val = I915_READ(SPCNTR(pipe, 1));
	if (s2_zorder)
		val |= SPRITE_ZORDER_ENABLE;
	if (s2_bottom)
		val |= SPRITE_FORCE_BOTTOM;
	if (dev_priv->atomic_update)
		intel_crtc->reg.spbcntr = val;
	else
		I915_WRITE(SPCNTR(pipe, 1), val);

	return 0;
}

static void
vlv_update_plane(struct drm_plane *dplane, struct drm_crtc *crtc,
		 struct drm_framebuffer *fb,
		 struct drm_i915_gem_object *obj, int crtc_x, int crtc_y,
		 unsigned int crtc_w, unsigned int crtc_h,
		 uint32_t x, uint32_t y,
		 uint32_t src_w, uint32_t src_h,
		 struct drm_pending_vblank_event *event)
{
	struct drm_device *dev = dplane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(dplane);
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct drm_display_mode *mode = &intel_crtc->config.requested_mode;
	unsigned long sprsurf_offset, linear_offset;
	int pipe = intel_plane->pipe;
	int plane = intel_plane->plane;
	int sprite_ddl, prec_multi, sp_prec_multi;
	int pixel_size = drm_format_plane_cpp(fb->pixel_format, 0);
	u32 sprctl;
	u32 mask, shift;
	bool rotate = false;

	sprctl = I915_READ(SPCNTR(pipe, plane));
	/* Mask out pixel format bits in case we change it */
	sprctl &= ~SP_PIXFORMAT_MASK;
	sprctl &= ~SP_YUV_BYTE_ORDER_MASK;
	sprctl &= ~SP_TILED;

	switch (fb->pixel_format) {
	case DRM_FORMAT_YUYV:
		sprctl |= SP_FORMAT_YUV422 | SP_YUV_ORDER_YUYV;
		break;
	case DRM_FORMAT_YVYU:
		sprctl |= SP_FORMAT_YUV422 | SP_YUV_ORDER_YVYU;
		break;
	case DRM_FORMAT_UYVY:
		sprctl |= SP_FORMAT_YUV422 | SP_YUV_ORDER_UYVY;
		break;
	case DRM_FORMAT_VYUY:
		sprctl |= SP_FORMAT_YUV422 | SP_YUV_ORDER_VYUY;
		break;
	case DRM_FORMAT_RGB565:
		sprctl |= SP_FORMAT_BGR565;
		break;
	case DRM_FORMAT_XRGB8888:
		sprctl |= SP_FORMAT_BGRX8888;
		break;
	case DRM_FORMAT_ARGB8888:
		if (!intel_plane->alpha)
			sprctl |= SP_FORMAT_BGRX8888;
		else
			sprctl |= SP_FORMAT_BGRA8888;
		break;
	case DRM_FORMAT_XBGR2101010:
		sprctl |= SP_FORMAT_RGBX1010102;
		break;
	case DRM_FORMAT_ABGR2101010:
		if (!intel_plane->alpha)
			sprctl |= SP_FORMAT_RGBX1010102;
		else
			sprctl |= SP_FORMAT_RGBA1010102;
		break;
	case DRM_FORMAT_XBGR8888:
		sprctl |= SP_FORMAT_RGBX8888;
		break;
	case DRM_FORMAT_ABGR8888:
		if (!intel_plane->alpha)
			sprctl |= SP_FORMAT_RGBX8888;
		else
			sprctl |= SP_FORMAT_RGBA8888;
		break;
	default:
		/*
		 * If we get here one of the upper layers failed to filter
		 * out the unsupported plane formats
		 */
		BUG();
		break;
	}

	if (obj->tiling_mode != I915_TILING_NONE)
		sprctl |= SP_TILED;
	else
		sprctl &= ~SP_TILED;

	sprctl |= SP_ENABLE;

	/* disable current DRRS work scheduled and restart
	 * to push work by another x seconds
	 */
	intel_update_drrs(dev);

	if (!dev_priv->atomic_update)
		intel_update_sprite_watermarks(dplane, crtc, src_w, pixel_size,
				true, src_w != crtc_w || src_h != crtc_h);

	if (!intel_plane->rotate180 != !((dev_priv->vbt.is_180_rotation_enabled) &&
									(pipe == 0)))
		rotate = true;

	/* Sizes are 0 based */
	crtc_w--;
	crtc_h--;
	/*
	 * Disable Max Fifo configuration when sprite plane is enabled.
	 * Do not disable if Max Fifo is already disabled.
	 */
#ifdef ENABLE_MAXFIFO
	if (I915_READ(FW_BLC_SELF_VLV) & FW_CSPWRDWNEN) {
		I915_WRITE(FW_BLC_SELF_VLV,
			I915_READ(FW_BLC_SELF_VLV) & ~FW_CSPWRDWNEN);
	}
#endif
	/* if panel fitter is enabled program the input src size */
	if (intel_crtc->scaling_src_size &&
		(intel_crtc->config.gmch_pfit.control & PFIT_ENABLE)) {
		intel_plane->reg.pfit_control =
				intel_crtc->config.gmch_pfit.control;
		intel_plane->reg.pipesrc = intel_crtc->scaling_src_size;
		if (!dev_priv->atomic_update) {
			I915_WRITE(PFIT_CONTROL, intel_plane->reg.pfit_control);
			I915_WRITE(PIPESRC(pipe), intel_plane->reg.pipesrc);
			intel_crtc->pfit_en_status = true;
		}
	} else if (intel_crtc->pfit_en_status) {
		i9xx_get_pfit_mode(crtc, src_w, src_h);
		intel_plane->reg.pfit_control =
			intel_crtc->config.gmch_pfit.control;
		intel_plane->reg.pipesrc =
			((mode->hdisplay - 1) << SCALING_SRCSIZE_SHIFT) |
			(mode->vdisplay - 1);
		if (!dev_priv->atomic_update) {
			I915_WRITE(PIPESRC(pipe), intel_plane->reg.pipesrc);
			I915_WRITE(PFIT_CONTROL, intel_plane->reg.pfit_control);
			intel_crtc->pfit_en_status = false;
		}
	}

	intel_plane->reg.stride = fb->pitches[0];
	if (!dev_priv->atomic_update)
		I915_WRITE(SPSTRIDE(pipe, plane), intel_plane->reg.stride);
	if ((dev_priv->vbt.is_180_rotation_enabled) && (pipe == 0)) {
		uint32_t width = crtc->hwmode.hdisplay;
		uint32_t height = crtc->hwmode.vdisplay;

		if (intel_crtc->scaling_src_size && intel_crtc->config.gmch_pfit.control) {
			width = ((intel_crtc->scaling_src_size >>
				SCALING_SRCSIZE_SHIFT) &
				SCALING_SRCSIZE_MASK) + 1;
			height = (intel_crtc->scaling_src_size &
				SCALING_SRCSIZE_MASK) + 1;
		}

		intel_plane->reg.pos = ((height -
			(crtc_y + crtc_h + 1)) << 16) |
			(width - (crtc_x + crtc_w + 1));
	} else
		intel_plane->reg.pos = (crtc_y << 16) | crtc_x;
	if (!dev_priv->atomic_update)
			I915_WRITE(SPPOS(pipe, plane), intel_plane->reg.pos);

	linear_offset = y * fb->pitches[0] + x * pixel_size;
	sprsurf_offset = intel_gen4_compute_page_offset(&x, &y,
							obj->tiling_mode,
							pixel_size,
							fb->pitches[0]);
	linear_offset -= sprsurf_offset;

	if (obj->tiling_mode != I915_TILING_NONE) {
		if (rotate)
			intel_plane->reg.tileoff =
				((y + crtc_h) << 16) | (x + crtc_w);
		else
			intel_plane->reg.tileoff = (y << 16) | x;
		if (!dev_priv->atomic_update)
			I915_WRITE(SPTILEOFF(pipe, plane),
						intel_plane->reg.tileoff);
	} else {
		if (rotate)
			/* Linear Offset should be the difference b/w the last pixel of
			 * the last line of the display data in its unrotated orientation
			 * and the display surface address.
			 */
			intel_plane->reg.linoff = linear_offset +
					 crtc_h * fb->pitches[0] +
					 (crtc_w) * pixel_size;
		else
			intel_plane->reg.linoff = linear_offset;
		if (!dev_priv->atomic_update)
			I915_WRITE(SPLINOFF(pipe, plane),
					intel_plane->reg.linoff);
	}
	intel_plane->reg.size = (crtc_h << 16) | crtc_w;
	if (!dev_priv->atomic_update)
		I915_WRITE(SPSIZE(pipe, plane), intel_plane->reg.size);
	if (rotate)
		sprctl |= DISPPLANE_180_ROTATION_ENABLE;
	else
		sprctl &= ~DISPPLANE_180_ROTATION_ENABLE;


	/* When in maxfifo dspcntr cannot be changed */
	if (sprctl != I915_READ(SPCNTR(pipe, plane)) &&
				dev_priv->maxfifo_enabled &&
				dev_priv->atomic_update) {
		I915_WRITE(FW_BLC_SELF_VLV, ~FW_CSPWRDWNEN);
		dev_priv->maxfifo_enabled = false;
		dev_priv->wait_vbl = true;
		dev_priv->vblcount = atomic_read(
				&dev->_vblank_count[intel_crtc->pipe]);
	}
	/*
	 * calculate the DDL and set to 0 is there is a change. Else cache
	 * the value and wrrite on next vblank.
	 */
	if (intel_plane->plane == 0) {
		mask = 0x0000ff00;
		shift = DDL_SPRITEA_SHIFT;
	} else {
		mask = 0x00ff0000;
		shift = DDL_SPRITEB_SHIFT;
	}
	vlv_calculate_ddl(crtc, pixel_size, &prec_multi, &sprite_ddl);
	sp_prec_multi = (prec_multi ==
					DRAIN_LATENCY_PRECISION_32) ?
					DDL_PLANE_PRECISION_32 :
					DDL_PLANE_PRECISION_64;
	sprite_ddl = (sp_prec_multi | sprite_ddl) << shift;
	if (intel_plane->plane) {
		intel_crtc->reg_ddl.spriteb_ddl = sprite_ddl;
		intel_crtc->reg_ddl.spriteb_ddl_mask = mask;
	} else {
		intel_crtc->reg_ddl.spritea_ddl = sprite_ddl;
		intel_crtc->reg_ddl.spritea_ddl_mask = mask;
	}
	if ((sprite_ddl & mask) != (I915_READ(VLV_DDL(pipe)) & mask))
		I915_WRITE_BITS(VLV_DDL(pipe), 0x00, mask);

	intel_plane->reg.cntr = sprctl;
	intel_plane->reg.surf = I915_READ(SPSURF(pipe, plane));
	/* mask the surface base addr offset bits */
	intel_plane->reg.surf &= ~DISP_BASEADDR_MASK;
	intel_plane->reg.surf |= i915_gem_obj_ggtt_offset(obj) + sprsurf_offset;
	/* update the rrb2 bit status */
	if (intel_plane->rrb2_enable)
		intel_plane->reg.surf |= PLANE_RESERVED_REG_BIT_2_ENABLE;
	else
		intel_plane->reg.surf &= ~PLANE_RESERVED_REG_BIT_2_ENABLE;
	if (!dev_priv->atomic_update) {
		I915_WRITE(SPCNTR(pipe, plane), sprctl);
		I915_MODIFY_DISPBASE(SPSURF(pipe, plane), i915_gem_obj_ggtt_offset(obj) +
			     sprsurf_offset);
	}
	i915_update_plane_stat(dev_priv, pipe, plane, true, SPRITE_PLANE);

	if (event == NULL)
		POSTING_READ(SPSURF(pipe, plane));

	if (!dev_priv->atomic_update)
		intel_update_sprite_watermarks(dplane, crtc, src_w, pixel_size,
				true, src_w != crtc_w || src_h != crtc_h);
}

static void
vlv_disable_plane(struct drm_plane *dplane, struct drm_crtc *crtc)
{
	struct drm_device *dev = dplane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(dplane);
	int pipe = intel_plane->pipe;
	int plane = intel_plane->plane;
	u32 mask, shift;

	intel_plane->reg.cntr = I915_READ(SPCNTR(pipe, plane)) & ~SP_ENABLE;
	if (!dev_priv->atomic_update)
		I915_WRITE(SPCNTR(pipe, plane), I915_READ(SPCNTR(pipe, plane)) &
		   ~SP_ENABLE);
	i915_update_plane_stat(dev_priv, pipe, plane, false, SPRITE_PLANE);
	/*
	 * Check if Max Fifo configuration is required when sprite
	 * is disabled.
	 */
#ifdef ENABLE_MAXFIFO
	if (is_maxfifo_needed(dev_priv))
		I915_WRITE(FW_BLC_SELF_VLV, FW_CSPWRDWNEN);
#endif
	/* Activate double buffered register update */
	intel_plane->reg.surf = I915_READ(SPSURF(pipe, plane));
	/* mask the surface base addr offset bits */
	intel_plane->reg.surf &= ~DISP_BASEADDR_MASK;
	if (!dev_priv->atomic_update) {
		I915_MODIFY_DISPBASE(SPSURF(pipe, plane), 0);
		POSTING_READ(SPSURF(pipe, plane));
	}

	if (!dev_priv->atomic_update)
		intel_update_sprite_watermarks(dplane, crtc, 0, 0, false, false);
	intel_plane->last_plane_state = false; /* false means disabled */
	/* set to 0 as the plane is disabled */
	if (intel_plane->plane == 0) {
		mask = 0x0000ff00;
		shift = DDL_SPRITEA_SHIFT;
	} else {
		mask = 0x00ff0000;
		shift = DDL_SPRITEB_SHIFT;
	}
	I915_WRITE_BITS(VLV_DDL(pipe), 0x00, mask);
}

void intel_prepare_sprite_page_flip(struct drm_device *dev, int plane)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc =
		to_intel_crtc(dev_priv->plane_to_crtc_mapping[plane]);
	unsigned long flags;

	spin_lock_irqsave(&dev->event_lock, flags);

	if (intel_crtc->sprite_unpin_work) {
		atomic_inc(&intel_crtc->sprite_unpin_work->pending);
		if (atomic_read(&intel_crtc->sprite_unpin_work->pending) > 1)
			DRM_ERROR("Prepared flip multiple times\n");
	}

	spin_unlock_irqrestore(&dev->event_lock, flags);

}

void intel_finish_sprite_page_flip(struct drm_device *dev, int pipe)
{
	drm_i915_private_t *dev_priv = dev->dev_private;
	struct drm_crtc *crtc = dev_priv->pipe_to_crtc_mapping[pipe];
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_unpin_work *work;
	struct drm_i915_gem_object *obj;
	unsigned long flags;

	/* Ignore early vblank irqs */
	if (intel_crtc == NULL)
		return;

	/* Program the precalculated DDL value */
	if (intel_crtc->reg_ddl.spritea_ddl) {
		I915_WRITE_BITS(VLV_DDL(pipe), intel_crtc->reg_ddl.spritea_ddl,
			intel_crtc->reg_ddl.spritea_ddl_mask);
		intel_crtc->reg_ddl.spritea_ddl = 0;
	}
	if (intel_crtc->reg_ddl.spriteb_ddl) {
		I915_WRITE_BITS(VLV_DDL(pipe), intel_crtc->reg_ddl.spriteb_ddl,
			intel_crtc->reg_ddl.spriteb_ddl_mask);
		intel_crtc->reg_ddl.spriteb_ddl = 0;
	}

	spin_lock_irqsave(&dev->event_lock, flags);
	work = intel_crtc->sprite_unpin_work;

	if (work == NULL || !atomic_read(&work->pending)) {
		spin_unlock_irqrestore(&dev->event_lock, flags);
		return;
	}

	intel_crtc->sprite_unpin_work = NULL;

	if (intel_crtc->dummy_flip)
		intel_crtc->dummy_flip = false;
	else {
		if (work->event)
			drm_send_vblank_event(dev, intel_crtc->pipe, work->event);
	}

	drm_vblank_put(dev, intel_crtc->pipe);
	spin_unlock_irqrestore(&dev->event_lock, flags);

	if (work->old_fb_obj != NULL) {
		obj = work->old_fb_obj;

		atomic_clear_mask(1 << intel_crtc->plane,
			&obj->pending_flip.counter);

		if (atomic_read(&obj->pending_flip) == 0)
			wake_up_all(&dev_priv->pending_flip_queue);
	} else
		wake_up_all(&dev_priv->pending_flip_queue);

	queue_work(dev_priv->wq, &work->work);
	trace_i915_flip_complete(intel_crtc->plane, work->pending_flip_obj);
}

void intel_unpin_sprite_work_fn(struct work_struct *__work)
{
	struct intel_unpin_work *work =
			container_of(__work, struct intel_unpin_work, work);
	struct drm_device *dev = work->crtc->dev;
	mutex_lock(&dev->struct_mutex);
	if (work->old_fb_obj != NULL) {
		intel_unpin_fb_obj(work->old_fb_obj);
		drm_gem_object_unreference(&work->old_fb_obj->base);
	}
	mutex_unlock(&dev->struct_mutex);

	kfree(work);
}

static int
vlv_update_colorkey(struct drm_plane *dplane,
		    struct drm_intel_sprite_colorkey *key)
{
	struct drm_device *dev = dplane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(dplane);
	int pipe = intel_plane->pipe;
	int plane = intel_plane->plane;
	u32 sprctl;

	if (key->flags & I915_SET_COLORKEY_DESTINATION)
		return -EINVAL;

	I915_WRITE(SPKEYMINVAL(pipe, plane), key->min_value);
	I915_WRITE(SPKEYMAXVAL(pipe, plane), key->max_value);
	I915_WRITE(SPKEYMSK(pipe, plane), key->channel_mask);

	sprctl = I915_READ(SPCNTR(pipe, plane));
	sprctl &= ~SP_SOURCE_KEY;
	if (key->flags & I915_SET_COLORKEY_SOURCE)
		sprctl |= SP_SOURCE_KEY;
	I915_WRITE(SPCNTR(pipe, plane), sprctl);

	POSTING_READ(SPKEYMSK(pipe, plane));

	return 0;
}

static void
vlv_get_colorkey(struct drm_plane *dplane,
		 struct drm_intel_sprite_colorkey *key)
{
	struct drm_device *dev = dplane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(dplane);
	int pipe = intel_plane->pipe;
	int plane = intel_plane->plane;
	u32 sprctl;

	key->min_value = I915_READ(SPKEYMINVAL(pipe, plane));
	key->max_value = I915_READ(SPKEYMAXVAL(pipe, plane));
	key->channel_mask = I915_READ(SPKEYMSK(pipe, plane));

	sprctl = I915_READ(SPCNTR(pipe, plane));
	if (sprctl & SP_SOURCE_KEY)
		key->flags = I915_SET_COLORKEY_SOURCE;
	else
		key->flags = I915_SET_COLORKEY_NONE;
}

static void
ivb_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
		 struct drm_framebuffer *fb,
		 struct drm_i915_gem_object *obj, int crtc_x, int crtc_y,
		 unsigned int crtc_w, unsigned int crtc_h,
		 uint32_t x, uint32_t y,
		 uint32_t src_w, uint32_t src_h,
		 struct drm_pending_vblank_event *e)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(plane);
	int pipe = intel_plane->pipe;
	u32 sprctl, sprscale = 0;
	unsigned long sprsurf_offset, linear_offset;
	int pixel_size = drm_format_plane_cpp(fb->pixel_format, 0);
	bool scaling_was_enabled = dev_priv->sprite_scaling_enabled;

	sprctl = I915_READ(SPRCTL(pipe));

	/* Mask out pixel format bits in case we change it */
	sprctl &= ~SPRITE_PIXFORMAT_MASK;
	sprctl &= ~SPRITE_RGB_ORDER_RGBX;
	sprctl &= ~SPRITE_YUV_BYTE_ORDER_MASK;
	sprctl &= ~SPRITE_TILED;

	switch (fb->pixel_format) {
	case DRM_FORMAT_XBGR8888:
		sprctl |= SPRITE_FORMAT_RGBX888 | SPRITE_RGB_ORDER_RGBX;
		break;
	case DRM_FORMAT_XRGB8888:
		sprctl |= SPRITE_FORMAT_RGBX888;
		break;
	case DRM_FORMAT_YUYV:
		sprctl |= SPRITE_FORMAT_YUV422 | SPRITE_YUV_ORDER_YUYV;
		break;
	case DRM_FORMAT_YVYU:
		sprctl |= SPRITE_FORMAT_YUV422 | SPRITE_YUV_ORDER_YVYU;
		break;
	case DRM_FORMAT_UYVY:
		sprctl |= SPRITE_FORMAT_YUV422 | SPRITE_YUV_ORDER_UYVY;
		break;
	case DRM_FORMAT_VYUY:
		sprctl |= SPRITE_FORMAT_YUV422 | SPRITE_YUV_ORDER_VYUY;
		break;
	default:
		BUG();
	}

	if (obj->tiling_mode != I915_TILING_NONE)
		sprctl |= SPRITE_TILED;

	/* must disable */
	sprctl |= SPRITE_TRICKLE_FEED_DISABLE;
	sprctl |= SPRITE_ENABLE;

	if (IS_HASWELL(dev))
		sprctl |= SPRITE_PIPE_CSC_ENABLE;

	intel_update_sprite_watermarks(plane, crtc, src_w, pixel_size, true,
				       src_w != crtc_w || src_h != crtc_h);

	/* Sizes are 0 based */
	src_w--;
	src_h--;
	crtc_w--;
	crtc_h--;

	/*
	 * IVB workaround: must disable low power watermarks for at least
	 * one frame before enabling scaling.  LP watermarks can be re-enabled
	 * when scaling is disabled.
	 */
	if (crtc_w != src_w || crtc_h != src_h) {
		dev_priv->sprite_scaling_enabled |= 1 << pipe;

		if (!scaling_was_enabled) {
			intel_update_watermarks(dev);
			intel_wait_for_vblank(dev, pipe);
		}
		sprscale = SPRITE_SCALE_ENABLE | (src_w << 16) | src_h;
	} else
		dev_priv->sprite_scaling_enabled &= ~(1 << pipe);

	I915_WRITE(SPRSTRIDE(pipe), fb->pitches[0]);
	I915_WRITE(SPRPOS(pipe), (crtc_y << 16) | crtc_x);

	linear_offset = y * fb->pitches[0] + x * pixel_size;
	sprsurf_offset =
		intel_gen4_compute_page_offset(&x, &y, obj->tiling_mode,
					       pixel_size, fb->pitches[0]);
	linear_offset -= sprsurf_offset;

	/* HSW consolidates SPRTILEOFF and SPRLINOFF into a single SPROFFSET
	 * register */
	if (IS_HASWELL(dev))
		I915_WRITE(SPROFFSET(pipe), (y << 16) | x);
	else if (obj->tiling_mode != I915_TILING_NONE)
		I915_WRITE(SPRTILEOFF(pipe), (y << 16) | x);
	else
		I915_WRITE(SPRLINOFF(pipe), linear_offset);

	I915_WRITE(SPRSIZE(pipe), (crtc_h << 16) | crtc_w);
	if (intel_plane->can_scale)
		I915_WRITE(SPRSCALE(pipe), sprscale);
	I915_WRITE(SPRCTL(pipe), sprctl);
	I915_MODIFY_DISPBASE(SPRSURF(pipe),
			     i915_gem_obj_ggtt_offset(obj) + sprsurf_offset);
	POSTING_READ(SPRSURF(pipe));

	/* potentially re-enable LP watermarks */
	if (scaling_was_enabled && !dev_priv->sprite_scaling_enabled)
		intel_update_watermarks(dev);
}

static void
ivb_disable_plane(struct drm_plane *plane, struct drm_crtc *crtc)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(plane);
	int pipe = intel_plane->pipe;
	bool scaling_was_enabled = dev_priv->sprite_scaling_enabled;

	I915_WRITE(SPRCTL(pipe), I915_READ(SPRCTL(pipe)) & ~SPRITE_ENABLE);
	/* Can't leave the scaler enabled... */
	if (intel_plane->can_scale)
		I915_WRITE(SPRSCALE(pipe), 0);
	/* Activate double buffered register update */
	I915_MODIFY_DISPBASE(SPRSURF(pipe), 0);
	POSTING_READ(SPRSURF(pipe));

	dev_priv->sprite_scaling_enabled &= ~(1 << pipe);

	intel_update_sprite_watermarks(plane, crtc, 0, 0, false, false);

	/* potentially re-enable LP watermarks */
	if (scaling_was_enabled && !dev_priv->sprite_scaling_enabled)
		intel_update_watermarks(dev);
}

static int
ivb_update_colorkey(struct drm_plane *plane,
		    struct drm_intel_sprite_colorkey *key)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane;
	u32 sprctl;
	int ret = 0;

	intel_plane = to_intel_plane(plane);

	I915_WRITE(SPRKEYVAL(intel_plane->pipe), key->min_value);
	I915_WRITE(SPRKEYMAX(intel_plane->pipe), key->max_value);
	I915_WRITE(SPRKEYMSK(intel_plane->pipe), key->channel_mask);

	sprctl = I915_READ(SPRCTL(intel_plane->pipe));
	sprctl &= ~(SPRITE_SOURCE_KEY | SPRITE_DEST_KEY);
	if (key->flags & I915_SET_COLORKEY_DESTINATION)
		sprctl |= SPRITE_DEST_KEY;
	else if (key->flags & I915_SET_COLORKEY_SOURCE)
		sprctl |= SPRITE_SOURCE_KEY;
	I915_WRITE(SPRCTL(intel_plane->pipe), sprctl);

	POSTING_READ(SPRKEYMSK(intel_plane->pipe));

	return ret;
}

static void
ivb_get_colorkey(struct drm_plane *plane, struct drm_intel_sprite_colorkey *key)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane;
	u32 sprctl;

	intel_plane = to_intel_plane(plane);

	key->min_value = I915_READ(SPRKEYVAL(intel_plane->pipe));
	key->max_value = I915_READ(SPRKEYMAX(intel_plane->pipe));
	key->channel_mask = I915_READ(SPRKEYMSK(intel_plane->pipe));
	key->flags = 0;

	sprctl = I915_READ(SPRCTL(intel_plane->pipe));

	if (sprctl & SPRITE_DEST_KEY)
		key->flags = I915_SET_COLORKEY_DESTINATION;
	else if (sprctl & SPRITE_SOURCE_KEY)
		key->flags = I915_SET_COLORKEY_SOURCE;
	else
		key->flags = I915_SET_COLORKEY_NONE;
}

static void
ilk_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
		 struct drm_framebuffer *fb,
		 struct drm_i915_gem_object *obj, int crtc_x, int crtc_y,
		 unsigned int crtc_w, unsigned int crtc_h,
		 uint32_t x, uint32_t y,
		 uint32_t src_w, uint32_t src_h,
		 struct drm_pending_vblank_event *e)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(plane);
	int pipe = intel_plane->pipe;
	unsigned long dvssurf_offset, linear_offset;
	u32 dvscntr, dvsscale;
	int pixel_size = drm_format_plane_cpp(fb->pixel_format, 0);

	dvscntr = I915_READ(DVSCNTR(pipe));

	/* Mask out pixel format bits in case we change it */
	dvscntr &= ~DVS_PIXFORMAT_MASK;
	dvscntr &= ~DVS_RGB_ORDER_XBGR;
	dvscntr &= ~DVS_YUV_BYTE_ORDER_MASK;
	dvscntr &= ~DVS_TILED;

	switch (fb->pixel_format) {
	case DRM_FORMAT_XBGR8888:
		dvscntr |= DVS_FORMAT_RGBX888 | DVS_RGB_ORDER_XBGR;
		break;
	case DRM_FORMAT_XRGB8888:
		dvscntr |= DVS_FORMAT_RGBX888;
		break;
	case DRM_FORMAT_YUYV:
		dvscntr |= DVS_FORMAT_YUV422 | DVS_YUV_ORDER_YUYV;
		break;
	case DRM_FORMAT_YVYU:
		dvscntr |= DVS_FORMAT_YUV422 | DVS_YUV_ORDER_YVYU;
		break;
	case DRM_FORMAT_UYVY:
		dvscntr |= DVS_FORMAT_YUV422 | DVS_YUV_ORDER_UYVY;
		break;
	case DRM_FORMAT_VYUY:
		dvscntr |= DVS_FORMAT_YUV422 | DVS_YUV_ORDER_VYUY;
		break;
	default:
		BUG();
	}

	if (obj->tiling_mode != I915_TILING_NONE)
		dvscntr |= DVS_TILED;

	if (IS_GEN6(dev))
		dvscntr |= DVS_TRICKLE_FEED_DISABLE; /* must disable */
	dvscntr |= DVS_ENABLE;

	intel_update_sprite_watermarks(plane, crtc, src_w, pixel_size, true,
				       src_w != crtc_w || src_h != crtc_h);

	/* Sizes are 0 based */
	src_w--;
	src_h--;
	crtc_w--;
	crtc_h--;

	dvsscale = 0;
	if (IS_GEN5(dev) || crtc_w != src_w || crtc_h != src_h)
		dvsscale = DVS_SCALE_ENABLE | (src_w << 16) | src_h;

	I915_WRITE(DVSSTRIDE(pipe), fb->pitches[0]);
	I915_WRITE(DVSPOS(pipe), (crtc_y << 16) | crtc_x);

	linear_offset = y * fb->pitches[0] + x * pixel_size;
	dvssurf_offset =
		intel_gen4_compute_page_offset(&x, &y, obj->tiling_mode,
					       pixel_size, fb->pitches[0]);
	linear_offset -= dvssurf_offset;

	if (obj->tiling_mode != I915_TILING_NONE)
		I915_WRITE(DVSTILEOFF(pipe), (y << 16) | x);
	else
		I915_WRITE(DVSLINOFF(pipe), linear_offset);

	I915_WRITE(DVSSIZE(pipe), (crtc_h << 16) | crtc_w);
	I915_WRITE(DVSSCALE(pipe), dvsscale);
	I915_WRITE(DVSCNTR(pipe), dvscntr);
	I915_MODIFY_DISPBASE(DVSSURF(pipe),
			     i915_gem_obj_ggtt_offset(obj) + dvssurf_offset);
	POSTING_READ(DVSSURF(pipe));
}

static void
ilk_disable_plane(struct drm_plane *plane, struct drm_crtc *crtc)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(plane);
	int pipe = intel_plane->pipe;

	I915_WRITE(DVSCNTR(pipe), I915_READ(DVSCNTR(pipe)) & ~DVS_ENABLE);
	/* Disable the scaler */
	I915_WRITE(DVSSCALE(pipe), 0);
	/* Flush double buffered register updates */
	I915_MODIFY_DISPBASE(DVSSURF(pipe), 0);
	POSTING_READ(DVSSURF(pipe));

	intel_update_sprite_watermarks(plane, crtc, 0, 0, false, false);
}

static void
intel_enable_primary(struct drm_plane *dplane, struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_plane *intel_plane = to_intel_plane(dplane);
	int reg = DSPCNTR(intel_crtc->plane);
	int plane = intel_crtc->plane;
	int pipe = intel_plane->pipe;

	if (!intel_crtc->primary_disabled)
		return;

	intel_crtc->primary_disabled = false;
	intel_update_fbc(dev);
	intel_update_drrs(dev);

	intel_crtc->reg.cntr = I915_READ(reg) | DISPLAY_PLANE_ENABLE;
	intel_plane->reg.dspcntr = I915_READ(reg) | DISPLAY_PLANE_ENABLE;
	intel_crtc->pri_update = true;
	intel_plane->pri_update = true;
	if (!dev_priv->atomic_update)
		I915_WRITE(reg, I915_READ(reg) | DISPLAY_PLANE_ENABLE);
	i915_update_plane_stat(dev_priv, pipe, plane, true, DISPLAY_PLANE);
}

static void
intel_disable_primary(struct drm_plane *dplane, struct drm_crtc *crtc)
{
	struct drm_device *dev = crtc->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_plane *intel_plane = to_intel_plane(dplane);
	int plane = intel_crtc->plane;
	int pipe = intel_plane->pipe;
	int reg = DSPCNTR(plane);
	int mask = 0x000000ff;

	if (intel_crtc->primary_disabled)
		return;
	intel_crtc->reg.cntr = I915_READ(reg) & ~DISPLAY_PLANE_ENABLE;
	intel_plane->reg.dspcntr = I915_READ(reg) & ~DISPLAY_PLANE_ENABLE;
	intel_crtc->pri_update = true;
	intel_plane->pri_update = true;
	if (!dev_priv->atomic_update) {
		I915_WRITE(reg, I915_READ(reg) & ~DISPLAY_PLANE_ENABLE);
		I915_WRITE(DSPSURF(plane), I915_READ(DSPSURF(plane)));
	}
	i915_update_plane_stat(dev_priv, pipe, plane, false, DISPLAY_PLANE);
	I915_WRITE_BITS(VLV_DDL(pipe), 0x00, mask);
	intel_crtc->primary_disabled = true;
	intel_update_fbc(dev);
}

static int
ilk_update_colorkey(struct drm_plane *plane,
		    struct drm_intel_sprite_colorkey *key)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane;
	u32 dvscntr;
	int ret = 0;

	intel_plane = to_intel_plane(plane);

	I915_WRITE(DVSKEYVAL(intel_plane->pipe), key->min_value);
	I915_WRITE(DVSKEYMAX(intel_plane->pipe), key->max_value);
	I915_WRITE(DVSKEYMSK(intel_plane->pipe), key->channel_mask);

	dvscntr = I915_READ(DVSCNTR(intel_plane->pipe));
	dvscntr &= ~(DVS_SOURCE_KEY | DVS_DEST_KEY);
	if (key->flags & I915_SET_COLORKEY_DESTINATION)
		dvscntr |= DVS_DEST_KEY;
	else if (key->flags & I915_SET_COLORKEY_SOURCE)
		dvscntr |= DVS_SOURCE_KEY;
	I915_WRITE(DVSCNTR(intel_plane->pipe), dvscntr);

	POSTING_READ(DVSKEYMSK(intel_plane->pipe));

	return ret;
}

static void
ilk_get_colorkey(struct drm_plane *plane, struct drm_intel_sprite_colorkey *key)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane;
	u32 dvscntr;

	intel_plane = to_intel_plane(plane);

	key->min_value = I915_READ(DVSKEYVAL(intel_plane->pipe));
	key->max_value = I915_READ(DVSKEYMAX(intel_plane->pipe));
	key->channel_mask = I915_READ(DVSKEYMSK(intel_plane->pipe));
	key->flags = 0;

	dvscntr = I915_READ(DVSCNTR(intel_plane->pipe));

	if (dvscntr & DVS_DEST_KEY)
		key->flags = I915_SET_COLORKEY_DESTINATION;
	else if (dvscntr & DVS_SOURCE_KEY)
		key->flags = I915_SET_COLORKEY_SOURCE;
	else
		key->flags = I915_SET_COLORKEY_NONE;
}

static bool
format_is_yuv(uint32_t format)
{
	switch (format) {
	case DRM_FORMAT_YUYV:
	case DRM_FORMAT_UYVY:
	case DRM_FORMAT_VYUY:
	case DRM_FORMAT_YVYU:
		return true;
	default:
		return false;
	}
}

static int
intel_update_plane(struct drm_plane *plane, struct drm_crtc *crtc,
		   struct drm_framebuffer *fb, int crtc_x, int crtc_y,
		   unsigned int crtc_w, unsigned int crtc_h,
		   uint32_t src_x, uint32_t src_y,
		   uint32_t src_w, uint32_t src_h,
		   struct drm_pending_vblank_event *event)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_crtc *intel_crtc = to_intel_crtc(crtc);
	struct intel_plane *intel_plane = to_intel_plane(plane);
	struct intel_framebuffer *intel_fb;
	struct drm_i915_gem_object *obj, *old_obj;
	unsigned long flags;
	int pipe = intel_plane->pipe;
	enum transcoder cpu_transcoder = intel_pipe_to_cpu_transcoder(dev_priv,
								      pipe);
	int ret = 0;
	bool disable_primary = false;
	bool visible;
	int hscale, vscale;
	int max_scale, min_scale;
	int pixel_size = drm_format_plane_cpp(fb->pixel_format, 0);
	struct drm_rect src = {
		/* sample coordinates in 16.16 fixed point */
		.x1 = src_x,
		.x2 = src_x + src_w,
		.y1 = src_y,
		.y2 = src_y + src_h,
	};
	struct drm_rect dst = {
		/* integer pixels */
		.x1 = crtc_x,
		.x2 = crtc_x + crtc_w,
		.y1 = crtc_y,
		.y2 = crtc_y + crtc_h,
	};
	struct drm_rect clip = {
		.x2 = crtc->mode.hdisplay,
		.y2 = crtc->mode.vdisplay,
	};
	struct intel_unpin_work *work = NULL;

	/* Avoid update plane operation if shutdown is in progress */
	if (dev_priv->pm.shutdown_in_progress) {
		DRM_ERROR("shutdown in progress, cant proceed with flip req\n");
		return -EINVAL;
	}

	intel_fb = to_intel_framebuffer(fb);
	obj = intel_fb->obj;

	old_obj = intel_plane->old_obj;

	intel_plane->crtc_x = crtc_x;
	intel_plane->crtc_y = crtc_y;
	intel_plane->crtc_w = crtc_w;
	intel_plane->crtc_h = crtc_h;
	intel_plane->src_x = src_x;
	intel_plane->src_y = src_y;
	intel_plane->src_w = src_w;
	intel_plane->src_h = src_h;

	/* Pipe must be running... */
	if (!(I915_READ(PIPECONF(cpu_transcoder)) & PIPECONF_ENABLE)) {
		DRM_DEBUG_KMS("Pipe disabled\n");
		return -EINVAL;
	}

	/* Don't modify another pipe's plane */
	if (intel_plane->pipe != intel_crtc->pipe) {
		DRM_DEBUG_KMS("Wrong plane <-> crtc mapping\n");
		return -EINVAL;
	}

	/* FIXME check all gen limits */
	if (fb->width < 3 || fb->height < 3 || fb->pitches[0] > 16384) {
		DRM_DEBUG_KMS("Unsuitable framebuffer for plane\n");
		return -EINVAL;
	}

	/* Sprite planes can be linear or x-tiled surfaces */
	switch (obj->tiling_mode) {
		case I915_TILING_NONE:
		case I915_TILING_X:
			break;
		default:
			DRM_DEBUG_KMS("Unsupported tiling mode\n");
			return -EINVAL;
	}

	/*
	 * FIXME the following code does a bunch of fuzzy adjustments to the
	 * coordinates and sizes. We probably need some way to decide whether
	 * more strict checking should be done instead.*/

	max_scale = intel_plane->max_downscale << 16;
	min_scale = intel_plane->can_scale ? 1 : (1 << 16);

	if (IS_VALLEYVIEW(dev) && intel_crtc->scaling_src_size &&
		intel_crtc->config.gmch_pfit.control) {
		clip.x2 = ((intel_crtc->scaling_src_size >>
				SCALING_SRCSIZE_SHIFT) &
				SCALING_SRCSIZE_MASK) + 1;
		clip.y2 = (intel_crtc->scaling_src_size &
				SCALING_SRCSIZE_MASK) + 1;
	}

	hscale = drm_rect_calc_hscale_relaxed(&src, &dst, min_scale, max_scale);
	BUG_ON(hscale < 0);

	vscale = drm_rect_calc_vscale_relaxed(&src, &dst, min_scale, max_scale);
	BUG_ON(vscale < 0);

	visible = drm_rect_clip_scaled(&src, &dst, &clip, hscale, vscale);

	crtc_x = dst.x1;
	crtc_y = dst.y1;
	crtc_w = drm_rect_width(&dst);
	crtc_h = drm_rect_height(&dst);

	if (visible) {
		/* check again in case clipping clamped the results */
		hscale = drm_rect_calc_hscale(&src, &dst, min_scale, max_scale);
		if (hscale < 0) {
			DRM_DEBUG_KMS("Horizontal scaling factor out of limits\n");
			drm_rect_debug_print(&src, true);
			drm_rect_debug_print(&dst, false);

			return hscale;
		}

		vscale = drm_rect_calc_vscale(&src, &dst, min_scale, max_scale);
		if (vscale < 0) {
			DRM_DEBUG_KMS("Vertical scaling factor out of limits\n");
			drm_rect_debug_print(&src, true);
			drm_rect_debug_print(&dst, false);

			return vscale;
		}

		/* Make the source viewport size an exact multiple of the scaling factors. */
		drm_rect_adjust_size(&src,
				     drm_rect_width(&dst) * hscale - drm_rect_width(&src),
				     drm_rect_height(&dst) * vscale - drm_rect_height(&src));

		/* sanity check to make sure the src viewport wasn't enlarged */
		WARN_ON(src.x1 < (int) src_x ||
			src.y1 < (int) src_y ||
			src.x2 > (int) (src_x + src_w) ||
			src.y2 > (int) (src_y + src_h));

		/*
		 * Hardware doesn't handle subpixel coordinates.
		 * Adjust to (macro)pixel boundary, but be careful not to
		 * increase the source viewport size, because that could
		 * push the downscaling factor out of bounds.
		 */
		src_x = src.x1 >> 16;
		src_w = drm_rect_width(&src) >> 16;
		src_y = src.y1 >> 16;
		src_h = drm_rect_height(&src) >> 16;

		if (format_is_yuv(fb->pixel_format)) {
			src_x &= ~1;
			src_w &= ~1;

			/*
			 * Must keep src and dst the
			 * same if we can't scale.
			 */
			if (!intel_plane->can_scale)
				crtc_w &= ~1;

			if (crtc_w == 0)
				visible = false;
		}
	}

	/* Check size restrictions when scaling */
	if (visible && (src_w != crtc_w || src_h != crtc_h)) {
		unsigned int width_bytes;

		WARN_ON(!intel_plane->can_scale);

		/* FIXME interlacing min height is 6 */

		if (crtc_w < 3 || crtc_h < 3)
			visible = false;

		if (src_w < 3 || src_h < 3)
			visible = false;

		width_bytes = ((src_x * pixel_size) & 63) + src_w * pixel_size;

		if (src_w > 2048 || src_h > 2048 ||
		    width_bytes > 4096 || fb->pitches[0] > 4096) {
			DRM_DEBUG_KMS("Source dimensions exceed hardware limits\n");
			return -EINVAL;
		}
	}

	dst.x1 = crtc_x;
	dst.x2 = crtc_x + crtc_w;
	dst.y1 = crtc_y;
	dst.y2 = crtc_y + crtc_h;

	/*
	 * If the sprite is completely covering the primary plane,
	 * we can disable the primary and save power.
	 */
	if (!IS_VALLEYVIEW(dev)) {
		disable_primary = drm_rect_equals(&dst, &clip);
		WARN_ON(disable_primary && !visible);
	}
	/*
	 * Ideally when one unpin work is in progress another request will not
	 * come from the HWC. But if in worst case faulty situations we get then
	 * the system will enter into an unrecoverable state, which needs hard
	 * shutdown. So as a precaution if the sprite_unpin_work is not null
	 * then unpin immediately. This is done by passing NULL event.
	 */
	if (intel_crtc->sprite_unpin_work)
		event = NULL;
	if (event) {
		work = kzalloc(sizeof(*work), GFP_KERNEL);
		if (work == NULL)
			return -ENOMEM;
		work->event = event;
		work->crtc = crtc;
		work->old_fb_obj = old_obj;
		INIT_WORK(&work->work, intel_unpin_sprite_work_fn);

		ret = drm_vblank_get(dev, intel_crtc->pipe);
		if (ret) {
			DRM_ERROR("get vblank failed with %d\n", ret);
			goto free_work;
		}

		/* We borrow the event spin lock for protecting unpin_work */
		spin_lock_irqsave(&dev->event_lock, flags);
		if (intel_crtc->sprite_unpin_work) {
			spin_unlock_irqrestore(&dev->event_lock, flags);
			kfree(work);
			drm_vblank_put(dev, intel_crtc->pipe);
			intel_crtc->sprite_unpin_work = NULL;
			DRM_ERROR("flip queue: crtc already busy\n");
			return -EBUSY;
		}

		intel_crtc->sprite_unpin_work = work;
		spin_unlock_irqrestore(&dev->event_lock, flags);

		ret = i915_mutex_lock_interruptible(dev);
		if (ret)
			goto cleanup;

		work->pending_flip_obj = obj;
		/* Block clients from rendering to the new back buffer until
		* the flip occurs and the object is no longer visible.
		*/
		if (work->old_fb_obj != NULL)
			atomic_add(1 << intel_crtc->plane,
				&work->old_fb_obj->pending_flip);
	} else
		mutex_lock(&dev->struct_mutex);
	/* Note that this will apply the VT-d workaround for scanouts,
	 * which is more restrictive than required for sprites. (The
	 * primary plane requires 256KiB alignment with 64 PTE padding,
	 * the sprite planes only require 128KiB alignment and 32 PTE padding.
	 */
	drm_gem_object_reference(&obj->base);
	ret = intel_pin_and_fence_fb_obj(dev, obj, NULL);
	if (ret) {
		DRM_ERROR("pin and fence of fb failed with %d\n", ret);
		drm_gem_object_unreference(&obj->base);
		spin_lock_irqsave(&dev->event_lock, flags);
		intel_crtc->sprite_unpin_work = NULL;
		spin_unlock_irqrestore(&dev->event_lock, flags);
		if (event)
			drm_vblank_put(dev, intel_crtc->pipe);
		goto out_unlock;
	}
	intel_plane->old_obj = intel_plane->obj;
	intel_plane->obj = obj;

	/*
	 * Be sure to re-enable the primary before the sprite is no longer
	 * covering it fully.
	 */
	if (!IS_VALLEYVIEW(dev)) {
		if (!disable_primary)
			intel_enable_primary(plane, crtc);
	}
	/* TODO: Re-visit: enable primary */
	if (event == NULL)
		intel_enable_primary(plane, crtc);

	if (visible) {
		intel_plane->update_plane(plane, crtc, fb, obj,
					  crtc_x, crtc_y, crtc_w, crtc_h,
					  src_x, src_y, src_w, src_h, event);
	} else
		intel_plane->disable_plane(plane, crtc);

	/* TODO: Re-visit: disable primary after enable of sprite */
	if (event != NULL)
		intel_disable_primary(plane, crtc);

	if (!IS_VALLEYVIEW(dev)) {
		if (disable_primary)
			intel_disable_primary(plane, crtc);
	}

	/* Unpin old obj after new one is active to avoid ugliness */
	if (old_obj && (event == NULL)) {
		if (!IS_VALLEYVIEW(dev)) {
			/*
			 * It's fairly common to simply update the position of
			 * an existing object.  In that case, we don't need to
			 * wait for vblank to avoid ugliness, we only need to
			 * do the pin & ref bookkeeping.
			 */
			if (old_obj != obj) {
				mutex_unlock(&dev->struct_mutex);
				intel_wait_for_vblank(dev,
					to_intel_crtc(crtc)->pipe);
				mutex_lock(&dev->struct_mutex);
			}
		}
		intel_unpin_fb_obj(old_obj);
		drm_gem_object_unreference(&old_obj->base);
		if (intel_plane->plane == 0)
			intel_crtc->sprite_unpin_work = NULL;
	}

out_unlock:
	mutex_unlock(&dev->struct_mutex);
	if (event)
		trace_i915_flip_request(intel_crtc->plane, obj);
	return ret;
cleanup:
	spin_lock_irqsave(&dev->event_lock, flags);
	intel_crtc->sprite_unpin_work = NULL;
	spin_unlock_irqrestore(&dev->event_lock, flags);
	drm_vblank_put(dev, intel_crtc->pipe);
free_work:
	kfree(work);

	return ret;
}

static void intel_disable_plane_unpin_work_fn(struct work_struct *__work)
{
	struct intel_plane *intel_plane =
			container_of(__work, struct intel_plane, work);
	struct drm_device *dev = intel_plane->base.dev;

	intel_wait_for_vblank(dev, intel_plane->pipe);
	if (intel_plane->obj || intel_plane->old_obj) {
		mutex_lock(&dev->struct_mutex);
		if (intel_plane->obj) {
			intel_unpin_fb_obj(intel_plane->obj);
			drm_gem_object_unreference(&intel_plane->obj->base);
		}

		if (intel_plane->old_obj) {
			intel_unpin_fb_obj(intel_plane->old_obj);
			drm_gem_object_unreference(&intel_plane->old_obj->base);
		}

		mutex_unlock(&dev->struct_mutex);
	}

	kfree(intel_plane);
}

static int
intel_disable_plane(struct drm_plane *plane)
{
	struct drm_device *dev = plane->dev;
	struct drm_i915_private *dev_priv = dev->dev_private;
	struct intel_plane *intel_plane = to_intel_plane(plane);
	struct intel_plane *intel_plane_wq;
	int ret = 0;

	if (!plane->fb)
		return 0;

	if (WARN_ON(!plane->crtc))
		return -EINVAL;

	intel_plane_wq = kzalloc(sizeof(*intel_plane_wq), GFP_KERNEL);
	if (!intel_plane_wq)
		return -ENOMEM;

	/* To support deffered plane disable */
	INIT_WORK(&intel_plane_wq->work, intel_disable_plane_unpin_work_fn);

	/* If MAX FIFO enabled disable */
	if (dev_priv->maxfifo_enabled) {
		I915_WRITE(FW_BLC_SELF_VLV, ~FW_CSPWRDWNEN);
		dev_priv->maxfifo_enabled = false;
	}

	intel_enable_primary(plane, plane->crtc);
	intel_plane->disable_plane(plane, plane->crtc);

	intel_plane_wq->base.dev = plane->dev;
	intel_plane_wq->old_obj = intel_plane->old_obj;
	intel_plane_wq->obj = intel_plane->obj;
	intel_plane_wq->pipe = intel_plane->pipe;

	intel_plane->obj = NULL;
	intel_plane->old_obj = NULL;

	schedule_work(&intel_plane_wq->work);

	return ret;
}

static void intel_destroy_plane(struct drm_plane *plane)
{
	struct intel_plane *intel_plane = to_intel_plane(plane);
	intel_disable_plane(plane);
	drm_plane_cleanup(plane);
	kfree(intel_plane);
}

int intel_sprite_set_colorkey(struct drm_device *dev, void *data,
			      struct drm_file *file_priv)
{
	struct drm_intel_sprite_colorkey *set = data;
	struct drm_mode_object *obj;
	struct drm_plane *plane;
	struct intel_plane *intel_plane;
	int ret = 0;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -ENODEV;

	/* Make sure we don't try to enable both src & dest simultaneously */
	if ((set->flags & (I915_SET_COLORKEY_DESTINATION | I915_SET_COLORKEY_SOURCE)) == (I915_SET_COLORKEY_DESTINATION | I915_SET_COLORKEY_SOURCE))
		return -EINVAL;

	drm_modeset_lock_all(dev);

	obj = drm_mode_object_find(dev, set->plane_id, DRM_MODE_OBJECT_PLANE);
	if (!obj) {
		ret = -EINVAL;
		goto out_unlock;
	}

	plane = obj_to_plane(obj);
	intel_plane = to_intel_plane(plane);
	ret = intel_plane->update_colorkey(plane, set);

out_unlock:
	drm_modeset_unlock_all(dev);
	return ret;
}

int intel_sprite_get_colorkey(struct drm_device *dev, void *data,
			      struct drm_file *file_priv)
{
	struct drm_intel_sprite_colorkey *get = data;
	struct drm_mode_object *obj;
	struct drm_plane *plane;
	struct intel_plane *intel_plane;
	int ret = 0;

	if (!drm_core_check_feature(dev, DRIVER_MODESET))
		return -ENODEV;

	drm_modeset_lock_all(dev);

	obj = drm_mode_object_find(dev, get->plane_id, DRM_MODE_OBJECT_PLANE);
	if (!obj) {
		ret = -EINVAL;
		goto out_unlock;
	}

	plane = obj_to_plane(obj);
	intel_plane = to_intel_plane(plane);
	intel_plane->get_colorkey(plane, get);

out_unlock:
	drm_modeset_unlock_all(dev);
	return ret;
}

void intel_plane_restore(struct drm_plane *plane)
{
	struct intel_plane *intel_plane = to_intel_plane(plane);

	if (!plane->crtc || !plane->fb)
		return;

	intel_update_plane(plane, plane->crtc, plane->fb,
			   intel_plane->crtc_x, intel_plane->crtc_y,
			   intel_plane->crtc_w, intel_plane->crtc_h,
			   intel_plane->src_x, intel_plane->src_y,
			   intel_plane->src_w, intel_plane->src_h, NULL);
}

void intel_plane_disable(struct drm_plane *plane)
{
	if (!plane->crtc || !plane->fb)
		return;

	intel_disable_plane(plane);
}

static const struct drm_plane_funcs intel_plane_funcs = {
	.update_plane = intel_update_plane,
	.disable_plane = intel_disable_plane,
	.destroy = intel_destroy_plane,
};

static uint32_t ilk_plane_formats[] = {
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY,
};

static uint32_t snb_plane_formats[] = {
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY,
};

static uint32_t vlv_plane_formats[] = {
	DRM_FORMAT_RGB565,
	DRM_FORMAT_ABGR8888,
	DRM_FORMAT_ARGB8888,
	DRM_FORMAT_XBGR8888,
	DRM_FORMAT_XRGB8888,
	DRM_FORMAT_XBGR2101010,
	DRM_FORMAT_ABGR2101010,
	DRM_FORMAT_YUYV,
	DRM_FORMAT_YVYU,
	DRM_FORMAT_UYVY,
	DRM_FORMAT_VYUY,
};

int
intel_plane_init(struct drm_device *dev, enum pipe pipe, int plane)
{
	struct intel_plane *intel_plane;
	unsigned long possible_crtcs;
	const uint32_t *plane_formats;
	int num_plane_formats;
	int ret;

	if (INTEL_INFO(dev)->gen < 5)
		return -ENODEV;

	intel_plane = kzalloc(sizeof(struct intel_plane), GFP_KERNEL);
	if (!intel_plane)
		return -ENOMEM;

	switch (INTEL_INFO(dev)->gen) {
	case 5:
	case 6:
		intel_plane->can_scale = true;
		intel_plane->max_downscale = 16;
		intel_plane->update_plane = ilk_update_plane;
		intel_plane->disable_plane = ilk_disable_plane;
		intel_plane->update_colorkey = ilk_update_colorkey;
		intel_plane->get_colorkey = ilk_get_colorkey;

		if (IS_GEN6(dev)) {
			plane_formats = snb_plane_formats;
			num_plane_formats = ARRAY_SIZE(snb_plane_formats);
		} else {
			plane_formats = ilk_plane_formats;
			num_plane_formats = ARRAY_SIZE(ilk_plane_formats);
		}
		break;

	case 7:
		if (IS_IVYBRIDGE(dev)) {
			intel_plane->can_scale = true;
			intel_plane->max_downscale = 2;
		} else {
			intel_plane->can_scale = false;
			intel_plane->max_downscale = 1;
		}

		if (IS_VALLEYVIEW(dev)) {
			intel_plane->update_plane = vlv_update_plane;
			intel_plane->disable_plane = vlv_disable_plane;
			intel_plane->update_colorkey = vlv_update_colorkey;
			intel_plane->get_colorkey = vlv_get_colorkey;

			plane_formats = vlv_plane_formats;
			num_plane_formats = ARRAY_SIZE(vlv_plane_formats);
		} else {
			intel_plane->update_plane = ivb_update_plane;
			intel_plane->disable_plane = ivb_disable_plane;
			intel_plane->update_colorkey = ivb_update_colorkey;
			intel_plane->get_colorkey = ivb_get_colorkey;

			plane_formats = snb_plane_formats;
			num_plane_formats = ARRAY_SIZE(snb_plane_formats);
		}
		break;

	default:
		kfree(intel_plane);
		return -ENODEV;
	}

	intel_plane->pipe = pipe;
	intel_plane->plane = plane;
	intel_plane->rotate180 = false;
	intel_plane->last_plane_state = false; /* false means disabled */
	possible_crtcs = (1 << pipe);
	ret = drm_plane_init(dev, &intel_plane->base, possible_crtcs,
			     &intel_plane_funcs,
			     plane_formats, num_plane_formats,
			     false);
	if (ret)
		kfree(intel_plane);

	return ret;
}
