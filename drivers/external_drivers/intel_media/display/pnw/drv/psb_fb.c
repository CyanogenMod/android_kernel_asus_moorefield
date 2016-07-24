/**************************************************************************
 * Copyright (c) 2007, Intel Corporation.
 * All Rights Reserved.
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
 **************************************************************************/

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/errno.h>
#include <linux/string.h>
#include <linux/mm.h>
#include <linux/tty.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/console.h>

#include <drm/drmP.h>
#include <drm/drm.h>
#include <drm/drm_crtc.h>

#include "psb_drv.h"
#include "psb_intel_reg.h"
#include "psb_ttm_userobj_api.h"
#include "psb_fb.h"
#include "psb_pvr_glue.h"

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_output.h"
#include "mdfld_output.h"
#include "mdfld_dsi_dbi_dsr.h"


#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int fill_fb_bitfield(struct fb_var_screeninfo *var, int depth)
{
	switch (depth) {
	case 8:
		var->red.offset = 0;
		var->green.offset = 0;
		var->blue.offset = 0;
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		var->transp.length = 0;
		var->transp.offset = 0;
		break;
	case 15:
		var->red.offset = 10;
		var->green.offset = 5;
		var->blue.offset = 0;
		var->red.length = 5;
		var->green.length = 5;
		var->blue.length = 5;
		var->transp.length = 1;
		var->transp.offset = 15;
		break;
	case 16:
		var->red.offset = 11;
		var->green.offset = 5;
		var->blue.offset = 0;
		var->red.length = 5;
		var->green.length = 6;
		var->blue.length = 5;
		var->transp.length = 0;
		var->transp.offset = 0;
		break;
	case 24:
		var->red.offset = 16;
		var->green.offset = 8;
		var->blue.offset = 0;
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		var->transp.length = 0;
		var->transp.offset = 0;
		break;
	case 32:
		var->red.offset = 16;
		var->green.offset = 8;
		var->blue.offset = 0;
		var->red.length = 8;
		var->green.length = 8;
		var->blue.length = 8;
		var->transp.length = 8;
		var->transp.offset = 24;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

struct psbfb_par {
	struct drm_device *dev;
	struct psb_framebuffer *psbfb;

	int dpms_state;

	int crtc_count;
	/* crtc currently bound to this */
	uint32_t crtc_ids[2];
};
#endif

static void psb_user_framebuffer_destroy(struct drm_framebuffer *fb);
static int psb_user_framebuffer_create_handle(struct drm_framebuffer *fb,
					      struct drm_file *file_priv,
					      unsigned int *handle);

static const struct drm_framebuffer_funcs psb_fb_funcs = {
	.destroy = psb_user_framebuffer_destroy,
	.create_handle = psb_user_framebuffer_create_handle,
};

#define CMAP_TOHW(_val, _width) ((((_val) << (_width)) + 0x7FFF - (_val)) >> 16)

void *psbfb_vdc_reg(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv;
	dev_priv = (struct drm_psb_private *) dev->dev_private;
	return dev_priv->vdc_reg;
}
/*EXPORT_SYMBOL(psbfb_vdc_reg); */

static int psbfb_setcolreg(unsigned regno, unsigned red, unsigned green,
			   unsigned blue, unsigned transp,
			   struct fb_info *info)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	struct psbfb_par *par = info->par;
	struct drm_framebuffer *fb = &par->psbfb->base;
#else
	struct psb_fbdev * fbdev = info->par;
	struct drm_framebuffer *fb = fbdev->psb_fb_helper.fb;
#endif
	uint32_t v;

	if (!fb)
		return -ENOMEM;

	if (regno > 255)
		return 1;

	red = CMAP_TOHW(red, info->var.red.length);
	blue = CMAP_TOHW(blue, info->var.blue.length);
	green = CMAP_TOHW(green, info->var.green.length);
	transp = CMAP_TOHW(transp, info->var.transp.length);

	v = (red << info->var.red.offset) |
	    (green << info->var.green.offset) |
	    (blue << info->var.blue.offset) |
	    (transp << info->var.transp.offset);

	if (regno < 16) {
		switch (fb->bits_per_pixel) {
		case 16:
			((uint32_t *) info->pseudo_palette)[regno] = v;
			break;
		case 24:
		case 32:
			((uint32_t *) info->pseudo_palette)[regno] = v;
			break;
		}
	}

	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static struct drm_display_mode *psbfb_find_first_mode(struct
						      fb_var_screeninfo
						      *var,
						      struct fb_info *info,
						      struct drm_crtc
						      *crtc)
{
	struct psbfb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct drm_display_mode *drm_mode;
	struct drm_display_mode *preferred_mode = NULL;
	struct drm_display_mode *last_mode = NULL;
	struct drm_connector *connector;
	int found;

	found = 0;
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		if (connector->encoder && connector->encoder->crtc == crtc) {
			found = 1;
			break;
		}
	}

	/* found no connector, bail */
	if (!found)
		return NULL;

	found = 0;
	list_for_each_entry(drm_mode, &connector->modes, head) {
		if (drm_mode->hdisplay == var->xres &&
		    drm_mode->vdisplay == var->yres
		    && drm_mode->clock != 0) {
			found = 1;
			last_mode = drm_mode;
			if (IS_POULSBO(dev)) {
				if (last_mode->type & DRM_MODE_TYPE_PREFERRED)
					preferred_mode = last_mode;
			}
		}
	}

	/* No mode matching mode found */
	if (!found)
		return NULL;

	if (IS_POULSBO(dev)) {
		if (preferred_mode)
			return preferred_mode;
		else
			return last_mode;
	} else {
		return last_mode;
	}
}

static int psbfb_check_var(struct fb_var_screeninfo *var,
			   struct fb_info *info)
{
	struct psbfb_par *par = info->par;
	struct psb_framebuffer *psbfb = par->psbfb;
	struct drm_device *dev = par->dev;
	int ret;
	int depth;
	int pitch;
	int bpp = var->bits_per_pixel;

	if (!psbfb)
		return -ENOMEM;

	if (!var->pixclock)
		return -EINVAL;

	/* don't support virtuals for now */
	if (var->xres_virtual > var->xres)
		return -EINVAL;

	if (var->yres_virtual > var->yres)
		return -EINVAL;

	switch (bpp) {
	case 16:
		depth = (var->green.length == 6) ? 16 : 15;
		break;
	case 24:		/* assume this is 32bpp / depth 24 */
		bpp = 32;
		/* fallthrough */
	case 32:
		depth = (var->transp.length > 0) ? 32 : 24;
		break;
	default:
		return -EINVAL;
	}

	pitch = ((var->xres * ((bpp + 1) / 8)) + 0x3f) & ~0x3f;

	/* Check that we can resize */
	if ((pitch * var->yres) > psbfb->size) {
#if 1
		/* Need to resize the fb object.
		 * But the generic fbdev code doesn't really understand
		 * that we can do this. So disable for now.
		 */
		DRM_INFO("Can't support requested size, too big!\n");
		return -EINVAL;
#endif
	}

	ret = fill_fb_bitfield(var, depth);
	if (ret)
		return ret;

#if 1
	/* Here we walk the output mode list and look for modes. If we haven't
	 * got it, then bail. Not very nice, so this is disabled.
	 * In the set_par code, we create our mode based on the incoming
	 * parameters. Nicer, but may not be desired by some.
	 */
	{
		struct drm_crtc *crtc;
		int i;

		list_for_each_entry(crtc, &dev->mode_config.crtc_list,
				    head) {
			struct psb_intel_crtc *psb_intel_crtc =
			    to_psb_intel_crtc(crtc);

			for (i = 0; i < par->crtc_count; i++)
				if (crtc->base.id == par->crtc_ids[i])
					break;

			if (i == par->crtc_count)
				continue;

			if (psb_intel_crtc->mode_set.num_connectors == 0)
				continue;

			if (!psbfb_find_first_mode(&info->var, info, crtc))
				return -EINVAL;
		}
	}
#else
	(void) i;
	(void) dev;		/* silence warnings */
	(void) crtc;
	(void) drm_mode;
	(void) connector;
#endif

	return 0;
}

/* this will let fbcon do the mode init */
static int psbfb_set_par(struct fb_info *info)
{
	struct psbfb_par *par = info->par;
	struct psb_framebuffer *psbfb = par->psbfb;
	struct drm_framebuffer *fb = &psbfb->base;
	struct drm_device *dev = par->dev;
	struct fb_var_screeninfo *var = &info->var;
	/* struct drm_psb_private *dev_priv = dev->dev_private; */
	struct drm_display_mode *drm_mode;
	int pitch;
	int depth;
	int bpp = var->bits_per_pixel;

	PSB_DEBUG_ENTRY("\n");

	if (!fb)
		return -ENOMEM;

	PSB_DEBUG_ENTRY("01. \n");

	switch (bpp) {
	case 8:
		depth = 8;
		break;
	case 16:
		depth = (var->green.length == 6) ? 16 : 15;
		break;
	case 24:		/* assume this is 32bpp / depth 24 */
		bpp = 32;
		/* fallthrough */
	case 32:
		depth = (var->transp.length > 0) ? 32 : 24;
		break;
	default:
		DRM_ERROR("Illegal BPP\n");
		return -EINVAL;
	}

	pitch = ((var->xres * ((bpp + 1) / 8)) + 0x3f) & ~0x3f;

	if ((pitch * var->yres) > (psbfb->size)) {
#if 1
		/* Need to resize the fb object.
		 * But the generic fbdev code doesn't really understand
		 * that we can do this. So disable for now.
		 */
		DRM_INFO("Can't support requested size, too big!\n");
		return -EINVAL;
#endif
	}

	psbfb->offset = 0;
	fb->width = var->xres;
	fb->height = var->yres;
	fb->bits_per_pixel = bpp;
	fb->pitches[0] = pitch;
	fb->depth = depth;

	info->fix.line_length = psbfb->base.pitches[0];
	info->fix.visual =
	    (psbfb->base.depth ==
	     8) ? FB_VISUAL_PSEUDOCOLOR : FB_VISUAL_DIRECTCOLOR;

	/* some fbdev's apps don't want these to change */
	info->fix.smem_start = dev->mode_config.fb_base + psbfb->offset;

#if 0
	/* relates to resize - disable */
	info->fix.smem_len = info->fix.line_length * var->yres;
	info->screen_size = info->fix.smem_len;	/* ??? */
#endif

	/* Should we walk the output's modelist or just create our own ???
	 * For now, we create and destroy a mode based on the incoming
	 * parameters. But there's commented out code below which scans
	 * the output list too.
	 */
#if 1
	/* This code is now in the for loop futher down. */
#endif

	{
		struct drm_crtc *crtc;
		int ret;
		int i;

		list_for_each_entry(crtc, &dev->mode_config.crtc_list,
				    head) {
			struct psb_intel_crtc *psb_intel_crtc =
			    to_psb_intel_crtc(crtc);

			for (i = 0; i < par->crtc_count; i++)
				if (crtc->base.id == par->crtc_ids[i])
					break;

			if (i == par->crtc_count)
				continue;

			if (psb_intel_crtc->mode_set.num_connectors == 0)
				continue;

#if 1
			drm_mode =
			    psbfb_find_first_mode(&info->var, info, crtc);
			if (!drm_mode)
				DRM_ERROR("No matching mode found\n");
			psb_intel_crtc->mode_set.mode = drm_mode;
#endif

#if 0				/* FIXME: TH */
			if (crtc->fb == psb_intel_crtc->mode_set.fb) {
#endif
				DRM_DEBUG
				    ("setting mode on crtc %p with id %u\n",
				     crtc, crtc->base.id);
				ret =
				    crtc->funcs->
				    set_config(&psb_intel_crtc->mode_set);
				if (ret) {
					DRM_ERROR("Failed setting mode\n");
					return ret;
				}
#if 0
			}
#endif
		}
		DRM_DEBUG("Set par returned OK.\n");
		return 0;
	}

	return 0;
}

void psbfb_imageblit(struct fb_info *info, const struct fb_image *image)
{
	if (unlikely(info->state != FBINFO_STATE_RUNNING))
		return;

	cfb_imageblit(info, image);
}

static void psbfb_onoff(struct fb_info *info, int dpms_mode)
{
	struct psbfb_par *par = info->par;
	struct drm_device *dev = par->dev;
	struct drm_crtc *crtc;
	struct drm_encoder *encoder;
	int i;

	/*
	 * For each CRTC in this fb, find all associated encoders
	 * and turn them off, then turn off the CRTC.
	 */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct drm_crtc_helper_funcs *crtc_funcs =
		    crtc->helper_private;

		for (i = 0; i < par->crtc_count; i++)
			if (crtc->base.id == par->crtc_ids[i])
				break;

		if (i == par->crtc_count)
			continue;

		if (dpms_mode == DRM_MODE_DPMS_ON)
			crtc_funcs->dpms(crtc, dpms_mode);

		/* Found a CRTC on this fb, now find encoders */
		list_for_each_entry(encoder,
				    &dev->mode_config.encoder_list, head) {
			if (encoder->crtc == crtc) {
				struct drm_encoder_helper_funcs
				*encoder_funcs;
				encoder_funcs = encoder->helper_private;
				encoder_funcs->dpms(encoder, dpms_mode);
			}
		}

		if (dpms_mode == DRM_MODE_DPMS_OFF)
			crtc_funcs->dpms(crtc, dpms_mode);
	}
}

static int psbfb_blank(int blank_mode, struct fb_info *info)
{
	struct psbfb_par *par = info->par;

	par->dpms_state = blank_mode;
	PSB_DEBUG_PM("psbfb_blank \n");
	switch (blank_mode) {
	case FB_BLANK_UNBLANK:
		psbfb_onoff(info, DRM_MODE_DPMS_ON);
		break;
	case FB_BLANK_NORMAL:
		psbfb_onoff(info, DRM_MODE_DPMS_STANDBY);
		break;
	case FB_BLANK_HSYNC_SUSPEND:
		psbfb_onoff(info, DRM_MODE_DPMS_STANDBY);
		break;
	case FB_BLANK_VSYNC_SUSPEND:
		psbfb_onoff(info, DRM_MODE_DPMS_SUSPEND);
		break;
	case FB_BLANK_POWERDOWN:
		psbfb_onoff(info, DRM_MODE_DPMS_OFF);
		break;
	}

	return 0;
}
#endif /*KERNEL_VERSION < 2.6.35*/

static int psbfb_kms_off(struct drm_device *dev, int suspend)
{
	struct drm_framebuffer *fb = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct psb_framebuffer * psbfb = to_psb_fb(fb);
	if (!psbfb) {
		DRM_ERROR("Invalid psbfb\n");
		return -EINVAL;
	}
#endif
	DRM_DEBUG("psbfb_kms_off_ioctl\n");

	mutex_lock(&dev->mode_config.mutex);
	list_for_each_entry(fb, &dev->mode_config.fb_list, head) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
		struct fb_info *info = psbfb->fbdev;

		if (suspend) {
			fb_set_suspend(info, 1);
			drm_fb_helper_blank(FB_BLANK_POWERDOWN, info);
		}
#else
		struct fb_info *info = fb->fbdev;

		if (suspend) {
			fb_set_suspend(info, 1);
			psbfb_blank(FB_BLANK_POWERDOWN, info);
		}
#endif
	}
	mutex_unlock(&dev->mode_config.mutex);
	return 0;
}

int psbfb_kms_off_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	int ret;

	if (drm_psb_no_fb)
		return 0;
	console_lock();
	ret = psbfb_kms_off(dev, 0);
	console_unlock();

	return ret;
}

static int psbfb_kms_on(struct drm_device *dev, int resume)
{
	struct drm_framebuffer *fb = 0;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct psb_framebuffer * psbfb = to_psb_fb(fb);
	if (!psbfb) {
		DRM_ERROR("Invalid psbfb\n");
		return -EINVAL;
	}
#endif

	DRM_DEBUG("psbfb_kms_on_ioctl\n");

	mutex_lock(&dev->mode_config.mutex);
	list_for_each_entry(fb, &dev->mode_config.fb_list, head) {
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
		struct fb_info *info = psbfb->fbdev;

		if (resume) {
			fb_set_suspend(info, 0);
			drm_fb_helper_blank(FB_BLANK_UNBLANK, info);
		}
#else
		struct fb_info *info = fb->fbdev;

		if (resume) {
			fb_set_suspend(info, 0);
			psbfb_blank(FB_BLANK_UNBLANK, info);
		}
#endif
	}
	mutex_unlock(&dev->mode_config.mutex);

	return 0;
}

int psbfb_kms_on_ioctl(struct drm_device *dev, void *data,
		       struct drm_file *file_priv)
{
	int ret;

	if (drm_psb_no_fb)
		return 0;
	console_lock();
	ret = psbfb_kms_on(dev, 0);
	console_unlock();
	drm_helper_disable_unused_functions(dev);
	return ret;
}

void psbfb_suspend(struct drm_device *dev)
{
	console_lock();
	psbfb_kms_off(dev, 1);
	console_unlock();
}

void psbfb_resume(struct drm_device *dev)
{
	console_lock();
	psbfb_kms_on(dev, 1);
	console_unlock();
	drm_helper_disable_unused_functions(dev);
}

static int psbfb_vm_fault(struct vm_area_struct * vma, struct vm_fault * vmf)
{
	int page_num = 0;
	int i;
	unsigned long address = 0;
	int ret;
	unsigned long pfn;
	struct psb_framebuffer *psbfb = (struct psb_framebuffer *)vma->vm_private_data;
	struct drm_device * dev = psbfb->base.dev;
        struct drm_psb_private * dev_priv = (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;
	unsigned long phys_addr = (unsigned long)pg->stolen_base;;

	page_num = (vma->vm_end - vma->vm_start) >> PAGE_SHIFT;

	address = (unsigned long)vmf->virtual_address;

	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	for(i=0; i<page_num; i++) {
		pfn = (phys_addr >> PAGE_SHIFT); //phys_to_pfn(phys_addr);

		ret = vm_insert_mixed(vma, address, pfn);
		if(unlikely((ret == -EBUSY) || (ret != 0 && i > 0)))
			break;
		else if(unlikely(ret != 0)) {
			ret = (ret == -ENOMEM) ? VM_FAULT_OOM : VM_FAULT_SIGBUS;
			return ret;
		}

		address += PAGE_SIZE;
		phys_addr += PAGE_SIZE;
	}

	return VM_FAULT_NOPAGE;
}

static void psbfb_vm_open(struct vm_area_struct * vma)
{
	DRM_DEBUG("vm_open\n");
}

static void psbfb_vm_close(struct vm_area_struct * vma)
{
	DRM_DEBUG("vm_close\n");
}

static struct vm_operations_struct psbfb_vm_ops = {
	.fault 	= psbfb_vm_fault,
	.open	= psbfb_vm_open,
	.close	= psbfb_vm_close
};

static int psbfb_mmap(struct fb_info *info, struct vm_area_struct *vma)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	struct psbfb_par *par = info->par;
	struct psb_framebuffer *psbfb = par->psbfb;
#else
	struct psb_fbdev * fbdev = info->par;
	struct psb_framebuffer *psbfb = fbdev->pfb;
#endif
	char * fb_screen_base = NULL;
	struct drm_device * dev = psbfb->base.dev;
	struct drm_psb_private * dev_priv = (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;

	if (vma->vm_pgoff != 0)
		return -EINVAL;
	if (vma->vm_pgoff > (~0UL >> PAGE_SHIFT))
		return -EINVAL;

	if (!psbfb->addr_space)
		psbfb->addr_space = vma->vm_file->f_mapping;

	fb_screen_base = (char *)info->screen_base;

	DRM_DEBUG("vm_pgoff 0x%lx, screen base %p vram_addr %p\n", vma->vm_pgoff, fb_screen_base, pg->vram_addr);

	/*if using stolen memory, */
	if(fb_screen_base == pg->vram_addr) {
		vma->vm_ops = &psbfb_vm_ops;
		vma->vm_private_data = (void *)psbfb;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0)
		vma->vm_flags |= VM_RESERVED | VM_IO | VM_MIXEDMAP | VM_DONTEXPAND;
#else
		vma->vm_flags |= VM_IO | VM_MIXEDMAP | VM_DONTEXPAND;
#endif
	} else {
	/*using IMG meminfo, can I use pvrmmap to map it?*/

	}

	/*forbid dsr*/
	mdfld_dsi_dsr_forbid(dev_priv->dsi_configs[0]);

	return 0;
}

/* Disable the obsolete fb blank callback function*/
static int fb_blank_void(int blank_mode, struct fb_info *info)
{
	return 0;
}
static void psb_cfb_fillrect(struct fb_info *p, const struct fb_fillrect *rect)
{
	return ;
}

static void psb_cfb_copyarea(struct fb_info *p, const struct fb_copyarea *area)
{
	return ;
}
static void psb_cfb_imageblit(struct fb_info *p, const struct fb_image *image)
{
	return ;
}
static struct fb_ops psbfb_ops = {
	.owner = THIS_MODULE,
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	.fb_check_var = psbfb_check_var,
	.fb_set_par = psbfb_set_par,
	.fb_blank = psbfb_blank,
#else
	.fb_check_var = drm_fb_helper_check_var,
	.fb_set_par = drm_fb_helper_set_par,
	.fb_blank = fb_blank_void,
#endif
	.fb_setcolreg = psbfb_setcolreg,
	.fb_fillrect = psb_cfb_fillrect,
	.fb_copyarea = psb_cfb_copyarea,
	.fb_imageblit = psb_cfb_imageblit,
	.fb_mmap = psbfb_mmap,
};

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static struct drm_mode_set panic_mode;

int psbfb_panic(struct notifier_block *n, unsigned long ununsed,
		void *panic_str)
{
	DRM_ERROR("panic occurred, switching back to text console\n");
	drm_crtc_helper_set_config(&panic_mode);

	return 0;
}
/*EXPORT_SYMBOL(psbfb_panic); */

static struct notifier_block paniced = {
	.notifier_call = psbfb_panic,
};
#endif

static struct drm_framebuffer *psb_framebuffer_create
			(struct drm_device *dev, struct drm_mode_fb_cmd2 *r,
			 void *mm_private)
{
	struct psb_framebuffer *fb;
	int ret;

	fb = kzalloc(sizeof(*fb), GFP_KERNEL);
	if (!fb)
		return NULL;

	ret = drm_framebuffer_init(dev, &fb->base, &psb_fb_funcs);

	if (ret)
		goto err;

	drm_helper_mode_fill_fb_struct(&fb->base, r);

	fb->pvrBO = mm_private;

	return &fb->base;

err:
	kfree(fb);
	return NULL;
}

static struct drm_framebuffer *psb_user_framebuffer_create
			(struct drm_device *dev, struct drm_file *filp,
			struct drm_mode_fb_cmd2 *r)
{
	struct psb_framebuffer *psbfb;
	struct drm_framebuffer *fb;
	struct fb_info *info;
	PVRSRV_KERNEL_MEM_INFO *psKernelMemInfo = IMG_NULL;
	IMG_HANDLE hKernelMemInfo = (IMG_HANDLE)r->handles[0];
	struct drm_psb_private *dev_priv
		= (struct drm_psb_private *) dev->dev_private;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct psb_fbdev * fbdev = dev_priv->fbdev;
#endif
	struct psb_gtt *pg = dev_priv->pg;
	int ret;
	uint32_t offset;
	uint64_t size;

	ret = psb_get_meminfo_by_handle(dev_priv, hKernelMemInfo,
			&psKernelMemInfo);
	if (ret) {
		DRM_ERROR("Cannot get meminfo for handle %p\n",
			  hKernelMemInfo);

		return ERR_PTR(ret);
	}

	DRM_DEBUG("Got Kernel MemInfo for handle %lx\n",
		  (unsigned long) hKernelMemInfo);

	/* JB: TODO not drop, make smarter */
	size = psKernelMemInfo->uAllocSize;
	if (size < r->height * r->pitches[0])
		return ERR_PTR(-ENOMEM);

	/* JB: TODO not drop, refcount buffer */
	/* return psb_framebuffer_create(dev, r, bo); */

	fb = psb_framebuffer_create(dev, r, (void *)psKernelMemInfo);
	if (!fb) {
		DRM_ERROR("failed to allocate fb.\n");
		return ERR_PTR(-EINVAL);
	}

	psbfb = to_psb_fb(fb);
	psbfb->size = size;
	psbfb->hKernelMemInfo = hKernelMemInfo;

	DRM_DEBUG("Mapping to gtt..., KernelMemInfo %p\n", psKernelMemInfo);

	/*if not VRAM, map it into tt aperture*/
	if (psKernelMemInfo->pvLinAddrKM != pg->vram_addr) {
		ret = psb_gtt_map_meminfo(dev, hKernelMemInfo, 0, &offset);
		if (ret) {
			DRM_ERROR("map meminfo for 0x%x failed\n",
				  (IMG_UINT32)hKernelMemInfo);
			return ERR_PTR(-ENOMEM);
		}
		psbfb->offset = (offset << PAGE_SHIFT);
	} else {
		psbfb->offset = 0;
	}
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	info = framebuffer_alloc(sizeof(struct psbfb_par), &dev->pdev->dev);
#else
	info = framebuffer_alloc(0, &dev->pdev->dev);
#endif
	if (!info)
		return ERR_PTR(-ENOMEM);

	strcpy(info->fix.id, "psbfb");

	info->flags = FBINFO_DEFAULT;
	info->fbops = &psbfb_ops;

	info->fix.smem_start = dev->mode_config.fb_base;
	info->fix.smem_len = size;

	info->screen_base = psKernelMemInfo->pvLinAddrKM;
	info->screen_size = size;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	info->fix.type = FB_TYPE_PACKED_PIXELS;
	info->fix.visual = FB_VISUAL_TRUECOLOR;
	info->fix.type_aux = 0;
	info->fix.xpanstep = 1;	/* doing it in hw */
	info->fix.ypanstep = 1;	/* doing it in hw */
	info->fix.ywrapstep = 0;
	info->fix.accel = FB_ACCEL_I830;
	info->fix.type_aux = 0;
	info->fix.line_length = fb->pitches[0];

	/* it is called for kms flip, the back buffer has been rendered,
	 * then we should not clear it*/
#if 0
	if (is_iomem)
		memset_io(info->screen_base, 0, size);
	else
		memset(info->screen_base, 0, size);
#endif
	info->pseudo_palette = fb->pseudo_palette;
	info->var.xres_virtual = fb->width;
	info->var.yres_virtual = fb->height;
	info->var.bits_per_pixel = fb->bits_per_pixel;
	info->var.xoffset = 0;
	info->var.yoffset = 0;
	info->var.activate = FB_ACTIVATE_NOW;
	info->var.height = -1;
	info->var.width = -1;

	info->var.xres = r->width;
	info->var.yres = r->height;

	fill_fb_bitfield(&info->var, fb->depth);
# else	/*KERNEL_VERSION > 2.6.35*/
	drm_fb_helper_fill_fix(info, fb->pitches[0], fb->depth);
	drm_fb_helper_fill_var(info, &fbdev->psb_fb_helper, fb->width, fb->height);
#endif

	info->fix.mmio_start = pci_resource_start(dev->pdev, 0);
	info->fix.mmio_len = pci_resource_len(dev->pdev, 0);

	info->pixmap.size = 64 * 1024;
	info->pixmap.buf_align = 8;
	info->pixmap.access_align = 32;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;
	info->pixmap.scan_align = 1;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	fb->fbdev = info;
#else
	psbfb->fbdev = info;
	fbdev->pfb = psbfb;

	fbdev->psb_fb_helper.fb = fb;
	fbdev->psb_fb_helper.fbdev = info;
#endif

	return fb;
}

static int psbfb_create(struct psb_fbdev * fbdev, struct drm_fb_helper_surface_size * sizes)
{
	struct drm_device * dev = fbdev->psb_fb_helper.dev;
	struct drm_psb_private * dev_priv = (struct drm_psb_private *)dev->dev_private;
	struct psb_gtt *pg = dev_priv->pg;
	struct fb_info * info;
	struct drm_framebuffer *fb;
	struct psb_framebuffer * psbfb;
	struct drm_mode_fb_cmd2 mode_cmd;
	struct device * device = &dev->pdev->dev;
	int size, aligned_size;
	int ret;
	struct mdfld_dsi_encoder *dsi_encoder =
		MDFLD_DSI_ENCODER_WITH_DRM_ENABLE(dev_priv->encoder0);
	struct mdfld_dsi_config *dsi_config =
		mdfld_dsi_encoder_get_config(dsi_encoder);
	struct drm_display_mode *fixed_mode = dsi_config->fixed_mode;

	/* PR2 panel must have 200 pixel dummy clocks,
	* So the display timing should be 800x1024, and surface
	* is 608x1024(64 bits align), or the information between android
	* and Linux frame buffer is not consistent.
	*/
#ifdef CONFIG_SUPPORT_DDS_MIPI_SWITCH   /*  ASUS_BSP: [DDS] +++ */
	if (0) {
		mode_cmd.width = 800;
		mode_cmd.height = 1280;
	} else {
		mode_cmd.width = fixed_mode->hdisplay;
		mode_cmd.height = fixed_mode->vdisplay;
	}
#else
	if (is_tmd_6x10_panel(dev, 0)) {
		mode_cmd.width  = fixed_mode->hdisplay - 200;
		mode_cmd.height = fixed_mode->vdisplay;
	} else {
		mode_cmd.width = fixed_mode->hdisplay;
		mode_cmd.height = fixed_mode->vdisplay;
	}
#endif  /*  ASUS_BSP: [DDS] --- */

        //HW requires pitch to be 64 byte aligned

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0))
	mode_cmd.bpp = 32;
	mode_cmd.pitches[0] = ALIGN(mode_cmd.width *
		((mode_cmd.bpp + 1) / 8), 64);
        mode_cmd.depth = 24;
#else
	/*  Note: sizes->surface_width == 800 and fixed_mode->hdisplay == 800,
	    but mode_cmd.width may be 600. */

	mode_cmd.pitches[0] = mode_cmd.width * (sizes->surface_bpp >> 3);
	mode_cmd.pitches[0] = ALIGN(mode_cmd.pitches[0], 64);

	mode_cmd.pixel_format = drm_mode_legacy_fb_format(sizes->surface_bpp,
		sizes->surface_depth);
#endif

	size = mode_cmd.pitches[0] * mode_cmd.height;
	aligned_size = roundup(size, PAGE_SIZE);

	mutex_lock(&dev->struct_mutex);
        fb = psb_framebuffer_create(dev, &mode_cmd, NULL);
        if (!fb) {
                DRM_ERROR("failed to allocate fb.\n");
                ret = -ENOMEM;
                goto out_err1;
        }
        psbfb = to_psb_fb(fb);
	if (!psbfb) {
		DRM_ERROR("Invalid psbfb\n");
		ret = -EINVAL;
		goto out_err1;
	}
        psbfb->size = size;

	info = framebuffer_alloc(sizeof(struct psb_fbdev), device);
	if(!info) {
		ret = -ENOMEM;
		goto out_err0;
	}

	info->par = fbdev;

	psbfb->fbdev = info;

	fbdev->psb_fb_helper.fb = fb;
	fbdev->psb_fb_helper.fbdev = info;
	fbdev->pfb = psbfb;

	strcpy(info->fix.id, "psbfb");

	info->flags = FBINFO_DEFAULT;
	info->fbops = &psbfb_ops;
	info->fix.smem_start = dev->mode_config.fb_base;
	info->fix.smem_len = size;
	info->screen_base = (char *)pg->vram_addr;
	info->screen_size = size;
	/* memset(info->screen_base, 0, size); */

	drm_fb_helper_fill_fix(info, fb->pitches[0], fb->depth);

	if (is_tmd_6x10_panel(dev, 0))
		drm_fb_helper_fill_var(info, &fbdev->psb_fb_helper, fb->width, fb->height);
	else
		drm_fb_helper_fill_var(info, &fbdev->psb_fb_helper, fb->width, fb->height);

	info->fix.mmio_start = pci_resource_start(dev->pdev, 0);
	info->fix.mmio_len = pci_resource_len(dev->pdev, 0);

	info->pixmap.size = 64 * 1024;
	info->pixmap.buf_align = 8;
	info->pixmap.access_align = 32;
	info->pixmap.flags = FB_PIXMAP_SYSTEM;
	info->pixmap.scan_align = 1;

	DRM_DEBUG("fb depth is %d\n", fb->depth);
	DRM_DEBUG("   pitch is %d\n", fb->pitches[0]);

	printk(KERN_INFO"allocated %dx%d fb\n",
		psbfb->base.width, psbfb->base.height);

	mutex_unlock(&dev->struct_mutex);

	return 0;
out_err0:
	fb->funcs->destroy(fb);
out_err1:
	mutex_unlock(&dev->struct_mutex);
	return ret;
}

static void psbfb_gamma_set(struct drm_crtc *crtc, u16 red, u16 green, u16 blue, int regno)
{
	DRM_DEBUG("%s\n", __func__);
}

static void psbfb_gamma_get(struct drm_crtc *crtc, u16 *red, u16 *green, u16 *blue, int regno)
{
	DRM_DEBUG("%s\n", __func__);
}

static int psbfb_probe(struct drm_fb_helper *helper, struct drm_fb_helper_surface_size *sizes)
{
	struct psb_fbdev * psb_fbdev = (struct psb_fbdev *)helper;
	int new_fb = 0;
	int ret;

	DRM_DEBUG("%s\n", __func__);

	if(!helper->fb) {
		ret = psbfb_create(psb_fbdev, sizes);
		if(ret) {
			return ret;
		}

		new_fb = 1;
	}

	return new_fb;
}

struct drm_fb_helper_funcs psb_fb_helper_funcs = {
	.gamma_set = psbfb_gamma_set,
	.gamma_get = psbfb_gamma_get,
	.fb_probe = psbfb_probe,
};

int psb_fbdev_destroy(struct drm_device * dev, struct psb_fbdev * fbdev)
{
	struct fb_info * info;
	struct psb_framebuffer * psbfb = fbdev->pfb;

	if(fbdev->psb_fb_helper.fbdev) {
		info = fbdev->psb_fb_helper.fbdev;
		unregister_framebuffer(info);
		iounmap(info->screen_base);
		framebuffer_release(info);
	}

	drm_fb_helper_fini(&fbdev->psb_fb_helper);

	drm_framebuffer_cleanup(&psbfb->base);

	return 0;
}

int psb_fbdev_init(struct drm_device *dev)
{
	struct psb_fbdev *fbdev;
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;
	int num_crtc;

	fbdev = kzalloc(sizeof(struct psb_fbdev), GFP_KERNEL);
	if(!fbdev) {
		DRM_ERROR("no memory\n");
		return -ENOMEM;
	}

	dev_priv->fbdev = fbdev;
	fbdev->psb_fb_helper.funcs = &psb_fb_helper_funcs;

	num_crtc = dev_priv->num_pipe;

	drm_fb_helper_init(dev, &fbdev->psb_fb_helper, num_crtc, INTELFB_CONN_LIMIT);

	drm_fb_helper_single_add_all_connectors(&fbdev->psb_fb_helper);
	drm_fb_helper_initial_config(&fbdev->psb_fb_helper, 32);
	return 0;
}

void psb_fbdev_fini(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)dev->dev_private;

	if(!dev_priv->fbdev) {
		return;
	}

	psb_fbdev_destroy(dev, dev_priv->fbdev);
	kfree(dev_priv->fbdev);
	dev_priv->fbdev = NULL;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
static int psbfb_multi_fb_probe_crtc(struct drm_device *dev,
				     struct drm_crtc *crtc)
{
	struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
	struct drm_framebuffer *fb = crtc->fb;
	struct psb_framebuffer *psbfb = to_psb_fb(crtc->fb);
	struct drm_connector *connector;
	struct fb_info *info;
	struct psbfb_par *par;
	struct drm_mode_set *modeset;
	unsigned int width, height;
	int new_fb = 0;
	int ret, i, conn_count;

	if (!drm_helper_crtc_in_use(crtc))
		return 0;

	if (!crtc->desired_mode)
		return 0;

	width = crtc->desired_mode->hdisplay;
	height = crtc->desired_mode->vdisplay;

	/* is there an fb bound to this crtc already */
	if (!psb_intel_crtc->mode_set.fb) {
		ret =
		    psbfb_create(dev, width, height, width, height,
				 &psbfb);
		if (ret)
			return -EINVAL;
		new_fb = 1;
	} else {
		fb = psb_intel_crtc->mode_set.fb;
		if ((fb->width < width) || (fb->height < height))
			return -EINVAL;
	}

	info = fb->fbdev;
	par = info->par;

	modeset = &psb_intel_crtc->mode_set;
	modeset->fb = fb;
	conn_count = 0;
	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		if (connector->encoder)
			if (connector->encoder->crtc == modeset->crtc) {
				modeset->connectors[conn_count] =
				    connector;
				conn_count++;
				if (conn_count > INTELFB_CONN_LIMIT)
					BUG();
			}
	}

	for (i = conn_count; i < INTELFB_CONN_LIMIT; i++)
		modeset->connectors[i] = NULL;

	par->crtc_ids[0] = crtc->base.id;

	modeset->num_connectors = conn_count;
	if (modeset->mode != modeset->crtc->desired_mode)
		modeset->mode = modeset->crtc->desired_mode;

	par->crtc_count = 1;

	if (new_fb) {
		info->var.pixclock = -1;
		if (register_framebuffer(info) < 0)
			return -EINVAL;
	} else
		psbfb_set_par(info);

	printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
	       info->fix.id);

	/* Switch back to kernel console on panic */
	panic_mode = *modeset;
	atomic_notifier_chain_register(&panic_notifier_list, &paniced);
	printk(KERN_INFO "registered panic notifier\n");

	return 0;
}

static int psbfb_multi_fb_probe(struct drm_device *dev)
{

	struct drm_crtc *crtc;
	int ret = 0;

	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		ret = psbfb_multi_fb_probe_crtc(dev, crtc);
		if (ret)
			return ret;
	}
	return ret;
}

static int psbfb_single_fb_probe(struct drm_device *dev)
{
	struct drm_crtc *crtc;
	struct drm_connector *connector;
	unsigned int fb_width = (unsigned) -1, fb_height = (unsigned) -1;
	unsigned int surface_width = 0, surface_height = 0;
	int new_fb = 0;
	int crtc_count = 0;
	int ret, i, conn_count = 0;
	struct fb_info *info;
	struct psbfb_par *par;
	struct drm_mode_set *modeset = NULL;
	struct drm_framebuffer *fb = NULL;
	struct psb_framebuffer *psbfb = NULL;

	PSB_DEBUG_ENTRY("\n");

	/* first up get a count of crtcs now in use and
	 * new min/maxes width/heights */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		if (drm_helper_crtc_in_use(crtc)) {
			if (crtc->desired_mode) {
				fb = crtc->fb;
				if (crtc->desired_mode->hdisplay <
				    fb_width)
					fb_width =
					    crtc->desired_mode->hdisplay;

				if (crtc->desired_mode->vdisplay <
				    fb_height)
					fb_height =
					    crtc->desired_mode->vdisplay;

				if (crtc->desired_mode->hdisplay >
				    surface_width)
					surface_width =
					    crtc->desired_mode->hdisplay;

				if (crtc->desired_mode->vdisplay >
				    surface_height)
					surface_height =
					    crtc->desired_mode->vdisplay;

			}
			crtc_count++;
		}
	}

	if (crtc_count == 0 || fb_width == -1 || fb_height == -1) {
		/* hmm everyone went away - assume VGA cable just fell out
		   and will come back later. */
		return 0;
	}

	/* do we have an fb already? */
	if (list_empty(&dev->mode_config.fb_kernel_list)) {
		/* create an fb if we don't have one */
		ret =
		    psbfb_create(dev, fb_width, fb_height, surface_width,
				 surface_height, &psbfb);
		if (ret)
			return -EINVAL;
		new_fb = 1;
		fb = &psbfb->base;
	} else {
		fb = list_first_entry(&dev->mode_config.fb_kernel_list,
				      struct drm_framebuffer, filp_head);

		/* if someone hotplugs something bigger than we have already
		 * allocated, we are pwned. As really we can't resize an
		 * fbdev that is in the wild currently due to fbdev not really
		 * being designed for the lower layers moving stuff around
		 * under it. - so in the grand style of things - punt. */
		if ((fb->width < surface_width)
		    || (fb->height < surface_height)) {
			DRM_ERROR
			("Framebuffer not large enough to scale"
			 " console onto.\n");
			return -EINVAL;
		}
	}

	info = fb->fbdev;
	par = info->par;

	crtc_count = 0;
	/* okay we need to setup new connector sets in the crtcs */
	list_for_each_entry(crtc, &dev->mode_config.crtc_list, head) {
		struct psb_intel_crtc *psb_intel_crtc = to_psb_intel_crtc(crtc);
		modeset = &psb_intel_crtc->mode_set;
		modeset->fb = fb;
		conn_count = 0;
		list_for_each_entry(connector,
				    &dev->mode_config.connector_list,
				    head) {
			if (connector->encoder)
				if (connector->encoder->crtc ==
				    modeset->crtc) {
					modeset->connectors[conn_count] =
					    connector;
					conn_count++;
					if (conn_count >
					    INTELFB_CONN_LIMIT)
						BUG();
				}
		}

		for (i = conn_count; i < INTELFB_CONN_LIMIT; i++)
			modeset->connectors[i] = NULL;

		par->crtc_ids[crtc_count++] = crtc->base.id;

		modeset->num_connectors = conn_count;
		if (modeset->mode != modeset->crtc->desired_mode)
			modeset->mode = modeset->crtc->desired_mode;
	}
	par->crtc_count = crtc_count;

	if (new_fb) {
		info->var.pixclock = -1;
		if (register_framebuffer(info) < 0)
			return -EINVAL;
	} else
		psbfb_set_par(info);

	printk(KERN_INFO "fb%d: %s frame buffer device\n", info->node,
	       info->fix.id);

	/* Switch back to kernel console on panic */
	panic_mode = *modeset;
	atomic_notifier_chain_register(&panic_notifier_list, &paniced);
	printk(KERN_INFO "registered panic notifier\n");

	return 0;
}

int psbfb_probe(struct drm_device *dev)
{
	int ret = 0;

	DRM_DEBUG("\n");

	/* something has changed in the lower levels of hell - deal with it
	   here */

	/* two modes : a) 1 fb to rule all crtcs.
	   b) one fb per crtc.
	   two actions 1) new connected device
	   2) device removed.
	   case a/1 : if the fb surface isn't big enough -
	   resize the surface fb.
	   if the fb size isn't big enough - resize fb into surface.
	   if everything big enough configure the new crtc/etc.
	   case a/2 : undo the configuration
	   possibly resize down the fb to fit the new configuration.
	   case b/1 : see if it is on a new crtc - setup a new fb and add it.
	   case b/2 : teardown the new fb.
	 */

	/* mode a first */
	/* search for an fb */
	if (0 /*i915_fbpercrtc == 1 */)
		ret = psbfb_multi_fb_probe(dev);
	else
		ret = psbfb_single_fb_probe(dev);

	return ret;
}
/*EXPORT_SYMBOL(psbfb_probe); */

#else /*KERNEL_VERSION >= 2.6.35*/

static void psbfb_output_poll_changed(struct drm_device * dev)
{
	struct drm_psb_private * dev_priv = (struct drm_psb_private *)dev->dev_private;
	struct psb_fbdev * fbdev = (struct psb_fbdev *)dev_priv->fbdev;
	drm_fb_helper_hotplug_event(&fbdev->psb_fb_helper);
}
#endif

int psbfb_remove(struct drm_device *dev, struct drm_framebuffer *fb)
{
	struct fb_info *info;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	struct psb_framebuffer * psbfb = to_psb_fb(fb);
#endif

	if (drm_psb_no_fb)
		return 0;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	info = psbfb->fbdev;
	psbfb->pvrBO = NULL;
#else
	info = fb->fbdev;
#endif

	if (info) {
		framebuffer_release(info);
	}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	atomic_notifier_chain_unregister(&panic_notifier_list, &paniced);
	memset(&panic_mode, 0, sizeof(struct drm_mode_set));
#endif
	return 0;
}
/*EXPORT_SYMBOL(psbfb_remove); */

static int psb_user_framebuffer_create_handle(struct drm_framebuffer *fb,
					      struct drm_file *file_priv,
					      unsigned int *handle)
{
	struct psb_framebuffer *psbfb = to_psb_fb(fb);
	(void) file_priv;
	*handle = (unsigned int)psbfb->hKernelMemInfo;
	return 0;
}

static void psb_user_framebuffer_destroy(struct drm_framebuffer *fb)
{
	struct drm_device *dev = fb->dev;
	struct psb_framebuffer *psbfb = to_psb_fb(fb);

	/*ummap gtt pages*/
	psb_gtt_unmap_meminfo(dev, psbfb->hKernelMemInfo);
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	if (fb->fbdev)
#else
	if (psbfb->fbdev)
#endif
	{
		psbfb_remove(dev, fb);
	}

	/* JB: TODO not drop, refcount buffer */
	drm_framebuffer_cleanup(fb);

	kfree(fb);
}

static const struct drm_mode_config_funcs psb_mode_funcs = {
	.fb_create = psb_user_framebuffer_create,
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(2,6,35))
	.output_poll_changed = psbfb_output_poll_changed,
#else
	.fb_changed = psbfb_probe,
#endif
};

static int psb_create_backlight_property(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv
				= (struct drm_psb_private *) dev->dev_private;
	struct drm_property *backlight;

	if (dev_priv->backlight_property)
		return 0;

	backlight = drm_property_create(dev,
			DRM_MODE_PROP_RANGE,
			"backlight",
			2);

	if (unlikely(!backlight)) {
		pr_err("%s: faild to create backlight property\n", __func__);
		return 0;
	}
	backlight->values[0] = 0;
	backlight->values[1] = 100;

	dev_priv->backlight_property = backlight;

	return 0;
}

static int psb_intel_connector_clones(struct drm_device *dev, int type_mask)
{
	int index_mask = 0;
	struct drm_connector *connector;
	int entry = 0;

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		struct psb_intel_output *psb_intel_output =
		    to_psb_intel_output(connector);
		if (type_mask & (1 << psb_intel_output->type))
			index_mask |= (1 << entry);
		entry++;
	}
	return index_mask;
}

static void psb_setup_outputs(struct drm_device *dev)
{
	struct drm_connector *connector;

	PSB_DEBUG_ENTRY("\n");

	drm_mode_create_scaling_mode_property(dev);

	psb_create_backlight_property(dev);

	mdfld_output_init(dev);

	list_for_each_entry(connector, &dev->mode_config.connector_list,
			    head) {
		struct psb_intel_output *psb_intel_output =
		    to_psb_intel_output(connector);
		struct drm_encoder *encoder = &psb_intel_output->enc;
		int crtc_mask = 0, clone_mask = 0;

		/* valid crtcs */
		switch (psb_intel_output->type) {
		case INTEL_OUTPUT_SDVO:
			crtc_mask = ((1 << 0) | (1 << 1));
			clone_mask = (1 << INTEL_OUTPUT_SDVO);
			break;
		case INTEL_OUTPUT_LVDS:
			PSB_DEBUG_ENTRY("LVDS. \n");
			if (IS_MRST(dev))
				crtc_mask = (1 << 0);
			else
				crtc_mask = (1 << 1);

			clone_mask = (1 << INTEL_OUTPUT_LVDS);
			break;
		case INTEL_OUTPUT_MIPI:
			PSB_DEBUG_ENTRY("MIPI. \n");
			crtc_mask = (1 << 0);
			clone_mask = (1 << INTEL_OUTPUT_MIPI);
			break;
		case INTEL_OUTPUT_MIPI2:
			PSB_DEBUG_ENTRY("MIPI2. \n");
			crtc_mask = (1 << 2);
			clone_mask = (1 << INTEL_OUTPUT_MIPI2);
			break;
		case INTEL_OUTPUT_HDMI:
			PSB_DEBUG_ENTRY("HDMI. \n");
			crtc_mask = (1 << 1);
			clone_mask = (1 << INTEL_OUTPUT_HDMI);
			break;
		}

		encoder->possible_crtcs = crtc_mask;
		encoder->possible_clones =
		    psb_intel_connector_clones(dev, clone_mask);

	}
}

static void *psb_bo_from_handle(struct drm_device *dev,
				struct drm_file *file_priv,
				unsigned int handle)
{
	PVRSRV_KERNEL_MEM_INFO *psKernelMemInfo = IMG_NULL;
	IMG_HANDLE hKernelMemInfo = (IMG_HANDLE)handle;
	int ret;
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *) dev->dev_private;

	ret = psb_get_meminfo_by_handle(dev_priv,
			hKernelMemInfo, &psKernelMemInfo);
	if (ret) {
		DRM_ERROR("Cannot get meminfo for handle %p\n",
			hKernelMemInfo);
		return NULL;
	}

	return (void *)psKernelMemInfo;
}

static size_t psb_bo_size(struct drm_device *dev, void *bof)
{
	PVRSRV_KERNEL_MEM_INFO *psKernelMemInfo	= (PVRSRV_KERNEL_MEM_INFO *)bof;
	return (size_t)psKernelMemInfo->uAllocSize;
}

static size_t psb_bo_offset(struct drm_device *dev, void *bof)
{
	struct psb_framebuffer *psbfb
		= (struct psb_framebuffer *)bof;

	return (size_t)psbfb->offset;
}

static int psb_bo_pin_for_scanout(struct drm_device *dev, void *bo)
{
	 return 0;
}

static int psb_bo_unpin_for_scanout(struct drm_device *dev, void *bo)
{
	return 0;
}

void psb_modeset_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    (struct drm_psb_private *) dev->dev_private;
	struct psb_intel_mode_device *mode_dev = &dev_priv->mode_dev;
	int i;

	PSB_DEBUG_ENTRY("\n");

	/* Init mm functions */
	mode_dev->bo_from_handle = psb_bo_from_handle;
	mode_dev->bo_size = psb_bo_size;
	mode_dev->bo_offset = psb_bo_offset;
	mode_dev->bo_pin_for_scanout = psb_bo_pin_for_scanout;
	mode_dev->bo_unpin_for_scanout = psb_bo_unpin_for_scanout;

	drm_mode_config_init(dev);

	dev->mode_config.min_width = 0;
	dev->mode_config.min_height = 0;

	dev->mode_config.funcs = (void *) &psb_mode_funcs;

	/* set memory base */
	/* MRST and PSB should use BAR 2*/
	pci_read_config_dword(dev->pdev, PSB_BSM,
			(uint32_t *)&(dev->mode_config.fb_base));

	for (i = 0; i < dev_priv->num_pipe; i++)
		psb_intel_crtc_init(dev, i, mode_dev);

	dev->mode_config.max_width = dev->mode_config.num_crtc * MDFLD_PLANE_MAX_WIDTH;
	dev->mode_config.max_height = dev->mode_config.num_crtc * MDFLD_PLANE_MAX_HEIGHT;

	psb_setup_outputs(dev);
}

void psb_modeset_cleanup(struct drm_device *dev)
{
#if (LINUX_VERSION_CODE < KERNEL_VERSION(2,6,35))
	drm_mode_config_cleanup(dev);
#else
	mutex_lock(&dev->struct_mutex);

	drm_kms_helper_poll_fini(dev);
	psb_fbdev_fini(dev);

	drm_mode_config_cleanup(dev);

	mutex_unlock(&dev->struct_mutex);
#endif
}
