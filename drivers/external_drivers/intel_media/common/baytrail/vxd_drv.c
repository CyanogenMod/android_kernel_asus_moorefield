/**************************************************************************
 * Copyright (c) 2007-2008, Intel Corporation.
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

#include <linux/device.h>
#include <linux/version.h>
#include "drmP.h"
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
#include "drm.h"
#else
#include <uapi/drm/drm.h>
#endif

#include "i915_drm.h"
#include "i915_drv.h"
#include "vxd_drv.h"
#include "vxd_drm.h"

#include <linux/console.h>
#include <linux/module.h>
#include <asm/uaccess.h>
#include <linux/intel_mid_pm.h>
#include <asm/intel_mid_pcihelpers.h>

extern int drm_psb_trap_pagefaults;

int drm_psb_cpurelax;
int drm_psb_udelaydivider = 1;
int drm_psb_priv_pmu_func = 0;

static struct pci_dev *pci_root;

int drm_psb_trap_pagefaults;

extern struct drm_device *i915_drm_dev;
int drm_psb_msvdx_tiling;

atomic_t g_videodec_access_count;

static void vxd_power_init(struct drm_device *dev);
static void vxd_power_post_init(struct drm_device *dev);

extern int pmc_nc_set_power_state(int islands, int state_type, int reg);
extern int pmc_nc_get_power_state(int islands, int reg);

module_param_named(trap_pagefaults, drm_psb_trap_pagefaults, int, 0600);

int drm_msvdx_pmpolicy = PSB_PMPOLICY_POWERDOWN;
int drm_msvdx_bottom_half = PSB_BOTTOM_HALF_WQ;
module_param_named(msvdx_pmpolicy, drm_msvdx_pmpolicy, int, 0600);
module_param_named(priv_pmu_func, drm_psb_priv_pmu_func, int, 0600);
MODULE_PARM_DESC(msvdx_pmpolicy,
		"control d0i3 of msvdx "
		"(default: 2"
		"0 - d0i3 is disabled"
		"1 - clockgating is enabled"
		"2 - powerdown is enabled)");
MODULE_PARM_DESC(priv_pmu_func,
		"Use private PMU function or not "
		"(default: 0"
		"0 - disable"
		"1 - enable)");

int drm_psb_debug = 0x0;
module_param_named(psb_debug, drm_psb_debug, int, 0600);
MODULE_PARM_DESC(psb_debug,
		"control debug info output"
		"default: 0"
		"0:PSB_D_GENERAL, 1:PSB_D_INIT, 2:PSB_D_IRQ, 3:PSB_D_ENTRY"
		"6:PSB_D_PM, 8:PSB_D_REG, 9:PSB_D_MSVDX, 13:PSB_D_WARN");

#define DRM_IOCTL_PSB_EXTENSION			\
	DRM_IOWR(DRM_PSB_EXTENSION, 		\
			 union drm_psb_extension_arg)
#define DRM_IOCTL_PSB_GETPAGEADDRS		\
	DRM_IOWR(DRM_PSB_GETPAGEADDRS,	\
			 struct drm_psb_getpageaddrs_arg)
#define DRM_IOCTL_PSB_CMDBUF			\
	DRM_IOW(DRM_PSB_CMDBUF,		\
			struct drm_psb_cmdbuf_arg)
#define DRM_IOCTL_PSB_TTM_PL_CREATE		\
	DRM_IOWR(DRM_PSB_TTM_PL_CREATE,	\
		 union ttm_pl_create_arg)
#define DRM_IOCTL_PSB_TTM_PL_REFERENCE		\
	DRM_IOWR(DRM_PSB_TTM_PL_REFERENCE,	\
		 union ttm_pl_reference_arg)
#define DRM_IOCTL_PSB_TTM_PL_UNREF		\
	DRM_IOW(DRM_PSB_TTM_PL_UNREF,	\
		struct ttm_pl_reference_req)
#define DRM_IOCTL_PSB_TTM_PL_SYNCCPU		\
	DRM_IOW(DRM_PSB_TTM_PL_SYNCCPU,	\
		struct ttm_pl_synccpu_arg)
#define DRM_IOCTL_PSB_TTM_PL_WAITIDLE		\
	DRM_IOW(DRM_PSB_TTM_PL_WAITIDLE,	\
		struct ttm_pl_waitidle_arg)
#define DRM_IOCTL_PSB_TTM_PL_SETSTATUS		\
	DRM_IOWR(DRM_PSB_TTM_PL_SETSTATUS,	\
		 union ttm_pl_setstatus_arg)
#define DRM_IOCTL_PSB_TTM_PL_CREATE_UB		\
	DRM_IOWR(DRM_PSB_TTM_PL_CREATE_UB,	\
		 union ttm_pl_create_ub_arg)
#define DRM_IOCTL_PSB_TTM_FENCE_SIGNALED	\
	DRM_IOWR(DRM_PSB_TTM_FENCE_SIGNALED,	\
		  union ttm_fence_signaled_arg)
#define DRM_IOCTL_PSB_TTM_FENCE_FINISH	\
	DRM_IOWR(DRM_PSB_TTM_FENCE_FINISH,	\
			 union ttm_fence_finish_arg)
#define DRM_IOCTL_PSB_TTM_FENCE_UNREF		\
	DRM_IOW(DRM_PSB_TTM_FENCE_UNREF,	\
		 struct ttm_fence_unref_arg)
#define DRM_IOCTL_PSB_VIDEO_GETPARAM		\
	DRM_IOWR(DRM_PSB_VIDEO_GETPARAM, \
		 struct drm_lnc_video_getparam_arg)

#define VXD_IOCTL_DEF_DRV(ioctl, _func, _flags)			\
	[DRM_IOCTL_NR(DRM_##ioctl) - DRM_COMMAND_VXD_OFFSET] =	\
	{.cmd = DRM_##ioctl - DRM_COMMAND_VXD_OFFSET, .func = _func, .flags = _flags, .cmd_drv = DRM_IOCTL_##ioctl}

struct drm_ioctl_desc vxd_ioctls[] = {
	VXD_IOCTL_DEF_DRV(PSB_EXTENSION, psb_extension_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_GETPAGEADDRS, psb_getpageaddrs_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_CMDBUF, psb_cmdbuf_ioctl,
			DRM_AUTH | DRM_UNLOCKED),
	VXD_IOCTL_DEF_DRV(PSB_TTM_PL_CREATE, psb_pl_create_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_PL_REFERENCE, psb_pl_reference_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_PL_UNREF, psb_pl_unref_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_PL_SYNCCPU, psb_pl_synccpu_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_PL_WAITIDLE, psb_pl_waitidle_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_PL_SETSTATUS, psb_pl_setstatus_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_PL_CREATE_UB, psb_pl_ub_create_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_FENCE_SIGNALED, psb_fence_signaled_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_FENCE_FINISH, psb_fence_finish_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_TTM_FENCE_UNREF, psb_fence_unref_ioctl,
			DRM_AUTH),
	VXD_IOCTL_DEF_DRV(PSB_VIDEO_GETPARAM, psb_video_getparam,
			DRM_AUTH | DRM_UNLOCKED),
};

int vxd_max_ioctl = DRM_ARRAY_SIZE(vxd_ioctls);

struct psb_fpriv *psb_fpriv(struct drm_file *file_priv)
{
	struct drm_i915_file_private *i915_file_priv =
		(struct drm_i915_file_private *)file_priv->driver_priv;
#ifdef CONFIG_DRM_VXD_BYT
	return i915_file_priv->pPriv;
#else
	return NULL;
#endif
}

struct drm_psb_private *psb_priv(struct drm_device *dev)
{
	struct drm_i915_private *i915_dev_priv = dev->dev_private;
	return i915_dev_priv->vxd_priv;
}

int vxd_release(struct inode *inode, struct file *filp)
{
	struct drm_file *file_priv;
	struct drm_i915_file_private *i915_file_priv;
	struct psb_fpriv *psb_fp;
	struct drm_psb_private *dev_priv;
	struct msvdx_private *msvdx_priv;
	int i;

	file_priv = (struct drm_file *)filp->private_data;
	i915_file_priv = file_priv->driver_priv;
	psb_fp = i915_file_priv->pPriv;

	dev_priv = psb_priv(file_priv->minor->dev);
	msvdx_priv = (struct msvdx_private *)dev_priv->msvdx_private;

#if 0
	/*cleanup for msvdx */
	if (msvdx_priv->tfile == BCVideoGetPriv(file_priv)->tfile) {
		msvdx_priv->fw_status = 0;
		msvdx_priv->host_be_opp_enabled = 0;
		memset(&msvdx_priv->frame_info, 0,
		       sizeof(struct drm_psb_msvdx_frame_info) *
		       MAX_DECODE_BUFFERS);
	}
#endif
#ifdef CONFIG_VIDEO_MRFLD_EC
	for (i = 0; i < PSB_MAX_EC_INSTANCE; i++) {
		if (msvdx_priv->msvdx_ec_ctx[i]->tfile == psb_fp->tfile) {
			msvdx_priv->msvdx_ec_ctx[i]->tfile = NULL;
			msvdx_priv->msvdx_ec_ctx[i]->context_id = 0;
			msvdx_priv->msvdx_ec_ctx[i]->fence = PSB_MSVDX_INVALID_FENCE;
		}
	}
#endif
	ttm_object_file_release(&psb_fp->tfile);
	kfree(psb_fp);

	/* remove video context */
	/* psb_remove_videoctx(dev_priv, filp); */
	return 0;
}

static struct vm_operations_struct psb_ttm_vm_ops;

int psb_mmap(struct file *filp, struct vm_area_struct *vma)
{
	struct drm_file *file_priv;
	struct drm_psb_private *dev_priv;
	int ret;

	file_priv = (struct drm_file *) filp->private_data;
	dev_priv = psb_priv(file_priv->minor->dev);

	ret = ttm_bo_mmap(filp, vma, &dev_priv->bdev);
	if (unlikely(ret != 0))
		return ret;

	if (unlikely(dev_priv->ttm_vm_ops == NULL)) {
		dev_priv->ttm_vm_ops = (struct vm_operations_struct *)vma->vm_ops;
		psb_ttm_vm_ops = *vma->vm_ops;
		psb_ttm_vm_ops.fault = &psb_ttm_fault;
	}

	vma->vm_ops = &psb_ttm_vm_ops;

	return 0;
}

static void psb_do_takedown(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv =
	    ((struct drm_i915_private *)dev->dev_private)->vxd_priv;
	struct ttm_bo_device *bdev = &dev_priv->bdev;

	if (dev_priv->have_mem_mmu) {
		ttm_bo_clean_mm(bdev, DRM_PSB_MEM_MMU);
		dev_priv->have_mem_mmu = 0;
	}

	psb_msvdx_uninit(dev);
}

static int  __vxd_driver_unload()
{
	struct drm_i915_private *i915_dev_priv = i915_drm_dev->dev_private;
	struct drm_psb_private *dev_priv = i915_dev_priv->vxd_priv;

	if (dev_priv) {
		psb_do_takedown(i915_dev_priv->dev);

		if (dev_priv->pf_pd) {
			psb_mmu_free_pagedir(dev_priv->pf_pd);
			dev_priv->pf_pd = NULL;
		}
		if (dev_priv->mmu) {
			psb_mmu_driver_takedown(dev_priv->mmu);
			dev_priv->mmu = NULL;
		}

		if (dev_priv->has_bo_device) {
			ttm_bo_device_release(&dev_priv->bdev);
			dev_priv->has_bo_device = 0;
		}
		if (dev_priv->has_fence_device) {
			ttm_fence_device_release(&dev_priv->fdev);
			dev_priv->has_fence_device = 0;
		}
#if 0
		if (dev_priv->msvdx_reg) {
			iounmap(dev_priv->msvdx_reg);
			dev_priv->msvdx_reg = NULL;
		}
#endif
		if (dev_priv->tdev)
			ttm_object_device_release(&dev_priv->tdev);

		if (dev_priv->has_global)
			psb_ttm_global_release(dev_priv);

		kfree(dev_priv);
		i915_dev_priv->vxd_priv = NULL;
	}
#if 0
	ospm_power_uninit();
#endif
	return 0;
}

static int __exit vxd_driver_unload()
{
	return __vxd_driver_unload();
}

#define PCI_ROOT_MSGBUS_CTRL_REG	0xD0
#define PCI_ROOT_MSGBUS_DATA_REG	0xD4
#define PCI_ROOT_MSGBUS_CTRL_EXT_REG	0xD8
#define PCI_ROOT_MSGBUS_READ		0x10
#define PCI_ROOT_MSGBUS_WRITE		0x11
#define PCI_ROOT_MSGBUS_DWORD_ENABLE	0xf0
#define PUNIT_PORT			0x04
#define VEDSSPM0 			0x32
#define VEDSSPM1 			0x33
#define VEDSSC				0x1

u32 intel_mid_msgbus_read32_vxd(u8 port, u32 addr)
{
	unsigned long irq_flags;
	u32 data;
	u32 cmd;
	u32 cmdext;

	cmd = (PCI_ROOT_MSGBUS_READ << 24) | (port << 16) |
		((addr & 0xff) << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;
	cmdext = addr & 0xffffff00;

	data = intel_mid_msgbus_read32_raw_ext(cmd, cmdext);

	return data;
}

void intel_mid_msgbus_write32_vxd(u8 port, u32 addr, u32 data)
{
	unsigned long irq_flags;
	u32 cmd;
	u32 cmdext;

	cmd = (PCI_ROOT_MSGBUS_WRITE << 24) | (port << 16) |
		((addr & 0xFF) << 8) | PCI_ROOT_MSGBUS_DWORD_ENABLE;
	cmdext = addr & 0xffffff00;

	intel_mid_msgbus_write32_raw_ext(cmd, cmdext, data);
}

static int __init vxd_driver_load()
{
	struct drm_i915_private *i915_dev_priv;
	struct drm_psb_private *dev_priv;
	struct ttm_bo_device *bdev;
	int ret = -ENOMEM;
	uint32_t pwr_sts;

	/* Check if DRM device is loaded first */
	if (!i915_drm_dev)
		return -ENODEV;

	i915_dev_priv = i915_drm_dev->dev_private;

	dev_priv = kzalloc(sizeof(*dev_priv), GFP_KERNEL);
	if (dev_priv == NULL)
		return -ENOMEM;
	INIT_LIST_HEAD(&dev_priv->video_ctx);
	spin_lock_init(&dev_priv->video_ctx_lock);
	dev_priv->dev = i915_dev_priv->dev;
	bdev = &dev_priv->bdev;

	ret = psb_ttm_global_init(dev_priv);
	if (unlikely(ret != 0))
		goto out_err;
	dev_priv->has_global = 1;

	dev_priv->tdev = ttm_object_device_init
	    (dev_priv->mem_global_ref.object, PSB_OBJECT_HASH_ORDER);
	if (unlikely(dev_priv->tdev == NULL))
		goto out_err;
	mutex_init(&dev_priv->cmdbuf_mutex);
	INIT_LIST_HEAD(&dev_priv->decode_context.validate_list);
	spin_lock_init(&dev_priv->reloc_lock);

	i915_dev_priv->vxd_priv = dev_priv;

	dev_priv->msvdx_reg = i915_dev_priv->regs + 0x170000;
	DRM_INFO("%s 3, i915_dev_priv->regs is 0x%x.\n",
				__func__, i915_dev_priv->regs);
	DRM_INFO("0 MSVDX_CORE_REV_OFFSET value is 0x%x.\n",
				readl(i915_dev_priv->regs + 0x170640));
	DRM_INFO("1 MSVDX_CORE_REV_OFFSET value is 0x%x.\n",
				PSB_RMSVDX32(MSVDX_CORE_REV_OFFSET));

	vxd_power_init(i915_dev_priv->dev);
	i915_dev_priv->vxd_driver_open = vxd_driver_open;
	i915_dev_priv->vxd_lastclose = vxd_lastclose;
	i915_dev_priv->vxd_ioctl = vxd_ioctl;
	i915_dev_priv->vxd_release = vxd_release;
	i915_dev_priv->psb_mmap = psb_mmap;
	i915_dev_priv->psb_msvdx_interrupt = psb_msvdx_interrupt;

#if 0
	get_imr_info(dev_priv);

	/* Init OSPM support */
	ospm_power_init(dev);
#endif

	ret = psb_ttm_fence_device_init(&dev_priv->fdev);
	if (unlikely(ret != 0))
		goto out_err;

	/* For VXD385 DE2.x firmware support 16bit fence value */
	dev_priv->fdev.fence_class[PSB_ENGINE_VIDEO].wrap_diff =
	    (1 << 14);
	dev_priv->fdev.fence_class[PSB_ENGINE_VIDEO].flush_diff =
	    (1 << 13);
	dev_priv->fdev.fence_class[PSB_ENGINE_VIDEO].sequence_mask =
	    0x0000ffff;

	dev_priv->has_fence_device = 1;
	ret = ttm_bo_device_init(bdev,
				 dev_priv->bo_global_ref.ref.object,
				 &psb_ttm_bo_driver,
				 DRM_PSB_FILE_PAGE_OFFSET, true);
	if (unlikely(ret != 0))
		goto out_err;
	dev_priv->has_bo_device = 1;
	ttm_lock_init(&dev_priv->ttm_lock);

	dev_priv->mmu = psb_mmu_driver_init((void *)0,
					    drm_psb_trap_pagefaults, 0,
					    dev_priv, IMG_MMU);
	if (!dev_priv->mmu)
		goto out_err;

	dev_priv->pf_pd = psb_mmu_alloc_pd(dev_priv->mmu, 1, 0);
	if (!dev_priv->pf_pd)
		goto out_err;

	psb_mmu_set_pd_context(psb_mmu_get_default_pd(dev_priv->mmu), 0);
	psb_mmu_set_pd_context(dev_priv->pf_pd, 1);

	spin_lock_init(&dev_priv->sequence_lock);

	PSB_DEBUG_INIT("Begin to init MSVDX.\n");

	/*
	 * Initialize sequence numbers for the different command
	 * submission mechanisms.
	 */
	dev_priv->sequence[PSB_ENGINE_VIDEO] = 1;

	if (!ttm_bo_init_mm(bdev,
			    DRM_PSB_MEM_MMU, PSB_MEM_TT_START >> PAGE_SHIFT)) {
		dev_priv->have_mem_mmu = 1;
		dev_priv->sizes.mmu_size = PSB_MEM_TT_START / (1024 * 1024);
	}

        /* Create tiling MMU region managed by TTM */
        if (!ttm_bo_init_mm(bdev, DRM_PSB_MEM_MMU_TILING,
			    (0x10000000) >> PAGE_SHIFT))
                dev_priv->have_mem_mmu_tiling = 1;

	PSB_DEBUG_INIT("Init MSVDX\n");
	psb_msvdx_init(i915_dev_priv->dev);

	vxd_power_post_init(i915_dev_priv->dev);

#if 0
	ospm_post_init(dev);
#endif
	/* enable msvdx tiling on BYT */
	drm_psb_msvdx_tiling = 1;
	return 0;
out_err:
	__vxd_driver_unload();
	return ret;

}

void vxd_lastclose(struct drm_device *dev)
{
	struct msvdx_private *msvdx_priv;
	struct drm_psb_private *dev_priv = psb_priv(dev);
	if (!dev_priv)
		return;

	msvdx_priv = dev_priv->msvdx_private;
	if (msvdx_priv) {
		mutex_lock(&msvdx_priv->msvdx_mutex);
		if (dev_priv->decode_context.buffers) {
			vfree(dev_priv->decode_context.buffers);
			dev_priv->decode_context.buffers = NULL;
		}
		mutex_unlock(&msvdx_priv->msvdx_mutex);
	}
}

int vxd_driver_open(struct drm_device *dev, struct drm_file *file)
{
	struct psb_fpriv *psb_fp;
	struct drm_psb_private *dev_priv;
	struct drm_i915_file_private *file_priv;
	dev_priv = psb_priv(dev);
	file_priv = file->driver_priv;
	psb_fp = kzalloc(sizeof(*psb_fp), GFP_KERNEL);
	if (unlikely(psb_fp == NULL))
		return -ENOMEM;
	psb_fp->tfile = ttm_object_file_init(dev_priv->tdev,
					     PSB_FILE_OBJECT_HASH_ORDER);
	if (unlikely(psb_fp->tfile == NULL)) {
		kfree(psb_fp);
		return -EINVAL;
	}
	if (unlikely(dev_priv->bdev.dev_mapping == NULL))
		dev_priv->bdev.dev_mapping = dev_priv->dev->dev_mapping;
#ifdef CONFIG_DRM_VXD_BYT
	file_priv->pPriv = psb_fp;
#endif
	return 0;
}

int psb_extension_ioctl(struct drm_device *dev, void *data,
			struct drm_file *file_priv)
{
	union drm_psb_extension_arg *arg = data;
	struct drm_psb_extension_rep *rep = &arg->rep;

	if (strcmp(arg->extension, "psb_ttm_placement_alphadrop") == 0) {
		rep->exists = 1;
		rep->driver_ioctl_offset = DRM_PSB_PLACEMENT_OFFSET;
		rep->sarea_offset = 0;
		rep->major = 1;
		rep->minor = 0;
		rep->pl = 0;
		return 0;
	}
	if (strcmp(arg->extension, "psb_ttm_fence_alphadrop") == 0) {
		rep->exists = 1;
		rep->driver_ioctl_offset = DRM_PSB_FENCE_OFFSET;
		rep->sarea_offset = 0;
		rep->major = 1;
		rep->minor = 0;
		rep->pl = 0;
		return 0;
	}
	if (strcmp(arg->extension, "psb_ttm_execbuf_alphadrop") == 0) {
		rep->exists = 1;
		rep->driver_ioctl_offset = DRM_PSB_CMDBUF;
		rep->sarea_offset = 0;
		rep->major = 1;
		rep->minor = 0;
		rep->pl = 0;
		return 0;
	}

	/* return the video rar offset */
	if (strcmp(arg->extension, "lnc_video_getparam") == 0) {
		rep->exists = 1;
		rep->driver_ioctl_offset = DRM_PSB_VIDEO_GETPARAM;
		rep->sarea_offset = 0;
		rep->major = 1;
		rep->minor = 0;
		rep->pl = 0;
		return 0;
	}

	rep->exists = 0;
	return 0;
}

/**
 * Called whenever a 32-bit process running under a 64-bit kernel
 * performs an ioctl on /dev/dri/card<n>.
 *
 * \param filp file pointer.
 * \param cmd command.
 * \param arg user argument.
 * \return zero on success or negative number on failure.
 */
long vxd_ioctl(struct file *filp,
	      unsigned int cmd, unsigned long arg)
{
	struct drm_file *file_priv = filp->private_data;
	struct drm_device *dev;
	struct drm_ioctl_desc *ioctl;
	drm_ioctl_t *func;
	unsigned int nr = DRM_IOCTL_NR(cmd);
	int retcode = -EINVAL;
	char stack_kdata[128];
	char *kdata = NULL;
	unsigned int usize, asize;
	u32 drv_size;

	dev = file_priv->minor->dev;

	if ((nr < DRM_COMMAND_VXD_BASE) ||
		(nr >= DRM_COMMAND_VXD_BASE + DRM_ARRAY_SIZE(vxd_ioctls)))
		return drm_ioctl(filp, cmd, arg);

	if (drm_device_is_unplugged(dev))
		return -ENODEV;

	/* workaround drm authentification issue on Android
	* don't need following check for DRM_AUTH
	* otherwise maybe it will be reset before the check
	*/
	file_priv->authenticated = 1;

	DRM_DEBUG("pid=%d, cmd=0x%02x, nr=0x%02x, dev 0x%lx, auth=%d\n",
		  task_pid_nr(current), cmd, nr,
		  (long)old_encode_dev(file_priv->minor->device),
		  file_priv->authenticated);

	ioctl = &vxd_ioctls[nr - DRM_COMMAND_VXD_BASE];
	drv_size = _IOC_SIZE(ioctl->cmd_drv);
	usize = asize = _IOC_SIZE(cmd);
	if (drv_size > asize)
		asize = drv_size;

	atomic_inc(&dev->ioctl_count);
	atomic_inc(&dev->counts[_DRM_STAT_IOCTLS]);
	++file_priv->ioctl_count;

	/* Do not trust userspace, use our own definition */
	func = ioctl->func;

	if (!func) {
		DRM_DEBUG("no function\n");
		retcode = -EINVAL;
	} else {
		if (cmd & (IOC_IN | IOC_OUT)) {
			if (asize <= sizeof(stack_kdata)) {
				kdata = stack_kdata;
			} else {
				kdata = kmalloc(asize, GFP_KERNEL);
				if (!kdata) {
					retcode = -ENOMEM;
					goto err_i1;
				}
			}
			if (asize > usize)
				memset(kdata + usize, 0, asize - usize);

			if (cmd & IOC_IN) {
				if (copy_from_user(kdata, (void __user *)arg,
						   usize) != 0) {
					retcode = -EFAULT;
					goto err_i1;
				}
			} else {
				memset(kdata, 0, usize);
			}
		}

		if (ioctl->flags & DRM_UNLOCKED)
			retcode = func(dev, kdata, file_priv);
		else {
			mutex_lock(&drm_global_mutex);
			retcode = func(dev, kdata, file_priv);
			mutex_unlock(&drm_global_mutex);
		}

		if ((cmd & IOC_OUT) && kdata) {
			if (copy_to_user((void __user *)arg, kdata,
					 usize) != 0)
				retcode = -EFAULT;
		}
	}

err_i1:
	if (kdata != stack_kdata)
		kfree(kdata);
	atomic_dec(&dev->ioctl_count);
	if (retcode)
		DRM_DEBUG("ret = %d\n", retcode);
	return retcode;
}

static bool vxd_power_down(struct drm_device *dev)
{
	uint32_t pwr_sts;
	PSB_DEBUG_PM("MSVDX: power off msvdx.\n");
	if (drm_psb_priv_pmu_func) {
		intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D3);
		udelay(10);
		pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		while (pwr_sts != 0x03000003) {
			intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D3);
			udelay(10);
			pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		}
		return true;
	}
	else {
		if (pmc_nc_set_power_state(VEDSSC, OSPM_ISLAND_DOWN, VEDSSPM0)) {
			PSB_DEBUG_PM("VED: pmu_nc_set_power_state DOWN failed!\n");
			return false;
		}
		return true;
	}
}

static bool vxd_power_on(struct drm_device *dev)
{
	uint32_t pwr_sts;
	PSB_DEBUG_PM("MSVDX: power on msvdx.\n");
	if (drm_psb_priv_pmu_func) {
		intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D0);
		udelay(10);
		pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		while (pwr_sts != 0x0) {
			intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D0);
			udelay(10);
			pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		}

		intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D3);
		udelay(10);
		pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		while (pwr_sts != 0x03000003) {
			intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D3);
			udelay(10);
			pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		}

		intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D0);
		udelay(10);
		pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		while (pwr_sts != 0x0) {
			intel_mid_msgbus_write32_vxd(PUNIT_PORT, VEDSSPM0, VXD_APM_STS_D0);
			udelay(10);
			pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
		}
		return true;
	}
	else {
		if (pmc_nc_set_power_state(VEDSSC, OSPM_ISLAND_UP, VEDSSPM0)) {
			PSB_DEBUG_PM("VED: pmu_nc_set_power_state ON failed!\n");
			return false;
		}
		return true;
	}
}

static void vxd_power_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	PSB_DEBUG_PM("MSVDX: vxd power init.\n");
	pci_root = pci_get_bus_and_slot(0, PCI_DEVFN(0, 0));
	if (!pci_root) {
		printk(KERN_ALERT "%s: Error: msgbus PCI handle NULL",
			__func__);
	}
	mutex_init(&dev_priv->vxd_pm_mutex);
	vxd_power_on(dev);
}

static void vxd_power_post_init(struct drm_device *dev)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	/* need power down msvdx after init */
	vxd_power_down(dev);
	msvdx_priv->msvdx_needs_reset = 1;
}


/**
 * is_island_on
 *
 * Description: checks to see if the island is up
 * returns true if hw_island is ON
 * returns false if hw_island is OFF
 */
bool is_vxd_on()
{
	uint32_t pwr_sts;
	if (drm_psb_priv_pmu_func)
		pwr_sts = intel_mid_msgbus_read32_vxd(PUNIT_PORT, VEDSSPM0);
	else
		pwr_sts = pmc_nc_get_power_state(VEDSSC, VEDSSPM0);

	if (pwr_sts == VXD_APM_STS_D0)
		return true;
	else
		return false;
}

int ospm_apm_power_down_msvdx(struct drm_device *dev, int force_off)
{
	struct drm_psb_private *dev_priv = psb_priv(dev);
	struct msvdx_private *msvdx_priv = dev_priv->msvdx_private;
	PSB_DEBUG_PM("MSVDX: work queue is scheduled to power off msvdx.\n");
	int ret = 0;
	mutex_lock(&dev_priv->vxd_pm_mutex);
	if (force_off)
		goto power_off;

	if (atomic_read(&g_videodec_access_count)) {
		ret = -EBUSY;
		PSB_DEBUG_PM("g_videodec_access_count has been set.\n");
		goto out;
	}

	if (is_vxd_on() == false) {
		PSB_DEBUG_PM("vxd already is in power off.\n");
		goto out;
	}

	if (psb_check_msvdx_idle(dev)) {
		ret = -EBUSY;
		goto out;
	}

	psb_msvdx_save_context(dev);

power_off:
	vxd_power_down(dev);
#ifdef CONFIG_PM_RUNTIME
	i915_rpm_put_vxd(dev);
#endif
	/* MSVDX_NEW_PMSTATE(dev, msvdx_priv, PSB_PMSTATE_POWERDOWN); */

out:
	mutex_unlock(&dev_priv->vxd_pm_mutex);
	return ret;
}

bool ospm_power_using_video_begin(int hw_island)
{
	struct pci_dev *pdev = i915_drm_dev->pdev;
	struct drm_psb_private *dev_priv = psb_priv(i915_drm_dev);

	PSB_DEBUG_PM("MSVDX: %s is called.\n", __func__);
	if (hw_island != OSPM_VIDEO_DEC_ISLAND) {
		DRM_ERROR("failed.\n");
		return true;
	}

	mutex_lock(&dev_priv->vxd_pm_mutex);
	/* ospm_resume_pci(pdev); */
	if (!is_vxd_on()) {
#ifdef CONFIG_PM_RUNTIME
		i915_rpm_get_vxd(dev_priv->dev);
#endif
		vxd_power_on(i915_drm_dev);
	}
	atomic_inc(&g_videodec_access_count);
	mutex_unlock(&dev_priv->vxd_pm_mutex);
	return true;
}

void ospm_power_using_video_end(int hw_island)
{
	struct pci_dev *pdev = i915_drm_dev->pdev;
	struct drm_psb_private *dev_priv = psb_priv(i915_drm_dev);

	PSB_DEBUG_PM("MSVDX: %s is called.\n", __func__);
	if (hw_island != OSPM_VIDEO_DEC_ISLAND) {
		DRM_ERROR("failed.\n");
		return true;
	}
	mutex_lock(&dev_priv->vxd_pm_mutex);
	if (atomic_read(&g_videodec_access_count) <= 0)
		DRM_ERROR("g_videodec_access_count <=0.\n");
	else
		atomic_dec(&g_videodec_access_count);
	mutex_unlock(&dev_priv->vxd_pm_mutex);
}

module_init(vxd_driver_load);
module_exit(vxd_driver_unload);
MODULE_LICENSE("GPL");
