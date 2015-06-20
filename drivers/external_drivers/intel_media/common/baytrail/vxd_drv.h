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

#ifndef _VXD_DRV_H_
#define _VXD_DRV_H_

#include "i915_drv.h"
#include <linux/io-mapping.h>
#include <linux/kref.h>

#include <ttm/ttm_object.h>
#include "psb_ttm_fence_driver.h"
#include "psb_ttm_userobj_api.h"
#include "ttm/ttm_bo_driver.h"
#include "ttm/ttm_lock.h"
#include "psb_video_drv.h"
#include "psb_msvdx.h"

#define PSB_PMPOLICY_NOPM		0
#define PSB_PMPOLICY_CLOCKGATING	1
#define PSB_PMPOLICY_POWERDOWN		2

#define PSB_BOTTOM_HALF_WQ              1
#define PSB_BOTTOM_HALF_TQ              2

#define PSB_MSVDX_CLOCKGATING	  0x2064

#define PSB_MMU_CACHED_MEMORY	  0x0001	/* Bind to MMU only */
#define PSB_MMU_RO_MEMORY	  0x0002	/* MMU RO memory */
#define PSB_MMU_WO_MEMORY	  0x0004	/* MMU WO memory */

/*
 *PTE's and PDE's
 */

#define PSB_PDE_MASK		  0x003FFFFF
#define PSB_PDE_SHIFT		  22
#define PSB_PTE_SHIFT		  12

#define PSB_PTE_VALID		  0x0001	/* PTE / PDE valid */
#define PSB_PTE_WO		  0x0002	/* Write only */
#define PSB_PTE_RO		  0x0004	/* Read only */
#define PSB_PTE_CACHED		  0x0008	/* CPU cache coherent */

#define IS_MDFLD(dev) (0)
#define IS_MRFLD(dev) (0)

enum APM_VXD_STATUS {
	VXD_APM_STS_D0 = 0,
	VXD_APM_STS_D1,
	VXD_APM_STS_D2,
	VXD_APM_STS_D3
};

extern struct ttm_bo_driver psb_ttm_bo_driver;

struct psb_context;
struct psb_validate_buffer;
struct psb_video_ctx;

/*
 *User options.
 */

struct drm_psb_uopt {
	int pad;		/*keep it here in case we use it in future */
};

struct drm_psb_private {
	/*
	 *TTM Glue.
	 */

	struct drm_global_reference mem_global_ref;
	struct ttm_bo_global_ref bo_global_ref;
	int has_global;

	struct drm_device *dev;
	struct ttm_object_device *tdev;
	struct ttm_fence_device fdev;
	struct ttm_bo_device bdev;
	struct ttm_lock ttm_lock;
	struct vm_operations_struct *ttm_vm_ops;
	int has_fence_device;
	int has_bo_device;

	struct drm_psb_dev_info_arg dev_info;
	struct drm_psb_uopt uopt;

	uint32_t sequence[PSB_NUM_ENGINES];
	uint32_t last_sequence[PSB_NUM_ENGINES];
	uint32_t last_submitted_seq[PSB_NUM_ENGINES];

	struct psb_mmu_driver *mmu;
	struct psb_mmu_pd *pf_pd;

	/* IMG video context */
	struct list_head video_ctx;
	spinlock_t video_ctx_lock;

	/*
	 *MSVDX
	 */
	uint8_t *msvdx_reg;
	atomic_t msvdx_mmu_invaldc;
	void *msvdx_private;

	spinlock_t irqmask_lock;
	spinlock_t sequence_lock;

	/*
	 *Memory managers
	 */
	int have_mem_mmu;
	int have_mem_mmu_tiling;
	struct mutex temp_mem;

	/*
	 *Relocation buffer mapping.
	 */

	spinlock_t reloc_lock;
	unsigned int rel_mapped_pages;
	wait_queue_head_t rel_mapped_queue;


	/*
	 * Sizes info
	 */

	struct drm_psb_sizes_arg sizes;

	uint32_t fuse_reg_value;

	/* pci revision id for B0:D2:F0 */
	uint8_t platform_rev_id;

	/*runtime PM state */
	int rpm_enabled;

	/*
	 *Scheduling.
	 */

	struct mutex reset_mutex;
	struct mutex cmdbuf_mutex;
	/*uint32_t ta_mem_pages;
	   struct psb_ta_mem *ta_mem;
	   int force_ta_mem_load; */
	atomic_t val_seq;

	/*
	 *TODO: change this to be per drm-context.
	 */
	struct psb_context decode_context;


	/*
	 *Watchdog
	 */

	spinlock_t watchdog_lock;
	struct timer_list watchdog_timer;
	struct work_struct watchdog_wq;
	struct work_struct msvdx_watchdog_wq;
	struct work_struct topaz_watchdog_wq;
	struct work_struct hdmi_hotplug_wq;
	struct work_struct hdmi_audio_wq;
	atomic_t hotplug_wq_done;
	int timer_available;

	/* read register value through sysfs. */
	int count;
	char buf[256];

	struct pci_dev *pci_root;

	struct mutex vxd_pm_mutex;
};

/*
 *Debug print bits setting
 */
#define PSB_D_GENERAL (1 << 0)
#define PSB_D_INIT    (1 << 1)
#define PSB_D_IRQ     (1 << 2)
#define PSB_D_ENTRY   (1 << 3)
/* debug the get H/V BP/FP count */
#define PSB_D_HV      (1 << 4)
#define PSB_D_DBI_BF  (1 << 5)
#define PSB_D_PM      (1 << 6)
#define PSB_D_RENDER  (1 << 7)
#define PSB_D_REG     (1 << 8)
#define PSB_D_MSVDX   (1 << 9)
#define PSB_D_TOPAZ   (1 << 10)
#define VSP_D_LOG   (1 << 11)
#define VSP_D_PERF   (1 << 12)
#define PSB_D_WARN    (1 << 13)
#define PSB_D_MIPI    (1 << 14)

#define VXD_TTM_MMAP_OFFSET_START DRM_PSB_FILE_PAGE_OFFSET
#define VXD_TTM_MMAP_OFFSET_END (DRM_PSB_FILE_PAGE_OFFSET + 0x10000000)

extern int drm_psb_debug;

#define PSB_DEBUG_GENERAL(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_GENERAL, _fmt, ##_arg)
#define PSB_DEBUG_INIT(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_INIT, _fmt, ##_arg)
#define PSB_DEBUG_IRQ(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_IRQ, _fmt, ##_arg)
#define PSB_DEBUG_ENTRY(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_ENTRY, _fmt, ##_arg)
#define PSB_DEBUG_HV(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_HV, _fmt, ##_arg)
#define PSB_DEBUG_DBI_BF(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_DBI_BF, _fmt, ##_arg)
#define PSB_DEBUG_PM(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_PM, _fmt, ##_arg)
#define PSB_DEBUG_RENDER(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_RENDER, _fmt, ##_arg)
#define PSB_DEBUG_REG(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_REG, _fmt, ##_arg)
#define PSB_DEBUG_MSVDX(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_MSVDX, _fmt, ##_arg)
#define PSB_DEBUG_TOPAZ(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_TOPAZ, _fmt, ##_arg)
#define VSP_DEBUG(_fmt, _arg...) \
	PSB_DEBUG(VSP_D_LOG, "VSP: "_fmt, ##_arg)
#define VSP_PERF(_fmt, _arg...) \
	PSB_DEBUG(VSP_D_PERF, "VSP PERFORMANCE: "_fmt, ##_arg)
/* force to print WARN msg */
#define PSB_DEBUG_WARN(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_WARN, _fmt, ##_arg)
#define PSB_DEBUG_MIPI(_fmt, _arg...) \
	PSB_DEBUG(PSB_D_MIPI, _fmt, ##_arg)

#define IS_MSVDX_MEM_TILE(dev) 1

#if DRM_DEBUG_CODE
#define PSB_DEBUG(_flag, _fmt, _arg...)					\
	do {								\
		if (unlikely((_flag) & drm_psb_debug))			\
			printk(KERN_INFO				\
			       "[psb:0x%02x:%s] " _fmt , _flag,		\
			       __func__ , ##_arg);			\
	} while (0)
#else
#define PSB_DEBUG(_fmt, _arg...)     do { } while (0)
#endif

struct psb_fpriv {
	struct ttm_object_file *tfile;
	bool dsr_blocked;
};

struct psb_fpriv *psb_fpriv(struct drm_file *file_priv);
struct drm_psb_private *psb_priv(struct drm_device *dev);

extern int drm_psb_cpurelax;
extern int drm_psb_udelaydivider;
extern int drm_psb_priv_pmu_func;

/*
 * set cpu_relax = 1 in sysfs to use cpu_relax instead of udelay bysy loop
 * set udelay_divider to reduce the udelay values,e.g.= 10, reduce 10 times
 */
#define PSB_UDELAY(usec)                        \
do {                                            \
	if (drm_psb_cpurelax == 0)              \
		DRM_UDELAY(usec / drm_psb_udelaydivider);   \
	else                                    \
		cpu_relax();                    \
} while (0)

struct psb_fpriv *psb_fpriv(struct drm_file *file_priv);
struct drm_psb_private *psb_priv(struct drm_device *dev);
int vxd_release(struct inode *inode, struct file *filp);
int psb_mmap(struct file *filp, struct vm_area_struct *vma);
int ivxd_mmap(struct file *filp, struct vm_area_struct *vma);
void vxd_lastclose(struct drm_device *dev);
int vxd_driver_open(struct drm_device *dev, struct drm_file *file);
long vxd_ioctl(struct file *filp, unsigned int cmd, unsigned long arg);

/* pm related start */
#define OSPM_VIDEO_DEC_ISLAND	1
bool ospm_power_using_video_begin(int hw_island);
void ospm_power_using_video_end(int hw_island);
int ospm_apm_power_down_msvdx(struct drm_device *dev, int force_off);
/* end */

#endif
