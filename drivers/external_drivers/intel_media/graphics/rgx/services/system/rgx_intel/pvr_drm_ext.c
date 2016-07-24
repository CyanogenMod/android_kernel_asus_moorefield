/*************************************************************************/ /*!                                                                                                               
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved                                                                                                               
@License        Dual MIT/GPLv2

The contents of this file are subject to the MIT license as set out below.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in
all copies or substantial portions of the Software.

Alternatively, the contents of this file may be used under the terms of
the GNU General Public License Version 2 ("GPL") in which case the provisions
of GPL are applicable instead of those above.

If you wish to allow use of your version of this file only under the terms of
GPL, and not to allow others to use your version of this file under the terms
of the MIT license, indicate your decision by deleting the provisions above
and replace them with the notice and other provisions required by GPL as set
out in the file called "GPL-COPYING" included in this distribution. If you do
not delete the provisions above, a recipient may use your version of this file
under the terms of either the MIT license or GPL.

This License is also included in this distribution in the file called
"MIT-COPYING".

EXCEPT AS OTHERWISE STATED IN A NEGOTIATED AGREEMENT: (A) THE SOFTWARE IS
PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR
PURPOSE AND NONINFRINGEMENT; AND (B) IN NO EVENT SHALL THE AUTHORS OR
COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/ /**************************************************************************/

#include <linux/mm.h>
#include <linux/version.h>
#include <drm/drmP.h>
#include <drm/drm.h>

#include "img_defs.h"
#include "lock.h"
#include "pvr_drm_ext.h"
#include "pvrsrv_interface.h"
#include "pvr_bridge.h"
#include "srvkm.h"
#include "dc_mrfld.h"
#include "drm_shared.h"
#include "linkage.h"

#if defined(PDUMP)
#include "linuxsrv.h"
#endif

#include <linux/module.h>
#include "pvrmodule.h"

#define PVR_DRM_SRVKM_CMD       DRM_PVR_RESERVED1
#define PVR_DRM_IS_MASTER_CMD   DRM_PVR_RESERVED4
#define PVR_DRM_DBGDRV_CMD      DRM_PVR_RESERVED6

#define PVR_DRM_SRVKM_IOCTL \
	DRM_IOW(DRM_COMMAND_BASE + PVR_DRM_SRVKM_CMD, PVRSRV_BRIDGE_PACKAGE)

#define PVR_DRM_IS_MASTER_IOCTL \
	DRM_IO(DRM_COMMAND_BASE + PVR_DRM_IS_MASTER_CMD)

#if defined(PDUMP)
#define	PVR_DRM_DBGDRV_IOCTL \
	DRM_IOW(DRM_COMMAND_BASE + PVR_DRM_DBGDRV_CMD, IOCTL_PACKAGE)
#endif

static int
PVRDRMIsMaster(struct drm_device *dev, void *arg, struct drm_file *pFile)
{
	return 0;
}

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0))
static struct drm_ioctl_desc pvr_ioctls[] = {
	{PVR_DRM_SRVKM_IOCTL, DRM_UNLOCKED, PVRSRV_BridgeDispatchKM},
	{PVR_DRM_IS_MASTER_IOCTL, DRM_MASTER, PVRDRMIsMaster},
#if defined(PDUMP)
	{PVR_DRM_DBGDRV_IOCTL, 0, dbgdrv_ioctl}
#endif
};
#else
static struct drm_ioctl_desc pvr_ioctls[] = {
	{PVR_DRM_SRVKM_IOCTL, DRM_UNLOCKED, PVRSRV_BridgeDispatchKM, PVR_DRM_SRVKM_IOCTL},
	{PVR_DRM_IS_MASTER_IOCTL, DRM_MASTER, PVRDRMIsMaster, PVR_DRM_IS_MASTER_IOCTL},
#if defined(PDUMP)
	{PVR_DRM_DBGDRV_IOCTL, 0, dbgdrv_ioctl. PVR_DRM_DBGDRV_IOCTL}
#endif
};
#endif /* (LINUX_VERSION_CODE < KERNEL_VERSION(3,8,0)) */

DECLARE_WAIT_QUEUE_HEAD(sWaitForInit);

static bool bInitComplete;
static bool bInitFailed;

struct pci_dev *gpsPVRLDMDev;

struct drm_device *gpsPVRDRMDev;

#define PVR_DRM_FILE struct drm_file *

int PVRSRVDrmLoad(struct drm_device *dev, unsigned long flags)
{
	int iRes = 0;

	DRM_DEBUG("PVRSRVDrmLoad");

	gpsPVRDRMDev = dev;
	gpsPVRLDMDev = dev->pdev;

#if defined(PDUMP)
	iRes = dbgdrv_init();
	if (iRes != 0)
	{
		goto exit;
	}
#endif
	
	iRes = PVRCore_Init();
	if (iRes != 0)
	{
		goto exit_dbgdrv_cleanup;
	}

	if (MerrifieldDCInit(dev) != PVRSRV_OK)
	{
		DRM_ERROR("%s: display class init failed\n", __FUNCTION__);
		goto exit_pvrcore_cleanup;
	}

	goto exit;

exit_pvrcore_cleanup:
	PVRCore_Cleanup();

exit_dbgdrv_cleanup:
#if defined(PDUMP)
	dbgdrv_cleanup();
#endif
exit:
	if (iRes != 0)
	{
		bInitFailed = true;
	}
	bInitComplete = true;

	wake_up_interruptible(&sWaitForInit);

	return iRes;
}

int PVRSRVDrmUnload(struct drm_device *dev)
{
	DRM_DEBUG("PVRSRVDrmUnload");

	if (MerrifieldDCDeinit() != PVRSRV_OK)
	{
		DRM_ERROR("%s: can't deinit display class\n", __FUNCTION__);
	}

	PVRCore_Cleanup();

#if defined(PDUMP)
	dbgdrv_cleanup();
#endif

	return 0;
}

int PVRSRVDrmOpen(struct drm_device *dev, struct drm_file *file)
{
	while (!bInitComplete)
	{
		DEFINE_WAIT(sWait);

		prepare_to_wait(&sWaitForInit, &sWait, TASK_INTERRUPTIBLE);

		if (!bInitComplete)
		{
			DRM_DEBUG("%s: Waiting for module initialisation to complete", __FUNCTION__);

			schedule();
		}

		finish_wait(&sWaitForInit, &sWait);

		if (signal_pending(current))
		{
			return -ERESTARTSYS;
		}
	}

	if (bInitFailed)
	{
		DRM_DEBUG("%s: Module initialisation failed", __FUNCTION__);
		return -EINVAL;
	}

	return PVRSRVOpen(dev, file);
}

void PVRSRVDrmPostClose(struct drm_device *dev, struct drm_file *file)
{
	PVRSRVRelease(dev, file);

	file->driver_priv = NULL;
}

void PVRSRVQueryIoctls(struct drm_ioctl_desc *ioctls)
{
	int i;

	for (i = 0; i < DRM_ARRAY_SIZE(pvr_ioctls); i++)
	{
		unsigned int slot = DRM_IOCTL_NR(pvr_ioctls[i].cmd) - DRM_COMMAND_BASE;
		ioctls[slot] = pvr_ioctls[i];
	}
}

unsigned int PVRSRVGetMeminfoSize(void* hMemHandle)
{
    PVRSRV_MEMINFO  minfo;

    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return 0;
    }

    return minfo.uiAllocationSize;
}

void * PVRSRVGetMeminfoCPUAddr(void* hMemHandle)
{
    PVRSRV_MEMINFO  minfo;

    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return 0;
    }

    return minfo.pvCpuVirtAddr;
}

int PVRSRVGetMeminfoPages(void* hMemHandle, int npages, struct page ***pages)
{
    PVRSRV_MEMINFO  minfo;
    struct page   **pglist;
    uint32_t        kaddr;
    int             res;

    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return -EFAULT;
    }

    kaddr = (uint32_t)(uintptr_t)minfo.pvCpuVirtAddr;

    if ((pglist = kzalloc(npages * sizeof(struct page*),GFP_KERNEL)) == NULL)
    {
        return -ENOMEM;
    }

    down_read(&current->mm->mmap_sem);
    res = get_user_pages(current,current->mm,kaddr,npages,0,0,pglist,NULL);
    up_read(&current->mm->mmap_sem);

    if (res <= 0)
    {
        kfree(pglist);
        return res;
    }

    *pages = pglist;
	return 0;
}

int PVRSRVGetMeminfoPfn(void           *hMemHandle,
                        int             npages,
                        unsigned long **pfns)
{
    PVRSRV_MEMINFO          minfo;
    struct vm_area_struct  *vma;
    unsigned long          *pfnlist;
    uint32_t                kaddr;
    int                     res, pg = 0;

    /*
     *  This 'handle' is a pointer in user space to a meminfo struct.
     *  We need to copy it here and get the user's view of memory.
     */
    if (copy_from_user(&minfo,hMemHandle,sizeof minfo))
    {
        return 0;
    }

    kaddr = (uint32_t)(uintptr_t)minfo.pvCpuVirtAddr;

    if ((pfnlist = kzalloc(npages * sizeof(unsigned long),
                           GFP_KERNEL)) == NULL)
    {
        return -ENOMEM;
    }

    while (pg < npages)
    {
        if ((vma = find_vma(current->mm,
                            kaddr + (pg * PAGE_SIZE))) == NULL)
        {
            kfree(pfnlist);
            return -EFAULT;
        }

        if ((res = follow_pfn(
                        vma,
                        (unsigned long)(kaddr + (pg * PAGE_SIZE)),
                        &pfnlist[pg])) < 0)
        {
            kfree(pfnlist);
            return res;
        }

        ++pg;
    }

    *pfns = pfnlist;
    return 0;
}

int PVRSRVInterrupt(struct drm_device* dev)
{
	return 1;
}

int PVRSRVMMap(struct file *pFile, struct vm_area_struct *ps_vma)
{
	return MMapPMR(pFile, ps_vma);
}
