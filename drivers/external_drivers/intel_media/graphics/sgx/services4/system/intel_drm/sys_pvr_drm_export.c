/**********************************************************************
 *
 * Copyright (C) Imagination Technologies Ltd. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful but, except
 * as otherwise stated in writing, without any warranty; without even the
 * implied warranty of merchantability or fitness for a particular purpose.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 * The full GNU General Public License is included in this distribution in
 * the file called "COPYING".
 *
 * Contact Information:
 * Imagination Technologies Ltd. <gpl-support@imgtec.com>
 * Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
 *
 ******************************************************************************/

#include <drm/drmP.h>
#include <drm/drm.h>

#include "pvr_drm_shared.h"

#include "services_headers.h"
#include "private_data.h"
#include "pvr_drm.h"

#include "pvr_bridge.h"
#include "mutex.h"
#include "lock.h"
#include "linkage.h"
#include "mmap.h"

#if defined(PDUMP)
#include "linuxsrv.h"
#endif

#include "sys_pvr_drm_import.h"

#include "sys_pvr_drm_export.h"
#include "psb_drv.h"

int
SYSPVRInit(void)
{
	PVRDPFInit();

	return 0;
}

int
SYSPVRLoad(struct drm_device *dev, unsigned long flags)
{
	return PVRSRVDrmLoad(dev, flags);
}

int
SYSPVROpen(struct drm_device *dev, struct drm_file *pFile)
{
	return PVRSRVDrmOpen(dev, pFile);
}

int
SYSPVRUnload(struct drm_device *dev)
{
	return PVRSRVDrmUnload(dev);
}

void
SYSPVRPostClose(struct drm_device *dev, struct drm_file *file)
{
	return PVRSRVDrmPostClose(dev, file);
}

int
SYSPVRBridgeDispatch(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile)
{
	return PVRSRV_BridgeDispatchKM(dev, arg, pFile);
}

int
SYSPVRDCDriverIoctl(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile)
{
	return PVRDRM_Dummy_ioctl(dev, arg, pFile);

}

int
SYSPVRBCDriverIoctl(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile)
{
	return PVRDRM_Dummy_ioctl(dev, arg, pFile);

}

int
SYSPVRIsMaster(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile)
{
	return PVRDRMIsMaster(dev, arg, pFile);
}

int
SYSPVRUnprivCmd(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile)
{
	return PVRDRMUnprivCmd(dev, arg, pFile);
}

int
SYSPVRMMap(struct file* pFile, struct vm_area_struct* ps_vma)
{
	int ret;

	ret = PVRMMap(pFile, ps_vma);
	if (ret == -ENOENT)
	{
		ret = drm_mmap(pFile, ps_vma);
	}

	return ret;
}

int
SYSPVRDBGDrivIoctl(struct drm_device *dev, IMG_VOID *arg, struct drm_file *pFile)
{
#if defined(PDUMP)
	return dbgdrv_ioctl(dev, arg, pFile);
#else
	PVR_UNREFERENCED_PARAMETER(dev);
	PVR_UNREFERENCED_PARAMETER(arg);
	PVR_UNREFERENCED_PARAMETER(pFile);

	return -EINVAL;
#endif
}

void
SYSPVRSuspendLock(struct drm_device *dev)
{
	PVR_UNREFERENCED_PARAMETER(dev);

	LinuxLockMutex(&gPVRSRVLock);
}

void
SYSPVRSuspendUnlock(struct drm_device *dev)
{
	PVR_UNREFERENCED_PARAMETER(dev);

	LinuxUnLockMutex(&gPVRSRVLock);
}

int
SYSPVRPreSuspend(struct drm_device *dev)
{
	if (PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D3) != PVRSRV_OK)
	{
		return -EBUSY;
	}

#if defined(DISPLAY_CONTROLLER)
        if (PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Suspend)(dev) != 0)
        {
		(void)PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D0);
                return -EBUSY;
        }
#else
	PVR_UNREFERENCED_PARAMETER(dev);
#endif

	return 0;
}

int
SYSPVRPostSuspend(struct drm_device *dev)
{
	if (PVRSRVSetPowerStateKM(PVRSRV_SYS_POWER_STATE_D0) != PVRSRV_OK)
	{
		return -EBUSY;
	}

	return 0;
}

int
SYSPVRResume(struct drm_device *dev)
{
#if defined(DISPLAY_CONTROLLER)
        if (PVR_DRM_MAKENAME(DISPLAY_CONTROLLER, _Resume)(dev) != 0)
        {
                return -EINVAL;
        }
#else
	PVR_UNREFERENCED_PARAMETER(dev);
#endif
	return 0;
}

PVRSRV_ERROR OSScheduleMISR2(void)
{
	SYS_DATA *psSysData;
	SysAcquireData(&psSysData);
	OSScheduleMISR(psSysData);
	return PVRSRV_OK;
}

int
SYSPVRFillCallback(struct drm_device *ddev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)ddev->dev_private;
	struct gpu_pvr_ops *pvr_ops =
		kmalloc(sizeof(struct gpu_pvr_ops), GFP_KERNEL);

	if (unlikely(!pvr_ops))
		return -ENOMEM;

	/* init wraper table */
	pvr_ops->PVRGetDisplayClassJTable = PVRGetDisplayClassJTable;
#if defined(SUPPORT_DRI_DRM_EXT)
	pvr_ops->SYSPVRServiceSGXInterrupt = SYSPVRServiceSGXInterrupt;
#endif
	pvr_ops->PVRSRVDrmLoad = PVRSRVDrmLoad;
	pvr_ops->SYSPVRInit = SYSPVRInit;
	pvr_ops->PVRDRM_Dummy_ioctl = PVRDRM_Dummy_ioctl;
	pvr_ops->PVRMMap = PVRMMap;
	pvr_ops->PVRSRVDrmPostClose = PVRSRVDrmPostClose;
	pvr_ops->PVRSRV_BridgeDispatchKM = PVRSRV_BridgeDispatchKM;
	pvr_ops->PVRSRVOpen = PVRSRVOpen;
	pvr_ops->PVRDRMIsMaster = PVRDRMIsMaster;
	pvr_ops->PVRDRMUnprivCmd = PVRDRMUnprivCmd;
	pvr_ops->SYSPVRDBGDrivIoctl = SYSPVRDBGDrivIoctl;
	pvr_ops->PVRSRVDrmUnload = PVRSRVDrmUnload;
	pvr_ops->PVRSRVPerProcessData = PVRSRVPerProcessData;
	pvr_ops->PVRSRVLookupHandle = PVRSRVLookupHandle;
	pvr_ops->LinuxMemAreaToCpuPAddr = (void *) LinuxMemAreaToCpuPAddr;
	pvr_ops->OSScheduleMISR2 = OSScheduleMISR2;

	dev_priv->pvr_ops = pvr_ops;
	return 0;
}

void SYSPVRClearCallback(struct drm_device *ddev)
{
	struct drm_psb_private *dev_priv =
		(struct drm_psb_private *)ddev->dev_private;

	kfree(dev_priv->pvr_ops);
	dev_priv->pvr_ops = NULL;
}
