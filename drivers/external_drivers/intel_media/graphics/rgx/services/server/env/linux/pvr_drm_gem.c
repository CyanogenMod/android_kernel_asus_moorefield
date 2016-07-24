/*************************************************************************/ /*!
@File
@Title          PowerVR DRM GEM interface
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Interface for managing GEM memory.
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

#if defined(SUPPORT_DRM)

#include <drm/drmP.h>
#include <drm/drm.h>

#include "private_data.h"
#include "driverlock.h"
#include "pmr.h"
#include "physmem.h"
#include "pvr_drm.h"
#include "pvr_drm_display.h"
#include "sync_server_internal.h"
#include "allocmem.h"

#if defined(PVR_DRM_USE_PRIME)
#include "physmem_dmabuf.h"
#endif

#if defined(PVR_RI_DEBUG)
#include "ri_server.h"
#endif

static PVRSRV_ERROR GEMSyncHandleDestroy(IMG_PVOID pvParam)
{
	SERVER_SYNC_PRIMITIVE *psSync = (SERVER_SYNC_PRIMITIVE *)pvParam;

	ServerSyncUnref(psSync);

	return PVRSRV_OK;
}

static int GEMSyncHandleCreate(CONNECTION_DATA *psConnection, SERVER_SYNC_PRIMITIVE *psSync, IMG_HANDLE *phSync)
{
	PVRSRV_ERROR eError;
	int iErr;

	ServerSyncRef(psSync);

	eError = PVRSRVAllocHandle(psConnection->psHandleBase,
				   phSync,
				   (void *)psSync,
				   PVRSRV_HANDLE_TYPE_SERVER_SYNC_PRIMITIVE,
				   PVRSRV_HANDLE_ALLOC_FLAG_MULTI,
				   (PFN_HANDLE_RELEASE)GEMSyncHandleDestroy);
	if (eError != PVRSRV_OK)
	{
		switch (eError)
		{
			case PVRSRV_ERROR_UNABLE_TO_ADD_HANDLE:
			case PVRSRV_ERROR_OUT_OF_MEMORY:
				iErr = -ENOMEM;
				break;
			case PVRSRV_ERROR_INVALID_PARAMS:
			default:
				iErr = -EINVAL;
		}

		goto ErrorSyncUnreference;
	}

	return 0;

ErrorSyncUnreference:
	ServerSyncUnref(psSync);

	return iErr;
}


/*************************************************************************/ /*!
* DRM GEM PMR factory
*/ /**************************************************************************/

typedef struct PMR_GEM_PRIV_TAG
{
	struct drm_gem_object	*psObj;
	PMR			*psBackingPMR;
} PMR_GEM_PRIV;


static PVRSRV_ERROR PMRGEMLockPhysAddress(PMR_IMPL_PRIVDATA pvPriv,
					  IMG_UINT32 uiLog2DevPageSize)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;

	/* Call  PMRLockSysPhysAddresses using the proper lock class to avoid a Lockdep issue */
	return PMRLockSysPhysAddressesNested(psGEMPriv->psBackingPMR, uiLog2DevPageSize, 1);
}

static PVRSRV_ERROR PMRGEMUnlockPhysAddress(PMR_IMPL_PRIVDATA pvPriv)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;

	return PMRUnlockSysPhysAddresses(psGEMPriv->psBackingPMR);
}

static PVRSRV_ERROR PMRGEMDevPhysAddr(PMR_IMPL_PRIVDATA pvPriv,
				      IMG_UINT32 ui32NumOfPages,
				      IMG_DEVMEM_OFFSET_T *uiOffset,
					  IMG_BOOL *pbValid,
				      IMG_DEV_PHYADDR *psDevAddrPtr)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;
	
	/* This use-case requires special treatment; as it is not a core
	   PMR factory (i.e. it's a PMR adapter), only uiOffset[0] is valid.
	   This is the initial look-up offset specified by higher level code.
	   This inconsistency is due to function type-info sharing between
	   the two. Normally core PMR is responsible for re-expressing an
	   offset/uiNumOfPages pair into offset[array] for PMR factories
	   to translate, i.e. PMR adapter -> PMR core -> PMR factories */
	return PMR_DevPhysAddr(psGEMPriv->psBackingPMR,
						   PAGE_SHIFT, 
						   ui32NumOfPages,
						   uiOffset[0], 
						   psDevAddrPtr,
						   pbValid);
}

#if defined(PDUMP)
static PVRSRV_ERROR PMRGEMPDumpSymbolicAddr(PMR_IMPL_PRIVDATA pvPriv,
					      IMG_DEVMEM_OFFSET_T uiOffset,
					      IMG_CHAR *pszMemspaceName,
					      IMG_UINT32 ui32MemspaceNameLen,
					      IMG_CHAR *pszSymbolicAddr,
					      IMG_UINT32 ui32SymbolicAddrLen,
					      IMG_DEVMEM_OFFSET_T *puiNewOffset,
					      IMG_DEVMEM_OFFSET_T *puiNextSymName)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;

	return PMR_PDumpSymbolicAddr(psGEMPriv->psBackingPMR,
				     uiOffset,
				     ui32MemspaceNameLen,
				     pszMemspaceName,
				     ui32SymbolicAddrLen,
				     pszSymbolicAddr,
				     puiNewOffset,
				     puiNextSymName);
}
#endif

static PVRSRV_ERROR PMRGEMAcquireKernelMappingData(PMR_IMPL_PRIVDATA pvPriv,
						   IMG_SIZE_T uiOffset,
						   IMG_SIZE_T uiSize,
						   void **ppvKernelAddressOut,
						   IMG_HANDLE *phHandleOut,
						   PMR_FLAGS_T unref__ ulFlags)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;
	IMG_SIZE_T uiLength;

	return PMRAcquireKernelMappingData(psGEMPriv->psBackingPMR,
					   uiOffset,
					   uiSize,
					   ppvKernelAddressOut,
					   &uiLength,
					   phHandleOut);
}

static void PMRGEMReleaseKernelMappingData(PMR_IMPL_PRIVDATA pvPriv,
					       IMG_HANDLE hHandle)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;

	PMRReleaseKernelMappingData(psGEMPriv->psBackingPMR, hHandle);
}

static PVRSRV_ERROR PMRGEMReadBytes(PMR_IMPL_PRIVDATA pvPriv,
				    IMG_DEVMEM_OFFSET_T uiOffset,
				    IMG_UINT8 *pcBuffer,
				    IMG_SIZE_T uiBufferSize,
				    IMG_SIZE_T *puiNumBytes)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;

	return PMR_ReadBytes(psGEMPriv->psBackingPMR,
			     uiOffset,
			     pcBuffer,
			     uiBufferSize,
			     puiNumBytes);
}

static PVRSRV_ERROR PMRGEMWriteBytes(PMR_IMPL_PRIVDATA pvPriv,
				     IMG_DEVMEM_OFFSET_T uiOffset,
				     IMG_UINT8 *pcBuffer,
				     IMG_SIZE_T uiBufferSize,
				     IMG_SIZE_T *puiNumBytes)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;

	return PMR_WriteBytes(psGEMPriv->psBackingPMR,
			      uiOffset,
			      pcBuffer,
			      uiBufferSize,
			      puiNumBytes);
}

static PVRSRV_ERROR PMRGEMFinalize(PMR_IMPL_PRIVDATA pvPriv)
{
	PMR_GEM_PRIV *psGEMPriv = pvPriv;

	PMRUnrefPMR(psGEMPriv->psBackingPMR);

	OSFreeMem(psGEMPriv);

	return PVRSRV_OK;
}

static const PMR_IMPL_FUNCTAB gsPMRGEMFuncTab = 
{
	.pfnLockPhysAddresses		= PMRGEMLockPhysAddress,
	.pfnUnlockPhysAddresses		= PMRGEMUnlockPhysAddress,
	.pfnDevPhysAddr			= PMRGEMDevPhysAddr,
#if defined(PDUMP)
	.pfnPDumpSymbolicAddr		= PMRGEMPDumpSymbolicAddr,
#endif
	.pfnAcquireKernelMappingData	= PMRGEMAcquireKernelMappingData,
	.pfnReleaseKernelMappingData	= PMRGEMReleaseKernelMappingData,
	.pfnReadBytes			= PMRGEMReadBytes,
	.pfnWriteBytes			= PMRGEMWriteBytes,
	.pfnFinalize			= PMRGEMFinalize,
};

PVRSRV_ERROR PVRSRVGEMCreatePMR(PVRSRV_DEVICE_NODE *psDevNode,
				struct drm_gem_object *psObj,
				PVRSRV_MEMALLOCFLAGS_T uiFlags,
				PMR **ppsPMR)
{
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(psObj);
	IMG_BOOL bMappingTable = IMG_TRUE;
	PMR_GEM_PRIV *psGEMPriv;
	PVRSRV_ERROR eError;

	/* Create the private data structure for the PMR */
	psGEMPriv = OSAllocZMem(sizeof *psGEMPriv);
	if (psGEMPriv == NULL)
	{
		return PVRSRV_ERROR_OUT_OF_MEMORY;
	}

	psGEMPriv->psObj = psObj;

	switch (psPVRObj->type)
	{
		case PVR_DRM_GEM_PMR:
			eError = PhysmemNewRamBackedPMR(psDevNode,
							psObj->size,
							psObj->size,
							1,
							1,
							&bMappingTable,
							PAGE_SHIFT,
							uiFlags,
							&psGEMPriv->psBackingPMR);
			break;
#if defined(SUPPORT_DRM_DC_MODULE)
		case PVR_DRM_GEM_DISPLAY_PMR:
			eError = PVRSRVDRMDisplayCreatePMR(psDevNode,
							   psObj->dev,
							   psObj->size,
							   uiFlags,
							   &psGEMPriv->psBackingPMR,
							   &psPVRObj->obj);
			break;
#endif
#if defined(PVR_DRM_USE_PRIME)
		case PVR_DRM_GEM_IMPORT_PMR:
			eError = PhysmemCreateNewDmaBufBackedPMR(psDevNode->apsPhysHeap[PVR_DRM_PHYS_HEAP],
								 psObj->import_attach,
								 NULL,
								 uiFlags,
								 &psGEMPriv->psBackingPMR);
			break;
#endif
		case PVR_DRM_GEM_UNDEFINED:
		default:
			eError = PVRSRV_ERROR_NOT_SUPPORTED;
			break;
	}

	if (eError != PVRSRV_OK)
	{
		goto ErrorFreePMRPriv;
	}

	eError = PMRCreatePMR(psDevNode->apsPhysHeap[PVR_DRM_PHYS_HEAP],
			      psObj->size,
			      psObj->size,
			      1,
			      1,
			      &bMappingTable,
			      PAGE_SHIFT,
			      uiFlags,
			      "PMRGEM",
			      &gsPMRGEMFuncTab,
			      psGEMPriv,
			      ppsPMR,
			      IMG_NULL,
			      IMG_FALSE);
	if (eError != PVRSRV_OK)
	{
		goto ErrorUnrefBackingPMR;
	}

#if defined(PVR_RI_DEBUG)
	eError = RIWritePMREntryKM(*ppsPMR,
				   sizeof("GEM"),
				   "GEM",
				   psObj->size);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING,
			 "%s: Failed to write PMR entry (%s)",
			 __FUNCTION__, PVRSRVGetErrorStringKM(eError)));
	}
#endif

	return PVRSRV_OK;

ErrorUnrefBackingPMR:
	PMRUnrefPMR(psGEMPriv->psBackingPMR);

ErrorFreePMRPriv:
	OSFreeMem(psGEMPriv);

	return eError;	
}


/*************************************************************************/ /*!
* PVR DRM IOCTL functions
*/ /**************************************************************************/

int PVRDRMGEMCreate(struct drm_device *dev, void *arg, struct drm_file *file)
{
	drm_pvr_gem_create *psGEMCreate = (drm_pvr_gem_create *)arg;
	struct drm_gem_object *psObj;
	enum pvr_drm_gem_object_type eType;
	int iRet;

	if ((psGEMCreate->usage_flags & PVR_GEM_USE_SCANOUT) &&
	    (psGEMCreate->usage_flags & PVR_GEM_USE_CURSOR))
	{
		return -EINVAL;
	}

	if ((psGEMCreate->usage_flags & PVR_GEM_USE_SCANOUT) ||
	    (psGEMCreate->usage_flags & PVR_GEM_USE_CURSOR))
	{
#if defined(SUPPORT_DRM_DC_MODULE)
		eType = PVR_DRM_GEM_DISPLAY_PMR;
#else
		return -EPERM;
#endif
	}
	else
	{
		eType = PVR_DRM_GEM_PMR;
	}

	psObj = PVRSRVGEMObjectCreate(dev,
				      eType,
				      psGEMCreate->size,
				      (PVRSRV_MEMALLOCFLAGS_T)psGEMCreate->alloc_flags);
	if (IS_ERR(psObj))
	{
		return PTR_ERR(psObj);
	}

	iRet = drm_gem_handle_create(file, psObj, &psGEMCreate->handle);

	drm_gem_object_unreference_unlocked(psObj);

	return iRet;
}

static PVRSRV_ERROR GEMDestroyPMRHandle(IMG_PVOID pvParam)
{
	struct drm_gem_object *psObj = PVRSRVGEMGetObject((PMR *)pvParam);

	drm_gem_object_unreference_unlocked(psObj);

	return PVRSRV_OK;
}

static int GEMCreatePMRHandle(CONNECTION_DATA *psConnection, PMR *psPMR, IMG_HANDLE *phPMR)
{
	struct drm_gem_object *psObj = PVRSRVGEMGetObject(psPMR);
	PVRSRV_ERROR eError;
	int iErr;

	/*
	 * Hold a reference to the GEM object so that it can't be destroyed
	 * until there are no more IMG handles for the PMR.
	 */
	drm_gem_object_reference(psObj);

	eError = PVRSRVAllocHandle(psConnection->psHandleBase,
				   phPMR,
				   (void *)psPMR,
				   PVRSRV_HANDLE_TYPE_PHYSMEM_PMR,
				   PVRSRV_HANDLE_ALLOC_FLAG_MULTI,
				   (PFN_HANDLE_RELEASE)GEMDestroyPMRHandle);
	if (eError != PVRSRV_OK)
	{
		switch (eError)
		{
			case PVRSRV_ERROR_UNABLE_TO_ADD_HANDLE:
			case PVRSRV_ERROR_OUT_OF_MEMORY:
				iErr = -ENOMEM;
				break;
			case PVRSRV_ERROR_INVALID_PARAMS:
			default:
				iErr = -EINVAL;
		}

		goto ErrorObjectUnreference;
	}

	return 0;

ErrorObjectUnreference:
	drm_gem_object_unreference_unlocked(psObj);

	return iErr;
}

int PVRDRMGEMToIMGHandle(struct drm_device *dev, void *arg, struct drm_file *file)
{
	drm_pvr_gem_to_img_handle *psGEMToIMGHandle = (drm_pvr_gem_to_img_handle *)arg;
	struct pvr_drm_gem_object *psPVRObj;
	struct drm_gem_object *psObj;
	int iRet;

	OSAcquireBridgeLock();

	psObj = drm_gem_object_lookup(dev, file, psGEMToIMGHandle->gem_handle);
	if (psObj == NULL)
	{
		iRet = -ENOENT;
		goto ExitUnlock;
	}
	psPVRObj = to_pvr_drm_gem_object(psObj);

	switch (psPVRObj->type)
	{
		case PVR_DRM_GEM_PMR:
#if defined(SUPPORT_DRM_DC_MODULE)
		case PVR_DRM_GEM_DISPLAY_PMR:
#endif
#if defined(PVR_DRM_USE_PRIME)
		case PVR_DRM_GEM_IMPORT_PMR:
#endif
		{
			IMG_HANDLE hPMR;

			iRet = GEMCreatePMRHandle(LinuxConnectionFromFile(PVR_FILE_FROM_DRM_FILE(file)),
						  psPVRObj->pmr,
						  &hPMR);
			if (iRet == 0)
			{
				psGEMToIMGHandle->img_handle = (uint64_t)(uintptr_t)hPMR;
			}
			break;
		}
		default:
			iRet = -EINVAL;
			break;
	}

	drm_gem_object_unreference_unlocked(psObj);

ExitUnlock:
	OSReleaseBridgeLock();

	return iRet;
}

int PVRDRMIMGToGEMHandle(struct drm_device *dev, void *arg, struct drm_file *file)
{
	drm_pvr_img_to_gem_handle *psIMGToGEMHandle = (drm_pvr_img_to_gem_handle *)arg;
	CONNECTION_DATA *psConnection = LinuxConnectionFromFile(PVR_FILE_FROM_DRM_FILE(file));
	struct drm_gem_object *psObj;
	PMR *psPMR;
	PVRSRV_ERROR eError;
	int iRet;

	OSAcquireBridgeLock();

	eError = PVRSRVLookupHandle(psConnection->psHandleBase,
				    (void **)&psPMR,
				    (IMG_HANDLE)(IMG_UINTPTR_T)psIMGToGEMHandle->img_handle,
				    PVRSRV_HANDLE_TYPE_PHYSMEM_PMR);
	if (eError != PVRSRV_OK)
	{
		iRet = -EINVAL;
		goto ExitUnlock;
	}

	psObj = PVRSRVGEMGetObject(psPMR);
	if (psObj == NULL)
	{
		iRet = -EINVAL;
		goto ExitUnlock;
	}

	iRet = drm_gem_handle_create(file, psObj, &psIMGToGEMHandle->gem_handle);

ExitUnlock:
	OSReleaseBridgeLock();

	return iRet;
}

int PVRDRMGEMSyncGet(struct drm_device *dev, void *arg, struct drm_file *file)
{
	drm_pvr_gem_sync_get *psGEMSyncGet = (drm_pvr_gem_sync_get *)arg;
	struct pvr_drm_gem_object *psPVRObj;
	struct drm_gem_object *psObj;
	int iRet;

	OSAcquireBridgeLock();

	psObj = drm_gem_object_lookup(dev, file, psGEMSyncGet->gem_handle);
	if (psObj == NULL)
	{
		iRet = -ENOENT;
		goto ExitUnlock;
	}
	psPVRObj = to_pvr_drm_gem_object(psObj);

	switch (psPVRObj->type)
	{
		case PVR_DRM_GEM_PMR:
#if defined(PVR_DRM_USE_PRIME)
		case PVR_DRM_GEM_IMPORT_PMR:
#endif
#if defined(SUPPORT_DRM_DC_MODULE)
		case PVR_DRM_GEM_DISPLAY_PMR:
#endif
		{
			SERVER_SYNC_PRIMITIVE *psSync;
			IMG_UINT32 uiSyncVAddr;
			IMG_HANDLE hSync = NULL;

			switch (psGEMSyncGet->type)
			{
				case PVRSRV_GEM_SYNC_TYPE_WRITE:
				case PVRSRV_GEM_SYNC_TYPE_READ_HW:
				case PVRSRV_GEM_SYNC_TYPE_READ_SW:
				case PVRSRV_GEM_SYNC_TYPE_READ_DISPLAY:
					psSync = psPVRObj->apsSyncPrim[psGEMSyncGet->type];
					uiSyncVAddr = psPVRObj->auiSyncPrimVAddr[psGEMSyncGet->type];
					break;
				default:
					iRet = -EINVAL;
					goto ExitUnref;
			}

			if (psSync != NULL)
			{
				iRet = GEMSyncHandleCreate(LinuxConnectionFromFile(PVR_FILE_FROM_DRM_FILE(file)),
							   psSync,
							   &hSync);
				if (iRet != 0)
				{
					goto ExitUnref;
				}
			}
			
			psGEMSyncGet->sync_handle = (uint64_t)(uintptr_t)hSync;
			psGEMSyncGet->firmware_addr = uiSyncVAddr;
			
			iRet = 0;
			break;
		}
		default:
			iRet = -EINVAL;
			break;
	}

ExitUnref:
	drm_gem_object_unreference_unlocked(psObj);

ExitUnlock:
	OSReleaseBridgeLock();

	return iRet;
}


/*************************************************************************/ /*!
* DRM GEM helper callbacks
*/ /**************************************************************************/

#if defined(SUPPORT_DRM_DC_MODULE)
int PVRSRVGEMDumbCreate(struct drm_file *file,
			struct drm_device *dev,
			struct drm_mode_create_dumb *args)
{
	struct drm_gem_object *psObj;
	uint32_t uiPitch;
	size_t uiSize;
	int iRet;

	uiPitch = args->width * (ALIGN(args->bpp, 8) / 8);
	uiSize = uiPitch * args->height;

	psObj = PVRSRVGEMObjectCreate(dev,
				      PVR_DRM_GEM_DISPLAY_PMR,
				      uiSize,
				      PVRSRV_MEMALLOCFLAG_WRITE_COMBINE);
	if (IS_ERR(psObj))
	{
		return PTR_ERR(psObj);
	}

	iRet = drm_gem_handle_create(file, psObj, &args->handle);
	if (iRet == 0)
	{
		args->pitch = uiPitch;
		args->size = uiSize;
	}

	drm_gem_object_unreference_unlocked(psObj);

	return iRet;
}

int PVRSRVGEMDumbDestroy(struct drm_file *file,
			 struct drm_device unref__ *dev,
			 uint32_t handle)
{
	return drm_gem_handle_delete(file, handle);
}

int PVRSRVGEMDumbMapOffset(struct drm_file *file,
			   struct drm_device *dev,
			   uint32_t handle,
			   uint64_t *offset)
{
	struct drm_gem_object *psObj;
	int iRet = 0;

	mutex_lock(&dev->struct_mutex);

	psObj = drm_gem_object_lookup(dev, file, handle);
	if (!psObj)
	{
		iRet = -ENOENT;
		goto ExitUnlock;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)) || defined(CHROMIUMOS_WORKAROUNDS_KERNEL310)
	iRet = drm_gem_create_mmap_offset(psObj);
	if (iRet)
	{
		goto ExitUnref;
	}

	*offset = drm_vma_node_offset_addr(&psObj->vma_node);
#else
	if (!psObj->map_list.map)
	{
		iRet = drm_gem_create_mmap_offset(psObj);
		if (iRet)
		{
			goto ExitUnref;
		}
	}

	*offset = (uint64_t)psObj->map_list.hash.key << PAGE_SHIFT;
#endif

ExitUnref:
	drm_gem_object_unreference(psObj);

ExitUnlock:
	mutex_unlock(&dev->struct_mutex);

	return iRet;
}
#endif

void PVRSRVGEMFreeObject(struct drm_gem_object *obj)
{
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(obj);
	int iSyncIndex;

	for (iSyncIndex = 0; iSyncIndex < ARRAY_SIZE(psPVRObj->apsSyncPrim); iSyncIndex++)
	{
		if (psPVRObj->apsSyncPrim[iSyncIndex] != NULL)
		{
			PVRSRVServerSyncFreeKM(psPVRObj->apsSyncPrim[iSyncIndex]);
		}
	}

	switch (psPVRObj->type)
	{
		case PVR_DRM_GEM_PMR:
#if defined(SUPPORT_DRM_DC_MODULE)
		case PVR_DRM_GEM_DISPLAY_PMR:
#endif
			if (psPVRObj->pmr != NULL)
			{
				PMRUnrefPMR(psPVRObj->pmr);
			}
			break;
#if defined(PVR_DRM_USE_PRIME)
		case PVR_DRM_GEM_IMPORT_PMR:
			if (psPVRObj->pmr != NULL)
			{
				PMRUnrefPMR(psPVRObj->pmr);

				drm_prime_gem_destroy(obj, NULL);
			}
			break;
#endif
		default:
			break;
	}

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)) || defined(CHROMIUMOS_WORKAROUNDS_KERNEL310)
	drm_gem_free_mmap_offset(obj);
#else
	if (obj->map_list.map)
	{
		drm_gem_free_mmap_offset(obj);
	}
#endif

	drm_gem_object_release(obj);
	OSFreeMem(psPVRObj);
}


/*************************************************************************/ /*!
* GEM interface
*/ /**************************************************************************/

int PVRSRVGEMInitObject(struct drm_gem_object *obj,
			enum pvr_drm_gem_object_type type,
			PVRSRV_MEMALLOCFLAGS_T alloc_flags)
{
	struct pvr_drm_dev_priv *psDevPriv = (struct pvr_drm_dev_priv *)obj->dev->dev_private;
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(obj);
	int iSyncIndex;
	PVRSRV_ERROR eError;
	int iRet;

	psPVRObj->type = type;

	eError = PVRSRVGEMCreatePMR(psDevPriv->dev_node,
				    obj,
				    alloc_flags,
				    &psPVRObj->pmr);
	if (eError != PVRSRV_OK)
	{
		return -ENOSPC;
	}

	if (psDevPriv->dev_node->hSyncPrimContext)
	{
		char *pszSyncName;

		BUG_ON(ARRAY_SIZE(psPVRObj->apsSyncPrim) != ARRAY_SIZE(psPVRObj->auiSyncPrimVAddr));

		for (iSyncIndex = 0; iSyncIndex < ARRAY_SIZE(psPVRObj->apsSyncPrim); iSyncIndex++)
		{
			if (iSyncIndex == PVRSRV_GEM_SYNC_TYPE_READ_DISPLAY)
			{
#if defined(SUPPORT_DRM_DC_MODULE)
				if (type != PVR_DRM_GEM_DISPLAY_PMR)
#endif
				{
					break;
				}
			}

			switch (iSyncIndex)
			{
				case PVRSRV_GEM_SYNC_TYPE_WRITE:
					pszSyncName = "pvr_drm_gem_write";
					break;
				case PVRSRV_GEM_SYNC_TYPE_READ_HW:
					pszSyncName = "pvr_drm_gem_read_hw";
					break;
				case PVRSRV_GEM_SYNC_TYPE_READ_SW:
					pszSyncName = "pvr_drm_gem_read_sw";
					break;
				case PVRSRV_GEM_SYNC_TYPE_READ_DISPLAY:
					pszSyncName = "pvr_drm_gem_read_display";
					break;
				default:
					PVR_ASSERT(0);
					pszSyncName = "pvr_drm_gem_unknown";
					break;
			}

			eError = PVRSRVServerSyncAllocKM(psDevPriv->dev_node,
							 &psPVRObj->apsSyncPrim[iSyncIndex],
							 &psPVRObj->auiSyncPrimVAddr[iSyncIndex],
							 strlen(pszSyncName),
							 pszSyncName);
			if (eError != PVRSRV_OK)
			{
				iRet = -ENOMEM;
				goto ErrorServerSyncFree;
			}
		}
	}

	return 0;

ErrorServerSyncFree:
	while (iSyncIndex--)
	{
		PVRSRVServerSyncFreeKM(psPVRObj->apsSyncPrim[iSyncIndex]);
		psPVRObj->apsSyncPrim[iSyncIndex] = NULL;
	}

	PMRUnrefPMR(psPVRObj->pmr);
	psPVRObj->pmr = NULL;

	psPVRObj->type = PVR_DRM_GEM_UNDEFINED;

	return iRet;
}

struct drm_gem_object *PVRSRVGEMObjectCreate(struct drm_device *dev,
					     enum pvr_drm_gem_object_type type,
					     size_t size,
					     PVRSRV_MEMALLOCFLAGS_T flags)
{
	struct drm_gem_object *psObj;
	struct pvr_drm_gem_object *psPVRObj;
	int iRet;

	psPVRObj = OSAllocZMem(sizeof *psPVRObj);
	if (psPVRObj == NULL)
	{
		return ERR_PTR(-ENOMEM);
	}
	psObj = &psPVRObj->base;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)) || defined(CHROMIUMOS_WORKAROUNDS_KERNEL310)
	drm_gem_private_object_init(dev, psObj, ALIGN(size, PAGE_SIZE));
#else
	iRet = drm_gem_private_object_init(dev, psObj, ALIGN(size, PAGE_SIZE));
	if (iRet)
	{
		OSFreeMem(psPVRObj);
		return ERR_PTR(iRet);
	}
#endif
	iRet = PVRSRVGEMInitObject(psObj,
				   type,
				   flags);
	if (iRet)
	{
		drm_gem_object_unreference_unlocked(psObj);
		return ERR_PTR(iRet);
	}

	return psObj;
}

struct drm_gem_object *PVRSRVGEMGetObject(PMR *psPMR)
{
	PMR_GEM_PRIV *psGEMPriv;

	psGEMPriv = PMRGetPrivateDataHack(psPMR, &gsPMRGEMFuncTab);
	if (psGEMPriv != NULL)
	{
		return psGEMPriv->psObj;
	}

	return NULL;
}

PMR *PVRSRVGEMMMapLookupPMR(struct file *psFile, struct vm_area_struct *psVMA)
{
	struct drm_file *psDrmFile = PVR_DRM_FILE_FROM_FILE(psFile);
	struct drm_device *psDev = psDrmFile->minor->dev;
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0))
	struct drm_gem_mm *psMM = psDev->mm_private;
#endif
	struct drm_gem_object *psObj;
	struct pvr_drm_gem_object *psPVRObj;
	PMR *psPMR = NULL;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)) || defined(CHROMIUMOS_WORKAROUNDS_KERNEL310)
	struct drm_vma_offset_node *psNode;
#else
	struct drm_hash_item *psHash;
	struct drm_local_map *psMap;
#endif

	mutex_lock(&psDev->struct_mutex);

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)) || defined(CHROMIUMOS_WORKAROUNDS_KERNEL310)
#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,14,0))
	psNode = drm_vma_offset_exact_lookup(&psMM->vma_manager, psVMA->vm_pgoff, vma_pages(psVMA));
#else
	psNode = drm_vma_offset_exact_lookup(psDev->vma_offset_manager, psVMA->vm_pgoff, vma_pages(psVMA));
#endif
	if (!psNode)
	{
		goto ExitUnlock;
	}

	psObj = container_of(psNode, struct drm_gem_object, vma_node);
#else
	if (drm_ht_find_item(&psMM->offset_hash, psVMA->vm_pgoff, &psHash) != 0)
	{
		goto ExitUnlock;
	}

	psMap = container_of(psHash, struct drm_map_list, hash)->map;
	if (!psMap)
	{
		goto ExitUnlock;
	}

	psObj = psMap->handle;
#endif

	psPVRObj = to_pvr_drm_gem_object(psObj);
	psPMR = psPVRObj->pmr;

ExitUnlock:
	mutex_unlock(&psDev->struct_mutex);

	return psPMR;
}

#endif /* defined(SUPPORT_DRM) */
