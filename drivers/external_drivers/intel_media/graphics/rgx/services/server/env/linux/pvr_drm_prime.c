/*************************************************************************/ /*!
@File
@Title          PowerVR DRM GEM Prime interface
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    Interface for managing prime memory.
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

#include "pvr_drm.h"
#include "allocmem.h"

#if defined(PVR_DRM_USE_PRIME)
#include "pmr.h"

#if defined(PVR_RI_DEBUG)
#include "ri_server.h"
#endif


/*************************************************************************/ /*!
* DRM Prime helper callbacks
*/ /**************************************************************************/

static struct sg_table *PrimeMapDmaBuf(struct dma_buf_attachment *psAttachment,
				       enum dma_data_direction eDir)
{
	struct drm_gem_object *psObj = psAttachment->dmabuf->priv;
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(psObj);
#if !defined(LMA)
	IMG_DEVMEM_SIZE_T uiSize = 0;
#endif
	struct sg_table *psSGTable;
	IMG_CPU_PHYADDR sCpuPAddr;
	IMG_BOOL bCpuPAddrValid;
	struct scatterlist *psScatterList;
	IMG_UINT uiPageCount;
	IMG_INT iPageNum;
	PVRSRV_ERROR eError;
	IMG_INT iErr;

	psSGTable = OSAllocMem(sizeof(*psSGTable));
	if (!psSGTable)
	{
		return ERR_PTR(-ENOMEM);
	}

#if defined(LMA)
	uiPageCount = 1;
#else
	eError = PMR_LogicalSize(psPVRObj->pmr, &uiSize);
	PVR_ASSERT(eError == PVRSRV_OK);

	uiPageCount = (IMG_SIZE_T)uiSize >> PAGE_SHIFT;
#endif

	iErr = sg_alloc_table(psSGTable, uiPageCount, GFP_KERNEL);
	if (iErr)
	{
		goto ErrorFreeSGTable;
	}

	eError = PMRLockSysPhysAddresses(psPVRObj->pmr, PAGE_SHIFT);
	if (eError != PVRSRV_OK)
	{
		iErr = -EPERM;
		goto ErrorSGFreeTable;
	}

	for_each_sg (psSGTable->sgl, psScatterList, uiPageCount, iPageNum)
	{
		eError = PMR_CpuPhysAddr(psPVRObj->pmr,
					 PAGE_SHIFT,
					 1,
					 iPageNum << PAGE_SHIFT,
					 &sCpuPAddr,
					 &bCpuPAddrValid);
		if (eError != PVRSRV_OK || !bCpuPAddrValid)
		{
			iErr = -ENOMEM;
			goto ErrorUnlockPMR;
		}

		sg_set_page(psScatterList, pfn_to_page(PFN_DOWN(sCpuPAddr.uiAddr)), PAGE_SIZE, 0);
	}

	(void)dma_map_sg(psAttachment->dev, psSGTable->sgl, psSGTable->nents, eDir);

	return psSGTable;

ErrorUnlockPMR:
	PMRUnlockSysPhysAddresses(psPVRObj->pmr);

ErrorSGFreeTable:
	sg_free_table(psSGTable);

ErrorFreeSGTable:
	OSFreeMem(psSGTable);

	return ERR_PTR(iErr);
}

static void PrimeUnmapDmaBuf(struct dma_buf_attachment *psAttachment,
			     struct sg_table *psSGTable,
			     enum dma_data_direction eDir)
{
	struct drm_gem_object *psObj = psAttachment->dmabuf->priv;
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(psObj);
	PVRSRV_ERROR eError;

	dma_unmap_sg(psAttachment->dev, psSGTable->sgl, psSGTable->nents, eDir);

	sg_free_table(psSGTable);
	OSFreeMem(psSGTable);

	eError = PMRUnlockSysPhysAddresses(psPVRObj->pmr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING, "%s: failed to unlock PMR (%d)\n",
			 __FUNCTION__, eError));
	}
}

static void PrimeRelease(struct dma_buf *psDmaBuf)
{
	struct drm_gem_object *psObj = psDmaBuf->priv;

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0)) && ! defined(CHROMIUMOS_WORKAROUNDS_KERNEL310)
	if (psObj->export_dma_buf == psDmaBuf)
	{
		psObj->export_dma_buf = NULL;

		drm_gem_object_unreference_unlocked(psObj);
	}
#else
	drm_gem_object_unreference_unlocked(psObj);
#endif
}

static int PrimeBeginCpuAccess(struct dma_buf *psDmaBuf,
			       size_t uiStart,
			       size_t uiLen,
			       enum dma_data_direction unref__ eDir)
{
	struct drm_gem_object *psObj = psDmaBuf->priv;
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(psObj);
	PVRSRV_ERROR eError;

	if (uiStart + uiLen > psDmaBuf->size)
	{
		return -EINVAL;
	}

	eError = PMRLockSysPhysAddresses(psPVRObj->pmr, PAGE_SHIFT);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed to lock PMR (%d)\n",
			 __FUNCTION__, eError));

		return (eError == PVRSRV_ERROR_PMR_FAILED_TO_ALLOC_PAGES) ? -ENOMEM : -EINVAL;
	}

	return 0;
}

static void PrimeEndCpuAccess(struct dma_buf *psDmaBuf,
			      size_t unref__ uiStart,
			      size_t unref__ uiLen,
			      enum dma_data_direction unref__ eDir)
{
	struct drm_gem_object *psObj = psDmaBuf->priv;
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(psObj);
	PVRSRV_ERROR eError;

	eError = PMRUnlockSysPhysAddresses(psPVRObj->pmr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING,
			 "%s: failed to unlock PMR (%d)\n",
			 __FUNCTION__, eError));
	}
}

static void *PrimeKMap(struct dma_buf unref__ *psDmaBuf,
		       unsigned long unref__ ulPageNum)
{
	return NULL;
}

static void *PrimeKMapAtomic(struct dma_buf unref__ *psDmaBuf,
			     unsigned long unref__ ulPageNum)
{
	return NULL;
}

static int PrimeMMap(struct dma_buf unref__ *psDmaBuf,
		     struct vm_area_struct unref__ *psVMA)
{
	return -EINVAL;
}

static void *PrimeVMap(struct dma_buf *psDmaBuf)
{
	struct drm_gem_object *psObj = psDmaBuf->priv;
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(psObj);
	void *pvCpuVAddr;
	IMG_SIZE_T uiMappedLength;
	IMG_HANDLE hMappingHandle;
	PVRSRV_ERROR eError;

	eError = PMRAcquireKernelMappingData(psPVRObj->pmr,
					     0,
					     0,
					     &pvCpuVAddr,
					     &uiMappedLength,
					     &hMappingHandle);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_ERROR,
			 "%s: failed to acquire kernel mapping data (%d)\n",
			 __FUNCTION__, eError));

		return ERR_PTR(-EINVAL);
	}

	/* We don't have anywhere to store the mapping handle but because we're 
	   mapping the entire buffer we assume that the handle is actually the 
	   same as the virtual address we get back */
	PVR_ASSERT((void *)hMappingHandle == pvCpuVAddr);

	return pvCpuVAddr;
}

static void PrimeVUnmap(struct dma_buf *psDmaBuf, void *pvVAddr)
{
	struct drm_gem_object *psObj = psDmaBuf->priv;
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(psObj);
	PVRSRV_ERROR eError;

	eError = PMRReleaseKernelMappingData(psPVRObj->pmr, (IMG_HANDLE)pvVAddr);
	if (eError != PVRSRV_OK)
	{
		PVR_DPF((PVR_DBG_WARNING,
			 "%s: failed to release kernel mapping data (%d)\n",
			 __FUNCTION__, eError));
	}
}

static const struct dma_buf_ops gsPrimeOps =
{
	.map_dma_buf		= PrimeMapDmaBuf,
	.unmap_dma_buf		= PrimeUnmapDmaBuf,
	.release		= PrimeRelease,
	.begin_cpu_access	= PrimeBeginCpuAccess,
	.end_cpu_access		= PrimeEndCpuAccess,
	.kmap			= PrimeKMap,
	.kmap_atomic		= PrimeKMapAtomic,
	.mmap			= PrimeMMap,
	.vmap			= PrimeVMap,
	.vunmap			= PrimeVUnmap,
};

struct dma_buf *PVRSRVPrimeExport(struct drm_device unref__ *dev,
				  struct drm_gem_object *obj,
				  int flags)
{
	struct pvr_drm_gem_object *psPVRObj = to_pvr_drm_gem_object(obj);

	switch (psPVRObj->type)
	{
		case PVR_DRM_GEM_PMR:
		case PVR_DRM_GEM_DISPLAY_PMR:
			break;
		default:
			PVR_ASSERT(0);
			return ERR_PTR(-EINVAL);
	}

	return dma_buf_export(obj,
			      &gsPrimeOps,
			      obj->size,
			      flags
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,17,0))
			      , NULL
#endif
			     );
}

struct drm_gem_object *PVRSRVPrimeImport(struct drm_device *dev,
					 struct dma_buf *dma_buf)
{
	struct drm_gem_object *psObj;
	struct pvr_drm_gem_object *psPVRObj;
	int iRet;

	if (dma_buf->ops == &gsPrimeOps)
	{
		psObj = dma_buf->priv;

		if (psObj->dev == dev)
		{
			drm_gem_object_reference(psObj);
			return psObj;
		}
	}

	psPVRObj = OSAllocZMem(sizeof *psPVRObj);
	if (psPVRObj == NULL)
	{
		return ERR_PTR(-ENOMEM);
	}
	psObj = &psPVRObj->base;

#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3,12,0)) || defined(CHROMIUMOS_WORKAROUNDS_KERNEL310)
	drm_gem_private_object_init(dev, psObj, dma_buf->size);
#else
	iRet = drm_gem_private_object_init(dev, psObj, dma_buf->size);
	if (iRet)
	{
		OSFreeMem(psPVRObj);
		return ERR_PTR(iRet);
	}
#endif
	psObj->import_attach = dma_buf_attach(dma_buf, dev->dev);
	if (IS_ERR(psObj->import_attach))
	{
		iRet = PTR_ERR(psObj->import_attach);
		goto ErrorGemUnref;
	}

	get_dma_buf(dma_buf);

	iRet = PVRSRVGEMInitObject(psObj,
				   PVR_DRM_GEM_IMPORT_PMR,
				   PVRSRV_MEMALLOCFLAG_UNCACHED);
	if (iRet)
	{
		goto ErrorDmaBufDetach;
	}

	return psObj;

ErrorDmaBufDetach:
	dma_buf_detach(dma_buf, psObj->import_attach);

ErrorGemUnref:
	drm_gem_object_unreference_unlocked(psObj);

	return ERR_PTR(iRet);
}

#endif /* defined(PVR_DRM_USE_PRIME) */
#endif /* defined(SUPPORT_DRM) */
