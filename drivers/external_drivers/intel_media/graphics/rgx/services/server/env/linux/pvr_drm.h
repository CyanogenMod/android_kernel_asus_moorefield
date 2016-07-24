/*************************************************************************/ /*!
@File
@Title          PowerVR drm driver
@Copyright      Copyright (c) Imagination Technologies Ltd. All Rights Reserved
@Description    drm module
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

#if !defined(__PVR_DRM_H__)
#define __PVR_DRM_H__

#include <linux/version.h>
#include <drm/drmP.h>

/*
 * Check for a kernel patched by IMG, with DMA-BUF and PRIME components
 * from a later version of the kernel.
 */
#if !defined(DRM_PRIME_LINUX_VERSION_CODE)
#define	DRM_PRIME_LINUX_VERSION_CODE LINUX_VERSION_CODE
#endif

#if (DRM_PRIME_LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,3))
#define	PVR_DRM_USE_PRIME
#endif

#if defined(LMA)
#define PVR_DRM_PHYS_HEAP	PVRSRV_DEVICE_PHYS_HEAP_GPU_LOCAL
#else
#define PVR_DRM_PHYS_HEAP	PVRSRV_DEVICE_PHYS_HEAP_CPU_LOCAL
#endif

#if defined(LDM_PLATFORM)
#define	LDM_DEV	struct platform_device
#endif /*LDM_PLATFORM */

#if defined(LDM_PCI)
#define	LDM_DEV	struct pci_dev
#endif /* LDM_PCI */

#include "connection_server.h"
#include "pvr_drm_external.h"
#include "pvr_drm_shared.h"
#include "pvr_drm_display.h"
#include "sync_server.h"
#include "pmr.h"

#if defined(PVR_DRM_USE_PRIME)
#include <linux/dma-buf.h>
#endif

#if defined(SUPPORT_DRM_DC_MODULE)
#include <linux/spinlock.h>
#include "scp.h"
#endif

#if defined(PDUMP)
#include "linuxsrv.h"
#endif

#if defined(SUPPORT_DRM)
#if (!defined(LDM_PLATFORM) && !defined(LDM_PCI)) || \
	(defined(LDM_PLATFORM) && defined(LDM_PCI))
	#error "LDM_PLATFORM or LDM_PCI must be defined"
#endif

#define	MAKESTRING(x)	#x
#define TOSTRING(x)	MAKESTRING(x)

#define	PVR_DRM_FILE_FROM_FILE(pFile)		((struct drm_file *)((pFile)->private_data))
#define	PVR_FILE_FROM_DRM_FILE(pDRMFile)	((pDRMFile)->filp)

struct pvr_drm_dev_priv
{
#if defined(SUPPORT_DRM_DC_MODULE)
	/* The DRM device funcs *MUST* be the first field in the structure
	   as pvr_drm_display.h relies on this being the case. */
	struct pvr_drm_device_funcs funcs;
	void *display_priv;

	spinlock_t flip_done_lock;
	struct list_head flip_done_head;
	SCP_CONTEXT *display_flip_context;

	IMG_HANDLE display_misr;
	IMG_HANDLE display_notify;
#endif

	PVRSRV_DEVICE_NODE *dev_node;

#if defined(SUPPORT_SYSTEM_INTERRUPT_HANDLING)
	IMG_HANDLE *hSysData;
#else
	unsigned int irq;
	pvr_drm_irq_handler irq_handler;
#endif
};

enum pvr_drm_gem_object_type
{
	PVR_DRM_GEM_UNDEFINED = 0,
	PVR_DRM_GEM_PMR,
	PVR_DRM_GEM_DISPLAY_PMR,
	PVR_DRM_GEM_IMPORT_PMR,
};

struct pvr_drm_gem_object
{
	enum pvr_drm_gem_object_type type;
	struct drm_gem_object base;
	PMR *pmr;
	void *obj;
	SERVER_SYNC_PRIMITIVE *apsSyncPrim[PVRSRV_GEM_SYNC_TYPE_COUNT];
	IMG_UINT32 auiSyncPrimVAddr[PVRSRV_GEM_SYNC_TYPE_COUNT];
};

#define to_pvr_drm_gem_object(obj) container_of(obj, struct pvr_drm_gem_object, base)

extern struct drm_driver sPVRDRMDriver;

int PVRSRVSystemInit(struct drm_device *pDrmDevice);
void PVRSRVSystemDeInit(LDM_DEV *pDevice);

int PVRSRVOpen(struct drm_device *dev, struct drm_file *file);
void PVRSRVRelease(struct drm_device *dev, struct drm_file *file);

#if defined(PDUMP)
int dbgdrv_init(void);
void dbgdrv_cleanup(void);
int dbgdrv_ioctl(struct drm_device *dev, void *arg, struct drm_file *file);
int dbgdrv_ioctl_compat(struct drm_device *dev, void *arg, struct drm_file *file);
#endif

int PVRSRV_BridgeDispatchKM(struct drm_device *dev, void *arg, struct drm_file *file);

#if defined(CONFIG_COMPAT)
int PVRSRV_BridgeCompatDispatchKM(struct file *file, unsigned int cmd, unsigned long arg);
#endif

int PVRDRMGEMCreate(struct drm_device *dev, void *arg, struct drm_file *file);
int PVRDRMGEMToIMGHandle(struct drm_device *dev, void *arg, struct drm_file *file);
int PVRDRMIMGToGEMHandle(struct drm_device *dev, void *arg, struct drm_file *file);
int PVRDRMGEMSyncGet(struct drm_device *dev, void *arg, struct drm_file *file);

int PVRSRVGEMInitObject(struct drm_gem_object *obj,
			enum pvr_drm_gem_object_type type,
			PVRSRV_MEMALLOCFLAGS_T alloc_flags);

struct drm_gem_object *PVRSRVGEMObjectCreate(struct drm_device *dev,
					     enum pvr_drm_gem_object_type type,
					     size_t size,
					     PVRSRV_MEMALLOCFLAGS_T alloc_flags);
void PVRSRVGEMFreeObject(struct drm_gem_object *obj);

PVRSRV_ERROR PVRSRVGEMCreatePMR(PVRSRV_DEVICE_NODE *psDevNode,
				struct drm_gem_object *psObj,
				PVRSRV_MEMALLOCFLAGS_T uiFlags,
				PMR **ppsPMR);
struct drm_gem_object *PVRSRVGEMGetObject(PMR *psPMR);
PMR *PVRSRVGEMMMapLookupPMR(struct file *psFile, struct vm_area_struct *psVMA);

#if defined(PVR_DRM_USE_PRIME)
struct drm_gem_object *PVRSRVPrimeImport(struct drm_device *dev, struct dma_buf *dma_buf);
struct dma_buf *PVRSRVPrimeExport(struct drm_device unref__ *dev, struct drm_gem_object *obj, int flags);
#endif

#if defined(SUPPORT_DRM_DC_MODULE)
int PVRSRVDRMDisplayInit(struct drm_device *dev);
int PVRSRVDRMDisplayDeinit(struct drm_device *dev);

PVRSRV_ERROR PVRSRVDRMDisplayCreatePMR(PVRSRV_DEVICE_NODE *psDevNode,
				       struct drm_device *dev,
				       size_t size,
				       PVRSRV_MEMALLOCFLAGS_T uiFlags,
				       PMR **ppsPMR,
				       void **buffer);
u32 PVRSRVDRMDisplayGetVBlankCounter(struct drm_device *dev, int crtc);
int PVRSRVDRMDisplayEnableVBlank(struct drm_device *dev, int crtc);
void PVRSRVDRMDisplayDisableVBlank(struct drm_device *dev, int crtc);

int PVRSRVGEMDumbCreate(struct drm_file *file, struct drm_device *dev, struct drm_mode_create_dumb *args);
int PVRSRVGEMDumbDestroy(struct drm_file *file, struct drm_device *dev, uint32_t handle);
int PVRSRVGEMDumbMapOffset(struct drm_file *file, struct drm_device *dev, uint32_t handle, uint64_t *offset);
#endif


/* These defines must be prefixed with "DRM_IOCTL_". */
#define	DRM_IOCTL_PVR_SRVKM_CMD			_IOWR(0, DRM_PVR_SRVKM_CMD, PVRSRV_BRIDGE_PACKAGE)
#define	DRM_IOCTL_PVR_UNPRIV_CMD		_IOWR(0, DRM_PVR_UNPRIV_CMD, drm_pvr_unpriv_cmd)

#if defined(PDUMP)
#define	DRM_IOCTL_PVR_DBGDRV_CMD		_IOWR(0, DRM_PVR_DBGDRV_CMD, IOCTL_PACKAGE)
#endif

#define	DRM_IOCTL_PVR_GEM_CREATE		_IOWR(0, DRM_PVR_GEM_CREATE, drm_pvr_gem_create)
#define	DRM_IOCTL_PVR_GEM_TO_IMG_HANDLE		_IOWR(0, DRM_PVR_GEM_TO_IMG_HANDLE, drm_pvr_gem_to_img_handle)
#define	DRM_IOCTL_PVR_IMG_TO_GEM_HANDLE		_IOWR(0, DRM_PVR_IMG_TO_GEM_HANDLE, drm_pvr_img_to_gem_handle)
#define DRM_IOCTL_PVR_GEM_SYNC_GET		_IOWR(0, DRM_PVR_GEM_SYNC_GET, drm_pvr_gem_sync_get)

#endif	/* defined(SUPPORT_DRM) */
#endif /* !defined(__PVR_DRM_H__) */
