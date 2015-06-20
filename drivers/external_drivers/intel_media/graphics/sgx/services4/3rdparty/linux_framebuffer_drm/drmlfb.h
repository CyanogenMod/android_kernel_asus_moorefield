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

#ifndef __MRSTLFB_H__
#define __MRSTLFB_H__

#include <drm/drmP.h>
#include "psb_intel_reg.h"
#include "psb_drm.h"

#define MRST_USING_INTERRUPTS

#define PSB_HWSTAM                0x2098
#define PSB_INSTPM                0x20C0
#define PSB_INT_IDENTITY_R        0x20A4
#define _PSB_VSYNC_PIPEB_FLAG     (1<<5)
#define _PSB_VSYNC_PIPEA_FLAG     (1<<7)
#define _PSB_IRQ_SGX_FLAG         (1<<18)
#define _PSB_IRQ_MSVDX_FLAG       (1<<19)
#define _LNC_IRQ_TOPAZ_FLAG       (1<<20)
#define PSB_INT_MASK_R            0x20A8
#define PSB_INT_ENABLE_R          0x20A0

#define MAX_SWAPCHAINS			  10

#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_NONE (0 << 0)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_A    (1 << 0)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_B    (1 << 1)
#define PVRSRV_SWAPCHAIN_ATTACHED_PLANE_C    (1 << 2)

typedef void *   MRST_HANDLE;

typedef enum tag_mrst_bool
{
	MRST_FALSE = 0,
	MRST_TRUE  = 1,
} MRST_BOOL, *MRST_PBOOL;

typedef int(* MRSTLFB_VSYNC_ISR_PFN)(struct drm_device* psDrmDevice, int iPipe);
typedef int(* MRSTLFB_SCREEN_EVENT_PFN)(struct drm_device* psDrmDevice, int state);


typedef struct MRSTLFB_BUFFER_TAG
{

    IMG_UINT32		             	ui32BufferSize;
	union {

		IMG_SYS_PHYADDR             *psNonCont;

		IMG_SYS_PHYADDR				sCont;
	} uSysAddr;

	IMG_DEV_VIRTADDR             	sDevVAddr;

    IMG_CPU_VIRTADDR             	sCPUVAddr;

	PVRSRV_SYNC_DATA             	*psSyncData;

	MRST_BOOL					 	bIsContiguous;

	MRST_BOOL					 	bIsAllocated;

	IMG_UINT32						ui32OwnerTaskID;
} MRSTLFB_BUFFER;


typedef struct MRSTLFB_VSYNC_FLIP_ITEM_TAG
{



	MRST_HANDLE      hCmdComplete;

	unsigned long    ulSwapInterval;

	MRST_BOOL        bValid;

	MRST_BOOL        bFlipped;

	MRST_BOOL        bCmdCompleted;






	MRSTLFB_BUFFER*	psBuffer;

	struct mdfld_plane_contexts sPlaneContexts;
} MRSTLFB_VSYNC_FLIP_ITEM;

typedef struct MRSTLFB_SWAPCHAIN_TAG
{

	unsigned long       ulBufferCount;

	IMG_UINT32			ui32SwapChainID;
	IMG_UINT32			ui32SwapChainPropertyFlag;
	unsigned long			ulSwapChainGTTOffset;


	MRSTLFB_BUFFER     **ppsBuffer;


	unsigned long	    ulSwapChainLength;


	MRSTLFB_VSYNC_FLIP_ITEM	*psVSyncFlips;


	unsigned long       ulInsertIndex;


	unsigned long       ulRemoveIndex;


	PVRSRV_DC_DISP2SRV_KMJTABLE	*psPVRJTable;


	struct drm_driver         *psDrmDriver;


	struct drm_device         *psDrmDev;

	struct MRSTLFB_SWAPCHAIN_TAG *psNext;

	struct MRSTLFB_DEVINFO_TAG *psDevInfo;
	MRSTLFB_VSYNC_FLIP_ITEM sLastItem;
} MRSTLFB_SWAPCHAIN;

typedef struct MRSTLFB_DEVINFO_TAG
{
	unsigned int           uiDeviceID;

	struct drm_device 	*psDrmDevice;



	MRSTLFB_BUFFER          sSystemBuffer;


	PVRSRV_DC_DISP2SRV_KMJTABLE	sPVRJTable;


	PVRSRV_DC_SRV2DISP_KMJTABLE	sDCJTable;


	unsigned long           ulRefCount;

	MRSTLFB_SWAPCHAIN      *psCurrentSwapChain;

	MRSTLFB_SWAPCHAIN      *apsSwapChains[MAX_SWAPCHAINS];

	IMG_UINT32	   	ui32SwapChainNum;

	MRSTLFB_BUFFER	*psCurrentBuffer;

	void *pvRegs;


	unsigned long ulSetFlushStateRefCount;


	MRST_BOOL           bFlushCommands;


	MRST_BOOL           bBlanked;


	struct fb_info         *psLINFBInfo;


	struct notifier_block   sLINNotifBlock;


	struct mutex		sSwapChainMutex;




	IMG_DEV_VIRTADDR	sDisplayDevVAddr;

	DISPLAY_INFO            sDisplayInfo;


	DISPLAY_FORMAT          sDisplayFormat;


	DISPLAY_DIMS            sDisplayDim;

	IMG_UINT32		ui32MainPipe;


	MRST_BOOL bSuspended;


	MRST_BOOL bLeaveVT;


	unsigned long ulLastFlipAddr;


	MRST_BOOL bLastFlipAddrValid;

	MRST_BOOL bScreenState;

	struct timer_list sFlipTimer;

	struct work_struct flip_complete_work;
}  MRSTLFB_DEVINFO;

#if 0
#define	MRSTLFB_PAGE_SIZE 4096
#define	MRSTLFB_PAGE_MASK (MRSTLFB_PAGE_SIZE - 1)
#define	MRSTLFB_PAGE_TRUNC (~MRSTLFB_PAGE_MASK)

#define	MRSTLFB_PAGE_ROUNDUP(x) (((x) + MRSTLFB_PAGE_MASK) & MRSTLFB_PAGE_TRUNC)
#endif

#ifdef	DEBUG
#define	DEBUG_PRINTK(x) printk x
#else
#define	DEBUG_PRINTK(x)
#endif

#define DISPLAY_DEVICE_NAME "PowerVR Moorestown Linux Display Driver"
#define	DRVNAME	"mrstlfb"
#define	DEVNAME	DRVNAME
#define	DRIVER_PREFIX DRVNAME

typedef enum _MRST_ERROR_
{
	MRST_OK                             =  0,
	MRST_ERROR_GENERIC                  =  1,
	MRST_ERROR_OUT_OF_MEMORY            =  2,
	MRST_ERROR_TOO_FEW_BUFFERS          =  3,
	MRST_ERROR_INVALID_PARAMS           =  4,
	MRST_ERROR_INIT_FAILURE             =  5,
	MRST_ERROR_CANT_REGISTER_CALLBACK   =  6,
	MRST_ERROR_INVALID_DEVICE           =  7,
	MRST_ERROR_DEVICE_REGISTER_FAILED   =  8
} MRST_ERROR;


#ifndef UNREFERENCED_PARAMETER
#define	UNREFERENCED_PARAMETER(param) (param) = (param)
#endif

MRST_ERROR MRSTLFBInit(struct drm_device * dev);
MRST_ERROR MRSTLFBDeinit(void);

void *MRSTLFBAllocKernelMem(unsigned long ulSize);
void MRSTLFBFreeKernelMem(void *pvMem);
MRST_ERROR MRSTLFBGetLibFuncAddr(char *szFunctionName, PFN_DC_GET_PVRJTABLE *ppfnFuncTable);
MRST_ERROR MRSTLFBInstallVSyncISR (MRSTLFB_DEVINFO *psDevInfo, MRSTLFB_VSYNC_ISR_PFN pVsyncHandler);
MRST_ERROR MRSTLFBUninstallVSyncISR(MRSTLFB_DEVINFO *psDevInfo);

void MRSTLFBEnableVSyncInterrupt(MRSTLFB_DEVINFO *psDevInfo);
void MRSTLFBDisableVSyncInterrupt(MRSTLFB_DEVINFO *psDevInfo);

IMG_BOOL MRSTLFBFlipToSurface(MRSTLFB_DEVINFO *psDevInfo,
		  unsigned long uiAddr);

void MRSTLFBSuspend(void);
void MRSTLFBResume(void);

MRST_ERROR MRSTLFBChangeSwapChainProperty(unsigned long *psSwapChainGTTOffset,
		unsigned long ulSwapChainGTTSize, IMG_INT32 i32Pipe);

#endif

