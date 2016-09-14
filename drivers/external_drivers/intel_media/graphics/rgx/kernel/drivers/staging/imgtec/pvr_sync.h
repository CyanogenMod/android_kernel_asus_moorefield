/*************************************************************************/ /*!
@File           pvr_sync.h
@Title          Kernel driver for Android's sync mechanism
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
/* vi: set ts=8: */

#ifndef _PVR_SYNC_H
#define _PVR_SYNC_H

#include "pvr_fd_sync_user.h"
#include "rgx_fwif_shared.h"

/* Services internal interface */
enum PVRSRV_ERROR pvr_sync_init(void);
void pvr_sync_deinit(void);

/* to keep track of the intermediate allocations done for the FD merge */
struct pvr_sync_fd_merge_data
{
	PRGXFWIF_UFO_ADDR *pauiFenceUFOAddress;
	__u32             *paui32FenceValue;
	PRGXFWIF_UFO_ADDR *pauiUpdateUFOAddress;
	__u32             *paui32UpdateValue;
};

enum PVRSRV_ERROR 
pvr_sync_merge_fences(__u32                         *pui32ClientFenceCountOut,
		      PRGXFWIF_UFO_ADDR             **ppauiFenceUFOAddressOut,
		      __u32                         **ppaui32FenceValueOut,
		      __u32                         *pui32ClientUpdateCountOut,
		      PRGXFWIF_UFO_ADDR             **ppauiUpdateUFOAddressOut,
		      __u32                         **ppaui32UpdateValueOut,
		      const char                    *pszName,
		      bool                          bUpdate,
		      const __u32                   ui32NumFDs,
		      const __s32                   *paui32FDs,
		      struct pvr_sync_fd_merge_data *psFDMergeData);

IMG_VOID pvr_sync_merge_fences_cleanup(struct pvr_sync_fd_merge_data *psFDMergeData);
enum PVRSRV_ERROR pvr_sync_nohw_update_fence(__s32 i32FDFence);

#endif /* _PVR_SYNC_H */
