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

#include <linux/string.h>

#include "bufferclass_video.h"
#include "bufferclass_interface.h"

#define VBUFFERCLASS_DEVICE_NAME "Video Bufferclass Device"
#define CBUFFERCLASS_DEVICE_NAME "Camera Bufferclass Device"

static void *gpvAnchorVideo[BC_VIDEO_DEVICE_MAX_ID];

static void *gpcAnchor;

BC_VIDEO_DEVINFO *
GetAnchorPtr(int id)
{
	BC_VIDEO_DEVINFO *AnchorPtr = NULL;
	if (id < BC_VIDEO_DEVICE_MAX_ID)
		AnchorPtr = gpvAnchorVideo[id];
	else if (id == BC_CAMERA_DEVICEID)
		AnchorPtr = gpcAnchor;
	return AnchorPtr;
}

static void
SetAnchorPtr(BC_VIDEO_DEVINFO * psDevInfo, int id)
{
	if (id < BC_VIDEO_DEVICE_MAX_ID)
		gpvAnchorVideo[id] = (void *) psDevInfo;
	else if (id == BC_CAMERA_DEVICEID)
		gpcAnchor = (void *) psDevInfo;
}

BCE_ERROR
BC_Video_Register(int id)
{
	BC_VIDEO_DEVINFO *psDevInfo;

	psDevInfo = GetAnchorPtr(id);

	if (psDevInfo == NULL) {
		psDevInfo =
			(BC_VIDEO_DEVINFO *) BCAllocKernelMem(sizeof(BC_VIDEO_DEVINFO));

		if (!psDevInfo) {
			return (BCE_ERROR_OUT_OF_MEMORY);
		}

		SetAnchorPtr((void *) psDevInfo, id);

		psDevInfo->ulRefCount = 0;

		if (BCOpenPVRServices(&psDevInfo->hPVRServices) != BCE_OK) {
			return (BCE_ERROR_INIT_FAILURE);
		}

		psDevInfo->ulNumBuffers = 0;

		psDevInfo->sBufferInfo.eIMGPixFmt = IMG_PIXFMT_UNKNOWN;
		psDevInfo->sBufferInfo.ui32Width = 0;
		psDevInfo->sBufferInfo.ui32Height = 0;
		psDevInfo->sBufferInfo.ui32ByteStride = 0;
		psDevInfo->sBufferInfo.ui32BufferDeviceID = id;
		psDevInfo->sBufferInfo.ui32Flags = 0;
		psDevInfo->sBufferInfo.ui32BufferCount =
			(IMG_UINT32) psDevInfo->ulNumBuffers;

		if (id < BC_VIDEO_DEVICE_MAX_ID) {
			strncpy(psDevInfo->sBufferInfo.szDeviceName,
				VBUFFERCLASS_DEVICE_NAME, MAX_BUFFER_DEVICE_NAME_SIZE);
		} else if (id == BC_CAMERA_DEVICEID) {
			strncpy(psDevInfo->sBufferInfo.szDeviceName,
				CBUFFERCLASS_DEVICE_NAME, MAX_BUFFER_DEVICE_NAME_SIZE);
		}
	}

	psDevInfo->ulRefCount++;

	return (BCE_OK);
}

BCE_ERROR
BC_Video_Unregister(int id)
{
	BC_VIDEO_DEVINFO *psDevInfo;

	psDevInfo = GetAnchorPtr(id);

	if (psDevInfo == NULL) {
		return (BCE_ERROR_GENERIC);
	}

	psDevInfo->ulRefCount--;

	if (psDevInfo->ulRefCount == 0) {
		if (BCClosePVRServices(psDevInfo->hPVRServices) != BCE_OK) {
			psDevInfo->hPVRServices = NULL;
			return (BCE_ERROR_GENERIC);
		}

		if (psDevInfo->psSystemBuffer) {
			BCFreeKernelMem(psDevInfo->psSystemBuffer);
		}

		BCFreeKernelMem(psDevInfo);

		SetAnchorPtr(NULL, id);
	}
	return (BCE_OK);
}

BCE_ERROR
BC_Video_Init(int id)
{
	BCE_ERROR eError;

	eError = BC_Video_Register(id);
	if (eError != BCE_OK) {
		return eError;
	}

	return (BCE_OK);
}

BCE_ERROR
BC_Video_Deinit(int id)
{
	BCE_ERROR eError;

	BCVideoDestroyBuffers(id);

	eError = BC_Video_Unregister(id);
	if (eError != BCE_OK) {
		return eError;
	}

	return (BCE_OK);
}
