/*************************************************************************/ /*!
@File
@Title          Services DRM definitions shared between kernel and user space.
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

#if !defined(__PVR_DRM_SHARED_H__)
#define __PVR_DRM_SHARED_H__

#if defined(SUPPORT_DRM)
#include <linux/types.h>

/* 
 * DRM command numbers, relative to DRM_COMMAND_BASE. 
 * These defines must be prefixed with "DRM_".
 */
#define DRM_PVR_SRVKM_CMD			0x12 /* Used for PVR Services ioctls */
#define DRM_PVR_DBGDRV_CMD			1 /* Debug driver (PDUMP) ioctls */
#define DRM_PVR_UNPRIV_CMD			2 /* PVR driver unprivileged ioctls */
#define DRM_PVR_GEM_CREATE			3
#define DRM_PVR_GEM_TO_IMG_HANDLE		4
#define DRM_PVR_IMG_TO_GEM_HANDLE		5
#define DRM_PVR_GEM_SYNC_GET			6


#if !defined(SUPPORT_KERNEL_SRVINIT)
/* Subcommands of DRM_PVR_UNPRIV_CMD */
#define	DRM_PVR_UNPRIV_CMD_INIT_SUCCESFUL	0 /* PVR Services init succesful */

typedef struct drm_pvr_unpriv_cmd_tag
{
	uint32_t	cmd;
	int32_t		result;
} drm_pvr_unpriv_cmd;
#endif	/* #if !defined(SUPPORT_KERNEL_SRVINIT) */

#define PVR_GEM_USE_SCANOUT	(1 << 0)
#define PVR_GEM_USE_CURSOR	(2 << 0)

typedef	struct drm_pvr_gem_create_tag
{
	/* Input parameters (preserved by the ioctl) */
	uint64_t	size;
	uint32_t	alloc_flags;
	uint32_t	usage_flags;

	/* Output parameters */
	uint32_t	handle;
	uint32_t	pad;
} drm_pvr_gem_create;

typedef	struct drm_pvr_gem_to_img_handle_tag
{	
	/* Input parameters (preserved by the ioctl) */
	uint32_t	gem_handle;
	uint32_t	pad;

	/* Output parameters */
	uint64_t	img_handle;
} drm_pvr_gem_to_img_handle;

typedef	struct drm_pvr_img_to_gem_handle_tag
{	
	/* Input parameters (preserved by the ioctl) */
	uint64_t	img_handle;

	/* Output parameters */
	uint32_t	gem_handle;
	uint32_t	pad;
} drm_pvr_img_to_gem_handle;

typedef struct drm_pvr_gem_sync_get_tag
{
	/* Input parameters (preserved by the ioctl) */
	uint32_t	gem_handle;
	uint32_t	type;

	/* Output parameters */
	uint64_t	sync_handle;
	uint32_t	firmware_addr;
	uint32_t	pad;
} drm_pvr_gem_sync_get;

#endif /* defined(SUPPORT_DRM) */
#endif /* defined(__PVR_DRM_SHARED_H__) */
