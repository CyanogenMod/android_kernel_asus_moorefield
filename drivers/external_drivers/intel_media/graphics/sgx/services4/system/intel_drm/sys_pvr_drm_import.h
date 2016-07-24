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

#if !defined(__SYS_PVR_DRM_IMPORT_H__)
#define __SYS_PVR_DRM_IMPORT_H__

#if defined(__KERNEL__)
#include "psb_drm.h"
#endif

#if defined( DRM_PVR_RESERVED_INTEL_ORDER )

#define PSB_MODE_OPERATION_MODE_VALID	0x01
#define PSB_MODE_OPERATION_SET_DC_BASE	0x02

#define DRM_PSB_GTT_MAP			0x0F
#define DRM_PSB_GTT_UNMAP		0x10
#define DRM_PSB_GETPAGEADDRS		0x11

#define DRM_PSB_MODE_OPERATION		0x0C

#define DRM_PSB_STOLEN_MEMORY		0x0D

#undef DRM_PSB_PLACEMENT_OFFSET

#if defined(PDUMP)
#define DRM_PSB_PLACEMENT_OFFSET	0x21
#else
#define DRM_PSB_PLACEMENT_OFFSET	0x25
#endif

#define DRM_PVR_RESERVED1	0x12
#define DRM_PVR_RESERVED2	0x13
#define DRM_PVR_RESERVED3	0x14
#define DRM_PVR_RESERVED4	0x15
#define DRM_PVR_RESERVED5	0x16
#define DRM_PVR_RESERVED6	0x1E

#else

#define	pvr_put_user	put_user
#define	pvr_get_user	get_user

#endif

#define DRM_PSB_VT_LEAVE	0x02
#define DRM_PSB_VT_ENTER	0x03

#endif
