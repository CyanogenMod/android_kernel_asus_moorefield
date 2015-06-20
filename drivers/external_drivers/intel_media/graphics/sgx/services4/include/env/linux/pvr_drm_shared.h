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

#if !defined(__PVR_DRM_SHARED_H__)
#define __PVR_DRM_SHARED_H__

#if defined(SUPPORT_DRI_DRM)

#if defined(SUPPORT_DRI_DRM_EXT)
#define PVR_DRM_SRVKM_CMD	DRM_PVR_RESERVED1
#define	PVR_DRM_DISP_CMD	DRM_PVR_RESERVED2
#define	PVR_DRM_BC_CMD		DRM_PVR_RESERVED3
#define PVR_DRM_IS_MASTER_CMD	DRM_PVR_RESERVED4
#define PVR_DRM_UNPRIV_CMD	DRM_PVR_RESERVED5
#define PVR_DRM_DBGDRV_CMD	DRM_PVR_RESERVED6
#else
#define PVR_DRM_SRVKM_CMD	0
#define	PVR_DRM_DISP_CMD	1
#define	PVR_DRM_BC_CMD		2
#define PVR_DRM_IS_MASTER_CMD	3
#define PVR_DRM_UNPRIV_CMD	4
#define PVR_DRM_DBGDRV_CMD	5
#endif

#define	PVR_DRM_UNPRIV_INIT_SUCCESFUL	0

#endif

#endif


