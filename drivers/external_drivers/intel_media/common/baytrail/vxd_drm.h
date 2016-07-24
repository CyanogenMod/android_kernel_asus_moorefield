/**************************************************************************
 * Copyright (c) 2007-2008, Intel Corporation.
 * All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * this program; if not, write to the Free Software Foundation, Inc.,
 * 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
 *
 **************************************************************************/

#ifndef _VXD_DRM_H_
#define _VXD_DRM_H_

#if (LINUX_VERSION_CODE < KERNEL_VERSION(3, 8, 0))
#include "drm.h"
#else
#include <uapi/drm/drm.h>
#endif

#if defined(__linux__) && !defined(__KERNEL__)
#include<stdint.h>
#include <linux/types.h>
#include "drm_mode.h"
#endif

#include "psb_ttm_fence_user.h"
#include "psb_ttm_placement_user.h"

#define DRM_PSB_EXTENSION		0x50
#define DRM_PSB_GETPAGEADDRS		0x51
#define DRM_PSB_CMDBUF			0x52
#define DRM_PSB_TTM_PL_CREATE		0x53
#define DRM_PSB_TTM_PL_REFERENCE	0x54
#define DRM_PSB_TTM_PL_UNREF		0x55
#define DRM_PSB_TTM_PL_SYNCCPU		0x56
#define DRM_PSB_TTM_PL_WAITIDLE		0x57
#define DRM_PSB_TTM_PL_SETSTATUS	0x58
#define DRM_PSB_TTM_PL_CREATE_UB	0x59
#define DRM_PSB_TTM_FENCE_SIGNALED	0x5a
#define DRM_PSB_TTM_FENCE_FINISH 	0x5b
#define DRM_PSB_TTM_FENCE_UNREF		0x5c
#define DRM_PSB_VIDEO_GETPARAM		0x5d

#define DRM_COMMAND_VXD_OFFSET 		0x50
#define DRM_COMMAND_VXD_BASE		0x90

#define DRM_PSB_PLACEMENT_OFFSET 	0x53
#define DRM_PSB_FENCE_OFFSET		0x5a

#define PSB_RELOC_MAGIC         0x67676767
#define PSB_RELOC_SHIFT_MASK    0x0000FFFF
#define PSB_RELOC_SHIFT_SHIFT   0
#define PSB_RELOC_ALSHIFT_MASK  0xFFFF0000
#define PSB_RELOC_ALSHIFT_SHIFT 16

#define PSB_RELOC_OP_OFFSET     0	/* Offset of the indicated
					 * buffer
					 */

struct drm_psb_reloc {
	uint32_t reloc_op;
	uint32_t where;		/* offset in destination buffer */
	uint32_t buffer;	/* Buffer reloc applies to */
	uint32_t mask;		/* Destination format: */
	uint32_t shift;		/* Destination format: */
	uint32_t pre_add;	/* Destination format: */
	uint32_t background;	/* Destination add */
	uint32_t dst_buffer;	/* Destination buffer. Index into buffer_list */
	uint32_t arg0;		/* Reloc-op dependant */
	uint32_t arg1;
};

#define PSB_GPU_ACCESS_READ         (1ULL << 32)
#define PSB_GPU_ACCESS_WRITE        (1ULL << 33)
#define PSB_GPU_ACCESS_MASK         (PSB_GPU_ACCESS_READ | PSB_GPU_ACCESS_WRITE)

#define PSB_BO_FLAG_COMMAND         (1ULL << 52)

#define PSB_ENGINE_2D 2
#define PSB_ENGINE_DECODE 0
#define PSB_ENGINE_VIDEO 0
#define LNC_ENGINE_ENCODE 1
#define VSP_ENGINE_VPP 6

/*
 * For this fence class we have a couple of
 * fence types.
 */

#define _PSB_FENCE_EXE_SHIFT           0
#define _PSB_FENCE_FEEDBACK_SHIFT      4

#define _PSB_FENCE_TYPE_EXE         (1 << _PSB_FENCE_EXE_SHIFT)
#define _PSB_FENCE_TYPE_FEEDBACK    (1 << _PSB_FENCE_FEEDBACK_SHIFT)

#define PSB_NUM_ENGINES 7

#define PSB_FEEDBACK_OP_VISTEST (1 << 0)

struct drm_psb_extension_rep {
	int32_t exists;
	uint32_t driver_ioctl_offset;
	uint32_t sarea_offset;
	uint32_t major;
	uint32_t minor;
	uint32_t pl;
};

#define DRM_PSB_EXT_NAME_LEN 128

union drm_psb_extension_arg {
	char extension[DRM_PSB_EXT_NAME_LEN];
	struct __user drm_psb_extension_rep rep;
};

struct psb_validate_req {
	uint64_t set_flags;
	uint64_t clear_flags;
	uint64_t next;
	uint64_t presumed_gpu_offset;
	uint32_t buffer_handle;
	uint32_t presumed_flags;
	uint32_t group;
	uint32_t pad64;
};

struct psb_validate_rep {
	uint64_t gpu_offset;
	uint32_t placement;
	uint32_t fence_type_mask;
};

#define PSB_USE_PRESUMED     (1 << 0)

struct psb_validate_arg {
	uint64_t handled;
	uint64_t ret;
	union {
		struct psb_validate_req req;
		struct psb_validate_rep rep;
	} d;
};

#define DRM_PSB_FENCE_NO_USER        (1 << 0)

struct psb_ttm_fence_rep {
	uint32_t handle;
	uint32_t fence_class;
	uint32_t fence_type;
	uint32_t signaled_types;
	uint32_t error;
};

typedef struct drm_psb_cmdbuf_arg {
	uint64_t buffer_list;	/* List of buffers to validate */
	uint64_t fence_arg;

	uint32_t cmdbuf_handle;	/* 2D Command buffer object or, */
	uint32_t cmdbuf_offset;	/* rasterizer reg-value pairs */
	uint32_t cmdbuf_size;

	uint32_t reloc_handle;	/* Reloc buffer object */
	uint32_t reloc_offset;
	uint32_t num_relocs;

	/* Not implemented yet */
	uint32_t fence_flags;
	uint32_t engine;

} drm_psb_cmdbuf_arg_t;

struct drm_psb_pageflip_arg_t {
	uint32_t flip_offset;
	uint32_t stride;
};

enum lnc_getparam_key {
	LNC_VIDEO_DEVICE_INFO,
	LNC_VIDEO_GETPARAM_RAR_INFO,
	LNC_VIDEO_GETPARAM_CI_INFO,
	LNC_VIDEO_FRAME_SKIP,
	IMG_VIDEO_DECODE_STATUS,
	IMG_VIDEO_NEW_CONTEXT,
	IMG_VIDEO_RM_CONTEXT,
	IMG_VIDEO_UPDATE_CONTEXT,
	IMG_VIDEO_MB_ERROR,
	IMG_VIDEO_SET_DISPLAYING_FRAME,
	IMG_VIDEO_GET_DISPLAYING_FRAME,
	IMG_VIDEO_GET_HDMI_STATE,
	IMG_VIDEO_SET_HDMI_STATE,
	PNW_VIDEO_QUERY_ENTRY,
	IMG_DISPLAY_SET_WIDI_EXT_STATE,
	IMG_VIDEO_IED_STATE
};

struct drm_lnc_video_getparam_arg {
	uint64_t key;
	uint64_t arg;		/* argument pointer */
	uint64_t value;		/* feed back pointer */
};

struct drm_video_displaying_frameinfo {
	uint32_t buf_handle;
	uint32_t width;
	uint32_t height;
	uint32_t size;		/* buffer size */
	uint32_t format;	/* fourcc */
	uint32_t luma_stride;	/* luma stride */
	uint32_t chroma_u_stride;	/* chroma stride */
	uint32_t chroma_v_stride;
	uint32_t luma_offset;	/* luma offset from the beginning of the memory */
	uint32_t chroma_u_offset;	/* UV offset from the beginning of the memory */
	uint32_t chroma_v_offset;
	uint32_t reserved;
};

struct drm_psb_dev_info_arg {
	uint32_t num_use_attribute_registers;
};
#define DRM_PSB_DEVINFO         0x01

struct drm_psb_sizes_arg {
	uint32_t ta_mem_size;
	uint32_t mmu_size;
	uint32_t pds_size;
	uint32_t rastgeom_size;
	uint32_t tt_size;
	uint32_t vram_size;
};

/*
 * Public memory types.
 */

#define DRM_PSB_MEM_MMU TTM_PL_PRIV1
#define DRM_PSB_FLAG_MEM_MMU TTM_PL_FLAG_PRIV1

#define TTM_PL_CI               TTM_PL_PRIV0
#define TTM_PL_FLAG_CI          TTM_PL_FLAG_PRIV0

#define TTM_PL_RAR               TTM_PL_PRIV2
#define TTM_PL_FLAG_RAR          TTM_PL_FLAG_PRIV2

#define DRM_PSB_MEM_MMU_TILING TTM_PL_PRIV3
#define DRM_PSB_FLAG_MEM_MMU_TILING TTM_PL_FLAG_PRIV3

#define MAX_SLICES_PER_PICTURE 72
struct  psb_msvdx_mb_region {
	uint32_t start;
	uint32_t end;
};

typedef struct drm_psb_msvdx_decode_status {
	uint32_t num_region;
	struct psb_msvdx_mb_region mb_regions[MAX_SLICES_PER_PICTURE];
} drm_psb_msvdx_decode_status_t;

/*Status of the command sent to the gfx device.*/
enum drm_cmd_status {
	DRM_CMD_SUCCESS,
	DRM_CMD_FAILED,
	DRM_CMD_HANG
};

struct drm_psb_getpageaddrs_arg {
	uint64_t handle;
	uint64_t page_addrs;
	uint64_t gtt_offset;
};
#endif				/* _IVXD_DRM_H_ */
