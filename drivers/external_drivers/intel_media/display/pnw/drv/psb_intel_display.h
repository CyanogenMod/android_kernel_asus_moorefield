/*
 * Copyright (c) 2009, Intel Corporation.
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
 */

#ifndef __INTEL_DRV_H__
#define __INTEL_DRV_H__

#include <drm/drm_crtc.h>
#include <drm/drm_crtc_helper.h>
#include <linux/gpio.h>

/* maximum connectors per crtcs in the mode set */
#define INTELFB_CONN_LIMIT 4

/* MDFLD KSEL only one of them can be set to 1 */
#ifdef CONFIG_CTP_CRYSTAL_38M4
#define KSEL_BYPASS_83_100_ENABLE 0
#define KSEL_CRYSTAL_19_ENABLED 0
#define KSEL_CRYSTAL_38_ENABLED 1
#else
#define KSEL_BYPASS_83_100_ENABLE 0
#define KSEL_CRYSTAL_19_ENABLED 1
#define KSEL_CRYSTAL_38_ENABLED 0
#endif

#define KSEL_CRYSTAL_19 1
#define KSEL_BYPASS_19 5
#define KSEL_BYPASS_25 6
#define KSEL_CRYSTAL_38 3
#define KSEL_BYPASS_83_100 7

/* these are outputs from the chip - integrated only
 * external chips are via DVO or SDVO output */
#define INTEL_OUTPUT_UNUSED 0
#define INTEL_OUTPUT_ANALOG 1
#define INTEL_OUTPUT_DVO 2
#define INTEL_OUTPUT_SDVO 3
#define INTEL_OUTPUT_LVDS 4
#define INTEL_OUTPUT_TVOUT 5
#define INTEL_OUTPUT_HDMI 6
#define INTEL_OUTPUT_MIPI 7
#define INTEL_OUTPUT_MIPI2 8

#define INTEL_DVO_CHIP_NONE 0
#define INTEL_DVO_CHIP_LVDS 1
#define INTEL_DVO_CHIP_TMDS 2
#define INTEL_DVO_CHIP_TVOUT 4

/**
 * Hold information useally put on the device driver privates here,
 * since it needs to be shared across multiple of devices drivers privates.
*/
struct psb_intel_mode_device {

	/* Abstracted memory manager operations */
	void *(*bo_from_handle) (struct drm_device *dev,
			struct drm_file *file_priv,
			unsigned int handle);
	size_t(*bo_size) (struct drm_device *dev, void *bo);
	size_t(*bo_offset) (struct drm_device *dev, void *bo);
	int (*bo_pin_for_scanout) (struct drm_device *dev, void *bo);
	int (*bo_unpin_for_scanout) (struct drm_device *dev, void *bo);
};

struct psb_intel_output {
	struct drm_connector base;

	struct drm_encoder enc;
	int type;
	struct i2c_adapter *hdmi_i2c_adapter;	/* for control functions */
	bool load_detect_temp;
	void *dev_priv;

	struct psb_intel_mode_device *mode_dev;

};

struct psb_intel_crtc_state {
	uint32_t saveDSPCNTR;
	uint32_t savePIPECONF;
	uint32_t savePIPESRC;
	uint32_t saveDPLL;
	uint32_t saveFP0;
	uint32_t saveFP1;
	uint32_t saveHTOTAL;
	uint32_t saveHBLANK;
	uint32_t saveHSYNC;
	uint32_t saveVTOTAL;
	uint32_t saveVBLANK;
	uint32_t saveVSYNC;
	uint32_t saveDSPSTRIDE;
	uint32_t saveDSPSIZE;
	uint32_t saveDSPPOS;
	uint32_t saveDSPBASE;
	uint32_t savePalette[256];
};

struct psb_intel_crtc {
	struct drm_crtc base;
	int pipe;
	int plane;
	uint32_t cursor_addr;
	u8 lut_r[256], lut_g[256], lut_b[256];
	u8 lut_adj[256];
	struct psb_intel_framebuffer *fbdev_fb;
	/* a mode_set for fbdev users on this crtc */
	struct drm_mode_set mode_set;

	/* current bo we scanout from */
	void *scanout_bo;

	/* current bo we cursor from */
	void *cursor_bo;

	struct drm_display_mode saved_mode;
	struct drm_display_mode saved_adjusted_mode;

	struct psb_intel_mode_device *mode_dev;

	/*crtc mode setting flags*/
	u32 mode_flags;

	/* crtc scaling type */
	u32 scaling_type;
};

#define to_psb_intel_crtc(x)	\
		container_of(x, struct psb_intel_crtc, base)
#define to_psb_intel_output(x)	\
		container_of(x, struct psb_intel_output, base)
#define enc_to_psb_intel_output(x)	\
		container_of(x, struct psb_intel_output, enc)
#define to_psb_intel_framebuffer(x)	\
		container_of(x, struct psb_intel_framebuffer, base)

extern void psb_intel_crtc_init(struct drm_device *dev, int pipe,
			    struct psb_intel_mode_device *mode_dev);
extern void psb_intel_crt_init(struct drm_device *dev);
extern inline
void intel_wait_for_pipe_enable_disable(struct drm_device *dev,
		int pipe, bool state);
extern void psb_intel_crtc_load_lut(struct drm_crtc *crtc);
extern void mdfld_disable_crtc(struct drm_device *dev, int pipe);
extern void psb_intel_wait_for_vblank(struct drm_device *dev);

#endif				/* __INTEL_DRV_H__ */
