# Makefile for the drm device driver. This driver provides support for the
# Direct Rendering Infrastructure (DRI) in XFree86 4.1.0 and higher.


ccflags-$(CONFIG_DRM_VXD_BYT) := -Iinclude/drm -Iinclude/drm/ttm -Idrivers/gpu/drm/i915 -Idrivers/staging/intel_media/common/baytrail -Idrivers/staging/intel_media/video/common -Idrivers/staging/intel_media/video/decode

obj-y += vxd_drv.o
