DRIVER_NAME := vxd392

obj-m += $(DRIVER_NAME).o

MEDIADIR=../..
BAYTRAILDIR=$(MEDIADIR)/common/baytrail
VIDEOCOMDIR=$(MEDIADIR)/video/common
VIDEODECDIR=$(MEDIADIR)/video/decode

MEDIAINC=drivers/external_drivers/intel_media
BAYTRAILINC=$(MEDIAINC)/common/baytrail
VIDEOCOMINC=$(MEDIAINC)/video/common
VIDEODECINC=$(MEDIAINC)/video/decode

include_dirs := \
	-Iinclude/drm \
	-Iinclude/drm/ttm \
	-Idrivers/gpu/drm/i915 \
        -I$(MEDIAINC)/interface \
	-I$(BAYTRAILINC) \
	-I$(VIDEOCOMINC) \
	-I$(VIDEODECINC)

ccflags-y += $(include_dirs) -DCONFIG_VIDEO_MRFLD_EC

$(DRIVER_NAME)-objs += \
	$(BAYTRAILDIR)/vxd_drv.o \
	$(VIDEOCOMDIR)/psb_ttm_glue.o \
	$(VIDEOCOMDIR)/psb_cmdbuf.o \
	$(VIDEOCOMDIR)/psb_buffer.o \
	$(VIDEOCOMDIR)/psb_fence.o \
	$(VIDEOCOMDIR)/psb_mmu.o \
	$(VIDEOCOMDIR)/psb_ttm_fence.o \
	$(VIDEOCOMDIR)/psb_ttm_fence_user.o \
	$(VIDEOCOMDIR)/psb_ttm_placement_user.o \
	$(VIDEODECDIR)/psb_msvdx.o \
	$(VIDEODECDIR)/psb_msvdxinit.o \
	$(VIDEODECDIR)/psb_msvdx_fw.o \
	$(VIDEODECDIR)/psb_msvdx_ec.o
