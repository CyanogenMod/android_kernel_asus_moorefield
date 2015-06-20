DRIVER_NAME := pvrsgx

MEDIADIR=../../..
GFXDIR=$(MEDIADIR)/graphics/sgx
DISPLAYDIR=$(MEDIADIR)/display/pnw/drv
GPU_BURST_DIR=$(MEDIADIR)/graphics/gburst

ENVDIR=$(GFXDIR)/services4/srvkm/env/linux
COMMONDIR=$(GFXDIR)/services4/srvkm/common
BRIDGEDDIR=$(GFXDIR)/services4/srvkm/bridged
SGXDIR=$(GFXDIR)/services4/srvkm/devices/sgx
SYSCONFIGDIR=$(GFXDIR)/services4/system/intel_drm

MEDIAINC=drivers/external_drivers/intel_media
GFXINC=$(MEDIAINC)/graphics/sgx
DISPLAYINC=$(MEDIAINC)/display/pnw/drv

# Get definitions for PVRINCS-y and PVRDEFS-y
include drivers/external_drivers/intel_media/bld/clovertrail/pvrdefs.mk

include_dirs := \
	$(PVRINCS-y) \
	-I$(DISPLAYINC) \
	-I$(MEDIAINC)/otm_hdmi/os/android/include \
	-I$(MEDIAINC)/otm_hdmi/pil/include \
	-I$(MEDIAINC)/video/decode \
	-I$(MEDIAINC)/video/encode \
	-I$(MEDIAINC)/video/common \
	-I$(MEDIAINC)/common \
	-I$(MEDIAINC)/interface \
	-Iinclude/linux \
	-Iinclude/drm \
	-Idrivers/staging/android

ifeq ($(DISPLAY_DRIVER_DEBUG_INTERFACE), true)
ccflags-y += -DDISPLAY_DRIVER_DEBUG_INTERFACE
endif

ccflags-y += $(include_dirs)

ccflags-y += $(PVRDEFS-y)

ifeq ($(CONFIG_PVR_PROC), y)
$(DRIVER_NAME)-objs += $(ENVDIR)/proc.o
endif

$(DRIVER_NAME)-objs +=  \
	$(GPU_BURST_DIR)/gburstm.o \
	$(GPU_BURST_DIR)/gburst_stats.o \
	$(GPU_BURST_DIR)/utilf.o \
	$(ENVDIR)/osfunc.o \
	$(ENVDIR)/mutils.o \
	$(ENVDIR)/mmap.o \
	$(ENVDIR)/module.o \
	$(ENVDIR)/pdump.o \
	$(ENVDIR)/pvr_bridge_k.o \
	$(ENVDIR)/pvr_debug.o \
	$(ENVDIR)/mm.o \
	$(ENVDIR)/mutex.o \
	$(ENVDIR)/event.o \
	$(ENVDIR)/osperproc.o \
	$(ENVDIR)/pvr_drm.o \
	$(ENVDIR)/pvr_sync.o \
	$(COMMONDIR)/buffer_manager.o \
	$(COMMONDIR)/devicemem.o \
	$(COMMONDIR)/deviceclass.o \
	$(COMMONDIR)/handle.o \
	$(COMMONDIR)/hash.o \
	$(COMMONDIR)/lists.o \
	$(COMMONDIR)/mem.o \
	$(COMMONDIR)/mem_debug.o \
	$(COMMONDIR)/metrics.o \
	$(COMMONDIR)/osfunc_common.o \
	$(COMMONDIR)/pdump_common.o \
	$(COMMONDIR)/perproc.o \
	$(COMMONDIR)/power.o \
	$(COMMONDIR)/pvrsrv.o \
	$(COMMONDIR)/queue.o \
	$(COMMONDIR)/ra.o \
	$(COMMONDIR)/refcount.o \
	$(COMMONDIR)/resman.o \
	$(BRIDGEDDIR)/bridged_support.o \
	$(BRIDGEDDIR)/bridged_pvr_bridge.o \
	$(BRIDGEDDIR)/sgx/bridged_sgx_bridge.o \
	$(SYSCONFIGDIR)/gburst_interface.o \
	$(SYSCONFIGDIR)/sysconfig.o \
	$(SYSCONFIGDIR)/sysutils.o \
	$(SYSCONFIGDIR)/sys_pvr_drm_export.o \
	$(SGXDIR)/gburst_hw.o \
	$(SGXDIR)/sgxinit.o \
	$(SGXDIR)/sgxpower.o \
	$(SGXDIR)/sgxreset.o \
	$(SGXDIR)/sgxutils.o \
	$(SGXDIR)/sgxkick.o \
	$(SGXDIR)/sgxtransfer.o \
	$(SGXDIR)/mmu.o \
	$(SGXDIR)/pb.o

obj-m += $(DRIVER_NAME).o
