#
# Copyright (C) Imagination Technologies Ltd. All rights reserved.
#
# This program is free software; you can redistribute it and/or modify it
# under the terms and conditions of the GNU General Public License,
# version 2, as published by the Free Software Foundation.
#
# This program is distributed in the hope it will be useful but, except
# as otherwise stated in writing, without any warranty; without even the
# implied warranty of merchantability or fitness for a particular purpose.
# See the GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License along with
# this program; if not, write to the Free Software Foundation, Inc.,
# 51 Franklin St - Fifth Floor, Boston, MA 02110-1301 USA.
#
# The full GNU General Public License is included in this distribution in
# the file called "COPYING".
#
# Contact Information:
# Imagination Technologies Ltd. <gpl-support@imgtec.com>
# Home Park Estate, Kings Langley, Herts, WD4 8LZ, UK
#
#

MRST_SOURCE_DIR := drivers/staging/mrst

ccflags-y += \
 -Iinclude/drm \
 -I$(TOP)/services4/include/env/linux \
 -I$(TOP)/services4/3rdparty/linux_framebuffer_drm \
 -DPVR_MRST_FB_SET_PAR_ON_INIT \
 -DCONFIG_DRM_INTEL_MID \
 -DCONFIG_DRM_MDFLD \
 -DCONFIG_MDFLD_DSI_DBI \
 -DCONFIG_MDFD_COMMAND_MODE_2

ifeq ($(MRST_DRIVER_SOURCE),)
ccflags-y += \
 -I$(OUT)/target/kbuild/external/$(MRST_SOURCE_DIR) \
 -I$(OUT)/target/kbuild/external/$(MRST_SOURCE_DIR)/drv \
 -I$(OUT)/target/kbuild/external/$(MRST_SOURCE_DIR)/imgv
else
ccflags-y += \
 -I$(MRST_DRIVER_SOURCE)/$(MRST_SOURCE_DIR) \
 -I$(MRST_DRIVER_SOURCE)/$(MRST_SOURCE_DIR)/drv \
 -I$(MRST_DRIVER_SOURCE)/$(MRST_SOURCE_DIR)/imgv
endif

medfield_drm-y += \
 services4/srvkm/env/linux/pvr_drm.o \
 services4/3rdparty/linux_framebuffer_drm/drmlfb_displayclass.o \
 services4/3rdparty/linux_framebuffer_drm/drmlfb_linux.o \
 services4/system/$(PVR_SYSTEM)/sys_pvr_drm_export.o

medfield_drm-y += \
 external/$(MRST_SOURCE_DIR)/drv/psb_bl.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_dpst.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_drv.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_fb.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_gtt.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_hotplug.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_bios.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_display.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_i2c.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_lvds.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_modes.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_sdvo.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_hdmi.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_hdmi_i2c.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_reset.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_schedule.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_sgx.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_socket.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_pvr_glue.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_umevents.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_dsi.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_intel_dsi_aava.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/mdfld_dsi_dbi.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/mdfld_dsi_dpi.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/mdfld_dsi_output.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_powermgmt.medfield.o \
 external/$(MRST_SOURCE_DIR)/drv/psb_irq.medfield.o

medfield_drm-y += \
 external/$(MRST_SOURCE_DIR)/imgv/lnc_topaz.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/lnc_topazinit.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/pnw_topaz.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/pnw_topazinit.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_buffer.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_fence.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_mmu.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_msvdx.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_msvdxinit.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_ttm_glue.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_ttm_fence.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_ttm_fence_user.medfield.o \
 external/$(MRST_SOURCE_DIR)/imgv/psb_ttm_placement_user.medfield.o \

medfield_drm-y += \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_agp_backend.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_memory.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_tt.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_bo.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_bo_util.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_bo_vm.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_module.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_global.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_object.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_lock.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_execbuf_util.o \
 external/$(MRST_SOURCE_DIR)/ttm/ttm_page_alloc.o

medfield_drm-y += \
 external/$(MRST_SOURCE_DIR)/drm_global.o
