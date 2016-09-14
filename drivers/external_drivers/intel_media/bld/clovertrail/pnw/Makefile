DRIVER_NAME := pnwdisp

MEDIADIR=../../..
GFXDIR=$(MEDIADIR)/graphics/sgx
DISPLAYDIR=$(MEDIADIR)/display/pnw/drv
OTM_HDMI_SRC = $(MEDIADIR)/otm_hdmi
FBDEVDIR=$(MEDIADIR)/graphics/sgx/services4/3rdparty/linux_framebuffer_drm
VIDEO_COMMON_DIR=$(MEDIADIR)/video/common
DECODE_DIR=$(MEDIADIR)/video/decode
ENCODE_DIR=$(MEDIADIR)/video/encode
MEDIA_COMMON_DIR=$(MEDIADIR)/common

MEDIAINC=drivers/external_drivers/intel_media
GFXINC=$(MEDIAINC)/graphics/sgx
DISPLAYINC=$(MEDIAINC)/display/pnw/drv
OTM_HDMI_INC=$(MEDIAINC)/otm_hdmi

# Get definitions for PVRINCS-y and PVRDEFS-y
include drivers/external_drivers/intel_media/bld/clovertrail/pvrdefs.mk

include_dirs := \
	$(PVRINCS-y) \
	-I$(MEDIAINC) \
	-I$(DISPLAYINC) \
	-I$(MEDIAINC)/otm_hdmi/os/android/include \
	-I$(MEDIAINC)/otm_hdmi/pil/include \
	-I$(MEDIAINC)/video/decode \
	-I$(MEDIAINC)/video/encode \
	-I$(MEDIAINC)/video/common \
	-I$(MEDIAINC)/common \
	-I$(MEDIAINC)/interface \
	-I$(OTM_HDMI_INC)/os/android/include \
	-I$(OTM_HDMI_INC)/pil/include \
	-I$(OTM_HDMI_INC)/pil/common \
	-I$(OTM_HDMI_INC)/pil/specific/include \
	-I$(OTM_HDMI_INC)/pil/specific/ctp \
	-I$(OTM_HDMI_INC)/ipil/include \
	-I$(OTM_HDMI_INC)/ipil/common \
	-I$(OTM_HDMI_INC)/ipil/specific/include \
	-I$(OTM_HDMI_INC)/ipil/specific/mfld \
	-Iinclude/linux \
	-Iinclude/drm

ifeq ($(DISPLAY_DRIVER_DEBUG_INTERFACE), true)
ccflags-y += -DDISPLAY_DRIVER_DEBUG_INTERFACE
endif

#only for internal testing
ccflags-y += -DOTM_HDMI_UNIT_TEST

#enable HDCP
ccflags-y += -DOTM_HDMI_HDCP_ENABLE
ccflags-y += -DCONFIG_VIDEO_MRFLD_EC
ccflags-y += -DCONFIG_SLICE_HEADER_PARSING

ccflags-y += $(include_dirs)

ccflags-y += $(PVRDEFS-y)

$(DRIVER_NAME)-objs += $(FBDEVDIR)/drmlfb_displayclass.o \
	$(FBDEVDIR)/drmlfb_linux.o \
	$(MEDIA_COMMON_DIR)/psb_drv.o \
	$(MEDIA_COMMON_DIR)/psb_powermgmt.o \
	$(MEDIA_COMMON_DIR)/psb_irq.o \
	$(MEDIA_COMMON_DIR)/psb_gtt.o \
	$(DISPLAYDIR)/psb_bl.o \
	$(DISPLAYDIR)/psb_fb.o \
	$(DISPLAYDIR)/psb_intel_display.o \
	$(DISPLAYDIR)/mdfld_csc.o \
	$(DISPLAYDIR)/psb_pvr_glue.o \
	$(DISPLAYDIR)/mdfld_dsi_dbi.o \
	$(DISPLAYDIR)/mdfld_dsi_output.o \
	$(DISPLAYDIR)/mdfld_output.o \
	$(DISPLAYDIR)/auo_sc1_vid.o \
	$(DISPLAYDIR)/auo_sc1_cmd.o \
	$(DISPLAYDIR)/gi_sony_vid.o \
	$(DISPLAYDIR)/gi_sony_cmd.o \
	$(DISPLAYDIR)/h8c7_vid.o \
	$(DISPLAYDIR)/mdfld_dsi_pkg_sender.o \
	$(DISPLAYDIR)/mdfld_dsi_esd.o \
	$(DISPLAYDIR)/mdfld_dsi_dbi_dsr.o \
	$(DISPLAYDIR)/tc35876x_vid.o \
	$(DISPLAYDIR)/gi_renesas_cmd.o \
	$(DISPLAYDIR)/mdfld_hdmi_audio.o \


ifeq ($(CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/mdfld_dsi_dpi_pf450.o
else ifeq ($(CONFIG_A500CG), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/mdfld_dsi_dpi.a500cg.o
else
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/mdfld_dsi_dpi.o
endif

ifeq ($(CONFIG_MDFD_VIDEO_DECODE), y)
$(DRIVER_NAME)-objs += $(ENCODE_DIR)/pnw_topaz.o \
	$(ENCODE_DIR)/pnw_topazinit.o \
	$(VIDEO_COMMON_DIR)/psb_cmdbuf.o \
	$(VIDEO_COMMON_DIR)/psb_buffer.o \
	$(VIDEO_COMMON_DIR)/psb_fence.o \
	$(VIDEO_COMMON_DIR)/psb_mmu.o \
	$(DECODE_DIR)/psb_msvdx.o \
	$(DECODE_DIR)/psb_msvdx_ec.o \
	$(DECODE_DIR)/psb_msvdxinit.o \
	$(DECODE_DIR)/psb_msvdx_fw.o \
	$(VIDEO_COMMON_DIR)/psb_ttm_glue.o \
	$(VIDEO_COMMON_DIR)/psb_ttm_fence.o \
	$(VIDEO_COMMON_DIR)/psb_ttm_fence_user.o \
	$(VIDEO_COMMON_DIR)/psb_ttm_placement_user.o
endif

ifeq ($(CONFIG_SUPPORT_HDMI), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/psb_umevents.o \
	$(DISPLAYDIR)/psb_socket.o \
	$(OTM_HDMI_SRC)/pil/common/otm_hdmi.o \
	$(OTM_HDMI_SRC)/pil/common/mode_info.o \
	$(OTM_HDMI_SRC)/pil/common/hdcp.o \
	$(OTM_HDMI_SRC)/pil/common/edid.o \
	$(OTM_HDMI_SRC)/pil/common/edid_print.o \
	$(OTM_HDMI_SRC)/pil/common/infoframes.o \
	$(OTM_HDMI_SRC)/ipil/common/otm_ipil_main.o \
	$(OTM_HDMI_SRC)/ipil/common/ipil_hdcp.o \
	$(OTM_HDMI_SRC)/os/android/android_hdmi.o \
	$(OTM_HDMI_SRC)/ipil/specific/mfld/ips_hdmi.o \
	$(OTM_HDMI_SRC)/ipil/specific/mfld/ips_hdcp.o \
	$(OTM_HDMI_SRC)/ipil/specific/ctp/ips_hdmi_priv.o \
	$(OTM_HDMI_SRC)/pil/specific/ctp/ps_hdmi.o
endif

ifeq ($(CONFIG_SUPPORT_YB_MIPI_DISPLAY), y)
	$(DRIVER_NAME)-objs += $(DISPLAYDIR)/yb_cmi_vid.o
endif

ifeq ($(CONFIG_CTP_DPST), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/psb_dpst.o \
	$(DISPLAYDIR)/psb_dpst_func.o \
	$(DISPLAYDIR)/dispmgrnl.o
endif

ifeq ($(CONFIG_SUPPORT_OTM8018B_MIPI_480X854_DISPLAY), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/pf450cl_vid.o
endif
ifeq ($(CONFIG_SUPPORT_TMD_MIPI_600X1024_DISPLAY), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/tmd_6x10_vid.o
endif

ifeq ($(CONFIG_SUPPORT_MIPI_H8C7_CMD_DISPLAY), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/h8c7_cmd.o
endif

ifeq ($(CONFIG_SUPPORT_MIPI_HX8394_DISPLAY), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/hx8394_vid.o
endif

ifeq ($(CONFIG_SUPPORT_MIPI_ORISE1283A_DISPLAY), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/orise1283a_vid.o
endif

ifeq ($(CONFIG_SUPPORT_VB_MIPI_DISPLAY), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/vb_cmd.o
endif

ifeq ($(CONFIG_R63311_MIPI_VIDEO_MODE), y)
$(DRIVER_NAME)-objs += $(DISPLAYDIR)/r63311_vid.o
endif

obj-m += $(DRIVER_NAME).o

