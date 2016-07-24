
MODULES_DIR =
BUFFER_DIR = $(MODULES_DIR)buffer
DEVPROXY_DIR = $(MODULES_DIR)devproxy
EVENT_QUEUE_DIR = $(MODULES_DIR)eq
EQ_SPACE_CHECKER_DIR = $(MODULES_DIR)eqc
VIED_NCI_ACC_DIR = $(MODULES_DIR)vied_nci_acc
IPFD_DIR = $(MODULES_DIR)vied_nci_input_feeder
IPFD_DMA_DIR = $(IPFD_DIR)/src/dma_utils
PSYSAPI_DIR = $(MODULES_DIR)psysapi
TPROXY_DIR = $(MODULES_DIR)tproxy
VIED_NCI_S2V_DIR = $(MODULES_DIR)vied_nci_s2v
SYSCOM_DIR = $(MODULES_DIR)/syscom
BXT_PSS_DIR = $(MODULES_DIR)/bxt_pss
TEST_INFRA_DIR = $(MODULES_DIR)/test_infra



EVENT_QUEUE_HOST_FILES = $(EVENT_QUEUE_DIR)/src/vied_nci_eq.o
EVENT_QUEUE_HOST_FILES += $(EVENT_QUEUE_DIR)/src/device_access/vied_nci_eq_host_reg_access.o
DEVPROXY_HOST_FILES = $(DEVPROXY_DIR)/src/devproxy.host.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_ctrl.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_test_service_comm.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_service_ipf_comm.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_service_wba_comm.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_service_anr_comm.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_service_dm_comm.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_service_ccm_comm.o
DEVPROXY_HOST_FILES += $(DEVPROXY_DIR)/src/devproxy_service_gtc_comm.o
BUFFER_HOST_FILES = $(BUFFER_DIR)/src/cpu/buffer_access.o
BUFFER_HOST_FILES += $(BUFFER_DIR)/src/cpu/ia_css_buffer.o
BUFFER_HOST_FILES += $(BUFFER_DIR)/src/cpu/ia_css_input_buffer.o
BUFFER_HOST_FILES += $(BUFFER_DIR)/src/cpu/ia_css_output_buffer.o
BUFFER_HOST_FILES += $(BUFFER_DIR)/src/cpu/ia_css_private_buffer.o
BUFFER_HOST_FILES += $(BUFFER_DIR)/src/cpu/ia_css_shared_buffer.o
TEST_INFRA_HOST_FILES += $(TEST_INFRA_DIR)/src/test_infra_loader.spc.o
BXT_PSS_HOST_FILES = $(BXT_PSS_DIR)/src/psys_pss_device_impl.o
BXT_PSS_HOST_FILES += $(BXT_PSS_DIR)/src/psys_pss_dummy_impl.o
BXT_PSS_HOST_FILES += $(BXT_PSS_DIR)/src/psys_pss_pg_impl.o
BXT_PSS_HOST_FILES += $(BXT_PSS_DIR)/src/psys_pss_process_group_cmd_impl.o
BXT_PSS_HOST_FILES += $(TEST_INFRA_HOST_FILES)
PSYSAPI_HOST_FILES    = $(PSYSAPI_DIR)/dynamic/src/ia_css_psys_terminal.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/dynamic/src/ia_css_psys_process.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/dynamic/src/ia_css_psys_process_group.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/static/src/ia_css_psys_parameter_manifest.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/static/src/ia_css_psys_terminal_manifest.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/static/src/ia_css_psys_program_manifest.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/static/src/ia_css_psys_program_group_manifest.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/data/src/ia_css_program_group_data.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/param/src/ia_css_program_group_param.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/kernel/src/ia_css_kernel_bitmap.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/sim/src/ia_css_psys_sim_data.o
PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/sim/src/vied_nci_psys_system.o
#PSYSAPI_HOST_FILES    += $(PSYSAPI_DIR)/sim/src/ia_css_psys_process_group_cmd_impl.o
VIED_NCI_ACC_HOST_FILES := $(VIED_NCI_ACC_DIR)/src/vied_nci_acc.o
VIED_NCI_ACC_HOST_FILES += $(VIED_NCI_ACC_DIR)/src/vied_nci_acc_host_reg_access.o
VIED_IPFD_HOST_FILES := $(IPFD_DIR)/src/vied_nci_input_feeder_dmfs.o \
		$(IPFD_DIR)/src/vied_nci_input_feeder_dmf.o \
                 $(IPFD_DIR)/src/vied_nci_input_feeder_ibuff_ctrl.o \
                 $(IPFD_DMA_DIR)/base_api.o \
                 $(IPFD_DMA_DIR)/unit_api.o \
                 $(IPFD_DMA_DIR)/span_api.o \
                 $(IPFD_DMA_DIR)/terminal_api.o \
                 $(IPFD_DMA_DIR)/channel_api.o \
                 $(IPFD_DMA_DIR)/master_api.o \
                 $(IPFD_DMA_DIR)/request_api.o \
                 $(IPFD_DMA_DIR)/global_api.o \
                 $(IPFD_DIR)/src/vied_nci_input_feeder_dma.o \
                 $(IPFD_DIR)/src/vied_nci_input_feeder.o
VIED_NCI_S2V_HOST_FILES := $(VIED_NCI_S2V_DIR)/src/vied_nci_s2v.o
VIED_NCI_S2V_HOST_FILES += $(VIED_NCI_S2V_DIR)/src/vied_nci_s2v_host_reg_access.o
HOST_IBUF_CNTRL_2600_FILES += $(MODULES_DIR)/hrt/ibuf_cntrl_2600_config.o
EQC_HOST_FILES = $(EQ_SPACE_CHECKER_DIR)/src/vied_nci_eqc.o

INCLUDES := -I$(srctree)/$(src)/buffer/interface \
		-I$(srctree)/$(src)/support \
		-I$(srctree)/$(src)/psysapi/device/interface \
		-I$(srctree)/$(src)/psysapi/data/interface \
		-I$(srctree)/$(src)/psysapi/param/interface \
		-I$(srctree)/$(src)/psysapi/param/src \
		-I$(srctree)/$(src)/psysapi/kernel/interface \
		-I$(srctree)/$(src)/psysapi/dynamic/interface \
		-I$(srctree)/$(src)/psysapi/static/interface \
		-I$(srctree)/$(src)/psysapi/sim/interface \
		-I$(srctree)/$(src)/psysapi/interface \
		-I$(srctree)/$(src)/bxt_pss/interface \
		-I$(srctree)/$(src)/devproxy/interface \
		-I$(srctree)/$(src)/src \
		-I$(srctree)/$(src)/syscom/interface \
		-I$(srctree)/$(src)/utils/system_defs/processing_system_system \
		-I$(srctree)/$(src)/utils/system_defs \
		-I$(srctree)/$(src)/eq/interface \
		-I$(srctree)/$(src)/src \
		-I$(srctree)/$(src)/src/device_access/host \
		-I$(srctree)/$(src)/vied_nci_dma/src/tools \
		-I$(srctree)/$(src)/vied_nci_dma/interface \
		-I$(srctree)/$(src)/tproxy/interface \
		-I$(srctree)/$(src)/vied_nci_input_feeder/src \
		-I$(srctree)/$(src)/vied_nci_input_feederinterface \
		-I$(srctree)/$(src)/utils/info_bits \
		-I$(srctree)/$(src)/utils/temp \
		-I$(srctree)/$(src)/utils/shared_memory \
		-I$(srctree)/$(src)/vied_nci_acc/interface \
		-I$(srctree)/$(src)/vied_nci_acc/src \
		-I$(IPFD_DIR)/interface/ \
                -I$(IPFD_DIR) \
                -I$(IPFD_DMA_DIR) \
		-I$(srctree)/$(src)/vied_nci_input_feeder/interface \
		-I$(srctree)/$(src)/eqc/interface \
                -I$(srctree)/$(src)/test/psa \
		-I$(srctree)/$(src)/hrt \
		-I$(srctree)/$(src)/hrt/interface \
		-I$(srctree)/$(src)/vied \
		-I$(srctree)/$(src)/vied_nci_s2v/interface \
		-I$(srctree)/$(src)/vied_nci_s2v/src \
		-I$(srctree)/$(src)/test_psys_4k60/bxtA0target \
		-I$(srctree)/$(src)/test_infra/interface




SRC_LIST = libcsspsys2600.o
SRC_LIST += $(EVENT_QUEUE_HOST_FILES)
SRC_LIST += $(DEVPROXY_HOST_FILES)
SRC_LIST += $(BUFFER_HOST_FILES)
SRC_LIST += $(PSYSAPI_HOST_FILES)
SRC_LIST += $(TPROXY_HOST_FILES)
SRC_LIST += $(HOST_IBUF_CNTRL_2600_FILES)
SRC_LIST += $(VIED_NCI_S2V_HOST_FILES)
SRC_LIST += $(EQC_HOST_FILES)
SRC_LIST += $(VIED_NCI_ACC_HOST_FILES)
SRC_LIST += $(VIED_IPFD_HOST_FILES)
SRC_LIST += $(BXT_PSS_HOST_FILES)
lib2600psys-mod-objs := $(SRC_LIST)
obj-$(CONFIG_VIDEO_CSS2600)	+= lib2600psys-mod.o
DEFINES :=  -DPSYS
DEFINES +=  -DHRT_ON_VIED_SUBSYSTEM_ACCESS=0
DEFINES +=  -DCFG_VIED_SUBSYSTEM_ACCESS_LIB_IMPL
DEFINES +=  -DHRT_USE_VIR_ADDRS
DEFINES +=  -DHRT_HW
DEFINES +=  -DHRT_KERNEL
DEFINES +=  -D__HOST__ -D__KERNEL__



PROGRAMS    = devproxy isp_main test_infra_spc_main ia_css_tproxy
SYSTEM      = processing_system_system
include $(srctree)/$(src)/gencfg.mk

#DEFINES += -DPROGNAME=$(PROGRAMS)
#DEFINES += -DPROGMAP=\"$(PROGRAMS).map.h\"
DEFINES += -DSUBSYSTEM_INCLUDE=$(SUBSYSTEM_INCLUDE_STRING)
DEFINES += -DCELL=$(CELL_STRING)

ccflags-y += $(INCLUDES) $(DEFINES) -fno-common -v
