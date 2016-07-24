ifeq ($(DO_ANDROID),)
DO_ANDROID := "1"
endif
ifeq ($(DO_SOCPERF),)
    DO_SOCPERF := "1"
endif
ifeq ($(DO_PROFILING),)
    DO_PROFILING := "0"
endif
ifeq ($(DO_DEBUG_BUILD),)
    DO_DEBUG_BUILD := "1"
endif
# Modified by GU for 1A src tree inclusion
INCDIR_1=$(M)/../include
INCDIR_2=$(M)/inc

EXTRA_CFLAGS:=-I$(INCDIR_1)
EXTRA_CFLAGS += -I$(INCDIR_2)

EXTRA_CFLAGS += -DDO_ANDROID=$(DO_ANDROID)

EXTRA_CFLAGS += -DDO_SOCPERF=$(DO_SOCPERF)

EXTRA_CFLAGS += -DDO_DRIVER_PROFILING=$(DO_PROFILING)

EXTRA_CFLAGS += -Werror=strict-prototypes
EXTRA_CFLAGS += -Werror=pointer-to-int-cast
EXTRA_CFLAGS += -Werror=int-to-pointer-cast
EXTRA_CFLAGS += -Werror=format
EXTRA_CFLAGS += -Werror=attributes
ifeq ($(DO_DEBUG_BUILD),1)
    EXTRA_CFLAGS += -Werror
endif

# Modified by GU for 1A src tree inclusion
#KBUILD_EXTRA_SYMBOLS:=$(MODULE_SYMVERS_FILE)

obj-m := socwatch2_1.o

socwatch2_1-objs := src/sw_driver.o \
		    src/sw_hardware_io.o \
		    src/sw_output_buffer.o \
		    src/sw_tracepoint_handlers.o \
		    src/sw_mem.o \
		    src/sw_collector.o \
		    src/sw_telem.o \

.PHONY: kernel_check

kernel_check:
ifeq "$(KERNEL_SRC_DIR)" ""
	@echo "Error: makefile MUST NOT be invoked directly! Use the \"build_driver\" script instead."
	@exit 255
endif

default: kernel_check
	@echo "************************************************************"
	@echo "KERNEL_SRC_DIR=$(KERNEL_SRC_DIR)"
	@echo "APWR_RED_HAT=$(APWR_RED_HAT)"
	@echo "DO_WAKELOCK_SAMPLE=$(WAKELOCK_SAMPLE)"
	@echo "DO_ANDROID=$(DO_ANDROID)"
	@echo "DO_SOCPERF=$(DO_SOCPERF)"
	@echo "MODULE_SYMVERS_FILE=$(MODULE_SYMVERS_FILE)"
	@echo "DO_DRIVER_PROFILING=$(DO_PROFILING)"
	@echo "************************************************************"
	make -C $(KERNEL_SRC_DIR) M=$(PWD) modules PWD=$(PWD)

clean: kernel_check
	make -C $(KERNEL_SRC_DIR) M=$(PWD) clean
