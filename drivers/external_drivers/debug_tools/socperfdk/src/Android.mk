# This makefile is included from vendor/intel/*/AndroidBoard.mk.
$(eval $(call build_kernel_module,$(call my-dir),socperf1_2,"BOARD_HAVE_SMALL_RAM=$(BOARD_HAVE_SMALL_RAM)"))
