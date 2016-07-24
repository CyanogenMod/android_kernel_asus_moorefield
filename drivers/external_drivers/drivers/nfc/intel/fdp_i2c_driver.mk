# This makefile is included from vendor/intel/*/AndroidBoard.mk.
$(eval $(call build_kernel_module,$(call my-dir),fdp_driver, CONFIG_FDP_NFC=m))
