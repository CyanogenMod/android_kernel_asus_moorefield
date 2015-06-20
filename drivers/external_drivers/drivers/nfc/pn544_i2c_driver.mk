# This makefile is included from vendor/intel/*/AndroidBoard.mk.
$(eval $(call build_kernel_module,$(call my-dir),pn544, CONFIG_PN544_NFC_FORK=m))

