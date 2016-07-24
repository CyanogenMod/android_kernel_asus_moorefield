#
# Makefile for the usb FUSB302 driver (if driver is inside kernel tree).
#

ccflags-$(CONFIG_USB_FUSB302) := -DPLATFORM_LINUX

obj-$(CONFIG_USB_FUSB302) += Platform_Linux.o

Platform_Linux-y := fusb30x_driver.o fusb30x_global.o hostcomm.o platform_helpers.o
