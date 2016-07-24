#
# Makefile for the usb FUSB302 driver (if driver is inside kernel tree).
#

ccflags-$(CONFIG_USB_FUSB302) := -DPLATFORM_LINUX
ccflags-$(CONFIG_USB_FUSB302) += -DFSC_INTERRUPT_TRIGGERED

obj-$(CONFIG_USB_FUSB302)		+= fusb302/
#obj-$(CONFIG_USB_FUSB302)		+= Platform_Linux/
obj-$(CONFIG_USB_FUSB302)		+= core/