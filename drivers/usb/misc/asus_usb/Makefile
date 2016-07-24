#
# Makefile for the usb type-c Anax/FUSB302 driver (if driver is inside kernel tree).
#

ccflags-$(CONFIG_USB_FUSB302) := -DPLATFORM_LINUX
ccflags-$(CONFIG_USB_FUSB302) += -DFSC_INTERRUPT_TRIGGERED

obj-$(CONFIG_USB_FUSB302)		+= Anxusb/
obj-$(CONFIG_USB_FUSB302)		+= Fairusb/