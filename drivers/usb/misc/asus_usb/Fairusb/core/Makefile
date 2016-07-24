#
# Makefile for the usb FUSB302 driver (if driver is inside kernel tree).
#

ccflags-$(CONFIG_USB_FUSB302) := -DFSC_PLATFORM_LINUX -DFSC_HAVE_SNK -DFSC_HAVE_SRC -DFSC_HAVE_DRP \
-DFSC_HAVE_ACCMODE -DFSC_HAVE_VDM -DFSC_HAVE_DP -DFSC_HAVE_DEBUG
ccflags-$(CONFIG_USB_FUSB302) += -DFSC_INTERRUPT_TRIGGERED

obj-$(CONFIG_USB_FUSB302) += fusb_core.o

fusb_core-y := AlternateModes.o core.o fusb30X.o Log.o PDPolicy.o PDProtocol.o TypeC.o

obj-$(CONFIG_USB_FUSB302)		+= vdm/
