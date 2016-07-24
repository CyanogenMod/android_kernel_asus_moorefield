#
# Makefile - Intel Management Engine Interface (Intel MEI) Linux driver
# Copyright (c) 2010-2011, Intel Corporation.
#
obj-m += heci.o
heci-objs := init.o
heci-objs += hbm.o
heci-objs += interrupt.o
heci-objs += client.o
heci-objs += main.o
heci-objs += bus.o
heci-$(CONFIG_DEBUG_FS) += debugfs.o

obj-m += heci-ish.o
heci-ish-objs := hw-ish.o
heci-ish-objs += pci-ish.o

obj-m += hid-heci-ish.o
hid-heci-ish-objs := heci-hid.o
hid-heci-ish-objs += heci-ish-client.o
