#
# Makefile for TI's shared transport line discipline
# and its protocol drivers (BT, FM, GPS)
#
obj-$(CONFIG_TI_ST_FORK)	+= st_drv.o
st_drv-objs			:= st_core.o st_kim.o st_ll.o
obj-$(CONFIG_ST_HCI_FORK)	+= tty_hci.o
