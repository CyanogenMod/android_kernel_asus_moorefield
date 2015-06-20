#
# Makefile for Intel MID GPS devices
#
obj-$(CONFIG_INTEL_MID_GPS)	+= intel_mid_gps.o

ifeq ($(UART_DEBUG), yes)
  KBUILD_CFLAGS += -DUART_DEBUG
endif
