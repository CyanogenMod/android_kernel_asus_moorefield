#
# Makefile for the input core drivers.
#

# Each configuration option enables a list of files.

obj-$(CONFIG_INPUT)		+= input-core.o
input-core-y := input.o input-compat.o input-mt.o ff-core.o
 
ifeq ($(TARGET_BUILD_VARIANT), userdebug)
KBUILD_CFLAGS += -DCONFIG_ASUS_SENSOR_ENG_CMD
endif

ifeq ($(TARGET_BUILD_VARIANT), eng)
KBUILD_CFLAGS += -DCONFIG_ASUS_SENSOR_ENG_CMD
endif

obj-$(CONFIG_INPUT_FF_MEMLESS)	+= ff-memless.o
obj-$(CONFIG_INPUT_POLLDEV)	+= input-polldev.o
obj-$(CONFIG_INPUT_SPARSEKMAP)	+= sparse-keymap.o
obj-$(CONFIG_INPUT_MATRIXKMAP)	+= matrix-keymap.o

obj-$(CONFIG_INPUT_MOUSEDEV)	+= mousedev.o
obj-$(CONFIG_INPUT_JOYDEV)	+= joydev.o
obj-$(CONFIG_INPUT_EVDEV)	+= evdev.o
obj-$(CONFIG_INPUT_EVBUG)	+= evbug.o

obj-$(CONFIG_INPUT_KEYBOARD)	+= keyboard/
obj-$(CONFIG_INPUT_MOUSE)	+= mouse/
obj-$(CONFIG_INPUT_JOYSTICK)	+= joystick/
obj-$(CONFIG_INPUT_TABLET)	+= tablet/
obj-$(CONFIG_INPUT_TOUCHSCREEN)	+= touchscreen/
obj-$(CONFIG_INPUT_MISC)	+= misc/
obj-$(CONFIG_HALL_SENSOR)       += hall/
obj-$(CONFIG_SENSOR_SYNAPTICS_FP)  += syna_fp/
obj-$(CONFIG_SENSOR_ELANTECH_FP)   += elan_fp/
obj-$(CONFIG_SENSORS_CM36686)      += cm36686/
obj-$(CONFIG_SENSOR_AP3045)      += ap3045/

obj-$(CONFIG_INPUT_APMPOWER)	+= apm-power.o
obj-$(CONFIG_INPUT_KEYRESET)	+= keyreset.o
obj-$(CONFIG_INPUT_KEYCOMBO)	+= keycombo.o
obj-$(CONFIG_MR_SENSOR)		+= mr/
