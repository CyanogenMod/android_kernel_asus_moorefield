#
#  Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
#  Written by chris chang chris1_chang@asus.com
#

ifeq ($(TARGET_BUILD_VARIANT),eng)
    KBUILD_CFLAGS += -DME372CG_ENG_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),user)
    KBUILD_CFLAGS += -DME372CG_USER_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
    KBUILD_CFLAGS += -DME372CG_USERDEBUG_BUILD
else
    KBUILD_CFLAGS += -DME372CG_OTHER_BUILD
endif

ifeq ($(ASUS_PROJECT),ME372CG)
    KBUILD_CFLAGS += -DASUS_PROJECT_ME372CG_TOUCH
endif


obj-$(CONFIG_ME372CG_BATTERY_SMB345)	+= battery_smb345.o
obj-$(CONFIG_ME372CG_BATTERY_BQ27520)	+= bq27520_battery_core.o
obj-$(CONFIG_ME372CG_BATTERY_SMB345) += asus_battery_power.o
