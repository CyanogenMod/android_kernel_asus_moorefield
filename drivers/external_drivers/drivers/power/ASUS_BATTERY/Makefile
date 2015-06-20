#
#  Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
#  Written by chih-hsuan chang chih-hsuan_chang@asus.com
#

ifeq ($(TARGET_BUILD_VARIANT),eng)
        KBUILD_CFLAGS += -DA500CG_ENG_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),user)
        KBUILD_CFLAGS += -DA500CGUSER_BUILD
else ifeq ($(TARGET_BUILD_VARIANT),userdebug)
        KBUILD_CFLAGS += -DA500CG_USERDEBUG_BUILD
else
        KBUILD_CFLAGS += -DA500CG_OTHER_BUILD
endif

#obj-$(CONFIG_A500CG_BATTERY_SMB347)	+= battery_smb347.o
#obj-$(CONFIG_UPI_BATTERY) += ug31xx/
#obj-y += asus_battery_power.o
#obj-$(CONFIG_PROC_FS) += asus_battery_proc_fs.o
#obj-$(CONFIG_ASUS_ENGINEER_MODE) += asus_battery_proc_fs_asustag.o
