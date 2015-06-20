#
#  Copyright (c) 2012, UPI-SEMI, Inc. All Rights Reserved.
#

EXTRA_CFLAGS    += -DBUILD_UG31XX_LINUX_KERNEL -DuG31xx_OS_ANDROID -DUG31XX_USE_DAEMON_AP_FOR_FILE_OP -DUG31XX_SHELL_ALGORITHM -DCONFIG_ASUS_ENGINEER_MODE -fno-pic

#$(obj)/uG31xx_API_Capacity.o: $(src)/uG31xx_API_Capacity.obj
#	if [ -a $< ]; then cp $< $@; fi;

ug31xx_battery-objs	+= uG31xx_API_Backup.o
ug31xx_battery-objs	+= uG31xx_API_Capacity.o
ug31xx_battery-objs	+= uG31xx_API_Measurement.o
ug31xx_battery-objs	+= uG31xx_API.o
ug31xx_battery-objs	+= uG31xx_API_Otp.o
ug31xx_battery-objs	+= uG31xx_API_System.o

ug31xx_battery-objs	+= uG31xx_API_Platform.o
ug31xx_battery-objs	+= ug31xx_gauge.o

obj-$(CONFIG_UPI_BATTERY)	+= ug31xx_battery.o

