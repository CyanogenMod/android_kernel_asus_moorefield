/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang Chris1_Chang@asus.com
 */
#ifndef __ASUS_BATTERY_PROC_FS_H__
#define __ASUS_BATTERY_PROC_FS_H__

int asus_battery_register_proc_fs(void);

#if defined(CONFIG_ASUS_ENGINEER_MODE)// && CONFIG_ASUS_ENGINEER_MODE
int asus_battery_register_proc_fs_test(void);
#endif

#endif //__ASUS_BATTERY_PROC_FS_H__
