/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang Chris1_Chang@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "asus_battery_proc_fs.h"

int asus_battery_register_proc_fs(void)
{
        int ret=0;

#if defined(CONFIG_ASUS_ENGINEER_MODE)// && CONFIG_ASUS_ENGINEER_MODE
        ret = asus_battery_register_proc_fs_test();
#endif
        return ret;
}
