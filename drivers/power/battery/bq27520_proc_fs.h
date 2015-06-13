/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#ifndef __BQ27520_PROC_FS_H__
#define __BQ27520_PROC_FS_H__


int bq27520_register_proc_fs(void);

#if defined(CONFIG_ASUS_ENGINEER_MODE) && CONFIG_ASUS_ENGINEER_MODE
int bq27520_register_proc_fs_test(void);
#endif
ssize_t bq27520_proc_bridge_write(struct file *filp, const char __user *buff,
		unsigned long len, void *data);

#endif
