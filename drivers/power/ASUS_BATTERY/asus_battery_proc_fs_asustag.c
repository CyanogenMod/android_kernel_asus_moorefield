/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/slab.h>

#include "asus_battery.h"

#define PROCFS_BATTERY 		"battery_status"

extern struct mutex batt_info_mutex;
extern struct battery_info_reply batt_info; 

ssize_t asus_battery_info_proc_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
        return count;
}

char str_batt_status[6][40] = {"POWER_SUPPLY_STATUS_UNKNOWN",
                                "POWER_SUPPLY_STATUS_CHARGING", 
                                "POWER_SUPPLY_STATUS_DISCHARGING", 
                                "POWER_SUPPLY_STATUS_NOT_CHARGING", 
                                "POWER_SUPPLY_STATUS_FULL",
                                "POWER_SUPPLY_STATUS_QUICK_CHARGING",
				"POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING"};

char str_batt_level[6][40] = {"POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN",
                                "POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL",
                                "POWER_SUPPLY_CAPACITY_LEVEL_LOW",
                                "POWER_SUPPLY_CAPACITY_LEVEL_NORMAL",
                                "POWER_SUPPLY_CAPACITY_LEVEL_HIGH",
                                "POWER_SUPPLY_CAPACITY_LEVEL_FULL"};

char str_batt_cable_sts[][20] = {
        "NO_CABLE", 
        "USB_ADAPTER", 
        "USB_PC"
};

char str_batt_drv_sts[][30] = {
        "DRV_NOT_READY", 
        "DRV_INIT", 
        "DRV_INIT_OK", 
        "DRV_REGISTER",
        "DRV_REGISTER_OK",
        "DRV_BATTERY_LOW",
        "DRV_SHUTDOWN",
};

ssize_t asus_battery_info_proc_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len=0;
	ssize_t ret = 0;
	char *buff;
	struct battery_info_reply tmp_batt_info;

	buff = kmalloc(1000,GFP_KERNEL);
  	if(!buff)
		return -ENOMEM;
  
        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);
	if(tmp_batt_info.status==9)
		tmp_batt_info.status = 5;
	else if(tmp_batt_info.status==10)
		tmp_batt_info.status = 6;
        len += sprintf(buff+len, 
                        "battery_status=%s\n"
                        "battery_voltage=%d\n"
                        "battery_current=%d\n"
                        "battery_available_energy=%d\n"
                        "percentage=%d%%\n"
                        "level=%s\n"
                        "battery_temperature=%d\n"
                        "polling_time=%d\n"
                        "critical_polling_time=%d\n"
                        "cable_status=%s\n"
                        "driver_status=%s\n",
                        str_batt_status[tmp_batt_info.status],
                        tmp_batt_info.batt_volt,
                        tmp_batt_info.batt_current,
                        tmp_batt_info.batt_energy,
                        tmp_batt_info.percentage,
                        str_batt_level[tmp_batt_info.level],
                        tmp_batt_info.batt_temp,
                        tmp_batt_info.polling_time,
                        tmp_batt_info.critical_polling_time,
                        str_batt_cable_sts[tmp_batt_info.cable_status],
                        str_batt_drv_sts[tmp_batt_info.drv_status]
                                );

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
  	kfree(buff);
  	return ret;

}

ssize_t asus_battery_profile_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len=0;
	unsigned int current_percent = 0;
	ssize_t ret = 0;
	char *buff;
	struct battery_info_reply tmp_batt_info;

	buff = kmalloc(100,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

	current_percent = tmp_batt_info.percentage;
	len += sprintf(buff+len, "%d\n", current_percent);

	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
	kfree(buff);
	return ret;

}

int asus_battery_register_proc_fs_test(void)
{
	struct battery_info_reply tmp_batt_info;
	struct proc_dir_entry *entry=NULL;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

	static struct file_operations asus_battery_info_fop = {
		.read = asus_battery_info_proc_read,
		//.write = asus_battery_info_proc_write,
	};
	entry = proc_create("asus_battery_info", 0666,NULL, &asus_battery_info_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/asus_battery_info fail\n");
	}

	static struct file_operations asus_battery_profile_fop = {
	    	.read = asus_battery_profile_read,
	};
	entry = proc_create(PROCFS_BATTERY, 0666,NULL, &asus_battery_profile_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/%s fail\n", PROCFS_BATTERY);
	}

        return 0;
}
