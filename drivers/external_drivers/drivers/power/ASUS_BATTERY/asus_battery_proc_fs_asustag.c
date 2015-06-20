/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#include <linux/proc_fs.h>
#include <linux/uaccess.h>

#include "asus_battery.h"

#define PROCFS_BATTERY 		"battery_status"

extern struct mutex batt_info_mutex;
extern struct battery_info_reply batt_info;

int asus_battery_info_proc_write(struct file *file, const char *buffer, unsigned long count, void *data)
{
        return count;
}

char str_batt_status[5][40] = {"POWER_SUPPLY_STATUS_UNKNOWN",
                               "POWER_SUPPLY_STATUS_CHARGING",
                               "POWER_SUPPLY_STATUS_DISCHARGING",
                               "POWER_SUPPLY_STATUS_NOT_CHARGING",
                               "POWER_SUPPLY_STATUS_FULL"
                              };

char str_batt_level[6][40] = {"POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN",
                              "POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL",
                              "POWER_SUPPLY_CAPACITY_LEVEL_LOW",
                              "POWER_SUPPLY_CAPACITY_LEVEL_NORMAL",
                              "POWER_SUPPLY_CAPACITY_LEVEL_HIGH",
                              "POWER_SUPPLY_CAPACITY_LEVEL_FULL"
                             };

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

int asus_battery_info_proc_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
        int len=0;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        len = sprintf(page,
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

        return len;
}

int asus_battery_profile_read(char *page, char **start, off_t off, int count, int *eof, void *date)
{
        int ret;
        unsigned int current_percent = 0;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        current_percent = tmp_batt_info.percentage;
        ret = sprintf(page, "%d\n", current_percent);

        return ret;
}

int asus_battery_register_proc_fs_test(void)
{
        struct proc_dir_entry *entry=NULL;
        struct battery_info_reply tmp_batt_info;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

#if 0
        if (tmp_batt_info.test_flag & TEST_INFO_PROC_DUMP) {
#endif
                //TODO: Need porting <Wade+>
                //entry = create_proc_entry("asus_battery_info", 0666, NULL);
                //if (!entry) {
                //        BAT_DBG_E("Unable to create asus_battery_info\n");
                //        return -EINVAL;
                //}
                //entry->read_proc = asus_battery_info_proc_read;
                //entry->write_proc = asus_battery_info_proc_write;
#if 0
        }
#endif

        //TODO: Need porting <Wade+>
        //entry = create_proc_entry(PROCFS_BATTERY, 0666, NULL);
        //if (!entry) {
        //        BAT_DBG_E("Unable to create %s\n", PROCFS_BATTERY);
        //        return -EINVAL;
        //}
        //entry->read_proc = asus_battery_profile_read;

        return 0;
}
