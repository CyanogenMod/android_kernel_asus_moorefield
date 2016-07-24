/* 
 * Copyright (C) 2015 ASUSTek Inc.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_LASER_FORCUS_SENSOR_SYSFS_H
#define __LINUX_LASER_FORCUS_SENSOR_SYSFS_H


#define LASERFOCUS_SENSOR_REGISTER_FILE_ZE550KL					"/factory/LaserFocus_Register_ZE550KL.txt"
#define LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE				"/factory/LaserFocus_Calibration10.txt"
#define LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE		"/factory/LaserFocus_Calibration40.txt"

#define SHIPPING_LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE				"/mnt/sdcard/LaserFocus_Calibration10.txt"
#define SHIPPING_LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE		"/mnt/sdcard/LaserFocus_Calibration40.txt"

#define LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE_CCI				"/persist/LaserFocus_Calibration10.txt"
#define LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE_CCI		"/persist/LaserFocus_Calibration40.txt"

#define LASERFOCUS_SENSOR_0x1e_FILE				"/data/data/LaserFocus_0x1e.txt"
#define LASERFOCUS_SENSOR_0x24_FILE				"/data/data/LaserFocus_0x24.txt"
#define LASERFOCUS_SENSOR_0x4d_FILE				"/data/data/LaserFocus_0x4d.txt"
#define LASERFOCUS_SENSOR_0x62_FILE				"/data/data/LaserFocus_0x62.txt"
#define LASERFOCUS_SENSOR_0x64_FILE				"/data/data/LaserFocus_0x64.txt"
#define LASERFOCUS_SENSOR_0x66_FILE				"/data/data/LaserFocus_0x66.txt"
#define LASERFOCUS_SENSOR_DMax_FILE				"/data/data/LaserFocus_DMax.txt"

bool Laser_Forcus_sysfs_write_0x1e(int calvalue);
bool Laser_Forcus_sysfs_write_0x24(int calvalue);
bool Laser_Forcus_sysfs_write_0x4d(int calvalue);
bool Laser_Forcus_sysfs_write_0x62(int calvalue);
bool Laser_Forcus_sysfs_write_0x64(int calvalue);
bool Laser_Forcus_sysfs_write_0x66(int calvalue);
bool Laser_Forcus_sysfs_write_DMax(int calvalue);

int Laser_Forcus_sysfs_read_offset(bool Factory_folder_file);
bool Laser_Forcus_sysfs_write_offset(int calvalue);
bool Laser_Forcus_sysfs_write_register_ze550kl(int register_name, uint16_t calvalue);
int Laser_Forcus_sysfs_read_cross_talk_offset(bool Factory_folder_file);
bool Laser_Forcus_sysfs_write_cross_talk_offset(int calvalue);

#endif

