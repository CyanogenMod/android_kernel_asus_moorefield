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
 *
 *	Author:	Jheng-Siou, Cai
 *	Time:	2015-05
 *
 */

#ifndef __LINUX_LASER_FORCUS_SENSOR_SYSFS_H
#define __LINUX_LASER_FORCUS_SENSOR_SYSFS_H

/* Path of offset file */
/* factory image */
#define LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FACTORY_FILE		"/factory/LaserFocus_Calibration10.txt"
/* Path of cross talk file */
#define LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FACTORY_FILE		"/factory/LaserFocus_Calibration40.txt"
/* Path of Laura calibration data file */
#define LAURA_CALIBRATION_FACTORY_FILE		"/factory/laura_cal_data.txt"
/* shipping image */
#define LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE		"/mnt/sdcard/LaserFocus_Calibration10.txt"
/* Path of cross talk file */
#define LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE		"/mnt/sdcard/LaserFocus_Calibration40.txt"
/* Path of Laura calibration data file */
#define LAURA_CALIBRATION_FILE		"/mnt/sdcard/laura_cal_data.txt"


/* Read one integer from file */
int Sysfs_read_int(char *filename, size_t size);
/* Read many words(two bytes one time) from file */
int Sysfs_read_word_seq(char *filename, int *value, int size);
/* write one integer to file */
bool Sysfs_write_int(char *filename, int value);
/* Write many words(two bytes one time) to file */
bool Sysfs_write_word_seq(char *filename, uint16_t *value, uint32_t size);
/* Read offset calibration value */
#ifdef kuncomment
int Laser_Forcus_sysfs_read_offset(void);
#endif
/* Write offset calibration value to file */
#ifdef kuncomment
bool Laser_Forcus_sysfs_write_offset(int calvalue);
#endif
/* Read cross talk calibration value */
#ifdef kuncomment
int Laser_Forcus_sysfs_read_cross_talk_offset(void);
#endif
/* Write cross talk calibration value to file */
#ifdef kuncomment
bool Laser_Forcus_sysfs_write_cross_talk_offset(int calvalue);
#endif
#endif

