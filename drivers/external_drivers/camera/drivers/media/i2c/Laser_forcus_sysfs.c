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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "Laser_forcus_sysfs.h"

int Laser_Forcus_sysfs_read_offset(bool Factory_folder_file)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	

	if(Factory_folder_file){
		printk("[LF][vl6180x] read calibration file from factory folder");
		fp = filp_open(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
		if (IS_ERR_OR_NULL(fp)) {
			pr_err("[LF][vl6180x] Offset Calibration file open (%s) fail\n", LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE);
			return -ENOENT;	/*No such file or directory*/
		}
	}
	else{
		printk("[LF][vl6180x] read calibration file from persist folder");
		fp = filp_open(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE_CCI, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
		if (IS_ERR_OR_NULL(fp)) {
			pr_err("[LF][vl6180x] Offset Calibration file open (%s) fail\n", LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE_CCI);
			return -ENOENT;	/*No such file or directory*/
		}
	}
	
	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		pr_err("[LF][vl6180x] Offset Calibration file strlen f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		pr_err("[LF][vl6180x] Offset Calibration file is NEGATIVE. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		printk("[LF][vl6180x] Read Offset Calibration value : %d\n", cal_val);
	}	

	return cal_val;
}

bool Laser_Forcus_sysfs_write_offset(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(SHIPPING_LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] Offset Calibration file open (%s) fail\n", SHIPPING_LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] Offset Calibration file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write Offset Calibration value : %s\n", buf);

	return true;
}

int Laser_Forcus_sysfs_read_cross_talk_offset(bool Factory_folder_file)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;	

	if(Factory_folder_file){
		printk("[LF][vl6180x] read calibration file from factory folder");
		fp = filp_open(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
		if (IS_ERR_OR_NULL(fp)) {
			pr_err("[LF][vl6180x] Offset Calibration file open (%s) fail\n", LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE);
			return -ENOENT;	/*No such file or directory*/
		}
	}
	else{
		printk("[LF][vl6180x] read calibration file from persist folder");
		fp = filp_open(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE_CCI, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
		if (IS_ERR_OR_NULL(fp)) {
			pr_err("[LF][vl6180x] Offset Calibration file open (%s) fail\n", LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE_CCI);
			return -ENOENT;	/*No such file or directory*/
		}
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 6, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		pr_err("[LF][vl6180x] Cross-talk Offset Calibration file strlen f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);	
	if(cal_val < 0) {
		pr_err("[LF][vl6180x] Cross-talk Offset Calibration file is NEGATIVE. (%d)\n", cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		printk("[LF][vl6180x] Read Cross-talk Offset Calibration value : %d\n", cal_val);
	}	

	return cal_val;
}

bool Laser_Forcus_sysfs_write_cross_talk_offset(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(SHIPPING_LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] Cross-talk Offset Calibration file open (%s) fail\n", SHIPPING_LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] Cross-talk Offset Calibration file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write Cross-talk Offset Calibration value : %s\n", buf);

	return true;
}

bool Laser_Forcus_sysfs_write_register_ze550kl(int register_name, uint16_t calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[32];

	sprintf(buf, "0x%x(0x%x)\n",register_name , calvalue);
	printk("[LF][vl6180x] write regiser: 0x%x(0x%x)",register_name , calvalue);
	fp = filp_open(LASERFOCUS_SENSOR_REGISTER_FILE_ZE550KL, O_RDWR | O_CREAT | O_APPEND, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] Register file open (%s) fail\n", LASERFOCUS_SENSOR_REGISTER_FILE_ZE550KL);
		return false;
	}
	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] Register file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write Register value : %s\n", buf);
	return true;
}


bool Laser_Forcus_sysfs_write_0x1e(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(LASERFOCUS_SENSOR_0x1e_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] 0x1e file open (%s) fail\n", LASERFOCUS_SENSOR_0x1e_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] 0x1e file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write 0x1e value : %s\n", buf);

	return true;
}


bool Laser_Forcus_sysfs_write_0x24(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(LASERFOCUS_SENSOR_0x24_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] 0x24 file open (%s) fail\n", LASERFOCUS_SENSOR_0x24_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] 0x24 file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write 0x24 value : %s\n", buf);

	return true;
}

bool Laser_Forcus_sysfs_write_0x4d(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(LASERFOCUS_SENSOR_0x4d_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] 0x4d file open (%s) fail\n", LASERFOCUS_SENSOR_0x4d_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] 0x4d file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write 0x4d value : %s\n", buf);

	return true;
}

bool Laser_Forcus_sysfs_write_0x62(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(LASERFOCUS_SENSOR_0x62_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] 0x62 file open (%s) fail\n", LASERFOCUS_SENSOR_0x62_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] 0x62 file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write 0x62 value : %s\n", buf);

	return true;
}


bool Laser_Forcus_sysfs_write_0x64(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(LASERFOCUS_SENSOR_0x64_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] 0x64 file open (%s) fail\n", LASERFOCUS_SENSOR_0x64_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] 0x64 file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write 0x64 value : %s\n", buf);

	return true;
}


bool Laser_Forcus_sysfs_write_0x66(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(LASERFOCUS_SENSOR_0x66_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] 0x66 file open (%s) fail\n", LASERFOCUS_SENSOR_0x66_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] 0x66 file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write 0x66 value : %s\n", buf);

	return true;
}

bool Laser_Forcus_sysfs_write_DMax(int calvalue)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	sprintf(buf, "%d", calvalue);

	fp = filp_open(LASERFOCUS_SENSOR_DMax_FILE, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("[LF][vl6180x] DMax file open (%s) fail\n", LASERFOCUS_SENSOR_DMax_FILE);
		return false;
	}

	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		pr_err("[LF][vl6180x] DMax file strlen: f_op=NULL or op->write=NULL\n");
		return false;
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	printk("[LF][vl6180x] write DMax value : %s\n", buf);

	return true;
}





