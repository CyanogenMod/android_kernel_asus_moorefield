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
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <asm/uaccess.h>
#include "show_sysfs.h"
#include "show_log.h"
#include "crc8.h"

/** @brief Read one integer from file
*	
*	@param filename the file to read
*	@param size the size to read
*
*/
int Sysfs_read_int(char *filename, size_t size)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];
	int cal_val = 0;
	int readlen = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		LOG_Handler(LOG_ERR, "%s: File open (%s) fail\n", __func__, filename);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	
	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, size, &pos_lsts);
		buf[readlen] = '\0';		
	} else {
		LOG_Handler(LOG_ERR, "%s: File (%s) strlen f_op=NULL or op->read=NULL\n", __func__, filename);
		return -ENXIO;	/*No such device or address*/
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* close file */
	filp_close(fp, NULL);

	sscanf(buf, "%d", &cal_val);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return cal_val;
}

/** @brief read many word(two bytes) from file
*	
*	@param filename the file to write
*	@param value the word which will store the calibration data from read file
*	@param size the size of write data
*
*/
int Sysfs_read_word_seq(char *filename, int *value, int size)
{
	int i = 0;
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[size][5];
	char crc_str[3];
	uint8_t calc_crc;
	int crc_sign;
	uint16_t data[17];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* open file */
	fp = filp_open(filename, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		LOG_Handler(LOG_ERR, "%s: File open (%s) fail\n", __func__, filename);
		return -ENOENT;	/*No such file or directory*/
	}

	/*For purpose that can use read/write system call*/
	
	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			fp->f_op->read(fp, buf[i], 5, &pos_lsts);
			buf[i][4]='\0';
			sscanf(buf[i], "%x", &value[i]);
			//LOG_Handler(LOG_DBG, "%s: 0x%s\n",__func__, buf[i]);
		}
	} else {
		LOG_Handler(LOG_ERR, "%s: File (%s) strlen f_op=NULL or op->read=NULL\n", __func__, filename);
		return -ENXIO;	/*No such device or address*/
	}
	fp->f_op->read(fp, crc_str, 2, &pos_lsts);
	crc_str[2]='\0';
	sscanf(crc_str, "%x", &crc_sign);
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);
	/* close file */
	filp_close(fp, NULL);

	for(i = 0;i < size;i++)
	{
		data[i] = value[i];
	}

	calc_crc = updcrc(data,size);
	if(calc_crc != crc_sign) {
		LOG_Handler(LOG_ERR, "%s: crc check fail,crc_sign=0x%02x,calc_crc=0x%02x\n", __func__,crc_sign,calc_crc);
		LOG_Handler(LOG_ERR, "crc data:0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x,0x%04x\n",
							data[0],data[1],data[2],data[3],data[4],data[5],data[6],data[7],data[8],
							data[9]);
		return -1;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}


/** @brief write one integer to file
*	
*	@param filename the file to write
*	@param value the integer which will be written to file
*
*/
bool Sysfs_write_int(char *filename, int value/*, umode_t mode*/)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[8];	

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	sprintf(buf, "%d", value);

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		LOG_Handler(LOG_ERR, "%s: File open (%s) fail\n", __func__, filename);
		return false;
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		fp->f_op->write(fp, buf, strlen(buf), &fp->f_pos);				
	} else {
		LOG_Handler(LOG_ERR, "%s: File (%s) strlen: f_op=NULL or op->write=NULL\n", __func__, filename);
		return false;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* Close file */
	filp_close(fp, NULL);

	LOG_Handler(LOG_DBG, "%s: Write %s to %s\n", __func__, buf, filename);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return true;	
}

/** @brief write many words(two bytes)  to file
*	
*	@param filename the file to write
*	@param value the word which will be written to file
*	@param size the size of write data
*
*/
bool Sysfs_write_word_seq(char *filename, uint16_t *value, uint32_t size)
{
	struct file *fp = NULL;
	int i = 0;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[size][5];	

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	for(i=0; i<size; i++){
		sprintf(buf[i], "%04x", value[i]);
		buf[i][4] = ' ';
	}

	/* Open file */
	fp = filp_open(filename, O_RDWR | O_CREAT | O_TRUNC, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		LOG_Handler(LOG_ERR, "%s: File open (%s) fail\n", __func__, filename);
		return false;
	}

	/*For purpose that can use read/write system call*/

	/* Save addr_limit of the current process */
	old_fs = get_fs();
	/* Set addr_limit of the current process to that of kernel */
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->write != NULL) {
		pos_lsts = 0;
		for(i = 0; i < size; i++){
			fp->f_op->write(fp, buf[i], 5, &fp->f_pos);
			//LOG_Handler(LOG_DBG, "%s: 0x%s\n",__func__, buf[i]);
		}
	} else {
		LOG_Handler(LOG_ERR, "%s: File (%s) strlen: f_op=NULL or op->write=NULL\n", __func__, filename);
		return false;
	}
	/* Set addr_limit of the current process back to its own */
	set_fs(old_fs);

	/* Close file */
	filp_close(fp, NULL);

	LOG_Handler(LOG_DBG, "%s: Write %s to %s\n", __func__, buf, filename);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return true;	
}

/** @brief Read offset calibration value
*	
*/
#ifdef kuncomment
int Laser_Forcus_sysfs_read_offset(void){
	int cal_val = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Read offset value from file */
	cal_val = Sysfs_read_int(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FACTORY_FILE, 6);

	if(cal_val < 0) {
		LOG_Handler(LOG_ERR, "%s: Offset Calibration file is NEGATIVE. (%d)\n", __func__, cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		LOG_Handler(LOG_DBG, "%s Read Offset Calibration value : %d\n", __func__, cal_val);
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return cal_val;
}
#endif
/** @brief Write offset calibration value to file
*
*	@param calvalue the offset value
*
*/
#ifdef kuncomment
bool Laser_Forcus_sysfs_write_offset(int calvalue)
{
	bool rc = false;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Write offset value to file */
#ifdef CONFIG_ASUS_FACTORY_MODE
	rc = Sysfs_write_int(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FACTORY_FILE, calvalue);
#else
	rc = Sysfs_write_int(LASERFOCUS_SENSOR_OFFSET_CALIBRATION_FACTORY_FILE, calvalue);
#endif
	if(!rc){
		LOG_Handler(LOG_ERR, "%s: Write Offset Calibration file fail\n", __func__);
		return false;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
#endif
/** @brief Read cross talk calibration value
*	
*/
#ifdef kuncomment
int Laser_Forcus_sysfs_read_cross_talk_offset(void)
{
	int cal_val = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Read cross talk value from file */
	cal_val = Sysfs_read_int(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FACTORY_FILE, 6);

	if(cal_val < 0) {
		LOG_Handler(LOG_ERR, "%s: Cross-talk Offset Calibration file is NEGATIVE. (%d)\n", __func__, cal_val);
		return -EINVAL;	/*Invalid argument*/
	} else {
		LOG_Handler(LOG_DBG, "%s: Read Cross-talk Offset Calibration value : %d\n", __func__, cal_val);
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return cal_val;
}
#endif
/** @brief Write cross talk calibration value to file
*
*	@param calvalue the cross talk value
*
*/
#ifdef kuncomment
bool Laser_Forcus_sysfs_write_cross_talk_offset(int calvalue)
{
	bool rc = false;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	/* Write cross talk value to file */
#ifdef CONFIG_ASUS_FACTORY_MODE
	rc = Sysfs_write_int(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FACTORY_FILE, calvalue);
#else
	rc = Sysfs_write_int(LASERFOCUS_SENSOR_CROSS_TALK_CALIBRATION_FACTORY_FILE, calvalue);
#endif
	if(!rc){
		LOG_Handler(LOG_ERR, "%s: Write Cross-talk Offset Calibration file fail\n", __func__);
		return false;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
#endif
