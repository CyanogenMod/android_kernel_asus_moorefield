/* Copyright (c) 2011-2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "%s:%d " fmt, __func__, __LINE__

#include <linux/ioctl.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#include <linux/platform_device.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/version.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/m12mo_workaround.h>

#include "msm_laser_focus.h"
#include "show_log.h"
#include "laura_debug.h"
#include "laura_interface.h"
#include "laura_factory_func.h"
#include "laura_shipping_func.h"
#include "laser_focus_hepta.h"
#include "olivia_dev.h"


#define LASER_OLIVIA_NAME     "olivia"


#define DO_CAL true
#define NO_CAL false
#define DO_MEASURE true
#define NO_MEASURE false

static int DMax = 400;
int ErrCode = 0;

struct msm_laser_focus_ctrl_t *laura_t = NULL;
static bool camera_on_flag = false;

static bool calibration_flag = true;

static int laser_focus_enforce_ctrl = 0;

static bool load_calibration_data = false;

static int ATD_status;

static int client=0;

extern int Laser_Product;
extern int FirmWare;

//temply use
bool factory = true;
struct msm_laser_focus_ctrl_t *get_laura_ctrl(void){
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return laura_t;
}

bool OLI_device_invalid(void){
	return (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
			laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI);
}	

//extern bool device_invalid(void);


int Laser_Disable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;
	
		mutex_ctrl(laura_t, MUTEX_LOCK);
		if(camera_on_flag){
              	LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
              	return rc;
              }
		#if 0
		rc = dev_deinit(laura_t);
		if (rc < 0)
			return rc;		
		#endif
		uninit_laser_controller();
		laura_t->device_state = val;
		load_calibration_data=false;
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		LOG_Handler(LOG_CDBG, "%s Deinit Device (%d)\n", __func__, laura_t->device_state);
		return rc;
}

int Laser_Enable(enum msm_laser_focus_atd_device_trun_on_type val){

		int rc=0;

		if(camera_on_flag){
              	LOG_Handler(LOG_DBG, "%s: Camera is running, do nothing!!\n ", __func__);
              	return rc;
             	}	
		if (laura_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF){
			//rc = dev_deinit(laura_t);
			laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
		}

		rc = init_laser_controller();
		if (rc){
			LOG_Handler(LOG_DBG, "[Olivia] ISP power on fail!!!", __func__);
			return rc;
		}
		CCI_I2C_RdWord(NULL, 0x28, (uint16_t*)(&rc));
		
		//?
		//laura_t->device_state = val;
		//dev_init(laura_t);
		if(val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION)
			rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag, NO_MEASURE);
		else
			rc = Laura_device_power_up_init_interface(laura_t, NO_CAL, &calibration_flag, NO_MEASURE);			
		if (rc < 0)
			return rc;
		
		laura_t->device_state = val;
		load_calibration_data = (val == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION?true:false);	
		LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
		return rc;
}

int Laser_Enable_by_Camera(enum msm_laser_focus_atd_device_trun_on_type val){

	int rc=0;
	
	laura_t->device_state = val;
	//rc = dev_init(laura_t);
	rc = Laura_device_power_up_init_interface(laura_t, DO_CAL, &calibration_flag, NO_MEASURE);
	if (rc < 0)
		return rc;	
	//?
	laura_t->device_state = val;
	load_calibration_data = true;
	camera_on_flag = true;
	
	LOG_Handler(LOG_CDBG, "%s Init Device (%d)\n", __func__, laura_t->device_state);
	return rc;
}

int Laser_Disable_by_Camera(enum msm_laser_focus_atd_device_trun_on_type val){

	int rc=0;
	mutex_ctrl(laura_t, MUTEX_LOCK);
	#if 0
	rc = dev_deinit(laura_t);
	if (rc < 0)
		return rc;	
	#endif
	laura_t->device_state = val;
	load_calibration_data = false;
	camera_on_flag = false;
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_CDBG, "%s Deinit Device (%d)\n", __func__, laura_t->device_state);
	return rc;
}

static ssize_t ATD_Laura_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, rc = 0;
	char messages[8];

	LOG_Handler(LOG_DBG, "%s: Enter\n", __func__);
	
	len =(len > 8 ?8:len);
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (laura_t->device_state == val)	{
		LOG_Handler(LOG_ERR, "%s Setting same command (%d) !!\n", __func__, val);
		return -EINVAL;
	}
	switch (val) {
		case MSM_LASER_FOCUS_DEVICE_OFF:
			rc = Laser_Disable(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;
			break;
			
		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:	
			rc = Laser_Enable(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;				
			break;
			
		case MSM_LASER_FOCUS_DEVICE_INIT_CCI:		
			rc = Laser_Enable_by_Camera(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;			
			break;
			
		case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
			rc = Laser_Disable_by_Camera(val);
			if (rc < 0)
				goto DEVICE_TURN_ON_ERROR;			
			break;
			
		default:
			LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
			break;
	}

	LOG_Handler(LOG_DBG, "%s: command (%d) done\n",__func__,val);
	return len;
	
DEVICE_TURN_ON_ERROR:

	#if 0
	rc = dev_deinit(laura_t);
	if (rc < 0) 
		LOG_Handler(LOG_ERR, "%s Laura_deinit failed %d\n", __func__, __LINE__);
	#endif

	laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
	
	LOG_Handler(LOG_DBG, "%s: Exit due to device trun on fail !!\n", __func__);
	return -EIO;
}

static int ATD_Laura_device_enable_read(struct seq_file *buf, void *v){
	seq_printf(buf, "%d\n", laura_t->device_state);
	return 0;
}

static int ATD_Laura_device_enable_open(struct inode *inode, struct  file *file){
	return single_open(file, ATD_Laura_device_enable_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_enable_open,
	.write = ATD_Laura_device_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_device_get_range_read(struct seq_file *buf, void *v)
{
	int Range = 0;
#if 0	
	struct timeval start, now;
#endif
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	mutex_ctrl(laura_t, MUTEX_LOCK);
#if 0
	start = get_current_time();
#endif
	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		//return -EBUSY;
		return 0;
	}

	if(laser_focus_enforce_ctrl != 0){
		seq_printf(buf, "%d\n", laser_focus_enforce_ctrl);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}

	Range = Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);

	if (Range >= OUT_OF_RANGE) {
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, Range);
		Range = OUT_OF_RANGE;
	}
	
	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, Range , laura_t->device_state);

	seq_printf(buf, "%d\n", Range);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);
#if 0
	now = get_current_time();
	LOG_Handler(LOG_DBG, "%d ms\n", (int) ((((now.tv_sec*1000000)+now.tv_usec)-((start.tv_sec*1000000)+start.tv_usec))));
#endif
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return 0;
}
 
static int ATD_Laura_device_get_range_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_device_get_range_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_device_get_range_read_more_info(struct seq_file *buf, void *v)
{
	int RawRange = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	mutex_ctrl(laura_t, MUTEX_LOCK);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(laser_focus_enforce_ctrl != 0){
		seq_printf(buf, "%d#%d#%d\n", laser_focus_enforce_ctrl, DMax, ErrCode);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}

	RawRange = (int) Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);

	if (RawRange >= OUT_OF_RANGE) {
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, RawRange);
		RawRange = OUT_OF_RANGE;
	}
	
	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, RawRange , laura_t->device_state);

	seq_printf(buf, "%d#%d#%d\n", RawRange, DMax, ErrCode);

	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return 0;
}
 
static int ATD_Laura_device_get_range_more_info_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_device_get_range_read_more_info, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_more_info_fos = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_more_info_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t ATD_Laura_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}

	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	LOG_Handler(LOG_DBG, "%s command : %d\n", __func__, val);
	
	switch (val) {
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Laura_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			return -EINVAL;
			break;
	}

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return len;
}

static int ATD_Olivia_dummy_read(struct seq_file *buf, void *v)
{
	return 0;
}

static int ATD_Olivia_dummy_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Olivia_dummy_read, NULL);
}

static ssize_t ATD_Olivia_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8];
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);

	if (OLI_device_invalid()){
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}
	len = (len>8?8:len);
	if (copy_from_user(messages, buff, len)){
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	LOG_Handler(LOG_DBG, "%s command : %d\n", __func__, val);	
	switch (val) {
		case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
		case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
		case MSM_LASER_FOCUS_APPLY_INFINITY_CALIBRATION:
			mutex_ctrl(laura_t, MUTEX_LOCK);
			ret = Olivia_device_calibration_interface(laura_t, load_calibration_data, &calibration_flag, val);
			mutex_ctrl(laura_t, MUTEX_UNLOCK);
			if (ret < 0)
				return ret;
			break;
		default:
			LOG_Handler(LOG_ERR, "%s command fail(%d) !!\n", __func__, val);
			return -EINVAL;
			break;
	}

	LOG_Handler(LOG_CDBG, "%s: Exit\n", __func__);	
	return len;
}


static const struct file_operations ATD_olivia_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Olivia_dummy_open,
	.write = ATD_Olivia_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations ATD_laser_focus_device_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_device_get_range_open,
	.write = ATD_Laura_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_get_calibration_input_data_proc_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Laura_get_calibration_input_data_interface, NULL);
}

static const struct file_operations ATD_Laura_get_calibration_input_data_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_get_calibration_input_data_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Olivia_get_calibration_input_data_proc_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Olivia_get_calibration_input_data_interface, NULL);
}

static const struct file_operations ATD_Olivia_get_calibration_input_data_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Olivia_get_calibration_input_data_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	ATD_status = dev_I2C_status_check(laura_t, 0);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	seq_printf(buf, "%d\n", ATD_status);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int ATD_Laura_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ATD_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_Laura_I2C_status_check_proc_read_for_camera(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	seq_printf(buf, "%d\n", ATD_status);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int ATD_Laura_I2C_status_check_proc_open_for_camera(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, ATD_Laura_I2C_status_check_proc_read_for_camera, NULL);
}

static const struct file_operations ATD_I2C_status_check_for_camera_fops = {
	.owner = THIS_MODULE,
	.open = ATD_Laura_I2C_status_check_proc_open_for_camera,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_register_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_laura_register_read, NULL);
}

static const struct file_operations dump_laser_focus_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_register_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	//mutex_ctrl(laura_t, MUTEX_LOCK);
	laura_debug_dump(buf, v);
	//mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_register_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_register_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#if 0
static int Laura_laser_focus_enforce_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int Laura_laser_focus_enforce_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Laura_laser_focus_enforce_read, NULL);
}

static ssize_t Laura_laser_focus_enforce_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_enforce(laura_t, buff, len, &laser_focus_enforce_ctrl);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}
#endif

#if 0
static const struct file_operations laser_focus_enforce_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_enforce_open,
	.write = Laura_laser_focus_enforce_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
#endif

static int Laura_laser_focus_log_contorl_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return 0;
}

static int Laura_laser_focus_log_contorl_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, Laura_laser_focus_log_contorl_read, NULL);
}

static ssize_t Laura_laser_focus_log_contorl_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	mutex_ctrl(laura_t, MUTEX_LOCK);
	rc = Laser_Focus_log_contorl(buff, len);
	mutex_ctrl(laura_t, MUTEX_UNLOCK);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return rc;
}

static const struct file_operations laser_focus_log_contorl_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_log_contorl_open,
	.write = Laura_laser_focus_log_contorl_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

/*++++++++++CE Debug++++++++++*/
static int dump_Laura_debug_value1_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

	seq_printf(buf,"%d",cal_data[3]);	
	
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value1_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value1_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value1_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value1_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value2_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

        seq_printf(buf,"%d",cal_data[4]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value2_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value2_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value2_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value2_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value3_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

        seq_printf(buf,"%d",cal_data[6]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value3_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value3_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value3_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value3_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value4_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

        seq_printf(buf,"%d",cal_data[7]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value4_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value4_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value4_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value4_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int dump_Laura_debug_value5_read(struct seq_file *buf, void *v)
{
	int16_t cal_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	
	Olivia_Read_Calibration_Value_From_File(NULL, cal_data);

        seq_printf(buf,"%d",cal_data[8]);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value5_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value5_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value5_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value5_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dump_Laura_debug_value6_read(struct seq_file *buf, void *v)
{
	int16_t rc = 0, RawConfidence = 0, confidence_level = 0;;
	
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		return -EBUSY;
	}

	/* Read result confidence level */
	rc = CCI_I2C_RdWord(laura_t, 0x0A, &RawConfidence);
	if (rc < 0){
		return rc;
	}
	//RawConfidence = swap_data(RawConfidence);
	confidence_level = (RawConfidence&0x7fff)>>4;
	
	seq_printf(buf,"%d",confidence_level);

	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

	return 0;
}

static int dump_Laura_laser_focus_debug_value6_open(struct inode *inode, struct  file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	return single_open(file, dump_Laura_debug_value6_read, NULL);
}

static const struct file_operations dump_laser_focus_debug_value6_fops = {
	.owner = THIS_MODULE,
	.open = dump_Laura_laser_focus_debug_value6_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

//ATD will tell how to impliment it
static int dump_Laura_value_check_read(struct seq_file *buf, void *v)
{
        LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);

	seq_printf(buf,"PASS\n");

        LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);

        return 0;
}

static int dump_Laura_laser_focus_value_check_open(struct inode *inode, struct  file *file)
{
        LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
        LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
        return single_open(file, dump_Laura_value_check_read, NULL);
}

static const struct file_operations dump_laser_focus_value_check_fops = {
        .owner = THIS_MODULE,
        .open = dump_Laura_laser_focus_value_check_open,
        .read = seq_read,
        .llseek = seq_lseek,
        .release = single_release,
};
/*----------CE Debug----------*/


static int Laura_laser_focus_set_K_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, calibration: %d\n", __func__, calibration_flag);
	seq_printf(buf,"%d",calibration_flag);
	return 0;
}
static int Laura_laser_focus_set_K_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_set_K_read, NULL);
}

static ssize_t Laura_laser_focus_set_K_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	ssize_t rc;
	int val;
	char messages[8];
	LOG_Handler(LOG_CDBG, "%s: Enter\n", __func__);

	len = (len>8?8:len);
	if (copy_from_user(messages, buff, len)){
		LOG_Handler(LOG_ERR, "%s command fail !!\n", __func__);
		return -EFAULT;
	}
	val = (int)simple_strtol(messages, NULL, 10);
	
	switch(val){
		case 0:
			calibration_flag = false;
			break;
		case 1:
			calibration_flag = true;
			break;
		default:
			LOG_Handler(LOG_DBG, "command '%d' is not valid\n", val);
	}

	LOG_Handler(LOG_CDBG, "%s: Exit, calibration: %d\n", __func__, calibration_flag);
	return rc;
}

static const struct file_operations laser_focus_set_K_fops = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_set_K_open,
	.write = Laura_laser_focus_set_K_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int Laura_laser_focus_product_family_read(struct seq_file *buf, void *v)
{
	LOG_Handler(LOG_DBG, "%s: Enter and Exit, Product Family: %d\n", __func__, Laser_Product);
	init_laser_controller();
	Laser_Match_Module(laura_t);
	uninit_laser_controller();
	seq_printf(buf,"%d",Laser_Product);
	return 0;
}
static int Laura_laser_focus_product_family_open(struct inode *inode, struct file *file)
{
	LOG_Handler(LOG_FUN, "%s: Enter and Exit\n", __func__);
	return single_open(file, Laura_laser_focus_product_family_read, NULL);
}

static const struct file_operations laser_focus_product_family = {
	.owner = THIS_MODULE,
	.open = Laura_laser_focus_product_family_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int laser_read_cali_data(struct seq_file *buf, void *v) {
	char buff[128];
	int i;
	unsigned int cali_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	unsigned int cali_crc;
	u16 mCali_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	u8 mCali_crc;
	read_cali_data_to_isp(mCali_data,&mCali_crc);
	for(i=0;i<SIZE_OF_OLIVIA_CALIBRATION_DATA;i++) {
		cali_data[i] = mCali_data[i];
	}
	cali_crc = mCali_crc;
	sprintf(buff, "%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %02x",
			cali_data[0],cali_data[1],cali_data[2],cali_data[3],cali_data[4],cali_data[5],cali_data[6],
			cali_data[7],cali_data[8],cali_data[9],cali_crc);
	seq_printf(buf,"%s",buff);
	return 0;
}
static int laser_read_cali_data_fops_open(struct inode *inode, struct file *file) {
	return single_open(file, laser_read_cali_data, NULL);
}
static ssize_t laser_write_cali_data_fops_write(struct file *filp, const char __user *buff, size_t len, loff_t *data) {
	int i;
	unsigned int cali_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	unsigned int cali_crc;
	u16 mCali_data[SIZE_OF_OLIVIA_CALIBRATION_DATA];
	u8 mCali_crc;
	sscanf(buff, "%04x %04x %04x %04x %04x %04x %04x %04x %04x %04x %02x",
			&cali_data[0],&cali_data[1],&cali_data[2],&cali_data[3],&cali_data[4],&cali_data[5],&cali_data[6],
			&cali_data[7],&cali_data[8],&cali_data[9],&cali_crc);
	for(i=0;i<SIZE_OF_OLIVIA_CALIBRATION_DATA;i++) {
		mCali_data[i] = cali_data[i];
	}
	mCali_crc = cali_crc;
	write_cali_data_to_isp(mCali_data,mCali_crc);
	return len;
}
static const struct file_operations laser_write_cali_data_fops = {
	.write = laser_write_cali_data_fops_write,
};

static const struct file_operations laser_read_cali_data_fops = {
	.owner = THIS_MODULE,
	.open = laser_read_cali_data_fops_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

#define MODULE_NAME "LaserSensor"
#define ASUS_LASER_NAME_SIZE	32
#define ASUS_LASER_DATA_SIZE	4
#define OLIVIA_IOC_MAGIC                      ('W')
#define ASUS_LASER_SENSOR_MEASURE     _IOR(OLIVIA_IOC_MAGIC  , 0, unsigned int[ASUS_LASER_DATA_SIZE])
#define ASUS_LASER_SENSOR_GET_NAME	_IOR(OLIVIA_IOC_MAGIC  , 4, char[ASUS_LASER_NAME_SIZE])

int Olivia_get_measure(int* distance){
	int RawRange = 0;

	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);
	mutex_ctrl(laura_t, MUTEX_LOCK);

	if (laura_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		laura_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		LOG_Handler(LOG_ERR, "%s: Device without turn on: (%d) \n", __func__, laura_t->device_state);
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return -EBUSY;
	}

	if(laser_focus_enforce_ctrl != 0){
		*distance = laser_focus_enforce_ctrl;
		mutex_ctrl(laura_t, MUTEX_UNLOCK);
		return 0;
	}

	RawRange = (int) Laura_device_read_range_interface(laura_t, load_calibration_data, &calibration_flag);

	if (RawRange < 0) {
		LOG_Handler(LOG_ERR, "%s: Read_range(%d) failed\n", __func__, RawRange);
		RawRange = 0;
	}
	
	LOG_Handler(LOG_DBG, "%s : Get range (%d)  Device (%d)\n", __func__, RawRange , laura_t->device_state);


	mutex_ctrl(laura_t, MUTEX_UNLOCK);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);	
	return 0;


}

static int Olivia_misc_open(struct inode *inode, struct file *file){

	client++;
	LOG_Handler(LOG_DBG,"%s: client enter(%d)\n", __func__, client);

	power_up(laura_t);

	//camera may open once at 70cm K
	if(factory)
		Laser_Enable(MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION);
	else
		Laser_Enable(MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION);
	
	return 0;
}

static int Olivia_misc_release(struct inode *inode, struct file *file)
{
	client--;
	LOG_Handler(LOG_DBG,"%s: client leave(%d)\n", __func__, client);


	if(client <=0){
		Laser_Disable(MSM_LASER_FOCUS_DEVICE_OFF);
		power_down(laura_t);
	}

	return 0;
}

static long Olivia_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{

 	unsigned int dist[4] = {0,0,0,0};
	char name[ASUS_LASER_NAME_SIZE];
	int distance;
	int ret = 0;
	snprintf(name, ASUS_LASER_NAME_SIZE, MODULE_NAME);
	
	switch (cmd) {
		
		case ASUS_LASER_SENSOR_MEASURE:
			Olivia_get_measure(&distance);
			//__put_user(dist, (int* __user*)arg);
			dist[0] = distance;
			dist[1] = ErrCode;
			dist[2] = DMax;
			dist[3] = calibration_flag;
			ret = copy_to_user((int __user*)arg, dist, sizeof(dist));
			LOG_Handler(LOG_DBG, "%s: range data [%d,%d,%d,%d]\n"
								,__func__,distance,ErrCode,DMax,calibration_flag);
			break;
		case ASUS_LASER_SENSOR_GET_NAME:
			//__put_user(MODULE_NAME, (int __user*)arg);
			ret = copy_to_user((int __user*)arg, &name, sizeof(name));			
			break;
		default:
			LOG_Handler(LOG_ERR,"%s: ioctrl command is not valid\n", __func__);
	}
	return 0;

}
static struct file_operations Olivia_fops = {
  .owner = THIS_MODULE,
  .open = Olivia_misc_open,
  .release = Olivia_misc_release,
  .unlocked_ioctl = Olivia_misc_ioctl,
  .compat_ioctl = Olivia_misc_ioctl
};

struct miscdevice Olivia_misc = {
  .minor = MISC_DYNAMIC_MINOR,
  .name = MODULE_NAME,
  .fops = &Olivia_fops
};

static int Olivia_misc_register(int Product)
{
	int rtn = 0;

	if(Product != PRODUCT_OLIVIA){
		LOG_Handler(LOG_CDBG, "LaserFocus is not supported, ProductFamily is not Olivia (%d)",Product);
		return rtn;
	}
	
	rtn = misc_register(&Olivia_misc);
	if (rtn < 0) {
		LOG_Handler(LOG_ERR,"Unable to register misc devices\n");
		misc_deregister(&Olivia_misc);
	}
	return rtn;
}



/*#define proc(file,mod,fop)	\
	LOG_Handler(LOG_DBG,"proc %s %s\n",file,proc_create(file,mod,NULL,fop)? "success":"fail");*/
#define proc(file,mod,fop)	\
	pr_err("[Kun]proc %s %s\n",file,proc_create(file,mod,NULL,fop)? "success":"fail");


static void Olivia_Create_proc(void)
{
	pr_err("[Kun]Olivia_Create_proc : E\n");
	proc(STATUS_PROC_FILE, 0776, &ATD_I2C_status_check_fops);
	proc(STATUS_PROC_FILE_FOR_CAMERA, 0776, &ATD_I2C_status_check_for_camera_fops);
	proc(DEVICE_TURN_ON_FILE, 0776, &ATD_laser_focus_device_enable_fops);
	proc(DEVICE_GET_VALUE, 0776, &ATD_laser_focus_device_get_range_fos);
	proc(DEVICE_GET_VALUE_MORE_INFO, 0776, &ATD_laser_focus_device_get_range_more_info_fos);

if(Laser_Product == PRODUCT_OLIVIA){
	proc(DEVICE_SET_CALIBRATION, 0776, &ATD_olivia_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0776, &ATD_Olivia_get_calibration_input_data_fops);
}
else{
	proc(DEVICE_SET_CALIBRATION, 0776, &ATD_laser_focus_device_calibration_fops);
	proc(DEVICE_GET_CALIBRATION_INPUT_DATA, 0776, &ATD_Laura_get_calibration_input_data_fops);
}
	
	proc(DEVICE_DUMP_REGISTER_VALUE, 0664, &dump_laser_focus_register_fops);
	proc(DEVICE_DUMP_DEBUG_VALUE, 0664, &dump_laser_focus_debug_register_fops);
	//proc(DEVICE_ENFORCE_FILE, 0664, &laser_focus_enforce_fops);
	proc(DEVICE_LOG_CTRL_FILE, 0664, &laser_focus_log_contorl_fops);

	//for ce
	proc(DEVICE_DEBUG_VALUE1, 0664, &dump_laser_focus_debug_value1_fops);
	proc(DEVICE_DEBUG_VALUE2, 0664, &dump_laser_focus_debug_value2_fops);
	proc(DEVICE_DEBUG_VALUE3, 0664, &dump_laser_focus_debug_value3_fops);
	proc(DEVICE_DEBUG_VALUE4, 0664, &dump_laser_focus_debug_value4_fops);
	proc(DEVICE_DEBUG_VALUE5, 0664, &dump_laser_focus_debug_value5_fops);	
	proc(DEVICE_DEBUG_VALUE6, 0664, &dump_laser_focus_debug_value6_fops);	
	proc(DEVICE_VALUE_CHECK, 0664, &dump_laser_focus_value_check_fops);
	//for ce

	//for dit
	proc(DEVICE_IOCTL_SET_K, 0776, &laser_focus_set_K_fops);
	proc(DEVICE_IOCTL_PRODUCT_FAMILY, 0776, &laser_focus_product_family);
	//for dit
	proc(DEVICE_WRITE_CALI_DATA, 0776, &laser_write_cali_data_fops);
	proc(DEVICE_READ_CALI_DATA, 0776, &laser_read_cali_data_fops);
	pr_err("[Kun]Olivia_Create_proc : X\n");
}

static void set_laser_config(struct platform_device *pdev){

	/* Init data struct */
	laura_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	laura_t->laser_focus_cross_talk_offset_value = 0;
	laura_t->laser_focus_offset_value = 0;
	laura_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
	
	LOG_Handler(LOG_FUN, "%s: done\n", __func__);
}

static int32_t Olivia_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;
	//,gp_sdio_2_clk,ret;
	//struct device_node *np = pdev->dev.of_node;
	//LOG_Handler(LOG_CDBG, "%s: Probe Start\n", __func__);
	pr_err("[Kun]Olivia_platform_probe\n");


        laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),
                GFP_KERNEL);
        if (!laura_t) {
                pr_err("%s:%d failed no memory\n", __func__, __LINE__);
                return -ENOMEM;
        }
	#if 0
	if(g_ASUS_laserID == 1){
		printk("[LASER_FOCUS] It is VL6180 sensor, do nothing!!\n");
		return -1;
	}
	#endif

/*
	////
	gp_sdio_2_clk = of_get_named_gpio(np, "gpios", 0);

	printk("%s: gp_sdio_2_clk = %d\n", __FUNCTION__, gp_sdio_2_clk);
	if ((!gpio_is_valid(gp_sdio_2_clk))) {
		printk("%s: gp_sdio_2_clk is not valid!\n", __FUNCTION__);
	}
	////
	ret = gpio_request(gp_sdio_2_clk,"LAURA");
	if (ret < 0) {
		printk("%s: request CHG_OTG gpio fail!\n", __FUNCTION__);
	}
	////
	gpio_direction_output(gp_sdio_2_clk, 0);
	if (ret < 0) {
		printk("%s: set direction of CHG_OTG gpio fail!\n", __FUNCTION__);
	}
	gpio_set_value(gp_sdio_2_clk, 1);
	printk("gpio_get_value %d\n"  ,gpio_get_value(gp_sdio_2_clk));

*/


#if 0
	rc = match_Olivia(pdev);
	if(rc < 0)
		goto probe_failure;

	pr_err("[Kun]get_dtsi_data\n");
	rc = get_dtsi_data(pdev->dev.of_node, laura_t);
	if (rc < 0) 
		goto probe_failure;

	pr_err("[Kun]set_i2c_client\n");
	rc = set_i2c_client(pdev);	
	if (rc < 0)
		goto probe_failure;

	pr_err("[Kun]set_cci_client\n");	
	set_cci_client(pdev);
	pr_err("[Kun]set_subdev\n");
	set_subdev(pdev);
	pr_err("[Kun]set_laser_config\n");
#endif
	set_laser_config(pdev);
	pr_err("[Kun]set_laser_config finished\n");

#if 0 // to do something	
	/* Check I2C status */
	if(dev_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA) == 0)
		goto probe_failure;
#endif

	/* Init mutex */
        mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

#if 0 // disable these function if we not always keep power
	pr_err("[Kun]Laura_Init_Chip_Status_On_Boot\n");
	Laura_Init_Chip_Status_On_Boot(laura_t);
	pr_err("[Kun]Olivia_Create_proc();\n");
#endif
	Olivia_Create_proc();

	pr_err("[Kun]Olivia_misc_register;\n");
	rc = Olivia_misc_register(Laser_Product);
	if (rc < 0)
		goto probe_failure;
	ATD_status = 2;	

	pr_err("[Kun] Probe Sucess\n");
	//LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;
	
probe_failure:
	//LOG_Handler(LOG_CDBG, "%s: Probe failed, rc = %d\n", __func__, rc);
	pr_err("[Kun] Probe failed\n");
	return rc;
}

#if 0
static int32_t Laura_platform_probe(struct platform_device *pdev)
{
	int32_t rc = 0;//, i = 0;
	//uint32_t id_info[3];
	const struct of_device_id *match;
	struct msm_camera_cci_client *cci_client = NULL;
	
	//if(g_ASUS_laserID == 1)
                //LOG_Handler(LOG_ERR, "%s: It is VL6180x sensor, do nothing!!\n", __func__);
               // return rc;
        
	printk("Laura_platform_probe\n");
	LOG_Handler(LOG_CDBG, "%s: Probe Start\n", __func__);
	ATD_status = 0;
	
	match = of_match_device(msm_laser_focus_dt_match, &pdev->dev);
	if (!match) {
		pr_err("device not match\n");
		return -EFAULT;
	}

	if (!pdev->dev.of_node) {
		pr_err("of_node NULL\n");
		return -EINVAL;
	}
	laura_t = kzalloc(sizeof(struct msm_laser_focus_ctrl_t),
		GFP_KERNEL);
	if (!laura_t) {
		pr_err("%s:%d failed no memory\n", __func__, __LINE__);
		return -ENOMEM;
	}
	/* Set platform device handle */
	laura_t->pdev = pdev;

	rc = get_dtsi_data(pdev->dev.of_node, laura_t);
	if (rc < 0) {
		pr_err("%s failed line %d rc = %d\n", __func__, __LINE__, rc);
		return rc;
	}
	/* Assign name for sub device */
	snprintf(laura_t->msm_sd.sd.name, sizeof(laura_t->msm_sd.sd.name),
			"%s", laura_t->sensordata->sensor_name);

	laura_t->act_v4l2_subdev_ops = &msm_laser_focus_subdev_ops;
	//laura_t->laser_focus_mutex = &msm_laser_focus_mutex;
	//laura_t->cam_name = pdev->id;

	/* Set device type as platform device */
	laura_t->act_device_type = MSM_CAMERA_PLATFORM_DEVICE;
	laura_t->i2c_client = &msm_laser_focus_i2c_client;
	if (NULL == laura_t->i2c_client) {
		pr_err("%s i2c_client NULL\n",
			__func__);
		rc = -EFAULT;
		goto probe_failure;
	}
	if (!laura_t->i2c_client->i2c_func_tbl)
		laura_t->i2c_client->i2c_func_tbl = &msm_sensor_cci_func_tbl;

	laura_t->i2c_client->cci_client = kzalloc(sizeof(
		struct msm_camera_cci_client), GFP_KERNEL);
	if (!laura_t->i2c_client->cci_client) {
		kfree(laura_t->vreg_cfg.cam_vreg);
		kfree(laura_t);
		pr_err("failed no memory\n");
		return -ENOMEM;
	}
	//laura_t->i2c_client->addr_type = MSM_CAMERA_I2C_WORD_ADDR;
	cci_client = laura_t->i2c_client->cci_client;
	cci_client->cci_subdev = msm_cci_get_subdev();
	cci_client->cci_i2c_master = laura_t->cci_master;
	if (laura_t->sensordata->slave_info->sensor_slave_addr)
		cci_client->sid = laura_t->sensordata->slave_info->sensor_slave_addr >> 1;
	cci_client->retries = 3;
	cci_client->id_map = 0;
	cci_client->i2c_freq_mode = I2C_FAST_MODE;
	v4l2_subdev_init(&laura_t->msm_sd.sd,
		laura_t->act_v4l2_subdev_ops);
	v4l2_set_subdevdata(&laura_t->msm_sd.sd, laura_t);
	laura_t->msm_sd.sd.internal_ops = &msm_laser_focus_internal_ops;
	laura_t->msm_sd.sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	snprintf(laura_t->msm_sd.sd.name,
		ARRAY_SIZE(laura_t->msm_sd.sd.name), "msm_laser_focus");
	media_entity_init(&laura_t->msm_sd.sd.entity, 0, NULL, 0);
	laura_t->msm_sd.sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV;
	//laura_t->msm_sd.sd.entity.group_id = MSM_CAMERA_SUBDEV_LASER_FOCUS;
	laura_t->msm_sd.close_seq = MSM_SD_CLOSE_2ND_CATEGORY | 0x2;
	msm_sd_register(&laura_t->msm_sd);
	laura_t->laser_focus_state = LASER_FOCUS_POWER_DOWN;

	/* Init data struct */
	laura_t->laser_focus_cross_talk_offset_value = 0;
	laura_t->laser_focus_offset_value = 0;
	laura_t->laser_focus_state = MSM_LASER_FOCUS_DEVICE_OFF;

	/* Check I2C status */
	if(dev_I2C_status_check(laura_t, MSM_CAMERA_I2C_WORD_DATA) == 0)
		goto probe_failure;

	/* Init mutex */
       mutex_ctrl(laura_t, MUTEX_ALLOCATE);
	mutex_ctrl(laura_t, MUTEX_INIT);

	ATD_status = 2;
	
	/* Create proc file */
	Olivia_Create_proc();
	LOG_Handler(LOG_CDBG, "%s: Probe Success\n", __func__);
	return 0;
probe_failure:
	LOG_Handler(LOG_CDBG, "%s: Probe failed\n", __func__);

	return rc;
}
#endif
static struct platform_driver olivia_laser_focus_platform_driver = {
	.driver = {
		.name = LASER_OLIVIA_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device olivia_laser_platform_device = {
        .name   = LASER_OLIVIA_NAME,
};


static int __init Laura_init_module(void)
{
	int32_t rc = 0;
	pr_err("[Kun]Laura_init_module\n");
	LOG_Handler(LOG_DBG, "%s: Olivia_hugo_Enter\n", __func__);
	printk("%s: Enter\n", __func__);
	
	platform_device_register(&olivia_laser_platform_device);
	rc = platform_driver_probe(&olivia_laser_focus_platform_driver,
		Olivia_platform_probe);

//	LOG_Handler(LOG_DBG, "%s rc %d\n", __func__, rc);
	printk("%s rc %d\n", __func__, rc);

	return rc;
}
static void __exit Laura_driver_exit(void)
{
	LOG_Handler(LOG_FUN, "%s: Enter\n", __func__);	
	//platform_driver_unregister(&msm_laser_focus_platform_driver);
	LOG_Handler(LOG_FUN, "%s: Exit\n", __func__);
	
	return;
}

module_init(Laura_init_module);
module_exit(Laura_driver_exit);
MODULE_DESCRIPTION("MSM LASER_FOCUS");
MODULE_LICENSE("GPL v2");
