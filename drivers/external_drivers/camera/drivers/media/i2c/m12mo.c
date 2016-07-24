/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * Partially based on m-5mols kernel driver,
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *
 * Partially based on jc_v4l2 kernel driver from http://opensource.samsung.com
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <asm/intel-mid.h>
#include <asm/irq.h>
#include <linux/atomisp_platform.h>
#include <linux/atomisp.h>
#include <media/m12mo_atomisp.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/kmod.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <linux/wait.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <linux/m12mo.h>
#include <linux/HWVersion.h>
#include <linux/m12mo_workaround.h>
#include <asm/intel_scu_pmic.h>
#include <linux/switch.h>
#include <linux/proc_fs.h>

#define off_c51
#define GPIO4CTLO_REG 0x82 //This REGISTER is for SKY81298 enable pin.
//#define NOIRQ
/* cross component debug message flag */
static bool PREVIEW_TEST_PATTERN = 0;
int dbglvl = 0;
static u32 previous_preview_cmd;
static int isp_update_status=1;
static u8 m12mo_capture_pre_flag;
static bool isCaptureMode = false;
static bool isFlashFwFail = false;
static u32 asus_camera_flag;
#ifndef ZX551ML_USER_BUILD //#ifdef CONFIG_ASUS_FACTORY_MODE
static u32 password_debug;
static bool CacCalibrationStatus = false;
#endif
static u8 m12mo_trace_log_lock = DOWNLOAD_PROTECT;
int ois_status = 0;
bool i2c_check_status[8] = {0};
int num_LLS = 0;
static struct v4l2_subdev *sdd = NULL;
module_param(dbglvl, int, 0644);
MODULE_PARM_DESC(dbglvl, "debug message on/off (default:off)");
static int m12mo_set_af_execution(struct v4l2_subdev *sd, s32 val);
static int m12mo_ispd4(struct m12mo_device *dev);
static u32 get_m12mo_wait_timeout_val(struct v4l2_subdev *sd, u8 requested_cmd);
extern int m12mo_break_log_loop;

// = M12MO_START_AF;//M12MO_NO_CMD_REQUEST;
/*#define __m12mo_reset_irq_wait_status()                                           \*/
	 /*dev->cmd = dev->requested_cmd;                                               \*/
     /*wake_up(&dev->irq_waitq);                                      \*/
	 /*flush_workqueue(dev->lens_wq);                                               \*/
	 /*dev->cmd = dev->requested_cmd = M12MO_NO_CMD_REQUEST;                        \*/
	 /*dev->wait_irq_flag = M12MO_IRQ_COMMAND_AVAILABDLE*/
////////////////////////////////////////////////////////////////////


static struct switch_dev m12mo_switch_dev;
static ssize_t m12mo_switch_name(struct switch_dev *sdev, char *buf)
{
//	return sprintf(buf, "%x%x-%x,V%03x,%x\n", fw_version_val1, fw_version_val2,DIT_version, ene_fw_version,ASUS_flag);
    struct m12mo_device *dev = to_m12mo_sensor(sdd);
    return sprintf(buf, "%04x-%x,%x\n", dev->ver.firmware, dev->ver.DIT_firmware, dev->ver.fw_auto_flash_flag);
}

void m12mo_startCapture(void) {
    if(!PREVIEW_TEST_PATTERN){
        printk("m12mo, @%s, Wait start! called by atomisp driver. \n", __func__);
        m12mo_request_cmd_effect(sdd, M12MO_SINGLE_CAPTURE_MODE, NULL);
		printk("m12mo, @%s, Wait end! called by atomisp driver. \n", __func__);
    }
}
EXPORT_SYMBOL(m12mo_startCapture);

void notify_m12mo_atomisp_dead(void) {
    struct m12mo_device *dev = to_m12mo_sensor(sdd);
	u8 previous_state;
	if(m12mo_trace_log_lock != DOWNLOAD_AVAILABLE) {
	      return;
	}
	if(dev->power == 1){
	      previous_state = m12mo_trace_log_lock;
	      dev->lock_i2c_write_flag = 0;
		  m12mo_trace_log_lock = DOWNLOADING;
          m12mo_ispd4(dev);
		  m12mo_trace_log_lock = previous_state;
	}
}
EXPORT_SYMBOL(notify_m12mo_atomisp_dead);

/*
 * m12mo_read -  I2C read function
 * @reg: combination of size, category and command for the I2C packet
 * @size: desired size of I2C packet
 * @val: read value
 *
 * Returns 0 on success, or else negative errno.
*/

static int m12mo_read(struct v4l2_subdev *sd, u8 len, u8 category, u8 reg, u32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	unsigned char data[5];
	struct i2c_msg msg[2];
	unsigned char recv_data[len + 1];
	int ret;

	if (!client->adapter)
		return -ENODEV;

	if (len != 1 && len != 2 && len != 4)	{
		dev_err(&client->dev, "Wrong data size\n");
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(data);
	msg[0].buf = data;

	data[0] = 5;
	data[1] = M12MO_BYTE_READ;
	data[2] = category;
	data[3] = reg;
        data[4] = len;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len + 1;
	msg[1].buf = recv_data;

	/* isp firmware becomes stable during this time*/
	usleep_range(200, 200);

	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret == 2) {
		if (len == 0x01)
			*val = recv_data[1];
		else if (len == 0x02)
			*val = recv_data[1] << 8 | recv_data[2];
		else
			*val = recv_data[1] << 24 | recv_data[2] << 16
				| recv_data[3] << 8 | recv_data[4];
	}

	printk("%s len :%d cat, reg, val: 0x%02x, 0x%02x, 0x%02x\n", __func__, len, category, reg, *val);
	dev_dbg(&client->dev,
		"%s len :%d cat, reg, val: 0x%02x, 0x%02x, 0x%02x\n",
		__func__, len, category, reg, *val);

	return (ret == 2) ? 0 : -EIO;
}

/**
 * m12mo_write - I2C command write function
 * @reg: combination of size, category and command for the I2C packet
 * @val: value to write
 *
 * Returns 0 on success, or else negative errno.
 */
static int m12mo_write(struct v4l2_subdev *sd, u8 len, u8 category, u8 reg, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[len + 4];
	struct i2c_msg msg;
	int ret;
	const int num_msg = 1;
	if (!client->adapter)
		return -ENODEV;

	if (len != 1 && len != 2 && len != 4) {
		dev_err(&client->dev, "Wrong data size\n");
		return -EINVAL;
	}

	dev_dbg(&client->dev,
		"%s len :%d cat, reg, val: 0x%02x, 0x%02x, 0x%02x\n",
		__func__, len, category, reg, val);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	data[0] = msg.len;
	data[1] = M12MO_BYTE_WRITE;
	data[2] = category;
	data[3] = reg;
	switch (len) {
	case 1:
		data[4] = val;
		break;
	case 2:
		data[4] = ((val >> 8) & 0xFF);
		data[5] = (val & 0xFF);
		break;
	case 4:
		data[4] = ((val >> 24) & 0xFF);
		data[5] = ((val >> 16) & 0xFF);
		data[6] = ((val >> 8) & 0xFF);
		data[7] = (val & 0xFF);
		break;
	default:
		/* No possible to happen - len is already validated */
		break;
	}

	/* isp firmware becomes stable during this time*/
	usleep_range(200, 200);

	ret = i2c_transfer(client->adapter, &msg, 1);

	printk("@%s, Write reg. len = %d, Category=0x%02X Reg=0x%02X Value=0x%X ret=%s\n", __func__,
		len, category, reg, val, (ret == num_msg) ? "OK" : "Error");

	return ret == num_msg ? 0 : -EIO;
}

int m12mo_writeb(struct v4l2_subdev *sd, u8 category, u8 reg, u32 val)
{
	return m12mo_write(sd, 1, category, reg, val);
}

int m12mo_writew(struct v4l2_subdev *sd, u8 category, u8 reg, u32 val)
{
	return m12mo_write(sd, 2, category, reg, val);
}

int m12mo_writel(struct v4l2_subdev *sd, u8 category, u8 reg, u32 val)
{
	return m12mo_write(sd, 4, category, reg, val);
}

int m12mo_readb(struct v4l2_subdev *sd, u8 category, u8 reg, u32 *val)
{
	return m12mo_read(sd, 1, category, reg, val);
}

int m12mo_readw(struct v4l2_subdev *sd, u8 category, u8 reg, u32 *val)
{
	return m12mo_read(sd, 2, category, reg, val);
}

int m12mo_readl(struct v4l2_subdev *sd, u8 category, u8 reg, u32 *val)
{
	return m12mo_read(sd, 4, category, reg, val);
}

int m12mo_memory_write(struct v4l2_subdev *sd, u8 cmd, u16 len, u32 addr, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m12mo_device *m12mo_dev =  to_m12mo_sensor(sd);
	struct i2c_msg msg;
	u8 *data = m12mo_dev->message_buffer;
	int i, ret;

	if(len==1)
	    dev_err(&client->dev, "Write mem. cmd=0x%02X len=%d addr=0x%X val=0x%X\n", cmd, len, addr, *val);
	else{
	   	dev_info(&client->dev, "Write mem. cmd=0x%02X len=%d addr=0x%X\n", cmd, len, addr);
	    for(i=0;i<len;i++){
            dev_dbg(&client->dev, "i=%d val=0x%X  ",i, val[i]);
	    }
	}
	if (!client->adapter){
        printk("%s %d failed\n", __func__, __LINE__);
		return -ENODEV;
    }

	if ((len + 8) > sizeof(m12mo_dev->message_buffer))
		return -ENOMEM;
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len + 8;
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = cmd;
	data[2] = (u8)((addr >> 24) & 0xFF);
	data[3] = (u8)((addr >> 16) & 0xFF);
	data[4] = (u8)((addr >> 8) & 0xFF);
	data[5] = (u8)(addr & 0xFF);
	data[6] = len >> 8;
	data[7] = len;
	/* Payload starts at offset 8 */
	memcpy(data + 8, val, len);

	usleep_range(200, 200);

	for (i = M12MO_I2C_RETRY; i; i--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1) {
			return 0;
		}
		msleep(20);
	}

	return ret;
}

int m12mo_memory_read(struct v4l2_subdev *sd, u16 len, u32 addr, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m12mo_device *m12mo_dev =  to_m12mo_sensor(sd);
	struct i2c_msg msg;
	unsigned char data[8];
	u8 *recv_data = m12mo_dev->message_buffer;
	int i, err = 0;

	if (!client->adapter)
		return -ENODEV;

	if (len <= 0)
		return -EINVAL;

	if ((len + 3) > sizeof(m12mo_dev->message_buffer))
		return -ENOMEM;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = 0x03;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;

	for (i = M12MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err == 0)
		return -EIO;

	if (err != 1)
		return err;

	msg.flags = I2C_M_RD;
	msg.len = len + 3;
	msg.buf = recv_data;
	for (i = M12MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		msleep(20);
	}

	if (err == 0)
		return -EIO;

	if (err != 1)
		return err;

	if (len != (recv_data[1] << 8 | recv_data[2])) {
		dev_err(&client->dev,
			"expected length %d, but return length %d\n",
			len, recv_data[1] << 8 | recv_data[2]);
		return -EIO;
	}

	memcpy(val, recv_data + 3, len);

	dev_dbg(&client->dev, "Read mem. len=%d addr=0x%X\n", len, addr);
	return 0;
}

static ssize_t asus_camera_show(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{
    char *buff = NULL;
    ssize_t ret = 0;
    int len = 0;
    int status = (int) asus_camera_flag;

    len += sprintf(buff+len, "%d\n", status);
    ret = simple_read_from_buffer(buffer, count, ppos, buff, len);
    kfree(buff);
    return ret;
}

static ssize_t asus_camera_store(struct file *dev, const char *buf, size_t count, loff_t *loff)
{
    int camera_status = -1;

    sscanf(buf, "%d ", &camera_status);
	printk("@%s %d, camera_status is %d\n", __func__, __LINE__, camera_status);
    if(camera_status > 0){
        asus_camera_flag = 1;
    }else{
        asus_camera_flag = 0;
    }
    return count;
}

static const struct file_operations asus_camera_proc_fops = {
     .read = asus_camera_show,
     .write = asus_camera_store,
};
/**
 * m12mo_setup_flash_controller - initialize flash controller
 *
 * Flash controller requires additional setup before
 * the use.
 */
int m12mo_setup_flash_controller(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data = 0x7F;
	int res;

	res = m12mo_memory_write(sd, M12MO_MEMORY_WRITE_8BIT,
				 1, 0x13000005, &data);
	if (res < 0)
		dev_err(&client->dev, "Setup flash controller failed\n");
	return res;
}

/**
 * m12mo_wait_interrupt - Clear interrupt pending bits and unmask interrupts
 *
 * Before writing desired interrupt value the INT_FACTOR register should
 * be read to clear pending interrupts.
 */

static int m12mo_enable_interrupt(struct v4l2_subdev *sd, u8 requested_cmd){
    int ret = 0;
	u32 enable_interrupt_bit = 0;
	switch (requested_cmd) {

    case M12MO_MANUAL_FOCUS:
    case M12MO_START_AF:
	     enable_interrupt_bit = 0x02;
	     break;

	case M12MO_CAMERA_START:
	case M12MO_SINGLE_CAPTURE_MODE:
         enable_interrupt_bit = 0x08;
	     break;

    case M12MO_MONITOR_MODE_ZSL_REQUEST_CMD:
	case M12MO_PARAMETER_MODE_REQUEST_CMD:
	     enable_interrupt_bit = 0x01;
         break;

	case M12MO_START_DIGITAL_ZOOM:
	     enable_interrupt_bit = 0x04;
         break;

	case M12MO_WRITE_SHD_TABLE:
	     enable_interrupt_bit = 0x40;
		 break;
    default:
	     printk("@%s, unexpected request cmd effect: %d \n",__func__, requested_cmd);
	     return -ENODEV;
	}
//   ret = m12mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_INT_ENABLE, enable_interrupt_bit);
    return ret;
}

static u32 get_m12mo_wait_timeout_val(struct v4l2_subdev *sd, u8 requested_cmd){
    u32 timeout_val = 0;
    u32 read_val = 0xFF;

	switch (requested_cmd) {

    case M12MO_MANUAL_FOCUS:
    case M12MO_START_AF:
	     timeout_val = M12MO_INIT_TIMEOUT;
	     break;

	case M12MO_CAMERA_START:
	     timeout_val = M12MO_INIT_TIMEOUT;
	     break;

	case M12MO_SINGLE_CAPTURE_MODE:
         (void) m12mo_readb(sd, CATEGORY_ASUS, ASUS_SHUTTER_SPEED, &read_val);

         if(read_val == SHUTTER_SPEED_32) { // Means exposure time is 32s, set the timeout is 40s.
              timeout_val = 40000;
         } else {
              timeout_val = M12MO_INIT_TIMEOUT;
         }
	     break;

    case M12MO_MONITOR_MODE_ZSL_REQUEST_CMD:
         timeout_val = M12MO_INIT_TIMEOUT;
	     break;

	case M12MO_PARAMETER_MODE_REQUEST_CMD:
	     timeout_val = M12MO_PARAMETER_MODE_TIMEOUT;
         break;

	case M12MO_START_DIGITAL_ZOOM:
	     timeout_val = M12MO_INIT_TIMEOUT;
         break;

	case M12MO_WRITE_SHD_TABLE:
	     timeout_val = M12MO_INIT_TIMEOUT;
		 break;
    default:
	     printk("@%s, unexpected request cmd effect: %d \n",__func__, requested_cmd);
	     timeout_val = M12MO_INIT_TIMEOUT;
		 break;
	}
    printk("@%s, request cmd effect is %d, related timeout is %d ms\n",__func__, requested_cmd, timeout_val);
    return timeout_val;
}

int m12mo_request_cmd_effect(struct v4l2_subdev *sd, u8 requested_cmd, void* data)
{
    struct m12mo_device *dev = to_m12mo_sensor(sd);
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u32 val = 0;

	wake_lock_timeout(&dev->m12mo_wake_lock, msecs_to_jiffies(60000));
	printk("@%s %d start, requested_cmd = %d. Try to get the mutex lock. TRYing...TRYing...\n", __func__, __LINE__, requested_cmd);
	mutex_lock(&dev->m12mo_request_cmd_lock);
	dev->m12mo_request_cmd_lock_flag = 1;
    dev->requested_cmd = requested_cmd;
    printk("@%s %d start, dev->requested_cmd = %d. Successful to get the mutex lock.\n", __func__, __LINE__, dev->requested_cmd);


	switch (dev->requested_cmd) {
	case M12MO_CAMERA_START:
		/*m12mo_enable_interrupt(sd, M12MO_CAMERA_START);*/
		ret = m12mo_writeb(sd, CATEGORY_FLASHROM, FLASH_CAM_START, 0x01);
		break;

	case M12MO_PARAMETER_MODE_REQUEST_CMD:
	    m12mo_enable_interrupt(sd, M12MO_PARAMETER_MODE_REQUEST_CMD);
        ret = m12mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_SYSMODE, 0x01);
		break;

	case M12MO_MONITOR_MODE_ZSL_REQUEST_CMD:         // 8
		printk("[m12mo]@%s m12mo_enable_interrupt.\n", __func__);
//	    m12mo_enable_interrupt(sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD);
	    printk("[m12mo]@%s SYSTEM_SYSMODE.\n", __func__);
		ret = m12mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_SYSMODE, 0x02);
		break;

	case M12MO_SINGLE_CAPTURE_MODE:
        m12mo_send_still_capture_cmds(sd);
		ret = 0;
		break;

	case M12MO_START_DIGITAL_ZOOM:
		 ret = m12mo_writeb(sd, CATEGORY_MONITOR, DIGIT_ZOOM, *(u32*)data);
	     break;

	default:
        printk("@%s %d, UNLOCK m12mo_request_cmd_lock. Invalid Mode is %d, break!\n", __func__, __LINE__, requested_cmd);
//        if (dev->m12mo_request_cmd_lock_flag == 1){
		    mutex_unlock(&dev->m12mo_request_cmd_lock);
//			dev->m12mo_request_cmd_lock_flag = 0;
//        }
		return -ENODEV;
	}

#if 0
    printk("m12mo, 45678\n");
    if(dev->requested_cmd == M12MO_START_AF || dev->requested_cmd == M12MO_MANUAL_FOCUS){
	      dev->wait_irq_flag = M12MO_FOCUSING;
		  printk(KERN_INFO "%s dev->wait_irq_flag is %d \n", __func__, dev->wait_irq_flag);
	}else if (dev->requested_cmd == M12MO_START_DIGITAL_ZOOM) {
	      dev->wait_irq_flag = M12MO_DIGITAL_ZOOMING;
		  printk(KERN_INFO "%s dev->wait_irq_flag is %d \n", __func__, dev->wait_irq_flag);
	}else {
	      dev->wait_irq_flag = M12MO_NOT_LENS_RELATED_BUSYING;
		  printk(KERN_INFO "%s dev->wait_irq_flag is %d \n", __func__, dev->wait_irq_flag);
	}
    if (dev->requested_cmd == M12MO_START_AF) {
        ret = m12mo_readb(sd, CATEGORY_LENS, 0x0b, &val);
        printk("@%s Lens status is %d, requested_cmd is %d \n", __func__, val, dev->requested_cmd);
        if(val == 0) {
            printk("@%s  IRQ before wait!!! requested_cmd is %d \n", __func__, dev->requested_cmd);
            goto out_m12mo_request_cmd_effect;
        }
    }
#endif
   
	val = get_m12mo_wait_timeout_val(sd, dev->requested_cmd);
    ret = m12mo_wait_mode_change(sd, dev->requested_cmd, val);
//	}
	printk("m12mo, 87654\n");
	if(requested_cmd == M12MO_PARAMETER_MODE_REQUEST_CMD){
	    dev->m12mo_mode = M12MO_PARAMETER_MODE;

	}else if( requested_cmd == M12MO_MONITOR_MODE_ZSL_REQUEST_CMD ){
	    dev->m12mo_mode = M12MO_MONITOR_MODE_ZSL;
    }

	dev->cmd = dev->requested_cmd = M12MO_NO_CMD_REQUEST;

	dev->wait_irq_flag = M12MO_IRQ_COMMAND_AVAILABDLE;

    mutex_unlock(&dev->m12mo_request_cmd_lock);
    printk("@%s %d, UNLOCK m12mo_request_cmd_lock.\n", __func__, __LINE__);
    wake_unlock(&dev->m12mo_wake_lock);
	return ret;
}

static int is_m12mo_in_monitor_mode(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);

	if (dev->cmd == M12MO_MONITOR_MODE_PANORAMA ||
	    dev->cmd == M12MO_MONITOR_MODE_ZSL_REQUEST_CMD ||
	    dev->cmd == M12MO_MONITOR_MODE ||
	    dev->cmd == M12MO_MONITOR_MODE_HIGH_SPEED)
		return 1;

	return 0;
}

int m12mo_wait_flashrom_int(struct v4l2_subdev *sd, u8 val, u32 timeout)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret;

    dev->flashrom_mode = 1;
    printk("%s start waiting FLASHROM INT = 0x%x val = 0x%x\n", __func__, dev->flashrom_int, val);
	ret = wait_event_timeout(dev->irq_waitq, dev->flashrom_int == val, msecs_to_jiffies(timeout));
    printk("%s stop waiting FLASHROM INT = 0x%x val = 0x%x\n", __func__, dev->flashrom_int, val);
    dev->flashrom_int = 0;
    dev->flashrom_mode = 0;

	if (ret > 0) {
		return 0;
	} else if (ret == 0) {
        printk("%s timeout = %d\n", __func__, timeout);
		return -ETIMEDOUT;
	}
	return ret;
}
int m12mo_wait_mode_change(struct v4l2_subdev *sd, u8 mode, u32 timeout)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 previous_state;
    printk("m12mo, 56789  dev->cmd = 0x%x mode = 0x%x\n",dev->cmd, mode);
	ret = wait_event_timeout(dev->irq_waitq, dev->cmd == mode, msecs_to_jiffies(timeout));
	printk("m12mo, 98765  dev->cmd = 0x%x mode = 0x%x\n",dev->cmd, mode);
	if (ret > 0) {
		return 0;
	} else if (ret == 0) {
		dev_err(&client->dev, "m12mo_wait_mode_change timed out. dev->cmd = %x mode = %x, timeout is %d\n",dev->cmd, mode, timeout);

		if(m12mo_trace_log_lock != DOWNLOAD_AVAILABLE) {
              return -ETIMEDOUT;
		}
		mutex_unlock(&dev->input_lock);
        dev->lock_i2c_write_flag = 0;
		previous_state = m12mo_trace_log_lock;
		m12mo_trace_log_lock = DOWNLOADING;
		m12mo_ispd4(dev);
		m12mo_trace_log_lock = previous_state;
		return -ETIMEDOUT;
	}

	return ret;
}

int __m12mo_param_mode_set(struct v4l2_subdev *sd)
{
	int ret;
	u32 read_val;

    printk("@%s %d start\n", __func__, __LINE__);
	(void) m12mo_readb(sd, CATEGORY_SYSTEM, SYSTEM_SYSMODE, &read_val);
	if(read_val == M12MO_PARAMETER_MODE_REQUEST_CMD){
	    printk(KERN_INFO "@%s, Already in parameter mode, no need to set again. \n", __func__);
		return 0;
	}
	ret = m12mo_request_cmd_effect(sd, M12MO_PARAMETER_MODE_REQUEST_CMD, NULL);
	if (ret)
		return ret;

	return ret;
}


static int m12mo_write_Flashrom_data(struct v4l2_subdev *sd, u32 command, u32 offset, u32 * reg_val )
{
	int ret=0, res=0, timeout=5000;

	printk("@%s %d,start write calibration! command = %x , offset = %x , reg_val[0] = %x , reg_val[1] = %x , reg_val[2] = %x , reg_val[3] = %x\n"
		,__func__, __LINE__, command, offset, reg_val[0], reg_val[1], reg_val[2], reg_val[3]);

	switch(command){
	    case 0x09://1 byte write
			if(CacCalibrationStatus == false)(void)__m12mo_param_mode_set(sd);
			(void) m12mo_writew(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H, offset);
			(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, reg_val[0]);
		    break;
	    case 0x0a://2 byte write
			if(CacCalibrationStatus == false)(void)__m12mo_param_mode_set(sd);

			(void) m12mo_writew(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H, offset);
			(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, reg_val[0]);
			(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE1, reg_val[1]);

		    break;
	    case 0x0c://4 byte write
			if(CacCalibrationStatus == false)(void)__m12mo_param_mode_set(sd);
			(void) m12mo_writew(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H, offset);
			(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, reg_val[0]);
			(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE1, reg_val[1]);
			(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE2, reg_val[2]);
			(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE3, reg_val[3]);
		    break;
	    default:
		    return ret;
	}
	
	(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_MODE, command);
	(void) m12mo_writeb(sd, CATEGORY_LOGLEDFLASH, FADJ_FLASH_MODE, 0x03);

	do {
		msleep(10);
		m12mo_readb(sd, CATEGORY_LOGLEDFLASH, SFLASH_SPI_STATUS, &res);
	} while ((res != 0) && --timeout);

	if (!timeout) {
		printk("@%s %d,timeout while waiting for chip op to finish\n",__func__, __LINE__);
		return -EINVAL;
	}
	printk("@%s %d,finish write calibration!\n",__func__, __LINE__);
	if(CacCalibrationStatus == false)
    	(void) m12mo_request_cmd_effect(sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
	return ret;
}

static int m12mo_read_Flashrom_data(struct v4l2_subdev *sd, u32 command, u32 offset, u32 * reg_val )
{

	int ret=0, res=0, timeout=5000;
	printk( "read/write calibration data\n\
			arg1: 	0x01: 1 byte read\n\
				0x02: 2 byte read\n\
				0x04: 4 byte read\n\
				0x09: 1 byte write\n\
				0x0a: 2 byte write\n\
				0x0c: 4 byte write\n\
			      others: setting mode\n\
			arg2: offset[0x0000~0fff]\n\
			arg3: data0\n\
			arg4: data1\n\
			arg5: data2\n\
			arg6: data3\n");
	printk("@%s %d,start read calibration!\n",__func__, __LINE__);

    if(command == 0x01 || command == 0x02 || command == 0x04){

	    (void)__m12mo_param_mode_set(sd);
		(void) m12mo_writeb( sd, CATEGORY_LOGLEDFLASH, FADJ_FLASH_MODE, 0x01);
		do {
			msleep(10);
			m12mo_readb( sd, CATEGORY_LOGLEDFLASH, SFLASH_SPI_STATUS, &res);
		} while ((res != 0) && --timeout);

		if (!timeout) {
			printk("@%s %d,timeout while waiting for chip op to finish\n", __func__, __LINE__);
			return 0;
		}
		(void) m12mo_writew( sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H, offset);
		(void) m12mo_writeb( sd, CATEGORY_LOGLEDFLASH, FADJ_RW_MODE, command);

		switch(command){
		    case 0x01://1 byte read
				(void) m12mo_readb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, &reg_val[0]);
			    break;
		    case 0x02://2 byte read
				(void) m12mo_readb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, &reg_val[0]);
				(void) m12mo_readb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE1, &reg_val[1]);
			    break;
		    case 0x04://4 byte read
				(void) m12mo_readb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, &reg_val[0]);
				(void) m12mo_readb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE1, &reg_val[1]);
				(void) m12mo_readb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE2, &reg_val[2]);
				(void) m12mo_readb(sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE3, &reg_val[3]);
			    break;
		}
	    (void) m12mo_request_cmd_effect(sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
		printk("@%s %d,finish read calibration!\n",__func__, __LINE__);
		return ret;
    }else{
		printk("@%s %d, command %x is not read command\n", __func__, __LINE__,command);
		return -EINVAL;
    }

    return ret;
}

static int m12mo_i2c_check_status(struct v4l2_subdev *sd)
{
	int ret = 0;
	u32 value = 0;
	u8 i = 8;
	printk("HW i2c access check\n\
		 0x00 Front camera(default)\n\
		 0x01 rear camera\n\
		 0x02 Laser\n\
		 0x03 RGB\n\
		 0x04 OIS\n\
		 0x05 VCM\n\
		 0x06 Flash\n");
	ret = m12mo_readb( sd, CATEGORY_SYSTEM, HW_I2C_CHECK, &value);
	printk("@%s %d, sensor i2c status = %x \n", __func__, __LINE__, value);
	if (ret)
	{
		printk("@%s %d, fail = %d", __func__, __LINE__, ret);
        return -EINVAL;
	}
	if( value < 0 )
	{
		printk("@%s %d, value = %d", __func__, __LINE__, value);
		for( i = 0 ; i < 8 ; ++i )  i2c_check_status[i] = 1;
        return -EINVAL;
	}

	for( i = 0 ; i < 8 ; ++i )  i2c_check_status[i] = 0;
	i = 8;
	while( i2c_check_status[--i] = value % 2, value = value / 2, value > 0 && i > 0 )
	{
		printk(" %d(%d, %d) ", i2c_check_status[i], i, value);
	}
	if( i > 0 )
		i2c_check_status[--i] = value%2;
	printk("\n");
	for( i = 0 ; i < 8 ; ++i )
		printk(" %d ", i2c_check_status[i]);
	printk("\n");

	return ret;
}

static int m12mo_detect_info(struct v4l2_subdev *sd)
{
    struct m12mo_device *dev = to_m12mo_sensor(sd);
    struct m12mo_version *ver = &dev->ver;
    int res = 0;
    printk("%s\t start \n",__func__);
    m12mo_readb(sd, CATEGORY_SYSTEM, SYSTEM_CUSTOMER_CODE, &res);
    ver->customer = res;
    m12mo_readb(sd, CATEGORY_SYSTEM, SYSTEM_PROJECT_CODE, &res);
    ver->project = res;

    m12mo_i2c_check_status(sd);

    //=== For debugging, to see the ISP FW version ===//
    m12mo_readw(sd, CATEGORY_SYSTEM, SYSTEM_VER_FIRMWARE, &res);
    ver->firmware = res;
    readFwForceUpdateFlag(dev, &ver->fw_auto_flash_flag);
    readDitVersion(dev, &ver->DIT_firmware);
    printk("%s fw_auto_flash_flag = 0x%x\n", __func__, dev->ver.fw_auto_flash_flag);
    printk("%s m12mo version : %04x-%04x\n", __func__, dev->ver.firmware, dev->ver.DIT_firmware);
    printk("%s Customer/Project[0x%02x/0x%02x]\n", __func__, dev->ver.customer, dev->ver.project);
    return 0;
}

static int __m12mo_fw_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
    printk("@%s %d start\n", __func__, __LINE__);
	/*
	 * Correct the pll value before fw start
	 */
#if 0	 // don't use PLL
    if(!PREVIEW_TEST_PATTERN){
        ret = m12mo_update_pll_setting(sd);
        if (ret < 0)
            return ret;

        ret = m12mo_setup_flash_controller(sd);
        if (ret < 0)
            return ret;
    }
#endif
	ret = m12mo_request_cmd_effect(sd, M12MO_CAMERA_START, NULL);
	if (ret)
		return ret;

	dev_info(&client->dev, "ISP Booted Successfully\n");

	return 0;
}

static int m12mo_fw_start(struct v4l2_subdev *sd, u32 val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret;
    printk("@%s %d start\n", __func__, __LINE__);

	mutex_lock(&dev->input_lock);

	ret = __m12mo_fw_start(sd);

	mutex_unlock(&dev->input_lock);

	return ret;
}

static int m12mo_set_af_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret = 0;
	int polling_times = 10;
	u32 read_back = 0;

    printk("@%s value %d start\n", __func__, val);

    if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set AF during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}

	//======= Hard code here for polling status by Derek says so.
	(void) m12mo_readb(sd, CATEGORY_ASUS,ASUS_FOCUS_MODE,&read_back);
	if(read_back != val){
		(void) m12mo_writeb(sd, CATEGORY_ASUS,ASUS_FOCUS_MODE,val);
		for(polling_times = 10; polling_times > 0; --polling_times){
			(void) m12mo_readb(sd, CATEGORY_ASUS,ASUS_FOCUS_MODE,&read_back);
			if(read_back == val){
				printk("@%s %d success !\n", __func__, __LINE__);
				return 0;
			}
		}
	}else{
	         printk("@%s, already in AF normal mode, no need to set! \n", __func__);
			 return 0;
	}
	printk("@%s %d fail !\n", __func__, __LINE__);
	return -EINVAL;
#if 0
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int id = M12MO_GET_FOCUS_MODE(dev->fw_type);

	u32 cur_af_mode;

	switch (val) {
	case EXT_ISP_FOCUS_MODE_NORMAL:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_normal;
		break;
	case EXT_ISP_FOCUS_MODE_MACRO:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_macro;
		break;
	case EXT_ISP_FOCUS_MODE_TOUCH_AF:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_touch;
		break;
	case EXT_ISP_FOCUS_MODE_PREVIEW_CAF:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_preview_caf;
		break;
	case EXT_ISP_FOCUS_MODE_MOVIE_CAF:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_movie_caf;
		break;
	case EXT_ISP_FOCUS_MODE_FACE_CAF:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_face_caf;
		break;
	case EXT_ISP_FOCUS_MODE_TOUCH_MACRO:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_touch_macro;
		break;
	case EXT_ISP_FOCUS_MODE_TOUCH_CAF:
		dev->monitor_params.af_mode = m12mo_af_parameters[id].af_touch_caf;
		break;
	default:
		return -EINVAL;
	}

	if (!is_m12mo_in_monitor_mode(sd))
		return ret;

	ret = m12mo_read(sd, 1, CATEGORY_LENS,
			m12mo_af_parameters[id].af_mode,
			&cur_af_mode);

	/*
	 * If af_mode has changed as expected already
	 * no need to set any more
	 */
	if (dev->monitor_params.af_mode == cur_af_mode)
		return ret;

	dev_info(&client->dev, "%s: In monitor mode, set AF mode to %d",
		 __func__, dev->monitor_params.af_mode);

	/* We are in monitor mode already, */
	/* af_mode can be applied immediately */
	ret = m12mo_writeb(sd, CATEGORY_LENS,
			m12mo_af_parameters[id].af_mode,
			dev->monitor_params.af_mode);
#endif
	return ret;
}

static int m12mo_set_af_execution(struct v4l2_subdev *sd, s32 val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int id = M12MO_GET_FOCUS_MODE(dev->fw_type);
	int ret = 0;

	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set AF during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
	switch (val) {
	case EXT_ISP_FOCUS_STOP:
		dev->monitor_params.exe_mode = m12mo_af_parameters[id].af_stop;
		break;
	case EXT_ISP_FOCUS_SEARCH:
		dev->monitor_params.exe_mode = m12mo_af_parameters[id].af_search;
		break;
	case EXT_ISP_PAN_FOCUSING:
		dev->monitor_params.exe_mode = m12mo_af_parameters[id].af_pan_focusing;
		break;
	default:
		return -EINVAL;
	}

	printk(KERN_INFO "%s: In monitor mode, set AF exe_mode to %d",
			 __func__, dev->monitor_params.exe_mode);
#if 0
		/* We are in monitor mode already, */
		/* exe_mode can be applied immediately */
		ret = m12mo_writeb(sd, CATEGORY_LENS,
				   m12mo_af_parameters[id].af_execution,
				   dev->monitor_params.exe_mode);
#else
	if(val == EXT_ISP_FOCUS_SEARCH){
        ret = m12mo_request_cmd_effect(sd, M12MO_START_AF, NULL);
		if (ret < 0)
			return ret;
#endif
	}
	return ret;
}

static int m12mo_set_af_position_x(struct v4l2_subdev *sd, unsigned int x)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int id = M12MO_GET_FOCUS_MODE(dev->fw_type);
	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
    printk("@%s: set DIT AF x = %d\n", __func__, x);

	dev->monitor_params.af_touch_posx = x;

    /* Set X Position to DIT*/
    ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_TOUCH_ROI_LEFT_UPPER_X, dev->monitor_params.af_touch_posx);
	if (ret){
		dev_err(&client->dev, "set DIT AF position x failed %d\n", ret);
        return ret;
    }

	if (is_m12mo_in_monitor_mode(sd)) {
		dev_info(&client->dev,
			 "%s: In monitor mode, set AF touch X pos to 0x%x",
			 __func__, dev->monitor_params.af_touch_posx);

		/* Set X Position */
		ret = m12mo_writew(sd, CATEGORY_LENS,
				   m12mo_af_parameters[id].af_touch_posx,
				   dev->monitor_params.af_touch_posx);
	}

	if (ret)
		dev_err(&client->dev, "AutoFocus position x failed %d\n", ret);

	return ret;
}

static int m12mo_set_af_position_y(struct v4l2_subdev *sd, unsigned int y)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int id = M12MO_GET_FOCUS_MODE(dev->fw_type);
	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
    printk("@%s: set DIT AF y = %d\n", __func__, y);

	dev->monitor_params.af_touch_posy = y;

    /* Set Y Position to DIT*/
    ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_TOUCH_ROI_LEFT_UPPER_Y, dev->monitor_params.af_touch_posy);
	if (ret){
		dev_err(&client->dev, "set DIT AF position y failed %d\n", ret);
        return ret;
    }

	if (is_m12mo_in_monitor_mode(sd)) {
		dev_info(&client->dev,
			 "%s: In monitor mode, set AF touch Y pos to 0x%x",
			 __func__, dev->monitor_params.af_touch_posy);

		/* Set Y Position */
		ret = m12mo_writew(sd, CATEGORY_LENS,
				   m12mo_af_parameters[id].af_touch_posy,
				   dev->monitor_params.af_touch_posy);
	}

	if (ret)
		dev_err(&client->dev, "AutoFocus position y failed %d\n", ret);

	return ret;
}

static int m12mo_set_af_width(struct v4l2_subdev *sd, unsigned int w)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m12mo_device *dev = to_m12mo_sensor(sd);
	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
    printk("@%s: set DIT AF ROI width = %d\n", __func__, w);
    dev->monitor_params.af_touch_width = w;

    /* Set AF ROI width to DIT */
    ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_TOUCH_WIDTH, dev->monitor_params.af_touch_width);

    if (ret)
        dev_err(&client->dev, "set DIT AF ROI width failed %d\n", ret);

    return ret;
}

static int m12mo_set_af_height(struct v4l2_subdev *sd, unsigned int h)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m12mo_device *dev = to_m12mo_sensor(sd);
	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
    printk("@%s: set DIT AF ROI height = %d\n", __func__, h);
    dev->monitor_params.af_touch_height = h;

    /* Set AF ROI height to DIT */
    ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_TOUCH_HEIGHT, dev->monitor_params.af_touch_height);

    if (ret)
        dev_err(&client->dev, "set DIT AF ROI height failed %d\n", ret);

    return ret;
}

static int m12mo_set_ae_position_x(struct v4l2_subdev *sd, unsigned int x)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m12mo_device *dev = to_m12mo_sensor(sd);
	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
    printk("@%s: set DIT AE x = %d\n", __func__, x);
    dev->monitor_params.ae_touch_posx = x;

    /* Set AE X Position to DIT */
    ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_TOUCH_POSITION_X, dev->monitor_params.ae_touch_posx);

    if (ret)
        dev_err(&client->dev, "set DIT AE position x failed %d\n", ret);

    return ret;
}

static int m12mo_set_ae_position_y(struct v4l2_subdev *sd, unsigned int y)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m12mo_device *dev = to_m12mo_sensor(sd);

	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}

    dev->monitor_params.ae_touch_posy = y;

    /* Set AE Y Position to DIT */
    ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_TOUCH_POSITION_Y, dev->monitor_params.ae_touch_posy);

    if (ret)
        dev_err(&client->dev, "set DIT AE position y failed %d\n", ret);

    return ret;
}

/* Because of different m12mo firmwares and parameter values, transform
   the values to the macro definition */
static u32 m12mo_af_parameter_transform(struct v4l2_subdev *sd, u32 val)
{
	u32 ret = 0xffffffff;
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int id = M12MO_GET_FOCUS_MODE(dev->fw_type);

	if (val == m12mo_af_parameters[id].caf_status_focusing) {
		ret = CAF_STATUS_FOCUSING;
	} else if (val == m12mo_af_parameters[id].caf_status_success) {
		ret = CAF_STATUS_SUCCESS;
	} else if (val == m12mo_af_parameters[id].caf_status_fail) {
		ret = CAF_STATUS_FAIL;
	} else if (val == m12mo_af_parameters[id].caf_status_restart_check) {
		ret = CAF_STATUS_RESTART_CHECK;
	} else if (val == m12mo_af_parameters[id].af_status_invalid) {
		ret = AF_STATUS_INVALID;
	} else if (val == m12mo_af_parameters[id].af_status_focusing) {
		ret = AF_STATUS_FOCUSING;
	} else if (val == m12mo_af_parameters[id].af_status_success) {
		ret = AF_STATUS_SUCCESS;
	} else if (val == m12mo_af_parameters[id].af_status_fail) {
		ret = AF_STATUS_FAIL;
	} else if (val == m12mo_af_parameters[id].af_normal) {
		ret = AF_NORMAL;
	} else if (val == m12mo_af_parameters[id].af_macro) {
		ret = AF_MACRO;
	} else if (val == m12mo_af_parameters[id].af_touch) {
		ret = AF_TOUCH;
	} else if (val == m12mo_af_parameters[id].af_preview_caf) {
		ret = AF_PREVIEW_CAF;
	} else if (val == m12mo_af_parameters[id].af_movie_caf) {
		ret = AF_MOVIE_CAF;
	} else if (val == m12mo_af_parameters[id].af_face_caf) {
		ret = AF_FACE_CAF;
	} else if (val == m12mo_af_parameters[id].af_touch_macro) {
		ret = AF_TOUCH_MACRO;
	} else if (val == m12mo_af_parameters[id].af_touch_caf) {
		ret = AF_TOUCH_CAF;
	}

	return ret;
}

static int m12mo_get_caf_status(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 af_result;
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int id = M12MO_GET_FOCUS_MODE(dev->fw_type);

	ret = m12mo_read(sd, 1, CATEGORY_LENS,
			m12mo_af_parameters[id].af_result, &af_result);
	if (ret)
		return ret;

	af_result = m12mo_af_parameter_transform(sd, af_result);

	switch (af_result) {
	case CAF_STATUS_FOCUSING:
		*status = EXT_ISP_CAF_STATUS_FOCUSING;
		break;
	case CAF_STATUS_SUCCESS:
		*status = EXT_ISP_CAF_STATUS_SUCCESS;
		break;
	case CAF_STATUS_FAIL:
		*status = EXT_ISP_CAF_STATUS_FAIL;
		break;
	case CAF_STATUS_RESTART_CHECK:
		*status = EXT_ISP_CAF_RESTART_CHECK;
		break;
	default:
		return -EINVAL;
	}

	return ret;
}


static int m12mo_set_hal_i2c_write(struct v4l2_subdev *sd, unsigned int *status)
{
//  struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 reg_val;
	u8 cat, reg_addr;
	int ret;

	reg_val  = *status & 0xFF;
	reg_addr = (*status >> 8) & 0xFF;
	cat      = (*status >> 16) & 0xFF;

	ret = m12mo_writeb(sd, cat, reg_addr, reg_val);
	return ret;
}

static int m12mo_get_hal_i2c_read(struct v4l2_subdev *sd, unsigned int *status)
{
//  struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 reg_val;
	u8 cat, reg_addr;
	int ret;

	reg_addr = (*status >> 8) & 0xFF;
	cat      = (*status >> 16) & 0xFF;

	ret = m12mo_readb(sd, cat, reg_addr, &reg_val);

	if(!ret){
	     *status = reg_val;
	} else {
	     *status = EXT_ISP_CID_M10MO_I2C_ONE_BYTE_READ_INVALID;
	}

	return ret;
}

static int m12mo_get_asus_flash_needed(struct v4l2_subdev *sd, unsigned int *status)
{
//  struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 val;
	int ret;
	ret = m12mo_readb(sd, CATEGORY_ASUS, ASUS_FLASH_NEEDED, &val);
	*status = val;

	return ret;
}

static int m12mo_get_irq_status(struct v4l2_subdev *sd, unsigned int *status)
{
    struct m12mo_device *dev = to_m12mo_sensor(sd);

    if(dev->wait_irq_flag == M12MO_IRQ_COMMAND_AVAILABDLE){
	      *status = EXT_ISP_M10MO_IRQ_IDLE;
    } else {
	       *status = EXT_ISP_M10MO_IRQ_BUSY;
	}
	return 0;
}

static int m12mo_set_battery_capacity(struct v4l2_subdev *sd, unsigned int *status)
{
//  struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 val;
	int ret;
    val = *status;

	ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_BATTERY_CAPACITY, val);
	return ret;
}

static int m12mo_get_LV(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;

	ret = m12mo_readw(sd, CATEGORY_ASUS, ASUS_LV, &val);
	*status = val;
	return ret;
}

static int m12mo_set_R_GAIN(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;
    val = *status;

	ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_GAIN2_RGAIN, val);
	return ret;
}

static int m12mo_get_R_GAIN(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;

	ret = m12mo_readw(sd, CATEGORY_ASUS, ASUS_GAIN1_RGAIN, &val);
	*status = val;
	return ret;
}

static int m12mo_set_B_GAIN(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;
    val = *status;

	ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_GAIN2_BGAIN, val);
	return ret;
}

static int m12mo_get_B_GAIN(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;

	ret = m12mo_readw(sd, CATEGORY_ASUS, ASUS_GAIN1_BGAIN, &val);
	*status = val;
	return ret;
}

static int m12mo_set_3A_lock(struct v4l2_subdev *sd, unsigned int *status)
{
//  struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 val;
	int ret;
    val = *status;

	ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_3A_LOCK, val);
	return ret;
}

static int m12mo_set_g_sensor_x(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;
    val = *status;

    ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_G_SENSOR_X, val);
	return ret;
}

static int m12mo_set_g_sensor_y(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;
    val = *status;

	ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_G_SENSOR_Y, val);
	return ret;
}

static int m12mo_set_g_sensor_z(struct v4l2_subdev *sd, unsigned int *status)
{
	u32 val;
	int ret;
    val = *status;

	ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_G_SENSOR_Z, val);
	return ret;
}

static int m12mo_get_af_result(struct v4l2_subdev *sd, unsigned int *status)
{
//  struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 read_val = 0;
	int ret;

	ret = m12mo_readb(sd, CATEGORY_ASUS, ASUS_FOCUS_RESULT, &read_val);

	if (read_val == AF_SUCCESS) {
	    *status = EXT_ISP_AF_RESULT_SUCCESS;

	} else if (read_val == AF_SUCCESS) {
	    *status = EXT_ISP_AF_RESULT_FAIL;

	} else if (read_val == AF_INVALID) {
	    *status = EXT_ISP_AF_RESULT_INVALID;

	} else {
	     *status = EXT_ISP_AF_RESULT_INVALID;
		 printk(KERN_ERR "@%s, Error, value (%d) not defined! \n", __func__, read_val);
		 return -EINVAL;
	}
	return ret;
}

static int m12mo_get_af_status(struct v4l2_subdev *sd, unsigned int *status)
{
    struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 read_val = 0;

    (void) m12mo_readb(sd, CATEGORY_ASUS, ASUS_FOCUS_RESULT, &read_val);
	if(read_val == AF_PRE_DONE) {
	      *status = EXT_ISP_AF_STATUS_PRE_DONE;
		  return 0;
	}

	if(dev->wait_irq_flag == M12MO_FOCUSING){
	      *status = EXT_ISP_AF_STATUS_FOCUSING;
	}else if (dev->wait_irq_flag == M12MO_IRQ_COMMAND_AVAILABDLE){
	       *status = EXT_ISP_AF_STATUS_SUCCESS;
	}else{
	       *status = EXT_ISP_LENS_STATUS_INVALID;
	}
	return 0;
}

static int m12mo_get_digital_zoom_status(struct v4l2_subdev *sd, unsigned int *status)
{
    struct m12mo_device *dev = to_m12mo_sensor(sd);

	if(dev->wait_irq_flag == M12MO_DIGITAL_ZOOMING){
	      *status = EXT_ISP_DIGITAL_ZOOM_STATUS_ZOOMING;
	}else if (dev->wait_irq_flag == M12MO_IRQ_COMMAND_AVAILABDLE){
	       *status = EXT_ISP_DIGITAL_ZOOM_STATUS_AVAILABLE;
	}else{
	       *status = EXT_ISP_LENS_STATUS_INVALID;
	}
	return 0;

}

/* Adding this as requested by HAL. Can be removed if HAL is saving af mode */
static int m12mo_get_af_mode(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 af_mode;

	ret =  m12mo_readb(sd, CATEGORY_ASUS,ASUS_FOCUS_MODE,&af_mode);
	if (ret)
		return ret;
	*status = af_mode;
	return ret;
#if 0
	af_mode = m12mo_af_parameter_transform(sd, af_mode);

	switch (af_mode) {
	case AF_NORMAL:
		*status = EXT_ISP_FOCUS_MODE_NORMAL;
		break;
	case AF_MACRO:
		*status = EXT_ISP_FOCUS_MODE_MACRO;
		break;
	case AF_TOUCH:
		*status = EXT_ISP_FOCUS_MODE_TOUCH_AF;
		break;
	case AF_PREVIEW_CAF:
		*status = EXT_ISP_FOCUS_MODE_PREVIEW_CAF;
		break;
	case AF_MOVIE_CAF:
		*status = EXT_ISP_FOCUS_MODE_MOVIE_CAF;
		break;
	case AF_FACE_CAF:
		*status = EXT_ISP_FOCUS_MODE_FACE_CAF;
		break;
	case AF_TOUCH_MACRO:
		*status = EXT_ISP_FOCUS_MODE_TOUCH_MACRO;
		break;
	case AF_TOUCH_CAF:
		*status = EXT_ISP_FOCUS_MODE_TOUCH_CAF;
		break;
	default:
		return -EINVAL;
	}

	return ret;
#endif
}

static int m12mo_get_exposure_time_numerator(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 numerator;
	ret = m12mo_readl(sd, CATEGORY_EXIF,EXIF_INFO_EXPTIME_NU, &numerator);
	*status = numerator;
	return ret;
}

static int m12mo_get_exposure_time_denominator(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 denominator;
	ret = m12mo_readl(sd, CATEGORY_EXIF,EXIF_INFO_EXPTIME_DE, &denominator);
	*status = denominator;
	return ret;
}

static int m12mo_get_preview_exposure_time_numerator(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 numerator;
	ret = m12mo_readl(sd, CATEGORY_ASUS,ASUS_PREVIEW_INFO_EXP_NUMERATOR, &numerator);
	*status = numerator;
	return ret;
}

static int m12mo_get_preview_exposure_time_denominator(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 denominator;
	ret = m12mo_readl(sd, CATEGORY_ASUS,ASUS_PREVIEW_INFO_EXP_DENOMINATOR, &denominator);
	*status = denominator;
	return ret;
}

static int m12mo_get_preview_iso(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 denominator;
	ret = m12mo_readw(sd, CATEGORY_ASUS,ASUS_PREVIEW_ISO, &denominator);
	*status = denominator;
	return ret;
}

static int m12mo_set_digital_zoom_position(struct v4l2_subdev *sd, u32 data)
{
    int ret = 0;
	u32 val = 0;
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	pr_info("%s, data is %d \n", __func__, data);

    if(data < 1 || data > 31 ){
	   pr_info("m12mo, wrong digital zoom data is %d \n", data);
	   return -EINVAL;;
	}

	if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set digital-zoom during M12MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}

	ret = m12mo_readb(sd, CATEGORY_MONITOR, DIGIT_ZOOM, &val);
	if(val == data){
	    pr_info("m12mo, get digital-zoom step is %d, set digital-zoom data is %d. No need to set\n", val, data);
	    return 0;
	}

    ret = m12mo_request_cmd_effect(sd, M12MO_START_DIGITAL_ZOOM, &data);
    if (ret)
        return ret;

    return 0;
}

//extern int set_torch_on_via_SOC(bool enable);
static int m12mo_set_flash_mode(struct v4l2_subdev *sd, unsigned int val)
{
	int ret = 0;
#if 1
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* by default force the torch off, value depends on incoming flash mode */
	dev->monitor_params.torch = LED_TORCH_OFF;

	switch (val) {
	case EXT_ISP_FLASH_MODE_OFF:
		dev->monitor_params.flash_mode = FLASH_MODE_OFF;
		if(dev->m12mo_mode == M12MO_PARAMETER_MODE) {
		    printk("@%s %d, parameter mode.\n", __func__, __LINE__);
//		    set_torch_on_via_SOC(0);
		}
		break;
	case EXT_ISP_FLASH_MODE_ON:
		dev->monitor_params.flash_mode = FLASH_MODE_ON;
		break;
	case EXT_ISP_FLASH_MODE_AUTO:
		dev->monitor_params.flash_mode = FLASH_MODE_AUTO;
		break;
	case EXT_ISP_LED_TORCH_OFF:
		dev->monitor_params.flash_mode = LED_TORCH_OFF;
		dev->monitor_params.torch = LED_TORCH_OFF;
		if(dev->m12mo_mode == M12MO_PARAMETER_MODE) {
		    printk("@%s %d, parameter mode.\n", __func__, __LINE__);
//		    set_torch_on_via_SOC(0);
		}
		break;
	case EXT_ISP_LED_TORCH_ON:
		dev->monitor_params.flash_mode = LED_TORCH_ON;
		dev->monitor_params.torch = LED_TORCH_ON;
		if(dev->m12mo_mode == M12MO_PARAMETER_MODE) {
		    printk("@%s %d, parameter mode.\n", __func__, __LINE__);
//		    set_torch_on_via_SOC(1);
		}
		break;
	default:
		return -EINVAL;
	}
#if 0 //It seems no need anymore.
	/* Only apply setting if we are in monitor mode */
	if (!is_m12mo_in_monitor_mode(sd))
		return ret;
#endif
	dev_info(&client->dev, "%s: In monitor mode, set flash mode to %d",
		 __func__, dev->monitor_params.flash_mode);

	/* TODO get current flash mode, and apply new setting only when needed? */

	ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_FLASH,dev->monitor_params.flash_mode);

#endif
	return ret;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret;

	pr_info("%s\n", __func__);

	if (dev->pdata->common.power_ctrl) {
		ret = dev->pdata->common.power_ctrl(sd, 1);
		if (ret)
			goto fail_power_ctrl;
	}

	if (dev->pdata->common.flisclk_ctrl) {
		ret = dev->pdata->common.flisclk_ctrl(sd, 1);
		if (ret)
			goto fail_clk_off;
	}

	/**ISP RESET**/
	ret = dev->pdata->common.gpio_ctrl(sd, 1);
	if (ret)
		goto fail_power_off;
	return 0;

fail_power_off:
	dev->pdata->common.gpio_ctrl(sd, 0);
fail_clk_off:
	if (dev->pdata->common.flisclk_ctrl)
		ret = dev->pdata->common.flisclk_ctrl(sd, 0);
fail_power_ctrl:
	if (dev->pdata->common.power_ctrl)
		ret = dev->pdata->common.power_ctrl(sd, 0);
	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	pr_info("%s\n", __func__);

	ret = dev->pdata->common.gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed\n");

	/* Even if the first one fails we still want to turn clock off */
	if (dev->pdata->common.flisclk_ctrl) {
		ret = dev->pdata->common.flisclk_ctrl(sd, 0);
		if (ret)
			dev_err(&client->dev, "stop clock failed\n");
	}

	if (dev->pdata->common.power_ctrl) {
		ret = dev->pdata->common.power_ctrl(sd, 0);
		if (ret)
			dev_err(&client->dev, "power off fail\n");
	}
	return ret;
}

static int __m12mo_bootrom_mode_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_adapter *adapter = client->adapter;
	u32 dummy;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;
	/* Dummy read to verify I2C functionality */
	ret = m12mo_readl(sd, CATEGORY_FLASHROM,  REG_FLASH_ADD, &dummy);
	printk(KERN_INFO "__m12mo_bootrom_mode_start, dummy is %d \n", dummy);
	if (ret < 0)
		dev_err(&client->dev, "Dummy I2C access fails\n");

	return ret;
}

static int __m12mo_s_power(struct v4l2_subdev *sd, int on, bool fw_update_mode)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret;

    struct i2c_client *client = v4l2_get_subdevdata(sd);

	pr_info("%s: on: %d, fw_update_mode is %d\n", __func__, on, fw_update_mode);
    dev->m12mo_mode = M12MO_PARAMETER_MODE;
	if (dev->power == on)
		return 0;

	dev->stream = 0;
	num_LLS = 0;
	m12mo_capture_pre_flag = 0;
	dev->lock_i2c_write_flag = 0;
    dev->flashrom_mode = 1;
    dev->flashrom_int = 0;
	isCaptureMode = false;
    dev->capture_mode = M12MO_CAPTURE_MODE_ZSL_NORMAL;
	//__m12mo_reset_irq_wait_status();//(dev, &lens_work);
//	if (dev->m12mo_request_cmd_lock_flag == 1){
	mutex_unlock(&dev->m12mo_request_cmd_lock);
	printk("@%s %d, UNLOCK m12mo_request_cmd_lock.\n", __func__, __LINE__);
//		dev->m12mo_request_cmd_lock_flag = 0;
//    }

	if (on) {
	    if(dev->disable_irq_flag){
	        enable_irq(client->irq);
			dev->disable_irq_flag = 0;
		}
		dev->cmd = M12MO_POWERING_ON;
		ret = power_up(sd);
		if (ret)
			return ret;
		dev->power = 1;
        dev->flashrom_mode = 0;

		/*ret = __m12mo_bootrom_mode_start(sd);*/
		/*if (ret)*/
			/*goto startup_failure;*/

		if (!fw_update_mode) {

            ret = __m12mo_fw_start(sd);
            if (ret)
                goto startup_failure;

            //===  Category:0x0D, Byte: 0x26, line blanking (Upper byte) (0x00～0x0F)  ===//
            //===  Category:0x0D, Byte: 0x27, line blanking (Lower byte) (0x00～0xFF)  ===//
            /*(void) m12mo_writeb(sd, CATEGORY_TEST, 0x26, 0x01);*/
            /*(void) m12mo_writeb(sd, CATEGORY_TEST, 0x27, 0xFF);*/
#ifdef CONFIG_ASUS_FACTORY_MODE
            //=== Only for factory, tell m12mo in fac image now! ===//
            /*(void) m12mo_writeb(sd, CATEGORY_ASUS, 0x02, 0x01);*/
#endif
        }
	} else {
	    if (!fw_update_mode) {
		    ret = __m12mo_param_mode_set(sd);
//            set_torch_on_via_SOC(0);
			if (ret)
				goto startup_failure;

			/*ret = m12mo_set_lens_position(sd);*/
			/*if (ret)*/
				/*goto startup_failure;*/

	    }
		asus_camera_flag = 0;
		wake_unlock(&dev->m12mo_wake_lock);
		disable_irq(client->irq);
		dev->disable_irq_flag = 1;
		ret = power_down(sd);
		dev->power = 0;
	}

	return ret;

startup_failure:
    printk("%s startup_failure\n", __func__);
	power_down(sd);
	dev->power = 0;
	return ret;
}

int m12mo_set_panorama_monitor(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u32 val;

	dev_info(&client->dev,
		"%s mode: %d Width: %d, height: %d, cmd: 0x%x vdis: %d\n",
		__func__, dev->cmd, dev->curr_res_table[dev->fmt_idx].width,
		dev->curr_res_table[dev->fmt_idx].height,
		dev->curr_res_table[dev->fmt_idx].command,
		(int)dev->curr_res_table[dev->fmt_idx].vdis);

	/* Check if m12mo already streaming @ required resolution */
	ret = m12mo_readb(sd, CATEGORY_PARAM,  PARAM_MON_SIZE, &val);
	if (ret)
		goto out;

	/* If mode is monitor mode and size same, do not configure again*/
	if (dev->cmd == M12MO_MONITOR_MODE_PANORAMA &&
		val == dev->curr_res_table[dev->fmt_idx].command) {
		dev_info(&client->dev,
			"%s Already streaming with required size\n", __func__);
		return 0;
	}

	if (dev->cmd != M12MO_CAMERA_START &&
		dev->cmd != M12MO_PARAMETER_MODE_REQUEST_CMD) {
		/* Already in panorama mode. So swith to parameter mode */
		ret = __m12mo_param_mode_set(sd);
		if (ret)
			goto out;
	}

	/* Change the Monitor Size */
	ret = m12mo_write(sd, 1, CATEGORY_PARAM, PARAM_MON_SIZE,
			dev->curr_res_table[dev->fmt_idx].command);
	if (ret)
		goto out;

	/* Set Panorama mode */
	ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, CAP_MODE,
            CAP_MODE_MOVIE);
	if (ret)
		goto out;

	/* Setting output to NV12/NV21. */
	ret = m12mo_writeb(sd, CATEGORY_PARAM, OUTPUT_FMT_SELECT,
			OUTPUT_FMT_SELECT_NV12NV21);
	if (ret)
		goto out;

	/* Select either NV12 or NV21 based on the format set from user space */
	val = dev->format.code == V4L2_MBUS_FMT_CUSTOM_NV21 ?
		CHOOSE_NV12NV21_FMT_NV21 : CHOOSE_NV12NV21_FMT_NV12;
	ret = m12mo_writeb(sd, CATEGORY_PARAM, CHOOSE_NV12NV21_FMT, val);
	if (ret)
		goto out;

	/* Enable metadata (the command sequence PDF-example) */
	ret = m12mo_writeb(sd, CATEGORY_PARAM, MON_METADATA_SUPPORT_CTRL,
			MON_METADATA_SUPPORT_CTRL_EN);
	if (ret)
		goto out;

	/* Enable interrupt signal */
	ret = m12mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_INT_ENABLE, 0x01);
	if (ret)
		goto out;

	/* Go to Panorama Monitor mode */
	ret = m12mo_request_cmd_effect(sd, M12MO_MONITOR_MODE_PANORAMA, NULL);
	if (ret)
		goto out;

	ret = m12mo_wait_mode_change(sd, M12MO_MONITOR_MODE_PANORAMA,
				M12MO_INIT_TIMEOUT);
	if (ret < 0)
		goto out;

	return 0;
out:
	dev_err(&client->dev, "m12mo_set_panorama_monitor failed %d\n", ret);
	return ret;
}

int m12mo_set_zsl_monitor(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
//	int mode = M12MO_GET_RESOLUTION_MODE(dev->fw_type);
    const struct m12mo_resolution *capture_res =
			resolutions[M12MO_MODE_CAPTURE_INDEX];
	int ret;
//	u32 read_val = 1;
//	u32 polling_times = 10;
    /*u32 fmt;*/

	previous_preview_cmd = dev->curr_res_table[dev->fmt_idx].command;
	dev_info(&client->dev,
		"%s dev->cmd: %d width: %d, height: %d, cmd: 0x%x vdis: %d\n",
		__func__, dev->cmd, dev->curr_res_table[dev->fmt_idx].width,
		dev->curr_res_table[dev->fmt_idx].height,
		dev->curr_res_table[dev->fmt_idx].command,
		(int)dev->curr_res_table[dev->fmt_idx].vdis);

	dev_info(&client->dev, "%s capture width: %d, height: %d, cmd: 0x%x\n",
		__func__, capture_res[dev->capture_res_idx].width,
		capture_res[dev->capture_res_idx].height,
		capture_res[dev->capture_res_idx].command);
/*
	if (dev->cmd != M12MO_CAMERA_START &&
		dev->cmd != M12MO_PARAMETER_MODE_REQUEST_CMD) {
		//
		// At this stage means we are already at ZSL. So switch to
		// param mode first and reset all the parameters.
		//

		ret = __m12mo_param_mode_set(sd);
		if (ret)
			goto out;
	}
*/

#if 0
    //=== For  monitor size 1920x1440, add line-blanking to 0x122, other sizes to 0x1FF ===//
    //=== to avoid buffer-overflow from Intel ISP.                                      ===//
    if(dev->curr_res_table[dev->fmt_idx].command == 0x52) {
         (void) m12mo_writeb(sd, CATEGORY_TEST, 0x26, 0x01);
		 (void) m12mo_writeb(sd, CATEGORY_TEST, 0x27, 0x22);
    } else if(dev->curr_res_table[dev->fmt_idx].command == 0x18
	           || dev->curr_res_table[dev->fmt_idx].command == 0x5E) {
         (void) m12mo_writeb(sd, CATEGORY_TEST, 0x26, 0x02);
		 (void) m12mo_writeb(sd, CATEGORY_TEST, 0x27, 0x58);
    } else {
         (void) m12mo_writeb(sd, CATEGORY_TEST, 0x26, 0x1);
	     (void) m12mo_writeb(sd, CATEGORY_TEST, 0x27, 0xFF);
    }
 #endif
    /* Change the Monitor Size */
	if(dev->shot_mode == CAP_HILIGHT && dev->run_mode == CI_MODE_PREVIEW) {
	     if(dev->curr_res_table[dev->fmt_idx].command == 0x28) {
		        (void) m12mo_writeb(sd, CATEGORY_PARAM, PARAM_MON_SIZE, 0x53);
		 }else if(dev->curr_res_table[dev->fmt_idx].command == 0x52) {
		        (void) m12mo_writeb(sd, CATEGORY_PARAM, PARAM_MON_SIZE, 0x52);
		 }
	} else { //=== Binning mode only provided in preview, didn't provided with recording. ===//
        ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_MON_SIZE,
                dev->curr_res_table[dev->fmt_idx].command);
        if (capture_res[dev->capture_res_idx].width == 6144 && capture_res[dev->capture_res_idx].height == 4608) {
            /*Rear Full Raw*/
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_1, 0x18);
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_2, 0x00);
        } else if (capture_res[dev->capture_res_idx].width == 3200 && capture_res[dev->capture_res_idx].height == 2400) {
            /*Rear Binning Raw*/
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_1, 0x0c);
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_2, 0x80);
        } else if (capture_res[dev->capture_res_idx].width == 3712 && capture_res[dev->capture_res_idx].height == 2784) {
            /*Front Full Raw*/
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_1, 0x0e);
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_2, 0x80);
        } else if (capture_res[dev->capture_res_idx].width == 1920 && capture_res[dev->capture_res_idx].height == 1440) {
            /*Frant Binning Raw*/
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_1, 0x07);
            ret = m12mo_writeb(sd, CATEGORY_PARAM, PARAM_SEND_UNIT_2, 0x80);
        }
        ret = m12mo_writeb(sd, CATEGORY_CAPTURE_PARAM, CAPP_MAIN_IMAGE_SIZE,
                capture_res[dev->capture_res_idx].command);
        if (ret)
            goto out;
    }

	/* Set ZSL mode */
	ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, CAP_MODE, CAP_MODE_ZSL);
	if (ret)
		goto out;

#if 0
	ret = m12mo_write(sd, 1, CATEGORY_PARAM, MOVIE_MODE, 0x00);		// 	#define MOVIE_MODE			0x3c
	if (ret)
		goto out;
#endif

	/* Go to ZSL Monitor mode */
	ret = m12mo_request_cmd_effect(sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
	if (ret)
		goto out;

#if 0
	ret = m12mo_writeb(sd, CATEGORY_MONITOR, IQCMD_CAP_GRP_NO, 0x02);

	for(polling_times = 10; polling_times > 0; --polling_times) {
	    (void) m12mo_readb(sd, CATEGORY_MONITOR, IQCMD_CAP_GRP_UPDATE,&read_val);
		if(read_val == 0x00){
		   break;
		}
    }
	//For DIT request, enable shot mode for
	(void) m12mo_writeb(sd, CATEGORY_ASUS, ASUS_AE_SCENE_MODE, dev->shot_mode + PRE_AUTO);
#endif

	return 0;
out:
	dev_err(&client->dev, "m12mo_set_zsl_monitor failed %d\n", ret);
	return ret;
}

static u32 __get_dual_capture_value(u8 capture_mode)
{
	switch(capture_mode) {
	case M12MO_CAPTURE_MODE_ZSL_BURST:
		return DUAL_CAPTURE_BURST_CAPTURE_START;
	case APP_LLS_CAP_MODE_ZSL:
		return DUAL_CAPTURE_LLS_CAPTURE_START;
	case M12MO_CAPTURE_MODE_ZSL_NORMAL:
		return DUAL_CAPTURE_ZSL_CAPTURE_START;
	case APP_HDR_CAP_MODE_ZSL:
	case M12MO_CAPTURE_MODE_ZSL_RAW:
	default:
		return DUAL_CAPTURE_SINGLE_CAPTURE_START;
	}
}

static int m12mo_set_zsl_capture(struct v4l2_subdev *sd, int sel_frame)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret, i;
	u32 val, dual_status;
	int finish = 0;

	/* TODO: Fix this. Currently we do not use this */
	(void) sel_frame;

	val = __get_dual_capture_value(dev->capture_mode);

	/* Check dual capture status before the capture request */
	for (i = POLL_NUM; i; i--) {
		ret = m12mo_readb(sd, CATEGORY_CAPTURE_CTRL, START_DUAL_STATUS,
				&dual_status);
		if (ret)
			continue;

		if ((dual_status == 0) || (dual_status == DUAL_STATUS_AF_WORKING)) {
			finish = 1;
			break;
		}

		msleep(10);
	}

	/*
	* If last capture not finished yet, return error code
	*/
	if (!finish) {
		dev_err(&client->dev, "%s Device busy. Status check failed %d\n",
			__func__, dual_status);
		return -EBUSY;
	}

	/* Start capture, JPEG encode & transfer start */
	ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, START_DUAL_CAPTURE, val);
	dev_dbg(&client->dev, "%s zsl capture trigger result: %d\n",
			__func__, ret);
	return ret;
}

int m12mo_set_zsl_raw_capture(struct v4l2_subdev *sd)
{
	int ret;

	/* Set capture mode - Infinity capture 3 */
	ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, CAP_MODE,
						CAP_MODE_ZSL);
	if (ret)
		return ret;

	/* Enable interrupt signal */
	ret = m12mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_INT_ENABLE, 0x01);
	if (ret)
		return ret;

	/* Go to ZSL Monitor mode */
	ret = m12mo_request_cmd_effect(sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
	if (ret)
		return ret;

	ret = m12mo_wait_mode_change(sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD,
						M12MO_INIT_TIMEOUT);
	if (ret < 0)
		return ret;

	/* switch to RAW capture mode */
	ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REG_CAP_NV12_MODE,
						RAW_CAPTURE);
	if (ret)
		return ret;

	/* RAW mode is set. Now do a normal capture */
	return m12mo_set_zsl_capture(sd, 1);
}

int m12mo_set_burst_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	if(!val){
	    dev->capture_mode &= ~M12MO_CAPTURE_MODE_ZSL_BURST;
	}else{
        dev->capture_mode |= M12MO_CAPTURE_MODE_ZSL_BURST;
	}
	return 0;
}

static int m12mo_set_lls_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	if(!val){
	    dev->capture_mode &= ~APP_LLS_CAP_MODE_ZSL;
	}else{
        dev->capture_mode |= APP_LLS_CAP_MODE_ZSL;
	}
	return 0;

}

static int m12mo_set_hdr_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
/*
	switch(val) {
	case STOP_HDR_MODE:
		//=== switch to normal capture. HDR MODE off ===//
		printk(KERN_INFO "@m12mo_set_hdr_mode, STOP_HDR_MODE\n");
		dev->capture_mode = M12MO_CAPTURE_MODE_ZSL_NORMAL;
		break;
	case START_HDR_MODE:
		//=== switch to HDR mode ===//
		printk(KERN_INFO "@m12mo_set_hdr_mode, START_HDR_MODE\n");
		dev->capture_mode = APP_HDR_CAP_MODE_ZSL;
		break;
	case RESUME_PREVIEW_IN_HDR_MODE:
		//=== switch to HDR mode ===//
		printk(KERN_INFO "@m12mo_set_hdr_mode, RESUME_PREVIEW_IN_HDR_MODE\n");
		dev->capture_mode = APP_HDR_CAP_MODE_ZSL;
		break;
	default:
		return -EINVAL;
	}
*/
    if(!val){
	    dev->capture_mode &= ~APP_HDR_CAP_MODE_ZSL;
	}else{
        dev->capture_mode |= APP_HDR_CAP_MODE_ZSL;
	}
	return 0;
}
/*
#define EXT_ISP_VIDEO_MODE_NORMAL 40
#define EXT_ISP_VIDEO_MODE_PRO 41
#define EXT_ISP_VIDEO_MODE_MMS 42
#define EXT_ISP_VIDEO_MODE_MINIATURE 43
#define EXT_ISP_VIDEO_MODE_SLOWMOTION 44
#define EXT_ISP_VIDEO_MODE_HISPEED 45
#define EXT_ISP_VIDEO_MODE_TIMELAPSE 46
#define EXT_ISP_VIDEO_MODE_LOWLIGHT 47
#define EXT_ISP_VIDEO_MODE_EFFECT 48
*/
static u32 movie_scene_mode_cmds (u32 val){
    switch (val) {
        case EXT_ISP_VIDEO_MODE_NORMAL:
		return VID_NORMAL;

        case EXT_ISP_VIDEO_MODE_PRO:
		return VID_NORMAL;

        case EXT_ISP_VIDEO_MODE_MMS:
        return VID_MMS;

        case EXT_ISP_VIDEO_MODE_MINIATURE:
	    return VID_MINIATURE;

        case EXT_ISP_VIDEO_MODE_SLOWMOTION:
		return VID_SLOWMOTION;

        case EXT_ISP_VIDEO_MODE_HISPEED:
		return VID_HISPEED;

        case EXT_ISP_VIDEO_MODE_TIMELAPSE:
		return VID_TIMELAPSE;

        case EXT_ISP_VIDEO_MODE_LOWLIGHT:
		return VID_HILIGHT;

        case EXT_ISP_VIDEO_MODE_EFFECT:
		return VID_NORMAL;
    }
	return 0;
}
static int m12mo_set_shot_mode(struct v4l2_subdev *sd, unsigned int val)
{
    struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 movie_cmd;
	printk("@%s %d val is %d \n", __func__, __LINE__, val);

	if(val >= 40) {
        movie_cmd = movie_scene_mode_cmds(val);
	    return m12mo_writeb(sd, CATEGORY_ASUS, ASUS_AE_SCENE_MODE, movie_cmd);
	}

	switch (val) {
    case EXT_ISP_SHOT_MODE_PRO:
	    dev->shot_mode = CAP_AUTO;
		break;
	case EXT_ISP_SHOT_MODE_NORMAL:
	    dev->shot_mode = CAP_AUTO;
	    break;
	case EXT_ISP_SHOT_MODE_EFFECT:
	    dev->shot_mode = CAP_AUTO;
	    break;
	case EXT_ISP_VIDEO_MODE_TIMELAPSE:
	    dev->shot_mode = CAP_AUTO;
		break;
	case EXT_ISP_SHOT_MODE_SUPERRESOLUTION:
	    dev->shot_mode = CAP_AUTO;
	    break;
	case EXT_ISP_SHOT_MODE_BEAUTY_FACE:
	    dev->shot_mode = CAP_BEAUTY;
	    break;
	case EXT_ISP_SHOT_MODE_BEST_FACE:
		dev->shot_mode = CAP_ALLSMILES;
	    break;
	case EXT_ISP_SHOT_MODE_RICH_TONE_HDR:
	    dev->shot_mode = CAP_HDR;
	    break;
	case EXT_ISP_SHOT_MODE_LOWLIGHT:
	    dev->shot_mode = CAP_HILIGHT;
	    break;
	case EXT_ISP_SHOT_MODE_NIGHT:
	    dev->shot_mode = CAP_NIGHT;
		break;
	case EXT_ISP_SHOT_MODE_DEFOCUS:
	    dev->shot_mode = CAP_DEFOCUS;
	    break;
	case EXT_ISP_SHOT_MODE_SELFIE:
	    dev->shot_mode = CAP_SELFIE;
		break;
	case EXT_ISP_SHOT_MODE_ANIMATED_PHOTO:
	    dev->shot_mode = CAP_GIF;
		break;
	case EXT_ISP_SHOT_MODE_PANORAMA:
	    dev->shot_mode = CAP_PANORAMA;
		break;
	case EXT_ISP_SHOT_MODE_MINIATURE:
	    dev->shot_mode = CAP_MINIATURE;
		break;
	case EXT_ISP_SHOT_MODE_ERASER:
	    dev->shot_mode = CAP_SMARTREMOVE;
		break;
	case EXT_ISP_SHOT_MODE_TIMEREWIND:
	    dev->shot_mode = CAP_TIMEREWIND;
		break;
#if 0
	case EXT_ISP_SHOT_MODE_AUTO:
		dev->shot_mode = SHOT_MODE_AUTO;
		break;
	case EXT_ISP_SHOT_MODE_BEAUTY_FACE:
		dev->shot_mode = SHOT_MODE_BEAUTY_FACE;
		break;
	case EXT_ISP_SHOT_MODE_BEST_PHOTO:
		dev->shot_mode = SHOT_MODE_BEST_PHOTO;
		break;
	case EXT_ISP_SHOT_MODE_DRAMA:
		dev->shot_mode = SHOT_MODE_DRAMA;
		break;
	case EXT_ISP_SHOT_MODE_BEST_FACE:
		dev->shot_mode = SHOT_MODE_BEST_FACE;
		break;
	case EXT_ISP_SHOT_MODE_ERASER:
		dev->shot_mode = SHOT_MODE_ERASER;
		break;
	case EXT_ISP_SHOT_MODE_PANORAMA:
		dev->shot_mode = SHOT_MODE_PANORAMA;
		break;
	case EXT_ISP_SHOT_MODE_RICH_TONE_HDR:
		dev->shot_mode = SHOT_MODE_RICH_TONE_HDR;
		break;
	case EXT_ISP_SHOT_MODE_NIGHT:
		dev->shot_mode = SHOT_MODE_NIGHT;
		break;
	case EXT_ISP_SHOT_MODE_SOUND_SHOT:
		dev->shot_mode = SHOT_MODE_SOUND_SHOT;
		break;
	case EXT_ISP_SHOT_MODE_ANIMATED_PHOTO:
		dev->shot_mode = SHOT_MODE_ANIMATED_PHOTO;
		break;
	case EXT_ISP_SHOT_MODE_SPORTS:
		dev->shot_mode = SHOT_MODE_SPORTS;
		break;
#endif
	default:
		return -EINVAL;
	}
	                                 //=== See ASUS_AE_SCENE_MODE_ in m12mo.h ===//
	(void) m12mo_writeb(sd, CATEGORY_ASUS, ASUS_AE_SCENE_MODE, dev->shot_mode + PRE_AUTO);

	return 0;
}

static int m12mo_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct m12mo_device *dev = to_m12mo_sensor(sd);

    printk("@%s %d start\n", __func__, __LINE__);
	mutex_lock(&dev->input_lock);
	ret = __m12mo_s_power(sd, on, false);
	mutex_unlock(&dev->input_lock);

	return ret;
}

int m12mo_single_capture_process(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret = 0;
	u32 fmt;
    printk("@%s %d start\n", __func__, __LINE__);

	/* Select frame */
	/*ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL,*/
			  /*CAPC_SEL_FRAME_MAIN, 0x01);*/
	/*if (ret)*/
		/*return ret;*/

	/* Image format */
	if (dev->format.code == V4L2_MBUS_FMT_JPEG_1X8){
		fmt = CAPTURE_FORMAT_JPEG8;
        printk("%s %d, capture format is JPEG8\n", __func__, __LINE__);
    }
	else{
		fmt = CAPTURE_FORMAT_YUV422;
        printk("%s %d, capture format is YUV422\n", __func__, __LINE__);
    }

	/*ret = m12mo_writeb(sd, CATEGORY_CAPTURE_PARAM, CAPP_YUVOUT_MAIN, fmt);*/
	/*if (ret)*/
		/*return ret;*/

	/* Image size */
	/*ret = m12mo_writeb(sd, CATEGORY_CAPTURE_PARAM, CAPP_MAIN_IMAGE_SIZE,*/
			   /*dev->curr_res_table[dev->fmt_idx].command);*/
	/*if (ret)*/
		/*return ret;*/

	/* Start image transfer */
	/*ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL,*/
			  /*CAPC_TRANSFER_START, 0x01);*/

	return ret;
}
#ifndef NOIRQ
static irqreturn_t m12mo_irq_thread(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_id;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 int_factor;
//	u32 val = 0;
    printk("%s %d get IRQ, requested_cmd = %d flashrom_mode %d\n"
    	, __func__, __LINE__, dev->requested_cmd, dev->flashrom_mode);

	/* Clear interrupt by reading interrupt factor register */
    if(dev->flashrom_mode){
        (void) m12mo_readb(sd, CATEGORY_FLASHROM, REG_FLASH_INT_FACTOR, &int_factor);
        if(int_factor == 0x01) {
            dev_info(&client->dev, "INT_FACTOR: 0x%x, FW start INT\n", int_factor);
            return IRQ_HANDLED;
        }
        else {
            dev->flashrom_int = int_factor;
            wake_up(&dev->irq_waitq);
            dev_info(&client->dev, "INT_FACTOR: 0x%x, dev->wait_irq_flag is %d\n", int_factor, dev->wait_irq_flag);
            return IRQ_HANDLED;
        }
    }
    (void) m12mo_readb(sd, CATEGORY_SYSTEM, SYSTEM_INT_FACTOR, &int_factor);
#if 0 // for debug 
    (void) m12mo_readb(sd, 0x00, 0x1E, &val);	// check camera i2c
    (void) m12mo_readb(sd, 0x00, 0x02, &val);  // fw version
    (void) m12mo_readb(sd, 0x00, 0x03, &val);	// fw version
#endif	
	dev_info(&client->dev, "INT_FACTOR: 0x%x\n", int_factor);

	switch (dev->requested_cmd) {
	case M12MO_CAMERA_START:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M12MO_CAMERA_START;
		}
		break;
	case M12MO_PARAMETER_MODE_REQUEST_CMD:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M12MO_PARAMETER_MODE_REQUEST_CMD;
		}
		break;
	case M12MO_MONITOR_MODE:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M12MO_MONITOR_MODE;
		}
		break;
	case M12MO_MONITOR_MODE_ZSL_REQUEST_CMD:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M12MO_MONITOR_MODE_ZSL_REQUEST_CMD;
		}
		break;
	case M12MO_SINGLE_CAPTURE_MODE:
		if (int_factor & REG_INT_STATUS_CAPTURE) {
			dev->cmd = M12MO_SINGLE_CAPTURE_MODE;
			dev->fw_ops->single_capture_process(sd);
		}
		break;
	case M12MO_BURST_CAPTURE_MODE:
		if (int_factor & REG_INT_STATUS_MODE){
			dev->cmd = M12MO_BURST_CAPTURE_MODE;
        }
		break;
	case M12MO_MONITOR_MODE_HIGH_SPEED:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M12MO_MONITOR_MODE_HIGH_SPEED;
		}
		break;
    case M12MO_START_AF:
        printk("%s %d, AF start finished\n", __func__, __LINE__);
        if (int_factor & 0x2) {
            dev->cmd = M12MO_START_AF;
        }
		break;
    case M12MO_MANUAL_FOCUS:
        printk("%s %d, MANUAL_FOCUS finished\n", __func__, __LINE__);
        if (int_factor & 0x2) {
            dev->cmd = M12MO_MANUAL_FOCUS;
        }
        break;
	case M12MO_START_DIGITAL_ZOOM:
        printk("%s %d, 0 digital ZOOM mode start finished\n", __func__, __LINE__);
        if (int_factor & 0x4) {
            printk("%s %d,  1 digital ZOOM mode start finished\n", __func__, __LINE__);
            dev->cmd = M12MO_START_DIGITAL_ZOOM;
        }
        break;
	default:
		dev_err(&client->dev, "m12mo_irq_thread :default case\n");
		return IRQ_HANDLED;
	}

    wake_up(&dev->irq_waitq);

	printk("%s dev->wait_irq_flag is %d\n", __func__, dev->wait_irq_flag);

	return IRQ_HANDLED;
}

static int m12mo_setup_irq(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int pin, ret;

    printk("%s %d start\n", __func__, __LINE__);
	if (!dev->pdata->common.gpio_intr_ctrl) {
		dev_err(&client->dev,
			"Missing gpio information in interrupt setup!\n");
		return -ENODEV;
	}

	pin = dev->pdata->common.gpio_intr_ctrl(sd);
	if (pin < 0) {
		ret = pin;
		goto out;
	}

	ret = gpio_to_irq(pin);

	if (ret < 0) {
		dev_err(&client->dev, "Configure gpio to irq failed!\n");
		goto out;
	}
	client->irq = ret;

	ret = request_threaded_irq(client->irq, NULL, m12mo_irq_thread,
			  IRQF_TRIGGER_RISING | IRQF_ONESHOT, M12MO_NAME, sd);
	if (ret < 0) {
		dev_err(&client->dev, "Cannot register IRQ: %d\n", ret);
		goto out;
	}
	return 0;

out:
	return ret;
}
#endif
int get_resolution_index(const struct m12mo_resolution *res,
			int entries, u32 w, u32 h)
{
	int i;

	for (i = 0; i < entries; i++) {
        printk("%s test res %d x %d\n", __func__, res[i].width, res[i].height);
		if (w != res[i].width)
			continue;
		if (h != res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

int __m12mo_try_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt, bool update_fmt)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct atomisp_input_stream_info *stream_info =
			(struct atomisp_input_stream_info *)fmt->reserved;
	const struct m12mo_resolution *res;

	int entries, idx;
	int mode = M12MO_GET_RESOLUTION_MODE(dev->fw_type);

    printk("@%s %d, format code = 0x%x\n", __func__, __LINE__, fmt->code);
//=== FIXME: Why HAL set so so many different format to driver?
//=== M12MO - temporarily hard code here. Start. ======//
	if (fmt->code != V4L2_MBUS_FMT_JPEG_1X8 &&
	    fmt->code != V4L2_MBUS_FMT_UYVY8_1X16 &&
		fmt->code != V4L2_MBUS_FMT_SGBRG8_1X8 &&
	    fmt->code != V4L2_MBUS_FMT_CUSTOM_NV12 &&
	    fmt->code != V4L2_MBUS_FMT_CUSTOM_NV21 &&
	    fmt->code != V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {
		printk("%s unsupported code: 0x%x. Set to NV12\n", __func__, fmt->code);
		fmt->code = V4L2_MBUS_FMT_CUSTOM_NV12;
	}
    if (fmt->code == V4L2_MBUS_FMT_SGBRG8_1X8) {
        printk(KERN_INFO "@%s, Raw Capture.\n", __func__);
        dev->capture_mode = M12MO_CAPTURE_MODE_ZSL_RAW;
	} else {
        fmt->code = V4L2_MBUS_FMT_UYVY8_1X16;
    }
//=== M12MO - temporarily hard code here. End. ======//
	/* In ZSL case, capture table needs to be handled separately */
    printk("@%s %d, mode = %d, stream = %d, run_mode = 0x%x, fmt->code is 0x%x, required size = %dX%d\n", __func__, __LINE__, mode, stream_info->stream, dev->run_mode, fmt->code, fmt->width, fmt->height);

    if (stream_info->stream == ATOMISP_INPUT_STREAM_CAPTURE &&
			(dev->run_mode == CI_MODE_PREVIEW ||
			 dev->run_mode == CI_MODE_VIDEO ||
			 dev->run_mode == CI_MODE_CONTINUOUS)) {
		res = resolutions[M12MO_MODE_CAPTURE_INDEX];
		entries = resolutions_sizes[M12MO_MODE_CAPTURE_INDEX];
        printk("@%s set capture resolution\n", __func__);
	} else {
		res = dev->curr_res_table;
		entries = dev->entries_curr_table;
        printk("@%s set current resolution\n", __func__);
	}

	/* check if the given resolutions are spported */
	idx = get_resolution_index(res, entries, fmt->width, fmt->height);
	if (idx < 0) {
		dev_err(&client->dev, "%s unsupported resolution: %dx%d\n",
			__func__, fmt->width, fmt->height);
		return -EINVAL;
	}

	/* If the caller wants to get updated fmt values based on the search */
	if (update_fmt) {
		if (fmt->code == V4L2_MBUS_FMT_JPEG_1X8) {
			fmt->width = dev->mipi_params.jpeg_width;
			fmt->height = dev->mipi_params.jpeg_height;
		} else if (fmt->code == V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {
			fmt->width = dev->mipi_params.raw_width;
			fmt->height = dev->mipi_params.raw_height;
		} else {
			fmt->width = res[idx].width;
			fmt->height = res[idx].height;
		}
	}
	return idx;
}

static int m12mo_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int idx;

	mutex_lock(&dev->input_lock);
	idx = dev->fw_ops->try_mbus_fmt(sd, fmt, true);
	mutex_unlock(&dev->input_lock);
	return idx >= 0 ? 0 : -EINVAL;
}

static int m12mo_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);

	mutex_lock(&dev->input_lock);

	fmt->width = dev->curr_res_table[dev->fmt_idx].width;
	fmt->height = dev->curr_res_table[dev->fmt_idx].height;
	fmt->code = dev->format.code;

	mutex_unlock(&dev->input_lock);

	return 0;
}

int __m12mo_update_stream_info(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct atomisp_input_stream_info *stream_info =
			(struct atomisp_input_stream_info*)fmt->reserved;
	int mode = M12MO_GET_RESOLUTION_MODE(dev->fw_type);

	/* TODO: Define a FW Type as well. Resolution could be reused */
	switch (mode) {
	case M12MO_RESOLUTION_MODE_0:
		/* TODO: handle FW type cases here */
        printk("%s resolution mode 0\n", __func__);
		break;
	case M12MO_RESOLUTION_MODE_1:
		/* Raw Capture is a special case. Capture data comes like HDR */
		if (fmt->code == V4L2_MBUS_FMT_JPEG_1X8 ||
			fmt->code == V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {
            printk("%s JPEG or RAW\n", __func__);
			/* fill stream info */
			stream_info->ch_id = M12MO_ZSL_JPEG_VIRTUAL_CHANNEL;
			stream_info->isys_configs = 1;
			stream_info->isys_info[0].input_format =
				(u8)ATOMISP_INPUT_FORMAT_USER_DEF3;
			stream_info->isys_info[0].width = 0;
			stream_info->isys_info[0].height = 0;
		} else if (fmt->code == V4L2_MBUS_FMT_CUSTOM_NV12 ||
			   fmt->code == V4L2_MBUS_FMT_CUSTOM_NV21) {
            printk("%s NV12 or NV21\n", __func__);
			stream_info->ch_id = M12MO_ZSL_NV12_VIRTUAL_CHANNEL;
			stream_info->isys_configs = 2;
			/* first stream */
			stream_info->isys_info[0].input_format =
				(u8)ATOMISP_INPUT_FORMAT_USER_DEF1;
			stream_info->isys_info[0].width = (u16)fmt->width;
			stream_info->isys_info[0].height = (u16)fmt->height;
			/* Second stream */
			stream_info->isys_info[1].input_format =
				(u8)ATOMISP_INPUT_FORMAT_USER_DEF2;
			stream_info->isys_info[1].width = (u16)fmt->width;
			stream_info->isys_info[1].height = (u16)fmt->height / 2;
		}
        printk("%s nothing\n", __func__);
		break;
	}
	return 0;
}

int __m12mo_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct atomisp_input_stream_info *stream_info =
			(struct atomisp_input_stream_info *)fmt->reserved;
	/*int mode = M12MO_GET_RESOLUTION_MODE(dev->fw_type);*/
	int index;
    printk(KERN_INFO "@%s, start! fmt->width is %d, fmt->height is %d\n", __func__, fmt->width, fmt->height);
	mutex_lock(&dev->input_lock);

	index = dev->fw_ops->try_mbus_fmt(sd, fmt, false);
	if (index < 0) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	dev->format.code = fmt->code;
	dev->fmt_idx = index;
	if (stream_info->stream == ATOMISP_INPUT_STREAM_CAPTURE) {
		/* Save the index for selecting the capture resolution */
		dev->capture_res_idx = dev->fmt_idx;
	}

	/*
	 * In ZSL Capture cases, for capture an image the run mode is not
	 * changed. So we need to maintain a separate capture table index
	 * to select the snapshot sizes.
	 */
	if (stream_info->stream == ATOMISP_INPUT_STREAM_CAPTURE &&
	    (dev->run_mode == CI_MODE_PREVIEW ||
	     dev->run_mode == CI_MODE_VIDEO ||
	     dev->run_mode == CI_MODE_CONTINUOUS))
		dev->capture_res_idx = dev->fmt_idx;

	dev_info(&client->dev,
		"%s index prev/cap: %d/%d width: %d, height: %d, code; 0x%x\n",
		 __func__, dev->fmt_idx, dev->capture_res_idx, fmt->width,
		 fmt->height, dev->format.code);

	/* Make the fixed width and height for JPEG and RAW formats */
	/*if (dev->format.code == V4L2_MBUS_FMT_JPEG_1X8) {*/
        /* The m12mo can only run JPEG in 30fps or lower */
		/*dev->fps = M12MO_NORMAL_FPS;*/
		/*fmt->width = dev->mipi_params.jpeg_width;*/
		/*fmt->height = dev->mipi_params.jpeg_height;*/
	/*} else if (dev->format.code == V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {*/
		/*fmt->width = dev->mipi_params.raw_width;*/
		/*fmt->height = dev->mipi_params.raw_height;*/
	/*}*/

	/* Update the stream info. Atomisp uses this for configuring mipi */
    /*__m12mo_update_stream_info(sd, fmt);*/

	/*
	 * Handle raw capture mode separately. Update the capture mode to RAW
	 * capture now. So that the next streamon call will start RAW capture.
	 */
    /*printk("%s fw type = 0x%x\n", __func__, dev->fw_type);*/
	/*if (mode == M12MO_RESOLUTION_MODE_1 &&*/
		/*dev->format.code == V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {*/
		/*dev_dbg(&client->dev, "%s RAW capture mode\n", __func__);*/
        /*printk("%s RAW capture mode\n", __func__);*/
		/*dev->capture_mode = M12MO_CAPTURE_MODE_ZSL_RAW;*/
		/*dev->capture_res_idx = dev->fmt_idx;*/
		/*dev->fmt_idx = 0;*/
	/*}*/

    if (PREVIEW_TEST_PATTERN) {
        m12mo_writeb(sd, CATEGORY_PARAM, CAPP_MAIN_IMAGE_SIZE, dev->curr_res_table[index].command);
/*
        if (fmt->width == 1920 && fmt->height == 1080) {
            m12mo_writeb(sd, 0x01, 0x01, 0x28); // 1920 x 1080
            m12mo_writeb(sd, 0x01, 0x02, 0x02); // 30fps
        } else if (fmt->width == 5472 && fmt->height == 4104) {
            m12mo_writeb(sd, 0x01, 0x01, 0x59); // 5472 x 4104
            m12mo_writeb(sd, 0x01, 0x02, 0x07); // 3fps
        }
*/
    }

	mutex_unlock(&dev->input_lock);
	return 0;
}

static int m12mo_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);

	return dev->fw_ops->set_mbus_fmt(sd, fmt);
}

static int m12mo_set_run_mode(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret = 0;
    printk("@%s %d start, capture_mode = %d, run_mode = 0x%x\n", __func__, __LINE__, dev->capture_mode, dev->run_mode);

	switch (dev->run_mode) {
	case CI_MODE_STILL_CAPTURE:
        if (dev->capture_mode != M12MO_CAPTURE_MODE_ZSL_RAW)
            m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, CAP_TRANSFER_START, START_CAP_START);

		break;
    case CI_MODE_PREVIEW:
	case CI_MODE_VIDEO:
        if(!isCaptureMode){
            printk("@%s %d, preview set monitor mode\n", __func__, __LINE__);
            ret = m12mo_set_zsl_monitor(sd);
        }
        else{
            printk("@%s %d, after capturing, start the preview frame \n", __func__, __LINE__);
            if (dev->capture_mode == M12MO_CAPTURE_MODE_ZSL_RAW) {
                ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, START_CAP, START_CAP_STOP);
                dev->capture_mode = M12MO_CAPTURE_MODE_ZSL_NORMAL;
            }
            else {
                ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, START_CAP, START_CAP_PREVIEW);
            }
        }

        isCaptureMode = false;
        break;
	default:
        isCaptureMode = false;
	}
	return ret;
}

static void m12mo_check_app_cap_mode(struct v4l2_subdev *sd)
{
    struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 read_val;
    printk("@%s %d start, capture_mode = %d, run_mode = 0x%x\n", __func__, __LINE__, dev->capture_mode, dev->run_mode);

	(void) m12mo_readb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, &read_val);
	if(read_val == dev->capture_mode){
	     printk("@%s, already in capture mode: %d, no need to set again. \n", __func__, read_val);
		 return;
	}
	switch(dev->capture_mode){
	case APP_HDR_CAP_MODE_ZSL:
	    printk(KERN_INFO "m12mo, APP HDR capture mode.\n");
	    (void) m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, APP_HDR_CAP_MODE_ZSL);
	    break;
	case APP_LLS_CAP_MODE_ZSL:
		printk(KERN_INFO "m12mo, APP set %d frame for LLS capture mode.\n",num_LLS);
		(void) m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, 0xC, num_LLS);
	    printk(KERN_INFO "m12mo, APP LLS capture mode.\n");
		(void) m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, APP_LLS_CAP_MODE_ZSL);
		break;
	case M12MO_CAPTURE_MODE_ZSL_NORMAL:
	    printk(KERN_INFO "m12mo, APP Auto capture mode.\n");
		(void) m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, M12MO_CAPTURE_MODE_ZSL_NORMAL);
		break;
	case M12MO_CAPTURE_MODE_ZSL_RAW:
//	    (void)__m12mo_param_mode_set(sd);
//	    (void) m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, RAW_CAP);
//		(void) m12mo_set_run_mode(sd);
		break;
	default:
	    printk(KERN_INFO "m12mo, erro APP capture mode %d \n", dev->capture_mode);
		break;
	}
}
#if 0
static int m12mo_identify_fw_type(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m12mo_fw_id *fw_ids = NULL;
	char buffer[M12MO_MAX_FW_ID_STRING];
	int ret;

	int m12mo_fw_address_cnt = m12mo_get_fw_address_count();
	int i;

    printk("%s start\n", __func__);
	ret = dev->pdata->identify_fw();
	if (ret != -1) {
		dev->fw_type = ret;
		return 0;
	}

	for (i = 0; i < m12mo_fw_address_cnt; i++) {
		fw_ids = dev->pdata->fw_ids;
		if (!fw_ids)
			return 0;

		ret = m12mo_get_isp_fw_version_string(dev, buffer,
				sizeof(buffer), i);
		if (ret)
			return ret;

		while (fw_ids->id_string) {
            dev_err(&client->dev, "Find FW id %s detected\n", buffer);
			if (!strncmp(fw_ids->id_string, buffer,
				     strlen(fw_ids->id_string)))
			{
				dev_info(&client->dev, "FW id %s detected\n", buffer);
				dev->fw_type = fw_ids->fw_type;
				dev->fw_addr_id = i;
				return 0;
			}
			fw_ids++;
		}
	}

	dev_err(&client->dev, "FW id string table given but no match found");
	return 0;
}
#endif

static void m12mo_mipi_initialization(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int id = M12MO_GET_MIPI_PACKET_SIZE_IDX(dev->fw_type);
	u32 mipi_packet_size = dev->pdata->mipi_packet_size[id];

	dev->mipi_params.jpeg_width = mipi_packet_size;
	dev->mipi_params.jpeg_height = M12MO_MAX_YUV422_SIZE / mipi_packet_size;

	dev->mipi_params.raw_width = mipi_packet_size;
	dev->mipi_params.raw_height = M12MO_MAX_RAW_SIZE / mipi_packet_size;

	dev_dbg(&client->dev,
		"%s JPEG: W=%d H=%d, RAW: W=%d H=%d\n", __func__,
		dev->mipi_params.jpeg_width, dev->mipi_params.jpeg_height,
		dev->mipi_params.raw_width, dev->mipi_params.raw_height);
}
//=========== TEST START ============//
int m12mo_send_still_capture_cmds(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 res_cmd;
	u32 read_val;
    isCaptureMode = true;
	res_cmd = dev->curr_res_table[dev->fmt_idx].command;
	printk("@%s %d start\n", __func__, __LINE__);

	dev_info(&client->dev, "%s mode: %d width: %d, height: %d, cmd: 0x%x, capture_mode: %d\n",
		__func__, dev->cmd, dev->curr_res_table[dev->fmt_idx].width,
		dev->curr_res_table[dev->fmt_idx].height,
		dev->curr_res_table[dev->fmt_idx].command,
        dev->capture_mode);

	if (dev->capture_mode != M12MO_CAPTURE_MODE_ZSL_RAW) {
        ret = m12mo_writeb(sd, CATEGORY_CAPTURE_PARAM, CAPP_MAIN_IMAGE_SIZE, res_cmd);
    }

	//=== Check "application capture mode" and send command to m12mo if needed. ===//
    m12mo_check_app_cap_mode(sd);

	(void) m12mo_readb(sd, CATEGORY_SYSTEM, SYSTEM_SYSMODE, &read_val);
	if(read_val == M12MO_PARAMETER_MODE_REQUEST_CMD){
          printk(KERN_INFO "@%s, In parameter mode, ILLIGAL!!! ILLIGAL!!! ILLIGAL to CAPTURE!!! \n", __func__);
	}else if(read_val == 0x02){
          printk(KERN_INFO "@%s, In monitor mode, It is legal to CAPTURE. \n", __func__);
	}

    //=== Send command to capture  ===//
	if (dev->capture_mode == M12MO_CAPTURE_MODE_ZSL_RAW) {
        ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, START_CAP, START_CAP_RAW);
	} else {
        ret = m12mo_writeb(sd, CATEGORY_CAPTURE_CTRL, START_CAP, START_CAP_START);
    }
	return ret;
}
//=========== TEST END ==============//

int m12mo_normal_streamoff(struct v4l2_subdev *sd)
{
//    struct m12mo_device *dev = to_m12mo_sensor(sd);
    pr_info("%s, m12mo_capture_pre_flag is %d", __func__, m12mo_capture_pre_flag);

    switch(m12mo_capture_pre_flag){
    case M12MO_NOT_CAPTURE_0:
	    return __m12mo_param_mode_set(sd);

	case M12MO_CAP_BEFORE_1ST_STREAMOFF_1:
	    pr_info("m12mo, 1st fake stream off during capture\n");
		m12mo_capture_pre_flag = M12MO_CAP_BETWEEN_1ST_AND_2ND_STREAMOFF_2;
//		previous_preview_cmd = dev->curr_res_table[dev->fmt_idx].command;
		return 0;

    case M12MO_CAP_BETWEEN_1ST_AND_2ND_STREAMOFF_2:
	    pr_info("m12mo, 2nd fake stream off during capture\n");
		m12mo_capture_pre_flag = M12MO_NOT_CAPTURE_0;
		return 0;

	default:
        return -EIO;
	}

}

int m12mo_test_pattern(struct v4l2_subdev *sd, u8 val)
{

	struct m12mo_device *dev = to_m12mo_sensor(sd);
	pr_info("%s, run mode: %x\n", __func__, dev->run_mode);

	m12mo_writeb(sd, CATEGORY_TEST, 0xEE, val);

	return 0;
}

static const struct m12mo_fw_ops fw_ops = {
	.set_run_mode           = m12mo_set_run_mode,
	.set_burst_mode         = m12mo_set_burst_mode,
	.stream_off             = m12mo_normal_streamoff,
	.single_capture_process = m12mo_single_capture_process,
	.try_mbus_fmt           =  __m12mo_try_mbus_fmt,
	.set_mbus_fmt           =  __m12mo_set_mbus_fmt,
	.test_pattern           = m12mo_test_pattern,
};

void m12mo_handlers_init(struct v4l2_subdev *sd)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);

	pr_info("%s, dev->fw_type: %d\n", __func__, dev->fw_type);

    dev->fw_ops = &fw_ops;
}

static int m12mo_s_config(struct v4l2_subdev *sd, int irq)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	int id = 0;

	pr_info("%s\n", __func__);

	mutex_lock(&dev->input_lock);

	init_waitqueue_head(&dev->irq_waitq);
    init_waitqueue_head(&dev->d_zoom_waitq);
	dev->fw_type = dev->pdata->def_fw_type;
	dev->ref_clock = 19200000;//dev->pdata->ref_clock_rate[id];

	if (dev->pdata->common.platform_init) {
		ret = dev->pdata->common.platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			return ret;
		}
	}

#ifndef NOIRQ
	/* set up irq */
	ret = m12mo_setup_irq(sd);
	if (ret) {
		dev_err(&client->dev, "IRQ err.\n");
		mutex_unlock(&dev->input_lock);
		return ret;
	}
#else
	wake_up(&dev->irq_waitq);
#endif
	ret = __m12mo_s_power(sd, 1, true);
	if (ret) {
		dev_err(&client->dev, "power-up err.\n");
		goto free_irq;
	}

	if (dev->pdata->common.csi_cfg) {
		ret = dev->pdata->common.csi_cfg(sd, 1);
		if (ret)
			goto fail;
	}

	ret = m12mo_fw_checksum(dev, &dev->ver.checksum);
	if (ret) {
		dev_err(&client->dev, "Checksum calculation fails.\n");
		goto fail;
	}
	if (dev->ver.checksum != M12MO_VALID_CHECKSUM){
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is 0x%04x.\n", dev->ver.checksum);
	}

    /* TBD: Trig FW update here */
	/*
	 * We don't care about the return value here. Even in case of
	 * wrong or non-existent fw this phase must pass.
	 * FW can be updated later.
	 */
	/*m12mo_identify_fw_type(sd);*/

	/*
	 * Only after identify_fw_type the correct dev->fw_type
	 * can be got, so here update the ref_clock
	 */
	id = M12MO_GET_CLOCK_RATE_MODE(dev->fw_type);
	dev->ref_clock = 19200000;//dev->pdata->ref_clock_rate[id];

	m12mo_mipi_initialization(sd);

	/* Set proper function pointers based on FW_TYPE */
	m12mo_handlers_init(sd);

	ret = __m12mo_s_power(sd, 0, true);
	if (ret) {
		dev_err(&client->dev, "power-down err.\n");
		goto free_irq;
	}

	mutex_unlock(&dev->input_lock);
	return 0;

fail:
	__m12mo_s_power(sd, 0, true);
free_irq:
#ifndef NOIRQ
	free_irq(client->irq, sd);
#endif
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "External ISP power-gating failed\n");
	return ret;
}

static int m12mo_recovery(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	/* still power off sensor in case power is cut off abnormally*/
	ret = __m12mo_s_power(sd, 0, false);
	if (ret) {
		dev_err(&client->dev, "power-down err.\n");
		return ret;
	}
	usleep_range(100, 200);

	ret = __m12mo_s_power(sd, 1, false);
	if (ret) {
		dev_err(&client->dev, "power-up err.\n");
		return ret;
	}

	return ret;
}

static int m12mo_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret = 0;

	pr_info("%s, enable: %d\n", __func__, enable);
    if(dev->stream == enable)
        return 0;

    dev->stream = enable;
	mutex_lock(&dev->input_lock);
	if (enable) {

		if (!PREVIEW_TEST_PATTERN){
			  ret = dev->fw_ops->set_run_mode(sd);
		}else{
			m12mo_test_pattern(sd, 1); // enable test mode.
        }
		if (ret) {
			ret = m12mo_recovery(sd);
			if (ret) {
				mutex_unlock(&dev->input_lock);
				return ret;
			}

			if (!PREVIEW_TEST_PATTERN)
				ret = dev->fw_ops->set_run_mode(sd);
			else
				m12mo_test_pattern(sd, 1); // enable test mode.
 		}
	} else {

		if (!PREVIEW_TEST_PATTERN){
		        ret = dev->fw_ops->stream_off(sd);
		}else{
			m12mo_test_pattern(sd, 0); // enable test mode.
 	    }
	}
	mutex_unlock(&dev->input_lock);

	/*
	 * Dump M12MO log when stream-off with checking folder
	 */
	if (!enable)
		m12mo_dump_log(sd);

	return ret;
}

static int
m12mo_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
    printk("%s, start \n", __func__);
	if (fsize->index >= dev->entries_curr_table)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->curr_res_table[fsize->index].width;
	fsize->discrete.height = dev->curr_res_table[fsize->index].height;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int
m12mo_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int index = fse->index;
	printk("%s, start, m12mo_capture_pre_flag is %d \n", __func__, m12mo_capture_pre_flag);

	mutex_lock(&dev->input_lock);
	if(m12mo_capture_pre_flag != M12MO_NOT_CAPTURE_0){
        dev->curr_res_table = resolutions[M12MO_MODE_CAPTURE_INDEX];
	}

	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fse->min_width = dev->curr_res_table[index].width;
	fse->min_height = dev->curr_res_table[index].height;
	fse->max_width = dev->curr_res_table[index].width;
	fse->max_height = dev->curr_res_table[index].height;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int m12mo_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);

	if (code->index)
		return -EINVAL;

	code->code = dev->format.code;
	return 0;
}

static struct v4l2_mbus_framefmt *
__m12mo_get_pad_format(struct m12mo_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	switch (which) {
	case V4L2_SUBDEV_FORMAT_TRY:
		return v4l2_subdev_get_try_format(fh, pad);
	case V4L2_SUBDEV_FORMAT_ACTIVE:
		return &sensor->format;
	default:
		return NULL;
	}
}

static int
m12mo_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct m12mo_device *snr = to_m12mo_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__m12mo_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	mutex_lock(&snr->input_lock);
	fmt->format = *format;
	mutex_unlock(&snr->input_lock);

	return 0;
}

static int
m12mo_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct m12mo_device *snr = to_m12mo_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__m12mo_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	mutex_lock(&snr->input_lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;
	mutex_unlock(&snr->input_lock);

	return 0;
}

static const struct media_entity_operations m12mo_entity_ops = {
	.link_setup = NULL,
};

static int m12mo_set_flicker_freq(struct v4l2_subdev *sd, s32 val)
{
#if 1
	unsigned int flicker_freq;

	switch (val) {
	case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
		flicker_freq = M12MO_FLICKER_OFF;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
		flicker_freq = M12MO_FLICKER_50HZ;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		flicker_freq = M12MO_FLICKER_60HZ;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		flicker_freq = M12MO_FLICKER_AUTO;
		break;
	default:
		return -EINVAL;
	}
#endif
	printk("Flicker_freq (%d/%d)",val,flicker_freq);
	return m12mo_writeb(sd, CATEGORY_ASUS, ASUS_FLICKER, (u32)flicker_freq);
}

static int m12mo_get_metering(struct v4l2_subdev *sd, s32 *val)
{
	int ret;
	u32 metering;

	ret = m12mo_readb(sd, CATEGORY_ASUS, ASUS_METERING_MODE, &metering);
	if (ret)
		return ret;
	*val = metering;
	return 0;
}

static int m12mo_set_metering(struct v4l2_subdev *sd, s32 val)
{
#if 0
	unsigned int metering;

	switch (val) {
	case V4L2_EXPOSURE_METERING_CENTER_WEIGHTED:
		metering = M12MO_METERING_CENTER;
		break;
	case V4L2_EXPOSURE_METERING_SPOT:
		metering = M12MO_METERING_SPOT;
		break;
	case V4L2_EXPOSURE_METERING_AVERAGE:
		metering = M12MO_METERING_AVERAGE;
		break;
	default:
		return -EINVAL;
	}

#endif
	return m12mo_writeb(sd, CATEGORY_ASUS, ASUS_METERING_MODE, (u32) val);
}

static int m12mo_set_white_balance(struct v4l2_subdev *sd, s32 val)
{
//=== The value of white-balance please refer to the below file:  ===//
//=== linux/kernel/include/uapi/linux/v4l2-controls.h             ===//
    int ret;
    ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_WB_MANUAL, val);
    return ret;
}

static int m12mo_set_ev_bias(struct v4l2_subdev *sd, s32 val)
{
	/* 0x06 refers to 0.0EV value in m12mo HW */
	int ret;
	int ev_bias = 0x06 + val;
	printk("V4L2_CID_EXPOSURE (%d)",ev_bias);
	ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_EV, ev_bias);
	return ret;
}

static int m12mo_get_ev_bias(struct v4l2_subdev *sd, s32 *val)
{
	int ret;
	u32 ev_bias;

	ret = m12mo_readb(sd, CATEGORY_AE, AE_EV_BIAS, &ev_bias);
	if (ret)
		return ret;

	*val = (ev_bias-6) * M12MO_EV_STEP;
	return 0;
}

static u32 m12mo_iso_value_translate_to_command(s32 iso_value){
   int i;
   for (i = 0; i < ARRAY_SIZE(iso_table); i++){
		if (iso_value == iso_table[i][0]){
		    return iso_table[i][1];
		}
	}
	return REG_AE_ISOMODE_NOT_FOUND;
}
static int m12mo_set_iso_sensitivity(struct v4l2_subdev *sd, s32 val)
{
	u32 iso_cmd;

	iso_cmd = m12mo_iso_value_translate_to_command(val);
	if(iso_cmd != REG_AE_ISOMODE_NOT_FOUND){
	     return m12mo_writeb(sd, CATEGORY_ASUS, ASUS_ISO, iso_cmd);
	}

    printk(KERN_INFO "@%s: ISO value is %d, not supported! \n", __func__, val);
	return -EINVAL;
}

static int m12mo_set_iso_mode(struct v4l2_subdev *sd, s32 val)
{
//=== This function seems not used. ===//
	return 0;
}

static int m12mo_get_iso_mode(struct v4l2_subdev *sd, s32 *val)
{
	// struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret;

	ret = m12mo_readw(sd, CATEGORY_EXIF, EXIF_INFO_ISO, val);
	printk("get ISO mode result = %d",*val);
	return 0;
}

static const unsigned short ce_lut[][2] = {
	{ V4L2_COLORFX_NONE,     COLOR_EFFECT_NONE},
	{ V4L2_COLORFX_NEGATIVE, COLOR_EFFECT_NEGATIVE},
	{ V4L2_COLORFX_WARM,     COLOR_EFFECT_WARM},
	{ V4L2_COLORFX_COLD,     COLOR_EFFECT_COLD},
	{ V4L2_COLORFX_WASHED,   COLOR_EFFECT_WASHED},
};

static const unsigned short cbcr_lut[][3] = {
	{ V4L2_COLORFX_SEPIA,   COLOR_CFIXB_SEPIA,   COLOR_CFIXR_SEPIA},
	{ V4L2_COLORFX_BW,      COLOR_CFIXB_BW,      COLOR_CFIXR_BW},
	{ V4L2_COLORFX_RED,     COLOR_CFIXB_RED,     COLOR_CFIXR_RED},
	{ V4L2_COLORFX_GREEN,   COLOR_CFIXB_GREEN,   COLOR_CFIXR_GREEN},
	{ V4L2_COLORFX_BLUE,    COLOR_CFIXB_BLUE,    COLOR_CFIXR_BLUE},
	{ V4L2_COLORFX_PINK ,   COLOR_CFIXB_PINK,    COLOR_CFIXR_PINK},
	{ V4L2_COLORFX_YELLOW,  COLOR_CFIXB_YELLOW,  COLOR_CFIXR_YELLOW},
	{ V4L2_COLORFX_PURPLE,  COLOR_CFIXB_PURPLE,  COLOR_CFIXR_PURPLE},
	{ V4L2_COLORFX_ANTIQUE, COLOR_CFIXB_ANTIQUE, COLOR_CFIXR_ANTIQUE},
};

static int m12mo_set_cb_cr(struct v4l2_subdev *sd, u8 cfixb, u8 cfixr)
{
	int ret;

	ret = m12mo_writeb(sd, CATEGORY_MONITOR,
			   MONITOR_CFIXB, cfixb);
	if (ret)
		return ret;

	ret = m12mo_writeb(sd, CATEGORY_MONITOR,
			   MONITOR_CFIXR, cfixr);
	if (ret)
		return ret;

	ret = m12mo_writeb(sd, CATEGORY_MONITOR,
			   MONITOR_COLOR_EFFECT, COLOR_EFFECT_ON);
	return ret;
}

static int m12mo_set_color_effect(struct v4l2_subdev *sd, s32 val)
{
	int ret;

	switch (val) {
	case V4L2_COLORFX_NONE:
	    ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_EFFECT, NONE);
		break;
	case V4L2_COLORFX_SEPIA:
	    ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_EFFECT, SEPIA);
		break;
	case V4L2_COLORFX_BW:
	    ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_EFFECT, MONO);
		break;
	case V4L2_COLORFX_NEGATIVE:
	    ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_EFFECT, NEGATIVE);
		break;
	default:
		ret = -EINVAL;
		break;
	};

	return ret;
}

static int m12mo_set_color_effect_cbcr(struct v4l2_subdev *sd, s32 val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int ret;
	u8 cr, cb;

	cr = val & 0xff;
	cb = (val >> 8) & 0xff;

	if (dev->colorfx->cur.val == V4L2_COLORFX_SET_CBCR) {
		ret = m12mo_set_cb_cr(sd, cb, cr);
		if (ret)
			return ret;
	}

	dev->colorfx_cr = cr;
	dev->colorfx_cb = cb;

	return 0;
}

static int m12mo_get_focal(struct v4l2_subdev *sd, s32 *val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 optical_step;
	printk(KERN_INFO "@%s \n", __func__);
	while(dev->wait_irq_flag == M12MO_ZOOMING){
	      msleep(1);
	      printk(KERN_INFO "@%s: Warning!!! WAIT! Still moving zoom les!!!!! Cannot get f number. \n", __func__);
	}

	(void) m12mo_readb(sd, CATEGORY_MONITOR, OPTICAL_ZOOM, &optical_step);
	*val = (s32) m12mo_focus_data_table[optical_step].focal_len;
	return 0;
}

static int m12mo_get_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	u32 optical_step;
	printk(KERN_INFO "@%s \n", __func__);
	while(dev->wait_irq_flag == M12MO_ZOOMING){
	      msleep(1);
	      printk(KERN_INFO "@%s: Warning!!! WAIT! Still moving zoom les!!!!! Cannot get f number. \n", __func__);
	}
	(void) m12mo_readb(sd, CATEGORY_MONITOR, OPTICAL_ZOOM, &optical_step);
	*val = (s32) m12mo_focus_data_table[optical_step].f_number;
	return 0;
}

static int m12mo_get_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (M12MO_F_NUMBER_DEFAULT_NUM << 24) |
		(M12MO_F_NUMBER_DEM << 16) |
		(M12MO_F_NUMBER_DEFAULT_NUM << 8) | M12MO_F_NUMBER_DEM;
	return 0;
}

static int m12mo_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct m12mo_device *dev = container_of(
		ctrl->handler, struct m12mo_device, ctrl_handler);
	int ret = 0;
	printk("@%s ctrl->id = %d\n", __func__, ctrl->id);
	if (!dev->power)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_POWER_LINE_FREQUENCY:
	/*	We need to set Flicker frequence mode
		Please verify this. */
		ret = m12mo_set_flicker_freq(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_METERING:
	/*	We use EXT_ISP_CID_GET_METERING_MODE/EXT_ISP_CID_SET_METERING_MODE
	*/
		ret = m12mo_set_metering(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_START_ZSL_CAPTURE:
		if (ctrl->val)
			ret = m12mo_set_zsl_capture(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		printk("DDDD V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE (%d)",ctrl->val);
		ret = m12mo_set_white_balance(&dev->sd, ctrl->val);
		ret = 0;
		break;
	case V4L2_CID_EXPOSURE:
		ret = m12mo_set_ev_bias(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_ISO_SENSITIVITY:
	/*	Should be OK now.
		We use 0 for V4L2_CID_ISO_SENSITIVITY with iso-AUTO.
	*/  printk("DDDD V4L2_CID_ISO_SENSITIVITY (%d)",ctrl->val);
		ret = m12mo_set_iso_sensitivity(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_ISO_SENSITIVITY_AUTO:
	/*	Have no idea about this control.
		We use 0 in V4L2_CID_ISO_SENSITIVITY with iso-AUTO.
	*/  printk("DDDD V4L2_CID_ISO_SENSITIVITY_AUTO (%d)",ctrl->val);
		ret = m12mo_set_iso_mode(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_COLORFX:
	    printk("DDDD V4L2_CID_COLORFX (%d)",ctrl->val);
		ret = m12mo_set_color_effect(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_COLORFX_CBCR:
		ret = m12mo_set_color_effect_cbcr(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = dev->fw_ops->test_pattern(&dev->sd, ctrl->val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static const u32 m12mo_mipi_freq[] = {
	M12MO_MIPI_FREQ_0,
	M12MO_MIPI_FREQ_1,
};

static int m12mo_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct m12mo_device *dev = container_of(
		ctrl->handler, struct m12mo_device, ctrl_handler);
	int ret = 0;
	int id = M12MO_GET_MIPI_FREQ_MODE(dev->fw_type);

	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		ctrl->val = m12mo_mipi_freq[id];
		break;
	case V4L2_CID_EXPOSURE:
		ret = m12mo_get_ev_bias(&dev->sd, &ctrl->val);
		break;
	case V4L2_CID_FOCAL_ABSOLUTE:
		ret = m12mo_get_focal(&dev->sd, &ctrl->val);
		break;
	case V4L2_CID_FNUMBER_ABSOLUTE:
		ret = m12mo_get_fnumber(&dev->sd, &ctrl->val);
		break;
	case V4L2_CID_FNUMBER_RANGE:
		ret = m12mo_get_fnumber_range(&dev->sd, &ctrl->val);
		break;
	case V4L2_CID_ISO_SENSITIVITY:
	/*	We get iso from here (For capture)
		Need change this method from FW.
	*/
		ret = m12mo_get_iso_mode(&dev->sd, &ctrl->val);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct v4l2_ctrl_ops m12mo_ctrl_ops = {
	.g_volatile_ctrl = m12mo_g_volatile_ctrl,
	.s_ctrl = m12mo_s_ctrl,
};

/* TODO: To move this to s_ctrl framework */
static int m12mo_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	int index;
	int mode = M12MO_GET_RESOLUTION_MODE(dev->fw_type);

    printk("@%s %d start, mode = %d, run_mode = 0x%x\n", __func__, __LINE__, mode, param->parm.capture.capturemode);
	mutex_lock(&dev->input_lock);
	dev->run_mode = param->parm.capture.capturemode;
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
	    index = M12MO_MODE_VIDEO_INDEX;
		break;
	case CI_MODE_STILL_CAPTURE:
		index = M12MO_MODE_CAPTURE_INDEX;
		break;
	default:
		index = M12MO_MODE_PREVIEW_INDEX;
		break;
	}
    if(PREVIEW_TEST_PATTERN)
        index = M12MO_MODE_TEST_PATTERN_INDEX;
	dev->entries_curr_table = resolutions_sizes[index];
	dev->curr_res_table = resolutions[index];

	mutex_unlock(&dev->input_lock);
	return 0;
}

static long m12mo_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
			void *arg)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct atomisp_ext_isp_ctrl *m12mo_ctrl
		= (struct atomisp_ext_isp_ctrl *)arg;
	int ret = 0;
	int read = 0;

    if(PREVIEW_TEST_PATTERN){
        printk("%s PREVIEW_TEST_PATTERN, no ioctl\n", __func__);
        return ret;
    }
	printk("dev->m12mo_mode == M12MO_PARAMETER_MODE (%d) / (%d)\n",dev->m12mo_mode == M12MO_PARAMETER_MODE,dev->m12mo_mode);
	printk("m12mo ioctl id is %d, data 0x%x, dev->wait_irq_flag is %d\n",m12mo_ctrl->id, m12mo_ctrl->data, dev->wait_irq_flag);
#if 0
	if(dev->wait_irq_flag){
	    printk(KERN_INFO "m12mo_ioctl, dev->wait_irq_flag is %d \n", dev->wait_irq_flag);
	    if(m12mo_ctrl->id == EXT_ISP_CID_FOCUS_EXECUTION
		   || m12mo_ctrl->id == EXT_ISP_CID_TOUCH_POSX
		   || m12mo_ctrl->id == EXT_ISP_CID_TOUCH_POSY
		   || m12mo_ctrl->id == EXT_ISP_CID_TOUCH_WIDTH
		   || m12mo_ctrl->id == EXT_ISP_CID_TOUCH_HEIGHT
		   || m12mo_ctrl->id == EXT_ISP_CID_ZOOM){
		       return -EBUSY;
		}
	}
#endif

	mutex_lock(&dev->input_lock);
	switch(m12mo_ctrl->id)
	{
	case EXT_ISP_CID_ISO:
		dev_info(&client->dev, "m12mo ioctl ISO\n");
		break;
	case EXT_ISP_CID_CAPTURE_HDR:
		ret = m12mo_set_hdr_mode(sd, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_CAPTURE_LLS:

		break;
	case EXT_ISP_CID_FOCUS_MODE:
	/*	We need to set AF mode (like Smart AF/CAF/infinity/MANUAL)
		Maybe it need modify Gategory 04/0x3B*/

		ret = m12mo_set_af_mode(sd, m12mo_ctrl->data);
//Workaround first
		printk("FORCE EXT_ISP_CID_FOCUS_MODE (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_FOCUS_EXECUTION:
        printk("FORCE EXT_ISP_CID_FOCUS_EXECUTION (%d)",m12mo_ctrl->data);

        if(m12mo_ctrl->data != EXT_ISP_FOCUS_SEARCH){
	       ret = -EINVAL;
		   goto m12mo_ioctl_out;
	    }
        if(m12mo_capture_pre_flag){
	       ret = -EINVAL;
		   goto m12mo_ioctl_out;
	    }
	    if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	        pr_info("%s, illegal to set AF during M12MO_PARAMETER_MODE, break!\n", __func__);
		    ret = -EINVAL;
			goto m12mo_ioctl_out;
        }
		ret = 0;
		ret = m12mo_set_af_execution(sd, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_TOUCH_POSX:
	/*	We need to set Focus X position
		Maybe it need modify Gategory 04/0x17-0x18*/

		ret = m12mo_set_af_position_x(sd, m12mo_ctrl->data);
//Workaround first
		printk("DDDD FORCE EXT_ISP_CID_TOUCH_POSX (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_TOUCH_POSY:
	/*	We need to set Focus Y position
		Maybe it need modify Gategory 04/0x19-0x1A*/

		ret = m12mo_set_af_position_y(sd, m12mo_ctrl->data);
//Workaround first
		printk("DDDD FORCE EXT_ISP_CID_TOUCH_POSY (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_TOUCH_WIDTH:
	/*	We need to set focus width
		Maybe it need modify Gategory 04/0x34-0x35*/

		ret = m12mo_set_af_width(sd, m12mo_ctrl->data);
//Workaround first
		printk("DDDD FORCE EXT_ISP_CID_TOUCH_WIDTH (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_TOUCH_HEIGHT:
	/*	We need to set focus height
		Maybe it need modify Gategory 04/0x36-0x37*/

		ret = m12mo_set_af_height(sd, m12mo_ctrl->data);
//Workaround first
		printk("DDDD FORCE EXT_ISP_CID_TOUCH_HEIGHT (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_TOUCH_AE_POSX:
	/*	We need to set AE position X */

		ret = m12mo_set_ae_position_x(sd, m12mo_ctrl->data);
//Workaround first
		printk("DDDD FORCE EXT_ISP_CID_TOUCH_AE_POSX (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_TOUCH_AE_POSY:
	/*	We need to set AE position Y */

		ret = m12mo_set_ae_position_y(sd, m12mo_ctrl->data);
//Workaround first
		printk("DDDD FORCE EXT_ISP_CID_TOUCH_AE_POSY (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_CAF_STATUS:
		ret = m12mo_get_caf_status(sd, &m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_AF_STATUS:
	/*	We need to get now AF status
		Maybe it need to check */

		ret = m12mo_get_af_status(sd, &m12mo_ctrl->data);
		printk("DDDD FORCE EXT_ISP_CID_AF_STATUS");
		ret = 0;

		break;
	case EXT_ISP_CID_DIGITAL_ZOOM_STATUS:
        printk("DDDD FORCE EXT_ISP_CID_DIGITAL_ZOOM_STATUS");
		ret = m12mo_get_digital_zoom_status(sd, &m12mo_ctrl->data);
	    break;
	case EXT_ISP_CID_GET_AF_MODE:
	/*	We need to set AF mode (like Smart AF/CAF/infinity/MANUAL)
		Maybe it need modify Gategory 04/0x3B*/

		ret = m12mo_get_af_mode(sd, &m12mo_ctrl->data);
//Workaround first
		printk("DDDD FORCE EXT_ISP_FOCUS_MODE_NORMAL (%d)",m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_CAPTURE_BURST:
	/*	We need to set now Burst Capture mode (for 5 frame to super resolution)
		Maybe it need modify Gategory 0C/0x0A ?*/
		printk("DDDD force EXT_ISP_CID_CAPTURE_BURST (%d)", m12mo_ctrl->data);
		if( m12mo_ctrl->data > 0 ){
			num_LLS = m12mo_ctrl->data;
		}else{
			num_LLS = 0;
		}
//		ret = m12mo_set_burst_mode(sd, m12mo_ctrl->data);
        ret = m12mo_set_lls_mode(sd, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_FLASH_MODE:
	/*	We need to set Flash mode (AUTO/OFF/ON/TORCH)
		Maybe it need modify Gategory 04/0x15 or Gategory 0D/0xF0*/
/*
#define EXT_ISP_FLASH_MODE_OFF		0
#define EXT_ISP_FLASH_MODE_ON		1
#define EXT_ISP_FLASH_MODE_AUTO		2
#define EXT_ISP_LED_TORCH_OFF		3 //We don't send this parameter, using EXT_ISP_FLASH_MODE_OFF instead.
#define EXT_ISP_LED_TORCH_ON		4
*/
		ret = m12mo_set_flash_mode(sd, m12mo_ctrl->data);
		printk("DDDD FORCE EXT_ISP_CID_FLASH_MODE (%d)",m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_ZOOM:  //=== Optical-zoom. ===//
	    printk(KERN_INFO "DDDD FORCE EXT_ISP_CID_ZOOM (%d) \n", m12mo_ctrl->data);

		if(dev->m12mo_mode == M12MO_PARAMETER_MODE) {
		    ret = -EINVAL;
		    goto m12mo_ioctl_out;
		}
        if(m12mo_ctrl->data > 32 || m12mo_ctrl->data < 1){
            pr_info("m12mo, wrong optical zoom data is %d \n", m12mo_ctrl->data);
            ret = -EINVAL;
            goto m12mo_ioctl_out;
	    }

		while(dev->requested_cmd == M12MO_START_DIGITAL_ZOOM) {
		    pr_info("%s, Digital-zooming, Wait!!!!!\n", __func__);
			msleep(5);
		}
		ret = m12mo_readb(sd, CATEGORY_MONITOR, DIGIT_ZOOM, &read);
        if(read != 0x1){
            pr_info("%s, get digital-zoom step is %d, illegal to set Optical-zoom, break!\n", __func__, read);
//	        ret = -EINVAL;
//		    goto m12mo_ioctl_out;
            read = 0x1;
            (void) m12mo_request_cmd_effect(sd, M12MO_START_DIGITAL_ZOOM, &read);
	    }

		if(m12mo_capture_pre_flag){
	       ret = -EINVAL;
		   goto m12mo_ioctl_out;
	    }

	    if(dev->m12mo_mode == M12MO_PARAMETER_MODE){
	        pr_info("%s, illegal to set Optical-zoom during M12MO_PARAMETER_MODE, break!\n", __func__);
		    ret = -EINVAL;
			goto m12mo_ioctl_out;
	    }

		ret = 0;
#if 0
		ret = m12mo_readb(sd, CATEGORY_MONITOR, OPTICAL_ZOOM, &read);
	    if(read == m12mo_ctrl->data){
		    pr_info("%s, read_step is %d, step is %d, no need to set again! \n", __func__, read, m12mo_ctrl->data);
		}
#endif
#if 0
		cancel_delayed_work(&optical_zoom_work);
		cancel_delayed_work(&auto_focus_work);

		if (dev->wait_irq_flag == M12MO_FOCUSING) {
			dev->cancel_irq_flag = 1;
		    __m12mo_reset_irq_wait_status();//(dev, &lens_work);
            queue_delayed_work(dev->cancel_auto_focus_wq, &cancel_auto_focus_work, 0);
			queue_delayed_work(dev->lens_wq, &dummy_work, 0);
		}else if (dev->wait_irq_flag == M12MO_ZOOMING) {
		    dev->cancel_irq_flag = 1;
		    __m12mo_reset_irq_wait_status();//(dev, &lens_work);
		    queue_delayed_work(dev->cancel_optical_zoom_wq, &cancel_optical_zoom_work, 0);
		    queue_delayed_work(dev->lens_wq, &dummy_work, 0);
		}
		m12mo_ctrl->data = clamp_t(unsigned int, m12mo_ctrl->data,
						ZOOM_POS_MIN, ZOOM_POS_MAX);
		ret = m12mo_set_optical_zoom_position(sd, m12mo_ctrl->data);
#endif
		break;
	case EXT_ISP_CID_SHOT_MODE:
/*
#define EXT_ISP_SHOT_MODE_AUTO 0
#define EXT_ISP_SHOT_MODE_BEAUTY_FACE 1
#define EXT_ISP_SHOT_MODE_BEST_PHOTO 2
#define EXT_ISP_SHOT_MODE_DRAMA 3
#define EXT_ISP_SHOT_MODE_BEST_FACE 4
#define EXT_ISP_SHOT_MODE_ERASER 5
#define EXT_ISP_SHOT_MODE_PANORAMA 6
#define EXT_ISP_SHOT_MODE_RICH_TONE_HDR 7
#define EXT_ISP_SHOT_MODE_NIGHT 8
#define EXT_ISP_SHOT_MODE_SOUND_SHOT 9
#define EXT_ISP_SHOT_MODE_ANIMATED_PHOTO 10
#define EXT_ISP_SHOT_MODE_SPORTS 11
#define EXT_ISP_SHOT_MODE_PRO 12
#define EXT_ISP_SHOT_MODE_LOWLIGHT 13
#define EXT_ISP_SHOT_MODE_TIMEREWIND 14
#define EXT_ISP_SHOT_MODE_SELFIE 15
#define EXT_ISP_SHOT_MODE_MINIATURE 16
#define EXT_ISP_SHOT_MODE_DEFOCUS 17
#define EXT_ISP_SHOT_MODE_PANOSELFIE 18
#define EXT_ISP_SHOT_MODE_SUPERRESOLUTION 19
#define EXT_ISP_SHOT_MODE_NORMAL 20
#define EXT_ISP_SHOT_MODE_EFFECT 21
#define EXT_ISP_SHOT_MODE_CONTINUOUS 22
#define EXT_ISP_VIDEO_MODE_NORMAL 40
#define EXT_ISP_VIDEO_MODE_PRO 41
#define EXT_ISP_VIDEO_MODE_MMS 42
#define EXT_ISP_VIDEO_MODE_MINIATURE 43
#define EXT_ISP_VIDEO_MODE_SLOWMOTION 44
#define EXT_ISP_VIDEO_MODE_HISPEED 45
#define EXT_ISP_VIDEO_MODE_TIMELAPSE 46
#define EXT_ISP_VIDEO_MODE_LOWLIGHT 47
#define EXT_ISP_VIDEO_MODE_EFFECT 48
*/
		/*	We need to set Asus Camera mode  */
		printk("DDDD FORCE EXT_ISP_CID_SHOT_MODE (%d)",m12mo_ctrl->data);
		ret = m12mo_set_shot_mode(sd, m12mo_ctrl->data);
		break;
	case EXT_ISP_M10MO_SET_CAPTURE_MODE:
	        //=== Hard code to set ois s2 mode. ===//
#ifndef CONFIG_ASUS_FACTORY_MODE
#if 0
      (void) m12mo_writew(sd, CATEGORY_TEST, OIS_RAMREG_ADDR_H, 0x0110);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_DATA31_24, 0x00);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_WRITE_READ_TRIG, 0x11);
		msleep(10);
      (void) m12mo_writew(sd, CATEGORY_TEST, OIS_RAMREG_ADDR_H, 0x0107);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_DATA31_24, 0x11);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_WRITE_READ_TRIG, 0x11);
		msleep(10);
      (void) m12mo_writew(sd, CATEGORY_TEST, OIS_RAMREG_ADDR_H, 0x0102);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_DATA31_24, 0x02);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_WRITE_READ_TRIG, 0x11);
		msleep(10);
      (void) m12mo_writew(sd, CATEGORY_TEST, OIS_RAMREG_ADDR_H, 0x0110);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_DATA31_24, 0x03);
      (void) m12mo_writeb(sd, CATEGORY_TEST, OIS_WRITE_READ_TRIG, 0x11);
		msleep(10);
#endif
#endif
	    m12mo_capture_pre_flag = M12MO_CAP_BEFORE_1ST_STREAMOFF_1;
		ret = 0;
		break;
	case EXT_ISP_CID_GET_METERING_MODE:
	/*	We need to get now AE metering mode (Average(0x00)/Center(0x01)/Spot(0x02))
		Maybe it need modify Gategory 04/0x16  */
		m12mo_get_metering(sd, &m12mo_ctrl->data);
		printk("EXT_ISP_CID_GET_METERING_MODE (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_SET_METERING_MODE:
	/*	We need to get now AE metering mode (Average(0x00)/Center(0x01)/Spot(0x02))
		Maybe it need modify Gategory 04/0x16*/
		m12mo_set_metering(sd, m12mo_ctrl->data);
		printk("EXT_ISP_CID_SET_METERING_MODE (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_EIS_MODE:
	/*	We need to set EIS to AE (0/1)
		It need to check wit DIT
		*/
		printk("DDDD force EXT_ISP_CID_EIS_MODE (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_VIDEO_OPTIMIZE:
	/*	We need to set video AE mode
		Just bypass the HAL value.
		*/
		printk("EXT_ISP_CID_VIDEO_OPTIMIZE (0x%x)", m12mo_ctrl->data);
	    ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_AE_SCENE_MODE,m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SHUTTER_SPEED:
		printk("DDDD force EXT_ISP_CID_SHUTTER_SPEED (%d)", m12mo_ctrl->data);
		ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_SHUTTER_SPEED, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_COLOR_TEMPERATURE:
	/*	We need to set color temperature to AWB
		It need to check wit DIT.
		If value = 0 , it means auto.
		*/
		printk("DDDD force EXT_ISP_CID_COLOR_TEMPERATURE (%d)", m12mo_ctrl->data);
		ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_COLOR_TEMPERATURE, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_MANUAL_FOCUS:
	/*	We need to set lens postion for focus
		It need to check wit DIT.
		If value = 0 , it means auto.
		*/
		printk("EXT_ISP_CID_MANUAL_FOCUS (%d)\n", m12mo_ctrl->data);
		if(m12mo_ctrl->data != 0){
			ret = m12mo_readb(sd, CATEGORY_ASUS, ASUS_FOCUS_MODE, &read);
			if(read != 3){
				printk("EXT_ISP_CID_MANUAL_FOCUS now FOCUS_MODE = %d, force to MANUAL_MODE.\n", read);
				ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_FOCUS_MODE, 3);
			}
			ret = m12mo_request_cmd_effect(sd, M12MO_MANUAL_FOCUS, &m12mo_ctrl->data);
		}
		break;
	case EXT_ISP_CID_SATURATION:
	/*	We need to set saturation
		It need to check wit DIT.
		The value should be  value/100.
		*/
		printk("EXT_ISP_CID_SATURATION (%d)", m12mo_ctrl->data);
		ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_SATURATION, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_CONTRAST:
	/*	We need to set saturation
		It need to check wit DIT.
		The value should be  value/100.
		*/
		printk("EXT_ISP_CID_CONTRAST (%d)", m12mo_ctrl->data);
		ret = m12mo_writeb(sd, CATEGORY_ASUS, ASUS_CONTRAST, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_EXPOSURETIME_NUMERATOR:
		ret = m12mo_get_exposure_time_numerator(sd, &m12mo_ctrl->data);
		printk("EXT_ISP_CID_EXPOSURETIME_NUMERATOR (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_EXPOSURETIME_DENOMINATOR:
		ret = m12mo_get_exposure_time_denominator(sd, &m12mo_ctrl->data);
		printk("EXT_ISP_CID_EXPOSURETIME_DENOMINATOR (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_PREVIEW_EXPOSURETIME_NUMERATOR:
		ret = m12mo_get_preview_exposure_time_numerator(sd, &m12mo_ctrl->data);
		printk("EXT_ISP_CID_PREVIEW_EXPOSURETIME_NUMERATOR (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_PREVIEW_EXPOSURETIME_DENOMINATOR:
		ret = m12mo_get_preview_exposure_time_denominator(sd, &m12mo_ctrl->data);
		printk("EXT_ISP_CID_PREVIEW_EXPOSURETIME_DENOMINATOR (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_PREVIEW_ISO:
		ret = m12mo_get_preview_iso(sd, &m12mo_ctrl->data);
		printk("EXT_ISP_CID_PREVIEW_ISO (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_AT_SCENE:
		ret = m12mo_writew(sd, CATEGORY_ASUS, ASUS_AT_SCENE, m12mo_ctrl->data);
		printk("EXT_ISP_CID_AT_SCENE (%d)", m12mo_ctrl->data);
		ret = 0;
		break;
	case EXT_ISP_CID_GET_AF_RESULT:
	    ret = m12mo_get_af_result(sd, &m12mo_ctrl->data);
        printk("EXT_ISP_CID_GET_AF_RESULT (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_M10MO_I2C_ONE_BYTE_WRITE:
        printk("EXT_ISP_CID_M10MO_I2C_ONE_BYTE_WRITE (%d)", m12mo_ctrl->data);
		if(dev->wait_irq_flag != M12MO_IRQ_COMMAND_AVAILABDLE) {
		    printk("EXT_ISP_CID_M10MO_I2C_ONE_BYTE_WRITE, dev->wait_irq_flag is %d, illegal for HAL to do i2c-write, skip this cmd.\n", dev->wait_irq_flag);
			if(dev->wait_irq_flag == M12MO_IRQ_COMMAND_AVAILABDLE) {
                while (1){
                    printk(KERN_INFO "Warning!!! EXT_ISP_CID_M10MO_I2C_ONE_BYTE_WRITE, it is ILLIGAL to set i2c cmds during irq_staus is busy!!!\n");
                    msleep(1);
                }
			}
		    ret = -EINVAL;
		    goto m12mo_ioctl_out;
		}
        ret = m12mo_set_hal_i2c_write(sd, &m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_CONTINUOUS_CAPTURE:
	    printk("EXT_ISP_CID_CONTINUOUS_CAPTURE (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_M10MO_I2C_ONE_BYTE_READ:
	    printk("Before EXT_ISP_CID_M10MO_I2C_ONE_BYTE_READ (%d)", m12mo_ctrl->data);
	    ret = m12mo_get_hal_i2c_read(sd, &m12mo_ctrl->data);
		printk("After EXT_ISP_CID_M10MO_I2C_ONE_BYTE_READ (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_BATTERY_CAPACITY:
	    printk("EXT_ISP_CID_SET_BATTERY_CAPACITY (%d)", m12mo_ctrl->data);
	    ret = m12mo_set_battery_capacity(sd, &m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_3A_LOCK:
	    printk("EXT_ISP_CID_SET_3A_LOCK (%d)", m12mo_ctrl->data);
		ret = m12mo_set_3A_lock(sd, &m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_GET_FLASH_NEEDED:
	    ret = m12mo_get_asus_flash_needed(sd, &m12mo_ctrl->data);
		printk("EXT_ISP_CID_GET_FLASH_NEEDED (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_DIGITAL_ZOOM:
	    printk("EXT_ISP_CID_SET_DIGITAL_ZOOM (%d)", m12mo_ctrl->data);
		ret = m12mo_set_digital_zoom_position(sd, m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_ASUS_CAMERA:
	    printk("EXT_ISP_CID_SET_ASUS_CAMERA (%d)", m12mo_ctrl->data);
	    break;
	case EXT_ISP_CID_GET_M10MO_IRQ_STATUS:
		ret = m12mo_get_irq_status(sd, &m12mo_ctrl->data);
	    printk("EXT_ISP_CID_GET_M10MO_IRQ_STATUS (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_M10MO_START_CAPTURE:
	    printk("EXT_ISP_CID_SET_M10MO_START_CAPTURE (0x%x)", m12mo_ctrl->data);
        ret = 0;
		break;
    case EXT_ISP_CID_SET_M10MO_START_PREVIEW:
	    printk("EXT_ISP_CID_SET_M10MO_START_PREVIEW (%d)", m12mo_ctrl->data);
        ret = 0;
		break;
	case EXT_ISP_CID_GET_DIT_LV:
		ret = m12mo_get_LV(sd, &m12mo_ctrl->data);
	    printk("EXT_ISP_CID_GET_DIT_LV (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_GET_DIT_R_GAIN:
		ret = m12mo_get_R_GAIN(sd, &m12mo_ctrl->data);
	    printk("EXT_ISP_CID_GET_DIT_R_GAIN (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_GET_DIT_B_GAIN:
		ret = m12mo_get_B_GAIN(sd, &m12mo_ctrl->data);
	    printk("EXT_ISP_CID_GET_DIT_B_GAIN (%d)", m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_DIT_R_GAIN:
	    printk("EXT_ISP_CID_SET_DIT_R_GAIN (%d)", m12mo_ctrl->data);
		ret = m12mo_set_R_GAIN(sd, &m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_DIT_B_GAIN:
	    printk("EXT_ISP_CID_SET_DIT_B_GAIN (%d)", m12mo_ctrl->data);
		ret = m12mo_set_B_GAIN(sd, &m12mo_ctrl->data);
		break;
	case EXT_ISP_CID_SET_G_SENSOR_X:
	    printk("EXT_ISP_CID_SET_G_SENSOR_X (%d)", m12mo_ctrl->data);
		ret = m12mo_set_g_sensor_x(sd, &m12mo_ctrl->data);
        break;
	case EXT_ISP_CID_SET_G_SENSOR_Y:
	    printk("EXT_ISP_CID_SET_G_SENSOR_Y (%d)", m12mo_ctrl->data);
		ret = m12mo_set_g_sensor_y(sd, &m12mo_ctrl->data);
        break;
	case EXT_ISP_CID_SET_G_SENSOR_Z:
	    printk("EXT_ISP_CID_SET_G_SENSOR_Y (%d)", m12mo_ctrl->data);
		ret = m12mo_set_g_sensor_z(sd, &m12mo_ctrl->data);
        break;
	default:
		ret = -EINVAL;
		dev_err(&client->dev, "m12mo ioctl: Unsupported ID\n");
		break;
	};

m12mo_ioctl_out:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static const struct v4l2_ctrl_config ctrls[] = {
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_LINK_FREQ,
		.name = "Link Frequency",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 1,
		.max = 1500000 * 1000,
		.step = 1,
		.def = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE | V4L2_CTRL_FLAG_READ_ONLY,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.name = "Light frequency filter",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 3,
		.max = 3,
		.step = 1,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_METERING,
		.name = "Metering",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = 2,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_START_ZSL_CAPTURE,
		.name = "Start zsl capture",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 1,
		.max = 4,
		.step = 1,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.name = "White Balance, Auto & Preset",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = V4L2_WHITE_BALANCE_MANUAL,
		.def = V4L2_WHITE_BALANCE_AUTO,
		.max = V4L2_WHITE_BALANCE_SHADE,
		.step = 1,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure Bias",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = M12MO_MIN_EV,
		.def = 0,
		.max = M12MO_MAX_EV,
		.step = 1,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_ISO_SENSITIVITY,
		.name = "Iso",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 0,
		.max = 3200,
		.step = 1,
		.flags = V4L2_CTRL_FLAG_VOLATILE,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_ISO_SENSITIVITY_AUTO,
		.name = "Iso mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = V4L2_ISO_SENSITIVITY_MANUAL,
		.def = V4L2_ISO_SENSITIVITY_AUTO,
		.max = V4L2_ISO_SENSITIVITY_AUTO,
		.step = 1,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.name = "Image Color Effect",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = V4L2_COLORFX_NONE,
		.def = V4L2_COLORFX_NONE,
		.max = V4L2_COLORFX_PURPLE,
		.step = 1,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_COLORFX_CBCR,
		.name = "Image Color Effect CbCr",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 0,
		.max = 0xffff,
		.step = 1,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_TEST_PATTERN,
		.name = "Test Pattern (Color Bar)",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 0,
		.max = 1,
		.step = 1
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_FOCAL_ABSOLUTE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "focal length",
		.min = M12MO_FOCAL_LENGTH_DEFAULT,
		.max = M12MO_FOCAL_LENGTH_DEFAULT,
		.step = 1,
		.def = M12MO_FOCAL_LENGTH_DEFAULT,
		.flags = 0,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_FNUMBER_ABSOLUTE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "f-number",
		.min = M12MO_F_NUMBER_DEFAULT,
		.max = M12MO_F_NUMBER_DEFAULT,
		.step = 1,
		.def = M12MO_F_NUMBER_DEFAULT,
		.flags = 0,
	},
	{
		.ops = &m12mo_ctrl_ops,
		.id = V4L2_CID_FNUMBER_RANGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "f-number range",
		.min = M12MO_F_NUMBER_RANGE,
		.max =  M12MO_F_NUMBER_RANGE,
		.step = 1,
		.def = M12MO_F_NUMBER_RANGE,
		.flags = 0,
	},
};

static int __m12mo_init_ctrl_handler(struct m12mo_device *dev)
{
	struct v4l2_ctrl_handler *hdl;
	int ret, i;

	hdl = &dev->ctrl_handler;

	ret = v4l2_ctrl_handler_init(&dev->ctrl_handler, ARRAY_SIZE(ctrls));
	if (ret)
		return ret;

	for (i = 0; i < ARRAY_SIZE(ctrls); i++)
		v4l2_ctrl_new_custom(&dev->ctrl_handler, &ctrls[i], NULL);

	if (dev->ctrl_handler.error) {
		ret = dev->ctrl_handler.error;
		v4l2_ctrl_handler_free(&dev->ctrl_handler);
		return ret;
	}

	dev->ctrl_handler.lock = &dev->input_lock;
	dev->sd.ctrl_handler = hdl;
	v4l2_ctrl_handler_setup(hdl);

	dev->link_freq = v4l2_ctrl_find(&dev->ctrl_handler, V4L2_CID_LINK_FREQ);
	if (NULL == dev->link_freq)
		return -ENODEV;
	v4l2_ctrl_s_ctrl(dev->link_freq, V4L2_CID_LINK_FREQ);

	dev->zsl_capture = v4l2_ctrl_find(&dev->ctrl_handler,
					V4L2_CID_START_ZSL_CAPTURE);
	if (NULL == dev->zsl_capture)
		return -ENODEV;
	v4l2_ctrl_s_ctrl(dev->zsl_capture, V4L2_CID_START_ZSL_CAPTURE);

	dev->colorfx = v4l2_ctrl_find(&dev->ctrl_handler,
				      V4L2_CID_COLORFX);
	return 0;
}

static int m12mo_s_routing(struct v4l2_subdev *sd, u32 input, u32 output, u32 config)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct atomisp_camera_caps *caps =
		dev->pdata->common.get_camera_caps();

    printk("%s start\n", __func__);
	/* Select operating sensor. */
	if (caps->sensor_num > 1) {
		m12mo_writeb(sd, CATEGORY_ASUS, ASUS_PHONE_DIRECTION, output);

		return m12mo_writeb(sd, CATEGORY_SYSTEM,
			SYSTEM_MASTER_SENSOR, !output);
	}

	return 0;
}

static int m12mo_s_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *interval)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int fps = 0;

	mutex_lock(&dev->input_lock);
	if (interval->interval.numerator != 0)
		fps = interval->interval.denominator / interval->interval.numerator;
	if (!fps) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}
	dev->fps = fps;
	dev_dbg(&client->dev, "%s: fps is %d\n", __func__, dev->fps);
	mutex_unlock(&dev->input_lock);

	return 0;
}

int m12mo_g_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *interval)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);

	mutex_lock(&dev->input_lock);
	interval->interval.denominator = 30;
	interval->interval.numerator = 1;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static const struct v4l2_subdev_video_ops m12mo_video_ops = {
	.try_mbus_fmt = m12mo_try_mbus_fmt,
	.s_mbus_fmt = m12mo_set_mbus_fmt,
	.g_mbus_fmt = m12mo_get_mbus_fmt,
	.s_stream = m12mo_s_stream,
	.s_parm = m12mo_s_parm,
	.enum_framesizes = m12mo_enum_framesizes,
	.s_routing = m12mo_s_routing,
	.s_frame_interval = m12mo_s_frame_interval,
	.g_frame_interval = m12mo_g_frame_interval,
};

static const struct v4l2_subdev_core_ops m12mo_core_ops = {
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.s_power = m12mo_s_power,
	.init	= m12mo_fw_start,
	.ioctl  = m12mo_ioctl,
};

static const struct v4l2_subdev_pad_ops m12mo_pad_ops = {
	.enum_mbus_code = m12mo_enum_mbus_code,
	.enum_frame_size = m12mo_enum_frame_size,
	.get_fmt = m12mo_get_pad_format,
	.set_fmt = m12mo_set_pad_format,

};

static int m12mo_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct m12mo_device *dev = to_m12mo_sensor(sd);
	pr_info("%s, run mode: 0x%x\n", __func__, dev->run_mode);

	mutex_lock(&dev->input_lock);

#ifdef off_c51
    *frames = 0;
#else
    if (dev->run_mode == CI_MODE_STILL_CAPTURE)
		*frames = 0;
    else
        *frames = 1;
#endif
	mutex_unlock(&dev->input_lock);

	return 0;
}

static const struct v4l2_subdev_sensor_ops m12mo_sensor_ops = {
	.g_skip_frames	= m12mo_g_skip_frames,
};

static const struct v4l2_subdev_ops m12mo_ops = {
	.core	= &m12mo_core_ops,
	.pad	= &m12mo_pad_ops,
	.video	= &m12mo_video_ops,
	.sensor = &m12mo_sensor_ops,
};

static int dump_fw(struct m12mo_device *dev)
{
	int ret = 0;
	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		ret = -EBUSY;
		goto leave;
	}
	__m12mo_s_power(&dev->sd, 1, true);
	m12mo_dump_fw(dev);
	__m12mo_s_power(&dev->sd, 0, true);
leave:
	mutex_unlock(&dev->input_lock);
	return ret;
}

#if 0
static int read_fw_checksum(struct m12mo_device *dev, u16 *result)
{
	int ret;

	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		ret = -EBUSY;
		goto leave;
	}
	__m12mo_s_power(&dev->sd, 1, true);
	ret = m12mo_fw_checksum(dev, result);
	__m12mo_s_power(&dev->sd, 0, true);
leave:
	mutex_unlock(&dev->input_lock);
	return ret;
}
#endif

static int read_fw_version(struct m12mo_device *dev, char *buf)
{
	int ret;

	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		ret = -EBUSY;
		goto leave;
	}
	__m12mo_s_power(&dev->sd, 1, true);
	ret = m12mo_get_isp_fw_version_string(dev, buf, M12MO_MAX_FW_ID_STRING,
			dev->fw_addr_id);
	__m12mo_s_power(&dev->sd, 0, true);
leave:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int update_fw(struct m12mo_device *dev, int erase_all)
{
	/*struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);*/
	int ret = 0;
	u16 result = 0;

	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		printk("%s %d\t power: %d !!!!!!!!!!!!! \n",__func__,__LINE__,dev->power);
		ret = -EBUSY;
		goto leave;
	}
	__m12mo_s_power(&dev->sd, 1, true);
	if(erase_all == 0)
		ret = m12mo_program_device(dev, 0);
	else if(erase_all == 1)
		ret = m12mo_program_device_erase_all_flash_partial(dev);
	else if(erase_all == 2)
		ret = m12mo_program_device_erase_all_flash_all(dev);
	else{
		__m12mo_s_power(&dev->sd, 0, true);
		goto leave;
	}
	__m12mo_s_power(&dev->sd, 0, true);
    if(dev->ver.checksum)
        goto leave;

    msleep(200);
    __m12mo_s_power(sdd, 1, false);
    m12mo_detect_info(sdd);
    __m12mo_s_power(sdd, 0, false);
#if 0
	if (ret)
		goto leave;
	/* Power cycle chip and re-identify the version */
	__m12mo_s_power(&dev->sd, 1, true);
	ret = m12mo_identify_fw_type(&dev->sd);
	__m12mo_s_power(&dev->sd, 0, true);
	if (ret)
		goto leave;

	__m12mo_s_power(&dev->sd, 1, true);
	ret = m12mo_fw_checksum(dev, &result);
	__m12mo_s_power(&dev->sd, 0, true);
	dev_info(&client->dev, "m12mo FW checksum: %d\n", result);
#endif

leave:
	if (ret || result!=0) isFlashFwFail = true;
	else isFlashFwFail = false;

	mutex_unlock(&dev->input_lock);
	return ret;
}

static void auto_Update_fw(struct work_struct *ws)
{
    struct i2c_client *client = v4l2_get_subdevdata(sdd);
    struct m12mo_device *dev = to_m12mo_sensor(sdd);
    int ret = 0, timeout=6000;
    u32 dit_fw_file_version=0, sni_fw_file_version=0;

    printk("@%s start\n", __func__);
	m12mo_USB_status(1);
	wake_lock_timeout(&dev->m12mo_update_wake_lock, msecs_to_jiffies(180000));
	mutex_lock(&dev->input_lock);

	while ((dev->power == 1) && --timeout){
		printk("m12mo Auto FW upgrade timeout = %x !!!!!!!!!!!!!!!!!!!!!\n", timeout);
		msleep(10);
	}
	if (!timeout) {
		printk("m12mo Auto FW upgrade fail!!!!!!!!!!!!!!!!!!!!!\n");
		goto leave;
	}

    __m12mo_s_power(sdd, 1, false);
    m12mo_detect_info(sdd);
    __m12mo_s_power(sdd, 0, false);

	isp_update_status = 0;

    if(dev->ver.checksum)
        goto flashfw;
    else if(!isNeedUpdateFwWhenBoot())
        goto leave;
    else if(BIT(0) & dev->ver.fw_auto_flash_flag){
        printk("%s fw_auto_flash_flag = 0x%x\n", __func__, dev->ver.fw_auto_flash_flag);
        goto leave;
    }
    else if(readFwFileVersion(&dit_fw_file_version, &sni_fw_file_version)){
        goto leave;
    }
    else{
        dev_info(&client->dev, "m12mo version : %x-%x\n", dev->ver.firmware, dev->ver.DIT_firmware);
        dev_info(&client->dev, "m12mo read /etc/firmware/M12MO.version : %x-%x\n", sni_fw_file_version, dit_fw_file_version);
    }

    isp_update_status = 1;

    if(sni_fw_file_version < dev->ver.firmware){
        dev_info(&client->dev, "%x < %x : exit!\n", sni_fw_file_version, dev->ver.firmware);
        goto leave;
    }else if(sni_fw_file_version == dev->ver.firmware && dit_fw_file_version <= dev->ver.DIT_firmware){
        dev_info(&client->dev, "%x < %x : exit!\n", dit_fw_file_version, dev->ver.DIT_firmware);
        goto leave;
    }else{
        goto flashfw;
    }

flashfw:
    printk("%s start flashfw in status %d\n", __func__, isp_update_status);
    msleep(200);
    __m12mo_s_power(sdd, 1, true);
    ret = m12mo_program_device(dev, 1);
    __m12mo_s_power(sdd, 0, true);

    if(dev->ver.checksum)
        goto leave;
    //read fw version
    msleep(200);
    __m12mo_s_power(sdd, 1, false);
    m12mo_detect_info(sdd);
    __m12mo_s_power(sdd, 0, false);

leave:
    printk("%s leave in status %d\n", __func__, isp_update_status);
    dev_info(&client->dev, "m12mo auto_Update_fw: %d\n", ret);
    mutex_unlock(&dev->input_lock);
    wake_unlock(&dev->m12mo_update_wake_lock);
    m12mo_USB_status(0);
}
static DECLARE_DELAYED_WORK(auto_Update_fw_work, auto_Update_fw);

static ssize_t m12mo_flash_rom_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "flash\n");
}
static ssize_t m12mo_flash_rom_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	int ret=0;
	wake_lock_timeout(&m12mo_dev->m12mo_update_wake_lock, msecs_to_jiffies(180000));
	if (!strncmp(buf, "flash", 5) || !strncmp(buf, "eraseall_flash", 14) || !strncmp(buf, "erase_flash_all", 15)) {

	    m12mo_USB_status(1);

        if (!strncmp(buf, "flash", 5)){
            printk("m12mo flash\n");
            ret=update_fw(m12mo_dev, 0);
        }else if (!strncmp(buf, "eraseall_flash", 14)){
            printk("m12mo eraseall_flash\n");
            ret=update_fw(m12mo_dev, 1);
        }else if (!strncmp(buf, "erase_flash_all", 15)){
            printk("m12mo erase_flash_all\n");
            ret=update_fw(m12mo_dev, 2);
        }else{
            printk("m12mo EINVAL\n");
            wake_unlock(&m12mo_dev->m12mo_update_wake_lock);
            return -EINVAL;
        }
#if 0
        msleep(200);
        if(!ret){

            m12mo_s_power_fac(1);
            DIT_version = writeDummyVersion(m12mo_dev);
            m12mo_s_power_fac(0);

            mdelay(200);
            __m12mo_s_power(sdd, 1, false);

            m12mo_set_zsl_monitor(sdd);
            msleep(3000);
            __m12mo_s_power(sdd, 0, false);

            //read fw version
            msleep(200);
            m12mo_s_power_fac(1);
            DIT_version = readDitVersion(m12mo_dev);
            m12mo_s_power_fac(0);
        }
#endif
        m12mo_USB_status(0);
        wake_unlock(&m12mo_dev->m12mo_update_wake_lock);
        return len;
    }
    wake_unlock(&m12mo_dev->m12mo_update_wake_lock);
	return -EINVAL;
}
static DEVICE_ATTR(isp_flashfw, S_IRUGO | S_IWUSR, m12mo_flash_rom_show,
		   m12mo_flash_rom_store);

static ssize_t m12mo_i2c_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "\n \
		sensor i2c status\n\
			0: I2C success\n\
			1: I2C fail\n\
		%d Rear Camera\n\
		%d Front Camera\n\
		%d Laser\n\
		%d RGB sensor(3593)\n\
		%d RGB sensor(3323)\n\
		%d OIS driver\n\
		%d VCM driver\n\
		%d Flash driver\n", i2c_check_status[7], i2c_check_status[6], i2c_check_status[5], i2c_check_status[4]
						  , i2c_check_status[3], i2c_check_status[2], i2c_check_status[1], i2c_check_status[0]);
}

static ssize_t m12mo_i2c_status_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	m12mo_i2c_check_status(&m12mo_dev->sd);

	return len;	
}

static DEVICE_ATTR(isp_sensor_i2c_status, S_IRUGO | S_IWUSR, m12mo_i2c_status_show, m12mo_i2c_status_store);

static ssize_t m12mo_rgb_ctrl_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	int ret = 0;
	u32 ctrl = 0,ctrl1 = 0;

    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_MONITOR, RGB_ENABLE, &ctrl);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0x2C, &ctrl1);
    printk("@%s %d, ctrl = %x, ctrl1 = %x\n", __func__, __LINE__, ctrl, ctrl1);

	return scnprintf(buf, PAGE_SIZE, "\n\
		rgb enable: \n \
		arg1: \n \
			bit7: \n \
			0: CM3323 \n \
			1: CM3593 \n \
			bit6_4: \n \
			0x0:Enable/ integration time 40ms \n \
			0x1:Enable/ integration time 80ms \n \
			0x2:Enable/ integration time 160ms \n \
			0x3:Enable/ integration time 320ms \n \
			0x4:Enable/ integration time 640ms \n \
			0x5:Enable/ integration time 1280ms \n \
			bit0: \n \
			0: Disable \n \
			1: Enable \n \
		arg2: \n \
			bit0: \n \
			0: Disable calibration\n \
			1: Enable calibration\n \
		rgb result = 0x%x 0x%x\n", ctrl, ctrl1);
}

static ssize_t m12mo_rgb_ctrl_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	u32 ctrl = 0, ctrl1 = 0;
	int ret = 0;
	sscanf(buf, "%x %x", &ctrl, &ctrl1);
	printk("@%s %d, rgb enable = 0x%x , calibration = 0x%x\n", __func__, __LINE__, ctrl, ctrl1);

	ret = m12mo_writeb(&m12mo_dev->sd, CATEGORY_MONITOR, RGB_ENABLE, ctrl);
	if (ret)
	{
		printk("@%s %d, m12mo_writeb fail = %d", __func__, __LINE__, ret);
        return -EINVAL;
	}

	ret = m12mo_writeb(&m12mo_dev->sd, CATEGORY_ASUS, 0x2C, ctrl1);
	if (ret)
	{
		printk("@%s %d, m12mo_writeb fail = %d", __func__, __LINE__, ret);
        return -EINVAL;
	}
	return len;	
}
static DEVICE_ATTR(isp_rgb_ctrl, S_IRUGO | S_IWUSR, m12mo_rgb_ctrl_show, m12mo_rgb_ctrl_store);


static ssize_t m12mo_rgb_calib_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "\n\
		rgb calibration: \n");
}

static ssize_t m12mo_rgb_calib_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
//	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	u32 reg[4];
//	int ret = 0;
	sscanf(buf, "%x,%x,%x,%x", &reg[0], &reg[1], &reg[2], &reg[3]);
	printk("@%s %d, rgb golden RGBW = 0x%x, 0x%x, 0x%x, 0x%x\n", __func__, __LINE__, reg[0], reg[1], reg[2], reg[3]);

#if 0// write golden RGBW to flash rom map
	// write  golden RGBW value to flash rom
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x0012, reg_val );   // golden data R
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x0014, reg_val );   // golden data G
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x0016, reg_val );   // golden data B
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x0018, reg_val );   // golden data W

		// 	由Catgory 4 讀出R (2 byte) G (2 byte) B (2 byte) W(2 byte)  寫入 flash map
	// write calibation data to flash rom
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x001A, reg_val );   // calibration data R
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x001C, reg_val );   // calibration data G
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x001E, reg_val );   // calibration data B
	ret = m12mo_write_Flashrom_data( &m12mo_dev->sd, 0x0a, 0x0020, reg_val );   // calibration data W

	if (ret)
	{
		printk("@%s %d, m12mo_write_Flashrom_data fail = %d", __func__, __LINE__, ret);
		return -EINVAL;
	}
#endif
	return len;	
}
static DEVICE_ATTR(isp_rgb_calib, S_IRUGO | S_IWUSR, m12mo_rgb_calib_show, m12mo_rgb_calib_store);


static ssize_t m12mo_rgb_data_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	int ret = 0;
	u32 ctrl = 0;
	u32 reg[4] = {0};
	u32 reg1[8] = {0};
	u32 reg2[4] = {0};
	u32 reg3[4] = {0};

    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_MONITOR, RGB_DATA31_24, &reg[3]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_MONITOR, RGB_DATA23_16, &reg[2]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_MONITOR, RGB_DATA15_8, &reg[1]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_MONITOR, RGB_DATA7_0, &reg[0]);

#if 1   // dit category 4 
// R
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb0, &reg1[1]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb1, &reg1[0]);
    
// G
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb2, &reg1[3]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb3, &reg1[2]);
    
// B
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb4, &reg1[5]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb5, &reg1[4]);
    
// W
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb6, &reg1[7]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xb7, &reg1[6]);
    
// CT
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0x4C, &reg2[0]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0x4D, &reg2[1]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0x4E, &reg2[2]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0x4F, &reg2[3]);

// lux
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xC4, &reg3[0]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xC5, &reg3[1]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xC6, &reg3[2]);
    ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0xC7, &reg3[3]);
#endif


    ctrl = reg[0] << 8 | reg[1];

	return scnprintf(buf, PAGE_SIZE, "\n\
		rgb read data: \n \
			bit7: \n \
			0: CM3323 \n \
			1: CM3593 \n \
			bit3_0: \n \
			0x0: R data \n \
			0x1: G data \n \
			0x2: B data \n \
			0x3: W data \n \
			0x4: Lux (Asus handle RGB_Lux) \n \
			0x5: CCT (Asus handle RGB_CCT) \n \
		result = %d , 0x%x %x %x %x\n \
		(R)0x%2x%2x (G)0x%2x%2x (B)0x%2x%2x (W)0x%2x%2x (CT)0x%2x%2x%2x%2x (LUX)0x%2x%2x%2x%2x \n", ctrl, reg[3],reg[2],reg[0],reg[1]
		 ,reg1[1], reg1[0], reg1[3], reg1[2], reg1[5], reg1[4], reg1[7], reg1[6]
		 ,reg2[3], reg2[2], reg2[1], reg2[0] 
		 ,reg3[3], reg3[2], reg3[1], reg3[0] );
}

static ssize_t m12mo_rgb_data_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	u32 ctrl;
	int ret = 0;
	sscanf(buf, "%x", &ctrl);
	printk("@%s %d, rgb enable = 0x%x\n", __func__, __LINE__, ctrl);

	ret = m12mo_writeb(&m12mo_dev->sd, CATEGORY_MONITOR, RGB_READ_DATA, ctrl);
	if (ret)
	{
		printk("@%s %d, m12mo_writeb fail = %d", __func__, __LINE__, ret);
		if( ctrl != 0xFF )
        	return -EINVAL;
	}
	return len;	
}
static DEVICE_ATTR(isp_rgb_data, S_IRUGO | S_IWUSR, m12mo_rgb_data_show, m12mo_rgb_data_store);
static ssize_t m12mo_flash_spi_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE,
			 "%d\n", m12mo_get_spi_state(m12mo_dev));
}
static ssize_t m12mo_flash_spi_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	ret = m12mo_set_spi_state(m12mo_dev, value);
	if (ret)
		return ret;
	return len;
}
static DEVICE_ATTR(isp_spi, S_IRUGO | S_IWUSR, m12mo_flash_spi_show,
		   m12mo_flash_spi_store);

static ssize_t m12mo_flash_checksum_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%04x\n", m12mo_dev->ver.checksum);
}
static DEVICE_ATTR(isp_checksum, S_IRUGO, m12mo_flash_checksum_show, NULL);

static ssize_t m12mo_flash_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m10_dev = dev_get_drvdata(dev);
	ssize_t ret;

	ret  = read_fw_version(m10_dev, buf);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%s\n", buf);
}
static DEVICE_ATTR(isp_fw_id, S_IRUGO, m12mo_flash_version_show, NULL);

static ssize_t m12mo_flash_customer_project_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE, "%x\n", m12mo_dev->ver.customer);
}
static DEVICE_ATTR(isp_fw_customer_project, S_IRUGO, m12mo_flash_customer_project_show, NULL);

static ssize_t m12mo_flash_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m10_dev = dev_get_drvdata(dev);
	dump_fw(m10_dev);
	return scnprintf(buf, PAGE_SIZE, "done\n");
}
static DEVICE_ATTR(isp_fw_dump, S_IRUGO, m12mo_flash_dump_show, NULL);

static ssize_t m12mo_debug(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    	u32 val=0;

	sscanf(buf, "%x", &val);
	m12mo_break_log_loop = val;

	return len;
}
static DEVICE_ATTR(isp_fw_debug, S_IRUGO | S_IWUSR, NULL, m12mo_debug);

static ssize_t m12mo_read_Flag(struct device *dev, struct device_attribute *attr, char *buf)
{
    struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
    return scnprintf(buf, PAGE_SIZE, "\nflag is %x\n", m12mo_dev->ver.fw_auto_flash_flag);
}
static ssize_t m12mo_write_Flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	int ret=0, res=0, timeout=5000;
    u32 val=0;

	sscanf(buf, "%x", &val);

	mutex_lock(&m12mo_dev->input_lock);
    (void) m12mo_s_power_fac(1);

	(void) m12mo_writew(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H, 0xf4);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, val);

	if(ret == 1){
		pr_err("timeout while waiting for chip op to finish\n");
		return 0xffff;
	}
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_MODE, 0x09);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_FLASH_MODE, 0x03);

	do {
		msleep(10);
		m12mo_readb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, SFLASH_SPI_STATUS, &res);
	} while ((res != 0) && --timeout);

	if (!timeout) {
		pr_err("timeout while waiting for chip op to finish\n");
		return 0xffff;
	}
	printk("flash done\n");
    m12mo_dev->ver.fw_auto_flash_flag = val;

    (void) m12mo_s_power_fac(0);
	mutex_unlock(&m12mo_dev->input_lock);
    printk("%s write flag %x\n", __func__, m12mo_dev->ver.fw_auto_flash_flag);

	return len;
}
static DEVICE_ATTR(isp_fw_default_flag, S_IRUGO | S_IWUSR, m12mo_read_Flag, m12mo_write_Flag);

static ssize_t m12mo_read_download_trace_log(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d", m12mo_trace_log_lock);
}
static ssize_t m12mo_write_download_trace_log(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    //=== Only tow kinds of value could be set in this file node. ===//
	u32 val;
	sscanf(buf, "%d", &val);
	if(val != DOWNLOAD_AVAILABLE
	     && val != DOWNLOAD_PROTECT) {
	    return len;
	}
	m12mo_trace_log_lock = val;

	return len;
}
static DEVICE_ATTR(m12mo_download_trace_log_when_time_out, S_IRUGO | S_IWUSR, m12mo_read_download_trace_log, m12mo_write_download_trace_log);

static ssize_t m12mo_read_asus_camera_flag(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%d", asus_camera_flag);
}
static ssize_t m12mo_write_asus_camera_flag(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	u32 val;
	sscanf(buf, "%d", &val);
	asus_camera_flag = val;

	return len;
}
static DEVICE_ATTR(m12mo_asus_camera_flag, S_IRUGO, m12mo_read_asus_camera_flag, m12mo_write_asus_camera_flag);

#ifndef ZX551ML_USER_BUILD //#ifdef CONFIG_ASUS_FACTORY_MODE

static ssize_t m12mo_shading_table_checksum_read(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	mutex_lock(&m12mo_dev->input_lock);
	if (m12mo_dev->power == 1) {
		mutex_unlock(&m12mo_dev->input_lock);
		return -EBUSY;
	}

	__m12mo_s_power(&m12mo_dev->sd, 1, true);
	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, 0x69, &val);
	__m12mo_s_power(&m12mo_dev->sd, 0, true);

	if (ret) {
		mutex_unlock(&m12mo_dev->input_lock);
        return -EINVAL;
	}

	mutex_unlock(&m12mo_dev->input_lock);
        printk("@%s %d, shading table checksum = %x\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "shading table checksum = %x\n", val);
}
static DEVICE_ATTR(isp_shading_table_checksum, S_IRUGO | S_IWUSR, m12mo_shading_table_checksum_read, NULL);

static ssize_t m12mo_digital_zoom_read(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_MONITOR, DIGIT_ZOOM, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, digital zoom position = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "digital zoom position = %d\n", val);
}
static ssize_t m12mo_digital_zoom_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
    printk("@%s %d, digital zoom position %d\n", __func__, __LINE__, val);

    if(0x0 < val && val < 0x20 ){
        ret = m12mo_request_cmd_effect(&m12mo_dev->sd, M12MO_START_DIGITAL_ZOOM, &val);
        if (ret < 0)
            return ret;
        return len;
    }else{
	    	printk("@%s %d, wrong zoom position %d\n", __func__, __LINE__, val);
		return -EINVAL;
	}


}
static DEVICE_ATTR(isp_digital_zoom, S_IRUGO | S_IWUSR, m12mo_digital_zoom_read, m12mo_digital_zoom_write);

static ssize_t m12mo_i2c_debug_byte_write_cat(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
//	if(password_debug != 0x3296) return -1;

	ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_MONITOR, OPTICAL_ZOOM, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, zoom step = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m12mo_i2c_debug_byte_write_echo(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u8 cat, reg_addr;
	u32 val, reg_val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
//	if(password_debug != 0x3296) return -1;

	sscanf(buf, "%x", &val);
    reg_val  = val & 0xFF;
	reg_addr = (val >> 8) & 0xFF;
	cat      = (val >> 16) & 0xFF;
    printk(KERN_INFO "ASUSBSP --- @m12mo_i2c_debug_byte_write_echo, cat is 0x%x, reg_addr is 0x%x, reg_val is 0x%x \n",
                                  cat, reg_addr, reg_val);

    ret = m12mo_writeb(&m12mo_dev->sd, cat, reg_addr, reg_val);
    if (!ret){
	    printk(KERN_INFO "ASUSBSP --- @m12mo_i2c_debug_byte_write_echo \n");
        return len;
	}else{
	    return -EINVAL;
    }
}
static DEVICE_ATTR(m12mo_i2c_debug_byte_write_change_name_by_jevian, S_IRUGO | S_IWUSR, m12mo_i2c_debug_byte_write_cat, m12mo_i2c_debug_byte_write_echo);

static ssize_t m12mo_debug_state_read(struct device *dev,	struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	printk("@%s %d, \n", __func__, __LINE__);
	return scnprintf(buf, PAGE_SIZE, "dev->requested_cmd is %d, dev->cmd is %d, m12mo_capture_pre_flag is %d \n", m12mo_dev->requested_cmd, m12mo_dev->cmd, m12mo_capture_pre_flag);
}
static ssize_t m12mo_debug_state_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%d", &val);
    m12mo_dev->cmd = val;

    return len;
}
static DEVICE_ATTR(m12mo_i2c_debug_state, S_IRUGO | S_IWUSR, m12mo_debug_state_read, m12mo_debug_state_write);

static ssize_t m12mo_i2c_debug_byte_read_cat(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_MONITOR, OPTICAL_ZOOM, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, zoom step = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m12mo_i2c_debug_byte_read_echo(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u8 cat, reg_addr;
	u32 val;
	u32 reg_val = 0;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
//    reg_val  = val & 0xFF;
	reg_addr = (val >> 8) & 0xFF;
	cat      = (val >> 16) & 0xFF;
    printk(KERN_INFO "ASUSBSP --- @m12mo_i2c_debug_byte_read_echo, cat is 0x%x, reg_addr is 0x%x \n",
                                  cat, reg_addr);

    ret = m12mo_readb(&m12mo_dev->sd, cat, reg_addr, &reg_val);
    if (!ret){
	    printk(KERN_INFO "ASUSBSP --- @m12mo_i2c_debug_byte_read_echo, reg_val is 0x%x\n", reg_val);
        return len;
	}else{
	    return -EINVAL;
    }
}
static DEVICE_ATTR(m12mo_i2c_debug_byte_read_change_name_by_jevian, S_IRUGO | S_IWUSR, m12mo_i2c_debug_byte_read_cat, m12mo_i2c_debug_byte_read_echo);

static ssize_t m12mo_read_IQ_calibration_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	mutex_lock(&m12mo_dev->input_lock);
    (void) m12mo_s_power_fac(1);
    	(void)__m12mo_param_mode_set(&m12mo_dev->sd);
	m12mo_isp_fw_IQ_R(m12mo_dev);
    (void) m12mo_s_power_fac(0);
	mutex_unlock(&m12mo_dev->input_lock);
	return 0;
}
static ssize_t m12mo_write_IQ_calibration_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	int res=0, timeout=5000;

	mutex_lock(&m12mo_dev->input_lock);
    	(void) m12mo_s_power_fac(1);

    	(void)__m12mo_param_mode_set(&m12mo_dev->sd);

	m12mo_isp_fw_IQ_W(m12mo_dev);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_FLASH_MODE, 0x30);

	do {
		msleep(10);
		m12mo_readb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, SFLASH_SPI_STATUS, &res);
	} while ((res != 0) && --timeout);

	if (!timeout) {
		printk("timeout while waiting for chip op to finish\n");
		return len;
	}
	printk("flash done\n");

    	(void) m12mo_s_power_fac(0);
	mutex_unlock(&m12mo_dev->input_lock);

	return len;
}
static DEVICE_ATTR(isp_fw_iq_RW, S_IRUGO | S_IWUSR, m12mo_read_IQ_calibration_data, m12mo_write_IQ_calibration_data);

static ssize_t m12mo_asus_awb_control_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val1, reg_val2, ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	printk( "ASUS AWB balance CONTROL \n \
			Byte 71: AWB_DEBUG1 \n \
			Byte 72: AWB_DEBUG2 \n \
			Byte 73: AWB_DEBUG3 \n \
			Byte 74: AWB_DEBUG4 \n \
			Byte 75: AWB_DEBUG5 \n \
			Byte 76: AWB_DEBUG6 \n \
			Byte 77: AWB_DEBUG7 \n \
			Byte 78: AWB_DEBUG8 \n \
					\n");

	ret = m12mo_readl(&m12mo_dev->sd, CATEGORY_WB, REG_AWB_DEBUG1, &reg_val1);
        if(ret)
            	goto out;
	printk("@%s %d, REG_AWB_DEBUG1 ~ REG_AWB_DEBUG4 = %x\n", __func__, __LINE__, reg_val1);

	ret = m12mo_readl(&m12mo_dev->sd, CATEGORY_WB, REG_AWB_DEBUG5, &reg_val2);
        if(ret)
            	goto out;
	printk("@%s %d, REG_AWB_DEBUG5 ~ REG_AWB_DEBUG8 = %x\n", __func__, __LINE__, reg_val2);

	return scnprintf(buf, PAGE_SIZE, "\n\
		ASUS AWB balance CONTROL \n \
			REG_AWB_DEBUG1 ~ REG_AWB_DEBUG4 = %x\n \
			REG_AWB_DEBUG5 ~ REG_AWB_DEBUG8 = %x\n \
					\n\n\n ", reg_val1, reg_val2);
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_asus_awb_control, S_IRUGO | S_IWUSR, m12mo_asus_awb_control_read, NULL);

static ssize_t m12mo_asus_awb_control_read_set(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "\n\
		set ASUS AWB balance CONTROL read Byte\n");
}
static DEVICE_ATTR(isp_asus_awb_control_set, S_IRUGO | S_IWUSR, m12mo_asus_awb_control_read_set, NULL);

static ssize_t m12mo_exif_info_exptime_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val[2],ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
#if 0
Unit : numerator / denominator (sec)
*For Example

numerator = 600(0x0000258)
denominator = 20000(0x00004E20)
600 / 2000 = 0.03sec
0x0000000 -> 30ms
#endif
    	ret = m12mo_readl(&m12mo_dev->sd, CATEGORY_EXIF, EXIF_INFO_EXPTIME_NU, &reg_val[0]);
        if(ret)
            goto out;
    	ret = m12mo_readl(&m12mo_dev->sd, CATEGORY_EXIF, EXIF_INFO_EXPTIME_DE, &reg_val[1]);
        if(ret)
            goto out;

	printk( "exif_info_exptime.\n\
		 numerator = %x\n \
		 denominatorF = %x\n \
		exposure time = %x\n", reg_val[0],reg_val[1],reg_val[0]/reg_val[1]);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 exif_info_exptime.\n\
		 numerator = %x\n \
		 denominatorF = %x\n\
		 exposure time = 0x%x\n" , reg_val[0],reg_val[1],reg_val[0]/reg_val[1]);
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_exif_info_exptime, S_IRUGO | S_IWUSR, m12mo_exif_info_exptime_read, NULL);

static ssize_t m12mo_exif_info_iso_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

    ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_EXIF, EXIF_INFO_ISO, &reg_val);
    if(ret)
        goto out;

	printk( "exif_info_ISO = %x\n", reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 exif_info_ISO = %x\n" , reg_val);
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_exif_info_iso, S_IRUGO | S_IWUSR, m12mo_exif_info_iso_read, NULL);

static ssize_t m12mo_read_power_ctrl(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk("@%s %d, \n", __func__, __LINE__);
	return scnprintf(buf, PAGE_SIZE, "power is %d, m12mo_trace_log_lock is %d \n", m12mo_dev->power, m12mo_trace_log_lock);
}
static ssize_t m12mo_write_power_ctrl(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	mutex_lock(&m12mo_dev->input_lock);
	sscanf(buf, "%d", &val);
    (void) m12mo_s_power_fac(val);
	mutex_unlock(&m12mo_dev->input_lock);

	return len;
}
static DEVICE_ATTR(m12mo_power_ctrl, S_IRUGO | S_IWUSR, m12mo_read_power_ctrl, m12mo_write_power_ctrl);

static ssize_t m12mo_debug_password_read(struct device *dev,	struct device_attribute *attr, char *buf)
{
	printk("@%s %d, \n", __func__, __LINE__);
	return scnprintf(buf, PAGE_SIZE, "password_debug is 0x%x \n", password_debug);
}
static ssize_t m12mo_debug_password_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	sscanf(buf, "%x", &password_debug);
    return len;
}
static DEVICE_ATTR(m12mo_debug_password, S_IRUGO | S_IWUSR, m12mo_debug_password_read, m12mo_debug_password_write);

static ssize_t m12mo_apk_capture_mode_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	val = m12mo_dev->capture_mode;
	printk("@%s %d, m12mo_dev->capture_mode = %d\n", __func__, __LINE__, val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m12mo_apk_capture_mode_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	m12mo_dev->capture_mode = val;
    (void)__m12mo_param_mode_set(&m12mo_dev->sd);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, val);
    (void) m12mo_request_cmd_effect(&m12mo_dev->sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
    printk("@%s %d, m12mo_dev->capture_mode %d\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_apk_capture_mode, S_IRUGO | S_IWUSR, m12mo_apk_capture_mode_read, m12mo_apk_capture_mode_write);

static ssize_t m12mo_distirtion_mode_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val, ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_PARAM, DISTORTION, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, distortion_mode = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "distortion_mode = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_distirtion_mode_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

    	(void)__m12mo_param_mode_set(&m12mo_dev->sd);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_PARAM, DISTORTION, val);
    	(void) m12mo_request_cmd_effect(&m12mo_dev->sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
    	printk("@%s %d, distortion_mode %x\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_distortion_mode, S_IRUGO | S_IWUSR, m12mo_distirtion_mode_read, m12mo_distirtion_mode_write);

static ssize_t m12mo_ois_data_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 ret, reg_val[9];
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA7_0, &reg_val[0]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data0 = %x\n", __func__, __LINE__, reg_val[0]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA15_8, &reg_val[1]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data1 = %x\n", __func__, __LINE__, reg_val[1]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA23_16, &reg_val[2]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data2 = %x\n", __func__, __LINE__, reg_val[2]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA31_24, &reg_val[3]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data3 = %x\n", __func__, __LINE__, reg_val[3]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA39_32, &reg_val[4]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data4 = %x\n", __func__, __LINE__, reg_val[4]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA47_40, &reg_val[5]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data5 = %x\n", __func__, __LINE__, reg_val[5]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA55_48, &reg_val[6]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data6 = %x\n", __func__, __LINE__, reg_val[6]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA63_56, &reg_val[7]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data7 = %x\n", __func__, __LINE__, reg_val[7]);

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA71_64, &reg_val[8]);
        if(ret)
            goto out;
	printk("@%s %d, ois_data8 = %x\n", __func__, __LINE__, reg_val[8]);


	return scnprintf(buf, PAGE_SIZE, "\n%x %x %x %x %x %x %x %x %x\n", reg_val[0]\
			, reg_val[1], reg_val[2], reg_val[3], reg_val[4], reg_val[5]\
			, reg_val[6], reg_val[7], reg_val[8]);
out:
	return -EINVAL;
}
static ssize_t m12mo_ois_data_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    	u32 reg_val[10];
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x %x %x %x %x %x %x %x %x", &reg_val[0]\
			, &reg_val[1], &reg_val[2], &reg_val[3], &reg_val[4], &reg_val[5]\
			, &reg_val[6], &reg_val[7], &reg_val[8]);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA7_0, reg_val[0]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA15_8, reg_val[1]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA23_16, reg_val[2]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA31_24, reg_val[3]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA39_32, reg_val[4]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA47_40, reg_val[5]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA55_48, reg_val[6]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA63_56, reg_val[7]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_DATA71_64, reg_val[8]);

    	printk("@%s %d, ois_data %x %x %x %x %x %x %x %x %x\n", __func__, __LINE__, reg_val[0]\
			, reg_val[1], reg_val[2], reg_val[3], reg_val[4], reg_val[5]\
			, reg_val[6], reg_val[7], reg_val[8]);

	return len;
}
static DEVICE_ATTR(isp_ois_data, S_IRUGO | S_IWUSR, m12mo_ois_data_read, m12mo_ois_data_write);

static ssize_t m12mo_ois_write_read_trig_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "OIS Write Read Start Triger\n \
		 0x00: non\n \
		 0x01: 1 byte Read\n \
		 0x02: 2 byte Read\n \
		 0x04: 4 byte Read\n \
		 0x11: 1 byte Write\n \
		 0x12: 2 byte Write\n \
		 0x14: 4 byte Write\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_WRITE_READ_TRIG, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, ois_write_read_trig = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 OIS Write Read Start Triger\n \
		 0x00: non\n \
		 0x01: 1 byte Read\n \
		 0x02: 2 byte Read\n \
		 0x04: 4 byte Read\n \
		 0x11: 1 byte Write\n \
		 0x12: 2 byte Write\n \
		 0x14: 4 byte Write\n \
		 ois_write_read_trig = 0x%x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_ois_write_read_trig_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	if(val < 0x00 || val > 0x14) goto out;
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_WRITE_READ_TRIG, val);
    	printk("@%s %d, ois_write_read_trig %x\n", __func__, __LINE__, val);

	return len;
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_ois_write_read_trig, S_IRUGO | S_IWUSR, m12mo_ois_write_read_trig_read, m12mo_ois_write_read_trig_write);

static ssize_t m12mo_ois_ramreg_addr_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "OIS Write Read Address H.\n");

    	ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_TEST, OIS_RAMREG_ADDR_H, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, ois_ramreg_addr_h = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "ois_ramreg_addr_h = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_ois_ramreg_addr_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	(void) m12mo_writew(&m12mo_dev->sd, CATEGORY_TEST, OIS_RAMREG_ADDR_H, val);
    	printk("@%s %d, ois_ramreg_addr_h %x\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_ois_ramreg_addr, S_IRUGO | S_IWUSR, m12mo_ois_ramreg_addr_read, m12mo_ois_ramreg_addr_write);

static ssize_t m12mo_ois_lib_api_start_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "Start the OIS API library.\n\
		 0x01: Start api library 1 (IniSet)\n\
		 0x02: Start api library 2 (OscAdj)\n\
		 0x03: Start api library 3 (TneRun)\n\
		 0x04: OIS OFF (RtnCen(0))\n\
		 0x05: OIS ON  (OisEna)\n\
		 0x06: SetOpt (x refer to byte 0x72-0x73, y refer to byte 0x74-0x75)\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_LIB_API_START, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, ois_lib_api_start = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 Start the OIS API library.\n\
		 0x01: Start api library 1 (IniSet)\n\
		 0x02: Start api library 2 (OscAdj)\n\
		 0x03: Start api library 3 (TneRun)\n\
		 0x04: OIS OFF (RtnCen(0))\n\
		 0x05: OIS ON  (OisEna)\n\
		 0x06: SetOpt (x refer to byte 0x72-0x73, y refer to byte 0x74-0x75)\n \
		 ois_lib_api_start = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_ois_lib_api_start_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val < 0x01 || val > 0x06) goto out;
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_LIB_API_START, val);
    	printk("@%s %d, ois_lib_api_start %x\n", __func__, __LINE__, val);

	return len;
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_ois_lib_api_start, S_IRUGO | S_IWUSR, m12mo_ois_lib_api_start_read, m12mo_ois_lib_api_start_write);

static ssize_t m12mo_ois_cali_start_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "OIS Calibration Request Start.\n\
		 1: Start\n\
		 0: Invaild\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_CALI_API_START, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, ois_cali_start = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 OIS Calibration Request Start.\n\
		 1: Start\n\
		 0: Invaild\n \
		 ois_cali_start = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_ois_cali_start_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val < 0x00 || val > 0x01) goto out;
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_CALI_API_START, val);
    	printk("@%s %d, ois_cali_start %x\n", __func__, __LINE__, val);

	return len;
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_ois_cali_start, S_IRUGO | S_IWUSR, m12mo_ois_cali_start_read, m12mo_ois_cali_start_write);

static ssize_t m12mo_ois_cali_result_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val[2],ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "please input data[15-0], eg, echo 0x5 0x6 --> 0x5 is H byte, 0x6 is L byte\n\
		 bit 0 = 1 : Serious error\n\
		 bit 1 = 1 : Process end\n\
		 bit 2 = 1 : Hall X offset/bias NG\n\
		 bit 3 = 1 : Hall Y offset/bias NG\n\
		 bit 4 = 1 : X Actuator gain NG\n\
		 bit 5 = 1 : Y Actuator gain NG\n\
		 bit 6 = 1 : X Gyro Offset NG\n\
		 bit 7 = 1 : Y Gyro Offset NG\n\
		 bit 8 = 1 : NG\n\
		 bit 9 = 1 : NG\n\
		 bit 10 = 1 : NG\n\
		 bit 11 = 1 : Hall X direction NG\n\
		 bit 11 = 1 : Hall Y direction NG\n\
		 All Pass = 0x0002\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_CALI_RESULT_7_0, &reg_val[1]);
        if(ret)
            goto out;
    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, OIS_CALI_RESULT_15_8, &reg_val[0]);
        if(ret)
            goto out;
	printk("@%s %d, ois_cali_result = %x%x\n", __func__, __LINE__, reg_val[0],reg_val[1]);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 please input data[15-0], eg, echo \"0x5 0x6\" > isp_ois_cali_resul --> 0x5 is H byte, 0x6 is L byte\n\
		 bit 0 = 1 : Serious error\n\
		 bit 1 = 1 : Process end\n\
		 bit 2 = 1 : Hall X offset/bias NG\n\
		 bit 3 = 1 : Hall Y offset/bias NG\n\
		 bit 4 = 1 : X Actuator gain NG\n\
		 bit 5 = 1 : Y Actuator gain NG\n\
		 bit 6 = 1 : X Gyro Offset NG\n\
		 bit 7 = 1 : Y Gyro Offset NG\n\
		 bit 8 = 1 : NG\n\
		 bit 9 = 1 : NG\n\
		 bit 10 = 1 : NG\n\
		 bit 11 = 1 : Hall X direction NG\n\
		 bit 11 = 1 : Hall Y direction NG\n\
		 All Pass = 0x0002\n \
		 ois_cali_result = 0x%x%x\n", reg_val[0],reg_val[1]);
out:
	return -EINVAL;
}
static ssize_t m12mo_ois_cali_result_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 reg_val[2];
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x %x", &reg_val[1], &reg_val[0]);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_CALI_RESULT_7_0, reg_val[1]);
    	printk("@%s %d, OIS_CALI_RESULT_7_0 %x\n", __func__, __LINE__, reg_val[1]);
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, OIS_CALI_RESULT_15_8, reg_val[0]);
    	printk("@%s %d, OIS_CALI_RESULT_15_8 %x\n", __func__, __LINE__, reg_val[0]);

	return len;
}
static DEVICE_ATTR(isp_ois_cali_result, S_IRUGO | S_IWUSR, m12mo_ois_cali_result_read, m12mo_ois_cali_result_write);


static ssize_t m12mo_ois_ctrl_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    u32 value,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "OIS Enable/Disable\n\
		0x00 disable\n \
		0x01 enable\n");

	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_MONITOR, OIS_ENABLE, &value);
	if(ret)
	{
		printk("@%s %d, fail = %d", __func__, __LINE__, ret);
        return -EINVAL;
	}	
	printk("@%s %d, ois result = %x\n", __func__, __LINE__, value);

	return scnprintf(buf, PAGE_SIZE, " OIS Enable/Disable\n\
	0x00 disable\n \
	0x01 enable\n \
		result = 0x%x\n", value);
}

static ssize_t m12mo_ois_ctrl_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 value;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &value);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_MONITOR, OIS_ENABLE, value);
    printk("@%s %d, OIS_ENABLE %x\n", __func__, __LINE__, value);

	return len;
}
static DEVICE_ATTR(isp_ois_ctrl, S_IRUGO | S_IWUSR, m12mo_ois_ctrl_read, m12mo_ois_ctrl_write);

static ssize_t m12mo_cap_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "Capture LDC SPR ON/OFF SW.\n\
		 0x00: LDC ON / SPR ON\n\
		 0x01: LDC OFF / SPR ON\n\
		 0x02: LDC OFF / SPR OFF\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, CAP_TEST, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, cap_test = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 Capture LDC SPR ON/OFF SW.\n\
		 0x00: LDC ON / SPR ON\n\
		 0x01: LDC OFF / SPR ON\n\
		 0x02: LDC OFF / SPR OFF\n\n\n\
		 cap_test = 0x%x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_cap_test_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val < 0x00 || val > 0x02) goto out;
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, CAP_TEST, val);
    	printk("@%s %d, cap_test %x\n", __func__, __LINE__, val);

	return len;
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_cap_test, S_IRUGO | S_IWUSR, m12mo_cap_test_read, m12mo_cap_test_write);

static ssize_t m12mo_pr_test_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "PR Test for LiveView.\n\
		 0x01: Start PR Detection\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, PR_TEST, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, pr_test = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 PR Test for LiveView.\n\
		 0x01: Start PR Detection\n\
		 pr_test = 0x%x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_pr_test_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val < 0x00 || val > 0x01) goto out;
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, PR_TEST, val);
    	printk("@%s %d, pr_test %x\n", __func__, __LINE__, val);

	return len;
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_pr_test, S_IRUGO | S_IWUSR, m12mo_pr_test_read, m12mo_pr_test_write);

static ssize_t m12mo_pr_led_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "LED On/Off for PR.\n\
		 0x00: LED2(zoom) OFF & LED1 (Focus) OFF\n \
		 0x01: LED2(zoom) OFF & LED1 (Focus) ON\n \
		 0x10: LED2(zoom) ON  & LED1 (Focus) OFF\n \
		 0x11: LED2(zoom) ON  & LED1 (Focus) ON\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, PR_LED, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, pr_led = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 LED On/Off for PR.\n\
		 0x00: LED2(zoom) OFF & LED1 (Focus) OFF\n \
		 0x01: LED2(zoom) OFF & LED1 (Focus) ON\n \
		 0x10: LED2(zoom) ON  & LED1 (Focus) OFF\n \
		 0x11: LED2(zoom) ON  & LED1 (Focus) ON\n\
		 pr_led = 0x%x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_pr_led_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, PR_LED, val);
    	printk("@%s %d, pr_led %x\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_pr_led, S_IRUGO | S_IWUSR, m12mo_pr_led_read, m12mo_pr_led_write);

static ssize_t m12mo_led_status_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_data, reg_status,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);



	//set address
    	ret = m12mo_writeb(&m12mo_dev->sd, CATEGORY_ASUS, LED_ERROR_ADDR, 0x09);
        if(ret)
            goto out;

	//read data from sky81296
    	ret = m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, LED_ERROR_WRITE_READ_TRIG, 0x1);
        if(ret)
            goto out;

	//get data from m12mo
    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, LED_ERROR_DATA, &reg_data);
        if(ret)
            goto out;


	//get status from m12mo
    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, LED_ERROR_STATUS, &reg_status);
        if(ret)
            goto out;

	printk("@%s %d, addr = 0x09\n", __func__, __LINE__);
	printk("@%s %d, data = %x\n", __func__, __LINE__, reg_data);
	printk("@%s %d, status = %x\n", __func__, __LINE__, reg_status);


	return scnprintf(buf, PAGE_SIZE, "\n\
		 addr = 0x09\n \
		 data = %x\n \
		 status = %x\n", reg_data, reg_status);
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_led_status, S_IRUGO | S_IWUSR, m12mo_led_status_read, NULL);

static ssize_t m12mo_maunal_focus_ctrl_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "setting Range: 0x0002-0x1fff, \n");

    	ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_TEST, MANUAL_FOCUS_CTRL_H, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, maunal_focus_ctrl = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		setting Range: 0x0002-0x1fff, \n\n\n\
		maunal_focus_ctrl = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_maunal_focus_ctrl_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    	u32 reg_val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &reg_val);

    	(void) m12mo_request_cmd_effect(&m12mo_dev->sd, M12MO_MANUAL_FOCUS, &reg_val);

	return len;
}
static DEVICE_ATTR(isp_maunal_focus_ctrl, S_IRUGO | S_IWUSR, m12mo_maunal_focus_ctrl_read, m12mo_maunal_focus_ctrl_write);

static ssize_t m12mo_sensor_nr_en_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "set NR enable.\n\
		 0x00: sensor NR OFF\n \
		 0x01: sensor NR ON\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, SENSOR_NR_EN, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, sensor_nr_en = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 set NR enable.\n\
		 0x00: sensor NR OFF\n \
		 0x01: sensor NR ON\n\
		 sensor_nr_en = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_sensor_nr_en_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, SENSOR_NR_EN, val);
    	printk("@%s %d, sensor_nr_en %x\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_sensor_nr_en, S_IRUGO | S_IWUSR, m12mo_sensor_nr_en_read, m12mo_sensor_nr_en_write);

static ssize_t m12mo_sensor_update_en_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "set update enable.\n\
		 0x00: sensor update OFF / OIS tempcorrection OFF\n \
		 0x01: sensor update ON / OIS tempcorrection OFF\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_TEST, SENSOR_UPDATE_EN, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, sensor_update_en = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		 set update enable.\n\
		 0x00: sensor update OFF / OIS tempcorrection OFF\n \
		 0x01: sensor update ON / OIS tempcorrection OFF\n\
		 sensor_update_en = 0x%x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_sensor_update_en_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_TEST, SENSOR_UPDATE_EN, val);
    	printk("@%s %d, sensor_update_en %x\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_sensor_update_en, S_IRUGO | S_IWUSR, m12mo_sensor_update_en_read, m12mo_sensor_update_en_write);

static ssize_t m12mo_ae_mode_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "AE Metering mode setting\n \
			0x00: METERING_CENTER\n \
			0x01: METERING_SPOT\n \
			0x02: METERING_AVERAGE\n \
			0xFF: AE OFF\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_AE, AE_MODE, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, ae_mode = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		AE Metering mode setting\n \
			0x00: METERING_CENTER\n \
			0x01: METERING_SPOT\n \
			0x02: METERING_AVERAGE\n \
			0xFF: AE OFF\n\n\n \
		ae_mode = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_ae_mode_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val == 0x0 || val == 0x01 || val == 0x02 || val == 0xFF){
		(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_AE, AE_MODE, val);
    		printk("@%s %d, ae_mode %x\n", __func__, __LINE__, val);
		return len;
	}else{
    		printk("@%s %d, arg error\n", __func__, __LINE__);
		return -EINVAL;
	}

}
static DEVICE_ATTR(isp_ae_mode, S_IRUGO | S_IWUSR, m12mo_ae_mode_read, m12mo_ae_mode_write);

static ssize_t m12mo_ae_target_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "AE TARGET (0x01~0x5A)\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_AE, AE_TARGET, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, ae_target = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		AE TARGET (0x01~0x5A)\n\n\n \
		ae_target = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_ae_target_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val > 0x00 && val < 0x5b){
		(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_AE, AE_TARGET, val);
    		printk("@%s %d, ae_target %x\n", __func__, __LINE__, val);
		return len;
	}else{
    		printk("@%s %d, arg error\n", __func__, __LINE__);
		return -EINVAL;
	}

}
static DEVICE_ATTR(isp_ae_target, S_IRUGO | S_IWUSR, m12mo_ae_target_read, m12mo_ae_target_write);

static ssize_t m12mo_ae_speed_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "AE_SPEED (0x01~0x64)\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_AE, AE_SPEED, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, ae_speed = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		AE_SPEED (0x01~0x64)\n\n\n \
		ae_speed = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_ae_speed_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val > 0x00 && val < 0x65){
		(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_AE, AE_SPEED, val);
    		printk("@%s %d, ae_speed %x\n", __func__, __LINE__, val);
		return len;
	}else{
    		printk("@%s %d, arg error\n", __func__, __LINE__);
		return -EINVAL;
	}

}
static DEVICE_ATTR(isp_ae_speed, S_IRUGO | S_IWUSR, m12mo_ae_speed_read, m12mo_ae_speed_write);

static ssize_t m12mo_af_start_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "Starting AF operation\n \
		<Write>\n \
			0x09: Focus Calibration\n \
			0x13: Single AF (Simple)\n \
			0x14: Single AF\n \
		<Read>\n \
			0x00: AF stop by force or AF released\n \
			0x01: AF starting\n \
			0x02: AF done\n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_LENS, AF_START, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, af_start = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		Starting AF operation\n \
		<Write>\n \
			0x09: Focus Calibration\n \
			0x13: Single AF (Simple)\n \
			0x14: Single AF\n \
		<Read>\n \
			0x00: AF stop by force or AF released\n \
			0x01: AF starting\n \
			0x02: AF done\n\n\n \
		af_start = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_af_start_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;//,timeout=0;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	if(val == 0x13){
		(void) m12mo_request_cmd_effect(&m12mo_dev->sd, M12MO_START_AF, NULL);
		return len;
	}else if(val == 0x9 || val == 0x14){
		(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_LENS, AF_START, val);
    		printk("@%s %d, af_start %x\n", __func__, __LINE__, val);
		return len;
	}else{
    		printk("@%s %d, arg error\n", __func__, __LINE__);
		return -EINVAL;
	}

}
static DEVICE_ATTR(isp_af_start, S_IRUGO | S_IWUSR, m12mo_af_start_read, m12mo_af_start_write);

static ssize_t m12mo_af_operation_result_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "AF operation result \n \
			0x00: AF OFF \n \
			0x01: Focus operation success \n \
			0x02: Focus operation fail \n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_LENS, AF_RESULT, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, af_operation_result = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		AF operation result \n \
			0x00: AF OFF \n \
			0x01: Focus operation success \n \
			0x02: Focus operation fail \n \n\n\
		af_operation_result = %x\n", reg_val);
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_af_operation_result, S_IRUGO | S_IWUSR, m12mo_af_operation_result_read, NULL);

static ssize_t m12mo_af_laser_start_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "Laser Sensor Start \n \
			0x01: Simple Distance \n \
			0x02: Distance (not support) \n");

    	ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_LENS, AF_LASER_START, &reg_val);
        if(ret)
            goto out;
	printk("@%s %d, af_laser_start = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		Laser Sensor Start \n \
			0x01: Simple Distance \n \
			0x02: Distance (not support) \n\n\n \
		af_laser_start = %x\n", reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_af_laser_start_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_LENS, AF_LASER_START, val);
    	printk("@%s %d, af_laser_start %x\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_af_laser_start, S_IRUGO | S_IWUSR, m12mo_af_laser_start_read, m12mo_af_laser_start_write);

static ssize_t m12mo_af_laser_distance_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	printk( "Unit:mm, \n \
		 Distance: 0mm ~ 500mm, \n");

    	ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_LENS, AF_LASER_DISTANCE_H, &reg_val);
        if(ret)
            goto out;

	printk("@%s %d, af_laser_distance = %x\n", __func__, __LINE__, reg_val);

	return scnprintf(buf, PAGE_SIZE, "\n\
		Unit:mm, \n \
		Distance: 0mm ~ 500mm, \n\n\n\
		af_laser_distance = 0x%x\n", reg_val);
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_af_laser_distance, S_IRUGO | S_IWUSR, m12mo_af_laser_distance_read, NULL);

#if 0
static ssize_t m12mo_asus_exposure_control_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    	u32 reg_val,ret;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	switch(m12mo_dev->asus_exposure_control_mode){

	case ASUS_SATURATION:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SATURATION, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SATURATION = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_CONTRAST:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_CONTRAST, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_CONTRAST = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FACTORY_MODE:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FACTORY_MODE, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FACTORY_MODE = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FLASH_NEEDED:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FLASH_NEEDED, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FLASH_NEEDED = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FOCUS_RESULT:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FOCUS_RESULT, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FOCUS_RESULT = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_3A_LOCK:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_3A_LOCK, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_3A_LOCK = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_BATTERY_CAPACITY:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_BATTERY_CAPACITY, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_BATTERY_CAPACITY = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_EFFECT:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_EFFECT, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_EFFECT = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_ISO:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_ISO, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_ISO = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_EV:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_EV, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_EV = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SHUTTER_SPEED:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SHUTTER_SPEED, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SHUTTER_SPEED = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FLICKER:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FLICKER, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FLICKER = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AE_SCENE_MODE:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AE_SCENE_MODE, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AE_SCENE_MODE = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FLASH:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FLASH, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FLASH = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_METERING_MODE:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_METERING_MODE, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_METERING_MODE = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_TOUCH_POSITION_X:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_TOUCH_POSITION_X, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_TOUCH_POSITION_X = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_TOUCH_POSITION_Y:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_TOUCH_POSITION_Y, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_TOUCH_POSITION_Y = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FULL_SEARCH_TARGET:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FULL_SEARCH_TARGET, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FULL_SEARCH_TARGET = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FULL_SEARCH_RANGE:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FULL_SEARCH_RANGE, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FULL_SEARCH_RANGE = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FULL_SEARCH_STEP:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FULL_SEARCH_STEP, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FULL_SEARCH_STEP = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_LONG_EXP_CAP:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_LONG_EXP_CAP, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_LONG_EXP_CAP = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AUTOLL:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AUTOLL, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AUTOLL = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AT_SCENE:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AT_SCENE, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AT_SCENE = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_TOUCH_ROI_LEFT_UPPER_X:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_TOUCH_ROI_LEFT_UPPER_X, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_TOUCH_ROI_LEFT_UPPER_X = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_TOUCH_ROI_LEFT_UPPER_Y:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_TOUCH_ROI_LEFT_UPPER_Y, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_TOUCH_ROI_LEFT_UPPER_Y = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_TOUCH_WIDTH:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_TOUCH_WIDTH, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_TOUCH_WIDTH = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_TOUCH_HEIGHT:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_TOUCH_HEIGHT, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_TOUCH_HEIGHT = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_PHONE_DIRECTION:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_PHONE_DIRECTION, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_PHONE_DIRECTION = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_ZOOM_POSITION:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_ZOOM_POSITION, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_ZOOM_POSITION = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FOCUS_STEP:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FOCUS_STEP, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FOCUS_STEP = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_FOCUS_MODE:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_FOCUS_MODE, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_FOCUS_MODE = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_G_SENSOR_X:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_G_SENSOR_X, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_G_SENSOR_X = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_G_SENSOR_Y:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_G_SENSOR_Y, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_G_SENSOR_Y = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_G_SENSOR_Z:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_G_SENSOR_Z, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_G_SENSOR_Z = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_WB_MANUAL:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_WB_MANUAL, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_WB_MANUAL = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_COLOR_TEMPERATURE:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_COLOR_TEMPERATURE, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_COLOR_TEMPERATURE = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_GAIN1_RGAIN:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_GAIN1_RGAIN, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_GAIN1_RGAIN = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_GAIN1_BGAIN:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_GAIN1_BGAIN, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_GAIN1_BGAIN = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_GAIN2_RGAIN:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_GAIN2_RGAIN, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_GAIN2_RGAIN = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_GAIN2_BGAIN:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_GAIN2_BGAIN, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_GAIN2_BGAIN = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_LV:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_LV, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_LV = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_LSC_ZOOM_INDEX_A:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_LSC_ZOOM_INDEX_A, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_LSC_ZOOM_INDEX_A = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_LSC_ZOOM_INDEX_B:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_LSC_ZOOM_INDEX_B, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_LSC_ZOOM_INDEX_B = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_LSC_LIGHTSOURCE_INDEX_A:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_LSC_LIGHTSOURCE_INDEX_A, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_LSC_LIGHTSOURCE_INDEX_A = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_LSC_LIGHTSOURCE_INDEX_B:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_LSC_LIGHTSOURCE_INDEX_B, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_LSC_LIGHTSOURCE_INDEX_B = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_LSC_BLEND_RATIO:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_LSC_BLEND_RATIO, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_LSC_BLEND_RATIO = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AWB_DEBUG9:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AWB_DEBUG9, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AWB_DEBUG9 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AWB_DEBUG10:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AWB_DEBUG10, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AWB_DEBUG10 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AWB_DEBUG11:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AWB_DEBUG11, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AWB_DEBUG11 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AWB_DEBUG12:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AWB_DEBUG12, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AWB_DEBUG12 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AWB_DEBUG13:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AWB_DEBUG13, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AWB_DEBUG13 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AWB_DEBUG14:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AWB_DEBUG14, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AWB_DEBUG14 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_AWB_DEBUG15:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_AWB_DEBUG15, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_AWB_DEBUG15 = %x\n", __func__, __LINE__, reg_val);
	break;
	case LED_ERROR_ADDR:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, LED_ERROR_ADDR, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, LED_ERROR_ADDR = %x\n", __func__, __LINE__, reg_val);
	break;
	case LED_ERROR_STATUS:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, LED_ERROR_STATUS, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, LED_ERROR_STATUS = %x\n", __func__, __LINE__, reg_val);
	break;
	case LED_ERROR_DATA:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, LED_ERROR_DATA, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, LED_ERROR_DATA = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_INFINITY_OFFSET:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_INFINITY_OFFSET, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_INFINITY_OFFSET = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MACRO_OFFSET:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MACRO_OFFSET, &reg_val);
        	if(ret)
            		goto out;
            printk("@%s %d, ASUS_MACRO_OFFSET = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_PREVIEW_FRAME_WIDTH:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_PREVIEW_FRAME_WIDTH, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_PREVIEW_FRAME_WIDTH = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_PREVIEW_FRAME_HEIGHT:
		ret = m12mo_readw(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_PREVIEW_FRAME_HEIGHT, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_PREVIEW_FRAME_HEIGHT = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR0:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR0, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR0 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR1:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR1, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR1 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR2:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR2, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR2 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR3:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR3, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR3 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR4:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR4, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR4 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR5:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR5, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR5 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR6:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR6, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR6 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR7:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR7, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR7 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR8:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR8, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR8 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR9:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR9, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR9 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR10:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR10, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR10 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR11:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR11, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR11 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR12:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR12, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR12 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR13:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR13, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR13 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR14:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR14, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR14 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR15:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR15, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR15 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR16:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR16, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR16 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR17:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR17, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR17 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR18:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR18, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR18 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_MOTOR19:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_MOTOR19, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_MOTOR19 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE0:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE0, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE0 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE1:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE1, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE1 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE2:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE2, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE2 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE3:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE3, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE3 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE4:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE4, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE4 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE5:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE5, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE5 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE6:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE6, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE6 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE7:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE7, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE7 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE8:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE8, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE8 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE9:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE9, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE9 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE10:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE10, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE10 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE11:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE11, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE11 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE12:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE12, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE12 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE13:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE13, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE13 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE14:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE14, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE14 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE15:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE15, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE15 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE16:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE16, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE16 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE17:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE17, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE17 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE18:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE18, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE18 = %x\n", __func__, __LINE__, reg_val);
	break;
	case ASUS_SCORE19:
		ret = m12mo_readb(&m12mo_dev->sd, CATEGORY_ASUS, ASUS_SCORE19, &reg_val);
        	if(ret)
            		goto out;
		printk("@%s %d, ASUS_SCORE19 = %x\n", __func__, __LINE__, reg_val);
	break;

	default:
		printk("@%s %d, Category error = %x\n", __func__, __LINE__, m12mo_dev->asus_exposure_control_mode);
	break;
	}


	return scnprintf(buf, PAGE_SIZE, "\n\
			ASUS EXPOSURE CONTROL Byte = %x data = %x\n", m12mo_dev->asus_exposure_control_mode, reg_val);
out:
	return -EINVAL;
}
static ssize_t m12mo_asus_exposure_control_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    	u32 val[2];
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x %x", &val[0], &val[1]);
	if(ASUS_TOUCH_POSITION_X == val[0] ||\
	   ASUS_TOUCH_POSITION_Y == val[0] ||\
	   ASUS_FULL_SEARCH_TARGET == val[0] ||\
	   ASUS_AT_SCENE == val[0] ||\
	   ASUS_TOUCH_ROI_LEFT_UPPER_X == val[0] ||\
	   ASUS_TOUCH_ROI_LEFT_UPPER_Y == val[0] ||\
	   ASUS_TOUCH_WIDTH == val[0] ||\
	   ASUS_TOUCH_HEIGHT == val[0] ||\
	   ASUS_G_SENSOR_X == val[0] ||\
	   ASUS_G_SENSOR_Y == val[0] ||\
	   ASUS_G_SENSOR_Z == val[0] ||\
	   ASUS_COLOR_TEMPERATURE == val[0] ||\
	   ASUS_GAIN1_RGAIN == val[0] ||\
	   ASUS_GAIN1_BGAIN == val[0] ||\
	   ASUS_GAIN2_RGAIN == val[0] ||\
	   ASUS_GAIN2_BGAIN == val[0] ||\
	   ASUS_LV == val[0] ||\
	   ASUS_LSC_BLEND_RATIO == val[0] ||\
	   ASUS_INFINITY_OFFSET == val[0] ||\
	   ASUS_MACRO_OFFSET == val[0] ||\
	   ASUS_PREVIEW_FRAME_WIDTH == val[0] ||\
	   ASUS_PREVIEW_FRAME_HEIGHT == val[0])
	(void) m12mo_writew(&m12mo_dev->sd, CATEGORY_ASUS, val[0], val[1]);
	else
	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_ASUS, val[0], val[1]);

    	printk("@%s %d, ASUS EXPOSURE CONTROL Byte = %x data = %x\n", __func__, __LINE__, val[0], val[1]);

	return len;
}
static DEVICE_ATTR(isp_asus_exposure_control, S_IRUGO | S_IWUSR, m12mo_asus_exposure_control_read, m12mo_asus_exposure_control_write);

static ssize_t m12mo_asus_exposure_control_read_set(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "\n\
		set ASUS EXPOSURE CONTROL read Byte\n");
}
static ssize_t m12mo_asus_exposure_control_write_set(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	m12mo_dev->asus_exposure_control_mode = val;
    	printk("@%s %d, ASUS EXPOSURE CONTROL read Byte = %x\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_asus_exposure_control_set, S_IRUGO | S_IWUSR, m12mo_asus_exposure_control_read_set, m12mo_asus_exposure_control_write_set);
#endif

static ssize_t m12mo_read_FlashRom_calibration_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	int ret = 0;
    u32 reg_val[4]={0x0,0x0,0x0,0x0};
    ret = m12mo_read_Flashrom_data( &m12mo_dev->sd, m12mo_dev->fadj_command, m12mo_dev->fadj_offset, reg_val );
    if(ret)
    {
    	ret = scnprintf(buf, PAGE_SIZE, "\n \
			read/write calibration data\n\
				arg1: 	0x01: 1 byte read\n\
					0x02: 2 byte read\n\
					0x04: 4 byte read\n\
					0x09: 1 byte write\n\
					0x0a: 2 byte write\n\
					0x0c: 4 byte write\n\
				      others: setting mode\n\
				arg2: offset[0x0000~0fff]\n\
				arg3: data0\n\
				arg4: data1\n\
				arg5: data2\n\
				arg6: data3\n\
				please input arg \n\n");
    	return ret;
    }
    switch(m12mo_dev->fadj_command){
	    case 0x01://1 byte read
			ret = scnprintf(buf, PAGE_SIZE, "\n \
			read/write calibration data\n\
				arg1: 	0x01: 1 byte read\n\
					0x02: 2 byte read\n\
					0x04: 4 byte read\n\
					0x09: 1 byte write\n\
					0x0a: 2 byte write\n\
					0x0c: 4 byte write\n\
				      others: setting mode\n\
				arg2: offset[0x0000~0fff]\n\
				arg3: data0\n\
				arg4: data1\n\
				arg5: data2\n\
				arg6: data3\n\n \
				command %x offset %x data0=%x\n", m12mo_dev->fadj_command,m12mo_dev->fadj_offset,reg_val[0]);
		    break;
	    case 0x02://2 byte read
			ret = scnprintf(buf, PAGE_SIZE, "\n \
			read/write calibration data\n\
				arg1: 	0x01: 1 byte read\n\
					0x02: 2 byte read\n\
					0x04: 4 byte read\n\
					0x09: 1 byte write\n\
					0x0a: 2 byte write\n\
					0x0c: 4 byte write\n\
				      others: setting mode\n\
				arg2: offset[0x0000~0fff]\n\
				arg3: data0\n\
				arg4: data1\n\
				arg5: data2\n\
				arg6: data3\n\n \
				command %x offset %x data0=%x data1=%x\n", m12mo_dev->fadj_command,m12mo_dev->fadj_offset,reg_val[0], reg_val[1]);
		    break;
	    case 0x04://4 byte read
			ret = scnprintf(buf, PAGE_SIZE, "\n \
			read/write calibration data\n\
				arg1: 	0x01: 1 byte read\n\
					0x02: 2 byte read\n\
					0x04: 4 byte read\n\
					0x09: 1 byte write\n\
					0x0a: 2 byte write\n\
					0x0c: 4 byte write\n\
				      others: setting mode\n\
				arg2: offset[0x0000~0fff]\n\
				arg3: data0\n\
				arg4: data1\n\
				arg5: data2\n\
				arg6: data3\n\n \
				command %x offset %x data0=%x data1=%x data2=%x data3=%x\n", m12mo_dev->fadj_command,m12mo_dev->fadj_offset,reg_val[0], reg_val[1],reg_val[2], reg_val[3]);
		    break;
	}
	return ret;
}
static ssize_t m12mo_write_FlashRom_calibration_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 reg_val[4]={0x0,0x0,0x0,0x0};
//	int ret=0, res=0, timeout=5000;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	sscanf(buf, "%x %x %x %x %x %x", &m12mo_dev->fadj_command, &m12mo_dev->fadj_offset, &reg_val[0], &reg_val[1], &reg_val[2], &reg_val[3]);


	printk("@%s %d,start write calibration! command = %x , offset = %x , reg_val[0] = %x , reg_val[1] = %x , reg_val[2] = %x , reg_val[3] = %x\n"
		,__func__, __LINE__, m12mo_dev->fadj_command, m12mo_dev->fadj_offset, reg_val[0], reg_val[1], reg_val[2], reg_val[3]);
    if( m12mo_dev->fadj_command != 0x09 && m12mo_dev->fadj_command != 0x0a && m12mo_dev->fadj_command != 0x0c )
    {
    	printk("@%s %d,set command=%x offset=%x\n",__func__, __LINE__,m12mo_dev->fadj_command, m12mo_dev->fadj_offset);
		return len;
    }

    m12mo_write_Flashrom_data( &m12mo_dev->sd, m12mo_dev->fadj_command, m12mo_dev->fadj_offset, reg_val );
	return len;

}
static DEVICE_ATTR(isp_FlashRom_RW, S_IRUGO | S_IWUSR, m12mo_read_FlashRom_calibration_data, m12mo_write_FlashRom_calibration_data);

static ssize_t m12mo_read_EEPROM_calibration_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	u32 val;	
	int ret = 0;
	printk("@%s %d,start read eeprom address: 0x%x 0x%x !\n",__func__, __LINE__, (m12mo_dev->eeprom_offset >> 8) & 0xFF, m12mo_dev->eeprom_offset & 0xFF);
   	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_ADJUSTMENT, SET_EEPROM_ADD_H, (m12mo_dev->eeprom_offset >> 8) & 0xFF); 
   	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_ADJUSTMENT, SET_EEPROM_ADD_L, m12mo_dev->eeprom_offset & 0xFF); 

	(void) m12mo_writeb(&m12mo_dev->sd, CATEGORY_ADJUSTMENT, READ_EEPROM_TAG, 0x01); 


	m12mo_readb(&m12mo_dev->sd, CATEGORY_ADJUSTMENT, READ_EEPROM, &val);

	ret = scnprintf(buf, PAGE_SIZE, "\n \
			read/write eeprom calibration data\n\
				eeprom calibration data = 0x%x \n", val);

	printk("@%s %d,finish read eeprom calibration data = 0x%x\n",__func__, __LINE__, val);
	return ret;
}

static ssize_t m12mo_write_EEPROM_calibration_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &m12mo_dev->eeprom_offset);

	if( (m12mo_dev->eeprom_offset < 0x0 || m12mo_dev->eeprom_offset > 0x1E71) && m12mo_dev->eeprom_offset != 0x3299 )
	{
		printk("@%s %d,set offset out of range( 0x%x ), please set range on [0x0000~0x1E71]\n",__func__, __LINE__, m12mo_dev->eeprom_offset);
		m12mo_dev->eeprom_offset = 0x0;
		return len;
	}

	printk("@%s %d,set read eeprom address: 0x%x(add_H) 0x%x(add_L) !\n",__func__, __LINE__, (m12mo_dev->eeprom_offset >> 8) & 0xFF, m12mo_dev->eeprom_offset & 0xFF);
	return len;

}
static DEVICE_ATTR(isp_EEPROM_RW, S_IRUGO | S_IWUSR, m12mo_read_EEPROM_calibration_data, m12mo_write_EEPROM_calibration_data);

static ssize_t m12mo_read_SHD_calibration_data(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	mutex_lock(&m12mo_dev->input_lock);
    (void) m12mo_s_power_fac(1);
	m12mo_isp_fw_SHD_R(m12mo_dev);
    (void) m12mo_s_power_fac(0);
	mutex_unlock(&m12mo_dev->input_lock);
	return 0;
}
static ssize_t m12mo_write_SHD_calibration_data(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
	mutex_lock(&m12mo_dev->input_lock);
    (void) m12mo_s_power_fac(1);
	m12mo_isp_fw_SHD_W(m12mo_dev);

	(void) m12mo_request_cmd_effect(&m12mo_dev->sd, M12MO_WRITE_SHD_TABLE, NULL);
    (void) m12mo_s_power_fac(0);
	mutex_unlock(&m12mo_dev->input_lock);
//    (void) m12mo_request_cmd_effect(&m12mo_dev->sd, M12MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);

	return len;
}
static DEVICE_ATTR(isp_fw_SHD_RW, S_IRUGO | S_IWUSR, m12mo_read_SHD_calibration_data, m12mo_write_SHD_calibration_data);

static ssize_t m12mo_open_m12mo_fac(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    	u32 val=0;

	sscanf(buf, "%x", &val);

	if(val == 0){
    	    (void) m12mo_s_power_fac(0);
	    CacCalibrationStatus = false;
	}else{
	    CacCalibrationStatus = true;
    	    (void) m12mo_s_power_fac(1);
	}
	return len;
}
static DEVICE_ATTR(isp_fw_power_fac, S_IRUGO | S_IWUSR, NULL, m12mo_open_m12mo_fac);
#endif

static ssize_t m12mo_isp_update_status(struct device *dev, struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "%x", isp_update_status);;
}
static DEVICE_ATTR(isp_update_status, S_IRUGO | S_IWUSR, m12mo_isp_update_status, NULL);

static int m12mo_ispd1(struct m12mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	struct v4l2_subdev *sd = &dev->sd;
	int ret = 0;

	mutex_lock(&dev->input_lock);

	dev_info(&client->dev, "ispd1 start\n");
	ret = m12mo_dump_string_log1(sd);
	if (ret < 0)
		dev_err(&client->dev, "isp log 1 error\n");

	dev_info(&client->dev, "ispd1 finished\n");
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int m12mo_ispd2(struct m12mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	struct v4l2_subdev *sd = &dev->sd;
	int ret = 0;

	mutex_lock(&dev->input_lock);

	dev_info(&client->dev, "ispd2 start\n");

	ret = m12mo_dump_string_log2_1(sd);
	if (ret != 0)
		dev_err(&client->dev, "m12mo_dump_string_log2_1 error\n");

	dev_info(&client->dev, "log2_1 finished\n");

	ret = m12mo_dump_string_log2_2(sd);
	if (ret != 0)
		dev_err(&client->dev, "m12mo_dump_string_log2_2 error\n");

	dev_info(&client->dev, "ispd2_2 finish\n");

	ret = m12mo_dump_string_log2_3(sd);
	if (ret != 0)
		dev_err(&client->dev, "m12mo_dump_string_log2_3 error\n");

	dev_info(&client->dev, "ispd2_3 finish\n");

	mutex_unlock(&dev->input_lock);
	return ret;
}

static int m12mo_ispd3(struct m12mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	struct v4l2_subdev *sd = &dev->sd;
	int ret = 0;

	mutex_lock(&dev->input_lock);

	/**ISP RESET**/
	ret = dev->pdata->common.gpio_ctrl(sd, 0);

	msleep(10);

	/**ISP RESET**/
	ret = dev->pdata->common.gpio_ctrl(sd, 1);

	msleep(50);

	dev_info(&client->dev, "ispd3 start\n");

	ret = m12mo_dump_string_log2_1(sd);
	if (ret != 0)
		dev_err(&client->dev, "m12mo_dump_string_log2_1 error\n");

	dev_info(&client->dev, "log2_1 finished\n");

	ret = m12mo_dump_string_log2_2(sd);
	if (ret != 0)
		dev_err(&client->dev, "m12mo_dump_string_log2_2 error\n");

	dev_info(&client->dev, "ispd2_2 finish\n");

	ret = m12mo_dump_string_log2_3(sd);
	if (ret != 0)
		dev_err(&client->dev, "m12mo_dump_string_log2_3 error\n");

	dev_info(&client->dev, "ispd2_3 finish\n");

	dev_info(&client->dev, "ispd3 finished\n");

	mutex_unlock(&dev->input_lock);
	return ret;
}

static int m12mo_ispd4(struct m12mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	struct v4l2_subdev *sd = &dev->sd;
	int ret = 0;

	mutex_lock(&dev->input_lock);

	dev_info(&client->dev, "ispd4 start\n");
	ret = m12mo_dump_string_log3(sd);
	if (ret < 0)
		dev_err(&client->dev, "isp log 4 error\n");

	dev_info(&client->dev, "ispd4 finished\n");
	mutex_unlock(&dev->input_lock);
	return ret;
}

void m12mo_dump_log(struct v4l2_subdev *sd)
{
	struct m12mo_device *m12mo_dev = to_m12mo_sensor(sd);

	/*
	 * dbglvl: bit0 to dump m12mo_ispd1
	 * dbglvl: bit1 to dump m12mo_ispd2
	 * dbglvl: bit2 to dump m12mo_ispd4
	 * Debug log 3 is most important, so dump first
	 */
	if (dbglvl & 4)
		m12mo_ispd4(m12mo_dev);

	if (dbglvl & 2)
		m12mo_ispd2(m12mo_dev);

	if (dbglvl & 1)
		m12mo_ispd1(m12mo_dev);
}

static ssize_t m12mo_isp_log_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	if (m12mo_dev->power == 0) {
		goto leave;
	}

	if (!strncmp(buf, "1", 1))
		m12mo_ispd1(m12mo_dev);
	else if (!strncmp(buf, "4", 1))
		m12mo_ispd4(m12mo_dev);
	else if (!strncmp(buf, "2", 1))
		m12mo_ispd2(m12mo_dev);
	else if (!strncmp(buf, "3", 1))
		m12mo_ispd3(m12mo_dev);
	else
		m12mo_ispd4(m12mo_dev);

leave:
	return len;
}
static DEVICE_ATTR(isp_log, S_IRUGO | S_IWUSR, NULL, m12mo_isp_log_store);

static ssize_t m12mo_fw_version_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);
    printk(KERN_INFO "m12mo, FW version is %04x-%x\n", m12mo_dev->ver.firmware, m12mo_dev->ver.DIT_firmware);
    return scnprintf(buf, PAGE_SIZE, "%x-%x\n", m12mo_dev->ver.firmware, m12mo_dev->ver.DIT_firmware);
}
static DEVICE_ATTR(isp_fw_version, S_IRUGO | S_IWUSR, m12mo_fw_version_read, NULL);

static ssize_t m12mo_test_pattern_show(struct device *dev, struct device_attribute *attr, char *buf)
{
    return scnprintf(buf, PAGE_SIZE, "%x\n", PREVIEW_TEST_PATTERN);
}
static ssize_t m12mo_test_pattern_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t len)
{
	unsigned int value;
	sscanf(buf, "%x", &value);
    if(value)
        PREVIEW_TEST_PATTERN = true;
    else
        PREVIEW_TEST_PATTERN = false;
	return len;
}
static DEVICE_ATTR(isp_test_pattern, S_IRUGO | S_IWUSR, m12mo_test_pattern_show, m12mo_test_pattern_store);

static ssize_t m12mo_log_enable(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    	u32 val=0;
	int ret=0;
	struct m12mo_device *m12mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);

	if(val == 1){
		ret = m12mo_writeb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_ENABLE);
		if (ret < 0)
			printk("%s, m12mo_writeb error\n", __func__);
	}else if(val == 0){
		ret = m12mo_writeb(&m12mo_dev->sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_DISABLE);
		if (ret < 0)
			printk("%s, m12mo_writeb error\n", __func__);
	}
	return len;
}
static DEVICE_ATTR(m12mo_log_enable, S_IRUGO | S_IWUSR, NULL, m12mo_log_enable);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_isp_sensor_i2c_status.attr,
	&dev_attr_isp_rgb_ctrl.attr,
	&dev_attr_isp_rgb_calib.attr,
	&dev_attr_isp_rgb_data.attr,
	&dev_attr_isp_checksum.attr,
	&dev_attr_isp_flashfw.attr,
	&dev_attr_isp_fw_dump.attr,
	&dev_attr_isp_update_status.attr,
	&dev_attr_isp_spi.attr,
	&dev_attr_isp_fw_id.attr,
	&dev_attr_isp_log.attr,
    &dev_attr_isp_fw_version.attr,
    &dev_attr_isp_test_pattern.attr,
    &dev_attr_m12mo_log_enable.attr,
    &dev_attr_isp_fw_customer_project.attr,
    &dev_attr_isp_fw_debug.attr,
	&dev_attr_isp_fw_default_flag.attr,
	&dev_attr_m12mo_download_trace_log_when_time_out.attr,
	&dev_attr_m12mo_asus_camera_flag.attr,
	&dev_attr_isp_shading_table_checksum.attr,
#ifndef ZX551ML_USER_BUILD //#ifdef CONFIG_ASUS_FACTORY_MODE
	&dev_attr_isp_digital_zoom.attr,
	&dev_attr_m12mo_i2c_debug_byte_write_change_name_by_jevian.attr,
	&dev_attr_m12mo_i2c_debug_state.attr,
	&dev_attr_isp_apk_capture_mode.attr,
	&dev_attr_m12mo_i2c_debug_byte_read_change_name_by_jevian.attr,
	&dev_attr_isp_distortion_mode.attr,
	&dev_attr_isp_ois_data.attr,
	&dev_attr_isp_ois_write_read_trig.attr,
	&dev_attr_isp_ois_ramreg_addr.attr,
	&dev_attr_isp_ois_lib_api_start.attr,
	&dev_attr_isp_ois_cali_start.attr,
	&dev_attr_isp_ois_cali_result.attr,
	&dev_attr_isp_cap_test.attr,
	&dev_attr_isp_ois_ctrl.attr,
	&dev_attr_isp_pr_test.attr,
	&dev_attr_isp_pr_led.attr,
	&dev_attr_isp_maunal_focus_ctrl.attr,
	&dev_attr_isp_led_status.attr,
	&dev_attr_isp_sensor_update_en.attr,
	&dev_attr_isp_sensor_nr_en.attr,
	&dev_attr_isp_af_start.attr,
	&dev_attr_isp_af_operation_result.attr,
	&dev_attr_isp_af_laser_start.attr,
	&dev_attr_isp_af_laser_distance.attr,
//	&dev_attr_isp_asus_exposure_control_set.attr,
//	&dev_attr_isp_asus_exposure_control.attr,
	&dev_attr_isp_FlashRom_RW.attr,
	&dev_attr_isp_EEPROM_RW.attr,
	&dev_attr_isp_ae_mode.attr,
	&dev_attr_isp_ae_target.attr,
	&dev_attr_isp_ae_speed.attr,
	&dev_attr_isp_asus_awb_control_set.attr,
	&dev_attr_isp_asus_awb_control.attr,
	&dev_attr_isp_exif_info_exptime.attr,
	&dev_attr_isp_exif_info_iso.attr,
	&dev_attr_isp_fw_SHD_RW.attr,
	&dev_attr_m12mo_power_ctrl.attr,
	&dev_attr_m12mo_debug_password.attr,
	&dev_attr_isp_fw_iq_RW.attr,
	&dev_attr_isp_fw_power_fac.attr,
#endif
	NULL
};

static struct attribute_group m12mo_attribute_group[] = {
	{.attrs = sysfs_attrs_ctrl },
};

static int m12mo_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct m12mo_device *dev = to_m12mo_sensor(sd);

	sysfs_remove_group(&client->dev.kobj,
			   m12mo_attribute_group);

	if (dev->pdata->common.platform_deinit)
		dev->pdata->common.platform_deinit();

	media_entity_cleanup(&dev->sd.entity);
	v4l2_device_unregister_subdev(sd);
#ifndef NOIRQ
	free_irq(client->irq, sd);
#endif
	kfree(dev);

	return 0;
}

static int m12mo_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct m12mo_device *dev;
	struct camera_mipi_info *mipi_info = NULL;
	struct proc_dir_entry* proc_entry_asus_camera;
	void* dummy = NULL;
	int ret;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "platform data missing\n");
		return -ENODEV;
	}

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	/*
	 * This I2C device is created by atomisp driver. Atomisp driver has
	 * a nasty assumption that all sensors uses similar platform data.
	 * For this driver we need more information. Platform data what was
	 * received with I2C device data structure points to the common
	 * structure. Pick the real platform data for this driver.
	 */
	dev->pdata = container_of(client->dev.platform_data,
				  struct m12mo_platform_data,
				  common);

	dev->cmd = M12MO_NO_CMD_REQUEST;
	dev->m12mo_mode = M12MO_PARAMETER_MODE;
	dev->requested_cmd = M12MO_NO_CMD_REQUEST;
	dev->iso_sensitivity =  REG_AE_ISOMODE_ISO100;
	dev->iso_mode = V4L2_ISO_SENSITIVITY_AUTO;
	dev->monitor_params.af_mode = AF_NORMAL;
	dev->monitor_params.exe_mode = AF_STOP;
	dev->asus_exposure_control_mode = 0;
	dev->fadj_command = 0;
	dev->fadj_offset = 0;
	dev->eeprom_offset = 0;
	dev->disable_irq_flag = 0;
	dev->wait_irq_flag = M12MO_IRQ_COMMAND_AVAILABDLE;
	dev->m12mo_request_cmd_lock_flag = 0;
    dev->lock_i2c_write_flag = 0;
    dev->flashrom_mode = 1;
    dev->flashrom_int = 0;
    dev->ver.checksum = M12MO_INVALID_CHECKSUM;

	mutex_init(&dev->input_lock);
	mutex_init(&dev->m12mo_request_cmd_lock);
	mutex_init(&dev->m12mo_lens_work_lock);
	wake_lock_init(&dev->m12mo_wake_lock, WAKE_LOCK_SUSPEND, "m12mo_wakelock");
	wake_lock_init(&dev->m12mo_update_wake_lock, WAKE_LOCK_SUSPEND, "m12mo_update_wake_lock");

	dev->auto_update_wq = create_singlethread_workqueue("m12mo_auto_update_wq");

	v4l2_i2c_subdev_init(&(dev->sd), client, &m12mo_ops);

	ret = m12mo_s_config(&dev->sd, client->irq);
	if (ret)
		goto out_free;

	/*
	 * We must have a way to reset the chip. If that is missing we
	 * simply can't continue.
	 */
	if (!dev->pdata->common.gpio_ctrl) {
		dev_err(&client->dev, "gpio control function missing\n");
		ret = -ENODEV;
		goto out_free_irq;
	}

	mipi_info = v4l2_get_subdev_hostdata(&dev->sd);

	ret = __m12mo_init_ctrl_handler(dev);
	if (ret)
		goto out_free_irq;

	if (mipi_info)
		dev->num_lanes = mipi_info->num_lanes;

	dev->curr_res_table = resolutions[0];
	dev->entries_curr_table = resolutions_sizes[0];

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &m12mo_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	dev->format.code = V4L2_MBUS_FMT_UYVY8_1X16;
	dev->shot_mode = CAP_AUTO;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret)
		goto out_free_irq;

	ret = sysfs_create_group(&client->dev.kobj,
				m12mo_attribute_group);
	if (ret) {
		dev_err(&client->dev, "%s Failed to create sysfs\n", __func__);
		goto out_sysfs_fail;
	}
    /*ret = gpio_request(60, "m12mo_cap_debug");*/
	/*gpio_direction_output(60, 1);*/

	/* Request SPI interface to enable FW update over the SPI */
	printk("%s\t spi_setup begin \n",__func__);
	if (dev->pdata->spi_setup)
		dev->pdata->spi_setup(&dev->pdata->spi_pdata, dev);
	sdd = &dev->sd;

    queue_delayed_work(dev->auto_update_wq, &auto_Update_fw_work, 0);

	/* register switch device for m12mo information versions report */
	m12mo_switch_dev.name = "camera";
	m12mo_switch_dev.print_name = m12mo_switch_name;
	if (switch_dev_register(&m12mo_switch_dev) < 0)
		dev_err(&client->dev, "%s: fail to register m12mo switch\n", __func__);

	proc_entry_asus_camera = proc_create_data("driver/asus_camera_open", 0660, NULL, &asus_camera_proc_fops, dummy);
    proc_set_user(proc_entry_asus_camera, 1000, 1000);

	return 0;

out_sysfs_fail:
	media_entity_cleanup(&dev->sd.entity);

out_free_irq:
#ifndef NOIRQ
	free_irq(client->irq, &dev->sd);
#endif
out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	/*destroy_workqueue(dev->lens_wq);*/
    destroy_workqueue(dev->auto_update_wq);
	/*destroy_workqueue(dev->gpio_set_ois_wq);*/
	wake_lock_destroy(&dev->m12mo_wake_lock);
	wake_lock_destroy(&dev->m12mo_update_wake_lock);
//  destroy_workqueue(dev->optical_zoom_wq);
//	destroy_workqueue(dev->auto_focus_wq);
	kfree(dev);
	return ret;
}

static const struct i2c_device_id m12mo_id[] = {
	{ M12MO_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, m12mo_id);

static struct i2c_driver m12mo_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = M12MO_NAME,
	},
	.probe = m12mo_probe,
	.remove = m12mo_remove,
	.id_table = m12mo_id,
};

static __init int init_m12mo(void)
{
    switch (Read_PROJ_ID()) {
		case PROJ_ID_ZS550ML:
		case PROJ_ID_ZS570ML:
			pr_info("Project ID is ZS550ML or ZS570ML, M12MO init...\n");
			return i2c_add_driver(&m12mo_driver);
		break;

		default:
			pr_info("Project ID is NOT ZS550ML nor ZS570ML, M12MO exit...\n");
			return 0;
		break;
    }//end switch
}

static __exit void exit_m12mo(void)
{
//	int ret;
//	ret = intel_scu_ipc_iowrite8(GPIO4CTLO_REG, 0x30);
	i2c_del_driver(&m12mo_driver);
}

int m12mo_s_power_fac(int on)
{
	struct m12mo_device *dev = to_m12mo_sensor(sdd);
	int ret;
    struct i2c_client *client = v4l2_get_subdevdata(sdd);

    printk("%s: on: %d",__func__, on);
    dev->m12mo_mode = M12MO_PARAMETER_MODE;
	if (dev->power == on)
	{
		printk("%s %d\t on: %d !!!!!!!!!!!!! \n",__func__,__LINE__,on);
		return 0;
	}

	dev->wait_irq_flag = M12MO_IRQ_COMMAND_AVAILABDLE;
	if (on) {
	    if(dev->disable_irq_flag){
	        enable_irq(client->irq);
			dev->disable_irq_flag = 0;
		}
		dev->cmd = M12MO_POWERING_ON;
		ret = power_up(sdd);
		if (ret)
			return ret;
		dev->power = 1;
        dev->flashrom_mode = 0;

		ret = __m12mo_bootrom_mode_start(sdd);
		if (ret)
			goto startup_failure;

			ret = __m12mo_fw_start(sdd);
			if (ret)
				goto startup_failure;

#ifdef CONFIG_ASUS_FACTORY_MODE
            //=== Only for factory, tell m12mo in fac image now! ===//
            (void) m12mo_writeb(sdd, CATEGORY_ASUS, 0x02, 0x01);
#endif

	} else {
        disable_irq(client->irq);
		dev->disable_irq_flag = 1;
		ret = power_down(sdd);
		dev->power = 0;
	}

	return ret;

startup_failure:
	power_down(sdd);
	dev->power = 0;
	return ret;
}

int m12mo_read_fac(u8 len, u8 category, u8 reg, u32 *val)
{
#if 1
	struct i2c_client *client = v4l2_get_subdevdata(sdd);
	unsigned char data[5];
	struct i2c_msg msg[2];
	unsigned char recv_data[len + 1];
	int ret;

	if(m12mo_trace_log_lock == DOWNLOADING) {
        printk("@%s %d, m12mo_trace_log_lock is %d, so break here!\n", __func__, __LINE__, m12mo_trace_log_lock);
		return -ENODEV;
	}
	if (!client->adapter)
		return -ENODEV;

	if (len != 1 && len != 2 && len != 4)	{
		dev_err(&client->dev, "Wrong data size\n");
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = sizeof(data);
	msg[0].buf = data;

	data[0] = 5;
	data[1] = M12MO_BYTE_READ;
	data[2] = category;
	data[3] = reg;
    data[4] = len;

	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = len + 1;
	msg[1].buf = recv_data;

	/* isp firmware becomes stable during this time*/
	usleep_range(200, 200);

	ret = i2c_transfer(client->adapter, msg, 2);

	if (ret == 2) {
		if (len == 0x01)
			*val = recv_data[1];
		else if (len == 0x02)
			*val = recv_data[1] << 8 | recv_data[2];
		else
			*val = recv_data[1] << 24 | recv_data[2] << 16
				| recv_data[3] << 8 | recv_data[4];
	}

	printk("%s len :%d cat, reg, val: 0x%02x, 0x%02x, 0x%02x\n", __func__, len, category, reg, *val);
	dev_dbg(&client->dev,
		"%s len :%d cat, reg, val: 0x%02x, 0x%02x, 0x%02x\n",
		__func__, len, category, reg, *val);

	return (ret == 2) ? 0 : -EIO;
#endif
return 0;
}

int m12mo_write_fac(u8 len, u8 category, u8 reg, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sdd);
	u8 data[len + 4];
	struct i2c_msg msg;
	int ret;
	const int num_msg = 1;
	if (!client->adapter)
		return -ENODEV;

	if (len != 1 && len != 2 && len != 4) {
		dev_err(&client->dev, "Wrong data size\n");
		return -EINVAL;
	}

	dev_dbg(&client->dev,
		"%s len :%d cat, reg, val: 0x%02x, 0x%02x, 0x%02x\n",
		__func__, len, category, reg, val);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	data[0] = msg.len;
	data[1] = M12MO_BYTE_WRITE;
	data[2] = category;
	data[3] = reg;
	switch (len) {
	case 1:
		data[4] = val;
		break;
	case 2:
		data[4] = ((val >> 8) & 0xFF);
		data[5] = (val & 0xFF);
		break;
	case 4:
		data[4] = ((val >> 24) & 0xFF);
		data[5] = ((val >> 16) & 0xFF);
		data[6] = ((val >> 8) & 0xFF);
		data[7] = (val & 0xFF);
		break;
	default:
		/* No possible to happen - len is already validated */
		break;
	}

	/* isp firmware becomes stable during this time*/
	usleep_range(200, 200);

	ret = i2c_transfer(client->adapter, &msg, 1);
	if(len ==1)
	printk("@%s, Write reg. len = %d, Category=0x%02X Reg=0x%02X Value=0x%X ret=%s\n", __func__,
		len, category, reg, data[4], (ret == 1) ? "OK" : "Error");
	else
	printk("@%s, Write reg. len = %d, Category=0x%02X Reg=0x%02X Value=0x%X ret=%s\n", __func__,
		len, category, reg, val, (ret == 1) ? "OK" : "Error");
	return ret == num_msg ? 0 : -EIO;
}

int m12mo_status_fac(void)
{
	struct m12mo_device *dev;
        if (sdd == NULL) return 0;

	dev = to_m12mo_sensor(sdd);
        return dev->power;
}

#if 0
static void inline m12mo_set_ois_mode(struct v4l2_subdev *sd, m12mo_ois_mode_t ois_mode) {
    struct m12mo_device *dev = to_m12mo_sensor(sd);

	mutex_lock(&dev->m12mo_ois_work_lock);

	mutex_unlock(&dev->m12mo_ois_work_lock);
}
#endif
EXPORT_SYMBOL(m12mo_write_fac);
EXPORT_SYMBOL(m12mo_read_fac);
EXPORT_SYMBOL(m12mo_s_power_fac);
EXPORT_SYMBOL(m12mo_status_fac);
module_init(init_m12mo);
module_exit(exit_m12mo);

MODULE_DESCRIPTION("M12MO ISP driver");
MODULE_AUTHOR("Kriti Pachhandara <kriti.pachhandara@intel.com>");
MODULE_LICENSE("GPL");

