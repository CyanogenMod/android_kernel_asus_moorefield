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
#include <media/m10mo_atomisp.h>
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
#include "m10mo.h"
#include <linux/HWVersion.h>
#include <linux/m10mo_workaround.h>
#include <asm/intel_scu_pmic.h>

#define off_c51
#define PREVIEW_TEST_PATTERN 0
#define GPIO4CTLO_REG 0x82 //This REGISTER is for SKY81296 enable pin.
//#define NOIRQ
/* cross component debug message flag */
int dbglvl = 0;

static u8 m10mo_capture_pre_flag;
static bool isCaptureMode = false;
static bool openIntelISP = true;
static bool AFDone = true;
struct v4l2_subdev *sdd;
module_param(dbglvl, int, 0644);
MODULE_PARM_DESC(dbglvl, "debug message on/off (default:off)");

/*
 * m10mo_read -  I2C read function
 * @reg: combination of size, category and command for the I2C packet
 * @size: desired size of I2C packet
 * @val: read value
 *
 * Returns 0 on success, or else negative errno.
*/

int getOpenIntelISP(void){
    return openIntelISP;
}

EXPORT_SYMBOL(getOpenIntelISP);

void setOpenIntelISP(bool b){

    openIntelISP = b;
}

EXPORT_SYMBOL(setOpenIntelISP);

void startCapture(void){
    printk("@%s \n", __func__);
    m10mo_set_still_capture(sdd);
}

EXPORT_SYMBOL(startCapture);

static int m10mo_read(struct v4l2_subdev *sd, u8 len, u8 category, u8 reg, u32 *val)
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
	data[1] = M10MO_BYTE_READ;
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
 * m10mo_write - I2C command write function
 * @reg: combination of size, category and command for the I2C packet
 * @val: value to write
 *
 * Returns 0 on success, or else negative errno.
 */
static int m10mo_write(struct v4l2_subdev *sd, u8 len, u8 category, u8 reg, u32 val)
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
	data[1] = M10MO_BYTE_WRITE;
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
	printk("@%s, Write reg. Category=0x%02X Reg=0x%02X Value=0x%X ret=%s\n", __func__,
		category, reg, data[4], (ret == 1) ? "OK" : "Error");
	else
	printk("@%s, Write reg. Category=0x%02X Reg=0x%02X Value=0x%X ret=%s\n", __func__,
		category, reg, val, (ret == 1) ? "OK" : "Error");
	return ret == num_msg ? 0 : -EIO;
}

int m10mo_writeb(struct v4l2_subdev *sd, u8 category, u8 reg, u32 val)
{
	return m10mo_write(sd, 1, category, reg, val);
}

int m10mo_writew(struct v4l2_subdev *sd, u8 category, u8 reg, u32 val)
{
	return m10mo_write(sd, 2, category, reg, val);
}

int m10mo_writel(struct v4l2_subdev *sd, u8 category, u8 reg, u32 val)
{
	return m10mo_write(sd, 4, category, reg, val);
}

int m10mo_readb(struct v4l2_subdev *sd, u8 category, u8 reg, u32 *val)
{
	return m10mo_read(sd, 1, category, reg, val);
}

int m10mo_readw(struct v4l2_subdev *sd, u8 category, u8 reg, u32 *val)
{
	return m10mo_read(sd, 2, category, reg, val);
}

int m10mo_readl(struct v4l2_subdev *sd, u8 category, u8 reg, u32 *val)
{
	return m10mo_read(sd, 4, category, reg, val);
}

int m10mo_memory_write(struct v4l2_subdev *sd, u8 cmd, u16 len, u32 addr, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m10mo_device *m10mo_dev =  to_m10mo_sensor(sd);
	struct i2c_msg msg;
	u8 *data = m10mo_dev->message_buffer;
	int i, ret;

	dev_dbg(&client->dev, "Write mem. cmd=0x%02X len=%d addr=0x%X\n", cmd, len, addr);

	if (!client->adapter)
		return -ENODEV;

	if ((len + 8) > sizeof(m10mo_dev->message_buffer))
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

	for (i = M10MO_I2C_RETRY; i; i--) {
		ret = i2c_transfer(client->adapter, &msg, 1);
		if (ret == 1) {
			return 0;
		}
		msleep(20);
	}

	return ret;
}

int m10mo_memory_read(struct v4l2_subdev *sd, u16 len, u32 addr, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m10mo_device *m10mo_dev =  to_m10mo_sensor(sd);
	struct i2c_msg msg;
	unsigned char data[8];
	u8 *recv_data = m10mo_dev->message_buffer;
	int i, err = 0;

	if (!client->adapter)
		return -ENODEV;

	if (len <= 0)
		return -EINVAL;

	if ((len + 3) > sizeof(m10mo_dev->message_buffer))
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

	for (i = M10MO_I2C_RETRY; i; i--) {
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
	for (i = M10MO_I2C_RETRY; i; i--) {
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

/**
 * m10mo_setup_flash_controller - initialize flash controller
 *
 * Flash controller requires additional setup before
 * the use.
 */
int m10mo_setup_flash_controller(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data = 0x7F;
	int res;

	res = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT,
				 1, 0x13000005, &data);
	if (res < 0)
		dev_err(&client->dev, "Setup flash controller failed\n");
	return res;
}

/**
 * m10mo_wait_interrupt - Clear interrupt pending bits and unmask interrupts
 *
 * Before writing desired interrupt value the INT_FACTOR register should
 * be read to clear pending interrupts.
 */

static int m10mo_enable_interrupt(struct v4l2_subdev *sd, u8 requested_cmd){
    int ret = 0;
	u32 enable_interrupt_bit = 0;
	switch (requested_cmd) {

	case M10MO_START_AF:
	case M10MO_ZOOM_LENS_TO_PR:
    case M10MO_FOCUS_LENS_TO_PR:
	case M10MO_START_OPTICAL_ZOOM:
	     enable_interrupt_bit = 0x02;
	     break;

	case M10MO_CAMERA_START:
	case M10MO_SINGLE_CAPTURE_MODE:
         enable_interrupt_bit = 0x08;
	     break;

    case M10MO_MONITOR_MODE_ZSL_REQUEST_CMD:
	case M10MO_HOME_SEARCHING_MODE:
         enable_interrupt_bit = 0x0F;
	     break;

	case M10MO_PARAMETER_MODE_REQUEST_CMD:
	     enable_interrupt_bit = 0x01;
         break;

	case M10MO_START_DIGITAL_ZOOM:
	     enable_interrupt_bit = 0x04;
         break;

    default:
	     printk("@%s, unexpected request cmd effect: %d \n",__func__, requested_cmd);
	     return -ENODEV;
	}
    ret = m10mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_INT_ENABLE, enable_interrupt_bit);
    return ret;
}

int m10mo_request_cmd_effect(struct v4l2_subdev *sd, u8 requested_cmd, void* data)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
//	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
    dev->requested_cmd = requested_cmd;
    printk("@%s %d start, requested_cmd = %d\n", __func__, __LINE__, requested_cmd);

    m10mo_enable_interrupt(sd, requested_cmd);

	switch (dev->requested_cmd) {
	case M10MO_CAMERA_START:
		ret = m10mo_writeb(sd, CATEGORY_FLASHROM, FLASH_CAM_START, 0x01);
		break;

	case M10MO_PARAMETER_MODE_REQUEST_CMD:
        ret = m10mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_SYSMODE, 0x01);
		break;

	case M10MO_MONITOR_MODE_ZSL_REQUEST_CMD:
		ret = m10mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_SYSMODE, 0x02);
		break;

	case M10MO_SINGLE_CAPTURE_MODE:
		ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL,0x05, 0x01);
		gpio_set_value(60, 0);
		printk(KERN_INFO "m10mo driver, gpio 60 set low\n");
		break;

	case M10MO_HOME_SEARCHING_MODE:
		ret = m10mo_writeb(sd, CATEGORY_PARAM, 0x50, 0x02);
        break;

	case M10MO_ZOOM_LENS_TO_PR:
		ret = m10mo_writew(sd, CATEGORY_TEST, 0x3e, 0x0);
        ret = m10mo_writew(sd, CATEGORY_TEST, 0x3f, 0x0);
        break;

	case M10MO_FOCUS_LENS_TO_PR:
		ret = m10mo_writew(sd, CATEGORY_TEST, 0x3c, 0x0);
        ret = m10mo_writew(sd, CATEGORY_TEST, 0x3d, 0x0);
        break;

	case M10MO_START_OPTICAL_ZOOM:
		ret = m10mo_writeb(sd, CATEGORY_MONITOR, 0x20, *(u32*)data);
        printk("@%s %d, write zoom position succeed, step is %d \n", __func__, __LINE__, *(u32*)data);
        break;

	case M10MO_START_AF:
		ret = m10mo_writeb(sd, CATEGORY_LENS, 0x2, 0x13);
        break;

	case M10MO_START_DIGITAL_ZOOM:
		 ret = m10mo_writeb(sd, CATEGORY_MONITOR, 0x1, *(u32*)data);
	     break;

    default:
        printk("@%s %d, Invalid Mode is %d, break!\n", __func__, __LINE__, requested_cmd);
		return -ENODEV;
	}

    printk("m10mo, 45678\n");
    ret = m10mo_wait_mode_change(sd, dev->requested_cmd, M10MO_INIT_TIMEOUT);
	printk("m10mo, 87654\n");

    if(requested_cmd == M10MO_PARAMETER_MODE_REQUEST_CMD){
	    dev->m10mo_mode = M10MO_PARAMETER_MODE;

	}else if( requested_cmd == M10MO_MONITOR_MODE_ZSL_REQUEST_CMD ){
	    dev->m10mo_mode = M10MO_MONITOR_MODE_ZSL;
    }

	dev->cmd = M10MO_NO_CMD_REQUEST;

	return ret;
}

static int is_m10mo_in_monitor_mode(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	if (dev->cmd == M10MO_MONITOR_MODE_PANORAMA ||
	    dev->cmd == M10MO_MONITOR_MODE_ZSL_REQUEST_CMD ||
	    dev->cmd == M10MO_MONITOR_MODE ||
	    dev->cmd == M10MO_MONITOR_MODE_HIGH_SPEED)
		return 1;

	return 0;
}

static int m10mo_set_monitor_parameters(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/*int id = M10MO_GET_FOCUS_MODE(dev->fw_type);*/
	int ret=0;

	dev_info(&client->dev,
		 "%s: af_mode: 0x%x exe_mode: 0x%x posx: 0x%x, posy: 0x%x\n",
		 __func__, dev->monitor_params.af_mode,
		 dev->monitor_params.exe_mode,
		 dev->monitor_params.af_touch_posx,
		 dev->monitor_params.af_touch_posy);

	/*ret = m10mo_writeb(sd, CATEGORY_LENS,*/
			   /*m10m0_af_parameters[id].af_mode,*/
			   /*dev->monitor_params.af_mode);*/
	/*if (ret)*/
		/*return ret;*/

	/*ret = m10mo_writew(sd, CATEGORY_LENS,*/
			   /*m10m0_af_parameters[id].af_touch_posx,*/
			   /*dev->monitor_params.af_touch_posx);*/
	/*if (ret)*/
		/*return ret;*/

	/*ret = m10mo_writew(sd, CATEGORY_LENS,*/
			   /*m10m0_af_parameters[id].af_touch_posy,*/
			   /*dev->monitor_params.af_touch_posy);*/
	/*if (ret)*/
		/*return ret;*/

	/*ret = m10mo_writeb(sd, CATEGORY_LENS,*/
			   /*m10m0_af_parameters[id].af_execution,*/
			   /*dev->monitor_params.exe_mode);*/

	/*if (ret)*/
		/*return ret;*/

	/*if (dev->monitor_params.flash_mode == LED_TORCH)*/
		/*ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH,*/
				   /*LED_TORCH,*/
				   /*dev->monitor_params.torch);*/
	/*else*/
		/*ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH,*/
				   /*FLASH_MODE,*/
				   /*dev->monitor_params.flash_mode);*/

	return ret;
}

int m10mo_wait_mode_change(struct v4l2_subdev *sd, u8 mode, u32 timeout)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
    printk("m10mo, 56789  \n");
	ret = wait_event_interruptible_timeout(dev->irq_waitq, dev->cmd == mode, msecs_to_jiffies(timeout));
	printk("m10mo, 98765  \n");
	if (ret > 0) {
		if (0&is_m10mo_in_monitor_mode(sd)){
			return m10mo_set_monitor_parameters(sd);
        }
		return 0;
	} else if (ret == 0) {
		dev_err(&client->dev, "m10mo_wait_mode_change timed out\n");
		return -ETIMEDOUT;
	}

	return ret;
}

int __m10mo_param_mode_set(struct v4l2_subdev *sd)
{
	int ret;

    printk("@%s %d start\n", __func__, __LINE__);
	ret = m10mo_request_cmd_effect(sd, M10MO_PARAMETER_MODE_REQUEST_CMD, NULL);
	if (ret)
		return ret;

	return ret;
}

static int m10mo_detect(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m10mo_version *ver = &dev->ver;
	int ret;

        ret = m10mo_read(sd, 1, CATEGORY_SYSTEM, SYSTEM_CUSTOMER_CODE,
			&ver->customer);
	if (!ret)
		ret = m10mo_read(sd, 1, CATEGORY_SYSTEM, SYSTEM_PROJECT_CODE,
				&ver->project);
	dev_info(&client->dev, "Customer/Project[0x%x/0x%x]\n", dev->ver.customer,
				dev->ver.project);
	return 0;
}

static int __m10mo_fw_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
    printk("@%s %d start\n", __func__, __LINE__);

	/*
	 * Correct the pll value before fw start
	 */
    ret = m10mo_update_pll_setting(sd);
    if (ret < 0)
        return ret;

	ret = m10mo_setup_flash_controller(sd);
	if (ret < 0)
		return ret;

	ret = m10mo_request_cmd_effect(sd, M10MO_CAMERA_START, NULL);
	if (ret)
		return ret;

	ret = m10mo_detect(sd);
	if (ret)
		return ret;

	dev_info(&client->dev, "ISP Booted Successfully\n");

    ret = m10mo_request_cmd_effect(sd, M10MO_HOME_SEARCHING_MODE, NULL);
    if (ret)
        return ret;

	return 0;
}

static int m10mo_fw_start(struct v4l2_subdev *sd, u32 val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret;
    printk("@%s %d start\n", __func__, __LINE__);

	mutex_lock(&dev->input_lock);
	ret = __m10mo_fw_start(sd);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int m10mo_set_af_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret = 0;
	int polling_times = 10;
	u32 normal_mode = 1;
	u32 read_back = 0;
    printk("@%s %d start\n", __func__, __LINE__);

    if(dev->m10mo_mode == M10MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set AF during M10MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
	//======= Hard code here
	//========because temporarily there is one mode (normal mode).
	(void) m10mo_writeb(sd, CATEGORY_LENS, 0x00, normal_mode);

	for(polling_times = 10; polling_times > 0; --polling_times){
        (void) m10mo_readb(sd, CATEGORY_LENS, 0x00, &read_back);
        if(read_back == normal_mode){
		     printk("@%s %d success !\n", __func__, __LINE__);
			 return 0;
	    }
	}
	printk("@%s %d fail !\n", __func__, __LINE__);
	return -EINVAL;
#if 0
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);

	u32 cur_af_mode;

	switch (val) {
	case EXT_ISP_FOCUS_MODE_NORMAL:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_normal;
		break;
	case EXT_ISP_FOCUS_MODE_MACRO:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_macro;
		break;
	case EXT_ISP_FOCUS_MODE_TOUCH_AF:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_touch;
		break;
	case EXT_ISP_FOCUS_MODE_PREVIEW_CAF:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_preview_caf;
		break;
	case EXT_ISP_FOCUS_MODE_MOVIE_CAF:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_movie_caf;
		break;
	case EXT_ISP_FOCUS_MODE_FACE_CAF:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_face_caf;
		break;
	case EXT_ISP_FOCUS_MODE_TOUCH_MACRO:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_touch_macro;
		break;
	case EXT_ISP_FOCUS_MODE_TOUCH_CAF:
		dev->monitor_params.af_mode = m10m0_af_parameters[id].af_touch_caf;
		break;
	default:
		return -EINVAL;
	}

	if (!is_m10mo_in_monitor_mode(sd))
		return ret;

	ret = m10mo_read(sd, 1, CATEGORY_LENS,
			m10m0_af_parameters[id].af_mode,
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
	ret = m10mo_writeb(sd, CATEGORY_LENS,
			m10m0_af_parameters[id].af_mode,
			dev->monitor_params.af_mode);
#endif
	return ret;
}

static int m10mo_set_af_execution(struct v4l2_subdev *sd, s32 val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);
	int ret = 0;
/////////
return 0;
////////
	if(dev->m10mo_mode == M10MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set AF during M10MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
	switch (val) {
	case EXT_ISP_FOCUS_STOP:
		dev->monitor_params.exe_mode = m10m0_af_parameters[id].af_stop;
		break;
	case EXT_ISP_FOCUS_SEARCH:
		dev->monitor_params.exe_mode = m10m0_af_parameters[id].af_search;
		break;
	case EXT_ISP_PAN_FOCUSING:
		dev->monitor_params.exe_mode = m10m0_af_parameters[id].af_pan_focusing;
		break;
	default:
		return -EINVAL;
	}

	dev_info(&client->dev, "%s: In monitor mode, set AF exe_mode to %d",
			 __func__, dev->monitor_params.exe_mode);
#if 0
		/* We are in monitor mode already, */
		/* exe_mode can be applied immediately */
		ret = m10mo_writeb(sd, CATEGORY_LENS,
				   m10m0_af_parameters[id].af_execution,
				   dev->monitor_params.exe_mode);
#else
	if(val == EXT_ISP_FOCUS_SEARCH){
		ret = m10mo_request_cmd_effect(sd, M10MO_START_AF, NULL);
		if (ret < 0)
			return ret;
#endif
	}
	return ret;
}

static int m10mo_set_af_position_x(struct v4l2_subdev *sd, unsigned int x)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);
    printk("@%s: set DIT AF x = %d\n", __func__, x);
//////////////
return 0;
///////////////
	dev->monitor_params.af_touch_posx = x;

    /* Set X Position to DIT*/
    ret = m10mo_writeb(sd, CATEGORY_ASUS, ASUS_TOUCH_ROI_LEFT_UPPER_X, dev->monitor_params.af_touch_posx);
	if (ret){
		dev_err(&client->dev, "set DIT AF position x failed %d\n", ret);
        return ret;
    }

	if (is_m10mo_in_monitor_mode(sd)) {
		dev_info(&client->dev,
			 "%s: In monitor mode, set AF touch X pos to 0x%x",
			 __func__, dev->monitor_params.af_touch_posx);

		/* Set X Position */
		ret = m10mo_writew(sd, CATEGORY_LENS,
				   m10m0_af_parameters[id].af_touch_posx,
				   dev->monitor_params.af_touch_posx);
	}

	if (ret)
		dev_err(&client->dev, "AutoFocus position x failed %d\n", ret);

	return ret;
}

static int m10mo_set_af_position_y(struct v4l2_subdev *sd, unsigned int y)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);
    printk("@%s: set DIT AF y = %d\n", __func__, y);

	dev->monitor_params.af_touch_posy = y;
//////////////
return 0;
///////////////
    /* Set Y Position to DIT*/
    ret = m10mo_writeb(sd, CATEGORY_ASUS, ASUS_TOUCH_ROI_LEFT_UPPER_Y, dev->monitor_params.af_touch_posy);
	if (ret){
		dev_err(&client->dev, "set DIT AF position y failed %d\n", ret);
        return ret;
    }

	if (is_m10mo_in_monitor_mode(sd)) {
		dev_info(&client->dev,
			 "%s: In monitor mode, set AF touch Y pos to 0x%x",
			 __func__, dev->monitor_params.af_touch_posy);

		/* Set Y Position */
		ret = m10mo_writew(sd, CATEGORY_LENS,
				   m10m0_af_parameters[id].af_touch_posy,
				   dev->monitor_params.af_touch_posy);
	}

	if (ret)
		dev_err(&client->dev, "AutoFocus position y failed %d\n", ret);

	return ret;
}

static int m10mo_set_af_width(struct v4l2_subdev *sd, unsigned int w)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m10mo_device *dev = to_m10mo_sensor(sd);
    printk("@%s: set DIT AF ROI width = %d\n", __func__, w);
//////////////////
return 0;
/////////////////
    dev->monitor_params.af_touch_width = w;

    /* Set AF ROI width to DIT */
    ret = m10mo_writeb(sd, CATEGORY_ASUS, ASUS_TOUCH_WIDTH, dev->monitor_params.af_touch_width);

    if (ret)
        dev_err(&client->dev, "set DIT AF ROI width failed %d\n", ret);

    return ret;
}

static int m10mo_set_af_height(struct v4l2_subdev *sd, unsigned int h)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m10mo_device *dev = to_m10mo_sensor(sd);
    printk("@%s: set DIT AF ROI height = %d\n", __func__, h);
///////////////
return 0;
////////////////
    dev->monitor_params.af_touch_height = h;

    /* Set AF ROI height to DIT */
    ret = m10mo_writeb(sd, CATEGORY_ASUS, ASUS_TOUCH_HEIGHT, dev->monitor_params.af_touch_height);

    if (ret)
        dev_err(&client->dev, "set DIT AF ROI height failed %d\n", ret);

    return ret;
}

static int m10mo_set_ae_position_x(struct v4l2_subdev *sd, unsigned int x)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m10mo_device *dev = to_m10mo_sensor(sd);
    printk("@%s: set DIT AE x = %d\n", __func__, x);
////////////////////////////////////
 return 0;
///////////////////////////////////
    dev->monitor_params.ae_touch_posx = x;

    /* Set AE X Position to DIT */
    ret = m10mo_writeb(sd, CATEGORY_ASUS, ASUS_TOUCH_POSITION_X, dev->monitor_params.ae_touch_posx);

    if (ret)
        dev_err(&client->dev, "set DIT AE position x failed %d\n", ret);

    return ret;
}

static int m10mo_set_ae_position_y(struct v4l2_subdev *sd, unsigned int y)
{
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret = 0;
    struct m10mo_device *dev = to_m10mo_sensor(sd);
    printk("@%s: set DIT AE y = %d\n", __func__, y);
////////////////////////////////////
 return 0;
///////////////////////////////////
    dev->monitor_params.ae_touch_posy = y;

    /* Set AE Y Position to DIT */
    ret = m10mo_writew(sd, CATEGORY_ASUS, ASUS_TOUCH_POSITION_Y, dev->monitor_params.ae_touch_posy);

    if (ret)
        dev_err(&client->dev, "set DIT AE position y failed %d\n", ret);

    return ret;
}

/* Because of different m10m0 firmwares and parameter values, transform
   the values to the macro definition */
static u32 m10mo_af_parameter_transform(struct v4l2_subdev *sd, u32 val)
{
	u32 ret = 0xffffffff;
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);

	if (val == m10m0_af_parameters[id].caf_status_focusing) {
		ret = CAF_STATUS_FOCUSING;
	} else if (val == m10m0_af_parameters[id].caf_status_success) {
		ret = CAF_STATUS_SUCCESS;
	} else if (val == m10m0_af_parameters[id].caf_status_fail) {
		ret = CAF_STATUS_FAIL;
	} else if (val == m10m0_af_parameters[id].caf_status_restart_check) {
		ret = CAF_STATUS_RESTART_CHECK;
	} else if (val == m10m0_af_parameters[id].af_status_invalid) {
		ret = AF_STATUS_INVALID;
	} else if (val == m10m0_af_parameters[id].af_status_focusing) {
		ret = AF_STATUS_FOCUSING;
	} else if (val == m10m0_af_parameters[id].af_status_success) {
		ret = AF_STATUS_SUCCESS;
	} else if (val == m10m0_af_parameters[id].af_status_fail) {
		ret = AF_STATUS_FAIL;
	} else if (val == m10m0_af_parameters[id].af_normal) {
		ret = AF_NORMAL;
	} else if (val == m10m0_af_parameters[id].af_macro) {
		ret = AF_MACRO;
	} else if (val == m10m0_af_parameters[id].af_touch) {
		ret = AF_TOUCH;
	} else if (val == m10m0_af_parameters[id].af_preview_caf) {
		ret = AF_PREVIEW_CAF;
	} else if (val == m10m0_af_parameters[id].af_movie_caf) {
		ret = AF_MOVIE_CAF;
	} else if (val == m10m0_af_parameters[id].af_face_caf) {
		ret = AF_FACE_CAF;
	} else if (val == m10m0_af_parameters[id].af_touch_macro) {
		ret = AF_TOUCH_MACRO;
	} else if (val == m10m0_af_parameters[id].af_touch_caf) {
		ret = AF_TOUCH_CAF;
	}

	return ret;
}

static int m10mo_get_caf_status(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 af_result;
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);

	ret = m10mo_read(sd, 1, CATEGORY_LENS,
			m10m0_af_parameters[id].af_result, &af_result);
	if (ret)
		return ret;

	af_result = m10mo_af_parameter_transform(sd, af_result);

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

static int m10mo_get_af_status(struct v4l2_subdev *sd, unsigned int *status)
{

	//int ret;
	//u32 af_result;
	//struct m10mo_device *dev = to_m10mo_sensor(sd);
#if 0
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);

	ret = m10mo_read(sd, 1, CATEGORY_LENS,
			m10m0_af_parameters[id].af_result, &af_result);
	if (ret)
		return ret;

	af_result = m10mo_af_parameter_transform(sd, af_result);

	switch (af_result) {
	case AF_STATUS_INVALID:
		*status = EXT_ISP_AF_STATUS_INVALID;
		break;
	case AF_STATUS_FOCUSING:
		*status = EXT_ISP_AF_STATUS_FOCUSING;
		break;
	case AF_STATUS_SUCCESS:
		*status = EXT_ISP_AF_STATUS_SUCCESS;
		break;
	case AF_STATUS_FAIL:
		*status = EXT_ISP_AF_STATUS_FAIL;
		break;
	default:
		return -EINVAL;
	}
	return ret;
#else
	return AFDone;
#endif

}

/* Adding this as requested by HAL. Can be removed if HAL is saving af mode */
static int m10mo_get_af_mode(struct v4l2_subdev *sd, unsigned int *status)
{
	int ret;
	u32 af_mode;
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int id = M10MO_GET_FOCUS_MODE(dev->fw_type);

	ret = m10mo_read(sd, 1, CATEGORY_LENS,
			m10m0_af_parameters[id].af_result, &af_mode);
	if (ret)
		return ret;

	af_mode = m10mo_af_parameter_transform(sd, af_mode);

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
}

static int m10mo_set_optical_zoom_position(struct v4l2_subdev *sd, u32 data)
{
    int ret = 0;
	u32 val = 0;
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	pr_info("%s\n", __func__);
    if(data < 1 || data > 9 ){
	   pr_info("m10mo, wrong optical zoom data is %d \n", data);
	   return -EINVAL;;
	}

	if(dev->m10mo_mode == M10MO_PARAMETER_MODE){
	    pr_info("%s, illegal to set AF during M10MO_PARAMETER_MODE, break!\n", __func__);
		return -EINVAL;
	}
	ret = m10mo_readb(sd, CATEGORY_MONITOR, 0x20, &val);
	if(val == data){
	    pr_info("m10mo, get zoom step is %d, set zoom data is %d. No need to set\n", val, data);
	    return 0;
	}
    ret = m10mo_request_cmd_effect(sd, M10MO_START_OPTICAL_ZOOM, &data);
    if (ret)
        return ret;


    return 0;
}

static int m10mo_set_flash_mode(struct v4l2_subdev *sd, unsigned int val)
{
	int ret = 0;
#if 0
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* by default force the torch off, value depends on incoming flash mode */
	dev->monitor_params.torch = LED_TORCH_OFF;

	switch (val) {
	case EXT_ISP_FLASH_MODE_OFF:
		dev->monitor_params.flash_mode = FLASH_MODE_OFF;
		break;
	case EXT_ISP_FLASH_MODE_ON:
		dev->monitor_params.flash_mode = FLASH_MODE_ON;
		break;
	case EXT_ISP_FLASH_MODE_AUTO:
		dev->monitor_params.flash_mode = FLASH_MODE_AUTO;
		break;
	case EXT_ISP_LED_TORCH_OFF:
		dev->monitor_params.flash_mode = LED_TORCH;
		dev->monitor_params.torch = LED_TORCH_OFF;
		break;
	case EXT_ISP_LED_TORCH_ON:
		dev->monitor_params.flash_mode = LED_TORCH;
		dev->monitor_params.torch = LED_TORCH_ON;
		break;
	default:
		return -EINVAL;
	}

	/* Only apply setting if we are in monitor mode */
	if (!is_m10mo_in_monitor_mode(sd))
		return ret;

	dev_info(&client->dev, "%s: In monitor mode, set flash mode to %d",
		 __func__, dev->monitor_params.flash_mode);

	/* TODO get current flash mode, and apply new setting only when needed? */

	if (dev->monitor_params.flash_mode == LED_TORCH)
		ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LED_TORCH,
				   dev->monitor_params.torch);
	else
		ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, FLASH_MODE,
				   dev->monitor_params.flash_mode);

#endif
	return ret;
}

static int m10mo_set_lens_position(struct v4l2_subdev *sd)
{
    int ret = 0;
	pr_info("%s\n", __func__);

    ret = m10mo_request_cmd_effect(sd, M10MO_ZOOM_LENS_TO_PR, NULL);
    if (ret)
        return ret;

    ret = m10mo_request_cmd_effect(sd, M10MO_FOCUS_LENS_TO_PR, NULL);
    if (ret)
        return ret;


    return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
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
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	pr_info("%s\n", __func__);
    ret = m10mo_set_lens_position(sd);

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

static int __m10mo_bootrom_mode_start(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct i2c_adapter *adapter = client->adapter;
	u32 dummy;
	int ret;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;
	/* Dummy read to verify I2C functionality */
	ret = m10mo_readl(sd, CATEGORY_FLASHROM,  REG_FLASH_ADD, &dummy);
	printk(KERN_INFO "__m10mo_bootrom_mode_start, dummy is %d \n", dummy);
	if (ret < 0)
		dev_err(&client->dev, "Dummy I2C access fails\n");

	return ret;
}

static int __m10mo_s_power(struct v4l2_subdev *sd, int on, bool fw_update_mode)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret;

	pr_info("%s: on: %d, fw_update_mode is %d\n", __func__, on, fw_update_mode);
    dev->m10mo_mode = M10MO_PARAMETER_MODE;
	if (dev->power == on)
		return 0;

	if (on) {
		dev->cmd = M10MO_POWERING_ON;
		ret = power_up(sd);
		if (ret)
			return ret;
		dev->power = 1;

		ret = __m10mo_bootrom_mode_start(sd);
		if (ret)
			goto startup_failure;

		if (!fw_update_mode) {
			ret = __m10mo_fw_start(sd);
			if (ret)
				goto startup_failure;
		}
	} else {
		ret = power_down(sd);
		dev->power = 0;
	}

	return ret;

startup_failure:
	power_down(sd);
	dev->power = 0;
	return ret;
}

int m10mo_set_panorama_monitor(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u32 val;

	dev_info(&client->dev,
		"%s mode: %d Width: %d, height: %d, cmd: 0x%x vdis: %d\n",
		__func__, dev->cmd, dev->curr_res_table[dev->fmt_idx].width,
		dev->curr_res_table[dev->fmt_idx].height,
		dev->curr_res_table[dev->fmt_idx].command,
		(int)dev->curr_res_table[dev->fmt_idx].vdis);

	/* Check if m10mo already streaming @ required resolution */
	ret = m10mo_readb(sd, CATEGORY_PARAM,  PARAM_MON_SIZE, &val);
	if (ret)
		goto out;

	/* If mode is monitor mode and size same, do not configure again*/
	if (dev->cmd == M10MO_MONITOR_MODE_PANORAMA &&
		val == dev->curr_res_table[dev->fmt_idx].command) {
		dev_info(&client->dev,
			"%s Already streaming with required size\n", __func__);
		return 0;
	}

	if (dev->cmd != M10MO_CAMERA_START &&
		dev->cmd != M10MO_PARAMETER_MODE_REQUEST_CMD) {
		/* Already in panorama mode. So swith to parameter mode */
		ret = __m10mo_param_mode_set(sd);
		if (ret)
			goto out;
	}

	/* Change the Monitor Size */
	ret = m10mo_write(sd, 1, CATEGORY_PARAM, PARAM_MON_SIZE,
			dev->curr_res_table[dev->fmt_idx].command);
	if (ret)
		goto out;

	/* Set Panorama mode */
	ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, CAPTURE_MODE,
			CAP_MODE_PANORAMA);
	if (ret)
		goto out;

	/* Setting output to NV12/NV21. */
	ret = m10mo_writeb(sd, CATEGORY_PARAM, OUTPUT_FMT_SELECT,
			OUTPUT_FMT_SELECT_NV12NV21);
	if (ret)
		goto out;

	/* Select either NV12 or NV21 based on the format set from user space */
	val = dev->format.code == V4L2_MBUS_FMT_CUSTOM_NV21 ?
		CHOOSE_NV12NV21_FMT_NV21 : CHOOSE_NV12NV21_FMT_NV12;
	ret = m10mo_writeb(sd, CATEGORY_PARAM, CHOOSE_NV12NV21_FMT, val);
	if (ret)
		goto out;

	/* Enable metadata (the command sequence PDF-example) */
	ret = m10mo_writeb(sd, CATEGORY_PARAM, MON_METADATA_SUPPORT_CTRL,
			MON_METADATA_SUPPORT_CTRL_EN);
	if (ret)
		goto out;

	/* Enable interrupt signal */
	ret = m10mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_INT_ENABLE, 0x01);
	if (ret)
		goto out;

	/* Go to Panorama Monitor mode */
	ret = m10mo_request_cmd_effect(sd, M10MO_MONITOR_MODE_PANORAMA, NULL);
	if (ret)
		goto out;

	ret = m10mo_wait_mode_change(sd, M10MO_MONITOR_MODE_PANORAMA,
				M10MO_INIT_TIMEOUT);
	if (ret < 0)
		goto out;

	return 0;
out:
	dev_err(&client->dev, "m10mo_set_panorama_monitor failed %d\n", ret);
	return ret;
}

int m10mo_set_zsl_monitor(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
//	int mode = M10MO_GET_RESOLUTION_MODE(dev->fw_type);
    const struct m10mo_resolution *capture_res =
			resolutions[M10MO_MODE_CAPTURE_INDEX];
	int ret;
    /*u32 fmt;*/

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
	if (dev->cmd != M10MO_CAMERA_START &&
		dev->cmd != M10MO_PARAMETER_MODE_REQUEST_CMD &&
        dev->cmd != M10MO_HOME_SEARCHING_MODE) {
		//
		// At this stage means we are already at ZSL. So switch to
		// param mode first and reset all the parameters.
		//

		ret = __m10mo_param_mode_set(sd);
		if (ret)
			goto out;
	}
*/
    /* Change the Monitor Size */
	ret = m10mo_writeb(sd, CATEGORY_PARAM, PARAM_MON_SIZE,
			dev->curr_res_table[dev->fmt_idx].command);
	if (ret)
		goto out;

	/* Set ZSL mode */
	ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, CAPTURE_MODE,
				CAP_MODE_INFINITY_ZSL);
	if (ret)
		goto out;



	/* Change the capture size */
	/*ret = m10mo_writeb(sd, CATEGORY_CAPTURE_PARAM, CAPP_MAIN_IMAGE_SIZE,*/
			/*capture_res[dev->capture_res_idx].command);*/
	/*if (ret)*/
		/*goto out;*/

	/* Select monitor/movie mode */
	/*ret = m10mo_write(sd, 1, CATEGORY_PARAM, MOVIE_MODE,*/
			/*dev->run_mode == CI_MODE_VIDEO ? 0x01 : 0x00);*/
	/*if (ret)*/
		/*goto out;*/

	/* vdis on/off */
	/*m10mo_writeb(sd, CATEGORY_MONITOR, PARAM_VDIS,*/
			 /*dev->curr_res_table[dev->fmt_idx].vdis ? 0x01 : 0x00);*/

	/* Select either NV12 or NV21 based on the format set from user space */
	/*fmt = dev->format.code == V4L2_MBUS_FMT_CUSTOM_NV21 ?*/
		/*CHOOSE_NV12NV21_FMT_NV21 : CHOOSE_NV12NV21_FMT_NV12;*/
	/*ret = m10mo_writeb(sd, CATEGORY_PARAM, CHOOSE_NV12NV21_FMT, fmt);*/
	/*if (ret)*/
		/*goto out;*/


	/* Go to ZSL Monitor mode */
	ret = m10mo_request_cmd_effect(sd, M10MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
	if (ret)
		goto out;

	return 0;
out:
	dev_err(&client->dev, "m10mo_set_zsl_monitor failed %d\n", ret);
	return ret;
}

static u32 __get_dual_capture_value(u8 capture_mode)
{
	switch(capture_mode) {
	case M10MO_CAPTURE_MODE_ZSL_BURST:
		return DUAL_CAPTURE_BURST_CAPTURE_START;
	case APP_LLS_CAP_MODE_ZSL:
		return DUAL_CAPTURE_LLS_CAPTURE_START;
	case M10MO_CAPTURE_MODE_ZSL_NORMAL:
		return DUAL_CAPTURE_ZSL_CAPTURE_START;
	case APP_HDR_CAP_MODE_ZSL:
	case M10MO_CAPTURE_MODE_ZSL_RAW:
	default:
		return DUAL_CAPTURE_SINGLE_CAPTURE_START;
	}
}

static int m10mo_set_zsl_capture(struct v4l2_subdev *sd, int sel_frame)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret, i;
	u32 val, dual_status;
	int finish = 0;

	/* TODO: Fix this. Currently we do not use this */
	(void) sel_frame;

	val = __get_dual_capture_value(dev->capture_mode);

	/* Check dual capture status before the capture request */
	for (i = POLL_NUM; i; i--) {
		ret = m10mo_readb(sd, CATEGORY_CAPTURE_CTRL, START_DUAL_STATUS,
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
	ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, START_DUAL_CAPTURE, val);
	dev_dbg(&client->dev, "%s zsl capture trigger result: %d\n",
			__func__, ret);
	return ret;
}

int m10mo_set_zsl_raw_capture(struct v4l2_subdev *sd)
{
	int ret;

	/* Set capture mode - Infinity capture 3 */
	ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, CAPTURE_MODE,
						CAP_MODE_INFINITY_ZSL);
	if (ret)
		return ret;

	/* Enable interrupt signal */
	ret = m10mo_writeb(sd, CATEGORY_SYSTEM, SYSTEM_INT_ENABLE, 0x01);
	if (ret)
		return ret;

	/* Go to ZSL Monitor mode */
	ret = m10mo_request_cmd_effect(sd, M10MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
	if (ret)
		return ret;

	ret = m10mo_wait_mode_change(sd, M10MO_MONITOR_MODE_ZSL_REQUEST_CMD,
						M10MO_INIT_TIMEOUT);
	if (ret < 0)
		return ret;

	/* switch to RAW capture mode */
	ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REG_CAP_NV12_MODE,
						RAW_CAPTURE);
	if (ret)
		return ret;

	/* RAW mode is set. Now do a normal capture */
	return m10mo_set_zsl_capture(sd, 1);
}

int m10mo_set_burst_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret = 0;

	switch(val) {
	case EXT_ISP_BURST_CAPTURE_CTRL_START:
		/* First check if already in ZSL monitor mode. If not start */
		if (dev->cmd != M10MO_MONITOR_MODE_ZSL_REQUEST_CMD) {
			ret = m10mo_set_zsl_monitor(sd);
			if (ret)
				return ret;
		}
		/* set cap mode to burst so that ZSL cap can differentiate */
		dev->capture_mode = M10MO_CAPTURE_MODE_ZSL_BURST;
		return m10mo_set_zsl_capture(sd, 1);
	case EXT_ISP_BURST_CAPTURE_CTRL_STOP:
		if (dev->capture_mode != M10MO_CAPTURE_MODE_ZSL_BURST)
			return 0;
		/* Stop the burst capture */
		ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL,
			START_DUAL_CAPTURE, DUAL_CAPTURE_BURST_CAPTURE_STOP);
		if (ret)
			return ret;
		/* captutre mode back to normal */
		dev->capture_mode = M10MO_CAPTURE_MODE_ZSL_NORMAL;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int m10mo_set_lls_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	if(!val){
	    dev->capture_mode &= ~APP_LLS_CAP_MODE_ZSL;
	}else{
        dev->capture_mode |= APP_LLS_CAP_MODE_ZSL;
	}
	return 0;

}

static int m10mo_set_hdr_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
/*
	switch(val) {
	case STOP_HDR_MODE:
		//=== switch to normal capture. HDR MODE off ===//
		printk(KERN_INFO "@m10mo_set_hdr_mode, STOP_HDR_MODE\n");
		dev->capture_mode = M10MO_CAPTURE_MODE_ZSL_NORMAL;
		break;
	case START_HDR_MODE:
		//=== switch to HDR mode ===//
		printk(KERN_INFO "@m10mo_set_hdr_mode, START_HDR_MODE\n");
		dev->capture_mode = APP_HDR_CAP_MODE_ZSL;
		break;
	case RESUME_PREVIEW_IN_HDR_MODE:
		//=== switch to HDR mode ===//
		printk(KERN_INFO "@m10mo_set_hdr_mode, RESUME_PREVIEW_IN_HDR_MODE\n");
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

static int m10mo_set_shot_mode(struct v4l2_subdev *sd, unsigned int val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int shot_mode_support = M10MO_SHOT_MODES_SUPPORTED(dev->fw_type);

	if (!shot_mode_support)
		return -EINVAL;

	switch (val) {
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
	default:
		return -EINVAL;
	}

	return 0;
}

static int m10mo_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	mutex_lock(&dev->input_lock);
	ret = __m10mo_s_power(sd, on, false);
	mutex_unlock(&dev->input_lock);

	return ret;
}

int m10mo_single_capture_process(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret = 0;
	u32 fmt;
    printk("@%s %d start\n", __func__, __LINE__);

	/* Select frame */
	/*ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL,*/
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

	/*ret = m10mo_writeb(sd, CATEGORY_CAPTURE_PARAM, CAPP_YUVOUT_MAIN, fmt);*/
	/*if (ret)*/
		/*return ret;*/

	/* Image size */
	/*ret = m10mo_writeb(sd, CATEGORY_CAPTURE_PARAM, CAPP_MAIN_IMAGE_SIZE,*/
			   /*dev->curr_res_table[dev->fmt_idx].command);*/
	/*if (ret)*/
		/*return ret;*/

	/* Start image transfer */
	/*ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL,*/
			  /*CAPC_TRANSFER_START, 0x01);*/

	return ret;
}
#ifndef NOIRQ
static irqreturn_t m10mo_irq_thread(int irq, void *dev_id)
{
	struct v4l2_subdev *sd = (struct v4l2_subdev *)dev_id;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	u32 int_factor;
    printk("%s %d get IRQ, requested_cmd = %d\n", __func__, __LINE__, dev->requested_cmd);

	/* Clear interrupt by reading interrupt factor register */
	(void) m10mo_read(sd, 1, CATEGORY_SYSTEM, SYSTEM_INT_FACTOR,
			  &int_factor);

	dev_info(&client->dev, "INT_FACTOR: 0x%x\n", int_factor);

	switch (dev->requested_cmd) {
	case M10MO_CAMERA_START:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M10MO_CAMERA_START;
		}
		break;
	case M10MO_PARAMETER_MODE_REQUEST_CMD:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M10MO_PARAMETER_MODE_REQUEST_CMD;
		}
		break;
	case M10MO_MONITOR_MODE:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M10MO_MONITOR_MODE;
		}
		break;
	case M10MO_MONITOR_MODE_ZSL_REQUEST_CMD:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M10MO_MONITOR_MODE_ZSL_REQUEST_CMD;
		}
		break;
	case M10MO_SINGLE_CAPTURE_MODE:
		if (int_factor & REG_INT_STATUS_CAPTURE) {
			dev->cmd = M10MO_SINGLE_CAPTURE_MODE;
            openIntelISP = true;
			dev->fw_ops->single_capture_process(sd);
			gpio_set_value(60, 1);
			printk(KERN_INFO "m10mo driver, gpio 60 set high\n");
		}
		break;
	case M10MO_BURST_CAPTURE_MODE:
		if (int_factor & REG_INT_STATUS_MODE){
			dev->cmd = M10MO_BURST_CAPTURE_MODE;
        }
		break;
	case M10MO_MONITOR_MODE_HIGH_SPEED:
		if (int_factor & REG_INT_STATUS_MODE) {
			dev->cmd = M10MO_MONITOR_MODE_HIGH_SPEED;
		}
		break;
    case M10MO_HOME_SEARCHING_MODE:
        printk("%s %d, home search finished\n", __func__, __LINE__);
        if (int_factor & 0X06) {
            dev->cmd = M10MO_HOME_SEARCHING_MODE;
        }
        break;
    case M10MO_ZOOM_LENS_TO_PR:
        printk("%s %d, set zoom lens to PR finished\n", __func__, __LINE__);
        if (int_factor & 0X02) {
            dev->cmd = M10MO_ZOOM_LENS_TO_PR;
        }
        break;
    case M10MO_FOCUS_LENS_TO_PR:
        printk("%s %d, set focus lens to PR finished\n", __func__, __LINE__);
        if (int_factor & 0X02) {
            dev->cmd = M10MO_FOCUS_LENS_TO_PR;
        }
        break;
    case M10MO_START_OPTICAL_ZOOM:
        printk("%s %d, 0 optical ZOOM mode start finished\n", __func__, __LINE__);
        if (int_factor & 0x2) {
            printk("%s %d, 1 optical ZOOM mode start finished\n", __func__, __LINE__);
            dev->cmd = M10MO_START_OPTICAL_ZOOM;
        }
        break;
    	case M10MO_START_AF:
		AFDone = true;
        printk("%s %d, AF start finished\n", __func__, __LINE__);
        if (int_factor & 0x2) {
            dev->cmd = M10MO_START_AF;
        }
        break;
	case M10MO_START_DIGITAL_ZOOM:
        printk("%s %d, 0 digital ZOOM mode start finished\n", __func__, __LINE__);
        if (int_factor & 0x4) {
            printk("%s %d,  1 digital ZOOM mode start finished\n", __func__, __LINE__);
            dev->cmd = M10MO_START_DIGITAL_ZOOM;
        }
        break;
	default:
		dev_err(&client->dev, "m10mo_irq_thread :default case\n");
		return IRQ_HANDLED;
	}

	if (dev->requested_cmd == dev->cmd){
		dev->requested_cmd = M10MO_NO_CMD_REQUEST;
    }
pr_info("m10mo, 1111111111\n");
	wake_up_interruptible(&dev->irq_waitq);
pr_info("m10mo, 2222222222\n");
	return IRQ_HANDLED;
}

static int m10mo_setup_irq(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m10mo_device *dev = to_m10mo_sensor(sd);
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

	ret = request_threaded_irq(client->irq, NULL, m10mo_irq_thread,
			  IRQF_TRIGGER_RISING | IRQF_ONESHOT, M10MO_NAME, sd);
//	ret = request_threaded_irq(client->irq, NULL, m10mo_irq_thread,
//			  IRQF_TRIGGER_RISING, M10MO_NAME, sd);
	if (ret < 0) {
		dev_err(&client->dev, "Cannot register IRQ: %d\n", ret);
		goto out;
	}
	return 0;

out:
	return ret;
}
#endif
int get_resolution_index(const struct m10mo_resolution *res,
			int entries, u32 w, u32 h)
{
	int i;

	for (i = 0; i < entries; i++) {
		if (w != res[i].width)
			continue;
		if (h != res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

int __m10mo_try_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt, bool update_fmt)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct atomisp_input_stream_info *stream_info =
			(struct atomisp_input_stream_info *)fmt->reserved;
	const struct m10mo_resolution *res;
	u32 read_val = 0;
	int entries, idx;
	int mode = M10MO_GET_RESOLUTION_MODE(dev->fw_type);

    printk("@%s %d, format code = 0x%x\n", __func__, __LINE__, fmt->code);
//=== FIXME: Why HAL set so so many different format to driver?
//=== M10MO - temporarily hard code here. Start. ======//
	if (fmt->code != V4L2_MBUS_FMT_JPEG_1X8 &&
	    fmt->code != V4L2_MBUS_FMT_UYVY8_1X16 &&
		fmt->code != V4L2_MBUS_FMT_SGBRG10_1X10 &&
	    fmt->code != V4L2_MBUS_FMT_CUSTOM_NV12 &&
	    fmt->code != V4L2_MBUS_FMT_CUSTOM_NV21 &&
	    fmt->code != V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {
		printk("%s unsupported code: 0x%x. Set to NV12\n", __func__, fmt->code);
		fmt->code = V4L2_MBUS_FMT_CUSTOM_NV12;
	}
    if(fmt->code == V4L2_MBUS_FMT_SGBRG10_1X10){
	    (void) m10mo_readb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, &read_val);
		if(read_val != RAW_CAP){
	        (void)__m10mo_param_mode_set(sd);
	        (void) m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, RAW_CAP);
            (void) m10mo_request_cmd_effect(sd, M10MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL); // Enter into m10mo's Monitor mode.
		    dev->capture_mode = M10MO_CAPTURE_MODE_ZSL_RAW;
		}
	}else{
        fmt->code = V4L2_MBUS_FMT_UYVY8_1X16;
    }
//=== M10MO - temporarily hard code here. End. ======//
	/* In ZSL case, capture table needs to be handled separately */
    printk("@%s %d, mode = %d, stream = %d, run_mode = 0x%x, fmt->code is 0x%x\n", __func__, __LINE__, mode, stream_info->stream, dev->run_mode, fmt->code);

    if (stream_info->stream == ATOMISP_INPUT_STREAM_CAPTURE &&
			(dev->run_mode == CI_MODE_PREVIEW ||
			 dev->run_mode == CI_MODE_VIDEO ||
			 dev->run_mode == CI_MODE_CONTINUOUS)) {
		res = resolutions[M10MO_MODE_CAPTURE_INDEX];
		entries =
		     resolutions_sizes[M10MO_MODE_CAPTURE_INDEX];
	} else {
		res = dev->curr_res_table;
		entries = dev->entries_curr_table;
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

static int m10mo_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int idx;

	mutex_lock(&dev->input_lock);
	idx = dev->fw_ops->try_mbus_fmt(sd, fmt, true);
	mutex_unlock(&dev->input_lock);
	return idx >= 0 ? 0 : -EINVAL;
}

static int m10mo_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	mutex_lock(&dev->input_lock);

	fmt->width = dev->curr_res_table[dev->fmt_idx].width;
	fmt->height = dev->curr_res_table[dev->fmt_idx].height;
	fmt->code = dev->format.code;

	mutex_unlock(&dev->input_lock);

	return 0;
}

int __m10mo_update_stream_info(struct v4l2_subdev *sd,
			       struct v4l2_mbus_framefmt *fmt)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct atomisp_input_stream_info *stream_info =
			(struct atomisp_input_stream_info*)fmt->reserved;
	int mode = M10MO_GET_RESOLUTION_MODE(dev->fw_type);

	/* TODO: Define a FW Type as well. Resolution could be reused */
	switch (mode) {
	case M10MO_RESOLUTION_MODE_0:
		/* TODO: handle FW type cases here */
		break;
	case M10MO_RESOLUTION_MODE_1:
		/* Raw Capture is a special case. Capture data comes like HDR */
		if (fmt->code == V4L2_MBUS_FMT_JPEG_1X8 ||
			fmt->code == V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {
			/* fill stream info */
			stream_info->ch_id = M10MO_ZSL_JPEG_VIRTUAL_CHANNEL;
			stream_info->isys_configs = 1;
			stream_info->isys_info[0].input_format =
				(u8)ATOMISP_INPUT_FORMAT_USER_DEF3;
			stream_info->isys_info[0].width = 0;
			stream_info->isys_info[0].height = 0;
		} else if (fmt->code == V4L2_MBUS_FMT_CUSTOM_NV12 ||
			   fmt->code == V4L2_MBUS_FMT_CUSTOM_NV21) {
			stream_info->ch_id = M10MO_ZSL_NV12_VIRTUAL_CHANNEL;
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
		break;
	}
	return 0;
}

int __m10mo_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct atomisp_input_stream_info *stream_info =
			(struct atomisp_input_stream_info *)fmt->reserved;
	int mode = M10MO_GET_RESOLUTION_MODE(dev->fw_type);
	int index;
    printk(KERN_INFO "@%s, start! \n", __func__);
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
	if (dev->format.code == V4L2_MBUS_FMT_JPEG_1X8) {
		/* The m10mo can only run JPEG in 30fps or lower */
		dev->fps = M10MO_NORMAL_FPS;
		fmt->width = dev->mipi_params.jpeg_width;
		fmt->height = dev->mipi_params.jpeg_height;
	} else if (dev->format.code == V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {
		fmt->width = dev->mipi_params.raw_width;
		fmt->height = dev->mipi_params.raw_height;
	}

	/* Update the stream info. Atomisp uses this for configuring mipi */
	__m10mo_update_stream_info(sd, fmt);

	/*
	 * Handle raw capture mode separately. Update the capture mode to RAW
	 * capture now. So that the next streamon call will start RAW capture.
	 */
	if (mode == M10MO_RESOLUTION_MODE_1 &&
	    dev->format.code == V4L2_MBUS_FMT_CUSTOM_M10MO_RAW) {
		dev_dbg(&client->dev, "%s RAW capture mode\n", __func__);
		dev->capture_mode = M10MO_CAPTURE_MODE_ZSL_RAW;
		dev->capture_res_idx = dev->fmt_idx;
		dev->fmt_idx = 0;
	}

    if (PREVIEW_TEST_PATTERN) {
        if (fmt->width == 1440 && fmt->height == 1080) {
            m10mo_writeb(sd, 0x01, 0x01, 0x37); // 1440 x 1080
        } else if (fmt->width == 640 && fmt->height == 480) {
            m10mo_writeb(sd, 0x01, 0x01, 0x17);
        }
    }

	mutex_unlock(&dev->input_lock);
	return 0;
}

static int m10mo_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	return dev->fw_ops->set_mbus_fmt(sd, fmt);
}

static int m10mo_set_run_mode(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret = 0;

    printk("@%s %d start, capture_mode = %d, run_mode = 0x%x\n", __func__, __LINE__, dev->capture_mode, dev->run_mode);
#if 0
	/*
	 * Handle RAW capture mode separately irrespective of the run mode
	 * being configured. Start the RAW capture right away.
	 */
	if (dev->capture_mode == M10MO_CAPTURE_MODE_ZSL_RAW) {
		/*
		 * As RAW capture is done from a command line tool, we are not
		 * restarting the preview after the RAW capture. So it is ok
		 * to reset the RAW capture mode here because the next RAW
		 * capture has to start from the Set format onwards.
		 */
		dev->capture_mode = M10MO_CAPTURE_MODE_ZSL_NORMAL;
		return m10mo_set_zsl_raw_capture(sd);
	}
#endif
	switch (dev->run_mode) {
	case CI_MODE_STILL_CAPTURE:
#ifndef off_c51
        ret = m10mo_set_still_capture(sd);
#endif
		break;
    case CI_MODE_PREVIEW:
            if(!isCaptureMode){
                printk("@%s %d, preview set monitor mode\n", __func__, __LINE__);
                ret = m10mo_set_zsl_monitor(sd);
            }
            else{
                printk("@%s %d, after capture do not set monitor mode\n", __func__, __LINE__);
				if(dev->capture_mode == M10MO_CAPTURE_MODE_ZSL_RAW){
				    //=== Send command to stop Raw-Capture  ===//
                    ret = m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, 0x05, STOP_RAW_CAP);
				}
//                ret = 0;
            }
        isCaptureMode = false;
		break;
	default:
		/* TODO: Revisit this logic on switching to panorama */
		if (dev->curr_res_table[dev->fmt_idx].command == 0x43)
			ret = m10mo_set_panorama_monitor(sd);
		else{
            if(!isCaptureMode){
                printk("@%s %d, preview set monitor mode\n", __func__, __LINE__);
                ret = m10mo_set_zsl_monitor(sd);
            }
            else{
                printk("@%s %d, after capture do not set monitor mode\n", __func__, __LINE__);
                ret = 0;
            }
        }
        isCaptureMode = false;
	}
	return ret;
}

static void m10mo_check_app_cap_mode(struct v4l2_subdev *sd)
{
    struct m10mo_device *dev = to_m10mo_sensor(sd);
    printk("@%s %d start, capture_mode = %d, run_mode = 0x%x\n", __func__, __LINE__, dev->capture_mode, dev->run_mode);

	switch(dev->capture_mode){
	case APP_HDR_CAP_MODE_ZSL:
	    printk(KERN_INFO "m10mo, APP HDR capture mode.\n");
	    (void) m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, HDR_CAP);
	    break;
	case APP_LLS_CAP_MODE_ZSL:
	    printk(KERN_INFO "m10mo, APP LLS capture mode.\n");
		(void) m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, LLS_CAP);
		break;
	case M10MO_CAPTURE_MODE_ZSL_NORMAL:
	    printk(KERN_INFO "m10mo, APP Auto capture mode.\n");
		break;
	case M10MO_CAPTURE_MODE_ZSL_RAW:
//	    (void)__m10mo_param_mode_set(sd);
//	    (void) m10mo_writeb(sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, RAW_CAP);
//		(void) m10mo_set_run_mode(sd);
		break;
	default:
	    printk(KERN_INFO "m10mo, erro APP capture mode %d \n", dev->capture_mode);
		break;
	}
}

static int m10mo_identify_fw_type(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct m10mo_fw_id *fw_ids = NULL;
	char buffer[M10MO_MAX_FW_ID_STRING];
	int ret;

	int m10mo_fw_address_cnt = m10mo_get_fw_address_count();
	int i;

	ret = dev->pdata->identify_fw();
	if (ret != -1) {
		dev->fw_type = ret;
		return 0;
	}

	for (i = 0; i < m10mo_fw_address_cnt; i++) {
		fw_ids = dev->pdata->fw_ids;
		if (!fw_ids)
			return 0;

		ret = m10mo_get_isp_fw_version_string(dev, buffer,
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

static void m10mo_mipi_initialization(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int id = M10MO_GET_MIPI_PACKET_SIZE_IDX(dev->fw_type);
	u32 mipi_packet_size = dev->pdata->mipi_packet_size[id];

	dev->mipi_params.jpeg_width = mipi_packet_size;
	dev->mipi_params.jpeg_height = M10MO_MAX_YUV422_SIZE / mipi_packet_size;

	dev->mipi_params.raw_width = mipi_packet_size;
	dev->mipi_params.raw_height = M10MO_MAX_RAW_SIZE / mipi_packet_size;

	dev_dbg(&client->dev,
		"%s JPEG: W=%d H=%d, RAW: W=%d H=%d\n", __func__,
		dev->mipi_params.jpeg_width, dev->mipi_params.jpeg_height,
		dev->mipi_params.raw_width, dev->mipi_params.raw_height);
}

int m10mo_set_still_capture(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u8 res_cmd;
    openIntelISP = false;
    isCaptureMode = true;
	res_cmd = dev->curr_res_table[dev->fmt_idx].command;
	printk("@%s %d start\n", __func__, __LINE__);

	dev_info(&client->dev, "%s mode: %d width: %d, height: %d, cmd: 0x%x\n",
		__func__, dev->cmd, dev->curr_res_table[dev->fmt_idx].width,
		dev->curr_res_table[dev->fmt_idx].height,
		dev->curr_res_table[dev->fmt_idx].command);

    //=== Set capture resolution command ===//
	ret = m10mo_writeb(sd, CATEGORY_CAPTURE_PARAM, PARAM_CAP_SIZE, res_cmd);

	//=== Check "application capture mode" and send command to m10mo if needed. ===//
    m10mo_check_app_cap_mode(sd);

	//=== Send command to capture  ===//
    ret = m10mo_request_cmd_effect(sd, M10MO_SINGLE_CAPTURE_MODE, NULL);

	return ret;
}

int m10mo_streamoff(struct v4l2_subdev *sd)
{
	pr_info("%s, m10mo_capture_pre_flag is %d", __func__, m10mo_capture_pre_flag);

    switch(m10mo_capture_pre_flag){
    case M10MO_NOT_CAPTURE_0:
	    return __m10mo_param_mode_set(sd);

	case M10MO_CAP_BEFORE_1ST_STREAMOFF_1:
	    pr_info("m10mo, 1st fake stream off during capture\n");
		m10mo_capture_pre_flag = M10MO_CAP_BETWEEN_1ST_AND_2ND_STREAMOFF_2;
		return 0;

    case M10MO_CAP_BETWEEN_1ST_AND_2ND_STREAMOFF_2:
	    pr_info("m10mo, 2nd fake stream off during capture\n");
		m10mo_capture_pre_flag = M10MO_NOT_CAPTURE_0;
		return 0;

	default:
        return -EIO;
	}

}

int m10mo_test_pattern(struct v4l2_subdev *sd, u8 val)
{

	struct m10mo_device *dev = to_m10mo_sensor(sd);
	pr_info("%s, run mode: %x\n", __func__, dev->run_mode);

	m10mo_writeb(sd, CATEGORY_TEST, 0xEE, val);

	return 0;
}

static const struct m10mo_fw_ops fw_ops = {
	.set_run_mode           = m10mo_set_run_mode,
	.set_burst_mode         = m10mo_set_burst_mode,
	.stream_off             = m10mo_streamoff,
	.single_capture_process = m10mo_single_capture_process,
	.try_mbus_fmt           =  __m10mo_try_mbus_fmt,
	.set_mbus_fmt           =  __m10mo_set_mbus_fmt,
	.test_pattern           = m10mo_test_pattern,
};

void m10mo_handlers_init(struct v4l2_subdev *sd)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	pr_info("%s, dev->fw_type: %d\n", __func__, dev->fw_type);

	switch (dev->fw_type) {
	case M10MO_FW_TYPE_1:
		dev->fw_ops = &fw_type1_5_ops;
		break;
	case M10MO_FW_TYPE_2:
		dev->fw_ops = &fw_type2_ops;
		break;
	case M10MO_FW_TYPE_5:
		dev->fw_ops = &fw_type1_5_ops;
		break;
	default:
		dev->fw_ops = &fw_ops;
	}
}

static int m10mo_s_config(struct v4l2_subdev *sd, int irq)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 result = M10MO_INVALID_CHECKSUM;
	int id = 0;

	pr_info("%s\n", __func__);

	mutex_lock(&dev->input_lock);

	init_waitqueue_head(&dev->irq_waitq);

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
	ret = m10mo_setup_irq(sd);
	if (ret) {
		dev_err(&client->dev, "IRQ err.\n");
		mutex_unlock(&dev->input_lock);
		return ret;
	}
#else
	wake_up_interruptible(&dev->irq_waitq);
#endif
	ret = __m10mo_s_power(sd, 1, true);
	if (ret) {
		dev_err(&client->dev, "power-up err.\n");
		goto free_irq;
	}

	if (dev->pdata->common.csi_cfg) {
		ret = dev->pdata->common.csi_cfg(sd, 1);
		if (ret)
			goto fail;
	}

	ret = m10mo_fw_checksum(dev, &result);
	if (ret) {
		dev_err(&client->dev, "Checksum calculation fails.\n");
		goto fail;
	}
	if (result != M10MO_VALID_CHECKSUM){
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is not 0.\n");
		dev_err(&client->dev, "Firmware checksum is %x.\n",result);
	}
		/* TBD: Trig FW update here */

	/*
	 * We don't care about the return value here. Even in case of
	 * wrong or non-existent fw this phase must pass.
	 * FW can be updated later.
	 */
	m10mo_identify_fw_type(sd);

	/*
	 * Only after identify_fw_type the correct dev->fw_type
	 * can be got, so here update the ref_clock
	 */
	id = M10MO_GET_CLOCK_RATE_MODE(dev->fw_type);
	dev->ref_clock = 19200000;//dev->pdata->ref_clock_rate[id];

	m10mo_mipi_initialization(sd);

	/* Set proper function pointers based on FW_TYPE */
	m10mo_handlers_init(sd);

	ret = __m10mo_s_power(sd, 0, true);
	if (ret) {
		dev_err(&client->dev, "power-down err.\n");
		goto free_irq;
	}

	mutex_unlock(&dev->input_lock);
	return 0;

fail:
	__m10mo_s_power(sd, 0, true);
free_irq:
#ifndef NOIRQ
	free_irq(client->irq, sd);
#endif
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "External ISP power-gating failed\n");
	return ret;

}

static int m10mo_recovery(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	/* still power off sensor in case power is cut off abnormally*/
	ret = __m10mo_s_power(sd, 0, false);
	if (ret) {
		dev_err(&client->dev, "power-down err.\n");
		return ret;
	}
	usleep_range(100, 200);

	ret = __m10mo_s_power(sd, 1, false);
	if (ret) {
		dev_err(&client->dev, "power-up err.\n");
		return ret;
	}

	return ret;
}

static int m10mo_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret = 0;

	pr_info("%s, enable: %d\n", __func__, enable);

	mutex_lock(&dev->input_lock);
	if (enable) {

		if (!PREVIEW_TEST_PATTERN)
			ret = dev->fw_ops->set_run_mode(sd);
		else
			m10mo_test_pattern(sd, 1); // enable test mode.

		if (ret) {
			ret = m10mo_recovery(sd);
			if (ret) {
				mutex_unlock(&dev->input_lock);
				return ret;
			}

			if (!PREVIEW_TEST_PATTERN)
				ret = dev->fw_ops->set_run_mode(sd);
			else
				m10mo_test_pattern(sd, 1); // enable test mode.
 		}
	} else {

		if (!PREVIEW_TEST_PATTERN)
			ret = dev->fw_ops->stream_off(sd);
		else
			m10mo_test_pattern(sd, 0); // enable test mode.
 	}
	mutex_unlock(&dev->input_lock);

	/*
	 * Dump M10MO log when stream-off with checking folder
	 */
	if (!enable)
		m10mo_dump_log(sd);

	return ret;
}

static int
m10mo_enum_framesizes(struct v4l2_subdev *sd, struct v4l2_frmsizeenum *fsize)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
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
m10mo_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int index = fse->index;
	printk("%s, start, m10mo_capture_pre_flag is %d \n", __func__, m10mo_capture_pre_flag);

	mutex_lock(&dev->input_lock);
	if(m10mo_capture_pre_flag != M10MO_NOT_CAPTURE_0){
	    dev->curr_res_table = resolutions[M10MO_MODE_CAPTURE_INDEX];
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

static int m10mo_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	if (code->index)
		return -EINVAL;

	code->code = dev->format.code;
	return 0;
}

static struct v4l2_mbus_framefmt *
__m10mo_get_pad_format(struct m10mo_device *sensor,
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
m10mo_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct m10mo_device *snr = to_m10mo_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__m10mo_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	mutex_lock(&snr->input_lock);
	fmt->format = *format;
	mutex_unlock(&snr->input_lock);

	return 0;
}

static int
m10mo_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct m10mo_device *snr = to_m10mo_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__m10mo_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	mutex_lock(&snr->input_lock);
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;
	mutex_unlock(&snr->input_lock);

	return 0;
}

static const struct media_entity_operations m10mo_entity_ops = {
	.link_setup = NULL,
};

static int m10mo_set_flicker_freq(struct v4l2_subdev *sd, s32 val)
{
	unsigned int flicker_freq;
//////
return 0;
/////
	switch (val) {
	case V4L2_CID_POWER_LINE_FREQUENCY_DISABLED:
		flicker_freq = M10MO_FLICKER_OFF;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_50HZ:
		flicker_freq = M10MO_FLICKER_50HZ;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_60HZ:
		flicker_freq = M10MO_FLICKER_60HZ;
		break;
	case V4L2_CID_POWER_LINE_FREQUENCY_AUTO:
		flicker_freq = M10MO_FLICKER_AUTO;
		break;
	default:
		return -EINVAL;
	}

	return m10mo_writeb(sd, CATEGORY_AE, AE_FLICKER, flicker_freq);
}

static int m10mo_set_metering(struct v4l2_subdev *sd, s32 val)
{
	unsigned int metering;

	switch (val) {
	case V4L2_EXPOSURE_METERING_CENTER_WEIGHTED:
		metering = M10MO_METERING_CENTER;
		break;
	case V4L2_EXPOSURE_METERING_SPOT:
		metering = M10MO_METERING_SPOT;
		break;
	case V4L2_EXPOSURE_METERING_AVERAGE:
		metering = M10MO_METERING_AVERAGE;
		break;
	default:
		return -EINVAL;
	}

	return m10mo_writeb(sd, CATEGORY_AE, AE_MODE, metering);
}

static const unsigned short wb_lut[][2] = {
	{ V4L2_WHITE_BALANCE_INCANDESCENT,  REG_AWB_INCANDESCENT },
	{ V4L2_WHITE_BALANCE_FLUORESCENT,   REG_AWB_FLUORESCENT_L },
	{ V4L2_WHITE_BALANCE_FLUORESCENT_H, REG_AWB_FLUORESCENT_H },
	{ V4L2_WHITE_BALANCE_HORIZON,       REG_AWB_HORIZON },
	{ V4L2_WHITE_BALANCE_DAYLIGHT,      REG_AWB_DAYLIGHT },
	{ V4L2_WHITE_BALANCE_FLASH,         REG_AWB_LEDLIGHT },
	{ V4L2_WHITE_BALANCE_CLOUDY,        REG_AWB_CLOUDY },
	{ V4L2_WHITE_BALANCE_SHADE,         REG_AWB_SHADE },
	{ V4L2_WHITE_BALANCE_AUTO,          REG_AWB_AUTO },
};

static int m10mo_set_white_balance(struct v4l2_subdev *sd, s32 val)
{
	int i, ret;
	int awb = REG_AWB_MANUAL;

	for (i = 0; i < ARRAY_SIZE(wb_lut); i++) {
		if (val == wb_lut[i][0])
			break;
	}

	if (i == ARRAY_SIZE(wb_lut))
		return -EINVAL;

	if (wb_lut[i][0] == V4L2_WHITE_BALANCE_AUTO)
		awb = REG_AWB_AUTO;

	ret = m10mo_writeb(sd, CATEGORY_WB,
			   AWB_MODE, awb);
	if (ret < 0)
		return ret;

	if (awb == REG_AWB_MANUAL)
		ret = m10mo_writeb(sd, CATEGORY_WB,
				   AWB_PRESET, wb_lut[i][1]);
	return ret;
}

static int m10mo_set_ev_bias(struct v4l2_subdev *sd, s32 val)
{
	/* 0x04 refers to 0.0EV value in m10mo HW */
	/* val refers to EV units, where the */
	/* value 1000 stands for +1EV */

	int ev_bias = 0x04 + (val/M10MO_EV_STEP);
	return m10mo_writeb(sd, CATEGORY_AE, AE_EV_BIAS, ev_bias);
}

static int m10mo_get_ev_bias(struct v4l2_subdev *sd, s32 *val)
{
	int ret;
	u32 ev_bias;

	ret = m10mo_readb(sd, CATEGORY_AE, AE_EV_BIAS, &ev_bias);
	if (ret)
		return ret;

	*val = (ev_bias-4) * M10MO_EV_STEP;
	return 0;
}


static const unsigned short iso_lut[][2] = {
	{ 100,  REG_AE_ISOMODE_ISO100},
	{ 200,  REG_AE_ISOMODE_ISO200},
	{ 400,  REG_AE_ISOMODE_ISO400},
	{ 800,  REG_AE_ISOMODE_ISO800},
	{ 1600, REG_AE_ISOMODE_ISO1600},
};

static int m10mo_set_iso_sensitivity(struct v4l2_subdev *sd, s32 val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret, i;

	for (i = 0; i < ARRAY_SIZE(iso_lut); i++) {
		if (val == iso_lut[i][0])
			break;
	}

	if (i == ARRAY_SIZE(iso_lut))
		return -EINVAL;

	if (dev->iso_mode == V4L2_ISO_SENSITIVITY_MANUAL) {
		ret = m10mo_writeb(sd, CATEGORY_AE,
				   AE_ISOMODE, iso_lut[i][1]);
		if (ret < 0)
			return ret;
	}
	dev->iso_sensitivity = iso_lut[i][1];

	return 0;
}

static int m10mo_set_iso_mode(struct v4l2_subdev *sd, s32 val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret;

	if (val == V4L2_ISO_SENSITIVITY_AUTO) {
		ret = m10mo_writeb(sd, CATEGORY_AE,
				   AE_ISOMODE, REG_AE_ISOMODE_AUTO);
		if (ret < 0)
			return ret;
		dev->iso_mode = V4L2_ISO_SENSITIVITY_AUTO;
	} else {
		ret = m10mo_writeb(sd, CATEGORY_AE,
				   AE_ISOMODE, dev->iso_sensitivity);
		if (ret < 0)
			return ret;
		dev->iso_mode = V4L2_ISO_SENSITIVITY_MANUAL;
	}

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

static int m10mo_set_cb_cr(struct v4l2_subdev *sd, u8 cfixb, u8 cfixr)
{
	int ret;

	ret = m10mo_writeb(sd, CATEGORY_MONITOR,
			   MONITOR_CFIXB, cfixb);
	if (ret)
		return ret;

	ret = m10mo_writeb(sd, CATEGORY_MONITOR,
			   MONITOR_CFIXR, cfixr);
	if (ret)
		return ret;

	ret = m10mo_writeb(sd, CATEGORY_MONITOR,
			   MONITOR_COLOR_EFFECT, COLOR_EFFECT_ON);
	return ret;
}

static int m10mo_set_color_effect(struct v4l2_subdev *sd, s32 val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	int i, ret;

	switch (val) {
	case V4L2_COLORFX_NONE:
	case V4L2_COLORFX_NEGATIVE:
	case V4L2_COLORFX_WARM:
	case V4L2_COLORFX_COLD:
	case V4L2_COLORFX_WASHED:
		for (i = 0; i < ARRAY_SIZE(ce_lut); i++) {
			if (val == ce_lut[i][0])
				break;
		}

		if (i == ARRAY_SIZE(ce_lut))
			return -EINVAL;

		ret = m10mo_writeb(sd, CATEGORY_MONITOR,
				   MONITOR_COLOR_EFFECT, ce_lut[i][1]);
		break;
	case V4L2_COLORFX_SEPIA:
	case V4L2_COLORFX_BW:
	case V4L2_COLORFX_ANTIQUE:
	case V4L2_COLORFX_RED:
	case V4L2_COLORFX_GREEN:
	case V4L2_COLORFX_BLUE:
	case V4L2_COLORFX_PINK:
	case V4L2_COLORFX_YELLOW:
	case V4L2_COLORFX_PURPLE:
		for (i = 0; i < ARRAY_SIZE(cbcr_lut); i++) {
			if (val == cbcr_lut[i][0])
				break;
		}

		if (i == ARRAY_SIZE(cbcr_lut))
			return -EINVAL;

		ret = m10mo_set_cb_cr(sd, cbcr_lut[i][1], cbcr_lut[i][2]);
		break;
	case V4L2_COLORFX_SET_CBCR:
		ret = m10mo_set_cb_cr(sd, dev->colorfx_cb, dev->colorfx_cr);
		break;
	default:
		ret = -EINVAL;
	};

	return ret;
}

static int m10mo_set_color_effect_cbcr(struct v4l2_subdev *sd, s32 val)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int ret;
	u8 cr, cb;

	cr = val & 0xff;
	cb = (val >> 8) & 0xff;

	if (dev->colorfx->cur.val == V4L2_COLORFX_SET_CBCR) {
		ret = m10mo_set_cb_cr(sd, cb, cr);
		if (ret)
			return ret;
	}

	dev->colorfx_cr = cr;
	dev->colorfx_cb = cb;

	return 0;
}

static int m10mo_get_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (M10MO_FOCAL_LENGTH_NUM << 16) | M10MO_FOCAL_LENGTH_DEM;
	return 0;
}

static int m10mo_get_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	*val = (M10MO_F_NUMBER_DEFAULT_NUM << 16) | M10MO_F_NUMBER_DEM;
	return 0;
}

static int m10mo_get_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (M10MO_F_NUMBER_DEFAULT_NUM << 24) |
		(M10MO_F_NUMBER_DEM << 16) |
		(M10MO_F_NUMBER_DEFAULT_NUM << 8) | M10MO_F_NUMBER_DEM;
	return 0;
}

static int m10mo_s_ctrl(struct v4l2_ctrl *ctrl)
{
	struct m10mo_device *dev = container_of(
		ctrl->handler, struct m10mo_device, ctrl_handler);
	int ret = 0;

	if (!dev->power)
		return 0;

	switch (ctrl->id) {
	case V4L2_CID_POWER_LINE_FREQUENCY:
		ret = m10mo_set_flicker_freq(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE_METERING:
		ret = m10mo_set_metering(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_START_ZSL_CAPTURE:
		if (ctrl->val)
			ret = m10mo_set_zsl_capture(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE:
		ret = m10mo_set_white_balance(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_EXPOSURE:
		ret = m10mo_set_ev_bias(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_ISO_SENSITIVITY:
		ret = m10mo_set_iso_sensitivity(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_ISO_SENSITIVITY_AUTO:
		ret = m10mo_set_iso_mode(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_COLORFX:
		ret = m10mo_set_color_effect(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_COLORFX_CBCR:
		ret = m10mo_set_color_effect_cbcr(&dev->sd, ctrl->val);
		break;
	case V4L2_CID_TEST_PATTERN:
		ret = dev->fw_ops->test_pattern(&dev->sd, ctrl->val);
		break;
	default:
		return -EINVAL;
	}

	return ret;
}

static const u32 m10mo_mipi_freq[] = {
	M10MO_MIPI_FREQ_0,
	M10MO_MIPI_FREQ_1,
};

static int m10mo_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct m10mo_device *dev = container_of(
		ctrl->handler, struct m10mo_device, ctrl_handler);
	int ret = 0;
	int id = M10MO_GET_MIPI_FREQ_MODE(dev->fw_type);

	switch (ctrl->id) {
	case V4L2_CID_LINK_FREQ:
		ctrl->val = m10mo_mipi_freq[id];
		break;
	case V4L2_CID_EXPOSURE:
		ret = m10mo_get_ev_bias(&dev->sd, &ctrl->val);
		break;
	case V4L2_CID_FOCAL_ABSOLUTE:
		ret = m10mo_get_focal(&dev->sd, &ctrl->val);
		break;
	case V4L2_CID_FNUMBER_ABSOLUTE:
		ret = m10mo_get_fnumber(&dev->sd, &ctrl->val);
		break;
	case V4L2_CID_FNUMBER_RANGE:
		ret = m10mo_get_fnumber_range(&dev->sd, &ctrl->val);
		break;
	default:
		return -EINVAL;
	}
	return ret;
}

static struct v4l2_ctrl_ops m10mo_ctrl_ops = {
	.g_volatile_ctrl = m10mo_g_volatile_ctrl,
	.s_ctrl = m10mo_s_ctrl,
};

/* TODO: To move this to s_ctrl framework */
static int m10mo_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	int index;
	int mode = M10MO_GET_RESOLUTION_MODE(dev->fw_type);

    printk("@%s %d start, mode = %d, run_mode = 0x%x\n", __func__, __LINE__, mode, param->parm.capture.capturemode);
	mutex_lock(&dev->input_lock);
	dev->run_mode = param->parm.capture.capturemode;
	switch (dev->run_mode) {
	case CI_MODE_STILL_CAPTURE:
		index = M10MO_MODE_CAPTURE_INDEX;
		break;
	default:
		index = M10MO_MODE_PREVIEW_INDEX;
		break;
	}
	dev->entries_curr_table = resolutions_sizes[index];
	dev->curr_res_table = resolutions[index];

	mutex_unlock(&dev->input_lock);
	return 0;
}

static long m10mo_ioctl(struct v4l2_subdev *sd, unsigned int cmd,
			void *arg)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct atomisp_ext_isp_ctrl *m10mo_ctrl
		= (struct atomisp_ext_isp_ctrl *)arg;
	int ret = 0;

	dev_info(&client->dev, "m10mo ioctl id is %d, data 0x%x\n",
			m10mo_ctrl->id, m10mo_ctrl->data);

	mutex_lock(&dev->input_lock);
	switch(m10mo_ctrl->id)
	{
	case EXT_ISP_CID_ISO:
		dev_info(&client->dev, "m10mo ioctl ISO\n");
		break;
	case EXT_ISP_CID_CAPTURE_HDR:
		ret = m10mo_set_hdr_mode(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_CAPTURE_LLS:
		ret = m10mo_set_lls_mode(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_FOCUS_MODE:
		ret = m10mo_set_af_mode(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_FOCUS_EXECUTION:
		ret = m10mo_set_af_execution(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_TOUCH_POSX:
		ret = m10mo_set_af_position_x(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_TOUCH_POSY:
		ret = m10mo_set_af_position_y(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_TOUCH_WIDTH:
		ret = m10mo_set_af_width(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_TOUCH_HEIGHT:
		ret = m10mo_set_af_height(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_TOUCH_AE_POSX:
		ret = m10mo_set_ae_position_x(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_TOUCH_AE_POSY:
		ret = m10mo_set_ae_position_y(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_CAF_STATUS:
		ret = m10mo_get_caf_status(sd, &m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_AF_STATUS:
		ret = m10mo_get_af_status(sd, &m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_GET_AF_MODE:
		ret = m10mo_get_af_mode(sd, &m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_CAPTURE_BURST:
		ret = dev->fw_ops->set_burst_mode(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_FLASH_MODE:
		ret = m10mo_set_flash_mode(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_ZOOM:
		m10mo_ctrl->data = clamp_t(unsigned int, m10mo_ctrl->data,
						ZOOM_POS_MIN, ZOOM_POS_MAX);
		ret = m10mo_set_optical_zoom_position(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_CID_SHOT_MODE:
		ret = m10mo_set_shot_mode(sd, m10mo_ctrl->data);
		break;
	case EXT_ISP_M10MO_SET_CAPTURE_MODE:
	    m10mo_capture_pre_flag = M10MO_CAP_BEFORE_1ST_STREAMOFF_1;
		ret = 0;
		break;
	default:
		ret = -EINVAL;
		dev_err(&client->dev, "m10mo ioctl: Unsupported ID\n");
	};

	mutex_unlock(&dev->input_lock);
	return ret;
}

static const struct v4l2_ctrl_config ctrls[] = {
	{
		.ops = &m10mo_ctrl_ops,
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
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_POWER_LINE_FREQUENCY,
		.name = "Light frequency filter",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 3,
		.max = 3,
		.step = 1,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_EXPOSURE_METERING,
		.name = "Metering",
		.type = V4L2_CTRL_TYPE_MENU,
		.min = 0,
		.max = 2,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_START_ZSL_CAPTURE,
		.name = "Start zsl capture",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 1,
		.max = 4,
		.step = 1,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_AUTO_N_PRESET_WHITE_BALANCE,
		.name = "White Balance, Auto & Preset",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = V4L2_WHITE_BALANCE_MANUAL,
		.def = V4L2_WHITE_BALANCE_AUTO,
		.max = V4L2_WHITE_BALANCE_SHADE,
		.step = 1,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_EXPOSURE,
		.name = "Exposure Bias",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = M10MO_MIN_EV,
		.def = 0,
		.max = M10MO_MAX_EV,
		.step = M10MO_EV_STEP
	},
	{
		.id = V4L2_CID_ISO_SENSITIVITY,
		.name = "Iso",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 100,
		.def = 100,
		.max = 1600,
		.step = 100,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_ISO_SENSITIVITY_AUTO,
		.name = "Iso mode",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = V4L2_ISO_SENSITIVITY_MANUAL,
		.def = V4L2_ISO_SENSITIVITY_AUTO,
		.max = V4L2_ISO_SENSITIVITY_AUTO,
		.step = 1,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_COLORFX,
		.name = "Image Color Effect",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = V4L2_COLORFX_NONE,
		.def = V4L2_COLORFX_NONE,
		.max = V4L2_COLORFX_PURPLE,
		.step = 1,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_COLORFX_CBCR,
		.name = "Image Color Effect CbCr",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 0,
		.max = 0xffff,
		.step = 1,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_TEST_PATTERN,
		.name = "Test Pattern (Color Bar)",
		.type = V4L2_CTRL_TYPE_INTEGER,
		.min = 0,
		.def = 0,
		.max = 1,
		.step = 1
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_FOCAL_ABSOLUTE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "focal length",
		.min = M10MO_FOCAL_LENGTH_DEFAULT,
		.max = M10MO_FOCAL_LENGTH_DEFAULT,
		.step = 1,
		.def = M10MO_FOCAL_LENGTH_DEFAULT,
		.flags = 0,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_FNUMBER_ABSOLUTE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "f-number",
		.min = M10MO_F_NUMBER_DEFAULT,
		.max = M10MO_F_NUMBER_DEFAULT,
		.step = 1,
		.def = M10MO_F_NUMBER_DEFAULT,
		.flags = 0,
	},
	{
		.ops = &m10mo_ctrl_ops,
		.id = V4L2_CID_FNUMBER_RANGE,
		.type = V4L2_CTRL_TYPE_INTEGER,
		.name = "f-number range",
		.min = M10MO_F_NUMBER_RANGE,
		.max =  M10MO_F_NUMBER_RANGE,
		.step = 1,
		.def = M10MO_F_NUMBER_RANGE,
		.flags = 0,
	},
};

static int __m10mo_init_ctrl_handler(struct m10mo_device *dev)
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

static int m10mo_s_routing(struct v4l2_subdev *sd, u32 input, u32 output, u32 config)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
	struct atomisp_camera_caps *caps =
		dev->pdata->common.get_camera_caps();

	/* Select operating sensor. */
	if (caps->sensor_num > 1) {
		return m10mo_write(sd, 1, CATEGORY_SYSTEM,
			SYSTEM_MASTER_SENSOR, !output);
	}

	return 0;
}

static int m10mo_s_frame_interval(struct v4l2_subdev *sd,
		struct v4l2_subdev_frame_interval *interval)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
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

static const struct v4l2_subdev_video_ops m10mo_video_ops = {
	.try_mbus_fmt = m10mo_try_mbus_fmt,
	.s_mbus_fmt = m10mo_set_mbus_fmt,
	.g_mbus_fmt = m10mo_get_mbus_fmt,
	.s_stream = m10mo_s_stream,
	.s_parm = m10mo_s_parm,
	.enum_framesizes = m10mo_enum_framesizes,
	.s_routing = m10mo_s_routing,
	.s_frame_interval = m10mo_s_frame_interval,
};

static const struct v4l2_subdev_core_ops m10mo_core_ops = {
	.g_ctrl = v4l2_subdev_g_ctrl,
	.s_ctrl = v4l2_subdev_s_ctrl,
	.s_power = m10mo_s_power,
	.init	= m10mo_fw_start,
	.ioctl  = m10mo_ioctl,
};

static const struct v4l2_subdev_pad_ops m10mo_pad_ops = {
	.enum_mbus_code = m10mo_enum_mbus_code,
	.enum_frame_size = m10mo_enum_frame_size,
	.get_fmt = m10mo_get_pad_format,
	.set_fmt = m10mo_set_pad_format,

};

static int m10mo_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct m10mo_device *dev = to_m10mo_sensor(sd);
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

static const struct v4l2_subdev_sensor_ops m10mo_sensor_ops = {
	.g_skip_frames	= m10mo_g_skip_frames,
};

static const struct v4l2_subdev_ops m10mo_ops = {
	.core	= &m10mo_core_ops,
	.pad	= &m10mo_pad_ops,
	.video	= &m10mo_video_ops,
	.sensor = &m10mo_sensor_ops,
};

static int dump_fw(struct m10mo_device *dev)
{
	int ret = 0;
	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		ret = -EBUSY;
		goto leave;
	}
	__m10mo_s_power(&dev->sd, 1, true);
	m10mo_dump_fw(dev);
	__m10mo_s_power(&dev->sd, 0, true);
leave:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int read_fw_checksum(struct m10mo_device *dev, u16 *result)
{
	int ret;

	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		ret = -EBUSY;
		goto leave;
	}
	__m10mo_s_power(&dev->sd, 1, true);
	ret = m10mo_fw_checksum(dev, result);
	__m10mo_s_power(&dev->sd, 0, true);
leave:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int read_fw_version(struct m10mo_device *dev, char *buf)
{
	int ret;

	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		ret = -EBUSY;
		goto leave;
	}
	__m10mo_s_power(&dev->sd, 1, true);
	ret = m10mo_get_isp_fw_version_string(dev, buf, M10MO_MAX_FW_ID_STRING,
			dev->fw_addr_id);
	__m10mo_s_power(&dev->sd, 0, true);
leave:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int update_fw(struct m10mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	int ret = 0;
	u16 result;

	mutex_lock(&dev->input_lock);
	if (dev->power == 1) {
		ret = -EBUSY;
		goto leave;
	}
	__m10mo_s_power(&dev->sd, 1, true);
	ret = m10mo_program_device(dev);
	__m10mo_s_power(&dev->sd, 0, true);
	if (ret)
		goto leave;
	/* Power cycle chip and re-identify the version */
	__m10mo_s_power(&dev->sd, 1, true);
	ret = m10mo_identify_fw_type(&dev->sd);
	__m10mo_s_power(&dev->sd, 0, true);
	if (ret)
		goto leave;

	__m10mo_s_power(&dev->sd, 1, true);
	ret = m10mo_fw_checksum(dev, &result);
	__m10mo_s_power(&dev->sd, 0, true);
	dev_info(&client->dev, "m10mo FW checksum: %d\n", result);

leave:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static ssize_t m10mo_optical_zoom_read(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	ret = m10mo_readb(&m10mo_dev->sd, CATEGORY_MONITOR, 0x20, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, zoom step = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m10mo_optical_zoom_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%d", &val);
    printk("@%s %d, zoom position %d\n", __func__, __LINE__, val);

    if(val < 10 && val > 0){

        ret = m10mo_request_cmd_effect(&m10mo_dev->sd, M10MO_START_OPTICAL_ZOOM, &val);
        if (ret)
            return ret;

        return len;
    }else if(val == 13){
        ret = m10mo_writeb(&m10mo_dev->sd, CATEGORY_LENS, 0x02, 0x13);
        if(ret) goto out;
        printk("@%s %d, single AF without LiteOn Cali data\n", __func__, __LINE__);
        return len;
    }else if(val == 14){
        ret = m10mo_writeb(&m10mo_dev->sd, CATEGORY_LENS, 0x02, 0x14);
        if(ret) goto out;
        printk("@%s %d, single AF with LiteOn Cali data\n", __func__, __LINE__);
        return len;
    }else if(val == 19){
        ret = m10mo_writeb(&m10mo_dev->sd, CATEGORY_LENS, 0x02, 0x09);
        if(ret) goto out;
        printk("@%s %d, focus calibration\n", __func__, __LINE__);
        return len;
    }

out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_optical_zoom, S_IRUGO | S_IWUSR, m10mo_optical_zoom_read, m10mo_optical_zoom_write);

static ssize_t m10mo_focus_step_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	ret = m10mo_readw(&m10mo_dev->sd, CATEGORY_LENS, 0x10, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, focus step = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m10mo_focus_step_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%d", &val);
    printk("@%s %d, focus position %d\n", __func__, __LINE__, val);
    if(val < 0x2000 && val > 0){
		ret = m10mo_writew(&m10mo_dev->sd, CATEGORY_TEST, 0x18, val);
        if(ret)
            goto out;
        printk("@%s %d, write focus position succeed\n", __func__, __LINE__);
		return len;
	}
out:
	return -EINVAL;
}
static DEVICE_ATTR(isp_focusstep, S_IRUGO | S_IWUSR, m10mo_focus_step_read, m10mo_focus_step_write);

static ssize_t m10mo_flash_rom_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return scnprintf(buf, PAGE_SIZE, "flash\n");
}

static ssize_t m10mo_flash_rom_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t len)
{
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	if (!strncmp(buf, "flash", 5)) {
		update_fw(m10mo_dev);
		return len;
	}
	return -EINVAL;
}
static DEVICE_ATTR(isp_flashfw, S_IRUGO | S_IWUSR, m10mo_flash_rom_show,
		   m10mo_flash_rom_store);

static ssize_t m10mo_flash_spi_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);
	return scnprintf(buf, PAGE_SIZE,
			 "%d\n", m10mo_get_spi_state(m10mo_dev));
}

static ssize_t m10mo_flash_spi_store(struct device *dev,
                                  struct device_attribute *attr,
                                  const char *buf, size_t len)
{
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);
	unsigned long value;
	int ret;

	if (strict_strtoul(buf, 0, &value))
		return -EINVAL;

	ret = m10mo_set_spi_state(m10mo_dev, value);
	if (ret)
		return ret;
	return len;
}
static DEVICE_ATTR(isp_spi, S_IRUGO | S_IWUSR, m10mo_flash_spi_show,
		   m10mo_flash_spi_store);

static ssize_t m10mo_flash_checksum_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m10mo_device *m10_dev = dev_get_drvdata(dev);
	ssize_t ret;
	u16 result;

	ret  = read_fw_checksum(m10_dev, &result);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%04x\n", result);
}
static DEVICE_ATTR(isp_checksum, S_IRUGO, m10mo_flash_checksum_show, NULL);

static ssize_t m10mo_flash_version_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m10mo_device *m10_dev = dev_get_drvdata(dev);
	ssize_t ret;

	ret  = read_fw_version(m10_dev, buf);
	if (ret)
		return ret;

	return scnprintf(buf, PAGE_SIZE, "%s\n", buf);
}
static DEVICE_ATTR(isp_version, S_IRUGO, m10mo_flash_version_show, NULL);

static ssize_t m10mo_flash_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct m10mo_device *m10_dev = dev_get_drvdata(dev);
	dump_fw(m10_dev);
	return scnprintf(buf, PAGE_SIZE, "done\n");
}
static DEVICE_ATTR(isp_fw_dump, S_IRUGO, m10mo_flash_dump_show, NULL);

static ssize_t m10mo_apk_capture_mode_read(struct device *dev, struct device_attribute *attr, char *buf)
{
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	val = m10mo_dev->capture_mode;
	printk("@%s %d, m10mo_dev->capture_mode = %d\n", __func__, __LINE__, val);

	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m10mo_apk_capture_mode_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
	m10mo_dev->capture_mode = val;
    (void)__m10mo_param_mode_set(&m10mo_dev->sd);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_CAPTURE_CTRL, REQUEST_MULTI_CAP_FRAMES, val);
    (void) m10mo_request_cmd_effect(&m10mo_dev->sd, M10MO_MONITOR_MODE_ZSL_REQUEST_CMD, NULL);
    printk("@%s %d, m10mo_dev->capture_mode %d\n", __func__, __LINE__, val);

	return len;
}
static DEVICE_ATTR(isp_apk_capture_mode, S_IRUGO | S_IWUSR, m10mo_apk_capture_mode_read, m10mo_apk_capture_mode_write);

static int m10mo_ispd1(struct m10mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	struct v4l2_subdev *sd = &dev->sd;
	int ret = 0;

	mutex_lock(&dev->input_lock);

	dev_info(&client->dev, "ispd1 start\n");
	ret = m10mo_dump_string_log1(sd);
	if (ret < 0)
		dev_err(&client->dev, "isp log 1 error\n");

	dev_info(&client->dev, "ispd1 finished\n");
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int m10mo_ispd2(struct m10mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	struct v4l2_subdev *sd = &dev->sd;
	int ret = 0;

	mutex_lock(&dev->input_lock);

	dev_info(&client->dev, "ispd2 start\n");

	ret = m10mo_dump_string_log2_1(sd);
	if (ret != 0)
		dev_err(&client->dev, "m10mo_dump_string_log2_1 error\n");

	dev_info(&client->dev, "log2_1 finished\n");

	ret = m10mo_dump_string_log2_2(sd);
	if (ret != 0)
		dev_err(&client->dev, "m10mo_dump_string_log2_2 error\n");

	dev_info(&client->dev, "ispd2_2 finish\n");

	ret = m10mo_dump_string_log2_3(sd);
	if (ret != 0)
		dev_err(&client->dev, "m10mo_dump_string_log2_3 error\n");

	dev_info(&client->dev, "ispd2_3 finish\n");

	mutex_unlock(&dev->input_lock);
	return ret;
}

static int m10mo_ispd3(struct m10mo_device *dev)
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

	ret = m10mo_dump_string_log2_1(sd);
	if (ret != 0)
		dev_err(&client->dev, "m10mo_dump_string_log2_1 error\n");

	dev_info(&client->dev, "log2_1 finished\n");

	ret = m10mo_dump_string_log2_2(sd);
	if (ret != 0)
		dev_err(&client->dev, "m10mo_dump_string_log2_2 error\n");

	dev_info(&client->dev, "ispd2_2 finish\n");

	ret = m10mo_dump_string_log2_3(sd);
	if (ret != 0)
		dev_err(&client->dev, "m10mo_dump_string_log2_3 error\n");

	dev_info(&client->dev, "ispd2_3 finish\n");

	dev_info(&client->dev, "ispd3 finished\n");

	mutex_unlock(&dev->input_lock);
	return ret;
}

static int m10mo_ispd4(struct m10mo_device *dev)
{
	struct i2c_client *client = v4l2_get_subdevdata(&dev->sd);
	struct v4l2_subdev *sd = &dev->sd;
	int ret = 0;

	mutex_lock(&dev->input_lock);

	dev_info(&client->dev, "ispd4 start\n");
	ret = m10mo_dump_string_log3(sd);
	if (ret < 0)
		dev_err(&client->dev, "isp log 4 error\n");

	dev_info(&client->dev, "ispd4 finished\n");
	mutex_unlock(&dev->input_lock);
	return ret;
}

void m10mo_dump_log(struct v4l2_subdev *sd)
{
	struct m10mo_device *m10mo_dev = to_m10mo_sensor(sd);

	/*
	 * dbglvl: bit0 to dump m10mo_ispd1
	 * dbglvl: bit1 to dump m10mo_ispd2
	 * dbglvl: bit2 to dump m10mo_ispd4
	 * Debug log 3 is most important, so dump first
	 */
	if (dbglvl & 4)
		m10mo_ispd4(m10mo_dev);

	if (dbglvl & 2)
		m10mo_ispd2(m10mo_dev);

	if (dbglvl & 1)
		m10mo_ispd1(m10mo_dev);
}

static ssize_t m10mo_isp_log_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t len)
{
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	if (!strncmp(buf, "1", 1))
		m10mo_ispd1(m10mo_dev);
	else if (!strncmp(buf, "4", 1))
		m10mo_ispd4(m10mo_dev);
	else if (!strncmp(buf, "2", 1))
		m10mo_ispd2(m10mo_dev);
	else if (!strncmp(buf, "3", 1))
		m10mo_ispd3(m10mo_dev);
	else
		m10mo_ispd4(m10mo_dev);

	return len;
}

static DEVICE_ATTR(isp_log, S_IRUGO | S_IWUSR, NULL, m10mo_isp_log_store);

static ssize_t m10mo_digital_zoom_read(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	ret = m10mo_readw(&m10mo_dev->sd, CATEGORY_MONITOR, 0x21, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, zoom step = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m10mo_digital_zoom_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%d", &val);
    printk("@%s %d, digital zoom position %d\n", __func__, __LINE__, val);

    if(0x0 < val && val < 0x20 ){
        ret = m10mo_request_cmd_effect(&m10mo_dev->sd, M10MO_START_DIGITAL_ZOOM, &val);
        if (ret < 0)
            return ret;
        return len;
    }else{
	    printk("@%s %d, wrong zoom position %d\n", __func__, __LINE__, val);
	}

	return -EINVAL;
}
static DEVICE_ATTR(isp_digital_zoom, S_IRUGO | S_IWUSR, m10mo_digital_zoom_read, m10mo_digital_zoom_write);

static ssize_t m10mo_i2c_debug_byte_write_cat(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	ret = m10mo_readw(&m10mo_dev->sd, CATEGORY_MONITOR, 0x21, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, zoom step = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m10mo_i2c_debug_byte_write_echo(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u8 cat, reg_addr;
	u32 val, reg_val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
    reg_val  = val & 0xFF;
	reg_addr = (val >> 8) & 0xFF;
	cat      = (val >> 16) & 0xFF;
    printk(KERN_INFO "ASUSBSP --- @m10mo_i2c_debug_byte_write_echo, cat is 0x%x, reg_addr is 0x%x, reg_val is 0x%x \n",
                                  cat, reg_addr, reg_val);

    ret = m10mo_writeb(&m10mo_dev->sd, cat, reg_addr, reg_val);
    if (!ret){
	    printk(KERN_INFO "ASUSBSP --- @m10mo_i2c_debug_byte_write_echo \n");
        return len;
	}else{
	    return -EINVAL;
    }
}
static DEVICE_ATTR(m10mo_i2c_debug_byte_write, S_IRUGO | S_IWUSR, m10mo_i2c_debug_byte_write_cat, m10mo_i2c_debug_byte_write_echo);
static ssize_t m10mo_debug_state_read(struct device *dev,	struct device_attribute *attr, char *buf)
{
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	printk("@%s %d, \n", __func__, __LINE__);
	return scnprintf(buf, PAGE_SIZE, "dev->requested_cmd is %d, dev->cmd is %d, m10mo_capture_pre_flag is %d \n", m10mo_dev->requested_cmd, m10mo_dev->cmd, m10mo_capture_pre_flag);
}

static ssize_t m10mo_debug_state_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
	u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%d", &val);
    m10mo_dev->cmd = val;

    return len;
}
static DEVICE_ATTR(m10mo_i2c_debug_state, S_IRUGO | S_IWUSR, m10mo_debug_state_read, m10mo_debug_state_write);

static ssize_t m10mo_i2c_debug_byte_read_cat(struct device *dev,	struct device_attribute *attr, char *buf)
{
    int ret = 0;
    u32 val;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	ret = m10mo_readw(&m10mo_dev->sd, CATEGORY_MONITOR, 0x21, &val);
	if (ret)
        return -EINVAL;

    printk("@%s %d, zoom step = %d\n", __func__, __LINE__, val);
	return scnprintf(buf, PAGE_SIZE, "%d\n", val);
}
static ssize_t m10mo_i2c_debug_byte_read_echo(struct device *dev, struct device_attribute *attr, const char *buf, size_t len)
{
    int ret = 0;
    u8 cat, reg_addr;
	u32 val;
	u32 reg_val = 0;
	struct m10mo_device *m10mo_dev = dev_get_drvdata(dev);

	sscanf(buf, "%x", &val);
//    reg_val  = val & 0xFF;
	reg_addr = (val >> 8) & 0xFF;
	cat      = (val >> 16) & 0xFF;
    printk(KERN_INFO "ASUSBSP --- @m10mo_i2c_debug_byte_read_echo, cat is 0x%x, reg_addr is 0x%x \n",
                                  cat, reg_addr);

    ret = m10mo_readb(&m10mo_dev->sd, cat, reg_addr, &reg_val);
    if (!ret){
	    printk(KERN_INFO "ASUSBSP --- @m10mo_i2c_debug_byte_read_echo, reg_val is 0x%x\n", reg_val);
        return len;
	}else{
	    return -EINVAL;
    }
}
static DEVICE_ATTR(m10mo_i2c_debug_byte_read, S_IRUGO | S_IWUSR, m10mo_i2c_debug_byte_read_cat, m10mo_i2c_debug_byte_read_echo);

static struct attribute *sysfs_attrs_ctrl[] = {
	&dev_attr_isp_optical_zoom.attr,
	&dev_attr_isp_focusstep.attr,
	&dev_attr_isp_flashfw.attr,
	&dev_attr_isp_checksum.attr,
	&dev_attr_isp_fw_dump.attr,
	&dev_attr_isp_spi.attr,
	&dev_attr_isp_version.attr,
	&dev_attr_isp_log.attr,
	&dev_attr_isp_digital_zoom.attr,
	&dev_attr_m10mo_i2c_debug_byte_write.attr,
	&dev_attr_m10mo_i2c_debug_state.attr,
	&dev_attr_isp_apk_capture_mode.attr,
	&dev_attr_m10mo_i2c_debug_byte_read.attr,
	NULL
};

static struct attribute_group m10mo_attribute_group[] = {
	{.attrs = sysfs_attrs_ctrl },
};

static int m10mo_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct m10mo_device *dev = to_m10mo_sensor(sd);

	sysfs_remove_group(&client->dev.kobj,
			   m10mo_attribute_group);

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

static int m10mo_probe(struct i2c_client *client,
				  const struct i2c_device_id *id)
{
	struct m10mo_device *dev;
	struct camera_mipi_info *mipi_info = NULL;
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
				  struct m10mo_platform_data,
				  common);

	dev->cmd = M10MO_NO_CMD_REQUEST;
	dev->m10mo_mode = M10MO_PARAMETER_MODE;
	dev->requested_cmd = M10MO_NO_CMD_REQUEST;
	dev->iso_sensitivity =  REG_AE_ISOMODE_ISO100;
	dev->iso_mode = V4L2_ISO_SENSITIVITY_AUTO;
	dev->monitor_params.af_mode = AF_NORMAL;
	dev->monitor_params.exe_mode = AF_STOP;

	mutex_init(&dev->input_lock);

	v4l2_i2c_subdev_init(&(dev->sd), client, &m10mo_ops);

	ret = m10mo_s_config(&dev->sd, client->irq);
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

	ret = __m10mo_init_ctrl_handler(dev);
	if (ret)
		goto out_free_irq;

	if (mipi_info)
		dev->num_lanes = mipi_info->num_lanes;

	dev->curr_res_table = resolutions[0];
	dev->entries_curr_table = resolutions_sizes[0];

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &m10mo_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
	dev->format.code = V4L2_MBUS_FMT_UYVY8_1X16;
	dev->shot_mode = SHOT_MODE_AUTO;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret)
		goto out_free_irq;

	ret = sysfs_create_group(&client->dev.kobj,
				m10mo_attribute_group);
	if (ret) {
		dev_err(&client->dev, "%s Failed to create sysfs\n", __func__);
		goto out_sysfs_fail;
	}
    ret = gpio_request(60, "m10mo_cap_debug");
	gpio_direction_output(60, 1);
	/* Request SPI interface to enable FW update over the SPI */
	if (dev->pdata->spi_setup)
		dev->pdata->spi_setup(&dev->pdata->spi_pdata, dev);
	sdd = &dev->sd;
	return 0;

out_sysfs_fail:
	media_entity_cleanup(&dev->sd.entity);
out_free_irq:
#ifndef NOIRQ
	free_irq(client->irq, &dev->sd);
#endif
out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}

static const struct i2c_device_id m10mo_id[] = {
	{ M10MO_NAME, 0 },
	{}
};
MODULE_DEVICE_TABLE(i2c, m10mo_id);

static struct i2c_driver m10mo_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = M10MO_NAME,
	},
	.probe = m10mo_probe,
	.remove = m10mo_remove,
	.id_table = m10mo_id,
};

static __init int init_m10mo(void)
{
	int ret;
    switch (Read_PROJ_ID()) {
		case PROJ_ID_ZX550ML:
			ret = intel_scu_ipc_iowrite8(GPIO4CTLO_REG, 0x31);
			pr_info("Project ID is ZX550ML, enable flash PIN... (%d)\n",ret);
		case PROJ_ID_ZE551ML:
		case PROJ_ID_ZE550ML:
			pr_info("Project ID is ZX550ML, M10MO init... ()\n");
			return i2c_add_driver(&m10mo_driver);
		break;

		case PROJ_ID_ZE500ML:
		case PROJ_ID_ZR550ML:
			pr_info("Project ID is NOT ZX550ML, M10MO exit...\n");
			return 0;
		break;
		default:
			pr_info("Project ID is NOT ZX550ML, M10MO exit...\n");
			return 0;
		break;
    }//end switch
}

static __exit void exit_m10mo(void)
{
	int ret;
	ret = intel_scu_ipc_iowrite8(GPIO4CTLO_REG, 0x30);
	i2c_del_driver(&m10mo_driver);
}

module_init(init_m10mo);
module_exit(exit_m10mo);

MODULE_DESCRIPTION("M10MO ISP driver");
MODULE_AUTHOR("Kriti Pachhandara <kriti.pachhandara@intel.com>");
MODULE_LICENSE("GPL");

