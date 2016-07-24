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

#include <linux/kernel.h> /* pr_info cO?Y include AER¡Ñ*/
#include <linux/init.h>
#include <linux/module.h> /* cO?3 module ?Y-nAER¡Ñ*/
#include <linux/version.h>

#include <linux/platform_device.h>

#include <linux/fs.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#include <linux/delay.h>
#include <linux/uaccess.h>
#include <linux/timer.h>
#include <linux/slab.h>
#include <linux/m10mo_workaround.h>
#include <linux/m10mo.h>
#include <asm/intel-mid.h>
#include <linux/HWVersion.h>

#include "vl6180_lf_api.h"
#include "Laser_forcus_sysfs.h"
#include "vl6180x_api.h"
#include "vl6180x_def.h"

#define VL6180_NAME     "vl6180"

//extern int m10mo_read_fac(u8 len, u8 category, u8 reg, u32 *val);
//extern int m10mo_write_fac(u8 len, u8 category, u8 reg, u32 val);
//extern int m10mo_s_power_fac(int on); 

#undef CDBG
#define CDBG(fmt, args...) pr_info(fmt, ##args)

#undef DBG_LOG
#define DBG_LOG(fmt, args...) pr_info(fmt, ##args)

#undef REG_RW_DBG
#define REG_RW_DBG(fmt, args...) pr_info(fmt, ##args)

#undef API_DBG
#define API_DBG(fmt, args...) pr_info(fmt, ##args)

/* Out of range */
#define OUT_OF_RANGE 765

/* Time out value: mm */
#define TIMEOUT_VAL 80	//ZE550KL define 80ms,other project can define again

#define VL6180_API_ERROR_COUNT_MAX		5

/*For LaserFocus STATUS Controll+++*/
#define	STATUS_PROC_FILE				"driver/LaserFocus_Status"
#define	STATUS_PROC_FILE_FOR_CAMERA	"driver/LaserFocus_Status_For_Camera"
#define	DEVICE_TURN_ON_FILE			"driver/LaserFocus_on"
#define	DEVICE_GET_VALUE				"driver/LaserFocus_value"
#define	DEVICE_GET_MORE_VALUE				"driver/LaserFocus_value_more_info"
#define	DEVICE_SET_CALIBRATION			"driver/LaserFocus_CalStart"
#define	DEVICE_DUMP_REGISTER_VALUE	"driver/LaserFocus_regiser_dump"
#define	DEVICE_SET_REGISTER_VALUE	"driver/LaserFocus_regiser_set"
#define CHECK_LASER_DEVICE          "driver/Check_Laser_Device"
#define DEBUG_DUMP                  "driver/LaserFocus_debug_dump"
static struct proc_dir_entry *status_proc_file;
static struct proc_dir_entry *device_trun_on_file;
static struct proc_dir_entry *device_get_value_file;
static struct proc_dir_entry *device_get_more_value_file;
static struct proc_dir_entry *device_set_calibration_file;
static struct proc_dir_entry *dump_laser_focus_register_file;
static struct proc_dir_entry *set_laser_focus_register_file;
static struct proc_dir_entry *check_laser_device_file;
static struct proc_dir_entry *dump_debug_register_file;

static int32_t VL6180x_power_up(struct laser_focus_ctrl_t *a_ctrl);
static int32_t VL6180x_power_down(struct laser_focus_ctrl_t *a_ctrl);
//static int VL6180x_match_id(struct laser_focus_ctrl_t *s_ctrl);
//static int VL6180x_init(struct laser_focus_ctrl_t *a_ctrl);
//static int VL6180x_deinit(struct laser_focus_ctrl_t *a_ctrl);

struct laser_focus_ctrl_t *vl6180x_t = NULL;
struct timeval timer, timer2;
bool camera_on_flag = false;
int vl6180x_check_status = 0;
static int ATD_status;
int Laser_Device_ID = 0;

static int DMax = 0;
static int errorStatus = 16;
struct mutex vl6180x_mutex;

/* Display current time */
struct timeval get_current_time(void){
	struct timeval now;
	
	do_gettimeofday(&now);

	/*
		tv_sec: second
		tv_usec: microseconds 
	*/
	//printk("Current UTC: %lu (%lu)\n", now.tv_sec, now.tv_usec);

	return now;
}

#define M10MO_I2C_ROUTER_CATGORY    0x0A
    #define M10MO_I2C_ROUTER_SLAVE_ADDR_H   0x83
    #define M10MO_I2C_ROUTER_SLAVE_ADDR_L   0x84
    #define M10MO_I2C_ROUTER_TRIGGER_CMD    0x85
        #define M10MO_LS_1BYTE_READ                 0x01
        #define M10MO_LS_2BYTE_READ                 0x02
        #define M10MO_LS_4BYTE_READ                 0x04
        #define M10MO_LS_1BYTE_WRITE                0x11
        #define M10MO_LS_2BYTE_WRITE                0x12
        #define M10MO_LS_4BYTE_WRITE                0x14

    #define M10MO_I2C_ROUTER_DATA1          0x86
    #define M10MO_I2C_ROUTER_DATA2          0x87
    #define M10MO_I2C_ROUTER_DATA3          0x88
    #define M10MO_I2C_ROUTER_DATA4          0x89
    #define M10MO_I2C_ROUTER_STATUS         0x8D
        #define M10MO_I2C_REASON_OK                 0x00
        #define M10MO_I2C_REASON_NG                 0xFF
        #define M10MO_I2C_REASON_NONE               0x01

#define VL6180_DEVICE_ID        0xB4

int STOP_Preview(void)
{
    int rc;
    u32 val;

    #if 1
    rc = m10mo_read_fac(0x01,0x00,0x0b,&val);
    if (!rc)
    {
        if (val != 0x01)
        {
            rc = m10mo_write_fac(0x01,0x00,0x0b,0x01);
            rc = m10mo_read_fac(0x01,0x00,0x0b,&val);
        }
    }
    #else
    rc = m10mo_write_fac(0x01,0x00,0x0b,0x01);
    rc = m10mo_read_fac(0x01,0x00,0x0b,&val);
    #endif

	printk("[LF][vl6180x][ISP]mode rc:0x%x  val:0x%x\n", rc, val);
    return rc;
}

int Check_Laser_Sensor(void)
{
    int rc;
    u32 val;
    int retry_i = 0;

	DBG_LOG("[LF][vl6180x] check laser sensor device%s \n", __func__);	
        m10mo_USB_status(1);
Retry_power_on:
    rc = m10mo_s_power_fac(1);
    if (rc)
    {
        m10mo_s_power_fac(0);
        if ( ++retry_i > 2) return -1;
        msleep(5000);
        DBG_LOG("[LF][vl6180x][ISP]retry power on i:%d rc:0x%x \n", retry_i, rc);	
        goto Retry_power_on;
    }
	DBG_LOG("[LF][vl6180x][ISP]power on  rc:0x%x \n", rc);	

    rc = m10mo_read_fac(0x01,0x00,0x02,&val);
	DBG_LOG("[LF][vl6180x][ISP]FW_H  rc:0x%x  val:0x%x\n", rc, val);	

    rc = m10mo_read_fac(0x01,0x00,0x03,&val);
	DBG_LOG("[LF][vl6180x][ISP]FW_L rc:0x%x  val:0x%x\n", rc, val);

    STOP_Preview();

    m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_H,0x00);
    m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_L,0x00);
    m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_TRIGGER_CMD,M10MO_LS_1BYTE_READ);

    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_STATUS,&val);
	DBG_LOG("[LF][vl6180x][ISP]ls status  rc:0x%x  val:0x%x\n", rc, val);

    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA1,&val);
	DBG_LOG("[LF][vl6180x][ISP]ls data1  rc:0x%x  val:0x%x\n", rc, val);

    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA2,&val);
	DBG_LOG("[LF][vl6180x][ISP]ls data2  rc:0x%x  val:0x%x\n", rc, val);

    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA3,&val);
	DBG_LOG("[LF][vl6180x][ISP]ls data3  rc:0x%x  val:0x%x\n", rc, val);

    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA4,&val);
    if (!rc)
    {
        Laser_Device_ID = val;
    }
	DBG_LOG("[LF][vl6180x][ISP]ls data4  rc:0x%x  val:0x%x\n", rc, Laser_Device_ID);

    if (Laser_Device_ID != VL6180_DEVICE_ID)
    {
	    printk("[LF][vl6180x][ISP]Device Id is not correct\n");
        return -1;
    }

    return rc;
}

int Write_ISP_NVM_calibration_Reg24(uint8_t data)
{
    int rc;
    u32 val;
    int retry_i = 0;

    rc = m10mo_write_fac(0x02,0x0D,0xA7,245); //laser_sensor_reg24_calibration
    rc = m10mo_write_fac(0x01,0x0D,0xA9,data);
    rc = m10mo_write_fac(0x01,0x0D,0xA6,0x09); // 1byte write
    rc = m10mo_write_fac(0x01,0x0D,0xA5,0x03);
    msleep(30);
    retry_i = 0;
retry_24:
    rc = m10mo_read_fac(0x01,0x0D,0xAE,&val);
    if (0x00 == val)
    {
        DBG_LOG("[LF][vl6180x][ISP]NVM_Reg24 done:0x%x\n", data);
    }
    else if ( 0xFF == val)
    {
        retry_i++;
        if (retry_i>=10)
        {
            printk("[LF][vl6180x][ISP]NVM_Reg24 retry fail\n");
            return -1;
        }
        else
        {
            DBG_LOG("[LF][vl6180x][ISP]NVM_Reg24 retry:%d\n", retry_i);
            msleep(10);
            goto retry_24;
        }
    }
    else
    {
        DBG_LOG("[LF][vl6180x][ISP]NVM_Reg24 fail rc:0x%x  val:0x%x\n", rc, val);
    }

    return 0;
}

int Write_ISP_NVM_calibration_Reg1E(uint16_t data)
{
    int rc;
    u32 val;
    int retry_i = 0;
    uint8_t data_h, data_l;

    rc = m10mo_write_fac(0x02,0x0D,0xA7,246); //laser_sensor_reg1e_calibration
    data_h = (uint8_t)(data & 0xFF00)>>8;
    data_l = (uint8_t)(data & 0xFF);
    rc = m10mo_write_fac(0x01,0x0D,0xA9,data_h);
    rc = m10mo_write_fac(0x01,0x0D,0xAA,data_l);
    rc = m10mo_write_fac(0x01,0x0D,0xA6,0x0A); // 2byte write
    rc = m10mo_write_fac(0x01,0x0D,0xA5,0x03);
    msleep(30);
    retry_i = 0;
retry_1e:
    rc = m10mo_read_fac(0x01,0x0D,0xAE,&val);
    if (0x00 == val)
    {
        DBG_LOG("[LF][vl6180x][ISP]NVM_Reg1E done:0x%x\n", data);
    }
    else if ( 0xFF == val)
    {
        retry_i++;
        if (retry_i>=10)
        {
            printk("[LF][vl6180x][ISP]NVM_Reg1E retry fail\n");
            return -1;
        }
        else
        {
            DBG_LOG("[LF][vl6180x][ISP]NVM_Reg1E retry:%d\n", retry_i);
            msleep(10);
            goto retry_1e;
        }
    }
    else
    {
        DBG_LOG("[LF][vl6180x][ISP]NVM_Reg1E fail rc:0x%x  val:0x%x\n", rc, val);
    }

    return 0;
}

int ASUS_VL6180x_WrByte(uint32_t register_addr, uint8_t i2c_write_data)
{
#if 1
    int rc;
    u32 val;
    uint8_t reg_h,reg_l;

    reg_h = (uint8_t)((register_addr&0x0000FF00) >> 8);
    reg_l = (uint8_t)(register_addr&0xFF);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_H,reg_h);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_L,reg_l);

   rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA4,i2c_write_data);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_TRIGGER_CMD,M10MO_LS_1BYTE_WRITE);
    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_STATUS,&val);
    if (M10MO_I2C_REASON_OK == val)
    {
        REG_RW_DBG("[LF][vl6180x][ISP]%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
    }
    else
    {
	    DBG_LOG("[LF][vl6180x][ISP]ls status  rc:0x%x  val:0x%x\n", rc, val);
    }

//ASUS_VL6180x_RdByte(register_addr, (uint16_t*)&val);
	return rc;
#else
    return 0;
#endif
}

int ASUS_VL6180x_RdByte(uint32_t register_addr, uint8_t *i2c_read_data)
{
#if 1
    int rc;
    u32 val;
    uint8_t reg_h,reg_l;

    reg_h = (uint8_t)((register_addr&0x0000FF00) >> 8);
    reg_l = (uint8_t)(register_addr&0xFF);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_H,reg_h);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_L,reg_l);

    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_TRIGGER_CMD,M10MO_LS_1BYTE_READ);
    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_STATUS,&val);
    if (M10MO_I2C_REASON_OK == val)
    {
        rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA4,(u32*)i2c_read_data);

        REG_RW_DBG("[LF][vl6180x][ISP]%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);
    }
    else
    {
	    DBG_LOG("[LF][vl6180x][ISP]ls status  rc:0x%x  val:0x%x\n", rc, val);
    }

	return rc;
#else
    return 0;
#endif
}

int ASUS_VL6180x_WrWord(uint32_t register_addr, uint16_t i2c_write_data)
{
#if 1
    int rc;
    u32 val;
    uint8_t reg_h,reg_l;

    reg_h = (uint8_t)((register_addr&0x0000FF00) >> 8);
    reg_l = (uint8_t)(register_addr&0xFF);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_H,reg_h);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_L,reg_l);

    rc = m10mo_write_fac(0x02,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA3,i2c_write_data);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_TRIGGER_CMD,M10MO_LS_2BYTE_WRITE);
    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_STATUS,&val);
    if (M10MO_I2C_REASON_OK == val)
    {
        REG_RW_DBG("[LF][vl6180x][ISP]%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
    }
    else
    {
	    DBG_LOG("[LF][vl6180x][ISP]ls status  rc:0x%x  val:0x%x\n", rc, val);
    }
//ASUS_VL6180x_RdWord(register_addr, (uint16_t*)&val);
	return rc;
#else
    return 0;
#endif
}

int ASUS_VL6180x_RdWord(uint32_t register_addr, uint16_t *i2c_read_data)
{
#if 1
    int rc;
    u32 val;
    uint8_t reg_h,reg_l;

    reg_h = (uint8_t)((register_addr&0x0000FF00) >> 8);
    reg_l = (uint8_t)(register_addr&0xFF);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_H,reg_h);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_L,reg_l);

    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_TRIGGER_CMD,M10MO_LS_2BYTE_READ);
    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_STATUS,&val);
    if (M10MO_I2C_REASON_OK == val)
    {
        rc = m10mo_read_fac(0x02,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA3,(u32*)i2c_read_data);

        REG_RW_DBG("[LF][vl6180x][ISP]%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);
    }
    else
    {
	    DBG_LOG("[LF][vl6180x][ISP]ls status  rc:0x%x  val:0x%x\n", rc, val);
    }

	return rc;
#else
    return 0;
#endif
}

int ASUS_VL6180x_WrDWord(uint32_t register_addr, uint32_t i2c_write_data)
{
#if 1
    int rc;
    u32 val;
    uint8_t reg_h,reg_l;

    reg_h = (uint8_t)((register_addr&0x0000FF00) >> 8);
    reg_l = (uint8_t)(register_addr&0xFF);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_H,reg_h);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_L,reg_l);

    rc = m10mo_write_fac(0x04,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA1,i2c_write_data);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_TRIGGER_CMD,M10MO_LS_4BYTE_WRITE);
    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_STATUS,&val);
    if (M10MO_I2C_REASON_OK == val)
    {
        REG_RW_DBG("[LF][vl6180x][ISP]%s: wirte register(0x%x) : 0x%x \n", __func__, register_addr, i2c_write_data);
    }
    else
    {
	    DBG_LOG("[LF][vl6180x][ISP]ls status  rc:0x%x  val:0x%x\n", rc, val);
    }
//ASUS_VL6180x_RdDWord(register_addr, (uint32_t*)&val, 4);
	return rc;
#else
    return 0;
#endif
}

int ASUS_VL6180x_RdDWord(uint32_t register_addr, uint32_t *i2c_read_data, uint16_t num_byte)
{

#if 1
    int rc;
    u32 val;
    uint8_t reg_h,reg_l;

    reg_h = (uint8_t)((register_addr&0x0000FF00) >> 8);
    reg_l = (uint8_t)(register_addr&0xFF);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_H,reg_h);
    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_SLAVE_ADDR_L,reg_l);

    rc = m10mo_write_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_TRIGGER_CMD,M10MO_LS_4BYTE_READ);
    rc = m10mo_read_fac(0x01,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_STATUS,&val);
    if (M10MO_I2C_REASON_OK == val)
    {
        rc = m10mo_read_fac(0x04,M10MO_I2C_ROUTER_CATGORY,M10MO_I2C_ROUTER_DATA1,(u32*)i2c_read_data);

        REG_RW_DBG("[LF][vl6180x][ISP]%s: read register(0x%x) : 0x%x \n", __func__, register_addr, *i2c_read_data);
    }
    else
    {
	    DBG_LOG("[LF][vl6180x][ISP]ls status  rc:0x%x  val:0x%x\n", rc, val);
    }

	return rc;
#else
    return 0;
#endif
}
int ASUS_VL6180x_UpdateByte(uint32_t register_addr, uint8_t AndData, uint8_t OrData)
{
	int status;
	uint8_t i2c_read_data, i2c_write_data;

	status = ASUS_VL6180x_RdByte(register_addr, &i2c_read_data);
	if (status < 0) {
		pr_err("[LF][vl6180x][ISP]%s: read register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

	i2c_write_data = ((uint8_t)(i2c_read_data&0xFF)&AndData) | OrData;

	status = ASUS_VL6180x_WrByte(register_addr, i2c_write_data);

	if (status < 0) {
		pr_err("[LF][vl6180x][ISP]%s: wirte register(0x%x) failed\n", __func__, register_addr);
		return status;
	}

 	REG_RW_DBG("[LF][vl6180x][ISP]%s: update register(0x%x) from 0x%x to 0x%x(AndData:0x%x;OrData:0x%x)\n", __func__, register_addr, i2c_read_data, i2c_write_data,AndData,OrData);

	return status;
}


static int VL6180x_device_Load_Calibration_Value(void){
	int status = 0;
	bool Factory_folder_file;

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_INIT_CCI)	{

        #if 1 // for test
		Factory_folder_file = true;
        #else	
		if(vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION){
			Factory_folder_file = true;
		}
		else{
			Factory_folder_file = false;
		}
        #endif
				
		/* Read Calibration data */
		vl6180x_t->laser_focus_offset_value = Laser_Forcus_sysfs_read_offset(Factory_folder_file);
		vl6180x_t->laser_focus_cross_talk_offset_value = Laser_Forcus_sysfs_read_cross_talk_offset(Factory_folder_file);

		/* Apply Calibration value */
		//VL6180x_SetOffsetCalibrationData(0, vl6180x_t->laser_focus_offset_value);
		
		status = ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, vl6180x_t->laser_focus_offset_value);
		if (status < 0) {
			pr_err("%s: wirte register(0x%x) failed\n", __func__, SYSRANGE_PART_TO_PART_RANGE_OFFSET);
			return status;
		}
		
		VL6180x_SetXTalkCompensationRate(0, vl6180x_t->laser_focus_cross_talk_offset_value);
		/*
		status = ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, vl6180x_t->laser_focus_cross_talk_offset_value);
		if (status < 0) {
			pr_err("%s: wirte register(0x%x) failed\n", __func__, SYSRANGE_CROSSTALK_COMPENSATION_RATE);
			return status;
		}
		*/
	}
	
	return status;
}

static ssize_t ATD_VL6180x_device_enable_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, rc = 0;
	char messages[8];
	if (len > 8) {
		len = 8;
	}

	if (copy_from_user(messages, buff, len)) {
		printk("[LF][vl6180x]%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);
	if (vl6180x_t->device_state == val)	{
		printk("[LF][vl6180x]%s Setting same commond (%d) !!\n", __func__, val);
		return -EINVAL;
	}

	switch (val) {
		case MSM_LASER_FOCUS_DEVICE_OFF:
			mutex_lock(&vl6180x_mutex);
			if(camera_on_flag){
				CDBG("[LF][vl6180x]%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				mutex_unlock(&vl6180x_mutex);
				break;
			}
			//rc = VL6180x_deinit(vl6180x_t);
			//rc = VL6180x_power_down(vl6180x_t);
            m10mo_s_power_fac(0);
            m10mo_USB_status(0);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
            printk("[LF][vl6180x]device off!\n");
			mutex_unlock(&vl6180x_mutex);
			break;
		case MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION:
			if(camera_on_flag){
				CDBG("[LF][vl6180x]%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				break;
			}
			if (vl6180x_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
				//rc = VL6180x_deinit(vl6180x_t);
				rc = VL6180x_power_down(vl6180x_t);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			}
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
			rc = VL6180x_power_up(vl6180x_t);
			//rc = VL6180x_init(vl6180x_t);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			} 

			API_DBG("[LF][vl6180x]%s: VL6180x_InitData Start\n", __func__);
			rc = VL6180x_InitData(0);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_InitData Success\n", __func__);

			API_DBG("[LF][vl6180x]%s: VL6180x_Prepare Start\n", __func__);
			rc = VL6180x_Prepare(0);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_Prepare Success\n", __func__);


			rc = VL6180x_device_Load_Calibration_Value();
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}

			API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Start\n", __func__);
			//rc = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
			/*if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Success\n", __func__);
			*/
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION;
			printk("[LF][vl6180x]%s Init Device (%d)\n", __func__, vl6180x_t->device_state);
		
			break;
		case MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION:
			if(camera_on_flag){
				CDBG("[LF][vl6180x]%s: %d Camera is running, do nothing!!\n ", __func__, __LINE__);
				break;
			}
			if (vl6180x_t->device_state != MSM_LASER_FOCUS_DEVICE_OFF)	{
				//rc = VL6180x_deinit(vl6180x_t);
				rc = VL6180x_power_down(vl6180x_t);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
			}
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
			rc = VL6180x_power_up(vl6180x_t);
			//rc = VL6180x_init(vl6180x_t);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			} 

			API_DBG("[LF][vl6180x]%s: VL6180x_InitData Start\n", __func__);
			rc = VL6180x_InitData(0);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_InitData Success\n", __func__);

			API_DBG("[LF][vl6180x]%s: VL6180x_Prepare Start\n", __func__);
			rc = VL6180x_Prepare(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_Prepare Success\n", __func__);


			API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Start\n", __func__);
			/*rc = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				//return -EIO;
				goto DEVICE_TURN_ON_ERROR;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Success\n", __func__);*/

			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION;
			printk("[LF][vl6180x]%s Init Device (%d)\n", __func__, vl6180x_t->device_state);
		
			break;
		case MSM_LASER_FOCUS_DEVICE_INIT_CCI:
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
			//rc = VL6180x_init(vl6180x_t);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			} 

			API_DBG("[LF][vl6180x]%s: VL6180x_InitData Start\n", __func__);
			rc = VL6180x_InitData(0);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_InitData Success\n", __func__);

			API_DBG("[LF][vl6180x]%s: VL6180x_Prepare Start\n", __func__);
			rc = VL6180x_Prepare(0);
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_Prepare Success\n", __func__);

			rc = VL6180x_device_Load_Calibration_Value();
			if (rc < 0){
				pr_err("[LF][vl6180x]%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
				
			API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Start\n", __func__);
			/*rc = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
			if (rc < 0){
				pr_err("%s Device trun on fail !!\n", __func__);
				vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_OFF;
				return -EIO;
			}
			API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Success\n", __func__);*/

			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_INIT_CCI;
			printk("[LF][vl6180x]%s Init Device (%d)\n", __func__, vl6180x_t->device_state);

			camera_on_flag = true;
			
			break;
		case MSM_LASER_FOCUS_DEVICE_DEINIT_CCI:
			mutex_lock(&vl6180x_mutex);
			//rc = VL6180x_deinit(vl6180x_t);
			vl6180x_t->device_state = MSM_LASER_FOCUS_DEVICE_DEINIT_CCI;
			camera_on_flag = false;
			mutex_unlock(&vl6180x_mutex);
			break;
		default:
			printk("[LF][vl6180x]%s commond fail !!\n", __func__);
			break;
	}
	return len;

DEVICE_TURN_ON_ERROR:
    #if 0
	rc = VL6180x_deinit(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("[LF][vl6180x]%s VL6180x_deinit failed %d\n", __func__, __LINE__);
	}
    #endif
		
	rc = VL6180x_power_down(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("[LF][vl6180x]%s VL6180x_power_down failed %d\n", __func__, __LINE__);
	}
	return -EIO;
}

static int ATD_VL6180x_device_enable_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "%d\n", vl6180x_t->device_state);
	return 0;
}

static int ATD_VL6180x_device_enable_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_device_enable_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_enable_fops = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_enable_open,
	.write = ATD_VL6180x_device_enable_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int ATD_VL6180x_device_read_range(VL6180x_RangeData_t *pRangeData)
{
	uint16_t RawRange;
	int status, i = 0;
	//struct msm_camera_i2c_client *sensor_i2c_client;
	uint8_t intStatus;
	//VL6180x_RangeData_t RangeData;

	timer = get_current_time();

    #if 0	
	/* Setting i2c client */
	sensor_i2c_client = vl6180x_t->i2c_client;
	if (!sensor_i2c_client) {
		pr_err("%s:%d failed: %p \n", __func__, __LINE__, sensor_i2c_client);
		return -EINVAL;
	}
    #endif

	/* Setting Range meansurement in single-shot mode */	
	API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Start\n", __func__);
	status = VL6180x_RangeClearInterrupt(0);
	if (status < 0) {
		pr_err("%s: rVL6180x_RangeClearInterrupt failed\n", __func__);
		return status;
	}
	API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Success\n", __func__);

	API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Start\n", __func__);
	status = VL6180x_RangeSetSystemMode(0, MODE_START_STOP|MODE_SINGLESHOT);
	if (status < 0) {
		pr_err("%s: VL6180x_RangeSetSystemMode failed\n", __func__);
		return status;
	}
	API_DBG("[LF][vl6180x]%s: VL6180x_RangeSetSystemMode Success\n", __func__);

	/* Get Sensor detect distance */
	RawRange = 0;
	for (i = 0; i <1000; i++)	{
		/* Check RESULT_INTERRUPT_STATUS_GPIO */
		API_DBG("[LF][vl6180x]%s: VL6180x_RangeGetInterruptStatus Start\n", __func__);
		status = VL6180x_RangeGetInterruptStatus(0, &intStatus);
		if (status < 0) {
			pr_err("%s: VL6180x_RangeGetInterruptStatus failed\n", __func__);
			return status;
		}
		
		API_DBG("[LF][vl6180x]%s: VL6180x_RangeGetInterruptStatus Success\n", __func__);
        API_DBG("[LF][vl6180x]intStatus: %x\n", intStatus);
		
		if (intStatus == RES_INT_STAT_GPIO_NEW_SAMPLE_READY){
			API_DBG("[LF][vl6180x]%s: VL6180x sensor ready! after loop:%d times GetInterruptStatus\n", __func__,i);
			break;
		}

		timer2 = get_current_time();
		if((((timer2.tv_sec*1000000)+timer2.tv_usec)-((timer.tv_sec*1000000)+timer.tv_usec)) > (TIMEOUT_VAL*1000)){
			API_DBG("[LF][vl6180x]%s: Timeout: Out Of Range!!\n", __func__);
			return OUT_OF_RANGE;
		}
	}
	
	if(i < 1000){		
		status = VL6180x_RangeGetMeasurement(0, pRangeData);
		if (status < 0) {
			pr_err("%s: VL6180x_RangeGetMeasurement failed\n", __func__);
			return status;
		}
		
		if (pRangeData->errorStatus == 0){
			DBG_LOG("[LF][vl6180x]%s: Read range:%d\n", __func__, (int)pRangeData->range_mm);
			API_DBG("[LF][vl6180x]%s: VL6180x_RangeGetMeasurement Success\n", __func__);
		}
		else{
			API_DBG("[LF][vl6180x]%s: VL6180x_RangeGetMeasurement Failed: errorStatus(%d)\n", __func__, pRangeData->errorStatus);
			pRangeData->range_mm = OUT_OF_RANGE;
		}
	}
	else
	{
		API_DBG("[LF][vl6180x]%s: VL6180x sensor no ready!\n", __func__);
		pRangeData->range_mm = OUT_OF_RANGE;
	}

	/* Setting SYSTEM_INTERRUPT_CLEAR to 0x01 */
	API_DBG("[LF][vl6180x]%s: VL6180x_RangeClearInterrupt Start\n", __func__);
	status = VL6180x_RangeClearInterrupt(0);
	if (status < 0) {
		pr_err("%s: VL6180x_RangeClearInterrupt failed\n", __func__);
		return status;
	}
	API_DBG("[LF][vl6180x]%s: VL6180x_RangeClearInterrupt Success\n", __func__);
	
#if VL6180x_WRAP_AROUND_FILTER_SUPPORT
	printk("[LF][vl6180x]: %d (%d:%d:%d)\n", pRangeData->range_mm, 
		pRangeData->FilteredData.def_range_mm,
		pRangeData->DMax,
		pRangeData->errorStatus);
#else
	printk("[LF][vl6180x]: %d (%d:%d)\n", pRangeData->range_mm, 
		pRangeData->DMax,
		pRangeData->errorStatus);
#endif

	return (int)pRangeData->range_mm;
}


static int ATD_VL6180x_device_get_range_read(struct seq_file *buf, void *v)
{
	int RawRange = 0;
	
	uint16_t test_0x1e_Value = 0;
	uint8_t test_0x24_Value = 0;
	uint8_t test_0x4d_Value = 0;
	uint8_t test_RangeValue = 0;
	uint8_t test_RawRange = 0;
	uint16_t test_rtnRate = 0;
	uint32_t test_DMax = 0;
	VL6180x_RangeData_t RangeData = {0};
    int camera_status;
    int ISP_Distance = -1, ISP_Dmax = -1, ISP_Error = -1;

	mutex_lock(&vl6180x_mutex);

    camera_status = Get_Camera_Status();
	printk(KERN_INFO "[vl6180][Camera_Status] get = %d\n", camera_status);

    if ( 1 == camera_status)
    {
        if (m10mo_status_fac()){
                //lanuch camera
                m10mo_read_laser(0x02,0x04,0xA9,&ISP_Distance);
                m10mo_read_laser(0x02,0x04,0xAB,&ISP_Dmax);
                m10mo_read_laser(0x01,0x04,0xAD,&ISP_Error);
     	        printk("[LF][vl6180x]: distance %d\n", ISP_Distance);

    	        seq_printf(buf, "%d\n", ISP_Distance);
        } else {

     	        printk("[LF][vl6180x]: m10mo power down");
    	        seq_printf(buf, "%d\n", 0x00);
        }
    }
    else
    {
    	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
    		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
    		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
    		seq_printf(buf, "%d\n", 0);
    		mutex_unlock(&vl6180x_mutex);
    		return 0;
    		//return -EBUSY;
    	}
    	
    	RawRange = ATD_VL6180x_device_read_range(&RangeData);
    	//RawRange = RawRange*3;
    
    	DBG_LOG("[LF][vl6180x]%s Test Data (%d)  Device (%d)\n", __func__, RawRange , vl6180x_t->device_state);
    
    	if (RawRange < 0) {
    		pr_err("%s: read_range(%d) failed\n", __func__, RawRange);
    		RawRange = 0;
    	}
    
    	DMax = RangeData.DMax;
    	errorStatus = RangeData.errorStatus;
    
    	seq_printf(buf, "%d\n", RawRange);
    	
    	
    	//create reg file
    	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION ||
    		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION)
        {
    
    		test_DMax = RangeData.DMax;//DMax
    		if (Laser_Forcus_sysfs_write_DMax(test_DMax) == false)
    			return -ENOENT;
    
    		ASUS_VL6180x_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE,&test_0x1e_Value);//0x1e
    		if (Laser_Forcus_sysfs_write_0x1e(test_0x1e_Value) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,SYSRANGE_PART_TO_PART_RANGE_OFFSET,&test_0x24_Value);//0x24
    		if (Laser_Forcus_sysfs_write_0x24(test_0x24_Value) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,RESULT_RANGE_STATUS,&test_0x4d_Value);//0x4d
    		if (Laser_Forcus_sysfs_write_0x4d(test_0x4d_Value) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,RESULT_RANGE_VAL,&test_RangeValue);//0x62
    		if (Laser_Forcus_sysfs_write_0x62(test_RangeValue) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,RESULT_RANGE_RAW,&test_RawRange);	//0x64
    		if (Laser_Forcus_sysfs_write_0x64(test_RawRange) == false)
    			return -ENOENT;
    
    		ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE,&test_rtnRate);//0x66
    		if (Laser_Forcus_sysfs_write_0x66(test_rtnRate) == false)
    			return -ENOENT;
    
    	}
    }

	mutex_unlock(&vl6180x_mutex);

	return 0;
}
 
static int ATD_VL6180x_device_get_range_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_device_get_range_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_range_fos = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_get_range_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_VL6180x_device_get_more_value_read(struct seq_file *buf, void *v)
{
	int RawRange = 0;
	
	uint16_t test_0x1e_Value = 0;
	uint8_t test_0x24_Value = 0;
	uint8_t test_0x4d_Value = 0;
	uint8_t test_RangeValue = 0;
	uint8_t test_RawRange = 0;
	uint16_t test_rtnRate = 0;
	uint32_t test_DMax = 0;
	VL6180x_RangeData_t RangeData = {0};
    int camera_status;
    u32 ISP_Distance = -1, ISP_Dmax = -1, ISP_Error = -1;

	mutex_lock(&vl6180x_mutex);

    camera_status = Get_Camera_Status();
	printk(KERN_INFO "[vl6180][Camera_Status] get = %d\n", camera_status);

    if ( 1 == camera_status)
    {
        if (m10mo_status_fac()){
                //lanuch camera
                m10mo_read_laser(0x02,0x04,0xA9,&ISP_Distance);
                m10mo_read_laser(0x02,0x04,0xAB,&ISP_Dmax);
                m10mo_read_laser(0x01,0x04,0xAD,&ISP_Error);
     	        printk("[LF][vl6180x]: %d#%d#%d\n", ISP_Distance, ISP_Dmax, ISP_Error);
    	        seq_printf(buf, "%d#%d#%d\n", ISP_Distance, ISP_Dmax, ISP_Error);
        }else{
     	        printk("[LF][vl6180x]: m10mo power down");
    		seq_printf(buf, "%d\n", 0);
        }
    }
    else
    {
    	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
    		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
    		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
    		seq_printf(buf, "%d\n", 0);
    		mutex_unlock(&vl6180x_mutex);
    		return 0;
    		//return -EBUSY;
    	}
    
    	
    	RawRange = ATD_VL6180x_device_read_range(&RangeData);
    	//RawRange = RawRange*3;
    
    	DBG_LOG("[LF][vl6180x]%s Test Data (%d)  Device (%d)\n", __func__, RawRange , vl6180x_t->device_state);
    
    	if (RawRange < 0) {
    		pr_err("%s: read_range(%d) failed\n", __func__, RawRange);
    		RawRange = 0;
    	}
    
    	DMax = RangeData.DMax;
    	errorStatus = RangeData.errorStatus;
    
    	seq_printf(buf, "%d#%d#%d\n", RawRange,RangeData.DMax,RangeData.errorStatus);
    	
    	//create reg file
    	
    
    	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_APPLY_CALIBRATION ||
    		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_NO_APPLY_CALIBRATION)
        {
    		test_DMax = RangeData.DMax;//DMax
    		if (Laser_Forcus_sysfs_write_DMax(test_DMax) == false)
    			return -ENOENT;
    
    		ASUS_VL6180x_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE,&test_0x1e_Value);//0x1e
    		if (Laser_Forcus_sysfs_write_0x1e(test_0x1e_Value) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,SYSRANGE_PART_TO_PART_RANGE_OFFSET,&test_0x24_Value);//0x24
    		if (Laser_Forcus_sysfs_write_0x24(test_0x24_Value) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,RESULT_RANGE_STATUS,&test_0x4d_Value);//0x4d
    		if (Laser_Forcus_sysfs_write_0x4d(test_0x4d_Value) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,RESULT_RANGE_VAL,&test_RangeValue);//0x62
    		if (Laser_Forcus_sysfs_write_0x62(test_RangeValue) == false)
    			return -ENOENT;
    
    		VL6180x_RdByte(0,RESULT_RANGE_RAW,&test_RawRange);	//0x64
    		if (Laser_Forcus_sysfs_write_0x64(test_RawRange) == false)
    			return -ENOENT;
    
    		ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE,&test_rtnRate);//0x66
    		if (Laser_Forcus_sysfs_write_0x66(test_rtnRate) == false)
    			return -ENOENT;
    	}
    }

	mutex_unlock(&vl6180x_mutex);

	return 0;
}

static int ATD_VL6180x_device_get_more_value_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_device_get_more_value_read, NULL);
}

static const struct file_operations ATD_laser_focus_device_get_more_value_fos = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_get_more_value_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ATD_VL6180x_device_clibration_offset(void)
{
	int i = 0, RawRange = 0, sum = 0;
	uint16_t distance;
	int16_t offset;
	int status= 0;
	VL6180x_RangeData_t RangeData = {0};
	int errorCount = 0;


	mutex_lock(&vl6180x_mutex);

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		mutex_unlock(&vl6180x_mutex);
		return -EBUSY;
	}
	
	/* Clean system offset */
	//VL6180x_SetOffsetCalibrationData(0, 0);
	VL6180x_SetXTalkCompensationRate(0, 0);
	ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0x00);
	//ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);
	
	for(i=0; i<STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{
		RawRange = (int)ATD_VL6180x_device_read_range(&RangeData);
		//msleep(50);
		printk("[LF][vl6180x]: %d, %d\n", RangeData.range_mm, RangeData.signalRate_mcps);
		if( RangeData.errorStatus != 0 )
		{
			errorCount++;
			if( i >= 0 )
			{
				i--;
			}
			
			if( errorCount > VL6180_API_ERROR_COUNT_MAX )
			{
				DBG_LOG("[LF][vl6180x]: too many 765 detected in offset calibration (%d)\n", errorCount);
				goto error;
			}

			continue;
		}
		sum += RawRange;
	}
	distance = (uint16_t)(sum / STMVL6180_RUNTIMES_OFFSET_CAL);
	DBG_LOG("[LF][vl6180x]The measure distanec is %d mm\n", distance);

	if((VL6180_OFFSET_CAL_RANGE - distance)<0){
		offset = (VL6180_OFFSET_CAL_RANGE - distance)/3;
		offset = 256+offset;
	}else{
		offset = (VL6180_OFFSET_CAL_RANGE - distance)/3;
	}
	
	//VL6180x_SetOffsetCalibrationData(0, offset);
	status = ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, offset);
	if (status < 0) {
		pr_err("%s: write register(0x%x) failed\n", __func__, SYSRANGE_PART_TO_PART_RANGE_OFFSET);
		return status;
	}

	/* Write calibration file */
	vl6180x_t->laser_focus_offset_value = offset;
	if (Laser_Forcus_sysfs_write_offset(offset) == false){
		mutex_unlock(&vl6180x_mutex);
		return -ENOENT;
	}

    Write_ISP_NVM_calibration_Reg24(offset);

	DBG_LOG("[LF][vl6180x]The measure distance is %d mm; The offset value is %d\n", distance, offset);

	mutex_unlock(&vl6180x_mutex);
	return 0;
	
error:
	
	ASUS_VL6180x_WrByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, 0x00);

	/* Write calibration file */
	vl6180x_t->laser_focus_offset_value = 0;
	Laser_Forcus_sysfs_write_offset(-1);
    Write_ISP_NVM_calibration_Reg24(-1);

	mutex_unlock(&vl6180x_mutex);
	
	return -ENOENT;
}

static int ATD_VL6180x_device_clibration_crosstalkoffset(void)
{
	int i = 0, RawRange = 0;
	int xtalk_sum  = 0, xrtn_sum = 0;
	uint16_t XtalkCompRate;
	uint16_t rtnRate = 0;
	uint8_t Data_value = 0;
	VL6180x_RangeData_t RangeData = {0};
	int errorCount = 0;


	//mutex_lock(&vl6180x_mutex);

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		mutex_unlock(&vl6180x_mutex);
		return -EBUSY;
	}

	/* Clean crosstalk offset */
	VL6180x_SetXTalkCompensationRate(0, 0);
	//ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, 0x00);

	for(i = 0; i < STMVL6180_RUNTIMES_OFFSET_CAL; i++)
	{	
		RawRange = ATD_VL6180x_device_read_range(&RangeData);
		printk("[LF][vl6180x]: %d, %d\n", RangeData.range_mm, RangeData.signalRate_mcps);
		if( RangeData.errorStatus != 0 )
		{
			errorCount++;
			if( i >= 0 )
			{
				i--;
			}
			
			if( errorCount > VL6180_API_ERROR_COUNT_MAX )
			{
				DBG_LOG("[LF][vl6180x]: too many 765 detected in xtalk calibration (%d)\n", errorCount);
				goto error;
			}
			continue;

		}
		//ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE, &rtnRate);
		rtnRate = RangeData.signalRate_mcps;
		xtalk_sum += RawRange;
		xrtn_sum += (int)rtnRate;

		//msleep(30);
	}
	printk("Crosstalk compensation rate is %d\n", xtalk_sum);
	
	//XtalkCompRate = (uint16_t)((rtnRate/STMVL6180_RUNTIMES_OFFSET_CAL)/128) * (1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*3000/VL6180_CROSSTALK_CAL_RANGE));
	//XtalkCompRate = XtalkCompRate*128;

	//Sean_Lu ++++ 0812 for factory test
	if((1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*1000/VL6180_CROSSTALK_CAL_RANGE) < 0))
	{
		printk("Crosstalk  is negative value !\n");
		ASUS_VL6180x_RdByte(0x21,&Data_value);

		if(Data_value==20)
		{
			ASUS_VL6180x_WrByte(0x21,0x6);
		}

		mutex_unlock(&vl6180x_mutex);

		goto negative;
	}
	else
	{
		XtalkCompRate = (uint16_t)(xrtn_sum/STMVL6180_RUNTIMES_OFFSET_CAL) * (1000-((xtalk_sum/STMVL6180_RUNTIMES_OFFSET_CAL)*1000/VL6180_CROSSTALK_CAL_RANGE));

		printk("Crosstalk compensation rate(after arithmetic) is %d\n", XtalkCompRate);

		XtalkCompRate = XtalkCompRate/1000;

		ASUS_VL6180x_RdByte(0x21,&Data_value);

		if(Data_value==20)
		{
			ASUS_VL6180x_WrByte(0x21,0x6);
		}

		printk("Crosstalk compensation rate(after/1000) is %d\n", XtalkCompRate);

		//ASUS_VL6180x_WrWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, XtalkCompRate);
		VL6180x_SetXTalkCompensationRate(0, XtalkCompRate);

		/* Write calibration file */
		vl6180x_t->laser_focus_cross_talk_offset_value = XtalkCompRate;
		if (Laser_Forcus_sysfs_write_cross_talk_offset(XtalkCompRate) == false){
			mutex_unlock(&vl6180x_mutex);
			return -ENOENT;
		}

        Write_ISP_NVM_calibration_Reg1E(XtalkCompRate);

		DBG_LOG("Crosstalk compensation rate is %d\n", XtalkCompRate);

		mutex_unlock(&vl6180x_mutex);
		return 0;
	}
	
negative:

	VL6180x_SetXTalkCompensationRate(0, 0);

	/* Write calibration file */
	vl6180x_t->laser_focus_cross_talk_offset_value = 0;
	Laser_Forcus_sysfs_write_cross_talk_offset(0);
	Write_ISP_NVM_calibration_Reg1E(0);

	mutex_unlock(&vl6180x_mutex);

	return 0;
	
error:
	
	VL6180x_SetXTalkCompensationRate(0, 0);

	/* Write calibration file */
	vl6180x_t->laser_focus_cross_talk_offset_value = 0;
	Laser_Forcus_sysfs_write_cross_talk_offset(-1);
    Write_ISP_NVM_calibration_Reg1E(-1);

	mutex_unlock(&vl6180x_mutex);
	
	return -ENOENT;
}

static ssize_t ATD_VL6180x_device_calibration_write(struct file *filp, const char __user *buff, size_t len, loff_t *data)
{
	int val, ret = 0;
	char messages[8];
	if (len > 8) {
		len = 8;
	}
	if (copy_from_user(messages, buff, len)) {
		printk("%s commond fail !!\n", __func__);
		return -EFAULT;
	}

	val = (int)simple_strtol(messages, NULL, 10);

	printk("%s commond : %d\n", __func__, val);
	switch (val) {
	case MSM_LASER_FOCUS_APPLY_OFFSET_CALIBRATION:
		ret = ATD_VL6180x_device_clibration_offset();
		if (ret < 0)
			return ret;
		break;
	case MSM_LASER_FOCUS_APPLY_CROSSTALK_CALIBRATION:
		ret = ATD_VL6180x_device_clibration_crosstalkoffset();
		if (ret < 0)
			return ret;
		break;
	default:
		printk("%s commond fail(%d) !!\n", __func__, val);
		return -EINVAL;
		break;
	}
	return len;
}

static const struct file_operations ATD_laser_focus_device_calibration_fops = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_device_get_range_open,
	.write = ATD_VL6180x_device_calibration_write,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int ATD_VL6180x_I2C_status_check(void)
{
	int32_t rc;
    // TODO: check M10MO is ready?

    // TODO: check laser sensor is ready?

	rc = VL6180x_power_up(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_power_up failed %d\n", __func__, __LINE__);
		return 0;
	}
    #if 0
	VL6180x_init(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_init failed %d\n", __func__, __LINE__);
		return 0;
	}
	
	rc = VL6180x_match_id(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_match_id failed %d\n", __func__, __LINE__);
		return 0;
	}
	rc = VL6180x_deinit(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_deinit failed %d\n", __func__, __LINE__);
		return 0;
	}
    #endif

	rc = VL6180x_power_down(vl6180x_t);
	if (rc < 0) {
		//kfree(vl6180x_t);
		pr_err("%s VL6180x_power_down failed %d\n", __func__, __LINE__);
		return 0;
	}

	vl6180x_check_status = 1;

	return 1;
}

static int ATD_VL6180x_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = ATD_VL6180x_I2C_status_check();

	seq_printf(buf, "%d\n", ATD_status);

	return 0;
}

static int ATD_VL6180x_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ATD_VL6180x_I2C_status_check_proc_read, NULL);
}

static const struct file_operations ATD_I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = ATD_VL6180x_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};
static int VL6180x_I2C_status_check_via_prob(void){
	return vl6180x_check_status;
}

static int VL6180x_I2C_status_check_proc_read(struct seq_file *buf, void *v)
{
	ATD_status = VL6180x_I2C_status_check_via_prob();

	seq_printf(buf, "%d\n", ATD_status);
	
	return 0;
}

static int VL6180x_I2C_status_check_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, VL6180x_I2C_status_check_proc_read, NULL);
}

static const struct file_operations I2C_status_check_fops = {
	.owner = THIS_MODULE,
	.open = VL6180x_I2C_status_check_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int dump_VL6180x_register_read(struct seq_file *buf, void *v)
{
 	int status, i = 0;
	uint16_t register_value = 0;


	for (i = 0; i <0x100; i++)	{
		register_value = 0;
		status = ASUS_VL6180x_RdWord(i, &register_value);
		if (status < 0) {
			pr_err("%s: read register(0x%x) failed\n", __func__, i);
			return status;
		}
		Laser_Forcus_sysfs_write_register_ze550kl(i, register_value);
	}
	seq_printf(buf, "%d\n", 0);

    return 0;
}

static int dump_VL6180x_laser_focus_register_open(struct inode *inode, struct  file *file)
{
	return single_open(file, dump_VL6180x_register_read, NULL);
}

static const struct file_operations dump_laser_focus_register_fops = {
	.owner = THIS_MODULE,
	.open = dump_VL6180x_laser_focus_register_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static ssize_t set_VL6180x_laser_focus_register_write(struct file *dev, const char *buff, size_t count, loff_t *loff)
{
	uint32_t reg_addr;
	uint16_t reg_val;
	int mode;
	int reg_addr_store;
	int reg_val_store;
	sscanf(buff, "%d %x %x", &mode, &reg_addr_store, &reg_val_store);
	
	reg_val = reg_val_store & 0xFFFF;
    reg_addr = reg_addr_store & 0xFFFFFFFF;
    
    switch(mode){
		case 0:{			
			printk("[LF][vl6180x] %s reg_addr:%x,reg_val:%x", __func__,reg_addr, reg_val);
			if (ASUS_VL6180x_WrWord(reg_addr, reg_val))
            {
                goto failure;
            }
			msleep(30);	
			if (ASUS_VL6180x_RdWord(reg_addr, &reg_val))
            {
                goto failure;
            }
			printk("[LF][vl6180x] %s reg_addr:%x,reg_val:%x", __func__,reg_addr, reg_val);			
			break;
		}
		case 1:{
			
			if (ASUS_VL6180x_WrWord(0x0026, 0x26))
            {
                goto failure;
            }
			if (ASUS_VL6180x_WrByte(0x002d, 0x12))
            {
                goto failure;
            }
			
			break;
		}
		case 2:{
			
			if (ASUS_VL6180x_WrByte(0x0025, 0xff))
            {
                goto failure;
            }
			
			break;
		}
		default:{
		
			break;
		}
	}
	
	return count;

failure:
	printk("[LF][vl6180x] %s failed\n", __func__);
	return 0;
}

static const struct file_operations set_laser_focus_register_fops = {
	.write = set_VL6180x_laser_focus_register_write,
};

static int VL6180x_I2C_laser_device_read(struct seq_file *buf, void *v)
{
    #if 0
    Check_Laser_Sensor();
    if (!camera_on_flag)
    {
        //m10mo_s_power_fac(0); // ISP power down.
    }
	seq_printf(buf, "%d#%d\n", Laser_Device_ID);
	return 0;
    #else
	int val;
	u32 ls_isp_reg24;
	u32 ls_isp_reg1e;

    (void) m10mo_write_fac(2,0x0D,0xA7,246); //laser_sensor_reg1e_calibration
    (void) m10mo_write_fac(1,0x0D,0xA6,0x02); // 2byte read
    (void) m10mo_write_fac(1,0x0D,0xA5,0x01);
    msleep(50);

    (void) m10mo_read_fac(1,0x0D,0xAE,&val);
    if (0x00 == val)
    {
        (void) m10mo_read_fac(2,0x0D,0xA9,&ls_isp_reg1e);
        printk("[vl6180]NVM_Reg1E read:0x%x\n", ls_isp_reg1e);
    }
	else
	{
        goto error;
	}

   	(void)  m10mo_write_fac(2,0x0D,0xA7,245); //laser_sensor_reg24_calibration
    (void)  m10mo_write_fac(1,0x0D,0xA6,0x01); // 1byte read
    (void)  m10mo_write_fac(1,0x0D,0xA5,0x01);
    msleep(50);

    (void)  m10mo_read_fac(1,0x0D,0xAE,&val);
    if (0x00 == val)
    {
        (void)  m10mo_read_fac(1,0x0D,0xA9,&ls_isp_reg24);
        printk("[vl6180]NVM_Reg24 read:0x%x\n", ls_isp_reg24);
    }
	else
	{
        goto error;
	}

	seq_printf(buf, "%d#%d\n", ls_isp_reg1e, ls_isp_reg24);  
	return 0;

error:
	seq_printf(buf, "0\n");
	return 0;
    #endif
    
}
static int VL6180x_I2C_check_laser_device(struct inode *inode, struct  file *file)
{
	return single_open(file, VL6180x_I2C_laser_device_read, NULL);
}
static const struct file_operations I2C_device_check_fops = {
	.owner = THIS_MODULE,
	.open = VL6180x_I2C_check_laser_device,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int VL6180x_I2C_debug_dump_read(struct seq_file *buf, void *v)
{
    #if 0
	int status, i = 0;
	uint16_t register_value = 0;
    VL6180x_RangeData_t RangeData = {0};
    static const int vl6180_reg_table[] =
    {
        0x1E,
        0x24,
        0x4D,
        0x66, 
    };
    #define VL6180_REG_TABLE_SIZE (sizeof(vl6180_reg_table)/sizeof(int))    

	mutex_lock(&vl6180x_mutex);

	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		seq_printf(buf, "%d\n", 0);
		mutex_unlock(&vl6180x_mutex);
		return 0;
		//return -EBUSY;
	}

    status = ATD_VL6180x_device_read_range(&RangeData);
    
    for (i = 0; i <VL6180_REG_TABLE_SIZE; i++)  {
        register_value = 0;
        status = ASUS_VL6180x_RdWord(vl6180_reg_table[i], &register_value);
        if (status < 0) {
            pr_err("%s: read register(0x%x) failed\n", __func__, i);
            return status;
        }
        seq_printf(buf, "register(0x%x) : 0x%x\n", vl6180_reg_table[i], register_value);
    }

    seq_printf(buf, "DMax : %d\n", RangeData.DMax);
    seq_printf(buf, "errorStatus : %d\n", 0);

	mutex_unlock(&vl6180x_mutex);
	return 0;
    #else
	uint16_t reg_data16 = 0;
        uint8_t reg_data8 = 0;
	
	if (vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_OFF ||
		vl6180x_t->device_state == MSM_LASER_FOCUS_DEVICE_DEINIT_CCI) {
		pr_err("%s:%d Device without turn on: (%d) \n", __func__, __LINE__, vl6180x_t->device_state);
		return -EBUSY;
	}
	
	ASUS_VL6180x_RdWord(SYSRANGE_CROSSTALK_COMPENSATION_RATE, &reg_data16);
	seq_printf(buf, "register(0x%x) : 0x%x\n", SYSRANGE_CROSSTALK_COMPENSATION_RATE, reg_data16);
	ASUS_VL6180x_RdByte(SYSRANGE_PART_TO_PART_RANGE_OFFSET, &reg_data8);
	seq_printf(buf, "register(0x%x) : 0x%x\n", SYSRANGE_PART_TO_PART_RANGE_OFFSET, reg_data8);
	ASUS_VL6180x_RdByte(RESULT_RANGE_STATUS, &reg_data8);
	seq_printf(buf, "register(0x%x) : 0x%x\n", RESULT_RANGE_STATUS, reg_data8);
	ASUS_VL6180x_RdWord(RESULT_RANGE_SIGNAL_RATE, &reg_data16);
	seq_printf(buf, "register(0x%x) : 0x%x\n", RESULT_RANGE_SIGNAL_RATE, reg_data16);
	seq_printf(buf, "DMax : %d\n", DMax);
	seq_printf(buf, "errorStatus : %d\n", errorStatus);
	return 0;
    #endif
}
static int VL6180x_I2C_debug_dump(struct inode *inode, struct  file *file)
{
	return single_open(file, VL6180x_I2C_debug_dump_read, NULL);
}
static const struct file_operations I2C_debug_dump_fops = {
	.owner = THIS_MODULE,
	.open = VL6180x_I2C_debug_dump,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static void VL6180x_create_proc_file(void)
{
	status_proc_file = proc_create(STATUS_PROC_FILE, 0776, NULL, &ATD_I2C_status_check_fops);
	if (status_proc_file) {
		printk("%s status_proc_file sucessed!\n", __func__);
	} else {
		printk("%s status_proc_file failed!\n", __func__);
	}
	
	device_trun_on_file = proc_create(DEVICE_TURN_ON_FILE, 0776, NULL, &ATD_laser_focus_device_enable_fops);
	if (device_trun_on_file) {
		printk("%s device_trun_on_file sucessed!\n", __func__);
	} else {
		printk("%s device_trun_on_file failed!\n", __func__);
	}

	device_get_value_file = proc_create(DEVICE_GET_VALUE, 0776, NULL, &ATD_laser_focus_device_get_range_fos);
	if (device_get_value_file) {
		printk("%s device_get_value_file sucessed!\n", __func__);
	} else {
		printk("%s device_get_value_file failed!\n", __func__);
	}
    proc_set_user(device_get_value_file, 1000, 1000);
	
	device_get_more_value_file = proc_create(DEVICE_GET_MORE_VALUE, 0776, NULL, &ATD_laser_focus_device_get_more_value_fos);
	if (device_get_more_value_file) {
		printk("%s device_get_more_value_file sucessed!\n", __func__);
	} else {
		printk("%s device_get_more_value_file failed!\n", __func__);
	}
    proc_set_user(device_get_more_value_file, 1000, 1000);

	device_set_calibration_file = proc_create(DEVICE_SET_CALIBRATION, 0776, NULL, &ATD_laser_focus_device_calibration_fops);
	if (device_set_calibration_file) {
		printk("%s device_set_calibration_file sucessed!\n", __func__);
	} else {
		printk("%s device_set_calibration_file failed!\n", __func__);
	}

	dump_laser_focus_register_file = proc_create(DEVICE_DUMP_REGISTER_VALUE, 0776, NULL, &dump_laser_focus_register_fops);
	if (dump_laser_focus_register_file) {
		printk("%s dump_laser_focus_register_file sucessed!\n", __func__);
	} else {
		printk("%s dump_laser_focus_register_file failed!\n", __func__);
	}
	
	set_laser_focus_register_file = proc_create(DEVICE_SET_REGISTER_VALUE, 0776, NULL, &set_laser_focus_register_fops);
	if (set_laser_focus_register_file) {
		printk("%s set_laser_focus_register_file sucessed!\n", __func__);
	} else {
		printk("%s set_laser_focus_register_file failed!\n", __func__);
	}

	status_proc_file = proc_create(STATUS_PROC_FILE_FOR_CAMERA, 0776, NULL, &I2C_status_check_fops);
	if (status_proc_file) {
		printk("%s status_proc_file sucessed!\n", __func__);
	} else {
		printk("%s status_proc_file failed!\n", __func__);
	}

	check_laser_device_file = proc_create(CHECK_LASER_DEVICE, 0776, NULL, &I2C_device_check_fops);
	if (check_laser_device_file) {
		printk("%s check_laser_device_file sucessed!\n", __func__);
	} else {
		printk("%s check_laser_device_file failed!\n", __func__);
	}

	dump_debug_register_file = proc_create(DEBUG_DUMP, 0776, NULL, &I2C_debug_dump_fops);
	if (dump_debug_register_file) {
		printk("%s dump_debug_register_file sucessed!\n", __func__);
	} else {
		printk("%s dump_debug_register_file failed!\n", __func__);
	}

}                                                                           

static int32_t VL6180x_power_down(struct laser_focus_ctrl_t *a_ctrl)
{
	int32_t rc = 0;
	CDBG("Enter\n");
	if (a_ctrl->laser_focus_state != LASER_FOCUS_POWER_DOWN)
    {
        //rc = m10mo_s_power_fac(0);
    	DBG_LOG("[LF][vl6180x] [ISP]power down  rc:0x%x \n", rc);	
		a_ctrl->laser_focus_state = LASER_FOCUS_POWER_DOWN;
	}
	CDBG("Exit\n");
	return rc;
}
#if 0
static int VL6180x_init(struct laser_focus_ctrl_t *a_ctrl)
{
	return 0;
}

static int VL6180x_deinit(struct laser_focus_ctrl_t *a_ctrl) 
{
	return 0;
}
#endif

static int32_t VL6180x_power_up(struct laser_focus_ctrl_t *a_ctrl)
{
	int rc = 0;
	CDBG("%s called\n", __func__);

	rc = Check_Laser_Sensor();
	if (rc < 0) {
		pr_err("%s failed %d\n", __func__, __LINE__);
		return rc;
	}

	a_ctrl->laser_focus_state = LASER_FOCUS_POWER_UP;

	CDBG("Exit\n");
	return 0;
}

static int32_t VL6180x_platform_probe(struct platform_device *pdev)
{

    CDBG("vl6180 Probe Start\n");
	ATD_status = 0;

	switch (Read_PROJ_ID()) {
		case PROJ_ID_ZX550ML:
			vl6180x_t = kzalloc(sizeof(struct laser_focus_ctrl_t),
				GFP_KERNEL);
			if (!vl6180x_t) {
				pr_err("%s:%d failed no memory\n", __func__, __LINE__);
				return -ENOMEM;
			}

			VL6180x_create_proc_file();
			break;

		default:
			printk("vl6180 not support this platform\n");
			break;
	}//end switch

	CDBG("Probe Success\n");
	return 0;
}



static struct platform_driver laser_focus_platform_driver = {
	.driver = {
		.name = VL6180_NAME,
		.owner = THIS_MODULE,
	},
};

static struct platform_device laser_platform_device = {
        .name   = VL6180_NAME,
};


static int __init vl6180_init(void)
{
	int32_t rc = 0;
	CDBG("vl6180 Enter\n");
	mutex_init(&vl6180x_mutex);
        platform_device_register(&laser_platform_device);
	rc = platform_driver_probe(&laser_focus_platform_driver,
		VL6180x_platform_probe);
	CDBG("%s:%d rc %d\n", __func__, __LINE__, rc);

	return rc;
}

static void __exit vl6180_exit(void)
{
        printk(KERN_INFO "vl6180 Goodbye\n");
	kfree(vl6180x_t);
	platform_driver_unregister(&laser_focus_platform_driver);

}

module_init(vl6180_init);
module_exit(vl6180_exit);

MODULE_DESCRIPTION("vl6180 laser focus driver");
MODULE_AUTHOR("Hugo Lin <hugo_lin@asus.com>");
MODULE_LICENSE("GPL");
