/* drivers/input/touchscreen/ftxxxx_ts.c
*
* FocalTech ftxxxx TouchScreen driver.
*
* Copyright (c) 2014  Focaltech Ltd.
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
//#define DEBUG
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
//#include <mach/irqs.h>
#include <linux/kernel.h>
#include <linux/semaphore.h>
#include <linux/mutex.h>
#include <linux/module.h>
#include <linux/syscalls.h>
#include <linux/unistd.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/string.h>
#include <linux/timer.h>
#include <linux/input/mt.h>
#include <linux/HWVersion.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/proc_fs.h>
#include <asm/intel-mid.h>
#include "ftxxxx_ts.h"
#include "ftxxxx_ex_fun.h"
#include "../../external_drivers/drivers/i2c/busses/i2c-designware-core.h"

//#include <mach/gpio.h>
//#include <mach/map.h>
//#include <mach/regs-clock.h>
//#include <mach/regs-gpio.h>
//#include <plat/gpio-cfg.h>
#include <linux/gpio.h>

#define SYSFS_DEBUG
#define FTS_APK_DEBUG
#define FTS_CTL_IIC 
#define FTS_GESTURE 
//#define FTXXXX_ENABLE_IRQ 

#define ASUS_TOUCH_PROXIMITY_NODE	//<ASUS_Proximity+>

#ifdef FTS_CTL_IIC
#include "focaltech_ctl.h"
#endif

#ifdef FTS_GESTURE
//zax 20140922
//#define  KEY_GESTURE_U	KEY_U
//#define  KEY_GESTURE_UP	KEY_UP
//#define  KEY_GESTURE_DOWN	KEY_DOWN
//#define  KEY_GESTURE_LEFT	KEY_LEFT 
//#define  KEY_GESTURE_RIGHT	KEY_RIGHT
//#define  KEY_GESTURE_O	KEY_O
#define  KEY_GESTURE_E		258
#define  KEY_GESTURE_C		257
//#define  KEY_GESTURE_M	KEY_M 
//#define  KEY_GESTURE_L	KEY_L
#define  KEY_GESTURE_W		261
#define  KEY_GESTURE_S		259
#define  KEY_GESTURE_V		260
#define  KEY_GESTURE_Z		262

//#define GESTURE_LEFT		0x20
//#define GESTURE_RIGHT		0x21
//#define GESTURE_UP		0x22
//#define GESTURE_DOWN		0x23
#define GESTURE_DOUBLECLICK	0x24//0x01
//#define GESTURE_O		0x30
#define GESTURE_W		0x31
//#define GESTURE_M		0x32
#define GESTURE_E		0x33
#define GESTURE_C		0x34
//#define GESTURE_L		0x44
#define GESTURE_S		0x46
#define GESTURE_V		0x54
#define GESTURE_Z		0x65//0x41

#define FTS_GESTURE_POINTS 255
#define FTS_GESTURE_POINTS_ONETIME  62
#define FTS_GESTURE_POINTS_HEADER 8
#define FTS_GESTURE_OUTPUT_ADRESS 0xD3
#define FTS_GESTURE_OUTPUT_UNIT_LENGTH 4

short pointnum = 0;
unsigned short coordinate_x[150] = {0};
unsigned short coordinate_y[150] = {0};

int gesture_id = 0;
extern int dclick_mode;
extern int gesture_mode;
#endif

#ifdef SYSFS_DEBUG
#include "ftxxxx_ex_fun.h"
#endif

extern int glove_mode;
extern int cover_mode;
extern bool keypad_enable;
static int virtual_keys_abs_y = 0;
int sleep_mode = 0;

struct ftxxxx_platform_data ftxxxx_pdata = {
	.gpio_irq = FTXXXX_INT_PIN,
	.gpio_reset = FTXXXX_RESET_PIN,
	.screen_max_x = TOUCH_MAX_X,
	.screen_max_y = TOUCH_MAX_Y,
};

struct ftxxxx_ts_data *check_ts;
//#define TOUCH_MAX_X						0x700
//#define TOUCH_MAX_Y						0x400

#define ANDROID_INPUT_PROTOCOL_B

//#define FTXXXX_RESET_PIN	EXYNOS4_GPK3(3)//EXYNOS4X12_GPM0(1)//EXYNOS4_GPM0(1) //S5PV210_GPB(2)
//#define FTXXXX_RESET_PIN_NAME	"ftxxxx-reset"

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
static ssize_t tp_proximity_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos);
static ssize_t tp_proximity_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos);

static unsigned int touch_proximity_at_phone = 0;

static const struct file_operations tp_proximity_proc_fops = {
	.owner = THIS_MODULE,
	.read = tp_proximity_proc_read,
	.write = tp_proximity_proc_write,
};
#endif
//<ASUS_Proximity->

static void ftxxxx_ts_suspend(struct early_suspend *handler);
static void ftxxxx_ts_resume(struct early_suspend *handler);

/*
*ftxxxx_i2c_Read-read data and write data by i2c
*@client: handle of i2c
*@writebuf: Data that will be written to the slave
*@writelen: How many bytes to write
*@readbuf: Where to store data read from slave
*@readlen: How many bytes to read
*
*Returns negative errno, else the number of messages executed
*
*
*/
int ftxxxx_i2c_Read(struct i2c_client *client, char *writebuf,
	int writelen, char *readbuf, int readlen)
{
	int ret;
	int retry = 0, retrycount = 5;

	if (writelen > 0) {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = 0,
				.len = writelen,
				.buf = writebuf,
			},
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		for (retry = 0; retry < retrycount; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 2);
			if (ret >= 0)
				break;
			msleep(1);
		}
	} else {
		struct i2c_msg msgs[] = {
			{
				.addr = client->addr,
				.flags = I2C_M_RD,
				.len = readlen,
				.buf = readbuf,
			},
		};
		for (retry = 0; retry < retrycount; retry++) {
			ret = i2c_transfer(client->adapter, msgs, 1);
			if (ret >= 0)
				break;
			msleep(1);
		}
	}

	if ((retry == retrycount) && (sleep_mode != 1)) {
		dev_err(&client->dev, "%s: i2c read error. error code = %d\n", __func__, ret);	
		dev_err(&client->dev, "%s: execute IC reset\n", __func__);

		disable_irq_nosync(client->irq);
		ftxxxx_reset_tp(0);
		msleep(10);
		ftxxxx_reset_tp(1);
		msleep(80);
		enable_irq(client->irq);	
	}

	retry = 0;
	return ret;
}
/*write data by i2c*/
int ftxxxx_i2c_Write(struct i2c_client *client, char *writebuf, int writelen)
{
	int ret;
	int retry = 0, retrycount = 5;

	struct i2c_msg msg[] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = writelen,
			.buf = writebuf,
		},
	};

	for (retry = 0; retry < retrycount; retry++) {
		ret = i2c_transfer(client->adapter, msg, 1);
		if (ret >= 0)
			break;
		msleep(1);
	}

	if ((retry == retrycount) && (sleep_mode != 1)) {
		dev_err(&client->dev, "%s: i2c write error. error code = %d\n", __func__, ret);	
		dev_err(&client->dev, "%s: execute IC reset\n", __func__);

		disable_irq_nosync(client->irq);
		ftxxxx_reset_tp(0);
		msleep(10);
		ftxxxx_reset_tp(1);
		msleep(80);
		enable_irq(client->irq);	
	}

	retry = 0;
	return ret;
}

#ifdef FTS_GESTURE//zax 20140922
static void check_gesture(struct ftxxxx_ts_data *data,int gesture_id)
{
	//Tempprintk("[ftxxxx] gesture_id==0x%x\n",gesture_id);

	switch(gesture_id)
	{
		//case GESTURE_LEFT:
		//	input_report_key(data->input_dev, KEY_GESTURE_LEFT, 1);
		//	input_sync(data->input_dev);
		//	input_report_key(data->input_dev, KEY_GESTURE_LEFT, 0);
		//	input_sync(data->input_dev);
		//	break;
		//case GESTURE_RIGHT:
		//	input_report_key(data->input_dev, KEY_GESTURE_RIGHT, 1);
		//	input_sync(data->input_dev);
		//	input_report_key(data->input_dev, KEY_GESTURE_RIGHT, 0);
		//	input_sync(data->input_dev);
		//	break;
		//case GESTURE_UP:
		//	input_report_key(data->input_dev, KEY_GESTURE_UP, 1);
		//	input_sync(data->input_dev);
		//	input_report_key(data->input_dev, KEY_GESTURE_UP, 0);
		//	input_sync(data->input_dev);                                  
		//	break;
		//case GESTURE_DOWN:
		//	input_report_key(data->input_dev, KEY_GESTURE_DOWN, 1);
		//	input_sync(data->input_dev);
		//	input_report_key(data->input_dev, KEY_GESTURE_DOWN, 0);
		//	input_sync(data->input_dev);
		//	break;
		case GESTURE_DOUBLECLICK:
			//input_report_key(data->input_dev, KEY_GESTURE_U, 1);
			//input_sync(data->input_dev);
			//input_report_key(data->input_dev, KEY_GESTURE_U, 0);
			//input_sync(data->input_dev);
			if (dclick_mode == 1) {
				input_report_key(data->input_dev, KEY_WAKEUP, 1);
				input_report_key(data->input_dev, KEY_WAKEUP, 0);
				input_sync(data->input_dev);
			}
			break;
		//case GESTURE_O:
		//	input_report_key(data->input_dev, KEY_GESTURE_O, 1);
		//	input_sync(data->input_dev);
		//	input_report_key(data->input_dev, KEY_GESTURE_O, 0);
		//	input_sync(data->input_dev);
		//	break;
		case GESTURE_W:
			if ((gesture_mode & 0x40) && (gesture_mode & 0x20)) {
				input_report_key(data->input_dev, KEY_GESTURE_W, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_W, 0);
				input_sync(data->input_dev);
			}
			break;
		//case GESTURE_M:
		//	input_report_key(data->input_dev, KEY_GESTURE_M, 1);
		//	input_sync(data->input_dev);
		//	input_report_key(data->input_dev, KEY_GESTURE_M, 0);
		//	input_sync(data->input_dev);
		//	break;
		case GESTURE_E:
			if ((gesture_mode & 0x40) && (gesture_mode & 0x08)) {
				input_report_key(data->input_dev, KEY_GESTURE_E, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_E, 0);
				input_sync(data->input_dev);
			}
			break;
		//case GESTURE_L:
		//	input_report_key(data->input_dev, KEY_GESTURE_L, 1);
		//	input_sync(data->input_dev);
		//	input_report_key(data->input_dev, KEY_GESTURE_L, 0);
		//	input_sync(data->input_dev);
		//	break;
		case GESTURE_S:
			if ((gesture_mode & 0x40) && (gesture_mode & 0x10)) {
				input_report_key(data->input_dev, KEY_GESTURE_S, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_S, 0);
				input_sync(data->input_dev);
			}
			break;
		case GESTURE_V:
			if ((gesture_mode & 0x40) && (gesture_mode & 0x01)) {
				input_report_key(data->input_dev, KEY_GESTURE_V, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_V, 0);
				input_sync(data->input_dev);
			}
			break;
		case GESTURE_Z:
			if ((gesture_mode & 0x40) && (gesture_mode & 0x02)) {
				input_report_key(data->input_dev, KEY_GESTURE_Z, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_Z, 0);
				input_sync(data->input_dev);
			}
			break;
		case GESTURE_C:
			if ((gesture_mode & 0x40) && (gesture_mode & 0x04)) {
				input_report_key(data->input_dev, KEY_GESTURE_C, 1);
				input_sync(data->input_dev);
				input_report_key(data->input_dev, KEY_GESTURE_C, 0);
				input_sync(data->input_dev);
			}
			break;
		default:
			break;
	}
}

static int fts_read_Gesturedata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTURE_POINTS * 3] = { 0 };
	int ret = -1;
	int i = 0;
	buf[0] = 0xd3;
	int gesture_id = 0;

	pointnum = 0;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, FTS_GESTURE_POINTS_HEADER);
	//Tempprintk("[ftxxxx] tpd read FTS_GESTURE_POINTS_HEADER.\n");

	if (ret < 0)
	{
		printk("[ftxxxx] %s read touchdata failed.\n", __func__);
		return ret;
	}

	/* FW */
	//if (fts_updateinfo_curr.CHIP_ID==0x54)
	//{
		gesture_id = buf[0];
		pointnum = (short)(buf[1]) & 0xff;
		buf[0] = 0xd3;
	 
		if((pointnum * 4 + 8)<255)
		{
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));
		}
		else
		{
			ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 255);
			ret = ftxxxx_i2c_Read(data->client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
		}
		if (ret < 0)
		{
			printk( "[ftxxxx] %s read touchdata failed.\n", __func__);
			return ret;
		}
		check_gesture(data,gesture_id);
		for(i = 0;i < pointnum;i++)
		{
			coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
			coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
		}
		return -1;
	//}
/*
	if (0x24 == buf[0])

	{
		gesture_id = 0x24;
		check_gesture(gesture_id);
		return -1;
	}
	
	pointnum = (short)(buf[1]) & 0xff;
	buf[0] = 0xd3;

	if((pointnum * 4 + 8)<255)
	{
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, (pointnum * 4 + 8));
	}
	else
	{
		ret = fts_i2c_Read(i2c_client, buf, 1, buf, 255);
		ret = fts_i2c_Read(i2c_client, buf, 0, buf+255, (pointnum * 4 + 8) -255);
	}
	if (ret < 0)
	{
		printk( "%s read touchdata failed.\n", __func__);
		return ret;
	}

	gesture_id = fetch_object_sample(buf, pointnum);
	check_gesture(gesture_id);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
		8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	}
	return -1;*/
}
#endif

//#ifdef FTS_GESTURE
/*static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	unsigned char buf[FTS_GESTURE_POINTS * 2] = { 0 };   
	int ret = -1;    
	int i = 0;    
	buf[0] = 0xd3; 

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, 8);    
	if (ret < 0) 
	{        
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);        
		return ret;    
	} 

	if (0x24 == buf[0])
	{        
		gesture_id = 0x24;
		printk(KERN_WARNING "The Gesture ID is %x %x \n",gesture_id,pointnum);
		return -1;   
	}

	pointnum = (short)(buf[1]) & 0xff;

	buf[0] = 0xd3;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (pointnum * 4 + 8));    
	/* Read two times*/
	//ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (8));    
	//ret = ftxxxx_i2c_Read(data->client, buf, 0, (buf+8), (8));    

/*	if (ret < 0) 
	{        
		dev_err(&data->client->dev, "%s read touchdata failed.\n", __func__);        
		return ret;    
	}

	gesture_id = fetch_object_sample(buf,pointnum);

	printk(KERN_WARNING "The Gesture ID is %x %x \n",gesture_id,pointnum);

	for(i = 0;i < pointnum;i++)
	{
		coordinate_x[i] =  (((s16) buf[0 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[1 + (4 * i)])& 0xFF);
		coordinate_y[i] = (((s16) buf[2 + (4 * i)]) & 0x0F) <<
			8 | (((s16) buf[3 + (4 * i)]) & 0xFF);
	} 

	return -1;
}*/
//#else

#define PRINTF_TOUCHDATA_EN   1

#if PRINTF_TOUCHDATA_EN == 1

#define TOUCHDATA_GESTURE_ID      1  //gesture ID
#define TOUCHDATA_POINT_NUM_ID    2  //point num

/*****************************************************************************************
Name      :    void Ft_Printf_Touchdata(struct ftxxxx_ts_data *data,u8* readbuf)

Input     :    (struct ftxxxx_ts_data *data,u8* readbuf)

Output    :    none

function  :    

******************************************************************************************/
static int FrameCnt = 0;

int  Ft_Printf_Touchdata(struct ftxxxx_ts_data *data, u8* readbuf)
{
	u8 buf[FT_TOUCH_STEP * 0x0f] = { 0 };
	int ret = -1;
	int i = 0;
	u8 temp;

	temp = readbuf[TOUCHDATA_GESTURE_ID];

	if((temp & 0xf0)!=0xf0)
	{
		return 0;
	}
	else
	{
	       FrameCnt++;
		printk("[FTS Touchdata:%5d] ", FrameCnt);
		for(i=0; i<POINT_READ_BUF; i++)
	     	{
	     		printk("%02X ", readbuf[i]);
	     	}
		printk("\n");
	}

	temp &= 0xf;

	if(temp == 0) 
	{
	    printk("\n\n\n");
	    return 0;
	}
	else
	    printk("\n");

	buf[0] = POINT_READ_BUF;
    
	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, (FT_TOUCH_STEP * temp));

	if (ret < 0) {
		dev_err(&data->client->dev, "%s Second read touchdata failed.\n",
			__func__);
		return ret;
	}
	else
	{
		printk("[FTS Touchdata Ex:%5d] ", FrameCnt);		
		for(i=0; i<(FT_TOUCH_STEP * temp); i++)
	     	{
	     		printk("%02X ", buf[i]);
	     	}
		printk("\n\n\n\n");
	}

}
#else
#define Ft_Printf_Touchdata(x,y)
#endif

/*Read touch point information when the interrupt  is asserted.*/
static int ftxxxx_read_Touchdata(struct ftxxxx_ts_data *data)
{
	struct ts_event *event = &data->event;
	u8 buf[POINT_READ_BUF] = { 0 };
	int ret = -1;
	int i = 0;
	u8 pointid = FT_MAX_ID;

	ret = ftxxxx_i2c_Read(data->client, buf, 1, buf, POINT_READ_BUF);
	if (ret < 0) {
		dev_err(&data->client->dev, "%s read touchdata failed.\n",
			__func__);
		return ret;
	}

	//TempFt_Printf_Touchdata(data,buf);
	
	memset(event, 0, sizeof(struct ts_event));

	event->touch_point = 0;
	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++) {
		pointid = (buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		if (pointid >= FT_MAX_ID)
			break;
		else
			event->touch_point++;
		event->au16_x[i] =
			(((s16) buf[FT_TOUCH_X_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_X_L_POS + FT_TOUCH_STEP * i])& 0xFF);
		event->au16_y[i] =
			(((s16) buf[FT_TOUCH_Y_H_POS + FT_TOUCH_STEP * i]) & 0x0F) <<
			8 | (((s16) buf[FT_TOUCH_Y_L_POS + FT_TOUCH_STEP * i]) & 0xFF);
		event->au8_touch_event[i] =
			buf[FT_TOUCH_EVENT_POS + FT_TOUCH_STEP * i] >> 6;
		event->au8_finger_id[i] =
			(buf[FT_TOUCH_ID_POS + FT_TOUCH_STEP * i]) >> 4;
		event->pressure[i] =
			(buf[FT_TOUCH_XY_POS + FT_TOUCH_STEP * i]);
		event->area[i] =
			(buf[FT_TOUCH_MISC + FT_TOUCH_STEP * i]) >> 4;
#if 0
		pr_info("id=%d event=%d x=%d y=%d\n", event->au8_finger_id[i],
			event->au8_touch_event[i], event->au16_x[i], event->au16_y[i]);
#endif
	}

	//event->pressure = FT_PRESS;	
	//event->pressure = 200;

	return 0;
}
//#endif

static void ftxxxx_free_fingers(struct ftxxxx_ts_data *data)
{
	int i;

	for (i = 0; i < CFG_MAX_TOUCH_POINTS; i++)
	{
		input_mt_slot(data->input_dev, i);
		input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
	}

//	input_report_key(data->input_dev, BTN_TOUCH, 0);
	input_mt_report_pointer_emulation(data->input_dev, false);
//	input_report_key(data->input_dev, BTN_TOOL_FINGER, 0);
//#ifndef TYPE_B_PROTOCOL
//	input_mt_sync(data->input_dev);
//#endif
	input_sync(data->input_dev);
}

/*
*report the point information
*/
static void ftxxxx_report_value(struct ftxxxx_ts_data *data)
{	
	struct ts_event *event = &data->event;
	int i;
	int uppoint = 0;
	//int ret = 0;
	int touchs = 0;


	for (i = 0; i < event->touch_point; i++)
	{
		// dev_info(&data->client->dev, "[ftxxxx] event= %d, x= %d, y= %d virtual@ %d, point= %d\n", event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], virtual_keys_abs_y, event->touch_point);

		if (virtual_keys_abs_y && !keypad_enable && event->au16_y[i] >= virtual_keys_abs_y) {
		    /* Eat the area of the touchscreen used by virtual keys */
		} else if (event->au16_y[i]==2500)
		{
			if(event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				switch(event->au16_x[i])
				{
					case 200:
						//Tempdev_dbg(&data->client->dev, "[ftxxxx] Press BACK KEY\n");
						if (keypad_enable)
							input_report_key(data->input_dev, KEY_BACK, 1);
						break;
					case 540:
						//Tempdev_dbg(&data->client->dev, "[ftxxxx] Press HOME KEY\n");
						if (keypad_enable)
							input_report_key(data->input_dev, KEY_HOME, 1);
						break;
					case 800:
						//Tempdev_dbg(&data->client->dev, "[ftxxxx] Press MENU KEY\n");
						if (keypad_enable)
							input_report_key(data->input_dev, KEY_MENU, 1);
						break;
					default:
						break;
				}
			}
			else
			{
				switch(event->au16_x[i])
				{
					case 200:
						if (keypad_enable)
							input_report_key(data->input_dev, KEY_BACK, 0);
						break;
					case 540:
						if (keypad_enable)
							input_report_key(data->input_dev, KEY_HOME, 0);
						break;
					case 800:
						if (keypad_enable)
							input_report_key(data->input_dev, KEY_MENU, 0);
						break;
					default:
						break;
				}
				uppoint++;
			}
		}
		else
		{
			//protocol B
			//Tempdev_dbg(&data->client->dev, "[ftxxxx] event= %d, x= %d, y= %d, point= %d\n", event->au8_touch_event[i], event->au16_x[i], event->au16_y[i], event->touch_point);
			//Tempdev_dbg(&data->client->dev, "[ftxxxx] finger_id=%d, event->pressure= %d, uppoint= %d\n", event->au8_finger_id[i], event->pressure, uppoint);

			input_mt_slot(data->input_dev, event->au8_finger_id[i]);

			if (event->au8_touch_event[i]== 0 || event->au8_touch_event[i] == 2)
			{
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, true);
				//input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure);
				//input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->pressure);
				input_report_abs(data->input_dev, ABS_MT_PRESSURE, event->pressure[i]);
				input_report_abs(data->input_dev, ABS_MT_TOUCH_MAJOR, event->area[i]);
				input_report_abs(data->input_dev, ABS_MT_POSITION_X, event->au16_x[i]);
				input_report_abs(data->input_dev, ABS_MT_POSITION_Y, event->au16_y[i]);	
				touchs |= BIT(event->au8_finger_id[i]);
				data->touchs |= BIT(event->au8_finger_id[i]);
			}
			else
			{
				uppoint++;
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
				data->touchs &= ~BIT(event->au8_finger_id[i]);
			}
		}
	}

	if(unlikely(data->touchs ^ touchs)){
		for(i = 0; i < CFG_MAX_TOUCH_POINTS; i++){
			if(BIT(i) & (data->touchs ^ touchs)){
				input_mt_slot(data->input_dev, i);
				input_mt_report_slot_state(data->input_dev, MT_TOOL_FINGER, false);
			}
		}
	}
	data->touchs = touchs;

/*
	if(event->touch_point == uppoint)
	{
		input_report_key(data->input_dev, BTN_TOUCH, 0);		
	}
	else
	{
		input_report_key(data->input_dev, BTN_TOUCH, event->touch_point > 0);
	}
*/
	input_mt_report_pointer_emulation(data->input_dev, false);

	input_sync(data->input_dev);
}

/*
*ftxxxx touch work function
*/
static void ftxxxx_ts_work_func(struct work_struct *work)
{
	struct ftxxxx_ts_data *ftxxxx_ts = container_of(work, struct ftxxxx_ts_data, work);
	struct dw_i2c_dev *dev = i2c_get_adapdata(ftxxxx_ts->client->adapter);
	int ret = 0;
	u8 state;
	mutex_lock(&ftxxxx_ts->mutex_lock);

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
	if (touch_proximity_at_phone == 1)
	{
		ftxxxx_read_Touchdata(ftxxxx_ts);
		mutex_unlock(&ftxxxx_ts->mutex_lock);
		enable_irq(ftxxxx_ts->irq);
		return;	
	}
#endif
//<ASUS_Proximity->

#ifdef FTS_GESTURE//zax 20140922

	//<ASUS_i2c_suspend+>
	if ((dclick_mode == 1) || (gesture_mode & 0x40)) {
		if(dev->status & STATUS_SUSPENDED){
			ret = i2c_dw_resume(dev,false);
			if (ret < 0) {
				printk("[ftxxxx] %s: ret = %d, i2c_dw_resume failed\n", __func__, ret);
			}			
			printk("[ftxxxx] unlock i2c suspend status=%d\n", dev->status);	
		}
	}
	//<ASUS_i2c_suspend->
	
	i2c_smbus_read_i2c_block_data(ftxxxx_ts->client, 0xd0, 1, &state);
	//Tempdev_dbg(&ftxxxx_ts->client->dev, "[ftxxxx] tpd fts_read_Gesturedata state=%d\n",state);
	if(state ==1 && ((dclick_mode == 1) || (gesture_mode & 0x40)))
	{
		fts_read_Gesturedata(ftxxxx_ts);
		/*continue;*/
	} else {
#endif

	ret = ftxxxx_read_Touchdata(ftxxxx_ts);
	if (ret == 0)
		ftxxxx_report_value(ftxxxx_ts);
#ifdef FTS_GESTURE/*zax 20140922*/
	}
#endif
	mutex_unlock(&ftxxxx_ts->mutex_lock);

	enable_irq(ftxxxx_ts->irq);

	return;	
}

/*The ftxxxx device will signal the host about TRIGGER_FALLING.
*Processed when the interrupt is asserted.
*/
static irqreturn_t ftxxxx_ts_interrupt(int irq, void *dev_id)
{
	struct ftxxxx_ts_data *ftxxxx_ts = dev_id;

	if(queue_work(ftxxxx_ts->ftxxxx_wq, &ftxxxx_ts->work))
	{
		disable_irq_nosync(ftxxxx_ts->irq);
	}

	return IRQ_HANDLED;
}

void ftxxxx_reset_tp(int HighOrLow)
{
	pr_info("[ftxxxx] set tp reset pin to %d\n", HighOrLow);	
	gpio_set_value(FTXXXX_RESET_PIN, HighOrLow);
}

void ftxxxx_Enable_IRQ(struct i2c_client *client, int enable)
{
	//if (FTXXXX_ENABLE_IRQ == enable)
	//if (FTXXXX_ENABLE_IRQ)
	//enable_irq(client->irq);
	//else
	//disable_irq_nosync(client->irq);
}

static int fts_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	int ret = 0;

	ret = gpio_request(FTXXXX_RESET_PIN, FTXXXX_RESET_PIN_NAME);
	if (ret) {
		pr_err("%s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTXXXX_RESET_PIN_NAME, ret);
		return ret;
	}
		
	//s3c_gpio_setpull(FTXXXX_RESET_PIN, S3C_GPIO_PULL_NONE);  
	//s3c_gpio_cfgpin(FTXXXX_RESET_PIN, S3C_GPIO_SFN(1));//  S3C_GPIO_OUTPUT
	//gpio_set_value(FTXXXX_RESET_PIN,  0);
	gpio_direction_output(FTXXXX_RESET_PIN, 1);	//reset set high
	pr_info("[ftxxxx] gpio_request reset gpio Num: %d\n", FTXXXX_RESET_PIN);

	ret = gpio_request(FTXXXX_INT_PIN, FTXXXX_INT_PIN_NAME);
	if (ret) {
		pr_err("%s: request GPIO %s for reset failed, ret = %d\n",
			__func__, FTXXXX_INT_PIN_NAME, ret);
		return ret;
	}
	gpio_direction_input(FTXXXX_INT_PIN);
	pr_info("[ftxxxx] gpio_request int gpio Num: %d\n", FTXXXX_INT_PIN);

	return ret;
}

//usb_cable_status+
void ftxxxx_usb_detection(bool plugin)
{
	if (check_ts == NULL) {
		printk("[ftxxxx][TOUCH_ERR] %s : ftxxxx_ts is null, skip \n", __func__);
		return;
	}

	
	if (plugin) {
		check_ts->usb_status = 1; /*AC plug in*/
		printk("ftxxxx usb_status = %d\n", check_ts->usb_status);
	}
	else {
		check_ts->usb_status = 0;	/*no AC */
		printk("ftxxxx usb_status = %d\n", check_ts->usb_status);
	}
	queue_work(check_ts->usb_wq, &check_ts->usb_detect_work);
	
	
}

static void ftxxxx_cable_status(struct work_struct *work)
{
	struct ftxxxx_ts_data *ftxxxx_ts = container_of(work, struct ftxxxx_ts_data, usb_detect_work);
	
	uint8_t buf[2] = {0};
	int status = ftxxxx_ts->usb_status;

	wake_lock(&ftxxxx_ts->wake_lock);
	mutex_lock(&ftxxxx_ts->mutex_lock);
	
	printk("[ftxxxx] cable_status=%d.\n", status);

	if (status == 0) {	/*no AC */
		buf[0] = 0x8B;
		buf[1] = 0x00;
		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
	} else if (status == 1) {	/*AC plug in*/
		buf[0] = 0x8B;
		buf[1] = 0x01;
		ftxxxx_write_reg(ftxxxx_ts->client, buf[0], buf[1]);
	}
	

	mutex_unlock(&ftxxxx_ts->mutex_lock);
	wake_unlock(&ftxxxx_ts->wake_lock);

	return;
}
//usb_cable_status-

static void fts_un_init_gpio_hw(struct ftxxxx_ts_data *ftxxxx_ts)
{
	gpio_free(FTXXXX_INT_PIN);
	gpio_free(FTXXXX_RESET_PIN);
}

//<ASUS_virtual_key+>
static ssize_t virtual_keys_show(struct kobject *kobj,  
             struct kobj_attribute *attr, char *buf)  
{
	if (Read_PROJ_ID() == PROJ_ID_ZE550ML || Read_PROJ_ID() == PROJ_ID_ZE551ML || 
		 Read_PROJ_ID() == PROJ_ID_ZE551ML_CKD || Read_PROJ_ID() == PROJ_ID_ZX550ML)
	{
		#if defined(CONFIG_ZS550ML) || defined(CONFIG_ZS570ML)
		if (Read_HW_ID() == HW_ID_MP)
		#else
		if (Read_HW_ID() == HW_ID_MP || (Read_PROJ_ID() == PROJ_ID_ZX550ML && Read_HW_ID() == HW_ID_MP_SD))
		#endif
		{
			if (Read_PROJ_ID() == PROJ_ID_ZE550ML)
			{
				virtual_keys_abs_y = 1341 - 100/2;
				return sprintf(buf,  
					__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":140:1341:180:100"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":360:1341:170:100"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":580:1341:180:100"  
					"\n");
			}
			else if (Read_PROJ_ID() == PROJ_ID_ZE551ML || Read_PROJ_ID() == PROJ_ID_ZE551ML_CKD)
			{
				virtual_keys_abs_y = 2061 - 250/2;
				return sprintf(buf,  
					__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":215:2061:260:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":540:2061:260:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":865:2061:260:250"  
					"\n");
			}
			else if (Read_PROJ_ID() == PROJ_ID_ZX550ML)
			{
				virtual_keys_abs_y = 2061 - 250/2;
				return sprintf(buf,  
					__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":215:2061:260:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":540:2061:260:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":865:2061:260:250"  
					"\n");
			}
		}
		else
		{
			if (Read_PROJ_ID() == PROJ_ID_ZE550ML)
			{
				virtual_keys_abs_y = 1330 - 250/2;
				return sprintf(buf,  
					__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":130:1330:160:100"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":360:1330:170:100"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":590:1330:160:100"  
					"\n");
			}
			else if (Read_PROJ_ID() == PROJ_ID_ZE551ML || Read_PROJ_ID() == PROJ_ID_ZE551ML_CKD)
			{
				virtual_keys_abs_y = 2045 - 250/2;
				return sprintf(buf,  
					__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":160:2045:240:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":520:2045:250:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":880:2045:240:250"  
					"\n");
			}
			else if (Read_PROJ_ID() == PROJ_ID_ZX550ML)
			{
				virtual_keys_abs_y = 2061 - 250/2;
				return sprintf(buf,  
					__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":215:2061:260:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":540:2061:260:250"  
					"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":865:2061:260:250"  
					"\n");
			}
		}
	}
	else if (Read_PROJ_ID() == PROJ_ID_ZS570ML || Read_PROJ_ID() == PROJ_ID_ZS571ML)
	{
		virtual_keys_abs_y = 2045 - 250/2;
		return sprintf(buf,  
				__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":160:2045:240:250"  
				"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":520:2045:250:250"  
				"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":880:2045:240:250"  
				"\n");
	}
	else
	{
		//default ZE551ML virtual key position.
		virtual_keys_abs_y = 2045 - 250/2;
		return sprintf(buf,  
			__stringify(EV_KEY) ":" __stringify(KEY_BACK) ":160:2045:240:250"  
			"\n" __stringify(EV_KEY) ":" __stringify(KEY_HOME) ":520:2045:250:250"  
			"\n" __stringify(EV_KEY) ":" __stringify(KEY_MENU) ":880:2045:240:250"  
			"\n");		
	}
}  
  
static struct kobj_attribute virtual_keys_attr = {  
	.attr = {  
		.name = "virtualkeys.ftxxxx_ts",  
		.mode = S_IRUGO,  
	},  
	.show = &virtual_keys_show,  
};  
 
static struct attribute *properties_attrs[] = {  
	&virtual_keys_attr.attr,  
	NULL  
};  
  
static struct attribute_group properties_attr_group = {  
	.attrs = properties_attrs,  
};  
  
static void virtual_keys_init(void)  
{  
	int ret;  

	struct kobject *properties_kobj;  
	properties_kobj = kobject_create_and_add("board_properties", NULL);  

	if (properties_kobj)  
		ret = sysfs_create_group(properties_kobj, 
		&properties_attr_group);  
 
	if (!properties_kobj || ret)  
		pr_err("failed to create board_properties\n");      
}  
//<ASUS_virtual_key->
//<-- ASUS-Bevis_Chen + -->

#ifdef CONFIG_ASUS_FACTORY_MODE

struct proc_dir_entry * touch_entry = NULL;
static int touch_proc_show(struct seq_file *m, void *v) {

     if(! touch_entry)

     return seq_printf(m, "-1\n");

     else

     return seq_printf(m, "1\n");

}

static int  touch_proc_open(struct inode *inode, struct  file *file) {

  return single_open(file, touch_proc_show, NULL);

}

static const struct file_operations  touch_proc_fops = {

  .owner = THIS_MODULE,

  .open =  touch_proc_open,

  .read = seq_read,

  .llseek = seq_lseek,

  .release = single_release,

};

int create_asusproc_touch_status_entry( void )

{
   touch_entry = proc_create("asus_tp_status", S_IWUGO| S_IRUGO, NULL,& touch_proc_fops);

    if (! touch_entry)

        return -ENOMEM;
	
    return 0;

}

#endif

//<-- ASUS-Bevis_Chen - -->

//<ASUS_SDev+>
static ssize_t touch_switch_name(struct switch_dev *sdev, char *buf)
{
	struct ftxxxx_ts_data *ftxxxx_ts = container_of(sdev, struct ftxxxx_ts_data, touch_sdev);

	return sprintf(buf, "5446_0x%x_0x%x\n",ftxxxx_ts->vendor_id, ftxxxx_ts->fw_ver);
}
//<ASUS_SDev->

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
static ssize_t tp_proximity_proc_read(struct file *file, char __user *buf, size_t count, loff_t *ppos)
	{
	char *str;
	int len;

	if (touch_proximity_at_phone == 1) {		//No Touch
		printk("[ftxxxx] Touch is disabled now\n");
		str = "Touch is disabled now\n";
	} else if (touch_proximity_at_phone == 0) {	//Touch
		printk("[ftxxxx] Touch is enabled now\n");
		str = "Touch is enabled now\n";
	}

	len = strlen(str);
	copy_to_user(buf, str, len);
	if (*ppos == 0)
 		*ppos += len;
	else
		len = 0;
	return len;
}
static ssize_t tp_proximity_proc_write(struct file *file, const char __user *buf, size_t count, loff_t *ppos)
{
	char str[128];
	if (count > PAGE_SIZE) //procfs write and read has PAGE_SIZE limit
		count = 128;

        if (copy_from_user(str, buf, count))
	{
		printk("copy_from_user failed!\n");
		return -EFAULT;
        }

	if (count > 1)
	{
		str[count-1] = '\0';
	}

	if ((int)(str[0]) == (1+48)) {		//No Touch
		touch_proximity_at_phone = 1;
		msleep(50);
		ftxxxx_free_fingers(check_ts);
		printk("[ftxxxx] Disable Touch & release pointer\n");
	} else {				//Touch
		touch_proximity_at_phone = 0;
		printk("[ftxxxx] Enable Touch\n");
	}

	return count;
}
#endif
//<ASUS_Proximity->

static int ftxxxx_ts_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	//struct ftxxxx_platform_data *pdata = (struct ftxxxx_platform_data *)client->dev.platform_data;
	struct ftxxxx_platform_data *pdata = &ftxxxx_pdata;
	struct ftxxxx_ts_data *ftxxxx_ts;
	struct input_dev *input_dev;
	int err = 0;
	unsigned char uc_reg_value;
	unsigned char uc_reg_addr;	
	int flag=0;
#ifdef ASUS_TOUCH_PROXIMITY_NODE
	struct proc_dir_entry *tp_proximity_proc; //<ASUS_Proximity+>
#endif

	printk("[ftxxxx] %s start\n", __func__);
	printk("[ftxxxx] Read_PROJ_ID: %x\n", Read_PROJ_ID());

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		printk(KERN_ERR "[ftxxxx] %s: i2c check functionality error\n", __func__);
		err = -ENODEV;
		goto exit_check_functionality_failed;
	}

	//uc_reg_addr = 0x9f;
	//flag = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	//if (flag<0) {
	//	printk(KERN_ERR "[ftxxxx] %s: i2c test failed\n", __func__);
	//	err = -ENODEV;
	//	goto exit_check_functionality_failed;
	//}

	ftxxxx_ts = kzalloc(sizeof(struct ftxxxx_ts_data), GFP_KERNEL);

	if (!ftxxxx_ts) {
		printk(KERN_ERR "[ftxxxx] %s: alloc data failed\n", __func__);
		err = -ENOMEM;
		goto exit_alloc_data_failed;
	}

	ftxxxx_ts->ftxxxx_wq = create_singlethread_workqueue("ftxxxx_wq");
	if (!ftxxxx_ts->ftxxxx_wq) 
	{
		printk(KERN_ERR "\n[ftxxxx] %s: create workqueue failed\n", __func__);
		err = -ENOMEM;
		goto exit_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->work, ftxxxx_ts_work_func);

	printk("\n[ftxxxx] Create workqueue success\n");

	
	ftxxxx_ts->usb_wq = create_singlethread_workqueue("focal_usb_wq");
	if (!ftxxxx_ts->usb_wq) 
	{
		printk(KERN_ERR "\n[ftxxxx] %s: create usb detect workqueue failed\n", __func__);
		err = -ENOMEM;
		goto exit_create_wq_failed;
	}
	INIT_WORK(&ftxxxx_ts->usb_detect_work, ftxxxx_cable_status);
	printk("\n[ftxxxx] Create usb detect workqueue success\n");
	
	mutex_init(&ftxxxx_ts->mutex_lock);
	wake_lock_init(&ftxxxx_ts->wake_lock, WAKE_LOCK_SUSPEND, "ftxxxx_wake_lock");

	i2c_set_clientdata(client, ftxxxx_ts);

	ftxxxx_ts->irq = pdata->gpio_irq;
	ftxxxx_ts->client = client;
	ftxxxx_ts->suspend_flag = 0;
	ftxxxx_ts->usb_status = 0;
	ftxxxx_ts->pdata = pdata;
	if (Read_PROJ_ID() == PROJ_ID_ZE551ML || Read_PROJ_ID() == PROJ_ID_ZX550ML || Read_PROJ_ID() == PROJ_ID_ZE551ML_CKD)
	{
		ftxxxx_ts->x_max = 1080 - 1;
		ftxxxx_ts->y_max = 1920 - 1;
		if(0 >= ftxxxx_ts->x_max) ftxxxx_ts->x_max = 1080;
		if(0 >= ftxxxx_ts->y_max) ftxxxx_ts->y_max = 1920;
	}
	else if (Read_PROJ_ID() == PROJ_ID_ZS570ML || Read_PROJ_ID() == PROJ_ID_ZS571ML)
	{
		ftxxxx_ts->x_max = 1080 - 1;
		ftxxxx_ts->y_max = 1920 - 1;
		if(0 >= ftxxxx_ts->x_max) ftxxxx_ts->x_max = 1080;
		if(0 >= ftxxxx_ts->y_max) ftxxxx_ts->y_max = 1920;
	}
	else
	{
		ftxxxx_ts->x_max = pdata->screen_max_x - 1;
		ftxxxx_ts->y_max = pdata->screen_max_y - 1;
		if(0 >= ftxxxx_ts->x_max) ftxxxx_ts->x_max = TOUCH_MAX_X;
		if(0 >= ftxxxx_ts->y_max) ftxxxx_ts->y_max = TOUCH_MAX_Y;
	}
	pr_info("[ftxxxx] max x= %d, max y= %d, gpio irq = %d,  ts irq = %d\n", 
		ftxxxx_ts->x_max,  ftxxxx_ts->y_max,  pdata->gpio_irq, ftxxxx_ts->irq);	
	ftxxxx_ts->pdata->gpio_reset = FTXXXX_RESET_PIN;
	ftxxxx_ts->pdata->gpio_irq = ftxxxx_ts->irq;
	client->irq = gpio_to_irq(ftxxxx_ts->irq);
	//client->irq = IRQ_EINT(13);
#ifdef FTS_GESTURE
	irq_set_irq_wake(client->irq, 1);
#endif
	
	pr_info("[ftxxxx] irq = %d\n", client->irq);

	if(fts_init_gpio_hw(ftxxxx_ts)<0)
		goto exit_init_gpio;	
#ifdef CONFIG_PM
	printk("[ftxxxx] config_pm exit\n");
#if 0
	err = gpio_request(pdata->gpio_reset, "ftxxxx reset");
	if (err < 0) {
		dev_err(&client->dev, "%s:failed to set gpio reset.\n",
			__func__);
		goto exit_request_reset;
	}
#endif
#endif

	err = request_threaded_irq(client->irq, NULL, ftxxxx_ts_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_ONESHOT, client->dev.driver->name,
		ftxxxx_ts);
	if (err < 0) {
		dev_err(&client->dev, "ftxxxx_probe: request irq failed\n");
		goto exit_irq_request_failed;
	}
	disable_irq(client->irq);

	input_dev = input_allocate_device();
	if (!input_dev) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate input device\n");
		goto exit_input_dev_alloc_failed;
	}

	ftxxxx_ts->input_dev = input_dev;

	if (Read_PROJ_ID() == PROJ_ID_ZE550ML || Read_PROJ_ID() == PROJ_ID_ZE551ML || Read_PROJ_ID() == PROJ_ID_ZX550ML || Read_PROJ_ID() == PROJ_ID_ZE551ML_CKD
		|| Read_PROJ_ID() == PROJ_ID_ZS570ML || Read_PROJ_ID() == PROJ_ID_ZS571ML)
	{
		virtual_keys_init();	//<ASUS_virtual_key+>
	}

	set_bit(KEY_BACK, input_dev->keybit);
	set_bit(KEY_HOME, input_dev->keybit);
	set_bit(KEY_MENU, input_dev->keybit);
	set_bit(KEY_POWER, input_dev->keybit);

	__set_bit(EV_ABS, input_dev->evbit);
	__set_bit(EV_KEY, input_dev->evbit);
	__set_bit(BTN_TOUCH, input_dev->keybit);
	__set_bit(INPUT_PROP_DIRECT, input_dev->propbit);

	input_mt_init_slots(input_dev, CFG_MAX_TOUCH_POINTS, 0);
	input_set_abs_params(input_dev, ABS_MT_TOUCH_MAJOR, 0, PRESS_MAX, 0, 0);
	input_set_abs_params(input_dev, ABS_MT_POSITION_X, 0, ftxxxx_ts->x_max, 0, 0);//ftxxxx_ts->x_max
 	input_set_abs_params(input_dev, ABS_MT_POSITION_Y, 0, ftxxxx_ts->y_max, 0, 0);//ftxxxx_ts->y_max
	input_set_abs_params(input_dev, ABS_MT_PRESSURE, 0, PRESS_MAX, 0, 0);	

	input_dev->name = FTXXXX_NAME;
	err = input_register_device(input_dev);
	if (err) {
		dev_err(&client->dev,
			"ftxxxx_ts_probe: failed to register input device: %s\n",
			dev_name(&client->dev));
		goto exit_input_register_device_failed;
	}
	/*make sure CTP already finish startup process */
	msleep(200);

#ifdef CONFIG_HAS_EARLYSUSPEND
	ftxxxx_ts->early_suspend.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN + 1;
	ftxxxx_ts->early_suspend.suspend = ftxxxx_ts_suspend;
	ftxxxx_ts->early_suspend.resume = ftxxxx_ts_resume;
	register_early_suspend(&ftxxxx_ts->early_suspend);
#endif
	
#ifdef SYSFS_DEBUG
	ftxxxx_create_sysfs(client);
#endif
#ifdef FTS_CTL_IIC
	if (ft_rw_iic_drv_init(client) < 0)
		dev_err(&client->dev, "%s:[FTS] create fts control iic driver failed\n", __func__);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_create_apk_debug_channel(client);
#endif

	fts_ctpm_auto_upgrade(client);

	/*get some register information */
	uc_reg_addr = FTXXXX_REG_FW_VER;
	flag = ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] Firmware version = 0x%x\n", uc_reg_value);
	ftxxxx_ts->fw_ver = uc_reg_value;
	if(flag<0)
	{
		printk("[ftxxxx] chip does not exits\n");
		goto exit_with_out_chip;
		
	}
	//else printk("[ftxxxx] chip exits\n");

	uc_reg_addr = FTXXXX_REG_POINT_RATE;
	ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] report rate is %dHz.\n", uc_reg_value * 10);

	uc_reg_addr = FTXXXX_REG_THGROUP;
	ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] touch threshold is %d.\n", uc_reg_value * 4);

	uc_reg_addr = FTXXXX_REG_VENDOR_ID;
	ftxxxx_i2c_Read(client, &uc_reg_addr, 1, &uc_reg_value, 1);
	pr_info("[FTS] VENDOR ID = 0x%x\n", uc_reg_value);
	ftxxxx_ts->vendor_id = uc_reg_value;
	
#ifdef FTS_GESTURE
	//init_para(720,1280,100,0,0);	

	//auc_i2c_write_buf[0] = 0xd0;
	//auc_i2c_write_buf[1] = 0x01;
	//ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw open gesture function

	//auc_i2c_write_buf[0] = 0xd1;
	//auc_i2c_write_buf[1] = 0xff;
	//ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);

	//auc_i2c_write_buf[0] = 0xd2;//d a g c e m w o gesture
	//auc_i2c_write_buf[1] = 0xff;
	//ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);
	/*
	auc_i2c_write_buf[0] = 0xd0;
	auc_i2c_write_buf[1] = 0x00;
	ftxxxx_i2c_Write(client, auc_i2c_write_buf, 2);	//let fw close gesture function 
	*/
	input_set_capability(input_dev, EV_KEY, KEY_WAKEUP);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_U); 
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_UP); 
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_DOWN);
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_LEFT); 
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_RIGHT); 
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_O);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_E); 
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_M); 
	//input_set_capability(input_dev, EV_KEY, KEY_GESTURE_L);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_W);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_S); 
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_V);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_Z);
	input_set_capability(input_dev, EV_KEY, KEY_GESTURE_C);

        __set_bit(KEY_WAKEUP, input_dev->keybit);
	//__set_bit(KEY_GESTURE_RIGHT, input_dev->keybit);
	//__set_bit(KEY_GESTURE_LEFT, input_dev->keybit);
	//__set_bit(KEY_GESTURE_UP, input_dev->keybit);
	//__set_bit(KEY_GESTURE_DOWN, input_dev->keybit);
	//__set_bit(KEY_GESTURE_U, input_dev->keybit);
	//__set_bit(KEY_GESTURE_O, input_dev->keybit);
	__set_bit(KEY_GESTURE_E, input_dev->keybit);
	//__set_bit(KEY_GESTURE_M, input_dev->keybit);
	__set_bit(KEY_GESTURE_W, input_dev->keybit);
	//__set_bit(KEY_GESTURE_L, input_dev->keybit);
	__set_bit(KEY_GESTURE_S, input_dev->keybit);
	__set_bit(KEY_GESTURE_V, input_dev->keybit);
	__set_bit(KEY_GESTURE_Z, input_dev->keybit);
	__set_bit(KEY_GESTURE_C, input_dev->keybit);
#endif

//<ASUS_SDev+>
	ftxxxx_ts->touch_sdev.name = "touch";
	ftxxxx_ts->touch_sdev.print_name = touch_switch_name;
	if(switch_dev_register(&ftxxxx_ts->touch_sdev) < 0){
		printk("switch_dev_register failed!\n");
	}
//<ASUS_SDev->

	//Register initial
	ftxxxx_write_reg(client, 0xc0, 0x00);
	ftxxxx_write_reg(client, 0xc1, 0x00);
	ftxxxx_write_reg(client, 0xc3, 0x00);
	ftxxxx_write_reg(client, 0xd0, 0x00);
	ftxxxx_write_reg(client, 0xd1, 0x30);
	ftxxxx_write_reg(client, 0xd2, 0x1A);
	ftxxxx_write_reg(client, 0xd5, 0x40);
	ftxxxx_write_reg(client, 0xd6, 0x10);
	ftxxxx_write_reg(client, 0xd7, 0x20);
	ftxxxx_write_reg(client, 0xd8, 0x00);
	if (Read_PROJ_ID() == PROJ_ID_ZE550ML)
	{
		ftxxxx_write_reg(client, 0xda, 0x64);
		ftxxxx_write_reg(client, 0xdb, 0x00);
		ftxxxx_write_reg(client, 0xdc, 0x00);
		ftxxxx_write_reg(client, 0xdd, 0x05);
		ftxxxx_write_reg(client, 0xde, 0x64);
		ftxxxx_write_reg(client, 0xdf, 0x00);
		ftxxxx_write_reg(client, 0xe0, 0x6c);
		ftxxxx_write_reg(client, 0xe1, 0x02);
		ftxxxx_write_reg(client, 0xe2, 0x20);
		ftxxxx_write_reg(client, 0xe3, 0x10);
		ftxxxx_write_reg(client, 0xe4, 0x3e);
		ftxxxx_write_reg(client, 0xe5, 0x06);
	}
	else if (Read_PROJ_ID() == PROJ_ID_ZE551ML || Read_PROJ_ID() == PROJ_ID_ZE551ML_CKD)
	{
		ftxxxx_write_reg(client, 0xda, 0x96);
		ftxxxx_write_reg(client, 0xdb, 0x00);
		ftxxxx_write_reg(client, 0xdc, 0x80);
		ftxxxx_write_reg(client, 0xdd, 0x07);
		ftxxxx_write_reg(client, 0xde, 0x96);
		ftxxxx_write_reg(client, 0xdf, 0x00);
		ftxxxx_write_reg(client, 0xe0, 0xa2);
		ftxxxx_write_reg(client, 0xe1, 0x03);
		ftxxxx_write_reg(client, 0xe2, 0x30);
		ftxxxx_write_reg(client, 0xe3, 0x18);
		ftxxxx_write_reg(client, 0xe4, 0x3e);
		ftxxxx_write_reg(client, 0xe5, 0x06);
	}
	else if (Read_PROJ_ID() == PROJ_ID_ZS570ML || Read_PROJ_ID() == PROJ_ID_ZS571ML)
	{
		ftxxxx_write_reg(client, 0xda, 0x96);
		ftxxxx_write_reg(client, 0xdb, 0x00);
		ftxxxx_write_reg(client, 0xdc, 0x80);
		ftxxxx_write_reg(client, 0xdd, 0x07);
		ftxxxx_write_reg(client, 0xde, 0x96);
		ftxxxx_write_reg(client, 0xdf, 0x00);
		ftxxxx_write_reg(client, 0xe0, 0xa2);
		ftxxxx_write_reg(client, 0xe1, 0x03);
		ftxxxx_write_reg(client, 0xe2, 0x30);
		ftxxxx_write_reg(client, 0xe3, 0x18);
		ftxxxx_write_reg(client, 0xe4, 0x3e);
		ftxxxx_write_reg(client, 0xe5, 0x06);
	}
	//Jeffery+++
	//add global variable check_ts for P-sensor checking
	check_ts = ftxxxx_ts;
	//Jeffery---
//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
	tp_proximity_proc = proc_create("asus_touch_proximity_status", 0664, NULL, &tp_proximity_proc_fops);

	if (!tp_proximity_proc) {
		dev_err(&client->dev,
				"%s: Failed to create proc proximity node\n",
				__func__);
		goto exit_with_out_chip;
	}
#endif
//<ASUS_Proximity->

	enable_irq(client->irq);

	printk("[ftxxxx] %s end\n", __func__);
#ifdef CONFIG_ASUS_FACTORY_MODE
		printk( " Gemini_jia asus_touch_status_factory");
	create_asusproc_touch_status_entry();
#endif

	return 0;
	
exit_with_out_chip:
#ifdef CONFIG_HAS_EARLYSUSPEND
	printk("[ftxxxx] free early suspend\n");
    unregister_early_suspend(&ftxxxx_ts->early_suspend);
#endif

exit_input_register_device_failed:
	input_free_device(input_dev);

exit_input_dev_alloc_failed:
	free_irq(client->irq, ftxxxx_ts);
#ifdef CONFIG_PM
#if 0
exit_request_reset:
	gpio_free(ftxxxx_ts->pdata->reset);
#endif
#endif

exit_irq_request_failed:
exit_init_gpio:
	fts_un_init_gpio_hw(ftxxxx_ts);

	i2c_set_clientdata(client, NULL);
	if (ftxxxx_ts->ftxxxx_wq)
	{
		destroy_workqueue(ftxxxx_ts->ftxxxx_wq);
	}
	if(ftxxxx_ts->usb_wq){
		destroy_workqueue(ftxxxx_ts->usb_wq);
	}
	mutex_destroy(&ftxxxx_ts->mutex_lock);
	wake_lock_destroy(&ftxxxx_ts->wake_lock);

exit_create_wq_failed:	
	kfree(ftxxxx_ts);

exit_alloc_data_failed:
exit_check_functionality_failed:
	return err;
}

#ifdef CONFIG_PM
static void ftxxxx_ts_suspend(struct early_suspend *handler)
{
	struct ftxxxx_ts_data *ts = container_of(handler, struct ftxxxx_ts_data, early_suspend);

	pr_info("[ftxxxx] ftxxxx ts early suspend\n");

	wake_lock(&ts->wake_lock);
	mutex_lock(&ts->mutex_lock);

	if (ts->suspend_flag) {
		pr_info("[ftxxxx] IC in suspend\n");
		wake_lock(&ts->wake_lock);
		mutex_lock(&ts->mutex_lock);
		return;
	}
	
	#ifdef FTS_GESTURE//zax 20140922
	if ((dclick_mode == 1) || (gesture_mode & 0x40)) {
		pr_info("[ftxxxx] ftxxxx ts early suspend in gesture mode\n");
		ftxxxx_write_reg(ts->client, 0xd0, 0x01);
		ftxxxx_write_reg(ts->client, 0xd1, 0x30);
		ftxxxx_write_reg(ts->client, 0xd2, 0x1A);
		ftxxxx_write_reg(ts->client, 0xd5, 0x40);
		ftxxxx_write_reg(ts->client, 0xd6, 0x10);
		ftxxxx_write_reg(ts->client, 0xd7, 0x20);
		ftxxxx_write_reg(ts->client, 0xd8, 0x00);
		/*if (fts_updateinfo_curr.CHIP_ID==0x54)
		{
			ftxxxx_write_reg(ts->client, 0xd1, 0xff);
			ftxxxx_write_reg(ts->client, 0xd2, 0xff);
			ftxxxx_write_reg(ts->client, 0xd5, 0xff);
			ftxxxx_write_reg(ts->client, 0xd6, 0xff);
			ftxxxx_write_reg(ts->client, 0xd7, 0xff);
			ftxxxx_write_reg(ts->client, 0xd8, 0xff);
		}*/
	} else {
	#endif
	disable_irq_nosync(ts->pdata->gpio_irq);
	ftxxxx_write_reg(ts->client,0xa5,0x03);
	msleep(20);
	//sleep mode
	sleep_mode = 1;
	#ifdef FTS_GESTURE//zax 20140922
	}
	#endif

	ftxxxx_free_fingers(ts);

	ts->suspend_flag = 1;

	mutex_unlock(&ts->mutex_lock);
	wake_unlock(&ts->wake_lock);
	return;
}

static void ftxxxx_ts_resume(struct early_suspend *handler)
{
	struct ftxxxx_ts_data *ts = container_of(handler, struct ftxxxx_ts_data, early_suspend);

	pr_info("[ftxxxx] ftxxxx ts late resume.\n");
	
	wake_lock(&ts->wake_lock);
	mutex_lock(&ts->mutex_lock);

	if (!ts->suspend_flag) {
		pr_info("[ftxxxx] IC did not enter suspend\n");
		mutex_unlock(&ts->mutex_lock);
		wake_unlock(&ts->wake_lock);
		return;
	}

	#ifdef FTS_GESTURE//zax 20140922
	if ((dclick_mode == 1) || (gesture_mode & 0x40)) {
		pr_info("[ftxxxx] ftxxxx ts late resume in gesture mode\n");
		gpio_set_value(ts->pdata->gpio_reset, 0);
		msleep(10);
		gpio_set_value(ts->pdata->gpio_reset, 1);
		msleep(80);
		ftxxxx_write_reg(ts->client,0xD0,0x00);
		if (glove_mode == 1) {
			ftxxxx_write_reg(ts->client, 0xC0, 1);
		}
		if (cover_mode == 1) {
			ftxxxx_write_reg(ts->client, 0xC1, 1);
			ftxxxx_write_reg(ts->client, 0xC3, 2);
		} else {
			ftxxxx_write_reg(ts->client, 0xC1, 0);
			ftxxxx_write_reg(ts->client, 0xC3, 0);
		}
	} else {
	#endif
	gpio_set_value(ts->pdata->gpio_reset, 0);
	msleep(10);
	gpio_set_value(ts->pdata->gpio_reset, 1);
	msleep(80);
	//Restore pre-set configuration
	//glove mode
	if (glove_mode == 1) {
		ftxxxx_write_reg(ts->client, 0xC0, 1);
	}
/*	
	//gesture mode
	ftxxxx_write_reg(ts->client, 0xd1, 0x30);
	ftxxxx_write_reg(ts->client, 0xd2, 0x1A);
	ftxxxx_write_reg(ts->client, 0xd5, 0x40);
	ftxxxx_write_reg(ts->client, 0xd6, 0x10);
	ftxxxx_write_reg(ts->client, 0xd7, 0x20);
	ftxxxx_write_reg(ts->client, 0xd8, 0x00);
	if (Read_PROJ_ID() == PROJ_ID_ZE550ML)
	{
		ftxxxx_write_reg(ts->client, 0xda, 0x64);
		ftxxxx_write_reg(ts->client, 0xdb, 0x00);
		ftxxxx_write_reg(ts->client, 0xdc, 0x40);
		ftxxxx_write_reg(ts->client, 0xdd, 0x05);
		ftxxxx_write_reg(ts->client, 0xde, 0x64);
		ftxxxx_write_reg(ts->client, 0xdf, 0x00);
		ftxxxx_write_reg(ts->client, 0xe0, 0x6c);
		ftxxxx_write_reg(ts->client, 0xe1, 0x02);
		ftxxxx_write_reg(ts->client, 0xe2, 0x20);
		ftxxxx_write_reg(ts->client, 0xe3, 0x10);
		ftxxxx_write_reg(ts->client, 0xe4, 0x3e);
		ftxxxx_write_reg(ts->client, 0xe5, 0x06);
	}
	else if (Read_PROJ_ID() == PROJ_ID_ZE551ML  || Read_PROJ_ID() == PROJ_ID_ZE551ML_CKD)
	{
		ftxxxx_write_reg(ts->client, 0xda, 0x96);
		ftxxxx_write_reg(ts->client, 0xdb, 0x00);
		ftxxxx_write_reg(ts->client, 0xdc, 0xe0);
		ftxxxx_write_reg(ts->client, 0xdd, 0x07);
		ftxxxx_write_reg(ts->client, 0xde, 0x96);
		ftxxxx_write_reg(ts->client, 0xdf, 0x00);
		ftxxxx_write_reg(ts->client, 0xe0, 0xa2);
		ftxxxx_write_reg(ts->client, 0xe1, 0x03);
		ftxxxx_write_reg(ts->client, 0xe2, 0x30);
		ftxxxx_write_reg(ts->client, 0xe3, 0x18);
		ftxxxx_write_reg(ts->client, 0xe4, 0x3e);
		ftxxxx_write_reg(ts->client, 0xe5, 0x06);
	}
*/	
	//cover mode
	if (cover_mode == 1) {
		ftxxxx_write_reg(ts->client, 0xC1, 1);
		ftxxxx_write_reg(ts->client, 0xC3, 2);
	} else {
		ftxxxx_write_reg(ts->client, 0xC1, 0);
		ftxxxx_write_reg(ts->client, 0xC3, 0);
	}
	//sleep mode
	sleep_mode = 0;
	enable_irq(ts->pdata->gpio_irq);
	#ifdef FTS_GESTURE//zax 20140922
	}
	#endif
	
	ts->suspend_flag = 0;

	mutex_unlock(&ts->mutex_lock);
	wake_unlock(&ts->wake_lock);
	return;
}
#else
#define ftxxxx_ts_suspend	    NULL
#define ftxxxx_ts_resume		NULL
#endif

static int __devexit ftxxxx_ts_remove(struct i2c_client *client)
{
	struct ftxxxx_ts_data *ftxxxx_ts;
	ftxxxx_ts = i2c_get_clientdata(client);
	input_unregister_device(ftxxxx_ts->input_dev);

#ifdef CONFIG_PM
	gpio_free(ftxxxx_ts->pdata->gpio_reset);
#endif
#ifdef FTS_CTL_IIC
	ft_rw_iic_drv_exit();
#endif
#ifdef SYSFS_DEBUG
	ftxxxx_remove_sysfs(client);
#endif
#ifdef FTS_APK_DEBUG
	ftxxxx_release_apk_debug_channel();
#endif

	fts_un_init_gpio_hw(ftxxxx_ts);

	free_irq(client->irq, ftxxxx_ts);

	if (ftxxxx_ts->ftxxxx_wq)
	{
		destroy_workqueue(ftxxxx_ts->ftxxxx_wq);
	}

	mutex_destroy(&ftxxxx_ts->mutex_lock);

	kfree(ftxxxx_ts);
	i2c_set_clientdata(client, NULL);

//<ASUS_Proximity+>
#ifdef ASUS_TOUCH_PROXIMITY_NODE
	proc_remove("asus_touch_proximity_status");
#endif
//<ASUS_Proximity->

	return 0;
}

static const struct i2c_device_id ftxxxx_ts_id[] = {
	{FTXXXX_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ftxxxx_ts_id);

static struct i2c_driver ftxxxx_ts_driver = {
	.probe = ftxxxx_ts_probe,
	.remove = __devexit_p(ftxxxx_ts_remove),
	.id_table = ftxxxx_ts_id,
//	.suspend = ftxxxx_ts_suspend,
//	.resume = ftxxxx_ts_resume,
	.driver = {
		.name = FTXXXX_NAME,
		.owner = THIS_MODULE,
	},
};

static int __init ftxxxx_ts_init(void)
{
	int ret;
	ret = i2c_add_driver(&ftxxxx_ts_driver);
	if (ret) {
		printk(KERN_WARNING "Adding ftxxxx driver failed "
			"(errno = %d)\n", ret);
	} else {
		pr_info("[ftxxxx] Successfully added driver %s\n",
			ftxxxx_ts_driver.driver.name);
	}
	return ret;
}

static void __exit ftxxxx_ts_exit(void)
{
	i2c_del_driver(&ftxxxx_ts_driver);
}

module_init(ftxxxx_ts_init);
module_exit(ftxxxx_ts_exit);

MODULE_AUTHOR("<OSTeam>");
MODULE_DESCRIPTION("FocalTech TouchScreen driver");
MODULE_LICENSE("GPL");
