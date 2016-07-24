/*
* Copyright (C) 2015 ASUSTeK Computer Inc.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
* General Public License for more details.
*
*/

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/input.h>
#include <linux/string.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <asm/uaccess.h>
#ifdef CONFIG_HAS_EARLYSUSPEND
#include <linux/earlysuspend.h>
#endif
#include <linux/workqueue.h>
#include <linux/timer.h>
#include <linux/hrtimer.h>
#include <linux/ap3045.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/HWVersion.h>
extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);

#define AP3045_DRV_NAME  "ap3045"
#define DRIVER_VERSION        "1"
#define AP3045_GPIO           44
#define AP3045_MIN_LUX        10
#define AP3045_MAX_LUX     58981
#include <linux/of_gpio.h>
#include <linux/wakelock.h>
#define INTERRUPT_MODE
#define DEBUG_FUN              0
#define DEBUG_VALUE            0
#define PL_TIMER_DELAY       200

#define PS_LED_CON_VALUE    0x08
#define ALS_TIME_VALUE      0x50
#define PS_TIME_VALUE       0x01
#define PS_AWAY_THD           50
#define PS_CLOSE_THD         100
#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
#define PS_CAL_PATH  "/factory/PSensor_Calibration.ini"
#define CONFIG_ASUS_FACTORY_SENSOR_MODE
#define ATTRIBUTES_PERMISSION 0666
#else
#define PS_CAL_PATH  "/data/PSensor_Calibration.ini"
#define ATTRIBUTES_PERMISSION  0660
#endif

#define LSC_DBG
#ifdef LSC_DBG
#define LDBG(s,args...)	{printk("[AP3045]:[%s],",__func__); printk(s,## args);}
#else
#define LDBG(s,args...) {}
#endif

static void plsensor_work_handler(struct work_struct *w);
static void pl_timer_callback(unsigned long pl_data);
static DECLARE_WORK(ap3045_irq_work, plsensor_work_handler);
static int ap3045_set_phthres(struct i2c_client *client, int val);
static int ap3045_set_plthres(struct i2c_client *client, int val);
static int ap3045_get_px_value(struct i2c_client *client);
static int ap3045_get_raw_px_value(struct i2c_client *client);
static int ap3045_get_adc_value(struct i2c_client *client);
static int calibration_light_ap3045(int cal_fac_big, int cal_fac_small, int report_lux);
static void ap3045_change_ls_threshold(struct i2c_client *client,int adc_value);
static int ap3045_ps_init(struct i2c_client *client);
static int ap3045_als_init(struct i2c_client *client);
struct proc_dir_entry *ap3045_lightsensor_entry = NULL;
struct proc_dir_entry *ap3045_proximitysensor_entry = NULL;
struct proc_dir_entry *ap3045_Lpsensor_Entry = NULL;
struct proc_dir_entry *ap3045_psensor_auto_calibration_entry  = NULL;
struct proc_dir_entry *ap3045_psensor_calibration_entry = NULL;
static struct ap3045_data *PL_Driver_Data = NULL;

static int lSensor_CALIDATA[2] = {0}; //input calibration data . Format : "200 lux -->lux value ; 1000 lux -->lux value"
static int pSensor_CALIDATA[2] = {0}; //input calibration data . Format : "near 3cm :--> value ; far  5cm :--> value"
static bool EnLSensorConfig_flag = 0;
static bool EnPSensorConfig_flag = 0;
static int PS_CONVERSION = 4;

// AP3045 register
static u8 ap3045_reg[AP3045_NUM_CACHABLE_REGS] =
{0x00,0x01,0x02,0x06,0x07,0x08,0x0A,0x0C,0x0D,0x0F,
	0x10,0x1B,0x26,0x27,0x2A,0x2B,0x2C,0x2D,0x2E,0x2F,
	0x30,0x31,0x32,0x33,0x34,0x35,0x36,0x37,0x38,0x39,
	0x3A,0x3B,0x3C,0x3D,0x80,0x81,0x82,0x83,0x84,0x85,
	0x86,0xE1};

// AP3045 range TBD

static unsigned int ap3045_range[4]  = {1,2,3,4};
static u8 *reg_array;
static unsigned int *range;
static int reg_num = 0;
static int cali = 100;
static int misc_ps_opened = 0;
static int misc_ls_opened = 0;
static int timer_opened = 0;
static DEFINE_MUTEX(ap3045_lock);
static DEFINE_MUTEX(ap3045_ls_lock);
static DEFINE_MUTEX(ap3045_ps_lock);

#define ADD_TO_IDX(addr,idx)	{														\
		int i;												\
		for(i = 0; i < reg_num; i++)						\
		{													\
			if (addr == reg_array[i])						\
			{												\
				idx = i;									\
				break;										\
			}												\
		}													\
	}


/*
* register access helpers
*/

static int __ap3045_read_reg(struct i2c_client *client,u32 reg, u8 mask, u8 shift)
{
	//struct ap3045_data *data = i2c_get_clientdata(client); //unused
	//u8 idx = 0xff;
	int val = 0;
	val = i2c_smbus_read_byte_data(client, reg);
	return (val & mask) >> shift;
}

static int __ap3045_write_reg(struct i2c_client *client,u32 reg, u8 mask, u8 shift, u8 val)
{
	struct ap3045_data *data = i2c_get_clientdata(client);
	int ret = 0;
	u8 tmp;
	u8 idx = 0xff;
	if(DEBUG_FUN) LDBG(" \n");
	ADD_TO_IDX(reg,idx)
	if (idx >= reg_num)
	return -EINVAL;

	//tmp = data->reg_cache[idx];
	tmp = i2c_smbus_read_byte_data(client, reg);
	tmp &= ~mask;
	tmp |= val << shift;
	if(DEBUG_VALUE) LDBG("val:0x%x\n",val);
	ret = i2c_smbus_write_byte_data(client, reg, tmp);
	if (!ret)
	data->reg_cache[idx] = tmp;

	return ret;
}

/*
* internally used functions
*/

/* range */
static int ap3045_get_range(struct i2c_client *client)
{
	unsigned int val;
	unsigned char rXGain;
	u8 idx = __ap3045_read_reg(client, AP3045_REG_ALS_CONF,
	AP3045_REG_ALS_AGAIN_CON_MASK | AP3045_REG_ALS_AXGAIN_CON_MASK, AP3045_REG_ALS_AGAIN_CON_SHIFT);
	if(DEBUG_FUN) LDBG(" \n");
	rXGain = (idx & AP3045_REG_ALS_AXGAIN_CON_MASK) >> AP3045_REG_ALS_AXGAIN_CON_SHIFT;
	idx = (idx & AP3045_REG_ALS_AGAIN_CON_MASK) >> AP3045_REG_ALS_AGAIN_CON_SHIFT;
	if (rXGain == 0)
	val = range[idx];
	else
	val = range[idx] * 2;
	if(DEBUG_VALUE) LDBG(" %x,%x,%d\n",idx,rXGain,val);

	return val;
}

static int ap3045_set_range(struct i2c_client *client, int range)
{
	if(DEBUG_FUN) LDBG(" \n");
	return __ap3045_write_reg(client, AP3045_REG_ALS_CONF,
	AP3045_REG_ALS_AGAIN_CON_MASK, AP3045_REG_ALS_AGAIN_CON_SHIFT, range);
}

/* mode */
static int ap3045_get_mode(struct i2c_client *client)
{
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	ret = __ap3045_read_reg(client, AP3045_REG_SYS_CONF,
	AP3045_REG_SYS_CONF_MASK, AP3045_REG_SYS_CONF_SHIFT);
	return ret;
}

static int ap3045_get_adc_value(struct i2c_client *client)
{
	unsigned int lsb,msb ,als_lux;
	lsb = i2c_smbus_read_byte_data(client, AP3045_REG_ALS_L_DATA_LOW);
	if(DEBUG_VALUE) LDBG("ALS val lsb =%d lux\n",lsb);
	if (lsb < 0) {
		return 0;
	}
	msb = i2c_smbus_read_byte_data(client, AP3045_REG_ALS_L_DATA_HIGH);
	if(DEBUG_VALUE) LDBG("ALS val msb =%d lux\n",msb);
	if (msb < 0)
	return 0;

	als_lux = msb << 8 | lsb;
	if(als_lux<AP3045_MIN_LUX) als_lux=0;
	if(DEBUG_VALUE) LDBG("ALS als_lux =%d lux\n",als_lux);
	return als_lux;
}

static int ap3045_get_px_value(struct i2c_client *client)
{
	int lsb, msb;
	int px_value;
	if(DEBUG_FUN) LDBG(" \n");
	lsb = i2c_smbus_read_byte_data(client, AP3045_REG_PS_DATA_LOW);
	if (lsb < 0) {
		LDBG(": lsb: %d <0\n",lsb);
		return lsb;
	}
	msb = i2c_smbus_read_byte_data(client, AP3045_REG_PS_DATA_HIGH);
	if (msb < 0) {
		LDBG(": msb: %d <0\n",msb);
		return msb;
	}
	if(DEBUG_VALUE) LDBG(": lsb:%d ,msb:%d\n",lsb,msb);
	px_value=((u32)(((msb & AP3045_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AP3045_REG_PS_DATA_LOW_MASK))) /PS_CONVERSION;
	return abs(px_value);
}

static int ap3045_get_raw_px_value(struct i2c_client *client)
{
	int lsb, msb;
	if(DEBUG_FUN) LDBG(" \n");
	lsb = i2c_smbus_read_byte_data(client, AP3045_REG_PS_DATA_LOW);
	if (lsb < 0) {
		LDBG(": lsb: %d <0\n",lsb);
		return lsb;
	}
	msb = i2c_smbus_read_byte_data(client, AP3045_REG_PS_DATA_HIGH);
	if (msb < 0) {
		LDBG(": msb: %d <0\n",msb);
		return msb;
	}
	if(DEBUG_VALUE) LDBG(": lsb:%d ,msb:%d\n",lsb,msb);
	return ((u32)(((msb & AP3045_REG_PS_DATA_HIGH_MASK) << 8) | (lsb & AP3045_REG_PS_DATA_LOW_MASK)));
}

static int ap3045_get_object(struct i2c_client *client)
{
	int val;
	if(DEBUG_FUN) LDBG(" \n");
	val = i2c_smbus_read_byte_data(client, AP3045_OBJ_COMMAND);
	val &= AP3045_OBJ_MASK;

	return val >> AP3045_OBJ_SHIFT;
}

/* ALS low threshold */
static int ap3045_get_althres(struct i2c_client *client)
{
	int lsb, msb;
	if(DEBUG_FUN) LDBG(" \n");
	lsb = __ap3045_read_reg(client, AP3045_REG_AL_THL_LOW,
	AP3045_REG_AL_THL_LOW_MASK, AP3045_REG_AL_THL_LOW_SHIFT);
	msb = __ap3045_read_reg(client, AP3045_REG_AL_THL_HIGH,
	AP3045_REG_AL_THL_HIGH_MASK, AP3045_REG_AL_THL_HIGH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3045_set_althres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	if(DEBUG_FUN) LDBG(" \n");
	msb = val >> 8;
	lsb = val & AP3045_REG_AL_THL_LOW_MASK;
	err = __ap3045_write_reg(client, AP3045_REG_AL_THL_LOW,
	AP3045_REG_AL_THL_LOW_MASK, AP3045_REG_AL_THL_LOW_SHIFT, lsb);
	if (err)
	return err;

	err = __ap3045_write_reg(client, AP3045_REG_AL_THL_HIGH,
	AP3045_REG_AL_THL_HIGH_MASK, AP3045_REG_AL_THL_HIGH_SHIFT, msb);

	return err;
}

/* ALS high threshold */
static int ap3045_get_ahthres(struct i2c_client *client)
{
	int lsb, msb;
	if(DEBUG_FUN) LDBG(" \n");
	lsb = __ap3045_read_reg(client, AP3045_REG_AL_THH_LOW,
	AP3045_REG_AL_THH_LOW_MASK, AP3045_REG_AL_THH_LOW_SHIFT);
	msb = __ap3045_read_reg(client, AP3045_REG_AL_THH_HIGH,
	AP3045_REG_AL_THH_HIGH_MASK, AP3045_REG_AL_THH_HIGH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3045_set_ahthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	if(DEBUG_FUN) LDBG(" \n");
	msb = val >> 8;
	lsb = val & AP3045_REG_AL_THH_LOW_MASK;
	err = __ap3045_write_reg(client, AP3045_REG_AL_THH_LOW,
	AP3045_REG_AL_THH_LOW_MASK, AP3045_REG_AL_THH_LOW_SHIFT, lsb);
	if (err)
	return err;

	err = __ap3045_write_reg(client, AP3045_REG_AL_THH_HIGH,
	AP3045_REG_AL_THH_HIGH_MASK, AP3045_REG_AL_THH_HIGH_SHIFT, msb);

	return err;
}

/* PX low threshold */
static int ap3045_get_plthres(struct i2c_client *client)
{
	int lsb, msb;
	if(DEBUG_FUN) LDBG(" \n");
	lsb = __ap3045_read_reg(client, AP3045_REG_PS_THL_LOW,
	AP3045_REG_PS_THL_LOW_MASK, AP3045_REG_PS_THL_LOW_SHIFT);
	msb = __ap3045_read_reg(client, AP3045_REG_PS_THL_HIGH,
	AP3045_REG_PS_THL_HIGH_MASK, AP3045_REG_PS_THL_HIGH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3045_set_plthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	if(DEBUG_FUN) LDBG(" \n");

	msb = val >> 8;
	lsb = val & AP3045_REG_PS_THL_LOW_MASK;
	if(DEBUG_VALUE) LDBG("msb:%d ,lsb:%d",msb,lsb);
	err = __ap3045_write_reg(client, AP3045_REG_PS_THL_LOW,
	AP3045_REG_PS_THL_LOW_MASK, AP3045_REG_PS_THL_LOW_SHIFT, lsb);
	if (err)
	return err;

	err = __ap3045_write_reg(client, AP3045_REG_PS_THL_HIGH,
	AP3045_REG_PS_THL_HIGH_MASK, AP3045_REG_PS_THL_HIGH_SHIFT, msb);

	return err;
}

/* PX high threshold */
static int ap3045_get_phthres(struct i2c_client *client)
{
	int lsb, msb;
	if(DEBUG_FUN) LDBG(" \n");
	lsb = __ap3045_read_reg(client, AP3045_REG_PS_THH_LOW,
	AP3045_REG_PS_THH_LOW_MASK, AP3045_REG_PS_THH_LOW_SHIFT);
	msb = __ap3045_read_reg(client, AP3045_REG_PS_THH_HIGH,
	AP3045_REG_PS_THH_HIGH_MASK, AP3045_REG_PS_THH_HIGH_SHIFT);
	return ((msb << 8) | lsb);
}

static int ap3045_set_phthres(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	if(DEBUG_FUN) LDBG(" \n");
	if(DEBUG_VALUE) LDBG(" val:%d",val);
	msb = val >> 8;
	lsb = val & AP3045_REG_PS_THH_LOW_MASK;
	if(DEBUG_VALUE) LDBG("msb:%d ,lsb:%d",msb,lsb);
	err = __ap3045_write_reg(client, AP3045_REG_PS_THH_LOW,
	AP3045_REG_PS_THH_LOW_MASK, AP3045_REG_PS_THH_LOW_SHIFT, lsb);
	if (err)
	return err;

	err = __ap3045_write_reg(client, AP3045_REG_PS_THH_HIGH,
	AP3045_REG_PS_THH_HIGH_MASK, AP3045_REG_PS_THH_HIGH_SHIFT, msb);

	return err;
}

//=========================================
//     Calibration Formula:
//     y = f(x)
//  -> ax - by = constant_k
//     a is f(x2) - f(x1) , b is x2 - x1
////=========================================
static int calibration_light_ap3045(int cal_fac_big, int cal_fac_small, int report_lux)
{
	//int cal_spec_big = 1000;//asus calibration 1000lux
	int cal_spec_small = 200;//asus calibration 200lux
	int cal_spec_diff =800;
	int cal_fac_diff = cal_fac_big-cal_fac_small;
	int constant_k;
	if(DEBUG_FUN) LDBG(" \n");

	constant_k = (cal_spec_diff*cal_fac_small) - (cal_fac_diff*cal_spec_small);
	if ( (report_lux*cal_spec_diff) < constant_k){
		return report_lux;
	} else {
		return (((report_lux*cal_spec_diff) - constant_k) / cal_fac_diff);
	}
}
static void ap3045_change_ls_threshold(struct i2c_client *client,int adc_value)
{
	int lowTH=0;
	int highTH=1;
	if(DEBUG_FUN) LDBG(" \n");
	if(adc_value > AP3045_MAX_LUX){
		lowTH=58981;
		highTH=65535;
	} else if(adc_value < AP3045_MIN_LUX){
		lowTH=0;
		highTH=5;
	}else{
		lowTH=adc_value-abs(adc_value/10);
		highTH=adc_value+abs(adc_value/10);
	}
	if(DEBUG_VALUE) LDBG("adc lowTH=%d ,highTH=%d \n",lowTH,highTH);
	ap3045_set_althres(client,lowTH);
	ap3045_set_ahthres(client,highTH);

}
static int ap3045_set_mode(struct i2c_client *client, int mode)
{
	int ret,report_lux,obj;
	int misc_ps_last=misc_ps_opened;
	int misc_ls_last=misc_ls_opened;
	struct ap3045_data *data = i2c_get_clientdata(client);
	if(DEBUG_FUN) LDBG(" \n");
	if(mode == AP3045_SYS_PS_ENABLE) {
		misc_ps_opened = 1;
		misc_ls_opened = 0;
	} else if(mode == AP3045_SYS_ALS_ENABLE) {
		misc_ps_opened = 0;
		misc_ls_opened = 1;
	} else if(mode == (AP3045_SYS_PS_ENABLE | AP3045_SYS_ALS_ENABLE)) {
		misc_ps_opened = 1;
		misc_ls_opened = 1;
	} else if(mode == AP3045_SYS_DEV_DOWN) {
		misc_ps_opened = 0;
		misc_ls_opened = 0;
	}else{
		misc_ps_opened = 0;
		misc_ls_opened = 0;
	}
#ifndef INTERRUPT_MODE
	if (misc_ps_opened == 1 || misc_ls_opened == 1  ){
		if (timer_opened == 0){
			timer_opened = 1;
			LDBG("Timer Add\n");
			ret = mod_timer(&data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
			if(!ret)
			LDBG("Timer Error\n");
		}
	}else if (misc_ps_opened == 0 && misc_ls_opened == 0  ){
		if (timer_opened == 1){
			timer_opened = 0;
			del_timer(&data->pl_timer);
			LDBG("Timer Del\n");
		}
	}
#endif
	ret = __ap3045_write_reg(client, AP3045_REG_SYS_CONF,
	AP3045_REG_SYS_CONF_MASK, AP3045_REG_SYS_CONF_SHIFT, mode);
	ret = __ap3045_write_reg(client, AP3045_REG_SYS_INTSTATUS,
	0xFF, 0x00, 0);
#ifdef INTERRUPT_MODE
	msleep(100);
	if(misc_ls_opened==1 && misc_ls_last==0){
		report_lux = ap3045_get_adc_value(data->client);
		ap3045_change_ls_threshold(data->client,report_lux);
		if(EnLSensorConfig_flag) {
			report_lux = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux);
		}
		/* sensor hal report the same value will skip*/
		input_report_abs(data->ls_input_dev, ABS_MISC, report_lux+1);
		input_report_abs(data->ls_input_dev, ABS_MISC, report_lux);
		input_sync(data->ls_input_dev);
		LDBG("initial sensor lux: %d\n", report_lux);
	}
	if(misc_ps_opened==1 && misc_ps_last==0){
		if(DEBUG_VALUE) LDBG("initial sensor ps_adc = %d \n", ap3045_get_px_value(data->client));
		obj = ap3045_get_object(data->client);
		input_report_abs(data->ps_input_dev, ABS_DISTANCE, obj);
		input_report_abs(data->ps_input_dev, ABS_DISTANCE, !obj);
		input_sync(data->ps_input_dev);
		LDBG("initial sensor = %s\n",  obj ? "obj near":"obj far");
	}
#endif
	return ret;
}

static int ap3045_set_PSCalibration(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	if(DEBUG_FUN) LDBG(" \n");
	msb = val >> 8;
	lsb = val & AP3045_REG_PS_CAL_LOW_MASK;

	err = __ap3045_write_reg(client, AP3045_REG_PS_CAL_LOW,
	AP3045_REG_PS_CAL_LOW_MASK, AP3045_REG_PS_CAL_LOW_SHIFT, lsb);
	if (err)
	return err;

	err = __ap3045_write_reg(client, AP3045_REG_PS_CAL_HIGH,
	AP3045_REG_PS_CAL_HIGH_MASK, AP3045_REG_PS_CAL_HIGH_SHIFT, msb);
	if(DEBUG_VALUE) LDBG(" lsb= %d,msb= %d\n",lsb, msb);
	return err;
}

static int ap3045_get_intstat(struct i2c_client *client)
{
	int val;
	if(DEBUG_FUN) LDBG(" \n");
	val = i2c_smbus_read_byte_data(client, AP3045_REG_SYS_INTSTATUS);
	return val ;
}

static int ap3045_lsensor_enable(struct i2c_client *client)
{
	int ret = 0,mode;
	if(DEBUG_FUN) LDBG(" \n");
	mode = ap3045_get_mode(client);
	if((mode & AP3045_SYS_ALS_ENABLE) == 0){
		mode |= AP3045_SYS_ALS_ENABLE;
		ret = ap3045_set_mode(client,mode);
	}
	return ret;
}

static int ap3045_lsensor_disable(struct i2c_client *client)
{
	int ret = 0,mode;
	if(DEBUG_FUN) LDBG(" \n");
	mode = ap3045_get_mode(client);
	if(mode & AP3045_SYS_ALS_ENABLE){
		mode &= ~AP3045_SYS_ALS_ENABLE;
		if(mode == AP3045_SYS_RST_ENABLE)
		mode = 0;
		ret = ap3045_set_mode(client,mode);
	}

	return ret;
}

static long lsensor_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int rc = 0,val = 0,report_lux = 0;
	char encalibration_flag = 0;
	void __user *argp = (void __user *)arg;
	if(DEBUG_FUN) LDBG(" \n");
	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned long __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		LDBG("LIGHTSENSOR_IOCTL_ENABLE, value = %d\n", val);
		rc = val ? ap3045_lsensor_enable(PL_Driver_Data -> client) : ap3045_lsensor_disable(PL_Driver_Data -> client);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = misc_ls_opened;
		LDBG("LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n", val);
		rc = put_user(val, (unsigned long __user *)arg);
		break;
	case ASUS_LIGHTSENSOR_IOCTL_START:
		LDBG("ASUS_LIGHTSENSOR_IOCTL_START  \n");
		break;
	case ASUS_LIGHTSENSOR_IOCTL_CLOSE:
		LDBG("ASUS_LIGHTSENSOR_IOCTL_CLOSE \n");
		break;
	case ASUS_LIGHTSENSOR_IOCTL_GETDATA:
		LDBG("ASUS_LIGHTSENSOR_IOCTL_GETDATA \n");
		rc = 0 ;
		report_lux = ap3045_get_adc_value(PL_Driver_Data -> client);
		if(EnLSensorConfig_flag == 1){//calibration enable
			LDBG("Before calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA,  report_lux is %d\n", report_lux);
			report_lux = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux);
			LDBG("After calibration, ASUS_LIGHTSENSOR_IOCTL_GETDATA, report_lux is %d\n", report_lux);
		}else{
			LDBG("NO calibration data, default config report_lux:%d\n", report_lux);
		}

		if ( copy_to_user(argp, &report_lux, sizeof(report_lux) ) ) {
			LDBG("ASUS failed to copy lightsense data to user space.\n");
			rc = -EFAULT;
			goto end;
		}
		break;
	case ASUS_LIGHTSENSOR_SETCALI_DATA:
		LDBG("ASUS_LIGHTSENSOR_SETCALI_DATA \n");
		rc = 0 ;
		memset(lSensor_CALIDATA, 0, 2*sizeof(int));
		if (copy_from_user(lSensor_CALIDATA, argp, sizeof(lSensor_CALIDATA)))
		{
			rc = -EFAULT;
			goto end;
		}
		LDBG("ASUS_LIGHTSENSOR SETCALI_DATA : lSensor_CALIDATA[0] :  %d ,lSensor_CALIDATA[1]:  %d \n",
		lSensor_CALIDATA[0],lSensor_CALIDATA[1]);
		if(lSensor_CALIDATA[0] <=0||lSensor_CALIDATA[1] <=0) {
			rc =  -EINVAL;
		}
		break;
	case ASUS_LIGHTSENSOR_EN_CALIBRATION:
		LDBG("ASUS_LIGHTSENSOR_EN_CALIBRATION \n");
		rc = 0 ;
		if (copy_from_user(&encalibration_flag , argp, sizeof(encalibration_flag )))
		{
			rc = -EFAULT;
			goto end;
		}
		EnLSensorConfig_flag =  encalibration_flag ;
		LDBG("ASUS_LIGHTSENSOR_EN_CALIBRATION : EnLSensorConfig_flag:%d  \n",EnLSensorConfig_flag);
		break;
	default:
		LDBG("invalid cmd %d\n", _IOC_NR(cmd));
		rc = -EINVAL;
	}

	return rc;
end:

	return rc;
}

static int lsensor_release(struct inode *inode, struct file *file)
{
	if(DEBUG_FUN) LDBG(" \n");
	PL_Driver_Data -> ls_opened = 0;
	return 0;
}

static int lsensor_open(struct inode *inode, struct file *file)
{
	int rc = 0;
	if(DEBUG_FUN) LDBG(" \n");

	if (PL_Driver_Data -> ls_opened) {
		LDBG("already opened\n");
		rc = -EBUSY;
	}

	PL_Driver_Data -> ls_opened = 1;
	return rc;
}

static const struct file_operations lsensor_fops = {
	.owner = THIS_MODULE,
	.open = lsensor_open,
	.release = lsensor_release,
	.unlocked_ioctl = lsensor_ioctl
};

static struct miscdevice lsensor_misc_ap3045 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lsensor_fops
};

static int ap3045_register_lsensor_device(struct i2c_client *client, struct ap3045_data *data)
{
	struct input_dev *input_dev;
	int rc;

	if(DEBUG_FUN) LDBG(" \n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev,"%s: could not allocate input device for lsensor\n", __FUNCTION__);
		rc = -ENOMEM;
		goto done;
	}
	data->ls_input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = "lightsensor-level";
	input_dev->dev.parent = &client->dev;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_MISC, 0, 65535, 0, 0);

	rc = input_register_device(input_dev);
	if (rc < 0) {
		LDBG("could not register input device for lsensor\n");
		goto done;
	}

	rc = misc_register(&lsensor_misc_ap3045);
	if (rc < 0) {
		LDBG( "could not register ps misc device\n");
		goto error_misc;
	} else {
		goto done;
	}

error_misc:
	misc_deregister(&lsensor_misc_ap3045);

done:
	return rc;
}

static void ap3045_unregister_lsensor_device(struct i2c_client *client, struct ap3045_data *data)
{
	if(DEBUG_FUN) LDBG(" \n");
	input_unregister_device(data->ls_input_dev);
}

static int ap3045_psensor_enable(struct i2c_client *client)
{
	int ret = 0,mode;
	if(DEBUG_FUN) LDBG(" \n");
	mode = ap3045_get_mode(client);
	if((mode & AP3045_SYS_PS_ENABLE) == 0){
		mode |= AP3045_SYS_PS_ENABLE;
		ret = ap3045_set_mode(client,mode);
	}
	return ret;
}

static int ap3045_psensor_disable(struct i2c_client *client)
{
	int ret = 0,mode;
	if(DEBUG_FUN) LDBG(" \n");
	mode = ap3045_get_mode(client);
	if(mode & AP3045_SYS_PS_ENABLE){
		mode &= ~AP3045_SYS_PS_ENABLE;
		if(mode == AP3045_SYS_RST_ENABLE)
		mode = AP3045_SYS_DEV_DOWN;
		ret = ap3045_set_mode(client,mode);
	}
	return ret;
}

static int ap3045_set_crosstalk(struct i2c_client *client, int val)
{
	int lsb, msb, err;
	if(DEBUG_FUN) LDBG(" \n");
	msb = val >> 8;
	lsb = val & AP3045_REG_PS_CAL_LOW_MASK;
	if(DEBUG_VALUE) LDBG("ap3045_set_crosstalk : msb :  %d ,lsb:  %d \n", msb,lsb);
	err = __ap3045_write_reg(client, AP3045_REG_PS_CAL_LOW,
	AP3045_REG_PS_CAL_LOW_MASK, AP3045_REG_PS_CAL_LOW_SHIFT, lsb);
	if (err)
	return err;

	err = __ap3045_write_reg(client, AP3045_REG_PS_CAL_HIGH,
	AP3045_REG_PS_CAL_HIGH_MASK, AP3045_REG_PS_CAL_HIGH_SHIFT, msb);

	return err;
}
static long psensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val,rc,ret;
	uint16_t px_value;
	char enPcalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;
	if(DEBUG_FUN) LDBG(" \n");
	if(DEBUG_VALUE) LDBG("cmd %d\n",  _IOC_NR(cmd));

	switch (cmd) {
	case PROXIMITYSENSOR_IOCTL_ENABLE:

		if (get_user(val, (unsigned long __user *)arg))
		return -EFAULT;
		LDBG("PROXIMITYSENSOR_IOCTL_ENABLE val=%d \n",val);
		if (val){
			return ap3045_psensor_enable(PL_Driver_Data -> client);
		}else{
			return ap3045_psensor_disable(PL_Driver_Data -> client);
		}

		break;
	case PROXIMITYSENSOR_IOCTL_GET_ENABLED:
		LDBG("PROXIMITYSENSOR_IOCTL_GET_ENABLED \n");
		return put_user(misc_ps_opened, (unsigned long __user *)arg);
		break;
	case ASUS_PSENSOR_IOCTL_GETDATA:
		LDBG("ASUS_PSENSOR_IOCTL_GETDATA \n");
		rc = 0 ;

		ret = ap3045_get_px_value(PL_Driver_Data -> client);
		LDBG("ASUS get_px_value:%d\n",ret);
		if (ret < 0) {
			LDBG("ASUS failed to get_px_value. \n");
			rc = -EIO;
			goto pend;
		}

		px_value = (uint16_t)ret;
		if ( copy_to_user(argp, &px_value, sizeof(px_value) ) ) {
			LDBG("ASUS failed to copy psense data to user space.\n");
			rc = -EFAULT;
			goto pend;
		}
		break;
	case ASUS_PSENSOR_SETCALI_DATA:
		LDBG("ASUS_PSENSOR_SETCALI_DATA \n");
		rc = 0 ;
		memset(pSensor_CALIDATA, 0, 2*sizeof(int));
		if (copy_from_user(pSensor_CALIDATA, argp, sizeof(pSensor_CALIDATA))){
			rc = -EFAULT;
			goto pend;
		}

		LDBG("ASUS_PSENSOR SETCALI_DATA : pSensor_CALIDATA[0] :  %d ,pSensor_CALIDATA[1]:  %d \n",
		pSensor_CALIDATA[0],pSensor_CALIDATA[1]);

		if(pSensor_CALIDATA[0] < 0||pSensor_CALIDATA[1] < 0 ) {
			rc =  -EINVAL;
			goto pend;
		}

		ap3045_set_plthres(PL_Driver_Data -> client,pSensor_CALIDATA[1]);//5cm
		ap3045_set_phthres(PL_Driver_Data -> client,pSensor_CALIDATA[0]);//3cm

	//	if (ap3045_set_crosstalk(PL_Driver_Data -> client, 0 )) {
	//		rc = -EFAULT;
	//		LDBG("ap3045_set_crosstalk error\n");
	//		goto pend;
	//	}
		break;

	case ASUS_PSENSOR_EN_CALIBRATION:
		LDBG("ASUS_PSENSOR_EN_CALIBRATION \n");
		rc = 0 ;
		if (copy_from_user(&enPcalibration_flag , argp, sizeof(enPcalibration_flag ))){
			rc = -EFAULT;
			goto pend;
		}
		EnPSensorConfig_flag =  enPcalibration_flag ;
		if(EnPSensorConfig_flag == 0){
			ap3045_set_crosstalk(PL_Driver_Data -> client, EnPSensorConfig_flag);
		}
		LDBG("ASUS_PSENSOR_EN_CALIBRATION : EnPSensorConfig_flag is : %d  \n",EnPSensorConfig_flag);
		break;

	default:
		LDBG(" invalid cmd %d\n",_IOC_NR(cmd));
		return -EINVAL;
	}

pend:
	return rc;
}
static int psensor_open(struct inode *inode, struct file *file)
{
	if(DEBUG_FUN) LDBG(" \n");
	if (PL_Driver_Data -> ps_opened) {
		return -EBUSY;
	}
	PL_Driver_Data -> ps_opened = 1;
	return 0;
}
static int psensor_release(struct inode *inode, struct file *file)
{
	if(DEBUG_FUN) LDBG(" \n");
	PL_Driver_Data -> ps_opened = 0;
	//	return ap3045_psensor_disable(PL_Driver_Data -> client);
	return 0;
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};
struct miscdevice psensor_misc_ap3045 = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "psensor",
	.fops = &psensor_fops
};

static int ap3045_register_psensor_device(struct i2c_client *client, struct ap3045_data *data)
{
	struct input_dev *input_dev;
	int rc;
	if(DEBUG_FUN) LDBG(" \n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&client->dev,"%s: could not allocate input device for psensor\n", __FUNCTION__);
		rc = -ENOMEM;
		goto done;
	}
	data->ps_input_dev = input_dev;
	input_set_drvdata(input_dev, data);
	input_dev->name = "proximity";
	input_dev->dev.parent = &client->dev;
	set_bit(EV_ABS, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_DISTANCE, 0, 1, 0, 0);

	rc = input_register_device(input_dev);
	if (rc < 0) {
		LDBG("could not register input device for psensor\n");
		goto done;
	}

	rc = misc_register(&psensor_misc_ap3045);
	if (rc < 0) {
		LDBG( "could not register ps misc device\n");
		goto error_misc;
	} else {
		goto done;
	}
error_misc:
	misc_deregister(&psensor_misc_ap3045);

	return 0;

done:
	return rc;

}

static void ap3045_unregister_psensor_device(struct i2c_client *client, struct ap3045_data *data)
{
	if(DEBUG_FUN) LDBG(" \n");
	input_unregister_device(data->ps_input_dev);
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static int ap3045_resume_psensor =0  ;
static int ap3045_resume_lsensor =0  ;
static struct early_suspend ap3045_early_suspend;
static void ap3045_suspend(struct early_suspend *h)
{
	if(DEBUG_FUN) LDBG(" \n");
#ifndef INTERRUPT_MODE
	if (misc_ps_opened) {
		ap3045_resume_psensor=1;
		ap3045_psensor_disable(PL_Driver_Data->client);
	}
	if (misc_ls_opened) {
		ap3045_resume_lsensor=1;
		ap3045_lsensor_disable(PL_Driver_Data->client);
	}
#else
	if (misc_ps_opened == 1) {
		enable_irq_wake(PL_Driver_Data->irq);
	}
#endif
}

static void ap3045_resume(struct early_suspend *h)
{
	if(DEBUG_FUN) LDBG(" \n");
#ifndef INTERRUPT_MODE
	if (ap3045_resume_psensor==1) {
		misc_ps_opened = 1;
		ap3045_psensor_enable(PL_Driver_Data->client);
	}
	if (ap3045_resume_lsensor==1) {
		misc_ls_opened = 1;
		ap3045_lsensor_enable(PL_Driver_Data->client);
	}
#else
	if (misc_ps_opened==1) {
		ap3045_psensor_enable(PL_Driver_Data->client);
	}
	if (misc_ls_opened==1) {
		ap3045_lsensor_enable(PL_Driver_Data->client);
	}
	if (misc_ps_opened == 1) {
		disable_irq_wake(PL_Driver_Data->irq);
	}
#endif
}
#endif

/* range */
static ssize_t ap3045_show_range(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(DEBUG_FUN) LDBG(" \n");
	return sprintf(buf, "%i\n", ap3045_get_range(PL_Driver_Data->client));
}

static ssize_t ap3045_store_range(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	unsigned long val;
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	if ((strict_strtoul(buf, 10, &val) < 0) || (val > 3)) {
		LDBG("val:%ld  Failed\n",val);
		return -EINVAL;
	}
	LDBG("val:%ld  \n",val);
	ret = ap3045_set_range(PL_Driver_Data->client, val);
	if (ret < 0)
	return ret;

	return count;
}

/* mode */
static ssize_t ap3045_show_mode(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(DEBUG_FUN) LDBG(" \n");
	return sprintf(buf, "%d\n", ap3045_get_mode(PL_Driver_Data->client));
}

static ssize_t ap3045_store_mode(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

	ret = ap3045_set_mode(PL_Driver_Data->client, val);
	if (ret < 0)
	return ret;

	return count;
}

static int ap3045_ps_init(struct i2c_client *client)
{
	int ret;
	if(DEBUG_FUN) LDBG(" \n");

	ret = __ap3045_write_reg(client, AP3045_REG_PS_TIME,
	AP3045_REG_PS_TIME_MASK, AP3045_REG_PS_TIME_SHIFT, PS_TIME_VALUE);
	if(ret < 0)	LDBG("Init AP3045_REG_PS_TIME Failed\n");

	ret = __ap3045_write_reg(client, AP3045_REG_PS_MEAN,
	AP3045_REG_PS_MEAN_MASK, AP3045_REG_PS_MEAN_SHIFT, 0x00);
	if(ret < 0)	LDBG("Init AP3045_REG_PS_MEAN Failed\n");
#ifdef INTERRUPT_MODE
	ret = __ap3045_write_reg(client, AP3045_REG_PS_PERS,
	AP3045_REG_PS_PERS_MASK, AP3045_REG_PS_PERS_SHIFT, 0x01);
	if(ret < 0)	LDBG("Init AP3045_REG_PS_PERS Failed\n");
#endif

	ret = __ap3045_write_reg(client, AP3045_REG_PS_LED_CON,
	AP3045_REG_PS_LED_CON_PSLPUW_MASK, AP3045_REG_PS_LED_CON_PSLPUW_SHIFT, PS_LED_CON_VALUE);
	if(ret < 0) LDBG("Init AP3045_REG_PS_LED_CON Failed\n");

	ret = ap3045_set_plthres(client, PS_AWAY_THD);
	if(ret < 0)	LDBG("Init ap3045_set_plthres Failed\n");
	ret = ap3045_set_phthres(client, PS_CLOSE_THD);
	if(ret < 0)	LDBG("Init ap3045_set_phthres Failed\n");
	ret = ap3045_set_PSCalibration(client, 0x00);
	if(ret < 0)	LDBG("Init ap3045_set_PSCalibration Failed\n");

	return ret;
}
static int ap3045_als_init(struct i2c_client *client)
{
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	ret = ap3045_set_althres(client, 0);
	if(!ret < 0) LDBG("Init ap3045_set_althres Failed\n");
	ret = ap3045_set_ahthres(client, 1);
	if(!ret < 0) LDBG("Init ap3045_set_ahthres Failed\n");
	ret = __ap3045_write_reg(client, AP3045_REG_ALS_TIME,
	AP3045_REG_ALS_TIME_MASK, AP3045_REG_ALS_TIME_SHIFT, ALS_TIME_VALUE);
	if(!ret < 0) LDBG("Init AP3045_REG_ALS_TIME Failed\n");
#ifdef INTERRUPT_MODE
	ret = __ap3045_write_reg(client, AP3045_REG_ALS_PERSIS,
	AP3045_REG_ALS_PERSIS_MASK, AP3045_REG_ALS_PERSIS_SHIFT, 0x01);

	if(ret < 0)	LDBG("Init AP3045_REG_ALS_PERSIS Failed\n");

	if (i2c_smbus_write_byte_data(client, AP3045_REG_SYS_INT_CTRL, 0xA8)< 0) {
		LDBG("I2C Write SYS_INT_CTRL Failed\n");
		ret = -1;
	}

#endif
	//ret = ap3045_set_range(PL_Driver_Data->client, 0X00); //AGAIN: B1 B0 , AXGAIN B2
	return ret;
}

static ssize_t ap3045_ls_enable(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	if(DEBUG_FUN) LDBG(" \n");
	if(DEBUG_VALUE) LDBG("mode = ,%s\n", buf);
	if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

	if(DEBUG_VALUE) LDBG("%ld\n",val);
	if (val == 1){
		ap3045_lsensor_enable(PL_Driver_Data->client);
	}else{
		ap3045_lsensor_disable(PL_Driver_Data->client);
	}
	return count;
}


static ssize_t ap3045_ps_enable(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	if(DEBUG_FUN) LDBG(" \n");
	if(DEBUG_VALUE) LDBG(" buf:%s\n", buf);

	if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

	if(val == AP3045_SYS_PS_ENABLE) {
		ap3045_psensor_enable(PL_Driver_Data->client);
	} else {
		ap3045_psensor_disable(PL_Driver_Data->client);
	}

	return count;
}

/* lux */
static ssize_t ap3045_show_ls_value(struct device *dev,struct device_attribute *attr, char *buf)
{
	int report_lux=0;
	if(DEBUG_FUN) LDBG(" \n");
	if (ap3045_get_mode(PL_Driver_Data->client) != AP3045_SYS_ALS_ENABLE) {
		ap3045_lsensor_enable(PL_Driver_Data->client);
	}
	report_lux = ap3045_get_adc_value(PL_Driver_Data->client);
	if(EnLSensorConfig_flag) {
		report_lux = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux);
		LDBG(" calibration lux %d\n",report_lux );
	} else {
		LDBG(" without calibration lux %d\n",report_lux );
	}
	return sprintf(buf, "%d\n", report_lux);
}

static ssize_t ap3045_show_ls_adc(struct device *dev,struct device_attribute *attr, char *buf)
{
	int report_lux=0;
	if(DEBUG_FUN) LDBG(" \n");
	if (ap3045_get_mode(PL_Driver_Data->client) != AP3045_SYS_ALS_ENABLE) {
		ap3045_lsensor_enable(PL_Driver_Data->client);
	}
	report_lux = ap3045_get_adc_value(PL_Driver_Data->client);

	return sprintf(buf, "%d\n", report_lux);
}

/* Px data */
static ssize_t ap3045_show_ps_adc(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(DEBUG_FUN) LDBG(" \n");
	if (ap3045_get_mode(PL_Driver_Data->client) != AP3045_SYS_PS_ENABLE) {
		ap3045_psensor_enable(PL_Driver_Data->client);
	}
	return sprintf(buf, "%d\n", ap3045_get_px_value(PL_Driver_Data->client));
}

static ssize_t ap3045_show_ps_raw(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(DEBUG_FUN) LDBG(" \n");
	if (ap3045_get_mode(PL_Driver_Data->client) != AP3045_SYS_PS_ENABLE) {
		ap3045_psensor_enable(PL_Driver_Data->client);
	}
	return sprintf(buf, "%d\n", ap3045_get_raw_px_value(PL_Driver_Data->client));
}

/* proximity object detect */
static ssize_t ap3045_show_object(struct device *dev,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", ap3045_get_object(PL_Driver_Data->client));
}

/* ALS low threshold */
static ssize_t ap3045_show_althres(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3045_get_althres(PL_Driver_Data->client));
}

static ssize_t ap3045_store_althres(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

	ret = ap3045_set_althres(PL_Driver_Data->client, val);
	if (ret < 0)
	return ret;

	return count;
}

/* ALS high threshold */
static ssize_t ap3045_show_ahthres(struct device *dev,struct device_attribute *attr, char *buf)
{
	if(DEBUG_FUN) LDBG(" \n");
	return sprintf(buf, "%d\n", ap3045_get_ahthres(PL_Driver_Data->client));
}

static ssize_t ap3045_store_ahthres(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

	ret = ap3045_set_ahthres(PL_Driver_Data->client, val);
	if (ret < 0)
	return ret;

	return count;
}

/* Px low threshold */
static ssize_t ap3045_show_plthres(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3045_get_plthres(PL_Driver_Data->client));
}

static ssize_t ap3045_store_plthres(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

	ret = ap3045_set_plthres(PL_Driver_Data->client, val);
	if (ret < 0)
	return ret;

	return count;
}

/* Px high threshold */
static ssize_t ap3045_show_phthres(struct device *dev,struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "%d\n", ap3045_get_phthres(PL_Driver_Data->client));
}

static ssize_t ap3045_store_phthres(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long val;
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	if (strict_strtoul(buf, 10, &val) < 0)
	return -EINVAL;

	ret = ap3045_set_phthres(PL_Driver_Data->client, val);
	if (ret < 0)
	return ret;

	return count;
}

/* calibration */
static ssize_t ap3045_show_calibration_state(struct device *dev,struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "%d\n", cali);
}

static ssize_t ap3045_store_calibration_state(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	int stdls, lux;
	char tmp[10];

	if(DEBUG_FUN) LDBG(" \n");

	/* No LUX data if not operational */
	if (ap3045_get_mode(PL_Driver_Data->client) == AP3045_SYS_DEV_DOWN)
	{
		LDBG("Please power up first!");
		return -EINVAL;
	}

	cali = 100;
	sscanf(buf, "%d %s", &stdls, tmp);

	if (!strncmp(tmp, "-setcv", 6))
	{
		cali = stdls;
		return -EBUSY;
	}

	if (stdls < 0)
	{
		LDBG("Std light source: [%d] < 0 !!!\nCheck again, please.\n\
			Set calibration factor to 100.\n", stdls);
		return -EBUSY;
	}

	lux = ap3045_get_adc_value(PL_Driver_Data->client);
	cali = stdls * 100 / lux;

	return -EBUSY;
}


#ifdef LSC_DBG
/* engineer mode */
static ssize_t ap3045_em_read(struct device *dev,struct device_attribute *attr,char *buf)
{
	int i;
	u8 tmp;
	if(DEBUG_FUN) LDBG(" \n");
	for (i = 0; i < reg_num; i++)
	{
		tmp = i2c_smbus_read_byte_data(PL_Driver_Data->client, reg_array[i]);
		LDBG("Reg[0x%x] Val[0x%x]\n", reg_array[i], tmp);
	}
	return 0;
}

static ssize_t ap3045_em_write(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
	u32 addr,val,idx=0;
	int ret = 0;
	if(DEBUG_FUN) LDBG(" \n");
	sscanf(buf, "%x%x", &addr, &val);
	LDBG("Write [%x] to Reg[%x]...\n",val,addr);
	ret = i2c_smbus_write_byte_data(PL_Driver_Data->client, addr, val);
	ADD_TO_IDX(addr,idx)
	if (!ret)
	PL_Driver_Data->reg_cache[idx] = val;

	return count;
}
#endif

static ssize_t ap3045_store_lux(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static ssize_t ap3045_store_lsadc(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static ssize_t ap3045_store_pxvalue(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static ssize_t ap3045_store_psadc(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}
static ssize_t ap3045_store_object(struct device *dev,struct device_attribute *attr, const char *buf, size_t count)
{
	return 0;
}


static struct device_attribute lsensor_attributes[] = {
	__ATTR(range, ATTRIBUTES_PERMISSION, ap3045_show_range, ap3045_store_range),
	__ATTR(mode, ATTRIBUTES_PERMISSION, ap3045_show_mode, ap3045_store_mode),
	__ATTR(lsensor, ATTRIBUTES_PERMISSION, ap3045_show_mode, ap3045_ls_enable),
	__ATTR(ls_adc, ATTRIBUTES_PERMISSION, ap3045_show_ls_adc, ap3045_store_lsadc),
	__ATTR(ls_value, ATTRIBUTES_PERMISSION, ap3045_show_ls_value, ap3045_store_lux),
	__ATTR(object, ATTRIBUTES_PERMISSION, ap3045_show_object, ap3045_store_object),
	__ATTR(althres, ATTRIBUTES_PERMISSION, ap3045_show_althres, ap3045_store_althres),
	__ATTR(ahthres, ATTRIBUTES_PERMISSION, ap3045_show_ahthres, ap3045_store_ahthres),
	__ATTR(calibration, ATTRIBUTES_PERMISSION, ap3045_show_calibration_state, ap3045_store_calibration_state),
#ifdef LSC_DBG
	__ATTR(em, ATTRIBUTES_PERMISSION, ap3045_em_read, ap3045_em_write),
#endif

};

static struct device_attribute psensor_attributes[] = {
	__ATTR(range, ATTRIBUTES_PERMISSION, ap3045_show_range, ap3045_store_range),
	__ATTR(mode, ATTRIBUTES_PERMISSION, ap3045_show_mode, ap3045_store_mode),
	__ATTR(psensor, ATTRIBUTES_PERMISSION, ap3045_show_mode, ap3045_ps_enable),
	__ATTR(ps_adc, ATTRIBUTES_PERMISSION, ap3045_show_ps_adc, ap3045_store_pxvalue),
	__ATTR(ps_raw, ATTRIBUTES_PERMISSION, ap3045_show_ps_raw, ap3045_store_psadc),
	__ATTR(object, ATTRIBUTES_PERMISSION, ap3045_show_object, ap3045_store_object),
	__ATTR(plthres, ATTRIBUTES_PERMISSION, ap3045_show_plthres, ap3045_store_plthres),
	__ATTR(phthres, ATTRIBUTES_PERMISSION, ap3045_show_phthres, ap3045_store_phthres),
	__ATTR(calibration, ATTRIBUTES_PERMISSION, ap3045_show_calibration_state, ap3045_store_calibration_state),
#ifdef LSC_DBG
	__ATTR(em, ATTRIBUTES_PERMISSION, ap3045_em_read, ap3045_em_write),
#endif

};

static int create_sysfs_interfaces(struct ap3045_data *sensor)
{
	int i;
	struct class *ap3045_class = NULL;
	struct device *ap3045_dev_psensor = NULL;
	struct device *ap3045_dev_lsensor = NULL;
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	ap3045_class = class_create(THIS_MODULE, "optical_sensors");

	if (IS_ERR(ap3045_class)) {
		ret = PTR_ERR(ap3045_class);
		ap3045_class = NULL;
		LDBG("could not allocate ap3045_class, ret = %d\n", ret);
		goto ap3045_class_error;
	}

	ap3045_dev_psensor= device_create(ap3045_class,  NULL, 0, "%s", "proximity");
	ap3045_dev_lsensor= device_create(ap3045_class,  NULL, 0, "%s", "lightsensor");
	if(ap3045_dev_psensor == NULL || ap3045_dev_lsensor == NULL) {
		LDBG("ap3045_dev == NULL\n");
		goto ap3045_device_error;
	}

	for (i = 0; i < ARRAY_SIZE(lsensor_attributes); i++){
		if (device_create_file(ap3045_dev_lsensor, lsensor_attributes + i)) {
			LDBG("ap3045_create_file_error \n");
			goto ap3045_create_file_error;
		}
	}
	for (i = 0; i < ARRAY_SIZE(psensor_attributes); i++){
		if (device_create_file(ap3045_dev_psensor, psensor_attributes + i)) {
			LDBG("ap3045_create_file_error \n");
			goto ap3045_create_file_error;
		}
	}

	return 0;

ap3045_create_file_error:

	for ( ; i >= 0; i--)
	device_remove_file(ap3045_dev_lsensor, lsensor_attributes + i);
	for ( ; i >= 0; i--)
	device_remove_file(ap3045_dev_psensor, psensor_attributes + i);

ap3045_device_error:
	class_destroy(ap3045_class);
ap3045_class_error:
	dev_err(&sensor->client->dev, "%s:Unable to create interface\n", __func__);
	return -1;
}

static int ap3045_init_client(struct i2c_client *client)
{
	struct ap3045_data *data = i2c_get_clientdata(client);
	int i;
	if(DEBUG_FUN) LDBG(" \n");
	/* set defaults */
	__ap3045_write_reg(client, 0x00,0xFF, 0x00, 0x04);

	/* read all the registers once to fill the cache.
	* if one of the reads fails, we consider the init failed */
	for (i = 0; i < reg_num; i++) {
		int v = i2c_smbus_read_byte_data(client, reg_array[i]);
		if (v < 0)
		return -ENODEV;

		data->reg_cache[i] = v;
	}
	__ap3045_write_reg(data->client, AP3045_REG_SYS_INTSTATUS,
	0xFF, 0x00, 0);

	return 0;
}

static void pl_timer_callback(unsigned long pl_data)
{
	int ret =0;
	if(DEBUG_FUN) LDBG(" \n");
	queue_work(PL_Driver_Data->plsensor_wq, &PL_Driver_Data->plsensor_work);

	ret = mod_timer(&PL_Driver_Data->pl_timer, jiffies + msecs_to_jiffies(PL_TIMER_DELAY));
	if(ret)
	LDBG("Timer Error\n");
}

static void plsensor_work_handler(struct work_struct *w)
{
	struct ap3045_data *data = PL_Driver_Data;
	u8 int_stat;
	int obj,ret;
	int report_lux;
	unsigned char INT_Reg;
	if(DEBUG_FUN) LDBG(" \n");
	int_stat = ap3045_get_intstat(data->client);
	INT_Reg = int_stat;
	if(DEBUG_VALUE) LDBG("int_stat = %d \n", int_stat);
	if (int_stat & AP3045_REG_SYS_INT_AL_MASK)
	INT_Reg &= 0xFE;
	if (int_stat & AP3045_REG_SYS_INT_PS_MASK)
	INT_Reg &= 0xFD;
	if (int_stat & AP3045_REG_SYS_INT_HMASK)
	INT_Reg &= 0xFB;
	if(DEBUG_VALUE) LDBG("INT_Reg = %d \n", INT_Reg);
	if (INT_Reg != int_stat){
		ret = __ap3045_write_reg(data->client, AP3045_REG_SYS_INTSTATUS,
		0xFF, 0x00, INT_Reg);
	}
	// ALS int
	if (misc_ls_opened == 1){
		if(DEBUG_VALUE) LDBG("misc_ls_opened: %d\n", misc_ls_opened);
#ifdef INTERRUPT_MODE

		if (int_stat & AP3045_REG_SYS_INT_AL_MASK)
		{
			report_lux = ap3045_get_adc_value(data->client);
			ap3045_change_ls_threshold(data->client,report_lux);
			if(EnLSensorConfig_flag) {
				report_lux = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux);
			}
			input_report_abs(data->ls_input_dev, ABS_MISC, report_lux);
			input_sync(data->ls_input_dev);
			LDBG("report lux: %d\n", report_lux);
		}

#else
		report_lux = ap3045_get_adc_value(data->client);
		if(EnLSensorConfig_flag) {
			report_lux = calibration_light_ap3045(lSensor_CALIDATA[1], lSensor_CALIDATA[0], report_lux);
		}
		input_report_abs(data->ls_input_dev, ABS_MISC, report_lux);
		input_sync(data->ls_input_dev);
		LDBG("report lux: %d\n", report_lux);
#endif
	}
	// PX int
	if (misc_ps_opened == 1){
#ifdef INTERRUPT_MODE
		if (int_stat & AP3045_REG_SYS_INT_PS_MASK)
		{
			if(DEBUG_VALUE) LDBG("AP3045_REG_SYS_INT_PS_MASK\n");
			obj = ap3045_get_object(data->client);
			input_report_abs(data->ps_input_dev, ABS_DISTANCE, !obj);
			input_sync(data->ps_input_dev);
			LDBG("report ps_value= %d \n", ap3045_get_px_value(data->client));
			LDBG("report = %s\n",  obj ? " near":" far");
		}
#else
		if(DEBUG_VALUE) LDBG("AP3045_REG_SYS_INT_PS_MASK\n");
		obj = ap3045_get_object(data->client);
		input_report_abs(data->ps_input_dev, ABS_DISTANCE, !obj);
		input_sync(data->ps_input_dev);
		LDBG("report ps_value= %d \n", ap3045_get_px_value(data->client));
		LDBG("report = %s\n",  obj ? " near":" far");
#endif
	}
#ifdef INTERRUPT_MODE
	enable_irq(data->irq);
#endif
}

static irqreturn_t ap3045_irq_handler(int irq, void *data_)
{
	struct ap3045_data *data= PL_Driver_Data;
	if(DEBUG_FUN) LDBG(" \n");
	disable_irq_nosync(data->irq);
	queue_work(data->plsensor_wq, &ap3045_irq_work);
	return IRQ_HANDLED;
}

static int ap3045_lightsensor_proc_show(struct seq_file *m, void *v)
{
	int ret = 0,idreg;
	if(DEBUG_FUN) LDBG(" \n");
	idreg = i2c_smbus_read_byte_data(PL_Driver_Data->client, AP3045_REG_ALS_TIME);
	ret =seq_printf(m," addr:(0x%x)\n",PL_Driver_Data->client->addr);
	if(idreg<0){
		ret =seq_printf(m," ERROR: i2c r/w test fail\n");
	}else{
		ret =seq_printf(m," ACK: i2c r/w test ok\n");
	}
	if(ret<0){
		ret =seq_printf(m," %s light status:error\n",AP3045_DRV_NAME);
	} else {
		ret =seq_printf(m," %s light status:ok\n",AP3045_DRV_NAME);
	}
	return ret;
}

static int ap3045_Proximitysensor_proc_show(struct seq_file *m, void *v)
{
	int ret = 0,idreg;
	if(DEBUG_FUN) LDBG(" \n");
	idreg = i2c_smbus_read_byte_data(PL_Driver_Data->client, AP3045_REG_PS_LED_CON);
	ret =seq_printf(m," addr:(0x%x)\n",PL_Driver_Data->client->addr);
	if(idreg<0){
		ret =seq_printf(m," ERROR: i2c r/w test fail\n");
	}else{
		ret =seq_printf(m," ACK: i2c r/w test ok\n");
	}

	if(ret<0){
		ret =seq_printf(m,"%s Proximity status:error\n",AP3045_DRV_NAME);
	} else {
		ret =seq_printf(m,"%s Proximity status:ok\n",AP3045_DRV_NAME);
	}
   	return ret;
}

static int ap3045_lightsensor_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ap3045_lightsensor_proc_show, NULL);
}

static int ap3045_Proximitysensor_proc_open(struct inode *inode, struct  file *file)
{
	return single_open(file, ap3045_Proximitysensor_proc_show, NULL);
}

static const struct file_operations ap3045_lightsensor_proc_fops = {
	.owner = THIS_MODULE,
	.open = ap3045_lightsensor_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static const struct file_operations ap3045_Proximitysensor_proc_fops = {
	.owner = THIS_MODULE,
	.open = ap3045_Proximitysensor_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int ap3045_setup(struct i2c_client *client,struct ap3045_data *data)
{
	int ret = 0,gpio;
	if(DEBUG_FUN) LDBG(" \n");

	gpio=AP3045_GPIO;
	ret = gpio_request(gpio, "ap3045_gpio");

	if (ret < 0) {
		LDBG("gpio %d request failed (%d)\n", gpio, ret);
		return ret;
	}
	ret = gpio_direction_input(gpio);
	if (ret < 0) {
		LDBG( "fail to set gpio %d as input (%d)\n", gpio, ret);
		goto fail_free_intr_pin;
	}
	data->irq =  gpio_to_irq(gpio);

	if (ret < 0) {
		LDBG( "req_irq(%d) fail for gpio %d (%d)\n", client->irq, gpio, ret);
		goto fail_free_intr_pin;
	}

	return ret;
fail_free_intr_pin:
	gpio_free(gpio);

	return ret;
}

static int psensor_calibration_read(struct seq_file *m, void *v)
{
	return seq_printf(m, "ps_close_thd_set:%d ,ps_away_thd_set:%d \n",pSensor_CALIDATA[0],pSensor_CALIDATA[1]);
}

static int psensor_calibration_open(struct inode *inode, struct  file *file)
{
  return single_open(file, psensor_calibration_read, NULL);
}

static const struct file_operations proc_psensor_calibration_send = {
	.owner = THIS_MODULE,
	.open = psensor_calibration_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int psensor_auto_calibration_show(struct seq_file *m, void *v)
{
	int i,CalidataX,CalidataL,CalidataH,ps_raw_value=0,tmplen=0;
	struct file *filp = NULL;
	char  tmpbuf[20] = "";
	mm_segment_t old_fs;

	if(DEBUG_FUN) LDBG(" \n");

	ap3045_psensor_enable(PL_Driver_Data->client);

	ps_raw_value=ap3045_get_px_value(PL_Driver_Data->client);
	for (i=0;i<5;i++) {
		CalidataX=ap3045_get_px_value(PL_Driver_Data->client);
		if(CalidataX>ap3045_get_px_value(PL_Driver_Data->client)) {
			ps_raw_value=CalidataX;
		}
	}
	if(ps_raw_value==0 && Read_HW_ID()!=HW_ID_EVB ) {
		LDBG("driver not ready");
		return seq_printf(m, "0");
	}

	LDBG("psensor_calibration ps_raw_value:%d \n",  ps_raw_value);
	LDBG("PS_CLOSE_THD = (%d), PS_AWAY_THD = (%d)\n", PS_CLOSE_THD , PS_AWAY_THD);
	// close_thd_value ,away_thd_value
	pSensor_CALIDATA[0] = (abs(ps_raw_value/3)+PS_CLOSE_THD)+ps_raw_value;
	pSensor_CALIDATA[1] = (abs(ps_raw_value/6)+PS_AWAY_THD)+ps_raw_value;
	ap3045_set_phthres(PL_Driver_Data -> client, pSensor_CALIDATA[0] );
	ap3045_set_plthres(PL_Driver_Data -> client, pSensor_CALIDATA[1] );
	CalidataX=ps_raw_value-(abs(ps_raw_value/10));
	CalidataL=pSensor_CALIDATA[1];
	CalidataH=pSensor_CALIDATA[0];
	LDBG("CalidataX = (%d), CalidataH = (%d), CalidataL = (%d)\n", CalidataX , CalidataH ,CalidataL);
	//save calibration
	filp = filp_open(PS_CAL_PATH, O_RDWR | O_CREAT,0660);
	if (IS_ERR(filp)) {
		LDBG("can't open %s \n",PS_CAL_PATH);
		return seq_printf(m, "0");
	} else {
		sprintf(tmpbuf,"%d %d %d ", CalidataX,CalidataH,CalidataL);
		tmplen=strlen(tmpbuf);
		LDBG("tmpbuf:%s , tmplen:%d  \n",tmpbuf, tmplen);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		filp->f_op->write(filp, tmpbuf,  tmplen, &filp->f_pos);
		set_fs(old_fs);
		filp_close(filp, NULL);
		LDBG("save %s \n",PS_CAL_PATH);
	}

	return seq_printf(m, "1");
}

static int psensor_auto_calibration_open(struct inode *inode, struct  file *file)
{
  return single_open(file, psensor_auto_calibration_show, NULL);
}

static const struct file_operations proc_psensor_auto_calibration_send = {
	.owner = THIS_MODULE,
	.open = psensor_auto_calibration_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};


static int lpsensor_proc_show(struct seq_file *m, void *v)
{
	if(!ap3045_Lpsensor_Entry)
	return seq_printf(m, "-1\n");
	else
	return seq_printf(m, "1\n");
}

static int lpsensor_proc_open(struct inode *inode, struct  file *file)
{
  return single_open(file, lpsensor_proc_show, NULL);
}

static const struct file_operations lpsensor_proc_fops = {
	.owner = THIS_MODULE,
	.open = lpsensor_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int  ap3045_probe(struct i2c_client *client,const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct ap3045_data *data;
	int err = 0,status = 0;

	if(DEBUG_FUN) LDBG(" \n");

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)){
		LDBG("I2C bus not ready!\n");
		err = -EIO;
		goto err_i2c_bus_fail;
	}

	//check i2c
	status = i2c_smbus_read_byte_data(client, AP3045_REG_SYS_CONF);
	if (status < 0) {
		LDBG("ap3045 is not find!\n");
		err = -ENODEV;
		goto err_ic_fail;
	}

	reg_array = ap3045_reg;
	range = ap3045_range;
	reg_num = AP3045_NUM_CACHABLE_REGS;

	data = kzalloc(sizeof(struct ap3045_data), GFP_KERNEL);
	if (!data){
		err = -ENOMEM;
		goto exit_free_data;
	}

	data->client = client;
	i2c_set_clientdata(client, data);

	// initialize the AP3045 chip
	err = ap3045_init_client(client);
	if (err)
	goto exit_kfree;

	err =ap3045_setup(client,data);
	if (err) {
		LDBG("ap3045_setup error!\n");
		goto exit_kfree;
	}
	err = ap3045_als_init(client);
	if (err)
	goto exit_kfree;
	err = ap3045_ps_init(client);
	if (err)
	goto exit_kfree;

	err = ap3045_register_lsensor_device(client,data);
	if (err){
		dev_err(&client->dev, "failed to register_lsensor_device\n");
		goto exit_kfree;
	}

	err = ap3045_register_psensor_device(client, data);
	if (err) {
		dev_err(&client->dev, "failed to register_psensor_device\n");
		goto exit_free_ls_device;
	}

	err = create_sysfs_interfaces(data);
	if (err)
	goto exit_free_ps_device;

#ifdef CONFIG_HAS_EARLYSUSPEND
	ap3045_early_suspend.suspend = ap3045_suspend;
	ap3045_early_suspend.resume  = ap3045_resume;
	ap3045_early_suspend.level   = 0x02;
	register_early_suspend(&ap3045_early_suspend);
#endif
	data->plsensor_wq = create_singlethread_workqueue("plsensor_wq");
	if (!data->plsensor_wq) {
		LDBG("create workqueue failed\n");
		err = -ENOMEM;
		goto err_create_wq_failed;
	}

#ifdef INTERRUPT_MODE
	PL_Driver_Data = data;
	err = request_irq(data->irq, ap3045_irq_handler, IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"ap3045_irq", data);

	if (err < 0) {
		LDBG("request_irq req_irq(%d) (%d)\n", data->irq , err);
		goto fail_free_intr_pin;
	}
#else
	LDBG("Timer module installing\n");
	setup_timer(&data->pl_timer, pl_timer_callback, 0);
	INIT_WORK(&data->plsensor_work, plsensor_work_handler);
	PL_Driver_Data = data;
#endif

	dev_info(&client->dev, "Driver version %s enabled\n", DRIVER_VERSION);
	ap3045_psensor_auto_calibration_entry = proc_create("psensor_auto_calibration", ATTRIBUTES_PERMISSION, NULL, &proc_psensor_auto_calibration_send);
	if (!ap3045_psensor_auto_calibration_entry) {
		LDBG("can't to create ap3045_psensor_auto_calibration_entry\n");
	}

#ifdef CONFIG_ASUS_FACTORY_SENSOR_MODE
	ap3045_psensor_calibration_entry = proc_create("psensor_calibration_data", ATTRIBUTES_PERMISSION, NULL, &proc_psensor_calibration_send);
	if (!ap3045_psensor_auto_calibration_entry) {
		LDBG("can't to create ap3045_psensor_auto_calibration_entry\n");
	}
	ap3045_lightsensor_entry = proc_create("lightsensor_status", ATTRIBUTES_PERMISSION, NULL,&ap3045_lightsensor_proc_fops);
	if(!ap3045_lightsensor_entry) {
		LDBG("can't to create ap3045_lightsensor_entry\n");
	}
	ap3045_proximitysensor_entry = proc_create("proximitysensor_status", ATTRIBUTES_PERMISSION, NULL,&ap3045_Proximitysensor_proc_fops);
	if(!ap3045_proximitysensor_entry) {
		LDBG("can't to create ap3045_proximitysensor_entry\n");
	}
   ap3045_Lpsensor_Entry = proc_create("lpsensor_status", ATTRIBUTES_PERMISSION, NULL,&lpsensor_proc_fops);
	if(!ap3045_Lpsensor_Entry) {
		LDBG("can't to create ap3045_Lpsensor_Entry\n");
	}
#endif

	LDBG("success \n");

	return 0;
fail_free_intr_pin:
	gpio_free(data->gpio);

err_create_wq_failed:
#ifndef INTERRUPT_MODE
	if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
#endif
	if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);


exit_free_ps_device:
	ap3045_unregister_psensor_device(client,data);


exit_free_ls_device:
	ap3045_unregister_lsensor_device(client,data);

exit_kfree:
	kfree(data);

exit_free_data:

err_ic_fail:

err_i2c_bus_fail:

	return err;
}

static int  ap3045_remove(struct i2c_client *client)
{
	struct ap3045_data *data = i2c_get_clientdata(client);
	if(DEBUG_FUN) LDBG(" \n");
	free_irq(data->irq, data);

	ap3045_unregister_psensor_device(client,data);
	ap3045_unregister_lsensor_device(client,data);

#ifdef CONFIG_HAS_EARLYSUSPEND
	unregister_early_suspend(&ap3045_early_suspend);
#endif

	ap3045_set_mode(data->client, 0);
	kfree(i2c_get_clientdata(client));

	if (data->plsensor_wq)
	destroy_workqueue(data->plsensor_wq);
#ifndef INTERRUPT_MODE
	if(&data->pl_timer != NULL)
	del_timer(&data->pl_timer);
#endif
	return 0;
}

static const struct i2c_device_id ap3045_id[] = {
	{ AP3045_DRV_NAME, 0 },
	{}
};

MODULE_DEVICE_TABLE(i2c, ap3045_id);

static struct i2c_driver ap3045_driver = {
	.driver = {
		.name	= AP3045_DRV_NAME,
		.owner	= THIS_MODULE,
	},
	.probe	= ap3045_probe,
	.remove	= ap3045_remove,
	.id_table = ap3045_id,
};

static int __init ap3045_init(void)
{
	int ret;
	if(DEBUG_FUN) LDBG(" \n");
	ret = i2c_add_driver(&ap3045_driver);
	if (ret) {
		LDBG("i2c_add_driver can't add ap3045_driver\n");
		i2c_del_driver(&ap3045_driver);
		return ret;
	}
	if(ap3045_psensor_auto_calibration_entry)
		remove_proc_entry("psensor_auto_calibration", NULL);

#ifdef CONFIG_ASUS_FACTORY_SENSOR_MODE
	if(ap3045_psensor_calibration_entry)
		remove_proc_entry("psensor_calibration_data", NULL);
	if (ap3045_Lpsensor_Entry)
		remove_proc_entry("lpsensor_status", NULL);
	if(ap3045_lightsensor_entry)
		remove_proc_entry("proximitysensor_status", NULL);
	if(ap3045_proximitysensor_entry)
		remove_proc_entry("lightsensor_status", NULL);
#endif
	return ret;
}

static void __exit ap3045_exit(void)
{
	i2c_del_driver(&ap3045_driver);
}

MODULE_DESCRIPTION("AP3045 driver.");
MODULE_LICENSE("GPL v2");
MODULE_VERSION(DRIVER_VERSION);

module_init(ap3045_init);
module_exit(ap3045_exit);