/*
 * Copyright (C) 2012 Kionix, Inc.
 * Written by Chris Hudson <chudson@kionix.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA
 * 02111-1307, USA
 */

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/slab.h>
#include <linux/version.h>
#include <linux/kxtj9.h>
#include <linux/input-polldev.h>
#include <linux/miscdevice.h>
#include <linux/HWVersion.h>
extern int Read_PROJ_ID(void);
//20131224 ASUS-Eve_Wen add for IOCTL
#include <linux/proc_fs.h>
#include <asm/intel-mid.h>
#include <linux/mutex.h>
//#include <asm/intel_scu_ipcutil.h>
//#include <asm/intel_scu_pmic.h>



// 2012.11.30 cheng_kao early_suspend ++
#include <linux/earlysuspend.h>
// 2012.11.30 cheng_kao early_suspend --
//<-- ASUS-Bevis_Chen + -->
#if LINUX_VERSION_CODE>=KERNEL_VERSION(3,8,0)
#define __devinit
#define __devexit_p(x) x
#endif

bool enAccelConfig_flag;
int Max_x,Min_x,Max_y,Min_y,Max_z,Min_z; //Calibration file  input value
int ACCEL_CALIDATA[6] = {0}; //input calibration data . Format : "Xmax, Xmin, Ymax, Ymin, Zmax, Zmin"
struct i2c_client * ex_client;
	 /* IOCTLs  library */
#define ASUS_GSENSOR_IOCTL_MAGIC			'a'
#define GBUFF_SIZE				12	/* Rx buffer size */
#define ASUS_GSENSOR_SETCALI_DATA		        _IOW(ASUS_GSENSOR_IOCTL_MAGIC, 0x10, int[6])
#define ASUS_GSENSOR_ENACCEL_CALIBRATION      _IOW(ASUS_GSENSOR_IOCTL_MAGIC, 0x11, char)
#define MSIC_VPROG2_MRFLD_CTRL	0xAD


struct proc_dir_entry *gsensor_entry = NULL;
int enAccelConfig_func(int raw_result,int Max,int Min);

#if 0
static int asus_gsensor_status_read(char *buffer, char **buffer_location,
							off_t offset, int buffer_length, int *eof, void *data)
{	
	if(!gsensor_entry)
	return sprintf(buffer, "-1\n");
	else
	return sprintf(buffer, "1\n");;
}
#endif
static long asus_gsensor_ioctl(struct file *file,
			  unsigned int cmd, unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	int result = 0;
	char encalibration_flag = 0 ;
//	int rawdata_x=0,rawdata_y=0,rawdata_z=0,err=0;
//	int reportdata_x=0,reportdata_y=0,reportdata_z=0;
//	char value = 0;
	if(ex_client == NULL){
		printk("%s: ERROR ! ASUS GSENSOR ioctl ex_client is NULL  \n", __func__);
		return -EFAULT;
		}
//	struct kxtj9_data *tj9 = i2c_get_clientdata(ex_client);
	switch (cmd) {
	case ASUS_GSENSOR_SETCALI_DATA:
		memset(ACCEL_CALIDATA, 0, 6*sizeof(int));
		if (copy_from_user(ACCEL_CALIDATA, argp, sizeof(ACCEL_CALIDATA)))
		{
			result = -EFAULT;
			goto error;
		}	
		printk("%s:ASUS_SETCALI_DATA : MAX_x:  %d , Min_x:  %d , MAX_y:  %d , Min_y:  %d ,MAX_z:  %d , Min_z:  %d \n", 
			__func__, ACCEL_CALIDATA[0],ACCEL_CALIDATA[1],ACCEL_CALIDATA[2], \
			ACCEL_CALIDATA[3],ACCEL_CALIDATA[4],ACCEL_CALIDATA[5]);

					 Max_x =  ACCEL_CALIDATA[0];
					 Min_x  =  ACCEL_CALIDATA[1];
					 Max_y =  ACCEL_CALIDATA[2];
					 Min_y  =  ACCEL_CALIDATA[3];
					 Max_z =  ACCEL_CALIDATA[4];
					 Min_z  =  ACCEL_CALIDATA[5];
		break;
		
	case ASUS_GSENSOR_ENACCEL_CALIBRATION:
		if (copy_from_user(&encalibration_flag, argp, sizeof(encalibration_flag)))
		{
			result = -EFAULT;
			goto error;
		}	
		enAccelConfig_flag =  encalibration_flag;
		break;
	default:
		result = -ENOTTY;
	goto error;
	}

error:
	return result;
}
struct file_operations asus_gsensor_fops = {
	.owner = THIS_MODULE,
	.unlocked_ioctl = asus_gsensor_ioctl,
};

struct miscdevice asus_gsensor_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "asus_gsensor",
	.fops = &asus_gsensor_fops,
};

int create_asusproc_gsensor_status_entry( void )
{   
    gsensor_entry = proc_create("asus_gsensor_a500_status", S_IWUGO| S_IRUGO, NULL, &asus_gsensor_fops);
    if (!gsensor_entry)
        return -ENOMEM;
   	// gsensor_entry->read_proc = asus_gsensor_status_read;
    	return 0;
}
//<-- ASUS-Bevis_Chen - -->

#define NAME			"kxtj9"
#define G_MAX			8000
/* OUTPUT REGISTERS */
#define XOUT_L			0x06
#define WHO_AM_I		0x0F
/* CONTROL REGISTERS */
#define INT_REL			0x1A
#define CTRL_REG1		0x1B
#define INT_CTRL1		0x1E
#define DATA_CTRL		0x21
/* CONTROL REGISTER 1 BITS */
#define PC1_OFF			0x7F
#define PC1_ON			(1 << 7)
/* Data ready funtion enable bit: set during probe if using irq mode */
#define DRDYE			(1 << 5)
/* DATA CONTROL REGISTER BITS */
#define ODR3_125F		0x0A
#define ODR6_25F		0x0B
#define ODR12_5F		0
#define ODR25F			1
#define ODR50F			2
#define ODR100F			3
#define ODR200F			4
#define ODR400F			5
#define ODR800F			6
#define ODR1600F		7
/* INTERRUPT CONTROL REGISTER 1 BITS */
/* Set these during probe if using irq mode */
#define KXTJ9_IEL		(1 << 3)
#define KXTJ9_IEA		(1 << 4)
#define KXTJ9_IEN		(1 << 5)
/* INPUT_ABS CONSTANTS */
#define FUZZ			3
#define FLAT			3
/* RESUME STATE INDICES */
#define RES_DATA_CTRL		0
#define RES_CTRL_REG1		1
#define RES_INT_CTRL1		2
#define RESUME_ENTRIES		3

//Use to open/close the debugmessage
#define KXTJ9_DEBUG_MESSAGE		1
#define KXTJ9_CALIBRATED_MESSAGE	1

//use to define raw data by the chip location
//#define KXTJ9_CHIP_LOCATION_EVB   	0
//#define KXTJ9_CHIP_LOCATION_SR		1
//#define KXTJ9_CHIP_LOCATION_PRE_ER      2
//>>ASUS-Eve_Wen 20131115
//#define HW_ID_EVB   KXTJ9_CHIP_LOCATION_EVB
//#define HW_ID_SR    KXTJ9_CHIP_LOCATION_SR
//#define HW_ID_PRE_ER KXTJ9_CHIP_LOCATION_PRE_ER
//<<ASUS-Eve_Wen 20131115
//int g_ilocation=0;

// added by cheng_kao 2013.06.01  for sensors calibration ++
#define GSENSOR_CALIBRATION_FILE_PATH	"/data/sensors/accel_cal_data.ini"
// added by cheng_kao 2013.06.01  for sensors calibration --
//
extern int Read_HW_ID(); //20131118 ASUS-Eve_Wen for SR HW ID

// define the GPIO PIN for Intel x86

static bool canEnableGsensor = true;
//int i2c_bus =    5;
//module_param(gpio_line, int, S_IRUGO);
//module_param(i2c_bus, int, S_IRUGO);

//The following table lists the maximum appropriate poll interval for each available output data rate.
static const struct {
	unsigned int cutoff;
	u8 mask;
} kxtj9_odr_table[] = {						// ms,	range,	mode
	{ 15,			ODR200F },			// 2.5,	6~ 10	FASTEST MODE , full power mode
	{ 35,			ODR50F },			// 20,	21~30	GAME MODE
	{ 70,			ODR25F  },			// 70,	31~70	UI MODE
	{ 250,			ODR12_5F  },		// 160,	71~250	NORMAL MODE
	{ 0xFFFFFFFF,	ODR12_5F},			// 160,	251~max	NO POLL
//	{ 0xFFFFFFFF,	ODR3_125F},			// 320,	251~max	NO POLL
};
/*	default
 kxtj9_odr_table[] = {
	{ 1,		ODR1600F },
	{ 3,		ODR800F },
	{ 5,		ODR400F },
	{ 10,	ODR200F },
	{ 20,	ODR100F },
	{ 40,	ODR50F  },
	{ 80,	ODR25F  },
	{ 0xFFFFFFFF,	ODR12_5F},
};
*/
struct kxtj9_data {
	struct i2c_client *client;
	struct kxtj9_platform_data pdata;
	struct input_dev *input_dev;
#if 0
#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
	struct input_polled_dev *poll_dev;
#endif
#endif
	unsigned int last_poll_interval;
	u8 shift;
	u8 ctrl_reg1;
	u8 data_ctrl;
	u8 int_ctrl;
	atomic_t enabled;
// added by cheng_kao 2013.06.01  for sensors calibration ++
	int accel_cal_data[6];
	int accel_cal_offset[3];
	int accel_cal_sensitivity[3];
// added by cheng_kao 2013.06.01  for sensors calibration --
#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND	
	struct early_suspend kxtj9_early_suspendresume;
#endif
#endif
	int irq;
	struct mutex mutex;
};

struct kxtj9_data *g_kxtj9_data = NULL;

#if 0
static int kxtj9_i2c_read(struct kxtj9_data *tj9, u8 addr, u8 *data, int len)
{
	struct i2c_msg msgs[] = {
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags,
			.len = 1,
			.buf = &addr,
		},
		{
			.addr = tj9->client->addr,
			.flags = tj9->client->flags | I2C_M_RD,
			.len = len,
			.buf = data,
		},
	};

	return i2c_transfer(tj9->client->adapter, msgs, 2);
}

static void kxtj9_report_acceleration_data(struct kxtj9_data *tj9)
{
	unsigned char acc_data[6];
	int rawdata_x=0,rawdata_y=0,rawdata_z=0,err=0;
	int reportdata_x=0,reportdata_y=0,reportdata_z=0;
        int direction_x=1, direction_y=1, direction_z=1;
	err = kxtj9_i2c_read(tj9, XOUT_L, (u8 *)acc_data, 6);
//	printk("Eve_Wen kxtj9_report_acceleration_data 1 \n");
	if (err < 0){
		dev_err(&tj9->client->dev, "accelerometer data read failed\n");
		return;
	}
//	printk("Eve_Wen kxtj9_report_acceleration_data 2 \n");
	rawdata_x = ( (acc_data[0]>>4) | (acc_data[1]<<4) );
	rawdata_y = ( (acc_data[2]>>4) | (acc_data[3]<<4) );
	rawdata_z = ( (acc_data[4]>>4) | (acc_data[5]<<4) );

	if(rawdata_x>2047)
		rawdata_x = rawdata_x-4096;
	if(rawdata_y>2047)
		rawdata_y = rawdata_y-4096;
	if(rawdata_z>2047)
		rawdata_z = rawdata_z-4096;

//	reportdata_x = rawdata_x;
//	reportdata_y = rawdata_y;
//	reportdata_z = rawdata_z;
	
	// transfromed by chip location
	switch(g_ilocation){
		/*case KXTJ9_CHIP_LOCATION_EVB :
			if( (tj9->accel_cal_sensitivity[0]==0)||(tj9->accel_cal_sensitivity[1]==0)||(tj9->accel_cal_sensitivity[2]==0) ){
				reportdata_x = (-1)*rawdata_x;
				reportdata_y = (-1)*rawdata_y;
				reportdata_z = rawdata_z;
				if(KXTJ9_DEBUG_MESSAGE) printk("KXTJ9_CHIP_LOCATION_EVB 1\n");
			}
			else{
				reportdata_x = (-1)*1024*(rawdata_x - tj9->accel_cal_offset[0])/tj9->accel_cal_sensitivity[0];
				reportdata_y = (-1)*1024*(rawdata_y - tj9->accel_cal_offset[1])/tj9->accel_cal_sensitivity[1];
				reportdata_z = 1024*(rawdata_z - tj9->accel_cal_offset[2])/tj9->accel_cal_sensitivity[2];
				if (enAccelConfig_flag == 1)
				{
					reportdata_x =enAccelConfig_func(reportdata_x,Max_x,Min_x); //use calibration data without reboot	
					reportdata_y =enAccelConfig_func(reportdata_y,Max_y,Min_y);
					reportdata_z =enAccelConfig_func(reportdata_z,Max_z,Min_z);
					//printk("Eve_Wen cali evb\n");
				}
				//if(KXTJ9_DEBUG_MESSAGE) printk("KXTJ9_CHIP_LOCATION_EVB 2 \n");
			}
	break;

		case KXTJ9_CHIP_LOCATION_SR :
		case KXTJ9_CHIP_LOCATION_PRE_ER:
			if( (tj9->accel_cal_sensitivity[0]==0)||(tj9->accel_cal_sensitivity[1]==0)||(tj9->accel_cal_sensitivity[2]==0) ){
				reportdata_y = rawdata_x*(-1);
				reportdata_x = rawdata_y;
				reportdata_z = rawdata_z;
				if(KXTJ9_DEBUG_MESSAGE) printk("KXTJ9_CHIP_LOCATION_SR or PRE ER 1\n");
			}
			else{
				reportdata_y = (-1024)*(rawdata_x - tj9->accel_cal_offset[0])/tj9->accel_cal_sensitivity[0];
				reportdata_x = 1024*(rawdata_y - tj9->accel_cal_offset[1])/tj9->accel_cal_sensitivity[1];
				reportdata_z = 1024*(rawdata_z - tj9->accel_cal_offset[2])/tj9->accel_cal_sensitivity[2];
				if (enAccelConfig_flag == 1)
				{
					reportdata_x =enAccelConfig_func(reportdata_x,Max_x,Min_x); //use calibration data without reboot	
					reportdata_y =enAccelConfig_func(reportdata_y,Max_y,Min_y);
					reportdata_z =enAccelConfig_func(reportdata_z,Max_z,Min_z);
					//printk("Eve_Wen Max_x = %d, Min_x = %d, Max_y = %d, Min_y = %d, Max_z= %d, Min_z = %d\n", Max_x, Min_x, Max_y, Min_y, Max_z, Min_z);
					//printk("Eve_Wen cali sr pre_er \n");
				}
				//if(KXTJ9_DEBUG_MESSAGE) printk("KXTJ9_CHIP_LOCATION_SR 2\n");
			}
		break;*/
                case PROJ_ID_A502CG:
                        direction_y = -1;
                        break; 

		default:
                        direction_x = -1;
	                break;
	}

			if( (tj9->accel_cal_sensitivity[0]==0)||(tj9->accel_cal_sensitivity[1]==0)||(tj9->accel_cal_sensitivity[2]==0) ){
				reportdata_y = rawdata_x*direction_x;
				reportdata_x = rawdata_y*direction_y;
				reportdata_z = rawdata_z*direction_z;
				if(KXTJ9_DEBUG_MESSAGE) printk("KXTJ9_CHIP_LOCATION_default 1\n");
			}
			else{
				reportdata_y = 1024*direction_x*(rawdata_x - tj9->accel_cal_offset[0])/tj9->accel_cal_sensitivity[0];
				reportdata_x = 1024*direction_y*(rawdata_y - tj9->accel_cal_offset[1])/tj9->accel_cal_sensitivity[1];
				reportdata_z = 1024*direction_z*(rawdata_z - tj9->accel_cal_offset[2])/tj9->accel_cal_sensitivity[2];
				if (enAccelConfig_flag == 1)
				{
					reportdata_x =enAccelConfig_func(reportdata_x,Max_x,Min_x); //use calibration data without reboot	
					reportdata_y =enAccelConfig_func(reportdata_y,Max_y,Min_y);
					reportdata_z =enAccelConfig_func(reportdata_z,Max_z,Min_z);
					//printk("Eve_Wen cali default \n");
				}
				//if(KXTJ9_DEBUG_MESSAGE) printk("KXTJ9_CHIP_LOCATION_SR 2\n");
			}
	

//	if(KXTJ9_DEBUG_MESSAGE) printk("report_acceleration data : (%d), (%d), (%d)\n",reportdata_x, reportdata_y, reportdata_z);

	input_report_abs(tj9->input_dev, ABS_X, reportdata_x);
	input_report_abs(tj9->input_dev, ABS_Y, reportdata_y);
	input_report_abs(tj9->input_dev, ABS_Z, reportdata_z);
	input_sync(tj9->input_dev);
}

static irqreturn_t kxtj9_isr(int irq, void *dev)
{
	struct kxtj9_data *tj9 = dev;
	int err;

	/* data ready is the only possible interrupt type */
//	printk("Eve_Wen in irqreturn_t kxtj9_isr \n");
	kxtj9_report_acceleration_data(tj9);
	err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
	if (err < 0)
		dev_err(&tj9->client->dev,
			"error clearing interrupt status: %d\n", err);

	// enable_irq(tj9->irq); 

	return IRQ_HANDLED;
}
#endif
/*
static int kxtj9_update_g_range(struct kxtj9_data *tj9, u8 new_g_range)
{

	switch (new_g_range) {
	case KXTJ9_G_2G:
		tj9->shift = 4;
		break;
	case KXTJ9_G_4G:
		tj9->shift = 3;
		break;
	case KXTJ9_G_8G:
		tj9->shift = 2;
		break;
	default:
		tj9->shift = 4;
		new_g_range = KXTJ9_G_2G;
//		return -EINVAL;
	}

	tj9->ctrl_reg1 &= 0xe7;
	tj9->ctrl_reg1 |= new_g_range;

	return 0;
}
*/
static int kxtj9_update_odr(struct kxtj9_data *tj9, unsigned int poll_interval)
{
	int err;
	int i;

	/* Use the lowest ODR that can support the requested poll interval */
	for (i = 0; i < ARRAY_SIZE(kxtj9_odr_table); i++) {
		if (poll_interval < kxtj9_odr_table[i].cutoff)
			break;
	}
	tj9->data_ctrl = kxtj9_odr_table[i].mask;
	if(KXTJ9_DEBUG_MESSAGE)
		printk("alp : kxtj9_update_odr  i(%d), cutoff(%d), mask(%d)\n", i , kxtj9_odr_table[i].cutoff, kxtj9_odr_table[i].mask);
	if(KXTJ9_DEBUG_MESSAGE)
		printk("alp : data_ctrl(%x), ctrl_reg(%x)\n",tj9->data_ctrl,tj9->ctrl_reg1);

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, DATA_CTRL, tj9->data_ctrl);
	if (err < 0)
		return err;

	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	return 0;
}

/*
static int kxtj9_device_power_on(struct kxtj9_data *tj9)
{
	if (tj9->pdata.power_on)
		return tj9->pdata.power_on();
	return 0;
}
*/

static void kxtj9_device_power_off(struct kxtj9_data *tj9)
{
	int err;
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_device_power_off ++\n");
	tj9->ctrl_reg1 &= PC1_OFF;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		dev_err(&tj9->client->dev, "soft power off failed\n");

//	if (tj9->pdata.power_off)
//		tj9->pdata.power_off();
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_device_power_off --\n");
}


#if 0
static int __devinit kxtj9_verify(struct kxtj9_data *tj9)
{
	int retval=0;
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_verify ++\n");

//	retval = kxtj9_device_power_on(tj9);
//	if (retval < 0)
//		return retval;

	retval = i2c_smbus_read_byte_data(tj9->client, WHO_AM_I);
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_verify ret = %d\n",retval);
	if (retval < 0) {
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : read fail ret = %d\n", retval);
		dev_err(&tj9->client->dev, "read err int source\n");
		goto out;
	}
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : read retval = %d\n", retval);

	if ((retval != 0x04) && (retval != 0x07) && (retval != 0x08) && (retval != 0x0f) && (retval != 0x09)) {
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : whoami = %d\n", retval);
		pr_warn("kxtj9 whoami = %d\n", retval);
		retval = -EIO;
	} else
		retval = 0;
out:
	kxtj9_device_power_off(tj9);
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_verify --\n");
	return retval;
}
#else

int  kxtj9_verify(void)
{
	int retval=0;
	if(g_kxtj9_data==NULL) return -1;
	
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_verify ++\n");
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : g_kxtj9_data=%x \n",g_kxtj9_data);
//	retval = kxtj9_device_power_on(tj9);
//	if (retval < 0)
//		return retval;

	retval = i2c_smbus_read_byte_data(g_kxtj9_data->client, WHO_AM_I);
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_verify ret = %d\n",retval);
	if (retval < 0) {
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : read fail ret = %d\n", retval);
		printk("read err int source\n");
		goto out;
	}
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : read retval = %d\n", retval);

	if ((retval != 0x04) && (retval != 0x07) && (retval != 0x08) && (retval != 0x0f) && (retval != 0x09)) {
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : whoami = %d\n", retval);
		printk("kxtj9 whoami = %d\n", retval);
		retval = -EIO;
	} else
		retval = 0;
out:
	kxtj9_device_power_off(g_kxtj9_data);
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_verify --\n");
	return retval;
}
EXPORT_SYMBOL(kxtj9_verify);
#endif
static int kxtj9_enable(struct kxtj9_data *tj9)
{
	int err;


//	intel_scu_ipc_msic_vprog1(1);
//	intel_scu_ipc_iowrite8(MSIC_VPROG2_MRFLD_CTRL, 0x41);

//	err = kxtj9_device_power_on(tj9);
//	if (err < 0)
//		return err;
	if(KXTJ9_DEBUG_MESSAGE) printk("alp:kxtj9_enable ++\n");

	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0){
		if(KXTJ9_DEBUG_MESSAGE) printk("alp:i2c error\n");
		return err;
	}
#if 0
	/* only write INT_CTRL_REG1 if in irq mode */
	if (tj9->irq) {
		err = i2c_smbus_write_byte_data(tj9->client,INT_CTRL1, tj9->int_ctrl);
		if (err < 0){
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_enable fail 1 irq = %d\n",tj9->irq);
			return err;
		}
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_enable  irq = %d\n",tj9->irq);
	}
#endif
//	err = kxtj9_update_g_range(tj9, tj9->pdata.g_range);

	/* turn on outputs */
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : turn on ++\n");
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if(KXTJ9_DEBUG_MESSAGE) printk("alp : turn on -- (%d)\n",err);
	if (err < 0)
		return err;

	err = kxtj9_update_odr(tj9, tj9->last_poll_interval);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
#if 0
	if (tj9->irq) {
		err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
		if (err < 0) {
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : error clearing interrupt: %d\n",err)	;
			goto fail;
		}
	}
#endif
	atomic_set(&tj9->enabled, 1);

	return 0;

//fail:
//	kxtj9_device_power_off(tj9);
//	return err;
}

static void kxtj9_disable(struct kxtj9_data *tj9)
{
	atomic_set(&tj9->enabled, 0);
	kxtj9_device_power_off(tj9);
}

static int kxtj9_enable_by_orientation(struct kxtj9_data *tj9)
{
	int err;
	if(KXTJ9_DEBUG_MESSAGE) printk("alp:kxtj9_enable_by_orientation ++\n");
	/* ensure that PC1 is cleared before updating control registers */
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, 0);
	if (err < 0)
		return err;

	/* only write INT_CTRL_REG1 if in irq mode */
#if 0
	if (tj9->irq) {
		err = i2c_smbus_write_byte_data(tj9->client,INT_CTRL1, tj9->int_ctrl);
		if (err < 0){
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_enable fail 1 irq = %d\n",tj9->irq);
			return err;
		}
		if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_enable  irq = %d\n",tj9->irq);
	}
#endif
	/* turn on outputs */
	tj9->ctrl_reg1 |= PC1_ON;
	err = i2c_smbus_write_byte_data(tj9->client, CTRL_REG1, tj9->ctrl_reg1);
	if (err < 0)
		return err;

	/* clear initial interrupt if in irq mode */
#if 0
	if (tj9->irq) {
		err = i2c_smbus_read_byte_data(tj9->client, INT_REL);
		if (err < 0) {
			if(KXTJ9_DEBUG_MESSAGE) printk("alp : error clearing interrupt: %d\n",err)	;
			goto fail;
		}
	}
#endif
	return 0;

//fail:
	//kxtj9_device_power_off(tj9);
	//return err;
}
#if 0
static int kxtj9_input_open(struct input_dev *input)
{
	struct kxtj9_data *tj9 = input_get_drvdata(input);
	int err = -1;
	if(canEnableGsensor == true) err=kxtj9_enable(tj9);
	return err;
}

static void kxtj9_input_close(struct input_dev *dev)
{
	struct kxtj9_data *tj9 = input_get_drvdata(dev);

	kxtj9_disable(tj9);
}

static void __devinit kxtj9_init_input_device(struct kxtj9_data *tj9,
					      struct input_dev *input_dev)
{
	if(KXTJ9_DEBUG_MESSAGE) printk("alp :  kxtj9_init_input_device ++\n");
	set_bit(EV_ABS, input_dev->evbit);
	set_bit(EV_SYN, input_dev->evbit);
	set_bit(EV_SW, input_dev->evbit);
	input_set_abs_params(input_dev, ABS_X, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Y, -G_MAX, G_MAX, FUZZ, FLAT);
	input_set_abs_params(input_dev, ABS_Z, -G_MAX, G_MAX, FUZZ, FLAT);

	input_dev->name = "kxtj9_accel";
	input_dev->id.bustype = BUS_I2C;
	input_dev->dev.parent = &tj9->client->dev;
	if(KXTJ9_DEBUG_MESSAGE) printk("alp :  kxtj9_init_input_device --\n");
}

static int __devinit kxtj9_setup_input_device(struct kxtj9_data *tj9)
{
	struct input_dev *input_dev;
	int err;
	//printk("Eve_Wen kxtj9_setup_input_device \n");
	input_dev = input_allocate_device();
	if (!input_dev) {
		dev_err(&tj9->client->dev, "input device allocate failed\n");
		return -ENOMEM;
	}

	tj9->input_dev = input_dev;

	input_dev->open = kxtj9_input_open;
	input_dev->close = kxtj9_input_close;
	input_set_drvdata(input_dev, tj9);

	kxtj9_init_input_device(tj9, input_dev);

	err = input_register_device(tj9->input_dev);
	if (err) {
		dev_err(&tj9->client->dev,
			"unable to register input polled device %s: %d\n",
			tj9->input_dev->name, err);
		input_free_device(tj9->input_dev);
		return err;
	}

	return 0;
}

/*
 * When IRQ mode is selected, we need to provide an interface to allow the user
 * to change the output data rate of the part.  For consistency, we are using
 * the set_poll method, which accepts a poll interval in milliseconds, and then
 * calls update_odr() while passing this value as an argument.  In IRQ mode, the
 * data outputs will not be read AT the requested poll interval, rather, the
 * lowest ODR that can support the requested interval.  The client application
 * will be responsible for retrieving data from the input node at the desired
 * interval.
 */

/* Returns currently selected poll interval (in ms) */
static ssize_t kxtj9_get_poll(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int kxtj9_ctrl = 0;
	kxtj9_ctrl = i2c_smbus_read_byte_data(client, DATA_CTRL);
	printk("alp : kxtj9_get_poll (%d) (%d)\n",tj9->last_poll_interval,kxtj9_ctrl);
	return sprintf(buf, "%d , %d\n", tj9->last_poll_interval,kxtj9_ctrl);
}

/* Allow users to select a new poll interval (in ms) */
static ssize_t kxtj9_set_poll(struct device *dev, struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;
	unsigned int interval=0;
        if(canEnableGsensor == false) return -1;
	interval = simple_strtoul(buf, NULL, 10);
	if(KXTJ9_DEBUG_MESSAGE) 
		printk("alp : interval (%u)\n",interval);
	if(KXTJ9_DEBUG_MESSAGE) 
		printk("alp : kxtj9_set_poll buf (%s)\n",buf);
	if(KXTJ9_DEBUG_MESSAGE) 
		printk("alp : last_poll_interval (%u), New_poll(%d)\n",tj9->last_poll_interval,interval);

	// Lock the device to prevent races with open/close (and itself)
	mutex_lock(&input_dev->mutex);
//	disable_irq(tj9->irq);
	 //Set current interval to the greater of the minimum interval or the requested interval
	kxtj9_update_odr(tj9, interval);
	tj9->last_poll_interval = interval;
//	enable_irq(tj9->irq);
	mutex_unlock(&input_dev->mutex);

	return count;
}
#endif
static ssize_t kxtj9_enable_show(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	printk("alp : kxtj9_enable_show (%d)\n",atomic_read(&tj9->enabled));
	return sprintf(buf, "%d\n", atomic_read(&tj9->enabled));
}

static ssize_t kxtj9_enable_store(struct device *dev,
					struct device_attribute *attr,
						const char *buf, size_t count)
{
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int val = simple_strtoul(buf, NULL, 10);
	switch(val){
		case 0:
			kxtj9_disable(tj9);
		break;
		case 1:
			if(canEnableGsensor == true) kxtj9_enable(tj9);
			else return -1;
		break;

		// handled by orientation added by cheng_kao 2012.12.05 ++
		case 2:
			printk("alp : kxtj9_disable_by_orientation !!!\n");
			kxtj9_device_power_off(tj9);
		break;
		case 3:
			printk("alp : kxtj9_enable_by_orientation !!!\n");
			if(canEnableGsensor == true) kxtj9_enable_by_orientation(tj9);
			else return -1;
		break;
		// handled by orientation added by cheng_kao 2012.12.05 --

	}
	return count;
}

static ssize_t get_kxtj9_state(struct device *dev, struct device_attribute *devattr, char *buf)
{	

	struct i2c_client *client = to_i2c_client(dev);
	int kxtj9_wia = 0 , ret=0;
	kxtj9_wia = i2c_smbus_read_byte_data(client, WHO_AM_I);
	printk("get_kxtj9_state wia : %d\n",kxtj9_wia);

	if(kxtj9_wia==9)
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n",ret);

}

static ssize_t get_cal_rawdata(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	int retval = 0;
	unsigned char data[6];
	int rawdata_x=0,rawdata_y=0,rawdata_z=0;
	int reportdata_x=0,reportdata_y=0,reportdata_z=0;
	
	retval = i2c_smbus_write_byte_data(client, CTRL_REG1, PC1_ON);
	if (retval < 0){
		printk("get_cal_rawdata write fail\n");
		return retval;
	}

	data[0] = i2c_smbus_read_byte_data(client, 0x06);
	data[1] = i2c_smbus_read_byte_data(client, 0x07);
	data[2] = i2c_smbus_read_byte_data(client, 0x08);
	data[3] = i2c_smbus_read_byte_data(client, 0x09);
	data[4] = i2c_smbus_read_byte_data(client, 0x0A);
	data[5] = i2c_smbus_read_byte_data(client, 0x0B);
	printk("get_kxtj9_state rawdata : (%d), (%d), (%d), (%d), (%d), (%d)\n",data[0], data[1], data[2],data[3],data[4],data[5]);
	rawdata_x = ( (data[0]>>4) | (data[1]<<4) );
	rawdata_y = ( (data[2]>>4) | (data[3]<<4) );
	rawdata_z = ( (data[4]>>4) | (data[5]<<4) );

	if(rawdata_x>2047)
		rawdata_x = rawdata_x-4096;
	if(rawdata_y>2047)
		rawdata_y = rawdata_y-4096;
	if(rawdata_z>2047)
		rawdata_z = rawdata_z-4096;

	printk("offset data : (%d), (%d), (%d)\n",tj9->accel_cal_offset[0], tj9->accel_cal_offset[1], tj9->accel_cal_offset[2]);
	printk("sensitivity data : (%d), (%d), (%d)\n",tj9->accel_cal_sensitivity[0], tj9->accel_cal_sensitivity[1], tj9->accel_cal_sensitivity[2]);
	printk("rawdata data : (%d), (%d), (%d)\n",rawdata_x, rawdata_y, rawdata_z);
	reportdata_x = 1024*(rawdata_x - tj9->accel_cal_offset[0])/tj9->accel_cal_sensitivity[0];
	reportdata_y = 1024*(rawdata_y - tj9->accel_cal_offset[1])/tj9->accel_cal_sensitivity[1];
	reportdata_z = 1024*(rawdata_z - tj9->accel_cal_offset[2])/tj9->accel_cal_sensitivity[2];
	printk("calibration data : (%d), (%d), (%d)\n",reportdata_x, reportdata_y, reportdata_z);

	return sprintf(buf, "%d %d %d\n",rawdata_x, rawdata_y, rawdata_z);
}

static ssize_t get_rawdata(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	struct i2c_client *client = to_i2c_client(dev);
	int retval = 0;
	unsigned char data[6];
	int test = 11;
	test = i2c_smbus_read_byte_data(client, 0x18);
	//printk("Eve_Wen before write PC1_on SATUS_REG = %d \n", test);
	retval = i2c_smbus_write_byte_data(client, CTRL_REG1, PC1_ON);
	if (retval < 0){
		printk("get_rawdata write fail\n");
		return retval;
	}
	test = i2c_smbus_read_byte_data(client, 0x18);
	//printk("Eve_Wen after PC1_on SATUS_REG = %d \n", test);

	data[0] = i2c_smbus_read_byte_data(client, 0x06);
	data[1] = i2c_smbus_read_byte_data(client, 0x07);
	data[2] = i2c_smbus_read_byte_data(client, 0x08);
	data[3] = i2c_smbus_read_byte_data(client, 0x09);
	data[4] = i2c_smbus_read_byte_data(client, 0x0A);
	data[5] = i2c_smbus_read_byte_data(client, 0x0B);
	printk("get_rawdata rawdata : (%d), (%d), (%d), (%d), (%d), (%d)\n",data[0], data[1], data[2],data[3],data[4],data[5]);
//	sprintf(buf, "%x %x %x %x %x %x\n",data[0], data[1], data[2], data[3], data[4], data[5]);
	//while(1)
//	{
//		mdelay(20);
		test = i2c_smbus_read_byte_data(client, 0x18);
		//printk("Eve_Wen after read   SATUS_REG = %d \n", test);
	//test=i2c_smbus_read_byte_data(client, INT_REL);
          //      printk("Eve_Wen after clear   SATUS_REG = %d \n", test);
printk("==================================================\n");
		//	}
		return sprintf(buf, "%x %x %x %x %x %x\n",data[0], data[1], data[2], data[3], data[4], data[5]);

}

static ssize_t reset_kxtj9_calibration(struct device *dev, struct device_attribute *devattr, char *buf)
{	
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	char tmp_data[128]={0};
	int retval = 0,index=0,iplane=0,ilen=0, icount=0;
	bool bNegative=false;
//	read file data++
	struct file *fp=NULL;
	mm_segment_t old_fs;
//	read file data--

//	disable_irq(tj9->irq);
	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp=filp_open(GSENSOR_CALIBRATION_FILE_PATH,O_RDWR|O_CREAT,S_IRWXU|S_IRWXG|S_IRWXO);
	if(IS_ERR(fp)){
		printk(KERN_INFO "filp_open fail\n");
		retval = -1;
		return retval;
	}
	ilen = fp->f_op->read(fp,tmp_data,128,&fp->f_pos);
	printk(KERN_INFO "KXTJ9_CONFIG_FILE %d\n",ilen);
	printk(KERN_INFO "%s \n",tmp_data);
	set_fs(old_fs);
	filp_close(fp,NULL);

	if(ilen>0){
//	for transfor form file to int ++
		do{
			if(tmp_data[icount]==38){	//	38="&"
				iplane = (tmp_data[icount-2]-48)-1;	// 1: start from 0, 2:start from 1 ; 0:start from -1 -> error!!!
				index =  tmp_data[icount-1]-48;
				if(KXTJ9_DEBUG_MESSAGE) printk(KERN_INFO "iplane(%d), index(%d) \n",iplane ,index );
				if(tmp_data[icount+1]==45){	//	45="-"
					bNegative = true;
					icount++;
				}
				switch(index){
					case 3:
						tj9->accel_cal_data[iplane]= ( (tmp_data[icount+1]-48)*100+(tmp_data[icount+2]-48)*10+(tmp_data[icount+3]-48) );
					break;

					case 4:
						tj9->accel_cal_data[iplane]= ( (tmp_data[icount+1]-48)*1000+(tmp_data[icount+2]-48)*100+(tmp_data[icount+3]-48)*10+(tmp_data[icount+4]-48) );
					break;

					default:
						printk(KERN_INFO "error index %d \n",index );
					break;
				}
				if(bNegative) tj9->accel_cal_data[iplane] = tj9->accel_cal_data[iplane]*(-1);
			}
			bNegative = false;
			icount++;
		}while(icount<ilen);
//	for transfor form file to int --
		if(KXTJ9_CALIBRATED_MESSAGE) {
			for(iplane=0;iplane<6;iplane++){
				printk(KERN_INFO "iplane[%d]  %d \n",iplane ,tj9->accel_cal_data[iplane] );
			}
		}
		retval=1;
	}
	//for X axis
	tj9->accel_cal_sensitivity[0] = (tj9->accel_cal_data[2]-tj9->accel_cal_data[3])/2;
	tj9->accel_cal_offset[0] = tj9->accel_cal_data[3]+tj9->accel_cal_sensitivity[0];
	if(tj9->accel_cal_sensitivity[0]==0)
		tj9->accel_cal_sensitivity[0]=1024;
	//for Y axis
	tj9->accel_cal_sensitivity[1] = (tj9->accel_cal_data[4]-tj9->accel_cal_data[5])/2;
	tj9->accel_cal_offset[1] = tj9->accel_cal_data[5]+tj9->accel_cal_sensitivity[1];
	if(tj9->accel_cal_sensitivity[1]==0)
		tj9->accel_cal_sensitivity[1]=1024;
	//for Z axis
	tj9->accel_cal_sensitivity[2] = (tj9->accel_cal_data[0]-tj9->accel_cal_data[1])/2;
	tj9->accel_cal_offset[2] = tj9->accel_cal_data[1]+tj9->accel_cal_sensitivity[2];
	if(tj9->accel_cal_sensitivity[2]==0)
		tj9->accel_cal_sensitivity[2]=1024;


	if(KXTJ9_CALIBRATED_MESSAGE) {
		printk("accel_cal_sensitivity : (%d), (%d), (%d)\n",tj9->accel_cal_sensitivity[0],tj9->accel_cal_sensitivity[1],tj9->accel_cal_sensitivity[2] );
		printk("accel_cal_offset : (%d), (%d), (%d)\n",tj9->accel_cal_offset[0],tj9->accel_cal_offset[1],tj9->accel_cal_offset[2] );
	}
//	enable_irq(tj9->irq);
	return retval;
}
#if 0
static DEVICE_ATTR(delay, 0660, kxtj9_get_poll, kxtj9_set_poll);
#endif
static DEVICE_ATTR(enable, 0660,kxtj9_enable_show,kxtj9_enable_store);
static DEVICE_ATTR(rawdata, S_IRUGO, get_rawdata, NULL);
static DEVICE_ATTR(state, S_IRUGO, get_kxtj9_state, NULL);
static DEVICE_ATTR(cal_rawdata, S_IRUGO, get_cal_rawdata, NULL);
static DEVICE_ATTR(calibration, S_IRUGO, reset_kxtj9_calibration, NULL);
static struct attribute *kxtj9_attributes[] = {
#if 0
	&dev_attr_delay.attr,
#endif
	&dev_attr_enable.attr,
	&dev_attr_rawdata.attr,
	&dev_attr_state.attr,
	&dev_attr_cal_rawdata.attr,
	&dev_attr_calibration.attr,
	NULL
};

static struct attribute_group kxtj9_attribute_group = {
	.attrs = kxtj9_attributes
};

#if 0
#ifdef CONFIG_INPUT_KXTJ9_POLLED_MODE
static void kxtj9_poll(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;
	unsigned int poll_interval = dev->poll_interval;
	//printk("Eve_Wen kxtj9_poll 1 \n");
	kxtj9_report_acceleration_data(tj9);
	//printk("Eve_Wen kxtj9_poll 2 \n");
	if (poll_interval != tj9->last_poll_interval) {
		kxtj9_update_odr(tj9, poll_interval);
		tj9->last_poll_interval = poll_interval;
	}
//	if(KXTJ9_DEBUG_MESSAGE) 
		printk("alp : kxtj9_poll poll_interval(%d)\n",tj9->last_poll_interval);
}

static void kxtj9_polled_input_open(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;

	kxtj9_enable(tj9);
}

static void kxtj9_polled_input_close(struct input_polled_dev *dev)
{
	struct kxtj9_data *tj9 = dev->private;

	kxtj9_disable(tj9);
}

static int __devinit kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
	int err;
	struct input_polled_dev *poll_dev;
	poll_dev = input_allocate_polled_device();

	if (!poll_dev) {
		dev_err(&tj9->client->dev,
			"Failed to allocate polled device\n");
		return -ENOMEM;
	}

	tj9->poll_dev = poll_dev;
	tj9->input_dev = poll_dev->input;

	poll_dev->private = tj9;
	poll_dev->poll = kxtj9_poll;
	poll_dev->open = kxtj9_polled_input_open;
	poll_dev->close = kxtj9_polled_input_close;

	kxtj9_init_input_device(tj9, poll_dev->input);

	err = input_register_polled_device(poll_dev);
	if (err) {
		dev_err(&tj9->client->dev,
			"Unable to register polled device, err=%d\n", err);
		input_free_polled_device(poll_dev);
		return err;
	}

	return 0;
}

static void __devexit kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
	input_unregister_polled_device(tj9->poll_dev);
	input_free_polled_device(tj9->poll_dev);
}

#else

static inline int kxtj9_setup_polled_device(struct kxtj9_data *tj9)
{
	return -ENOSYS;
}

static inline void kxtj9_teardown_polled_device(struct kxtj9_data *tj9)
{
}

#endif
#endif


/*
static struct file_operations kxtj9_fops = {
	.owner			= 	THIS_MODULE,
//	.poll 			= 	kxtj9_poll,
//	.read 			= 	kxtj9_read,
	.unlocked_ioctl	=	kxtj9_ioctl,
//	.compat_ioctl		=	kxtj9_ioctl,
	.open			=	kxtj9_open,
	.release			=	kxtj9_release,
};
*/

static struct miscdevice kxtj9_device = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "kxtj9_dev",
//	.fops = &kxtj9_fops,
};

#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
static void kxtj9_early_suspend(struct early_suspend *h)
{
	struct kxtj9_data *tj9 = container_of(h,struct kxtj9_data,kxtj9_early_suspendresume);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
	disable_irq(tj9->irq);
	kxtj9_disable(tj9);
	canEnableGsensor = false;
	mutex_unlock(&input_dev->mutex);
	printk("alp : kxtj9_early_suspend irq(%d)\n",tj9->irq);
}

static void kxtj9_late_resume(struct early_suspend *h)
{
	struct kxtj9_data *tj9 = container_of(h,struct kxtj9_data,kxtj9_early_suspendresume);
	struct input_dev *input_dev = tj9->input_dev;
	mutex_lock(&input_dev->mutex);
	kxtj9_enable(tj9);
	enable_irq(tj9->irq);
	canEnableGsensor = true;
	mutex_unlock(&input_dev->mutex);
	printk("alp : kxtj9_late_resume irq(%d)\n",tj9->irq);
}
#endif
#else
void kxtj9_suspend(void)
{
	if(g_kxtj9_data!=NULL){
	printk("alp : kxtj9_suspend +++\n");
	mutex_lock(&g_kxtj9_data->mutex);
	kxtj9_disable(g_kxtj9_data);
	canEnableGsensor = false;
	mutex_unlock(&g_kxtj9_data->mutex);
	printk("alp : kxtj9_suspend ---\n");
	}
}

void kxtj9_resume(void)
{
	if(g_kxtj9_data!=NULL){
	printk("alp : kxtj9_resume +++\n");
	mutex_lock(&g_kxtj9_data->mutex);
	kxtj9_enable(g_kxtj9_data);
	canEnableGsensor = true;
	mutex_unlock(&g_kxtj9_data->mutex);
	printk("alp : kxtj9_resume ---\n");
	}
}
EXPORT_SYMBOL(kxtj9_suspend);
EXPORT_SYMBOL(kxtj9_resume);
#endif

static int __devinit kxtj9_probe(struct i2c_client *client,
				 const struct i2c_device_id *id)
{
	struct kxtj9_platform_data *pdata = client->dev.platform_data;
	struct kxtj9_data *tj9;
	int gpio=0, iloop=0;
	int err;
	ex_client= client;

//	g_ilocation = Read_PROJ_ID();
//	printk("GSENSOR KXTJ9 read PROJ ID = %d\n", g_ilocation);

	//g_ilocation = KXTJ9_CHIP_LOCATION_SR;

	if(KXTJ9_DEBUG_MESSAGE) printk("alp : kxtj9_probe ++\n");
	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_BYTE_DATA)) {
		dev_err(&client->dev, "client is not i2c capable\n");
		return -ENXIO;
	}

	if (!pdata) {
		dev_err(&client->dev, "platform data is NULL; exiting\n");
		return -EINVAL;
	}

	tj9 = kzalloc(sizeof(*tj9), GFP_KERNEL);
	if (!tj9) {
		dev_err(&client->dev,
			"failed to allocate memory for module data\n");
		return -ENOMEM;
	}

	tj9->client = client;
	tj9->pdata = *pdata;

	//init calibration data
	for(iloop=0;iloop<6;iloop++){
		tj9->accel_cal_data[iloop]=0;
	}
	for(iloop=0;iloop<3;iloop++){
		tj9->accel_cal_offset[iloop]=0;
		tj9->accel_cal_sensitivity[iloop]=1024;
	}

#if 0
//	if(KXTJ9_DEBUG_MESSAGE) 
		printk("alp : irq=%d\n",tj9->irq);
	gpio = 76;//pdata->gpio;
	err = gpio_request(gpio,"accel_kxtj9");
	err = gpio_direction_input(gpio);
	tj9->irq = gpio_to_irq(gpio);
	printk("alp : final irq=%d\n",tj9->irq);

	err = kxtj9_verify(tj9);
	if (err < 0) {
		dev_err(&client->dev, "device not recognized\n");
		goto err_pdata_exit;
	}
#endif
	i2c_set_clientdata(client, tj9);
	tj9->ctrl_reg1 = tj9->pdata.res_12bit | tj9->pdata.g_range;
	tj9->last_poll_interval = tj9->pdata.init_interval;

#if 0
	if (tj9->irq) {
		/* If in irq mode, populate INT_CTRL_REG1 and enable DRDY. */
		tj9->int_ctrl |= KXTJ9_IEN | KXTJ9_IEA | KXTJ9_IEL;
		tj9->ctrl_reg1 |= DRDYE;
		tj9->ctrl_reg1 &= 0xe7;
		err = kxtj9_setup_input_device(tj9);
		if (err)
			goto err_pdata_exit;

		err = request_threaded_irq(tj9->irq, NULL, kxtj9_isr,
					   IRQF_TRIGGER_RISING |
					   IRQF_ONESHOT, 
					   "kxtj9-irq", tj9);
		//err = request_threaded_irq(tj9->irq, NULL, kxtj9_isr,IRQF_TRIGGER_RISING , "kxtj9-irq", tj9);
		if (err) {
			dev_err(&client->dev, "request irq failed: %d\n", err);
			goto err_destroy_input;
		}

		err = sysfs_create_group(&client->dev.kobj,
				&kxtj9_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_free_irq;
		}
		err = misc_register(&kxtj9_device);
		if (err) {
//			if(KXTJ9_DEBUG_MESSAGE) 
				printk("alp :  kxtj9_misc_register failed\n");
		}
	} else {
		err = kxtj9_setup_polled_device(tj9);
		if (err)
			goto err_pdata_exit;
	}
#else
		err = sysfs_create_group(&client->dev.kobj,
				&kxtj9_attribute_group);
		if (err) {
			dev_err(&client->dev, "sysfs create failed: %d\n", err);
			goto err_free_mem;
		}
#endif
	atomic_set(&tj9->enabled, 0);

#if 0
#ifdef CONFIG_HAS_EARLYSUSPEND
	tj9->kxtj9_early_suspendresume.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 1;
	tj9->kxtj9_early_suspendresume.suspend = kxtj9_early_suspend;
	tj9->kxtj9_early_suspendresume.resume = kxtj9_late_resume;
	register_early_suspend(&tj9->kxtj9_early_suspendresume);
#endif
#endif
	if(create_asusproc_gsensor_status_entry( ))
		printk("[%s] : ERROR to create g_sensor proc entry\n",__func__);
//	if(KXTJ9_DEBUG_MESSAGE) 
	g_kxtj9_data = tj9;
	mutex_init(&g_kxtj9_data->mutex);
		printk("alp : kxtj9_probe --\n");


	return 0;

//err_free_irq:
//	free_irq(tj9->irq, tj9);
//err_destroy_input:
//	input_unregister_device(tj9->input_dev);
//err_pdata_exit:
//	if (tj9->pdata.exit)
//		tj9->pdata.exit();
err_free_mem:
	kfree(tj9);
	return err;
}
int enAccelConfig_func(int raw_result,int Max,int Min)
{
	 s64 sensitivity;
	 int zero_offset,After_calib;
	 sensitivity  =  (Max - Min) /2 ;
	 zero_offset =  Min + sensitivity;

	 After_calib = raw_result -zero_offset;
   	 return After_calib;
}

int gsensor_register(void)
{
	int err=-1;
	err = misc_register(&asus_gsensor_device);
	if (err)
		printk(KERN_ERR "%s: asus_gsensor misc register failed\n", __func__);
	return err;
}

static int __devexit_p(kxtj9_remove)(struct i2c_client *client)
{
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
#if 0
	if (tj9->irq) {
		sysfs_remove_group(&client->dev.kobj, &kxtj9_attribute_group);
		free_irq(tj9->irq, tj9);
		input_unregister_device(tj9->input_dev);
	} else {
		kxtj9_teardown_polled_device(tj9);
	}

	if (tj9->pdata.exit)
		tj9->pdata.exit();
#else
	sysfs_remove_group(&client->dev.kobj, &kxtj9_attribute_group);
#endif
	kfree(tj9);

	return 0;
}

#if 0
#ifdef CONFIG_PM_SLEEP
static int kxtj9_suspend(struct device *dev)
{
/*
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
//	if (input_dev->users){
//		disable_irq(tj9->irq);
		kxtj9_disable(tj9);
//	}
	mutex_unlock(&input_dev->mutex);
*/
	printk("alp : kxtj9_suspend\n");
	return 0;
}

static int kxtj9_resume(struct device *dev)
{
/*
	struct i2c_client *client = to_i2c_client(dev);
	struct kxtj9_data *tj9 = i2c_get_clientdata(client);
	struct input_dev *input_dev = tj9->input_dev;

	mutex_lock(&input_dev->mutex);
//	if (input_dev->users){
//		enable_irq(tj9->irq);
		kxtj9_enable(tj9);
//	}
	mutex_unlock(&input_dev->mutex);
*/
	printk("alp : kxtj9_resume\n");
	return 0;
}
#endif

#endif
//static SIMPLE_DEV_PM_OPS(kxtj9_pm_ops, kxtj9_suspend, kxtj9_resume);

static const struct i2c_device_id kxtj9_id[] = {
	{ NAME, 0 },
	{ },
};

MODULE_DEVICE_TABLE(i2c, kxtj9_id);

static struct i2c_driver kxtj9_driver = {
	.driver = {
		.name	= NAME,
		.owner	= THIS_MODULE,
//		.pm	= &kxtj9_pm_ops,
	},
	.probe		= kxtj9_probe,
	.remove		= __devexit_p(kxtj9_remove),
	.id_table	= kxtj9_id,
};

static int __init kxtj9_init(void)
{

    int res;

    switch (Read_PROJ_ID()) {

	case PROJ_ID_ZR550ML:
		pr_info("Project ID is ZR550ML, kxtj9 init...\n");
		res = i2c_add_driver(&kxtj9_driver);
		return res;
	break;

	case PROJ_ID_ZE500ML:
	case PROJ_ID_ZE550ML:
	case PROJ_ID_ZE551ML:
	case PROJ_ID_ZX550ML:
		pr_info("Project ID is NOT ZR550ML, kxtj9 exit...\n");
		return -1;
	break;
	default:
		pr_info("Project ID is NOT ZR550ML, kxtj9 exit...\n");
		return -1;
	break;
    }//end switch

}
module_init(kxtj9_init);

static void __exit kxtj9_exit(void)
{
//	gpio_free(46);
	misc_deregister(&asus_gsensor_device);
	if (gsensor_entry)
		remove_proc_entry("asus_gsensor_a500_status", NULL);
	i2c_del_driver(&kxtj9_driver);
}
module_exit(kxtj9_exit);

MODULE_DESCRIPTION("KXTJ9 accelerometer driver");
MODULE_AUTHOR("Chris Hudson <chudson@kionix.com>");
MODULE_LICENSE("GPL");
