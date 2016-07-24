/*
 * Copyright (C) 2015 ASUSTeK Computer Inc.
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

#include <linux/delay.h>
//#include <linux/earlysuspend.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/sensors.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/miscdevice.h>
#include <linux/lightsensor.h>
#include <linux/slab.h>
#include <asm/uaccess.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/file.h>
#include <linux/cm36686.h>
#include <asm/setup.h>
#include <linux/wakelock.h>
#include <linux/jiffies.h>
#include <linux/types.h>
#include <linux/proc_fs.h>
#include <linux/of_gpio.h>
#include <linux/HWVersion.h>
#include <linux/seq_file.h>
#include <linux/regulator/consumer.h>

#include <linux/intel_mid_pm.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel-mid.h>

extern int Read_PROJ_ID(void);
extern int Read_HW_ID(void);

#define DEBUG_FUN        0
#define DEBUG_VALUE      0
#define SENSITIVITY_8    8  //80ms 65536/7864
#define SENSITIVITY_16  16  //160ms 65536/3277
#define SENSITIVITY_33  33  //320ms 65536/1638
#define SENSITIVITY_66  66  //640ms 65536/983
#define SENSITIVITY_20  20
#define SENSITIVITY_25  25
#define Driverversion  "1.0.0"
#define VENDOR       "CM32683"

#ifdef CONFIG_ASUS_SENSOR_ENG_CMD
#define PS_CAL_PATH  "/factory/PSensor_Calibration.ini"
#define CONFIG_ASUS_FACTORY_SENSOR_MODE
#define ATTRIBUTES_PERMISSION 0666
#else
#define PS_CAL_PATH  "/data/PSensor_Calibration.ini"
#define ATTRIBUTES_PERMISSION  0660
#endif

#define I2C_RETRY_COUNT 10
#define NEAR_DELAY_TIME ((100 * HZ) / 1000)
#define CONTROL_INT_ISR_REPORT        0x00
#define CONTROL_ALS                   0x01
#define CONTROL_PS                    0x02
#define LS_RAWDATA_WINDOW             (10)  // 0.05 step/lux

#define PS_CLOSE 1
#define PS_AWAY  (1<<1)
#define PS_CLOSE_AND_AWAY PS_CLOSE+PS_AWAY

#define PS_AWAY_THD  0x80
#define PS_CLOSE_THD 0xE0

static int Sensitivity_X = 5;
static bool IC_Ver = 0 ; //  default = 0 means cm3602 ,1 means cm36686
static int Proximity_State = 1;
static int Proximity_Int_Away = 0;
static int Platform_Data_Probe_Fail = 0;
static int Als_It = 0;

bool En_LSensor_Cal = 0;
bool En_PSensor_Cal = 0;
uint16_t default_levels[10] = { 0x0A, 0xA0, 0xE1, 0x140, 0x280,0x500, 0xA28, 0x16A8, 0x1F40, 0x2800};

int LSensor_CALIDATA[2] = {0}; //input calibration data . Format : "200 lux -->lux value ; 1000 lux -->lux value"
int PSensor_CALIDATA[2] = {0}; //input calibration data . Format : "near 3cm :--> value ; far  5cm :--> value"
struct proc_dir_entry *Lpsensor_Entry = NULL;
struct proc_dir_entry *Lightsensor_Entry = NULL;
struct proc_dir_entry *Proximitysensor_Entry = NULL;
struct proc_dir_entry* proc_psensor_auto_calibration_entry  = NULL;
struct proc_dir_entry* proc_psensor_calibration_entry  = NULL;
static int lpsensor_proc_show(struct seq_file *m, void *v)
{
	if(!Lpsensor_Entry)
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

//=========================================
//     Calibration Formula:
//     y = f(x)
//  -> ax - by = constant_k
//     a is f(x2) - f(x1) , b is x2 - x1
////=========================================

 static int calibration_light(int cal_fac_big, int cal_fac_small, int report_lux)
 {
	//int cal_spec_big = 1000;//asus calibration 1000lux
	int cal_spec_small = 200;//asus calibration 200lux
	int cal_spec_diff =800;
	int cal_fac_diff = cal_fac_big-cal_fac_small;
	int constant_k;

	constant_k = (cal_spec_diff*cal_fac_small) - (cal_fac_diff*cal_spec_small);
	if ( (report_lux*cal_spec_diff) < constant_k){
		return 0;
		} else {
		return (((report_lux*cal_spec_diff) - constant_k) / cal_fac_diff);
	}
}

static int record_init_fail = 0;
static void sensor_irq_do_work(struct work_struct *work);
static void light_sensor_initial_value_work_routine(struct work_struct *work);
static void proximity_initial_value_work_routine(struct work_struct *work);
static DECLARE_WORK(sensor_irq_work, sensor_irq_do_work);
static DECLARE_DELAYED_WORK(light_sensor_initial_value_work, light_sensor_initial_value_work_routine);
static DECLARE_DELAYED_WORK(proximity_initial_value_work, proximity_initial_value_work_routine);

struct driver_info {
	struct class *driver_class;
	struct device *ls_dev;
	struct device *ps_dev;
	struct input_dev *ls_input_dev;
	struct input_dev *ps_input_dev;
	//struct early_suspend early_suspend;
	struct i2c_client *i2c_client;
	struct workqueue_struct *lp_wq;
	int als_enable;
	int ps_enable;
	int ps_irq_flag;
	uint16_t *adc_table;
	uint16_t cali_table[10];
	int irq;
	int ls_calibrate;
	int (*power)(int, uint8_t); /* power to the chip */
	uint32_t Als_Cal_Adc;
	uint32_t als_gadc;
	struct wake_lock ps_wake_lock;
	int psensor_opened;
	int lightsensor_opened;
	int current_level;
	uint16_t current_adc;
	uint8_t record_clear_int_fail;
	uint8_t psensor_sleep_becuz_suspend;
	uint8_t lsensor_sleep_becuz_early_suspend;
	uint8_t status_calling;
	int last_initial_report_lux;
	struct regulator *vdd;
	struct regulator *vio;

	int intr_pin;
	uint16_t levels[10];
	uint16_t golden_adc;
	uint8_t slave_addr;
	uint16_t ps_close_thd_set;
	uint16_t ps_away_thd_set;
	uint16_t ls_cmd;
	uint16_t ps_conf1_val;
	uint16_t ps_conf3_val;
	bool polling;
};

static struct driver_info *PL_Driver_Data = NULL;
//#define DEBUG_VEMMC2
#ifdef DEBUG_VEMMC2
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_pmic.h>
#include <linux/mfd/intel_msic.h>

static struct device* vemmc2_userCtrl_dev;
static struct class* vemmc2_userCtrl_class;
//static struct class* gpio_userCtrl_class;
//static struct device* gpio_userCtrl_dev;

static struct regulator *vemmc2_reg;
static ssize_t vemmc2_reg_show(struct device *dev,struct device_attribute *attr, char *buf){
return 0;
}

static ssize_t vemmc2_reg_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count){
	struct driver_info *lpi = PL_Driver_Data;
	int vemmc2_en, reg_err;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	vemmc2_en = -1;
	sscanf(buf, "%d", &vemmc2_en);

	if (vemmc2_en != 0 && vemmc2_en != 1
		&& vemmc2_en != 2 && vemmc2_en != 3 && vemmc2_en != 4)
		return -EINVAL;

	if (vemmc2_en==0) {
		printk("[CM36686][proximity-Touch test]: report =%d\n", vemmc2_en);
		input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 0);
		input_sync(lpi->ps_input_dev);
	}else if(vemmc2_en==1){
		printk("[CM36686][proximity-Touch test] : report =%d\n", vemmc2_en);
		input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 1);
		input_sync(lpi->ps_input_dev);
	}else if(vemmc2_en==2){
 		printk("[CM36686][proximity-Touch test]: report =%d\n", vemmc2_en);
		input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 2);
		input_sync(lpi->ps_input_dev);
	}else if(vemmc2_en==3){
 		printk("[CM36686][proximity-Touch test]: report =%d\n", vemmc2_en);
		input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 3);
		input_sync(lpi->ps_input_dev);
	}

return count;
}
DEVICE_ATTR(vemmc2_ctrl, ATTRIBUTES_PERMISSION, vemmc2_reg_show, vemmc2_reg_store);

#endif // DEBUG_VEMMC2


int Lsensor_fLevel=-1;

static struct mutex Als_Enable_Mutex, Als_Disable_Mutex, Als_Get_Adc_Mutex;
static struct mutex Ps_Enable_Mutex, Ps_Disable_Mutex, Ps_Get_Adc_Mutex;
static struct mutex Driver_Control_Mutex;
static int lightsensor_enable(struct driver_info *lpi);
static int lightsensor_disable(struct driver_info *lpi);
static int initial_driver(struct driver_info *lpi);
static void psensor_initial_cmd(struct driver_info *lpi);


int32_t Als_Cal_Adc;
static int control_and_report(struct driver_info *lpi, uint8_t mode, uint16_t param);

static int I2C_RxData(uint16_t slaveAddr, uint8_t cmd, uint8_t *rxData, int length)
{
	uint8_t loop_i;
	uint8_t subaddr[1];
	struct i2c_msg msgs[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = 1,
		 .buf = subaddr,
		 },

		{
		 .addr = slaveAddr,
		 .flags = I2C_M_RD,
		 .len = length,
		 .buf = rxData,
		 },
	};
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	subaddr[0] = cmd;
	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(PL_Driver_Data->i2c_client->adapter, msgs, 2) > 0)
			break;

		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			printk("[CM36686][PS] i2c err, slaveAddr 0x%x , record_init_fail %d \n",
				 slaveAddr,  record_init_fail);
		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk("[CM36686][PS] retry over %d\n", I2C_RETRY_COUNT);
		return -EIO;
	}
	return 0;
}

static int I2C_TxData(uint16_t slaveAddr, uint8_t *txData, int length)
{
	uint8_t loop_i;
	struct i2c_msg msg[] = {
		{
		 .addr = slaveAddr,
		 .flags = 0,
		 .len = length,
		 .buf = txData,
		 },
	};
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	for (loop_i = 0; loop_i < I2C_RETRY_COUNT; loop_i++) {
		if (i2c_transfer(PL_Driver_Data->i2c_client->adapter, msg, 1) > 0)
			break;

		/*check intr GPIO when i2c error*/
		if (loop_i == 0 || loop_i == I2C_RETRY_COUNT -1)
			printk("[CM36686][PS] i2c err, slaveAddr 0x%x, value 0x%x, record_init_fail %d\n",
				 slaveAddr, txData[0], record_init_fail);

		msleep(10);
	}

	if (loop_i >= I2C_RETRY_COUNT) {
		printk("[CM36686][PS] retry over %d\n", I2C_RETRY_COUNT);
		return -EIO;
	}

	return 0;
}

static int _CM36XXX_I2C_Read_Word(uint16_t slaveAddr, uint8_t cmd, uint16_t *pdata)
{
	uint8_t buffer[2];
	int ret = 0;
	if (pdata == NULL)
		return -EFAULT;

	ret = I2C_RxData(slaveAddr, cmd, buffer, 2);
	if (ret < 0) {
		printk("[CM36686][PS] :I2C_RxData can't read [0x%x, 0x%x]\n", slaveAddr, cmd);
		return ret;
	}

	*pdata = (buffer[1]<<8)|buffer[0];
	return ret;
}


static int _CM36XXX_I2C_Write_Word(uint16_t SlaveAddress, uint8_t cmd, uint16_t data)
{
	char buffer[3];
	int ret = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	buffer[0] = cmd;
	buffer[1] = (uint8_t)(data&0xff);
	buffer[2] = (uint8_t)((data&0xff00)>>8);

	ret = I2C_TxData(SlaveAddress, buffer, 3);
	if (ret < 0) {
		printk("[CM36686][PS] can't write I2C_TxData  \n" );
		return -EIO;
	}
	return ret;
}

static int get_ls_adc_value(uint16_t *als_step, bool resume)
{
	int ret = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (als_step == NULL)
		return -EFAULT;

	/* Read ALS data: */
	ret = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, ALS_DATA, als_step);
	if (ret < 0) {
		printk("[CM36686][LS]_CM36686_I2C can't Read_Word \n");
		return -EIO;
	}
	if(DEBUG_VALUE) printk("[CM36686][LS] als_step:%d \n",*als_step);

	if(*als_step < LS_RAWDATA_WINDOW*Sensitivity_X){
		*als_step=0x00;
	}

	return ret;
}

static int set_lsensor_range(uint16_t adc_value)
{
	int ret = 0;
	uint16_t ls_low_thd;
	uint16_t ls_high_thd;
	int ls_thd=LS_RAWDATA_WINDOW*Sensitivity_X;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	if (adc_value < ls_thd) {
		ls_low_thd = 0;
		ls_high_thd = ls_thd;
	}else if (adc_value > 0xFFFF - ls_thd) {
		ls_low_thd = 0xFFFF - ls_thd;
		ls_high_thd = 0xFFFF;
	}else{
		ls_low_thd = adc_value - ls_thd;
		ls_high_thd = adc_value + ls_thd;
	}
	if(DEBUG_VALUE) printk("[CM36686] ls_low_thd:%d ,ls_high_thd:%d\n",ls_low_thd,ls_high_thd);
	_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, ALS_THDH, ls_high_thd);
	_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, ALS_THDL, ls_low_thd);
	return ret;
}

static int get_ps_adc_value(uint16_t *data)
{
	int ret = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (data == NULL)
		return -EFAULT;
	ret = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, PS_DATA, data);
	if(IC_Ver == 0) {
		(*data) &= 0xFF;

	}
	if (ret < 0) {
		printk("[CM36686][PS]: _CM36686_I2C can't Read_Word  \n");
		return -EIO;
	} else {
		if(DEBUG_VALUE) printk("[CM36686][PS]: _CM36686_I2C Read_Word OK 0x%x\n", *data);
	}
	printk("[CM36686][PS]: ps_adc:%d\n",*data);
	return ret;
}

static uint16_t mid_value(uint16_t value[], uint8_t size)
{
	int i = 0, j = 0;
	uint16_t temp = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (size < 3)
		return 0;

	for (i = 0; i < (size - 1); i++) {
		for (j = (i + 1); j < size; j++) {
			if (value[i] > value[j]) {
				temp = value[i];
				value[i] = value[j];
				value[j] = temp;
			}
		}
	}
	return value[((size - 1) / 2)];
}

static int get_stable_ps_adc_value(uint16_t *ps_adc)
{
	uint16_t value[3] = {0, 0, 0}, mid_val = 0;
	int ret = 0;
	int i = 0;
	int wait_count = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	for (i = 0; i < 3; i++) {
		/*wait interrupt GPIO high*/
		while (gpio_get_value(PL_Driver_Data->intr_pin) == 0) {
			msleep(10);
			wait_count++;

			if (wait_count > 12) {
				printk("[CM36686][PS] interrupt GPIO low, get_ps_adc_value\n");
				return -EIO;
			}
		}

		ret = get_ps_adc_value(&value[i]);
		if (ret < 0) {
			printk("[CM36686][PS] get_ps_adc_value\n");
			return -EIO;
		}

		if (wait_count < 60/10) {/*wait gpio less than 60ms*/
			msleep(60 - (10*wait_count));
		}
		wait_count = 0;
	}

	mid_val = mid_value(value, 3);
	printk("[CM36686][PS]: After sort, value[0, 1, 2] = [0x%x, 0x%x, 0x%x]",value[0], value[1], value[2]);
	*ps_adc = (mid_val & 0xFF);

	return 0;
}

static void proximity_initial_value_work_routine(struct work_struct *work)
{
	uint16_t ps_adc_value_init;
	int ret;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret = get_ps_adc_value(&ps_adc_value_init);
	if(!ret){
		if(ps_adc_value_init > PL_Driver_Data->ps_close_thd_set){
			input_report_abs(PL_Driver_Data->ps_input_dev, ABS_DISTANCE, 1);
			input_report_abs(PL_Driver_Data->ps_input_dev, ABS_DISTANCE, 0);
			input_sync(PL_Driver_Data->ps_input_dev);
			Proximity_State = 0;
			printk("[CM36686][PS] proximity initial NEAR\n");
		} else {
			input_report_abs(PL_Driver_Data->ps_input_dev, ABS_DISTANCE, 0);
			input_report_abs(PL_Driver_Data->ps_input_dev, ABS_DISTANCE, 1);
			input_sync(PL_Driver_Data->ps_input_dev);
			Proximity_State = 1;
			printk("[CM36686][PS] proximity initial FAR\n");
		}
	}
	enable_irq(PL_Driver_Data->irq);
}

bool proximity_check_status(void){
	struct driver_info *lpi;
	uint16_t ps_adc_value_init = 0 , data =0, data1= 0;
	int ret = 0;
	int p_value;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (Platform_Data_Probe_Fail) {
		printk("proximity_check_status default return FAR\n");
		goto error_return_far;
	}

	mutex_lock(&Ps_Enable_Mutex);

	lpi = PL_Driver_Data;
	ret = _CM36XXX_I2C_Read_Word(lpi->slave_addr, PS_CONF1, &data);
	if (ret < 0) goto error_return_far;

	if ( data & CM36686_PS_SD ) {
		data1 = data & CM36686_PS_SD_MASK; //disable = 0
		ret = _CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_CONF1, data1);
		if (ret < 0) goto error_return_far;
	}

	msleep(50); //need some delay

	ret = get_ps_adc_value(&ps_adc_value_init);
	if(!ret){
		if(ps_adc_value_init > lpi->ps_close_thd_set){
			printk("[CM36686][PS] proximity initial NEAR\n");
			p_value = 1;
		}else{
			printk("[CM36686][PS] proximity initial FAR\n");
			p_value = 0;
		}
	} else {
		goto error_return_far;
	}


	if ( data & CM36686_PS_SD) {
		data1 = data | CM36686_PS_SD; //disable = 1
		ret = _CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_CONF1, data1);
		if (ret < 0) goto error_return_far;
	}

	mutex_unlock(&Ps_Enable_Mutex);
	return p_value;
error_return_far:
	return 0;
}
EXPORT_SYMBOL(proximity_check_status);

static void light_sensor_initial_value_work_routine(struct work_struct *work)
{
	int ret;
	uint16_t adc_value;

	struct driver_info *lpi = PL_Driver_Data;
	int report_lux=0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	get_ls_adc_value(&adc_value, 0);

	ret = set_lsensor_range(adc_value);
	report_lux = adc_value/Sensitivity_X;
	if(En_LSensor_Cal == 1 ){//calibration enable
		report_lux = calibration_light(LSensor_CALIDATA[1], LSensor_CALIDATA[0], report_lux);
		printk("[CM36686][LS] calibration-initial sensor lux is %d\n", report_lux);
	} else {
		printk("[CM36686][LS] no calibration-initial sensor lux is %d\n", report_lux);
	}
	input_report_abs(lpi->ls_input_dev, ABS_MISC,report_lux+1);
	input_report_abs(lpi->ls_input_dev, ABS_MISC,report_lux);
	input_sync(lpi->ls_input_dev);
}

static void sensor_irq_do_work(struct work_struct *work)
{
	struct driver_info *lpi = PL_Driver_Data;
	uint16_t intFlag;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	_CM36XXX_I2C_Read_Word(lpi->slave_addr, INT_FLAG, &intFlag);
 //  printk("[CM36686] sensor_irq_do_work, intFlag = 0x%X\n", intFlag);
	control_and_report(lpi, CONTROL_INT_ISR_REPORT, intFlag);
	enable_irq(lpi->irq);

	if (Proximity_Int_Away == 1 || Proximity_State == 0) {
		wake_unlock(&(lpi->ps_wake_lock));
		Proximity_Int_Away = 0;
	}
}

static int lightsensor_proc_show(struct seq_file *m, void *v)
{
	int ret = 0,ret1=0;
	uint16_t idReg;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret1 = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, ID_REG, &idReg);
	printk("[CM36686] sensor idReg : 0x0c=0x%x \n", idReg);

	if(ret1<0){
		ret =seq_printf(m," ERROR: i2c r/w test fail\n");
	}else{
		ret =seq_printf(m," ACK: i2c r/w test ok\n");
	}
	if(Driverversion != NULL){
		ret =seq_printf(m," Driver version:%s\n",Driverversion);
	} else {
		ret =seq_printf(m," Driver version:NULL\n");
	}

	if(ret1<0){
		ret =seq_printf(m," %s light status:error\n", CM36686_I2C_NAME);
	} else {
		ret =seq_printf(m," %s light status:ok\n", CM36686_I2C_NAME);
	}
   	return ret;
}

static int Proximitysensor_proc_show(struct seq_file *m, void *v)
{
	int ret = 0,ret1=0;
	uint16_t idReg;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret1 = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, ID_REG, &idReg);
	printk("[CM36686] sensor idReg : 0x0c=0x%x \n", idReg);

	if(ret1<0){
			ret =seq_printf(m," ERROR: i2c r/w test fail\n");
	}else{
			ret =seq_printf(m," ACK: i2c r/w test ok\n");
	}
	if(Driverversion != NULL){
		ret =seq_printf(m," Driver version:%s\n",Driverversion);
	}
	else{
		ret =seq_printf(m," Driver version:NULL\n");
	}

	if(ret1<0){
		ret =seq_printf(m," %s proximity status:error\n", CM36686_I2C_NAME);
	}else{
		ret =seq_printf(m," %s proximity status:ok\n", CM36686_I2C_NAME);
	}
	return ret;
}

static int lightsensor_proc_open(struct inode *inode, struct  file *file)
{
  return single_open(file, lightsensor_proc_show, NULL);
}

static const struct file_operations lightsensor_proc_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static int Proximitysensor_proc_open(struct inode *inode, struct  file *file)
{
  return single_open(file, Proximitysensor_proc_show, NULL);
}

static const struct file_operations Proximitysensor_proc_fops = {
	.owner = THIS_MODULE,
	.open = Proximitysensor_proc_open,
	.read = seq_read,
	.llseek = seq_lseek,
	.release = single_release,
};

static irqreturn_t CM36686_irq_handler(int irq, void *data)
{
	struct driver_info *lpi = PL_Driver_Data;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (Proximity_State == 0) wake_lock_timeout(&(lpi->ps_wake_lock), 1 * HZ);
	disable_irq_nosync(lpi->irq);
	queue_work(lpi->lp_wq, &sensor_irq_work);
  //printk("CM36686 --- End of CM36686_irq_handler\n");
	return IRQ_HANDLED;
}

static int als_power(int enable)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (PL_Driver_Data->power)
		PL_Driver_Data->power(LS_PWR_ON, 1);

	return 0;
}

static void ls_initial_cmd(struct driver_info *lpi)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	/*must disable l-sensor interrupt befrore IST create*//*disable ALS func*/
	lpi->ls_cmd &= CM36686_ALS_INT_MASK;
	lpi->ls_cmd |= CM36686_ALS_SD;
	_CM36XXX_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
}

static void psensor_initial_cmd(struct driver_info *lpi)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	/*must disable p-sensor interrupt befrore IST create*/
	lpi->ps_conf1_val |= CM36686_PS_SD;
	lpi->ps_conf1_val &= CM36686_PS_INT_MASK;

	lpi->ps_conf1_val = 0x3d7;
	lpi->ps_conf3_val = 0x210;

	_CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);
	_CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_CONF3, lpi->ps_conf3_val);

	_CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set);
	_CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set);
}

static int psensor_enable(struct driver_info *lpi)
{
	int ret = -EIO;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	mutex_lock(&Ps_Enable_Mutex);


	if ( lpi->ps_enable ) {
		printk("[CM36686][PS] already enabled\n");
		ret = 0;
	}else{

	disable_irq_nosync(lpi->irq);
	ret = control_and_report(lpi, CONTROL_PS, 1);

	queue_delayed_work(lpi->lp_wq, &proximity_initial_value_work, 10);
	}

	mutex_unlock(&Ps_Enable_Mutex);
	if (ret!=0){
		printk("[CM36686][PS] psensor can't enable-- !!!\n");
	}else{
		printk("[CM36686][PS] psensor_enable--success!!!\n");
	}

	return ret;
}

static int psensor_disable(struct driver_info *lpi)
{
	int ret = -EIO;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	mutex_lock(&Ps_Disable_Mutex);
	if ( lpi->ps_enable == 0 ) {
		printk("[CM36686][PS] already disabled\n");
		ret = 0;
	} else{
		ret = control_and_report(lpi, CONTROL_PS,0);
	}
//	disable_irq(lpi->irq);
	mutex_unlock(&Ps_Disable_Mutex); //For next time event be guaranteed to be sent!
	//input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, 4);
	//input_sync(lpi->ps_input_dev);
	return ret;
}

static int psensor_open(struct inode *inode, struct file *file)
{
	struct driver_info *lpi = PL_Driver_Data;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (lpi->psensor_opened)
		return -EBUSY;

	lpi->psensor_opened = 1;
	return 0;
}

static int psensor_release(struct inode *inode, struct file *file)
{
	struct driver_info *lpi = PL_Driver_Data;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	lpi->psensor_opened = 0;

//	return psensor_disable(lpi);
	return 0;
}

static long psensor_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	int val;
	int rc ;
	struct driver_info *lpi = PL_Driver_Data;
	char enPcalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if(DEBUG_VALUE) printk("[CM36686][PS] cmd %d\n" , _IOC_NR(cmd));

	switch (cmd) {
	case CAPELLA_CM36686_IOCTL_ENABLE:
		printk("[CM36686][PS] CAPELLA_CM36686_IOCTL_ENABLE \n");
		if (get_user(val, (unsigned int __user *)arg))
			return -EFAULT;

		if (val){
			return psensor_enable(lpi);
		}else{
			return psensor_disable(lpi);
		}

		break;
	case CAPELLA_CM36686_IOCTL_GET_ENABLED:
		printk("[CM36686][PS] CAPELLA_CM36686_IOCTL_GET_ENABLED \n");
		return put_user(lpi->ps_enable, (unsigned int __user *)arg);

	/*case ASUS_PSENSOR_IOCTL_START:
			printk("[CM36686]: ASUS_PSENSOR_IOCTL_START  \n" );
			break;

	case ASUS_PSENSOR_IOCTL_CLOSE:
			printk("[CM36686]: ASUS_PSENSOR_IOCTL_CLOSE \n" );
			break;*/
	case ASUS_PSENSOR_IOCTL_GETDATA:
		{
			uint16_t ps_adc_value = 0;
			int ret=0;

			rc = 0 ;
			printk("[CM36686][PS] ASUS_PSENSOR_IOCTL_GETDATA \n");
			ret = get_ps_adc_value(&ps_adc_value);
			printk("[CM36686][PS] get_ps_adc_value:%d \n",ret);
			if (ret < 0) {
					printk("[CM36686][PS] can't to get_ps_adc_value. \n");
					rc = -EIO;
					goto pend;
			}

			if (copy_to_user(argp, &ps_adc_value, sizeof(ps_adc_value) ) ) {
					printk("[CM36686][PS] can't to copy psense data to user space.\n");
						rc = -EFAULT;
						goto pend;
			}

			  printk("[CM36686][PS] ASUS_PSENSOR_IOCTL_GETDATA end\n");
		}
		break;
	case ASUS_PSENSOR_SETCALI_DATA:
		if(DEBUG_FUN) printk("[CM36686][PS]:ASUS_PSENSOR_SETCALI_DATA \n");
		rc = 0 ;
		memset(PSensor_CALIDATA, 0, 2*sizeof(int));
		if (copy_from_user(PSensor_CALIDATA, argp, sizeof(PSensor_CALIDATA))){
			rc = -EFAULT;
			goto pend;
		}

		printk("[CM36686][PS] ASUS_PSENSOR SETCALI_DATA : PSensor_CALIDATA[0] :  %d ,PSensor_CALIDATA[1]:  %d \n",
			 PSensor_CALIDATA[0],PSensor_CALIDATA[1]);

		if(PSensor_CALIDATA[0] <= 0||PSensor_CALIDATA[1] <= 0
			||PSensor_CALIDATA[0] <= PSensor_CALIDATA[1] )
			rc =  -EINVAL;


		lpi->ps_away_thd_set = PSensor_CALIDATA[1] ;
		lpi->ps_close_thd_set = PSensor_CALIDATA[0];
		printk("[CM36686][PS] ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",lpi->ps_close_thd_set, lpi->ps_away_thd_set);
		if(IC_Ver == 0)
		{
			_CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_THD, (lpi->ps_close_thd_set <<8)| lpi->ps_away_thd_set);
		} else {
			_CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_THDL, lpi->ps_away_thd_set);
			_CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_THDH, lpi->ps_close_thd_set);
		}
		break;

	case ASUS_PSENSOR_EN_CALIBRATION:
		printk("[CM36686][PS] ASUS_PSENSOR_EN_CALIBRATION \n");
		rc = 0 ;
		if (copy_from_user(&enPcalibration_flag , argp, sizeof(enPcalibration_flag ))){
			rc = -EFAULT;
			goto pend;
		}
		En_PSensor_Cal =  enPcalibration_flag ;
		if(DEBUG_VALUE) printk("[CM36686][PS] ASUS_PSENSOR_EN_CALIBRATION : En_PSensor_Cal is : %d  \n",En_PSensor_Cal);
		break;

	default:
		printk("[CM36686][PS] invalid cmd %d\n", _IOC_NR(cmd));
		return -EINVAL;
	}

	pend:
 		return rc;
}

static const struct file_operations psensor_fops = {
	.owner = THIS_MODULE,
	.open = psensor_open,
	.release = psensor_release,
	.unlocked_ioctl = psensor_ioctl
};

struct miscdevice psensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "psensor",
	.fops = &psensor_fops
};

void lightsensor_set_kvalue_cm36686(struct driver_info *lpi)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if (!lpi) {
		printk("[CM36686][LS] ls_info is empty\n");
		return;
	}

	if(DEBUG_VALUE) printk("[CM36686][LS] ALS calibrated Als_Cal_Adc=0x%x\n", Als_Cal_Adc);

	if (Als_Cal_Adc >> 16 == ALS_CALIBRATED)
		lpi->Als_Cal_Adc = Als_Cal_Adc & 0xFFFF;
	else{
		lpi->Als_Cal_Adc = 0;
		printk("[CM36686][LS]  no ALS calibrated\n");
	}

	if (lpi->Als_Cal_Adc && lpi->golden_adc > 0) {
		lpi->Als_Cal_Adc = (lpi->Als_Cal_Adc > 0 && lpi->Als_Cal_Adc < 0x1000) ?
				lpi->Als_Cal_Adc : lpi->golden_adc;
		lpi->als_gadc = lpi->golden_adc;
	}else{
		lpi->Als_Cal_Adc = 1;
		lpi->als_gadc = 1;
	}

	if(DEBUG_VALUE) printk("[CM36686][LS] Als_Cal_Adc=0x%x, als_gadc=0x%x\n",lpi->Als_Cal_Adc, lpi->als_gadc);
}

static int lightsensor_enable(struct driver_info *lpi)
{
	int ret = -EIO;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	mutex_lock(&Als_Enable_Mutex);
	if (lpi->als_enable) {
		printk("[CM36686][LS] already enabled\n");
		ret = 0;
	}else{
		ret = control_and_report(lpi, CONTROL_ALS, 1);
	}
	mutex_unlock(&Als_Enable_Mutex);
	return ret;
}

static int lightsensor_disable(struct driver_info *lpi)
{
	int ret = -EIO;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	mutex_lock(&Als_Disable_Mutex);

	if ( lpi->als_enable == 0 ) {
		printk("[CM36686][LS]  already disabled\n");
		ret = 0;
	}else{
		ret = control_and_report(lpi, CONTROL_ALS, 0);
	}
	mutex_unlock(&Als_Disable_Mutex);
	return ret;
}

static int lightsensor_open(struct inode *inode, struct file *file)
{
	struct driver_info *lpi = PL_Driver_Data;
	int rc = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	if (lpi->lightsensor_opened) {
		printk("[CM36686][LS] already opened\n");
		rc = -EBUSY;
	}
	lpi->lightsensor_opened = 1;
	return rc;
}

static int lightsensor_release(struct inode *inode, struct file *file)
{
	struct driver_info *lpi = PL_Driver_Data;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	lpi->lightsensor_opened = 0;
	return 0;
}

static long lightsensor_ioctl(struct file *file, unsigned int cmd,unsigned long arg)
{
	int rc=0, val;
	struct driver_info *lpi = PL_Driver_Data;
	char encalibration_flag = 0 ;
	void __user *argp = (void __user *)arg;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if(DEBUG_VALUE) printk("[CM36686][LS]  cmd = %d\n",cmd);
	switch (cmd) {
	case LIGHTSENSOR_IOCTL_ENABLE:
		if (get_user(val, (unsigned int __user *)arg)) {
			rc = -EFAULT;
			break;
		}
		if(DEBUG_VALUE) printk("[CM36686][LS] LIGHTSENSOR_IOCTL_ENABLE, value = %d\n", val);

		rc = val ? lightsensor_enable(lpi) : lightsensor_disable(lpi);
		break;
	case LIGHTSENSOR_IOCTL_GET_ENABLED:
		val = lpi->als_enable;
		printk("[CM36686][LS] LIGHTSENSOR_IOCTL_GET_ENABLED, enabled %d\n", val);

		rc = put_user(val, (unsigned int __user *)arg);
		break;

	case ASUS_LIGHTSENSOR_IOCTL_START:
		printk("[CM36686][LS] ASUS_LIGHTSENSOR_IOCTL_START  \n");
		break;
	case ASUS_LIGHTSENSOR_IOCTL_CLOSE:
		printk("[CM36686][LS] ASUS_LIGHTSENSOR_IOCTL_CLOSE \n");
		break;
	case ASUS_LIGHTSENSOR_IOCTL_GETDATA:
	{
			uint16_t adc_value = 0;
			int report_lux;
			rc = 0 ;
			if(DEBUG_FUN) printk("[CM36686][LS] ASUS_LIGHTSENSOR_IOCTL_GETDATA \n");

			mutex_lock(&Driver_Control_Mutex);
			get_ls_adc_value(&adc_value, 0);
			mutex_unlock(&Driver_Control_Mutex);
			report_lux = adc_value/Sensitivity_X;;
			if(En_LSensor_Cal == 1 ){//calibration enable
				report_lux = calibration_light(LSensor_CALIDATA[1], LSensor_CALIDATA[0], report_lux);
				printk("[CM36686][LS] calibration-report_lux is %d\n", report_lux);
			} else {
				printk("[CM36686][LS] no calibration-report_lux is %d\n", report_lux);
			}

			if ( copy_to_user(argp, &report_lux, sizeof(report_lux) ) ) {
				printk("can't to copy lightsense data to user space.\n");
				rc = -EFAULT;
				goto end;
			}
			printk("[CM36686][LS] ASUS_LIGHTSENSOR_IOCTL_GETDATA end\n");
		}
		break;
	case ASUS_LIGHTSENSOR_SETCALI_DATA:
		if(DEBUG_FUN) printk("[CM36686][LS] ASUS_LIGHTSENSOR_SETCALI_DATA \n");
		rc = 0 ;
		memset(LSensor_CALIDATA, 0, 2*sizeof(int));

		if (copy_from_user(LSensor_CALIDATA, argp, sizeof(LSensor_CALIDATA)))
		{
			rc = -EFAULT;
			goto end;
		}

		printk("[CM36686][LS] ASUS_LIGHTSENSOR SETCALI_DATA : LSensor_CALIDATA[0] :  %d ,LSensor_CALIDATA[1]:  %d \n",
			 LSensor_CALIDATA[0],LSensor_CALIDATA[1]);

		if(LSensor_CALIDATA[0] <= 0||LSensor_CALIDATA[1] <= 0
			||LSensor_CALIDATA[0] >= LSensor_CALIDATA[1] )
			rc =  -EINVAL;
		break;
	case ASUS_LIGHTSENSOR_EN_CALIBRATION:
		if(DEBUG_FUN) printk("[CM36686][LS] ASUS_LIGHTSENSOR_EN_CALIBRATION \n");
		rc = 0 ;
		if (copy_from_user(&encalibration_flag , argp, sizeof(encalibration_flag )))
		{
			rc = -EFAULT;
			goto end;
		}
		En_LSensor_Cal =  encalibration_flag ;
		printk("[CM36686][LS] ASUS_LIGHTSENSOR_EN_CALIBRATION : En_LSensor_Cal is : %d  \n",En_LSensor_Cal);
		break;
	default:
		printk("[CM36686][LS] invalid cmd %d\n", _IOC_NR(cmd));
		rc = -EINVAL;
	}
end:
    return rc;
}

static const struct file_operations lightsensor_fops = {
	.owner = THIS_MODULE,
	.open = lightsensor_open,
	.release = lightsensor_release,
	.unlocked_ioctl = lightsensor_ioctl
};

static struct miscdevice lightsensor_misc = {
	.minor = MISC_DYNAMIC_MINOR,
	.name = "lightsensor",
	.fops = &lightsensor_fops
};

static ssize_t ps_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint16_t value;
	int ret;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	psensor_enable(PL_Driver_Data);
	get_ps_adc_value(&value);
	ret = sprintf(buf, "ADC[0x%04X], ENABLE = %d\n", value, PL_Driver_Data->ps_enable);
	return ret;
}

static ssize_t ps_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ps_en;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ps_en = -1;
	sscanf(buf, "%d", &ps_en);

	if (ps_en != 0 && ps_en != 1
		&& ps_en != 10 && ps_en != 13 && ps_en != 16)
		return -EINVAL;


	if (ps_en) {
		printk("[CM36686][PS]  ps_en=%d\n", ps_en);
		psensor_enable(PL_Driver_Data);
	}else{
		psensor_disable(PL_Driver_Data);
	}

	return count;
}
static DEVICE_ATTR(ps_adc, ATTRIBUTES_PERMISSION, ps_adc_show, ps_enable_store);
unsigned PS_cmd_test_value_cm36686;

static ssize_t ps_parameters_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret = sprintf(buf, "PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set, PS_cmd_test_value_cm36686);

	return ret;
}

static ssize_t ps_parameters_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	char *token[10];
	int i;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	for (i = 0; i < 3; i++)
		token[i] = strsep((char **)&buf, " ");

	PL_Driver_Data->ps_close_thd_set = simple_strtoul(token[0], NULL, 16);
	PL_Driver_Data->ps_away_thd_set = simple_strtoul(token[1], NULL, 16);
	PS_cmd_test_value_cm36686 = simple_strtoul(token[2], NULL, 16);

	printk("[CM36686][PS]Set PS_close_thd_set = 0x%x, PS_away_thd_set = 0x%x, PS_cmd_cmd:value = 0x%x\n",
		PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set, PS_cmd_test_value_cm36686);


	return count;
}

static DEVICE_ATTR(ps_parameters, ATTRIBUTES_PERMISSION, ps_parameters_show, ps_parameters_store);

static ssize_t ps_conf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	return sprintf(buf, "PS_CONF1 = 0x%x, PS_CONF3 = 0x%x\n", PL_Driver_Data->ps_conf1_val, PL_Driver_Data->ps_conf3_val);
}

static ssize_t ps_conf_store(struct device *dev, struct device_attribute *attr,	const char *buf, size_t count)
{
	int code1, code2;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	sscanf(buf, "0x%x 0x%x", &code1, &code2);
	printk("[CM36686][PS]  store value PS conf1 reg = 0x%x PS conf3 reg = 0x%x\n", code1, code2);
	PL_Driver_Data->ps_conf1_val = code1;
	PL_Driver_Data->ps_conf3_val = code2;

	_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, PS_CONF3, PL_Driver_Data->ps_conf3_val );
	_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, PS_CONF1, PL_Driver_Data->ps_conf1_val );

	return count;
}

static DEVICE_ATTR(ps_conf, ATTRIBUTES_PERMISSION, ps_conf_show, ps_conf_store);

static ssize_t cm36686_dump_reg(struct device *dev, struct device_attribute *attr, char *buf)
{
	uint8_t i = 0;
	uint16_t value;
	int ret;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	for(i = 0; i <0xD; i++)
	{
		ret = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, i, &value);
		printk("cmd =%02x value = %02x\n", i, value);
	}

	return sprintf(buf, "%d\n", ret);
}
static DEVICE_ATTR(cm36686_dump, 0440, cm36686_dump_reg, NULL);


static ssize_t ps_thd_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int dummy;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if(IC_Ver==0)
	{
		uint16_t thd;
		ret = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, PS_THD, &thd);
		if (!ret){
			printk("[CM36686][PS]threshold = 0x%x\n", thd);
		}else{
			printk("[CM36686][PS] --- can't to read threshold  \n");
		}
		dummy = sprintf(buf, "%s ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set);
		return dummy;
	}
	else
	{
		ret = sprintf(buf, "[CM36686][PS]PS Hi/Low THD ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set, PL_Driver_Data->ps_away_thd_set);
		return ret;
	}
}

static ssize_t ps_thd_store(struct device *dev,	struct device_attribute *attr, const char *buf, size_t count)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if(IC_Ver ==0)
	{
		int code;
		int ret=0;
		sscanf(buf, "0x%x", &code);
		printk("[PS] store value = 0x%x\n", code);

		PL_Driver_Data->ps_away_thd_set = code &0xFF;
		PL_Driver_Data->ps_close_thd_set = (code &0xFF00)>>8;
		_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, PS_THD, (PL_Driver_Data->ps_close_thd_set <<8)| PL_Driver_Data->ps_away_thd_set);
		printk("[PS] ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set);
		//ret = sprintf(buf,"[PS]%s: ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n", __func__, PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set );

		return ret;
	}
	else
	{
		int code1, code2;
		sscanf(buf, "0x%x 0x%x", &code1, &code2);

		PL_Driver_Data->ps_close_thd_set = code1;
		PL_Driver_Data->ps_away_thd_set = code2;

		_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, PS_THDH, PL_Driver_Data->ps_close_thd_set );
		_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, PS_THDL, PL_Driver_Data->ps_away_thd_set );
		printk("[CM36686][PS] ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set, PL_Driver_Data->ps_away_thd_set);

		//ret = sprintf(buf,"[CM36686][PS]%s: ps_close_thd_set = 0x%04x(%d), ps_away_thd_set = 0x%04x(%d)\n", __func__, PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set, PL_Driver_Data->ps_away_thd_set);
		return count;
	}
}


static DEVICE_ATTR(ps_thd, ATTRIBUTES_PERMISSION, ps_thd_show, ps_thd_store);

static ssize_t ps_status_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	int val;
	uint16_t ps_data = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if(IC_Ver==0)
	{

		psensor_enable(PL_Driver_Data);
		get_stable_ps_adc_value(&ps_data);
		val = (ps_data >= PL_Driver_Data->ps_close_thd_set) ? 0 : 1;
	// ret  = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, INT_FLAG, &thd);

		printk("[CM36686][PS]INT_FLAG = 0x%x\n", ps_data);

		ret = sprintf(buf, "%s ps status= %x\n", __func__, val);
		return ret;
	} else {
		psensor_enable(PL_Driver_Data);
		get_stable_ps_adc_value(&ps_data);
		val = (ps_data >= PL_Driver_Data->ps_close_thd_set) ? 0 : 1;
 	 // ret  = _CM36XXX_I2C_Read_Word(PL_Driver_Data->slave_addr, INT_FLAG, &thd);
		printk("[CM36686][PS]INT_FLAG = 0x%x\n", ps_data);
		ret = sprintf(buf, "%s ps status= %x\n", __func__, val);
		return ret;
	}

}

static DEVICE_ATTR(ps_status, 0440, ps_status_show, NULL);

static ssize_t ps_hw_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret = sprintf(buf, "PS1: reg = 0x%x, PS3: reg = 0x%x, ps_close_thd_set = 0x%x, ps_away_thd_set = 0x%x\n",
	PL_Driver_Data->ps_conf1_val, PL_Driver_Data->ps_conf3_val, PL_Driver_Data->ps_close_thd_set, PL_Driver_Data->ps_away_thd_set);
	return ret;
}

static ssize_t ps_hw_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int code;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	sscanf(buf, "0x%x", &code);
	printk("[CM36686][PS] store value = 0x%x\n", code);
	return count;
}

static DEVICE_ATTR(ps_hw, ATTRIBUTES_PERMISSION, ps_hw_show, ps_hw_store);

static ssize_t ps_calling_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret = sprintf(buf,"[CM36686][PS]%s: calling status is %d\n", __func__, PL_Driver_Data->status_calling);
	return ret;
}

static ssize_t ps_calling_store(struct device *dev,struct device_attribute *attr,const char *buf, size_t count)
{
    int status_calling;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	status_calling = -1;
	sscanf(buf, "%d", &status_calling);
	PL_Driver_Data->status_calling = status_calling;
	printk("[CM36686][PS] calling status is %d\n", PL_Driver_Data->status_calling);
	return count;
}

static DEVICE_ATTR(ps_calling, ATTRIBUTES_PERMISSION, ps_calling_show, ps_calling_store);

static ssize_t ls_adc_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t report_lux;
	uint16_t als_step;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	lightsensor_enable(PL_Driver_Data);
	get_ls_adc_value(&als_step, 0);
	report_lux=als_step/Sensitivity_X;
	printk("[CM36686][LS]  ADC = %d !!!\n", report_lux);
	ret = sprintf(buf, "%d\n", report_lux);
	return ret;
}

static ssize_t ls_adc_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{   // NOP
	return count;
}

static DEVICE_ATTR(ls_adc, ATTRIBUTES_PERMISSION, ls_adc_show, ls_adc_store);

static ssize_t ls_value_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret;
	uint16_t report_lux;
	uint16_t als_step;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	lightsensor_enable(PL_Driver_Data);
	get_ls_adc_value(&als_step, 0);
	report_lux=als_step/Sensitivity_X;
	if(En_LSensor_Cal == 1 ){//calibration enable
		report_lux = calibration_light(LSensor_CALIDATA[1], LSensor_CALIDATA[0], report_lux);
		printk("[CM36686][LS] calibration lux = %d !!!\n", report_lux);
	} else {
		printk("[CM36686][LS] without calibration lux = %d !!!\n", report_lux);
	}
	ret = sprintf(buf, "%d\n", report_lux);

	return ret;
}
static ssize_t ls_value_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{   // NOP
	return count;
}
static DEVICE_ATTR(ls_value, ATTRIBUTES_PERMISSION, ls_value_show, ls_value_store);

static ssize_t ls_enable_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret = sprintf(buf, "Light sensor Auto Enable = %d\n", PL_Driver_Data->als_enable);
	return ret;
}

static ssize_t ls_enable_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int ret = 0;
	int ls_auto;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ls_auto = -1;
	sscanf(buf, "%d", &ls_auto);

	if (ls_auto != 0 && ls_auto != 1 && ls_auto != 147)
		return -EINVAL;

	if (ls_auto) {
		PL_Driver_Data->ls_calibrate = (ls_auto == 147) ? 1 : 0;
		ret = lightsensor_enable(PL_Driver_Data);
	}else{
		PL_Driver_Data->ls_calibrate = 0;
		ret = lightsensor_disable(PL_Driver_Data);
	}

	printk("[CM36686][LS] PL_Driver_Data->als_enable = %d, PL_Driver_Data->ls_calibrate = %d, ls_auto=%d\n",
		PL_Driver_Data->als_enable, PL_Driver_Data->ls_calibrate, ls_auto);

	if (ret < 0)
		printk("[CM36686][LS]can't set auto light sensor \n");

	return count;
}

static DEVICE_ATTR(ls_auto, ATTRIBUTES_PERMISSION, ls_enable_show, ls_enable_store);

static ssize_t ls_conf_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	return sprintf(buf, "ALS_CONF = 0x%x\n", PL_Driver_Data->ls_cmd);
}

static ssize_t ls_conf_store(struct device *dev, struct device_attribute *attr, const char *buf, size_t count)
{
	int value = 0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	sscanf(buf, "0x%x", &value);

	PL_Driver_Data->ls_cmd = value;
	printk("[LS]set ALS_CONF = %x\n", PL_Driver_Data->ls_cmd);

	_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, ALS_CONF, PL_Driver_Data->ls_cmd);

	return count;
}

static DEVICE_ATTR(ls_conf, ATTRIBUTES_PERMISSION, ls_conf_show, ls_conf_store);

static int lightsensor_setup(struct driver_info *lpi)
{
	int ret;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	lpi->ls_input_dev = input_allocate_device();

	if (!lpi->ls_input_dev) {
		printk("[CM36686][LS] could not allocate ls input device\n");
		return -ENOMEM;
	}

	lpi->ls_input_dev->name = "lightsensor-level";
	set_bit(EV_ABS, lpi->ls_input_dev->evbit);
	//input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9, 0, 0);
	if(Als_It==CM36686_ALS_IT_80ms) {
		input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9000, 0, 0);
	} else if(Als_It==CM36686_ALS_IT_160ms) {
		input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 4500, 0, 0);
	} else if(Als_It==CM36686_ALS_IT_320ms) {
		input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 2300, 0, 0);
	} else if(Als_It==CM36686_ALS_IT_640ms) {
		input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 1200, 0, 0);
	} else {
		input_set_abs_params(lpi->ls_input_dev, ABS_MISC, 0, 9000, 0, 0);
	}
	ret = input_register_device(lpi->ls_input_dev);

	if (ret < 0) {
		printk("[CM36686][LS] can't register ls input device\n");
		goto err_free_ls_input_device;
	}

	ret = misc_register(&lightsensor_misc);

	if (ret < 0) {
		printk("[CM36686][LS] can't register ls misc device\n");
		goto err_unregister_ls_input_device;
	}

	return ret;

err_unregister_ls_input_device:

	input_unregister_device(lpi->ls_input_dev);

err_free_ls_input_device:

	input_free_device(lpi->ls_input_dev);
	return ret;
}

static int psensor_setup(struct driver_info *lpi)
{
	int ret;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	lpi->ps_input_dev = input_allocate_device();
	if (!lpi->ps_input_dev) {
		printk("[CM36686][PS] could not allocate ps input device\n");
		return -ENOMEM;
	}

	lpi->ps_input_dev->name = "proximity";
	set_bit(EV_ABS, lpi->ps_input_dev->evbit);
	input_set_abs_params(lpi->ps_input_dev, ABS_DISTANCE, 0, 1, 0, 0);
	ret = input_register_device(lpi->ps_input_dev);

	if (ret < 0) {
		printk("[CM36686][PS] could not register ps input device\n");
		goto err_free_ps_input_device;
	}

	ret = misc_register(&psensor_misc);

	if (ret < 0) {
		printk("[CM36686][PS] could not register ps misc device\n");
		goto err_unregister_ps_input_device;
	}
	return ret;
err_unregister_ps_input_device:
	input_unregister_device(lpi->ps_input_dev);

err_free_ps_input_device:
	input_free_device(lpi->ps_input_dev);

	return ret;
}

static int initial_driver(struct driver_info *lpi)
{
	int ret;
	uint16_t idReg;

 	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	ret = _CM36XXX_I2C_Read_Word(lpi->slave_addr, ID_REG, &idReg);
	idReg = idReg&0x00FF;
	if(DEBUG_VALUE) printk("[CM36686] :0x%x  ID_REG: %d \n", lpi->slave_addr,idReg);
	if (ret < 0) {
		if (record_init_fail == 0)
				record_init_fail = 1;
		if(ret==-5) printk("[CM36686] is not present !\n");
		printk("[CM36686] ret = %d \n", ret);
		return -ENOMEM;
	}
	if(idReg == 0x0083)
	{
		Sensitivity_X = SENSITIVITY_20;
		IC_Ver = 0;
	} else if(idReg == 0x0086){// 0x0086 cm36686
		if(Als_It==CM36686_ALS_IT_80ms) {
			Sensitivity_X = SENSITIVITY_8;
		} else if(Als_It==CM36686_ALS_IT_160ms) {
			Sensitivity_X = SENSITIVITY_16;
		} else if(Als_It==CM36686_ALS_IT_320ms) {
			Sensitivity_X = SENSITIVITY_33;
		} else if(Als_It==CM36686_ALS_IT_640ms) {
			Sensitivity_X = SENSITIVITY_66;
		} else {
			Sensitivity_X = SENSITIVITY_8;
		}
		IC_Ver = 1;
	} else {//0x0082 cm32683
		Sensitivity_X = SENSITIVITY_25;
		IC_Ver = 1;
	}
	if(DEBUG_VALUE) printk("[CM36686]p-sensor sensitivity = %d\n", Sensitivity_X);
	return 0;
}

static int driver_setup(struct driver_info *lpi)
{
	int ret = 0;
	int irq=0;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	als_power(1);
	msleep(5);

	ret = gpio_request(lpi->intr_pin, "cm36686_intr_pin");
	if(DEBUG_VALUE) printk("[CM36686][PS] intr_pin:%d ,gpio_request ret:(%d)\n", lpi->intr_pin, ret);
	ret = gpio_direction_input(lpi->intr_pin);
	if (ret < 0) {
		printk("[CM36686][PS] can't to set gpio %d as input (%d)\n", lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	/*Default disable P sensor and L sensor*/
	ls_initial_cmd(lpi);
	psensor_initial_cmd(lpi);

	irq =  gpio_to_irq(lpi->intr_pin);
	if(DEBUG_VALUE) printk("[CM36686][PS]  lpi->irq:%d \n",lpi->irq);
	lpi->irq =irq;
	ret = request_irq(lpi->irq,  CM36686_irq_handler,  IRQF_TRIGGER_FALLING | IRQF_ONESHOT,"cm36686_irq", lpi);

	if (ret < 0) {
		printk("[CM36686][PS]can't request_irq req_irq(%d),gpio %d (%d)\n", lpi->irq, lpi->intr_pin, ret);
		goto fail_free_intr_pin;
	}

	return ret;

fail_free_intr_pin:
	gpio_free(lpi->intr_pin);
	return ret;
}

static int psensor_calibration_read(struct seq_file *m, void *v)
{
	return seq_printf(m, "ps_close_thd_set:%d ,ps_away_thd_set:%d \n",PL_Driver_Data->ps_close_thd_set,PL_Driver_Data->ps_away_thd_set);
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
	int i,CalidataX,CalidataL,CalidataH;
	uint16_t ps_raw_value=0x00,ps_value;

	struct file *filp = NULL;
	char  tmpbuf[20] = "";
	int tmplen=0;
	mm_segment_t old_fs;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	if ( PL_Driver_Data->ps_enable ) {
		printk("[CM36686][PS]  already enabled\n");
	}else{
		psensor_enable(PL_Driver_Data);
	}

	msleep(100);
	get_ps_adc_value(&ps_raw_value);
	for(i=0;i<5;i++){
		get_ps_adc_value(&ps_value);
		if(ps_value<ps_raw_value) {
			ps_raw_value=ps_value;
		}
	}

	if(ps_raw_value==0x00 && Read_HW_ID()!=HW_ID_EVB) {
		printk("[CM36686][PS] driver not ready");
		return seq_printf(m, "0");
	}

	printk("[CM36686][PS] psensor_calibration ps_raw_value:%d \n",  ps_raw_value);
	printk("[CM36686][PS]  PS_CLOSE_THD = (%d), PS_AWAY_THD = (%d)\n", PS_CLOSE_THD , PS_AWAY_THD);
	PL_Driver_Data->ps_close_thd_set = (abs(ps_raw_value/3)+PS_CLOSE_THD)+ps_raw_value;
	PL_Driver_Data->ps_away_thd_set = (abs(ps_raw_value/6)+PS_AWAY_THD)+ps_raw_value;
	_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, PS_THDH, PL_Driver_Data->ps_close_thd_set );
	_CM36XXX_I2C_Write_Word(PL_Driver_Data->slave_addr, PS_THDL, PL_Driver_Data->ps_away_thd_set );
	CalidataX=ps_raw_value-(abs(ps_raw_value/6));
	CalidataL=PL_Driver_Data->ps_away_thd_set;
	CalidataH=PL_Driver_Data->ps_close_thd_set;
	printk("CalidataX = (%d), CalidataH = (%d), CalidataL = (%d)\n", CalidataX , CalidataH ,CalidataL);
	//save calibration
	filp = filp_open(PS_CAL_PATH, O_RDWR | O_CREAT,0660);
	if (IS_ERR(filp)) {
		printk("[CM36686][PS] can't open %s \n",PS_CAL_PATH);
		return seq_printf(m, "0");
	} else {
	    sprintf(tmpbuf,"%d %d %d ", CalidataX,CalidataH,CalidataL);
		tmplen=strlen(tmpbuf);
		printk("[CM36686][PS] tmpbuf:%s , tmplen:%d  \n",tmpbuf, tmplen);
		old_fs = get_fs();
		set_fs(KERNEL_DS);
		filp->f_op->write(filp, tmpbuf,  tmplen, &filp->f_pos);
		set_fs(old_fs);
		filp_close(filp, NULL);
		printk("[CM36686][PS] save %s \n",PS_CAL_PATH);
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


static void __vpro3_power_ctrl(bool on)
{
        u8 addr, value;
        addr = 0xae;
        if (intel_scu_ipc_ioread8(addr, &value))
                printk("%s: %d: failed to read vPro3\n", __func__, __LINE__);
        printk("[DEBUG] vpro3 = %x\n", value);

        /* Control vPROG3 power rail with 2.85v. */
        if (on)
                value |= 0x1;
        else
                value &= ~0x1;

        if (intel_scu_ipc_iowrite8(addr, value))
                printk("%s: %d: failed to write vPro3\n",
                                __func__, __LINE__);
}

static int CM36686_probe (struct i2c_client *client,const struct i2c_device_id *id)
{
	int ret = 0;
	struct driver_info *lpi;
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA)) {
		printk("[CM36686] I2C bus not ready -1 \n"  );
		return -EIO;
	} else {
		printk("[CM36686] I2C bus ready \n"  );
	}

	lpi = kzalloc(sizeof(struct driver_info), GFP_KERNEL);

	if (!lpi)
		return -ENOMEM;

	lpi->i2c_client = client;

	Als_It=CM36686_ALS_IT_80ms;
	lpi->intr_pin = 44;
	lpi->power = NULL;
	lpi->slave_addr = client->addr;
	lpi->ps_away_thd_set = PS_AWAY_THD;
	lpi->ps_close_thd_set = PS_CLOSE_THD;
	lpi->ps_conf1_val = CM36686_PS_ITB_1 | CM36686_PS_DR_1_320 | CM36686_PS_IT_1_6T | CM36686_PS_PERS_2 | CM36686_PS_RES_1 |CM36686_PS_INT_IN_AND_OUT;
	lpi->ps_conf3_val =  CM36686_PS_MS_NORMAL | CM36686_PS_PROL_255 | CM36686_PS_SMART_PERS_ENABLE;
	lpi->status_calling = 0;
	lpi->ls_cmd  = Als_It | CM36686_ALS_GAIN_2;
	lpi->record_clear_int_fail=0;

	if(DEBUG_VALUE) printk("[CM36686] ls_cmd 0x%x\n", lpi->ls_cmd);

	PL_Driver_Data = lpi;

	ret = initial_driver(lpi);
	if (ret < 0) {
		printk("[CM36686] can't to initial CM36686 (%d)\n", ret);
		goto err_fail_init_data;
	}

	mutex_init(&Driver_Control_Mutex);
	mutex_init(&Als_Enable_Mutex);
	mutex_init(&Als_Disable_Mutex);
	mutex_init(&Als_Get_Adc_Mutex);

	ret = lightsensor_setup(lpi);
	if (ret < 0) {
		printk("[CM36686][LS] lightsensor can't setup!!\n");
		goto err_lightsensor_setup;
	}
	mutex_init(&Ps_Enable_Mutex);
	mutex_init(&Ps_Disable_Mutex);
	mutex_init(&Ps_Get_Adc_Mutex);

	ret = psensor_setup(lpi);
	if (ret < 0) {
		printk("[CM36686][PS] psensor can't setup!!\n");
		goto err_psensor_setup;
	}

	//SET LUX STEP FACTOR HERE
	// if adc raw value one step = 5/100 = 1/20 = 0.05 lux
	// the following will set the factor 0.05 = 1/20
	// and lpi->golden_adc = 1;
	// set Als_Cal_Adc = (ALS_CALIBRATED <<16) | 20;

	Als_Cal_Adc = (ALS_CALIBRATED <<16) | 20;
	lpi->golden_adc = 1;
	//ls calibrate always set to 1
	lpi->ls_calibrate = 1;
	lightsensor_set_kvalue_cm36686(lpi);

	wake_lock_init(&(lpi->ps_wake_lock), WAKE_LOCK_SUSPEND, "proximity");

	ret = driver_setup(lpi);
	if (ret < 0) {
		printk("[CM36686][PS]: CM36686 can't setup !\n" );
		goto err_driver_setup;
	}
	lpi->lp_wq = create_singlethread_workqueue("CM36686_wq");
	if (!lpi->lp_wq) {
		printk("[CM36686][PS] can't create workqueue\n");
		ret = -ENOMEM;
		goto err_create_singlethread_workqueue;
	}

	lpi->driver_class = class_create(THIS_MODULE, "optical_sensors");

	if (IS_ERR(lpi->driver_class)) {
		ret = PTR_ERR(lpi->driver_class);
		lpi->driver_class = NULL;
		goto err_create_class;
	}

	lpi->ls_dev = device_create (lpi->driver_class,NULL,0,"%s","lightsensor");

	if (unlikely(IS_ERR(lpi->ls_dev))) {
		ret = PTR_ERR(lpi->ls_dev);
		lpi->ls_dev = NULL;
		goto err_create_ls_device;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_adc);
	if (ret)
		goto err_create_ls_device_file;
	
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_value);
	if (ret)
		goto err_create_ls_device_file;	

	/* register the attributes */
	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_auto);

	if (ret)
		goto err_create_ls_device_file;

	ret = device_create_file(lpi->ls_dev, &dev_attr_ls_conf);

	if (ret)
		goto err_create_ls_device_file;

	lpi->ps_dev = device_create( lpi->driver_class, NULL, 0,"%s","proximity");

	if (unlikely(IS_ERR(lpi->ps_dev))) {
 		  ret = PTR_ERR(lpi->ps_dev);
          lpi->ps_dev = NULL;
          goto err_create_ls_device_file;
	}

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_adc);

	if (ret)
		goto err_create_ps_device;

	ret = device_create_file( lpi->ps_dev, &dev_attr_ps_parameters);

	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_conf);

	if (ret)
		goto err_create_ps_device;

	/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_thd);

	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_status);
	if (ret)
		goto err_create_ps_device;

		/* register the attributes */
	ret = device_create_file(lpi->ps_dev, &dev_attr_cm36686_dump);

	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_hw);

	if (ret)
		goto err_create_ps_device;

	ret = device_create_file(lpi->ps_dev, &dev_attr_ps_calling);

	if (ret)
		goto err_create_ps_device;

#ifdef DEBUG_VEMMC2
	vemmc2_userCtrl_class = class_create(THIS_MODULE, "vemmc2_userCtrl_dev");
	vemmc2_userCtrl_dev = device_create(vemmc2_userCtrl_class, NULL, 0, "%s", "vemmc2_userCtrl");
	ret = device_create_file(vemmc2_userCtrl_dev, &dev_attr_vemmc2_ctrl);

	if(ret){
		device_unregister(vemmc2_userCtrl_dev);
	}

#endif //DEBUG_VEMMC2

    proc_psensor_auto_calibration_entry = proc_create("psensor_auto_calibration", ATTRIBUTES_PERMISSION, NULL, &proc_psensor_auto_calibration_send);
	if (!proc_psensor_auto_calibration_entry) {
		printk("[CM36686] can't to create proc_psensor_auto_calibration_entry\n");
	}

#ifdef CONFIG_ASUS_FACTORY_SENSOR_MODE
	proc_psensor_calibration_entry = proc_create("psensor_calibration_data", ATTRIBUTES_PERMISSION, NULL, &proc_psensor_calibration_send);
	if (!proc_psensor_calibration_entry) {
		printk("[CM36686] can't to create proc_psensor_calibration_entry\n");
	}
    Lpsensor_Entry = proc_create("lpsensor_status", ATTRIBUTES_PERMISSION, NULL,&lpsensor_proc_fops);
	if(!Lpsensor_Entry)
		printk("[CM36686] can't to create Lpsensor_Entry\n");
	Lightsensor_Entry = proc_create("lightsensor_status", ATTRIBUTES_PERMISSION, NULL,&lightsensor_proc_fops);
	if(!Lightsensor_Entry)
		printk("[CM36686] can't to create Lightsensor_Entry\n");
	Proximitysensor_Entry = proc_create("proximitysensor_status", ATTRIBUTES_PERMISSION, NULL,&Proximitysensor_proc_fops);
	if(!Proximitysensor_Entry)
		printk("[CM36686] can't create Proximitysensor_Entry\n");
#endif

	printk("[CM36686] Probe success!\n");
	return ret;

err_create_ps_device:
	device_unregister(lpi->ps_dev);

err_create_ls_device_file:
	device_unregister(lpi->ls_dev);

err_create_ls_device:
	class_destroy(lpi->driver_class);

err_create_class:

err_create_singlethread_workqueue:
	destroy_workqueue(lpi->lp_wq);
	wake_lock_destroy(&(lpi->ps_wake_lock));

err_driver_setup:
	misc_deregister(&psensor_misc);

	input_unregister_device(lpi->ls_input_dev);
	input_free_device(lpi->ls_input_dev);
	input_unregister_device(lpi->ps_input_dev);
	input_free_device(lpi->ps_input_dev);

err_psensor_setup:
	mutex_destroy(&Driver_Control_Mutex);
	mutex_destroy(&Ps_Enable_Mutex);
	mutex_destroy(&Ps_Disable_Mutex);
	mutex_destroy(&Ps_Get_Adc_Mutex);
	misc_deregister(&lightsensor_misc);

err_lightsensor_setup:
	mutex_destroy(&Als_Enable_Mutex);
    mutex_destroy(&Als_Disable_Mutex);
	mutex_destroy(&Als_Get_Adc_Mutex);

err_fail_init_data:

	return ret;
}

static int control_and_report( struct driver_info *lpi, uint8_t mode, uint16_t param )
{
	int ret=0;
	uint16_t adc_value = 0;
	uint16_t ps_data = 0;
	int level = 0,  val;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	mutex_lock(&Driver_Control_Mutex);

	if( mode == CONTROL_ALS ){
		int als_wr_result;
		if(param){
			lpi->ls_cmd &= CM36686_ALS_SD_MASK;
		}else{
			lpi->ls_cmd |= CM36686_ALS_SD;
		}
		als_wr_result =_CM36XXX_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
		if (!als_wr_result){
			lpi->als_enable=param;
		}
	}else if( mode == CONTROL_PS ){
		int ps_wr_result;
		if(param){
			lpi->ps_conf1_val &= CM36686_PS_SD_MASK;
			lpi->ps_conf1_val |= CM36686_PS_INT_IN_AND_OUT;
		}else{
			lpi->ps_conf1_val |= CM36686_PS_SD;
			lpi->ps_conf1_val &= CM36686_PS_INT_MASK;
		}

		ps_wr_result = _CM36XXX_I2C_Write_Word(lpi->slave_addr, PS_CONF1, lpi->ps_conf1_val);

		if (!ps_wr_result){
			lpi->ps_enable=param;
		}
	}

	if(mode == CONTROL_PS){
		if( param==1 ){
			//<Add for reduce enable Psensor Time>
			msleep(20);
		}
	}

	if(lpi->als_enable){
		int report_lux;
		if( mode == CONTROL_ALS
			||( mode == CONTROL_INT_ISR_REPORT
			&& ((param&INT_FLAG_ALS_IF_L)||(param&INT_FLAG_ALS_IF_H)))){
			if(DEBUG_VALUE) printk("[CM36686] mode:%d\n",mode);
			lpi->ls_cmd &= CM36686_ALS_INT_MASK;
			ret = _CM36XXX_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);

			if(mode == CONTROL_ALS) {
				if(Als_It==CM36686_ALS_IT_80ms) {
					msleep(80);
				} else if(Als_It==CM36686_ALS_IT_160ms) {
					msleep(160);
				} else if(Als_It==CM36686_ALS_IT_320ms) {
					msleep(320);
				} else if(Als_It==CM36686_ALS_IT_640ms) {
					msleep(640);
				} else {
					msleep(80);
				}
			}
			get_ls_adc_value(&adc_value, 0);
			ret = set_lsensor_range(adc_value);
			lpi->ls_cmd |= CM36686_ALS_INT_EN;
			ret = _CM36XXX_I2C_Write_Word(lpi->slave_addr, ALS_CONF, lpi->ls_cmd);
			lpi->current_level = level;
			lpi->current_adc = adc_value;
			report_lux = adc_value/Sensitivity_X;
			if(En_LSensor_Cal == 1 ){//calibration enable
				report_lux = calibration_light(LSensor_CALIDATA[1], LSensor_CALIDATA[0], report_lux);
				printk("[CM36686][LS] calibration-report_lux:%d\n", report_lux);
			} else {
				printk("[CM36686][LS] no calibration-report_lux:%d\n", report_lux);
			}

			if(mode == CONTROL_ALS){
				queue_delayed_work(lpi->lp_wq, &light_sensor_initial_value_work, (0.15)*HZ);
			}else{
				input_report_abs(lpi->ls_input_dev, ABS_MISC,report_lux);
				input_sync(lpi->ls_input_dev);
			}
		}
	}

	if(lpi->ps_enable){
		int ps_status = 0;

	if( mode == CONTROL_PS ){
		ps_status = PS_CLOSE_AND_AWAY;
	}else if(mode == CONTROL_INT_ISR_REPORT ){
		if ( param & INT_FLAG_PS_IF_CLOSE )
			ps_status |= PS_CLOSE;
		if ( param & INT_FLAG_PS_IF_AWAY )
			ps_status |= PS_AWAY;
	}

	if (ps_status!=0){
		switch(ps_status){
			case PS_CLOSE_AND_AWAY:
				//<Add for reduce enable Psensor time>
				//get_stable_ps_adc_value(&ps_data);
				get_ps_adc_value(&ps_data);
				val = (ps_data >= lpi->ps_close_thd_set) ? 0 : 1;
				printk("[CM36686][PS] proximity --ps_status = PS_CLOSE_AND_AWAY\n");
				mutex_unlock(&Driver_Control_Mutex);
				return ret;
				break;
			case PS_AWAY:
				 val = 1;
				Proximity_Int_Away = 1;
				printk("[CM36686][PS] ps_away_thd:%d ps_status = PS_AWAY\n",lpi->ps_away_thd_set);
				break;
			case PS_CLOSE:
				val = 0;
				printk("[CM36686][PS] ps_close_thd:%d ps_status = PS_CLOSE\n",lpi->ps_close_thd_set);
				break;
			};
			input_report_abs(lpi->ps_input_dev, ABS_DISTANCE, val);
			input_sync(lpi->ps_input_dev);
			Proximity_State = val;
		}
	}

  mutex_unlock(&Driver_Control_Mutex);
  return ret;
}

static int CM36686_suspend(struct device *dev)
{
	int status_calling = PL_Driver_Data->ps_enable;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	PL_Driver_Data->status_calling = status_calling;
	if (!status_calling) {

	} else {
		enable_irq_wake(PL_Driver_Data->irq);
	}
	return 0;
}

static int CM36686_resume(struct device *dev)
{
	int status_calling = PL_Driver_Data->status_calling;
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);

	if (!status_calling) {

	} else {
		if (Proximity_State == 0) {
			wake_lock_timeout(&(PL_Driver_Data->ps_wake_lock), 1 * HZ);
		}
		disable_irq_wake(PL_Driver_Data->irq);
	}
	return 0;
}

static UNIVERSAL_DEV_PM_OPS(cm36686_pm, CM36686_suspend, CM36686_resume, NULL);

static const struct i2c_device_id CM36686_i2c_id[] = {
	{CM36686_I2C_NAME, 0},
	{}
};

static struct of_device_id cm36686_match_table[] = {
	{ .compatible = "capella,cm36686",},
	{ },
};
static struct i2c_driver CM36686_driver = {
	.id_table = CM36686_i2c_id,
	.probe = CM36686_probe,
	.driver = {
		.name = CM36686_I2C_NAME,
		.owner = THIS_MODULE,
		.pm = &cm36686_pm,
		.of_match_table = cm36686_match_table,
	},
};
static struct i2c_board_info cm36686_board_info[] = {
	{
		I2C_BOARD_INFO(CM36686_I2C_NAME, CM36686_slave_addr),
	},
};

static int __init CM36686_init(void)
{
	int ret=0;
	__vpro3_power_ctrl(1);
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	ret= i2c_add_driver(&CM36686_driver);
	if (ret) {
		printk("[CM36686] i2c_add_driver can't add CM36686_driver\n");
		i2c_del_driver(&CM36686_driver);
		return ret;
	}

	ret = i2c_register_board_info(6, cm36686_board_info, ARRAY_SIZE(cm36686_board_info));
	if (ret) {
		printk("[CM36686] i2c_register_board_info can't register cm36686_board_info\n");
		return ret;
	}
    return ret;
}

static void __exit CM36686_exit(void)
{
	if(DEBUG_FUN) printk("[CM36686] %s\n", __func__);
	if(proc_psensor_auto_calibration_entry)
		remove_proc_entry("psensor_auto_calibration", NULL);

#ifdef CONFIG_ASUS_FACTORY_SENSOR_MODE
	if(proc_psensor_calibration_entry)
		remove_proc_entry("psensor_calibration_data", NULL);
	if (Lpsensor_Entry)
		remove_proc_entry("lpsensor_status", NULL);
	if(Proximitysensor_Entry)
		remove_proc_entry("proximitysensor_status", NULL);
	if(Lightsensor_Entry)
		remove_proc_entry("lightsensor_status", NULL);
#endif
	i2c_del_driver(&CM36686_driver);
	__vpro3_power_ctrl(0);
}

module_init(CM36686_init);
module_exit(CM36686_exit);
MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("CM36686 Driver");
