/******************** (C) COPYRIGHT 2010 STMicroelectronics ********************
 *
 * File Name          : lis3dh_acc.c
 * Authors            : MSH - Motion Mems BU - Application Team
 *		      : Matteo Dameno (matteo.dameno@st.com)
 *		      : Carmine Iascone (carmine.iascone@st.com)
 *		      : Both authors are willing to be considered the contact
 *		      : and update points for the driver.
 * Version            : V.1.0.8
 * Date               : 2010/Apr/01
 * Description        : LIS3DH accelerometer sensor API
 *
 *******************************************************************************
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 *
 * THE PRESENT SOFTWARE IS PROVIDED ON AN "AS IS" BASIS, WITHOUT WARRANTIES
 * OR CONDITIONS OF ANY KIND, EITHER EXPRESS OR IMPLIED, FOR THE SOLE
 * PURPOSE TO SUPPORT YOUR APPLICATION DEVELOPMENT.
 * AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
 * INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
 * CONTENT OF SUCH SOFTWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
 * INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOFTWARE IS SPECIFICALLY DESIGNED FOR EXCLUSIVE USE WITH ST PARTS.
 *
 ******************************************************************************
 Revision 1.0.0 05/11/09
 First Release;
 Revision 1.0.3 22/01/2010
  Linux K&R Compliant Release;
 Revision 1.0.5 16/08/2010
  modified _get_acceleration_data function;
  modified _update_odr function;
  manages 2 interrupts;
 Revision 1.0.6 15/11/2010
  supports sysfs;
  no more support for ioctl;
 Revision 1.0.7 26/11/2010
  checks for availability of interrupts pins
  correction on FUZZ and FLAT values;
 Revision 1.0.8 2010/Apr/01
  corrects a bug in interrupt pin management in 1.0.7

 ******************************************************************************/
#include <linux/module.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/i2c.h>
#include <linux/input.h>
#include <linux/uaccess.h>
#include <linux/workqueue.h>
#include <linux/irq.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/slab.h>
#include <linux/input/lis3dh.h>
#include <linux/hrtimer.h>
#include <linux/kthread.h>

#define	DEBUG	0
#define DEBUG_DATA_LOG 0

#define	G_MAX		16000

#define LIS3DH_6D_REPORT_DELAY 67
#define LIS3DH_6D_REPORT_CNT   120

#define SENSITIVITY_2G		1	/**	mg/LSB	*/
#define SENSITIVITY_4G		2	/**	mg/LSB	*/
#define SENSITIVITY_8G		4	/**	mg/LSB	*/
#define SENSITIVITY_16G		12	/**	mg/LSB	*/

/* Accelerometer Sensor Full Scale */
#define LIS3DH_ACC_FS_MASK		0x30
#define LIS3DH_ACC_G_2G		0x00
#define LIS3DH_ACC_G_4G		0x10
#define LIS3DH_ACC_G_8G		0x20
#define LIS3DH_ACC_G_16G		0x30

/* Accelerometer Sensor Operating Mode */
#define LIS3DH_ACC_ENABLE	0x01
#define LIS3DH_ACC_DISABLE	0x00

#define	HIGH_RESOLUTION		0x08

#define	AXISDATA_REG		0x28
#define WHOAMI_LIS3DH_ACC	0x32	/*	Expected content for WAI */

/*	CONTROL REGISTERS	*/
#define WHO_AM_I		0x0F	/*	WhoAmI register		*/
#define	TEMP_CFG_REG		0x1F	/*	temper sens control reg	*/
/* ctrl 1: ODR3 ODR2 ODR ODR0 LPen Zenable Yenable Zenable */
#define	CTRL_REG1		0x20	/*	control reg 1		*/
#define	CTRL_REG2		0x21	/*	control reg 2		*/
#define	CTRL_REG3		0x22	/*	control reg 3		*/
#define	CTRL_REG4		0x23	/*	control reg 4		*/
#define	CTRL_REG5		0x24	/*	control reg 5		*/
#define	CTRL_REG6		0x25	/*	control reg 6		*/

#define	FIFO_CTRL_REG		0x2E	/*	FiFo control reg	*/

#define	INT_CFG1		0x30	/*	interrupt 1 config	*/
#define	INT_SRC1		0x31	/*	interrupt 1 source	*/
#define	INT_THS1		0x32	/*	interrupt 1 threshold	*/
#define	INT_DUR1		0x33	/*	interrupt 1 duration	*/

#define	TT_CFG			0x38	/*	tap config		*/
#define	TT_SRC			0x39	/*	tap source		*/
#define	TT_THS			0x3A	/*	tap threshold		*/
#define	TT_LIM			0x3B	/*	tap time limit		*/
#define	TT_TLAT			0x3C	/*	tap time latency	*/
#define	TT_TW			0x3D	/*	tap time window		*/
/*	end CONTROL REGISTRES	*/

#define ENABLE_HIGH_RESOLUTION	1

#define LIS3DH_ACC_PM_OFF		0x00
#define LIS3DH_ACC_ENABLE_ALL_AXES	0x07

#define PMODE_MASK			0x08
#define ODR_MASK			0XF0

#define ODR1		0x10  /* 1Hz output data rate */
#define ODR10		0x20  /* 10Hz output data rate */
#define ODR25		0x30  /* 25Hz output data rate */
#define ODR50		0x40  /* 50Hz output data rate */
#define ODR100		0x50  /* 100Hz output data rate */
#define ODR200		0x60  /* 200Hz output data rate */
#define ODR400		0x70  /* 400Hz output data rate */
#define ODR1250		0x90  /* 1250Hz output data rate */

#define	IA			0x40
#define	ZH			0x20
#define	ZL			0x10
#define	YH			0x08
#define	YL			0x04
#define	XH			0x02
#define	XL			0x01

/* CTRL REG BITS*/
#define	CTRL_REG3_I1_AOI1	0x40
#define	CTRL_REG6_I2_TAPEN	0x80
#define	CTRL_REG6_HLACTIVE	0x02

#define NO_MASK			0xFF
#define INT1_DURATION_MASK	0x7F
#define	INT1_THRESHOLD_MASK	0x7F
#define TAP_CFG_MASK		0x3F
#define	TAP_THS_MASK		0x7F
#define	TAP_TLIM_MASK		0x7F
#define	TAP_TLAT_MASK		NO_MASK
#define	TAP_TW_MASK		NO_MASK

/* TAP_SOURCE_REG BIT */
#define	DTAP			0x20
#define	STAP			0x10
#define	SIGNTAP			0x08
#define	ZTAP			0x04
#define	YTAP			0x02
#define	XTAZ			0x01

#define	FUZZ			0
#define	FLAT			0
#define	I2C_RETRY_DELAY		5
#define	I2C_RETRIES		5
#define	I2C_AUTO_INCREMENT	0x80

/* RESUME STATE INDICES */
#define	RES_CTRL_REG1		0
#define	RES_CTRL_REG2		1
#define	RES_CTRL_REG3		2
#define	RES_CTRL_REG4		3
#define	RES_CTRL_REG5		4
#define	RES_CTRL_REG6		5

#define	RES_INT_CFG1		6
#define	RES_INT_THS1		7
#define	RES_INT_DUR1		8

#define	RES_TT_CFG		9
#define	RES_TT_THS		10
#define	RES_TT_LIM		11
#define	RES_TT_TLAT		12
#define	RES_TT_TW		13

#define	RES_TEMP_CFG_REG	14
#define	RES_REFERENCE_REG	15
#define	RES_FIFO_CTRL_REG	16

#define	RESUME_ENTRIES		17
/* end RESUME STATE INDICES */

/* the device need a short dealy for stable */
#define HW_STABLE_DELAY_US	(10 * 1000)

struct {
	unsigned int cutoff_ms;
	unsigned int mask;
} lis3dh_acc_odr_table[] = {
		{    1, ODR1250 },
		{    3, ODR400  },
		{    5, ODR200  },
		{   10, ODR100  },
		{   20, ODR50   },
		{   40, ODR25   },
		{  100, ODR10   },
		{ 1000, ODR1    },
};

struct lis3dh_acc_data {
	struct input_dev *input_dev;
	struct i2c_client *client;
	struct lis3dh_acc_platform_data *pdata;

	struct mutex lock;
	struct hrtimer work_timer;
	struct completion report_complete;
	struct task_struct *thread;
	bool hrtimer_running;
	int report_cnt;
	int report_interval;

	int hw_initialized;
	/* hw_working=-1 means not tested yet */
	int hw_working;
	int enabled;
	int need_resume;

	u8 sensitivity;

	u8 resume_state[RESUME_ENTRIES];

	int irq1;
	int irq2;

#ifdef DEBUG
	u8 reg_addr;
#endif
};

static int lis3dh_acc_i2c_read(struct lis3dh_acc_data *acc,
			       u8 reg, u8 *buf, int len)
{
	if (len > 1)
		reg |= I2C_AUTO_INCREMENT;

	return i2c_smbus_read_i2c_block_data(acc->client, reg, len, buf);
}

static int lis3dh_acc_i2c_write(struct lis3dh_acc_data *acc,
				u8 reg, u8 *buf, int len)
{
	if (len > 1)
		reg |= I2C_AUTO_INCREMENT;

	return i2c_smbus_write_i2c_block_data(acc->client, reg, len, buf);
}

static int lis3dh_acc_hw_init(struct lis3dh_acc_data *acc)
{
	int err = -1;
	u8 buf[7] = { 0 };

	dev_dbg(&acc->client->dev, "hw init start\n");

	/* LSM303DLHC doesn't have WHO_AM_I register */
	if (acc->pdata->model != MODEL_LSM303DLHC) {
		err = lis3dh_acc_i2c_read(acc, WHO_AM_I, buf, 1);
		if (err < 0) {
			dev_warn(&acc->client->dev, "Error reading WHO_AM_I: "
					"is device available/working?\n");
			goto err_firstread;
		}
	}

	acc->hw_working = 1;

	buf[0] = acc->resume_state[RES_CTRL_REG1];
	err = lis3dh_acc_i2c_write(acc, CTRL_REG1, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_TEMP_CFG_REG];
	err = lis3dh_acc_i2c_write(acc, TEMP_CFG_REG, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_FIFO_CTRL_REG];
	err = lis3dh_acc_i2c_write(acc, FIFO_CTRL_REG, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_THS1];
	err = lis3dh_acc_i2c_write(acc, INT_THS1, buf, 1);
	if (err < 0)
		goto err_resume_state;
	buf[0] = acc->resume_state[RES_INT_DUR1];
	err = lis3dh_acc_i2c_write(acc, INT_DUR1, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_INT_CFG1];
	err = lis3dh_acc_i2c_write(acc, INT_CFG1, buf, 1);
	if (err < 0)
		goto err_resume_state;

	buf[0] = acc->resume_state[RES_CTRL_REG3];
	err = lis3dh_acc_i2c_write(acc, CTRL_REG3, buf, 1);
	if (err < 0)
		goto err_resume_state;

	acc->hw_initialized = 1;
	dev_dbg(&acc->client->dev, "hw init done\n");
	return 0;

err_firstread:
	acc->hw_working = 0;
err_resume_state:
	acc->hw_initialized = 0;
	dev_err(&acc->client->dev, "hw init error 0x%x,0x%x: %d\n", buf[0],
			buf[1], err);
	return err;
}

static void lis3dh_acc_device_power_off(struct lis3dh_acc_data *acc)
{
	int err;
	u8 buf;

	buf = LIS3DH_ACC_PM_OFF;
	err = lis3dh_acc_i2c_write(acc, CTRL_REG1, &buf, 1);
	if (err < 0)
		dev_err(&acc->client->dev, "soft power off failed: %d\n", err);

	if (acc->hw_initialized) {
		if (acc->pdata->power_off)
			acc->pdata->power_off();
		acc->hw_initialized = 0;
	}
}

static int lis3dh_acc_device_power_on(struct lis3dh_acc_data *acc)
{
	int err;

	if (!acc->hw_initialized) {
		err = lis3dh_acc_hw_init(acc);
		if (acc->hw_working == 1 && err < 0) {
			lis3dh_acc_device_power_off(acc);
			return err;
		}
	}

	return 0;
}

int lis3dh_acc_update_g_range(struct lis3dh_acc_data *acc, u8 new_g_range)
{
	int err = -1;
	u8 sensitivity;
	u8 buf = 0;
	u8 updated_val;
	u8 init_val;
	u8 new_val;
	u8 mask = LIS3DH_ACC_FS_MASK | HIGH_RESOLUTION;
	u8 fs_bits = 0;

	switch (new_g_range) {
	case 2:
		fs_bits = LIS3DH_ACC_G_2G;
		sensitivity = SENSITIVITY_2G;
		break;
	case 4:
		fs_bits = LIS3DH_ACC_G_4G;
		sensitivity = SENSITIVITY_4G;
		break;
	case 8:
		fs_bits = LIS3DH_ACC_G_8G;
		sensitivity = SENSITIVITY_8G;
		break;
	case 16:
		fs_bits = LIS3DH_ACC_G_16G;
		sensitivity = SENSITIVITY_16G;
		break;
	default:
		dev_err(&acc->client->dev, "invalid grange requested: %u\n",
				new_g_range);
		return -EINVAL;
	}

	/* Updates configuration register 4,
	 * which contains g range setting */
	err = lis3dh_acc_i2c_read(acc, CTRL_REG4, &buf, 1);
	if (err < 0)
		goto error;
	init_val = buf;
	acc->resume_state[RES_CTRL_REG4] = init_val;
	new_val = fs_bits | HIGH_RESOLUTION;
	updated_val = ((mask & new_val) | ((~mask) & init_val));
	buf = updated_val;
	err = lis3dh_acc_i2c_write(acc, CTRL_REG4, &buf, 1);
	if (err < 0)
		goto error;
	acc->resume_state[RES_CTRL_REG4] = updated_val;
	acc->sensitivity = sensitivity;

	return err;
error:
	dev_err(&acc->client->dev, "update grange failed 0x%x: %d\n", buf, err);
	return err;
}

int lis3dh_acc_update_odr(struct lis3dh_acc_data *acc, int poll_interval_ms)
{
	int err = -1;
	int i;
	u8 config = 0;

	/* Following, looks for the longest possible odr interval scrolling the
	 * odr_table vector from the end (shortest interval) backward (longest
	 * interval), to support the poll_interval requested by the system.
	 * It must be the longest interval lower then the poll interval.*/
	for (i = ARRAY_SIZE(lis3dh_acc_odr_table) - 1; i >= 0; i--) {
		if (lis3dh_acc_odr_table[i].cutoff_ms <= poll_interval_ms)
			break;
	}

	/* i < 0 should not happen ... */
	if (i < 0)
		goto error;

	config = lis3dh_acc_odr_table[i].mask;
	config |= LIS3DH_ACC_ENABLE_ALL_AXES;

	err = lis3dh_acc_i2c_write(acc, CTRL_REG1, &config, 1);
	if (err < 0)
		goto error;
	acc->resume_state[RES_CTRL_REG1] = config;

	return err;

error:
	dev_err(&acc->client->dev, "update odr failed 0x%x: %d\n",
		config, err);

	return err;
}

static int lis3dh_acc_get_acceleration_data(struct lis3dh_acc_data *acc,
		int *xyz)
{
	int err = -1;
	/* Data bytes from hardware xL, xH, yL, yH, zL, zH */
	u8 acc_data[6] = { 0 };
	/* x,y,z hardware data */
	s16 hw_d[3] = { 0 };
	err = lis3dh_acc_i2c_read(acc, AXISDATA_REG, acc_data, 6);
	if (err < 0)
		return err;

	hw_d[0] = (((s16) ((acc_data[1] << 8) | acc_data[0])) >> 4);
	hw_d[1] = (((s16) ((acc_data[3] << 8) | acc_data[2])) >> 4);
	hw_d[2] = (((s16) ((acc_data[5] << 8) | acc_data[4])) >> 4);

	hw_d[0] = hw_d[0] * acc->sensitivity;
	hw_d[1] = hw_d[1] * acc->sensitivity;
	hw_d[2] = hw_d[2] * acc->sensitivity;

	xyz[0] = ((acc->pdata->negate_x) ? (-hw_d[acc->pdata->axis_map_x])
		   : (hw_d[acc->pdata->axis_map_x]));
	xyz[1] = ((acc->pdata->negate_y) ? (-hw_d[acc->pdata->axis_map_y])
		   : (hw_d[acc->pdata->axis_map_y]));
	xyz[2] = ((acc->pdata->negate_z) ? (-hw_d[acc->pdata->axis_map_z])
		   : (hw_d[acc->pdata->axis_map_z]));

#if DEBUG_DATA_LOG
	printk(KERN_INFO "%s read x=%d, y=%d, z=%d\n",
		LIS3DH_ACC_DEV_NAME, xyz[0], xyz[1], xyz[2]);
#endif
	return err;
}

static void lis3dh_acc_report_values(struct lis3dh_acc_data *acc, int *xyz)
{
	input_report_rel(acc->input_dev, REL_X, xyz[0]);
	input_report_rel(acc->input_dev, REL_Y, xyz[1]);
	input_report_rel(acc->input_dev, REL_Z, xyz[2]);
	input_sync(acc->input_dev);
}

static int report_event(void *data)
{
	int xyz[3] = { 0 };
	int err;
	struct lis3dh_acc_data *acc = data;

	while(1)
	{
		/* wait for report event */
		wait_for_completion(&acc->report_complete);

		mutex_lock(&acc->lock);
		if (!acc->enabled) {
			mutex_unlock(&acc->lock);
			continue;
		}
		err = lis3dh_acc_get_acceleration_data(acc, xyz);
		if (err < 0)
			dev_err(&acc->client->dev, "get_acceleration_data failed\n");
		else
			lis3dh_acc_report_values(acc, xyz);
		mutex_unlock(&acc->lock);
	}

	return 0;
}

/* mutex lock must be held when calling this function */
static void lis3dh_acc_launch_work(struct lis3dh_acc_data *acc)
{
	ktime_t poll_delay;

	if (!acc->enabled)
		return;
	if (acc->pdata->poll_interval > 0) {
		poll_delay = ktime_set(0, acc->pdata->poll_interval * NSEC_PER_MSEC);
	} else {
		acc->report_cnt = LIS3DH_6D_REPORT_CNT;
		poll_delay = ktime_set(0, acc->report_interval * NSEC_PER_MSEC);
	}
	acc->hrtimer_running = true;
	hrtimer_start(&acc->work_timer, poll_delay, HRTIMER_MODE_REL);
}

static int lis3dh_acc_get_int1_source(struct lis3dh_acc_data *acc)
{
	u8 data = 0;
	int ret;

	ret = lis3dh_acc_i2c_read(acc, INT_SRC1, &data, 1);
	if (ret < 0) {
		printk(KERN_INFO "Error when read int1_source\n");
		return ret;
	}

	return data;
}

#define INT1_IA (1 << 6)
static irqreturn_t lis3dh_acc_isr1(int irq, void *data)
{
	struct lis3dh_acc_data *acc = data;
	struct device *dev = &acc->client->dev;
	int int1_stat;
	int irq_ret = IRQ_HANDLED;

	mutex_lock(&acc->lock);

	if (!acc->enabled)
		goto out;

	int1_stat = lis3dh_acc_get_int1_source(acc);
	if (int1_stat < 0) {
		irq_ret = IRQ_NONE;
		goto out;
	}

	if (int1_stat & INT1_IA) {
		dev_dbg(dev, "6d interrupt reported\n");
		lis3dh_acc_launch_work(acc);
	}

out:
	mutex_unlock(&acc->lock);
	return irq_ret;
}

static int lis3dh_acc_enable(struct lis3dh_acc_data *acc)
{
	/* clear interrupt source */
	lis3dh_acc_get_int1_source(acc);
	lis3dh_acc_device_power_on(acc);
	usleep_range(HW_STABLE_DELAY_US, HW_STABLE_DELAY_US + 1000);
	acc->enabled = 1;

	/* Send initial events to userspace */
	lis3dh_acc_launch_work(acc);
	return 0;
}

static int lis3dh_acc_disable(struct lis3dh_acc_data *acc)
{
	lis3dh_acc_device_power_off(acc);
	acc->enabled = 0;
	return 0;
}


static ssize_t read_single_reg(struct device *dev, char *buf, u8 reg)
{
	ssize_t ret;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int err;
	u8 data = 0;

	err = lis3dh_acc_i2c_read(acc, reg, &data, 1);
	if (err < 0)
		return err;
	ret = sprintf(buf, "0x%02x\n", data);
	return ret;
}

static int write_reg(struct device *dev, const char *buf, u8 reg,
		u8 mask, int resumeIndex)
{
	int err;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	u8 new_val;
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	new_val = ((u8) val & mask);
	err = lis3dh_acc_i2c_write(acc, reg, &new_val, 1);
	if (err < 0)
		return err;
	acc->resume_state[resumeIndex] = new_val;
	return err;
}

static ssize_t attr_get_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     char *buf)
{
	int val;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);

	mutex_lock(&acc->lock);
	val = acc->pdata->poll_interval;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_polling_rate(struct device *dev,
				     struct device_attribute *attr,
				     const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long interval_ms;

	if (strict_strtoul(buf, 10, &interval_ms))
		return -EINVAL;

	if (!interval_ms)
		hrtimer_cancel(&acc->work_timer);

	mutex_lock(&acc->lock);
	acc->pdata->poll_interval = interval_ms;

	/* Send initial events to userspace */
	if (acc->enabled)
		lis3dh_acc_launch_work(acc);
	mutex_unlock(&acc->lock);
	return size;
}
static DEVICE_ATTR(poll, S_IRUGO | S_IWUSR,
		   attr_get_polling_rate, attr_set_polling_rate);

static ssize_t attr_get_range(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	char range;

	mutex_lock(&acc->lock);
	range = acc->pdata->g_range ;
	mutex_unlock(&acc->lock);
	return sprintf(buf, "%d\n", range);
}

static ssize_t attr_set_range(struct device *dev,
			      struct device_attribute *attr,
			      const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val != 2 && val != 4 && val != 8 && val != 16)
		return -EINVAL;


	mutex_lock(&acc->lock);
	acc->pdata->g_range = (u8) val;
	lis3dh_acc_update_g_range(acc, acc->pdata->g_range);
	mutex_unlock(&acc->lock);

	return size;
}
static DEVICE_ATTR(range, S_IRUGO | S_IWUSR, attr_get_range, attr_set_range);

static ssize_t attr_get_enable(struct device *dev,
			       struct device_attribute *attr, char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int val;

	mutex_lock(&acc->lock);
	val = acc->enabled;
	mutex_unlock(&acc->lock);

	return sprintf(buf, "%d\n", val);
}

static ssize_t attr_set_enable(struct device *dev,
			       struct device_attribute *attr,
			       const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 10, &val))
		return -EINVAL;

	if (val != 0 && val != 1)
		return -EINVAL;

	dev_dbg(dev, "enable %ld\n", val);

	mutex_lock(&acc->lock);
	if (val)
		lis3dh_acc_enable(acc);
	else
		lis3dh_acc_disable(acc);
	mutex_unlock(&acc->lock);

	return size;
}
static DEVICE_ATTR(enable, S_IRUGO | S_IWUSR, attr_get_enable, attr_set_enable);

static ssize_t attr_set_intconfig1(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_CFG1, NO_MASK, RES_INT_CFG1);
}

static ssize_t attr_get_intconfig1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_CFG1);
}
static DEVICE_ATTR(int1_config, S_IRUGO | S_IWUSR,
		   attr_get_intconfig1, attr_set_intconfig1);

static ssize_t attr_set_duration1(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_DUR1, INT1_DURATION_MASK, RES_INT_DUR1);
}

static ssize_t attr_get_duration1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_DUR1);
}
static DEVICE_ATTR(int1_duration, S_IRUGO | S_IWUSR,
		   attr_get_duration1, attr_set_duration1);

static ssize_t attr_set_thresh1(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, INT_THS1, INT1_THRESHOLD_MASK, RES_INT_THS1);
}

static ssize_t attr_get_thresh1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_THS1);
}
static DEVICE_ATTR(int1_threshold, S_IRUGO | S_IWUSR,
		   attr_get_thresh1, attr_set_thresh1);

static ssize_t attr_get_source1(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, INT_SRC1);
}
static DEVICE_ATTR(int1_source, S_IRUGO, attr_get_source1, NULL);

static ssize_t attr_set_click_cfg(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_CFG, TAP_CFG_MASK, RES_TT_CFG);
}

static ssize_t attr_get_click_cfg(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_CFG);
}
static DEVICE_ATTR(click_config, S_IRUGO | S_IWUSR,
		   attr_get_click_cfg, attr_set_click_cfg);

static ssize_t attr_get_click_source(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_SRC);
}
static DEVICE_ATTR(click_source, S_IRUGO, attr_get_click_source, NULL);

static ssize_t attr_set_click_ths(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_THS, TAP_THS_MASK, RES_TT_THS);
}

static ssize_t attr_get_click_ths(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_THS);
}
static DEVICE_ATTR(click_threshold, S_IRUGO | S_IWUSR,
		   attr_get_click_ths, attr_set_click_ths);

static ssize_t attr_set_click_tlim(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_LIM, TAP_TLIM_MASK, RES_TT_LIM);
}

static ssize_t attr_get_click_tlim(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_LIM);
}
static DEVICE_ATTR(click_timelimit, S_IRUGO | S_IWUSR,
		   attr_get_click_tlim, attr_set_click_tlim);

static ssize_t attr_set_click_tlat(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TLAT_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tlat(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}
static DEVICE_ATTR(click_timelatency, S_IRUGO | S_IWUSR,
		   attr_get_click_tlat, attr_set_click_tlat);

static ssize_t attr_set_click_tw(struct device *dev,
		struct device_attribute *attr, const char *buf, size_t size)
{
	return write_reg(dev, buf, TT_TLAT, TAP_TW_MASK, RES_TT_TLAT);
}

static ssize_t attr_get_click_tw(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	return read_single_reg(dev, buf, TT_TLAT);
}
static DEVICE_ATTR(click_timewindow, S_IRUGO | S_IWUSR,
		   attr_get_click_tw, attr_set_click_tw);


#ifdef DEBUG
/* PAY ATTENTION: These DEBUG funtions don't manage resume_state */
static ssize_t attr_reg_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	int rc;
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	rc = lis3dh_acc_i2c_write(acc, acc->reg_addr, (u8 *)&val, 1);
	mutex_unlock(&acc->lock);

	if (rc == 0)
		rc = size;
	return rc;
}

static ssize_t attr_reg_get(struct device *dev, struct device_attribute *attr,
				char *buf)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	int rc;
	u8 data = 0;

	mutex_lock(&acc->lock);
	rc = lis3dh_acc_i2c_read(acc, acc->reg_addr, &data, 1);
	mutex_unlock(&acc->lock);

	if (rc < 0)
		return rc;
	else
		return sprintf(buf, "0x%02x\n", data);
}
static DEVICE_ATTR(reg_value, S_IRUGO | S_IWUSR, attr_reg_get, attr_reg_set);

static ssize_t attr_addr_set(struct device *dev, struct device_attribute *attr,
				const char *buf, size_t size)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);
	unsigned long val;

	if (strict_strtoul(buf, 16, &val))
		return -EINVAL;

	mutex_lock(&acc->lock);
	acc->reg_addr = val;
	mutex_unlock(&acc->lock);

	return size;
}
static DEVICE_ATTR(reg_addr, S_IWUSR, NULL, attr_addr_set);
#endif

static struct attribute *lis3dh_attrs[] = {
	&dev_attr_poll.attr,
	&dev_attr_range.attr,
	&dev_attr_enable.attr,
	&dev_attr_int1_config.attr,
	&dev_attr_int1_duration.attr,
	&dev_attr_int1_threshold.attr,
	&dev_attr_int1_source.attr,
	&dev_attr_click_config.attr,
	&dev_attr_click_source.attr,
	&dev_attr_click_threshold.attr,
	&dev_attr_click_timelimit.attr,
	&dev_attr_click_timelatency.attr,
	&dev_attr_click_timewindow.attr,
#ifdef DEBUG
	&dev_attr_reg_value.attr,
	&dev_attr_reg_addr.attr,
#endif
	NULL
};

static struct attribute_group lis3dh_attr_group = {
	.name = "lis3dh",
	.attrs = lis3dh_attrs
};

static enum hrtimer_restart lis3dh_acc_input_work_func(struct hrtimer *timer)
{
	struct lis3dh_acc_data *acc;
	ktime_t poll_delay;
	acc = container_of((struct hrtimer *)timer,
			struct lis3dh_acc_data, work_timer);

	if (!acc->enabled)
		return HRTIMER_NORESTART;

	complete(&acc->report_complete);

	if (acc->pdata->poll_interval > 0) {
		poll_delay = ktime_set(0, acc->pdata->poll_interval * NSEC_PER_MSEC);
	} else if (acc->report_cnt-- > 0) {
		poll_delay = ktime_set(0, acc->report_interval * NSEC_PER_MSEC);
	}
	hrtimer_start(&acc->work_timer, poll_delay, HRTIMER_MODE_REL);

	return HRTIMER_NORESTART;
}

static int lis3dh_acc_validate_pdata(struct lis3dh_acc_data *acc)
{
	acc->pdata->poll_interval = max(acc->pdata->poll_interval,
			acc->pdata->min_interval);

	if (acc->pdata->axis_map_x > 2 ||
	    acc->pdata->axis_map_y > 2 ||
	    acc->pdata->axis_map_z > 2) {
		dev_err(&acc->client->dev, "invalid axis_map value "
			"x:%u y:%u z%u\n", acc->pdata->axis_map_x,
				acc->pdata->axis_map_y, acc->pdata->axis_map_z);
		return -EINVAL;
	}

	/* Only allow 0 and 1 for negation boolean flag */
	if (acc->pdata->negate_x > 1 || acc->pdata->negate_y > 1
			|| acc->pdata->negate_z > 1) {
		dev_err(&acc->client->dev, "invalid negate value "
			"x:%u y:%u z:%u\n", acc->pdata->negate_x,
				acc->pdata->negate_y, acc->pdata->negate_z);
		return -EINVAL;
	}

	/* Enforce minimum polling interval */
	if (acc->pdata->poll_interval < acc->pdata->min_interval) {
		dev_err(&acc->client->dev, "minimum poll interval violated\n");
		return -EINVAL;
	}

	return 0;
}

static int lis3dh_acc_input_init(struct lis3dh_acc_data *acc)
{
	int err;

	hrtimer_init(&acc->work_timer, CLOCK_MONOTONIC,HRTIMER_MODE_REL);
	acc->work_timer.function = lis3dh_acc_input_work_func;
	acc->hrtimer_running = false;
	acc->report_interval = LIS3DH_6D_REPORT_DELAY;

	acc->input_dev = input_allocate_device();
	if (!acc->input_dev) {
		err = -ENOMEM;
		dev_err(&acc->client->dev, "input device allocation failed\n");
		goto err0;
	}
	init_completion(&acc->report_complete);
	acc->thread = kthread_run(report_event, acc, "acc_report_event");
	if (IS_ERR(acc->thread)) {
		err = -EINVAL;
		dev_err(&acc->client->dev,
				"unable to create report_event thread\n");
		goto err0;
	}

	acc->input_dev->name = LIS3DH_ACC_DEV_NAME;
	acc->input_dev->id.bustype = BUS_I2C;
	acc->input_dev->dev.parent = &acc->client->dev;

	input_set_drvdata(acc->input_dev, acc);

	set_bit(EV_REL, acc->input_dev->evbit);
	set_bit(REL_X, acc->input_dev->relbit);
	set_bit(REL_Y, acc->input_dev->relbit);
	set_bit(REL_Z, acc->input_dev->relbit);
	/*	next is used for interruptA sources data if the case */
	set_bit(ABS_MISC, acc->input_dev->absbit);
	/*	next is used for interruptB sources data if the case */
	set_bit(ABS_WHEEL, acc->input_dev->absbit);

	/*	next is used for interruptA sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_MISC, INT_MIN, INT_MAX, 0, 0);
	/*	next is used for interruptB sources data if the case */
	input_set_abs_params(acc->input_dev, ABS_WHEEL, INT_MIN, INT_MAX, 0, 0);

	err = input_register_device(acc->input_dev);
	if (err) {
		dev_err(&acc->client->dev,
				"unable to register input device %s\n",
				acc->input_dev->name);
		goto err1;
	}

	err = sysfs_create_group(&acc->client->dev.kobj, &lis3dh_attr_group);
	if (err < 0) {
		dev_err(&acc->client->dev,
		   "device LIS3DH_ACC_DEV_NAME sysfs register failed\n");
		goto err0;
	}

	return 0;

err1:
	input_free_device(acc->input_dev);
	kthread_stop(acc->thread);
err0:
	return err;
}

static void lis3dh_init_resume_state(struct lis3dh_acc_data *acc)
{
	memset(acc->resume_state, 0, ARRAY_SIZE(acc->resume_state));
	acc->resume_state[RES_CTRL_REG1] = LIS3DH_ACC_ENABLE_ALL_AXES;
	acc->resume_state[RES_CTRL_REG2] = 0x00;
	acc->resume_state[RES_CTRL_REG3] = 0x40;
	acc->resume_state[RES_CTRL_REG4] = 0x00;
	acc->resume_state[RES_CTRL_REG5] = 0x08;
	acc->resume_state[RES_CTRL_REG6] = 0x00;

	acc->resume_state[RES_TEMP_CFG_REG] = 0x00;
	acc->resume_state[RES_FIFO_CTRL_REG] = 0x00;
	acc->resume_state[RES_INT_CFG1] = 0x7f;
	acc->resume_state[RES_INT_THS1] = 0x30;
	acc->resume_state[RES_INT_DUR1] = 5;

	acc->resume_state[RES_TT_CFG] = 0x00;
	acc->resume_state[RES_TT_THS] = 0x00;
	acc->resume_state[RES_TT_LIM] = 0x00;
	acc->resume_state[RES_TT_TLAT] = 0x00;
	acc->resume_state[RES_TT_TW] = 0x00;
}

static int lis3dh_acc_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct lis3dh_acc_data *acc;
	int err;

	if (!client->dev.platform_data) {
		dev_err(&client->dev, "platform data is NULL. exiting.\n");
		err = -ENODEV;
		goto out;
	}

	if (!i2c_check_functionality(client->adapter,
				I2C_FUNC_I2C | I2C_FUNC_SMBUS_I2C_BLOCK)) {
		dev_err(&client->dev, "client not i2c capable\n");
		err = -ENODEV;
		goto out;
	}

	acc = kzalloc(sizeof(struct lis3dh_acc_data), GFP_KERNEL);
	if (!acc) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate memory\n");
		goto out;
	}

	mutex_init(&acc->lock);
	mutex_lock(&acc->lock);

	acc->client = client;
	i2c_set_clientdata(client, acc);

	acc->pdata = kmalloc(sizeof(*acc->pdata), GFP_KERNEL);
	if (!acc->pdata) {
		err = -ENOMEM;
		dev_err(&client->dev, "failed to allocate memory for pdata\n");
		goto out_unlock;
	}

	memcpy(acc->pdata, client->dev.platform_data, sizeof(*acc->pdata));
	err = lis3dh_acc_validate_pdata(acc);
	if (err < 0) {
		dev_err(&client->dev, "failed to validate platform data\n");
		goto out_free_pdata;
	}

	if (acc->pdata->init) {
		err = acc->pdata->init();
		if (err < 0) {
			dev_err(&client->dev, "init failed: %d\n", err);
			goto out_free_pdata;
		}
	}

	lis3dh_init_resume_state(acc);
	err = lis3dh_acc_device_power_on(acc);
	if (err < 0) {
		dev_err(&client->dev, "power on failed: %d\n", err);
		goto out_pdata_exit;
	}

	err = lis3dh_acc_update_g_range(acc, acc->pdata->g_range);
	if (err < 0) {
		dev_err(&client->dev, "update_g_range failed\n");
		goto  out_power_off;
	}

	err = lis3dh_acc_update_odr(acc, 3);
	if (err < 0) {
		dev_err(&client->dev, "update_odr failed\n");
		goto  out_power_off;
	}

	err = lis3dh_acc_input_init(acc);
	if (err < 0) {
		dev_err(&client->dev, "input init failed\n");
		goto out_power_off;
	}

	if (acc->pdata->gpio_int1 >= 0) {
		gpio_request(acc->pdata->gpio_int1, "accel_int1");
		gpio_direction_input(acc->pdata->gpio_int1);
		acc->irq1 = gpio_to_irq(acc->pdata->gpio_int1);
		dev_dbg(&client->dev, "irq1:%d mapped on gpio:%d\n",
			 acc->irq1, acc->pdata->gpio_int1);

		err = request_threaded_irq(acc->irq1, NULL, lis3dh_acc_isr1,
				IRQF_TRIGGER_RISING | IRQF_ONESHOT, "lis3dh_acc_irq1", acc);
		if (err < 0) {
			dev_err(&client->dev, "request irq1 failed: %d\n", err);
			goto out_free_input;
		}
	}

	lis3dh_acc_disable(acc);

	mutex_unlock(&acc->lock);
	dev_info(&client->dev, "successfully probed\n");

	return 0;

out_free_input:
	sysfs_remove_group(&client->dev.kobj, &lis3dh_attr_group);
	input_unregister_device(acc->input_dev);
	kthread_stop(acc->thread);
out_power_off:
	lis3dh_acc_device_power_off(acc);
out_pdata_exit:
	if (acc->pdata->exit)
		acc->pdata->exit();
out_free_pdata:
	kfree(acc->pdata);
out_unlock:
	mutex_unlock(&acc->lock);
	kfree(acc);
out:
	return err;
}

static int lis3dh_acc_remove(struct i2c_client *client)
{
	struct lis3dh_acc_data *acc = i2c_get_clientdata(client);

	if (acc->pdata->gpio_int1 >= 0) {
		free_irq(acc->irq1, acc);
		gpio_free(acc->pdata->gpio_int1);
	}
	kthread_stop(acc->thread);
	input_unregister_device(acc->input_dev);
	lis3dh_acc_device_power_off(acc);

	sysfs_remove_group(&client->dev.kobj, &lis3dh_attr_group);

	if (acc->pdata->exit)
		acc->pdata->exit();
	kfree(acc->pdata);
	kfree(acc);

	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int lis3dh_acc_resume(struct device *dev)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);

	dev_dbg(&acc->client->dev, "enter resume\n");
	enable_irq(acc->irq1);

	mutex_lock(&acc->lock);
	if (acc->need_resume)
		lis3dh_acc_enable(acc);
	mutex_unlock(&acc->lock);

	return 0;
}

static int lis3dh_acc_suspend(struct device *dev)
{
	struct lis3dh_acc_data *acc = dev_get_drvdata(dev);

	dev_dbg(&acc->client->dev, "enter suspend\n");
	disable_irq(acc->irq1);

	mutex_lock(&acc->lock);
	acc->need_resume = acc->enabled;
	if (acc->enabled)
		lis3dh_acc_disable(acc);
	if (acc->hrtimer_running) {
		acc->hrtimer_running = false;
		hrtimer_cancel(&acc->work_timer);
	}
	mutex_unlock(&acc->lock);

	return 0;
}

static const struct dev_pm_ops lis3dh_acc_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(lis3dh_acc_suspend,
			lis3dh_acc_resume)
};
#endif /* CONFIG_PM_SLEEP */

static const struct i2c_device_id lis3dh_acc_id[]
		= { { LIS3DH_ACC_DEV_NAME, 0 }, { "lsm303dl", 0 }, { }, };

MODULE_DEVICE_TABLE(i2c, lis3dh_acc_id);

static struct i2c_driver lis3dh_acc_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = LIS3DH_ACC_DEV_NAME,
#ifdef CONFIG_PM_SLEEP
		.pm = &lis3dh_acc_pm_ops,
#endif /* CONFIG_PM_SLEEP */
	},
	.probe = lis3dh_acc_probe,
	.remove = lis3dh_acc_remove,
	.id_table = lis3dh_acc_id,
};

static int __init lis3dh_acc_init(void)
{
	pr_info("lis3dh driver: init\n");
	return i2c_add_driver(&lis3dh_acc_driver);
}

static void __exit lis3dh_acc_exit(void)
{
	pr_info("lis3dh driver exit\n");
	i2c_del_driver(&lis3dh_acc_driver);
}

module_init(lis3dh_acc_init);
module_exit(lis3dh_acc_exit);

MODULE_DESCRIPTION("lis3dh digital accelerometer sysfs driver");
MODULE_AUTHOR("Matteo Dameno, Carmine Iascone, STMicroelectronics");
MODULE_LICENSE("GPL");
