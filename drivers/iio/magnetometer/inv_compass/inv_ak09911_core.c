/*
* Copyright (C) 2013 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*/
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include <linux/seq_file.h>
#include <linux/pinctrl/consumer.h>


#include "inv_ak09911_iio.h"
#include "../../imu/inv_mpu6515/inv_test/inv_counters.h"

//<ASUS-annacheng20150129>>>>>>>>>>>>
#include <linux/proc_fs.h>
#define Driverversion  "1.0.0"
#define VENDOR  "AK09911"
struct proc_dir_entry *compasssensor_entry=NULL;
//<ASUS-annacheng20150129><<<<<<<<<<<<+

//<asus-danielchan20150415>>>>>>>>>+
#include <linux/kernel.h>
//<ASUS-danielchan20150415><<<<<<<<+

#include <linux/regulator/consumer.h>

//<asus_zx20150328>+>>

bool akm_enable_status =false;
bool because_of_suspend = false;

static int inv_ak09911_pm_suspend(struct device *dev);
static int inv_ak09911_pm_resume(struct device *dev);



static struct mpu_platform_data ak09911_compass_data = {
        .orientation = {   1,  0,  0,
                           0,  1,  0,
                           0,  0,  1 },
};

//<asus-danielchan20150415>>>>>>>>>+
static struct mpu_platform_data ZD550KL_ak09911_compass_data = {
        .orientation = {   1,  0,  0,
                           0, -1,  0,
                           0,  0, -1 },
};
//<ASUS-danielchan20150415><<<<<<<<+


/*
static struct i2c_board_info __initdata compass_board_info[] = {
        {
                I2C_BOARD_INFO("ak09911", 0xc),
                .platform_data = &ak09911_compass_data,
        },
};

*/
//<asus_zx20150328>+<<
static s64 get_time_ns(void)
{
	struct timespec ts;
    #if 0
         ktime_get_ts(&ts);
    #else
        get_monotonic_boottime(&ts);
    #endif
	return timespec_to_ns(&ts);
}

/**
 *  inv_serial_read() - Read one or more bytes from the device registers.
 *  @st:     Device driver instance.
 *  @reg:    First device register to be read from.
 *  @length: Number of bytes to read.
 *  @data:   Data read from device.
 *  NOTE:    The slave register will not increment when reading from the FIFO.
 */
int inv_serial_read(struct inv_ak09911_state_s *st, u8 reg, u16 length, u8 *data)
{
	int result;
	INV_I2C_INC_COMPASSWRITE(3);
	INV_I2C_INC_COMPASSREAD(length);
	result = i2c_smbus_read_i2c_block_data(st->i2c, reg, length, data);
	if (result != length) {
		if (result < 0)
			return result;
		else
			return -EINVAL;
	} else {
		return 0;
	}
}

/**
 *  inv_serial_single_write() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
 */
int inv_serial_single_write(struct inv_ak09911_state_s *st, u8 reg, u8 data)
{
	u8 d[1];
	d[0] = data;
	INV_I2C_INC_COMPASSWRITE(3);

	return i2c_smbus_write_i2c_block_data(st->i2c, reg, 1, d);
}

static int ak09911_init(struct inv_ak09911_state_s *st)
{
	int result = 0;
	unsigned char serial_data[3];

	result = inv_serial_single_write(st, AK09911_REG_CNTL,
					 AK09911_CNTL_MODE_POWER_DOWN);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}
	/* Wait at least 100us */
	udelay(100);

	result = inv_serial_single_write(st, AK09911_REG_CNTL,
					 AK09911_CNTL_MODE_FUSE_ACCESS);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}

	/* Wait at least 200us */
	udelay(200);

	result = inv_serial_read(st, AK09911_FUSE_ASAX, 3, serial_data);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}

	st->asa[0] = serial_data[0];
	st->asa[1] = serial_data[1];
	st->asa[2] = serial_data[2];

	result = inv_serial_single_write(st, AK09911_REG_CNTL,
					 AK09911_CNTL_MODE_POWER_DOWN);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}
	udelay(100);

	return result;
}

int ak09911_read(struct inv_ak09911_state_s *st, short rawfixed[3])
{
	unsigned char regs[9];
	unsigned char *stat = &regs[0];
	unsigned char *stat2 = &regs[8];
	int result = 0;
	int status = 0;

	result = inv_serial_read(st, AK09911_REG_ST1, 9, regs);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
	return result;
	}

	rawfixed[0] = (short)((regs[2]<<8) | regs[1]);
	rawfixed[1] = (short)((regs[4]<<8) | regs[3]);
	rawfixed[2] = (short)((regs[6]<<8) | regs[5]);

	/*
	 * ST : data ready -
	 * Measurement has been completed and data is ready to be read.
	 */
	if (*stat & 0x01)
		status = 0;
	/*
	 * ST2 : overflow -
	 * the sum of the absolute values of all axis |X|+|Y|+|Z| < 2400uT.
	 * This is likely to happen in presence of an external magnetic
	 * disturbance; it indicates, the sensor data is incorrect and should
	 * be ignored.
	 * An error is returned.
	 * HOFL bit clears when a new measurement starts.
	 */
	if (*stat2 & 0x08)
		status = 0x08;
	/*
	 * ST : overrun -
	 * the previous sample was not fetched and lost.
	 * Valid in continuous measurement mode only.
	 * In single measurement mode this error should not occour and we
	 * don't consider this condition an error.
	 * DOR bit is self-clearing when ST2 or any meas. data register is
	 * read.
	 */
	if (*stat & 0x02) {
		/* status = INV_ERROR_COMPASS_DATA_UNDERFLOW; */
		status = 0;
	}

	/*
	 * trigger next measurement if:
	 *	- stat is non zero;
	 *	- if stat is zero and stat2 is non zero.
	 * Won't trigger if data is not ready and there was no error.
	 */
	result = inv_serial_single_write(st, AK09911_REG_CNTL,
				AK09911_CNTL_MODE_SNG_MEASURE);
	if (result) {
		pr_err("%s, line=%d\n", __func__, __LINE__);
		return result;
	}

	if (status)
		pr_err("%s, line=%d, status=%d\n", __func__, __LINE__, status);

#if 0
	return status;
#else
    return 0;
#endif
}

/**
 *  ak09911_read_raw() - read raw method.
 */
static int ak09911_read_raw(struct iio_dev *indio_dev,
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct inv_ak09911_state_s  *st = iio_priv(indio_dev);
	int scale = 0;
printk("wlin inv_ak9911 read raw mask:%ld\n",mask);
	switch (mask) {
	case 0:
		if (!(iio_buffer_enabled(indio_dev)))
			return -EINVAL;
		if (chan->type == IIO_MAGN) {
printk("wlin inv_ak9911 read IIO_MAGN\n");
			*val = st->compass_data[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}

		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
printk("wlin inv_ak9911 read raw info scale\n");
		scale = 19661;
		scale *= (1L << 15);
		*val = scale;
			return IIO_VAL_INT;
		return -EINVAL;
	default:
		return -EINVAL;
	}
}

static ssize_t ak09911_value_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak09911_state_s *st = iio_priv(indio_dev);
	short c[3];

	mutex_lock(&indio_dev->mlock);
	c[0] = st->compass_data[0];
	c[1] = st->compass_data[1];
	c[2] = st->compass_data[2];
	mutex_unlock(&indio_dev->mlock);
	return sprintf(buf, "%d, %d, %d\n", c[0], c[1], c[2]);
}

static ssize_t ak09911_rate_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak09911_state_s *st = iio_priv(indio_dev);
	/* transform delay in ms to rate */
	return sprintf(buf, "%d\n", (1000 / st->delay));
}

/**
 * ak09911_matrix_show() - show orientation matrix
 */
static ssize_t ak09911_matrix_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_ak09911_state_s *st = iio_priv(indio_dev);
	m = st->plat_data.orientation;
	return sprintf(buf,
		"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

void set_ak09911_enable(struct iio_dev *indio_dev, bool enable)
{
	struct inv_ak09911_state_s *st = iio_priv(indio_dev);
	int result = 0;

	akm_enable_status = enable;

	printk("[SENSOR] set_ak09911_enable\n");
	if (enable) {
			result = inv_serial_single_write(st, AK09911_REG_CNTL,
						AK09911_CNTL_MODE_SNG_MEASURE);
			if (result)
				pr_err("%s, line=%d\n", __func__, __LINE__);
			schedule_delayed_work(&st->work,
				msecs_to_jiffies(st->delay));
	} else {
			cancel_delayed_work_sync(&st->work);
			result = inv_serial_single_write(st, AK09911_REG_CNTL,
						AK09911_CNTL_MODE_POWER_DOWN);
			if (result)
				pr_err("%s, line=%d\n", __func__, __LINE__);
			mdelay(1);	/* wait at least 100us */
	}
}

static ssize_t ak09911_rate_store(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	unsigned long data;
	int error;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak09911_state_s *st = iio_priv(indio_dev);

	error = kstrtoul(buf, 10, &data);
	if (error)
		return error;
	/* transform rate to delay in ms */
	data = 1000 / data;
	if (data > AK09911_MAX_DELAY)
		data = AK09911_MAX_DELAY;
	if (data < AK09911_MIN_DELAY)
		data = AK09911_MIN_DELAY;
	st->delay = (unsigned int) data;
	return count;
}

/**
 * ak09911_self_test_show() - show self_test result
 */
#define ST_COMPASS_RETRY_TIMES	10
#define ST_COMPASS_WAIT_MIN     (10 * 1000)
#define ST_COMPASS_WAIT_MAX     (15 * 1000)
static const short AK09911_ST_Lower[3] = {-30, -30, -400};
static const short AK09911_ST_Upper[3] = {30, 30, -50};

static ssize_t ak09911_self_test_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_ak09911_state_s *st = iio_priv(indio_dev);
	unsigned char sens[3],counter;
	short x,y,z;
	int result;
	unsigned char data[6];
printk("ak8911 selftest start\n");
	/* set to power down mode */
	result = inv_serial_single_write(st,AK09911_REG_CNTL, AK09911_CNTL_MODE_POWER_DOWN);
	if (result)
			goto AKM_fail;
	result = inv_serial_read(st, AK09911_FUSE_ASAX, 3, sens);

        /* set self test mode */
      result = inv_serial_single_write(st,AK09911_REG_CNTL, AK09911_CNTL_MODE_SELF_TEST);
      if (result)
          goto AKM_fail;
      counter = ST_COMPASS_RETRY_TIMES;
        while (counter > 0) {
            usleep_range(ST_COMPASS_WAIT_MIN, ST_COMPASS_WAIT_MAX);
            result = inv_serial_read(st,AK09911_REG_ST1, 1, data);
            if (result)
                goto AKM_fail;
            if ((data[0] & AK09911_DATA_DRDY) == 0)
                counter--;
            else
                counter = 0;
        }
        if ((data[0] & AK09911_DATA_DRDY) == 0) {
            result = -EINVAL;
            goto AKM_fail;
        }
        result = inv_serial_read(st,AK09911_REG_HXL,
                        6, data);
        if (result)
            goto AKM_fail;

	x = le16_to_cpup((__le16 *)(&data[0]));
	y = le16_to_cpup((__le16 *)(&data[2]));
	z = le16_to_cpup((__le16 *)(&data[4]));
	x = ((x * (sens[0] + 128)) >> 7);
	y = ((y * (sens[1] + 128)) >> 7);
	z = ((z * (sens[2] + 128)) >> 7);


	result = -EINVAL;
	if (x > AK09911_ST_Upper[0] || x < AK09911_ST_Lower[0])
		goto AKM_fail;
	if (y > AK09911_ST_Upper[1] || y < AK09911_ST_Lower[1])
		goto AKM_fail;
	if (z > AK09911_ST_Upper[2] || z < AK09911_ST_Lower[2])
		goto AKM_fail;
	result = 1;
AKM_fail:
	/* set to power down mode */
	inv_serial_single_write(st,AK09911_REG_CNTL, AK09911_CNTL_MODE_POWER_DOWN);
printk("ak8911 selftest end result:%d\n",result);

		return sprintf(buf, "%d\n", result);
}

static void ak09911_work_func(struct work_struct *work)
{
	struct inv_ak09911_state_s *st =
		container_of((struct delayed_work *)work,
			struct inv_ak09911_state_s, work);
	struct iio_dev *indio_dev = iio_priv_to_dev(st);
	unsigned long delay = msecs_to_jiffies(st->delay);

//	mutex_lock(&indio_dev->mlock);
//	if (!(iio_buffer_enabled(indio_dev)))
//		goto error_ret;

	st->timestamp = get_time_ns();
	schedule_delayed_work(&st->work, delay);
	inv_read_ak09911_fifo(indio_dev);
	INV_I2C_INC_COMPASSIRQ();

//error_ret:
//	mutex_unlock(&indio_dev->mlock);
}

static const struct iio_chan_spec compass_channels[] = {
	{
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		//.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = INV_AK09911_SCAN_MAGN_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		//.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = INV_AK09911_SCAN_MAGN_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		//.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.info_mask_shared_by_type = BIT(IIO_CHAN_INFO_SCALE),
		.scan_index = INV_AK09911_SCAN_MAGN_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_AK09911_SCAN_TIMESTAMP)
};

static DEVICE_ATTR(value, S_IRUGO, ak09911_value_show, NULL);
static DEVICE_ATTR(sampling_frequency, S_IRUGO | S_IWUSR, ak09911_rate_show,
						ak09911_rate_store);
static DEVICE_ATTR(compass_matrix, S_IRUGO, ak09911_matrix_show, NULL);
static DEVICE_ATTR(self_test, S_IRUGO, ak09911_self_test_show, NULL);

static struct attribute *inv_ak09911_attributes[] = {
	&dev_attr_value.attr,
	&dev_attr_sampling_frequency.attr,
	&dev_attr_compass_matrix.attr,
	&dev_attr_self_test.attr,
	NULL,
};

static const struct attribute_group inv_attribute_group = {
	.name = "ak09911",
	.attrs = inv_ak09911_attributes
};

static const struct iio_info ak09911_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &ak09911_read_raw,
	.attrs = &inv_attribute_group,
};



//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>>>>>>+
struct inv_ak09911_state_s *inv_ak09911;
static int Compasssensor_proc_show(struct seq_file *m, void *v) {

		int ret = 0,err=0;
		uint8_t buffer[2];
		uint8_t idReg[2];
		struct inv_ak09911_state_s  *st =inv_ak09911;

		 err = inv_serial_read(st,0x00, 2, buffer);
		idReg[0]=buffer[0];
		idReg[1]=buffer[1];
		if (err < 0)
			printk(KERN_INFO "[AKM] i2c err\n");

		printk("anna-sensor idReg : 0x00=0x%x \n", buffer[0]);

		if(err<0){
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
		if((idReg[0]==0x48)&&(idReg[1]==0X05)){//vendorid  ,deviceid
			ret =seq_printf(m," Vendor:%s(0x0%x%x)\n", VENDOR,idReg[1],idReg[0]);
		}else{
			ret =seq_printf(m," Vendor:%s(0x0%x%x)\n", VENDOR,idReg[1],idReg[0]);
		}

		//buffer[0]=0x00;
		//err = akm_i2c_rxdata(akm->i2c, buffer, 1);
		err = inv_serial_read(st,0x00, 1, buffer);
		printk("anna-sensor idReg33 : 0x00=0x%x \n", buffer[0]);
		if(err<0){
			ret =seq_printf(m," Device status:error\n");
		}else{
			ret =seq_printf(m," Device status:ok\n");
		}

	return ret;

}


static int Compasssensor_proc_open(struct inode *inode, struct  file *file) {
  return single_open(file, Compasssensor_proc_show, NULL);
}

static const struct file_operations compasssensor_proc_fops = {
  .owner = THIS_MODULE,
  .open = Compasssensor_proc_open,
  .read = seq_read,
  .llseek = seq_lseek,
  .release = single_release,
};

int create_asusproc_compasssensor_status_entry(void){
	compasssensor_entry = proc_create("ecompass_status", S_IWUGO| S_IRUGO, NULL,&compasssensor_proc_fops);
	if (!compasssensor_entry)
		 return -ENOMEM;

	return 0;
}

//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
/*constant IIO attribute */
/**
 *  inv_ak09911_probe() - probe function.
 */

static int inv_ak09911_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
	struct inv_ak09911_state_s *st;
	struct iio_dev *indio_dev;
	int result;
pr_info("wlin: inv_ak09911_probe\n");
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
		goto out_no_free;
	}
	indio_dev = iio_device_alloc(sizeof(*st));
	if (indio_dev == NULL) {
		result =  -ENOMEM;
		goto out_no_free;
	}
	st = iio_priv(indio_dev);
	st->i2c = client;
	st->sl_handle = client->adapter;
	//st->plat_data =
	//	*(struct mpu_platform_data *)dev_get_platdata(&client->dev);
	st->plat_data = ak09911_compass_data;

	st->i2c_addr = client->addr;
	st->delay = AK09911_DEFAULT_DELAY;
	st->compass_id = id->driver_data;

	printk("[ak9916] i2c addr = %d\n", st->i2c_addr);

	i2c_set_clientdata(client, indio_dev);
	result = ak09911_init(st);
	if (result)
		goto out_free;

	indio_dev->dev.parent = &client->dev;
	indio_dev->name = id->name;
	indio_dev->channels = compass_channels;
	indio_dev->num_channels = ARRAY_SIZE(compass_channels);
	indio_dev->info = &ak09911_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;
	//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>>>>>>+
	inv_ak09911 = st;
	if(create_asusproc_compasssensor_status_entry())
		printk("[%s] : ERROR to create compasssensor proc entry\n",__func__);
	//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
	result = inv_ak09911_configure_ring(indio_dev);
	if (result)
		goto out_free;
	result = iio_buffer_register(indio_dev, indio_dev->channels,
					indio_dev->num_channels);
	if (result)
		goto out_unreg_ring;
	result = inv_ak09911_probe_trigger(indio_dev);
	if (result)
		goto out_remove_ring;
pr_info("wlin:inv_ak09911 register device\n");
	result = iio_device_register(indio_dev);
	if (result)
		goto out_remove_trigger;
	INIT_DELAYED_WORK(&st->work, ak09911_work_func);

	pr_info("%s: wlin Probe name %s\n", __func__, id->name);
	return 0;
out_remove_trigger:
	if (indio_dev->modes & INDIO_BUFFER_TRIGGERED)
		inv_ak09911_remove_trigger(indio_dev);
out_remove_ring:
	iio_buffer_unregister(indio_dev);
out_unreg_ring:
	inv_ak09911_unconfigure_ring(indio_dev);
out_free:
	iio_device_free(indio_dev);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
	return -EIO;
}

/**
 *  inv_ak09911_remove() - remove function.
 */
static int inv_ak09911_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct inv_ak09911_state_s *st = iio_priv(indio_dev);
	cancel_delayed_work_sync(&st->work);
	iio_device_unregister(indio_dev);
	inv_ak09911_remove_trigger(indio_dev);
	iio_buffer_unregister(indio_dev);
	inv_ak09911_unconfigure_ring(indio_dev);
	iio_device_free(indio_dev);

	dev_info(&client->adapter->dev, "inv-ak09911-iio module removed.\n");
	return 0;
}

static int inv_ak09911_pm_suspend(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));

	printk("[SENSOR] : inv_ak09911_pm_suspend\n");

	if(akm_enable_status)
	{
		set_ak09911_enable(indio_dev,false);
		because_of_suspend = true;
	}

	return 0;
}

static int inv_ak09911_pm_resume(struct device *dev)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(to_i2c_client(dev));

	printk("[SENSOR] : inv_ak09911_pm_resume\n");

	if(because_of_suspend)
	{
		set_ak09911_enable(indio_dev,true);
		because_of_suspend = false;
	}

	return 0;
}

static const struct dev_pm_ops inv_ak09911_pm = {
	.suspend       = inv_ak09911_pm_suspend,
	.resume        = inv_ak09911_pm_resume,
};

#define INV_AK09911_PM (&inv_ak09911_pm)

static const unsigned short normal_i2c[] = { I2C_CLIENT_END };

/* device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_ak09911_id[] = {
	//{"ak09911", COMPASS_ID_AK09911},
	{"ak9916", COMPASS_ID_AK09911},
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_ak09911_id);

static struct i2c_driver inv_ak09911_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_ak09911_probe,
	.remove		=	inv_ak09911_remove,
	.id_table	=	inv_ak09911_id,
	.driver = {
		.owner	=	THIS_MODULE,
		//.name	=	"inv-ak09911-iio",
		.name	=	"ak9916",
		.pm = INV_AK09911_PM,
	},
	.address_list = normal_i2c,
};

static int __init inv_ak09911_init(void)
{
	printk("[ak9916] init +++ \n");
	int result = i2c_add_driver(&inv_ak09911_driver);
	if (result) {
		pr_err("%s failed\n", __func__);
		return result;
	}
	return 0;
}

static void __exit inv_ak09911_exit(void)
{
	i2c_del_driver(&inv_ak09911_driver);
	//<asus-annacheng20150129>>>>>>>>>>>>>>>>>>+
	if(compasssensor_entry)
		remove_proc_entry("compasssensor_status", NULL);
	//<asus-annacheng20150129><<<<<<<<<<<<<<<<<+
}

module_init(inv_ak09911_init);
module_exit(inv_ak09911_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv-ak09911-iio");
