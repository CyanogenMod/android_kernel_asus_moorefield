/*
 * Support for Aptina ar0543_raw 1080p HD camera sensor.
 *
 * Copyright (c) 2011 Intel Corporation. All Rights Reserved.
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kmod.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/bitops.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_mid_rpmsg.h>
#include "ar0543_raw.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define to_ar0543_raw_sensor(sd) container_of(sd, struct ar0543_raw_device, sd)

#define HOME_POS 255
#define PR3_2_FW 0x0400
#define PR3_3_FW 0x0500


#define VCM_MODE            0x02
#define VCM_MOVE_TIME       0x03
#define VCM_CODE_MSB        0x04
#define VCM_CODE_LSB        0x05
#define VCM_THRESHOLD_MSB   0x06
#define VCM_THRESHOLD_LSB   0x07

#define RING_CTRL           0x1   //enable Ringing
#define RING_MODE           0x15  //Owen ARC MODE = 0x15
#define VCM_FREQ            0x60  //Owen 87Hz Resonant freq.
#define VCM_TRSH_MValue     0x00  //Owen Threshold MSB
#define VCM_TRSH_LValue     0xa0  //Owen Threshold LSB


/* divides a by b using half up rounding and div/0 prevention
 * (result is 0 if b == 0) */
#define divsave_rounded(a, b)	(((b) != 0) ? (((a)+((b)>>1))/(b)) : (-1))

//For LED FLASH Controll+++
#ifdef CONFIG_PF450CL
#define FLED_DRIVER_ENT "FLED_DRIVER_ENT"
#define FLED_DRIVER_ENF "FLED_DRIVER_ENF"

static int flash_ent;
static int flash_enf;
#endif
//For LED FLASH Controll---

//Add for ATD command+++
extern int build_version; //Add build version -> user:3, userdebug:2, eng:1
struct v4l2_subdev *main_sd;

int ATD_ar0543_raw_status = 0;
static char camera_module_otp[60];

static void *ar0543_raw_otp_read(struct v4l2_subdev *sd);

static ssize_t ar0543_raw_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	printk("%s: get ar0543_raw status (%d) !!\n", __func__, ATD_ar0543_raw_status);
	//Check sensor connect status, just do it  in begining for ATD camera status

	return sprintf(buf,"%d\n", ATD_ar0543_raw_status);
}

static ssize_t ar0543_raw_read_otp(struct device *dev,struct device_attribute *attr,char *buf)
{
	printk("%s: get ar0543 module OTP %s !!\n", __func__, camera_module_otp);
	//Check sensor OTP value, just do it in begining for ATD camera status
/*
	if(build_version != 1){ //not eng, need to read otp first
		ar0543_raw_otp_read(main_sd);
	}
*/
	return sprintf(buf,"%s", camera_module_otp);
}

static DEVICE_ATTR(ar0543_raw_status, S_IRUGO,ar0543_raw_show_status,NULL);
static DEVICE_ATTR(ar0543_raw_read_otp, S_IRUGO,ar0543_raw_read_otp,NULL);

static struct attribute *ar0543_raw_attributes[] = {
	&dev_attr_ar0543_raw_status.attr,
	&dev_attr_ar0543_raw_read_otp.attr,
	NULL
};
//Add for ATD command---

//Add for vcm +++
#if 0
static int vcm_i2c_rd8(struct i2c_client *client, u8 reg, u8 *val)
{
	struct i2c_msg msg[2];
	u8 buf[2];
	buf[0] = reg;
	buf[1] = 0;

	msg[0].addr = VCM_ADDR;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &buf[0];

	msg[1].addr = VCM_ADDR;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &buf[1];
	*val = 0;
	if (i2c_transfer(client->adapter, msg, 2) != 2)
		return -EIO;
	*val = buf[1];
	return 0;
}
#endif

static int vcm_i2c_wr8(struct i2c_client *client, u8 reg, u8 val)
{
        int err;
	struct i2c_msg msg;
	u8 buf[2];
	buf[0] = reg;
	buf[1] = val;
	msg.addr = 0xc;
	msg.flags = 0;
	msg.len = 2;
	msg.buf = &buf[0];

        err = i2c_transfer(client->adapter, &msg, 1);

	if (err != 1){
                printk("%s: rear camera vcm i2c fail, err code = %d\n", __func__, err);
		return -EIO;
        }
	return 0;
}
//Add for vcm ---

static int
ar0543_raw_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	u16 data[AR0543_RAW_SHORT_MAX];
	int err, i;

	struct i2c_msg msg[2] = {
		{
			.addr = client->addr,
			.flags = 0,
			.len = I2C_MSG_LENGTH,
			.buf = (unsigned char *)&reg,
		}, {
			.addr = client->addr,
			.len = len,
			.flags = I2C_M_RD,
			.buf = (u8 *)data,
		}
	};

	reg = cpu_to_be16(reg);

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	/* @len should be even when > 1 */
	if (len > AR0543_RAW_BYTE_MAX) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	/* high byte comes first */
	if (len == AR0543_RAW_8BIT) {
		*val = (u8)data[0];
	} else {
		/* 16-bit access is default when len > 1 */
		for (i = 0; i < (len >> 1); i++)
			val[i] = be16_to_cpu(data[i]);
	}

	return 0;

error:
	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int ar0543_raw_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;
	int retry = 0;

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	/*
	 * It is said that Rev 2 sensor needs some delay here otherwise
	 * registers do not seem to load correctly. But tests show that
	 * removing the delay would not cause any in-stablility issue and the
	 * delay will cause serious performance down, so, removed previous
	 * mdelay(1) here.
	 */

	if (ret == num_msg)
		return 0;

	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying i2c write transfer... %d",
			retry);
		retry++;
		msleep(20);
		goto again;
	}

	return ret;
}

static int
ar0543_raw_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != AR0543_RAW_8BIT && data_length != AR0543_RAW_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}
	/* high byte goes out first */
	*(u16 *)data = cpu_to_be16(reg);
	switch (data_length) {
	case AR0543_RAW_8BIT:
		data[2] = (u8)val;
		break;
	case AR0543_RAW_16BIT:
		*(u16 *)&data[2] = cpu_to_be16(val);
		break;
	default:
		dev_err(&client->dev,
			"write error: invalid length type %d\n", data_length);
	}

	ret = ar0543_raw_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}


/**
 * ar0543_raw_rmw_reg - Read/Modify/Write a value to a register in the sensor
 * device
 * @client: i2c driver client structure
 * @data_length: 8/16-bits length
 * @reg: register address
 * @mask: masked out bits
 * @set: bits set
 *
 * Read/modify/write a value to a register in the  sensor device.
 * Returns zero if successful, or non-zero otherwise.
 */
static int ar0543_raw_rmw_reg(struct i2c_client *client, u16 data_length, u16 reg,
			   u16 mask, u16 set)
{
	int err;
	u16 val[AR0543_RAW_SHORT_MAX];

	/* Exit when no mask */
	if (mask == 0)
		return 0;

	/* @mask must not exceed data length */
	if (data_length == AR0543_RAW_8BIT && mask & ~0xff)
		return -EINVAL;

	err = ar0543_raw_read_reg(client, data_length, reg, val);
	if (err) {
		v4l2_err(client, "ar0543_raw_rmw_reg error exit, read failed\n");
		return -EINVAL;
	}

	val[0] &= ~mask;

	/*
	 * Perform the OR function if the @set exists.
	 * Shift @set value to target bit location. @set should set only
	 * bits included in @mask.
	 *
	 * REVISIT: This function expects @set to be non-shifted. Its shift
	 * value is then defined to be equal to mask's LSB position.
	 * How about to inform values in their right offset position and avoid
	 * this unneeded shift operation?
	 */
	set <<= ffs(mask) - 1;
	val[0] |= set & mask;

	err = ar0543_raw_write_reg(client, data_length, reg, val[0]);
	if (err) {
		v4l2_err(client, "ar0543_raw_rmw_reg error exit, write failed\n");
		return -EINVAL;
	}

	return 0;
}


/*
 * ar0543_raw_write_reg_array - Initializes a list of ar0543_raw registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ar0543_raw_flush_reg_array, __ar0543_raw_buf_reg_array() and
 * __ar0543_raw_write_reg_is_consecutive() are internal functions to
 * ar0543_raw_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ar0543_raw_flush_reg_array(struct i2c_client *client,
				     struct ar0543_raw_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ar0543_raw_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ar0543_raw_buf_reg_array(struct i2c_client *client,
				   struct ar0543_raw_write_ctrl *ctrl,
				   const struct ar0543_raw_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case AR0543_RAW_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case AR0543_RAW_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg.sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= AR0543_RAW_MAX_WRITE_BUF_SIZE)
		return __ar0543_raw_flush_reg_array(client, ctrl);

	return 0;
}

static int
__ar0543_raw_write_reg_is_consecutive(struct i2c_client *client,
				   struct ar0543_raw_write_ctrl *ctrl,
				   const struct ar0543_raw_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg.sreg;
}

static int ar0543_raw_write_reg_array(struct i2c_client *client,
				   const struct ar0543_raw_reg *reglist)
{
	const struct ar0543_raw_reg *next = reglist;
	struct ar0543_raw_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != AR0543_RAW_TOK_TERM; next++) {
		switch (next->type & AR0543_RAW_TOK_MASK) {
		case AR0543_RAW_TOK_DELAY:
			err = __ar0543_raw_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;

		case AR0543_RAW_RMW:
			err = __ar0543_raw_flush_reg_array(client, &ctrl);
			err |= ar0543_raw_rmw_reg(client,
					       next->type & ~AR0543_RAW_RMW,
					       next->reg.sreg, next->val,
					       next->val2);
			if (err) {
				v4l2_err(client, "%s: rwm error, "
						"aborted\n", __func__);
				return err;
			}
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__ar0543_raw_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ar0543_raw_flush_reg_array(client, &ctrl);
				if (err)
					return err;
			}
			err = __ar0543_raw_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ar0543_raw_flush_reg_array(client, &ctrl);
}

static int ar0543_raw_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	int ret = 0;
	u8 msb, lsb;

        value = clamp(value, 0, VCM_MAX_FOCUS_POS);
        msb = (((value >> 8) & 0x03)| RING_CTRL<<2);  //owen for Arc enable
        lsb = (value & 0xff);

        //printk("%s: ========== FOCUS_POS:%x,%x,%x \n", __func__, value,msb,lsb);
        ret = vcm_i2c_wr8(client, VCM_CODE_MSB, msb);
        ret = vcm_i2c_wr8(client, VCM_CODE_LSB, lsb);

	//ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT, AR0543_RAW_VCM_CODE, value);
	if (ret == 0) {
		dev->number_of_steps = value - dev->focus;
		dev->focus = value;
	//	getnstimeofday(&(dev->timestamp_t_focus_abs));
	}
	return ret;
}

static int ar0543_raw_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	return ar0543_raw_t_focus_abs(sd, dev->focus + value);
}

#define DELAY_PER_STEP_NS	1000000
#define DELAY_MAX_PER_STEP_NS	(1000000*40)
static int ar0543_raw_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	u32 status = 0;
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct timespec temptime;
	const struct timespec timedelay = {
		0,
		min((u32)abs(dev->number_of_steps)*DELAY_PER_STEP_NS,
			(u32)DELAY_MAX_PER_STEP_NS),
	};

	getnstimeofday(&temptime);

	temptime = timespec_sub(temptime, (dev->timestamp_t_focus_abs));

	if (timespec_compare(&temptime, &timedelay) <= 0) {
		status |= ATOMISP_FOCUS_STATUS_MOVING;
		status |= ATOMISP_FOCUS_HP_IN_PROGRESS;
	} else {
		status |= ATOMISP_FOCUS_STATUS_ACCEPTS_NEW_MOVE;
		status |= ATOMISP_FOCUS_HP_COMPLETE;
	}
	*value = status;
	return 0;
}

static int ar0543_raw_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	s32 val;

	ar0543_raw_q_focus_status(sd, &val);

	if (val & ATOMISP_FOCUS_STATUS_MOVING)
		*value  = dev->focus - dev->number_of_steps;
	else
		*value  = dev->focus ;

	return 0;
}

//For DIT VCM Debug Interface+++
#define	VCM_PROC_FILE	"driver/camera_vcm"
static struct proc_dir_entry *vcm_proc_file;

static int vcm_proc_read(struct seq_file *buf, void *v)
{
    s32 vcm_steps;
    ar0543_raw_q_focus_abs(main_sd, &vcm_steps);
    pr_info("vcm_proc_read vcm_steps %d\n", vcm_steps);
    seq_printf(buf, "%d\n", vcm_steps);
    return 0;
}

static int vcm_proc_open(struct inode *inode, struct  file *file) {
    return single_open(file, vcm_proc_read, NULL);
}

static ssize_t vcm_proc_write(struct file *filp, const char __user *buff,
            size_t len, loff_t *off)
{
    char buffer[256];
    s32 value;

    if (len > 256)
        len = 256;

    pr_info("vcm_proc_write %s\n", buff);
    if (copy_from_user(buffer, buff, len)) {
        printk(KERN_INFO "%s: proc write to buffer failed.\n", __func__);
        return -EFAULT;
    }
    if (!strncmp("setabs",buffer,6)) {
        sscanf(&buffer[7],"%d",&value);
        printk("set position to %d\n", value);
        ar0543_raw_t_focus_abs(main_sd, value);
    } else if(!strncmp("setrel",buffer,6)){
        sscanf(&buffer[7],"%d",&value);
        printk("add position %d\n", value);
        ar0543_raw_t_focus_rel(main_sd, value);
    } else {
        pr_info("command not support\n");
    }

    return len;
}

static const struct file_operations vcm_fops = {
        .owner = THIS_MODULE,
        .open = vcm_proc_open,
        .write = vcm_proc_write,
        .read = seq_read,
};

void create_vcm_proc_file(void)
{
    vcm_proc_file = proc_create(VCM_PROC_FILE, 0666, NULL,&vcm_fops);
    if(vcm_proc_file){
        printk("proc file create sucessed!\n");
    }
    else{
        printk("proc file create failed!\n");
    }
}
//For DIT VCM Debug Interface---

//For LED FLASH Controll+++
#ifdef CONFIG_PF450CL
#define	LED_PROC_FILE	"driver/camera_flash"
static struct proc_dir_entry *led_proc_file;

ssize_t led_proc_read(struct file *file, char __user *buf, size_t size, loff_t *ppos)
{
    return sprintf(buf, "Not support this command\n");
}

static int led_proc_open_show(struct seq_file *buf, void *v)
{
    seq_printf(buf, "Not support this command\n");
    return 0;
}


static int led_proc_open(struct inode *inode, struct  file *file) {
    return single_open(file, led_proc_open_show, NULL);
}

static ssize_t led_proc_write(struct file *filp, const char __user *buff,
            size_t len, loff_t *off)
{
    char buffer[256];
//    s32 value;
    int ret;

    if (len > 256)
        len = 256;

    pr_info("led_proc_write %s\n", buff);
    if (copy_from_user(buffer, buff, len)) {
        printk(KERN_INFO "%s: proc write to buffer failed.\n", __func__);
        return -EFAULT;
    }

    //FLED_DRIVER_ENT 159
    if (flash_ent < 0) {
        ret = gpio_request(159, FLED_DRIVER_ENT);
        if (ret < 0) {
            printk("%s not available.\n", FLED_DRIVER_ENT);
            return ret;
        }
        gpio_direction_output(159, 0);
        flash_ent = 159;
    }

    //FLED_DRIVER_ENF 174
    if (flash_enf < 0) {
        ret = gpio_request(174, FLED_DRIVER_ENF);
        if (ret < 0) {
            printk("%s not available.\n", FLED_DRIVER_ENF);
            return ret;
        }
        gpio_direction_output(174, 0);
        flash_enf = 174;
    }

    if (!strncmp("set_torch_on",buffer,12)) {
        printk("set torch on\n");
        gpio_set_value(flash_ent, 0);
        gpio_set_value(flash_enf, 0);
        msleep(10);
        gpio_set_value(flash_ent, 1);
    } else if(!strncmp("set_torch_off",buffer,13)){
        printk("set torch off\n");
        gpio_set_value(flash_ent, 0);
        gpio_set_value(flash_enf, 0);
        msleep(10);
    } else if(!strncmp("set_flash_on",buffer,12)){
        printk("set flash on\n");
        gpio_set_value(flash_ent, 0);
        gpio_set_value(flash_enf, 0);
        msleep(10);
        gpio_set_value(flash_ent, 1);
        msleep(1);
        gpio_set_value(flash_enf, 1);
        msleep(1);
        gpio_set_value(flash_ent, 0);
        msleep(1);
    } else if(!strncmp("set_flash_off",buffer,13)){
        printk("set flash off\n");
        gpio_set_value(flash_ent, 0);
        gpio_set_value(flash_enf, 0);
        msleep(10);
    } else {
        pr_info("command not support\n");
    }

    return len;
}

static const struct file_operations led_fops = {
        .owner = THIS_MODULE,
        .open = led_proc_open,
        .write = led_proc_write,
        .read = led_proc_read,
};

void create_led_proc_file(void)
{
    led_proc_file = proc_create(LED_PROC_FILE, 0666, NULL,&led_fops);
    if(led_proc_file){
        printk("led proc file create sucessed!\n");
    }
    else{
        printk("led proc file create failed!\n");
    }
}
#endif
//For LED FLASH Controll---

static int ar0543_raw_real_to_register_gain(u16 gain, u16 *real_gain)
{

                u16 reg_gain;
                u16 cg, asc1;
//		u16 gain_value;
		u16 calculated_gain;
#if 0
                if (_gain > 255)
			gain = 255 * 256; /*Cap max gain to use only analog portion.*/
		else
			gain = _gain * 256;
#endif
                if (gain < 22) /*Use 2nd stage as soon as possible*/
                {
				calculated_gain = gain * 2;
                                cg = 0x0;
                                asc1 = 0x0;
                }
                else if (gain < 32)
                {
				calculated_gain = gain * 6 / 4;
                                cg = 0x0;
                                asc1 = 0x100;
                }
                else if (gain < 43)
                {
				calculated_gain = gain;
                                cg = 0x800;
                                asc1 = 0x0;
                }
                else if (gain < 48)
                {
				calculated_gain = gain * 768 / 0x400;
                                cg = 0x800;
                                asc1 = 0x100;
                }
                else if (gain < 64)
                {
				calculated_gain = gain * 682 / 0x400;
                                cg = 0x400;
                                asc1 = 0x0;
                }
                else if (gain < 85)
                {
				calculated_gain = gain * 2 / 4;
                                cg = 0xc00;
                                asc1 = 0x0;
                }
                else if (gain < 128)
                {
				calculated_gain = gain * 384 / 0x400;
                                cg = 0xc00;
                                asc1 = 0x100;
                }
                else
                {
				calculated_gain = gain / 4;
                                cg = 0xc00;
                                asc1 = 0x200;
                }
		reg_gain = calculated_gain;
		//printk("%s gain %d calculated_gain %x\n", __func__, gain, reg_gain);
                reg_gain |= cg;
                reg_gain |= asc1;
		*real_gain = reg_gain;

		return 0;
}

static long ar0543_raw_set_exposure(struct v4l2_subdev *sd, u16 coarse_itg,
				 u16 fine_itg, u16 a_gain, u16 d_gain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 frame_length;
	u16 real_gain;
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	mutex_lock(&dev->input_lock);

	/* enable group hold */
	ret = ar0543_raw_write_reg_array(client, ar0543_raw_param_hold);
	if (ret)
		goto out;

	frame_length = ar0543_raw_res[dev->fmt_idx].lines_per_frame;
	if (frame_length < coarse_itg)
		frame_length = coarse_itg + 5;

	/* set frame length lines */
	ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT,
		AR0543_RAW_FRAME_LENGTH_LINES, frame_length);
	if (ret)
		goto out_disable;

	/* set coarse integration time */
	ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT,
			AR0543_RAW_COARSE_INTEGRATION_TIME, coarse_itg);
	if (ret)
		goto out_disable;

	/* set fine integration time */
	ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT,
			AR0543_RAW_FINE_INTEGRATION_TIME, fine_itg);
	if (ret)
		goto out_disable;

	ret = ar0543_raw_real_to_register_gain(a_gain, &real_gain);
	real_gain |= 0x1000;

//	printk("%s write gain %x fine %x coarse %x frame_length %x\n", __func__,
//		real_gain, fine_itg, coarse_itg, frame_length);

	/* set global gain */
	ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT,
			AR0543_RAW_GLOBAL_GAIN, real_gain);

	if (ret)
		goto out_disable;


	dev->gain       = real_gain;
	dev->coarse_itg = coarse_itg;
	dev->fine_itg   = fine_itg;

out_disable:
	/* disable group hold */
	ar0543_raw_write_reg_array(client, ar0543_raw_param_update);
out:
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long ar0543_raw_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	u16 coarse_itg, fine_itg, analog_gain, digital_gain;

	coarse_itg = exposure->integration_time[0];
	fine_itg = exposure->integration_time[1];
	analog_gain = exposure->gain[0];
	digital_gain = exposure->gain[1];

	/* we should not accept the invalid value below */
	if (fine_itg == 0 || analog_gain == 0) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		v4l2_err(client, "%s: invalid value\n", __func__);
		return -EINVAL;
	}

	//printk("%s analog %x digital %x\n", __func__, analog_gain, digital_gain);
	return ar0543_raw_set_exposure(sd, coarse_itg, fine_itg, analog_gain, digital_gain);
}

static int ar0543_raw_read_reg_array(struct i2c_client *client, u16 size, u16 addr,
				  void *data)
{
	u8 *buf = data;
	u16 index;
	int ret = 0;

	for (index = 0; index + AR0543_RAW_BYTE_MAX <= size;
	     index += AR0543_RAW_BYTE_MAX) {
		ret = ar0543_raw_read_reg(client, AR0543_RAW_BYTE_MAX, addr + index,
				       (u16 *)&buf[index]);
		if (ret)
			return ret;
	}

	if (size - index > 0)
		ret = ar0543_raw_read_reg(client, size - index, addr + index,
				       (u16 *)&buf[index]);

	return ret;
}

static int ar0543_raw_otp_switch_bank(struct v4l2_subdev *sd, u16 bank)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	int retry = 100;
	u16 ready;

	//pr_info("%s Read bank addr 0x%X\n",__func__,bank);
	// choose to only read record type 0x30
	ret = ar0543_raw_rmw_reg(client, AR0543_RAW_16BIT, AR0543_RAW_OTP_RECORD_TYPE, 0xFF00, bank);
	if (ret) {
		v4l2_err(client, "%s: failed to choose the record type\n",
			 __func__);
		return ret;
	}

	// auto read start
	ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT, AR0543_RAW_OTP_AUTO_READ, 0x0010);
	if (ret) {
		v4l2_err(client, "%s: failed to read start automatically\n",
			 __func__);
		return ret;
	}

	do {
		ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT,
				       AR0543_RAW_OTP_AUTO_READ, &ready);
		if (ret) {
			v4l2_err(client, "%s: failed to read OTP memory "
					 "status\n", __func__);
			return ret;
		}
		if (ready & AR0543_RAW_OTP_READY_REG_DONE)
			break;
	} while (--retry);

	if (!retry) {
		v4l2_err(client, "%s: OTP memory read timeout.\n", __func__);
		return -ETIMEDOUT;
	}

	if (!(ready & AR0543_RAW_OTP_READY_REG_OK)) {
		v4l2_err(client, "%s: Bank 0x%x OTP memory wasn't read successfully\n",
			  __func__, bank);
		return -EIO;
	}

	return 0;
}

static int
__ar0543_raw_otp_read(struct v4l2_subdev *sd, struct ar0543_raw_af_data *buf)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 read_value[10], i;

	// RESET_REGISTER_REG_RD_EN
	ret = ar0543_raw_rmw_reg(client, AR0543_RAW_16BIT, AR0543_RAW_OTP_READ_EN, 0x20, 1);
	if (ret) {
		v4l2_err(client, "%s: failed to reset register REG_RD_EN\n",
			 __func__);
		return ret;
	}

	// disable streaming
	ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT, AR0543_RAW_OTP_READ_EN, 0x0610);
	if (ret) {
		v4l2_err(client, "%s: failed to disable streaming\n",
			 __func__);
		return ret;
	}

	// timing parameters for otp read
	ret = ar0543_raw_write_reg(client, AR0543_RAW_16BIT, AR0543_RAW_OTP_TIMING, 0xCD95);
	if (ret) {
		v4l2_err(client, "%s: failed to set timing parameters for OTPM read\n",
			 __func__);
		return ret;
	}

	//Decide OTP Bank
	ret = ar0543_raw_otp_switch_bank(sd,0x32); //bank2
	if(ret) { //fail
		//pr_info("%s Switch to bank 2 fail, try bank 1\n",__func__);
		ret = ar0543_raw_otp_switch_bank(sd,0x31); //bank1
		if(ret) {
			//pr_info("%s Switch to bank 1 fail, try bank 0\n",__func__);
			ret = ar0543_raw_otp_switch_bank(sd,0x30); //bank0
			if(ret) {
				//pr_info("%s Switch to bank 0 fail, otp read fail !!\n",__func__);
				return -EIO;
			}
		}
	}

	//Read OTP value
	for (i=0; i<10; i++) {
		ar0543_raw_read_reg(client, AR0543_RAW_8BIT, AR0543_RAW_OTP_AF_INF_POS_H+i, &read_value[i]);
	}

	//Return OTP value
	snprintf(camera_module_otp, sizeof(camera_module_otp), "0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X 0x%X\n"
		, read_value[0], read_value[1], read_value[2], read_value[3], read_value[4]
		, read_value[5], read_value[6], read_value[7], read_value[8], read_value[9]);

	pr_info("%s OTP value: %s",__func__, camera_module_otp);

	buf->af_inf_pos = read_value[0]<<8 | read_value[1];
	buf->af_1m_pos = read_value[2]<<8 | read_value[3];
	buf->af_10cm_pos = read_value[4]<<8 | read_value[5];
	buf->af_start_curr = read_value[6]<<8 | read_value[7];
	buf->module_id = read_value[8];
	buf->vendor_id = read_value[9];
	buf->default_af_inf_pos = AR0543_DEFAULT_AF_INF;
	buf->default_af_10cm_pos = AR0543_DEFAULT_AF_10CM;
	buf->default_af_start = AR0543_DEFAULT_AF_START;
	buf->default_af_end = AR0543_DEFAULT_AF_END;

	return 0;
}

static void *ar0543_raw_otp_read(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	void *buf;
	int ret;

	buf = kmalloc(AR0543_RAW_OTP_DATA_SIZE, GFP_KERNEL);
	if (!buf)
		return ERR_PTR(-ENOMEM);

	/*
	 * Try all banks in reverse order and return after first success.
	 * Last used bank has most up-to-date data.
	 */
	ret = __ar0543_raw_otp_read(sd, buf);
	/* Driver has failed to find valid data */
	if (ret) {
		v4l2_err(client, "%s: sensor found no valid OTP data\n",
			  __func__);
		kfree(buf);
		return ERR_PTR(ret);
	}

	return buf;
}

static u8 *ar0543_raw_fuseid_read(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 data[AR0543_RAW_FUSEID_SIZE];
	u8 *fuseid;
	int ret, i;

	ret = ar0543_raw_read_reg_array(client, AR0543_RAW_FUSEID_SIZE,
				     AR0543_RAW_FUSEID_START_ADDR, &data);
	if (ret < 0) {
		v4l2_err(client, "%s: error reading FUSEID.\n", __func__);
		return ERR_PTR(ret);
	}

	fuseid = kmalloc(sizeof(*fuseid) * AR0543_RAW_FUSEID_SIZE, GFP_KERNEL);

	if (!fuseid) {
		v4l2_err(client, "%s: no memory available when reading "
				 "FUSEID.\n", __func__);
		return ERR_PTR(-ENOMEM);
	}

	/* FUSEID is in reverse order */
	for (i = 0; i < AR0543_RAW_FUSEID_SIZE; i++)
		fuseid[i] = data[AR0543_RAW_FUSEID_SIZE - i - 1];

	return fuseid;
}

static int ar0543_raw_g_priv_int_data(struct v4l2_subdev *sd,
				   struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;

	/* No OTP data available on sensor */
	if (!dev->otp_data || !dev->fuseid)
		return -EIO;

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	/* Correct read_size value only if bigger than maximum */
	if (read_size > AR0543_RAW_OTP_DATA_SIZE)
		read_size = AR0543_RAW_OTP_DATA_SIZE;

	ret = copy_to_user(to, dev->otp_data, read_size);
	if (ret) {
		v4l2_err(client, "%s: failed to copy OTP data to user\n",
			 __func__);
		return -EFAULT;
	}

	/* No room for FUSEID */
	if (priv->size <= AR0543_RAW_OTP_DATA_SIZE)
		goto out;

	read_size = priv->size - AR0543_RAW_OTP_DATA_SIZE;
	if (read_size > AR0543_RAW_FUSEID_SIZE)
		read_size = AR0543_RAW_FUSEID_SIZE;
	to += AR0543_RAW_OTP_DATA_SIZE;

	ret = copy_to_user(to, dev->fuseid, read_size);
	if (ret) {
		v4l2_err(client, "%s: failed to copy FUSEID to user\n",
			 __func__);
		return -EFAULT;
	}

out:
	/* Return correct size */
	priv->size = AR0543_RAW_OTP_DATA_SIZE + AR0543_RAW_FUSEID_SIZE;

	return 0;
}

static long ar0543_raw_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ar0543_raw_s_exposure(sd, (struct atomisp_exposure *)arg);
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return ar0543_raw_g_priv_int_data(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int ar0543_raw_init_registers(struct v4l2_subdev *sd)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
        vcm_i2c_wr8(client, 0x01, 0x01); // vcm init test

	//owen  Arc function initial setting
	vcm_i2c_wr8(client, VCM_MODE, RING_MODE);
	vcm_i2c_wr8(client, VCM_MOVE_TIME, VCM_FREQ);
	vcm_i2c_wr8(client, VCM_THRESHOLD_MSB,  VCM_TRSH_MValue);
	vcm_i2c_wr8(client, VCM_THRESHOLD_LSB,  VCM_TRSH_LValue);
	//printk("%s: ========== initial setting:%x,%x,%x,%x \n", __func__, RING_MODE,VCM_FREQ,VCM_TRSH_MValue,VCM_TRSH_LValue);
	ret  = ar0543_raw_write_reg_array(client, ar0543_raw_reset_register);
	return ret;
}

static int __ar0543_raw_init(struct v4l2_subdev *sd, u32 val)
{
	int ret;
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	u8 fw_rev[16] = {0};

	/* set inital registers */
	ret = ar0543_raw_init_registers(sd);

	/*set VCM to home position */
	//ret |= ar0543_raw_t_focus_abs(sd, HOME_POS);

	/* restore settings */
	ar0543_raw_res = ar0543_raw_res_preview;
	N_RES = N_RES_PREVIEW;

	/* The only way to detect whether this VCM maintains focus after putting
	 * the sensor into standby mode is by looking at the SCU FW version.
	 * PR3.3 or higher runs on FW version 05.00 or higher.
	 * We cannot distinguish between PR3.2 and PR3.25, so we need to be
	 * conservative. PR3.25 owners can change the comparison to compare
	 * to PR3_2_FW instead of PR3_3_FW for testing purposes.
	 */
	dev->keeps_focus_pos = false;
	ret |= rpmsg_send_generic_command(IPCMSG_FW_REVISION, 0, NULL, 0,
				       (u32 *)fw_rev, 4);
	if (ret == 0) {
		u16 fw_version = (fw_rev[15] << 8) | fw_rev[14];
		dev->keeps_focus_pos = fw_version >= PR3_3_FW;
	}
	if (!dev->keeps_focus_pos) {
		v4l2_warn(sd, "VCM does not maintain focus position in standby"
			      "mode, using software workaround\n");
	}

	return ret;
}

static int ar0543_raw_init(struct v4l2_subdev *sd, u32 val)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ar0543_raw_init(sd, val);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static void ar0543_raw_uninit(struct v4l2_subdev *sd)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	dev->coarse_itg = 0;
	dev->fine_itg   = 0;
	dev->gain       = 0;
	dev->focus      = AR0543_RAW_INVALID_CONFIG;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	int ret;

	dev_info(&client->dev, "%s\n", __func__);
       /* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");
	msleep(20);

//For LED FLASH Controll+++
#ifdef CONFIG_PF450CL
	//FLED_DRIVER_ENT 159
	if (flash_ent < 0) {
		ret = gpio_request(159, FLED_DRIVER_ENT);
		if (ret < 0) {
			printk("%s not available.\n", FLED_DRIVER_ENT);
			return ret;
		}
		gpio_direction_output(159, 0);
		flash_ent = 159;
	}

	//FLED_DRIVER_ENF 174
	if (flash_enf < 0) {
		ret = gpio_request(174, FLED_DRIVER_ENF);
		if (ret < 0) {
			printk("%s not available.\n", FLED_DRIVER_ENF);
			return ret;
		}
		gpio_direction_output(174, 0);
		flash_enf = 174;
	}
#endif
//For LED FLASH Controll---

	return 0;

fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

       dev_info(&client->dev, "%s\n", __func__);

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed 1\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int __ar0543_raw_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	int ret;

	if (on == 0) {
		ar0543_raw_uninit(sd);
		ret = power_down(sd);
		dev->power = 0;
	} else {
		ret = power_up(sd);
		if (!ret) {
			dev->power = 1;
			/* init motor initial position */
			return __ar0543_raw_init(sd, 0);
		}
	}

	return ret;
}

static int ar0543_raw_s_power(struct v4l2_subdev *sd, int on)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	int ret;

	printk("%s: s %d\n",__FUNCTION__, on);

	mutex_lock(&dev->input_lock);
	ret = __ar0543_raw_s_power(sd, on);
	mutex_unlock(&dev->input_lock);
        printk("%s: e\n",__FUNCTION__);
	return ret;
}

static int ar0543_raw_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!chip)
		return -EINVAL;

	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_AR0543_RAW, 0);

	return 0;
}

static int
ar0543_raw_get_intg_factor(struct i2c_client *client,
			struct camera_mipi_info *info,
			const struct ar0543_raw_reg *reglist)
{
	unsigned int	vt_pix_clk_div;
	unsigned int	vt_sys_clk_div;
	unsigned int	pre_pll_clk_div;
	unsigned int	pll_multiplier;
	unsigned int	op_pix_clk_div;
	unsigned int	op_sys_clk_div;

    /* TODO: this should not be a constant but should be set by a call to
     * MSIC's driver to get the ext_clk that MSIC supllies to the sensor.
     */
	const int ext_clk_freq_mhz = 19200000;
	struct atomisp_sensor_mode_data buf;
	const struct ar0543_raw_reg *next = reglist;
	int vt_pix_clk_freq_mhz;
	u16 data[AR0543_RAW_SHORT_MAX];

	unsigned int coarse_integration_time_min;
	unsigned int coarse_integration_time_max_margin;
	unsigned int fine_integration_time_min;
	unsigned int fine_integration_time_max_margin;
	unsigned int frame_length_lines;
	unsigned int line_length_pck;
	unsigned int read_mode;
	u16 value;
	int ret;

	if (info == NULL)
		return -EINVAL;

	/* To make Klocwork happy*/
	buf.reserved[0] = 0;
	buf.reserved[1] = 0;

	memset(data, 0, AR0543_RAW_SHORT_MAX * sizeof(u16));
	if (ar0543_raw_read_reg(client, 12, AR0543_RAW_VT_PIX_CLK_DIV, data))
		return -EINVAL;
	vt_pix_clk_div = data[0];
	vt_sys_clk_div = data[1];
	pre_pll_clk_div = data[2];
	pll_multiplier = data[3];
	op_pix_clk_div = data[4];
	op_sys_clk_div = data[5];

	memset(data, 0, AR0543_RAW_SHORT_MAX * sizeof(u16));
	if (ar0543_raw_read_reg(client, 4, AR0543_RAW_FRAME_LENGTH_LINES, data))
		return -EINVAL;
	frame_length_lines = data[0];
	line_length_pck = data[1];

	memset(data, 0, AR0543_RAW_SHORT_MAX * sizeof(u16));
	if (ar0543_raw_read_reg(client, 8, AR0543_RAW_COARSE_INTG_TIME_MIN, data))
		return -EINVAL;
	coarse_integration_time_min = data[0];
	coarse_integration_time_max_margin = data[1];
	fine_integration_time_min = data[2];
	fine_integration_time_max_margin = data[3];

	memset(data, 0, AR0543_RAW_SHORT_MAX * sizeof(u16));
	if (ar0543_raw_read_reg(client, 2, AR0543_RAW_READ_MODE, data))
		return -EINVAL;
	read_mode = data[0];

	vt_pix_clk_freq_mhz = divsave_rounded(ext_clk_freq_mhz*pll_multiplier,
				pre_pll_clk_div*vt_sys_clk_div*vt_pix_clk_div);

	for (; next->type != AR0543_RAW_TOK_TERM; next++) {
		if (next->type == AR0543_RAW_16BIT) {
			if (next->reg.sreg == AR0543_RAW_FINE_INTEGRATION_TIME) {
				buf.fine_integration_time_def = next->val;
				break;
			}
		}
	}

	/* something's wrong here, this mode does not have fine_igt set! */
	if (next->type == AR0543_RAW_TOK_TERM)
		return -EINVAL;

	buf.coarse_integration_time_min = coarse_integration_time_min;
	buf.coarse_integration_time_max_margin =
					coarse_integration_time_max_margin;
	buf.fine_integration_time_min = fine_integration_time_min;
	buf.fine_integration_time_max_margin = fine_integration_time_max_margin;
	buf.vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	buf.line_length_pck = line_length_pck;
	buf.frame_length_lines = frame_length_lines;
	buf.read_mode = read_mode;

	/* 1: normal 3:inc 2, 7:inc 4 addresses in X direction*/
	buf.binning_factor_x =
		(((read_mode & AR0543_RAW_READ_MODE_X_ODD_INC) >> 6) + 1) / 2;

	/*
	 * 1:normal 3:inc 2, 7:inc 4, 15:inc 8, 31:inc 16, 63:inc 32 addresses
	 * in Y direction
	 */
	buf.binning_factor_y =
			((read_mode & AR0543_RAW_READ_MODE_Y_ODD_INC) + 1) / 2;

	printk("%s binning x %d y %d\n", __func__, buf.binning_factor_x, buf.binning_factor_y);
	/* Get the cropping and output resolution to ISP for this mode. */

	ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT,
				AR0543_RAW_HORIZONTAL_START_H, &value);
	if (ret)
		return ret;
	buf.crop_horizontal_start = value;

	ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT, AR0543_RAW_VERTICAL_START_H,
				&value);
	if (ret)
		return ret;
	buf.crop_vertical_start = value;

	ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT, AR0543_RAW_HORIZONTAL_END_H,
				&value);
	if (ret)
		return ret;
	buf.crop_horizontal_end = value;

	ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT, AR0543_RAW_VERTICAL_END_H,
				&value);
	if (ret)
		return ret;
	buf.crop_vertical_end = value;

	ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT,
				AR0543_RAW_HORIZONTAL_OUTPUT_SIZE_H, &value);
	if (ret)
		return ret;
	buf.output_width = value;

	ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT,
				AR0543_RAW_VERTICAL_OUTPUT_SIZE_H, &value);
	if (ret)
		return ret;
	buf.output_height = value;
	printk("%s crop hori %d - %d vert %d - %d output %d x %d\n", __func__, buf.crop_horizontal_start, buf.crop_horizontal_end,
								buf.crop_vertical_start, buf.crop_vertical_end,
								buf.output_width, buf.output_height);
	memcpy(&info->data, &buf, sizeof(buf));

	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ar0543_raw_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	int ret;

	/* the fine integration time is currently not calculated */
	ret = ar0543_raw_read_reg(client, AR0543_RAW_16BIT,
			       AR0543_RAW_COARSE_INTEGRATION_TIME, &coarse);
	*value = coarse;

	return ret;
}

static int ar0543_raw_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	return ar0543_raw_write_reg(client, AR0543_RAW_16BIT, 0x3070, value);
}


static int ar0543_raw_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (value > AR0543_RAW_VCM_SLEW_STEP_MAX)
		return -EINVAL;

	return ar0543_raw_rmw_reg(client, AR0543_RAW_16BIT, AR0543_RAW_VCM_SLEW_STEP,
				AR0543_RAW_VCM_SLEW_STEP_MASK, value);
}

static int ar0543_raw_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/* Max 16 bits */
	if (value > AR0543_RAW_VCM_SLEW_TIME_MAX)
		return -EINVAL;

	return ar0543_raw_write_reg(client, AR0543_RAW_16BIT, AR0543_RAW_VCM_SLEW_TIME,
				 value);
}

static int ar0543_raw_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (AR0543_RAW_FOCAL_LENGTH_NUM << 16) | AR0543_RAW_FOCAL_LENGTH_DEM;
	return 0;
}

static int ar0543_raw_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for ar0543_raw*/
	*val = (AR0543_RAW_F_NUMBER_DEFAULT_NUM << 16) | AR0543_RAW_F_NUMBER_DEM;
	return 0;
}

static int ar0543_raw_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (AR0543_RAW_F_NUMBER_DEFAULT_NUM << 24) |
		(AR0543_RAW_F_NUMBER_DEM << 16) |
		(AR0543_RAW_F_NUMBER_DEFAULT_NUM << 8) | AR0543_RAW_F_NUMBER_DEM;
	return 0;
}

static int ar0543_raw_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	*val = ar0543_raw_res[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int ar0543_raw_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	*val = ar0543_raw_res[dev->fmt_idx].bin_factor_y;

	return 0;
}

static struct ar0543_raw_control ar0543_raw_controls[] = {
	{
		.qc = {
			.id = V4L2_CID_EXPOSURE_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "exposure",
			.minimum = 0x0,
			.maximum = 0xffff,
			.step = 0x01,
			.default_value = 0x00,
			.flags = 0,
		},
		.query = ar0543_raw_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_TEST_PATTERN,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "Test pattern",
			.minimum = 0,
			.maximum = 0xffff,
			.step = 1,
			.default_value = 0,
		},
		.tweak = ar0543_raw_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = VCM_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_raw_t_focus_abs,
		//.query = ar0543_raw_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = AR0543_RAW_MAX_FOCUS_NEG,
			.maximum = AR0543_RAW_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_raw_t_focus_rel,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_STATUS,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus status",
			.minimum = 0,
			.maximum = 100, /* allow enum to grow in the future */
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ar0543_raw_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.minimum = 0,
			.maximum = AR0543_RAW_VCM_SLEW_STEP_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_raw_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.minimum = 0,
			.maximum = AR0543_RAW_VCM_SLEW_TIME_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ar0543_raw_t_vcm_timing,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = AR0543_RAW_FOCAL_LENGTH_DEFAULT,
			.maximum = AR0543_RAW_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = AR0543_RAW_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ar0543_raw_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = AR0543_RAW_F_NUMBER_DEFAULT,
			.maximum = AR0543_RAW_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = AR0543_RAW_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ar0543_raw_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = AR0543_RAW_F_NUMBER_RANGE,
			.maximum =  AR0543_RAW_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = AR0543_RAW_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ar0543_raw_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = AR0543_RAW_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ar0543_raw_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = AR0543_RAW_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ar0543_raw_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ar0543_raw_controls))

static struct ar0543_raw_control *ar0543_raw_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ar0543_raw_controls[i].qc.id == id)
			return &ar0543_raw_controls[i];
	return NULL;
}

static int ar0543_raw_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct ar0543_raw_control *ctrl = ar0543_raw_find_control(qc->id);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* ar0543_raw control set/get */
static int ar0543_raw_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct ar0543_raw_control *s_ctrl;
	int ret;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = ar0543_raw_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ar0543_raw_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct ar0543_raw_control *octrl = ar0543_raw_find_control(ctrl->id);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

/*
 * distance - calculate the distance
 * @res: resolution
 * @w: width
 * @h: height
 *
 * Get the gap between resolution and w/h.
 * res->width/height smaller than w/h wouldn't be considered.
 * Returns the value of gap or -1 if fail.
 */
static int distance(struct ar0543_raw_resolution const *res, const u32 w,
		    const u32 h, const s32 m)
{
	u32 w_ratio = ((res->width<<13)/w);
	u32 h_ratio = ((res->height<<13)/h);
	s32 match   = abs(((w_ratio<<13)/h_ratio) - ((s32)8192));

	if ((w_ratio < (s32)8192) || (h_ratio < (s32)8192)  || (match > m))
		return -1;

	return w_ratio + h_ratio;
}


/*
 * Tune this value so that the DVS resolutions get selected properly,
 * but make sure 16:9 does not match 4:3
 */
#define LARGEST_ALLOWED_RATIO_MISMATCH 140

/*
 * Returns the nearest higher resolution index.
 * @w: width
 * @h: height
 * matching is done based on enveloping resolution and
 * aspect ratio. If the aspect ratio cannot be matched
 * to any index, the search is done again using envelopel
 * matching only. if no match can be found again, -1 is
 * returned.
 */
static int nearest_resolution_index(int w, int h)
{
	int i, j;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct ar0543_raw_resolution *tmp_res = NULL;
	s32 m = LARGEST_ALLOWED_RATIO_MISMATCH;

	for (j = 0; j < 2; ++j) {
		for (i = 0; i < N_RES; i++) {
			tmp_res = &ar0543_raw_res[i];
			dist = distance(tmp_res, w, h, m);
			if (dist == -1)
				continue;
			if (dist < min_dist) {
				min_dist = dist;
				idx = i;
			}
		}
		if (idx != -1)
			break;
		m = LONG_MAX;
	}
	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ar0543_raw_res[i].width)
			continue;
		if (h != ar0543_raw_res[i].height)
			continue;
		/* Found it */
		return i;
	}
	return -1;
}

static int __ar0543_raw_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if (!fmt)
		return -EINVAL;

	if (fmt->width > AR0543_RAW_RES_WIDTH_MAX ||
	    fmt->height > AR0543_RAW_RES_HEIGHT_MAX) {
		fmt->width = AR0543_RAW_RES_WIDTH_MAX;
		fmt->height = AR0543_RAW_RES_HEIGHT_MAX;
	} else {
		idx = nearest_resolution_index(fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller
		 * resolutions. If it fails, it means the requested
		 * resolution is higher than we can support.
		 * Fallback to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = N_RES - 1;

		fmt->width = ar0543_raw_res[idx].width;
		fmt->height = ar0543_raw_res[idx].height;
	}
#ifdef CONFIG_ME175CG
	fmt->code = V4L2_MBUS_FMT_SGBRG10_1X10;
#else
	fmt->code = V4L2_MBUS_FMT_SGRBG10_1X10;
#endif

	return 0;
}

static int ar0543_raw_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ar0543_raw_try_mbus_fmt(sd, fmt);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ar0543_raw_get_mbus_format_code(struct v4l2_subdev *sd)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	if (!dev->power)
		return -EIO;
#ifdef CONFIG_ME175CG
	return V4L2_MBUS_FMT_SGBRG10_1X10;
#else
	return V4L2_MBUS_FMT_SGRBG10_1X10;
#endif
}

static int ar0543_raw_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	const struct ar0543_raw_reg *ar0543_raw_def_reg;
	struct camera_mipi_info *ar0543_raw_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
//	u16 stream_on;

	printk("%s: s \n",__FUNCTION__);

	ar0543_raw_info = v4l2_get_subdev_hostdata(sd);
	if (ar0543_raw_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = __ar0543_raw_try_mbus_fmt(sd, fmt);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "try fmt fail\n");
		return ret;
	}
	dev->fmt_idx = get_resolution_index(fmt->width, fmt->height);
	printk("%s: __get_resolution_index[%d] %dx%d\n",__FUNCTION__, dev->fmt_idx, fmt->width, fmt->height);
	/* Sanity check */
	if (unlikely(dev->fmt_idx == -1)) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(sd, "get resolution fail\n");
		return -EINVAL;
	}

	ar0543_raw_def_reg = ar0543_raw_res[dev->fmt_idx].regs;
	printk("%s RES %s selected\n", __func__, ar0543_raw_res[dev->fmt_idx].desc);
	ret = ar0543_raw_write_reg_array(client, ar0543_raw_def_reg);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fmt->code = ar0543_raw_get_mbus_format_code(sd);
	if (fmt->code < 0) {
		mutex_unlock(&dev->input_lock);
		return fmt->code;
	}
	dev->fps = ar0543_raw_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ar0543_raw_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ar0543_raw_res[dev->fmt_idx].lines_per_frame;
	dev->coarse_itg = 0;
	dev->fine_itg = 0;
	dev->gain = 0;

	ret = ar0543_raw_get_intg_factor(client, ar0543_raw_info, ar0543_raw_def_reg);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(sd, "failed to get integration_factor\n");
		return -EINVAL;
	}
        printk("%s: e\n",__FUNCTION__);
	return 0;
}

static int ar0543_raw_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = ar0543_raw_res[dev->fmt_idx].width;
	fmt->height = ar0543_raw_res[dev->fmt_idx].height;
	fmt->code = ar0543_raw_get_mbus_format_code(sd);
	mutex_unlock(&dev->input_lock);

	return fmt->code < 0 ? fmt->code : 0;
}

static int ar0543_raw_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 reg;
	ATD_ar0543_raw_status = 0;

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip model and revision IDs */
	if (ar0543_raw_read_reg(client, AR0543_RAW_16BIT, AR0543_RAW_SC_CMMN_CHIP_ID,
				id)) {
		v4l2_err(client, "Reading sensor_id error.\n");
		return -ENODEV;
	}

	if (*id != AR0543_RAW_ID && *id != AR0543_RAW_ID2) {
		v4l2_err(client,
			"sensor ID error, sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}else{
              printk("sensor ID, sensor_id = 0x%x\n", *id);
	}

	if (ar0543_raw_read_reg(client, AR0543_RAW_8BIT, AR0543_RAW_SC_CMMN_REV_ID,
				&reg)) {
		v4l2_err(client, "Reading sensor_rev_id error.\n");
		return -ENODEV;
	}
	*revision = (u8)reg;

	ATD_ar0543_raw_status = 1;

	return 0;
}

/*
 * ar0543_raw stream on/off
 */
static int ar0543_raw_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
//	u16 stream_on;

	printk("%s enable %d\n", __func__, enable);
	mutex_lock(&dev->input_lock);
	if (enable) {
		if (!dev->keeps_focus_pos) {
			struct ar0543_raw_reg ar0543_raw_stream_enable[] = {
				ar0543_raw_streaming[0],
				{AR0543_RAW_16BIT, {0x30F2}, 0x0000}, /* VCM_NEW_CODE */
				INIT_VCM_CONTROL,
				{AR0543_RAW_16BIT, {0x30F2}, 0x0000}, /* VCM_NEW_CODE */
				{AR0543_RAW_TOK_DELAY, {0}, 60},
				{AR0543_RAW_TOK_TERM, {0}, 0}
			};

			ar0543_raw_stream_enable[1].val = dev->focus + 1;
			ar0543_raw_stream_enable[3].val = dev->focus;

			ret = ar0543_raw_write_reg_array(client, ar0543_raw_stream_enable);
		} else {
			ret = ar0543_raw_write_reg_array(client, ar0543_raw_streaming);
		}

		if (ret != 0) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "write_reg_array err\n");
			return ret;
		}
		dev->streaming = 1;
	} else {

		ret = ar0543_raw_write_reg_array(client, ar0543_raw_soft_standby);
		if (ret != 0) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "write_reg_array err\n");
			return ret;
		}
		dev->streaming = 0;
	}

	//Do not restore settings here, as we do not set parameters when switching resolution in video recording
	//ar0543_raw_res = ar0543_raw_res_preview;
	//N_RES = N_RES_PREVIEW;

	/*ar0543_raw_read_reg(client, AR0543_RAW_8BIT, 0x100, &stream_on);
	printk("%s stream_on %d\n", __func__, stream_on);

	ar0543_raw_read_reg(client, AR0543_RAW_16BIT, 0x31AE, &stream_on);
	printk("%s dual-lane %d\n", __func__, stream_on);*/

	mutex_unlock(&dev->input_lock);
	return 0;
}

/*
 * ar0543_raw enum frame size, frame intervals
 */
static int ar0543_raw_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ar0543_raw_res[index].width;
	fsize->discrete.height = ar0543_raw_res[index].height;
	fsize->reserved[0] = ar0543_raw_res[index].used;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar0543_raw_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	unsigned int index = fival->index;

	if (index >= N_RES)
		return -EINVAL;

	mutex_lock(&dev->input_lock);

	/*
	 * Since the isp will donwscale the resolution to the right size,
	 * find the nearest one that will allow the isp to do so important
	 * to ensure that the resolution requested is padded correctly by
	 * the requester, which is the atomisp driver in this case.
	 */
	index = nearest_resolution_index(fival->width, fival->height);

	if (-1 == index) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
/*	fival->width = ar0543_raw_res[index].width;
	fival->height = ar0543_raw_res[index].height; */
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ar0543_raw_res[index].fps;

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar0543_raw_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	mutex_lock(&dev->input_lock);
	*code = ar0543_raw_get_mbus_format_code(sd);
	mutex_unlock(&dev->input_lock);

	return *code < 0 ? *code : 0;
}

static int ar0543_raw_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;
	void *otp_data;
	void *fuseid;

	if (pdata == NULL)
		return -ENODEV;

#ifdef CONFIG_ME372CG

        if (HW_ID == 0xFF){
                HW_ID = Read_HW_ID();
        }

        if(HW_ID == HW_ID_PR || HW_ID == HW_ID_MP){
                //Check for ME372CG or flashless ++
                int PCB_ID10;
                ret = gpio_request(118,"PCB_ID10");
                if (ret) {
                        pr_err("%s: failed to request gpio(pin 118)\n", __func__);
                        return -EINVAL;
                }

                ret = gpio_direction_input(118);
                PCB_ID10 = gpio_get_value(118);
                printk("%s: PCB_ID10 %d\n", __func__, PCB_ID10);
                gpio_free(118);

                //PCBID: 0->IntelISP, !=0->iCatch
                if(PCB_ID10!=0) {
                        printk("%s Pass register this sensor\n",__func__);
                        return -ENODEV;
                }
                //Check for ME372CG or flashless --
        } else {
                printk("%s SKU:0x%x Pass register this sensor\n",HW_ID,__func__);
                return -ENODEV;
        }
#endif

	dev->platform_data = pdata;

	mutex_lock(&dev->input_lock);

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			dev_err(&client->dev, "ar0543_raw platform init err\n");
			return ret;
		}
	}

	/*
	 * The initial state of physical power is unknown
	 * so first power down it to make it to a known
	 * state, and then start the power up sequence
	 */
	power_down(sd);
	msleep(20);

	ret = __ar0543_raw_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "ar0543_raw power-up err.\n");
		//mutex_unlock(&dev->input_lock);
		//return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = ar0543_raw_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "ar0543_raw_detect err s_config.\n");
		//goto fail_detect;
	}

	sensor_revision = 0;
	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	/* Read sensor's OTP data */
	otp_data = ar0543_raw_otp_read(sd);
	if (!IS_ERR(otp_data))
		dev->otp_data = otp_data;

	fuseid = ar0543_raw_fuseid_read(sd);
	if (!IS_ERR(fuseid))
		dev->fuseid = fuseid;

	/* power off sensor */
	ret = __ar0543_raw_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	if (ret) {
		v4l2_err(client, "ar0543_raw power-down err.\n");
		return ret;
	}

	return 0;

//fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	__ar0543_raw_s_power(sd, 0);
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
ar0543_raw_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	if (code->index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	code->code = ar0543_raw_get_mbus_format_code(sd);
	mutex_unlock(&dev->input_lock);

	return code->code < 0 ? code->code : 0;
}

static int
ar0543_raw_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fse->min_width = ar0543_raw_res[index].width;
	fse->min_height = ar0543_raw_res[index].height;
	fse->max_width = ar0543_raw_res[index].width;
	fse->max_height = ar0543_raw_res[index].height;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static struct v4l2_mbus_framefmt *
__ar0543_raw_get_pad_format(struct ar0543_raw_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		v4l2_err(client, "%s err. pad %x\n", __func__, pad);
		return NULL;
	}

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
ar0543_raw_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ar0543_raw_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	fmt->format = *format;

	return 0;
}

static int
ar0543_raw_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ar0543_raw_get_pad_format(dev, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;
	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
ar0543_raw_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	mutex_lock(&dev->input_lock);

	if (dev->streaming) {
		mutex_unlock(&dev->input_lock);
		return -EBUSY;
	}

	dev->run_mode = param->parm.capture.capturemode;

	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		ar0543_raw_res = ar0543_raw_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		//ar0543_raw_res = ar0543_raw_res_still;
		//N_RES = N_RES_STILL;
		//break;
	default:
		ar0543_raw_res = ar0543_raw_res_preview;
		N_RES = N_RES_PREVIEW;
	}

	/* Reset sensor mode */
	/* we do not reset sensor mode, as we now only have one s_mbus_fmt following s_parm
	dev->fmt_idx = 0;
	dev->fps = ar0543_raw_res[dev->fmt_idx].fps;
	dev->pixels_per_line = ar0543_raw_res[dev->fmt_idx].pixels_per_line;
	dev->lines_per_frame = ar0543_raw_res[dev->fmt_idx].lines_per_frame;
	dev->coarse_itg = 0;
	dev->fine_itg = 0;
	dev->gain = 0;
	*/

	mutex_unlock(&dev->input_lock);
	return 0;
}

static int
ar0543_raw_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 lines_per_frame;

	mutex_lock(&dev->input_lock);

	/*
	 * if no specific information to calculate the fps,
	 * just used the value in sensor settings
	 */
	if (!dev->pixels_per_line || !dev->lines_per_frame) {
		interval->interval.numerator = 1;
		interval->interval.denominator = dev->fps;
		mutex_unlock(&dev->input_lock);
		return 0;
	}

	/*
	 * DS: if coarse_integration_time is set larger than
	 * lines_per_frame the frame_size will be expanded to
	 * coarse_integration_time+1
	 */
	if (dev->coarse_itg > dev->lines_per_frame) {
		if (dev->coarse_itg == 0xFFFF) {
			/*
			 * we can not add 1 according to ds, as this will
			 * cause over flow
			 */
			v4l2_warn(client, "%s: abnormal coarse_itg:0x%x\n",
				  __func__, dev->coarse_itg);
			lines_per_frame = dev->coarse_itg;
		} else
			lines_per_frame = dev->coarse_itg + 1;
	} else
		lines_per_frame = dev->lines_per_frame;

	interval->interval.numerator = dev->pixels_per_line *
					lines_per_frame;
	interval->interval.denominator = AR0543_RAW_MCLK * 1000000;

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int ar0543_raw_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*frames = ar0543_raw_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}
static const struct v4l2_subdev_video_ops ar0543_raw_video_ops = {
	.s_stream = ar0543_raw_s_stream,
	.enum_framesizes = ar0543_raw_enum_framesizes,
	.enum_frameintervals = ar0543_raw_enum_frameintervals,
	.enum_mbus_fmt = ar0543_raw_enum_mbus_fmt,
	.try_mbus_fmt = ar0543_raw_try_mbus_fmt,
	.g_mbus_fmt = ar0543_raw_g_mbus_fmt,
	.s_mbus_fmt = ar0543_raw_s_mbus_fmt,
	.s_parm = ar0543_raw_s_parm,
	.g_frame_interval = ar0543_raw_g_frame_interval,
};

static struct v4l2_subdev_sensor_ops ar0543_raw_sensor_ops = {
	.g_skip_frames	= ar0543_raw_g_skip_frames,
};

static const struct v4l2_subdev_core_ops ar0543_raw_core_ops = {
	.g_chip_ident = ar0543_raw_g_chip_ident,
	.queryctrl = ar0543_raw_queryctrl,
	.g_ctrl = ar0543_raw_g_ctrl,
	.s_ctrl = ar0543_raw_s_ctrl,
	.s_power = ar0543_raw_s_power,
	.ioctl = ar0543_raw_ioctl,
	.init = ar0543_raw_init,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops ar0543_raw_pad_ops = {
	.enum_mbus_code = ar0543_raw_enum_mbus_code,
	.enum_frame_size = ar0543_raw_enum_frame_size,
	.get_fmt = ar0543_raw_get_pad_format,
	.set_fmt = ar0543_raw_set_pad_format,
};

static const struct v4l2_subdev_ops ar0543_raw_ops = {
	.core = &ar0543_raw_core_ops,
	.video = &ar0543_raw_video_ops,
	.pad = &ar0543_raw_pad_ops,
	.sensor = &ar0543_raw_sensor_ops,
};

static const struct media_entity_operations ar0543_raw_entity_ops = {
	.link_setup = NULL,
};

static int ar0543_raw_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ar0543_raw_device *dev = to_ar0543_raw_sensor(sd);

	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	kfree(dev->otp_data);
	kfree(dev->fuseid);
	kfree(dev);

	return 0;
}

static int ar0543_raw_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct ar0543_raw_device *dev;
	int ret;

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	create_vcm_proc_file(); //For DIT VCM Debug Interface+++

#ifdef CONFIG_PF450CL
	create_led_proc_file(); //For LED FLASH Controll+++
#endif

	//Add for ATD command+++
	dev->sensor_i2c_attribute.attrs = ar0543_raw_attributes;

	// Register sysfs hooks
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	//Add for ATD command---

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	v4l2_i2c_subdev_init(&(dev->sd), client, &ar0543_raw_ops);

	if (client->dev.platform_data) {
		ret = ar0543_raw_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->sd.entity.ops = &ar0543_raw_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;
#ifdef CONFIG_ME175CG
	dev->format.code = V4L2_MBUS_FMT_SGBRG10_1X10;
#else
	dev->format.code = V4L2_MBUS_FMT_SGRBG10_1X10;
#endif

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		ar0543_raw_remove(client);
		return ret;
	}

	main_sd = &dev->sd;

	return 0;
}

static const struct i2c_device_id ar0543_raw_id[] = {
	{AR0543_RAW_NAME, 0},
	{}
};

MODULE_DEVICE_TABLE(i2c, ar0543_raw_id);

static struct i2c_driver ar0543_raw_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = AR0543_RAW_NAME,
	},
	.probe = ar0543_raw_probe,
	.remove = ar0543_raw_remove,
	.id_table = ar0543_raw_id,
};

static __init int init_ar0543_raw(void)
{
	return i2c_add_driver(&ar0543_raw_driver);
}

static __exit void exit_ar0543_raw(void)
{
	i2c_del_driver(&ar0543_raw_driver);
}

module_init(init_ar0543_raw);
module_exit(exit_ar0543_raw);

MODULE_DESCRIPTION("A low-level driver for Aptina AR0543_RAW sensors");
MODULE_LICENSE("GPL");
