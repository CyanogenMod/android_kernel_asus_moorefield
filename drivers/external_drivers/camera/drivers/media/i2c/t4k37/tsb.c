/*
 * Support for Toshiba T4K37(13MP)/T4K35(8MP) camera sensor.
 *
 * Copyright (c) 2012 Intel Corporation. All Rights Reserved.
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
#include <asm/intel-mid.h>
#include <linux/atomisp_platform.h>
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/gpio.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/libmsrlisthelper.h>
#include <linux/mm.h>
#include <linux/kmod.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-device.h>
#include "tsb.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#if (defined(CONFIG_KXTJ9_SENSOR) && CONFIG_KXTJ9_SENSOR)
#include <linux/kxtj9.h>
#endif
#include <linux/HWVersion.h>
extern u8 tsb_otp_data[24];
static int binning_sum;
static struct tsb_vcm *inner_vcm;
static struct tsb_device *inner_tsb;
static int last_vcm;

static enum atomisp_bayer_order tsb_bayer_order_mapping[] = {
#if 0	// debug bayer 20140609
	atomisp_bayer_order_rggb,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_gbrg,
	atomisp_bayer_order_bggr
#else	// debug bayer 20140609
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg,
	atomisp_bayer_order_grbg
#endif	// debug bayer 20140609
};
#if (defined(CONFIG_KXTJ9_SENSOR) && CONFIG_KXTJ9_SENSOR)
bool isKxtj9Verify = false;
#endif
static int
tsb_read_reg(struct i2c_client *client, u16 len, u16 reg, u16 *val)
{
	struct i2c_msg msg[2];
	u16 data[TSB_SHORT_MAX];
	int err, i;

	//printk("[DEBUG] %s, start\n", __func__);

	if (len > TSB_BYTE_MAX) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));
	memset(data, 0 , sizeof(data));

	msg[0].addr = client->addr;
    msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = (u8 *)data;
	/* high byte goes first */
	data[0] = cpu_to_be16(reg);

	msg[1].addr = client->addr;
	msg[1].len = len;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = (u8 *)data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		goto error;
	}

	/* high byte comes first */
	if (len == TSB_8BIT) {
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


static int tsb_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;

	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

int
tsb_write_reg(struct i2c_client *client, u16 data_length, u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
	u16 *wreg = (u16 *)data;
	const u16 len = data_length + sizeof(u16); /* 16-bit address + data */

	//printk("[DEBUG] %s, start\n", __func__);

	if (data_length != TSB_8BIT && data_length != TSB_16BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	/* high byte goes out first */
	*wreg = cpu_to_be16(reg);

	if (data_length == TSB_8BIT)
		data[2] = (u8)(val);
	else {
		/* TSB_16BIT */
		u16 *wdata = (u16 *)&data[2];
		*wdata = cpu_to_be16(val);
	}

	ret = tsb_i2c_write(client, len, data);
	if (ret)
		dev_err(&client->dev,
			"write error: wrote 0x%x to offset 0x%x error %d",
			val, reg, ret);

	return ret;
}

/*
 * tsb_write_reg_array - Initializes a list of tsb registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __tsb_flush_reg_array, __tsb_buf_reg_array() and
 * __tsb_write_reg_is_consecutive() are internal functions to
 * tsb_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __tsb_flush_reg_array(struct i2c_client *client,
				     struct tsb_write_ctrl *ctrl)
{
	u16 size;

	//printk("[DEBUG] %s, start\n", __func__);

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return tsb_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __tsb_buf_reg_array(struct i2c_client *client,
				   struct tsb_write_ctrl *ctrl,
				   const struct tsb_reg *next)
{
	int size;
	u16 *data16;

	//printk("[DEBUG] %s, start\n", __func__);

	switch (next->type) {
	case TSB_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case TSB_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->sreg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= TSB_MAX_WRITE_BUF_SIZE)
		return __tsb_flush_reg_array(client, ctrl);

	return 0;
}

static int
__tsb_write_reg_is_consecutive(struct i2c_client *client,
				   struct tsb_write_ctrl *ctrl,
				   const struct tsb_reg *next)
{
	//printk("[DEBUG] %s, start\n", __func__);

	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->sreg;
}

static int tsb_write_reg_array(struct i2c_client *client,
				   const struct tsb_reg *reglist)
{
	const struct tsb_reg *next = reglist;
	struct tsb_write_ctrl ctrl;
	int err;

	//printk("[DEBUG] %s, start\n", __func__);

	ctrl.index = 0;
	for (; next->type != TSB_TOK_TERM; next++) {
		switch (next->type & TSB_TOK_MASK) {
		case TSB_TOK_DELAY:
			err = __tsb_flush_reg_array(client, &ctrl);
			if (err){
				return err;
            }
			msleep(next->val);
			break;

		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__tsb_write_reg_is_consecutive(client, &ctrl, next)) {
				err = __tsb_flush_reg_array(client, &ctrl);
				if (err){
					return err;
                }
			}
			err = __tsb_buf_reg_array(client, &ctrl, next);
			if (err) {
				v4l2_err(client, "%s: write error, aborted\n", __func__);
				return err;
			}
			break;
		}
	}

	return __tsb_flush_reg_array(client, &ctrl);
}

static int __tsb_min_fps_diff(int fps, const struct tsb_fps_setting *fps_list)
{
	int diff = INT_MAX;
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	if (fps == 0)
		return 0;

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (abs(fps_list[i].fps - fps) < diff)
			diff = abs(fps_list[i].fps - fps);
	}

	return diff;
}

static int __tsb_nearest_fps_index(int fps,
					const struct tsb_fps_setting *fps_list)
{
	int fps_index = 0;
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (abs(fps_list[i].fps - fps)
		    < abs(fps_list[fps_index].fps - fps))
			fps_index = i;
	}
    printk("%s, fps index = %d\n", __func__, fps_index);
	return fps_index;
}

/*
 * This is to choose the nearest fps setting above the requested fps
 * fps_list should be in ascendant order.
 */
static int __tsb_above_nearest_fps_index(int fps,
					const struct tsb_fps_setting *fps_list)
{
	int fps_index = 0;
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (!fps_list[i].fps)
			break;
		if (fps <= fps_list[i].fps) {
			fps_index = i;
			break;
		}
	}

	return fps_index;
}

static int __tsb_get_max_fps_index(
				const struct tsb_fps_setting *fps_settings)
{
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < MAX_FPS_OPTIONS_SUPPORTED; i++) {
		if (fps_settings[i].fps == 0)
			break;
	}

	return i - 1;
}

static int __tsb_update_exposure_timing(struct i2c_client *client, u16 exposure,
			u16 llp, u16 fll)
{
	int ret = 0;

	//printk("[DEBUG] %s, start\n", __func__);

	/* Increase the VTS to match exposure + margin */
	if (exposure > fll - TSB_INTEGRATION_TIME_MARGIN)
		fll = exposure + TSB_INTEGRATION_TIME_MARGIN;

	ret = tsb_write_reg(client, TSB_16BIT, TSB_LINE_LENGTH_PIXELS, llp);
	if (ret)
		return ret;

	ret = tsb_write_reg(client, TSB_16BIT, TSB_FRAME_LENGTH_LINES, fll);
	if (ret)
		return ret;

	if (exposure)
		ret = tsb_write_reg(client, TSB_16BIT,
			TSB_COARSE_INTEGRATION_TIME, exposure);
	return ret;
}

static int __tsb_update_gain(struct v4l2_subdev *sd, u16 gain)
{
	//struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	/* set global gain */
	//ret = tsb_write_reg(client, TSB_8BIT, TSB_GLOBAL_GAIN, gain);
	ret = tsb_write_reg(client, TSB_16BIT, TSB_GLOBAL_GAIN, gain);
	if (ret)
		return ret;

#if 0	// TSB
	/* set short analog gain */
	if (dev->sensor_id == IMX135_ID)
		ret = imx_write_reg(client, IMX_8BIT, IMX_SHORT_AGC_GAIN, gain);
#endif	// TSB

	return ret;
}

static int __tsb_update_digital_gain(struct i2c_client *client, u16 digitgain)
{
	struct tsb_write_buffer digit_gain;

	//printk("[DEBUG] %s, start\n", __func__);

	digit_gain.addr = cpu_to_be16(TSB_DGC_ADJ);
	digit_gain.data[0] = (digitgain >> 8) & 0xFF;
	digit_gain.data[1] = digitgain & 0xFF;
	digit_gain.data[2] = (digitgain >> 8) & 0xFF;
	digit_gain.data[3] = digitgain & 0xFF;
	digit_gain.data[4] = (digitgain >> 8) & 0xFF;
	digit_gain.data[5] = digitgain & 0xFF;
	digit_gain.data[6] = (digitgain >> 8) & 0xFF;
	digit_gain.data[7] = digitgain & 0xFF;

	return tsb_i2c_write(client, TSB_DGC_LEN, (u8 *)&digit_gain);
}

static int tsb_set_exposure_gain(struct v4l2_subdev *sd, u16 coarse_itg,
	u16 gain, u16 digitgain)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	printk("%s, coarse %d gain %d d_gain %d start\n", __func__, coarse_itg, gain, digitgain);

	/* Validate exposure:  cannot exceed VTS-6 where VTS is 16bit */
	coarse_itg = clamp_t(u16, coarse_itg, 0, TSB_MAX_EXPOSURE_SUPPORTED);

	/* Validate gain: must not exceed maximum 12bit value */
	gain = clamp_t(u16, gain, 55, TSB_MAX_GLOBAL_GAIN_SUPPORTED);

	/* Validate digital gain: must not exceed 10 bit value*/
	digitgain = clamp_t(u16, digitgain, 256, TSB_MAX_DIGITAL_GAIN_SUPPORTED);

	mutex_lock(&dev->input_lock);

	ret = __tsb_update_exposure_timing(client, coarse_itg,
			dev->pixels_per_line, dev->lines_per_frame);
	if (ret)
		goto out;
	dev->coarse_itg = coarse_itg;

#if 0	// TSB
	if (dev->sensor_id == IMX175_ID)
		ret = __imx_update_gain(sd, dev->gain);
	else
#endif	// TSB
		ret = __tsb_update_gain(sd, gain);
	if (ret)
		goto out;
	dev->gain = gain;

#if 0	// TSB
	if ((dev->sensor_id == IMX175_ID) && dev->digital_gain)
		ret = __imx_update_digital_gain(client, dev->digital_gain);
	else
#endif	// TSB
		ret = __tsb_update_digital_gain(client, digitgain);
	if (ret)
		goto out;
	dev->digital_gain = digitgain;

out:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static long tsb_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
    /*printk("%s, start\n", __func__);*/
    struct i2c_client *client = v4l2_get_subdevdata(sd);
    int ret;
    ret = tsb_write_reg_array(client, tsb_param_hold);
    if (ret)
        return ret;
    ret = tsb_set_exposure_gain(sd, exposure->integration_time[0], exposure->gain[0], exposure->gain[1]);
    if (ret)
        return ret;
    return tsb_write_reg_array(client, tsb_param_update);
}

/* FIXME -To be updated with real OTP reading */
#if 1	// TSB 20140529
static int tsb_g_priv_int_data(struct v4l2_subdev *sd,
				   struct v4l2_private_int_data *priv)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);
	u8 __user *to = priv->data;
	u32 read_size = priv->size;
	int ret;

	printk("[DEBUG] %s, start\n", __func__);

	/* No need to copy data if size is 0 */
	if (!read_size)
		goto out;

	if (IS_ERR(dev->otp_data)) {
		dev_err(&client->dev, "OTP data not available");
		return PTR_ERR(dev->otp_data);
	}
	/* Correct read_size value only if bigger than maximum */
	if (read_size > dev->otp_driver->size)
		read_size = dev->otp_driver->size;

	ret = copy_to_user(to, dev->otp_data, read_size);
	if (ret) {
		dev_err(&client->dev, "%s: failed to copy OTP data to user\n",
			 __func__);
		return -EFAULT;
	}
out:
	/* Return correct size */
	priv->size = dev->otp_driver->size;

	return 0;
}
#endif	// TSB 20140529

static int __tsb_init(struct v4l2_subdev *sd, u32 val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);

    printk("%s, start\n", __func__);

	if (dev->sensor_id == TSB_ID_DEFAULT){
        printk("%s, sensor id = %u\n", __func__, dev->sensor_id);
		return 0;
    }

	/* Sets the default FPS */
	dev->fps_index = 0;
	dev->curr_res_table = dev->mode_tables->res_preview;
	dev->entries_curr_table = dev->mode_tables->n_res_preview;

	return tsb_write_reg_array(client,
			dev->mode_tables->init_settings);
}

static int tsb_init(struct v4l2_subdev *sd, u32 val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret = 0;

    printk("%s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	ret = __tsb_init(sd, val);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long tsb_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	printk("%s, start cmd = %u\n", __func__, cmd);

	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return tsb_s_exposure(sd, arg);
	case ATOMISP_IOC_S_BINNING_SUM:
        binning_sum = *(int*)arg;
        printk("Set low-light mode %d\n", binning_sum);
        return 0;
#if 1	// TSB 20140529
	case ATOMISP_IOC_G_SENSOR_PRIV_INT_DATA:
		return tsb_g_priv_int_data(sd, arg);
#endif	// TSB 20140529
	default:
		return -EINVAL;
	}
	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	printk("%s, start\n", __func__);

       /* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret){
		dev_err(&client->dev, "%s power failed\n", __func__);
		goto fail_power;
    }

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret){
		dev_err(&client->dev, "%s flis clock failed\n", __func__);
		goto fail_clk;
    }

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "%s gpio failed\n", __func__);
		goto fail_gpio;
	}

	printk("%s, end\n", __func__);
	return 0;

fail_gpio:
	dev->platform_data->gpio_ctrl(sd, 0);
fail_clk:
	dev->platform_data->flisclk_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	printk("%s, start\n", __func__);

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed\n");

	/* clock ctrl */
	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int __tsb_s_power(struct v4l2_subdev *sd, int on)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret = 0;
	int r = 0;

	printk("%s, start\n", __func__);

    if (on == 0) {
#if (defined(CONFIG_KXTJ9_SENSOR) && CONFIG_KXTJ9_SENSOR)
	if(!isKxtj9Verify){
	    ret = kxtj9_verify();
	    if (ret) {
		printk("tsb_kxtj9 error detect.\n");
	    }
	    else
	    {
	  	isKxtj9Verify = true;
	    }
	}
	else
	    kxtj9_suspend();
#endif
        ret = power_down(sd);
        if (dev->vcm_driver && dev->vcm_driver->power_down){
            r = dev->vcm_driver->power_down(sd);
        }
        if (ret == 0){
            ret = r;
        }
        dev->power = 0;
    } else {
        if (dev->vcm_driver && dev->vcm_driver->power_up){
            ret = dev->vcm_driver->power_up(sd);
        }
        if (ret){
            return ret;
        }
        ret = power_up(sd);
	mdelay(450);
        if (!ret) {
            dev->power = 1;
	    ret = __tsb_init(sd, 0);
#if (defined(CONFIG_KXTJ9_SENSOR) && CONFIG_KXTJ9_SENSOR)
	if(!isKxtj9Verify){
	    r = kxtj9_verify();
	    if (r) {
		printk("tsb_kxtj9 error detect.\n");
	    }
	    else
	    {
	  	isKxtj9Verify = true;
	    }
	}
	else
	    kxtj9_resume();
#endif
            return ret;
        }
    }

	return ret;
}

static int tsb_s_power(struct v4l2_subdev *sd, int on)
{
	int ret;
	struct tsb_device *dev = to_tsb_sensor(sd);

	printk("%s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	ret = __tsb_s_power(sd, on);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int tsb_g_chip_ident(struct v4l2_subdev *sd,
				struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	if (!chip)
		return -EINVAL;

	v4l2_chip_ident_i2c_client(client, chip, V4L2_IDENT_TSB, 0);

	return 0;
}

static int tsb_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct tsb_reg *reglist)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tsb_device *dev = to_tsb_sensor(sd);
	u32 vt_pix_clk_div;
	u32 vt_sys_clk_div;
	u32 pre_pll_clk_div;
	u32 pll_multiplier;

	const int ext_clk_freq_hz = 19200000;
	struct atomisp_sensor_mode_data *buf = &info->data;
	int ret;
	u16 data[TSB_INTG_BUF_COUNT];

	u32 vt_pix_clk_freq_mhz;
	u32 coarse_integration_time_min;
	u32 coarse_integration_time_max_margin;
	u32 read_mode;
	u32 div;
    int offset;

    printk("%s, start\n", __func__);

	if (info == NULL)
		return -EINVAL;

	memset(data, 0, TSB_INTG_BUF_COUNT * sizeof(u16));
	ret = tsb_read_reg(client, 1, TSB_VT_PIX_CLK_DIV, data);
	/*printk("[DEBUG] ret3a=%d\n", ret);*/
	if (ret)
		return ret;
#if 0	// TSB
	vt_pix_clk_div = data[0] & IMX_MASK_5BIT;
#else	// TSB
	vt_pix_clk_div = data[0];
	if(vt_pix_clk_div==0)
		vt_pix_clk_div = 1;
#endif	// TSB

#if 0	// TSB
	if (dev->sensor_id == IMX132_ID)
		ret = imx_read_reg(client, 1, IMX132_VT_RGPLTD, data);
	else
#endif	// TSB
		ret = tsb_read_reg(client, 1, TSB_VT_SYS_CLK_DIV, data);
	/*printk("[DEBUG] ret3b=%d\n", ret);*/
	if (ret)
		return ret;
#if 0	// TSB
	vt_sys_clk_div = data[0] & IMX_MASK_2BIT;
#else	// TSB
	vt_sys_clk_div = data[0];
	if(vt_sys_clk_div==0)
		vt_sys_clk_div = 1;
#endif	// TSB
	ret = tsb_read_reg(client, 1, TSB_PRE_PLL_CLK_DIV, data);
	/*printk("[DEBUG] ret3c=%d\n", ret);*/
	if (ret)
		return ret;
#if 0	// TSB
	pre_pll_clk_div = data[0] & TSB_MASK_4BIT;
#else	// TSB
	pre_pll_clk_div = data[0] + 1;
#endif	// TSB
#if 0	// TSB
	ret = imx_read_reg(client, 2,
		(dev->sensor_id == IMX132_ID) ?
		IMX132_PLL_MULTIPLIER : IMX_PLL_MULTIPLIER, data);
#else	// TSB
	ret = tsb_read_reg(client, 2, TSB_PLL_MULTIPLIER, data);
#endif	// TSB
	/*printk("[DEBUG] ret3d=%d\n", ret);*/
	if (ret)
		return ret;
	//pll_multiplier = data[0] & IMX_MASK_11BIT;
	pll_multiplier = data[0];

	memset(data, 0, TSB_INTG_BUF_COUNT * sizeof(u16));
#if 0	// TSB
	ret = tsb_read_reg(client, 4, TSB_COARSE_INTG_TIME_MIN, data);
	if (ret)
		return ret;
	coarse_integration_time_min = data[0];
	coarse_integration_time_max_margin = data[1];
#else	// TSB
	coarse_integration_time_min = 1;
	coarse_integration_time_max_margin = 6;
#endif	// TSB

	/* Get the cropping and output resolution to ISP for this mode. */

	memset(data, 0, TSB_INTG_BUF_COUNT * sizeof(u16));
	/* 2:1/4binning enabled, 1:1/2binning enabled, 0:disabled */
	ret = tsb_read_reg(client, 1, TSB_H_BINNING_ENABLE, data);
	if (ret)
		return ret;
	buf->binning_factor_x = data[0] & TSB_MASK_2BIT;
	buf->binning_factor_x *= 2;
    if(!(buf->binning_factor_x))
        buf->binning_factor_x = 1;
    read_mode = data[0] & TSB_MASK_2BIT;

	ret = tsb_read_reg(client, 1, TSB_V_BINNING_ENABLE, data);
	if (ret)
		return ret;
	buf->binning_factor_y = data[0] & TSB_MASK_2BIT;
	buf->binning_factor_y *= 2;
    if(!(buf->binning_factor_y))
        buf->binning_factor_y = 1;

	ret = tsb_read_reg(client, 2, TSB_HORIZONTAL_OUTPUT_SIZE_H, data);
	/*printk("[DEBUG] ret3g=%d\n", ret);*/
	if (ret)
		return ret;
	buf->output_width = data[0];

	ret = tsb_read_reg(client, 2, TSB_VERTICAL_OUTPUT_SIZE_H, data);
	/*printk("[DEBUG] ret3h=%d\n", ret);*/
	if (ret)
		return ret;
	buf->output_height = data[0];

    if((buf->output_width * buf->binning_factor_x * 3) > (buf->output_height * buf->binning_factor_y * 4)){
        buf->crop_horizontal_start = 0;
        buf->crop_horizontal_end = buf->output_width * buf->binning_factor_x - 1;
        offset = (buf->output_width * buf->binning_factor_x*3/4 - buf->output_height * buf->binning_factor_y)/2;
        if(offset%2)
            offset -= 1;
        buf->crop_vertical_start = offset;
        buf->crop_vertical_end = buf->output_height * buf->binning_factor_y + offset - 1;
    }
    else{
        buf->crop_vertical_start = 0;
        buf->crop_vertical_end = buf->output_height * buf->binning_factor_y - 1;
        offset = (buf->output_height * buf->binning_factor_y*4/3 - buf->output_width * buf->binning_factor_x)/2;
        if(offset%2)
            offset -= 1;
        buf->crop_horizontal_start = offset;
        buf->crop_horizontal_end = buf->output_width * buf->binning_factor_x + offset - 1;
    }

	div = pre_pll_clk_div*vt_sys_clk_div*vt_pix_clk_div;
	/*printk("[DEBUG] ret3j=%d\n", div);*/
	if (div == 0)
		return -EINVAL;

#if 0	// TSB
	vt_pix_clk_freq_mhz = 2 * ext_clk_freq_hz / div;
	vt_pix_clk_freq_mhz *= pll_multiplier;
#else	// TSB
	vt_pix_clk_freq_mhz = 4 * ext_clk_freq_hz / div;	// K37
	//vt_pix_clk_freq_mhz = 2 * ext_clk_freq_hz / div;	// K35
	vt_pix_clk_freq_mhz *= pll_multiplier;
#endif	// TSB

	dev->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;

	buf->vt_pix_clk_freq_mhz = vt_pix_clk_freq_mhz;
	/*printk("[DEBUG] vt_pix_clk_freq_mhz=%d\n", buf->vt_pix_clk_freq_mhz);*/
	buf->coarse_integration_time_min = coarse_integration_time_min;
	buf->coarse_integration_time_max_margin =
				coarse_integration_time_max_margin;
	/*printk("[DEBUG] coarse_intg_min=%d\n", buf->coarse_integration_time_min);*/
	/*printk("[DEBUG] coarse_intg_max_margin=%d\n", buf->coarse_integration_time_max_margin);*/

	buf->fine_integration_time_min = TSB_FINE_INTG_TIME;
	buf->fine_integration_time_max_margin = TSB_FINE_INTG_TIME;
	buf->fine_integration_time_def = TSB_FINE_INTG_TIME;
	buf->frame_length_lines = dev->lines_per_frame;
	buf->line_length_pck = dev->pixels_per_line;
	buf->read_mode = read_mode;

	/*printk("[DEBUG] end of ret3\n");*/
    printk("[DEBUG] bin_y=%d\n", buf->binning_factor_y);
    printk("@%s, binning_factor %dx%d, crop %d~%d x %d~%d, size %dx%d\n", __func__, buf->binning_factor_x, buf->binning_factor_y, buf->crop_horizontal_start, buf->crop_horizontal_end, buf->crop_vertical_start, buf->crop_vertical_end, buf->output_width, buf->output_height);
	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int tsb_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 coarse;
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	/* the fine integration time is currently not calculated */
	ret = tsb_read_reg(client, TSB_16BIT,
			       TSB_COARSE_INTEGRATION_TIME, &coarse);
	*value = coarse;

	return ret;
}

static int tsb_test_pattern(struct v4l2_subdev *sd, s32 value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	return tsb_write_reg(client, TSB_16BIT, TSB_TEST_PATTERN_MODE, value);
}

static enum v4l2_mbus_pixelcode
tsb_translate_bayer_order(enum atomisp_bayer_order code)
{
	/*printk("%s, start\n", __func__);*/

#if 0	// debug bayer 20140609
	switch (code) {
	case atomisp_bayer_order_rggb:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SRGGB10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SRGGB10_1X10;
	case atomisp_bayer_order_grbg:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SGRBG10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SGRBG10_1X10;
	case atomisp_bayer_order_bggr:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SBGGR10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SBGGR10_1X10;
	case atomisp_bayer_order_gbrg:
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SGBRG10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SGBRG10_1X10;
	}
#else	// debug bayer 20140609
		printk("[DEBUG] bayer = V4L2_MBUS_FMT_SGRBG10_1X10\n");	// 20140609
		return V4L2_MBUS_FMT_SGRBG10_1X10;
#endif	// debug bayer 20140609
	return 0;
}

static int tsb_v_flip(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct camera_mipi_info *tsb_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val;

	//printk("[DEBUG] %s, start\n", __func__);

	ret = tsb_write_reg_array(client, tsb_param_hold);
	if (ret)
		return ret;
	ret = tsb_read_reg(client, TSB_8BIT, TSB_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= TSB_VFLIP_BIT;
	else
		val &= ~TSB_VFLIP_BIT;
	ret = tsb_write_reg(client, TSB_8BIT,
			TSB_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info) {
		val &= (TSB_VFLIP_BIT|TSB_HFLIP_BIT);
		tsb_info->raw_bayer_order = tsb_bayer_order_mapping[val];
		dev->format.code = tsb_translate_bayer_order(
			tsb_info->raw_bayer_order);
	}

	return tsb_write_reg_array(client, tsb_param_update);
}

static int tsb_h_flip(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct camera_mipi_info *tsb_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	u16 val;

	//printk("[DEBUG] %s, start\n", __func__);

	ret = tsb_write_reg_array(client, tsb_param_hold);
	if (ret)
		return ret;
	ret = tsb_read_reg(client, TSB_8BIT, TSB_IMG_ORIENTATION, &val);
	if (ret)
		return ret;
	if (value)
		val |= TSB_HFLIP_BIT;
	else
		val &= ~TSB_HFLIP_BIT;
	ret = tsb_write_reg(client, TSB_8BIT,
			TSB_IMG_ORIENTATION, val);
	if (ret)
		return ret;

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info) {
		val &= (TSB_VFLIP_BIT|TSB_HFLIP_BIT);
		tsb_info->raw_bayer_order = tsb_bayer_order_mapping[val];
		dev->format.code = tsb_translate_bayer_order(
		tsb_info->raw_bayer_order);
	}

	return tsb_write_reg_array(client, tsb_param_update);
}

static int tsb_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	//printk("[DEBUG] %s, start\n", __func__);

	*val = (TSB_FOCAL_LENGTH_NUM << 16) | TSB_FOCAL_LENGTH_DEM;
	return 0;
}

static int tsb_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	//printk("[DEBUG] %s, start\n", __func__);

	/*const f number for tsb*/
	*val = (TSB_F_NUMBER_DEFAULT_NUM << 16) | TSB_F_NUMBER_DEM;
	return 0;
}

static int tsb_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	//printk("[DEBUG] %s, start\n", __func__);

	*val = (TSB_F_NUMBER_DEFAULT_NUM << 24) |
		(TSB_F_NUMBER_DEM << 16) |
		(TSB_F_NUMBER_DEFAULT_NUM << 8) | TSB_F_NUMBER_DEM;
	return 0;
}

static int tsb_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	*val = dev->curr_res_table[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int tsb_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	*val = dev->curr_res_table[dev->fmt_idx].bin_factor_y;

	return 0;
}

int tsb_vcm_power_up(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->power_up)
		return dev->vcm_driver->power_up(sd);
	return 0;
}

int tsb_vcm_power_down(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->power_down)
		return dev->vcm_driver->power_down(sd);
	return 0;
}

int tsb_vcm_init(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->init)
		return dev->vcm_driver->init(sd);
	return 0;
}

int tsb_t_focus_vcm(struct v4l2_subdev *sd, u16 val)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_focus_vcm)
		return dev->vcm_driver->t_focus_vcm(sd, val);
	return 0;
}

int tsb_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);
	last_vcm = value;
	if (dev->vcm_driver && dev->vcm_driver->t_focus_abs)
		return dev->vcm_driver->t_focus_abs(sd, value);
	return 0;
}
int tsb_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_focus_rel)
		return dev->vcm_driver->t_focus_rel(sd, value);
	return 0;
}

int tsb_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->q_focus_status)
		return dev->vcm_driver->q_focus_status(sd, value);
	return 0;
}

int tsb_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->q_focus_abs)
		return dev->vcm_driver->q_focus_abs(sd, value);
	return 0;
}

int tsb_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_vcm_slew)
		return dev->vcm_driver->t_vcm_slew(sd, value);
	return 0;
}

int tsb_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	//printk("[DEBUG] %s, start\n", __func__);

	if (dev->vcm_driver && dev->vcm_driver->t_vcm_timing)
		return dev->vcm_driver->t_vcm_timing(sd, value);
	return 0;
}

struct tsb_control tsb_controls[] = {
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
		.query = tsb_q_exposure,
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
		.tweak = tsb_test_pattern,
	},
	{
		.qc = {
			.id = V4L2_CID_VFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Flip",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = tsb_v_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_HFLIP,
			.type = V4L2_CTRL_TYPE_BOOLEAN,
			.name = "Mirror",
			.minimum = 0,
			.maximum = 1,
			.step = 1,
			.default_value = 0,
		},
		.tweak = tsb_h_flip,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move absolute",
			.minimum = 0,
			.maximum = TSB_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_focus_abs,
		.query = tsb_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = TSB_MAX_FOCUS_NEG,
			.maximum = TSB_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_focus_rel,
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
		.query = tsb_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.minimum = 0,
			.maximum = TSB_VCM_SLEW_STEP_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.minimum = 0,
			.maximum = TSB_VCM_SLEW_TIME_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = tsb_t_vcm_timing,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = TSB_FOCAL_LENGTH_DEFAULT,
			.maximum = TSB_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = TSB_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = tsb_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = TSB_F_NUMBER_DEFAULT,
			.maximum = TSB_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = TSB_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = tsb_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = TSB_F_NUMBER_RANGE,
			.maximum =  TSB_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = TSB_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = tsb_g_fnumber_range,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = TSB_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = tsb_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = TSB_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = tsb_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(tsb_controls))

static struct tsb_control *tsb_find_control(u32 id)
{
	int i;

	//printk("[DEBUG] %s, start\n", __func__);

	for (i = 0; i < N_CONTROLS; i++)
		if (tsb_controls[i].qc.id == id)
			return &tsb_controls[i];
	return NULL;
}

static int tsb_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct tsb_control *ctrl = tsb_find_control(qc->id);
	struct tsb_device *dev = to_tsb_sensor(sd);

	//printk("[DEBUG] %s, start\n", __func__);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* tsb control set/get */
static int tsb_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tsb_control *s_ctrl;
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

	if (!ctrl)
		return -EINVAL;

	s_ctrl = tsb_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int tsb_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct tsb_control *octrl = tsb_find_control(ctrl->id);
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	//printk("[DEBUG] %s, start\n", __func__);

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
#define LARGEST_ALLOWED_RATIO_MISMATCH 600
static int distance(struct tsb_resolution const *res, u32 w, u32 h)
{
	unsigned int w_ratio;
	unsigned int h_ratio;
	int match;

	//printk("[DEBUG] %s, start\n", __func__);

	if (w == 0)
		return -1;
	w_ratio = (res->width << 13) / w;
	if (h == 0)
		return -1;
	h_ratio = (res->height << 13) / h;
	if (h_ratio == 0)
		return -1;
	match   = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(struct v4l2_subdev *sd, int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int fps_diff;
	int min_fps_diff = INT_MAX;
	int min_dist = INT_MAX;
	const struct tsb_resolution *tmp_res = NULL;
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/

	for (i = 0; i < dev->entries_curr_table; i++) {
		tmp_res = &dev->curr_res_table[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
		if (dist == min_dist) {
			fps_diff = __tsb_min_fps_diff(dev->fps,
						tmp_res->fps_options);
			if (fps_diff < min_fps_diff) {
				min_fps_diff = fps_diff;
				idx = i;
			}
		}
	}
    if(binning_sum==1 && dev->curr_res_table[idx].width == dev->curr_res_table[idx+1].width && dev->curr_res_table[idx].height == dev->curr_res_table[idx+1].height){
        if(dev->digital_gain>=512){
            idx++;
            printk("T4K37 binning sum mode on\n");
        }
    }
    printk("%s, index = %d\n", __func__, idx);

	return idx;
}

static int tsb_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int idx = 0;

    printk("%s, start\n", __func__);

	mutex_lock(&dev->input_lock);

	if ((fmt->width > tsb_max_res.res_max_width)
		|| (fmt->height > tsb_max_res.res_max_height)) {
		fmt->width =  tsb_max_res.res_max_width;
		fmt->height = tsb_max_res.res_max_height;
	} else {
		idx = nearest_resolution_index(sd, fmt->width, fmt->height);

		/*
		 * nearest_resolution_index() doesn't return smaller
		 *  resolutions. If it fails, it means the requested
		 *  resolution is higher than wecan support. Fallback
		 *  to highest possible resolution in this case.
		 */
		if (idx == -1)
			idx = dev->entries_curr_table - 1;

		fmt->width = dev->curr_res_table[idx].width;
		fmt->height = dev->curr_res_table[idx].height;
	}

	fmt->code = dev->format.code;

	mutex_unlock(&dev->input_lock);
	return 0;
}

/* Call with ctrl_handler.lock hold */
static int __adjust_hvblank(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);
	u16 new_frame_length_lines, new_line_length_pck;
	int ret;

	/*printk("%s, start\n", __func__);*/

	/*
	 * No need to adjust h/v blank if not set dbg value
	 * Note that there is no other checking on the h/v blank value,
	 * as h/v blank can be set to any value above zero for debug purpose
	 */
	if (!dev->v_blank->val || !dev->h_blank->val)
		return 0;

	new_frame_length_lines = dev->curr_res_table[dev->fmt_idx].height +
		dev->v_blank->val;
	new_line_length_pck = dev->curr_res_table[dev->fmt_idx].width +
		dev->h_blank->val;

	ret = tsb_write_reg(client, TSB_16BIT, TSB_LINE_LENGTH_PIXELS,
			    new_line_length_pck);
	if (ret)
		return ret;
	ret = tsb_write_reg(client, TSB_16BIT, TSB_FRAME_LENGTH_LINES,
			    new_frame_length_lines);
	if (ret)
		return ret;

	dev->lines_per_frame = new_frame_length_lines;
	dev->pixels_per_line = new_line_length_pck;

	return 0;
}

static int tsb_s_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct camera_mipi_info *tsb_info = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct tsb_resolution *res;
	int ret;
	u16 val;

	/*printk("%s, start\n", __func__);*/

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info == NULL)
		return -EINVAL;
	ret = tsb_try_mbus_fmt(sd, fmt);
	if (ret)
		return ret;

	mutex_lock(&dev->input_lock);

	printk("%s, width=%d, height=%d\n", __func__, fmt->width, fmt->height);
	dev->fmt_idx = nearest_resolution_index(sd, fmt->width, fmt->height);
	if (dev->fmt_idx == -1) {
		ret = -EINVAL;
		goto out;
	}
	res = &dev->curr_res_table[dev->fmt_idx];

	/* Adjust the FPS selection based on the resolution selected */
	dev->fps_index = __tsb_nearest_fps_index(dev->fps, res->fps_options);
	dev->fps = res->fps_options[dev->fps_index].fps;
	dev->regs = res->fps_options[dev->fps_index].regs;
	if (!dev->regs)
		dev->regs = res->regs;

	ret = tsb_write_reg_array(client, dev->regs);
	if (ret)
		goto out;

	dev->pixels_per_line = res->fps_options[dev->fps_index].pixels_per_line;
	dev->lines_per_frame = res->fps_options[dev->fps_index].lines_per_frame;
	printk("%s RES %s selected\n", __func__, res->desc);

	/* dbg h/v blank time */
	mutex_lock(dev->ctrl_handler.lock);
	__adjust_hvblank(sd);
	mutex_unlock(dev->ctrl_handler.lock);

	ret = __tsb_update_exposure_timing(client, dev->coarse_itg,
		dev->pixels_per_line, dev->lines_per_frame);
	//printk("[DEBUG] ret1=%d\n", ret);
	if (ret)
		goto out;

	ret = tsb_write_reg_array(client, tsb_param_update);
	//printk("[DEBUG] ret2=%d\n", ret);
	if (ret)
		goto out;

	ret = tsb_get_intg_factor(client, tsb_info, dev->regs);
	/*printk("[DEBUG] ret3=%d\n", ret);*/
	if (ret)
		goto out;

	ret = tsb_read_reg(client, TSB_8BIT, TSB_IMG_ORIENTATION, &val);
	/*printk("[DEBUG] ret4=%d\n", ret);*/
	if (ret)
		goto out;
	val &= (TSB_VFLIP_BIT|TSB_HFLIP_BIT);
	tsb_info->raw_bayer_order = tsb_bayer_order_mapping[val];
	/*printk("[DEBUG] bayer_order=%d\n", tsb_info->raw_bayer_order);*/
	dev->format.code = tsb_translate_bayer_order(
		tsb_info->raw_bayer_order);
out:
	printk("%s, ret = %d\n", __func__, ret);
	mutex_unlock(&dev->input_lock);
	return ret;
}


static int tsb_g_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = dev->curr_res_table[dev->fmt_idx].width;
	fmt->height = dev->curr_res_table[dev->fmt_idx].height;
	fmt->code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int tsb_detect(struct i2c_client *client, u16 *id, u8 *revision)
{
	struct i2c_adapter *adapter = client->adapter;

	/*printk("%s, start\n", __func__);*/

	/* i2c check */
	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	/* check sensor chip ID	 */
#if 0	// TSB
	if (imx_read_reg(client, IMX_16BIT, IMX132_175_CHIP_ID, id)) {
		v4l2_err(client, "sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	if (*id == IMX132_ID || *id == IMX175_ID)
		goto found;

	if (imx_read_reg(client, IMX_16BIT, IMX134_135_CHIP_ID, id)) {
		v4l2_err(client, "sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	if (*id != IMX134_ID && *id != IMX135_ID) {
		v4l2_err(client, "no imx sensor found\n");
		return -ENODEV;
	}
#else	// TSB
	if (tsb_read_reg(client, TSB_16BIT, TSB_CHIP_ID, id)) {
		v4l2_err(client, "sensor_id = 0x%x\n", *id);
		return -ENODEV;
	}
	if (*id == T4K37_ID ) {
        *id = T4K37_NAME_ID;
    }
    else{
		v4l2_err(client, "no toshiba sensor found\n");
		return -ENODEV;
	}
#endif	// TSB
//found:	// TSB
	v4l2_info(client, "sensor_id = 0x%x\n", *id);

	/* TODO - need to be updated */
	*revision = 0;

	return 0;
}

static void __tsb_print_timing(struct v4l2_subdev *sd)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 width = dev->curr_res_table[dev->fmt_idx].width;
	u16 height = dev->curr_res_table[dev->fmt_idx].height;

	/*printk("%s, start\n", __func__);*/

	dev_dbg(&client->dev, "Dump tsb timing in stream on:\n");
	dev_dbg(&client->dev, "width: %d:\n", width);
	dev_dbg(&client->dev, "height: %d:\n", height);
	dev_dbg(&client->dev, "pixels_per_line: %d:\n", dev->pixels_per_line);
	dev_dbg(&client->dev, "line per frame: %d:\n", dev->lines_per_frame);
	dev_dbg(&client->dev, "pix freq: %d:\n", dev->vt_pix_clk_freq_mhz);
	dev_dbg(&client->dev, "init fps: %d:\n", dev->vt_pix_clk_freq_mhz /
			dev->pixels_per_line / dev->lines_per_frame);
	dev_dbg(&client->dev, "HBlank: %d nS:\n",
			1000 * (dev->pixels_per_line - width) /
			(dev->vt_pix_clk_freq_mhz / 1000000));
	dev_dbg(&client->dev, "VBlank: %d uS:\n",
			(dev->lines_per_frame - height) * dev->pixels_per_line /
			(dev->vt_pix_clk_freq_mhz / 1000000));
}

/*
 * tsb stream on/off
 */
static int tsb_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct tsb_device *dev = to_tsb_sensor(sd);

	printk("%s, start\n", __func__);

	mutex_lock(&dev->input_lock);
	if (enable) {
		/* Noise reduction & dead pixel applied before streaming */
		if (dev->fw == NULL) {
			dev_warn(&client->dev, "No MSR loaded from library");
		} else {
			ret = apply_msr_data(client, dev->fw);
			if (ret) {
				mutex_unlock(&dev->input_lock);
				return ret;
			}
		}
		__tsb_print_timing(sd);
		ret = tsb_write_reg_array(client, tsb_streaming);
		if (ret != 0) {
			v4l2_err(client, "write_reg_array err\n");
			mutex_unlock(&dev->input_lock);
			return ret;
		}
		dev->streaming = 1;
	} else {
		ret = tsb_write_reg_array(client, tsb_soft_standby);
		if (ret != 0) {
			v4l2_err(client, "write_reg_array err\n");
			mutex_unlock(&dev->input_lock);
			return ret;
		}
		dev->streaming = 0;
		dev->fps_index = 0;
		dev->fps = 0;
	}
	mutex_unlock(&dev->input_lock);
	printk("%s, end\n", __func__);

	return 0;
}

/*
 * tsb enum frame size, frame intervals
 */
static int tsb_enum_framesizes(struct v4l2_subdev *sd,
				   struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/
	printk("%s, index in tsb_enum_framesizes is %d\n", __func__, index);

	mutex_lock(&dev->input_lock);
	if (index >= dev->entries_curr_table) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->curr_res_table[index].width;
	fsize->discrete.height = dev->curr_res_table[index].height;
	fsize->reserved[0] = dev->curr_res_table[index].used;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int tsb_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	/* since the isp will donwscale the resolution to the right size,
	  * find the nearest one that will allow the isp to do so
	  * important to ensure that the resolution requested is padded
	  * correctly by the requester, which is the atomisp driver in
	  * this case.
	  */
	i = nearest_resolution_index(sd, fival->width, fival->height);

	if (i == -1)
		goto out;

	/* Check if this index is supported */
	if (index > __tsb_get_max_fps_index(dev->curr_res_table[i].fps_options))
		goto out;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = dev->curr_res_table[i].width;
	fival->height = dev->curr_res_table[i].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = dev->curr_res_table[i].fps_options[index].fps;
	mutex_unlock(&dev->input_lock);
	return 0;
out:
	mutex_unlock(&dev->input_lock);
	return -EINVAL;
}

static int tsb_enum_mbus_fmt(struct v4l2_subdev *sd, unsigned int index,
				 enum v4l2_mbus_pixelcode *code)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	/*printk("%s, start\n", __func__);*/

	if (index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}


static int __update_tsb_device_settings(struct tsb_device *dev, u16 sensor_id)
{
	/*printk("%s, start\n", __func__);*/

	switch (sensor_id) {
	case T4K37_NAME_ID:
		dev->mode_tables = &tsb_set;
		dev->vcm_driver = &tsb_vcm;
		break;
    /*
     *case T4K35_ID:
     *    dev->mode_tables = &tsb_sets[T4K35_MERRFLD];
     *    dev->vcm_driver = &tsb_vcms[T4K35_MERRFLD];
     *    break;
     */
	default:
		return -EINVAL;
	}

	return dev->vcm_driver->init(&dev->sd);
}

static int tsb_s_config(struct v4l2_subdev *sd,
			    int irq, void *pdata)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u8 sensor_revision;
	u16 sensor_id;
	int ret;
	printk("[DEBUG] %s, start\n", __func__);

	if (pdata == NULL)
		return -ENODEV;

	dev->platform_data = pdata;

	mutex_lock(&dev->input_lock);

	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			dev_err(&client->dev, "tsb platform init err\n");
			return ret;
		}
	}

	ret = __tsb_s_power(sd, 1);
	if (ret) {
		v4l2_err(client, "tsb power-up err.\n");
		mutex_unlock(&dev->input_lock);
		return ret;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = tsb_detect(client, &sensor_id, &sensor_revision);
	if (ret) {
		v4l2_err(client, "tsb_detect err s_config.\n");
		goto fail_detect;
	}

#if (defined(CONFIG_KXTJ9_SENSOR) && CONFIG_KXTJ9_SENSOR)
	if(!isKxtj9Verify){
	    ret = kxtj9_verify();
	    if (ret) {
		printk("tsb_kxtj9 error detect.\n");
	    }
	    else
	    {
	  	isKxtj9Verify = true;
	    }
	}
#endif
	dev->sensor_id = sensor_id;
	dev->sensor_revision = sensor_revision;

	/* Resolution settings depend on sensor type and platform */
	ret = __update_tsb_device_settings(dev, dev->sensor_id);
	if (ret)
		goto fail_detect;

	/* Read sensor's OTP data */
	dev->otp_data = dev->otp_driver->otp_read(sd);

	/* power off sensor */
	ret = __tsb_s_power(sd, 0);

	mutex_unlock(&dev->input_lock);
	if (ret)
		v4l2_err(client, "tsb power-down err.\n");

	return ret;

fail_detect:
	dev->platform_data->csi_cfg(sd, 0);
fail_csi_cfg:
	__tsb_s_power(sd, 0);
	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int
tsb_enum_mbus_code(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_mbus_code_enum *code)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	/*printk("%s, start\n", __func__);*/

	if (code->index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	code->code = dev->format.code;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int
tsb_enum_frame_size(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
			struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
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

static struct v4l2_mbus_framefmt *
__tsb_get_pad_format(struct tsb_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	/*printk("%s, start\n", __func__);*/

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
tsb_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__tsb_get_pad_format(dev, fh, fmt->pad, fmt->which);

	/*printk("%s, start\n", __func__);*/

	fmt->format = *format;

	return 0;
}

static int
tsb_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		dev->format = fmt->format;

	return 0;
}

static int
tsb_s_parm(struct v4l2_subdev *sd, struct v4l2_streamparm *param)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;

	/*printk("%s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		dev->curr_res_table = dev->mode_tables->res_video;
		dev->entries_curr_table = dev->mode_tables->n_res_video;
		break;
	case CI_MODE_STILL_CAPTURE:
		dev->curr_res_table = dev->mode_tables->res_still;
		dev->entries_curr_table = dev->mode_tables->n_res_still;
		break;
	default:
		dev->curr_res_table = dev->mode_tables->res_preview;
		dev->entries_curr_table = dev->mode_tables->n_res_preview;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

int
tsb_g_frame_interval(struct v4l2_subdev *sd,
				struct v4l2_subdev_frame_interval *interval)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	const struct tsb_resolution *res =
				&dev->curr_res_table[dev->fmt_idx];

	/*printk("%s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	interval->interval.denominator = res->fps_options[dev->fps_index].fps;
	interval->interval.numerator = 1;
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int __tsb_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct tsb_resolution *res =
				&dev->curr_res_table[dev->fmt_idx];
	struct camera_mipi_info *tsb_info = NULL;
	unsigned short pixels_per_line;
	unsigned short lines_per_frame;
	unsigned int fps_index;
	int fps;
	int ret = 0;


	/*printk("%s, start\n", __func__);*/

	tsb_info = v4l2_get_subdev_hostdata(sd);
	if (tsb_info == NULL)
		return -EINVAL;

	if (!interval->interval.numerator)
		interval->interval.numerator = 1;

	fps = interval->interval.denominator / interval->interval.numerator;

	if (!fps)
		/* currently does not support fps format like 1/2, 1/3 */
		fps = 1;

	/* No need to proceed further if we are not streaming */
	if (!dev->streaming) {
		/* Save the new FPS and use it while selecting setting */
		dev->fps = fps;
		return 0;
	}

	 /* Ignore if we are already using the required FPS. */
	if (fps == dev->fps)
		return 0;

	/*
	 * Start here, sensor is already streaming, so adjust fps dynamically
	 */
	fps_index = __tsb_above_nearest_fps_index(fps, res->fps_options);

	if (res->fps_options[fps_index].regs &&
	    res->fps_options[fps_index].regs != dev->regs) {
		/*
		 * if need a new setting, but the new setting has difference
		 * with current setting, not use this one, as may have
		 * unexpected result, e.g. PLL, IQ.
		 */
		dev_dbg(&client->dev, "Sensor is streaming, not apply new sensor setting\n");
		if (fps > res->fps_options[dev->fps_index].fps) {
			/*
			 * Does not support increase fps based on low fps
			 * setting, as the high fps setting could not be used,
			 * and fps requested is above current setting fps.
			 */
			dev_warn(&client->dev, "Could not support fps: %d, keep current: %d.\n",
					fps, dev->fps);
			goto done;
		}
	} else {
		dev->fps_index = fps_index;
		dev->fps = res->fps_options[dev->fps_index].fps;
	}

	/* Update the new frametimings based on FPS */
	pixels_per_line = res->fps_options[dev->fps_index].pixels_per_line;
	lines_per_frame = res->fps_options[dev->fps_index].lines_per_frame;

	if (fps > res->fps_options[fps_index].fps) {
		/*
		 * if does not have high fps setting, not support increase fps
		 * by adjust lines per frame.
		 */
		dev_warn(&client->dev, "Could not support fps: %d. Use:%d.\n",
				fps, res->fps_options[fps_index].fps);
		goto update;
	}

	/* if the new setting does not match exactly */
	if (dev->fps != fps) {
#define MAX_LINES_PER_FRAME	0xffff
		dev_dbg(&client->dev, "adjusting fps using lines_per_frame\n");
		/*
		 * FIXME!
		 * 1: check DS on max value of lines_per_frame
		 * 2: consider use pixel per line for more range?
		 */
		if (dev->lines_per_frame * dev->fps / fps >
				MAX_LINES_PER_FRAME) {
			dev_warn(&client->dev,
					"adjust lines_per_frame out of range, try to use max value.\n");
			lines_per_frame = MAX_LINES_PER_FRAME;
		} else {
			lines_per_frame = lines_per_frame * dev->fps / fps;
		}
	}
update:
	/* Update the new frametimings based on FPS */
	dev->pixels_per_line = pixels_per_line;
	dev->lines_per_frame = lines_per_frame;

	/* Update the new values so that user side knows the current settings */
	ret = __tsb_update_exposure_timing(client,
		dev->coarse_itg, dev->pixels_per_line, dev->lines_per_frame);
	if (ret)
		return ret;

	dev->fps = fps;

	ret = tsb_get_intg_factor(client, tsb_info, dev->regs);
	if (ret)
		return ret;
done:
	interval->interval.denominator = res->fps_options[dev->fps_index].fps;
	interval->interval.numerator = 1;
	__tsb_print_timing(sd);

	return ret;
}

static int tsb_s_frame_interval(struct v4l2_subdev *sd,
			struct v4l2_subdev_frame_interval *interval)
{
	struct tsb_device *dev = to_tsb_sensor(sd);
	int ret;

	/*printk("%s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	ret = __tsb_s_frame_interval(sd, interval);
	mutex_unlock(&dev->input_lock);

	return ret;
}
static int tsb_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/

	mutex_lock(&dev->input_lock);
	*frames = dev->curr_res_table[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);
	printk("%s, skip_frames=%d\n", __func__, dev->curr_res_table[dev->fmt_idx].skip_frames);

	return 0;
}

static const struct v4l2_subdev_sensor_ops tsb_sensor_ops = {
	.g_skip_frames	= tsb_g_skip_frames,
};

static int tsb_set_ctrl(struct v4l2_ctrl *ctrl)
{
	/*printk("%s, start\n", __func__);*/

	return 0;
}

static int tsb_g_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct tsb_device *dev = container_of(ctrl->handler, struct tsb_device,
			ctrl_handler);

    printk("%s, start, ctrl id = %d\n", __func__, ctrl->id);

	switch (ctrl->id) {
	case V4L2_CID_VBLANK:
		ctrl->val = dev->lines_per_frame -
			dev->curr_res_table[dev->fmt_idx].height;
		break;
	case V4L2_CID_HBLANK:
		ctrl->val = dev->pixels_per_line -
			dev->curr_res_table[dev->fmt_idx].width;
		break;
	case V4L2_CID_PIXEL_RATE:
		ctrl->val = dev->vt_pix_clk_freq_mhz;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static struct v4l2_ctrl_ops tsb_ctrl_ops = {
	.s_ctrl = tsb_set_ctrl,
	.g_volatile_ctrl = tsb_g_volatile_ctrl,
};

static const struct v4l2_subdev_video_ops tsb_video_ops = {
	.s_stream = tsb_s_stream,
	.enum_framesizes = tsb_enum_framesizes,
	.enum_frameintervals = tsb_enum_frameintervals,
	.enum_mbus_fmt = tsb_enum_mbus_fmt,
	.try_mbus_fmt = tsb_try_mbus_fmt,
	.g_mbus_fmt = tsb_g_mbus_fmt,
	.s_mbus_fmt = tsb_s_mbus_fmt,
	.s_parm = tsb_s_parm,
	.g_frame_interval = tsb_g_frame_interval,
	.s_frame_interval = tsb_s_frame_interval,
};

static const struct v4l2_subdev_core_ops tsb_core_ops = {
	.g_chip_ident = tsb_g_chip_ident,
	.queryctrl = tsb_queryctrl,
	.g_ctrl = tsb_g_ctrl,
	.s_ctrl = tsb_s_ctrl,
	.s_power = tsb_s_power,
	.ioctl = tsb_ioctl,
	.init = tsb_init,
};

static const struct v4l2_subdev_pad_ops tsb_pad_ops = {
	.enum_mbus_code = tsb_enum_mbus_code,
	.enum_frame_size = tsb_enum_frame_size,
	.get_fmt = tsb_get_pad_format,
	.set_fmt = tsb_set_pad_format,
};

static const struct v4l2_subdev_ops tsb_ops = {
	.core = &tsb_core_ops,
	.video = &tsb_video_ops,
	.pad = &tsb_pad_ops,
	.sensor = &tsb_sensor_ops,
};

static const struct media_entity_operations tsb_entity_ops = {
	.link_setup = NULL,
};

static int tsb_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct tsb_device *dev = to_tsb_sensor(sd);

	/*printk("%s, start\n", __func__);*/

	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();

	media_entity_cleanup(&dev->sd.entity);
	v4l2_ctrl_handler_free(&dev->ctrl_handler);
	dev->platform_data->csi_cfg(sd, 0);
	v4l2_device_unregister_subdev(sd);
	release_msr_list(client, dev->fw);
	kfree(dev);

	remove_proc_entry("otp_t4k37", NULL);

	return 0;
}

static int __tsb_init_ctrl_handler(struct tsb_device *dev)
{
	struct v4l2_ctrl_handler *hdl;

	/*printk("%s, start\n", __func__);*/

	hdl = &dev->ctrl_handler;

	v4l2_ctrl_handler_init(&dev->ctrl_handler, 3);

	dev->pixel_rate = v4l2_ctrl_new_std(&dev->ctrl_handler,
					    &tsb_ctrl_ops,
					    V4L2_CID_PIXEL_RATE,
					    0, UINT_MAX, 1, 0);

	dev->h_blank = v4l2_ctrl_new_std(&dev->ctrl_handler,
					  &tsb_ctrl_ops,
					  V4L2_CID_HBLANK, 0, SHRT_MAX, 1, 0);

	dev->v_blank = v4l2_ctrl_new_std(&dev->ctrl_handler,
					  &tsb_ctrl_ops,
					  V4L2_CID_VBLANK, 0, SHRT_MAX, 1, 0);

	if (dev->ctrl_handler.error || dev->pixel_rate == NULL
		|| dev->h_blank == NULL || dev->v_blank == NULL) {
		return dev->ctrl_handler.error;
	}

	dev->sd.ctrl_handler = hdl;

	dev->pixel_rate->flags |= V4L2_CTRL_FLAG_VOLATILE;
	dev->h_blank->flags |= V4L2_CTRL_FLAG_VOLATILE;
	dev->v_blank->flags |= V4L2_CTRL_FLAG_VOLATILE;

	return 0;
}

static int tsb_otp_proc_show(struct seq_file *s, void *v)
{
       int i;

        for(i=0 ; i<24 ; i++)
        {
                seq_printf(s, "0x%X", tsb_otp_data[i]);
                if((i+1) % 8 != 0 && (i+1) != 24)
                        seq_printf(s, " ");
                else
                        seq_printf(s, "\n");
        }

       return 0;
}

static int tsb_proc_open(struct inode *inode, struct  file *file)
{
       return single_open(file, tsb_otp_proc_show, NULL);
}

static const struct file_operations otp_proc_fops = {
       .owner = THIS_MODULE,
       .open = tsb_proc_open,
       .read = seq_read,
       .llseek = seq_lseek,
       .release = single_release,
};

static ssize_t vcm_read(struct file *dev, char *buffer, size_t count, loff_t *ppos)
{

	int pos;
	int re;
	ssize_t ret = 0;
	char *buff;
	int len = 0;
	re = inner_vcm->q_focus_abs(&inner_tsb->sd, &pos);

	printk(KERN_INFO "[AsusVCM] Read VCM postion %d\n", last_vcm);
	buff = kmalloc(100,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;

	len += sprintf(buff+len, "%d\n", last_vcm);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
	kfree(buff);

	return ret;

}

static const struct file_operations vcm_proc_fops = {
       .owner = THIS_MODULE,
       .read = vcm_read,
};

static int tsb_probe(struct i2c_client *client,
			 const struct i2c_device_id *id)
{
	struct tsb_device *dev;
	struct camera_mipi_info *tsb_info = NULL;
	int ret;
	char *msr_file_name = NULL;
	struct proc_dir_entry* proc_entry_vcm;
	void* dummy = NULL;

    if(Read_PROJ_ID() == PROJ_ID_ZX550ML){
        printk(KERN_INFO "@%s, It is ZX project, but ZE expected for t4k37, break! \n", __func__);
        return -1;
    }
	printk("%s, start\n", __func__);

	/* allocate sensor device & init sub device */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		v4l2_err(client, "%s: out of memory\n", __func__);
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);

	dev->i2c_id = id->driver_data;
	dev->fmt_idx = 0;
	dev->sensor_id = 0;
	dev->vcm_driver = &tsb_vcm;
	dev->otp_driver = &tsb_otps;

	v4l2_i2c_subdev_init(&(dev->sd), client, &tsb_ops);

	if (client->dev.platform_data) {
		ret = tsb_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret)
			goto out_free;
	}
	tsb_info = v4l2_get_subdev_hostdata(&dev->sd);

	/*
	 * sd->name is updated with sensor driver name by the v4l2.
	 * change it to sensor name in this case.
	 */
	snprintf(dev->sd.name, sizeof(dev->sd.name), "%s%x %d-%04x",
		TSB_SUBDEV_PREFIX, dev->sensor_id,
		i2c_adapter_id(client->adapter), client->addr);

	ret = __tsb_init_ctrl_handler(dev);
	if (ret)
		goto out_ctrl_handler_free;

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = tsb_translate_bayer_order(
		tsb_info->raw_bayer_order);
	dev->sd.entity.ops = &tsb_entity_ops;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret) {
		tsb_remove(client);
		return ret;
	}

    proc_create("otp_t4k37", 0, NULL, &otp_proc_fops);

	proc_entry_vcm = proc_create_data("driver/vcm", 0666, NULL, &vcm_proc_fops, dummy);
	proc_set_user(proc_entry_vcm, 1000, 1000);
	inner_vcm = &tsb_vcm;
	inner_tsb = dev;
	/* Load the Noise reduction, Dead pixel registers from cpf file*/
	if (dev->platform_data->msr_file_name != NULL)
		msr_file_name = dev->platform_data->msr_file_name();
	if (msr_file_name) {
		ret = load_msr_list(client, msr_file_name, &dev->fw);
		if (ret) {
			tsb_remove(client);
			return ret;
		}
	} else {
		dev_warn(&client->dev, "Drvb file not present");
	}

	return ret;

out_ctrl_handler_free:
	v4l2_ctrl_handler_free(&dev->ctrl_handler);

out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}

static const struct i2c_device_id tsb_ids[] = {
#if 0	// TSB
	{IMX_NAME_175, IMX175_ID},
	{IMX_NAME_135, IMX135_ID},
	{IMX_NAME_135_FUJI, IMX135_FUJI_ID},
	{IMX_NAME_134, IMX134_ID},
	{IMX_NAME_132, IMX132_ID},
#else	// TSB
	{TSB_NAME_K37, T4K37_NAME_ID},
	/*{TSB_NAME_K35, T4K35_ID},*/
#endif	// TSB
	{}
};

MODULE_DEVICE_TABLE(i2c, tsb_ids);

static struct i2c_driver tsb_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = TSB_DRIVER,
	},
	.probe = tsb_probe,
	.remove = tsb_remove,
	.id_table = tsb_ids,
};


static __init int init_tsb(void)
{
	/*printk("%s, start\n", __func__);*/

	return i2c_add_driver(&tsb_driver);
}

static __exit void exit_tsb(void)
{
	/*printk("%s, start\n", __func__);*/

	i2c_del_driver(&tsb_driver);
}

module_init(init_tsb);
module_exit(exit_tsb);

MODULE_DESCRIPTION("A low-level driver for Toshiba sensors");
MODULE_AUTHOR("Shenbo Huang <shenbo.huang@intel.com>");
MODULE_LICENSE("GPL");

