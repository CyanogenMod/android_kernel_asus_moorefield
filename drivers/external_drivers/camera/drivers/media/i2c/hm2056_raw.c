/*
 * Support for HM2056_raw Camera Sensor.
 *
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>

#include "hm2056_raw.h"

#define to_hm2056_raw_sensor(sd) container_of(sd, struct hm2056_raw_device, sd)

/* Add for ATD command+++ */
/* Add build version -> user:3, userdebug:2, eng:1 */
/*extern int build_version;*/
struct v4l2_subdev *main_sd;

int ATD_hm2056_raw_status = 0;
static char camera_module_otp[60];

/* static void *hm2056_raw_otp_read(struct v4l2_subdev *sd); */

static ssize_t hm2056_raw_show_status(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	pr_info("%s: get hm2056_raw status (%d) !!\n",
				__func__, ATD_hm2056_raw_status);
	/* Check sensor connect status,
	just do it  in begining for ATD camera status */
	return sprintf(buf, "%d\n", ATD_hm2056_raw_status);
}

static ssize_t hm2056_raw_read_otp(struct device *dev,
				struct device_attribute *attr, char *buf)
{
	pr_info("%s: get hm2056 module OTP %s !!\n",
				 __func__, camera_module_otp);
	/*Check sensor OTP value,just do it in begining for ATD camera status */
/*
	if(build_version != 1){ //not eng, need to read otp first
		hm2056_raw_otp_read(main_sd);
	}
*/
	return sprintf(buf, "%s", camera_module_otp);
}

static DEVICE_ATTR(hm2056_raw_status, S_IRUGO, hm2056_raw_show_status, NULL);
static DEVICE_ATTR(hm2056_raw_read_otp, S_IRUGO, hm2056_raw_read_otp, NULL);

static struct attribute *hm2056_raw_attributes[] = {
	&dev_attr_hm2056_raw_status.attr,
	&dev_attr_hm2056_raw_read_otp.attr,
	NULL
};
/* Add for ATD command--- */

static int
hm2056_raw_read_reg(struct i2c_client *client,
			u16 data_length, u32 reg, u32 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[2];

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != HM2056_8BIT) {
		v4l2_err(client, "%s error, invalid data length\n", __func__);
		return -EINVAL;
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = MSG_LEN_OFFSET;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u16) (reg >> 8);
	data[1] = (u16) (reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err >= 0) {
		*val = data[0];
		return 0;
	}

	dev_err(&client->dev, "read from offset 0x%x error %d", reg, err);
	return err;
}

static int
hm2056_raw_write_reg(struct i2c_client *client,
			u16 data_length, u16 reg, u32 val)
{
	int num_msg;
	struct i2c_msg msg;
	unsigned char data[6] = {0};
	u16 *wreg;
	int retry = 0;

	if (!client->adapter) {
		v4l2_err(client, "%s error, no client->adapter\n", __func__);
		return -ENODEV;
	}

	if (data_length != HM2056_8BIT) {
		v4l2_err(client, "%s error, invalid data_length\n", __func__);
		return -EINVAL;
	}

	memset(&msg, 0, sizeof(msg));

again:
	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(u16) + data_length;
	msg.buf = data;

	/* high byte goes out first */
	wreg = (u16 *)data;
	*wreg = cpu_to_be16(reg);

	data[2] = (u8)(val);

	num_msg = i2c_transfer(client->adapter, &msg, 1);
	if (num_msg >= 0)
		return 0;

	dev_err(&client->dev, "write error: wrote 0x%x to offset 0x%x error %d",
		val, reg, num_msg);
	if (retry <= I2C_RETRY_COUNT) {
		dev_dbg(&client->dev, "retrying... %d", retry);
		retry++;
		msleep(20);
		goto again;
	}

	return num_msg;
}

/*
 * hm2056_raw_write_reg_array - write a list of hm2056_raw registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function write a list of registers.
 */
static int hm2056_raw_write_reg_array(struct i2c_client *client,
				const struct hm2056_reg *reglist)
{
	const struct hm2056_reg *next = reglist;
	struct hm2056_write_ctrl ctrl;
	int ret;

	ctrl.index = 0;
	for (; next->type != HM2056_TOK_TERM; next++) {
		switch (next->type & HM2056_TOK_MASK) {
		case HM2056_TOK_DELAY:
			msleep(next->val);
			break;
		default:
			ret = hm2056_raw_write_reg(client, next->type,
							next->reg, next->val);
			if (ret) {
				v4l2_err(client, "%s: write %x error, aborted\n",
					 __func__, next->reg);
				return ret;
			}
			break;
		}
	}
	return 0;
}

static int hm2056_raw_set_suspend(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	dev_info(&client->dev, "%s\n", __func__);

	ret = hm2056_raw_write_reg_array(client, hm2056_stream_off);
	if (ret)
		return -EINVAL;
	return 0;
}

static int hm2056_raw_set_streaming(struct v4l2_subdev *sd)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	dev_info(&client->dev, "%s\n", __func__);

	ret = hm2056_raw_write_reg_array(client, hm2056_stream_on);
	if (ret)
		return -EINVAL;

	return 0;
}

static int power_up(struct v4l2_subdev *sd)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	dev_info(&client->dev, "%s\n", __func__);

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "power_ctrl failed\n");
		goto fail_power;
	}

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret) {
		dev_err(&client->dev, "flisclk_ctrl failed\n");
		goto fail_clk;
	}
	msleep(20);
	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret)
		dev_err(&client->dev, "gpio_ctrl failed\n");

	msleep(50);

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
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	dev_info(&client->dev, "%s\n", __func__);

	if (NULL == dev->platform_data) {
		dev_err(&client->dev, "no camera_sensor_platform_data\n");
		return -ENODEV;
	}

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "gpio failed\n");

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	/*according to DS, 20ms is needed after power down*/
	msleep(20);

	return ret;
}

static int hm2056_raw_s_power(struct v4l2_subdev *sd, int power)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	int ret;

	dev_info(&client->dev, "%s on/off %d\n", __func__, power);

	mutex_lock(&dev->input_lock);
	if (power == 0) {
		ret = power_down(sd);
	} else {
		ret = power_up(sd);
		if (!ret)
			ret = hm2056_raw_write_reg_array(client, hm2056_init);
	}
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int hm2056_raw_to_res(struct v4l2_subdev *sd, u32 w, u32 h)
{
	int idx;
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	for (idx = 0; idx < dev->n_res; idx++) {
		if ((dev->hm2056_raw_res[idx].width == w) &&
		    (dev->hm2056_raw_res[idx].height == h))
			break;
	}

	/* No mode found */
	if (idx >= dev->n_res)
		return -1;
	return idx;


}
static int hm2056_raw_try_res(struct v4l2_subdev *sd, u32 *w, u32 *h)
{
	int idx;
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	/*
	 * The mode list is in ascending order. We're done as soon as
	 * we have found the first equal or bigger size.
	 */
	for (idx = 0; idx < dev->n_res; idx++) {
		if ((dev->hm2056_raw_res[idx].width >= *w) &&
		    (dev->hm2056_raw_res[idx].height >= *h))
			break;
	}

	/*
	 * If no mode was found, it means we can provide only a smaller size.
	 * Returning the biggest one available in this case.
	 */
	if (idx == dev->n_res)
		idx--;

	*w = dev->hm2056_raw_res[idx].width;
	*h = dev->hm2056_raw_res[idx].height;
	return 0;
}

static int hm2056_raw_try_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	int ret;

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = hm2056_raw_try_res(sd, &fmt->width, &fmt->height);
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int hm2056_raw_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct hm2056_raw_res_struct *res)
{
	struct atomisp_sensor_mode_data *buf = &info->data;

	dev_info(&client->dev, "%s\n", __func__);
	if (info == NULL || res == NULL)
		return -EINVAL;

	buf->vt_pix_clk_freq_mhz = res->pixel_clk;
	buf->frame_length_lines = res->frame_length_lines;
	buf->line_length_pck = res->line_length_pck;

	/* get integration time */
	buf->coarse_integration_time_min = HM2056_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
					HM2056_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = HM2056_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
					HM2056_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = HM2056_FINE_INTG_TIME_MIN;

	buf->output_width = res->width;
	buf->output_height = res->height;
	buf->read_mode = res->bin_mode;
	buf->binning_factor_x = res->bin_factor_x;
	buf->binning_factor_y = res->bin_factor_y;
	buf->crop_horizontal_start = res->horizontal_start;
	buf->crop_horizontal_end = res->horizontal_end;
	buf->crop_vertical_start = res->vertical_start;
	buf->crop_vertical_end = res->vertical_end;

	/*yunliang modified to avoid warnings*/
	dev_info(&client->dev,
	"%s h start %d end %d v start %d end %d frame_length_line %d line_length_pck %d read mode %d binning_x %d binning_y %d\n",
		__func__, buf->crop_horizontal_start, buf->crop_horizontal_end,
		buf->crop_vertical_start, buf->crop_vertical_end,
		buf->frame_length_lines, buf->line_length_pck,
		buf->read_mode,	buf->binning_factor_x, buf->binning_factor_y);

	return 0;
}

static int hm2056_raw_get_mbus_fmt(struct v4l2_subdev *sd,
				struct v4l2_mbus_framefmt *fmt)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (!fmt)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fmt->width = dev->hm2056_raw_res[dev->fmt_idx].width;
	fmt->height = dev->hm2056_raw_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SGRBG8_1X8;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int hm2056_raw_set_mbus_fmt(struct v4l2_subdev *sd,
			      struct v4l2_mbus_framefmt *fmt)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	struct camera_mipi_info *hm2056_raw_info = NULL;
	int ret;
	int index;
	u32 width;
	u32 height;

	if (!fmt)
		return -EINVAL;

	width =  fmt->width;
	height =  fmt->height;
	hm2056_raw_info = v4l2_get_subdev_hostdata(sd);

	if (hm2056_raw_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	hm2056_raw_try_res(sd, &width, &height);

	dev_info(&client->dev, "%s %d x %d => %d x %d\n",
			__func__, fmt->width, fmt->height, width, height);

	dev->fmt_idx = hm2056_raw_to_res(sd, width, height);
	if (dev->fmt_idx == -1) {
		dev->fmt_idx = 0;
		mutex_unlock(&dev->input_lock);
		dev_err(&client->dev, "%s no resolution found\n", __func__);
		return -EINVAL;
	}

	ret = hm2056_raw_write_reg_array(client,
			dev->hm2056_raw_res[dev->fmt_idx].regs);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	if (hm2056_raw_set_suspend(sd)) {
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	/*
	 * Marked current sensor res as being "used"
	 *
	 * REVISIT: We don't need to use an "used" field on each mode
	 * list entry to know which mode is selected. If this
	 * information is really necessary, how about to use a single
	 * variable on sensor dev struct?
	 */
	for (index = 0; index < dev->n_res; index++) {
		if ((width == dev->hm2056_raw_res[index].width) &&
		    (height == dev->hm2056_raw_res[index].height)) {
			dev->hm2056_raw_res[index].used = 1;
			continue;
		}
		dev->hm2056_raw_res[index].used = 0;
	}

	ret = hm2056_raw_get_intg_factor(client, hm2056_raw_info,
					&dev->hm2056_raw_res[dev->fmt_idx]);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		dev_err(&client->dev, "failed to get integration_factor\n");
		return -EINVAL;
	}

	fmt->width = width;
	fmt->height = height;
	fmt->code = V4L2_MBUS_FMT_SGRBG8_1X8;

	mutex_unlock(&dev->input_lock);
	return 0;
}

static int hm2056_raw_q_exposure(struct v4l2_subdev *sd, s32 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u32 integration_time_h = 0;
	u32 integration_time_l = 0;
	int ret = 0;

	ret = hm2056_raw_read_reg(client, HM2056_8BIT,
			HM2056_REG_INTEGRATION_TIME_H, &integration_time_h);
	if (ret) {
		v4l2_err(client,
			"%s: read HM2056_REG_INTEGRATION_TIME_H error %d\n",
			__func__, ret);
		return ret;
	}

	ret = hm2056_raw_read_reg(client, HM2056_8BIT,
			HM2056_REG_INTEGRATION_TIME_L, &integration_time_l);
	if (ret) {
		v4l2_err(client,
			"%s: read HM2056_REG_INTEGRATION_TIME_L error %d\n",
			__func__, ret);
		return ret;
	}

	*val = integration_time_l | (integration_time_h << 8);
	return 0;
}

static long hm2056_raw_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	u8 analog_gain = 0;
	u8 digital_gain = 0;
	u32 reg_val_l, reg_val_h;
	unsigned int frame_length_lines = 0;
	unsigned int real_gain = exposure->gain[0];

	/* dev_info(&client->dev, "%s(%d %d %d %d)\n", __func__,
	 exposure->integration_time[0], exposure->integration_time[1],
	 exposure->gain[0], exposure->gain[1]); */

	mutex_lock(&dev->input_lock);

	frame_length_lines =
			dev->hm2056_raw_res[dev->fmt_idx].frame_length_lines;

	if (frame_length_lines < exposure->integration_time[0]) {
		/* dev_info(&client->dev, "%s update frame_length_line %d "
		 "integration_time %d\n", __func__, frame_length_lines,
		 exposure->integration_time[0]); */
		frame_length_lines = exposure->integration_time[0] + 5;
	}

	reg_val_l = (frame_length_lines -
		dev->hm2056_raw_res[dev->fmt_idx].height) & 0xFF;
	reg_val_h = ((frame_length_lines -
		dev->hm2056_raw_res[dev->fmt_idx].height) & 0xFF00) >> 8;

	ret = hm2056_raw_write_reg(client, HM2056_8BIT,
				HM2056_REG_BLANKING_ROW_H, reg_val_h);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client,
			"%s: write HM2056_REG_BLANKING_ROW_H error %x\n",
			__func__, reg_val_h);
		return ret;
	}

	ret = hm2056_raw_write_reg(client, HM2056_8BIT,
				HM2056_REG_BLANKING_ROW_L, reg_val_l);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client,
			"%s: write HM2056_REG_BLANKING_ROW_L error %x\n",
			__func__, reg_val_l);
		return ret;
	}

	reg_val_h = (exposure->integration_time[0] & 0xFF00) >> 8;
	reg_val_l = exposure->integration_time[0] & 0xFF;

	ret = hm2056_raw_write_reg(client, HM2056_8BIT,
				HM2056_REG_INTEGRATION_TIME_H, reg_val_h);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client,
			"%s: write HM2056_REG_INTEGRATION_TIME_H error %x\n",
			__func__, reg_val_h);
		return ret;
	}

	ret = hm2056_raw_write_reg(client, HM2056_8BIT,
				HM2056_REG_INTEGRATION_TIME_L, reg_val_l);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client,
			"%s: write HM2056_REG_INTEGRATION_TIME_L error %x\n",
			__func__, reg_val_l);
		return ret;
	}

	/* DIT request the AG and DG can't more than 4x,
	  so the max gain value will be 4xAG*4xDG=16x */
	if (16 <= real_gain && real_gain < 32) {
		analog_gain = 0x0;
		digital_gain = real_gain * 4;
	} else if (32 <= real_gain && real_gain < 64) {
		analog_gain = 0x1;
		digital_gain = real_gain * 2;
	} else if (64 <= real_gain && real_gain <= 256) {
		analog_gain = 0x2;
		digital_gain = real_gain;
	} else
		v4l2_err(client, "unsupported gain value\n");

	ret = hm2056_raw_write_reg(client, HM2056_8BIT,
				 HM2056_REG_AGAIN, analog_gain);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2056_REG_AGAIN error %x\n",
					 __func__, analog_gain);
		return ret;
	}

	ret = hm2056_raw_write_reg(client, HM2056_8BIT,
					HM2056_REG_DGAIN, digital_gain);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2056_REG_DGAIN error %x\n",
			 __func__, digital_gain);
		return ret;
	}

	ret = hm2056_raw_write_reg(client, HM2056_8BIT,
					HM2056_REG_COMMAND_UPDATE, 1);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "%s: write HM2056_REG_COMMAND_UPDATE error\n",
			__func__);
		return ret;
	}
	mutex_unlock(&dev->input_lock);
	return ret;
}

static long hm2056_raw_ioctl(struct v4l2_subdev *sd,
				 unsigned int cmd, void *arg)
{
	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return hm2056_raw_s_exposure(sd, arg);
	default:
		return -EINVAL;
	}
	return 0;
}

static int hm2056_raw_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	*val = dev->hm2056_raw_res[dev->fmt_idx].bin_factor_x - 1;
	return 0;
}

static int hm2056_raw_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	*val = dev->hm2056_raw_res[dev->fmt_idx].bin_factor_y - 1;
	return 0;
}

static struct hm2056_raw_control hm2056_raw_controls[] = {
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
		.query = hm2056_raw_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = 2,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2056_raw_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = 2,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = hm2056_raw_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(hm2056_raw_controls))

static struct hm2056_raw_control *hm2056_raw_find_control(__u32 id)
{
	int i;
	for (i = 0; i < N_CONTROLS; i++) {
		if (hm2056_raw_controls[i].qc.id == id)
			return &hm2056_raw_controls[i];
	}
	return NULL;
}

static int hm2056_raw_detect(struct hm2056_raw_device *dev,
				struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u32 reg_h, reg_l;
	u32 chip_id;
	int ret;

	ATD_hm2056_raw_status = 0;	/* Add for ATD command+++ */

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C)) {
		dev_err(&client->dev, "%s: i2c error", __func__);
		return -ENODEV;
	}

	ret = hm2056_raw_read_reg(client, HM2056_8BIT,
				HM2056_REG_CHIP_ID_H, &reg_h);
	if (ret) {
		v4l2_err(client, "%s: fail to read HM2056_REG_CHIP_ID_H\n",
				__func__);
		return -EINVAL;
	}

	ret = hm2056_raw_read_reg(client, HM2056_8BIT,
				HM2056_REG_CHIP_ID_L, &reg_l);
	if (ret) {
		v4l2_err(client, "%s: fail to read HM2056_REG_CHIP_ID_L\n",
			 __func__);
		return -EINVAL;
	}

	chip_id = reg_l | (reg_h << 8);

	if (chip_id != HM2056_MOD_ID) {
		dev_err(&client->dev,
			"%s: failed: client->addr = %x, id read %x\n",
			__func__, client->addr, chip_id);
		return -ENODEV;
	} else {
		dev_info(&client->dev, "%s sensor ID is 0x%x\n",
						__func__, chip_id);
	}

	ATD_hm2056_raw_status = 1;	/* Add for ATD command+++ */

	return 0;
}

static int
hm2056_raw_s_config(struct v4l2_subdev *sd, int irq, void *platform_data)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	if (NULL == platform_data)
		return -ENODEV;

	dev->platform_data =
	    (struct camera_sensor_platform_data *)platform_data;

	mutex_lock(&dev->input_lock);
	if (dev->platform_data->platform_init) {
		ret = dev->platform_data->platform_init(client);
		if (ret) {
			mutex_unlock(&dev->input_lock);
			v4l2_err(client, "hm2056_raw platform init err\n");
			return ret;
		}
	}
	ret = power_up(sd);
	if (ret) {
		v4l2_err(client, "hm2056_raw power-up err");
		/* goto fail_detect; */
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret) {
		v4l2_err(client, "hm2056_raw config csi err");
		goto fail_csi_cfg;
	}

	/* config & detect sensor */
	ret = hm2056_raw_detect(dev, client);
	if (ret) {
		v4l2_err(client, "hm2056_raw_detect err s_config.\n");
		/* goto fail_detect; */
	}

	ret = power_down(sd);
	if (ret) {
		mutex_unlock(&dev->input_lock);
		v4l2_err(client, "hm2056_raw power down err");
		return ret;
	}

	mutex_unlock(&dev->input_lock);
	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
/* fail_detect: */
	power_down(sd);
	mutex_unlock(&dev->input_lock);
	dev_err(&client->dev, "sensor power-gating failed\n");
	return ret;
}

static int hm2056_raw_queryctrl(struct v4l2_subdev *sd,
				struct v4l2_queryctrl *qc)
{
	struct hm2056_raw_control *ctrl = hm2056_raw_find_control(qc->id);
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int hm2056_raw_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (!param)
		return -EINVAL;

	dev->run_mode = param->parm.capture.capturemode;

	mutex_lock(&dev->input_lock);

	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		dev->hm2056_raw_res = hm2056_raw_res_video;
		dev->n_res = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		dev->hm2056_raw_res = hm2056_raw_res_still;
		dev->n_res = N_RES_STILL;
		break;
	default:
		dev->hm2056_raw_res = hm2056_raw_res_preview;
		dev->n_res = N_RES_PREVIEW;
	}

	dev->fmt_idx = 0;

	mutex_unlock(&dev->input_lock);

	return 0;
}

static int hm2056_raw_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2056_raw_control *octrl = hm2056_raw_find_control(ctrl->id);
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	int ret;

	if (octrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2056_raw_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct hm2056_raw_control *octrl = hm2056_raw_find_control(ctrl->id);
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);
	int ret;

	if (!octrl || !octrl->tweak)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int hm2056_raw_s_stream(struct v4l2_subdev *sd, int enable)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	dev_info(&client->dev, "%s enable %d\n", __func__, enable);

	mutex_lock(&dev->input_lock);
	if (enable)
		ret = hm2056_raw_set_streaming(sd);
	else
		ret = hm2056_raw_set_suspend(sd);

	mutex_unlock(&dev->input_lock);

	return ret;
}

static int
hm2056_raw_enum_framesizes(struct v4l2_subdev *sd,
				struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (index >= dev->n_res)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = dev->hm2056_raw_res[index].width;
	fsize->discrete.height = dev->hm2056_raw_res[index].height;
	fsize->reserved[0] = dev->hm2056_raw_res[index].used;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static int hm2056_raw_enum_frameintervals(struct v4l2_subdev *sd,
				       struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;
	int i;
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (index >= dev->n_res)
		return -EINVAL;

	mutex_lock(&dev->input_lock);

	/* find out the first equal or bigger size */
	for (i = 0; i < dev->n_res; i++) {
		if ((dev->hm2056_raw_res[i].width >= fival->width) &&
		    (dev->hm2056_raw_res[i].height >= fival->height))
			break;
	}
	if (i == dev->n_res)
		i--;

	index = i;
	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = dev->hm2056_raw_res[index].fps;

	mutex_unlock(&dev->input_lock);
	return 0;
}

static int
hm2056_raw_g_chip_ident(struct v4l2_subdev *sd,
			struct v4l2_dbg_chip_ident *chip)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	return v4l2_chip_ident_i2c_client(client, chip,
						V4L2_IDENT_HM2056_RAW, 0);
}

static int hm2056_raw_enum_mbus_code(struct v4l2_subdev *sd,
				  struct v4l2_subdev_fh *fh,
				  struct v4l2_subdev_mbus_code_enum *code)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (!code || code->index >= MAX_FMTS)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	code->code = V4L2_MBUS_FMT_SGRBG8_1X8;
	mutex_unlock(&dev->input_lock);

	return code->code < 0 ? code->code : 0;
}

static int hm2056_raw_enum_frame_size(struct v4l2_subdev *sd,
	struct v4l2_subdev_fh *fh,
	struct v4l2_subdev_frame_size_enum *fse)
{
	unsigned int index;
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (!fse)
		return -EINVAL;

	index = fse->index;

	if (index >= dev->n_res)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	fse->min_width = dev->hm2056_raw_res[index].width;
	fse->min_height = dev->hm2056_raw_res[index].height;
	fse->max_width = dev->hm2056_raw_res[index].width;
	fse->max_height = dev->hm2056_raw_res[index].height;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static struct v4l2_mbus_framefmt *
__hm2056_raw_get_pad_format(struct hm2056_raw_device *sensor,
			 struct v4l2_subdev_fh *fh, unsigned int pad,
			 enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,  "%s err. pad %x\n", __func__, pad);
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
hm2056_raw_get_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct hm2056_raw_device *snr = to_hm2056_raw_sensor(sd);
	struct v4l2_mbus_framefmt *format;

	if (!fmt)
		return -EINVAL;

	format = __hm2056_raw_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	fmt->format = *format;

	return 0;
}

static int
hm2056_raw_set_pad_format(struct v4l2_subdev *sd, struct v4l2_subdev_fh *fh,
		       struct v4l2_subdev_format *fmt)
{
	struct hm2056_raw_device *snr = to_hm2056_raw_sensor(sd);
	struct v4l2_mbus_framefmt *format;

	if (!fmt)
		return -EINVAL;

	format = __hm2056_raw_get_pad_format(snr, fh, fmt->pad, fmt->which);

	if (format == NULL)
		return -EINVAL;

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int hm2056_raw_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct hm2056_raw_device *dev = to_hm2056_raw_sensor(sd);

	if (frames == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*frames = dev->hm2056_raw_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}
static const struct v4l2_subdev_video_ops hm2056_raw_video_ops = {
	.s_parm = hm2056_raw_s_parm,
	.try_mbus_fmt = hm2056_raw_try_mbus_fmt,
	.s_mbus_fmt = hm2056_raw_set_mbus_fmt,
	.g_mbus_fmt = hm2056_raw_get_mbus_fmt,
	.s_stream = hm2056_raw_s_stream,
	.enum_framesizes = hm2056_raw_enum_framesizes,
	.enum_frameintervals = hm2056_raw_enum_frameintervals,
};

static struct v4l2_subdev_sensor_ops hm2056_raw_sensor_ops = {
	.g_skip_frames	= hm2056_raw_g_skip_frames,
};

static const struct v4l2_subdev_core_ops hm2056_raw_core_ops = {
	.g_chip_ident = hm2056_raw_g_chip_ident,
	.queryctrl = hm2056_raw_queryctrl,
	.g_ctrl = hm2056_raw_g_ctrl,
	.s_ctrl = hm2056_raw_s_ctrl,
	.s_power = hm2056_raw_s_power,
	.ioctl = hm2056_raw_ioctl,
};

/* REVISIT: Do we need pad operations? */
static const struct v4l2_subdev_pad_ops hm2056_raw_pad_ops = {
	.enum_mbus_code = hm2056_raw_enum_mbus_code,
	.enum_frame_size = hm2056_raw_enum_frame_size,
	.get_fmt = hm2056_raw_get_pad_format,
	.set_fmt = hm2056_raw_set_pad_format,
};

static const struct v4l2_subdev_ops hm2056_raw_ops = {
	.core = &hm2056_raw_core_ops,
	.video = &hm2056_raw_video_ops,
	.pad = &hm2056_raw_pad_ops,
	.sensor = &hm2056_raw_sensor_ops,
};

static int hm2056_raw_remove(struct i2c_client *client)
{
	struct hm2056_raw_device *dev;
	struct v4l2_subdev *sd = i2c_get_clientdata(client);

	dev = container_of(sd, struct hm2056_raw_device, sd);

	dev->platform_data->csi_cfg(sd, 0);

	if (dev->platform_data->platform_deinit)
		dev->platform_data->platform_deinit();
	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);
	return 0;
}

static int hm2056_raw_probe(struct i2c_client *client,
		       const struct i2c_device_id *id)
{
	struct hm2056_raw_device *dev;
	int ret;

	/* Setup sensor configuration structure */
	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}

	mutex_init(&dev->input_lock);
	dev->fmt_idx = 0;
	dev->hm2056_raw_res = hm2056_raw_res_preview;
	dev->n_res = N_RES_STILL;

	/* Add for ATD command+++ */
	dev->sensor_i2c_attribute.attrs = hm2056_raw_attributes;

	/* Register sysfs hooks */
	ret = sysfs_create_group(&client->dev.kobj, &dev->sensor_i2c_attribute);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	/* Add for ATD command--- */

	v4l2_i2c_subdev_init(&dev->sd, client, &hm2056_raw_ops);

	if (client->dev.platform_data) {
		ret = hm2056_raw_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret) {
			v4l2_device_unregister_subdev(&dev->sd);
			kfree(dev);
			return ret;
		}
	}

	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_SGRBG8_1X8;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);
	if (ret)
		hm2056_raw_remove(client);

	main_sd = &dev->sd;	/* Add for ATD command+++ */

	return ret;
}

MODULE_DEVICE_TABLE(i2c, hm2056_raw_id);

static struct i2c_driver hm2056_raw_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = HM2056_NAME,
	},
	.probe = hm2056_raw_probe,
	.remove = hm2056_raw_remove,
	.id_table = hm2056_raw_id,
};

static __init int init_hm2056_raw(void)
{
	return i2c_add_driver(&hm2056_raw_driver);
}

static __exit void exit_hm2056_raw(void)
{
	i2c_del_driver(&hm2056_raw_driver);
}

module_init(init_hm2056_raw);
module_exit(exit_hm2056_raw);

MODULE_AUTHOR("Hayden Huang <hayden.huang@intel.com>");
MODULE_DESCRIPTION("A low-level driver for Himax HM2056 sensors");
MODULE_LICENSE("GPL");
