/*
 * Support for OmniVision OV5670 5M camera sensor.
 * Based on OmniVision OV2722 driver.
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
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/gpio.h>
#include <linux/moduleparam.h>
#include <media/v4l2-device.h>
#include <media/v4l2-chip-ident.h>
#include <linux/io.h>
#include "ov5670.h"
//#include "decode_lib.h"

#define OV5670_DEBUG_EN 0
#define ov5670_debug dev_err



static struct i2c_client* client_local = NULL;

extern int ov5670_update_awb_gain(struct v4l2_subdev *sd);
u8 ov5670_otp_data[24];		//store otp data
int ATD_ov5670_status = 0;		//camera state, 1:ok 0:start_err
static int binning_sum;
//Add for ATD read camera status
static ssize_t ov5670_show_status(struct device *dev,struct device_attribute *attr,char *buf)
{
	return sprintf(buf,"%d\n", ATD_ov5670_status);//Check sensor connect status, just do it in begining for ATD camera status
}

/* i2c read/write stuff */
static int ov5670_read_reg(struct i2c_client *client,
			   u16 data_length, u16 reg, u16 *val)
{
	int err;
	struct i2c_msg msg[2];
	unsigned char data[6];

    if(client_local==NULL)
        client_local = client;

	if (!client->adapter) {
		dev_err(&client->dev, "%s error, no client->adapter\n",
			__func__);
		return -ENODEV;
	}

    if(client_local==NULL)
        client_local = client;

	if (data_length != OV5670_8BIT && data_length != OV5670_16BIT
					&& data_length != OV5670_32BIT) {
		dev_err(&client->dev, "%s error, invalid data length\n",
			__func__);
		return -EINVAL;
	}

	memset(msg, 0 , sizeof(msg));

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = I2C_MSG_LENGTH;
	msg[0].buf = data;

	/* high byte goes out first */
	data[0] = (u8)(reg >> 8);
	data[1] = (u8)(reg & 0xff);

	msg[1].addr = client->addr;
	msg[1].len = data_length;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = data;

	err = i2c_transfer(client->adapter, msg, 2);
	if (err != 2) {
		if (err >= 0)
			err = -EIO;
		dev_err(&client->dev,
			"read from offset 0x%x error %d", reg, err);
		return err;
	}

	*val = 0;
	/* high byte comes first */
	if (data_length == OV5670_8BIT)
		*val = (u8)data[0];
	else if (data_length == OV5670_16BIT)
		*val = be16_to_cpu(*(u16 *)&data[0]);
	else
		*val = be32_to_cpu(*(u32 *)&data[0]);

	return 0;
}

static int ov5670_i2c_write(struct i2c_client *client, u16 len, u8 *data)
{
	struct i2c_msg msg;
	const int num_msg = 1;
	int ret;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = len;
	msg.buf = data;
	ret = i2c_transfer(client->adapter, &msg, 1);

	return ret == num_msg ? 0 : -EIO;
}

static int ov5670_write_reg(struct i2c_client *client, u16 data_length,
							u16 reg, u16 val)
{
	int ret;
	unsigned char data[4] = {0};
    u16 *wreg = (u16 *)data;
    const u16 len = data_length + sizeof(u16); /* 16-bit address + data */
	if(client_local==NULL)
        client_local = client;
    if (data_length != OV5670_8BIT && data_length != OV5670_16BIT) {
        dev_err(&client->dev,
                "%s error, invalid data_length\n", __func__);
        return -EINVAL;
    }

    /* high byte goes out first */
    *wreg = cpu_to_be16(reg);

    if (data_length == OV5670_8BIT) {
        data[2] = (u8)(val);
    } else {
        /* OV5670_16BIT */
        u16 *wdata = (u16 *)&data[2];
        *wdata = cpu_to_be16(val);
    }

    ret = ov5670_i2c_write(client, len, data);
    if (ret)
        dev_err(&client->dev,
                "write error: wrote 0x%x to offset 0x%x error %d",
                val, reg, ret);

    return ret;
}

static struct class* ov5670_userCtrl_class;
static struct device* ov5670_register_ctrl_dev;

static ssize_t ov5670_write_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf){

    return 0;
}


static ssize_t ov5670_write_reg_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count){


    int reg_store;
    u16 reg_addr, reg_val;
    reg_store = -1;
    sscanf(buf, "%x", &reg_store);
    reg_val = reg_store&0xFF;
    reg_addr = (reg_store &0xFFFF00)>>8;
    ov5670_write_reg(client_local, OV5670_8BIT, reg_addr, reg_val);

    printk("Write regisetr address in ov5670: 0x%x, register value: 0x%x\n", reg_addr, reg_val);
    return count;
}


DEVICE_ATTR(write_reg, 0664, ov5670_write_reg_show, ov5670_write_reg_store);
static int ctrl_read_reg_addr ;
static ssize_t ov5670_read_reg_show(struct device *dev,
        struct device_attribute *attr, char *buf){

    int ret;
    u16 reg_val;
    ov5670_read_reg(client_local, OV5670_8BIT, ctrl_read_reg_addr, &reg_val);
    ret = sprintf(buf, "ov5670_read_reg_show Regisetr address in ov5670: 0x%x, register value: 0x%x\n", ctrl_read_reg_addr, reg_val);
    return ret;
}


static ssize_t ov5670_read_reg_store(struct device *dev,
        struct device_attribute *attr,
        const char *buf, size_t count){

    u16 reg_val = 0;

    sscanf(buf, "%x", &ctrl_read_reg_addr);
    printk("Read Register address: 0x%x\n", ctrl_read_reg_addr) ;

    //    ov5670_write_reg(client_local, OV5670_8BIT, reg_addr, reg_value);
    ov5670_read_reg(client_local, OV5670_8BIT, ctrl_read_reg_addr, &reg_val);
    //    ret = sprintf(buf, "Regisetr address in ov5670: 0x%x, register value: 0x%x\n", reg_addr, reg_val);
    printk("Read Regisetr address in ov5670: 0x%x, register value: 0x%x\n", ctrl_read_reg_addr, reg_val);

    return count;
}
DEVICE_ATTR(read_reg, 0664, ov5670_read_reg_show, ov5670_read_reg_store);


/*
 * ov5670_write_reg_array - Initializes a list of OV5670 registers
 * @client: i2c driver client structure
 * @reglist: list of registers to be written
 *
 * This function initializes a list of registers. When consecutive addresses
 * are found in a row on the list, this function creates a buffer and sends
 * consecutive data in a single i2c_transfer().
 *
 * __ov5670_flush_reg_array, __ov5670_buf_reg_array() and
 * __ov5670_write_reg_is_consecutive() are internal functions to
 * ov5670_write_reg_array_fast() and should be not used anywhere else.
 *
 */

static int __ov5670_flush_reg_array(struct i2c_client *client,
				    struct ov5670_write_ctrl *ctrl)
{
	u16 size;

	if (ctrl->index == 0)
		return 0;

	size = sizeof(u16) + ctrl->index; /* 16-bit address + data */
	ctrl->buffer.addr = cpu_to_be16(ctrl->buffer.addr);
	ctrl->index = 0;

	return ov5670_i2c_write(client, size, (u8 *)&ctrl->buffer);
}

static int __ov5670_buf_reg_array(struct i2c_client *client,
				  struct ov5670_write_ctrl *ctrl,
				  const struct ov5670_reg *next)
{
	int size;
	u16 *data16;

	switch (next->type) {
	case OV5670_8BIT:
		size = 1;
		ctrl->buffer.data[ctrl->index] = (u8)next->val;
		break;
	case OV5670_16BIT:
		size = 2;
		data16 = (u16 *)&ctrl->buffer.data[ctrl->index];
		*data16 = cpu_to_be16((u16)next->val);
		break;
	default:
		return -EINVAL;
	}

	/* When first item is added, we need to store its starting address */
	if (ctrl->index == 0)
		ctrl->buffer.addr = next->reg;

	ctrl->index += size;

	/*
	 * Buffer cannot guarantee free space for u32? Better flush it to avoid
	 * possible lack of memory for next item.
	 */
	if (ctrl->index + sizeof(u16) >= OV5670_MAX_WRITE_BUF_SIZE)
		return __ov5670_flush_reg_array(client, ctrl);

	return 0;
}

static int __ov5670_write_reg_is_consecutive(struct i2c_client *client,
					     struct ov5670_write_ctrl *ctrl,
					     const struct ov5670_reg *next)
{
	if (ctrl->index == 0)
		return 1;

	return ctrl->buffer.addr + ctrl->index == next->reg;
}

static int ov5670_write_reg_array(struct i2c_client *client,
				  const struct ov5670_reg *reglist)
{
	const struct ov5670_reg *next = reglist;
	struct ov5670_write_ctrl ctrl;
	int err;

	ctrl.index = 0;
	for (; next->type != OV5670_TOK_TERM; next++) {
		switch (next->type & OV5670_TOK_MASK) {
		case OV5670_TOK_DELAY:
			err = __ov5670_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			msleep(next->val);
			break;
		default:
			/*
			 * If next address is not consecutive, data needs to be
			 * flushed before proceed.
			 */
			if (!__ov5670_write_reg_is_consecutive(client, &ctrl,
								next)) {
				err = __ov5670_flush_reg_array(client, &ctrl);
			if (err)
				return err;
			}
			err = __ov5670_buf_reg_array(client, &ctrl, next);
			if (err) {
				dev_err(&client->dev, "%s: write error, aborted\n",
					 __func__);
				return err;
			}
			break;
		}
	}

	return __ov5670_flush_reg_array(client, &ctrl);
}
static int ov5670_g_focal(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV5670_FOCAL_LENGTH_NUM << 16) | OV5670_FOCAL_LENGTH_DEM;
	return 0;
}

static int ov5670_g_fnumber(struct v4l2_subdev *sd, s32 *val)
{
	/*const f number for imx*/
	*val = (OV5670_F_NUMBER_DEFAULT_NUM << 16) | OV5670_F_NUMBER_DEM;
	return 0;
}

static int ov5670_g_fnumber_range(struct v4l2_subdev *sd, s32 *val)
{
	*val = (OV5670_F_NUMBER_DEFAULT_NUM << 24) |
		(OV5670_F_NUMBER_DEM << 16) |
		(OV5670_F_NUMBER_DEFAULT_NUM << 8) | OV5670_F_NUMBER_DEM;
	return 0;
}

static int ov5670_g_bin_factor_x(struct v4l2_subdev *sd, s32 *val)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);

	*val = ov5670_res[dev->fmt_idx].bin_factor_x;

	return 0;
}

static int ov5670_g_bin_factor_y(struct v4l2_subdev *sd, s32 *val)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);

	*val = ov5670_res[dev->fmt_idx].bin_factor_y;

	return 0;
}

static int ov5670_get_intg_factor(struct i2c_client *client,
				struct camera_mipi_info *info,
				const struct ov5670_resolution *res)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct atomisp_sensor_mode_data *buf = &info->data;
	unsigned int pix_clk_freq_hz;
	u16 reg_val;
    int offset;
	int ret;

	if (info == NULL)
		return -EINVAL;

	/* pixel clock */
	pix_clk_freq_hz = res->pix_clk_freq * 1000000;

	dev->vt_pix_clk_freq_mhz = pix_clk_freq_hz;
	buf->vt_pix_clk_freq_mhz = pix_clk_freq_hz;

	/* get integration time */
	buf->coarse_integration_time_min = OV5670_COARSE_INTG_TIME_MIN;
	buf->coarse_integration_time_max_margin =
					OV5670_COARSE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_min = OV5670_FINE_INTG_TIME_MIN;
	buf->fine_integration_time_max_margin =
					OV5670_FINE_INTG_TIME_MAX_MARGIN;

	buf->fine_integration_time_def = OV5670_FINE_INTG_TIME_MIN;
	buf->frame_length_lines = res->lines_per_frame;
	buf->line_length_pck = res->pixels_per_line;
	buf->read_mode = res->bin_mode;

	/* get the cropping and output resolution to ISP for this mode. */
	buf->binning_factor_x = res->bin_factor_x + 1;
	buf->binning_factor_y = res->bin_factor_y + 1;

	ret = ov5670_read_reg(client, OV5670_16BIT,
					OV5670_HORIZONTAL_OUTPUT_SIZE_H, &reg_val);
	if (ret)
		return ret;
	buf->output_width = reg_val;

	ret = ov5670_read_reg(client, OV5670_16BIT,
					OV5670_VERTICAL_OUTPUT_SIZE_H, &reg_val);
	if (ret)
		return ret;
	buf->output_height = reg_val;

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
    /*horizontal_offset = (OV5670_MAX_WIDTH - (int)(buf->output_width * buf->binning_factor_x)) / 2;*/
    /*if(horizontal_offset < 0)*/
        /*horizontal_offset = 0;*/
    /*else if(horizontal_offset%2 == 1)*/
        /*horizontal_offset -= 1;*/
    /*vertical_offset = (OV5670_MAX_HEIGHT - (int)(buf->output_height * buf->binning_factor_y)) / 2;*/
    /*if(vertical_offset < 0)*/
        /*vertical_offset = 0;*/
    /*else if(vertical_offset%2 == 1)*/
        /*vertical_offset -= 1;*/

    /*buf->crop_horizontal_start = horizontal_offset;*/
    /*buf->crop_horizontal_end = OV5670_MAX_WIDTH - horizontal_offset - 1;*/
    /*buf->crop_vertical_start = vertical_offset;*/
    /*buf->crop_vertical_end = OV5670_MAX_HEIGHT - vertical_offset - 1;*/

    printk("@%s, binning_factor %dx%d, crop %d~%d x %d~%d, size %dx%d\n", __func__, buf->binning_factor_x, buf->binning_factor_y, buf->crop_horizontal_start, buf->crop_horizontal_end, buf->crop_vertical_start, buf->crop_vertical_end, buf->output_width, buf->output_height);
	return 0;
}

static long __ov5670_set_exposure(struct v4l2_subdev *sd, int coarse_itg,
				 int gain, int digitgain)

{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	u16 vts,hts;
	u8 reg,reg_5045;
	int ret,exp_val,vts_val;

  //printk(KERN_ERR "__ov5670_set_exposure coarse_itg %d, gain %d, digitgain %d++\n",coarse_itg, gain, digitgain);

  printk(KERN_INFO "__ov5670_set_exposure coarse_itg %d, gain %d, digitgain %d++\n",coarse_itg, gain, digitgain);


	if (dev->run_mode == CI_MODE_VIDEO)
		ov5670_res = ov5670_res_video;
	else if (dev->run_mode == CI_MODE_STILL_CAPTURE)
		ov5670_res = ov5670_res_still;
	else
		ov5670_res = ov5670_res_preview;


	hts = ov5670_res[dev->fmt_idx].pixels_per_line;
	vts = ov5670_res[dev->fmt_idx].lines_per_frame;

	ret = ov5670_read_reg(client, OV5670_8BIT,
					0x4003,
					(u16*)&reg);
	//printk(KERN_ALERT"ASUSov5670 -----0x4003 = %x\n",reg);
	if(ret)
		return ret;
	ret = ov5670_read_reg(client, OV5670_8BIT,
					0x5045,
					(u16*)&reg_5045);
	reg_5045 = reg_5045|0x04;
//	printk(KERN_ALERT"ASUSov5670 -----0x5045 = %x\n",reg_5045);
	if(ret)
		return ret;

	ret = ov5670_write_reg(client, OV5670_8BIT,
			       0x3209, 0x00);
	if (ret)
		return ret;

	ret = ov5670_write_reg(client, OV5670_8BIT,
			       0x320a, 0x01);
	if (ret)
		return ret;

	/* group hold */
	ret = ov5670_write_reg(client, OV5670_8BIT,
			       OV5670_GROUP_ACCESS, 0x00);
	if (ret)
		return ret;

	  if(gain >= 1*128 && gain < 2*128){
		  printk(KERN_ALERT"ASUSov5670 0x366A=0x00");
		  ov5670_write_reg(client, OV5670_8BIT, 0x366A, 0x00);
	  }else if(gain >= 2*128 && gain < 4*128){
		  printk(KERN_ALERT"ASUSov5670 0x366A=0x01");
		  ov5670_write_reg(client, OV5670_8BIT, 0x366A, 0x01);
	  }else if(gain >= 4*128 && gain < 8*128){
		  printk(KERN_ALERT"ASUSov5670 0x366A=0x03");
		  ov5670_write_reg(client, OV5670_8BIT, 0x366A, 0x03);
	  }else if(gain >= 8*128){
		  printk(KERN_ALERT"ASUSov5670 0x366A=0x07");
		  ov5670_write_reg(client, OV5670_8BIT, 0x366A, 0x07);
	  }

	#if 1
	//add for MWB
	ret = ov5670_write_reg(client, OV5670_8BIT, 0x5045, reg_5045);
	if(ret)
		return ret;
	ret = ov5670_write_reg(client, OV5670_8BIT, 0x5048, reg);
	if(ret)
		return ret;
	/* Digital gain */
//	printk(KERN_ALERT "ASUSov5670-----------D gain = %d\n",digitgain);
	if (digitgain) {
		if (digitgain > 128)
			digitgain = 128;

		ret = ov5670_write_reg(client, OV5670_16BIT,
				OV5670_MWB_RED_GAIN_H, digitgain<<4);
		if (ret)
			return ret;

		ret = ov5670_write_reg(client, OV5670_16BIT,
				OV5670_MWB_GREEN_GAIN_H, digitgain<<4);
		if (ret)
			return ret;

		ret = ov5670_write_reg(client, OV5670_16BIT,
				OV5670_MWB_BLUE_GAIN_H, digitgain<<4);
		if (ret)
			return ret;
	}
	// add for MWB end.
	#endif


	/* End group */
	ret = ov5670_write_reg(client, OV5670_8BIT,
			       OV5670_GROUP_ACCESS, 0x10);
	if (ret)
		return ret;

	/* group hold */
	ret = ov5670_write_reg(client, OV5670_8BIT,
			       OV5670_GROUP_ACCESS, 0x01);
	if (ret)
		return ret;

	/* Increase the VTS to match exposure + 8 */
	if (coarse_itg + OV5670_INTEGRATION_TIME_MARGIN > vts)
		vts_val = coarse_itg + OV5670_INTEGRATION_TIME_MARGIN;
	else
		vts_val = vts;
	{

		//printk(KERN_ERR "__ov5670_set_exposure  0x%x,0x%x,0x%x,++\n",vts_val,(vts_val & 0xFF),((vts_val >> 8) & 0xFF));

	printk(KERN_ERR "__ov5670_set_exposure  0x%x,0x%x,0x%x,++\n",vts_val,(vts_val & 0xFF),((vts_val >> 8) & 0xFF));

		ret = ov5670_write_reg(client, OV5670_8BIT, OV5670_TIMING_VTS_H, (vts_val >> 8) & 0xFF);
		if (ret)
			return ret;
		ret = ov5670_write_reg(client, OV5670_8BIT, OV5670_TIMING_VTS_L, vts_val & 0xFF);
		if (ret)
			return ret;
	}

	/* set exposure */
	/* Lower four bit should be 0*/
	exp_val = coarse_itg << 4;

	ret = ov5670_write_reg(client, OV5670_8BIT,
			       OV5670_EXPOSURE_L, exp_val & 0xFF);
	if (ret)
		return ret;

	ret = ov5670_write_reg(client, OV5670_8BIT,
			       OV5670_EXPOSURE_M, (exp_val >> 8) & 0xFF);
	if (ret)
		return ret;

	ret = ov5670_write_reg(client, OV5670_8BIT,
			       OV5670_EXPOSURE_H, (exp_val >> 16) & 0x0F);
	if (ret)
		return ret;

    /* Analog gain */
    ret = ov5670_write_reg(client, OV5670_8BIT, OV5670_AGC_L, gain & 0xff);
    if (ret){
		return ret;
	}

    ret = ov5670_write_reg(client, OV5670_8BIT, OV5670_AGC_H, (gain >> 8) & 0xff);
    if (ret){
		return ret;
	}

	/* End group */
	ret = ov5670_write_reg(client, OV5670_8BIT,
			       OV5670_GROUP_ACCESS, 0x11);
	if (ret)
		return ret;

	ret = ov5670_write_reg(client, OV5670_8BIT,
			       0x320B, 0x15);
	if (ret)
		return ret;

	/* Delay launch group */
	ret = ov5670_write_reg(client, OV5670_8BIT,
					   OV5670_GROUP_ACCESS, 0xa1);// a0, ;e0=fast launch a0= delay launch
	if (ret)
		return ret;

	return ret;
}

static int ov5670_set_exposure(struct v4l2_subdev *sd, int exposure,
	int gain, int digitgain)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	int ret;

	mutex_lock(&dev->input_lock);
	ret = __ov5670_set_exposure(sd, exposure, gain, digitgain);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static long ov5670_s_exposure(struct v4l2_subdev *sd,
			       struct atomisp_exposure *exposure)
{
	int exp = exposure->integration_time[0];
	int gain = exposure->gain[0];
	int digitgain = exposure->gain[1];

	/* we should not accept the invalid value below. */
	if (gain == 0) {
		struct i2c_client *client = v4l2_get_subdevdata(sd);
		v4l2_err(client, "%s: invalid value\n", __func__);
		return -EINVAL;
	}

	return ov5670_set_exposure(sd, exp, gain, digitgain);
}

static long ov5670_ioctl(struct v4l2_subdev *sd, unsigned int cmd, void *arg)
{

	switch (cmd) {
	case ATOMISP_IOC_S_EXPOSURE:
		return ov5670_s_exposure(sd, arg);
	case ATOMISP_IOC_S_BINNING_SUM:
		binning_sum = *(int*)arg;
		printk("Set low-light mode %d\n", binning_sum);
	       return 0;
	default:
		return -EINVAL;
	}
	return 0;
}

/* This returns the exposure time being used. This should only be used
   for filling in EXIF data, not for actual image processing. */
static int ov5670_q_exposure(struct v4l2_subdev *sd, s32 *value)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	u16 reg_v, reg_v2;
	int ret;

	/* get exposure */
	ret = ov5670_read_reg(client, OV5670_8BIT,
					OV5670_EXPOSURE_L,
					&reg_v);
	if (ret)
		goto err;

	ret = ov5670_read_reg(client, OV5670_8BIT,
					OV5670_EXPOSURE_M,
					&reg_v2);
	if (ret)
		goto err;

	reg_v += reg_v2 << 8;
	ret = ov5670_read_reg(client, OV5670_8BIT,
					OV5670_EXPOSURE_H,
					&reg_v2);
	if (ret)
		goto err;

	*value = reg_v + (((u32)reg_v2 << 16));
err:
	return ret;
}

int ov5670_vcm_power_up(struct v4l2_subdev *sd)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->power_up)
		return dev->vcm_driver->power_up(sd);
	return 0;
}

int ov5670_vcm_power_down(struct v4l2_subdev *sd)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->power_down)
		return dev->vcm_driver->power_down(sd);
	return 0;
}

int ov5670_vcm_init(struct v4l2_subdev *sd)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->init)
		return dev->vcm_driver->init(sd);
	return 0;
}

int ov5670_t_focus_vcm(struct v4l2_subdev *sd, u16 val)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->t_focus_vcm)
		return dev->vcm_driver->t_focus_vcm(sd, val);
	return 0;
}

int ov5670_t_focus_abs(struct v4l2_subdev *sd, s32 value)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->t_focus_abs)
		return dev->vcm_driver->t_focus_abs(sd, value);
	return 0;
}
int ov5670_t_focus_rel(struct v4l2_subdev *sd, s32 value)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->t_focus_rel)
		return dev->vcm_driver->t_focus_rel(sd, value);
	return 0;
}

int ov5670_q_focus_status(struct v4l2_subdev *sd, s32 *value)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->q_focus_status)
		return dev->vcm_driver->q_focus_status(sd, value);
	return 0;
}

int ov5670_q_focus_abs(struct v4l2_subdev *sd, s32 *value)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->q_focus_abs)
		return dev->vcm_driver->q_focus_abs(sd, value);
	return 0;
}

int ov5670_t_vcm_slew(struct v4l2_subdev *sd, s32 value)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->t_vcm_slew)
		return dev->vcm_driver->t_vcm_slew(sd, value);
	return 0;
}

int ov5670_t_vcm_timing(struct v4l2_subdev *sd, s32 value)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	if (dev->vcm_driver && dev->vcm_driver->t_vcm_timing)
		return dev->vcm_driver->t_vcm_timing(sd, value);
	return 0;
}

struct ov5670_control ov5670_controls[] = {
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
		.query = ov5670_q_exposure,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCAL_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focal length",
			.minimum = OV5670_FOCAL_LENGTH_DEFAULT,
			.maximum = OV5670_FOCAL_LENGTH_DEFAULT,
			.step = 0x01,
			.default_value = OV5670_FOCAL_LENGTH_DEFAULT,
			.flags = 0,
		},
		.query = ov5670_g_focal,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_ABSOLUTE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number",
			.minimum = OV5670_F_NUMBER_DEFAULT,
			.maximum = OV5670_F_NUMBER_DEFAULT,
			.step = 0x01,
			.default_value = OV5670_F_NUMBER_DEFAULT,
			.flags = 0,
		},
		.query = ov5670_g_fnumber,
	},
	{
		.qc = {
			.id = V4L2_CID_FNUMBER_RANGE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "f-number range",
			.minimum = OV5670_F_NUMBER_RANGE,
			.maximum =  OV5670_F_NUMBER_RANGE,
			.step = 0x01,
			.default_value = OV5670_F_NUMBER_RANGE,
			.flags = 0,
		},
		.query = ov5670_g_fnumber_range,
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
		.tweak = ov5670_t_focus_abs,
		.query = ov5670_q_focus_abs,
	},
	{
		.qc = {
			.id = V4L2_CID_FOCUS_RELATIVE,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "focus move relative",
			.minimum = OV5670_MAX_FOCUS_NEG,
			.maximum = OV5670_MAX_FOCUS_POS,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ov5670_t_focus_rel,
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
		.query = ov5670_q_focus_status,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_SLEW,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm slew",
			.minimum = 0,
			.maximum = OV5670_VCM_SLEW_STEP_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ov5670_t_vcm_slew,
	},
	{
		.qc = {
			.id = V4L2_CID_VCM_TIMEING,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vcm step time",
			.minimum = 0,
			.maximum = OV5670_VCM_SLEW_TIME_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.tweak = ov5670_t_vcm_timing,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_HORZ,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "horizontal binning factor",
			.minimum = 0,
			.maximum = OV5670_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ov5670_g_bin_factor_x,
	},
	{
		.qc = {
			.id = V4L2_CID_BIN_FACTOR_VERT,
			.type = V4L2_CTRL_TYPE_INTEGER,
			.name = "vertical binning factor",
			.minimum = 0,
			.maximum = OV5670_BIN_FACTOR_MAX,
			.step = 1,
			.default_value = 0,
			.flags = 0,
		},
		.query = ov5670_g_bin_factor_y,
	},
};
#define N_CONTROLS (ARRAY_SIZE(ov5670_controls))

static struct ov5670_control *ov5670_find_control(u32 id)
{
	int i;

	for (i = 0; i < N_CONTROLS; i++)
		if (ov5670_controls[i].qc.id == id)
			return &ov5670_controls[i];
	return NULL;
}

static int ov5670_queryctrl(struct v4l2_subdev *sd, struct v4l2_queryctrl *qc)
{
	struct ov5670_control *ctrl = ov5670_find_control(qc->id);
	struct ov5670_device *dev = to_ov5670_sensor(sd);

	if (ctrl == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	*qc = ctrl->qc;
	mutex_unlock(&dev->input_lock);

	return 0;
}

/* imx control set/get */
static int ov5670_g_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov5670_control *s_ctrl;
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	int ret;

	if (!ctrl)
		return -EINVAL;

	s_ctrl = ov5670_find_control(ctrl->id);
	if ((s_ctrl == NULL) || (s_ctrl->query == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = s_ctrl->query(sd, &ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov5670_s_ctrl(struct v4l2_subdev *sd, struct v4l2_control *ctrl)
{
	struct ov5670_control *octrl = ov5670_find_control(ctrl->id);
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	int ret;

	if ((octrl == NULL) || (octrl->tweak == NULL))
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = octrl->tweak(sd, ctrl->value);
	mutex_unlock(&dev->input_lock);

	return ret;
}

static int ov5670_init(struct v4l2_subdev *sd)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	mutex_lock(&dev->input_lock);

	/* restore settings */
	ov5670_res = ov5670_res_preview;
	N_RES = N_RES_PREVIEW;

	ret = ov5670_write_reg(client, OV5670_8BIT,
					OV5670_SW_RESET, 0x01);
	if (ret) {
		dev_err(&client->dev, "ov5670 reset err.\n");
		return ret;
	}

	mutex_unlock(&dev->input_lock);

	return 0;
}


static int power_up(struct v4l2_subdev *sd)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	dev_err(&client->dev, "@%s: \n", __func__);
	if (NULL == dev->platform_data) {
		dev_err(&client->dev,
			"no camera_sensor_platform_data");
		return -ENODEV;
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 1);
	if (ret)
		goto fail_power;

	/* according to DS, at least 5ms is needed between DOVDD and PWDN */
	usleep_range(5000, 6000);

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 1);
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 1);
		if (ret)
			goto fail_power;
	}

	/* flis clock control */
	ret = dev->platform_data->flisclk_ctrl(sd, 1);
	if (ret)
		goto fail_clk;

	/* according to DS, 20ms is needed between PWDN and i2c access */
	msleep(20);

	return 0;

fail_clk:
	dev->platform_data->gpio_ctrl(sd, 0);
fail_power:
	dev->platform_data->power_ctrl(sd, 0);
	dev_err(&client->dev, "sensor power-up failed\n");

	return ret;
}

static int power_down(struct v4l2_subdev *sd)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (NULL == dev->platform_data) {
		dev_err(&client->dev,
			"no camera_sensor_platform_data");
		return -ENODEV;
	}

	ret = dev->platform_data->flisclk_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "flisclk failed\n");

	/* gpio ctrl */
	ret = dev->platform_data->gpio_ctrl(sd, 0);
	if (ret) {
		ret = dev->platform_data->gpio_ctrl(sd, 0);
		if (ret)
			dev_err(&client->dev, "gpio failed 2\n");
	}

	/* power control */
	ret = dev->platform_data->power_ctrl(sd, 0);
	if (ret)
		dev_err(&client->dev, "vprog failed.\n");

	return ret;
}

static int ov5670_s_power(struct v4l2_subdev *sd, int on)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
//	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int ret = 0;
	if (on == 0) {
		ret = power_down(sd);
		if (dev->vcm_driver && dev->vcm_driver->power_down)
		{
			ret |= dev->vcm_driver->power_down(sd);

		}
	} else {
		if (dev->vcm_driver && dev->vcm_driver->power_up)
			ret = dev->vcm_driver->power_up(sd);
		if (ret)
			return ret;

		ret |= power_up(sd);
		if (!ret)
			return ov5670_init(sd);
	}
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
#define LARGEST_ALLOWED_RATIO_MISMATCH 800
static int distance(struct ov5670_resolution *res, u32 w, u32 h)
{
	unsigned int w_ratio = ((res->width << 13)/w);
	unsigned int h_ratio;
	int match;

	if (h == 0)
		return -1;
	h_ratio = ((res->height << 13) / h);
	if (h_ratio == 0)
		return -1;
	match   = abs(((w_ratio << 13) / h_ratio) - ((int)8192));

	if ((w_ratio < (int)8192) || (h_ratio < (int)8192)  ||
		(match > LARGEST_ALLOWED_RATIO_MISMATCH))
		return -1;

	return w_ratio + h_ratio;
}

/* Return the nearest higher resolution index */
static int nearest_resolution_index(int w, int h)
{
	int i;
	int idx = -1;
	int dist;
	int min_dist = INT_MAX;
	struct ov5670_resolution *tmp_res = NULL;

	for (i = 0; i < N_RES; i++) {
		tmp_res = &ov5670_res[i];
		dist = distance(tmp_res, w, h);
		if (dist == -1)
			continue;
		if (dist < min_dist) {
			min_dist = dist;
			idx = i;
		}
	}

	// if((binning_sum == 1) && (idx + 1 < dev->entries_curr_table)){
	// 	idx++;
	// 	printk("Select binning sum mode idx = %d\n",idx);
	// }
	printk("%s, index = %d\n", __func__, idx);
	return idx;
}

static int get_resolution_index(int w, int h)
{
	int i;

	for (i = 0; i < N_RES; i++) {
		if (w != ov5670_res[i].width)
			continue;
		if (h != ov5670_res[i].height)
			continue;

		return i;
	}

	return -1;
}

static int ov5670_try_mbus_fmt(struct v4l2_subdev *sd,
			struct v4l2_mbus_framefmt *fmt)
{
	int idx;

	if (!fmt)
		return -EINVAL;
	idx = nearest_resolution_index(fmt->width,
					fmt->height);
	if (idx == -1) {
		/* return the largest resolution */
		fmt->width = ov5670_res[0].width;
		fmt->height = ov5670_res[0].height;
	} else {
		fmt->width = ov5670_res[idx].width;
		fmt->height = ov5670_res[idx].height;
	}
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

/* TODO: remove it. */
static int startup(struct v4l2_subdev *sd)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;
	/*
	ret = ov5670_write_reg(client, OV5670_8BIT,
					OV5670_SW_RESET, 0x01);
	if (ret) {
		dev_err(&client->dev, "ov5670 reset err.\n");
		return ret;
	}*/
	ret = ov5670_write_reg_array(client, ov5670_res[dev->fmt_idx].regs);
	if (ret) {
		dev_err(&client->dev, "ov5670 write register err.\n");
		return ret;
	}
	if(dev->current_otp.otp_en == 1)
	{
		ov5670_update_awb_gain(sd);
	}
	return ret;
}

static int ov5670_s_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct camera_mipi_info *ov5670_info = NULL;
	int ret = 0;

	ov5670_info = v4l2_get_subdev_hostdata(sd);
	if (ov5670_info == NULL)
		return -EINVAL;

	mutex_lock(&dev->input_lock);
	ret = ov5670_try_mbus_fmt(sd, fmt);
	if (ret == -1) {
		dev_err(&client->dev, "try fmt fail\n");
		goto err;
	}
	//printk("@%s:width:%d, height:%d\n", __func__, fmt->width, fmt->height);
	dev->fmt_idx = get_resolution_index(fmt->width,
					      fmt->height);
	if (dev->fmt_idx == -1) {
		dev_err(&client->dev, "get resolution fail\n");
		mutex_unlock(&dev->input_lock);
		return -EINVAL;
	}

	ret = startup(sd);
	if (ret)
		dev_err(&client->dev, "ov5670 startup err\n");

	ret = ov5670_get_intg_factor(client, ov5670_info,
					&ov5670_res[dev->fmt_idx]);
	if (ret) {
		dev_err(&client->dev, "failed to get integration_factor\n");
		goto err;
	}
	printk("%s Camera set RES %s selected\n", __func__, ov5670_res[dev->fmt_idx].desc);
err:
	mutex_unlock(&dev->input_lock);
	return ret;
}
static int ov5670_g_mbus_fmt(struct v4l2_subdev *sd,
			     struct v4l2_mbus_framefmt *fmt)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);

	if (!fmt)
		return -EINVAL;

	fmt->width = ov5670_res[dev->fmt_idx].width;
	fmt->height = ov5670_res[dev->fmt_idx].height;
	fmt->code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

static int ov5670_detect(struct i2c_client *client)
{
	struct i2c_adapter *adapter = client->adapter;
	u16 high, low;
	int ret;
	u16 id;
	u8 revision;

	if (!i2c_check_functionality(adapter, I2C_FUNC_I2C))
		return -ENODEV;

	ret = ov5670_read_reg(client, OV5670_8BIT,
					OV5670_SC_CMMN_CHIP_ID_H, &high);
	if (ret) {
		dev_err(&client->dev, "sensor_id_high = 0x%x\n", high);
		return -ENODEV;
	}
	ret = ov5670_read_reg(client, OV5670_8BIT,
					OV5670_SC_CMMN_CHIP_ID_L, &low);
	id = ((((u16) high) << 8) | (u16) low);

	if (id != OV5670_ID) {
		dev_err(&client->dev, "sensor ID error\n");
		return -ENODEV;
	}

	ret = ov5670_read_reg(client, OV5670_8BIT,
					OV5670_SC_CMMN_SUB_ID, &high);
	revision = (u8) high & 0x0f;

	//ov5670_debug(&client->dev, "+++++++++++ sensor_revision = 0x%x\n", revision);
	//dev_dbg(&client->dev, " detect ov5670 success\n");
	//dev_dbg(&client->dev, "sensor_revision = 0x%x\n", revision);
	//dev_dbg(&client->dev, "detect ov5670 success\n");
	return 0;
}

static int ov5670_s_stream(struct v4l2_subdev *sd, int enable)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	//printk("@%s:\n", __func__);
	mutex_lock(&dev->input_lock);

	ret = ov5670_write_reg(client, OV5670_8BIT, OV5670_SW_STREAM,
				enable ? OV5670_START_STREAMING :
				OV5670_STOP_STREAMING);
	if(enable){
		msleep(20);
	}
	mutex_unlock(&dev->input_lock);

	return ret;
}

/* ov5670 enum frame size, frame intervals */
static int ov5670_enum_framesizes(struct v4l2_subdev *sd,
				  struct v4l2_frmsizeenum *fsize)
{
	unsigned int index = fsize->index;

	if (index >= N_RES)
		return -EINVAL;

	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;
	fsize->discrete.width = ov5670_res[index].width;
	fsize->discrete.height = ov5670_res[index].height;
	fsize->reserved[0] = ov5670_res[index].used;

	return 0;
}

static int ov5670_enum_frameintervals(struct v4l2_subdev *sd,
				      struct v4l2_frmivalenum *fival)
{
	unsigned int index = fival->index;

	if (index >= N_RES)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_DISCRETE;
	fival->width = ov5670_res[index].width;
	fival->height = ov5670_res[index].height;
	fival->discrete.numerator = 1;
	fival->discrete.denominator = ov5670_res[index].fps;

	return 0;
}

static int ov5670_enum_mbus_fmt(struct v4l2_subdev *sd,
				unsigned int index,
				enum v4l2_mbus_pixelcode *code)
{
	*code = V4L2_MBUS_FMT_SBGGR10_1X10;

	return 0;
}

int ov5670_read_reg2(struct i2c_client *client, u16 data_length, u16 reg)
{
	u16 val = 0;
	int ret = 0;
	ret = ov5670_read_reg(client,data_length,reg,&val);

	//ov5670_debug(&client->dev,  "++++ read 5670 otp reg:0x%4x val:%x\n", reg, val);
	return (int)val;
}

// index: index of otp group. (1, 2, 3)
// return: 0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int ov5670_check_otp_info(struct i2c_client *client, int index)
{
	int flag;
	//set 0x5002[1] to “0”
	int temp1;
	temp1 = ov5670_read_reg2(client,OV5670_8BIT,0x5002);

	ov5670_write_reg(client,OV5670_8BIT,0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
	ov5670_write_reg(client,OV5670_8BIT,0x3d84, 0xC0);
	//partial mode OTP write start address
	ov5670_write_reg(client,OV5670_8BIT,0x3d88, 0x70);
	ov5670_write_reg(client,OV5670_8BIT,0x3d89, 0x10);
	// partial mode OTP write end address
	ov5670_write_reg(client,OV5670_8BIT,0x3d8A, 0x70);
	ov5670_write_reg(client,OV5670_8BIT,0x3d8B, 0x10);
	// read otp into buffer
	ov5670_write_reg(client,OV5670_8BIT,0x3d81, 0x01);
	msleep(5);

	flag = ov5670_read_reg2(client, OV5670_8BIT, 0x7010);
	//select group
	if (index == 1){
		flag = (flag>>6) & 0x03;
	}
	else if (index == 2){
		flag = (flag>>4) & 0x03;
	}else if(index == 3){
		flag = (flag>>2) & 0x03;
	}
	// clear otp buffer
	ov5670_write_reg(client,OV5670_8BIT,0x7010, 0x00);

	//set 0x5002[1] to “1”
	temp1 = ov5670_read_reg2(client, OV5670_8BIT, 0x5002);
	ov5670_write_reg(client,OV5670_8BIT,0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));
	if (flag == 0x00) {
		return 0;
	}else if (flag & 0x02) {
		return 1;
	}else {
		return 2;
	}

}

// index: index of otp group. (1, 2, 3)
// return:0, group index is empty
// 1, group index has invalid data
// 2, group index has valid data
int ov5670_check_otp_wb(struct i2c_client *client,int index)
{
	int flag;
	//set 0x5002[1] to “0”
	int temp1;
	temp1 = ov5670_read_reg2(client, OV5670_8BIT, 0x5002);

	ov5670_write_reg(client,OV5670_8BIT,0x5002, (0x00 & 0x02) | (temp1 & (~0x02)));
	ov5670_write_reg(client,OV5670_8BIT,0x3d84, 0xC0);
	//partial mode OTP write start address
	ov5670_write_reg(client,OV5670_8BIT,0x3d88, 0x70);
	ov5670_write_reg(client,OV5670_8BIT,0x3d89, 0x20);
	// partial mode OTP write end address
	ov5670_write_reg(client,OV5670_8BIT,0x3d8A, 0x70);
	ov5670_write_reg(client,OV5670_8BIT,0x3d8B, 0x20);
	// read otp into buffer
	ov5670_write_reg(client,OV5670_8BIT,0x3d81, 0x01);
	msleep(5);
	//select group
	flag = ov5670_read_reg2(client, OV5670_8BIT, 0x7020);

	if (index == 1){
		flag = (flag>>6) & 0x03;
	}
	else if (index == 2){
		flag = (flag>>4) & 0x03;
	}
	else if (index == 3){
		flag = (flag>>2) & 0x03;
	}

	// clear otp buffer
	ov5670_write_reg(client,OV5670_8BIT,0x7020, 0x00);

	//set 0x5002[1] to “1”
	temp1 = ov5670_read_reg2(client, OV5670_8BIT,0x5002);
	ov5670_write_reg(client,OV5670_8BIT,0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));
	if (flag == 0x00) {
		return 0;
	}else if (flag & 0x02) {
		return 1;
	}else {
		return 2;
	}
}

// R_gain, sensor red gain of AWB, 0x400 =1
// G_gain, sensor green gain of AWB, 0x400 =1
// B_gain, sensor blue gain of AWB, 0x400 =1
// return 0;
int ov5670_update_awb_gain(struct v4l2_subdev *sd)
{

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct ov5670_device *dev = to_ov5670_sensor(sd);

	int R_gain = dev->current_otp.R_gain;
	int G_gain = dev->current_otp.G_gain;
	int B_gain = dev->current_otp.B_gain;

	if (R_gain>0x400) {
		ov5670_write_reg(client,OV5670_8BIT,0x5032, R_gain>>8);
		ov5670_write_reg(client,OV5670_8BIT,0x5033, R_gain & 0x00ff);
	}
	if (G_gain>0x400) {
		ov5670_write_reg(client,OV5670_8BIT,0x5034, G_gain>>8);
		ov5670_write_reg(client,OV5670_8BIT,0x5035, G_gain & 0x00ff);
	}
	if (B_gain>0x400) {
		ov5670_write_reg(client,OV5670_8BIT,0x5036, B_gain>>8);
		ov5670_write_reg(client,OV5670_8BIT,0x5037, B_gain & 0x00ff);
	}
/*
	#ifdef OV5670_DEBUG_EN > 0
	ov5670_debug(&client->dev, "_ov5670_: %s :rgain1:%d ggain1 %d bgain1 %d\n",
				__func__,R_gain,G_gain,B_gain);
	#endif
*/
	return 0;
}




int ov5670_read_otp_asus(struct v4l2_subdev *sd)
{

	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int i, j, temp1, ret=0;
	int start_addr=0, end_addr=0;
	u8 read_value[24];
	u8 mFind = 0;
    	u16 why = 0;

	temp1 = ov5670_read_reg2(client,OV5670_8BIT,0x5002);
	ov5670_write_reg(client,OV5670_8BIT,0x5002, ((0x00&0x02) | (temp1&(~0x02))));

	for(i = 2; i >= 0; i--)
	{
		if(i == 0){
			start_addr = 0x7010;
			end_addr   = 0x7027;
		}else if(i == 1){
			start_addr = 0x7028;
			end_addr   = 0x703f;
		}else if(i == 2){
			start_addr = 0x7040;
			end_addr   = 0x7057;
		}
		memset(read_value,0,sizeof(read_value));

		ov5670_write_reg(client,OV5670_8BIT,0x3d84, 0xc0);
		//partial mode OTP write start address
		ov5670_write_reg(client,OV5670_8BIT,0x3d88, (start_addr >> 8)&0xff );
		ov5670_write_reg(client,OV5670_8BIT,0x3d89, start_addr &0xff );
		// partial mode OTP write end address
		ov5670_write_reg(client,OV5670_8BIT,0x3d8A, (end_addr >> 8) & 0xff);
		ov5670_write_reg(client,OV5670_8BIT,0x3d8B, end_addr & 0xff);
		// read otp into buffer
		ov5670_write_reg(client,OV5670_8BIT,0x3d81, 0x01);

		msleep(5);
		for(j = 0; j < 24 && mFind != 1; j++)
		{

			ret = ov5670_read_reg(client,OV5670_8BIT,start_addr + j,&why);
			read_value[j] = why;

			if(ret)
			{
				dev_err(&client->dev, "failed to read OTP data i:%d,j:%d\n",i,j);
				continue;
			}
		}
#if 1
                printk("%s Check bank %d [0]=0x%X [1]=0x%X [9]=0x%X [23]=0x%X\n", __func__, i, read_value[0], read_value[1], read_value[9], read_value[23]);
		if(read_value[0] == 0 && read_value[1] == 0 && read_value[9] == 0 && read_value[23] == 0){
			dev_err(&client->dev, "read otp bank fail, find next...\n");
		}else{
			memcpy(ov5670_otp_data, read_value, 24);
			dev_err(&client->dev, "data ok and copy to ov5670_otp_data\n");
			mFind = 1;
		}
#endif
	}

	//set 0x5002[1] to “1”
	temp1 = ov5670_read_reg2(client, OV5670_8BIT, 0x5002);
	ov5670_write_reg(client,OV5670_8BIT,0x5002, (0x02 & 0x02) | (temp1 & (~0x02)));
	return mFind;
}

static int ov5670_s_config(struct v4l2_subdev *sd,
			   int irq, void *platform_data)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (platform_data == NULL)
		return -ENODEV;

	dev->platform_data =
		(struct camera_sensor_platform_data *)platform_data;

	mutex_lock(&dev->input_lock);
	/* power off the module, then power on it in future
	 * as first power on by board may not fulfill the
	 * power on sequqence needed by the module
	 */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "ov5670 power-off err.\n");
		goto fail_power_off;
	}

	ret = power_up(sd);
	if (ret) {
		dev_err(&client->dev, "ov5670 power-up err.\n");
		goto fail_power_on;
	}

	ret = dev->platform_data->csi_cfg(sd, 1);
	if (ret)
		goto fail_csi_cfg;

	/* config & detect sensor */
	ret = ov5670_detect(client);
	msleep(10);
	if (ret) {
		dev_err(&client->dev, "ov5670_detect err s_config.\n");
		goto fail_csi_cfg;
	}
/*
	#ifdef OV5670_DEBUG_EN > 0
	ov5670_debug(&client->dev, "5670 otp_en:%d \r\n", dev->current_otp.otp_en);
	#endif
*/
	/*if(dev->current_otp.otp_en == 1)
	{
		ov5670_otp_read(sd);
	}
	*/
	//otp valid after mipi on and sw stream on
	ret = ov5670_write_reg(client,OV5670_8BIT,
		OV5670_SW_STREAM,OV5670_START_STREAMING);
	//opt process read
	ov5670_read_otp_asus(sd);
	ret = ov5670_write_reg(client,OV5670_8BIT,
		OV5670_SW_STREAM,OV5670_STOP_STREAMING);
	if(ret)	printk("otp read sw stream err\n");
	/* turn off sensor, after probed */
	ret = power_down(sd);
	if (ret) {
		dev_err(&client->dev, "ov5670 power-off err.\n");
		goto fail_csi_cfg;
	}

	ov5670_userCtrl_class = class_create(THIS_MODULE, "ov5670_dev");
    ov5670_register_ctrl_dev = device_create(ov5670_userCtrl_class, NULL, 0, "%s", "register");
    device_create_file(ov5670_register_ctrl_dev, &dev_attr_write_reg);
    device_create_file(ov5670_register_ctrl_dev, &dev_attr_read_reg);

    mutex_unlock(&dev->input_lock);

	return 0;

fail_csi_cfg:
	dev->platform_data->csi_cfg(sd, 0);
fail_power_on:
	power_down(sd);
	dev_err(&client->dev, "sensor power-gating failed\n");
fail_power_off:
	mutex_unlock(&dev->input_lock);
	return ret;
}

static int ov5670_g_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!param)
		return -EINVAL;

	if (param->type != V4L2_BUF_TYPE_VIDEO_CAPTURE) {
		dev_err(&client->dev,  "unsupported buffer type.\n");
		return -EINVAL;
	}

	memset(param, 0, sizeof(*param));
	param->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

	if (dev->fmt_idx >= 0 && dev->fmt_idx < N_RES) {
		param->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
		param->parm.capture.timeperframe.numerator = 1;
		param->parm.capture.capturemode = dev->run_mode;
		param->parm.capture.timeperframe.denominator =
			ov5670_res[dev->fmt_idx].fps;
	}
	return 0;
}

static int ov5670_s_parm(struct v4l2_subdev *sd,
			struct v4l2_streamparm *param)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	dev->run_mode = param->parm.capture.capturemode;

	mutex_lock(&dev->input_lock);
	switch (dev->run_mode) {
	case CI_MODE_VIDEO:
		ov5670_res = ov5670_res_video;
		N_RES = N_RES_VIDEO;
		break;
	case CI_MODE_STILL_CAPTURE:
		ov5670_res = ov5670_res_still;
		N_RES = N_RES_STILL;
		break;
	default:
		ov5670_res = ov5670_res_preview;
		N_RES = N_RES_PREVIEW;
	}
	mutex_unlock(&dev->input_lock);
	return 0;
}

static int ov5670_g_frame_interval(struct v4l2_subdev *sd,
				   struct v4l2_subdev_frame_interval *interval)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);

	interval->interval.numerator = 1;
	interval->interval.denominator = ov5670_res[dev->fmt_idx].fps;

	return 0;
}

static int ov5670_enum_mbus_code(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_mbus_code_enum *code)
{
	if (code->index >= MAX_FMTS)
		return -EINVAL;

	code->code = V4L2_MBUS_FMT_SBGGR10_1X10;
	return 0;
}

static int ov5670_enum_frame_size(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_frame_size_enum *fse)
{
	int index = fse->index;

	if (index >= N_RES)
		return -EINVAL;

	fse->min_width = ov5670_res[index].width;
	fse->min_height = ov5670_res[index].height;
	fse->max_width = ov5670_res[index].width;
	fse->max_height = ov5670_res[index].height;

	return 0;

}

static struct v4l2_mbus_framefmt *
__ov5670_get_pad_format(struct ov5670_device *sensor,
			struct v4l2_subdev_fh *fh, unsigned int pad,
			enum v4l2_subdev_format_whence which)
{
	struct i2c_client *client = v4l2_get_subdevdata(&sensor->sd);

	if (pad != 0) {
		dev_err(&client->dev,
			"__ov5670_get_pad_format err. pad %x\n", pad);
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

static int ov5670_get_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct ov5670_device *snr = to_ov5670_sensor(sd);
	struct v4l2_mbus_framefmt *format =
			__ov5670_get_pad_format(snr, fh, fmt->pad, fmt->which);
	if (!format)
		return -EINVAL;

	fmt->format = *format;
	return 0;
}

static int ov5670_set_pad_format(struct v4l2_subdev *sd,
				struct v4l2_subdev_fh *fh,
				struct v4l2_subdev_format *fmt)
{
	struct ov5670_device *snr = to_ov5670_sensor(sd);

	if (fmt->which == V4L2_SUBDEV_FORMAT_ACTIVE)
		snr->format = fmt->format;

	return 0;
}

static int ov5670_g_skip_frames(struct v4l2_subdev *sd, u32 *frames)
{
	struct ov5670_device *dev = to_ov5670_sensor(sd);

	mutex_lock(&dev->input_lock);
	*frames = ov5670_res[dev->fmt_idx].skip_frames;
	mutex_unlock(&dev->input_lock);

	return 0;
}

static ssize_t ov5670_raw_read_otp(struct device *dev,struct device_attribute *attr,char *buf)
{
	//	test
	int k=0;
	for(;k<24;k++)printk("otp[%d]:0x%02X\n",k,ov5670_otp_data[k]);
	return sprintf(buf, "0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X 0x%02X\n",
			ov5670_otp_data[0],
			ov5670_otp_data[1],
			ov5670_otp_data[2],
			ov5670_otp_data[3],
			ov5670_otp_data[4],
			ov5670_otp_data[5],
			ov5670_otp_data[6],
			ov5670_otp_data[7],
			ov5670_otp_data[8],
			ov5670_otp_data[9],
			ov5670_otp_data[10],
			ov5670_otp_data[11],
			ov5670_otp_data[12],
			ov5670_otp_data[13],
			ov5670_otp_data[14],
			ov5670_otp_data[15],
			ov5670_otp_data[16],
			ov5670_otp_data[17],
			ov5670_otp_data[18],
			ov5670_otp_data[19],
			ov5670_otp_data[20],
			ov5670_otp_data[21],
			ov5670_otp_data[22],
			ov5670_otp_data[23]);
}

static DEVICE_ATTR(ov5670_state, S_IRUGO,ov5670_show_status,NULL);
static DEVICE_ATTR(ov5670_raw_read_otp, S_IRUGO,ov5670_raw_read_otp,NULL);
static struct attribute *ov5670_attrs[] = {
	&dev_attr_ov5670_state.attr,
	&dev_attr_ov5670_raw_read_otp.attr,
		NULL,
};

static struct attribute_group ov5670_attributes_group= {
	.attrs = ov5670_attrs,
};

static const struct v4l2_subdev_sensor_ops ov5670_sensor_ops = {
	.g_skip_frames	= ov5670_g_skip_frames,
};

static const struct v4l2_subdev_video_ops ov5670_video_ops = {
	.s_stream = ov5670_s_stream,
	.g_parm = ov5670_g_parm,
	.s_parm = ov5670_s_parm,
	.enum_framesizes = ov5670_enum_framesizes,
	.enum_frameintervals = ov5670_enum_frameintervals,
	.enum_mbus_fmt = ov5670_enum_mbus_fmt,
	.try_mbus_fmt = ov5670_try_mbus_fmt,
	.g_mbus_fmt = ov5670_g_mbus_fmt,
	.s_mbus_fmt = ov5670_s_mbus_fmt,
	.g_frame_interval = ov5670_g_frame_interval,
};

static const struct v4l2_subdev_core_ops ov5670_core_ops = {
	.s_power = ov5670_s_power,
	.queryctrl = ov5670_queryctrl,
	.g_ctrl = ov5670_g_ctrl,
	.s_ctrl = ov5670_s_ctrl,
	.ioctl = ov5670_ioctl,
};

static const struct v4l2_subdev_pad_ops ov5670_pad_ops = {
	.enum_mbus_code = ov5670_enum_mbus_code,
	.enum_frame_size = ov5670_enum_frame_size,
	.get_fmt = ov5670_get_pad_format,
	.set_fmt = ov5670_set_pad_format,
};

static const struct v4l2_subdev_ops ov5670_ops = {
	.core = &ov5670_core_ops,
	.video = &ov5670_video_ops,
	.pad = &ov5670_pad_ops,
	.sensor = &ov5670_sensor_ops,
};

static int ov5670_remove(struct i2c_client *client)
{
	struct v4l2_subdev *sd = i2c_get_clientdata(client);
	struct ov5670_device *dev = to_ov5670_sensor(sd);
	dev_dbg(&client->dev, "ov5670_remove...\n");

	dev->platform_data->csi_cfg(sd, 0);

	v4l2_device_unregister_subdev(sd);
	media_entity_cleanup(&dev->sd.entity);
	kfree(dev);

	return 0;
}

static int ov5670_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct ov5670_device *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		dev_err(&client->dev, "out of memory\n");
		return -ENOMEM;
	}
	//Add for ATD read camera status+++
	// Register sysfs hooks
	ret = sysfs_create_group(&client->dev.kobj, &ov5670_attributes_group);
	if (ret) {
		dev_err(&client->dev, "Not able to create the sysfs\n");
		return ret;
	}
	//Add for ATD read camera status---

	mutex_init(&dev->input_lock);

	dev->fmt_idx = 0;
	//otp functions
	dev->current_otp.otp_en = 1;// enable otp functions
	v4l2_i2c_subdev_init(&(dev->sd), client, &ov5670_ops);

	if (client->dev.platform_data) {
		ret = ov5670_s_config(&dev->sd, client->irq,
				       client->dev.platform_data);
		if (ret)
			goto out_free;
	}
/*
#ifdef CONFIG_VIDEO_WV511
	dev->vcm_driver = &ov5670_vcms[WV511];
	dev->vcm_driver->init(&dev->sd);
	dev_err(&client->dev, "CONFIG_VIDEO_WV511\n");
#elif defined (CONFIG_VIDEO_DW9714)
	dev->vcm_driver = &ov5670_vcms[DW9714];
	dev->vcm_driver->init(&dev->sd);
#endif
*/
	dev->sd.flags |= V4L2_SUBDEV_FL_HAS_DEVNODE;
	dev->pad.flags = MEDIA_PAD_FL_SOURCE;
	dev->format.code = V4L2_MBUS_FMT_SBGGR10_1X10;
	dev->sd.entity.type = MEDIA_ENT_T_V4L2_SUBDEV_SENSOR;

	ret = media_entity_init(&dev->sd.entity, 1, &dev->pad, 0);


	if (ret)
		ov5670_remove(client);

	return ret;
out_free:
	v4l2_device_unregister_subdev(&dev->sd);
	kfree(dev);
	return ret;
}

MODULE_DEVICE_TABLE(i2c, ov5670_id);
static struct i2c_driver ov5670_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name = OV5670_NAME,
	},
	.probe = ov5670_probe,
	.remove = ov5670_remove,
	.id_table = ov5670_id,
};

static int init_ov5670(void)
{
	return i2c_add_driver(&ov5670_driver);
}

static void exit_ov5670(void)
{

	i2c_del_driver(&ov5670_driver);
}

module_init(init_ov5670);
module_exit(exit_ov5670);

MODULE_AUTHOR("Mason Hua <mason.hua@ovt.com>");
MODULE_DESCRIPTION("A low-level driver for OmniVision 5670 sensors");
MODULE_LICENSE("GPL");
