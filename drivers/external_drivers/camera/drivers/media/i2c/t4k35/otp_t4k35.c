/*
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
#include <linux/bitops.h>
#include <linux/device.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/fs.h>
#include <linux/init.h>
#include <linux/i2c.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/string.h>
#include <linux/slab.h>
#include <linux/types.h>
#include <media/v4l2-chip-ident.h>
#include <media/v4l2-device.h>
#include <asm/intel-mid.h>
#include "common.h"

/* Defines for OTP Data Registers */
#define TSB_OTP_START_ADDR	0x3504
#define TSB_OTP_PAGE_REG	0x3502
#define TSB_OTP_ENABLE		0x3500
#define TSB_OTP_PCLK		0x3545
#define TSB_OTP_PAGE_SIZE	24
#define TSB_OTP_DATA_SIZE	512
#define TSB_OTP_READ_ONETIME	24

#define TSB_DEFAULT_AF_10CM	370
#define TSB_DEFAULT_AF_INF	235
#define TSB_DEFAULT_AF_START	156
#define TSB_DEFAULT_AF_END	660

struct tsb_af_data {
	u16 af_inf_pos;
	u16 af_1m_pos;
	u16 af_10cm_pos;
	u16 af_start_curr;
	u8 module_id;
	u8 vendor_id;
	u16 default_af_inf_pos;
	u16 default_af_10cm_pos;
	u16 default_af_start;
	u16 default_af_end;
};

u8 tsb_otp_data[24];

static int
tsb_read_otp_data(struct i2c_client *client, u16 len, u16 reg, void *val)
{
        struct i2c_msg msg[2];
        u16 data[TSB_SHORT_MAX] = { 0 };
        int err;

        if (!client->adapter) {
                v4l2_err(client, "%s error, no client->adapter\n", __func__);
                return -ENODEV;
        }

        if (len > TSB_BYTE_MAX) {
                v4l2_err(client, "%s error, invalid data length\n", __func__);
                return -EINVAL;
        }

        memset(msg, 0, sizeof(msg));
        memset(data, 0, sizeof(data));

        msg[0].addr = client->addr;
        msg[0].flags = 0;
        msg[0].len = I2C_MSG_LENGTH;
        msg[0].buf = (u8 *)data;
        data[0] = cpu_to_be16(reg);

        msg[1].addr = client->addr;
        msg[1].len = len;
        msg[1].flags = I2C_M_RD;
        msg[1].buf = (u8 *)data;

        err = i2c_transfer(client->adapter, msg, 2);
        if (err < 0) {
                dev_err(&client->dev, "read from offset 0x%x error %d", reg,
                        err);
                return err;
        }

        memcpy(val, data, len);
        return 0;
}

static int tsb_read_otp_reg_array(struct i2c_client *client, u16 size, u16 addr, u8 *buf) {
        u16 index;
        int ret;
        for (index = 0; index + TSB_OTP_READ_ONETIME <= size;
                                        index += TSB_OTP_READ_ONETIME) {
                ret = tsb_read_otp_data(client, TSB_OTP_READ_ONETIME,
                                        addr + index, &buf[index]);
                if (ret)
                        return ret;
        }
        return 0;
}

static int __tsb_otp_read(struct v4l2_subdev *sd, struct tsb_af_data *buf)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        int ret;
        int i;
        u8 read_value[24];

        ret = tsb_write_reg(client, TSB_8BIT, TSB_OTP_PCLK, 0x05);
        if (ret) {
                dev_err(&client->dev, "failed to write OTP input clock\n");
                return ret;
        }

        for(i=2; i>=0; i--) {

                /*set page NO.*/
                ret = tsb_write_reg(client, TSB_8BIT, TSB_OTP_PAGE_REG, i);
                if (ret) {
                        dev_err(&client->dev, "failed to prepare OTP page\n");
                        return ret;
                }

	        ret = tsb_write_reg(client, TSB_8BIT, TSB_OTP_ENABLE, 0x81);
	        if (ret) {
	                dev_err(&client->dev, "failed to write otp enable.\n");
	                return ret;
	        }

                /* Reading the OTP data array */
                ret = tsb_read_otp_reg_array(client, TSB_OTP_PAGE_SIZE,
                        TSB_OTP_START_ADDR, read_value);
                if (ret) {
                        dev_err(&client->dev, "failed to read OTP data\n");
                        return ret;
                }

                printk("%s Check bank %d 0x%X 0x%X\n", __func__, i, read_value[0], read_value[1]);
                if((read_value[0]!=0 || read_value[1]!=0) && (read_value[0]!=0xff || read_value[1]!=0xff))
                        break;
        }

        memcpy(tsb_otp_data, read_value, 24);

        buf->af_inf_pos = read_value[0]<<8 | read_value[1];
        buf->af_1m_pos = read_value[2]<<8 | read_value[3];
        buf->af_10cm_pos = read_value[4]<<8 | read_value[5];
        buf->af_start_curr = read_value[6]<<8 | read_value[7];
        buf->module_id = read_value[8];
        buf->vendor_id = read_value[9];
        buf->default_af_inf_pos = TSB_DEFAULT_AF_INF;
        buf->default_af_10cm_pos = TSB_DEFAULT_AF_10CM;
        buf->default_af_start = TSB_DEFAULT_AF_START;
        buf->default_af_end = TSB_DEFAULT_AF_END;

        return 0;
}

void *tsb_otp_read(struct v4l2_subdev *sd)
{
        struct i2c_client *client = v4l2_get_subdevdata(sd);
        struct tsb_af_data *buf;
        int ret;

        buf = devm_kzalloc(&client->dev, TSB_OTP_DATA_SIZE, GFP_KERNEL);
        if (!buf)
                return ERR_PTR(-ENOMEM);

        ret = __tsb_otp_read(sd, buf);

        /* Driver has failed to find valid data */
        if (ret) {
                dev_err(&client->dev, "sensor found no valid OTP data\n");
                return ERR_PTR(ret);
        }

        return buf;
}
