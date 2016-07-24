/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * Partially based on m-5mols kernel driver,
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *
 * Partially based on jc_v4l2 kernel driver from http://opensource.samsung.com
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 */

#include <linux/atomisp_platform.h>
#include <linux/delay.h>
#include <linux/firmware.h>
#include <linux/i2c.h>
#include <linux/kernel.h>
#include <linux/mm.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/spi/spi.h>
#include <media/v4l2-device.h>
#include <linux/sizes.h>
#include <linux/m10mo.h>
#include <media/m10mo_atomisp.h>
#include <linux/fs.h>
#include <asm/uaccess.h>


/*
 * Currently the FW image and dump paths are hardcoded here.
 * TBD: flexible interface for defining proper path as needed
 */
#define M10MO_FW_LOG1_NAME      "/data/M10MO_log1"
#define M10MO_FW_LOG2_1_NAME    "/data/M10MO_log2_1"
#define M10MO_FW_LOG2_2_NAME    "/data/M10MO_log2_2"
#define M10MO_FW_LOG2_3_NAME    "/data/M10MO_log2_3"
#define M10MO_FW_LOG3_NAME      "/data/M10MO_log3"

#define M10MO_FW_LOG_SUFFIX     ".bin"
#define M10MO_FW_LOG_MAX_NAME_LEN (128)

#define M10MO_FW_DUMP_PATH      "/data/M10MO_dump.bin"
#define M10MO_CALI_DATA_DUMP_PATH      "/data/M10MO_cali_data_dump.bin"
#define M10MO_FW_NAME           "M10MO_fw.bin"
#define M10MO_DIT_FW_NAME       "M10MO_DIT_fw.bin"
#define M10MO_SHD_NAME           "ASUS_SHD.bin"
#define M10MO_SHD_DUMP_PATH      "/data/ASUS_SHD.bin"
#define M10MO_IQ_NAME           "cac.bin"
#define M10MO_IQ_DUMP_PATH      "/data/cac_dump.bin"
#define M10MO_FW_VERSION	"/etc/firmware/M10MO.version"
#define USB_DEV_PATH      "/sys/bus/pci/drivers/dwc3_otg/0000:00:11.0/power/control"

#define SRAM_BUFFER_ADDRESS  0x01100000
#define SDRAM_BUFFER_ADDRESS 0x20000000

#define M10MO_INFO_CHECKSUM  0xedb0

#define M10MO_FLASH_READ_BASE_ADDR	0x18000000
#define PLL_SETTINGS_26MHZ   0x0015013c
#define PLL_SETTINGS_24MHZ   0x00170141
#define PLL_SETTINGS_19_2MHZ 0x001d0152

#define PORT_SETTINGS0_ADDR  0x90001200
#define PORT_SETTINGS1_ADDR  0x90001000
#define PORT_SETTINGS2_ADDR  0x90001100

#define PORT_SETTING_DELAY   (20*1000)
#define I2C_DELAY	     (10*1000)

#define I2C_DUMP_SIZE	     0x20 /* keep as power of 2 values */
#define FW_SIZE		     0x00200000
#define FW_INFO_SIZE         0x00000080
#define FW_SHD_SIZE          0x00021800
#define FW_IQ_CAPTURE_SIZE   0x00000036
#define FW_IQ_CAPTURE_OFFSET 0x00000002
#define FLASH_BLOCK_SIZE     0x10000
#define FLASH_SECTOR_SIZE    0x1000
#define SIO_BLOCK_SIZE	     8192
#define DUMP_BLOCK_SIZE      0x1000
#define SHD_ADDRRESS	     0x27DA3C00
#define IQ_CAC_ADDRRESS	     0x27D582DA

#define FW_VERSION_INFO_ADDR 0x181EF080

#define ONE_WRITE_SIZE	     64

#define ONE_WAIT_LOOP_TIME   10 /* milliseconds */
#define CHIP_ERASE_TIMEOUT (15000 / ONE_WAIT_LOOP_TIME)
#define SECTOR_ERASE_TIMEOUT (5000 / ONE_WAIT_LOOP_TIME)
#define PROGRAMMING_TIMEOUT (15000 / ONE_WAIT_LOOP_TIME)
#define CHECKSUM_TIMEOUT   (5000 / ONE_WAIT_LOOP_TIME)
#define STATE_TRANSITION_TIMEOUT (3000 / ONE_WAIT_LOOP_TIME)

#define NEW_FLASHFW_FLOW

int m10mo_break_log_loop;
int m10mo_usb_state = 0;

/* Tables for m10mo pin configurations */
static const u8 buf_port_settings0_m10mo[] = {
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x0C, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x3F, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x0C,
		  0x00, 0x00, 0x00, 0x18, 0x00, 0x00, 0x00, 0x05,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0xFF,
		 };

static const u8 buf_port_settings1_m10mo[] = {
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x02, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 };

static const u8 buf_port_settings2_m10mo[] = {
		  0x00, 0x00, 0x00, 0x03, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x20, 0x00, 0x00, 0x00, 0x14,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00,
		 };

static const u32 m10mo_fw_address[] = {
	M10MO_FW_VERSION_INFO_ADDR_0,
	M10MO_FW_VERSION_INFO_ADDR_1,
};

static int m10mo_set_flash_address(struct v4l2_subdev *sd, u32 addr)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	ret = m10mo_writel(sd, CATEGORY_FLASHROM, REG_FLASH_ADD, addr);
	if (ret)
		dev_err(&client->dev, "Set flash address failed\n");
	return ret;
}

static int m10mo_set_checksum_size(struct v4l2_subdev *sd, u32 size)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	ret = m10mo_writel(sd, CATEGORY_FLASHROM, REG_CHECK_SUM_SIZE, size);
	if (ret)
		dev_err(&client->dev, "Set checksum size = %x\n",size);
	return ret;
}

static u32 m10mo_get_pll_cfg(u32 freq)
{
	u32 ret;
	switch(freq) {
    case 26000000:
        ret = PLL_SETTINGS_26MHZ;
        break;
	case 24000000:
		ret = PLL_SETTINGS_24MHZ;
		break;
	case 19200000:
		ret = PLL_SETTINGS_19_2MHZ;
		break;
	default:
		/* Defaults to development board xtal freq */
		ret = PLL_SETTINGS_24MHZ;
		break;
	}
	return ret;
}

static int m10mo_wait_operation_complete(struct v4l2_subdev *sd, u8 reg,
					 u32 timeout)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int res;
	do {
		msleep(ONE_WAIT_LOOP_TIME);
		m10mo_readb(sd, CATEGORY_FLASHROM, reg, &res);
	} while ((res != 0) && --timeout);

	if (!timeout) {
		dev_err(&client->dev,
			"timeout while waiting for chip op to finish\n");
		return -ETIME;
	}
	return 0;
}

int m10mo_update_pll_setting(struct v4l2_subdev *sd)
{
	struct m10mo_device *m10mo_dev = to_m10mo_sensor(sd);
	int err;

	pr_info("%s\n", __func__);

	err = m10mo_writel(sd, CATEGORY_FLASHROM,
			   REG_PLL_VALUES,
			   m10mo_get_pll_cfg(m10mo_dev->ref_clock));

	return err;
}

static int m10mo_to_fw_access_mode(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int err;

	err = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT, SZ_64,
				 PORT_SETTINGS0_ADDR , (u8 *)buf_port_settings0_m10mo);
	if (err)
		goto fail;

	usleep_range(PORT_SETTING_DELAY, PORT_SETTING_DELAY);

	err = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT, SZ_64,
				 PORT_SETTINGS1_ADDR, (u8 *)buf_port_settings1_m10mo);
	if (err)
		goto fail;

	usleep_range(PORT_SETTING_DELAY, PORT_SETTING_DELAY);

	err = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT, SZ_64,
				 PORT_SETTINGS2_ADDR, (u8 *)buf_port_settings2_m10mo);
	if (err)
		goto fail;
	usleep_range(PORT_SETTING_DELAY, PORT_SETTING_DELAY);

	err = m10mo_writel(sd, CATEGORY_FLASHROM,
			   REG_PLL_VALUES,
			   m10mo_get_pll_cfg(m10mo_dev->ref_clock));
	if (err)
		goto fail;
	return 0;
fail:
	dev_err(&client->dev, "transition to fw mode failed\n");
	return err;
}


static int m10mo_memory_dump(struct m10mo_device *m10mo_dev, u16 len,
			     u32 addr, u8 *val)
{
	struct i2c_client *client = v4l2_get_subdevdata(&m10mo_dev->sd);
	struct i2c_msg msg;
	unsigned char data[8];
	u16 len_received;
	int i, err = 0;

	if (!client->adapter)
		return -ENODEV;

	if (len >= (sizeof(m10mo_dev->message_buffer) - 3))
		return -EINVAL;

	msg.addr = client->addr;
	msg.flags = 0;
	msg.len = sizeof(data);
	msg.buf = data;

	/* high byte goes out first */
	data[0] = 0x00;
	data[1] = M10MO_MEMORY_READ_8BIT;
	data[2] = (addr >> 24) & 0xFF;
	data[3] = (addr >> 16) & 0xFF;
	data[4] = (addr >> 8) & 0xFF;
	data[5] = addr & 0xFF;
	data[6] = (len >> 8) & 0xFF;
	data[7] = len & 0xFF;

	for (i = M10MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		usleep_range(I2C_DELAY, I2C_DELAY);
	}

	if (err != 1)
		return err;

	msg.flags = I2C_M_RD;
	msg.len = len + 3;
	msg.buf = m10mo_dev->message_buffer;
	for (i = M10MO_I2C_RETRY; i; i--) {
		err = i2c_transfer(client->adapter, &msg, 1);
		if (err == 1)
			break;
		usleep_range(I2C_DELAY, I2C_DELAY);
	}

	if (err != 1)
		return err;

	len_received = m10mo_dev->message_buffer[1] << 8 |
		m10mo_dev->message_buffer[2];
	if (len != len_received)
		dev_err(&client->dev,
			"expected length %d, but return length %d\n",
			len, len_received);

	memcpy(val, m10mo_dev->message_buffer + 3, len);
	return err;
}


int m10mo_SHD_write_block(struct m10mo_device *dev, u32 target_addr,
			    u8 *block, u32 block_size)
{
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int ret;
	u32 ram_buffer = target_addr;
	int i;

	for (i = 0; i < block_size / ONE_WRITE_SIZE && ram_buffer < SHD_ADDRRESS+FW_SHD_SIZE; i++) {
		ret = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT,
					 ONE_WRITE_SIZE,
					 ram_buffer, block);
		if (ret) {
			/* Retry once */
			dev_err(&client->dev,
				"Write block data send retry\n");
			ret = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT,
						 ONE_WRITE_SIZE,
						 ram_buffer, block);
			if (ret) {
				dev_err(&client->dev,
					"Write block data send failed\n");
				return ret;
			}
		}
		ram_buffer += ONE_WRITE_SIZE;
		block += ONE_WRITE_SIZE;
	}

	return ret;
}


int m10mo_isp_fw_SHD_R(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct file *fp;
	mm_segment_t old_fs;
	u8 *buf;
	u32 addr, unit, count;
	int i;
	int err;

	dev_dbg(&client->dev, "Begin FW dump to file %s\n", M10MO_SHD_DUMP_PATH);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(M10MO_SHD_DUMP_PATH,
		O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
			"failed to open %s, err %ld\n",
			M10MO_SHD_DUMP_PATH, PTR_ERR(fp));
		err = -ENOENT;
		goto out_file;
	}

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto out_close;
	}

	addr = SHD_ADDRRESS;
	unit = I2C_DUMP_SIZE;
	count = FW_SHD_SIZE / I2C_DUMP_SIZE;
	for (i = 0; i < count; i++) {
		err = m10mo_memory_dump(m10mo_dev,
					unit,
					addr + (i * unit),
					buf);
		if (err < 0) {
			dev_err(&client->dev, "Memory dump failed %d\n", err);
			goto out_mem_free;
		}
		vfs_write(fp, buf, unit, &fp->f_pos);
	}
	dev_dbg(&client->dev, "End of FW dump to file\n");

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	return err;
}

int m10mo_IQ_write_block(struct m10mo_device *dev, u32 target_addr,
			    u8 *block)
{
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int ret;
		dev_err(&client->dev,"m10mo write target_addr=%x\n",target_addr);
		ret = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT,
					 0x02,
					 target_addr, block);
		if (ret) {
			/* Retry once */
			dev_err(&client->dev,
				"m10mo Write block data send retry\n");
			ret = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT,
						 0x02,
						 target_addr, block);
			if (ret) {
				dev_err(&client->dev,
					"m10mo Write block data send failed\n");
				return ret;
			}
		}
	return ret;
}


int m10mo_isp_fw_IQ_R(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct file *fp;
	mm_segment_t old_fs;
	u8 *buf;
	u32 addr, unit, count;
	int i;
	int err;

	dev_dbg(&client->dev, "m10mo Begin FW dump to file %s\n", M10MO_IQ_DUMP_PATH);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(M10MO_IQ_DUMP_PATH,
		O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
			"failed to open %s, err %ld\n",
			M10MO_IQ_DUMP_PATH, PTR_ERR(fp));
		err = -ENOENT;
		goto out_file;
	}

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto out_close;
	}

	addr = IQ_CAC_ADDRRESS;
	unit = FW_IQ_CAPTURE_OFFSET;
	count = FW_IQ_CAPTURE_SIZE / FW_IQ_CAPTURE_OFFSET;
	for (i = 0; i < count; i++) {

		dev_err(&client->dev,"m10mo read target_addr=%x\n",addr);
		err = m10mo_memory_dump(m10mo_dev,
					unit,
					addr,
					buf);
		if (err < 0) {
			dev_err(&client->dev, "m10mo Memory dump failed %d\n", err);
			goto out_mem_free;
		}
		vfs_write(fp, buf, unit, &fp->f_pos);
		addr += FW_IQ_CAPTURE_OFFSET;
	}
	dev_dbg(&client->dev, "m10mo End of FW dump to file\n");

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	return err;
}

int m10mo_dump_fw(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct file *fp;
	mm_segment_t old_fs;
	u8 *buf;
	u32 addr, unit, count;
	int i;
	int err;

	dev_dbg(&client->dev, "Begin FW dump to file %s\n", M10MO_FW_DUMP_PATH);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(M10MO_FW_DUMP_PATH,
		O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
			"failed to open %s, err %ld\n",
			M10MO_FW_DUMP_PATH, PTR_ERR(fp));
		err = -ENOENT;
		goto out_file;
	}

	err = m10mo_to_fw_access_mode(m10mo_dev);
	if (err)
		goto out_close;

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto out_close;
	}

	err = m10mo_writeb(sd, CATEGORY_FLASHROM,
			   REG_FW_READ, REG_FW_READ_CMD_READ);

	if (err) {
		dev_err(&client->dev, "FW read cmd failed %d\n", err);
		goto out_mem_free;
	}

	addr = M10MO_FLASH_READ_BASE_ADDR;
	unit = I2C_DUMP_SIZE;
	count = FW_SIZE / I2C_DUMP_SIZE;
	for (i = 0; i < count; i++) {
		err = m10mo_memory_dump(m10mo_dev,
					unit,
					addr + (i * unit),
					buf);
		if (err < 0) {
			dev_err(&client->dev, "Memory dump failed %d\n", err);
			goto out_mem_free;
		}
		vfs_write(fp, buf, unit, &fp->f_pos);
	}
	dev_dbg(&client->dev, "End of FW dump to file\n");

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	return err;
}

int m10mo_dump_cali_data(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	struct file *fp;
	mm_segment_t old_fs;
	u8 *buf;
	u32 addr, unit, count;
	int i;
	int err;

	dev_dbg(&client->dev, "Begin calibration data dump to file %s\n", M10MO_CALI_DATA_DUMP_PATH);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(M10MO_CALI_DATA_DUMP_PATH,
		O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
			"failed to open %s, err %ld\n",
			M10MO_CALI_DATA_DUMP_PATH, PTR_ERR(fp));
		err = -ENOENT;
		goto out_file;
	}

	err = m10mo_to_fw_access_mode(m10mo_dev);
	if (err)
		goto out_close;

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		err = -ENOMEM;
		goto out_close;
	}

	err = m10mo_writeb(sd, CATEGORY_FLASHROM,
			   REG_FW_READ, REG_FW_READ_CMD_READ);

	if (err) {
		dev_err(&client->dev, "FW read cmd failed %d\n", err);
		goto out_mem_free;
	}

	addr = M10MO_FLASH_READ_BASE_ADDR + 0x1F4000;
	unit = I2C_DUMP_SIZE;
	count = FLASH_SECTOR_SIZE / I2C_DUMP_SIZE;
	for (i = 0; i < count; i++) {
		err = m10mo_memory_dump(m10mo_dev,
					unit,
					addr + (i * unit),
					buf);
		if (err < 0) {
			dev_err(&client->dev, "Memory dump failed %d\n", err);
			goto out_mem_free;
		}
		vfs_write(fp, buf, unit, &fp->f_pos);
	}
	dev_dbg(&client->dev, "End of calibration data dump to file\n");

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	return err;
}

static void m10mo_gen_log_name(char *name, char *prefix)
{
	static long long time;

	time = ktime_to_ms(ktime_get());
	snprintf(name, M10MO_FW_LOG_MAX_NAME_LEN, "%s_%lld%s", prefix, time, M10MO_FW_LOG_SUFFIX);
	printk("m10mo log path is %s\n", name);
}

int m10mo_dump_string_log3(struct v4l2_subdev *sd)
{
	u32 addr;
	mm_segment_t old_fs;
	struct file *fp;
	u32 len = MAX_LOG_STR_LEN;
	u32 ret = 0;
	u32 count = 0;
	u32 count_len = 0;
	u32 ptr = 0;
	char *buf = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	char filename[M10MO_FW_LOG_MAX_NAME_LEN] = {0};

	m10mo_gen_log_name(filename, M10MO_FW_LOG3_NAME);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename,
			O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
				"failed to open %s, err %ld\n",
				M10MO_FW_DUMP_PATH, PTR_ERR(fp));
		ret = -ENOENT;
		goto out_file;
	}

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out_close;
	}

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_MODE, LOG_TRACE_MODE);
	if (ret < 0)
		goto out_mem_free;

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_DISABLE);
	if (ret < 0)
		goto out_mem_free;

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ADD_SHOW, LOG_ADD_SHOW_INIT_VALUE);
	if (ret < 0)
		goto out_mem_free;

	m10mo_break_log_loop = 0;
	while (count++ < MAX_MEM_DUMP_NUM_LOG3) {

		if(m10mo_break_log_loop != 0) goto out_mem_free;

		ret = m10mo_writew(sd, CATEGORY_LOGLEDFLASH, LOG_SEL1, ptr);
		if (ret < 0)
			goto out_mem_free;

		ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_OUTPUT_STR);
		if (ret < 0)
			goto out_mem_free;

		do {
			ret = m10mo_readb(sd, CATEGORY_LOGLEDFLASH, LOG_STR_LEN, &len);
			if (ret < 0)
				goto out_mem_free;
			msleep(10);
			count_len++;
		} while ((len == MAX_LOG_STR_LEN) && (count_len < 10));

		if (len == MIN_LOG_STR_LEN) {
			goto out_mem_free;
		} else {
			ret = m10mo_readl(sd, CATEGORY_LOGLEDFLASH, LOG_STR_ADD3, &addr);
			if (ret < 0)
				goto out_mem_free;

			ret = m10mo_memory_read(sd, len, addr, buf);
			if (ret < 0)
				goto out_mem_free;
			/* Do not add buf[len] = '\n'; */
			vfs_write(fp, buf, len, &fp->f_pos);
		}
		len = MAX_LOG_STR_LEN;
		ptr = ptr + 1;
	}

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	if (ret < 0)
		dev_err(&client->dev, "%s, dump log error\n", __func__);

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_ENABLE);
	if (ret < 0)
		dev_err(&client->dev, "%s, m10mo_writeb error\n", __func__);
	m10mo_break_log_loop = 0;
	return ret;
}

/* Not verified */
int m10mo_dump_string_log2_3(struct v4l2_subdev *sd)
{
	u32 addr, i;
	mm_segment_t old_fs;
	struct file *fp;
	u32 len = MAX_LOG_STR_LEN_LOG2;
	u32 ret = 0;
	u32 count = 0;
	u32 unit_count = 0;
	u32 ptr = 0;
	char *buf = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	char filename[M10MO_FW_LOG_MAX_NAME_LEN] = {0};

	m10mo_gen_log_name(filename, M10MO_FW_LOG2_3_NAME);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename,
			O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
				"failed to open %s, err %ld\n",
				M10MO_FW_DUMP_PATH, PTR_ERR(fp));
		ret = -ENOENT;
		goto out_file;
	}

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out_close;
	}

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_MODE, LOG_ANALYZE_MODE2);
	if (ret < 0)
		goto out_mem_free;

	while (count++ < MAX_MEM_DUMP_NUM) {
		ret = m10mo_writew(sd, CATEGORY_LOGLEDFLASH, LOG_SEL1, ptr);
		if (ret < 0)
			goto out_mem_free;

		ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_OUTPUT_STR);
		if (ret < 0)
			goto out_mem_free;

		do {
			ret = m10mo_readw(sd, CATEGORY_LOGLEDFLASH, LOG_DATA_LEN1, &len);
			if (ret < 0)
				goto out_mem_free;
		} while (len == MAX_LOG_STR_LEN_LOG2);

		if (len == MIN_LOG_STR_LEN_LOG2) {
			goto out_mem_free;
		} else {

			if (len > MAX_LOG_STR_LEN_LOG2)
				len = MAX_LOG_STR_LEN_LOG2;

			ret = m10mo_readl(sd, CATEGORY_LOGLEDFLASH, LOG_STR_ADD3, &addr);
			if (ret < 0)
				goto out_mem_free;

			unit_count =  len / I2C_MEM_READ_SIZE;
			for (i = 0; i <= unit_count; i += I2C_MEM_READ_SIZE) {
				if ((len - i) <= I2C_MEM_READ_SIZE) {
					ret = m10mo_memory_read(sd, len - i, addr + i, buf);
					if (ret < 0)
						goto out_mem_free;

					vfs_write(fp, buf, len - i, &fp->f_pos);
					break;
				} else {
					ret = m10mo_memory_read(sd, I2C_MEM_READ_SIZE, addr + i, buf);
					if (ret < 0)
						goto out_mem_free;

					vfs_write(fp, buf, I2C_MEM_READ_SIZE, &fp->f_pos);
				}
			}
		}
		len = MAX_LOG_STR_LEN_LOG2;
		ptr = ptr + 1;
	}

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	if (ret < 0)
		dev_err(&client->dev, "%s, dump log error\n", __func__);
	return ret;
}

/* Not verified */
int m10mo_dump_string_log2_2(struct v4l2_subdev *sd)
{
	u32 addr, i;
	mm_segment_t old_fs;
	struct file *fp;
	u32 len = MAX_LOG_STR_LEN_LOG2;
	u32 ret = 0;
	u32 count = 0;
	u32 unit_count = 0;
	u32 ptr = 0;
	char *buf = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	char filename[M10MO_FW_LOG_MAX_NAME_LEN] = {0};

	m10mo_gen_log_name(filename, M10MO_FW_LOG2_2_NAME);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename,
			O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
				"failed to open %s, err %ld\n",
				M10MO_FW_DUMP_PATH, PTR_ERR(fp));
		ret = -ENOENT;
		goto out_file;
	}

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out_close;
	}

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_MODE, LOG_ANALYZE_MODE1);
	if (ret < 0)
		goto out_mem_free;

	while (count++ < MAX_MEM_DUMP_NUM) {
		ret = m10mo_writew(sd, CATEGORY_LOGLEDFLASH, LOG_SEL1, ptr);
		if (ret < 0)
			goto out_mem_free;

		ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_OUTPUT_STR);
		if (ret < 0)
			goto out_mem_free;

		do {
			ret = m10mo_readw(sd, CATEGORY_LOGLEDFLASH, LOG_DATA_LEN1, &len);
			if (ret < 0)
				goto out_mem_free;
		} while (len == MAX_LOG_STR_LEN_LOG2);

		if (len == MIN_LOG_STR_LEN_LOG2) {
			goto out_mem_free;
		} else {

			if (len > MAX_LOG_STR_LEN_LOG2)
				len = MAX_LOG_STR_LEN_LOG2;

			ret = m10mo_readl(sd, CATEGORY_LOGLEDFLASH, LOG_STR_ADD3, &addr);
			if (ret < 0)
				goto out_mem_free;

			unit_count =  len / I2C_MEM_READ_SIZE;
			for (i = 0; i <= unit_count; i += I2C_MEM_READ_SIZE) {
				if ((len - i) <= I2C_MEM_READ_SIZE) {
					ret = m10mo_memory_read(sd, len - i, addr + i, buf);
					if (ret < 0)
						goto out_mem_free;

					vfs_write(fp, buf, len - i, &fp->f_pos);
					break;
				} else {
					ret = m10mo_memory_read(sd, I2C_MEM_READ_SIZE, addr + i, buf);
					if (ret < 0)
						goto out_mem_free;

					vfs_write(fp, buf, I2C_MEM_READ_SIZE, &fp->f_pos);
				}
			}
		}
		len = MAX_LOG_STR_LEN_LOG2;
		ptr = ptr + 1;
	}

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	if (ret < 0)
		dev_err(&client->dev, "%s, dump log error\n", __func__);
	return ret;
}

int m10mo_dump_string_log2_1(struct v4l2_subdev *sd)
{
	u32 addr, i;
	mm_segment_t old_fs;
	struct file *fp;
	u32 len = MAX_LOG_STR_LEN_LOG2;
	u32 ret = 0;
	u32 count = 0;
	u32 unit_count = 0;
	u32 ptr = 0;
	char *buf = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	char filename[M10MO_FW_LOG_MAX_NAME_LEN] = {0};

	m10mo_gen_log_name(filename, M10MO_FW_LOG2_1_NAME);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename,
			O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
				"failed to open %s, err %ld\n",
				M10MO_FW_DUMP_PATH, PTR_ERR(fp));
		ret = -ENOENT;
		goto out_file;
	}

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out_close;
	}

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_MODE, LOG_ANALYZE_MODE0);
	if (ret < 0)
		goto out_mem_free;

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_DISABLE);
	if (ret < 0)
		goto out_mem_free;

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ADD_SHOW, LOG_ADD_SHOW_INIT_VALUE);
	if (ret < 0)
		goto out_mem_free;

	while (count++ < MAX_MEM_DUMP_NUM) {
		ret = m10mo_writew(sd, CATEGORY_LOGLEDFLASH, LOG_SEL1, ptr);
		if (ret < 0)
			goto out_mem_free;

		ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_OUTPUT_STR);
		if (ret < 0)
			goto out_mem_free;

		do {
			ret = m10mo_readw(sd, CATEGORY_LOGLEDFLASH, LOG_DATA_LEN1, &len);
			if (ret < 0)
				goto out_mem_free;
		} while (len == MAX_LOG_STR_LEN_LOG2);

		if (len == MIN_LOG_STR_LEN_LOG2) {
				goto out_mem_free;
		} else {

			if (len > MAX_LOG_STR_LEN_LOG2)
				len = MAX_LOG_STR_LEN_LOG2;

			ret = m10mo_readl(sd, CATEGORY_LOGLEDFLASH, LOG_STR_ADD3, &addr);
			if (ret < 0)
				goto out_mem_free;

			unit_count =  len / I2C_MEM_READ_SIZE;
			for (i = 0; i <= unit_count; i += I2C_MEM_READ_SIZE) {
				if ((len - i) <= I2C_MEM_READ_SIZE) {
					ret = m10mo_memory_read(sd, len - i, addr + i, buf);
					if (ret < 0)
						goto out_mem_free;

					vfs_write(fp, buf, len - i, &fp->f_pos);
					break;
				} else {
					ret = m10mo_memory_read(sd, I2C_MEM_READ_SIZE, addr + i, buf);
					if (ret < 0)
						goto out_mem_free;

					vfs_write(fp, buf, I2C_MEM_READ_SIZE, &fp->f_pos);
				}
			}
		}
		len = MAX_LOG_STR_LEN_LOG2;
		ptr = ptr + 1;
	}

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	if (ret < 0)
		dev_err(&client->dev, "%s, dump log error\n", __func__);
	return ret;
}

int m10mo_dump_string_log1(struct v4l2_subdev *sd)
{
	u32 addr;
	mm_segment_t old_fs;
	struct file *fp;
	u32 len = MAX_LOG_STR_LEN;
	u32 ret = 0;
	u32 count = 0;
	u32 count_len = 0;
	u32 ptr = 0;
	char *buf = NULL;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	char filename[M10MO_FW_LOG_MAX_NAME_LEN] = {0};

	m10mo_gen_log_name(filename, M10MO_FW_LOG1_NAME);

	old_fs = get_fs();
	set_fs(KERNEL_DS);
	fp = filp_open(filename,
			O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		dev_err(&client->dev,
				"failed to open %s, err %ld\n",
				M10MO_FW_DUMP_PATH, PTR_ERR(fp));
		ret = -ENOENT;
		goto out_file;
	}

	buf = kmalloc(DUMP_BLOCK_SIZE, GFP_KERNEL);
	if (!buf) {
		dev_err(&client->dev, "Failed to allocate memory\n");
		ret = -ENOMEM;
		goto out_close;
	}

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_MODE, LOG_STANDARD_MODE);
	if (ret < 0)
		goto out_mem_free;

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_DISABLE);
	if (ret < 0)
		goto out_mem_free;

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ADD_SHOW, LOG_ADD_SHOW_INIT_VALUE);
	if (ret < 0)
		goto out_mem_free;

	while (count++ < MAX_MEM_DUMP_NUM) {
		ret = m10mo_writew(sd, CATEGORY_LOGLEDFLASH, LOG_SEL1, ptr);
		if (ret < 0)
			goto out_mem_free;

		ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_OUTPUT_STR);
		if (ret < 0)
			goto out_mem_free;

		do {
			ret = m10mo_readb(sd, CATEGORY_LOGLEDFLASH, LOG_STR_LEN, &len);
			if (ret < 0)
				goto out_mem_free;
			msleep(10);
			count_len++;
		} while ((len == MAX_LOG_STR_LEN) && (count_len < 10));

		if (len == MIN_LOG_STR_LEN) {
				goto out_mem_free;
		} else {
			ret = m10mo_readl(sd, CATEGORY_LOGLEDFLASH, LOG_STR_ADD3, &addr);
			if (ret < 0)
				goto out_mem_free;

				ret = m10mo_memory_read(sd, len, addr, buf);
				if (ret < 0)
					goto out_mem_free;

				buf[len] = '\n';
				vfs_write(fp, buf, len + 1, &fp->f_pos);
		}
		len = MAX_LOG_STR_LEN;
		ptr = ptr + 1;
	}

out_mem_free:
	kfree(buf);
out_close:
	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);

	if (ret < 0)
		dev_err(&client->dev, "%s, dump log error\n", __func__);

	ret = m10mo_writeb(sd, CATEGORY_LOGLEDFLASH, LOG_ACT, LOG_ACT_ENABLE);
	if (ret < 0)
		dev_err(&client->dev, "%s, m10mo_writeb error\n", __func__);

	return ret;
}

int m10mo_get_fw_address_count(void)
{
	return ARRAY_SIZE(m10mo_fw_address);
}

int m10mo_get_isp_fw_version_string(struct m10mo_device *dev,
		char *buf, int len, int fw_address_id)
{
	int err;
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	err = m10mo_to_fw_access_mode(dev);
	if (err)
		return err;

	err = m10mo_writeb(sd, CATEGORY_FLASHROM,
				REG_FW_READ, REG_FW_READ_CMD_READ);
	if (err) {
		dev_err(&client->dev, "Read mode transition fail: %d\n", err);
		return err;
	}
	msleep(10);

	memset(buf, 0, len);
	if ((fw_address_id < 0) ||
		(fw_address_id >= ARRAY_SIZE(m10mo_fw_address))) {
		dev_err(&client->dev, "Error FW address ID: %d\n",
				fw_address_id);
		fw_address_id = 0;
	}
	err = m10mo_memory_read(sd, len - 1,
			m10mo_fw_address[fw_address_id], buf);
	if (err)
		dev_err(&client->dev, "version read failed\n");

	/* Return value checking intentionally omitted */
	(void) m10mo_writeb(sd, CATEGORY_FLASHROM,
			    REG_FW_READ, REG_FW_READ_CMD_NONE);
	return err;
}

int m10mo_fw_checksum(struct m10mo_device *dev, u16 *result)
{
	int err;
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int res = 0, fw_info_res = 0;

	err = m10mo_setup_flash_controller(sd);
	if (err)
		goto leave;

	//err = m10mo_to_fw_access_mode(dev);
	//if (err)
//		goto leave;

	/* Set start address to 0*/
	err = m10mo_set_flash_address(sd, 0x0);
	if (err)
		goto leave;

	/* request checksum */
#ifndef NEW_FLASHFW_FLOW
	err = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_FLASH_CHECK, 4);
#else

	err = m10mo_set_checksum_size(sd, 0x001a2000);

	if (err)
		goto leave;

	err = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_FLASH_CHECK, 2);//partial update
#endif
	if (err) {
		dev_err(&client->dev, "Request checksum failed\n");
		goto leave;
	}

	err = m10mo_wait_operation_complete(sd, REG_FLASH_CHECK,
					    CHECKSUM_TIMEOUT);
	if (err)
		goto leave;

	err = m10mo_readw(sd, CATEGORY_FLASHROM, REG_FLASH_SUM , &res);
	if (err) {
		dev_err(&client->dev, "Checksum read failed\n");
		goto leave;
	}
	if(res == 0x2840) res=0;
	*result = (u16)res;

#if 1
	//for FW_info//

	/* Set start address to 0*/
	err = m10mo_set_flash_address(sd, 0x001ff000);
	if (err)
		goto leave;

	/* request checksum */
	err = m10mo_set_checksum_size(sd, 0x00000080);
	if (err)
		goto leave;

	err = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_FLASH_CHECK, 2);//partial update

	if (err) {
		dev_err(&client->dev, "Request checksum failed\n");
		goto leave;
	}

	err = m10mo_wait_operation_complete(sd, REG_FLASH_CHECK,
					    CHECKSUM_TIMEOUT);
	if (err)
		goto leave;

	err = m10mo_readw(sd, CATEGORY_FLASHROM, REG_FLASH_SUM , &fw_info_res);
	if (err) {
		dev_err(&client->dev, "Checksum read failed\n");
		goto leave;
	}

	if(fw_info_res == M10MO_INFO_CHECKSUM) fw_info_res=0;

	*result = (u16)res + (u16)fw_info_res;
#endif

leave:
	return err;
}

int m10mo_sector_erase_flash(struct m10mo_device *dev, u32 sector_addr)
{
	int ret;
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	/*
	 * Preconditions - system is already in flash access mode,
	 * plls configured
	 */

	/* Set start address */
	ret = m10mo_set_flash_address(sd, sector_addr);
	if (ret)
		return ret;

	ret = m10mo_writeb(sd, CATEGORY_FLASHROM,
			   REG_FLASH_ERASE,
			   REG_FLASH_ERASE_SECTOR_ERASE);
	if (ret) {
		dev_err(&client->dev, "Checksum cmd failed\n");
		return ret;
	}

	ret = m10mo_wait_operation_complete(sd, REG_FLASH_ERASE,
					    SECTOR_ERASE_TIMEOUT);
	return ret;
}


int m10mo_block_erase_flash(struct m10mo_device *dev, u32 block_addr, bool initRAM)
{
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	/*
	 * Preconditions - system is already in flash access mode,
	 * plls configured
	 */

if(!initRAM){
	/* Setup internal RAM */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_RAM_START,
			   REG_RAM_START_SRAM);
	if (ret) {
		dev_err(&client->dev, "Ram setup failed\n");
		return ret;
	}
	mdelay(10);
}
	/* Set start address to 0*/
	ret = m10mo_set_flash_address(sd, block_addr);
	if (ret)
		return ret;
	/* chip erase command */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_FLASH_ERASE,
			   REG_FLASH_ERASE_BLOCK64k_ERASE);

	if (ret) {
		dev_err(&client->dev, "Chip erase cmd failed\n");
		return ret;
	}
	ret = m10mo_wait_operation_complete(sd, REG_FLASH_ERASE,
					    CHIP_ERASE_TIMEOUT);


	return ret;
}

/* Full chip erase */
int m10mo_chip_erase_flash(struct m10mo_device *dev)
{
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;
	/*
	 * Preconditions - system is already in flash access mode,
	 * plls configured
	 */

	/* Setup internal RAM */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_RAM_START,
			   REG_RAM_START_SRAM);
	if (ret) {
		dev_err(&client->dev, "Ram setup failed\n");
		return ret;
	}
	mdelay(10);

	/* Set start address to 0*/
	ret = m10mo_set_flash_address(sd, 0x0);
	if (ret)
		return ret;

	/* chip erase command */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_FLASH_ERASE,
			   REG_FLASH_ERASE_CHIP_ERASE);
	if (ret) {
		dev_err(&client->dev, "Chip erase cmd failed\n");
		return ret;
	}
	ret = m10mo_wait_operation_complete(sd, REG_FLASH_ERASE,
					    CHIP_ERASE_TIMEOUT);
	return ret;
}

int m10mo_flash_write_block(struct m10mo_device *dev, u32 target_addr,
			    u8 *block, u32 block_size)
{
	struct v4l2_subdev *sd = &dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	int ret;
	u32 ram_buffer = SRAM_BUFFER_ADDRESS;
	int i;

	ret = m10mo_set_flash_address(sd, target_addr);
	if (ret)
		return ret;

	/* Set block size of 64k == 0 as reg value */
	ret = m10mo_writew(sd, CATEGORY_FLASHROM, REG_FLASH_BYTE, block_size);
	if (ret) {
		dev_err(&client->dev, "Set flash block size failed\n");
		return ret;
	}

	for (i = 0; i < block_size / ONE_WRITE_SIZE; i++) {
		ret = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT,
					 ONE_WRITE_SIZE,
					 ram_buffer, block);
		if (ret) {
			/* Retry once */
			dev_err(&client->dev,
				"Write block data send retry\n");
			ret = m10mo_memory_write(sd, M10MO_MEMORY_WRITE_8BIT,
						 ONE_WRITE_SIZE,
						 ram_buffer, block);
			if (ret) {
				dev_err(&client->dev,
					"Write block data send failed\n");
				return ret;
			}
		}
		ram_buffer += ONE_WRITE_SIZE;
		block += ONE_WRITE_SIZE;
	}

	/* Program block */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_FLASH_WRITE,
			   REG_FLASH_WRITE_START_PRG);
	if (ret) {
		dev_err(&client->dev, "FW program block failed\n");
		return ret;
	}

	ret = m10mo_wait_operation_complete(sd, REG_FLASH_WRITE, PROGRAMMING_TIMEOUT);

	return ret;
}

static int m10mo_sio_write(struct m10mo_device *m10mo_dev, u8 *buf, u32 size, u32 target_addr)
{
	int ret;
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (!m10mo_dev->spi) {
		dev_err(&client->dev, "No spi device available\n");
		return -ENODEV;
	}

	/* Set SIO destination address */
	ret = m10mo_writel(sd, CATEGORY_FLASHROM, REG_DATA_RAM_ADDR,
			   SDRAM_BUFFER_ADDRESS);
	if (ret) {
		dev_err(&client->dev, "sio address setting failed\n");
		return ret;
	}
#ifdef NEW_FLASHFW_FLOW
	/* Set programming size - multiples of 16 bytes */
	ret = m10mo_writel(sd, CATEGORY_FLASHROM, REG_DATA_TRANS_SIZE,
			   size);
#else
	/* Set programming size - multiples of 16 bytes */
	ret = m10mo_writel(sd, CATEGORY_FLASHROM, REG_DATA_TRANS_SIZE,
			   FW_SIZE);
#endif
	if (ret) {
		dev_err(&client->dev, "set program size failed\n");
		return ret;
	}

	/* Set SDRAM - mystical value from flow picture */
	ret = m10mo_writew(sd, CATEGORY_FLASHROM, REG_SDRAM_CFG, 0x0608);
	if (ret) {
		dev_err(&client->dev, "set sdram failed\n");
		return ret;
	}

	/* Set sio mode: */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_SIO_MODE,
			   REG_SIO_MODE_RISING_LATCH);
	if (ret) {
		dev_err(&client->dev, "set sio mode failed\n");
		return ret;
	}

	/* Start sio mode */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_RAM_START,
			   REG_RAM_START_SDRAM);
	if (ret) {
		dev_err(&client->dev, "start sio mode failed \n");
		return ret;
	}
	msleep(3);
	ret = m10mo_wait_operation_complete(sd, REG_RAM_START,
					    STATE_TRANSITION_TIMEOUT);
	if (ret)
		return ret;

	usleep_range(30000, 30000);  /* TDB: is that required */
#ifdef NEW_FLASHFW_FLOW
	ret = m10mo_dev->spi->write(m10mo_dev->spi->spi_device,
				    buf, size, SIO_BLOCK_SIZE);
#else
	ret = m10mo_dev->spi->write(m10mo_dev->spi->spi_device,
				    buf, FW_SIZE, SIO_BLOCK_SIZE);
#endif
	if (ret)
		return ret;

	msleep(5); /* TDB: is that required */

	/* Flash address to 0*/
	ret = m10mo_set_flash_address(sd, target_addr);
	if (ret)
		return ret;
#ifdef NEW_FLASHFW_FLOW
	/* Programming size */
	ret = m10mo_writel(sd, CATEGORY_FLASHROM, REG_DATA_TRANS_SIZE, size);
#else
	/* Programming size */
	ret = m10mo_writel(sd, CATEGORY_FLASHROM, REG_DATA_TRANS_SIZE, FW_SIZE);
#endif
	if (ret) {
		dev_err(&client->dev, "set sio programming size failed \n");
		return ret;
	}

	/* Start programming */
	ret = m10mo_writeb(sd, CATEGORY_FLASHROM, REG_FLASH_WRITE,
			   REG_FLASH_WRITE_START_PRG);
	if (ret) {
		dev_err(&client->dev, "SIO start programming failed\n");
		return ret;
	}
	ret = m10mo_wait_operation_complete(sd, REG_FLASH_WRITE,
					    PROGRAMMING_TIMEOUT);
	return ret;
}

static const struct firmware *
m10mo_load_firmware(struct m10mo_device *m10mo_dev, const char * name)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	const struct firmware *fw;
	int i, ret;
	u16 *fw_ptr, csum = 0;

	ret = request_firmware(&fw, name, &client->dev);
	if (ret) {
		dev_err(&client->dev,
			"Error %d while requesting firmware %s\n",
			ret, name);
		return NULL;
	}

	if (!(fw->size == FW_SIZE || fw->size == FW_INFO_SIZE || fw->size == FW_SHD_SIZE || \
	      fw->size == FW_IQ_CAPTURE_SIZE)) {
		dev_err(&client->dev,
			"Illegal FW size detected  %s %x\n",name, (int)fw->size);
		release_firmware(fw);
		return NULL;
	}

	if(fw->size == FW_SIZE){
		fw_ptr = (u16 *)fw->data;
		for (i = 0; i < FW_SIZE/2; i++, fw_ptr++)
			csum += be16_to_cpup(fw_ptr);

		if (csum) {
			dev_err(&client->dev,
				"Illegal %s FW csum: %d\n", name, csum);
		}
	}

	return fw;
}

int m10mo_isp_fw_SHD_W(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -ENODEV;
	u32 i,j;

	const struct firmware *fw_shd = NULL;

	fw_shd = m10mo_load_firmware(m10mo_dev, M10MO_SHD_NAME);
	if (!fw_shd)
		return -ENOENT;
	dev_info(&client->dev, "load %s ok!\n", M10MO_SHD_NAME);

	for (i = SHD_ADDRRESS, j = 0 ; i < SHD_ADDRRESS+FW_SHD_SIZE; i = i + FLASH_BLOCK_SIZE,j = j + FLASH_BLOCK_SIZE) {
		ret = m10mo_SHD_write_block(m10mo_dev,
			i, (u8 *)&fw_shd->data[j],
			FLASH_BLOCK_SIZE);
		if (ret) {
			dev_err(&client->dev, "Flash write failed\n");
			goto release_fw;
		}
	}

	ret = 0;

release_fw:
	release_firmware(fw_shd);
	return ret;
}

int m10mo_isp_fw_IQ_W(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -ENODEV;
	u32 addr,i,count;

	const struct firmware *fw_shd = NULL;

	fw_shd = m10mo_load_firmware(m10mo_dev, M10MO_IQ_NAME);
	if (!fw_shd)
		return -ENOENT;
	dev_info(&client->dev, "load %s ok!\n", M10MO_IQ_NAME);

	addr = IQ_CAC_ADDRRESS;
	count = FW_IQ_CAPTURE_SIZE / FW_IQ_CAPTURE_OFFSET;
	for (i = 0 ;i < count ; i = i + 1) {
		ret = m10mo_IQ_write_block(m10mo_dev,
			addr, (u8 *)&fw_shd->data[i*FW_IQ_CAPTURE_OFFSET]);
		if (ret) {
			dev_err(&client->dev, "m10mo Flash write failed\n");
			goto release_fw;
		}
		addr += FW_IQ_CAPTURE_OFFSET;
	}

	ret = 0;

release_fw:
	release_firmware(fw_shd);
	return ret;
}

int m10mo_program_device_erase_all_flash_all(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -ENODEV;
	//char fw_version[16];
	int i = 0;
#ifdef NEW_FLASHFW_FLOW
	const struct firmware *fw_info;
#endif
	const struct firmware *fw;

	dev_info(&client->dev, "=======Start FW update=======\n");

	    fw = m10mo_load_firmware(m10mo_dev, M10MO_FW_NAME);
	    if (!fw)
		return -ENOENT;
	    dev_info(&client->dev, "load %s ok!\n", M10MO_FW_NAME);


#ifdef NEW_FLASHFW_FLOW
	fw_info = m10mo_load_firmware(m10mo_dev, "FW_INFO.bin");
	if (!fw_info)
		return -ENOENT;
	dev_info(&client->dev, "load %s ok!\n", "FW_INFO.bin");
#endif
	ret = m10mo_to_fw_access_mode(m10mo_dev);
	if (ret)
		goto release_fw;
	dev_info(&client->dev, "m10mo_to_fw_access_mode ok\n");


	dev_info(&client->dev, "m10mo_chip_erase_flash start--------\n");
	ret = m10mo_chip_erase_flash(m10mo_dev);
	if (ret) {
		dev_err(&client->dev, "Erase failed\n");
		goto release_fw;
	}
	dev_info(&client->dev, "m10mo_chip_erase_flash end----------\n");


	if (m10mo_dev->spi && m10mo_dev->spi->spi_enabled) {
		ret = m10mo_sio_write(m10mo_dev, (u8 *)fw->data, 0x1A2000, 0);
		if (ret) {
			dev_err(&client->dev, "Flash write FW_DATA failed\n");
			goto release_fw;
		}
		m10mo_dev->pdata->common.gpio_ctrl(sd, 0);
		mdelay(1);
		m10mo_dev->pdata->common.gpio_ctrl(sd, 1);
		mdelay(1);
		ret = m10mo_sio_write(m10mo_dev, (u8 *)&fw_info->data[0], 0x80, 0x1ff000);
		if (ret) {
			dev_err(&client->dev, "Flash write FW_INFO failed\n");
			goto release_fw;
		}
	} else {

		for (i = 0 ; i < FW_SIZE; i = i + FLASH_BLOCK_SIZE) {
			dev_err(&client->dev, "Writing block %d\n", i / FLASH_BLOCK_SIZE);
			ret = m10mo_flash_write_block(m10mo_dev,
						      i, (u8 *)&fw->data[i],
						      FLASH_BLOCK_SIZE);
			if (ret) {
				dev_err(&client->dev, "Flash write failed\n");
				goto release_fw;
			}
		}
	}

	dev_info(&client->dev, "=======Flashing done=======\n");
	msleep(50);

	ret = 0;

release_fw:
	release_firmware(fw);
	return ret;
}


int m10mo_program_device_erase_all_flash_partial(struct m10mo_device *m10mo_dev)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -ENODEV;
	//char fw_version[16];
	int i = 0;
#ifdef NEW_FLASHFW_FLOW
	const struct firmware *fw_info;
#endif
	const struct firmware *fw;

	dev_info(&client->dev, "=======Start FW update=======\n");

	    fw = m10mo_load_firmware(m10mo_dev, M10MO_FW_NAME);
	    if (!fw)
		return -ENOENT;
	    dev_info(&client->dev, "load %s ok!\n", M10MO_FW_NAME);


#ifdef NEW_FLASHFW_FLOW
	fw_info = m10mo_load_firmware(m10mo_dev, "FW_INFO.bin");
	if (!fw_info)
		return -ENOENT;
	dev_info(&client->dev, "load %s ok!\n", "FW_INFO.bin");
#endif
	ret = m10mo_to_fw_access_mode(m10mo_dev);
	if (ret)
		goto release_fw;
	dev_info(&client->dev, "m10mo_to_fw_access_mode ok\n");


	dev_info(&client->dev, "m10mo_chip_erase_flash start--------\n");
	ret = m10mo_chip_erase_flash(m10mo_dev);
	if (ret) {
		dev_err(&client->dev, "Erase failed\n");
		goto release_fw;
	}
	dev_info(&client->dev, "m10mo_chip_erase_flash end----------\n");


	if (m10mo_dev->spi && m10mo_dev->spi->spi_enabled) {
		ret = m10mo_sio_write(m10mo_dev, (u8 *)fw->data, 0x1A2000, 0);
		if (ret) {
			dev_err(&client->dev, "Flash write FW_DATA failed\n");
			goto release_fw;
		}
		m10mo_dev->pdata->common.gpio_ctrl(sd, 0);
		mdelay(1);
		m10mo_dev->pdata->common.gpio_ctrl(sd, 1);
		mdelay(1);
		ret = m10mo_sio_write(m10mo_dev, (u8 *)&fw_info->data[0], 0x80, 0x1ff000);
		if (ret) {
			dev_err(&client->dev, "Flash write FW_INFO failed\n");
			goto release_fw;
		}
	} else {
#ifndef NEW_FLASHFW_FLOW
		for (i = 0 ; i < FW_SIZE; i = i + FLASH_BLOCK_SIZE) {
			dev_err(&client->dev, "Writing block %d\n", i / FLASH_BLOCK_SIZE);
			ret = m10mo_flash_write_block(m10mo_dev,
						      i, (u8 *)&fw->data[i],
						      FLASH_BLOCK_SIZE);
			if (ret) {
				dev_err(&client->dev, "Flash write failed\n");
				goto release_fw;
			}
		}
#else
	dev_info(&client->dev, "======Start to write=======\n\n");

	dev_err(&client->dev, "write block=26 size=1664k bytes\n");
	for (i = 0 ; i < 0x1A0000; i = i + FLASH_BLOCK_SIZE) {
		dev_err(&client->dev, "count=%d, Writing block addr=%x\n",i / FLASH_BLOCK_SIZE, i);
		ret = m10mo_flash_write_block(m10mo_dev,
			i, (u8 *)&fw->data[i],
			FLASH_BLOCK_SIZE);
		if (ret) {
			dev_err(&client->dev, "Flash write failed\n");
			goto release_fw;
		}
	}

	dev_err(&client->dev, "write block=26 size=1664k bytes ok\n");
	dev_info(&client->dev, "==================================\n");
	dev_err(&client->dev, "write sector=2 size=8k bytes\n");
	dev_err(&client->dev, "Writing sector addr=0x1A0000 size=0x2000\n");
	ret = m10mo_flash_write_block(m10mo_dev,
			0x1A0000, (u8 *)&fw->data[0x1A0000],
			0x2000);//8k bytes
	if (ret) {
		dev_err(&client->dev, "Flash write failed\n");
		goto release_fw;
	}
#if 1
	dev_err(&client->dev, "write sector=2 size=8k bytes ok\n");
	dev_info(&client->dev, "==================================\n");
	dev_err(&client->dev, "write sector=1 size=128 bytes\n");
	dev_err(&client->dev, "Writing sector addr=0x1ff000 size=0x80\n");
	ret = m10mo_flash_write_block(m10mo_dev,
			0x1ff000, (u8 *)&fw_info->data[0],
			0x80);//128 bytes
	if (ret) {
		dev_err(&client->dev, "Flash write failed\n");
		goto release_fw;
	}
	dev_err(&client->dev, "write sector=1 size=128 bytes ok\n");
#endif
#endif
	}

	dev_info(&client->dev, "=======Flashing done=======\n");
	msleep(50);

	ret = 0;

release_fw:
	release_firmware(fw);
	return ret;
}

int m10mo_program_device(struct m10mo_device *m10mo_dev, int autoupdate ,u32 version, u32 dit_version)
{
	struct v4l2_subdev *sd = &m10mo_dev->sd;
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = -ENODEV;
	u32 i, dit_fw_file_version=0, sni_fw_file_version=0;
	//char fw_version[16];

#ifdef NEW_FLASHFW_FLOW
	int j, start_addr;
	const struct firmware *fw_info;
#endif
	const struct firmware *fw;

	dev_info(&client->dev, "=======Start FW update=======\n");

	if(autoupdate == 0){
	    fw = m10mo_load_firmware(m10mo_dev, M10MO_FW_NAME);
	    if (!fw)
		return -ENOENT;
	    dev_info(&client->dev, "load %s ok!\n", M10MO_FW_NAME);
	}else{

	    if(readFwFileVersion(&dit_fw_file_version, &sni_fw_file_version))
		return -ENOENT;
	    else{
		dev_info(&client->dev, "m10mo version : %x-%x\n", version, dit_version);
		dev_info(&client->dev, "m10mo read /etc/firmware/M10MO.version : %x-%x\n", sni_fw_file_version, dit_fw_file_version);
	    }
#ifdef ZX551ML_USER_BUILD
	    if((sni_fw_file_version==version) && (dit_fw_file_version==dit_version))
	    {
		return -ECANCELED;
	    }
#else
	    if(sni_fw_file_version < version){
		dev_info(&client->dev, "%x < %x : exit!\n", sni_fw_file_version, version);
		return -ECANCELED;
	    }else if(sni_fw_file_version == version && dit_fw_file_version <= dit_version){
		dev_info(&client->dev, "%x < %x : exit!\n", dit_fw_file_version, dit_version);
		return -ECANCELED;
	    }
#endif
	    else{
	    	fw = m10mo_load_firmware(m10mo_dev, M10MO_DIT_FW_NAME);
	    	if (!fw){
		    dev_info(&client->dev, " load %s fail!\n", M10MO_DIT_FW_NAME);
		    return -ENOENT;
		}else{
		    dev_info(&client->dev, "load %s ok!\n", M10MO_DIT_FW_NAME);
		    }
		}
	    }

#ifdef NEW_FLASHFW_FLOW
	fw_info = m10mo_load_firmware(m10mo_dev, "FW_INFO.bin");
	if (!fw_info)
		return -ENOENT;
	dev_info(&client->dev, "load %s ok!\n", "FW_INFO.bin");
#endif
	ret = m10mo_to_fw_access_mode(m10mo_dev);
	if (ret)
		goto release_fw;
	dev_info(&client->dev, "m10mo_to_fw_access_mode ok\n");
#ifndef NEW_FLASHFW_FLOW
	ret = m10mo_chip_erase_flash(m10mo_dev);
	if (ret) {
		dev_err(&client->dev, "Erase failed\n");
		goto release_fw;
	}
#else
	dev_info(&client->dev, "======Start to erase=======\n\n");

	start_addr = 0x0;
	dev_info(&client->dev, "erase block=26 addr=%x size=1664k bytes\n", start_addr);
	for(j=0;j<26;j++){
		dev_info(&client->dev, "count=%d erase block addr=%x size=0x10000\n", j+1,start_addr);
		ret = m10mo_block_erase_flash(m10mo_dev, start_addr, j);
		start_addr += 0x10000;//64k byte
		if (ret) {
			dev_err(&client->dev, "addr=%x Erase block failed\n", start_addr);
			goto release_fw;
		}
	}
	dev_err(&client->dev, "erase block=26 size=1664k bytes ok\n");
	dev_info(&client->dev, "==================================\n");
	start_addr = 0x1A0000;
	dev_info(&client->dev, "erase sector=2 addr=%x size=8k bytes\n", start_addr);
	for(j=0;j<2;j++){
		dev_info(&client->dev, "count=%d erase sector addr=%x size=0x1000\n", j+1,start_addr);
		ret = m10mo_sector_erase_flash(m10mo_dev, start_addr);
		start_addr += 0x1000;
		if (ret) {
			dev_err(&client->dev, "addr=%x Erase sector failed\n", start_addr);
			goto release_fw;
		}
	}
	dev_err(&client->dev, "erase sector=2 size=8k bytes ok\n");
	dev_info(&client->dev, "==================================\n");
	start_addr = 0x1ff000;
	dev_info(&client->dev, "erase sector addr=%x size=4k bytes\n", start_addr);
	dev_info(&client->dev, "count=0 erase sector addr=%x size=0x1000\n",start_addr);
	ret = m10mo_sector_erase_flash(m10mo_dev, start_addr);
	if (ret) {
		dev_err(&client->dev, "addr=%x Erase sector failed\n", start_addr);
		goto release_fw;
	}
	dev_err(&client->dev, "erase sector=1 size=4k bytes ok\n");

#endif
	if (m10mo_dev->spi && m10mo_dev->spi->spi_enabled) {
		ret = m10mo_sio_write(m10mo_dev, (u8 *)fw->data, 0x1A2000, 0);
		if (ret) {
			dev_err(&client->dev, "Flash write FW_DATA failed\n");
			goto release_fw;
		}
		m10mo_dev->pdata->common.gpio_ctrl(sd, 0);
		mdelay(1);
		m10mo_dev->pdata->common.gpio_ctrl(sd, 1);
		mdelay(1);
		ret = m10mo_sio_write(m10mo_dev, (u8 *)&fw_info->data[0], 0x80, 0x1ff000);
		if (ret) {
			dev_err(&client->dev, "Flash write FW_INFO failed\n");
			goto release_fw;
		}
	} else {
#ifndef NEW_FLASHFW_FLOW
		for (i = 0 ; i < FW_SIZE; i = i + FLASH_BLOCK_SIZE) {
			dev_err(&client->dev, "Writing block %d\n", i / FLASH_BLOCK_SIZE);
			ret = m10mo_flash_write_block(m10mo_dev,
						      i, (u8 *)&fw->data[i],
						      FLASH_BLOCK_SIZE);
			if (ret) {
				dev_err(&client->dev, "Flash write failed\n");
				goto release_fw;
			}
		}
#else
	dev_info(&client->dev, "======Start to write=======\n\n");

	dev_err(&client->dev, "write block=26 size=1664k bytes\n");
	for (i = 0 ; i < 0x1A0000; i = i + FLASH_BLOCK_SIZE) {
		dev_err(&client->dev, "count=%d, Writing block addr=%x\n",i / FLASH_BLOCK_SIZE, i);
		ret = m10mo_flash_write_block(m10mo_dev,
			i, (u8 *)&fw->data[i],
			FLASH_BLOCK_SIZE);
		if (ret) {
			dev_err(&client->dev, "Flash write failed\n");
			goto release_fw;
		}
	}

	dev_err(&client->dev, "write block=26 size=1664k bytes ok\n");
	dev_info(&client->dev, "==================================\n");
	dev_err(&client->dev, "write sector=2 size=8k bytes\n");
	dev_err(&client->dev, "Writing sector addr=0x1A0000 size=0x2000\n");
	ret = m10mo_flash_write_block(m10mo_dev,
			0x1A0000, (u8 *)&fw->data[0x1A0000],
			0x2000);//8k bytes
	if (ret) {
		dev_err(&client->dev, "Flash write failed\n");
		goto release_fw;
	}
#if 1
	dev_err(&client->dev, "write sector=2 size=8k bytes ok\n");
	dev_info(&client->dev, "==================================\n");
	dev_err(&client->dev, "write sector=1 size=128 bytes\n");
	dev_err(&client->dev, "Writing sector addr=0x1ff000 size=0x80\n");
	ret = m10mo_flash_write_block(m10mo_dev,
			0x1ff000, (u8 *)&fw_info->data[0],
			0x80);//128 bytes
	if (ret) {
		dev_err(&client->dev, "Flash write failed\n");
		goto release_fw;
	}
	dev_err(&client->dev, "write sector=1 size=128 bytes ok\n");
#endif
#endif
	}

	dev_info(&client->dev, "=======Flashing done=======\n");
	msleep(50);

	ret = 0;

release_fw:
	release_firmware(fw);
	return ret;
}

int m10mo_get_spi_state(struct m10mo_device *m10mo_dev)
{
	if (m10mo_dev->spi && m10mo_dev->spi->spi_enabled)
		return 1;
	return 0;
}

int m10mo_set_spi_state(struct m10mo_device *m10mo_dev, bool enabled)
{
	if (m10mo_dev->spi) {
		m10mo_dev->spi->spi_enabled = !!enabled;
		return 0;
	}
	return -ENODEV;
}

void m10mo_register_spi_fw_flash_interface(struct m10mo_device *dev,
					   struct m10mo_spi *m10mo_spi_dev)
{
	pr_debug("m10mo: Spi interface registered\n");
	dev->spi = m10mo_spi_dev;
}
EXPORT_SYMBOL_GPL(m10mo_register_spi_fw_flash_interface);



bool isNeedUpdateFwWhenBoot(void){

#ifdef CONFIG_ASUS_FACTORY_MODE
	pr_err("m10mo: FACTORY mode\n");
	pr_err("m10mo: isNeedUpdateFwWhenBoot = no\n");
	return false;
#else
	pr_err("m10mo: NOT FACTORY mode\n");
	pr_err("m10mo: isNeedUpdateFwWhenBoot = yes\n");
	return true;
#endif

}

EXPORT_SYMBOL_GPL(isNeedUpdateFwWhenBoot);
//bit0 is update flag
// 0 --> enable
// 1 --> disable
int readFwForceUpdateFlag(struct m10mo_device *m10mo_dev){

	int ret=0,res=0,timeout=5000;
    //	(void)__m10mo_param_mode_set(&m10mo_dev->sd);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_FLASH_MODE, 0x01);
	do {
		msleep(10);
		m10mo_readb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, SFLASH_SPI_STATUS, &res);
	} while ((res != 0) && --timeout);

	if (!timeout) {
		pr_err("timeout while waiting for chip op to finish\n");
		return 0;
	}
	(void) m10mo_writew(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H,0xf4);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_MODE, 0x01);
	(void) m10mo_readb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, &ret);
	printk("m10mo readFwForceUpdateFlag = %x\n",ret);

	return ret;

}
EXPORT_SYMBOL_GPL(readFwForceUpdateFlag);

int readDitVersion(struct m10mo_device *m10mo_dev){

	int ret = 0;

	(void) m10mo_writew(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H,0xf8);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_MODE, 0x04);
	(void) m10mo_readl(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, &ret);
	printk("m10mo readDitVersion = %x\n",ret);

	return ret;

}
EXPORT_SYMBOL_GPL(readDitVersion);

//M10MO_FW_VERSION
int readFwFileVersion(u32 *dit_version, u32 *sni_version)
{
	struct file *fp = NULL;
	mm_segment_t old_fs;
	loff_t pos_lsts = 0;
	char buf[16],buf2[16];
	int readlen = 0;



	fp = filp_open(M10MO_FW_VERSION, O_RDONLY, S_IRWXU | S_IRWXG | S_IRWXO);
	if (IS_ERR_OR_NULL(fp)) {
		pr_err("m10mo read version file (%s) fail\n", M10MO_FW_VERSION);
		return -ENOENT;	/*No such file or directory*/
	}


	/*For purpose that can use read/write system call*/
	old_fs = get_fs();
	set_fs(KERNEL_DS);

	if (fp->f_op != NULL && fp->f_op->read != NULL) {

		if((int)fp->f_dentry->d_inode->i_size != 10)
		    return -EINVAL;

		pos_lsts = 0;
		readlen = fp->f_op->read(fp, buf, 4, &pos_lsts);
		buf[readlen] = '\0';

		pos_lsts = 5;
		readlen = fp->f_op->read(fp, buf2, 11, &pos_lsts);
		buf2[readlen] = '\0';


	} else {
		set_fs(old_fs);
		filp_close(fp, NULL);
		pr_err("m10mo read version file: f_op=NULL or op->read=NULL\n");
		return -ENXIO;	/*No such device or address*/
	}
	set_fs(old_fs);
	filp_close(fp, NULL);

	sscanf(buf2, "%x", dit_version);
	sscanf(buf, "%x", sni_version);
	return 0;
}



int writeDummyVersion(struct m10mo_device *m10mo_dev){

	int ret=0, res=0, timeout=5000;

	(void) m10mo_writew(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_OFFSET_H, 0xf8);


	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE0, 0x0);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE1, 0x0);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE2, 0x0);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_DATA_BYTE3, 0x0);

	if(ret == 1){
		pr_err("timeout while waiting for chip op to finish\n");
		return 0xffff;
	}
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_RW_MODE, 0x0c);
	(void) m10mo_writeb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, FADJ_FLASH_MODE, 0x03);

	do {
		msleep(10);
		m10mo_readb(&m10mo_dev->sd, CATEGORY_LOGLEDFLASH, SFLASH_SPI_STATUS, &res);
	} while ((res != 0) && --timeout);

	if (!timeout) {
		pr_err("timeout while waiting for chip op to finish\n");
		return 0xffff;
	}
	printk("flash done\n");
	return 0;
}
EXPORT_SYMBOL_GPL(writeDummyVersion);

int m10mo_USB_status(bool status)
{
	struct file *fp;
	mm_segment_t old_fs;
	int err=0;
	
	if(m10mo_usb_state == status) return err;
	else m10mo_usb_state = status;

	printk("m10mo Begin set %s to on\n", USB_DEV_PATH);

	old_fs = get_fs();
	set_fs(KERNEL_DS);

	fp = filp_open(USB_DEV_PATH,
		O_WRONLY|O_CREAT|O_TRUNC, S_IRUGO|S_IWUGO|S_IXUSR);
	if (IS_ERR(fp)) {
		printk("m10mo failed to open %s, err %ld\n",
			USB_DEV_PATH, PTR_ERR(fp));
		err = -ENOENT;
		goto out_file;
	}
	if(status == 0)
		vfs_write(fp, "auto\n", sizeof("auto\n"), &fp->f_pos);
	else
		vfs_write(fp, "on\n", sizeof("on\n"), &fp->f_pos);
	printk("m10mo end set %s to on\n", USB_DEV_PATH);

	if (!IS_ERR(fp))
		filp_close(fp, current->files);
out_file:
	set_fs(old_fs);
	msleep(2000);
	return err;
}
EXPORT_SYMBOL(m10mo_USB_status);

