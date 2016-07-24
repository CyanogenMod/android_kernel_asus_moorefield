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
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/types.h>
#include <media/v4l2-device.h>
#include <media/m12mo_atomisp.h>
#include <linux/m12mo.h>

static inline int spi_xmit(struct spi_device *spi, const u8 *addr, const int len)
{
	int ret;
	struct spi_message msg;

	struct spi_transfer xfer = {
		.len = len,
		.tx_buf = addr,
		.bits_per_word = 8,
	};

	spi_message_init(&msg);
	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(spi, &msg);

	if (ret < 0)
		dev_err(&spi->dev, "error %d\n", ret);

	return ret;
}

static inline int spi_xmit_rx(struct spi_device *spi, u8 *in_buf, size_t len)
{
	int ret;
	u8 read_out_buf[2];

	struct spi_message msg;
	struct spi_transfer xfer = {
		.tx_buf = read_out_buf,
		.rx_buf = in_buf,
		.len	= len,
		.cs_change = 0,
	};

	spi_message_init(&msg);

	spi_message_add_tail(&xfer, &msg);

	ret = spi_sync(spi, &msg);

	if (ret < 0)
		dev_err(&spi->dev, "%s - error %d\n", __func__, ret);

	return ret;
}

/* TDB: Not tested */
int m12mo_spi_read(struct spi_device *spi, u8 *buf, size_t len,
		   const int rxSize)
{
	int k;
	int ret = 0;
	u8 temp_buf[4] = {0};
	u32 count = len / rxSize;
	u32 extra = len % rxSize;

	for (k = 0; k < count; k++) {
		ret = spi_xmit_rx(spi, &buf[rxSize * k], rxSize);
		if (ret < 0) {
			dev_err(&spi->dev, "%s - error %d\n", __func__, ret);
			return -EINVAL;
		}
	}

	if (extra != 0) {
		ret = spi_xmit_rx(spi, &buf[rxSize * k], extra);
		if (ret < 0) {
			dev_err(&spi->dev, "%s - error %d\n", __func__, ret);
			return -EINVAL;
		}
	}

	for (k = 0; k < len - 3; k += 4) {
		memcpy(temp_buf, (char *)&buf[k], sizeof(temp_buf));
		buf[k] = temp_buf[3];
		buf[k+1] = temp_buf[2];
		buf[k+2] = temp_buf[1];
		buf[k+3] = temp_buf[0];
	}
	return 0;
}

int m12mo_spi_write(struct spi_device *spi, const u8 *addr,
		    const int len, const int txSize)
{
	int i, j = 0;
	int ret = 0;
	u8 paddingData[8];
	u32 count = len / txSize;
	u32 extra = len % txSize;
	dev_err(&spi->dev, "Entered to spi write with count = %d extra = %d\n",
	       count, extra);

	for (i = 0 ; i < count ; i++) {
		ret = spi_xmit(spi, &addr[j], txSize);
		j += txSize;
		if (ret < 0) {
			dev_err(&spi->dev, "failed to write spi_xmit, count = %d\n", i);
			goto exit_err;
		}
	}

	if (extra) {
		ret = spi_xmit(spi, &addr[j], extra);
		if (ret < 0) {
			dev_err(&spi->dev, "failed to write spi_xmit\n");
			goto exit_err;
		}
	}

	for (i = 0; i < 4; i++) {
		memset(paddingData, 0, sizeof(paddingData));
		ret = spi_xmit(spi, paddingData, 8);
		if (ret < 0) {
			dev_err(&spi->dev, "failed to write spi_xmit\n");
			goto exit_err;
		}
	}
	dev_dbg(&spi->dev, "FW upload done!!\n");
exit_err:
	return ret;
}

static int m12mo_spi_probe(struct spi_device *spi)
{
	int ret = -ENODEV;
	struct m12mo_spi *m12mo_spi_dev;
	struct m12mo_atomisp_spi_platform_data *pdata;

	dev_dbg(&spi->dev, "Probe M12MO SPI\n");

	pdata = dev_get_platdata(&spi->dev);
	if (!pdata) {
		dev_err(&spi->dev, "Missing platform data. Can't continue");
		return -ENODEV;
	}
	if (!pdata->device_data) {
		dev_err(&spi->dev, "Missing link to m12mo main driver. Can't continue");
		return -ENODEV;
	}

	m12mo_spi_dev = kzalloc(sizeof(struct m12mo_spi), GFP_KERNEL);
	if (!m12mo_spi_dev) {
		dev_err(&spi->dev, "Can't get memory\n");
		return -ENOMEM;
	}

	if (spi_setup(spi)) {
		dev_err(&spi->dev, "failed to setup spi for m12mo_spi\n");
		ret = -EINVAL;
		goto err_setup;
	}

	spi_set_drvdata(spi, m12mo_spi_dev);
	m12mo_spi_dev->spi_device  = spi;
	m12mo_spi_dev->read        = m12mo_spi_read;
	m12mo_spi_dev->write       = m12mo_spi_write;
	m12mo_spi_dev->spi_enabled = pdata->spi_enabled;

	m12mo_register_spi_fw_flash_interface(pdata->device_data,
					      m12mo_spi_dev);

	dev_err(&spi->dev, "m12mo_spi successfully probed\n");

	return 0;
err_setup:
	kfree(m12mo_spi_dev);
	return ret;
}

static int m12mo_spi_remove(struct spi_device *spi)
{
	struct m12mo_spi *m12mo_spi_dev;
	m12mo_spi_dev = spi_get_drvdata(spi);
	kfree(m12mo_spi_dev);
	return 0;
}

static const struct spi_device_id m12mo_spi_id_table[] = {
	{ "m12mo_spi",	0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, m12mo_spi_id_table);

static struct spi_driver m12mo_spi_driver = {
	.driver = {
		.name	= "m12mo_spi",
		.owner	= THIS_MODULE,
	},
	.probe		= m12mo_spi_probe,
	.remove		= m12mo_spi_remove,
	.id_table	= m12mo_spi_id_table,
};
module_spi_driver(m12mo_spi_driver);

MODULE_LICENSE("GPL v2");
MODULE_AUTHOR("Samu Onkalo <samu.onkalo@intel.com>");
MODULE_DESCRIPTION("m12mo spi interface driver");

