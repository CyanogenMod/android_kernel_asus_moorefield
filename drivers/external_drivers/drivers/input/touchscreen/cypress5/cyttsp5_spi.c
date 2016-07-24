/*
 * cyttsp5_spi.c
 * Cypress TrueTouch(TM) Standard Product V5 SPI Module.
 * For use with Cypress Txx5xx parts.
 * Supported parts include:
 * TMA5XX
 *
 * Copyright (C) 2012-2013 Cypress Semiconductor
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2, and only version 2, as published by the
 * Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * Contact Cypress Semiconductor at www.cypress.com <ttdrivers@cypress.com>
 *
 */

#include <asm/unaligned.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/spi/spi.h>
#include <linux/delay.h>
#include <linux/mutex.h>
#include <linux/of_device.h>
#include <linux/pm_runtime.h>
#include <linux/hrtimer.h>
#include <linux/cyttsp5_bus.h>
#include <linux/cyttsp5_core.h>
#include "cyttsp5_spi.h"

#include "cyttsp5_devtree.h"

#define CY_SPI_WR_OP		0x00 /* r/~w */
#define CY_SPI_RD_OP		0x01
#define CY_SPI_BITS_PER_WORD	8
#define CY_SPI_SYNC_ACK         0x62

struct cyttsp5_spi {
	struct spi_device *client;
	char const *id;
	struct mutex lock;
};

static void cyttsp5_spi_add_rw_msg(struct spi_message *msg,
		struct spi_transfer *xfer, u8 *w_header, u8 *r_header, u8 op)
{
	xfer->tx_buf = w_header;
	xfer->rx_buf = r_header;
	w_header[0] = op;
	xfer->len = 1;
	spi_message_add_tail(xfer, msg);
}

static int cyttsp5_spi_xfer(u8 op, struct cyttsp5_spi *ts, u8 *buf, int length)
{
	struct device *dev = &ts->client->dev;
	struct spi_message msg;
	struct spi_transfer xfer[2];
	u8 w_header[2];
	u8 r_header[2];
	int rc;

	memset(xfer, 0, sizeof(xfer));

	spi_message_init(&msg);
	cyttsp5_spi_add_rw_msg(&msg, &xfer[0], w_header, r_header, op);

	switch (op) {
	case CY_SPI_RD_OP:
		xfer[1].rx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
		break;
	case CY_SPI_WR_OP:
		xfer[1].tx_buf = buf;
		xfer[1].len = length;
		spi_message_add_tail(&xfer[1], &msg);
		break;
	default:
		rc = -EIO;
		goto exit;
	}

	rc = spi_sync(ts->client, &msg);
exit:
	if (rc < 0)
		dev_vdbg(dev, "%s: spi_sync() error %d\n", __func__, rc);

	if (r_header[0] != CY_SPI_SYNC_ACK)
		return -EIO;

	return rc;
}

static int cyttsp5_spi_read_default_(struct cyttsp5_adapter *adap,
	void *buf, int size)
{
	struct cyttsp5_spi *ts = dev_get_drvdata(adap->dev);
	int rc;

	if (!buf || !size)
		return 0;

	rc = cyttsp5_spi_xfer(CY_SPI_RD_OP, ts, buf, size);

	return rc;
}

static int cyttsp5_spi_read_default(struct cyttsp5_adapter *adap,
	void *buf, int size)
{
	struct cyttsp5_spi *ts = dev_get_drvdata(adap->dev);
	int rc;
	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp5_spi_read_default_(adap, buf, size);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);
	return rc;
}

static int cyttsp5_spi_read_default_nosize_(struct cyttsp5_adapter *adap,
	u8 *buf, int max)
{
	struct cyttsp5_spi *ts = dev_get_drvdata(adap->dev);
	int size;
	int rc;

	if (!buf)
		return 0;

	rc = cyttsp5_spi_xfer(CY_SPI_RD_OP, ts, buf, 2);
	if (rc < 0)
		return rc;

	size = get_unaligned_le16(&buf[0]);
	if (!size)
		return rc;

	if (size > max)
		return -EINVAL;

	rc = cyttsp5_spi_read_default_(adap, buf, size);

	return rc;
}

static int cyttsp5_spi_read_default_nosize(struct cyttsp5_adapter *adap,
	void *buf, int max)
{
	struct cyttsp5_spi *ts = dev_get_drvdata(adap->dev);
	int rc;
	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp5_spi_read_default_nosize_(adap, buf, max);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);
	return rc;
}

static int cyttsp5_spi_write_read_specific_(struct cyttsp5_adapter *adap,
		u8 write_len, u8 *write_buf, u8 *read_buf)
{
	struct cyttsp5_spi *ts = dev_get_drvdata(adap->dev);
	int rc;

	rc = cyttsp5_spi_xfer(CY_SPI_WR_OP, ts, write_buf, write_len);
	if (rc < 0)
		return rc;

	if (read_buf)
		rc = cyttsp5_spi_read_default_nosize_(adap, read_buf, 512);

	return rc;
}

static int cyttsp5_spi_write_read_specific(struct cyttsp5_adapter *adap,
		u8 write_len, u8 *write_buf, u8 *read_buf)
{
	struct cyttsp5_spi *ts = dev_get_drvdata(adap->dev);
	int rc;
	pm_runtime_get_noresume(adap->dev);
	mutex_lock(&ts->lock);
	rc = cyttsp5_spi_write_read_specific_(adap, write_len, write_buf,
			read_buf);
	mutex_unlock(&ts->lock);
	pm_runtime_put_noidle(adap->dev);
	return rc;
}

static struct cyttsp5_ops ops = {
	.read_default = cyttsp5_spi_read_default,
	.read_default_nosize = cyttsp5_spi_read_default_nosize,
	.write_read_specific = cyttsp5_spi_write_read_specific,
};

static struct of_device_id cyttsp5_spi_of_match[] = {
	{ .compatible = "cy,cyttsp5_spi_adapter", },
	{ }
};
MODULE_DEVICE_TABLE(of, cyttsp5_spi_of_match);

static int cyttsp5_spi_probe(struct spi_device *spi)
{
	struct cyttsp5_spi *ts_spi;
	int rc = 0;
	struct device *dev = &spi->dev;
	const struct of_device_id *match;
	char const *adap_id;

	spi->bits_per_word = CY_SPI_BITS_PER_WORD;
	spi->mode = SPI_MODE_0;

	rc = spi_setup(spi);
	if (rc < 0) {
		dev_err(dev, "%s: SPI setup error %d\n", __func__, rc);
		return rc;
	}

	ts_spi = kzalloc(sizeof(*ts_spi), GFP_KERNEL);
	if (ts_spi == NULL) {
		dev_err(dev, "%s: Error, kzalloc\n", __func__);
		rc = -ENOMEM;
		goto error_alloc_data_failed;
	}

	match = of_match_device(of_match_ptr(cyttsp5_spi_of_match), dev);
	if (match) {
		rc = of_property_read_string(dev->of_node, "cy,adapter_id",
				&adap_id);
		if (rc) {
			dev_err(dev, "%s: OF error rc=%d\n", __func__, rc);
			goto error_free_data;
		}
		cyttsp5_devtree_register_devices(dev);
	} else {
		adap_id = dev_get_platdata(dev);
	}

	mutex_init(&ts_spi->lock);
	ts_spi->client = spi;
	ts_spi->id = (adap_id) ? adap_id : CYTTSP5_SPI_NAME;
	dev_set_drvdata(&spi->dev, ts_spi);

	dev_dbg(dev, "%s: add adap='%s' (CYTTSP5_SPI_NAME=%s)\n", __func__,
		ts_spi->id, CYTTSP5_SPI_NAME);

	pm_runtime_enable(&spi->dev);

	rc = cyttsp5_add_adapter(ts_spi->id, &ops, dev);
	if (rc) {
		dev_err(dev, "%s: Error on probe %s\n", __func__,
			CYTTSP5_SPI_NAME);
		goto add_adapter_err;
	}

	return 0;

add_adapter_err:
	pm_runtime_disable(&spi->dev);
	dev_set_drvdata(&spi->dev, NULL);
error_free_data:
	kfree(ts_spi);
error_alloc_data_failed:
	return rc;
}

static int cyttsp5_spi_remove(struct spi_device *spi)
{
	struct device *dev = &spi->dev;
	struct cyttsp5_spi *ts_spi = dev_get_drvdata(dev);

	cyttsp5_del_adapter(ts_spi->id);
	pm_runtime_disable(&spi->dev);
	dev_set_drvdata(&spi->dev, NULL);
	kfree(ts_spi);
	return 0;
}

static const struct spi_device_id cyttsp5_spi_id[] = {
	{ CYTTSP5_SPI_NAME, 0 },
	{ }
};
MODULE_DEVICE_TABLE(spi, cyttsp5_spi_id);

static struct spi_driver cyttsp5_spi_driver = {
	.driver = {
		.name = CYTTSP5_SPI_NAME,
		.bus = &spi_bus_type,
		.owner = THIS_MODULE,
		.of_match_table = cyttsp5_spi_of_match,
	},
	.probe = cyttsp5_spi_probe,
	.remove = cyttsp5_spi_remove,
	.id_table = cyttsp5_spi_id,
};

static int __init cyttsp5_spi_init(void)
{
	int err;

	err = spi_register_driver(&cyttsp5_spi_driver);
	pr_info("%s: Cypress TTSP v5 SPI Driver (Built %s) rc=%d\n",
		 __func__, CY_DRIVER_DATE, err);

	return err;
}
module_init(cyttsp5_spi_init);

static void __exit cyttsp5_spi_exit(void)
{
	spi_unregister_driver(&cyttsp5_spi_driver);
}
module_exit(cyttsp5_spi_exit);

MODULE_LICENSE("GPL");
MODULE_DESCRIPTION("Cypress TrueTouch(R) Standard Product SPI Driver");
MODULE_AUTHOR("Cypress Semiconductor <ttdrivers@cypress.com>");
