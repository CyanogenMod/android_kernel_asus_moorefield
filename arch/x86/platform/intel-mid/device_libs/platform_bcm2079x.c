/*
 * platform_bcm2079x.c: bcm2079x platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/nfc/bcm2079x.h>
#include <asm/intel-mid.h>
#include "platform_bcm2079x.h"

static int bcm2079x_nfc_request_resources(struct i2c_client *client)
{
	int ret;
	struct bcm2079x_pdata *platform_data;

	platform_data = client->dev.platform_data;

	ret = gpio_request(platform_data->irq_gpio, BCM_HOST_INT_GPIO);
	if (ret) {
		dev_err(&client->dev,
			"Request NFC IRQ GPIO fails %d\n", ret);
		return ret;
	}

	ret = gpio_direction_input(platform_data->irq_gpio);
	if (ret) {
		dev_err(&client->dev,
			"Set NFC IRQ GPIO direction fails %d\n", ret);
		goto err_irq;
	}

	ret = gpio_request(platform_data->en_gpio, BCM_ENABLE_GPIO);
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC EN GPIO fails %d\n", ret);
		goto err_irq;
	}

	ret = gpio_direction_output(platform_data->en_gpio, 0);
	if (ret) {
		dev_err(&client->dev,
			"Set NFC EN GPIO direction fails %d\n", ret);
		goto err_en;
	}

	ret = gpio_request(platform_data->wake_gpio, BCM_FW_WAKE_GPIO);
	if (ret) {
		dev_err(&client->dev,
			"Request for NFC WAKE GPIO fails %d\n", ret);
		goto err_en;
	}

	ret = gpio_direction_output(platform_data->wake_gpio, 0);
	if (ret) {
		dev_err(&client->dev,
			"Set NFC WAKE GPIO direction fails %d\n", ret);
		goto err_wake;
	}
	return 0;
err_wake:
	gpio_free(platform_data->wake_gpio);
err_en:
	gpio_free(platform_data->en_gpio);
err_irq:
	gpio_free(platform_data->irq_gpio);
	return ret;
}

void *bcm2079x_platform_data(void *info)
{
	struct i2c_board_info *i2c_info = (struct i2c_board_info *) info;
	static struct bcm2079x_pdata bcm2079x_nfc_platform_data;

	memset(&bcm2079x_nfc_platform_data, 0x00,
		sizeof(struct bcm2079x_pdata));

        i2c_info->addr = 0x76; //<Wade+> hard code temporily

	bcm2079x_nfc_platform_data.irq_gpio =
		get_gpio_by_name(BCM_HOST_INT_GPIO);
	if (bcm2079x_nfc_platform_data.irq_gpio == -1)
		return NULL;

	bcm2079x_nfc_platform_data.en_gpio =
		get_gpio_by_name(BCM_ENABLE_GPIO);
	if (bcm2079x_nfc_platform_data.en_gpio == -1)
		return NULL;

	bcm2079x_nfc_platform_data.wake_gpio =
		get_gpio_by_name(BCM_FW_WAKE_GPIO);
	if (bcm2079x_nfc_platform_data.wake_gpio == -1)
		return NULL;

	i2c_info->irq =	 bcm2079x_nfc_platform_data.irq_gpio + INTEL_MID_IRQ_OFFSET;

	bcm2079x_nfc_platform_data.request_resources =
		bcm2079x_nfc_request_resources;

	return &bcm2079x_nfc_platform_data;
}
