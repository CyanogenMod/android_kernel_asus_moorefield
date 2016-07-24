/*
 * platform_a1026.c: a1026 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/a1026.h>
#include <asm/intel-mid.h>
#include "platform_a1026.h"

static int audience_request_resources(struct i2c_client *client)
{
	struct a1026_platform_data *pdata = (struct a1026_platform_data *)
		client->dev.platform_data;
	int ret;

	pr_debug("Audience: request ressource audience\n");
	if (!pdata)
		return -1;
	ret = gpio_request(pdata->gpio_a1026_wakeup, AUDIENCE_WAKEUP_GPIO);
	if (ret) {
		dev_err(&client->dev, "Request AUDIENCE WAKEUP GPIO %d fails %d\n",
			pdata->gpio_a1026_wakeup, ret);
		return -1;
	}
	ret = gpio_direction_output(pdata->gpio_a1026_wakeup, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_wake;
	}
	ret = gpio_request(pdata->gpio_a1026_reset, AUDIENCE_RESET_GPIO);
	if (ret) {
		dev_err(&client->dev,
				"Request for Audience reset GPIO %d fails %d\n",
					pdata->gpio_a1026_reset, ret);
		goto err_wake;
	}
	ret = gpio_direction_output(pdata->gpio_a1026_reset, 0);
	if (ret) {
		dev_err(&client->dev, "Set GPIO Direction fails %d\n", ret);
		goto err_reset;
	}
	return 0;
err_reset:
	gpio_free(pdata->gpio_a1026_reset);
err_wake:
	gpio_free(pdata->gpio_a1026_wakeup);
	return -1;
}

static void audience_free_resources(struct i2c_client *client)
{
	struct a1026_platform_data *pdata = (struct a1026_platform_data *)
		&client->dev.platform_data;

	gpio_free(pdata->gpio_a1026_wakeup);
	gpio_free(pdata->gpio_a1026_reset);
}

static void audience_wake_up(bool state)
{
	int wakeup_gpio;

	wakeup_gpio = get_gpio_by_name(AUDIENCE_WAKEUP_GPIO);
	if (wakeup_gpio == -1) {
		pr_err("%s invalid wakeup gpio", __func__);
		return;
	}
	gpio_set_value(wakeup_gpio, state);
	pr_debug("Audience: WAKE UP %d\n", state);
}

static void audience_reset(bool state)
{
	int reset_gpio;

	reset_gpio = get_gpio_by_name(AUDIENCE_RESET_GPIO);
	if (reset_gpio == -1) {
		pr_err("%s invalid reset gpio", __func__);
		return;
	}
	gpio_set_value(reset_gpio, state);
	pr_debug("Audience: RESET %d\n", state);
}

void *audience_platform_data(void *info)
{
	static struct a1026_platform_data pdata;

	pdata.gpio_a1026_wakeup = get_gpio_by_name(AUDIENCE_WAKEUP_GPIO);
	pdata.gpio_a1026_reset = get_gpio_by_name(AUDIENCE_RESET_GPIO);
	pdata.request_resources	= audience_request_resources;
	pdata.free_resources	= audience_free_resources;
	pdata.wakeup			= audience_wake_up;
	pdata.reset			= audience_reset;

	return &pdata;
}
