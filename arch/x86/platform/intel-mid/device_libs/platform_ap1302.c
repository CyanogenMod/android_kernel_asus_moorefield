/*
 * platform_ap1302.c: ap1302 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi_gpio.h>
#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif
#include <media/v4l2-subdev.h>

#include "platform_camera.h"
#include "platform_ap1302.h"

#define OSC_CAM_CLK 0x1
#define CLK_19P2MHz 0x1
#define CLK_ON	0x1
#define CLK_OFF	0x2

#define VPROG_2P8V 0x5D
#define VPROG_ENABLE 0x63
#define VPROG_DISABLE 0x62

/* To be confirmed */
#define GPIO_BANK_PATH	"\\_SB.GPO1"
#define GPIO_RESET   50
#define GPIO_V1P8EN  55
#define GPIO_V1P2EN  49
#define GPIO_STANDBY 54

static int gpio_reset = -1;
static int gpio_standby = -1;
static int gpio_v1p8_en = -1;
static int gpio_v1p2_en = -1;

static int ap1302_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	if (gpio_standby == -1)
		return -ENODEV;
	gpio_set_value(gpio_standby, flag);

	return 0;
}

static int ap1302_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
/* Platform clock driver for CHT is not ready yet.
   Need to revise this part after the API is ready. */
#ifdef CONFIG_INTEL_SOC_PMC
	if (flag) {
		int ret;
		ret = pmc_pc_set_freq(OSC_CAM_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
	}
	return pmc_pc_configure(OSC_CAM_CLK, flag ? CLK_ON : CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CAM_CLK,
			flag ? clock_khz : 0);
#else
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	dev_err(&client->dev, "ap1302 clock is not set\n");
	return 0;
#endif
}

static int ap1302_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret = 0;

	if (gpio_reset == -1 ||
	    gpio_v1p8_en == -1 ||
	    gpio_v1p2_en == -1)
		return -ENODEV;

	if (flag) {
		ret = gpio_request(gpio_v1p8_en, "ap1302_v1p8_en");
		if (ret) {
			dev_err(&client->dev, "Request ap1302_v1p8_en failed.\n");
			return ret;
		}
		gpio_direction_output(gpio_v1p8_en, 0);
/* Will change to use VRF when it is ready on CHT. */
#ifdef CONFIG_CRYSTAL_COVE
		ret = intel_mid_pmic_writeb(VPROG_2P8V, VPROG_ENABLE);
		if (ret)
			dev_err(&client->dev,
				"Failed to enable 2.8V analog power.\n");
#endif

		gpio_set_value(gpio_reset, 0);

		gpio_set_value(gpio_v1p2_en, 1);

		usleep_range(200, 1000);

		gpio_set_value(gpio_v1p8_en, 1);

		usleep_range(200, 1000);

		ret = ap1302_flisclk_ctrl(sd, 1);
		if (ret)
			dev_err(&client->dev, "Failed to enable 19.2M clock\n");

		usleep_range(1, 1000);

		gpio_set_value(gpio_reset, 1);
	} else {
		gpio_set_value(gpio_reset, 0);

		ap1302_flisclk_ctrl(sd, 0);

		usleep_range(1, 1000);

		gpio_set_value(gpio_v1p8_en, 0);

		usleep_range(200, 1000);

		gpio_set_value(gpio_v1p2_en, 0);

#ifdef CONFIG_CRYSTAL_COVE
		intel_mid_pmic_writeb(VPROG_2P8V, VPROG_DISABLE);
#endif
		gpio_free(gpio_v1p8_en);
		gpio_v1p8_en = -1;
	}
	return ret;
}

static int ap1302_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
		ATOMISP_INPUT_FORMAT_YUV422_8, 0, flag);
}

static int ap1302_platform_init(struct i2c_client *client)
{
	int ret;
	gpio_reset = acpi_get_gpio(GPIO_BANK_PATH, GPIO_RESET);
	gpio_v1p8_en = acpi_get_gpio(GPIO_BANK_PATH, GPIO_V1P8EN);
	gpio_v1p2_en = acpi_get_gpio(GPIO_BANK_PATH, GPIO_V1P2EN);
	gpio_standby = acpi_get_gpio(GPIO_BANK_PATH, GPIO_STANDBY);
	if (gpio_reset < 0 ||
	    gpio_v1p8_en < 0 ||
	    gpio_v1p2_en < 0 ||
	    gpio_standby < 0) {
		dev_err(&client->dev,
			"Cannot get gpio pins. "
			"reset=%d v1p8_en=%d v1p2_en=%d standby=%d\n",
			gpio_reset, gpio_v1p8_en, gpio_v1p2_en, gpio_standby);
		return -ENODEV;
	}
	ret = gpio_request(gpio_reset, "ap1302_reset");
	if (ret) {
		dev_err(&client->dev, "Request ap1302_reset failed.\n");
		return -ENODEV;
	}
	gpio_direction_output(gpio_reset, 0);

	gpio_request(gpio_v1p2_en, "ap1302_v1p2_en");
	if (ret) {
		dev_err(&client->dev, "Request ap1302_v1p2_en failed.\n");
		goto fail_gpio_v1p2_en;
	}
	gpio_direction_output(gpio_v1p2_en, 0);

	gpio_request(gpio_standby, "ap1302_standby");
	if (ret) {
		dev_err(&client->dev, "Request ap1302_standby failed.\n");
		goto fail_gpio_standby;
	}
	gpio_direction_output(gpio_standby, 0);
	gpio_set_value(gpio_standby, 0);
	return 0;
fail_gpio_standby:
	gpio_free(gpio_v1p2_en);
	gpio_v1p2_en = -1;
fail_gpio_v1p2_en:
	gpio_free(gpio_reset);
	gpio_reset = -1;
	return -ENODEV;
}

static int ap1302_platform_deinit(void)
{
	if (gpio_reset >= 0)
		gpio_free(gpio_reset);
	if (gpio_v1p8_en >= 0)
		gpio_free(gpio_v1p8_en);
	if (gpio_v1p2_en >= 0)
		gpio_free(gpio_v1p2_en);
	if (gpio_standby >= 0)
		gpio_free(gpio_standby);
	gpio_reset = -1;
	gpio_v1p8_en = -1;
	gpio_v1p2_en = -1;
	gpio_standby = -1;
	return 0;
}

static struct atomisp_camera_caps ap1302_camera_caps;

static struct atomisp_camera_caps *ap1302_get_camera_caps(void)
{
	ap1302_camera_caps.sensor_num = 1;
	ap1302_camera_caps.sensor[0].stream_num = 2;
	return &ap1302_camera_caps;
}

static struct camera_sensor_platform_data ap1302_sensor_platform_data = {
	.gpio_ctrl      = ap1302_gpio_ctrl,
	.flisclk_ctrl   = ap1302_flisclk_ctrl,
	.power_ctrl     = ap1302_power_ctrl,
	.csi_cfg        = ap1302_csi_configure,
	.platform_init  = ap1302_platform_init,
	.platform_deinit = ap1302_platform_deinit,
	.get_camera_caps = ap1302_get_camera_caps,
};

void *ap1302_platform_data(void *info)
{
	return &ap1302_sensor_platform_data;
}
