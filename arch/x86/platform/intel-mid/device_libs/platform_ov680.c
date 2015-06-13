/*
 * platform_ov680.c: ov680 platform data initialization file
 *
 * (C) Copyright 2014 Intel Corporation
 * Author: xiaolin.zhang@intel.com
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <linux/acpi_gpio.h>
#include <linux/atomisp_platform.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/clk.h>
#include <media/v4l2-subdev.h>

#include "platform_camera.h"
#include "platform_fsa642.h"
#include "platform_ov680.h"

static int camera_reset = -1; /* GP_CAMERASB03 - CAM_2_3_RST_N - ALS_INT_N_R */
static int isp1_pwdn = -1; /* GP_CAMERASB08- FLASH_RST_N_R */
static struct atomisp_camera_caps ov680_camera_caps;

static int ov680_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	struct i2c_client *client = v4l2_get_subdevdata(sd);

	if (camera_reset < 0) { /* isp reset pin */
		ret = camera_sensor_gpio(-1, GP_CAMERA_2_3_RESET,
			GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		dev_dbg(&client->dev,
			"%s: camera reset gpio number: %d\n", __func__, ret);
		camera_reset = ret;
	}

	if (isp1_pwdn < 0) {
		/* Fetch ISP1_PWDN gpio number and set it to low */
		ret = camera_sensor_gpio(-1, GP_CAMERA_ISP1_POWERDOWN,
			GPIOF_DIR_OUT, 0);
		if (ret < 0)
			return ret;
		isp1_pwdn = ret;
	}

	if (flag) {
		dev_dbg(&client->dev,
			"%s: Turning on isp reset pin 0, mux pin 1\n",
			__func__);
		if (camera_reset < 0 || isp1_pwdn < 0) {
			dev_err(&client->dev, "One of the GPIOs missing\n");
			return -EINVAL;
		}
		fsa642_gpio_ctrl(1); /* Flip CSI mux in favor of ov680 */
		gpio_set_value(camera_reset, 0);
		msleep(40);
		gpio_set_value(camera_reset, 1);
		msleep(40);
	} else {
		dev_dbg(&client->dev,
			"%s: Turning off isp reset pin\n", __func__);
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int ov680_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#ifdef CONFIG_X86_INTEL_OSC_CLK
	int ret = 0;
	static const unsigned int clock_khz = 19200;

	struct clk *pclk = clk_get(NULL, "osc.4");
	if (pclk == NULL) {
		dev_err(&client->dev, "get osc clock failed\n");
		return -EINVAL;
	}
	clk_prepare(pclk);
	clk_set_rate(pclk, clock_khz);
	if (flag) {
		ret = clk_enable(pclk);
		msleep(20);
		dev_dbg(&client->dev, "osc clock 4 enabled\n");
	} else {
		clk_disable(pclk);
		msleep(20);
		dev_dbg(&client->dev, "osc clock 4 disabled\n");
	}
	clk_put(pclk);

	return ret;
#else
	dev_err(&client->dev, "ov680 clock is not set\n");
	return 0;
#endif
}

static int ov680_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
	int ret;

	dev_dbg(&client->dev, "%s: vprog1 %s\n", __func__, flag ? "on" : "off");
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	ret = camera_set_vprog_power(CAMERA_VPROG1, flag, DEFAULT_VOLTAGE);
#else
	ret = -EINVAL;
#endif
	if (ret) {
		dev_err(&client->dev, "vprog1 power control failed\n");
		return ret;
	}

	dev_dbg(&client->dev, "%s: vprog3 %s\n", __func__, flag ? "on" : "off");
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	ret = camera_set_vprog_power(CAMERA_VPROG3, flag, CAMERA_1_83_VOLT);
#else
	ret = -EINVAL;
#endif
	if (ret) {
		dev_err(&client->dev, "vprog3 power control failed\n");
		if (flag)
			camera_set_vprog_power(CAMERA_VPROG1, !flag,
					       DEFAULT_VOLTAGE);
		return ret;
	}

	if (flag)
		msleep(20); /* Wait for power lines to stabilize */

	return ret;
}

static int ov680_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
		 ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static int ov680_platform_init(struct i2c_client *client)
{
	return 0;
}

static int ov680_platform_deinit(void)
{
	return 0;
}

static struct atomisp_camera_caps *ov680_get_camera_caps(void)
{
	ov680_camera_caps.sensor_num = 1;
	ov680_camera_caps.sensor[0].stream_num = 1;
	ov680_camera_caps.sensor[0].is_slave = 1;

	return &ov680_camera_caps;
}

static struct camera_sensor_platform_data ov680_sensor_platform_data = {
	.gpio_ctrl      = ov680_gpio_ctrl,
	.flisclk_ctrl   = ov680_flisclk_ctrl,
	.power_ctrl     = ov680_power_ctrl,
	.csi_cfg        = ov680_csi_configure,
	.platform_init  = ov680_platform_init,
	.platform_deinit = ov680_platform_deinit,
	.get_camera_caps = ov680_get_camera_caps,
};

void *ov680_platform_data(void *info)
{
	return &ov680_sensor_platform_data;
}
