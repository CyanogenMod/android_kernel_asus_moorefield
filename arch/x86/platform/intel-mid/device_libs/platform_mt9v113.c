/*
 * platform_mt9v113.c: mt9v113 platform data initilization file
 *
 * (C) Copyright 2012 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_mt9v113.h"

static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_1p8;

/*
 * MFLD PR2 primary camera sensor - MT9V113 platform data
 */
static int mt9v113_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag)
		gpio_set_value(camera_reset, 1);
	else
		gpio_set_value(camera_reset, 0);

	return 0;
}

static int mt9v113_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int mt9v113_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0) {
			pr_err("%s not available.", GP_CAMERA_1_POWER_DOWN);
			return ret;
		}
		camera_power_down = ret;
	}

	if (camera_1p8 < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1P8,
					 GPIOF_DIR_OUT, 1);
		if (ret < 0) {
			pr_err("%s not available.", GP_CAMERA_1P8);
			return ret;
		}
		camera_1p8 = ret;
	}

	if (flag) {
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 0);
		if (camera_1p8 >= 0)
			gpio_set_value(camera_1p8, 0);
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(1);
		}
	} else {
		/* Put sensor into HW standby mode */
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 1);
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(0);
		}
		if (camera_1p8 >= 0)
			gpio_set_value(camera_1p8, 1);
		/* Release the standby */
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 0);

		/*
		 * camera_1p8 is shared pin, so should free it
		 * when don't use it.
		 */
		gpio_free(camera_1p8);
		camera_1p8 = -1;
	}

	return 0;
}

static int mt9v113_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, 1,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static struct camera_sensor_platform_data mt9v113_sensor_platform_data = {
	.gpio_ctrl	= mt9v113_gpio_ctrl,
	.flisclk_ctrl	= mt9v113_flisclk_ctrl,
	.power_ctrl	= mt9v113_power_ctrl,
	.csi_cfg	= mt9v113_csi_configure,
};

void *mt9v113_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;
	camera_1p8 = -1;

	return &mt9v113_sensor_platform_data;
}
