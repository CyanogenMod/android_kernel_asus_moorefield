/*
 * platform_mt9e013.c: mt9e013 platform data initilization file
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
#include <linux/delay.h>
#include <linux/atomisp_platform.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_mt9e013.h"

static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;

/*
 * MFLD PR2 primary camera sensor - MT9E013 platform data
 */
static int mt9e013_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
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

static int mt9e013_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int mt9e013_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_power_down < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_POWER_DOWN,
					 GPIOF_DIR_OUT, 1);
		/*
		 * on some HW, this pin is not a connected pin,
		 * while on others, this indeed is avaiable.
		 * so just operate it when available and continue
		 * if it is failed.
		 */
		if (ret < 0)
			pr_debug("%s not available.", GP_CAMERA_0_POWER_DOWN);
		camera_power_down = ret;
	}

	if (flag) {
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 1);
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(1);
		}
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(0);
		}
		if (camera_power_down >= 0)
			gpio_set_value(camera_power_down, 0);
	}

	return 0;
}

static int mt9e013_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 2,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
}

static bool mt9e013_low_fps(void)
{
	if (INTEL_MID_BOARD(1, PHONE, MFLD) &&
	    (SPID_PRODUCT_ID(INTEL, MFLD, PHONE, LEX, PRO) ||
	     SPID_PRODUCT_ID(INTEL, MFLD, PHONE, LEX, ENG)))
		return true;
	else
		return false;
}

static struct camera_sensor_platform_data mt9e013_sensor_platform_data = {
	.gpio_ctrl	= mt9e013_gpio_ctrl,
	.flisclk_ctrl	= mt9e013_flisclk_ctrl,
	.power_ctrl	= mt9e013_power_ctrl,
	.csi_cfg	= mt9e013_csi_configure,
	.low_fps	= mt9e013_low_fps,
};


void mt9e013_reset(struct v4l2_subdev *sd)
{
	mt9e013_power_ctrl(sd, 1);
	mt9e013_gpio_ctrl(sd, 0);
	mt9e013_gpio_ctrl(sd, 1);
	mt9e013_gpio_ctrl(sd, 0);
	mt9e013_power_ctrl(sd, 0);
}

void *mt9e013_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &mt9e013_sensor_platform_data;
}
