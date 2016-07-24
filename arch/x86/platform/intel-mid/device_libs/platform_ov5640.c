/*
 * platform_ov5640.c: ov5640 platform data initilization file
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
#include "platform_ov5640.h"


static int camera_reset;
static int camera_vcm_power;
static int camera_vprog1_on;

/*
 * GRACELAND DV1 primary camera sensor - OV5640 platform data
 */

static int ov5640_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
		gpio_set_value(camera_reset, 0);
		msleep(20);
		gpio_set_value(camera_reset, 1);
	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int ov5640_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

static int ov5640_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_vcm_power < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_VCM_POWER,
					 GPIOF_DIR_OUT, 1);
		/*
		 * on some HW, this pin is not a connected pin,
		 * while on others, this indeed is avaiable.
		 * so just operate it when available and continue
		 * if it is failed.
		 */
		if (ret < 0)
			pr_debug("%s not available.", GP_CAMERA_0_VCM_POWER);
		camera_vcm_power = ret;
	}
	if (flag) {
		if (camera_vcm_power >= 0)
			gpio_set_value(camera_vcm_power, 1);
		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(1);
			/*
			 * delay 20ms to wait sensor power up stable.
			 */
			msleep(20);
		}
	} else {
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(0);
		}
		if (camera_vcm_power >= 0)
			gpio_set_value(camera_vcm_power, 0);

	}

	return 0;
}

static int ov5640_csi_configure(struct v4l2_subdev *sd, int flag)
{
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, 2,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static struct camera_sensor_platform_data ov5640_sensor_platform_data = {
	.gpio_ctrl      = ov5640_gpio_ctrl,
	.flisclk_ctrl   = ov5640_flisclk_ctrl,
	.power_ctrl     = ov5640_power_ctrl,
	.csi_cfg        = ov5640_csi_configure,
};

void *ov5640_platform_data(void *info)
{
	camera_reset = -1;
	camera_vcm_power = -1;

	return &ov5640_sensor_platform_data;
}
