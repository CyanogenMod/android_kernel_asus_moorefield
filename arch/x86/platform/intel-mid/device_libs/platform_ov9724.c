/*
 * platform_ov9724.c: ov9724 platform data initilization file
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
#include <linux/regulator/consumer.h>
#include <asm/intel_scu_ipcutil.h>
#include <asm/intel-mid.h>
#include <media/v4l2-subdev.h>
#include "platform_camera.h"
#include "platform_ov9724.h"


static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static struct regulator *vprog1_reg;
#define VPROG1_VAL 2800000
/*
 * MRFLD VV secondary camera sensor - OV9724 platform data
 */

static int ov9724_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(-1, GP_CAMERA_1_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0)
			return ret;
		camera_reset = ret;
	}

	if (flag) {
		gpio_set_value(camera_reset, 1);
		/* ov9724 initializing time - t1+t2
		 * 427us(t1) - 8192 mclk(19.2Mhz) before sccb communication
		 * 1ms(t2) - sccb stable time when using internal dvdd
		 */
		usleep_range(1500, 1500);
	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int ov9724_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int ov9724_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	if (flag) {
		if (!camera_vprog1_on) {
			ret = regulator_enable(vprog1_reg);
			if (!ret)
				camera_vprog1_on = 1;
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			ret = regulator_disable(vprog1_reg);
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}
	return 0;
}

static int ov9724_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 1;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

static int ov9724_platform_init(struct i2c_client *client)
{
	int ret;

	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}
	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}
	return ret;
}

static int ov9724_platform_deinit(void)
{
	regulator_put(vprog1_reg);

	return 0;
}

static struct camera_sensor_platform_data ov9724_sensor_platform_data = {
	.gpio_ctrl      = ov9724_gpio_ctrl,
	.flisclk_ctrl   = ov9724_flisclk_ctrl,
	.power_ctrl     = ov9724_power_ctrl,
	.csi_cfg        = ov9724_csi_configure,
	.platform_init = ov9724_platform_init,
	.platform_deinit = ov9724_platform_deinit,
};

void *ov9724_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &ov9724_sensor_platform_data;
}

