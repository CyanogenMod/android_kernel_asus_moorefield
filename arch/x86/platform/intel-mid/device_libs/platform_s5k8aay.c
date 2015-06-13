/*
 * platform_s5k8aay.c: s5k8aay platform data initilization file
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
#include <linux/regulator/consumer.h>
#include "platform_camera.h"
#include "platform_s5k8aay.h"


static int camera_reset;
static int camera_power;

#ifdef CONFIG_BOARD_CTP
static int camera_vemmc1_on;
static struct regulator *vemmc1_reg;
#define VEMMC1_VAL 2850000
static int camera_vprog1_on;
static struct regulator *vprog1_reg;
#define VPROG1_VAL 1200000
#else
static int camera_vprog1_on;
#endif


/*
 * MRFLD VV primary camera sensor - IMX135 platform data
 */

static int s5k8aay_gpio_ctrl(struct v4l2_subdev *sd, int flag)
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
		/* min 250us -Initializing time of silicon */
		usleep_range(250, 300);

	} else {
		gpio_set_value(camera_reset, 0);
	}

	return 0;
}

static int s5k8aay_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM1, flag ? clock_khz : 0);
}

static int s5k8aay_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#ifdef CONFIG_BOARD_CTP
	int reg_err;
#endif
	if (flag) {
#ifdef CONFIG_BOARD_CTP
		if (!camera_vemmc1_on) {
			camera_vemmc1_on = 1;
			reg_err = regulator_enable(vemmc1_reg);
			if (reg_err) {
				dev_err(&client->dev, "Failed to enable regulator vemmc1\n");
				return reg_err;
			}
		}

		/* Small waiting is needed between Vana and Vdig */
		usleep_range(50, 50);

		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			reg_err = regulator_enable(vprog1_reg);
			if (reg_err) {
				dev_err(&client->dev, "Failed to enable regulator vprog1\n");
				return reg_err;
			}
		}

		if (camera_power < 0) {
			reg_err = camera_sensor_gpio(-1, GP_CAMERA_1_POWER_DOWN,
						GPIOF_DIR_OUT, 1);
			if (reg_err < 0)
				return reg_err;
			camera_power = reg_err;
		}

		gpio_set_value(camera_power, 1);
		/* min 250us -Initializing time of silicon */
		usleep_range(250, 300);
#else

		if (!camera_vprog1_on) {
			camera_vprog1_on = 1;
			intel_scu_ipc_msic_vprog1(0);
		}
#endif
	} else {
#ifdef CONFIG_BOARD_CTP
		gpio_set_value(camera_power, 0);

		if (camera_vprog1_on) {
			camera_vprog1_on = 0;

			reg_err = regulator_disable(vprog1_reg);
			if (reg_err) {
				dev_err(&client->dev, "Failed to disable regulator vprog1\n");
				return reg_err;
			}
		}
		if (camera_vemmc1_on) {
			camera_vemmc1_on = 0;

			reg_err = regulator_disable(vemmc1_reg);
			if (reg_err) {
				dev_err(&client->dev, "Failed to disable regulator vemmc1\n");
				return reg_err;
			}
		}
#else
		if (camera_vprog1_on) {
			camera_vprog1_on = 0;
			intel_scu_ipc_msic_vprog1(1);
		}
#endif
	}
	return 0;
}

#ifdef CONFIG_BOARD_CTP
static int s5k8aay_platform_init(struct i2c_client *client)
{
	int ret;
	vemmc1_reg = regulator_get(&client->dev, "vemmc1");
	if (IS_ERR(vemmc1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vemmc1_reg);
	}
	vprog1_reg = regulator_get(&client->dev, "vprog1");
	if (IS_ERR(vprog1_reg)) {
		dev_err(&client->dev, "vprog1 regulator_get failed\n");
		return PTR_ERR(vprog1_reg);
	}

	ret = regulator_set_voltage(vprog1_reg, VPROG1_VAL, VPROG1_VAL);
	if (ret) {
		dev_err(&client->dev, "vprog1 regulator voltage set failed\n");
		regulator_put(vprog1_reg);
	}
	return 0;
}

static int s5k8aay_platform_deinit(void)
{
	regulator_put(vprog1_reg);
	regulator_put(vemmc1_reg);
	return 0;
}
#endif

static int s5k8aay_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 1;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_SECONDARY, LANES,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

static char *s5k8aay_msr_file_name(void)
{
	/* Only one platform at the moment */
	return "01s5k8aay.drvb";
}

static struct camera_sensor_platform_data s5k8aay_sensor_platform_data = {
	.gpio_ctrl      = s5k8aay_gpio_ctrl,
	.flisclk_ctrl   = s5k8aay_flisclk_ctrl,
	.power_ctrl     = s5k8aay_power_ctrl,
	.csi_cfg        = s5k8aay_csi_configure,
#ifdef CONFIG_BOARD_CTP
	.platform_init = s5k8aay_platform_init,
	.platform_deinit = s5k8aay_platform_deinit,
	.msr_file_name   = s5k8aay_msr_file_name,
#endif
};

void *s5k8aay_platform_data(void *info)
{
	camera_reset = -1;
	camera_power = -1;

	return &s5k8aay_sensor_platform_data;
}

