/*
 * platform_imx227.c: imx227 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * (C) Copyright 2013 Google
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
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/regulator/consumer.h>

#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

#include "platform_camera.h"
#include "platform_imx227.h"

#ifdef CONFIG_VLV2_PLAT_CLK
#define OSC_CAM0_CLK 0x0
#define CLK_19P2MHz 0x1
#endif
static struct regulator *ldo1_reg;
static int camera_reset;
static int camera_ldo1_on;
int camera_1p2_on;
/*
 * MRFLD VV primary camera sensor - IMX227 platform data
 */

static int imx227_gpio_ctrl(struct v4l2_subdev *sd, int flag)
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
		gpio_set_value(camera_reset, 1);
		/* imx227 core silicon initializing time - t1+t2+t3
		 * 400us(t1) - Time to VDDL is supplied after REGEN high
		 * 600us(t2) - imx227 core Waking up time
		 * 459us(t3, 8825clocks) -Initializing time of silicon
		 */
		usleep_range(1500, 1600);

	} else {
		gpio_set_value(camera_reset, 0);
		/* 1us - Falling time of REGEN after XCLR H -> L */
		udelay(1);
	}

	return 0;
}

static int imx227_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
#ifdef CONFIG_VLV2_PLAT_CLK
	if (flag) {
		int ret;
		ret = vlv2_plat_set_clock_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret)
			return ret;
	}
	return vlv2_plat_configure_clock(OSC_CAM0_CLK, flag);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
			flag ? clock_khz : 0);
#else
	pr_err("imx227 clock is not set\n");
	return 0;
#endif
}

static int imx227_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	if (flag) {
		if (camera_1p2_on < 0) {
			ret = camera_sensor_gpio(-1, GP_CAMERA_1P2,
					GPIOF_DIR_OUT, 1);
			if (ret < 0)
				return ret;
			camera_1p2_on = ret;
		}

		if (!camera_ldo1_on) {
			ret = regulator_enable(ldo1_reg);
			if (!ret) {
				usleep_range(300, 400);

				gpio_set_value(camera_1p2_on, 1);
				usleep_range(200, 300);

				camera_ldo1_on = 1;
			}
			return ret;
		}
	} else {
		if (camera_reset >= 0) {
			gpio_free(camera_reset);
			camera_reset = -1;
		}

		if (camera_1p2_on >= 0) {
			gpio_set_value(camera_1p2_on, 0);
			gpio_free(camera_1p2_on);
			camera_1p2_on = -1;
		}
		if (camera_ldo1_on) {
			ret = regulator_disable(ldo1_reg);
			if (!ret)
				camera_ldo1_on = 0;
			return ret;
		}
	}
	return 0;
}

static int imx227_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static int imx227_platform_init(struct i2c_client *client)
{
	ldo1_reg = regulator_get(&client->dev, "LDO1");
	if (IS_ERR(ldo1_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(ldo1_reg);
	}
	return 0;
}

static int imx227_platform_deinit(void)
{
	regulator_put(ldo1_reg);
	return 0;
}

static struct camera_sensor_platform_data imx227_sensor_platform_data = {
	.gpio_ctrl      = imx227_gpio_ctrl,
	.flisclk_ctrl   = imx227_flisclk_ctrl,
	.power_ctrl     = imx227_power_ctrl,
	.csi_cfg        = imx227_csi_configure,
	.platform_init  = imx227_platform_init,
	.platform_deinit = imx227_platform_deinit,
};

void *imx227_platform_data(void *info)
{
	camera_reset = -1;
	camera_1p2_on = -1;
	return &imx227_sensor_platform_data;
}
