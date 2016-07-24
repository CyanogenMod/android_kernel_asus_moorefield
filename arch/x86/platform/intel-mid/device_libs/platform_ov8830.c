/*
 * platform_ov8830.c: ov8830 platform data initilization file
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
#include <linux/regulator/consumer.h>
#include <linux/sfi.h>
#include "platform_camera.h"
#include "platform_ov8830.h"


static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;

static struct regulator *vprog1_reg;
#define VPROG1_VAL 2800000
/*
 * CLV PR0 primary camera sensor - OV8830 platform data
 */

static int ov8830_gpio_ctrl(struct v4l2_subdev *sd, int flag)
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

static int ov8830_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

/*
 * Checking the SOC type is temporary workaround to enable OV8830
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ov8830_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	/* The camera powering is different on RedHookBay and VictoriaBay
	 * On RHB, vprog1 is at 2.8V and supplies both cameras
	 * On VB, vprog1 supplies the 2nd camera and must not rise over 1.2V
	 * Check if the RHB SW has accidentally been flashed to VB
	 * If yes, don't turn on the regulator. The VB secondary camera will
	 * be permanently damaged by the too high voltage
	 */
	if (INTEL_MID_BOARD(2, PHONE, CLVTP, VB, PRO) ||
	    INTEL_MID_BOARD(2, PHONE, CLVTP, VB, ENG)) {
		printk(KERN_ALERT \
		"Aborted vprog1 enable to protect VictoriaBay 2nd camera HW\n");
		return -ENODEV;
	}

	if (flag) {
		if (!camera_vprog1_on) {
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(1);
			else
				ret = regulator_enable(vprog1_reg);
			if (!ret)
				camera_vprog1_on = 1;
			else
				printk(KERN_ALERT "Failed to enable regulator vprog1\n");
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
			if (intel_mid_identify_cpu() ==
			   INTEL_MID_CPU_CHIP_TANGIER)
				ret = intel_scu_ipc_msic_vprog1(0);
			else
				ret = regulator_disable(vprog1_reg);

			if (!ret)
				camera_vprog1_on = 0;
			else
				printk(KERN_ALERT "Failed to disable regulator vprog1\n");
			return ret;
		}
	}
	return ret;
}

static int ov8830_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_bggr, flag);
}

/*
 * Checking the SOC type is temporary workaround to enable OV8830
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ov8830_platform_init(struct i2c_client *client)
{
	int ret;
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		return 0;

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

/*
 * Checking the SOC type is temporary workaround to enable OV8830 on Bodegabay
 * (tangier) platform once standard regulator devices (e.g. vprog1, vprog2) and
 * control functions (pmic_avp) are added for the platforms with tangier, then
 * we can revert this change.(dongwon.kim@intel.com
 */
static int ov8830_platform_deinit(void)
{
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_TANGIER)
		return 0;

	regulator_put(vprog1_reg);

	return 0;
}
static struct camera_sensor_platform_data ov8830_sensor_platform_data = {
	.gpio_ctrl      = ov8830_gpio_ctrl,
	.flisclk_ctrl   = ov8830_flisclk_ctrl,
	.power_ctrl     = ov8830_power_ctrl,
	.csi_cfg        = ov8830_csi_configure,
	.platform_init = ov8830_platform_init,
	.platform_deinit = ov8830_platform_deinit,
};

void *ov8830_platform_data(void *info)
{
	camera_reset = -1;
	camera_power_down = -1;

	return &ov8830_sensor_platform_data;
}
