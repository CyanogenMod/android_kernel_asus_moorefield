/*
 * platform_imx219.c: imx219 platform data initilization file
 *
 * (C) Copyright 2014 Intel Corporation
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
#include "platform_imx219.h"

static int camera_reset;

static int is_moorefield(void)
{
	return INTEL_MID_BOARD(1, PHONE, MOFD) ||
	       INTEL_MID_BOARD(1, TABLET, MOFD);
}

#ifdef CONFIG_SFI
#define CONFIG_VIDEO_IMX219_FAKE_SFI_TABLE
#endif
/*
 * MOFD VV primary camera sensor - imx219 platform data
 */

static int imx219_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
#ifdef CONFIG_VIDEO_IMX219_FAKE_SFI_TABLE
	int pin;
#endif

	if (camera_reset < 0) {
#ifdef CONFIG_VIDEO_IMX219_FAKE_SFI_TABLE
		pin = get_gpio_by_name(GP_CAMERA_0_RESET);
		gpio_free(pin);
#endif
		ret = camera_sensor_gpio(-1, GP_CAMERA_0_RESET,
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

static int imx219_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
#else
	pr_err("imx219 clock is not set.\n");
	return 0;
#endif
}

static int imx219_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;

	if (is_moorefield()) {
#ifdef CONFIG_INTEL_SCU_IPC_UTIL
		ret = camera_set_vprog_power(CAMERA_VPROG1, flag,
					     DEFAULT_VOLTAGE);
		if (ret) {
			pr_err("imx219 power %s vprog1 failed\n",
			       flag ? "on" : "off");
			return ret;
		}

		if (flag)
			usleep_range(1000, 1200);
#else
		ret = -ENODEV;
#endif
		return ret;
	}

	return -ENODEV;
}

static int imx219_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 4;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_rggb, flag);
}

static struct camera_sensor_platform_data imx219_sensor_platform_data = {
	.gpio_ctrl      = imx219_gpio_ctrl,
	.flisclk_ctrl   = imx219_flisclk_ctrl,
	.power_ctrl     = imx219_power_ctrl,
	.csi_cfg        = imx219_csi_configure,
};

void *imx219_platform_data(void *info)
{
	camera_reset = -1;
	return &imx219_sensor_platform_data;
}

#ifdef CONFIG_VIDEO_IMX219_FAKE_SFI_TABLE
static struct sfi_device_table_entry imx219_entry = {
	.type = SFI_DEV_TYPE_I2C,
	.host_num = 4,
	.addr = 0x1a,
	.irq = 0xFF,
	.max_freq = 0x400000,
	.name = "imx219",
};

static int __init platform_imx219_module_init(void)
{
	if (is_moorefield()) {
		struct devs_id *dev;
		dev = get_device_id(imx219_entry.type, imx219_entry.name);
		if (dev && dev->device_handler)
			dev->device_handler(&imx219_entry, dev);
	}
	return 0;
}

module_init(platform_imx219_module_init);
#endif

