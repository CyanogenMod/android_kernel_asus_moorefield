/*
 * platform_ar0543_raw.c: ar0543_raw platform data initilization file
 *
 * (C) Copyright 2013 ASUSTeK COMPUTER INC
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
#include "platform_ar0543_raw.h"

static int camera_reset;
static int camera_power_down;
static int camera_vprog1_on;
static int camera_vprog2_on; /* duel sim */
static int camera_sensor_2_8v;
static int camera_vemmc1_on;
static int camera_sensor_1_8v; /* A400CG */

static struct regulator *vprog1_reg;
static struct regulator *vprog2_reg;
#define VPROG1_VAL 1800000
#define VPROG2_VAL 2800000
/*
 * CLV PR0 primary camera sensor - AR0543_RAW platform data
 */

static int ar0543_raw_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;
	int reset_gpio_pin;
	int PCB_ID1 = 0; /* For duel SIM++ */

#if defined(CONFIG_ME372CL) || defined(CONFIG_PF450CL)
	if (PROJECT_ID == 0xFF)
		PROJECT_ID = Read_PROJ_ID();
#else
		PROJECT_ID = PROJ_ID_ME372CL;
#endif

	/* For duel SIM++ */
	if (PROJECT_ID == PROJ_ID_ME175CG) {
		ret = gpio_request(174, "PCB_ID1");
		if (ret) {
			pr_err("%s: failed to request gpio(pin 174)\n", __func__);
			return -EINVAL;
		}

		ret = gpio_direction_input(174);
		PCB_ID1 = gpio_get_value(174);
		pr_err("%s: PCB_ID1 %d\n", __func__, PCB_ID1);
		gpio_free(174);
	}
	/* For duel SIM-- */


	if ((PROJECT_ID == PROJ_ID_ME175CG) || (PROJECT_ID == PROJ_ID_A400CG) || (PROJECT_ID == PROJ_ID_PF450CL))
		reset_gpio_pin = 161; /* ME175CG */
	else
		reset_gpio_pin = 177; /* ME372CG_CR */

	if (camera_reset < 0) {
		ret = camera_sensor_gpio(reset_gpio_pin, GP_CAMERA_0_RESET,
					GPIOF_DIR_OUT, 1);
		if (ret < 0) {
			pr_err("%s error\n", GP_CAMERA_0_RESET);
			return ret;
		}
		camera_reset = reset_gpio_pin;
	}

	if ((PCB_ID1 != 0) || (PROJECT_ID == PROJ_ID_PF450CL)/*|| (PROJECT_ID == PROJ_ID_A400CG)*/) {
		pr_err("ME175CG Duel SIM SKU / PF450CL: Change to vprog2\n");
	} else {
		if (camera_sensor_2_8v < 0) {
			ret = camera_sensor_gpio(GP_CORE_014, GP_SENSOR_2_8V,
					 GPIOF_DIR_OUT, 0);
			if (ret < 0) {
				pr_err("%s error\n", GP_SENSOR_2_8V);
				return ret;
			}
			camera_sensor_2_8v = GP_CORE_014;
		}
	}

	if (flag) {
		gpio_set_value(camera_reset, 0);
		usleep_range(20000, 20000);
		gpio_set_value(camera_reset, 1);
		usleep_range(20000, 20000);

		if ((PCB_ID1 != 0) ||
			(PROJECT_ID == PROJ_ID_PF450CL) ||
			((PROJECT_ID == PROJ_ID_A400CG) && (HW_ID != HW_ID_SR1) && (HW_ID != HW_ID_SR2))) {
			if (!camera_vprog2_on) {
				ret = regulator_enable(vprog2_reg);
				usleep_range(1000, 1000);
				if (!ret) {
					camera_vprog2_on = 1;
				} else {
					pr_err("Failed to enable regulator vprog2\n");
					return ret;
				}
			}
		} else {
			/* turn on power sensor AVDD 2.8V */
			if (camera_sensor_2_8v >= 0) {
				gpio_set_value(camera_sensor_2_8v, 1);
				pr_err("<<< AVDD_SENSOR 2.8V = 1\n");

				if (PROJECT_ID != PROJ_ID_A400CG)
					usleep_range(1000, 1000);
			}
		}
	} else {
		gpio_set_value(camera_reset, 0);
		if ((PCB_ID1 != 0) ||
			(PROJECT_ID == PROJ_ID_PF450CL) ||
			((PROJECT_ID == PROJ_ID_A400CG) && (HW_ID != HW_ID_SR1) && (HW_ID != HW_ID_SR2))) {
			if (camera_vprog2_on) {
				ret = regulator_disable(vprog2_reg);
				if (!ret) {
					camera_vprog2_on = 0;
				} else {
					pr_err("Failed to disable regulator vprog2\n");
					return ret;
				}
			}
		} else {
			if (camera_sensor_2_8v >= 0) {
				gpio_set_value(camera_sensor_2_8v, 0);
				pr_err("<<< AVDD_SENSOR 2.8V = 0\n");
				gpio_free(camera_sensor_2_8v);
				camera_sensor_2_8v = -1;
				usleep_range(10000, 10000);
			}

		}
	}

	return 0;
}

static int ar0543_raw_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0, flag ? clock_khz : 0);
}

/*
 * Checking the SOC type is temporary workaround to enable AR0543_RAW
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ar0543_raw_power_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret = 0;
	int reg_err;

#if defined(CONFIG_ME372CL) || defined(CONFIG_PF450CL)
	if (PROJECT_ID == 0xFF)
		PROJECT_ID = Read_PROJ_ID();

	if (HW_ID == 0xFF)
		HW_ID = Read_HW_ID();
#else
	PROJECT_ID = PROJ_ID_ME372CL;
	HW_ID = 0xFF;
#endif


	if (PROJECT_ID == PROJ_ID_A400CG && (HW_ID == HW_ID_SR1 || HW_ID == HW_ID_SR2)) {
		if (camera_sensor_1_8v < 0) {
			ret = camera_sensor_gpio(GP_AON_052, GP_CAMERA_CAM2_1V8_EN,
					GPIOF_DIR_OUT, 0);
			if (ret < 0) {
				pr_err("%s not available.\n", GP_CAMERA_CAM2_1V8_EN);
				return ret;
			}
			camera_sensor_1_8v = GP_AON_052;
		}
	}

	if (flag) {
		/* turn on VCM power 2.85V */
		if (!camera_vemmc1_on) {
			camera_vemmc1_on = 1;
			reg_err = intel_scu_ipc_msic_vemmc1(1);
			/* reg_err = regulator_enable(vemmc1_reg); */
			if (reg_err) {
				pr_err("Failed to enable regulator vemmc1\n");
				return reg_err;
			}
			pr_err("<<< VCM 2.85V = 1\n");
			usleep_range(10000, 10000);
		}

		if (PROJECT_ID == PROJ_ID_A400CG && (HW_ID == HW_ID_SR1 || HW_ID == HW_ID_SR2)) {
			/* turn on 1.8V power */
			if (camera_sensor_1_8v >= 0) {
				gpio_set_value(camera_sensor_1_8v, 1);
				pr_err("<<< 1.8V = 1\n");
				usleep_range(2000, 3000);
			}
		} else {
			if (!camera_vprog1_on) {
				ret = regulator_enable(vprog1_reg);

				if (PROJECT_ID == PROJ_ID_A400CG || PROJECT_ID == PROJ_ID_PF450CL)
					usleep_range(2000, 3000);
				else
					usleep_range(20000, 20000);

				if (!ret) {
					camera_vprog1_on = 1;
				} else {
					pr_err("Failed to enable regulator vprog1\n");
					return ret;
				}
			}
		}
	} else {
		if (PROJECT_ID == PROJ_ID_A400CG && (HW_ID == HW_ID_SR1 || HW_ID == HW_ID_SR2)) {
			/* turn off 1.8V power */
			if (camera_sensor_1_8v >= 0) {
				gpio_set_value(camera_sensor_1_8v, 0);
				pr_err("<<< 1.8V = 0\n");
				gpio_free(camera_sensor_1_8v);
				camera_sensor_1_8v = -1;
				usleep_range(10000, 11000);
			}
		} else {
			if (camera_vprog1_on) {
				ret = regulator_disable(vprog1_reg);
				if (!ret) {
					camera_vprog1_on = 0;
				} else {
					pr_err("Failed to disable regulator vprog1\n");
					return ret;
				}
			}
		}

		/* turn off VCM power 2.85V */
		if (camera_vemmc1_on) {
			camera_vemmc1_on = 0;
			reg_err = intel_scu_ipc_msic_vemmc1(0);
			/* reg_err = regulator_disable(vemmc1_reg); */
			if (reg_err) {
				pr_err("Failed to disable regulator vemmc1\n");
				return reg_err;
			}
			pr_err("<<< VCM 2.85V = 0\n");
			usleep_range(10000, 10000);
		}
	}
	return 0;
}

static int ar0543_raw_csi_configure(struct v4l2_subdev *sd, int flag)
{
	static const int LANES = 2;
#ifdef CONFIG_ME175CG
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_gbrg, flag);
#else
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, LANES,
		ATOMISP_INPUT_FORMAT_RAW_10, atomisp_bayer_order_grbg, flag);
#endif
}

/*
 * Checking the SOC type is temporary workaround to enable AR0543_RAW
 * on Bodegabay (tangier) platform. Once standard regulator devices
 * (e.g. vprog1, vprog2) and control functions (pmic_avp) are added
 * for the platforms with tangier, then we can revert this change.
 * (dongwon.kim@intel.com)
 */
static int ar0543_raw_platform_init(struct i2c_client *client)
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

#if defined(CONFIG_ME175CG) || defined(CONFIG_A400CG) || defined(CONFIG_PF450CL)
	vprog2_reg = regulator_get(&client->dev, "vprog2");
	if (IS_ERR(vprog2_reg)) {
		dev_err(&client->dev, "regulator_get failed\n");
		return PTR_ERR(vprog2_reg);
	}
	ret = regulator_set_voltage(vprog2_reg, VPROG2_VAL, VPROG2_VAL);
	if (ret) {
		dev_err(&client->dev, "regulator voltage set failed\n");
		regulator_put(vprog2_reg);
	}
#endif
	return ret;
}

/*
 * Checking the SOC type is temporary workaround to enable AR0543_RAW on Bodegabay
 * (tangier) platform once standard regulator devices (e.g. vprog1, vprog2) and
 * control functions (pmic_avp) are added for the platforms with tangier, then
 * we can revert this change.(dongwon.kim@intel.com
 */
static int ar0543_raw_platform_deinit(void)
{
	regulator_put(vprog1_reg);

#if defined(CONFIG_ME175CG) || defined(CONFIG_A400CG)
	regulator_put(vprog2_reg);
#endif
	return 0;
}
static struct camera_sensor_platform_data ar0543_raw_sensor_platform_data = {
	.gpio_ctrl      = ar0543_raw_gpio_ctrl,
	.flisclk_ctrl   = ar0543_raw_flisclk_ctrl,
	.power_ctrl     = ar0543_raw_power_ctrl,
	.csi_cfg        = ar0543_raw_csi_configure,
	.platform_init = ar0543_raw_platform_init,
	.platform_deinit = ar0543_raw_platform_deinit,
};

void *ar0543_raw_platform_data(void *info)
{
	pr_info("%s - enter\n", __func__);
	camera_reset = -1;
	camera_power_down = -1;
	camera_sensor_2_8v = -1;
	camera_sensor_1_8v = -1;

	return &ar0543_raw_sensor_platform_data;
}
