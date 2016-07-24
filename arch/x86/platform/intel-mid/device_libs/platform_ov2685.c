/*
 * platform_ov2685.c: ov2685 platform data initilization file
 *
 * (C) Copyright 2013 Intel Corporation
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
#include <linux/mfd/intel_mid_pmic.h>
#include "platform_camera.h"
#include "platform_ov2685.h"

#ifdef CONFIG_INTEL_SOC_PMC
#include <asm/intel_soc_pmc.h>
#endif

#ifdef CONFIG_VLV2_PLAT_CLK
#include <linux/vlv2_plat_clock.h>
#endif

/* workround - pin defined for byt */
#define CAMERA_0_PWDN 123
#define CAMERA_1P8_EN	128

#ifdef CONFIG_INTEL_SOC_PMC
#define OSC_CAM0_CLK 0x0
#define CLK_19P2MHz 0x1
#define CLK_ON	0x1
#define CLK_OFF	0x2
#endif

#ifdef CONFIG_CRYSTAL_COVE
#define VPROG_2P8V 0x66
#define VPROG_1P8V 0x5D
#endif

static int camera_power_down;
static int camera_vprog1_on;
static struct regulator *v1p8_reg;
static struct regulator *v2p8_reg;

/* PMIC HID */
#define PMIC_HID_ROHM	"INT33FD:00"
#define PMIC_HID_XPOWER	"INT33F4:00"
#define PMIC_HID_TI	"INT33F5:00"

enum pmic_ids {
	PMIC_ROHM = 0,
	PMIC_XPOWER,
	PMIC_TI,
	PMIC_MAX
};

static enum pmic_ids pmic_id;

/*
 * MRFLD VV primary camera sensor - ov2685 platform data
 */
#ifdef CONFIG_CRYSTAL_COVE
static int match_name(struct device *dev, void *data)
{
	const char *name = data;
	struct i2c_client *client = i2c_verify_client(dev);
	return client ? !strncmp(client->name, name, strlen(name)) : 0;
}

static struct i2c_client *i2c_find_client_by_name(char *name)
{
	struct device *dev = bus_find_device(&i2c_bus_type, NULL,
						name, match_name);
	return dev ? to_i2c_client(dev) : NULL;
}

static enum pmic_ids camera_pmic_probe()
{
	/* search by client name */
	struct i2c_client *client;
	if (spid.hardware_id != BYT_TABLET_BLK_CRV2 ||
		i2c_find_client_by_name(PMIC_HID_ROHM))
		return PMIC_ROHM;

	client = i2c_find_client_by_name(PMIC_HID_XPOWER);
	if (client)
		return PMIC_XPOWER;

	client = i2c_find_client_by_name(PMIC_HID_TI);
	if (client)
		return PMIC_TI;

	return PMIC_MAX;
}

static int camera_pmic_set(bool flag)
{
	int val;
	int ret = 0;
	if (pmic_id == PMIC_MAX) {
		pmic_id = camera_pmic_probe();
		if (pmic_id == PMIC_MAX)
			return -EINVAL;
	}

	if (flag) {
		switch (pmic_id) {
		case PMIC_ROHM:
			ret = regulator_enable(v2p8_reg);
			if (ret)
				return ret;

			ret = regulator_enable(v1p8_reg);
			if (ret)
				regulator_disable(v2p8_reg);
			break;
		case PMIC_XPOWER:
			/* ALDO1 */
			ret = intel_mid_pmic_writeb(0x28, 0x16);
			if (ret)
				return ret;

			/* PMIC Output CTRL 3 for ALDO1 */
			val = intel_mid_pmic_readb(0x13);
			val |= (1 << 5);
			ret = intel_mid_pmic_writeb(0x13, val);
			if (ret)
				return ret;

			/* ELDO2 */
			ret = intel_mid_pmic_writeb(0x1A, 0x16);
			if (ret)
				return ret;

			/* PMIC Output CTRL 2 for ELDO2 */
			val = intel_mid_pmic_readb(0x12);
			val |= (1 << 1);
			ret = intel_mid_pmic_writeb(0x12, val);
			break;
		case PMIC_TI:
			/* LDO9 */
			ret = intel_mid_pmic_writeb(0x49, 0x2F);
			if (ret)
				return ret;

			/* LDO10 */
			ret = intel_mid_pmic_writeb(0x4A, 0x59);
			if (ret)
				return ret;
			break;
		default:
			return -EINVAL;
		}

	} else {
		switch (pmic_id) {
		case PMIC_ROHM:
			ret = regulator_disable(v2p8_reg);
			ret += regulator_disable(v1p8_reg);
			break;
		case PMIC_XPOWER:
			val = intel_mid_pmic_readb(0x13);
			val &= ~(1 << 5);
			ret = intel_mid_pmic_writeb(0x13, val);
			if (ret)
				return ret;

			val = intel_mid_pmic_readb(0x12);
			val &= ~(1 << 1);
			ret = intel_mid_pmic_writeb(0x12, val);
			break;
		case PMIC_TI:
			/* LDO9 */
			ret = intel_mid_pmic_writeb(0x49, 0x2E);
			if (ret)
				return ret;

			/* LDO10 */
			ret = intel_mid_pmic_writeb(0x4A, 0x58);
			if (ret)
				return ret;
			break;
		default:
			return -EINVAL;
		}
	}
	return ret;
}
#endif

static int ov2685_gpio_ctrl(struct v4l2_subdev *sd, int flag)
{
	int ret;

	/*
	 * FIXME: WA using hardcoded GPIO value here.
	 * The GPIO value would be provided by ACPI table, which is
	 * not implemented currently.
	 */
	if (camera_power_down < 0) {
		ret = gpio_request(CAMERA_0_PWDN, "camera_0_power");
		if (ret) {
			pr_err("%s: failed to request gpio(pin %d)\n",
				__func__, CAMERA_0_PWDN);
			return ret;
		}
	}
	camera_power_down = CAMERA_0_PWDN;
	ret = gpio_direction_output(camera_power_down, 0);

	if (ret) {
		pr_err("%s: failed to set gpio(pin %d) direction\n",
			__func__, camera_power_down);
		gpio_free(camera_power_down);
		return ret;
	}

	if (flag) {
		gpio_set_value(camera_power_down, 0);
		usleep_range(1000, 1200);
		gpio_set_value(camera_power_down, 1);
		/* XSHUTDOWN to XCLK: 8192 clks */
		usleep_range(1000, 1200);

	} else {
		gpio_set_value(camera_power_down, 0);
		gpio_free(camera_power_down);
		camera_power_down = -1;
	}

	return 0;
}

/*
 * WORKAROUND:
 * This func will return 0 since MCLK is enabled by BIOS
 * and will be always on event if set MCLK failed here.
 * TODO: REMOVE WORKAROUND, err should be returned when
 * set MCLK failed.
 */
static int ov2685_flisclk_ctrl(struct v4l2_subdev *sd, int flag)
{
	static const unsigned int clock_khz = 19200;
#ifdef CONFIG_INTEL_SOC_PMC
	int ret = 0;
	if (flag) {
		ret = pmc_pc_set_freq(OSC_CAM0_CLK, CLK_19P2MHz);
		if (ret) {
			pr_err("Fail to set ov2685 clock.\n");
			return ret;
		}
		ret = pmc_pc_configure(OSC_CAM0_CLK, CLK_ON);
		/* XCLK to SCCB: 8192 clks */
		usleep_range(10000, 12000);
		return ret;
	}
	usleep_range(1000, 1200);
	return pmc_pc_configure(OSC_CAM0_CLK, CLK_OFF);
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
	return intel_scu_ipc_osc_clk(OSC_CLK_CAM0,
				     flag ? clock_khz : 0);
#else
	pr_err("ov2685 clock is not set.\n");
	return 0;
#endif
}

static int ov2685_power_ctrl(struct v4l2_subdev *sd, int flag)
{
#ifdef CONFIG_CRYSTAL_COVE
	struct i2c_client *client = v4l2_get_subdevdata(sd);
#endif
	int ret = 0;

	if (flag) {
		if (!camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			ret = camera_pmic_set(flag);
			if (ret) {
				dev_err(&client->dev,
						"Failed to enable regulator\n");
				return ret;
			}
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(1);
#else
			pr_err("ov2685 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 1;
			usleep_range(10000, 11000);
			return ret;
		}
	} else {
		if (camera_vprog1_on) {
#ifdef CONFIG_CRYSTAL_COVE
			ret = camera_pmic_set(flag);
			if (ret) {
				dev_err(&client->dev,
						"Failed to enable regulator\n");
				return ret;
			}
#elif defined(CONFIG_INTEL_SCU_IPC_UTIL)
			ret = intel_scu_ipc_msic_vprog1(0);
#else
			pr_err("ov2685 power is not set.\n");
#endif
			if (!ret)
				camera_vprog1_on = 0;
			return ret;
		}
	}

	return 0;
}



static int ov2685_csi_configure(struct v4l2_subdev *sd, int flag)
{
	const u32 csi_lane = 2;
	return camera_sensor_csi(sd, ATOMISP_CAMERA_PORT_PRIMARY, csi_lane,
		ATOMISP_INPUT_FORMAT_YUV422_8, -1, flag);
}

#ifdef CONFIG_CRYSTAL_COVE
static int ov2722_platform_init(struct i2c_client *client)
{
	pmic_id = camera_pmic_probe();
	if (pmic_id != PMIC_ROHM)
		return 0;

	v1p8_reg = regulator_get(&client->dev, "v1p8sx");
	if (IS_ERR(v1p8_reg)) {
		dev_err(&client->dev, "v1p8s regulator_get failed\n");
		return PTR_ERR(v1p8_reg);
	}

	v2p8_reg = regulator_get(&client->dev, "v2p85sx");
	if (IS_ERR(v2p8_reg)) {
		regulator_put(v1p8_reg);
		dev_err(&client->dev, "v2p85sx regulator_get failed\n");
		return PTR_ERR(v2p8_reg);
	}

	return 0;
}


static int ov2722_platform_deinit(void)
{
	if (pmic_id != PMIC_ROHM)
		return 0;

	regulator_put(v1p8_reg);
	regulator_put(v2p8_reg);

	return 0;
}
#endif

static struct camera_sensor_platform_data ov2685_sensor_platform_data = {
	.gpio_ctrl	= ov2685_gpio_ctrl,
	.flisclk_ctrl	= ov2685_flisclk_ctrl,
	.power_ctrl	= ov2685_power_ctrl,
	.csi_cfg	= ov2685_csi_configure,
	.platform_init = ov2722_platform_init,
	.platform_deinit = ov2722_platform_deinit,

};

void *ov2685_platform_data(void *info)
{
	camera_power_down = -1;
	camera_vprog1_on = 0;
	pmic_id = PMIC_MAX;
	return &ov2685_sensor_platform_data;
}

