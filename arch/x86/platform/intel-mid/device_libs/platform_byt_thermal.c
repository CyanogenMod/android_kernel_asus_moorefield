/*
 * platform_byt_thermal.c: Platform data initilization file for
 *			Intel Baytrail Platform thermal driver
 *
 * (C) Copyright 2013 Intel Corporation
 * Author: Durgadoss R <durgadoss.r@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#define pr_fmt(fmt)  "intel_mid_thermal: " fmt

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>
#include <linux/mfd/intel_mid_pmic.h>

#include <asm/intel-mid.h>
#include <asm/intel_mid_thermal.h>

#define BYT_THERM_DEV_NAME	"crystal_cove_thermal"
#define BYT_EC_THERM_DEV_NAME	"byt_ec_thermal"
#define BYT_CR_THERM_DEV_NAME	"byt_cr_thermal"

enum {
	byt_thermal,
	byt_ec_thermal,
	byt_cr_thermal,
};

static struct intel_mid_thermal_sensor byt_sensors[] = {
	{
		.name = "SYSTHERM0",
		.index = 0,
	},
	{
		.name = "SYSTHERM1",
		.index = 1,
	},
	{
		.name = "SYSTHERM2",
		.index = 2,
	},
	{
		.name = "PMICDIE",
		.index = 3,
		.direct = true,
	},
	/* Virtual Sensors should always be at the end */
	{
		.name = "FrontSkin",
		.index = 4,
	},
	{
		.name = "BackSkin",
		.index = 5,
	},
};

static int linear_correlation(void *info, long temp, long *res)
{
	struct intel_mid_thermal_sensor *sensor = info;

	if (!sensor)
		return -EINVAL;

	*res = ((temp * sensor->slope) / 1000) + sensor->intercept;

	return 0;
}


static struct intel_mid_thermal_sensor byt_cr_sensors[] = {
	{
		.name = "PMICDIE",
		.index = 0,
	},
	{
		.name = "skin1",
		.index = 1,
	},
};

/* 'r' stands for 'remote' and 'l' for 'local' */
static struct intel_mid_thermal_sensor byt_ec_sensors[] = {
	{
		.name = "CPU_r",
		.index = 0,
	},
	{
		.name = "CPU_l",
		.index = 1,
	},
	{
		.name = "Ambient_r",
		.index = 2,
	},
	{
		.name = "Ambient_l",
		.index = 3,
	},
	{
		.name = "Ambient_r2",
		.index = 4,
	},
	{
		.name = "DDR3_r",
		.index = 5,
	},
	{
		.name = "DDR3_l",
		.index = 6,
	},
};

static struct intel_mid_thermal_platform_data pdata[] = {
	[byt_thermal] = {
		.num_sensors = 4,
		.sensors = byt_sensors,
		.num_virtual_sensors = 2,
	},
	[byt_ec_thermal] = {
		.num_sensors = 7,
		.sensors = byt_ec_sensors,
	},
	[byt_cr_thermal] = {
		.num_sensors = 2,
		.sensors = byt_cr_sensors,
	},
};

static int set_byt_platform_thermal_data(void)
{
	return intel_mid_pmic_set_pdata(BYT_THERM_DEV_NAME,
				&pdata[byt_thermal],
				sizeof(pdata[byt_thermal]), 0);
}

static int set_byt_cr_platform_thermal_data(void)
{
	struct platform_device *pdev;

	pdev = platform_device_register_simple(
					BYT_CR_THERM_DEV_NAME,
					-1, NULL, 0);
	if (!pdev) {
		pr_err("pdev_register failed for byt_cr_thermal\n");
		return PTR_ERR(pdev);
	}
	pdev->dev.platform_data = &pdata[byt_cr_thermal];
	return 0;
}

static int set_byt_ec_platform_thermal_data(void)
{
	struct platform_device *pdev;

	pdev = platform_device_register_simple(
					BYT_EC_THERM_DEV_NAME,
					-1, NULL, 0);
	if (!pdev) {
		pr_err("pdev_register failed for byt_ec_thermal\n");
		return PTR_ERR(pdev);
	}

	pdev->dev.platform_data = &pdata[byt_ec_thermal];
	return 0;
}

static int __init byt_platform_thermal_init(void)
{
	int ret = -EINVAL;

	if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR0) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR1) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 10PR11) ||
		INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 10PR11) ||
		INTEL_MID_BOARD(1, TABLET, CHT)) {
		ret = set_byt_platform_thermal_data();
	} else if (INTEL_MID_BOARD(3, TABLET, BYT, BLB, PRO, CRBV3) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLB, ENG, CRBV3)) {
		ret = set_byt_ec_platform_thermal_data();
	} else if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, CRV2) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, CRV2)) {
		ret = set_byt_cr_platform_thermal_data();
	} else {
		pr_err("Cannot detect exact BYT platform\n");
	}

	if (ret)
		pr_err("Configuring platform data failed:%d\n", ret);

	return ret;
}

/*
 * This needs to be earlier than subsys_initcall; so
 * that we set the pdata for thermal subsystem, before
 * the crystal_cove driver registers a device for the same.
 */
arch_initcall(byt_platform_thermal_init);
