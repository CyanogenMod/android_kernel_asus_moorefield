/*
 * platform_cht_wcove_regulator.c - Cherrytrail regulator machine driver for
 * WhiskeyCove pmic
 *
 * Copyright (c) 2014, Intel Corporation.
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#include <linux/platform_device.h>
#include <linux/regulator/intel_whiskey_cove_pmic.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/intel_mid_pmic.h>

/***********V3P3A REGUATOR platform data*************/
static struct regulator_consumer_supply v3p3a_consumer[] = { };

static struct regulator_init_data v3p3a_data = {
	.constraints = {
		.name = "V3P3A",
		.min_uV = 3000000,
		.max_uV = 3350000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v3p3a_consumer),
	.consumer_supplies = v3p3a_consumer,
};

static struct wcove_regulator_info v3p3a_info = {
	.init_data = &v3p3a_data,
};

/***********V1P8A REGUATOR platform data*************/
static struct regulator_consumer_supply v1p8a_consumer[] = { };

static struct regulator_init_data v1p8a_data = {
	.constraints = {
		.name = "V1P8A",
		.min_uV = 250000,
		.max_uV = 2100000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v1p8a_consumer),
	.consumer_supplies = v1p8a_consumer,
};

static struct wcove_regulator_info v1p8a_info = {
	.init_data = &v1p8a_data,
};

/***********V1P8SX REGUATOR platform data*************/
static struct regulator_consumer_supply v1p8sx_consumer[] = { };

static struct regulator_init_data v1p8sx_data = {
	.constraints = {
		.name = "V1P8SX",
		.min_uV = 800000,
		.max_uV = 3400000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v1p8sx_consumer),
	.consumer_supplies = v1p8sx_consumer,
};

static struct wcove_regulator_info v1p8sx_info = {
	.init_data = &v1p8sx_data,
};

/***********V1P2A REGUATOR platform data*************/
static struct regulator_consumer_supply v1p2a_consumer[] = { };

static struct regulator_init_data v1p2a_data = {
	.constraints = {
		.name = "V1P2A",
		.min_uV = 800000,
		.max_uV = 3400000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v1p2a_consumer),
	.consumer_supplies = v1p2a_consumer,
};

static struct wcove_regulator_info v1p2a_info = {
	.init_data = &v1p2a_data,
};

/***********V1P2SX REGUATOR platform data*************/
static struct regulator_consumer_supply v1p2sx_consumer[] = { };

static struct regulator_init_data v1p2sx_data = {
	.constraints = {
		.name = "V1P2SX",
		.min_uV = 800000,
		.max_uV = 3400000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v1p2sx_consumer),
	.consumer_supplies = v1p2sx_consumer,
};

static struct wcove_regulator_info v1p2sx_info = {
	.init_data = &v1p2sx_data,
};

/***********V2P8SX REGUATOR platform data*************/
static struct regulator_consumer_supply v2p8sx_consumer[] = { };

static struct regulator_init_data v2p8sx_data = {
	.constraints = {
		.name = "V2P8SX",
		.min_uV = 800000,
		.max_uV = 3400000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v2p8sx_consumer),
	.consumer_supplies = v2p8sx_consumer,
};

static struct wcove_regulator_info v2p8sx_info = {
	.init_data = &v2p8sx_data,
};

/***********VDDQ REGUATOR platform data*************/
static struct regulator_consumer_supply vddq_consumer[] = { };

static struct regulator_init_data vddq_data = {
	.constraints = {
		.name = "VDDQ",
		.min_uV = 250000,
		.max_uV = 1440000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(vddq_consumer),
	.consumer_supplies = vddq_consumer,
};

static struct wcove_regulator_info vddq_info = {
	.init_data = &vddq_data,
};

/***********VSDIO REGUATOR platform data*************/
static struct regulator_consumer_supply vsdio_consumer[] = {
	REGULATOR_SUPPLY("vqmmc", "INT33BB:01"),
};

static struct regulator_init_data vsdio_data = {
	.constraints = {
		.name = "VSDIO",
		.min_uV = 1700000,
		.max_uV = 3400000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(vsdio_consumer),
	.consumer_supplies = vsdio_consumer,
};

static struct wcove_regulator_info vsdio_info = {
	.init_data = &vsdio_data,
};

/***********V3P3SD REGUATOR platform data*************/
static struct regulator_consumer_supply v3p3sd_consumer[] = { };

static struct regulator_init_data v3p3sd_data = {
	.constraints = {
		.name = "V3P3SD",
		.min_uV = 3000000,
		.max_uV = 3400000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v3p3sd_consumer),
	.consumer_supplies = v3p3sd_consumer,
};

static struct wcove_regulator_info v3p3sd_info = {
	.init_data = &v3p3sd_data,
};

static int __init regulator_init(void)
{
	intel_mid_pmic_set_pdata("wcove_regulator", &v3p3a_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_V3P3A + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &v1p8a_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_V1P8A + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &v1p8sx_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_V1P8SX + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &v1p2a_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_V1P2A + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &v1p2sx_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_V1P2SX + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &v2p8sx_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_V2P8SX + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &vddq_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_VDDQ + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &vsdio_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_VSDIO + 1);
	intel_mid_pmic_set_pdata("wcove_regulator", &v3p3sd_info, sizeof(struct
				wcove_regulator_info), WCOVE_ID_V3P3SD + 1);
	return 0;
}
device_initcall(regulator_init);
