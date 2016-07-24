/*
 * platform_cht_regulator.c - Cherrytrail regulator machine drvier
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
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 *
 */

#include <linux/platform_device.h>
#include <linux/regulator/intel_crystal_cove_plus_pmic.h>
#include <linux/regulator/machine.h>

/***********V3P3A REGUATOR platform data*************/
static struct regulator_consumer_supply v3p3a_consumer[] = { };

static struct regulator_init_data v3p3a_data = {
	.constraints = {
		.name = "v3p3a",
		.min_uV = 2970000,
		.max_uV = 3630000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v3p3a_consumer),
	.consumer_supplies = v3p3a_consumer,
};

static struct ccovep_regulator_info v3p3a_info = {
	.init_data = &v3p3a_data,
};

static struct platform_device v3p3a_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_V3P3A,
	.dev = {
		.platform_data = &v3p3a_info,
	},
};

/***********V3P3SX REGUATOR platform data*************/
static struct regulator_consumer_supply v3p3sx_consumer[] = { };

static struct regulator_init_data v3p3sx_data = {
	.constraints = {
		.name = "v3p3sx",
		.min_uV = 2970000,
		.max_uV = 3630000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v3p3sx_consumer),
	.consumer_supplies = v3p3sx_consumer,
};

static struct ccovep_regulator_info v3p3sx_info = {
	.init_data = &v3p3sx_data,
};

static struct platform_device v3p3sx_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_V3P3SX,
	.dev = {
		.platform_data = &v3p3sx_info,
	},
};

/***********V1P05A REGUATOR platform data*************/
static struct regulator_consumer_supply v1p05a_consumer[] = { };

static struct regulator_init_data v1p05a_data = {
	.constraints = {
		.name = "v1p05a",
		.min_uV = 945000,
		.max_uV = 1155000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v1p05a_consumer),
	.consumer_supplies = v1p05a_consumer,
};

static struct ccovep_regulator_info v1p05a_info = {
	.init_data = &v1p05a_data,
};

static struct platform_device v1p05a_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_V1P05A,
	.dev = {
		.platform_data = &v1p05a_info,
	},
};

/***********V1P15 REGUATOR platform data*************/
static struct regulator_consumer_supply v1p15_consumer[] = { };

static struct regulator_init_data v1p15_data = {
	.constraints = {
		.name = "v1p15",
		.min_uV = 1040000,
		.max_uV = 1270000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v1p15_consumer),
	.consumer_supplies = v1p15_consumer,
};

static struct ccovep_regulator_info v1p15_info = {
	.init_data = &v1p15_data,
};

static struct platform_device v1p15_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_V1P15,
	.dev = {
		.platform_data = &v1p15_info,
	},
};

/***********V1P8A REGUATOR platform data*************/
static struct regulator_consumer_supply v1p8a_consumer[] = { };

static struct regulator_init_data v1p8a_data = {
	.constraints = {
		.name = "v1p8a",
		.min_uV = 1635000,
		.max_uV = 1999000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v1p8a_consumer),
	.consumer_supplies = v1p8a_consumer,
};

static struct ccovep_regulator_info v1p8a_info = {
	.init_data = &v1p8a_data,
};

static struct platform_device v1p8a_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_V1P8A,
	.dev = {
		.platform_data = &v1p8a_info,
	},
};

/***********V5P0A REGUATOR platform data*************/
static struct regulator_consumer_supply v5p0a_consumer[] = { };

static struct regulator_init_data v5p0a_data = {
	.constraints = {
		.name = "v5p0a",
		.min_uV = 4500000,
		.max_uV = 5500000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v5p0a_consumer),
	.consumer_supplies = v5p0a_consumer,
};

static struct ccovep_regulator_info v5p0a_info = {
	.init_data = &v5p0a_data,
};

static struct platform_device v5p0a_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_V5P0A,
	.dev = {
		.platform_data = &v5p0a_info,
	},
};

/***********V2P8SX REGUATOR platform data*************/
static struct regulator_consumer_supply v2p8sx_consumer[] = { };

static struct regulator_init_data v2p8sx_data = {
	.constraints = {
		.name = "v2p8sx",
		.min_uV = 2565000,
		.max_uV = 3300000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(v2p8sx_consumer),
	.consumer_supplies = v2p8sx_consumer,
};

static struct ccovep_regulator_info v2p8sx_info = {
	.init_data = &v2p8sx_data,
};

static struct platform_device v2p8sx_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_V2P8SX,
	.dev = {
		.platform_data = &v2p8sx_info,
	},
};

/***********VDDQ REGUATOR platform data*************/
static struct regulator_consumer_supply vddq_consumer[] = { };

static struct regulator_init_data vddq_data = {
	.constraints = {
		.name = "vddq",
		.min_uV = 1080000,
		.max_uV = 1500000,
		.valid_ops_mask = REGULATOR_CHANGE_STATUS |
			REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask = REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies = ARRAY_SIZE(vddq_consumer),
	.consumer_supplies = vddq_consumer,
};

static struct ccovep_regulator_info vddq_info = {
	.init_data = &vddq_data,
};

static struct platform_device vddq_device = {
	.name = "ccovep_regulator",
	.id = CCOVEP_ID_VDDQ,
	.dev = {
		.platform_data = &vddq_info,
	},
};


static struct platform_device *regulator_devices[] __initdata = {
	&v3p3a_device,
	&v3p3sx_device,
	&v1p05a_device,
	&v1p15_device,
	&v1p8a_device,
	&v5p0a_device,
	&v2p8sx_device,
	&vddq_device,
};

static int __init regulator_init(void)
{
	platform_add_devices(regulator_devices, ARRAY_SIZE(regulator_devices));
	return 0;
}
device_initcall(regulator_init);
