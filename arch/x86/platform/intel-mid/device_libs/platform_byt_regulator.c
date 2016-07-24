/*
 * platform_byt_regulator.c - Baytrail regulator machine drvier
 * Copyright (c) 2013, Intel Corporation.
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

#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/intel_crystal_cove_pmic.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/intel_dcovex_regulator.h>
#include <linux/regulator/intel_dcovet_regulator.h>
#include <asm/spid.h>
#include <linux/i2c.h>
#include <linux/mfd/intel_mid_pmic.h>


/* 3P3SX regulator controlled over gpio */
#define GPIO_3P3SX_EN	151


/* To differentiate between BYTCR Versions based on PMIC */
#define REGULATOR_PMIC_HID_RHM		"INT33FD:00"
#define REGULATOR_PMIC_HID_XPR		"INT33F4:00"
#define REGULATOR_PMIC_HID_TEIN		"INT33F5:00"

/***********V2P85S REGUATOR platform data*************/
static struct regulator_consumer_supply v2p85s_consumer[] = {
};
static struct regulator_init_data v2p85s_data = {
	.constraints = {
		.name = "v2p85s",
		.min_uV			= 2565000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v2p85s_consumer),
	.consumer_supplies	= v2p85s_consumer,
};

static struct intel_pmic_info v2p85s_info = {
	.pmic_reg   = V2P85SCNT_ADDR,
	.init_data  = &v2p85s_data,
	.table_len  = ARRAY_SIZE(V2P85S_VSEL_TABLE),
	.table      = V2P85S_VSEL_TABLE,
};
static struct platform_device v2p85s_device = {
	.name = "intel_regulator",
	.id = V2P85S,
	.dev = {
		.platform_data = &v2p85s_info,
	},
};

/***********V2P85SX REGUATOR platform data*************/
static struct regulator_consumer_supply v2p85sx_consumer[] = {
};
static struct regulator_init_data v2p85sx_data = {
	.supply_regulator = "v2p85s",
	.constraints = {
		.name = "v2p85sx",
		.min_uV			= 2900000,
		.max_uV			= 2900000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v2p85sx_consumer),
	.consumer_supplies	= v2p85sx_consumer,
};

static struct intel_pmic_info v2p85sx_info = {
	.pmic_reg   = V2P85SXCNT_ADDR,
	.init_data  = &v2p85sx_data,
	.table_len  = ARRAY_SIZE(V2P85SX_VSEL_TABLE),
	.table      = V2P85SX_VSEL_TABLE,
};
static struct platform_device v2p85sx_device = {
	.name = "intel_regulator",
	.id = V2P85SX,
	.dev = {
		.platform_data = &v2p85sx_info,
	},
};

/***********V3P3S REGUATOR platform data*************/
static struct regulator_consumer_supply v3p3sx_consumer[] = {
/* Add consumers here */
	REGULATOR_SUPPLY("v3p3sx", "0000:00:02.0"), /* Display drm */
	REGULATOR_SUPPLY("v3p3sx", "1-0035"), /* smb347 charger */
};

static struct pmic_regulator_gpio_en v3p3sx_gpio_data = {
	.gpio = GPIO_3P3SX_EN,
	.init_gpio_state = GPIOF_OUT_INIT_HIGH,
};

static struct regulator_init_data v3p3sx_data = {
	.constraints = {
		.name = "v3p3sx",
		.min_uV			= 3332000,
		.max_uV			= 3332000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v3p3sx_consumer),
	.consumer_supplies	= v3p3sx_consumer,
};

static struct intel_pmic_info v3p3sx_info = {
	.pmic_reg   = V3P3SXCNT_ADDR,
	.init_data  = &v3p3sx_data,
	.table_len  = ARRAY_SIZE(V3P3SX_VSEL_TABLE),
	.table      = V3P3SX_VSEL_TABLE,
	.en_pin	=  &v3p3sx_gpio_data,
};

static struct platform_device v3p3sx_device = {
	.name = "intel_regulator",
	.id = V3P3SX,
	.dev = {
		.platform_data = &v3p3sx_info,
	},
};

/***********V1P8S REGUATOR platform data*************/
static struct regulator_consumer_supply v1p8s_consumer[] = {
};
static struct regulator_init_data v1p8s_data = {
	.constraints = {
		.name = "v1p8s",
		.min_uV			= 1817000,
		.max_uV			= 1817000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v1p8s_consumer),
	.consumer_supplies	= v1p8s_consumer,
};

static struct intel_pmic_info v1p8s_info = {
	.pmic_reg   = V1P8SCNT_ADDR,
	.init_data  = &v1p8s_data,
	.table_len  = ARRAY_SIZE(V1P8S_VSEL_TABLE),
	.table      = V1P8S_VSEL_TABLE,
};
static struct platform_device v1p8s_device = {
	.name = "intel_regulator",
	.id = V1P8S,
	.dev = {
		.platform_data = &v1p8s_info,
	},
};

/* V1P8SX regulator platform data */
static struct regulator_consumer_supply v1p8sx_consumer[] = {
};

static struct regulator_init_data v1p8sx_data = {
	.constraints = {
		.name = "v1p8sx",
		.min_uV			= 1817000,
		.max_uV			= 1817000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v1p8sx_consumer),
	.consumer_supplies	= v1p8sx_consumer,
};

static struct intel_pmic_info v1p8sx_info = {
	.pmic_reg   = V1P8SXCNT_ADDR,
	.init_data  = &v1p8sx_data,
	.table_len  = ARRAY_SIZE(V1P8SX_VSEL_TABLE),
	.table      = V1P8SX_VSEL_TABLE,
};

static struct platform_device v1p8sx_device = {
	.name = "intel_regulator",
	.id = V1P8SX,
	.dev = {
		.platform_data = &v1p8sx_info,
	},
};

/***********VSYS_S REGUATOR platform data*************/
static struct regulator_consumer_supply vsys_s_consumer[] = {
};
static struct regulator_init_data vsys_s_data = {
	.constraints = {
		.name = "vsys_s",
		.min_uV			= 4200000,
		.max_uV			= 4200000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(vsys_s_consumer),
	.consumer_supplies	= vsys_s_consumer,
};

static struct intel_pmic_info vsys_s_info = {
	.pmic_reg   = VSYS_SCNT_ADDR,
	.init_data  = &vsys_s_data,
	.table_len  = ARRAY_SIZE(VSYS_S_VSEL_TABLE),
	.table      = VSYS_S_VSEL_TABLE,
};
static struct platform_device vsys_s_device = {
	.name = "intel_regulator",
	.id = VSYS_S,
	.dev = {
		.platform_data = &vsys_s_info,
	},
};

/* v1p0a regulator platform data */
static struct regulator_consumer_supply v1p0a_consumer[] = {
/* Add consumers */
};
static struct regulator_init_data v1p0a_data = {
	.constraints = {
		.name = "v1p0a",
		.min_uV			= 900000,
		.max_uV			= 1100000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v1p0a_consumer),
	.consumer_supplies	= v1p0a_consumer,
};

static struct intel_pmic_info v1p0a_info = {
	.pmic_reg   = V1P0ACNT_ADDR,
	.init_data  = &v1p0a_data,
	.table_len  = ARRAY_SIZE(V1P0A_VSEL_TABLE),
	.table      = V1P0A_VSEL_TABLE,
};
static struct platform_device v1p0a_device = {
	.name = "intel_regulator",
	.id = V1P0A,
	.dev = {
		.platform_data = &v1p0a_info,
	},
};

/* v1p8a regulator platform data */
static struct regulator_consumer_supply v1p8a_consumer[] = {
/* Add consumers */
};
static struct regulator_init_data v1p8a_data = {
	.constraints = {
		.name = "v1p8a",
		.min_uV			= 1620000,
		.max_uV			= 1980000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS |
						REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
		.boot_on		= 1,
	},
	.num_consumer_supplies	= ARRAY_SIZE(v1p8a_consumer),
	.consumer_supplies	= v1p8a_consumer,
};

static struct intel_pmic_info v1p8a_info = {
	.pmic_reg   = V1P8ACNT_ADDR,
	.init_data  = &v1p8a_data,
	.table_len  = ARRAY_SIZE(V1P8A_VSEL_TABLE),
	.table      = V1P8A_VSEL_TABLE,
};
static struct platform_device v1p8a_device = {
	.name = "intel_regulator",
	.id = V1P8A,
	.dev = {
		.platform_data = &v1p8a_info,
	},
};

static struct platform_device *byt_t_ffd8_regulator_devices[] __initdata = {
	&v2p85s_device,
	&v2p85sx_device,
	&v3p3sx_device,
	&v1p8s_device,
	&v1p8sx_device,
	&vsys_s_device,
	&v1p0a_device,
	&v1p8a_device,
};

/**************** Regulator Devices for BYTCRV2.0 ******************/
static struct platform_device *byt_t_crv20_regulator_devices[] __initdata = {
	&v2p85s_device,
	&v2p85sx_device,
	&v3p3sx_device,
	&v1p8s_device,
	&v1p8sx_device,
	&vsys_s_device,
};

/****************DCOVEX LDO8 RAIL Platform Data for CRV2.2****************/
static struct regulator_consumer_supply dcovet_ldo8_consumer[] = {
	REGULATOR_SUPPLY("vmmc", "80860F14:01"),
	/* Add More Consumers here */
};

static struct regulator_init_data dcovet_ldo8_data = {
	.constraints = {
		.name			= "LDO_8",
		.min_uV			= 1800000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(dcovet_ldo8_consumer),
	.consumer_supplies	= dcovet_ldo8_consumer,
};

static struct dcovet_regulator_info dcovet_ldo8 = {
	.init_data = &dcovet_ldo8_data,
};

static struct platform_device dcovet_ldo8_device = {
	.name = "dcovet_regulator",
	.id = DCOVET_ID_LDO8,
	.dev = {
		.platform_data = &dcovet_ldo8,
	},
};

/****************DCOVEX LDO_7 RAIL Platform Data for CRV2.2****************/
static struct regulator_consumer_supply dcovet_ldo7_consumer[] = {
	REGULATOR_SUPPLY("vqmmc", "80860F14:01"),
	/* Add More Consumers here */
};

static struct regulator_init_data dcovet_ldo7_data = {
	.constraints = {
		.name			= "LDO_7",
		.min_uV			= 1800000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_VOLTAGE |
						REGULATOR_CHANGE_STATUS,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(dcovet_ldo7_consumer),
	.consumer_supplies	= dcovet_ldo7_consumer,
};

static struct dcovet_regulator_info dcovet_ldo7 = {
	.init_data = &dcovet_ldo7_data,
};

static struct platform_device dcovet_ldo7_device = {
	.name = "dcovet_regulator",
	.id = DCOVET_ID_LDO7,
	.dev = {
		.platform_data = &dcovet_ldo7,
	},
};


/**************** Regulator Devices for BYTCRV2.2 ******************/
static struct platform_device *byt_t_crv22_regulator_devices[] __initdata = {
	&dcovet_ldo8_device,
	&dcovet_ldo7_device,
};


static int regulator_match_name(struct device *dev, void *data)
{
	const char *name = data;
	struct i2c_client *client = to_i2c_client(dev);
	return !strncmp(client->name, name, strlen(client->name));
}

static struct i2c_client *regulator_find_i2c_client_by_name(char *name)
{
	struct device *dev = bus_find_device(&i2c_bus_type, NULL,
						name, regulator_match_name);
	return dev ? to_i2c_client(dev) : NULL;
}

static int __init regulator_init(void)
{
	if (spid.hardware_id == BYT_TABLET_BLK_CRV2) {

		if (regulator_find_i2c_client_by_name(REGULATOR_PMIC_HID_RHM)) {
			pr_info("Regulator Devices Found for BYTCRv2.0 (RHM)\n");
			platform_add_devices(byt_t_crv20_regulator_devices,
				ARRAY_SIZE(byt_t_crv20_regulator_devices));
		}

		else if (regulator_find_i2c_client_by_name(REGULATOR_PMIC_HID_TEIN)) {
			pr_info("Regulator Devices Found for BYTCRv2.2 (TE. IN.)\n");
			platform_add_devices(byt_t_crv22_regulator_devices,
				ARRAY_SIZE(byt_t_crv22_regulator_devices));
		}

		else {
			pr_err("%s: No Regulator Devices Found for BYT_CR :: \
				Version NOT DETECTED !!\n", __func__);
			return -ENODEV;
		}
	}

	else {
		platform_add_devices(byt_t_ffd8_regulator_devices,
			ARRAY_SIZE(byt_t_ffd8_regulator_devices));
	}

	return 0;
}
device_initcall(regulator_init);
