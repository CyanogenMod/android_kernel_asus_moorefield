/*
 * platform_btns_regulator.c - Merrifield regulator machine drvier
 * Copyright (c) 2015, Intel Corporation.
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
#include <linux/regulator/intel_dollar_cove_pmic.h>
#include <linux/regulator/machine.h>

#include <asm/intel-mid.h>
#include <asm/intel_scu_pmic.h>

int ldo_regulator_init(void *driver_data)
{
	int ret;

	struct dcovex_regulator_info *pdata =
		(struct dcovex_regulator_info *)driver_data;

	if (pdata) {
		u8 reg;

		ret = intel_scu_ipc_ioread8(pdata->vol_reg, &reg);
		if (ret) {
			pr_err("intel_scu_ipc_ioread8 returns error %08x\n",
				ret);
			return ret;
		}

		reg &= 0xE0;
		switch (pdata->vol_reg) {
		case DCOVEX_LDO1_VOL_CTRL:
		case DCOVEX_LDO2_VOL_CTRL:
		case DCOVEX_LDO3_VOL_CTRL:
			reg |= 21;
			break;
		case DCOVEX_LDO4_VOL_CTRL:
			reg |= 18;
			break;
		default:
			break;
		}

		pr_debug("%s default voltage value: reg:0x%08X: %d\n", __func__,
				pdata->vol_reg, reg);
		ret = intel_scu_ipc_iowrite8(pdata->vol_reg, reg);
		if (ret) {
			pr_err("intel_scu_ipc_iowrite8 error %08x\n", ret);
			return ret;
		}
	}

	return 0;
}

/***********LDO1 REGULATOR platform data*************/
static struct regulator_consumer_supply ldo1_consumer[] = {
};
static struct regulator_init_data ldo1_data = {
	.constraints = {
		.min_uV			= 700000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo1_consumer),
	.consumer_supplies	= ldo1_consumer,
	.regulator_init         = ldo_regulator_init,
};
static struct dcovex_regulator_info ldo1_info = {
	.vol_reg   = DCOVEX_LDO1_VOL_CTRL,
	.init_data  = &ldo1_data,
};
static struct platform_device ldo1_device = {
	.name = "intel_regulator",
	.id = DCOVEX_ID_LDO1,
	.dev = {
		.platform_data = &ldo1_info,
	},
};

/***********LDO2 REGULATOR platform data*************/
static struct regulator_consumer_supply ldo2_consumer[] = {
};
static struct regulator_init_data ldo2_data = {
	.constraints = {
		.min_uV			= 700000,
		.max_uV			= 4200000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo2_consumer),
	.consumer_supplies	= ldo2_consumer,
	.regulator_init         = ldo_regulator_init,
};
static struct dcovex_regulator_info ldo2_info = {
	.vol_reg   = DCOVEX_LDO2_VOL_CTRL,
	.init_data  = &ldo2_data,
};
static struct platform_device ldo2_device = {
	.name = "intel_regulator",
	.id = DCOVEX_ID_LDO2,
	.dev = {
		.platform_data = &ldo2_info,
	},
};

/***********LDO3 REGULATOR platform data*************/
static struct regulator_consumer_supply ldo3_consumer[] = {
};
static struct regulator_init_data ldo3_data = {
	.constraints = {
		.min_uV			= 700000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo3_consumer),
	.consumer_supplies	= ldo3_consumer,
	.regulator_init         = ldo_regulator_init,
};
static struct dcovex_regulator_info ldo3_info = {
	.vol_reg   = DCOVEX_LDO3_VOL_CTRL,
	.init_data  = &ldo3_data,
};
static struct platform_device ldo3_device = {
	.name = "intel_regulator",
	.id = DCOVEX_ID_LDO3,
	.dev = {
		.platform_data = &ldo3_info,
	},
};

/***********LDO4 REGULATOR platform data*************/
static struct regulator_consumer_supply ldo4_consumer[] = {
};
static struct regulator_init_data ldo4_data = {
	.constraints = {
		.min_uV			= 700000,
		.max_uV			= 3300000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
	},
	.num_consumer_supplies	= ARRAY_SIZE(ldo4_consumer),
	.consumer_supplies	= ldo4_consumer,
	.regulator_init         = ldo_regulator_init,
};
static struct dcovex_regulator_info ldo4_info = {
	.vol_reg   = DCOVEX_LDO4_VOL_CTRL,
	.init_data  = &ldo4_data,
};
static struct platform_device ldo4_device = {
	.name = "intel_regulator",
	.id = DCOVEX_ID_LDO4,
	.dev = {
		.platform_data = &ldo4_info,
	},
};

static struct platform_device *regulator_devices[] __initdata = {
	&ldo1_device,
	&ldo2_device,
	&ldo3_device,
	&ldo4_device
};

static int __init regulator_init(void)
{
	platform_add_devices(regulator_devices,
		ARRAY_SIZE(regulator_devices));

	return 0;
}
device_initcall(regulator_init);
