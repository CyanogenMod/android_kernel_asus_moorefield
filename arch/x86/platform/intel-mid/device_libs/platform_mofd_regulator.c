/*
 * platform_mofd_regulator.c - Moorefield regulator machine drvier
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
#include <linux/regulator/intel_shady_cove_pmic.h>
#include <linux/regulator/machine.h>

#include <asm/intel-mid.h>
#include <asm/intel_scu_ipc.h>
#include <asm/intel_scu_pmic.h>

/***********VPROG1 REGUATOR platform data*************/
static struct regulator_consumer_supply vprog1_consumer[] = {
};

static struct regulator_init_data vprog1_data = {
	.constraints = {
		.min_uV			= 1500000,
		.max_uV			= 2800000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL,
	},
	.num_consumer_supplies	= ARRAY_SIZE(vprog1_consumer),
	.consumer_supplies	= vprog1_consumer,
};

static struct intel_pmic_info vprog1_info = {
	.pmic_reg   = VPROG1CNT_ADDR,
	.init_data  = &vprog1_data,
	.table_len  = ARRAY_SIZE(VPROG1_VSEL_table),
	.table      = VPROG1_VSEL_table,
};

static struct platform_device vprog1_device = {
	.name = "intel_regulator",
	.id = VPROG1,
	.dev = {
		.platform_data = &vprog1_info,
	},
};

/***********VPROG2 REGUATOR platform data*************/
static struct regulator_consumer_supply vprog2_consumer[] = {
};

static struct regulator_init_data vprog2_data = {
	.constraints = {
		.min_uV			= 1500000,
		.max_uV			= 2850000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
	},
	.num_consumer_supplies	= ARRAY_SIZE(vprog2_consumer),
	.consumer_supplies	= vprog2_consumer,
};

static struct intel_pmic_info vprog2_info = {
	.pmic_reg   = VPROG2CNT_ADDR,
	.init_data  = &vprog2_data,
	.table_len  = ARRAY_SIZE(VPROG2_VSEL_table),
	.table      = VPROG2_VSEL_table,
};

static struct platform_device vprog2_device = {
	.name = "intel_regulator",
	.id = VPROG2,
	.dev = {
		.platform_data = &vprog2_info,
	},
};

/***********VPROG3 REGUATOR platform data*************/
static struct regulator_consumer_supply vprog3_consumer[] = {
};

static struct regulator_init_data vprog3_data = {
	.constraints = {
		.min_uV			= 1050000,
		.max_uV			= 2900000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
	},
	.num_consumer_supplies	= ARRAY_SIZE(vprog2_consumer),
	.consumer_supplies	= vprog3_consumer,
};

static struct intel_pmic_info vprog3_info = {
	.pmic_reg   = VPROG3CNT_ADDR,
	.init_data  = &vprog3_data,
	.table_len  = ARRAY_SIZE(VPROG3_VSEL_table),
	.table      = VPROG3_VSEL_table,
};

static struct platform_device vprog3_device = {
	.name = "intel_regulator",
	.id = VPROG3,
	.dev = {
		.platform_data = &vprog3_info,
	},
};

/***********VFLEX REGUATOR platform data*************/
static struct regulator_consumer_supply vflex_consumer[] = {
};

static struct regulator_init_data vflex_data = {
	.constraints = {
		.min_uV			= 4500000,
		.max_uV			= 5000000,
		.valid_ops_mask		= REGULATOR_CHANGE_STATUS
			| REGULATOR_CHANGE_VOLTAGE,
		.valid_modes_mask	= REGULATOR_MODE_NORMAL
	},
	.num_consumer_supplies	= ARRAY_SIZE(vflex_consumer),
	.consumer_supplies	= vflex_consumer,
};

static struct intel_pmic_info vflex_info = {
	.pmic_reg   = VFLEXCNT_ADDR,
	.init_data  = &vflex_data,
	.table_len  = ARRAY_SIZE(VFLEX_VSEL_table),
	.table      = VFLEX_VSEL_table,
};

static struct platform_device vflex_device = {
	.name = "intel_regulator",
	.id = VFLEX,
	.dev = {
		.platform_data = &vflex_info,
	},
};

#define PMIC_ID_ADDR		0x00
#define PMIC_CHIP_ID_B0_VAL	0x08

static struct platform_device *regulator_devices[] __initdata = {
	&vprog1_device,
	&vprog2_device,
	&vprog3_device,
	&vflex_device,
};

static int __init regulator_init(void)
{
	int ret;
	uint8_t pmic_id;

	ret = intel_scu_ipc_ioread8(PMIC_ID_ADDR, &pmic_id);
	if (ret)
		pr_err("Failed to read PMIC ID!\n");

	if (PMIC_CHIP_ID_B0_VAL == pmic_id) {
		vprog1_info.pmic_reg = VPROG1CNT_B0_PMIC_ADDR;
		vprog2_info.pmic_reg = VPROG2CNT_B0_PMIC_ADDR;
		vprog3_info.pmic_reg = VPROG3CNT_B0_PMIC_ADDR;
	}

	/* register the regulator only if SoC is Tangier */
	if (intel_mid_identify_cpu() == INTEL_MID_CPU_CHIP_ANNIEDALE)
		platform_add_devices(regulator_devices,
				ARRAY_SIZE(regulator_devices));

	return 0;
}
device_initcall(regulator_init);
