/*
 * Crystal Cove Regulator Driver
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
 */
#include <linux/module.h>
#include <linux/gpio.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_crystal_cove_pmic.h>
#include <linux/regulator/machine.h>

#include <linux/mfd/intel_mid_pmic.h>

/* Intel Voltage cntrl register parameters*/
#define REG_ENA_STATUS_MASK	0x01
#define REG_VSEL_MASK		0xe0
#define VSEL_SHIFT		5

#define REG_ON			0x01
#define REG_OFF			0xfe
#define REG_CNT_ENBL		0x02

#define ON			1
#define OFF			0

const u16 reg_addr_offset[] = {
	V2P85SCNT_ADDR, V2P85SXCNT_ADDR, V3P3SXCNT_ADDR,
	V1P8SCNT_ADDR, V1P8SXCNT_ADDR, V1P0ACNT_ADDR,
	V1P8ACNT_ADDR, VSYS_SCNT_ADDR
};

static int intel_pmic_reg_is_enabled(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_value;

	reg_value = intel_mid_pmic_readb(pmic_info->pmic_reg);
	if (reg_value < 0) {
		dev_err(&rdev->dev,
			"intel_mid_pmic_readb returns error %08x\n", reg_value);
		return reg_value;
	}

	if (!(reg_value & REG_CNT_ENBL))
		return -EINVAL;

	return reg_value & REG_ENA_STATUS_MASK;
}

static int intel_pmic_reg_enable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_value;

	reg_value = intel_mid_pmic_readb(pmic_info->pmic_reg);
	if (reg_value < 0) {
		dev_err(&rdev->dev,
			"intel_mid_pmic_readb returns error %08x\n", reg_value);
		return reg_value;
	}

	return intel_mid_pmic_writeb(pmic_info->pmic_reg,
				(reg_value | REG_ON | REG_CNT_ENBL));
}

static int intel_pmic_reg_disable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_value;

	reg_value = intel_mid_pmic_readb(pmic_info->pmic_reg);
	if (reg_value < 0) {
		dev_err(&rdev->dev,
			"intel_mid_pmic_readb returns error %08x\n", reg_value);
		return reg_value;
	}

	return intel_mid_pmic_writeb(pmic_info->pmic_reg,
				((reg_value | REG_CNT_ENBL) & REG_OFF));
}

static int intel_pmic_reg_getvoltage_sel(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  vsel;
	int reg_value;

	reg_value = intel_mid_pmic_readb(pmic_info->pmic_reg);
	if (reg_value < 0) {
		dev_err(&rdev->dev,
			"intel_mid_pmic_readb returns error %08x\n", reg_value);
		return reg_value;
	}
	vsel = (reg_value & REG_VSEL_MASK) >> VSEL_SHIFT;

	return vsel;
}

static int intel_pmic_reg_setvoltage_sel(struct regulator_dev *rdev,
					unsigned selector)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_value;

	reg_value = intel_mid_pmic_readb(pmic_info->pmic_reg);
	if (reg_value < 0) {
		dev_err(&rdev->dev,
			"intel_mid_pmic_readb returns error %08x\n", reg_value);
		return reg_value;
	}

	reg_value &= ~REG_VSEL_MASK;
	reg_value |= selector << VSEL_SHIFT;

	return intel_mid_pmic_writeb(pmic_info->pmic_reg, reg_value);
}

static struct regulator_ops intel_pmic_regulator_ops = {
	.is_enabled = intel_pmic_reg_is_enabled,
	.enable = intel_pmic_reg_enable,
	.disable = intel_pmic_reg_disable,
	.get_voltage_sel = intel_pmic_reg_getvoltage_sel,
	.set_voltage_sel = intel_pmic_reg_setvoltage_sel,
	.list_voltage = regulator_list_voltage_table,
};

static struct regulator_desc intel_pmic_desc[] = {
	{
		.name = "v2p85s",
		.id = V2P85S,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(V2P85S_VSEL_TABLE),
		.volt_table = V2P85S_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "v2p85sx",
		.supply_name = "v2p85s",
		.id = V2P85SX,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(V2P85SX_VSEL_TABLE),
		.volt_table = V2P85SX_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "v3p3sx",
		.id = V3P3SX,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(V3P3SX_VSEL_TABLE),
		.volt_table = V3P3SX_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "v1p8s",
		.id = V1P8S,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(V1P8S_VSEL_TABLE),
		.volt_table = V1P8S_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "v1p8sx",
		.id = V1P8SX,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(V1P8SX_VSEL_TABLE),
		.volt_table = V1P8SX_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "v1p0a",
		.id = V1P0A,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(V1P0A_VSEL_TABLE),
		.volt_table = V1P0A_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "v1p8a",
		.id = V1P8A,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(V1P8A_VSEL_TABLE),
		.volt_table = V1P8A_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vsys_s",
		.id = VSYS_S,
		.ops = &intel_pmic_regulator_ops,
		.n_voltages = ARRAY_SIZE(VSYS_S_VSEL_TABLE),
		.volt_table = VSYS_S_VSEL_TABLE,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int crystal_cove_pmic_probe(struct platform_device *pdev)
{
	struct intel_pmic_info *pdata = dev_get_platdata(&pdev->dev);
	unsigned int i;
	static int no_of_regulator_probed;
	struct regulator_config config = { };

	if (!pdata || !pdata->pmic_reg)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(reg_addr_offset); i++) {
		if (reg_addr_offset[i] == pdata->pmic_reg)
			break;
	}

	if (i == (ARRAY_SIZE(reg_addr_offset)))
		return -EINVAL;

	/* set the initial setting for the gpio */
	if (pdata->en_pin) {
		config.ena_gpio = pdata->en_pin->gpio;
		config.ena_gpio_flags = pdata->en_pin->init_gpio_state;
	}

	config.dev = &pdev->dev;
	config.init_data = pdata->init_data;
	config.driver_data = pdata;

	pdata->intel_pmic_rdev = regulator_register(&intel_pmic_desc[i],
								&config);
	if (IS_ERR(pdata->intel_pmic_rdev)) {
		dev_err(&pdev->dev, "can't register regulator..error %ld\n",
				PTR_ERR(pdata->intel_pmic_rdev));
		return PTR_ERR(pdata->intel_pmic_rdev);
	}
	platform_set_drvdata(pdev, pdata->intel_pmic_rdev);
	dev_dbg(&pdev->dev, "registered regulator\n");

	return 0;
}

static int crystal_cove_pmic_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

static const struct platform_device_id crystal_cove_id_table[] = {
	{ "intel_regulator", 0 },
	{ },
};

MODULE_DEVICE_TABLE(platform, crystal_cove_id_table);

static struct platform_driver crystal_cove_pmic_driver = {
	.driver		= {
		.name = "intel_regulator",
		.owner = THIS_MODULE,
	},
	.probe = crystal_cove_pmic_probe,
	.remove = crystal_cove_pmic_remove,
	.id_table = crystal_cove_id_table,
};
static int __init crystal_cove_pmic_init(void)
{
	return platform_driver_register(&crystal_cove_pmic_driver);
}
late_initcall(crystal_cove_pmic_init);

static void __exit crystal_cove_pmic_exit(void)
{
	platform_driver_unregister(&crystal_cove_pmic_driver);
}
module_exit(crystal_cove_pmic_exit);

MODULE_DESCRIPTION("Crystal Cove voltage regulator driver");
MODULE_AUTHOR("Dyut/Sudarshan");
MODULE_LICENSE("GPL v2");
