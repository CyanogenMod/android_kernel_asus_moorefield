/*
 * pmic_shady_cove.c - Moorefield regulator driver
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
 */

#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_shady_cove_pmic.h>
#include <linux/regulator/machine.h>
#include <asm/intel_scu_pmic.h>

/* Intel Voltage cntrl register parameters*/
#define REG_ENA_STATUS_MASK	0x01
#define REG_VSEL_MASK		0xc0
#define VSEL_SHIFT		6

#define REG_ON			0x01
#define REG_OFF			0xfe

const u16 reg_addr_offset[] = {
		VPROG1CNT_ADDR, VPROG2CNT_ADDR,
		VPROG3CNT_ADDR, VFLEXCNT_ADDR
};

static int intel_pmic_reg_is_enabled(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8 reg;
	int ret;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}

	return reg & REG_ENA_STATUS_MASK;
}

static int intel_pmic_reg_enable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg;
	int ret;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}
	return intel_scu_ipc_iowrite8(pmic_info->pmic_reg, (reg | REG_ON));
}

static int intel_pmic_reg_disable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg;
	int ret;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}
	return intel_scu_ipc_iowrite8(pmic_info->pmic_reg,
		(reg & REG_OFF));
}

static int intel_pmic_reg_getvoltage(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg, vsel;
	int ret;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 returns error %08x\n", ret);
		return ret;
	}

	vsel = (reg & REG_VSEL_MASK) >> VSEL_SHIFT;
	dev_dbg(&rdev->dev, "Voltage value is %d mV\n",
		vsel);

	return vsel;
}

static int intel_pmic_reg_setvoltage(struct regulator_dev *rdev,
						unsigned selector)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int ret;
	u8 reg;

	ret = intel_scu_ipc_ioread8(pmic_info->pmic_reg, &reg);
	if (ret) {
		dev_err(&rdev->dev,
			"intel_scu_ipc_ioread8 error %08x\n", ret);
		return ret;
	}
	reg &= ~REG_VSEL_MASK;
	reg |= selector << VSEL_SHIFT;

	return intel_scu_ipc_iowrite8(pmic_info->pmic_reg, reg);
}

static struct regulator_ops intel_pmic_ops = {
	.is_enabled = intel_pmic_reg_is_enabled,
	.enable = intel_pmic_reg_enable,
	.disable = intel_pmic_reg_disable,
	.get_voltage_sel = intel_pmic_reg_getvoltage,
	.set_voltage_sel = intel_pmic_reg_setvoltage,
	.list_voltage = regulator_list_voltage_table,
};

static struct regulator_desc intel_pmic_desc[] = {
	{
		.name = "vprog1",
		.id = VPROG1,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG1_VSEL_table),
		.volt_table = VPROG1_VSEL_table,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vprog2",
		.id = VPROG2,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG2_VSEL_table),
		.volt_table = VPROG2_VSEL_table,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vprog3",
		.id = VPROG3,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG3_VSEL_table),
		.volt_table = VPROG3_VSEL_table,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vflex",
		.id = VFLEX,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VFLEX_VSEL_table),
		.volt_table = VFLEX_VSEL_table,
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int shady_cove_pmic_probe(struct platform_device *pdev)
{
	struct intel_pmic_info *pdata = dev_get_platdata(&pdev->dev);
	struct regulator_config config = { };
	unsigned int i;

	if (!pdata || !pdata->pmic_reg)
		return -EINVAL;

	config.dev = &pdev->dev;
	config.init_data = pdata->init_data;
	config.driver_data = pdata;

	for (i = 0; i < ARRAY_SIZE(reg_addr_offset); i++) {
		if (reg_addr_offset[i] == pdata->pmic_reg)
			break;
	}
	if (i == (ARRAY_SIZE(reg_addr_offset)))
		return -EINVAL;

	pdata->intel_pmic_rdev =
	regulator_register(&intel_pmic_desc[i], &config);
	if (IS_ERR(pdata->intel_pmic_rdev)) {
		dev_err(&pdev->dev, "can't register regulator..error %ld\n",
				PTR_ERR(pdata->intel_pmic_rdev));
		return PTR_ERR(pdata->intel_pmic_rdev);
	}
	platform_set_drvdata(pdev, pdata->intel_pmic_rdev);
	dev_dbg(&pdev->dev, "registered regulator\n");
	return 0;
}

static int shady_cove_pmic_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

static const struct platform_device_id shady_cove_id_table[] = {
	{ "intel_regulator", 0 },
	{ },
};

MODULE_DEVICE_TABLE(platform, shady_cove_id_table);

static struct platform_driver shady_cove_pmic_driver = {
	.driver		= {
		.name = "intel_regulator",
		.owner = THIS_MODULE,
	},
	.probe = shady_cove_pmic_probe,
	.remove = shady_cove_pmic_remove,
	.id_table = shady_cove_id_table,
};
static int __init shady_cove_pmic_init(void)
{
	return platform_driver_register(&shady_cove_pmic_driver);
}
subsys_initcall(shady_cove_pmic_init);

static void __exit shady_cove_pmic_exit(void)
{
	platform_driver_unregister(&shady_cove_pmic_driver);
}
module_exit(shady_cove_pmic_exit);

MODULE_DESCRIPTION("Shady Cove voltage regulator driver");
MODULE_AUTHOR("Nivedha Krishnakumar <nivedha.krishnakumar@intel.com>");
MODULE_LICENSE("GPL v2");
