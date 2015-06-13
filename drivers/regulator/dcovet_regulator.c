/*
 * Regulator driver for DollarCove TB PMIC
 *	(Based on Te. Ins. Design)
 * Copyright(c) 2014 Intel Corporation
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/regulator/intel_dcovet_regulator.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/delay.h>

/* LDO control registers */
#define DCOVET_LDO6_CTRL		0x46
#define DCOVET_LDO7_CTRL		0x47
#define DCOVET_LDO8_CTRL		0x48
#define DCOVET_LDO9_CTRL		0x49
#define DCOVET_LDO10_CTRL		0x4A
#define DCOVET_LDO11_CTRL		0x4B
#define DCOVET_LDO12_CTRL		0x4C
#define DCOVET_LDO13_CTRL		0x4D
#define DCOVET_LDO14_CTRL		0x4E

#define DCOVET_LDO_ENABLE_BIT		0x1

#define DCOVET_VOLT_CTRL_EDGEMASK	0x3E	/* 111110 */

static const unsigned int LDO6_14_table[] = {
	900000, 950000, 1000000, 1050000, 1100000, 1150000, 1200000, 1250000, 1300000, 1350000,
	1400000, 1450000, 1500000, 1550000, 1600000, 1650000, 1700000, 1750000, 1800000, 1850000,
	1900000, 1950000, 2000000, 2050000, 2100000, 2150000, 2200000, 2250000, 2300000, 2350000,
	2400000, 2450000, 2500000, 2550000, 2600000, 2650000, 2700000, 2750000, 2800000, 2850000,
	2900000, 2950000, 3000000, 3050000, 3100000, 3150000, 3200000, 3250000, 3300000,
};

static int dcovet_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct dcovet_regulator_info *info = rdev_get_drvdata(rdev);
	int reg_val;

	reg_val = intel_mid_pmic_readb(info->ctrl_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	return reg_val & DCOVET_LDO_ENABLE_BIT;
}

static int dcovet_regulator_enable(struct regulator_dev *rdev)
{
	struct dcovet_regulator_info *info = rdev_get_drvdata(rdev);
	int reg_val;

	/* enable_reg is output power on-off control register */
	reg_val = intel_mid_pmic_readb(info->ctrl_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	reg_val |= DCOVET_LDO_ENABLE_BIT;

	return intel_mid_pmic_writeb(info->ctrl_reg, reg_val);
}

static int dcovet_regulator_disable(struct regulator_dev *rdev)
{
	struct dcovet_regulator_info *info = rdev_get_drvdata(rdev);
	int reg_val;

	reg_val = intel_mid_pmic_readb(info->ctrl_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	reg_val &= ~DCOVET_LDO_ENABLE_BIT;

	return intel_mid_pmic_writeb(info->ctrl_reg, reg_val);
}

static int dcovet_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct dcovet_regulator_info *info = rdev_get_drvdata(rdev);
	int reg_val, mask;

	/* vol_reg is voltage control registers */
	reg_val = intel_mid_pmic_readb(info->ctrl_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	mask = ((1 << info->volt_nbits) - 1) << info->volt_shift;
	reg_val = (reg_val & mask) >> info->volt_shift;

	return DCOVET_VOLT_CTRL_EDGEMASK - reg_val;
}

static int dcovet_regulator_set_voltage_sel(struct regulator_dev *rdev,
				unsigned selector)
{
	struct dcovet_regulator_info *info = rdev_get_drvdata(rdev);
	int reg_val, mask;

	reg_val = intel_mid_pmic_readb(info->ctrl_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	mask = ((1 << info->volt_nbits) - 1) << info->volt_shift;
	reg_val &= ~mask;
	reg_val |= ((DCOVET_VOLT_CTRL_EDGEMASK - selector) << info->volt_shift);

	return intel_mid_pmic_writeb(info->ctrl_reg, reg_val);
}

static struct regulator_ops dcovet_regulator_ops = {
	.enable			= dcovet_regulator_enable,
	.disable		= dcovet_regulator_disable,
	.is_enabled		= dcovet_regulator_is_enabled,
	.get_voltage_sel	= dcovet_regulator_get_voltage_sel,
	.set_voltage_sel	= dcovet_regulator_set_voltage_sel,
	.list_voltage		= regulator_list_voltage_table,
};

/* TODO: If the mfd is regmap complaint, most of the mask, shifts not needed */
#define DCOVET_LDO(_id, vreg, shift, nbits)				\
{									\
	.desc	= {							\
		.name	= "LDO" #_id,					\
		.ops	= &dcovet_regulator_ops,			\
		.type	= REGULATOR_VOLTAGE,				\
		.id	= DCOVET_ID_LDO##_id,				\
		.n_voltages = ARRAY_SIZE(LDO6_14_table),		\
		.volt_table = LDO6_14_table,				\
		.owner	= THIS_MODULE,					\
	},								\
	.ctrl_reg	= DCOVET_##vreg##_CTRL,				\
	.volt_shift	= (shift),					\
	.volt_nbits	= (nbits),					\
}

static struct dcovet_regulator_info dcovet_regulator_info[] = {
	DCOVET_LDO(6, LDO6, 1, 6),
	DCOVET_LDO(7, LDO7, 1, 6),
	DCOVET_LDO(8, LDO8, 1, 6),
	DCOVET_LDO(9, LDO9, 1, 6),
	DCOVET_LDO(10, LDO10, 1, 6),
	DCOVET_LDO(11, LDO11, 1, 6),
	DCOVET_LDO(12, LDO12, 1, 6),
	DCOVET_LDO(13, LDO13, 1, 6),
	DCOVET_LDO(14, LDO14, 1, 6),
};

static inline struct dcovet_regulator_info *find_regulator_info(int id)
{
	struct dcovet_regulator_info *di;
	int i;

	for (i = 0; i < ARRAY_SIZE(dcovet_regulator_info); i++) {
		di = &dcovet_regulator_info[i];
		if (di->desc.id == id)
			return di;
	}
	return NULL;
}

static int dcovet_regulator_probe(struct platform_device *pdev)
{
	struct dcovet_regulator_info *pdata = dev_get_platdata(&pdev->dev);
	struct regulator_config config = { };
	struct dcovet_regulator_info *di = NULL;

	if (!pdata) {
		dev_err(&pdev->dev, "No dcovet_regulator_info\n");
		return -EINVAL;
	}

	di = find_regulator_info(pdev->id);
	if (di == NULL) {
		dev_err(&pdev->dev, "invalid regulator\n");
		return -EINVAL;
	}

	config.dev = &pdev->dev;
	config.init_data = pdata->init_data;
	config.driver_data = di;

	pdata->regulator = regulator_register(&di->desc, &config);
	if (IS_ERR(pdata->regulator)) {
		dev_err(&pdev->dev, "failed to register DCOVE TB regulator %s\n",
			pdata->desc.name);
		return PTR_ERR(pdata->regulator);
	}

	platform_set_drvdata(pdev, pdata->regulator);

	dev_dbg(&pdev->dev, "registered dollarcove TB regulator as %s\n",
			dev_name(&pdata->regulator->dev));

	return 0;
}

static int dcovet_regulator_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

static const struct platform_device_id dcovet_regulator_id_table[] = {
	{ "dcovet_regulator", 0 },
	{ },
};
MODULE_DEVICE_TABLE(platform, dcovet_regulator_id_table);

static struct platform_driver dcovet_regulator_driver = {
	.driver	= {
		.name	= "dcovet_regulator",
		.owner	= THIS_MODULE,
	},
	.probe		= dcovet_regulator_probe,
	.remove		= dcovet_regulator_remove,
	.id_table	= dcovet_regulator_id_table,
};

static int __init dcovet_regulator_init(void)
{
	return platform_driver_register(&dcovet_regulator_driver);
}
module_init(dcovet_regulator_init);

static void __exit dcovet_regulator_exit(void)
{
	platform_driver_unregister(&dcovet_regulator_driver);
}
module_exit(dcovet_regulator_exit);

MODULE_DESCRIPTION("DollarCove TB regulator driver");
MODULE_AUTHOR("Souvik K Chakravarty <souvik.k.chakravarty@intel.com>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:dcovet_regulator");
