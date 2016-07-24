/*
 * pmic_crystal_cove_plus.c - CherryTrail regulator driver
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/machine.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/regulator/intel_crystal_cove_plus_pmic.h>

/* crystal cove plus pmic register parameters */
#define CCOVEP_REG_VSEL_SHIFT 5
#define CCOVEP_REG_VSEL_MASK 0xe0
#define CCOVEP_REG_ENBL_MASK 0x01
#define CCOVEP_REG_DSBL_MASK 0xfe
#define CCOVEP_REG_ENBL_CTRL_MASK 0x02

/* voltage control regulator offsets */

/* buck boost regulators */
#define CCOVEP_V3P3A_CTRL	0x5e
#define CCOVEP_V3P3SX_CTRL	0x5f
/* buck regulators */
#define CCOVEP_V1P05A_CTRL	0x3b
#define CCOVEP_V1P15_CTRL	0x3c
#define CCOVEP_V1P8A_CTRL	0x56
/* boot regulators */
#define CCOVEP_V5P0A_CTRL	0x60
/* linear regulators */
#define CCOVEP_V2P8SX_CTRL	0x5d
#define CCOVEP_VDDQ_CTRL	0x58

/* voltage tables */
static const unsigned int CCOVEP_V3P3A_VSEL_TABLE[] = {
	2970000, 3135000, 3300000, 3332000, 3340000, 3400000, 3465000, 3630000,
};
static const unsigned int CCOVEP_V3P3SX_VSEL_TABLE[] = {
	2970000, 3135000, 3300000, 3332000, 3340000, 3400000, 3465000, 3630000,
};
static const unsigned int CCOVEP_V1P05A_VSEL_TABLE[] = {
	945000, 998000, 1040000, 1050000, 1061000, 1071000, 1103000, 1155000,
};
static const unsigned int CCOVEP_V1P15_VSEL_TABLE[] = {
	1040000, 1090000, 1140000, 1150000, 1160000, 1170000, 1210000, 1270000,
};
static const unsigned int CCOVEP_V1P8A_VSEL_TABLE[] = {
	1635000, 1726000, 1799000, 1817000, 1835000, 1853000, 1908000, 1999000,
};
static const unsigned int CCOVEP_V5P0A_VSEL_TABLE[] = {
	4500000, 4750000, 5000000, 5048000, 5100000, 5100000, 5250000, 5500000,
};
static const unsigned int CCOVEP_V2P8SX_VSEL_TABLE[] = {
	2565000, 2700000, 2750000, 2800000, 2850000, 2900000, 3135000, 3300000,
};
static const unsigned int CCOVEP_VDDQ_VSEL_TABLE[] = {
	1080000, 1140000, 1200000, 1240000, 1350000, 1390000, 1418000, 1500000,
};

static int ccovep_regulator_enable(struct regulator_dev *rdev)
{
	struct ccovep_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_val;

	reg_val = intel_mid_pmic_readb(pmic_info->vol_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	return intel_mid_pmic_writeb(pmic_info->vol_reg,
			(reg_val | CCOVEP_REG_ENBL_MASK |
			 CCOVEP_REG_ENBL_CTRL_MASK));
}

static int ccovep_regulator_disable(struct regulator_dev *rdev)
{
	struct ccovep_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_val;

	reg_val = intel_mid_pmic_readb(pmic_info->vol_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	return intel_mid_pmic_writeb(pmic_info->vol_reg,
			((reg_val | CCOVEP_REG_ENBL_CTRL_MASK) &
			 CCOVEP_REG_DSBL_MASK));
}

static int ccovep_regulator_is_enabled(struct regulator_dev *rdev)
{
	struct ccovep_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_val;

	reg_val = intel_mid_pmic_readb(pmic_info->vol_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	if (!(reg_val & CCOVEP_REG_ENBL_CTRL_MASK))
		return -EINVAL;

	return reg_val & CCOVEP_REG_ENBL_MASK;
}

static int ccovep_regulator_get_voltage_sel(struct regulator_dev *rdev)
{
	struct ccovep_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_val, vsel;

	reg_val = intel_mid_pmic_readb(pmic_info->vol_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	vsel = (reg_val & CCOVEP_REG_VSEL_MASK) >> CCOVEP_REG_VSEL_SHIFT;

	return vsel;
}

static int ccovep_regulator_set_voltage_sel(struct regulator_dev *rdev,
		unsigned selector)
{
	struct ccovep_regulator_info *pmic_info = rdev_get_drvdata(rdev);
	int reg_val;

	reg_val = intel_mid_pmic_readb(pmic_info->vol_reg);
	if (reg_val < 0) {
		dev_err(&rdev->dev, "error reading pmic, %x\n", reg_val);
		return reg_val;
	}

	reg_val &= ~CCOVEP_REG_VSEL_MASK;
	reg_val |= selector << CCOVEP_REG_VSEL_SHIFT;

	return intel_mid_pmic_writeb(pmic_info->vol_reg, reg_val);
}

/* regulator ops */
static struct regulator_ops ccovep_regulator_ops = {
	.enable = ccovep_regulator_enable,
	.disable = ccovep_regulator_disable,
	.is_enabled = ccovep_regulator_is_enabled,
	.get_voltage_sel = ccovep_regulator_get_voltage_sel,
	.set_voltage_sel = ccovep_regulator_set_voltage_sel,
	.list_voltage = regulator_list_voltage_table,
};

/* Regulator descriptions */
static struct ccovep_regulator_info regulators_info[] = {
	{
		.desc = {
			.name = "v3p3a",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_V3P3A,
			.n_voltages = ARRAY_SIZE(CCOVEP_V3P3A_VSEL_TABLE),
			.volt_table = CCOVEP_V3P3A_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_V3P3A_CTRL,
	},
	{
		.desc = {
			.name = "v3p3sx",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_V3P3SX,
			.n_voltages = ARRAY_SIZE(CCOVEP_V3P3SX_VSEL_TABLE),
			.volt_table = CCOVEP_V3P3SX_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_V3P3SX_CTRL,
	},
	{
		.desc = {
			.name = "v1p05a",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_V1P05A,
			.n_voltages = ARRAY_SIZE(CCOVEP_V1P05A_VSEL_TABLE),
			.volt_table = CCOVEP_V1P05A_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_V1P05A_CTRL,
	},
	{
		.desc = {
			.name = "v1p15",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_V1P15,
			.n_voltages = ARRAY_SIZE(CCOVEP_V1P15_VSEL_TABLE),
			.volt_table = CCOVEP_V1P15_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_V1P15_CTRL,
	},
	{
		.desc = {
			.name = "v1p8a",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_V1P8A,
			.n_voltages = ARRAY_SIZE(CCOVEP_V1P8A_VSEL_TABLE),
			.volt_table = CCOVEP_V1P8A_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_V1P8A_CTRL,
	},
	{
		.desc = {
			.name = "v5p0a",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_V5P0A,
			.n_voltages = ARRAY_SIZE(CCOVEP_V5P0A_VSEL_TABLE),
			.volt_table = CCOVEP_V5P0A_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_V5P0A_CTRL,
	},
	{
		.desc = {
			.name = "v2p8sx",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_V2P8SX,
			.n_voltages = ARRAY_SIZE(CCOVEP_V2P8SX_VSEL_TABLE),
			.volt_table = CCOVEP_V2P8SX_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_V2P8SX_CTRL,
	},
	{
		.desc = {
			.name = "vddq",
			.ops = &ccovep_regulator_ops,
			.type = REGULATOR_VOLTAGE,
			.id = CCOVEP_ID_VDDQ,
			.n_voltages = ARRAY_SIZE(CCOVEP_VDDQ_VSEL_TABLE),
			.volt_table = CCOVEP_VDDQ_VSEL_TABLE,
			.owner = THIS_MODULE,
		},
		.vol_reg = CCOVEP_VDDQ_CTRL,
	},
};

static inline struct ccovep_regulator_info *ccovep_find_regulator_info(int id)
{
	struct ccovep_regulator_info *reg_info;
	int i;

	for (i = 0; i < ARRAY_SIZE(regulators_info); i++) {
		if (regulators_info[i].desc.id == id) {
			reg_info = &regulators_info[i];
			return reg_info;
		}
	}
	return NULL;
}


static int ccovep_regulator_probe(struct platform_device *pdev)
{
	struct ccovep_regulator_info *pdata = dev_get_platdata(&pdev->dev);
	struct regulator_config config = { };
	struct ccovep_regulator_info *reg_info = NULL;

	if (!pdata) {
		dev_err(&pdev->dev, "No regulator info\n");
		return -EINVAL;
	}

	reg_info = ccovep_find_regulator_info(pdev->id);
	if (reg_info == NULL) {
		dev_err(&pdev->dev, "invalid regulator %d\n", pdev->id);
		return -EINVAL;
	}

	config.dev = &pdev->dev;
	config.init_data = pdata->init_data;
	config.driver_data = reg_info;

	pdata->regulator = regulator_register(&reg_info->desc, &config);

	if (IS_ERR(pdata->regulator)) {
		dev_err(&pdev->dev, "failed to register regulator as %s\n",
				reg_info->desc.name);
		return PTR_ERR(pdata->regulator);
	}

	platform_set_drvdata(pdev, pdata->regulator);

	dev_dbg(&pdev->dev, "registered crystal cove plus regulator as %s\n",
			dev_name(&pdata->regulator->dev));

	return 0;
}

static int ccovep_regulator_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

static const struct platform_device_id ccovep_regulator_id_table[] = {
	{ "ccovep_regulator", 0},
	{ },
};
MODULE_DEVICE_TABLE(platform, ccovep_regulator_id_table);

static struct platform_driver ccovep_regulator_driver = {
	.driver = {
		.name = "ccovep_regulator",
		.owner = THIS_MODULE,
	},
	.probe = ccovep_regulator_probe,
	.remove = ccovep_regulator_remove,
	.id_table = ccovep_regulator_id_table,
};

static int __init ccovep_regulator_init(void)
{
	return platform_driver_register(&ccovep_regulator_driver);
}
fs_initcall(ccovep_regulator_init);

static void __exit ccovep_regulator_exit(void)
{
	platform_driver_unregister(&ccovep_regulator_driver);
}
module_exit(ccovep_regulator_exit);

MODULE_DESCRIPTION("CrystalCove Plus regulator driver");
MODULE_AUTHOR("Nitheesh K L <nitheesh.k.l@intel.com");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:intel_regulator");

