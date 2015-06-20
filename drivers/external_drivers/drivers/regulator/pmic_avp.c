/*
 * pmic_avp.c - Intel regulator Machine interface layer
 * Copyright (c) 2012, Intel Corporation.
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

#include <linux/version.h>	/*FIXME*/
#include <linux/bug.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/regulator/driver.h>
#include <linux/regulator/intel_pmic.h>
#include <linux/regulator/machine.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/mfd/intel_msic.h>

/* Intel Voltage cntrl register parameters*/
#define CTL_REG_MASK	0x07
#define AOACCTL_REG_MASK	0x38
#define REG_VSEL_MASK	0xC0
#define VSEL_SHIFT	6
#define REG_CTRL_MASK	0xF8

/**
*REGULATOR_MODE_FAST=REGULATOR_AUTO_MODE
*REGULATOR_MODE_NORMAL= REGULATOR_NORMAL_MODE
*REGULATOR_MODE_IDLE	= Not applicable on intel platforms
*REGULATOR_MODE_STANDBY=REGULATOR_LOW_POWER_MODE
*/
#define REGULATOR_NORMAL_MODE	0x07
#define REGULATOR_AUTO_MODE	0x06
#define REGULATOR_LOW_POWER_MODE	0x05
#define REGULATOR_OFF_MODE	0x04

#define MSIC_REG_ON	0x07
#define MSIC_REG_OFF	0x04

/* Intel regulator Error code */
#define INTEL_REGULATOR_ERR	-55

static int reg_debug;
#define MY_NAME "machine_interface"

#define pmic_dbg(format, arg...) \
	do {						\
		if (reg_debug)			\
			printk(KERN_INFO "%s: " format,	\
					MY_NAME , ## arg);		\
	} while (0)

#define pmic_err(format, arg...)	\
	printk(KERN_ERR "%s: " format,	\
			MY_NAME , ## arg)

const u16 reg_addr_offset[] = { VPROG1CNT_ADDR, VPROG2CNT_ADDR, VEMMC1CNT_ADDR,
						VEMMC2CNT_ADDR };
/**
* intel_pmic_reg_is_enabled - To check if the regulator is enabled
* @rdev:    regulator_dev structure
* @return value : 0 - Regulator is ON
*			  :1 - Regulator is OFF
*/
static int intel_pmic_reg_is_enabled(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8 reg, mode;
	int ret;

	pmic_dbg("This is is_enabled function %s\n", rdev->desc->name);
	ret = intel_msic_reg_read(pmic_info->pmic_reg, &reg);
	if (ret) {
		pmic_err("intel_msic_reg_read returns error %08x\n",
					ret);
		return INTEL_REGULATOR_ERR;
	}
	mode = reg & CTL_REG_MASK ;
	switch (mode) {
	case REGULATOR_NORMAL_MODE:
	case REGULATOR_AUTO_MODE:
	case REGULATOR_LOW_POWER_MODE:
		return 1;
	case REGULATOR_OFF_MODE:
		return 0;
	default:
		pmic_dbg("UNDEFINED MODE but returning OFF!!\n");
		return 0;
	}
}
/**
* intel_pmic_reg_enable - To enable the regulator
* @rdev:    regulator_dev structure
* @return value : 0 - Regulator enabling success
*			:1 - Regulator enabling failed
*/
static int intel_pmic_reg_enable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg;
	int ret;

	pmic_dbg("This is enable function for %s\n", rdev->desc->name);
	ret = intel_msic_reg_read(pmic_info->pmic_reg, &reg);
	if (ret) {
		pmic_err("intel_msic_reg_read returns"
					"error %08x\n", ret);
		return INTEL_REGULATOR_ERR;
	}
	return intel_msic_reg_write(pmic_info->pmic_reg,
				((reg & REG_CTRL_MASK) | MSIC_REG_ON));
}
/**
* intel_pmic_reg_disable - To disable the regulator
* @rdev:    regulator_dev structure
* @return value :0 - Regulator disabling success
*			:1 - Regulator disabling failed
*/
static int intel_pmic_reg_disable(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg;
	int ret;

	pmic_dbg("This is intel_pmic_reg_disable function for %s\n",
				rdev->desc->name);
	ret = intel_msic_reg_read(pmic_info->pmic_reg, &reg);
	if (ret) {
		pmic_err("intel_msic_reg_read returns"
					"error %08x\n", ret);
		return INTEL_REGULATOR_ERR;
	}
	return intel_msic_reg_write(pmic_info->pmic_reg,
		((reg & REG_CTRL_MASK) | MSIC_REG_OFF));
}
/**
* intel_pmic_reg_listvoltage - Return the voltage value,this is called
*                                   from core framework
* @rdev: regulator source
* @index : passed on from core
* @return value : Returns the voltage value.
 */
static int intel_pmic_reg_listvoltage(struct regulator_dev *rdev,
								unsigned index)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);

	if (index >= pmic_info->table_len) {
		pmic_err("Index out of range in listvoltage\n");
		return INTEL_REGULATOR_ERR;
	}
	return pmic_info->table[index] * 1000;
}
/**
* intel_pmic_reg_getvoltage - Return the current voltage value in  uV
* @rdev:    regulator_dev structure
*  @return value : Returns the voltage value.
*/
static int intel_pmic_reg_getvoltage(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	u8  reg, vsel;
	int ret;

	pmic_dbg("intel_pmic_reg_getvoltage function for %s\n",
				rdev->desc->name);
	ret = intel_msic_reg_read(pmic_info->pmic_reg, &reg);
	if (ret) {
		pmic_err("intel_msic_reg_read returns"
					"error %08x\n", ret);
		return INTEL_REGULATOR_ERR;
	}
	vsel = (reg & REG_VSEL_MASK) >> VSEL_SHIFT;
	if (vsel >= pmic_info->table_len) {
		pmic_err("vsel value is out of range\n");
		return INTEL_REGULATOR_ERR;
	}
	pmic_dbg("Voltage value is %d mV\n",
	pmic_info->table[vsel]);
	return pmic_info->table[vsel] * 1000;
}

/**
* intel_pmic_reg_setvoltage - Set voltage to the regulator
* @rdev:    regulator_dev structure
* @min_uV: Minimum required voltage in uV
* @max_uV: Maximum acceptable voltage in uV
* @selector: Voltage value passed back to core layer
* Sets a voltage regulator to the desired output voltage
* @return value : Returns 0 if success
*			: Return INTEL_REGULATOR_ERR
*/
static int intel_pmic_reg_setvoltage(struct regulator_dev *rdev, int min_uV,
					int max_uV, unsigned *selector)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int ret;
	u8 reg, vsel;

	pmic_dbg("This is intel_pmic_reg_setvoltage function for %s\n",
				rdev->desc->name);
	for (vsel = 0; vsel < pmic_info->table_len; vsel++) {
		int mV = pmic_info->table[vsel];
		int uV = mV * 1000;
		if (min_uV <= uV && uV <= max_uV) {
			*selector = vsel;
			ret = intel_msic_reg_read(pmic_info->pmic_reg, &reg);
			if (ret) {
				pmic_err("intel_msic_reg_read returns"
							"error %08x\n", ret);
				return INTEL_REGULATOR_ERR;
			}
			reg &= ~REG_VSEL_MASK;
			reg |= vsel << VSEL_SHIFT;
			pmic_dbg("intel_pmic_reg_setvoltage voltage is"
						"val %xuV\n", reg);
			return intel_msic_reg_write(pmic_info->pmic_reg,
						reg);
		}
	}
	return INTEL_REGULATOR_ERR;
}
/**
* intel_pmic_reg_getmode - Get the regulator mode
* @rdev:    regulator_dev structure
* Get the regulator mode
* @return value   : Returns  mode
*				: Return INTEL_REGULATOR_ERR
 */
static unsigned int intel_pmic_reg_getmode(struct regulator_dev *rdev)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int ret;
	u8 reg, mode;

	pmic_dbg("This is intel_pmic_reg_getmode function for %s\n",
				rdev->desc->name);
	ret = intel_msic_reg_read(pmic_info->pmic_reg, &reg);
	if (ret) {
		pmic_err("intel_msic_reg_read returns error %08x\n",
				ret);
		return INTEL_REGULATOR_ERR;
	}
	mode = (reg & CTL_REG_MASK);
	switch (mode) {
	case REGULATOR_NORMAL_MODE:
		return REGULATOR_MODE_NORMAL;
	case REGULATOR_AUTO_MODE:
		return REGULATOR_MODE_FAST;
	case REGULATOR_LOW_POWER_MODE:
		return REGULATOR_MODE_STANDBY;
	case REGULATOR_OFF_MODE:
		return REGULATOR_STATUS_OFF;
	default:
		pmic_dbg("Mode is not a valid one MODE %d", mode);
	}
	return INTEL_REGULATOR_ERR;
}
/**
* intel_pmic_reg_setmode - Set the regulator mode
* @rdev:    regulator_dev structure
* Set the regulator mode
* @return value:	Returns  0 if success
			:	Return INTEL_REGULATOR_ERR
*/
static int intel_pmic_reg_setmode(struct regulator_dev *rdev, unsigned mode)
{
	struct intel_pmic_info *pmic_info = rdev_get_drvdata(rdev);
	int ret;
	u8 reg, avp_mode;

	pmic_dbg("This is intel_pmic_reg_setmode function for %s\n",
				rdev->desc->name);
	ret = intel_msic_reg_read(pmic_info->pmic_reg, &reg);
	if (ret) {
		pmic_err("intel_msic_reg_read returns error %08x\n",
					ret);
		return INTEL_REGULATOR_ERR;
	}
	reg &= ~CTL_REG_MASK;
	switch (mode) {
	case REGULATOR_MODE_NORMAL:
		avp_mode = REGULATOR_NORMAL_MODE;
		break;
	case REGULATOR_MODE_FAST:
		avp_mode = REGULATOR_AUTO_MODE;
		break;
	case REGULATOR_MODE_IDLE:
	case REGULATOR_MODE_STANDBY:
		avp_mode = REGULATOR_LOW_POWER_MODE;
		break;
	case REGULATOR_STATUS_OFF:
		avp_mode = REGULATOR_OFF_MODE;
		break;
	default:
		pmic_dbg("Mode is not a valid one MODE %d", mode);
		return INTEL_REGULATOR_ERR;
	}
	pmic_dbg("intel_pmic_reg_setmode is %d\n", avp_mode);
	return intel_msic_reg_write(pmic_info->pmic_reg, (reg|avp_mode));
}
/* regulator_ops registration */
static struct regulator_ops intel_pmic_ops = {
	.is_enabled = intel_pmic_reg_is_enabled,
	.enable = intel_pmic_reg_enable,
	.disable = intel_pmic_reg_disable,
	.set_mode = intel_pmic_reg_setmode,
	.get_mode = intel_pmic_reg_getmode,
	.get_voltage = intel_pmic_reg_getvoltage,
	.set_voltage = intel_pmic_reg_setvoltage,
	.list_voltage = intel_pmic_reg_listvoltage,
};
/**
* struct regulator_desc - Regulator descriptor
* Each regulator registered with the core is described with a structure of
* this type.
* @name: Identifying name for the regulator.
* @id: Numerical identifier for the regulator.
* @n_voltages: Number of selectors available for ops.list_voltage().
* @ops: Regulator operations table.
* @irq: Interrupt number for the regulator.
* @type: Indicates if the regulator is a voltage or current regulator.
* @owner: Module providing the regulator, used for refcounting.
*/
static struct regulator_desc intel_pmic_desc[] = {
	{
		.name = "vprog1",
		.id = VPROG1,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG1_VSEL_table),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vprog2",
		.id = VPROG2,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VPROG2_VSEL_table),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vemmc1",
		.id = VEMMC1,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VEMMC1_VSEL_table),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
	{
		.name = "vemmc2",
		.id = VEMMC2,
		.ops = &intel_pmic_ops,
		.n_voltages = ARRAY_SIZE(VEMMC2_VSEL_table),
		.type = REGULATOR_VOLTAGE,
		.owner = THIS_MODULE,
	},
};

static int avp_pmic_probe(struct platform_device *pdev)
{
	struct intel_pmic_info *pdata = dev_get_platdata(&pdev->dev);
	int i;

	struct intel_pmic_info *intel_pmic;
	if (!pdata || !pdata->pmic_reg)
		return -EINVAL;
	intel_pmic = pdev->dev.platform_data;
	if (!intel_pmic)
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(reg_addr_offset); i++) {
		if (reg_addr_offset[i] == pdata->pmic_reg)
			break;
	}
	if (i < (ARRAY_SIZE(reg_addr_offset))) {
		platform_set_drvdata(pdev, intel_pmic->intel_pmic_rdev);
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 5, 0))
		{
		struct regulator_config config = { 0 };
		config.dev = &pdev->dev;
		config.init_data = pdata->init_data;
		config.driver_data = pdata;
		intel_pmic->intel_pmic_rdev =
			regulator_register(&intel_pmic_desc[i], &config);
		}
#else
		intel_pmic->intel_pmic_rdev =
			regulator_register(&intel_pmic_desc[i],
					&pdev->dev, pdata->init_data, pdata,
					pdev->dev.of_node);
#endif
		if (IS_ERR(intel_pmic->intel_pmic_rdev)) {
			pmic_err("can't register regulator..error %ld\n",
					PTR_ERR(intel_pmic->intel_pmic_rdev));
			return PTR_ERR(intel_pmic->intel_pmic_rdev);
		}
		dev_dbg(&pdev->dev, "registered regulator\n");
		return 0;
	}
	return INTEL_REGULATOR_ERR;
}

static int avp_pmic_remove(struct platform_device *pdev)
{
	regulator_unregister(platform_get_drvdata(pdev));
	return 0;
}

static const struct platform_device_id avp_id_table[] = {
	{ "intel_regulator", 0 },
	{ },
};

MODULE_DEVICE_TABLE(platform, avp_id_table);

static struct platform_driver avp_pmic_driver = {
	.driver		= {
		.name = "intel_regulator",
		.owner = THIS_MODULE,
	},
	.probe = avp_pmic_probe,
	.remove = avp_pmic_remove,
	.id_table = avp_id_table,
};
static int __init avp_pmic_init(void)
{
	return platform_driver_register(&avp_pmic_driver);
}
subsys_initcall(avp_pmic_init);

static void __exit avp_pmic_exit(void)
{
	platform_driver_unregister(&avp_pmic_driver);
}
module_exit(avp_pmic_exit);

MODULE_DESCRIPTION("AVP voltage regulator driver");
MODULE_AUTHOR("Vishwesh/Mahesh");
MODULE_LICENSE("GPL v2");
