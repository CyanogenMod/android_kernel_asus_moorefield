/*
 * Baytrail CR reverse boost(5V) control
 * -- Device access for reverse boost control
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/acpi.h>

#define BYT_CR_BCNTL_REG	0x5
#define BYT_CR_BCFG_REG		0xd
#define BOOST_EN_CNTL_BIT	(1 << 4)

/* No of times we should retry on -EAGAIN error */
#define NR_RETRY_CNT		3

static struct i2c_client *i2c_bcntl;

static int bcntl_write_reg8(struct i2c_client *client, u8 reg, u8 value)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_write_byte_data(client, reg, value);
		if (ret < 0)
			continue;
		else
			break;
	}
	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Write error:%d\n", ret);

	return ret;
}

static int bcntl_read_reg8(struct i2c_client *client, u8 reg)
{
	int ret, i;

	for (i = 0; i < NR_RETRY_CNT; i++) {
		ret = i2c_smbus_read_byte_data(client, reg);
		if (ret < 0)
			continue;
		else
			break;
	}
	if (ret < 0)
		dev_err(&client->dev, "I2C SMbus Read error:%d\n", ret);

	return ret;
}

int intel_bytcr_boost_enable(void)
{
	int ret, val;

	if (!i2c_bcntl)
		return -EAGAIN;

	ret = bcntl_read_reg8(i2c_bcntl, BYT_CR_BCNTL_REG);
	if (ret < 0)
		goto i2c_err;

	val = ret | BOOST_EN_CNTL_BIT;
	ret = bcntl_write_reg8(i2c_bcntl, BYT_CR_BCNTL_REG, val);

i2c_err:
	return ret;
}
EXPORT_SYMBOL(intel_bytcr_boost_enable);

int intel_bytcr_boost_disable(void)
{
	int ret, val;

	if (!i2c_bcntl)
		return -EAGAIN;

	ret = bcntl_read_reg8(i2c_bcntl, BYT_CR_BCNTL_REG);
	if (ret < 0)
		goto i2c_err;

	val = ret & ~BOOST_EN_CNTL_BIT;
	ret = bcntl_write_reg8(i2c_bcntl, BYT_CR_BCNTL_REG, val);

i2c_err:
	return ret;
}
EXPORT_SYMBOL(intel_bytcr_boost_disable);

static void init_boost_cntl_regs(void)
{
	int ret, val;

	/* configure the reverse boost control register */
	ret = bcntl_read_reg8(i2c_bcntl, BYT_CR_BCFG_REG);
	if (ret < 0)
		goto i2c_err;

	val = ret & ~BOOST_EN_CNTL_BIT;
	ret = bcntl_write_reg8(i2c_bcntl, BYT_CR_BCFG_REG, val);
	if (ret < 0)
		goto i2c_err;

	/* disable reverse boost during init */
	ret = intel_bytcr_boost_disable();
	if (ret < 0)
		goto i2c_err;

	return;
i2c_err:
	dev_err(&i2c_bcntl->dev, "bcntl init failed\n");
}

static int bcntl_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
				"SM bus doesn't support BYTE transactions\n");
		return -EIO;
	}

	i2c_bcntl = client;
	init_boost_cntl_regs();

	return 0;
}

static int bcntl_i2c_remove(struct i2c_client *i2c)
{
	return 0;
}

static void bcntl_shutdown(struct i2c_client *client)
{
	return;
}

static int bcntl_suspend(struct device *dev)
{
	return 0;
}

static int bcntl_resume(struct device *dev)
{
	return 0;
}

static int bcntl_runtime_suspend(struct device *dev)
{
	return 0;
}

static int bcntl_runtime_resume(struct device *dev)
{
	return 0;
}

static int bcntl_runtime_idle(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops bcntl_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(bcntl_suspend,
				bcntl_resume)
		SET_RUNTIME_PM_OPS(bcntl_runtime_suspend,
				bcntl_runtime_resume,
				bcntl_runtime_idle)
};

static const struct i2c_device_id bcntl_i2c_id[] = {
	{ "bytcr_bcntl", },
	{"INBC0000", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, bcntl_i2c_id);

static const struct acpi_device_id bcntl_acpi_id[] = {
	{"INBC0000", },
	{}
};
MODULE_DEVICE_TABLE(acpi, bcntl_acpi_id);

static struct i2c_driver bcntl_i2c_driver = {
	.driver = {
		.name = "intel_bytcr_bcntl",
		.owner = THIS_MODULE,
		.pm = &bcntl_pm_ops,
		.acpi_match_table = ACPI_PTR(bcntl_acpi_id),
	},
	.probe = bcntl_i2c_probe,
	.remove = bcntl_i2c_remove,
	.shutdown = bcntl_shutdown,
	.id_table = bcntl_i2c_id,
};

static int __init bcntl_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&bcntl_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register pmic I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(bcntl_i2c_init);

static void __exit bcntl_i2c_exit(void)
{
	i2c_del_driver(&bcntl_i2c_driver);
}
module_exit(bcntl_i2c_exit);

MODULE_DESCRIPTION("Baytrail CR Reverse Boost Control");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com");
