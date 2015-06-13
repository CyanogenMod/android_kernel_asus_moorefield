/*
 * Crystal cove page0  -- Device access for Crystal cove
 * PMIC page0 register or memory map.
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

/* No of times we should retry on -EAGAIN error */
#define NR_RETRY_CNT		3

static struct i2c_client *ccpage0;

static int ccpage0_write_reg8(struct i2c_client *client, u8 reg, u8 value)
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

static int ccpage0_read_reg8(struct i2c_client *client, u8 reg)
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

static void init_page0_registers(void)
{
	ccpage0_write_reg8(ccpage0, 0xf9, 0xad);
	ccpage0_write_reg8(ccpage0, 0xf9, 0xeb);
}

static int ccpage0_i2c_probe(struct i2c_client *client,
			    const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
		dev_err(&client->dev,
				"SM bus doesn't support BYTE transactions\n");
		return -EIO;
	}

	ccpage0 = client;
	init_page0_registers();

	return 0;
}

static int ccpage0_i2c_remove(struct i2c_client *i2c)
{
	return 0;
}

static int ccpage0_suspend(struct device *dev)
{
	return 0;
}

static int ccpage0_resume(struct device *dev)
{
	return 0;
}

static int ccpage0_runtime_suspend(struct device *dev)
{
	return 0;
}

static int ccpage0_runtime_resume(struct device *dev)
{
	return 0;
}

static int ccpage0_runtime_idle(struct device *dev)
{
	return 0;
}

static const struct dev_pm_ops ccpage0_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(ccpage0_suspend,
				ccpage0_resume)
		SET_RUNTIME_PM_OPS(ccpage0_runtime_suspend,
				ccpage0_runtime_resume,
				ccpage0_runtime_idle)
};

static const struct i2c_device_id ccpage0_i2c_id[] = {
	{ "crystalcove_page0", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, ccpage0_i2c_id);

static struct acpi_device_id pmic_acpi_match[] = {
	{ "TEST0002", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, pmic_acpi_match);

static struct i2c_driver ccpage0_i2c_driver = {
	.driver = {
		.name = "intel_crystalcove_page0",
		.owner = THIS_MODULE,
		.pm = &ccpage0_pm_ops,
		.acpi_match_table = ACPI_PTR(pmic_acpi_match),
	},
	.probe = ccpage0_i2c_probe,
	.remove = ccpage0_i2c_remove,
	.id_table = ccpage0_i2c_id,
};

static int __init ccpage0_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&ccpage0_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register pmic I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(ccpage0_i2c_init);

static void __exit ccpage0_i2c_exit(void)
{
	i2c_del_driver(&ccpage0_i2c_driver);
}
module_exit(ccpage0_i2c_exit);

MODULE_DESCRIPTION("Crystal Cove page0 device");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com");
