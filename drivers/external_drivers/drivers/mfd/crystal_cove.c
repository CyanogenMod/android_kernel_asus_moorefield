/*
 * Crystal Cove  --  Device access for Intel PMIC for VLV2
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * Author: Yang Bin <bin.yang@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/delay.h>
#include <linux/mfd/core.h>
#include <linux/err.h>
#include <linux/i2c.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/workqueue.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi.h>
#include <asm/intel_vlv2.h>
#include <linux/version.h>

#define PMIC_IRQ_NUM	7

#define CHIPID		0x00
#define CHIPVER		0x01
#define IRQLVL1		0x02
#define MIRQLVL1	0x0E
enum {
	PWRSRC_IRQ = 0,
	THRM_IRQ,
	BCU_IRQ,
	ADC_IRQ,
	CHGR_IRQ,
	GPIO_IRQ,
	VHDMIOCP_IRQ
};

struct intel_mid_pmic {
	struct i2c_client *i2c;
	struct mutex io_lock;
	struct device *dev;
	int irq;
	struct mutex irq_lock;
	int irq_base;
	unsigned long irq_mask;
	struct workqueue_struct *workqueue;
	struct work_struct      work;
};

static struct device *gpio_dev;
static struct resource gpio_resources[] = {
	{
		.name	= "GPIO",
		.start	= GPIO_IRQ,
		.end	= GPIO_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource pwrsrc_resources[] = {
	{
		.name  = "PWRSRC",
		.start = PWRSRC_IRQ,
		.end   = PWRSRC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource adc_resources[] = {
	{
		.name  = "ADC",
		.start = ADC_IRQ,
		.end   = ADC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource thermal_resources[] = {
	{
		.name  = "THERMAL",
		.start = THRM_IRQ,
		.end   = THRM_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
static struct resource bcu_resources[] = {
	{
		.name  = "BCU",
		.start = BCU_IRQ,
		.end   = BCU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};
static struct mfd_cell crystal_cove_data[] = {
	{
		.name = "crystal_cove_pwrsrc",
		.id = 0,
		.num_resources = ARRAY_SIZE(pwrsrc_resources),
		.resources = pwrsrc_resources,
	},
	{
		.name = "crystal_cove_adc",
		.id = 0,
		.num_resources = ARRAY_SIZE(adc_resources),
		.resources = adc_resources,
	},
	{
		.name = "crystal_cove_thermal",
		.id = 0,
		.num_resources = ARRAY_SIZE(thermal_resources),
		.resources = thermal_resources,
	},
	{
		.name = "crystal_cove_bcu",
		.id = 0,
		.num_resources = ARRAY_SIZE(bcu_resources),
		.resources = bcu_resources,
	},
	{
		.name = "crystal_cove_gpio",
		.id = 0,
		.num_resources = ARRAY_SIZE(gpio_resources),
		.resources = gpio_resources,
		.platform_data = &gpio_dev,
		.pdata_size = sizeof(gpio_dev),
	},
	{NULL, },
};

int intel_mid_pmic_set_pdata(const char *name, void *data, int len)
{
	int i;
	struct mfd_cell *cell;

	for (i = 0; i < ARRAY_SIZE(crystal_cove_data); i++) {
		cell = &crystal_cove_data[i];
		if (!strcmp(cell->name, name)) {
			cell->platform_data = data;
			cell->pdata_size = len;
			return 0;
		}
	}
	return -EINVAL;
}
EXPORT_SYMBOL(intel_mid_pmic_set_pdata);

/* wrapper function needed by Baytrail BCU driver */
int intel_scu_ipc_read_mip(u8 *data, int len, int offset, int issigned)
{
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_read_mip);

static struct intel_mid_pmic intel_mid_pmic;
static struct intel_mid_pmic *pmic = &intel_mid_pmic;

/* These intel_scu_ipc_* APIs are formed to
 * be compatible with old SCU IPC APIs.
 */
int intel_scu_ipc_ioread8(u16 addr, u8 *data)
{
	int ret;

	ret = intel_mid_pmic_readb(addr);
	if (ret < 0)
		return ret;

	*data = ret;
	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_ioread8);

int intel_scu_ipc_iowrite8(u16 addr, u8 data)
{
	return intel_mid_pmic_writeb(addr, data);
}
EXPORT_SYMBOL(intel_scu_ipc_iowrite8);

int intel_scu_ipc_update_register(u16 addr, u8 data, u8 mask)
{
	int ret;

	if (!pmic->i2c)
		return -ENODEV;

	mutex_lock(&pmic->io_lock);

	ret = i2c_smbus_read_byte_data(pmic->i2c, addr);
	if (ret < 0)
		goto err;

	data &= mask;
	ret &= ~mask;
	ret |= data;

	ret = i2c_smbus_write_byte_data(pmic->i2c, addr, ret);

err:
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_scu_ipc_update_register);

int intel_scu_ipc_readv(u16 *addr, u8 *data, int len)
{
	int i;
	int ret;

	if (len < 1 || len > 4)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		ret = intel_scu_ipc_ioread8(addr[i], &data[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_readv);

int intel_scu_ipc_writev(u16 *addr, u8 *data, int len)
{
	int i;
	int ret;

	if (len < 1 || len > 4)
		return -EINVAL;

	for (i = 0; i < len; i++) {
		ret = intel_scu_ipc_iowrite8(addr[i], data[i]);
		if (ret)
			return ret;
	}

	return 0;
}
EXPORT_SYMBOL(intel_scu_ipc_writev);

int intel_mid_pmic_readb(int reg)
{
	int ret;

	if (!pmic->i2c)
		return -ENODEV;

	mutex_lock(&pmic->io_lock);
	ret = i2c_smbus_read_byte_data(pmic->i2c, reg);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_readb);

int intel_mid_pmic_writeb(int reg, u8 val)
{
	int ret;

	if (!pmic->i2c)
		return -ENODEV;

	mutex_lock(&pmic->io_lock);
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}
EXPORT_SYMBOL(intel_mid_pmic_writeb);

int intel_mid_pmic_setb(int reg, u8 mask)
{
	int ret;
	int val;

	if (!pmic->i2c)
		return -ENODEV;

	mutex_lock(&pmic->io_lock);
	val = i2c_smbus_read_byte_data(pmic->i2c, reg);
	val |= mask;
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

int intel_mid_pmic_clearb(int reg, u8 mask)
{
	int ret;
	int val;

	if (!pmic->i2c)
		return -ENODEV;

	mutex_lock(&pmic->io_lock);
	val = i2c_smbus_read_byte_data(pmic->i2c, reg);
	val &= ~mask;
	ret = i2c_smbus_write_byte_data(pmic->i2c, reg, val);
	mutex_unlock(&pmic->io_lock);
	return ret;
}

static void pmic_irq_enable(struct irq_data *data)
{
	clear_bit(data->irq - pmic->irq_base, &pmic->irq_mask);
	queue_work(pmic->workqueue, &pmic->work);
}

static void pmic_irq_disable(struct irq_data *data)
{
	set_bit(data->irq - pmic->irq_base, &pmic->irq_mask);
	queue_work(pmic->workqueue, &pmic->work);
}

static void pmic_irq_sync_unlock(struct irq_data *data)
{
	mutex_unlock(&pmic->irq_lock);
}

static void pmic_irq_lock(struct irq_data *data)
{
	mutex_lock(&pmic->irq_lock);
}

static void pmic_work(struct work_struct *work)
{
	mutex_lock(&pmic->irq_lock);
	intel_mid_pmic_writeb(MIRQLVL1, (u8)pmic->irq_mask);
	mutex_unlock(&pmic->irq_lock);
}

static irqreturn_t pmic_irq_isr(int irq, void *data)
{
	return IRQ_WAKE_THREAD;
}

static irqreturn_t pmic_irq_thread(int irq, void *data)
{
	int i;
	int pending;

	mutex_lock(&pmic->irq_lock);
	intel_mid_pmic_writeb(MIRQLVL1, (u8)pmic->irq_mask);
	pending = intel_mid_pmic_readb(IRQLVL1) & (~pmic->irq_mask);
	for (i = 0; i < PMIC_IRQ_NUM; i++)
		if (pending & (1 << i))
			handle_nested_irq(pmic->irq_base + i);
	mutex_unlock(&pmic->irq_lock);
	return IRQ_HANDLED;
}

static struct irq_chip pmic_irq_chip = {
	.name			= "intel_mid_pmic",
	.irq_bus_lock		= pmic_irq_lock,
	.irq_bus_sync_unlock	= pmic_irq_sync_unlock,
	.irq_disable		= pmic_irq_disable,
	.irq_enable		= pmic_irq_enable,
};

static void pmic_shutdown(struct i2c_client *client)
{
	dev_dbg(&client->dev, "%s called\n", __func__);

	if (pmic->irq > 0)
		disable_irq(pmic->irq);

	return;
}

static int pmic_suspend(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);

	if (pmic->irq > 0)
		disable_irq(pmic->irq);

	return 0;
}

static int pmic_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);

	if (pmic->irq > 0)
		enable_irq(pmic->irq);

	return 0;
}

static const struct dev_pm_ops pmic_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(pmic_suspend,
				pmic_resume)
};

static int pmic_irq_init(void)
{
	int cur_irq;
	int ret;

	pmic->irq_mask = 0xff;
	ret = intel_mid_pmic_writeb(MIRQLVL1, pmic->irq_mask);
	if (ret) {
		dev_err(pmic->dev, "Failed to communicate with PMIC.");
		return ret;
	}
	pmic->irq_mask = intel_mid_pmic_readb(MIRQLVL1);
	pmic->irq_base = irq_alloc_descs(VV_PMIC_IRQBASE, 0, PMIC_IRQ_NUM, 0);
	if (pmic->irq_base < 0) {
		dev_warn(pmic->dev, "Failed to allocate IRQs: %d\n",
			 pmic->irq_base);
		pmic->irq_base = 0;
		return -EINVAL;
	}

	/* Register them with genirq */
	for (cur_irq = pmic->irq_base;
	     cur_irq < PMIC_IRQ_NUM + pmic->irq_base;
	     cur_irq++) {
		irq_set_chip_data(cur_irq, pmic);
		irq_set_chip_and_handler(cur_irq, &pmic_irq_chip,
					 handle_edge_irq);
		irq_set_nested_thread(cur_irq, 1);
		irq_set_noprobe(cur_irq);
	}

	ret = request_threaded_irq(pmic->irq, pmic_irq_isr, pmic_irq_thread,
			IRQF_TRIGGER_RISING | IRQF_ONESHOT,
			"intel_mid_pmic", pmic);
	if (ret != 0) {
		dev_err(pmic->dev, "Failed to request IRQ %d: %d\n",
				pmic->irq, ret);
		return ret;
	}
	ret = enable_irq_wake(pmic->irq);
	if (ret != 0) {
		dev_warn(pmic->dev, "Can't enable PMIC IRQ as wake source: %d\n",
			 ret);
	}

	return 0;
}

static int pmic_i2c_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
	int i, ret;
	struct mfd_cell *cell_dev = crystal_cove_data;

	mutex_init(&pmic->io_lock);
	mutex_init(&pmic->irq_lock);
	pmic->workqueue =
		create_singlethread_workqueue("crystal cove");
	INIT_WORK(&pmic->work, pmic_work);
	gpio_dev = &i2c->dev;
	pmic->i2c = i2c;
	pmic->dev = &i2c->dev;
	pmic->irq = i2c->irq;
	ret = pmic_irq_init();
	if (ret)
		return ret;

	dev_info(&i2c->dev, "Crystal Cove: ID 0x%02X, VERSION 0x%02X\n",
		intel_mid_pmic_readb(CHIPID), intel_mid_pmic_readb(CHIPVER));
	for (i = 0; cell_dev[i].name != NULL; i++)
		;
#if (LINUX_VERSION_CODE >= KERNEL_VERSION(3, 10, 1))
	return mfd_add_devices(pmic->dev, -1, cell_dev, i,
			NULL, pmic->irq_base, NULL);
#else
	return mfd_add_devices(pmic->dev, -1, cell_dev, i,
			NULL, pmic->irq_base);
#endif
}

static int pmic_i2c_remove(struct i2c_client *i2c)
{
	mfd_remove_devices(pmic->dev);
	return 0;
}

static const struct i2c_device_id pmic_i2c_id[] = {
	{ "crystal_cove", },
	{ "INT33FD", },
	{ }
};
MODULE_DEVICE_TABLE(i2c, pmic_i2c_id);

static struct acpi_device_id pmic_acpi_match[] = {
	{ "TEST0001", 0 },
	{ "INT33FD", 0 },
	{ },
};
MODULE_DEVICE_TABLE(acpi, pmic_acpi_match);

static struct i2c_driver pmic_i2c_driver = {
	.driver = {
		.name = "intel_mid_i2c_pmic",
		.owner = THIS_MODULE,
		.pm = &pmic_pm_ops,
		.acpi_match_table = ACPI_PTR(pmic_acpi_match),
	},
	.probe = pmic_i2c_probe,
	.remove = pmic_i2c_remove,
	.id_table = pmic_i2c_id,
	.shutdown = pmic_shutdown,
};

static int __init pmic_i2c_init(void)
{
	int ret;

	ret = i2c_add_driver(&pmic_i2c_driver);
	if (ret != 0)
		pr_err("Failed to register pmic I2C driver: %d\n", ret);

	return ret;
}
subsys_initcall(pmic_i2c_init);

static void __exit pmic_i2c_exit(void)
{
	i2c_del_driver(&pmic_i2c_driver);
}
module_exit(pmic_i2c_exit);

MODULE_DESCRIPTION("Crystal Cove support for ValleyView2 PMIC");
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");


