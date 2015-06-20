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
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi.h>
#include <asm/intel_vlv2.h>
#include <linux/version.h>
#include "./pmic.h"

#define CRYSTAL_COVE_IRQ_NUM	7

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
static struct mfd_cell crystal_cove_dev[] = {
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
	},
	{
		.name = "reg-fixed-voltage",
		.id = 0,
		.num_resources = 0,
		.resources = NULL,
	},
	{
		.name = "gpio-regulator",
		.id = 0,
		.num_resources = 0,
		.resources = NULL,
	},
	{NULL, },
};

#define CRYSTAL_COVE_IRQREGMAP(irq) \
	[irq] = { \
		{MIRQLVL1, irq, 1, 0}, \
		{IRQLVL1,  irq, 1, 0}, \
		INTEL_PMIC_REG_NULL, \
	}

struct intel_pmic_irqregmap crystal_cove_irqregmap[] = {
	CRYSTAL_COVE_IRQREGMAP(PWRSRC_IRQ),
	CRYSTAL_COVE_IRQREGMAP(THRM_IRQ),
	CRYSTAL_COVE_IRQREGMAP(BCU_IRQ),
	CRYSTAL_COVE_IRQREGMAP(ADC_IRQ),
	CRYSTAL_COVE_IRQREGMAP(CHGR_IRQ),
	CRYSTAL_COVE_IRQREGMAP(GPIO_IRQ),
	CRYSTAL_COVE_IRQREGMAP(VHDMIOCP_IRQ),
};

static int crystal_cove_init(void)
{
	pr_info("Crystal Cove: ID 0x%02X, VERSION 0x%02X\n",
		intel_mid_pmic_readb(CHIPID), intel_mid_pmic_readb(CHIPVER));
	return 0;
}

struct intel_mid_pmic crystal_cove_pmic = {
	.label		= "crystal cove",
	.irq_flags	= IRQF_TRIGGER_RISING | IRQF_ONESHOT,
	.init		= crystal_cove_init,
	.cell_dev 	= crystal_cove_dev,
	.irq_regmap	= crystal_cove_irqregmap,
	.irq_num	= CRYSTAL_COVE_IRQ_NUM,
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");

