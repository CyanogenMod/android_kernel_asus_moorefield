/*
 * Whiskey Cove  --  Device access for Intel WhiskeyCove PMIC
 *
 * Copyright (c) 2013, Intel Corporation.
 *
 * Author: Yang Bin <bin.yang@intel.com>
 * Author: Kannappan <r.kannappan@intel.com>
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
#include <linux/regulator/intel_whiskey_cove_pmic.h>

#include <asm/intel_basincove_gpadc.h>
#include <linux/iio/iio.h>
#include <linux/iio/machine.h>
#include <linux/iio/types.h>

#define WHISKEY_COVE_IRQ_NUM	17

#define CHIPID		0x00
#define CHIPVER	0x01

#define IRQLVL1	0x02
#define PWRSRCIRQ	0x03
#define THRM0IRQ	0x04
#define THRM1IRQ	0x05
#define THRM2IRQ	0x06
#define THRM3IRQ	0xD9
#define CHGRIRQ	0x0A

#define MIRQLVL1	0x0E
#define MPWRSRCIRQ	0x0F
#define MTHRMIRQ0	0x0D
#define MTHRMIRQ1	0x12
#define MTHRMIRQ2	0x13
#define MTHRMIRQ3	0xDA
#define MCHGRIRQ	0x17

static struct gpadc_regmap_t whiskeycove_gpadc_regmaps[GPADC_NUM_CHANNELS] = {
	{"VBAT",	5,	0x4F03, 0x4F04, 0xFF, 0xFF, 0xFF, 0xFF},
	{"BATID",	4,	0x4F06, 0x4F07, 0xFF, 0xFF, 0xFF, 0xFF},
	{"PMICTEMP",	3,	0x4F42,	0x4F43, 0x4F33, 0x4F34, 0x4F33, 0x4F34},
	{"BATTEMP0",	2,	0x4F15, 0x4F16, 0xFF, 0xFF, 0xFF, 0xFF},
	{"BATTEMP1",	2,	0x4F17, 0x4F18, 0xFF, 0xFF, 0xFF, 0xFF},
	{"SYSTEMP0",	3,	0x4F38, 0x4F39, 0x4F23, 0x4F24, 0x4F25, 0x4F26},
	{"SYSTEMP1",	3,	0x4F3A, 0x4F3B, 0x4F27, 0x4F28, 0x4F29, 0x4F2A},
	{"SYSTEMP2",	3,	0x4F3C, 0x4F3D, 0x4F2B, 0x4F2C, 0x4F2D, 0x4F2E},
	{"USBID",	1,	0x4F08, 0x4F09, 0xFF, 0xFF, 0xFF, 0xFF},
	{"PEAK",	7,	0x4F13, 0x4F14, 0xFF, 0xFF, 0xFF, 0xFF},
	{"AGND",	6,	0x4F0A, 0x4F0B, 0xFF, 0xFF, 0xFF, 0xFF},
	{"VREF",	6,	0x4F0A, 0x4F0B, 0xFF, 0xFF, 0xFF, 0xFF},
};

static struct gpadc_regs_t whiskeycove_gpadc_regs = {
	.gpadcreq	=	0x4F02,
	.gpadcreq_irqen	=	0,
	.gpadcreq_busy	=	(1 << 0),
	.mirqlvl1	=	0x6e0E,
	.mirqlvl1_adc	=	(1 << 3),
	.adc1cntl	=	0x4F05,
	.adcirq		=	0x6E08,
	.madcirq	=	0x6E15,
};

#define MSIC_ADC_MAP(_adc_channel_label,			\
		     _consumer_dev_name,                        \
		     _consumer_channel)                         \
	{                                                       \
		.adc_channel_label = _adc_channel_label,        \
		.consumer_dev_name = _consumer_dev_name,        \
		.consumer_channel = _consumer_channel,          \
	}

static struct iio_map wc_iio_maps[] = {
	MSIC_ADC_MAP("CH0", "VIBAT", "VBAT"),
	MSIC_ADC_MAP("CH1", "BATID", "BATID"),
	MSIC_ADC_MAP("CH2", "PMICTEMP", "PMICTEMP"),
	MSIC_ADC_MAP("CH3", "BATTEMP", "BATTEMP0"),
	MSIC_ADC_MAP("CH4", "BATTEMP", "BATTEMP1"),
	MSIC_ADC_MAP("CH5", "SYSTEMP", "SYSTEMP0"),
	MSIC_ADC_MAP("CH6", "SYSTEMP", "SYSTEMP1"),
	MSIC_ADC_MAP("CH7", "SYSTEMP", "SYSTEMP2"),
	MSIC_ADC_MAP("CH8", "USBID", "USBID"),
	MSIC_ADC_MAP("CH9", "PEAK", "PEAK"),
	MSIC_ADC_MAP("CH10", "GPMEAS", "AGND"),
	MSIC_ADC_MAP("CH11", "GPMEAS", "VREF"),
	{ },
};

#define MSIC_ADC_CHANNEL(_type, _channel, _datasheet_name) \
	{                               \
		.indexed = 1,           \
		.type = _type,          \
		.channel = _channel,    \
		.datasheet_name = _datasheet_name,      \
	}

static const struct iio_chan_spec const wc_adc_channels[] = {
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 0, "CH0"),
	MSIC_ADC_CHANNEL(IIO_RESISTANCE, 1, "CH1"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 2, "CH2"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 3, "CH3"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 4, "CH4"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 5, "CH5"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 6, "CH6"),
	MSIC_ADC_CHANNEL(IIO_TEMP, 7, "CH7"),
	MSIC_ADC_CHANNEL(IIO_RESISTANCE, 8, "CH8"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 9, "CH9"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 10, "CH10"),
	MSIC_ADC_CHANNEL(IIO_VOLTAGE, 11, "CH11"),
};

enum {
	PWRSRC_LVL1 = 0,
	THRM_LVL1,
	BCU_IRQ,
	ADC_IRQ,
	CHGR_LVL1,
	GPIO_IRQ,
	CRIT_IRQ = 7,
	PWRSRC_IRQ,
	THRM1_IRQ,
	BATALRT_IRQ,
	BATZC_IRQ,
	CHGR_IRQ,
	THRM0_IRQ,
	PMICI2C_IRQ,
	THRM3_IRQ,
	CTYPE_IRQ,
};

struct intel_mid_pmic whiskey_cove_pmic;
static struct intel_basincove_gpadc_platform_data wc_adc_pdata;

static struct resource gpio_resources[] = {
	{
		.name	= "GPIO",
		.start	= GPIO_IRQ,
		.end	= GPIO_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource pmic_ccsm_resources[] = {
	{
		.start = PWRSRC_IRQ,
		.end   = PWRSRC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BATZC_IRQ,
		.end   = BATZC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BATALRT_IRQ,
		.end   = BATALRT_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CTYPE_IRQ,
		.end   = CTYPE_IRQ,
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

static struct resource charger_resources[] = {
	{
		.name  = "CHARGER",
		.start = CHGR_IRQ,
		.end   = CHGR_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource pmic_i2c_resources[] = {
	{
		.name  = "PMIC_I2C",
		.start = PMICI2C_IRQ,
		.end   = PMICI2C_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource thermal_resources[] = {
	{
		.start = THRM0_IRQ,
		.end   = THRM0_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = THRM1_IRQ,
		.end   = THRM1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = THRM3_IRQ,
		.end   = THRM3_IRQ,
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
static struct mfd_cell whiskey_cove_dev[] = {
	{
		.name = "whiskey_cove_adc",
		.id = 0,
		.num_resources = ARRAY_SIZE(adc_resources),
		.resources = adc_resources,
	},
	{
		.name = "whiskey_cove_thermal",
		.id = 0,
		.num_resources = ARRAY_SIZE(thermal_resources),
		.resources = thermal_resources,
	},
	{
		.name = "pmic_ccsm",
		.id = 0,
		.num_resources = ARRAY_SIZE(pmic_ccsm_resources),
		.resources = pmic_ccsm_resources,
	},
	{
		.name = "i2c_pmic_adap",
		.id = 0,
		.num_resources = ARRAY_SIZE(pmic_i2c_resources),
		.resources = pmic_i2c_resources,
	},
	{
		.name = "bd71621",
		.id = 0,
		.num_resources = ARRAY_SIZE(charger_resources),
		.resources = charger_resources,
	},
	{
		.name = "whiskey_cove_bcu",
		.id = 0,
		.num_resources = ARRAY_SIZE(bcu_resources),
		.resources = bcu_resources,
	},
	{
		.name = "whiskey_cove_gpio",
		.id = 0,
		.num_resources = ARRAY_SIZE(gpio_resources),
		.resources = gpio_resources,
	},
	{
		.name = "sw_fuel_gauge",
		.id = 0,
		.num_resources = 0,
		.resources = NULL,
	},
	{
		.name = "sw_fuel_gauge_ha",
		.id = 0,
		.num_resources = 0,
		.resources = NULL,
	},
	{
		.name = "wcove_regulator",
		.id = WCOVE_ID_V3P3SD + 1,
		.num_resources = 0,
		.resources = NULL,
	},
	{
		.name = "wcove_regulator",
		.id = WCOVE_ID_VSDIO + 1,
		.num_resources = 0,
		.resources = NULL,
	},
	{NULL, },
};

#define WHISKEY_COVE_IRQREGMAP(irq) \
	[irq] = { \
		{MIRQLVL1, irq, 1, 0}, \
		{IRQLVL1,  irq, 1, 0}, \
		INTEL_PMIC_REG_NULL, \
	}

struct intel_pmic_irqregmap whiskey_cove_irqregmap[] = {
	WHISKEY_COVE_IRQREGMAP(PWRSRC_LVL1),
	WHISKEY_COVE_IRQREGMAP(THRM_LVL1),
	WHISKEY_COVE_IRQREGMAP(BCU_IRQ),
	WHISKEY_COVE_IRQREGMAP(ADC_IRQ),
	WHISKEY_COVE_IRQREGMAP(CHGR_LVL1),
	WHISKEY_COVE_IRQREGMAP(GPIO_IRQ),
	WHISKEY_COVE_IRQREGMAP(CRIT_IRQ),
	{
		{MIRQLVL1, 0, 0x1, 0},
		{PWRSRCIRQ, 0, 0x1F, INTEL_PMIC_REG_W1C},
		{PWRSRCIRQ, 0, 0x1F, INTEL_PMIC_REG_W1C},
	},
	{ /* THERM1 IRQ */
		{MIRQLVL1, 1, 0x1, 0},
		{THRM1IRQ, 0, 0xF, INTEL_PMIC_REG_W1C},
		{THRM1IRQ, 0, 0xF, INTEL_PMIC_REG_W1C},
	},
	{ /* THERM2 */
		{MIRQLVL1, 1, 0x1, 0},
		{THRM2IRQ, 0, 0xC3, INTEL_PMIC_REG_W1C},
		{THRM2IRQ, 0, 0xC3, INTEL_PMIC_REG_W1C},
	},
	{ /* BATZONE CHANGED */
		{MIRQLVL1, 1, 0x1, 0},
		{THRM1IRQ, 7, 1, INTEL_PMIC_REG_W1C},
		{THRM1IRQ, 7, 1, INTEL_PMIC_REG_W1C},
	},
	{ /* Ext. Chrgr */
		{MIRQLVL1, 4, 0x1, 0},
		{CHGRIRQ, 0, 1, INTEL_PMIC_REG_W1C},
		{CHGRIRQ, 0, 1, INTEL_PMIC_REG_W1C},
	},
	{ /* THERM0 IRQ */
		{MIRQLVL1, 1, 0x1, 0},
		{THRM0IRQ, 0, 0xFF, INTEL_PMIC_REG_W1C},
		{THRM0IRQ, 0, 0xFF, INTEL_PMIC_REG_W1C},
	},
	{ /* External I2C Transaction */
		{MIRQLVL1, 4, 0x1, 0},
		{CHGRIRQ, 1, 7, INTEL_PMIC_REG_W1C},
		{CHGRIRQ, 1, 7, INTEL_PMIC_REG_W1C},
	},
	{ /* THERM3 */
		{MIRQLVL1, 1, 0x1, 0},
		{THRM3IRQ, 0, 0xF0, INTEL_PMIC_REG_W1C},
		{THRM3IRQ, 0, 0xF0, INTEL_PMIC_REG_W1C},
	},
	{ /* CTYP */
		{MIRQLVL1, 4, 0x1, 0},
		{CHGRIRQ, 4, 1, INTEL_PMIC_REG_W1C},
		{CHGRIRQ, 4, 1, INTEL_PMIC_REG_W1C},
	},
};

static void wc_set_adc_pdata(void)
{
	wc_adc_pdata.channel_num = GPADC_NUM_CHANNELS;
	wc_adc_pdata.intr_mask = MUSBID | MPEAK | MBATTEMP
		| MSYSTEMP | MBATT | MVIBATT | MGPMEAS | MCCTICK;
	wc_adc_pdata.gpadc_iio_maps = wc_iio_maps;
	wc_adc_pdata.gpadc_regmaps = whiskeycove_gpadc_regmaps;
	wc_adc_pdata.gpadc_regs = &whiskeycove_gpadc_regs;
	wc_adc_pdata.gpadc_channels = wc_adc_channels;

	intel_mid_pmic_set_pdata("whiskey_cove_adc", (void *)&wc_adc_pdata,
			sizeof(struct intel_basincove_gpadc_platform_data), 0);
}

static int whiskey_cove_init(void)
{
	pr_info("Whiskey Cove: ID 0x%02X, VERSION 0x%02X\n",
		intel_mid_pmic_readb(CHIPID), intel_mid_pmic_readb(CHIPVER));

	wc_set_adc_pdata();
	return 0;
}

struct intel_mid_pmic whiskey_cove_pmic = {
	.label		= "whiskey cove",
	.irq_flags	= IRQF_TRIGGER_HIGH | IRQF_ONESHOT,
	.init		= whiskey_cove_init,
	.cell_dev 	= whiskey_cove_dev,
	.irq_regmap	= whiskey_cove_irqregmap,
	.irq_num	= WHISKEY_COVE_IRQ_NUM,
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");

