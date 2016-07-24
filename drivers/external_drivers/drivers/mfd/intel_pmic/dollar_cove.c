/*
 * Dollar Cove  --  Device access for Intel PMIC for CR
 *
 * Copyright (c) 2014, Intel Corporation.
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
#include <linux/gpio.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/acpi.h>
#include <asm/intel_vlv2.h>
#include <linux/lnw_gpio.h>
#include <linux/version.h>
#include <asm/dc_xpwr_pwrsrc.h>
#include <linux/power/dc_xpwr_battery.h>
#include <linux/power/dc_xpwr_charger.h>
#include <linux/regulator/intel_dcovex_regulator.h>
#include <asm/intel_em_config.h>
#include <linux/power/battery_id.h>
#include "./pmic.h"
#include <asm/spid.h>

enum {
	VBUS_FALLING_IRQ = 2,
	VBUS_RISING_IRQ,
	VBUS_OV_IRQ,
	VBUS_FALLING_ALT_IRQ,
	VBUS_RISING_ALT_IRQ,
	VBUS_OV_ALT_IRQ,

	CHARGE_DONE_IRQ = 10,
	CHARGE_CHARGING_IRQ,
	BAT_SAFE_QUIT_IRQ,
	BAT_SAFE_ENTER_IRQ,
	BAT_ABSENT_IRQ,
	BAT_APPEND_IRQ,

	QWBTU_IRQ = 16,
	WBTU_IRQ,
	QWBTO_IRQ,
	WBTO_IRQ,
	QCBTU_IRQ,
	CBTU_IRQ,
	QCBTO_IRQ,
	CBTO_IRQ,

	WL2_IRQ = 24,
	WL1_IRQ,
	GPADC_IRQ,
	OT_IRQ = 31,

	GPIO0_IRQ = 32,
	GPIO1_IRQ,
	POKO_IRQ,
	POKL_IRQ,
	POKS_IRQ,
	POKN_IRQ,
	POKP_IRQ,
	EVENT_IRQ,

	MV_CHNG_IRQ = 40,
	BC_USB_CHNG_IRQ,
};

static struct resource power_button_resources[] = {
	{
		.start	= POKN_IRQ,
		.end	= POKN_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
	{
		.start	= POKP_IRQ,
		.end	= POKP_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};


static struct resource gpio_resources[] = {
	{
		.start	= GPIO0_IRQ,
		.end	= GPIO1_IRQ,
		.flags	= IORESOURCE_IRQ,
	},
};

static struct resource adc_resources[] = {
	{
		.start = GPADC_IRQ,
		.end   = GPADC_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource pwrsrc_resources[] = {
	{
		.start = VBUS_FALLING_IRQ,
		.end   = VBUS_FALLING_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = VBUS_RISING_IRQ,
		.end   = VBUS_RISING_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = MV_CHNG_IRQ,
		.end   = MV_CHNG_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BC_USB_CHNG_IRQ,
		.end   = BC_USB_CHNG_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource charger_resources[] = {
	{
		.start = VBUS_OV_IRQ,
		.end   = VBUS_OV_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CHARGE_DONE_IRQ,
		.end   = CHARGE_DONE_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CHARGE_CHARGING_IRQ,
		.end   = CHARGE_CHARGING_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BAT_SAFE_QUIT_IRQ,
		.end   = BAT_SAFE_QUIT_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = BAT_SAFE_ENTER_IRQ,
		.end   = BAT_SAFE_ENTER_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = QCBTU_IRQ,
		.end   = QCBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CBTU_IRQ,
		.end   = CBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = QCBTO_IRQ,
		.end   = QCBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = CBTO_IRQ,
		.end   = CBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct resource battery_resources[] = {
	{
		.start = QWBTU_IRQ,
		.end   = QWBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WBTU_IRQ,
		.end   = WBTU_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = QWBTO_IRQ,
		.end   = QWBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WBTO_IRQ,
		.end   = WBTO_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WL2_IRQ,
		.end   = WL2_IRQ,
		.flags = IORESOURCE_IRQ,
	},
	{
		.start = WL1_IRQ,
		.end   = WL1_IRQ,
		.flags = IORESOURCE_IRQ,
	},
};

static struct mfd_cell dollar_cove_dev[] = {
	{
		.name = "dollar_cove_adc",
		.id = 0,
		.num_resources = ARRAY_SIZE(adc_resources),
		.resources = adc_resources,
	},
	{
		.name = "dollar_cove_gpio",
		.id = 0,
		.num_resources = ARRAY_SIZE(gpio_resources),
		.resources = gpio_resources,
	},
	{
		.name = "dollar_cove_power_button",
		.id = 0,
		.num_resources = ARRAY_SIZE(power_button_resources),
		.resources = power_button_resources,
	},
	{
		.name = "dollar_cove_pwrsrc",
		.id = 0,
		.num_resources = ARRAY_SIZE(pwrsrc_resources),
		.resources = pwrsrc_resources,
	},
	{
		.name = "dollar_cove_charger",
		.id = 0,
		.num_resources = ARRAY_SIZE(charger_resources),
		.resources = charger_resources,
	},
	{
		.name = "dollar_cove_battery",
		.id = 0,
		.num_resources = ARRAY_SIZE(battery_resources),
		.resources = battery_resources,
	},
	{
		.name = "dcovex_regulator",
		.id = DCOVEX_ID_LDO2 + 1,
		.num_resources = 0,
		.resources = NULL,
	},
	{
		.name = "dcovex_regulator",
		.id = DCOVEX_ID_GPIO1 + 1,
		.num_resources = 0,
		.resources = NULL,
	},
	{NULL, },
};

#define DOLLAR_COVE_IRQREGMAP(irq) \
	[irq] = { \
		{(0x40 + (irq / 8)), (irq % 8), 1, INTEL_PMIC_REG_INV},\
		{(0x48 + (irq / 8)), (irq % 8), 1, INTEL_PMIC_REG_W1C},\
		{(0x48 + (irq / 8)), (irq % 8), 1, INTEL_PMIC_REG_W1C},\
	}

struct intel_pmic_irqregmap dollar_cove_irqregmap[] = {
	DOLLAR_COVE_IRQREGMAP(VBUS_FALLING_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_RISING_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_OV_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_FALLING_ALT_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_RISING_ALT_IRQ),
	DOLLAR_COVE_IRQREGMAP(VBUS_OV_ALT_IRQ),
	DOLLAR_COVE_IRQREGMAP(CHARGE_DONE_IRQ),
	DOLLAR_COVE_IRQREGMAP(CHARGE_CHARGING_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_SAFE_QUIT_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_SAFE_ENTER_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_ABSENT_IRQ),
	DOLLAR_COVE_IRQREGMAP(BAT_APPEND_IRQ),
	DOLLAR_COVE_IRQREGMAP(QWBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(WBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(QWBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(WBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(QCBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(CBTU_IRQ),
	DOLLAR_COVE_IRQREGMAP(QCBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(CBTO_IRQ),
	DOLLAR_COVE_IRQREGMAP(WL2_IRQ),
	DOLLAR_COVE_IRQREGMAP(WL1_IRQ),
	DOLLAR_COVE_IRQREGMAP(GPADC_IRQ),
	DOLLAR_COVE_IRQREGMAP(OT_IRQ),
	DOLLAR_COVE_IRQREGMAP(GPIO0_IRQ),
	DOLLAR_COVE_IRQREGMAP(GPIO1_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKO_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKL_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKS_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKN_IRQ),
	DOLLAR_COVE_IRQREGMAP(POKP_IRQ),
	DOLLAR_COVE_IRQREGMAP(EVENT_IRQ),
	DOLLAR_COVE_IRQREGMAP(MV_CHNG_IRQ),
	DOLLAR_COVE_IRQREGMAP(BC_USB_CHNG_IRQ),
};




#ifdef CONFIG_POWER_SUPPLY_CHARGER
static struct ps_pse_mod_prof atl_820_batt = {
	"ATL820",
	512,
	800,
	4350,
	80,
	3000,
	60,
	-20,
	4,
	{
		{60, 0, 0, 0, 0, 0},
		{45, 4350, 600, 4250, 4350, 20},
		{23, 4350, 240, 4250, 4350, 20},
		{10, 4350, 80, 4250, 4350, 20},
	},
	0,
};

#define DC_CHRG_CHRG_CUR_NOLIMIT	1800
#define DC_CHRG_CHRG_CUR_MEDIUM		1400
#define DC_CHRG_CHRG_CUR_LOW		1000

static struct ps_batt_chg_prof ps_batt_chrg_prof;
static struct ps_pse_mod_prof batt_chg_profile;
static struct power_supply_throttle dc_chrg_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = DC_CHRG_CHRG_CUR_NOLIMIT,
	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = DC_CHRG_CHRG_CUR_MEDIUM,
	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = DC_CHRG_CHRG_CUR_LOW,
	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGING,
	},
};

static char *dc_chrg_supplied_to[] = {
	"dollar_cove_battery"
};

static void *platform_get_batt_charge_profile(void)
{
#ifdef CONFIG_BTNS_PMIC
	int retval = 0;
	retval = get_batt_prop(&ps_batt_chrg_prof);
	if (retval) {
		pr_err("Error reading battery profile from battid framework. Use hard coded value.\n");
		memcpy(&batt_chg_profile, &atl_820_batt, sizeof(struct ps_pse_mod_prof));
		ps_batt_chrg_prof.batt_prof = &batt_chg_profile;
		ps_batt_chrg_prof.chrg_prof_type = PSE_MOD_CHRG_PROF;
	}
#if defined(CONFIG_POWER_SUPPLY_CHARGING_ALGO_STEP) && defined(CONFIG_POWER_SUPPLY_BATTID)
	ps_batt_chrg_prof.chrg_prof_type = STEP_MOD_CHRG_PROF;
#endif
#else
	if (!em_config_get_charge_profile(&batt_chg_profile))
		ps_batt_chrg_prof.chrg_prof_type = CHRG_PROF_NONE;
	else
		ps_batt_chrg_prof.chrg_prof_type = PSE_MOD_CHRG_PROF;

	ps_batt_chrg_prof.batt_prof = &batt_chg_profile;
#endif
	battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED, &ps_batt_chrg_prof);
	return &ps_batt_chrg_prof;
}

static void platform_init_chrg_params(struct dollarcove_chrg_pdata *pdata)
{
	pdata->throttle_states = dc_chrg_throttle_states;
	pdata->supplied_to = dc_chrg_supplied_to;
	pdata->num_throttle_states = ARRAY_SIZE(dc_chrg_throttle_states);
	pdata->num_supplicants = ARRAY_SIZE(dc_chrg_supplied_to);
	pdata->supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB;
	pdata->chg_profile = (struct ps_batt_chg_prof *)
			platform_get_batt_charge_profile();
}
#endif

static void dc_xpwr_chrg_pdata(void)
{
	static struct dollarcove_chrg_pdata pdata;
	int ret;

#ifdef CONFIG_BTNS_PMIC
	pdata.max_cc = 600;
	pdata.max_cv = 4350;
	pdata.def_cc = 500;
	pdata.def_cv = 4350;
	pdata.def_ilim = 900;
	pdata.def_iterm = 300;
	pdata.def_max_temp = 45;
	pdata.def_min_temp = 0;
	pdata.otg_gpio = -1;
#else
	pdata.max_cc = 2000;
	pdata.max_cv = 4350;
	pdata.def_cc = 500;
	pdata.def_cv = 4350;
	pdata.def_ilim = 900;
	pdata.def_iterm = 300;
	pdata.def_max_temp = 55;
	pdata.def_min_temp = 0;

	pdata.otg_gpio = 117; /* GPIONC_15 */
	/* configure output */
	ret = gpio_request(pdata.otg_gpio, "otg_gpio");
	if (ret) {
		pr_err("unable to request GPIO pin\n");
		pdata.otg_gpio = -1;
	} else {
		lnw_gpio_set_alt(pdata.otg_gpio, 0);
	}
#endif
	platform_init_chrg_params(&pdata);

	intel_mid_pmic_set_pdata("dollar_cove_charger",
				(void *)&pdata, sizeof(pdata), 0);
}

static void dc_xpwr_fg_pdata(void)
{
	static struct dollarcove_fg_pdata pdata;
	struct em_config_oem0_data data;
	int i;
#ifdef CONFIG_BTNS_PMIC
	snprintf(pdata.battid, (BATTID_LEN + 1),
			"%s", "INTN0001");
	pdata.technology = POWER_SUPPLY_TECHNOLOGY_LION;

	pdata.design_cap = 728;
	pdata.design_min_volt = 3550;
	pdata.design_max_volt = 4350;
	pdata.max_temp = 45;
	pdata.min_temp = 0;
#else
	if (em_config_get_oem0_data(&data)) {
		snprintf(pdata.battid, (BATTID_LEN + 1),
				"%s", "INTN0001");
		pdata.technology = POWER_SUPPLY_TECHNOLOGY_LION;
	} else {
		snprintf(pdata.battid, (BATTID_LEN + 1),
				"%s", "UNKNOWNB");
		pdata.technology = POWER_SUPPLY_TECHNOLOGY_UNKNOWN;
	}

	pdata.design_cap = 4980;
	pdata.design_min_volt = 3400;
	pdata.design_max_volt = 4350;
	pdata.max_temp = 55;
	pdata.min_temp = 0;
#endif
	intel_mid_pmic_set_pdata("dollar_cove_battery",
				(void *)&pdata, sizeof(pdata), 0);
}

static void dc_xpwr_pwrsrc_pdata(void)
{
	static struct dc_xpwr_pwrsrc_pdata pdata;

#if defined(CONFIG_MRD8) || defined(CONFIG_MRD7P05)
	int ret;

	pdata.mux_gpio = 131; /* GPIO_S5[1] */
	ret = gpio_request(pdata.mux_gpio, "otg_gpio");
	if (ret) {
		pr_err("unable to request GPIO pin\n");
		pdata.mux_gpio = -1;
	} else {
		lnw_gpio_set_alt(pdata.mux_gpio, 0);
	}
	/*
	 * set en_chrg_det to true if the
	 * D+/D- lines are connected to
	 * PMIC itself.
	 */
	pdata.en_chrg_det = true;
#else
	pdata.en_chrg_det = false;
#endif

	intel_mid_pmic_set_pdata("dollar_cove_pwrsrc",
				(void *)&pdata, sizeof(pdata), 0);
}

static int dollar_cove_init(void)
{
	pr_info("Dollar Cove: IC_TYPE 0x%02X\n", intel_mid_pmic_readb(0x03));
	dc_xpwr_pwrsrc_pdata();
	dc_xpwr_fg_pdata();
	dc_xpwr_chrg_pdata();
	return 0;
}

struct intel_mid_pmic dollar_cove_pmic = {
	.label		= "dollar cove",
	.irq_flags	= IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
	.init		= dollar_cove_init,
	.cell_dev 	= dollar_cove_dev,
	.irq_regmap	= dollar_cove_irqregmap,
	.irq_num	= 48,
};

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Yang Bin <bin.yang@intel.com");

