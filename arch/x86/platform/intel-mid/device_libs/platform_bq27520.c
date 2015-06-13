/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */

#include <linux/gpio.h>
#include <linux/i2c.h>
#include <linux/lnw_gpio.h>
#include <linux/power_supply.h>
#include <linux/power/bq27520_battery.h>
#include <linux/power/intel_mdf_battery.h>
#include <asm/intel-mid.h>
#include <asm/delay.h>
#include <asm/intel_scu_ipc.h>
#include "platform_bq27520.h"

static struct bq27520_platform_data bq27520_pdata = {
	.bat_id_gpio = -1,
	.low_bat = -1,
	.adc_alert = -1,
};

void *bq27520_platform_data(void *info)
{
#if defined(CONFIG_ME371MG)
	/* acquired gpio for battery id */
	bq27520_pdata.bat_id_gpio = get_gpio_by_name("bat_id");
#elif defined(CONFIG_ME372CL) || defined(CONFIG_PF450CL)
	bq27520_pdata.bat_id_gpio = get_gpio_by_name("BAT_ID");
#elif defined(CONFIG_PF450CL)
	bq27520_pdata.bat_id_gpio = get_gpio_by_name("BAT_ID");
#endif

	/* acquired gpio for battery low pin */
	bq27520_pdata.low_bat = get_gpio_by_name("P_BAT_LBO");

	/* acquired gpio for battery adc alert pin */
	bq27520_pdata.adc_alert = get_gpio_by_name("ADC_Alert");

	return &bq27520_pdata;
}

