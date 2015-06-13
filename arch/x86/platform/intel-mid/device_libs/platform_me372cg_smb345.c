/*
 * smb345 platform data initilization file
 *
 */

#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <linux/power/smb345-me372cg-charger.h>
#include <asm/intel-mid.h>
#include "platform_me372cg_smb345.h"
#if 1
static struct smb345_charger_platform_data smb345_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.max_charge_current		= 3360000,
	.max_charge_voltage		= 4200000,
	.otg_uvlo_voltage		= 3300000,
	.chip_temp_threshold		= 120,
	.soft_cold_temp_limit		= 5,
	.soft_hot_temp_limit		= 50,
	.hard_cold_temp_limit		= 5,
	.hard_hot_temp_limit		= 55,
	.suspend_on_hard_temp_limit	= true,
	.soft_temp_limit_compensation	= SMB347_SOFT_TEMP_COMPENSATE_CURRENT
	| SMB347_SOFT_TEMP_COMPENSATE_VOLTAGE,
	.charge_current_compensation	= 900000,
	.use_mains			= true,
#if 0
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.irq_gpio			= SMB347_IRQ_GPIO,
#endif
#if defined(CONFIG_PF400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_A400CG)
	.gp_sdio_2_clk		= -1,
#endif
	.irq_gpio			= -1,
	.inok_gpio			= -1,
#ifdef CONFIG_ME302C
	.mains_current_limit	= 1800,
#else
	.mains_current_limit	= 1200,
#endif
	.usb_hc_current_limit	= 500,
	.twins_h_current_limit	= 1800,
};

void *smb345_platform_data(void *info)
{
	smb345_pdata.inok_gpio = 44;
#if defined(CONFIG_PF400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_A400CG) || defined(CONFIG_PF450CL)
	smb345_pdata.gp_sdio_2_clk = 62 + 96;
#endif
	return &smb345_pdata;
}
#endif
