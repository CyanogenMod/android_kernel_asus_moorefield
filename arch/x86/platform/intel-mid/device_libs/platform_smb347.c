/*
 * platform_smb347.c: smb347 platform data initilization file
 *
 * (C) Copyright 2008 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/gpio.h>
#include <asm/intel-mid.h>
#include <linux/lnw_gpio.h>
#include <linux/power/smb347-charger.h>
#include <asm/intel-mid.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include "platform_smb347.h"
#include <linux/power_supply.h>
#include <linux/power/battery_id.h>
#include <linux/iio/consumer.h>
#include <asm/intel_em_config.h>
#if defined(CONFIG_ME372CG_BATTERY_SMB345)
#include "platform_me372cg_smb345.h"
#endif
#define BYT_FFD8_PR1_BATID_LL 0x2D0
#define BYT_FFD8_PR1_BATID_UL 0x2F0

#if defined(CONFIG_ME372CG_BATTERY_SMB345)
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


	.irq_gpio			= -1,
	.inok_gpio			= -1,

	.mains_current_limit	= 1200,

	.usb_hc_current_limit	= 500,
	.twins_h_current_limit	= 1800,
};
#endif

/* Redridge DV2.1 */
static struct smb347_charger_platform_data smb347_rr_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= true,
	.is_valid_battery		= true,
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.irq_gpio			= SMB347_IRQ_GPIO,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0xFC,
						0x01, 0x95,
						0x02, 0x83,
						0x03, 0xE3,
						0x04, 0x3A,
						0x05, 0x1A,
						0x06, 0x65,
						0x07, 0xEF,
						0x08, 0x09,
						0x09, 0xDF,
						0x0A, 0xAB,
						0x0B, 0x5A,
						0x0C, 0xC1,
						0x0D, 0x46,
						0xFF, 0xFF
					},
};

/* Salitpa EV 0.5 */
static struct smb347_charger_platform_data smb347_ev05_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= true,
	.is_valid_battery		= true,
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_DISABLED,
	.irq_gpio			= SMB347_IRQ_GPIO,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0xA1,
						0x01, 0x6C,
						0x02, 0x93,
						0x03, 0xE5,
						0x04, 0x3E,
						0x05, 0x16,
						0x06, 0x0C,
						0x07, 0x8c,
						0x08, 0x08,
						0x09, 0x0c,
						0x0A, 0xA4,
						0x0B, 0x13,
						0x0C, 0x81,
						0x0D, 0x02,
						0x0E, 0x20,
						0x10, 0x7F,
						0xFF, 0xFF
					},
};

/* Salitpa EV 1.0 */
static struct smb347_charger_platform_data smb347_ev10_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 3700000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= true,
	.is_valid_battery		= true,
	.enable_control			= SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	.otg_control			= SMB347_OTG_CONTROL_DISABLED,
	.irq_gpio			= SMB347_IRQ_GPIO,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0xAA,
						0x01, 0x6C,
						0x02, 0x93,
						0x03, 0xE5,
						0x04, 0x3E,
						0x05, 0x16,
						0x06, 0x74,
						0x07, 0xCC,
						0x09, 0x0C,
						0x0A, 0xA4,
						0x0B, 0x13,
						0x0C, 0x8D,
						0x0D, 0x00,
						0x0E, 0x20,
						0x10, 0x7F,
						0xFF, 0xFF
					},
};

/* baytrail ffrd8 */
static struct smb347_charger_platform_data byt_t_ffrd8_pr0_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 4350000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= false,
	.is_valid_battery		= true,
	.use_usb			= true,
	.enable_control			= SMB347_CHG_ENABLE_SW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0x46,
						0x01, 0x65,
					/* disable opticharge*/
						0x02, 0x83,
						0x03, 0xED,
						0x04, 0xB8,
						0x05, 0x05,
						0x06, 0x14,
						0x07, 0x95,
						0x09, 0x00,
						0x0A, 0xC7,
						0x0B, 0x95,
						0x0C, 0x8F,
						0x0D, 0xB4,
						0x0E, 0x60,
						0x10, 0x40,
			/* disable suspend as charging didnot start*/
						0x30, 0x42,  /*orig:0x46*/
						0x31, 0x00,
						0xFF, 0xFF
					},
};

/* baytrail ffrd8 */
static struct smb347_charger_platform_data byt_t_ffrd8_pr1_pdata = {
	.battery_info	= {
		.name			= "UP110005",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 4350000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 6894000,
	},
	.use_mains			= false,
	.is_valid_battery		= false,
	.use_usb			= true,
	.enable_control			= SMB347_CHG_ENABLE_SW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0x46,
						0x01, 0x65,
					/* disable opticharge*/
						0x02, 0x87,
						0x03, 0xED,
			/* enable Auto recharge, Disable Turbo charge+ */
						0x04, 0x38,
					/* USB-OK stat, disable safety timer */
						0x05, 0x4F,
				/* enable APSD interrupt along with others */
						0x06, 0x06,
						0x07, 0x85,
					/* I2C control OTG */
						0x9, 0x80,
				/* Switch Freq. 1.5MHz OTG current  500mA*/
						0x0A, 0x87,
					/* OTG interrupt settings */
						0x0C, 0xBF,
						0x0D, 0xF4,
						0x10, 0x40,
						0x31, 0x01,
						0xFF, 0xFF
					},
};

/* baytrail CR RVP */
static struct smb347_charger_platform_data byt_t_cr_crb_pdata = {
	.battery_info	= {
		.name			= "AT4188A2",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 4350000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 4900000,
	},
	.use_mains			= false,
	.is_valid_battery		= true,
	.use_usb			= true,
	.enable_control			= SMB347_CHG_ENABLE_SW,
	.otg_control			= SMB347_OTG_CONTROL_DISABLED,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0x46,
						0x01, 0x65,
						0x02, 0x83,
						0x03, 0xED,
						0x04, 0xB8,
						0x05, 0x25,
						0x06, 0x1E,
						0x07, 0x90,
						0x09, 0x00,
						0x0A, 0xC7,
						0x0B, 0x95,
						0x0C, 0x8F,
						0x0D, 0xB4,
						0x0E, 0x60,
						0x10, 0x40,
						0x30, 0x42,
						0x31, 0x00,
						0xFF, 0xFF
					},
};

/* Cherrytrail RVP/FFRD -T */
static struct smb347_charger_platform_data cht_t_pdata = {
	.battery_info	= {
		.name			= "AT4188A2",
		.technology		= POWER_SUPPLY_TECHNOLOGY_LIPO,
		.voltage_max_design	= 4350000,
		.voltage_min_design	= 3000000,
		.charge_full_design	= 4900000,
	},
	.use_mains			= false,
	.is_valid_battery		= true,
	.use_usb			= true,
	.enable_control			= SMB347_CHG_ENABLE_SW,
	.otg_control			= SMB347_OTG_CONTROL_SW,
	.char_config_regs		= {
						/* Reg  Value */
						0x00, 0x46,
						0x01, 0x65,
						0x02, 0x87,
						0x03, 0xED,
						0x04, 0x38,
						0x05, 0x05,
						0x06, 0x06,
						0x07, 0xA5,
						0x09, 0x8F,
						0x0A, 0x87,
						0x0B, 0x95,
						0x0C, 0xBF,
						0x0D, 0xF4,
						0x0E, 0xA0,
						0x10, 0x66,
						0x31, 0x01,
						0xFF, 0xFF
					},
};

#ifdef CONFIG_POWER_SUPPLY_CHARGER
#define SMB347_CHRG_CUR_NOLIMIT		1800
#define SMB347_CHRG_CUR_MEDIUM		1400
#define SMB347_CHRG_CUR_LOW		1000

static struct ps_batt_chg_prof ps_batt_chrg_prof;
static struct ps_pse_mod_prof batt_chg_profile;
static struct power_supply_throttle smb347_byt_throttle_states[] = {
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = SMB347_CHRG_CUR_NOLIMIT,
	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = SMB347_CHRG_CUR_MEDIUM,
	},
	{
		.throttle_action = PSY_THROTTLE_CC_LIMIT,
		.throttle_val = SMB347_CHRG_CUR_LOW,
	},
	{
		.throttle_action = PSY_THROTTLE_DISABLE_CHARGING,
	},
};

static char *smb347_supplied_to[] = {
	"max170xx_battery",
	"max17042_battery",
	"max17047_battery",
};

bool smb347_is_valid_batid(void)
{
	int val = 0;
	bool is_valid = false;
	struct iio_channel *indio_chan = NULL;

	/* Get batid pmic channel */
	indio_chan = iio_channel_get(NULL, "BATID");
	if (IS_ERR_OR_NULL(indio_chan)) {
		pr_err("platform_smb347: IIO channel get error!!\n");
		return is_valid;
	}

	/* Read pmic batid ADC */
	if (iio_read_channel_raw(indio_chan, &val) >= 0) {
		pr_info("platform_smb347: IIO channel read Sucess,"
				 " val=%.3X\n", val);
		/* Due to internal capacitor 0.1uF across batid resistor,
		 * the pmic batid adc count is not in expected range,
		 * between 0x40 to 0x60. This is workaround to check
		 * the valid batid with actual battery.
		 */
		if ((val <= 0x60)
			|| (val > BYT_FFD8_PR1_BATID_LL
			&& val < BYT_FFD8_PR1_BATID_UL))
			is_valid = true;
	} else {
		pr_info("platform_smb347: unable to read batid");
	}
	iio_channel_release(indio_chan);
	return is_valid;
}
EXPORT_SYMBOL(smb347_is_valid_batid);

static void *platform_get_batt_charge_profile(struct smb347_charger_platform_data *pdata)
{
	struct ps_temp_chg_table temp_mon_range[BATT_TEMP_NR_RNG];

	char batt_str[] = "INTN0001";

	if (pdata->is_valid_battery) {
		/*
		 * WA: hard coding the profile
		 * till we get OEM0 table from FW.
		 */
		memcpy(batt_chg_profile.batt_id, batt_str, strlen(batt_str));

		batt_chg_profile.battery_type = 0x2;
		batt_chg_profile.capacity = 0x2C52;
		batt_chg_profile.voltage_max = 4350;
		batt_chg_profile.chrg_term_ma = 300;
		batt_chg_profile.low_batt_mV = 3400;
		batt_chg_profile.disch_tmp_ul = 55;
		batt_chg_profile.disch_tmp_ll = 0;
		batt_chg_profile.temp_mon_ranges = 5;

		temp_mon_range[0].temp_up_lim = 55;
		temp_mon_range[0].full_chrg_vol = 4100;
		temp_mon_range[0].full_chrg_cur = 1800;
		temp_mon_range[0].maint_chrg_vol_ll = 4050;
		temp_mon_range[0].maint_chrg_vol_ul = 4100;
		temp_mon_range[0].maint_chrg_cur = 1800;

		temp_mon_range[1].temp_up_lim = 45;
		temp_mon_range[1].full_chrg_vol = 4350;
		temp_mon_range[1].full_chrg_cur = 1800;
		temp_mon_range[1].maint_chrg_vol_ll = 4300;
		temp_mon_range[1].maint_chrg_vol_ul = 4350;
		temp_mon_range[1].maint_chrg_cur = 1800;

		temp_mon_range[2].temp_up_lim = 23;
		temp_mon_range[2].full_chrg_vol = 4350;
		temp_mon_range[2].full_chrg_cur = 1400;
		temp_mon_range[2].maint_chrg_vol_ll = 4300;
		temp_mon_range[2].maint_chrg_vol_ul = 4350;
		temp_mon_range[2].maint_chrg_cur = 1400;

		temp_mon_range[3].temp_up_lim = 10;
		temp_mon_range[3].full_chrg_vol = 4350;
		temp_mon_range[3].full_chrg_cur = 1000;
		temp_mon_range[3].maint_chrg_vol_ll = 4300;
		temp_mon_range[3].maint_chrg_vol_ul = 4350;
		temp_mon_range[3].maint_chrg_cur = 1000;

		temp_mon_range[4].temp_up_lim = 0;
		temp_mon_range[4].full_chrg_vol = 0;
		temp_mon_range[4].full_chrg_cur = 0;
		temp_mon_range[4].maint_chrg_vol_ll = 0;
		temp_mon_range[4].maint_chrg_vol_ul = 0;
		temp_mon_range[4].maint_chrg_vol_ul = 0;
		temp_mon_range[4].maint_chrg_cur = 0;

		memcpy(batt_chg_profile.temp_mon_range,
			temp_mon_range,
			BATT_TEMP_NR_RNG * sizeof(struct ps_temp_chg_table));

		batt_chg_profile.temp_low_lim = 0;
		ps_batt_chrg_prof.chrg_prof_type = PSE_MOD_CHRG_PROF;
	} else {
		memset(&batt_chg_profile, 0, sizeof(struct ps_pse_mod_prof));
		ps_batt_chrg_prof.chrg_prof_type = CHRG_PROF_NONE;
	}
	ps_batt_chrg_prof.batt_prof = &batt_chg_profile;
	battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED, &ps_batt_chrg_prof);
	return &ps_batt_chrg_prof;
}

static void platform_init_chrg_params(
	struct smb347_charger_platform_data *pdata)
{
	pdata->throttle_states = smb347_byt_throttle_states;
	pdata->supplied_to = smb347_supplied_to;
	pdata->num_throttle_states = ARRAY_SIZE(smb347_byt_throttle_states);
	pdata->num_supplicants = ARRAY_SIZE(smb347_supplied_to);
	pdata->supported_cables = POWER_SUPPLY_CHARGER_TYPE_USB;
	if (INTEL_MID_BOARD(1, TABLET, BYT))
		pdata->chg_profile = (struct ps_batt_chg_prof *)
			platform_get_batt_charge_profile(pdata);
	else if (INTEL_MID_BOARD(1, TABLET, CHT)) {
		if (!em_config_get_charge_profile(&batt_chg_profile))
			ps_batt_chrg_prof.chrg_prof_type = CHRG_PROF_NONE;
		else
			ps_batt_chrg_prof.chrg_prof_type = PSE_MOD_CHRG_PROF;

		ps_batt_chrg_prof.batt_prof = &batt_chg_profile;
		battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED,
					&ps_batt_chrg_prof);
		pdata->chg_profile = (struct ps_batt_chg_prof *)
					&ps_batt_chrg_prof;
	}
}
#else
static void platform_init_chrg_params(
	struct smb347_charger_platform_data *pdata)
{
}
static void *platform_get_batt_charge_profile(struct smb347_charger_platform_data *pdata)
{
}
bool smb347_is_valid_batid(void)
{
	return true;
}
EXPORT_SYMBOL(smb347_is_valid_batid);
#endif

static void *get_platform_data(void)
{
	/* Redridge all */
	if (INTEL_MID_BOARD(2, TABLET, MFLD, RR, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, RR, PRO)) {
		return &smb347_rr_pdata;
	} else if (INTEL_MID_BOARD(2, TABLET, MFLD, SLP, ENG) ||
		INTEL_MID_BOARD(2, TABLET, MFLD, SLP, PRO)) {
		/* Salitpa */
		/* EV 0.5 */
		if (SPID_HARDWARE_ID(MFLD, TABLET, SLP, EV05))
			return &smb347_ev05_pdata;
		/* EV 1.0 and later */
		else
			return &smb347_ev10_pdata;
	} else if (INTEL_MID_BOARD(1, TABLET, BYT)) {
		if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR0) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR0)) {
			byt_t_ffrd8_pr0_pdata.detect_chg = false;
			byt_t_ffrd8_pr0_pdata.gpio_mux = -1;
			platform_init_chrg_params(&byt_t_ffrd8_pr0_pdata);
			return &byt_t_ffrd8_pr0_pdata;
		} else if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, 8PR1) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, 8PR1)) {
			byt_t_ffrd8_pr1_pdata.detect_chg = true;
			byt_t_ffrd8_pr1_pdata.use_regulator = true;
			byt_t_ffrd8_pr1_pdata.gpio_mux = 131; /* GPIO_SUS1*/
			/* configure output */
			lnw_gpio_set_alt(byt_t_ffrd8_pr1_pdata.gpio_mux, 0);
			gpio_request(byt_t_ffrd8_pr1_pdata.gpio_mux,
								"gpio_mux");
			byt_t_ffrd8_pr1_pdata.is_valid_battery = smb347_is_valid_batid();
			platform_init_chrg_params(&byt_t_ffrd8_pr1_pdata);
			return &byt_t_ffrd8_pr1_pdata;
		} else if (INTEL_MID_BOARD(3, TABLET, BYT, BLK, PRO, CRV2) ||
			INTEL_MID_BOARD(3, TABLET, BYT, BLK, ENG, CRV2)) {
			byt_t_cr_crb_pdata.detect_chg = false;
			byt_t_cr_crb_pdata.gpio_mux = -1;
			platform_init_chrg_params(&byt_t_cr_crb_pdata);
			return &byt_t_cr_crb_pdata;
		}
	} else if (INTEL_MID_BOARD(1, TABLET, CHT)) {
		pr_info("%s: CHT charger ...", __func__);
		cht_t_pdata.detect_chg = true;
		cht_t_pdata.gpio_mux = -1;
		cht_t_pdata.is_valid_battery = true;
		platform_init_chrg_params(&cht_t_pdata);
		return &cht_t_pdata;
	}
	return NULL;
}

void *smb347_platform_data(void *info)
{
#if defined(CONFIG_ME372CG_BATTERY_SMB345)
	smb345_pdata.inok_gpio = 44;
	return &smb345_pdata;
#else
	return get_platform_data();
#endif
}
