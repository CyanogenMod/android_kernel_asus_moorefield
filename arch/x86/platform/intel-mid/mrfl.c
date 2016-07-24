/*
 * mrfl.c: Intel Merrifield platform specific setup code
 *
 * (C) Copyright 2012 Intel Corporation
 * Author: Mark F. Brown <mark.f.brown@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/module.h>
#include <linux/power_supply.h>
#include <linux/power/battery_id.h>
#include <asm/setup.h>
#include <asm/intel-mid.h>
#include <asm/processor.h>

#define APIC_DIVISOR 16
#define MRFL_I2_TERM_MA 120

#define SMIP_SRAM_ADDRESS	0xFFFC5C00
#define SMIP_EM_MISCFLAGS	0x071B
#define BYPASSINVALID		(1 << 4)
#define SMIP_FPO1_OFFSET	0x023E
#define CHARGE_INVALID_BATTERY	(1 << 2)
#define MOFD_VBAT_MIN		3450

enum intel_mid_sim_type __intel_mid_sim_platform;
EXPORT_SYMBOL_GPL(__intel_mid_sim_platform);

static void (*intel_mid_timer_init)(void);
static struct ps_pse_mod_prof *battery_chrg_profile;
static struct ps_batt_chg_prof *ps_batt_chrg_profile;

static void tangier_arch_setup(void);

/* tangier arch ops */
static struct intel_mid_ops tangier_ops = {
	.arch_setup = tangier_arch_setup,
};

static unsigned long __init tangier_calibrate_tsc(void)
{
	/* [REVERT ME] fast timer calibration method to be defined */
	if ((intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_VP) ||
	    (intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_HVP)) {
		lapic_timer_frequency = 50000;
		return 1000000;
	}

	if ((intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_SLE) ||
	    (intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_NONE)) {

		unsigned long fast_calibrate;
		u32 lo, hi, ratio, fsb, bus_freq;

		/* *********************** */
		/* Compute TSC:Ratio * FSB */
		/* *********************** */

		/* Compute Ratio */
		rdmsr(MSR_PLATFORM_INFO, lo, hi);
		pr_debug("IA32 PLATFORM_INFO is 0x%x : %x\n", hi, lo);

		ratio = (lo >> 8) & 0xFF;
		pr_debug("ratio is %d\n", ratio);
		if (!ratio) {
			pr_err("Read a zero ratio, force tsc ratio to 4 ...\n");
			ratio = 4;
		}

		/* Compute FSB */
		rdmsr(MSR_FSB_FREQ, lo, hi);
		pr_debug("Actual FSB frequency detected by SOC 0x%x : %x\n",
			hi, lo);

		bus_freq = lo & 0x7;
		pr_debug("bus_freq = 0x%x\n", bus_freq);

		if (bus_freq == 0)
			fsb = FSB_FREQ_83SKU;
		else if (bus_freq == 1)
			fsb = FSB_FREQ_100SKU;
		else if (bus_freq == 2)
			fsb = FSB_FREQ_133SKU;
		else if (bus_freq == 3)
			fsb = FSB_FREQ_167SKU;
		else if (bus_freq == 4)
			fsb = FSB_FREQ_83SKU;
		else if (bus_freq == 5)
			fsb = FSB_FREQ_400SKU;
		else if (bus_freq == 6)
			fsb = FSB_FREQ_267SKU;
		else if (bus_freq == 7)
			fsb = FSB_FREQ_333SKU;
		else {
			BUG();
			pr_err("Invalid bus_freq! Setting to minimal value!\n");
			fsb = FSB_FREQ_100SKU;
		}

		/* TSC = FSB Freq * Resolved HFM Ratio */
		fast_calibrate = ratio * fsb;
		pr_debug("calculate tangier tsc %lu KHz\n", fast_calibrate);

		/* ************************************ */
		/* Calculate Local APIC Timer Frequency */
		/* ************************************ */
		lapic_timer_frequency = (fsb * 1000) / HZ;

		pr_debug("Setting lapic_timer_frequency = %d\n",
			lapic_timer_frequency);

		/* mark tsc clocksource as reliable */
		set_cpu_cap(&boot_cpu_data, X86_FEATURE_TSC_RELIABLE);

		if (fast_calibrate)
			return fast_calibrate;
	}
	return 0;
}

/* Allow user to enable simulator quirks settings for kernel */
static int __init set_simulation_platform(char *str)
{
	int platform;

	__intel_mid_sim_platform = INTEL_MID_CPU_SIMULATION_NONE;
	if (get_option(&str, &platform)) {
		__intel_mid_sim_platform = platform;
		pr_info("simulator mode %d enabled.\n",
			__intel_mid_sim_platform);
		return 0;
	}

	return -EINVAL;
}
early_param("mrfld_simulation", set_simulation_platform);

static void __init tangier_time_init(void)
{
	/* [REVERT ME] ARAT capability not set in VP. Force setting */
	if (intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_VP ||
	    intel_mid_identify_sim() == INTEL_MID_CPU_SIMULATION_HVP)
		set_cpu_cap(&boot_cpu_data, X86_FEATURE_ARAT);

	if (intel_mid_timer_init)
		intel_mid_timer_init();
}

static void __init tangier_arch_setup(void)
{
	x86_platform.calibrate_tsc = tangier_calibrate_tsc;
	intel_mid_timer_init = x86_init.timers.timer_init;
	x86_init.timers.timer_init = tangier_time_init;
}

static int set_safe_battery_param(void)
{
	pr_info("%s:\n", __func__);

	battery_chrg_profile = kzalloc(
			sizeof(*battery_chrg_profile), GFP_KERNEL);
	if (!battery_chrg_profile) {
		pr_err("%s(): Error in kzalloc\n", __func__);
		return -ENOMEM;
	}

	/* Populate the safe charging parameters */
	memcpy(battery_chrg_profile->batt_id, "I2", 2);
	battery_chrg_profile->battery_type = 2;
	battery_chrg_profile->capacity = 2100;
	battery_chrg_profile->voltage_max = 4100;
	battery_chrg_profile->chrg_term_ma = 150;
	battery_chrg_profile->low_batt_mV = 3500;
	battery_chrg_profile->disch_tmp_ul = 60;
	battery_chrg_profile->disch_tmp_ll = 0;
	battery_chrg_profile->temp_mon_ranges = 4;

	battery_chrg_profile->temp_mon_range[0].temp_up_lim  = 60;
	battery_chrg_profile->temp_mon_range[0].full_chrg_vol = 4100;
	battery_chrg_profile->temp_mon_range[0].full_chrg_cur = 500;
	battery_chrg_profile->temp_mon_range[0].maint_chrg_vol_ll = 4050;
	battery_chrg_profile->temp_mon_range[0].maint_chrg_vol_ul = 4100;
	battery_chrg_profile->temp_mon_range[0].maint_chrg_cur = 500;

	battery_chrg_profile->temp_mon_range[1].temp_up_lim  = 45;
	battery_chrg_profile->temp_mon_range[1].full_chrg_vol = 4100;
	battery_chrg_profile->temp_mon_range[1].full_chrg_cur = 500;
	battery_chrg_profile->temp_mon_range[1].maint_chrg_vol_ll = 4050;
	battery_chrg_profile->temp_mon_range[1].maint_chrg_vol_ul = 4100;
	battery_chrg_profile->temp_mon_range[1].maint_chrg_cur = 500;

	battery_chrg_profile->temp_mon_range[2].temp_up_lim  = 28;
	battery_chrg_profile->temp_mon_range[2].full_chrg_vol = 4100;
	battery_chrg_profile->temp_mon_range[2].full_chrg_cur = 500;
	battery_chrg_profile->temp_mon_range[2].maint_chrg_vol_ll = 4050;
	battery_chrg_profile->temp_mon_range[2].maint_chrg_vol_ul = 4100;
	battery_chrg_profile->temp_mon_range[2].maint_chrg_cur = 500;

	battery_chrg_profile->temp_mon_range[3].temp_up_lim  = 10;
	battery_chrg_profile->temp_mon_range[3].full_chrg_vol = 4100;
	battery_chrg_profile->temp_mon_range[3].full_chrg_cur = 500;
	battery_chrg_profile->temp_mon_range[3].maint_chrg_vol_ll = 4050;
	battery_chrg_profile->temp_mon_range[3].maint_chrg_vol_ul = 4100;
	battery_chrg_profile->temp_mon_range[3].maint_chrg_cur = 500;

	battery_chrg_profile->temp_low_lim = 0;

	ps_batt_chrg_profile = kzalloc(sizeof(*ps_batt_chrg_profile),
					GFP_KERNEL);
	if (!ps_batt_chrg_profile) {
		pr_err("%s(): Error in kzalloc\n", __func__);
		kfree(battery_chrg_profile);
		return -ENOMEM;
	}

	ps_batt_chrg_profile->chrg_prof_type = PSE_MOD_CHRG_PROF;
	ps_batt_chrg_profile->batt_prof = battery_chrg_profile;
#ifdef CONFIG_POWER_SUPPLY_BATTID
	battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED,
						ps_batt_chrg_profile);
#endif
	return 1;
}

static int charge_invalid_battery(void)
{
	void __iomem *fpo1_iomap;
	void __iomem *miscflg_iomap;
	int fpo1, miscflg;

	pr_info("%s:\n", __func__);

	/* Check if invalid battery charging is allowed */
	miscflg_iomap = ioremap_nocache(
			(SMIP_SRAM_ADDRESS + SMIP_EM_MISCFLAGS), 4);
	miscflg = ioread8(miscflg_iomap);

	fpo1_iomap = ioremap_nocache(
			(SMIP_SRAM_ADDRESS + SMIP_FPO1_OFFSET), 4);
	fpo1 = ioread8(fpo1_iomap);

	if ((fpo1 & CHARGE_INVALID_BATTERY) &&
		(miscflg & BYPASSINVALID))
		return set_safe_battery_param();
	else
		return 0;
}

static void override_batt_chrg_prof(struct ps_pse_mod_prof *batt_prof)
{
	int i;

	for (i = 0; i <= batt_prof->temp_mon_ranges; i++) {
		batt_prof->temp_mon_range[i].full_chrg_vol = 4100;
		batt_prof->temp_mon_range[i].maint_chrg_vol_ll = 4000;
		batt_prof->temp_mon_range[i].maint_chrg_vol_ul = 4100;
	}
}

static void set_batt_chrg_prof(struct ps_pse_mod_prof *batt_prof,
				struct ps_pse_mod_prof *pentry)
{
	int i, j;

	if (batt_prof == NULL || pentry == NULL) {
		pr_err("%s: Invalid Pointer\n", __func__);
		return;
	}

	memcpy(batt_prof->batt_id, pentry->batt_id, BATTID_STR_LEN);
	batt_prof->battery_type = pentry->battery_type;
	batt_prof->capacity = pentry->capacity;
	batt_prof->voltage_max = pentry->voltage_max;
	if ((pentry->batt_id[0] == 'I') && (pentry->batt_id[1] == '2'))
		batt_prof->chrg_term_ma = MRFL_I2_TERM_MA;
	else
		batt_prof->chrg_term_ma = pentry->chrg_term_ma;

	if (INTEL_MID_BOARD(1, PHONE, MOFD) ||
		INTEL_MID_BOARD(1, TABLET, MOFD))
		batt_prof->low_batt_mV = MOFD_VBAT_MIN;
	else
		batt_prof->low_batt_mV = pentry->low_batt_mV;
	batt_prof->disch_tmp_ul = pentry->disch_tmp_ul;
	batt_prof->disch_tmp_ll = pentry->disch_tmp_ll;
	batt_prof->temp_low_lim = pentry->temp_low_lim;

	for (i = 0, j = 0; i < pentry->temp_mon_ranges; i++) {
		if (pentry->temp_mon_range[i].temp_up_lim != 0xff) {
			memcpy(&batt_prof->temp_mon_range[j],
			       &pentry->temp_mon_range[i],
			       sizeof(struct ps_temp_chg_table));
			j++;
		}
	}
	batt_prof->temp_mon_ranges = j;

	/* Overriding Charger profile values for UER Battery */
	if (INTEL_MID_BOARD(1, PHONE, MOFD) ||
		INTEL_MID_BOARD(1, TABLET, MOFD)) {
		if ((pentry->batt_id[0] == 'I') && (pentry->batt_id[1] == '2'))
			override_batt_chrg_prof(batt_prof);
	}

	return;
}

static int __init mrfl_platform_init(void)
{
	struct sfi_table_simple *sb;
	struct ps_pse_mod_prof *pentry;
	int totentrs = 0, totlen = 0;
	struct sfi_table_header *table;

	table = get_oem0_table();

	if (!(INTEL_MID_BOARD(1, PHONE, MRFL) ||
	      INTEL_MID_BOARD(1, TABLET, MRFL) ||
		INTEL_MID_BOARD(1, PHONE, MOFD) ||
		 INTEL_MID_BOARD(1, TABLET, MOFD)))
		return 0;

	if (!table) {
		if ((INTEL_MID_BOARD(1, PHONE, MOFD) ||
			INTEL_MID_BOARD(1, TABLET, MOFD)))
			return charge_invalid_battery();
		else
			return 0;
	}

	sb = (struct sfi_table_simple *)table;
	totentrs = SFI_GET_NUM_ENTRIES(sb, struct ps_pse_mod_prof);
	if (totentrs) {
		battery_chrg_profile = kzalloc(
				sizeof(*battery_chrg_profile), GFP_KERNEL);
		if (!battery_chrg_profile) {
			pr_err("%s(): Error in kzalloc\n", __func__);
			return -ENOMEM;
		}
		pentry = (struct ps_pse_mod_prof *)sb->pentry;
		totlen = totentrs * sizeof(*pentry);
		if (totlen <= sizeof(*battery_chrg_profile)) {
			set_batt_chrg_prof(battery_chrg_profile, pentry);
			ps_batt_chrg_profile = kzalloc(
					sizeof(*ps_batt_chrg_profile),
					GFP_KERNEL);
			if (!ps_batt_chrg_profile) {
				pr_err("%s(): Error in kzalloc\n", __func__);
				kfree(battery_chrg_profile);
				return -ENOMEM;
			}
			ps_batt_chrg_profile->chrg_prof_type =
				PSE_MOD_CHRG_PROF;
			ps_batt_chrg_profile->batt_prof = battery_chrg_profile;
#ifdef CONFIG_POWER_SUPPLY_BATTID
			battery_prop_changed(POWER_SUPPLY_BATTERY_INSERTED,
					     ps_batt_chrg_profile);
#endif
		} else {
			pr_err("%s: Error in copying batt charge profile\n",
				__func__);
			kfree(battery_chrg_profile);
			return -ENOMEM;
		}
	} else {
		pr_err("%s: Error in finding batt charge profile\n",
			__func__);
	}

	return 0;
}
arch_initcall_sync(mrfl_platform_init);

void *get_tangier_ops(void)
{
	return &tangier_ops;
}

/* piggy back on anniedale ops right now */
void *get_anniedale_ops(void)
{
	return &tangier_ops;
}
