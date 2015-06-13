/*
 * intel_byt_thermal.c - Intel Baytrail Platform Thermal Driver
 *
 * Copyright (C) 2013 Intel Corporation
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the GNU
 * General Public License for more details.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Durgadoss R <durgadoss.r@intel.com>
 */

#define pr_fmt(fmt)  "intel_byt_thermal: " fmt

#include <linux/pm.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/kfifo.h>
#include <linux/module.h>
#include <linux/debugfs.h>
#include <linux/thermal.h>
#include <linux/seq_file.h>
#include <linux/interrupt.h>
#include <linux/completion.h>
#include <linux/platform_device.h>

#include <linux/mfd/intel_mid_pmic.h>

#include <asm/processor.h>
#include <asm/intel_mid_thermal.h>
#include <asm/intel_crystalcove_gpadc.h>

#include <linux/iio/consumer.h>

#define DEVICE_NAME "crystal_cove_thermal"

/* Number of Thermal sensors on the PMIC */
#define PMIC_THERMAL_SENSORS	4

/* Registers that govern Thermal Monitoring */
#define THRMIRQ0	0x04
#define THRMIRQ1	0x05
#define MTHRMIRQ0	0x11
#define MTHRMIRQ1	0x12
#define IRQLVL1		0x02
#define MIRQLVL1	0x0E
#define IRQ_MASK_ALL	0x0F

#define THRM_MON_CTRL0	0x8E
#define THRM_MON_CTRL1	0x8F
#define TS_ENABLE	0x90
#define TS_CRIT_ENABLE	0x91
#define TS_A0_STS	0x92
#define TS_A1_STS	0x93
#define TS_A2_STS	0xBD

#define PMICSTS		(1 << 5)
#define PMICALRT	(1 << 3)
#define SYS2ALRT	(1 << 2)
#define SYS1ALRT	(1 << 1)
#define SYS0ALRT	(1 << 0)
#define THERM_EN	(1 << 0)
#define ALERT_EN	(1 << 6)
#define PROCHOT_EN	(1 << 7)
#define IRQ_LVL1_EN	(1 << 1)
#define TS_ENABLE_ALL	0x27

/* ADC to Temperature conversion table length */
#define TABLE_LENGTH	35
#define TEMP_INTERVAL	5

/* Default Alert threshold 85 C */
#define DEFAULT_MAX_TEMP	85
#define MIN_CRIT_TEMP		55
#define PMIC_DIE_MIN_CRIT_TEMP	120

/*
 * Default Hysteresis value: 15 corresponds to 3C.
 * Why 15? : Hysteresis value is 4 bits wide. This is
 * the maximum possible value that can be supported.
 * Can be changed at run-time through Sysfs interface
 */
#define DEFAULT_HYST		15

#define NUM_ALERT_LEVELS	3
#define ALERT_RW_MASK		0x07
#define LEVEL_ALERT0		0
#define LEVEL_ALERT1		1
#define LEVEL_ALERT2		2

/*
 * LOW event is defined as 0 (implicit)
 * HIGH event is defined as 1 (implicit)
 * Hence this event is defined as 2.
 */
#define EMUL_TEMP_EVENT		2
#define TEMP_WRITE_TIMEOUT	(2 * HZ)

/* Constants defined in CrystalCove PMIC spec */
#define PMIC_DIE_SENSOR		3
#define PMIC_DIE_ADC_MIN	488
#define PMIC_DIE_ADC_MAX	802
#define PMIC_DIE_TEMP_MIN	-40
#define PMIC_DIE_TEMP_MAX	125

#define IRQ_FIFO_MAX		16

struct thermal_event {
	int sensor; /* For which sensor ? */
	int event;  /* Whether LOW or HIGH event ? */
	int level;  /* Which alert level 0/1/2 ? */
};

static DEFINE_KFIFO(irq_fifo, struct thermal_event, IRQ_FIFO_MAX);

static const int params[3][PMIC_THERMAL_SENSORS] = {
		{ TS_A0_STS, TS_A1_STS, TS_A2_STS, 0 }, /* status register */
		{ SYS0ALRT, SYS1ALRT, SYS2ALRT, PMICALRT }, /* Interrupt bit */
		{ SYS0ALRT, SYS1ALRT, SYS2ALRT, PMICSTS },  /* Status bit */
		};
/*
 * ADC Result registers: The 10 bit ADC code is stored in two registers.
 * The 'high' register holds D[8:9] of the ADC code, in D[0:1]. The 'low'
 * register holds D[0:7] of the ADC code. These register addresses are
 * consecutive.
 */
static const int adc_res_reg_l[PMIC_THERMAL_SENSORS] = {
					0x75, 0x77, 0x79, 0x7F };
/*
 * Alert registers store the 'alert' temperature for each sensor,
 * as 10 bit ADC code. The higher two bits are stored in bits[0:1] of
 * alert_regs_h. The lower eight bits are stored in alert_regs_l.
 * The hysteresis value is stored in bits[2:5] of alert_regs_h.
 * Alert level 2 (also known as 'critical level') is 8 bits wide
 * and hence does not have a 'high' register.
 *
 * static const int alert_regs_h[3][4] = {
 *			SYS0, SYS1, SYS2, PMIC_DIE
 *		Alert 0	{ 0x94, 0x99, 0x9E, 0xAF },
 *		Alert 1	{ 0x96, 0x9B, 0xA0, 0xB1 },
 *		Alert 2	{    -,    -,    -,    - },
 *			};
 */
static const int alert_regs_l[3][4] = {
				/* SYS0, SYS1, SYS2, PMIC_DIE */
		/* Alert 0 */	{ 0x95, 0x9A, 0x9F, 0xB0 },
		/* Alert 1 */	{ 0x97, 0x9C, 0xA1, 0xB2 },
		/* Alert 2 */	{ 0x98, 0x9D, 0xA2, 0xB3 },
				};
/*
 * ADC code vs Temperature table
 * This table will be different for different thermistors
 * Row 0: ADC code
 * Row 1: Temperature (in degree celsius)
 */
static const int adc_code[2][TABLE_LENGTH] = {
	{977, 961, 941, 917, 887, 853, 813, 769, 720, 669, 615, 561, 508, 456,
		407, 357, 315, 277, 243, 212, 186, 162, 140, 107,
		94, 82, 72, 64, 56, 50, 44, 39, 35, 31},
	{-20, -15, -10, -5, 0, 5, 10, 15, 20, 25, 30, 35, 40, 45, 50, 55, 60,
		65, 70, 75, 80, 85, 90, 95, 100, 105, 110, 115, 120, 125,
		130, 135, 140, 145, 150},
	};

static DEFINE_MUTEX(thrm_update_lock);

struct thermal_device_info {
	struct intel_mid_thermal_sensor *sensor;
	long trip_temp[NUM_ALERT_LEVELS];
	struct completion temp_write_complete;
	int sensor_index;
};

struct thermal_data {
	struct platform_device *pdev;
	struct iio_channel *iio_chan;
	struct work_struct thermal_work;
	struct mutex thrm_irq_lock;
	struct thermal_zone_device **tzd;
	/* Caching information */
	bool is_initialized;
	unsigned long last_updated;
	int cached_vals[PMIC_THERMAL_SENSORS];
	/* Details obtained from platform data */
	int num_sensors;
	int num_virtual_sensors;
	unsigned int irq;
	struct intel_mid_thermal_sensor *sensors;
	/* CPU data */
	bool is_vlv;
};
static struct thermal_data *tdata;

#ifdef CONFIG_DEBUG_FS
static struct thermal_regs {
	char *name;
	int addr;
} thermal_regs[] = {
	/* Thermal Management Registers */
	{"THRM_MON_CTRL0",	0x8E},
	{"THRM_MON_CTRL1",	0x8F},
	{"TS_ENABLE",		0x90},
	{"TS_CRIT_ENABLE",	0x91},
	{"TS_A0_STATUS",	0x92},
	{"TS_A1_STATUS",	0x93},
	{"TS_CRIT_STATUS",	0xBD},
	{"IRQLVL1",		0x02},
	{"MIRQLVL1",		0x0E},
	{"THRMIRQ0",		0x04},
	{"THRMIRQ1",		0x05},
	{"MTHRMIRQ0",		0x11},
	{"MTHRMIRQ1",		0x12},
	{"A0_SYS0_H",		0x94},
	{"A0_SYS1_H",		0x99},
	{"A0_SYS2_H",		0x9E},
	{"A0_BAT0_H",		0xA3},
	{"A0_BAT1_H",		0xA9},
	{"A0_PMIC_H",		0xAF},
};

static struct dentry *thermal_dent[ARRAY_SIZE(thermal_regs)];
static struct dentry *ccove_thermal_dir;

static int ccove_thermal_debugfs_show(struct seq_file *s, void *unused)
{
	int addr = *((int *)s->private);
	int val = intel_mid_pmic_readb(addr);

	seq_printf(s, "Addr[0x%X] Val: 0x%X\n", addr, val);

	return 0;
}

static int debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, ccove_thermal_debugfs_show, inode->i_private);
}

static const struct file_operations ccove_thermal_debugfs_ops = {
	.open           = debugfs_open,
	.read           = seq_read,
	.llseek         = seq_lseek,
	.release        = single_release,
};

static void create_ccove_thermal_debugfs(void)
{
	int i, err;

	/* /sys/kernel/debug/ccove_thermal/ */
	ccove_thermal_dir = debugfs_create_dir("ccove_thermal", NULL);
	if (IS_ERR(ccove_thermal_dir)) {
		err = PTR_ERR(ccove_thermal_dir);
		pr_err("debugfs_create_dir failed:%d\n", err);
		return;
	}

	/* /sys/kernel/debug/ccove_thermal/REG_NAME */
	for (i = 0; i < ARRAY_SIZE(thermal_regs); i++) {
		thermal_dent[i] = debugfs_create_file(thermal_regs[i].name,
						S_IFREG | S_IRUGO,
						ccove_thermal_dir,
						&thermal_regs[i].addr,
						&ccove_thermal_debugfs_ops);
		if (IS_ERR(thermal_dent[i])) {
			err = PTR_ERR(thermal_dent[i]);
			debugfs_remove_recursive(ccove_thermal_dir);
			pr_debug("debugfs_create_file failed:%d\n", err);
		}
	}
}

static void remove_ccove_thermal_debugfs(void)
{
	debugfs_remove_recursive(ccove_thermal_dir);
}
#else
static inline void create_pmic_thermal_debugfs(void) { }
static inline void remove_pmic_thermal_debugfs(void) { }
#endif

static inline int adc_to_pmic_die_temp(unsigned int val)
{
	/* return temperature in mC */
	return 382100 - val * 526;
}

static inline int pmic_die_temp_to_adc(int temp)
{
	/* 'temp' is in C, convert to mC and then do calculations */
	return (382100 - temp * 1000) / 526;
}

/**
 * find_adc_code - searches the ADC code using binary search
 * @val: value to find in the array
 *
 * This function does binary search on an array sorted in 'descending' order
 * Can sleep
 */
static int find_adc_code(uint16_t val)
{
	int left = 0;
	int right = TABLE_LENGTH - 1;
	int mid;
	while (left <= right) {
		mid = (left + right)/2;
		if (val == adc_code[0][mid] ||
			(mid > 0 &&
			val > adc_code[0][mid] && val < adc_code[0][mid-1]))
			return mid;
		else if (val > adc_code[0][mid])
			right = mid - 1;
		else if (val < adc_code[0][mid])
			left = mid + 1;
	}
	return -EINVAL;
}

/**
 * adc_to_temp - converts the ADC code to temperature in mC
 * @direct: true if the sensor uses direct conversion
 * @adc_val: the ADC code to be converted
 * @tp: temperature return value
 *
 * Can sleep
 */
static int adc_to_temp(int direct, uint16_t adc_val, long *tp)
{
	int x0, x1, y0, y1;
	int nr, dr;		/* Numerator & Denominator */
	int indx;
	int x = adc_val;

	/* Direct conversion for pmic die temperature */
	if (direct) {
		if (adc_val < PMIC_DIE_ADC_MIN || adc_val > PMIC_DIE_ADC_MAX)
			return -EINVAL;

		*tp = adc_to_pmic_die_temp(adc_val);
		return 0;
	}

	indx = find_adc_code(adc_val);
	if (indx < 0)
		return -EINVAL;

	if (adc_code[0][indx] == adc_val) {
		*tp = adc_code[1][indx] * 1000;
		return 0;
	}

	/*
	 * The ADC code is in between two values directly defined in the
	 * table. So, do linear interpolation to calculate the temperature.
	 */
	x0 = adc_code[0][indx];
	x1 = adc_code[0][indx - 1];
	y0 = adc_code[1][indx];
	y1 = adc_code[1][indx - 1];

	/*
	 * Find y:
	 * Of course, we can avoid these variables, but keep them
	 * for readability and maintainability.
	 */
	nr = (x-x0)*y1 + (x1-x)*y0;
	dr =  x1-x0;

	if (!dr)
		return -EINVAL;
	/*
	 * We have to report the temperature in milli degree celsius.
	 * So, to reduce the loss of precision, do (Nr*1000)/Dr, instead
	 * of (Nr/Dr)*1000.
	 */
	*tp = (nr * 1000)/dr;

	return 0;
}

/**
 * temp_to_adc - converts the temperature(in C) to ADC code
 * @direct: true if the sensor uses direct conversion
 * @temp: the temperature to be converted
 * @adc_val: ADC code return value
 *
 * Can sleep
 */
static int temp_to_adc(int direct, int temp, int *adc_val)
{
	int indx;
	int x0, x1, y0, y1;
	int nr, dr;		/* Numerator & Denominator */
	int x = temp;

	/* Direct conversion for pmic die temperature */
	if (direct) {
		if (temp < PMIC_DIE_TEMP_MIN || temp > PMIC_DIE_TEMP_MAX)
			return -EINVAL;

		*adc_val = pmic_die_temp_to_adc(temp);
		return 0;
	}

	if (temp < adc_code[1][0] || temp > adc_code[1][TABLE_LENGTH - 1])
		return -EINVAL;


	/* Find the 'indx' of this 'temp' in the table */
	indx = (temp - adc_code[1][0]) / TEMP_INTERVAL;

	if (temp == adc_code[1][indx]) {
		*adc_val = adc_code[0][indx];
		return 0;
	}

	/*
	 * Temperature is not a multiple of 'TEMP_INTERVAL'. So,
	 * do linear interpolation to obtain a better ADC code.
	 */
	x0 = adc_code[1][indx];
	x1 = adc_code[1][indx + 1];
	y0 = adc_code[0][indx];
	y1 = adc_code[0][indx + 1];

	nr = (x-x0)*y1 + (x1-x)*y0;
	dr =  x1-x0;

	if (!dr)
		return -EINVAL;

	*adc_val = nr/dr;

	return 0;
}

/**
 * set_hyst_val - sets the given 'val' as the 'hysteresis'
 * @alert_reg_h: The 'high' register address
 * @hyst:        Hysteresis value (in ADC codes) to be programmed
 *
 * Not protected. Calling function should handle synchronization.
 * Can sleep
 */
static int set_hyst_val(int alert_reg_h, int hyst)
{
	int ret;

	ret = intel_mid_pmic_readb(alert_reg_h);
	if (ret < 0)
		return ret;

	/* Set bits [2:5] to value of hyst */
	ret = (ret & 0xC3) | (hyst << 2);

	return intel_mid_pmic_writeb(alert_reg_h, ret);
}

/**
 * set_alert_temp - sets the given 'adc_val' to the 'alert_reg'
 * @alert_reg_l: The 'low' register address
 * @adc_val:     ADC value to be programmed
 * @level:       0 - alert0, 1 - alert1, 2 - alert2(only 8 bits wide)
 *
 * Not protected. Calling function should handle synchronization.
 * Can sleep
 */
static int set_alert_temp(int alert_reg_l, int adc_val, int level)
{
	int ret;

	/*
	 * Method used for VLV-CRC PMICs:
	 * The alert register stores B[1:8] of val and the HW
	 * while comparing prefixes and suffixes this value with
	 * a 0; i.e B[0] and B[9] are 0.
	 *
	 * Method used for CHV-CRC+ PMICs:
	 * Use B[2..9]; B[0] and B[1] are assumed to be 0 by the HW.
	 */
	if (level == LEVEL_ALERT2) {
		if (tdata->is_vlv)
			adc_val = (adc_val & 0x1FF) >> 1;
		else
			adc_val = (adc_val & 0x3FF) >> 2;

		return intel_mid_pmic_writeb(alert_reg_l, adc_val);
	}

	/* Extract bits[0:7] of 'adc_val' and write them into alert_reg_l */
	ret = intel_mid_pmic_writeb(alert_reg_l, adc_val & 0xFF);
	if (ret < 0)
		return ret;

	/* Get the address of alert_reg_h */
	--alert_reg_l;

	ret = intel_mid_pmic_readb(alert_reg_l);
	if (ret < 0)
		return ret;

	/* Set bits[0:1] of alert_reg_h to bits[8:9] of 'adc_val' */
	ret = (ret & ~0x03) | (adc_val >> 8);

	return intel_mid_pmic_writeb(alert_reg_l, ret);
}

/**
 * get_alert_temp - gets the ADC code from the alert register
 * @alert_reg_l: The 'low' register address
 * @level:       0 - alert0, 1 - alert1, 2 - alert2(only 8 bits wide)
 *
 * Not protected. Calling function should handle synchronization.
 * Can sleep
 */
static int get_alert_temp(int alert_reg_l, int level)
{
	int l, h;

	l = intel_mid_pmic_readb(alert_reg_l);
	if (l < 0)
		return l;

	if (level == LEVEL_ALERT2) {
		/* For VLV-CRC based platforms */
		if (tdata->is_vlv)
			return l << 1;
		/* For other platforms */
		return l << 2;
	}

	/* Get the address of alert_reg_h */
	--alert_reg_l;

	h = intel_mid_pmic_readb(alert_reg_l);
	if (h < 0)
		return h;

	/* Concatenate 'h' and 'l' to get 10-bit ADC code */
	return ((h & 0x03) << 8) | l;
}

static int disable_prochot(void)
{
	int i, reg, ret;

	mutex_lock(&thrm_update_lock);

	for (i = 0; i < PMIC_THERMAL_SENSORS; i++) {
		reg = alert_regs_l[0][i] - 1;
		ret = intel_mid_pmic_clearb(reg, PROCHOT_EN);
		if (ret < 0)
			goto exit;
	}

exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

/**
 * program_tmax - programs a default _max value for each sensor
 * @dev: device pointer
 *
 * Can sleep
 */
static int program_tmax(struct device *dev)
{
	int i, ret, level;
	int pmic_die_val, adc_val, val;
	int pmic_die_crit_val;

	ret = temp_to_adc(1, PMIC_DIE_MIN_CRIT_TEMP, &pmic_die_crit_val);
	if (ret)
		return ret;

	ret = temp_to_adc(1, DEFAULT_MAX_TEMP, &pmic_die_val);
	if (ret)
		return ret;

	/* ADC code corresponding to max Temp 85 C */
	ret = temp_to_adc(0, DEFAULT_MAX_TEMP, &adc_val);
	if (ret)
		return ret;

	for (level = 0; level <= LEVEL_ALERT2; level++) {
		for (i = 0; i < PMIC_THERMAL_SENSORS; i++) {
			val = (i == PMIC_DIE_SENSOR) ? pmic_die_val : adc_val;

			if (level == LEVEL_ALERT2 && i == PMIC_DIE_SENSOR)
				val = pmic_die_crit_val;

			ret = set_alert_temp(alert_regs_l[level][i],
						val, level);
			if (ret < 0)
				goto exit_err;
			/* Set default Hysteresis for Alerts 0,1 only */
			if (level == LEVEL_ALERT2)
				continue;
			ret = set_hyst_val(alert_regs_l[level][i] - 1,
					DEFAULT_HYST);
			if (ret < 0)
				goto exit_err;
		}
	}

	return ret;

exit_err:
	dev_err(dev, "set alert %d for channel %d failed:%d\n", level, i, ret);
	return ret;
}

static ssize_t store_trip_hyst(struct thermal_zone_device *tzd,
				int trip, long hyst)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg = alert_regs_l[trip][td_info->sensor_index] - 1;

	/*
	 * Alert level 2 does not support hysteresis; and (for
	 * other levels) the hysteresis value is 4 bits wide.
	 */
	if (trip == LEVEL_ALERT2 || hyst > DEFAULT_HYST)
		return -EINVAL;

	mutex_lock(&thrm_update_lock);

	ret = set_hyst_val(alert_reg, hyst);

	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_trip_hyst(struct thermal_zone_device *tzd,
				int trip, long *hyst)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg_l = alert_regs_l[trip][td_info->sensor_index];

	if (trip == LEVEL_ALERT2) {
		*hyst = 0;
		return 0;
	}

	mutex_lock(&thrm_update_lock);

	/* Get the address of alert_reg_h */
	--alert_reg_l;

	ret = intel_mid_pmic_readb(alert_reg_l);
	if (ret >= 0)
		*hyst = (ret >> 2) & 0x0F; /* Extract bits[2:5] of data */

	mutex_unlock(&thrm_update_lock);

	return 0;
}

static ssize_t store_trip_temp(struct thermal_zone_device *tzd,
				int trip, long trip_temp)
{
	int ret, adc_val, min_tcrit = MIN_CRIT_TEMP;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg_l = alert_regs_l[trip][td_info->sensor_index];

	if (trip_temp != 0 && trip_temp < 1000) {
		dev_err(&tzd->device, "Temperature should be in mC\n");
		return -EINVAL;
	}

	/* Convert to C */
	trip_temp /= 1000;

	/* Minimum Tcrit for PMIC DIE is different from that of others */
	if (td_info->sensor->direct)
		min_tcrit = PMIC_DIE_MIN_CRIT_TEMP;

	if (trip == LEVEL_ALERT2 && trip_temp < min_tcrit) {
		dev_err(&tzd->device,
			"Tcrit should be more than %dC\n", min_tcrit);
		return -EINVAL;
	}

	mutex_lock(&thrm_update_lock);

	ret = temp_to_adc(td_info->sensor->direct, (int)trip_temp, &adc_val);
	if (ret) {
		adc_val = trip_temp > 0 ?
				adc_code[0][TABLE_LENGTH - 1] : adc_code[0][0];

		dev_err(&tzd->device,
			"ADC code out of range. Capping it to %s possible\n",
			trip_temp > 0 ? "highest" : "lowest");
	}

	/*
	 * Hold the irq lock, so that no alert threshold for any sensor
	 * is being programmed while we are handling an interrupt
	 */
	mutex_lock(&tdata->thrm_irq_lock);

	ret = set_alert_temp(alert_reg_l, adc_val, trip);

	mutex_unlock(&tdata->thrm_irq_lock);

	/* Store the trip point value written into the register */
	ret = adc_to_temp(td_info->sensor->direct, adc_val,
			&td_info->trip_temp[trip]);
	if (ret)
		dev_err(&tzd->device,
			"adc_to_temp for trip%d failed:%d\n", trip, ret);

	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_trip_temp(struct thermal_zone_device *tzd,
				int trip, long *trip_temp)
{
	int ret = -EINVAL, adc_val;
	struct thermal_device_info *td_info = tzd->devdata;
	int alert_reg_l = alert_regs_l[trip][td_info->sensor_index];

	mutex_lock(&thrm_update_lock);

	adc_val = get_alert_temp(alert_reg_l, trip);
	if (adc_val < 0)
		goto exit;

	ret = adc_to_temp(td_info->sensor->direct, adc_val, trip_temp);
exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static ssize_t show_trip_type(struct thermal_zone_device *tzd,
			int trip, enum thermal_trip_type *trip_type)
{
	/* All are passive trip points */
	*trip_type = THERMAL_TRIP_PASSIVE;

	return 0;
}

static int read_result_regs(void)
{
	int i, ret;

	for (i = 0; i < PMIC_THERMAL_SENSORS; i++) {
		/*
		 * Exploit the fact that the result registers store
		 * the value in the same format as that of the alert
		 * registers. So, use get_alert_temp but pass the
		 * result register address, and level as 0.
		 */
		ret = get_alert_temp(adc_res_reg_l[i], 0);
		if (ret < 0)
			return ret;
		tdata->cached_vals[i] = ret;
	}
	return 0;
}

static int update_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret;
	struct thermal_device_info *td_info = tzd->devdata;
	int indx = td_info->sensor_index;

	if (!tdata->iio_chan)
		return -EINVAL;

	if (!tdata->is_initialized ||
			time_after(jiffies, tdata->last_updated + HZ)) {
		ret = iio_read_channel_all_raw(tdata->iio_chan,
						tdata->cached_vals);
		if (ret == -ETIMEDOUT) {
			dev_err(&tzd->device,
				"ADC sampling failed:%d Reading rslt regs\n",
				ret);

			ret = read_result_regs();
			if (ret)
				return ret;
		}
		tdata->last_updated = jiffies;
		tdata->is_initialized = true;
	}

	ret = adc_to_temp(td_info->sensor->direct,
				tdata->cached_vals[indx], temp);
	return ret;
}

static ssize_t show_emul_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret = 0;
	char *thermal_event[3];
	unsigned long timeout;
	struct thermal_device_info *td_info = tzd->devdata;

	thermal_event[0] = kasprintf(GFP_KERNEL, "NAME=%s", tzd->type);
	thermal_event[1] = kasprintf(GFP_KERNEL, "EVENT=%d", EMUL_TEMP_EVENT);
	thermal_event[2] = NULL;

	INIT_COMPLETION(td_info->temp_write_complete);
	kobject_uevent_env(&tzd->device.kobj, KOBJ_CHANGE, thermal_event);

	timeout = wait_for_completion_timeout(&td_info->temp_write_complete,
						TEMP_WRITE_TIMEOUT);
	if (timeout == 0) {
		/* Waiting timed out */
		ret = -ETIMEDOUT;
		goto exit;
	}

	*temp = tzd->emul_temperature;
exit:
	kfree(thermal_event[1]);
	kfree(thermal_event[0]);
	return ret;
}

static ssize_t store_emul_temp(struct thermal_zone_device *tzd,
				unsigned long temp)
{
	struct thermal_device_info *td_info = tzd->devdata;

	tzd->emul_temperature = temp;
	complete(&td_info->temp_write_complete);
	return 0;
}

static ssize_t show_temp(struct thermal_zone_device *tzd, long *temp)
{
	int ret;

	mutex_lock(&thrm_update_lock);

	ret = update_temp(tzd, temp);

	mutex_unlock(&thrm_update_lock);
	return ret;
}

static int enable_tm(void)
{
	int i, reg, ret, level;

	mutex_lock(&thrm_update_lock);

	/* Setting these bits enables ADC to poll for these Thermistors */
	ret = intel_mid_pmic_setb(TS_ENABLE, TS_ENABLE_ALL);
	if (ret < 0)
		goto exit;

	/*
	 * Enable Interrupts for Alert level 0 and 1. Alert
	 * level 2 is critical and is enabled by default in the HW.
	 */
	for (level = 0; level <= LEVEL_ALERT1; level++) {
		/* Unmask the 2nd level interrupts for Alerts 0,1,2 */
		ret = intel_mid_pmic_writeb(MTHRMIRQ0 + level, 0x00);
		if (ret < 0)
			goto exit;

		for (i = 0; i < PMIC_THERMAL_SENSORS; i++) {
			reg = alert_regs_l[level][i] - 1;
			ret = intel_mid_pmic_setb(reg, ALERT_EN);
			if (ret < 0)
				goto exit;
		}
	}

	/* Unmask the first level IRQ bit for Thermal alerts */
	ret = intel_mid_pmic_clearb(MIRQLVL1, IRQ_LVL1_EN);

exit:
	mutex_unlock(&thrm_update_lock);
	return ret;
}

static struct thermal_device_info *initialize_sensor(int index,
				struct intel_mid_thermal_sensor *sensor)
{
	struct thermal_device_info *td_info =
		kzalloc(sizeof(struct thermal_device_info), GFP_KERNEL);

	if (!td_info)
		return NULL;

	td_info->sensor = sensor;
	td_info->sensor_index = index;

	init_completion(&td_info->temp_write_complete);
	return td_info;
}

static void notify_thermal_event(struct thermal_event te)
{
	int ret;
	long cur_temp;
	char *thermal_event[5];

	struct thermal_zone_device *tzd = tdata->tzd[te.sensor];
	struct thermal_device_info *td_info = tzd->devdata;

	mutex_lock(&thrm_update_lock);

	/*
	 * We want to get the 'latest' temperature value after an
	 * interrupt. So, make sure update_temp() actually samples and
	 * returns the 'latest' temperature values; and not
	 * the previously cached ones.
	 */
	tdata->is_initialized = false;

	/*
	 * Read the current Temperature and send it to user land;
	 * so that the user space can avoid a sysfs read.
	 */
	ret = update_temp(tzd, &cur_temp);
	if (ret) {
		dev_err(&tzd->device, "Cannot update temperature\n");
		goto exit;
	}

	pr_info("Thermal Event: sensor: %s, cur_temp: %ld, event: %d, level: %d\n",
			tzd->type, cur_temp, te.event, te.level);
	/*
	 * Send UEvents only when temperature goes below
	 * ALERT0 or goes above ALERT1. No UEvents for
	 * ALERT2 as the actions are taken by Hardware.
	 */
	if (te.level == LEVEL_ALERT2 ||
			(te.event == 1 && te.level == LEVEL_ALERT0) ||
			(te.event == 0 && te.level == LEVEL_ALERT1)) {
		goto exit;
	}

	/*
	 * For Cold-to-Hot events, make sure the temperature is
	 * 'at least' as high as the programmed threshold.
	 */
	if (te.event == 1) {
		if (cur_temp < td_info->trip_temp[te.level])
			cur_temp = td_info->trip_temp[te.level];
	}

	thermal_event[0] = kasprintf(GFP_KERNEL, "NAME=%s", tzd->type);
	thermal_event[1] = kasprintf(GFP_KERNEL, "TEMP=%ld", cur_temp);
	thermal_event[2] = kasprintf(GFP_KERNEL, "EVENT=%d", te.event);
	thermal_event[3] = kasprintf(GFP_KERNEL, "LEVEL=%d", te.level);
	thermal_event[4] = NULL;

	kobject_uevent_env(&tzd->device.kobj, KOBJ_CHANGE, thermal_event);

	kfree(thermal_event[3]);
	kfree(thermal_event[2]);
	kfree(thermal_event[1]);
	kfree(thermal_event[0]);

exit:
	mutex_unlock(&thrm_update_lock);
	return;
}

static int update_intrpt_params(int irq, int level)
{
	int i, sts;
	struct thermal_event te;

	/* Read the appropriate status register */
	sts = intel_mid_pmic_readb(params[0][level]);
	if (sts < 0)
		return sts;

	for (i = 0; i < PMIC_THERMAL_SENSORS; i++) {
		if (irq & params[1][i]) {
			te.level = level;
			te.sensor = i;
			te.event = !!(sts & params[2][i]);
			kfifo_put(&irq_fifo, &te);
		}
	}
	return 0;
}

static void thermal_work_func(struct work_struct *work)
{
	int gotten;
	struct thermal_event te;

	while (!kfifo_is_empty(&irq_fifo)) {
		gotten = kfifo_get(&irq_fifo, &te);
		if (!gotten) {
			pr_err("kfifo empty\n");
			return;
		}

		/* Notify the user space through UEvent */
		notify_thermal_event(te);
	}
}

static irqreturn_t thermal_intrpt(int irq_nr, void *id)
{
	int ret, irq, irq_cache, reg, level;
	struct thermal_data *tdata = (struct thermal_data *)id;

	if (!tdata)
		return IRQ_HANDLED;

	mutex_lock(&tdata->thrm_irq_lock);

	/*
	 * Assertion of THRMIRQ1[0:3] indicates Alert2 ('critical')
	 * event. Register Layout:
	 *
	 * THRMIRQ0: PMIC_DIE SYS2 SYS1 SYS0 PMIC_DIE SYS2 SYS1 SYS0
	 * Alert0-1:   Alert1 A1   A1   A1   Alert0   A0   A0   A0
	 *
	 * THRMIRQ1:     RSVD RSVD RSVD RSVD PMIC_DIE SYS2 SYS1 SYS0
	 *  Alert 2:			     Alert2   A2   A2   A2
	 */
	for (level = 0; level <= LEVEL_ALERT2; level++) {
		reg = THRMIRQ0 + (level == LEVEL_ALERT2);
		irq = intel_mid_pmic_readb(reg);
		if (irq < 0)
			goto exit;

		irq_cache = irq;

		irq = irq >> (PMIC_THERMAL_SENSORS * (level == LEVEL_ALERT1));
		if (irq & 0x0F)
			goto handle_event;
	}

	/*
	 * If we are here, then this event is none of Alert0/1/2
	 * events but somehow we got the interrupt. Just exit.
	 * Very unlikely case though.
	 */
	goto exit;

handle_event:
	/*
	 * From the interrupt and status register, find out
	 * which sensor caused the event, and for what transition
	 * [either 'Hot to Cold' or 'Cold to Hot']
	 */
	ret = update_intrpt_params(irq, level);
	if (ret < 0)
		goto exit_err;

	ret = intel_mid_pmic_writeb(reg, irq_cache);
	if (ret < 0)
		goto exit_err;

	schedule_work(&tdata->thermal_work);

	mutex_unlock(&tdata->thrm_irq_lock);
	return IRQ_HANDLED;

exit_err:
	pr_err("I2C read/write failed:%d\n", ret);
exit:
	mutex_unlock(&tdata->thrm_irq_lock);
	return IRQ_HANDLED;
}

static struct thermal_zone_device_ops tzd_emul_ops = {
	.get_temp = show_emul_temp,
	.set_emul_temp = store_emul_temp,
};

static struct thermal_zone_device_ops tzd_ops = {
	.get_temp = show_temp,
	.get_trip_type = show_trip_type,
	.get_trip_temp = show_trip_temp,
	.set_trip_temp = store_trip_temp,
	.get_trip_hyst = show_trip_hyst,
	.set_trip_hyst = store_trip_hyst,
};

static int byt_thermal_probe(struct platform_device *pdev)
{
	int i, size, ret;
	int total_sensors; /* real + virtual sensors */
	struct intel_mid_thermal_platform_data *pdata;
	struct cpuinfo_x86 *c = &cpu_data(0);

	pdata = pdev->dev.platform_data;
	if (!pdata) {
		dev_err(&pdev->dev, "Unable to fetch platform data\n");
		return -EINVAL;
	}

	tdata = kzalloc(sizeof(struct thermal_data), GFP_KERNEL);
	if (!tdata) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		return -ENOMEM;
	}

	tdata->pdev = pdev;
	tdata->num_sensors = pdata->num_sensors;
	tdata->num_virtual_sensors = pdata->num_virtual_sensors;
	tdata->sensors = pdata->sensors;
	tdata->irq = platform_get_irq(pdev, 0);
	platform_set_drvdata(pdev, tdata);
	mutex_init(&tdata->thrm_irq_lock);

	/*
	 * Identify whether this is VLV(0x37) or CHV(0x4c) board.
	 * TODO: Use PMIC registers on I2C space to differentiate this.
	 */
	if (c->x86_model == 0x37)
		tdata->is_vlv = true;
	else
		tdata->is_vlv = false;

	total_sensors = tdata->num_sensors;
#ifdef CONFIG_THERMAL_EMULATION
	total_sensors += tdata->num_virtual_sensors;
#endif
	size = sizeof(struct thermal_zone_device *) * total_sensors;
	tdata->tzd = kzalloc(size, GFP_KERNEL);
	if (!tdata->tzd) {
		dev_err(&pdev->dev, "kzalloc failed\n");
		ret = -ENOMEM;
		goto exit_free;
	}

	/* Disable prochot on alert0 crossing */
	ret = disable_prochot();
	if (ret) {
		dev_err(&pdev->dev, "Disabling prochot failed:%d\n", ret);
		goto exit_tzd;
	}

	/* Program a default _max value for each sensor */
	ret = program_tmax(&pdev->dev);
	if (ret) {
		dev_err(&pdev->dev, "Programming _max failed:%d\n", ret);
		goto exit_tzd;
	}

	/* Register with IIO to sample temperature values */
	tdata->iio_chan = iio_channel_get_all(&pdev->dev);
	if (tdata->iio_chan == NULL) {
		dev_err(&pdev->dev, "tdata->iio_chan is null\n");
		ret = -EINVAL;
		goto exit_tzd;
	}

	/* Check whether we got all the four channels */
	ret = iio_channel_get_num(tdata->iio_chan);
	if (ret != PMIC_THERMAL_SENSORS) {
		dev_err(&pdev->dev, "incorrect number of channels:%d\n", ret);
		ret = -EFAULT;
		goto exit_iio;
	}

	/* Register each sensor with the generic thermal framework */
	for (i = 0; i < total_sensors; i++) {
		if (i < tdata->num_sensors) {
			tdata->tzd[i] = thermal_zone_device_register(
					tdata->sensors[i].name,
					NUM_ALERT_LEVELS, ALERT_RW_MASK,
					initialize_sensor(i, &tdata->sensors[i]),
					&tzd_ops,
					NULL, 0, 0);
		} else {
			tdata->tzd[i] = thermal_zone_device_register(
					tdata->sensors[i].name,
					0, 0,
					initialize_sensor(i, &tdata->sensors[i]),
					&tzd_emul_ops,
					NULL, 0, 0);
		}
		if (IS_ERR(tdata->tzd[i])) {
			ret = PTR_ERR(tdata->tzd[i]);
			dev_err(&pdev->dev,
				"registering thermal sensor %s failed: %d\n",
				tdata->sensors[i].name, ret);
			goto exit_reg;
		}
	}

	INIT_WORK(&tdata->thermal_work, thermal_work_func);

	/* Register for Interrupt Handler */
	ret = request_threaded_irq(tdata->irq, NULL, thermal_intrpt,
					IRQF_TRIGGER_RISING,
					DEVICE_NAME, tdata);
	if (ret) {
		dev_err(&pdev->dev, "request_threaded_irq failed:%d\n", ret);
		goto exit_reg;
	}

	/* Enable Thermal Monitoring */
	ret = enable_tm();
	if (ret) {
		dev_err(&pdev->dev, "Enabling TM failed:%d\n", ret);
		goto exit_irq;
	}

	create_ccove_thermal_debugfs();

	return 0;

exit_irq:
	free_irq(tdata->irq, tdata);
exit_reg:
	while (--i >= 0)
		thermal_zone_device_unregister(tdata->tzd[i]);
exit_iio:
	iio_channel_release_all(tdata->iio_chan);
exit_tzd:
	kfree(tdata->tzd);
exit_free:
	mutex_destroy(&tdata->thrm_irq_lock);
	kfree(tdata);
	return ret;
}

static int byt_thermal_resume(struct device *dev)
{
	return 0;
}

static int byt_thermal_suspend(struct device *dev)
{
	return 0;
}

static int byt_thermal_remove(struct platform_device *pdev)
{
	int i, total_sensors;
	struct thermal_data *tdata = platform_get_drvdata(pdev);

	if (!tdata)
		return 0;

	total_sensors = tdata->num_sensors;

#ifdef CONFIG_THERMAL_EMULATION
	total_sensors += tdata->num_virtual_sensors;
#endif

	for (i = 0; i < total_sensors; i++)
		thermal_zone_device_unregister(tdata->tzd[i]);

	iio_channel_release_all(tdata->iio_chan);
	free_irq(tdata->irq, tdata);
	mutex_destroy(&tdata->thrm_irq_lock);
	kfree(tdata->tzd);
	kfree(tdata);

	remove_ccove_thermal_debugfs();

	return 0;
}

/*********************************************************************
 *		Driver initialization and finalization
 *********************************************************************/

static const struct dev_pm_ops thermal_pm_ops = {
	.suspend = byt_thermal_suspend,
	.resume = byt_thermal_resume,
};

static struct platform_driver byt_thermal_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.owner = THIS_MODULE,
		.pm = &thermal_pm_ops,
		},
	.probe = byt_thermal_probe,
	.remove = byt_thermal_remove,
};

static int byt_thermal_module_init(void)
{
	return platform_driver_register(&byt_thermal_driver);
}

static void byt_thermal_module_exit(void)
{
	platform_driver_unregister(&byt_thermal_driver);
}

late_initcall(byt_thermal_module_init);
module_exit(byt_thermal_module_exit);

MODULE_AUTHOR("Durgadoss R <durgadoss.r@intel.com>");
MODULE_DESCRIPTION("Intel Baytrail Platform Thermal Driver");
MODULE_LICENSE("GPL");
