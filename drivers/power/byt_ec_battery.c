/*
 * byt_ec_battery.c - Baytrail EC based battery driver
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
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 * Author: Ramakrishna Pallala <ramakrishna.pallala@intel.com>
 */

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/jiffies.h>
#include <linux/io.h>
#include <linux/slab.h>
#include <linux/pm_runtime.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/notifier.h>
#include <linux/workqueue.h>
#include <linux/power_supply.h>
#include <linux/acpi.h>
#include <asm/intel_byt_ec.h>

/* 8 bit registers and offsets */
#define EC_MAX_PLAT_TEMP_REG		1
#define EC_SENSOR_TEMP_REG		2
#define EC_REAL_AC_PWR_REG		3
#define REAL_AC_PWR_ON			0x01

#define EC_CRIT_TEMP_REG		47
#define EC_VIRT_AC_PWR_REG		48
#define EC_BAT0_STAT_REG		50
#define BAT0_STAT_DISCHARGING		(1 << 0)
#define BAT0_STAT_CHARGING		(1 << 1)
#define BAT0_STAT_LOW_BATT		(1 << 2)
#define BAT0_STAT_BATT_PRESENT		(1 << 3)
#define EC_BAT0_CUR_RATE_REG		51
#define EC_BAT0_CUR_CAP_REG		52
#define EC_BAT0_VOLT_REG		53

/* 16 bit registers and offsets */
#define EC_BAT0_DESIGN_CAP_REG		87
#define EC_BAT0_REM_CAP_REG		89
#define EC_BAT0_FULL_CHRG_CAP_REG	91
#define EC_BAT0_VBATT_REG		93
#define EC_BAT0_DIBATT_CUR_REG		95
#define EC_BAT0_CIBATT_CUR_REG		97
#define EC_BAT0_BATTSBS_STATUS		202
#define BAT0_BATTSBS_STATUS_OTP		(1 << 12)
#define BAT0_BATTSBS_STATUS_FC		(1 << 5)
#define BAT0_BATTSBS_STATUS_INIT	(1 << 7)
#define EC_BAT0_TEMP_REG		206
#define EC_BAT0_AVG_CUR_REG		208

/* 6% is minimun threshold  for platform shutdown*/
#define EC_BAT_SAFE_MIN_CAPACITY	6
#define EC_BAT_DEAD_VOLTAGE		6550  /* Dead = 6.55V */

/* Battery temperature related macros*/
#define EC_BAT_TEMP_CONV_FACTOR		27315 /* K = C + 273.15*/
/* Battery operating temperature range is 0C to 40C while
 * charging and -20 C to 60C while Discharging/Full/Not Chraging.
 */
#define EC_BAT_CHRG_OVER_TEMP		3131  /* 313.1K ~ 40 C */
#define EC_BAT_CHRG_UNDER_TEMP		2731  /* 273.1K ~ 0 C  */


struct ec_battery_info {
	struct platform_device *pdev;
	struct power_supply	bat;
	struct power_supply	chrg;
	struct mutex		lock;
	struct notifier_block	nb;
};

static enum power_supply_property ec_battery_props[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_CURRENT_AVG,
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_TEMP,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_CHARGE_FULL,
	POWER_SUPPLY_PROP_CHARGE_NOW,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
};

static enum power_supply_property ec_charger_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MODEL_NAME,
	POWER_SUPPLY_PROP_MANUFACTURER,
};

static short adjust_sign_value(int value)
{
	short result, temp = (short)value;

	/* check if sign bit is set */
	if (temp & 0x8000) {
		result = ~temp;
		result++;
		result *= -1;
	} else {
		result = temp;
	}

	return result;
}

static int ec_get_battery_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct ec_battery_info *chip = container_of(psy,
				struct ec_battery_info, bat);
	int ret = 0, cur_sign = -1;
	int comp_cap = 0;
	u8 val8, cap;
	u16 val16;

	mutex_lock(&chip->lock);
	switch (psp) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = byt_ec_read_byte(EC_BAT0_CUR_CAP_REG, &cap);
		if (ret < 0)
			goto ec_read_err;

		ret = byt_ec_read_byte(EC_BAT0_STAT_REG, &val8);
		if (ret < 0)
			goto ec_read_err;

		ret = byt_ec_read_word(EC_BAT0_BATTSBS_STATUS, &val16);
		if (ret < 0)
			goto ec_read_err;

		if (val8 & BAT0_STAT_DISCHARGING)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if (val8 & BAT0_STAT_CHARGING)
			val->intval = POWER_SUPPLY_STATUS_CHARGING;
		else if (val8 & BAT0_STAT_LOW_BATT)
			val->intval = POWER_SUPPLY_STATUS_DISCHARGING;
		else if ((cap == 100) || (val16 & BAT0_BATTSBS_STATUS_FC))
			val->intval = POWER_SUPPLY_STATUS_FULL;
		else if (cap != 0)
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		else
			val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		ret = byt_ec_read_byte(EC_BAT0_STAT_REG, &val8);
		if (ret < 0)
			goto ec_read_err;
		/* If battery is not connected
		 * return POWER_SUPPLY_HEALTH_UNKNOWN
		 */
		if (!(val8 & BAT0_STAT_BATT_PRESENT)) {
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
			break;
		}

		/* Battery operational temperature range is
		 * 0C to 40C while charging and -20 C to 60C
		 * while Discharging/Full/Not Charging.
		 */
		ret = byt_ec_read_word(EC_BAT0_TEMP_REG, &val16);
		if (ret < 0)
			goto ec_read_err;

		/* Android framwork don't differentiate b/w hot and cold.
		 *Both are treated as  overheat cases.
		 *So report overheat in both high and low temp cases.
		 */
		if ((val16 >= EC_BAT_CHRG_OVER_TEMP)
			| (val16 <= EC_BAT_CHRG_UNDER_TEMP)) {
			val->intval = POWER_SUPPLY_HEALTH_OVERHEAT;
			break;
		}
		ret = byt_ec_read_word(EC_BAT0_VBATT_REG, &val16);
		if (ret < 0)
			goto ec_read_err;
		/* The battery is treated as DEAD if the battery voltage is
		 *bellow <= 6.55V in BYT-M
		 */
		if (val16 <= EC_BAT_DEAD_VOLTAGE) {
			val->intval = POWER_SUPPLY_HEALTH_DEAD;
			break;
		}

		val->intval = POWER_SUPPLY_HEALTH_GOOD;
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = byt_ec_read_word(EC_BAT0_VBATT_REG, &val16);
		if (ret < 0)
			goto ec_read_err;
		val->intval = val16 * 1000;
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = byt_ec_read_byte(EC_BAT0_STAT_REG, &val8);
		if (ret < 0)
			goto ec_read_err;

		if (val8 & BAT0_STAT_CHARGING) {
			ret = byt_ec_read_word(EC_BAT0_CIBATT_CUR_REG, &val16);
			cur_sign = 1;
		} else {
			ret = byt_ec_read_word(EC_BAT0_DIBATT_CUR_REG, &val16);
			cur_sign = -1;
		}
		if (ret < 0)
			goto ec_read_err;
		val->intval = cur_sign * ((int)val16 * 1000);
		break;
	case POWER_SUPPLY_PROP_CURRENT_AVG:
		ret = byt_ec_read_word(EC_BAT0_AVG_CUR_REG, &val16);
		if (ret < 0)
			goto ec_read_err;

		val->intval = (int)adjust_sign_value(val16) * 1000;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		ret = byt_ec_read_byte(EC_BAT0_CUR_CAP_REG, &cap);
		if (ret < 0)
			goto ec_read_err;

		ret = byt_ec_read_byte(EC_BAT0_STAT_REG, &val8);
		if (ret < 0)
			goto ec_read_err;

		if ((val8 & 0x7) || (cap != 0))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = byt_ec_read_byte(EC_BAT0_CUR_CAP_REG, &val8);
		if (ret < 0)
			goto ec_read_err;
		/* 6% of battery capacity is minimun treshold for BYT-M
		 *So, the 6% is mapped to 0% in android.
		 * 6% to 100% is compensated with 0% to 100% to OS.
		 * Compensated capacity = cap - ((100 - cap)*6)/100 + 0.5
		 */
		comp_cap = val8;
		comp_cap = comp_cap*100 - ((100 - comp_cap)
				*EC_BAT_SAFE_MIN_CAPACITY) + 50;
		comp_cap /= 100;
		if (comp_cap < 0)
			comp_cap = 0;
		val->intval = comp_cap;
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = byt_ec_read_word(EC_BAT0_TEMP_REG, &val16);
		if (ret < 0)
			goto ec_read_err;
		/*
		 * convert temperature from degree kelvin
		 * to degree celsius: T(C) = T(K) - 273.15
		 * also 1 LSB of Temp register = 0.1 Kelvin.
		 */
		val->intval = (((int)val16 * 10) -
			EC_BAT_TEMP_CONV_FACTOR) / 10;
		break;
	case POWER_SUPPLY_PROP_TECHNOLOGY:
		ret = byt_ec_read_byte(EC_BAT0_STAT_REG, &val8);
		if (ret < 0)
			goto ec_read_err;
		/*If battery is not connected
		 *return POWER_SUPPLY_TECHNOLOGY_UNKNOWN
		 */
		if (val8 & BAT0_STAT_BATT_PRESENT)
			val->intval = POWER_SUPPLY_TECHNOLOGY_LION;
		else
			val->intval =   POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_CHARGE_NOW:
		ret = byt_ec_read_word(EC_BAT0_REM_CAP_REG, &val16);
		if (ret < 0)
			goto ec_read_err;
		val->intval = ((int)val16) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL:
		ret = byt_ec_read_word(EC_BAT0_FULL_CHRG_CAP_REG, &val16);
		if (ret < 0)
			goto ec_read_err;
		val->intval = ((int)val16) * 1000;
		break;
	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		ret = byt_ec_read_word(EC_BAT0_DESIGN_CAP_REG, &val16);
		if (ret < 0)
			goto ec_read_err;
		val->intval = ((int)val16) * 1000;
		break;
	default:
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}

	mutex_unlock(&chip->lock);
	return 0;

ec_read_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static int ec_get_charger_property(struct power_supply *psy,
					enum power_supply_property psp,
					union power_supply_propval *val)
{
	struct ec_battery_info *chip = container_of(psy,
				struct ec_battery_info, chrg);
	int ret = 0;
	int chrg_present = 0;
	u8 data;

	mutex_lock(&chip->lock);
	ret = byt_ec_read_byte(EC_REAL_AC_PWR_REG, &data);
	if (ret < 0)
		goto ec_read_err;
	if (data & REAL_AC_PWR_ON)
		chrg_present = 1;
	/* As EC provides only charger present status, all other parameters
	 * are hardcoded as per requirements.
	 */
	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:
	case POWER_SUPPLY_PROP_PRESENT:

		if (chrg_present)
			val->intval = 1;
		else
			val->intval = 0;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		if (chrg_present)
			val->intval = POWER_SUPPLY_HEALTH_GOOD;
		else
			val->intval = POWER_SUPPLY_HEALTH_UNKNOWN;
		break;
	case POWER_SUPPLY_PROP_MODEL_NAME:
		if (chrg_present)
			val->strval = "INBYTM";
		else
			val->strval = "Unknown";
		break;
	case POWER_SUPPLY_PROP_MANUFACTURER:
		if (chrg_present)
			val->strval = "Intel";
		else
			val->strval = "Unknown";
		break;

	default:
		mutex_unlock(&chip->lock);
		return -EINVAL;
	}
	mutex_unlock(&chip->lock);
	return 0;

ec_read_err:
	mutex_unlock(&chip->lock);
	return ret;
}

static int byt_ec_evt_batt_callback(struct notifier_block *nb,
					unsigned long event, void *data)
{
	struct ec_battery_info *chip = container_of(nb,
					struct ec_battery_info, nb);
	int ret = NOTIFY_DONE;
	bool chrg_uevent = false, batt_uevent = false;

	switch (event) {
	case BYT_EC_SCI_ACINSERTION:
		dev_info(&chip->pdev->dev, "Charger plug event\n");
		chrg_uevent = true;
		break;
	case BYT_EC_SCI_ACREMOVAL:
		dev_info(&chip->pdev->dev, "Charger unplug event\n");
		chrg_uevent = true;
		break;
	case BYT_EC_SCI_BATTERY:
		dev_info(&chip->pdev->dev, "Battery related event\n");
		batt_uevent = true;
		break;
	case BYT_EC_SCI_BATTERY_PRSNT:
		dev_info(&chip->pdev->dev, "Battery plug/unplug event\n");
		batt_uevent = true;
		break;
	case BYT_EC_SCI_BATTERY_OTP:
		/*Battery temp >40 is considered as over temperature*/
		dev_info(&chip->pdev->dev, "Battery over temp event\n");
		batt_uevent = true;
		break;
	case BYT_EC_SCI_BATTERY_OTP_CLR:
		/*Battery temp <= 39 will clear over temperature*/
		dev_info(&chip->pdev->dev, "Battery over temp clear event\n");
		batt_uevent = true;
		break;
	case BYT_EC_SCI_BATTERY_ETP:
		/*Battery temp > 60 is considered as extreme temperature*/
		dev_info(&chip->pdev->dev, "Battery extreme temp event\n");
		batt_uevent = true;
		break;
	case BYT_EC_SCI_BATTERY_ETP_CLR:
		/*Battery temp <= 59 will clear extreme temperature*/
		dev_info(&chip->pdev->dev, "Battery extreme temp clear event\n");
		batt_uevent = true;
		break;
	default:
		dev_dbg(&chip->pdev->dev, "not valid battery event\n");
	}

	if (chrg_uevent) {
		power_supply_changed(&chip->chrg);
		power_supply_changed(&chip->bat);
		ret = NOTIFY_OK;
	} else if (batt_uevent) {
		power_supply_changed(&chip->bat);
		ret = NOTIFY_OK;
	} else {
		ret = NOTIFY_DONE;
	}

	return ret;
}

static int ec_battery_probe(struct platform_device *pdev)
{
	struct ec_battery_info *chip;
	int ret;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip) {
		dev_err(&pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	chip->pdev = pdev;
	platform_set_drvdata(pdev, chip);
	mutex_init(&chip->lock);

	chip->bat.name = "byt_battery";
	chip->bat.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->bat.properties = ec_battery_props;
	chip->bat.num_properties = ARRAY_SIZE(ec_battery_props);
	chip->bat.get_property = ec_get_battery_property;
	ret = power_supply_register(&pdev->dev, &chip->bat);
	if (ret) {
		dev_err(&pdev->dev, "failed to register battery: %d\n", ret);
		goto probe_failed_1;
	}

	chip->chrg.name = "byt_charger";
	chip->chrg.type = POWER_SUPPLY_TYPE_MAINS;
	chip->chrg.properties = ec_charger_properties;
	chip->chrg.num_properties = ARRAY_SIZE(ec_charger_properties);
	chip->chrg.get_property = ec_get_charger_property;
	ret = power_supply_register(&pdev->dev, &chip->chrg);
	if (ret) {
		dev_err(&pdev->dev, "failed to register charger: %d\n", ret);
		goto probe_failed_2;
	}

	/* register for EC SCI events */
	chip->nb.notifier_call = &byt_ec_evt_batt_callback;
	byt_ec_evt_register_notify(&chip->nb);

	return 0;

probe_failed_2:
	power_supply_unregister(&chip->bat);
probe_failed_1:
	kfree(chip);
	return ret;
}

static int ec_battery_remove(struct platform_device *pdev)
{
	struct ec_battery_info *chip = platform_get_drvdata(pdev);

	byt_ec_evt_unregister_notify(&chip->nb);
	power_supply_unregister(&chip->chrg);
	power_supply_unregister(&chip->bat);
	kfree(chip);
	return 0;
}

static struct platform_driver ec_battery_driver = {
	.probe = ec_battery_probe,
	.remove = ec_battery_remove,
	.driver = {
		.name = "ec_battery",
	},
};

static int __init ec_battery_init(void)
{
	int ret = 0;

	ret = platform_driver_register(&ec_battery_driver);
	if (ret < 0) {
		pr_err("platform driver reg failed %s\n",
					"ec_battery");
		return ret;
	}
	return 0;
}
module_init(ec_battery_init);

static void __exit ec_battery_exit(void)
{
	platform_driver_unregister(&ec_battery_driver);
}
module_exit(ec_battery_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Baytrail EC Battery Driver");
MODULE_LICENSE("GPL");
