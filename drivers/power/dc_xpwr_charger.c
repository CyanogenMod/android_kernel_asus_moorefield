/*
 * Intel Dollar Cove PMIC Charger driver
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
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/device.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/pm_runtime.h>
#include <linux/usb/otg.h>
#include <linux/notifier.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/notifier.h>
#include <linux/gpio.h>
#include <linux/acpi.h>
#include <linux/mfd/intel_mid_pmic.h>
#include <linux/power/dc_xpwr_charger.h>

#define DC_PS_STAT_REG			0x00
#define PS_STAT_VBUS_TRIGGER		(1 << 0)
#define PS_STAT_BAT_CHRG_DIR		(1 << 2)
#define PS_STAT_VBAT_ABOVE_VHOLD	(1 << 3)
#define PS_STAT_VBUS_VALID		(1 << 4)
#define PS_STAT_VBUS_PRESENT		(1 << 5)

#define DC_CHRG_STAT_REG		0x01
#define CHRG_STAT_BAT_SAFE_MODE		(1 << 3)
#define CHRG_STAT_BAT_VALID		(1 << 4)
#define CHRG_STAT_BAT_PRESENT		(1 << 5)
#define CHRG_STAT_CHARGING		(1 << 6)
#define CHRG_STAT_PMIC_OTP		(1 << 7)

#define DC_VBUS_ISPOUT_REG		0x30
#define VBUS_ISPOUT_CUR_LIM_MASK	0x03
#define VBUS_ISPOUT_CUR_LIM_BIT_POS	0
#define VBUS_ISPOUT_CUR_LIM_900MA	0x0	/* 900mA */
#define VBUS_ISPOUT_CUR_LIM_1500MA	0x1	/* 1500mA */
#define VBUS_ISPOUT_CUR_LIM_2000MA	0x2	/* 2000mA */
#define VBUS_ISPOUT_CUR_NO_LIM		0x3	/* 2500mA */
#define VBUS_ISPOUT_VHOLD_SET_MASK	0x31
#define VBUS_ISPOUT_VHOLD_SET_BIT_POS	0x3
#define VBUS_ISPOUT_VHOLD_SET_OFFSET	4000	/* 4000mV */
#define VBUS_ISPOUT_VHOLD_SET_LSB_RES	100	/* 100mV */
#define VBUS_ISPOUT_VHOLD_SET_4300MV	0x3	/* 4300mV */
#define VBUS_ISPOUT_VBUS_PATH_DIS	(1 << 7)

#define DC_CHRG_CCCV_REG		0x33
#define CHRG_CCCV_CC_MASK		0xf		/* 4 bits */
#define CHRG_CCCV_CC_BIT_POS		0
#define CHRG_CCCV_CC_OFFSET		200		/* 200mA */
#define CHRG_CCCV_CC_LSB_RES		200		/* 200mA */
#define CHRG_CCCV_ITERM_20P		(1 << 4)	/* 20% of CC */
#define CHRG_CCCV_CV_MASK		0x60		/* 2 bits */
#define CHRG_CCCV_CV_BIT_POS		5
#define CHRG_CCCV_CV_4100MV		0x0		/* 4.10V */
#define CHRG_CCCV_CV_4150MV		0x1		/* 4.15V */
#define CHRG_CCCV_CV_4200MV		0x2		/* 4.20V */
#define CHRG_CCCV_CV_4350MV		0x3		/* 4.35V */
#define CHRG_CCCV_CHG_EN		(1 << 7)

#define DC_CHRG_CNTL2_REG		0x34
#define CNTL2_CC_TIMEOUT_MASK		0x3	/* 2 bits */
#define CNTL2_CC_TIMEOUT_OFFSET		6	/* 6 Hrs */
#define CNTL2_CC_TIMEOUT_LSB_RES	2	/* 2 Hrs */
#define CNTL2_CC_TIMEOUT_12HRS		0x3	/* 12 Hrs */
#define CNTL2_CHGLED_TYPEB		(1 << 4)
#define CNTL2_CHG_OUT_TURNON		(1 << 5)
#define CNTL2_PC_TIMEOUT_MASK		0xC0
#define CNTL2_PC_TIMEOUT_OFFSET		40	/* 40 mins */
#define CNTL2_PC_TIMEOUT_LSB_RES	10	/* 10 mins */
#define CNTL2_PC_TIMEOUT_70MINS		0x3

#define DC_CHRG_ILIM_REG		0x35
#define CHRG_ILIM_TEMP_LOOP_EN		(1 << 3)
#define CHRG_ILIM_MASK			0xf0
#define CHRG_ILIM_BIT_POS		4
#define CHRG_ILIM_100MA			0x0	/* 100mA */
#define CHRG_ILIM_500MA			0x1	/* 500mA */
#define CHRG_ILIM_900MA			0x2	/* 900mA */
#define CHRG_ILIM_1500MA		0x3	/* 1500mA */
#define CHRG_ILIM_2000MA		0x4	/* 2000mA */
#define CHRG_ILIM_2500MA		0x5	/* 2500mA */
#define CHRG_ILIM_3000MA		0x6	/* 3000mA */

#define DC_CHRG_VLTFC_REG		0x38
#define CHRG_VLTFC_N5C			0xAA	/* 0 DegC */

#define DC_CHRG_VHTFC_REG		0x39
#define CHRG_VHTFC_60C			0x1E	/* 45 DegC */

#define DC_PWRSRC_IRQ_CFG_REG		0x40
#define PWRSRC_IRQ_CFG_VBUS_LOW		(1 << 2)
#define PWRSRC_IRQ_CFG_VBUS_HIGH	(1 << 3)
#define PWRSRC_IRQ_CFG_VBUS_OVP		(1 << 4)
#define PWRSRC_IRQ_CFG_SVBUS_LOW	(1 << 5)
#define PWRSRC_IRQ_CFG_SVBUS_HIGH	(1 << 6)
#define PWRSRC_IRQ_CFG_SVBUS_OVP	(1 << 7)

#define DC_BAT_IRQ_CFG_REG		0x41
#define BAT_IRQ_CFG_CHRG_DONE		(1 << 2)
#define BAT_IRQ_CFG_CHRG_START		(1 << 3)
#define BAT_IRQ_CFG_BAT_SAFE_EXIT	(1 << 4)
#define BAT_IRQ_CFG_BAT_SAFE_ENTER	(1 << 5)
#define BAT_IRQ_CFG_BAT_DISCON		(1 << 6)
#define BAT_IRQ_CFG_BAT_CONN		(1 << 7)
#define BAT_IRQ_CFG_BAT_MASK		0xFC

#define DC_TEMP_IRQ_CFG_REG		0x42
#define TEMP_IRQ_CFG_QCBTU		(1 << 4)
#define TEMP_IRQ_CFG_CBTU		(1 << 5)
#define TEMP_IRQ_CFG_QCBTO		(1 << 6)
#define TEMP_IRQ_CFG_CBTO		(1 << 7)
#define TEMP_IRQ_CFG_MASK		0xF0

#define DC_PWRSRC_IRQ_STAT_REG		0x48
#define PWRSRC_IRQ_STAT_VBUS_LOW	(1 << 2)
#define PWRSRC_IRQ_STAT_VBUS_HIGH	(1 << 3)
#define PWRSRC_IRQ_STAT_VBUS_OVP	(1 << 4)

#define DC_BAT_IRQ_STAT_REG		0x49
#define BAT_IRQ_STAT_CHRG_DONE		(1 << 2)
#define BAT_IRQ_STAT_CHARGING		(1 << 3)
#define BAT_IRQ_STAT_BAT_SAFE_EXIT	(1 << 4)
#define BAT_IRQ_STAT_BAT_SAFE_ENTER	(1 << 5)
#define BAT_IRQ_STAT_BAT_DISCON		(1 << 6)
#define BAT_IRQ_STAT_BAT_CONN		(1 << 7)

#define DC_TEMP_IRQ_STAT_REG		0x4A
#define TEMP_IRQ_STAT_QWBTU		(1 << 0)
#define TEMP_IRQ_STAT_WBTU		(1 << 1)
#define TEMP_IRQ_STAT_QWBTO		(1 << 2)
#define TEMP_IRQ_STAT_WBTO		(1 << 3)
#define TEMP_IRQ_STAT_QCBTU		(1 << 4)
#define TEMP_IRQ_STAT_CBTU		(1 << 5)
#define TEMP_IRQ_STAT_QCBTO		(1 << 6)
#define TEMP_IRQ_STAT_CBTO		(1 << 7)

#define DC_PMIC_IRQ_STAT_REG		0x4B
#define PMIC_IRQ_STAT_OTP		(1 << 7)

#define CV_4100				4100	/* 4100mV */
#define CV_4150				4150	/* 4150mV */
#define CV_4200				4200	/* 4200mV */
#define CV_4350				4350	/* 4350mV */

#define ILIM_100MA			100	/* 100mA */
#define ILIM_500MA			500	/* 500mA */
#define ILIM_900MA			900	/* 900mA */
#define ILIM_1500MA			1500	/* 1500mA */
#define ILIM_2000MA			2000	/* 2000mA */
#define ILIM_2500MA			2500	/* 2500mA */
#define ILIM_3000MA			3000	/* 3000mA */

#define DC_CHRG_INTR_NUM		9
#define FULL_THREAD_JIFFIES		(HZ * 30) /* 30sec */

#define RETRY_RW			3

#define DEV_NAME			"dollar_cove_charger"

enum {
	VBUS_OV_IRQ = 0,
	CHARGE_DONE_IRQ,
	CHARGE_CHARGING_IRQ,
	BAT_SAFE_QUIT_IRQ,
	BAT_SAFE_ENTER_IRQ,
	QCBTU_IRQ,
	CBTU_IRQ,
	QCBTO_IRQ,
	CBTO_IRQ,
};

struct pmic_chrg_info {
	struct platform_device *pdev;
	struct dollarcove_chrg_pdata	*pdata;
	int			irq[DC_CHRG_INTR_NUM];
	struct power_supply	psy_usb;
	struct mutex		lock;
	struct usb_phy		*otg;
	struct work_struct	otg_work;
	struct notifier_block	id_nb;
	bool			id_short;

	int chrg_health;
	int chrg_status;
	int cc;
	int cv;
	int inlmt;
	int max_cc;
	int max_cv;
	int iterm;
	int cable_type;
	int cntl_state;
	int max_temp;
	int min_temp;
	bool online;
	bool present;
	bool a_bus_enable;
	bool is_charging_enabled;
	bool is_charger_enabled;
	bool is_hw_chrg_term;
	struct delayed_work chrg_full_wrkr;
};

static enum power_supply_property pmic_chrg_usb_props[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_ENABLE_CHARGING,
	POWER_SUPPLY_PROP_ENABLE_CHARGER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CUR,
	POWER_SUPPLY_PROP_CABLE_TYPE,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_MAX_TEMP,
	POWER_SUPPLY_PROP_MIN_TEMP,
};

static struct pmic_chrg_info *pinfo;
static int probe_retry_cnt;

#ifdef CONFIG_BTNS_PMIC
#define MATCHED_OCV    4230
static struct power_supply *fg_psy;

/* check_batt_psy -check for whether power supply type is battery
 * @dev : Power Supply dev structure
 * @data : Power Supply Driver Data
 * Context: can sleep
 *
 * Return true if power supply type is battery
 *
 */
static int check_batt_psy(struct device *dev, void *data)
{
	struct power_supply *psy = dev_get_drvdata(dev);

	/* check for whether power supply type is battery */
	if (psy->type == POWER_SUPPLY_TYPE_BATTERY) {
		fg_psy = psy;
		return 1;
	}
	return 0;
}

/**
 * get_fg_chip_psy - identify the Fuel Gauge Power Supply device
 * Context: can sleep
 *
 * Return Fuel Gauge power supply structure
 */
static struct power_supply *get_fg_chip_psy(void)
{
	if (fg_psy)
		return fg_psy;

	/* loop through power supply class */
	class_for_each_device(power_supply_class, NULL, NULL,
		check_batt_psy);
	return fg_psy;
}

/**
 * fg_chip_get_property - read a power supply property from Fuel Gauge driver
 * @psp : Power Supply property
 *
 * Return power supply property value
 *
 */
static int fg_chip_get_property(enum power_supply_property psp)
{
	union power_supply_propval val;
	int ret = -ENODEV;

	if (!fg_psy)
		fg_psy = get_fg_chip_psy();
	if (fg_psy) {
		ret = fg_psy->get_property(fg_psy, psp, &val);
		if (!ret)
			return val.intval;
	}
	return ret;
}
#endif

static int pmic_chrg_reg_readb(struct pmic_chrg_info *info, int reg)
{
	int ret, i;

	for (i = 0; i < RETRY_RW; i++) {
		ret = intel_mid_pmic_readb(reg);
		if (ret < 0) {
			dev_warn(&info->pdev->dev,
				"failed to read reg 0x%x: %d\n", reg, ret);
			usleep_range(1000, 2000);
		} else
			break;
	}
	return ret;
}

static int pmic_chrg_reg_writeb(struct pmic_chrg_info *info, int reg, u8 val)
{
	int ret, i;

	for (i = 0; i < RETRY_RW; i++) {
		ret = intel_mid_pmic_writeb(reg, val);
		if (ret < 0) {
			dev_warn(&info->pdev->dev,
				"failed to write reg 0x%x: %d\n", reg, ret);
			usleep_range(1000, 2000);
		} else
			break;
	}

	return ret;
}

static int pmic_chrg_reg_setb(struct pmic_chrg_info *info, int reg, u8 mask)
{
	int ret, i;

	for (i = 0; i < RETRY_RW; i++) {
		ret = intel_mid_pmic_setb(reg, mask);
		if (ret < 0) {
			dev_warn(&info->pdev->dev,
				"failed to set reg 0x%x: %d\n", reg, ret);
			usleep_range(1000, 2000);
		} else
			break;
	}

	return ret;
}

static int pmic_chrg_reg_clearb(struct pmic_chrg_info *info, int reg, u8 mask)
{
	int ret, i;

	for (i = 0; i < RETRY_RW; i++) {
		ret = intel_mid_pmic_clearb(reg, mask);
		if (ret < 0) {
			dev_warn(&info->pdev->dev,
				"failed to clear reg 0x%x: %d\n", reg, ret);
			usleep_range(1000, 2000);
		} else
			break;
	}

	return ret;
}

static enum power_supply_type get_power_supply_type(
		enum power_supply_charger_cable_type cable)
{

	switch (cable) {

	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		return POWER_SUPPLY_TYPE_USB_DCP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		return POWER_SUPPLY_TYPE_USB_CDP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
	case POWER_SUPPLY_CHARGER_TYPE_ACA_DOCK:
		return POWER_SUPPLY_TYPE_USB_ACA;
	case POWER_SUPPLY_CHARGER_TYPE_AC:
		return POWER_SUPPLY_TYPE_MAINS;
	case POWER_SUPPLY_CHARGER_TYPE_NONE:
	case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
	default:
		return POWER_SUPPLY_TYPE_USB;
	}

	return POWER_SUPPLY_TYPE_USB;
}

static inline int pmic_chrg_set_cc(struct pmic_chrg_info *info, int cc)
{
	u8 reg_val;
	int ret, vocv;

	dev_info(&info->pdev->dev, "%s: cc=%d\n", __func__, cc);
	/* read CCCV register */
	ret = pmic_chrg_reg_readb(info, DC_CHRG_CCCV_REG);
	if (ret < 0)
		goto set_cc_fail;

	if (cc < CHRG_CCCV_CC_OFFSET)
		cc = CHRG_CCCV_CC_OFFSET;
	else if (cc > info->max_cc)
		cc = info->max_cc;

#ifdef CONFIG_BTNS_PMIC
	/* read the battery OCV */
	vocv = fg_chip_get_property(POWER_SUPPLY_PROP_VOLTAGE_OCV);
	if (vocv == -ENODEV || vocv == -EINVAL)
		dev_err(&info->pdev->dev, "Can't read battery OCV from FG\n");
	else if ((vocv/1000) > MATCHED_OCV)
		cc = CHRG_CCCV_CC_OFFSET;
#endif

	reg_val = (cc - CHRG_CCCV_CC_OFFSET) / CHRG_CCCV_CC_LSB_RES;
	reg_val = (ret & ~CHRG_CCCV_CC_MASK) | (reg_val << CHRG_CCCV_CC_BIT_POS);
	ret = pmic_chrg_reg_writeb(info, DC_CHRG_CCCV_REG, reg_val);

set_cc_fail:
	return ret;
}

static inline int pmic_chrg_set_cv(struct pmic_chrg_info *info, int cv)
{
	u8 reg_val;
	int ret;

	dev_info(&info->pdev->dev, "%s: cv=%d\n", __func__, cv);
	/* read CCCV register */
	ret = pmic_chrg_reg_readb(info, DC_CHRG_CCCV_REG);
	if (ret < 0)
		goto set_cv_fail;

	if (cv <= CV_4100)
		reg_val = CHRG_CCCV_CV_4100MV;
	else if (cv <= CV_4150)
		reg_val = CHRG_CCCV_CV_4150MV;
	else if (cv <= CV_4200)
		reg_val = CHRG_CCCV_CV_4200MV;
	else
		reg_val = CHRG_CCCV_CV_4350MV;

	reg_val = (ret & ~CHRG_CCCV_CV_MASK) |
				(reg_val << CHRG_CCCV_CV_BIT_POS);

	ret = pmic_chrg_reg_writeb(info, DC_CHRG_CCCV_REG, reg_val);

set_cv_fail:
	return ret;
}

static inline int pmic_chrg_set_inlmt(struct pmic_chrg_info *info, int inlmt)
{
	u8 reg_val;
	int ret;

	/* Read in limit register */
	ret = pmic_chrg_reg_readb(info, DC_CHRG_ILIM_REG);
	if (ret < 0)
		goto set_inlmt_fail;

	if (inlmt <= ILIM_100MA)
		reg_val = CHRG_ILIM_100MA;
	else if (inlmt <= ILIM_500MA)
		reg_val = CHRG_ILIM_500MA;
	else if (inlmt <= ILIM_900MA)
		reg_val = CHRG_ILIM_900MA;
	else if (inlmt <= ILIM_1500MA)
		reg_val = CHRG_ILIM_1500MA;
	else if (inlmt <= ILIM_2000MA)
		reg_val = CHRG_ILIM_2000MA;
	else if (inlmt <= ILIM_2500MA)
		reg_val = CHRG_ILIM_2500MA;
	else
		reg_val = CHRG_ILIM_3000MA;

	reg_val = (ret & ~CHRG_ILIM_MASK) | (reg_val << CHRG_ILIM_BIT_POS);
	ret = pmic_chrg_reg_writeb(info, DC_CHRG_ILIM_REG, reg_val);

set_inlmt_fail:
	return ret;
}

static int pmic_chrg_enable_charger(struct pmic_chrg_info *info, bool enable)
{
	int ret;

	dev_info(&info->pdev->dev, "%s: enable=%d\n", __func__, enable);
	if (enable)
		ret = pmic_chrg_reg_clearb(info,
			DC_VBUS_ISPOUT_REG, VBUS_ISPOUT_VBUS_PATH_DIS);
	else
		ret = pmic_chrg_reg_setb(info,
			DC_VBUS_ISPOUT_REG, VBUS_ISPOUT_VBUS_PATH_DIS);
	return ret;
}

static int pmic_chrg_enable_charging(struct pmic_chrg_info *info, bool enable)
{
	int ret;

	dev_info(&info->pdev->dev, "%s: enable=%d\n", __func__, enable);
	if (enable)
		ret = pmic_chrg_reg_setb(info,
			DC_CHRG_CCCV_REG, CHRG_CCCV_CHG_EN);
	else
		ret = pmic_chrg_reg_clearb(info,
			DC_CHRG_CCCV_REG, CHRG_CCCV_CHG_EN);
	return ret;
}

static int pmic_chrg_is_present(struct pmic_chrg_info *info)
{
	int ret, present = 0;

	ret = pmic_chrg_reg_readb(info, DC_PS_STAT_REG);
	if (ret < 0)
		return ret;

	if (ret & PS_STAT_VBUS_PRESENT)
		present = 1;
	return present;
}

static int pmic_chrg_is_online(struct pmic_chrg_info *info)
{
	int ret, online = 0;

	ret = pmic_chrg_reg_readb(info, DC_PS_STAT_REG);
	if (ret < 0)
		return ret;

	if (ret & PS_STAT_VBUS_VALID)
		online = 1;
	return online;
}

static int get_charger_health(struct pmic_chrg_info *info)
{
	int ret, pwr_stat, chrg_stat, pwr_irq;
	int health = POWER_SUPPLY_HEALTH_UNKNOWN;

	ret = pmic_chrg_reg_readb(info, DC_PS_STAT_REG);
	if ((ret < 0) || !(ret & PS_STAT_VBUS_PRESENT))
		goto health_read_fail;
	else
		pwr_stat = ret;

	ret = pmic_chrg_reg_readb(info, DC_CHRG_STAT_REG);
	if (ret < 0)
		goto health_read_fail;
	else
		chrg_stat = ret;

	ret = pmic_chrg_reg_readb(info, DC_PWRSRC_IRQ_STAT_REG);
	if (ret < 0)
		goto health_read_fail;
	else
		pwr_irq = ret;

	if (!(pwr_stat & PS_STAT_VBUS_VALID)) {
		health = POWER_SUPPLY_HEALTH_DEAD;
		info->is_charger_enabled = 0;
		/* Xpwr pmic doesnt have interuppt for VBUS undervoltage.
		 * hence trigger a power_supply_change when charger
		 * health chnaged to dead.
		 */
		if (info->chrg_health != health)
			power_supply_changed(&info->psy_usb);
	} else if (pwr_irq & PWRSRC_IRQ_CFG_SVBUS_OVP) {
		health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		info->is_charger_enabled = 0;
	}
	else if (chrg_stat & CHRG_STAT_PMIC_OTP)
		health = POWER_SUPPLY_HEALTH_OVERHEAT;
	else if (chrg_stat & CHRG_STAT_BAT_SAFE_MODE)
		health = POWER_SUPPLY_HEALTH_SAFETY_TIMER_EXPIRE;
	else if (!info->present)
		health = POWER_SUPPLY_HEALTH_UNKNOWN;
	else {
		health = POWER_SUPPLY_HEALTH_GOOD;
		info->is_charger_enabled = 1;
	}

health_read_fail:
	info->chrg_health = health;
	return health;
}

static void xpwr_full_worker(struct work_struct *work)
{
	struct pmic_chrg_info *info = container_of(work,
							struct pmic_chrg_info,
							chrg_full_wrkr.work);

	power_supply_changed(NULL);
	/* schedule the thread to let the framework know about FULL */
	schedule_delayed_work(&info->chrg_full_wrkr, FULL_THREAD_JIFFIES);
}

static int get_charging_status(struct pmic_chrg_info *info)
{
	int stat = POWER_SUPPLY_STATUS_UNKNOWN;
	int pwr_stat, chrg_stat, bat_irq_stat;

	pwr_stat = pmic_chrg_reg_readb(info, DC_PS_STAT_REG);
	if (pwr_stat < 0)
		goto chrg_stat_read_fail;

	chrg_stat = pmic_chrg_reg_readb(info, DC_CHRG_STAT_REG);
	if (chrg_stat < 0)
		goto chrg_stat_read_fail;

	bat_irq_stat = pmic_chrg_reg_readb(info, DC_BAT_IRQ_STAT_REG);
	if (bat_irq_stat < 0)
		goto chrg_stat_read_fail;

	if (bat_irq_stat & BAT_IRQ_STAT_BAT_DISCON)
		stat = POWER_SUPPLY_STATUS_UNKNOWN;
	else if (!(pwr_stat & PS_STAT_VBUS_PRESENT))
		stat = POWER_SUPPLY_STATUS_DISCHARGING;
	else if (bat_irq_stat & CHRG_STAT_CHARGING)
		stat = POWER_SUPPLY_STATUS_CHARGING;
	else if (bat_irq_stat & BAT_IRQ_STAT_CHRG_DONE)
		stat = POWER_SUPPLY_STATUS_FULL;
	else
		stat = POWER_SUPPLY_STATUS_NOT_CHARGING;

chrg_stat_read_fail:
	return stat;
}

static int pmic_chrg_usb_set_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    const union power_supply_propval *val)
{
	struct pmic_chrg_info *info = container_of(psy,
						    struct pmic_chrg_info,
						    psy_usb);
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		info->present = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		info->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		ret = pmic_chrg_enable_charging(info, val->intval);
		if (ret < 0)
			dev_warn(&info->pdev->dev, "enable charging failed\n");
		info->is_charging_enabled = val->intval;
		if (!val->intval)
			/* Cancel Full charge worker when disable charging */
			cancel_delayed_work_sync(&info->chrg_full_wrkr);
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		/*
		 * Disabling the VSYS or Charger is not
		 * allowing the PMIC to detect the subsequent
		 * USB plug events. For better to keep the
		 * default VBUS ON bit always to have better
		 * user experience. So commenting teh actual
		 * enable_charger() and for charger enable/disable
		 * we need to use teh same enable_charginig() to
		 * align with charegr framework.
		 *
		 * ret = pmic_chrg_enable_charger(info, val->intval);
		 */
		ret = pmic_chrg_enable_charging(info, val->intval);
		if (ret < 0)
			dev_warn(&info->pdev->dev, "enable charger failed\n");
		 info->is_charger_enabled = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		ret = pmic_chrg_set_cc(info, val->intval);
		if (ret < 0)
			dev_warn(&info->pdev->dev, "set inlimit failed\n");
		info->cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_INLMT:
		ret = pmic_chrg_set_inlmt(info, val->intval);
		if (ret < 0)
			dev_warn(&info->pdev->dev, "set inlimit failed\n");
		info->inlmt = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		ret = pmic_chrg_set_cv(info, val->intval);
		if (ret < 0)
			dev_warn(&info->pdev->dev, "set inlimit failed\n");
		info->cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		info->max_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		info->max_cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		info->iterm = val->intval;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		info->cable_type = val->intval;
		info->psy_usb.type = get_power_supply_type(info->cable_type);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		info->cntl_state = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		info->max_temp = val->intval;
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		info->min_temp = val->intval;
		break;
	default:
		ret = -EINVAL;
	}

	mutex_unlock(&info->lock);
	return ret;
}

static int pmic_chrg_usb_get_property(struct power_supply *psy,
				    enum power_supply_property psp,
				    union power_supply_propval *val)
{
	struct pmic_chrg_info *info = container_of(psy,
				struct pmic_chrg_info, psy_usb);
	int ret = 0;

	mutex_lock(&info->lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		/* check for OTG case first */
		if (info->id_short) {
			val->intval = 0;
			break;
		}
		ret = pmic_chrg_is_present(info);
		if (ret < 0)
			goto psy_get_prop_fail;
		val->intval = ret;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		/* check for OTG case first */
		if (info->id_short) {
			val->intval = 0;
			break;
		}
		val->intval = info->online;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = get_charger_health(info);
		/*
		 * X-Power Inlimit is getting set to default(500mA)
		 * whenever we hit the Charger UVP condition and the
		 * setting remains same even after UVP recovery.
		 * As a WA to make sure INLMT is intact, we are
		 * reprogramming the inlimit.
		 */
		if (val->intval == POWER_SUPPLY_HEALTH_GOOD) {
			ret = pmic_chrg_set_inlmt(info, info->inlmt);
			if (ret < 0)
				dev_warn(&info->pdev->dev, "set inlimit failed\n");
		}
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		val->intval = info->max_cc;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		val->intval = info->max_cv;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		val->intval = info->cc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		val->intval = info->cv;
		break;
	case POWER_SUPPLY_PROP_INLMT:
		val->intval = info->inlmt;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = info->iterm;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = info->cable_type;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		val->intval = info->is_charging_enabled;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = info->is_charger_enabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = info->cntl_state;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = info->pdata->num_throttle_states;
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		val->intval = info->max_temp;
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		val->intval = info->min_temp;
		break;
	default:
		mutex_unlock(&info->lock);
		return -EINVAL;
	}

psy_get_prop_fail:
	mutex_unlock(&info->lock);
	return ret;
}

static irqreturn_t pmic_chrg_thread_handler(int irq, void *dev)
{
	struct pmic_chrg_info *info = dev;
	int i;

	for (i = 0; i < DC_CHRG_INTR_NUM; i++) {
		if (info->irq[i] == irq)
			break;
	}

	if (i >= DC_CHRG_INTR_NUM) {
		dev_warn(&info->pdev->dev, "spurious interrupt!!\n");
		return IRQ_NONE;
	}

	switch (i) {
	case VBUS_OV_IRQ:
		dev_info(&info->pdev->dev, "VBUS Over Voltage INTR\n");
		break;
	case CHARGE_DONE_IRQ:
		dev_info(&info->pdev->dev, "Charging Done INTR\n");

		/* schedule the thread to let the framework know about FULL */
		schedule_delayed_work(&info->chrg_full_wrkr, 0);
		return IRQ_HANDLED;
	case CHARGE_CHARGING_IRQ:
		dev_info(&info->pdev->dev, "Start Charging IRQ\n");
		break;
	case QCBTU_IRQ:
		dev_info(&info->pdev->dev,
			"Quit Battery Under Temperature(CHRG) INTR\n");
		break;
	case BAT_SAFE_QUIT_IRQ:
		dev_info(&info->pdev->dev,
			"Quit Safe Mode(restart timer) Charging IRQ\n");
		break;
	case BAT_SAFE_ENTER_IRQ:
		dev_info(&info->pdev->dev,
			"Enter Safe Mode(timer expire) Charging IRQ\n");
		break;
	case CBTU_IRQ:
		dev_info(&info->pdev->dev,
			"Hit Battery Under Temperature(CHRG) INTR\n");
		break;
	case QCBTO_IRQ:
		dev_info(&info->pdev->dev,
			"Quit Battery Over Temperature(CHRG) INTR\n");
		break;
	case CBTO_IRQ:
		dev_info(&info->pdev->dev,
			"Hit Battery Over Temperature(CHRG) INTR\n");
		break;
	default:
		dev_warn(&info->pdev->dev, "Spurious Interrupt!!!\n");
	}

	power_supply_changed(&info->psy_usb);
	return IRQ_HANDLED;
}

static int dc_xpwr_turn_otg_vbus(struct pmic_chrg_info *info, bool votg_on)
{
	int ret;

	if (votg_on && info->a_bus_enable)
		ret = gpio_direction_output(info->pdata->otg_gpio, true);
	else
		ret = gpio_direction_output(info->pdata->otg_gpio, false);

	return ret;
}

static void dc_xpwr_otg_event_worker(struct work_struct *work)
{
	struct pmic_chrg_info *info =
	    container_of(work, struct pmic_chrg_info, otg_work);
	int ret;

	mutex_lock(&info->lock);
	/* disable VBUS path before enabling the 5V boost */
	ret = pmic_chrg_enable_charger(info, !info->id_short);
	if (ret < 0)
		dev_warn(&info->pdev->dev, "vbus path disable failed\n");

	if (info->pdata->otg_gpio >= 0) {
		ret = dc_xpwr_turn_otg_vbus(info, info->id_short);
		if (ret < 0)
			dev_err(&info->pdev->dev,
					"VBUS ON/OFF FAILED: %d\n",
					ret);
	}
	mutex_unlock(&info->lock);
}

static int dc_xpwr_handle_otg_event(struct notifier_block *nb,
				   unsigned long event, void *param)
{
	struct pmic_chrg_info *info =
	    container_of(nb, struct pmic_chrg_info, id_nb);
	int *val = (int *)param;

	if (!val || (event != USB_EVENT_ID))
		return NOTIFY_DONE;

	dev_info(&info->pdev->dev,
		"[OTG notification]evt:%lu val:%d\n", event, *val);

	switch (event) {
	case USB_EVENT_ID:
		/*
		 * in case of ID short(*id = 0)
		 * enable vbus else disable vbus.
		 */
		if (*val)
			info->id_short = false;
		else
			info->id_short = true;
		schedule_work(&info->otg_work);
		break;
	default:
		dev_warn(&info->pdev->dev, "invalid OTG event\n");
	}

	return NOTIFY_OK;
}

static int pmic_chrg_init_hw_regs(struct pmic_chrg_info *info)
{
	int ret;
	/* program temperature thresholds */
	ret = pmic_chrg_reg_writeb(info, DC_CHRG_VLTFC_REG, CHRG_VLTFC_N5C);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to program low temp thresholds\n");
		return ret;
	}
	ret = pmic_chrg_reg_writeb(info, DC_CHRG_VHTFC_REG, CHRG_VHTFC_60C);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to program high temp thresholds\n");
		return ret;
	}

#ifdef CONFIG_BTNS_PMIC
	/* Turn-off charger o/p after charge cycle ends */
	ret = pmic_chrg_reg_clearb(info, DC_CHRG_CNTL2_REG, CNTL2_CHG_OUT_TURNON);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to Turn-off charger\n");
		return ret;
	}

	/* set the Charge end condition to 10% of CC */
	ret = pmic_chrg_reg_clearb(info, DC_CHRG_CCCV_REG, CHRG_CCCV_ITERM_20P);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to program Iterm as 10\%\n");
		return ret;
	}
#else
	/* do not turn-off charger o/p after charge cycle ends.
	 * Set charge timer as 12Hrs.
	 */
	ret = pmic_chrg_reg_setb(info, DC_CHRG_CNTL2_REG,
			CNTL2_CHG_OUT_TURNON | CNTL2_CC_TIMEOUT_12HRS);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to program CNTL2 reg\n");
		return ret;
	}

	/* set the Charge end condition to 20% of CC */
	ret = pmic_chrg_reg_setb(info, DC_CHRG_CCCV_REG, CHRG_CCCV_ITERM_20P);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to program Iterm as 20\%\n");
		return ret;
	}
#endif
	/* enable interrupts */
	ret = pmic_chrg_reg_setb(info, DC_BAT_IRQ_CFG_REG, BAT_IRQ_CFG_BAT_MASK);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to program IRQ batt config\n");
		return ret;
	}
	ret = pmic_chrg_reg_setb(info, DC_TEMP_IRQ_CFG_REG, TEMP_IRQ_CFG_MASK);
	if (ret < 0) {
		dev_err(&info->pdev->dev, "Failed to program IRQ temp condig\n");
		return ret;
	}
	return 0;
}

static void pmic_chrg_init_psy_props(struct pmic_chrg_info *info)
{
	info->max_cc = info->pdata->max_cc;
	info->max_cv = info->pdata->max_cv;
}

static void pmic_chrg_init_irq(struct pmic_chrg_info *info)
{
	int ret, i;

	for (i = 0; i < DC_CHRG_INTR_NUM; i++) {
		info->irq[i] = platform_get_irq(info->pdev, i);
		ret = request_threaded_irq(info->irq[i],
				NULL, pmic_chrg_thread_handler,
				IRQF_ONESHOT, DEV_NAME, info);
		if (ret) {
			dev_warn(&info->pdev->dev,
				"cannot get IRQ:%d\n", info->irq[i]);
			info->irq[i] = -1;
			goto intr_failed;
		} else {
			dev_info(&info->pdev->dev, "IRQ No:%d\n", info->irq[i]);
		}
	}

	return;

intr_failed:
	for (; i > 0; i--) {
		free_irq(info->irq[i - 1], info);
		info->irq[i - 1] = -1;
	}
}

static void dc_xpwr_usb_otg_enable(struct usb_phy *phy)
{
	struct pmic_chrg_info *info;
	int ret, id_value = -1;

	if (!pinfo)
		return;

	info = pinfo;
	mutex_lock(&info->lock);
	if (phy->vbus_state == VBUS_DISABLED)
		info->a_bus_enable = false;
	else
		info->a_bus_enable = true;
	mutex_unlock(&info->lock);

	if (phy->get_id_status) {
		/* get_id_status will store 0 in id_value if otg device is connected */
		ret = phy->get_id_status(phy, &id_value);
		if (ret < 0) {
			dev_warn(&info->pdev->dev,
					"otg get ID status failed:%d\n", ret);
			return;
		} else if (id_value != 0) {
			/* otg vbus should not be enabled or disabled if otg device is not connected */
			return;
		}
	} else {
		dev_err(&info->pdev->dev, "get_id_status is not defined for usb_phy");
		return;
	}

	mutex_lock(&info->lock);
	if (phy->vbus_state == VBUS_DISABLED) {
		dev_info(&info->pdev->dev, "OTG Disable");
		ret = dc_xpwr_turn_otg_vbus(info, false);
		if (ret < 0)
			dev_err(&info->pdev->dev, "VBUS OFF FAILED:\n");
	} else {
		dev_info(&info->pdev->dev, "OTG Enable");
		ret = dc_xpwr_turn_otg_vbus(info, true);
		if (ret < 0)
			dev_err(&info->pdev->dev, "VBUS ON FAILED:\n");
	}
	mutex_unlock(&info->lock);
}

static int pmic_chrg_probe(struct platform_device *pdev)
{
	struct pmic_chrg_info *info;
	int ret, i;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info) {
		dev_err(&pdev->dev, "mem alloc failed\n");
		return -ENOMEM;
	}

	info->pdev = pdev;
	info->pdata = pdev->dev.platform_data;
	if (!info->pdata)
		return -ENODEV;

	mutex_init(&info->lock);
	INIT_WORK(&info->otg_work, dc_xpwr_otg_event_worker);

	pmic_chrg_init_psy_props(info);
	info->a_bus_enable = true;

	platform_set_drvdata(pdev, info);
	pinfo = info;

	/* Register for OTG notification */
	info->otg = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!info->otg) {
		dev_warn(&pdev->dev, "Failed to get otg transceiver!!\n");
	} else {
		info->id_nb.notifier_call = dc_xpwr_handle_otg_event;
		info->otg->a_bus_drop = dc_xpwr_usb_otg_enable;
		ret = usb_register_notifier(info->otg, &info->id_nb);
		if (ret)
			dev_err(&pdev->dev, "failed to register otg notifier\n");
	}

	info->psy_usb.name = DEV_NAME;
	info->psy_usb.type = POWER_SUPPLY_TYPE_USB;
	info->psy_usb.properties = pmic_chrg_usb_props;
	info->psy_usb.num_properties = ARRAY_SIZE(pmic_chrg_usb_props);
	info->psy_usb.get_property = pmic_chrg_usb_get_property;
	info->psy_usb.set_property = pmic_chrg_usb_set_property;
	info->psy_usb.supplied_to = info->pdata->supplied_to;
	info->psy_usb.num_supplicants = info->pdata->num_supplicants;
	info->psy_usb.throttle_states = info->pdata->throttle_states;
	info->psy_usb.num_throttle_states = info->pdata->num_throttle_states;
	info->psy_usb.supported_cables = info->pdata->supported_cables;
	ret = power_supply_register(&pdev->dev, &info->psy_usb);
	if (ret) {
		dev_err(&pdev->dev, "Failed: power supply register (%d)\n",
			ret);
		goto psy_reg_failed;
	}

	pmic_chrg_init_irq(info);
	ret = pmic_chrg_init_hw_regs(info);
	if (ret < 0) {
		dev_err(&pdev->dev, "Failed to initialize the charge registers, ret=%d\n", ret);
		goto chrg_init_failed;
	}

	INIT_DELAYED_WORK(&info->chrg_full_wrkr, xpwr_full_worker);

	return 0;

chrg_init_failed:
	probe_retry_cnt++;
	/* XPWR pmic will give read write error if busy with other R/W operations.
	 * Defer probe if charger register initialization failed.
	 */
	if (probe_retry_cnt < RETRY_RW)
		ret = -EPROBE_DEFER;
	/* Free IRQs */
	for (i = 0; i < DC_CHRG_INTR_NUM && info->irq[i] != -1; i++)
		free_irq(info->irq[i], info);
	if (info->otg)
		usb_put_phy(info->otg);
	power_supply_unregister(&info->psy_usb);
psy_reg_failed:
	return ret;
}

static int pmic_chrg_remove(struct platform_device *pdev)
{
	struct pmic_chrg_info *info =  dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < DC_CHRG_INTR_NUM && info->irq[i] != -1; i++)
		free_irq(info->irq[i], info);
	if (info->otg)
		usb_put_phy(info->otg);
	power_supply_unregister(&info->psy_usb);
	return 0;
}

static int pmic_chrg_suspend(struct device *dev)
{
	return 0;
}

static int pmic_chrg_resume(struct device *dev)
{
	return 0;
}

static int pmic_chrg_runtime_suspend(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int pmic_chrg_runtime_resume(struct device *dev)
{
	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static int pmic_chrg_runtime_idle(struct device *dev)
{

	dev_dbg(dev, "%s called\n", __func__);
	return 0;
}

static const struct dev_pm_ops pmic_chrg_pm_ops = {
		SET_SYSTEM_SLEEP_PM_OPS(pmic_chrg_suspend,
				pmic_chrg_resume)
		SET_RUNTIME_PM_OPS(pmic_chrg_runtime_suspend,
				pmic_chrg_runtime_resume,
				pmic_chrg_runtime_idle)
};

static struct platform_driver pmic_chrg_driver = {
	.driver = {
		.name = DEV_NAME,
		.owner	= THIS_MODULE,
		.pm = &pmic_chrg_pm_ops,
	},
	.probe = pmic_chrg_probe,
	.remove = pmic_chrg_remove,
};

static int __init dc_pmic_chrg_init(void)
{
	return platform_driver_register(&pmic_chrg_driver);

}
device_initcall(dc_pmic_chrg_init);

static void __exit dc_pmic_chrg_exit(void)
{
	platform_driver_unregister(&pmic_chrg_driver);
}
module_exit(dc_pmic_chrg_exit);

MODULE_AUTHOR("Ramakrishna Pallala <ramakrishna.pallala@intel.com>");
MODULE_DESCRIPTION("Dollar Cove Xpower PMIC Charger Driver");
MODULE_LICENSE("GPL");
