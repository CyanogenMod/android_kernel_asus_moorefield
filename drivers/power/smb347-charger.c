/*
 * Summit Microelectronics SMB347 Battery Charger Driver
 *
 * Copyright (C) 2011, Intel Corporation
 *
 * Authors: Bruce E. Robertson <bruce.e.robertson@intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/debugfs.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/i2c.h>
#include <linux/mutex.h>
#include <linux/power_supply.h>
#include <linux/workqueue.h>
#include <linux/power/smb347-charger.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/phy.h>
#include <linux/notifier.h>
#include <linux/extcon.h>
#include <linux/acpi.h>
#include <linux/acpi_gpio.h>
#include <linux/wakelock.h>
#include <linux/regulator/of_regulator.h>
#include <linux/regulator/consumer.h>
/*
 * Configuration registers. These are mirrored to volatile RAM and can be
 * written once %CMD_A_ALLOW_WRITE is set in %CMD_A register. They will be
 * reloaded from non-volatile registers after POR.
 */
#define CFG_CHARGE_CURRENT			0x00
#define CFG_CHARGE_CURRENT_FCC_MASK		0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT		5
#define CFG_CHARGE_CURRENT_PCC_MASK		0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT		3
#define CFG_CHARGE_CURRENT_TC_MASK		0x07
#define CFG_CURRENT_LIMIT			0x01
#define CFG_CURRENT_LIMIT_DC_MASK		0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT		4
#define CFG_CURRENT_LIMIT_USB_MASK		0x0f
#define CFG_VARIOUS_FUNCS			0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB		BIT(2)
#define CFG_FLOAT_VOLTAGE			0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK	0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT	6
#define CFG_CHRG_CONTROL			0x4
#define CFG_CHRG_CTRL_HW_TERM			BIT(6)
#define CFG_STAT				0x05
#define CFG_STAT_DISABLED			BIT(5)
#define CFG_STAT_ACTIVE_HIGH			BIT(7)
#define CFG_PIN					0x06
#define CFG_PIN_EN_CTRL_MASK			0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH		0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW		0x60
#define CFG_PIN_EN_APSD_IRQ			BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR		BIT(2)
#define CFG_THERM				0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK	0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT	0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK	0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT	2
#define CFG_THERM_MONITOR_DISABLED		BIT(4)
#define CFG_SYSOK				0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED	BIT(2)
#define CFG_OTHER				0x09
#define CFG_OTHER_RID_MASK			0xc0
#define CFG_OTHER_RID_DISABLED_OTG_I2C		0x00
#define CFG_OTHER_RID_DISABLED_OTG_PIN		0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C		0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG		0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW		BIT(5)
#define CFG_OTG					0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK		0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT		4
#define CFG_OTG_CC_COMPENSATION_MASK		0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT		6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK	0x03
#define CFG_TEMP_LIMIT				0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK		0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT		0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK		0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT		2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK		0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT		4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK		0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT		6
#define CFG_FAULT_IRQ				0x0c
#define CFG_FAULT_IRQ_DCIN_UV			BIT(2)
#define CFG_FAULT_IRQ_OTG_UV			BIT(5)
#define CFG_STATUS_IRQ				0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT		BIT(7)
#define CFG_STATUS_OTG_DET			BIT(6)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER	BIT(4)
#define CFG_STATUS_IRQ_INOK			BIT(2)
#define CFG_ADDRESS				0x0e

/* Command registers */
#define CMD_A					0x30
#define CMD_A_CHG_ENABLED			BIT(1)
#define CMD_A_SUSPEND_ENABLED			BIT(2)
#define CMD_A_OTG_ENABLED			BIT(4)
#define CMD_A_FORCE_FCC				BIT(6)
#define CMD_A_ALLOW_WRITE			BIT(7)
#define CMD_B					0x31
#define CMD_B_MODE_HC				BIT(0)
#define CMD_C					0x33

/* Interrupt Status registers */
#define IRQSTAT_A				0x35
#define IRQSTAT_A_HOT_HARD_STAT		BIT(6)
#define IRQSTAT_A_HOT_HARD_IRQ			BIT(7)
#define IRQSTAT_A_COLD_HARD_STAT		BIT(4)
#define IRQSTAT_A_COLD_HARD_IRQ		BIT(5)
#define IRQSTAT_B				0x36
#define IRQSTAT_B_BATOVP_STAT			BIT(6)
#define IRQSTAT_B_BATOVP_IRQ			BIT(7)
#define IRQSTAT_C				0x37
#define IRQSTAT_C_TERMINATION_STAT		BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ		BIT(1)
#define IRQSTAT_C_TAPER_IRQ			BIT(3)
#define IRQSTAT_D				0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT		BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ		BIT(3)
#define IRQSTAT_D_APSD_STAT			BIT(6)
#define IRQSTAT_D_APSD_IRQ			BIT(7)
#define IRQSTAT_E				0x39
#define IRQSTAT_E_USBIN_UV_STAT			BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ			BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT			BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ			BIT(5)
/*
 * In SMB349 the DCIN UV status bits are reversed in the
 * IRQSTAT_E register, the DCIN_UV IRQ & Status correspond
 * to bits 1 & 0 respectively.
 */
#define SMB349_IRQSTAT_E_DCIN_UV_STAT		BIT(0)
#define SMB349_IRQSTAT_E_DCIN_UV_IRQ		BIT(1)
#define SMB349_IRQSTAT_E_DCIN_OV_STAT		BIT(2)
#define SMB349_IRQSTAT_E_DCIN_OV_IRQ		BIT(3)
#define IRQSTAT_F				0x3a
#define IRQSTAT_F_OTG_UV_IRQ			BIT(5)
#define IRQSTAT_F_OTG_UV_STAT			BIT(4)
#define IRQSTAT_F_OTG_DET_IRQ			BIT(3)
#define IRQSTAT_F_OTG_DET_STAT			BIT(2)
#define IRQSTAT_F_PWR_OK_IRQ			BIT(1)
#define IRQSTAT_F_PWR_OK_STAT			BIT(0)

/* Status registers */
#define STAT_A					0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK		0x3f
#define STAT_B					0x3c
#define STAT_B_RID_GROUND			0x10
#define STAT_C					0x3d
#define STAT_C_CHG_ENABLED			BIT(0)
#define STAT_C_HOLDOFF_STAT			BIT(3)
#define STAT_C_CHG_MASK				0x06
#define STAT_C_CHG_SHIFT			1
#define STAT_C_CHG_TERM				BIT(5)
#define STAT_C_CHARGER_ERROR			BIT(6)
#define STAT_D					0x3E
#define STAT_E					0x3F
#define SMB349_SUSPENDED			(1<<7)

#define STATUS_UPDATE_INTERVAL			(HZ * 60)	/*60 sec */

#define SMB_FCC_1000				(0<<4)
#define SMB_FCC_1200				(1<<4)
#define SMB_FCC_1400				(2<<4)
#define SMB_FCC_1600				(3<<4)
#define SMB_FCC_1800				(4<<4)
#define SMB_FCC_2000				(5<<4)
#define SMB_FCC_MASK				(0xf<<4)

#define CC_1800					1800
#define CC_1600					1600
#define CC_1400					1400
#define CC_1200					1200
#define CC_1000					1000

#define SMB_INLMT_500				0
#define SMB_INLMT_900				1
#define SMB_INLMT_1000				2
#define SMB_INLMT_1100				3
#define SMB_INLMT_1200				4
#define SMB_INLMT_1300				5
#define SMB_INLMT_1500				6
#define SMB_INLMT_1600				7
#define SMB_INLMT_1700				8
#define SMB_INLMT_1800				9
#define SMB_INLMT_2000				0xa
#define SMB_INLMT_MASK				0xf

#define ILIM_1800				1800
#define ILIM_1500				1500
#define ILIM_1000				1000
#define ILIM_500				500

#define SMB_FVLTG_4350				0x2D
#define SMB_FVLTG_4200				0x25
#define SMB_FVLTG_4100				0x20
#define SMB_FVLTG_4000				0x1B
#define SMB_FVLTG_MASK				0x3F

#define CV_4350					4350
#define CV_4200					4200
#define CV_4100					4100
#define CV_4000					4000

#define ITERM_400				400
#define ITERM_300				300
#define ITERM_200				200
#define ITERM_100				100

#define SMB_ITERM_200				(0<<2)
#define SMB_ITERM_300				(1<<2)
#define SMB_ITERM_400				(2<<2)
#define SMB_ITERM_500				(3<<2)
#define SMB_ITERM_600				(4<<2)
#define SMB_ITERM_700				(5<<2)
#define SMB_ITERM_100				(6<<2)

#define SMB_ITERM_MASK				(7<<2)

#define SMB_CHRG_TYPE_ACA_DOCK	(1<<7)
#define SMB_CHRG_TYPE_ACA_C		(1<<6)
#define SMB_CHRG_TYPE_ACA_B		(1<<5)
#define SMB_CHRG_TYPE_ACA_A		(1<<4)
#define SMB_CHRG_TYPE_CDP		(1<<3)
#define SMB_CHRG_TYPE_DCP		(1<<2)
#define SMB_CHRG_TYPE_SDP		(1<<1)
#define SMB_CHRG_TYPE_UNKNOWN	(1<<0)

#define SMB_CHRG_CUR_DCP		1800
#define SMB_CHRG_CUR_ACA		1800
#define SMB_CHRG_CUR_CDP		1500
#define SMB_CHRG_CUR_SDP		500

#define CFG_PIN_DEFAULT_CFG		0x7E
#define SMB34X_FULL_WORK_JIFFIES		(30*HZ)
#define SMB34X_TEMP_WORK_JIFFIES		(30*HZ)
#define SMB34X_CHG_UPD_JIFFIES  (HZ)

#define RETRY_WRITE			3
#define RETRY_READ			3

#define SMB34X_EXTCON_SDP		"CHARGER_USB_SDP"
#define SMB34X_EXTCON_DCP		"CHARGER_USB_DCP"
#define SMB34X_EXTCON_CDP		"CHARGER_USB_CDP"

#define REGULATOR_V3P3SX		"v3p3sx"

static const char *smb34x_extcon_cable[] = {
	SMB34X_EXTCON_SDP,
	SMB34X_EXTCON_DCP,
	SMB34X_EXTCON_CDP,
	NULL,
};

static const short smb349_inlim[] = { /* mA */
	500, 900, 1000, 1100, 1200, 1300, 1500, 1600,
	1700, 1800, 2000, 2200, 2400, 2500, 3000, 3500
};

struct smb347_otg_event {
	struct list_head	node;
	bool			param;
};

/**
 * struct smb347_charger - smb347 charger instance
 * @lock: protects concurrent access to online variables
 * @client: pointer to i2c client
 * @mains: power_supply instance for AC/DC power
 * @usb: power_supply instance for USB power
 * @battery: power_supply instance for battery
 * @mains_online: is AC/DC input connected
 * @usb_online: is USB input connected
 * @charging_enabled: is charging enabled
 * @running: the driver is up and running
 * @dentry: for debugfs
 * @otg: pointer to OTG transceiver if any
 * @otg_nb: notifier for OTG notifications
 * @otg_work: work struct for OTG notifications
 * @otg_queue: queue holding received OTG notifications
 * @otg_queue_lock: protects concurrent access to @otg_queue
 * @otg_last_param: last parameter value from OTG notification
 * @otg_enabled: OTG VBUS is enabled
 * @otg_battery_uv: OTG battery undervoltage condition is on
 * @pdata: pointer to platform data
 */
struct smb347_charger {
	struct mutex		lock;
	struct i2c_client	*client;
	struct power_supply	mains;
	struct power_supply	usb;
	struct power_supply	battery;
	struct delayed_work     chg_upd_worker;
	struct delayed_work	temp_upd_worker;
	bool			mains_online;
	bool			usb_online;
	bool			charging_enabled;
	bool			running;
	bool			is_smb349;
	bool			drive_vbus;
	bool			a_bus_enable;
	struct dentry		*dentry;
	struct usb_phy		*otg;
	struct notifier_block	otg_nb;
	struct work_struct	otg_work;
	struct list_head	otg_queue;
	spinlock_t		otg_queue_lock;
	bool			otg_enabled;
	bool			otg_battery_uv;
	bool			is_disabled;
	const struct smb347_charger_platform_data	*pdata;
	struct extcon_dev	*edev;
	/* power supply properties */
	enum power_supply_charger_cable_type cable_type;
	int			inlmt;
	int			cc;
	int			cv;
	int			max_cc;
	int			max_cv;
	int			max_temp;
	int			min_temp;
	int			iterm;
	int			cntl_state;
	int			online;
	int			present;
	/*
	 * regulator v3p3sx used by display driver to save 7mW in
	 * S3 for USB Host
	 */
	struct regulator	*regulator_v3p3sx;
	bool			regulator_enabled;
#ifdef CONFIG_POWER_SUPPLY_CHARGER
	struct delayed_work	full_worker;
#endif
	struct wake_lock	wakelock;
};

static struct smb347_charger *smb347_dev;
static int smb347_set_iterm(struct smb347_charger *smb, int iterm);
static int smb347_disable_suspend(struct smb347_charger *smb);
static int sm347_reload_setting(struct smb347_charger *smb);

static int smb347_read(struct smb347_charger *smb, u8 reg)
{
	int ret, i;

	for (i = 0; i < RETRY_READ; i++) {
		ret = i2c_smbus_read_byte_data(smb->client, reg);
		if (ret < 0) {
			dev_warn(&smb->client->dev,
				"failed to read reg 0x%x: %d\n", reg, ret);
			mdelay(1);
			continue;
		} else
			break;
	}
	return ret;
}

static int smb347_write(struct smb347_charger *smb, u8 reg, u8 val)
{
	int ret, i;

	for (i = 0; i < RETRY_WRITE; i++) {
		ret = i2c_smbus_write_byte_data(smb->client, reg, val);
		if (ret < 0) {
			dev_warn(&smb->client->dev,
			"failed to write reg 0x%x: %d\n", reg, ret);
			mdelay(1);
			continue;
		} else
			break;
	}
	return ret;
}

static bool smb347_is_suspended(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_read(smb, STAT_E);
	if (ret < 0) {
		dev_warn(&smb->client->dev, "i2c failed %d", ret);
		return false;
	}
	return (ret & SMB349_SUSPENDED) == SMB349_SUSPENDED;
}

/*
 * smb347_set_writable - enables/disables writing to non-volatile registers
 * @smb: pointer to smb347 charger instance
 *
 * You can enable/disable writing to the non-volatile configuration
 * registers by calling this function.
 *
 * Returns %0 on success and negative errno in case of failure.
 */
static int smb347_set_writable(struct smb347_charger *smb, bool writable)
{
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb347_write(smb, CMD_A, ret);
}

static inline int smb347_force_fcc(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	ret |= CMD_A_FORCE_FCC;

	return smb347_write(smb, CMD_A, ret);
}

static int smb34x_get_health(struct smb347_charger *smb)
{
	int stat_e = 0, usb;
	int chrg_health;

	if (!smb->is_smb349) {
		chrg_health = POWER_SUPPLY_HEALTH_UNKNOWN;
		goto end;
	}
	if (smb->pdata->detect_chg) {
		usb = smb347_read(smb, STAT_D);
		if (usb < 0) {
			dev_err(&smb->client->dev, "%s:i2c read error", __func__);
			chrg_health = POWER_SUPPLY_HEALTH_UNKNOWN;
			goto end;
		}
		usb = !smb->is_disabled && usb;
	} else {
		usb = !smb->is_disabled;
	}

	stat_e = smb347_read(smb, IRQSTAT_E);
	if (stat_e < 0) {
		dev_warn(&smb->client->dev, "i2c failed %d", stat_e);
		chrg_health = POWER_SUPPLY_HEALTH_UNSPEC_FAILURE;
		goto end;
	}

	if (usb) {
		/* charger present && charger not disabled */
		if (stat_e & SMB349_IRQSTAT_E_DCIN_UV_STAT)
			chrg_health = POWER_SUPPLY_HEALTH_DEAD;
		else if (stat_e & SMB349_IRQSTAT_E_DCIN_OV_STAT)
			chrg_health = POWER_SUPPLY_HEALTH_OVERVOLTAGE;
		else
			chrg_health = POWER_SUPPLY_HEALTH_GOOD;

	} else {
			chrg_health = POWER_SUPPLY_HEALTH_UNKNOWN;
	}
	if (chrg_health != POWER_SUPPLY_HEALTH_GOOD)
		smb->online = 0;
end:
	return chrg_health;
}

/**
 * smb347_update_status - updates the charging status
 * @smb: pointer to smb347 charger instance
 *
 * Function checks status of the charging and updates internal state
 * accordingly. Returns %0 if there is no change in status, %1 if the
 * status has changed and negative errno in case of failure.
 */
static int smb347_update_status(struct smb347_charger *smb)
{
	bool usb = false;
	bool dc = false;
	int ret;

	ret = smb347_read(smb, IRQSTAT_E);
	if (ret < 0)
		return ret;

	if (smb->is_smb349) {
		if (smb->pdata->use_mains)
			dc = !(ret & SMB349_IRQSTAT_E_DCIN_UV_STAT);
		else if (smb->pdata->use_usb) {
			usb = !(ret & (SMB349_IRQSTAT_E_DCIN_UV_STAT |
					SMB349_IRQSTAT_E_DCIN_OV_STAT)) &&
				!smb->is_disabled;
		}

		mutex_lock(&smb->lock);
		ret = smb->mains_online != dc || smb->usb_online != usb;
		smb->mains_online = dc;
		smb->usb_online = usb;
		mutex_unlock(&smb->lock);
		return ret;
	}

	/*
	 * Dc and usb are set depending on whether they are enabled in
	 * platform data _and_ whether corresponding undervoltage is set.
	 */
	if (smb->pdata->use_mains)
		dc = !(ret & IRQSTAT_E_DCIN_UV_STAT);
	if (smb->pdata->use_usb)
		usb = !(ret & IRQSTAT_E_USBIN_UV_STAT);

	mutex_lock(&smb->lock);
	ret = smb->mains_online != dc || smb->usb_online != usb;
	smb->mains_online = dc;
	smb->usb_online = usb;
	mutex_unlock(&smb->lock);

	return ret;
}

/*
 * smb347_is_online - returns whether input power source is connected
 * @smb: pointer to smb347 charger instance
 *
 * Returns %true if input power source is connected. Note that this is
 * dependent on what platform has configured for usable power sources. For
 * example if USB is disabled, this will return %false even if the USB
 * cable is connected.
 */
static bool smb347_is_online(struct smb347_charger *smb)
{
	bool ret;

	mutex_lock(&smb->lock);
	ret = (smb->usb_online || smb->mains_online) && !smb->otg_enabled;
	mutex_unlock(&smb->lock);

	return ret;
}

/**
 * smb347_charging_status - returns status of charging
 * @smb: pointer to smb347 charger instance
 *
 * Function returns charging status. %0 means no charging is in progress,
 * %1 means pre-charging, %2 fast-charging and %3 taper-charging.
 */
static int smb347_charging_status(struct smb347_charger *smb)
{
	int ret;

	if (!smb347_is_online(smb))
		return 0;

	ret = smb347_read(smb, STAT_C);
	if (ret < 0)
		return 0;

	return (ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT;
}

static void smb347_enable_termination(struct smb347_charger *smb,
						bool enable)
{
	int ret;

	mutex_lock(&smb->lock);
	ret = smb347_set_writable(smb, true);
	if (ret < 0) {
		dev_warn(&smb->client->dev, "i2c error %d", ret);
		return;
	}

	ret = smb347_read(smb, CFG_STATUS_IRQ);
	if (ret < 0) {
		dev_warn(&smb->client->dev, "i2c error %d", ret);
		goto err_term;
	}

	if (enable)
		ret |= CFG_STATUS_IRQ_TERMINATION_OR_TAPER;
	else
		ret &= ~CFG_STATUS_IRQ_TERMINATION_OR_TAPER;

	ret = smb347_write(smb, CFG_STATUS_IRQ, ret);

	if (ret < 0)
		dev_warn(&smb->client->dev, "i2c error %d", ret);

err_term:
	ret = smb347_set_writable(smb, false);
	if (ret < 0)
		dev_warn(&smb->client->dev, "i2c error %d", ret);
	mutex_unlock(&smb->lock);

}

static int smb347_is_charger_present(struct smb347_charger *smb)
{
	int chg_type;
	if (smb->pdata->detect_chg) {
		chg_type = smb347_read(smb, STAT_D);

		return (chg_type <= 0) ? 0 : 1;
	}
	return 1;
}

static int smb347_set_otg_reg_ctrl(struct smb347_charger *smb)
{
	int ret;

	if (smb->pdata->detect_chg) {
		mutex_lock(&smb->lock);
		smb347_set_writable(smb, true);
		ret = smb347_read(smb, CFG_OTHER);
		if (ret < 0)
			goto err_reg_ctrl;
		ret |= CFG_OTHER_RID_ENABLED_OTG_I2C;
		ret = smb347_write(smb, CFG_OTHER, ret);
		smb347_set_writable(smb, false);
		mutex_unlock(&smb->lock);
	} else {
		return 0;
	}

err_reg_ctrl:
	return ret;


}

static int smb347_charging_set(struct smb347_charger *smb, bool enable)
{
	int ret = 0;

	if (smb->pdata->enable_control != SMB347_CHG_ENABLE_SW) {
		dev_info(&smb->client->dev,
			"charging enable/disable in SW disabled\n");
		return 0;
	}

	mutex_lock(&smb->lock);
	if (smb->charging_enabled != enable) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		smb->charging_enabled = enable;

		if (enable)
			ret |= CMD_A_CHG_ENABLED;
		else
			ret &= ~CMD_A_CHG_ENABLED;

		ret |= CMD_A_FORCE_FCC;
		ret = smb347_write(smb, CMD_A, ret);
	}
out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_charging_enable(struct smb347_charger *smb)
{
	return smb347_charging_set(smb, true);
}

static inline int smb347_charging_disable(struct smb347_charger *smb)
{
	return smb347_charging_set(smb, false);
}

static int smb347_enable_suspend(struct smb347_charger *smb)
{
	int ret;

	if (!smb)
		return -EINVAL;

	mutex_lock(&smb->lock);
	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		goto en_sus_err;

	ret |= CMD_A_SUSPEND_ENABLED;

	ret = smb347_write(smb, CMD_A, ret);
	if (ret < 0)
		goto en_sus_err;

	mutex_unlock(&smb->lock);

	return 0;

en_sus_err:
	dev_info(&smb->client->dev, "i2c error %d", ret);
	mutex_unlock(&smb->lock);
	return ret;
}

static int smb347_disable_suspend(struct smb347_charger *smb)
{
	int ret;

	if (!smb)
		return -EINVAL;

	mutex_lock(&smb->lock);
	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		goto dis_sus_err;

	ret &= ~CMD_A_SUSPEND_ENABLED;

	ret = smb347_write(smb, CMD_A, ret);
	if (ret < 0)
		goto dis_sus_err;

	mutex_unlock(&smb->lock);

	return 0;

dis_sus_err:
	dev_info(&smb->client->dev, "i2c error %d", ret);
	mutex_unlock(&smb->lock);
	return ret;
}

/*
 * smb347_enable_charger:
 *	Exposed to EXTCON for enabling charging
 *	Update the status variable, for device active state
 */
int smb347_enable_charger()
{
	struct smb347_charger *smb = smb347_dev;
	int ret;

	ret = smb347_disable_suspend(smb);
	if (ret < 0)
		return ret;

	mutex_lock(&smb->lock);
	smb->is_disabled = false;
	mutex_unlock(&smb->lock);

	return 0;
}
EXPORT_SYMBOL(smb347_enable_charger);

/*
 * smb347_disable_charger:
 *	Exposed to EXTCON for disabling charging via SDP
 *	Update the status variable, for device active state
 */
int smb347_disable_charger()
{
	struct smb347_charger *smb = smb347_dev;
	int ret;

	ret = smb347_enable_suspend(smb);
	if (ret < 0)
		return ret;

	mutex_lock(&smb->lock);
	smb->is_disabled = true;
	mutex_unlock(&smb->lock);

	return 0;
}
EXPORT_SYMBOL(smb347_disable_charger);

static int smb347_update_online(struct smb347_charger *smb)
{
	int ret;

	/*
	 * Depending on whether valid power source is connected or not, we
	 * disable or enable the charging. We do it manually because it
	 * depends on how the platform has configured the valid inputs.
	 */
	if (smb347_is_online(smb)) {
		ret = smb347_charging_enable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to enable charging\n");
	} else {
		ret = smb347_charging_disable(smb);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"failed to disable charging\n");
	}

	return ret;
}

static int smb347_otg_set(struct smb347_charger *smb, bool enable)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	mutex_lock(&smb->lock);

	if (pdata->otg_control == SMB347_OTG_CONTROL_SW) {
		ret = smb347_read(smb, CMD_A);
		if (ret < 0)
			goto out;

		if (enable)
			ret |= CMD_A_OTG_ENABLED;
		else
			ret &= ~CMD_A_OTG_ENABLED;

		ret = smb347_write(smb, CMD_A, ret);
		if (ret < 0)
			goto out;
	} else {
		/*
		 * Switch to pin control or auto-OTG depending on how
		 * platform has configured.
		 */
		smb347_set_writable(smb, true);

		ret = smb347_read(smb, CFG_OTHER);
		if (ret < 0) {
			smb347_set_writable(smb, false);
			goto out;
		}

		ret &= ~CFG_OTHER_RID_MASK;

		switch (pdata->otg_control) {
		case SMB347_OTG_CONTROL_SW_PIN:
			if (enable) {
				ret |= CFG_OTHER_RID_DISABLED_OTG_PIN;
				ret |= CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			} else {
				ret &= ~CFG_OTHER_OTG_PIN_ACTIVE_LOW;
			}
			break;

		case SMB347_OTG_CONTROL_SW_AUTO:
			if (enable)
				ret |= CFG_OTHER_RID_ENABLED_AUTO_OTG;
			break;

		default:
			dev_err(&smb->client->dev,
				"impossible OTG control configuration: %d\n",
				pdata->otg_control);
			break;
		}

		ret = smb347_write(smb, CFG_OTHER, ret);
		smb347_set_writable(smb, false);
		if (ret < 0)
			goto out;
	}

	smb->otg_enabled = enable;

out:
	mutex_unlock(&smb->lock);
	return ret;
}

static inline int smb347_otg_enable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, true);
}

static inline int smb347_otg_disable(struct smb347_charger *smb)
{
	return smb347_otg_set(smb, false);
}

static void smb347_otg_drive_vbus(struct smb347_charger *smb, bool enable)
{
	if (enable == smb->otg_enabled)
		return;

	if (enable) {
		if (smb->otg_battery_uv) {
			dev_dbg(&smb->client->dev,
				"battery low voltage, won't enable OTG VBUS\n");
			return;
		}

		/*
		 * Normal charging must be disabled first before we try to
		 * enable OTG VBUS.
		 */
		smb347_charging_disable(smb);
		smb347_otg_enable(smb);

		if (smb->pdata->use_mains)
			power_supply_changed(&smb->mains);
		if (smb->pdata->use_usb)
			power_supply_changed(&smb->usb);

		dev_dbg(&smb->client->dev, "OTG VBUS on\n");
	} else {
		smb347_otg_disable(smb);
		/*
		 * Only re-enable charging if we have some power supply
		 * connected.
		 */
		if (smb347_is_online(smb)) {
			smb347_charging_enable(smb);
			smb->otg_battery_uv = false;
			if (smb->pdata->use_mains)
				power_supply_changed(&smb->mains);
			if (smb->pdata->use_usb)
				power_supply_changed(&smb->usb);
		}

		dev_dbg(&smb->client->dev, "OTG VBUS off\n");
	}
}

#ifdef CONFIG_POWER_SUPPLY_CHARGER
static void smb347_full_worker(struct work_struct *work)
{
	struct smb347_charger *smb =
		container_of(work, struct smb347_charger, full_worker.work);

	dev_info(&smb->client->dev, "%s", __func__);

	power_supply_changed(NULL);

	schedule_delayed_work(&smb->full_worker,
				SMB34X_FULL_WORK_JIFFIES);
}
#endif


static void smb347_otg_detect(struct smb347_charger *smb)
{
	int ret;

	ret = smb347_read(smb, STAT_B);
	dev_info(&smb->client->dev, "stat_b = %x", ret);
	if (ret < 0)
		dev_err(&smb->client->dev, "i2c error %d", ret);
	else if (ret & STAT_B_RID_GROUND) {
		smb->drive_vbus = true;
		if (smb->pdata->gpio_mux >= 0)
			gpio_direction_output(smb->pdata->gpio_mux, 0);
		if (smb->a_bus_enable) {
			smb347_otg_enable(smb);
			if (smb->regulator_v3p3sx) {
				regulator_enable(smb->regulator_v3p3sx);
				smb->regulator_enabled = true;
			}
		}
	} else {
		smb->drive_vbus = false;
		smb347_otg_disable(smb);
		if (smb->regulator_v3p3sx && smb->regulator_enabled) {
			regulator_disable(smb->regulator_v3p3sx);
			smb->regulator_enabled = false;
		}
	}
}

static void smb34x_update_charger_type(struct smb347_charger *smb)
{
	static struct power_supply_cable_props cable_props;
	static int notify_chrg, notify_usb;
	int ret, vbus_present, power_ok;

	ret = smb347_read(smb, STAT_D);
	if (ret < 0) {
		dev_err(&smb->client->dev, "%s:i2c read error", __func__);
		return;
	}

	dev_info(&smb->client->dev, "charger type %x\n", ret);

	/*
	 * sometimes, charger type is present on removal,
	 * check the UV status and decide disconnect
	 */
	power_ok = smb347_read(smb, IRQSTAT_E);
	if ((power_ok & (SMB349_IRQSTAT_E_DCIN_UV_STAT |
			SMB349_IRQSTAT_E_DCIN_OV_STAT)) && ret != 0) {
		/*
		 * during UV condition,chgr removal is
		 * not identified and using worker thread
		 * status is updated
		 */
		schedule_delayed_work(&smb->chg_upd_worker,
			SMB34X_CHG_UPD_JIFFIES);
		return;
	}

	switch (ret) {
	case SMB_CHRG_TYPE_ACA_DOCK:
	case SMB_CHRG_TYPE_ACA_C:
	case SMB_CHRG_TYPE_ACA_B:
	case SMB_CHRG_TYPE_ACA_A:
		cable_props.chrg_evt =
			POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type =
			POWER_SUPPLY_CHARGER_TYPE_USB_ACA;
		cable_props.ma = SMB_CHRG_CUR_ACA;
		notify_chrg = 1;
		vbus_present = 1;
		break;

	case SMB_CHRG_TYPE_CDP:
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
		cable_props.ma = SMB_CHRG_CUR_CDP;
		notify_chrg = 1;
		notify_usb = 1;
		vbus_present = 1;
		break;

	case SMB_CHRG_TYPE_SDP:
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
		cable_props.ma = SMB_CHRG_CUR_SDP;
		notify_usb = 1;
		notify_chrg = 1;
		vbus_present = 1;
		break;

	case SMB_CHRG_TYPE_DCP:
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_CONNECT;
		cable_props.chrg_type = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
		cable_props.ma = SMB_CHRG_CUR_DCP;
		notify_chrg = 1;
		vbus_present = 1;
		break;

	default:
		cable_props.chrg_evt = POWER_SUPPLY_CHARGER_EVENT_DISCONNECT;
		cable_props.ma = 0;
		vbus_present = 0;
		break;
	}

	dev_info(&smb->client->dev, "notify usb %d notify charger %d",
					notify_usb, notify_chrg);

	if (notify_usb) {
		if (smb->pdata->gpio_mux >= 0)
			gpio_direction_output(smb->pdata->gpio_mux, 1);
		atomic_notifier_call_chain(&smb->otg->notifier,
				USB_EVENT_VBUS, &vbus_present);
	}

	if (notify_chrg) {
		if (vbus_present) {
			sm347_reload_setting(smb);
			if (!wake_lock_active(&smb->wakelock))
				wake_lock(&smb->wakelock);
		} else {
			if (wake_lock_active(&smb->wakelock))
				wake_unlock(&smb->wakelock);
		}
		atomic_notifier_call_chain(&power_supply_notifier,
				POWER_SUPPLY_CABLE_EVENT, &cable_props);
	}

	if (cable_props.chrg_evt == POWER_SUPPLY_CHARGER_EVENT_DISCONNECT) {
		notify_chrg = 0;
		notify_usb = 0;
		if (smb->pdata->gpio_mux >= 0)
			gpio_direction_output(smb->pdata->gpio_mux, 0);
	}

	return;
}

static void smb347_chg_upd_worker(struct work_struct *work)
{
	struct smb347_charger *smb =
		container_of(work, struct smb347_charger, chg_upd_worker.work);

	dev_info(&smb->client->dev, "%s", __func__);

	smb34x_update_charger_type(smb);
}

static void smb347_temp_upd_worker(struct work_struct *work)
{
	struct smb347_charger *smb =
		container_of(work, struct smb347_charger,
						temp_upd_worker.work);
	int stat_c, irqstat_e, irqstat_a;
	int chg_status, ov_uv_stat, temp_stat;

	dev_info(&smb->client->dev, "%s", __func__);
	stat_c = smb347_read(smb, STAT_C);
	if (stat_c < 0)
		goto err_upd;

	irqstat_e = smb347_read(smb, IRQSTAT_E);
	if (irqstat_e < 0)
		goto err_upd;

	irqstat_a = smb347_read(smb, IRQSTAT_A);
	if (irqstat_a < 0)
		goto err_upd;

	chg_status = (stat_c & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT;
	ov_uv_stat = irqstat_e & (SMB349_IRQSTAT_E_DCIN_UV_STAT |
				SMB349_IRQSTAT_E_DCIN_OV_STAT);
	temp_stat = irqstat_a &
			(IRQSTAT_A_HOT_HARD_STAT|IRQSTAT_A_COLD_HARD_STAT);

	/* status = not charging, no uv, ov status, hard temp stat */
	if (!chg_status && !ov_uv_stat && temp_stat)
		power_supply_changed(&smb->usb);
	else
		return;
err_upd:
	schedule_delayed_work(&smb->temp_upd_worker,
						SMB34X_TEMP_WORK_JIFFIES);
}

static void smb347_otg_work(struct work_struct *work)
{
	struct smb347_charger *smb =
		container_of(work, struct smb347_charger, otg_work);
	struct smb347_otg_event *evt, *tmp;
	unsigned long flags;

	/* Process the whole event list in one go. */
	spin_lock_irqsave(&smb->otg_queue_lock, flags);
	list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
		list_del(&evt->node);
		spin_unlock_irqrestore(&smb->otg_queue_lock, flags);

		/* For now we only support set vbus events */
		smb347_otg_drive_vbus(smb, evt->param);
		kfree(evt);

		spin_lock_irqsave(&smb->otg_queue_lock, flags);
	}
	spin_unlock_irqrestore(&smb->otg_queue_lock, flags);
}

static int smb347_otg_notifier(struct notifier_block *nb, unsigned long event,
			       void *param)
{
	struct smb347_charger *smb =
		container_of(nb, struct smb347_charger, otg_nb);
	struct smb347_otg_event *evt;

	dev_dbg(&smb->client->dev, "OTG notification: %lu\n", event);
	if (!param || event != USB_EVENT_DRIVE_VBUS || !smb->running)
		return NOTIFY_DONE;

	evt = kzalloc(sizeof(*evt), GFP_ATOMIC);
	if (!evt) {
		dev_err(&smb->client->dev,
			"failed to allocate memory for OTG event\n");
		return NOTIFY_DONE;
	}

	evt->param = *(bool *)param;
	INIT_LIST_HEAD(&evt->node);

	spin_lock(&smb->otg_queue_lock);
	list_add_tail(&evt->node, &smb->otg_queue);
	spin_unlock(&smb->otg_queue_lock);

	queue_work(system_nrt_wq, &smb->otg_work);
	return NOTIFY_OK;
}

static int sm347_reload_setting(struct smb347_charger *smb)
{
	int ret, i, loop_count;
	int reg_offset = 0;

	mutex_lock(&smb->lock);
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto err_reload;

	loop_count = MAXSMB347_CONFIG_DATA_SIZE / 2;

	/*
	 * Program the platform specific configuration values to the device
	 */
	for (i = 0; (i < loop_count) &&
		(smb->pdata->char_config_regs[reg_offset] != 0xff); i++) {
		smb347_write(smb, smb->pdata->char_config_regs[reg_offset],
			smb->pdata->char_config_regs[reg_offset+1]);
		reg_offset += 2;
	}

	smb347_write(smb, CMD_B, CMD_B_MODE_HC);
	ret = smb347_set_writable(smb, false);

err_reload:
	mutex_unlock(&smb->lock);
	return ret;
}


static void smb347_usb_otg_enable(struct usb_phy *phy)
{
	struct smb347_charger *smb = smb347_dev;

	if (!smb)
		return;

	dev_info(&smb->client->dev, "%s:%d", __func__, __LINE__);

	if (phy->vbus_state == VBUS_DISABLED) {
		dev_info(&smb->client->dev, "OTG Disable");
		smb->a_bus_enable = false;
		if (smb->drive_vbus) {
			smb347_otg_disable(smb);
			if (smb->regulator_v3p3sx &&
					smb->regulator_enabled) {
				regulator_disable(smb->regulator_v3p3sx);
				smb->regulator_enabled = false;
			}
		}
	} else {
		dev_info(&smb->client->dev, "OTG Enable");
		smb->a_bus_enable = true;
		if (smb->drive_vbus) {
			smb347_otg_enable(smb);
			if (smb->regulator_v3p3sx) {
				regulator_enable(smb->regulator_v3p3sx);
				smb->regulator_enabled = true;
			}
		}
	}
}


static int smb347_hw_init(struct smb347_charger *smb)
{
	int ret, loop_count, i;
	int reg_offset = 0;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	loop_count = MAXSMB347_CONFIG_DATA_SIZE / 2;

	/*
	 * Program the platform specific configuration values to the device
	 */
	for (i = 0; (i < loop_count) &&
		(smb->pdata->char_config_regs[reg_offset] != 0xff); i++) {
		smb347_write(smb, smb->pdata->char_config_regs[reg_offset],
			smb->pdata->char_config_regs[reg_offset+1]);
		reg_offset += 2;
	}

	/* disable charging to recover from previous errors */
	ret = smb347_read(smb, CMD_A);
	if (ret >= 0) {
		ret &= ~CMD_A_CHG_ENABLED;
		smb347_write(smb, CMD_A, ret);
	}

	switch (smb->pdata->otg_control) {
	case SMB347_OTG_CONTROL_DISABLED:
		break;

	case SMB347_OTG_CONTROL_SW:
	case SMB347_OTG_CONTROL_SW_PIN:
	case SMB347_OTG_CONTROL_SW_AUTO:
		smb->otg = usb_get_phy(USB_PHY_TYPE_USB2);
		if (smb->otg) {
			INIT_WORK(&smb->otg_work, smb347_otg_work);
			INIT_LIST_HEAD(&smb->otg_queue);
			spin_lock_init(&smb->otg_queue_lock);

			smb->a_bus_enable = true;
			smb->otg->a_bus_drop = smb347_usb_otg_enable;
			smb->otg_nb.notifier_call = smb347_otg_notifier;
			ret = usb_register_notifier(smb->otg, &smb->otg_nb);
			if (ret < 0) {
				usb_put_phy(smb->otg);
				smb->otg = NULL;
				goto fail;
			}

			dev_info(&smb->client->dev,
				"registered to OTG notifications\n");
		}
		break;

	case SMB347_OTG_CONTROL_PIN:
	case SMB347_OTG_CONTROL_AUTO:
		/*
		 * no need to program again as the OTG settings
		 * as they are already configured as part of platform
		 * specific register configuration.
		 */
		break;
	}

	ret = smb347_update_status(smb);
	if (ret < 0)
		goto fail;

	smb347_set_writable(smb, false);
	return ret;

fail:
	if (smb->otg) {
		usb_unregister_notifier(smb->otg, &smb->otg_nb);
		usb_put_phy(smb->otg);
		smb->otg = NULL;
	}
	smb347_set_writable(smb, false);
	return ret;
}

static void smb347_hw_uninit(struct smb347_charger *smb)
{
	if (smb->otg) {
		struct smb347_otg_event *evt, *tmp;

		usb_unregister_notifier(smb->otg, &smb->otg_nb);
		smb347_otg_disable(smb);
		usb_put_phy(smb->otg);

		/* Clear all the queued events. */
		flush_work_sync(&smb->otg_work);
		list_for_each_entry_safe(evt, tmp, &smb->otg_queue, node) {
			list_del(&evt->node);
			kfree(evt);
		}
	}
}

static irqreturn_t smb347_interrupt(int irq, void *data)
{
	struct smb347_charger *smb = data;
	int stat_c, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	int irqstat_a, irqstat_b, stat_a;
	irqreturn_t ret = IRQ_NONE;

	stat_c = smb347_read(smb, STAT_C);
	if (stat_c < 0) {
		dev_warn(&smb->client->dev, "reading STAT_C failed\n");
		return IRQ_NONE;
	}

	stat_a = smb347_read(smb, STAT_A);
	if (stat_a < 0) {
		dev_warn(&smb->client->dev, "reading STAT_A failed\n");
		return IRQ_NONE;
	}

	irqstat_a = smb347_read(smb, IRQSTAT_A);
	if (irqstat_a < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_A failed\n");
		return IRQ_NONE;
	}

	irqstat_b = smb347_read(smb, IRQSTAT_B);
	if (irqstat_b < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_B failed\n");
		return IRQ_NONE;
	}

	irqstat_c = smb347_read(smb, IRQSTAT_C);
	if (irqstat_c < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_d = smb347_read(smb, IRQSTAT_D);
	if (irqstat_d < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_D failed\n");
		return IRQ_NONE;
	}

	irqstat_e = smb347_read(smb, IRQSTAT_E);
	if (irqstat_e < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_E failed\n");
		return IRQ_NONE;
	}

	irqstat_f = smb347_read(smb, IRQSTAT_F);
	if (irqstat_f < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_F failed\n");
		return IRQ_NONE;
	}

	/*
	 * If we get charger error we report the error back to user.
	 * If the error is recovered charging will resume again.
	 */
	if (stat_c & STAT_C_CHARGER_ERROR) {
		dev_err(&smb->client->dev,
			"charging stopped due to charger error\n");
		if (smb->pdata->use_usb && smb->is_smb349) {
			smb->usb_online = 0;
			power_supply_changed(&smb->usb);
		}
		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);

		ret = IRQ_HANDLED;
	}

	if (irqstat_a & (IRQSTAT_A_HOT_HARD_IRQ|IRQSTAT_A_COLD_HARD_IRQ)) {
		dev_info(&smb->client->dev, "extreme temperature interrupt");
		schedule_delayed_work(&smb->temp_upd_worker, 0);
	}

	if (irqstat_b & IRQSTAT_B_BATOVP_IRQ) {
		dev_info(&smb->client->dev, "BATOVP interrupt");
		/* Reset charging in case of battery OV */
		smb347_charging_set(smb, false);
		if (smb->pdata->use_usb)
			power_supply_changed(&smb->usb);
	}
	/*
	 * If we reached the termination current the battery is charged and
	 * we can update the status now. Charging is automatically
	 * disabled by the hardware.
	 */
#ifndef CONFIG_POWER_SUPPLY_CHARGER
	if (irqstat_c & (IRQSTAT_C_TERMINATION_IRQ | IRQSTAT_C_TAPER_IRQ)) {
		if ((irqstat_c & IRQSTAT_C_TERMINATION_STAT) &&
						smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
		dev_info(&smb->client->dev,
			"[Charge Terminated] Going to HW Maintenance mode\n");
		ret = IRQ_HANDLED;
	}
#else
	if (irqstat_c & (IRQSTAT_C_TAPER_IRQ | IRQSTAT_C_TERMINATION_IRQ)) {
		if (smb->charging_enabled) {
			/*
			 * reduce the termination current value, to avoid
			 * repeated interrupts.
			 */
			smb347_set_iterm(smb, SMB_ITERM_100);
			smb347_enable_termination(smb, false);
			/*
			 * since termination has happened, charging will not be
			 * re-enabled until, disable and enable charging is
			 * done.
			 */
			smb347_charging_set(smb, false);
			smb347_charging_set(smb, true);
			schedule_delayed_work(&smb->full_worker, 0);
		}
		ret = IRQ_HANDLED;
	}
#endif

	/*
	 * If we got a complete charger timeout int that means the charge
	 * full is not detected with in charge timeout value.
	 */
	if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_IRQ) {
		dev_info(&smb->client->dev,
			"[Charge Timeout]:Total Charge Timeout INT recieved\n");
		if (irqstat_d & IRQSTAT_D_CHARGE_TIMEOUT_STAT)
			dev_info(&smb->client->dev,
				"[Charge Timeout]:charging stopped\n");

		/* Restart charging once timeout has happened */
		smb347_charging_set(smb, false);
		smb347_charging_set(smb, true);
		if (smb->pdata->show_battery)
			power_supply_changed(&smb->battery);
		ret = IRQ_HANDLED;
	}

	if (smb->pdata->detect_chg) {
		if (irqstat_d & IRQSTAT_D_APSD_STAT) {
			smb347_disable_suspend(smb);
			ret = IRQ_HANDLED;
		}
	}
	/*
	 * If we got an under voltage interrupt it means that AC/USB input
	 * was connected or disconnected.
	 */
	if (irqstat_e & (IRQSTAT_E_USBIN_UV_IRQ | IRQSTAT_E_DCIN_UV_IRQ |
			SMB349_IRQSTAT_E_DCIN_OV_IRQ |
			SMB349_IRQSTAT_E_DCIN_OV_STAT)) {
		if (smb347_update_status(smb) > 0) {
			smb347_update_online(smb);
			/*
			 * In SMB349 chip the charging is not starting
			 * immediately after charger connect. We have to
			 * wait untill POWER OK signal/pin becomes good.
			 * So only send the uevent notification for smb347.
			 * For smb349 uevent notification will be sent upon
			 * Power Ok interrupt.
			 */
#ifndef CONFIG_POWER_SUPPLY_CHARGER
			if (smb->pdata->use_mains && !smb->is_smb349)
				power_supply_changed(&smb->mains);
			if (smb->pdata->use_usb && !smb->is_smb349)
				power_supply_changed(&smb->usb);
#endif
		}

		ret = IRQ_HANDLED;
	}

	/*
	 * If the battery voltage falls below OTG UVLO the VBUS is
	 * automatically turned off but we must not enable it again unless
	 * UVLO is cleared. It will be cleared when external power supply
	 * is connected and the battery voltage goes over the UVLO
	 * threshold.
	 */
	if (irqstat_f & IRQSTAT_F_OTG_UV_IRQ) {
		smb->otg_battery_uv = !!(irqstat_f & IRQSTAT_F_OTG_UV_STAT);
		dev_info(&smb->client->dev, "Vbatt is below OTG UVLO\n");
		smb347_otg_disable(smb);
		ret = IRQ_HANDLED;
	}

	if (irqstat_f & IRQSTAT_F_OTG_DET_IRQ) {
		smb347_otg_detect(smb);
		ret = IRQ_HANDLED;
	}

	if (irqstat_f & IRQSTAT_F_PWR_OK_IRQ) {
		dev_info(&smb->client->dev, "PowerOK INTR recieved\n");

		smb347_update_status(smb);

		if (smb->pdata->detect_chg)
			smb34x_update_charger_type(smb);

		/*
		 * In case of charger OV/UV update_charger_type
		 * will not send power_supply_changed. Also during
		 * charger unplug UV interrupt will be triggered
		 * Hence check if the charger is present and health
		 * is over voltage or under voltage send power
		 * supply changed notification.
		 */
		if (smb347_is_charger_present(smb)) {
			int health = smb34x_get_health(smb);
			if (health == POWER_SUPPLY_HEALTH_OVERVOLTAGE ||
				health == POWER_SUPPLY_HEALTH_DEAD) {
				if (smb->pdata->use_usb)
					power_supply_changed(&smb->usb);
				if (smb->pdata->use_mains)
					power_supply_changed(&smb->mains);
			}
		}

		ret = IRQ_HANDLED;
	}

	return ret;
}

static int smb347_irq_set(struct smb347_charger *smb, bool enable)
{
	int ret;

	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/*
	 * Enable/disable interrupts for:
	 *	- under voltage
	 *	- termination current reached
	 *	- charger error
	 */
	if (enable) {
		int val = CFG_FAULT_IRQ_DCIN_UV;

		if (smb->otg)
			val |= CFG_FAULT_IRQ_OTG_UV;

		ret = smb347_write(smb, CFG_FAULT_IRQ, val);
		if (ret < 0)
			goto fail;

		val = CFG_STATUS_IRQ_CHARGE_TIMEOUT |
			CFG_STATUS_IRQ_TERMINATION_OR_TAPER |
			CFG_STATUS_OTG_DET |
			CFG_STATUS_IRQ_INOK;
		ret = smb347_write(smb, CFG_STATUS_IRQ, val);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret |= (CFG_PIN_EN_CHARGER_ERROR | CFG_PIN_EN_APSD_IRQ);

		ret = smb347_write(smb, CFG_PIN, ret);
	} else {
		ret = smb347_write(smb, CFG_FAULT_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_write(smb, CFG_STATUS_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb347_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CHARGER_ERROR;

		ret = smb347_write(smb, CFG_PIN, ret);
	}

fail:
	smb347_set_writable(smb, false);
	return ret;
}

static inline int smb347_irq_enable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, true);
}

static inline int smb347_irq_disable(struct smb347_charger *smb)
{
	return smb347_irq_set(smb, false);
}

static int smb347_irq_init(struct smb347_charger *smb)
{
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, smb347_interrupt,
				   IRQF_TRIGGER_FALLING | IRQF_ONESHOT,
				   smb->client->name,
				   smb);
	if (ret < 0) {
		dev_info(&smb->client->dev, "request irq failed");
		goto fail_gpio;
	}

	mutex_lock(&smb->lock);
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto fail_irq;

	/*
	 * Configure the STAT output to be suitable for interrupts: disable
	 * all other output (except interrupts) and make it active low.
	 */
	ret = smb347_read(smb, CFG_STAT);
	if (ret < 0)
		goto fail_readonly;

	ret &= ~CFG_STAT_ACTIVE_HIGH;
	ret |= CFG_STAT_DISABLED;

	ret = smb347_write(smb, CFG_STAT, ret);
	if (ret < 0)
		goto fail_readonly;

	ret = smb347_irq_enable(smb);
	if (ret < 0)
		goto fail_readonly;

	smb347_set_writable(smb, false);
	smb->client->irq = irq;
	enable_irq_wake(smb->client->irq);
	mutex_unlock(&smb->lock);
	return 0;

fail_readonly:
	smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
fail_irq:
	free_irq(irq, smb);
fail_gpio:
	gpio_free(pdata->irq_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static int smb347_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, mains);

	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		val->intval = smb->mains_online;
		return 0;
	}
	return -EINVAL;
}

static enum power_supply_property smb347_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

#ifdef CONFIG_POWER_SUPPLY_CHARGER
static enum power_supply_type power_supply_cable_type(
		enum power_supply_charger_cable_type cable)
{

	switch (cable) {

	case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
		return POWER_SUPPLY_TYPE_USB_DCP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
		return POWER_SUPPLY_TYPE_USB_CDP;
	case POWER_SUPPLY_CHARGER_TYPE_USB_ACA:
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
#endif

static int smb347_set_cc(struct smb347_charger *smb, int cc)
{
	int ret;
	int smb_cc;

	mutex_lock(&smb->lock);
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto err_cc;

	if (cc >= CC_1800)
		smb_cc = SMB_FCC_1800;
	else if (cc >= CC_1600)
		smb_cc = SMB_FCC_1600;
	else if (cc >= CC_1400)
		smb_cc = SMB_FCC_1400;
	else if (cc >= CC_1200)
		smb_cc = SMB_FCC_1200;
	else
		smb_cc = SMB_FCC_1000;

	ret = smb347_read(smb, CFG_CHARGE_CURRENT);
	if (ret < 0)
		goto err_cc;

	ret &= ~SMB_FCC_MASK;
	smb_cc |= ret;
	ret = smb347_write(smb, CFG_CHARGE_CURRENT, smb_cc);
	if (ret < 0)
		goto err_cc;

	ret = smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
	return 0;

err_cc:
	ret = smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
	dev_info(&smb->client->dev, "%s:error writing to i2c", __func__);
	return ret;
}

static int smb347_set_inlmt(struct smb347_charger *smb, int inlmt)
{
	int ret;
	int smb_inlmt;

	mutex_lock(&smb->lock);
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto err_inlmt;

	for (smb_inlmt = 0; smb_inlmt < ARRAY_SIZE(smb349_inlim); smb_inlmt++)
		if (inlmt <= smb349_inlim[smb_inlmt])
			break;

	ret = smb347_read(smb, CFG_CHARGE_CURRENT);
	if (ret < 0)
		goto err_inlmt;

	ret &= ~SMB_INLMT_MASK;
	smb_inlmt |= ret;

	ret = smb347_write(smb, CFG_CHARGE_CURRENT, smb_inlmt);
	if (ret < 0)
		goto err_inlmt;

	ret = smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
	return 0;

err_inlmt:
	ret = smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
	dev_info(&smb->client->dev, "%s:error writing to i2c", __func__);
	return ret;
}

static int smb347_set_cv(struct smb347_charger *smb, int cv)
{
	int ret;
	int smb_cv;

	mutex_lock(&smb->lock);
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto err_cv;

	if (cv >= CV_4350)
		smb_cv = SMB_FVLTG_4350;
	else if (cv >= CV_4200)
		smb_cv = SMB_FVLTG_4200;
	else if (cv >= CV_4100)
		smb_cv = SMB_FVLTG_4100;
	else
		smb_cv = SMB_FVLTG_4000;

	ret = smb347_read(smb, CFG_FLOAT_VOLTAGE);
	if (ret < 0)
		goto err_cv;

	ret &= ~SMB_FVLTG_MASK;
	smb_cv |= ret;

	ret = smb347_write(smb, CFG_FLOAT_VOLTAGE, smb_cv);
	if (ret < 0)
		goto err_cv;

	ret = smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
	return 0;

err_cv:
	ret = smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
	dev_info(&smb->client->dev, "%s:error writing to i2c", __func__);
	return ret;
}

static int smb347_set_iterm(struct smb347_charger *smb, int iterm)
{
	int ret;
	int smb_iterm;

	mutex_lock(&smb->lock);
	ret = smb347_set_writable(smb, true);
	if (ret < 0)
		goto err_iterm;

	if (iterm >= ITERM_400)
		smb_iterm = SMB_ITERM_400;
	else if (iterm >= ITERM_300)
		smb_iterm = SMB_ITERM_300;
	else if (iterm >= ITERM_200)
		smb_iterm = SMB_ITERM_200;
	else
		smb_iterm = SMB_ITERM_100;

	ret = smb347_read(smb, CFG_CURRENT_LIMIT);
	if (ret < 0)
		goto err_iterm;

	ret &= ~SMB_ITERM_MASK;
	smb_iterm |= ret;

	ret = smb347_write(smb, CFG_CURRENT_LIMIT, smb_iterm);
	if (ret < 0)
		goto err_iterm;

	ret = smb347_set_writable(smb, false);
	mutex_unlock(&smb->lock);
	return 0;

err_iterm:
	ret = smb347_set_writable(smb, false);
	dev_info(&smb->client->dev, "%s:error writing to i2c", __func__);
	mutex_unlock(&smb->lock);
	return ret;
}

#ifndef CONFIG_POWER_SUPPLY_CHARGER
static int smb347_throttle_charging(struct smb347_charger *smb, int lim)
{
	struct power_supply_throttle *throttle_states =
					smb->pdata->throttle_states;
	int ret;

	if (lim < 0 || lim > (smb->pdata->num_throttle_states - 1))
		return -ERANGE;

	if (throttle_states[lim].throttle_action ==
				PSY_THROTTLE_CC_LIMIT) {
		ret = smb347_enable_charger();
		if (ret < 0)
			goto throttle_fail;
		ret = smb347_set_cc(smb, throttle_states[lim].throttle_val);
		if (ret < 0)
			goto throttle_fail;
		ret = smb347_charging_set(smb, true);
	} else if (throttle_states[lim].throttle_action ==
				PSY_THROTTLE_INPUT_LIMIT) {
		ret = smb347_enable_charger();
		if (ret < 0)
			goto throttle_fail;
		ret = smb347_set_inlmt(smb, throttle_states[lim].throttle_val);
		if (ret < 0)
			goto throttle_fail;
		ret = smb347_charging_set(smb, true);
	} else if (throttle_states[lim].throttle_action ==
				PSY_THROTTLE_DISABLE_CHARGING) {
		ret = smb347_enable_charger();
		if (ret < 0)
			goto throttle_fail;
		ret = smb347_charging_set(smb, false);
	} else if (throttle_states[lim].throttle_action ==
				PSY_THROTTLE_DISABLE_CHARGER) {
		ret = smb347_disable_charger();
	} else {
		return -EINVAL;
	}

throttle_fail:
	return ret;
}
#endif

static int smb347_usb_set_property(struct power_supply *psy,
					enum power_supply_property psp,
					const union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, usb);
	int ret = 0;

	switch (psp) {
	case POWER_SUPPLY_PROP_PRESENT:
		smb->present = val->intval;
		break;
	case POWER_SUPPLY_PROP_ONLINE:
		smb->online = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		smb->max_cc = val->intval;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		smb->max_cv = val->intval;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		ret = smb347_charging_set(smb, (bool)val->intval);
		if (ret < 0)
			dev_err(&smb->client->dev,
				"Error %d in %s charging", ret,
				(val->intval ? "enable" : "disable"));
#ifdef CONFIG_POWER_SUPPLY_CHARGER
		if (!val->intval)
			cancel_delayed_work(&smb->full_worker);
		else {
			smb347_set_iterm(smb, smb->iterm);
			smb347_enable_termination(smb, true);
		}
#endif
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		if (val->intval)
			ret = smb347_enable_charger();
		else {
			/*
			 * OTG cannot function if charger is put to
			 *  suspend state, send fake ntf in such case
			 */
			if (!smb->pdata->detect_chg ||
				smb347_is_charger_present(smb))
				ret = smb347_disable_charger();
			else
				smb->is_disabled = true;
		}
		if (ret < 0)
			dev_err(&smb->client->dev,
				"Error %d in %s charger", ret,
				(val->intval ? "enable" : "disable"));
		/*
		 * Set OTG in Register control, as default HW
		 * settings did not enable OTG.
		 */
		smb347_set_otg_reg_ctrl(smb);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		ret = smb347_set_cc(smb, val->intval);
		if (!ret) {
			mutex_lock(&smb->lock);
			smb->cc = val->intval;
			mutex_unlock(&smb->lock);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		ret = smb347_set_cv(smb, val->intval);
		if (!ret) {
			mutex_lock(&smb->lock);
			smb->cv = val->intval;
			mutex_unlock(&smb->lock);
		}
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		ret = smb347_set_iterm(smb, val->intval);
		if (!ret) {
			mutex_lock(&smb->lock);
			smb->iterm = val->intval;
			mutex_unlock(&smb->lock);
		}
		break;
#ifdef CONFIG_POWER_SUPPLY_CHARGER
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		mutex_lock(&smb->lock);
		smb->cable_type = val->intval;
		smb->usb.type = power_supply_cable_type(smb->cable_type);
		mutex_unlock(&smb->lock);
		break;
#endif
	case POWER_SUPPLY_PROP_INLMT:
		ret = smb347_set_inlmt(smb, val->intval);
		if (!ret) {
			mutex_lock(&smb->lock);
			smb->inlmt = val->intval;
			mutex_unlock(&smb->lock);
		}
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		mutex_lock(&smb->lock);
		smb->max_temp = val->intval;
		mutex_unlock(&smb->lock);
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		mutex_lock(&smb->lock);
		smb->min_temp = val->intval;
		mutex_unlock(&smb->lock);
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
#ifdef CONFIG_POWER_SUPPLY_CHARGER
		if (val->intval < smb->pdata->num_throttle_states)
			smb->cntl_state = val->intval;
		else
			ret = -ERANGE;
#else
		if (val->intval < smb->pdata->num_throttle_states) {
			ret = smb347_throttle_charging(smb, val->intval);
			if (ret < 0)
				break;
			smb->cntl_state = val->intval;
		} else {
			ret = -ERANGE;
		}
#endif
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

static int smb347_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	struct smb347_charger *smb =
		container_of(psy, struct smb347_charger, usb);
	int ret = 0;

	mutex_lock(&smb->lock);
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		val->intval = smb->online;
		break;
	case POWER_SUPPLY_PROP_PRESENT:
		val->intval = smb->present;
		break;
	case POWER_SUPPLY_PROP_HEALTH:
		val->intval = smb34x_get_health(smb);
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT:
		val->intval = smb->max_cc;
		break;
	case POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE:
		val->intval = smb->max_cv;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CURRENT:
		val->intval = smb->cc;
		break;
	case POWER_SUPPLY_PROP_CHARGE_VOLTAGE:
		val->intval = smb->cv;
		break;
	case POWER_SUPPLY_PROP_INLMT:
		val->intval = smb->inlmt;
		break;
	case POWER_SUPPLY_PROP_CHARGE_TERM_CUR:
		val->intval = smb->iterm;
		break;
	case POWER_SUPPLY_PROP_CABLE_TYPE:
		val->intval = smb->cable_type;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGING:
		val->intval = smb->charging_enabled;
		break;
	case POWER_SUPPLY_PROP_ENABLE_CHARGER:
		val->intval = !smb->is_disabled;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT:
		val->intval = smb->cntl_state;
		break;
	case POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX:
		val->intval = smb->pdata->num_throttle_states;
		break;
	case POWER_SUPPLY_PROP_MAX_TEMP:
		val->intval = smb->max_temp;
		break;
	case POWER_SUPPLY_PROP_MIN_TEMP:
		val->intval = smb->min_temp;
		break;
	default:
		return -EINVAL;
	}
	mutex_unlock(&smb->lock);
	return ret;
}

int smb347_get_charging_status(void)
{
	int ret, status;

	if (!smb347_dev)
		return -EINVAL;

	if (!smb347_is_online(smb347_dev))
		return POWER_SUPPLY_STATUS_DISCHARGING;

	ret = smb347_read(smb347_dev, STAT_C);
	if (ret < 0)
		return ret;

	if ((ret & STAT_C_CHARGER_ERROR) ||
		(ret & STAT_C_HOLDOFF_STAT)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_C_CHG_MASK) >> STAT_C_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_C_CHG_TERM) {
			/* set the status to FULL if battery is not in pre
			 * charge, fast charge or taper charging mode AND
			 * charging is terminated at least once.
			 */
			status = POWER_SUPPLY_STATUS_FULL;
		} else {
			/* in this case no charger error or termination
			 * occured but charging is not in progress!!!
			 */
			status = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
	}

	return status;
}
EXPORT_SYMBOL_GPL(smb347_get_charging_status);

static enum power_supply_property smb347_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_HEALTH,
	POWER_SUPPLY_PROP_TYPE,
	POWER_SUPPLY_PROP_MAX_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_MAX_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_CHARGE_CURRENT,
	POWER_SUPPLY_PROP_CHARGE_VOLTAGE,
	POWER_SUPPLY_PROP_INLMT,
	POWER_SUPPLY_PROP_ENABLE_CHARGING,
	POWER_SUPPLY_PROP_ENABLE_CHARGER,
	POWER_SUPPLY_PROP_CHARGE_TERM_CUR,
#ifdef CONFIG_POWER_SUPPLY_CHARGER
	POWER_SUPPLY_PROP_CABLE_TYPE,
#endif
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT,
	POWER_SUPPLY_PROP_CHARGE_CONTROL_LIMIT_MAX,
	POWER_SUPPLY_PROP_MAX_TEMP,
	POWER_SUPPLY_PROP_MIN_TEMP
};

static int smb347_battery_get_property(struct power_supply *psy,
				       enum power_supply_property prop,
				       union power_supply_propval *val)
{
	struct smb347_charger *smb =
			container_of(psy, struct smb347_charger, battery);
	const struct smb347_charger_platform_data *pdata = smb->pdata;
	int ret;

	ret = smb347_update_status(smb);
	if (ret < 0)
		return ret;

	switch (prop) {
	case POWER_SUPPLY_PROP_STATUS:
		ret = smb347_get_charging_status();
		if (ret < 0)
			return ret;
		val->intval = ret;
		break;

	case POWER_SUPPLY_PROP_CHARGE_TYPE:
		if (!smb347_is_online(smb))
			return -ENODATA;

		/*
		 * We handle trickle and pre-charging the same, and taper
		 * and none the same.
		 */
		switch (smb347_charging_status(smb)) {
		case 1:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_TRICKLE;
			break;
		case 2:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_FAST;
			break;
		default:
			val->intval = POWER_SUPPLY_CHARGE_TYPE_NONE;
			break;
		}
		break;

	case POWER_SUPPLY_PROP_TECHNOLOGY:
		val->intval = pdata->battery_info.technology;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN:
		val->intval = pdata->battery_info.voltage_min_design;
		break;

	case POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN:
		val->intval = pdata->battery_info.voltage_max_design;
		break;

	case POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN:
		val->intval = pdata->battery_info.charge_full_design;
		break;

	case POWER_SUPPLY_PROP_MODEL_NAME:
		val->strval = pdata->battery_info.name;
		break;

	default:
		return -EINVAL;
	}

	return 0;
}

static enum power_supply_property smb347_battery_properties[] = {
	POWER_SUPPLY_PROP_STATUS,
	POWER_SUPPLY_PROP_CHARGE_TYPE,
	POWER_SUPPLY_PROP_TECHNOLOGY,
	POWER_SUPPLY_PROP_VOLTAGE_MIN_DESIGN,
	POWER_SUPPLY_PROP_VOLTAGE_MAX_DESIGN,
	POWER_SUPPLY_PROP_CHARGE_FULL_DESIGN,
	POWER_SUPPLY_PROP_MODEL_NAME,
};

static int smb347_debugfs_show(struct seq_file *s, void *data)
{
	struct smb347_charger *smb = s->private;
	int ret;
	u8 reg;

	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb347_read(smb, reg);
		if (ret < 0)
			return ret;
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Command registers:\n");
	seq_printf(s, "==================\n");
	ret = smb347_read(smb, CMD_A);
	if (ret < 0)
		return ret;
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_A, ret);
	ret = smb347_read(smb, CMD_B);
	if (ret < 0)
		return ret;
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_B, ret);
	ret = smb347_read(smb, CMD_C);
	if (ret < 0)
		return ret;
	seq_printf(s, "0x%02x:\t0x%02x\n", CMD_C, ret);
	seq_printf(s, "\n");

	seq_printf(s, "Interrupt status registers:\n");
	seq_printf(s, "===========================\n");
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb347_read(smb, reg);
		if (ret < 0)
			return ret;
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}
	seq_printf(s, "\n");

	seq_printf(s, "Status registers:\n");
	seq_printf(s, "=================\n");
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb347_read(smb, reg);
		if (ret < 0)
			return ret;
		seq_printf(s, "0x%02x:\t0x%02x\n", reg, ret);
	}

	return 0;
}

static int smb347_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb347_debugfs_show, inode->i_private);
}

static const struct file_operations smb347_debugfs_fops = {
	.open		= smb347_debugfs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_ACPI
extern void *smb347_platform_data(void *);
#endif
static int smb347_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct smb347_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb347_charger *smb;
	int ret;
	struct acpi_gpio_info gpio_info;

	pdata = dev->platform_data;

#ifdef CONFIG_ACPI
	pdata = smb347_platform_data(NULL);
#endif
	if (!pdata)
		return -EINVAL;

#ifdef CONFIG_ACPI
	pdata->irq_gpio = acpi_get_gpio_by_index(&client->dev, 0, &gpio_info);
	if (pdata->irq_gpio < 0)
		return -EINVAL;
	else
		dev_info(&client->dev, "gpio no:%d\n", pdata->irq_gpio);
#endif

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	mutex_init(&smb->lock);
	smb->client = client;
	smb->pdata = pdata;

#ifndef CONFIG_ACPI
	smb->is_smb349 = strcmp(id->name, "smb349") ? 0 : 1;
#else
	smb->is_smb349 = 1;
#endif
	dev_info(&smb->client->dev, "%s chip in use.\n",
			smb->is_smb349 ? "smb349" : "smb347");

	ret = smb347_hw_init(smb);
	if (ret < 0)
		return ret;

	wake_lock_init(&smb->wakelock, WAKE_LOCK_SUSPEND, "smb_wakelock");

	smb347_dev = smb;
	if (smb->pdata->use_regulator) {
		smb->regulator_v3p3sx = regulator_get(dev, REGULATOR_V3P3SX);
		if (IS_ERR(smb->regulator_v3p3sx)) {
			dev_warn(&smb->client->dev, "V3P3SX failed");
			smb->regulator_v3p3sx = NULL;
		}
	}

	INIT_DELAYED_WORK(&smb->chg_upd_worker, smb347_chg_upd_worker);
#ifdef CONFIG_POWER_SUPPLY_CHARGER
	INIT_DELAYED_WORK(&smb->full_worker, smb347_full_worker);
#endif
	INIT_DELAYED_WORK(&smb->temp_upd_worker, smb347_temp_upd_worker);

	if (smb->pdata->use_mains) {
		smb->mains.name = "smb34x-ac_charger";
		smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
		smb->mains.get_property = smb347_mains_get_property;
		smb->mains.properties = smb347_mains_properties;
		smb->mains.num_properties = ARRAY_SIZE(smb347_mains_properties);
		smb->mains.supplied_to = pdata->supplied_to;
		smb->mains.num_supplicants = pdata->num_supplicants;
		ret = power_supply_register(dev, &smb->mains);
		if (ret < 0)
			goto psy_reg1_failed;
	}

	if (smb->pdata->use_usb) {
		smb->usb.name = "smb34x-usb_charger";
		smb->usb.type = POWER_SUPPLY_TYPE_USB;
		smb->usb.get_property = smb347_usb_get_property;
		smb->usb.properties = smb347_usb_properties;
		smb->usb.num_properties = ARRAY_SIZE(smb347_usb_properties);
		smb->usb.supplied_to = pdata->supplied_to;
		smb->usb.num_supplicants = pdata->num_supplicants;
		smb->usb.throttle_states = pdata->throttle_states;
		smb->usb.num_throttle_states = pdata->num_throttle_states;
		smb->usb.supported_cables = pdata->supported_cables;
		smb->usb.set_property = smb347_usb_set_property;
		smb->max_cc = 1800;
		smb->max_cv = 4350;
		ret = power_supply_register(dev, &smb->usb);
		if (ret < 0)
			goto psy_reg2_failed;
	}

	if (smb->pdata->show_battery) {
		smb->battery.name = "smb34x_battery";
		smb->battery.type = POWER_SUPPLY_TYPE_BATTERY;
		smb->battery.get_property = smb347_battery_get_property;
		smb->battery.properties = smb347_battery_properties;
		smb->battery.num_properties =
				ARRAY_SIZE(smb347_battery_properties);
		ret = power_supply_register(dev, &smb->battery);
		if (ret < 0)
			goto psy_reg3_failed;
	}

	/* register with extcon */
	smb->edev = devm_kzalloc(dev, sizeof(struct extcon_dev), GFP_KERNEL);
	if (!smb->edev) {
		dev_err(&client->dev, "mem alloc failed\n");
		ret = -ENOMEM;
		goto psy_reg4_failed;
	}
	smb->edev->name = "smb34x";
	smb->edev->supported_cable = smb34x_extcon_cable;
	ret = extcon_dev_register(smb->edev, &client->dev);
	if (ret) {
		dev_err(&client->dev, "extcon registration failed!!\n");
		goto psy_reg4_failed;
	}

	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = smb347_irq_init(smb);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
			goto psy_irq_failed;
		}
	}

	if (smb->pdata->detect_chg)
		smb34x_update_charger_type(smb);
	smb347_otg_detect(smb);

	smb->running = true;
	smb->dentry = debugfs_create_file("smb347-regs", S_IRUSR, NULL, smb,
					  &smb347_debugfs_fops);
	return 0;

psy_irq_failed:
	extcon_dev_unregister(smb->edev);
psy_reg4_failed:
	if (smb->pdata->show_battery)
		power_supply_unregister(&smb->battery);
psy_reg3_failed:
	if (smb->pdata->use_usb)
		power_supply_unregister(&smb->usb);
psy_reg2_failed:
	if (smb->pdata->use_mains)
		power_supply_unregister(&smb->mains);
psy_reg1_failed:
	smb347_hw_uninit(smb);
	wake_lock_destroy(&smb->wakelock);
	smb347_dev = NULL;
	return ret;
}

static int smb347_remove(struct i2c_client *client)
{
	struct smb347_charger *smb = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	smb->running = false;

	if (client->irq) {
		smb347_irq_disable(smb);
		free_irq(client->irq, smb);
		gpio_free(smb->pdata->irq_gpio);
	}

	smb347_hw_uninit(smb);
	extcon_dev_unregister(smb->edev);
	if (smb->pdata->show_battery)
		power_supply_unregister(&smb->battery);
	if (smb->pdata->use_usb)
		power_supply_unregister(&smb->usb);
	if (smb->pdata->use_mains)
		power_supply_unregister(&smb->mains);
	wake_lock_destroy(&smb->wakelock);

	if (smb->regulator_v3p3sx)
		regulator_put(smb->regulator_v3p3sx);

	return 0;
}

static void smb347_shutdown(struct i2c_client *client)
{
	struct smb347_charger *smb = i2c_get_clientdata(client);

	if (client->irq > 0)
		disable_irq(client->irq);

	if (smb->drive_vbus)
		smb347_otg_disable(smb);

	return;
}

static int smb347_suspend(struct device *dev)
{
	struct smb347_charger *smb = dev_get_drvdata(dev);

	if (smb->client->irq > 0)
		disable_irq(smb->client->irq);

	return 0;
}

static int smb347_resume(struct device *dev)
{
	struct smb347_charger *smb = dev_get_drvdata(dev);

	if (smb->client->irq > 0)
		enable_irq(smb->client->irq);

	return 0;
}

static const struct dev_pm_ops smb347_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(smb347_suspend, smb347_resume)
};

static const struct i2c_device_id smb347_id[] = {
	{ "smb347", 0},
	{ "smb349", 1},
	{ "SMB0347", 0},
	{ "SMB0349", 1},
	{}
};
MODULE_DEVICE_TABLE(i2c, smb347_id);

#ifdef CONFIG_ACPI
static struct acpi_device_id smb349_acpi_match[] = {
	{"SMB0349", 0},
	{}
};
MODULE_DEVICE_TABLE(acpi, smb349_acpi_match);

#endif


static struct i2c_driver smb347_driver = {
	.driver = {
		.name	= "smb347",
		.owner	= THIS_MODULE,
		.pm	= &smb347_pm_ops,
#ifdef CONFIG_ACPI
		.acpi_match_table = ACPI_PTR(smb349_acpi_match),
#endif
	},
	.probe		= smb347_probe,
	.remove		= smb347_remove,
	.id_table	= smb347_id,
	.shutdown	= smb347_shutdown,
};

static int __init smb347_init(void)
{
	return i2c_add_driver(&smb347_driver);
}
late_initcall(smb347_init);

static void __exit smb347_exit(void)
{
	i2c_del_driver(&smb347_driver);
}
module_exit(smb347_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_DESCRIPTION("SMB347 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb347");
