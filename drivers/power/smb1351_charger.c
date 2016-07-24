/*
 * Copyright (c) 2014, ASUSTek, Inc. All Rights Reserved.
 * Written by Chih-Hsuan Chang <Chih-Hsuan_Chang@asus.com>
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
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>
#include <linux/workqueue.h>
#include <linux/lnw_gpio.h>
#include <linux/earlysuspend.h>
#include <linux/HWVersion.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_mid_thermal.h>
#include <asm/pmic_pdata.h>
#include <asm/intel-mid.h>

/*smb1351 registers*/
#define SMB1351_FAST_CHRG_CURR_REG		0x00
#define SMB1351_OTHER_CHRG_CURR_REG		0x01
#define SMB1351_VARIOUS_FUNC_REG			0x02
#define SMB1351_FLOAT_VOLTAGE_REG			0x03
#define SMB1351_PIN_ENABLE_CTRL_REG		0x06
#define SMB1351_THERM_CTRL_A_REG			0x07
#define SMB1351_WTD_CTRL_REG				0x08
#define SMB1351_OTG_CTRL_REG				0x0A
#define SMB1351_SH_LIMIT_TEMP_REG			0x0B
#define SMB1351_VAR_FUNC_REG				0x0E
#define SMB1351_HVDCP_REG				0x12
#define SMB1351_OTG_MODE_PWR_REG		0x14
#define SMB1351_I2C_CMD_REG				0x30
#define SMB1351_IL_CMD_REG				0x31
#define SMB1351_APSD_REG					0x34
#define SMB1351_STATUS_REG				0x36
#define SMB1351_STATUS_4_REG				0x3A
#define SMB1351_STATUS_7_REG				0x3D
#define SMB1351_IRQ_G_REG					0x47

#define SMB1351_SW_JEITA_INIT_TIME			0*HZ
#define SMB1351_SW_JEITA_POLLING_TIME		60*HZ
#define SMB1351_CHARGING_INIT_TIME		0*HZ
#define SMB1351_OTG_INIT_TIME				0*HZ
#define SMB1351_CHARGING_STAUTS_TIME		0*HZ
#define SMB1351_DETECT_ASUS_ADAPTER_TIME	30*HZ

/*ads1013 registers*/
#define ADS1013_CONVERSION_REG		0x00
#define ADS1013_CONFIG_REG			0x01

/*us5587 registers*/
#define US5587_ADC_REG				0x04

/*other definitions*/
#define CHR_INFO(...)			printk("[smb1351] " __VA_ARGS__)
#define CHR_ERR(...)			printk("[smb1351_err] " __VA_ARGS__)
#define SMB1351_INOK_GPIO		"CHG_INOK#"
#define SMB1351_ULPI_D2		"ADCPWREN_GP1"
#define PMIC_GPIO1_CTLO		0x7F
#define PMIC_GPIO2_CTLO		0x80
#define PMIC_GPIO3_CTLO		0x81
#define PMIC_GPIO6_CTLO		0x84

#define BYTETOBINARYPATTERN "%d%d%d%d-%d%d%d%db"
#define BYTETOBINARY(byte) \
	(byte & 0x80 ? 1 : 0), \
	(byte & 0x40 ? 1 : 0), \
	(byte & 0x20 ? 1 : 0), \
	(byte & 0x10 ? 1 : 0), \
	(byte & 0x08 ? 1 : 0), \
	(byte & 0x04 ? 1 : 0), \
	(byte & 0x02 ? 1 : 0), \
	(byte & 0x01 ? 1 : 0)

/*global variables*/
static int parallel_charger_flag=0, chrg_mode=0, otg_mode=0, boot_mode=0;
static int hvdcp_flag=0, debug_flag=0, type_c_flag=0;
struct workqueue_struct *smb1351_delay_wq, *smb1351_parallel_delay_wq;
struct delayed_work smb1351_charging_work, smb1351_sw_jeita_work, smb1351_detect_ASUS_adapter_work;
struct delayed_work smb1351_otg_work, smb1351_charging_status_work;
struct delayed_work smb1351_parallel_charging_work, smb1351_parallel_sw_jeita_work;
static struct smb1351_charger *smb1351_a_dev, *smb1351_b_dev;
static struct adc_i2c_dev *ads1013_i2c_dev, *us5587_i2c_dev;
enum type_c_type {
	TYPE_C_OTHERS=0,
	TYPE_C_1_5_A,
	TYPE_C_3_A,
	TYPE_C_PD,
};
enum dual_charger {
	DUALCHR_NONE=0,
	DUALCHR_ASUS_2A,
	DUALCHR_TYPE_C_3A,
};
enum hvdcp_voltage {
	HVDCP_5V=0,
	HVDCP_9V,
	HVDCP_12V,
};
enum charger_voltage {
	V_4_1=0,
	V_4_38,
};
enum fastcharge_I {
	FC_I_1000MA=0,
	FC_I_1400MA,
	FC_I_1600MA,
	FC_I_1800MA,
	FC_I_2000MA,
	FC_I_2400MA,
};
enum usb_in_I {
	IUSB_IN_500MA=0,
	IUSB_IN_1000MA,
	IUSB_IN_1500MA,
	IUSB_IN_2000MA,
};
enum parallel_enable {
	Disable=0,
	Enable_1,
	Enable_2,
	Enable_3,
	Enable_4,
};
enum temperature_range {  //unit: 0.1 degree Celsius
	BELOW_0=0,
	IN_0_200,
	IN_200_500,
	IN_500_600,
	ABOVE_600,
};
static int dual_charger_flag=DUALCHR_NONE, temp_flag=IN_200_500, vchg_flag=V_4_38;

/*extern symbols*/
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
extern int Read_Boot_Mode(void);
extern __u16 core_get_advertised_current(void);
#ifdef CONFIG_LEDS_ASUS
extern void led_brightness_set(int led, int brightness);
#endif
#ifdef CONFIG_LEDS_KTD2026
extern void ktd2026_led_brightness_set(int led, int brightness);
#endif

struct smb1351_charger {
	struct mutex lock;
	struct i2c_client *client;
	struct power_supply mains;
	struct power_supply usb;
	struct notifier_block	chr_type_nb;
	struct usb_phy *transceiver;
	struct wake_lock wakelock_cable;
	struct wake_lock wakelock_cable_t;
	int inok_gpio;
	int ulpi_d2_gpio;
};

struct adc_i2c_dev {
	struct i2c_client *client;
};

static int smb1351_read(struct smb1351_charger *smb, u8 reg)
{
	int ret;

	ret = i2c_smbus_read_byte_data(smb->client, reg);
	if (ret < 0)
		CHR_ERR("failed to read reg 0x%02x: %d\n", reg, ret);

	return ret;
}

static int smb1351_write(struct smb1351_charger *smb, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(smb->client, reg, val);
	if (ret < 0)
		CHR_ERR("failed to write reg 0x%02x: %d\n", reg, ret);

	return ret;
}

static char *smb1351_power_supplied_to[] = {
	"battery",
};

static int smb1351_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		if ((chrg_mode==POWER_SUPPLY_CHARGER_TYPE_USB_DCP)||
			(chrg_mode==POWER_SUPPLY_CHARGER_TYPE_NONE))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property smb1351_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb1351_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	switch (prop) {
	case POWER_SUPPLY_PROP_ONLINE:
		if ((chrg_mode==POWER_SUPPLY_CHARGER_TYPE_USB_SDP)||
			(chrg_mode==POWER_SUPPLY_CHARGER_TYPE_USB_CDP))
			val->intval = 1;
		else
			val->intval = 0;
		break;
	default:
		return -EINVAL;
	}
	return 0;
}

static enum power_supply_property smb1351_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb1351_get_charging_status(void)
{
	int ret, status;

	if (!smb1351_a_dev) {
		CHR_INFO("Warning: smb1351_a_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb1351_read(smb1351_a_dev, SMB1351_STATUS_4_REG);
	if (ret < 0)
		return ret;
	if (ret & BIT(3)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & 0x06) >> 1) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 * also there is quick charging mode in smb1351.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & BIT(5)) {
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
	CHR_INFO("reg SMB1351_STATUS_4_REG = 0x%02x, charging status = %d\n", ret, status);

	return status;
}

static int smb1351_set_writable(struct smb1351_charger *smb, bool writable)
{
	int ret;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	CHR_INFO("%s: set BQ config volatile access: %s\n", __func__, writable ? "on" : "off");
	ret = smb1351_read(smb, SMB1351_I2C_CMD_REG);
	if (ret < 0) {
		CHR_ERR("%s failed!!!\n", __func__);
		return ret;
	}
	if (writable)
		ret |= BIT(6);

	return smb1351_write(smb, SMB1351_I2C_CMD_REG, ret);
}

static int smb1351_set_charging(struct smb1351_charger *smb, bool on)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	CHR_INFO("%s: %s\n", __func__, on ? "Enbale" : "Disable");
	if (on) {
		/* 06h[6:5]="11" */
		val = smb1351_read(smb, SMB1351_PIN_ENABLE_CTRL_REG);
		if (val < 0)
			goto out;
		val |= (BIT(6)|BIT(5));
		ret = smb1351_write(smb, SMB1351_PIN_ENABLE_CTRL_REG, val);
		if (ret < 0)
			goto out;
	} else {
		/* 06h[6:5]="10" */
		val = smb1351_read(smb, SMB1351_PIN_ENABLE_CTRL_REG);
		if (val < 0)
			goto out;
		val |= (BIT(6));
		val &= ~(BIT(5));
		ret = smb1351_write(smb, SMB1351_PIN_ENABLE_CTRL_REG, val);
		if (ret < 0)
			goto out;
	}
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_parallel_set_charging(struct smb1351_charger *smb, bool on)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	CHR_INFO("%s: %s\n", __func__, on ? "Enbale" : "Disable");
	if (on) {
		/* 06h[6:5]="10" */
		val = smb1351_read(smb, SMB1351_PIN_ENABLE_CTRL_REG);
		if (val < 0)
			goto out;
		val |= (BIT(6));
		val &= ~(BIT(5));
		ret = smb1351_write(smb, SMB1351_PIN_ENABLE_CTRL_REG, val);
		if (ret < 0)
			goto out;
	} else {
		/* 06h[6:5]="11" */
		val = smb1351_read(smb, SMB1351_PIN_ENABLE_CTRL_REG);
		if (val < 0)
			goto out;
		val |= (BIT(6)|BIT(5));
		ret = smb1351_write(smb, SMB1351_PIN_ENABLE_CTRL_REG, val);
		if (ret < 0)
			goto out;
	}
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_set_fastcharge_I(struct smb1351_charger *smb, int type)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	switch (type) {
		case FC_I_1000MA:
			/* 1000mA: 00h[7:4]="0000" */
			CHR_INFO("%s: set fast charge current 1000mA\n", __func__);
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7)|BIT(6)|BIT(5)|BIT(4));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case FC_I_1400MA:
			/* 1400mA: 00h[7:4]="0010" */
			CHR_INFO("%s: set fast charge current 1400mA\n", __func__);
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7)|BIT(6)|BIT(4));
			val |= (BIT(5));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case FC_I_1600MA:
			/* 1600mA: 00h[7:4]="0011" */
			CHR_INFO("%s: set fast charge current 1600mA\n", __func__);
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7)|BIT(6));
			val |= (BIT(5)|BIT(4));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case FC_I_1800MA:
			/* 1800mA: 00h[7:4]="0100" */
			CHR_INFO("%s: set fast charge current 1800mA\n", __func__);
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7)|BIT(5)|BIT(4));
			val |= (BIT(6));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case FC_I_2000MA:
			/* 2000mA: 00h[7:4]="0101" */
			CHR_INFO("%s: set fast charge current 2000mA\n", __func__);
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7)|BIT(5));
			val |= (BIT(6)|BIT(4));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case FC_I_2400MA:
			/* 2400mA: 00h[7:4]="0111" */
			CHR_INFO("%s: set fast charge current 2400mA\n", __func__);
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7));
			val |= (BIT(6)|BIT(5)|BIT(4));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		default:
			return -EINVAL;
	}
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_set_voltage(struct smb1351_charger *smb, int volt)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	switch (volt) {
		case V_4_1:
			/* 4.1V, 03h[5:0]="011110" */
			CHR_INFO("%s: set float voltage 4.1V\n", __func__);
			val = smb1351_read(smb, SMB1351_FLOAT_VOLTAGE_REG);
			if (val < 0)
				goto out;
			val |= (BIT(4)|BIT(3)|BIT(2)|BIT(1));
			val &= ~(BIT(5)|BIT(0));
			ret = smb1351_write(smb, SMB1351_FLOAT_VOLTAGE_REG, val);
			if (ret < 0)
				goto out;
			vchg_flag=V_4_1;
			break;
		case V_4_38:
			/* 4.4V, 03h[5:0]="101100" */
			CHR_INFO("%s: set float voltage 4.38V\n", __func__);
			val = smb1351_read(smb, SMB1351_FLOAT_VOLTAGE_REG);
			if (val < 0)
				goto out;
			val |= (BIT(5)|BIT(3)|BIT(2));
			val &= ~(BIT(4)|BIT(1)|BIT(0));
			ret = smb1351_write(smb, SMB1351_FLOAT_VOLTAGE_REG, val);
			if (ret < 0)
				goto out;
			vchg_flag=V_4_38;
			break;
		default:
			return -EINVAL;
	}
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_set_hvdcp_voltage(struct smb1351_charger *smb, int type)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	switch (type) {
		case HVDCP_5V:
			/* 5V, 12h[7:6]="00" */
			CHR_INFO("%s: set HVDCP voltage 5V\n", __func__);
			val = smb1351_read(smb, SMB1351_HVDCP_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7)|BIT(6));
			ret = smb1351_write(smb, SMB1351_HVDCP_REG, val);
			if (ret < 0)
				goto out;
			break;
		case HVDCP_9V:
			/* 9V, 12h[7:6]="01" */
			CHR_INFO("%s: set HVDCP voltage 9V\n", __func__);
			val = smb1351_read(smb, SMB1351_HVDCP_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(7));
			val |= (BIT(6));
			ret = smb1351_write(smb, SMB1351_HVDCP_REG, val);
			if (ret < 0)
				goto out;
			break;
		case HVDCP_12V:
			break;
		default:
			return -EINVAL;
	}
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_aicl(struct smb1351_charger *smb, bool on)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	CHR_INFO("%s: %s AICL\n", __func__, on ? "Enbale" : "Disable");
	if (on) {
		/* 02h[4]="1" */
		val = smb1351_read(smb, SMB1351_VARIOUS_FUNC_REG);
		if (val < 0)
			goto out;
		val |= (BIT(4));
		ret = smb1351_write(smb, SMB1351_VARIOUS_FUNC_REG, val);
		if (ret < 0)
			goto out;
	}
	else {
		/* 02h[4]="0" */
		val = smb1351_read(smb, SMB1351_VARIOUS_FUNC_REG);
		if (val < 0)
			goto out;
		val &= ~(BIT(4));
		ret = smb1351_write(smb, SMB1351_VARIOUS_FUNC_REG, val);
		if (ret < 0)
			goto out;
	}
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_charging_init(void)
{
	int ret=0, val=0;

	CHR_INFO("%s\n", __func__);

	if (!smb1351_a_dev) {
		CHR_INFO("Warning: smb1351_a_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb1351_a_dev->lock);
	ret = smb1351_set_writable(smb1351_a_dev, true);
	if (ret < 0)
		goto out;

	/* set pre-charge current = 200mA: 01h[7:5]="000" */
	val = smb1351_read(smb1351_a_dev, SMB1351_OTHER_CHRG_CURR_REG);
	if (val < 0)
		goto out;
	val &= ~(BIT(7)|BIT(6)|BIT(5));
	ret = smb1351_write(smb1351_a_dev, SMB1351_OTHER_CHRG_CURR_REG, val);
	if (ret < 0)
		goto out;
	/* set fast charge current = 2400mA: 00h[7:4]="0111" */
	ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_2400MA);
	if (ret < 0)
		goto out;
	/* set soft cold current compensationt = 1000mA: 0Eh[5]="1" */
	val = smb1351_read(smb1351_a_dev, SMB1351_VAR_FUNC_REG);
	if (val < 0)
		goto out;
	val |= (BIT(5));
	ret = smb1351_write(smb1351_a_dev, SMB1351_VAR_FUNC_REG, val);
	if (ret < 0)
		goto out;
	/* set charger disable: 06h[6:5]="10"  */
	ret = smb1351_set_charging(smb1351_a_dev, false);
	if (ret < 0)
		goto out;
	/* set charger voltage=4.38V: 03h[5:0]="101100" */
	ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
	if (ret < 0)
		goto out;
	/* set charger enable: 06h[6:5]="11" */
	ret = smb1351_set_charging(smb1351_a_dev, true);
	if (ret < 0)
		goto out;
	/* set HVDCP=5V: 12h[7:6]="00" */
	ret = smb1351_set_hvdcp_voltage(smb1351_a_dev, HVDCP_5V);
	if (ret < 0)
		goto out;
	if (Read_HW_ID()==HW_ID_EVB) {
		/* set adaptor identification mode = normal mode: 14h[1:0]="00" */
		val = smb1351_read(smb1351_a_dev, SMB1351_OTG_MODE_PWR_REG);
		if (val < 0)
			goto out;
		val &= ~(BIT(1)|BIT(0));
		ret = smb1351_write(smb1351_a_dev, SMB1351_OTG_MODE_PWR_REG, val);
		if (ret < 0)
			goto out;
	}
	/* set watch dog timer enable: 08h[0]="1" */
	val = smb1351_read(smb1351_a_dev, SMB1351_WTD_CTRL_REG);
	if (val < 0)
		goto out;
	val |= (BIT(0));
	ret = smb1351_write(smb1351_a_dev, SMB1351_WTD_CTRL_REG, val);
	if (ret < 0)
		goto out;
	if (Read_HW_ID()==HW_ID_EVB) {
		/* set Fsw=1Mhz: 0Ah[7:6]="01" */
		val = smb1351_read(smb1351_a_dev, SMB1351_OTG_CTRL_REG);
		if (val < 0)
			goto out;
		val &= ~(BIT(7));
		val |= (BIT(6));
		ret = smb1351_write(smb1351_a_dev, SMB1351_OTG_CTRL_REG, val);
		if (ret < 0)
			goto out;
	}
	//CHR_INFO("%s done successfully\n", __func__);
	mutex_unlock(&smb1351_a_dev->lock);
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	mutex_unlock(&smb1351_a_dev->lock);
	return ret;
}

static int smb1351_parallel_charging_init(void)
{
	int ret=0, val=0;

	CHR_INFO("%s\n", __func__);

	if (!smb1351_b_dev) {
		CHR_INFO("Warning: smb1351_b_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb1351_b_dev->lock);
	ret = smb1351_set_writable(smb1351_b_dev, true);
	if (ret < 0)
		goto out;

	/* set charger suspend: 31h[6]="1" */
	val = smb1351_read(smb1351_b_dev, SMB1351_IL_CMD_REG);
	if (val < 0)
		goto out;
	val |= (BIT(6));
	ret = smb1351_write(smb1351_b_dev, SMB1351_IL_CMD_REG, val);
	if (ret < 0)
		goto out;
	/* set charger voltage=4.38V: 03h[5:0]="101100" */
	ret = smb1351_set_voltage(smb1351_b_dev, V_4_38);
	if (ret < 0)
		goto out;
	/* set HC mode: 31h[3]&&[0]="1" */
	val = smb1351_read(smb1351_b_dev, SMB1351_IL_CMD_REG);
	if (val < 0)
		goto out;
	val |= (BIT(0)|BIT(3));
	ret = smb1351_write(smb1351_b_dev, SMB1351_IL_CMD_REG, val);
	if (ret < 0)
		goto out;
	/* disable AICL */
	ret = smb1351_aicl(smb1351_b_dev, false);
	if (ret < 0)
		goto out;
	if (Read_HW_ID()==HW_ID_EVB) {
		/* set adaptor identification mode = normal mode: 14h[1:0]="00" */
		val = smb1351_read(smb1351_b_dev, SMB1351_OTG_MODE_PWR_REG);
		if (val < 0)
			goto out;
		val &= ~(BIT(1)|BIT(0));
		ret = smb1351_write(smb1351_b_dev, SMB1351_OTG_MODE_PWR_REG, val);
		if (ret < 0)
			goto out;
	}
	/* set watch dog timer enable: 08h[0]="1" */
	val = smb1351_read(smb1351_b_dev, SMB1351_WTD_CTRL_REG);
	if (val < 0)
		goto out;
	val |= (BIT(0));
	ret = smb1351_write(smb1351_b_dev, SMB1351_WTD_CTRL_REG, val);
	if (ret < 0)
		goto out;
	if (Read_HW_ID()==HW_ID_EVB) {
		/* set Fsw=1Mhz: 0Ah[7:6]="01" */
		val = smb1351_read(smb1351_b_dev, SMB1351_OTG_CTRL_REG);
		if (val < 0)
			goto out;
		val &= ~(BIT(7));
		val |= (BIT(6));
		ret = smb1351_write(smb1351_b_dev, SMB1351_OTG_CTRL_REG, val);
		if (ret < 0)
			goto out;
	}
	//CHR_INFO("%s done successfully\n", __func__);
	mutex_unlock(&smb1351_b_dev->lock);
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	mutex_unlock(&smb1351_b_dev->lock);
	return ret;
}

static int smb1351_set_inputI(struct smb1351_charger *smb, int type)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	switch (type) {
		case IUSB_IN_500MA:
			CHR_INFO("%s: set usb in current 500mA\n", __func__);
			/* set usb in I = 500mA: 00h[3:0]="0000" */
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(3)|BIT(2)|BIT(1)|BIT(0));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case IUSB_IN_1000MA:
			CHR_INFO("%s: set usb in current 1000mA\n", __func__);
			/* set usb in I = 1000mA: 00h[3:0]="0010" */
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val |= (BIT(1));
			val &= ~(BIT(3)|BIT(2)|BIT(0));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case IUSB_IN_1500MA:
			CHR_INFO("%s: set usb in current 1500mA\n", __func__);
			/* set usb in I = 1500mA: 00h[3:0]="0110" */
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val |= (BIT(2)|BIT(1));
			val &= ~(BIT(3)|BIT(0));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		case IUSB_IN_2000MA:
			CHR_INFO("%s: set usb in current 2000mA\n", __func__);
			/* set usb in I = 1500mA: 00h[3:0]="1010" */
			val = smb1351_read(smb, SMB1351_FAST_CHRG_CURR_REG);
			if (val < 0)
				goto out;
			val |= (BIT(3)|BIT(1));
			val &= ~(BIT(2)|BIT(0));
			ret = smb1351_write(smb, SMB1351_FAST_CHRG_CURR_REG, val);
			if (ret < 0)
				goto out;
			break;
		default:
			return -EINVAL;
	}
	//CHR_INFO("%s done successfully\n", __func__);
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

int smb1351_otg(int on)
{
	int ret=0, val=0;
	uint8_t ctrldata=0;

	if (!smb1351_a_dev) {
		CHR_INFO("%s: Warning: smb1351_a_dev is null due to probe function has error\n", __func__);
		return 1;
	}

	CHR_INFO("%s: set otg: %s\n", __func__, on ? "ON" : "OFF");
	mutex_lock(&smb1351_a_dev->lock);
	ret = smb1351_set_writable(smb1351_a_dev, true);
	if (ret < 0)
		goto out;

	if (on) {
		/* Default OTG current limit 500mA: 0Ah[3:2]="01" */
		val = smb1351_read(smb1351_a_dev, SMB1351_OTG_CTRL_REG);
		if (val < 0)
			goto out;
		val |= (BIT(2));
		val &= ~(BIT(3));
		ret = smb1351_write(smb1351_a_dev, SMB1351_OTG_CTRL_REG, val);
		if (ret < 0)
			goto out;
		/* Enable OTG function: PMIC GPIO 6 = 1 */
		ret = intel_scu_ipc_ioread8(PMIC_GPIO6_CTLO, &ctrldata);
		if (ret) {
			CHR_ERR("IPC Failed to read PMIC_GPIO6: %d\n", ret);
			goto out;
		}
		ctrldata |= (BIT(0)|BIT(4)|BIT(5));
		ret = intel_scu_ipc_iowrite8(PMIC_GPIO6_CTLO, ctrldata);
		if (ret) {
			CHR_ERR("IPC Failed to write PMIC_GPIO6: %d\n", ret);
			goto out;
		}
		/* Set OTG current limit 1000mA: 0Ah[3:2]="11" */
		val = smb1351_read(smb1351_a_dev, SMB1351_OTG_CTRL_REG);
		if (val < 0)
			goto out;
		val |= (BIT(2)|BIT(3));
		ret = smb1351_write(smb1351_a_dev, SMB1351_OTG_CTRL_REG, val);
		if (ret < 0)
			goto out;
	} else {
		/* Set OTG current limit 500mA: 0Ah[3:2]="01" */
		val = smb1351_read(smb1351_a_dev, SMB1351_OTG_CTRL_REG);
		if (val < 0)
			goto out;
		val |= (BIT(2));
		val &= ~(BIT(3));
		ret = smb1351_write(smb1351_a_dev, SMB1351_OTG_CTRL_REG, val);
		if (ret < 0)
			goto out;
		/* Disable OTG function: PMIC GPIO 6 = 0 */
		ret = intel_scu_ipc_ioread8(PMIC_GPIO6_CTLO, &ctrldata);
		if (ret) {
			CHR_ERR("IPC Failed to read PMIC_GPIO6: %d\n", ret);
			goto out;
		}
		ctrldata &= ~(BIT(0)|BIT(4)|BIT(5));
		ret = intel_scu_ipc_iowrite8(PMIC_GPIO6_CTLO, ctrldata);
		if (ret) {
			CHR_ERR("IPC Failed to write PMIC_GPIO6: %d\n", ret);
			goto out;
		}
	}
	//CHR_INFO("%s done successfully\n", __func__);
	mutex_unlock(&smb1351_a_dev->lock);
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	mutex_unlock(&smb1351_a_dev->lock);
	return ret;
}
EXPORT_SYMBOL(smb1351_otg);

static irqreturn_t smb1351_inok_interrupt(int irq, void *data)
{
	struct smb1351_charger *smb = data;

	if (gpio_get_value(smb->inok_gpio)) {
		CHR_INFO( "%s: INOK pin HIGH\n", __func__);
		pmic_handle_low_supply();
	}
	else {
		CHR_INFO("%s: INOK pin LOW\n", __func__);
	}
	return 0;
}

static int smb1351_inok_gpio_init(struct smb1351_charger *smb)
{
	int ret, irq = gpio_to_irq(smb->inok_gpio);

	CHR_INFO("%s\n", __func__);
	ret = gpio_request_one(smb->inok_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0) {
		CHR_INFO("request INOK gpio fail!\n");
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL, smb1351_inok_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					smb->client->name,
					smb);
	if (ret < 0) {
		CHR_INFO("config INOK gpio as IRQ fail!\n");
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(smb->inok_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static int smb1351_handle_notification(struct notifier_block *nb,
				   unsigned long event, void *param)
{
	struct power_supply_cable_props *cap;

	cap = param;
	CHR_INFO("%s: usb phy event = %ld, system charging mode now = %d\n", __func__, event, chrg_mode);
	if (event==USB_EVENT_CHARGER) {
		CHR_INFO("usb dedicated charger, chrg_evt=%u, chrg_type=%u\n", cap->chrg_evt, cap->chrg_type);
		if (cap->chrg_evt==POWER_SUPPLY_CHARGER_EVENT_DISCONNECT) {
			if (chrg_mode!=-1) {
				CHR_INFO("charging type: CABLE OUT\n");
				chrg_mode = -1;
				cancel_delayed_work(&smb1351_charging_work);
				queue_delayed_work(smb1351_delay_wq, &smb1351_charging_work,
					SMB1351_CHARGING_INIT_TIME);
				/* updatet charging status immediately */
				schedule_delayed_work(&smb1351_charging_status_work,
					SMB1351_CHARGING_STAUTS_TIME);
				/* release wake lock when cable out and hold timeout to a safe value of 5s(the same as pmic) to let panel on */
				if (wake_lock_active(&smb1351_a_dev->wakelock_cable)) {
					CHR_INFO("release wake lock and hold another wake lock t for 5s\n");
					wake_lock_timeout(&smb1351_a_dev->wakelock_cable_t, 5*HZ);
					wake_unlock(&smb1351_a_dev->wakelock_cable);
				}else {
					wake_lock_timeout(&smb1351_a_dev->wakelock_cable_t, 5*HZ);
				}
			}
		} else if ((cap->chrg_evt==POWER_SUPPLY_CHARGER_EVENT_CONNECT)&&(!otg_mode)) {
			/* hold wake lock when cable in for SW Jeita */
			if (!wake_lock_active(&smb1351_a_dev->wakelock_cable)) {
				CHR_INFO("hold wake lock\n");
				wake_lock(&smb1351_a_dev->wakelock_cable);
			}
			switch (cap->chrg_type) {
				case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
					if (chrg_mode!=POWER_SUPPLY_CHARGER_TYPE_USB_SDP) {
						CHR_INFO("charging type: SDP\n");
						chrg_mode = POWER_SUPPLY_CHARGER_TYPE_USB_SDP;
						queue_delayed_work(smb1351_delay_wq, &smb1351_charging_work,
							SMB1351_CHARGING_INIT_TIME);
						/* updatet charging status immediately */
						schedule_delayed_work(&smb1351_charging_status_work,
							SMB1351_CHARGING_STAUTS_TIME);
					}
					break;
				case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
					if (chrg_mode!=POWER_SUPPLY_CHARGER_TYPE_USB_DCP) {
						CHR_INFO("charging type: DCP\n");
						chrg_mode = POWER_SUPPLY_CHARGER_TYPE_USB_DCP;
						queue_delayed_work(smb1351_delay_wq, &smb1351_charging_work,
							SMB1351_CHARGING_INIT_TIME);
						/* updatet charging status immediately */
						schedule_delayed_work(&smb1351_charging_status_work,
							SMB1351_CHARGING_STAUTS_TIME);
					}
					break;
				case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
					if (chrg_mode!=POWER_SUPPLY_CHARGER_TYPE_USB_CDP) {
						CHR_INFO("charging type: CDP\n");
						chrg_mode = POWER_SUPPLY_CHARGER_TYPE_USB_CDP;
						queue_delayed_work(smb1351_delay_wq, &smb1351_charging_work,
							SMB1351_CHARGING_INIT_TIME);
						/* updatet charging status immediately */
						schedule_delayed_work(&smb1351_charging_status_work,
							SMB1351_CHARGING_STAUTS_TIME);
					}
					break;
				default:
					if (chrg_mode!=POWER_SUPPLY_CHARGER_TYPE_NONE) {
						CHR_INFO("charging type: Others\n");
						chrg_mode = POWER_SUPPLY_CHARGER_TYPE_NONE;
						queue_delayed_work(smb1351_delay_wq, &smb1351_charging_work,
							SMB1351_CHARGING_INIT_TIME);
						/* updatet charging status immediately */
						schedule_delayed_work(&smb1351_charging_status_work,
							SMB1351_CHARGING_STAUTS_TIME);
					}
					break;
			}
		}
	} else if (event==USB_EVENT_ID) {
		CHR_INFO("id pin event: %s\n", cap->chrg_evt ? "Detected" : "Removed");
		otg_mode = cap->chrg_evt;
		queue_delayed_work(smb1351_delay_wq, &smb1351_otg_work,
			SMB1351_OTG_INIT_TIME);
	}
	return NOTIFY_OK;
}

static inline int smb1351_register_otg_notification(struct smb1351_charger *smb)
{

	int retval;

	CHR_INFO("%s\n", __func__);
	smb->chr_type_nb.notifier_call = smb1351_handle_notification;
	/* Get the USB transceiver instance */
	smb->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);
	if (!smb->transceiver) {
		CHR_ERR("Failed to get the USB transceiver\n");
		return -EINVAL;
	}
	retval = usb_register_notifier(smb->transceiver, &smb->chr_type_nb);
	if (retval) {
		CHR_ERR("failed to register notifier\n");
		return -EINVAL;
	}

	return 0;
}

static void smb1351_charging_status_func(struct work_struct *work)
{
	struct power_supply *psy;
	union power_supply_propval p_val;

	CHR_INFO("%s\n", __func__);

	power_supply_changed(&smb1351_a_dev->mains);
	power_supply_changed(&smb1351_a_dev->usb);
	psy = power_supply_get_by_name(smb1351_power_supplied_to[0]);
	if (!psy) {
		CHR_INFO("get battery power_supply error!\n");
	} else {
		if (chrg_mode==-1)
			psy->set_property(psy, POWER_SUPPLY_STATUS_NOT_CHARGING, &p_val);
		else
			psy->set_property(psy, POWER_SUPPLY_STATUS_CHARGING, &p_val);
	}
}

static int type_c_dfp_setting(void)
{
	int curr;

	CHR_INFO("%s\n", __func__);
	/* wait 10s */
	msleep(10000);
	/* read CC logic ic */
	curr = core_get_advertised_current();
	CHR_INFO("type C result: current = %d\n", curr);
	switch (curr) {
		case 0:
		case 500:
			return TYPE_C_OTHERS;
		case 1500:
			return TYPE_C_1_5_A;
		case 3000:
			return TYPE_C_3_A;
		default:
			return TYPE_C_PD;
	}
}

static int smb1351_typeC_check(int flag)
{
	int ret=0, val=0;

	CHR_INFO("%s, flag = %d\n", __func__, flag);
	switch (flag) {
		case TYPE_C_1_5_A:
			/* check if AICL done: 36h[7]=1&&aicl result<=1300mA*/
			val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
			if (val < 0)
				goto out;
			if ((val&(BIT(7)))&&(val<=0x05)) {
				if (chrg_mode==POWER_SUPPLY_CHARGER_TYPE_USB_SDP) {
					/* set IUSB_IN = 500mA: 00h[3:0]="0000" */
					ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_500MA);
					if (ret < 0)
						goto out;
				} else {
					/* set IUSB_IN = 1000mA: 00h[3:0]="0010" */
					ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1000MA);
					if (ret < 0)
						goto out;
				}
			}
			break;
		case TYPE_C_3_A:
			/* check if AICL done: 36h[7]=1&&aicl result<=500mA*/
			val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
			if (val < 0)
				goto out;
			if ((val&(BIT(7)))&&(val<=0x00)) {
				dual_charger_flag = DUALCHR_NONE;
				if (chrg_mode==POWER_SUPPLY_CHARGER_TYPE_USB_SDP) {
					/* set IUSB_IN = 500mA: 00h[3:0]="0000" */
					ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_500MA);
					if (ret < 0)
						goto out;
				} else {
					/* set IUSB_IN = 1000mA: 00h[3:0]="0010" */
					ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1000MA);
					if (ret < 0)
						goto out;
				}
			} else {
				dual_charger_flag = DUALCHR_TYPE_C_3A;
			}
			break;
		default:
			CHR_INFO("not valid typeC type\n");
			break;
	}
	return 0;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_typeC_setting(int flag)
{
	int ret=0, val=0;

	CHR_INFO("%s, flag = %d\n", __func__, flag);
	switch (flag) {
		case TYPE_C_OTHERS:
			break;
		case TYPE_C_1_5_A:
			/* set HC mode: 31h[3]&&[0]="1" */
			val = smb1351_read(smb1351_a_dev, SMB1351_IL_CMD_REG);
			if (val < 0)
				goto out;
			val |= (BIT(0)|BIT(3));
			ret = smb1351_write(smb1351_a_dev, SMB1351_IL_CMD_REG, val);
			if (ret < 0)
				goto out;
			/* disable AICL */
			ret = smb1351_aicl(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set IUSB_IN = 1500mA: 00h[3:0]="0110" */
			ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1500MA);
			if (ret < 0)
				goto out;
			/* enable AICL */
			ret = smb1351_aicl(smb1351_a_dev, true);
			if (ret < 0)
				goto out;
			/* delay 2s */
			msleep(2000);
			ret = smb1351_typeC_check(flag);
			if (ret < 0)
				goto out;
			break;
		case TYPE_C_3_A:
			/* set HC mode: 31h[3]&&[0]="1" */
			val = smb1351_read(smb1351_a_dev, SMB1351_IL_CMD_REG);
			if (val < 0)
				goto out;
			val |= (BIT(0)|BIT(3));
			ret = smb1351_write(smb1351_a_dev, SMB1351_IL_CMD_REG, val);
			if (ret < 0)
				goto out;
			/* disable AICL */
			ret = smb1351_aicl(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set IUSB_IN = 1500mA: 00h[3:0]="0110" */
			ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1500MA);
			if (ret < 0)
				goto out;
			/* enable AICL */
			ret = smb1351_aicl(smb1351_a_dev, true);
			if (ret < 0)
				goto out;
			/* delay 2s */
			msleep(2000);
			ret = smb1351_typeC_check(flag);
			if (ret < 0)
				goto out;
			break;
		default:
			CHR_INFO("not valid typeC type\n");
			break;
	}
	return 0;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static void smb1351_detect_ASUS_adapter_func(struct work_struct *work)
{
	int ret=0, val=0, vadc=0;
	uint8_t ctrldata=0;

	CHR_INFO("%s\n", __func__);

	/* set charger suspend: 31h[6]="1" */
	val = smb1351_read(smb1351_a_dev, SMB1351_IL_CMD_REG);
	if (val < 0)
		goto out;
	val |= (BIT(6));
	ret = smb1351_write(smb1351_a_dev, SMB1351_IL_CMD_REG, val);
	if (ret < 0)
		goto out;
	/* delay 5ms */
	msleep(5);
	/* check ADC value */
	if (0) {
		/* set ADS1013 continuous conversion mode: 01h[8] = 0*/
		vadc = i2c_smbus_read_word_data(ads1013_i2c_dev->client, ADS1013_CONFIG_REG);
		if (vadc < 0) {
			CHR_ERR("failed to read ads1013 reg 0x01: %d\n", vadc);
			goto out;
		}
		vadc &= ~(BIT(8));
		ret = i2c_smbus_write_word_data(ads1013_i2c_dev->client, ADS1013_CONFIG_REG, vadc);
		if (ret < 0) {
			CHR_ERR("failed to write ads1013 reg 0x01: %d\n", ret);
			goto out;
		}
		vadc = i2c_smbus_read_word_data(ads1013_i2c_dev->client, ADS1013_CONVERSION_REG);
		if (vadc < 0) {
			CHR_ERR("failed to read ads1013 reg 0x00: %d\n", vadc);
			goto out;
		}
		CHR_INFO("ads1013 vadc first read = 0x%04X\n", vadc);
		if (vadc < 0x2600) {
			/* Vadc < 0.6V */
			/* SOC GPIO "GP144_ULPI_D2 = "H" */
			gpio_direction_output(smb1351_a_dev->ulpi_d2_gpio, 1);
			/* delay 5ms */
			msleep(5);
			vadc = i2c_smbus_read_word_data(ads1013_i2c_dev->client, ADS1013_CONVERSION_REG);
			if (vadc < 0) {
				CHR_ERR("failed to read ads1013 reg 0x00: %d\n", vadc);
				goto out;
			}
			CHR_INFO("ads1013 vadc second read = 0x%04X\n", vadc);
			if (vadc <= 0x7C60) {
				if ((vadc >= 0x707)&&(vadc <= 0x7CF)) {
					CHR_INFO("detect DUALCHR_ASUS_2A(5V/2A)!!!\n");
					dual_charger_flag = DUALCHR_ASUS_2A;
				} else if ((vadc >= 0x308)&&(vadc <= 0x3D0)) {
					CHR_INFO("detect DUALCHR_ASUS_2A(9V/2A)!!!\n");
					dual_charger_flag = DUALCHR_ASUS_2A;
				}
			}
		} else if (vadc > 0x7C60) {
			CHR_INFO("detect DUALCHR_ASUS_2A!!!\n");
			dual_charger_flag = DUALCHR_ASUS_2A;
		}
	} else {
		vadc = i2c_smbus_read_byte_data(us5587_i2c_dev->client, US5587_ADC_REG);
		if (vadc < 0) {
			CHR_ERR("failed to read us5587 reg 0x04: %d\n", vadc);
			goto out;
		}
		CHR_INFO("us5587 vadc first read = 0x%02X\n", vadc);
		if (vadc < 0x40) {
			/* Vadc < 0.6V */
			/* SOC GPIO "GP144_ULPI_D2 = "H" */
			gpio_direction_output(smb1351_a_dev->ulpi_d2_gpio, 1);
			/* delay 5ms */
			msleep(5);
			vadc = i2c_smbus_read_byte_data(us5587_i2c_dev->client, US5587_ADC_REG);
			if (vadc < 0) {
				CHR_ERR("failed to read us5587 reg 0x04: %d\n", vadc);
				goto out;
			}
			CHR_INFO("us5587 vadc second read = 0x%02X\n", vadc);
			if (vadc <= 0xDF) {
				if ((vadc >= 0xBA)&&(vadc <= 0xCF)) {
					CHR_INFO("detect DUALCHR_ASUS_2A(5V/2A)!!!\n");
					dual_charger_flag = DUALCHR_ASUS_2A;
				} else if ((vadc >= 0x41)&&(vadc <= 0x64)) {
					CHR_INFO("detect DUALCHR_ASUS_2A(9V/2A)!!!\n");
					dual_charger_flag = DUALCHR_ASUS_2A;
				}
			}
		} else if (vadc > 0xDF) {
			CHR_INFO("detect DUALCHR_ASUS_2A!!!\n");
			dual_charger_flag = DUALCHR_ASUS_2A;
		}
	}
out:
	/* smb1351 work: 31h[6]="0" */
	val = smb1351_read(smb1351_a_dev, SMB1351_IL_CMD_REG);
	if (val < 0)
		CHR_ERR("%s failed!!!\n", __func__);
	val &= ~(BIT(6));
	ret = smb1351_write(smb1351_a_dev, SMB1351_IL_CMD_REG, val);
	if (ret < 0)
		CHR_ERR("%s failed!!!\n", __func__);
	/* USB DPDM Switch to SMB: PMIC GPIO 2/3 = L */
	if (Read_HW_ID()==HW_ID_EVB) {
		ret = intel_scu_ipc_ioread8(PMIC_GPIO3_CTLO, &ctrldata);
	} else {
		ret = intel_scu_ipc_ioread8(PMIC_GPIO2_CTLO, &ctrldata);
	}
	if (ret)
		CHR_ERR("IPC Failed to read PMIC_GPIO2/3: %d\n", ret);
	ctrldata &= ~(BIT(0)|BIT(4)|BIT(5));
	if (Read_HW_ID()==HW_ID_EVB) {
		ret = intel_scu_ipc_iowrite8(PMIC_GPIO3_CTLO, ctrldata);
	} else {
		ret = intel_scu_ipc_iowrite8(PMIC_GPIO2_CTLO, ctrldata);
	}
	if (ret)
		CHR_ERR("IPC Failed to write PMIC_GPIO2/3: %d\n", ret);
	/* pull-low 1M ohm: SOC GPIO "GP144_ULPI_D2 = "L" */
	gpio_direction_output(smb1351_a_dev->ulpi_d2_gpio, 0);
	/* =======================
	  * ASUS Adapter Detection End
	  * =======================
	*/
	/* check if DFP Type=PD */
	type_c_flag = type_c_dfp_setting(); //need to be done
	if ((type_c_flag!=TYPE_C_PD)||((type_c_flag==TYPE_C_PD)&&hvdcp_flag)) {
		/* check BC1.2 Flow Define Result: HVDCP/ASUS Adapter/Apple mode? */
		if ((dual_charger_flag==DUALCHR_NONE)&&(!hvdcp_flag)) {
			CHR_INFO("NOT HVDCP/ASUS Adapter/Apple mode, start TYPE C DFP setting\n");
			ret = smb1351_typeC_setting(type_c_flag);
			if (ret < 0)
				goto out;
		} else if (hvdcp_flag) {
			/* set HVDCP=9V: 12h[7:6]="01" */
			ret = smb1351_set_hvdcp_voltage(smb1351_a_dev, HVDCP_9V);
			if (ret < 0)
				goto out;
		}
	} else {
		CHR_INFO("PD and NOT HVDCP, start TYPE C DFP setting\n");
		ret = smb1351_typeC_setting(type_c_flag);
		if (ret < 0)
			goto out;
	}
	/* start SW jeita function */
	queue_delayed_work(smb1351_delay_wq, &smb1351_sw_jeita_work,
		SMB1351_SW_JEITA_INIT_TIME);
	if (parallel_charger_flag)
		queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_sw_jeita_work,
			SMB1351_SW_JEITA_INIT_TIME);
}

static void smb1351_charging_flow_func(struct work_struct *work)
{
	int ret=0, val=0;
	uint8_t ctrldata=0;

	CHR_INFO("%s and charging mode = %d\n", __func__, chrg_mode);

	switch (chrg_mode) {
		case -1:
			hvdcp_flag = 0;
			cancel_delayed_work(&smb1351_sw_jeita_work);
			cancel_delayed_work(&smb1351_detect_ASUS_adapter_work);
			if (parallel_charger_flag) {
				cancel_delayed_work(&smb1351_parallel_charging_work);
				cancel_delayed_work(&smb1351_parallel_sw_jeita_work);
			}
			/* reset PMIC GPIO: PMIC GPIO 1->L, PMIC GPIO 2/3->L */
			ret = intel_scu_ipc_ioread8(PMIC_GPIO1_CTLO, &ctrldata);
			if (ret) {
				CHR_ERR("IPC Failed to read PMIC_GPIO1: %d\n", ret);
				goto out;
			}
			ctrldata &= ~(BIT(0)|BIT(4)|BIT(5));
			ret = intel_scu_ipc_iowrite8(PMIC_GPIO1_CTLO, ctrldata);
			if (ret) {
				CHR_ERR("IPC Failed to write PMIC_GPIO1: %d\n", ret);
				goto out;
			}
			if (Read_HW_ID()==HW_ID_EVB) {
				ret = intel_scu_ipc_ioread8(PMIC_GPIO3_CTLO, &ctrldata);
			} else {
				ret = intel_scu_ipc_ioread8(PMIC_GPIO2_CTLO, &ctrldata);
			}
			if (ret) {
				CHR_ERR("IPC Failed to read PMIC_GPIO2/3: %d\n", ret);
				goto out;
			}
			ctrldata &= ~(BIT(0)|BIT(4)|BIT(5));
			if (Read_HW_ID()==HW_ID_EVB) {
				ret = intel_scu_ipc_iowrite8(PMIC_GPIO3_CTLO, ctrldata);
			} else {
				ret = intel_scu_ipc_iowrite8(PMIC_GPIO2_CTLO, ctrldata);
			}
			if (ret) {
				CHR_ERR("IPC Failed to write PMIC_GPIO2/3: %d\n", ret);
				goto out;
			}
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
			smb1351_charging_init();
			type_c_flag = type_c_dfp_setting(); //need to be done
			if (type_c_flag!=TYPE_C_OTHERS) {
				ret = smb1351_typeC_setting(type_c_flag);
				if (ret < 0)
					goto out;
			}
			/* synchronize porting parallel charger*/
			if (parallel_charger_flag)
				queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_charging_work,
					SMB1351_CHARGING_INIT_TIME);
			/* start SW jeita function */
			queue_delayed_work(smb1351_delay_wq, &smb1351_sw_jeita_work,
				SMB1351_SW_JEITA_INIT_TIME);
			if (parallel_charger_flag)
				queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_sw_jeita_work,
					SMB1351_SW_JEITA_INIT_TIME);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
			smb1351_charging_init();
			/* synchronize porting parallel charger*/
			if (parallel_charger_flag)
				queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_charging_work,
					SMB1351_CHARGING_INIT_TIME);
			/* set USB switch D+/D- to smb1351: PMIC GPIO 1->H, PMIC GPIO 2/3->L */
			ret = intel_scu_ipc_ioread8(PMIC_GPIO1_CTLO, &ctrldata);
			if (ret) {
				CHR_ERR("IPC Failed to read PMIC_GPIO1: %d\n", ret);
				goto out;
			}
			ctrldata |= (BIT(0)|BIT(4)|BIT(5));
			ret = intel_scu_ipc_iowrite8(PMIC_GPIO1_CTLO, ctrldata);
			if (ret) {
				CHR_ERR("IPC Failed to write PMIC_GPIO1: %d\n", ret);
				goto out;
			}
			if (Read_HW_ID()==HW_ID_EVB) {
				ret = intel_scu_ipc_ioread8(PMIC_GPIO3_CTLO, &ctrldata);
			} else {
				ret = intel_scu_ipc_ioread8(PMIC_GPIO2_CTLO, &ctrldata);
			}
			if (ret) {
				CHR_ERR("IPC Failed to read PMIC_GPIO23: %d\n", ret);
				goto out;
			}
			ctrldata &= ~(BIT(0)|BIT(4)|BIT(5));
			if (Read_HW_ID()==HW_ID_EVB) {
				ret = intel_scu_ipc_iowrite8(PMIC_GPIO3_CTLO, ctrldata);
			} else {
				ret = intel_scu_ipc_iowrite8(PMIC_GPIO2_CTLO, ctrldata);
			}
			if (ret) {
				CHR_ERR("IPC Failed to write PMIC_GPIO2/3: %d\n", ret);
				goto out;
			}
			/* return APSD: 34h[7]="1" */
			val = smb1351_read(smb1351_a_dev, SMB1351_APSD_REG);
			if (val < 0)
				goto out;
			val |= (BIT(7));
			ret = smb1351_write(smb1351_a_dev, SMB1351_APSD_REG, val);
			if (ret < 0)
				goto out;
			/* Delay 5s */
			msleep(5000);
			/* Force QC2.0 mode: 34h[5]="1" */
			val = smb1351_read(smb1351_a_dev, SMB1351_APSD_REG);
			if (val < 0)
				goto out;
			val |= (BIT(5));
			ret = smb1351_write(smb1351_a_dev, SMB1351_APSD_REG, val);
			if (ret < 0)
				goto out;
			/* disable AICL */
			ret = smb1351_aicl(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set IUSB_IN = 1000mA: 00h[3:0]="0010" */
			ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1000MA);
			if (ret < 0)
				goto out;
			/* enable AICL */
			ret = smb1351_aicl(smb1351_a_dev, true);
			if (ret < 0)
				goto out;
			/* detect HVDCP: read 3Dh[1]="1"(QC2.0) or 47h[4]="1"(QC3.0)*/
			CHR_INFO("check if HVDCP: 3Dh=0x%02X, 47h=0x%02X\n",
					smb1351_read(smb1351_a_dev, SMB1351_STATUS_7_REG),
					smb1351_read(smb1351_a_dev, SMB1351_IRQ_G_REG));
			if ((smb1351_read(smb1351_a_dev, SMB1351_STATUS_7_REG)&(BIT(1)))||
				(smb1351_read(smb1351_a_dev, SMB1351_IRQ_G_REG)&(BIT(4)))) {
				CHR_INFO("detect HVDCP\n");
				hvdcp_flag = 1;
			}
			/* =======================
			  * ASUS Adapter Detection Start
			  * =======================
			*/
			/* pull-low 1M ohm: SOC GPIO "GP144_ULPI_D2 = "L" */
			gpio_direction_output(smb1351_a_dev->ulpi_d2_gpio, 0);
			/* USB DPDM Switch to ADC: PMIC GPIO 3 = H */
			if (Read_HW_ID()==HW_ID_EVB) {
				ret = intel_scu_ipc_ioread8(PMIC_GPIO3_CTLO, &ctrldata);
			} else {
				ret = intel_scu_ipc_ioread8(PMIC_GPIO2_CTLO, &ctrldata);
			}
			if (ret) {
				CHR_ERR("IPC Failed to read PMIC_GPIO2/3: %d\n", ret);
				goto out;
			}
			ctrldata |= (BIT(0)|BIT(4)|BIT(5));
			if (Read_HW_ID()==HW_ID_EVB) {
				ret = intel_scu_ipc_iowrite8(PMIC_GPIO3_CTLO, ctrldata);
			} else {
				ret = intel_scu_ipc_iowrite8(PMIC_GPIO2_CTLO, ctrldata);
			}
			if (ret) {
				CHR_ERR("IPC Failed to write PMIC_GPIO2/3: %d\n", ret);
				goto out;
			}
			/* delay 30s */
			queue_delayed_work(smb1351_delay_wq, &smb1351_detect_ASUS_adapter_work,
				SMB1351_DETECT_ASUS_ADAPTER_TIME);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
			smb1351_charging_init();
			/* synchronize porting parallel charger*/
			if (parallel_charger_flag)
				queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_charging_work,
					SMB1351_CHARGING_INIT_TIME);
			/* set HC mode: 31h[3]&&[0]="1" */
			val = smb1351_read(smb1351_a_dev, SMB1351_IL_CMD_REG);
			if (val < 0)
				goto out;
			val |= (BIT(0)|BIT(3));
			ret = smb1351_write(smb1351_a_dev, SMB1351_IL_CMD_REG, val);
			if (ret < 0)
				goto out;
			/* disable AICL */
			ret = smb1351_aicl(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set IUSB_IN = 1000mA: 00h[3:0]="0010" */
			ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1000MA);
			if (ret < 0)
				goto out;
			/* enable AICL */
			ret = smb1351_aicl(smb1351_a_dev, true);
			if (ret < 0)
				goto out;
			type_c_flag = type_c_dfp_setting(); //need to be done
			if (type_c_flag==TYPE_C_OTHERS) {
				/* set HC mode: 31h[3]&&[0]="1" */
				val = smb1351_read(smb1351_a_dev, SMB1351_IL_CMD_REG);
				if (val < 0)
					goto out;
				val |= (BIT(0)|BIT(3));
				ret = smb1351_write(smb1351_a_dev, SMB1351_IL_CMD_REG, val);
				if (ret < 0)
					goto out;
				/* disable AICL */
				ret = smb1351_aicl(smb1351_a_dev, false);
				if (ret < 0)
					goto out;
				/* set IUSB_IN = 1500mA: 00h[3:0]="0110" */
				ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1500MA);
				if (ret < 0)
					goto out;
				/* enable AICL */
				ret = smb1351_aicl(smb1351_a_dev, true);
				if (ret < 0)
					goto out;
			} else {
				ret = smb1351_typeC_setting(type_c_flag);
				if (ret < 0)
					goto out;
			}
			/* start SW jeita function */
			queue_delayed_work(smb1351_delay_wq, &smb1351_sw_jeita_work,
				SMB1351_SW_JEITA_INIT_TIME);
			if (parallel_charger_flag)
				queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_sw_jeita_work,
					SMB1351_SW_JEITA_INIT_TIME);
			break;
		case POWER_SUPPLY_CHARGER_TYPE_NONE:
			smb1351_charging_init();
			/* synchronize porting parallel charger*/
			if (parallel_charger_flag)
				queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_charging_work,
					SMB1351_CHARGING_INIT_TIME);
			/* set HC mode: 31h[3]&&[0]="1" */
			val = smb1351_read(smb1351_a_dev, SMB1351_IL_CMD_REG);
			if (val < 0)
				goto out;
			val |= (BIT(0)|BIT(3));
			ret = smb1351_write(smb1351_a_dev, SMB1351_IL_CMD_REG, val);
			if (ret < 0)
				goto out;
			/* disable AICL */
			ret = smb1351_aicl(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set IUSB_IN = 1000mA: 00h[3:0]="0010" */
			ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1000MA);
			if (ret < 0)
				goto out;
			/* enable AICL */
			ret = smb1351_aicl(smb1351_a_dev, true);
			if (ret < 0)
				goto out;
			type_c_flag = type_c_dfp_setting(); //need to be done
			if (type_c_flag!=TYPE_C_OTHERS) {
				ret = smb1351_typeC_setting(type_c_flag);
				if (ret < 0)
					goto out;
			}
			/* start SW jeita function */
			queue_delayed_work(smb1351_delay_wq, &smb1351_sw_jeita_work,
				SMB1351_SW_JEITA_INIT_TIME);
			if (parallel_charger_flag)
				queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_sw_jeita_work,
					SMB1351_SW_JEITA_INIT_TIME);
			break;
		default:
			CHR_INFO("not valid charging type\n");
			break;
	}
	//CHR_INFO("%s done successfully\n", __func__);
	return;
out:
	CHR_ERR("%s failed!!!\n", __func__);
}

static void smb1351_parallel_charging_flow_func(struct work_struct *work)
{
	CHR_INFO("%s\n", __func__);

	smb1351_parallel_charging_init();
}

static int smb1351_parallel_enable(struct smb1351_charger *smb, int type)
{
	int ret=0, val=0;

	if (!smb) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	switch (type) {
		case Disable:
			CHR_INFO("%s: set parallel charger settings: Disable\n", __func__);
			/* set charger suspend: 31h[6]="1" */
			val = smb1351_read(smb, SMB1351_IL_CMD_REG);
			if (val < 0)
				goto out;
			val |= (BIT(6));
			ret = smb1351_write(smb, SMB1351_IL_CMD_REG, val);
			if (ret < 0)
				goto out;
			/* set charger disable: 06h[6:5]="11"(slave charger is different) */
			ret = smb1351_parallel_set_charging(smb, false);
			if (ret < 0)
				goto out;
			break;
		case Enable_1:
			CHR_INFO("%s: set parallel charger settings: Enable 1\n", __func__);
			/* set charger non-suspend: 31h[6]="0" */
			val = smb1351_read(smb, SMB1351_IL_CMD_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(6));
			ret = smb1351_write(smb, SMB1351_IL_CMD_REG, val);
			if (ret < 0)
				goto out;
			/* set IUSB_IN = 1000mA: 00h[3:0]="0010" */
			ret = smb1351_set_inputI(smb, IUSB_IN_1000MA);
			if (ret < 0)
				break;
			/* set fast charge current = 1800mA: 00h[7:4]="0100" */
			ret = smb1351_set_fastcharge_I(smb, FC_I_1800MA);
			if (ret < 0)
				goto out;
			/* set charger enable: 06h[6:5]="10"(slave charger is different)  */
			ret = smb1351_parallel_set_charging(smb, true);
			if (ret < 0)
				goto out;
			break;
		case Enable_2:
			CHR_INFO("%s: set parallel charger settings: Enable 2\n", __func__);
			/* set charger non-suspend: 31h[6]="0" */
			val = smb1351_read(smb, SMB1351_IL_CMD_REG);
			if (val < 0)
				goto out;
			val &= ~(BIT(6));
			ret = smb1351_write(smb, SMB1351_IL_CMD_REG, val);
			if (ret < 0)
				goto out;
			/* set IUSB_IN = 1500mA: 00h[3:0]="0110" */
			ret = smb1351_set_inputI(smb, IUSB_IN_1500MA);
			if (ret < 0)
				break;
			/* set fast charge current = 1800mA: 00h[7:4]="0100" */
			ret = smb1351_set_fastcharge_I(smb, FC_I_1800MA);
			if (ret < 0)
				goto out;
			/* set charger enable: 06h[6:5]="10"(slave charger is different)  */
			ret = smb1351_parallel_set_charging(smb, true);
			if (ret < 0)
				goto out;
			break;
		default:
			return -EINVAL;
	}
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static int smb1351_jeita_charging_settings(int temp, int vbat)
{
	int ret=0, val=0;

	if (!smb1351_a_dev) {
		CHR_INFO("%s: Warning: smb1351 dev is null due to probe function has error\n", __func__);
		return 1;
	}

	switch (temp) {
		case BELOW_0:
			CHR_INFO("%s: below 0oC\n", __func__);
			if (parallel_charger_flag) {
				ret = smb1351_parallel_enable(smb1351_b_dev, Disable);
				if (ret < 0)
					goto out;
			}
			/* check if AICL not done: 36h[7]=0&&dual-charger-flag=ASUS_2A/TYPE_C_3A&&hvdcp flag=0*/
			val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
			if (val < 0)
				goto out;
			if ( !(val&(BIT(7)))&&((dual_charger_flag==DUALCHR_ASUS_2A)||(dual_charger_flag==DUALCHR_TYPE_C_3A))&&(!hvdcp_flag)) {
				/* disable AICL */
				ret = smb1351_aicl(smb1351_a_dev, false);
				if (ret < 0)
					goto out;
				/* set IUSB_IN = 2000mA: 00h[3:0]="1010" */
				ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_2000MA);
				if (ret < 0)
					goto out;
				/* enable AICL */
				ret = smb1351_aicl(smb1351_a_dev, true);
				if (ret < 0)
					goto out;
			}
			/* set charger voltage=4.38V: 03h[5:0]="101100" */
			ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
			if (ret < 0)
				goto out;
			/* set charger disable: 06h[6:5]="10"  */
			ret = smb1351_set_charging(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set fast charge current = 1800mA: 00h[7:4]="0100" */
			ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_1800MA);
			if (ret < 0)
				goto out;
			break;
		case IN_0_200:
			CHR_INFO("%s: 0oC - 20oC\n", __func__);
			if (parallel_charger_flag) {
				ret = smb1351_parallel_enable(smb1351_b_dev, Disable);
				if (ret < 0)
					goto out;
			}
			/* check if AICL not done: 36h[7]=0&&dual-charger-flag=ASUS_2A/TYPE_C_3A&&hvdcp flag=0*/
			val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
			if (val < 0)
				goto out;
			if ( !(val&(BIT(7)))&&((dual_charger_flag==DUALCHR_ASUS_2A)||(dual_charger_flag==DUALCHR_TYPE_C_3A))&&(!hvdcp_flag)) {
				/* disable AICL */
				ret = smb1351_aicl(smb1351_a_dev, false);
				if (ret < 0)
					goto out;
				/* set IUSB_IN = 2000mA: 00h[3:0]="1010" */
				ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_2000MA);
				if (ret < 0)
					goto out;
				/* enable AICL */
				ret = smb1351_aicl(smb1351_a_dev, true);
				if (ret < 0)
					goto out;
			}
			/* set charger voltage=4.38V: 03h[5:0]="101100" */
			ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
			if (ret < 0)
				goto out;
			/* set charger enable: 06h[6:5]="11" */
			ret = smb1351_set_charging(smb1351_a_dev, true);
			if (ret < 0)
				goto out;
			/* set fast charge current = 1800mA: 00h[7:4]="0100" */
			ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_1800MA);
			if (ret < 0)
				goto out;
			break;
		case IN_200_500:
			CHR_INFO("%s: 20oC - 50oC\n", __func__);
			/* check dual-charger-flag */
			if ( dual_charger_flag==DUALCHR_NONE) {
				if (parallel_charger_flag) {
					ret = smb1351_parallel_enable(smb1351_b_dev, Disable);
					if (ret < 0)
						goto out;
				}
				/* set charger voltage=4.38V: 03h[5:0]="101100" */
				ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
				if (ret < 0)
					goto out;
				/* set charger enable: 06h[6:5]="11" */
				ret = smb1351_set_charging(smb1351_a_dev, true);
				if (ret < 0)
					goto out;
				/* set fast charge current = 2000mA: 00h[7:4]="1110" */
				ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_2000MA);
				if (ret < 0)
					goto out;
			} else if ( dual_charger_flag==DUALCHR_ASUS_2A) {
				if ((vbat>=3000)&&(vbat<4250)) {
					/* set charger voltage=4.38V: 03h[5:0]="101100" */
					ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
					if (ret < 0)
						goto out;
					/* set charger enable: 06h[6:5]="11" */
					ret = smb1351_set_charging(smb1351_a_dev, true);
					if (ret < 0)
						goto out;
					/* set fast charge current = 1800mA: 00h[7:4]="0100" */
					ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_1800MA);
					if (ret < 0)
						goto out;
					/* set IUSB_IN = 1000mA: 00h[3:0]="0010" */
					ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1000MA);
					if (ret < 0)
						goto out;
					if (parallel_charger_flag) {
						ret = smb1351_parallel_enable(smb1351_b_dev, Enable_1);
						if (ret < 0)
							goto out;
					}
				} else {
					if (parallel_charger_flag) {
						ret = smb1351_parallel_enable(smb1351_b_dev, Disable);
						if (ret < 0)
							goto out;
					}
					/* check if AICL not done: 36h[7]=0&&dual-charger-flag=ASUS_2A/TYPE_C_3A&&hvdcp flag=0*/
					val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
					if (val < 0)
						goto out;
					if ( !(val&(BIT(7)))&&((dual_charger_flag==DUALCHR_ASUS_2A)||(dual_charger_flag==DUALCHR_TYPE_C_3A))&&(!hvdcp_flag)) {
						/* disable AICL */
						ret = smb1351_aicl(smb1351_a_dev, false);
						if (ret < 0)
							goto out;
						/* set IUSB_IN = 2000mA: 00h[3:0]="1010" */
						ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_2000MA);
						if (ret < 0)
							goto out;
						/* enable AICL */
						ret = smb1351_aicl(smb1351_a_dev, true);
						if (ret < 0)
							goto out;
					}
				}
			} else if ( dual_charger_flag==DUALCHR_TYPE_C_3A) {
				if ((vbat>=3000)&&(vbat<4250)) {
					/* set charger voltage=4.38V: 03h[5:0]="101100" */
					ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
					if (ret < 0)
						goto out;
					/* set charger enable: 06h[6:5]="11" */
					ret = smb1351_set_charging(smb1351_a_dev, true);
					if (ret < 0)
						goto out;
					/* set fast charge current = 1800mA: 00h[7:4]="0100" */
					ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_1800MA);
					if (ret < 0)
						goto out;
					/* set IUSB_IN = 1500mA: 00h[3:0]="0110" */
					ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_1500MA);
					if (ret < 0)
						goto out;
					if (parallel_charger_flag) {
						ret = smb1351_parallel_enable(smb1351_b_dev, Enable_2);
						if (ret < 0)
							goto out;
					}
				} else {
					if (parallel_charger_flag) {
						ret = smb1351_parallel_enable(smb1351_b_dev, Disable);
						if (ret < 0)
							goto out;
					}
					/* check if AICL not done: 36h[7]=0&&dual-charger-flag=ASUS_2A/TYPE_C_3A&&hvdcp flag=0*/
					val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
					if (val < 0)
						goto out;
					if ( !(val&(BIT(7)))&&((dual_charger_flag==DUALCHR_ASUS_2A)||(dual_charger_flag==DUALCHR_TYPE_C_3A))&&(!hvdcp_flag)) {
						/* disable AICL */
						ret = smb1351_aicl(smb1351_a_dev, false);
						if (ret < 0)
							goto out;
						/* set IUSB_IN = 2000mA: 00h[3:0]="1010" */
						ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_2000MA);
						if (ret < 0)
							goto out;
						/* enable AICL */
						ret = smb1351_aicl(smb1351_a_dev, true);
						if (ret < 0)
							goto out;
					}
				}
			}
			break;
		case IN_500_600:
			CHR_INFO("%s: 50oC - 60oC\n", __func__);
			if (parallel_charger_flag) {
				ret = smb1351_parallel_enable(smb1351_b_dev, Disable);
				if (ret < 0)
					goto out;
			}
			/* check if AICL not done: 36h[7]=0&&dual-charger-flag=ASUS_2A/TYPE_C_3A&&hvdcp flag=0*/
			val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
			if (val < 0)
				goto out;
			if ( !(val&(BIT(7)))&&((dual_charger_flag==DUALCHR_ASUS_2A)||(dual_charger_flag==DUALCHR_TYPE_C_3A))&&(!hvdcp_flag)) {
				/* disable AICL */
				ret = smb1351_aicl(smb1351_a_dev, false);
				if (ret < 0)
					goto out;
				/* set IUSB_IN = 2000mA: 00h[3:0]="1010" */
				ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_2000MA);
				if (ret < 0)
					goto out;
				/* enable AICL */
				ret = smb1351_aicl(smb1351_a_dev, true);
				if (ret < 0)
					goto out;
			}
			/* check if Vchg==4.38V && Vbat>=4.1V*/
			if ((vchg_flag==V_4_38)&&(vbat>=4100)) {
				/* set charger voltage=4.38V: 03h[5:0]="101100" */
				ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
				if (ret < 0)
					goto out;
				/* set charger disable: 06h[6:5]="10"  */
				ret = smb1351_set_charging(smb1351_a_dev, false);
				if (ret < 0)
					goto out;
				/* set fast charge current = 1800mA: 00h[7:4]="0100" */
				ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_1800MA);
				if (ret < 0)
					goto out;
			} else {
				/* set charger voltage=4.1V: 03h[5:0]="011110" */
				ret = smb1351_set_voltage(smb1351_a_dev, V_4_1);
				if (ret < 0)
					goto out;
				/* set charger enable: 06h[6:5]="11" */
				ret = smb1351_set_charging(smb1351_a_dev, true);
				if (ret < 0)
					goto out;
				/* set fast charge current = 1800mA: 00h[7:4]="0100" */
				ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_1800MA);
				if (ret < 0)
					goto out;
			}
			break;
		case ABOVE_600:
			CHR_INFO("%s: above 60oC\n", __func__);
			if (parallel_charger_flag) {
				ret = smb1351_parallel_enable(smb1351_b_dev, Disable);
				if (ret < 0)
					goto out;
			}
			/* check if AICL not done: 36h[7]=0&&dual-charger-flag=ASUS_2A/TYPE_C_3A&&hvdcp flag=0*/
			val = smb1351_read(smb1351_a_dev, SMB1351_STATUS_REG);
			if (val < 0)
				goto out;
			if ( !(val&(BIT(7)))&&((dual_charger_flag==DUALCHR_ASUS_2A)||(dual_charger_flag==DUALCHR_TYPE_C_3A))&&(!hvdcp_flag)) {
				/* disable AICL */
				ret = smb1351_aicl(smb1351_a_dev, false);
				if (ret < 0)
					goto out;
				/* set IUSB_IN = 2000mA: 00h[3:0]="1010" */
				ret = smb1351_set_inputI(smb1351_a_dev, IUSB_IN_2000MA);
				if (ret < 0)
					goto out;
				/* enable AICL */
				ret = smb1351_aicl(smb1351_a_dev, true);
				if (ret < 0)
					goto out;
			}
			/* set charger voltage=4.38V: 03h[5:0]="101100" */
			ret = smb1351_set_voltage(smb1351_a_dev, V_4_38);
			if (ret < 0)
				goto out;
			/* set charger disable: 06h[6:5]="10"  */
			ret = smb1351_set_charging(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set fast charge current = 1800mA: 00h[7:4]="0100" */
			ret = smb1351_set_fastcharge_I(smb1351_a_dev, FC_I_1800MA);
			if (ret < 0)
				goto out;
			break;
		default:
			return -EINVAL;
	}
	//CHR_INFO("%s done successfully\n", __func__);
	return ret;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	return ret;
}

static void smb1351_dump_reg(void)
{
	int ret=0;
	u8 reg=0;

	CHR_INFO("%s\n", __func__);
	if (debug_flag) {
		CHR_INFO("+++++smb1351 main+++++\n");
		CHR_INFO("#Addr\t#Value\n");
		for (reg = 0; reg <= 0x16; reg++) {
			ret = smb1351_read(smb1351_a_dev, reg);
			CHR_INFO("0x%02X:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		for (reg = 0x25; reg <= 0x2E; reg++) {
			ret = smb1351_read(smb1351_a_dev, reg);
			CHR_INFO("0x%02X:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		for (reg = 0x30; reg <= 0x46; reg++) {
			ret = smb1351_read(smb1351_a_dev, reg);
			CHR_INFO("0x%02X:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		CHR_INFO("-----smb1351 main-----\n");
		CHR_INFO("+++++smb1351 slave+++++\n");
		CHR_INFO("#Addr\t#Value\n");
		for (reg = 0; reg <= 0x16; reg++) {
			ret = smb1351_read(smb1351_b_dev, reg);
			CHR_INFO("0x%02X:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		for (reg = 0x25; reg <= 0x2E; reg++) {
			ret = smb1351_read(smb1351_b_dev, reg);
			CHR_INFO("0x%02X:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		for (reg = 0x30; reg <= 0x46; reg++) {
			ret = smb1351_read(smb1351_b_dev, reg);
			CHR_INFO("0x%02X:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		CHR_INFO("-----smb1351 slave-----\n");
	}
}

static void smb1351_sw_jeita_set(struct work_struct *work)
{
	int ret=0, val=0, temp=0, volt=0, soc=0, chrg_status=0;
	struct power_supply *psy;
	union power_supply_propval p_val;

	CHR_INFO("%s\n", __func__);
	/*set hard hot limit = 72 oC: 0Bh[5:4]="11" */
	val = smb1351_read(smb1351_a_dev, SMB1351_SH_LIMIT_TEMP_REG);
	if (val < 0)
		goto out;
	val |= (BIT(5)|BIT(4));
	ret = smb1351_write(smb1351_a_dev, SMB1351_SH_LIMIT_TEMP_REG, val);
	if (ret < 0)
		goto out;
	/*set soft cold limit behavior = no response: 07h[3:2]="00" */
	val = smb1351_read(smb1351_a_dev, SMB1351_THERM_CTRL_A_REG);
	if (val < 0)
		goto out;
	val &= ~(BIT(3)|BIT(2));
	ret = smb1351_write(smb1351_a_dev, SMB1351_THERM_CTRL_A_REG, val);
	if (ret < 0)
		goto out;
	/*set soft hot temp limit  = no response: 07h[1:0]="00" */
	val = smb1351_read(smb1351_a_dev, SMB1351_THERM_CTRL_A_REG);
	if (val < 0)
		goto out;
	val &= ~(BIT(1)|BIT(0));
	ret = smb1351_write(smb1351_a_dev, SMB1351_THERM_CTRL_A_REG, val);
	if (ret < 0)
		goto out;
	/* get battery temperature from gauge */
	psy = power_supply_get_by_name(smb1351_power_supplied_to[0]);
	if (!psy) {
		CHR_INFO("get battery power_supply error!\n");
		temp = 250;
		volt = 3700;
		soc = 50;
	} else {
		psy->get_property(psy, POWER_SUPPLY_PROP_TEMP, &p_val);
		temp = p_val.intval;
		CHR_INFO("temp = %d\n", temp);
		psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &p_val);
		volt = p_val.intval/1000;
		CHR_INFO("volt = %d\n", volt);
		psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &p_val);
		soc = p_val.intval;
		CHR_INFO("soc = %d\n", soc);
	}
	if (temp<0) {
		temp_flag = BELOW_0;
	} else if ((temp>=0)&&(temp<200)) {
		if (!((temp_flag<=BELOW_0)&&(temp<30)))
			temp_flag = IN_0_200;
	} else if ((temp>=200)&&(temp<500)) {
		if (!((temp_flag<=IN_0_200)&&(temp<230))&&
			!((temp_flag>=IN_500_600)&&(temp>470)))
			temp_flag = IN_200_500;
	} else if ((temp>=500)&&(temp<600)) {
		if (!((temp_flag>=ABOVE_600)&&(temp>570)))
			temp_flag = IN_500_600;
	} else {
		temp_flag = ABOVE_600;
	}
	ret = smb1351_jeita_charging_settings(temp_flag, volt);
	if (ret < 0)
		goto out;
	/* recharge function */
	chrg_status = smb1351_get_charging_status();
	if (chrg_status==POWER_SUPPLY_STATUS_FULL) {
		if (soc<=98) {
			CHR_INFO("need to recharge\n");
			/* set charger disable: 06h[6:5]="10"  */
			ret = smb1351_set_charging(smb1351_a_dev, false);
			if (ret < 0)
				goto out;
			/* set charger enable: 06h[6:5]="11" */
			ret = smb1351_set_charging(smb1351_a_dev, true);
			if (ret < 0)
				goto out;
		} else if (soc<100) {
			CHR_INFO("full charged but soc is not 100, notify gauge\n");
			if (!psy) {
				CHR_INFO("get battery power_supply error!\n");
			} else {
				psy->set_property(psy, POWER_SUPPLY_STATUS_FULL, &p_val);
			}
		}
	}
#ifdef CONFIG_LEDS_ASUS
	/* set LED in charger mode*/
	if ((boot_mode==4)&&(Read_PROJ_ID()!=PROJ_ID_ZS550ML_SEC)) {
		CHR_INFO("in charger mode and charging status = %d, set LED\n", chrg_status);
		if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
			led_brightness_set(0, 0);
			led_brightness_set(1, 1);
		}else if ((chrg_status == POWER_SUPPLY_STATUS_CHARGING)||(chrg_status == POWER_SUPPLY_STATUS_QUICK_CHARGING)||(chrg_status == POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING)) {
			led_brightness_set(0, 1);
			led_brightness_set(1, 1);
		}else {
			led_brightness_set(0, 0);
			led_brightness_set(1, 0);
		}
	}
#endif
#ifdef CONFIG_LEDS_KTD2026
	/* set LED in charger mode*/
	if ((boot_mode==4)&&(Read_PROJ_ID()==PROJ_ID_ZS550ML_SEC)) {
		CHR_INFO("in charger mode and charging status = %d, set LED\n", chrg_status);
		if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
			ktd2026_led_brightness_set(0, 0);
			ktd2026_led_brightness_set(1, 1);
		}else if ((chrg_status == POWER_SUPPLY_STATUS_CHARGING)||(chrg_status == POWER_SUPPLY_STATUS_QUICK_CHARGING)||(chrg_status == POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING)) {
			ktd2026_led_brightness_set(0, 1);
			ktd2026_led_brightness_set(1, 1);
		}else {
			ktd2026_led_brightness_set(0, 0);
			ktd2026_led_brightness_set(1, 0);
		}
	}
#endif

	smb1351_dump_reg();
	//CHR_INFO("%s done successfully\n", __func__);
	queue_delayed_work(smb1351_delay_wq, &smb1351_sw_jeita_work, SMB1351_SW_JEITA_POLLING_TIME);
	return;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	queue_delayed_work(smb1351_delay_wq, &smb1351_sw_jeita_work, SMB1351_SW_JEITA_POLLING_TIME);
	return;
}

static void smb1351_parallel_sw_jeita_set(struct work_struct *work)
{
	int ret=0, val=0;

	CHR_INFO("%s\n", __func__);
	/*set hard hot limit = 72 oC: 0Bh[5:4]="11" */
	val = smb1351_read(smb1351_b_dev, SMB1351_SH_LIMIT_TEMP_REG);
	if (val < 0)
		goto out;
	val |= (BIT(5)|BIT(4));
	ret = smb1351_write(smb1351_b_dev, SMB1351_SH_LIMIT_TEMP_REG, val);
	if (ret < 0)
		goto out;
	/*set soft cold limit behavior = no response: 07h[3:2]="00" */
	val = smb1351_read(smb1351_b_dev, SMB1351_THERM_CTRL_A_REG);
	if (val < 0)
		goto out;
	val &= ~(BIT(3)|BIT(2));
	ret = smb1351_write(smb1351_b_dev, SMB1351_THERM_CTRL_A_REG, val);
	if (ret < 0)
		goto out;
	/*set soft hot temp limit  = no response: 07h[1:0]="00" */
	val = smb1351_read(smb1351_b_dev, SMB1351_THERM_CTRL_A_REG);
	if (val < 0)
		goto out;
	val &= ~(BIT(1)|BIT(0));
	ret = smb1351_write(smb1351_b_dev, SMB1351_THERM_CTRL_A_REG, val);
	if (ret < 0)
		goto out;
	//CHR_INFO("%s done successfully\n", __func__);
	queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_sw_jeita_work, SMB1351_SW_JEITA_POLLING_TIME);
	return;
out:
	CHR_ERR("%s failed!!!\n", __func__);
	queue_delayed_work(smb1351_parallel_delay_wq, &smb1351_parallel_sw_jeita_work, SMB1351_SW_JEITA_POLLING_TIME);
	return;
}

static void smb1351_otg_flow_func(struct work_struct *work)
{
	int ret=0;

	CHR_INFO("%s\n", __func__);

	if (otg_mode) {
		CHR_INFO("Enable 5V\n");
		ret = smb1351_otg(1);
		if (ret)
			CHR_ERR("failed to enable 5V\n");
	} else {
		CHR_INFO("Disable 5V\n");
		ret = smb1351_otg(0);
		if (ret)
			CHR_ERR("failed to disable 5V\n");
	}
}

static int smb1351_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	//struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct smb1351_charger *smb;
	int ret;

	CHR_INFO("%s +++\n", __func__);

	// cannot use this function when using pmic i2c so ignore
	//if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
	//	return -EIO;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;
	i2c_set_clientdata(client, smb);
	smb->client = client;

	/* initial mutex */
	mutex_init(&smb->lock);

	/* register power supply */
	smb->mains.name = "ac";
	smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
	smb->mains.get_property = smb1351_mains_get_property;
	smb->mains.properties = smb1351_mains_properties;
	smb->mains.num_properties = ARRAY_SIZE(smb1351_mains_properties);
	smb->mains.supplied_to = smb1351_power_supplied_to;
	smb->mains.num_supplicants =
			ARRAY_SIZE(smb1351_power_supplied_to);
	ret = power_supply_register(dev, &smb->mains);
	if (ret < 0)
		return ret;

	smb->usb.name = "usb";
	smb->usb.type = POWER_SUPPLY_TYPE_USB;
	smb->usb.get_property = smb1351_usb_get_property;
	smb->usb.properties = smb1351_usb_properties;
	smb->usb.num_properties = ARRAY_SIZE(smb1351_usb_properties);
	smb->usb.supplied_to = smb1351_power_supplied_to;
	smb->usb.num_supplicants = ARRAY_SIZE(smb1351_power_supplied_to);
	ret = power_supply_register(dev, &smb->usb);
	if (ret < 0) {
			power_supply_unregister(&smb->mains);
		return ret;
	}

	/* INOK pin configuration */
	smb->inok_gpio =  get_gpio_by_name(SMB1351_INOK_GPIO);
	if (smb->inok_gpio >= 0) {
		CHR_INFO("inok gpio = %d\n", smb->inok_gpio);
		ret = smb1351_inok_gpio_init(smb);
		if (ret < 0) {
			CHR_ERR("failed to initialize INOK gpio: %d\n", ret);
		}
	}
	smb->ulpi_d2_gpio=  get_gpio_by_name(SMB1351_ULPI_D2);
	if (smb->ulpi_d2_gpio >= 0) {
		CHR_INFO("ulpi d2 gpio = %d\n", smb->ulpi_d2_gpio);
		gpio_request(smb->ulpi_d2_gpio, SMB1351_ULPI_D2);
	}
	/* Register to get USB transceiver events */
	ret = smb1351_register_otg_notification(smb);
	if (ret) {
		CHR_ERR("Rrgister OTG notification failed\n");
	}
	/*initial work queue and works*/
	smb1351_delay_wq = create_workqueue("SMB1351_WORK_QUEUE");
	if (unlikely(!smb1351_delay_wq)) {
		CHR_ERR("unable to create smb1351 charger workqueue\n");
	} else {
		INIT_DELAYED_WORK(&smb1351_sw_jeita_work, smb1351_sw_jeita_set);
		INIT_DELAYED_WORK(&smb1351_charging_work, smb1351_charging_flow_func);
		INIT_DELAYED_WORK(&smb1351_detect_ASUS_adapter_work, smb1351_detect_ASUS_adapter_func);
		INIT_DELAYED_WORK(&smb1351_otg_work, smb1351_otg_flow_func);
	}
	smb1351_parallel_delay_wq = create_workqueue("SMB1351_PARALLEL_WORK_QUEUE");
	if (unlikely(!smb1351_parallel_delay_wq)) {
		CHR_ERR("unable to create smb1351 parallel charger workqueue\n");
	} else {
		INIT_DELAYED_WORK(&smb1351_parallel_charging_work, smb1351_parallel_charging_flow_func);
		INIT_DELAYED_WORK(&smb1351_parallel_sw_jeita_work, smb1351_parallel_sw_jeita_set);
	}
	INIT_DELAYED_WORK(&smb1351_charging_status_work, smb1351_charging_status_func);
	/* init wakelock */
	wake_lock_init(&smb->wakelock_cable, WAKE_LOCK_SUSPEND, "cable_wakelock");
	wake_lock_init(&smb->wakelock_cable_t, WAKE_LOCK_SUSPEND, "cable_wakelock_timeout");
	/* check if power on with cable in */
	if ((smb1351_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING)||(!gpio_get_value(smb->inok_gpio))) {
		CHR_INFO("power on with cable in\n");
		if (!wake_lock_active(&smb->wakelock_cable)) {
			CHR_INFO("hold wake lock\n");
			wake_lock(&smb->wakelock_cable);
		}
	}

	smb1351_a_dev = smb;
	boot_mode = Read_Boot_Mode();
	CHR_INFO("%s ---\n", __func__);
	return 0;
}

static int smb1351_parallel_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct smb1351_charger *smb;

	CHR_INFO("%s +++\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);
	smb->client = client;

	/* initial mutex */
	mutex_init(&smb->lock);

	smb1351_b_dev = smb;
	parallel_charger_flag = 1;
	CHR_INFO("%s ---\n", __func__);

	return 0;
}

static int ads1013_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct adc_i2c_dev *adc;

	CHR_INFO("%s +++\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;
	adc = devm_kzalloc(dev, sizeof(*adc), GFP_KERNEL);
	if (!adc)
		return -ENOMEM;
	i2c_set_clientdata(client, adc);
	adc->client = client;
	ads1013_i2c_dev = adc;

	CHR_INFO("%s ---\n", __func__);
	return 0;
}

static int us5587_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct device *dev = &client->dev;
	struct adc_i2c_dev *adc;

	CHR_INFO("%s +++\n", __func__);

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;
	adc = devm_kzalloc(dev, sizeof(*adc), GFP_KERNEL);
	if (!adc)
		return -ENOMEM;
	i2c_set_clientdata(client, adc);
	adc->client = client;
	us5587_i2c_dev = adc;

	CHR_INFO("%s ---\n", __func__);
	return 0;
}

static void smb1351_shutdown(struct i2c_client *client)
{
	struct smb1351_charger *smb = i2c_get_clientdata(client);

	CHR_INFO("%s\n", __func__);
	power_supply_unregister(&smb->mains);
	power_supply_unregister(&smb->usb);
	i2c_set_clientdata(client, NULL);
	kfree(smb);
	if (parallel_charger_flag)
		kfree(smb1351_b_dev);
	kfree(ads1013_i2c_dev);
	kfree(us5587_i2c_dev);
}

static const struct i2c_device_id smb1351_id[] = {
	{ "smb1351_a", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb1351_id);

static const struct i2c_device_id smb1351_parallel_id[] = {
	{ "smb1351_b", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb1351_parallel_id);

static const struct i2c_device_id ads1013_id[] = {
	{ "ads1013", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, ads1013_id);

static const struct i2c_device_id us5587_id[] = {
	{ "us5587", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, us5587_id);


static struct i2c_driver smb1351_driver = {
	.driver = {
		.name	= "smb1351_a",
		.owner	= THIS_MODULE,
	},
	.id_table		= smb1351_id,
	.probe		= smb1351_probe,
	.shutdown	= smb1351_shutdown,
};

static struct i2c_driver smb1351_parallel_driver = {
	.driver = {
		.name	= "smb1351_b",
		.owner	= THIS_MODULE,
	},
	.probe	= smb1351_parallel_probe,
	.id_table	= smb1351_parallel_id,
};

static struct i2c_driver ads1013_driver = {
	.driver = {
		.name	= "ads1013",
		.owner	= THIS_MODULE,
	},
	.probe	= ads1013_probe,
	.id_table	= ads1013_id,
};

static struct i2c_driver us5587_driver = {
	.driver = {
		.name	= "us5587",
		.owner	= THIS_MODULE,
	},
	.probe	= us5587_probe,
	.id_table	= us5587_id,
};

static struct i2c_board_info smb1351_board_info[] = {
	{
		I2C_BOARD_INFO("smb1351_a", 0x6a),
	},
};

static struct i2c_board_info ads1013_board_info[] = {
	{
		I2C_BOARD_INFO("ads1013", 0x48),
	},
};

static struct i2c_board_info us5587_board_info[] = {
	{
		I2C_BOARD_INFO("us5587", 0x38),
	},
};

static int __init smb1351_init(void)
{
	int ret;

	CHR_INFO("%s\n", __func__);

	ret = i2c_add_driver(&smb1351_driver);
	if (ret) {
		CHR_ERR("%s: i2c_add_driver for main charger failed\n", __func__);
		return ret;
	}
	ret = i2c_register_board_info(8, smb1351_board_info, ARRAY_SIZE(smb1351_board_info));
	if (ret) {
		CHR_ERR("i2c_register_board_info for main charger failed\n");
		return ret;
	}
	ret = i2c_add_driver(&smb1351_parallel_driver);
	if (ret) {
		CHR_ERR("i2c_add_driver for parallel charger failed\n");
		i2c_del_driver(&smb1351_driver);
		return ret;
	}
	if (0) {
		CHR_INFO("use ads1013 to read ADC\n");
		ret = i2c_add_driver(&ads1013_driver);
		if (ret) {
			CHR_ERR("%s: i2c_add_driver for ads1013 failed\n", __func__);
			return ret;
		}
		ret = i2c_register_board_info(5, ads1013_board_info, ARRAY_SIZE(ads1013_board_info));
		if (ret) {
			CHR_ERR("i2c_register_board_info for ads1013 failed\n");
			return ret;
		}
	} else {
		CHR_INFO("use us5587 to read ADC\n");
		ret = i2c_add_driver(&us5587_driver);
		if (ret) {
			CHR_ERR("%s: i2c_add_driver for us5587 failed\n", __func__);
			return ret;
		}
		ret = i2c_register_board_info(5, us5587_board_info, ARRAY_SIZE(us5587_board_info));
		if (ret) {
			CHR_ERR("i2c_register_board_info for us5587 failed\n");
			return ret;
		}
	}
	return ret;
}
module_init(smb1351_init);

static void __exit smb1351_exit(void)
{
	CHR_INFO("%s\n", __func__);
	i2c_del_driver(&smb1351_driver);
	i2c_del_driver(&smb1351_parallel_driver);
}
module_exit(smb1351_exit);

MODULE_AUTHOR("Chih-Hsuan Chang <Chih-Hsuan_Chang@asus.com>");
MODULE_DESCRIPTION("SMB1351 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb1351");
