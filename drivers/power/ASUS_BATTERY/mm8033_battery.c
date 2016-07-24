/*
 * drivers/power/mm8033_battery.c
 *
 * Gas Gauge driver for MITSUMI's MM8033A01
 *
 * Copyright (c) 2014, ASUSTek Inc.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/i2c.h>
#include <linux/slab.h>
#include <linux/switch.h>
#include <linux/HWVersion.h>
#include <linux/earlysuspend.h>
#include <asm/unaligned.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_mid_thermal.h>
#include "asus_battery.h"
#include "smb_external_include.h"
#include <linux/proc_fs.h>
#include <linux/seq_file.h>

#define DRIVER_VERSION					"1.5.7"
#define TRY								5
#define SKIP								10
#define MM8033_ID							0x0110
#define SYSTEM_LOWVOLTAGE_LIMIT		3400


#define REG_STATUS						0x00
#define REG_V_ALERT_THRESHOLD			0x01
#define REG_T_ALERT_THRESHOLD			0x02
#define REG_S_ALERT_THRESHOLD			0x03
#define REG_ATRATE						0x04
#define REG_REM_CAPACITY				0x05
#define REG_SOC							0x06
#define REG_SOH							0x07
#define REG_TEMPERATURE				0x08
#define REG_VOLTAGE						0x09
#define REG_CURRENT						0x0A
#define REG_AVERAGE_CURRENT			0x0B
#define REG_IC_TEMPERATURE				0x0C
#define REG_ABSOLUTE_SOC				0x0D
#define REG_SOC_NF						0x0E
#define REG_ABSOLUTE_REM_CAPACITY           0x0F
#define REG_FULL_CHARGE_CAPACITY		0x10
#define REG_TTE							0x11
#define REG_SOI							0x12
#define REG_FULL_VOLTAGE_THR			0x13
#define REG_CYCLE_COUNT				0x17
#define REG_DESIGN_CAPACITY				0x18
#define REG_AVERAGE_VOLTAGE			0x19
#define REG_PEAK_TEMPERATURE			0x1A
#define REG_PEAK_VOLTAGE				0x1B
#define REG_PEAK_CURRENT				0x1C
#define REG_CONFIG						0x1D
#define REG_CHARGE_TERMINAL_CURRENT	0x1E
#define REG_REM_CAPACITY_NF			0x1F
#define REG_FG_CONDITION				0x20
#define REG_IDENTIFY						0x21
#define REG_BATTERY_IMPEDANCE			0x22
#define REG_BATTERY_CAPACITY			0x23
#define REG_PROJECT_NAME				0x2C
#define REG_PACKCELL_VENDOR			0x2D
#define REG_DATE						0x2E

#define REG_EMPTY_VOLTAGE				0x3A
#define REG_FG_STAT 						0x3D

#define GAUGE_ERR(...)        printk("[MM8033_ERR] " __VA_ARGS__);
#define GAUGE_INFO(...)       printk("[MM8033] " __VA_ARGS__);
#define SOC_SMOOTH_ALGO

struct mm8033_batparams {
	u8 params[512];
	u8 en_i2c_sh;
	int fullvoltage_thr;
	int designcapacity;
	int charge_terminal_current;
	int paramrevision;
	int batterycode;
	int default_cyclecount;
	int default_batterycapacity;
	int default_batteryimpedance;
};
struct mm8033_chip {
#ifdef REGISTER_POWER_SUPPLY
	struct power_supply battery;
#endif
	struct i2c_client *client;
	int cyclecount;
	int batterycapacity;
	int batteryimpedance;
	int skipcheck;
	struct mm8033_batparams batparams;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend es;
#endif
};

/* global variables */
extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
static struct switch_dev mm8033_batt_dev;
static struct dev_func mm8033_tbl;
static struct mm8033_chip *g_mm8033_chip;
static char *batt_id;
static int bat_year, bat_week_1, bat_week_2, invalid_bat=0, i2c_rw_lock=0;
static int g_curr=0x10000, g_soc=50, g_volt=3000, g_temp=25, g_fcc=3000, g_rm=1500;
static u16 parameter_version, battery_ctrlcode;
struct delayed_work 	batt_state_wq;
#ifdef SOC_SMOOTH_ALGO
static int mm8033_pre_soc=0;
#endif

static int mm8033_write_reg(struct i2c_client *client, u8 reg, u16 value)
{
	int ret = i2c_smbus_write_word_data(client, reg, value);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	msleep(4);
	return ret;
}

static int mm8033_read_reg(struct i2c_client *client, u8 reg)
{
	int ret = i2c_smbus_read_word_data(client, reg);

	if (ret < 0)
		dev_err(&client->dev, "%s: err %d\n", __func__, ret);

	msleep(4);
	return ret;
}

static int mm8033_writex_reg(struct i2c_client *client, u8 reg, u8 *buf, u16 len)
{
	struct i2c_msg msg[1];
	int i, ret;
	u8 *wbuf;

	wbuf = kzalloc(sizeof(u8) * (len + 1), GFP_KERNEL);
	wbuf[0] = reg;
	for (i = 0; i < len; i++) {
		wbuf[i + 1] = buf[i];
	}

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = (u16)(len + 1);
	msg[0].buf = wbuf;

	ret = i2c_transfer(client->adapter, msg, 1);
	kfree(wbuf);

	if (ret < 0) {
		return ret;
	}

	return 0;
}

static int mm8033_readx_reg(struct i2c_client *client, u8 reg, u8 *buf, u16 len)
{
	struct i2c_msg msg[2];
	int ret;

	msg[0].addr = client->addr;
	msg[0].flags = 0;
	msg[0].len = 1;
	msg[0].buf = &reg;
	msg[1].addr = client->addr;
	msg[1].flags = I2C_M_RD;
	msg[1].buf = buf;
	msg[1].len = len;

	ret = i2c_transfer(client->adapter, msg, 2);
	if (ret < 0) return ret;

	return 0;
}
#ifdef REGISTER_POWER_SUPPLY
static int mm8033_soc(struct mm8033_chip *chip, int *val)
{
	int soc = mm8033_read_reg(chip->client, REG_SOC);

	if (soc < 0) return soc;
	*val = soc / 256;

	return 0;
}

static int mm8033_current(struct mm8033_chip *chip, int *val)
{
	int curr = mm8033_read_reg(chip->client, REG_CURRENT);

	if (curr < 0) return curr;

	if (curr > 32767) {
		curr -= 65536;
	}
	*val = curr;

	return 0;
}

static int mm8033_temperature(struct mm8033_chip *chip, int *val)
{
	int temp = mm8033_read_reg(chip->client, REG_IC_TEMPERATURE);

	if (temp < 0) return temp;

	if (temp > 32767) {
		temp -= 65536;
	}
	*val = temp;

	return 0;
}
#endif
/*
static int mm8033_status(struct mm8033_chip *chip, int *val)
{
	int stat = mm8033_read_reg(chip->client, REG_STATUS);

	if (stat < 0) return stat;
	*val = stat;

	return 0;
}
*/
static int mm8033_voltage(struct mm8033_chip *chip, int *val)
{
	int volt = mm8033_read_reg(chip->client, REG_VOLTAGE);

	if (volt < 0) return volt;
	*val = volt;

	return 0;
}

static int mm8033_identify(struct mm8033_chip *chip, int *val)
{
	int id = mm8033_read_reg(chip->client, REG_IDENTIFY);

	if (id < 0) return id;
	*val = id;

	return 0;
}

static int mm8033_projectname(struct mm8033_chip *chip, int *val)
{
	int pn = mm8033_read_reg(chip->client, REG_PROJECT_NAME);

	if (pn < 0) return pn;
	*val = pn;

	return 0;
}

static int mm8033_packcellvendor(struct mm8033_chip *chip, int *val)
{
	int pcv = mm8033_read_reg(chip->client, REG_PACKCELL_VENDOR);

	if (pcv < 0) return pcv;
	*val = pcv;

	return 0;
}
static int mm8033_bat_date(struct mm8033_chip *chip, int *val)
{
	int date = mm8033_read_reg(chip->client, REG_DATE);

	if (date < 0) return date;
	*val = date;

	return 0;
}

static int mm8033_fgstat(struct mm8033_chip *chip, int *val)
{
	int stat = mm8033_read_reg(chip->client, REG_FG_STAT);

	if (stat < 0) return stat;
	*val = stat;

	return 0;
}

static int mm8033_designcapacity(struct mm8033_chip *chip, int *val)
{
	int dc = mm8033_read_reg(chip->client, REG_DESIGN_CAPACITY);

	if (dc < 0) return dc;
	*val = dc;

	return 0;
}

static int mm8033_cyclecount(struct mm8033_chip *chip, int *val)
{
	int cc = mm8033_read_reg(chip->client, REG_CYCLE_COUNT);

	if (cc < 0) return cc;
	*val = cc;

	return 0;
}

static int mm8033_batteryimpedance(struct mm8033_chip *chip, int *val)
{
	int bi = mm8033_read_reg(chip->client, REG_BATTERY_IMPEDANCE);

	if (bi < 0) return bi;
	*val = bi;

	return 0;
}

static int mm8033_batterycapacity(struct mm8033_chip *chip, int *val)
{
	int bc = mm8033_read_reg(chip->client, REG_BATTERY_CAPACITY);

	if (bc < 0) return bc;
	*val = bc;

	return 0;
}

static int mm8033_setFgParameter(struct mm8033_chip *chip)
{
	int i, j;
	int ret;
	int val;
	u8 buf[8];
	u8 buf_89[8] = {0x5F, 0xF3, 0xFF, 0xFF, 0xFF, 0x18, 0xFF, 0xFF};
	u8 buf_8a[8] = {0xFF, 0xFF, 0xDA, 0x1D, 0x0B, 0x00, 0xFF, 0xFF};
#if 1
	GAUGE_INFO("%s +++\n", __func__);
	for (i = 0; i < 0x40; i++) {
		if ((i != 0x9) && (i != 0xa)) {
			for (j = 0; j < 8; j++) {
				buf[j] = chip->batparams.params[i*8 + j];
			}

			ret = mm8033_writex_reg(chip->client, (u8)(0x80 + i), buf, (u16)8);
			if (ret) {
				dev_err(&chip->client->dev, "failed to send parameter.\n");
				return ret;
			}
			msleep(5);
		}else if((Read_HW_ID()==HW_ID_ER)||(Read_HW_ID()==HW_ID_ER1_1)||(Read_HW_ID()==HW_ID_ER1_2)) {
			// write 0x89, 0x8a
			GAUGE_INFO("0x89=0x%04x\n", mm8033_read_reg(chip->client, 0x89));
			if(mm8033_read_reg(chip->client, 0x89)!=0xF35F) {
				GAUGE_INFO("write 0x89 for correcting wrong value writen to these register\n");
				ret = mm8033_writex_reg(chip->client, (u8)0x89, buf_89, (u16)8);
				if (ret) {
					dev_err(&chip->client->dev, "failed to send parameter(0x89).\n");
					return ret;
				}
				msleep(5);
				GAUGE_INFO("write 0x8a for correcting wrong value writen to these register\n");
				ret = mm8033_writex_reg(chip->client, (u8)0x8a, buf_8a, (u16)8);
				if (ret) {
					dev_err(&chip->client->dev, "failed to send parameter(0x8a).\n");
					return ret;
				}
				msleep(5);
			}else {
				GAUGE_INFO("no need to re-write 0x89/0x8a\n");
			}
		}
	}
#ifdef I2CLOW_RESTART
	if (chip->batparams.en_i2c_sh) {
		ret = mm8033_write_reg(chip->client, REG_CONFIG, 0x40);
		if (ret < 0) return ret;
		msleep(5);
	}
#endif
	ret = mm8033_write_reg(chip->client, REG_FULL_VOLTAGE_THR, chip->batparams.fullvoltage_thr);
	if (ret < 0) return ret;
	msleep(5);
	ret = mm8033_write_reg(chip->client, REG_DESIGN_CAPACITY, chip->batparams.designcapacity);
	if (ret < 0) return ret;
	msleep(5);
	ret = mm8033_write_reg(chip->client, REG_CHARGE_TERMINAL_CURRENT, chip->batparams.charge_terminal_current);
	if (ret < 0) return ret;
	msleep(5);
	ret = mm8033_write_reg(chip->client, REG_BATTERY_IMPEDANCE, chip->batteryimpedance);
	if (ret < 0) return ret;
	msleep(5);
	ret = mm8033_write_reg(chip->client, REG_BATTERY_CAPACITY, chip->batterycapacity);
	if (ret < 0) return ret;
	msleep(5);
	ret = mm8033_write_reg(chip->client, REG_CYCLE_COUNT, chip->cyclecount);
	if (ret < 0) return ret;
	if (smb1357_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING) {
		GAUGE_INFO("make sure not charging when reset IC and get soc.\n");
		smb1357_charging_toggle(false);
		msleep(3000);
		i2c_rw_lock = 1;
		ret = mm8033_write_reg(chip->client, REG_FG_CONDITION, 0x24);
		if (ret < 0) {
			i2c_rw_lock = 0;
			return ret;
		}
		msleep(100);
		smb1357_charging_toggle(true);
	}else {
		GAUGE_INFO("charging status=%d\n", smb1357_get_charging_status());
		msleep(5);
		i2c_rw_lock = 1;
		ret = mm8033_write_reg(chip->client, REG_FG_CONDITION, 0x24);
		if (ret < 0) {
			i2c_rw_lock = 0;
			return ret;
		}
		msleep(100);
	}
	msleep(50);
	i2c_rw_lock = 0;
	do {
		ret = mm8033_fgstat(chip, &val);
		if (ret) return ret;
	} while (val & 0x0001);
	GAUGE_INFO("%s ---\n", __func__);
#else
	GAUGE_INFO("%s: do not set parameter\n", __func__);
#endif
	return 0;
}

static int mm8033_checkRamData(struct mm8033_chip *chip)
{
	//int i, ret, val;
	int ret, val;
	u8 buf[8];

	GAUGE_INFO("%s +++\n", __func__);

	ret = mm8033_readx_reg(chip->client, 0xBE, buf, (u16)8);
	if (ret) return ret;
	if (((((buf[7] & 0xff) << 8) | (buf[6] & 0xff)) < chip->batparams.paramrevision)
	|| ((((buf[5] & 0xff) << 8) | (buf[4] & 0xff)) != chip->batparams.batterycode)) {
		if((Read_HW_ID()==HW_ID_ER)||(Read_HW_ID()==HW_ID_ER1_1)||(Read_HW_ID()==HW_ID_ER1_2)) {
			if ( (((buf[7] & 0xff) << 8) | (buf[6] & 0xff)) < chip->batparams.paramrevision ) {
				GAUGE_INFO("paramrevision = 0x%02x%02x, set parameter\n", buf[7], buf[6]);
				goto SETPARAMETER;
			}
		}else {
			GAUGE_INFO("paramrevision = 0x%02x%02x, set parameter\n", buf[7], buf[6]);
			goto SETPARAMETER;
		}
	} else {
		ret = mm8033_designcapacity(chip, &val);
		if (ret) return ret;

		if (val != chip->batparams.designcapacity) {
			if((Read_HW_ID()==HW_ID_ER)||(Read_HW_ID()==HW_ID_ER1_1)||(Read_HW_ID()==HW_ID_ER1_2)) {
				if((val<2800)||(val>3000)) {
					GAUGE_INFO("capacity = %d, set parameter\n", val);
					goto SETPARAMETER;
				}
			}else {
				GAUGE_INFO("capacity = %d, set parameter\n", val);
				goto SETPARAMETER;
			}
		} else {
			goto GETPARAMETER;
		}
	}

GETPARAMETER:
	GAUGE_INFO("GETPARAMETER\n");
	ret = mm8033_cyclecount(chip, &val);
	if (ret) return ret;
	chip->cyclecount = val;

	ret = mm8033_batteryimpedance(chip, &val);
	if (ret) return ret;
	chip->batteryimpedance = val;

	ret = mm8033_batterycapacity(chip, &val);
	if (ret) return ret;
	chip->batterycapacity = val;

	goto EXIT;

SETPARAMETER:
#if 0
	GAUGE_INFO("SETPARAMETER\n");
	for (i = 0; i < TRY; i++) {
		ret = mm8033_setFgParameter(chip);
		if (ret) return ret;

		ret = mm8033_readx_reg(chip->client, 0xBE, buf, (u16)8);
		if (ret) return ret;
		/* need to check after */
		if ((((buf[7] & 0xff) << 8) | (buf[6] & 0xff)) != chip->batparams.paramrevision) continue;
		if ((((buf[5] & 0xff) << 8) | (buf[4] & 0xff)) != chip->batparams.batterycode) continue;

		ret = mm8033_cyclecount(chip, &val);
		if (ret) return ret;
		if (val != chip->cyclecount) continue;

		ret = mm8033_designcapacity(chip, &val);
		if (ret) return ret;
		if (val != chip->batparams.designcapacity) continue;

		break;
	}
	if (i >= TRY) return -1;
#else
	GAUGE_INFO("don't SETPARAMETER here\n");
#endif
EXIT:
	GAUGE_INFO("%s ---\n", __func__);
	return 0;
}

static int mm8033_checkdevice(struct mm8033_chip *chip)
{
	int i;
	int val;
	int ret;
	u8 projectname_low;
	u8 projectname_high;
	u8 packvendor;
	u8 cellvendor;
	u8 date_low;
	u8 date_high;
	u8 buf[8];

	struct mm8033_batparams *batparams;
	// NVT ATL battery parameter
	struct mm8033_batparams params_nvt = {
	{
		0xAB, 0x17, 0x94, 0x1E, 0xED, 0x2A, 0xE4, 0x3F,
		0xA8, 0x5D, 0xE3, 0x5D, 0x22, 0x70, 0x6A, 0x78,
		0x5B, 0x11, 0x42, 0xC2, 0x0D, 0x45, 0xF6, 0x0A,
		0xDA, 0xE4, 0x48, 0x16, 0x48, 0x07, 0x40, 0xF4,
		0x2B, 0x06, 0x46, 0x05, 0x3D, 0xFA, 0xB4, 0x01,
		0x86, 0x03, 0xBD, 0xFD, 0xF3, 0xFF, 0x3F, 0x08,
		0x49, 0xF7, 0x27, 0x02, 0x60, 0xFD, 0x1C, 0x06,
		0x1A, 0xFD, 0xFE, 0xDE, 0xCA, 0x28, 0x34, 0xF3,
		0x00, 0x0E, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x14, 0x2F, 0x01, 0x66, 0x30, 0x00, 0x72, 0x06,
		0x54, 0x0B, 0x48, 0x0D, 0x30, 0x11, 0x32, 0x41,
		0x1E, 0x05, 0x3C, 0x2D, 0x78, 0x01, 0x00, 0x01,
		0xB1, 0x0C, 0xED, 0x0C, 0x25, 0x0D, 0x97, 0x0D,
		0x1E, 0x0E, 0x41, 0x0E, 0x60, 0x0E, 0x6F, 0x0E,
		0x7A, 0x0E, 0x93, 0x0E, 0xB1, 0x0E, 0xC9, 0x0E,
		0xE3, 0x0E, 0x0A, 0x0F, 0x34, 0x0F, 0x71, 0x0F,
		0xAD, 0x0F, 0xFA, 0x0F, 0x72, 0x10, 0x1A, 0x11,
		0x00, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x14, 0x00,
		0x26, 0x00, 0x32, 0x00, 0x4B, 0x00, 0x67, 0x00,
		0x84, 0x00, 0xB4, 0x00, 0xFA, 0x00, 0x36, 0x01,
		0x90, 0x01, 0xEA, 0x01, 0x2B, 0x02, 0x62, 0x02,
		0xA8, 0x02, 0xF8, 0x02, 0x66, 0x03, 0xFC, 0x03,
		0x00, 0xE1, 0x00, 0x4D, 0xE1, 0x00, 0xA6, 0xF0,
		0x00, 0xFF, 0xD9, 0x00, 0x0D, 0xF6, 0x00, 0x39,
		0xCB, 0x00, 0x5A, 0xF6, 0x00, 0x80, 0x3C, 0x00,
		0x14, 0x0A, 0x05, 0x0A, 0x5C, 0xFF, 0xE4, 0x00,
		0x9A, 0x0B, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0x28, 0x00, 0x28, 0x00,
		0x2D, 0x00, 0x46, 0x00, 0x00, 0x07, 0x08, 0xFF,
		0xC8, 0x00, 0xC8, 0x00, 0x6E, 0x00, 0x82, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0xFE, 0x01, 0xFE, 0x01,
		0xC8, 0x00, 0x18, 0x01, 0x00, 0x07, 0x08, 0x78,
		0xFF, 0xFF, 0x9E, 0x02, 0x9E, 0x02, 0x2C, 0x01,
		0x2C, 0x01, 0xA4, 0x01, 0xFF, 0xFF, 0x00, 0x07,
		0x08, 0xE6, 0xFF, 0xFF, 0x1A, 0x04, 0x1A, 0x04,
		0xF0, 0x00, 0x94, 0x02, 0x94, 0x02, 0xFF, 0xFF,
		0x56, 0x0E, 0x0A, 0x2C, 0xD8, 0x0E, 0x0A, 0x20,
		0xE6, 0x0C, 0x0C, 0xFE, 0x3A, 0xFC, 0x68, 0x03,
		0x82, 0x00, 0xA8, 0x94, 0x0F, 0x66, 0xFA, 0x1E,
		0xF4, 0x01, 0xE4, 0x00, 0x02, 0x02, 0x32, 0x0A,
		0xA4, 0x01, 0x1A, 0x01, 0xA0, 0x00, 0xFF, 0xFF,
		0xAA, 0x02, 0x35, 0x01, 0x20, 0x03, 0x60, 0x62,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x80, 0x9C, 0xCE, 0xFF, 0x01, 0x32, 0x64, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x80, 0xFA, 0xFD, 0xFF, 0x01, 0x03, 0x06, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x00, 0x00, 0x14, 0x14, 0x14, 0x05, 0x00, 0x00,
		0x06, 0x00, 0x19, 0x10, 0x17, 0x17, 0x3E, 0x17,
		0x17, 0x02, 0x1C, 0x03, 0x0D, 0x32, 0x0A, 0xFF,
		0x0A, 0x12, 0x66, 0xFF, 0x02, 0x05, 0x05, 0x01,
		0x70, 0xC2, 0x0A, 0x00, 0x00, 0x00, 0x03, 0x99,
		0x99, 0x96, 0xFF, 0xCC, 0x4B, 0x0F, 0x71, 0xC1,
		0x62, 0x05, 0x14, 0x0A, 0x2C, 0x01, 0x5F, 0x05,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0x7A, 0x0D, 0x48, 0x0D, 0x03, 0x00, 0x0A, 0x0A,
		0x64, 0x00, 0x54, 0x0B, 0x64, 0x02, 0x02, 0x02,
		0x7D, 0x73, 0xFA, 0xE6, 0xBC, 0xB7, 0x6C, 0x71,
		0x4B, 0x78, 0x82, 0x09, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x30, 0x74, 0x0F, 0xFF, 0x03, 0x00, 0x06, 0x01,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	},
#ifdef I2CLOW_RESTART
	1,			// enable i2c_sh
#else
	0,			// enable i2c_sh
#endif
	4350,   // full voltage threshold
	2970,   // design capacity
	130,    // charge terminal current
	0x0106, // parameter revision
	0x0003, // battery code
	0,      // default cycle count
	2970,   // default battery capacity
	130     // default battery impedance
	};

	// Simplo Coslight battery parameter
	struct mm8033_batparams params_simplo = {
	{
		0xEB, 0x17, 0xC0, 0x1F, 0x35, 0x2C, 0xBE, 0x3F,
		0xB0, 0x42, 0x36, 0x60, 0xB6, 0x71, 0x41, 0x79,
		0x73, 0x11, 0xB7, 0xC1, 0xC8, 0x45, 0x53, 0x0A,
		0xD8, 0xE7, 0xC5, 0x12, 0x14, 0x07, 0xF0, 0xF4,
		0x93, 0x05, 0x2D, 0x05, 0x72, 0xFA, 0x96, 0x01,
		0xB4, 0x04, 0x65, 0xFB, 0x1C, 0x01, 0x56, 0x03,
		0x05, 0xFE, 0xDA, 0xFF, 0x48, 0xFD, 0x12, 0x06,
		0x2C, 0xFD, 0xA4, 0xD1, 0x33, 0x37, 0x59, 0xEF,
		0x00, 0x0E, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x14, 0x2F, 0x01, 0x66, 0x30, 0x00, 0x72, 0x06,
		0x54, 0x0B, 0x48, 0x0D, 0x30, 0x11, 0x32, 0x41,
		0x1E, 0x05, 0x3C, 0x2D, 0x78, 0x01, 0x00, 0x01,
		0xB8, 0x0C, 0xFC, 0x0C, 0x45, 0x0D, 0xAF, 0x0D,
		0x0F, 0x0E, 0x3C, 0x0E, 0x63, 0x0E, 0x70, 0x0E,
		0x7A, 0x0E, 0x91, 0x0E, 0xAF, 0x0E, 0xC7, 0x0E,
		0xE3, 0x0E, 0x08, 0x0F, 0x31, 0x0F, 0x6C, 0x0F,
		0xA9, 0x0F, 0xF9, 0x0F, 0x6E, 0x10, 0x15, 0x11,
		0x00, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x14, 0x00,
		0x26, 0x00, 0x37, 0x00, 0x4D, 0x00, 0x67, 0x00,
		0x84, 0x00, 0xB4, 0x00, 0xFA, 0x00, 0x36, 0x01,
		0x90, 0x01, 0xEA, 0x01, 0x2B, 0x02, 0x62, 0x02,
		0xA8, 0x02, 0xF8, 0x02, 0x66, 0x03, 0xFC, 0x03,
		0x00, 0xE1, 0x00, 0x4D, 0xE1, 0x00, 0xA6, 0xF0,
		0x00, 0xFF, 0xD4, 0x00, 0x0D, 0xF6, 0x00, 0x39,
		0xCB, 0x00, 0x5A, 0xF6, 0x00, 0x80, 0x3C, 0x00,
		0x14, 0x0A, 0x05, 0x0A, 0x5C, 0xFF, 0xE4, 0x00,
		0x54, 0x0B, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0x96, 0x00, 0x96, 0x00,
		0x58, 0x00, 0x5A, 0x00, 0x00, 0x07, 0x08, 0xFF,
		0xDC, 0x00, 0xDC, 0x00, 0x69, 0x00, 0x8C, 0x00,
		0x00, 0x07, 0x08, 0xFF, 0x1C, 0x02, 0x1C, 0x02,
		0x09, 0x01, 0x5E, 0x01, 0x00, 0x07, 0x08, 0x6E,
		0xFF, 0xFF, 0x80, 0x02, 0x80, 0x02, 0x7C, 0x01,
		0x68, 0x01, 0xF4, 0x01, 0xFF, 0xFF, 0x00, 0x07,
		0x08, 0x78, 0xFF, 0xFF, 0x84, 0x03, 0x84, 0x03,
		0x58, 0x02, 0xE0, 0x01, 0x8A, 0x02, 0xFF, 0xFF,
		0x42, 0x0E, 0x0A, 0x1E, 0xEC, 0x0E, 0x0A, 0x32,
		0xE6, 0x0C, 0xD4, 0xFE, 0x57, 0xFC, 0x92, 0x03,
		0x8C, 0x00, 0xA8, 0x94, 0x0F, 0x66, 0xFA, 0x1E,
		0xF4, 0x01, 0xD4, 0x00, 0x02, 0x02, 0x32, 0x0A,
		0x6D, 0x01, 0xFB, 0x00, 0x96, 0x00, 0xFF, 0xFF,
		0xAA, 0x02, 0x35, 0x01, 0x20, 0x03, 0x60, 0x62,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x80, 0x9C, 0xCE, 0xFF, 0x01, 0x32, 0x64, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x80, 0xFA, 0xFD, 0xFF, 0x01, 0x03, 0x06, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x00, 0x00, 0x14, 0x14, 0x14, 0x05, 0x00, 0x00,
		0x06, 0x00, 0x19, 0x10, 0x17, 0x17, 0x3E, 0x17,
		0x17, 0x02, 0x1C, 0x03, 0x0D, 0x32, 0x0A, 0xFF,
		0x0A, 0x12, 0x66, 0xFF, 0x02, 0x05, 0x05, 0x01,
		0x70, 0xC2, 0x0A, 0x00, 0x00, 0x00, 0x03, 0x99,
		0x99, 0x96, 0xFF, 0xCC, 0x4B, 0x0F, 0x71, 0xC1,
		0x62, 0x05, 0x14, 0x0A, 0x2C, 0x01, 0x5F, 0x05,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0x98, 0x0D, 0x48, 0x0D, 0x03, 0x00, 0x0A, 0x0A,
		0x64, 0x00, 0x54, 0x0B, 0x64, 0x02, 0x02, 0x02,
		0x7D, 0x73, 0xFA, 0xE6, 0xBC, 0xB7, 0x6C, 0x71,
		0x4B, 0x78, 0x82, 0x09, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x40, 0x7A, 0x0F, 0xFF, 0x04, 0x00, 0x06, 0x01,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	},
#ifdef I2CLOW_RESTART
	1,			// enable i2c_sh
#else
	0,			// enable i2c_sh
#endif
	4350,   // full voltage threshold
	2900,   // design capacity
	130,    // charge terminal current
	0x0106, // parameter revision
	0x0004, // battery code
	0,      // default cycle count
	2900,   // default battery capacity
	140     // default battery impedance
	};

	// Celxpert Coslight battery parameter
	struct mm8033_batparams params_celxpert = {
	{
		0xEB, 0x17, 0xC0, 0x1F, 0x35, 0x2C, 0xBE, 0x3F,
		0xB0, 0x42, 0x36, 0x60, 0xB6, 0x71, 0x41, 0x79,
		0x73, 0x11, 0xB7, 0xC1, 0xC8, 0x45, 0x53, 0x0A,
		0xD8, 0xE7, 0xC5, 0x12, 0x14, 0x07, 0xF0, 0xF4,
		0x93, 0x05, 0x2D, 0x05, 0x72, 0xFA, 0x96, 0x01,
		0xB4, 0x04, 0x65, 0xFB, 0x1C, 0x01, 0x56, 0x03,
		0x05, 0xFE, 0xDA, 0xFF, 0x48, 0xFD, 0x12, 0x06,
		0x2C, 0xFD, 0xA4, 0xD1, 0x33, 0x37, 0x59, 0xEF,
		0x00, 0x0E, 0x0D, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x14, 0x2F, 0x01, 0x66, 0x30, 0x00, 0x72, 0x06,
		0x54, 0x0B, 0x48, 0x0D, 0x30, 0x11, 0x32, 0x41,
		0x1E, 0x05, 0x3C, 0x2D, 0x78, 0x01, 0x00, 0x01,
		0x9E, 0x0C, 0xF2, 0x0C, 0x39, 0x0D, 0x8E, 0x0D,
		0xF3, 0x0D, 0x2F, 0x0E, 0x5D, 0x0E, 0x69, 0x0E,
		0x74, 0x0E, 0x8D, 0x0E, 0xAB, 0x0E, 0xC4, 0x0E,
		0xE3, 0x0E, 0x09, 0x0F, 0x31, 0x0F, 0x62, 0x0F,
		0xA7, 0x0F, 0xF6, 0x0F, 0x6B, 0x10, 0x13, 0x11,
		0x00, 0x00, 0x05, 0x00, 0x0A, 0x00, 0x14, 0x00,
		0x26, 0x00, 0x32, 0x00, 0x41, 0x00, 0x67, 0x00,
		0x84, 0x00, 0xB4, 0x00, 0xFA, 0x00, 0x36, 0x01,
		0x90, 0x01, 0xEA, 0x01, 0x2B, 0x02, 0x62, 0x02,
		0xA8, 0x02, 0xF8, 0x02, 0x66, 0x03, 0xFC, 0x03,
		0x00, 0xE1, 0x00, 0x4D, 0xE1, 0x00, 0xA6, 0xF0,
		0x00, 0xFF, 0xDD, 0x00, 0x0D, 0xF6, 0x00, 0x39,
		0xCB, 0x00, 0x5A, 0xF6, 0x00, 0x80, 0x3C, 0x00,
		0x14, 0x0A, 0x05, 0x0A, 0x5C, 0xFF, 0xE4, 0x00,
		0x54, 0x0B, 0x1E, 0x1E, 0x00, 0x00, 0x00, 0x00,
		0x00, 0x08, 0x09, 0xFF, 0x6E, 0x00, 0x6E, 0x00,
		0x50, 0x00, 0x5A, 0x00, 0x00, 0x08, 0x09, 0xFF,
		0xFA, 0x00, 0xFA, 0x00, 0xA0, 0x00, 0xAA, 0x00,
		0x00, 0x08, 0x09, 0xFF, 0x3A, 0x02, 0x3A, 0x02,
		0x2C, 0x01, 0x90, 0x01, 0x00, 0x08, 0x09, 0x8C,
		0xFF, 0xFF, 0x94, 0x02, 0x94, 0x02, 0x90, 0x01,
		0x90, 0x01, 0xE0, 0x01, 0xFF, 0xFF, 0x00, 0x08,
		0x09, 0xB4, 0xFF, 0xFF, 0x84, 0x03, 0x84, 0x03,
		0x58, 0x02, 0x3A, 0x02, 0xA8, 0x02, 0xFF, 0xFF,
		0x56, 0x0E, 0x0A, 0x1E, 0xEC, 0x0E, 0x0A, 0x10,
		0xE6, 0x0C, 0xD4, 0xFE, 0x94, 0xFB, 0x98, 0x03,
		0xAA, 0x00, 0xA8, 0x94, 0x0F, 0x66, 0xFA, 0x1E,
		0xF4, 0x01, 0xAE, 0x00, 0x02, 0x02, 0x32, 0x0A,
		0x72, 0x01, 0xFC, 0x00, 0x96, 0x00, 0xFF, 0xFF,
		0xAA, 0x02, 0x35, 0x01, 0x20, 0x03, 0x60, 0x62,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x80, 0x9C, 0xCE, 0xFF, 0x01, 0x32, 0x64, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x80, 0xFA, 0xFD, 0xFF, 0x01, 0x03, 0x06, 0x7F,
		0x04, 0x05, 0x06, 0x08, 0x0F, 0x14, 0x19, 0x19,
		0x00, 0x00, 0x14, 0x14, 0x14, 0x05, 0x00, 0x00,
		0x06, 0x00, 0x19, 0x10, 0x17, 0x17, 0x3E, 0x17,
		0x17, 0x02, 0x1C, 0x03, 0x0D, 0x32, 0x0A, 0xFF,
		0x0A, 0x12, 0x66, 0xFF, 0x02, 0x05, 0x05, 0x01,
		0x70, 0xC2, 0x0A, 0x00, 0x00, 0x00, 0x03, 0x99,
		0x99, 0x96, 0xFF, 0xCC, 0x4B, 0x0F, 0x71, 0xC1,
		0x62, 0x05, 0x14, 0x0A, 0x2C, 0x01, 0x5F, 0x05,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x00, 0x00, 0x00,
		0xAC, 0x0D, 0x48, 0x0D, 0x03, 0x00, 0x0A, 0x0A,
		0x64, 0x00, 0x54, 0x0B, 0x64, 0x02, 0x02, 0x02,
		0x7D, 0x73, 0xFA, 0xE6, 0xBC, 0xB7, 0x6C, 0x71,
		0x4B, 0x78, 0x82, 0x09, 0xFF, 0xFF, 0xFF, 0xFF,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
		0x30, 0x64, 0x0F, 0xFF, 0x05, 0x00, 0x06, 0x01,
		0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
	},
#ifdef I2CLOW_RESTART
	1,			// enable i2c_sh
#else
	0,			// enable i2c_sh
#endif
	4350,   // full voltage threshold
	2900,   // design capacity
	130,    // charge terminal current
	0x0106, // parameter revision
	0x0005, // battery code
	0,      // default cycle count
	2900,   // default battery capacity
	170     // default battery impedance
	};

	if((Read_HW_ID()!=HW_ID_SR1)&&(Read_HW_ID()!=HW_ID_SR2)) { //SR's battery ID in OTP is old so donnot check
		ret = mm8033_identify(chip, &val);
		if (ret) return ret;
		if (val != MM8033_ID) {
			GAUGE_INFO("ID = 0x%04x\n", val);
			return -1;
		}
	}

	do {
		ret = mm8033_fgstat(chip, &val);
		if (ret) return ret;
	} while (val & 0x0001);

	ret = mm8033_voltage(chip, &val);
	if (ret) return ret;
	if (val < SYSTEM_LOWVOLTAGE_LIMIT) return -1;

	// get project ids
	ret = mm8033_projectname(chip, &val);
	if (ret) return ret;
	projectname_low = (u8)((val & 0x00ff) >> 0);
	projectname_high = (u8)((val & 0xff00) >> 8);
	//get vendor id
	ret = mm8033_packcellvendor(chip, &val);
	if (ret) return ret;
	packvendor = (u8)((val & 0x00ff) >> 0);
	cellvendor = (u8)((val & 0xff00) >> 8);
	//get ic date
	ret = mm8033_bat_date(chip, &val);
	if (ret) return ret;
	date_low = (u8)((val & 0x00ff) >> 0);
	date_high = (u8)((val & 0xff00) >> 8);
	//show bat parameter date[15:9]=year, date[8:5]=week_1, date[4:0]=week_2
	bat_year = (int)(date_high >> 1);
	bat_week_1 = (int)( ((date_high & 0x01) << 3) +((date_low & 0xe0) >> 5) );
	bat_week_2 = (int)(date_low & 0x1f);

	// select params
	if ((projectname_low != (u8)'Z') || (projectname_high != (u8)'2')) {
		if(((projectname_low == (u8)'2') && (projectname_high == (u8)'Z'))&&((packvendor == (u8)'3') && (cellvendor == (u8)'C'))) {
			batparams = &params_celxpert;
			GAUGE_INFO("BATTERY is CELXPERT(inverse ID info)\n");
			batt_id = "Z2C3";
		} else {
			batparams = &params_nvt;
			GAUGE_INFO("PROJECT is 0x%02x%02x, not Z2, use NVT as default\n", projectname_high, projectname_low);
			batt_id = "----";
			invalid_bat = 1;
		}
	} else {
		if ((packvendor == (u8)'N') && (cellvendor == (u8)'3')) {
			batparams = &params_nvt;
			GAUGE_INFO("BATTERY is NVT\n");
			batt_id = "Z2N3";
		} else if ((packvendor == (u8)'S') && (cellvendor == (u8)'3')) {
			batparams = &params_simplo;
			GAUGE_INFO("BATTERY is SIMPLO\n");
			batt_id = "Z2S3";
		} else if ((packvendor == (u8)'C') && (cellvendor == (u8)'3')) {
			batparams = &params_celxpert;
			GAUGE_INFO("BATTERY is CELXPERT\n");
			batt_id = "Z2C3";
		} else {
			batparams = &params_nvt;
			GAUGE_INFO("BATTERY is 0x%02x%02x, not valid, use NVT as default\n", cellvendor, packvendor);
			batt_id = "----";
			invalid_bat = 1;
		}
	}

	// copy params
	for (i = 0; i < 512; i++) {
		chip->batparams.params[i] = batparams->params[i];
	}
	chip->batparams.en_i2c_sh = batparams->en_i2c_sh;
	chip->batparams.fullvoltage_thr = batparams->fullvoltage_thr;
	chip->batparams.designcapacity = batparams->designcapacity;
	chip->batparams.charge_terminal_current = batparams->charge_terminal_current;
	chip->batparams.paramrevision = batparams->paramrevision;
	chip->batparams.batterycode = batparams->batterycode;
	chip->batparams.default_cyclecount = batparams->default_cyclecount;
	chip->batparams.default_batterycapacity = batparams->default_batterycapacity;
	chip->batparams.default_batteryimpedance = batparams->default_batteryimpedance;
	chip->cyclecount = batparams->default_cyclecount;
	chip->batterycapacity = batparams->default_batterycapacity;
	chip->batteryimpedance = batparams->default_batteryimpedance;
	ret = mm8033_readx_reg(chip->client, 0xBE, buf, (u16)8);
	if (ret) return ret;
	parameter_version = ((buf[7] & 0xff) << 8) | (buf[6] & 0xff);
	battery_ctrlcode = ((buf[5] & 0xff) << 8) | (buf[4] & 0xff);

	/* reload battery parameter if para version is 0xffff when AC in and batt voltage >= 3.9V */
	ret = mm8033_voltage(chip, &val);
	if (ret) return ret;
	GAUGE_INFO("para=0x%04x, val=%d\n", parameter_version, val);
	if ((val>=3900)&&(parameter_version==0xffff)) {
		GAUGE_INFO("set parameter\n");
		ret = mm8033_setFgParameter(chip);
		if (ret) {
			GAUGE_ERR("set parameter fail!\n");
		}
		ret = mm8033_readx_reg(chip->client, 0xBE, buf, (u16)8);
		if (ret) return ret;
		parameter_version = ((buf[7] & 0xff) << 8) | (buf[6] & 0xff);
	}
	if((Read_HW_ID()!=HW_ID_SR1)&&(Read_HW_ID()!=HW_ID_SR2)) {
		ret = mm8033_checkRamData(chip);
		if (ret) return ret;
	}

	return 0;
}

#ifdef REGISTER_POWER_SUPPLY
static enum power_supply_property mm8033_battery_props[] = {
	POWER_SUPPLY_PROP_CAPACITY,
	POWER_SUPPLY_PROP_VOLTAGE_NOW,
	POWER_SUPPLY_PROP_CURRENT_NOW,
	POWER_SUPPLY_PROP_TEMP,
};

static int mm8033_get_property(struct power_supply *psy,
			enum power_supply_property psp,
			union power_supply_propval *val)
{
	int ret = 0;
	struct mm8033_chip *chip = container_of(psy, struct mm8033_chip, battery);

	if (chip->skipcheck > SKIP) {
		chip->skipcheck = 0;
		ret = mm8033_checkRamData(chip);
	} else {
		chip->skipcheck++;
	}

	switch (psp) {
	case POWER_SUPPLY_PROP_CAPACITY:
		ret = mm8033_soc(chip, &(val->intval));
		break;
	case POWER_SUPPLY_PROP_VOLTAGE_NOW:
		ret = mm8033_voltage(chip, &(val->intval));
		break;
	case POWER_SUPPLY_PROP_CURRENT_NOW:
		ret = mm8033_current(chip, &(val->intval));
		break;
	case POWER_SUPPLY_PROP_TEMP:
		ret = mm8033_temperature(chip, &(val->intval));
		break;
	default:
		return -EINVAL;
	}
	return ret;
}
#endif

/* +++for tbl usage+++*/
int mm8033_read_current(void)
{
	int curr = 0;

	if (!i2c_rw_lock)
		curr = mm8033_read_reg(g_mm8033_chip->client, REG_CURRENT);
	else
		curr = g_curr;

	if (curr < 0) {
		GAUGE_ERR("error in reading battery current = 0x%04x\n", curr);
		return g_curr;
	}else {
		if (curr > 32767) {
			curr -= 65536;
		}
		curr += 0x10000;
		//GAUGE_INFO("curr = %d\n", curr);
		g_curr = curr;
		return curr;
	}
}
int mm8033_read_volt(void)
{
	int volt = 0;

	if (!i2c_rw_lock)
		volt = mm8033_read_reg(g_mm8033_chip->client, REG_VOLTAGE);
	else
		volt = g_volt;

	if (volt < 0) {
		GAUGE_ERR("error in reading battery voltage = 0x%04x\n", volt);
		return g_volt;
	}else {
		//GAUGE_INFO("volt = %d\n", volt);
		g_volt = volt;
		return volt;
	}
}
int mm8033_read_percentage(void)
{
	int soc = 0;
#ifdef SOC_SMOOTH_ALGO
	int soc_final = 0, ret = 0;
#endif

	if (!i2c_rw_lock)
		soc = mm8033_read_reg(g_mm8033_chip->client, REG_SOC);
	else
		soc = g_soc;
	if (soc < 0) {
		GAUGE_ERR("error in reading battery soc = 0x%04x\n", soc);
		return g_soc;
	} else {
		//GAUGE_INFO("percentage = %d\n", soc / 256);
		/*>.5% + 1%*/
		if (((soc+128) / 256)>100) {
			return 100;
		} else {
#ifdef SOC_SMOOTH_ALGO
			soc_final = (soc+128) / 256;
			GAUGE_INFO("%s start smooth algo, soc last=%d, soc now=%d\n",__func__, mm8033_pre_soc, soc_final);
			if (mm8033_pre_soc!=0) {
				if ((mm8033_pre_soc+3) <= soc_final) {
					/* reset ocv if gauge soc=100, now soc<100, 0<cur<150 */
					if ((soc_final==100)&&((mm8033_read_current()-0x10000)>0)&&((mm8033_read_current()-0x10000)<150)) {
						i2c_rw_lock = 1;
						ret = mm8033_write_reg(g_mm8033_chip->client, REG_FG_CONDITION, 0x0020);
						if (ret < 0) {
							GAUGE_ERR("reset OCV failed with ret=%d\n", ret);
						} else {
							msleep(100);
							GAUGE_INFO("reset OCV success\n");
						}
						i2c_rw_lock = 0;
					}
					soc_final = mm8033_pre_soc + 3;
				} else if((soc_final>2)&&(mm8033_read_volt()>3200)&&(mm8033_read_volt()<3300)) {
					/* reset ocv if gauge soc>2, 3.2V<bat<3.3V*/
					i2c_rw_lock = 1;
					ret = mm8033_write_reg(g_mm8033_chip->client, REG_FG_CONDITION, 0x0020);
					if (ret < 0) {
						GAUGE_ERR("reset OCV failed with ret=%d\n", ret);
					} else {
						msleep(100);
						GAUGE_INFO("reset OCV success\n");
					}
					i2c_rw_lock = 0;
				}
			}
			mm8033_pre_soc = soc_final;
			g_soc = soc_final;
			return soc_final;
#else
			g_soc = (soc+128) / 256;
			return (soc+128) / 256;
#endif
		}
	}
}
int mm8033_read_temp(void)
{
	int temp = 0;

	if (!i2c_rw_lock)
		temp = mm8033_read_reg(g_mm8033_chip->client, REG_TEMPERATURE);
	else
		temp = g_temp;

	if (temp < 0) {
		GAUGE_ERR("error in reading battery temperature = 0x%04x\n", temp);
		return g_temp;
	}else {
		if (temp > 32767) {
			temp -= 65536;
		}
		//GAUGE_INFO("temp = %d\n", temp);
		g_temp = temp;
		return temp;
	}

}
int mm8033_read_fcc(void)
{
	int fcc = 0;

	if (!i2c_rw_lock)
		fcc = mm8033_read_reg(g_mm8033_chip->client, REG_FULL_CHARGE_CAPACITY);
	else
		fcc = g_fcc;

	if (fcc < 0) {
		GAUGE_ERR("error in reading battery fcc = 0x%04x\n", fcc);
		return g_fcc;
	}else {
		//GAUGE_INFO("fcc = %d\n", fcc);
		g_fcc = fcc;
		return 3000;
	}
}
int mm8033_read_rm(void)
{
	int rm = 0;

	if (!i2c_rw_lock)
		rm = mm8033_read_reg(g_mm8033_chip->client, REG_REM_CAPACITY);
	else
		rm = g_rm;

	if (rm < 0) {
		GAUGE_ERR("error in reading battery rm = 0x%04x\n", rm);
		return g_rm;
	}else {
		//GAUGE_INFO("rm = %d\n", rm);
		g_rm = rm;
		return rm;
	}
}
int mm8033_read_cyclecount(void)
{
	int cc = mm8033_read_reg(g_mm8033_chip->client, REG_CYCLE_COUNT);

	if (cc < 0) {
		GAUGE_ERR("error in reading battery cyclecount = 0x%04x\n", cc);
	}
    return cc;
}
int mm8033_read_designcapacity(void)
{
	int dc = mm8033_read_reg(g_mm8033_chip->client, REG_DESIGN_CAPACITY);

	if (dc < 0) {
		GAUGE_ERR("error in reading battery designcapacity = 0x%04x\n", dc);
	}
    return dc;
}
static int mm8033_read_soh(void)
{
	int soh = mm8033_read_reg(g_mm8033_chip->client, REG_SOH);

	if (soh < 0) {
		GAUGE_ERR("error in reading battery soh = 0x%04x\n", soh);
		return soh;
	}
	return soh;
}
/* ---for tbl usage---*/

static void batt_state_func(struct work_struct *work)
{
	u8 buf[8];

	GAUGE_INFO("%s +++ in every 60s \n",__func__);
	mm8033_readx_reg(g_mm8033_chip->client, 0xBE, buf, (u16)8);
	parameter_version = ((buf[7] & 0xff) << 8) | (buf[6] & 0xff);
	battery_ctrlcode = ((buf[5] & 0xff) << 8) | (buf[4] & 0xff);
	GAUGE_INFO("0x00=0x%04x, 0x06=0x%04x, 0x08=0x%04x, 0x09=0x%04x, 0x0A=0x%04x, 0x0C=0x%04x, 0x10=0x%04x, 0x17=0x%04x, 0x18=0x%04x, 0x1D=0x%04x, 0x23=0x%04x\n",
				  mm8033_read_reg(g_mm8033_chip->client, REG_STATUS),
				  mm8033_read_reg(g_mm8033_chip->client, REG_SOC),
				  mm8033_read_reg(g_mm8033_chip->client, REG_TEMPERATURE),
				  mm8033_read_reg(g_mm8033_chip->client, REG_VOLTAGE),
				  mm8033_read_reg(g_mm8033_chip->client, REG_CURRENT),
				  mm8033_read_reg(g_mm8033_chip->client, REG_IC_TEMPERATURE),
				  mm8033_read_reg(g_mm8033_chip->client, REG_FULL_CHARGE_CAPACITY),
				  mm8033_read_reg(g_mm8033_chip->client, REG_CYCLE_COUNT),
				  mm8033_read_reg(g_mm8033_chip->client, REG_DESIGN_CAPACITY),
				  mm8033_read_reg(g_mm8033_chip->client, REG_CONFIG),
				  mm8033_read_reg(g_mm8033_chip->client, REG_BATTERY_CAPACITY)
				  );
	if(invalid_bat==1) {
		GAUGE_INFO("info: ---- %04x%04x ------ version: %s\n", parameter_version, battery_ctrlcode, DRIVER_VERSION);
	}else {
		GAUGE_INFO("info: %s %04x%04x %d%d%d version: %s\n", batt_id, parameter_version, battery_ctrlcode, bat_year, bat_week_1, bat_week_2, DRIVER_VERSION);
	}
	/* check ram data after state work 10 times */
	if (g_mm8033_chip->skipcheck > SKIP) {
		g_mm8033_chip->skipcheck = 0;
		mm8033_checkRamData(g_mm8033_chip);
	} else {
		g_mm8033_chip->skipcheck++;
	}
	schedule_delayed_work(&batt_state_wq, 60*HZ);
}

/****Add battery status proc file+++*****/
static struct proc_dir_entry *battery_status_proc_file;
static int battery_status_proc_read(struct seq_file *buf, void *v)
{
	seq_printf(buf, "FCC=%d(mAh),DC=%d(mAh),RM=%d(mAh),TEMP=%d(C),VOLT=%d(mV),CUR=%d(mA),CC=%d,SOH=%d(%)\n",
		mm8033_read_fcc(),
		mm8033_read_designcapacity(),
		mm8033_read_rm(),
		mm8033_read_temp()/10,
		mm8033_read_volt(),
		mm8033_read_current(),
		mm8033_read_cyclecount(),
		mm8033_read_soh()
	);
	return 0;
}

static int battery_status_proc_open(struct inode *inode, struct  file *file)
{
    return single_open(file, battery_status_proc_read, NULL);
}


static struct file_operations battery_status_proc_ops = {
	.open = battery_status_proc_open,
	.read = seq_read,
	.release = single_release,
};

static void create_battery_status_proc_file(void)
{
    GAUGE_INFO("create_battery_status_proc_file\n");
    battery_status_proc_file = proc_create("battery_soh", 0444,NULL, &battery_status_proc_ops);
    if(battery_status_proc_file){
        GAUGE_INFO("create battery_status_proc_file sucessed!\n");
    }else{
		GAUGE_INFO("create battery_status_proc_file failed!\n");
    }
}
/****Add battery status proc file---*****/

static ssize_t batt_switch_name(struct switch_dev *sdev, char *buf)
{
	if(invalid_bat==1) {
		return sprintf(buf, "---- %04x%04x ------\n", parameter_version, battery_ctrlcode);
	}else {
		return sprintf(buf, "%s %04x%04x %d%d%d\n", batt_id, parameter_version, battery_ctrlcode, bat_year, bat_week_1, bat_week_2);
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void mm8033_early_suspend(struct early_suspend *h)
{
	GAUGE_INFO("%s ++\n", __func__);
	cancel_delayed_work(&batt_state_wq);
}

static void mm8033_late_resume(struct early_suspend *h)
{
	GAUGE_INFO("%s ++\n", __func__);
	schedule_delayed_work(&batt_state_wq, 5*HZ);
}
static void mm8033_config_earlysuspend(struct mm8033_chip *chip)
{
	chip->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
	chip->es.suspend = mm8033_early_suspend;
	chip->es.resume = mm8033_late_resume;
	register_early_suspend(&chip->es);
}
#else
#define mm8033_early_suspend NULL
#define mm8033_late_resume NULL
static void mm8033_config_earlysuspend(struct mm8033_chip *chip)
{
	return;
}
#endif

static int mm8033_probe(struct i2c_client *client, const struct i2c_device_id *id)
{
	struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);
	struct mm8033_chip *chip;
	int ret;
	u32 test_major_flag=0;
	struct asus_bat_config bat_cfg;

	GAUGE_INFO("%s ++\n", __func__);

	//turn to jiffeys
	bat_cfg.polling_time = 0;
	bat_cfg.critical_polling_time = 0;
	bat_cfg.polling_time *= HZ;
	bat_cfg.critical_polling_time *= HZ;

	if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_WORD_DATA))
		return -EIO;

	chip = kzalloc(sizeof(*chip), GFP_KERNEL);
	if (!chip)
		return -ENOMEM;
	chip->client = client;
	i2c_set_clientdata(client, chip);

#ifdef REGISTER_POWER_SUPPLY
	chip->battery.name = "mm8033_battery";
	chip->battery.type = POWER_SUPPLY_TYPE_BATTERY;
	chip->battery.get_property = mm8033_get_property;
	chip->battery.properties = mm8033_battery_props;
	chip->battery.num_properties = 4;

	ret = power_supply_register(&client->dev, &chip->battery);
	if (ret) {
		dev_err(&client->dev, "failed: power supply register\n");
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		return ret;
	}
#endif
	// read stored data
	chip->skipcheck = SKIP;

	ret = mm8033_checkdevice(chip);
#ifdef STOP_IF_FAIL
	if (ret) {
		dev_err(&client->dev, "failed to access\n");
		i2c_set_clientdata(client, NULL);
		kfree(chip);
		return ret;
	}else {
		GAUGE_INFO("init asus battery\n");
		ret = asus_battery_init(bat_cfg.polling_time, bat_cfg.critical_polling_time, test_major_flag);
		if (ret)
			GAUGE_ERR("asus_battery_init fail\n");
	}
#else
	if (ret) {
		GAUGE_ERR("error in checkdevice with ret: %d\n", ret);
	}else {
		GAUGE_INFO("checkdevice success\n");
	}
	GAUGE_INFO("init asus battery\n");
	ret = asus_battery_init(bat_cfg.polling_time, bat_cfg.critical_polling_time, test_major_flag);
	if (ret)
		GAUGE_ERR("asus_battery_init fail\n");
#endif
	g_mm8033_chip = chip;

	/* register switch device for battery information versions report */
	mm8033_batt_dev.name = "battery";
	mm8033_batt_dev.print_name = batt_switch_name;
	if (switch_dev_register(&mm8033_batt_dev) < 0)
		GAUGE_ERR("%s: fail to register battery switch\n", __func__);
	switch_set_state(&mm8033_batt_dev, 0);

	/* register power supply in asus_battery_power.c */
	mm8033_tbl.read_percentage = mm8033_read_percentage;
	mm8033_tbl.read_current = mm8033_read_current;
	mm8033_tbl.read_volt = mm8033_read_volt;
	mm8033_tbl.read_temp = mm8033_read_temp;
	mm8033_tbl.read_fcc = mm8033_read_fcc;
	mm8033_tbl.read_rm = mm8033_read_rm;
	mm8033_tbl.read_soh = mm8033_read_soh;
	ret = asus_register_power_supply(&client->dev, &mm8033_tbl);
	if (ret)
                GAUGE_ERR("asus_register_power_supply fail\n");

	INIT_DELAYED_WORK(&batt_state_wq, batt_state_func);
	schedule_delayed_work(&batt_state_wq, 10*HZ);
#ifdef CONFIG_HAS_EARLYSUSPEND
	mm8033_config_earlysuspend(chip);
#endif
    //print battery status in proc/battery_soh
	create_battery_status_proc_file();

	GAUGE_INFO("%s --\n", __func__);
	return 0;
}

static int mm8033_remove(struct i2c_client *client)
{
	struct mm8033_chip *chip = i2c_get_clientdata(client);
#ifdef REGISTER_POWER_SUPPLY
	power_supply_unregister(&chip->battery);
#endif
	i2c_set_clientdata(client, NULL);
	kfree(chip);

	return 0;
}

static const struct i2c_device_id mm8033_id[] = {
	{ "mm8033_batt", 0 },
	{},
};
MODULE_DEVICE_TABLE(i2c, mm8033_id);

static struct i2c_driver mm8033_i2c_driver = {
	.driver	= {
		.name	= "mm8033_batt",
	},
	.probe	= mm8033_probe,
	.remove	= mm8033_remove,
	.id_table	= mm8033_id,
};
#ifdef REGISTER_I2C
static struct i2c_board_info mm8033_board_info[] = {
	{
		I2C_BOARD_INFO("mm8033", 0x36),
	},
};
#endif
static int __init mm8033_init(void)
{
	int ret;

	GAUGE_INFO("%s +++\n", __func__);
	if (Read_HW_ID()==HW_ID_EVB) {
		GAUGE_INFO("HW version is EVB, so donot init\n");
		return 0;
	} else if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
		GAUGE_INFO("Project version is ZX550ML, so donot init\n");
		return 0;
	} else if (!((Read_PROJ_ID()==PROJ_ID_ZE550ML)||(Read_PROJ_ID()==PROJ_ID_ZE551ML)||
			(Read_PROJ_ID()==PROJ_ID_ZE551ML_CKD)||(Read_PROJ_ID()==PROJ_ID_ZE551ML_ESE))) {
		GAUGE_INFO("Project version is not ZE55xML, so donot init\n");
		return 0;
	}

	ret = i2c_add_driver(&mm8033_i2c_driver);
	if (ret)
		GAUGE_ERR("%s: i2c_add_driver failed\n", __func__);
#ifdef REGISTER_I2C
	ret = i2c_register_board_info(2, mm8033_board_info, ARRAY_SIZE(mm8033_board_info));
	if (ret)
		GAUGE_ERR("%s: i2c_register_board_info failed\n", __func__);
#endif

	GAUGE_INFO("%s ---\n", __func__);
	return ret;
}
late_initcall(mm8033_init);

static void __exit mm8033_exit(void)
{
	i2c_del_driver(&mm8033_i2c_driver);
}
module_exit(mm8033_exit);

MODULE_AUTHOR("Chih-Hsuan Chang <Chih-Hsuan_Chang@asus.com>");
MODULE_DESCRIPTION("mm8033a01 battery monitor driver");
MODULE_LICENSE("GPL");
