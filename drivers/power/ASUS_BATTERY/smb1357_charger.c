/*
 * Copyright (c) 2014, ASUSTek, Inc. All Rights Reserved.
 * Written by wigman sun wigman_sun@asus.com
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
#include <linux/power/smb1357-charger.h>
#include <linux/proc_fs.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>	/* Needed for KERN_DEBUG */
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_gpadc.h>
#include <asm/intel_mid_thermal.h>
#include <asm/pmic_pdata.h>
#include <linux/workqueue.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#include "smb_external_include.h"
#include "asus_battery.h"
#include <linux/HWVersion.h>
#include <linux/earlysuspend.h>

#define PMIC_CHARGER		1
#define GPIO_USB_SWITH  		"CHG_PHY_ON"
#define GPIO_CHARGE_INPUT 	"CHG_PMOS_EN#"

/*smb1357 registers*/
#define STAT_PRODUCT_ID_REG 				0x32
#define CFG_PRE_FAST_CHARGE_CURRENT_REG 	0x1c
#define CFG_TERMINATION_CURRENT_REG  		0x03
#define CFG_CURRENT_COMPEN_REG  			0x1d
#define CFG_CHARGE_EN_REG 				0x14
#define CFG_FLOAT_VOLTAGE_REG 				0x1e
#define CFG_TEMP_BEHAVIOR_REG 			0x1a
#define RESULT_AICL_REG 					0x46
#define RESULT_STATUS_REG 					0x47
#define CFG_AICL_REG 						0x0d
#define CFG_INPUT_CURRENT_LIMIT_REG 		0x0c
#define CFG_OTG_I_LIMIT_REG 				0x12
#define STAT_CHARGE_REG 					0x4a
#define STAT_PRE_FAST_CHARGE_CURRENT_REG	0x49
#define STAT_PWRSRC_REG 					0x4b
#define CFG_WRITE_REG 						0x40
#define STAT_HVDCP_REG 					0x4d
#define CFG_RECHARGE_REG 					0x05
#define CMD_I2C_REG   						0x40
#define CMD_IL_REG    						0x41
#define CMD_CHG_REG   					0x42
#define WATCHDOG_REG					0x10
#define HVDCP_STATUS_REG					0x0e
#define IRQ_E_REG							0x54
#define IRQ_F_REG							0x55

/*smb1357 config values*/
#define CFG_HOT_LIMIT 					0x1b
#define CFG_FLOAT_438  				0x2d
#define CFG_FLOAT_434  				0x2a
#define CFG_FLOAT_410  				0x1e
#define CFG_FLOAT_400  				0x19
#define PRECHARGE_250MA 				0x60
#define PRECHARGE_100MA 				0x00
#define FAST_CHARGE_3000MA 			0x1f
#define FAST_CHARGE_2050MA 			0x1a
#define FAST_CHARGE_1400MA 			0x0e
#define FAST_CHARGE_1000MA 			0x0b
#define FAST_CHARGE_700MA  			0x08
#define FAST_CHARGE_400MA 			0x01
#define STAT_HOLDOFF  					BIT(3)
#define STAT_CHG_MASK					0x06
#define STAT_CHG_SHIFT					1
#define STAT_CHG_TERM					BIT(5)
#define STAT_CHG_ERROR     				BIT(2)
#define STAT_DCP_MASK      				BIT(6)
#define STAT_AICL_DONE     				BIT(7)
#define STAT_AICL_1500MA   				BIT(4)
#define STAT_AICL_500MA    				BIT(2)
#define INPUT_CURRENT_LIMIT_700MA 		BIT(3)
#define INPUT_CURRENT_LIMIT_900MA 		BIT(0)|BIT(3)
#define INPUT_CURRENT_LIMIT_1000MA 	BIT(0)|BIT(1)|BIT(3)
#define INPUT_CURRENT_LIMIT_1200MA 	BIT(0)|BIT(2)|BIT(3)
#define INPUT_CURRENT_LIMIT_1400MA 	BIT(1)|BIT(2)|BIT(3)
#define INPUT_CURRENT_LIMIT_1600MA 	BIT(0)|BIT(4)
#define INPUT_CURRENT_LIMIT_1800MA 	BIT(1)|BIT(4)
#define INPUT_CURRENT_LIMIT_1910MA 	BIT(0)|BIT(2)|BIT(4)

#define CHR_INFO(...)	printk("[SMB1357] " __VA_ARGS__);
#define CHR_ERR(...)	printk("[SMB1357_ERR] " __VA_ARGS__);

#define EXPORT_CHARGER_OTG
#define SMB1357_OTG_PMIC_PIN	0x84
#define PMIC_ENABLE_OTG_MODE	0x4c
#define SCHGRIRQ1_REG			0x4f

/**
 * struct smb1357_charger - smb1357 charger instance
 * @lock: protects concurrent access to online variables
 * @client: pointer to i2c client
 * @mains: power_supply instance for AC/DC power
 * @usb: power_supply instance for USB power
 * @dentry: for debugfs
 * @pdata: pointer to platform data
 * @power_state_wrkr: print info for debugging
 * @query_DCPmode_wrkr: check DCP mode work
 * @gpio_usb_swith: gpio_usb_swith for QC usage
 * @gpio_chrg_signal: gpio_chrg_signal for QC usage
 */
struct smb1357_charger {
	struct mutex		lock;
	struct i2c_client		*client;
	struct power_supply	mains;
	struct power_supply	usb;
	struct dentry		*dentry;
	const struct smb1357_charger_platform_data	*pdata;
	struct delayed_work 	power_state_wrkr;
	struct delayed_work 	query_DCPmode_wrkr;
	int	gpio_usb_swith;
	int	gpio_chrg_signal;
#ifdef CONFIG_HAS_EARLYSUSPEND
	struct early_suspend es;
#endif
};

static char *smb1357_power_supplied_to[] = {
			"smb1357_battery",
};

static struct smb1357_charger *smb1357_dev;
static int not_ready_flag=1, dcp_count=0, g_cable_status=0, debug_flag=0, first_out_flag=0, usb_detect_flag=0, gpio57_flag=0, otg_flag=0, disable_chrg=0;
static int first_sdp_mode=0;
static struct delayed_work inok_work; //WA for HTC 5W adaptor
static struct delayed_work set_cur_work; //WA to decrease inpuit current when panel on for thermal
int chr_suspend_flag=0, hvdcp_mode=0, dcp_mode=0, set_usbin_cur_flag=0;
EXPORT_SYMBOL(hvdcp_mode);
EXPORT_SYMBOL(dcp_mode);
EXPORT_SYMBOL(set_usbin_cur_flag);
EXPORT_SYMBOL(chr_suspend_flag);

extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
extern struct battery_info_reply batt_info;
extern unsigned int query_cable_status(void);
extern int boot_mode;
#ifdef CONFIG_TOUCHSCREEN_FTXXXX
extern void ftxxxx_usb_detection(bool plugin); //usb_cable_status : write touch reg 0x8B
#endif
struct wake_lock wakelock_cable, wakelock_cable_t;
int early_suspend_flag=0;
EXPORT_SYMBOL(early_suspend_flag);

static int smb1357_read(struct smb1357_charger *smb, u8 reg)
{
	int ret, i;

	for (i=0;i<5;i++) {
		ret = i2c_smbus_read_byte_data(smb->client, reg);
		if (ret < 0)
			msleep(10);
		else
			break;
	}
	if (ret < 0)
		CHR_ERR("failed to read reg 0x%02x: %d\n", reg, ret);

	return ret;
}

static int smb1357_write(struct smb1357_charger *smb, u8 reg, u8 val)
{
	int ret;

	ret = i2c_smbus_write_byte_data(smb->client, reg, val);
	if (ret < 0)
		CHR_ERR("failed to write reg 0x%02x: %d\n", reg, ret);

	return ret;
}

/*
 * smb1357_set_writable - enables/disables writing to non-volatile registers
 * @smb: pointer to smb1357 charger instance
 *
 * You can enable/disable writing to the non-volatile configuration
 * registers by calling this function.
 *
 * Returns %0 on success and negative errno in case of failure.
 */
static int smb1357_set_writable(struct smb1357_charger *smb, bool writable)
{
	int ret;

	ret = smb1357_read(smb, CFG_WRITE_REG);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= BIT(6)|BIT(0);
	else
		ret &= ~(BIT(6));

	return smb1357_write(smb, CFG_WRITE_REG, ret);
}

static int enable_AICL(void)
{
	int ret=0;
	
	CHR_INFO("%s +++\n", __func__);
	
	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	ret = smb1357_read(smb1357_dev,CFG_AICL_REG);
	if (ret < 0)
		goto out;

	ret |= BIT(2);

	ret = smb1357_write(smb1357_dev,CFG_AICL_REG,ret);
	if (ret < 0)
		goto out;
	
out:
	return ret;
}

static int disable_AICL(void)
{
	int ret=0;
	
	CHR_INFO("%s +++\n", __func__);
	
	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	ret = smb1357_read(smb1357_dev,CFG_AICL_REG);
	if (ret < 0)
		goto out;

	ret &= ~(BIT(2));

	ret = smb1357_write(smb1357_dev,CFG_AICL_REG,ret);
	if (ret < 0)
		goto out;
	
out:
	return ret;
}

/*This function is used to set DCP/HVDCP input current limit ,
    if plug DCP with aicl step to 1100mA, you should call set_QC_inputI_limit(0)
    if plug HVDCP with aicl step to 1910mA, you should call set_QC_inputI_limit(1)
    if plug DCP without aicl step to 1200mA, you should call set_QC_inputI_limit(2)
    if plug DCP without aicl step to 1000mA, you should call set_QC_inputI_limit(3)
    if plug DCP with aicl step to 1000mA, you should call set_QC_inputI_limit(4)
    if plug DCP without aicl step to 900mA, you should call set_QC_inputI_limit(5)
    if plug DCP without aicl step to 500mA, you should call set_QC_inputI_limit(6)
    if plug DCP without aicl step to 700mA, you should call set_QC_inputI_limit(7)
    if plug DCP without aicl step to 300mA, you should call set_QC_inputI_limit(8)
*/
int set_QC_inputI_limit(int type)
{
	int ret;

	CHR_INFO("%s +++ and type=%d, set_usbin_cur_flag=%d\n", __func__, type, set_usbin_cur_flag);
	if(set_usbin_cur_flag==1) {
		CHR_INFO("usb in current is setting, so ignore this event\n");
		return 0;
	}else {
		set_usbin_cur_flag = 1;
	}
	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;
	CHR_INFO("need to wait 1s before setting input current\n");
	msleep(1000);
	if (type==1) {
		CHR_INFO("HVDCP plug in with aicl step to 1910mA! \n");
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 700mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 700mA\n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(4));
		ret |= (BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
		msleep(1000);
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		if((ret&0x1f) >= (INPUT_CURRENT_LIMIT_700MA)) {
			/* Config INPUT_CURRENT_LIMIT_REG register 1000mA*/
			CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1000mA\n");
			disable_AICL();
			ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
			if (ret < 0)
				goto out;
			ret &= ~(BIT(2)|BIT(4));
			ret |= (BIT(0)|BIT(1)|BIT(3));
			ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
			enable_AICL();
			msleep(1000);
			ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
			if (ret < 0)
				goto out;
			CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
			if((ret&0x1f) >= (INPUT_CURRENT_LIMIT_1000MA)) {
				/* Config INPUT_CURRENT_LIMIT_REG register 1400mA*/
				CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1400mA\n");
				disable_AICL();
				ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
				if (ret < 0)
					goto out;
				ret &= ~(BIT(0)|BIT(4));
				ret |= (BIT(1)|BIT(2)|BIT(3));
				ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
				enable_AICL();
				msleep(1000);
				ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
				if (ret < 0)
					goto out;
				CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
				if((ret&0x1f) >= (INPUT_CURRENT_LIMIT_1400MA)) {
					/* Config INPUT_CURRENT_LIMIT_REG register 1600mA*/
					CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1600mA\n");
					disable_AICL();
					ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
					if (ret < 0)
						goto out;
					ret &= ~(BIT(1)|BIT(2)|BIT(3));
					ret |= (BIT(0)|BIT(4));
					ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
					enable_AICL();
					msleep(1000);
					ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
					if (ret < 0)
						goto out;
					CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
					if ((ret&0x1f) >= (INPUT_CURRENT_LIMIT_1600MA)) {
						/* Config INPUT_CURRENT_LIMIT_REG register 1910mA*/
						CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1910mA\n");
						disable_AICL();
						ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
						if (ret < 0)
							goto out;
						ret &= ~(BIT(1)|BIT(3));
						ret |= (BIT(0)|BIT(2)|BIT(4));
						ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
						enable_AICL();
						msleep(1000);
					}
				}
			}
		}
	} else if (type==0) {
		// I USB IN should bigger than 1100mA in DCP
		CHR_INFO("DCP plug in with aicl step to 1100mA! \n");
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 700mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 700mA\n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(4));
		ret |= (BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
		msleep(1000);
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		if((ret&0x1f) >= (INPUT_CURRENT_LIMIT_700MA)) {
			/* Config INPUT_CURRENT_LIMIT_REG register 900mA*/
			CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 900mA\n");
			disable_AICL();
			ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
			if (ret < 0)
				goto out;
			ret &= ~(BIT(1)|BIT(2)|BIT(4));
			ret |= (BIT(0)|BIT(3));
			ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
			enable_AICL();
			msleep(1000);
			ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
			if (ret < 0)
				goto out;
			CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
			if((ret&0x1f) >= (INPUT_CURRENT_LIMIT_900MA)) {
				/* Config INPUT_CURRENT_LIMIT_REG register 1100mA*/
				CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1100mA\n");
				disable_AICL();
				ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
				if (ret < 0)
					goto out;
				ret &= ~(BIT(0)|BIT(1)|BIT(4));
				ret |= (BIT(2)|BIT(3));
				ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
				enable_AICL();
			}
		}
	} else if (type==3) {
		// I USB IN should bigger than 1000mA
		CHR_INFO("DCP plug in without aicl step to 1000mA,! \n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 1000mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1000mA without aicl\n");
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(2)|BIT(4));
		ret |= (BIT(0)|BIT(1)|BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
	} else if (type==4) {
		// I USB IN should bigger than 1000mA in other charging port
		CHR_INFO("DCP/CDP/Other plug in with aicl step to 1000mA! \n");
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 700mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 700mA\n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(4));
		ret |= (BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
		msleep(1000);
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		if((ret&0x1f) >= (INPUT_CURRENT_LIMIT_700MA)) {
			/* Config INPUT_CURRENT_LIMIT_REG register 1000mA*/
			CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1000mA\n");
			disable_AICL();
			ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
			if (ret < 0)
				goto out;
			ret &= ~(BIT(2)|BIT(4));
			ret |= (BIT(0)|BIT(1)|BIT(3));
			ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
			enable_AICL();
		}
	} else if (type==2) {
		// I USB IN should bigger than 1200mA
		CHR_INFO("DCP plug in without aicl step to 1200mA,! \n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 1200mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 1200mA without aicl\n");
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(1)|BIT(4));
		ret |= (BIT(0)|BIT(2)|BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
	} else if (type==5) {
		// I USB IN should bigger than 900mA
		CHR_INFO("HVDCP plug in without aicl step to 900mA,! \n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 900mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 900mA without aicl\n");
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(1)|BIT(2)|BIT(4));
		ret |= (BIT(0)|BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
	} else if (type==6) {
		// I USB IN should bigger than 600mA
		CHR_INFO("DCP plug in without aicl step to 500mA,! \n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 500mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 500mA without aicl\n");
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(0)|BIT(1)|BIT(3)|BIT(4));
		ret |= (BIT(2));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
	} else if (type==7) {
		// I USB IN should bigger than 700mA
		CHR_INFO("DCP plug in without aicl step to 700mA,! \n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 700mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 700mA without aicl\n");
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(4));
		ret |= (BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
	} else if (type==8) {
		// I USB IN should bigger than 300mA
		CHR_INFO("DCP plug in without aicl step to 300mA,! \n");
		disable_AICL();
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		if (ret < 0)
			goto out;
		CHR_INFO("RESULT_AICL_REG 0x46=0x%02x\n", ret);
		/* Config INPUT_CURRENT_LIMIT_REG register 300mA*/
		CHR_INFO("Config INPUT_CURRENT_LIMIT_REG register 300mA without aicl\n");
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		if (ret < 0)
			goto out;
		ret &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(3)|BIT(4));
		ret = smb1357_write(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG, ret);
		enable_AICL();
	}
	set_usbin_cur_flag = 0;
	return 0;
out:
	CHR_INFO("%s fail\n", __func__);
	set_usbin_cur_flag = 0;
	return ret;
}


#ifdef EXPORT_CHARGER_OTG
static int otg(int toggle)
{
	int ret=0;
	int ival=0;
	uint8_t ctrldata, otg_mde_data;

	CHR_INFO("%s +++ set otg: %s \n", __func__, toggle ? "on" : "off");

	mutex_lock(&smb1357_dev->lock);
	
	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	if (toggle){
		//set OTG current limit 250mA
		ival = smb1357_read(smb1357_dev, CFG_OTG_I_LIMIT_REG);
		if (ival < 0)
			goto out;
		ival &= ~(BIT(3)|BIT(2));
		ret = smb1357_write(smb1357_dev,CFG_OTG_I_LIMIT_REG,ival);
		if (ret < 0)
			goto out;
		//set PMIC Enable OTG Mode: PMIC 104Ch[6]="1"
		ret = intel_scu_ipc_ioread8(PMIC_ENABLE_OTG_MODE, &otg_mde_data);
		if (ret) {
			CHR_ERR(" IPC Failed to read %d\n", ret);
		}
		otg_mde_data |= BIT(6);
		ret = intel_scu_ipc_iowrite8(PMIC_ENABLE_OTG_MODE, otg_mde_data);
		if (ret) {
				CHR_ERR(" IPC Failed to write %d\n", ret);
		}
		//Enable OTG function: PMIC GPIO 6 = 1
		ret = intel_scu_ipc_ioread8(SMB1357_OTG_PMIC_PIN, &ctrldata);
		if (ret) {
			CHR_ERR(" IPC Failed to read %d\n", ret);
		}
		ctrldata |= (BIT(5)|BIT(4)|BIT(0));
		ret = intel_scu_ipc_iowrite8(SMB1357_OTG_PMIC_PIN, ctrldata);
		if (ret) {
			CHR_ERR(" IPC Failed to write %d\n", ret);
		}
		/*set OTG current limit = 600mA, 12h[3:2]=01*/
		ival = smb1357_read(smb1357_dev, CFG_OTG_I_LIMIT_REG);
		if (ival < 0)
			goto out;
		ival &= ~(BIT(3));
		ival |= BIT(2);
		ret = smb1357_write(smb1357_dev,CFG_OTG_I_LIMIT_REG,ival);
		if (ret < 0)
			goto out;
	}
	else {
		//Disable OTG function: PMIC GPIO 6 = 0
		ret = intel_scu_ipc_ioread8(SMB1357_OTG_PMIC_PIN, &ctrldata);
		if (ret) {
			CHR_ERR(" IPC Failed to read %d\n", ret);
		}
		ctrldata &= ~(BIT(5)|BIT(4)|BIT(0));
		ret = intel_scu_ipc_iowrite8(SMB1357_OTG_PMIC_PIN, ctrldata);
		if (ret) {
			CHR_ERR(" IPC Failed to write %d\n", ret);
		}
		//set PMIC disable OTG Mode: PMIC 104Ch[6]="0"
		ret = intel_scu_ipc_ioread8(PMIC_ENABLE_OTG_MODE, &otg_mde_data);
		if (ret) {
			CHR_ERR(" IPC Failed to read %d\n", ret);
		}
		otg_mde_data &= ~(BIT(6));
		ret = intel_scu_ipc_iowrite8(PMIC_ENABLE_OTG_MODE, otg_mde_data);
		if (ret) {
				CHR_ERR(" IPC Failed to write %d\n", ret);
		}
		//set OTG current limit 250mA
		ival = smb1357_read(smb1357_dev, CFG_OTG_I_LIMIT_REG);
		if (ival < 0)
			goto out;
		ival &= ~(BIT(3)|BIT(2));
		ret = smb1357_write(smb1357_dev, CFG_OTG_I_LIMIT_REG,ival);
		if (ret < 0)
			goto out;
	}
out:
	   mutex_unlock(&smb1357_dev->lock);
	   return ret;
}

int setSMB1357Charger(int usb_state)
{
	int ret = 0;

	CHR_INFO("%s +++, usb state=%d, not_ready_flag=%d\n", __func__, usb_state, not_ready_flag);
	g_cable_status = batt_info.cable_source = usb_state;
	if (!batt_info.isValidBattID) {
		CHR_INFO("%s  not Valid BattID\n", __func__);
		smb1357_charging_toggle(false);
		return 0;
	}

	if(not_ready_flag&&(usb_state==ENABLE_5V||usb_state==DISABLE_5V)) {
		CHR_INFO("charger not ready yet\n");
		return 0;
	}
	if (smb1357_dev->pdata->use_mains)
		power_supply_changed(&smb1357_dev->mains);
	if (smb1357_dev->pdata->use_usb)
		power_supply_changed(&smb1357_dev->usb);

	switch (usb_state)
	{
	case USB_IN:
		CHR_INFO("usb_state: USB_IN\n");
		usb_to_battery_callback(USB_PC);
		break;
	case AC_IN:
		CHR_INFO("usb_state: AC_IN\n");
		#ifdef CONFIG_TOUCHSCREEN_FTXXXX
		ftxxxx_usb_detection(true);
		#endif
		usb_to_battery_callback(USB_ADAPTER);
		break;
	case CABLE_OUT:
		CHR_INFO("usb_state: CABLE_OUT\n");
		#ifdef CONFIG_TOUCHSCREEN_FTXXXX
		ftxxxx_usb_detection(false);
		#endif
		usb_to_battery_callback(NO_CABLE);
		break;
	case ENABLE_5V:
		if ((dcp_mode)&&(Read_PROJ_ID()==PROJ_ID_ZX550ML)) {
			dcp_mode = 0;
			CHR_INFO("OTG mode but in ac_in state, send cable out event\n");
			CHR_INFO("usb_state: CABLE_OUT\n");
			usb_to_battery_callback(NO_CABLE);
		}
		if (first_sdp_mode) {
			CHR_INFO("OTG mode but usb_in state first, send cable out event\n");
			CHR_INFO("usb_state: CABLE_OUT\n");
			usb_to_battery_callback(NO_CABLE);
			first_sdp_mode = 0;
		}
		CHR_INFO("usb_state: ENABLE_5V\n");
		ret = otg(1);
		otg_flag = 1;
		break;
	case DISABLE_5V:
		if ((dcp_mode)&&(Read_PROJ_ID()==PROJ_ID_ZX550ML)) {
			dcp_mode = 0;
			CHR_INFO("OTG mode but in ac_in state, send cable out event\n");
			CHR_INFO("usb_state: CABLE_OUT\n");
			usb_to_battery_callback(NO_CABLE);
		}
		CHR_INFO("usb_state: DISABLE_5V\n");
		ret = otg(0);
		otg_flag = 0;
		break;
	default:
		CHR_INFO("ERROR: wrong usb state value = %d\n", usb_state);
		ret = 1;
	}
	return ret;
}
EXPORT_SYMBOL(setSMB1357Charger);
#endif

int smb1357_control_JEITA(bool on)
{
	int ret = 0;

	CHR_INFO("%s +++ control_JEITA: %s \n", __func__, on ? "on" : "off");

	if (!smb1357_dev) {
		CHR_INFO("Warning: smb1357_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_TEMP_BEHAVIOR_REG register */
	ret = smb1357_read(smb1357_dev, CFG_TEMP_BEHAVIOR_REG);
	if (ret < 0)
		goto out;

	if (on) {
		ret &= ~(BIT(0)|BIT(3));
	} else {
		ret |= (BIT(0)|BIT(3)|BIT(6));
	}

	CHR_INFO("write CFG_TEMP_BEHAVIOR_REG 0x1A = 0x%02x\n", ret);
	ret = smb1357_write(smb1357_dev, CFG_TEMP_BEHAVIOR_REG, ret);
	if (ret < 0)
		goto out;

	// === Set Hard Hot Limit  ====
	ret = smb1357_read(smb1357_dev, CFG_HOT_LIMIT);
	if (ret < 0)
		goto out;

	if (on) {
		ret |= (BIT(5)|BIT(4));
	} else {
		ret &= ~(BIT(5)|BIT(1));
		ret |= (BIT(0)|BIT(4));
	}

	CHR_INFO("write control JEITA, Set Hard Hot Limit 0x1B = 0x%02x\n", ret);
	ret = smb1357_write(smb1357_dev,  CFG_HOT_LIMIT, ret);
	if (ret < 0)
		goto out;
	
	if(!on){
		//enable charge
		ret = smb1357_read(smb1357_dev, CFG_CHARGE_EN_REG);
		if (ret < 0)
			goto out;
		ret |= BIT(6);
		ret = smb1357_write(smb1357_dev,	CFG_CHARGE_EN_REG, ret);
		if (ret < 0)
			goto out;
    	}
out:
	mutex_unlock(&smb1357_dev->lock);
	return ret;

}

int smb1357_get_aicl_result(void)
{
        int ret = 0;

	if (!smb1357_dev) {
		CHR_INFO("Warning: smb1357_dev is null due to probe function has error\n");
		return 1;
	}

	CHR_INFO("%s +++\n", __func__);

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	/*AICL result register */
	ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
	if (ret < 0)
		goto out;

	ret &= 0x1f;
	CHR_INFO("smb1357 get aicl result(&0x1f) = 0x%02x\n", ret);

out:
	mutex_unlock(&smb1357_dev->lock);
	return ret;
}

/* set float voltage : true :lowV  false:highV*/
int smb1357_set_voltage(bool on)
{
	int ret = 0;

	CHR_INFO("%s +++ set float voltage: %s \n", __func__, on ? "4.1V" : "4.38V");

	if (!smb1357_dev) {
		CHR_INFO("Warning: smb1357_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_FLOAT_VOLTAGE register */
	if (on) {
		/* voltage = 4.1v */
		ret = CFG_FLOAT_410;
	} else {
		/* voltage = 4.38v */
		ret = CFG_FLOAT_438;
	}

	ret = smb1357_write(smb1357_dev, CFG_FLOAT_VOLTAGE_REG, ret);
	if (ret < 0)
		goto out;
	
out:
	mutex_unlock(&smb1357_dev->lock);
	return ret;
}

int smb1357_set_fast_charge(void)
{
	int ret = 0;

	CHR_INFO("%s +++\n", __func__);
	
	if (!smb1357_dev) {
		CHR_INFO("Warning: smb1357_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	/*config fast-charge current=2800*/
	ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
	if (ret < 0)
		goto out;
	ret |= (BIT(1)|BIT(2)|BIT(4));
	ret &= ~(BIT(0)|BIT(3));
	ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
	if (ret < 0)
		goto out;
		/* Config termination current 100mA */
	ret = smb1357_read(smb1357_dev, CFG_TERMINATION_CURRENT_REG);
	if (ret < 0)
		goto out;
	ret |= BIT(4);
	ret &= ~(BIT(3)|BIT(5));
	ret = smb1357_write(smb1357_dev, CFG_TERMINATION_CURRENT_REG, ret);
	if (ret < 0)
		goto out;
	/*config fastcharge current compensation 700mA*/
	ret = smb1357_read(smb1357_dev, CFG_CURRENT_COMPEN_REG);
	if (ret < 0)
		goto out;
	ret |= BIT(6);
	ret &= ~(BIT(7));
	ret = smb1357_write(smb1357_dev, CFG_CURRENT_COMPEN_REG, ret);
	if (ret < 0)
		goto out;
		/*disable HW Jeita*/
	ret = smb1357_read(smb1357_dev, CFG_TEMP_BEHAVIOR_REG);
	if (ret < 0)
		goto out;
	ret &= ~(BIT(6));
	ret = smb1357_write(smb1357_dev, CFG_TEMP_BEHAVIOR_REG, ret);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&smb1357_dev->lock);
	return ret;
}

int smb1357_watchdog_timer_enable(void)
{
	int ret = 0;

	CHR_INFO("%s +++\n", __func__);

	if (!smb1357_dev) {
		CHR_INFO("Warning: smb1357_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	/* watchdog timer enable*/
	ret = smb1357_read(smb1357_dev, WATCHDOG_REG);
	if (ret < 0)
		goto out;
	//CHR_INFO("WATCHDOG_REG = 0x%02x\n", ret);
	ret |= BIT(0);
	ret = smb1357_write(smb1357_dev, WATCHDOG_REG, ret);
	if (ret < 0)
		goto out;

out:
	mutex_unlock(&smb1357_dev->lock);
	return ret;
}

int smb1357_AC_in_current(void)
{
	int ret = 0, gpio_status=0;;

	CHR_INFO("%s +++\n", __func__);

	if (!smb1357_dev) {
		CHR_INFO("Warning: smb1357_dev is null due to probe function has error\n");
		return 1;
	}

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0) {
		if (Read_PROJ_ID()==PROJ_ID_ZX550ML)
			goto check_cable_status;
		else
			goto out;
	}
	if ((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER)||(Read_PROJ_ID()==PROJ_ID_ZX550ML)) {
		if (dcp_mode==1) {
			//NEED add GPIO control in QC_PMIC_TO_CHARGER
			gpio_direction_output(smb1357_dev->gpio_usb_swith, 1);
			msleep(1000);
			/* check again if cable still in for quick plug in/out action */
			if (!gpio_get_value(smb1357_dev->pdata->inok_gpio)||usb_detect_flag) {
				/* disable missing poller: set 12h[5]=0 */
				smb1357_set_writable(smb1357_dev, true);
				ret = smb1357_read(smb1357_dev, CFG_OTG_I_LIMIT_REG);
				if (ret < 0)
					goto out;
				ret &= ~(BIT(5));
				ret = smb1357_write(smb1357_dev, CFG_OTG_I_LIMIT_REG, ret);
				if (ret < 0)
					goto out;
				gpio57_flag = 1;
				gpio_direction_output(smb1357_dev->gpio_chrg_signal, 1);
				msleep(1000);
				gpio_direction_output(smb1357_dev->gpio_chrg_signal, 0);
				CHR_INFO("gpio 57 done, detect charging type\n");
				gpio57_flag = 2;
				dcp_count = 2;
				schedule_delayed_work(&smb1357_dev->query_DCPmode_wrkr, 5*HZ);
			} else {
				CHR_INFO("inok high, so ignore detecting charging type action\n");
				if (Read_PROJ_ID()==PROJ_ID_ZX550ML)
					goto check_cable_status;
			}
		} else {
			CHR_INFO("dcp_mode=%d, set AC in current without gpio 57 action\n", dcp_mode);
			dcp_count = 1;
			//smb1357 detect HVDCP need about 3s , use a work queue to detect it
			schedule_delayed_work(&smb1357_dev->query_DCPmode_wrkr, 0.5*HZ);
		}
	} else {
		if(hvdcp_mode) {
			CHR_INFO("%s set HVDCP_IN current\n", __func__);
			 if ((boot_mode==1)&&(!early_suspend_flag))
				set_QC_inputI_limit(5);
			else
				set_QC_inputI_limit(1);
		}
		else if ((dcp_mode==2)||(Read_PROJ_ID()==PROJ_ID_ZE551ML_CKD)) {
			CHR_INFO("%s set other DCP_IN current\n", __func__);
			set_QC_inputI_limit(4);
		}else {
			CHR_INFO("%s set DCP_IN current\n", __func__);
			set_QC_inputI_limit(0);
		}
	}
out:
	return ret;
check_cable_status:
	msleep(1000);
	gpio_status = gpio_get_value(smb1357_dev->pdata->inok_gpio);
	CHR_INFO("check inok status = %d\n", gpio_status);
	ret = smb1357_read(smb1357_dev, RESULT_STATUS_REG);
	CHR_INFO("check RESULT_STATUS_REG 0x47=0x%02x\n", ret);
	if  ((!(ret&BIT(1)))&&(gpio_status)) {
		CHR_INFO("AC_IN but no usb/cable in!\n");
		gpio_direction_output(smb1357_dev->gpio_chrg_signal, 0);
		gpio_direction_output(smb1357_dev->gpio_usb_swith, 0);
		/* clear flag again */
		dcp_mode=0;
		hvdcp_mode=0;
		first_out_flag=0;
		usb_detect_flag = 0;
		gpio57_flag = 0;
		chr_suspend_flag = 0;
		setSMB1357Charger(CABLE_OUT);
	}
	return ret;
}

int smb1357_charging_toggle(bool on)
{
	int ret = 0;

	CHR_INFO("%s +++ charging toggle: %s \n", __func__, on ? "ON" : "OFF");

	if (disable_chrg) {
		on = false;
		CHR_INFO("disable charging flag is set, charging toggle: %s \n", on ? "ON" : "OFF");
	}

	if (!smb1357_dev) {
		CHR_INFO("Warning: smb1357_dev is null due to probe function has error\n");
		return 1;
	}

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	/* Config CFG_PIN register */

	ret = smb1357_read(smb1357_dev, CFG_CHARGE_EN_REG);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	if (on) {
		ret |= BIT(6);
	} else {
		ret &= ~(BIT(6));	
	}

	ret = smb1357_write(smb1357_dev, CFG_CHARGE_EN_REG, ret);
	if (ret < 0)
		goto out;
	
out:
	mutex_unlock(&smb1357_dev->lock);
	return ret;
}

/*
 * To set fast charge current
 * support 700/1400/2300/2500/2800/3000 mA for ZE550ML/ZE550ML
 * support 2000 mA for ZX550ML
 */
int smb1357_set_Ichg(int i)
{
	int ret=0;

	CHR_INFO("%s +++\n", __func__);

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	if(i==2800){
		/*config fast-charge current=2800mA*/
		CHR_INFO("set Ichg 2800mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(1)|BIT(2)|BIT(4));
		ret &= ~(BIT(0)|BIT(3));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
	if(i==2500){
		/*config fast-charge current=2500mA*/
		CHR_INFO("set Ichg 2500mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(1)|BIT(2)|BIT(3)|BIT(4));
		ret &= ~(BIT(0));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
	if(i==2300){
		/*config fast-charge current=2300mA*/
		CHR_INFO("set Ichg 2300mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(2)|BIT(3)|BIT(4));
		ret &= ~(BIT(0)|BIT(1));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
	if(i==2000){
		/*config fast-charge current=2000mA*/
		CHR_INFO("set Ichg 2000mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(0)|BIT(3)|BIT(4));
		ret &= ~(BIT(1)|BIT(2));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
	if(i==1400){
		/*config fast-charge current=1400mA*/
		CHR_INFO("set Ichg 1400mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(1)|BIT(2)|BIT(3));
		ret &= ~(BIT(0)|BIT(4));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
	if(i==1000){
		/*config fast-charge current=1000mA*/
		CHR_INFO("set Ichg 1000mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(0)|BIT(1)|BIT(3));
		ret &= ~(BIT(2)|BIT(4));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
	if(i==700){
		/*config fast-charge current=700mA*/
		CHR_INFO("set Ichg 700mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(3));
		ret &= ~(BIT(0)|BIT(1)|BIT(2)|BIT(4));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
	if(i==400){
		/*config fast-charge current=400mA*/
		CHR_INFO("set Ichg 400mA!\n");
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		if (ret < 0)
			goto out;
		ret |= (BIT(0));
		ret &= ~(BIT(1)|BIT(2)|BIT(3)|BIT(4));
		ret = smb1357_write(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG, ret);
		if (ret < 0)
			goto out;
	}
out:
	mutex_unlock(&smb1357_dev->lock);
	return ret;	
}
/*
 * To set recharge voltage
 * true meas recharge voltage=Vflt-50mV
 * false meas recharge voltage=Vflt-200mV
 */
int smb1357_set_recharge(bool high)
{
	int ret=0;

	CHR_INFO("%s : %s\n", __func__, high ? "50mV" : "200mV");

	mutex_lock(&smb1357_dev->lock);

	ret = smb1357_set_writable(smb1357_dev, true);
	if (ret < 0)
		goto out;

	if(high){
		/*config recharge voltage=Vflt-50mV*/
		ret = smb1357_read(smb1357_dev, CFG_RECHARGE_REG);
		if (ret < 0)
			goto out;
		ret = smb1357_write(smb1357_dev, CFG_RECHARGE_REG,ret&0xfb);
		if (ret < 0)
			goto out;
	}else{
	   /*config recharge voltage=Vflt-200mV*/
		ret = smb1357_read(smb1357_dev, CFG_RECHARGE_REG);
		if (ret < 0)
			goto out;
		ret = smb1357_write(smb1357_dev, CFG_RECHARGE_REG,ret|BIT(2));
		if (ret < 0)
			goto out;
	}

out:
	mutex_unlock(&smb1357_dev->lock);
	return ret; 
}

int smb1357_uv_result(void)
{
	int ret = 0;

	ret = smb1357_read(smb1357_dev, IRQ_E_REG);
	CHR_INFO("IRQ_E_REG 0x54=0x%02x\n", ret);
	if  ((ret >= 0)&&(ret&0x1)) {
		CHR_INFO("under voltage happen!\n");
		return 1;
	}else {
		return 0;
	}
}
EXPORT_SYMBOL(smb1357_uv_result);

int smb1357_power_ok(void)
{
	int ret = 0;

	ret = smb1357_read(smb1357_dev, IRQ_F_REG);
	CHR_INFO("IRQ_F_REG 0x55=0x%02x\n", ret);
	if  ((ret >= 0)&&!(ret&0x01)) {
		CHR_INFO("power ok happen!\n");
		return 1;
	}else {
		return 0;
	}
}
EXPORT_SYMBOL(smb1357_power_ok);

int smb1357_chr_suspend(void)
{
	int ret = 0;

	ret = smb1357_read(smb1357_dev, RESULT_STATUS_REG);
	CHR_INFO("RESULT_STATUS_REG 0x47=0x%02x\n", ret);
	if  ((ret >= 0)&&(ret&0x8)) {
		CHR_INFO("charger suspend happen!\n");
		return 1;
	}else {
		return 0;
	}
}
EXPORT_SYMBOL(smb1357_chr_suspend);

int smb1357_set_volt_in(int volt)
{
	int ret = 0;

	CHR_INFO("%s +++ set adaptor voltage: %s \n", __func__, volt ? "5V" : "9V");
	if (volt==0) {
		ret = smb1357_read(smb1357_dev, HVDCP_STATUS_REG);
		if (!(ret&0x10)) {
			/*set 9v*/
			smb1357_charging_toggle(false);
			smb1357_set_writable(smb1357_dev, true);
			ret &= ~(BIT(5));
			ret |= (BIT(4));
			smb1357_write(smb1357_dev, HVDCP_STATUS_REG, ret);
			smb1357_charging_toggle(true);
		}
	} else {
		ret = smb1357_read(smb1357_dev, HVDCP_STATUS_REG);
		if (ret&0x10) {
			/*set 5v*/
			smb1357_charging_toggle(false);
			smb1357_set_writable(smb1357_dev, true);
			ret &= ~(BIT(4)|BIT(5));
			smb1357_write(smb1357_dev, HVDCP_STATUS_REG, ret);
			smb1357_charging_toggle(true);
		}
	}
	return 0;
}
EXPORT_SYMBOL(smb1357_set_volt_in);

static void smb1357_inok_debounce_queue(struct work_struct *work)
{
	uint8_t data=0;

	CHR_INFO("%s +++\n", __func__);

	first_out_flag=0;
	if (!((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER))&&(Read_PROJ_ID()!=PROJ_ID_ZX550ML) ) {
		cancel_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
		//flush_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
		dcp_mode=0;
		hvdcp_mode=0;
		usb_detect_flag = 0;
		pmic_handle_low_supply();
		if (wake_lock_active(&wakelock_cable)) {
			CHR_INFO("%s: wake unlock\n", __func__);
			wake_lock_timeout(&wakelock_cable_t, 3*HZ);
			wake_unlock(&wakelock_cable);
		}else
			wake_lock_timeout(&wakelock_cable_t, 3*HZ);
	} else {
		if (gpio57_flag==1) {
			/*WA for cable out with gpio 57 high at the same time*/
			CHR_INFO("gpio57_flag = 1, check gpio 57 status and pmic vbus\n");
			msleep(1000);
			intel_scu_ipc_ioread8(SCHGRIRQ1_REG, &data);
			CHR_INFO("PMIC REG SVBUSDET(104F)=0x%02x\n", data);
			data &= 0x01;
			if (data) {
				CHR_INFO("inok high due to gpio 57 high and vbus connect, ignore\n");
				if (!gpio_get_value(smb1357_dev->pdata->inok_gpio)) {
					CHR_INFO("inok low, init charging settings\n");
					smb1357_set_fast_charge();
					smb1357_charging_toggle(false); // for not charging when almost full battery workaround
					smb1357_set_voltage(false);
					smb1357_charging_toggle(true); // for not charging when almost full battery workaround
					smb1357_watchdog_timer_enable();
					smb1357_control_JEITA(true);
					gpio57_flag = 0;
				} else {
					CHR_INFO("inok high again, cable out\n");
					if (!not_ready_flag) {
						cancel_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
						//flush_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
					}
					dcp_mode=0;
					hvdcp_mode=0;
					usb_detect_flag = 0;
					gpio57_flag = 0;
					pmic_handle_low_supply();
					if (wake_lock_active(&wakelock_cable)) {
						CHR_INFO("%s: wake unlock\n", __func__);
						wake_lock_timeout(&wakelock_cable_t, 3*HZ);
						wake_unlock(&wakelock_cable);
					} else
						wake_lock_timeout(&wakelock_cable_t, 3*HZ);
				}
			} else {
				CHR_INFO("inok high due to cable out\n");
				if (!not_ready_flag) {
					cancel_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
					//flush_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
				}
				dcp_mode=0;
				hvdcp_mode=0;
				usb_detect_flag = 0;
				gpio57_flag = 0;
				pmic_handle_low_supply();
				if (wake_lock_active(&wakelock_cable)) {
					CHR_INFO("%s: wake unlock\n", __func__);
					wake_lock_timeout(&wakelock_cable_t, 3*HZ);
					wake_unlock(&wakelock_cable);
				} else
					wake_lock_timeout(&wakelock_cable_t, 3*HZ);
			}
		} else {
			if (!not_ready_flag) {
				cancel_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
				//flush_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
			}
			dcp_mode=0;
			hvdcp_mode=0;
			usb_detect_flag = 0;
			gpio57_flag = 0;
			chr_suspend_flag = 0;
			pmic_handle_low_supply();
			if (wake_lock_active(&wakelock_cable)) {
				CHR_INFO("%s: wake unlock\n", __func__);
				wake_lock_timeout(&wakelock_cable_t, 3*HZ);
				wake_unlock(&wakelock_cable);
			} else
				wake_lock_timeout(&wakelock_cable_t, 3*HZ);
		}
	}
}

static irqreturn_t smb1357_inok_interrupt(int irq, void *data)
{
	struct smb1357_charger *smb = data;
	irqreturn_t ret = IRQ_NONE;

	CHR_INFO("%s +++\n", __func__);
	pm_runtime_get_sync(&smb->client->dev);
	if (gpio_get_value(smb->pdata->inok_gpio)) {
		CHR_INFO( "%s: >>> INOK pin (HIGH:power plug out)<<<\n", __func__);
		first_out_flag=1;
		schedule_delayed_work(&inok_work, 0.5*HZ);
	}
	else {
		CHR_INFO("%s: >>> INOK pin (LOW:power plug in)<<<\n", __func__);
		if (!wake_lock_active(&wakelock_cable)) {
			CHR_INFO("%s: wake lock\n", __func__);
			wake_lock(&wakelock_cable);
		}
		if(!((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER))&&(Read_PROJ_ID()!=PROJ_ID_ZX550ML) ) {
			CHR_INFO("%s: first_out_flag=%d\n", __func__, first_out_flag);
			if (first_out_flag)
				cancel_delayed_work(&inok_work);
			dcp_count = 1;
			//smb1357 detect DCP need about 5s , use a work queue to detect it
			cancel_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
			flush_delayed_work(&smb1357_dev->query_DCPmode_wrkr);
			schedule_delayed_work(&smb1357_dev->query_DCPmode_wrkr, 5*HZ);
			/*set SDP first to show battery icon/LED after 0.5ms*/
			msleep(500);
			CHR_INFO("%s: set CHRG_SDP first !!!\n", __func__);
			setSMB1357Charger(USB_IN);
			first_sdp_mode = 1;
		} else if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
			CHR_INFO("%s: first_out_flag=%d\n", __func__, first_out_flag);
			if (first_out_flag)
				cancel_delayed_work(&inok_work);
			if (chr_suspend_flag==1) {
				CHR_INFO("set usb_in current after charger suspend and reset\n");
				if (!not_ready_flag) {
					dcp_count = 1;
					schedule_delayed_work(&smb1357_dev->query_DCPmode_wrkr, 5*HZ);
				}
				chr_suspend_flag = 2;
			}
			if (gpio57_flag==2) {
				smb1357_set_fast_charge();
				smb1357_charging_toggle(false); // for not charging when almost full battery workaround
				smb1357_set_voltage(false);
				smb1357_charging_toggle(true); // for not charging when almost full battery workaround
				smb1357_watchdog_timer_enable();
				smb1357_control_JEITA(true);
				gpio57_flag = 0;
			}
		}
	}

	pm_runtime_put_sync(&smb->client->dev);
	return ret;
}

static int smb1357_inok_gpio_init(struct smb1357_charger *smb)
{
	const struct smb1357_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->inok_gpio);

	ret = gpio_request_one(pdata->inok_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0) {
		CHR_INFO("smb1357: request INOK gpio fail!\n");
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL, smb1357_inok_interrupt,
					IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING | IRQF_ONESHOT,
					smb->client->name,
					smb);
	if (ret < 0) {
		CHR_INFO("smb1357: config INOK gpio as IRQ fail!\n");
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(pdata->inok_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static int smb1357_mains_get_property(struct power_supply *psy,
				     enum power_supply_property prop,
				     union power_supply_propval *val)
{
	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		if ((g_cable_status==AC_IN)&&(dcp_mode!=3))
			val->intval = 1;
		else
			val->intval = 0;
		return 0;
	}
	return -EINVAL;
}

static enum power_supply_property smb1357_mains_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb1357_usb_get_property(struct power_supply *psy,
				   enum power_supply_property prop,
				   union power_supply_propval *val)
{
	if (prop == POWER_SUPPLY_PROP_ONLINE) {
		if ((g_cable_status==USB_IN)||(dcp_mode==3))
			val->intval = 1;
		else
			val->intval = 0;
		return 0;
	}
	return -EINVAL;
}

bool smb1357_has_charger_error(void)
{
	int ret;

	if (!smb1357_dev)
		return -EINVAL;

	ret = smb1357_read(smb1357_dev, 0x56);

	if (ret & STAT_CHG_ERROR){
		CHR_ERR("%s !!!!!!!",__func__);
		return true;
	}

	return false;
}

int smb1357_get_charging_status(void)
{
	int ret, status;

	if (!smb1357_dev)
		return -EINVAL;

	ret = smb1357_read(smb1357_dev, STAT_CHARGE_REG);
	if (ret < 0)
		return ret;

	if ((ret & STAT_HOLDOFF)) {
		/* set to NOT CHARGING upon charger error
		 * or charging has stopped.
		 */
		status = POWER_SUPPLY_STATUS_NOT_CHARGING;
	} else {
		if ((ret & STAT_CHG_MASK) >> STAT_CHG_SHIFT) {
			/* set to charging if battery is in pre-charge,
			 * fast charge or taper charging mode.
			 */
			status = POWER_SUPPLY_STATUS_CHARGING;
		} else if (ret & STAT_CHG_TERM) {
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

static enum power_supply_property smb1357_usb_properties[] = {
	POWER_SUPPLY_PROP_ONLINE,
};


static int smb1357_debugfs_show(struct seq_file *s, void *data)
{
	struct smb1357_charger *smb = s->private;
	int ret;
	u8 reg;

	seq_printf(s, "All registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");
	for (reg = 0; reg <= 0x1f; reg++) {
		ret = smb1357_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}
	ret = smb1357_read(smb, CMD_I2C_REG);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_I2C_REG, BYTETOBINARY(ret));
	ret = smb1357_read(smb, CMD_IL_REG);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_IL_REG, BYTETOBINARY(ret));
	ret = smb1357_read(smb, CMD_CHG_REG);
	seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_CHG_REG, BYTETOBINARY(ret));
	for (reg = 0x46; reg <= 0x4f; reg++) {
		ret = smb1357_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}
	for (reg = 0x50; reg <= 0x56; reg++) {
		ret = smb1357_read(smb, reg);
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}

static int smb1357_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb1357_debugfs_show, inode->i_private);
}

static const struct file_operations smb1357_debugfs_fops = {
	.open	= smb1357_debugfs_open,
	.read		= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

/* This function is only for debug */
static void query_PowerState_worker(struct work_struct *work)
{
	int ret=0;
	u8 reg;

	CHR_INFO("%s +++ in every 30s \n",__func__);
	if(debug_flag) {
		CHR_INFO("All registers:\n");
		CHR_INFO("==================\n");
		CHR_INFO("#Addr\t#Value\n");
		for (reg = 0; reg <= 0x1f; reg++) {
			ret = smb1357_read(smb1357_dev, reg);
			CHR_INFO("0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		ret = smb1357_read(smb1357_dev, CMD_I2C_REG);
		CHR_INFO("0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_I2C_REG, BYTETOBINARY(ret));
		ret = smb1357_read(smb1357_dev, CMD_IL_REG);
		CHR_INFO("0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_IL_REG, BYTETOBINARY(ret));
		ret = smb1357_read(smb1357_dev, CMD_CHG_REG);
		CHR_INFO("0x%02x:\t" BYTETOBINARYPATTERN "\n", CMD_CHG_REG, BYTETOBINARY(ret));
		for (reg = 0x46; reg <= 0x4f; reg++) {
			ret = smb1357_read(smb1357_dev, reg);
			CHR_INFO("0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		for (reg = 0x50; reg <= 0x56; reg++) {
			ret = smb1357_read(smb1357_dev, reg);
			CHR_INFO("0x%02x:\t" BYTETOBINARYPATTERN "\n", reg, BYTETOBINARY(ret));
		}
		CHR_INFO("==================\n");
	}else {
		ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
		CHR_INFO("the CFG_INPUT_CURRENT_LIMIT_REG 0Ch is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, HVDCP_STATUS_REG);
		CHR_INFO("the HVDCP_STATUS_REG 0Eh is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, CFG_TEMP_BEHAVIOR_REG);
		CHR_INFO("the CFG_TEMP_BEHAVIOR_REG 1Ah is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, CFG_HOT_LIMIT);
		CHR_INFO("the CFG_HOT_LIMIT 1Bh is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, CFG_PRE_FAST_CHARGE_CURRENT_REG);
		CHR_INFO("the CFG_PRE_FAST_CHARGE_CURRENT_REG 1Ch is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, RESULT_AICL_REG);
		CHR_INFO("the RESULT_AICL_REG 46h is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, RESULT_STATUS_REG);
		CHR_INFO("the RESULT_STATUS_REG 47h is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, STAT_PWRSRC_REG);
		CHR_INFO("the STAT_PWRSRC_REG 4Bh is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, STAT_HVDCP_REG);
		CHR_INFO("the STAT_HVDCP_REG 4Dh is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, STAT_CHARGE_REG);
		CHR_INFO("the STAT_CHARGE_REG 4Ah is 0x%02x\n", ret);
		ret = smb1357_read(smb1357_dev, IRQ_E_REG);
		CHR_INFO("the IRQ_E_REG 54h is 0x%02x\n", ret);
	}

	if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
		if (smb1357_chr_suspend()) {
			CHR_INFO("charger suspend, reset charger\n");
			chr_suspend_flag = 1;
			gpio57_flag = 1;
			gpio_direction_output(smb1357_dev->gpio_chrg_signal, 1);
			msleep(3000);
			gpio_direction_output(smb1357_dev->gpio_chrg_signal, 0);
		} else {
			CHR_INFO("charger not suspend\n");
		}
	}
	schedule_delayed_work(&smb1357_dev->power_state_wrkr, 60*HZ);
}

static void query_DCPmode_worker(struct work_struct *work)
{
	int ret=0, i;

	CHR_INFO("%s +++\n",__func__);

	if((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER)||(Read_PROJ_ID()==PROJ_ID_ZX550ML) ) {
		ret = smb1357_read(smb1357_dev, STAT_HVDCP_REG);
		CHR_INFO("0x4d=0x%02x\n", ret);
		for(i=0;i<5;i++) {
			if(ret < 0) {
				CHR_INFO("read 0x4d fail, retry %d times\n", 5-i);
				msleep(50);
				ret = smb1357_read(smb1357_dev, CFG_INPUT_CURRENT_LIMIT_REG);
				CHR_INFO("0x4d=0x%02x\n", ret);
			}else {
				break;
			}
		}
		if ((first_out_flag==1)&&(dcp_count!=1)) {
			CHR_INFO("detect inok pin high then low in 500ms, detect charging port!\n");
			if(dcp_mode!=2) {
				dcp_mode = 2;
				set_QC_inputI_limit(4);
			}
		}
		if ((dcp_mode!=3)&&(ret&BIT(4))) {
			CHR_INFO("detect HVDCP!\n");
			hvdcp_mode = 1;
			asus_update_all();
		}
		if (!chr_suspend_flag) {
			if (hvdcp_mode) {
				set_QC_inputI_limit(1);
				if (boot_mode==1) {
					smb1357_charging_toggle(false);
					if (early_suspend_flag)
						smb1357_set_volt_in(0);
					else
						smb1357_set_volt_in(1);
					smb1357_charging_toggle(true);
				}
			} else if ((dcp_mode==3)||((dcp_count!=1)&&(dcp_mode!=2)))  {
				set_QC_inputI_limit(4);
			}
		} else {
			set_QC_inputI_limit(6);
		}
	}else {
		if(gpio_get_value(smb1357_dev->pdata->inok_gpio)) {
			CHR_INFO("detect NO CABLE!\n");
			setSMB1357Charger(CABLE_OUT);
			return;
		}

		ret = smb1357_read(smb1357_dev, STAT_PWRSRC_REG);
		if (first_out_flag==1) {
			CHR_INFO("detect inok pin high then low in 500ms, set other charging port!\n");
			if(dcp_mode!=2) {
				dcp_mode = 2;
				CHR_INFO("%s CHRG_DCP !!!\n", __func__);
				setSMB1357Charger(AC_IN);
			}
		} else if (ret&BIT(6)) {
			CHR_INFO("detect DCP!\n");
			if(dcp_mode!=1) {
				dcp_mode = 1;
				ret = smb1357_read(smb1357_dev, STAT_HVDCP_REG);
				if(ret&BIT(4)){
					CHR_INFO("detect HVDCP!\n");
					if(hvdcp_mode!=1) {
						if ((boot_mode==1)||(boot_mode==4)) {
							hvdcp_mode = 1;
							//asus_update_all();
						}
						if (disable_chrg) {
							/* set 5V */
							CHR_INFO("set 5V!\n");
							ret = smb1357_set_writable(smb1357_dev, true);
							ret = smb1357_read(smb1357_dev, HVDCP_STATUS_REG);
							CHR_INFO("read 0x%02x from 0x0E\n", ret);
							ret &= ~(BIT(4)|BIT(5));
							CHR_INFO("write 0x%0xx to 0x0E\n", ret);
							smb1357_write(smb1357_dev, HVDCP_STATUS_REG, ret);
						}
						CHR_INFO("%s CHRG_DCP(HVDCP) !!!\n", __func__);
					}
				}else {
					CHR_INFO("%s CHRG_DCP !!!\n", __func__);
				}
				setSMB1357Charger(AC_IN);
			}
		} else if (ret&BIT(5)) {
			CHR_INFO("detect other charging port!\n");
			if(dcp_mode!=2) {
				dcp_mode = 2;
				CHR_INFO("%s Other Charging port !!!\n", __func__);
				setSMB1357Charger(AC_IN);
			}
		} else if (dcp_mode==3) {
			CHR_INFO("detect CDP port!\n");
			setSMB1357Charger(AC_IN);
		} else {
			CHR_INFO("detect SDP, no action!\n");
		}
	}
	CHR_INFO("dcp_count=%d, hvdcp_mode=%d, dcp_mode=%d, chr_suspend_flag=%d\n", dcp_count, hvdcp_mode, dcp_mode, chr_suspend_flag);
	if (dcp_count>0) {
		if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
			if (hvdcp_mode||(dcp_mode==3)||chr_suspend_flag) {
				CHR_INFO("no need to detect again\n");
				dcp_count = 0;
			} else {
				dcp_count--;
				CHR_INFO("need to detect again\n");
				schedule_delayed_work(&smb1357_dev->query_DCPmode_wrkr, 5*HZ);
			}
		} else {
			dcp_count--;
			schedule_delayed_work(&smb1357_dev->query_DCPmode_wrkr, 0.5*HZ);
		}
	}
}

static int cable_status_notify2(struct notifier_block *self, unsigned long action, void *dev)
{
	int ret=0, gpio_status=0;
	uint8_t data=0;

	CHR_INFO("%s: action = %ld\n", __func__, action);

	if (((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER)||(Read_PROJ_ID()==PROJ_ID_ZX550ML))&&(usb_detect_flag)&&(!dcp_mode)) {
		intel_scu_ipc_ioread8(SCHGRIRQ1_REG, &data);
		CHR_INFO("PMIC REG SVBUSDET(104F)=0x%02x\n", data);
		data &= 0x01;
		if((!data)&&(action!=POWER_SUPPLY_CHARGER_TYPE_NONE)) {
			CHR_INFO("action = %ld but no VBUS detected\n", action);
			CHR_INFO("set CHRG_UNKNOWN !!!\n");
			if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
				gpio_direction_output(smb1357_dev->gpio_chrg_signal, 0);
			}else {
				gpio_direction_output(smb1357_dev->gpio_chrg_signal, 1);
			}
			gpio_direction_output(smb1357_dev->gpio_usb_swith, 0);
			/* clear flag again */
			dcp_mode=0;
			hvdcp_mode=0;
			first_out_flag=0;
			usb_detect_flag = 0;
			gpio57_flag = 0;
			chr_suspend_flag = 0;
			setSMB1357Charger(CABLE_OUT);
			return NOTIFY_OK;
		} else if((data)&&(action==POWER_SUPPLY_CHARGER_TYPE_NONE)) {
			CHR_INFO("action = 0, cable out due to usb detection so ignore\n");
			return NOTIFY_OK;
		}
	}

	switch (action) {
		case POWER_SUPPLY_CHARGER_TYPE_USB_SDP:
			if((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER)||(Read_PROJ_ID()==PROJ_ID_ZX550ML) ) {
				//gpio_direction_output(smb1357_dev->gpio_chrg_signal, 0);
				CHR_INFO("%s CHRG_SDP !!!\n", __func__);
				setSMB1357Charger(USB_IN);
				if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
					gpio_status = gpio_get_value(smb1357_dev->pdata->inok_gpio);
					CHR_INFO("check inok status = %d\n", gpio_status);
					ret = smb1357_read(smb1357_dev, RESULT_STATUS_REG);
					CHR_INFO("check RESULT_STATUS_REG 0x47=0x%02x\n", ret);
					if  ((!(ret&BIT(1)))&&(gpio_status)) {
						CHR_INFO("SDP but no usb/cable in!\n");
						gpio_direction_output(smb1357_dev->gpio_chrg_signal, 0);
						gpio_direction_output(smb1357_dev->gpio_usb_swith, 0);
						/* clear flag again */
						dcp_mode=0;
						hvdcp_mode=0;
						first_out_flag=0;
						usb_detect_flag = 0;
						gpio57_flag = 0;
						chr_suspend_flag = 0;
						setSMB1357Charger(CABLE_OUT);
						return NOTIFY_OK;
					} else if (!usb_detect_flag) {
						//WA for USB detect wrong issue
						CHR_INFO("detect SDP first so set gpio 51 then detect again \n");
						usb_detect_flag = 1;
						gpio_direction_output(51, 1);
						msleep(2000);
						gpio_direction_output(51, 0);
					}
				}
			}
		break;

		case POWER_SUPPLY_CHARGER_TYPE_USB_CDP:
			CHR_INFO("%s CHRG_CDP !!!\n", __func__);
			dcp_mode = 3;
			if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
				ret = smb1357_read(smb1357_dev, CMD_IL_REG);
				CHR_INFO("CDP mode so check 41h = 0x%02x, then set bit [0][2] to 1\n", ret);
				smb1357_set_writable(smb1357_dev, true);
				ret |= (BIT(0)|BIT(2));
				smb1357_write(smb1357_dev, CMD_IL_REG, ret);
			}
			setSMB1357Charger(AC_IN);
		break;

		case POWER_SUPPLY_CHARGER_TYPE_USB_DCP:
			if (dcp_mode!=1) {
				dcp_mode = 1;
				CHR_INFO("%s CHRG_DCP !!!\n", __func__);
				setSMB1357Charger(AC_IN);
			} else {
				CHR_INFO("%s already CHRG_DCP !!!\n", __func__);
			}
		break;

		case POWER_SUPPLY_CHARGER_TYPE_AC:
			CHR_INFO("%s CHRG_ACA !!!\n", __func__);
			if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
				dcp_mode = 3;
				ret = smb1357_read(smb1357_dev, CMD_IL_REG);
				CHR_INFO("ACA mode so check 41h = 0x%02x, then set bit [0][2] to 1\n", ret);
				smb1357_set_writable(smb1357_dev, true);
				ret |= (BIT(0)|BIT(2));
				smb1357_write(smb1357_dev, CMD_IL_REG, ret);
			}
			setSMB1357Charger(AC_IN);
		break;

		case POWER_SUPPLY_CHARGER_TYPE_SE1:
			CHR_INFO("%s CHRG_SE1 !!!\n", __func__);
			if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
				dcp_mode = 3;
				ret = smb1357_read(smb1357_dev, CMD_IL_REG);
				CHR_INFO("SE1 mode so check 41h = 0x%02x, then set bit [0][2] to 1\n", ret);
				smb1357_set_writable(smb1357_dev, true);
				ret |= (BIT(0)|BIT(2));
				smb1357_write(smb1357_dev, CMD_IL_REG, ret);
			}
			setSMB1357Charger(AC_IN);
		break;

		case POWER_SUPPLY_CHARGER_TYPE_MHL:
			CHR_INFO("%s CHRG_MHL !!!\n", __func__);
			if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
				dcp_mode = 3;
				ret = smb1357_read(smb1357_dev, CMD_IL_REG);
				CHR_INFO("MHL mode so check 41h = 0x%02x, then set bit [0][2] to 1\n", ret);
				smb1357_set_writable(smb1357_dev, true);
				ret |= (BIT(0)|BIT(2));
				smb1357_write(smb1357_dev, CMD_IL_REG, ret);
			}
			setSMB1357Charger(AC_IN);
		break;

		case POWER_SUPPLY_CHARGER_TYPE_NONE:
		default:
			CHR_INFO("%s CHRG_UNKNOWN !!!\n", __func__);
			if((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER)||(Read_PROJ_ID()==PROJ_ID_ZX550ML) ) {
				if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
					gpio_direction_output(smb1357_dev->gpio_chrg_signal, 0);
				}else {
					gpio_direction_output(smb1357_dev->gpio_chrg_signal, 1);
				}
				gpio_direction_output(smb1357_dev->gpio_usb_swith, 0);
				/* clear flag again */
				dcp_mode=0;
				hvdcp_mode=0;
				first_out_flag=0;
				usb_detect_flag = 0;
				gpio57_flag = 0;
				chr_suspend_flag = 0;
			}else {
				if(!gpio_get_value(smb1357_dev->pdata->inok_gpio)) {
					CHR_INFO("inok pin low but detect no charging, wait 500ms\n");
					msleep(500);
					if(!gpio_get_value(smb1357_dev->pdata->inok_gpio)) {
						CHR_INFO("inok pin low but detect no charging, ignore\n");
						break;
					}
				}
			}
			setSMB1357Charger(CABLE_OUT);
		break;
	}
	return NOTIFY_OK;
}

static struct notifier_block cable_status_notifier2 = {
	.notifier_call = cable_status_notify2,
};

extern int cable_status_register_client(struct notifier_block *nb);
extern int cable_status_unregister_client(struct notifier_block *nb);

#ifdef CONFIG_PROC_FS
ssize_t smb1357_debug_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kmalloc(100,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;
	len += sprintf(buff + len, "%d\n", debug_flag);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
	kfree(buff);
	return ret;
}
ssize_t smb1357_debug_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	if (buffer[0] == '1') {
		/* turn on all register dump on for debug */
		debug_flag = 1;
	}
	else if (buffer[0] == '0') {
		/* turn off all register dump off for  */
		debug_flag = 0;
	}
	return count;
}
int init_smb1357_debug(void)
{
	struct proc_dir_entry *entry=NULL;

	static struct file_operations smb1357_debug_fop = {
		.read = smb1357_debug_read,
		.write = smb1357_debug_write,
	};
	entry = proc_create("smb1357_debug", 0664, NULL, &smb1357_debug_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/smb1357_debug fail\n");
	}
	return 0;
}

ssize_t smb1357_disable_chrg_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kmalloc(100,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;
	len += sprintf(buff + len, "%d\n", disable_chrg);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
	kfree(buff);
	return ret;
}
ssize_t smb1357_disable_chrg_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	int ret = 0;

	if (buffer[0] == '1') {
		/* disable charging */
		disable_chrg = 1;
		if (hvdcp_mode) {
			/* set 5v */
			ret = smb1357_set_writable(smb1357_dev, true);
			ret = smb1357_read(smb1357_dev, HVDCP_STATUS_REG);
			ret &= ~(BIT(4)|BIT(5));
			smb1357_write(smb1357_dev, HVDCP_STATUS_REG, ret);
		}
		if (smb1357_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING) {
			smb1357_charging_toggle(false);
		}
	}
	else if (buffer[0] == '0') {
		/* enable charging */
		disable_chrg = 0;
		if (hvdcp_mode) {
			/* set 9v */
			ret = smb1357_set_writable(smb1357_dev, true);
			ret = smb1357_read(smb1357_dev, HVDCP_STATUS_REG);
			ret &= ~(BIT(5));
			ret |= (BIT(4));
			smb1357_write(smb1357_dev, HVDCP_STATUS_REG, ret);
		}
	}
	return count;
}
int init_smb1357_disable_chrg(void)
{
	struct proc_dir_entry *entry=NULL;

	static struct file_operations smb1357_disable_chrg_fop = {
		.read = smb1357_disable_chrg_read,
		.write = smb1357_disable_chrg_write,
	};
	entry = proc_create("smb1357_disable_chrg", 0664, NULL, &smb1357_disable_chrg_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/smb1357_disable_chrg fail\n");
	}
	return 0;
}
#endif

static void smb1357_set_incur_queue(struct work_struct *work)
{
	CHR_INFO("%s ++ with early_suspend_flag=%d\n", __func__, early_suspend_flag);
	if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
		if (early_suspend_flag) {
			smb1357_set_volt_in(0);
		} else {
			smb1357_set_volt_in(1);
		}
	} else {
		smb1357_AC_in_current();
	}
}

#ifdef CONFIG_HAS_EARLYSUSPEND
static void smb1357_early_suspend(struct early_suspend *h)
{
	CHR_INFO("%s ++\n", __func__);
	early_suspend_flag = 1;
	if ((hvdcp_mode)&&(boot_mode==1)) {
		if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
			cancel_delayed_work(&set_cur_work);
			schedule_delayed_work(&set_cur_work, 10*HZ);
		} else {
			cancel_delayed_work(&set_cur_work);
			schedule_delayed_work(&set_cur_work, 0.5*HZ);
		}
	}
}

static void smb1357_late_resume(struct early_suspend *h)
{
	CHR_INFO("%s ++\n", __func__);
	early_suspend_flag = 0;
	if ((hvdcp_mode)&&(boot_mode==1)) {
		if (Read_PROJ_ID()==PROJ_ID_ZX550ML) {
			cancel_delayed_work(&set_cur_work);
			schedule_delayed_work(&set_cur_work, 0*HZ);
		} else {
			cancel_delayed_work(&set_cur_work);
			schedule_delayed_work(&set_cur_work, 0.5*HZ);
		}
	}
}
static void smb1357_config_earlysuspend(struct smb1357_charger *smb)
{
	smb->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
	smb->es.suspend = smb1357_early_suspend;
	smb->es.resume = smb1357_late_resume;
	register_early_suspend(&smb->es);
}
#endif

static int smb1357_probe(struct i2c_client *client,
			const struct i2c_device_id *id)
{
	const struct smb1357_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb1357_charger *smb;
	int ret;

	CHR_INFO("%s +++\n", __func__);

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	mutex_init(&smb->lock);
	smb->client = client;
	smb->pdata = pdata;

	if (smb->pdata->use_mains) {
		smb->mains.name = "ac";
		smb->mains.type = POWER_SUPPLY_TYPE_MAINS;
		smb->mains.get_property = smb1357_mains_get_property;
		smb->mains.properties = smb1357_mains_properties;
		smb->mains.num_properties = ARRAY_SIZE(smb1357_mains_properties);
		smb->mains.supplied_to = smb1357_power_supplied_to;
		smb->mains.num_supplicants =
				ARRAY_SIZE(smb1357_power_supplied_to);
		ret = power_supply_register(dev, &smb->mains);
		if (ret < 0)
			return ret;
	}

	if (smb->pdata->use_usb) {
		smb->usb.name = "usb";
		smb->usb.type = POWER_SUPPLY_TYPE_USB;
		smb->usb.get_property = smb1357_usb_get_property;
		smb->usb.properties = smb1357_usb_properties;
		smb->usb.num_properties = ARRAY_SIZE(smb1357_usb_properties);
		smb->usb.supplied_to = smb1357_power_supplied_to;
		smb->usb.num_supplicants = ARRAY_SIZE(smb1357_power_supplied_to);
		ret = power_supply_register(dev, &smb->usb);
		if (ret < 0) {
			if (smb->pdata->use_mains)
				power_supply_unregister(&smb->mains);
			return ret;
		}
	}

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&smb->client->dev);
	pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);

	/* INOK pin configuration */
	if (pdata->inok_gpio >= 0) {
		ret = smb1357_inok_gpio_init(smb);
		if (ret < 0) {
			CHR_ERR("failed to initialize INOK gpio: %d\n", ret);
		}
	}
	smb->dentry = debugfs_create_file("smb1357-regs", S_IRUSR, NULL, smb,
					  &smb1357_debugfs_fops);

	cable_status_register_client(&cable_status_notifier2);
	wake_lock_init(&wakelock_cable, WAKE_LOCK_SUSPEND, "cable_wakelock");
	wake_lock_init(&wakelock_cable_t, WAKE_LOCK_SUSPEND, "cable_wakelock_timeout");

	if((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER)||(Read_PROJ_ID()==PROJ_ID_ZX550ML)) {
		CHR_INFO("set SOC QC control\n");
		smb->gpio_chrg_signal = 57;
		smb->gpio_usb_swith = 10;
		if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
			ret = gpio_request_one(smb->gpio_chrg_signal, GPIOF_OUT_INIT_LOW, smb->client->name);
		}else {
			ret = gpio_request_one(smb->gpio_chrg_signal, GPIOF_OUT_INIT_HIGH, smb->client->name);
		}
		if (ret < 0) {
			CHR_ERR("smb1357: request GPO57 fail!\n");
		}
		ret = gpio_request_one(smb->gpio_usb_swith, GPIOF_OUT_INIT_LOW, smb->client->name);
		if (ret < 0) {
			CHR_ERR("smb1357: request GPO10 fail!\n");
		}
		if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
			ret = gpio_request_one(51, GPIOF_OUT_INIT_LOW, smb->client->name);
			if (ret < 0)
				CHR_ERR("smb1357: request GPO51 fail!\n");
		}
	}else
		CHR_INFO("set Charger QC control\n");
	INIT_DELAYED_WORK(&inok_work, smb1357_inok_debounce_queue);
	INIT_DELAYED_WORK(&set_cur_work, smb1357_set_incur_queue);
	INIT_DELAYED_WORK(&smb->query_DCPmode_wrkr, query_DCPmode_worker);
	INIT_DELAYED_WORK(&smb->power_state_wrkr, query_PowerState_worker);
	smb1357_dev = smb;
	cable_status_notify2( NULL, query_cable_status(), dev);
	if((smb1357_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING)||(!gpio_get_value(smb->pdata->inok_gpio))) {
		if (!wake_lock_active(&wakelock_cable)) {
			CHR_INFO("%s: first status is charging, wake lock\n", __func__);
			wake_lock(&wakelock_cable);
		}
		if(!((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER))&&(Read_PROJ_ID()!=PROJ_ID_ZX550ML) ) {
			CHR_INFO("%s: first status is charging, detect charging type\n", __func__);
			dcp_count = 1;
			//smb1357 detect DCP need about 3s , use a work queue to detect it
			cancel_delayed_work(&smb->query_DCPmode_wrkr);
			flush_delayed_work(&smb->query_DCPmode_wrkr);
			schedule_delayed_work(&smb->query_DCPmode_wrkr, 0.5*HZ);
			//msleep(500);
			CHR_INFO("%s: set CHRG_SDP first !!!\n", __func__);
			setSMB1357Charger(USB_IN);
		}
	}
#ifdef CONFIG_HAS_EARLYSUSPEND
	smb1357_config_earlysuspend(smb);
#endif
#ifdef CONFIG_PROC_FS
	ret = init_smb1357_debug();
	if (ret) {
		BAT_DBG_E("Unable to create proc init_smb1357_debug\n");
	}
	ret = init_smb1357_disable_chrg();
	if (ret) {
		BAT_DBG_E("Unable to create proc init_smb1357_disable_chrg\n");
	}
#endif
	schedule_delayed_work(&smb->power_state_wrkr, 5*HZ);
	not_ready_flag = 0;
	CHR_INFO("%s ---\n", __func__);

	return 0;
}

static int smb1357_remove(struct i2c_client *client)
{
	struct smb1357_charger *smb = i2c_get_clientdata(client);

	CHR_INFO("%s +++\n", __func__);
	cable_status_unregister_client(&cable_status_notifier2);
	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);
	pm_runtime_get_noresume(&smb->client->dev);
	if((Read_HW_ID()==HW_ID_SR1)||(Read_HW_ID()==HW_ID_ER)||(Read_PROJ_ID()==PROJ_ID_ZX550ML)) {
		gpio_free(smb->gpio_chrg_signal);
		gpio_free(smb->gpio_usb_swith);
		if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
			gpio_free(51);
		}
	}
	return 0;
}

void smb1357_shutdown(struct i2c_client *client)
{

	CHR_INFO("%s +++\n", __func__);

        if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
             /*chargerIC control JEITA*/
             smb1357_control_JEITA(false);
        }
	/* Disable OTG during shutdown */
	//otg(0);
	cancel_delayed_work(&smb1357_dev->power_state_wrkr);
	flush_delayed_work(&smb1357_dev->power_state_wrkr);

	return;
}

#ifdef CONFIG_PM
static int smb1357_prepare(struct device *dev)
{
	struct smb1357_charger *smb = dev_get_drvdata(dev);

	CHR_INFO("%s +++\n", __func__);
	/*
	 * disable irq here doesn't mean smb1357 interrupt
	 * can't wake up system. smb1357 interrupt is triggered
	 * by GPIO pin, which is always active.
	 * When resume callback calls enable_irq, kernel
	 * would deliver the buffered interrupt (if it has) to
	 * driver.
	 */
	if (smb->client->irq > 0)
		disable_irq(smb->client->irq);

	return 0;
}

static void smb1357_complete(struct device *dev)
{
	struct smb1357_charger *smb = dev_get_drvdata(dev);

	CHR_INFO("%s +++\n", __func__);
	if (smb->client->irq > 0)
		enable_irq(smb->client->irq);
}
#else
#define smb1357_prepare NULL
#define smb1357_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb1357_runtime_suspend(struct device *dev)
{
	CHR_INFO("%s +++\n", __func__);
	return 0;
}

static int smb1357_runtime_resume(struct device *dev)
{
	CHR_INFO("%s +++\n", __func__);
	return 0;
}

static int smb1357_runtime_idle(struct device *dev)
{

	CHR_INFO("%s +++\n", __func__);
	return 0;
}
#else
#define smb1357_runtime_suspend	NULL
#define smb1357_runtime_resume	NULL
#define smb1357_runtime_idle	NULL
#endif

static int smb1357_suspend(struct device *dev)
{
	CHR_INFO("%s +++\n", __func__);
	if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
		smb1357_control_JEITA(false);
    	}
	cancel_delayed_work(&smb1357_dev->power_state_wrkr);
	flush_delayed_work(&smb1357_dev->power_state_wrkr);
	return 0;
}

static int smb1357_resume(struct device *dev)
{
	CHR_INFO("%s +++\n", __func__);
	if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
		smb1357_control_JEITA(true);
	}
	//cable_status_notify2( NULL, query_cable_status(), dev);
	schedule_delayed_work(&smb1357_dev->power_state_wrkr, 3*HZ);
	return 0;
}


static const struct i2c_device_id smb1357_id[] = {
	{ "smb1357_charger", 0},
	{},
};
MODULE_DEVICE_TABLE(i2c, smb1357_id);

static const struct dev_pm_ops smb1357_pm_ops = {
	.prepare		= smb1357_prepare,
	.complete		= smb1357_complete,
	.suspend                = smb1357_suspend,
	.resume	                = smb1357_resume,
	.runtime_suspend	= smb1357_runtime_suspend,
	.runtime_resume		= smb1357_runtime_resume,
	.runtime_idle		= smb1357_runtime_idle,
};

static struct i2c_driver smb1357_driver = {
	.driver = {
		.name	= "smb1357_charger",
		.owner	= THIS_MODULE,
		.pm		= &smb1357_pm_ops,
	},
	.probe		= smb1357_probe,
	.remove		= smb1357_remove,
	.shutdown	= smb1357_shutdown,
	.id_table	= smb1357_id,
};

static int __init smb1357_init(void)
{
	CHR_INFO("%s +++\n", __func__);
	return i2c_add_driver(&smb1357_driver);
}
module_init(smb1357_init);

static void __exit smb1357_exit(void)
{
	CHR_INFO("%s +++\n", __func__);
	i2c_del_driver(&smb1357_driver);
}
module_exit(smb1357_exit);


MODULE_AUTHOR("wigman sun <wigman_sun@asus.com>");
MODULE_DESCRIPTION("SMB1357 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb1357");
