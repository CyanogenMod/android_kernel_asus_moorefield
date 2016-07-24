/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
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
#include <linux/power/smb345-me372cg-charger.h>
#include <linux/seq_file.h>
#include <linux/slab.h>
#include <linux/usb/otg.h>
#include <linux/kernel.h>
#include <linux/wakelock.h>

#include "smb345_external_include.h"
#include "asus_battery.h"
#include <linux/proc_fs.h>
#include <linux/random.h>
#include <linux/usb/penwell_otg.h>
/*#include "../intel_mdf_charger.h"*/
#include <linux/HWVersion.h>
#include <asm/intel-mid.h>
#include <linux/microp_api.h>
static int HW_ID;
extern int Read_HW_ID(void);
extern int entry_mode;
static bool is_pad_present();
static void smb345_config_max_current(int usb_state);
static bool g_disable_700 = true;
static bool g_init_disable_aicl = true;

/* I2C communication related */
#define I2C_RETRY_COUNT 3
#define I2C_RETRY_DELAY 5

#define CFG_CHARGE_CURRENT            0x00
#define CFG_CHARGE_CURRENT_FCC_MASK        0xe0
#define CFG_CHARGE_CURRENT_FCC_SHIFT        5
#define CFG_CHARGE_CURRENT_PCC_MASK        0x18
#define CFG_CHARGE_CURRENT_PCC_SHIFT        3
#define CFG_CHARGE_CURRENT_TC_MASK        0x07
#define CFG_CHARGE_CURRENT_ALL        0x41

#define CFG_CURRENT_LIMIT            0x01
#define CFG_CURRENT_LIMIT_DC_MASK        0xf0
#define CFG_CURRENT_LIMIT_DC_SHIFT        4
#define CFG_CURRENT_LIMIT_USB_MASK        0x0f
#define CFG_CURRENT_LIMIT_SMB346_MASK   0xf0
#define CFG_CURRENT_LIMIT_SMB346_VALUE_1200 0x40
#define CFG_CURRENT_LIMIT_SMB346_VALUE_700 0x20
#define CFG_CURRENT_LIMIT_SMB346_VALUE_500 0x10
#define CFG_VARIOUS_FUNCS            0x02
#define CFG_VARIOUS_FUNCS_PRIORITY_USB        BIT(2)
#define CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE    BIT(4)
#define CFG_VARIOUS_FUNCS_BATTERY_OV    BIT(1)
#define CFG_FLOAT_VOLTAGE            0x03
#define CFG_FLOAT_VOLTAGE_THRESHOLD_MASK    0xc0
#define CFG_FLOAT_VOLTAGE_THRESHOLD_SHIFT    6
#define CFG_STAT                0x05
#define CFG_STAT_DISABLED            BIT(5)
#define CFG_STAT_ACTIVE_HIGH            BIT(7)
#define CFG_PIN                    0x06
#define CFG_PIN_EN_CTRL_MASK            0x60
#define CFG_PIN_EN_CTRL_ACTIVE_HIGH        0x40
#define CFG_PIN_EN_CTRL_ACTIVE_LOW        0x60
#define CFG_PIN_EN_APSD_IRQ            BIT(1)
#define CFG_PIN_EN_CHARGER_ERROR        BIT(2)
#define CFG_THERM                0x07
#define CFG_THERM_SOFT_HOT_COMPENSATION_MASK    0x03
#define CFG_THERM_SOFT_HOT_COMPENSATION_SHIFT    0
#define CFG_THERM_SOFT_COLD_COMPENSATION_MASK    0x0c
#define CFG_THERM_SOFT_COLD_COMPENSATION_SHIFT    2
#define CFG_THERM_MONITOR_DISABLED        BIT(4)
#define CFG_SYSOK                0x08
#define CFG_SYSOK_SUSPEND_HARD_LIMIT_DISABLED    BIT(2)
#define CFG_OTHER                0x09
#define CFG_OTHER_RID_MASK            0xc0
#define CFG_OTHER_RID_DISABLED_OTG_PIN        0x40
#define CFG_OTHER_RID_ENABLED_OTG_I2C        0x80
#define CFG_OTHER_RID_ENABLED_AUTO_OTG        0xc0
#define CFG_OTHER_OTG_PIN_ACTIVE_LOW        BIT(5)
#define CFG_OTG                    0x0a
#define CFG_OTG_TEMP_THRESHOLD_MASK        0x30
#define CFG_OTG_TEMP_THRESHOLD_SHIFT        4
#define CFG_OTG_CC_COMPENSATION_MASK        0xc0
#define CFG_OTG_CC_COMPENSATION_SHIFT        6
#define CFG_OTG_BATTERY_UVLO_THRESHOLD_MASK    0x03
#define CFG_TEMP_LIMIT                0x0b
#define CFG_TEMP_LIMIT_SOFT_HOT_MASK        0x03
#define CFG_TEMP_LIMIT_SOFT_HOT_SHIFT        0
#define CFG_TEMP_LIMIT_SOFT_COLD_MASK        0x0c
#define CFG_TEMP_LIMIT_SOFT_COLD_SHIFT        2
#define CFG_TEMP_LIMIT_HARD_HOT_MASK        0x30
#define CFG_TEMP_LIMIT_HARD_HOT_SHIFT        4
#define CFG_TEMP_LIMIT_HARD_COLD_MASK        0xc0
#define CFG_TEMP_LIMIT_HARD_COLD_SHIFT        6
#define CFG_FAULT_IRQ                0x0c
#define CFG_FAULT_IRQ_DCIN_UV            BIT(2)
#define CFG_FAULT_IRQ_OTG_UV            BIT(5)
#define CFG_STATUS_IRQ                0x0d
#define CFG_STATUS_IRQ_CHARGE_TIMEOUT        BIT(7)
#define CFG_STATUS_IRQ_TERMINATION_OR_TAPER    BIT(4)
#define CFG_ADDRESS                0x0e

/* Command registers */
#define CMD_A                    0x30
#define CMD_A_CHG_ENABLED            BIT(1)
#define CMD_A_SUSPEND_ENABLED            BIT(2)
#define CMD_A_OTG_ENABLED            BIT(4)
#define CMD_A_ALLOW_WRITE            BIT(7)
#define CMD_B                    0x31
#define CMD_B_USB9_AND_HC_MODE    0x03
#define CMD_C                    0x33

/* Interrupt Status registers */
#define IRQSTAT_A                0x35
#define IRQSTAT_C                0x37
#define IRQSTAT_C_TERMINATION_STAT        BIT(0)
#define IRQSTAT_C_TERMINATION_IRQ        BIT(1)
#define IRQSTAT_C_TAPER_IRQ            BIT(3)
#define IRQSTAT_D                0x38
#define IRQSTAT_D_CHARGE_TIMEOUT_STAT        BIT(2)
#define IRQSTAT_D_CHARGE_TIMEOUT_IRQ        BIT(3)
#define IRQSTAT_E                0x39
#define IRQSTAT_E_USBIN_UV_STAT            BIT(0)
#define IRQSTAT_E_USBIN_UV_IRQ            BIT(1)
#define IRQSTAT_E_DCIN_UV_STAT            BIT(4)
#define IRQSTAT_E_DCIN_UV_IRQ            BIT(5)
#define IRQSTAT_F                0x3a
#define IRQSTAT_F_OTG_UV_IRQ            BIT(5)
#define IRQSTAT_F_OTG_UV_STAT            BIT(4)

/* Status registers */
#define STAT_A                    0x3b
#define STAT_A_FLOAT_VOLTAGE_MASK        0x3f
#define STAT_B                    0x3c
#define STAT_C                    0x3d
#define STAT_C_CHG_ENABLED            BIT(0)
#define STAT_C_HOLDOFF_STAT            BIT(3)
#define STAT_C_CHG_MASK                0x06
#define STAT_C_CHG_SHIFT            1
#define STAT_C_CHG_TERM                BIT(5)
#define STAT_C_CHARGER_ERROR            BIT(6)
#define STAT_E                    0x3f

#define STATUS_UPDATE_INTERVAL            (HZ * 60)

struct smb345_otg_event {
	struct list_head    node;
	bool            param;
};

struct smb345_charger {
	struct mutex        lock;
	struct i2c_client    *client;
	struct power_supply    mains;
	struct power_supply    usb;
	struct power_supply    battery;
	bool            mains_online;
	bool            usb_online;
	bool            charging_enabled;
	bool            running;
	struct dentry        *dentry;
	struct dentry        *dentry2;
	struct otg_transceiver    *otg;
	struct notifier_block    otg_nb;
	struct work_struct    otg_work;
	struct delayed_work aicl_dete_work;
	struct workqueue_struct *chrgr_work_queue;
	struct list_head    otg_queue;
	spinlock_t        otg_queue_lock;
	bool            otg_enabled;
	bool            otg_battery_uv;
	const struct smb345_charger_platform_data    *pdata;
	/* wake lock to prevent S3 during charging */
	struct wake_lock wakelock;
};

static bool pad_pwr_supply();
#ifdef CONFIG_UPI_BATTERY
struct wake_lock wlock;
struct wake_lock wlock_t;
#ifdef ME372CG_ENG_BUILD
bool eng_charging_limit = true;
int asus_charging_limit_toggle_read(char *page, char **start, off_t off,
		int count, int *eof, void *date)
{
	int len = 0;

	BAT_DBG_E(" %s:\n", __func__);
	return len;
}

int asus_charging_limit_toggle_write(struct file *file, const char *buffer,
		unsigned long count, void *data)
{
	BAT_DBG_E(" %s:\n", __func__);

	if (buffer[0] == '1') {
		/* turn on charging limit in eng mode */
		eng_charging_limit = true;
	} else if (buffer[0] == '0') {
		/* turn off charging limit in eng mode */
		eng_charging_limit = false;
	}

	aicl_dete_worker(NULL);

	return count;
}

#ifndef CONFIG_NEW_PROC_FS
int init_asus_charging_limit_toggle(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = create_proc_entry("driver/charger_limit_enable", 0666, NULL);
	if (!entry) {
		BAT_DBG_E("Unable to create asus_charging_toggle\n");
		return -EINVAL;
	}
	entry->read_proc = asus_charging_limit_toggle_read;
	entry->write_proc = asus_charging_limit_toggle_write;

	return 0;
}
#else
static ssize_t nasus_charging_limit_toggle_write(struct file *file,
	const char __user *buf, size_t count, loff_t *ppos)
{
	char proc_buf[64];

	if (count > sizeof(proc_buf)) {
		BAT_DBG("%s: data error\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(proc_buf, buf, count)) {
		BAT_DBG("%s: read data from user space error\n", __func__);
		return -EFAULT;
	}

	if (proc_buf[0] == '1') {
		/* turn on charging limit in eng mode */
		eng_charging_limit = true;
	} else if (proc_buf[0] == '0') {
		/* turn off charging limit in eng mode */
		eng_charging_limit = false;
	}

	aicl_dete_worker(NULL);

	return count;
}

static int nasus_charging_limit_toggle_read(struct seq_file *m, void *v)
{
	seq_printf(m, "%s\n", __func__);
	return 0;
}
static int nasus_charging_limit_toggle_open(struct inode *inode, struct file *file)
{
	return single_open(file, nasus_charging_limit_toggle_read, NULL);
}
int init_asus_charging_limit_toggle(void)
{
	static const struct file_operations charging_limit_toggle_fops = {
		.owner = THIS_MODULE,
		.open = nasus_charging_limit_toggle_open,
		.read = seq_read,
		.write = nasus_charging_limit_toggle_write,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("driver/charger_limit_enable", 0666, NULL,
		&charging_limit_toggle_fops);
	if (!entry) {
		BAT_DBG_E("Unable to create asus_charging_toggle\n");
		return -EINVAL;
	}

	return 0;
}
#endif
#else
int init_asus_charging_limit_toggle(void)
{
	return 0;
}
#endif
#endif

/* global charger type variable lock */
DEFINE_MUTEX(g_usb_state_lock);
static int g_usb_state = CABLE_OUT;
static int g_pad_cable_state = CABLE_OUT;
/* global software charging toggle lock */
DEFINE_MUTEX(g_charging_toggle_lock);
static bool g_charging_toggle = true;

static struct smb345_charger *smb345_dev;
static struct chgr_dev_func smb345_tbl;

#if 0
/* Fast charge current in uA */
static const unsigned int fcc_tbl[] = {
	700000,
	900000,
	1200000,
	1500000,
	1800000,
	2000000,
	2200000,
	2500000,
};

/* Pre-charge current in uA */
static const unsigned int pcc_tbl[] = {
	100000,
	150000,
	200000,
	250000,
};

/* Termination current in uA */
static const unsigned int tc_tbl[] = {
	37500,
	50000,
	100000,
	150000,
	200000,
	250000,
	500000,
	600000,
};
#endif

/* Input current limit in mA */
static const unsigned int icl_tbl[] = {
	300,
	500,
	700,
	900,
	1200,
	1500,
	1800,
	2000,
	2200,
	2500,
};

/* Charge current compensation in uA */
static const unsigned int ccc_tbl[] = {
	250000,
	700000,
	900000,
	1200000,
};

#define EXPORT_CHARGER_OTG

#define DEBUG 1
#define DRIVER_VERSION            "1.1.0"

#define SMB345_MASK(BITS, POS)  ((unsigned char)(((1 << BITS) - 1) << POS))

/* Register definitions */
#define CHG_CURRENT_REG            0x00
#define INPUT_CURRENT_LIMIT_REG    0x01
#define VAR_FUNC_REG            0x02
#define FLOAT_VOLTAGE_REG        0x03
#define CHG_CTRL_REG            0x04
#define STAT_TIMER_REG            0x05
#define PIN_ENABLE_CTRL_REG        0x06
#define THERM_CTRL_A_REG        0x07
#define SYSOK_USB3_SELECT_REG    0x08
#define OTHER_CTRL_A_REG        0x09
#define OTG_TLIM_THERM_CNTRL_REG                0x0A

#define HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG    0x0B
#define SOFT_LIMIT_HOT_CELL_TEMP_MASK            SMB345_MASK(2, 0)
#define SOFT_LIMIT_COLD_CELL_TEMP_MASK            SMB345_MASK(2, 2)
#define HARD_LIMIT_HOT_CELL_TEMP_MASK            SMB345_MASK(2, 4)
#define HARD_LIMIT_COLD_CELL_TEMP_MASK            SMB345_MASK(2, 6)

#define FAULT_INTERRUPT_REG        0x0C
#define STATUS_INTERRUPT_REG    0x0D
#define I2C_BUS_SLAVE_REG        0x0E
#define CMD_A_REG        0x30
#define CMD_B_REG        0x31
#define CMD_C_REG        0x33
#define INTERRUPT_A_REG        0x35
#define INTERRUPT_B_REG        0x36
#define INTERRUPT_C_REG        0x37
#define INTERRUPT_D_REG        0x38
#define INTERRUPT_E_REG        0x39
#define INTERRUPT_F_REG        0x3A
#define STATUS_A_REG    0x3B
#define STATUS_B_REG    0x3C
#define STATUS_C_REG    0x3D
#define STATUS_D_REG    0x3E
#define STATUS_E_REG    0x3F

/* Status bits and masks */
#define CHG_STATUS_MASK        SMB345_MASK(2, 1)
#define CHG_ENABLE_STATUS_BIT        BIT(0)

/* Control bits and masks */
#define FAST_CHG_CURRENT_MASK            SMB345_MASK(4, 4)
#define AC_INPUT_CURRENT_LIMIT_MASK        SMB345_MASK(4, 0)
#define PRE_CHG_CURRENT_MASK            SMB345_MASK(3, 5)
#define TERMINATION_CURRENT_MASK        SMB345_MASK(3, 2)
#define PRE_CHG_TO_FAST_CHG_THRESH_MASK    SMB345_MASK(2, 6)
#define FLOAT_VOLTAGE_MASK                SMB345_MASK(6, 0)
#define CHG_ENABLE_BIT            BIT(1)
#define VOLATILE_W_PERM_BIT        BIT(7)
#define USB_SELECTION_BIT        BIT(1)
#define SYSTEM_FET_ENABLE_BIT    BIT(7)
#define AUTOMATIC_INPUT_CURR_LIMIT_BIT            BIT(4)
#define AUTOMATIC_POWER_SOURCE_DETECTION_BIT    BIT(2)
#define BATT_OV_END_CHG_BIT        BIT(1)
#define VCHG_FUNCTION            BIT(0)
#define CURR_TERM_END_CHG_BIT    BIT(6)

#define OTGID_PIN_CONTROL_MASK    SMB345_MASK(2, 6)
#define OTGID_PIN_CONTROL_BITS    BIT(6)

#define OTG_CURRENT_LIMIT_AT_USBIN_MASK    SMB345_MASK(2, 2)
#define OTG_CURRENT_LIMIT_750mA    (BIT(2) | BIT(3))
#define OTG_CURRENT_LIMIT_500mA    BIT(3)
#define OTG_CURRENT_LIMIT_250mA    BIT(2)
#define OTG_BATTERY_UVLO_THRESHOLD_MASK    SMB345_MASK(2, 0)

#define CHARGE_CURRENT_COMPENSATION         SMB345_MASK(2, 6)
#define CHARGE_CURRENT_COMPENSATION_VALUE   0x00

#define CREATE_DEBUGFS_INTERRUPT_STATUS_REGISTERS
#define SMB358_FAST_CHG_CURRENT_MASK            SMB345_MASK(3, 5)
#define SMB358_TERMINATION_CURRENT_MASK         SMB345_MASK(3, 0)
#define SMB358_TERMINATION_CURRENT_VALUE_200mA    BIT(0)|BIT(1)|BIT(2)
#define SMB358_FAST_CHG_CURRENT_VALUE_2000mA    BIT(5)|BIT(6)|BIT(7)

static int smb346_soc_detect_batt_tempr(int usb_state);
static inline int get_battery_rsoc(int *rsoc);
static inline int get_battery_temperature(int *tempr);
static inline int get_battery_voltage(int *volt);

#ifdef CONFIG_PF400CG
extern void set_off();
extern void set_red();
extern void set_green();
extern void set_orange();
extern void set_blink();

void led_control()
{
	int usb_state;
	int rsoc;

	if (entry_mode == 1)
		return;
	if (is_pad_present())
		return;

	mutex_lock(&g_usb_state_lock);
	usb_state = g_usb_state;
	mutex_unlock(&g_usb_state_lock);

	if (usb_state != USB_IN && usb_state != AC_IN && usb_state != CABLE_OUT)
		return;

	/* acquire battery rsoc here */
	if (get_battery_rsoc(&rsoc)) {
		BAT_DBG_E(" %s: * fail to get battery rsoc *\n", __func__);
	} else {
		if (usb_state == CABLE_OUT) {
			if (rsoc < 15 && rsoc > -1) {
				BAT_DBG(" %s: ======== LED BLINK ========\n", __func__);
				set_blink();
			} else {
				BAT_DBG(" %s: ======== LED OFF ========\n", __func__);
				set_off();
			}
		} else {
			if (rsoc == 100) {
				BAT_DBG(" %s: ======== LED GREEN ========\n", __func__);
				set_green();
			} else {
				BAT_DBG(" %s: ======== LED ORANGE ========\n", __func__);
				set_orange();
			}
		}
	}
}
#else
void led_control() {}
#endif

static char *supply_list[] = {
	"battery",
};

static enum power_supply_property asus_power_properties[] = {
	POWER_SUPPLY_PROP_PRESENT,
	POWER_SUPPLY_PROP_ONLINE,
};

static int smb345_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val);

static struct power_supply smb345_power_supplies[] = {
	{
		.name = "ac",
		.type = POWER_SUPPLY_TYPE_MAINS,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
	{
		.name = "usb",
		.type = POWER_SUPPLY_TYPE_USB,
		.supplied_to = supply_list,
		.num_supplicants = ARRAY_SIZE(supply_list),
		.properties = asus_power_properties,
		.num_properties = ARRAY_SIZE(asus_power_properties),
		.get_property = smb345_power_get_property,
	},
};

static int smb345_power_get_property(struct power_supply *psy,
		enum power_supply_property psp,
		union power_supply_propval *val)
{
	int ret = 0;
	int usb_state;
	int chrg_status;

	mutex_lock(&g_usb_state_lock);
	usb_state = g_usb_state;
	mutex_unlock(&g_usb_state_lock);

	switch (psp) {
	case POWER_SUPPLY_PROP_ONLINE:

		if (psy->type == POWER_SUPPLY_TYPE_USB) {

			if (is_pad_present()) {
				val->intval = 0;
			} else{
				val->intval = (usb_state == USB_IN) ? 1 : 0;
			}

		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (is_pad_present()) {
				val->intval = (AX_MicroP_get_USBDetectStatus(Batt_P01) == P01_CABLE_CHARGER) ? 1 : 0;
			} else {
				val->intval = (g_pad_cable_state == AC_IN) ? 1 : 0;
			}

		} else {

			ret = -EINVAL;
		}
		break;
	case POWER_SUPPLY_PROP_PRESENT:

		if (psy->type == POWER_SUPPLY_TYPE_USB) {
			/* for ATD test to acquire the status about charger ic */
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}
			chrg_status = smb345_get_charging_status();
			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			} else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
			if (!smb345_has_charger_error()) {
				val->intval = 1;
				return 0;
			}

			chrg_status = smb345_get_charging_status();

			if (chrg_status == POWER_SUPPLY_STATUS_CHARGING) {
				val->intval = 1;
				return 0;
			} else if (chrg_status == POWER_SUPPLY_STATUS_FULL) {
				val->intval = 1;
				return 0;
			}
			val->intval = 0;
		} else
			ret = -EINVAL;
		break;
	default:
		ret = -EINVAL;
	}

	return ret;
}

extern bool get_sw_charging_toggle()
{
	bool ret;

	mutex_lock(&g_charging_toggle_lock);
	ret = g_charging_toggle;
	mutex_unlock(&g_charging_toggle_lock);

	return ret;
}

extern int get_charger_type()
{
	int ret;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return -1;
	}

	mutex_lock(&g_usb_state_lock);
	if (is_pad_present())
		ret = PAD_SUPPLY;
	else
		ret = g_usb_state;
	mutex_unlock(&g_usb_state_lock);

	return ret;
}

int request_power_supply_changed()
{
	int ret = -1;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return -1;
	}

	power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
	power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);

	return ret;
}

static int smb345_register_power_supply(struct device *dev)
{
	int ret;

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_USB-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply USB\n");
		goto batt_err_reg_fail_usb;
	}

	ret = power_supply_register(dev, &smb345_power_supplies[CHARGER_AC-1]);
	if (ret) {
		BAT_DBG_E("Fail to register power supply AC\n");
		goto batt_err_reg_fail_ac;
	}

	return 0;

batt_err_reg_fail_ac:
	power_supply_unregister(&smb345_power_supplies[CHARGER_USB-1]);
batt_err_reg_fail_usb:
	return ret;
}

static int smb345_read_reg(struct i2c_client *client, int reg,
	u8 *val, int ifDebug)
{
	s32 ret;
	struct smb345_charger *smb345_chg;

	smb345_chg = i2c_get_clientdata(client);
	ret = i2c_smbus_read_byte_data(smb345_chg->client, reg);
	if (ret < 0) {
		dev_err(&smb345_chg->client->dev,
			"i2c read fail: can't read from Reg%02Xh: %d\n", reg, ret);
		return ret;
	} else {
		*val = ret;
	}
	if (ifDebug)
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(*val));

	return 0;
}

static int smb345_write_reg(struct i2c_client *client, int reg, u8 val)
{

	s32 ret;
	struct smb345_charger *smb345_chg;

	smb345_chg = i2c_get_clientdata(client);

	ret = i2c_smbus_write_byte_data(smb345_chg->client, reg, val);
	if (ret < 0) {
		dev_err(&smb345_chg->client->dev,
			"i2c write fail: can't write %02X to %02X: %d\n",
			val, reg, ret);
		return ret;
	}
	return 0;
}

static int smb345_masked_write(struct i2c_client *client, int reg,
		u8 mask, u8 val)
{
	s32 rc;
	u8 temp;

	rc = smb345_read_reg(client, reg, &temp, 0);
	if (rc) {
		pr_err("smb345_read_reg failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	temp &= ~mask;
	temp |= val & mask;
	rc = smb345_write_reg(client, reg, temp);
	if (rc) {
		pr_err("smb345_write failed: reg=%03X, rc=%d\n", reg, rc);
		return rc;
	}
	return 0;
}

static int smb345_read(struct smb345_charger *smb, u8 reg)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_read_byte_data(smb->client, reg);
		if (ret < 0) {
			retry_count--;
			dev_warn(&smb->client->dev, "fail to read reg %02xh: %d\n",
				reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);


	return ret;
}

static int smb345_write(struct smb345_charger *smb, u8 reg, u8 val)
{
	int ret;
	int retry_count = I2C_RETRY_COUNT;

	do {
		ret = i2c_smbus_write_byte_data(smb->client, reg, val);
		if (ret < 0) {
			retry_count--;
			dev_warn(&smb->client->dev, "fail to write reg %02xh: %d\n",
				reg, ret);
			msleep(I2C_RETRY_DELAY);
		}
	} while (ret < 0 && retry_count > 0);

	return ret;
}

static int smb345_set_writable(struct smb345_charger *smb, bool writable)
{
	int ret;

	ret = smb345_read(smb, CMD_A);
	if (ret < 0)
		return ret;

	if (writable)
		ret |= CMD_A_ALLOW_WRITE;
	else
		ret &= ~CMD_A_ALLOW_WRITE;

	return smb345_write(smb, CMD_A, ret);
}

static int cancel_soft_hot_temp_limit(bool cancel_it)
{
	int ret = -1;

	/* ME371MG EVB/SR1 meet this problem that smb345 stop charging
	    when IC temperature is high up to Soft Hot Limit. But Battery
	    is not full charging. We disable this limitation.
	*/

	ret = smb345_read(smb345_dev, CFG_THERM);
	if (ret < 0)
		return ret;

	ret &= ~CFG_THERM_SOFT_HOT_COMPENSATION_MASK;
	if (!cancel_it)
		ret &= 0x02;

	ret = smb345_write(smb345_dev, CFG_THERM, ret);

	return ret;
}

/*----------------------------------------------------------------------------*/
/* JEITA function for cell temperature control by SoC
 */
#ifndef CONFIG_ME302C
int smb345_soc_control_jeita(void)
{
	int ret;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return 1;
	}

	pr_info("%s:", __func__);
	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	/* write 0bh[5:4]="11" */
	ret = smb345_masked_write(smb345_dev->client,
			HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
			HARD_LIMIT_HOT_CELL_TEMP_MASK,
			0xff);
	if (ret) {
		pr_err("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;

	}

	/* write 0bh[1:0]="11" */
	ret = smb345_masked_write(smb345_dev->client,
			HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
			SOFT_LIMIT_HOT_CELL_TEMP_MASK,
			0xff);
	if (ret) {
		pr_err("fail to set SOFT_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}


#if defined(CONFIG_PF400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_A400CG)
	/* write 07h[1:0]="00" */
	ret = cancel_soft_hot_temp_limit(true);
	if (ret) {
		pr_err("fail to set Soft Hot Limit Behavior to No Response ret=%d\n", ret);
		return ret;
	}

#endif

	return ret;
}

/* JEITA function for cell temperature control by Charger IC
 */
int smb345_charger_control_jeita(void)
{
	int ret;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return 1;
	}


	pr_info("%s:", __func__);
	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	/* write 0bh[5:4]= "01" or "00" */
	ret = smb345_masked_write(smb345_dev->client,
			HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
			HARD_LIMIT_HOT_CELL_TEMP_MASK,
#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
			0x01);
#else
			0x00);
#endif
	if (ret) {
		pr_err("fail to set HARD_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}


	/* write 0bh[1:0]="01" */
	ret = smb345_masked_write(smb345_dev->client,
			HARD_SOFT_LIMIT_CELL_TEMP_MONITOR_REG,
			SOFT_LIMIT_HOT_CELL_TEMP_MASK,
			0x01);
	if (ret) {
		pr_err("fail to set SOFT_LIMIT_HOT_CELL_TEMP_MASK ret=%d\n", ret);
		return ret;
	}


#if defined(CONFIG_PF400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_A400CG)
	/* write 07h[1:0]="10" */
	ret = cancel_soft_hot_temp_limit(false);
	if (ret) {
		pr_err("fail to set Soft Hot Limit Behavior to Float Voltage Compensation ret=%d\n", ret);
		return ret;
	}
#endif


	return ret;
}

/* JEITA function for float voltage configuration by SoC
 */
int smb345_soc_control_float_vol(int bat_temp)
{
	int ret;
	int batt_volt;

	if (!smb345_dev) {
		pr_err("Warning: smb345_dev is null due to probe function has error\n");
		return 1;
	}


	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG) || defined(CONFIG_ME175CG)
	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
		return ret;
	} else
		BAT_DBG(" %s: get battery voltage(%d)\n", __func__, batt_volt);

#endif

	/* the unit of battery temperature is 0.1C */
	if (bat_temp < FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD) {
		/* write 03h[5:0]="101010" or "101011"*/
		ret = smb345_masked_write(smb345_dev->client,
			FLOAT_VOLTAGE_REG,
			FLOAT_VOLTAGE_MASK,
#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
			0x2A);
#else
			0x2B);
#endif
	} else {

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
		if (bat_temp < 600) {
			if (batt_volt < 4110) {
				/* write 03h[5:0]="011110"*/
				ret = smb345_masked_write(smb345_dev->client,
						FLOAT_VOLTAGE_REG,
						FLOAT_VOLTAGE_MASK,
						0x1E);
			} else {

				/* write 03h[5:0]="101010"*/
				ret = smb345_masked_write(smb345_dev->client,
						FLOAT_VOLTAGE_REG,
						FLOAT_VOLTAGE_MASK,
						0x2A);
			}
		} else {

			/* write 03h[5:0]="101010"*/
			ret = smb345_masked_write(smb345_dev->client,
						FLOAT_VOLTAGE_REG,
						FLOAT_VOLTAGE_MASK,
						0x2A);
		}
#elif defined(CONFIG_ME175CG)
		if (bat_temp < 550) {
			if (batt_volt < 4110) {
				/* write 03h[5:0]="011110"*/
				ret = smb345_masked_write(smb345_dev->client,
						FLOAT_VOLTAGE_REG,
						FLOAT_VOLTAGE_MASK,
						0x1E);
			} else {

				/* write 03h[5:0]="101011"*/
				ret = smb345_masked_write(smb345_dev->client,
					FLOAT_VOLTAGE_REG,
					FLOAT_VOLTAGE_MASK,
					0x2B);
			}
		} else {

			/* write 03h[5:0]="011110"*/
			ret = smb345_masked_write(smb345_dev->client,
					FLOAT_VOLTAGE_REG,
					FLOAT_VOLTAGE_MASK,
					0x1E);
		}
#else
		/* write 03h[5:0]="011111"*/
		ret = smb345_masked_write(smb345_dev->client,
					FLOAT_VOLTAGE_REG,
					FLOAT_VOLTAGE_MASK,
					0x1F);
#endif
	}
	if (ret)
		pr_err("fail to set FLOAT_VOLTAGE_REG ret=%d\n", ret);

	return ret;
}
#else
int smb345_soc_control_jeita(void)
{
	return 0;
};
int smb345_charger_control_jeita(void)
{
	return 0;
};
int smb345_soc_control_float_vol(int bat_temp)
{
	return 0;
};
#endif
/*----------------------------------------------------------------------------*/
/* Convert register value to current using lookup table */
static int hw_to_current(const unsigned int *tbl,
		size_t size, unsigned int val)
{
	if (val >= size)
		return tbl[size-1];
	return tbl[val];
}
/* Acquire the value of AICL Results in Status Register E (3Fh)
   return the current value (unit: mA)
*/
static int get_aicl_results(void)
{
	int ret;
	ret = smb345_read(smb345_dev, STAT_E);
	if (ret < 0) {
		BAT_DBG_E(" %s: fail to read STAT_E reg\n", __func__);
		return ret;
	}

	ret &= 0x0F;
	return hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret);
}

/* Acquire the value of AICL Results in Status Register E (3Fh) */
static ssize_t get_input_current(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: ERROR: smb345_dev is null due to probe function has error\n",
			__func__);

		return sprintf(buf, "%d\n", -EINVAL);
	}

	ret = smb345_read(smb345_dev, STAT_E);
	if (ret < 0) {
		pr_info("%s: ERROR: i2c read error\n", __func__);
		return sprintf(buf, "%d\n", -EIO);
	}

	ret &= 0x0F;

	return sprintf(buf, "%d\n", hw_to_current(icl_tbl, ARRAY_SIZE(icl_tbl), ret));
}

/* Acquire the charging status */
static ssize_t get_charge_status(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	int ret;

	ret = smb345_get_charging_status();
	if (ret == POWER_SUPPLY_STATUS_CHARGING || ret == POWER_SUPPLY_STATUS_FULL)
		ret = 1;
	else
		ret = 0;
	return sprintf(buf, "%d\n", ret);
}

static char gbuffer[64];
/* Generate UUID by invoking kernel library */
static void generate_key(void)
{
	char sysctl_bootid[16];

	generate_random_uuid(sysctl_bootid);
	sprintf(gbuffer, "%pU", sysctl_bootid);
}

/* Acquire the UUID */
static ssize_t get_charge_keys(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	generate_key();
	return sprintf(buf, "%s\n", gbuffer);
}

static DEVICE_ATTR(charge_keys, S_IRUGO, get_charge_keys, NULL);
static DEVICE_ATTR(input_current, S_IRUGO, get_input_current, NULL);
static DEVICE_ATTR(charge_status, S_IRUGO, get_charge_status, NULL);
static struct attribute *dev_attrs[] = {
	&dev_attr_input_current.attr,
	&dev_attr_charge_status.attr,
	&dev_attr_charge_keys.attr,
	NULL,
};
static struct attribute_group dev_attr_grp = {
	.attrs = dev_attrs,
};

#define SMB_DUMP(...) \
do { \
	local_len = sprintf(page, __VA_ARGS__); \
	len += local_len; \
	page += local_len; \
} while (0)

/*----------------------------------------------------------------------------*/

#if defined(CONFIG_ME372CG) || defined(CONFIG_ME372CL)
static int config_otg_regs(int toggle)
{
	int ret;

	if (toggle) {
		/* I2C enable OTG function */
		ret = smb345_masked_write(smb345_dev->client, CMD_A_REG,
				BIT(4), BIT(4));
		if (ret) {
			pr_err("fail to set OTG Enable bit ret=%d\n", ret);
			return ret;
		}
		/* Refer to SMB345 Application Note 72 to solve serious problems */
		ret = smb345_masked_write(smb345_dev->client,
					OTG_TLIM_THERM_CNTRL_REG,
					OTG_CURRENT_LIMIT_AT_USBIN_MASK,
					OTG_CURRENT_LIMIT_750mA);
		if (ret) {
			pr_err("fail to set OTG current limit 750mA. ret=%d\n", ret);
			return ret;

		}

		/* OTG 5V function set to PIN control due to HW will
		    reset IC to default when external USB device is
		    removed. This function support in SR2. And
		    charger IC will be replaced with smb345
		*/
		ret = smb345_masked_write(smb345_dev->client,
				OTHER_CTRL_A_REG,
				OTGID_PIN_CONTROL_MASK,
				OTGID_PIN_CONTROL_BITS);
		if (ret) {
			pr_err("fail to set OTGID_PIN_CONTROL_BITS rc=%d\n", ret);
			return ret;
		}
	} else {

	}


	return ret;
}
#elif defined(CONFIG_PF450CL)
static int config_otg_regs(int toggle)
{
	int ret;

	if (toggle) {
		/* I2C disable OTG function */
		ret = smb345_masked_write(smb345_dev->client, CMD_A_REG,
			BIT(4), 0);
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			return ret;
	}

	/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
	ret = smb345_masked_write(smb345_dev->client,
		OTG_TLIM_THERM_CNTRL_REG,
		OTG_CURRENT_LIMIT_AT_USBIN_MASK,
		0);
	if (ret) {
		pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
		return ret;
	}

	/* Set OTG battery UVLO threshold to 2.7V: 0Ah[1:0]="00" */
	ret = smb345_masked_write(smb345_dev->client,
		OTG_TLIM_THERM_CNTRL_REG,
		OTG_BATTERY_UVLO_THRESHOLD_MASK,
		0);
	if (ret) {
		pr_err("fail to set OTG battery UVLO threshold to 2.7V ret=%d\n",
		ret);
		return ret;
	}

	/* Toggle to enable OTG function: output High */
	gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 1);

	/* Set OTG current limit to 750mA: 0Ah[3:0]="1000" */
	ret = smb345_masked_write(smb345_dev->client,
		OTG_TLIM_THERM_CNTRL_REG,
		OTG_CURRENT_LIMIT_AT_USBIN_MASK,
		BIT(3));
	if (ret) {
		pr_err("fail to set OTG current limit 750mA ret=%d\n", ret);
		return ret;
		}
	} else {

	#if 0
	/* Reset Charger IC: 31h[7]="1" */
	ret = smb345_masked_write(smb345_dev->client,
		CMD_B,
		BIT(7),
		BIT(7));
	if (ret) {
		pr_err("fail to reset Charger IC ret=%d\n", ret);
		return ret;
	}
	#endif

	/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
	ret = smb345_masked_write(smb345_dev->client,
		OTG_TLIM_THERM_CNTRL_REG,
		OTG_CURRENT_LIMIT_AT_USBIN_MASK,
		0);
	if (ret) {
		pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
		return ret;
	}

		/* Toggle to disable OTG function: output Low */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 0);
	}

	return ret;
}
#elif defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
static int config_otg_regs(int toggle)
{
	int ret = -1;

	if (toggle) {
		/* I2C disable OTG function */
		ret = smb345_masked_write(smb345_dev->client, CMD_A_REG,
					BIT(4), 0);
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			return ret;
		}

		/* Set OTG current limit to 500mA: 0Ah[3:2]="10" */
		ret = smb345_masked_write(smb345_dev->client,
					OTG_TLIM_THERM_CNTRL_REG,
					OTG_CURRENT_LIMIT_AT_USBIN_MASK,
					OTG_CURRENT_LIMIT_500mA);
		if (ret) {
			pr_err("fail to set OTG current limit 500mA ret=%d\n", ret);
			return ret;
		}

		/* Toggle to enable OTG function: output High */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 1);
	} else {

		/* Set OTG current limit to 250mA: 0Ah[3:2]="01" */
		ret = smb345_masked_write(smb345_dev->client,
			OTG_TLIM_THERM_CNTRL_REG,
			OTG_CURRENT_LIMIT_AT_USBIN_MASK,
			OTG_CURRENT_LIMIT_250mA);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Toggle to disable OTG function: output Low */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 0);
	}

	return ret;
}
#elif defined(CONFIG_PF450CL)
static int config_otg_regs(int toggle)
{
	int ret = -1;

	if (toggle) {
		/* I2C disable OTG function */
		ret = smb345_masked_write(smb345_dev->client, CMD_A_REG,
			BIT(4), 0);
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			return ret;
		}

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(smb345_dev->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Set OTG battery UVLO threshold to 2.7V: 0Ah[1:0]="00" */
		ret = smb345_masked_write(smb345_dev->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_BATTERY_UVLO_THRESHOLD_MASK,
				0);
		if (ret) {
			pr_err("fail to set OTG battery UVLO threshold to 2.7V ret=%d\n",
				ret);
			return ret;
		}

		/* Toggle to enable OTG function: output High */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 1);

		/* Set OTG current limit to 750mA: 0Ah[3:0]="1000" */
		ret = smb345_masked_write(smb345_dev->client,
					OTG_TLIM_THERM_CNTRL_REG,
					OTG_CURRENT_LIMIT_AT_USBIN_MASK,
					BIT(3));
		if (ret) {
			pr_err("fail to set OTG current limit 750mA ret=%d\n", ret);
			return ret;
		}
	} else {
		#if 0
		/* Reset Charger IC: 31h[7]="1" */
		ret = smb345_masked_write(smb345_dev->client,
					CMD_B,
					BIT(7),
					BIT(7));
		if (ret) {
			pr_err("fail to reset Charger IC ret=%d\n", ret);
			return ret;
		}
		#endif

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(smb345_dev->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Toggle to disable OTG function: output Low */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 0);
	}

	return ret;
}
#elif defined(CONFIG_ME175CG)
static int config_otg_regs(int toggle)
{
	int ret = -1;

	if (toggle) {
		/* I2C disable OTG function */
		ret = smb345_masked_write(smb345_dev->client, CMD_A_REG,
				BIT(4), 0);
		if (ret) {
			pr_err("fail to set OTG Disable bit ret=%d\n", ret);
			return ret;
		}

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(smb345_dev->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Set OTG battery UVLO threshold to 2.7V: 0Ah[1:0]="00" */
		ret = smb345_masked_write(smb345_dev->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_BATTERY_UVLO_THRESHOLD_MASK,
				0);
		if (ret) {
			pr_err("fail to set OTG battery UVLO threshold to 2.7V ret=%d\n",
				ret);
			return ret;
		}

		/* Toggle to enable OTG function: output High */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 1);

		/* Set OTG current limit to 750mA: 0Ah[3:0]="1000" */
		ret = smb345_masked_write(smb345_dev->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				BIT(3));
		if (ret) {
			pr_err("fail to set OTG current limit 750mA ret=%d\n", ret);
			return ret;
		}
	} else {

#if 0
		/* Reset Charger IC: 31h[7]="1" */
		ret = smb345_masked_write(smb345_dev->client,
				CMD_B,
				BIT(7),
				BIT(7));
		if (ret) {
			pr_err("fail to reset Charger IC ret=%d\n", ret);
			return ret;
		}
#endif

		/* Set OTG current limit to 250mA: 0Ah[3:2]="00" */
		ret = smb345_masked_write(smb345_dev->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				0);
		if (ret) {
			pr_err("fail to set OTG current limit 250mA ret=%d\n", ret);
			return ret;
		}

		/* Toggle to disable OTG function: output Low */
		gpio_set_value(smb345_dev->pdata->gp_sdio_2_clk, 0);
	}

	return ret;
}
#elif defined(CONFIG_ME302C)
static int config_otg_regs(int toggle)
{
	int ret = -1;

	if (toggle) {
		ret = smb345_masked_write(smb345_dev->client, CMD_A_REG,
				BIT(4), BIT(4));
		if (ret) {
			pr_err("Failed to set OTG Enable bit ret=%d\n", ret);
			return ret;
		}


		ret = smb345_masked_write(smb345_dev->client, OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK, OTG_CURRENT_LIMIT_750mA);
		if (ret) {
			pr_err("Failed to set OTG current limit 750mA. ret=%d\n", ret);
			return ret;
		}

		/* OTG 5V function set to PIN control due to HW will reset IC to default
		when external USB device is removed. This function support in SR2. And
		charger IC will be replaced with smb345 */
		ret = smb345_masked_write(smb345_dev->client, OTHER_CTRL_A_REG,
				OTGID_PIN_CONTROL_MASK, OTGID_PIN_CONTROL_BITS);
		if (ret) {
			pr_err("Failed to set OTGID_PIN_CONTROL_BITS rc=%d\n", ret);
			return ret;
		}
	} else {

	}
}
#else
static int config_otg_regs(int toggle)
{
	return 0;
}
#endif
/*----------------------------------------------------------------------------*/
static int otg(int toggle)
{
	int ret;

	if (!smb345_dev) {
		pr_info("Warning: smb345_dev is null due to probe function has error\n");
		return 1;
	}


	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;


	ret = config_otg_regs(toggle);
	if (ret < 0)
		return ret;

	smb345_dev->otg_enabled = (toggle > 0 ? true : false);
	return 0;
}

/* enable/disable AICL function */
static int smb345_OptiCharge_Toggle(bool on)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return -1;
	}

	ret = smb345_read(smb345_dev, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;
	else
		ret &= ~CFG_VARIOUS_FUNCS_OPTICHARGE_TOGGLE;

	ret = smb345_write(smb345_dev, CFG_VARIOUS_FUNCS, ret);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}

/* print the value in Various Functions Register */
static int smb345_get_AICL(void)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
		__func__);

		return -1;
	}

	ret = smb345_read(smb345_dev, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_VARIOUS_FUNCS, BYTETOBINARY(ret));
fail:
	return ret;
}

/* print the value in CMD_B */
static int smb345_get_USB9_HC_Toggle(void)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);
		return -1;

	}

	ret = smb345_read(smb345_dev, CMD_B);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
fail:
	return ret;
}

/* enable USB5 or USB9 and HC mode function */
#if !defined(CONFIG_PF400CG) && !defined(CONFIG_A400CG)
static int smb345_USB9_HC_Toggle(bool on)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);
		return -1;

	}

	ret = smb345_read(smb345_dev, CMD_B);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= CMD_B_USB9_AND_HC_MODE;
	else
		ret &= ~CMD_B_USB9_AND_HC_MODE;
	ret = smb345_write(smb345_dev, CMD_B, ret);

fail:
	return ret;
}
#else
static int smb345_USB9_HC_Toggle(bool on)
{
	return 0;
}
#endif
/* print the value in CFG_PIN */
static int smb345_get_USB9_HC_PIN_Control(void)
{
	int ret;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return -1;
	}

	ret = smb345_read(smb345_dev, CFG_PIN);
	if (ret < 0)
		goto fail;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_PIN, BYTETOBINARY(ret));
fail:
	return ret;
}

/* enable USB5 or USB9 and HC mode pin control function */
#if !defined(CONFIG_PF400CG) && !defined(CONFIG_ME175CG) && !defined(CONFIG_A400CG)
static int smb345_USB9_HC_PIN_Control(bool on)
{
	int ret;
	u8 b = BIT(4);

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);
		return -1;

	}

	ret = smb345_read(smb345_dev, CFG_PIN);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= b;
	else
		ret &= ~b;
	ret = smb345_write(smb345_dev, CFG_PIN, ret);

fail:
	return ret;
}
#else
static int smb345_USB9_HC_PIN_Control(bool on)
{
	return 0;
}
#endif
/* Convert current to register value using lookup table */
static int current_to_hw(const unsigned int *tbl,
		size_t size, unsigned int val)
{
	size_t i;

	for (i = 0; i < size; i++)
		if (val < tbl[i])
			break;
	return i > 0 ? i - 1 : -EINVAL;
}

/* print the value in CFG_CURRENT_LIMIT */
static int smb345_get_current_limits()
{
	int ret, index = -1;

	if (!smb345_dev) {
		pr_info("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);
		return -1;

	}

	ret = smb345_read(smb345_dev, CFG_CURRENT_LIMIT);
	if (ret < 0)
		return ret;
	else
		pr_info("Reg%02Xh = " BYTETOBINARYPATTERN
			"\n", CFG_CURRENT_LIMIT, BYTETOBINARY(ret));
	return ret;
}

/* config DC_IN current limits according to BAT RSOC */
static int smb346_set_pad_supply_dc_in_limits(int rsoc)
{
	int ret;

	if (rsoc >= 20)
		g_disable_700 = true;
	else if (rsoc <= 15)
		g_disable_700 = false;


	if ((rsoc > 15) && (g_disable_700)) {

		pr_info("%s: CFG_CURRENT_LIMIT_SMB346_VALUE_500(%d)\n", __func__, g_disable_700);

		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}

		ret = smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_500);

		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
	}  else if (rsoc > -1) {

		pr_info("%s: CFG_CURRENT_LIMIT_SMB346_VALUE_700(%d)\n", __func__, g_disable_700);


		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}

		ret = smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_700);

		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
	} else{

		pr_info("%s: smb346_set_pad_supply_dc_in_limits FAIL, rsoc = %d, g_disable_700 = %d !!!\n", __func__, rsoc, g_disable_700);
		ret = -1;
	}


	if (ret < 0)
		return ret;

	return ret;
}

#if defined(CONFIG_PF450CL)
static int smb345_set_current_limits(int usb_state, bool is_twinsheaded)
{
	int ret;

	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	if (usb_state == AC_IN) {
		return smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_1200);
	} else if (usb_state == USB_IN) {
		return smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_500);
	}

	return ret;
}
#elif CONFIG_ME175CG
static int smb345_set_current_limits(int usb_state, bool is_twinsheaded)
{
	int ret;

	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	if (usb_state == AC_IN) {
		return smb345_masked_write(smb345_dev->client,
				CFG_CURRENT_LIMIT,
				CFG_CURRENT_LIMIT_SMB346_MASK,
				CFG_CURRENT_LIMIT_SMB346_VALUE_1200);
	} else if (usb_state == USB_IN) {

	}

	return ret;
}
#else
static int smb345_set_current_limits(int usb_state, bool is_twinsheaded)
{
	int ret, index = -1;

	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	ret = smb345_read(smb345_dev, CFG_CURRENT_LIMIT);
	if (ret < 0)
		return ret;

	if (usb_state == AC_IN) {
		index = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				smb345_dev->pdata->mains_current_limit);
	} else if (usb_state == USB_IN) {

		if (is_twinsheaded)
			index = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				smb345_dev->pdata->twins_h_current_limit);
		else
			index = current_to_hw(icl_tbl, ARRAY_SIZE(icl_tbl),
				smb345_dev->pdata->usb_hc_current_limit);
	}
	if (index < 0)
		return index;

	return smb345_write(smb345_dev, CFG_CURRENT_LIMIT, index);
}

#endif

static void aicl_cur_control(int usb_state)
{
	int aicl_result;

	if (usb_state != AC_IN) {
		if (usb_state == PAD_SUPPLY)
			smb345_config_max_current(usb_state);
		return;
	}

	aicl_result = get_aicl_results();
	if (aicl_result > 500) {
		dev_info(&smb345_dev->client->dev,
			"%s: do nothing when aicl result(%dmA) > 500mA.\n",
			__func__,
			aicl_result);
		return;
	} else {

		dev_info(&smb345_dev->client->dev,
			"%s: execute AICL routine control work\n",
			aicl_result,
			__func__);
	}

	/* Disable AICL - Write 02h[4]="0" */
	if (smb345_OptiCharge_Toggle(false) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to disable AICL\n", __func__);
		return;
	}

	/* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" */
	if (smb345_set_current_limits(AC_IN, false) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to set max current limits for USB_IN\n",
			__func__);
		return;
	}

	/* Enable AICL - Write 02h[4]="1" */
	if (smb345_OptiCharge_Toggle(true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to enable AICL\n", __func__);
		return;
	}
}

void aicl_dete_worker(struct work_struct *dat)
{
	int usb_state;
	int rsoc;

	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return;
	}

	mutex_lock(&g_usb_state_lock);
	usb_state = g_usb_state;
	mutex_unlock(&g_usb_state_lock);

	aicl_cur_control(usb_state);
	smb346_soc_detect_batt_tempr(usb_state);

	/* acquire battery rsoc here */
	if (get_battery_rsoc(&rsoc)) {
		dev_err(&smb345_dev->client->dev,
			" %s: fail to get battery rsoc\n", __func__);
	} else {

		if (rsoc == 5 || rsoc == 1)
			smb345_dump_registers(NULL);
	}

	if (dat)
		queue_delayed_work(smb345_dev->chrgr_work_queue,
			&smb345_dev->aicl_dete_work,
			30*HZ);

	led_control();
}
EXPORT_SYMBOL(aicl_dete_worker);

static void verifyFW()
{
	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return;
	}

	/* USB Mode Detection (by SOC) */

	/* Get USB to HC mode */
	if (smb345_get_USB9_HC_Toggle() < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to get USB9 and HC mode!\n", __func__);
		return;
	}

	/* Get USB5/1/HC to register control */
	if (smb345_get_USB9_HC_PIN_Control() < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to get USB9 and HC mode pin control!\n", __func__);
		return;
	}

#if 0
	/* Get AICL value */
	if (smb345_get_AICL() < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to get AICL mode!\n", __func__);
		return;
	}
#endif

	/* Get I_USB_IN value */
	if (smb345_get_current_limits() < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to get max current limits!\n", __func__);
		return;
	}
}

static void smb345_config_max_current_twinheadeddragon()
{
	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return;
	}

	/* check if ACOK# = 0 */
	if (smb345_dev->pdata->inok_gpio >= 0) {
		if (gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_err(&smb345_dev->client->dev,
				"%s: system input voltage is not valid >>> INOK pin (HIGH) <<<\n",
				__func__);

			return;
		}
	} else {

		dev_err(&smb345_dev->client->dev,
			"%s: negative gpio number: INOK!\n", __func__);
		return;
	}

	/* Allow violate register can be written - Write 30h[7]="1" */
	if (smb345_set_writable(smb345_dev, true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: smb345_set_writable failed!\n", __func__);
		return;
	}

	/* Disable AICL - Write 02h[4]="0" */
	if (smb345_OptiCharge_Toggle(false) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to disable AICL\n", __func__);
		return;
	}

	/* Set I_USB_IN=1800mA - 01h[3:0]="0110" */
	if (smb345_set_current_limits(USB_IN, true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to set max current limits for TWINSHEADED\n",
			__func__);
		return;
	}

	/* Set USB to HC mode - Write 31h[1:0]="11" */
	if (smb345_USB9_HC_Toggle(true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to enable USB9 and HC mode!\n",
			__func__);
		return;
	}

	/* Set USB5/1/HC to register control - Write 06h[4]="0" */
	if (smb345_USB9_HC_PIN_Control(false) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to disable USB9 and HC mode pin control!\n",
			__func__);
		return;
	}

#if 0
	/* Enable AICL - Write 02h[4]="1" */
	if (smb345_OptiCharge_Toggle(true) < 0) {
		dev_err(&smb345_dev->client->dev,
		"%s: fail to enable AICL\n", __func__);
		return;
	}
#endif

	smb345_soc_control_jeita();

	/* check if ACOK# = 0 */
	if (gpio_get_value(smb345_dev->pdata->inok_gpio)) {
		dev_err(&smb345_dev->client->dev,
		"%s: system input voltage is not valid after charge current settings\n",
		__func__);

		return;
	}

	pr_info("%s: charger type: TWINHEADEDDRAGON done.\n",
		__func__);
}

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
/* enable/disable AICL function */
static int smb346_Battery_OV_Toggle(bool on)
{
	int ret;

	ret = smb345_read(smb345_dev, CFG_VARIOUS_FUNCS);
	if (ret < 0)
		goto fail;

	if (on)
		ret |= CFG_VARIOUS_FUNCS_BATTERY_OV;
	else
		ret &= ~CFG_VARIOUS_FUNCS_BATTERY_OV;

	ret = smb345_write(smb345_dev, CFG_VARIOUS_FUNCS, ret);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}

static int smb346_config_charge_current(void)
{
	int ret;

	ret = smb345_write(smb345_dev,
			CFG_CHARGE_CURRENT,
			CFG_CHARGE_CURRENT_ALL);
	if (ret < 0)
		goto fail;

fail:
	return ret;
}

static int smb346_config_ccc(void)
{
	int ret;

	ret = smb345_masked_write(smb345_dev->client,
			OTG_TLIM_THERM_CNTRL_REG,
			CHARGE_CURRENT_COMPENSATION,
			CHARGE_CURRENT_COMPENSATION_VALUE);
	if (ret < 0)
		return ret;

	return ret;
}

static void smb346_pre_config(void)
{
	/* set battery OV does not end charge cycle */
	smb346_Battery_OV_Toggle(false);

	/* set fast charge current: 600mA
	       pre-charge current: 50mA
	       termination current: 50mA
	*/
	smb346_config_charge_current();

	/* set cold soft limit current: 125mA */
	smb346_config_ccc();
	return;
}
#else
static void smb346_pre_config(void)
{
	return;
}
#endif

#ifdef CONFIG_ME175CG
static int smb358_pre_config()
{
	int ret;

	/* set fast charge current: 2000mA */
	ret = smb345_masked_write(smb345_dev->client,
			CHG_CURRENT_REG,
			SMB358_FAST_CHG_CURRENT_MASK,
			SMB358_FAST_CHG_CURRENT_VALUE_2000mA);
	if (ret < 0)
		goto fail;

	/* set termination current: 200mA */
	ret = smb345_masked_write(smb345_dev->client,
			CHG_CURRENT_REG,
			SMB358_TERMINATION_CURRENT_MASK,
			SMB358_TERMINATION_CURRENT_VALUE_200mA);
	if (ret < 0)
		goto fail;

	/* set cold soft limit current: 600mA */
	ret = smb345_masked_write(smb345_dev->client,
			OTG_TLIM_THERM_CNTRL_REG,
			CHARGE_CURRENT_COMPENSATION,
			BIT(7));
	if (ret < 0)
		return ret;

fail:
	return ret;
}
#else
static int smb358_pre_config()
{
	return;
}
#endif
/*----------------------------------------------------------------------------*/
#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
static void smb3xx_config_max_current(int usb_state)
{
	int rsoc;

	/* USB Mode Detection (by SOC) */
	if ((usb_state == AC_IN) && (!g_init_disable_aicl)) {
		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			g_init_disable_aicl = false;
			return;
		}

		/* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" */
		if (smb345_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			g_init_disable_aicl = false;
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			g_init_disable_aicl = false;
			return;
		}
	} else if (usb_state == USB_IN) {


	} else if (usb_state == PAD_SUPPLY) {
		/* acquire battery rsoc here */
		if (get_battery_rsoc(&rsoc)) {
			dev_err(&smb345_dev->client->dev,
				" %s: fail to get battery rsoc\n", __func__);
			g_init_disable_aicl = false;
			return;
		} else
			dev_err(&smb345_dev->client->dev,
				" %s: get battery rsoc(%d)\n", __func__, rsoc);


		if (smb346_set_pad_supply_dc_in_limits(rsoc) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to config DC_IN limits\n", __func__);
		}
	}

	if (g_init_disable_aicl)
		dev_err(&smb345_dev->client->dev, "%s: not call disable/enable function when device init and AC-IN !!!\n", __func__);

	g_init_disable_aicl = false;
}
#elif defined(CONFIG_ME175CG)
static void smb3xx_config_max_current(int usb_state)
{
#if 0
	/* Set USB to HC mode - Write 31h[1:0]="11" */
	if (smb345_USB9_HC_Toggle(true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: fail to enable USB9 and HC mode!\n",
			__func__);
		return;
	}
#endif

	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}

		/* Set I_USB_IN=1200mA - Write 01h[7:4]="0100" */
		if (smb345_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			return;
		}

		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
	} else if (usb_state == USB_IN) {


	}
}
#elif defined(CONFIG_ME372CG) || defined(CONFIG_ME372CL) || defined(CONFIG_PF450CL)
static void smb3xx_config_max_current(int usb_state)
{
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		/* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" */
		if (smb345_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n", __func__);
			return;
		}

		/* Set USB to HC mode - Write 31h[1:0]="11" */
		if (smb345_USB9_HC_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable USB9 and HC mode!\n", __func__);
			return;
		}

		/* Set USB5/1/HC to register control - Write 06h[4]="0" */
		if (smb345_USB9_HC_PIN_Control(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable USB9 and HC mode pin control!\n", __func__);
			return;
		}
	} else if (usb_state == USB_IN) {

		/* Set I_USB_IN=500mA - Write 01h[3:0]="0001" */
		if (smb345_set_current_limits(USB_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n", __func__);
			return;
		}
	}
}
#elif defined(CONFIG_ME302C)
static void smb3xx_config_max_current(int usb_state)
{
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {
		/* Set I_USB_IN=1800mA - Write 01h[3:0]="0110" */
		if (smb345_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			return;
		}

		/* Set USB to HC mode - Write 31h[1:0]="11" */
		if (smb345_USB9_HC_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable USB9 and HC mode!\n",
				__func__);
			return;
		}

		/* Set USB5/1/HC to register control - Write 06h[4]="0" */
		if (smb345_USB9_HC_PIN_Control(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable USB9 and HC mode pin control!\n",
				__func__);
			return;
		}
	} else if (usb_state == USB_IN) {

		/* Set I_USB_IN=500mA - Write 01h[3:0]="0001" */
		if (smb345_set_current_limits(USB_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			return;
		}
	}
}
#else
static void smb3xx_config_max_current(int usb_state)
{
	/* USB Mode Detection (by SOC) */
	if (usb_state == AC_IN) {

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
		/* Disable AICL - Write 02h[4]="0" */
		if (smb345_OptiCharge_Toggle(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable AICL\n", __func__);
			return;
		}
#endif

		/* Set I_USB_IN=1200mA - Write 01h[3:0]="0100" */
		if (smb345_set_current_limits(AC_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			return;
		}

		/* Set USB to HC mode - Write 31h[1:0]="11" */
		if (smb345_USB9_HC_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable USB9 and HC mode!\n",
				__func__);
			return;
		}

#ifndef CONFIG_ME175CG
		/* Set USB5/1/HC to register control - Write 06h[4]="0" */
		if (smb345_USB9_HC_PIN_Control(false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to disable USB9 and HC mode pin control!\n",
				__func__);
			return;
		}
#endif

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
		/* Enable AICL - Write 02h[4]="1" */
		if (smb345_OptiCharge_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable AICL\n", __func__);
			return;
		}
#endif
	} else if (usb_state == USB_IN) {

#ifndef CONFIG_ME175CG
		/* Set I_USB_IN=500mA - Write 01h[3:0]="0001" */
		if (smb345_set_current_limits(USB_IN, false) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to set max current limits for USB_IN\n",
				__func__);
			return;
		}
#else
		/* Set USB to HC mode - Write 31h[1:0]="11" */
		if (smb345_USB9_HC_Toggle(true) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to enable USB9 and HC mode!\n",
				__func__);
			return;
		}
#endif
	} else if (usb_state == PAD_SUPPLY) {

#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
		/* acquire battery rsoc here */
		if (get_battery_rsoc(&rsoc)) {
			dev_err(&smb345_dev->client->dev,
				" %s: fail to get battery rsoc\n", __func__);
			return;
		} else
			dev_err(&smb345_dev->client->dev,
				" %s: get battery rsoc(%d)\n", __func__, rsoc);


		if (smb346_set_pad_supply_dc_in_limits(rsoc) < 0) {
			dev_err(&smb345_dev->client->dev,
				"%s: fail to config DC_IN limits\n", __func__);
		}
#endif
	}
}
#endif
/*----------------------------------------------------------------------------*/

static void smb345_config_max_current(int usb_state)
{
	if (usb_state != AC_IN && usb_state != USB_IN && usb_state != PAD_SUPPLY)
		return;

	if (!smb345_dev) {
		pr_err("%s: smb345_dev is null due to driver probed isn't ready\n",
			__func__);

		return;
	}

	/* check if ACOK# = 0 */
	if (smb345_dev->pdata->inok_gpio >= 0) {
		if (gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_err(&smb345_dev->client->dev,
				"%s: system input voltage is not valid >>> INOK pin (HIGH) <<<\n",
				__func__);

			return;
		}
	} else {

		dev_err(&smb345_dev->client->dev,
			"%s: negative gpio number: INOK!\n", __func__);
		return;
	}

	/* Allow violate register can be written - Write 30h[7]="1" */
	if (smb345_set_writable(smb345_dev, true) < 0) {
		dev_err(&smb345_dev->client->dev,
			"%s: smb345_set_writable failed!\n", __func__);
		return;
	}

	smb346_pre_config();
	smb358_pre_config();
	smb3xx_config_max_current(usb_state);
	smb345_soc_control_jeita();

	/* check if ACOK# = 0 */
	if (gpio_get_value(smb345_dev->pdata->inok_gpio)) {
		dev_err(&smb345_dev->client->dev,
			"%s: system input voltage is not valid after charge current settings\n",
			__func__);

		return;
	}

	pr_info("%s: charger type:%d done.\n", __func__, usb_state);
}

#if 0
static int smb345_proc_read(char *page, char **start, off_t off,
		int count, int *eof, void *date)
{
	int ret;

	if (!gbuffer)
		return count;

	/* print the key to screen for debugging */
	ret = sprintf(page, "%s\n", gbuffer);

	/* re-generate the key to clean the memory */
	generate_key();

	return ret;
}

static int smb345_proc_write(struct file *file, const char *buffer,
		unsigned long count, void *data)
{
	char proc_buf[64];

	if (count > sizeof(gbuffer)) {
		BAT_DBG("%s: data error\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(proc_buf, buffer, count)) {
		BAT_DBG("%s: read data from user space error\n", __func__);
		return -EFAULT;
	}

	if (!memcmp(proc_buf, gbuffer, 36)) {
		BAT_DBG_E("%s: EQ\n", __func__);

		if (smb345_dev && !gpio_get_value(smb345_dev->pdata->inok_gpio)) {

			/* ME372CG charge current control algorithm:
			   config the charge current only when
			   Vbus is legal (a valid input voltage
			   is present)
			*/
			mutex_lock(&g_usb_state_lock);
			dev_warn(&smb345_dev->client->dev,
		"%s: config current when TWINHEADEDDRAGON present!\n",
		__func__);
			/* force max 1.8A charge current for ME372CG */
			smb345_config_max_current_twinheadeddragon();
			mutex_unlock(&g_usb_state_lock);
		}

	} else {

		BAT_DBG_E("%s: NOT EQ\n", __func__);

		/* re-generate the key to clean memory to avoid
		   "brute-force attack". we do not allow someone
		   who using "repetitive try-error" to find out
		   the correct one.
		*/
		generate_key();
	}

	return count;
}

#ifndef CONFIG_NEW_PROC_FS
int smb345_proc_fs_current_control(void)
{
	struct proc_dir_entry *entry = NULL;

	entry = create_proc_entry("twinheadeddragon", 0666, NULL);
	if (!entry) {
		BAT_DBG_E("Unable to create twinheadeddragon\n");
		return -EINVAL;
	}
	entry->read_proc = smb345_proc_read;
	entry->write_proc = smb345_proc_write;

	return 0;
}
#else
static ssize_t nsmb345_proc_write(struct file *file,
		const char __user *buf, size_t count, loff_t *ppos)
{
	char proc_buf[64];

	if (count > sizeof(gbuffer)) {
		BAT_DBG("%s: data error\n", __func__);
		return -EINVAL;
	}

	if (copy_from_user(proc_buf, buf, count)) {
		BAT_DBG("%s: read data from user space error\n", __func__);
		return -EFAULT;
	}

	if (!memcmp(proc_buf, gbuffer, 36)) {
		BAT_DBG_E("%s: EQ\n", __func__);

		if (smb345_dev && !gpio_get_value(smb345_dev->pdata->inok_gpio)) {

			/* ME372CG charge current control algorithm:
			   config the charge current only when
			   Vbus is legal (a valid input voltage
			   is present)
			*/
			mutex_lock(&g_usb_state_lock);
			dev_warn(&smb345_dev->client->dev,
			"%s: config current when TWINHEADEDDRAGON present!\n",
			__func__);
			/* force max 1.8A charge current for ME372CG */
			smb345_config_max_current_twinheadeddragon();
			mutex_unlock(&g_usb_state_lock);
		}

	} else {

		BAT_DBG_E("%s: NOT EQ\n", __func__);

		/* re-generate the key to clean memory to avoid
		   "brute-force attack". we do not allow someone
		   who using "repetitive try-error" to find out
		   the correct one.
		*/
		generate_key();
	}

	return count;
}
static int nsmb345_proc_read(struct seq_file *m, void *v)
{
	if (!gbuffer)
		return 0;

	/* print the key to screen for debugging */
	seq_printf(m, "%s\n", gbuffer);

	/* re-generate the key to clean the memory */
	generate_key();

	return 0;
}
static int nsmb345_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, nsmb345_proc_read, NULL);
}
int smb345_proc_fs_current_control(void)
{
	static const struct file_operations proc_fs_current_control_fops = {
		.owner = THIS_MODULE,
		.open = nsmb345_proc_open,
		.read = seq_read,
		.write = nsmb345_proc_write,
		.llseek = seq_lseek,
		.release = single_release,
	};
	struct proc_dir_entry *entry = NULL;

	entry = proc_create("twinheadeddragon", 0666, NULL,
		&proc_fs_current_control_fops);
	if (!entry) {
		BAT_DBG_E("Unable to create twinheadeddragon\n");
		return -EINVAL;
	}

	return 0;
}
#endif
#else
int smb345_proc_fs_current_control(void)
{
	return 0;
}
#endif
#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG)
int pad_in_otg(int otg_state)
{
	int ret;
	switch (otg_state) {

	case ENABLE_5V:
		BAT_DBG(" %s: ENABLE_5V\n", __func__);
		ret = otg(1);
		break;
	case DISABLE_5V:
		BAT_DBG(" %s: DISABLE_5V\n", __func__);
		ret = otg(0);
		break;
	}
	return ret;
}
#endif

int setSMB345Charger(int usb_state)
{
	int ret = 0;
	mutex_lock(&g_usb_state_lock);
	g_pad_cable_state = usb_state;

	if (is_pad_present()) {

		usb_state = PAD_SUPPLY;
		g_usb_state = usb_state;
		BAT_DBG(" %s: Ignore usb_state: %d as PAD_SUPPLY\n",
			__func__, usb_state);
	} else if (g_usb_state != PAD_SUPPLY)
		g_usb_state = usb_state;


	mutex_unlock(&g_usb_state_lock);

	if (smb345_dev) {
		switch (usb_state) {
		case USB_IN:
			BAT_DBG_E("%s(),USB_IN\n", __func__);
			power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
			break;
		case AC_IN:
			BAT_DBG_E("%s(),AC_IN\n", __func__);
			power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			break;
		case CABLE_OUT:
			BAT_DBG_E("%s(),CABLE_OUT\n", __func__);
			power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			power_supply_changed(&smb345_power_supplies[CHARGER_USB-1]);
			break;
		case PAD_SUPPLY:
			if (g_pad_cable_state == AC_IN || g_pad_cable_state == CABLE_OUT)
				power_supply_changed(&smb345_power_supplies[CHARGER_AC-1]);
			break;
		}
	}

	switch (usb_state) {

	case USB_IN:
		BAT_DBG(" usb_state: USB_IN\n");
		usb_to_battery_callback(USB_PC);

		if (smb345_dev && !gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_warn(&smb345_dev->client->dev,
				"%s: >>> INOK pin (LOW) <<<\n", __func__);

			/* ME372CG charge current control algorithm:
			   config the charge current only when
			   Vbus is legal (a valid input voltage
			   is present)
			*/
			mutex_lock(&g_usb_state_lock);
			dev_warn(&smb345_dev->client->dev,
				"%s: config current when USB_IN\n", __func__);
			smb345_config_max_current(USB_IN);
			mutex_unlock(&g_usb_state_lock);
		}

		led_control();
		break;
	case AC_IN:
		BAT_DBG(" usb_state: AC_IN\n");
		usb_to_battery_callback(USB_ADAPTER);

		if (smb345_dev && !gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_warn(&smb345_dev->client->dev,
				"%s: >>> INOK pin (LOW) <<<\n", __func__);

			/* ME372CG charge current control algorithm:
			   config the charge current only when
			   Vbus is legal (a valid input voltage
			   is present)
			*/
			mutex_lock(&g_usb_state_lock);
			dev_warn(&smb345_dev->client->dev,
				"%s: config current when AC_IN\n", __func__);
			smb345_config_max_current(AC_IN);
			mutex_unlock(&g_usb_state_lock);
		}

#ifdef CONFIG_UPI_BATTERY
		if (smb345_dev) {
			if (entry_mode == 4) {
				if (!wake_lock_active(&wlock)) {
					BAT_DBG(" %s: asus_battery_power_wakelock -> wake lock\n",
						__func__);

					wake_lock(&wlock);
				}
			}
		}
#endif
		led_control();
		break;

	case CABLE_OUT:
		BAT_DBG(" usb_state: CABLE_OUT\n");
		usb_to_battery_callback(NO_CABLE);
#ifdef CONFIG_UPI_BATTERY
		if (smb345_dev) {
			if (entry_mode == 4) {
				if (wake_lock_active(&wlock)) {
					BAT_DBG(" %s: asus_battery_power_wakelock -> wake unlock\n",
						__func__);

					/* timeout value as same as the
					<charger.exe>\asus_global.h
					#define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
					*/
					wake_lock_timeout(&wlock_t, 3*HZ);
					wake_unlock(&wlock);
				} else {
					wake_lock_timeout(&wlock_t, 3*HZ);
				}
			}
		}
#endif
		led_control();
		break;

	case ENABLE_5V:
		BAT_DBG(" usb_state: ENABLE_5V\n");
		ret = otg(1);
		break;
	case DISABLE_5V:
		BAT_DBG(" usb_state: DISABLE_5V\n");
		ret = otg(0);
		break;
	case PAD_SUPPLY:
		BAT_DBG(" usb_state: PAD_SUPPLY\n");
		if (smb345_dev && !gpio_get_value(smb345_dev->pdata->inok_gpio)) {
			dev_warn(&smb345_dev->client->dev,
				"%s: >>> INOK pin (LOW) <<<\n", __func__);

			/* ME372CG charge current control algorithm:
			   config the charge current only when
			   Vbus is legal (a valid input voltage
			   is present)
			*/
			mutex_lock(&g_usb_state_lock);
			dev_warn(&smb345_dev->client->dev,
				"%s: config current when PAD_SUPPLY\n", __func__);
			smb345_config_max_current(PAD_SUPPLY);
			mutex_unlock(&g_usb_state_lock);
		}
		break;

	default:
		BAT_DBG(" ERROR: wrong usb state value = %d\n", usb_state);
		ret = 1;
	}

	return ret;
}
EXPORT_SYMBOL(setSMB345Charger);


/* write 06h[6:5]="00" or "11" */
int smb345_charging_toggle(charging_toggle_level_t level, bool on)
{
	int ret = 0;
	int charging_toggle;
	static charging_toggle_level_t old_lvl = JEITA;
	char *level_str[] = {
		"BALANCE",
		"JEITA",
		"FLAGS",
	};

	if (!smb345_dev) {
		pr_info("Warning: smb345_dev is null "
			"due to probe function has error\n");
		return 1;
	}

	mutex_lock(&g_charging_toggle_lock);
	charging_toggle = g_charging_toggle;
	mutex_unlock(&g_charging_toggle_lock);

	BAT_DBG("%s: old_lvl:%s, charging_toggle:%s, level:%s, on:%s\n",
		__func__,
		level_str[old_lvl],
		charging_toggle ? "YES" : "NO",
		level_str[level],
		on ? "YES" : "NO");

	/* do charging or not? */
	if (level != FLAGS) {
		if (on) {
			/* want to start charging? */
			if (level == JEITA) {
				if (!charging_toggle) {
					/* want to restart charging? */
					if (old_lvl != JEITA) {
						/* reject the request! someone stop charging before */
						BAT_DBG_E("%s: * reject RESTART charging to phone! *\n",
							__func__);
						return -1;
					}
				}
			} else if (level == BALANCE) {
				if (!charging_toggle) {
					/* want to restart charging? */
					if (old_lvl != BALANCE) {
						/* reject the request! someone stop charging before */
						BAT_DBG_E("%s: * reject RESTART charging to phone! *\n",
							__func__);
						return -1;
					}
				}
			} else {

				/* what the hell are you? */
			}
		} else {

			/* want to stop charging? just do it! */
		}
	 } else {

		/* it's the highest level. just do it! */
	}

	/* level value assignment */
	old_lvl = level;

	if (!on)
		BAT_DBG_E(" %s: *** charging toggle: OFF ***\n",
			__func__);
	else
		BAT_DBG(" %s: --------------- charging toggle: ON ---------------\n",
			__func__);

	ret = smb345_set_writable(smb345_dev, true);
	if (ret < 0)
		return ret;

	/* Config CFG_PIN register */

	ret = smb345_read(smb345_dev, CFG_PIN);
	if (ret < 0)
		goto out;

	/*
	 * Make the charging functionality controllable by a write to the
	 * command register unless pin control is specified in the platform
	 * data.
	 */
	ret &= ~CFG_PIN_EN_CTRL_MASK;
	if (on) {
		/* set Pin Controls - active low (ME371MG connect EN to GROUND) */
		ret |= CFG_PIN_EN_CTRL_ACTIVE_LOW;
	} else {
		/* Do nothing, 0 means i2c control
		    . I2C Control - "0" in Command Register disables charger */
	}

	ret = smb345_write(smb345_dev, CFG_PIN, ret);
	if (ret < 0)
		goto out;

	mutex_lock(&g_charging_toggle_lock);
	g_charging_toggle = on;
	mutex_unlock(&g_charging_toggle_lock);

#if 0
	/* Config CMD_A register */

	ret = smb345_read(smb345_dev, CMD_A);
	if (ret < 0)
		goto out;



	if (on)
		ret |= CMD_A_CHG_ENABLED;
	else
		ret &= ~CMD_A_CHG_ENABLED;

	ret = smb345_write(smb345_dev, CMD_A, ret);
#endif
out:
	return ret;
}

#if 0
static irqreturn_t smb345_interrupt(int irq, void *data)
{
	struct smb345_charger *smb = data;
	int stat_c, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	irqreturn_t ret = IRQ_NONE;

	pm_runtime_get_sync(&smb->client->dev);
	dev_warn(&smb->client->dev, "%s\n", __func__);

	stat_c = smb345_read(smb, STAT_C);
	if (stat_c < 0) {
		dev_warn(&smb->client->dev, "reading STAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_c = smb345_read(smb, IRQSTAT_C);
	if (irqstat_c < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_C failed\n");
		return IRQ_NONE;
	}

	irqstat_d = smb345_read(smb, IRQSTAT_D);
	if (irqstat_d < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_D failed\n");
		return IRQ_NONE;
	}

	irqstat_e = smb345_read(smb, IRQSTAT_E);
	if (irqstat_e < 0) {
		dev_warn(&smb->client->dev, "reading IRQSTAT_E failed\n");
		return IRQ_NONE;
	}

	irqstat_f = smb345_read(smb, IRQSTAT_F);
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
			"****** charging stopped due to charger error ******\n");

		ret = IRQ_HANDLED;
	}

	/*
	 * If we reached the termination current the battery is charged and
	 * we can update the status now. Charging is automatically
	 * disabled by the hardware.
	 */
	if (irqstat_c & (IRQSTAT_C_TERMINATION_IRQ | IRQSTAT_C_TAPER_IRQ)) {
		if ((irqstat_c & IRQSTAT_C_TERMINATION_STAT) &&
			smb->pdata->show_battery)
			dev_info(&smb->client->dev,
				"[Charge Terminated] Going to HW Maintenance mode\n");
		ret = IRQ_HANDLED;
	}

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
		ret = IRQ_HANDLED;
	}

	/*
	 * If we got an under voltage interrupt it means that AC/USB input
	 * was connected or disconnected.
	 */
	if (irqstat_e & (IRQSTAT_E_USBIN_UV_IRQ | IRQSTAT_E_DCIN_UV_IRQ)) {



		if (smb->mains_online || smb->usb_online)
			dev_info(&smb->client->dev, "Charger connected\n");
		else
			dev_info(&smb->client->dev, "Charger disconnected\n");

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
		ret = IRQ_HANDLED;
	}

	pm_runtime_put_sync(&smb->client->dev);
	return ret;
}
#endif

static int smb345_irq_set(struct smb345_charger *smb, bool enable)
{
	int ret;

#if 0
	ret = smb345_set_writable(smb, true);
	if (ret < 0)
		return ret;
#endif

	/*
	 * Enable/disable interrupts for:
	 *    - under voltage
	 *    - termination current reached
	 *    - charger error
	 */
	if (enable) {
#if 0
		int val = CFG_FAULT_IRQ_DCIN_UV;

		if (smb->otg)
			val |= CFG_FAULT_IRQ_OTG_UV;

		ret = smb345_write(smb, CFG_FAULT_IRQ, val);
		if (ret < 0)
			goto fail;

		val = CFG_STATUS_IRQ_CHARGE_TIMEOUT |
		      CFG_STATUS_IRQ_TERMINATION_OR_TAPER;
		ret = smb345_write(smb, CFG_STATUS_IRQ, val);
		if (ret < 0)
			goto fail;
#endif

		ret = smb345_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret |= CFG_PIN_EN_CHARGER_ERROR;

		ret = smb345_write(smb, CFG_PIN, ret);
	} else {
#if 0
		ret = smb345_write(smb, CFG_FAULT_IRQ, 0);
		if (ret < 0)
			goto fail;

		ret = smb345_write(smb, CFG_STATUS_IRQ, 0);
		if (ret < 0)
			goto fail;
#endif

		ret = smb345_read(smb, CFG_PIN);
		if (ret < 0)
			goto fail;

		ret &= ~CFG_PIN_EN_CHARGER_ERROR;

		ret = smb345_write(smb, CFG_PIN, ret);
	}

fail:
#if 0
	smb345_set_writable(smb, false);
#endif
	return ret;
}

bool external_power_source_present()
{
	return !gpio_get_value(get_gpio_by_name("CHG_INOK#"));
}
#if defined(CONFIG_PF400CG)
static bool is_pad_present()
{
	bool ret;

	ret = !gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N"));

	return ret;
}
static bool pad_pwr_supply()
{
	bool ret;

	/* return TRUE if gpio for pad insertion detected
	   and charger received the valid external power
	*/
	ret = !gpio_get_value(get_gpio_by_name("PAD_PLUG_IN_N")) &&
		!gpio_get_value(get_gpio_by_name("CHG_INOK#"));
	return ret;
}
#else
static bool pad_pwr_supply()
{
	return false;
}
static bool is_pad_present()
{
	return false;
}
#endif
static irqreturn_t smb345_inok_interrupt(int irq, void *data)
{
	struct smb345_charger *smb = data;
	int stat_c, irqstat_c, irqstat_d, irqstat_e, irqstat_f;
	irqreturn_t ret = IRQ_NONE;
	int charger_type;

	/* wake lock to prevent system instantly
	   enter S3 while it's in resuming flow */
	wake_lock_timeout(&smb->wakelock, HZ);
	pm_runtime_get_sync(&smb->client->dev);

	if (gpio_get_value(smb->pdata->inok_gpio)) {
		dev_warn(&smb->client->dev,
			"%s: >>> INOK pin (HIGH) <<<\n", __func__);

		mutex_lock(&g_usb_state_lock);
		g_usb_state = CABLE_OUT;
		mutex_unlock(&g_usb_state_lock);

		/* reset to default as missing external power source */
		mutex_lock(&g_charging_toggle_lock);
		if (!is_pad_present())
			g_charging_toggle = true;
		mutex_unlock(&g_charging_toggle_lock);
	} else {

		dev_warn(&smb->client->dev,
			"%s: >>> INOK pin (LOW) <<<\n", __func__);

		/* ME372CG charge current control algorithm:
		   config the charge current only when
		   Vbus is legal (a valid input voltage
		   is present)
		*/
		mutex_lock(&g_usb_state_lock);
		if (pad_pwr_supply()) {
			g_usb_state = PAD_SUPPLY;
			charger_type = PAD_SUPPLY;
		} else
			charger_type = g_usb_state;

		dev_warn(&smb->client->dev,
			"%s: config current when inok interrupt\n", __func__);
		smb345_config_max_current(charger_type);
		mutex_unlock(&g_usb_state_lock);
	}

	pm_runtime_put_sync(&smb->client->dev);
	return IRQ_HANDLED;
}

#if defined(CONFIG_PF400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_A400CG)
static int smb346_otg_gpio_init(struct smb345_charger *smb)
{
	const struct smb345_charger_platform_data *pdata = smb->pdata;
	int ret;

	if (pdata->gp_sdio_2_clk < 0) {
		BAT_DBG_E("%s: fail to request CHG_OTG gpio number\n", __func__);
		return -1;
	}

	ret = gpio_request_one(pdata->gp_sdio_2_clk,
			GPIOF_OUT_INIT_LOW,
			"smb346_otg");
	if (ret < 0)
		BAT_DBG_E("%s: request CHG_OTG gpio fail!\n", __func__);

	return ret;
}
#else
static int smb346_otg_gpio_init(struct smb345_charger *smb)
{
	return 0;
}
#endif
static int smb345_inok_gpio_init(struct smb345_charger *smb)
{
	const struct smb345_charger_platform_data *pdata = smb->pdata;
	int ret, irq = gpio_to_irq(pdata->inok_gpio);
	int gpio_usb0_id;

#ifdef CHRIS
	gpio_usb0_id = get_gpio_by_name("USB0_ID_LS");
	if (gpio_get_value(gpio_usb0_id))
		BAT_DBG(">>> USB0_ID (HIGH) <<<\n");
	else
		BAT_DBG(">>> USB0_ID (LOW) <<<\n");
#endif
	if (gpio_get_value(pdata->inok_gpio))
		BAT_DBG(">>> INOK (HIGH) <<<\n");
	else
		BAT_DBG(">>> INOK (LOW) <<<\n");

	ret = gpio_request_one(pdata->inok_gpio, GPIOF_IN, "smb345_inok");
	if (ret < 0) {
		BAT_DBG_E("%s: request INOK gpio fail!\n", __func__);
		goto fail;
	}

	ret = request_threaded_irq(irq, NULL, smb345_inok_interrupt,
			IRQF_PERCPU | IRQF_NO_SUSPEND | IRQF_FORCE_RESUME |
			IRQF_ONESHOT |
			IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
			smb->client->name,
			smb);

	if (ret < 0) {
		BAT_DBG_E("%s: config INOK gpio as IRQ fail!\n", __func__);
		goto fail_gpio;
	}

	return 0;
fail_gpio:
	gpio_free(pdata->inok_gpio);
fail:
	smb->client->irq = 0;
	return ret;
}

static inline int smb345_irq_enable(struct smb345_charger *smb)
{
	return smb345_irq_set(smb, true);
}

static inline int smb345_irq_disable(struct smb345_charger *smb)
{
	return smb345_irq_set(smb, false);
}

static int smb345_irq_init(struct smb345_charger *smb)
{
	const struct smb345_charger_platform_data *pdata = smb->pdata;
	int ret;
#if 0
	int ret, irq = gpio_to_irq(pdata->irq_gpio);

	ret = gpio_request_one(pdata->irq_gpio, GPIOF_IN, smb->client->name);
	if (ret < 0)
		goto fail;

	ret = request_threaded_irq(irq, NULL, smb345_interrupt,
		IRQF_TRIGGER_FALLING, smb->client->name,
		smb);
	ret = request_threaded_irq(irq, NULL, smb345_interrupt,
		IRQF_TRIGGER_FALLING | IRQF_TRIGGER_RISING,
		smb->client->name,
		smb);
	if (ret < 0)
		goto fail_gpio;
#endif

	ret = smb345_set_writable(smb, true);
	if (ret < 0)
		goto fail;

	/*
	 * Configure the STAT output to be suitable for interrupts: disable
	 * all other output (except interrupts) and make it active low.
	 */
	ret = smb345_read(smb, CFG_STAT);
	if (ret < 0)
		goto fail_readonly;

	ret &= ~CFG_STAT_ACTIVE_HIGH;
	ret |= CFG_STAT_DISABLED;

	ret = smb345_write(smb, CFG_STAT, ret);
	if (ret < 0)
		goto fail_readonly;

	ret = smb345_irq_enable(smb);
	if (ret < 0)
		goto fail_readonly;

#if 0
	smb345_set_writable(smb, false);
	smb->client->irq = irq;
	enable_irq_wake(smb->client->irq);
#endif
	return 0;

fail_readonly:
#if 0
	smb345_set_writable(smb, false);
fail_irq:
	free_irq(irq, smb);
fail_gpio:
	gpio_free(pdata->irq_gpio);
#endif
fail:
	smb->client->irq = 0;
	return ret;
}

bool smb345_has_charger_error(void)
{
	int ret, status;

	if (!smb345_dev)
		return -EINVAL;

	ret = smb345_read(smb345_dev, STAT_C);
	if (ret < 0)
		return true;

	if (ret & STAT_C_CHARGER_ERROR)
		return true;

	return false;
}

int smb345_get_charging_status(void)
{
	int ret, status;
	int irqstat_c;

	if (!smb345_dev)
		return -EINVAL;

	ret = smb345_read(smb345_dev, STAT_C);
	if (ret < 0)
		return ret;

	irqstat_c = smb345_read(smb345_dev, IRQSTAT_C);
	if (irqstat_c < 0)
		return irqstat_c;





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

		if (irqstat_c & IRQSTAT_C_TERMINATION_STAT)
			status = POWER_SUPPLY_STATUS_FULL;
	}

	return status;
}


#ifndef ME372CG_USER_BUILD
int smb345_dump_registers(struct seq_file *s)
{
	struct smb345_charger *smb;
	int ret;
	u8 reg;

	if (s) {
		smb = s->private;
	} else {

		if (!smb345_dev) {
			BAT_DBG(" %s: smb345_dev is null!\n",
			__func__);
			return -1;
		} else {

			smb = smb345_dev;
		}
	}

	BAT_DBG(" %s:\n", __func__);
	BAT_DBG(" Control registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");

	for (reg = CFG_CHARGE_CURRENT; reg <= CFG_ADDRESS; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Command registers:\n");
	BAT_DBG(" ==================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Command registers:\n");
		seq_printf(s, "==================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}

	ret = smb345_read(smb, CMD_A);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_A, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", CMD_A, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_B);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
			"\n", CMD_B, BYTETOBINARY(ret));
	ret = smb345_read(smb, CMD_C);
	BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
		"\n", CMD_C, BYTETOBINARY(ret));
	if (s)
		seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", CMD_C, BYTETOBINARY(ret));
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Interrupt status registers:\n");
	BAT_DBG(" ===========================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Interrupt status registers:\n");
		seq_printf(s, "===========================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = IRQSTAT_A; reg <= IRQSTAT_F; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}
	BAT_DBG("\n");
	if (s)
		seq_printf(s, "\n");

	BAT_DBG(" Status registers:\n");
	BAT_DBG(" =================\n");
	BAT_DBG(" #Addr\t#Value\n");
	if (s) {
		seq_printf(s, "Status registers:\n");
		seq_printf(s, "=================\n");
		seq_printf(s, "#Addr\t#Value\n");
	}
	for (reg = STAT_A; reg <= STAT_E; reg++) {
		ret = smb345_read(smb, reg);
		BAT_DBG(" 0x%02x:\t" BYTETOBINARYPATTERN
			"\n", reg, BYTETOBINARY(ret));
		if (s)
			seq_printf(s, "0x%02x:\t" BYTETOBINARYPATTERN
				"\n", reg, BYTETOBINARY(ret));
	}

	return 0;
}
#else
int smb345_dump_registers(struct seq_file *s)
{
	return 0;
}
#endif

static int smb345_debugfs_show(struct seq_file *s, void *data)
{
	seq_printf(s, "Control registers:\n");
	seq_printf(s, "==================\n");
	seq_printf(s, "#Addr\t#Value\n");

	smb345_dump_registers(s);

	return 0;
}

static int smb345_debugfs_open(struct inode *inode, struct file *file)
{
	return single_open(file, smb345_debugfs_show, inode->i_private);
}

static const struct file_operations smb345_debugfs_fops = {
	.open	= smb345_debugfs_open,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};

#ifdef CONFIG_ME372CG_PMIC_SNB5072C1
static int smb345_debugfs_show_pmic(struct seq_file *s,
		void *data)
{
	seq_printf(s, "PMIC battery-related registers:\n");
	seq_printf(s, "===============================\n");
	seq_printf(s, "#Addrs   #Value       #Name\n");

	pmic_dump_registers(MSIC_CHRG_REG_DUMP_INT |
			MSIC_CHRG_REG_DUMP_BOOT	|
			MSIC_CHRG_REG_DUMP_EVENT |
			MSIC_CHRG_REG_DUMP_OTHERS,
			s);

	return 0;
}

static int smb345_debugfs_open_pmic(struct inode *inode,
		struct file *file)
{
	return single_open(file,
			smb345_debugfs_show_pmic,
			inode->i_private);
}

static const struct file_operations smb345_debugfs_fops2 = {
	.open	= smb345_debugfs_open_pmic,
	.read	= seq_read,
	.llseek	= seq_lseek,
	.release	= single_release,
};
#endif

#if defined(CONFIG_PF400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_A400CG)
static int smb346_routine_aicl_control()
{
	BAT_DBG(" %s\n", __func__);

	INIT_DELAYED_WORK(&smb345_dev->aicl_dete_work,
		aicl_dete_worker);
	smb345_dev->chrgr_work_queue =
			create_singlethread_workqueue("smb346_wq");
	if (!smb345_dev->chrgr_work_queue) {
		BAT_DBG_E(" fail to create \"smb346_wq\"");
		return -ENOMEM;
	}

	queue_delayed_work(smb345_dev->chrgr_work_queue,
			&smb345_dev->aicl_dete_work,
			60*HZ);

	return 0;
}

static inline struct power_supply *get_psy_battery(void)
{
	struct class_dev_iter iter;
	struct device *dev;
	static struct power_supply *pst;

	class_dev_iter_init(&iter, power_supply_class, NULL, NULL);
	while ((dev = class_dev_iter_next(&iter))) {
		pst = (struct power_supply *)dev_get_drvdata(dev);
		if (pst->type == POWER_SUPPLY_TYPE_BATTERY) {
			class_dev_iter_exit(&iter);
			return pst;
		}
	}
	class_dev_iter_exit(&iter);

	return NULL;
}

static inline int get_battery_temperature(int *tempr)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_TEMP_AMBIENT, &val);
	if (!ret)
		*tempr = val.intval;

	return ret;
}

static inline int get_battery_voltage(int *volt)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_VOLTAGE_NOW, &val);
	if (!ret)
		*volt = val.intval / 1000;

	return ret;
}

static inline int get_battery_rsoc(int *rsoc)
{
	struct power_supply *psy;
	union power_supply_propval val;
	int ret;

	psy = get_psy_battery();
	if (!psy)
		return -EINVAL;

	ret = psy->get_property(psy, POWER_SUPPLY_PROP_CAPACITY, &val);
	if (!ret)
		*rsoc = val.intval;

	return ret;
}

static int smb346_soc_detect_batt_tempr(int usb_state)
{
	int ret;
	int batt_tempr = 250;/* unit: C  */
	int batt_volt = 4000;/* unit: mV */
	int high_tempr_thrsd = STOP_CHARGING_HIGH_TEMPERATURE;
	int low_tempr_thrsd = STOP_CHARGING_LOW_TEMPERATURE;
	int rsoc;

	if (usb_state != AC_IN && usb_state != USB_IN && usb_state != PAD_SUPPLY)
		return 0;

	/* acquire battery temperature here */
	ret = get_battery_temperature(&batt_tempr);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery temperature\n", __func__);
		return ret;
	} else
		BAT_DBG(" %s: get battery temperature(%d)\n", __func__, batt_tempr);


#ifdef CONFIG_ME175CG
	/* acquire battery voltage here */
	ret = get_battery_voltage(&batt_volt);
	if (ret) {
		BAT_DBG_E(" %s: fail to get battery voltage\n", __func__);
		return ret;
	} else
		BAT_DBG(" %s: get battery voltage(%d)\n", __func__, batt_volt);

#endif

	ret = smb345_soc_control_float_vol(batt_tempr);
	if (ret)
		return ret;

	if (batt_tempr < low_tempr_thrsd) {
		ret = smb345_charging_toggle(JEITA, false);
		request_power_supply_changed();
		return ret;
	} else if (batt_tempr > high_tempr_thrsd) {

		ret = smb345_charging_toggle(JEITA, false);
		request_power_supply_changed();
		return ret;
	}
#if defined(CONFIG_PF400CG) || defined(CONFIG_A400CG) || defined(CONFIG_ME175CG)
	else if (batt_tempr >= FLOAT_VOLTAGE_TEMPERATURE_THRESHOLD && batt_volt > 4110) {
		ret = smb345_charging_toggle(JEITA, false);
		request_power_supply_changed();
		return ret;
	}
#endif
	else{

#if defined(ME372CG_ENG_BUILD) && defined(CONFIG_UPI_BATTERY)
		/* acquire battery rsoc here */
		if (get_battery_rsoc(&rsoc)) {
			BAT_DBG(" %s: fail to get battery rsoc\n", __func__);
		} else {

			if (rsoc > 59 && eng_charging_limit) {
				ret = smb345_charging_toggle(JEITA, false);
				request_power_supply_changed();
				return ret;
			}
		}
#endif

#ifndef CONFIG_PF400CG
		ret = smb345_charging_toggle(JEITA, true);
#else
		if (usb_state == AC_IN || usb_state == USB_IN) {
			ret = smb345_charging_toggle(JEITA, true);
			ret = smb345_charging_toggle(BALANCE, true);
		}
#endif
	}

	return ret;
}
#else
static inline int get_battery_rsoc(int *rsoc)
{
	return -1;
}
static int smb346_routine_aicl_control()
{
	return 0;
}
static int smb346_soc_detect_batt_tempr(int usb_state)
{
	return 0;
}
#endif

static int smb345_probe(struct i2c_client *client,
		const struct i2c_device_id *id)
{
	const struct smb345_charger_platform_data *pdata;
	struct device *dev = &client->dev;
	struct smb345_charger *smb;
	int ret;
	int charger_type;

	BAT_DBG_E(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);

	pdata = dev->platform_data;
	if (!pdata)
		return -EINVAL;

	if (!pdata->use_mains && !pdata->use_usb)
		return -EINVAL;

	smb = devm_kzalloc(dev, sizeof(*smb), GFP_KERNEL);
	if (!smb)
		return -ENOMEM;

	i2c_set_clientdata(client, smb);

	smb->client = client;
	smb->pdata = pdata;

#ifdef CONFIG_UPI_BATTERY
	/* init wake lock in COS */
	if (entry_mode == 4) {
		BAT_DBG(" %s: wake lock init: asus_battery_power_wakelock\n",
			__func__);
		wake_lock_init(&wlock,
			WAKE_LOCK_SUSPEND,
			"asus_battery_power_wakelock");
		wake_lock_init(&wlock_t,
			WAKE_LOCK_SUSPEND,
			"asus_battery_power_wakelock_timeout");

		/* prevent system from entering s3 in COS
		    while AC charger is connected
		*/
		mutex_lock(&g_usb_state_lock);
		charger_type = g_usb_state;
		mutex_unlock(&g_usb_state_lock);

		if (charger_type == AC_IN) {
			if (!wake_lock_active(&wlock)) {
				BAT_DBG(" %s: asus_battery_power_wakelock "
					"-> wake lock\n", __func__);
				wake_lock(&wlock);
			}
		}
	}
#endif
	smb345_dev = smb;
	smb345_dump_registers(NULL);

	/* enable register writing - chris */
	ret = smb345_set_writable(smb, true);
	if (ret < 0)
		return ret;

	/* Refer to SMB345 Application Note 72 to solve serious problems */
	ret = smb345_masked_write(smb->client,
				OTG_TLIM_THERM_CNTRL_REG,
				OTG_CURRENT_LIMIT_AT_USBIN_MASK,
				OTG_CURRENT_LIMIT_250mA);
	if (ret < 0)
		return ret;

	/* Init Runtime PM State */
	pm_runtime_put_noidle(&smb->client->dev);
	pm_schedule_suspend(&smb->client->dev, MSEC_PER_SEC);


#if 0
	/*
	 * Interrupt pin is optional. If it is connected, we setup the
	 * interrupt support here.
	 */
	if (pdata->irq_gpio >= 0) {
		ret = smb345_irq_init(smb);
		if (ret < 0) {
			dev_warn(dev, "failed to initialize IRQ: %d\n", ret);
			dev_warn(dev, "disabling IRQ support\n");
		}
	}
#endif

	/* INOK pin configuration */
	if (pdata->inok_gpio >= 0) {
		ret = smb345_inok_gpio_init(smb);
		if (ret < 0) {
			dev_warn(dev,
				"fail to initialize INOK gpio: %d\n",
				ret);
		}
	}

	/* OTG pin configuration */
	ret = smb346_otg_gpio_init(smb);
	if (ret < 0)
		return ret;

	smb->running = true;
	smb->dentry = debugfs_create_file("smb", S_IRUGO, NULL, smb,
			&smb345_debugfs_fops);
#ifdef CONFIG_ME372CG_PMIC_SNB5072C1
	smb->dentry = debugfs_create_file("pmic", S_IRUGO, NULL, smb,
						&smb345_debugfs_fops2);
#endif
	sysfs_create_group(&client->dev.kobj, &dev_attr_grp);

	ret = smb345_register_power_supply(&client->dev);
	if (ret < 0)
		return ret;

#if 0
	ret = smb346_routine_aicl_control();
	if (ret < 0)
		return ret;
#endif

	BAT_DBG_E(" ++++++++++++++++ %s done ++++++++++++++++\n", __func__);

	/* init wake lock */
	wake_lock_init(&smb->wakelock,
			WAKE_LOCK_SUSPEND, "smb345_wakelock");

	/* ME372CG charge current control algorithm:
	   config the charge current only when
	   Vbus is legal (a valid input voltage
	   is present)
	*/
	mutex_lock(&g_usb_state_lock);
	charger_type = g_usb_state;
	verifyFW();
	dev_warn(&smb345_dev->client->dev,
			"%s: config current when probe\n", __func__);
	smb345_config_max_current(charger_type);
	mutex_unlock(&g_usb_state_lock);

	generate_key();
	smb345_proc_fs_current_control();

#ifdef CONFIG_ME372CG_PMIC_SNB5072C1
	pmic_dump_registers(MSIC_CHRG_REG_DUMP_INT |
			MSIC_CHRG_REG_DUMP_BOOT |
			MSIC_CHRG_REG_DUMP_EVENT |
			MSIC_CHRG_REG_DUMP_OTHERS,
			NULL);
#endif
#if 0
	smb345_tbl.soc_control_float_vol = smb345_soc_control_float_vol;
	smb345_tbl.charging_toggle = smb345_charging_toggle;
	smb345_tbl.dump_registers = smb345_dump_registers;
	asus_register_power_supply_charger(dev, &smb345_tbl);
#endif
#ifdef CONFIG_UPI_BATTERY
	ret = init_asus_charging_limit_toggle();
	if (ret) {
		BAT_DBG_E("Unable to create proc file\n");
		return ret;
	}
#endif

#ifdef CONFIG_ME372CG_PMIC_SNB5072C1
	BAT_DBG(" %s: battery id: %d\n", __func__, pmic_get_batt_id());
#endif
	return 0;
}

static int smb345_remove(struct i2c_client *client)
{
	struct smb345_charger *smb = i2c_get_clientdata(client);

	if (!IS_ERR_OR_NULL(smb->dentry))
		debugfs_remove(smb->dentry);

	smb->running = false;

#if 0
	if (client->irq) {
		smb345_irq_disable(smb);
		free_irq(client->irq, smb);
		gpio_free(smb->pdata->irq_gpio);
	}
#endif

	wake_lock_destroy(&smb->wakelock);
	pm_runtime_get_noresume(&smb->client->dev);

	return 0;
}

static int smb345_shutdown(struct i2c_client *client)
{
	dev_info(&client->dev, "%s\n", __func__);





	/* JEITA function by charger protection */
	mutex_lock(&g_usb_state_lock);
	if (g_usb_state == AC_IN || g_usb_state == USB_IN || g_usb_state == PAD_SUPPLY)
		smb345_charger_control_jeita();
	mutex_unlock(&g_usb_state_lock);

	/* Disable OTG during shutdown */
	otg(0);

	/* registers dump */
	smb345_dump_registers(NULL);

	return 0;
}

#ifdef CONFIG_PM
static int smb345_prepare(struct device *dev)
{
	struct smb345_charger *smb = dev_get_drvdata(dev);

	dev_info(&smb->client->dev, "smb345 suspend\n");
	/*
	 * disable irq here doesn't mean smb345 interrupt
	 * can't wake up system. smb345 interrupt is triggered
	 * by GPIO pin, which is always active.
	 * When resume callback calls enable_irq, kernel
	 * would deliver the buffered interrupt (if it has) to
	 * driver.
	 */
	if (smb->client->irq > 0)
		disable_irq(smb->client->irq);

	/* JEITA function by charger protection */
	mutex_lock(&g_usb_state_lock);
	if (g_usb_state == AC_IN || g_usb_state == USB_IN || g_usb_state == PAD_SUPPLY)
		smb345_charger_control_jeita();
	mutex_unlock(&g_usb_state_lock);

	return 0;
}

static void smb345_complete(struct device *dev)
{
	struct smb345_charger *smb = dev_get_drvdata(dev);

	dev_info(&smb->client->dev, "smb345 resume\n");
}
#else
#define smb345_prepare NULL
#define smb345_complete NULL
#endif

#ifdef CONFIG_PM_RUNTIME
static int smb345_runtime_suspend(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb345_runtime_resume(struct device *dev)
{
	dev_info(dev, "%s called\n", __func__);
	return 0;
}

static int smb345_runtime_idle(struct device *dev)
{

	dev_info(dev, "%s called\n", __func__);
	return 0;
}
#else
#define smb345_runtime_suspend    NULL
#define smb345_runtime_resume    NULL
#define smb345_runtime_idle    NULL
#endif

static const struct i2c_device_id smb345_id[] = {
#if defined(CONFIG_PF450CL)
	{"smb358", 0},
#else
	{"smb345", 0},
#endif
	{},
};
MODULE_DEVICE_TABLE(i2c, smb345_id);

static const struct dev_pm_ops smb345_pm_ops = {
	.prepare        = smb345_prepare,
	.complete        = smb345_complete,
	.runtime_suspend    = smb345_runtime_suspend,
	.runtime_resume        = smb345_runtime_resume,
	.runtime_idle        = smb345_runtime_idle,
};

static struct i2c_driver smb345_driver = {
	.driver = {
#if defined(CONFIG_PF450CL)
		.name    = "smb358",
#else
		.name    = "smb345",
#endif
		.owner    = THIS_MODULE,
		.pm    = &smb345_pm_ops,
	},
	.probe        = smb345_probe,
	.remove        = smb345_remove,
	.shutdown    = smb345_shutdown,
	.id_table    = smb345_id,
};

static int __init smb345_init(void)
{
	BAT_DBG_E(" ++++++++++++++++ %s ++++++++++++++++\n", __func__);
	if (entry_mode == 5)
		return -1;

	HW_ID = Read_HW_ID();
#ifdef CONFIG_ME175CG
	if (HW_ID == 0)
		return -EAGAIN;
#endif
	return i2c_add_driver(&smb345_driver);
}
module_init(smb345_init);

static void __exit smb345_exit(void)
{
	i2c_del_driver(&smb345_driver);
}
module_exit(smb345_exit);

MODULE_AUTHOR("Bruce E. Robertson <bruce.e.robertson@intel.com>");
MODULE_AUTHOR("Mika Westerberg <mika.westerberg@linux.intel.com>");
MODULE_AUTHOR("Chris Chang <chris1_chang@asus.com>");
MODULE_DESCRIPTION("SMB345 battery charger driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("i2c:smb345");

