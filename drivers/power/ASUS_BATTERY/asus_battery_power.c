/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chih-hsuan chang chih-hsuan_chang@asus.com
 */
#include <linux/module.h>
#include <linux/param.h>
#include <linux/jiffies.h>
#include <linux/workqueue.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <linux/power_supply.h>
#include <linux/slab.h>
#include <asm/unaligned.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/uaccess.h>
#include <linux/gpio.h>

#include "asus_battery.h"
#include "asus_battery_proc_fs.h"
#include "smb_external_include.h"
#include <linux/HWVersion.h>
#include <linux/wakelock.h>
#include <linux/usb/penwell_otg.h>
#include <asm/intel_scu_pmic.h>
#include <asm/intel_mid_gpadc.h>
#include <linux/switch.h>

#ifdef CONFIG_BLX
#include <linux/blx.h>
#endif

#define DISABLE_CHARGING_AT_LOW_TEMP false
#define VWARN1_CFG_REG	0x3C
#define VPROG2_CFG_REG	0xAD
#define PBCONFIG_REG		0x29

#define THERMAL_CTRL		1

static unsigned int  battery_current;
static unsigned int  battery_remaining_capacity;

module_param(battery_current , uint, 0644);
module_param(battery_remaining_capacity , uint, 0644);

extern int Read_HW_ID(void);
extern int Read_PROJ_ID(void);
int boot_mode, invalid_charger=0;
EXPORT_SYMBOL(boot_mode);
EXPORT_SYMBOL(invalid_charger);

//struct delayed_work battery_low_init_work;		//battery low init
struct delayed_work battery_poll_data_work;		//polling data
struct delayed_work detect_cable_work;			//check cable status
struct workqueue_struct *battery_work_queue=NULL;

/* wake lock to prevent S3 during charging */
struct wake_lock wakelock;
struct wake_lock wakelock_t;    // for wake_lokc_timout() useage
#ifdef CONFIG_A500CG_BATTERY_SMB347
static int temp_status=1; // 0:<0, 1: 0-45, 2:45-60, 3:>60
#endif
#ifdef CONFIG_SMB1357_CHARGER
enum temperature_type{  //unit: 0.1 degree Celsius
	BELOW_15=0,
	IN_15_100,
	IN_100_200,
	IN_200_400,
	IN_400_450,
	IN_450_500,
	IN_500_600,
	ABOVE_600,
};
static int temp_type=IN_200_400;
static int hvdcp_vbat_flag=0, highchgvol_flag=0, aicl_set_flag=0;
extern int hvdcp_mode, dcp_mode, early_suspend_flag, set_usbin_cur_flag;
static struct switch_dev charger_dev;
extern int smb1357_chr_suspend(void);
#endif
#ifdef THERMAL_CTRL
static int qc_disable=1, temp_1400=55000, temp_700=60000, temp_0=70000;
long systherm2_temp;
extern long read_systherm2_temp(void);
enum systherm2_control_current_block{
	LIMIT_NONE=0,
	LIMIT_1400,
	LIMIT_700,
	LIMIT_0,
};
static int current_type=LIMIT_NONE;
#endif

#ifdef CONFIG_LEDS_ASUS
extern void led_brightness_set(int led, int brightness);
#endif

#define FULL_CHARGED_SOC
#ifdef FULL_CHARGED_SOC
static int pre_soc=0, full_charged_flag=0;
#endif

DEFINE_MUTEX(batt_info_mutex);

static char *supply_list[] = {
        "battery",
};

struct battery_info_reply batt_info = {
        .drv_status = DRV_NOT_READY,
        .cable_status = NO_CABLE,
#ifdef CONFIG_ASUS_FACTORY_MODE
        .eng_charging_limit = true,
#endif
        .thermal_charging_limit = true, //in A500, thermal_charging_limit means thermal protection on or false
        .cable_source = CABLE_OUT,
        .gauge_version = 4,
        .isValidBattID = true,
};

#ifdef CONFIG_ASUS_FACTORY_MODE
ssize_t asus_charging_toggle_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	struct battery_info_reply tmp_batt_info;
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kmalloc(100,GFP_KERNEL);
  	if(!buff)
		return -ENOMEM;

	mutex_lock(&batt_info_mutex);
	tmp_batt_info = batt_info;
	mutex_unlock(&batt_info_mutex);

	BAT_DBG("%s:\n", __func__);

	len += sprintf(buff + len, "%d\n", tmp_batt_info.eng_charging_limit);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
  	kfree(buff);
  	return ret;
}

ssize_t asus_charging_toggle_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
    struct battery_info_reply tmp_batt_info;
    bool eng_charging_limit = true;

    BAT_DBG("%s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    eng_charging_limit = tmp_batt_info.eng_charging_limit;

    if (buffer[0] == '1') {
        /* turn on charging limit in eng mode */
        eng_charging_limit = true;
    }
    else if (buffer[0] == '0') {
        /* turn off charging limit in eng mode */
        eng_charging_limit = false;
    }

    tmp_batt_info.eng_charging_limit = eng_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return count;
}

ssize_t asus_charging_for_gague_toggle_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
    int len = 0;

    BAT_DBG("%s:\n", __func__);
    return len;
}

ssize_t asus_charging_for_gague_toggle_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
    struct battery_info_reply tmp_batt_info;
    //bool eng_charging_limit = true;

    BAT_DBG("%s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);

    //eng_charging_limit = tmp_batt_info.eng_charging_limit;

    if (buffer[0] == '0') {
        /* turn on charging limit in eng mode */
        //eng_charging_limit = true;
        BAT_DBG("disable charging:\n");
#if defined(CONFIG_A500CG_BATTERY_SMB347)
        smb347_charging_toggle(false);
#elif defined(CONFIG_SMB1357_CHARGER)
	smb1357_charging_toggle(false);
#endif
    }
    else if (buffer[0] == '1') {
        /* turn off charging limit in eng mode */
        //eng_charging_limit = false;
        BAT_DBG("enable charging:\n");
#if defined(CONFIG_A500CG_BATTERY_SMB347)
        smb347_charging_toggle(true);
#elif defined(CONFIG_SMB1357_CHARGER)
	smb1357_charging_toggle(true);
#endif
    }

    //tmp_batt_info.eng_charging_limit = eng_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    return count;
}

int init_asus_for_gague_charging_toggle(void)
{
	struct proc_dir_entry *entry=NULL;

	static struct file_operations asus_eng_for_gague_charging_limit_fop = {
	    	.read = asus_charging_for_gague_toggle_read,
		.write = asus_charging_for_gague_toggle_write,
	};
	entry = proc_create("asus_eng_for_gague_charging_limit", 0666,NULL, &asus_eng_for_gague_charging_limit_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/asus_eng_for_gague_charging_limit fail\n");
	}

	return 0;
}

int init_asus_charging_toggle(void)
{
	struct proc_dir_entry *entry=NULL;

	static struct file_operations charger_limit_enable_fop = {
	    	.read = asus_charging_toggle_read,
		.write = asus_charging_toggle_write,
	};
	entry = proc_create("charger_limit_enable", 0666,NULL, &charger_limit_enable_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/charger_limit_enable fail\n");
	}

	return 0;
}

#endif



//++Sewell+++ charging toggle interface for thermal
ssize_t thermal_charging_toggle_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	struct battery_info_reply tmp_batt_info;
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kmalloc(100,GFP_KERNEL);
  	if(!buff)
		return -ENOMEM;

	mutex_lock(&batt_info_mutex);
	tmp_batt_info = batt_info;
	mutex_unlock(&batt_info_mutex);

	BAT_DBG("%s:\n", __func__);

	len += sprintf(buff + len, "%d\n", tmp_batt_info.thermal_charging_limit);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
  	kfree(buff);
  	return ret;
}

ssize_t thermal_charging_toggle_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
    struct battery_info_reply tmp_batt_info;
    bool thermal_charging_limit = true;

     BAT_DBG("%s:\n", __func__);

    mutex_lock(&batt_info_mutex);
    tmp_batt_info = batt_info;
    mutex_unlock(&batt_info_mutex);


    if (buffer[0] == '0') {
        // Disable thermal protection
        thermal_charging_limit = false;

    }
    else // Enable thermal protection
        thermal_charging_limit = true;

    tmp_batt_info.thermal_charging_limit = thermal_charging_limit;

    mutex_lock(&batt_info_mutex);
    batt_info = tmp_batt_info;
    mutex_unlock(&batt_info_mutex);

    asus_queue_update_all();

    return count;
}

int init_thermal_charging_toggle(void)
{
	struct proc_dir_entry *entry=NULL;

	static struct file_operations thermal_charging_limit_fop = {
	    	.read = thermal_charging_toggle_read,
		.write = thermal_charging_toggle_write,
	};
	entry = proc_create("thermal_charging_limit", 0664,NULL, &thermal_charging_limit_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/thermal_charging_limit fail\n");
	}

	return 0;
}
//----Sewell----

#ifdef THERMAL_CTRL
ssize_t init_thermal_disable_qc_read(struct file *filp, char __user *buffer, size_t count, loff_t *ppos)
{
	int len = 0;
	ssize_t ret = 0;
	char *buff;

	buff = kmalloc(100,GFP_KERNEL);
	if(!buff)
		return -ENOMEM;
	len += sprintf(buff + len, "qc_disable=%d, temp_1400=%d, temp_700=%d, temp_0=%d\n", qc_disable, temp_1400, temp_700, temp_0);
	ret = simple_read_from_buffer(buffer,count,ppos,buff,len);
	kfree(buff);
	return ret;
}

ssize_t init_thermal_disable_qc_write(struct file *filp, const char __user *buffer, size_t count, loff_t *ppos)
{
	int buf_index=0, temp_index=0, temp_value=1, i=0, temp_1400_tmp=0, temp_700_tmp=0, temp_0_tmp=0;

	BAT_DBG("%s +++\n", __func__);
	/*handle input string to interger*/
	/*qc_disable*/
	while(buffer[buf_index]!=' ') {
		qc_disable = (int)buffer[buf_index] - 48;
		buf_index++;
	}
	buf_index++;
	/*temp_1400*/
	temp_index = 4;
	while(buffer[buf_index]!=' ') {
		for(i=0;i<temp_index;i++)
			temp_value = 10*temp_value;
		temp_1400_tmp += ((int)buffer[buf_index] - 48)*temp_value;
		buf_index++;
		temp_index--;
		temp_value = 1;
	}
	buf_index++;
	/*temp_700*/
	temp_index = 4;
	while(buffer[buf_index]!=' ') {
		for(i=0;i<temp_index;i++)
			temp_value = 10*temp_value;
		temp_700_tmp += ((int)buffer[buf_index] - 48)*temp_value;
		buf_index++;
		temp_index--;
		temp_value = 1;
	}
	buf_index++;
	/*temp_0*/
	temp_index = 4;
	while(buffer[buf_index]!='\n') {
		for(i=0;i<temp_index;i++)
			temp_value = 10*temp_value;
		temp_0_tmp += ((int)buffer[buf_index] - 48)*temp_value;
		buf_index++;
		temp_index--;
		temp_value = 1;
	}
	temp_1400 = temp_1400_tmp;
	temp_700 = temp_700_tmp;
	temp_0 = temp_0_tmp;
	return count;
}

int init_thermal_disable_qc(void)
{
	struct proc_dir_entry *entry=NULL;

	static struct file_operations thermal_charging_limit_fop = {
		.read = init_thermal_disable_qc_read,
		.write = init_thermal_disable_qc_write,
	};
	entry = proc_create("thermal_disable_qc", 0664,NULL, &thermal_charging_limit_fop);
	if(!entry)
	{
		BAT_DBG_E("create /proc/thermal_disable_qc fail\n");
	}
	return 0;
}
#endif

static enum power_supply_property asus_battery_props[] = {
        POWER_SUPPLY_PROP_STATUS,				//0x00
        POWER_SUPPLY_PROP_HEALTH,				//0x02
        POWER_SUPPLY_PROP_PRESENT,				//0x03
        POWER_SUPPLY_PROP_TECHNOLOGY,			//0x05
        POWER_SUPPLY_PROP_VOLTAGE_NOW,			//0x0A
        POWER_SUPPLY_PROP_CURRENT_NOW,			//0x0C
        POWER_SUPPLY_PROP_ENERGY_NOW,			//0x1B
        POWER_SUPPLY_PROP_CAPACITY,				//0x1D
        POWER_SUPPLY_PROP_CAPACITY_LEVEL,		//0x1E
        POWER_SUPPLY_PROP_TEMP,					//0x1F
        POWER_SUPPLY_PROP_MANUFACTURER,
        POWER_SUPPLY_PROP_MODEL_NAME,
        POWER_SUPPLY_PROP_SERIAL_NUMBER,
        POWER_SUPPLY_PROP_CURRENT_AVG,
        POWER_SUPPLY_PROP_CHARGE_FULL,
        POWER_SUPPLY_PROP_CHARGE_NOW,
#if 0
        POWER_SUPPLY_PROP_BATTERY_ID,
        POWER_SUPPLY_PROP_FIRMWARE_VERSION,
#endif
        //	POWER_SUPPLY_PROP_TIME_TO_EMPTY_NOW,	//0x21
        //	POWER_SUPPLY_PROP_TIME_TO_EMPTY_AVG,	//0x22
        //	POWER_SUPPLY_PROP_TIME_TO_FULL_NOW,		//0x23
};

static enum power_supply_property asus_power_properties[] = {
        POWER_SUPPLY_PROP_PRESENT,	//0x03
        POWER_SUPPLY_PROP_ONLINE,	//0x04
};

static int asus_battery_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val);

static int asus_power_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val);


static struct power_supply asus_power_supplies[] = {
        {
                .name = "battery",
                .type = POWER_SUPPLY_TYPE_BATTERY,
                .properties = asus_battery_props,
                .num_properties = ARRAY_SIZE(asus_battery_props),
                .get_property = asus_battery_get_property,
        },
        {
                .name = "ac",
                .type = POWER_SUPPLY_TYPE_MAINS,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = asus_power_properties,
                .num_properties = ARRAY_SIZE(asus_power_properties),
                .get_property = asus_power_get_property,
        },
        {
                .name = "usb",
                .type = POWER_SUPPLY_TYPE_USB,
                .supplied_to = supply_list,
                .num_supplicants = ARRAY_SIZE(supply_list),
                .properties = asus_power_properties,
                .num_properties = ARRAY_SIZE(asus_power_properties),
                .get_property = asus_power_get_property,
        },
};

int asus_battery_low_event()
{
        drv_status_t drv_status;

        mutex_lock(&batt_info_mutex);    
        drv_status = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        drv_status = DRV_BATTERY_LOW;

        mutex_lock(&batt_info_mutex);    
        batt_info.drv_status = drv_status;
        mutex_unlock(&batt_info_mutex);
	BAT_DBG("update battery info to frameworks\n");
        power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);
        //power_supply_changed(&asus_power_supplies[CHARGER_AC]);
        //power_supply_changed(&asus_power_supplies[CHARGER_USB]);

        return 0;
}

/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int asus_battery_update_temp_no_mutex(void)
{
        int temperature = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_temp)
                temperature = tmp_batt_info.tbl->read_temp();

        return temperature;
}

/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
static int asus_battery_update_voltage_no_mutex(void)
{
        int volt = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_volt)
                volt = tmp_batt_info.tbl->read_volt();

        return volt;
}

/*
 * Return the battery average current
 * Note that current can be negative signed as well(-65536 ~ 65535).
 * So, the value get from device need add a base(0x10000) to be a positive number if no error. 
 * Or < 0 if something fails.
 */
static int asus_battery_update_current_no_mutex(void)
{
        int curr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_current)
                curr = tmp_batt_info.tbl->read_current();

        return curr;
}
//not used in A500/A600
#if 0
static int asus_battery_update_available_energy_no_mutex(void)
{
        int mWhr = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_av_energy)
                mWhr = tmp_batt_info.tbl->read_av_energy();

        return mWhr;
}
#endif
static int asus_battery_update_percentage_no_mutex(void)
{
        int percentage = -EINVAL;
        drv_status_t drv_status;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        drv_status = batt_info.drv_status;

        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_percentage) {
                percentage = tmp_batt_info.tbl->read_percentage();

                if (percentage == ERROR_CODE_I2C_FAILURE)
                    return -EINVAL;

                if (percentage == 0) {
                        drv_status = DRV_SHUTDOWN;
                }

                batt_info.drv_status = drv_status;
        }

        return percentage; //% return adjust percentage, and error code ( < 0)
}

/*
 * Return the battery fcc in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int asus_battery_update_fcc_no_mutex(void)
{
        int fcc = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_fcc)
                fcc = tmp_batt_info.tbl->read_fcc();

        return fcc;
}

/*
 * Return the battery nac in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int asus_battery_update_nac_no_mutex(void)
{
        int nac = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_nac)
                nac = tmp_batt_info.tbl->read_nac();

        return nac;
}

/*
 * Return the battery nac in tenths of degree Celsius
 * Or < 0 if something fails.
 */
static int asus_battery_update_rm_no_mutex(void)
{
        int nac = -EINVAL;
        struct battery_info_reply tmp_batt_info;

        tmp_batt_info = batt_info;
        if (tmp_batt_info.tbl && tmp_batt_info.tbl->read_rm)
                nac = tmp_batt_info.tbl->read_rm();

        return nac;
}

//range:   range_min <= X < range_max
struct lvl_entry {
        char name[30];
        int range_min;
        int range_max;
        int ret_val;
};

#define MAX_DONT_CARE    1000000
#define MIN_DONT_CARE    -999999
struct lvl_entry lvl_tbl[] = {
        { "FULL",      100,    MAX_DONT_CARE,   POWER_SUPPLY_CAPACITY_LEVEL_FULL},  
        { "HIGH",       80,              100,   POWER_SUPPLY_CAPACITY_LEVEL_HIGH},  
        { "NORMAL",     20,               80,   POWER_SUPPLY_CAPACITY_LEVEL_NORMAL},  
        { "LOW",         5,               20,   POWER_SUPPLY_CAPACITY_LEVEL_LOW},  
        { "CRITICAL",    0,                5,   POWER_SUPPLY_CAPACITY_LEVEL_CRITICAL},  
        { "UNKNOWN", MIN_DONT_CARE,  MAX_DONT_CARE,  POWER_SUPPLY_CAPACITY_LEVEL_UNKNOWN},  
};

static int asus_battery_update_capacity_level(int percentage)
{	
        int i=0;

        for (i=0; i<sizeof(lvl_tbl)/sizeof(struct lvl_entry); i++) {
                if (lvl_tbl[i].range_max != MAX_DONT_CARE && 
                        percentage >= lvl_tbl[i].range_max)
                        continue;
                if (lvl_tbl[i].range_min != MIN_DONT_CARE && 
                        lvl_tbl[i].range_min > percentage)
                        continue;
                //printk(KERN_WARNING "[BATT] battery level = %s\n", lvl_tbl[i].name);
                return lvl_tbl[i].ret_val;
        }

        return -EINVAL;
}

static int asus_battery_update_status_no_mutex(int percentage)
{
        int status;
        int temperature, vbat;
		
        struct battery_info_reply tmp_batt_info;
        u32 cable_status;

        tmp_batt_info = batt_info;
        cable_status = tmp_batt_info.cable_status;
        BAT_DBG("%s , cable status = %d, percentage = %d\n", __func__, cable_status, percentage);

        if (cable_status == USB_ADAPTER || cable_status == USB_PC) {
                status = POWER_SUPPLY_STATUS_CHARGING;
#ifdef CONFIG_BLX
                if (percentage >= get_charginglimit() && get_charginglimit() <= 98) {
#if defined(CONFIG_A500CG_BATTERY_SMB347)
			smb347_charging_toggle(false);
#elif defined(CONFIG_SMB1357_CHARGER)
			smb1357_charging_toggle(false);
#endif
						//Fake the battery to be full now and exit here
                        status = POWER_SUPPLY_STATUS_FULL;
                        goto final;
                }
#endif

#ifdef CONFIG_SMB1357_CHARGER
		/* read aicl if needed to set input current*/
		vbat = asus_battery_update_voltage_no_mutex();
		if((cable_status == USB_ADAPTER)&&(dcp_mode==1)&&(aicl_set_flag==0)) {
			if(hvdcp_mode==1) {
				if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
					if((smb1357_get_aicl_result()>=0x0b)&&(smb1357_get_aicl_result()<0x12)) {
						BAT_DBG("HVDCP in and aicl result between 1000~1800mA\n");
						if(set_usbin_cur_flag==1) {
							BAT_DBG("setting step current now so ignore!\n");
						}else {
							BAT_DBG("set current again!\n");
							set_QC_inputI_limit(3);
							aicl_set_flag = 1;
						}
					}
				}else {
					if((smb1357_get_aicl_result()>=0x0d)&&(smb1357_get_aicl_result()<0x12)) {
						BAT_DBG("HVDCP in and aicl result between 1200~1800mA\n");
						if(set_usbin_cur_flag==1) {
							BAT_DBG("setting step current now so ignore!\n");
						}else {
							BAT_DBG("set current again!\n");
							set_QC_inputI_limit(2);
							aicl_set_flag = 1;
						}
					}
				}
			}else {
				if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
					if((smb1357_get_aicl_result()>=0x0b)&&(smb1357_get_aicl_result()<0x10)) {
						BAT_DBG("DCP in and aicl result between 1000~1500mA\n");
						if(set_usbin_cur_flag==1) {
							BAT_DBG("setting step current now so ignore!\n");
						}else {
							BAT_DBG("set current again!\n");
							set_QC_inputI_limit(3);
							aicl_set_flag = 1;
						}
					}else if( (vbat>4300)||(percentage>70) ) {
						BAT_DBG("DCP in and vbat >4.3V  or soc > 70 \n");
						if(set_usbin_cur_flag==1) {
							BAT_DBG("setting step current now so ignore!\n");
						}else {
							BAT_DBG("set current again!\n");
							set_QC_inputI_limit(3);
							aicl_set_flag = 1;
						}
					}
				}else if(Read_PROJ_ID()==PROJ_ID_ZE551ML_CKD) {
					if((smb1357_get_aicl_result()>=0x0b)&&(smb1357_get_aicl_result()<0x12)) {
						BAT_DBG("BZ SKU: DCP in and aicl result between 1000~1800mA\n");
						if(set_usbin_cur_flag==1) {
							BAT_DBG("setting step current now so ignore!\n");
						}else {
							BAT_DBG("set current again!\n");
							set_QC_inputI_limit(3);
							aicl_set_flag = 1;
						}
					}else if(vbat>3800) {
						BAT_DBG("BZ SKU: DCP in and vbat >3.8V\n");
						if(set_usbin_cur_flag==1) {
							BAT_DBG("setting step current now so ignore!\n");
						}else {
							BAT_DBG("set current again!\n");
							set_QC_inputI_limit(3);
							aicl_set_flag = 1;
						}
					}
				}
			}
		}
		
		/*JEITA rule*/
		temperature = asus_battery_update_temp_no_mutex();
		BAT_DBG("get temperature:%d from gauge\n",temperature);

		/*ZE550ML/ZE551ML/ZX550ML*/
		if ((temperature != ERROR_CODE_I2C_FAILURE)&&(tmp_batt_info.thermal_charging_limit)) {
			if (temperature < 15) {
				smb1357_set_voltage(false);
				highchgvol_flag = 0;
				smb1357_charging_toggle(false);
				status = POWER_SUPPLY_STATUS_DISCHARGING;
				temp_type = BELOW_15;
				goto final;
			}
			if( temperature >=15 && temperature <100){
				if(temp_type<=BELOW_15&&temperature<45){
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					smb1357_charging_toggle(false);
					status = POWER_SUPPLY_STATUS_DISCHARGING;
					goto final;
				}else{
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					smb1357_set_Ichg(700);
					smb1357_charging_toggle(true);
					temp_type = IN_15_100;
					if(hvdcp_mode==1) {
						if(percentage<=70)
							status = POWER_SUPPLY_STATUS_QUICK_CHARGING;
						else
							status = POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING;
					}
				}
				#ifdef CONFIG_BLX
				if(get_charginglimit() >= 99)
				{
					if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
						smb1357_charging_toggle(false);
						smb1357_charging_toggle(true);
					}
				}
				#else
				/*recharge*/
				if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
					smb1357_charging_toggle(false);
					smb1357_charging_toggle(true);
				}
				#endif
			}
			if( temperature >=100 && temperature <200){
				if(temp_type<=IN_15_100&&temperature<130){
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					smb1357_set_Ichg(700);
					smb1357_charging_toggle(true);
				}else{
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					if(hvdcp_mode==1) {
						smb1357_set_Ichg(1400);
						if(percentage<=70)
							status = POWER_SUPPLY_STATUS_QUICK_CHARGING;
						else
							status = POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING;
					} else {
						if(Read_PROJ_ID()==PROJ_ID_ZX550ML)
							smb1357_set_Ichg(2000);
						else
							smb1357_set_Ichg(2800);
					}
					smb1357_charging_toggle(true);
					temp_type = IN_100_200;
				}
				#ifdef CONFIG_BLX
				if(get_charginglimit() >= 99)
				{
					if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
						smb1357_charging_toggle(false);
						smb1357_charging_toggle(true);
					}
				}
				#else
				/*recharge*/
				if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
					smb1357_charging_toggle(false);
					smb1357_charging_toggle(true);
				}
				#endif
			}
			if( temperature >=200 && temperature <400){
				if(temp_type<=IN_100_200&&temperature<230){
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					smb1357_set_Ichg(1400);
					smb1357_charging_toggle(true);
				}else if(temp_type>=IN_400_450&&temperature>370){
				    goto handle400_450;
				}else{
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					if(hvdcp_mode==1) {
						BAT_DBG("HVDCP mode in 20 - 40 oC! hvdcp_vbat_flag=%d\n", hvdcp_vbat_flag);
						if(vbat<4250) {
							if(hvdcp_vbat_flag) {
								if(vbat<4100) {
									smb1357_set_Ichg(2800);
									hvdcp_vbat_flag = 0;
								}
							}else {
								smb1357_set_Ichg(2800);
							}
						}else {
							smb1357_set_Ichg(1400);
							hvdcp_vbat_flag = 1;
						}
						if(percentage<=70)
							status = POWER_SUPPLY_STATUS_QUICK_CHARGING;
						else
							status = POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING;
					}else {
						if(Read_PROJ_ID()==PROJ_ID_ZX550ML)
							smb1357_set_Ichg(2000);
						else
							smb1357_set_Ichg(2800);
					}
					smb1357_charging_toggle(true);
					temp_type = IN_200_400;
				}
				#ifdef CONFIG_BLX
				if(get_charginglimit() >= 99)
				{
					if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
						smb1357_charging_toggle(false);
						smb1357_charging_toggle(true);
					}
				}
				#else
				/*recharge*/
				if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
					smb1357_charging_toggle(false);
					smb1357_charging_toggle(true);
				}
				#endif
			}
			if( temperature >=400 && temperature <450){
				if(temp_type>=IN_450_500&&temperature>420){
				    goto handle450_500;
				}else{
handle400_450:
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					if(hvdcp_mode==1) {
						BAT_DBG("HVDCP mode in 40 - 45 oC! hvdcp_vbat_flag=%d\n", hvdcp_vbat_flag);
						if(vbat<4250) {
							if(hvdcp_vbat_flag) {
								if(vbat<4100) {
									smb1357_set_Ichg(2500);
									hvdcp_vbat_flag = 0;
								}
							}else {
								smb1357_set_Ichg(2500);
							}
						}else {
							smb1357_set_Ichg(1400);
							hvdcp_vbat_flag = 1;
						}
						if(percentage<=70)
							status = POWER_SUPPLY_STATUS_QUICK_CHARGING;
						else
							status = POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING;
					}else {
						if(Read_PROJ_ID()==PROJ_ID_ZX550ML)
							smb1357_set_Ichg(2000);
						else
							smb1357_set_Ichg(2800);
					}
					smb1357_charging_toggle(true);
					temp_type = IN_400_450;
				}

				#ifdef CONFIG_BLX
				if(get_charginglimit() >= 99)
				{
					if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
						smb1357_charging_toggle(false);
						smb1357_charging_toggle(true);
					}
				}
				#else
				/*recharge*/
				if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
					smb1357_charging_toggle(false);
					smb1357_charging_toggle(true);
				}
				#endif
			}
			if( temperature >=450 && temperature <500){
				if(temp_type>=IN_500_600&&temperature>470){
				    goto handle500_600;
				}else{
handle450_500:
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					if(hvdcp_mode==1) {
						BAT_DBG("HVDCP mode in 45 - 50 oC! hvdcp_vbat_flag=%d\n", hvdcp_vbat_flag);
						if(vbat<4250) {
							if(hvdcp_vbat_flag) {
								if(vbat<4100) {
									smb1357_set_Ichg(2300);
									hvdcp_vbat_flag = 0;
								}
							}else {
								smb1357_set_Ichg(2300);
							}
						}else {
							smb1357_set_Ichg(1400);
							hvdcp_vbat_flag = 1;
						}
						if(percentage<=70)
							status = POWER_SUPPLY_STATUS_QUICK_CHARGING;
						else
							status = POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING;
					}else {
						if(Read_PROJ_ID()==PROJ_ID_ZX550ML)
							smb1357_set_Ichg(2000);
						else
							smb1357_set_Ichg(2800);
					}
					smb1357_charging_toggle(true);
					temp_type = IN_450_500;
				}
								#ifdef CONFIG_BLX
				if(get_charginglimit() >= 99)
				{
					if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
						smb1357_charging_toggle(false);
						smb1357_charging_toggle(true);
					}
				}
				#else
				/*recharge*/
				if((percentage <= 98)&&(smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
					smb1357_charging_toggle(false);
					smb1357_charging_toggle(true);
				}
				#endif
			}
			if(temperature >=500 && temperature <600){
handle500_600:
				if(((highchgvol_flag==0)&&(vbat>4100))||(temp_type>=ABOVE_600&&temperature>570)){
					smb1357_set_voltage(false);
					highchgvol_flag = 0;
					smb1357_charging_toggle(false);
					status = POWER_SUPPLY_STATUS_DISCHARGING;
					goto final;
				}else{
					smb1357_set_voltage(true);
					highchgvol_flag = 1;
					/* set recharge voltage */
					if(vbat < 4000)
						smb1357_set_recharge(true);
					else
						smb1357_set_recharge(false);
					smb1357_set_Ichg(1400);
					smb1357_charging_toggle(true);
					temp_type = IN_500_600;
					if(hvdcp_mode==1) {
						if(percentage<=70)
							status = POWER_SUPPLY_STATUS_QUICK_CHARGING;
						else
							status = POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING;
					}
				}
			}
			if(temperature >=600){
				smb1357_set_voltage(false);
				highchgvol_flag = 0;
				smb1357_charging_toggle(false);
				temp_type = ABOVE_600;
				status = POWER_SUPPLY_STATUS_DISCHARGING;
				goto final;
			}
		}else if(temperature != ERROR_CODE_I2C_FAILURE) {
			/*thermal protection off*/
			smb1357_set_voltage(false);
			highchgvol_flag = 0;
			smb1357_charging_toggle(true);
		}
#endif

#ifdef CONFIG_A500CG_BATTERY_SMB347
				BAT_DBG("smb358 !!!!!%s\n",__func__);
                if (status == POWER_SUPPLY_STATUS_CHARGING) {
			/* A500CG limit to protect battery from damaged when battery temperature is too low or High*/
			temperature = asus_battery_update_temp_no_mutex();
                        if ((temperature != ERROR_CODE_I2C_FAILURE)&&(tmp_batt_info.thermal_charging_limit)) {
					if (temperature <= 0) {
						smb347_set_voltage(false);
						smb347_charging_toggle(false);
						status = POWER_SUPPLY_STATUS_DISCHARGING;
						temp_status = 0;
						goto final;
					} else if (temperature > 0 && temperature <= 450) {
						if(temp_status==0&&temperature<50) {
							smb347_charging_toggle(false);
							status = POWER_SUPPLY_STATUS_DISCHARGING;
							goto final;
						}else if(temp_status==2&&temperature>400) {
							smb347_set_voltage(true);
						}else {
							smb347_set_voltage(false);
							smb347_charging_toggle(true);
							temp_status = 1;
						}
					} else if (temperature > 450 && temperature <= 600) {
						if(temp_status==3&&temperature>550) {
							smb347_charging_toggle(false);
							status = POWER_SUPPLY_STATUS_DISCHARGING;
							goto final;
						}else {
							smb347_set_voltage(true);
							smb347_charging_toggle(true);
							temp_status = 2;
						}
					} else {
						smb347_set_voltage(true);
						smb347_charging_toggle(false);
						status = POWER_SUPPLY_STATUS_DISCHARGING;
						temp_status = 3;
						goto final;
					}
                        }
                        else if(temperature != ERROR_CODE_I2C_FAILURE) {
					/*thermal protection off*/
					smb347_set_voltage(false);
					smb347_charging_toggle(true);
			    }
                        else
                                status = POWER_SUPPLY_STATUS_UNKNOWN;
			    if (cable_status == USB_ADAPTER && smb347_get_aicl_result() <= 0x01 && tmp_batt_info.batt_volt>=3000 ) {// AICL result < 500mA
                        BAT_DBG("AICL get result, AICL results < 500mA, voltage=%d\n", tmp_batt_info.batt_volt);
                        smb347_AC_in_current();
                    }
                }
#endif
#ifdef THERMAL_CTRL
#ifdef CONFIG_SMB1357_CHARGER
		/*for QC thermal shutdown issue, so add monitoring SYSTHERM2 function for ZE550ML/ZE551ML*/
		if(qc_disable&&hvdcp_mode&&(!early_suspend_flag)&&(Read_PROJ_ID()!=PROJ_ID_ZX550ML)&&(boot_mode==1)) {
			systherm2_temp = read_systherm2_temp();
			if(systherm2_temp<temp_1400) {
				if(current_type>=LIMIT_1400 && systherm2_temp>(temp_1400-3000)) {
					smb1357_set_voltage(false);
					smb1357_set_Ichg(1400);
					smb1357_charging_toggle(true);
					current_type=LIMIT_1400;
				}else {
					current_type=LIMIT_NONE;
				}
			}else if(systherm2_temp>=temp_1400 && systherm2_temp<temp_700) {
				if(current_type>=LIMIT_700 && systherm2_temp>(temp_700-3000)) {
					smb1357_set_voltage(false);
					smb1357_set_Ichg(700);
					smb1357_charging_toggle(true);
					current_type=LIMIT_700;
				}else {
					smb1357_set_voltage(false);
					smb1357_set_Ichg(1400);
					smb1357_charging_toggle(true);
					current_type=LIMIT_1400;
				}
			}else if(systherm2_temp>=temp_700 && systherm2_temp<temp_0) {
				if(current_type>=LIMIT_0 && systherm2_temp>(temp_0-3000)) {
					smb1357_set_voltage(false);
					smb1357_charging_toggle(false);
					current_type = LIMIT_0;
					//status = POWER_SUPPLY_STATUS_DISCHARGING;
					//goto final;
				}else {
					smb1357_set_voltage(false);
					smb1357_set_Ichg(700);
					smb1357_charging_toggle(true);
					current_type=LIMIT_700;
				}
			}else {
				smb1357_set_voltage(false);
				smb1357_charging_toggle(false);
				current_type = LIMIT_0;
				//status = POWER_SUPPLY_STATUS_DISCHARGING;
				//goto final;
			}
			BAT_DBG("use thermal policy and systherm2_temp=%ld, current_type=%d\n", systherm2_temp, current_type);
		}
#endif
#endif
               if (percentage >= 0 && percentage <= 100) {               
#ifdef CONFIG_BLX
					if (percentage >= get_charginglimit())
#else
					if (percentage == 100)
#endif
					{
							status = POWER_SUPPLY_STATUS_FULL;
							goto final;
					}
					
					if (percentage == 99) 
					{
#if defined(CONFIG_A500CG_BATTERY_SMB347)
                        if (smb347_get_charging_status() == POWER_SUPPLY_STATUS_FULL) {
#elif defined(CONFIG_SMB1357_CHARGER)
                        if (smb1357_get_charging_status() == POWER_SUPPLY_STATUS_FULL) {
#else
						if(0) {
#endif
						//check if Full-charged is detected
                             BAT_DBG("Full-charged is detected when in 99%% !\n");
                             status = POWER_SUPPLY_STATUS_FULL;
                             goto final;
                          }
					}
				} 
				else 
					BAT_DBG_E("Incorrect percentage !!!!!\n");
                } else {
                        status = POWER_SUPPLY_STATUS_NOT_CHARGING;
#ifdef CONFIG_SMB1357_CHARGER
						aicl_set_flag = 0;
#endif
				}

final:
        if (status == POWER_SUPPLY_STATUS_FULL)
                printk(KERN_WARNING "[BATT] battery status = POWER_SUPPLY_STATUS_FULL\n");
        else if (status == POWER_SUPPLY_STATUS_CHARGING)
                printk(KERN_WARNING "[BATT] battery status = POWER_SUPPLY_STATUS_CHARGING\n");
        else if (status == POWER_SUPPLY_STATUS_DISCHARGING)
                printk(KERN_WARNING "[BATT] battery status = POWER_SUPPLY_STATUS_DISCHARGING\n");
        else if (status == POWER_SUPPLY_STATUS_NOT_CHARGING)
                printk(KERN_WARNING "[BATT] battery status = POWER_SUPPLY_STATUS_NOT_CHARGING\n");
        else if (status == POWER_SUPPLY_STATUS_UNKNOWN)
                printk(KERN_WARNING "[BATT] battery status = POWER_SUPPLY_STATUS_UNKNOWN\n");
	else if (status == POWER_SUPPLY_STATUS_QUICK_CHARGING)
                printk(KERN_WARNING "[BATT] battery status = POWER_SUPPLY_STATUS_QUICK_CHARGING\n");
	else if (status == POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING)
                printk(KERN_WARNING "[BATT] battery status = POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING\n");

	/* LED behavior in charger mode*/
	if(boot_mode == 4) {
		BAT_DBG("in charger mode and status = %d, set LED\n", status);
#ifdef CONFIG_LEDS_ASUS
#ifdef CONFIG_SMB1357_CHARGER
		if (invalid_charger) {
			led_brightness_set(0, 0);
			led_brightness_set(1, 0);
		} else if (status == POWER_SUPPLY_STATUS_FULL) {
#else
		if (status == POWER_SUPPLY_STATUS_FULL) {
#endif
			led_brightness_set(0, 0);
			led_brightness_set(1, 1);
		}else if ((status == POWER_SUPPLY_STATUS_CHARGING)||(status == POWER_SUPPLY_STATUS_QUICK_CHARGING)||(status == POWER_SUPPLY_STATUS_NOT_QUICK_CHARGING)) {
			led_brightness_set(0, 1);
			led_brightness_set(1, 1);
		}else {
			led_brightness_set(0, 0);
			led_brightness_set(1, 0);
		}
#endif
	}
        return status;
}

static void asus_battery_get_info_no_mutex(void)
{
        struct battery_info_reply tmp_batt_info;
        static int pre_batt_percentage = -1;
        static int pre_cable_status = -1;
        int tmp=0;

        tmp_batt_info = batt_info;

        tmp = asus_battery_update_voltage_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_volt = tmp;

        tmp = asus_battery_update_current_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_current = tmp - 0x10000;

        tmp = asus_battery_update_percentage_no_mutex();
        if (tmp >= 0) {
                if (pre_cable_status < 0)
                        pre_cable_status = tmp_batt_info.cable_status;
                if (pre_batt_percentage<0) {
                        tmp_batt_info.percentage = tmp;
                        pre_batt_percentage = tmp;
                        BAT_DBG("Init the battery percentage values = %d\n", pre_batt_percentage);
                }
                else if ((tmp_batt_info.cable_status == NO_CABLE) && (pre_cable_status == NO_CABLE) && ( pre_batt_percentage < tmp)) {
                        tmp_batt_info.percentage = pre_batt_percentage;
                        printk(KERN_WARNING "[BATT] keep pre battery percentage = (pre:%d, now:%d)\n", pre_batt_percentage, tmp);
                }
                else {
                        /* FIX: suddently battery percentage drop while
                           it is nearly battery low (TT259005).
                           We adopt the Dichotomy method to report the percentage smoothly
                         */
                        if (tmp < 4 && pre_batt_percentage > 5) {
                            printk(KERN_WARNING "[BATT] modify dropping percentage = (now:%d, mod:%d)\n", tmp, (tmp+pre_batt_percentage)/2);
                            tmp = (tmp + pre_batt_percentage) / 2;
                        }
                        tmp_batt_info.percentage = tmp;
                        pre_batt_percentage = tmp;
                }
                pre_cable_status = tmp_batt_info.cable_status;
        }

        tmp = asus_battery_update_capacity_level(tmp_batt_info.percentage);
        if (tmp >= 0) tmp_batt_info.level = tmp;

        tmp = asus_battery_update_status_no_mutex(tmp_batt_info.percentage);
        if (tmp >= 0) tmp_batt_info.status = tmp;

        tmp = asus_battery_update_temp_no_mutex();
        if (tmp != ERROR_CODE_I2C_FAILURE && tmp != -EINVAL) tmp_batt_info.batt_temp = tmp;

        tmp = asus_battery_update_fcc_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_fcc = tmp;

        tmp = asus_battery_update_nac_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_nac = tmp;

	tmp = asus_battery_update_rm_no_mutex();
        if (tmp >= 0) tmp_batt_info.batt_rm = tmp;

        // support for PSChange app
        //battery_remaining_capacity = bq27520_asus_battery_dev_read_remaining_capacity(); /* no bq27520 */
        if (battery_remaining_capacity < 0) battery_remaining_capacity = 0;
        battery_current            = tmp_batt_info.batt_current;

#ifdef FULL_CHARGED_SOC
	BAT_DBG("WA for full-charged issue, full_charged_flag=%d, pre_soc=%d, percentage=%d, cable_status=%d\n", full_charged_flag, pre_soc, tmp_batt_info.percentage, tmp_batt_info.cable_status);
	if (tmp_batt_info.cable_status == NO_CABLE) {
		full_charged_flag = 0;
	}
	#ifdef CONFIG_BLX
	else if(tmp_batt_info.percentage >= get_charginglimit()) {
		tmp = (get_charginglimit() == 100 ? 99 : get_charginglimit()); //We need to catch the case if BLX is just set to 100
		if(pre_soc >= tmp)
		{
			tmp_batt_info.status = POWER_SUPPLY_STATUS_FULL;
			full_charged_flag = 1;
		}
	#else
	else if(tmp_batt_info.percentage == 100) {
		if (pre_soc>=99) 
		{
			tmp_batt_info.status = POWER_SUPPLY_STATUS_FULL;
			full_charged_flag = 1;
		}
	#endif 
		else
			full_charged_flag = 0;
		
	}
#ifdef CONFIG_BLX
	tmp = (get_charginglimit() >= 99 ? 98 : get_charginglimit()); //if BLX is set to 99 or 100
	if ((full_charged_flag==1)&&(tmp_batt_info.cable_status ==USB_ADAPTER)&&(tmp_batt_info.percentage>=tmp)) {
#else
	if ((full_charged_flag==1)&&(tmp_batt_info.cable_status ==USB_ADAPTER)&&(tmp_batt_info.percentage>=98)) {
#endif
		tmp_batt_info.percentage = 100;
		tmp_batt_info.status = POWER_SUPPLY_STATUS_FULL;
	}
	pre_soc = tmp_batt_info.percentage;
#endif

	printk(KERN_WARNING "[BATT] battery info (P:%d %%, V:%d mV, C:%d mA, T:%d C)\n",
                tmp_batt_info.percentage,
                tmp_batt_info.batt_volt,
                tmp_batt_info.batt_current,
                tmp_batt_info.batt_temp);

        batt_info = tmp_batt_info;

}

static void asus_polling_data(struct work_struct *dat)
{
        u32 polling_time = 60*HZ;
        struct battery_info_reply tmp_batt_info;

        asus_update_all();
        msleep(10);

        /* battery polling algorithm */

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        if (tmp_batt_info.batt_temp >= 630) {
            BAT_DBG("Critical condition!! -> temperature \n");
            polling_time = BATTERY_CRITICAL_POLL_TIME;
        }
        else if (tmp_batt_info.batt_temp >= 500) {
            BAT_DBG("Nearly critical condition!! -> temperature \n");
            polling_time = 10*HZ;
        }

        if (tmp_batt_info.percentage >= 50 && tmp_batt_info.percentage <= 100)
            polling_time = (60*HZ) < polling_time ? (60*HZ) : polling_time;
        else if (tmp_batt_info.percentage >= 20 && tmp_batt_info.percentage <= 49)
            polling_time = (30*HZ) < polling_time ? (30*HZ) : polling_time;
        else if (tmp_batt_info.percentage >= 5 && tmp_batt_info.percentage <= 19)
            polling_time = (10*HZ) < polling_time ? (10*HZ) : polling_time;
        else if (tmp_batt_info.percentage >= 0 && tmp_batt_info.percentage <= 4)
            polling_time = (5*HZ) < polling_time ? (5*HZ) : polling_time;
        else {
            BAT_DBG_E("*** Battery percentage is out of the legal range (percentage < 0 or percentage > 100) ***\n");
            polling_time = 5*HZ;
        }
        BAT_DBG("battery polling interval = %d secs.\n", polling_time/HZ);

        queue_delayed_work(battery_work_queue, &battery_poll_data_work, polling_time);
}

void asus_update_all(void)
{
        mutex_lock(&batt_info_mutex);
#ifdef CONFIG_SMB1357_CHARGER
        asus_battery_get_info_no_mutex();
        // if the power source changes, all power supplies may change state 
	if (smb1357_chr_suspend()&&(Read_PROJ_ID()!=PROJ_ID_ZX550ML)) {
		BAT_DBG("invalid charger!!!\n");
		invalid_charger = 1;
	} else {
		invalid_charger = 0;
	}
	switch_set_state(&charger_dev, invalid_charger);
#endif
        BAT_DBG("update battery info to frameworks\n");
        power_supply_changed(&asus_power_supplies[CHARGER_BATTERY]);
        //power_supply_changed(&asus_power_supplies[CHARGER_AC]);
        //power_supply_changed(&asus_power_supplies[CHARGER_USB]);

        mutex_unlock(&batt_info_mutex);
}

static void USB_cable_status_worker(struct work_struct *dat)
{
        asus_update_all();
}

void asus_queue_update_all(void)
{
        queue_delayed_work(battery_work_queue, &detect_cable_work, 0.1*HZ);
}

void usb_to_battery_callback(u32 usb_cable_state)
{
        drv_status_t drv_sts;    

        BAT_DBG("%s\n", __func__) ;

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        mutex_lock(&batt_info_mutex);
        batt_info.cable_status = usb_cable_state;
        mutex_unlock(&batt_info_mutex);

        if (batt_info.cable_status == USB_ADAPTER || batt_info.cable_status == USB_PC) {
#if defined(CONFIG_A500CG_BATTERY_SMB347)
            smb347_set_fast_charge();
            smb347_set_voltage(false);
            smb347_control_JEITA(true);
            if (batt_info.cable_status == USB_ADAPTER) {
                /*A500CG  set I_USB_IN=1200mA*/
                smb347_AC_in_current();
            }
#elif defined(CONFIG_SMB1357_CHARGER)
		smb1357_set_fast_charge();
		smb1357_charging_toggle(false); // for not charging when almost full battery workaround
		smb1357_set_voltage(false);
		smb1357_charging_toggle(true); // for not charging when almost full battery workaround
		smb1357_watchdog_timer_enable();
		if ((batt_info.cable_status == USB_ADAPTER)&&(Read_PROJ_ID()!=PROJ_ID_ZX550ML)) {
			smb1357_AC_in_current();
		}
		highchgvol_flag = 0;
		smb1357_control_JEITA(true);
#endif
        }

        if (drv_sts != DRV_REGISTER_OK) {
                BAT_DBG_E("Battery module not ready\n");
                return;
        }

        mutex_lock(&batt_info_mutex);

        /* prevent system from entering s3 in COS while AC charger is connected */
        if (boot_mode == 4) {
            if (batt_info.cable_status == USB_ADAPTER) {
                if (!wake_lock_active(&wakelock)) {
                    BAT_DBG("%s: asus_battery_power_wakelock -> wake lock\n", __func__);
                    wake_lock(&wakelock);
                }
            }
            else if (batt_info.cable_status == NO_CABLE) {
                if (wake_lock_active(&wakelock)) {
                    BAT_DBG("%s: asus_battery_power_wakelock -> wake unlock\n", __func__);
                    wake_lock_timeout(&wakelock_t, 3*HZ);  // timeout value as same as the <charger.exe>\asus_global.h #define ASUS_UNPLUGGED_SHUTDOWN_TIME(3 sec)
                    wake_unlock(&wakelock);
                } else { // for PC case
                    wake_lock_timeout(&wakelock_t, 3*HZ);
                }
            }
        }
        mutex_unlock(&batt_info_mutex);

        queue_delayed_work(battery_work_queue, &detect_cable_work, 0.1*HZ);
#if defined(CONFIG_SMB1357_CHARGER)
	if ((batt_info.cable_status == USB_ADAPTER)&&(Read_PROJ_ID()==PROJ_ID_ZX550ML)) {
		smb1357_AC_in_current();
	}
#endif
}
EXPORT_SYMBOL(usb_to_battery_callback);

int receive_USBcable_type(void)
{
        u32 cable_status;
        struct battery_info_reply tmp_batt_info;

        BAT_DBG("%s\n", __func__);

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        cable_status = tmp_batt_info.cable_status ;

        return cable_status; 
}
EXPORT_SYMBOL(receive_USBcable_type);

static int asus_battery_get_property(struct power_supply *psy,
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret = 0, tmp = 0;
        struct battery_info_reply tmp_batt_info;

        //BAT_DBG("%s 0x%04X\n", __func__, psp);

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        switch (psp) {
        case POWER_SUPPLY_PROP_STATUS:
                val->intval = tmp_batt_info.status;
		//BAT_DBG("%s: battery satus=%d, cable status=%d\n",__func__, val->intval, batt_info.cable_status);
		if((batt_info.cable_status==NO_CABLE)&&(tmp_batt_info.status!=POWER_SUPPLY_STATUS_NOT_CHARGING)){
			BAT_DBG("no cable but status is not \"not charging\", update it\n");
			val->intval = POWER_SUPPLY_STATUS_NOT_CHARGING;
		}
                break;
        case POWER_SUPPLY_PROP_HEALTH:
                val->intval = POWER_SUPPLY_HEALTH_GOOD;
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                val->intval = tmp_batt_info.present;
                break;
        case POWER_SUPPLY_PROP_TECHNOLOGY:
                val->intval = POWER_SUPPLY_TECHNOLOGY_LIPO;
                break;
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
                val->intval = tmp_batt_info.batt_volt;
                //BAT_DBG("VOLTAGE_NOW = %d\n", tmp_batt_info.batt_volt);
                /* change the voltage unit from Milli Voltage (mV) to Micro Voltage (uV) */
                val->intval *= 1000;
                break;
        case POWER_SUPPLY_PROP_CURRENT_NOW:
                tmp = asus_battery_update_current_no_mutex();
                if (tmp >= 0) tmp_batt_info.batt_current = tmp - 0x10000;
        case POWER_SUPPLY_PROP_CURRENT_AVG:
                val->intval = tmp_batt_info.batt_current;
                /* change the current unit from Milli Ampere (mA) to Micro Ampere (uA) */
                val->intval *= 1000;
                //BAT_DBG("CURRENT_NOW = %d\n", tmp_batt_info.batt_current);
                break;
        case POWER_SUPPLY_PROP_ENERGY_NOW:
                val->intval = tmp_batt_info.batt_energy;
                //BAT_DBG("ENERGY_NOW = %d\n", tmp_batt_info.batt_energy);
                break;
        case POWER_SUPPLY_PROP_CAPACITY:
                if (tmp_batt_info.status == POWER_SUPPLY_STATUS_FULL) {
                    val->intval = 100;
                    //BAT_DBG("CAPACITY = %d\n", val->intval);
                } else {
                    val->intval = tmp_batt_info.percentage;
                    //BAT_DBG("CAPACITY = %d\n", tmp_batt_info.percentage);
                }
                break;
        case POWER_SUPPLY_PROP_CAPACITY_LEVEL:
                val->intval = tmp_batt_info.level;
                break;
        case POWER_SUPPLY_PROP_MANUFACTURER:
        //case POWER_SUPPLY_PROP_BATTERY_ID:
                val->strval = tmp_batt_info.manufacturer;
                break;
        case POWER_SUPPLY_PROP_MODEL_NAME:
                val->strval = tmp_batt_info.model;
                break;
        case POWER_SUPPLY_PROP_SERIAL_NUMBER:
        //case POWER_SUPPLY_PROP_FIRMWARE_VERSION:
                val->strval = tmp_batt_info.serial_number;
                break;
        case POWER_SUPPLY_PROP_TEMP:
                val->intval = tmp_batt_info.batt_temp;
                //BAT_DBG("TEMP = %d\n", tmp_batt_info.batt_temp);
                break;
        case POWER_SUPPLY_PROP_CHARGE_FULL:
                val->intval = tmp_batt_info.batt_fcc;
                break;
        case POWER_SUPPLY_PROP_CHARGE_NOW:
                val->intval = tmp_batt_info.batt_rm;
                break;
        default:
                return -EINVAL;
        }

        return ret;
}

static int asus_power_get_property(struct power_supply *psy, 
                enum power_supply_property psp,
                union power_supply_propval *val)
{
        int ret;
        struct battery_info_reply tmp_batt_info;
        u32 cable_status;
        u32 drv_status;

        mutex_lock(&batt_info_mutex);
        tmp_batt_info = batt_info;
        mutex_unlock(&batt_info_mutex);

        cable_status = tmp_batt_info.cable_status;
        drv_status = tmp_batt_info.drv_status;

        ret = 0;
        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                if (drv_status == DRV_SHUTDOWN) {
                        val->intval = 0;

                }else if (drv_status == DRV_BATTERY_LOW) {
                        val->intval = 0;

#if 0
                } else if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS) {
                        val->intval = ((cable_status == USB_ADAPTER || 
                                        cable_status == USB_PC) ? 1 : 0);
#endif
                } else if (psy->type == POWER_SUPPLY_TYPE_USB) {
                        val->intval = (cable_status == USB_PC) ? 1 : 0;

                } else if (psy->type == POWER_SUPPLY_TYPE_MAINS) {
                        val->intval = (cable_status == USB_ADAPTER) ? 1 : 0;

                } else {
                        ret = -EINVAL;
                }
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                if (psy->type == POWER_SUPPLY_TYPE_USB || psy->type == POWER_SUPPLY_TYPE_MAINS) {
                    /* for ATD test to acquire the status about charger ic */
#if defined(CONFIG_A500CG_BATTERY_SMB347)
                    if (!smb347_has_charger_error() || smb347_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING)
#elif defined(CONFIG_SMB1357_CHARGER)
                    if (!smb1357_has_charger_error() || smb1357_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING)
#else
		  if(0)
#endif
                        val->intval = 1;
                    else
                        val->intval = 0;
                } else {
                    ret = -EINVAL;
                }
                break;
        default:
                ret = -EINVAL;
        }

        return ret;
}


int asus_register_power_supply(struct device *dev, struct dev_func *tbl)
{
        int ret;
        int test_flag=0;
        drv_status_t drv_status;

        BAT_DBG("%s\n", __func__);

        mutex_lock(&batt_info_mutex);
        drv_status = batt_info.drv_status;
        test_flag = batt_info.test_flag;
        mutex_unlock(&batt_info_mutex);

        if (!dev) {
                BAT_DBG_E("%s, device pointer is NULL.\n", __func__);
                return -EINVAL;
        } else if (!tbl) {
                BAT_DBG_E("%s, dev_func pointer is NULL.\n", __func__);
                return -EINVAL;
        }else if (drv_status != DRV_INIT_OK) {
                BAT_DBG_E("%s, asus_battery not init ok.\n", __func__);
                return -EINVAL;
        } else if (test_flag & TEST_INFO_NO_REG_POWER) {
                BAT_DBG_E("%s, Not register power class.\n", __func__);
                return 0;
        }

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_REGISTER;
        batt_info.tbl = tbl;
        mutex_unlock(&batt_info_mutex);

        //register to power supply driver
        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_BATTERY]);
        if (ret) { BAT_DBG_E("Fail to register battery\n"); goto batt_err_reg_fail_battery; }
#if 0 // register in charger driver
        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_USB]);
        if (ret) { BAT_DBG_E("Fail to register USB\n"); goto batt_err_reg_fail_usb; }

        ret = power_supply_register(dev, &asus_power_supplies[CHARGER_AC]);
        if (ret) { BAT_DBG_E("Fail to register AC\n"); goto batt_err_reg_fail_ac; }
#endif

        /* init wake lock in COS */
        if (boot_mode == 4) {
            BAT_DBG("%s: wake lock init: asus_battery_power_wakelock\n", __func__);
            wake_lock_init(&wakelock, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock");
            wake_lock_init(&wakelock_t, WAKE_LOCK_SUSPEND, "asus_battery_power_wakelock_timeout");
        }

        /* prevent system from entering s3 in COS while AC charger is connected */
        if (boot_mode == 4) {
            if (batt_info.cable_status == USB_ADAPTER) {
                if (!wake_lock_active(&wakelock)) {
                    BAT_DBG("%s: asus_battery_power_wakelock -> wake lock\n", __func__);
                    wake_lock(&wakelock);
                }
            }
        }

        //first update current information
        mutex_lock(&batt_info_mutex);
        asus_battery_get_info_no_mutex();
        batt_info.drv_status = DRV_REGISTER_OK;
        mutex_unlock(&batt_info_mutex);

        //start working 
        queue_delayed_work(battery_work_queue, &battery_poll_data_work, 
                        BATTERY_DEFAULT_POLL_TIME);

        BAT_DBG("%s register OK.\n", __func__);
        return 0;
#if 0 // register in charger driver
batt_err_reg_fail_ac:
        power_supply_unregister(&asus_power_supplies[CHARGER_AC]);
batt_err_reg_fail_usb:
        power_supply_unregister(&asus_power_supplies[CHARGER_USB]);
#endif
batt_err_reg_fail_battery:
        power_supply_unregister(&asus_power_supplies[CHARGER_BATTERY]);

        return ret;
}
EXPORT_SYMBOL(asus_register_power_supply);

static int __init setup_boot_mode(char *buf)
{
	if (!buf)
		return 0;

	boot_mode=0;
	while (*buf != '\0') {
		if (!strncmp(buf, "main", 4)) {
			boot_mode=1;
		}
		if (!strncmp(buf, "fota", 4)) {
			boot_mode=2;
		}
		if (!strncmp(buf, "fastboot", 8)) {
			boot_mode=3;
		}
		if (!strncmp(buf, "charger", 7)) {
			boot_mode=4;
		}
		buf++;
	}
	BAT_DBG("boot_mode=%d\n", boot_mode);
	return 0;
}
early_param("androidboot.mode", setup_boot_mode);

int asus_battery_init(
        u32 polling_time, 
        u32 critical_polling_time, 
        u32 test_flag
)
{
        int ret=0;
        drv_status_t drv_sts;    
	uint8_t data;

        BAT_DBG("%s, %d, %d, 0x%08X\n", __func__, polling_time, critical_polling_time, test_flag);

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        if (drv_sts != DRV_NOT_READY) {
                //other battery device registered.
                BAT_DBG_E("Error!! Already registered by other driver\n");
                ret = -EINVAL;
                if (test_flag & TEST_INFO_NO_REG_POWER) {
                        ret = 0;
                }
                                
                goto already_init;

        }

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_INIT;
        batt_info.polling_time = polling_time > (5*HZ) ? polling_time : BATTERY_DEFAULT_POLL_TIME;
        batt_info.critical_polling_time = critical_polling_time > (3*HZ) ? critical_polling_time : BATTERY_CRITICAL_POLL_TIME;
        batt_info.critical_polling_time = BATTERY_CRITICAL_POLL_TIME;
        batt_info.percentage = 50;
        batt_info.batt_temp = 250;
        batt_info.present = 1;
        batt_info.test_flag = test_flag;
        if (test_flag)
                BAT_DBG("test_flag: 0x%08X\n", test_flag);

        mutex_unlock(&batt_info_mutex);

        if (test_flag & TEST_INFO_NO_REG_POWER) {
                BAT_DBG_E("Not allow initiallize  power class. Skip ...\n");
                ret = 0;

                mutex_lock(&batt_info_mutex);
                batt_info.drv_status = DRV_INIT_OK;
                mutex_unlock(&batt_info_mutex);

                goto already_init;
        }

#ifdef CONFIG_PROC_FS
        ret = asus_battery_register_proc_fs();
        if (ret) {
                BAT_DBG_E("Unable to create proc asus_battery_register_proc_fs\n");
                goto proc_fail;
        }
#endif

        battery_work_queue = create_singlethread_workqueue("asus_battery");
        if(battery_work_queue == NULL)
        {
                BAT_DBG_E("Create battery thread fail");
                ret = -ENOMEM;
                goto error_workq;
        }
        INIT_DELAYED_WORK(&battery_poll_data_work, asus_polling_data);
        INIT_DELAYED_WORK(&detect_cable_work, USB_cable_status_worker);

#ifdef CONFIG_ASUS_FACTORY_MODE
#if CONFIG_PROC_FS
        ret = init_asus_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create proc init_asus_charging_toggle\n");
                goto proc_fail;
        }
        ret = init_asus_for_gague_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create init_asus_for_gague_charging_toggl\n");
                goto proc_fail;
        }
#endif
#endif

#if CONFIG_PROC_FS
        //+++Sewell+++charging_toggle interface
        ret = init_thermal_charging_toggle();
        if (ret) {
                BAT_DBG_E("Unable to create proc init_thermal_charging_toggle\n");
                goto proc_fail;
        }
        //---Sewell---
        ret = init_thermal_disable_qc();
        if (ret) {
                BAT_DBG_E("Unable to create proc init_thermal_disable_qc\n");
                goto proc_fail;
        }
#endif

        mutex_lock(&batt_info_mutex);
        batt_info.drv_status = DRV_INIT_OK;
        mutex_unlock(&batt_info_mutex);

	//set PMIC voltage drop warning
	intel_scu_ipc_iowrite8(VWARN1_CFG_REG, 0xFF);
	if(Read_PROJ_ID()==PROJ_ID_ZX550ML) {
		//set PMIC VPROG2 1V8 default on in ZX550ML
		intel_scu_ipc_iowrite8(VPROG2_CFG_REG, 0x4B);
	}
	//set power key pressed for HW shutdown to 8s
	ret = intel_scu_ipc_ioread8(PBCONFIG_REG, &data);
	if (ret) {
		BAT_DBG_E(" IPC Failed to read PBCONFIG_REG: %d\n", ret);
	}
	data |= (BIT(3));
	data &= ~(BIT(2)|BIT(1)|BIT(0));
	ret = intel_scu_ipc_iowrite8(PBCONFIG_REG, data);
	if (ret) {
		BAT_DBG_E(" IPC Failed to write PBCONFIG_REG: %d\n", ret);
	}
#ifdef CONFIG_SMB1357_CHARGER
	/* register switch device for invalid charger status */
	charger_dev.name = "invalid_charger";
	if (switch_dev_register(&charger_dev) < 0) {
		BAT_DBG_E("%s: fail to register charger switch\n", __func__);
	} else {
		switch_set_state(&charger_dev, 0);
	}
#endif
        BAT_DBG("%s: success\n", __func__);

        return 0;

error_workq:    
#if CONFIG_PROC_FS
proc_fail:
#endif
already_init:
        return ret;
}
EXPORT_SYMBOL(asus_battery_init);

void asus_battery_exit(void)
{
        drv_status_t drv_sts;    

        BAT_DBG("Driver unload\n");

        mutex_lock(&batt_info_mutex);
        drv_sts = batt_info.drv_status;
        mutex_unlock(&batt_info_mutex);

        if (drv_sts == DRV_REGISTER_OK) {
                power_supply_unregister(&asus_power_supplies[CHARGER_BATTERY]);
                //power_supply_unregister(&asus_power_supplies[CHARGER_USB]);
                //power_supply_unregister(&asus_power_supplies[CHARGER_AC]);
                if (boot_mode == 4)
                    wake_lock_destroy(&wakelock);
        }
#ifdef CONFIG_SMB1357_CHARGER
	switch_dev_unregister(&charger_dev);
#endif
        BAT_DBG("Driver unload OK\n");
}

static int __init asus_battery_fake_init(void)
{
        return 0;
}
late_initcall(asus_battery_fake_init);

static void __exit asus_battery_fake_exit(void)
{
        /* SHOULD NOT REACHE HERE */
        BAT_DBG("%s exit\n", __func__);
}
module_exit(asus_battery_fake_exit);

MODULE_AUTHOR("chris1_chang@asus.com");
MODULE_DESCRIPTION("battery driver");
MODULE_LICENSE("GPL");

