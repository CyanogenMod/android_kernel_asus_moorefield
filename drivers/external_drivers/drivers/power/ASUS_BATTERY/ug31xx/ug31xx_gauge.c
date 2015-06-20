/*
 * Copyright (c) 2012, uPI Semiconductor Corp. All Rights Reserved.
 */
#include <linux/init.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/param.h>
#include <linux/i2c.h>
#include <linux/delay.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/power_supply.h>
#include <linux/idr.h>
#include <linux/reboot.h>
#include <linux/notifier.h>
#include <linux/jiffies.h>
#include <linux/err.h>
#include <asm/unaligned.h>
#include <linux/wakelock.h>
#include <linux/kobject.h>
#include <linux/string.h>
#include <linux/sysfs.h>
#include <linux/earlysuspend.h>

#include "ug31xx_ggb_data_A500.h"
#include "ug31xx_ggb_data_A500-2.h"
#include "ug31xx_ggb_data_A600.h"

#include "uG31xx_Platform.h"
#include "ug31xx_gauge.h"
#include "uG31xx_API_Platform.h"
#include <linux/ioctl.h>
#include <asm/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/proc_fs.h>

#include <asm/intel_mid_gpadc.h>
#include <asm/intel_mid_thermal.h>

#include "../smb347_external_include.h"
#include "../asus_battery.h"
#include <linux/HWVersion.h>
#include <linux/switch.h>

#include <linux/timer.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,8,0)
#ifdef CONFIG_ANDROID
#include <../drivers/staging/android/android_alarm.h>
#endif
#else
#include <linux/android_alarm.h>
#endif

extern int Read_HW_ID(void);
struct switch_dev batt_dev;
extern int Read_PROJ_ID(void);

//#define	UPI_CALLBACK_FUNC	            ///< [AT-PM] : Used for removing callback function ; 04/15/2013
#define	UG31XX_DYNAMIC_POLLING	      ///< [AT-PM] : Used for dynamic polling time ; 04/30/2013
//#define UG31XX_WAIT_CHARGER_FC        ///< [AT-PM] : Used for full charge status decided by charger ; 07/25/2013
//#define UG31XX_DYNAMIC_TAPER_CURRENT  ///< [AT-PM] : Used for changing taper current threshold ; 09/01/2013
#define UG31XX_REGISTER_I2C     ///< [AT-PM] : Used for register I2C inside module ; 09/03/2013
//#define UG31XX_REGISTER_POWERSUPPLY   ///< [AT-PM] : Used for register powersupply class ; 09/05/2013
#define UG31XX_SHOW_EXT_TEMP    ///< [FC] : Set temperature reference by external ; 09/30/2013
#define UG31XX_MISC_DEV
#define UG31XX_PROBE_CHARGER_OFF      ///< [AT-PM] : Used for charger off at probe ; 12/06/2013
#define UG31XX_EARLY_SUSPEND			///< [AT-PM] : Used for early suspend instead of suspend ; 12/07/2013
#define UG31XX_KOBJECT      ///< [AT-PM] : Used for register kobject ; 12/23/2013
#define UG31XX_USER_SPACE_ALGORITHM     ///< [AT-PM] : Used for user space algorithm operation ; 12/24/2013
#define UG31XX_USER_SPACE_BACKUP        ///< [AT-PM] : Used for user space backup operation ; 12/30/2013

//TODO: create_proc_read_entry in UG31XX_PROC_DEV token already be deprecated.  <Wade+>
//#define UG31XX_PROC_DEV
//TODO: code in UG31XX_WAKEUP_ALARM token need upi porting again <Wade+>
//#define UG31XX_WAKEUP_ALARM ///< [FC] : wakeup alarm functions ; 02/08/2014

#define UG31XX_USER_DAEMON_VER_LENGTH   (16)
#define UG31XX_CALI_BO_LOW_TEMP         (100)
#define UG31XX_CALI_BO_HIGH_TEMP        (450)

#define UG31XX_WAKEUP_ALARM_TIME        (40*60)  ///< [FC] : wakeup alarm 40 min ; 02/08/2014

struct ug31xx_gauge {
#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND)
        struct early_suspend es;
#endif	///< end of defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND) 
        struct i2c_client       *client;
        struct device           *dev;
        struct delayed_work     batt_info_update_work;
        struct delayed_work     batt_power_update_work;
        struct delayed_work     batt_probe_work;
        struct delayed_work     board_offset_cali_work;
        struct delayed_work     shell_algorithm_work;
        struct delayed_work     shell_backup_work;
        struct delayed_work     shell_timeout_work;
        struct wake_lock        batt_wake_lock;
        struct wake_lock        shell_algorithm_wake_lock;
        struct wake_lock        shell_timeout_wake_lock;
        struct mutex            info_update_lock;

        u32 cable_status;
        u32 polling_time;         ///< [AT-PM] : unit in second ; 12/10/2013
        u32 update_time;          ///< [AT-PM] : unit in second ; 12/10/2013
        u32 batt_volt;            ///< [AT-PM] : unit in mV ; 10/07/2013
        int batt_capacity;        ///< [AT-PM] : unit in % ; 10/07/2013
        int batt_capacity_last;   ///< [AT-PM] : unit in % ; 10/07/2013
        int batt_capacity_real;   ///< [AT-PM] : unit in % ; 11/27/2013
        u32 batt_charge_now;      ///< [AT-PM] : unit in mAh ; 10/07/2013
        u32 batt_charge_full;     ///< [AT-PM] : unit in mAh ; 10/07/2013
        int batt_charge_counter;  ///< [AT-PM] : unit in mAh ; 02/18/2014
        int batt_current;         ///< [AT-PM] : unit in mA ; 10/07/2013
        int batt_current_last;    ///< [AT-PM] : unit in mA ; 10/07/2013
        int batt_temp;            ///< [AT-PM] : unit in 0.1oC ; 10/07/2013
        int batt_avg_temp;        ///< [AT-PM] : unit in 0.1oC ; 11/27/2013
        int batt_ready;
        int batt_remove;
        int batt_alarm_sts;
        int batt_status;
        int batt_cycle_count;
        int batt_ntc_sts;
        int board_offset;
        char effective_board_offset[8];
        char daemon_ver[UG31XX_USER_DAEMON_VER_LENGTH];
        int daemon_uevent_count;

#ifdef UG31XX_WAKEUP_ALARM
        struct alarm wakeup_alarm;
        ktime_t last_poll;
#endif  ///<end of UG31XX_WAKEUP_ALARM
};

static void batt_info_update_work_func(struct work_struct *work);

#define CABLE_STATUS_CHANGE_RELEASE_COUNT (1)

#define GAUGE_err(...)        printk(KERN_ERR "[GAUGE_ERR] " __VA_ARGS__);
#define GAUGE_notice(...)     printk(KERN_NOTICE "[GAUGE] " __VA_ARGS__);
#define GAUGE_info(...)       printk(KERN_INFO "[GAUGE] " __VA_ARGS__);

#ifdef UG31XX_MISC_DEV

#define UG31XX_IOC_MAGIC 'U'
#define UG31XX_IOCTL_RESET                    _IO(UG31XX_IOC_MAGIC, 1)
#define UG31XX_IOCTL_ENABLE_SUSPEND_LOG       _IO(UG31XX_IOC_MAGIC, 2)
#define UG31XX_IOCTL_ENABLE_LOG               _IO(UG31XX_IOC_MAGIC, 3)
#define UG31XX_IOCTL_DISABLE_LOG              _IO(UG31XX_IOC_MAGIC, 4)
#define UG31XX_IOCTL_I2C_READ                 _IOWR(UG31XX_IOC_MAGIC, 5, unsigned char *)
#define UG31XX_IOCTL_I2C_WRITE                _IOWR(UG31XX_IOC_MAGIC, 6, unsigned char *)
#define UG31XX_IOCTL_RESET_CYCLE_COUNT        _IO(UG31XX_IOC_MAGIC, 7)
#define UG31XX_IOCTL_BACKUP_SIZE              _IOWR(UG31XX_IOC_MAGIC, 8, unsigned char *)
#define UG31XX_IOCTL_BACKUP_READ              _IOWR(UG31XX_IOC_MAGIC, 9, unsigned char *)
#define UG31XX_IOCTL_BACKUP_WRITE             _IOWR(UG31XX_IOC_MAGIC, 10, unsigned char *)
#define UG31XX_IOCTL_DAEMON_GET_CNTL          _IOWR(UG31XX_IOC_MAGIC, 11, unsigned char *)
#define UG31XX_IOCTL_DAEMON_SET_CNTL          _IOWR(UG31XX_IOC_MAGIC, 12, unsigned char *)
#define UG31XX_IOCTL_DAEMON_DELAY             _IOWR(UG31XX_IOC_MAGIC, 13, unsigned char *)
#define UG31XX_IOCTL_DAEMON_FILENAME          _IOWR(UG31XX_IOC_MAGIC, 14, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_SIZE           _IOWR(UG31XX_IOC_MAGIC, 15, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_START          _IOWR(UG31XX_IOC_MAGIC, 16, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_UPDATE         _IOWR(UG31XX_IOC_MAGIC, 17, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_OPTION         _IOWR(UG31XX_IOC_MAGIC, 18, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_BACKUP         _IOWR(UG31XX_IOC_MAGIC, 19, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_BACKUP_BUFFER  _IOWR(UG31XX_IOC_MAGIC, 20, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_BACKUP_SIZE    _IOWR(UG31XX_IOC_MAGIC, 21, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_TABLE_SIZE     _IOWR(UG31XX_IOC_MAGIC, 22, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_TABLE_DATA     _IOWR(UG31XX_IOC_MAGIC, 23, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_TABLE_BUF      _IOWR(UG31XX_IOC_MAGIC, 24, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_VERSION        _IOWR(UG31XX_IOC_MAGIC, 25, unsigned char *)
#define UG31XX_IOCTL_ALGORITHM_LOCK           _IO(UG31XX_IOC_MAGIC, 26)
#define UG31XX_IOCTL_ALGORITHM_UNLOCK         _IO(UG31XX_IOC_MAGIC, 27)

#endif	///< end of UG31XX_MISC_DEV

#ifdef  UG31XX_CELL_REPLACE_TEST

#define UG31XX_WAKE_LOCK_TIMEOUT      		(60)

#else   ///< else of UG31XX_CELL_REPLACE_TEST

#define UG31XX_WAKE_LOCK_TIMEOUT      		(1)

#endif  ///< end of UG31XX_CELL_REPLACE_TEST

static struct dev_func ug31xx_tbl;

#define	UG31XX_DRV_NOT_READY		(DRV_NOT_READY)
#define	UG31XX_DRV_INIT_OK			(DRV_INIT_OK)
#define	UG31XX_DRV_SUSPEND			(DRV_INIT_OK + 1)

#define	UG31XX_NO_CABLE				(NO_CABLE)
#define	UG31XX_USB_PC_CABLE			(USB_PC_CABLE)
#define	UG31XX_AC_ADAPTER_CABLE		(AC_ADAPTER_CABLE)

/* Functions Declaration */
#ifdef	UG31XX_REGISTER_POWERSUPPLY

static int ug31xx_battery_get_property(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val);

#endif	///< end of UG31XX_REGISTER_POWERSUPPLY

#ifdef	UPI_CALLBACK_FUNC

static int ug31xx_power_get_property(struct power_supply *psy,
                                     enum power_supply_property psp,
                                     union power_supply_propval *val);

#else	///< else of UPI_CALLBACK_FUNC

#ifdef	UG31XX_REGISTER_POWERSUPPLY

static void ug31xx_battery_external_power_changed(struct power_supply *psy);

#endif	///< end of UG31XX_REGISTER_POWERSUPPLY

#endif	///< end of UPI_CALLBACK_FUNC

#ifdef	UG31XX_REGISTER_POWERSUPPLY

static int ug31xx_update_psp(enum power_supply_property psp,
                             union power_supply_propval *val);

#endif	///< end of UG31XX_REGISTER_POWERSUPPLY

/* Extern Function */
static char *FactoryGGBXFile;
extern char FactoryGGBXFile_A500[];
extern char FactoryGGBXFile_A500_2[];
extern char FactoryGGBXFile_A600[];

/* Global Variables */
static struct ug31xx_gauge *ug31 = NULL;
unsigned char cur_cable_status = UG31XX_NO_CABLE;
int ug31xx_drv_status = UG31XX_DRV_NOT_READY;
static bool charger_detect_full = false;
static bool curr_charger_full_status = false;
static int charge_termination_current = 0;          ///< [AT-PM] : Set charging termination current (0 = from GGB settings) ; 09/05/2013
static int cable_status_changed = 0;
static bool force_power_supply_change = true;
static bool charger_full_status = false;
static bool charger_dc_in_before_suspend = false;
static unsigned char op_options = LKM_OPTIONS_DEBUG_ERROR;	///< [AT-PM] : Set to "LKM_OPTIONS_ENABLE_REVERSE_CURRENT" to enable reverse current direction feature ; 11/12/2013
static int rsense_value = 0;                        ///< [AT-PM] : Set R-Sense value (0 = from GGB settings) ; 09/05/2013
static int ug31xx_backup_file_status = 0;
static unsigned short design_capacity = 0;
static int kbo_result = 0;
static bool probe_with_cable = false;
static bool in_early_suspend = false;
static int delta_q_in_suspend = 0;
static int rsoc_before_suspend = 0;
static bool user_space_algorithm_response = true;
static int user_space_algorithm_prev_fc_sts = 0;
static int user_space_algorithm_now_fc_sts = 0;
static bool user_space_in_progress = false;
static bool force_update_backup_file = false;
static bool enable_board_offset_cali_at_eoc = false;
static bool board_offset_cali_finish = false;
static int ggb_board_offset = 0;
static int ntc_offset = 0;
static int standby_current = 0;
static int ggb_board_gain = 1000;
static int hw_id_flag=0;	/*hw id, 0:evb, 1:sr1*/

#define CHARGER_LIST_NUM      (1)
//static char *charger_list[] = {"ac"};

static void get_ggb_array(void)
{
        /*read battery ID*/
        uint32_t bat_adc_value[1], bat_adc_channel[1];
        void *bat_adc_handle;
        int ret, vol_temp=0;

        bat_adc_channel[0] = 0x06 | CH_NEED_VREF | CH_NEED_VCALIB;
        bat_adc_handle = gpadc_alloc_channels(1, bat_adc_channel);
        if(!bat_adc_handle) {
                GAUGE_info("gpadc_alloc_channels fail\n");
        } else {
                ret = get_gpadc_sample(bat_adc_handle, 1, bat_adc_value);
                if(ret) {
                        GAUGE_info("get_gpadc_sample\n");
                } else {
                        vol_temp = (int)bat_adc_value[0];
                        GAUGE_info("battery ID pin ADC value=0x%x, decimal value=%d, voltage=%dmV\n", bat_adc_value[0], vol_temp, vol_temp*1500/1023);
                }
        }

        if(PROJ_ID_A600CG == Read_PROJ_ID()) {
                GAUGE_info("%s: use 600 ggb data\n", __func__);
                FactoryGGBXFile = FactoryGGBXFile_A600;
        } else {
                if(vol_temp > 915) { //1.343V
                        GAUGE_info("%s: use 500 ggb data 2, battery = COSLIGHT\n", __func__);
                        FactoryGGBXFile = FactoryGGBXFile_A500_2;
                } else if(vol_temp > 778) { //1.141V
                        GAUGE_info("%s: use 500 ggb data 1, battery = ATL \n", __func__);
                        FactoryGGBXFile = FactoryGGBXFile_A500;
                } else {
                        GAUGE_info("%s: use 500 ggb data, battery = LG \n", __func__);
                        FactoryGGBXFile = FactoryGGBXFile_A500;
                }
        }
}

static void set_project_config(void)
{
}

static void update_project_config(void)
{
        if(ug31_module.get_charge_termination_current() != charge_termination_current) {
                ug31_module.set_charge_termination_current(charge_termination_current);
        }
        if(ug31_module.get_rsense() != rsense_value) {
                ug31_module.set_rsense(rsense_value);
        }
        if(ug31_module.get_ggb_board_offset() != ggb_board_offset) {
                ug31_module.set_ggb_board_offset(ggb_board_offset);
        }
        if(ug31_module.get_ntc_offset() != ntc_offset) {
                ug31_module.set_ntc_offset(ntc_offset);
        }
        if(ug31_module.get_standby_current() != standby_current) {
                ug31_module.set_standby_current(standby_current);
        }
        if(ug31_module.get_ggb_board_gain() != ggb_board_gain) {
                ug31_module.set_ggb_board_gain(ggb_board_gain);
        }
}

static bool is_charging_full(void)
{
#if 0
        struct power_supply *ug31_charger;
        union power_supply_propval val_charger;
        int ug31_charger_idx;

        /// [AT-PM] : Check charger full statue ; 07/30/2013
        ug31_charger_idx = 0;
        while(ug31_charger_idx < CHARGER_LIST_NUM) {
                ug31_charger = power_supply_get_by_name(charger_list[ug31_charger_idx]);
                if(ug31_charger != NULL) {
                        ug31_charger->get_property(ug31_charger, POWER_SUPPLY_PROP_STATUS, &val_charger);
                        if(val_charger.intval == POWER_SUPPLY_STATUS_FULL) {
                                return (true);
                        }
                }
                ug31_charger_idx = ug31_charger_idx + 1;
        }
        return (false);
#endif
        if(smb347_get_charging_status() == POWER_SUPPLY_STATUS_FULL) {
                return (true);
        } else {
                return (false);
        }
}

static bool is_charging(void)
{
#if 0
        struct power_supply *ug31_charger;
        union power_supply_propval val_charger;
        int ug31_charger_idx;

        /// [AT-PM] : Check charger full statue ; 07/30/2013
        ug31_charger_idx = 0;
        while(ug31_charger_idx < CHARGER_LIST_NUM) {
                ug31_charger = power_supply_get_by_name(charger_list[ug31_charger_idx]);
                if(ug31_charger != NULL) {
                        ug31_charger->get_property(ug31_charger, POWER_SUPPLY_PROP_STATUS, &val_charger);
                        if(val_charger.intval == POWER_SUPPLY_STATUS_FULL) {
                                return (true);
                        }
                        if(val_charger.intval == POWER_SUPPLY_STATUS_CHARGING) {
                                return (true);
                        }
                }
                ug31_charger_idx = ug31_charger_idx + 1;
        }
        return (false);
#endif
        if((smb347_get_charging_status() == POWER_SUPPLY_STATUS_CHARGING) ||
            (smb347_get_charging_status() == POWER_SUPPLY_STATUS_FULL)) {
                return (true);
        } else {
                return (false);
        }
}

#ifdef UG31XX_MISC_DEV

static int ug31xx_misc_open(struct inode *inode, struct file *file)
{
        UG31_LOGN("[%s]\n", __func__);
        return 0;
}

static int ug31xx_misc_release(struct inode *inode, struct file *file)
{
        UG31_LOGN("[%s]\n", __func__);
        return 0;
}

static long ug31xx_misc_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
        int rc = 0;
        unsigned char val[2];
        unsigned char data = 0;
        unsigned char *backup_buf;
        int backup_size;
        unsigned char backup_file[256];

        switch (cmd) {
        case UG31XX_IOCTL_RESET:
                UG31_LOGN("[%s] cmd -> RESET\n", __func__);
                op_options = op_options | LKM_OPTIONS_FORCE_RESET;
                break;

        case UG31XX_IOCTL_ENABLE_SUSPEND_LOG:
                UG31_LOGN("[%s] cmd -> ENABLE_SUSPEND_LOG\n", __func__);
                op_options = op_options | LKM_OPTIONS_ENABLE_SUSPEND_DATA_LOG;
                break;

        case UG31XX_IOCTL_ENABLE_LOG:
                UG31_LOGN("[%s] cmd -> ENABLE_LOG\n", __func__);
                op_options = op_options | LKM_OPTIONS_ENABLE_DEBUG_LOG;
                break;

        case UG31XX_IOCTL_DISABLE_LOG:
                UG31_LOGN("[%s] cmd -> DISABLE_LOG\n", __func__);
                op_options = op_options & (~LKM_OPTIONS_ENABLE_DEBUG_LOG);
                break;

        case UG31XX_IOCTL_I2C_READ:
                UG31_LOGN("[%s] cmd -> I2C_READ\n", __func__);
                if(copy_from_user((void *)&val[0], (void __user *)arg, sizeof(unsigned char))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_from_user fail\n", __func__);
                        break;
                }
                UG31_LOGN("[%s] read addr: 0x%x\n", __func__, val[0]);
                if(ug31_module.ug31xx_i2c_read(val[0], &data)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] I2C read fail\n", __func__);
                        break;
                }
                UG31_LOGN("[%s] read data: 0x%x\n", __func__, data);
                if(copy_to_user((void __user *)arg, (void *)&data, sizeof(unsigned char))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                        break;
                }
                break;

        case UG31XX_IOCTL_I2C_WRITE:
                UG31_LOGN("[%s] cmd -> I2C_WRITE\n", __func__);
                if(copy_from_user((void *)&val[0], (void __user *)arg, sizeof(val))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_from_user fail\n", __func__);
                        break;
                }
                UG31_LOGN("[%s] write addr: 0x%x, data: 0x%x\n", __func__, val[0], val[1]);
                if(ug31_module.ug31xx_i2c_write(val[0], &val[1])) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] I2C write fail\n", __func__);
                        break;
                }
                break;

        case UG31XX_IOCTL_RESET_CYCLE_COUNT:
                UG31_LOGN("[%s] cmd -> RESET_CYCLE_COUNT\n", __func__);
                rc = ug31_module.reset_cycle_count();
                break;

        case UG31XX_IOCTL_BACKUP_SIZE:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_BACKUP_SIZE\n", __func__);
                backup_buf = ug31_module.get_backup_buffer(&backup_size);
                data = (unsigned char)backup_size;
                if(copy_to_user((void __user *)arg, (void *)&data, sizeof(unsigned char))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_BACKUP_READ:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_BACKUP_READ\n", __func__);
                backup_buf = ug31_module.get_backup_buffer(&backup_size);
                if(copy_to_user((void __user *)arg, (void *)backup_buf, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_BACKUP_WRITE:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_BACKUP_WRITE\n", __func__);
                backup_buf = ug31_module.get_backup_buffer(&backup_size);
                if(copy_from_user((void *)backup_buf, (void __user *)arg, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_from_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_DAEMON_GET_CNTL:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_DAEMON_GET_CNTL\n", __func__);
                mutex_lock(&ug31->info_update_lock);
                data = ug31_module.get_backup_daemon_cntl();
                mutex_unlock(&ug31->info_update_lock);
                if(copy_to_user((void __user *)arg, (void *)&data, 1)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_DAEMON_SET_CNTL:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_DAEMON_SET_CNTL\n", __func__);
                if(copy_from_user((void *)&data, (void __user *)arg, 1)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_from_user fail\n", __func__);
                }
                mutex_lock(&ug31->info_update_lock);
                ug31_module.set_backup_daemon_cntl(data);
                mutex_unlock(&ug31->info_update_lock);
                break;

        case UG31XX_IOCTL_DAEMON_DELAY:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_DAEMON_DELAY\n", __func__);
                mutex_lock(&ug31->info_update_lock);
                data = ug31_module.get_backup_daemon_period();
                mutex_unlock(&ug31->info_update_lock);
                data = data - 1;
                if(copy_to_user((void __user *)arg, (void *)&data, 1)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_DAEMON_FILENAME:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_DAEMON_FILENAME\n", __func__);
                sprintf(backup_file, "%s", UPI_UG31XX_BACKUP_FILE);
                if(copy_to_user((void __user *)arg, (void *)backup_file, 256)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_SIZE:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_SIZE\n", __func__);
                backup_buf = ug31_module.shell_memory(&backup_size);
                val[0] = backup_size/256;
                val[1] = backup_size%256;
                if(copy_to_user((void __user *)arg, (void *)&val[0], sizeof(val))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_START:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_START\n", __func__);
                backup_buf = ug31_module.shell_memory(&backup_size);
                if(copy_to_user((void __user *)arg, (void *)backup_buf, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_UPDATE:
                UG31_LOGI("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_UPDATE\n", __func__);
                backup_buf = ug31_module.shell_memory(&backup_size);
                ug31_module.backup_pointer();
                if(copy_from_user((void *)backup_buf, (void __user *)arg, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_from_user fail\n", __func__);
                }
                ug31_module.restore_pointer();
                schedule_delayed_work(&ug31->shell_algorithm_work, 0*HZ);
                break;

        case UG31XX_IOCTL_ALGORITHM_OPTION:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_OPTION\n", __func__);
                if(copy_to_user((void __user *)arg, (void *)&op_options, sizeof(op_options))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_BACKUP:
                UG31_LOGI("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_BACKUP\n", __func__);
                backup_buf = ug31_module.shell_memory(&backup_size);
                ug31_module.backup_pointer();
                if(copy_from_user((void *)backup_buf, (void __user *)arg, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_from_user fail\n", __func__);
                }
                ug31_module.restore_pointer();
                schedule_delayed_work(&ug31->shell_backup_work, 0*HZ);
                break;

        case UG31XX_IOCTL_ALGORITHM_BACKUP_BUFFER:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_BACKUP_BUFFER\n", __func__);
                backup_buf = ug31_module.shell_backup_memory(&backup_size);
                if(copy_to_user((void __user *)arg, (void *)backup_buf, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_BACKUP_SIZE:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_BACKUP_SIZE\n", __func__);
                backup_buf = ug31_module.shell_backup_memory(&backup_size);
                val[0] = backup_size/256;
                val[1] = backup_size%256;
                if(copy_to_user((void __user *)arg, (void *)&val[0], sizeof(val))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_TABLE_SIZE:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_TABLE_SIZE\n", __func__);
                backup_buf = ug31_module.shell_table_memory(&backup_size);
                val[0] = backup_size/256;
                val[1] = backup_size%256;
                if(copy_to_user((void __user *)arg, (void *)&val[0], sizeof(val))) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_TABLE_DATA:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_TABLE_DATA\n", __func__);
                backup_buf = ug31_module.shell_table_memory(&backup_size);
                if(copy_to_user((void __user *)arg, (void *)backup_buf, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_TABLE_BUF:
                UG31_LOGN("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_TABLE_BUF\n", __func__);
                backup_buf = ug31_module.shell_table_buf_memory(&backup_size);
                if(copy_to_user((void __user *)arg, (void *)backup_buf, backup_size)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_to_user fail\n", __func__);
                }
                break;

        case UG31XX_IOCTL_ALGORITHM_VERSION:
                UG31_LOGE("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_VERSION\n", __func__);
                if(copy_from_user((void *)ug31->daemon_ver, (void __user *)arg, UG31XX_USER_DAEMON_VER_LENGTH)) {
                        rc = -EINVAL;
                        UG31_LOGE("[%s] copy_from_user fail\n", __func__);
                }
                ug31->daemon_uevent_count = 0;
                break;

        case UG31XX_IOCTL_ALGORITHM_LOCK:
                UG31_LOGI("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_LOCK\n", __func__);
                if(wake_lock_active(&ug31->shell_timeout_wake_lock) != 0) {
                        wake_unlock(&ug31->shell_timeout_wake_lock);
                }
                wake_lock(&ug31->shell_timeout_wake_lock);
                mutex_lock(&ug31->info_update_lock);
                user_space_in_progress = true;
                schedule_delayed_work(&ug31->shell_timeout_work, 1*HZ);
                break;

        case UG31XX_IOCTL_ALGORITHM_UNLOCK:
                UG31_LOGI("[%s] cmd -> UG31XX_IOCTL_ALGORITHM_UNLOCK\n", __func__);
                user_space_in_progress = false;
                mutex_unlock(&ug31->info_update_lock);
                if(wake_lock_active(&ug31->shell_timeout_wake_lock) != 0) {
                        wake_unlock(&ug31->shell_timeout_wake_lock);
                }
                break;

        default:
                UG31_LOGE("[%s] invalid cmd %d\n", __func__, _IOC_NR(cmd));
                rc = -EINVAL;
                break;
        }

        return (rc);
}

static struct file_operations ug31xx_fops = {
        .owner = THIS_MODULE,
        .open = ug31xx_misc_open,
        .release = ug31xx_misc_release,
        .unlocked_ioctl = ug31xx_misc_ioctl
};

struct miscdevice ug31xx_misc = {
        .minor = MISC_DYNAMIC_MINOR,
        .name = "ug31xx",
        .fops = &ug31xx_fops
};

#endif	///< end of UG31XX_MISC_DEV

#ifdef  UG31XX_KOBJECT

struct ug31xx_kobj {
        struct kobject kobj;
        int driver_version;
        int local_version;
};

#define to_ug31xx_kobj(x) container_of(x, struct ug31xx_kobj, kobj)

struct ug31xx_attribute {
        struct attribute attr;
        ssize_t (*show)(struct ug31xx_kobj *obj, struct ug31xx_attribute *attr, char *buf);
};

#define to_ug31xx_attr(x) container_of(x, struct ug31xx_attribute, attr)

static ssize_t ug31xx_attr_show(struct kobject *kobj, struct attribute *attr, char *buf)
{
        struct ug31xx_attribute *attribute;
        struct ug31xx_kobj *obj;

        attribute = to_ug31xx_attr(attr);
        obj = to_ug31xx_kobj(kobj);

        if(!attribute->show) {
                return (-EIO);
        }

        return (attribute->show(obj, attribute, buf));
}

static struct sysfs_ops ug31xx_sysfs_ops = {
        .show   = ug31xx_attr_show,
};

static void ug31xx_release(struct kobject *kobj)
{
        kfree(to_ug31xx_kobj(kobj));
}

static struct attribute *ug31xx_default_attrs[] = {
        NULL,
};

static struct kobj_type ug31xx_ktype = {
        .sysfs_ops      = &ug31xx_sysfs_ops,
        .release        = ug31xx_release,
        .default_attrs  = ug31xx_default_attrs,
};

enum UG31XX_KOBJ_ENV {
        UG31XX_KOBJ_ENV_UPDATE_CAPACITY = 0,
        UG31XX_KOBJ_ENV_BACKUP_DATA,
        UG31XX_KOBJ_ENV_COUNT,
};

static struct kset *ug31xx_set;
static struct ug31xx_kobj *ug31_kobj;
static int kobj_event_env = UG31XX_KOBJ_ENV_COUNT;

static void change_ug31xx_kobj(void)
{
        char *cmd1[] = {"OP_NAME=update_capacity", NULL};
        char *cmd2[] = {"OP_NAME=backup_data", NULL};
        char *cmd3[] = {"OP_NAME=backup_data", "OP_ACTION=write", NULL};

#ifdef  UG31XX_USER_SPACE_ALGORITHM

        if(wake_lock_active(&ug31->shell_algorithm_wake_lock) != 0) {
                wake_unlock(&ug31->shell_algorithm_wake_lock);
        }
        wake_lock_timeout(&ug31->shell_algorithm_wake_lock, UG31XX_WAKE_LOCK_TIMEOUT*HZ);

#endif  ///< end of UG31XX_USER_SPACE_ALGORITHM

        GAUGE_notice("[%s]: Send change uevent (%d).\n", __func__, kobj_event_env);

        switch(kobj_event_env) {
        case UG31XX_KOBJ_ENV_UPDATE_CAPACITY:
                user_space_algorithm_prev_fc_sts = ug31_module.get_full_charge_status();

                kobject_uevent_env(&ug31_kobj->kobj, KOBJ_CHANGE, cmd1);
                break;
        case UG31XX_KOBJ_ENV_BACKUP_DATA:
                if(force_update_backup_file == true) {
                        force_update_backup_file = false;
                        kobject_uevent_env(&ug31_kobj->kobj, KOBJ_CHANGE, cmd3);
                } else {
                        kobject_uevent_env(&ug31_kobj->kobj, KOBJ_CHANGE, cmd2);
                }
                break;
        default:
                break;
        }

        kobj_event_env = UG31XX_KOBJ_ENV_COUNT;
        ug31->daemon_uevent_count = ug31->daemon_uevent_count + 1;
}

static void create_ug31xx_kobj(void)
{
        int rtn;

        ug31xx_set = kset_create_and_add("upi", NULL, kernel_kobj);

        ug31_kobj = kzalloc(sizeof(struct ug31xx_kobj), GFP_KERNEL);
        if(!ug31_kobj) {
                GAUGE_err("[%s]: create ug31_kobj fail.\n", __func__);
                return;
        }

        ug31_kobj->kobj.kset = ug31xx_set;
        ug31_kobj->driver_version = 21;
        ug31_kobj->local_version = 0;

        rtn = kobject_init_and_add(&ug31_kobj->kobj, &ug31xx_ktype, NULL, "ug31xx");
        if(rtn) {
                kobject_put(&ug31_kobj->kobj);
                GAUGE_err("[%s]: kobject_init_and_add fail\n", __func__);
                return;
        }

        kobject_uevent(&ug31_kobj->kobj, KOBJ_ADD);
}

static void destroy_ug31xx_kobj(void)
{
        kobject_put(&ug31_kobj->kobj);
}

#endif  ///< end of UG31XX_KOBJECT

#ifdef	UG31XX_REGISTER_POWERSUPPLY

static enum power_supply_property ug31xx_batt_props[] = {
        POWER_SUPPLY_PROP_STATUS,
        POWER_SUPPLY_PROP_HEALTH,
        POWER_SUPPLY_PROP_PRESENT,
        POWER_SUPPLY_PROP_VOLTAGE_NOW,
        POWER_SUPPLY_PROP_VOLTAGE_AVG,
        POWER_SUPPLY_PROP_CURRENT_NOW,
        POWER_SUPPLY_PROP_CURRENT_AVG,
        POWER_SUPPLY_PROP_CAPACITY,
        POWER_SUPPLY_PROP_TEMP,
        POWER_SUPPLY_PROP_TEMP_AMBIENT,
        POWER_SUPPLY_PROP_CHARGE_NOW,
        POWER_SUPPLY_PROP_CHARGE_FULL,
        POWER_SUPPLY_PROP_CHARGE_COUNTER,
        POWER_SUPPLY_PROP_SERIAL_NUMBER,
};

#endif	///< end of UG31XX_REGISTER_POWERSUPPLY

#ifdef	UPI_CALLBACK_FUNC
static enum power_supply_property ug31xx_pwr_props[] = {
        POWER_SUPPLY_PROP_ONLINE,
};
#endif	///< end of UPI_CALLBACK_FUNC

#ifdef	UPI_CALLBACK_FUNC
static char *supply_list[] = {
        "battery",
        "ac",
        "usb",
};
#endif	///< end of UPI_CALLBACK_FUNC

#ifdef	UG31XX_REGISTER_POWERSUPPLY

static struct power_supply ug31xx_supply[] = {
        {
                .name			= "battery",
                .type			= POWER_SUPPLY_TYPE_BATTERY,
                .properties		= ug31xx_batt_props,
                .num_properties 	= ARRAY_SIZE(ug31xx_batt_props),
                .get_property		= ug31xx_battery_get_property,

#ifndef	UPI_CALLBACK_FUNC
                .external_power_changed	= ug31xx_battery_external_power_changed,
#endif	///< end of UPI_CALLBACK_FUNC

        },
#ifdef	UPI_CALLBACK_FUNC
        {
                .name			= "ac",
                .type			= POWER_SUPPLY_TYPE_MAINS,
                .supplied_to		= supply_list,
                .num_supplicants	= ARRAY_SIZE(supply_list),
                .properties 		= ug31xx_pwr_props,
                .num_properties		= ARRAY_SIZE(ug31xx_pwr_props),
                .get_property		= ug31xx_power_get_property,
        },
        {
                .name			= "usb",
                .type			= POWER_SUPPLY_TYPE_USB,
                .supplied_to		= supply_list,
                .num_supplicants	= ARRAY_SIZE(supply_list),
                .properties 		= ug31xx_pwr_props,
                .num_properties 	= ARRAY_SIZE(ug31xx_pwr_props),
                .get_property 		= ug31xx_power_get_property,
        },
#endif	///< end of UPI_CALLBACK_FUNC
};

static int ug31xx_battery_get_property(struct power_supply *psy,
                                       enum power_supply_property psp,
                                       union power_supply_propval *val)
{
        int ret = 0;

        switch (psp) {
        case POWER_SUPPLY_PROP_HEALTH:
                val->intval = POWER_SUPPLY_HEALTH_GOOD;
                break;
        case POWER_SUPPLY_PROP_PRESENT:
                val->intval = 1;
                break;
        case POWER_SUPPLY_PROP_STATUS:
        case POWER_SUPPLY_PROP_VOLTAGE_NOW:
        case POWER_SUPPLY_PROP_VOLTAGE_AVG:
        case POWER_SUPPLY_PROP_CURRENT_NOW:
        case POWER_SUPPLY_PROP_CURRENT_AVG:
        case POWER_SUPPLY_PROP_CAPACITY:
        case POWER_SUPPLY_PROP_TEMP:
        case POWER_SUPPLY_PROP_TEMP_AMBIENT:
        case POWER_SUPPLY_PROP_CHARGE_NOW:
        case POWER_SUPPLY_PROP_CHARGE_FULL:
        case POWER_SUPPLY_PROP_CHARGE_COUNTER:
        case POWER_SUPPLY_PROP_SERIAL_NUMBER:
                if (ug31xx_update_psp(psp, val))
                        return -EINVAL;
                break;
        default:
                return -EINVAL;
        }
        return ret;
}

#endif	///< end of UG31XX_REGISTER_POWERSUPPLY

#ifdef	UPI_CALLBACK_FUNC

static int ug31xx_power_get_property(struct power_supply *psy,
                                     enum power_supply_property psp,
                                     union power_supply_propval *val)
{
        int ret = 0;

        switch (psp) {
        case POWER_SUPPLY_PROP_ONLINE:
                if((psy->type == POWER_SUPPLY_TYPE_MAINS) && (cur_cable_status == UG31XX_AC_ADAPTER_CABLE)) {
                        val->intval = 1;
                } else if((psy->type == POWER_SUPPLY_TYPE_USB) && (cur_cable_status == UG31XX_USB_PC_CABLE)) {
                        val->intval = 1;
                } else {
                        val->intval = 0;
                }
                break;
        default:
                return -EINVAL;
        }
        return ret;
}

#endif	///< end of UPI_CALLBACK_FUNC

static void check_backup_file_routine(void)
{
        if(ug31xx_drv_status != UG31XX_DRV_INIT_OK) {
                return;
        }

        /// [AT-PM] : Enable check backup file ; 05/14/2013
        if(op_options & LKM_OPTIONS_DISABLE_BACHUP_FILE) {
                ug31_module.set_backup_file(false);
        } else {
                ug31_module.set_backup_file(true);

#ifdef  UG31XX_USER_SPACE_BACKUP

                kobj_event_env = UG31XX_KOBJ_ENV_BACKUP_DATA;
                change_ug31xx_kobj();

#else   ///< else of UG31XX_USER_SPACE_BACKUP

                ug31xx_backup_file_status = ug31_module.chk_backup_file();
                if((ug31xx_backup_file_status > 0) && (!(op_options & LKM_OPTIONS_FORCE_RESET))) {
                        GAUGE_err("[%s] Gauge driver version mismatched.\n", __func__);
                        op_options = op_options | LKM_OPTIONS_FORCE_RESET;
#ifdef  UG31XX_USE_DAEMON_AP_FOR_FILE_OP
                        set_file_op_status_bit(UG31XX_KERNEL_FILE_VERSION);
#endif  ///< end of UG31XX_USE_DAEMON_AP_FOR_FILE_OP
                } else {
#ifdef  UG31XX_USE_DAEMON_AP_FOR_FILE_OP
                        clear_file_op_status_bit(UG31XX_KERNEL_FILE_VERSION);
#endif  ///< end of UG31XX_USE_DAEMON_AP_FOR_FILE_OP
                }

#endif  ///< end of UG31XX_USER_SPACE_BACKUP
        }
}

#ifdef	UG31XX_REGISTER_POWERSUPPLY

static int ug31xx_update_psp(enum power_supply_property psp,
                             union power_supply_propval *val)
{
        if (ug31xx_drv_status == UG31XX_DRV_NOT_READY) {
                GAUGE_err("[%s] Gauge driver not init finish\n", __func__);
                return -EINVAL;
        }

        if(psp == POWER_SUPPLY_PROP_TEMP) {
                val->intval = ug31->batt_temp;
                GAUGE_info("[%s] Temperature=%d\n", __func__, val->intval);
        }
        if(psp == POWER_SUPPLY_PROP_TEMP_AMBIENT) {
                val->intval = ug31->batt_avg_temp;
        }
        if(psp == POWER_SUPPLY_PROP_CAPACITY) {
                if (ug31->batt_status == POWER_SUPPLY_STATUS_FULL) {
                        ug31->batt_capacity = 100;
                }
                val->intval = ug31->batt_capacity;
                if (ug31->batt_ntc_sts != UPI_UG31XX_NTC_NORMAL)
                        val->intval = -99;
        }
        if(psp == POWER_SUPPLY_PROP_VOLTAGE_NOW) {
                mutex_lock(&ug31->info_update_lock);
                val->intval = ug31_module.get_voltage_now();
                mutex_unlock(&ug31->info_update_lock);
        }
        if(psp == POWER_SUPPLY_PROP_VOLTAGE_AVG) {
                val->intval = ug31->batt_volt;
        }
        if(psp == POWER_SUPPLY_PROP_CURRENT_NOW) {
                mutex_lock(&ug31->info_update_lock);
                val->intval = ug31_module.get_current_now();
                mutex_unlock(&ug31->info_update_lock);
                if((is_charging() == false) && (val->intval > 0)) {
                        val->intval = 0;
                }
        }
        if(psp == POWER_SUPPLY_PROP_CURRENT_AVG) {
                val->intval = ug31->batt_current;
        }
        if(psp == POWER_SUPPLY_PROP_STATUS) {
                if(cur_cable_status) {
                        if (ug31->batt_capacity == 100) {
                                ug31->batt_status = POWER_SUPPLY_STATUS_FULL;
                        } else {
                                ug31->batt_status = POWER_SUPPLY_STATUS_CHARGING;
                        }
                } else {
                        ug31->batt_status = POWER_SUPPLY_STATUS_DISCHARGING;
                }
                val->intval = ug31->batt_status;
                if (ug31->batt_ntc_sts != UPI_UG31XX_NTC_NORMAL)
                        val->intval = POWER_SUPPLY_STATUS_UNKNOWN;
        }

        if(psp == POWER_SUPPLY_PROP_CHARGE_NOW) {
                val->intval = ug31->batt_charge_now;
        }
        if (psp == POWER_SUPPLY_PROP_CHARGE_FULL) {
                val->intval = ug31->batt_charge_full;
        }
        if(psp == POWER_SUPPLY_PROP_CHARGE_COUNTER) {
                val->intval = ug31->batt_charge_counter;
        }
        if(psp == POWER_SUPPLY_PROP_SERIAL_NUMBER) {
                if(board_offset_cali_finish == true) {
                        sprintf(ug31->effective_board_offset, "C%d", ug31->board_offset);
                } else {
                        sprintf(ug31->effective_board_offset, "G%d", ug31->board_offset);
                }
                val->strval = ug31->effective_board_offset;
        }

        return 0;
}
#else	///< else of UG31XX_REGISTER_POWERSUPPLY

int ug31xx_asus_battery_dev_read_percentage(void)
{
        int percent = 0;

        //mutex_lock(&ug31->info_update_lock);
        percent = ug31_module.get_relative_state_of_charge();
        //mutex_unlock(&ug31->info_update_lock);
        return percent;
}

int ug31xx_asus_battery_dev_read_volt(void)
{
        int voltage_mV = 0;

        //mutex_lock(&ug31->info_update_lock);
        voltage_mV = ug31_module.get_voltage();
        //mutex_unlock(&ug31->info_update_lock);
        return voltage_mV;
}

int ug31xx_asus_battery_dev_read_current(void)
{
        int current_mA = 0;

        mutex_lock(&ug31->info_update_lock);
        current_mA = ug31_module.get_current_now() + 0x10000;
        mutex_unlock(&ug31->info_update_lock);
        return current_mA;
}

int ug31xx_asus_battery_dev_read_av_energy(void)
{
        int av_energy = 0;
        int fcc = 0;
        int rm = 0;

        //mutex_lock(&ug31->info_update_lock);
        fcc = ug31_module.get_full_charge_capacity();
        rm = ug31_module.get_remaining_capacity();
        //mutex_unlock(&ug31->info_update_lock);
        av_energy = ((fcc & 0xffff) << 16) | (rm & 0xffff);
        return av_energy;
}

int ug31xx_asus_battery_dev_read_temp(void)
{
        int temperature = 0;

        //mutex_lock(&ug31->info_update_lock);
        temperature = ug31_module.get_avg_external_temperature();
        //mutex_unlock(&ug31->info_update_lock);
        return temperature;
}

int ug31xx_asus_battery_dev_read_fcc(void)
{
        int fcc = 0;

        //mutex_lock(&ug31->info_update_lock);
        fcc = ug31_module.get_full_charge_capacity();
        //mutex_unlock(&ug31->info_update_lock);
        return fcc;
}

int ug31xx_asus_battery_dev_read_nac(void)
{
        int nac = 0;

        //mutex_lock(&ug31->info_update_lock);
        nac = ug31_module.get_remaining_capacity();
        //mutex_unlock(&ug31->info_update_lock);
        return nac;
}

int ug31xx_asus_battery_dev_read_rm(void)
{
        int rm = 0;

        //mutex_lock(&ug31->info_update_lock);
        rm = ug31_module.get_remaining_capacity();
        //mutex_unlock(&ug31->info_update_lock);
        return rm;
}

#endif  ///< end of UG31XX_REGISTER_POWERSUPPLY

static int ug31xx_powersupply_init(struct i2c_client *client)
{
#ifdef  UG31XX_REGISTER_POWERSUPPLY
        int i, ret;
        for (i = 0; i < ARRAY_SIZE(ug31xx_supply); i++) {
                ret = power_supply_register(&client->dev, &ug31xx_supply[i]);
                if (ret) {
                        GAUGE_err("[%s] Failed to register power supply\n", __func__);
                        do {
                                power_supply_unregister(&ug31xx_supply[i]);
                        } while ((--i) >= 0);
                        return ret;
                }
        }
#else   ///< else of UG31XX_REGISTER_POWERSUPPLY

        int ret;

        ug31xx_tbl.read_percentage = ug31xx_asus_battery_dev_read_percentage;
        ug31xx_tbl.read_current = ug31xx_asus_battery_dev_read_current;
        ug31xx_tbl.read_volt = ug31xx_asus_battery_dev_read_volt;
        ug31xx_tbl.read_av_energy = ug31xx_asus_battery_dev_read_av_energy;
        ug31xx_tbl.read_temp = ug31xx_asus_battery_dev_read_temp;
        ug31xx_tbl.read_fcc = ug31xx_asus_battery_dev_read_fcc;
        ug31xx_tbl.read_nac = ug31xx_asus_battery_dev_read_nac;
        //ug31xx_tbl.read_rm = ug31xx_asus_battery_dev_read_rm;

        ret = asus_register_power_supply(&client->dev, &ug31xx_tbl);
        if (ret) {
                GAUGE_err("[%s] Failed to register power supply\n", __func__);
        }

#endif  ///< end of UG31XX_REGISTER_POWERSUPPLY
        return 0;
}

#ifdef	UPI_CALLBACK_FUNC

int ug31xx_cable_callback(unsigned char usb_cable_state)
{
        int old_cable_status = cur_cable_status;

        cur_cable_status = usb_cable_state;

        if(ug31xx_drv_status == UG31XX_DRV_NOT_READY) {
                GAUGE_err("[%s] Gauge driver not init finish\n", __func__);
                return (0);
        }

        if(old_cable_status != cur_cable_status) {
                cable_status_changed = CABLE_STATUS_CHANGE_RELEASE_COUNT;
                GAUGE_notice("[%s] Cable status changed (%d -> %d)\n", __func__,
                             old_cable_status,
                             cur_cable_status)
        }

#ifdef  UG31XX_REGISTER_POWERSUPPLY

        if(cur_cable_status == UG31XX_NO_CABLE) {
                if(old_cable_status == UG31XX_AC_ADAPTER_CABLE) {
                        power_supply_changed(&ug31xx_supply[PWR_SUPPLY_AC]);
                } else if(old_cable_status == UG31XX_USB_PC_CABLE) {
                        power_supply_changed(&ug31xx_supply[PWR_SUPPLY_USB]);
                }
        } else if(cur_cable_status == UG31XX_USB_PC_CABLE) {
                power_supply_changed(&ug31xx_supply[PWR_SUPPLY_USB]);
        } else if(cur_cable_status == UG31XX_AC_ADAPTER_CABLE) {
                power_supply_changed(&ug31xx_supply[PWR_SUPPLY_AC]);
        }
#endif  ///< end of UG31XX_REGISTER_POWERSUPPLY
        return (0);
}
EXPORT_SYMBOL(ug31xx_cable_callback);

#else	///< else of UPI_CALLBACK_FUNC

#ifdef	UG31XX_REGISTER_POWERSUPPLY

static void ug31xx_battery_external_power_changed(struct power_supply *psy)
{
        int old_cable_status = cur_cable_status;

        cur_cable_status = (unsigned char)power_supply_am_i_supplied(psy);

        if(ug31xx_drv_status == UG31XX_DRV_NOT_READY) {
                GAUGE_err("[%s] Gauge driver not init finish\n", __func__);
                return;
        }

        if(old_cable_status != cur_cable_status) {
                cable_status_changed = CABLE_STATUS_CHANGE_RELEASE_COUNT;
        }
}

#endif	///< end of UG31XX_REGISTER_POWERSUPPLY

#endif	///< end of UPI_CALLBACK_FUNC

static void set_default_batt_status(void)
{
        ug31->polling_time		= 5;
        ug31->update_time		= 5;
        ug31->batt_volt			= 3700;
        ug31->batt_capacity		= 50;
        ug31->batt_capacity_real	= 50;
        ug31->batt_charge_now	= 1000;
        ug31->batt_charge_full	= 2000;
        ug31->batt_charge_counter = 0;
        ug31->batt_current		= 0;
        ug31->batt_temp			= 250;
        ug31->batt_avg_temp		= 250;
        ug31->batt_ready		= UPI_UG31XX_MODULE_NOT_READY;
        ug31->batt_alarm_sts		= 0;
        ug31->batt_remove		= UPI_UG31XX_BATTERY_REMOVED;
        ug31->batt_cycle_count	= 0;
        ug31->batt_ntc_sts		= UPI_UG31XX_NTC_NORMAL;

        ug31->batt_capacity_last	= ug31->batt_capacity;
        ug31->batt_current_last	= ug31->batt_current;

        ug31->board_offset		= 0;
        cable_status_changed		= 0;
}

static void show_abnormal_batt_status(void)
{
        set_default_batt_status();

        ug31->batt_capacity		= -99;

        GAUGE_notice("ABN STS -> V=%d(mV) C=%d(mA) T=%d-%d(0.1oC) RSOC=%d(%d)(%%) RM=%d(mAh) FCC=%d(mAh) Q=%d(mAh) STS=%d CYCLE=%d\n",
                     ug31->batt_volt,
                     ug31->batt_current,
                     ug31->batt_temp,
                     ug31->batt_avg_temp,
                     ug31->batt_capacity,
                     ug31->batt_capacity_real,
                     ug31->batt_charge_now,
                     ug31->batt_charge_full,
                     ug31->batt_charge_counter,
                     ug31->batt_status,
                     ug31->batt_cycle_count);
}

static void show_default_batt_status(void)
{
        set_default_batt_status();

        GAUGE_notice("DEF STS -> V=%d(mV) C=%d(mA) T=%d-%d(0.1oC) RSOC=%d(%d)(%%) RM=%d(mAh) FCC=%d(mAh) Q=%d(mAh) STS=%d CYCLE=%d\n",
                     ug31->batt_volt,
                     ug31->batt_current,
                     ug31->batt_temp,
                     ug31->batt_avg_temp,
                     ug31->batt_capacity,
                     ug31->batt_capacity_real,
                     ug31->batt_charge_now,
                     ug31->batt_charge_full,
                     ug31->batt_charge_counter,
                     ug31->batt_status,
                     ug31->batt_cycle_count);
}

#define MAX_BATTERY_UPDATE_INTERVAL     (600)

static void show_update_batt_status(void)
{
        ug31->batt_capacity_last	= ug31->batt_capacity;
        ug31->batt_current_last	= ug31->batt_current;

#ifdef  UG31XX_DYNAMIC_POLLING
        ug31->polling_time		= (u32)ug31_module.get_polling_time();
        ug31->update_time		= (u32)ug31_module.get_update_time();
#else   ///< else of UG31XX_DYNAMIC_POLLING
        ug31->polling_time		= 5;
        ug31->update_time		= 5;
#endif  ///< end of UG31XX_DYNAMIC_POLLING
        ug31->batt_volt			= (u32)ug31_module.get_voltage();
        ug31->batt_capacity		= (int)ug31_module.get_relative_state_of_charge();
        ug31->batt_capacity_real	= (int)ug31_module.get_predict_rsoc();
        ug31->batt_charge_now	= (u32)ug31_module.get_remaining_capacity();
        ug31->batt_charge_full	= (u32)ug31_module.get_full_charge_capacity();
        ug31->batt_charge_counter = ug31_module.get_cumulative_capacity();
        ug31->batt_current		= ug31_module.get_current();
#ifdef UG31XX_SHOW_EXT_TEMP
        ug31->batt_temp			= ug31_module.get_external_temperature();
#else   ///< else of UG31XX_SHOW_EXT_TEMP
        ug31->batt_temp			= ug31_module.get_internal_temperature();
#endif  ///< end of UG31XX_SHOW_EXT_TEMP
        ug31->batt_avg_temp		= ug31_module.get_avg_external_temperature();
        ug31->batt_ready		= ug31_module.get_module_ready();
        ug31->batt_alarm_sts		= ug31_module.get_alarm_status();
        ug31->batt_remove		= ug31_module.get_battery_removed();
        ug31->batt_cycle_count	= ug31_module.get_cycle_count();
        ug31->batt_ntc_sts		= ug31_module.get_ntc_status();
        ug31->board_offset		= ug31_module.get_board_offset();

        if(cable_status_changed > 0) {
                ug31->update_time = 3;
                cable_status_changed = cable_status_changed - 1;
                force_power_supply_change = true;
        }

        if(ug31->batt_capacity != ug31->batt_capacity_last) {
                force_update_backup_file = true;
        }

        if(ug31->batt_capacity != 100) {
                enable_board_offset_cali_at_eoc = true;
        }

        GAUGE_notice("BAT STS -> V=%d(mV) C=%d(mA) T=%d-%d(0.1oC) RSOC=%d(%d)(%%) RM=%d(mAh) FCC=%d(mAh) Q=%d(mAh) STS=%d CYCLE=%d P=%d-%d(sec) BO=%d %s-%s (%d)\n",
                     ug31->batt_volt,
                     ug31->batt_current,
                     ug31->batt_temp,
                     ug31->batt_avg_temp,
                     ug31->batt_capacity,
                     ug31->batt_capacity_real,
                     ug31->batt_charge_now,
                     ug31->batt_charge_full,
                     ug31->batt_charge_counter,
                     ug31->batt_status,
                     ug31->batt_cycle_count,
                     ug31->polling_time,
                     ug31->update_time,
                     ug31->board_offset,
                     ug31_module.get_version(),
                     ug31->daemon_ver,
                     ug31->daemon_uevent_count);
}

static void adjust_cell_table(void)
{
        int rtn;

        if(op_options & LKM_OPTIONS_ADJUST_DESIGN_CAPACITY) {
                if(design_capacity >= 1) {
                        rtn = ug31_module.adjust_cell_table(design_capacity);
                        GAUGE_info("[%s] design capacity has been adjusted. (%d)\n", __func__, rtn);
                }
        }
        op_options = op_options & (~LKM_OPTIONS_ADJUST_DESIGN_CAPACITY);
}

#ifdef UG31XX_WAKEUP_ALARM

static void ug31xx_gauge_alarm(struct alarm *alarm)
{
        GAUGE_info("[%s] wake up alarm\n", __func__);
}

static void ug31xx_program_alarm(int seconds)
{
        ktime_t low_interval = ktime_set(seconds, 0);
        ktime_t slack = ktime_set(20, 0);
        ktime_t next;

        next = ktime_add(ug31->last_poll, low_interval);
        alarm_start_range(&ug31->wakeup_alarm, next, ktime_add(next, slack));
}

#endif ///< end of UG31XX_WAKEUP_ALARM

#define	UG31_BATTERY_MAX_UPDATE_INTERVAL	(100)

static void batt_power_update_work_func(struct work_struct *work)
{
        struct ug31xx_gauge *ug31_dev;

        ug31_dev = container_of(work, struct ug31xx_gauge, batt_power_update_work.work);

#ifdef  UG31XX_REGISTER_POWERSUPPLY

        if(wake_lock_active(&ug31_dev->batt_wake_lock) != 0) {
                wake_unlock(&ug31_dev->batt_wake_lock);
        }
        wake_lock_timeout(&ug31->batt_wake_lock, UG31XX_WAKE_LOCK_TIMEOUT*HZ);
        power_supply_changed(&ug31xx_supply[PWR_SUPPLY_BATTERY]);

#endif  ///< end of UG31XX_REGISTER_POWERSUPPLY

        schedule_delayed_work(&ug31_dev->batt_power_update_work, ug31_dev->polling_time*HZ);
        GAUGE_info("[%s] issue power_supply_changed.\n", __func__);
}

/// =================================================================
/// Charger control for initial capacity prediction (Start)
/// =================================================================

static void start_charging(void)
{
        smb347_charging_toggle(true);
}

static void stop_charging(void)
{
        smb347_charging_toggle(false);
}

static ssize_t batt_switch_name(struct switch_dev *sdev, char *buf)
{
        const char* FAIL = "0xFFFF";

        if (ug31xx_drv_status == UG31XX_DRV_NOT_READY) {
                GAUGE_err("[%s] Gauge driver not init finish\n", __func__);
                return sprintf(buf, "%s\n", FAIL);
        }

        return sprintf(buf, "%s-%s\n", ug31_module.get_version(), ug31->daemon_ver);
}

/// =================================================================
/// Charger control for initial capacity prediction (Stop)
/// =================================================================

#define UG31_BATTERY_WORK_MAX_INTERVAL      (30)
#define UG31XX_PROBE_CHARGER_OFF_DELAY      (1)

static void batt_info_update_work_func(struct work_struct *work)
{
        int gg_status;
        struct ug31xx_gauge *ug31_dev;
        int curr_direction_changed;
        int prev_fc_sts;
        int now_fc_sts;

        ug31_dev = container_of(work, struct ug31xx_gauge, batt_info_update_work.work);

        gg_status = 0;

        if(wake_lock_active(&ug31_dev->batt_wake_lock) != 0) {
                wake_unlock(&ug31_dev->batt_wake_lock);
        }
        wake_lock_timeout(&ug31_dev->batt_wake_lock, UG31XX_WAKE_LOCK_TIMEOUT*HZ);

        curr_charger_full_status = is_charging_full();

        if ((charger_full_status == false) && (curr_charger_full_status == true)) {
                charger_detect_full = true;
                force_power_supply_change = true;
        }
        charger_full_status = curr_charger_full_status;

        /// [AT-PM] : Update gauge information ; 02/04/2013

        mutex_lock(&ug31_dev->info_update_lock);
        ug31_module.set_options(op_options);
        if(op_options & LKM_OPTIONS_FORCE_RESET) {
                ug31_module.reset(FactoryGGBXFile);
        }
        op_options = op_options & (~LKM_OPTIONS_FORCE_RESET);

        adjust_cell_table();
        update_project_config();

        if(is_charging() == true) {
                ug31_module.set_cable_out(UG31XX_CABLE_IN);
        } else {
                ug31_module.set_cable_out(UG31XX_CABLE_OUT);
        }

        prev_fc_sts = ug31_module.get_full_charge_status();
        gg_status = ug31_module.update((user_space_algorithm_response == true) ? UG31XX_USER_SPACE_RESPONSE : UG31XX_USER_SPACE_NO_RESPONSE);
        now_fc_sts = ug31_module.get_full_charge_status();
        if(gg_status == 0) {
                if(charger_detect_full == true) {
                        charger_detect_full = false;
                        gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                }

#ifndef UG31XX_WAIT_CHARGER_FC

                if((prev_fc_sts == 0) && (now_fc_sts == 1)) {
                        gg_status = ug31_module.set_charger_full(UG31XX_TAPER_REACHED);
                }

                if((is_charging() == true) && (now_fc_sts == -1)) {
                        gg_status = ug31_module.set_charger_full(UG31XX_TAPER_REACHED);
                }

                if(curr_charger_full_status == true) {
                        gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                }

#endif  ///< end of UG31XX_WAIT_CHARGER_FC

                if(in_early_suspend == true) {
                        delta_q_in_suspend = delta_q_in_suspend + ug31_module.get_delta_q();
                }
        }

        if(gg_status == 0) {
                show_update_batt_status();
        } else {
                show_abnormal_batt_status();
                GAUGE_err("[%s] Fail to update gauge info.\n", __func__);
        }
        mutex_unlock(&ug31_dev->info_update_lock);

        curr_direction_changed = ug31_dev->batt_current_last*ug31_dev->batt_current;
#ifdef  UG31XX_USER_SPACE_ALGORITHM

        if(curr_direction_changed < 0) {
                force_power_supply_change = true;
        }

        if(user_space_algorithm_response == false) {
                if((ug31_dev->batt_capacity != ug31_dev->batt_capacity_last) ||
                    (force_power_supply_change == true)) {
                        force_power_supply_change = false;

                        cancel_delayed_work_sync(&ug31_dev->batt_power_update_work);
                        schedule_delayed_work(&ug31_dev->batt_power_update_work, 0*HZ);
                }
        }

#else   ///< else of UG31XX_USER_SPACE_ALGORITHM

        if((ug31_dev->batt_capacity != ug31_dev->batt_capacity_last) ||
            (curr_direction_changed < 0) ||
            (force_power_supply_change == true)) {
                force_power_supply_change = false;

                cancel_delayed_work_sync(&ug31_dev->batt_power_update_work);
                schedule_delayed_work(&ug31_dev->batt_power_update_work, 0*HZ);
        }

#endif  ///< end of UG31XX_USER_SPACE_ALGORITHM

#ifdef  UG31XX_USER_SPACE_ALGORITHM
        user_space_algorithm_response = false;
#endif  ///< end of UG31XX_USER_SPACE_ALGORITHM
        kobj_event_env = UG31XX_KOBJ_ENV_UPDATE_CAPACITY;
        change_ug31xx_kobj();

        mutex_lock(&ug31_dev->info_update_lock);
        check_backup_file_routine();
#ifndef	UG31XX_USER_SPACE_ALGORITHM
        if(in_early_suspend == false) {
                ug31_module.set_capacity_suspend_mode(_UPI_FALSE_);
        }
#endif	///< end of UG31XX_USER_SPACE_ALGORITHM
        mutex_unlock(&ug31_dev->info_update_lock);

        if(op_options & LKM_OPTIONS_FORCE_RESET) {
                batt_info_update_work_func(&ug31_dev->batt_info_update_work.work);
        } else {
                schedule_delayed_work(&ug31_dev->batt_info_update_work, ug31_dev->update_time*HZ);
        }

        if((is_charging_full() == true) && (enable_board_offset_cali_at_eoc == true)) {
                enable_board_offset_cali_at_eoc = false;
                if((ug31_dev->batt_avg_temp >= UG31XX_CALI_BO_LOW_TEMP) && (ug31_dev->batt_avg_temp <= UG31XX_CALI_BO_HIGH_TEMP)) {
                        schedule_delayed_work(&ug31->board_offset_cali_work, UG31XX_PROBE_CHARGER_OFF_DELAY*HZ);
                        GAUGE_info("[%s] Wait %d seconds to calibrate board offset\n", __func__, UG31XX_PROBE_CHARGER_OFF_DELAY);
                        stop_charging();
                } else {
                        GAUGE_err("[%s] No board offset calibration because of temperature (%d) (%d-%d)\n", __func__, ug31_dev->batt_avg_temp, UG31XX_CALI_BO_LOW_TEMP, UG31XX_CALI_BO_HIGH_TEMP);
                }
        }
}

#ifdef  UG31XX_WAIT_CHARGER_FC

int ug31xx_charger_detect_full(bool is_charger_full)
{
        charger_detect_full = is_charger_full;

        if(ug31xx_drv_status != UG31XX_DRV_NOT_READY) {
                cancel_delayed_work_sync(&ug31->batt_info_update_work);
                batt_info_update_work_func(&ug31_dev->batt_info_update_work.work);
        }

        if(charger_detect_full == true) {
                GAUGE_info("[%s] charger_detect_full = true\n", __func__);
        } else {
                GAUGE_info("[%s] charger_detect_full = false\n", __func__);
        }
        return (0);
}
EXPORT_SYMBOL(ug31xx_charger_detect_full);

#endif  ///< end of UG31XX_WAIT_CHARGER_FC

#ifdef  UG31XX_DYNAMIC_TAPER_CURRENT

int ug31xx_set_charge_termination_current(int curr)
{
        charge_termination_current = curr;
        GAUGE_info("[%s] charge_termination_current = %d\n", __func__, charge_termination_current);
        return (0);
}
EXPORT_SYMBOL(ug31xx_set_charge_termination_current);

#endif  ///< end of UG31XX_DYNAMIC_TAPER_CURRENT

#ifdef UG31XX_PROC_DEV
struct proc_dir_entry *battery;
int ug31xx_get_proc_rsoc(char *buf, char **start, off_t off, int count,
                         int *eof, void *data )
{
        int len = 0;
        len += sprintf(buf+len, "%d\n", ug31_module.get_predict_rsoc());
        return len;
}

int ug31xx_get_proc_psoc(char *buf, char **start, off_t off, int count,
                         int *eof, void *data )
{
        int len = 0;
        len += sprintf(buf+len, "%d\n", ug31_module.get_relative_state_of_charge());
        return len;
}

int ug31xx_get_proc_rm(char *buf, char **start, off_t off, int count,
                       int *eof, void *data )
{
        int len = 0;
        len += sprintf(buf+len, "%d\n", ug31_module.get_remaining_capacity());
        return len;
}

int ug31xx_get_proc_fcc(char *buf, char **start, off_t off, int count,
                        int *eof, void *data )
{
        int len = 0;
        len += sprintf(buf+len, "%d\n", ug31_module.get_full_charge_capacity());
        return len;
}

int ug31xx_get_proc_curr(char *buf, char **start, off_t off, int count,
                         int *eof, void *data )
{
        int len = 0;
        len += sprintf(buf+len, "%d\n", ug31_module.get_current_now());
        return len;
}

int ug31xx_get_proc_temp(char *buf, char **start, off_t off, int count, int *eof, void *data)
{
        int len = 0;
        int value;

        value = ug31_module.get_avg_external_temperature();
        len += sprintf(buf+len, "%d.%d\n", value/10, value%10);
        return len;
}

#endif  ///< end of UG31XX_PROC_DEV

#if defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND)

static void ug31xx_early_suspend(struct early_suspend *e)
{
        int gg_status;
        struct ug31xx_gauge *ug31_dev = container_of(e, struct ug31xx_gauge, es);

        dev_info(&ug31_dev->client->dev, "enter %s\n", __func__);
        GAUGE_info("[%s]\n", __func__);

        charger_dc_in_before_suspend = is_charging();

        mutex_lock(&ug31_dev->info_update_lock);
        gg_status = ug31_module.suspend((charger_dc_in_before_suspend == true) ? 1 : 0);
        if(gg_status == 0) {
                show_update_batt_status();
        } else {
                show_abnormal_batt_status();
                GAUGE_err("[%s] Fail in early suspend.\n", __func__);
        }
        rsoc_before_suspend = ug31_module.get_predict_rsoc();
        ug31_module.set_capacity_suspend_mode(_UPI_TRUE_);
        mutex_unlock(&ug31_dev->info_update_lock);

        in_early_suspend = true;
        delta_q_in_suspend = 0;
        GAUGE_info("[%s] Driver early suspend.\n", __func__);
}

static void ug31xx_late_resume(struct early_suspend *e)
{
        struct ug31xx_gauge *ug31_dev = container_of(e, struct ug31xx_gauge, es);
        int gg_status;
        int prev_fc_sts;
        int now_fc_sts;
        bool charger_dc_in_after_resume;
        int prev_rsoc;

        dev_info(&ug31_dev->client->dev, "enter %s\n", __func__);
        GAUGE_info("[%s]\n", __func__);

        cancel_delayed_work_sync(&ug31_dev->batt_info_update_work);

        charger_dc_in_after_resume = is_charging();

        mutex_lock(&ug31_dev->info_update_lock);
        prev_fc_sts = ug31_module.get_full_charge_status();
        prev_rsoc = rsoc_before_suspend;
        gg_status = ug31_module.resume((user_space_algorithm_response == true) ? UG31XX_USER_SPACE_RESPONSE : UG31XX_USER_SPACE_NO_RESPONSE);
        delta_q_in_suspend = delta_q_in_suspend + ug31_module.get_delta_q();
        now_fc_sts = ug31_module.get_full_charge_status();
        if(gg_status == 0) {
                if(charger_detect_full == true) {
                        charger_detect_full = false;
                        gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                }

#ifndef UG31XX_WAIT_CHARGER_FC

                if(charger_dc_in_before_suspend == false) {
                        GAUGE_info("[%s] Enter early suspend without charger\n", __func__);
                } else {
                        GAUGE_info("[%s] Enter early suspend with charger\n", __func__);
                        if((prev_fc_sts == 0) && (now_fc_sts == 1)) {
                                gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                GAUGE_info("[%s] [FC] status changed (0->1)\n", __func__);
                        } else {
                                if(charger_dc_in_after_resume == false) {
                                        GAUGE_info("[%s] Exit early suspend without charger\n", __func__);
                                        prev_rsoc = (100 - prev_rsoc)*(ug31_module.get_design_capacity())/100;
                                        if(delta_q_in_suspend > prev_rsoc) {
                                                GAUGE_info("[%s] Charged capacity is enouth to check full charged (%d) > (%d)\n", __func__, delta_q_in_suspend, prev_rsoc);

                                                if(ug31_module.get_predict_rsoc() > 90) {
                                                        gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                                        GAUGE_info("[%s] Set to full charge status\n", __func__);
                                                }
                                        }
                                } else {
                                        GAUGE_info("[%s] Exit early suspend with charger\n", __func__);
                                        if(curr_charger_full_status == true) {
                                                GAUGE_info("[%s] Charger detects full\n", __func__);
                                                gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                                GAUGE_info("[%s] Set to full charge status\n", __func__);
                                        } else {
                                                prev_rsoc = (100 - prev_rsoc)*(ug31_module.get_design_capacity())/100;
                                                if(delta_q_in_suspend > prev_rsoc) {
                                                        GAUGE_info("[%s] Charged capacity is enouth to check full charged (%d) > (%d)\n", __func__, delta_q_in_suspend, prev_rsoc);
                                                        if(ug31_module.get_predict_rsoc() > 95) {
                                                                gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                                                GAUGE_info("[%s] Set to full charge status\n", __func__);
                                                        }
                                                }
                                        }
                                }
                        }
                }

#endif  ///< end of UG31XX_WAIT_CHARGER_FC

                if(gg_status == 0) {
                        show_update_batt_status();
                } else {
                        show_abnormal_batt_status();
                        GAUGE_err("[%s] Fail to set charger full.\n", __func__);
                }
        } else {
                show_abnormal_batt_status();
                GAUGE_err("[%s] Fail to late resume.\n", __func__);
        }
        mutex_unlock(&ug31_dev->info_update_lock);

        batt_info_update_work_func(&ug31_dev->batt_info_update_work.work);

        force_power_supply_change = true;
        in_early_suspend = false;
        GAUGE_info("[%s] Gauge late resumed.\n", __func__);
}

static void ug31xx_config_earlysuspend(struct ug31xx_gauge *chip)
{
        chip->es.level = EARLY_SUSPEND_LEVEL_BLANK_SCREEN - 2;
        chip->es.suspend = ug31xx_early_suspend;
        chip->es.resume = ug31xx_late_resume;
        register_early_suspend(&chip->es);
}

#else	///< else of defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND) 

#define ug31xx_early_suspend NULL
#define ug31xx_late_resume NULL

static void ug31xx_config_earlysuspend(struct ug31xx_gauge *chip)
{
        return;
}

#endif	///< end of defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND) 

/**
 * @brief board_offset_cali_work_func
 *
 *  Work for calibrate board offset
 *
 * @para  work  address of struct work_struct
 * @return  NULL
 */
static void board_offset_cali_work_func(struct work_struct *work)
{
        mutex_lock(&ug31->info_update_lock);
        ug31_module.calibrate_offset(UG31XX_BOARD_OFFSET_CALI_AVG);
        mutex_unlock(&ug31->info_update_lock);

        start_charging();
        GAUGE_info("[%s] Board offset calibration done\n", __func__);
}

static void shell_timeout_work_func(struct work_struct *work)
{
        if(user_space_in_progress == true) {
                UG31_LOGI("[%s]: Daemon timeout\n", __func__);
                user_space_in_progress = false;
                mutex_unlock(&ug31->info_update_lock);
                if(wake_lock_active(&ug31->shell_timeout_wake_lock) != 0) {
                        wake_unlock(&ug31->shell_timeout_wake_lock);
                }
        }
}

static void shell_backup_work_func(struct work_struct *work)
{
        ug31xx_backup_file_status = ug31_module.shell_backup();

        if((ug31xx_backup_file_status > 0) && (!(op_options & LKM_OPTIONS_FORCE_RESET))) {
                UG31_LOGE("[%s] Gauge driver version mismatched.\n", __func__);
                op_options = op_options | LKM_OPTIONS_FORCE_RESET;
                cancel_delayed_work_sync(&ug31->batt_info_update_work);
                schedule_delayed_work(&ug31->batt_info_update_work, 0*HZ);
                return;
        }

        if(ug31xx_backup_file_status < 0) {
                show_update_batt_status();
                cancel_delayed_work_sync(&ug31->batt_info_update_work);
                schedule_delayed_work(&ug31->batt_info_update_work, ug31->update_time*HZ);
        }
}

static void shell_algorithm_work_func(struct work_struct *work)
{
        mutex_lock(&ug31->info_update_lock);

        ug31_module.shell_update();

        user_space_algorithm_now_fc_sts = ug31_module.get_full_charge_status();
#ifndef UG31XX_WAIT_CHARGER_FC

        if((user_space_algorithm_prev_fc_sts == 0) && (user_space_algorithm_now_fc_sts == 1)) {
                ug31_module.set_charger_full(UG31XX_TAPER_REACHED);
        }

        if((is_charging() == true) && (user_space_algorithm_now_fc_sts == -1)) {
                ug31_module.set_charger_full(UG31XX_TAPER_REACHED);
        }

        if(curr_charger_full_status == true) {
                ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
        }

#endif  ///< end of UG31XX_WAIT_CHARGER_FC

        show_update_batt_status();

        if((ug31->batt_capacity != ug31->batt_capacity_last) ||
            (force_power_supply_change == true)) {
                force_power_supply_change = false;

                cancel_delayed_work_sync(&ug31->batt_power_update_work);
                schedule_delayed_work(&ug31->batt_power_update_work, 0*HZ);
        }

        if(in_early_suspend == false) {
                ug31_module.set_capacity_suspend_mode(_UPI_FALSE_);
        }

        mutex_unlock(&ug31->info_update_lock);

        user_space_algorithm_response = true;
}

/**
 * @brief batt_probe_work_func
 *
 *  Register uG31xx driver
 *
 * @para  work  address of struct work_struct
 * @return  NULL
 */
static void batt_probe_work_func(struct work_struct *work)
{
        int rtn;
#ifdef UG31XX_PROC_DEV
        struct proc_dir_entry *ent;
#endif  ///< end of UG31XX_PROC_DEV

        /*read hw id first*/
        if(HW_ID_EVB == Read_HW_ID())
                hw_id_flag = 0;
        else
                hw_id_flag = 1;

        printk("%s, HW_ID=0x%x,  hw_id_flag=%d\n", __func__, Read_HW_ID(), hw_id_flag);

        if(hw_id_flag==0)
                op_options = op_options | LKM_OPTIONS_ENABLE_REVERSE_CURRENT;

        ug31_module.set_options(op_options);
        rtn = ug31_module.initial(FactoryGGBXFile, ((probe_with_cable == true) ? UG31XX_CABLE_IN : UG31XX_CABLE_OUT));
        op_options = op_options & (~LKM_OPTIONS_FORCE_RESET);
        if(rtn != 0) {
                goto pwr_supply_fail;
        }

        op_options = op_options | LKM_OPTIONS_ADJUST_DESIGN_CAPACITY;
        adjust_cell_table();

        ug31_module.set_shell_ap(UPI_UG31XX_SHELL_AP, strlen(UPI_UG31XX_SHELL_AP));
        ug31_module.set_backup_file_name(UPI_UG31XX_BACKUP_FILE, strlen(UPI_UG31XX_BACKUP_FILE));
        ug31_module.set_suspend_file_name(UPI_UG31XX_BACKUP_SUSPEND, strlen(UPI_UG31XX_BACKUP_SUSPEND));

        charge_termination_current = ug31_module.get_charge_termination_current();
        rsense_value = ug31_module.get_rsense();
        ggb_board_offset = ug31_module.get_ggb_board_offset();
        ntc_offset = ug31_module.get_ntc_offset();
        standby_current = ug31_module.get_standby_current();
        ggb_board_gain = ug31_module.get_ggb_board_gain();
        set_project_config();
        update_project_config();
        GAUGE_info("[%s] ug31_module initialized. (%s)\n", __func__, ug31_module.get_version());

#ifdef UG31XX_SHOW_EXT_TEMP
        ug31_module.set_battery_temp_external();
#else  ///< else of UG31XX_SHOW_EXT_TEMP
        ug31_module.set_battery_temp_internal();
#endif  ///< end of UG31XX_SHOW_EXT_TEMP
        GAUGE_info("[%s] set battery temperature from external\n", __func__);

        INIT_DELAYED_WORK(&ug31->batt_info_update_work, batt_info_update_work_func);
        INIT_DELAYED_WORK(&ug31->batt_power_update_work, batt_power_update_work_func);
        INIT_DELAYED_WORK(&ug31->board_offset_cali_work, board_offset_cali_work_func);
        INIT_DELAYED_WORK(&ug31->shell_algorithm_work, shell_algorithm_work_func);
        INIT_DELAYED_WORK(&ug31->shell_backup_work, shell_backup_work_func);
        INIT_DELAYED_WORK(&ug31->shell_timeout_work, shell_timeout_work_func);

        if(ug31xx_powersupply_init(ug31->client)) {
                goto pwr_supply_fail;
        }
        ug31xx_drv_status = UG31XX_DRV_INIT_OK;

#ifdef	UPI_CALLBACK_FUNC

        if (cur_cable_status) {
                ug31xx_cable_callback(cur_cable_status);
        }

#endif	///< end of UPI_CALLBACK_FUNC

#ifdef UG31XX_MISC_DEV

        rtn = misc_register(&ug31xx_misc);
        if(rtn < 0) {
                GAUGE_err("[%s] Unable to register misc deive\n", __func__);
                misc_deregister(&ug31xx_misc);
        }

#endif	///< end of UG31XX_MISC_DEV

#ifdef UG31XX_PROC_DEV
        ent = create_proc_read_entry("BMSSOC", 0744, NULL, ug31xx_get_proc_rsoc, NULL );
        if(!ent) {
                GAUGE_err("create /proc/BMSSOC fail\n");
        }
        ent = create_proc_read_entry("RSOC", 0744, NULL, ug31xx_get_proc_psoc, NULL );
        if(!ent) {
                GAUGE_err("create /proc/RSOC fail\n");
        }
        ent = create_proc_read_entry("RM", 0744, NULL, ug31xx_get_proc_rm, NULL );
        if(!ent) {
                GAUGE_err("create /proc/RM fail\n");
        }
        ent = create_proc_read_entry("FCC", 0744, NULL, ug31xx_get_proc_fcc, NULL );
        if(!ent) {
                GAUGE_err("create /proc/FCC fail\n");
        }
        ent = create_proc_read_entry("bat_current", 0744, NULL, ug31xx_get_proc_curr, NULL );
        if(!ent) {
                GAUGE_err("create /proc/bat_current fail\n");
        }
        ent = create_proc_read_entry("driver/BatTemp", 0744, NULL, ug31xx_get_proc_temp, NULL);
        if(!ent) {
                GAUGE_err("create /proc/driver/BatTemp fail\n");
        }
#endif	///< end of UG31XX_PROC_DEV

        /* register switch device for battery information versions report */
        batt_dev.name = "battery";
        batt_dev.print_name = batt_switch_name;
        if (switch_dev_register(&batt_dev) < 0) {
                GAUGE_err("%s: fail to register battery switch\n", __func__);
                goto pwr_supply_fail;
        }
        ug31xx_config_earlysuspend(ug31);

        if(probe_with_cable == true) {
                ug31_module.calibrate_offset(UG31XX_BOARD_OFFSET_CALI_AVG);
        }
        start_charging();

        schedule_delayed_work(&ug31->batt_info_update_work, 0*HZ);

        force_power_supply_change = true;
        GAUGE_info("[%s] Driver %s registered done\n", __func__, ug31->client->name);
        return;

pwr_supply_fail:
        start_charging();
        kfree(ug31);
}

static int __devinit ug31xx_i2c_probe(struct i2c_client *client,
                                      const struct i2c_device_id *id)
{
        struct i2c_adapter *adapter = to_i2c_adapter(client->dev.parent);

        GAUGE_info("++++++++++++++++ %s ++++++++++++++++\n", __func__);
        if (!i2c_check_functionality(adapter, I2C_FUNC_SMBUS_BYTE)) {
                return -EIO;
        }

        ug31 = kzalloc(sizeof(*ug31), GFP_KERNEL);
        if (!ug31) {
                return -ENOMEM;
        }

        show_default_batt_status();

        ug31->client = client;
        ug31->dev = &client->dev;
        sprintf(ug31->daemon_ver, "UPI");
        ug31->daemon_uevent_count = 0;

        i2c_set_clientdata(client, ug31);
        API_I2C_Init(ug31->client);

        mutex_init(&ug31->info_update_lock);
        wake_lock_init(&ug31->batt_wake_lock, WAKE_LOCK_SUSPEND, "ug31xx_driver_update");
        wake_lock_init(&ug31->shell_algorithm_wake_lock, WAKE_LOCK_SUSPEND, "ug31xx_shell_algorithm");
        wake_lock_init(&ug31->shell_timeout_wake_lock, WAKE_LOCK_SUSPEND, "ug31xx_shell_timeout");
#ifdef UG31XX_WAKEUP_ALARM
        ug31->last_poll = alarm_get_elapsed_realtime();
        alarm_init(&ug31->wakeup_alarm, ANDROID_ALARM_ELAPSED_REALTIME_WAKEUP,
                   ug31xx_gauge_alarm);
#endif ///< end of UG31XX_WAKEUP_ALARM
        INIT_DELAYED_WORK(&ug31->batt_probe_work, batt_probe_work_func);

        get_ggb_array();

#ifdef  UG31XX_PROBE_CHARGER_OFF

        probe_with_cable = is_charging();

        if(probe_with_cable == true) {
                schedule_delayed_work(&ug31->batt_probe_work, UG31XX_PROBE_CHARGER_OFF_DELAY*HZ);
                GAUGE_info("[%s] Wait %d seconds to register driver %s\n", __func__, UG31XX_PROBE_CHARGER_OFF_DELAY, ug31->client->name);
                stop_charging();
        } else {
                schedule_delayed_work(&ug31->batt_probe_work, 0*HZ);
                GAUGE_info("[%s] No wait because not in charging\n", __func__);
        }

#else   ///< else of UG31XX_PROBE_CHARGER_OFF

        schedule_delayed_work(&ug31->batt_probe_work, 0*HZ);

#endif  ///< end of UG31XX_PROBE_CHARGER_OFF

        return 0;
}

static int __devexit ug31xx_i2c_remove(struct i2c_client *client)
{
        struct ug31xx_gauge *ug31_dev;

#ifdef  UG31XX_REGISTER_POWERSUPPLY

        int i;

#endif  ///< end of UG31XX_REGISTER_POWERSUPPLY

        mutex_lock(&ug31->info_update_lock);
        wake_lock_destroy(&ug31->batt_wake_lock);
        wake_lock_destroy(&ug31->shell_algorithm_wake_lock);
        wake_lock_destroy(&ug31->shell_timeout_wake_lock);

#ifdef  UG31XX_REGISTER_POWERSUPPLY

        for (i = 0; i < ARRAY_SIZE(ug31xx_supply); i++) {
                power_supply_unregister(&ug31xx_supply[i]);
        }

#endif  ///< end of UG31XX_REGISTER_POWERSUPPLY

#ifdef UG31XX_MISC_DEV

        misc_deregister(&ug31xx_misc);

#endif	///< end of UG31XX_MISC_DEV

#ifdef UG31XX_PROC_DEV

        remove_proc_entry( "RSOC",NULL );
        remove_proc_entry( "PSOC",NULL );
        remove_proc_entry( "RM",NULL );
        remove_proc_entry( "FCC",NULL );
        remove_proc_entry( "bat_current",NULL );

#endif	///< end of UG31XX_PROC_DEV

        cancel_delayed_work_sync(&ug31->batt_info_update_work);
        cancel_delayed_work_sync(&ug31->batt_power_update_work);

        ug31_module.uninitial();
        GAUGE_info("[%s] Driver removed.\n", __func__);

        ug31_dev = i2c_get_clientdata(client);
        if(ug31_dev) {
                kfree(ug31_dev);
        }

        mutex_unlock(&ug31->info_update_lock);
        mutex_destroy(&ug31->info_update_lock);

        if(ug31) {
                kfree(ug31);
        }
        return 0;
}

static int ug31xx_i2c_suspend(struct i2c_client *client, pm_message_t mesg)
{
        int gg_status;

        cancel_delayed_work_sync(&ug31->batt_info_update_work);

#if !(defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND))

        mutex_lock(&ug31->info_update_lock);
        ug31_module.set_backup_file(false);
        mutex_unlock(&ug31->info_update_lock);

        charger_dc_in_before_suspend = is_charging();

        mutex_lock(&ug31->info_update_lock);
        gg_status = ug31_module.suspend((charger_dc_in_before_suspend == true) ? 1 : 0);
        if(gg_status == 0) {
                show_update_batt_status();
                GAUGE_info("[%s] Driver suspend.\n", __func__);
        } else {
                show_abnormal_batt_status();
                GAUGE_err("[%s] Fail in suspend.\n", __func__);
        }
        rsoc_before_suspend = ug31_module.get_predict_rsoc();
        mutex_unlock(&ug31->info_update_lock);

#endif	///< end of defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND) 

        mutex_lock(&ug31->info_update_lock);
        gg_status = ug31_module.set_capacity_suspend_mode(_UPI_TRUE_);
        /// Setting wakeup alarm
#ifdef UG31XX_WAKEUP_ALARM
        ug31->last_poll = alarm_get_elapsed_realtime();
        ug31xx_program_alarm(UG31XX_WAKEUP_ALARM_TIME);
#endif ///< end of UG31XX_WAKEUP_ALARM
        mutex_unlock(&ug31->info_update_lock);

        ug31xx_drv_status = UG31XX_DRV_SUSPEND;
        GAUGE_info("[%s] ug31xx_i2c_suspend() end\n", __func__);
        return 0;
}

static int ug31xx_i2c_resume(struct i2c_client *client)
{
#if !(defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND))

        int gg_status;
        int prev_fc_sts;
        int now_fc_sts;
        bool charger_dc_in_after_resume;
        int prev_rsoc;

        charger_dc_in_after_resume = is_charging();

        mutex_lock(&ug31->info_update_lock);
        prev_fc_sts = ug31_module.get_full_charge_status();
        prev_rsoc = rsoc_before_suspend;
        gg_status = ug31_module.resume((user_space_algorithm_response == true) ? UG31XX_USER_SPACE_RESPONSE : UG31XX_USER_SPACE_NO_RESPONSE);
        delta_q_in_suspend = delta_q_in_suspend + ug31_module.get_delta_q();
        now_fc_sts = ug31_module.get_full_charge_status();
        if(gg_status == 0) {
                if(charger_detect_full == true) {
                        charger_detect_full = false;
                        gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                }

#ifndef UG31XX_WAIT_CHARGER_FC

                if(charger_dc_in_before_suspend == false) {
                        GAUGE_info("[%s] Enter suspend without charger\n", __func__);
                } else {
                        GAUGE_info("[%s] Enter suspend with charger\n", __func__);
                        if((prev_fc_sts == 0) && (now_fc_sts == 1)) {
                                gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                curr_charger_full_status = is_charging_full();
                                GAUGE_info("[%s] [FC] status changed (0->1)\n", __func__);
                        } else {
                                if(charger_dc_in_after_resume == false) {
                                        GAUGE_info("[%s] Exit suspend without charger\n", __func__);
                                        prev_rsoc = (100 - prev_rsoc)*(ug31_module.get_design_capacity())/100;
                                        if(delta_q_in_suspend > prev_rsoc) {
                                                GAUGE_info("[%s] Charged capacity is enouth to check full charged (%d) > (%d)\n", __func__, delta_q_in_suspend, prev_rsoc);

                                                if(ug31_module.get_predict_rsoc() > 90) {
                                                        gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                                        GAUGE_info("[%s] Set to full charge status\n", __func__);
                                                }
                                        }
                                } else {
                                        GAUGE_info("[%s] Exit suspend with charger\n", __func__);
                                        curr_charger_full_status = is_charging_full();
                                        if(curr_charger_full_status == true) {
                                                GAUGE_info("[%s] Charger detects full\n", __func__);
                                                gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                                GAUGE_info("[%s] Set to full charge status\n", __func__);
                                        } else {
                                                prev_rsoc = (100 - prev_rsoc)*(ug31_module.get_design_capacity())/100;
                                                if(delta_q_in_suspend > prev_rsoc) {
                                                        GAUGE_info("[%s] Charged capacity is enouth to check full charged (%d) > (%d)\n", __func__, delta_q_in_suspend, prev_rsoc);
                                                        if(ug31_module.get_predict_rsoc() > 95) {
                                                                gg_status = ug31_module.set_charger_full(UG31XX_CHARGER_DETECTS_FULL_STEP);
                                                                GAUGE_info("[%s] Set to full charge status\n", __func__);
                                                        }
                                                }
                                        }
                                }
                        }
                }

#endif  ///< end of UG31XX_WAIT_CHARGER_FC

                if(gg_status == 0) {
                        show_update_batt_status();
                        GAUGE_info("[%s] Gauge resumed.\n", __func__);
                } else {
                        show_abnormal_batt_status();
                        GAUGE_err("[%s] Fail to set charger full.\n", __func__);
                }
        } else {
                show_abnormal_batt_status();
                GAUGE_err("[%s] Fail to resume.\n", __func__);
        }
        mutex_unlock(&ug31->info_update_lock);

        schedule_delayed_work(&ug31->batt_info_update_work, 0*HZ);

        force_power_supply_change = true;

#else   ///< else of defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND) 

        int time_interval;

        curr_charger_full_status = is_charging_full();
        if(curr_charger_full_status == true) {
                charger_detect_full = true;
        }

        mutex_lock(&ug31->info_update_lock);
        time_interval = ug31_module.get_update_interval();
        mutex_unlock(&ug31->info_update_lock);
        if((cable_status_changed > 0) ||
            (is_charging() != charger_dc_in_before_suspend)) {
                cable_status_changed = 0;
                schedule_delayed_work(&ug31->batt_info_update_work, ug31->update_time*HZ);
        } else {
                if(time_interval > MAX_BATTERY_UPDATE_INTERVAL) {
                        batt_info_update_work_func(&ug31->batt_info_update_work.work);
                        GAUGE_info("[%s]: time interval = %d > %d -> re-schedule batt_info_update_work.\n", __func__, time_interval, MAX_BATTERY_UPDATE_INTERVAL);
                } else {
                        schedule_delayed_work(&ug31->batt_info_update_work, ug31->update_time*HZ);
                }
        }

#ifndef UG31XX_USER_SPACE_ALGORITHM
        mutex_lock(&ug31->info_update_lock);
        ug31_module.set_capacity_suspend_mode(_UPI_FALSE_);
        mutex_unlock(&ug31->info_update_lock);
#endif  ///< end of UG31XX_USER_SPACE_ALGORITHM

#endif	///< end of defined(CONFIG_HAS_EARLYSUSPEND) && defined(UG31XX_EARLY_SUSPEND) 

        ug31xx_drv_status = UG31XX_DRV_INIT_OK;

        GAUGE_info("[%s] ug31xx_i2c_resume() end\n", __func__);
        return 0;
}

static void ug31xx_i2c_shutdown(struct i2c_client *client)
{
        int gg_status = 0;

        cancel_delayed_work_sync(&ug31->batt_info_update_work);
        cancel_delayed_work_sync(&ug31->batt_power_update_work);
        cancel_delayed_work_sync(&ug31->shell_algorithm_work);
        cancel_delayed_work_sync(&ug31->shell_backup_work);
        cancel_delayed_work_sync(&ug31->shell_timeout_work);

        mutex_lock(&ug31->info_update_lock);
        gg_status = ug31_module.shutdown();
        if(gg_status == 0) {
                show_update_batt_status();
                GAUGE_info("[%s] Gauge driver shutdown.\n", __func__);
        } else {
                show_abnormal_batt_status();
                GAUGE_err("[%s] Fail to shutdown gauge.\n", __func__);
        }
        mutex_unlock(&ug31->info_update_lock);
        GAUGE_info("[%s] Driver shutdown. gg_status=0x%02x\n", __func__, gg_status);
}

static const struct i2c_device_id ug31xx_i2c_id[] = {
        { UG31XX_DEV_NAME, 0 },
        { },
};

MODULE_DEVICE_TABLE(i2c, ug31xx_i2c_id);

static struct i2c_driver ug31xx_i2c_driver = {
        .driver    = {
                .name  = UG31XX_DEV_NAME,
                .owner = THIS_MODULE,
        },
        .probe     = ug31xx_i2c_probe,
        .remove    = __devexit_p(ug31xx_i2c_remove),
        .suspend   = ug31xx_i2c_suspend,
        .resume    = ug31xx_i2c_resume,
        .shutdown  = ug31xx_i2c_shutdown,
        .id_table  = ug31xx_i2c_id,
};

#ifdef	UG31XX_REGISTER_I2C

static struct i2c_board_info ug31xx_i2c_board_info = {
        .type          = "ug31xx-gauge",
        .flags         = 0x00,
        .addr          = 0x70,
        .platform_data = NULL,
        .archdata      = NULL,
        .irq           = -1,
};


#define	UG31XX_I2C_ADAPTER	(2)

static struct i2c_client *i2c_client;
static struct i2c_adapter *i2c_adap;

#endif	///< end of UG31XX_REGISTER_I2C

static int __init ug31xx_i2c_init(void)
{
        int ret = 0;
        u32 test_major_flag=0;
        struct asus_bat_config bat_cfg;

        //turn to jiffeys/s
        bat_cfg.polling_time = 0;
        bat_cfg.critical_polling_time = 0;
        bat_cfg.polling_time *= HZ;
        bat_cfg.critical_polling_time *= HZ;

#ifdef	UG31XX_REGISTER_I2C
        i2c_adap = i2c_get_adapter(UG31XX_I2C_ADAPTER);
        if (!i2c_adap) {
                GAUGE_err("[%s] Cannot get i2c adapter %d\n", __func__, UG31XX_I2C_ADAPTER);
                ret = -ENODEV;
                goto err1;
        }


        i2c_client = i2c_new_device(i2c_adap, &ug31xx_i2c_board_info);
        if (!i2c_client) {
                GAUGE_err("[%s] Unable to add I2C device for 0x%x\n", __func__,
                          ug31xx_i2c_board_info.addr);
                ret = -ENODEV;
                goto err2;
        }
#endif	///< end of UG31XX_REGISTER_I2C

        //init battery info & work queue
        ret = asus_battery_init(bat_cfg.polling_time, bat_cfg.critical_polling_time, test_major_flag);
        if (ret)
                goto err4;

        ret =  i2c_add_driver(&ug31xx_i2c_driver);
        if (ret) {
#ifdef	UG31XX_REGISTER_I2C
                goto err3;
#else	///< else of UG31XX_REGISTER_I2C
                goto err1;
#endif	///< end of UG31XX_REGISTER_I2C
        }

        create_ug31xx_kobj();

        return 0;

#ifdef	UG31XX_REGISTER_I2C
err3:
        i2c_unregister_device(i2c_client);
err2:
        i2c_put_adapter(i2c_adap);
#endif	///< end of UG31XX_REGISTER_I2C
err1:
        asus_battery_exit();
err4:
        return ret;
}
late_initcall(ug31xx_i2c_init);

static void __exit ug31xx_i2c_exit(void)
{
#ifdef	UG31XX_REGISTER_I2C
        i2c_put_adapter(i2c_adap);
#endif	///< end of UG31XX_REGISTER_I2C
        i2c_del_driver(&ug31xx_i2c_driver);
#ifdef	UG31XX_REGISTER_I2C
        i2c_unregister_device(i2c_client);
#endif	///< end of UG31XX_REGISTER_I2C

        destroy_ug31xx_kobj();
}
module_exit(ug31xx_i2c_exit);

MODULE_DESCRIPTION("ug31xx gauge driver");
MODULE_LICENSE("GPL");

module_param(op_options, byte, 0644);
MODULE_PARM_DESC(op_options, "Set operation options for uG31xx driver.");
module_param(design_capacity, ushort, 0644);
MODULE_PARM_DESC(design_capacity, "Set new design capacity for uG31xx driver.");
module_param(kbo_result, int, 0644);
MODULE_PARM_DESC(kbo_result, "Set board offset for uG31xx driver.");
module_param(charge_termination_current, int, 0644);
MODULE_PARM_DESC(charge_termination_current, "Set charge termination current for uG31xx driver.");
module_param(rsense_value, int, 0644);
MODULE_PARM_DESC(rsense_value, "Set RSense resistance for uG31xx driver");
module_param(ggb_board_offset, int, 0644);
MODULE_PARM_DESC(ggb_board_offset, "Set board offset in GGB file");
module_param(ntc_offset, int, 0644);
MODULE_PARM_DESC(ntc_offset, "Set NTC offset");
module_param(standby_current, int, 0644);
MODULE_PARM_DESC(standby_current, "Set current for suspend mode.");
module_param(ggb_board_gain, int, 0644);
MODULE_PARM_DESC(ggb_board_gain, "Set board gain in GGB file");
