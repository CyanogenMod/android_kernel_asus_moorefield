/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by Chris Chang chris1_chang@asus.com
 */
#ifndef __ASUS_BATTERY_H__
#define __ASUS_BATTERY_H__

#include <linux/i2c.h>

typedef enum {
        NO_CABLE=0,     
        USB_ADAPTER,    /* usb plugg-in adapter */
        USB_PC,         /* usb plugg-in PC */
        NUM_CABLE_STATES,
} cable_type_t;

typedef enum {
        CHARGER_BATTERY = 0,
        CHARGER_AC,
        CHARGER_USB
} charger_type_t;

typedef enum {
        DRV_NOT_READY = 0,
        DRV_INIT,
        DRV_INIT_OK,
        DRV_REGISTER,
        DRV_REGISTER_OK,
        DRV_BATTERY_LOW,
        DRV_SHUTDOWN,
} drv_status_t;

typedef enum {
        DEV_NOT_READY = 0,
        DEV_INIT,
        DEV_INIT_OK,
} dev_status_t;

struct battery_dev_info {
        dev_status_t status;
        u32 test_flag;
        struct i2c_client *i2c;
};

struct asus_bat_config {
        int battery_support;
        int polling_time;
        int critical_polling_time;
        int slave_addr;
};

struct asus_batgpio_set{ 
     int  active;  // pull up/down, or < 0 not support
     int  bitmap;   
     int  ctraddr;   
     int  icaddr;   
     int  idaddr;   
     int  ipcaddr;  
     int  ipdaddr;  
     int  int_active; //support interrupt or not. if < 0 mean not support.
     int  int_bitmap;
     int  int_sts_addr;
     int  int_ctrl_addr;
     int  int_type_addr;
     int  int_type_val_clr;
     int  int_type_val_set;
};  

//i2c device relative function
/*
 * Return an unsigned integer value of the predicted remaining battery capacity 
 * expressd as a percentage of FullChargeCapacity(), with a range of 0 to 100%.
 * Or < 0 if something fails.
 */
//int asus_battery_dev_read_percentage(void);
/*
 * Return the battery Voltage in milivolts
 * Or < 0 if something fails.
 */
//int asus_battery_dev_read_volt(void);
/*
 * Return the battery average current
 * Note that current can be negative signed as well(-65536 ~ 65535).
 * So, the value get from device need add a base(0x10000) to be a positive number if no error. 
 * Or < 0 if something fails.
 */
//int asus_battery_dev_read_current(void);
/*
 * Return an unsigned integer value of the predicted charge or energy remaining in 
 * the battery. The value is reported in units of mWh.
 * Or < 0 if something fails.
 */
//int asus_battery_dev_read_av_energy(void);
//
/*
 * Return the battery temperature in tenths of degree Celsius
 * Or < 0 if something fails.
 */
//int asus_battery_dev_read_temp(void);

//Collect functions as structure
struct dev_func {
        /* state of charge */
        int (*read_percentage)(void);
        /* voltage */
        int (*read_volt)(void);
        /* average current */
        int (*read_current)(void);
        /* average energy */
        int (*read_av_energy)(void);
        /* temperature */
        int (*read_temp)(void);
        /* fcc */
        int (*read_fcc)(void);
        /* nac */
        int (*read_nac)(void);
        /* rm */
        int (*read_rm)(void);
		/* battery soh*/
        int (*read_soh)(void);
};


//power relative function, called some times by other drivers,
//but could only service one device.
int asus_battery_init(
        u32 polling_time, 
        u32 critical_polling_time, 
        u32 test_flag
);
void asus_battery_exit(void);
int asus_register_power_supply(struct device *dev, struct dev_func *tbl);
int asus_battery_low_event(void);
void asus_update_all(void);
void asus_queue_update_all(void);
void usb_to_battery_callback(u32);

struct battery_info_reply {
        u32 status;				/* Battery status */
        u32 health;				/* Battery health */
        u32 present;			/* Battery present */
        u32 technology;			/* Battery technology */
        u32 batt_volt;			/* Battery voltage */
        int batt_current;		/* Battery current */
        u32 batt_energy;		/* Battery energy */
        u32 percentage;			/* Battery percentage, -1 is unknown status */
        u32 level;				/* Battery level */
        int batt_temp;			/* Battery Temperature */
        const char* manufacturer;	/* Battery Manufacturer */
        const char* model;			/* Battery Gauge IC model */
        const char* serial_number;	/* Battery Gauge IC firmware version */
//        u32 charging_source;	/* 0: no cable, 1:usb, 2:AC */
//        u32 charging_enabled;	/* 0: Disable, 1: Enable */
//        u32 full_bat;			/* Full capacity of battery (mAh) */
        u32 polling_time;
        u32 critical_polling_time;
        u32 cable_status;
        u32 test_flag;
#ifdef CONFIG_ASUS_FACTORY_MODE
        bool eng_charging_limit;
#endif
	bool thermal_charging_limit;

                /*
                 * BIT0: seperate with other devices, 
                 *       other drivers should not register power supply class
                 * BIT1: not update forever
                 */
        drv_status_t        drv_status;				/* Battery driver status */
        struct dev_func *tbl;
        u32 cable_source;/* 1:USB_IN 2:AC_IN 3:CABLE_OUT 4:ENABLE_5V 5:DISABLE_5V*/
        void *gpadc_handle;
        u32 gpadc_resistance_val;/* battery Resistance */
        u8  gauge_version;/* G3:3, G4:4*/
        bool isValidBattID;/* valid batt ID*/
        u32 batt_fcc;			/* Battery FCC */
        u32 batt_nac;			/* Battery NAC*/
        u32 batt_rm;			/* Battery RM*/
};

#define ERROR_CODE_I2C_FAILURE    -99999
#define BIT0  0x00000001
#define BIT1  0x00000002
#define BIT2  0x00000004
#define BIT3  0x00000008
#define BIT4  0x00000010
#define BIT5  0x00000020
#define BIT6  0x00000040
#define BIT7  0x00000080

#define BIT8  0x00000100
#define BIT9  0x00000200
#define BIT10 0x00000400
#define BIT11 0x00000800
#define BIT12 0x00001000
#define BIT13 0x00002000
#define BIT14 0x00004000
#define BIT15 0x00008000

#define TEST_INFO_NO_REG_POWER  BIT0 
#define TEST_INFO_PROC_DUMP     BIT2

#define BATTERY_DEFAULT_POLL_TIME   (20 * HZ) //20 seconds
#define BATTERY_CRITICAL_POLL_TIME  (5 * HZ)  //5 seconds

/* ADC Channel Numbers */
#define CLT_BATT_NUM_GPADC_SENSORS	1
#define CLT_GPADC_BPRESIST_CHNUM	0x2
#define CLT_GPADC_BPTHERM_SAMPLE_COUNT	1
/* vaild BattID range  1. 102k ohm (297~352) 2. 76.5k ohm (239~290)*/
#define MAX_102K_RESIST_REF             352
#define MIN_102K_RESIST_REF             297

#define MAX_76_5K_RESIST_REF            290
#define MIN_76_5K_RESIST_REF            239
//#define BATTERY_PROC_NAME  "driver/virtual_battery"


#define BATTERY_TAG "[BATT] "
#define BATTERY_ERRORTAG "[BATT_ERR] "
#define BAT_DBG(...)  printk(KERN_INFO BATTERY_TAG __VA_ARGS__);
#define BAT_DBG_L(level, ...)  printk(level BATTERY_TAG __VA_ARGS__);
#define BAT_DBG_E(...)  printk(KERN_ERR BATTERY_ERRORTAG __VA_ARGS__);

#define BATTERY_I2C_ANY_ADDR    0xFF

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


#endif //#define __ASUS_BATTERY_H__
