/*
 * Copyright (c) 2013, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#ifndef __BQ27520_BATTERY_H__
#define __BQ27520_BATTERY_H__

/* IC version: G4 */
#define CURRENT_IC_VERSION			4
#define IC_VERSION_G4				4
#define IC_VERSION_G3				3

/* Bq27520 standard data commands */
#if CURRENT_IC_VERSION == IC_VERSION_G3
    #define BQ27520_REG_CNTL		0x00
    #define BQ27520_REG_AR			0x02
    #define BQ27520_REG_ARTTE		0x04
    #define BQ27520_REG_TEMP		0x06
    #define BQ27520_REG_VOLT		0x08
    #define BQ27520_REG_FLAGS		0x0A
    #define BQ27520_REG_NAC			0x0C
    #define BQ27520_REG_FAC			0x0e
    #define BQ27520_REG_RM			0x10
    #define BQ27520_REG_FCC			0x12
    #define BQ27520_REG_AI			0x14
    #define BQ27520_REG_TTE			0x16
    #define BQ27520_REG_TTF			0x18
    #define BQ27520_REG_SI			0x1a
    #define BQ27520_REG_STTE		0x1c
    #define BQ27520_REG_MLI			0x1e
    #define BQ27520_REG_MLTTE		0x20
    #define BQ27520_REG_AE			0x22
    #define BQ27520_REG_AP			0x24
    #define BQ27520_REG_TTECP		0x26
    #define BQ27520_REG_SOH			0x28
    #define BQ27520_REG_CC          0x2a
    #define BQ27520_REG_SOC			0x2c
    #define BQ27520_REG_NIC			0x2e
    #define BQ27520_REG_ICR			0x30
    #define BQ27520_REG_LOGIDX		0x32
    #define BQ27520_REG_LOGBUF		0x34
    #define BQ27520_FLAG_DSC		BIT(0)
    #define BQ27520_FLAG_FC			BIT(9)
    #define BQ27520_FLAG_BAT_DET	BIT(3)
    #define BQ27520_CS_DLOGEN		BIT(15)
    #define BQ27520_CS_SS		    BIT(13)

#elif CURRENT_IC_VERSION == IC_VERSION_G4

    #define BQ27520_REG_CNTL		0x00
    #define BQ27520_REG_AR			0x02
    #define BQ27520_REG_ARTTE		0x04
    #define BQ27520_REG_TEMP		0x06
    #define BQ27520_REG_VOLT		0x08
    #define BQ27520_REG_FLAGS		0x0A
    #define BQ27520_REG_NAC			0x0C
    #define BQ27520_REG_FAC			0x0e
    #define BQ27520_REG_RM			0x10
    #define BQ27520_REG_FCC			0x12
    #define BQ27520_REG_AI			0x14
    #define BQ27520_REG_TTE			0x16
    #define BQ27520_REG_SI			0x18
    #define BQ27520_REG_STTE		0x1a
    #define BQ27520_REG_SOH			0x1c
    #define BQ27520_REG_CC			0x1e
    #define BQ27520_REG_SOC			0x20

    #define BQ27520_REG_IC			0x22
    #define BQ27520_REG_IT			0x28
    #define BQ27520_REG_RS			0x2a
    #define BQ27520_REG_OC			0x2c
    #define BQ27520_REG_DC			0x2e

    #define BQ27520_FLAG_DSC		BIT(0)
    #define BQ27520_FLAG_FC			BIT(9)
    #define BQ27520_FLAG_BAT_DET	BIT(3)
    #define BQ27520_CS_DLOGEN		BIT(15)
    #define BQ27520_CS_SS		    BIT(13)
#else
    /* may compile error */
#endif

/* Control subcommands */
#if CURRENT_IC_VERSION == IC_VERSION_G3
    #define BQ27520_SUBCMD_CTNL_STATUS		0x0000
    #define BQ27520_SUBCMD_DEVICE_TYPE		0x0001
    #define BQ27520_SUBCMD_FW_VER			0x0002
    #define BQ27520_SUBCMD_PREV_MACW		0x0007
    #define BQ27520_SUBCMD_CHEM_ID			0x0008
    #define BQ27520_SUBCMD_OCV				0x000c
    #define BQ27520_SUBCMD_BAT_INS			0x000d
    #define BQ27520_SUBCMD_BAT_REM			0x000e
    #define BQ27520_SUBCMD_SET_HIB			0x0011
    #define BQ27520_SUBCMD_CLR_HIB			0x0012
    #define BQ27520_SUBCMD_SET_SLP			0x0013
    #define BQ27520_SUBCMD_CLR_SLP			0x0014
    #define BQ27520_SUBCMD_FCT_RES			0x0015
    #define BQ27520_SUBCMD_ENABLE_DLOG		0x0018
    #define BQ27520_SUBCMD_DISABLE_DLOG		0x0019
    #define BQ27520_SUBCMD_DF_VERSION		0x001f
    #define BQ27520_SUBCMD_SEALED			0x0020
    #define BQ27520_SUBCMD_ENABLE_IT		0x0021
    #define BQ27520_SUBCMD_DISABLE_IT		0x0023
    #define BQ27520_SUBCMD_RESET			0x0041

#elif CURRENT_IC_VERSION == IC_VERSION_G4

    #define BQ27520_SUBCMD_CTNL_STATUS		0x0000
    #define BQ27520_SUBCMD_DEVICE_TYPE		0x0001
    #define BQ27520_SUBCMD_FW_VER			0x0002
    #define BQ27520_SUBCMD_PREV_MACW		0x0007
    #define BQ27520_SUBCMD_CHEM_ID			0x0008
    #define BQ27520_SUBCMD_OCV				0x000c
    #define BQ27520_SUBCMD_BAT_INS			0x000d
    #define BQ27520_SUBCMD_BAT_REM			0x000e
    #define BQ27520_SUBCMD_SET_HIB			0x0011
    #define BQ27520_SUBCMD_CLR_HIB			0x0012
    #define BQ27520_SUBCMD_SET_SLP			0x0013
    #define BQ27520_SUBCMD_CLR_SLP			0x0014
    #define BQ27520_SUBCMD_DF_VERSION		0x001f
    #define BQ27520_SUBCMD_SEALED			0x0020
    #define BQ27520_SUBCMD_ENABLE_IT		0x0021
    #define BQ27520_SUBCMD_RESET			0x0041

#else
    /* may compile error */
#endif

#define BQ27520_I2C_BUS                 2
#define BQ27520_DEV_NAME                "bq27520"
#define BQ27520_I2C_DEFAULT_ADDR        0x55
struct bq27520_bat_platform_data {
	u32 i2c_bus_id;
};

#define BQ27520_DEV_UPT_NAME            "bq27520f"
#define BQ27520_I2C_DEFAULT_UPT_ADDR    0x0B

/* acquired gpio for battery id */
#define BATTERY_CELL_ID_GPIO_NAME       "bat_id"
#define BATTERY_ID_GPIO_NAME       "BAT_ID"

int bq27520_batt_current_sel_type(void);
int bq27520_write_i2c(struct i2c_client *client,
	u8 reg, int value, int b_single);

int bq27520_read_i2c(struct i2c_client *client,
	u8 reg, int *rt_value, int b_single);

int bq27520_send_subcmd(struct i2c_client *client, int *rt_value, u16 sub_cmd);
int bq27520_cmp_i2c(int reg_off, int value);

int TIgauge_i2c_read(struct i2c_client *client, u8 addr, int len, void *data);
int TIgauge_i2c_write(struct i2c_client *client, u8 addr, int len, void *data);
ssize_t explain_dffs_string(char *OriginalMessages, unsigned long len);
void TIgauge_LockStep(void);

int bq27520_asus_battery_dev_read_percentage(void);
int bq27520_asus_battery_dev_read_current(void);
int bq27520_asus_battery_dev_read_volt(void);
int bq27520_asus_battery_dev_read_temp(void);
int bq27520_asus_battery_dev_read_remaining_capacity(void);
int bq27520_asus_battery_dev_read_full_charge_capacity(void);
int bq27520_asus_battery_dev_nominal_available_capacity(void);
int bq27520_asus_battery_dev_read_chemical_id(void);
int bq27520_asus_battery_dev_read_fw_version(void);
int bq27520_asus_battery_dev_read_fw_cfg_version(void);
int bq27520_asus_battery_dev_read_cycle_count(void);
int bq27520_is_normal_mode(void);
int bq27520_is_rom_mode(void);
int bq27520_batt_fw_sel_type(void);





#define PROC_TRUE -256
#define PROC_FALSE -257

#define WAKEUP0_EVENT_ID          BIT0

#ifndef CONFIG_ME302C
#define CONFIG_ADC_ALERT_GPIO_AS_WAKEUP
#define CONFIG_BATT_LOW_GPIO_AS_WAKEUPx
#else
#define CONFIG_ADC_ALERT_GPIO_AS_WAKEUPx
#define CONFIG_BATT_LOW_GPIO_AS_WAKEUP
#endif

#define ERROR_CODE_I2C_FAILURE    -99999

#define WAKE_UP_LEVEL_LOW       0
#define WAKE_UP_LEVEL_HIGH      1
#define WAKE_UP_FALLING_EDGE    2
#define WAKE_UP_RISING_EDGE     3
#define WAKE_UP_BOTH_EDGE       4
struct battery_low_config {
	u32 wake_up_event_en_bit;
	u32 wake_up_event_type;
};

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

#endif
