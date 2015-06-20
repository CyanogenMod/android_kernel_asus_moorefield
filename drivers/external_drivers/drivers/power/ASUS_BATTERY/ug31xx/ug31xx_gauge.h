/*
 * Copyright (c) 2012, uPI Semiconductor Corp. All Rights Reserved.
 */

#ifndef __UPI_ug31xx_GAUGE_H
#define __UPI_ug31xx_GAUGE_H

#define UG31XX_DEV_NAME        "ug31xx-gauge"

#ifndef CONFIG_UPI_BATTERY

typedef enum {
	DRV_NOT_READY = 0,
	DRV_INIT_OK,
	DRV_SUSPEND,
} drv_status_t;

#endif  /*/< end of uG31xx_ASUS_ME172V */

typedef enum {
	UG31XX_GPIO_1 = 0,
	UG31XX_GPIO_2,
	UG31XX_GPIO_3,
	UG31XX_GPIO_4,
} ug31xx_gpio_idx_t;

typedef enum {
	UG31XX_GPIO_STS_LOW = 0,
	UG31XX_GPIO_STS_HIGH,
	UG31XX_GPIO_STS_UNKNOWN,
} ug31xx_gpio_status_t;

#define UPI_UG31XX_SHELL_AP         ("/system/bin/upi_gg_ctl")
#define	UPI_UG31XX_BACKUP_FILE		  ("/sdcard/upi_gg")
#define UPI_UG31XX_BACKUP_SUSPEND   ("/sdcard/upi_table")
#define	UPI_UG31XX_MODULE_READY		  (1)
#define	UPI_UG31XX_MODULE_NOT_READY	(0)
#define	UPI_UG31XX_BATTERY_REMOVED	(1)
#define	UPI_UG31XX_BATTERY_INSERTED	(0)
#define	UPI_UG31XX_ALARM_STATUS_UV	(1<<0)
#define	UPI_UG31XX_ALARM_STATUS_UET	(1<<1)
#define	UPI_UG31XX_ALARM_STATUS_OET	(1<<2)
#define UPI_UG31XX_NTC_NORMAL       (0)
#define UPI_UG31XX_NTC_OPEN         (1)
#define UPI_UG31XX_NTC_SHORT        (2)

#ifndef _LKM_OPTIONS_

#define _LKM_OPTIONS_
#define LKM_OPTIONS_FORCE_RESET             (1<<0)
#define LKM_OPTIONS_ENABLE_SUSPEND_DATA_LOG (1<<1)
#define LKM_OPTIONS_ENABLE_DEBUG_LOG        (3<<2)
#define LKM_OPTIONS_DEBUG_ERROR           (0<<2)
#define LKM_OPTIONS_DEBUG_INFO            (1<<2)
#define LKM_OPTIONS_DEBUG_NOTICE          (2<<2)
#define LKM_OPTIONS_DEBUG_DEBUG           (3<<2)
#define LKM_OPTIONS_ENABLE_REVERSE_CURRENT  (1<<4)
#define LKM_OPTIONS_ADJUST_DESIGN_CAPACITY  (1<<5)
#define LKM_OPTIONS_DISABLE_BACHUP_FILE     (1<<6)

#endif  /*< end of _LKM_OPTIONS_*/

struct ug31xx_module_interface {
	int (*initial)(char *ggb, unsigned char cable);
	int (*uninitial)(void);
	int (*suspend)(char dc_in);
	int (*resume)(char user_space_response);
	int (*shutdown)(void);
	int (*update)(char user_space_response);
	int (*reset)(char *ggb);

	int (*shell_update)(void);
	unsigned char * (*shell_memory)(int *mem_size);
	int (*shell_backup)(void);
	unsigned char * (*shell_backup_memory)(int *mem_size);
	unsigned char * (*shell_table_memory)(int *mem_size);
	unsigned char * (*shell_table_buf_memory)(int *mem_size);

	int (*get_voltage)(void);
	int (*get_voltage_now)(void);
	int (*get_current)(void);
	int (*get_current_now)(void);
	int (*get_external_temperature)(void);
	int (*get_external_temperature_now)(void);
	int (*get_internal_temperature)(void);
	int (*get_internal_temperature_now)(void);
	int (*get_remaining_capacity)(void);
	int (*get_full_charge_capacity)(void);
	int (*get_relative_state_of_charge)(void);
	char * (*get_version)(void);
	int (*get_polling_time)(void);
	int (*get_module_ready)(void);
	int (*get_battery_removed)(void);
	int (*get_alarm_status)(void);
	int (*get_charge_termination_current)(void);
	int (*get_full_charge_status)(void);
	int (*get_design_capacity)(void);
	int (*get_rsense)(void);
	int (*get_predict_rsoc)(void);
	int (*get_gpio)(ug31xx_gpio_idx_t gpio);
	int (*get_cycle_count)(void);
	int (*get_avg_external_temperature)(void);
	int (*get_ntc_status)(void);
	unsigned char * (*get_backup_buffer)(int *size);
	unsigned char (*get_backup_daemon_cntl)(void);
	unsigned char (*get_backup_daemon_period)(void);
	int (*get_update_interval)(void);
	int (*get_update_time)(void);
	int (*get_board_offset)(void);
	int (*get_delta_q)(void);
	int (*get_ggb_board_offset)(void);
	int (*get_ntc_offset)(void);
	int (*get_cumulative_capacity)(void);
	int (*get_standby_current)(void);
	int (*get_ggb_board_gain)(void);

	int (*set_backup_file)(char enable);
	int (*set_charger_full)(char is_full);
	int (*set_charge_termination_current)(int curr);
	int (*set_battery_temp_external)(void);
	int (*set_battery_temp_internal)(void);
	int (*set_rsense)(int rsense);
	int (*set_backup_file_name)(char *filename, int length);
	int (*set_suspend_file_name)(char *filename, int length);
	int (*set_options)(unsigned char options);
	int (*set_gpio)(ug31xx_gpio_idx_t gpio, int status);
	int (*set_shell_ap)(char *apname, int length);
	int (*set_backup_daemon_cntl)(unsigned char cntl);
	int (*set_capacity_suspend_mode)(char in_suspend);
	int (*set_cable_out)(unsigned char cntl);
	int (*set_ggb_board_offset)(int offset);
	int (*set_board_offset)(int offset, char from_upi_bo);
	int (*set_ntc_offset)(int offset);
	int (*set_standby_current)(int curr);
	int (*set_ggb_board_gain)(int gain);

	int (*chk_backup_file)(void);
	int (*enable_save_data)(char enable);
	int (*change_to_pri_batt)(char *ggb, char pri_batt);
	int (*ug31xx_i2c_read)(unsigned short addr, unsigned char *data);
	int (*ug31xx_i2c_write)(unsigned short addr, unsigned char *data);
	int (*reset_cycle_count)(void);
	int (*adjust_cell_table)(unsigned short design_capacity);
	int (*calibrate_offset)(unsigned char options);
	int (*backup_pointer)(void);
	int (*restore_pointer)(void);
};

enum {
	PWR_SUPPLY_BATTERY = 0,
	PWR_SUPPLY_AC,
	PWR_SUPPLY_USB
};

#ifndef CONFIG_UPI_BATTERY
enum {
	NO_CABLE = 0,
	USB_PC_CABLE = 1,
	AC_ADAPTER_CABLE = 3
};
#endif  /*< end of uG31xx_ASUS_ME172V*/

enum {
	UG31XX_CHARGER_NO_DETECTS_FULL = 0,
	UG31XX_CHARGER_DETECTS_FULL,
	UG31XX_CHARGER_DETECTS_FULL_STEP,
	UG31XX_CHARGER_DETECTS_FULL_CHECK,
	UG31XX_TAPER_REACHED,
	UG31XX_BOARD_OFFSET_CALI_STEP,
	UG31XX_BOARD_OFFSET_CALI_FULL,
	UG31XX_BOARD_OFFSET_CALI_FULL_NO_UPPER,
	UG31XX_BOARD_OFFSET_CALI_AVG,
	UG31XX_BOARD_OFFSET_FROM_UPI_BO,
	UG31XX_BOARD_OFFSET_NOT_FROM_UPI_BO,
	UG31XX_CABLE_OUT,
	UG31XX_CABLE_IN,
	UG31XX_USER_SPACE_RESPONSE,
	UG31XX_USER_SPACE_NO_RESPONSE,
};

extern struct ug31xx_module_interface ug31_module;

#endif /*__UPI_ug31xx_GAUGE_H */
