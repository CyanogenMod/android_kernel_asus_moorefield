/*
 * Copyright (c) 2012, ASUSTek, Inc. All Rights Reserved.
 * Written by chris chang chris1_chang@asus.com
 */
#ifndef __BQ27520_BATTERY_H_
#define __BQ27520_BATTERY_H_

struct bq27520_platform_data {
	bool enable_current_sense;
	bool is_init_done;
	bool is_volt_shutdown;
	bool is_capacity_shutdown;
	bool is_lowbatt_shutdown;
	int technology;

	int (*current_sense_enabled)(void);
	int (*battery_present)(void);
	int (*battery_health)(void);
	int (*battery_status)(void);
	int (*battery_pack_temp)(int *);
	int (*save_config_data)(const char *name, void *data, int len);
	int (*restore_config_data)(const char *name, void *data, int len);
	void (*reset_i2c_lines)(void);

	bool (*is_cap_shutdown_enabled)(void);
	bool (*is_volt_shutdown_enabled)(void);
	bool (*is_lowbatt_shutdown_enabled)(void);
	int (*get_vmin_threshold)(void);

	/* ID for different battery cell (LG and Coslight) */
	int bat_id_gpio;

	/* Battery low pin */
	int low_bat;

	/* Adc alert pin */
	int adc_alert;
};


#endif /* __BQ27520_BATTERY_H_ */
