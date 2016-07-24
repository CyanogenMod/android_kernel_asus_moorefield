/*
 * intel_pwm_world_led.h: Intel interface for gps devices
 *
 * (C) Copyright 2015 Intel Corporation
 * Author:
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; version 2
 * of the License.
 */

#ifndef __INTEL_WORLD_LED_H
#define __INTEL_WORLD_LED_H

#define INTEL_WORLD_LED_DRV_NAME "intel_world_led_driver"

#define INTEL_WORLD_LED_MAX_TIMEDIVISOR  0xFF
#define INTEL_WORLD_LED_MAX_BASEUNIT 0x80

#define WORLD_LED_PWM_GPIO 12

struct world_led_pdata {
	u8 time_divisor;
	u8 base_unit;
	u8 alt_fn;
	u8 ext_drv;
	int gpio_en;
	int gpio_pwm;
	const char *name;
};

#endif
