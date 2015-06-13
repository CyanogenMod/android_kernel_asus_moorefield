/*
 *  Intel-mid vibrator platform data definitions
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Hevendra Rajareddy <hevendrax.raja.reddy@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef __INTEL_MID_VIBRA_H
#define __INTEL_MID_VIBRA_H

#define INTEL_VIBRA_DRV_NAME "intel_vibra_driver"

#define INTEL_VIBRA_MAX_TIMEDIVISOR  0xFF
#define INTEL_VIBRA_MAX_BASEUNIT 0x8000

#define INTEL_VIBRA_ENABLE_GPIO 40
#define INTEL_PWM_ENABLE_GPIO 49

#if defined(CONFIG_ME372CL) || defined(CONFIG_PF450CL)
#ifdef INTEL_VIBRA_MAX_BASEUNIT
#undef INTEL_VIBRA_MAX_BASEUNIT
#define INTEL_VIBRA_MAX_BASEUNIT 0x80
#endif
#define PWM0DUTYCYCLE   0x67
#define DUTY_VALUE_MAX  0x63
#define PWM0CLKDIV1     0x61
#define PWM0CLKDIV0     0x62
#endif /* CONFIG_ME372CL || CONFIG_PF450CL */

struct mid_vibra_pdata {
	u8 time_divisor;
	u8 base_unit;
	u8 alt_fn;
	u8 ext_drv;
	int gpio_en;
	int gpio_pwm;
	const char *name;
	bool use_gpio_en; /* whether vibra needs gpio based enable control */
};

#endif /* __INTEL_MID_VIBRA_H */
