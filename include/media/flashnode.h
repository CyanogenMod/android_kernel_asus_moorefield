/*
 * leds-flashnode.c	FLASHNODE flash LED driver
 *
 * Copyright 2014 Skyworks Solutions Inc.
 * Author : Gyungoh Yoo <jack.yoo@skyworksinc.com>
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation; either version 2, or (at your option) any
 * later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, see <http://www.gnu.org/licenses/>.
 */

#ifndef _FLASHNODE_H
#define _FLASHNODE_H

#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>

enum flashnode_id
{
	FLASHNODE_FIRST,
	FLASHNODE_FL1 = FLASHNODE_FIRST,
	FLASHNODE_FL2,
	FLASHNODE_MM1,
	FLASHNODE_MM2,
#ifndef CONFIG_SKY81927
	FLASHNODE_BL1,
	FLASHNODE_BL2,
#endif
	FLASHNODE_MAX,
	FLASHNODE_INVALID = FLASHNODE_MAX
};

#define	v4l2_queryctrl_entry_integer(_id, _name,\
		_minimum, _maximum, _step, \
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_INTEGER, \
		.name = _name, \
		.minimum = (_minimum), \
		.maximum = (_maximum), \
		.step = (_step), \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}
#define	v4l2_queryctrl_entry_boolean(_id, _name,\
		_default_value, _flags)	\
	{\
		.id = (_id), \
		.type = V4L2_CTRL_TYPE_BOOLEAN, \
		.name = _name, \
		.minimum = 0, \
		.maximum = 1, \
		.step = 1, \
		.default_value = (_default_value),\
		.flags = (_flags),\
	}

#define	s_ctrl_id_entry_integer(_id, _name, \
		_minimum, _maximum, _step, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_integer(_id, _name,\
				_minimum, _maximum, _step,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

#define	s_ctrl_id_entry_boolean(_id, _name, \
		_default_value, _flags, \
		_s_ctrl, _g_ctrl)	\
	{\
		.qc = v4l2_queryctrl_entry_boolean(_id, _name,\
				_default_value, _flags), \
		.s_ctrl = _s_ctrl, \
		.g_ctrl = _g_ctrl, \
	}

enum flashnode_flash_mode{
	FLASHNODE_MODE_SHUTDOWN,
	FLASHNODE_MODE_FLASH,
	FLASHNODE_MODE_INDICATOR,
	FLASHNODE_MODE_TORCH,
};

enum flashnode_flash_timeout{
	FLASHNODE_FLASHTIMEOUT_OFF, //0
	FLASHNODE_FLASHTIMEOUT_95MS,
	FLASHNODE_FLASHTIMEOUT_190MS,
	FLASHNODE_FLASHTIMEOUT_285MS,
	FLASHNODE_FLASHTIMEOUT_380MS,
	FLASHNODE_FLASHTIMEOUT_475MS,
	FLASHNODE_FLASHTIMEOUT_570MS,
	FLASHNODE_FLASHTIMEOUT_665MS,
	FLASHNODE_FLASHTIMEOUT_760MS,
	FLASHNODE_FLASHTIMEOUT_855MS,
	FLASHNODE_FLASHTIMEOUT_950MS,
	FLASHNODE_FLASHTIMEOUT_1045MS,
	FLASHNODE_FLASHTIMEOUT_1140MS,
	FLASHNODE_FLASHTIMEOUT_1235MS,
	FLASHNODE_FLASHTIMEOUT_1330MS,
	FLASHNODE_FLASHTIMEOUT_1425MS, //15
};

enum flashnode_flash_current{
	FLASHNODE_FLASH_CURRENT_250MA, //0
	FLASHNODE_FLASH_CURRENT_300MA,
	FLASHNODE_FLASH_CURRENT_350MA,
	FLASHNODE_FLASH_CURRENT_400MA,
	FLASHNODE_FLASH_CURRENT_450MA,
	FLASHNODE_FLASH_CURRENT_500MA,
	FLASHNODE_FLASH_CURRENT_550MA,
	FLASHNODE_FLASH_CURRENT_600MA,
	FLASHNODE_FLASH_CURRENT_650MA,
	FLASHNODE_FLASH_CURRENT_700MA,
	FLASHNODE_FLASH_CURRENT_750MA,
	FLASHNODE_FLASH_CURRENT_800MA,
	FLASHNODE_FLASH_CURRENT_850MA,
	FLASHNODE_FLASH_CURRENT_900MA,
	FLASHNODE_FLASH_CURRENT_950MA,
	FLASHNODE_FLASH_CURRENT_1000MA,
	FLASHNODE_FLASH_CURRENT_1100MA,
	FLASHNODE_FLASH_CURRENT_1200MA,
	FLASHNODE_FLASH_CURRENT_1300MA,
	FLASHNODE_FLASH_CURRENT_1400MA,
	FLASHNODE_FLASH_CURRENT_1500MA, //20
};

/**
 * struct flashnode_platform_data
 * @name:	Array of strings for FL1, FL2, MM1, MM2, BL1 and BL2
 * @gpio_enable:The GPIO number for EN pin. 0 if EN pin is not conntrolled.
 * @fl1_by_pin:	true if FL1 is controlled by FLEN1
 * @fl2_by_pin:	true if FL2 is controlled by FLEN2
 * @current_limit:	DC-DC boost converter current limit threshold.
 *		Should be one of 2420, 2600, 2800 and 3000mA.
 * @disable_short_led_report:	true if shorted LED fault should be inhibited.
 * @shutoff_on_inhibit_mode:	Shot off the flash current on FLINH=high
 * @enable_voltage_monitor:	if true, input voltage monitor is enabled.
 * @input_voltage_threshold:	Input voltage monitor threshold level, in mV.
 *		Should be between 2800mV and 3900mV.
 * @input_voltage_hysteresis:	Input voltage monitor threshold level, in mV.
 *		Should be between 2900mV and 4000mV.
 */

struct flashnode_platform_data
{
	const char *name[FLASHNODE_MAX];
	int gpio_enable;
	bool fl1_by_pin;
	bool fl2_by_pin;
	int current_limit;
	bool disable_short_led_report;
	bool shutoff_on_inhibit_mode;
	bool enable_voltage_monitor;
	int input_voltage_threshold;
	int input_voltage_hysteresis;
};

#endif
