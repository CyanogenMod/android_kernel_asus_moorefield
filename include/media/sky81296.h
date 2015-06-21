/*
 * leds-sky81296.c	SKY81296 flash LED driver
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

#ifndef _SKY81296_H
#define _SKY81296_H

#include <linux/videodev2.h>
#include <media/v4l2-subdev.h>

enum sky81296_id
{
	SKY81296_FIRST,
	SKY81296_FL1 = SKY81296_FIRST,
	SKY81296_FL2,
	SKY81296_MM1,
	SKY81296_MM2,
#ifndef CONFIG_SKY81927
	SKY81296_BL1,
	SKY81296_BL2,
#endif
	SKY81296_MAX,
	SKY81296_INVALID = SKY81296_MAX
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

enum sky81296_flash_mode{
	SKY81296_MODE_SHUTDOWN,
	SKY81296_MODE_FLASH,
	SKY81296_MODE_INDICATOR,
	SKY81296_MODE_TORCH,
};

enum sky81296_flash_timeout{
	SKY81296_FLASHTIMEOUT_OFF, //0
	SKY81296_FLASHTIMEOUT_95MS,
	SKY81296_FLASHTIMEOUT_190MS,
	SKY81296_FLASHTIMEOUT_285MS,
	SKY81296_FLASHTIMEOUT_380MS,
	SKY81296_FLASHTIMEOUT_475MS,
	SKY81296_FLASHTIMEOUT_570MS,
	SKY81296_FLASHTIMEOUT_665MS,
	SKY81296_FLASHTIMEOUT_760MS,
	SKY81296_FLASHTIMEOUT_855MS,
	SKY81296_FLASHTIMEOUT_950MS,
	SKY81296_FLASHTIMEOUT_1045MS,
	SKY81296_FLASHTIMEOUT_1140MS,
	SKY81296_FLASHTIMEOUT_1235MS,
	SKY81296_FLASHTIMEOUT_1330MS,
	SKY81296_FLASHTIMEOUT_1425MS, //15
};

enum sky81296_flash_current{
	SKY81296_FLASH_CURRENT_250MA, //0
	SKY81296_FLASH_CURRENT_300MA,
	SKY81296_FLASH_CURRENT_350MA,
	SKY81296_FLASH_CURRENT_400MA,
	SKY81296_FLASH_CURRENT_450MA,
	SKY81296_FLASH_CURRENT_500MA,
	SKY81296_FLASH_CURRENT_550MA,
	SKY81296_FLASH_CURRENT_600MA,
	SKY81296_FLASH_CURRENT_650MA,
	SKY81296_FLASH_CURRENT_700MA,
	SKY81296_FLASH_CURRENT_750MA,
	SKY81296_FLASH_CURRENT_800MA,
	SKY81296_FLASH_CURRENT_850MA,
	SKY81296_FLASH_CURRENT_900MA,
	SKY81296_FLASH_CURRENT_950MA,
	SKY81296_FLASH_CURRENT_1000MA,
	SKY81296_FLASH_CURRENT_1100MA,
	SKY81296_FLASH_CURRENT_1200MA,
	SKY81296_FLASH_CURRENT_1300MA,
	SKY81296_FLASH_CURRENT_1400MA,
	SKY81296_FLASH_CURRENT_1500MA, //20
};

enum sky81296_torch_current{
	SKY81296_TORCH_CURRENT_25MA, //0
	SKY81296_TORCH_CURRENT_50MA,
	SKY81296_TORCH_CURRENT_75MA,
	SKY81296_TORCH_CURRENT_100MA,
	SKY81296_TORCH_CURRENT_125MA,
	SKY81296_TORCH_CURRENT_150MA,
	SKY81296_TORCH_CURRENT_175MA,
	SKY81296_TORCH_CURRENT_200MA,
	SKY81296_TORCH_CURRENT_225MA,
	SKY81296_TORCH_CURRENT_250MA, // 9
	SKY81296_TORCH_CURRENT_NUM,
};


static int inline mapping_torch_intensity(int light_intensity_percentage){
    int ret;
    ret = SKY81296_TORCH_CURRENT_25MA;
    if(light_intensity_percentage > 10)
        ret = SKY81296_TORCH_CURRENT_25MA;
    if(light_intensity_percentage > 20)
        ret = SKY81296_TORCH_CURRENT_50MA;
    if(light_intensity_percentage > 30)
        ret = SKY81296_TORCH_CURRENT_75MA;
    if(light_intensity_percentage > 40)
        ret = SKY81296_TORCH_CURRENT_100MA;
    if(light_intensity_percentage > 60)
        ret = SKY81296_TORCH_CURRENT_125MA;
    if(light_intensity_percentage > 80)
    	ret = SKY81296_TORCH_CURRENT_150MA;
	if(light_intensity_percentage > 90)
    	ret = SKY81296_TORCH_CURRENT_175MA;
    return ret;
}

/**
 * struct sky81296_platform_data
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

struct sky81296_platform_data
{
	const char *name[SKY81296_MAX];
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
