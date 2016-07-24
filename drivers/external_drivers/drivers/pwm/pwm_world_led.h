/*
 *  Intel-mid world led local header
 *
 *  Copyright (C) 2013 Intel Corp
 *  Author: Yun Wei <yun.wei@intel.com>
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
#ifndef _WORLD_LED_H
#define _WORLD_LED_H

struct world_led_info {
	int     enabled;
	struct mutex	lock;
	struct device	*dev;
	void __iomem	*shim;
	const char	*name;
	unsigned long *base_unit;
	unsigned long *duty_cycle;
	u8  max_base_unit;
	u8  max_duty_cycle;
	int gpio_pwm;
	int alt_fn;
	int ext_drv;

	void (*enable)(struct world_led_info *info);
	void (*disable)(struct world_led_info *info);
	int (*pwm_configure)(struct world_led_info *info, bool enable);

	const struct attribute_group *world_led_attr_group;
};

struct world_led_info *world_led_setup(struct device *dev, struct world_led_pdata *data);
#endif
