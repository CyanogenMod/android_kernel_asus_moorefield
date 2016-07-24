/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */

#ifndef __IA_CSS_FWCTRL_PUBLIC_H__
#define __IA_CSS_FWCTRL_PUBLIC_H__

enum ia_css_fwctrl_event_type {
	IA_CSS_FWCTRL_NO_EVENT,
	IA_CSS_FWCTRL_ISYS_EVENT,
	IA_CSS_FWCTRL_PSYS_EVENT,
	IA_CSS_FWCTRL_MAX_MSG
};

/* TODO: open, close and devconfig must be private once isys sync's to the
 * new device API's */
/*Use this structure to provide firmware address
during queue initialization and sp start */
struct ia_css_fwctrl_devconfig{
	void * firmware_address;
};

/**
 * @brief  ia_css_fwctrl_device_open
 *
 * @return:
 *     int  : status of the API execution
 */
extern int ia_css_fwctrl_device_open(
	struct ia_css_fwctrl_devconfig *device_config
);

/**
 * @brief  ia_css_fwctrl_device_close
 *
 * @return:
 *     int  : status of the API execution
 */
extern int ia_css_fwctrl_device_close(void);

/**
 * @brief  ia_css_fwctrl_dequeue_event
 *
 * @return:
 *     int  : status of the API execution
 */
extern int ia_css_fwctrl_dequeue_event(
	enum ia_css_fwctrl_event_type *event_type
);

#endif /*__IA_CSS_FWCTRL_PUBLIC_H__*/
