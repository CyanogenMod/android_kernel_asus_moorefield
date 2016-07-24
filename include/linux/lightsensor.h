/* include/linux/lightsensor.h
 *
 * Copyright (C) 2009 Google, Inc.
 * Author: Iliyan Malchev <malchev@google.com>
 *  
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifndef __LINUX_LIGHTSENSOR_H
#define __LINUX_LIGHTSENSOR_H

#include <linux/types.h>
#include <linux/ioctl.h>
/*
#define LIGHTSENSOR_IOCTL_MAGIC 'l'

#define LIGHTSENSOR_IOCTL_GET_ENABLED _IOR(LIGHTSENSOR_IOCTL_MAGIC, 1, int *)
#define LIGHTSENSOR_IOCTL_ENABLE _IOW(LIGHTSENSOR_IOCTL_MAGIC, 2, int *)
*/
/*ASUS - bevis chen-- add +*/
#define LBUFF_SIZE				16	/* Rx buffer size */

#define ASUS_LIGHTSENSOR_IOCTL_CLOSE		         _IO(LIGHTSENSOR_IOCTL_MAGIC, 0x12)
#define ASUS_LIGHTSENSOR_IOCTL_START		         _IO(LIGHTSENSOR_IOCTL_MAGIC, 0x13)
#define ASUS_LIGHTSENSOR_IOCTL_GETDATA             _IOR(LIGHTSENSOR_IOCTL_MAGIC, 0x14, char[LBUFF_SIZE+1])

#define ASUS_LIGHTSENSOR_SETCALI_DATA	         _IOW(LIGHTSENSOR_IOCTL_MAGIC, 0x15, int[2])
#define ASUS_LIGHTSENSOR_EN_CALIBRATION           _IOW(LIGHTSENSOR_IOCTL_MAGIC, 0x16, char)
/*ASUS - bevis chen-- add -*/



struct lightsensor_mpp_config_data {
	uint32_t lightsensor_mpp;
	uint32_t lightsensor_amux;
};

struct lightsensor_smd_platform_data {
	const char      *name;
	uint16_t        levels[10];
	uint16_t        golden_adc;
	int             (*ls_power)(int, uint8_t);
	struct lightsensor_mpp_config_data mpp_data;
};

#endif
