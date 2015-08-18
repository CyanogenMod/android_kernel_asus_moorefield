/*
 * Copyright (c) 2014 Intel Corporation. All Rights Reserved.
 *
 * Partially based on m-5mols kernel driver,
 * Copyright (C) 2011 Samsung Electronics Co., Ltd.
 *
 * Partially based on jc_v4l2 kernel driver from http://opensource.samsung.com
 * Copyright (c) 2011, Code Aurora Forum. All rights reserved.
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
 */

#include <linux/kernel.h>
#include <linux/types.h>
#include "m10mo.h"

static const struct m10mo_resolution const m10mo_preview_modes_fw0[] = {
	{
		.width = 1280,
		.height = 720,
		.command = 0x21,
	},
    {
		.width = 1280,
		.height = 960,
		.command = 0x24,
	},
	{
		.width = 1920,
		.height = 1440,
		.command = 0x52,
	},
    {
		.width = 1920,
		.height = 1080,
		.command = 0x53,
	},
};

static const struct m10mo_resolution const m10mo_capture_modes_fw0[] = {
	{
		.width = 4736,
		.height = 3552,
		.command = 0x2C,
	},
	{
		.width = 4160,
		.height = 3120,
		.command = 0x2C,
		.burst_capture_monitor_size_command = 0x27,
	},
	{
		.width = 2560,
		.height = 1440,
		.command = 0x1C,
		.burst_capture_monitor_size_command = 0x4F,
	},
    {
        .width = 2560,
        .height = 1920,
        .command = 0x1F,
		.burst_capture_monitor_size_command = 0x49,
    },
	{
		.width = 3264,
		.height = 1836,
		.command = 0x21,
		.burst_capture_monitor_size_command = 0x43,
	},
	{
		.width = 3264,
		.height = 2448,
		.command = 0x25,
		.burst_capture_monitor_size_command = 0x29,
	},
	{
		.width = 4160,
		.height = 2340,
		.command = 0x2B,
		.burst_capture_monitor_size_command = 0x5A,
	},
	{
		.width = 2080,
		.height = 1560,
		.command = BINNING_CAP_CMD,
	},
	{
		.width = 2080,
		.height = 1170,
		.command = BINNING_CAP_CMD,
	},
/*
	{
		.width = 1280,
		.height = 720,
		.command = 0x10,
	},
	{
		.width = 1920,
		.height = 1080,
		.command = 0x19,
	},
	{
		.width = 2048,
		.height = 1536,
		.command = 0x1b,
	},
	{
		.width = 4128,
		.height = 2336,
		.command = 0x30,
	},
	{
		.width = 4128,
		.height = 3096,
		.command = 0x2c,
	},
*/
};

static const struct m10mo_resolution const m10mo_video_modes_fw0[] = {
	{
		.width = 320,
		.height = 240,
		.command = 0x09,
	},
	{
		.width = 640,
		.height = 480,
		.command = 0x17,
	},
	{
		.width = 1280,
		.height = 720,
		.command = 0x21,
	},
	{
		.width = 1920,
		.height = 1080,
		.command = 0x28,
	},
	{
		.width = 352,
		.height = 288,
		.command = 0x58,
	},
	{
		.width = 176,
		.height = 144,
		.command = 0x59,
	},
};

const struct m10mo_resolution *resolutions[] = {
	  m10mo_preview_modes_fw0,
	  m10mo_capture_modes_fw0,
	  m10mo_video_modes_fw0 ,
};

const ssize_t resolutions_sizes[] = {
	  ARRAY_SIZE(m10mo_preview_modes_fw0),
	  ARRAY_SIZE(m10mo_capture_modes_fw0),
	  ARRAY_SIZE(m10mo_video_modes_fw0)  ,
};

const struct M10MO_AF_Parameters m10m0_af_parameters[] = {
	/* parameters for firmware M10MO_AF_MODE_0 */
	{
		0x01,
		0x00,
		0x01,
		0x02,
		0x03,
		0x04,
		0x05,
		0x06,
		0x07,

		0x02,
		0x00,
		0x01,
		0x02,

		0x03,
		0x01,
		0x02,
		0x03,
		0x04,
		0x10,
		0x20,
		0x30,
		0x40,

		0x30,
		0x32
	},

	/* parameters for firmware M10MO_AF_MODE_1 */
	{
		0x00,
		0x01,
		0x03,
		0x02,
		0x06,
		0x06,
		0x06,
		0x06,
		0x06,
		0x02,
		0x00,
		0x01,
		0x01,

		0x03,
		0x03,
		0x04,
		0x01,
		0x02,
		0x03,
		0x04,
		0x01,
		0x02,

		0x30,
		0x32
	},

	{},
};
