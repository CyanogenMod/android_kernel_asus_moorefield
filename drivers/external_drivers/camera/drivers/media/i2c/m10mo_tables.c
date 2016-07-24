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
#include <linux/m10mo.h>

static const struct m10mo_resolution const m10mo_preview_modes_fw0[] = {
	{
		.width = 176,
		.height = 144,
		.command = 0x5,
	},
	{
		.width = 320,
		.height = 240,
		.command = 0x09,
	},
	{
		.width = 352,
		.height = 288,
		.command = 0x0E,
	},
	{
		.width = 640,
		.height = 360,
		.command = 0x15,
	},
	{
		.width = 640,
		.height = 480,
		.command = 0x17,
	},
	{
		.width = 720,
		.height = 480,
		.command = 0x18,
	},
	{
		.width = 720,
		.height = 576,
		.command = 0x5E,
	},
	{
		.width = 960,
		.height = 720,
		.command = 0x34,
	},
	{
		.width = 1024,
		.height = 576,
		.command = 0x5F,
	},
	{
		.width = 1024,
		.height = 768,
		.command = 0x20,
	},
// =========================== //
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
		.command = 0x28,
	},
//=== For binning 1920x1080, hard code to set in m10mo_set_zsl_monitor. ===//
//	{
//		.width = 1920,
//		.height = 1080,
//		.command = 0x53,
//	},
};

static const struct m10mo_resolution const m10mo_capture_modes_fw0[] = {
	{
		.width = 4736,
		.height = 3552,
		.command = 0x2C,
	},
//=========================//
	{
		.width = 640,
		.height = 480,
		.command = 0x09,
	},
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
//========================//
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
		.burst_capture_monitor_size_command = 0x5B,
	},
	{
		.width = 2080,
		.height = 1170,
		.command = BINNING_CAP_CMD,
		.burst_capture_monitor_size_command = 0x5C,
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
		.width = 176,
		.height = 144,
		.command = 0x5,
	},
	{
		.width = 320,
		.height = 240,
		.command = 0x09,
	},
	{
		.width = 352,
		.height = 288,
		.command = 0x0E,
	},
	{
		.width = 640,
		.height = 360,
		.command = 0x15,
	},
	{
		.width = 640,
		.height = 480,
		.command = 0x17,
	},
	{
		.width = 720,
		.height = 480,
		.command = 0x18,
	},
	{
		.width = 720,
		.height = 576,
		.command = 0x5E,
	},
	{
		.width = 960,
		.height = 720,
		.command = 0x34,
	},
	{
		.width = 1024,
		.height = 576,
		.command = 0x5F,
	},
	{
		.width = 1024,
		.height = 768,
		.command = 0x20,
	},
// =========================== //
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
		.command = 0x28,
	},


//=== Following resolutions are For m10mo's VSS resolution, but we do not use now. ===///
//=== We use m10mo's monitor resolutions to do recording. ===//
#if 0
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
#endif
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

const struct m10mo_focus_data_t m10mo_focus_data_table[] = {
//=== Number x 100 then report to HAL. ===//
//=== Example: 270 means 2.7           ===//
    {  // dummy
	   .f_number = 0,
	   .focal_len = 0,
	},
	{   // zoom step: 1, 1-0
	   .f_number = 270,
	   .focal_len = 380,
	},
	{   // zoom step: 2, 1-1
	   .f_number = 280,
	   .focal_len = 400,
	},
	{   // zoom step: 3, 1-2
	   .f_number = 280,
	   .focal_len = 410,
	},
	{   // zoom step: 4, 1-3
	   .f_number = 290,
	   .focal_len = 425,
	},
	{   // zoom step: 5, 2-0
	   .f_number = 300,
	   .focal_len = 440,
	},
	{   // zoom step: 6, 2-1
	   .f_number = 300,
	   .focal_len = 455,
	},
	{   // zoom step: 7, 2-2
	   .f_number = 300,
	   .focal_len = 470,
	},
	{   // zoom step: 8, 2-3
	   .f_number = 310,
	   .focal_len = 490,
	},
	{   // zoom step: 9, 3-0
	   .f_number = 320,
	   .focal_len = 510,
	},
	{   // zoom step: 10, 3-1
	   .f_number = 320,
	   .focal_len = 525,
	},
	{   // zoom step: 11, 3-2
	   .f_number = 330,
	   .focal_len = 540,
	},
	{   // zoom step: 12, 3-3
	   .f_number = 330,
	   .focal_len = 560,
	},
	{   // zoom step: 13, 4-0
	   .f_number = 340,
	   .focal_len = 580,
	},
	{   // zoom step: 14, 4-1
	   .f_number = 340,
	   .focal_len = 595,
	},
	{   // zoom step: 15, 4-2
	   .f_number = 350,
	   .focal_len = 615,
	},
	{   // zoom step: 16, 4-3
	   .f_number = 360,
	   .focal_len = 640,
	},
	{   // zoom step: 17, 5-0
	   .f_number = 360,
	   .focal_len = 660,
	},
	{   // zoom step: 18, 5-1
	   .f_number = 370,
	   .focal_len = 680,
	},
	{   // zoom step: 19, 5-2
	   .f_number = 370,
	   .focal_len = 700,
	},
	{   // zoom step: 20, 5-3
	   .f_number = 380,
	   .focal_len = 725,
	},
	{   // zoom step: 21, 6-0
	   .f_number = 390,
	   .focal_len = 750,
	},
	{   // zoom step: 22, 6-1
	   .f_number = 390,
	   .focal_len = 775,
	},
	{   // zoom step: 23, 6-2
	   .f_number = 400,
	   .focal_len = 800,
	},
	{   // zoom step: 24, 6-3
	   .f_number = 410,
	   .focal_len = 830,
	},
	{   // zoom step: 25, 7-0
	   .f_number = 420,
	   .focal_len = 860,
	},
	{   // zoom step: 26, 7-1
	   .f_number = 430,
	   .focal_len = 885,
	},
	{   // zoom step: 27, 7-2
	   .f_number = 430,
	   .focal_len = 917,
	},
	{   // zoom step: 28, 7-3
	   .f_number = 440,
	   .focal_len = 950,
	},
	{   // zoom step: 29, 8-0
	   .f_number = 450,
	   .focal_len = 980,
	},
	{   // zoom step: 30, 8-1
	   .f_number = 460,
	   .focal_len = 1015,
	},
	{   // zoom step: 31, 8-2
	   .f_number = 470,
	   .focal_len = 1045,
	},
	{   // zoom step: 32, 9
	   .f_number = 480,
	   .focal_len = 1140,
	},
};
