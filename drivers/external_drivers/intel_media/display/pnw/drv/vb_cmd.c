/*
 * Copyright © 2010 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/i2c.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/slab.h>
#include <linux/gpio.h>
#include <linux/poll.h>
#include <linux/earlysuspend.h>
#include <linux/miscdevice.h>
#include <linux/pm_runtime.h>
#include <linux/notifier.h>
#include <linux/reboot.h>

#include "mdfld_dsi_dbi.h"
#include "mdfld_dsi_pkg_sender.h"
#include "mdfld_dsi_esd.h"

#define I2C_ADAPTER 0x02
#define I2C_ADDRESS 0x54

enum vb_panel_type {
	PANEL_IGZO = 0,
	PANEL_CGS,
	PANEL_IGZO_G8
};

enum dsi_init_command_type {
	GEN_LONG_LP_CMD,
	GEN_SHORT_LP_CMD,
	MCS_LONG_LP_CMD,
	MCS_SHORT_LP_CMD,
	GEN_LONG_HS_CMD,
	GEN_SHORT_HS_CMD,
	MCS_LONG_HS_CMD,
	MCS_SHORT_HS_CMD,
	DELAY_CMD,
	DONE_CMD
};

struct dsi_init_command {
	enum dsi_init_command_type type;
	u32 count;
	u8 *param;
};

#define GENERIC_CMD(type, param) {type, sizeof(param), param}
#define GEN_LONG_LP(param) GENERIC_CMD(GEN_LONG_LP_CMD, param)
#define GEN_SHORT_LP(param) GENERIC_CMD(GEN_SHORT_LP_CMD, param)
#define MCS_LONG_LP(param) GENERIC_CMD(MCS_LONG_LP_CMD, param)
#define MCS_SHORT_LP(param) GENERIC_CMD(MCS_SHORT_LP_CMD, param)
#define GEN_LONG_HS(param) GENERIC_CMD(GEN_LONG_HS_CMD, param)
#define GEN_SHORT_HS(param) GENERIC_CMD(GEN_SHORT_HS_CMD, param)
#define MCS_LONG_HS(param) GENERIC_CMD(MCS_LONG_HS_CMD, param)
#define MCS_SHORT_HS(param) GENERIC_CMD(MCS_SHORT_HS_CMD, param)
#define DELAY(delay) {DELAY_CMD, delay, NULL}
#define DONE() {DONE_CMD, 0, NULL}

static u8 ls04x_mcap[] = { 0xb0, 0x00 };
static u8 ls04x_mcap_cgs[] = { 0xb0, 0x04 };
static u8 ls04x_device_code_read[] = { 0xbf };
static u8 ls04x_frame_memory_access_igzo[] = {
	0xb3, 0x00, 0xc0, 0x00,
	0x00, 0x00, 0x00};
static u8 ls04x_frame_memory_access_igzo_g8[] = {
	0xb3, 0x0c, 0x10, 0x00,
	0x00, 0x00};
static u8 ls04x_frame_memory_access_cgs[] = {
	0xb3, 0x00, 0x00, 0x22,
	0x00, 0x00};
static u8 ls04x_interface_id[] = {
	0xb4, 0x0c, 0x12};
static u8 ls04x_interface_id_igzo_g8[] = {
	0xb4, 0x0c, 0x00};
static u8 ls04x_dsi_control1_igzo[] = {
	0xb6, 0x39, 0xb3};
static u8 ls04x_dsi_control1_igzo_g8[] = {
	0xb6, 0x09, 0xa3};
static u8 ls04x_dsi_control1_cgs[] = {
	0xb6, 0x31, 0xb5};
static u8 ls04x_dsi_control2[] = { 0xb7, 0x00 };
static u8 ls04x_external_clock_igzo_g8[] = {
	0xbe, 0x00, 0x04};
static u8 ls04x_panel_pin_ctrl[] = {
	0xcb, 0x67, 0x26, 0xc0,
	0x19, 0x0e, 0x00, 0x00,
	0x00, 0x00, 0xc0, 0x00};
static u8 ls04x_panel_pin_ctrl_igzo_g8[] = {
	0xcb, 0x80, 0xcc, 0x34,
	0x0b, 0x73, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 ls04x_panel_interface_ctrl_igzo[] = { 0xcc, 0x01 };
static u8 ls04x_panel_interface_ctrl_igzo_g8[] = { 0xcc, 0x03 };
static u8 ls04x_panel_interface_ctrl_cgs[] = { 0xcc, 0x06 };
static u8 ls04x_display_setting1_igzo[] = {
	0xc1, 0x0c, 0x62, 0x40,
	0x52, 0x02, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x02,
	0xc7, 0x06, 0x02, 0x08,
	0x09, 0x08, 0x09, 0x00,
	0x00, 0x00, 0x00, 0x62,
	0x30, 0x40, 0xa5, 0x0f,
	0x04, 0x00, 0x20, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00};
static u8 ls04x_display_setting1_igzo_g8[] = {
	0xc1, 0x8c, 0xa2, 0x00,
	0x53, 0xc3, 0x91, 0xab,
	0xab, 0x06, 0x04, 0x26,
	0x06, 0xba, 0xba, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00,
	0x00, 0x62, 0x30, 0x40,
	0xa5, 0x0f, 0x04, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00};
static u8 ls04x_display_setting1_cgs[] = {
	0xc1, 0x04, 0x64, 0x10,
	0x41, 0x00, 0x00, 0x8e,
	0x29, 0xef, 0xbd, 0xf7,
	0xde, 0x7b, 0xef, 0xbd,
	0xf7, 0xde, 0x7b, 0xc5,
	0x1c, 0x02, 0x86, 0x08,
	0x22, 0x22, 0x00, 0x20};
static u8 ls04x_display_setting2_igzo[] = {
	0xc2, 0x30, 0xf5, 0x00,
	0x0c, 0x0e, 0x00, 0x00};
static u8 ls04x_display_setting2_igzo_g8[] = {
	0xc2, 0x50, 0xf5, 0x00,
	0x0c, 0x4d, 0x00, 0x00};
static u8 ls04x_display_setting2_cgs[] = {
	0xc2, 0x20, 0xf5, 0x00,
	0x08, 0x08, 0x00};
static u8 ls04x_tpc_sync_ctrl[] = {
	0xc3, 0x00, 0x00, 0x00};
static u8 ls04x_source_timing_setting_igzo[] = {
	0xc4, 0x70, 0x00, 0x00,
	0x00, 0x43, 0x2d, 0x00,
	0x00, 0x00, 0x00, 0x03,
	0x2d, 0x00};
static u8 ls04x_source_timing_setting_igzo_g8[] = {
	0xc4, 0x70, 0x00, 0x0f,
	0x0f, 0xcf, 0x00, 0x01,
	0x01, 0x00, 0x00, 0x01};
static u8 ls04x_source_timing_setting_cgs[] = {
	0xc4, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x09,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x09};
static u8 ls04x_gip_timing_setting[] = {
	0xc6, 0xb2, 0x13, 0xa2,
	0xb2, 0x13, 0xa2};
static u8 ls04x_gip_timing_setting_igzo_g8[] = {
	0xc6, 0xb3, 0x7a, 0x7a,
	0x00, 0x00};
static u8 ls04x_ltps_timing_setting_cgs[] = {
	0xc6, 0xb0, 0x00, 0xb1,
	0x05, 0xa6, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0b, 0x21,
	0x10, 0xb0, 0x00, 0xb1,
	0x05, 0xa6, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x0b, 0x21,
	0x10};
static u8 ls04x_gamma_a_setting_igzo[] = {
	0xc7, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x34, 0x41, 0x4d, 0x5e,
	0x6b, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x32, 0x3e, 0x4a, 0x5b,
	0x67};
static u8 ls04x_gamma_b_setting_igzo[] = {
	0xc8, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x34, 0x41, 0x4d, 0x5e,
	0x6b, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x32, 0x3e, 0x4a, 0x5b,
	0x67};
static u8 ls04x_gamma_c_setting_igzo[] = {
	0xc9, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x34, 0x41, 0x4d, 0x5e,
	0x6b, 0x00, 0x06, 0x0c,
	0x13, 0x20, 0x36, 0x26,
	0x32, 0x3e, 0x4a, 0x5b,
	0x67};
static u8 ls04x_gamma_a_setting_igzo_g8[] = {
	0xc7, 0x00, 0x10, 0x1c,
	0x2b, 0x38, 0x43, 0x5c,
	0x6e, 0x7c, 0x89, 0x3b,
	0x47, 0x55, 0x69, 0x72,
	0x7e, 0x8c, 0x93, 0x97,
	0x00, 0x10, 0x1c, 0x2b,
	0x38, 0x43, 0x5c, 0x6e,
	0x7c, 0x89, 0x3b, 0x47,
	0x55, 0x69, 0x72, 0x7e,
	0x8c, 0x93, 0x97};
static u8 ls04x_gamma_b_setting_igzo_g8[] = {
	0xc8, 0x00, 0x10, 0x1c,
	0x2b, 0x38, 0x43, 0x5c,
	0x6e, 0x7c, 0x89, 0x3b,
	0x47, 0x55, 0x69, 0x72,
	0x7e, 0x8c, 0x93, 0x97,
	0x00, 0x10, 0x1c, 0x2b,
	0x38, 0x43, 0x5c, 0x6e,
	0x7c, 0x89, 0x3b, 0x47,
	0x55, 0x69, 0x72, 0x7e,
	0x8c, 0x93, 0x97};
static u8 ls04x_gamma_c_setting_igzo_g8[] = {
	0xc9, 0x00, 0x10, 0x1c,
	0x2b, 0x38, 0x43, 0x5c,
	0x6e, 0x7c, 0x89, 0x3b,
	0x47, 0x55, 0x69, 0x72,
	0x7e, 0x8c, 0x93, 0x97,
	0x00, 0x10, 0x1c, 0x2b,
	0x38, 0x43, 0x5c, 0x6e,
	0x7c, 0x89, 0x3b, 0x47,
	0x55, 0x69, 0x72, 0x7e,
	0x8c, 0x93, 0x97};
static u8 ls04x_gamma_a_setting_cgs[] = {
	0xc7, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d};
static u8 ls04x_gamma_b_setting_cgs[] = {
	0xc8, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d};
static u8 ls04x_gamma_c_setting_cgs[] = {
	0xc9, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d, 0x00, 0x0c, 0x15,
	0x1f, 0x2e, 0x42, 0x2e,
	0x3b, 0x46, 0x51, 0x5f,
	0x6d};
static u8 ls04x_backlight_setting1_pwr_save[] = {
	0xb8, 0x18, 0x80, 0x18,
	0x18, 0xcf, 0x1f, 0x00,
	0x0c, 0x10, 0x5c, 0x10,
	0xac, 0x10, 0x0c, 0x10,
	0xda, 0x6d, 0xff, 0xff,
	0x10, 0x67, 0x89, 0xaf,
	0xd6, 0xff};
static u8 ls04x_backlight_setting2_pwr_save[] = {
	0xb9, 0x0f, 0x18, 0x04,
	0x40, 0x9f, 0x1f, 0x80};
static u8 ls04x_backlight_setting4_pwr_save[] = {
	0xba, 0x0f, 0x18, 0x04,
	0x40, 0x9f, 0x1f, 0xd7};
static u8 ls04x_backlight_setting4_igzo_g8[] = {
	0xce, 0x7d, 0x40, 0x48,
	0x56, 0x67, 0x78, 0x88,
	0x98, 0xa7, 0xb5, 0xc3,
	0xd1, 0xde, 0xe9, 0xf2,
	0xfa, 0xff, 0x04, 0x00,
	0x04, 0x04, 0x44, 0x00,
	0x00};
static u8 ls04x_backlight_setting6_igzo[] = {
	0xce, 0x00, 0x04, 0x08,
	0x01, 0x00, 0x00, 0x00};
static u8 ls04x_power_setting_igzo[] = {
	0xd0, 0x00, 0x10, 0x19,
	0x18, 0x99, 0x99, 0x18,
	0x00, 0x89, 0x01, 0xbb,
	0x0c, 0x8f, 0x0e, 0x21,
	0x20};
static u8 ls04x_power_setting_igzo_g8[] = {
	0xd0, 0x11, 0x43, 0x63,
	0xe3, 0x4c, 0x19, 0x19,
	0x0c, 0x0c, 0xaa};
static u8 ls04x_power_setting_cgs[] = {
	0xd0, 0x10, 0x4c, 0x18,
	0xcc, 0xda, 0x5a, 0x01,
	0x8a, 0x01, 0xbb, 0x58,
	0x4a};
static u8 ls04x_power_setting_switching_reg_igzo_g8[] = {
	0xd1, 0x10, 0x00, 0x80,
	0x02, 0x04, 0x06, 0x08,
	0x00, 0xc0, 0x00, 0x00,
	0x1e, 0x02, 0xd0, 0x00,
	0x80, 0x02, 0x04, 0x06,
	0x08, 0x00, 0x00, 0x1e,
	0x03, 0x00};
static u8 ls04x_power_setting_int_pwr1_igzo[] = { 0xd2, 0x9c };
static u8 ls04x_power_setting_int_pwr1_cgs[] = { 0xd2, 0xb8 };
static u8 ls04x_power_setting_int_pwr2_igzo[] = {
	0xd3, 0x1b, 0xb3, 0xbb,
	0xbb, 0x33, 0x33, 0x33,
	0x33, 0x55, 0x01, 0x00,
	0xa0, 0xa8, 0xa0, 0x07,
	0xc7, 0xb7, 0x33, 0xa2,
	0x73, 0xc7, 0x00, 0x00,
	0x00};
static u8 ls04x_power_setting_int_pwr2_igzo_g8[] = {
	0xd3, 0x1b, 0x33, 0xbb,
	0xbb, 0xb3, 0x33, 0x33,
	0x33, 0x80, 0x88, 0x9f,
	0x5f, 0xc2, 0x42, 0x33,
	0x33, 0xf7, 0xf3, 0x3d,
	0xbc, 0x07, 0x00, 0x00,
	0x00};
static u8 ls04x_power_setting_int_pwr2_cgs[] = {
	0xd3, 0x1a, 0xb3, 0xbb,
	0xff, 0x77, 0x33, 0x33,
	0x33, 0x00, 0x01, 0x00,
	0xa0, 0x38, 0xa0, 0x00,
	0xdb, 0xb7, 0x33, 0xa2,
	0x72, 0xdb};
static u8 ls04x_vcom_setting[] = {
	0xd5, 0x06, 0x00, 0x00,
	0x01, 0x2b, 0x01, 0x2b};
static u8 ls04x_vcom_setting_igzo_g8[] = {
	0xd5, 0x0e, 0x00, 0x00,
	0x01, 0xa0, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00};
static u8 ls04x_vcom_setting_igzo_g8_poweroff[] = {
	0xd5, 0x0e, 0x00, 0x00,
	0x01, 0x50, 0x00, 0x00,
	0x01, 0x00, 0x00, 0x00};
static u8 ls04x_sleep_out_nvm_load_setting[] = { 0xd6, 0x01 };
static u8 ls04x_sequencer_timing_power_on_igzo[] = {
	0xd7, 0x44, 0x01, 0xff,
	0xff, 0x3f, 0xfc, 0x51,
	0x9d, 0x71, 0xf0, 0x0f,
	0x00, 0xe0, 0xff, 0x01,
	0xf0, 0x03, 0x00, 0x1e,
	0x00, 0x08, 0x94, 0x40,
	0xf0, 0x73, 0x7f, 0x78,
	0x08, 0xc0, 0x07, 0x0a,
	0x55, 0x15, 0x28, 0x54,
	0x55, 0xa0, 0xf0, 0xff,
	0x00, 0x40, 0x55, 0x05,
	0x00, 0x20, 0x20, 0x01};
static u8 ls04x_sequencer_timing_power_on_cgs[] = {
	0xd7, 0x20, 0x80, 0xfc,
	0xff, 0x7f, 0x22, 0xa2,
	0xa2, 0x80, 0x0a, 0xf0,
	0x60, 0x7e, 0x00, 0x3c,
	0x18, 0x40, 0x05, 0x7e,
	0x00, 0x00, 0x00};
static u8 ls04x_low_power_function[] = {
	0xd9, 0x09, 0x00, 0x4f,
	0x07, 0x00, 0x10, 0x00,
	0xc0, 0x00, 0x76, 0x33,
	0x33, 0x00, 0xf0, 0x33,
	0x33};
static u8 ls04x_lowpower_function1[] = {
	0xdd, 0x21, 0x31, 0x01,
	0xb2, 0x09, 0x05, 0x00,
	0xf7, 0x00, 0x00, 0xfa,
	0xc1, 0x80, 0x00, 0x00,
	0x00, 0x3f, 0x88, 0x88,
	0x00, 0x71, 0x01};
static u8 ls04x_seq_timing_ctrl_gip_igzo_g8[] = {
	0xeb, 0x09, 0x55, 0xe4,
	0x00, 0x04, 0x00, 0x55,
	0x01, 0x00, 0x00, 0xa8,
	0x6a, 0x55, 0xaa, 0xaa,
	0x80, 0x68, 0x20, 0xff,
	0xff, 0xff, 0xff, 0xff,
	0xaa, 0xa4, 0xa4, 0xaa,
	0x01, 0x09, 0x55, 0x00};
static u8 ls04x_panel_sync_out1[] = {
	0xec, 0x01, 0x40};
static u8 ls04x_panel_sync_out1_igzo_g8[] = {
	0xec, 0x04, 0x10, 0x00,
	0x00};
static u8 ls04x_panel_sync_out2[] = {
	0xed, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 ls04x_panel_sync_out2_igzo_g8[] = {
	0xed, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00};
static u8 ls04x_panel_sync_out3[] = { 0xee, 0x00 };
static u8 ls04x_panel_sync_out4[] = {
	0xef, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00, 0x00, 0x00,
	0x00, 0x00};
static u8 ls04x_set_column_addr[] = {
	0x2a, 0x00, 0x00, 0x02,
	0xcf};
static u8 ls04x_set_page_addr[] = {
	0x2b, 0x00, 0x00, 0x04,
	0xff};
static u8 ls04x_ledpwm_on[] = { 0x53, 0x2c };
static u8 ls04x_cabc_on[] = { 0x55, 0x01 };
static u8 ls04x_deep_standby[] = { 0xb1, 0x01 };

static struct dsi_init_command igzo_init_commands[] = {
	GEN_SHORT_HS(ls04x_mcap),
	GEN_LONG_HS(ls04x_frame_memory_access_igzo),
	GEN_LONG_HS(ls04x_interface_id),
	GEN_LONG_HS(ls04x_dsi_control1_igzo),
	GEN_SHORT_HS(ls04x_dsi_control2),
	GEN_LONG_HS(ls04x_panel_pin_ctrl),
	GEN_SHORT_HS(ls04x_panel_interface_ctrl_igzo),
	GEN_LONG_HS(ls04x_display_setting1_igzo),
	GEN_LONG_HS(ls04x_display_setting2_igzo),
	GEN_LONG_HS(ls04x_tpc_sync_ctrl),
	GEN_LONG_HS(ls04x_source_timing_setting_igzo),
	GEN_LONG_HS(ls04x_gip_timing_setting),
	GEN_LONG_HS(ls04x_backlight_setting1_pwr_save),
	GEN_LONG_HS(ls04x_backlight_setting2_pwr_save),
	GEN_LONG_HS(ls04x_backlight_setting4_pwr_save),
	GEN_LONG_HS(ls04x_backlight_setting6_igzo),
	GEN_LONG_HS(ls04x_gamma_a_setting_igzo),
	GEN_LONG_HS(ls04x_gamma_b_setting_igzo),
	GEN_LONG_HS(ls04x_gamma_c_setting_igzo),
	GEN_LONG_HS(ls04x_power_setting_igzo),
	GEN_SHORT_HS(ls04x_power_setting_int_pwr1_igzo),
	GEN_LONG_HS(ls04x_power_setting_int_pwr2_igzo),
	GEN_SHORT_HS(ls04x_sleep_out_nvm_load_setting),
	GEN_LONG_HS(ls04x_sequencer_timing_power_on_igzo),
	GEN_LONG_HS(ls04x_low_power_function),
	GEN_LONG_HS(ls04x_panel_sync_out1),
	GEN_LONG_HS(ls04x_panel_sync_out2),
	GEN_SHORT_HS(ls04x_panel_sync_out3),
	GEN_LONG_HS(ls04x_panel_sync_out4),
	MCS_SHORT_HS(ls04x_ledpwm_on),
	MCS_SHORT_HS(ls04x_cabc_on),
	MCS_LONG_HS(ls04x_set_column_addr),
	MCS_LONG_HS(ls04x_set_page_addr),
	DONE()
};

static struct dsi_init_command igzo_g8_init_commands[] = {
	GEN_SHORT_HS(ls04x_mcap),
	GEN_LONG_HS(ls04x_frame_memory_access_igzo_g8),
	GEN_LONG_HS(ls04x_interface_id_igzo_g8),
	GEN_LONG_HS(ls04x_dsi_control1_igzo_g8),
	GEN_LONG_HS(ls04x_external_clock_igzo_g8),
	GEN_LONG_HS(ls04x_panel_pin_ctrl_igzo_g8),
	GEN_SHORT_HS(ls04x_panel_interface_ctrl_igzo_g8),
	GEN_LONG_HS(ls04x_display_setting1_igzo_g8),
	GEN_LONG_HS(ls04x_display_setting2_igzo_g8),
	GEN_LONG_HS(ls04x_tpc_sync_ctrl),
	GEN_LONG_HS(ls04x_source_timing_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gip_timing_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gamma_a_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gamma_b_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gamma_c_setting_igzo_g8),
	GEN_LONG_HS(ls04x_backlight_setting4_igzo_g8),
	GEN_LONG_HS(ls04x_power_setting_igzo_g8),
	GEN_LONG_HS(ls04x_power_setting_switching_reg_igzo_g8),
	GEN_LONG_HS(ls04x_power_setting_int_pwr2_igzo_g8),
	GEN_LONG_HS(ls04x_vcom_setting_igzo_g8),
	GEN_SHORT_HS(ls04x_sleep_out_nvm_load_setting),
	GEN_LONG_HS(ls04x_seq_timing_ctrl_gip_igzo_g8),
	GEN_LONG_HS(ls04x_lowpower_function1),
	GEN_LONG_HS(ls04x_panel_sync_out1_igzo_g8),
	GEN_LONG_HS(ls04x_panel_sync_out2_igzo_g8),
	MCS_SHORT_HS(ls04x_ledpwm_on),
	MCS_SHORT_HS(ls04x_cabc_on),
	MCS_LONG_HS(ls04x_set_column_addr),
	MCS_LONG_HS(ls04x_set_page_addr),
	DONE()
};

static struct dsi_init_command programed_igzo_g8_init_commands[] = {
	GEN_SHORT_HS(ls04x_mcap),
	GEN_LONG_HS(ls04x_frame_memory_access_igzo_g8),
	GEN_LONG_HS(ls04x_interface_id_igzo_g8),
	GEN_LONG_HS(ls04x_dsi_control1_igzo_g8),
	GEN_LONG_HS(ls04x_external_clock_igzo_g8),
	GEN_LONG_HS(ls04x_panel_pin_ctrl_igzo_g8),
	GEN_SHORT_HS(ls04x_panel_interface_ctrl_igzo_g8),
	GEN_LONG_HS(ls04x_display_setting1_igzo_g8),
	GEN_LONG_HS(ls04x_display_setting2_igzo_g8),
	GEN_LONG_HS(ls04x_tpc_sync_ctrl),
	GEN_LONG_HS(ls04x_source_timing_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gip_timing_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gamma_a_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gamma_b_setting_igzo_g8),
	GEN_LONG_HS(ls04x_gamma_c_setting_igzo_g8),
	GEN_LONG_HS(ls04x_backlight_setting4_igzo_g8),
	GEN_LONG_HS(ls04x_power_setting_igzo_g8),
	GEN_LONG_HS(ls04x_power_setting_switching_reg_igzo_g8),
	GEN_LONG_HS(ls04x_power_setting_int_pwr2_igzo_g8),
	GEN_SHORT_HS(ls04x_sleep_out_nvm_load_setting),
	GEN_LONG_HS(ls04x_seq_timing_ctrl_gip_igzo_g8),
	GEN_LONG_HS(ls04x_lowpower_function1),
	GEN_LONG_HS(ls04x_panel_sync_out1_igzo_g8),
	GEN_LONG_HS(ls04x_panel_sync_out2_igzo_g8),
	MCS_SHORT_HS(ls04x_ledpwm_on),
	MCS_SHORT_HS(ls04x_cabc_on),
	MCS_LONG_HS(ls04x_set_column_addr),
	MCS_LONG_HS(ls04x_set_page_addr),
	DONE()
};

static struct dsi_init_command cgs_init_commands[] = {
	GEN_SHORT_HS(ls04x_mcap_cgs),
	GEN_LONG_HS(ls04x_frame_memory_access_cgs),
	GEN_LONG_HS(ls04x_interface_id),
	GEN_LONG_HS(ls04x_dsi_control1_cgs),
	GEN_SHORT_HS(ls04x_panel_interface_ctrl_cgs),
	GEN_LONG_HS(ls04x_display_setting1_cgs),
	GEN_LONG_HS(ls04x_display_setting2_cgs),
	GEN_LONG_HS(ls04x_tpc_sync_ctrl),
	GEN_LONG_HS(ls04x_source_timing_setting_cgs),
	GEN_LONG_HS(ls04x_ltps_timing_setting_cgs),
	GEN_LONG_HS(ls04x_gamma_a_setting_cgs),
	GEN_LONG_HS(ls04x_gamma_b_setting_cgs),
	GEN_LONG_HS(ls04x_gamma_c_setting_cgs),
	GEN_LONG_HS(ls04x_power_setting_cgs),
	GEN_SHORT_HS(ls04x_power_setting_int_pwr1_cgs),
	GEN_LONG_HS(ls04x_power_setting_int_pwr2_cgs),
	GEN_LONG_HS(ls04x_vcom_setting),
	GEN_SHORT_HS(ls04x_sleep_out_nvm_load_setting),
	GEN_LONG_HS(ls04x_sequencer_timing_power_on_cgs),
	MCS_SHORT_HS(ls04x_ledpwm_on),
	MCS_SHORT_HS(ls04x_cabc_on),
	MCS_LONG_HS(ls04x_set_column_addr),
	MCS_LONG_HS(ls04x_set_page_addr),
	DONE()
};

static u8 ir2e69_power_on_seq_cabc[][2] = {
	{0x03, 0x01},
	{0,      20},
	{0x27, 0xe8},
	{0x03, 0x83},
	{0,      20},
	{0x28, 0x40},
	{0x2b, 0x01},
	{0x05, 0x0d},
	{0x06, 0x01},
	{0x25, 0x20},
	{0x0a, 0xc8},
	{0x0b, 0xc8},
	{0xdc, 0x3b},
	{0xee, 0x00},
	{0xf1, 0x00},
	{0xda, 0x30},
	{0xd8, 0x00} };
static u8 ir2e69_power_on_seq[][2] = {
	{0x03, 0x01},
	{0,      20},
	{0x27, 0xe8},
	{0x03, 0x83},
	{0,      20},
	{0x28, 0x40},
	{0x2b, 0x01},
	{0x05, 0x09},
	{0x06, 0x01},
	{0x25, 0x20},
	{0x0a, 0x00},
	{0x0b, 0x00},
	{0xdc, 0x3b},
	{0xee, 0x00},
	{0xf1, 0x00},
	{0xda, 0x30},
	{0xd8, 0x00} };
static u8 ir2e69_bias_on_seq[][2] = {
	{0x28, 0x40},
	{0,       1},
	{0x2b, 0x01},
	{0,       1} };
static u8 ir2e69_bias_off_seq[][2] = {
	{0x2b, 0x00},
	{0,       1},
	{0x28, 0x00},
	{0,       1} };
static u8 ir2e69_standby_seq[][2] = {
	{0x03, 0x00} };
static u8 ir2e69_normal_seq[][2] = {
	{0x03, 0x83} };
static u8 ls04x_reset_low[] = {0xd8, 0x00};
static u8 ls04x_reset_high[] = {0xd8, 0x10};

static struct i2c_board_info dcdc_board_info = {
	I2C_BOARD_INFO("vb_cmd_ir2e69", I2C_ADDRESS)
};

static struct i2c_client *i2c_client;

static int mipi_reset_gpio = -1;

static int pwm_width;

static int handle_dsi_init_commands(struct mdfld_dsi_pkg_sender *sender,
				struct dsi_init_command *command)
{
	int r = 0;

	if (!sender)
		return -EINVAL;

	while (command->type != DONE_CMD && r == 0) {
		switch (command->type) {
		case GEN_LONG_LP_CMD:
			r = mdfld_dsi_send_gen_long_lp(sender, command->param,
				command->count, MDFLD_DSI_SEND_PACKAGE);
			break;
		case GEN_SHORT_LP_CMD:
			r = mdfld_dsi_send_gen_short_lp(sender,
				command->param[0],
				command->count > 1 ? command->param[1] : 0,
				command->count, MDFLD_DSI_SEND_PACKAGE);
			break;
		case MCS_LONG_LP_CMD:
			r = mdfld_dsi_send_mcs_long_lp(sender, command->param,
				command->count, MDFLD_DSI_SEND_PACKAGE);
			break;
		case MCS_SHORT_LP_CMD:
			r = mdfld_dsi_send_mcs_short_lp(sender,
				command->param[0],
				command->count > 1 ? command->param[1] : 0,
				command->count - 1, MDFLD_DSI_SEND_PACKAGE);
			break;
		case GEN_LONG_HS_CMD:
			r = mdfld_dsi_send_gen_long_hs(sender, command->param,
				command->count, MDFLD_DSI_SEND_PACKAGE);
			break;
		case GEN_SHORT_HS_CMD:
			r = mdfld_dsi_send_gen_short_hs(sender,
				command->param[0],
				command->count > 1 ? command->param[1] : 0,
				command->count, MDFLD_DSI_SEND_PACKAGE);
			break;
		case MCS_LONG_HS_CMD:
			r = mdfld_dsi_send_mcs_long_hs(sender, command->param,
				command->count, MDFLD_DSI_SEND_PACKAGE);
			break;
		case MCS_SHORT_HS_CMD:
			r = mdfld_dsi_send_mcs_short_hs(sender,
				command->param[0],
				command->count > 1 ? command->param[1] : 0,
				command->count - 1, MDFLD_DSI_SEND_PACKAGE);
			break;
		case DELAY_CMD:
			usleep_range(command->count, command->count * 3 / 2);
			break;
		case DONE_CMD:
			break;
		}
		command++;
	}
	return r;
}

static void ir2e69_set_gpio(int value)
{
	PSB_DEBUG_ENTRY(": %d\n", value);
	if (mipi_reset_gpio == -1) {
		DRM_ERROR("mipi_reset_gpio is not correctly set\n");
		return;
	}

	gpio_set_value(mipi_reset_gpio, value);
}

static void ir2e69_register(void)
{
	struct i2c_adapter *adapter;
	int ret;

	adapter = i2c_get_adapter(I2C_ADAPTER);
	i2c_client = i2c_new_device(adapter, &dcdc_board_info);

	mipi_reset_gpio = get_gpio_by_name("mipi-reset");
	if (mipi_reset_gpio < 0) {
		DRM_ERROR("can't get mipi-reset gpio pin\n");
		mipi_reset_gpio = -1;
		return;
	}
	ret = gpio_request(mipi_reset_gpio, "mipi_display");
	if (ret < 0) {
		DRM_ERROR("failed to request gpio %d\n", mipi_reset_gpio);
		mipi_reset_gpio = -1;
		return;
	}
	gpio_direction_output(mipi_reset_gpio, 0);
}

static int ir2e69_send_sequence(u8 data[][2], int count)
{
	int r = 0;
	int i;

	for (i = 0; i < count && r >= 0; i++) {
		if (data[i][0] != 0) {
			r = i2c_master_send(i2c_client,
					data[i], sizeof(data[i]));
			if (r < 0)
				dev_err(&i2c_client->dev, "%d: error %d\n",
					i, r);
		} else
			usleep_range(data[i][1], data[i][1] * 3 / 2);
	}
	return r;
}

static void ir2e69_reset(void)
{
	PSB_DEBUG_ENTRY("\n");

	ir2e69_set_gpio(0);
	mdelay(20);
	ir2e69_set_gpio(1);
	mdelay(20);
	if (drm_psb_enable_cabc)
		ir2e69_send_sequence(ir2e69_power_on_seq_cabc,
				ARRAY_SIZE(ir2e69_power_on_seq_cabc));
	else
		ir2e69_send_sequence(ir2e69_power_on_seq,
				ARRAY_SIZE(ir2e69_power_on_seq));
}

static void ir2e69_power_off(void)
{
	PSB_DEBUG_ENTRY("\n");
	ir2e69_send_sequence(ir2e69_bias_off_seq,
			ARRAY_SIZE(ir2e69_bias_off_seq));
	if (ir2e69_send_sequence(ir2e69_standby_seq,
				 ARRAY_SIZE(ir2e69_standby_seq)) < 0)
		mdelay(200);
	else
		mdelay(20);
	ir2e69_set_gpio(0);
}

static void ir2e69_power_on(void)
{
	PSB_DEBUG_ENTRY("\n");
	ir2e69_send_sequence(ir2e69_normal_seq,
			ARRAY_SIZE(ir2e69_normal_seq));
	ir2e69_send_sequence(ir2e69_bias_on_seq,
			ARRAY_SIZE(ir2e69_bias_on_seq));
}

static void ls04x_reset(void)
{
	PSB_DEBUG_ENTRY("\n");
	i2c_master_send(i2c_client, ls04x_reset_low, sizeof(ls04x_reset_low));
	mdelay(10);
	i2c_master_send(i2c_client, ls04x_reset_high, sizeof(ls04x_reset_high));
	mdelay(20);
}

static int ls04x_igzo_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	DRM_INFO("IGZO panel detected!\n");
	pwm_width = 12;
	return handle_dsi_init_commands(mdfld_dsi_get_pkg_sender(dsi_config),
						igzo_init_commands);
}

#define DEFAULT_VCOM_SETTING		0x62
static int ls04x_igzo_g8_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	DRM_INFO("IGZO G8 panel detected!\n");
	pwm_width = 8;
	u8 data[16];
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);
	mdfld_dsi_read_gen_hs(sender,
			ls04x_vcom_setting_igzo_g8[0], 0, 1, data, 11);
	if (DEFAULT_VCOM_SETTING == data[4])
		return handle_dsi_init_commands(
					mdfld_dsi_get_pkg_sender(dsi_config),
					igzo_g8_init_commands);
	else
		return handle_dsi_init_commands(
					mdfld_dsi_get_pkg_sender(dsi_config),
					programed_igzo_g8_init_commands);
}

static int ls04x_cgs_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	DRM_INFO("CGS panel detected!\n");
	pwm_width = 8;
	return handle_dsi_init_commands(mdfld_dsi_get_pkg_sender(dsi_config),
						cgs_init_commands);
}

static int ls04x_drv_ic_init(struct mdfld_dsi_config *dsi_config)
{
	int r = 0;
	u8 data[16];
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);

	PSB_DEBUG_ENTRY("\n");

	if (!sender)
		return -EINVAL;
	sender->status = MDFLD_DSI_PKG_SENDER_FREE;
	memset(data, 0, sizeof(data));
	mdfld_dsi_send_gen_short_hs(sender,
			ls04x_mcap[0],
			ls04x_mcap[1], 2,
			MDFLD_DSI_SEND_PACKAGE);

	r = mdfld_dsi_read_gen_hs(sender,
			ls04x_device_code_read[0], 0, 1, data, 5);
	PSB_DEBUG_GENERAL("device code read: %d %02x %02x %02x %02x %02x\n",
			  r, data[0], data[1], data[2], data[3], data[4]);

	if ((data[2] == 0x14) && (data[3] == 0x13))
		r = ls04x_igzo_drv_ic_init(dsi_config);
	else if ((data[2] == 0x34) && (data[3] == 0x15))
		r = ls04x_cgs_drv_ic_init(dsi_config);
	else if ((data[2] == 0x94) && (data[3] == 0x31))
		r = ls04x_igzo_g8_drv_ic_init(dsi_config);
	else
		DRM_INFO("unknown device code: %02x %02x\n", data[2], data[3]);

	return r;
}

static void ls04x_dsi_controller_init(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_hw_context *hw_ctx = &dsi_config->dsi_hw_context;

	PSB_DEBUG_ENTRY("\n");

	dsi_config->lane_count = 4;
	dsi_config->lane_config = MDFLD_DSI_DATA_LANE_4_0;

	hw_ctx->cck_div = 1;
	hw_ctx->pll_bypass_mode = 0;

	hw_ctx->mipi_control = 0x00;
	hw_ctx->intr_en = 0xffffffff;
	hw_ctx->hs_tx_timeout = 0xffffff;
	hw_ctx->lp_rx_timeout = 0xffffff;
	hw_ctx->turn_around_timeout = 0x18;
	hw_ctx->device_reset_timer = 0xffff;
	hw_ctx->high_low_switch_count = 0x1b;
	hw_ctx->init_count = 0xf0;
	hw_ctx->eot_disable = 0x2;
	hw_ctx->lp_byteclk = 0x4;
	hw_ctx->clk_lane_switch_time_cnt = 0x1b000c;
	hw_ctx->dbi_bw_ctrl = 820;
	hw_ctx->dphy_param = 0x1b104315;
	hw_ctx->dsi_func_prg = (0xa000 | dsi_config->lane_count);
	hw_ctx->mipi = TE_TRIGGER_GPIO_PIN | PASS_FROM_SPHY_TO_AFE;
	hw_ctx->mipi |= dsi_config->lane_config;
	hw_ctx->vgacntr = 0x80000000;
}

static struct drm_display_mode *ls04x_cmd_get_config_mode(void)
{
	struct drm_display_mode *mode;

	PSB_DEBUG_ENTRY("\n");

	mode = kzalloc(sizeof(*mode), GFP_KERNEL);
	if (!mode)
		return NULL;

	mode->htotal = 920;
	mode->hdisplay = 720;
	mode->hsync_start = 816;
	mode->hsync_end = 824;
	mode->vtotal = 1550;
	mode->vdisplay = 1280;
	mode->vsync_start = 1294;
	mode->vsync_end = 1296;
	mode->vrefresh = 60;
	mode->clock =  mode->vrefresh * mode->vtotal * mode->htotal / 1000;
	mode->type |= DRM_MODE_TYPE_PREFERRED;

	drm_mode_set_name(mode);
	drm_mode_set_crtcinfo(mode, 0);

	return mode;
}

static void ls04x_cmd_get_panel_info(int pipe, struct panel_info *pi)
{
	PSB_DEBUG_ENTRY("\n");
	if (pipe == 0) {
		pi->width_mm = PANEL_4DOT3_WIDTH;
		pi->height_mm = PANEL_4DOT3_HEIGHT;
	}
}

static int ls04x_cgs_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	if (drm_psb_enable_cabc) {
		struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
		if (!sender)
			return -EINVAL;

		u8 brightness[3] = {0, };

		brightness[0] = write_display_brightness;
		brightness[1] = 0;
		brightness[2] = (level * 255 / 100) & 0xff;
		mdfld_dsi_send_mcs_long_hs(sender,
				brightness,
				sizeof(brightness),
				MDFLD_DSI_SEND_PACKAGE);
	} else {
		u8 brightness[2];

		PSB_DEBUG_ENTRY("%d\n", level);
		brightness[0] = 0x0a;
		brightness[1] = level * 255 / 100;
		i2c_master_send(i2c_client, brightness, sizeof(brightness));
		brightness[0] = 0x0b;
		i2c_master_send(i2c_client, brightness, sizeof(brightness));
	}

	return 0;
}

static int ls04x_cmd_set_brightness(struct mdfld_dsi_config *dsi_config,
					int level)
{
	u8 brightness[3];
	if (drm_psb_enable_cabc) {
		struct mdfld_dsi_pkg_sender *sender
			= mdfld_dsi_get_pkg_sender(dsi_config);
		if (!sender)
			return -EINVAL;

		brightness[0] = write_display_brightness;
		if (pwm_width == 12) {
			brightness[1] = (level * 4095 / 100) >> 8;
			brightness[2] = (level * 4095 / 100) & 0xff;
		} else {
			brightness[1] = (level * 255 / 100);
		}
		mdfld_dsi_send_mcs_long_lp(sender,
				brightness,
				pwm_width == 12 ? 3 : 2,
				MDFLD_DSI_SEND_PACKAGE);
	} else {
		brightness[0] = 0x0a;
		brightness[1] = level * 200 / 100;
		i2c_master_send(i2c_client, brightness, 2);
		brightness[0] = 0x0b;
		i2c_master_send(i2c_client, brightness, 2);
	}

	return 0;
}

static int vb_cmd_reset(struct mdfld_dsi_config *dsi_config)
{
	PSB_DEBUG_ENTRY("\n");
	mdelay(10);
	ir2e69_reset();
	ls04x_reset();
	mdelay(3);
	return 0;
}

static int vb_cmd_power_on(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender)
		return -EINVAL;

	ir2e69_power_on();
	mdfld_dsi_send_mcs_short_hs(sender, set_tear_on, 0x00, 1,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, set_display_on, 0, 0,
				    MDFLD_DSI_SEND_PACKAGE);
	mdfld_dsi_send_mcs_short_hs(sender, exit_sleep_mode, 0, 0,
				    MDFLD_DSI_SEND_PACKAGE);
	mdelay(100);

	if (drm_psb_enable_cabc) {
		mdfld_dsi_send_mcs_short_hs(sender,
					    ls04x_ledpwm_on[0],
					    ls04x_ledpwm_on[1], 1,
					    MDFLD_DSI_SEND_PACKAGE);
		mdfld_dsi_send_mcs_short_hs(sender,
					    ls04x_cabc_on[0],
					    ls04x_cabc_on[1], 1,
					    MDFLD_DSI_SEND_PACKAGE);
	}

	return 0;
}

static int vb_cmd_power_off(struct mdfld_dsi_config *dsi_config)
{
	struct mdfld_dsi_pkg_sender *sender
				= mdfld_dsi_get_pkg_sender(dsi_config);
	if (!sender)
		return -EINVAL;

	PSB_DEBUG_ENTRY("\n");
	mdfld_dsi_send_mcs_short_hs(sender, set_display_off, 0, 0,
					MDFLD_DSI_SEND_PACKAGE);
	mdelay(16);

	mdfld_dsi_send_gen_long_hs(sender, ls04x_vcom_setting_igzo_g8_poweroff,
					sizeof(ls04x_vcom_setting_igzo_g8),
					MDFLD_DSI_SEND_PACKAGE);
	mdelay(16);

	mdfld_dsi_send_mcs_short_hs(sender, enter_sleep_mode, 0, 0,
					MDFLD_DSI_SEND_PACKAGE);
	mdelay(180);

	mdfld_dsi_send_gen_short_hs(sender,
			ls04x_deep_standby[0],
			ls04x_deep_standby[1],
			2, MDFLD_DSI_SEND_PACKAGE);
	mdelay(16);

	ir2e69_power_off();
	mdelay(15);

	i2c_master_send(i2c_client, ls04x_reset_low, sizeof(ls04x_reset_low));
	mdelay(10);

	return 0;
}

static int vb_cmd_detect(struct mdfld_dsi_config *dsi_config)
{
	int status = MDFLD_DSI_PANEL_DISCONNECTED;
#if 1
	struct drm_device *dev = dsi_config->dev;
	struct mdfld_dsi_hw_registers *regs = &dsi_config->regs;
	u32 dpll_val, device_ready_val;

	PSB_DEBUG_ENTRY("\n");
	dpll_val = REG_READ(regs->dpll_reg);
	device_ready_val = REG_READ(regs->device_ready_reg);
	if ((device_ready_val & DSI_DEVICE_READY) &&
	    (dpll_val & DPLL_VCO_ENABLE)) {
		dsi_config->dsi_hw_context.panel_on = true;
	} else {
		dsi_config->dsi_hw_context.panel_on = false;
		DRM_INFO("%s: panel is not detected!\n", __func__);
	}

	dsi_config->dsi_hw_context.panel_on = false;
	status = MDFLD_DSI_PANEL_CONNECTED;
#else
	u8 power = 0;

	PSB_DEBUG_ENTRY("\n");
	if (mdfld_dsi_get_power_mode(dsi_config, &power,
				     MDFLD_DSI_LP_TRANSMISSION) > 0) {
		if (power & 0x04)
			dsi_config->dsi_hw_context.panel_on = true;
		else
			dsi_config->dsi_hw_context.panel_on = false;
		status = MDFLD_DSI_PANEL_CONNECTED;
	}
	PSB_DEBUG_GENERAL(": %d %x\n", status, power);
	PSB_DEBUG_GENERAL("display connection state: %d, power: %x\n",
			  status,
			  power);
#endif

	return status;

}

static int vb_cmd_reboot(struct notifier_block *this, unsigned long code,
			 void *unused)
{
	ir2e69_power_off();
	return NOTIFY_DONE;
}

struct notifier_block vb_cmd_reboot_notifier_block = {
	.notifier_call = vb_cmd_reboot
};

void vb_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	p_funcs->get_config_mode = ls04x_cmd_get_config_mode;
	p_funcs->get_panel_info = ls04x_cmd_get_panel_info;
	p_funcs->reset = vb_cmd_reset;
	p_funcs->dsi_controller_init = ls04x_dsi_controller_init;
	p_funcs->detect = vb_cmd_detect;
	p_funcs->power_on = vb_cmd_power_on;
	p_funcs->power_off = vb_cmd_power_off;
	p_funcs->set_brightness = ls04x_cmd_set_brightness;
	ir2e69_register();
	register_reboot_notifier(&vb_cmd_reboot_notifier_block);
}

void vb_igzo_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	vb_cmd_init(dev, p_funcs);
	p_funcs->drv_ic_init = ls04x_drv_ic_init;
	p_funcs->set_brightness = ls04x_cmd_set_brightness;
}

void vb_igzo_g8_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	//FIXME,instability issue casued by CABC feature,fix it later
	drm_psb_enable_cabc = 0;

	vb_cmd_init(dev, p_funcs);
	p_funcs->drv_ic_init = ls04x_drv_ic_init;
	p_funcs->set_brightness = ls04x_cmd_set_brightness;
}

void vb_cgs_cmd_init(struct drm_device *dev, struct panel_funcs *p_funcs)
{
	PSB_DEBUG_ENTRY("\n");

	vb_cmd_init(dev, p_funcs);
	p_funcs->drv_ic_init = ls04x_cgs_drv_ic_init;
	p_funcs->set_brightness = ls04x_cgs_cmd_set_brightness;
}

static int vb_lcd_probe(struct platform_device *pdev)
{
	int ret = 0;
	enum vb_panel_type panel_type;

	DRM_INFO("%s\n", __func__);
	panel_type = platform_get_device_id(pdev)->driver_data;
	if (panel_type == PANEL_IGZO) {
		DRM_INFO("%s: IGZO panel detected\n", __func__);
		intel_mid_panel_register(vb_igzo_cmd_init);
	} else if (panel_type == PANEL_CGS) {
		DRM_INFO("%s: CGS panel detected\n", __func__);
		intel_mid_panel_register(vb_cgs_cmd_init);
	} else if (panel_type == PANEL_IGZO_G8) {
		DRM_INFO("%s: IGZO G8 panel detected\n", __func__);
		intel_mid_panel_register(vb_igzo_g8_cmd_init);
	} else {
		DRM_ERROR("bad vb panel type %d\n", panel_type);
		return -EINVAL;
	}

	return ret;
}

static struct platform_device_id vb_panel_tbl[] = {
	{ "SHARP IGZO VKB", PANEL_IGZO },
	{ "SHARP CGS VKB",  PANEL_CGS },
	{ "SHARP IGZOG8 VKB",  PANEL_IGZO_G8 },
	{ }
};

struct platform_driver vb_lcd_driver = {
	.probe	= vb_lcd_probe,
	.driver	= {
		.name	= "vb_lcd",
		.owner	= THIS_MODULE,
	},
	.id_table = vb_panel_tbl
};
