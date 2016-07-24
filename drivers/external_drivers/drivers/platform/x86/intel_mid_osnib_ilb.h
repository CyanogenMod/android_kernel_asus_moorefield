/*
 * include/linux/cmos_osnib_ilb.h
 *
 * Copyright (C) 2013 Intel Corp
 * Author: Asutosh Pathak <asutosh.pathak@intel.com>
 * Author: Vincent Tinelli (vincent.tinelli@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program; if not, write to the Free Software Foundation, Inc.,
 * 59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 */

#ifndef __CMOS_OSNIB_ILB_H
#define __CMOS_OSNIB_ILB_H

#include <linux/kobject.h>
#include <linux/sysfs.h>
#include <linux/err.h>
#include <linux/platform_device.h>
#include <linux/module.h>
#include <linux/mc146818rtc.h>
#include <linux/reboot.h>
#include <asm/intel-mid.h>

#define OSNIB_SIZE 64
#define OSNIB_OEM_RSVD_SIZE 32
#define OSNIB_INTEL_SIZE (OSNIB_SIZE - OSNIB_OEM_RSVD_SIZE)
#define OSNIB_DEBUG_SIZE 14
#define OSNIB_FW_RSVD_SIZE 3
#define OSNIB_CMOS_BASE_ADDR 0x0E
#define OSNIB_FW_UPDATE_BIT 2

#define OSNIB_FW_UPDATE_SET_VALUE(a) \
	((u8) ((a & 0x1) << (OSNIB_FW_UPDATE_BIT - 1)))

#define OSNIB_FW_UPDATE_GET_VALUE(a) \
	((u8) (a >> (OSNIB_FW_UPDATE_BIT - 1)))


enum intel_mid_bootflow_type {
	END_USER,
	MANUFACTURING,
	GPP_CERTIFICATION
};

struct bootflow_type {
	u8 id;
	const char *name;
};

enum intel_mid_wake_src {
	WAKE_BATT_INSERT,
	WAKE_PWR_BUTTON_PRESS,
	WAKE_RTC_TIMER,
	WAKE_USB_CHRG_INSERT,
	WAKE_RESERVED,
	WAKE_REAL_RESET,
	WAKE_PLATFORM_RESET,
	WAKE_UNKNOWN,
	WAKE_KERNEL_WATCHDOG_RESET,
	WAKE_SECURITY_WATCHDOG_RESET,
	WAKE_WATCHDOG_COUNTER_EXCEEDED,
	WAKE_POWER_SUPPLY_DETECTED,
	WAKE_FASTBOOT_BUTTONS_COMBO,
	WAKE_NO_MATCHING_OSIP_ENTRY,
	WAKE_CRITICAL_BATTERY,
	WAKE_INVALID_CHECKSUM,
	WAKE_FORCED_RESET,
	WAKE_ACDC_CHRG_INSERT
};

struct wake_src {
	u8 id;
	const char *name;
};

enum intel_mid_target_os {
	MAIN = 0x00,
	CHARGING = 0x0A,
	RECOVERY = 0x0C,
	FASTBOOT = 0x0E,
	FACTORY = 0x12,
	DNX = 0x14,
	RAMCONSOLE = 0x16,
	RESERVED_INTEL_BEGIN = 0x0F,
	RESERVED_INTEL_END = 0xEF,
	RESERVED_OEM_BEGIN = 0xF0,
	RESERVED_OEM_END = 0xFF
};

struct target_os {
	const char *name;
	u32 id;
};

struct cmos_osnib {

	struct {
		u8 magic[4];
		u8 version_major;
		u8 version_minor;
		u8 reserved[2];
	} __packed header;

	struct {
		u8 type;
	} __packed bootflow;

	struct {
		struct {
			u8 kernel_watchdog:1;
			u8 security_watchdog:1;
			u8 pmc_watchdog:1;
			u8 reserved:1;
			u8 wdt_counter:4;
		} __packed bf;
		u8 wake_src;
		u8 debug[OSNIB_DEBUG_SIZE];
		u8 fw_update_status;
	} __packed fw_to_os;

	u8 firmware_reserved[OSNIB_FW_RSVD_SIZE];

	struct {
		u8 target_mode;
		struct {
			u8 rtc_alarm_charger:1;
			u8 fw_update:1;
			u8 ramconsole:1;
			u8 reserved:5;
		} bf;
	} __packed os_to_fw;

	u8 checksum;

	struct {
		u8 reserved[OSNIB_OEM_RSVD_SIZE];
	} __packed oem;
} __packed;

/**
 * intel_mid_ilb_write_osnib_field() - write osnib field to cmos
 * @osnib: osnib buffer
 * @offset: field offset in osnib buffer
 * @value: field value to be written
 *
 * Write osnib field to cmos without checksum
 */
void intel_mid_ilb_write_osnib_field(struct cmos_osnib *osnib,
		int offset, u8 value);

/**
 * intel_mid_ilb_write_osnib() - write osnib content to cmos
 * @osnib: osnib buffer
 *
 * Write osnib content to cmos with checksum
 */
void intel_mid_ilb_write_osnib(struct cmos_osnib *osnib);

/**
 * intel_mid_ilb_write_osnib_checksum() - write osnib checksum to cmos
 * @osnib: osnib buffer
 *
 * Write osnib checksum to cmos
 */
void intel_mid_ilb_write_osnib_checksum(struct cmos_osnib *osnib);

/**
 * intel_mid_ilb_read_osnib_field() - read osnib field from cmos
 * @osnib: osnib buffer
 * @offset: field offset in osnib buffer
 *
 * Read osnib field to cmos without checksum
 */
u8 intel_mid_ilb_read_osnib_field(struct cmos_osnib *osnib,
		int offset);

/**
 * intel_mid_ilb_read_osnib() - read osnib content from cmos
 * @osnib: osnib buffer
 *
 * Write osnib content to cmos with checksum
 */
int intel_mid_ilb_read_osnib(struct cmos_osnib *osnib);

/**
 * intel_mid_ilb_dump_osnib() - dump osnib content to cmos
 * @osnib: osnib buffer
 *
 * Dump osnib buffer to logs
 */
void intel_mid_ilb_dump_osnib(struct cmos_osnib *osnib);

/**
 * intel_mid_ilb_reset_osnib() - reset osnib content to 0
 * @osnib: osnib buffer
 *
 * Reset all osnib content to 0. Checksum algorithm is 2's complement, force
 * checksum to 1 to ignore content and use default flow.
 */
void intel_mid_ilb_reset_osnib(struct cmos_osnib *osnib);

/**
 * intel_mid_ilb_checksum_osnib() - returns osnib's content
 * @osnib: osnib buffer
 *
 * Use 2's complement on osnib buffer to compute checksum
 */
u8 intel_mid_ilb_checksum_osnib(struct cmos_osnib *osnib);

/**
 * intel_mid_ilb_is_osnib_valid() - check if osnib content is valid
 * @osnib: osnib buffer
 *
 */
int intel_mid_ilb_is_osnib_valid(struct cmos_osnib *osnib);

/**
 * intel_mid_ilb_write_osnib_rr() - write reboot reason to osnib
 * @rr: target os
 *
 */
int intel_mid_ilb_write_osnib_rr(const char *target, int id);

/**
 * intel_mid_ilb_read_osnib_rr() - read reboot reason from osnib
 *
 */
const char *intel_mid_ilb_read_osnib_rr(void);



#endif
