/*
 * Summit Microelectronics SMB1357 Battery Charger Driver
 *
 * Copyright (C) 2011, Intel Corporation
 *
 * Authors: Bruce E. Robertson <bruce.e.robertson@intel.com>
 *          Mika Westerberg <mika.westerberg@linux.intel.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#ifndef SMB1357_CHARGER_H
#define SMB1357_CHARGER_H

#include <linux/types.h>
#include <linux/power_supply.h>


/**
 * struct smb1357_charger_platform_data - platform data for SMB1357 charger
 * @battery_info: Information about the battery
 * @max_charge_current: maximum current (in uA) the battery can be charged
 * @max_charge_voltage: maximum voltage (in uV) the battery can be charged
 * @pre_charge_current: current (in uA) to use in pre-charging phase
 * @termination_current: current (in uA) used to determine when the
 *			 charging cycle terminates
 * @pre_to_fast_voltage: voltage (in uV) treshold used for transitioning to
 *			 pre-charge to fast charge mode
 * @mains_current_limit: maximum input current drawn from AC/DC input (in uA)
 * @usb_hc_current_limit: maximum input high current (in uA) drawn from USB
 *			  input
 * @otg_uvlo_voltage: voltage threshold (in uV) when reached the OTG VBUS
 *		      is disabled
 * @chip_temp_threshold: die temperature where device starts limiting charge
 *			 current [%100 - %130] (in degree C)
 * @soft_cold_temp_limit: soft cold temperature limit [%0 - %15] (in degree C),
 *			  granularity is 5 deg C.
 * @soft_hot_temp_limit: soft hot temperature limit [%40 - %55] (in degree  C),
 *			 granularity is 5 deg C.
 * @hard_cold_temp_limit: hard cold temperature limit [%-5 - %10] (in degree C),
 *			  granularity is 5 deg C.
 * @hard_hot_temp_limit: hard hot temperature limit [%50 - %65] (in degree C),
 *			 granularity is 5 deg C.
 * @suspend_on_hard_temp_limit: suspend charging when hard limit is hit
 * @soft_temp_limit_compensation: compensation method when soft temperature
 *				  limit is hit
 * @charge_current_compensation: current (in uA) for charging compensation
 *				 current when temperature hits soft limits
 * @use_mains: AC/DC input can be used
 * @use_usb: USB input can be used
 * @irq_gpio: GPIO number used for interrupts (%-1 if not used)
 * @enable_control: how charging enable/disable is controlled
 *		    (driver/pin controls)
 * @otg_control: how OTG VBUS is controlled
 *
 * @use_main, @use_usb, and @otg_control are means to enable/disable
 * hardware support for these. This is useful when we want to have for
 * example OTG charging controlled via OTG transceiver driver and not by
 * the SMB1357 hardware.
 *
 * Hard and soft temperature limit values are given as described in the
 * device data sheet and assuming NTC beta value is %3750. Even if this is
 * not the case, these values should be used. They can be mapped to the
 * corresponding NTC beta values with the help of table %2 in the data
 * sheet. So for example if NTC beta is %3375 and we want to program hard
 * hot limit to be %53 deg C, @hard_hot_temp_limit should be set to %50.
 *
 * If zero value is given in any of the current and voltage values, the
 * factory programmed default will be used. For soft/hard temperature
 * values, pass in %SMB1357_TEMP_USE_DEFAULT instead.
 */
struct smb1357_charger_platform_data {
	bool		use_mains;
	bool		use_usb;
	int		inok_gpio;
};

int smb1357_get_charging_status(void);

#endif /* SMB1357_CHARGER_H */
