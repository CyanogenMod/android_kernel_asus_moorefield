/*
 * SMB345 Battery Charger Driver
 */

#ifndef SMB345_CHARGER_H
#define SMB345_CHARGER_H

#include <linux/types.h>
#include <linux/power_supply.h>

enum {
	/* use the default compensation method */
	SMB347_SOFT_TEMP_COMPENSATE_DEFAULT = -1,

	SMB347_SOFT_TEMP_COMPENSATE_NONE,
	SMB347_SOFT_TEMP_COMPENSATE_CURRENT,
	SMB347_SOFT_TEMP_COMPENSATE_VOLTAGE,
};

/* Use default factory programmed value for hard/soft temperature limit */
#define SMB347_TEMP_USE_DEFAULT		-273

/*
 * Charging enable can be controlled by software (via i2c) by
 * smb347-charger driver or by EN pin (active low/high).
 */
enum smb347_chg_enable {
	SMB347_CHG_ENABLE_SW,
	SMB347_CHG_ENABLE_PIN_ACTIVE_LOW,
	SMB347_CHG_ENABLE_PIN_ACTIVE_HIGH,
};

/*
 * Driving VBUS can be controlled by software (via i2c) by pin or by
 * hardware ID pin autodetection. If set to %SMB347_OTG_CONTROL_DISABLED
 * the functionality is disabled.
 *
 * %SMB347_OTG_CONTROL_DISABLED - don't use OTG at all
 * %SMB347_OTG_CONTROL_PIN - use ID pin to detect when VBUS should be
 *			     driven and raise VBUS automatically
 * %SMB347_OTG_CONTROL_AUTO - Use auto-OTG and RID detection algorithm
 * %SMB347_OTG_CONTROL_SW - enable OTG VBUS via register when we receive an
 *			    OTG event from transceiver driver
 * %SMB347_OTG_CONTROL_SW_PIN - enable OTG VBUS by switching to pin control
 *				mode when OTG event is received
 * %SMB347_OTG_CONTROL_SW_AUTO - enable OTG VBUS by switching to auto-OTG
 *				 mode when OTG event is received
 */
enum smb347_otg_control {
	SMB347_OTG_CONTROL_DISABLED,
	SMB347_OTG_CONTROL_PIN,
	SMB347_OTG_CONTROL_AUTO,
	SMB347_OTG_CONTROL_SW,
	SMB347_OTG_CONTROL_SW_PIN,
	SMB347_OTG_CONTROL_SW_AUTO,
};

/**
 * struct smb347_charger_platform_data - platform data for SMB347 charger
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
 * the SMB347 hardware.
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
 * values, pass in %SMB347_TEMP_USE_DEFAULT instead.
 */
struct smb345_charger_platform_data {
	struct power_supply_info battery_info;
	unsigned int	max_charge_current;
	unsigned int	max_charge_voltage;
	unsigned int	pre_charge_current;
	unsigned int	termination_current;
	unsigned int	pre_to_fast_voltage;
	unsigned int	mains_current_limit;
	unsigned int	usb_hc_current_limit;
	unsigned int	twins_h_current_limit;
	unsigned int	otg_uvlo_voltage;
	unsigned int	chip_temp_threshold;
	int		soft_cold_temp_limit;
	int		soft_hot_temp_limit;
	int		hard_cold_temp_limit;
	int		hard_hot_temp_limit;
	bool		suspend_on_hard_temp_limit;
	unsigned int	soft_temp_limit_compensation;
	unsigned int	charge_current_compensation;
	bool		use_mains;
	bool		use_usb;
	bool		show_battery;
	int		irq_gpio;
	int		inok_gpio;
#if defined(CONFIG_PF400CG) || defined(CONFIG_ME175CG) || defined(CONFIG_A400CG) || defined(CONFIG_PF450CL)
	int gp_sdio_2_clk;
#endif
	enum smb347_chg_enable enable_control;
	enum smb347_otg_control otg_control;
};

#endif /* SMB347_CHARGER_H */
