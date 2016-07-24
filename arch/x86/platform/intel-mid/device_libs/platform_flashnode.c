/*
 * platform_flashnode: FLASHNODE platform data initilization file
 *
 * Chung-Yi (chung-yi_chou@asus.com)
 */

#include <linux/init.h>
#include <linux/types.h>
#include <asm/intel-mid.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <media/flashnode.h>

#include "platform_flashnode.h"

/**
 * struct FLASHNODE_platform_data
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
// struct FLASHNODE_platform_data
// {
// 	const char *name[FLASHNODE_MAX];
// 	int gpio_enable;
// 	bool fl1_by_pin;
// 	bool fl2_by_pin;
// 	int current_limit;
// 	bool disable_short_led_report;
// 	bool shutoff_on_inhibit_mode;
// 	bool enable_voltage_monitor;
// 	int input_voltage_threshold;
// 	int input_voltage_hysteresis;
// };

void *flashnode_platform_data_func(void *info)
{
	static struct flashnode_platform_data platform_FLASHNODE_data;
	platform_FLASHNODE_data.gpio_enable = 4;
	platform_FLASHNODE_data.fl1_by_pin = true;
	platform_FLASHNODE_data.fl2_by_pin = true;
	platform_FLASHNODE_data.current_limit = 2600;
	platform_FLASHNODE_data.disable_short_led_report = true;
	platform_FLASHNODE_data.shutoff_on_inhibit_mode = true;
	platform_FLASHNODE_data.enable_voltage_monitor = true;
	platform_FLASHNODE_data.input_voltage_threshold = 3400;
	platform_FLASHNODE_data.input_voltage_hysteresis = 3500;
	printk("flashnode_platform_data_func\n");
	return &platform_FLASHNODE_data;
}
