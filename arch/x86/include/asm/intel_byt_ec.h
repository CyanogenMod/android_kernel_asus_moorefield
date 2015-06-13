
#ifndef __INTEL_BYT_EC_H_
#define __INTEL_BYT_EC_H_

/*
 * SCI NOTIFICATIONS: EC When generates an event or interrupts
 * EC driver has to query the EC and based on the event EC driver
 * has to notify the corresponding consumer driver.
 */
#define BYT_EC_SCI_SMB			0x01	/* SMBus event notification */
#define BYT_EC_SCI_DEVINSERTION		0x20	/* Device bay insertion SCI */
#define BYT_EC_SCI_DEVREMOVAL		0x21	/* Device bay removal SCI */
#define BYT_EC_SCI_NEWCARD		0x22/* NewCard insertion event SCI */
#define BYT_EC_SCI_ACINSERTION		0x30	/* AC insertion SCI */
#define BYT_EC_SCI_ACREMOVAL		0x31	/* AC removal SCI */
#define BYT_EC_SCI_BATTERY		0x32	/* Battery event SCI */
#define BYT_EC_SCI_BATTERY_PRSNT	0x33/* Battery insertion/removal SCI */
#define BYT_EC_SCI_BATTERY_OTP		0x34/* Battery Over Temperature SCI */
 /* Battery Over Temperature to Normal SCI */
#define BYT_EC_SCI_BATTERY_OTP_CLR	0x35
/* Battery Extreme Temperature SCI */
#define BYT_EC_SCI_BATTERY_ETP		0x38
/* Battery Extreme Temperature Clear SCI */
#define BYT_EC_SCI_BATTERY_ETP_CLR	0x39
#define BYT_EC_SCI_DOCKED		0x40	/* Dock complete SCI */
#define BYT_EC_SCI_UNDOCKED		0x41	/* Undock complete SCI */
#define BYT_EC_SCI_UNDOCKREQUEST	0x42	/* Undocking request SCI */
/* Express card insertion/removal SCI */
#define BYT_EC_SCI_EXPCARDPRSNT		0x43

#define BYT_EC_SCI_RING			0x50/* SCI from ring indicate event */
#define BYT_EC_SCI_LID			0x51	/* SCI from lid */
#define BYT_EC_SCI_HOTKEY		0x52	/* SCI from keyboard hotkey */
/* SCI from virtual battery switch */
#define BYT_EC_SCI_VB			0x53
#define BYT_EC_SCI_PWRBTN		0x54	/* SCI from EL power button */
#define BYT_EC_SCI_RESUME		0x55	/* SCI from Resuming from S3 */

#define BYT_EC_SCI_BT_PWR_OFF		0x60/* SCI from Bluetooth power off */
#define BYT_EC_SCI_BT_PWR_ON		0x61/* SCI from Bluetooth power on */
#define BYT_EC_SCI_ALS			0x70	/* SCI from ALS lux change */
#define BYT_EC_SCI_BURST_ACK		0x90	/* SCI burst acknowledge byte */
/* SCI for Geyserville break event */
#define BYT_EC_SCI_GYSRVL		0xB0
#define BYT_EC_SCI_THERMAL		0xF0	/* Thermal transition SCI */
/* Thermal trip point transition SCI */
#define BYT_EC_SCI_THERMTRIP		0xF1

/* Legacy Event from EC */
#define BYT_EC_SCI_AON_UP		0xD0	/* SCI from AON Up button */
#define BYT_EC_SCI_AON_DOWN		0xD1	/* SCI from AON Down button */
#define BYT_EC_SCI_AON_SELECT		0xD2	/* SCI from AON Select button */
#define BYT_EC_SCI_AON_ESC		0xD3	/* SCI from AON Escape button */
/* SCI from AON Global Escape button */
#define BYT_EC_SCI_AON_GESC		0xD4

/* Button Event */
#define BYT_EC_SCI_VOLUMEUP_BTN		0x80	/* SCI from vol up button */
#define BYT_EC_SCI_VOLUMEDOWN_BTN	0x81	/* SCI from vol down button */
#define BYT_EC_SCI_HOME_BTN		0x85	/* SCI from home button */
#define BYT_EC_SCI_POWER_BTN		0x86	/* SCI from power button */

#define BYT_EC_BUTTON_STATUS		0xC9

/* Button Event */
#define BYT_EC_SCI_VOLUMEUP_BTN		0x80	/* SCI from vol up button */
#define BYT_EC_SCI_VOLUMEDOWN_BTN	0x81	/* SCI from vol down button */
#define BYT_EC_SCI_HOME_BTN		0x85	/* SCI from home button */
#define BYT_EC_SCI_POWER_BTN		0x86	/* SCI from power button */

#define BYT_EC_BUTTON_STATUS		0xC9

/* EC commands */
enum byt_ec_commands {
	BYT_EC_ACPI_ENABLE = 0xAA,
	BYT_EC_ACPI_DISABLE = 0xAB,
	BYT_EC_COMMAND_READ = 0x80,
	BYT_EC_COMMAND_WRITE = 0x81,
	BYT_EC_BURST_ENABLE = 0x82,
	BYT_EC_BURST_DISABLE = 0x83,
	BYT_EC_COMMAND_QUERY = 0x84,
	BYT_EC_SET_FAN_SPEED = 0x1A,
	BYT_EC_SET_THERM_THRESHOLD = 0x4A,
};

#ifdef CONFIG_INTEL_BYT_EC
extern int byt_ec_read_byte(u8 addr, u8 *val);
extern int byt_ec_write_byte(u8 addr, u8 val);
extern int byt_ec_read_word(u8 addr, u16 *val);
extern int byt_ec_write_word(u8 addr, u16 val);
extern int byt_ec_send_cmd(u8 command);
extern void byt_ec_evt_register_notify(struct notifier_block *nb);
extern void byt_ec_evt_unregister_notify(struct notifier_block *nb);
#else
static int byt_ec_read_byte(u8 addr, u8 *val)
{
	return 0;
}
static int byt_ec_write_byte(u8 addr, u8 val)
{
	return 0;
}
static int byt_ec_read_word(u8 addr, u16 *val)
{
	return 0;
}
static int byt_ec_write_word(u8 addr, u16 val)
{
	return 0;
}
static int byt_ec_send_cmd(u8 command)
{
	return 0;
}
static void byt_ec_evt_register_notify(struct notifier_block *nb) { }
static void byt_ec_evt_unregister_notify(struct notifier_block *nb) { }
#endif

#endif	/* __INTEL_BYT_EC_H_ */
