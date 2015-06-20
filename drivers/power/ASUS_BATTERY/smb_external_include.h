#ifndef _SMB347_EXTERNAL_INCLUDE_H
#define _SMB347_EXTERNAL_INCLUDE_H 1

#if defined(CONFIG_A500CG_BATTERY_SMB347)
/* This function is exported to external.
 * usb_state should be one of the above
 *
 * return 0 means success */
extern int setSMB347Charger(int usb_state);

/* To know the charging status
 *
 * return true when charger is charging */
extern int smb347_get_charging_status(void);

/* To enable/disable charging
 *
 * return 0 means success */
extern int smb347_charging_toggle(bool on);

/* To set fast charge
 *
 * return 0 means success */
extern int smb347_set_fast_charge(void);

/* To set ac in current
 *
 * return 0 means success */
extern int smb347_AC_in_current(void);

/* To control JEITA
 *
 * return 0 means success */
extern int smb347_control_JEITA(bool on);


/* To set voltage
 *
 * return 0 means success */
extern int smb347_set_voltage(bool on);

/* To set Battery 0V
 *
 * return 0 means success */
extern int smb347_set_battery_0V(void);

/*
 * To get AICL result
 */
extern int smb347_get_aicl_result(void);

/* To know if charger has an error
 *
 * return true means charger has an error */
extern bool smb347_has_charger_error(void);
#elif defined(CONFIG_SMB1357_CHARGER)
extern int setSMB1357Charger(int usb_state);
extern int smb1357_get_charging_status(void);
extern int smb1357_charging_toggle(bool on);
extern int smb1357_set_fast_charge(void);
extern int smb1357_AC_in_current(void);
extern int smb1357_control_JEITA(bool on);
extern int smb1357_set_voltage(bool on);
extern int smb1357_get_aicl_result(void);
extern bool smb1357_has_charger_error(void);
extern int set_QC_inputI_limit(int type);
/*
 * To set fast charge current
 */
extern int smb1357_set_Ichg(int i);
/*
 * To set recharge voltage
 * true meas recharge voltage=Vflt-50mV
 * false meas recharge voltage=Vflt-200mV
 */
extern int smb1357_set_recharge(bool high);
extern int smb1357_watchdog_timer_enable(void);
#else
static int setSMB347Charger(int usb_state)
{
	return 0;
}
static int smb347_get_charging_status(void)
{
	return 0;
}
static int smb347_charging_toggle(bool on)
{
	return 0;
}
static int smb347_set_fast_charge(void)
{
	return 0;
}
static int smb347_AC_in_current(void)
{
	return 0;
}
static int smb347_control_JEITA(bool on)
{
	return 0;
}
static int smb347_set_voltage(bool on)
{
	return 0;
}
static int smb347_set_battery_0V(void)
{
	return 0;
}
static int smb347_get_aicl_result(void)
{
	return 0;
}
static int smb347_has_charger_error(void)
{
	return 0;
}
static int setSMB1357Charger(int usb_state)
{
	return 0;
}
static int smb1357_get_charging_status(void)
{
	return 0;
}
static int smb1357_charging_toggle(bool on)
{
	return 0;
}
static int smb1357_set_fast_charge(void)
{
	return 0;
}
static int smb1357_AC_in_current(void)
{
	return 0;
}
static int smb1357_control_JEITA(bool on)
{
	return 0;
}
static int smb1357_set_voltage(bool on)
{
	return 0;
}
static int smb1357_set_battery_0V(void)
{
	return 0;
}
static int smb1357_get_aicl_result(void)
{
	return 0;
}
static int smb1357_has_charger_error(void)
{
	return 0;
}
static int smb1357_set_Ichg(int i)
{
	return 0;
}
static int smb1357_set_recharge(bool high)
{
	return 0;
}
static int set_QC_inputI_limit(int type)
{
	return 0;
}
static int smb1357_watchdog_timer_enable(void)
{
	return 0;
}
#endif

#endif /* _SMB347_EXTERNAL_INCLUDE_H */

