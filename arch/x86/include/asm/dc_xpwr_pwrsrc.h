
#ifndef __DC_XPWR_PWRSRC_H_
#define __DC_XPWR_PWRSRC_H_

struct dc_xpwr_pwrsrc_pdata {
	bool	en_chrg_det;
#if defined(CONFIG_MRD8) || defined(CONFIG_MRD7P05)
	int     mux_gpio;
#endif
};

#ifdef CONFIG_INTEL_MID_PMIC
int dc_xpwr_vbus_on_status(void);
#else
static int dc_xpwr_vbus_on_status(void)
{
	return 0;
}
#endif

#endif
