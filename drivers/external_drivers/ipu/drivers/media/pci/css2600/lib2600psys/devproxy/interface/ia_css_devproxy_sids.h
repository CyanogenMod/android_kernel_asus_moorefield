/*
 * This file enumerates the SID's of devices that send tokens to the 'devproxy'
 */
#ifndef IA_CSS_DEVPROXY_SIDS_H_
#define IA_CSS_DEVPROXY_SIDS_H_

/* the device SID's supported by devproxy */
typedef enum ia_css_devproxy_sid {
	IA_CSS_DEVPROXY_SID_CTRL,
	IA_CSS_DEVPROXY_SID_IPF0, /* need a SID per channel, no other way to distinguish */
	IA_CSS_DEVPROXY_SID_IPF1,
	IA_CSS_DEVPROXY_SID_IPF2,
	IA_CSS_DEVPROXY_SID_IPF3,
	IA_CSS_DEVPROXY_SID_IPF4,
	IA_CSS_DEVPROXY_SID_TOTAL_NR
} ia_css_devproxy_sid_t;

#endif /*IA_CSS_DEVPROXY_SIDS_H_*/
