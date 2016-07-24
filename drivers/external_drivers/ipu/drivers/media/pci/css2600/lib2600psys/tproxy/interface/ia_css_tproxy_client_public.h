#ifndef _IA_CSS_TPROXY_CLIENT_PUBLIC_H_
#define _IA_CSS_TPROXY_CLIENT_PUBLIC_H_

STORAGE_CLASS_TPROXY_H void ia_css_tproxy_chan_open(
	const ia_css_tproxy_chan_handle_t chan_handle,
	const struct ia_css_tproxy_transfer_config *transfer_config);

STORAGE_CLASS_TPROXY_H void ia_css_tproxy_chan_configure(
	const ia_css_tproxy_chan_handle_t chan_handle,
	const struct ia_css_tproxy_terminal_config *terminal_config);

STORAGE_CLASS_TPROXY_H void ia_css_tproxy_chan_next(
	const ia_css_tproxy_chan_handle_t chan_handle);

STORAGE_CLASS_TPROXY_H void ia_css_tproxy_chan_nextN(
	const ia_css_tproxy_chan_handle_t chan_handle,
	const uint8_t n);

STORAGE_CLASS_TPROXY_H void ia_css_tproxy_chan_close(
	const ia_css_tproxy_chan_handle_t chan_handle);

#endif /*_IA_CSS_TPROXY_CLIENT_PUBLIC_H_*/
