#ifndef _IA_CSS_TPROXY_CLIENT_PRIVATE_H_
#define _IA_CSS_TPROXY_CLIENT_PRIVATE_H_

#include "ia_css_tproxy_dev_access.h"
#include "ia_css_tproxy_eq.h"

STORAGE_CLASS_TPROXY_C ia_css_tproxy_chan_id_t ia_css_tproxy_get_chan_id(
		const ia_css_tproxy_chan_handle_t handle)
{
	return ((ia_css_tproxy_chan_id_t)handle);
}

STORAGE_CLASS_TPROXY_C void ia_css_tproxy_chan_open(
	const ia_css_tproxy_chan_handle_t chan_handle,
	const struct ia_css_tproxy_transfer_config * const transfer_config)
{
	struct ia_css_tproxy_eq_msg msg = {0, 0, 0};
	const ia_css_tproxy_chan_id_t chan_id =
		ia_css_tproxy_get_chan_id(chan_handle);
	uint32_t server_config_addr;

	IA_CSS_TPROXY_DEBUG_INFO("chan_open enter\n");

	// Send event to proxy server
	msg.event = IA_CSS_TPROXY_EVENT_CHAN_OPEN;
	msg.chan_id = chan_id;

	// Write config pointer to the server
	server_config_addr = IA_CSS_TPROXY_CHAN_CMEM_ADDR  +
		sizeof(struct ia_css_tproxy_server_chan_handle) * chan_id +
		offsetof(struct ia_css_tproxy_server_chan_handle,
			transfer_config_addr);
	ia_css_tproxy_mem_store_32(server_config_addr,
		(uint32_t)transfer_config + IA_CSS_TPROXY_CLIENT_DMEM_BASE);

	ia_css_tproxy_eq_send(IA_CSS_TPROXY_CLIENT_EQ_ID,
		IA_CSS_TPROXY_SERVER_EQ_ID, &msg)
		SYNC(IA_CSS_TPROXY_SYNC_LABEL0);

	IA_CSS_TPROXY_DEBUG_INFO("chan_open exit\n");
}

STORAGE_CLASS_TPROXY_C void ia_css_tproxy_chan_configure(
	const ia_css_tproxy_chan_handle_t chan_handle,
	const struct ia_css_tproxy_terminal_config * const terminal_config)
{
	struct ia_css_tproxy_eq_msg msg = {0, 0, 0};
	const ia_css_tproxy_chan_id_t chan_id =
		ia_css_tproxy_get_chan_id(chan_handle);
	uint32_t server_config_addr;

	IA_CSS_TPROXY_DEBUG_INFO("chan_configure enter\n");

	// Send event to proxy server
	msg.event = IA_CSS_TPROXY_EVENT_CHAN_CONFIGURE;
	msg.chan_id = chan_id;

	// Write config pointer to the server
	server_config_addr = IA_CSS_TPROXY_CHAN_CMEM_ADDR  +
		sizeof(struct ia_css_tproxy_server_chan_handle) * chan_id +
		offsetof(struct ia_css_tproxy_server_chan_handle,
			terminal_config_addr);
	ia_css_tproxy_mem_store_32(server_config_addr,
		(uint32_t)terminal_config + IA_CSS_TPROXY_CLIENT_DMEM_BASE);

	ia_css_tproxy_eq_send(IA_CSS_TPROXY_CLIENT_EQ_ID,
		IA_CSS_TPROXY_SERVER_EQ_ID, &msg)
		SYNC(IA_CSS_TPROXY_SYNC_LABEL1);

	IA_CSS_TPROXY_DEBUG_INFO("chan_configure exit\n");
}

STORAGE_CLASS_TPROXY_C void ia_css_tproxy_chan_nextN(
	const ia_css_tproxy_chan_handle_t chan_handle,
	const uint8_t n)
{
	struct ia_css_tproxy_eq_msg msg = {0, 0, 0};
	const ia_css_tproxy_chan_id_t chan_id =
		ia_css_tproxy_get_chan_id(chan_handle);

	IA_CSS_TPROXY_DEBUG_INFO("chan_nextN enter\n");

	// Send event to proxy server
	msg.event = IA_CSS_TPROXY_EVENT_CHAN_NEXT;
	msg.chan_id = chan_id;
	msg.opt   = n;
	ia_css_tproxy_eq_send(IA_CSS_TPROXY_CLIENT_EQ_ID,
		IA_CSS_TPROXY_SERVER_EQ_ID, &msg);

	IA_CSS_TPROXY_DEBUG_INFO("chan_nextN exit\n");
}

STORAGE_CLASS_TPROXY_C void ia_css_tproxy_chan_next(
	const ia_css_tproxy_chan_handle_t chan_handle)
{
	IA_CSS_TPROXY_DEBUG_INFO("chan_next enter\n");

	ia_css_tproxy_chan_nextN(chan_handle, 1);

	IA_CSS_TPROXY_DEBUG_INFO("chan_next exit\n");
}

STORAGE_CLASS_TPROXY_C void ia_css_tproxy_chan_close(
	ia_css_tproxy_chan_handle_t chan_handle)
{
	struct ia_css_tproxy_eq_msg msg = {0, 0, 0};
	const ia_css_tproxy_chan_id_t chan_id =
		ia_css_tproxy_get_chan_id(chan_handle);

	IA_CSS_TPROXY_DEBUG_INFO("chan_close enter\n");

	msg.event = IA_CSS_TPROXY_EVENT_CHAN_CLOSE;
	msg.chan_id = chan_id;
	ia_css_tproxy_eq_send(IA_CSS_TPROXY_CLIENT_EQ_ID,
		IA_CSS_TPROXY_SERVER_EQ_ID, &msg);

	IA_CSS_TPROXY_DEBUG_INFO("chan_close exit\n");
}

#endif /* _IA_CSS_TPROXY_CLIENT_PRIVATE_H_ */
