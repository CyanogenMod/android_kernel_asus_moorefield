/*
 * ia_css_devproxy_ctrl_inline.h
 *
 *  Created on: Nov 28, 2013
 *      Author: jboogers
 */

#ifndef IA_CSS_DEVPROXY_CTRL_INLINE_H_
#define IA_CSS_DEVPROXY_CTRL_INLINE_H_

#include "ia_css_devproxy_comm.h"
#include "eq_device_id.h"
#include "vied_nci_eq_send.h"
#include "ia_css_devproxy_sids.h"
#include "ia_css_devproxy_ctrl_eq_msg_structure.h"

#ifndef VOLATILE
# define VOLATILE
# warning VOLATILE not available on host
#endif

#ifndef NOT_USED
#define NOT_USED(a) ((a) = (a))
#endif

#define SID IA_CSS_DEVPROXY_SID_CTRL

IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_C
void ia_css_devproxy_ctrl_init(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num, /* [in] the channel to use */
	ia_css_devproxy_ctrl_reference_t *ref   /* [in] size of the state buffer */
)
{
	// place data somewhere, then
	// send event to eq
	vied_nci_eq_send_port_t p;
	vied_nci_eq_token_t token;
	// determin service offset
	vied_subsystem_address_t addr = get_service_addr(service_pid, channel_num);
	switch(service_pid){
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_DUMMY_0:
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_DUMMY_1:
		addr += offsetof(struct test_service_state_type, init_ref);
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_WBA_BC:
		addr += offsetof(struct ia_css_devproxy_service_wba_state_type, init_ref);
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_ANR:
		addr += offsetof(struct ia_css_devproxy_service_anr_state_type, init_ref);
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_DM:
		addr += offsetof(struct ia_css_devproxy_service_dm_state_type, init_ref);
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_BD_CCM:
		addr += offsetof(struct ia_css_devproxy_service_ccm_state_type, init_ref);
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_GTC:
		addr += offsetof(struct ia_css_devproxy_service_gtc_state_type, init_ref);
		break;
	case IA_CSS_DEVPROXY_CTRL_SERVICE_PID_IPF:
		addr += offsetof(struct ia_css_devproxy_service_ipf_state_type, init_ref);
		break;
	default:
		addr = 0;
		break;
	}
	if (addr != 0) {
		store_data_to_proxy(
			addr,
			ref,
			sizeof(*ref)) VOLATILE;
	}
	//Open EQ for send
	p = vied_nci_eq_send_port_open(EVENT_QUEUE_SPP0_ID);

	token = ia_css_devproxy_ctrl_token_pack(
		SID,
		service_pid,
		IA_CSS_DEVPROXY_CTRL_CMD_INIT,
		channel_num,
		IA_CSS_DEVPROXY_CTRL_EVENT_TYPE_TRIGGER);

	// reserve a token
	while (vied_nci_eq_reserve(p) == 0)
		;

	vied_nci_eq_send(p, IA_CSS_DEVPROXY_EQ_PRIORITY_CTRL, token);
}

IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_C
void ia_css_devproxy_ctrl_next(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num  /* [in] the channel to use */
)
{
	// send event to eq
	vied_nci_eq_token_t token = ia_css_devproxy_ctrl_token_pack(
		SID,
		service_pid,
		IA_CSS_DEVPROXY_CTRL_CMD_NEXT,
		channel_num,
		IA_CSS_DEVPROXY_CTRL_EVENT_TYPE_TRIGGER);

	vied_nci_eq_send_port_t p = vied_nci_eq_send_port_open(EVENT_QUEUE_SPP0_ID);

	// reserve a token
	while (vied_nci_eq_reserve(p) == 0)
		;
	vied_nci_eq_send(p, IA_CSS_DEVPROXY_EQ_PRIORITY_CTRL, token);
}

IA_CSS_DEVPROXY_CTRL_STORAGE_CLASS_C
void ia_css_devproxy_ctrl_done(
	unsigned int service_pid, /* [in] the service to address */
	unsigned int channel_num  /* [in] the channel to use */
)
{
	// TODO: implement
	// send event to eq
	vied_nci_eq_token_t token = ia_css_devproxy_ctrl_token_pack(
		SID,
		service_pid,
		IA_CSS_DEVPROXY_CTRL_CMD_DONE,
		channel_num,
		IA_CSS_DEVPROXY_CTRL_EVENT_TYPE_TRIGGER);

	vied_nci_eq_send_port_t p = vied_nci_eq_send_port_open(EVENT_QUEUE_SPP0_ID);

	// reserve a token
	while (vied_nci_eq_reserve(p) == 0)
		;

	vied_nci_eq_send(p, IA_CSS_DEVPROXY_EQ_PRIORITY_CTRL, token);
}



#endif /*IA_CSS_DEVPROXY_CTRL_INLINE_H_*/
