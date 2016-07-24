/*
 * This file defines how the devproxy uses the msg field of event queue tokens
 * Note that this is an internal definition as devproxy uses interfaces that hide the eq
 */

#ifndef DEVPROXY_CTRL_EQ_MSG_STRUCTURE_H_
#define DEVPROXY_CTRL_EQ_MSG_STRUCTURE_H_

#include "vied_nci_eq_send.h"
#include "vied_nci_eq_recv.h"
#include "vied_nci_eq_types.h"
#include "type_support.h"

/*
Note:
The split the eq 'msg' field into a 'func', 'channel', 'subtype' and optionally
'data' fields is needed, because some classes of events will carry parameter
values. The data sub-field is used to support parameters that.
This implies that the devproxy event pump will have to mask off the 'data' sub-field
So before classifying events the 'data' sub-field needs to be masked off,
then when delegating event the original event needs to forwarded.
20 bits, not always all available:
[subtype][channel][func]
*/

#ifndef NOT_USED
#define NOT_USED(a) ((a) = (a))
#endif

/*This is how event queue priorities are used. See vied_nci_eq_device_open()*/
enum ia_css_devproxy_eq_priorities {
	IA_CSS_DEVPROXY_EQ_PRIORITY_SHUTDOWN, /* lowest number: highest event priority */
	IA_CSS_DEVPROXY_EQ_PRIORITY_ACK,
	IA_CSS_DEVPROXY_EQ_PRIORITY_CTRL,      /* highest number: lowest event priority */
	IA_CSS_DEVPROXY_EQ_NUM_PRIORITIES
};


/* masks to split the eq 'msg' field into a 'func' and 'data' sub-field */
#define IA_CSS_DEVPROXY_CTRL_FUNC_MASK	0x0000000F
#define IA_CSS_DEVPROXY_CTRL_CHAN_MASK	0x00000FF0
#define IA_CSS_DEVPROXY_CTRL_CHAN_POS  4
#define IA_CSS_DEVPROXY_CTRL_TYPE_MASK	0x0000F000
#define IA_CSS_DEVPROXY_CTRL_TYPE_POS  12
#define	IA_CSS_DEVPROXY_CTRL_DATA_MASK	0xFFFF0000 /* ~DEVPROXY_CTRL_FUNC_MASK */
#define IA_CSS_DEVPROXY_CTRL_DATA_POS  16

/* predefined values of the 'func' field */
#define IA_CSS_DEVPROXY_CTRL_CMD_SHUTDOWN 	 0 /* reserved for DEVPROXY_CTRL_EVENT_SHUTDOWN */
#define IA_CSS_DEVPROXY_CTRL_CMD_INIT 	 1
#define IA_CSS_DEVPROXY_CTRL_CMD_NEXT 	 2
#define IA_CSS_DEVPROXY_CTRL_CMD_DONE 	 3
#define IA_CSS_DEVPROXY_CTRL_CMD_ACK 	 4

#define IA_CSS_DEVPROXY_CTRL_EVENT_TYPE_TRIGGER 7

#define IA_CSS_DEVPROXY_CTRL_EVENT_SHUTDOWN 	 0  /* this value is checked often so it is not split in sid,pid,...etc.. */

static inline vied_nci_eq_token_t ia_css_devproxy_ctrl_token_pack(unsigned int sid, unsigned int service, unsigned int cmd, unsigned int channel, unsigned int type)
{
	vied_nci_eq_token_t token =  vied_nci_eq_pack(
		sid,
		service,
		(cmd & IA_CSS_DEVPROXY_CTRL_FUNC_MASK) |
		((channel << IA_CSS_DEVPROXY_CTRL_CHAN_POS) & IA_CSS_DEVPROXY_CTRL_CHAN_MASK) |
		((type << IA_CSS_DEVPROXY_CTRL_TYPE_POS) & IA_CSS_DEVPROXY_CTRL_TYPE_MASK));
	return token;
}

static inline vied_nci_eq_msg_t ia_css_devproxy_ctrl_token_get_func(vied_nci_eq_token_t event)
{
	/* extract the 'func'/'cmd' field from the 'msg' field */
	vied_nci_eq_msg_t func;
	func = vied_nci_eq_get_msg(event) & IA_CSS_DEVPROXY_CTRL_FUNC_MASK;
	return func;
}

static inline unsigned int ia_css_devproxy_ctrl_token_get_channel(vied_nci_eq_token_t event)
{
	/* extract the 'channel' field from the 'msg' field */
	unsigned int channel;
	channel = (vied_nci_eq_get_msg(event) & IA_CSS_DEVPROXY_CTRL_CHAN_MASK) >> IA_CSS_DEVPROXY_CTRL_CHAN_POS;
	return channel;
}

static inline vied_nci_eq_msg_t ia_css_devproxy_ctrl_token_get_subtype(vied_nci_eq_token_t event)
{
	/* extract the 'subtype' field from the 'msg' field */
	unsigned int subtype;
	subtype = (vied_nci_eq_get_msg(event) & IA_CSS_DEVPROXY_CTRL_TYPE_MASK) >> IA_CSS_DEVPROXY_CTRL_TYPE_POS;
	return subtype;
}


// TODO: remove, should not be used/available
static inline vied_nci_eq_msg_t ia_css_devproxy_ctrl_token_get_data(vied_nci_eq_token_t event)
{
	/* extract the 'data' field from the 'msg' field */
	vied_nci_eq_msg_t data;
	data = vied_nci_eq_get_msg(event) & IA_CSS_DEVPROXY_CTRL_DATA_MASK;
	return data;
}

#endif /*DEVPROXY_CTRL_EQ_MSG_STRUCTURE_H_*/
