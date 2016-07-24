#ifndef _IA_CSS_TPROXY_EQ_H_
#define _IA_CSS_TPROXY_EQ_H_

#include <type_support.h>
#include <vied_bit.h>
#include <eq_device_id.h>
#include <vied_nci_eq_types.h>
#include <vied_nci_eq_device.h>
#include <vied_nci_eq_send.h>
#include <vied_nci_eq_recv.h>
#include "ia_css_tproxy_local.h"
#include "ia_css_tproxy_dev_access.h"

#define DEFAULT_EVENT_PRIO 0
#define EVENT_PRIO_OFFSET  0x4
#define IA_CSS_TPROXY_EQ_NUM_QUEUES     8 /*see vied_nci_eq_device_open()*/
#define IA_CSS_TPROXY_EQ_NUM_PRIORITIES 1 /*see vied_nci_eq_device_open()*/

//TODO: place the following structure here to enable config write by client,
// should place in ia_css_tproxy_server.h
enum ia_css_tproxy_chan_state {
	IA_CSS_TPROXY_CHAN_STATE_IDLE,
	IA_CSS_TPROXY_CHAN_STATE_AVALIABLE,
	IA_CSS_TPROXY_CHAN_STATE_READY,
	IA_CSS_TPROXY_CHAN_STATE_RUNNING,
	IA_CSS_N_TPROXY_CHAN_STATE
};

struct vied_nci_dma_chan_t;
struct ia_css_tproxy_server_chan_handle {
	ia_css_tproxy_chan_id_t chan_id;
	uint8_t state;
	uint8_t command;
	struct vied_nci_dma_chan_t *hndl;
	SYNC_WITH(IA_CSS_TPROXY_SYNC_LABEL0) uint32_t transfer_config_addr;
	SYNC_WITH(IA_CSS_TPROXY_SYNC_LABEL1) uint32_t terminal_config_addr;
	struct ia_css_tproxy_buf_info buf_info;
};

/* This is how PIDs are distributed over the pidmap queues */
enum ia_css_tproxy_eq_pid_mapping {

	IA_CSS_TPROXY_EQ_PID_DMA_REQ   =  0x00,

	/* the above PIDS all fall in pidmap queue 0 */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_0 = 0x10,

	/* these PIDS fall in pidmap queue 1..6 */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_1 = 0x11, /* not used */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_2 = 0x12, /* not used */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_3 = 0x13, /* not used */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_4 = 0x14, /* not used */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_5 = 0x15, /* not used */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_6 = 0x16, /* not used */

	IA_CSS_TPROXY_EQ_PID_MAX = 0x3F, /* largest PID value to fit in 6 bits */

	/* the remaining PIDS fall in pidmap queue7 */
	IA_CSS_TPROXY_EQ_PID_END_QUEUE_7 = IA_CSS_TPROXY_EQ_PID_MAX,
};

/* This enumeration defines the read port numbers of the above pidmap queues */
enum tproxy_eq_pidmap_recv_port {
	IA_CSS_TPROXY_EQ_RECV_QUEUE = 0,
	IA_CSS_TPROXY_EQ_ACK_QUEUE  = 7,
};

enum ia_css_tproxy_event {
	IA_CSS_TPROXY_EVENT_NONE,
	IA_CSS_TPROXY_EVENT_CHAN_OPEN,
	IA_CSS_TPROXY_EVENT_CHAN_CONFIGURE,
	IA_CSS_TPROXY_EVENT_CHAN_NEXT,
	IA_CSS_TPROXY_EVENT_CHAN_ACK,
	IA_CSS_TPROXY_EVENT_CHAN_CLOSE,
	IA_CSS_N_TPROXY_EVENT
};
;

struct ia_css_tproxy_eq_msg {
	uint8_t event;
	ia_css_tproxy_chan_id_t chan_id;
	uint8_t opt;
};


/* For as long as it lasts, lets use a simple 1:1 mapping of accelerators to PID's */
//#define IA_CSS_TPROXY_MAP_DEV_2_PID(dev)   ((uint32_t)dev)
//#define IA_CSS_TPROXY_MAP_PID_2_DEV(pid)   ((enum ia_css_tproxy_dev_id)pid)
#define IA_CSS_TPROXY_MSG_EVENT_BITS	4   /* Event bits in message */
#define IA_CSS_TPROXY_MSG_CID_BITS	8   /* Channel ID bits in message */
#define IA_CSS_TPROXY_MSG_OPT_BITS	8   /* Optional data bits e.g. n for nextN */
#define IA_CSS_TPROXY_MSG_BITS		20


STORAGE_CLASS_INLINE vied_nci_eq_msg_t ia_css_tproxy_eq_create_msg(
	const enum ia_css_tproxy_event event,
	const ia_css_tproxy_chan_id_t chan_id,
	const uint8_t opt)
{
	uint32_t msg
		= vied_bit_shift_OR(event, IA_CSS_TPROXY_MSG_CID_BITS, chan_id);

	IA_CSS_TPROXY_DEBUG_ASSERT((IA_CSS_TPROXY_MSG_EVENT_BITS +
			IA_CSS_TPROXY_MSG_CID_BITS +
			IA_CSS_TPROXY_MSG_OPT_BITS) <= IA_CSS_TPROXY_MSG_BITS);

	return vied_bit_shift_OR(msg, IA_CSS_TPROXY_MSG_OPT_BITS, opt);
}


STORAGE_CLASS_INLINE uint8_t ia_css_tproxy_eq_get_msg_event(
	const vied_nci_eq_msg_t msg)
{
	return vied_bit_slice(msg,
		IA_CSS_TPROXY_MSG_OPT_BITS + IA_CSS_TPROXY_MSG_CID_BITS,
		IA_CSS_TPROXY_MSG_EVENT_BITS);
}

STORAGE_CLASS_INLINE uint8_t ia_css_tproxy_eq_get_msg_cid(
	const vied_nci_eq_msg_t msg)
{
	return vied_bit_slice(msg, IA_CSS_TPROXY_MSG_OPT_BITS,
		IA_CSS_TPROXY_MSG_CID_BITS);
}

STORAGE_CLASS_INLINE uint8_t ia_css_tproxy_eq_get_msg_opt(
	const vied_nci_eq_msg_t msg)
{
	return vied_bit_slice(msg, 0, IA_CSS_TPROXY_MSG_OPT_BITS);
}

STORAGE_CLASS_INLINE vied_nci_eq_token_t ia_css_tproxy_eq_queue_addr(
	const uint8_t prio)
{
	return IA_CSS_TPROXY_SERVER_EQ_BASE + prio * EVENT_PRIO_OFFSET;
}

STORAGE_CLASS_INLINE vied_nci_eq_token_t ia_css_tproxy_eq_pack(
	const struct ia_css_tproxy_eq_msg * const tmsg)
{
	const event_queue_ID_t sid = IA_CSS_TPROXY_SERVER_EQ_ID;
	vied_nci_eq_sid_t pid;
	vied_nci_eq_msg_t msg;

	msg = ia_css_tproxy_eq_create_msg(
		tmsg->event, tmsg->chan_id, tmsg->opt);
	pid = 0;  //TODO: need to use pid?
	return vied_nci_eq_pack(sid, pid, msg);
}

STORAGE_CLASS_INLINE void ia_css_tproxy_eq_send(
	const event_queue_ID_t src,
	const event_queue_ID_t dest,
	const struct ia_css_tproxy_eq_msg * const tmsg)
{
	vied_nci_eq_send_port_t p;
	vied_nci_eq_pid_t pid;
	vied_nci_eq_token_t token;
	vied_nci_eq_msg_t msg;

	//Open EQ for send
	p = vied_nci_eq_send_port_open(dest);

	IA_CSS_TPROXY_DEBUG_ASSERT(p);
	// reserve a token
	while (vied_nci_eq_reserve(p) == 0)
		;

	msg = ia_css_tproxy_eq_create_msg(
		tmsg->event, tmsg->chan_id, tmsg->opt);
	pid = 0;  //TODO: need to use pid?
	token = vied_nci_eq_pack(src, pid, msg);
	IA_CSS_TPROXY_DEBUG_INFO("EQ_send:");
	IA_CSS_TPROXY_DEBUG_DUMP(token);

	vied_nci_eq_send(p, DEFAULT_EVENT_PRIO, token);
	IA_CSS_TPROXY_DEBUG_INFO("EQ_send exit\n");
}

STORAGE_CLASS_INLINE bool ia_css_tproxy_eq_is_event_available(
	const uint32_t queue)
{
	const vied_nci_eq_recv_port_t p =
		vied_nci_eq_recv_port_open(IA_CSS_TPROXY_SERVER_EQ_ID);

	if (vied_nci_eq_available(p, queue) == 0) {
		return false;
	}

	return true;
}


STORAGE_CLASS_INLINE bool ia_css_tproxy_eq_receive(
	const uint32_t queue,
	struct ia_css_tproxy_eq_msg * const tmsg)
{
	//TODO: recv_port_open always open the port on its device
	vied_nci_eq_msg_t msg;
	vied_nci_eq_token_t token;
	const vied_nci_eq_recv_port_t p =
		vied_nci_eq_recv_port_open(IA_CSS_TPROXY_SERVER_EQ_ID);

	if (vied_nci_eq_available(p, queue) == 0) {
		return false;
	}

	token = vied_nci_eq_recv(p, queue);
	msg = vied_nci_eq_get_msg(token);
	IA_CSS_TPROXY_DEBUG_INFO("event received: ");
	IA_CSS_TPROXY_DEBUG_DUMP(token);
	tmsg->chan_id = ia_css_tproxy_eq_get_msg_cid(msg);
	tmsg->event = ia_css_tproxy_eq_get_msg_event(msg);
	tmsg->opt = ia_css_tproxy_eq_get_msg_opt(msg);

	return true;
}

#endif /*_IA_CSS_TPROXY_EQ_H_*/
