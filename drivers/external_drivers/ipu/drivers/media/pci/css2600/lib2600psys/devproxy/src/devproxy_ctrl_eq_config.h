/*
 * This file defines how the devproxy event queue is configured
 * It defines the pid mapping of the event queue
 * It defines the how the priority feature of the event queue is used
 * Note that this is an internal definition as devproxy uses interfaces that hide the event queue
 */
#ifndef DEVPROXY_CTRL_EQ_CONFIG_H_
#define DEVPROXY_CTRL_EQ_CONFIG_H_

#include "ia_css_devproxy_ctrl_service_pids.h"

#define DEVPROXY_EQ_NUM_QUEUES     8 /*see vied_nci_eq_device_open()*/

#define DEVPROXY_EQ_PID_MAX 0x3F /* largest PID value to fit in 6 bits */
#define PIDMAP_QUEUE_NOT_USED DEVPROXY_EQ_PID_MAX

/* This is how PIDs are distributed over the pidmap queues */
enum devproxy_eq_pid_mapping {

	/* all service PIDs all fall in pidmap queue 0 */
	DEVPROXY_EQ_PID_END_QUEUE_0 = IA_CSS_DEVPROXY_CTRL_SERVICE_TOTAL_NR,

	/* these PIDS fall in pidmap queue 1..6 */
	DEVPROXY_EQ_PID_END_QUEUE_1 = PIDMAP_QUEUE_NOT_USED,
	DEVPROXY_EQ_PID_END_QUEUE_2 = PIDMAP_QUEUE_NOT_USED,
	DEVPROXY_EQ_PID_END_QUEUE_3 = PIDMAP_QUEUE_NOT_USED,
	DEVPROXY_EQ_PID_END_QUEUE_4 = PIDMAP_QUEUE_NOT_USED,
	DEVPROXY_EQ_PID_END_QUEUE_5 = PIDMAP_QUEUE_NOT_USED,
	DEVPROXY_EQ_PID_END_QUEUE_6 = PIDMAP_QUEUE_NOT_USED,
	DEVPROXY_EQ_PID_END_QUEUE_7 = PIDMAP_QUEUE_NOT_USED,
};

/* This enumeration defines the read port numbers of the above pidmap queues */
enum devproxy_eq_pidmap_recv_port {
	DEVPROXY_EQ_CTRL_RECV_PORT_NR = 0 /* only this one is used */
};


#endif /*DEVPROXY_CTRL_EQ_CONFIG_H_*/
