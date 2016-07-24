#ifndef _VIED_NCI_EQ_RECV_INLINE_H
#define _VIED_NCI_EQ_RECV_INLINE_H

#include "vied_nci_eq_const.h"
#include "vied_nci_eq_recv.h"
#include "vied_nci_eq_properties.h"
#include "vied_nci_eq_reg_access.h"
#include "assert_support.h"
#include "vied_nci_eq_storage_class.h"

#ifdef _EVENT_QUEUE_ON_CELL_
#include "vied_nci_eq_cell_reg_access.h"
#endif

VIED_NCI_EQ_STORAGE_CLASS_C vied_nci_eq_recv_port_t
vied_nci_eq_recv_port_open(vied_device_id id)
{
    // check that the given device is connected to this cell's qmem port

    // return the device base address on this cell's qmem port
    id = id;
    return SELF_QUEUE;
}

/* 0x400
 * Get number of tokens for queue = queue_nr */
VIED_NCI_EQ_STORAGE_CLASS_C
unsigned int
vied_nci_eq_available (vied_nci_eq_recv_port_t port, unsigned int queue_nr)
{
    unsigned int read_val;

    port = port;
    OP___assert(queue_nr < eq_device_properties.nr_queues);

    read_val  = event_queue_op_reg_load(EVENT_QUEUE_IP_QSTAT_BASE + queue_nr*0x4);

    return (read_val);
}

/* 0x600
 * Get number of tokens for all queues */
VIED_NCI_EQ_STORAGE_CLASS_C
unsigned int
vied_nci_eq_all_available(vied_nci_eq_recv_port_t port)
{
    port = port;
    return event_queue_op_reg_load(EVENT_QUEUE_IP_TOT_QSTAT);
}

/* Read toke from the event queue
 * Read must from the output port
 */
VIED_NCI_EQ_STORAGE_CLASS_C
vied_nci_eq_token_t
vied_nci_eq_recv(vied_nci_eq_recv_port_t port, unsigned int queue_nr)
{
    port = port;
    return event_queue_op_reg_load(EVENT_QUEUE_0P_QUEUE_BASE+(queue_nr*0x4));
}

/* Token help function */
VIED_NCI_EQ_STORAGE_CLASS_C
vied_nci_eq_msg_t
vied_nci_eq_get_msg(vied_nci_eq_token_t token)
{
    return token & EQ_MSG_MAX;
}

VIED_NCI_EQ_STORAGE_CLASS_C
vied_nci_eq_pid_t
vied_nci_eq_get_pid(vied_nci_eq_token_t token)
{
    return (token >> EQ_MSG_BITS) & EQ_PID_MAX;
}

VIED_NCI_EQ_STORAGE_CLASS_C
vied_nci_eq_sid_t
vied_nci_eq_get_sid(vied_nci_eq_token_t token)
{
    return token >> (EQ_MSG_BITS + EQ_PID_BITS);
}


#endif /* _VIED_NCI_EQ_RECV_INLINE_H */

