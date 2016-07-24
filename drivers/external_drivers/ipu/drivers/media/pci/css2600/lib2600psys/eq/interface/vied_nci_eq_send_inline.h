#ifndef _VIED_NCI_EQ_INLINE_H
#define _VIED_NCI_EQ_INLINE_H

#include "vied_nci_eq_const.h"
#include "vied_nci_eq_send.h"
#include "vied_nci_eq_properties.h"
#include "device_address.h"
#include "vied_nci_eq_reg_access.h"
#include "assert_support.h"

#include "vied_nci_eq_storage_class.h"

#ifdef _EVENT_QUEUE_ON_CELL_
#include "vied_nci_eq_cell_reg_access.h"
#endif

/* Get the device properties, it is platform depdendent */
VIED_NCI_EQ_STORAGE_CLASS_C
const vied_nci_eq_device_properties_t *vied_nci_eq_get_properties(void)
{
    return &eq_device_properties;
}

VIED_NCI_EQ_STORAGE_CLASS_C
vied_nci_eq_send_port_t
vied_nci_eq_send_port_open(vied_device_id id)
{
    //OP___assert(id < N_EVENT_QUEUE_ID);
    // vied open device
    // vied get route -> base address -> dev
    return (unsigned int)EVENT_QUEUE_IP_BASE[id];
    // open the device
    // get route  -> device base address
}

VIED_NCI_EQ_STORAGE_CLASS_C
int
vied_nci_eq_reserve(vied_nci_eq_send_port_t port)
{
    return event_queue_ip_reg_load(port, EVENT_QUEUE_IP_QRESERVE);
}

VIED_NCI_EQ_STORAGE_CLASS_C
vied_nci_eq_token_t
vied_nci_eq_pack(vied_nci_eq_sid_t sid, vied_nci_eq_pid_t pid, vied_nci_eq_msg_t msg)
{
    OP___assert(sid <= (unsigned int)EQ_SID_MAX);
    OP___assert(pid <= (unsigned int)EQ_PID_MAX);
    OP___assert(msg <= (unsigned int)EQ_MSG_MAX);

    return (((sid << eq_device_properties.pid_size) + pid) <<
            eq_device_properties.msg_size) + msg;
}

VIED_NCI_EQ_STORAGE_CLASS_C
void
vied_nci_eq_send(vied_nci_eq_send_port_t port, unsigned int prio, vied_nci_eq_token_t token)
{
    event_queue_ip_reg_store(port, EVENT_QUEUE_IP_QUEUE_BASE+(prio * 0x4), token);

}

#endif /* _VIED_NCI_EQ_INLINE_H */

