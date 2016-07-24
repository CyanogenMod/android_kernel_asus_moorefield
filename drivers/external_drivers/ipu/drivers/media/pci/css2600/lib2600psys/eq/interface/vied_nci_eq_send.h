#ifndef _VIED_NCI_EQ_SEND_H
#define _VIED_NCI_EQ_SEND_H

#include "vied_nci_eq_storage_class.h"
#include "vied_nci_eq_types.h"

typedef unsigned int vied_nci_eq_send_port_t; // impl dependent

VIED_NCI_EQ_STORAGE_CLASS_H
vied_nci_eq_send_port_t
vied_nci_eq_send_port_open(vied_device_id id);

VIED_NCI_EQ_STORAGE_CLASS_H
int
vied_nci_eq_reserve(vied_nci_eq_send_port_t port);

VIED_NCI_EQ_STORAGE_CLASS_H
vied_nci_eq_token_t
vied_nci_eq_pack(vied_nci_eq_sid_t sid, vied_nci_eq_pid_t pid, vied_nci_eq_msg_t msg);

VIED_NCI_EQ_STORAGE_CLASS_H
void
vied_nci_eq_send(vied_nci_eq_send_port_t port, vied_nci_eq_priority_t prio, vied_nci_eq_token_t token);

/* get a send port for eq that can be used from another device's master port mt */
/*
vied_nci_eq_send_port_t
vied_nci_eq_get_remote_send_port(vied_device_id id, vied_master_port_id mt);
*/

#ifdef __INLINE_VIED_NCI_EQ__
#include "vied_nci_eq_send_inline.h"
#endif

#endif

