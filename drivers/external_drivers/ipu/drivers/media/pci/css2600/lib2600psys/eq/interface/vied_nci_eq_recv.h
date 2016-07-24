#ifndef _VIED_NCI_EQ_RECV_H_
#define _VIED_NCI_EQ_RECV_H_

#include "vied_nci_eq_storage_class.h"
#include "vied_nci_eq_types.h"

typedef unsigned int vied_nci_eq_recv_port_t;

VIED_NCI_EQ_STORAGE_CLASS_H vied_nci_eq_recv_port_t
vied_nci_eq_recv_port_open(vied_device_id id);

VIED_NCI_EQ_STORAGE_CLASS_H unsigned int
vied_nci_eq_all_available(vied_nci_eq_recv_port_t port);

VIED_NCI_EQ_STORAGE_CLASS_H unsigned int
vied_nci_eq_available(vied_nci_eq_recv_port_t port, unsigned int queue);

VIED_NCI_EQ_STORAGE_CLASS_H vied_nci_eq_token_t
vied_nci_eq_recv(vied_nci_eq_recv_port_t port, unsigned int queue);

// move this to eq_token.h?
VIED_NCI_EQ_STORAGE_CLASS_H
vied_nci_eq_msg_t  vied_nci_eq_get_msg(vied_nci_eq_token_t token);
VIED_NCI_EQ_STORAGE_CLASS_H
vied_nci_eq_pid_t  vied_nci_eq_get_pid(vied_nci_eq_token_t token);
VIED_NCI_EQ_STORAGE_CLASS_H
vied_nci_eq_sid_t  vied_nci_eq_get_sid(vied_nci_eq_token_t token);

#ifdef __INLINE_VIED_NCI_EQ__
#include "vied_nci_eq_recv_inline.h"
#endif

#endif

