#ifndef _VIED_NCI_REG_ACCESS_H_
#define _VIED_NCI_REG_ACCESS_H_

#include "vied_nci_eq_device.h"
#include "vied_nci_eq_storage_class.h"

VIED_NCI_EQ_STORAGE_CLASS_H
void event_queue_ip_reg_store(
    const vied_nci_eq_device_t  dev,
    const unsigned int	       reg,
    const unsigned int		       value);

VIED_NCI_EQ_STORAGE_CLASS_H
unsigned int event_queue_ip_reg_load(
    const vied_nci_eq_device_t  dev,
    const unsigned int              reg);

VIED_NCI_EQ_STORAGE_CLASS_H
void event_queue_op_reg_store(
    const unsigned int	       reg,
    const unsigned int		       value);

VIED_NCI_EQ_STORAGE_CLASS_H
unsigned int
event_queue_op_reg_load(const unsigned int reg);

#ifdef __INLINE_VIED_NCI_EQ__
#include "vied_nci_eq_cell_reg_access.h"
#endif

#endif //_VIED_NCI_REG_ACCESS_H_
