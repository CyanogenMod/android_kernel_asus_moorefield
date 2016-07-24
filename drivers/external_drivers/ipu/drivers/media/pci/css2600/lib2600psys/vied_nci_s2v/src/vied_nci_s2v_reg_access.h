/*
 * vied_nci_s2v_reg_access.h
 *
 *  Created on: May 8, 2014
 *     Authors: tversteg
 *              vilic
 */

#ifndef VIED_NCI_S2V_REG_ACCESS_H_
#define VIED_NCI_S2V_REG_ACCESS_H_

#include "vied_nci_s2v_storage_class.h"


/**
 * vied_nci_s2v_set_register()
 */
VIED_NCI_S2V_STORAGE_CLASS_H
void vied_nci_s2v_set_register(unsigned int s2v_id, unsigned int reg, unsigned int val);


/**
 * vied_nci_s2v_set_ack_register()
 */
VIED_NCI_S2V_STORAGE_CLASS_H
void vied_nci_s2v_set_ack_register(unsigned int s2v_id, unsigned int reg, unsigned int val);

#endif /* VIED_NCI_S2V_REG_ACCESS_H_ */

