/*
 * vied_nci_s2v_cell_reg_access.h
 *
 *  Created on: May 8, 2014
 *     Authors: tversteg
 *              vilic
 */

#include <hive/support.h>
#include "vied_nci_s2v_storage_class.h"

#include "vied_nci_s2v_device_address.h"


/**
 * s2v_store_cmem()
 */
static inline void s2v_store_cmem(unsigned int address, unsigned int value)
{
    int MEM(cmem) *p;
    p = (int MEM(cmem) *)address;
    *p = value;
}

/**
 * vied_nci_s2v_set_register()
 */
VIED_NCI_S2V_STORAGE_CLASS_C
void vied_nci_s2v_set_register(unsigned int s2v_id, unsigned int reg, unsigned int val)
{
    /* Avoid compiler warnings */
    (void*)STR2VEC_IP_ACK[0];

    s2v_store_cmem((STR2VEC_IP_BASE[s2v_id] + reg), val);
}


/**
 * vied_nci_s2v_set_ack_register()
 */
VIED_NCI_S2V_STORAGE_CLASS_C
void vied_nci_s2v_set_ack_register(unsigned int s2v_id, unsigned int reg, unsigned int val)
{
    s2v_store_cmem((STR2VEC_IP_ACK[s2v_id] + reg), val);
}

