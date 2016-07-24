/*
 * vied_nci_s2v_host_reg_access.c
 *
 *  Created on: May 8, 2014
 *     Authors: tversteg
 *              vilic
 */

#include "psys.h"
#include "psys_tb.h"

#include "vied_nci_s2v_local_defs.h"
#include "vied_nci_s2v_defs.h"

#define SYS_ID psys0


/**
 * vied_nci_s2v_set_register()
 */
void vied_nci_s2v_set_register(unsigned int s2v_id, unsigned int reg, unsigned int val)
{
    vied_subsystem_store_32(SYS_ID, (S2V_BASE_ADDR(s2v_id) + reg), val);
}


/**
 * vied_nci_s2v_set_ack_register()
 */
void vied_nci_s2v_set_ack_register(unsigned int s2v_id, unsigned int reg, unsigned int val)
{
    vied_subsystem_store_32(SYS_ID, (S2V_ACK_ADDR(s2v_id) + reg), val);
}

