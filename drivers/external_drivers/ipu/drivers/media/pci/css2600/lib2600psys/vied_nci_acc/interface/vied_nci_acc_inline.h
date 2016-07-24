/*
 * vied_nci_acc_inline.h
 *
 *  Created on: May 8, 2014
 *     Authors: vilic
 */

#ifndef __VIED_NCI_ACC_INLINE_H_INCLUDED__
#define __VIED_NCI_ACC_INLINE_H_INCLUDED__

#include <hrt/api.h>
#include <ga_acb_api.h>
#include <type_support.h>
#include "misc_support.h"

#ifdef PSYS
  #include "vied_nci_acc_psys_defs.h"
  #include "vied_nci_acc_psys_local_defs.h"
#else
  #include "vied_nci_acc_isys_defs.h"
  #include "vied_nci_acc_isys_local_defs.h"
#endif

#include "vied_nci_acc_storage_class.h"
#include "vied_nci_acc_reg_access.h"
#ifdef _VIED_NCI_ACC_ON_CELL
  #include "vied_nci_acc_cell_reg_access.h"
#endif

/* Description : Declarations of NCI functions used both for PSYS and ISYS clusters...
*/


/********************************************************/
/*             Load time interface                      */
/********************************************************/

VIED_NCI_ACC_STORAGE_CLASS_C
enum vied_nci_acc_id vied_nci_acc_open (enum vied_nci_acc_id device_id)
{
    return device_id;
}

VIED_NCI_ACC_STORAGE_CLASS_C
enum vied_nci_err vied_nci_acc_close (enum vied_nci_acc_id acc_handle)
{
    NOT_USED(acc_handle);
    return VIED_NCI_SUCCESS;
}


/********************************************************/
/*        Configuration time interface                  */
/********************************************************/

VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_queue_idle_state( enum vied_nci_acc_id acc_handle)
{
    vied_nci_acc_device_set_acb_register (acc_handle, GA_ACB_CMD_FIFO_TAIL_ADDR, ACC_INIT_CMD_ID);

    return;
}


/********************************************************/
/*                Run time interface                    */
/********************************************************/

VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_queue_process_fragment( enum vied_nci_acc_id acc_handle, uint32_t fragment_part_height, uint32_t part_param_set_id)
{
    vied_nci_acc_device_set_acb_register (acc_handle, GA_ACB_CMD_FIFO_TAIL_ADDR, ((fragment_part_height << 16) | (part_param_set_id << 8) | ACC_PROC_N_LINES_CMD_ID));

    return;
}


#endif /* __VIED_NCI_ACC_INLINE_H_INCLUDED__ */

