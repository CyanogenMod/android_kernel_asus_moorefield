/*
 * vied_nci_acc_reg_access.h
 *
 *  Created on: Feb 5, 2014
 *      Author: tversteg
 */

#ifndef VIED_NCI_ACC_REG_ACCESS_H_
#define VIED_NCI_ACC_REG_ACCESS_H_


#include "vied_nci_acc_storage_class.h"

/*
 * vied_nci_acc_device_set_acb_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_device_set_acb_register(unsigned int acc_id, unsigned int reg, unsigned int val);


/*
 * vied_nci_acc_device_get_acb_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_acb_register(unsigned int acc_id, unsigned int reg);

/*
 * vied_nci_acc_device_set_ack_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_device_set_ack_register(unsigned int acc_id, unsigned int reg, unsigned int val);

/*
 * vied_nci_acc_device_get_ack_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ack_register(unsigned int acc_id, unsigned int reg);

/*
 * vied_nci_acc_device_set_ff_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_device_set_ff_register(unsigned int ff_id, unsigned int reg, unsigned int val);

/*
 * vied_nci_acc_device_get_ff_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_register( unsigned int ff_id, unsigned int reg);


/*
 * vied_nci_acc_device_get_ff_params_space_base()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_params_space_base(unsigned int ff_id);


/*
 * vied_nci_acc_device_get_ff_buffer_space_base()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_buffer_space_base(unsigned int ff_id, unsigned int set_id);


/*
 * vied_nci_acc_device_get_ff_partition_params_space_base()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_partition_params_space_base(unsigned int ff_id, unsigned int set_id);


/*
 * vied_nci_acc_device_get_ff_params_space_end()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_params_space_end(unsigned int ff_id);


/*
 * vied_nci_acc_device_get_ff_buffer_space_end()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_buffer_space_end(unsigned int ff_id, unsigned int set_id);


/*
 * vied_nci_acc_device_get_ff_partition_params_space_end()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_partition_params_space_end(unsigned int ff_id, unsigned int set_id);


/*
 * vied_nci_acc_device_set_ff_part_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_device_set_ff_part_register(unsigned int ff_id, unsigned int set_id, unsigned int reg, unsigned int val);


/*
 * vied_nci_acc_device_get_ff_part_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_part_register(unsigned int ff_id, unsigned int set_id, unsigned int reg);


/*
 * vied_nci_acc_device_get_ff_meta_data_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
unsigned int vied_nci_acc_device_get_ff_meta_data_register( unsigned int ff_id, unsigned int set_id, unsigned int reg);


/*
 * vied_nci_acc_device_set_mux_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_device_set_mux_register(unsigned int acc_id);

/*
 * vied_nci_acc_device_set_pif_conv_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_H
void vied_nci_acc_device_set_pif_conv_register(unsigned int acc_id, unsigned int reg, unsigned int val);

#endif /* VIED_NCI_ACC_REG_ACCESS_H_ */

