/*
 * vied_nci_acc_host_reg_access.c
 *
 *  Created on: Feb 5, 2014
 *      Author: tversteg
 */

#include "vied_nci_acc_storage_class.h"
#include "misc_support.h"
#include "print_support.h"

#ifdef PSYS
  #include "psys.h"
  #include "psys_tb.h"
  #include "ISA_Cluster_api.h"

  #include "vied_nci_acc_psys_local_defs.h"
  #include "vied_nci_acc_psys_defs.h"

  #define SYS_ID psys0
#else
  #include "isys.h"
  #include "isys_tb.h"

  #include "isys_info_bits.h"
  #include "system_defs.h"

  #include "vied_nci_acc_isys_local_defs.h"
  #include "vied_nci_acc_isys_defs.h"

  #define SYS_ID isys0
#endif


/*
 * vied_nci_acc_device_set_acb_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_device_set_acb_register(unsigned int acc_id, unsigned int reg, unsigned int val)
{
	vied_subsystem_store_32(SYS_ID, (ACC_ACB_path(acc_id) + reg), val);
}


/*
 * vied_nci_acc_device_get_acb_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_acb_register(unsigned int acc_id, unsigned int reg)
{
	return vied_subsystem_load_32(SYS_ID, (ACC_ACB_path(acc_id) + reg));
}

/*
 * vied_nci_acc_device_set_ack_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_device_set_ack_register(unsigned int acc_id, unsigned int reg, unsigned int val)
{
	vied_subsystem_store_32(SYS_ID, (ACC_ack_path(acc_id) + reg), val);
}

/*
 * vied_nci_acc_device_get_ack_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ack_register(unsigned int acc_id, unsigned int reg)
{
	return vied_subsystem_load_32(SYS_ID, (ACC_ack_path(acc_id) + reg));
}

/*
 * vied_nci_acc_device_set_ff_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_device_set_ff_register(unsigned int ff_id, unsigned int reg, unsigned int val)
{
	vied_subsystem_store_32(SYS_ID, (FF_ADDR(ff_id) + FF_PARAMS_BASE_ADDR(ff_id) + reg), val);
}

/*
 * vied_nci_acc_device_get_ff_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_register( unsigned int ff_id, unsigned int reg)
{
	return vied_subsystem_load_32(SYS_ID, (FF_ADDR(ff_id) + FF_PARAMS_BASE_ADDR(ff_id) + reg));
}

/*
 * vied_nci_acc_device_get_ff_params_space_base()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_params_space_base(unsigned int ff_id)
{
	return (FF_ADDR(ff_id) + FF_PARAMS_BASE_ADDR(ff_id));
}

/*
 * vied_nci_acc_device_get_ff_buffer_space_base()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_buffer_space_base(unsigned int ff_id, unsigned int set_id)
{
	return (FF_ADDR(ff_id) + FF_META_DATA_SET_BASE_ADDR(ff_id, set_id));
}

/*
 * vied_nci_acc_device_get_ff_partition_params_space_base()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_partition_params_space_base(unsigned int ff_id, unsigned int set_id)
{
	return (FF_ADDR(ff_id) + FF_PARAM_SET_BASE_ADDR(ff_id, set_id));
	//return vied_subsystem_load_32(SYS_ID, (FF_ADDR(ff_id) + FF_META_DATA_SET_BASE_ADDR(ff_id, set_id) + reg));
}

/*
 * vied_nci_acc_device_get_ff_params_space_end()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_params_space_end(unsigned int ff_id)
{
	return (FF_ADDR(ff_id) + FF_PARAMS_END_ADDR(ff_id));
}

/*
 * vied_nci_acc_device_get_ff_buffer_space_end()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_buffer_space_end(unsigned int ff_id, unsigned int set_id)
{
	return (FF_ADDR(ff_id) + FF_META_DATA_SET_END_ADDR(ff_id, set_id));
}

/*
 * vied_nci_acc_device_get_ff_partition_params_space_end()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_partition_params_space_end(unsigned int ff_id, unsigned int set_id)
{
	return (FF_ADDR(ff_id) + FF_PARAM_SET_END_ADDR(ff_id, set_id));
}

/*
 * vied_nci_acc_device_set_ff_part_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_device_set_ff_part_register(unsigned int ff_id, unsigned int set_id, unsigned int reg, unsigned int val)
{
	vied_subsystem_store_32(SYS_ID, (FF_ADDR(ff_id) + FF_PARAM_SET_BASE_ADDR(ff_id, set_id) + reg), val);
}

/*
 * vied_nci_acc_device_get_ff_part_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_part_register(unsigned int ff_id, unsigned int set_id, unsigned int reg)
{
	return vied_subsystem_load_32(SYS_ID, (FF_ADDR(ff_id) + FF_PARAM_SET_BASE_ADDR(ff_id, set_id) + reg));
}

/*
 * vied_nci_acc_device_get_ff_meta_data_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
unsigned int vied_nci_acc_device_get_ff_meta_data_register( unsigned int ff_id, unsigned int set_id, unsigned int reg)
{
PRINT("%x\n", vied_subsystem_load_32(SYS_ID, (FF_ADDR(ff_id) + FF_META_DATA_SET_BASE_ADDR(ff_id, set_id) + reg)));
	return vied_subsystem_load_32(SYS_ID, (FF_ADDR(ff_id) + FF_META_DATA_SET_BASE_ADDR(ff_id, set_id) + reg));
}

/*
 * vied_nci_acc_device_set_mux_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_device_set_mux_register(unsigned int acc_id)
{
#ifdef ISYS
	vied_subsystem_store_32(SYS_ID, HOST_IS_A_ISA_GP_REGS_ADDR + ISA_MUX_SEL_ADDR, MUX_SEL(acc_id));
#else
	vied_subsystem_store_32(SYS_ID, HOST_PS_ISLC_ISA_GP_REGS_ADDR + ISA_MUX_SEL_ADDR, MUX_SEL(acc_id));
#endif
}

/*
 * vied_nci_acc_device_set_pif_conv_register()
 */
VIED_NCI_ACC_STORAGE_CLASS_C
void vied_nci_acc_device_set_pif_conv_register(unsigned int acc_id, unsigned int reg, unsigned int val)
{
	NOT_USED(acc_id);
	vied_subsystem_store_32(SYS_ID, reg, val);
}

