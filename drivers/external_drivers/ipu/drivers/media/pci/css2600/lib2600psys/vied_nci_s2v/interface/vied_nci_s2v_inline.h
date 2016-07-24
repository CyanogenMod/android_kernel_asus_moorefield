/*
 * vied_nci_s2v_inline.h
 *
 *  Created on: May 8, 2014
 *     Authors: vilic
 */

#ifndef __VIED_NCI_S2V_INLINE_H_INCLUDED__
#define __VIED_NCI_S2V_INLINE_H_INCLUDED__

#include <hrt/api.h>
#include <str_to_vec_v2_2_defs.h>
#include <type_support.h>

#include "vied_nci_s2v_defs.h"
#include "vied_nci_s2v_local_defs.h"
#include "vied_nci_s2v_storage_class.h"
#include "vied_nci_s2v_reg_access.h"
#ifdef _VIED_NCI_S2V_ON_CELL
#include "vied_nci_s2v_cell_reg_access.h"
#endif

#ifndef NOT_USED
#define NOT_USED(a) ((a) = (a))
#endif


/* Description : Declarations of S2V NCI functions
*/

VIED_NCI_S2V_STORAGE_CLASS_C
enum vied_nci_s2v_id vied_nci_s2v_open(enum vied_nci_s2v_id device_id)
{
	return (enum vied_nci_s2v_id)device_id;
}

VIED_NCI_S2V_STORAGE_CLASS_C
enum vied_nci_err vied_nci_s2v_close(enum vied_nci_s2v_id s2v_handle)
{
	NOT_USED(s2v_handle);
	return VIED_NCI_SUCCESS;
}

VIED_NCI_S2V_STORAGE_CLASS_C
void vied_nci_s2v_start(enum vied_nci_s2v_id s2v_handle)
{
	enum vied_nci_s2v_id s2v_id = (enum vied_nci_s2v_id)s2v_handle;

    vied_nci_s2v_set_register (s2v_id,
		(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_CMD),
		_STR_TO_VEC_V2_2_CMD_INIT);
}

VIED_NCI_S2V_STORAGE_CLASS_C
void vied_nci_s2v_run(enum vied_nci_s2v_id s2v_handle,
	uint32_t nof_vectors)
{
	enum vied_nci_s2v_id s2v_id = (enum vied_nci_s2v_id)s2v_handle;

	vied_nci_s2v_set_register(s2v_id,
		(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_CMD),
		_STR_TO_VEC_V2_2_CMD_PROC_N_VEC + (nof_vectors << 16));

	return;
}

#endif /* __VIED_NCI_S2V_INLINE_H_INCLUDED__ */

