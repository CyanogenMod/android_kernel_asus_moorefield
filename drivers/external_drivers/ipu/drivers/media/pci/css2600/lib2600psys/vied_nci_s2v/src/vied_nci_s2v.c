/*
 * vied_nci_s2v.c
 *
 *  Created on: May 8, 2014
 *     Authors: mmarkov1
 *              vilic
 */

#include <hrt/api.h>
#include <str_to_vec_v2_2_defs.h>
#include <type_support.h>

#include "vied_nci_acc_psys_defs.h"
#include "vied_nci_s2v_defs.h"
#include "vied_nci_s2v_local_defs.h"
#include "vied_nci_s2v_storage_class.h"
#include "vied_nci_s2v_reg_access.h"

#ifndef _INLINE_VIED_NCI_S2V
#include "vied_nci_s2v_inline.h"
#else
#ifdef _VIED_NCI_S2V_ON_CELL
#include "vied_nci_s2v_cell_reg_access.h"
#endif
#endif


void vied_nci_s2v_config(
	enum vied_nci_s2v_id s2v_handle,
	uint32_t ack_vec_nr,
	uint32_t pxlcmp_per_line,
	uint32_t lines_per_frame,
	uint32_t yuv420_enable,
	uint32_t interleave_enable,
	uint32_t dev_null_enable,
	struct vied_nci_s2v_buf_s *buffers,
	int number_of_buffers)
{
	int i;

	enum vied_nci_s2v_id s2v_id = (enum vied_nci_s2v_id)s2v_handle;
	uint32_t offset = _STR_TO_VEC_V2_2_0_ST_ADDR;

	vied_nci_s2v_set_register(s2v_id,
		(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_ACK_K_VEC_REG),
		ack_vec_nr);
	vied_nci_s2v_set_register(s2v_id,
		(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_PXL_LINE_REG),
		pxlcmp_per_line);
	vied_nci_s2v_set_register(s2v_id,
		(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_LINE_FRAME_REG),
		lines_per_frame);
	if ((s2v_id == VIED_NCI_S2V_YUV1_ID) || (s2v_id == VIED_NCI_S2V_YUV2_ID)) {
		vied_nci_s2v_set_register(s2v_id,
			(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_YUV420_EN_REG),
			yuv420_enable);
	}
	if ((s2v_id == VIED_NCI_S2V_BAYER1_ID) || (s2v_id == VIED_NCI_S2V_BAYER2_ID)) {
		vied_nci_s2v_set_register(s2v_id,
			(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_INTERLEAVE_EN_REG),
			interleave_enable);
	}
	vied_nci_s2v_set_register(s2v_id,
		(_STR_TO_VEC_V2_2_REG_ALIGN * _STR_TO_VEC_V2_2_DEV_NULL_EN_REG),
		dev_null_enable);

	for (i = 0; i < number_of_buffers; i++) {
		vied_nci_s2v_set_register(s2v_id,
			(_STR_TO_VEC_V2_2_REG_ALIGN * offset++),
			buffers[i].start_address);
		vied_nci_s2v_set_register(s2v_id,
			(_STR_TO_VEC_V2_2_REG_ALIGN * offset++),
			buffers[i].end_address);
		vied_nci_s2v_set_register(s2v_id,
			(_STR_TO_VEC_V2_2_REG_ALIGN * offset++),
			buffers[i].offset);
		vied_nci_s2v_set_register(s2v_id,
			(_STR_TO_VEC_V2_2_REG_ALIGN * offset++),
			buffers[i].stride);
	}


	return;
}


enum vied_nci_err vied_nci_s2v_config_ack(
	enum vied_nci_s2v_id s2v_handle,
	uint32_t ack_address,
	uint32_t ack_sid,
	uint32_t ack_pid,
	uint32_t ack_msg)
{
	vied_nci_s2v_set_ack_register(s2v_handle, VIED_NCI_S2V_ACK_ADDR_ADDR, (ack_address));
	vied_nci_s2v_set_ack_register(s2v_handle, VIED_NCI_S2V_ACK_CMD_ADDR,
		((ack_sid << 26) | (ack_pid << 20) | ack_msg));

	return VIED_NCI_SUCCESS;
}
