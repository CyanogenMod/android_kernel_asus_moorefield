/*
 * vied_nci_s2v.h
 *
 *  Created on: May 8, 2014
 *     Authors: mmarkov1
 *              vilic
 */

#ifndef __VIED_NCI_S2V_H_INCLUDED__
#define __VIED_NCI_S2V_H_INCLUDED__

#include <hrt/api.h>
#include <type_support.h>
#include "vied_nci_s2v_defs.h"
#include "vied_nci_s2v_storage_class.h"
#include "vied_nci_acc_psys_defs.h"


/* Description : Declarations of S2V NCI functions
*/


VIED_NCI_S2V_STORAGE_CLASS_H
enum vied_nci_s2v_id vied_nci_s2v_open(
	enum vied_nci_s2v_id device_id);


VIED_NCI_S2V_STORAGE_CLASS_H
enum vied_nci_err vied_nci_s2v_close(
	enum vied_nci_s2v_id s2v_handle
);


enum vied_nci_err vied_nci_s2v_config_ack(
	enum vied_nci_s2v_id s2v_handle,
	uint32_t ack_address,
	uint32_t ack_sid,
	uint32_t ack_pid,
	uint32_t ack_msg);

void vied_nci_s2v_config(
	enum vied_nci_s2v_id s2v_handle,
	uint32_t ack_vec_nr,
	uint32_t pxlcmp_per_line,
	uint32_t lines_per_frame,
	uint32_t yuv420_enable,
	uint32_t interleave_enable,
	uint32_t dev_null_enable,
	struct vied_nci_s2v_buf_s *buffers,
	int number_of_buffers);


VIED_NCI_S2V_STORAGE_CLASS_H
void vied_nci_s2v_start(enum vied_nci_s2v_id s2v_handle);


VIED_NCI_S2V_STORAGE_CLASS_H
void vied_nci_s2v_run(enum vied_nci_s2v_id s2v_handle,
	uint32_t nof_vectors);


VIED_NCI_S2V_STORAGE_CLASS_H
void vied_nci_s2v_queue_process_vectors(
	enum vied_nci_s2v_id s2v_handle,
	uint32_t number_of_vectors);


#ifdef _INLINE_VIED_NCI_S2V
#include "vied_nci_s2v_inline.h"
#endif


#endif /* __VIED_NCI_S2V_H_INCLUDED__ */
