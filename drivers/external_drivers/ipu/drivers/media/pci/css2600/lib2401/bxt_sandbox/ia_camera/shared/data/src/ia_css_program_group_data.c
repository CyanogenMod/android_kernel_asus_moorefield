/*
 * Support for Intel Camera Imaging ISP subsystem.
 *
 * Copyright (c) 2010 - 2014 Intel Corporation. All Rights Reserved.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License version
 * 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston, MA
 * 02110-1301, USA.
 *
 */


#include "ia_css_program_group_data.h"

#include <error_support.h>
#include <assert_support.h>
#include <print_support.h>
#include <misc_support.h>

int ia_css_frame_print(
	const ia_css_frame_t					*frame,
	void									*fid)
{
	int		retval = -1;

	NOT_USED(fid);

	verifexit(frame != NULL, EINVAL);

	PRINT("ia_css_frame_print\n");

	PRINT("\tbuffer = %d\n",ia_css_frame_get_buffer(frame));
	PRINT("\tbuffer_state = %d\n",ia_css_frame_get_buffer_state(frame));
	/*	PRINT("\tbuffer_state = %s\n",ia_css_buffer_state_string(ia_css_frame_get_buffer_state(frame))); */
	PRINT("\tpointer_state = %d\n",ia_css_frame_get_pointer_state(frame));
	/*	PRINT("\tpointer_state = %s\n",ia_css_pointer_state_string(ia_css_frame_get_pointer_state(frame))); */
	PRINT("\tdata_bytes = %d\n",frame->data_bytes);

	retval = 0;
EXIT:
	return retval;
}

vied_vaddress_t	ia_css_frame_get_buffer(
	const ia_css_frame_t					*frame)
{
	vied_vaddress_t	buffer = VIED_NULL;

	verifexit(frame != NULL, EINVAL);
	buffer = frame->data;
EXIT:
	return buffer;
}

int ia_css_frame_set_buffer(
	ia_css_frame_t							*frame,
	vied_vaddress_t							buffer)
{
	int	retval = -1;

	verifexit(frame != NULL, EINVAL);
	frame->data = buffer;

	retval = 0;
EXIT:
	return retval;
}

int ia_css_frame_set_data_bytes(
	ia_css_frame_t							*frame,
	unsigned int							size)
{
	int	retval = -1;

	verifexit(frame != NULL, EINVAL);
	frame->data_bytes = size;

	retval = 0;
EXIT:
	return retval;
}

ia_css_buffer_state_t ia_css_frame_get_buffer_state(
	const ia_css_frame_t					*frame)
{
	ia_css_buffer_state_t	buffer_state = IA_CSS_N_BUFFER_STATES;

	verifexit(frame != NULL, EINVAL);
	buffer_state = frame->buffer_state;
EXIT:
	return buffer_state;
}

int ia_css_frame_set_buffer_state(
	ia_css_frame_t							*frame,
	const ia_css_buffer_state_t				buffer_state)
{
	int	retval = -1;

	verifexit(frame != NULL, EINVAL);
	frame->buffer_state = buffer_state;

	retval = 0;
EXIT:
	return retval;
}

ia_css_pointer_state_t ia_css_frame_get_pointer_state(
	const ia_css_frame_t					*frame)
{
	ia_css_pointer_state_t	pointer_state = IA_CSS_N_POINTER_STATES;

	verifexit(frame != NULL, EINVAL);
	pointer_state = frame->pointer_state;
EXIT:
	return pointer_state;
}

int ia_css_frame_set_pointer_state(
	ia_css_frame_t							*frame,
	const ia_css_pointer_state_t			pointer_state)
{
	int	retval = -1;

	verifexit(frame != NULL, EINVAL);
	frame->pointer_state = pointer_state;

	retval = 0;
EXIT:
	return retval;
}

int ia_css_frame_descriptor_print(
	const ia_css_frame_descriptor_t			*frame_descriptor,
	void									*fid)
{
	int	retval = -1;
	int		i;
	uint8_t	frame_plane_count;

	NOT_USED(fid);

	assert(IA_CSS_N_DATA_DIMENSION > 0);

	verifexit(frame_descriptor != NULL, EINVAL);

	PRINT("ia_css_frame_descriptor_print\n");

	PRINT("\tframe_format_type = %d\n",frame_descriptor->frame_format_type);
	/*	PRINT("\tframe_format_type = %s\n",ia_css_frame_format_string(frame_descriptor->frame_format_type)); */

	PRINT("\tbpp = %d\n",frame_descriptor->bpp);
	PRINT("\tbpe = %d\n",frame_descriptor->bpe);

	frame_plane_count = IA_CSS_N_FRAME_PLANES;
	/*	frame_plane_count = ia_css_frame_plane_count(frame_descriptor->frame_format_type); */

	verifexit(frame_plane_count > 0, EINVAL);

	PRINT("\tplane_offsets[%d]: [",frame_plane_count);
	for (i = 0; i < (int)frame_plane_count - 1; i++) {
		PRINT("%4d, ",frame_descriptor->plane_offsets[i]);
	}
	PRINT("%4d]\n",frame_descriptor->plane_offsets[i]);

	PRINT("\tdimension[%d]   = {",IA_CSS_N_DATA_DIMENSION);
	for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
		PRINT("%4d, ",frame_descriptor->dimension[i]);
	}
	PRINT("%4d}\n",frame_descriptor->dimension[i]);

	PRINT("\tstride[%d]  = {",IA_CSS_N_DATA_DIMENSION - 1);
	for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 2; i++) {
		PRINT("%4d, ",frame_descriptor->stride[i]);
	}
	PRINT("%4d}\n",frame_descriptor->stride[i]);

	retval = 0;
EXIT:
	return retval;
}

int ia_css_fragment_descriptor_print(
	const ia_css_fragment_descriptor_t		*fragment_descriptor,
	void									*fid)
{
	int		retval = -1;
	int		i;

	NOT_USED(fid);

	verifexit(fragment_descriptor != NULL, EINVAL);

	PRINT("ia_css_fragment_descriptor_print\n");

	PRINT("\tdimension[%d]   = {",IA_CSS_N_DATA_DIMENSION);
	for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
		PRINT("%4d, ",fragment_descriptor->dimension[i]);
	}
	PRINT("%4d}\n",fragment_descriptor->dimension[i]);

	PRINT("\tindex[%d]  = {",IA_CSS_N_DATA_DIMENSION);
	for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
		PRINT("%4d, ",fragment_descriptor->index[i]);
	}
	PRINT("%4d}\n",fragment_descriptor->index[i]);

	PRINT("\toffset[%d] = {",IA_CSS_N_DATA_DIMENSION);
	for (i = 0; i < (int)IA_CSS_N_DATA_DIMENSION - 1; i++) {
		PRINT("%4d, ",fragment_descriptor->offset[i]);
	}
	PRINT("%4d}\n",fragment_descriptor->offset[i]);

	retval = 0;
EXIT:
	return retval;
}

ia_css_frame_format_bitmap_t ia_css_frame_format_bit_mask(
	const ia_css_frame_format_type_t		frame_format_type)
{
	ia_css_frame_format_bitmap_t	bit_mask = 0;
	if ((frame_format_type < IA_CSS_N_FRAME_FORMAT_TYPES) && (frame_format_type < IA_CSS_FRAME_FORMAT_BITMAP_BITS)) {
		bit_mask = (ia_css_frame_format_bitmap_t)1 << frame_format_type;
	}

	return bit_mask;
}

ia_css_frame_format_bitmap_t ia_css_frame_format_bitmap_clear(void)
{
	return 0;
}

size_t ia_css_sizeof_frame_descriptor(
		const uint8_t			plane_count)
{
	size_t size =0;

	verifexit(plane_count > 0, EINVAL);
	size += sizeof(ia_css_frame_descriptor_t);
	size += plane_count * sizeof(uint32_t);
EXIT:
	return size;
}

size_t ia_css_sizeof_kernel_param_descriptor(
	const uint16_t							section_count)
{
	size_t	size = 0;

/* On a program group with zero parameter sections, there is no parameter terminal */
	verifexit(section_count > 0, EINVAL);

	size += sizeof(ia_css_kernel_param_descriptor_t);

	/* offset values for each section
	 * (ia_css_kernel_param_descriptor_s ->section_offsets[section_count]
	 * )*/
	size += section_count * sizeof(uint32_t);

EXIT:
	return size;
}

vied_vaddress_t	ia_css_kernel_param_get_buffer(
	const ia_css_kernel_param_t				*kernel_param)
{
	vied_vaddress_t	buffer = VIED_NULL;

	verifexit(kernel_param != NULL, EINVAL);
	buffer = kernel_param->buffer;
EXIT:
	return buffer;
}

int ia_css_kernel_param_set_buffer(
	ia_css_kernel_param_t					*kernel_param,
	vied_vaddress_t							buffer)
{
	int	retval = -1;

	verifexit(kernel_param != NULL, EINVAL);
	kernel_param->buffer = buffer;

	retval = 0;
EXIT:
	return retval;
}


size_t ia_css_kernel_param_descriptor_get_size(
	const ia_css_kernel_param_descriptor_t	*kernel_param_descriptor)
{
	size_t	size = 0;

	if (kernel_param_descriptor != NULL) {
		size = kernel_param_descriptor->size;
	}

	return size;
}

uint16_t ia_css_kernel_param_descriptor_get_section_count(
	const ia_css_kernel_param_descriptor_t	*kernel_param_descriptor)
{
	uint16_t	section_count = 0;

	if (kernel_param_descriptor != NULL) {
		section_count = kernel_param_descriptor->section_count;
	}

	return section_count;
}
