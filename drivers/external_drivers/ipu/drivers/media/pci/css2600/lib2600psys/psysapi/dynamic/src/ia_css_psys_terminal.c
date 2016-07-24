
#include <ia_css_psys_terminal.h>

#include <ia_css_psys_process_types.h>
#include <ia_css_psys_terminal_manifest.h>

#include <ia_css_program_group_data.h>
#include <ia_css_program_group_param.h>

#include <ia_css_psys_process_group.h>

#include <type_support.h>
#include <error_support.h>
#include <print_support.h>
#include <misc_support.h>

#include <cpu_mem_support.h>
/*
 * Note: "frame" should be attached by reference rather than the buffer it holds
 * It is not possible to propagate the empty/full status otherwise
 */
struct ia_css_terminal_s {
	uint16_t								size;											/**< Size of this structure */
	ia_css_terminal_type_t					terminal_type;									/**< Type {in, out, state, ...} */
	ia_css_process_group_t					*parent;										/**< Reference to the process group */
};

/*
 * The (data) terminal can be attached to a buffer or a stream. The stream interface
 * is not necessarily limited to strict in-order access. For a stream the restriction
 * is that contrary to a buffer it cannot be addressed directly, i.e. it behaves as a
 * port, but it may support stream_pos() and/or seek() operations
 */
struct ia_css_data_terminal_s {
	ia_css_terminal_t						base;											/**< Data terminal base */
	ia_css_frame_format_type_t				frame_format_type;								/**< Indicates if this is a generic type or inbuild with variable size descriptor */
	ia_css_connection_type_t				connection_type;								/**< Connection {buffer, stream, ...} */
/*	uint16_t								fragment_count; */								/**< Number of data fragments (this is derived per subgraph from the parent process group) */
	uint8_t									subgraph_id;									/**< Indicate to which subgraph this terminal belongs for common constraints */
	ia_css_frame_descriptor_t				frame_descriptor;								/**< Properties of the data attached to the terminal */
	ia_css_frame_t							frame;											/**< Data buffer handle attached to the terminal */
	ia_css_stream_t							*stream;										/**< (exclusive) Data stream handle attached to the terminal if the data is sourced over a device port */
	ia_css_fragment_descriptor_t			*fragment_descriptor;							/**< Array[fragment_count] (fragment_count being equal for all terminals in a subgraph) of fragment descriptors */
};

/* This terminal may optionally hold a queue of parameter handles for late binding */
struct ia_css_param_terminal_s {
	ia_css_terminal_t						base;											/**< Parameter terminal base */
/*	uint8_t									param_queue_size; */							/**< Number of parameter buffer handles that can be queued for late binding */
	ia_css_kernel_param_descriptor_t		*param_descriptor;								/**< Properties of the parameters attached to the terminal */
	ia_css_kernel_param_t					param;											/**< Parameter buffer handle attached to the terminal */
};

size_t ia_css_sizeof_terminal(
	const ia_css_terminal_manifest_t		*manifest,
	const ia_css_program_group_param_t		*param)
{
	size_t		size = 0;

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);

	if (ia_css_is_terminal_manifest_parameter_terminal(manifest)) {
		size += sizeof(ia_css_param_terminal_t);
	} else if (ia_css_is_terminal_manifest_data_terminal(manifest)) {
		size += sizeof(ia_css_data_terminal_t);
	}
EXIT:
	return size;
}

ia_css_terminal_t *ia_css_terminal_create(
	const ia_css_terminal_manifest_t		*manifest,
	const ia_css_terminal_param_t			*terminal_param)
{
	size_t				size = 0;
	size_t				tmp_size = 0;
	ia_css_terminal_t	*terminal = NULL;
	uint16_t			fragment_count;
	int 			i;
	int retval = -1;

	ia_css_program_group_param_t *param = ia_css_terminal_param_get_parent(terminal_param);
	fragment_count = ia_css_program_group_param_get_fragment_count(param);

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);

	if (ia_css_is_terminal_manifest_data_terminal(manifest) == true) {
		tmp_size = sizeof(ia_css_data_terminal_t);
	} else if (ia_css_is_terminal_manifest_parameter_terminal(manifest) == true) {
		tmp_size = sizeof(ia_css_param_terminal_t);
	} else {
		verifexit(false, EFAULT);
	}
	terminal = (ia_css_terminal_t *)ia_css_cpu_mem_alloc(tmp_size);
	verifexit(terminal != NULL, EINVAL);
	ia_css_cpu_mem_set_zero(terminal, tmp_size);
	size += tmp_size;

	terminal->size = ia_css_sizeof_terminal(manifest, param);
	verifexit(terminal->size == size, EINVAL);
	verifexit(ia_css_terminal_set_type(terminal, ia_css_terminal_manifest_get_type(manifest)) == 0, EINVAL);

	verifexit(ia_css_terminal_set_buffer(terminal, VIED_NULL) == 0, EINVAL);

	if (ia_css_is_terminal_manifest_data_terminal(manifest) == true) {
		ia_css_data_terminal_t *dterminal = (ia_css_data_terminal_t *)terminal;
		ia_css_frame_t	*frame = ia_css_data_terminal_get_frame(dterminal);

		verifexit(frame != NULL, EINVAL);
		verifexit(ia_css_frame_set_buffer_state(frame, IA_CSS_BUFFER_NULL) == 0, EINVAL);

		tmp_size = fragment_count * sizeof(ia_css_fragment_descriptor_t);
		dterminal->fragment_descriptor = (ia_css_fragment_descriptor_t *)
						ia_css_cpu_mem_alloc(tmp_size);
		verifexit(dterminal->fragment_descriptor != NULL, ENOBUFS);
		ia_css_cpu_mem_set_zero(dterminal->fragment_descriptor, tmp_size);
		size += tmp_size;
		/* some terminal and fragment initialization */
		dterminal->frame_descriptor.frame_format_type = terminal_param->frame_format_type;
		for (i = 0; i < IA_CSS_N_DATA_DIMENSION; i++) {
			dterminal->frame_descriptor.dimension[i] = terminal_param->dimensions[i];
		}
		dterminal->frame_descriptor.stride[0] = terminal_param->stride;
		dterminal->frame_descriptor.bpp = terminal_param->bpp;
		/* initial solution for single fragment initialization */
		/* TODO: where to get the fragment description params from??? */
		{
			ia_css_fragment_descriptor_t *fragment_descriptor = (ia_css_fragment_descriptor_t *)(&(dterminal->fragment_descriptor[0]));
			fragment_descriptor->offset[0] = terminal_param->offset;
			for (i = 0; i < IA_CSS_N_DATA_DIMENSION; i++) {
				fragment_descriptor->dimension[i] = terminal_param->fragment_dimensions[i];
			}
		}
		/* end fragment stuff */
	} else if (ia_css_is_terminal_manifest_parameter_terminal(manifest) == true) {
		uint16_t section_count = ia_css_param_terminal_compute_section_count(manifest, NULL);
		if (section_count != 0) {
			/* Do nothing if section_count is 0.
			 * FIXME: change this to error check, as we add sections and
			 * if broxton also should have non-zero section count*/
			ia_css_param_terminal_t *pterminal = (ia_css_param_terminal_t *)terminal;
			tmp_size = section_count * sizeof(ia_css_kernel_param_descriptor_t);
			pterminal->param_descriptor = (ia_css_kernel_param_descriptor_t *)
						ia_css_cpu_mem_alloc(tmp_size);
			verifexit(pterminal->param_descriptor != NULL, ENOBUFS);
			ia_css_cpu_mem_set_zero(pterminal->param_descriptor, tmp_size);
			size += tmp_size;
		}
	} else {
		verifexit(false, EFAULT);;
	}

	retval = 0;
EXIT:
	if (retval!= 0) {
		terminal = ia_css_terminal_destroy(terminal);
	}
	return terminal;
}

ia_css_terminal_t *ia_css_terminal_destroy(
	ia_css_terminal_t						*terminal)
{
	if (terminal == NULL)
		return terminal;

	if (ia_css_is_terminal_data_terminal(terminal)) {
		ia_css_cpu_mem_free((void *)((ia_css_data_terminal_t *)terminal)->fragment_descriptor);
	} else {
		ia_css_cpu_mem_free((void *)((ia_css_param_terminal_t *)terminal)->param_descriptor);
	}
	ia_css_cpu_mem_free((void *)terminal);
	terminal = NULL;
	return terminal;
}

int ia_css_terminal_print(
	const ia_css_terminal_t					*terminal,
	void									*fid)
{
	int	retval = -1;
	int	i;
	bool is_data = false;
	uint16_t	fragment_count;

	NOT_USED(fid);
	verifexit(terminal != NULL, EINVAL);
	verifexit(fragment_count != 0, EINVAL);

	is_data = ia_css_is_terminal_data_terminal(terminal);
	PRINT("ia_css_terminal_print\n");
	PRINT("sizeof(terminal) = %d\n",(int)ia_css_terminal_get_size(terminal));
	PRINT("typeof(terminal) = %d\n",(int)ia_css_terminal_get_type(terminal));
	/*	PRINT("typeof(terminal) = %s\n",(int)ia_css_terminal_type_string(ia_css_terminal_get_type(terminal))); */
	PRINT("parent(terminal) = %p\n",(void *)ia_css_terminal_get_parent(terminal));

	if (is_data) {
		ia_css_data_terminal_t *dterminal = (ia_css_data_terminal_t *)terminal;
		fragment_count = ia_css_data_terminal_get_fragment_count(dterminal);
		retval = ia_css_frame_descriptor_print(ia_css_data_terminal_get_frame_descriptor(dterminal), fid);
		verifexit(retval == 0, EINVAL);
		retval = ia_css_frame_print(ia_css_data_terminal_get_frame(dterminal), fid);
		verifexit(retval == 0, EINVAL);
		for (i = 0; i < (int)fragment_count; i++) {
			retval = ia_css_fragment_descriptor_print(ia_css_data_terminal_get_fragment_descriptor(dterminal, i), fid);
			verifexit(retval == 0, EINVAL);
		}
	} else {
		/*TODO: FIXME print param terminal sections.*/
	}

	retval = 0;
EXIT:
	return retval;
}

bool ia_css_is_terminal_input(
	const ia_css_terminal_t					*terminal)
{
	bool is_input = false;
	ia_css_terminal_type_t	terminal_type = ia_css_terminal_get_type(terminal);

	verifexit(terminal != NULL, EINVAL);

	switch (terminal_type) {
	case IA_CSS_TERMINAL_TYPE_DATA_IN:			/* Fall through */
	case IA_CSS_TERMINAL_TYPE_STATE_IN:			/* Fall through */
	case IA_CSS_TERMINAL_TYPE_PARAM_STREAM:		/* Fall through */
	case IA_CSS_TERMINAL_TYPE_PARAM_CACHED:
		is_input = true;
		break;
	case IA_CSS_TERMINAL_TYPE_DATA_OUT:			/* Fall through */
	case IA_CSS_TERMINAL_TYPE_STATE_OUT:
		is_input = false;
		break;
	default:
		verifexit(false, EINVAL);
		break;
	}

EXIT:
	return is_input;
}

size_t ia_css_terminal_get_size(
	const ia_css_terminal_t					*terminal)
{
	size_t	size = 0;

	if (terminal != NULL) {
		size = terminal->size;
	}

	return size;
}

ia_css_terminal_type_t ia_css_terminal_get_type(
	const ia_css_terminal_t					*terminal)
{
	ia_css_terminal_type_t	terminal_type = IA_CSS_N_TERMINAL_TYPES;

	if (terminal != NULL) {
		terminal_type = terminal->terminal_type;
	}

	return terminal_type;
}

int ia_css_terminal_set_type(
	ia_css_terminal_t						*terminal,
	const ia_css_terminal_type_t			terminal_type)
{
	int	retval = -1;

	verifexit(terminal != NULL, EINVAL);
	terminal->terminal_type = terminal_type;

	retval = 0;
EXIT:
	return retval;
}

ia_css_connection_type_t ia_css_data_terminal_get_connection_type(
	const ia_css_data_terminal_t			*dterminal)
{
	ia_css_connection_type_t	connection_type = IA_CSS_N_CONNECTION_TYPES;

	verifexit(dterminal != NULL, EINVAL);
	connection_type = dterminal->connection_type;
EXIT:
	return connection_type;
}

int ia_css_data_terminal_set_connection_type(
	ia_css_data_terminal_t			*dterminal,
	const ia_css_connection_type_t			connection_type)
{
	int	retval = -1;

	verifexit(dterminal != NULL, EINVAL);
	dterminal->connection_type = connection_type;

	retval = 0;
EXIT:
	return retval;
}

ia_css_process_group_t *ia_css_terminal_get_parent(
	const ia_css_terminal_t					*terminal)
{
	ia_css_process_group_t	*parent = NULL;

	verifexit(terminal != NULL, EINVAL);

	parent = terminal->parent;
EXIT:
	return parent;
}

int ia_css_terminal_set_parent(
	ia_css_terminal_t					*terminal,
	ia_css_process_group_t					*parent)
{
	int	retval = -1;

	verifexit(terminal != NULL, EINVAL);
	verifexit(parent != NULL, EINVAL);

	terminal->parent = parent;

	retval = 0;
EXIT:
	return retval;
}

ia_css_frame_t *ia_css_data_terminal_get_frame(
	const ia_css_data_terminal_t		*dterminal)
{
	ia_css_frame_t	*frame = NULL;

	verifexit(dterminal != NULL, EINVAL);

	frame = (ia_css_frame_t	*)(&(dterminal->frame));
EXIT:
	return frame;
}

ia_css_frame_descriptor_t *ia_css_data_terminal_get_frame_descriptor(
	const ia_css_data_terminal_t		*dterminal)
{
	ia_css_frame_descriptor_t	*frame_descriptor = NULL;

	verifexit(dterminal != NULL, EINVAL);

	frame_descriptor = (ia_css_frame_descriptor_t *)(&(dterminal->frame_descriptor));
EXIT:
	return frame_descriptor;
}

ia_css_fragment_descriptor_t *ia_css_data_terminal_get_fragment_descriptor(
	const ia_css_data_terminal_t				*dterminal,
	const unsigned int					fragment_index)
{
	ia_css_fragment_descriptor_t	*fragment_descriptor = NULL;
	uint16_t				fragment_count = ia_css_data_terminal_get_fragment_count(dterminal);

	verifexit(dterminal != NULL, EINVAL);
	verifexit(fragment_count != 0, EINVAL);

	verifexit(fragment_index < fragment_count, EINVAL);

	fragment_descriptor = (ia_css_fragment_descriptor_t *)(&(dterminal->fragment_descriptor[fragment_index]));
EXIT:
	return fragment_descriptor;
}

uint16_t ia_css_data_terminal_get_fragment_count(
	const ia_css_data_terminal_t					*dterminal)
{
	ia_css_process_group_t			*parent = ia_css_terminal_get_parent((ia_css_terminal_t *)dterminal);
	uint16_t						fragment_count = 0;

	verifexit(dterminal != NULL, EINVAL);
	verifexit(parent != NULL, EINVAL);

	fragment_count = ia_css_process_group_get_fragment_count(parent);
EXIT:
	return fragment_count;
}

bool ia_css_is_terminal_parameter_terminal(
	const ia_css_terminal_t					*terminal)
{
/* will return an error value on error */
	ia_css_terminal_type_t	terminal_type = ia_css_terminal_get_type(terminal);
	return (terminal_type == IA_CSS_TERMINAL_TYPE_PARAM_CACHED);
}

bool ia_css_is_terminal_data_terminal(
	const ia_css_terminal_t					*terminal)
{
/* will return an error value on error */
	ia_css_terminal_type_t	terminal_type = ia_css_terminal_get_type(terminal);
	return ((terminal_type != IA_CSS_TERMINAL_TYPE_PARAM_CACHED) &&
			(terminal_type < IA_CSS_N_TERMINAL_TYPES));
}

uint16_t ia_css_param_terminal_compute_section_count(
	const ia_css_terminal_manifest_t		*manifest,
	const ia_css_program_group_param_t		*param) /* Delete 2nd argument*/
{
	uint16_t	section_count = 0;

	NOT_USED(param);

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);
	section_count = ia_css_param_terminal_manifest_get_section_count(
						(ia_css_param_terminal_manifest_t*)manifest);
EXIT:
	return section_count;
}

uint8_t ia_css_data_terminal_compute_plane_count(
	const ia_css_terminal_manifest_t		*manifest,
	const ia_css_program_group_param_t		*param)
{
	uint8_t	plane_count = 1;

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);
	/* TODO: Implementation Missing*/

EXIT:
	return plane_count;
}

ia_css_kernel_param_t *ia_css_param_terminal_get_kernel_param(
	const ia_css_param_terminal_t		*pterminal)
{
	ia_css_kernel_param_t	*kernel_param = NULL;

	verifexit(pterminal != NULL, EINVAL);

	kernel_param = (ia_css_kernel_param_t*)(&(pterminal->param));
EXIT:
	return kernel_param;
}

vied_vaddress_t  ia_css_terminal_get_buffer(
		ia_css_terminal_t *terminal)
{
	vied_vaddress_t buffer = VIED_NULL;
	if (ia_css_is_terminal_data_terminal(terminal)) {
		ia_css_frame_t			*frame = ia_css_data_terminal_get_frame((ia_css_data_terminal_t *)terminal);
		verifexit(frame != NULL, EINVAL);
		buffer = ia_css_frame_get_buffer(frame);
	} else if (ia_css_is_terminal_parameter_terminal(terminal)) {
		ia_css_kernel_param_t	*kernel_param = ia_css_param_terminal_get_kernel_param((ia_css_param_terminal_t *)terminal);
		verifexit(kernel_param != NULL, EINVAL);
		buffer = ia_css_kernel_param_get_buffer(kernel_param);
	}
EXIT:
	return buffer;
}


int ia_css_terminal_set_buffer(ia_css_terminal_t *terminal,
				vied_vaddress_t buffer)
{
	int retval = -1;

	if (ia_css_is_terminal_data_terminal(terminal) == true) {
		/* Currently using Frames inside data terminal , TODO: start directly using data.*/
		ia_css_data_terminal_t *dterminal = (ia_css_data_terminal_t *)terminal;
		ia_css_frame_t *frame = ia_css_data_terminal_get_frame(dterminal);
		verifexit(frame != NULL, EINVAL);
		retval = ia_css_frame_set_buffer(frame, buffer);
		verifexit(retval == 0, EINVAL);
	} else if (ia_css_is_terminal_parameter_terminal(terminal) == true) {
		ia_css_param_terminal_t *pterminal = (ia_css_param_terminal_t *)terminal;
		ia_css_kernel_param_t *kernel_param = ia_css_param_terminal_get_kernel_param(pterminal);
		retval = ia_css_kernel_param_set_buffer(kernel_param, buffer);
		verifexit(retval == 0, EINVAL);
	} else {
		return retval;
	}

	retval = 0;
EXIT:
	return retval;
}
