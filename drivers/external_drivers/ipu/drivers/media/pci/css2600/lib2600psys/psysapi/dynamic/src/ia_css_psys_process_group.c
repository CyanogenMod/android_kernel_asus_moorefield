
#include <ia_css_psys_process_group.h>
#include "ia_css_psys_process_group_cmd_impl.h"
#include <ia_css_psys_terminal.h>
#include <ia_css_psys_process.h>
#include <ia_css_psys_terminal_manifest.h>
#include <ia_css_psys_program_manifest.h>
#include <ia_css_psys_program_group_manifest.h>

#include <ia_css_kernel_bitmap.h>				/* ia_css_kernel_bitmap_t */

#include <vied_nci_psys_system_global.h>
#include <ia_css_program_group_data.h>
#include <type_support.h>
#include <error_support.h>
#include <print_support.h>
#include <misc_support.h>			/* NOT_USED */

#include "cpu_mem_support.h"

size_t ia_css_sizeof_process_group(
	const ia_css_program_group_manifest_t	*manifest,
	const ia_css_program_group_param_t		*param)
{
	size_t	size = 0;
	int		i;
	uint8_t	process_count, terminal_count;

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);

	process_count = ia_css_process_group_compute_process_count(manifest, param);
	terminal_count = ia_css_process_group_compute_terminal_count(manifest, param);
	verifexit(process_count != 0, EINVAL);
	verifexit(terminal_count != 0, EINVAL);

	size += sizeof(ia_css_process_group_t);

	size += process_count * sizeof(ia_css_process_t *);
	size += terminal_count * sizeof(ia_css_terminal_t *);

/* All functions in the loops below can set errno, thus no need to exit on error, all can fail silently */
	for (i = 0; i < (int)process_count; i++) {
		ia_css_program_manifest_t	*program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, i);
		ia_css_program_param_t		*program_param = ia_css_program_group_param_get_program_param(param, i);
		size += ia_css_sizeof_process(program_manifest, program_param);
	}

	for (i = 0; i < (int)terminal_count; i++) {
		ia_css_terminal_manifest_t	*terminal_manifest = ia_css_program_group_manifest_get_terminal_manifest(manifest, i);
/*		ia_css_terminal_param_t *terminal_param = ia_css_program_group_param_get_terminal_param(param); */
		size += ia_css_sizeof_terminal(terminal_manifest, param);
/*		size += ia_css_sizeof_terminal(terminal_manifest, terminal_param); */
	}

EXIT:
	return size;
}

ia_css_process_group_t *ia_css_process_group_create(
	const ia_css_program_group_manifest_t	*manifest,
	const ia_css_program_group_param_t		*param)
{
	size_t		size = 0;
	size_t		tmp_size = 0;
	int retval = -1, ret;
	int			i;
	ia_css_process_group_t	*process_group = NULL;
	uint8_t		process_count, terminal_count;
	uint16_t	fragment_count;

	/* size_t size = ia_css_sizeof_process_group(manifest, param); */

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);
	verifexit(ia_css_is_program_group_manifest_valid(manifest), EINVAL);

	tmp_size = sizeof(ia_css_process_group_t);
	process_group = (ia_css_process_group_t	*)ia_css_cpu_mem_alloc(tmp_size);
	verifexit(process_group != NULL, EINVAL);
	ia_css_cpu_mem_set_zero(process_group, tmp_size);
	size += tmp_size;

	process_group->state = IA_CSS_PROCESS_GROUP_CREATED;

	fragment_count = ia_css_program_group_param_get_fragment_count(param);
	process_count = ia_css_process_group_compute_process_count(manifest, param);
	terminal_count = ia_css_process_group_compute_terminal_count(manifest, param);

	process_group->fragment_count = fragment_count;
	process_group->process_count = process_count;
	process_group->terminal_count = terminal_count;

	/* Set default */
	verifexit(ia_css_process_group_set_fragment_limit(process_group, fragment_count) == 0, EINVAL);

	tmp_size = process_count * sizeof(ia_css_process_t *);
	process_group->processes = (ia_css_process_t **)ia_css_cpu_mem_alloc(tmp_size);
	verifexit(process_group->processes != NULL, ENOBUFS);
	ia_css_cpu_mem_set_zero(process_group->processes, tmp_size);
	size += tmp_size;

	for (i = 0; i < process_count; i++) {
		ia_css_program_manifest_t *program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, i);
		ia_css_program_param_t *program_param = ia_css_program_group_param_get_program_param(param, i);
		process_group->processes[i] = ia_css_process_create(program_manifest, program_param);
		verifexit(process_group->processes[i] != NULL, ENOBUFS);
		size += ia_css_process_get_size(process_group->processes[i]);

		ia_css_process_set_parent(process_group->processes[i], process_group);
		if (ia_css_has_program_manifest_fixed_cell(program_manifest)) {
			vied_nci_cell_ID_t	cell_id = ia_css_program_manifest_get_cell_ID(program_manifest);
			ia_css_process_set_cell(process_group->processes[i], cell_id);
		}
	}

	process_group->terminals = (ia_css_terminal_t **)ia_css_cpu_mem_alloc(terminal_count * sizeof(ia_css_terminal_t *));
	verifexit(process_group->terminals != NULL, ENOBUFS);
	size += terminal_count * sizeof(ia_css_terminal_t *);
	ia_css_cpu_mem_set_zero(process_group->terminals, terminal_count * sizeof(ia_css_terminal_t *));

	for (i = 0; i < terminal_count; i++) {
		ia_css_terminal_manifest_t *terminal_manifest = ia_css_program_group_manifest_get_terminal_manifest(manifest, i);
		ia_css_terminal_param_t *terminal_param = ia_css_program_group_param_get_terminal_param(param, i);
		process_group->terminals[i] = ia_css_terminal_create(terminal_manifest, terminal_param);
		verifexit(process_group->terminals[i] != NULL, ENOBUFS);
		size += ia_css_terminal_get_size(process_group->terminals[i]);
		ia_css_terminal_set_parent(process_group->terminals[i], process_group);
	}

	process_group->size = ia_css_sizeof_process_group(manifest, param);
	verifexit(process_group->size == size, EINVAL);
	process_group->ID = ia_css_program_group_manifest_get_program_group_ID(manifest);

	verifexit(process_group->ID != 0, EINVAL);

	ret = ia_css_process_group_on_create(process_group, manifest, param);
	verifexit(ret == 0, EINVAL);

	process_group->state = IA_CSS_PROCESS_GROUP_READY;
	retval = 0;

EXIT:
	if (retval != 0) {
		process_group = ia_css_process_group_destroy(process_group);
	}
	return process_group;
}

ia_css_process_group_t *ia_css_process_group_destroy(
	ia_css_process_group_t			 		*process_group)
{
	int		i;

	if (process_group != NULL) {
		ia_css_process_group_on_destroy(process_group);
		if (process_group->terminals != NULL) {
			uint8_t	terminal_count = ia_css_process_group_get_terminal_count(process_group);
			for (i = (int)terminal_count - 1; i >= 0; i--) {
				process_group->terminals[i] = ia_css_terminal_destroy(process_group->terminals[i]);
			}

			ia_css_cpu_mem_free((void *)process_group->terminals);
			process_group->terminals = NULL;
		}

		if (process_group->processes != NULL) {
			uint8_t	process_count = ia_css_process_group_get_process_count(process_group);
			for (i = (int)process_count - 1; i >= 0; i--) {
				process_group->processes[i] = ia_css_process_destroy(process_group->processes[i]);
			}

			ia_css_cpu_mem_free((void *)process_group->processes);
			process_group->processes = NULL;
		}

		ia_css_cpu_mem_free((void *)process_group);
		process_group = NULL;
	}
	return process_group;
}

int ia_css_process_group_submit(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_SUBMIT);
}

int ia_css_process_group_start(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_START);
}

int ia_css_process_group_stop(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_STOP);
}

int ia_css_process_group_run(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_RUN);
}

int ia_css_process_group_suspend(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_SUSPEND);
}

int ia_css_process_group_resume(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_RESUME);
}

int ia_css_process_group_reset(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_RESET);
}

int ia_css_process_group_abort(
	ia_css_process_group_t					*process_group)
{
	return ia_css_process_group_exec_cmd(process_group, IA_CSS_PROCESS_GROUP_CMD_ABORT);
}

extern uint64_t ia_css_process_group_get_token(
	ia_css_process_group_t					*process_group)
{
	uint64_t	token = 0;

	verifexit(process_group != NULL, EINVAL);

	token = process_group->token;
EXIT:
	return token;
}


int ia_css_process_group_set_token(
	ia_css_process_group_t					*process_group,
	const uint64_t							token)
{
	int	retval = -1;

	verifexit(process_group != NULL, EINVAL);
	verifexit(token != 0, EINVAL);

	process_group->token = token;

	retval = 0;
EXIT:
	return retval;
}

extern uint64_t ia_css_process_group_get_private_token(
	ia_css_process_group_t					*process_group)
{
	uint64_t	token = 0;

	verifexit(process_group != NULL, EINVAL);

	token = process_group->private_token;
EXIT:
	return token;
}


int ia_css_process_group_set_private_token(
	ia_css_process_group_t					*process_group,
	const uint64_t							token)
{
	int	retval = -1;

	verifexit(process_group != NULL, EINVAL);
	verifexit(token != 0, EINVAL);

	process_group->private_token = token;

	retval = 0;
EXIT:
	return retval;
}

uint16_t ia_css_process_group_get_fragment_limit(
	ia_css_process_group_t					*process_group)
{
	uint16_t	fragment_limit = 0;
	verifexit(process_group != NULL, EINVAL);

	fragment_limit = process_group->fragment_limit;
EXIT:
	return fragment_limit;
}

int ia_css_process_group_set_fragment_limit(
	ia_css_process_group_t					*process_group,
	const uint16_t							fragment_limit)
{
	int	retval = -1;
	uint16_t	current_limit = ia_css_process_group_get_fragment_limit(process_group);

	verifexit(process_group != NULL, EINVAL);
	verifexit(fragment_limit > current_limit, EINVAL);
	verifexit(fragment_limit <= ia_css_process_group_get_fragment_count(process_group), EINVAL);

	process_group->fragment_limit = fragment_limit;

	retval = 0;
EXIT:
	return retval;
}

int ia_css_process_group_clear_fragment_limit(
	ia_css_process_group_t					*process_group)
{
	int	retval = -1;

	verifexit(process_group != NULL, EINVAL);
	process_group->fragment_limit = 0;

	retval = 0;
EXIT:
	return retval;
}
int ia_css_process_group_attach_buffer(
	ia_css_process_group_t					*process_group,
	vied_vaddress_t							buffer,
	const ia_css_buffer_state_t				buffer_state,
	const unsigned int						terminal_index)
{
	int	retval = -1;
	ia_css_terminal_t *terminal = ia_css_process_group_get_terminal(process_group, terminal_index);
	verifexit(terminal != NULL, EINVAL);
	verifexit(ia_css_process_group_get_state(process_group) == IA_CSS_PROCESS_GROUP_READY, EINVAL);

	ia_css_terminal_set_buffer(terminal, buffer);

	if (ia_css_is_terminal_data_terminal(terminal) == true) {
		ia_css_frame_t	*frame = ia_css_data_terminal_get_frame((ia_css_data_terminal_t *)terminal);
		verifexit(frame != NULL, EINVAL);
		verifexit(ia_css_frame_set_buffer_state(frame, buffer_state) == 0, EINVAL);
	}

	retval = 0;

EXIT:
	return retval;
}

vied_vaddress_t ia_css_process_group_detach_buffer(
	ia_css_process_group_t					*process_group,
	const unsigned int						terminal_index)
{
	vied_vaddress_t		buffer = VIED_NULL;

	ia_css_terminal_t	*terminal = ia_css_process_group_get_terminal(process_group, terminal_index);
	ia_css_process_group_state_t	state = ia_css_process_group_get_state(process_group);

	verifexit(terminal != NULL, EINVAL);
	verifexit(state == IA_CSS_PROCESS_GROUP_READY, EINVAL);

	buffer = ia_css_terminal_get_buffer(terminal);

	if (ia_css_is_terminal_data_terminal(terminal) == true) {
		ia_css_frame_t	*frame = ia_css_data_terminal_get_frame((ia_css_data_terminal_t *)terminal);
		verifexit(frame != NULL, EINVAL);
		verifexit(ia_css_frame_set_buffer_state(frame, IA_CSS_BUFFER_NULL) == 0, EINVAL);
	}
	ia_css_terminal_set_buffer(terminal, VIED_NULL);

EXIT:
/* buffer pointer will appear on output, regardless of subsequent fails to avoid memory leaks */
	return buffer;
}

int ia_css_process_group_attach_stream(
	ia_css_process_group_t					*process_group,
	uint32_t								stream,
	const ia_css_buffer_state_t				buffer_state,
	const unsigned int						terminal_index)
{
	int	retval = -1;
	NOT_USED(process_group);
	NOT_USED(stream);
	NOT_USED(buffer_state);
	NOT_USED(terminal_index);
	verifexit(0, ENOSYS);
	retval = 0;
EXIT:
	return retval;
}

uint32_t ia_css_process_group_detach_stream(
	ia_css_process_group_t					*process_group,
	const unsigned int						terminal_index)
{
	uint32_t	stream = 0;
	NOT_USED(process_group);
	NOT_USED(terminal_index);
	verifexit(0, ENOSYS);
EXIT:
	return stream;
}

int ia_css_process_group_set_barrier(
	ia_css_process_group_t					*process_group,
	const vied_nci_barrier_ID_t				barrier_index)
{
	int	retval = -1;
	vied_nci_resource_bitmap_t	bit_mask;
	vied_nci_resource_bitmap_t	resource_bitmap = ia_css_process_group_get_resource_bitmap(process_group);

	verifexit(process_group != NULL, EINVAL);

	bit_mask = vied_nci_barrier_bit_mask(barrier_index);

	verifexit(bit_mask != 0, EINVAL);
	verifexit(vied_nci_is_bitmap_clear(bit_mask, resource_bitmap), EINVAL);

	resource_bitmap = vied_nci_bitmap_set(resource_bitmap, bit_mask);

	retval = ia_css_process_group_set_resource_bitmap(process_group, resource_bitmap);
EXIT:
	return retval;
}

int ia_css_process_group_clear_barrier(
	ia_css_process_group_t					*process_group,
	const vied_nci_barrier_ID_t				barrier_index)
{
	int	retval = -1;
	vied_nci_resource_bitmap_t	bit_mask;
	vied_nci_resource_bitmap_t	resource_bitmap = ia_css_process_group_get_resource_bitmap(process_group);

	verifexit(process_group != NULL, EINVAL);

	bit_mask = vied_nci_barrier_bit_mask(barrier_index);

	verifexit(bit_mask != 0, EINVAL);
	verifexit(vied_nci_is_bitmap_set(bit_mask, resource_bitmap), EINVAL);

	resource_bitmap = vied_nci_bitmap_clear(resource_bitmap, bit_mask);

	retval = ia_css_process_group_set_resource_bitmap(process_group, resource_bitmap);
EXIT:
	return retval;
}

int ia_css_process_group_print(
	const ia_css_process_group_t			*process_group,
	void									*fid)
{
	int	retval = -1;
	int	i;

	uint8_t	process_count, terminal_count;

	NOT_USED(fid);

	verifexit(process_group != NULL, EINVAL);

	PRINT("ia_css_process_group_print\n");
	PRINT("sizeof(process_group) = %d\n",(int)ia_css_process_group_get_size(process_group));

	PRINT("program_group(process_group) = %d\n",(int)ia_css_process_group_get_program_group_ID(process_group));
/*	PRINT("program_group(process_group) = %s\n",(ia_css_program_group_string(ia_css_process_group_get_program_group_ID(process_group))); */

	process_count = ia_css_process_group_get_process_count(process_group);
	terminal_count = ia_css_process_group_get_terminal_count(process_group);

	PRINT("%d processes\n",(int)process_count);
	for (i = 0; i < (int)process_count; i++) {
		ia_css_process_t	*process = ia_css_process_group_get_process(process_group, i);
		retval = ia_css_process_print(process, fid);
		verifjmpexit(retval == 0);
	}
	PRINT("%d terminals\n",(int)terminal_count);
	for (i = 0; i < (int)terminal_count; i++) {
		ia_css_terminal_t	*terminal = ia_css_process_group_get_terminal(process_group, i);
		retval = ia_css_terminal_print(terminal, fid);
		verifjmpexit(retval == 0);
	}

	retval = 0;
EXIT:
	return retval;
}

bool ia_css_is_process_group_valid(
	const ia_css_process_group_t			*process_group,
	const ia_css_program_group_manifest_t	*manifest,
	const ia_css_program_group_param_t		*param)
{
	bool is_valid = false;

	verifexit(process_group != NULL, EINVAL);
	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);

	is_valid = ia_css_is_program_group_manifest_valid(manifest);
EXIT:
	return is_valid;
}

bool ia_css_can_process_group_submit (
	const ia_css_process_group_t			*process_group)
{
	int		i;
	bool	can_submit = false;
	uint8_t	terminal_count = ia_css_process_group_get_terminal_count(process_group);

	verifexit(process_group != NULL, EINVAL);

	for (i = 0; i < (int)terminal_count; i++) {
		ia_css_terminal_t		*terminal = ia_css_process_group_get_terminal(process_group, i);
		vied_vaddress_t			buffer;
		ia_css_buffer_state_t	buffer_state;

		verifexit(terminal != NULL, EINVAL);
		buffer = ia_css_terminal_get_buffer(terminal);
		if (buffer == VIED_NULL) {
			break;
		}

		/* buffer_state is applicable only for data terminals*/
		if (ia_css_is_terminal_data_terminal(terminal) == true) {
			ia_css_frame_t	*frame = ia_css_data_terminal_get_frame((ia_css_data_terminal_t *)terminal);
			verifexit(frame != NULL, EINVAL);
			buffer_state = ia_css_frame_get_buffer_state(frame);
			if ((buffer_state == IA_CSS_BUFFER_NULL) ||
			    (buffer_state == IA_CSS_N_BUFFER_STATES)) {
				break;
			}
		} else if (ia_css_is_terminal_parameter_terminal(terminal) != true) {
			/* neither data nor parameter terminal, so error.*/
			break;
		}

/* FAS allows for attaching NULL buffers to satisfy SDF, but only if l-Scheduler is embedded */
	}
/* Only true if no check failed */
	can_submit = (i == terminal_count);
EXIT:
	return can_submit;
}


bool ia_css_can_process_group_start (
	const ia_css_process_group_t			*process_group)
{
	int		i;
	bool	can_start = false;
	uint8_t	terminal_count = ia_css_process_group_get_terminal_count(process_group);

	verifexit(process_group != NULL, EINVAL);

	for (i = 0; i < (int)terminal_count; i++) {
		ia_css_terminal_t		*terminal = ia_css_process_group_get_terminal(process_group, i);
		ia_css_buffer_state_t	buffer_state;
		bool					is_input = ia_css_is_terminal_input(terminal);
		bool					ok = false;

		verifexit(terminal != NULL, EINVAL);
		if (ia_css_is_terminal_data_terminal(terminal) == true) {
			/* buffer_state is applicable only for data terminals*/
			ia_css_frame_t	*frame = ia_css_data_terminal_get_frame((ia_css_data_terminal_t *)terminal);
			verifexit(frame != NULL, EINVAL);
			buffer_state = ia_css_frame_get_buffer_state(frame);

			ok = ((is_input && (buffer_state == IA_CSS_BUFFER_FULL)) || (!is_input && (buffer_state == IA_CSS_BUFFER_EMPTY)));

		} else if (ia_css_is_terminal_parameter_terminal(terminal) == true) {
			/*FIXME: is there any pre-requisite for param_terminal ?*/
			ok = true;
		} else {
			/* neither data nor parameter terminal, so error.*/
			break;
		}

		if (!ok) {
			break;
		}
	}
/* Only true if no check failed */
	can_start = (i == terminal_count);
EXIT:
	return can_start;
}

size_t ia_css_process_group_get_size(
	const ia_css_process_group_t			*process_group)
{
	size_t	size = 0;

	if (process_group != NULL) {
		size = process_group->size;
	}

	return size;
}

ia_css_process_group_state_t ia_css_process_group_get_state(
	const ia_css_process_group_t			*process_group)
{
	ia_css_process_group_state_t	state = IA_CSS_N_PROCESS_GROUP_STATES;

	if (process_group != NULL) {
		state = process_group->state;
	}

	return state;
}

uint16_t ia_css_process_group_get_fragment_count(
	const ia_css_process_group_t			*process_group)
{
	uint16_t	fragment_count = 0;
	if (process_group != NULL) {
		fragment_count = process_group->fragment_count;
	}

	return fragment_count;
}

uint8_t ia_css_process_group_get_process_count(
	const ia_css_process_group_t			*process_group)
{
	uint8_t		process_count = 0;
	if (process_group != NULL) {
		process_count = process_group->process_count;
	}

	return process_count;
}

uint8_t ia_css_process_group_get_terminal_count(
	const ia_css_process_group_t			*process_group)
{
	uint8_t		terminal_count = 0;
	if (process_group != NULL) {
		terminal_count = process_group->terminal_count;
	}

	return terminal_count;
}

ia_css_terminal_t *ia_css_process_group_get_terminal(
	const ia_css_process_group_t			*process_group,
	const unsigned int						terminal_index)
{
	ia_css_terminal_t *terminal = NULL;

	uint8_t		terminal_count = ia_css_process_group_get_terminal_count(process_group);

	verifexit(process_group != NULL, EINVAL);
	verifexit(terminal_index < terminal_count, EINVAL);

	terminal = process_group->terminals[terminal_index];
EXIT:
	return terminal;
}

ia_css_process_t *ia_css_process_group_get_process(
	const ia_css_process_group_t			*process_group,
	const unsigned int						process_index)
{
	ia_css_process_t *process = NULL;

	uint8_t process_count = ia_css_process_group_get_process_count(process_group);

	verifexit(process_group != NULL, EINVAL);
	verifexit(process_index < process_count, EINVAL);

	process = process_group->processes[process_index];
EXIT:
	return process;
}

ia_css_program_group_ID_t ia_css_process_group_get_program_group_ID(
	const ia_css_process_group_t			*process_group)
{
	ia_css_program_group_ID_t	id = 0;

	verifexit(process_group != NULL, EINVAL);

	id = process_group->ID;

EXIT:
	return id;
}

vied_nci_resource_bitmap_t ia_css_process_group_get_resource_bitmap(
	const ia_css_process_group_t			*process_group)
{
	vied_nci_resource_bitmap_t	resource_bitmap = 0;

	verifexit(process_group != NULL, EINVAL);

	resource_bitmap = process_group->resource_bitmap;

EXIT:
	return resource_bitmap;
}

int ia_css_process_group_set_resource_bitmap(
	ia_css_process_group_t					*process_group,
	const vied_nci_resource_bitmap_t		resource_bitmap)
{
	int	retval = -1;

	verifexit(process_group != NULL, EINVAL);

	process_group->resource_bitmap = resource_bitmap;

	retval = 0;
EXIT:
	return retval;
}

uint32_t ia_css_process_group_compute_cycle_count(
	const ia_css_program_group_manifest_t	*manifest,
	const ia_css_program_group_param_t		*param)
{
	uint32_t	cycle_count = 0;

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);

	cycle_count = 1;
EXIT:
	return cycle_count;
}

uint8_t ia_css_process_group_compute_process_count(
	const ia_css_program_group_manifest_t	*manifest,
	const ia_css_program_group_param_t		*param)
{
	uint8_t						process_count = 0;
	int							i;

	ia_css_kernel_bitmap_t		total_bitmap = ia_css_program_group_manifest_get_kernel_bitmap(manifest);
	ia_css_kernel_bitmap_t		enable_bitmap = ia_css_program_group_param_get_kernel_enable_bitmap(param);

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);
	verifexit(ia_css_is_program_group_manifest_valid(manifest), EINVAL);
	verifexit(ia_css_is_kernel_bitmap_subset(total_bitmap, enable_bitmap), EINVAL);
	verifexit(!ia_css_is_kernel_bitmap_empty(enable_bitmap), EINVAL);

	for (i = 0; i < (int)ia_css_program_group_manifest_get_program_count(manifest); i++) {
		ia_css_program_manifest_t	*program_manifest = ia_css_program_group_manifest_get_program_manifest(manifest, i);
		ia_css_kernel_bitmap_t		program_bitmap = ia_css_program_manifest_get_kernel_bitmap(program_manifest);
/* Programs can be orthogonal, a mutually exclusive subset, or a concurrent subset */
		if (!ia_css_is_kernel_bitmap_intersection_empty(enable_bitmap, program_bitmap)) {
			ia_css_program_type_t		program_type = ia_css_program_manifest_get_type(program_manifest);
/* An exclusive subnode < exclusive supernode, so simply don't count it */
			if (program_type != IA_CSS_PROGRAM_TYPE_EXCLUSIVE_SUB) {
				process_count++;
			}
		}
	}

EXIT:
	return process_count;
}

uint8_t ia_css_process_group_compute_terminal_count(
	const ia_css_program_group_manifest_t	*manifest,
	const ia_css_program_group_param_t		*param)
{
	uint8_t						terminal_count = 0;
	int							i;

	ia_css_kernel_bitmap_t		total_bitmap = ia_css_program_group_manifest_get_kernel_bitmap(manifest);
	ia_css_kernel_bitmap_t		enable_bitmap = ia_css_program_group_param_get_kernel_enable_bitmap(param);

	verifexit(manifest != NULL, EINVAL);
	verifexit(param != NULL, EINVAL);
	verifexit(ia_css_is_program_group_manifest_valid(manifest), EINVAL);
	verifexit(ia_css_is_kernel_bitmap_subset(total_bitmap, enable_bitmap), EINVAL);
	verifexit(!ia_css_is_kernel_bitmap_empty(enable_bitmap), EINVAL);

	for (i = 0; i < (int)ia_css_program_group_manifest_get_terminal_count(manifest); i++) {
		ia_css_terminal_manifest_t *tmanifest =
			ia_css_program_group_manifest_get_terminal_manifest(manifest, i);
		if (ia_css_is_terminal_manifest_data_terminal(tmanifest)) {
			ia_css_data_terminal_manifest_t	*data_terminal_manifest =
				(ia_css_data_terminal_manifest_t *)tmanifest;
			/* Parameter terminals don't contribute */
			if (data_terminal_manifest != NULL) {
				ia_css_kernel_bitmap_t terminal_bitmap =
					ia_css_data_terminal_manifest_get_kernel_bitmap(
						data_terminal_manifest);
/* Terminals depend on a kernel, if the kernel is present the program it contains and the terminal the program depends on are active */
				if (!ia_css_is_kernel_bitmap_intersection_empty(enable_bitmap, terminal_bitmap)) {
					terminal_count++;
				}
			}
		} else if (ia_css_is_terminal_manifest_parameter_terminal(tmanifest)) {
			terminal_count++;
		}
	}

EXIT:
	return terminal_count;
}

