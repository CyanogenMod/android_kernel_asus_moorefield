
#include "ia_css_psys_device.h"

#include <error_support.h>
#include <assert_support.h>
#include <print_support.h>
#include <misc_support.h>


#define IA_CSS_PSYS_CMD_QUEUE_SIZE				0x20
#define IA_CSS_PSYS_EVENT_QUEUE_SIZE			0x40

static struct ia_css_syscom_config	psys_syscom_config = {0, 0, 0, 0, 0, 0, NULL, 0, NULL, NULL};
static bool	external_alloc = true;


static unsigned int ia_css_psys_cmd_queue_size[IA_CSS_N_PSYS_CMD_QUEUE_ID] = {
	IA_CSS_PSYS_CMD_QUEUE_SIZE,
	IA_CSS_PSYS_CMD_QUEUE_SIZE};
static unsigned int ia_css_psys_event_queue_size[IA_CSS_N_PSYS_EVENT_QUEUE_ID] = {
	IA_CSS_PSYS_EVENT_QUEUE_SIZE};

static unsigned int ia_css_psys_cmd_msg_size[IA_CSS_N_PSYS_CMD_QUEUE_ID] = {
	IA_CSS_PSYS_CMD_BITS/8,
	IA_CSS_PSYS_CMD_BITS/8};
static unsigned int ia_css_psys_event_msg_size[IA_CSS_N_PSYS_EVENT_QUEUE_ID] = {
	IA_CSS_PSYS_EVENT_BITS/8};



int ia_css_psys_config_print(
	const struct ia_css_syscom_config		*config,
	void									*fh)
{
	int	retval = -1;
	FILE *fid = (FILE *)fh;

	verifexit(config != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(fid,"ia_css_psys_config_print\n");

	retval = 0;
EXIT:
	return retval;
}


int ia_css_psys_print(
	const struct ia_css_syscom_context		*context,
	void									*fh)
{
	int	retval = -1;
	FILE *fid = (FILE *)fh;

	verifexit(context != NULL, EINVAL);
	verifexit(fid != NULL, EINVAL);

	fprintf(fid,"ia_css_psys_print\n");

	retval = 0;
EXIT:
	return retval;
}

struct ia_css_syscom_config *ia_css_psys_specify(void)
{
	struct ia_css_syscom_config	*config = &psys_syscom_config;

	config->num_input_queues = IA_CSS_N_PSYS_CMD_QUEUE_ID;
	config->num_output_queues = IA_CSS_N_PSYS_EVENT_QUEUE_ID;
	config->input_queue_size = IA_CSS_PSYS_CMD_QUEUE_SIZE;
	config->output_queue_size = IA_CSS_PSYS_EVENT_QUEUE_SIZE;
	config->input_token_size = IA_CSS_PSYS_CMD_BITS/8;
	config->output_token_size = IA_CSS_PSYS_EVENT_BITS/8;

	return config;
}

size_t ia_css_sizeof_psys(
	struct ia_css_syscom_config				*config)
{
	size_t	size = 0;

	NOT_USED(config);

	return size;
}

struct ia_css_syscom_context* ia_css_psys_open(
	const struct ia_css_psys_buffer_s		*buffer,
	struct ia_css_syscom_config				*config)
{
	struct ia_css_syscom_context	*context = NULL;

	verifexit(config != NULL, EINVAL);

	if (buffer == NULL) {
/* Allocate locally */
		external_alloc = false;
	}
/*
 * Here we would like to pass separately the sub-system ID
 * and optionally the user pointer to be mapped, depending on
 * where this open is called, and which virtual memory handles
 * we see here.
 */
/*	context = ia_css_syscom_open(get_virtual_memory_handle(vied_psys_ID), buffer, config); */
	context = ia_css_syscom_open(config);
EXIT:
	return context;
}

struct ia_css_syscom_context* ia_css_psys_close(
	struct ia_css_syscom_context			*context)
{
	verifexit((ia_css_syscom_close(context) == 0), EFAULT);
	context = NULL;

	if (external_alloc) {
/*		memset(); */
	} else {
/* Free local allocations */
/* Reset */
		external_alloc = true;
	}
EXIT:
	return context;
}


bool ia_css_is_psys_cmd_queue_full(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id)
{
	bool			is_full = false;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_send_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	is_full = (num_tokens == 0);
EXIT:
	return is_full;
}

bool ia_css_is_psys_cmd_queue_not_full(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id)
{
	bool			is_not_full = false;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_send_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	is_not_full = (num_tokens != 0);
EXIT:
	return is_not_full;
}

bool ia_css_has_psys_cmd_queue_N_space(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id,
	const unsigned int						N)
{
	bool			has_N_space = false;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_send_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	has_N_space = (num_tokens >= N);
EXIT:
	return has_N_space;
}

int ia_css_psys_cmd_queue_get_available_space(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id)
{
	int				N_space = -1;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_send_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	N_space = (int)(num_tokens);
EXIT:
	return N_space;
}

bool ia_css_any_psys_event_queue_not_empty(
	const struct ia_css_syscom_context		*context)
{
	ia_css_psys_cmd_queue_ID_t	i;
	bool	any_msg = false;

	for (i = (ia_css_psys_event_queue_ID_t)0; i < IA_CSS_N_PSYS_EVENT_QUEUE_ID; i++) {
		any_msg = any_msg || ia_css_is_psys_event_queue_not_empty(context, i);
	}

	return any_msg;
}

bool ia_css_is_psys_event_queue_empty(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id)
{
	bool			is_empty = false;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_recv_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	is_empty = (num_tokens == 0);
EXIT:
	return is_empty;
}

bool ia_css_is_psys_event_queue_not_empty(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id)
{
	bool			is_not_empty = false;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_recv_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	is_not_empty = (num_tokens != 0);
EXIT:
	return is_not_empty;
}

bool ia_css_has_psys_event_queue_N_msgs(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id,
	const unsigned int						N)
{
	bool			has_N_msgs = false;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_recv_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	has_N_msgs = (num_tokens >= N);
EXIT:
	return has_N_msgs;
}

int ia_css_psys_event_queue_get_available_msgs(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id)
{
	int				N_msgs = -1;
	unsigned int	num_tokens;
	verifexit(ia_css_syscom_recv_port_available(context, (unsigned int)id, &num_tokens) == 0, EINVAL);
	N_msgs = (int)(num_tokens);
EXIT:
	return N_msgs;
}


int ia_css_psys_cmd_queue_send(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_cmd_queue_ID_t		id,
	const void								*cmd_msg_buffer)
{
	int	count = 0;

	verifexit(context != NULL, EINVAL);
/* The ~full check fails on receive queues */
	verifexit(ia_css_is_psys_cmd_queue_not_full(context, id), EINVAL);
	verifexit(cmd_msg_buffer != NULL, EINVAL);

	verifexit(ia_css_syscom_send_port_open(context, (unsigned int)id) == 0, EINVAL);
	verifexit(ia_css_syscom_send_port_transfer(context, (unsigned int)id, cmd_msg_buffer) == 0, EINVAL);
/* If close fails we lose the knowledge of the message, ergo, open and close should no be here at all */
	verifexit(ia_css_syscom_send_port_close(context, (unsigned int)id) == 0, EINVAL);

	count = 1;
EXIT:
	return count;
}

int ia_css_psys_cmd_queue_send_N(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_cmd_queue_ID_t		id,
	const void								*cmd_msg_buffer,
	const unsigned int						N)
{
	struct ia_css_psys_cmd_s	*cmd_msg_buffer_loc = (struct ia_css_psys_cmd_s *)cmd_msg_buffer;
	int	count;

	for (count = 0; count < (int)N; count++) {
		int	count_loc = ia_css_psys_cmd_queue_send(context, id , (void *)(&cmd_msg_buffer_loc[count]));
		verifexit(count_loc == 1, EINVAL);
	}

EXIT:
	return count;
}

int ia_css_psys_event_queue_receive(
	struct ia_css_syscom_context			*context,
	ia_css_psys_event_queue_ID_t			id,
	void									*event_msg_buffer)
{
	int	count = 0;

	verifexit(context != NULL, EINVAL);
/* The ~empty check fails on send queues */
	verifexit(ia_css_is_psys_event_queue_not_empty(context, id), EINVAL);
	verifexit(event_msg_buffer != NULL, EINVAL);

	verifexit(ia_css_syscom_recv_port_open(context, (unsigned int)id) == 0, EINVAL);
	verifexit(ia_css_syscom_recv_port_transfer(context, (unsigned int)id, event_msg_buffer) == 0, EINVAL);
/* If close fails we lose the message, ergo, open and close should no be here at all */
	verifexit(ia_css_syscom_recv_port_close(context, (unsigned int)id) == 0, EINVAL);

	count = 1;
EXIT:
	return count;
}

int ia_css_psys_event_queue_receive_N(
	struct ia_css_syscom_context			*context,
	const ia_css_psys_event_queue_ID_t		id,
	void									*event_msg_buffer,
	const unsigned int						N)
{
	struct ia_css_psys_event_s	*event_msg_buffer_loc = (struct ia_css_psys_event_s *)event_msg_buffer;
	int	count;

	for (count = 0; count < (int)N; count++) {
		int	count_loc = ia_css_psys_event_queue_receive(context, id , (void *)(&event_msg_buffer_loc[count]));
		verifexit(count_loc == 1, EINVAL);
	}

EXIT:
	return count;
}


size_t ia_css_psys_get_size(
	const struct ia_css_syscom_context		*context)
{
	size_t	size = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
EXIT:
	return size;
}

unsigned int ia_css_psys_get_cmd_queue_count(
	const struct ia_css_syscom_context		*context)
{
	unsigned int	count = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	count = (unsigned int)IA_CSS_N_PSYS_CMD_QUEUE_ID;
EXIT:
	return count;
}

unsigned int ia_css_psys_get_event_queue_count(
	const struct ia_css_syscom_context		*context)
{
	unsigned int	count = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	count = (unsigned int)IA_CSS_N_PSYS_EVENT_QUEUE_ID;
EXIT:
	return count;
}

size_t ia_css_psys_get_cmd_queue_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id)
{
	size_t	queue_size = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	NOT_USED(id);
	queue_size = ia_css_psys_cmd_queue_size[0];
EXIT:
	return queue_size;
}

size_t ia_css_psys_get_event_queue_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id)
{
	size_t	queue_size = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	NOT_USED(id);
	queue_size = ia_css_psys_event_queue_size[0];
EXIT:
	return queue_size;
}

size_t ia_css_psys_get_cmd_msg_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_cmd_queue_ID_t		id)
{
	size_t	msg_size = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	NOT_USED(id);
	msg_size = ia_css_psys_cmd_msg_size[0];
EXIT:
	return msg_size;
}

size_t ia_css_psys_get_event_msg_size(
	const struct ia_css_syscom_context		*context,
	const ia_css_psys_event_queue_ID_t		id)
{
	size_t	msg_size = 0;
	verifexit (context != NULL, EINVAL);
/* How can I query the context ? */
	NOT_USED(context);
	NOT_USED(id);
	msg_size = ia_css_psys_event_msg_size[0];
EXIT:
	return msg_size;
}
