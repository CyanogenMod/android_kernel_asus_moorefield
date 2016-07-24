#include "ia_css_psys_device.h"
#include "ia_css_psysapi.h"
#include "test_infra_loader.spc.h"
#include "ia_css_bxt_pss_process_group.h"

#include "error_support.h"

size_t ia_css_sizeof_psys(
	struct ia_css_syscom_config				*config)
{
	(void)config;
	return 100;//sizeof(struct ia_css_syscom_context);
}

struct ia_css_syscom_context *ia_css_psys_open(
	const struct ia_css_psys_buffer_s		*buffer,
	struct ia_css_syscom_config			*config)
{
	// TODO: implement platform load code
	(void)config;

	platform_load();

	test_infra_spc_main_program_config(5);

	platform_start();

	return (struct ia_css_syscom_context *)buffer;
}

struct ia_css_syscom_context *ia_css_psys_close(
	struct ia_css_syscom_context			*context)
{
	// TODO: check what still needs to be done for close
	platform_stop();
	return context;
}

int ia_css_psys_event_queue_receive(
	struct ia_css_syscom_context			*context,
	ia_css_psys_event_queue_ID_t			id,
	void						*event_msg_buffer)
{
	int count = 0;

	verifexit(context != NULL, EINVAL);
	verifexit(event_msg_buffer != NULL, EINVAL);
	verifexit(id == IA_CSS_PSYS_EVENT_QUEUE_MAIN_ID, EINVAL);
	/* some defaults */
	struct ia_css_psys_event_s *event = (struct ia_css_psys_event_s *)event_msg_buffer;
	event->status = -1;
	event->command = 0;
	event->process_group = 0;
	event->token = 0;
	/* if we have an event, we return it in the buffer */
	if (process_group_ready()) {
		/* fill in count and event */
		count = 1;
		event->status = 0;
		event->command = IA_CSS_PROCESS_GROUP_CMD_STOP;
		event->process_group = 1;
	}
EXIT:
	return count;
}

