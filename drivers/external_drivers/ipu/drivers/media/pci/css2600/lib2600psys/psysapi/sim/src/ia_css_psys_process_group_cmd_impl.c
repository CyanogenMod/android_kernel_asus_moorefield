#include "ia_css_psys_process_group_cmd_impl.h"
#include "ia_css_psysapi.h"
#include "ia_css_psys_process.h"
#include "ia_css_psys_process.psys.h"
#include "ia_css_psys_process_group.h"
#include "ia_css_psys_process_group.psys.h"
#include "error_support.h"
#include "vied_nci_psys_system_global.h"
#include "misc_support.h"

/* Dummy implementation for sim */
int ia_css_process_group_on_create(
	ia_css_process_group_t					*process_group,
	const ia_css_program_group_manifest_t	*program_group_manifest,
	const ia_css_program_group_param_t		*program_group_param)
{
	NOT_USED(process_group);
	NOT_USED(program_group_manifest);
	NOT_USED(program_group_param);
	return 0;
}

/* Dummy implementation for sim */
int ia_css_process_group_on_destroy(
	ia_css_process_group_t					*process_group)
{
	NOT_USED(process_group);

	return 0;
}

int ia_css_process_group_exec_cmd(
	ia_css_process_group_t					*process_group,
	const ia_css_process_group_cmd_t		cmd)
{
	int	retval = -1;
	ia_css_process_group_state_t	state = ia_css_process_group_get_state(process_group);

	verifexit(process_group != NULL, EINVAL);
	verifexit(state != IA_CSS_PROCESS_GROUP_ERROR, EINVAL);
	verifexit(state < IA_CSS_N_PROCESS_GROUP_STATES, EINVAL);

	switch (cmd) {
	case IA_CSS_PROCESS_GROUP_CMD_NOP:
		break;
	case IA_CSS_PROCESS_GROUP_CMD_SUBMIT:
		verifexit(state == IA_CSS_PROCESS_GROUP_READY, EINVAL);

/* External resource availability checks */
		verifexit(ia_css_can_process_group_submit(process_group), EINVAL);

		process_group->state = IA_CSS_PROCESS_GROUP_BLOCKED;
		break;
	case IA_CSS_PROCESS_GROUP_CMD_ATTACH:
		verifexit(state == IA_CSS_PROCESS_GROUP_READY, EINVAL);
		break;
	case IA_CSS_PROCESS_GROUP_CMD_DETACH:
		verifexit(state == IA_CSS_PROCESS_GROUP_READY, EINVAL);
		break;
	case IA_CSS_PROCESS_GROUP_CMD_START:
		verifexit(state == IA_CSS_PROCESS_GROUP_BLOCKED, EINVAL);

/* External resource state checks */
		verifexit(ia_css_can_process_group_start(process_group), EINVAL);

		process_group->state = IA_CSS_PROCESS_GROUP_STARTED;
		break;
	case IA_CSS_PROCESS_GROUP_CMD_RUN:
		verifexit(state == IA_CSS_PROCESS_GROUP_STARTED, EINVAL);
		process_group->state = IA_CSS_PROCESS_GROUP_RUNNING;
		break;
	case IA_CSS_PROCESS_GROUP_CMD_STOP:
		verifexit(state == IA_CSS_PROCESS_GROUP_RUNNING, EINVAL);
		process_group->state = IA_CSS_PROCESS_GROUP_STOPPED;
		break;
	case IA_CSS_PROCESS_GROUP_CMD_SUSPEND:
		verifexit(state == IA_CSS_PROCESS_GROUP_RUNNING, EINVAL);
		process_group->state = IA_CSS_PROCESS_GROUP_STARTED;
		break;
	case IA_CSS_PROCESS_GROUP_CMD_RESUME:
		verifexit(state == IA_CSS_PROCESS_GROUP_STARTED, EINVAL);
		process_group->state = IA_CSS_PROCESS_GROUP_RUNNING;
		break;
	case IA_CSS_PROCESS_GROUP_CMD_ABORT:
		verifexit(((state == IA_CSS_PROCESS_GROUP_RUNNING) || (state == IA_CSS_PROCESS_GROUP_STARTED)), EINVAL);
		process_group->state = IA_CSS_PROCESS_GROUP_STOPPED;
		break;
	case IA_CSS_PROCESS_GROUP_CMD_RESET:
/* We accept a reset command in the stopped state, mostly for simplifying the statemachine test */
		verifexit(((state == IA_CSS_PROCESS_GROUP_RUNNING) || (state == IA_CSS_PROCESS_GROUP_STARTED) || (state == IA_CSS_PROCESS_GROUP_STOPPED)), EINVAL);
		process_group->state = IA_CSS_PROCESS_GROUP_BLOCKED;
		break;
	case IA_CSS_N_PROCESS_GROUP_CMDS:	/* Fall through */
	default:
		verifexit(false, EINVAL);
		break;
	}

	retval = 0;
EXIT:
	return retval;
}
