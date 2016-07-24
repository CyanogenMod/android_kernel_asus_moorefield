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

#include "ia_css_psys_process_group_cmd_impl.h"
#include "ia_css_psys_process.h"
#include "ia_css_psys_process.psys.h"
#include "ia_css_psys_process_group.h"
#include "ia_css_psys_process_group.psys.h"
#include "error_support.h"
#include "ia_css_psys_state_change_actions.h"

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
	ia_css_process_group_state_change(process_group);
EXIT:
	return retval;
}
