/*
 * Copyright © 2012 Intel Corporation
 *
 * Permission is hereby granted, free of charge, to any person obtaining a
 * copy of this software and associated documentation files (the "Software"),
 * to deal in the Software without restriction, including without limitation
 * the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the
 * Software is furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice (including the next
 * paragraph) shall be included in all copies or substantial portions of the
 * Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.  IN NO EVENT SHALL
 * THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING
 * FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER
 * DEALINGS IN THE SOFTWARE.
 *
 * Authors:
 * Hitesh K. Patel <hitesh.k.patel@intel.com>
 *
 */

#ifndef _DISPLAY_MANAGER_NETLINK_H_
#define _DISPLAY_MANAGER_NETLINK_H_

#include <drm/drmP.h>
#include "mdfld_dsi_output.h"

#define SUCCESS		1
#define FAILED		0

enum dispmgr_module_enum {
	DISPMGR_MOD_UNKNOWN,
	DISPMGR_MOD_NETLINK,
	DISPMGR_MOD_DPST,
};

enum dispmgr_event_enum {
	DISPMGR_UNKNOWN,
	DISPMGR_TEST,
	DISPMGR_TEST_TEXT,
};

enum dispmgr_dpst_event_enum {
	DISPMGR_DPST_UNKNOWN,
	DISPMGR_DPST_INIT_COMM,
	DISPMGR_DPST_UPDATE_GUARD,
	DISPMGR_DPST_HIST_ENABLE,
	DISPMGR_DPST_HIST_DATA,
	DISPMGR_DPST_BL_SET,
	DISPMGR_DPST_GAMMA_SET,
	DISPMGR_DPST_DIET_ENABLE,
	DISPMGR_DPST_DIET_DISABLE,
	DISPMGR_DPST_GET_MODE,
};

/* Display Manager Command Header */
struct dispmgr_command_hdr {
	unsigned int module;	/* module to receive the command */
	unsigned int cmd;	/* command from Userspace */
	unsigned int data_size;	/* data size of command_data in number of bytes */
	void *data;		/* command data */
};

void dispmgr_start(struct drm_device *dev);
void dispmgr_nl_send_msg(struct dispmgr_command_hdr *cmd_hdr);
void dpstmgr_reg_restore_locked(struct mdfld_dsi_config *dsi_config);

#endif				/* _DISPLAY_MANAGER_NETLINK_H_ */
