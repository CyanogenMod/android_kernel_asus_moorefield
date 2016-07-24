/*
 * Copyright (c) 2013 Intel Corporation. All Rights Reserved.
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

#ifndef CSS2600_PSYS_STUB_H
#define CSS2600_PSYS_STUB_H

#include <linux/cdev.h>
#include <linux/list.h>
#include "css2600-psys.h"

#define CSS2600_PSYS_STUB_NAME		"css2600-psys-stub"

#define inode_to_css2600_device(inode) \
	container_of((inode)->i_cdev, struct css2600_device, cdev)

#define device_to_css2600_device(device) \
	container_of(device, struct css2600_device, dev)

struct css2600_kbuffer {
	void *userptr;
	int mapped;
	int fd;
	struct list_head list;
};

struct css2600_eventq {
	struct css2600_event *ev;
	struct list_head list;
};

#define CSS2600_STUB_CMD_SUSPEND	(1 << 0)
#define CSS2600_STUB_CMD_CANCEL		(1 << 1)

struct css2600_run_cmd {
	struct css2600_command command;
	struct css2600_fh *fh;
	struct list_head list;
	uint32_t flags;
};

struct css2600_fh {
	struct device *dev;
	wait_queue_head_t wait;
	struct list_head list;
	struct list_head bufmap;
	struct list_head eventq;
};

struct css2600_device {
	struct cdev cdev;
	struct device dev;

	/* Serialise access to everything below mutex. */
	struct mutex mutex;
	struct list_head fhs;
	struct list_head commands[CSS2600_CMD_PRIORITY_NUM];
	struct workqueue_struct	*run_cmd_queue;
	struct work_struct run_cmd;

	struct css2600_run_cmd *cur_cmd;
	bool queue_empty;
};

#endif
