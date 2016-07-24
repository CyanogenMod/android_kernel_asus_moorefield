/*
 * Copyright (c) 2013--2014 Intel Corporation. All Rights Reserved.
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
 */

#ifndef _UAPI_CSS2600_PSYS_H
#define _UAPI_CSS2600_PSYS_H

#include <linux/types.h>

struct css2600_psys_capability {
	uint32_t version;
	uint8_t driver[20];
	uint32_t reserved[10];
} __attribute__ ((packed));

struct css2600_psys_event {
	uint32_t type;		/* CSS2600_PSYS_EVENT_TYPE_ */
	uint32_t id;
	uint64_t issue_id;
	uint32_t buffer_idx;
	uint32_t error;
	int32_t reserved[2];
} __attribute__ ((packed));

#define CSS2600_PSYS_EVENT_TYPE_CMD_COMPLETE	1
#define CSS2600_PSYS_EVENT_TYPE_BUFFER_COMPLETE	2

/**
 * struct css2600_psys_buffer - for input/output terminals
 * @len:	buffer size
 * @userptr:	user pointer (NULL if not mapped to user space)
 * @fd:		DMA-BUF handle (filled by driver)
 * @flags:	flags
 */
struct css2600_psys_buffer {
	uint64_t len;
	void __user *userptr;
	int fd;
	uint32_t flags;
	uint32_t reserved[2];
} __attribute__ ((packed));

#define CSS2600_BUFFER_FLAG_INPUT	(1 << 0)
#define CSS2600_BUFFER_FLAG_OUTPUT	(1 << 1)
#define CSS2600_BUFFER_FLAG_MAPPED	(1 << 2)

#define	CSS2600_PSYS_CMD_PRIORITY_HIGH	0
#define	CSS2600_PSYS_CMD_PRIORITY_MED	1
#define	CSS2600_PSYS_CMD_PRIORITY_LOW	2
#define	CSS2600_PSYS_CMD_PRIORITY_NUM	3

/**
 * struct css2600_psys_command - processing command
 * @issue_id:		unique id for the command set by user
 * @id:			id of the command
 * @priority:		priority of the command
 * @pg_manifest:	userspace pointer to program group manifest
 * @pg_params:		userspace pointer to program group parameters
 * @buffers:		userspace pointers to array of DMA-BUF handles
 * @pg_manifest_size:	size of program group manifest
 * @pg_params_size:	size of program group parameters
 * @bufcount:		number of buffers in buffers array
 *
 * Specifies a processing command with input and output buffers.
 */
struct css2600_psys_command {
	uint64_t issue_id;
	uint32_t id;
	uint32_t priority;
	void __user *pg_manifest;
	void __user *pg_params;
	int __user *buffers;
	uint32_t pg_manifest_size;
	uint32_t pg_params_size;
	uint32_t bufcount;
	uint32_t reserved[3];
} __attribute__ ((packed));

#define CSS2600_IOC_QUERYCAP _IOR('A', 1, struct css2600_psys_capability)
#define CSS2600_IOC_MAPBUF _IOWR('A', 2, int)
#define CSS2600_IOC_UNMAPBUF _IOWR('A', 3, int)
#define CSS2600_IOC_GETBUF _IOWR('A', 4, struct css2600_psys_buffer)
#define CSS2600_IOC_PUTBUF _IOWR('A', 5, struct css2600_psys_buffer)
#define CSS2600_IOC_QCMD _IOWR('A', 6, struct css2600_psys_command)
#define CSS2600_IOC_DQEVENT _IOWR('A', 7, struct css2600_psys_event)
#define CSS2600_IOC_CMD_CANCEL _IOWR('A', 8, struct css2600_psys_command)

#endif /* _UAPI_CSS2600_PSYS_H */
