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
 */

#ifndef CSS2600_PSYS_H
#define CSS2600_PSYS_H

#include <linux/types.h>

struct css2600_capability {
	uint32_t version;
	uint8_t driver[20];
} __attribute__ ((packed));

struct css2600_event {
	uint32_t type;		/* CSS2600_EVENT_TYPE_ */
	union {
		struct {
			uint32_t id;
			uint32_t issue_id;
		} cmd_done;
	} ev;
	uint32_t reserved[3];
} __attribute__ ((packed));

#define CSS2600_EVENT_TYPE_CMD_COMPLETE      1

/**
 * struct css2600_buffer - input/output port descriptor
 * @len:	buffer size
 * @userptr:	user pointer (NULL if not mapped to user space)
 * @fd:	DMA-BUF handle (filled by driver)
 * @flags:	flags
 */
struct css2600_buffer {
	uint64_t len;
	void __user *userptr;
	int fd;
	uint32_t flags;
	uint32_t reserved2[4];
} __attribute__ ((packed)) ;

#define CSS2600_BUFFER_FLAG_INPUT	(1 << 0)
#define CSS2600_BUFFER_FLAG_OUTPUT	(1 << 1)
#define CSS2600_BUFFER_FLAG_MAPPED	(1 << 2)

#define	CSS2600_CMD_PRIORITY_HIGH	0
#define	CSS2600_CMD_PRIORITY_MED	1
#define	CSS2600_CMD_PRIORITY_LOW	2
#define	CSS2600_CMD_PRIORITY_NUM	3

/**
 * struct css2600_command - processing command
 * @id:	id of the command
 * @priority:	priority of the command
 * @bufcount:	number of buffers in bufs array
 * @bufs:	userspace pointers to array of css2600_buffer structs
 * @issue_id:	issue id to link command and event
 *
 * Specifies a processing command including input and output buffers.
 */
struct css2600_command {
	uint32_t id;
	uint32_t priority;
	uint32_t bufcount;
	struct css2600_buffer __user *buffers;
	uint32_t issue_id;
	uint32_t reserved[3];
} __attribute__ ((packed));

#define CSS2600_IOC_QUERYCAP _IOWR('A', 1, struct css2600_capability)
#define CSS2600_IOC_MAPBUF _IOWR('A', 2, struct css2600_buffer)
#define CSS2600_IOC_UNMAPBUF _IOWR('A', 3, struct css2600_buffer)
#define CSS2600_IOC_GETBUF _IOWR('A', 4, struct css2600_buffer)
#define CSS2600_IOC_PUTBUF _IOWR('A', 5, struct css2600_buffer)
#define CSS2600_IOC_QCMD _IOWR('A', 6, struct css2600_command)
#define CSS2600_IOC_DQEVENT _IOWR('A', 7, struct css2600_event)
#define CSS2600_IOC_CMD_CANCEL _IOWR('A', 8, struct css2600_command)

#ifdef CONFIG_COMPAT
struct css2600_buffer32 {
	uint64_t len;
	compat_uptr_t userptr;
	int fd;
	uint32_t flags;
	uint32_t reserved2[4];
} __attribute__ ((packed));

struct css2600_command32 {
	uint32_t id;
	uint32_t priority;
	uint32_t bufcount;
	compat_uptr_t buffers;
	uint32_t issue_id;
	uint32_t reserved[3];
} __attribute__ ((packed));

#define CSS2600_IOC_MAPBUF32 _IOWR('A', 2, struct css2600_buffer32)
#define CSS2600_IOC_UNMAPBUF32 _IOWR('A', 3, struct css2600_buffer32)
#define CSS2600_IOC_GETBUF32 _IOWR('A', 4, struct css2600_buffer32)
#define CSS2600_IOC_PUTBUF32 _IOWR('A', 5, struct css2600_buffer32)
#define CSS2600_IOC_QCMD32 _IOWR('A', 6, struct css2600_command32)
#define CSS2600_IOC_CMD_CANCEL32 _IOWR('A', 8, struct css2600_command32)
#endif

#endif
