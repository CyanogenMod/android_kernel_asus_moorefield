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

#ifndef CSS2600_PSYS_H
#define CSS2600_PSYS_H

#include <linux/cdev.h>

#include "css2600.h"
#include "css2600-pdata.h"

struct css2600_psys {
	struct cdev cdev;
	struct device dev;

	struct mutex mutex;
	struct list_head fhs;
	struct css2600_psys_pdata *pdata;
	struct css2600_bus_device *adev;
	struct ia_css_syscom_context *dev_ctx;
	uint32_t resource_bitmap;
};

struct css2600_psys_fh {
	struct css2600_psys *psys;
	struct list_head list;
	struct list_head bufmap;
	struct list_head eventq;
	struct list_head kcmds[CSS2600_PSYS_CMD_PRIORITY_NUM];
	wait_queue_head_t wait;
};

struct css2600_psys_eventq {
	struct css2600_psys_event ev;
	struct list_head list;
};

struct css2600_psys_kcmd {
	struct css2600_psys_fh *fh;
	struct list_head list;
	void *pg_manifest;
	size_t pg_manifest_size;
	void *pg_params;
	size_t pg_params_size;
	struct css2600_psys_kbuffer **kbufs;
	size_t nbuffers;
	ia_css_process_group_t *pg;
	uint32_t id;
	uint64_t issue_id;
	uint32_t priority;
};

struct css2600_psys_kbuffer {
	uint64_t len;
	void *userptr;
	int fd;
	void *kaddr;
	struct list_head list;
	bool vma_is_io;
	dma_addr_t dma_addr;
	struct sg_table *sgt;
	struct page **pages;
	size_t npages;
	struct dma_buf_attachment *db_attach;
	struct dma_buf *dbuf;
};

#define inode_to_css2600_psys(inode) \
	container_of((inode)->i_cdev, struct css2600_psys, cdev)

#ifdef CONFIG_COMPAT
extern long css2600_psys_compat_ioctl32(struct file *file, unsigned int cmd,
					unsigned long arg);
#endif
#endif /* CSS2600_PSYS_H */
