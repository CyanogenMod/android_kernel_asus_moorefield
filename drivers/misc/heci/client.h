/*
 * HECI client logic
 *
 * Copyright (c) 2003-2014, Intel Corporation.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */

#ifndef _HECI_CLIENT_H_
#define _HECI_CLIENT_H_

#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/poll.h>
#include "heci.h"
#include "heci_dev.h"

int heci_me_cl_by_uuid(const struct heci_device *dev, const uuid_le *cuuid);
int heci_me_cl_by_id(struct heci_device *dev, u8 client_id);

/*
 * HECI IO Functions
 */
struct heci_cl_cb *heci_io_cb_init(struct heci_cl *cl, struct file *fp);
void heci_io_cb_free(struct heci_cl_cb *priv_cb);
int heci_io_cb_alloc_req_buf(struct heci_cl_cb *cb, size_t length);
int heci_io_cb_alloc_resp_buf(struct heci_cl_cb *cb, size_t length);


/**
 * heci_io_list_init - Sets up a queue list.
 *
 * @list: An instance cl callback structure
 */
static inline void heci_io_list_init(struct heci_cl_cb *list)
{
	INIT_LIST_HEAD(&list->list);
}
void heci_io_list_flush(struct heci_cl_cb *list, struct heci_cl *cl);

/*
 * HECI Host Client Functions
 */

struct heci_cl *heci_cl_allocate(struct heci_device *dev);
void heci_cl_init(struct heci_cl *cl, struct heci_device *dev);


int heci_cl_link(struct heci_cl *cl, int id);
int heci_cl_unlink(struct heci_cl *cl);

int heci_cl_flush_queues(struct heci_cl *cl);
struct heci_cl_cb *heci_cl_find_read_cb(struct heci_cl *cl);

/**
 * heci_cl_cmp_id - tells if file private data have same id
 *
 * @fe1: private data of 1. file object
 * @fe2: private data of 2. file object
 *
 * returns true  - if ids are the same and not NULL
 */
static inline bool heci_cl_cmp_id(const struct heci_cl *cl1,
				const struct heci_cl *cl2)
{
	return cl1 && cl2 &&
		(cl1->host_client_id == cl2->host_client_id) &&
		(cl1->me_client_id == cl2->me_client_id);
}

int heci_cl_flow_ctrl_creds(struct heci_cl *cl);

int heci_cl_flow_ctrl_reduce(struct heci_cl *cl);

/*
 *  HECI input output function prototype
 */
bool heci_cl_is_other_connecting(struct heci_cl *cl);
int heci_cl_disconnect(struct heci_cl *cl);
int heci_cl_connect(struct heci_cl *cl, struct file *file);
int heci_cl_read_start(struct heci_cl *cl, size_t length);
int heci_cl_write(struct heci_cl *cl, struct heci_cl_cb *cb, bool blocking);
void heci_cl_complete(struct heci_cl *cl, struct heci_cl_cb *cb);

void heci_host_client_init(struct work_struct *work);

void heci_cl_all_disconnect(struct heci_device *dev);
void heci_cl_all_read_wakeup(struct heci_device *dev);
void heci_cl_all_write_clear(struct heci_device *dev);

#endif /* _HECI_CLIENT_H_ */
