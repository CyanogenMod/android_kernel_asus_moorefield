/*
 * HECI client logic (for both HECI bus driver and user-mode API)
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
 *
 */

#include <linux/export.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/delay.h>
#include <linux/pm_runtime.h>
#include "heci.h"
#include "heci_dev.h"
#include "hbm.h"
#include "client.h"

#ifdef dev_dbg
#undef dev_dbg
#endif
static void no_dev_dbg(void *v, char *s, ...)
{
}
#define dev_dbg no_dev_dbg
/*#define dev_dbg dev_err*/

/**
 * heci_me_cl_by_uuid - locate index of me client
 *
 * @dev: heci device
 * returns me client index or -ENOENT if not found
 */
int heci_me_cl_by_uuid(const struct heci_device *dev, const uuid_le *uuid)
{
	int i, res = -ENOENT;

	for (i = 0; i < dev->me_clients_num; ++i) {
		if (uuid_le_cmp(*uuid, dev->me_clients[i].props.protocol_name) == 0) {
			res = i;
			break;
		}
	}
	return res;
}
EXPORT_SYMBOL(heci_me_cl_by_uuid);


/**
 * heci_me_cl_by_id return index to me_clients for client_id
 *
 * @dev: the device structure
 * @client_id: me client id
 *
 * Locking: called under "dev->device_lock" lock
 *
 * returns index on success, -ENOENT on failure.
 */

int heci_me_cl_by_id(struct heci_device *dev, u8 client_id)
{
	int i;
	for (i = 0; i < dev->me_clients_num; i++)
		if (dev->me_clients[i].client_id == client_id)
			break;
	if (WARN_ON(dev->me_clients[i].client_id != client_id))
		return -ENOENT;

	if (i == dev->me_clients_num)
		return -ENOENT;

	return i;
}


/**
 * heci_io_list_flush - removes list entry belonging to cl.
 *
 * @list:  An instance of our list structure
 * @cl: host client
 */
void heci_io_list_flush(struct heci_cl_cb *list, struct heci_cl *cl)
{
	struct heci_cl_cb *cb;
	struct heci_cl_cb *next;

	list_for_each_entry_safe(cb, next, &list->list, list) {
		if (cb->cl && heci_cl_cmp_id(cl, cb->cl))
			list_del(&cb->list);
	}
}

/**
 * heci_io_cb_free - free heci_cb_private related memory
 *
 * @cb: heci callback struct
 */
void heci_io_cb_free(struct heci_cl_cb *cb)
{
	if (cb == NULL)
		return;

	kfree(cb->request_buffer.data);
	kfree(cb->response_buffer.data);
	kfree(cb);
}
EXPORT_SYMBOL(heci_io_cb_free);

/**
 * heci_io_cb_init - allocate and initialize io callback
 *
 * @cl - heci client
 * @file: pointer to file structure
 *
 * returns heci_cl_cb pointer or NULL;
 */
struct heci_cl_cb *heci_io_cb_init(struct heci_cl *cl, struct file *fp)
{
	struct heci_cl_cb *cb;

	cb = kzalloc(sizeof(struct heci_cl_cb), GFP_KERNEL);
	if (!cb)
		return NULL;

	heci_io_list_init(cb);

	cb->file_object = fp;
	cb->cl = cl;
	cb->buf_idx = 0;
	return cb;
}

/**
 * heci_io_cb_alloc_req_buf - allocate request buffer
 *
 * @cb -  io callback structure
 * @size: size of the buffer
 *
 * returns 0 on success
 *         -EINVAL if cb is NULL
 *         -ENOMEM if allocation failed
 */
int heci_io_cb_alloc_req_buf(struct heci_cl_cb *cb, size_t length)
{
	if (!cb)
		return -EINVAL;

	if (length == 0)
		return 0;

	cb->request_buffer.data = kmalloc(length, GFP_KERNEL);
	if (!cb->request_buffer.data)
		return -ENOMEM;
	cb->request_buffer.size = length;
	return 0;
}

/**
 * heci_io_cb_alloc_req_buf - allocate respose buffer
 *
 * @cb -  io callback structure
 * @size: size of the buffer
 *
 * returns 0 on success
 *         -EINVAL if cb is NULL
 *         -ENOMEM if allocation failed
 */
int heci_io_cb_alloc_resp_buf(struct heci_cl_cb *cb, size_t length)
{
	if (!cb)
		return -EINVAL;

	if (length == 0)
		return 0;

	cb->response_buffer.data = kmalloc(length, GFP_KERNEL);
	if (!cb->response_buffer.data)
		return -ENOMEM;
	cb->response_buffer.size = length;
	return 0;
}

/**
 * heci_cl_flush_queues - flushes queue lists belonging to cl.
 *
 * @dev: the device structure
 * @cl: host client
 */
int heci_cl_flush_queues(struct heci_cl *cl)
{
	if (WARN_ON(!cl || !cl->dev))
		return -EINVAL;

	dev_dbg(&cl->dev->pdev->dev, "remove list entry belonging to cl\n");
	heci_io_list_flush(&cl->dev->read_list, cl);
	heci_io_list_flush(&cl->dev->write_list, cl);
	heci_io_list_flush(&cl->dev->write_waiting_list, cl);
	heci_io_list_flush(&cl->dev->ctrl_wr_list, cl);
	heci_io_list_flush(&cl->dev->ctrl_rd_list, cl);
	return 0;
}
EXPORT_SYMBOL(heci_cl_flush_queues);


/**
 * heci_cl_init - initializes intialize cl.
 *
 * @cl: host client to be initialized
 * @dev: heci device
 */
void heci_cl_init(struct heci_cl *cl, struct heci_device *dev)
{
	memset(cl, 0, sizeof(struct heci_cl));
	init_waitqueue_head(&cl->wait);
	init_waitqueue_head(&cl->rx_wait);
	init_waitqueue_head(&cl->tx_wait);
	INIT_LIST_HEAD(&cl->link);
	INIT_LIST_HEAD(&cl->device_link);
	cl->reading_state = HECI_IDLE;
	cl->writing_state = HECI_IDLE;
	cl->dev = dev;
}

/**
 * heci_cl_allocate - allocates cl  structure and sets it up.
 *
 * @dev: heci device
 * returns  The allocated file or NULL on failure
 */
struct heci_cl *heci_cl_allocate(struct heci_device *dev)
{
	struct heci_cl *cl;

	/* DD -- fixed GFP_KERNEL to GFP_ATOMIC since
	 * this can be called from ISR BH.
	 * We don't really want it to sleep */
	cl = kmalloc(sizeof(struct heci_cl), GFP_ATOMIC);
	if (!cl)
		return NULL;

	heci_cl_init(cl, dev);

	return cl;
}
EXPORT_SYMBOL(heci_cl_allocate);

/**
 * heci_cl_find_read_cb - find this cl's callback in the read list
 *
 * @dev: device structure
 * returns cb on success, NULL on error
 */
struct heci_cl_cb *heci_cl_find_read_cb(struct heci_cl *cl)
{
	struct heci_device *dev = cl->dev;
	struct heci_cl_cb *cb = NULL;
	struct heci_cl_cb *next = NULL;

	list_for_each_entry_safe(cb, next, &dev->read_list.list, list)
		if (heci_cl_cmp_id(cl, cb->cl))
			return cb;
	return NULL;
}
EXPORT_SYMBOL(heci_cl_find_read_cb);

/** heci_cl_link: allocte host id in the host map
 *
 * @cl - host client
 * @id - fixed host id or -1 for genereting one
 * returns 0 on success
 *	-EINVAL on incorrect values
 *	-ENONET if client not found
 */
int heci_cl_link(struct heci_cl *cl, int id)
{
	struct heci_device *dev;

	if (WARN_ON(!cl || !cl->dev))
		return -EINVAL;

	dev = cl->dev;

	/* If Id is not asigned get one*/
	if (id == HECI_HOST_CLIENT_ID_ANY)
		id = find_first_zero_bit(dev->host_clients_map,
					HECI_CLIENTS_MAX);

	if (id >= HECI_CLIENTS_MAX) {
		dev_err(&dev->pdev->dev, "id exceded %d", HECI_CLIENTS_MAX);
		return -ENOENT;
	}

	dev->open_handle_count++;
	cl->host_client_id = id;
	list_add_tail(&cl->link, &dev->file_list);
	set_bit(id, dev->host_clients_map);
	cl->state = HECI_FILE_INITIALIZING;
	dev_dbg(&dev->pdev->dev, "link cl host id = %d\n", cl->host_client_id);

	return 0;
}
EXPORT_SYMBOL(heci_cl_link);

/**
 * heci_cl_unlink - remove me_cl from the list
 *
 * @dev: the device structure
 */
int heci_cl_unlink(struct heci_cl *cl)
{
	struct heci_device *dev;
	struct heci_cl *pos, *next;

	/* don't shout on error exit path */
	if (!cl)
		return 0;

	dev = cl->dev;
	list_for_each_entry_safe(pos, next, &dev->file_list, link) {
		if (cl->host_client_id == pos->host_client_id) {
			dev_dbg(&dev->pdev->dev, "remove host client = %d, ME client = %d\n",
				pos->host_client_id, pos->me_client_id);
			list_del_init(&pos->link);
			break;
		}
	}
	return 0;
}
EXPORT_SYMBOL(heci_cl_unlink);

void heci_host_client_init(struct work_struct *work)
{
	struct heci_device *dev =
			container_of(work, struct heci_device, init_work);
	struct heci_client_properties *client_props;
	int i;

	mutex_lock(&dev->device_lock);

#if 0
/* Moved */
	bitmap_zero(dev->host_clients_map, HECI_CLIENTS_MAX);
	dev->open_handle_count = 0;

	/*
	 * Reserving the first three client IDs
	 * 0: Reserved for HECI Bus Message communications
	 * 1: Reserved for Watchdog
	 * 2: Reserved for AMTHI
	 */
	bitmap_set(dev->host_clients_map, 0, 3);
#endif

	for (i = 0; i < dev->me_clients_num; i++)
		client_props = &dev->me_clients[i].props;

	dev->dev_state = HECI_DEV_ENABLED;

	mutex_unlock(&dev->device_lock);

	pm_runtime_mark_last_busy(&dev->pdev->dev);
	dev_dbg(&dev->pdev->dev, "rpm: autosuspend\n");
	pm_runtime_autosuspend(&dev->pdev->dev);
}


/**
 * heci_cl_disconnect - disconnect host clinet form the me one
 *
 * @cl: host client
 *
 * Locking: called under "dev->device_lock" lock
 *
 * returns 0 on success, <0 on failure.
 */
int heci_cl_disconnect(struct heci_cl *cl)
{
	struct heci_device *dev;
	struct heci_cl_cb *cb;
	int rets, err;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	if (cl->state != HECI_FILE_DISCONNECTING)
		return 0;

	cb = heci_io_cb_init(cl, NULL);
	if (!cb)
		return -ENOMEM;

	cb->fop_type = HECI_FOP_CLOSE;

	mutex_unlock(&dev->device_lock);
	rets = pm_runtime_get_sync(&dev->pdev->dev);
	dev_dbg(&dev->pdev->dev, "rpm: get sync %d\n", rets);
	mutex_lock(&dev->device_lock);
	if (IS_ERR_VALUE(rets)) {
		dev_err(&dev->pdev->dev, "rpm: get sync failed %d\n", rets);
		return rets;
	}

	if (dev->hbuf_is_ready) {
		dev->hbuf_is_ready = false;
		if (heci_hbm_cl_disconnect_req(dev, cl)) {
			rets = -ENODEV;
			dev_err(&dev->pdev->dev, "failed to disconnect.\n");
			goto free;
		}
		mdelay(10); /* Wait for hardware disconnection ready */
		list_add_tail(&cb->list, &dev->ctrl_rd_list.list);
	} else {
		dev_dbg(&dev->pdev->dev, "add disconnect cb to control write list\n");
		list_add_tail(&cb->list, &dev->ctrl_wr_list.list);

	}
	mutex_unlock(&dev->device_lock);

	err = wait_event_timeout(dev->wait_recvd_msg,
			(dev->dev_state == HECI_DEV_ENABLED && HECI_FILE_DISCONNECTED == cl->state),
			heci_secs_to_jiffies(HECI_CL_CONNECT_TIMEOUT));

	mutex_lock(&dev->device_lock);

	/* If FW reset arrived, this will happen.
	 * Don't check cl->, as 'cl' may be freed already
	 */
	if (dev->dev_state != HECI_DEV_ENABLED) {
		rets = -ENODEV;
		goto	free;
	}

	if (HECI_FILE_DISCONNECTED == cl->state) {
		rets = 0;
		dev_dbg(&dev->pdev->dev, "successfully disconnected from FW client.\n");
	} else {
		rets = -ENODEV;
		if (HECI_FILE_DISCONNECTED != cl->state)
			dev_dbg(&dev->pdev->dev, "wrong status client disconnect.\n");

		if (err)
			dev_dbg(&dev->pdev->dev,
				"wait failed disconnect err=%08x\n", err);

		dev_dbg(&dev->pdev->dev, "failed to disconnect from FW client.\n");
	}

	heci_io_list_flush(&dev->ctrl_rd_list, cl);
	heci_io_list_flush(&dev->ctrl_wr_list, cl);
free:

	mutex_unlock(&dev->device_lock);
	dev_dbg(&dev->pdev->dev, "rpm: autosuspend\n");
	pm_runtime_mark_last_busy(&dev->pdev->dev);
	pm_runtime_put_autosuspend(&dev->pdev->dev);
	mutex_lock(&dev->device_lock);

	heci_io_cb_free(cb);
	return rets;
}
EXPORT_SYMBOL(heci_cl_disconnect);


/**
 * heci_cl_is_other_connecting - checks if other
 *    client with the same me client id is connecting
 *
 * @cl: private data of the file object
 *
 * returns ture if other client is connected, 0 - otherwise.
 */
bool heci_cl_is_other_connecting(struct heci_cl *cl)
{
	struct heci_device *dev;
	struct heci_cl *pos;
	struct heci_cl *next;

	if (WARN_ON(!cl || !cl->dev))
		return false;

	dev = cl->dev;

	list_for_each_entry_safe(pos, next, &dev->file_list, link) {
		if ((pos->state == HECI_FILE_CONNECTING) &&
		    (pos != cl) && cl->me_client_id == pos->me_client_id)
			return true;

	}

	return false;
}

/**
 * heci_cl_connect - connect host clinet to the me one
 *
 * @cl: host client
 *
 * Locking: called under "dev->device_lock" lock
 *
 * returns 0 on success, <0 on failure.
 */
int heci_cl_connect(struct heci_cl *cl, struct file *file)
{
	struct heci_device *dev;
	struct heci_cl_cb *cb;
	long timeout = heci_secs_to_jiffies(HECI_CL_CONNECT_TIMEOUT);
	int rets;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	cb = heci_io_cb_init(cl, file);
	if (!cb) {
		rets = -ENOMEM;
		goto out;
	}


	mutex_unlock(&dev->device_lock);
	rets = pm_runtime_get_sync(&dev->pdev->dev);
	dev_dbg(&dev->pdev->dev, "rpm: get sync %d\n", rets);
	mutex_lock(&dev->device_lock);
	if (IS_ERR_VALUE(rets)) {
		dev_err(&dev->pdev->dev, "rpm: get sync failed %d\n", rets);
		return rets;
	}

	cb->fop_type = HECI_FOP_IOCTL;

	if (dev->hbuf_is_ready && !heci_cl_is_other_connecting(cl)) {
		dev->hbuf_is_ready = false;

		if (heci_hbm_cl_connect_req(dev, cl)) {
			rets = -ENODEV;
			goto out;
		}
		cl->timer_count = HECI_CONNECT_TIMEOUT;
		list_add_tail(&cb->list, &dev->ctrl_rd_list.list);
	} else {
		list_add_tail(&cb->list, &dev->ctrl_wr_list.list);
	}

	mutex_unlock(&dev->device_lock);
	rets = wait_event_timeout(dev->wait_recvd_msg,
				(dev->dev_state == HECI_DEV_ENABLED &&
				 (cl->state == HECI_FILE_CONNECTED ||
				  cl->state == HECI_FILE_DISCONNECTED)),
				 timeout * HZ);
	mutex_lock(&dev->device_lock);

	/* If FW reset arrived, this will happen.
	 * Don't check cl->, as 'cl' may be freed already
	 */
	if (dev->dev_state != HECI_DEV_ENABLED) {
		rets = -EFAULT;
		goto	out;
	}

	if (cl->state != HECI_FILE_CONNECTED) {
		rets = -EFAULT;

		heci_io_list_flush(&dev->ctrl_rd_list, cl);
		heci_io_list_flush(&dev->ctrl_wr_list, cl);
		goto out;
	}

	rets = cl->status;

out:
	mutex_unlock(&dev->device_lock);
	dev_dbg(&dev->pdev->dev, "rpm: autosuspend\n");
	pm_runtime_mark_last_busy(&dev->pdev->dev);
	pm_runtime_put_autosuspend(&dev->pdev->dev);
	mutex_lock(&dev->device_lock);

	heci_io_cb_free(cb);
	return rets;
}

/**
 * heci_cl_flow_ctrl_creds - checks flow_control credits for cl.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 *
 * returns 1 if heci_flow_ctrl_creds >0, 0 - otherwise.
 *	-ENOENT if heci_cl is not present
 *	-EINVAL if single_recv_buf == 0
 */
int heci_cl_flow_ctrl_creds(struct heci_cl *cl)
{
	struct heci_device *dev;
	int i;

	if (WARN_ON(!cl || !cl->dev))
		return -EINVAL;

	dev = cl->dev;

	if (!dev->me_clients_num)
		return 0;

	if (cl->heci_flow_ctrl_creds > 0)
		return 1;

	for (i = 0; i < dev->me_clients_num; i++) {
		struct heci_me_client  *me_cl = &dev->me_clients[i];
		if (me_cl->client_id == cl->me_client_id) {
			if (me_cl->heci_flow_ctrl_creds) {
				if (WARN_ON(me_cl->props.single_recv_buf == 0))
					return -EINVAL;
				return 1;
			} else {
				return 0;
			}
		}
	}
	return -ENOENT;
}

/**
 * heci_cl_flow_ctrl_reduce - reduces flow_control.
 *
 * @dev: the device structure
 * @cl: private data of the file object
 * @returns
 *	0 on success
 *	-ENOENT when me client is not found
 *	-EINVAL when ctrl credits are <= 0
 */
int heci_cl_flow_ctrl_reduce(struct heci_cl *cl)
{
	struct heci_device *dev;
	int i;

	if (WARN_ON(!cl || !cl->dev))
		return -EINVAL;

	dev = cl->dev;

	if (!dev->me_clients_num)
		return -ENOENT;

	for (i = 0; i < dev->me_clients_num; i++) {
		struct heci_me_client  *me_cl = &dev->me_clients[i];
		if (me_cl->client_id == cl->me_client_id) {
			if (me_cl->props.single_recv_buf != 0) {
				if (WARN_ON(me_cl->heci_flow_ctrl_creds <= 0))
					return -EINVAL;
				dev->me_clients[i].heci_flow_ctrl_creds--;
			} else {
				if (WARN_ON(cl->heci_flow_ctrl_creds <= 0))
					return -EINVAL;
				cl->heci_flow_ctrl_creds--;
			}
			return 0;
		}
	}
	return -ENOENT;
}

/**
 * heci_cl_read_start - the start read client message function.
 *
 * @cl: host client
 *
 * returns 0 on success, <0 on failure.
 */
int heci_cl_read_start(struct heci_cl *cl, size_t length)
{
	struct heci_device *dev;
	struct heci_cl_cb *cb;
	int rets;
	int i;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	if (cl->state != HECI_FILE_CONNECTED)
		return -ENODEV;

	if (dev->dev_state != HECI_DEV_ENABLED)
		return -ENODEV;

	if (cl->read_cb) {
		dev_dbg(&dev->pdev->dev, "read is pending.\n");
		return -EBUSY;
	}
	i = heci_me_cl_by_id(dev, cl->me_client_id);
	if (i < 0) {
		dev_err(&dev->pdev->dev, "no such me client %d\n",
			cl->me_client_id);
		return  -ENODEV;
	}

	mutex_unlock(&dev->device_lock);
	rets = pm_runtime_get_sync(&dev->pdev->dev);
	dev_dbg(&dev->pdev->dev, "rpm: get sync %d\n", rets);
	mutex_lock(&dev->device_lock);
	if (IS_ERR_VALUE(rets)) {
		dev_err(&dev->pdev->dev, "rpm: get sync failed %d\n", rets);
		return rets;
	}

	cb = heci_io_cb_init(cl, NULL);
	if (!cb) {
		rets = -ENOMEM;
		goto out;
	}

	/* always allocate at least client max message */
	length = max_t(size_t, length, dev->me_clients[i].props.max_msg_length);
	rets = heci_io_cb_alloc_resp_buf(cb, length);
	if (rets)
		goto out;

	cb->fop_type = HECI_FOP_READ;
	cl->read_cb = cb;


	if (dev->hbuf_is_ready) {
		dev->hbuf_is_ready = false;
		if (heci_hbm_cl_flow_control_req(dev, cl)) {
			rets = -ENODEV;
			goto out;
		}
		list_add_tail(&cb->list, &dev->read_list.list);
	} else {
		list_add_tail(&cb->list, &dev->ctrl_wr_list.list);
	}

out:
	mutex_unlock(&dev->device_lock);
	dev_dbg(&dev->pdev->dev, "rpm: autosuspend\n");
	pm_runtime_mark_last_busy(&dev->pdev->dev);
	pm_runtime_put_autosuspend(&dev->pdev->dev);
	mutex_lock(&dev->device_lock);

	if (rets)
		heci_io_cb_free(cb);

	return rets;
}
EXPORT_SYMBOL(heci_cl_read_start);

/**
 * heci_cl_write - submit a write cb to heci device
	assumes device_lock is locked
 *
 * @cl: host client
 * @cl: write callback with filled data
 *
 * returns numbe of bytes sent on success, <0 on failure.
 */
int heci_cl_write(struct heci_cl *cl, struct heci_cl_cb *cb, bool blocking)
{
	struct heci_device *dev;
	struct heci_msg_data *buf;
	struct heci_msg_hdr heci_hdr;
	int rets;


	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	if (WARN_ON(!cb))
		return -EINVAL;

	dev = cl->dev;


	buf = &cb->request_buffer;

	dev_dbg(&dev->pdev->dev, "heci_cl_write %d\n", buf->size);

	mutex_unlock(&dev->device_lock);
	rets = pm_runtime_get_sync(&dev->pdev->dev);
	dev_dbg(&dev->pdev->dev, "rpm: get sync %d\n", rets);
	mutex_lock(&dev->device_lock);
	if (IS_ERR_VALUE(rets)) {
		dev_err(&dev->pdev->dev, "rpm: get sync failed %d\n", rets);
		return rets;
	}

	cb->fop_type = HECI_FOP_WRITE;

	rets = heci_cl_flow_ctrl_creds(cl);
	if (rets < 0)
		goto err;

	/* Host buffer is not ready, we queue the request */
	if (rets == 0 || !dev->hbuf_is_ready) {
		cb->buf_idx = 0;
		/* unseting complete will enqueue the cb for write */
		heci_hdr.msg_complete = 0;
		cl->writing_state = HECI_WRITING;
		rets = buf->size;
		goto out;
	}

	dev->hbuf_is_ready = false;

	/* Check for a maximum length */
	if (buf->size > heci_hbuf_max_len(dev)) {
		heci_hdr.length = heci_hbuf_max_len(dev);
		heci_hdr.msg_complete = 0;
	} else {
		heci_hdr.length = buf->size;
		heci_hdr.msg_complete = 1;
	}

	heci_hdr.host_addr = cl->host_client_id;
	heci_hdr.me_addr = cl->me_client_id;
	heci_hdr.reserved = 0;

	dev_dbg(&dev->pdev->dev, "write " HECI_HDR_FMT "\n",
		HECI_HDR_PRM(&heci_hdr));


	if (heci_write_message(dev, &heci_hdr, buf->data)) {
		rets = -EIO;
		goto err;
	}

	cl->writing_state = HECI_WRITING;
	cb->buf_idx = heci_hdr.length;

	rets = buf->size;
out:
	if (heci_hdr.msg_complete) {
		if (heci_cl_flow_ctrl_reduce(cl)) {
			rets = -ENODEV;
			goto err;
		}
		list_add_tail(&cb->list, &dev->write_waiting_list.list);
	} else {
		list_add_tail(&cb->list, &dev->write_list.list);
	}


	if (blocking && cl->writing_state != HECI_WRITE_COMPLETE) {
		mutex_unlock(&dev->device_lock);
		if (wait_event_interruptible(cl->tx_wait,
			cl->writing_state == HECI_WRITE_COMPLETE)) {
				if (signal_pending(current))
					rets = -EINTR;
				else
					rets = -ERESTARTSYS;
		}
		mutex_lock(&dev->device_lock);
	}
err:
	mutex_unlock(&dev->device_lock);
	dev_dbg(&dev->pdev->dev, "rpm: autosuspend\n");
	pm_runtime_mark_last_busy(&dev->pdev->dev);
	pm_runtime_put_autosuspend(&dev->pdev->dev);
	mutex_lock(&dev->device_lock);

	return rets;
}


/**
 * heci_cl_complete - processes completed operation for a client
 *
 * @cl: private data of the file object.
 * @cb: callback block.
 */
void heci_cl_complete(struct heci_cl *cl, struct heci_cl_cb *cb)
{
	if (cb->fop_type == HECI_FOP_WRITE) {
		heci_io_cb_free(cb);
		cb = NULL;
		cl->writing_state = HECI_WRITE_COMPLETE;
		if (waitqueue_active(&cl->tx_wait))
			wake_up_interruptible(&cl->tx_wait);

	} else if (cb->fop_type == HECI_FOP_READ &&
			HECI_READING == cl->reading_state) {
		cl->reading_state = HECI_READ_COMPLETE;
		if (waitqueue_active(&cl->rx_wait))
			wake_up_interruptible(&cl->rx_wait);
		else
			heci_cl_bus_rx_event(cl);

	}
}


/**
 * heci_cl_all_disconnect - disconnect forcefully all connected clients
 *
 * @dev - heci device
 */
void heci_cl_all_disconnect(struct heci_device *dev)
{
	struct heci_cl *cl, *next;

	list_for_each_entry_safe(cl, next, &dev->file_list, link) {
		cl->state = HECI_FILE_DISCONNECTED;
		cl->heci_flow_ctrl_creds = 0;
		cl->read_cb = NULL;
		cl->timer_count = 0;
	}
}


/**
 * heci_cl_all_read_wakeup  - wake up all readings so they can be interrupted
 *
 * @dev  - heci device
 */
void heci_cl_all_read_wakeup(struct heci_device *dev)
{
	struct heci_cl *cl, *next;
	list_for_each_entry_safe(cl, next, &dev->file_list, link) {
		if (waitqueue_active(&cl->rx_wait)) {
			dev_dbg(&dev->pdev->dev, "Waking up client!\n");
			wake_up_interruptible(&cl->rx_wait);
		}
	}
}

/**
 * heci_cl_all_write_clear - clear all pending writes

 * @dev - heci device
 */
void heci_cl_all_write_clear(struct heci_device *dev)
{
	struct heci_cl_cb *cb, *next;

	list_for_each_entry_safe(cb, next, &dev->write_list.list, list) {
		list_del(&cb->list);
		heci_io_cb_free(cb);
	}
}
