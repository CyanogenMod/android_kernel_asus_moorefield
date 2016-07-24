/*
 * Interrupt handling and implementation of protocols over IPC other than HECI
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

#include <linux/export.h>
#include <linux/pci.h>
#include <linux/kthread.h>
#include <linux/interrupt.h>
#include <linux/fs.h>
#include <linux/jiffies.h>
#include "heci.h"
#include "heci_dev.h"
#include "hbm.h"
#include "client.h"

#ifdef dev_dbg
#undef dev_dbg
#endif
static  void no_dev_dbg(void *v, char *s, ...)
{
}
#define dev_dbg no_dev_dbg
/*#define dev_dbg dev_err*/

/**
 * heci_irq_compl_handler - dispatch complete handelers
 *	for the completed callbacks
 *
 * @dev - heci device
 * @compl_list - list of completed cbs
 */
void heci_irq_compl_handler(struct heci_device *dev, struct heci_cl_cb *compl_list)
{
	struct heci_cl_cb *cb, *next;
	struct heci_cl *cl;

	list_for_each_entry_safe(cb, next, &compl_list->list, list)
	{
		cl = cb->cl;
		list_del(&cb->list);
		if (!cl)
			continue;

		dev_dbg(&dev->pdev->dev, "completing call back.\n");
		heci_cl_complete(cl, cb);
	}
}
EXPORT_SYMBOL_GPL(heci_irq_compl_handler);

/**
 * heci_cl_hbm_equal - check if hbm is addressed to the client
 *
 * @cl: host client
 * @heci_hdr: header of heci client message
 *
 * returns true if matches, false otherwise
 */
static inline int heci_cl_hbm_equal(struct heci_cl *cl,
				struct heci_msg_hdr *heci_hdr)
{
	return cl->host_client_id == heci_hdr->host_addr &&
		cl->me_client_id == heci_hdr->me_addr;
}

/**
 * heci_cl_is_reading - checks if the client
		is the one to read this message
 *
 * @cl: heci client
 * @heci_hdr: header of heci message
 *
 * returns true on match and false otherwise
 */
static bool heci_cl_is_reading(struct heci_cl *cl,
				struct heci_msg_hdr *heci_hdr)
{
	return heci_cl_hbm_equal(cl, heci_hdr) &&
		cl->state == HECI_FILE_CONNECTED &&
		cl->reading_state != HECI_READ_COMPLETE;
}

/**
 * heci_irq_read_client_message - process client message
 *
 * @dev: the device structure
 * @heci_hdr: header of heci client message
 * @complete_list: An instance of our list structure
 *
 * returns 0 on success, <0 on failure.
 */
static int heci_cl_irq_read_msg(struct heci_device *dev,
			       struct heci_msg_hdr *heci_hdr,
			       struct heci_cl_cb *complete_list)
{
	struct heci_cl *cl;
	struct heci_cl_cb *cb, *next;
	unsigned char *buffer = NULL;

	list_for_each_entry_safe(cb, next, &dev->read_list.list, list) {
		cl = cb->cl;
		if (!cl || !heci_cl_is_reading(cl, heci_hdr))
			continue;

		cl->reading_state = HECI_READING;

		if (cb->response_buffer.size == 0 ||
		    cb->response_buffer.data == NULL) {
			dev_err(&dev->pdev->dev, "response buffer is not allocated.\n");
			list_del(&cb->list);
			return -ENOMEM;
		}

		if (cb->response_buffer.size < heci_hdr->length + cb->buf_idx) {
			dev_dbg(&dev->pdev->dev, "message overflow. size %d len %d idx %ld\n",
				cb->response_buffer.size,
				heci_hdr->length, cb->buf_idx);
			buffer = krealloc(cb->response_buffer.data,
					  heci_hdr->length + cb->buf_idx,
					  GFP_KERNEL);

			if (!buffer) {
				dev_err(&dev->pdev->dev, "allocation failed.\n");
				list_del(&cb->list);
				return -ENOMEM;
			}
			cb->response_buffer.data = buffer;
			cb->response_buffer.size =
				heci_hdr->length + cb->buf_idx;
		}

		buffer = cb->response_buffer.data + cb->buf_idx;
		heci_read_slots(dev, buffer, heci_hdr->length);

		cb->buf_idx += heci_hdr->length;
		if (heci_hdr->msg_complete) {
			cl->status = 0;
			list_del(&cb->list);
			dev_dbg(&dev->pdev->dev, "completed read H cl = %d, ME cl = %d, length = %lu\n",
				cl->host_client_id,
				cl->me_client_id,
				cb->buf_idx);
			list_add_tail(&cb->list, &complete_list->list);
		}
		break;
	}

	dev_dbg(&dev->pdev->dev, "message read\n");
	if (!buffer) {
		heci_read_slots(dev, dev->rd_msg_buf, heci_hdr->length);
		dev_dbg(&dev->pdev->dev, "discarding message " HECI_HDR_FMT "\n",
				HECI_HDR_PRM(heci_hdr));
	}

	return 0;
}

/**
 * heci_cl_irq_close - processes close related operation from
 *	interrupt thread context - send disconnect request
 *
 * @cl: client
 * @cb: callback block.
 * @slots: free slots.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int heci_cl_irq_close(struct heci_cl *cl, struct heci_cl_cb *cb,
			s32 *slots, struct heci_cl_cb *cmpl_list)
{
	struct heci_device *dev = cl->dev;

	u32 msg_slots =
		heci_data2slots(sizeof(struct hbm_client_connect_request));

	if (*slots < msg_slots)
		return -EMSGSIZE;

	*slots -= msg_slots;

	if (heci_hbm_cl_disconnect_req(dev, cl)) {
		cl->status = 0;
		cb->buf_idx = 0;
		list_move_tail(&cb->list, &cmpl_list->list);
		return -EIO;
	}

	cl->state = HECI_FILE_DISCONNECTING;
	cl->status = 0;
	cb->buf_idx = 0;
	list_move_tail(&cb->list, &dev->ctrl_rd_list.list);
	cl->timer_count = HECI_CONNECT_TIMEOUT;

	return 0;
}


/**
 * heci_cl_irq_close - processes client read related operation from the
 *	interrupt thread context - request for flow control credits
 *
 * @cl: client
 * @cb: callback block.
 * @slots: free slots.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int heci_cl_irq_read(struct heci_cl *cl, struct heci_cl_cb *cb,
			   s32 *slots, struct heci_cl_cb *cmpl_list)
{
	struct heci_device *dev = cl->dev;

	u32 msg_slots = heci_data2slots(sizeof(struct hbm_flow_control));

	if (*slots < msg_slots) {
		/* return the cancel routine */
		list_del(&cb->list);
		return -EMSGSIZE;
	}

	*slots -= msg_slots;

	if (heci_hbm_cl_flow_control_req(dev, cl)) {
		cl->status = -ENODEV;
		cb->buf_idx = 0;
		list_move_tail(&cb->list, &cmpl_list->list);
		return -ENODEV;
	}
	list_move_tail(&cb->list, &dev->read_list.list);

	return 0;
}


/**
 * heci_cl_irq_ioctl - processes client ioctl related operation from the
 *	interrupt thread context -   send connection request
 *
 * @cl: client
 * @cb: callback block.
 * @slots: free slots.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int heci_cl_irq_ioctl(struct heci_cl *cl, struct heci_cl_cb *cb,
			   s32 *slots, struct heci_cl_cb *cmpl_list)
{
	struct heci_device *dev = cl->dev;

	u32 msg_slots =
		heci_data2slots(sizeof(struct hbm_client_connect_request));

	if (*slots < msg_slots) {
		/* return the cancel routine */
		list_del(&cb->list);
		return -EMSGSIZE;
	}

	*slots -=  msg_slots;

	cl->state = HECI_FILE_CONNECTING;

	if (heci_hbm_cl_connect_req(dev, cl)) {
		cl->status = -ENODEV;
		cb->buf_idx = 0;
		list_del(&cb->list);
		return -ENODEV;
	}

	list_move_tail(&cb->list, &dev->ctrl_rd_list.list);
	cl->timer_count = HECI_CONNECT_TIMEOUT;
	return 0;
}

/**
 * heci_cl_irq_write_complete - write messages to device.
 *
 * @cl: client
 * @cb: callback block.
 * @slots: free slots.
 * @cmpl_list: complete list.
 *
 * returns 0, OK; otherwise, error.
 */
static int heci_cl_irq_write_complete(struct heci_cl *cl, struct heci_cl_cb *cb,
				     s32 *slots, struct heci_cl_cb *cmpl_list)
{
	struct heci_device *dev = cl->dev;
	struct heci_msg_hdr heci_hdr;
	size_t len = cb->request_buffer.size - cb->buf_idx;
	u32 msg_slots = heci_data2slots(len);

	heci_hdr.host_addr = cl->host_client_id;
	heci_hdr.me_addr = cl->me_client_id;
	heci_hdr.reserved = 0;

	if (*slots >= msg_slots) {
		heci_hdr.length = len;
		heci_hdr.msg_complete = 1;
	/* Split the message only if we can write the whole host buffer */
	} else if (*slots == dev->hbuf_depth) {
		msg_slots = *slots;
		len = (*slots * sizeof(u32)) - sizeof(struct heci_msg_hdr);
		heci_hdr.length = len;
		heci_hdr.msg_complete = 0;
	} else {
		/* wait for next time the host buffer is empty */
		return 0;
	}

	dev_dbg(&dev->pdev->dev, "buf: size = %d idx = %lu\n",
			cb->request_buffer.size, cb->buf_idx);
	dev_dbg(&dev->pdev->dev, HECI_HDR_FMT, HECI_HDR_PRM(&heci_hdr));

	*slots -=  msg_slots;
	if (heci_write_message(dev, &heci_hdr,
			cb->request_buffer.data + cb->buf_idx)) {
		cl->status = -ENODEV;
		list_move_tail(&cb->list, &cmpl_list->list);
		return -ENODEV;
	}


	cl->status = 0;
	cb->buf_idx += heci_hdr.length;
	if (heci_hdr.msg_complete) {
		if (heci_cl_flow_ctrl_reduce(cl))
			return -ENODEV;
		list_move_tail(&cb->list, &dev->write_waiting_list.list);
	}

	return 0;
}

/**
 * heci_irq_thread_read_handler - bottom half read routine after ISR to
 * handle the read processing.
 *
 * @dev: the device structure
 * @cmpl_list: An instance of our list structure
 * @slots: slots to read.
 *
 * returns 0 on success, <0 on failure.
 */
int heci_irq_read_handler(struct heci_device *dev,
		struct heci_cl_cb *cmpl_list, s32 *slots)
{
	struct heci_msg_hdr *heci_hdr;
	struct heci_cl *cl_pos = NULL;
	struct heci_cl *cl_next = NULL;
	int ret = 0;

	if (!dev->rd_msg_hdr) {
		dev->rd_msg_hdr = heci_read_hdr(dev);
		dev_dbg(&dev->pdev->dev, "slots =%08x.\n", *slots);
		(*slots)--;
		dev_dbg(&dev->pdev->dev, "slots =%08x.\n", *slots);
	}
	heci_hdr = (struct heci_msg_hdr *) &dev->rd_msg_hdr;
	dev_dbg(&dev->pdev->dev, HECI_HDR_FMT, HECI_HDR_PRM(heci_hdr));

	if (heci_hdr->reserved || !dev->rd_msg_hdr) {
		dev_dbg(&dev->pdev->dev, "corrupted message header.\n");
		ret = -EBADMSG;
		goto end;
	}

	if (heci_hdr->host_addr || heci_hdr->me_addr) {
		list_for_each_entry_safe(cl_pos, cl_next,
					&dev->file_list, link) {
			dev_dbg(&dev->pdev->dev,
					"list_for_each_entry_safe read host"
					" client = %d, ME client = %d\n",
					cl_pos->host_client_id,
					cl_pos->me_client_id);
			if (heci_cl_hbm_equal(cl_pos, heci_hdr))
				break;
		}

		if (&cl_pos->link == &dev->file_list) {
			dev_dbg(&dev->pdev->dev, "corrupted message header\n");
			ret = -EBADMSG;
			goto end;
		}
	}
	if (((*slots) * sizeof(u32)) < heci_hdr->length) {
		dev_err(&dev->pdev->dev,
				"we can't read the message slots =%08x.\n",
				*slots);
		/* we can't read the message */
		ret = -ERANGE;
		goto end;
	}

	/* decide where to read the message too */
	if (!heci_hdr->host_addr) {
		dev_dbg(&dev->pdev->dev, "call heci_irq_thread_read_bus_message.\n");
		heci_hbm_dispatch(dev, heci_hdr);
		dev_dbg(&dev->pdev->dev, "end heci_irq_thread_read_bus_message.\n");
	} else {
		dev_dbg(&dev->pdev->dev, "call heci_cl_irq_read_msg.\n");
		dev_dbg(&dev->pdev->dev, HECI_HDR_FMT, HECI_HDR_PRM(heci_hdr));
		ret = heci_cl_irq_read_msg(dev, heci_hdr, cmpl_list);
		if (ret)
			goto end;
	}

	/* reset the number of slots and header */
	*slots = heci_count_full_read_slots(dev);
	dev->rd_msg_hdr = 0;

	if (*slots == -EOVERFLOW) {
		/* overflow - reset */
		dev_err(&dev->pdev->dev, "resetting due to slots overflow.\n");
		/* set the event since message has been read */
		ret = -ERANGE;
		goto end;
	}
end:
	return ret;
}
EXPORT_SYMBOL_GPL(heci_irq_read_handler);


/**
 * heci_irq_write_handler -  dispatch write requests
 *  after irq received
 *
 * @dev: the device structure
 * @cmpl_list: An instance of our list structure
 *
 * returns 0 on success, <0 on failure.
 */
int heci_irq_write_handler(struct heci_device *dev, struct heci_cl_cb *cmpl_list)
{
	struct heci_cl *cl;
	struct heci_cl_cb *cb, *next;
	struct heci_cl_cb *list;
	s32 slots;
	int ret;

	if (!heci_hbuf_is_ready(dev)) {
		dev_dbg(&dev->pdev->dev, "host buffer is not empty.\n");
		return 0;
	}
	slots = heci_hbuf_empty_slots(dev);
	if (slots <= 0)
		return -EMSGSIZE;

	/* complete all waiting for write CB */
	dev_dbg(&dev->pdev->dev, "complete all waiting for write cb.\n");

	list = &dev->write_waiting_list;
	list_for_each_entry_safe(cb, next, &list->list, list) {
		cl = cb->cl;
		if (cl == NULL)
			continue;

		cl->status = 0;
		list_del(&cb->list);
		if (HECI_WRITING == cl->writing_state &&
		    cb->fop_type == HECI_FOP_WRITE) {
			dev_dbg(&dev->pdev->dev, "HECI WRITE COMPLETE\n");
			cl->writing_state = HECI_WRITE_COMPLETE;
			list_add_tail(&cb->list, &cmpl_list->list);
		}
	}

	if (dev->wr_ext_msg.hdr.length) {
		heci_write_message(dev, &dev->wr_ext_msg.hdr,
				dev->wr_ext_msg.data);
		slots -= heci_data2slots(dev->wr_ext_msg.hdr.length);
		dev->wr_ext_msg.hdr.length = 0;
	}

	/* complete control write list CB */
	dev_dbg(&dev->pdev->dev, "complete control write list cb.\n");
	list_for_each_entry_safe(cb, next, &dev->ctrl_wr_list.list, list) {
		cl = cb->cl;
		if (!cl) {
			list_del(&cb->list);
			return -ENODEV;
		}
		switch (cb->fop_type) {
		case HECI_FOP_CLOSE:
			/* send disconnect message */
			ret = heci_cl_irq_close(cl, cb, &slots, cmpl_list);
			if (ret)
				return ret;

			break;
		case HECI_FOP_READ:
			/* send flow control message */
			ret = heci_cl_irq_read(cl, cb, &slots, cmpl_list);
			if (ret)
				return ret;

			break;
		case HECI_FOP_IOCTL:
			/* connect message */
			if (heci_cl_is_other_connecting(cl))
				continue;
			ret = heci_cl_irq_ioctl(cl, cb, &slots, cmpl_list);
			if (ret)
				return ret;

			break;

		default:
			BUG();
		}

	}
	/* complete  write list CB */
	dev_dbg(&dev->pdev->dev, "complete write list cb.\n");
	list_for_each_entry_safe(cb, next, &dev->write_list.list, list) {
		cl = cb->cl;
		if (cl == NULL)
			continue;
		if (heci_cl_flow_ctrl_creds(cl) <= 0) {
			dev_dbg(&dev->pdev->dev,
				"No flow control credentials for client %d, not sending.\n",
				cl->host_client_id);
			continue;
		}

		ret = heci_cl_irq_write_complete(cl, cb, &slots, cmpl_list);
		if (ret)
			return ret;
	}
	return 0;
}
EXPORT_SYMBOL_GPL(heci_irq_write_handler);



/**
 * heci_timer - timer function.
 *
 * @work: pointer to the work_struct structure
 *
 * NOTE: This function is called by timer interrupt work
 */
void heci_timer(struct work_struct *work)
{
	unsigned long timeout;
	struct heci_cl *cl_pos = NULL;
	struct heci_cl *cl_next = NULL;
	struct heci_cl_cb  *cb_pos = NULL;
	struct heci_cl_cb  *cb_next = NULL;

	struct heci_device *dev = container_of(work,
					struct heci_device, timer_work.work);

	mutex_lock(&dev->device_lock);
	if (dev->dev_state != HECI_DEV_ENABLED) {
		if (dev->dev_state == HECI_DEV_INIT_CLIENTS) {
			if (dev->init_clients_timer) {
				if (--dev->init_clients_timer == 0) {
					dev_err(&dev->pdev->dev, "reset: init clients timeout hbm_state = %d.\n",
						dev->hbm_state);
					heci_reset(dev, 1);
				}
			}
		}
		goto out;
	}

	/*** connect/disconnect timeouts ***/
	list_for_each_entry_safe(cl_pos, cl_next, &dev->file_list, link) {
		if (cl_pos->timer_count) {
			if (--cl_pos->timer_count == 0) {
				dev_err(&dev->pdev->dev, "reset: connect/disconnect timeout.\n");
				heci_reset(dev, 1);
				goto out;
			}
		}
	}

out:
	schedule_delayed_work(&dev->timer_work, 2 * HZ);
	mutex_unlock(&dev->device_lock);
}
