/*
 *  Android DvC.Trace Debug Interface for STM
 *
 * Copyright (C) 2014, Intel Corporation.
 *
 * This software is licensed under the terms of the GNU General Public
 * License version 2, as published by the Free Software Foundation, and
 * may be copied, distributed, and modified under those terms.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt, __func__

#include <linux/module.h>
#include <linux/usb/dvctrace_io.h>
#include <linux/sdm.h>
#include <linux/sched.h>

#define TRACE_TX_REQ_MAX 3

struct stm_dvc_trace_dev {
	struct usb_ep *ep_in;
	struct usb_function *function;

	spinlock_t lock;

	struct list_head tx_idle;
	struct list_head tx_xfer;

	atomic_t *dvc_io_status;
	wait_queue_head_t write_wq;
};


struct stm_dvc_trace_dev *dev;

static struct usb_request *stm_dvc_trace_request_new(struct usb_ep *ep,
		int buffer_size, dma_addr_t dma)
{
	struct usb_request *req = usb_ep_alloc_request(ep, GFP_KERNEL);
	if (!req)
		return NULL;

	req->dma = dma;
	/* now allocate buffers for the requests */
	req->buf = kmalloc(buffer_size, GFP_KERNEL);
	if (!req->buf) {
		usb_ep_free_request(ep, req);
		return NULL;
	}

	return req;
}

/* add a request to the tail of a list */
static void stm_dvc_trace_req_put(struct stm_dvc_trace_dev *dev, struct list_head *head,
		struct usb_request *req)
{
	unsigned long flags;

	spin_lock_irqsave(&dev->lock, flags);
	list_add_tail(&req->list, head);
	spin_unlock_irqrestore(&dev->lock, flags);
}

/* remove a request from the head of a list */
static struct usb_request *stm_dvc_trace_req_get(struct stm_dvc_trace_dev *dev,
			struct list_head *head)
{
	unsigned long flags;
	struct usb_request *req;

	spin_lock_irqsave(&dev->lock, flags);
	if (list_empty(head)) {
		req = 0;
	} else {
		req = list_first_entry(head, struct usb_request, list);
		list_del(&req->list);
	}
	spin_unlock_irqrestore(&dev->lock, flags);
	return req;
}

static void stm_dvc_trace_request_free(struct usb_request *req, struct usb_ep *ep)
{
	if (req) {
		kfree(req->buf);
		usb_ep_free_request(ep, req);
	}
}

static void stm_dvc_trace_complete_in(struct usb_ep *ep, struct usb_request *req)
{
	if (req->status != 0)
		DVC_IO_STATUS_CLEAR(dev->dvc_io_status, DVC_IO_MASK_TRANSFER);

	stm_dvc_trace_req_put(dev, &dev->tx_idle, req);

	wake_up(&dev->write_wq);
}

static int stm_dvc_io_on_dvc_setup(atomic_t *dvc_status)
{
	dev->dvc_io_status = dvc_status;
	/*might need some other checks */
	return 0;
}

static int stm_dvc_io_on_endpoint_creation(struct usb_ep *ep, struct usb_function *function)
{
	int i;
	struct usb_request *req;

	dev->ep_in = ep;
	dev->function = function;
	for (i = 0; i < TRACE_TX_REQ_MAX; i++) {
		req = stm_dvc_trace_request_new(dev->ep_in, TRACE_BULK_BUFFER_SIZE,
			(dma_addr_t)TRACE_BULK_IN_BUFFER_ADDR);
		if (!req)
			goto fail;
		req->complete = stm_dvc_trace_complete_in;
		stm_dvc_trace_req_put(dev, &dev->tx_idle, req);
		pr_debug("req= %p : for %s predefined TRB\n",
			req, ep->name);
	}

	return 0;

fail:
	pr_err("could not allocate requests\n");
	return -EINVAL;
}

static int stm_dvc_io_on_start_transfer(int count)
{
	int r = count, xfer;
	int ret = -EINVAL;
	struct usb_request *req = 0;

	while (r > 0) {
		if (DVC_IO_STATUS_GET(dev->dvc_io_status, DVC_IO_MASK_ERROR)) {
			pr_debug("dev->error\n");
			r = -EIO;
			break;
		}

		if (!DVC_IO_STATUS_GET(dev->dvc_io_status, DVC_IO_MASK_ONLINE)) {
			pr_debug("!dev->online\n");
			r = -EIO;
			break;
		}

		/* get an idle tx request to use */
		req = 0;
		ret = wait_event_interruptible(dev->write_wq,
				DVC_IO_STATUS_GET(dev->dvc_io_status, DVC_IO_MASK_ERROR) ||
				!DVC_IO_STATUS_GET(dev->dvc_io_status, DVC_IO_MASK_ONLINE) ||
				(req = stm_dvc_trace_req_get(dev, &dev->tx_idle)));

		if (ret < 0) {
			r = ret;
			break;
		}

		if (req != 0) {
			if (count > TRACE_BULK_BUFFER_SIZE)
				xfer = TRACE_BULK_BUFFER_SIZE;
			else
				xfer = count;
			pr_debug("queue tx_idle list req to dev->ep_in\n");
			req->no_interrupt = 1;
			req->context = &dev->function;
			req->length = xfer;
			ret = usb_ep_queue(dev->ep_in, req, GFP_ATOMIC);
			if (ret < 0) {
				pr_err("xfer error %d\n", ret);
				DVC_IO_STATUS_CLEAR(dev->dvc_io_status, DVC_IO_MASK_TRANSFER);
				DVC_IO_STATUS_SET(dev->dvc_io_status, DVC_IO_MASK_ERROR);
				r = -EIO;
				break;
			}
			pr_debug("xfer=%d/%d  queued req/%p\n", xfer, r, req);
			stm_dvc_trace_req_put(dev, &dev->tx_xfer, req);
			r -= xfer;

			/* zero this so we don't try to free it on error exit */
			req = 0;
		}
	}
	if (req) {
		pr_debug("req re-added to tx_idle on error\n");
		stm_dvc_trace_req_put(dev, &dev->tx_idle, req);
	} else {
		DVC_IO_STATUS_SET(dev->dvc_io_status, DVC_IO_MASK_TRANSFER);
	}
	return ret;
}

static void stm_dvc_io_on_dvc_cleanup(void)
{
	/*clean ep, fnc, status since will be freed from dvc*/

	dev->ep_in = NULL;
	dev->function = NULL;
	dev->dvc_io_status = NULL;
}

static int stm_dvc_io_on_disable_transfer(void)
{
	struct usb_request *req = NULL;
	int ret;

	/* get an xfer tx request to use */
	while ((req = stm_dvc_trace_req_get(dev, &dev->tx_xfer))) {
		ret = usb_ep_dequeue(dev->ep_in, req);
		if (ret < 0) {
			pr_err("dequeue error %d\n", ret);
			DVC_IO_STATUS_SET(dev->dvc_io_status, DVC_IO_MASK_ERROR);
			return -EIO;
		}
		pr_debug("dequeued req/%p\n", req);
	}
	return 0;
}
static void stm_dvc_io_on_dvc_unbind(void)
{
	struct usb_request *req = NULL;
	while ((req = stm_dvc_trace_req_get(dev, &dev->tx_idle)))
			stm_dvc_trace_request_free(req, dev->ep_in);
}

static int stm_dvc_io_is_enabled(void)
{
	if (!stm_is_enabled()) {
		pr_info("STM/PTI block is not enabled\n");
		return 0;
	}
	return 1;
}


static struct dvc_io stm_ops = {
	.on_dvc_setup         = stm_dvc_io_on_dvc_setup,
	.on_endpoint_creation = stm_dvc_io_on_endpoint_creation,
	.on_start_transfer    = stm_dvc_io_on_start_transfer,
	.on_dvc_cleanup       = stm_dvc_io_on_dvc_cleanup,
	.on_disable_transfer  = stm_dvc_io_on_disable_transfer,
	.on_dvc_unbind        = stm_dvc_io_on_dvc_unbind,
	.is_io_enabled        = stm_dvc_io_is_enabled,
};


static int __init stm_dvc_io_init(void)
{
	pr_info("module init\n");

	dev = kzalloc(sizeof(struct stm_dvc_trace_dev), GFP_KERNEL);
	if (!dev)
		return -ENOMEM;

	INIT_LIST_HEAD(&dev->tx_idle);
	INIT_LIST_HEAD(&dev->tx_xfer);

	spin_lock_init(&dev->lock);
	init_waitqueue_head(&dev->write_wq);

	return dvc_io_register(&stm_ops);
}


static void __exit stm_dvc_io_exit(void)
{
	dvc_io_unregister(&stm_ops);
	kfree(dev);
}

module_init(stm_dvc_io_init);
module_exit(stm_dvc_io_exit);

MODULE_AUTHOR("Traian Schiau <traianx.schiau@intel.com>");
MODULE_DESCRIPTION("Intel DVC_IO STM driver");
MODULE_LICENSE("GPL v2");
