/*
 * Gadget Driver for Android DvC.Trace Debug Capability
 *
 * Copyright (C) 2008-2010, Intel Corporation.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": %s: " fmt "\n", __func__

#include <linux/module.h>
#include <linux/init.h>
#include <linux/poll.h>
#include <linux/delay.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/sched.h>
#include <linux/types.h>
#include <linux/device.h>
#include <linux/miscdevice.h>
#include <linux/usb/debug.h>
#include <linux/usb/dvctrace_io.h>

#include <npk_pci.h>
#include <npk_trace.h>

/********************************
 *        API functions         *
 ********************************/

struct npk_mw_buffer {
	struct msu_win_ctx ctx;
	struct scatterlist *sg_array;

	int (*allocate)(void *);
	int (*cleanup)(void *);

	int (*default_payload)(void *, struct usb_request *);
	int (*get_next_window)(void *, struct usb_request *);
};

int npk_api_allocate(void *container)
{
	struct npk_mw_buffer *npk_ctx = (struct npk_mw_buffer *)container;
	int ret;

	if (!npk_ctx) {
		pr_err("No npk context");
		return -EINVAL;
	}

	ret = npk_win_get_ctx(&npk_ctx->ctx);
	if (ret || !npk_ctx->ctx.nr_wins) {
		pr_err("No npk window context");
		return -ENXIO;
	}

	npk_ctx->sg_array = kzalloc(npk_ctx->ctx.nr_blks *
				    sizeof(struct scatterlist), GFP_KERNEL);
	if (!npk_ctx->sg_array) {
		pr_err("Cannot allocate sg_array");
		return -ENOMEM;
	}

	sg_init_table(npk_ctx->sg_array, npk_ctx->ctx.nr_blks);

	return 0;
}

int npk_api_free(void *container)
{
	struct npk_mw_buffer *npk_ctx = (struct npk_mw_buffer *)container;

	if (!npk_ctx) {
		pr_err("No npk context");
		return -EINVAL;
	};

	npk_ctx->ctx.nr_wins = 0;
	kfree(npk_ctx->sg_array);
	kfree(npk_ctx);

	return 0;
}

int npk_api_default_payload(void *container, struct usb_request *req)
{

	if (!req) {
		pr_err("NULL request");
		return -EINVAL;
	}

	req->length = 0;		/* keeps the dma and pointer untouched */
	req->buf = 0;			/* any address will do */
	req->dma = (dma_addr_t)0;	/* any address will do */

	return 0;
}

int npk_api_get_next_window(void *container, struct usb_request *req)
{
	struct npk_mw_buffer *npk_buffer = (struct npk_mw_buffer *)container;
	int nb_sg;
	int ret;

	ret = npk_win_get_ctx(&npk_buffer->ctx);

	if (ret || !npk_buffer->ctx.nr_wins) {
		pr_err("No npk window context");
		return -ENXIO;
	}
	if (!npk_buffer || !npk_buffer->ctx.nr_wins) {
		pr_err("Invalid NPK buffer");
		return -EINVAL;
	}

	nb_sg = npk_win_get_ordered_blocks(npk_buffer->ctx.cur_win,
					   npk_buffer->sg_array);
	if (nb_sg <= 0) {
		pr_err("Get blocks failed %d", nb_sg);
		return -1;
	}

	req->sg = npk_buffer->sg_array;
	req->num_sgs = nb_sg;
	req->length = npk_buffer->sg_array->length;

	return 0;
}

/********************************
 *    End of API functions      *
 ********************************/


static struct npk_mw_buffer _npk_mw_buffer = {
	.ctx			= {0,},
	.allocate		= npk_api_allocate,
	.cleanup		= npk_api_free,
	.default_payload	= npk_api_default_payload,
	.get_next_window	= npk_api_get_next_window,
};

enum npk_tranfer_type {
	npk_t_data,
	npk_t_sync
};

struct dvc_io_npk {
	spinlock_t lock;

	struct usb_ep *ep_in;
	struct usb_function *ctx;
	struct dvc_io *dops;
	struct npk_mw_buffer *npk_buf;
	atomic_t *dvc_io_status;
	atomic_t one_req;

	struct usb_request *req_data;

	struct workqueue_struct *work_queue;
	struct work_struct *queue_data_work;
	enum npk_tranfer_type next_tranfer_type;
};

static struct dvc_io_npk *_npk_dev;

/****************************************************
 *
 *		UTILITY FUNCTIONS
 *
 ****************************************************/

static inline int npk_dvc_io_lock(atomic_t *excl)
{
	if (atomic_inc_return(excl) == 1) {
		return 0;
	} else {
		atomic_dec(excl);
		return -1;
	}
}

static inline void npk_dvc_io_unlock(atomic_t *excl)
{
	atomic_dec(excl);
}


/****************************************************
 *
 *	Request control for DVC.Trace
 *
 ****************************************************/

static void data_completed(struct usb_ep *ep, struct usb_request *req)
{
	if (!_npk_dev)
		return;
	npk_dvc_io_unlock(&_npk_dev->one_req);
	if (req->status != 0) {
		DVC_IO_STATUS_CLEAR(_npk_dev->dvc_io_status,
				    DVC_IO_MASK_ONLINE_T);
		pr_err("Request ended with status: %d ", req->status);
	}

	if (_npk_dev->dvc_io_status) {
		unsigned int ret;
		ret =
		    DVC_IO_STATUS_GET(_npk_dev->dvc_io_status,
				      DVC_IO_MASK_ONLINE_T);
		if (ret == DVC_IO_MASK_ONLINE_T)
			queue_work(_npk_dev->work_queue,
				   _npk_dev->queue_data_work);
		else
			pr_debug("No longer online and transferring");
	}
}

static int data_payload(struct usb_request *req)
{
	int ret;

	if (!_npk_dev) {
		pr_err("No npk device");
		return -ENODEV;
	}

	if (!req || !_npk_dev->npk_buf || !_npk_dev->npk_buf->ctx.nr_wins) {
		pr_err("No npk buffer");
		return -EINVAL;
	}

	req->complete = data_completed;
	req->context = _npk_dev->ctx;

	if (!_npk_dev->npk_buf->get_next_window) {
		pr_err("Cannot get window,invalid callback: %p",
		       _npk_dev->npk_buf->get_next_window);
		return -EINVAL;
	}

	ret = _npk_dev->npk_buf->get_next_window(_npk_dev->npk_buf, req);

	if (ret < 0)
		pr_err("Get window failed: %d", ret);

	return ret;
}

static int sync_payload(struct usb_request *req)
{
	if (!_npk_dev)
		return -ENODEV;

	if (!req || !_npk_dev->npk_buf)
		return -EINVAL;

	req->complete = data_completed;
	req->context = _npk_dev->ctx;

	if (_npk_dev->npk_buf->default_payload)
		return _npk_dev->npk_buf->default_payload(_npk_dev->npk_buf,
							  req);
	return -EINVAL;
}

static void queue_data_request(struct work_struct *work)
{
	int ret = -EINVAL;

	if (!_npk_dev) {
		pr_err("No npk device");
		return;
	}

	if (!_npk_dev->ep_in || !_npk_dev->req_data) {
		pr_err("Invalid endpoint data");
		return;
	}

	if (npk_dvc_io_lock(&_npk_dev->one_req)) {
		/*if the transfer was cancel between queue and execution */
		ret =
		    DVC_IO_STATUS_GET(_npk_dev->dvc_io_status,
				      DVC_IO_MASK_ONLINE_T);
		if (ret == DVC_IO_MASK_ONLINE_T)
			pr_err("Cannot lock");
		else {
			pr_info("No longer online and transferring");
			npk_dvc_io_unlock(&_npk_dev->one_req);
		}
		return;
	}
	if (_npk_dev->next_tranfer_type == npk_t_data) {
		if (data_payload(_npk_dev->req_data)) {
			pr_err("Data payload failed");
			return;
		}
		_npk_dev->next_tranfer_type = npk_t_sync;
	} else if (_npk_dev->next_tranfer_type == npk_t_sync) {
		if (sync_payload(_npk_dev->req_data)) {
			pr_err("Sync payload failed");
			return;
		}
		_npk_dev->next_tranfer_type = npk_t_data;
	} else {
		pr_err("INVALID transfer type");
		return;
	}

	ret = usb_ep_queue(_npk_dev->ep_in, _npk_dev->req_data, GFP_ATOMIC);
	if (ret < 0)
		pr_err("xfer error %d", ret);
}

static void npk_usb_request_free(struct usb_ep *ep, struct usb_request *req)
{
	if (req && ep)
		usb_ep_free_request(ep, req);
}

static int npk_prepare_usb_requests(void)
{
	if (!_npk_dev || !_npk_dev->ep_in) {
		pr_err("Invalid npk device or enpoint");
		return -ENODEV;
	}

	_npk_dev->req_data = usb_ep_alloc_request(_npk_dev->ep_in, GFP_KERNEL);
	if (!_npk_dev->req_data) {
		npk_usb_request_free(_npk_dev->ep_in, _npk_dev->req_data);
		_npk_dev->req_data = NULL;
		pr_err("Cannot allocate usb request");
		return -ENOMEM;
	}
	return 0;
}

/****************************************************
 *
 *	NPK_IO entry points, accessed by pointers
 *
 ****************************************************/

static int npk_dvc_io_on_dvc_setup(atomic_t *dvc_status)
{
	if (!_npk_dev) {
		pr_err("No _npk_dev");
		return -ENODEV;
	}
	_npk_dev->dvc_io_status = dvc_status;

	/*setup work */

	BUG_ON(_npk_dev->work_queue != NULL);

	_npk_dev->work_queue = create_workqueue("NPK-dvc-wq");
	if (!_npk_dev->work_queue) {
		pr_err("Cannot allocate work queue");
		return -ENOMEM;
	}

	BUG_ON(_npk_dev->queue_data_work != NULL);

	_npk_dev->queue_data_work =
	    kmalloc(sizeof(struct work_struct), GFP_KERNEL);
	if (!_npk_dev->queue_data_work) {
		pr_err("Cannot allocate work queue");
		return -ENOMEM;
	}

	INIT_WORK(_npk_dev->queue_data_work, queue_data_request);
	return 0;
}

static int npk_dvc_io_on_endpoint_creation(struct usb_ep *ep,
					   struct usb_function *ctx)
{
	int ret;
	if (!_npk_dev)
		return -ENODEV;
	if (!ep || !ctx)
		return -EINVAL;

	_npk_dev->ep_in = ep;
	_npk_dev->ctx = ctx;

	_npk_dev->npk_buf = &_npk_mw_buffer;

	if (!_npk_dev->npk_buf) {
		pr_err("No npk_mw_buffer");
		return -ENOMEM;
	}

	if (!_npk_dev->npk_buf->allocate) {
		pr_err("Invalid npk_mw_buffer");
		return -EINVAL;
	}

	ret = _npk_dev->npk_buf->allocate(_npk_dev->npk_buf);
	if (ret) {
		pr_err("npk_mw_buffer-> allocate failed with %d", ret);
		return -ENOMEM;
	}

	ret = npk_prepare_usb_requests();
	if (ret)
		pr_err("npk_prepare_usb_requests failed with %d", ret);

	return ret;
}

static int npk_dvc_io_on_start_transfer(int ignored)
{
	DVC_IO_STATUS_SET(_npk_dev->dvc_io_status, DVC_IO_MASK_TRANSFER);

	if (atomic_read(&_npk_dev->one_req))
		pr_warn("One request ongoing");

	if (!queue_work(_npk_dev->work_queue, _npk_dev->queue_data_work))
		pr_warn("Work already queued");

	return 0;
}

static void npk_dvc_io_on_dvc_cleanup(void)
{
	_npk_dev->npk_buf->cleanup(_npk_dev->npk_buf);

	flush_workqueue(_npk_dev->work_queue);
	destroy_workqueue(_npk_dev->work_queue);

	_npk_dev->work_queue = NULL;

	kfree(_npk_dev->queue_data_work);
	_npk_dev->queue_data_work = NULL;

	return;
}

static int npk_dvc_io_on_disable_transfer(void)
{
	pr_info("disable transfer");
	return 0;
}

static void npk_dvc_io_on_dvc_unbind(void)
{
	if (!_npk_dev || !_npk_dev->ep_in || !_npk_dev->req_data)
		return;

	npk_usb_request_free(_npk_dev->ep_in, _npk_dev->req_data);
}

static int npk_dvc_io_is_enabled(void)
{
	struct msu_win_ctx ctx = { 0, };

	if (npk_win_get_ctx(&ctx)) {
		pr_warn("npk_win_get_ctx FAILED");
		return 0;
	}

	return ctx.nr_wins && ctx.nr_blks;
}

/****************************************************
 *
 *	NPK_IO structure, accessed by DvC.Trace
 *
 ****************************************************/

static struct dvc_io npk_ops = {
	.on_dvc_setup = npk_dvc_io_on_dvc_setup,
	.on_endpoint_creation = npk_dvc_io_on_endpoint_creation,
	.on_start_transfer = npk_dvc_io_on_start_transfer,
	.on_dvc_cleanup = npk_dvc_io_on_dvc_cleanup,
	.on_disable_transfer = npk_dvc_io_on_disable_transfer,
	.on_dvc_unbind = npk_dvc_io_on_dvc_unbind,
	.is_io_enabled = npk_dvc_io_is_enabled,
};

static int __init npk_dvc_io_init(void)
{
	struct dvc_io_npk *dev;
	int ret;

	dev = kzalloc(sizeof(*dev), GFP_KERNEL);
	if (!dev) {
		pr_err("Cannot allocate device");
		return -ENOMEM;
	}

	spin_lock_init(&dev->lock);
	atomic_set(&dev->one_req, 0);
	dev->dops = &npk_ops;
	dev->next_tranfer_type = npk_t_sync;

	_npk_dev = dev;

	ret = dvc_io_register(dev->dops);
	if (ret)
		pr_err("Cannot register DvC-io device (%d)", ret);

	return ret;
}

static void __exit npk_dvc_io_exit(void)
{
	dvc_io_unregister(_npk_dev->dops);
	kfree(_npk_dev);
}

module_init(npk_dvc_io_init);
module_exit(npk_dvc_io_exit);

MODULE_AUTHOR("Traian Schiau <traianx.schiau@intel.com>");
MODULE_DESCRIPTION("Intel DVC_IO npk driver");
MODULE_LICENSE("GPL v2");
