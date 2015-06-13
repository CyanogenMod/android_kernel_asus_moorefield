/*
 * HECI bus driver
 *
 * Copyright (c) 2012-2014, Intel Corporation.
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

#include <linux/module.h>
#include <linux/device.h>
#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/errno.h>
#include <linux/slab.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/pci.h>
#include "heci_cl_bus.h"
#include "heci_dev.h"
#include "client.h"
#include <asm/page.h>
#include "hbm.h"
#include "utils.h"

#define to_heci_cl_driver(d) container_of(d, struct heci_cl_driver, driver)
#define to_heci_cl_device(d) container_of(d, struct heci_cl_device, dev)

int	host_dma_enabled;
void	*host_dma_buf;
unsigned	host_dma_buf_size = (1024*1024);
uint64_t	host_dma_buf_phys;
int	dma_ready = 1;

static int heci_cl_device_match(struct device *dev, struct device_driver *drv)
{
	struct heci_cl_device *device = to_heci_cl_device(dev);
	struct heci_cl_driver *driver = to_heci_cl_driver(drv);
	const struct heci_cl_device_id *id;

/*
 * DD -- return true and let driver's probe() routine decide.
 * If this solution lives up, we can rearrange it
 * by simply removing match() routine at all
 */
	return	1;

#if 0
	if (!device)
		return 0;

	if (!driver || !driver->id_table)
		return 0;

	id = driver->id_table;

	while (id->name[0]) {
		if (!strcmp(dev_name(dev), id->name))
			return 1;

		id++;
	}
#endif
	return 0;
}

static int heci_cl_device_probe(struct device *dev)
{
	struct heci_cl_device *device = to_heci_cl_device(dev);
	struct heci_cl_driver *driver;
	struct heci_cl_device_id id;

	if (!device)
		return 0;

	/* in many cases here will be NULL */
	driver = to_heci_cl_driver(dev->driver);
	if (!driver || !driver->probe)
		return -ENODEV;

	dev_dbg(dev, "Device probe\n");

	strncpy(id.name, dev_name(dev), HECI_CL_NAME_SIZE);

	return driver->probe(device, &id);
}

static int heci_cl_device_remove(struct device *dev)
{
	struct heci_cl_device *device = to_heci_cl_device(dev);
	struct heci_cl_driver *driver;

	if (!device || !dev->driver)
		return 0;

	if (device->event_cb) {
		device->event_cb = NULL;
		cancel_work_sync(&device->event_work);
	}

	driver = to_heci_cl_driver(dev->driver);
	if (!driver->remove) {
		dev->driver = NULL;

		return 0;
	}

	return driver->remove(device);
}

static ssize_t modalias_show(struct device *dev,
			struct device_attribute *a, char *buf)
{
	int len;

	len = snprintf(buf, PAGE_SIZE, "heci:%s\n", dev_name(dev));
	return (len >= PAGE_SIZE) ? (PAGE_SIZE - 1) : len;
}

static struct device_attribute heci_cl_dev_attrs[] = {
	__ATTR_RO(modalias),
	__ATTR_NULL,
};

static int heci_cl_uevent(struct device *dev,
			struct kobj_uevent_env *env)
{
	if (add_uevent_var(env, "MODALIAS=heci:%s", dev_name(dev)))
		return -ENOMEM;

	return 0;
}

static struct bus_type heci_cl_bus_type = {
	.name		= "heci",
	.dev_attrs	= heci_cl_dev_attrs,
	.match		= heci_cl_device_match,
	.probe		= heci_cl_device_probe,
	.remove		= heci_cl_device_remove,
	.uevent		= heci_cl_uevent,
};

static void heci_cl_dev_release(struct device *dev)
{
	kfree(to_heci_cl_device(dev));
}

static struct device_type heci_cl_device_type = {
	.release	= heci_cl_dev_release,
};

static struct heci_cl *heci_bus_find_heci_cl_by_uuid(struct heci_device *dev,
							uuid_le uuid)
{
	struct heci_cl *cl, *next;

	list_for_each_entry_safe(cl, next, &dev->device_list, device_link) {
		if (!uuid_le_cmp(uuid, cl->device_uuid))
			return cl;
	}

	return NULL;
}


/*
 * Allocate HECI bus client device,
 * attach it to uuid and register with HECI bus
 */
struct heci_cl_device *heci_cl_add_device(struct heci_device *dev,
			uuid_le uuid, char *name, struct heci_cl_ops *ops)
{
	struct heci_cl_device *device;
	struct heci_cl *cl;
	int status;

	cl = heci_bus_find_heci_cl_by_uuid(dev, uuid);
	if (cl == NULL)
		return NULL;

	device = kzalloc(sizeof(struct heci_cl_device), GFP_KERNEL);
	if (!device)
		return NULL;

	device->cl = cl;
	device->ops = ops;

	device->dev.parent = &dev->pdev->dev;
	device->dev.bus = &heci_cl_bus_type;
	device->dev.type = &heci_cl_device_type;

	dev_set_name(&device->dev, "%s", name);

	status = device_register(&device->dev);
	if (status) {
		dev_err(&dev->pdev->dev, "Failed to register HECI device\n");
		kfree(device);
		return NULL;
	}

	cl->device = device;

	dev_dbg(&device->dev, "client %s registered\n", name);

	return device;
}
EXPORT_SYMBOL_GPL(heci_cl_add_device);


/*
 * This is a counterpart of heci_cl_add_device.
 * Device is unregistered and its structure is also freed
 */
void heci_cl_remove_device(struct heci_cl_device *device)
{
	device_unregister(&device->dev);
	/*kfree(device);*/
}
EXPORT_SYMBOL_GPL(heci_cl_remove_device);


/*
 * Part of reset flow
 */
void	heci_bus_remove_all_clients(struct heci_device *heci_dev)
{
	struct heci_cl_device *heci_cl_dev;
	struct heci_cl	*cl, *next;

	list_for_each_entry_safe(cl, next, &heci_dev->device_list, device_link) {
		heci_cl_dev = cl->device;

		/*
		 * Wake any pending process.
		 * The waiter would check dev->state and determine
		 * that it's not enabled already,
		 * and will return error to its caller
		 */
		if (waitqueue_active(&cl->rx_wait))
			wake_up_interruptible(&cl->rx_wait);

		/* Disband any pending read/write requests */
		mutex_lock(&heci_dev->device_lock);

		/* Flush queues and remove any pending read */
		heci_cl_flush_queues(cl);

		if (cl->read_cb) {
			struct heci_cl_cb *cb = NULL;

			cb = heci_cl_find_read_cb(cl);
			/* Remove entry from read list */
			if (cb)
				list_del(&cb->list);

			cb = cl->read_cb;
			cl->read_cb = NULL;

			if (cb) {
				heci_io_cb_free(cb);
				cb = NULL;
			}
		}
		mutex_unlock(&heci_dev->device_lock);

		/* Unregister HECI bus client device */
		heci_cl_remove_device(heci_cl_dev);

		/* Free client and HECI bus client device structures */
		kfree(cl);
	}
	if (waitqueue_active(&heci_dev->wait_recvd_msg))
		wake_up(&heci_dev->wait_recvd_msg);

	/* Free all client structures */
	kfree(heci_dev->me_clients);
	heci_dev->me_clients = NULL;
	heci_dev->me_clients_num = 0;
	heci_dev->me_client_presentation_num  = 0;
	heci_dev->me_client_index = 0;
	bitmap_zero(heci_dev->me_clients_map, HECI_CLIENTS_MAX);
	bitmap_zero(heci_dev->host_clients_map, HECI_CLIENTS_MAX);
	bitmap_set(heci_dev->host_clients_map, 0, 3);
}
EXPORT_SYMBOL_GPL(heci_bus_remove_all_clients);


int __heci_cl_driver_register(struct heci_cl_driver *driver,
				struct module *owner)
{
	int err;

	driver->driver.name = driver->name;
	driver->driver.owner = owner;
	driver->driver.bus = &heci_cl_bus_type;

	err = driver_register(&driver->driver);
	if (err)
		return err;

	pr_debug("heci: driver [%s] registered\n", driver->driver.name);
	return 0;
}
EXPORT_SYMBOL_GPL(__heci_cl_driver_register);

void heci_cl_driver_unregister(struct heci_cl_driver *driver)
{
	driver_unregister(&driver->driver);

	pr_debug("heci: driver [%s] unregistered\n", driver->driver.name);
}
EXPORT_SYMBOL_GPL(heci_cl_driver_unregister);

#if 0
static int ___heci_cl_send(struct heci_cl *cl, u8 *buf,
				size_t length, bool blocking)
{
	struct heci_device *dev;
	struct heci_cl_cb *cb;
	int id;
	int rets;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;
	if (cl->state != HECI_FILE_CONNECTED)
		return -ENODEV;

	/* Check if we have an ME client device */
	id = heci_me_cl_by_id(dev, cl->me_client_id);
	if (id < 0)
		return -ENODEV;

	if (length > dev->me_clients[id].props.max_msg_length) {
		/* If the client supports DMA, try to use it */
		if (host_dma_enabled && dev->me_clients[id].props.dma_hdr_len & HECI_CLIENT_DMA_ENABLED) {
			struct heci_msg_hdr hdr;
			struct hbm_client_dma_request heci_dma_request_msg;
			unsigned len = sizeof(struct hbm_client_dma_request);
			int preview_len = dev->me_clients[id].props.dma_hdr_len & 0x7F;

			/* DMA max msg size is 1M */
			if (length > host_dma_buf_size)
				return	-EMSGSIZE;

			/* Client for some reason specified props.dma_hdr_len > 12, mistake? */
			if (preview_len > 12)
				return	-EINVAL;

			/* Todo: if previous DMA transfer is in progress, go to sleep */
			wait_event(dev->wait_dma_ready, dma_ready);
			dma_ready = 0;
			/* First 'preview_len' bytes of buffer are preview bytes, omitted from DMA message */
			memcpy(host_dma_buf, buf + preview_len, length - preview_len);
			heci_hbm_hdr(&hdr, len);
			heci_dma_request_msg.hbm_cmd = CLIENT_DMA_REQ_CMD;
			heci_dma_request_msg.me_addr = cl->me_client_id;
			heci_dma_request_msg.host_addr = cl->host_client_id;
			heci_dma_request_msg.reserved = 0;
			heci_dma_request_msg.msg_addr = host_dma_buf_phys;
			heci_dma_request_msg.msg_len = length - preview_len;
			heci_dma_request_msg.reserved2 = 0;
			memcpy(heci_dma_request_msg.msg_preview, buf, preview_len);
			heci_write_message(dev, &hdr, &heci_dma_request_msg);
		} else
			return -EINVAL;		/* -EMSGSIZE? */
	}

	cb = heci_io_cb_init(cl, NULL);
	if (!cb)
		return -ENOMEM;

	rets = heci_io_cb_alloc_req_buf(cb, length);
	if (rets < 0) {
		heci_io_cb_free(cb);
		return rets;
	}

	memcpy(cb->request_buffer.data, buf, length);
	mutex_lock(&dev->device_lock);
	rets = heci_cl_write(cl, cb, blocking);
	mutex_unlock(&dev->device_lock);

	if (rets < 0)
		heci_io_cb_free(cb);

	return rets;
}

#else

static int ___heci_cl_send(struct heci_cl *cl, u8 *buf,
				size_t length, bool blocking)
{
	struct heci_device *dev;
#if 0
	struct heci_cl_cb *cb;
#endif
	int id;
	int rets;
	struct heci_msg_hdr heci_hdr;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;
	if (cl->state != HECI_FILE_CONNECTED)
		return -ENODEV;

	/* Check if we have an ME client device */
	id = heci_me_cl_by_id(dev, cl->me_client_id);
	if (id < 0)
		return -ENODEV;

	if (length > dev->me_clients[id].props.max_msg_length) {
		/* If the client supports DMA, try to use it */
		if (host_dma_enabled && dev->me_clients[id].props.dma_hdr_len & HECI_CLIENT_DMA_ENABLED) {
			struct heci_msg_hdr hdr;
			struct hbm_client_dma_request heci_dma_request_msg;
			unsigned len = sizeof(struct hbm_client_dma_request);
			int preview_len = dev->me_clients[id].props.dma_hdr_len & 0x7F;

			/* DMA max msg size is 1M */
			if (length > host_dma_buf_size)
				return	-EMSGSIZE;

			/* Client for some reason specified props.dma_hdr_len > 12, mistake? */
			if (preview_len > 12)
				return	-EINVAL;

			/* Todo: if previous DMA transfer is in progress, go to sleep */
			wait_event(dev->wait_dma_ready, dma_ready);
			dma_ready = 0;
			/* First 'preview_len' bytes of buffer are preview bytes, omitted from DMA message */
			memcpy(host_dma_buf, buf + preview_len, length - preview_len);
			heci_hbm_hdr(&hdr, len);
			heci_dma_request_msg.hbm_cmd = CLIENT_DMA_REQ_CMD;
			heci_dma_request_msg.me_addr = cl->me_client_id;
			heci_dma_request_msg.host_addr = cl->host_client_id;
			heci_dma_request_msg.reserved = 0;
			heci_dma_request_msg.msg_addr = host_dma_buf_phys;
			heci_dma_request_msg.msg_len = length - preview_len;
			heci_dma_request_msg.reserved2 = 0;
			memcpy(heci_dma_request_msg.msg_preview, buf, preview_len);
			heci_write_message(dev, &hdr, (uint8_t *)&heci_dma_request_msg);
		} else
			return -EINVAL;		/* -EMSGSIZE? */
	}

#if 0
	cb = heci_io_cb_init(cl, NULL);
	if (!cb)
		return -ENOMEM;

	rets = heci_io_cb_alloc_req_buf(cb, length);
	if (rets < 0) {
		heci_io_cb_free(cb);
		return rets;
	}
	memcpy(cb->request_buffer.data, buf, length);
#endif
	mutex_lock(&dev->device_lock);
#if 0
	rets = heci_cl_write(cl, cb, blocking);
#else
	/* Check flow control creds. If it's 0,
	 * wait up to 3 sec for FW client FC to arrive
	 */
	rets = heci_cl_flow_ctrl_creds(cl);
	if (rets < 0)
		goto	out;

	if (rets == 0) {
		timed_wait_for_timeout(WAIT_FOR_SEND_SLICE,
					cl->heci_flow_ctrl_creds, (3*HZ));
		if (!cl->heci_flow_ctrl_creds) {
			rets = -ETIMEDOUT;
			goto out;
		}
	}

	while (length) {
		/* If doorbell is busy, wait for it to become free, up to 3 sec */
		/* FIXME: is it correct to return -EBUSY and partially send fragmented message? */
		if (!dev->ops->hbuf_is_ready(dev)) {
			timed_wait_for_timeout(WAIT_FOR_SEND_SLICE,
					dev->ops->hbuf_is_ready(dev), (3*HZ));
			if (!dev->ops->hbuf_is_ready(dev))
				return	-EBUSY;
		}

		/* HOST2ISH doorbell OK, go on write message */
		if (length > heci_hbuf_max_len(dev)) {
			heci_hdr.length = heci_hbuf_max_len(dev);
			heci_hdr.msg_complete = 0;
		} else {
			heci_hdr.length = length;
			heci_hdr.msg_complete = 1;
		}

		heci_hdr.host_addr = cl->host_client_id;
		heci_hdr.me_addr = cl->me_client_id;
		heci_hdr.reserved = 0;

		rets = heci_write_message(dev, &heci_hdr, buf);
		if (rets) {
			rets = -EIO;
			goto out;
		}

		length -= heci_hdr.length;
	}

	heci_cl_flow_ctrl_reduce(cl);

#endif
out:
	mutex_unlock(&dev->device_lock);
#if 0
	if (rets < 0)
		heci_io_cb_free(cb);
#endif
	return rets;
}
#endif

int __heci_cl_recv(struct heci_cl *cl, u8 *buf, size_t length)
{
	struct heci_device *dev;
	struct heci_cl_cb *cb;
	size_t r_length;
	int err;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);

	if (!cl->read_cb) {
		err = heci_cl_read_start(cl, length);
		if (err < 0) {
			mutex_unlock(&dev->device_lock);
			return err;
		}
	}

	if (cl->reading_state != HECI_READ_COMPLETE &&
			!waitqueue_active(&cl->rx_wait)) {
		mutex_unlock(&dev->device_lock);

		if (wait_event_interruptible(cl->rx_wait,
			(HECI_READ_COMPLETE == cl->reading_state))) {
			if (signal_pending(current))
				return -EINTR;
			return -ERESTARTSYS;
		}

		mutex_lock(&dev->device_lock);
	}

	cb = cl->read_cb;

	if (cl->reading_state != HECI_READ_COMPLETE) {
		r_length = 0;
		goto out;
	}

	r_length = min_t(size_t, length, cb->buf_idx);

	memcpy(buf, cb->response_buffer.data, r_length);

	heci_io_cb_free(cb);
	cl->reading_state = HECI_IDLE;
	cl->read_cb = NULL;

out:
	mutex_unlock(&dev->device_lock);

	return r_length;
}

inline int __heci_cl_async_send(struct heci_cl *cl,
				u8 *buf, size_t length)
{
	return ___heci_cl_send(cl, buf, length, 0);
}

inline int __heci_cl_send(struct heci_cl *cl, u8 *buf,
				size_t length)
{
	return ___heci_cl_send(cl, buf, length, 1);
}

int heci_cl_send(struct heci_cl_device *device, u8 *buf,
				size_t length)
{
	struct heci_cl *cl = device->cl;

	if (cl == NULL)
		return -ENODEV;

	if (device->ops && device->ops->send)
		return device->ops->send(device, buf, length);

	return __heci_cl_send(cl, buf, length);
}
EXPORT_SYMBOL_GPL(heci_cl_send);

int heci_cl_recv(struct heci_cl_device *device, u8 *buf,
			size_t length)
{
	struct heci_cl *cl =  device->cl;

	if (cl == NULL)
		return -ENODEV;

	if (device->ops && device->ops->recv)
		return device->ops->recv(device, buf, length);

	return __heci_cl_recv(cl, buf, length);
}
EXPORT_SYMBOL_GPL(heci_cl_recv);

static void heci_bus_event_work(struct work_struct *work)
{
	struct heci_cl_device *device;

	device = container_of(work, struct heci_cl_device, event_work);

	if (device->event_cb)
		device->event_cb(device, device->events,
					device->event_context);

	device->events = 0;

	/* Prepare for the next read */
	mutex_lock(&device->cl->dev->device_lock);
	heci_cl_read_start(device->cl, 0);
	mutex_unlock(&device->cl->dev->device_lock);
}

int heci_cl_register_event_cb(struct heci_cl_device *device,
			heci_cl_event_cb_t event_cb, void *context)
{
	if (device->event_cb)
		return -EALREADY;

	device->events = 0;
	device->event_cb = event_cb;
	device->event_context = context;
	INIT_WORK(&device->event_work, heci_bus_event_work);

	mutex_lock(&device->cl->dev->device_lock);
	heci_cl_read_start(device->cl, 0);
	mutex_unlock(&device->cl->dev->device_lock);

	return 0;
}
EXPORT_SYMBOL_GPL(heci_cl_register_event_cb);

void *heci_cl_get_drvdata(const struct heci_cl_device *device)
{
	return dev_get_drvdata(&device->dev);
}
EXPORT_SYMBOL_GPL(heci_cl_get_drvdata);

void heci_cl_set_drvdata(struct heci_cl_device *device, void *data)
{
	dev_set_drvdata(&device->dev, data);
}
EXPORT_SYMBOL_GPL(heci_cl_set_drvdata);

int heci_cl_enable_device(struct heci_cl_device *device)
{
	int err;
	struct heci_device *dev;
	struct heci_cl *cl = device->cl;

	if (cl == NULL)
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);

	cl->state = HECI_FILE_CONNECTING;

	err = heci_cl_connect(cl, NULL);
	if (err < 0) {
		mutex_unlock(&dev->device_lock);
		dev_err(&dev->pdev->dev, "Could not connect to the ME client");

		return err;
	}

	mutex_unlock(&dev->device_lock);

	if (device->event_cb && !cl->read_cb) {
		mutex_lock(&dev->device_lock);
		heci_cl_read_start(device->cl, 0);
		mutex_unlock(&dev->device_lock);
	}

	if (!device->ops || !device->ops->enable)
		return 0;

	return device->ops->enable(device);
}
EXPORT_SYMBOL_GPL(heci_cl_enable_device);

int heci_cl_disable_device(struct heci_cl_device *device)
{
	int err;
	struct heci_device *dev;
	struct heci_cl *cl = device->cl;

	if (cl == NULL)
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);

	if (cl->state != HECI_FILE_CONNECTED) {
		mutex_unlock(&dev->device_lock);
		dev_err(&dev->pdev->dev, "Already disconnected");

		return 0;
	}

	cl->state = HECI_FILE_DISCONNECTING;

	err = heci_cl_disconnect(cl);
	if (err < 0) {
		mutex_unlock(&dev->device_lock);
		dev_err(&dev->pdev->dev,
			"Could not disconnect from the ME client");

		return err;
	}

	/* Flush queues and remove any pending read */
	heci_cl_flush_queues(cl);

	if (cl->read_cb) {
		struct heci_cl_cb *cb = NULL;

		cb = heci_cl_find_read_cb(cl);
		/* Remove entry from read list */
		if (cb)
			list_del(&cb->list);

		cb = cl->read_cb;
		cl->read_cb = NULL;

		if (cb) {
			heci_io_cb_free(cb);
			cb = NULL;
		}
	}

	mutex_unlock(&dev->device_lock);

	if (!device->ops || !device->ops->disable)
		return 0;

	return device->ops->disable(device);
}
EXPORT_SYMBOL_GPL(heci_cl_disable_device);

void heci_cl_bus_rx_event(struct heci_cl *cl)
{
	struct heci_cl_device *device = cl->device;

	if (!device || !device->event_cb)
		return;

	set_bit(HECI_CL_EVENT_RX, &device->events);

	schedule_work(&device->event_work);
}

int __init heci_cl_bus_init(void)
{
	int	rv;
	int	order;
	unsigned	temp;
	rv = bus_register(&heci_cl_bus_type);

	/* Try to allocate 256 contiguous pages (1 M) for DMA and enabled host DMA */
	for (order = 0, temp = host_dma_buf_size / PAGE_SIZE + 1; temp; temp >>= 1)
		++order;
	host_dma_buf = __get_free_pages(GFP_KERNEL, order);
	if (host_dma_buf) {
		host_dma_buf_phys = __pa(host_dma_buf);
		host_dma_enabled = 1;
	}

	return	rv;
}

void __exit heci_cl_bus_exit(void)
{
	bus_unregister(&heci_cl_bus_type);
}

/* Read and write specific to ISH.
 * Meanwhile here, we'll later decide where to move this code */
int ish_heci_cl_recv(struct heci_cl *cl, u8 *buf, size_t length)
{
	return	__heci_cl_recv(cl, buf, length);
}
EXPORT_SYMBOL(ish_heci_cl_recv);

int ish_heci_cl_send(struct heci_cl *cl, u8 *buf, size_t length,
			bool blocking)
{
	return	___heci_cl_send(cl, buf, length, blocking);
}
EXPORT_SYMBOL(ish_heci_cl_send);


/*
 * Enum-completion callback for HECI bus - heci_device has reported its clients
 * NOTE: this is called in context of threaded IRQ handler (BH).
 * Consider consequences (in particular, don't use kmalloc(..., GFP_KERNEL))
 */
int heci_bus_new_client(struct heci_device *dev)
{
	int i;
	struct heci_cl	*cl;
	char *dev_name;
	struct heci_cl_device *cl_device;
	int rv;

	/*
	 * For all reported clients, create an unconnected client and add its device to HECI bus.
	 * If appropriate driver has loaded, this will trigger its probe().
	 * Otherwise, probe() will be called when driver is loaded
	 */
	i = dev->me_client_presentation_num - 1;
	cl = heci_cl_allocate(dev);
	if (!cl)
		return	-ENOMEM;

	rv = heci_cl_link(cl, HECI_HOST_CLIENT_ID_ANY);
	if (rv)
		return	-ENOMEM;

	cl->me_client_id = dev->me_clients[i].client_id;
	cl->device_uuid = dev->me_clients[i].props.protocol_name;
	dev_name = kasprintf(GFP_ATOMIC, "{%02X%02X%02X%02X-%02X%02X-%02X%02X-%02X%02X-%02X%02X%02X%02X%02X%02X}",
		cl->device_uuid.b[3], cl->device_uuid.b[2],
		cl->device_uuid.b[1], cl->device_uuid.b[0],
		cl->device_uuid.b[5], cl->device_uuid.b[4],
		cl->device_uuid.b[7], cl->device_uuid.b[6],
		cl->device_uuid.b[8], cl->device_uuid.b[9],
		cl->device_uuid.b[10], cl->device_uuid.b[11],
		cl->device_uuid.b[12], cl->device_uuid.b[13],
		cl->device_uuid.b[14], cl->device_uuid.b[15]);
	if (!dev_name) {
		kfree(cl);
		return	-ENOMEM;
	}

	list_add_tail(&cl->device_link, &dev->device_list);
	cl_device = heci_cl_add_device(dev, cl->device_uuid, dev_name, NULL);
	if (!cl_device) {
		kfree(dev_name);
		kfree(cl);
		return	-ENOENT;
	}

	return	0;
}


static int does_driver_bind_uuid(struct device *dev, void *id)
{
	uuid_le	*uuid = id;
	struct heci_cl_device *device;
	struct heci_cl	*cl;

	if (!dev->driver)
		return	0;

	device = to_heci_cl_device(dev);
	cl = device->cl;
	if (!cl)
		return	0;

	if (!uuid_le_cmp(cl->device_uuid, *uuid))
		return	1;

	return	0;
}


int heci_can_client_connect(struct heci_device *heci_dev, uuid_le *uuid)
{
	int rv;

	rv = bus_for_each_dev(&heci_cl_bus_type, NULL, uuid,
				does_driver_bind_uuid);
	return !rv;
}
