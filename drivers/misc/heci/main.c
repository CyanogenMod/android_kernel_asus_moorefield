/*
 * User-mode HECI API
 *
 * Copyright (c) 2014, Intel Corporation.
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

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/kernel.h>
#include <linux/device.h>
#include <linux/fs.h>
#include <linux/errno.h>
#include <linux/types.h>
#include <linux/fcntl.h>
#include <linux/aio.h>
#include <linux/pci.h>
#include <linux/poll.h>
#include <linux/init.h>
#include <linux/ioctl.h>
#include <linux/cdev.h>
#include <linux/sched.h>
#include <linux/uuid.h>
#include <linux/compat.h>
#include <linux/jiffies.h>
#include <linux/interrupt.h>
#include <linux/miscdevice.h>
#include "heci.h"
#include "heci_dev.h"
#include "client.h"

#ifdef dev_dbg
#undef dev_dbg
#endif
static  void no_dev_dbg(void *v, char *s, ...)
{
}
#define dev_dbg no_dev_dbg
/*#define dev_dbg dev_err*/

extern int host_dma_enabled;

/**
 * heci_open - the open function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 e
 * returns 0 on success, <0 on error
 */
static int heci_open(struct inode *inode, struct file *file)
{
	struct miscdevice *misc = file->private_data;
	struct pci_dev *pdev;
	struct heci_cl *cl;
	struct heci_device *dev;

	int err;

	err = -ENODEV;
	if (!misc->parent)
		goto out;

	pdev = container_of(misc->parent, struct pci_dev, dev);

	dev = pci_get_drvdata(pdev);
	if (!dev)
		goto out;

	mutex_lock(&dev->device_lock);
	err = -ENOMEM;
	cl = heci_cl_allocate(dev);
	if (!cl)
		goto out_unlock;

	/* We may have a case of issued open()
	 * with dev->dev_state == HECI_DEV_DISABLED,
	 * as part of re-enabling path */
#if 0
	err = -ENODEV;
	if (dev->dev_state != HECI_DEV_ENABLED) {
		dev_dbg(&dev->pdev->dev, "dev_state != HECI_ENABLED  dev_state = %s\n",
		    heci_dev_state_str(dev->dev_state));
		goto out_unlock;
	}
#endif
	err = -EMFILE;
	if (dev->open_handle_count >= HECI_MAX_OPEN_HANDLE_COUNT) {
		dev_err(&dev->pdev->dev, "open_handle_count exceded %d",
			HECI_MAX_OPEN_HANDLE_COUNT);
		goto out_unlock;
	}

	err = heci_cl_link(cl, HECI_HOST_CLIENT_ID_ANY);
	if (err)
		goto out_unlock;

	file->private_data = cl;
	mutex_unlock(&dev->device_lock);

	return nonseekable_open(inode, file);

out_unlock:
	mutex_unlock(&dev->device_lock);
	kfree(cl);
out:
	return err;
}

/**
 * heci_release - the release function
 *
 * @inode: pointer to inode structure
 * @file: pointer to file structure
 *
 * returns 0 on success, <0 on error
 */
static int heci_release(struct inode *inode, struct file *file)
{
	struct heci_cl *cl = file->private_data;
	struct heci_cl_cb *cb;
	struct heci_device *dev;
	int rets = 0;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	/* May happen if device sent FW reset or was intentionally
	 * halted by host SW. The client is then invalid
	 */
	if (dev->dev_state != HECI_DEV_ENABLED)
		return	0;

	mutex_lock(&dev->device_lock);
	if (cl->state == HECI_FILE_CONNECTED) {
		cl->state = HECI_FILE_DISCONNECTING;
		dev_dbg(&dev->pdev->dev,
			"disconnecting client host client = %d, "
		    "ME client = %d\n",
		    cl->host_client_id,
		    cl->me_client_id);
		rets = heci_cl_disconnect(cl);
	}

	heci_cl_flush_queues(cl);
	dev_dbg(&dev->pdev->dev, "remove client host client = %d, ME client = %d\n",
				cl->host_client_id,
				cl->me_client_id);

	if (dev->open_handle_count > 0) {
		clear_bit(cl->host_client_id, dev->host_clients_map);
		dev->open_handle_count--;
	}
	heci_cl_unlink(cl);


	/* free read cb */
	cb = NULL;
	if (cl->read_cb) {
		cb = heci_cl_find_read_cb(cl);
		/* Remove entry from read list */
		if (cb)
			list_del(&cb->list);

		cb = cl->read_cb;
		cl->read_cb = NULL;
	}

	file->private_data = NULL;

	if (cb) {
		heci_io_cb_free(cb);
		cb = NULL;
	}

	kfree(cl);
out:
	mutex_unlock(&dev->device_lock);
	return rets;
}


/**
 * heci_read - the read function.
 *
 * @file: pointer to file structure
 * @ubuf: pointer to user buffer
 * @length: buffer length
 * @offset: data offset in buffer
 *
 * returns >=0 data length on success , <0 on error
 */
static ssize_t heci_read(struct file *file, char __user *ubuf,
			size_t length, loff_t *offset)
{
	struct heci_cl *cl = file->private_data;
	struct heci_cl_cb *cb_pos = NULL;
	struct heci_cl_cb *cb = NULL;
	struct heci_device *dev;
	int rets;
	int err;


	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);
	if (dev->dev_state != HECI_DEV_ENABLED) {
		rets = -ENODEV;
		goto out;
	}

	if (cl->read_cb && cl->read_cb->buf_idx > *offset) {
		cb = cl->read_cb;
		goto copy_buffer;
	} else if (cl->read_cb && cl->read_cb->buf_idx > 0 &&
		   cl->read_cb->buf_idx <= *offset) {
		cb = cl->read_cb;
		rets = 0;
		goto free;
	} else if ((!cl->read_cb || !cl->read_cb->buf_idx) && *offset > 0) {
		/*Offset needs to be cleaned for contiguous reads*/
		*offset = 0;
		rets = 0;
		goto out;
	}

	err = heci_cl_read_start(cl, length);
	if (err && err != -EBUSY) {
		dev_dbg(&dev->pdev->dev,
			"heci start read failure with status = %d\n", err);
		rets = err;
		goto out;
	}

	if (HECI_READ_COMPLETE != cl->reading_state &&
			!waitqueue_active(&cl->rx_wait)) {
		if (file->f_flags & O_NONBLOCK) {
			rets = -EAGAIN;
			goto out;
		}

		mutex_unlock(&dev->device_lock);

		if (wait_event_interruptible(cl->rx_wait,
			(dev->dev_state == HECI_DEV_ENABLED &&
			(HECI_READ_COMPLETE == cl->reading_state ||
			 HECI_FILE_INITIALIZING == cl->state ||
			 HECI_FILE_DISCONNECTED == cl->state ||
			 HECI_FILE_DISCONNECTING == cl->state)))) {
			if (signal_pending(current))
				return -EINTR;
			return -ERESTARTSYS;
		}

		mutex_lock(&dev->device_lock);

		/* If FW reset arrived, this will happen.
		 * Don't check cl->, as 'cl' may be freed already
		 */
		if (dev->dev_state != HECI_DEV_ENABLED) {
			rets = -ENODEV;
			goto	out;
		}

		if (HECI_FILE_INITIALIZING == cl->state ||
		    HECI_FILE_DISCONNECTED == cl->state ||
		    HECI_FILE_DISCONNECTING == cl->state) {
			rets = -EBUSY;
			goto out;
		}
	}

	cb = cl->read_cb;

	if (!cb) {
		rets = -ENODEV;
		goto out;
	}
	if (cl->reading_state != HECI_READ_COMPLETE) {
		rets = 0;
		goto out;
	}
	/* now copy the data to user space */
copy_buffer:
	dev_dbg(&dev->pdev->dev, "buf.size = %d buf.idx= %ld\n",
	    cb->response_buffer.size, cb->buf_idx);
	if (length == 0 || ubuf == NULL || *offset > cb->buf_idx) {
		rets = -EMSGSIZE;
		goto free;
	}

	/* length is being truncated to PAGE_SIZE,
	 * however buf_idx may point beyond that */
	length = min_t(size_t, length, cb->buf_idx - *offset);

	if (copy_to_user(ubuf, cb->response_buffer.data + *offset, length)) {
		rets = -EFAULT;
		goto free;
	}

	rets = length;
	*offset += length;
	if ((unsigned long)*offset < cb->buf_idx)
		goto out;

free:
	cb_pos = heci_cl_find_read_cb(cl);
	/* Remove entry from read list */
	if (cb_pos)
		list_del(&cb_pos->list);
	heci_io_cb_free(cb);
	cl->reading_state = HECI_IDLE;
	cl->read_cb = NULL;
out:
	dev_dbg(&dev->pdev->dev, "end heci read rets= %d\n", rets);
	mutex_unlock(&dev->device_lock);
	return rets;
}


/**
 * heci_write - the write function.
 *
 * @file: pointer to file structure
 * @ubuf: pointer to user buffer
 * @length: buffer length
 * @offset: data offset in buffer
 *
 * returns >=0 data length on success , <0 on error
 */
static ssize_t heci_write(struct file *file, const char __user *ubuf,
			 size_t length, loff_t *offset)
{
	struct heci_cl *cl = file->private_data;
	struct heci_cl_cb *write_cb = NULL;
	struct heci_device *dev;
	unsigned long timeout = 0;
	int rets;
	int id;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);

	if (dev->dev_state != HECI_DEV_ENABLED) {
		rets = -ENODEV;
		goto out;
	}

	id = heci_me_cl_by_id(dev, cl->me_client_id);
	if (id < 0) {
		rets = -ENODEV;
		goto out;
	}

	/* FIXME: check for DMA size for clients that accept DMA transfers */
	if (length > dev->me_clients[id].props.max_msg_length || length <= 0) {
		/* If the client supports DMA, try to use it */
		if (!(host_dma_enabled && dev->me_clients[id].props.dma_hdr_len & HECI_CLIENT_DMA_ENABLED)) {
			rets = -EMSGSIZE;
			goto out;
		}
	}

	if (cl->state != HECI_FILE_CONNECTED) {
		dev_err(&dev->pdev->dev, "host client = %d,  is not connected to ME client = %d",
			cl->host_client_id, cl->me_client_id);
		rets = -ENODEV;
		goto out;
	}

	/* free entry used in read */
	if (cl->reading_state == HECI_READ_COMPLETE) {
		*offset = 0;
		write_cb = heci_cl_find_read_cb(cl);
		if (write_cb) {
			list_del(&write_cb->list);
			heci_io_cb_free(write_cb);
			write_cb = NULL;
			cl->reading_state = HECI_IDLE;
			cl->read_cb = NULL;
		}
	} else if (cl->reading_state == HECI_IDLE)
		*offset = 0;

	write_cb = heci_io_cb_init(cl, file);
	if (!write_cb) {
		dev_err(&dev->pdev->dev, "write cb allocation failed\n");
		rets = -ENOMEM;
		goto out;
	}
	rets = heci_io_cb_alloc_req_buf(write_cb, length);
	if (rets)
		goto out;

	rets = copy_from_user(write_cb->request_buffer.data, ubuf, length);
	if (rets)
		goto out;

	rets = heci_cl_write(cl, write_cb, false);
out:
	mutex_unlock(&dev->device_lock);
	if (rets < 0)
		heci_io_cb_free(write_cb);
	return rets;
}


int     heci_can_client_connect(struct heci_device *heci_dev, uuid_le *uuid);


/**
 * heci_ioctl_connect_client - the connect to fw client IOCTL function
 *
 * @dev: the device structure
 * @data: IOCTL connect data, input and output parameters
 * @file: private data of the file object
 *
 * Locking: called under "dev->device_lock" lock
 *
 * returns 0 on success, <0 on failure.
 */
static int heci_ioctl_connect_client(struct file *file,
			struct heci_connect_client_data *data)
{
	struct heci_device *dev;
	struct heci_client *client;
	struct heci_cl *cl;
	int i;
	int rets;

	cl = file->private_data;
	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	dev = cl->dev;

	if (dev->dev_state != HECI_DEV_ENABLED) {
		rets = -ENODEV;
		goto end;
	}

	if (cl->state != HECI_FILE_INITIALIZING &&
	    cl->state != HECI_FILE_DISCONNECTED) {
		rets = -EBUSY;
		goto end;
	}

	/* find ME client we're trying to connect to */
	i = heci_me_cl_by_uuid(dev, &data->in_client_uuid);
	if (i < 0 || dev->me_clients[i].props.fixed_address) {
		dev_dbg(&dev->pdev->dev, "Cannot connect to FW Client UUID = %pUl\n",
				&data->in_client_uuid);
		rets = -ENODEV;
		goto end;
	}

	/* Check if there's driver attached to this UUID */
	if (!heci_can_client_connect(dev, &data->in_client_uuid))
		return	-EBUSY;

	cl->me_client_id = dev->me_clients[i].client_id;
	cl->state = HECI_FILE_CONNECTING;

	dev_dbg(&dev->pdev->dev, "Connect to FW Client ID = %d\n",
			cl->me_client_id);
	dev_dbg(&dev->pdev->dev, "FW Client - Protocol Version = %d\n",
			dev->me_clients[i].props.protocol_version);
	dev_dbg(&dev->pdev->dev, "FW Client - Max Msg Len = %d\n",
			dev->me_clients[i].props.max_msg_length);

	/* prepare the output buffer */
	client = &data->out_client_properties;
	client->max_msg_length = dev->me_clients[i].props.max_msg_length;
	client->protocol_version = dev->me_clients[i].props.protocol_version;
	dev_dbg(&dev->pdev->dev, "Can connect?\n");


	rets = heci_cl_connect(cl, file);

end:
	return rets;
}


/**
 * heci_ioctl - the IOCTL function
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to heci message structure
 *
 * returns 0 on success , <0 on error
 */
static long heci_ioctl(struct file *file, unsigned int cmd, unsigned long data)
{
	struct heci_device *dev;
	struct heci_cl *cl = file->private_data;
	struct heci_connect_client_data *connect_data = NULL;
	int rets;

	dev = cl->dev;
	dev_dbg(&dev->pdev->dev, "IOCTL cmd = 0x%x", cmd);

	/* Test API for triggering host-initiated IPC reset to ISH */
	if (cmd == 0x12345678) {
		/* Re-init */
		dev->dev_state = HECI_DEV_INITIALIZING;
		heci_reset(dev, 1);

		if (heci_hbm_start_wait(dev)) {
			dev_err(&dev->pdev->dev, "HBM haven't started");
			goto err;
		}

		if (!heci_host_is_ready(dev)) {
			dev_err(&dev->pdev->dev, "host is not ready.\n");
			goto err;
		}

		if (!heci_hw_is_ready(dev)) {
			dev_err(&dev->pdev->dev, "ME is not ready.\n");
			goto err;
		}

		return	0;
err:
		dev_err(&dev->pdev->dev, "link layer initialization failed.\n");
		dev->dev_state = HECI_DEV_DISABLED;
		return -ENODEV;
	}

	/* Test API for triggering host disabling */
	if (cmd == 0xAA55AA55) {
		/* Handle ISH reset against upper layers */
		/* Remove all client devices */
		heci_bus_remove_all_clients(dev);
		dev->dev_state = HECI_DEV_DISABLED;
		return	0;
	}

	if (cmd != IOCTL_HECI_CONNECT_CLIENT)
		return -EINVAL;

	if (WARN_ON(!cl || !cl->dev))
		return -ENODEV;

	mutex_lock(&dev->device_lock);
	if (dev->dev_state != HECI_DEV_ENABLED) {
		rets = -ENODEV;
		goto out;
	}

	dev_dbg(&dev->pdev->dev, ": IOCTL_HECI_CONNECT_CLIENT.\n");

	connect_data = kzalloc(sizeof(struct heci_connect_client_data),
							GFP_KERNEL);
	if (!connect_data) {
		rets = -ENOMEM;
		goto out;
	}
	dev_dbg(&dev->pdev->dev, "copy connect data from user\n");
	if (copy_from_user(connect_data, (char __user *)data,
				sizeof(struct heci_connect_client_data))) {
		dev_dbg(&dev->pdev->dev, "failed to copy data from userland\n");
		rets = -EFAULT;
		goto out;
	}

	rets = heci_ioctl_connect_client(file, connect_data);

	/* if all is ok, copying the data back to user. */
	if (rets)
		goto out;

	dev_dbg(&dev->pdev->dev, "copy connect data to user\n");
	if (copy_to_user((char __user *)data, connect_data,
				sizeof(struct heci_connect_client_data))) {
		dev_dbg(&dev->pdev->dev, "failed to copy data to userland\n");
		rets = -EFAULT;
		goto out;
	}

out:
	kfree(connect_data);
	mutex_unlock(&dev->device_lock);
	return rets;
}

/**
 * heci_compat_ioctl - the compat IOCTL function
 *
 * @file: pointer to file structure
 * @cmd: ioctl command
 * @data: pointer to heci message structure
 *
 * returns 0 on success , <0 on error
 */
#ifdef CONFIG_COMPAT
static long heci_compat_ioctl(struct file *file,
			unsigned int cmd, unsigned long data)
{
	return heci_ioctl(file, cmd, (unsigned long)compat_ptr(data));
}
#endif


/**
 * heci_poll - the poll function
 *
 * @file: pointer to file structure
 * @wait: pointer to poll_table structure
 *
 * returns poll mask
 */
static unsigned int heci_poll(struct file *file, poll_table *wait)
{
	struct heci_cl *cl = file->private_data;
	struct heci_device *dev;
	unsigned int mask = 0;

	if (WARN_ON(!cl || !cl->dev))
		return mask;

	dev = cl->dev;

	mutex_lock(&dev->device_lock);

	if (dev->dev_state != HECI_DEV_ENABLED)
		goto out;

	mutex_unlock(&dev->device_lock);
	poll_wait(file, &cl->tx_wait, wait);
	mutex_lock(&dev->device_lock);
	if (HECI_WRITE_COMPLETE == cl->writing_state)
		mask |= (POLLIN | POLLRDNORM);

out:
	mutex_unlock(&dev->device_lock);
	return mask;
}

/*
 * file operations structure will be used for heci char device.
 */
static const struct file_operations heci_fops = {
	.owner = THIS_MODULE,
	.read = heci_read,
	.unlocked_ioctl = heci_ioctl,
#ifdef CONFIG_COMPAT
	.compat_ioctl = heci_compat_ioctl,
#endif
	.open = heci_open,
	.release = heci_release,
	.write = heci_write,
	.poll = heci_poll,
	.llseek = no_llseek
};

/*
 * Misc Device Struct
 */
static struct miscdevice  heci_misc_device = {
		/* "heci" changed to "ish", stuff it #2 */
		.name = "ish",
		.fops = &heci_fops,
		.minor = MISC_DYNAMIC_MINOR,
};


int heci_register(struct heci_device *dev)
{
	int ret;
	heci_misc_device.parent = &dev->pdev->dev;
	ret = misc_register(&heci_misc_device);
	if (ret)
		return ret;

	if (heci_dbgfs_register(dev, heci_misc_device.name))
		dev_err(&dev->pdev->dev, "cannot register debugfs\n");

	return 0;
}
EXPORT_SYMBOL_GPL(heci_register);

void heci_deregister(struct heci_device *dev)
{
	heci_dbgfs_deregister(dev);
	misc_deregister(&heci_misc_device);
	heci_misc_device.parent = NULL;
}
EXPORT_SYMBOL_GPL(heci_deregister);

static int __init heci_init(void)
{
	return heci_cl_bus_init();
}

static void __exit heci_exit(void)
{
	heci_cl_bus_exit();
}

module_init(heci_init);
module_exit(heci_exit);

MODULE_AUTHOR("Intel Corporation");
MODULE_DESCRIPTION("Intel(R) Management Engine Interface");
MODULE_LICENSE("GPL v2");
