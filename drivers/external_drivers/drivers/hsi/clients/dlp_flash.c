/*
 * dlp_flash.c
 *
 * Intel Mobile Communication protocol driver for modem boot/flashing:
 *
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
 *
 * Contact: Faouaz Tenoutit <faouazx.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/hsi/hsi.h>
#include <linux/uaccess.h>

#include "dlp_main.h"


#define FLASHING_DEVNAME	"tty"CONFIG_HSI_FLASHING_DEV_NAME

/* Number of RX msg to push
 * if the controller FIFO
 */
#define DLP_FLASH_NB_RX_MSG 10


/*
 * struct flashing_driver - HSI Modem flashing driver protocol
 *
 * @lock: spinlock to serialise access to the driver information
 * @major: char device major number
 * @tdev: char device type dev
 * @dev: char device
 * @cdev: char device
 * @class: char device class
 * @opened: This flasg is used to allow only ONE instance of this driver
 * @read_wq: Read/Poll/Select wait event
 * @rx_msgs: RX messages queue
 * @ch_ctx: Flashing Channel context
*/
struct dlp_flash_ctx {
	/* Char device registration */
	int major;
	dev_t tdev;
	struct device *dev;
	struct cdev cdev;
	struct class *class;

	/* Used to prevent multiple access to device */
	unsigned int opened;

	/* A waitqueue for poll/read operations */
	wait_queue_head_t read_wq;

	/* RX msg queue */
	struct list_head rx_msgs;

	struct dlp_channel *ch_ctx;
};


/*
 *
 */
static void dlp_flash_complete_rx(struct hsi_msg *msg);


/*
* @brief
*
* @param flash_ctx
* @param value
*/
static inline void dlp_flash_set_opened(struct dlp_channel *ch_ctx,
		int value)
{
	unsigned long flags;
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;

	spin_lock_irqsave(&ch_ctx->lock, flags);

	/* Set the open flag */
	flash_ctx->opened = value;

	spin_unlock_irqrestore(&ch_ctx->lock, flags);
}

/*
* @brief
*
* @param flash_ctx
* @param value
*/
static inline int dlp_flash_get_opened(struct dlp_channel *ch_ctx)
{
	int opened;
	unsigned long flags;
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;

	/* Set the open flag */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	opened = flash_ctx->opened;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	return opened;
}

/*
* @brief Called to destruct a fixed msg size (DLP_FLASH_PDU_SIZE)
*
* @param msg
*/
static inline void dlp_flash_msg_destruct_fix(struct hsi_msg *msg)
{
	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);
}

/*
* @brief Called to destruct a variable msg size (_write function)
*
* @param msg
*/
static inline void dlp_flash_msg_destruct_var(struct hsi_msg *msg)
{
	/* Delete the received msg
	 * (size is variable, so dont consider the default PDU size) */
	dlp_pdu_free(msg, -1);
}

/*
 * Push RX pdu on channel 0
 *
 */
static int dlp_flash_push_rx_pdu(struct dlp_channel *ch_ctx)
{
	int ret;
	struct hsi_msg *rx_msg;

	/* Allocate a new RX msg */
	rx_msg = dlp_pdu_alloc(ch_ctx->hsi_channel,
				HSI_MSG_READ,
				DLP_FLASH_RX_PDU_SIZE,
				1,
				ch_ctx,
				dlp_flash_complete_rx,
				dlp_flash_msg_destruct_fix);

	if (!rx_msg) {
		pr_err(DRVNAME ": dlp_pdu_alloc(RX) failed\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Send the RX HSI msg */
	ret = hsi_async(rx_msg->cl, rx_msg);
	if (ret) {
		pr_err(DRVNAME ": hsi_async() failed, ret:%d\n", ret);
		ret = -EIO;
		goto free_msg;
	}

	return 0;

free_msg:
	/* Free the msg */
	dlp_pdu_free(rx_msg, rx_msg->channel);

out:
	return ret;
}

/*
* @brief Insert the provided msg at the begining (head) of the list
*
* @param ch_ctx
* @param msg
*/
static void dlp_boot_rx_queue_head(struct dlp_channel *ch_ctx,
					struct hsi_msg *msg)
{
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;
	unsigned long flags;

	/* Add at the begining of the list */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	list_add(&msg->link, &flash_ctx->rx_msgs);
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
}

/*
* @brief Insert the provided msg at the end (tail) of the list
*
* @param ch_ctx
* @param msg
*/
static void dlp_boot_rx_queue_tail(struct dlp_channel *ch_ctx,
					struct hsi_msg *msg)
{
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;
	unsigned long flags;

	/* Add at the end of the list */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	list_add_tail(&msg->link, &flash_ctx->rx_msgs);
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
}

/*
* @brief Remove & return the first list item (return NULL if empty)
*
* @param ch_ctx
* @param msg
*/
static struct hsi_msg *dlp_boot_rx_dequeue(struct dlp_channel *ch_ctx)
{
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;
	struct hsi_msg *msg = NULL;
	unsigned long flags;

	spin_lock_irqsave(&ch_ctx->lock, flags);

	if (!list_empty(&flash_ctx->rx_msgs)) {
		/* Get the fist list item (head) */
		msg = list_entry(flash_ctx->rx_msgs.next, struct hsi_msg, link);

		/* Remove the item from the list */
		list_del_init(&msg->link);
	}

	spin_unlock_irqrestore(&ch_ctx->lock, flags);
	return msg;
}


/*
* @brief
*
* @param msg
*/
static void dlp_flash_complete_tx(struct hsi_msg *msg)
{
	/* Delete the received msg */
	dlp_pdu_free(msg, -1);
}

/*
* @brief
*
* @param msg
*/
static void dlp_flash_complete_rx(struct hsi_msg *msg)
{
	struct dlp_channel *ch_ctx = msg->context;
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;
	int ret;

	if (msg->status != HSI_STATUS_COMPLETED) {
		pr_err(DRVNAME ": Invalid msg status: %d (ignored)\n",
				msg->status);

		/* Push again the RX msg */
		ret = hsi_async(msg->cl, msg);
		if (ret) {
			pr_err(DRVNAME ": hsi_async failed (%d) => FIFO will be empty\n",
					ret);

			/* Delete the received msg */
			dlp_pdu_free(msg, msg->channel);
		}
	} else {
		/* Add the received msg to the RX queue */
		dlp_boot_rx_queue_tail(ch_ctx, msg);

		/* Wakeup any waiting clients for read/poll */
		wake_up_interruptible(&flash_ctx->read_wq);
	}
}


/*
 * Called when a process tries to open the device file
 */
static int dlp_flash_dev_open(struct inode *inode, struct file *filp)
{
	int ret = 0;
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_FLASH);

	/* Only ONE instance of this device can be opened */
	if (dlp_flash_get_opened(ch_ctx)) {
		ret = -EBUSY;
		goto out;
	}

	/* Update/Set the eDLP channel id */
	dlp_drv.channels_hsi[ch_ctx->hsi_channel].edlp_channel = ch_ctx->ch_id;

	/* Save private data for futur use */
	filp->private_data = ch_ctx;

	/* Set the open flag */
	dlp_flash_set_opened(ch_ctx, 1);

	/* device opened => Set the channel state flag */
	dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_OPENED);

	/* claim & setup HSI port */
	dlp_hsi_port_claim();

	/* Push RX PDUs */
	for (ret = DLP_FLASH_NB_RX_MSG; ret; ret--)
		dlp_flash_push_rx_pdu(ch_ctx);

out:
	return ret;
}

/*
 * Called when a process closes the device file.
 */
static int dlp_flash_dev_close(struct inode *inode, struct file *filp)
{
	struct dlp_channel *ch_ctx = filp->private_data;

	/* flush everything */
	hsi_flush(dlp_drv.client);

	/* Release the hsi controller */
	dlp_hsi_port_unclaim();

	/* Set the open flag */
	dlp_flash_set_opened(ch_ctx, 0);

	/* device closed => Set the channel state flag */
	dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_CLOSED);

	return 0;
}

/*
 * Called when a process, which already opened the dev file, attempts to
 * read from it.
 */
static ssize_t dlp_flash_dev_read(struct file *filp,
			   char __user *data,
			   size_t count,
			   loff_t *ppos)
{
	struct dlp_channel *ch_ctx = filp->private_data;
	struct dlp_flash_ctx *flash_ctx;
	struct hsi_msg *msg;
	int ret, to_copy, copied, available;
	unsigned long flags;

	/* Have some data to read ? */
	if (!ch_ctx || !ch_ctx->ch_data)
		return -ENOENT;
	else {
		flash_ctx = ch_ctx->ch_data;
		spin_lock_irqsave(&ch_ctx->lock, flags);
		ret = list_empty(&flash_ctx->rx_msgs);
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
	}

	/* List empty ? */
	if (ret) {
		/* Descriptor opened in Non-Blocking mode ? */
		if (filp->f_flags & O_NONBLOCK)
			return -EWOULDBLOCK;
		else {
			if (ch_ctx && flash_ctx)
				ret = wait_event_interruptible(flash_ctx->read_wq, !dlp_flash_get_opened(ch_ctx));
			else
				return -ENOENT;

			if (ret)
				return -EINTR;
			else
				/* driver closed !*/
				return -ENOENT;
		}
	}

	copied = 0;
	available = count;

	/* Parse RX msgs queue */
	while (ch_ctx && (msg = dlp_boot_rx_dequeue(ch_ctx))) {
		/* Check if we have enough of space in the user buffer */
		to_copy = MIN(msg->actual_len, available);
		/* In case no more buffer at user space for the read */
		if (ch_ctx && (to_copy == 0)) {
			/* Put the msg back in the RX queue */
			dlp_boot_rx_queue_head(ch_ctx, msg);

			/* Copy done (The user buffer is full) */
			break;
		}

		/* Copy data to the user buffer */
		ret = copy_to_user(data+copied, sg_virt(msg->sgt.sgl), to_copy);
		if (ret) {
			pr_err(DRVNAME ": Uanble to copy data to the user buffer\n");

			/* Put the msg back in the RX queue */
			if (ch_ctx)
				dlp_boot_rx_queue_head(ch_ctx, msg);

			/* Stop copying */
			break;
		}

		copied += to_copy;
		available -= to_copy;

		/* Data read => Queue the RX msg */
		/* Push again the RX msg to the controller */
		ret = hsi_async(msg->cl, msg);
		if (ret) {
			pr_err(DRVNAME ": hsi_async failed (%d) => FIFO will be empty\n",
					ret);

			/* Delete the received msg */
			dlp_pdu_free(msg, msg->channel);
		}
	}

	if (!ch_ctx)
		return -ENOENT;
	else {
		/* Update the position */
		(*ppos) += copied;

		return copied;
	}
}

/*
 * Called when a process writes to dev file
 */
static ssize_t dlp_flash_dev_write(struct file *filp,
		const char __user *data,
		size_t count,
		loff_t *ppos)
{
	int ret;
	struct dlp_channel *ch_ctx = filp->private_data;
	struct hsi_msg *tx_msg = NULL;

	/* Allocate a new TX msg */
	tx_msg = dlp_pdu_alloc(ch_ctx->hsi_channel,
				HSI_MSG_WRITE,
				count,
				1,
				ch_ctx,
				dlp_flash_complete_tx,
				dlp_flash_msg_destruct_var);

	if (!tx_msg) {
		pr_err(DRVNAME ": dlp_pdu_alloc(TX, len: %d) failed\n", count);
		ret = -ENOMEM;
		goto out;
	}

	/* Copy the user data */
	memcpy(sg_virt(tx_msg->sgt.sgl), data, count);

	/* Send the TX HSI msg */
	ret = hsi_async(tx_msg->cl, tx_msg);
	if (ret) {
		pr_err(DRVNAME ": hsi_async(TX) failed (ret:%d)\n", ret);
		if (ret != -EACCES)
			ret = -EIO;
		goto free_tx;
	}

	ret = count;
	return ret;

free_tx:
	/* Delete the received msg
	 * (size is variable, so dont consider the default PDU size) */
	dlp_pdu_free(tx_msg, -1);

out:
	return ret;
}


/*
* @brief
*
* @param filp
* @param wait
*
* @return
*/
static unsigned int dlp_flash_dev_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct dlp_channel *ch_ctx;
	struct dlp_flash_ctx *flash_ctx;
	unsigned long flags;
	unsigned int ret = 0;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
				__func__);
		ret = POLLHUP;
		return ret;
	}
	ch_ctx = filp->private_data;
	flash_ctx = ch_ctx->ch_data;

	poll_wait(filp, &flash_ctx->read_wq, pt);

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
				__func__);
		ret = POLLHUP;
		return ret;
	}
	/* Have some data to read ? */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (!list_empty(&flash_ctx->rx_msgs))
		ret = POLLIN | POLLRDNORM;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
	return ret;
}


/*
* Device driver file operations
*/
static const struct file_operations dlp_flash_ops = {
	.open	= dlp_flash_dev_open,
	.read	= dlp_flash_dev_read,
	.write	= dlp_flash_dev_write,
	.poll	= dlp_flash_dev_poll,
	.release = dlp_flash_dev_close
};

static int dlp_flash_ctx_cleanup(struct dlp_channel *ch_ctx);

/*
* @brief
*
* @param ch_id
* @param hsi_channel
* @param dev
*
* @return
*/
struct dlp_channel *dlp_flash_ctx_create(unsigned int ch_id,
		unsigned int hsi_channel,
		struct device *dev)
{
	int ret;
	struct hsi_client *client = to_hsi_client(dev);
	struct dlp_channel *ch_ctx;
	struct dlp_flash_ctx *flash_ctx;

	/* Allocate channel struct data */
	ch_ctx = kzalloc(sizeof(struct dlp_channel), GFP_KERNEL);
	if (!ch_ctx) {
		pr_err(DRVNAME ": Out of memory (flash_ch_ctx)\n");
		goto out;
	}

	/* Allocate the context private data */
	flash_ctx = kzalloc(sizeof(struct dlp_flash_ctx), GFP_KERNEL);
	if (!flash_ctx) {
		pr_err(DRVNAME ": Out of memory (flash_ctx)\n");
		goto free_ch;
	}

	/* Save params */
	ch_ctx->ch_data = flash_ctx;
	ch_ctx->ch_id = ch_id;
	ch_ctx->hsi_channel = hsi_channel;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_waitqueue_head(&flash_ctx->read_wq);
	flash_ctx->ch_ctx = ch_ctx;
	INIT_LIST_HEAD(&flash_ctx->rx_msgs);

	/* Init the RX/TX contexts */
	dlp_xfer_ctx_init(ch_ctx,
			  DLP_FLASH_TX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_FLASH_RX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_READ);

	/* Register the device */
	ret = alloc_chrdev_region(&flash_ctx->tdev, 0, 1, FLASHING_DEVNAME);
	if (ret) {
		pr_err(DRVNAME ": alloc_chrdev_region failed (err: %d)\n", ret);
		goto free_ctx;
	}

	flash_ctx->major = MAJOR(flash_ctx->tdev);
	cdev_init(&flash_ctx->cdev, &dlp_flash_ops);
	flash_ctx->cdev.owner = THIS_MODULE;

	ret = cdev_add(&flash_ctx->cdev, flash_ctx->tdev, 1);
	if (ret) {
		pr_err(DRVNAME ": cdev_add failed (err: %d)", ret);
		goto unreg_reg;
	}

	flash_ctx->class = class_create(THIS_MODULE, DRVNAME"-flash");
	if (IS_ERR(flash_ctx->class))
		goto del_cdev;

	flash_ctx->dev = device_create(flash_ctx->class,
			NULL,
			flash_ctx->tdev,
			NULL, FLASHING_DEVNAME"%d", DLP_TTY_DEV_NUM);
	if (IS_ERR(flash_ctx->dev)) {
		pr_err(DRVNAME ": device_create failed (err: %ld)",
				PTR_ERR(flash_ctx->dev));
		goto del_class;
	}

	/* Register cleanup CB */
	ch_ctx->cleanup = dlp_flash_ctx_cleanup;

	return ch_ctx;

del_class:
	class_destroy(flash_ctx->class);

del_cdev:
	cdev_del(&flash_ctx->cdev);

unreg_reg:
	unregister_chrdev_region(flash_ctx->tdev, 1);

free_ctx:
	kfree(flash_ctx);

free_ch:
	kfree(ch_ctx);

out:
	return NULL;
}

/*
* @brief This function will delete/unregister
*	the char device and class
*
* @param ch_ctx: Flash channel context
*
* @return 0 when sucess, error code otherwise
*/
static int dlp_flash_ctx_cleanup(struct dlp_channel *ch_ctx)
{
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;
	struct hsi_msg *msg;
	int ret = 0;

	/* stop Reading thread */
	dlp_flash_set_opened(ch_ctx, 0);
	/* empty rx list*/
	while (!list_empty(&flash_ctx->rx_msgs)) {
		msg = dlp_boot_rx_dequeue(ch_ctx);
		if (msg != NULL) {
			dlp_pdu_free(msg, msg->channel);
		}
	}

	/* Unregister/Delete char device & class */
	device_destroy(flash_ctx->class, flash_ctx->tdev);
	cdev_del(&flash_ctx->cdev);
	unregister_chrdev_region(flash_ctx->tdev, 1);
	class_destroy(flash_ctx->class);

	return ret;
}

/*
 * This function will release the allocated memory
 * done in the _ctx_create function
 */
int dlp_flash_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_flash_ctx *flash_ctx = ch_ctx->ch_data;
	/* Free the BOOT/FLASHING context */
	kfree(flash_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);
	return 0;
}

static int dlp_flash_set_flashing_mode(const char *val, struct kernel_param *kp)
{
	long flashing;

	if (kstrtol(val, 16, &flashing) < 0)
		return -EINVAL;

	dlp_set_flashing_mode(flashing);
	return 0;
}

module_param_call(set_flashing_mode, dlp_flash_set_flashing_mode,
		NULL, NULL, 0644);

