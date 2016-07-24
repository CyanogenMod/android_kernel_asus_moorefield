/*
 * hsi-flash.c
 *
 * HSI flash device driver, implements the character device
 * interface and it is used for modem FW flashing over HSI link
 *
 * Copyright (C) 2010 Nokia Corporation. All rights reserved.
 * Copyright (C) RMC. All rights reserved.
 *
 *
 * Contact: Andras Domokos <andras.domokos@nokia.com>
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

#include <linux/errno.h>
#include <linux/types.h>
#include <linux/atomic.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/poll.h>
#include <linux/ioctl.h>
#include <linux/wait.h>
#include <linux/fs.h>
#include <linux/sched.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/scatterlist.h>
#include <linux/hsi/hsi.h>
#include <linux/hsi/hsi_flash.h>
#include <linux/delay.h>

#define NEW_HSI_CORE_IF
#define HSI_FLASH_USE_DEBUG

#ifdef HSI_FLASH_USE_DEBUG
# define DPRINTK(...)    printk(KERN_DEBUG __VA_ARGS__)
#else
# define DPRINTK(...)
#endif


#define HSI_FLASH_CHANNELS	8
#define HSI_FLASH_DEVS		8
#define HSI_FLASH_MSGS		4

#define HSI_CHST_UNAVAIL	0 /* SBZ! */
#define HSI_CHST_AVAIL		1
#define HSI_CHST_CLOSED		(0 << 4)
#define HSI_CHST_CLOSING	(1 << 4)
#define HSI_CHST_OPENING	(2 << 4)
#define HSI_CHST_OPENED		(3 << 4)

#define HSI_CHST_READOFF	(0 << 8)
#define HSI_CHST_READON		(1 << 8)
#define HSI_CHST_READING	(2 << 8)

#define HSI_CHST_WRITEOFF	(0 << 12)
#define HSI_CHST_WRITEON	(1 << 12)
#define HSI_CHST_WRITING	(2 << 12)

#define HSI_CHST_OC_MASK	0xf0
#define HSI_CHST_RD_MASK	0xf00
#define HSI_CHST_WR_MASK	0xf000

#define HSI_CHST_OC(c)		((c)->state & HSI_CHST_OC_MASK)
#define HSI_CHST_RD(c)		((c)->state & HSI_CHST_RD_MASK)
#define HSI_CHST_WR(c)		((c)->state & HSI_CHST_WR_MASK)

#define HSI_CHST_OC_SET(c, v) \
	do { \
		(c)->state &= ~HSI_CHST_OC_MASK; \
		(c)->state |= v; \
	} while (0);

#define HSI_CHST_RD_SET(c, v) \
	do { \
		(c)->state &= ~HSI_CHST_RD_MASK; \
		(c)->state |= v; \
	} while (0);

#define HSI_CHST_WR_SET(c, v) \
	do { \
		(c)->state &= ~HSI_CHST_WR_MASK; \
		(c)->state |= v; \
	} while (0);

#define HSI_FLASH_POLL_RST	(-1)
#define HSI_FLASH_POLL_OFF	0
#define HSI_FLASH_POLL_ON	1

#define HSI_FLASH_RX		0
#define HSI_FLASH_TX		1

/* LCH START */
#define POLLIN_MESSAGE_LENGTH 4
/* LCH STOP */

struct hiocgwake {
	unsigned wake_state;
	unsigned wake_edges;
};

struct hsi_flash_channel {
	unsigned int		ch;
	unsigned int		state;
	int			wlrefcnt;
	int			rxpoll;
	struct hsi_client	*cl;
	struct list_head	free_msgs_list;
	struct list_head	rx_msgs_queue;
	struct list_head	tx_msgs_queue;
	struct list_head	pollin_msgs_queue;
	int			poll_event;
	struct hiocgwake        wake_event;
	unsigned int            break_event;
	spinlock_t		lock;
	struct fasync_struct	*async_queue;
	wait_queue_head_t	rx_wait;
	wait_queue_head_t	tx_wait;
};

struct hsi_flash_client_data {
	atomic_t		refcnt;
	int			attached;
	struct hsi_msg *break_msg_receive_before_setup;
	struct hsi_flash_channel	channels[HSI_FLASH_DEVS];
};

static unsigned int max_data_size =  65536;
module_param(max_data_size, uint, 1);
MODULE_PARM_DESC(max_data_size, "max read/write data size [4,8..65536] (^2)");

static int channels_map[HSI_FLASH_DEVS] = {0, -1, -1 , -1, -1, -1, -1, -1};
module_param_array(channels_map, int, NULL, 0);
MODULE_PARM_DESC(channels_map, "Array of HSI channels ([0...7]) to be probed");

static dev_t hsi_flash_dev;
static struct hsi_flash_client_data hsi_flash_cl_data;

static int hsi_flash_rx_poll(struct hsi_flash_channel *channel);
static int hsi_flash_break_request(struct hsi_client *cl);

static inline void hsi_flash_msg_free(struct hsi_msg *msg)
{
	msg->complete = NULL;
	msg->destructor = NULL;
	kfree(sg_virt(msg->sgt.sgl));
	hsi_free_msg(msg);
}

static inline void hsi_flash_msgs_free(struct hsi_flash_channel *channel)
{
	struct hsi_msg *msg, *tmp;

	list_for_each_entry_safe(msg, tmp, &channel->free_msgs_list, link) {
		list_del(&msg->link);
		hsi_flash_msg_free(msg);
	}
	list_for_each_entry_safe(msg, tmp, &channel->rx_msgs_queue, link) {
		list_del(&msg->link);
		hsi_flash_msg_free(msg);
	}
	list_for_each_entry_safe(msg, tmp, &channel->tx_msgs_queue, link) {
		list_del(&msg->link);
		hsi_flash_msg_free(msg);
	}
	list_for_each_entry_safe(msg, tmp, &channel->pollin_msgs_queue, link) {
		list_del(&msg->link);
		hsi_flash_msg_free(msg);
	}



}

static inline struct hsi_msg *hsi_flash_msg_alloc(unsigned int alloc_size)
{
	struct hsi_msg *msg;
	void *buf;

	msg = hsi_alloc_msg(1, GFP_KERNEL);
	if (!msg)
		goto out;

	buf = kmalloc(alloc_size, GFP_KERNEL);

	if (!buf) {
		hsi_free_msg(msg);
		goto out;
	}
	sg_init_one(msg->sgt.sgl, buf, alloc_size);
	msg->context = buf;
	return msg;
out:
	return NULL;
}

static inline int hsi_flash_msgs_alloc(struct hsi_flash_channel *channel)
{
	struct hsi_msg *msg;
	int i;

	for (i = 0; i < HSI_FLASH_MSGS; i++) {
		msg = hsi_flash_msg_alloc(max_data_size);
		if (!msg)
			goto out;
		msg->channel = channel->ch;
		list_add_tail(&msg->link, &channel->free_msgs_list);
	}
	return 0;
out:
	hsi_flash_msgs_free(channel);

	return -ENOMEM;
}

static int _hsi_flash_release(struct hsi_flash_channel *channel, int remove)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(channel->cl);
	int ret = 0, refcnt;

	DPRINTK("_hsi_flash_release\n");
	spin_lock_bh(&channel->lock);
	if (HSI_CHST_OC(channel) != HSI_CHST_OPENED)
		goto out;
	HSI_CHST_OC_SET(channel, HSI_CHST_CLOSING);
	spin_unlock_bh(&channel->lock);

	while (channel->wlrefcnt > 0) {
		hsi_stop_tx(channel->cl);
		channel->wlrefcnt--;
	}

	if (channel->rxpoll == HSI_FLASH_POLL_ON)
		channel->poll_event |= POLLERR;

	wake_up_interruptible(&channel->rx_wait);
	wake_up_interruptible(&channel->tx_wait);

	refcnt = atomic_dec_return(&cl_data->refcnt);
	if (!refcnt) {
		DPRINTK("hsi_flush 1\n");
		hsi_flush(channel->cl);
		hsi_release_port(channel->cl);
		cl_data->attached = 0;
	}
	hsi_flash_msgs_free(channel);

	spin_lock_bh(&channel->lock);
	HSI_CHST_OC_SET(channel, HSI_CHST_CLOSED);
	HSI_CHST_RD_SET(channel, HSI_CHST_READOFF);
	HSI_CHST_WR_SET(channel, HSI_CHST_WRITEOFF);
out:
	if (remove)
		channel->cl = NULL;
	spin_unlock_bh(&channel->lock);

	return ret;
}

static void hsi_flash_start_rx(struct hsi_client *cl)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(cl);
	struct hsi_flash_channel *channel = cl_data->channels;
	int i;

	DPRINTK("hsi_flash_start_rx\n");

	for (i = 0; i < HSI_FLASH_DEVS; i++, channel++) {
		if (HSI_CHST_OC(channel) != HSI_CHST_OPENED)
			continue;

		channel->wake_event.wake_state = 1;
		channel->wake_event.wake_edges++;
		channel->poll_event |= POLLHUP;
		wake_up_interruptible(&channel->rx_wait);
	}
}

static void hsi_flash_stop_rx(struct hsi_client *cl)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(cl);
	struct hsi_flash_channel *channel = cl_data->channels;
	int i;

	DPRINTK("hsi_flash_stop_rx\n");

	for (i = 0; i < HSI_FLASH_DEVS; i++, channel++) {
		if (HSI_CHST_OC(channel) != HSI_CHST_OPENED)
			continue;

		channel->wake_event.wake_state = 0;
		channel->wake_event.wake_edges++;

		channel->poll_event |= POLLHUP;
		wake_up_interruptible(&channel->rx_wait);
	}
}

#ifdef NEW_HSI_CORE_IF

static void hsi_flash_start_stop_rx(struct hsi_client *cl, unsigned long state)
{
	if (state == HSI_EVENT_START_RX)
		hsi_flash_start_rx(cl);
	else
		hsi_flash_stop_rx(cl);
}
#endif /* NEW_HSI_CORE_IF */

static int hsi_flash_probe(struct device *dev)
{
	struct hsi_flash_client_data *cl_data = &hsi_flash_cl_data;
	struct hsi_flash_channel *channel = cl_data->channels;
	struct hsi_client *cl = to_hsi_client(dev);
	int i;

	DPRINTK("hsi_flash_probe\n");

	for (i = 0; i < HSI_FLASH_DEVS; i++, channel++) {
		if (channel->state == HSI_CHST_AVAIL)
			channel->cl = cl;
	}

	atomic_set(&cl_data->refcnt, 0);
	cl_data->attached = 0;
	cl_data->break_msg_receive_before_setup = NULL;
	hsi_client_set_drvdata(cl, cl_data);

	return 0;
}

static int hsi_flash_remove(struct device *dev)
{
	struct hsi_client *cl = to_hsi_client(dev);
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(cl);
	struct hsi_flash_channel *channel = cl_data->channels;
	int i;

#ifdef NEW_HSI_CORE_IF
	hsi_unregister_port_event(channel->cl);
#else
	cl->hsi_start_rx = NULL;
	cl->hsi_stop_rx = NULL;
#endif /* NEW_HSI_CORE_IF */

	for (i = 0; i < HSI_FLASH_DEVS; i++, channel++) {
		if (!(channel->state & HSI_CHST_AVAIL))
			continue;
		_hsi_flash_release(channel, 1);
	}

	return 0;
}

static inline unsigned int hsi_flash_msg_len_get(struct hsi_msg *msg)
{
	return msg->sgt.sgl->length;
}

static inline void hsi_flash_msg_len_set(struct hsi_msg *msg, unsigned int len)
{
	msg->sgt.sgl->length = len;
}

static void hsi_flash_data_available(struct hsi_msg *msg)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(msg->cl);
	struct hsi_flash_channel *channel = cl_data->channels + msg->channel;
	int ret;

	DPRINTK("hsi_flash_data_available\n");

	if (msg->status == HSI_STATUS_ERROR) {
		ret = hsi_async_read(channel->cl, msg);
		if (ret < 0) {
			spin_lock_bh(&channel->lock);
			list_add_tail(&msg->link, &channel->free_msgs_list);
			channel->rxpoll = HSI_FLASH_POLL_OFF;
			spin_unlock_bh(&channel->lock);
		}
	} else {
		spin_lock_bh(&channel->lock);
		channel->poll_event |= (POLLIN | POLLRDNORM);
/* LCH START */
		list_add_tail(&msg->link, &channel->pollin_msgs_queue);
		/*list_add_tail(&msg->link, &channel->free_msgs_list);*/
/* LCH STOP */
		spin_unlock_bh(&channel->lock);

		wake_up_interruptible(&channel->rx_wait);
	}
}

static void hsi_flash_rx_completed(struct hsi_msg *msg)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(msg->cl);
	struct hsi_flash_channel *channel = cl_data->channels + msg->channel;

	DPRINTK("hsi_flash_rx_completed\n");

	spin_lock_bh(&channel->lock);
	list_add_tail(&msg->link, &channel->rx_msgs_queue);
	spin_unlock_bh(&channel->lock);
	wake_up_interruptible(&channel->rx_wait);
}

static void hsi_flash_rx_msg_destructor(struct hsi_msg *msg)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(msg->cl);
	struct hsi_flash_channel *channel = cl_data->channels + msg->channel;

	spin_lock_bh(&channel->lock);
	list_add_tail(&msg->link, &channel->free_msgs_list);
	HSI_CHST_RD_SET(channel, HSI_CHST_READOFF);
	spin_unlock_bh(&channel->lock);
}

static void hsi_flash_rx_poll_destructor(struct hsi_msg *msg)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(msg->cl);
	struct hsi_flash_channel *channel = cl_data->channels + msg->channel;

	spin_lock_bh(&channel->lock);
	list_add_tail(&msg->link, &channel->free_msgs_list);
	channel->rxpoll = HSI_FLASH_POLL_RST;
	spin_unlock_bh(&channel->lock);
}

static int hsi_flash_rx_poll(struct hsi_flash_channel *channel)
{
	struct hsi_msg *msg;
	int ret = 0;

	spin_lock_bh(&channel->lock);
	if (list_empty(&channel->free_msgs_list)) {
		ret = -ENOMEM;
		goto out;
	}
	if (channel->rxpoll == HSI_FLASH_POLL_ON)
		goto out;

	msg = list_first_entry(&channel->free_msgs_list, struct hsi_msg, link);
	list_del(&msg->link);
	channel->rxpoll = HSI_FLASH_POLL_ON;
	spin_unlock_bh(&channel->lock);
/*LCH START*/
	hsi_flash_msg_len_set(msg, POLLIN_MESSAGE_LENGTH); /*, 0);*/
/*LCH STOP*/
	msg->complete = hsi_flash_data_available;
	msg->destructor = hsi_flash_rx_poll_destructor;
	/* don't touch msg->context! */

	DPRINTK(\
	"hsi_flash_rx_poll -> allocate empty message and pass it to "
		"controller\n");
	ret = hsi_async_read(channel->cl, msg);

	spin_lock_bh(&channel->lock);
	if (ret < 0) {
		DPRINTK("hsi_flash_rx_poll hsi_async_read - ERROR\n");
		list_add_tail(&msg->link, &channel->free_msgs_list);
		channel->rxpoll = HSI_FLASH_POLL_OFF;
		goto out;
	}
out:
	spin_unlock_bh(&channel->lock);

	return ret;
}

static void hsi_flash_tx_completed(struct hsi_msg *msg)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(msg->cl);
	struct hsi_flash_channel *channel = cl_data->channels + msg->channel;

	/*DPRINTK("hsi_flash_tx_completed\n");	*/

	/*hsi_stop_tx(channel->cl);*/

	spin_lock_bh(&channel->lock);
	list_add_tail(&msg->link, &channel->tx_msgs_queue);
	channel->poll_event |= (POLLOUT | POLLWRNORM);
	spin_unlock_bh(&channel->lock);
	wake_up_interruptible(&channel->tx_wait);
}

static void hsi_flash_tx_msg_destructor(struct hsi_msg *msg)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(msg->cl);
	struct hsi_flash_channel *channel = cl_data->channels + msg->channel;

	spin_lock_bh(&channel->lock);
	list_add_tail(&msg->link, &channel->free_msgs_list);
	HSI_CHST_WR_SET(channel, HSI_CHST_WRITEOFF);
	spin_unlock_bh(&channel->lock);
}

static void hsi_flash_rx_poll_rst(struct hsi_client *cl)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(cl);
	struct hsi_flash_channel *channel = cl_data->channels;
	int i;

	for (i = 0; i < HSI_FLASH_DEVS; i++, channel++) {
		if ((HSI_CHST_OC(channel) == HSI_CHST_OPENED) &&
			(channel->rxpoll == HSI_FLASH_POLL_RST))
			hsi_flash_rx_poll(channel);
	}
}

static void hsi_flash_reset(struct hsi_client *cl)
{
	DPRINTK("hsi_flush 2\n");
	hsi_flush(cl);
	hsi_flash_rx_poll_rst(cl);
}

static void hsi_flash_rx_cancel(struct hsi_flash_channel *channel)
{
	DPRINTK("hsi_flush 3\n");
	hsi_flush(channel->cl);
	hsi_flash_rx_poll_rst(channel->cl);
}

static void hsi_flash_tx_cancel(struct hsi_flash_channel *channel)
{
	DPRINTK("hsi_flush 4\n");
	hsi_flush(channel->cl);
	hsi_flash_rx_poll_rst(channel->cl);
}

static void hsi_flash_bcast_break(struct hsi_client *cl)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(cl);
	struct hsi_flash_channel *channel = cl_data->channels;
	int i;

	for (i = 0; i < HSI_FLASH_DEVS; i++, channel++) {
		if (HSI_CHST_OC(channel) != HSI_CHST_OPENED)
			continue;

		channel->break_event++;
		channel->poll_event |= POLLPRI;
			DPRINTK("hsi_flash_break_received => POLLPRI\n");
		wake_up_interruptible(&channel->rx_wait);
		wake_up_interruptible(&channel->tx_wait);
	}
}

static void hsi_flash_break_received(struct hsi_msg *msg)
{
	struct hsi_flash_client_data *cl_data = hsi_client_drvdata(msg->cl);

	DPRINTK("hsi_flash_break_received\n");

	if (!cl_data->attached) {
		/* Wait setup completion */
		cl_data->break_msg_receive_before_setup = msg;

		DPRINTK(\
		"hsi_flash_break_received - save msg as setup is not ended\n");

		return;
	}

	hsi_flash_bcast_break(msg->cl);
	msg->destructor(msg);
}

static void hsi_flash_break_req_destructor(struct hsi_msg *msg)
{
	hsi_free_msg(msg);
}

static int hsi_flash_break_request(struct hsi_client *cl)
{
	struct hsi_msg *msg;
	int ret = 0;

	DPRINTK("hsi_flash_break_request\n");

	msg = hsi_alloc_msg(0, GFP_KERNEL);
	if (!msg)
		return -ENOMEM;
	msg->break_frame = 1;
	msg->complete = hsi_flash_break_received;
	msg->destructor = hsi_flash_break_req_destructor;
	ret = hsi_async_read(cl, msg);
	if (ret < 0)
		hsi_free_msg(msg);

	return ret;
}

static int hsi_flash_break_send(struct hsi_client *cl)
{
	struct hsi_msg *msg;
	int ret = 0;

	msg = hsi_alloc_msg(0, GFP_ATOMIC);
	if (!msg)
		return -ENOMEM;

	/*ret = hsi_start_tx(cl);
	if (ret < 0) {
		hsi_stop_tx(cl);
		hsi_free_msg(msg);
		return ret;
	}*/

	msg->break_frame = 1;
	msg->complete = hsi_free_msg;
	msg->destructor = hsi_free_msg;
	ret = hsi_async_write(cl, msg);
	if (ret < 0) {
		hsi_free_msg(msg);
		return ret;
	}
	return ret;
}

static ssize_t hsi_flash_read(struct file *file, char __user *buf,
						size_t len, loff_t *ppos)
{
	struct hsi_flash_channel *channel = file->private_data;
	struct hsi_msg *msg = NULL;
	ssize_t ret;

	DPRINTK("hsi_flash_read\n");

	if (len == 0)
		return 0;

	if (!IS_ALIGNED(len, sizeof(u32)))
		return -EINVAL;

	if (len > max_data_size)
		len = max_data_size;

	spin_lock_bh(&channel->lock);
	if (HSI_CHST_OC(channel) != HSI_CHST_OPENED) {
		ret = -ENODEV;
		goto out;
	}
	if (HSI_CHST_RD(channel) != HSI_CHST_READOFF) {
		ret = -EBUSY;
		goto out;
	}
	if (channel->ch >= channel->cl->rx_cfg.channels) {
		ret = -ENODEV;
		goto out;
	}
	if (list_empty(&channel->free_msgs_list)) {
		ret = -ENOMEM;
		goto out;
	}

/*LCH START*/

	/* Managing here the polling case as HSI core
	doesn't support this mode*/
	if ((!list_empty(&channel->pollin_msgs_queue)) && \
(POLLIN_MESSAGE_LENGTH == len)) {
		msg = list_first_entry(&channel->pollin_msgs_queue,
				struct hsi_msg, link);

		HSI_CHST_RD_SET(channel, HSI_CHST_READOFF);
		channel->poll_event &= ~(POLLIN | POLLRDNORM);

		spin_unlock_bh(&channel->lock);

		DPRINTK("###hsi_flash_read _ POLLIN part : 0x%08X\n",
						*(u32 *)msg->context);

		ret = copy_to_user((void __user *)buf,
					msg->context,
					hsi_flash_msg_len_get(msg));

		spin_lock_bh(&channel->lock);

		if (ret)
			ret = -EFAULT;
		else
			ret = hsi_flash_msg_len_get(msg);

		list_del(&msg->link);

		channel->rxpoll = HSI_FLASH_POLL_OFF;

		goto out;
	}

/*LCH STOP*/

	msg = list_first_entry(&channel->free_msgs_list, struct hsi_msg, link);
	list_del(&msg->link);
	spin_unlock_bh(&channel->lock);
	hsi_flash_msg_len_set(msg, len);
	msg->complete = hsi_flash_rx_completed;
	msg->destructor = hsi_flash_rx_msg_destructor;
	ret = hsi_async_read(channel->cl, msg);
	spin_lock_bh(&channel->lock);

	channel->rxpoll = HSI_FLASH_POLL_OFF;

	if (ret < 0)
		goto out;
	HSI_CHST_RD_SET(channel, HSI_CHST_READING);
	msg = NULL;

	for ( ; ; ) {
		DEFINE_WAIT(wait);

		if (!list_empty(&channel->rx_msgs_queue)) {
			msg = list_first_entry(&channel->rx_msgs_queue,
					struct hsi_msg, link);
			HSI_CHST_RD_SET(channel, HSI_CHST_READOFF);
			channel->poll_event &= ~(POLLIN | POLLRDNORM);
			list_del(&msg->link);
			spin_unlock_bh(&channel->lock);
			if (msg->status == HSI_STATUS_ERROR) {
				ret = -EIO;
			} else {
				ret = copy_to_user((void __user *)buf,
						msg->context,
						hsi_flash_msg_len_get(msg));

				DPRINTK("###hsi_flash_read : 0x%08X\n",
						*(u32 *)msg->context);

				if (ret)
					ret = -EFAULT;
				else
					ret = hsi_flash_msg_len_get(msg);
			}
			spin_lock_bh(&channel->lock);
			break;
		} else if (signal_pending(current)) {
			spin_unlock_bh(&channel->lock);
			hsi_flash_rx_cancel(channel);
			spin_lock_bh(&channel->lock);
			HSI_CHST_RD_SET(channel, HSI_CHST_READOFF);
			ret = -EINTR;
			break;
		} else if ((HSI_CHST_OC(channel) == HSI_CHST_CLOSING) ||
				(HSI_CHST_OC(channel) == HSI_CHST_CLOSED)) {
			ret = -EIO;
			break;
		}
		prepare_to_wait(&channel->rx_wait, &wait, TASK_INTERRUPTIBLE);
		spin_unlock_bh(&channel->lock);

		schedule();

		spin_lock_bh(&channel->lock);
		finish_wait(&channel->rx_wait, &wait);
	}
out:
	if (msg)
		list_add_tail(&msg->link, &channel->free_msgs_list);
	spin_unlock_bh(&channel->lock);


	return ret;
}

static ssize_t hsi_flash_write(struct file *file, const char __user *buf,
						size_t len, loff_t *ppos)
{
	struct hsi_flash_channel *channel = file->private_data;
	struct hsi_msg *msg = NULL;
	ssize_t ret;

	if ((len == 0) || !IS_ALIGNED(len, sizeof(u32))) {
		DPRINTK("###hsi_flash_write  fail 1\n");
		return -EINVAL;
	}

	if (len > max_data_size)
		len = max_data_size;

	spin_lock_bh(&channel->lock);
	if (HSI_CHST_OC(channel) != HSI_CHST_OPENED) {
		DPRINTK("###hsi_flash_write  fail 2\n");
		ret = -ENODEV;
		goto out;
	}
	if (HSI_CHST_WR(channel) != HSI_CHST_WRITEOFF) {
		DPRINTK("###hsi_flash_write  fail 3\n");
		ret = -EBUSY;
		goto out;
	}
	if (channel->ch >= channel->cl->tx_cfg.channels) {
		DPRINTK("###hsi_flash_write  fail 4\n");
		ret = -ENODEV;
		goto out;
	}
	if (list_empty(&channel->free_msgs_list)) {
		DPRINTK("###hsi_flash_write  fail 5\n");
		ret = -ENOMEM;
		goto out;
	}


	msg = list_first_entry(&channel->free_msgs_list, struct hsi_msg, link);
	list_del(&msg->link);
	HSI_CHST_WR_SET(channel, HSI_CHST_WRITEON);
	spin_unlock_bh(&channel->lock);

	if (copy_from_user(msg->context, (void __user *)buf, len)) {
		spin_lock_bh(&channel->lock);
		HSI_CHST_WR_SET(channel, HSI_CHST_WRITEOFF);
		DPRINTK("###hsi_flash_write  fail 6\n");
		ret = -EFAULT;
		goto out;
	}

	/*DPRINTK("###hsi_flash_write : 0x%08X\n", *(u32 *) msg->context);*/

/*
	ret = hsi_start_tx(channel->cl);

	spin_lock_bh(&channel->lock);
	if (ret < 0) {
		hsi_stop_tx(channel->cl);
		spin_lock_bh(&channel->lock);
		channel->poll_event |= (POLLOUT | POLLWRNORM);
		HSI_CHST_WR_SET(channel, HSI_CHST_WRITEOFF);
		goto out;
	}
*/


	hsi_flash_msg_len_set(msg, len);
	msg->complete = hsi_flash_tx_completed;
	msg->destructor = hsi_flash_tx_msg_destructor;
	channel->poll_event &= ~(POLLOUT | POLLWRNORM);
	ret = hsi_async_write(channel->cl, msg);

	spin_lock_bh(&channel->lock);
	if (ret < 0) {
		channel->poll_event |= (POLLOUT | POLLWRNORM);
		HSI_CHST_WR_SET(channel, HSI_CHST_WRITEOFF);
		goto out;
	}

	HSI_CHST_WR_SET(channel, HSI_CHST_WRITING);
	msg = NULL;

	for ( ; ; ) {
		DEFINE_WAIT(wait);

		if (!list_empty(&channel->tx_msgs_queue)) {
			msg = list_first_entry(&channel->tx_msgs_queue,
					struct hsi_msg, link);
			list_del(&msg->link);
			HSI_CHST_WR_SET(channel, HSI_CHST_WRITEOFF);
			if (msg->status == HSI_STATUS_ERROR)
				ret = -EIO;
			else
				ret = hsi_flash_msg_len_get(msg);
			break;
		} else if (signal_pending(current)) {

			DPRINTK("signal_pending !!!\n");

			spin_unlock_bh(&channel->lock);
			hsi_flash_tx_cancel(channel);
			spin_lock_bh(&channel->lock);
			HSI_CHST_WR_SET(channel, HSI_CHST_WRITEOFF);
			ret = -EINTR;
			break;
		} else if ((HSI_CHST_OC(channel) == HSI_CHST_CLOSING) ||
				(HSI_CHST_OC(channel) == HSI_CHST_CLOSED)) {
			ret = -EIO;
			break;
		}
		prepare_to_wait(&channel->tx_wait, &wait, TASK_INTERRUPTIBLE);
		spin_unlock_bh(&channel->lock);

		schedule();

		spin_lock_bh(&channel->lock);
		finish_wait(&channel->tx_wait, &wait);
	}
out:

	if (msg)
		list_add_tail(&msg->link, &channel->free_msgs_list);

	spin_unlock_bh(&channel->lock);

	return ret;
}

static unsigned int hsi_flash_poll(struct file *file, poll_table *wait)
{
	struct hsi_flash_channel *channel = file->private_data;
	unsigned int ret;

	spin_lock_bh(&channel->lock);
	if ((HSI_CHST_OC(channel) != HSI_CHST_OPENED) ||
		(channel->ch >= channel->cl->rx_cfg.channels)) {
		spin_unlock_bh(&channel->lock);
		return -ENODEV;
	}
	poll_wait(file, &channel->rx_wait, wait);
	poll_wait(file, &channel->tx_wait, wait);
	ret = channel->poll_event;
	spin_unlock_bh(&channel->lock);

	hsi_flash_rx_poll(channel);
	return ret;
}

static long hsi_flash_ioctl(struct file *file, unsigned int cmd,
							unsigned long arg)
{
	struct hsi_flash_channel *channel = file->private_data;
	struct hsi_flash_client_data *cl_data = &hsi_flash_cl_data;
	unsigned int speed;
	int ret = 0;

	DPRINTK("hsi_flash_ioctl - cmd %d\n", cmd);

	if (HSI_CHST_OC(channel) != HSI_CHST_OPENED)
		return -ENODEV;

	switch (cmd) {

	case HIOCRESET:
		channel->rxpoll = HSI_FLASH_POLL_OFF;
		hsi_flash_reset(channel->cl);
		channel->break_event = 0;
		channel->wake_event.wake_edges = 0;
		hsi_flash_break_request(channel->cl);
		break;

	case HIOCSNDBRK:
		DPRINTK("send break\n");

		return hsi_flash_break_send(channel->cl);

	case HIOCSPEED:
		if (copy_from_user(&speed, (void __user *)arg,
			sizeof(unsigned int)))
			return -EFAULT;

		DPRINTK("change speed: %d Khz\n", speed);

			channel->cl->tx_cfg.speed = speed;

			ret = hsi_setup(channel->cl);

			DPRINTK("hsi_setup returns %d\n", ret);

		break;

	case HIOCGWAKE:
		DPRINTK("Get WAKE event: wake_state %d, wake_edges %d\n",
			channel->wake_event.wake_state,
			channel->wake_event.wake_edges);

		channel->poll_event &= ~POLLHUP;

		if (copy_to_user((void __user *)arg, &channel->wake_event,
				sizeof(struct hiocgwake)))
			return -EFAULT;

		channel->wake_event.wake_edges = 0;

		break;

	case HIOCGBRK:
		DPRINTK("Get BREAK event: break:%d\n", channel->break_event);

		channel->poll_event &= ~POLLPRI;

		if (copy_to_user((void __user *)arg, &channel->break_event,
				sizeof(unsigned)))
			return -EFAULT;

		channel->break_event = 0;

		break;

	case HIOCTXCTRL:
		/* Set WAKE LINE ON */
		hsi_start_tx(channel->cl);

		if (cl_data->break_msg_receive_before_setup) {
			DPRINTK(\
			"Setup is done, treat break msg reception now\n");
			hsi_flash_break_received(
				cl_data->break_msg_receive_before_setup);

			cl_data->break_msg_receive_before_setup = NULL;
		}
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return ret;
}

static int hsi_flash_open(struct inode *inode, struct file *file)
{
	struct hsi_flash_client_data *cl_data = &hsi_flash_cl_data;
	struct hsi_flash_channel *channel;
	int ret = 0, refcnt, minor = iminor(inode);

	DPRINTK("hsi_flash_open\n");

	if (minor >= HSI_FLASH_DEVS) {
		pr_err("Invalid node id (%d)\n", minor);
		return -EINVAL;
	}

	channel = cl_data->channels + minor;
	spin_lock_bh(&channel->lock);
	if ((channel->state == HSI_CHST_UNAVAIL) || (!channel->cl)) {
		ret = -ENODEV;
		goto out;
	}
	if (HSI_CHST_OC(channel) != HSI_CHST_CLOSED) {
		ret = -EBUSY;
		goto out;
	}
	HSI_CHST_OC_SET(channel, HSI_CHST_OPENING);
	spin_unlock_bh(&channel->lock);

	refcnt = atomic_inc_return(&cl_data->refcnt);

	channel->break_event = 0;
	channel->wake_event.wake_state = 0;
	channel->wake_event.wake_edges = 0;

	if (refcnt == 1) {
		if (cl_data->attached) {
			atomic_dec(&cl_data->refcnt);
			spin_lock_bh(&channel->lock);
			HSI_CHST_OC_SET(channel, HSI_CHST_CLOSED);
			ret = -EBUSY;
			goto out;
		}
		ret = hsi_claim_port(channel->cl, 0);
		if (ret < 0) {
			atomic_dec(&cl_data->refcnt);
			spin_lock_bh(&channel->lock);
			HSI_CHST_OC_SET(channel, HSI_CHST_CLOSED);
			goto out;
		}

#ifdef NEW_HSI_CORE_IF
		hsi_register_port_event(channel->cl, hsi_flash_start_stop_rx) ;
#else
		channel->cl->hsi_start_rx = hsi_flash_start_rx;
		channel->cl->hsi_stop_rx = hsi_flash_stop_rx;

#endif /* NEW_HSI_CORE_IF */

		ret = hsi_setup(channel->cl);
		DPRINTK("hsi_setup returns %d\n", ret);

		/* Prepare msg reception for break */
		ret = hsi_flash_break_request(channel->cl);
		DPRINTK("hsi_flash_break_request returns %d\n", ret);

	} else if (!cl_data->attached) {
		atomic_dec(&cl_data->refcnt);
		spin_lock_bh(&channel->lock);
		HSI_CHST_OC_SET(channel, HSI_CHST_CLOSED);
		ret = -ENODEV;
		goto out;
	}

	ret = hsi_flash_msgs_alloc(channel);

	if (ret < 0) {
		refcnt = atomic_dec_return(&cl_data->refcnt);
		if (!refcnt)
			hsi_release_port(channel->cl);
		spin_lock_bh(&channel->lock);
		HSI_CHST_OC_SET(channel, HSI_CHST_CLOSED);
		goto out;
	}
	if (refcnt == 1)
		cl_data->attached = 1;
	channel->wlrefcnt = 0;
	channel->rxpoll = HSI_FLASH_POLL_OFF;

	channel->poll_event = (POLLOUT | POLLWRNORM);
	file->private_data = channel;
	spin_lock_bh(&channel->lock);
	HSI_CHST_OC_SET(channel, HSI_CHST_OPENED);

out:
	spin_unlock_bh(&channel->lock);

	return ret;
}

static int hsi_flash_release(struct inode *inode, struct file *file)
{
	struct hsi_flash_channel *channel = file->private_data;

	DPRINTK("hsi_flash_release\n");

	/* Set WAKE LINE OFF */
	hsi_stop_tx(channel->cl);
#ifdef NEW_HSI_CORE_IF
	hsi_unregister_port_event(channel->cl);
#else
	channel->cl->hsi_start_rx = NULL;
	channel->cl->hsi_stop_rx = NULL;
#endif /* NEW_HSI_CORE_IF */

	return _hsi_flash_release(channel, 0);
}

static int hsi_flash_fasync(int fd, struct file *file, int on)
{
	struct hsi_flash_channel *channel = file->private_data;

	if (fasync_helper(fd, file, on, &channel->async_queue) < 0)
		return -EIO;

	return 0;
}

static const struct file_operations hsi_flash_fops = {
	.owner		= THIS_MODULE,
	.read		= hsi_flash_read,
	.write		= hsi_flash_write,
	.poll		= hsi_flash_poll,
	.unlocked_ioctl	= hsi_flash_ioctl,
	.open		= hsi_flash_open,
	.release	= hsi_flash_release,
	.fasync		= hsi_flash_fasync,
};

static struct hsi_client_driver hsi_flash_driver = {
	.driver = {
		.name	= "hsi_flash",
		.owner	= THIS_MODULE,
		.probe	= hsi_flash_probe,
		.remove	= hsi_flash_remove,
	},
};

static inline void hsi_flash_channel_init(struct hsi_flash_channel *channel)
{
	channel->state = HSI_CHST_AVAIL;
	INIT_LIST_HEAD(&channel->free_msgs_list);
	init_waitqueue_head(&channel->rx_wait);
	init_waitqueue_head(&channel->tx_wait);
	spin_lock_init(&channel->lock);
	INIT_LIST_HEAD(&channel->rx_msgs_queue);
	INIT_LIST_HEAD(&channel->tx_msgs_queue);
	INIT_LIST_HEAD(&channel->pollin_msgs_queue);
}

static struct cdev hsi_flash_cdev;

static int __init hsi_flash_init(void)
{
	char devname[] = "hsi_flash";
	struct hsi_flash_client_data *cl_data = &hsi_flash_cl_data;
	struct hsi_flash_channel *channel = cl_data->channels;
	unsigned long ch_mask = 0;
	unsigned int i;
	int ret;

	if ((max_data_size < 4) || (max_data_size > 0x10000) ||
		(max_data_size & (max_data_size - 1))) {
		pr_err("Invalid max read/write data size");
		return -EINVAL;
	}

	for (i = 0; i < HSI_FLASH_DEVS && channels_map[i] >= 0; i++) {
		if (channels_map[i] >= HSI_FLASH_DEVS) {
			pr_err("Invalid HSI/SSI channel specified");
			return -EINVAL;
		}
		set_bit(channels_map[i], &ch_mask);
	}

	if (i == 0) {
		pr_err("No HSI channels available");
		return -EINVAL;
	}

	memset(cl_data->channels, 0, sizeof(cl_data->channels));
	for (i = 0; i < HSI_FLASH_DEVS; i++, channel++) {
		channel->ch = i;
		channel->state = HSI_CHST_UNAVAIL;
		if (test_bit(i, &ch_mask))
			hsi_flash_channel_init(channel);
	}

	ret = hsi_register_client_driver(&hsi_flash_driver);
	if (ret) {
		pr_err("Error while registering HSI/SSI driver %d", ret);
		return ret;
	}

	ret = alloc_chrdev_region(&hsi_flash_dev, 0, HSI_FLASH_DEVS, devname);
	if (ret < 0) {
		hsi_unregister_client_driver(&hsi_flash_driver);
		return ret;
	}

	cdev_init(&hsi_flash_cdev, &hsi_flash_fops);
	ret = cdev_add(&hsi_flash_cdev, hsi_flash_dev, HSI_FLASH_DEVS);

	if (ret) {
		unregister_chrdev_region(hsi_flash_dev, HSI_FLASH_DEVS);
		hsi_unregister_client_driver(&hsi_flash_driver);
		return ret;
	}

	pr_info("HSI flash device loaded\n");

	return 0;
}
module_init(hsi_flash_init);

static void __exit hsi_flash_exit(void)
{
	cdev_del(&hsi_flash_cdev);
	unregister_chrdev_region(hsi_flash_dev, HSI_FLASH_DEVS);
	hsi_unregister_client_driver(&hsi_flash_driver);
	pr_info("HSI flash device removed\n");
}
module_exit(hsi_flash_exit);

MODULE_AUTHOR("RMC");
MODULE_ALIAS("hsi:hsi_flash");
MODULE_DESCRIPTION("HSI flash device");
MODULE_LICENSE("GPL v2");

