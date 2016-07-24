/*
 * dlp_trace.c
 *
 * Intel Mobile Communication protocol driver for modem tracing
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
#include <linux/hsi/hsi_info_board.h>
#include <linux/uaccess.h>

#include "dlp_main.h"

#define TRACE_DEVNAME	CONFIG_HSI_TRACE_DEV_NAME
#define HSI_TRACE_TEMP_BUFFERS	4

/*
 * struct trace_driver - HSI Modem trace driver protocol
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
 * @ch_ctx: Trace Channel context
*/
struct dlp_trace_ctx {
	/* Char device registration */
	int major;
	dev_t tdev;
	struct device *dev;
	struct cdev cdev;
	struct class *class;

	/* Used to prevent multiple access to device */
	unsigned int opened;
	unsigned int hangup;

	/* A waitqueue for poll/read operations */
	wait_queue_head_t read_wq;

	/* RX msg queue */
	struct list_head rx_msgs;
	atomic_t rx_msgs_count;
	unsigned int rx_msgs_count_max;

	struct dlp_channel *ch_ctx;

	unsigned int proc_pkt;
	unsigned long proc_byte;
	unsigned int dropped_pkt;
	unsigned long dropped_byte;
	unsigned int bad_sig_pkt;
	atomic_t rx_pdu_count;
};


/*
 *
 */
static void dlp_trace_complete_rx(struct hsi_msg *msg);


/*
* @brief Called to destroy the allocated msg
*
* @param msg
*/
static inline void dlp_trace_msg_destruct(struct hsi_msg *msg)
{
	struct dlp_channel *ch_ctx = msg->context;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;

	/* track total number of pdus */
	atomic_dec(&trace_ctx->rx_pdu_count);

	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);
}

/*
 * Push RX pdu on trace channel
 *
 */
static int dlp_trace_push_rx_pdu(struct dlp_channel *ch_ctx)
{
	int ret;
	struct hsi_msg *rx_msg;

	/* Allocate a new RX msg */
	rx_msg = dlp_pdu_alloc(ch_ctx->hsi_channel,
				HSI_MSG_READ,
				DLP_TRACE_RX_PDU_SIZE,
				1,
				ch_ctx,
				dlp_trace_complete_rx,
				dlp_trace_msg_destruct);

	if (!rx_msg) {
		pr_err(DRVNAME": dlp_pdu_alloc(RX) failed\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Send the RX HSI msg */
	ret = hsi_async(rx_msg->cl, rx_msg);
	if (ret) {
		pr_err(DRVNAME": hsi_async() failed, ret:%d\n", ret);
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
* @brief
*
* @param msg
*/
static void dlp_trace_complete_rx(struct hsi_msg *msg)
{
	struct dlp_channel *ch_ctx = msg->context;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	u32 *header = sg_virt(msg->sgt.sgl);
	unsigned long flags;
	int count, ret;

	/* Check link readiness, TTY and trace channel still opened */
	if (!dlp_tty_is_link_valid()) {
		pr_debug(DRVNAME ": TRACE: CH%d PDU ignored (close:%d, Time out: %d)\n",
				ch_ctx->ch_id,
				dlp_drv.tty_closed, dlp_drv.tx_timeout);
		/* Delete the received msg */
		dlp_pdu_free(msg, msg->channel);
		return;
	}

	/* Check the PDU status & signature */
	if (msg->status != HSI_STATUS_COMPLETED) {
		pr_err(DRVNAME": Invalid PDU status: %d (ignored)\n",
				msg->status);
		goto push_again;
	} else if (!DLP_HEADER_VALID_SIGNATURE(header[0])) {
		if (!trace_ctx->bad_sig_pkt) {
			pr_err("\n" DRVNAME ": Invalid PDU signature 0x%x\n",
				header[0]);

			/* Dump the first 64 bytes */
			print_hex_dump(KERN_DEBUG,
				DRVNAME"_LOG", DUMP_PREFIX_OFFSET,
				16, 4, header, 64, 1);
		}
		trace_ctx->bad_sig_pkt += 1;
		goto push_again;
	}


	/* Still have space in the rx queue ? */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (atomic_read(&trace_ctx->rx_msgs_count) >= DLP_HSI_TRACE_WAIT_FIFO) {
		/* Just drop the msg */
		trace_ctx->dropped_pkt += 1;

		/* extract net byte for statistics */
		count = DLP_PACKET_IN_PDU_COUNT;
		do {
			/* Get the size value */
			header += 2;
			trace_ctx->dropped_byte +=
				DLP_HDR_DATA_SIZE(*header) - DLP_HDR_SPACE_AP;
		} while((*header & DLP_HDR_MORE_DESC) && (count-- > 0));
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
		goto push_again;
	}

	/* Queue the msg to the read queue */
	list_add_tail(&msg->link, &trace_ctx->rx_msgs);

	/* for SMP, because we check only rx_msgs_count */
	barrier();

	/* Update the counters */
	atomic_inc(&trace_ctx->rx_msgs_count);
	if (atomic_read(&trace_ctx->rx_msgs_count) > trace_ctx->rx_msgs_count_max)
		trace_ctx->rx_msgs_count_max = atomic_read(&trace_ctx->rx_msgs_count);
	trace_ctx->proc_pkt++;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* Wakeup any waiting clients for read/poll */
	wake_up_interruptible(&trace_ctx->read_wq);
	return;

push_again:
	/* Push again the RX msg */
	ret = hsi_async(msg->cl, msg);
	if (ret) {
		pr_err(DRVNAME": hsi_async failed (%d), FIFO will be empty\n",
				ret);

		/* Delete the received msg */
		dlp_pdu_free(msg, msg->channel);
	}
}


/*
 * Called when a process tries to open the device file
 */
static int dlp_trace_dev_open(struct inode *inode, struct file *filp)
{
	int ret, state;
	unsigned long flags;
	struct dlp_channel *ch_ctx = DLP_CHANNEL_CTX(DLP_CHANNEL_TRACE);
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;

	/* Check if the the channel is not already opened by the NET IF */
	state = dlp_ctrl_get_channel_state(ch_ctx->hsi_channel);
	if (state != DLP_CH_STATE_CLOSED) {
		pr_err(DRVNAME ": Can't open CH%d (HSI CH%d) => invalid state: %d\n",
				ch_ctx->ch_id, ch_ctx->hsi_channel, state);
		return -EBUSY;
	}

	/* Update/Set the eDLP channel id */
	dlp_drv.channels_hsi[ch_ctx->hsi_channel].edlp_channel = ch_ctx->ch_id;

	/* Only ONE instance of this device can be opened */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (trace_ctx->opened) {
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
		pr_err(DRVNAME": Trace channel ALREADY opened");
		return -EBUSY;
	}
	/* Set the open flag */
	trace_ctx->opened = 1;
	/* reset hangup flag */
	trace_ctx->hangup = 0;
	/* reset statistics */
	trace_ctx->proc_pkt = trace_ctx->proc_byte = 0;
	trace_ctx->dropped_pkt = 0;
	trace_ctx->dropped_byte = 0;
	trace_ctx->bad_sig_pkt = 0;
	trace_ctx->rx_msgs_count_max = 0;

	/* remove old data from wait queue */
	while (atomic_read(&trace_ctx->rx_msgs_count) > 0) {
		struct hsi_msg *msg;

		/* sanity check. shall never fail */
		if (list_empty(&trace_ctx->rx_msgs)) {
			atomic_set(&trace_ctx->rx_msgs_count, 0);
			pr_crit(DRVNAME": message list unexpected empty\n");
			ret = -EIO;
			goto cleanup_err;
		}

		msg = list_entry(trace_ctx->rx_msgs.next,
			struct hsi_msg, link);

		/* Update the counter */
		atomic_dec(&trace_ctx->rx_msgs_count);

		/* for SMP, because we check only rx_msgs_count */
		barrier();

		/* Remove the item from the list */
		list_del_init(&msg->link);

		/* Read done => Queue the RX msg again */
		ret = hsi_async(msg->cl, msg);
		if (ret) {
			pr_err(DRVNAME": hsi_async failed (%d) FIFO "
				"will be empty\n", ret);

			/* Delete the received msg */
			dlp_pdu_free(msg, msg->channel);
			ret = -EIO;
			goto cleanup_err;
		}
	}

	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* Save private data for future use */
	filp->private_data = ch_ctx;

	/* Disable the flow control */
	ch_ctx->use_flow_ctrl = 1;

	/* Push missing RX PDUs */
	while (atomic_read(&trace_ctx->rx_pdu_count)
	    < (DLP_HSI_TRACE_WAIT_FIFO + HSI_TRACE_TEMP_BUFFERS)) {
		ret = dlp_trace_push_rx_pdu(ch_ctx);
		if (ret) {
			pr_err(DRVNAME ": ch%d open failed while pushing "
				"RX-buffer to controller!\n", ch_ctx->ch_id);
			goto err;
		}
		atomic_inc(&trace_ctx->rx_pdu_count);
	}

	/* Reply to any waiting OPEN_CONN command */
	ret = dlp_ctrl_send_ack_nack(ch_ctx);
	if (ret) {
		pr_err(DRVNAME ": ch%d open failed while sending ack_nack!\n", ch_ctx->ch_id);
		ret = -EIO;
		goto err;
	}

	if (dlp_drv.sys_info->mdm_ver == MODEM_7160) {
		ret = dlp_ctrl_open_channel(ch_ctx);
		if (ret) {
			pr_err(DRVNAME ": open channel(ch%d) failed :%d)\n",
				   ch_ctx->hsi_channel, ret);
			goto err;
		}
	} else {
		/* device opened => Set the channel state flag */
		dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
		DLP_CH_STATE_OPENED);
	}

	pr_debug(DRVNAME": Trace Channel opened");
	return ret;

err:
	/* release trace channel */
	spin_lock_irqsave(&ch_ctx->lock, flags);
cleanup_err:
	/* set the hangup flag to make the poll function return an error*/
	trace_ctx->hangup = 1;
	/* Set the open flag */
	trace_ctx->opened = 0;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	return ret;
}

/*
 * Called when a process closes the device file.
 */
static int dlp_trace_dev_close(struct inode *inode, struct file *filp)
{
	struct dlp_channel *ch_ctx = filp->private_data;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	unsigned long flags;

	pr_debug(DRVNAME": Close Trace channel");

	spin_lock_irqsave(&ch_ctx->lock, flags);
	/* set the hangup flag to make the poll function return an error*/
	trace_ctx->hangup = 1;
	/* Set the open flag */
	trace_ctx->opened = 0;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	/* device closed => Set the channel state flag */
	dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_CLOSED);

	/*
	 * Wake up the read waitqueue to unblock the poll_wait
	 */
	wake_up_interruptible(&trace_ctx->read_wq);

	return 0;
}

/*
 * Called when a process, which already opened the dev file, attempts to
 * read from it.
 */
static ssize_t dlp_trace_dev_read(struct file *filp,
			   char __user *data,
			   size_t available,
			   loff_t *ppos)
{
	struct dlp_channel *ch_ctx = filp->private_data;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	struct hsi_msg *msg;
	int ret, to_copy, copied, more_packets, left, packets;
	unsigned int data_size, offset;
	unsigned char *data_addr, *start_addr;
	unsigned int *ptr;
	unsigned long flags;

	if (unlikely(available == 0))
		return 0;

	if (unlikely(available < 0))
		return -EINVAL;

	if (unlikely(!trace_ctx->opened))
		return -EWOULDBLOCK;

	/* List empty ? */
	if (atomic_read(&trace_ctx->rx_msgs_count) <= 0) {
		/* Descriptor opened in Non-Blocking mode ? */
		if (filp->f_flags & O_NONBLOCK) {
			return -EWOULDBLOCK;
		} else {
			ret = wait_event_interruptible(trace_ctx->read_wq,
				   (atomic_read(&trace_ctx->rx_msgs_count) > 0)
				|| (!trace_ctx->opened));
			if (ret) {
				return -EINTR;
			}
			if (!trace_ctx->opened)
				return -EBADF;
		}
	}

	left = copied = 0;

	/* Parse RX msgs queue */
	do {
		msg = list_entry(trace_ctx->rx_msgs.next,
			struct hsi_msg, link);

		ptr = sg_virt(msg->sgt.sgl);
		start_addr = (unsigned char *)ptr;
		packets = DLP_PACKET_IN_PDU_COUNT;

		do {
			/* Get the start offset */
			ptr++;
			offset = (*ptr);

			/* Get the size & address */
			ptr++;
			more_packets = (*ptr) & DLP_HDR_MORE_DESC;
			data_size  = DLP_HDR_DATA_SIZE((*ptr));
			data_size -= DLP_HDR_SPACE_AP;
			data_addr = start_addr + offset + DLP_HDR_SPACE_AP;

			/* CP data PDU header sanity check for max number
			   of packets per PDU and receive buffer overflow */
			if (--packets <= 0) {
				pr_err(DRVNAME ": number of trace package "
					"exceeded!\n");
				more_packets = 0; /* recycle rest of PDU */
				break;
			}
			if ((offset + DLP_HDR_SPACE_AP + data_size)
			    > DLP_TRACE_RX_PDU_SIZE) {
				pr_err(DRVNAME ": trace buffer overflow!\n");
				more_packets = 0; /* recycle rest of PDU */
				break;
			}

			/* short cut, when we have to process multi
			   pack trace messages the 2nd time */
			if (unlikely(data_size == 0))
				continue;

			/* Calculate the data size */
			if (available >= data_size) {
				to_copy = data_size;

				/* write back we have complete processed */
				if (more_packets) {
					*ptr = DLP_HDR_MORE_DESC
					     | DLP_HDR_SPACE_AP;
				}
			} else {
				/* Note, a left > 0 will later result in
				   available == 0, so we will leave the loop */
				to_copy = available;
				left = data_size - available;

				/* write back left size */
				*ptr-- = more_packets
				       | (left + DLP_HDR_SPACE_AP);

				/* write back new offset */
				*ptr++ = offset + to_copy;
			}


			/* Copy data to the user buffer */
			ret = copy_to_user(data+copied, data_addr, to_copy);
			if (ret) {
				/* Stop copying */
				pr_err(DRVNAME": Unable to copy data to user "
						"buffer\n");
				break;
			}

			copied += to_copy;
			available -= to_copy;
			trace_ctx->proc_byte += to_copy;
		} while ((more_packets) && (available > 0));

		if (!left && !more_packets) {
			spin_lock_irqsave(&ch_ctx->lock, flags);

			/* Update the counter */
			atomic_dec(&trace_ctx->rx_msgs_count);

			/* for SMP, because we check only rx_msgs_count */
			barrier();

			/* Remove the item from the list */
			list_del_init(&msg->link);

			spin_unlock_irqrestore(&ch_ctx->lock, flags);

			/* Read done => Queue the RX msg again */
			ret = hsi_async(msg->cl, msg);
			if (ret) {
				pr_err(DRVNAME": hsi_async failed (%d) FIFO "
					"will be empty\n", ret);

				/* Delete the received msg */
				dlp_pdu_free(msg, msg->channel);
			}
		}

	} while ((available > 0) && (atomic_read(&trace_ctx->rx_msgs_count) > 0));

	/* Update the position */
	(*ppos) += copied;

	return copied;
}

/*
 * Called when a process writes to dev file
 */
static ssize_t dlp_trace_dev_write(struct file *filp,
		const char __user *data,
		size_t count,
		loff_t *ppos)
{
	pr_err(DRVNAME": Modem trace TX path is not allowed !\n");
	return 0;
}


/*
* @brief
*
* @param filp
* @param wait
*
* @return
*/
static unsigned int dlp_trace_dev_poll(struct file *filp,
		struct poll_table_struct *pt)
{
	struct dlp_channel *ch_ctx;
	struct dlp_trace_ctx *trace_ctx;
	unsigned long flags;
	unsigned int ret = 0;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
				__func__);
		ret = POLLHUP;
		goto out;
	}
	ch_ctx = filp->private_data;
	trace_ctx = ch_ctx->ch_data;

	/* if the channel is hang-up/closed no reason to wait*/
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (trace_ctx->hangup) {
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
		ret = POLLHUP;
		goto out;
	}
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

	poll_wait(filp, &trace_ctx->read_wq, pt);

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
				__func__);
		ret = POLLHUP;
		goto out;
	}
	/* Have some data to read ? */
	spin_lock_irqsave(&ch_ctx->lock, flags);
	if (trace_ctx->hangup) {
		/* The close function has been executed <=> hangup */
		ret = POLLHUP;
	} else if (atomic_read(&trace_ctx->rx_msgs_count) > 0) {
		ret = POLLIN | POLLRDNORM;
	}
	spin_unlock_irqrestore(&ch_ctx->lock, flags);

out:
	return ret;
}

/**
 * dlp_net_hsi_tx_timeout_cb - Called when we have an HSI TX timeout
 * @ch_ctx : Channel context ref
 */
static void dlp_trace_dev_tx_timeout_cb(struct dlp_channel *ch_ctx)
{
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	unsigned long flags;

	spin_lock_irqsave(&ch_ctx->lock, flags);
	/* Set hangup flag */
	trace_ctx->hangup = 1;
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
	wake_up_interruptible(&trace_ctx->read_wq);

}


/*
 * PDU Header analyzer
 * Count all trace payload bytes in the PDU and Trace packages
 * print output into a string vaiable
 *
 * @param *str : pointer to string, used as sprintf write buffer
 * @param *format : const format string, used to format the sprintf
 * @param *pdu : pointer to PDU messages buffer
 * @return: adjusted pointer to string
 *
 * Context: Any
 * Notes: Take care to prepare enough space in str buffer
 *        Fkt doesn't check PDU header or sequence validity!
 */
static char *pdu_to_hd_ana(char *str, int strSize, const char *format, struct hsi_msg *pdu) {
	u32 *ptr;
	bool more_packets;
	unsigned int size=0, packages=0;
	int value;

	/* Start Address & Size */
	ptr = sg_virt(pdu->sgt.sgl);
	ptr+=2;

	do {
		more_packets = ((*ptr) & DLP_HDR_MORE_DESC);

		/* Read the size (mask the N/P fields) */
		size += DLP_HDR_DATA_SIZE((*ptr)) - DLP_HDR_SPACE_AP;

		/* Go to the next packet desc */
		ptr += 2;
		packages++;

	} while ((more_packets) && (packages < DLP_PACKET_IN_PDU_COUNT));

	value = snprintf(str, strSize, format, packages, size);
	if (value < 0) {
		/* Buffer size too small */
		pr_err(DRVNAME": in pdu_to_hd_ana, str size is not big enough\n");
	} else {
		str += value;
	}
	return str;
}



/*
* @brief Dump information about the trace channel state
*
* @param ch_ctx : channel context to consider
* @param m : seq file to consider
*
*/
static void dlp_trace_dump_channel_state(struct dlp_channel *ch_ctx, struct seq_file *m)
{
	unsigned long flags;
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;

	struct list_head *curr;
	struct hsi_msg *pdu;
	int i;
	char s[20];

	seq_printf(m, "\nChannel: %d\n", ch_ctx->hsi_channel);
	seq_printf(m, "-------------\n");
	seq_printf(m, " state    : %d\n",
			dlp_drv.channels_hsi[ch_ctx->hsi_channel].state);
	seq_printf(m, " credits  : %d\n", ch_ctx->credits);
	seq_printf(m, " flow ctrl: %d\n", ch_ctx->use_flow_ctrl);
	seq_printf(m, " hangup: %d\n", trace_ctx->hangup);
	seq_printf(m, " opened: %d\n", trace_ctx->opened);

	/* Dump the RX context info */
	seq_printf(m, "\n RX ctx:\n");
	spin_lock_irqsave(&ch_ctx->lock, flags);
	seq_printf(m, "   wait_max    : %d\n", DLP_HSI_TRACE_WAIT_FIFO);
	seq_printf(m, "   ctrl_max    : %d\n", DLP_HSI_TRACE_WAIT_FIFO + HSI_TRACE_TEMP_BUFFERS);
	seq_printf(m, "   wait_len    : %d\n", atomic_read(&trace_ctx->rx_msgs_count));
	seq_printf(m, "   wait_len_max: %d\n", trace_ctx->rx_msgs_count_max);
	seq_printf(m, "   total_len   : %d\n", atomic_read(&trace_ctx->rx_pdu_count));
	seq_printf(m, "   proc_pkt    : %u\n", trace_ctx->proc_pkt);
	seq_printf(m, "   proc_net    : %lu Byte\n", trace_ctx->proc_byte);
	seq_printf(m, "   drop_pkt    : %d\n", trace_ctx->dropped_pkt);
	seq_printf(m, "   drop_net    : %lu Byte\n", trace_ctx->dropped_byte);
	seq_printf(m, "   bad_sig_pkt : %d\n", trace_ctx->bad_sig_pkt);
	seq_printf(m, "   pdu_size    : %d\n", ch_ctx->rx.pdu_size);

	seq_printf(m, "   Waiting PDUs:\n");
	i = 0;
	list_for_each(curr, &trace_ctx->rx_msgs) {
		pdu = list_entry(curr, struct hsi_msg, link);
		pdu_to_hd_ana(s, sizeof(s), "%2d Pack : %d", pdu);
		seq_printf(m, "      %02d: %s Byte\n", ++i, s);
	}
	spin_unlock_irqrestore(&ch_ctx->lock, flags);
}


/*
* Device driver file operations
*/
static const struct file_operations dlp_trace_ops = {
	.open	= dlp_trace_dev_open,
	.read	= dlp_trace_dev_read,
	.write	= dlp_trace_dev_write,
	.poll	= dlp_trace_dev_poll,
	.release = dlp_trace_dev_close
};

static int dlp_trace_ctx_cleanup(struct dlp_channel *ch_ctx);

/*
* @brief
*
* @param ch_id
* @param hsi_channel
* @param dev
*
* @return
*/
struct dlp_channel *dlp_trace_ctx_create(unsigned int ch_id,
		unsigned int hsi_channel,
		struct device *dev)
{
	int ret;
	struct hsi_client *client = to_hsi_client(dev);
	struct dlp_channel *ch_ctx;
	struct dlp_trace_ctx *trace_ctx;

	/* Allocate channel struct data */
	ch_ctx = kzalloc(sizeof(struct dlp_channel), GFP_KERNEL);
	if (!ch_ctx) {
		pr_err(DRVNAME": Out of memory (ch%d)\n", ch_id);
		return NULL;
	}

	/* Allocate the context private data */
	trace_ctx = kzalloc(sizeof(struct dlp_trace_ctx), GFP_KERNEL);
	if (!trace_ctx) {
		pr_err(DRVNAME": Out of memory (trace_ctx)\n");
		goto free_ch;
	}

	/* Save params */
	ch_ctx->ch_data = trace_ctx;
	ch_ctx->ch_id = ch_id;
	ch_ctx->hsi_channel = hsi_channel;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_waitqueue_head(&trace_ctx->read_wq);
	INIT_LIST_HEAD(&trace_ctx->rx_msgs);
	atomic_set(&trace_ctx->rx_msgs_count, 0);
	atomic_set(&trace_ctx->rx_pdu_count, 0);

	/* Register debug, cleanup CBs */
	ch_ctx->dump_state = dlp_trace_dump_channel_state;
	ch_ctx->cleanup = dlp_trace_ctx_cleanup;

	/* Hangup context */
	dlp_ctrl_hangup_ctx_init(ch_ctx, dlp_trace_dev_tx_timeout_cb);

	/* Init the RX/TX contexts */
	dlp_xfer_ctx_init(ch_ctx,
			DLP_TRACE_TX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			DLP_TRACE_RX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_READ);

	/* Register the device */
	ret = alloc_chrdev_region(&trace_ctx->tdev, 0, 1, TRACE_DEVNAME);
	if (ret) {
		pr_err(DRVNAME": Unable to allocate the device (err: %d)\n",
				ret);
		goto free_ctx;
	}

	trace_ctx->major = MAJOR(trace_ctx->tdev);
	cdev_init(&trace_ctx->cdev, &dlp_trace_ops);
	trace_ctx->cdev.owner = THIS_MODULE;

	ret = cdev_add(&trace_ctx->cdev, trace_ctx->tdev, 1);
	if (ret) {
		pr_err(DRVNAME": Unable to register the device (err: %d)\n",
				ret);
		goto unreg_reg;
	}

	trace_ctx->class = class_create(THIS_MODULE, DRVNAME"-trace");
	if (IS_ERR(trace_ctx->class))
		goto del_cdev;

	trace_ctx->dev = device_create(trace_ctx->class,
			NULL,
			trace_ctx->tdev,
			NULL, TRACE_DEVNAME);
	if (IS_ERR(trace_ctx->dev)) {
		pr_err(DRVNAME": Unable to create the device (err: %ld)\n",
				PTR_ERR(trace_ctx->dev));
		goto del_class;
	}

	return ch_ctx;

del_class:
	class_destroy(trace_ctx->class);

del_cdev:
	cdev_del(&trace_ctx->cdev);

unreg_reg:
	unregister_chrdev_region(trace_ctx->tdev, 1);

free_ctx:
	kfree(trace_ctx);

free_ch:
	kfree(ch_ctx);
	return NULL;
}

/*
* @brief This function will delete/unregister
*	the char device and class
*
* @param ch_ctx: Trace channel context
*
* @return 0 when sucess, error code otherwise
*/
static int dlp_trace_ctx_cleanup(struct dlp_channel *ch_ctx)
{
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;
	int ret = 0;

	/* Unregister/Delete char device & class */
	device_destroy(trace_ctx->class, trace_ctx->tdev);
	cdev_del(&trace_ctx->cdev);
	unregister_chrdev_region(trace_ctx->tdev, 1);
	class_destroy(trace_ctx->class);

	return ret;
}

/*
 * This function will release the allocated memory
 * done in the _ctx_create function
 */
int dlp_trace_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_trace_ctx *trace_ctx = ch_ctx->ch_data;

	/* Free the Trace context */
	kfree(trace_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);
	return 0;
}

