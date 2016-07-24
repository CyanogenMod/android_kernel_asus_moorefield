/*
 * dlp_tty.c
 *
 * Intel Mobile Communication modem protocol driver for DLP
 * (Data Link Protocl (LTE)). This driver is implementing a 5-channel HSI
 * protocol consisting of:
 * - An internal communication control channel;
 * - A multiplexed channel exporting a TTY interface;
 * - Three dedicated high speed channels exporting each a network interface.
 * All channels are using fixed-length pdus, although of different sizes.
 *
 * Copyright (C) 2010-2011 Intel Corporation. All rights reserved.
 *
 * Contact: Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>
 *          Faouaz Tenoutit <faouazx.tenoutit@intel.com>
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

#include <linux/log2.h>
#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/jiffies.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/hsi/hsi_dlp.h>
#include <linux/hsi/hsi.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/version.h>

#include "dlp_main.h"

#define IPC_TTYNAME				"tty"CONFIG_HSI_DLP_IPC_TTY_NAME

#define RX_TTY_FORWARDING_BIT		(1<<DLP_GLOBAL_STATE_SZ)
#define RX_TTY_REFORWARD_BIT		(2<<DLP_GLOBAL_STATE_SZ)

#define TX_TTY_WRITE_PENDING_BIT	(1<<DLP_GLOBAL_STATE_SZ)

/*
 * struct dlp_tty_context - TTY channel private data
 *
 * @tty_drv: TTY driver struct
 * @tty_prt: TTY port struct
 * @buffer_wq: Workqueue for tty buffer flush
 * @ch_ctx : Channel context ref
 * @do_tty_forward: dedicated TTY forwarding work structure
 */
struct dlp_tty_context {
	struct tty_port tty_prt;
	struct tty_driver *tty_drv;

	struct dlp_channel *ch_ctx;
	struct work_struct	do_tty_forward;
};


/**
 * Push as many RX PDUs  as possible to the controller FIFO
 *
 * @param ch_ctx : The channel context to consider
 *
 * @return 0 when OK, error value otherwise
 */
static int dlp_tty_push_rx_pdus(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	struct dlp_xfer_ctx *rx_ctx = &ch_ctx->rx;

	ret = dlp_pop_recycled_push_ctrl(rx_ctx);
	if (ret == -EAGAIN) {
		mod_timer(&rx_ctx->timer, jiffies + rx_ctx->delay);
		ret = 0;
	}

	return ret;
}

/**
 * dlp_tty_pdu_data_ptr - helper function for getting the actual virtual address of
 *	a pdu data, taking into account the header offset
 *
 * @pdu: a reference to the considered pdu
 * @offset: an offset to add to the current virtual address of the pdu data
 *
 * Returns the virtual base address of the actual pdu data
 */
inline __attribute_const__
unsigned char *dlp_tty_pdu_data_ptr(struct hsi_msg *pdu, unsigned int offset)
{
	u32 *addr = sg_virt(pdu->sgt.sgl);

	/* Skip the Signature (+2) & Start address (+2) */
	addr += 4;
	return ((unsigned char *)addr) + offset;
}

/**
 * dlp_tty_pdu_set_length - write down the length information to the frame header
 * @pdu: a reference to the considered pdu
 * @sz: the length information to encode in the header
 */
inline void dlp_tty_pdu_set_length(struct hsi_msg *pdu, u32 sz)
{
	u32 *header = (u32 *) (sg_virt(pdu->sgt.sgl));
	header[1] = DLP_TTY_HEADER_LENGTH;
	header[2] = DLP_HDR_NO_MORE_DESC |
		DLP_HDR_COMPLETE_PACKET | DLP_HDR_DATA_SIZE(sz);
}

/**
 * dlp_tty_wakeup - wakeup an asleep TTY write function call
 * @ch_ctx: a reference to the context related to this TTY
 *
 * This helper function awakes any asleep TTY write callback function.
 */
static void dlp_tty_wakeup(struct dlp_channel *ch_ctx)
{
	struct tty_struct *tty;
	struct dlp_tty_context *tty_ctx = ch_ctx->ch_data;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	tty_port_tty_wakeup(&tty_ctx->tty_prt);
#else
	tty = tty_port_tty_get(&tty_ctx->tty_prt);
	if (likely(tty)) {
		tty_wakeup(tty);
		tty_kref_put(tty);
	}
#endif
}

/**
 * _dlp_forward_tty - RX data TTY forwarding helper function
 * @tty: a reference to the TTY where the data shall be forwarded
 * @xfer_ctx: a reference to the RX context where the FIFO of waiting pdus sits
 *
 * Data contained in the waiting pdu FIFO shall be forwarded to the TTY.
 * This function is :
 *	- Pushing as much data as possible to the TTY interface
 *  - Recycling pdus that have been fully forwarded
 *  - Kicking a TTY insert
 *  - Restart delayed job if some data is remaining in the waiting FIFO
 *	  or if the controller FIFO is not full yet.
 */
static void _dlp_forward_tty(struct tty_struct *tty,
			    struct dlp_xfer_ctx *xfer_ctx)
{
	struct hsi_msg *pdu;
	unsigned long flags;
	unsigned char *data_addr, *start_addr;
	unsigned int copied, data_size, offset, more_packets;
	int *ptr, do_push, ret;
	char tty_flag;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	struct dlp_channel *ch_ctx = (struct dlp_channel *)tty->driver_data;
	struct dlp_tty_context *tty_ctx = ch_ctx->ch_data;
#endif

	/* Initialised to 1 to prevent unexpected TTY forwarding resume
	 * function when there is no TTY or when it is throttled */
	copied = 1;
	do_push = 0;

	del_timer_sync(&xfer_ctx->timer);

	read_lock_irqsave(&xfer_ctx->lock, flags);
	while (xfer_ctx->wait_len > 0) {
		read_unlock_irqrestore(&xfer_ctx->lock, flags);

		write_lock_irqsave(&xfer_ctx->lock, flags);
		pdu = dlp_fifo_wait_pop(xfer_ctx);
		write_unlock_irqrestore(&xfer_ctx->lock, flags);
		if (!pdu)
			goto no_more_tty_insert;

		if (pdu->status == HSI_STATUS_COMPLETED)
			tty_flag = (likely(!pdu->break_frame)) ?
			    TTY_NORMAL : TTY_BREAK;
		else
			tty_flag = TTY_FRAME;

		if (unlikely(!tty))
			goto free_pdu;

		/* Read packets desc  */
		/*---------------------*/
		ptr = sg_virt(pdu->sgt.sgl);
		start_addr = (unsigned char *)ptr;

		do {
			if (test_bit(TTY_THROTTLED, &tty->flags)) {
				/* Initialised to 1 to prevent unexpected TTY
				 * forwarding resume function schedule */
				copied = 1;
				dlp_fifo_wait_push_back(xfer_ctx, pdu);
				goto no_more_tty_insert;
			}

			/* Get the start offset */
			ptr++;
			offset = (*ptr);

			/* Get the size & address */
			ptr++;
			more_packets = (*ptr) & DLP_HDR_MORE_DESC;
			data_size = DLP_HDR_DATA_SIZE((*ptr));
			data_addr = start_addr + offset;

			/* Copy the data to the TTY buffer */
			do {

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
				copied = (unsigned int)
					tty_insert_flip_string_fixed_flag(&tty_ctx->tty_prt,
							data_addr,
							tty_flag,
							data_size);
#else
				copied = (unsigned int)
					tty_insert_flip_string_fixed_flag(tty,
							data_addr,
							tty_flag,
							data_size);
#endif

				data_addr += copied;
				data_size -= copied;

				/* We'll push the flip buffers each
				 * time something has been written
				 * to them to allow low latency */
				do_push |= (copied > 0);
			} while ((data_size) && (copied));

			/* Still have not copied data ? */
			if (data_size) {
				dlp_fifo_wait_push_back(xfer_ctx, pdu);
				goto no_more_tty_insert;
			}
		} while ((more_packets) && (copied));

free_pdu:
		/* Reset the pdu offset & length */
		dlp_pdu_reset(xfer_ctx,
			      pdu,
			      xfer_ctx->payload_len + DLP_TTY_HEADER_LENGTH);

		/* Recycle or free the pdu */
		dlp_pdu_recycle(xfer_ctx, pdu);

		read_lock_irqsave(&xfer_ctx->lock, flags);
	}

	read_unlock_irqrestore(&xfer_ctx->lock, flags);

no_more_tty_insert:
	if (do_push) {
		/* Schedule a flip since called from complete_rx()
		 * in an interrupt context instead of
		 * tty_flip_buffer_push() */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
		tty_schedule_flip(&tty_ctx->tty_prt);
#else
		tty_schedule_flip(tty);
#endif

	}
	/* Push any available pdus to the CTRL */
	ret = dlp_pop_recycled_push_ctrl(xfer_ctx);

	/* Shoot again later if there is still pending data to serve or if
	 * the RX controller FIFO is not full yet */
	if ((!copied) || (unlikely(ret == -EAGAIN)))
		mod_timer(&xfer_ctx->timer, jiffies + xfer_ctx->delay);
}

/**
 * dlp_do_tty_forward - forwarding data to the above line discipline
 * @work: a reference to work queue element
 */
static void dlp_do_tty_forward(struct work_struct *work)
{
	struct dlp_tty_context *tty_ctx;
	struct tty_struct *tty;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
				__func__);
		return;
	}

	tty_ctx = container_of(work, struct dlp_tty_context, do_tty_forward);
	tty = tty_port_tty_get(&tty_ctx->tty_prt);
	if (tty) {
		/* Lock really needed ?
		 * We are using a single thread workqueue,
		 * so works are executed sequentially */
		_dlp_forward_tty(tty, &tty_ctx->ch_ctx->rx);
		tty_kref_put(tty);
	}
}

/**
 * dlp_tty_rx_forward_retry - TTY forwarding retry job
 * @param: a casted reference to the to the RX context where the FIFO of
 *	   waiting pdus sits
 *
 * This simply calls the TTY forwarding function in a tasklet shell.
 */
static void dlp_tty_rx_forward_retry(unsigned long param)
{
	struct dlp_xfer_ctx *xfer_ctx = (struct dlp_xfer_ctx *)param;
	struct dlp_tty_context *tty_ctx = xfer_ctx->channel->ch_data;

	queue_work(dlp_drv.rx_wq, &tty_ctx->do_tty_forward);
}

/**
 * dlp_tty_rx_forward_resume - TTY forwarding resume callback
 * @tty: a reference to the TTY requesting the resume
 *
 * This simply calls the TTY forwarding function as a response to a TTY
 * unthrottle event.
 */
static void dlp_tty_rx_forward_resume(struct tty_struct *tty)
{
	struct dlp_channel *ch_ctx;

	/* Get the context reference from the driver data if already opened */
	ch_ctx = (struct dlp_channel *)tty->driver_data;

	if (ch_ctx) {
		struct dlp_tty_context *tty_ctx = ch_ctx->ch_data;
		queue_work(dlp_drv.rx_wq, &tty_ctx->do_tty_forward);
	}
}

/**
 * dlp_tty_complete_tx - bottom-up flow for the TX side
 * @pdu: a reference to the completed pdu
 *
 * A TX transfer has completed: recycle the completed pdu and kick a new
 * delayed request to enter the IDLE state if nothing else is expected.
 */
static void dlp_tty_complete_tx(struct hsi_msg *pdu)
{
	struct dlp_xfer_ctx *xfer_ctx = pdu->context;
	struct dlp_channel *ch_ctx = xfer_ctx->channel;
	int wakeup, avail, pending;
	unsigned long flags;

	/* Recycle or Free the pdu */
	write_lock_irqsave(&xfer_ctx->lock, flags);
	dlp_pdu_delete(xfer_ctx, pdu, flags);

	/* Decrease the CTRL fifo size */
	dlp_hsi_controller_pop(xfer_ctx);

	/* Check the wait FIFO size */
	avail = (xfer_ctx->wait_len <= xfer_ctx->wait_max / 2);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	/* Start new waiting pdus (if any) */
	dlp_pop_wait_push_ctrl(xfer_ctx);

	/* Wake-up the TTY write whenever the TX wait FIFO is half empty, and
	 * not before, to prevent too many wakeups */
	pending = dlp_ctx_has_flag(xfer_ctx, TX_TTY_WRITE_PENDING_BIT);
	wakeup = (pending && avail);
	if (wakeup) {
		dlp_ctx_clear_flag(xfer_ctx, TX_TTY_WRITE_PENDING_BIT);
		dlp_tty_wakeup(ch_ctx);
	}
}

/**
 * dlp_tty_complete_rx - bottom-up flow for the RX side
 * @pdu: a reference to the completed pdu
 *
 * A RX transfer has completed: push the data conveyed in the pdu to the TTY
 * interface and signal any existing error.
 */
static void dlp_tty_complete_rx(struct hsi_msg *pdu)
{
	struct dlp_xfer_ctx *xfer_ctx = pdu->context;
	struct dlp_tty_context *tty_ctx = xfer_ctx->channel->ch_data;
	unsigned long flags;
	int ret;

	/* Check the link readiness (TTY still opened) */
	if (!dlp_tty_is_link_valid()) {
		if ((EDLP_TTY_RX_DATA_REPORT) ||
			(EDLP_TTY_RX_DATA_LEN_REPORT))
				pr_debug(DRVNAME ": TTY: CH%d RX PDU ignored (close:%d, Time out: %d)\n",
					xfer_ctx->channel->ch_id,
					dlp_drv.tty_closed, dlp_drv.tx_timeout);
		goto recycle;
	}

	/* Check the received PDU header & seq_num */
	ret = dlp_pdu_header_check(xfer_ctx, pdu);
	if (ret == -EINVAL) {
		/* Dump the first 64 bytes */
		print_hex_dump(KERN_DEBUG,
				DRVNAME"_TTY", DUMP_PREFIX_OFFSET,
				16, 4,
				sg_virt(pdu->sgt.sgl), 64, 1);

		goto recycle;
	}

	/* Check and update the PDU len & status */
	dlp_pdu_update(tty_ctx->ch_ctx, pdu);

	/* Dump the RX data/length */
	if (EDLP_TTY_RX_DATA_REPORT)
		print_hex_dump(KERN_DEBUG,
				DRVNAME": TTY_RX", DUMP_PREFIX_OFFSET,
				16, 4,
				dlp_tty_pdu_data_ptr(pdu, 0),
				MIN(64, pdu->actual_len), 1);
	else if (EDLP_TTY_RX_DATA_LEN_REPORT)
		pr_debug(DRVNAME ": TTY_RX %d bytes\n", pdu->actual_len);

	/* Decrease the CTRL fifo size */
	write_lock_irqsave(&xfer_ctx->lock, flags);
	dlp_hsi_controller_pop(xfer_ctx);
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	if (pdu->status != HSI_STATUS_ERROR)
		dlp_fifo_wait_push(xfer_ctx, pdu);
	else {
		pr_debug(DRVNAME ": TTY: CH%d RX PDU ignored\n",
						xfer_ctx->channel->ch_id);
		goto recycle;
	}
#ifdef CONFIG_HSI_DLP_TTY_STATS
	xfer_ctx->tty_stats.data_sz += pdu->actual_len;
	xfer_ctx->tty_stats.pdus_cnt++;
	if (!xfer_ctx->ctrl_len)
		xfer_ctx->tty_stats.overflow_cnt++;
#endif
	queue_work(dlp_drv.rx_wq, &tty_ctx->do_tty_forward);
	return;

recycle:
	/* Recycle or free the pdu */
	dlp_pdu_recycle(xfer_ctx, pdu);
}

/**
 * dlp_tty_tx_fifo_wait_recycle - recycle the whole content of the TX waiting FIFO
 * @xfer_ctx: a reference to the TX context to consider
 *
 * This helper function is emptying a waiting TX FIFO and recycling all its
 * pdus.
 */
static void dlp_tty_tx_fifo_wait_recycle(struct dlp_xfer_ctx *xfer_ctx)
{
	struct hsi_msg *pdu;
	unsigned long flags;

	dlp_ctx_clear_flag(xfer_ctx, TX_TTY_WRITE_PENDING_BIT);

	write_lock_irqsave(&xfer_ctx->lock, flags);

	while ((pdu = dlp_fifo_wait_pop(xfer_ctx))) {
		xfer_ctx->room -= dlp_pdu_room_in(pdu);

		/* check if pdu is active in dlp_tty_do_write */
		if (pdu->status != HSI_STATUS_PENDING)
			dlp_pdu_delete(xfer_ctx, pdu, flags);
		else
			pdu->break_frame = 0;
	}

	write_unlock_irqrestore(&xfer_ctx->lock, flags);
}

/**
 * dlp_tty_rx_fifo_wait_recycle - recycle the whole content of the RX waiting FIFO
 * @xfer_ctx: a reference to the RX context to consider
 *
 * This helper function is emptying a waiting RX FIFO and recycling all its
 * pdus.
 */
static void dlp_tty_rx_fifo_wait_recycle(struct dlp_xfer_ctx *xfer_ctx)
{
	struct hsi_msg *pdu;
	unsigned long flags;
	unsigned int length = xfer_ctx->payload_len + DLP_TTY_HEADER_LENGTH;

	dlp_ctx_clear_flag(xfer_ctx,
			   RX_TTY_FORWARDING_BIT | RX_TTY_REFORWARD_BIT);

	write_lock_irqsave(&xfer_ctx->lock, flags);

	while ((pdu = dlp_fifo_wait_pop(xfer_ctx))) {
		/* Reset offset & length */
		dlp_pdu_reset(xfer_ctx, pdu, length);

		/* Recycle or Free the pdu */
		dlp_pdu_delete(xfer_ctx, pdu, flags);
	}

	write_unlock_irqrestore(&xfer_ctx->lock, flags);
}

/*
 * TTY handling methods
 */

/**
 * dlp_tty_wait_until_ctx_sent - waits for all the TX FIFO to be empty
 * @ch_ctx: a reference to the considered context
 * @timeout: a timeout value expressed in jiffies
 */
inline void dlp_tty_wait_until_ctx_sent(struct dlp_channel *ch_ctx, int timeout)
{
	wait_event_interruptible_timeout(ch_ctx->tx_empty_event,
					 dlp_ctx_is_empty(&ch_ctx->tx),
					 timeout);
}

/**
 * dlp_tty_cleanup - clear timers and flush all TX/RX pending
 * @ch_ctx : Channel context ref
 *
 */
static void dlp_tty_cleanup(struct dlp_channel *ch_ctx)
{
	struct dlp_tty_context *tty_ctx;
	struct dlp_xfer_ctx *tx_ctx;
	struct dlp_xfer_ctx *rx_ctx;
	int ret;

	tty_ctx = ch_ctx->ch_data;
	tx_ctx = &ch_ctx->tx;
	rx_ctx = &ch_ctx->rx;

	del_timer_sync(&dlp_drv.timer[ch_ctx->ch_id]);

	/* Flush any pending fw work */
	flush_work_sync(&tty_ctx->do_tty_forward);

	/* RX */
	del_timer_sync(&rx_ctx->timer);
	dlp_tty_rx_fifo_wait_recycle(rx_ctx);
	dlp_stop_rx(rx_ctx, ch_ctx);

	/* TX */
	del_timer_sync(&tx_ctx->timer);
	dlp_stop_tx(tx_ctx);
	dlp_tty_tx_fifo_wait_recycle(tx_ctx);

	dlp_ctx_set_state(tx_ctx, IDLE);

	/* Close the HSI channel */
	ret = dlp_ctrl_close_channel(ch_ctx);
	if (ret)
		pr_err(DRVNAME ": TT close channel failed :%d\n", ret);

	/* Flush the ACWAKE works */
	cancel_work_sync(&ch_ctx->start_tx_w);
	cancel_work_sync(&ch_ctx->stop_tx_w);
}

/**
 * dlp_tty_port_activate - callback to the TTY port activate function
 * @port: a reference to the calling TTY port
 * @tty: a reference to the calling TTY
 *
 * Return 0 on success or a negative error code on error.
 *
 * The TTY port activate is only called on the first port open.
 */
static int dlp_tty_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct dlp_channel *ch_ctx;
	struct dlp_xfer_ctx *tx_ctx;
	struct dlp_xfer_ctx *rx_ctx;
	int ret = 0;

	pr_debug(DRVNAME": port activate request\n");

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return -ENODEV;
	}

	/* Get the context reference stored in the TTY open() */
	ch_ctx = (struct dlp_channel *)tty->driver_data;
	tx_ctx = &ch_ctx->tx;
	rx_ctx = &ch_ctx->rx;

	/* Update the TX and RX HSI configuration */
	dlp_ctx_update_status(tx_ctx);
	dlp_ctx_update_status(rx_ctx);

	/* Configure the DLP channel */
	ret = dlp_ctrl_open_channel(ch_ctx);
	if (ret)
		pr_err(DRVNAME ": TTY open channel failed :%d)\n", ret);

	pr_debug(DRVNAME ": port activate done (ret: %d)\n", ret);
	return ret;
}

/**
 * dlp_tty_port_shutdown - callback to the TTY port shutdown function
 * @port: a reference to the calling TTY port
 *
 * The TTY port shutdown is only called on the last port close.
 */
static void dlp_tty_port_shutdown(struct tty_port *port)
{
	struct dlp_channel *ch_ctx;
	struct dlp_tty_context *tty_ctx;
	struct dlp_xfer_ctx *tx_ctx;
	struct dlp_xfer_ctx *rx_ctx;

	pr_debug(DRVNAME": port shutdown request\n");

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return;
	}

	tty_ctx = container_of(port, struct dlp_tty_context, tty_prt);
	ch_ctx = tty_ctx->ch_ctx;
	tx_ctx = &ch_ctx->tx;
	rx_ctx = &ch_ctx->rx;

	/* Don't wait if already in TX timeout state */
	if (dlp_tty_is_link_valid())
		dlp_tty_wait_until_ctx_sent(ch_ctx, 0);

	/* TTY channel cleanup */
	dlp_tty_cleanup(ch_ctx);

	/* device closed => Set the channel state flag */
	dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_CLOSED);

	pr_debug(DRVNAME ": port shutdown done\n");
}

/**
 * dlp_tty_open - callback to the TTY open function
 * @tty: a reference to the calling TTY
 * @filp: a reference to the calling file
 *
 * Return 0 on success or a negative error code on error.
 *
 * The HSI layer is only initialised during the first opening.
 */
static int dlp_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct dlp_channel *ch_ctx;
	struct dlp_tty_context *tty_ctx;
	int ret;

	pr_debug(DRVNAME": TTY device open request (%s, %d)\n",
			current->comm, current->tgid);

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		ret = -ENODEV;
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		goto out;
	}

	/* Get the context reference from the driver data if already opened */
	ch_ctx = (struct dlp_channel *)tty->driver_data;

	/* First open ? */
	if (!ch_ctx) {
		ch_ctx = dlp_drv.channels[DLP_CHANNEL_TTY];
		tty->driver_data = ch_ctx;
	}

	if (unlikely(!ch_ctx)) {
		ret = -ENODEV;
		pr_err(DRVNAME ": Cannot find TTY context\n");
		goto out;
	}

	/* Needed only once */
	if (tty->count == 1) {
		/* Reset the Tx timeout/TTY close flag */
		dlp_tty_set_link_valid(0, 0);

		/* Claim & Setup the HSI port */
		dlp_hsi_port_claim();

		/* Register the event callback */
		hsi_register_port_event(dlp_drv.client, dlp_drv.ehandler);

		/* Push RX pdus for ALL channels */
		dlp_push_rx_pdus();
	}

	/* Update/Set the eDLP channel id */
	dlp_drv.channels_hsi[ch_ctx->hsi_channel].edlp_channel = ch_ctx->ch_id;

	/* Open the TTY port (calls port->activate on first opening) */
	tty_ctx = ch_ctx->ch_data;
	ret = tty_port_open(&tty_ctx->tty_prt, tty, filp);
	if (ret)
		pr_err(DRVNAME ": TTY port open failed (%d)\n", ret);

	/* Set the TTY_NO_WRITE_SPLIT to transfer as much data as possible on
	 * the first write request. This shall not introduce denial of service
	 * as this flag will later adapt to the available TX buffer size. */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);

out:
	pr_debug(DRVNAME ": TTY device open done (ret: %d)\n", ret);
	return ret;
}

/**
 * dlp_tty_flush_tx_buffer - flushes the TX waiting FIFO
 * @tty: a reference to the requesting TTY
 */
static void dlp_tty_flush_tx_buffer(struct tty_struct *tty)
{
	struct dlp_channel *ch_ctx = (struct dlp_channel *)tty->driver_data;
	struct dlp_xfer_ctx *xfer_ctx = &ch_ctx->tx;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return;
	}

	dlp_tty_tx_fifo_wait_recycle(xfer_ctx);
}

/**
 * dlp_tty_tx_stop - update the TX state machine after expiration of the TX active
 *		 timeout further to a no outstanding TX transaction status
 * @param: a hidden reference to the TX context to consider
 *
 * This helper function updates the TX state if it is currently active and
 * inform the HSI pduwork and attached controller.
 */
void dlp_tty_tx_stop(unsigned long param)
{
	struct dlp_xfer_ctx *xfer_ctx = (struct dlp_xfer_ctx *)param;

	dlp_stop_tx(xfer_ctx);
}

/**
 * dlp_tty_hsi_tx_timeout_cb - Called when we have an HSI TX timeout
 * @ch_ctx : Channel context ref
 */
static void dlp_tty_hsi_tx_timeout_cb(struct dlp_channel *ch_ctx)
{
	struct dlp_tty_context *tty_ctx = ch_ctx->ch_data;
	struct tty_struct *tty;

	tty = tty_port_tty_get(&tty_ctx->tty_prt);
	if (tty) {
		/* Let's stop all queue and cleanup */
		dlp_tty_cleanup(ch_ctx);

		/* Clean any waiting data to release potential
		   dlp_tty_wait_until_sent lock */
		hsi_flush(dlp_drv.client);

		tty_vhangup(tty);
		tty_kref_put(tty);
	}
}

/**
 * dlp_tty_hangup - callback to a TTY hangup request
 * @tty: a reference to the requesting TTY
 */
static void dlp_tty_hangup(struct tty_struct *tty)
{
	struct dlp_tty_context *tty_ctx =
	    (((struct dlp_channel *)tty->driver_data))->ch_data;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return;
	}

	pr_err(DRVNAME ": TTY hangup\n");

	/* Will call the port_shutdown function */
	tty_port_hangup(&tty_ctx->tty_prt);
}

/**
 * dlp_tty_wait_until_sent - callback to a TTY wait until sent request
 * @tty: a reference to the requesting TTY
 * @timeout: a timeout value expressed in jiffies
 */
static void dlp_tty_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct dlp_channel *ch_ctx = (struct dlp_channel *)tty->driver_data;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return;
	}

	dlp_tty_wait_until_ctx_sent(ch_ctx, timeout);
}

/**
 * dlp_tty_close - callback to the TTY close function
 * @tty: a reference to the calling TTY
 * @filp: a reference to the calling file
 *
 * The HSI layer is only released during the last closing.
 */
static void dlp_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct dlp_channel *ch_ctx = NULL;
	int need_cleanup = 0;
	if (tty) {
		need_cleanup = (tty->count == 1);

		pr_debug(DRVNAME ": TTY device close request (%s, %d)\n",
				current->comm, current->tgid);

		/* Set TTY as closed to prevent RX/TX transactions */
		if (need_cleanup)
			tty->flow_stopped = 1;

		ch_ctx = (struct dlp_channel *)tty->driver_data;
		if (filp && ch_ctx) {
			struct dlp_tty_context *tty_ctx = ch_ctx->ch_data;
			if (&tty_ctx->tty_prt)
				tty_port_close(&tty_ctx->tty_prt, tty, filp);
		}
	}

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return;
	}

	if (need_cleanup)
		dlp_tty_set_link_valid(1, dlp_drv.tx_timeout);

	/* Flush everything & Release the HSI port */
	if (likely(!atomic_read(&dlp_drv.drv_remove_ongoing)) && need_cleanup) {
		pr_debug(DRVNAME": Flushing the HSI controller\n");
		hsi_flush(dlp_drv.client);
		dlp_hsi_port_unclaim();
	}

	dlp_ctrl_clean_stored_cmd();

	pr_debug(DRVNAME ": TTY device close done\n");
}

/**
 * dlp_tty_do_write - writes data coming from the TTY to the TX FIFO
 * @xfer_ctx: a reference to the considered TX context
 * @buf: the virtual address of the current input buffer (from TTY)
 * @len: the remaining buffer size
 *
 * Returns the total size of what has been transferred.
 *
 * This is a recursive function, the core of the TTY write callback function.
 */
int dlp_tty_do_write(struct dlp_xfer_ctx *xfer_ctx, unsigned char *buf,
			    int len)
{
	struct hsi_msg *pdu;
	int offset, avail, copied;
	unsigned int updated_actual_len;
	unsigned long flags;

	offset = 0;
	avail = 0;
	copied = 0;

	spin_lock_irqsave(&xfer_ctx->channel->lock, flags);
	if (!dlp_ctx_have_credits(xfer_ctx, xfer_ctx->channel)) {
		spin_unlock_irqrestore(&xfer_ctx->channel->lock, flags);
		if ((EDLP_TTY_TX_DATA_REPORT) ||
			(EDLP_TTY_TX_DATA_LEN_REPORT))
				pr_warn(DRVNAME ": CH%d (HSI CH%d) out of credits (%d)",
					xfer_ctx->channel->ch_id,
					xfer_ctx->channel->hsi_channel,
					xfer_ctx->seq_num);
		goto out;
	}
	spin_unlock_irqrestore(&xfer_ctx->channel->lock, flags);

	write_lock_irqsave(&xfer_ctx->lock, flags);
	pdu = dlp_fifo_tail(&xfer_ctx->wait_pdus);
	if (pdu) {
		if (pdu->status != HSI_STATUS_PENDING) {
			offset = pdu->actual_len;
			avail = xfer_ctx->payload_len - offset;
			if (avail)
				pdu->status = HSI_STATUS_PENDING;
		}
	}
	write_unlock_irqrestore(&xfer_ctx->lock, flags);

	if (avail == 0) {
		pdu = dlp_fifo_recycled_pop(xfer_ctx);
		if (pdu) {
			offset = 0;

			read_lock_irqsave(&xfer_ctx->lock, flags);
			avail = xfer_ctx->payload_len;
			read_unlock_irqrestore(&xfer_ctx->lock, flags);

			dlp_fifo_wait_push(xfer_ctx, pdu);
		}
	}

	if (!pdu)
		goto out;

	/* Do a start TX on new frames only and after having marked
	 * the current frame as pending, e.g. don't touch ! */
	if (offset == 0) {
		dlp_start_tx(xfer_ctx);
	} else {
		dlp_ctx_set_flag(xfer_ctx, TX_TTY_WRITE_PENDING_BIT);
#ifdef CONFIG_HSI_DLP_TTY_STATS
		xfer_ctx->tty_stats.overflow_cnt++;
#endif
	}

	copied = min(avail, len);
	updated_actual_len = pdu->actual_len + copied;
	dlp_tty_pdu_set_length(pdu, updated_actual_len);
	(void)memcpy(dlp_tty_pdu_data_ptr(pdu, offset), buf, copied);

	if (pdu->status != HSI_STATUS_ERROR) {	/* still valid ? */
		pdu->actual_len = updated_actual_len;

		write_lock_irqsave(&xfer_ctx->lock, flags);
		xfer_ctx->buffered += copied;
		xfer_ctx->room -= copied;
		pdu->status = HSI_STATUS_COMPLETED;
		write_unlock_irqrestore(&xfer_ctx->lock, flags);

		if (dlp_ctx_get_state(xfer_ctx) != IDLE)
			dlp_pop_wait_push_ctrl(xfer_ctx);
	} else {
		/* ERROR frames have already been popped from the wait FIFO */
		write_lock_irqsave(&xfer_ctx->lock, flags);
		dlp_pdu_delete(xfer_ctx, pdu, flags);
		write_unlock_irqrestore(&xfer_ctx->lock, flags);
		copied = 0;
	}

out:
	return copied;
}

/**
 * dlp_tty_write - writes data coming from the TTY to the TX FIFO
 * @tty: a reference to the calling TTY
 * @buf: the virtual address of the current input buffer (from TTY)
 * @len: the TTY buffer size
 *
 * Returns the total size of what has been transferred in the TX FIFO
 *
 * This is the TTY write callback function.
 */
static int dlp_tty_write(struct tty_struct *tty, const unsigned char *buf,
			 int len)
{
	struct dlp_xfer_ctx *xfer_ctx =
	    &((struct dlp_channel *)tty->driver_data)->tx;
	int already_copied, copied;
	unsigned char *ptr;
	unsigned long flags;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return -ENODEV;
	}

	/* Dump the TX data/length */
	if (EDLP_TTY_TX_DATA_REPORT)
		print_hex_dump(KERN_DEBUG,
				DRVNAME": TTY_TX", DUMP_PREFIX_OFFSET,
				16, 4,
				buf, len, 1);
	else if (EDLP_TTY_TX_DATA_LEN_REPORT)
		pr_debug(DRVNAME ": TTY_TX %d bytes\n", len);

	read_lock_irqsave(&xfer_ctx->lock, flags);
	if (xfer_ctx->room >= len)
		set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	else
		clear_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	read_unlock_irqrestore(&xfer_ctx->lock, flags);

	already_copied = 0;
	while (len > 0) {
		ptr = (unsigned char *)(buf + already_copied);
		copied = dlp_tty_do_write(xfer_ctx, ptr, len);
		if (copied == 0)
			break;
		already_copied += copied;
		len -= copied;
	}

	return already_copied;
}

/**
 * dlp_tty_write_room - returns the available buffer size on the TX FIFO
 * @tty: a reference to the calling TTY
 *
 * Returns the total available size in the TX wait FIFO.
 */
static int dlp_tty_write_room(struct tty_struct *tty)
{
	struct dlp_xfer_ctx *ch_ctx =
	    &((struct dlp_channel *)tty->driver_data)->tx;
	int room;
	unsigned long flags;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return -ENODEV;
	}

	read_lock_irqsave(&ch_ctx->lock, flags);
	room = ch_ctx->room;
	read_unlock_irqrestore(&ch_ctx->lock, flags);
	return room;
}

/**
 * dlp_tty_chars_in_buffer - returns the size of the data hold in the TX FIFO
 * @tty: a reference to the calling TTY
 *
 * Returns the total size of data hold in the TX wait FIFO. It does not take
 * into account the data which has already been passed to the HSI controller
 * in both in software and hardware FIFO.
 */
static int dlp_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct dlp_xfer_ctx *ch_ctx =
	    &((struct dlp_channel *)tty->driver_data)->tx;
	int buffered;
	unsigned long flags;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return -ENODEV;
	}

	read_lock_irqsave(&ch_ctx->lock, flags);
	buffered = ch_ctx->buffered;
	read_unlock_irqrestore(&ch_ctx->lock, flags);
	return buffered;
}

/**
 * dlp_tty_ioctl - manages the IOCTL read and write requests
 * @tty: a reference to the calling TTY
 * @cmd: the IOCTL command
 * @arg: the I/O argument to pass or retrieve data
 *
 * Returns 0 upon normal completion or the error code in case of an error.
 */
static int dlp_tty_ioctl(struct tty_struct *tty,
			 unsigned int cmd, unsigned long arg)
{
	struct dlp_channel *ch_ctx = (struct dlp_channel *)tty->driver_data;
#ifdef CONFIG_HSI_DLP_TTY_STATS
	struct hsi_dlp_stats stats;
#endif
	unsigned long flags;
	int ret;

	if (unlikely(atomic_read(&dlp_drv.drv_remove_ongoing))) {
		pr_err(DRVNAME ": %s: Driver is currently removed by the system",
						__func__);
		return -ENODEV;
	}

	switch (cmd) {
#ifdef CONFIG_HSI_DLP_TTY_STATS
	case HSI_DLP_RESET_TX_STATS:
		write_lock_irqsave(&ch_ctx->tx.lock, flags);
		ch_ctx->tx.tty_stats.data_sz = 0;
		ch_ctx->tx.tty_stats.pdus_cnt = 0;
		ch_ctx->tx.tty_stats.overflow_cnt = 0;
		write_unlock_irqrestore(&ch_ctx->tx.lock, flags);
		break;

	case HSI_DLP_GET_TX_STATS:
		read_lock_irqsave(&ch_ctx->tx.lock, flags);
		stats.data_sz = ch_ctx->tx.tty_stats.data_sz;
		stats.pdus_cnt = ch_ctx->tx.tty_stats.pdus_cnt;
		stats.overflow_cnt = ch_ctx->tx.tty_stats.overflow_cnt;
		read_unlock_irqrestore(&ch_ctx->tx.lock, flags);
		return copy_to_user((void __user *)arg, &stats, sizeof(stats));
		break;

	case HSI_DLP_RESET_RX_STATS:
		write_lock_irqsave(&ch_ctx->rx.lock, flags);
		ch_ctx->rx.tty_stats.data_sz = 0;
		ch_ctx->rx.tty_stats.pdus_cnt = 0;
		ch_ctx->rx.tty_stats.overflow_cnt = 0;
		write_unlock_irqrestore(&ch_ctx->rx.lock, flags);
		break;

	case HSI_DLP_GET_RX_STATS:
		read_lock_irqsave(&ch_ctx->rx.lock, flags);
		stats.data_sz = ch_ctx->rx.tty_stats.data_sz;
		stats.pdus_cnt = ch_ctx->rx.tty_stats.pdus_cnt;
		stats.overflow_cnt = ch_ctx->rx.tty_stats.overflow_cnt;
		read_unlock_irqrestore(&ch_ctx->rx.lock, flags);
		return copy_to_user((void __user *)arg, &stats, sizeof(stats));
		break;
#endif

	case HSI_DLP_SET_FLASHING_MODE:
		ret = dlp_set_flashing_mode(arg);
		break;

	default:
		return -ENOIOCTLCMD;
	}

	return 0;
}

/*
 * Protocol driver handling routines
 */

/*
 * dlp_termios_init - default termios initialisation
 */
static const struct ktermios dlp_termios_init = {
	.c_iflag = 0,
	.c_oflag = 0,
	.c_cflag = B115200 | CS8,
	.c_lflag = 0,
	.c_cc = INIT_C_CC,
	.c_ispeed = 0,
	.c_ospeed = 0
};

/*
 * dlp_driver_tty_ops - table of supported TTY operations
 */
static const struct tty_operations dlp_driver_tty_ops = {
	.open = dlp_tty_open,
	.close = dlp_tty_close,
	.write = dlp_tty_write,
	.write_room = dlp_tty_write_room,
	.chars_in_buffer = dlp_tty_chars_in_buffer,
	.ioctl = dlp_tty_ioctl,
	.hangup = dlp_tty_hangup,
	.wait_until_sent = dlp_tty_wait_until_sent,
	.unthrottle = dlp_tty_rx_forward_resume,
	.flush_buffer = dlp_tty_flush_tx_buffer,
};

/*
 * dlp_port_tty_ops - table of supported TTY port operations
 */
static const struct tty_port_operations dlp_port_tty_ops = {
	.activate = dlp_tty_port_activate,
	.shutdown = dlp_tty_port_shutdown,
};

/****************************************************************************
 *
 * Exported functions
 *
 ***************************************************************************/

static int dlp_tty_ctx_cleanup(struct dlp_channel *ch_ctx);

struct dlp_channel *dlp_tty_ctx_create(unsigned int ch_id,
		unsigned int hsi_channel,
		struct device *dev)
{
	struct hsi_client *client = to_hsi_client(dev);
	struct tty_driver *new_drv;
	struct dlp_channel *ch_ctx;
	struct dlp_tty_context *tty_ctx;
	struct hsi_msg *hsi_msg, *hsi_msg_tmp;
	int ret;

	ch_ctx = kzalloc(sizeof(struct dlp_channel), GFP_KERNEL);
	if (!ch_ctx) {
		pr_err(DRVNAME ": Out of memory (tty_ch_ctx)\n");
		return NULL;
	}

	/* Allocate the context private data */
	tty_ctx = kzalloc(sizeof(struct dlp_tty_context), GFP_KERNEL);
	if (!tty_ctx) {
		pr_err(DRVNAME ": Out of memory (tty_ctx)\n");
		goto free_ch;
	}

	/* Allocate & configure the TTY driver */
	new_drv = alloc_tty_driver(1);
	if (!new_drv) {
		pr_err(DRVNAME ": alloc_tty_driver failed\n");
		goto free_ctx;
	}

	new_drv->magic = TTY_DRIVER_MAGIC;
	new_drv->owner = THIS_MODULE;
	new_drv->driver_name = DRVNAME;
	new_drv->name = IPC_TTYNAME;
	new_drv->minor_start = 0;
	new_drv->num = DLP_TTY_DEV_NUM;
	new_drv->type = TTY_DRIVER_TYPE_SERIAL;
	new_drv->subtype = SERIAL_TYPE_NORMAL;
	new_drv->flags = TTY_DRIVER_REAL_RAW | TTY_DRIVER_DYNAMIC_DEV;
	new_drv->init_termios = dlp_termios_init;

	tty_set_operations(new_drv, &dlp_driver_tty_ops);

	/* Register the TTY driver */
	ret = tty_register_driver(new_drv);
	if (ret) {
		pr_err(DRVNAME ": tty_register_driver failed (%d)\n", ret);
		goto free_drv;
	}

	ch_ctx->ch_data = tty_ctx;
	ch_ctx->ch_id = ch_id;
	ch_ctx->hsi_channel = hsi_channel;
	/* Temporay test waiting for the modem FW */
	if (dlp_drv.flow_ctrl)
		ch_ctx->use_flow_ctrl = 1;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	init_waitqueue_head(&ch_ctx->tx_empty_event);

	/* Hangup context */
	dlp_ctrl_hangup_ctx_init(ch_ctx, dlp_tty_hsi_tx_timeout_cb);

	/* Register the PDUs push, Reset/Coredump, cleanup CBs */
	ch_ctx->push_rx_pdus = dlp_tty_push_rx_pdus;
	ch_ctx->dump_state = dlp_dump_channel_state;
	ch_ctx->cleanup = dlp_tty_ctx_cleanup;

	/* TX & RX contexts */
	dlp_xfer_ctx_init(ch_ctx,
			  DLP_TTY_TX_PDU_SIZE, DLP_HSI_TX_DELAY,
			  DLP_HSI_TX_WAIT_FIFO, DLP_HSI_TX_CTRL_FIFO,
			  dlp_tty_complete_tx, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_TTY_RX_PDU_SIZE, DLP_HSI_RX_DELAY,
			  DLP_HSI_RX_WAIT_FIFO, DLP_HSI_RX_CTRL_FIFO,
			  dlp_tty_complete_rx, HSI_MSG_READ);

	INIT_WORK(&ch_ctx->start_tx_w, dlp_do_start_tx);
	INIT_WORK(&ch_ctx->stop_tx_w, dlp_do_send_nop);

	INIT_WORK(&tty_ctx->do_tty_forward, dlp_do_tty_forward);

	ch_ctx->tx.timer.function = dlp_tty_tx_stop;
	ch_ctx->rx.timer.function = dlp_tty_rx_forward_retry;

	/* Register the TTY device (port) */
	tty_port_init(&(tty_ctx->tty_prt));
	tty_ctx->tty_prt.ops = &dlp_port_tty_ops;

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	if (!tty_port_register_device(&tty_ctx->tty_prt, new_drv, 0, dev)) {
#else
	if (!tty_register_device(new_drv, 0, dev)) {
#endif

		pr_err(DRVNAME ": tty_register_device failed (%d)\n", ret);
		goto unreg_drv;
	}

	tty_ctx->ch_ctx = ch_ctx;
	tty_ctx->tty_drv = new_drv;

	/* Allocate TX FIFO */
	ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->tx);
	if (ret) {
		pr_err(DRVNAME ": Cant allocate TX FIFO pdus for ch%d\n",
				ch_id);
		goto cleanup;
	}

	/* Allocate RX FIFO */
	ret = dlp_allocate_pdus_pool(ch_ctx, &ch_ctx->rx);
	if (ret) {
		pr_err(DRVNAME ": Cant allocate RX FIFO pdus for ch%d\n",
				ch_id);
		goto free_tx_fifo;
	}

	return ch_ctx;

unreg_drv:
	tty_unregister_driver(new_drv);

free_drv:
	put_tty_driver(new_drv);

free_ctx:
	kfree(tty_ctx);

free_ch:
	kfree(ch_ctx);

	pr_err(DRVNAME": Failed to create context for ch%d", ch_id);
	return NULL;

free_tx_fifo:
	/* Free tx fifo */
	list_for_each_entry_safe(hsi_msg, hsi_msg_tmp,
				&ch_ctx->tx.recycled_pdus, link) {
		list_del(&hsi_msg->link);
		dlp_pdu_free(hsi_msg, hsi_msg->channel);
	}

cleanup:
	dlp_tty_ctx_delete(ch_ctx);

	pr_err(DRVNAME": Failed to create context for ch%d", ch_id);
	return NULL;
}

/*
* @brief This function will:
*	- Delete TX/RX timer
*	- Flush RX/TX queues
*	- Unregister the TTY device
*
* @param ch_ctx: TTY channel context
*
* @return 0 when sucess, error code otherwise
*/
static int dlp_tty_ctx_cleanup(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	struct dlp_tty_context *tty_ctx = ch_ctx->ch_data;
	struct tty_struct *tty;

	dlp_tty_cleanup(ch_ctx);

	tty = tty_port_tty_get(&tty_ctx->tty_prt);
	if (tty) {
		/* Clean any waiting data to release potential
		   dlp_tty_wait_until_sent lock */
		hsi_flush(dlp_drv.client);

		tty_vhangup(tty);
		tty_kref_put(tty);
	}

	/* Clear the hangup context */
	dlp_ctrl_hangup_ctx_deinit(ch_ctx);

	/* Unregister device */
	tty_unregister_device(tty_ctx->tty_drv, 0);

	/* Unregister driver */
	tty_unregister_driver(tty_ctx->tty_drv);

	/* Free */
	put_tty_driver(tty_ctx->tty_drv);
	tty_ctx->tty_drv = NULL;

	/* Delete the xfers context */
	dlp_xfer_ctx_clear(&ch_ctx->tx);
	dlp_xfer_ctx_clear(&ch_ctx->rx);

	return ret;
}

/*
 * This function will release the allocated memory
 * done in the _ctx_create function
 */
int dlp_tty_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_tty_context *tty_ctx = ch_ctx->ch_data;

	/* Free the tty_ctx */
	kfree(tty_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);
	return 0;
}



/*
 * Set the TTY close/TX timeout state
 */
void dlp_tty_set_link_valid(int tty_closed, int tx_timeout)
{
	unsigned long flags;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	dlp_drv.tty_closed = tty_closed;
	dlp_drv.tx_timeout = tx_timeout;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);
}

/*
 * Check the TTY link readiness state
 *
 * @return:
 *   - 0 (invalid) : in case of TTY close or TX timeout
 *   - 1 (valid)    : Otherwise
 */
int dlp_tty_is_link_valid(void)
{
	int valid = 1;
	unsigned long flags;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	if ((dlp_drv.tty_closed) || (dlp_drv.tx_timeout))
		valid = 0;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);

	return valid;
}
