/*
 * dlp_ctrl.c
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

#include <linux/slab.h>
#include <linux/sched.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/jiffies.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/hsi/hsi_dlp.h>
#include <linux/hsi/hsi.h>

#include "dlp_main.h"

/*
 *   23            16 15            8  7              0
 *  |                |                |                |
 *   ................ ................ ................
 *
 *        PARAM3            PARAM2           PARAM1
 */

#define PARAM1(w)  ((unsigned char) (w))
#define PARAM2(w)  ((unsigned char)((w) >> 8))
#define PARAM3(w)  ((unsigned char)((w) >> 16))

#define LSB(b)    ((unsigned char) ((b) &  0xF))
#define MSB(b)    ((unsigned char) ((b) >> 4))
#define CMD_ID(b, id) (LSB(b) | (id << 4))
#define CMD_ID_ERR(p, err) (CMD_ID(p->data3, p->id) | err)

#define DLP_CMD_TX_TIMOUT		 1000  /* 1 sec */
#define DLP_CMD_RX_TIMOUT	     1000  /* 1 sec */

#define DLP_ECHO_CMD_CHECKSUM	0xACAFFE
#define DLP_NOP_CMD_CHECKSUM	0xE7E7EC

#define DLP_CTRL_CTX	(dlp_drv.channels[DLP_CHANNEL_CTRL]->ch_data)

/* DLP commands list */
#define DLP_CMD_BREAK				0x00
#define DLP_CMD_ECHO				0x01
#define DLP_CMD_NOP					0x04
#define DLP_CMD_CONF_CH				0x05
#define DLP_CMD_OPEN_CONN			0x07
#define DLP_CMD_CLOSE_CONN			0x0A
#define DLP_CMD_ACK					0x0B
#define DLP_CMD_NACK				0x0C
#define DLP_CMD_OPEN_CONN_OCTET		0x0E
#define DLP_CMD_CREDITS				0x0F
#define DLP_CMD_NONE				0xFF

/* Flow control */
enum {
	DLP_FLOW_CTRL_NONE,
	DLP_FLOW_CTRL_CREDITS
};

/* Data format */
enum {
	DLP_DATA_FORMAT_RAW,
	DLP_DATA_FORMAT_PACKET
};

/* Direction */
enum {
	DLP_DIR_TRANSMIT,
	DLP_DIR_RECEIVE,
	DLP_DIR_TRANSMIT_AND_RECEIVE
};

/**
 * struct dlp_command_params - DLP modem comamnd/response
 * @data1: Command data (byte1)
 * @data2: Command data (byte2)
 * @data3: Command data (byte3)
 * @channel: the HSI channel number
 * @id: The command id
 */
struct dlp_command_params {
	unsigned char data1:8;
	unsigned char data2:8;
	unsigned char data3:8;

	unsigned char channel:4;
	unsigned char id:4;
};

/**
 * struct dlp_command - DLP comamnd
 * @params: DLP modem comamnd/response
 * @channel: the DLP channel context
 * @status: Status of the transfer when completed
 */
struct dlp_command {
	struct dlp_command_params params;

	struct dlp_channel *channel;
	int status;
};

/*
 * struct dlp_ctrl_context - CTRL channel private data
 *
 * @wq: Modem readiness/TX timeout worqueue
 * @tx_timeout_work: Modem TX timeout work
 * @response: Received response from the modem
 * @ehandler: HSI client events CB
 * @open_lock: Lock to prevent concurrent open issues
 */
struct dlp_ctrl_context {
	/* Modem readiness/TX timeout work & worqueue */
	struct workqueue_struct *wq;
	struct work_struct tx_timeout_work;

	/* Modem response */
	struct dlp_command response;

	/* HSI events callback */
	hsi_client_cb ehandler;

	/* Lock for concurrent open operations */
	spinlock_t open_lock;
};

/**
 * dlp_ctrl_hsi_tx_timout_cb - timer function for tx timeout
 * @param: a reference to the channel to consider
 *
 */
static void dlp_ctrl_hsi_tx_timout_cb(unsigned long int param)
{
	struct dlp_channel *ch_ctx = (struct dlp_channel *)param;
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;

	pr_debug(DRVNAME ": HSI TX Timeout (CH%d, HSI CH%d)\n",
			ch_ctx->ch_id, ch_ctx->hsi_channel);

	/* Set the reason & launch the work to handle the hangup */
	dlp_drv.tx_timeout = 1;
	queue_work(ctrl_ctx->wq, &ctrl_ctx->tx_timeout_work);
}

/**
 * dlp_ctrl_handle_tx_timeout - Manage the HSI TX timeout
 * @work: a reference to work queue element
 *
 * Required since hsi_port->flush calls might sleep
 */
static void dlp_ctrl_handle_tx_timeout(struct work_struct *work)
{
	struct dlp_channel *ch_ctx;
	int i;

	pr_err(DRVNAME ": Processing HSI TX Timeout\n");

	/* Call any register TX timeout CB */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		ch_ctx = DLP_CHANNEL_CTX(i);
		/* Call the channel callback function */
		if ((ch_ctx) && (ch_ctx->modem_tx_timeout_cb))
			ch_ctx->modem_tx_timeout_cb(ch_ctx);
	}
}

/*
 * This function will check if the CTRL channel ctx is created
 *	(In some special cases (PnP tests for exp), the eDLP protocol will
 *  not be registered and the CTRL channel is not initialized at all,
 *  but the module_param are created
 */
static inline int dlp_ctrl_have_control_context(void)
{
	int have_ctrl = 0;
	struct dlp_channel *ctrl_ch = DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL);

	if ((ctrl_ch) && (ctrl_ch->ch_data))
		have_ctrl = 1;

	return have_ctrl;
}

/*
 * Check if the provided PDU size is compatible with the
 * channel PDU size
 *
 *  Return the response to send ACK/NACK
 */
static int dlp_ctrl_check_pdu_size(struct dlp_channel *ch_ctx,
					struct dlp_command_params *params,
					struct dlp_command_params *tx_params)
{
	int size, response;

	memcpy(tx_params, params, sizeof(struct dlp_command_params));

	size = ((params->data2 << 8) | params->data1);

	/* OPEN_CONN => ACK by default */
	response = DLP_CMD_ACK;
	tx_params->data3 = CMD_ID(params->data3, params->id);

	/* Check the requested PDU size */
	if (ch_ctx->rx.pdu_size != size) {
		pr_err(DRVNAME ": ch%d wrong pdu size %d (expected %d)\n",
				ch_ctx->hsi_channel,
				size,
				ch_ctx->rx.pdu_size);

		/* OPEN_CONN => NACK (Unexpected PDU size) */
		response = DLP_CMD_NACK;

		/* Set the response params */
		tx_params->data1 = 0;
		tx_params->data2 = 0;
		tx_params->data3 = CMD_ID_ERR(params,
				EDLP_ERR_WRONG_PDU_SIZE);
	}

	return response;
}

/****************************************************************************
 *
 * Control flow
 *
 *
 ***************************************************************************/

/*
 *
 */
static struct dlp_command *dlp_ctrl_cmd_alloc(struct dlp_channel *ch_ctx,
					      unsigned char id,
					      unsigned char param1,
					      unsigned char param2,
					      unsigned char param3)
{
	struct dlp_command *dlp_cmd;
	int flags = in_interrupt() ? GFP_ATOMIC : GFP_KERNEL;

	/* Allocate DLP command */
	dlp_cmd = kmalloc(sizeof(struct dlp_command), flags);
	if (!dlp_cmd) {
		pr_err(DRVNAME ": Out of memory (dlp_cmd: 0x%X)\n", id);
		goto out;
	}

	/* Set command params */
	dlp_cmd->params.id = id;
	dlp_cmd->params.channel = ch_ctx->hsi_channel;
	dlp_cmd->params.data1 = param1;
	dlp_cmd->params.data2 = param2;
	dlp_cmd->params.data3 = param3;

	dlp_cmd->channel = ch_ctx;

out:
	return dlp_cmd;
}

/*
 *
 */
static inline void dlp_ctrl_cmd_free(struct dlp_command *dlp_cmd)
{
	kfree(dlp_cmd);
}

/*
 *
 */
static void dlp_ctrl_msg_destruct(struct hsi_msg *msg)
{
	struct dlp_command *dlp_cmd = msg->context;

	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);

	/* Delete the command */
	kfree(dlp_cmd);
}

/*
 * Send eDLP response
 */

/* Forward declaration */
static void dlp_ctrl_complete_tx_async(struct hsi_msg *msg);

static int dlp_ctrl_send_response(struct dlp_channel *ch_ctx,
					struct dlp_command_params *tx_params,
					int response)
{
	int ret, state;
	struct hsi_msg *tx_msg;
	struct dlp_command *dlp_cmd;
	unsigned long flags;

	/* Allocate the eDLP response */
	dlp_cmd = dlp_ctrl_cmd_alloc(ch_ctx,
				     response,
				     tx_params->data1,
				     tx_params->data2, tx_params->data3);
	if (!dlp_cmd) {
		pr_err(DRVNAME ": Out of memory (dlp_cmd: 0x%X)\n", response);
		return -ENOMEM;
	}

	/* Allocate a new TX msg */
	tx_msg = dlp_pdu_alloc(DLP_CHANNEL_CTRL,
			       HSI_MSG_WRITE,
			       DLP_CTRL_TX_PDU_SIZE,
			       1,
			       dlp_cmd,
			       dlp_ctrl_complete_tx_async,
			       dlp_ctrl_msg_destruct);

	if (!tx_msg) {
		pr_err(DRVNAME ": TX alloc failed\n");

		/* Delete the command */
		kfree(dlp_cmd);
		return -ENOMEM;
	}

	/* Copy the command data */
	memcpy(sg_virt(tx_msg->sgt.sgl),
	       &dlp_cmd->params, sizeof(struct dlp_command_params));

	spin_lock_irqsave(&ch_ctx->lock, flags);
	state = dlp_ctrl_get_channel_state(ch_ctx->ch_id);
	if ((state != DLP_CH_STATE_OPENING) && (state != DLP_CH_STATE_OPENED) &&
			(ch_ctx->ch_id != DLP_CHANNEL_TRACE))
		spin_unlock_irqrestore(&ch_ctx->lock, flags);

	else {
		spin_unlock_irqrestore(&ch_ctx->lock, flags);
		/* Send the TX HSI msg */
		ret = hsi_async(tx_msg->cl, tx_msg);
		if (ret)
			pr_err(DRVNAME ": TX xfer failed ! (cmd:0x%X, ret:%d)\n",
				dlp_cmd->params.id, ret);

		else {
			/* Dump the TX command */
			if (EDLP_CTRL_TX_DATA_REPORT)
				pr_debug(DRVNAME ": CTRL_TX (0x%X)\n",
						*((u32 *)&dlp_cmd->params));
			return 0;
		}
	}

	/* Free the TX msg */
	dlp_pdu_free(tx_msg, tx_msg->channel);

	/* Delete the command */
	kfree(dlp_cmd);

	return 0;
}

/**
 * Synchronous TX message callback
 *
 * @msg: a reference to the HSI msg
 *
 * This function:
 *		- Set the xfer status
 *		- wakeup the caller
 *		- delete the hsi message
 */
static void dlp_ctrl_complete_tx(struct hsi_msg *msg)
{
	struct dlp_command *dlp_cmd = msg->context;
	struct dlp_channel *ch_ctx = dlp_cmd->channel;

	dlp_cmd->status = (msg->status == HSI_STATUS_COMPLETED) ? 0 : -EIO;

	/* Command done, notify the sender */
	complete(&ch_ctx->tx.cmd_xfer_done);

	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);
}

/**
 * Asynchronous TX message callback
 *
 * @msg: a reference to the HSI msg
 *
 * This function:
 *		- delete the hsi message
 */
static void dlp_ctrl_complete_tx_async(struct hsi_msg *msg)
{
	struct dlp_command *dlp_cmd = msg->context;

	/* Delete the received msg */
	dlp_pdu_free(msg, msg->channel);

	/* Delete the command */
	kfree(dlp_cmd);
}

/*
 *
 */
static void dlp_ctrl_complete_rx(struct hsi_msg *msg)
{
	struct dlp_channel *ch_ctx;
	struct dlp_ctrl_context *ctrl_ctx;
	struct dlp_command *dlp_cmd = msg->context;
	struct dlp_command_params params, tx_params;
	unsigned long flags;
	int hsi_channel, elp_channel, ret, response, msg_complete, state;

	params.channel = 0; /* To please KlocWork */

	/* Check the link readiness (TTY still opened) */
	if (!dlp_tty_is_link_valid()) {
		if (EDLP_CTRL_RX_DATA_REPORT)
			pr_debug(DRVNAME ": CTRL: CH%d RX PDU ignored (close:%d, Time out: %d)\n",
				params.channel,
				dlp_drv.tty_closed, dlp_drv.tx_timeout);
		/* Delete the received msg */
		dlp_pdu_free(msg, msg->channel);

		/* Delete the command */
		dlp_ctrl_cmd_free(dlp_cmd);
		return;
	}

	/* Copy the reponse */
	memcpy(&params,
			sg_virt(msg->sgt.sgl),
			sizeof(struct dlp_command_params));

	ctrl_ctx = DLP_CTRL_CTX;
	response = -1;
	memcpy(&tx_params, &params, sizeof(struct dlp_command_params));
	msg_complete = (msg->status == HSI_STATUS_COMPLETED);

	/* Dump the RX command */
	if (EDLP_CTRL_RX_DATA_REPORT)
		pr_debug(DRVNAME ": CTRL_RX (0x%X)\n", *((u32 *)&params));

	hsi_channel = params.channel;
	if ((hsi_channel < 0) || (hsi_channel >= DLP_CHANNEL_COUNT)) {
		pr_err(DRVNAME ": Invalid HSI channel id (%d)\n", hsi_channel);
		goto push_rx;
	}

	elp_channel = dlp_drv.channels_hsi[hsi_channel].edlp_channel;
	ch_ctx = DLP_CHANNEL_CTX(elp_channel);
	switch (params.id) {
	case DLP_CMD_NOP:
		break;

	case DLP_CMD_CREDITS:
		if (!msg_complete)
			break;

		/* Increase the CREDITS counter */
		spin_lock_irqsave(&ch_ctx->lock, flags);
		ch_ctx->credits += params.data3;
		ret = ch_ctx->credits;
		spin_unlock_irqrestore(&ch_ctx->lock, flags);

		/* Credits available ==> Notify the channel */
		if (ch_ctx->credits_available_cb)
			ch_ctx->credits_available_cb(ch_ctx);

		/* CREDITS => ACK */
		response = DLP_CMD_ACK;

		/* Set the response params */
		tx_params.data1 = params.data3;
		tx_params.data2 = 0;
		tx_params.data3 = 0;
		break;

	case DLP_CMD_OPEN_CONN:
		if (!msg_complete)
			break;

		/* Ready ? if not, just save the OPEN_CONN request params */
		spin_lock_irqsave(&ctrl_ctx->open_lock, flags);
		state = dlp_ctrl_get_channel_state(hsi_channel);
		if ((state != DLP_CH_STATE_OPENING) && (state != DLP_CH_STATE_OPENED)) {
			pr_debug(DRVNAME ": HSI CH%d OPEN_CONN received (postponed) when state is %d\n",
					params.channel, state);
			/* Only CHANEL_TRACE can support this */
			if (ch_ctx->ch_id == DLP_CHANNEL_TRACE) {
				struct dlp_hsi_channel *hsi_ch;

				hsi_ch = &dlp_drv.channels_hsi[hsi_channel];
				memcpy(&hsi_ch->open_conn, &params, sizeof(params));
				spin_unlock_irqrestore(&ctrl_ctx->open_lock, flags);

				response = -1;
				goto push_rx;
			} else {
				/* OPEN_CONN => NACK (Unexpected open when closed) */
				spin_unlock_irqrestore(&ctrl_ctx->open_lock, flags);

				pr_debug(DRVNAME ": Not allowed for this channel => answer NAK\n");
				response = DLP_CMD_NACK;
				/* Set the response params */
				tx_params.data1 = params.id;
				tx_params.data2 = 0;
				struct dlp_command_params *params_pt = &params;
				tx_params.data3 = CMD_ID_ERR(params_pt, EDLP_ERR_CH_ALREADY_CLOSED);
				break;
			}
		}
		spin_unlock_irqrestore(&ctrl_ctx->open_lock, flags);

		pr_debug(DRVNAME ": HSI CH%d OPEN_CONN received (size: %d)\n",
					params.channel,
					(params.data2 << 8) | params.data1);

		/* Check the PDU size */
		response = dlp_ctrl_check_pdu_size(ch_ctx, &params, &tx_params);
		break;

	case DLP_CMD_CLOSE_CONN:
		/* CLOSE_CONN => ACK */
		response = DLP_CMD_ACK;

		/* Set the response params */
		tx_params.data1 = params.data3;
		tx_params.data2 = 0;
		tx_params.data3 = CMD_ID(params.data3, params.id);
		pr_debug(DRVNAME ": ch%d close_conn received\n",
				params.channel);
		break;

	default:
		/* Save the modem response */
		ctrl_ctx->response.status = (msg_complete ? 0 : -EIO);

		memcpy(&ctrl_ctx->response.params,
		       &params, sizeof(struct dlp_command_params));

		/* Command done, notify the sender */
		complete(&ch_ctx->rx.cmd_xfer_done);
		break;
	}

	/* Any response to send ? */
	if (response == -1)
		goto push_rx;

	dlp_ctrl_send_response(ch_ctx, &tx_params, response);

push_rx:
	/* Push the RX msg again for futur response */
	ret = hsi_async(msg->cl, msg);
	if (ret) {
		pr_err(DRVNAME ": RX push failed, ret:%d\n", ret);

		/* We have A BIG PROBLEM if the RX msg cant be  */
		/* pushed again in the controller ==>           */
		/* No response could be received (FIFO empty)   */

		/* Delete the received msg */
		dlp_pdu_free(msg, msg->channel);

		/* Delete the command */
		dlp_ctrl_cmd_free(dlp_cmd);
	}
}

/**
 * Send an HSI msg AND wait for the status
 *
 * @ch_ctx: a reference to the channel context to use
 * @id: the DLP command id
 * @response_id: the expected response id
 * @interm_state: the intermidiate state (to be set when TX is OK)
 * @final_state: the final state (to be set when RX (response) is OK)
 * @param1: the DLP command params
 * @param2: the DLP command params
 * @param3: the DLP command params
 *
 * This function blocks the caller until a response is received
 * or the timeout expires
 */
static int dlp_ctrl_cmd_send(struct dlp_channel *ch_ctx,
				unsigned char id,
				unsigned char response_id,
				unsigned char interm_state,
				unsigned char final_state,
				unsigned char param1,
				unsigned char param2, unsigned char param3)
{
	int ret = 0, old_state;
	struct dlp_ctrl_context *ctrl_ctx;
	struct dlp_command *dlp_cmd;
	struct dlp_command_params expected_resp;
	struct hsi_msg *tx_msg = NULL;

	/* Check the link readiness (TTY still opened) */
	if (!dlp_tty_is_link_valid()) {
		if (EDLP_CTRL_TX_DATA_REPORT)
			pr_debug(DRVNAME ": CH%d (HSI CH%d) cmd 0x%X ignored (close:%d, Time out: %d)\n",
					ch_ctx->ch_id, ch_ctx->hsi_channel,
					id, dlp_drv.tty_closed,
					dlp_drv.tx_timeout);

		return ret;
	}

	ctrl_ctx = DLP_CTRL_CTX;

	/* Save the current channel state */
	old_state = dlp_ctrl_get_channel_state(ch_ctx->hsi_channel);

	/* Backup RX callback */
	dlp_save_rx_callbacks(&ctrl_ctx->ehandler);

	/* Allocate the DLP command */
	dlp_cmd = dlp_ctrl_cmd_alloc(ch_ctx, id, param1, param2, param3);
	if (!dlp_cmd) {
		pr_err(DRVNAME ": Out of memory (dlp_cmd: 0x%X)\n", id);
		ret = -ENOMEM;
		goto out;
	}

	/* Allocate a new TX msg */
	tx_msg = dlp_pdu_alloc(DLP_CHANNEL_CTRL,
			       HSI_MSG_WRITE,
			       DLP_CTRL_TX_PDU_SIZE,
			       1,
			       dlp_cmd,
			       dlp_ctrl_complete_tx, dlp_ctrl_msg_destruct);

	if (!tx_msg) {
		pr_err(DRVNAME ": dlp_pdu_alloc(TX) failed\n");
		ret = -ENOMEM;
		goto free_cmd;
	}

	/* Copy the command data */
	memcpy(sg_virt(tx_msg->sgt.sgl),
	       &dlp_cmd->params, sizeof(struct dlp_command_params));

	/* Send the TX HSI msg */
	ret = hsi_async(tx_msg->cl, tx_msg);
	if (ret) {
		pr_err(DRVNAME ": Unable to send 0x%X cmd (ret:%d)\n",
			dlp_cmd->params.id, ret);

		/* Free the TX msg */
		dlp_pdu_free(tx_msg, tx_msg->channel);

		goto free_cmd;
	}

	/* Dump the TX command */
	if (EDLP_CTRL_TX_DATA_REPORT)
		pr_debug(DRVNAME ": CTRL_TX (0x%X)\n",
				*((u32 *)&dlp_cmd->params));

	/* Wait for TX msg to be sent */
	ret = wait_for_completion_timeout(&ch_ctx->tx.cmd_xfer_done,
					  msecs_to_jiffies(DLP_CMD_TX_TIMOUT));
	if (ret == 0) {
		pr_err(DRVNAME ": hsi_ch:%d, cmd:0x%X => TX timeout\n",
			dlp_cmd->params.channel, dlp_cmd->params.id);

		/* No need to call the complete sending call back,
		 * because of failure */
		tx_msg->complete = NULL;
		ret = -EIO;
		/* free only the cmd, because
		 * the message is already in the controller fifo.
		 * It will be freed when the controller fifo will be
		 * flushed/freed
		 */
		goto free_cmd;
	}

	/* TX msg sent, check the status */
	if (dlp_cmd->status) {
		pr_err(DRVNAME ": Failed to send cmd:0x%X\n",
				dlp_cmd->params.id);

		ret = -EIO;
		/* free only the command because
		 * the message has been already freed by the complete_tx
		 * callback
		 */
		goto free_cmd;
	}

	/* TX OK */
	/* 1. Set the intermidiate channel state */
	if (interm_state != DLP_CH_STATE_NONE)
		dlp_ctrl_set_channel_state(ch_ctx->hsi_channel, interm_state);

	/* Wait for response ? */
	if (response_id == DLP_CMD_NONE) {
		ret = 0;
		goto no_resp;
	}

	/* 2. Wait for the response */
	ret = wait_for_completion_timeout(&ch_ctx->rx.cmd_xfer_done,
					  msecs_to_jiffies(DLP_CMD_RX_TIMOUT));
	if (ret == 0) {
		pr_err(DRVNAME ": hsi_ch:%d, cmd:0x%X => RX timeout\n",
			dlp_cmd->params.channel, dlp_cmd->params.id);

		ret = -EIO;
		goto free_cmd;
	}

	/* Set the expected response params */
	expected_resp.id = response_id;
	expected_resp.channel = ch_ctx->hsi_channel;

	switch (id) {
	case DLP_CMD_CLOSE_CONN:
		expected_resp.data1 = param3;
		expected_resp.data2 = param2;
		expected_resp.data3 = CMD_ID(param1, id);
		break;

	case DLP_CMD_OPEN_CONN:
	default:
		expected_resp.data1 = param1;
		expected_resp.data2 = param2;
		expected_resp.data3 = CMD_ID(param3, id);
	}

	/* Check the received response params */
	ret = 0;
	if (memcmp(&ctrl_ctx->response.params,
				&expected_resp,
				sizeof(expected_resp))) {
		pr_err(DRVNAME": cmd 0x%X unexpected response 0x%X (expected 0x%X)",
			id,
			(unsigned int)(*(u32 *)&ctrl_ctx->response.params),
			(unsigned int)(*(u32 *)(&expected_resp)));

		tx_msg->context = NULL;
		ret = -EIO;
		goto free_cmd;
	}

no_resp:
	/* Response received & OK => set the new channel state */
	if (final_state != DLP_CH_STATE_NONE)
		dlp_ctrl_set_channel_state(ch_ctx->hsi_channel, final_state);

free_cmd:
	/* Free the DLP command */
	dlp_ctrl_cmd_free(dlp_cmd);

out:
	/* Restore RX callback */
	dlp_restore_rx_callbacks(&ctrl_ctx->ehandler);

	/* Restore the channel state in error case */
	if (ret)
		dlp_ctrl_set_channel_state(ch_ctx->hsi_channel, old_state);

	return ret;
}

/**
 * Push RX pdu for any modem command
 *
 */
static int dlp_ctrl_push_rx_pdu(struct dlp_channel *ch_ctx)
{
	int ret;
	struct hsi_msg *rx_msg;
	struct dlp_command *dlp_cmd;

	/* Allocate the DLP command */
	dlp_cmd = dlp_ctrl_cmd_alloc(ch_ctx, DLP_CMD_NOP, 0, 0, 0);
	if (!dlp_cmd) {
		pr_err(DRVNAME ": Out of memory (rx_pdu)\n");
		ret = -ENOMEM;
		goto out;
	}

	/* Allocate a new RX msg */
	rx_msg = dlp_pdu_alloc(DLP_CHANNEL_CTRL,
			       HSI_MSG_READ,
			       DLP_CTRL_RX_PDU_SIZE,
			       1,
			       dlp_cmd,
			       dlp_ctrl_complete_rx, dlp_ctrl_msg_destruct);

	if (!rx_msg) {
		pr_err(DRVNAME ": dlp_pdu_alloc() failed\n");
		ret = -ENOMEM;
		goto free_cmd;
	}

	/* Send the RX HSI msg */
	ret = hsi_async(rx_msg->cl, rx_msg);
	if (ret) {
		pr_err(DRVNAME ": RX push failed, ret:%d\n", ret);
		ret = -EIO;
		goto free_msg;
	}

	return 0;

free_msg:
	/* Free the msg */
	dlp_pdu_free(rx_msg, rx_msg->channel);

free_cmd:
	/* Delete the command */
	kfree(dlp_cmd);

out:
	return ret;
}


/**
*  Push RX PDUs in the controller FIFO for modem requests
*
 * @ch_ctx: a reference to related channel context
*
* @return 0 when OK, an error otherwise
*/
static int dlp_ctrl_push_rx_pdus(struct dlp_channel *ch_ctx)
{
	int i, ret = 0;

	/* Push RX pdus for CREDTIS/OPEN_CONN/NOP commands */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++)
		dlp_ctrl_push_rx_pdu(ch_ctx);

	return ret;
}

/****************************************************************************
 *
 * Exported functions
 *
 ***************************************************************************/

/*
* @brief
*
* @param ch_id
* @param hsi_channel
* @param dev
*
* @return
*/
static int dlp_ctrl_ctx_cleanup(struct dlp_channel *ch_ctx);

struct dlp_channel *dlp_ctrl_ctx_create(unsigned int ch_id,
		unsigned int hsi_channel,
		struct device *dev)
{
	struct hsi_client *client = to_hsi_client(dev);
	struct dlp_channel *ch_ctx;
	struct dlp_ctrl_context *ctrl_ctx;

	ch_ctx = kzalloc(sizeof(struct dlp_channel), GFP_KERNEL);
	if (!ch_ctx) {
		pr_err(DRVNAME ": Out of memory (ctrl_ch_ctx)\n");
		return NULL;
	}

	/* Allocate the context private data */
	ctrl_ctx = kzalloc(sizeof(struct dlp_ctrl_context), GFP_KERNEL);
	if (!ctrl_ctx) {
		pr_err(DRVNAME ": Out of memory (ctrl_ctx)\n");
		goto free_ch;
	}

	/* Create a workqueue to to manage:
	 *	- Modem readiness
	 *	- HSI TX timeout
	 */
	ctrl_ctx->wq = create_singlethread_workqueue(DRVNAME "-ctrl_wq");
	if (!ctrl_ctx->wq) {
		pr_err(DRVNAME ": Unable to create CTRL workqueue\n");
		goto free_ctx;
	}

	/* Save params */
	ch_ctx->ch_data = ctrl_ctx;
	ch_ctx->ch_id = ch_id;
	ch_ctx->hsi_channel = hsi_channel;
	ch_ctx->rx.config = client->rx_cfg;
	ch_ctx->tx.config = client->tx_cfg;

	spin_lock_init(&ch_ctx->lock);
	INIT_WORK(&ctrl_ctx->tx_timeout_work, dlp_ctrl_handle_tx_timeout);
	spin_lock_init(&ctrl_ctx->open_lock);

	/* Register PDUs push, cleanup CBs */
	ch_ctx->push_rx_pdus = dlp_ctrl_push_rx_pdus;
	ch_ctx->cleanup = dlp_ctrl_ctx_cleanup;

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_CTRL_TX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_WRITE);

	dlp_xfer_ctx_init(ch_ctx,
			  DLP_CTRL_RX_PDU_SIZE, 0, 0, 0, NULL, HSI_MSG_READ);

	/* Set ch_ctx, not yet done in the probe */
	DLP_CHANNEL_CTX(DLP_CHANNEL_CTRL) = ch_ctx;

	return ch_ctx;

free_ctx:
	kfree(ctrl_ctx);

free_ch:
	kfree(ch_ctx);
	return NULL;
}

/*
* @brief Delete any resources allocated by dlp_ctrl_ctx_create
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
static int dlp_ctrl_ctx_cleanup(struct dlp_channel *ch_ctx)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;
	int ret = 0;

	/* Delete the modem readiness/tx timeout worqueue */
	destroy_workqueue(ctrl_ctx->wq);

	return ret;
}

int dlp_ctrl_ctx_delete(struct dlp_channel *ch_ctx)
{
	struct dlp_ctrl_context *ctrl_ctx = ch_ctx->ch_data;

	/* Free the CTRL context */
	kfree(ctrl_ctx);

	/* Free the ch_ctx */
	kfree(ch_ctx);
	return 0;
}

/*
* @brief Open the specified channel for communication
*	- Send the OPEN_CONN command to the modem
*	- Return the operation status
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_open_channel(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	unsigned char param1 = PARAM1(ch_ctx->tx.pdu_size);
	unsigned char param2 = PARAM2(ch_ctx->tx.pdu_size);
	struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;

	/* Send the OPEN_CONN command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_OPEN_CONN, DLP_CMD_ACK,
				DLP_CH_STATE_OPENING, DLP_CH_STATE_NONE,
				param1, param2, 0);

	/* Channel correctly opened ? */
	if (ret == 0) {
		unsigned long flags;

		spin_lock_irqsave(&ctrl_ctx->open_lock, flags);
		dlp_ctrl_set_channel_state(ch_ctx->hsi_channel,
				DLP_CH_STATE_OPENED);
		spin_unlock_irqrestore(&ctrl_ctx->open_lock, flags);

		/* Check if we have any waiting OPEN_CONN */
		ret = dlp_ctrl_send_ack_nack(ch_ctx);
	}

	return ret;
}

/*
* @brief Close the specified channel
*	- Send the CLOSE_CONN command to the modem
*	- Return the operation status
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_close_channel(struct dlp_channel *ch_ctx)
{
	int state, ret = 0;
	unsigned char param3 = PARAM1(DLP_DIR_TRANSMIT_AND_RECEIVE);

	/* Reset the credits counter */
	ch_ctx->credits = 0;

	/* Reset the RX/TX seq_num */
	ch_ctx->rx.seq_num = 0;
	ch_ctx->tx.seq_num = 0;

	/* Check if the channel was correctly opened */
	state = dlp_ctrl_get_channel_state(ch_ctx->hsi_channel);
	if (state == DLP_CH_STATE_OPENED) {
		/* Send the command */
		ret = dlp_ctrl_cmd_send(ch_ctx,
				DLP_CMD_CLOSE_CONN, DLP_CMD_ACK,
				DLP_CH_STATE_CLOSING, DLP_CH_STATE_CLOSED,
				0, 0, param3);
	} else {
		pr_warn(DRVNAME ": Can't close CH%d (HSI CH%d) => invalid state: %d\n",
				ch_ctx->ch_id, ch_ctx->hsi_channel, state);
	}

	return ret;
}

/*
* @brief Send the NOP command
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_send_nop(struct dlp_channel *ch_ctx)
{
	int ret;
	unsigned char param1, param2, param3;

	param1 = PARAM1(DLP_NOP_CMD_CHECKSUM);
	param2 = PARAM2(DLP_NOP_CMD_CHECKSUM);
	param3 = PARAM3(DLP_NOP_CMD_CHECKSUM);

	/* Send the NOP command */
	ret = dlp_ctrl_cmd_send(ch_ctx,
			DLP_CMD_NOP, DLP_CMD_NONE,
			DLP_CH_STATE_NONE, DLP_CH_STATE_NONE,
			param1, param2, param3);

	return ret;
}

/*
* @brief Reply to any waiting OPEN_CONN command
*
* @param ch_ctx : The channel context to consider
*
* @return 0 when OK, error value otherwise
*/
int dlp_ctrl_send_ack_nack(struct dlp_channel *ch_ctx)
{
	int ret = 0;
	struct dlp_hsi_channel *hsi_ch;

	/* Get any saved OPEN_CONN params */
	hsi_ch = &dlp_drv.channels_hsi[ch_ctx->hsi_channel];

	/* Check if we have any waiting OPEN_CONN */
	if (hsi_ch->open_conn) {
		int response;
		struct dlp_command_params *params;
		struct dlp_command_params tx_params;

		/* Check the PDU size */
		params = (struct dlp_command_params *)&hsi_ch->open_conn;
		response = dlp_ctrl_check_pdu_size(ch_ctx, params, &tx_params);

		/* Send the response (ACK/NACK) */
		ret = dlp_ctrl_send_response(ch_ctx, &tx_params, response);
		if (ret)
			pr_err(DRVNAME ": ch%d will not opened\n",
					ch_ctx->hsi_channel);
		else {
			pr_debug(DRVNAME ": ch%d open_conn response (0x%X) sent\n",
						params->channel, response);

			/* Respnse sent => clear the saved command */
			hsi_ch->open_conn = 0;
		}
	}

	return ret;
}

/*
 * @brief Clean the stored open_conn command from channel context
 *
 * @param none
 *
 * @return none
 */
void dlp_ctrl_clean_stored_cmd(void)
{
	int i;
	struct dlp_hsi_channel *hsi_ch;

	/* Get any saved OPEN_CONN params */
	for (i = 0; i < DLP_CHANNEL_COUNT; i++) {
		hsi_ch = &dlp_drv.channels_hsi[i];
		hsi_ch->open_conn = 0;
	}
}

/*
* @brief Get the current channel state
*
* @param hsi_channel : The HSI channel ID to consider
*
* @return the current channel state
*/
inline unsigned char dlp_ctrl_get_channel_state(unsigned int hsi_channel)
{
	unsigned long flags;
	unsigned char state;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	state = dlp_drv.channels_hsi[hsi_channel].state;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);

	return state;
}

/*
* @brief Set the given channel state
*
* @param hsi_channel : The HSI channel ID to consider
* @param state : The new channel state to set
*
*/
inline void dlp_ctrl_set_channel_state(unsigned int hsi_channel,
				unsigned char state)
{
	unsigned long flags;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	dlp_drv.channels_hsi[hsi_channel].state = state;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);
}

/****************************************************************************
 *
 * Hangup/Reset management
 *
 ***************************************************************************/
/*
* dlp_ctrl_hangup_ctx_init - Initialises the given hangup context
*
* @param ch_ctx : Channel context to consider
*/
void dlp_ctrl_hangup_ctx_init(struct dlp_channel *ch_ctx,
		void (*timeout_func)(struct dlp_channel *ch_ctx))
{
	/* Init values */
	ch_ctx->modem_tx_timeout_cb = timeout_func;

	/* Register the timer CB (Use always the CTRL context) */
	init_timer(&dlp_drv.timer[ch_ctx->ch_id]);
	dlp_drv.timer[ch_ctx->ch_id].function = dlp_ctrl_hsi_tx_timout_cb;
	dlp_drv.timer[ch_ctx->ch_id].data = (unsigned long int)ch_ctx;
}

/**
 * dlp_ctrl_hangup_ctx_deinit - Clears a hangup context
 * @hangup_ctx: a reference to the considered hangup context
 */
void dlp_ctrl_hangup_ctx_deinit(struct dlp_channel *ch_ctx)
{
	unsigned long flags;
	int is_hunging_up;

	spin_lock_irqsave(&dlp_drv.lock, flags);
	is_hunging_up = dlp_drv.tx_timeout;
	spin_unlock_irqrestore(&dlp_drv.lock, flags);

	/* No need to wait for the end of the calling work! */
	if (!is_hunging_up) {
		struct dlp_ctrl_context *ctrl_ctx = DLP_CTRL_CTX;

		if (del_timer_sync(&dlp_drv.timer[ch_ctx->ch_id]))
			cancel_work_sync(&ctrl_ctx->tx_timeout_work);
		else
			flush_work(&ctrl_ctx->tx_timeout_work);
	}
}

