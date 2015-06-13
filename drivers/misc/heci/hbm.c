/*
 * HECI bus layer messages handling
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
 *
 */

#include <linux/export.h>
#include <linux/pci.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include "heci.h"
#include "heci_dev.h"
#include "hbm.h"

#ifdef dev_dbg
#undef dev_dbg
#endif
static  void no_dev_dbg(void *v, char *s, ...)
{
}
#define dev_dbg no_dev_dbg
/*#define dev_dbg dev_err*/

static int	ish_special_client;
struct heci_cl   *ish_heci_cl;
extern int	dma_ready;

void	set_ish_special_client(struct heci_cl *cl)
{
	ish_heci_cl = cl;
	ish_special_client = 1;
}
EXPORT_SYMBOL(set_ish_special_client);

void	unset_ish_special_client(void)
{
	ish_heci_cl = NULL;
	ish_special_client = 0;
}
EXPORT_SYMBOL(unset_ish_special_client);

/**
 * heci_hbm_me_cl_allocate - allocates storage for me clients
 *
 * @dev: the device structure
 *
	 * returns none.
 */
static void heci_hbm_me_cl_allocate(struct heci_device *dev)
{
	struct heci_me_client *clients;
	int b;

	/* count how many ME clients we have */
	for_each_set_bit(b, dev->me_clients_map, HECI_CLIENTS_MAX)
		dev->me_clients_num++;

	if (dev->me_clients_num <= 0)
		return;

	kfree(dev->me_clients);
	dev->me_clients = NULL;

	dev_dbg(&dev->pdev->dev, "memory allocation for ME clients size=%zd.\n",
		dev->me_clients_num * sizeof(struct heci_me_client));

	/* allocate storage for ME clients representation */
	clients = kcalloc(dev->me_clients_num, sizeof(struct heci_me_client), GFP_ATOMIC);
	if (!clients) {
		dev_err(&dev->pdev->dev, "memory allocation for ME clients failed.\n");
		dev->dev_state = HECI_DEV_RESETTING;
		heci_reset(dev, 1);
		return;
	}
	dev->me_clients = clients;
	return;
}

/**
 * heci_hbm_cl_hdr - construct client hbm header
 * @cl: - client
 * @hbm_cmd: host bus message command
 * @buf: buffer for cl header
 * @len: buffer length
 */
static inline
void heci_hbm_cl_hdr(struct heci_cl *cl, u8 hbm_cmd, void *buf, size_t len)
{
	struct heci_hbm_cl_cmd *cmd = buf;

	memset(cmd, 0, len);

	cmd->hbm_cmd = hbm_cmd;
	cmd->host_addr = cl->host_client_id;
	cmd->me_addr = cl->me_client_id;
}

/**
 * same_disconn_addr - tells if they have the same address
 *
 * @file: private data of the file object.
 * @disconn: disconnection request.
 *
 * returns true if addres are same
 */
static inline
bool heci_hbm_cl_addr_equal(struct heci_cl *cl, void *buf)
{
	struct heci_hbm_cl_cmd *cmd = buf;
	return cl->host_client_id == cmd->host_addr &&
		cl->me_client_id == cmd->me_addr;
}


/**
 * set_conn_state - checks if the message belongs
 * to the file private data.
 *
 * @cl: private data of the file object
 * @rs: connect response bus message
 *
 */
static bool set_conn_state(struct heci_cl *cl,
		struct hbm_client_connect_response *rs)
{
	if (heci_hbm_cl_addr_equal(cl, rs)) {
		if (!rs->status) {
			cl->state = HECI_FILE_CONNECTED;
			cl->status = 0;

		} else {
			cl->state = HECI_FILE_DISCONNECTED;
			cl->status = -ENODEV;
		}
		cl->timer_count = 0;

		return true;
	}
	return false;
}

int heci_hbm_start_wait(struct heci_device *dev)
{
	int ret;
	if (dev->hbm_state > HECI_HBM_START)
		return 0;

	mutex_unlock(&dev->device_lock);
	dev_err(&dev->pdev->dev, "Going to wait for heci start hbm_state=%08X\n", dev->hbm_state);
wait_again:
	ret = wait_event_interruptible_timeout(dev->wait_recvd_msg,
#if 0
			dev->hbm_state == HECI_HBM_IDLE ||
#endif
			dev->hbm_state >= HECI_HBM_STARTED,
#if 0
			heci_secs_to_jiffies(HECI_INTEROP_TIMEOUT));
#else
			(HECI_INTEROP_TIMEOUT * HZ));

	/* If interrupted by a signal, wait again */
	if (ret < 0)
		goto	wait_again;
#endif

	mutex_lock(&dev->device_lock);
	dev_err(&dev->pdev->dev, "Woke up from waiting for heci start ret=%d hbm_state=%08X\n", ret, dev->hbm_state);

	if (ret <= 0 && (dev->hbm_state <= HECI_HBM_START)) {
		dev->hbm_state = HECI_HBM_IDLE;
		dev_err(&dev->pdev->dev, "wating for heci start failed ret=%d hbm_state=%08X\n", ret, dev->hbm_state);
		return -ETIMEDOUT;
	}
	return 0;
}

/**
 * heci_hbm_start_req - sends start request message.
 *
 * @dev: the device structure
 */
int heci_hbm_start_req(struct heci_device *dev)
{
	struct heci_msg_hdr *heci_hdr = &dev->wr_msg.hdr;
	struct hbm_host_version_request *start_req;
	const size_t len = sizeof(struct hbm_host_version_request);

	heci_hbm_hdr(heci_hdr, len);

	/* host start message */
	start_req = (struct hbm_host_version_request *)dev->wr_msg.data;
	memset(start_req, 0, len);
	start_req->hbm_cmd = HOST_START_REQ_CMD;
	start_req->host_version.major_version = HBM_MAJOR_VERSION;
	start_req->host_version.minor_version = HBM_MINOR_VERSION;

	dev->hbm_state = HECI_HBM_IDLE;
	if (heci_write_message(dev, heci_hdr, dev->wr_msg.data)) {
		dev_err(&dev->pdev->dev, "version message writet failed\n");
		dev->dev_state = HECI_DEV_RESETTING;
		heci_reset(dev, 1);
		return -ENODEV;
	}
	dev->hbm_state = HECI_HBM_START;
	dev->init_clients_timer = HECI_CLIENTS_INIT_TIMEOUT;
	return 0;
}
EXPORT_SYMBOL(heci_hbm_start_req);

/*
 * heci_hbm_enum_clients_req - sends enumeration client request message.
 *
 * @dev: the device structure
 *
 * returns none.
 */
void heci_hbm_enum_clients_req(struct heci_device *dev)
{
	struct heci_msg_hdr *heci_hdr = &dev->wr_msg.hdr;
	struct hbm_host_enum_request *enum_req;
	const size_t len = sizeof(struct hbm_host_enum_request);
	/* enumerate clients */
	heci_hbm_hdr(heci_hdr, len);

	enum_req = (struct hbm_host_enum_request *)dev->wr_msg.data;
	memset(enum_req, 0, len);
	enum_req->hbm_cmd = HOST_ENUM_REQ_CMD;

	if (heci_write_message(dev, heci_hdr, dev->wr_msg.data)) {
		dev->dev_state = HECI_DEV_RESETTING;
		dev_err(&dev->pdev->dev, "enumeration request write failed.\n");
		heci_reset(dev, 1);
	}
	dev->hbm_state = HECI_HBM_ENUM_CLIENTS;
	dev->init_clients_timer = HECI_CLIENTS_INIT_TIMEOUT;
	return;
}

/**
 * heci_hbm_prop_requsest - request property for a single client
 *
 * @dev: the device structure
 *
 * returns none.
 */

static int heci_hbm_prop_req(struct heci_device *dev)
{

	struct heci_msg_hdr *heci_hdr = &dev->wr_msg.hdr;
	struct hbm_props_request *prop_req;
	const size_t len = sizeof(struct hbm_props_request);
	unsigned long next_client_index;
	u8 client_num;


	client_num = dev->me_client_presentation_num;

	next_client_index = find_next_bit(dev->me_clients_map, HECI_CLIENTS_MAX,
					  dev->me_client_index);

	/* We got all client properties */
	if (next_client_index == HECI_CLIENTS_MAX) {
		schedule_work(&dev->init_work);

		return 0;
	}

	dev->me_clients[client_num].client_id = next_client_index;
	dev->me_clients[client_num].heci_flow_ctrl_creds = 0;

	heci_hbm_hdr(heci_hdr, len);
	prop_req = (struct hbm_props_request *)dev->wr_msg.data;

	memset(prop_req, 0, sizeof(struct hbm_props_request));


	prop_req->hbm_cmd = HOST_CLIENT_PROPERTIES_REQ_CMD;
	prop_req->address = next_client_index;

	if (heci_write_message(dev, heci_hdr, dev->wr_msg.data)) {
		dev->dev_state = HECI_DEV_RESETTING;
		dev_err(&dev->pdev->dev, "properties request write failed\n");
		heci_reset(dev, 1);

		return -EIO;
	}

	dev->init_clients_timer = HECI_CLIENTS_INIT_TIMEOUT;
	dev->me_client_index = next_client_index;

	return 0;
}

/**
 * heci_hbm_stop_req_prepare - perpare stop request message
 *
 * @dev - heci device
 * @heci_hdr - heci message header
 * @data - hbm message body buffer
 */
static void heci_hbm_stop_req_prepare(struct heci_device *dev,
		struct heci_msg_hdr *heci_hdr, unsigned char *data)
{
	struct hbm_host_stop_request *req =
			(struct hbm_host_stop_request *)data;
	const size_t len = sizeof(struct hbm_host_stop_request);

	heci_hbm_hdr(heci_hdr, len);

	memset(req, 0, len);
	req->hbm_cmd = HOST_STOP_REQ_CMD;
	req->reason = DRIVER_STOP_REQUEST;
}

/**
 * heci_hbm_cl_flow_control_req - sends flow control requst.
 *
 * @dev: the device structure
 * @cl: client info
 *
 * This function returns -EIO on write failure
 */
int heci_hbm_cl_flow_control_req(struct heci_device *dev, struct heci_cl *cl)
{
	struct heci_msg_hdr *heci_hdr = &dev->wr_msg.hdr;
	const size_t len = sizeof(struct hbm_flow_control);

	heci_hbm_hdr(heci_hdr, len);
	heci_hbm_cl_hdr(cl, HECI_FLOW_CONTROL_CMD, dev->wr_msg.data, len);

	dev_dbg(&dev->pdev->dev, "sending flow control host client = %d, ME client = %d\n",
		cl->host_client_id, cl->me_client_id);

	return heci_write_message(dev, heci_hdr, dev->wr_msg.data);
}
EXPORT_SYMBOL(heci_hbm_cl_flow_control_req);

/**
 * add_single_flow_creds - adds single buffer credentials.
 *
 * @file: private data ot the file object.
 * @flow: flow control.
 */
static void heci_hbm_add_single_flow_creds(struct heci_device *dev,
				  struct hbm_flow_control *flow)
{
	struct heci_me_client *client;
	int i;

	for (i = 0; i < dev->me_clients_num; i++) {
		client = &dev->me_clients[i];
		if (client && flow->me_addr == client->client_id) {
			if (client->props.single_recv_buf) {
				client->heci_flow_ctrl_creds++;
				dev_dbg(&dev->pdev->dev, "recv flow ctrl msg ME %d (single).\n",
				    flow->me_addr);
				dev_dbg(&dev->pdev->dev, "flow control credentials =%d.\n",
				    client->heci_flow_ctrl_creds);
			} else {
				BUG();	/* error in flow control */
			}
		}
	}
}

/**
 * heci_hbm_cl_flow_control_res - flow control response from me
 *
 * @dev: the device structure
 * @flow_control: flow control response bus message
 */
static void heci_hbm_cl_flow_control_res(struct heci_device *dev,
		struct hbm_flow_control *flow_control)
{
	struct heci_cl *cl = NULL;
	struct heci_cl *next = NULL;

	/* normal connection */
	list_for_each_entry_safe(cl, next, &dev->file_list, link) {
		if (heci_hbm_cl_addr_equal(cl, flow_control)) {
			cl->heci_flow_ctrl_creds++;
			dev_dbg(&dev->pdev->dev, "flow ctrl msg for host %d ME %d.\n",
				flow_control->host_addr, flow_control->me_addr);
			dev_dbg(&dev->pdev->dev, "flow control credentials = %d.\n",
				    cl->heci_flow_ctrl_creds);
				break;
		}
	}
}


/**
 * heci_hbm_cl_disconnect_req - sends disconnect message to fw.
 *
 * @dev: the device structure
 * @cl: a client to disconnect from
 *
 * This function returns -EIO on write failure
 */
int heci_hbm_cl_disconnect_req(struct heci_device *dev, struct heci_cl *cl)
{
	struct heci_msg_hdr *heci_hdr = &dev->wr_msg.hdr;
	const size_t len = sizeof(struct hbm_client_connect_request);

	heci_hbm_hdr(heci_hdr, len);
	heci_hbm_cl_hdr(cl, CLIENT_DISCONNECT_REQ_CMD, dev->wr_msg.data, len);

	return heci_write_message(dev, heci_hdr, dev->wr_msg.data);
}

/**
 * heci_hbm_cl_disconnect_res - disconnect response from ME
 *
 * @dev: the device structure
 * @rs: disconnect response bus message
 */
static void heci_hbm_cl_disconnect_res(struct heci_device *dev,
		struct hbm_client_connect_response *rs)
{
	struct heci_cl *cl;
	struct heci_cl_cb *pos = NULL, *next = NULL;

	dev_dbg(&dev->pdev->dev,
			"disconnect_response:\n"
			"ME Client = %d\n"
			"Host Client = %d\n"
			"Status = %d\n",
			rs->me_addr,
			rs->host_addr,
			rs->status);

	list_for_each_entry_safe(pos, next, &dev->ctrl_rd_list.list, list) {
		cl = pos->cl;

		if (!cl) {
			list_del(&pos->list);
			return;
		}

		dev_dbg(&dev->pdev->dev, "list_for_each_entry_safe in ctrl_rd_list.\n");
		if (heci_hbm_cl_addr_equal(cl, rs)) {
			list_del(&pos->list);
			if (!rs->status)
				cl->state = HECI_FILE_DISCONNECTED;

			cl->status = 0;
			cl->timer_count = 0;
			break;
		}
	}
}

/**
 * heci_hbm_cl_connect_req - send connection request to specific me client
 *
 * @dev: the device structure
 * @cl: a client to connect to
 *
 * returns -EIO on write failure
 */
int heci_hbm_cl_connect_req(struct heci_device *dev, struct heci_cl *cl)
{
	struct heci_msg_hdr *heci_hdr = &dev->wr_msg.hdr;
	const size_t len = sizeof(struct hbm_client_connect_request);

	heci_hbm_hdr(heci_hdr, len);
	heci_hbm_cl_hdr(cl, CLIENT_CONNECT_REQ_CMD, dev->wr_msg.data, len);

	return heci_write_message(dev, heci_hdr,  dev->wr_msg.data);
}
EXPORT_SYMBOL(heci_hbm_cl_connect_req);

/**
 * heci_hbm_cl_connect_res - connect resposne from the ME
 *
 * @dev: the device structure
 * @rs: connect response bus message
 */
static void heci_hbm_cl_connect_res(struct heci_device *dev, struct hbm_client_connect_response *rs)
{

	struct heci_cl *cl;
	struct heci_cl_cb *pos = NULL, *next = NULL;

	dev_dbg(&dev->pdev->dev,
			"connect_response:\n"
			"ME Client = %d\n"
			"Host Client = %d\n"
			"Status = %d\n",
			rs->me_addr,
			rs->host_addr,
			rs->status);

	/* DD -- if ISH client, treat specially */
	if (ish_special_client) {
		dev_dbg(&dev->pdev->dev, "ish_special_client\n");
		if (set_conn_state(ish_heci_cl, rs)) {
			dev_dbg(&dev->pdev->dev, "Successfully connected to ISH client.\n");
			cl->timer_count = 0;
			return;
		} else {
			dev_err(&dev->pdev->dev, "Error connecting to ISH client.\n");
		}
	}

	list_for_each_entry_safe(pos, next, &dev->ctrl_rd_list.list, list) {
		cl = pos->cl;
		if (!cl) {
			list_del(&pos->list);
			return;
		}
		if (pos->fop_type == HECI_FOP_IOCTL) {
			if (set_conn_state(cl, rs)) {
				list_del(&pos->list);
				cl->status = 0;
				cl->timer_count = 0;
				break;
			}
		}
	}
}


/**
 * heci_client_disconnect_request - disconnect request initiated by me
 *  host sends disoconnect response
 *
 * @dev: the device structure.
 * @disconnect_req: disconnect request bus message from the me
 */
static void heci_hbm_fw_disconnect_req(struct heci_device *dev,
		struct hbm_client_connect_request *disconnect_req)
{
	struct heci_cl *cl, *next;
	const size_t len = sizeof(struct hbm_client_connect_response);

	list_for_each_entry_safe(cl, next, &dev->file_list, link) {
		if (heci_hbm_cl_addr_equal(cl, disconnect_req)) {
			dev_dbg(&dev->pdev->dev, "disconnect request host client %d ME client %d.\n",
					disconnect_req->host_addr,
					disconnect_req->me_addr);
			cl->state = HECI_FILE_DISCONNECTED;
			cl->timer_count = 0;

			/* prepare disconnect response */
			heci_hbm_hdr(&dev->wr_ext_msg.hdr, len);
			heci_hbm_cl_hdr(cl, CLIENT_DISCONNECT_RES_CMD, dev->wr_ext_msg.data, len);
			break;
		}
	}
}


/**
 * heci_hbm_dispatch - bottom half read routine after ISR to
 * handle the read bus message cmd processing.
 *
 * @dev: the device structure
 * @heci_hdr: header of bus message
 */
void heci_hbm_dispatch(struct heci_device *dev, struct heci_msg_hdr *hdr)
{
	struct heci_bus_message *heci_msg;
	struct heci_me_client *me_client;
	struct hbm_host_version_response *version_res;
	struct hbm_client_connect_response *connect_res;
	struct hbm_client_connect_response *disconnect_res;
	struct hbm_client_connect_request *disconnect_req;
	struct hbm_flow_control *flow_control;
	struct hbm_props_response *props_res;
	struct hbm_host_enum_response *enum_res;

	/* read the message to our buffer */
	BUG_ON(hdr->length >= sizeof(dev->rd_msg_buf));
	heci_read_slots(dev, dev->rd_msg_buf, hdr->length);
	heci_msg = (struct heci_bus_message *)dev->rd_msg_buf;
	dev_dbg(&dev->pdev->dev, "bus cmd = %lu\n", heci_msg->hbm_cmd);

	switch (heci_msg->hbm_cmd) {
	case HOST_START_RES_CMD:
		version_res = (struct hbm_host_version_response *)heci_msg;
		if (!version_res->host_version_supported) {
			dev->version = version_res->me_max_version;
			dev_dbg(&dev->pdev->dev, "version mismatch.\n");

			dev->hbm_state = HECI_HBM_STOPPED;
			heci_hbm_stop_req_prepare(dev, &dev->wr_msg.hdr,
						dev->wr_msg.data);
			heci_write_message(dev, &dev->wr_msg.hdr,
					dev->wr_msg.data);
			return;
		}

		dev->version.major_version = HBM_MAJOR_VERSION;
		dev->version.minor_version = HBM_MINOR_VERSION;
		if (dev->dev_state == HECI_DEV_INIT_CLIENTS &&
		    dev->hbm_state == HECI_HBM_START) {
			dev->hbm_state = HECI_HBM_STARTED;
			dev->init_clients_timer = 0;
			heci_hbm_enum_clients_req(dev);
		} else {
			dev_err(&dev->pdev->dev, "reset: wrong host start response\n");
			heci_reset(dev, 1);
			return;
		}

		wake_up_interruptible(&dev->wait_recvd_msg);
		dev_dbg(&dev->pdev->dev, "host start response message received.\n");
		break;

	case CLIENT_CONNECT_RES_CMD:
		connect_res = (struct hbm_client_connect_response *) heci_msg;
		heci_hbm_cl_connect_res(dev, connect_res);
		dev_dbg(&dev->pdev->dev, "client connect response message received.\n");
		wake_up(&dev->wait_recvd_msg);
		break;

	case CLIENT_DISCONNECT_RES_CMD:
		disconnect_res = (struct hbm_client_connect_response *) heci_msg;
		heci_hbm_cl_disconnect_res(dev, disconnect_res);
		dev_dbg(&dev->pdev->dev, "client disconnect response message received.\n");
		wake_up(&dev->wait_recvd_msg);
		break;

	case HECI_FLOW_CONTROL_CMD:
		flow_control = (struct hbm_flow_control *) heci_msg;
		heci_hbm_cl_flow_control_res(dev, flow_control);
		dev_dbg(&dev->pdev->dev, "client flow control response message received.\n");
		break;

	case HOST_CLIENT_PROPERTIES_RES_CMD:
		props_res = (struct hbm_props_response *)heci_msg;
		me_client = &dev->me_clients[dev->me_client_presentation_num];

		if (props_res->status || !dev->me_clients) {
			dev_err(&dev->pdev->dev, "reset: properties response hbm wrong status.\n");
			heci_reset(dev, 1);
			return;
		}

		if (me_client->client_id != props_res->address) {
			dev_err(&dev->pdev->dev, "reset: host properties response address mismatch\n");
			heci_reset(dev, 1);
			return;
		}

		if (dev->dev_state != HECI_DEV_INIT_CLIENTS ||
		    dev->hbm_state != HECI_HBM_CLIENT_PROPERTIES) {
			dev_err(&dev->pdev->dev, "reset: unexpected properties response\n");
			heci_reset(dev, 1);

			return;
		}

		me_client->props = props_res->client_properties;
		dev->me_client_index++;
		dev->me_client_presentation_num++;

		/* Add new client device */
		heci_bus_new_client(dev);

		/* request property for the next client */
		heci_hbm_prop_req(dev);

		break;

	case HOST_ENUM_RES_CMD:
		enum_res = (struct hbm_host_enum_response *) heci_msg;
		memcpy(dev->me_clients_map, enum_res->valid_addresses, 32);
		if (dev->dev_state == HECI_DEV_INIT_CLIENTS &&
		    dev->hbm_state == HECI_HBM_ENUM_CLIENTS) {
				dev->init_clients_timer = 0;
				dev->me_client_presentation_num = 0;
				dev->me_client_index = 0;
				heci_hbm_me_cl_allocate(dev);
				dev->hbm_state = HECI_HBM_CLIENT_PROPERTIES;

				/* first property reqeust */
				heci_hbm_prop_req(dev);
		} else {
			dev_err(&dev->pdev->dev, "reset: unexpected enumeration response hbm.\n");
			heci_reset(dev, 1);
			return;
		}
		break;

	case HOST_STOP_RES_CMD:
		if (dev->hbm_state != HECI_HBM_STOPPED)
			dev_err(&dev->pdev->dev, "unexpected stop response.\n");

		dev->dev_state = HECI_DEV_DISABLED;
		dev_info(&dev->pdev->dev, "reset: FW stop response.\n");
		heci_reset(dev, 1);
		break;

	case CLIENT_DISCONNECT_REQ_CMD:
		/* search for client */
		disconnect_req = (struct hbm_client_connect_request *)heci_msg;
		heci_hbm_fw_disconnect_req(dev, disconnect_req);
		break;

	case ME_STOP_REQ_CMD:

		dev->hbm_state = HECI_HBM_STOPPED;
		heci_hbm_stop_req_prepare(dev, &dev->wr_ext_msg.hdr, dev->wr_ext_msg.data);
		break;

	case CLIENT_DMA_RES_CMD:
		/* TODO: wake up anybody who could be waiting for DMA completion */
		dma_ready = 1;
		if (waitqueue_active(&dev->wait_dma_ready))
			wake_up(&dev->wait_dma_ready);
		break;

	default:
		BUG();
		break;

	}
}
