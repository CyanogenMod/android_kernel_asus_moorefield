
/* hsi_logical.c
*
* Copyright (C) 2011 Renesas. All rights reserved.
*
*
* Implementation of the HSI logical using RENESAS HSI protocol.
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

#include <linux/atomic.h>
#include <linux/clk.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/if_ether.h>
#include <linux/if_arp.h>
#include <linux/if_phonet.h>
#include <linux/init.h>
#include <linux/irq.h>
#include <linux/list.h>
#include <linux/module.h>
#include <linux/netdevice.h>
#include <linux/notifier.h>
#include <linux/scatterlist.h>
#include <linux/skbuff.h>
#include <linux/slab.h>
#include <linux/spinlock.h>
#include <linux/timer.h>
#include <linux/hsi/hsi_logical.h>
#include <linux/hsi/hsi.h>
#include <linux/wakelock.h>
#include <linux/gpio.h>

struct wake_lock wake_resoft;
struct wake_lock wake_no_suspend_l2header;
struct wake_lock wake_temp;

/* Interface with upper layer (header in hsi_logical.h)*/
/*int hsi_logical_open(struct hsi_protocol * hsi_protocol_ctx);*/
/*int hsi_logical_close(struct hsi_protocol * hsi_protocol_ctx);*/
/*int hsi_logical_send(struct hsi_protocol * hsi_protocol_ctx,
			struct sk_buff *skb, u16 client_id);*/

/* Configurations exchange */
static void hsi_logical_check_if_boot_done(
	struct hsi_protocol *hsi_protocol_context);

static void hsi_logical_boot_info_req_complete(
	struct hsi_msg *msg);

static void hsi_logical_boot_info_resp_complete(
	struct hsi_msg *msg);

/* TX */
static int hsi_logical_alloc_msg_and_send_on_control_channel(
	struct hsi_protocol_client *hsi_client_context,
	u32 msg,
	unsigned char channel_id,
	void (*complete)(struct hsi_msg *msg));

static int hsi_logical_send_on_control_channel(
	struct hsi_msg *msg,
	struct hsi_protocol_client *hsi_data_client);

static void hsi_logical_tx_control_complete(
	struct hsi_msg *msg);

static int hsi_ack_received(
	struct hsi_protocol_client *hsi_client_context,
	unsigned char channel_id);

static int hsi_nack_received(
	struct hsi_protocol_client *hsi_client_context,
	unsigned char channel_id);

static void hsi_logical_tx_data_complete(struct hsi_msg *msg);

/* RX */
static void hsi_logical_wait_rx_control(
	struct hsi_client *cl,
	unsigned char channel_id);
static void hsi_logical_wait_rx_control_l2h(
	struct hsi_client *cl,
	unsigned char channel_id);
static void hsi_logical_wait_rx_control_wrong_channel(
	struct hsi_client *cl,
	unsigned char channel_id);
static void hsi_logical_rx_control_complete(
	struct hsi_msg *msg);
static void hsi_rx_data_ack(
	struct hsi_protocol *hsi_protocol_context,
	struct hsi_protocol_client *hsi_data_client);
static void hsi_rx_mem_test(
	unsigned long in);
static int hsi_l2header_receive(
	struct hsi_protocol_client *hsi_control_client,
	unsigned char channel_id, u32 l2header);
static void hsi_logical_rx_data_complete(
	struct hsi_msg *msg);
static void hsi_logical_rx_control_complete_step2(
	struct hsi_msg *msg);

/* Destructor callbacks */
static void hsi_logical_data_free_msg_and_skb(struct hsi_msg *msg);
static void hsi_logical_data_free_msg(struct hsi_msg *msg);
static void hsi_logical_control_destructor(struct hsi_msg *msg);
static void hsi_logical_control_destructor_l2h(struct hsi_msg *msg);
static void hsi_logical_tx_control_destructor(struct hsi_msg *msg);
static void hsi_logical_rx_control_destructor(struct hsi_msg *msg);
static void hsi_logical_rx_control_destructor_l2h(struct hsi_msg *msg);

/* hsi_msg allocation */
static struct hsi_msg *hsi_logical_alloc_data(
	struct sk_buff *skb,
	gfp_t flags);
static struct hsi_msg *hsi_logical_get_control(
	struct hsi_protocol_client *hsi_client_context);
static struct hsi_msg *hsi_logical_get_control_l2h(
	struct hsi_protocol_client *hsi_client_context);
static struct hsi_msg *hsi_logical_alloc_tx_control(
	struct hsi_protocol_client *hsi_client_context,
	void (*complete)(struct hsi_msg *msg));

/* Libraries */
#ifdef HSI_LOGICAL_USE_DEBUG
static int hsi_logical_get_client_id(
	unsigned char mapid);
#endif
static int hsi_logical_retrieve_data_client(
	struct hsi_protocol_client *hsi_client_context,
	unsigned char channel_id,
	struct hsi_protocol_client **hsi_data_client);
static void hsi_logical_skb_to_msg(
	struct sk_buff *skb,
	struct hsi_msg *msg);

/*power*/
static void hsi_logical_inactivity_timer(unsigned long data);
void hsi_logical_start_tx(struct hsi_protocol *hsi_protocol_ctx);
void hsi_logical_stop_tx(struct hsi_protocol *hsi_protocol_ctx);
void hsi_logical_send_power_off(struct hsi_protocol *hsi_protocol_context);

/* Traces */
extern int hsi_logical_trace_state;
u32 debug_nb_crash_modem;
u32 init_spinlock;

#ifdef HSI_LOGICAL_USE_DEBUG
#define DPRINTK(...)		{if (hsi_logical_trace_state == HSI_ALL) \
					printk(KERN_DEBUG __VA_ARGS__); }

#define DPRINTK_HIGH(...)	{if ((hsi_logical_trace_state == HSI_HIGH)\
				|| (hsi_logical_trace_state == HSI_ALL) \
				|| (hsi_logical_trace_state == HSI_HIGH_RX) \
				|| (hsi_logical_trace_state == HSI_HIGH_TX)) \
					printk(KERN_DEBUG __VA_ARGS__); }

#define DPRINTK_HIGH_RX(...)	{if ((hsi_logical_trace_state == HSI_HIGH)\
				|| (hsi_logical_trace_state == HSI_ALL) \
				|| (hsi_logical_trace_state == HSI_HIGH_RX)) \
					printk(KERN_DEBUG __VA_ARGS__); }

#define DPRINTK_HIGH_TX(...)	{if ((hsi_logical_trace_state == HSI_HIGH)\
				|| (hsi_logical_trace_state == HSI_ALL) \
				|| (hsi_logical_trace_state == HSI_HIGH_TX)) \
					printk(KERN_DEBUG __VA_ARGS__); }
#else
# define DPRINTK(...)
#endif

#ifdef HSI_V3_USED
void  hsi_logical_do_power_sequence(
		struct hsi_protocol *hsi_protocol_context,
		int hsi_power_state);
#endif /*HSI_V3_USED*/

static void
hsi_logical_config_fail(unsigned long in)
{
	struct hsi_protocol *hsi_protocol_ctx = (struct hsi_protocol *)in;
	int pwr_on = hsi_protocol_ctx->cl[0]->gpio_pwr_on;
	wake_unlock(&wake_resoft);
	if (hsi_protocol_ctx->main_state == HSI_CONFIG_EXCHANGE_NO_DONE) {
		EPRINTK("hsi_logical state %d, tx_boot_info_req %d, " \
			"tx_boot_info_resp %d, rx_boot_info_resp %d, " \
				" modem_config %x\n",
			hsi_protocol_ctx->main_state,
			hsi_protocol_ctx->cfg_ex.tx_boot_info_req_complete,
			hsi_protocol_ctx->cfg_ex.tx_boot_info_resp_complete,
			hsi_protocol_ctx->cfg_ex.rx_boot_info_resp_complete,
			hsi_protocol_ctx->cfg_ex.received_modem_config);


		EPRINTK("Configuration exchange with modem fails. HSI link is broken\n");
		gpio_set_value(pwr_on, 0);
	}
}

int
hsi_logical_open(struct hsi_protocol *hsi_protocol_ctx)
{
	struct hsi_client *cl;
	int err = 0;
	int i;

	DPRINTK("%s - NB HSI core clients: %d\n",
		__func__,
		hsi_protocol_ctx->nb_client);

	if (init_spinlock == 1) {
		init_spinlock = 0;

		for (i = 0; i < hsi_protocol_ctx->nb_client; i++) {
			spin_lock_init(&hsi_protocol_ctx->cl[i]->tx_lock);
			spin_lock_init(&hsi_protocol_ctx->cl[i]->rx_lock);
		}
		spin_lock_init(&hsi_protocol_ctx->tx_activity_status_lock);
		spin_lock_init(&hsi_protocol_ctx->ctrl_msg_array_lock);
		spin_lock_init(&hsi_protocol_ctx->ctrl_msg_array_l2h_lock);
	}

	/* Alloc pool of control message */
	for (i = 0; i < HSI_LOGICAL_NB_CONTROL_MSG; i++) {
		u8 *buf;

		hsi_protocol_ctx->hsi_ctrl_msg_array[i].msg =
				hsi_alloc_msg(1, GFP_KERNEL | GFP_DMA);

		if (!hsi_protocol_ctx->hsi_ctrl_msg_array[i].msg)
			BUG();

		buf = kmalloc(HSI_LOGICAL_CONTROL_MESSAGE_SIZE,
				GFP_KERNEL | GFP_DMA);

		if (!buf)
			BUG();

		sg_init_one(
			hsi_protocol_ctx->hsi_ctrl_msg_array[i].msg->sgt.sgl,
			buf,
			HSI_LOGICAL_CONTROL_MESSAGE_SIZE);


		hsi_protocol_ctx->hsi_ctrl_msg_array[i].free = 1;
	}

	/* Alloc pool of control message l2h*/
	for (i = 0; i < HSI_LOGICAL_NB_CONTROL_MSG_L2H; i++) {
		u8 *buf;

		hsi_protocol_ctx->hsi_ctrl_msg_array_l2h[i].msg =
				hsi_alloc_msg(1, GFP_KERNEL | GFP_DMA);

		if (!hsi_protocol_ctx->hsi_ctrl_msg_array_l2h[i].msg)
			BUG();

		buf = kmalloc(HSI_LOGICAL_L2HEADER_MESSAGE_SIZE,
				GFP_KERNEL | GFP_DMA);

		if (!buf)
			BUG();

		sg_init_one(
			hsi_protocol_ctx->hsi_ctrl_msg_array_l2h[i].msg->sgt.sgl,
			buf,
			HSI_LOGICAL_L2HEADER_MESSAGE_SIZE);


		hsi_protocol_ctx->hsi_ctrl_msg_array_l2h[i].free = 1;
	}

	for (i = 0; i < hsi_protocol_ctx->nb_client; i++) {
		cl = hsi_protocol_ctx->cl[i]->hsi_cl;

		if (!cl) {
			DPRINTK("Client %d not initialised\n", i);
			err =  -ENOMEM;
			goto out;
		}

		setup_timer(&hsi_protocol_ctx->cl[i]->rx_mem,
				hsi_rx_mem_test,
				(unsigned long)hsi_protocol_ctx->cl[i]);

		hsi_protocol_ctx->cl[i]->client_id = i;

		DPRINTK("Client %d claim port\n", i);

		err = hsi_claim_port(cl, 1);
		if (err < 0) {
			DPRINTK("HSI port %d already claimed\n", i);
			goto out;
		}

		/* Several INIT */
		spin_lock(&hsi_protocol_ctx->cl[i]->tx_lock);
		hsi_protocol_ctx->cl[i]->send_state = HSI_TX_IDLE;
		hsi_protocol_ctx->cl[i]->recv_state = HSI_RX_IDLE;
		spin_unlock(&hsi_protocol_ctx->cl[i]->tx_lock);

		/* Save top level HSI protocol context per client
		(to be able to communicate with other
		clients, especialy control client) */
		hsi_protocol_ctx->cl[i]->hsi_top_protocol_context =
			hsi_protocol_ctx;
	}

	/* Setup port using control client setup */
	if (hsi_get_port(
		hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl)->setup) {
		hsi_setup(hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl);
	} else {
		DPRINTK("hsi_get_port(cl)->setup is NULL\n");
		err =  -ENOMEM;
		goto out;
	}

	init_timer_deferrable(&hsi_protocol_ctx->inactivity_timer);
	hsi_protocol_ctx->inactivity_timer.data =
		(unsigned long)hsi_protocol_ctx;
	hsi_protocol_ctx->inactivity_timer.function =
		hsi_logical_inactivity_timer;

	hsi_protocol_ctx->inactivity_timer_running = false;

	hsi_protocol_ctx->main_state = HSI_CONFIG_EXCHANGE_NO_DONE;
	hsi_protocol_ctx->cfg_ex.tx_boot_info_req_complete = 0;
	hsi_protocol_ctx->cfg_ex.rx_boot_info_resp_complete = 0;
	hsi_protocol_ctx->cfg_ex.tx_boot_info_resp_complete = 0;
	setup_timer(&hsi_protocol_ctx->cfg_ex.to,
		hsi_logical_config_fail,
		(unsigned long)hsi_protocol_ctx);
	mod_timer(&hsi_protocol_ctx->cfg_ex.to,
			jiffies + msecs_to_jiffies(10000));
	wake_lock_timeout(&wake_resoft, 31 * HZ);

	/* Init is done - wait incomming data on RX control bank
	As several l2 header can be received on a raw, allocate
	several RX control message (one per bank).
	Later each time a RX control message is completed another
	one will be allocated.
	=> hsi physical will always have a pool of HSI_NB_CLIENT msg*/
	for (i = 0; i < HSI_NB_CLIENT; i++)
		hsi_logical_wait_rx_control(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			HSI_CONTROL_CHANNEL);

	hsi_logical_wait_rx_control_l2h(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			1);
	hsi_logical_wait_rx_control_l2h(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			2);
	hsi_logical_wait_rx_control_l2h(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			3);

	hsi_logical_wait_rx_control_wrong_channel(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			4);
	hsi_logical_wait_rx_control_wrong_channel(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			5);
	hsi_logical_wait_rx_control_wrong_channel(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			6);
	hsi_logical_wait_rx_control_wrong_channel(
			hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl,
			7);

	hsi_logical_start_tx(hsi_protocol_ctx);

	return 0;

out:
	/* Free pool of control message */
	for (i = 0; i < HSI_LOGICAL_NB_CONTROL_MSG; i++) {
		kfree(sg_virt(hsi_protocol_ctx->
				hsi_ctrl_msg_array[i].msg->
					sgt.sgl));
		hsi_free_msg(hsi_protocol_ctx->hsi_ctrl_msg_array[i].msg);
	}

	/* Free pool of control message l2h*/
	for (i = 0; i < HSI_LOGICAL_NB_CONTROL_MSG_L2H; i++) {
		kfree(sg_virt(hsi_protocol_ctx->
				hsi_ctrl_msg_array_l2h[i].msg->
					sgt.sgl));
		hsi_free_msg(hsi_protocol_ctx->hsi_ctrl_msg_array_l2h[i].msg);
	}
	return err;
}

int
hsi_logical_close(struct hsi_protocol *hsi_protocol_ctx)
{
	struct hsi_client *cl;
	int err = 0;
	int i;

	EPRINTK("Modem crash occurs - NB modem reboot since power on : %d\n",
			++debug_nb_crash_modem);
	wake_lock_timeout(&wake_resoft, 30 * HZ);
	del_timer(&hsi_protocol_ctx->cfg_ex.to);

	/*hsi_logical_send_power_off(hsi_protocol_ctx);*/
	spin_lock_bh(&hsi_protocol_ctx->tx_activity_status_lock);
	hsi_protocol_ctx->wake_state = 0;
	spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

	if (netif_running(hsi_protocol_ctx->netdev))
		netif_carrier_off(hsi_protocol_ctx->netdev);

	/* scan the list of clients in reverse order so that control
	channel is the last one */
	/* Mixed mode messages are attached to control channel 0 */
	/* they shall be released when all other channels are closed */
	for (i = hsi_protocol_ctx->nb_client-1; i >= 0; i--) {

		cl = hsi_protocol_ctx->cl[i]->hsi_cl;

		if (!cl) {
			DPRINTK("Client %d not initialised\n", i);
			err = -ENOMEM;
			goto out;
		}
		del_timer(&hsi_protocol_ctx->cl[i]->rx_mem);

		spin_lock_bh(&hsi_protocol_ctx->cl[i]->tx_lock);

		hsi_protocol_ctx->cl[i]->send_state = HSI_TX_CLOSE;
		hsi_protocol_ctx->cl[i]->recv_state = HSI_RX_CLOSE;

		spin_unlock_bh(&hsi_protocol_ctx->cl[i]->tx_lock);

	}

	hsi_flush(hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID]->hsi_cl);


	netif_wake_subqueue(hsi_protocol_ctx->netdev,
		HSI_LOG_CL_ID_TO_QUEUE_ID(hsi_protocol_ctx->cl[1]->client_id));

	netif_wake_subqueue(hsi_protocol_ctx->netdev,
		HSI_LOG_CL_ID_TO_QUEUE_ID(hsi_protocol_ctx->cl[2]->client_id));

	netif_wake_subqueue(hsi_protocol_ctx->netdev,
		HSI_LOG_CL_ID_TO_QUEUE_ID(hsi_protocol_ctx->cl[3]->client_id));

	spin_lock_bh(&hsi_protocol_ctx->tx_activity_status_lock);
	{
		del_timer(&hsi_protocol_ctx->inactivity_timer);
		hsi_protocol_ctx->inactivity_timer_running = false;
	}
	spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

	/* Now that all FIFO are flushed release HSI IP */
	for (i = 0; i < hsi_protocol_ctx->nb_client; i++) {
		/* Perform HW release just once (confusion in
		port comprehension != bank) */
		cl = hsi_protocol_ctx->cl[i]->hsi_cl;
		hsi_release_port(hsi_protocol_ctx->cl[i]->hsi_cl);
	}

	/* Free pool of control message */
	for (i = 0; i < HSI_LOGICAL_NB_CONTROL_MSG; i++) {
		kfree(sg_virt(hsi_protocol_ctx->
				hsi_ctrl_msg_array[i].msg->
					sgt.sgl));
		hsi_free_msg(hsi_protocol_ctx->hsi_ctrl_msg_array[i].msg);
	}

	/* Free pool of control message l2h*/
	for (i = 0; i < HSI_LOGICAL_NB_CONTROL_MSG_L2H; i++) {
		kfree(sg_virt(hsi_protocol_ctx->
				hsi_ctrl_msg_array_l2h[i].msg->
					sgt.sgl));
		hsi_free_msg(hsi_protocol_ctx->hsi_ctrl_msg_array_l2h[i].msg);
	}

	return 0;

out:
	return err;
}

void
hsi_logical_start_tx(struct hsi_protocol *hsi_protocol_ctx)
{
	DPRINTK("%s - %d", __func__, hsi_protocol_ctx->wake_state);
	spin_lock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

#ifdef HSI_V3_USED
	hsi_protocol_ctx->power_state = HSI_POWER_ON_STATE;
#endif

	if (0 == hsi_protocol_ctx->wake_state) {

		hsi_protocol_ctx->wake_state = 1;

		spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

		DPRINTK_HIGH(">hsi_start_tx called by HSI logical");

		hsi_start_tx(hsi_protocol_ctx->
				cl[HSI_CONTROL_CLIENT_ID]->
					hsi_cl);

	} else {
		spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);
	}
}

void
hsi_logical_stop_tx(struct hsi_protocol *hsi_protocol_ctx)
{

#ifdef HSI_LOGICAL_POWER_SAVING
	DPRINTK("%s - %d", __func__, hsi_protocol_ctx->wake_state);

	spin_lock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

#ifdef HSI_V3_USED

	if (hsi_protocol_ctx->power_state == HSI_POWER_SEND_OFF_STATE) {

		/*End the power sequence now*/
		hsi_protocol_ctx->power_state = HSI_POWER_OFF_STATE;
#endif /*HSI_V3_USED*/

	if (1 == hsi_protocol_ctx->wake_state) {

		hsi_protocol_ctx->wake_state = 0;

			spin_unlock_bh(
				&hsi_protocol_ctx->tx_activity_status_lock);

			DPRINTK_HIGH("<hsi_stop_tx called by HSI logical");

			hsi_stop_tx(hsi_protocol_ctx->
					cl[HSI_CONTROL_CLIENT_ID]->
						hsi_cl);
	} else {
			spin_unlock_bh(
				&hsi_protocol_ctx->tx_activity_status_lock);
		}
#ifdef HSI_V3_USED
	} else {
		spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);
	}
#endif /*HSI_V3_USED*/

#endif /* HSI_LOGICAL_POWER_SAVING */
}

#ifdef HSI_USE_SEND_SCHEDULED
void
hsi_logical_send_work(struct work_struct *work)
{
	struct hsi_protocol_client *hsi_data_client =
	container_of(work, struct hsi_protocol_client, send_work);
#else
int
hsi_logical_do_send(struct hsi_protocol_client *hsi_data_client)
{
#endif /*HSI_USE_SEND_SCHEDULED*/

	struct hsi_msg *msg = hsi_data_client->data_msg_to_be_send;
	struct hsi_protocol *hsi_protocol_ctx =
		hsi_data_client->hsi_top_protocol_context;


	msg->destructor = hsi_logical_data_free_msg_and_skb;
	msg->complete = hsi_logical_tx_data_complete;
	msg->channel =
	HSI_LOGICAL_SET_CHANNEL_ACCORDING_TO_PRIO(hsi_data_client->client_id);

	hsi_data_client->send_state = HSI_TX_HDR_SENT;
	DPRINTK("%s - L2 header sent: 0x%08x\n",
		 __func__,
		hsi_data_client->tx_l2_header);

	spin_lock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

	/* Stop inactivity timer. It will be relaunched on TX
	data completion callback */
	del_timer(&hsi_protocol_ctx->inactivity_timer);
	hsi_protocol_ctx->inactivity_timer_running = false;

	spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

	hsi_logical_start_tx(hsi_protocol_ctx);

	hsi_logical_alloc_msg_and_send_on_control_channel(
			hsi_data_client,
			hsi_data_client->tx_l2_header,
			HSI_LOGICAL_SET_CHANNEL_ACCORDING_TO_PRIO(
				hsi_data_client->client_id
				),
			hsi_logical_tx_control_complete);
#ifdef HSI_USE_SEND_SCHEDULED
	return;
#else
	return 0;
#endif /*HSI_USE_SEND_SCHEDULED*/

}

int
hsi_logical_send(struct hsi_protocol *hsi_protocol_ctx,
		struct sk_buff *skb,
		u16 client_id)
{
	struct hsi_protocol_client *hsi_data_client ;
	struct hsi_msg *msg;
	u32	l2hdr;

#ifdef HSI_LOGICAL_USE_DEBUG
	u8 mapid = MAPID_FROM_HEADER(*(u32 *)skb->data);
	int client_id_according_to_mapid = hsi_logical_get_client_id(mapid);

	DPRINTK_HIGH_TX("%s on mapid %d - client id %d data = 0x%08X\n",
		__func__, mapid, client_id, *(u32 *)skb->data);

	BUG_ON(client_id_according_to_mapid != client_id);
#endif


#ifdef HSI_LOGICAL_POWER_SAVING
	wake_lock(&wake_temp);
#endif

	hsi_data_client = hsi_protocol_ctx->cl[client_id];

	/* Get L2 header of msg */
	l2hdr = *(u32 *)skb->data;
	/* Remove L2 header from data message */
	skb_pull(skb, HSI_LOGICAL_CONTROL_MESSAGE_SIZE);

	/* Build data message */
	msg = hsi_logical_alloc_data(skb, GFP_ATOMIC | GFP_DMA);

	DPRINTK("HSI LOGICAL- NB_FRAGS = %d total size: %d frags size: %d\n",
			skb_shinfo(skb)->nr_frags, skb->len, skb->data_len);

	if (!msg) {
		DPRINTK("Dropping tx data: No memory\n");
		goto drop;
	}
	spin_lock_bh(&hsi_data_client->tx_lock);

	/* Build L2 header msg */
	hsi_data_client->tx_l2_header = l2hdr;

	/* Reset nb time L2 header has been sent */
	hsi_data_client->nb_time_l2_header_sent = 0;

	/* Associated net device client queue is stopped when client
	is not ready to transmit */
	if ((hsi_protocol_ctx->main_state != HSI_CONFIG_EXCHANGE_DONE)
		|| (hsi_data_client->send_state !=  HSI_TX_IDLE)) {
			int m = hsi_protocol_ctx->main_state;
			int s = hsi_data_client->send_state;
			spin_unlock_bh(&hsi_data_client->tx_lock);
			hsi_free_msg(msg);
		EPRINTK("%s - main_state %d, send_state %d\n",
			 __func__, m, s);
			return -ENETDOWN;
	} else {

		hsi_data_client->send_state = HSI_TX_WAIT;

		/* Save data message in data client context */
		hsi_data_client->data_msg_to_be_send = msg;
		spin_unlock_bh(&hsi_data_client->tx_lock);
#ifdef HSI_USE_SEND_SCHEDULED
		return queue_work(hsi_data_client->send_wq,
				&hsi_data_client->send_work);
#else
		return hsi_logical_do_send(hsi_data_client);
#endif	/*HSI_USE_SEND_SCHEDULED*/
	}

drop:

	return -ENOMEM;
}

static void
hsi_logical_boot_info_req_complete(struct hsi_msg *msg)
{
	struct hsi_protocol_client *hsi_control_client =
		hsi_client_drvdata(msg->cl);
	struct hsi_protocol *hsi_protocol_context =
		hsi_control_client->hsi_top_protocol_context;

	DPRINTK("%s - client ID %d - port nb %d - channel_id %d\n",
		__func__,
		hsi_control_client->client_id,
		(hsi_get_port(msg->cl))->num,
		msg->channel);

	if ((msg->status == HSI_STATUS_ERROR)
	|| (hsi_control_client->client_id != HSI_CONTROL_CLIENT_ID)) {
		DPRINTK("%s - TX control error !!\n", __func__);

		hsi_logical_stop_tx(hsi_protocol_context);
		BUG();
	} else {
		hsi_protocol_context->cfg_ex.tx_boot_info_req_complete = 1;
		hsi_logical_check_if_boot_done(hsi_protocol_context);
	}
}

static void
hsi_logical_boot_info_resp_complete(struct hsi_msg *msg)
{
	struct hsi_protocol_client *hsi_control_client =
				hsi_client_drvdata(msg->cl);
	struct hsi_protocol *hsi_protocol_context =
				hsi_control_client->hsi_top_protocol_context;

	DPRINTK("%s - client ID %d - port nb %d - channel_id %d\n",
			__func__,
		hsi_control_client->client_id,
		(hsi_get_port(msg->cl))->num,
		msg->channel);

	if ((msg->status == HSI_STATUS_ERROR)
	|| (hsi_control_client->client_id != HSI_CONTROL_CLIENT_ID)) {
		DPRINTK("%s - TX control error !!\n", __func__);

		hsi_logical_stop_tx(hsi_protocol_context);
		BUG();
	} else {
		hsi_protocol_context->cfg_ex.tx_boot_info_resp_complete = 1;
		hsi_logical_check_if_boot_done(hsi_protocol_context);
	}
}

static void
hsi_logical_check_if_boot_done(struct hsi_protocol *hsi_protocol_context)
{
	DPRINTK("%s - tx_boot_info_req_complete %d" \
		" - rx_boot_info_resp_complete %d - " \
			"tx_boot_info_resp_complete %d\n",
		__func__,
		hsi_protocol_context->cfg_ex.tx_boot_info_req_complete,
		hsi_protocol_context->cfg_ex.rx_boot_info_resp_complete,
		hsi_protocol_context->cfg_ex.tx_boot_info_resp_complete);

	if (hsi_protocol_context->main_state != HSI_CONFIG_EXCHANGE_DONE) {
		if (hsi_protocol_context->cfg_ex.tx_boot_info_req_complete
		&& hsi_protocol_context->cfg_ex.rx_boot_info_resp_complete
		&& hsi_protocol_context->cfg_ex.tx_boot_info_resp_complete) {
			/* Configuration exchange is finished */
			/* Compare sent_ape_config and received_modem_config */

			EPRINTK("Baseband HSI link is up:\n"
					" - RX L2Header size %d bytes\n"
#ifdef HSI_LOGICAL_RXDATA_128_BYTES_PADDED
					" - RX data 128 bytes padded\n"
#endif
#ifdef HSI_LOGICAL_POWER_SAVING
					" - Power saving activated: %dms inactivity timer\n"
#else
					" - Power saving deactivated\n"
#endif
				, HSI_LOGICAL_L2HEADER_MESSAGE_SIZE
#ifdef HSI_LOGICAL_POWER_SAVING
				, HSI_LOGICAL_INACTIVITY_TIMER
#endif
				);

			if (hsi_protocol_context->cfg_ex.sent_ape_config !=
			hsi_protocol_context->cfg_ex.received_modem_config) {
				/* TODO adapt config to match both peers */

				EPRINTK("sent_ape_config: 0x%08x" \
						" received_modem_config:0x%08x",
				hsi_protocol_context->cfg_ex.sent_ape_config,
					hsi_protocol_context->
						cfg_ex.received_modem_config);

				hsi_logical_stop_tx(hsi_protocol_context);

				BUG();

				/* If necessary setup again control bank */
				/* hsi_setup(hsi_protocol_ctx->
					cl[HSI_CONTROL_CLIENT_ID]->hsi_cl); */
			}

			del_timer(&hsi_protocol_context->cfg_ex.to);
			wake_unlock(&wake_resoft);
			spin_lock_bh(&hsi_protocol_context->
					tx_activity_status_lock);

			hsi_protocol_context->main_state =
				HSI_CONFIG_EXCHANGE_DONE;

			/* Start inactivity timer. On expiration, if
			all queues are ready call hsi_stop_tx */
			mod_timer(&hsi_protocol_context->inactivity_timer,
				jiffies + msecs_to_jiffies(
					HSI_LOGICAL_INACTIVITY_TIMER)
				);
			hsi_protocol_context->inactivity_timer_running = true;

			spin_unlock_bh(&hsi_protocol_context->
					tx_activity_status_lock);

			/* Wake up all queues to allow transmission
			at upper layer */
			netif_tx_wake_all_queues(hsi_protocol_context->netdev);
		}
	}
}

static int
hsi_logical_alloc_msg_and_send_on_control_channel(
	struct hsi_protocol_client *hsi_client_context,
	u32 msg,
	unsigned char channel_id,
	void (*complete)(struct hsi_msg *msg))
{
	struct hsi_msg *msg_ctrl;
	u32 *data;

	msg_ctrl = hsi_logical_alloc_tx_control(hsi_client_context, complete);

	data = sg_virt(msg_ctrl->sgt.sgl);
	*data = msg;

	msg_ctrl->channel = channel_id ;

	return hsi_logical_send_on_control_channel(msg_ctrl,
						hsi_client_context);
}

/* Send current message on control channel */
static int
hsi_logical_send_on_control_channel(
	struct hsi_msg *msg,
	struct hsi_protocol_client *hsi_client)
{
	struct hsi_protocol_client *hsi_control_client =
		hsi_client->
			hsi_top_protocol_context->cl[HSI_CONTROL_CLIENT_ID];
	int err;
	DPRINTK("%s - channel_id %d\n", __func__, msg->channel);

	err = hsi_async_write(hsi_control_client->hsi_cl, msg);

	if (err)
		EPRINTK("%s - hsi_async_write error=%d\n", __func__, err);

	return err;
}

static void
hsi_logical_tx_control_complete(struct hsi_msg *msg)
{
	struct hsi_protocol_client *hsi_control_client =
		hsi_client_drvdata(msg->cl);

#ifdef HSI_LOGICAL_USE_DEBUG
	if (msg->channel == 0) {
		DPRINTK_HIGH_RX("%s - client ID %d" \
			" - port nb %d - channel_id %d\n",
			__func__,
		hsi_control_client->client_id,
		(hsi_get_port(msg->cl))->num,
		msg->channel);
	} else {
		DPRINTK_HIGH_TX("%s - client ID %d" \
			" - port nb %d - channel_id %d\n",
			__func__,
			hsi_control_client->client_id,
			(hsi_get_port(msg->cl))->num,
			msg->channel);
	}
#endif

	if (hsi_control_client->send_state != HSI_TX_CLOSE) {
		if ((msg->status == HSI_STATUS_ERROR) ||
			(hsi_control_client->client_id !=
				HSI_CONTROL_CLIENT_ID)) {
				EPRINTK("TX control error\n");
		}
	}

	/* Free control message */
	hsi_logical_tx_control_destructor(msg);
}

static int
hsi_ack_received(
	struct hsi_protocol_client *hsi_control_client,
	unsigned char channel_id)
{
	struct hsi_protocol_client *hsi_data_client;
	struct hsi_protocol *hsi_protocol_context;

	int err = 0;

	DPRINTK_HIGH_TX("%s - for channel_id %d\n", __func__, channel_id);

	if (hsi_logical_retrieve_data_client(
		hsi_control_client,
		channel_id,
		&hsi_data_client) < 0) {
		EPRINTK("hsi_ack_received - ERROR mismatch with" \
			" channel ID used in L2 Header\n");
		return 0;
	}

	hsi_protocol_context = hsi_data_client->hsi_top_protocol_context;

	spin_lock_bh(&hsi_data_client->tx_lock);

	if (unlikely(hsi_data_client->send_state != HSI_TX_HDR_SENT)) {
		/* Should never happen */
		EPRINTK("hsi_ack_received - ERROR: state is %d !!\n",
			hsi_data_client->send_state);
		spin_unlock_bh(&hsi_data_client->tx_lock);
		return 0;
	}

	hsi_data_client->send_state = HSI_TX_ACK_RCV;

	spin_unlock_bh(&hsi_data_client->tx_lock);

	/* Next operation performs the write operation*/
	if (hsi_async_write(hsi_data_client->hsi_cl,
				hsi_data_client->data_msg_to_be_send)) {

		spin_lock_bh(&hsi_data_client->tx_lock);
		hsi_data_client->send_state = HSI_TX_IDLE;
		hsi_data_client->data_msg_to_be_send = NULL;
		spin_unlock_bh(&hsi_data_client->tx_lock);

		/* Send is aborted - wake up associated queue*/
		netif_wake_subqueue(hsi_protocol_context->netdev,
			HSI_LOG_CL_ID_TO_QUEUE_ID(hsi_data_client->client_id));

		EPRINTK("hsi_ack_received hsi_async_write - ERROR: %d\n", err);
	}

	return 0;
}

int
hsi_nack_received(
	struct hsi_protocol_client *hsi_control_client,
	unsigned char channel_id)
{
	struct hsi_protocol_client *hsi_data_client;

	EPRINTK("%s - channel_id %d\n", __func__, channel_id);

	if (hsi_logical_retrieve_data_client(hsi_control_client,
						channel_id,
						&hsi_data_client) < 0) {
		EPRINTK("%s - ERROR mismatch with channel" \
			" ID used in L2 Header\n", __func__);

		return 0;
	}

	spin_lock_bh(&hsi_data_client->tx_lock);

	if (unlikely(hsi_data_client->send_state != HSI_TX_HDR_SENT)) {
		/* Should never happen */
		EPRINTK("%s - ERROR: state is %d !!\n",
			__func__,
			hsi_data_client->send_state);
		spin_unlock_bh(&hsi_data_client->tx_lock);
		return 0;
	}

	spin_unlock_bh(&hsi_data_client->tx_lock);

	if (unlikely(hsi_data_client->nb_time_l2_header_sent++ >= 2)) {
		EPRINTK("%s - ERROR:" \
				" nb_time_l2_header_sent = %d\n",
			__func__,
			hsi_data_client->nb_time_l2_header_sent);

		/* Free data message */
		hsi_logical_data_free_msg_and_skb(
			hsi_data_client->data_msg_to_be_send);

	} else {
		/* Check WAKE line, in case TX is off,
		need to start TX for sending L2H */
		hsi_logical_start_tx(hsi_data_client->hsi_top_protocol_context);

		/* Send again L2 header */
		hsi_logical_alloc_msg_and_send_on_control_channel(
			hsi_data_client,
			hsi_data_client->tx_l2_header,
			HSI_LOGICAL_SET_CHANNEL_ACCORDING_TO_PRIO(
				hsi_data_client->client_id),
			hsi_logical_tx_control_complete);
	}

	return 0;
}

static void
hsi_logical_tx_data_complete(struct hsi_msg *msg)
{
	struct hsi_protocol_client *hsi_data_client =
		hsi_client_drvdata(msg->cl);
	struct hsi_protocol *hsi_protocol_context =
		hsi_data_client->hsi_top_protocol_context;

	DPRINTK_HIGH_TX("%s - client ID %d" \
		" - port nb %d - channel_id %d\n",
		__func__,
		hsi_data_client->client_id,
		(hsi_get_port(msg->cl))->num,
		msg->channel);

	if ((msg->status == HSI_STATUS_ERROR)
	|| (hsi_data_client->client_id == HSI_CONTROL_CLIENT_ID)
	|| (msg->channel == HSI_CONTROL_CHANNEL)) {
		EPRINTK("TX data error\n");
	}

	hsi_logical_data_free_msg_and_skb(msg);

	spin_lock_bh(&hsi_data_client->tx_lock);

	if (hsi_data_client->send_state != HSI_TX_CLOSE) {
		if (unlikely(hsi_data_client->send_state != HSI_TX_ACK_RCV))
			EPRINTK("TX data error - wrong state !!\n");

		hsi_data_client->send_state = HSI_TX_IDLE;
		spin_unlock_bh(&hsi_data_client->tx_lock);

		/* Send is now complete - wake up associated queue*/
		netif_wake_subqueue(hsi_protocol_context->netdev,
		HSI_LOG_CL_ID_TO_QUEUE_ID(hsi_data_client->client_id));
	} else
		spin_unlock_bh(&hsi_data_client->tx_lock);

	DPRINTK("HSI netdevice Subqueue %d is now awake\n",
	HSI_LOG_CL_ID_TO_QUEUE_ID(hsi_data_client->client_id));


	/* Start inactivity timer. On expiration,
		if all queues are ready call hsi_stop_tx */
	spin_lock_bh(&hsi_protocol_context->tx_activity_status_lock);
	{
		mod_timer(&hsi_protocol_context->inactivity_timer,
		jiffies + msecs_to_jiffies(HSI_LOGICAL_INACTIVITY_TIMER));
		hsi_protocol_context->inactivity_timer_running = true;
	}
	spin_unlock_bh(&hsi_protocol_context->tx_activity_status_lock);

}


/* Allocate message on control port and wait reception */
static void
hsi_logical_wait_rx_control(struct hsi_client *cl, unsigned char channel_id)
{
	struct hsi_msg *msg = hsi_logical_get_control(hsi_client_drvdata(cl));

	DPRINTK("%s - on channel id %d\n", __func__, channel_id);

	msg->channel = channel_id;
	msg->complete = hsi_logical_rx_control_complete;
	msg->destructor = hsi_logical_rx_control_destructor;

	if (hsi_async_read(cl, msg))
		EPRINTK("%s - hsi_async_read error\n", __func__);

}

/* Allocate message on control port and wait reception */
static void
hsi_logical_wait_rx_control_l2h(struct hsi_client *cl, unsigned char channel_id)
{
	struct hsi_msg *msg = hsi_logical_get_control_l2h(hsi_client_drvdata(cl));

	DPRINTK("%s - on channel id %d\n", __func__, channel_id);

	msg->channel = channel_id;
	msg->complete = hsi_logical_rx_control_complete;
	msg->destructor = hsi_logical_rx_control_destructor_l2h;

	if (hsi_async_read(cl, msg))
		EPRINTK("%s - hsi_async_read error\n", __func__);

}

static void
hsi_logical_rx_control_complete_wrong_channel(struct hsi_msg *msg)
{
	EPRINTK("%s  Receive buffer on wrong channel: 0x%08x, channel_id %d\n",
		__func__,
		*(u32 *)sg_virt(msg->sgt.sgl), msg->channel);

	msg->destructor(msg);
}

/* Allocate message on control port and wait reception */
static void
hsi_logical_wait_rx_control_wrong_channel(
				struct hsi_client *cl,
				unsigned char channel_id)
{
	struct hsi_msg *msg = hsi_logical_get_control(hsi_client_drvdata(cl));

	DPRINTK("%s - on channel id %d\n", __func__, channel_id);

	msg->channel = channel_id;
	msg->complete = hsi_logical_rx_control_complete_wrong_channel;
	msg->destructor = hsi_logical_rx_control_destructor;

	if (hsi_async_read(cl, msg))
		EPRINTK("%s - hsi_async_read error\n", __func__);

}

static void
hsi_logical_rx_control_complete(struct hsi_msg *msg)
{
#ifdef HSI_USE_RCV_SCHEDULED

	struct hsi_protocol_client *hsi_control_client =
		hsi_client_drvdata(msg->cl);
	struct hsi_protocol *hsi_protocol_ctx =
		hsi_control_client->hsi_top_protocol_context;

	DPRINTK_HIGH_RX("%s  Receive control buffer content: 0x%08x, channel_id %d\n",
			__func__,
			*(u32 *)sg_virt(msg->sgt.sgl), msg->channel);

	spin_lock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

	/*Now we need to schedule the reception as start_tx will
	generate some exceptions*/
	if (0 == hsi_protocol_ctx->wake_state) {
		DPRINTK("hsi logical woken up by modem (0x%p)\n", msg);

		spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);

		spin_lock_bh(&hsi_control_client->rcv_msgs_lock);
		list_add_tail(&msg->link, &hsi_control_client->rcv_msgs);
		spin_unlock_bh(&hsi_control_client->rcv_msgs_lock);

		queue_work(hsi_control_client->rcv_wq,
			&hsi_control_client->rcv_work);

	} else {
		spin_unlock_bh(&hsi_protocol_ctx->tx_activity_status_lock);
		hsi_logical_rx_control_complete_step2(msg);
	}
#else
	hsi_logical_rx_control_complete_step2(msg);

#endif /* HSI_USE_RCV_SCHEDULED */
}

#ifdef HSI_USE_RCV_SCHEDULED
void
hsi_logical_rcv_work(struct work_struct *work)
{
	struct hsi_msg *msg, *tmp_msg;
	struct hsi_protocol_client *hsi_control_client =
			container_of(work,
				struct hsi_protocol_client,
				rcv_work);

	spin_lock_bh(&hsi_control_client->rcv_msgs_lock);

	list_for_each_entry_safe(msg,
			tmp_msg,
			&hsi_control_client->rcv_msgs,
			link) {
		list_del(&msg->link);
		spin_unlock_bh(&hsi_control_client->rcv_msgs_lock);

		hsi_logical_rx_control_complete_step2(msg);
		spin_lock_bh(&hsi_control_client->rcv_msgs_lock);
	}

	spin_unlock_bh(&hsi_control_client->rcv_msgs_lock);
	DPRINTK("hsi: calling complete callback (0x%p)\n", msg);

}
#endif /* HSI_USE_RCV_SCHEDULED */

static void
hsi_logical_rx_control_complete_step2(struct hsi_msg *msg)
{
	struct hsi_protocol_client *hsi_control_client =
		hsi_client_drvdata(msg->cl);
	struct hsi_protocol *hsi_protocol_ctx =
		hsi_control_client->hsi_top_protocol_context;
	struct hsi_client *cl = msg->cl;
	u32 control_msg;
	unsigned char channel_id = msg->channel;

	DPRINTK("hsi_logical_rx_control_complete - client ID %d" \
		" - port nb %d - channel_id %d\n",
		hsi_control_client->client_id,
		(hsi_get_port(msg->cl))->num,
		msg->channel);

	if (hsi_control_client->recv_state == HSI_RX_CLOSE) {
		/* Free control message */
		msg->destructor(msg);
		return;
	}

	if (msg->status == HSI_STATUS_ERROR) {
		EPRINTK("RX error on control bank\n");
		goto out;
	}

	if (hsi_control_client->client_id != HSI_CONTROL_CLIENT_ID) {
		EPRINTK("Not on control port !!\n");
		goto out;
	}

	control_msg = *(u32 *) sg_virt(msg->sgt.sgl);

	DPRINTK(" Receive control buffer content: 0x%08x\n",
			*(u32 *)sg_virt(msg->sgt.sgl));

	switch (MAPID_FROM_HEADER(control_msg)) {
	case HSI_BOOT_INFO_REQ: /*We received a REQ from modem */

		if (hsi_protocol_ctx->main_state != HSI_CONFIG_EXCHANGE_DONE) {
			/* TODO for next release: analyse message content
			to define the content of BOOT_INFO_RESP, and delay
			sending of HSI_BOOT_INFO_RESP after the end of
			config exchange.
			Indeed if banks need to be reconfigured, we need
			to block the modem to avoid receiving a L2 header
			before a potential reconfigure of the banks.
			For now we assume modem and APE configs are identical
			=> keep sending of HSI_BOOT_INFO_RESP immediately
			to allow the modem to send messages as soon as possible
			(to speed the boot).
			That means we can receive L2 header from modem,
			even if config exchange is not completed from
			APE point of view. This is authorized in HSI
			logical (only TX is bloqued until end of config
			exchange). */

			if ((control_msg&CONF_VERSION_MASK) != HSI_SUPPORTED_VERSION) {
				EPRINTK("Modem HSI version is not compatible in" \
					"control_msg 0x%08x\n", control_msg);
				goto out;
			}

			hsi_logical_alloc_msg_and_send_on_control_channel(
				hsi_control_client,
				(HSI_BOOT_INFO_RESP << 24) |
					(control_msg & 0x00FFFFFF),
				HSI_CONTROL_CHANNEL,
				hsi_logical_boot_info_resp_complete);

			/* TODO retrieve config from port config */
			hsi_protocol_ctx->cfg_ex.sent_ape_config =
				HSI_DEFAULT_BOOT_CONFIG;

			/* Start config exchange by sending HSI_BOOT_INFO_REQ */
			hsi_logical_alloc_msg_and_send_on_control_channel(
				hsi_protocol_ctx->cl[HSI_CONTROL_CLIENT_ID],
					(HSI_BOOT_INFO_REQ << 24) |
						hsi_protocol_ctx->
						cfg_ex.sent_ape_config,
					HSI_CONTROL_CHANNEL,
					hsi_logical_boot_info_req_complete);

		}
		hsi_logical_wait_rx_control(cl, channel_id);
	break;


	case HSI_BOOT_INFO_RESP: /* We received a RESP from modem */

		if (hsi_protocol_ctx->main_state != HSI_CONFIG_EXCHANGE_DONE) {
			hsi_protocol_ctx->cfg_ex.received_modem_config =
				control_msg & 0x00FFFFFF;
			hsi_protocol_ctx->cfg_ex.rx_boot_info_resp_complete = 1;
			hsi_logical_check_if_boot_done(hsi_protocol_ctx);
		}
		hsi_logical_wait_rx_control(cl, channel_id);
	break;


	case HSI_ACK:

		hsi_ack_received(hsi_control_client,
				CHANNEL_FROM_ACK(control_msg));
		hsi_logical_wait_rx_control(cl, channel_id);
	break;


	case HSI_NACK:

		hsi_nack_received(hsi_control_client,
				CHANNEL_FROM_ACK(control_msg));
		hsi_logical_wait_rx_control(cl, channel_id);
	break;


	case HSI_POWER_OFF:
		DPRINTK_HIGH("hsi_logical_rx_control_complete -" \
			"receive POWER_OFF notification");
		/*TRASHED*/
		hsi_logical_wait_rx_control(cl, channel_id);
	break;


	default:
		/* In that case message received on control channel.
		As it is not a control message, it must be a L2 HEADER */
		hsi_l2header_receive(
			hsi_control_client,
			msg->channel,
			control_msg);

	break;
	}

out:

	/* Free control message */
	msg->destructor(msg);

}

static void hsi_rx_mem_test(unsigned long in)
{
	struct hsi_protocol_client *hsi_data_client =
		(struct hsi_protocol_client *)in;
	struct hsi_protocol *hsi_protocol_context =
		hsi_data_client->hsi_top_protocol_context;
	struct hsi_client *cl;
	cl = hsi_data_client->hsi_cl;

	spin_lock_bh(&hsi_data_client->rx_lock);
	if ((hsi_data_client->recv_state == HSI_RX_RVCING_DATA) ||
	(hsi_data_client->recv_state == HSI_RX_HDR_RCV_WHILE_DATA_ONGOING)) {
		spin_unlock_bh(&hsi_data_client->rx_lock);
		hsi_rx_data_ack(hsi_protocol_context, hsi_data_client);
	} else
		spin_unlock_bh(&hsi_data_client->rx_lock);
}

static void hsi_rx_data_ack(struct hsi_protocol *hsi_protocol_context,
				   struct hsi_protocol_client *hsi_data_client)
{
	struct hsi_msg *msg = NULL;
	struct sk_buff *skb;
	u32 l3len;
	unsigned char channel_id = hsi_data_client->rx_l2_header_channel;
	struct hsi_protocol_client *hsi_control_client =
		hsi_data_client->
			hsi_top_protocol_context->
				cl[HSI_CONTROL_CLIENT_ID];
#ifdef HSI_LOGICAL_SEND_ACK_BEFORE_ALLOC
	/* Check WAKE line, in case TX is off,
	need to start TX for sending ACK */
	hsi_logical_start_tx(hsi_protocol_context);

	hsi_logical_alloc_msg_and_send_on_control_channel(
					hsi_control_client,
					(HSI_ACK<<24 | channel_id),
					HSI_CONTROL_CHANNEL,
					hsi_logical_tx_control_complete);
#endif

	l3len = LENGTH_IN_HEADER(hsi_data_client->rx_l2_header);

#ifdef HSI_LOGICAL_RXDATA_128_BYTES_PADDED
	if ((l3len%128) != 0)
		l3len = l3len + 128 - l3len%128;
#endif


	/* Allocate msg for reception */
	skb = netdev_alloc_skb(hsi_protocol_context->netdev, l3len);
	if (unlikely(!skb)) {
		DPRINTK("create_rx_data_msg - No memory for rx skb\n");
		goto err;
	}

	skb->dev = hsi_protocol_context->netdev;
	skb_put(skb, l3len);

	msg = hsi_logical_alloc_data(skb, GFP_ATOMIC | GFP_DMA);
	if (unlikely(!msg)) {
		DPRINTK("create_rx_data_msg - No memory for RX data msg\n");
		dev_kfree_skb(skb);
		goto err;
	}
	msg->destructor = hsi_logical_data_free_msg_and_skb;
	msg->complete = hsi_logical_rx_data_complete;

	msg->channel = channel_id;


	/* Wait data message on corresponding port */
	if (hsi_async_read(hsi_data_client->hsi_cl, msg)) {
		/* SKB is already released by HSI physical */
		EPRINTK("Error hsi_l2header_receive Failed to" \
				"hsi_async_read data message\n");
		return;
	}

	/* Next L2 header will be received on channel X */
	hsi_logical_wait_rx_control_l2h(hsi_control_client->hsi_cl, channel_id);

	/* Then send ACK */
	DPRINTK_HIGH_RX("%s - send ACK for channel_id %d\n",
			__func__,
			channel_id);

#ifndef HSI_LOGICAL_SEND_ACK_BEFORE_ALLOC
	/* Check WAKE line, in case TX is off,
	need to start TX for sending ACK */
	hsi_logical_start_tx(hsi_protocol_context);

	hsi_logical_alloc_msg_and_send_on_control_channel(
					hsi_control_client,
					(HSI_ACK<<24 | channel_id),
					HSI_CONTROL_CHANNEL,
					hsi_logical_tx_control_complete);
#endif

	wake_lock_timeout(&wake_no_suspend_l2header, 1 * HZ);

	return;
err:
	wake_lock_timeout(&wake_no_suspend_l2header, 5 * HZ);
	hsi_data_client->rx_mem.function = hsi_rx_mem_test;
	hsi_data_client->rx_mem.data = (unsigned long)hsi_data_client;
	mod_timer(&hsi_data_client->rx_mem, jiffies + msecs_to_jiffies(50));
	return;
}

static int
hsi_l2header_receive(struct hsi_protocol_client *hsi_control_client,
			unsigned char channel_id,
			u32 l2header)
{
	struct hsi_protocol *hsi_protocol_context =
			hsi_control_client->hsi_top_protocol_context;
	struct hsi_protocol_client *hsi_data_client;
	u32 l3len;
	int client_id;

	if (hsi_logical_retrieve_data_client(
		hsi_control_client,
		channel_id,
		&hsi_data_client) < 0) {
		EPRINTK("hsi_l2header_receive - ERROR mismatch with" \
			" channel ID used in L2 Header\n");
		return 0;
	}
	client_id = hsi_data_client->client_id;

#ifdef HSI_LOGICAL_USE_DEBUG
	BUG_ON(hsi_logical_get_client_id(MAPID_FROM_HEADER(l2header)) != client_id);
#endif

	l3len = LENGTH_IN_HEADER(l2header);

	DPRINTK_HIGH_RX("hsi_l2header_receive - mapid %d - len %d" \
		" - channel_id %d - corresponding client id %d\n",
		MAPID_FROM_HEADER(l2header),
		l3len,
		channel_id,
		client_id);

	if (l3len > MAXIMUM_MSG_LENGTH_ALLOWED) {
		/* length exceed max length => send NACK*/
		EPRINTK("hsi_l2header 0x%08x received - length %d exceed" \
			" max length send NACK\n", l2header, l3len);

		goto nack;
	}
	if (l3len == 0) {
		/* length exceed max length => send NACK*/
		EPRINTK("hsi_l2header 0x%08x received - length %d exceed" \
		" max length send NACK\n", l2header, l3len);

		goto nack;
	} else {
		spin_lock_bh(&hsi_data_client->rx_lock);

		if (hsi_data_client->recv_state == HSI_RX_IDLE) {
			DPRINTK("hsi_l2header_receive RX state of data" \
				" client is idle => send ACK\n");
			hsi_data_client->recv_state = HSI_RX_RVCING_DATA;
		} else if (hsi_data_client->recv_state == HSI_RX_RVCING_DATA) {
			/* RX client is not ready
			Wait previous RX completion before sending ACK */

			EPRINTK("hsi_l2header_receive L2header while" \
				" previous RX data ongoing: 0x%08x on channel_id: %d\n",
				 l2header,
				 channel_id);

			hsi_data_client->pending_rx_l2_header = l2header;
			hsi_data_client->pending_rx_l2_header_channel =
				channel_id;
			hsi_data_client->recv_state =
			HSI_RX_HDR_RCV_WHILE_DATA_ONGOING;

			spin_unlock_bh(&hsi_data_client->rx_lock);
			return 0;
		} else if (hsi_data_client->recv_state ==
				HSI_RX_HDR_RCV_WHILE_DATA_ONGOING) {
			EPRINTK("Error hsi_l2header_receive L2header" \
				" while previous RX data ongoing AND " \
				"there is already a pending RX L2 header\n");
			/*BUG();*/
			spin_unlock_bh(&hsi_data_client->rx_lock);
			return 0;
		}

		/* Stop inactivity timer. It will be relaunched on RX
		data completion callback */
		spin_lock_bh(&hsi_protocol_context->tx_activity_status_lock);
		del_timer(&hsi_protocol_context->inactivity_timer);
		hsi_protocol_context->inactivity_timer_running = false;
		spin_unlock_bh(&hsi_protocol_context->tx_activity_status_lock);

		/* Save rx L2 header (to concatenete later with payload) */
		hsi_data_client->rx_l2_header = l2header;
		hsi_data_client->rx_l2_header_channel = channel_id;
		spin_unlock_bh(&hsi_data_client->rx_lock);
		hsi_rx_data_ack(hsi_protocol_context, hsi_data_client);
	}

	return 0;

nack:

	/* Check WAKE line, in case TX is off,
	need to start TX for sending NACK */
	hsi_logical_start_tx(hsi_protocol_context);

	hsi_logical_alloc_msg_and_send_on_control_channel(
			hsi_control_client,
				(HSI_NACK<<24 | channel_id),
				HSI_CONTROL_CHANNEL,
				hsi_logical_tx_control_complete);

	/* Reallocate to receive the correct L2HEADER
	   Next L2 header will be received on channel X */
	hsi_logical_wait_rx_control_l2h(hsi_control_client->hsi_cl, channel_id);

	return -1;
}


static void
hsi_logical_rx_data_complete(struct hsi_msg *msg)
{
	struct hsi_client *cl = msg->cl;
	struct hsi_protocol_client *hsi_data_client = hsi_client_drvdata(cl);
	struct hsi_protocol *hsi_protocol_context =
		hsi_data_client->hsi_top_protocol_context;
	struct sk_buff *skb = msg->context;

	DPRINTK_HIGH_RX("hsi_logical_rx_data_complete - client ID %d - " \
		"port nb %d - channel_id %d - Latest frame is 0x%08x\n",
		hsi_data_client->client_id,
		(hsi_get_port(msg->cl))->num,
		msg->channel,
		*(u32 *)(sg_virt(msg->sgt.sgl) + skb->len - 4));

	/* Start inactivity timer if not already running for TX. On expiration,
	if all queues are ready call hsi_stop_tx */
	spin_lock_bh(&hsi_protocol_context->tx_activity_status_lock);
	{
		if (hsi_protocol_context->inactivity_timer_running != true) {
			mod_timer(&hsi_protocol_context->inactivity_timer,
				jiffies + msecs_to_jiffies(HSI_LOGICAL_INACTIVITY_TIMER));
			hsi_protocol_context->inactivity_timer_running = true;
		}
	}
	spin_unlock_bh(&hsi_protocol_context->tx_activity_status_lock);

	if ((msg->status == HSI_STATUS_ERROR)
	|| (hsi_data_client->client_id == HSI_CONTROL_CLIENT_ID)
	|| (msg->channel == HSI_CONTROL_CHANNEL)) {
		EPRINTK("RX data error\n");
		/* Delete msg and skb */
		hsi_logical_data_free_msg_and_skb(msg);
		return;
	}

	/* Add previously received L2 header */
	skb_push(skb, HSI_LOGICAL_CONTROL_MESSAGE_SIZE);
	*(u32 *)(skb->data) = hsi_data_client->rx_l2_header;

#ifdef HSI_LOGICAL_RXDATA_128_BYTES_PADDED
	skb_trim(skb, 4 + LENGTH_IN_HEADER(hsi_data_client->rx_l2_header));
#endif

	/* Delete only hsi msg - skb will be freed by appli */
	hsi_logical_data_free_msg(msg);


	spin_lock_bh(&hsi_data_client->rx_lock);

	if (hsi_data_client->recv_state == HSI_RX_RVCING_DATA) {
		hsi_data_client->recv_state = HSI_RX_IDLE;
		spin_unlock_bh(&hsi_data_client->rx_lock);
		/* Callback receive function */
		hsi_data_client->hsi_top_protocol_context->receive_fn(skb);
	} else if (hsi_data_client->recv_state ==
			HSI_RX_HDR_RCV_WHILE_DATA_ONGOING) {
		EPRINTK("hsi_logical_rx_data_complete - managing" \
			"pending received L2 header: 0x%08x on channel_id %d\n",
			 hsi_data_client->pending_rx_l2_header,
			 hsi_data_client->pending_rx_l2_header_channel);
		hsi_data_client->recv_state = HSI_RX_IDLE;
		spin_unlock_bh(&hsi_data_client->rx_lock);
		/* Callback receive function */
		hsi_data_client->hsi_top_protocol_context->receive_fn(skb);

		hsi_l2header_receive(
			hsi_data_client->hsi_top_protocol_context->
		cl[HSI_CONTROL_CLIENT_ID],
			hsi_data_client->pending_rx_l2_header_channel,
			hsi_data_client->pending_rx_l2_header);
	} else if (hsi_data_client->recv_state == HSI_RX_CLOSE) {
		spin_unlock_bh(&hsi_data_client->rx_lock);
		kfree_skb(skb);
	} else {
		spin_unlock_bh(&hsi_data_client->rx_lock);
		EPRINTK("hsi_logical_rx_data_complete - wrong " \
				"state for RX client: %d\n",
			hsi_data_client->recv_state);
	}
}


static void
hsi_logical_data_free_msg_and_skb(struct hsi_msg *msg)
{
	struct sk_buff *skb = msg->context;

	DPRINTK("hsi_logical_data_free_msg_and_skb - port num %d" \
		" - channel_id %d\n", (hsi_get_port(msg->cl))->num,
		msg->channel);

	msg->destructor = NULL;

	dev_kfree_skb(skb);
	hsi_free_msg(msg);
}

static void
hsi_logical_data_free_msg(struct hsi_msg *msg)
{
	DPRINTK("hsi_logical_data_free_msg - port num %d" \
		" - channel_id %d\n", (hsi_get_port(msg->cl))->num,
		msg->channel);

	msg->destructor = NULL;
	hsi_free_msg(msg);
}

static void
hsi_logical_control_destructor(struct hsi_msg *msg)
{
	struct hsi_protocol_client *hsi_client =
		hsi_client_drvdata(msg->cl);
	struct hsi_protocol *hsi_protocol_context =
	hsi_client->hsi_top_protocol_context;
	int i = 0;

	spin_lock_bh(&hsi_protocol_context->ctrl_msg_array_lock);

	while ((msg != hsi_protocol_context->hsi_ctrl_msg_array[i].msg)
	&& (i < HSI_LOGICAL_NB_CONTROL_MSG-1))
		i++;

	if (msg == hsi_protocol_context->hsi_ctrl_msg_array[i].msg) {
		/* Msg to free found */
		DPRINTK("hsi_logical_control_destructor - " \
			"msg id %d is now free\n", i);

		hsi_protocol_context->hsi_ctrl_msg_array[i].free = 1;
	} else {
		/* Msg not found ! */
		BUG();
	}
	spin_unlock_bh(&hsi_protocol_context->ctrl_msg_array_lock);

}

static void
hsi_logical_control_destructor_l2h(struct hsi_msg *msg)
{
	struct hsi_protocol_client *hsi_client =
		hsi_client_drvdata(msg->cl);
	struct hsi_protocol *hsi_protocol_context =
	hsi_client->hsi_top_protocol_context;
	int i = 0;

	spin_lock_bh(&hsi_protocol_context->ctrl_msg_array_l2h_lock);

	while ((msg != hsi_protocol_context->hsi_ctrl_msg_array_l2h[i].msg)
	&& (i < HSI_LOGICAL_NB_CONTROL_MSG_L2H-1))
		i++;

	if (msg == hsi_protocol_context->hsi_ctrl_msg_array_l2h[i].msg) {
		/* Msg to free found */
		DPRINTK("hsi_logical_control_destructor - " \
			"msg id %d is now free\n", i);

		hsi_protocol_context->hsi_ctrl_msg_array_l2h[i].free = 1;
	} else {
		/* Msg not found ! */
		BUG();
	}
	spin_unlock_bh(&hsi_protocol_context->ctrl_msg_array_l2h_lock);

}


static void
hsi_logical_tx_control_destructor(struct hsi_msg *msg)
{
	DPRINTK("hsi_logical_tx_control_destructor on port num %d -" \
		"channel_id %d\n", (hsi_get_port(msg->cl))->num,
		msg->channel);

	hsi_logical_control_destructor(msg);
}


static void
hsi_logical_rx_control_destructor(struct hsi_msg *msg)
{
	DPRINTK("hsi_logical_rx_control_destructor on port num %d-" \
		" channel_id %d\n", (hsi_get_port(msg->cl))->num, msg->channel);

	hsi_logical_control_destructor(msg);
}

static void
hsi_logical_rx_control_destructor_l2h(struct hsi_msg *msg)
{
	DPRINTK("hsi_logical_rx_control_destructor_l2h on port num %d-" \
		" channel_id %d\n", (hsi_get_port(msg->cl))->num, msg->channel);

	hsi_logical_control_destructor_l2h(msg);
}

static struct hsi_msg *
hsi_logical_alloc_data(struct sk_buff *skb, gfp_t flags)
{
	struct hsi_msg *msg;

	msg = hsi_alloc_msg(skb_shinfo(skb)->nr_frags + 1, flags);
	if (!msg)
		goto out;
	hsi_logical_skb_to_msg(skb, msg);
	msg->context = skb;

	return msg;

out:
	EPRINTK("hsi_logical_alloc_data BUG\n") ;
	BUG();
	return NULL; /* Useless, just avoid Klockwork error */
}

static struct hsi_msg *
hsi_logical_get_control(struct hsi_protocol_client *hsi_client_context)
{
	struct hsi_protocol *hsi_protocol_context =
	hsi_client_context->hsi_top_protocol_context;
	int i = 0;

	spin_lock_bh(&hsi_protocol_context->ctrl_msg_array_lock);

	/* Find a free msg */
	while ((hsi_protocol_context->hsi_ctrl_msg_array[i].free == 0)
	&& (i < HSI_LOGICAL_NB_CONTROL_MSG))
		i++;

	if (hsi_protocol_context->hsi_ctrl_msg_array[i].free == 0) {
		/* No free msg !! */
		BUG();
	}

	/* msg is now busy */
	hsi_protocol_context->hsi_ctrl_msg_array[i].free = 0;

	spin_unlock_bh(&hsi_protocol_context->ctrl_msg_array_lock);

	DPRINTK("hsi_logical_get_control - msg id %d is now used\n", i);

	return hsi_protocol_context->hsi_ctrl_msg_array[i].msg;
}

static struct hsi_msg *
hsi_logical_get_control_l2h(struct hsi_protocol_client *hsi_client_context)
{
	struct hsi_protocol *hsi_protocol_context =
	hsi_client_context->hsi_top_protocol_context;
	int i = 0;

	spin_lock_bh(&hsi_protocol_context->ctrl_msg_array_l2h_lock);

	/* Find a free msg */
	while ((i < HSI_LOGICAL_NB_CONTROL_MSG_L2H) &&
		(hsi_protocol_context->hsi_ctrl_msg_array_l2h[i].free == 0))
		i++;

	if (hsi_protocol_context->hsi_ctrl_msg_array_l2h[i].free == 0) {
		/* No free msg !! */
		BUG();
	}

	/* msg is now busy */
	hsi_protocol_context->hsi_ctrl_msg_array_l2h[i].free = 0;

	spin_unlock_bh(&hsi_protocol_context->ctrl_msg_array_l2h_lock);

	DPRINTK("hsi_logical_get_control l2h- msg id %d is now used\n", i);

	return hsi_protocol_context->hsi_ctrl_msg_array_l2h[i].msg;
}

static struct hsi_msg *
hsi_logical_alloc_tx_control(struct hsi_protocol_client *hsi_client_context,
void (*complete)(struct hsi_msg *msg))
{
	struct hsi_msg *msg = hsi_logical_get_control(hsi_client_context);

	msg->destructor = hsi_logical_tx_control_destructor;
	msg->complete = complete;

	return msg;
}

#ifdef HSI_LOGICAL_USE_DEBUG
static int hsi_logical_get_client_id(unsigned char mapid)
{
	int client_id;

	switch (mapid) {
	case MHI_L3_PHONET:
	case MHI_L3_FILE:
	case MHI_L3_SECURITY:
	case MHI_L3_TEST:
	case MHI_L3_MED_PRIO_TEST:
		/* Medium prio */
		client_id = HSI_LOGICAL_CLIENT_PRIO_MEDIUM;
	break;

	case MHI_L3_XFILE:
	case MHI_L3_MHDP_DL:
	case MHI_L3_MHDP_UL:
	case MHI_L3_LOW_PRIO_TEST:
		/* Low prio */
		client_id = HSI_LOGICAL_CLIENT_PRIO_LOW;
	break;

	case MHI_L3_AUDIO:
	case MHI_L3_TEST_PRIO:
	case MHI_L3_HIGH_PRIO_TEST:
		/*high prio*/
		client_id = HSI_LOGICAL_CLIENT_PRIO_HIGH;

	break;

	default:
		if ((mapid >= 128) && (mapid <= 191)) {
			/* High prio */
			client_id = HSI_LOGICAL_CLIENT_PRIO_HIGH;

		} else if ((mapid >= 192) && (mapid <= 255)) {
			/* Medium prio */
			client_id = HSI_LOGICAL_CLIENT_PRIO_MEDIUM;
		} else {
			/* Undefined for now */
			client_id = HSI_LOGICAL_CLIENT_PRIO_LOW;
		}
	break;
	}

	return client_id;
}
#endif


static int
hsi_logical_retrieve_data_client(
	struct hsi_protocol_client *hsi_client_context,
	unsigned char channel_id,
	struct hsi_protocol_client **hsi_data_client)
{
	if (unlikely(channel_id > HSI_MAX_CHANNEL_ID)) {
		/* Should never happen */
		DPRINTK("hsi_logical_retrieve_data_client" \
			" - ERROR: wrong channel ID:  %d !!\n", channel_id);

		goto out;
	}

	*hsi_data_client =
		hsi_client_context->
			hsi_top_protocol_context->
				cl[HSI_LOGICAL_GET_PRIO_ACCORDING_TO_CHANNEL(channel_id)];

	if (unlikely(!hsi_data_client)) {
		/* Should never happen */
		DPRINTK("hsi_logical_retrieve_data_client - ERROR: Client" \
				"is not allocated (channel ID:  %d) !!\n",
			channel_id);

		goto out;
	}

	return 0;

out:
	return -1;
}

static void hsi_logical_skb_to_msg(struct sk_buff *skb, struct hsi_msg *msg)
{
	skb_frag_t *frag;
	struct scatterlist *sg;
	int i;

	BUG_ON(msg->sgt.nents !=
		(unsigned int)(skb_shinfo(skb)->nr_frags + 1));

	sg = msg->sgt.sgl;
	sg_set_buf(sg, skb->data, skb_headlen(skb));
	for (i = 0; i < skb_shinfo(skb)->nr_frags; i++) {
		sg = sg_next(sg);
		BUG_ON(!sg);
		frag = &skb_shinfo(skb)->frags[i];
		sg_set_page(sg, frag->page.p, frag->size, frag->page_offset);
	}
}

#ifdef HSI_V3_USED

static void
hsi_logical_power_off_complete(struct hsi_msg *msg)
{

	struct hsi_protocol_client *hsi_control_client =
		hsi_client_drvdata(msg->cl);

	DPRINTK("hsi_logical_power_off_complete - client ID %d" \
		" - port nb %d - channel_id %d\n",
		hsi_control_client->client_id,
		(hsi_get_port(msg->cl))->num,
		msg->channel);

	hsi_logical_do_power_sequence(
		hsi_control_client->hsi_top_protocol_context,
		HSI_POWER_OFF_STEP_2);

	/* Free control message */
	hsi_logical_tx_control_destructor(msg);
}


void
hsi_logical_send_power_off(struct hsi_protocol *hsi_protocol_context)
{
	DPRINTK("hsi_logical_send_power_off");
	hsi_logical_alloc_msg_and_send_on_control_channel(
			hsi_protocol_context->cl[HSI_CONTROL_CLIENT_ID],
			HSI_POWER_OFF<<24,
			HSI_CONTROL_CHANNEL,
			hsi_logical_power_off_complete);


}

void
hsi_logical_do_power_sequence(
		struct hsi_protocol *hsi_protocol_context,
		int hsi_power_state)
{

	DPRINTK("hsi_logical_do_power_sequence step %d", hsi_power_state);

	switch (hsi_power_state) {

	case HSI_POWER_OFF_STEP_1:
		hsi_protocol_context->power_state = HSI_POWER_SEND_OFF_STATE;
		hsi_logical_send_power_off(hsi_protocol_context);
	break;

	case HSI_POWER_OFF_STEP_2:
		hsi_logical_stop_tx(hsi_protocol_context);
	break;

	default:
	break;

	}

}

#endif /*HSI_V3_USED*/


static void
hsi_logical_inactivity_timer(unsigned long data)
{
	struct hsi_protocol *hsi_protocol_context =
		(struct hsi_protocol *)data;
	int i;
	int one_client_up = 0;

	DPRINTK("hsi_logical_inactivity_timer");

	spin_lock_bh(&hsi_protocol_context->tx_activity_status_lock);
	{
		hsi_protocol_context->inactivity_timer_running = false;

		for (i = 1; i < HSI_NB_CLIENT; i++) {
			if (__netif_subqueue_stopped(
				hsi_protocol_context->netdev,
				HSI_LOG_CL_ID_TO_QUEUE_ID(i))) {
				one_client_up++;
				break;
			}
		}
		if ((!one_client_up) &&
			(hsi_protocol_context->main_state != HSI_CONFIG_EXCHANGE_NO_DONE)) {

#ifdef HSI_V3_USED
			/*start the power off sequence*/
			hsi_logical_do_power_sequence(
				hsi_protocol_context,
				HSI_POWER_OFF_STEP_1);

#else

			DPRINTK("hsi_logical_inactivity_timer expires." \
				"No client want to transmit => Stop TX\n");
			hsi_logical_stop_tx(hsi_protocol_context);
#endif /*HSI_V3_USED*/

#ifdef HSI_LOGICAL_POWER_SAVING
			wake_unlock(&wake_temp);
#endif /*HSI_LOGICAL_POWER_SAVING*/

		}
#ifdef HSI_LOGICAL_USE_DEBUG
		else {
			DPRINTK("hsi_logical_inactivity_timer expires." \
				"Don't stop TX (client want to transmit)\n");
		}
#endif
	}
	spin_unlock_bh(&hsi_protocol_context->tx_activity_status_lock);
}


static int __init hsi_logical_init(void)
{
	DPRINTK("hsi_logical_init\n");
	wake_lock_init(&wake_resoft, WAKE_LOCK_SUSPEND, "HSI_modem_upload");
	wake_lock_init(&wake_no_suspend_l2header, WAKE_LOCK_SUSPEND,
			"HSI_L2_header_no_suspend");
	wake_lock_init(&wake_temp, WAKE_LOCK_SUSPEND,
			"HSI_prevent_system_power");

#ifndef HSI_LOGICAL_POWER_SAVING
	/*DEFINITIVELY TAKE WAKE LOCK*/
	wake_lock(&wake_temp);
#endif /*HSI_LOGICAL_POWER_SAVING*/

	init_spinlock = 1;

	return 0;
}
module_init(hsi_logical_init);

static void __exit hsi_logical_exit(void)
{
	DPRINTK("hsi_logical_exit\n");
}
module_exit(hsi_logical_exit);


MODULE_ALIAS("hsi_logical");
MODULE_AUTHOR("RMC");
MODULE_DESCRIPTION("HSI logical using RENESAS HSI protocol");
MODULE_LICENSE("GPL");
