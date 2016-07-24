/*
* File: hsi_logical.h
*
* hsi logical header file
*
* Copyright (C) 2011 Renesas Mobile Corporation. All rights reserved.
*
* This program is free software; you can redistribute it and/or
* modify it under the terms of the GNU General Public License
* version 2 as published by the Free Software Foundation.
*
* This program is distributed in the hope that it will be useful, but
* WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. Â See the GNU
* General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with this program; if not, write to the Free Software
* Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
* 02110-1301 USA
*
 */


#include <linux/kernel.h>
#include <linux/workqueue.h>
#include <linux/skbuff.h>
#include <linux/platform_device.h>
#include <linux/l2mux.h>

#define HSI_V3_USED
/*#define HSI_V2_USED */
#define HSI_USE_SEND_SCHEDULED
#define HSI_USE_RCV_SCHEDULED

#if (CONFIG_HSI_RMC_VERSION == 2)
#define HSI_V2_USED
#elif (CONFIG_HSI_RMC_VERSION == 3)
#define HSI_V3_USED
#endif

#define HSI_LOGICAL_POWER_SAVING

#define HSI_LOGICAL_USE_DEBUG
# define EPRINTK(...)    printk(KERN_EMERG __VA_ARGS__)


#define HSI_NB_CLIENT 4

#define HSI_LOGICAL_QUEUE_PRIO_LOW 0
#define HSI_LOGICAL_QUEUE_PRIO_MEDIUM 1
#define HSI_LOGICAL_QUEUE_PRIO_HIGH 2

#define HSI_LOGICAL_CLIENT_PRIO_HIGH 1
#define HSI_LOGICAL_CLIENT_PRIO_MEDIUM 2
#define HSI_LOGICAL_CLIENT_PRIO_LOW 3

#define HSI_CONTROL_CHANNEL 0

#define HSI_CONTROL_CLIENT_ID 0

#define HSI_MAX_CHANNEL_ID 15

#define HSI_LOGICAL_CONTROL_MESSAGE_SIZE 4
#define HSI_LOGICAL_L2HEADER_MESSAGE_SIZE 128

#define HSI_LOGICAL_INACTIVITY_TIMER 10

/*#define HSI_LOGICAL_SEND_ACK_BEFORE_ALLOC*/
#define HSI_LOGICAL_RXDATA_128_BYTES_PADDED
/*Control messages definition*/
#define HSI_BOOT_INFO_REQ         0x10
#define HSI_BOOT_INFO_RESP        0x20
#define HSI_ACK                   0x50
#define HSI_NACK                  0x60
#define HSI_POWER_OFF             0x70

#define CONF_VERSION_MASK 0x000000FF

#define LENGTH_IN_HEADER(HEADER) (((u32)HEADER) & 0x00FFFFFF)

#define MAPID_FROM_HEADER(HEADER) ((u8)((HEADER & 0xFF000000) >> 24))

#define CHANNEL_FROM_ACK(ACK) ((u8)ACK & 0x000000FF)


#define HSI_LOGICAL_SET_CHANNEL_ACCORDING_TO_PRIO(CLIENT_ID) (CLIENT_ID)
#define HSI_LOGICAL_GET_PRIO_ACCORDING_TO_CHANNEL(CHANNEL) (CHANNEL)


/* As HSI LOGICAL client ID 0 is HSI control client
	=> Associated physical bank can't be accessed by client
	=> No associated queue */
#define HSI_LOG_CL_ID_TO_QUEUE_ID(CLIENT_ID) (HSI_NB_CLIENT - 1 - CLIENT_ID)
#define HSI_LOG_QUEUE_ID_TO_CL_ID(QUEUE_ID) (HSI_NB_CLIENT - 1 - QUEUE_ID)

/* Retrieve from modem: */

#define MAXIMUM_MSG_LENGTH_ALLOWED 0x8000

/** @def HSI_DEFAULT_BOOT_CONFIG
*   @brief Default boot configuration:
*       break length = 37,
*       trans mode = frame (1),
*       CHID size = 2bits,
*       Rx max channels = 3 channels can be received in parallel (2bits used)
*       Audio option is set in this release (hard coded bit 11)
*       version = 2
*             */

#ifdef HSI_V2_USED
#define HSI_DEFAULT_BOOT_CONFIG 0x002A3002 /* audio not set */
#define HSI_SUPPORTED_VERSION 2
#endif /*HSI_V2_USED*/
#ifdef HSI_V3_USED
#define HSI_DEFAULT_BOOT_CONFIG 0x000B3003 /* audio not set */
#define HSI_SUPPORTED_VERSION 3
#endif /*HSI_V3_USED*/


enum {
	HSI_CONFIG_EXCHANGE_NO_DONE,
	HSI_CONFIG_EXCHANGE_DONE
};


enum HSI_TX_CHANNEL_STATE {
	HSI_TX_CLOSE,
	HSI_TX_IDLE,
	HSI_TX_WAIT,
	HSI_TX_HDR_SENT,
	HSI_TX_ACK_RCV
};


enum HSI_RX_BUFFER_STR {
	HSI_RX_CLOSE,
	HSI_RX_IDLE,
	HSI_RX_RVCING_DATA,
	HSI_RX_HDR_RCV_WHILE_DATA_ONGOING
};

enum HSI_POWER_STATE {
	HSI_POWER_ON_STATE,
	HSI_POWER_SEND_OFF_STATE,
	HSI_POWER_OFF_STATE
};

enum HSI_POWER_STEP {
	HSI_POWER_OFF_STEP_1,
	HSI_POWER_OFF_STEP_2
};


enum hsi_logical_trace_states {
	HSI_ALL = 0,
	HSI_HIGH,
	HSI_HIGH_RX,
	HSI_HIGH_TX,
	HSI_OFF
};

/* Prototype of receive callback function called by HSI logical */
typedef void (*l3if_receive_t) (struct sk_buff *p_l3_info);

struct hsi_protocol_client {
	unsigned int client_id;
	spinlock_t tx_lock;
	unsigned int send_state;
	spinlock_t rx_lock;
	unsigned int recv_state;
	struct hsi_client *hsi_cl;
	/* usefull for data clients
		(this message will be send when ACK is received) */
	struct hsi_msg *data_msg_to_be_send;
	/* usefull when NACK is received */
	unsigned int nb_time_l2_header_sent;
	/* used for TX client */
	u32 tx_l2_header;
	u32 rx_l2_header;
	/* Timer to wait for memory */
	struct timer_list rx_mem;
	/* used for RX client when receiving L2 header
		while previous receiving data not completed */
	u32 pending_rx_l2_header;
	unsigned char rx_l2_header_channel;
	unsigned char pending_rx_l2_header_channel;
	/* pointer of top level context */
	struct hsi_protocol *hsi_top_protocol_context;
#ifdef HSI_USE_SEND_SCHEDULED
	struct work_struct send_work;
	struct workqueue_struct *send_wq;
#endif /* HSI_USE_SEND_SCHEDULED */
#ifdef HSI_USE_RCV_SCHEDULED
	struct workqueue_struct *rcv_wq;
	struct work_struct rcv_work;
	struct list_head rcv_msgs;
	spinlock_t rcv_msgs_lock;
#endif /* HSI_USE_RCV_SCHEDULED */
	int gpio_pwr_on;
	int gpio_rst_out;
};

struct config_exchange {
	u8 tx_boot_info_req_complete;
	u8 rx_boot_info_resp_complete;
	u8 tx_boot_info_resp_complete;
	u32 sent_ape_config;
	u32 received_modem_config;
	struct timer_list to;	/* Timer to wait for initial config */
};

struct hsi_msg_array {
	unsigned int free; /* 0 => busy; 1=> free */
	struct hsi_msg *msg;
};

#define HSI_LOGICAL_NB_CONTROL_MSG (HSI_NB_CLIENT * 7)
#define HSI_LOGICAL_NB_CONTROL_MSG_L2H (HSI_NB_CLIENT * 2)

struct hsi_protocol {
	unsigned int main_state;
	struct timer_list inactivity_timer;
	bool inactivity_timer_running;
	spinlock_t tx_activity_status_lock;
	int wake_state;
	struct config_exchange cfg_ex;
	l3if_receive_t receive_fn;
	struct net_device *netdev;
	int nb_client;
	int power_state;
	struct hsi_protocol_client *cl[HSI_NB_CLIENT];
	spinlock_t ctrl_msg_array_lock;
	struct hsi_msg_array  hsi_ctrl_msg_array[HSI_LOGICAL_NB_CONTROL_MSG];
	spinlock_t ctrl_msg_array_l2h_lock;
	struct hsi_msg_array  hsi_ctrl_msg_array_l2h[HSI_LOGICAL_NB_CONTROL_MSG_L2H];
};

/**** HSI LOGICAL INTERFACES FOR L3 CLIENTS ****/
int hsi_logical_open(struct hsi_protocol *hsi_protocol_ctx);
int hsi_logical_close(struct hsi_protocol *hsi_protocol_ctx);
int hsi_logical_send(struct hsi_protocol *hsi_protocol_ctx,
			struct sk_buff *skb,
			u16 client_id);
#ifdef HSI_USE_SEND_SCHEDULED
void hsi_logical_send_work(struct work_struct *work);
#endif /*#ifdef HSI_USE_SEND_SCHEDULED*/

#ifdef HSI_USE_RCV_SCHEDULED
void hsi_logical_rcv_work(struct work_struct *work);
#endif /*#ifdef HSI_USE_RCV_SCHEDULED*/


