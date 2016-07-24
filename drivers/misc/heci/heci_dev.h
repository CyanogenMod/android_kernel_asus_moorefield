/*
 * Most HECI provider device and HECI logic declarations
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
 */

#ifndef _HECI_DEV_H_
#define _HECI_DEV_H_

/* Debug mutex locking/unlocking */
#define	DEBUG_LOCK	0

#ifdef DEBUG_LOCK

static void do_mutex_lock(void *m)
{
	mutex_lock(m);
}

static void do_mutex_unlock(void *m)
{
	mutex_unlock(m);
}

#ifdef mutex_lock
#undef mutex_lock
#endif
#ifdef mutex_unlock
#undef mutex_unlock
#endif

#define mutex_lock(a) \
	do {	\
		pr_alert("%s:%d[%s] -- mutex_lock(%p)\n", __FILE__, __LINE__, __func__, a);	\
		do_mutex_lock(a);	\
	} while (0)

#define mutex_unlock(a) \
	do {	\
		pr_alert("%s:%d[%s] -- mutex_unlock(%p)\n", __FILE__, __LINE__, __func__, a);	\
		do_mutex_unlock(a);	\
	} while (0)
#endif /* DEBUG_LOCK */
/*************************************/

#include <linux/types.h>
#include <linux/watchdog.h>
#include <linux/poll.h>
#include "heci.h"
#include "heci_cl_bus.h"
#include "hw.h"
#include "hbm.h"
#include "platform-config.h"

#define HECI_RD_MSG_BUF_SIZE           (128 * sizeof(u32))

/*
 * Number of Maximum HECI Clients
 */
#define HECI_CLIENTS_MAX 256

/*
 * Number of File descriptors/handles
 * that can be opened to the driver.
 *
 * Limit to 255: 256 Total Clients
 * minus internal client for HECI Bus Messags
 */
#define  HECI_MAX_OPEN_HANDLE_COUNT (HECI_CLIENTS_MAX - 1)

/*
 * Internal Clients Number
 */
#define HECI_HOST_CLIENT_ID_ANY        (-1)
#define HECI_HBM_HOST_CLIENT_ID         0 /* not used, just for documentation */

/* File state */
enum file_state {
	HECI_FILE_INITIALIZING = 0,
	HECI_FILE_CONNECTING,
	HECI_FILE_CONNECTED,
	HECI_FILE_DISCONNECTING,
	HECI_FILE_DISCONNECTED
};

/* HECI device states */
enum heci_dev_state {
	HECI_DEV_INITIALIZING = 0,
	HECI_DEV_INIT_CLIENTS,
	HECI_DEV_ENABLED,
	HECI_DEV_RESETTING,
	HECI_DEV_DISABLED,
	HECI_DEV_POWER_DOWN,
	HECI_DEV_POWER_UP
};

const char *heci_dev_state_str(int state);

enum heci_file_transaction_states {
	HECI_IDLE,
	HECI_WRITING,
	HECI_WRITE_COMPLETE,
	HECI_FLOW_CONTROL,
	HECI_READING,
	HECI_READ_COMPLETE
};

/**
 * enum heci_cb_file_ops  - file operation associated with the callback
 * @HECI_FOP_READ   - read
 * @HECI_FOP_WRITE  - write
 * @HECI_FOP_IOCTL  - ioctl
 * @HECI_FOP_OPEN   - open
 * @HECI_FOP_CLOSE  - close
 */
enum heci_cb_file_ops {
	HECI_FOP_READ = 0,
	HECI_FOP_WRITE,
	HECI_FOP_IOCTL,
	HECI_FOP_OPEN,
	HECI_FOP_CLOSE
};

/*
 * Intel HECI message data struct
 */
struct heci_msg_data {
	u32 size;
	unsigned char *data;
};

/**
 * struct heci_me_client - representation of me (fw) client
 *
 * @props  - client properties
 * @client_id - me client id
 * @heci_flow_ctrl_creds - flow control credits
 */
struct heci_me_client {
	struct heci_client_properties props;
	u8 client_id;
	u8 heci_flow_ctrl_creds;
};


struct heci_cl;

/**
 * struct heci_cl_cb - file operation callback structure
 *
 * @cl - file client who is running this operation
 * @fop_type - file operation type
 */
struct heci_cl_cb {
	struct list_head list;
	struct heci_cl *cl;
	enum heci_cb_file_ops fop_type;
	struct heci_msg_data request_buffer;
	struct heci_msg_data response_buffer;
	unsigned long buf_idx;
	unsigned long read_time;
	struct file *file_object;
};

/* HECI client instance carried as file->pirvate_data*/
struct heci_cl {
	struct list_head link;
	struct heci_device *dev;
	enum file_state state;
	wait_queue_head_t tx_wait;
	wait_queue_head_t rx_wait;
	wait_queue_head_t wait;
	int status;
	/* ID of client connected */
	u8 host_client_id;
	u8 me_client_id;
	u8 heci_flow_ctrl_creds;
	u8 timer_count;
	enum heci_file_transaction_states reading_state;
	enum heci_file_transaction_states writing_state;
	struct heci_cl_cb *read_cb;

	/* HECI CL bus data */
	struct heci_cl_device *device;
	struct list_head device_link;
	uuid_le device_uuid;
};

/** struct heci_hw_ops
 *
 * @host_is_ready    - query for host readiness

 * @hw_is_ready      - query if hw is ready
 * @hw_reset         - reset hw
 * @hw_start         - start hw after reset
 * @hw_config        - configure hw

 * @intr_clear       - clear pending interrupts
 * @intr_enable      - enable interrupts
 * @intr_disable     - disable interrupts

 * @hbuf_free_slots  - query for write buffer empty slots
 * @hbuf_is_ready    - query if write buffer is empty
 * @hbuf_max_len     - query for write buffer max len

 * @write            - write a message to FW

 * @rdbuf_full_slots - query how many slots are filled

 * @read_hdr         - get first 4 bytes (header)
 * @read             - read a buffer from the FW
 */
struct heci_hw_ops {

	bool (*host_is_ready) (struct heci_device *dev);

	bool (*hw_is_ready) (struct heci_device *dev);
	int (*hw_reset) (struct heci_device *dev, bool enable);
	int  (*hw_start) (struct heci_device *dev);
	void (*hw_config) (struct heci_device *dev);

	void (*intr_clear) (struct heci_device *dev);
	void (*intr_enable) (struct heci_device *dev);
	void (*intr_disable) (struct heci_device *dev);

	int (*hbuf_free_slots) (struct heci_device *dev);
	bool (*hbuf_is_ready) (struct heci_device *dev);
	size_t (*hbuf_max_len) (const struct heci_device *dev);

	int (*write)(struct heci_device *dev,
		     struct heci_msg_hdr *hdr,
		     unsigned char *buf);

	int (*rdbuf_full_slots)(struct heci_device *dev);

	u32 (*read_hdr)(const struct heci_device *dev);
	int (*read) (struct heci_device *dev,
		     unsigned char *buf, unsigned long len);
};

/* HECI bus API*/

/**
 * struct heci_cl_ops - HECI CL device ops
 * This structure allows ME host clients to implement technology
 * specific operations.
 *
 * @enable: Enable an HECI CL device. Some devices require specific
 *	HECI commands to initialize completely.
 * @disable: Disable an HECI CL device.
 * @send: Tx hook for the device. This allows ME host clients to trap
 *	the device driver buffers before actually physically
 *	pushing it to the ME.
 * @recv: Rx hook for the device. This allows ME host clients to trap the
 *	ME buffers before forwarding them to the device driver.
 */
struct heci_cl_ops {
	int (*enable)(struct heci_cl_device *device);
	int (*disable)(struct heci_cl_device *device);
	int (*send)(struct heci_cl_device *device, u8 *buf, size_t length);
	int (*recv)(struct heci_cl_device *device, u8 *buf, size_t length);
};

struct heci_cl_device *heci_cl_add_device(struct heci_device *dev,
					uuid_le uuid, char *name,
					struct heci_cl_ops *ops);
void heci_cl_remove_device(struct heci_cl_device *device);

int __heci_cl_async_send(struct heci_cl *cl, u8 *buf, size_t length);
int __heci_cl_send(struct heci_cl *cl, u8 *buf, size_t length);
int __heci_cl_recv(struct heci_cl *cl, u8 *buf, size_t length);
void heci_cl_bus_rx_event(struct heci_cl *cl);
int heci_cl_bus_init(void);
void heci_cl_bus_exit(void);
int     heci_bus_new_client(struct heci_device *dev);
void	heci_remove_all_clients(struct heci_device *dev);

/**
 * struct heci_cl_device - HECI device handle
 * An heci_cl_device pointer is returned from heci_add_device()
 * and links HECI bus clients to their actual ME host client pointer.
 * Drivers for HECI devices will get an heci_cl_device pointer
 * when being probed and shall use it for doing ME bus I/O.
 *
 * @dev: linux driver model device pointer
 * @uuid: me client uuid
 * @cl: heci client
 * @ops: ME transport ops
 * @event_cb: Drivers register this callback to get asynchronous ME
 *	events (e.g. Rx buffer pending) notifications.
 * @events: Events bitmask sent to the driver.
 * @priv_data: client private data
 */
struct heci_cl_device {
	struct device dev;

	struct heci_cl *cl;

	const struct heci_cl_ops *ops;

	struct work_struct event_work;
	heci_cl_event_cb_t event_cb;
	void *event_context;
	unsigned long events;

	void *priv_data;
};

/**
 * struct heci_device -  HECI private device struct

 * @hbm_state - state of host bus message protocol
 * @support_rpm - support runtime power managment
 * @mem_addr - mem mapped base register address

 * @hbuf_depth - depth of hardware host/write buffer is slots
 * @hbuf_is_ready - query if the host host/write buffer is ready
 * @wr_msg - the buffer for hbm control messages
 * @wr_ext_msg - the buffer for hbm control responses (set in read cycle)
 */
struct heci_device {
	struct pci_dev *pdev;	/* pointer to pci device struct */
	/*
	 * lists of queues
	 */
	/* array of pointers to aio lists */
	struct heci_cl_cb read_list;		/* driver read queue */
	struct heci_cl_cb write_list;		/* driver write queue */
	struct heci_cl_cb write_waiting_list;	/* write waiting queue */
	struct heci_cl_cb ctrl_wr_list;		/* managed write IOCTL list */
	struct heci_cl_cb ctrl_rd_list;		/* managed read IOCTL list */

	/*
	 * list of files
	 */
	struct list_head file_list;
	long open_handle_count;

	/*
	 * lock for the device
	 */
	struct mutex device_lock; /* device lock */
	struct delayed_work timer_work;	/* HECI timer delayed work (timeouts) */

	bool recvd_hw_ready;
	/*
	 * waiting queue for receive message from FW
	 */
	wait_queue_head_t wait_hw_ready;
	wait_queue_head_t wait_recvd_msg;
	wait_queue_head_t wait_dma_ready;

	/*
	 * heci device  states
	 */
	enum heci_dev_state dev_state;
	enum heci_hbm_state hbm_state;
	u16 init_clients_timer;
	bool support_rpm;

	unsigned char rd_msg_buf[HECI_RD_MSG_BUF_SIZE];	/* control messages */
	u32 rd_msg_hdr;

	/* write buffer */
	u8 hbuf_depth;
	bool hbuf_is_ready;

	/* used for control messages */
	struct {
		struct heci_msg_hdr hdr;
		unsigned char data[128];
	} wr_msg;

	struct {
		struct heci_msg_hdr hdr;
		unsigned char data[4];	/* All HBM messages are 4 bytes */
	} wr_ext_msg;		/* for control responses */

	struct hbm_version version;

	struct heci_me_client *me_clients; /* Note: memory has to be allocated */
	DECLARE_BITMAP(me_clients_map, HECI_CLIENTS_MAX);
	DECLARE_BITMAP(host_clients_map, HECI_CLIENTS_MAX);
	u8 me_clients_num;
	u8 me_client_presentation_num;
	u8 me_client_index;

	struct work_struct init_work;

	/* List of bus devices */
	struct list_head device_list;

#if IS_ENABLED(CONFIG_DEBUG_FS)
	struct dentry *dbgfs_dir;
#endif /* CONFIG_DEBUG_FS */


	const struct heci_hw_ops *ops;
	char hw[0] __aligned(sizeof(void *));
};

static inline unsigned long heci_secs_to_jiffies(unsigned long sec)
{
	return sec * HZ;	/*msecs_to_jiffies(sec * MSEC_PER_SEC);*/
}

/**
 * heci_data2slots - get slots - number of (dwords) from a message length
 *	+ size of the heci header
 * @length - size of the messages in bytes
 * returns  - number of slots
 */
static inline u32 heci_data2slots(size_t length)
{
	return DIV_ROUND_UP(sizeof(struct heci_msg_hdr) + length, 4);
}

/*
 * heci init function prototypes
 */
void heci_device_init(struct heci_device *dev);
void heci_reset(struct heci_device *dev, int interrupts);
int heci_start(struct heci_device *dev);
void heci_stop(struct heci_device *dev);

/*
 *  HECI interrupt functions prototype
 */

void heci_timer(struct work_struct *work);
int heci_irq_read_handler(struct heci_device *dev,
		struct heci_cl_cb *cmpl_list, s32 *slots);

int heci_irq_write_handler(struct heci_device *dev, struct heci_cl_cb *cmpl_list);
void heci_irq_compl_handler(struct heci_device *dev, struct heci_cl_cb *cmpl_list);

/*
 * Register Access Function
 */

static inline void heci_hw_config(struct heci_device *dev)
{
	dev->ops->hw_config(dev);
}
static inline int heci_hw_reset(struct heci_device *dev, bool enable)
{
	return dev->ops->hw_reset(dev, enable);
}

static inline int heci_hw_start(struct heci_device *dev)
{
	return dev->ops->hw_start(dev);
}

static inline void heci_clear_interrupts(struct heci_device *dev)
{
	dev->ops->intr_clear(dev);
}

static inline void heci_enable_interrupts(struct heci_device *dev)
{
	dev->ops->intr_enable(dev);
}

static inline void heci_disable_interrupts(struct heci_device *dev)
{
	dev->ops->intr_disable(dev);
}

static inline bool heci_host_is_ready(struct heci_device *dev)
{
	return dev->ops->host_is_ready(dev);
}
static inline bool heci_hw_is_ready(struct heci_device *dev)
{
	return dev->ops->hw_is_ready(dev);
}

static inline bool heci_hbuf_is_ready(struct heci_device *dev)
{
	return dev->ops->hbuf_is_ready(dev);
}

static inline int heci_hbuf_empty_slots(struct heci_device *dev)
{
	return dev->ops->hbuf_free_slots(dev);
}

static inline size_t heci_hbuf_max_len(const struct heci_device *dev)
{
	return dev->ops->hbuf_max_len(dev);
}

static inline int heci_write_message(struct heci_device *dev,
			struct heci_msg_hdr *hdr,
			unsigned char *buf)
{
	return dev->ops->write(dev, hdr, buf);
}

static inline u32 heci_read_hdr(const struct heci_device *dev)
{
	return dev->ops->read_hdr(dev);
}

static inline void heci_read_slots(struct heci_device *dev,
		     unsigned char *buf, unsigned long len)
{
	dev->ops->read(dev, buf, len);
}

static inline int heci_count_full_read_slots(struct heci_device *dev)
{
	return dev->ops->rdbuf_full_slots(dev);
}

bool heci_write_is_idle(struct heci_device *dev);

#if IS_ENABLED(CONFIG_DEBUG_FS)
int heci_dbgfs_register(struct heci_device *dev, const char *name);
void heci_dbgfs_deregister(struct heci_device *dev);
#else
static inline int heci_dbgfs_register(struct heci_device *dev, const char *name)
{
	return 0;
}
static inline void heci_dbgfs_deregister(struct heci_device *dev) {}
#endif /* CONFIG_DEBUG_FS */


int heci_register(struct heci_device *dev);
void heci_deregister(struct heci_device *dev);

void    heci_bus_remove_all_clients(struct heci_device *heci_dev);

#define HECI_HDR_FMT "hdr:host=%02d me=%02d len=%d comp=%1d"
#define HECI_HDR_PRM(hdr)                  \
	(hdr)->host_addr, (hdr)->me_addr, \
	(hdr)->length, (hdr)->msg_complete

#endif
