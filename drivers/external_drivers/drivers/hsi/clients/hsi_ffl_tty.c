/*
 * hsi_ffl_tty.c
 *
 * Fixed frame length protocol over HSI, implements a TTY interface for
 * this protocol.
 *
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
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
#include <linux/hsi/hsi_ffl_tty.h>
#include <linux/hsi/hsi.h>
#include <linux/dma-mapping.h>
#include <linux/wait.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/version.h>

#define DRVNAME				"hsi-ffl"
#define TTYNAME				"tty"CONFIG_HSI_FFL_TTY_NAME

/* Set the following to use the IPC error recovery mechanism */
#undef USE_IPC_ERROR_RECOVERY

/* Set the following to use the WAKE post boot handshake */
#define USE_WAKE_POST_BOOT_HANDSHAKE

/* Set the following to ignore the TTY_THROTTLE bit */
#define IGNORE_TTY_THROTTLE

/* Maximal number of TTY lines supported by this driver */
#define FFL_TTY_MAX_LINES		8

/* Maximal number of frame allocation failure prior firing an error message */
#define FFL_FRAME_ALLOC_RETRY_MAX_CNT	10

/* Defaut TX delay expressed in microseconds */
#define FFL_TX_DELAY		10000

/* Defaut RX delay expressed in microseconds */
#define FFL_RX_DELAY		100000

/* Defaut TX timeout delay expressed in microseconds */
#define TTY_HANGUP_DELAY	500000

/* Delays for powering up/resetting the modem, ms */
#define PO_INTERLINE_DELAY	1
#define PO_POST_DELAY		200

/* ACWAKE minimal pulse udelay in us (set to 0 if none is necessary) */
#define ACWAKE_MINIMAL_PULSE_UDELAY	800

/* ACWAKE to DATA transmission minimal delay in us (0 if none is necessary) */
#define ACWAKE_TO_DATA_MINIMAL_UDELAY	100

/* Initial minimal buffering size (in bytes) */
#define FFL_MIN_TX_BUFFERING	65536
#define FFL_MIN_RX_BUFFERING	65536

/* Modem post book wake handshake timeout value (10 seconds) */
#define POST_BOOT_HANDSHAKE_TIMEOUT_JIFFIES	(msecs_to_jiffies(60000))

/* Error recovery related timings */
#define RECOVERY_TO_NORMAL_DELAY_JIFFIES	(usecs_to_jiffies(100000))
#define RECOVERY_TX_DRAIN_TIMEOUT_JIFFIES	(usecs_to_jiffies(100000))
#define RECOVERY_BREAK_RESPONSE_TIMEOUT_JIFFIES	(usecs_to_jiffies(100000))

/* Round-up the frame and header length to a multiple of 4-bytes to align
 * on the HSI 4-byte granularity*/
#define FFL_FRAME_LENGTH (((CONFIG_HSI_FFL_TTY_FRAME_LENGTH+3)/4)*4)
#define FFL_HEADER_LENGTH (((CONFIG_HSI_FFL_TTY_HEADER_LENGTH+3)/4)*4)
#define FFL_DATA_LENGTH (FFL_FRAME_LENGTH-FFL_HEADER_LENGTH)
#define FFL_LENGTH_MASK (roundup_pow_of_two(FFL_DATA_LENGTH)-1)

/* Find the best allocation method */
#if ((FFL_FRAME_LENGTH >= PAGE_SIZE) && \
	(((FFL_FRAME_LENGTH) & (FFL_FRAME_LENGTH-1)) == 0))
#define FFL_FRAME_ALLOC_PAGES
#define FFL_FRAME_ALLOC_ORDER	(ilog2(FFL_FRAME_LENGTH/PAGE_SIZE))
#endif

/* Compute the TX and RX, FIFO depth from the buffering requirements */
/* For optimal performances the FFL_TX_CTRL_FIFO size shall be set to 2 at
 * least to allow back-to-back transfers. */
#define FFL_TX_ALL_FIFO		\
	((FFL_MIN_TX_BUFFERING+FFL_DATA_LENGTH-1)/FFL_DATA_LENGTH)
#define FFL_TX_CTRL_FIFO	2
#define FFL_TX_WAIT_FIFO	max(FFL_TX_ALL_FIFO-FFL_TX_CTRL_FIFO, 1)
#define FFL_RX_ALL_FIFO		\
	((FFL_MIN_RX_BUFFERING+FFL_DATA_LENGTH-1)/FFL_DATA_LENGTH)
#define FFL_RX_WAIT_FIFO	max(FFL_RX_ALL_FIFO/2, 1)
#define FFL_RX_CTRL_FIFO	max(FFL_RX_ALL_FIFO-FFL_RX_WAIT_FIFO, 1)

/* Tag for detecting buggy frame sizes (must be greater than the maximum frame
 * size */
#define FFL_BUGGY_FRAME_SIZE	0xFFFFFFFFUL

/* RX and TX state machine definitions */
enum {
	IDLE,
	READY,
	ACTIVE,
};

#define FFL_GLOBAL_STATE_SZ		2
#define FFL_GLOBAL_STATE_MASK		((1<<FFL_GLOBAL_STATE_SZ)-1)

#define TX_TTY_WRITE_ONGOING_BIT	(1<<FFL_GLOBAL_STATE_SZ)
#define TX_TTY_WRITE_PENDING_BIT	(2<<FFL_GLOBAL_STATE_SZ)
#define TX_TTY_WRITE_FORWARDING_BIT	(4<<FFL_GLOBAL_STATE_SZ)

/* Flags for the error recovery / shutdown mechanisms */
#define ERROR_RECOVERY_RX_ERROR_BIT	(0x10 << FFL_GLOBAL_STATE_SZ)
#define ERROR_RECOVERY_TX_DRAINED_BIT	(0x10 << FFL_GLOBAL_STATE_SZ)
#define ERROR_RECOVERY_ONGOING_BIT	(0x20 << FFL_GLOBAL_STATE_SZ)
#define TTY_OFF_BIT			(0x40 << FFL_GLOBAL_STATE_SZ)

/* For modem cold boot */
#define V1P35CNT_W		0x0E0	/* PMIC register used to power off */
					/* the modem */
#define V1P35_OFF		4
#define V1P35_ON		6
#define COLD_BOOT_DELAY_OFF	20000	/* 20 ms (use of usleep_range) */
#define COLD_BOOT_DELAY_ON	10000	/* 10 ms (use of usleep_range) */

/* Forward declaration for ffl_xfer_ctx structure */
struct ffl_ctx;

/**
 * struct ffl_xfer_ctx - TX/RX transfer context
 * @wait_frames: head of the FIFO of TX/RX waiting frames
 * @recycled_frames: head of the FIFO of TX/RX recycled frames
 * @timer: context of the TX active timeout/RX TTY insert retry timer
 * @lock: spinlock to serialise access to the TX/RX context information
 * @delay: number of jiffies for the TX active timeout/RX TTY insert retry timer
 * @wait_len: current length of the TX/RX waiting frames FIFO
 * @ctrl_len: current count of TX/RX outstanding frames in the controller
 * @all_len: total count of TX/RX frames (including recycled ones)
 * @wait_max: maximal length of the TX/RX waiting frames FIFO
 * @ctrl_max: maximal count of outstanding TX/RX frames in the controller
 * @state: current TX/RX state (global and internal one)
 * @buffered: number of bytes currently buffered in the wait FIFO
 * @room: room available in the wait FIFO with a byte granularity
 * @main_ctx: reference to the main context
 * @data_len: current maximal size of a frame data payload in bytes
 * @channel: HSI channel to use
 * @increase_pool: context of the increase pool work queue
 * @data_sz: total size of actual transferred data
 * @frame_cnt: total number of transferred frames
 * @overflow_cnt: total number of transfer stalls due to FIFO full
 * @config: current updated HSI configuration
 */
struct ffl_xfer_ctx {
	struct list_head	wait_frames;
	struct list_head	recycled_frames;
	struct timer_list	timer;
	spinlock_t		lock;
	unsigned long		delay;
	int			wait_len;
	int			ctrl_len;
	int			all_len;
	int			wait_max;
	int			ctrl_max;
	unsigned int		state;
	unsigned int		buffered;
	unsigned int		room;
	struct ffl_ctx		*main_ctx;
	unsigned int		data_len;
	unsigned int		channel;
	struct work_struct	increase_pool;
#ifdef CONFIG_HSI_FFL_TTY_STATS
	unsigned long long	data_sz;
	unsigned int		frame_cnt;
	unsigned int		overflow_cnt;
#endif
	struct hsi_config	config;
};

#define main_ctx(ctx) (ctx->main_ctx)
#define xfer_ctx_is_tx_ctx(ctx) ((ctx) == &main_ctx(ctx)->tx)
#define xfer_ctx_is_rx_ctx(ctx) ((ctx) == &main_ctx(ctx)->rx)

/**
 * struct ffl_hangup_ctx - hangup context for the fixed frame length protocol
 * @tx_timeout: there was a tx timeout or not
 * @timer: the timer for the TX timeout
 * @work: the context of for the TX timeout work queue
 */
struct ffl_hangup_ctx {
	int			tx_timeout;
	struct timer_list	timer;
	struct work_struct	work;
};

#ifdef USE_IPC_ERROR_RECOVERY
/**
 * struct ffl_recovery_ctx - error recovery context for the FFL protocol
 * @tx_drained_event: TX drain complete event
 * @do_tx_drain: the context of the TX drain work queue
 * @tx_break: the HSI message used for TX break emission
 * @rx_drain_timer: the timer for scheduling a RX drain work
 * @do_rx_drain: the context of the RX drain work queue
 * @rx_break: the HSI message used for RX break reception
 */
struct ffl_recovery_ctx {
	wait_queue_head_t	tx_drained_event;
	struct work_struct	do_tx_drain;
	struct hsi_msg		tx_break;
	struct timer_list	rx_drain_timer;
	struct work_struct	do_rx_drain;
	struct hsi_msg		rx_break;
};
#endif

/**
 * struct ffl_ctx - fixed frame length protocol on HSI context
 * @client: reference to this HSI client
 * @controller: reference to the controller bound to this context
 * @index: the TTY index of this context
 * @tty_prt: TTY port structure
 * @tx_full_pipe_clean_event: event signalling that the full TX pipeline is
 *			      clean or about to be clean
 * @tx_write_pipe_clean_event: event signalling that the write part of the TX
 *			       pipeline is clean (no more write can occur)
 * @start_tx: work for making synchronous TX start request
 * @stop_tx: work for making synchronous TX stop request
 * @tx: current TX context
 * @rx: current RX context
 * @do_tty_forward: dedicated TTY forwarding work structure
 * @hangup: hangup context
 * @reset: modem reset context
 * @recovery: error recovery context
 * @modem_awake_event: modem WAKE post boot handshake event
 */
struct ffl_ctx {
	struct hsi_client	*client;
	struct device		*controller;
	int			index;
	struct tty_port		tty_prt;
	wait_queue_head_t	tx_full_pipe_clean_event;
	wait_queue_head_t	tx_write_pipe_clean_event;
	struct work_struct	start_tx;
	struct work_struct	stop_tx;
	struct ffl_xfer_ctx	tx;
	struct ffl_xfer_ctx	rx;
	struct work_struct	do_tty_forward;
	struct ffl_hangup_ctx	hangup;
#ifdef USE_IPC_ERROR_RECOVERY
	struct ffl_recovery_ctx	recovery;
#endif
#ifdef USE_WAKE_POST_BOOT_HANDSHAKE
	wait_queue_head_t	modem_awake_event;
#endif
};

/**
 * struct ffl_driver - fixed frame length protocol on HSI driver data
 * @tty_drv: TTY driver reference
 * @ctx: array of FFL contex references
 */
struct ffl_driver {
	struct tty_driver	*tty_drv;
	struct ffl_ctx		*ctx[FFL_TTY_MAX_LINES];
};

/*
 * Static protocol driver global variables
 */

/* Protocol driver instance */
static struct ffl_driver ffl_drv;

/* Workqueue for submitting tx background tasks */
static struct workqueue_struct *ffl_tx_wq;

/* Workqueue for submitting rx background tasks */
static struct workqueue_struct *ffl_rx_wq;

/* Workqueue for submitting hangup background tasks */
static struct workqueue_struct *ffl_hu_wq;

/*
 * State handling routines
 */

/**
 * _ffl_ctx_get_state - get the global state of a state machine
 * @ctx: a reference to the state machine context
 *
 * Returns the current state of the requested TX or RX context.
 */
static inline __must_check
unsigned int _ffl_ctx_get_state(struct ffl_xfer_ctx *ctx)
{
	return ctx->state & FFL_GLOBAL_STATE_MASK;
}

/**
 * ffl_ctx_get_state - get the global state of a state machine
 * @ctx: a reference to the state machine context
 *
 * Returns the current state of the requested TX or RX context.
 *
 * This version adds the spinlock guarding
 */
static inline __must_check
unsigned int ffl_ctx_get_state(struct ffl_xfer_ctx *ctx)
{
	unsigned int state;
	unsigned long flags;

	spin_lock_irqsave(&ctx->lock, flags);
	state = _ffl_ctx_get_state(ctx);
	spin_unlock_irqrestore(&ctx->lock, flags);

	return state;
}

/**
 * _ffl_ctx_is_state - checks the global state of a state machine
 * @ctx: a reference to the state machine context
 * @state: the state to consider
 *
 * Returns a non-zero value if in the requested state.
 */
static inline __must_check int _ffl_ctx_is_state(struct ffl_xfer_ctx *ctx,
						 unsigned int state)
{
#ifdef DEBUG
	BUG_ON(state & ~FFL_GLOBAL_STATE_MASK);
#endif

	return (_ffl_ctx_get_state(ctx) == state);
}

/**
 * _ffl_ctx_set_state - sets the global state of a state machine
 * @ctx: a reference to the state machine context
 * @state: the state to set
 */
static inline void _ffl_ctx_set_state(struct ffl_xfer_ctx *ctx,
				      unsigned int state)
{
#ifdef DEBUG
	BUG_ON(state & ~FFL_GLOBAL_STATE_MASK);
#endif

	ctx->state = (ctx->state & ~FFL_GLOBAL_STATE_MASK) | state;
}

/**
 * _ffl_ctx_has_flag - checks if a flag is present in the state
 * @ctx: a reference to the state machine context
 * @flag: the flag(s) to consider
 *
 * Returns a non-zero value if all requested flags are present.
 */
static inline __must_check int _ffl_ctx_has_flag(struct ffl_xfer_ctx *ctx,
						 unsigned int flag)
{
#ifdef DEBUG
	BUG_ON(flag & FFL_GLOBAL_STATE_MASK);
#endif

	return ((ctx->state & flag) == flag);
}

/**
 * _ffl_ctx_has_any_flag - checks if any flag is present in the state
 * @ctx: a reference to the state machine context
 * @flag: the flag(s) to consider
 *
 * Returns a non-zero value if all requested flags are present.
 */
static inline __must_check int _ffl_ctx_has_any_flag(struct ffl_xfer_ctx *ctx,
						     unsigned int flag)
{
#ifdef DEBUG
	BUG_ON(flag & FFL_GLOBAL_STATE_MASK);
#endif

	return ctx->state & flag;
}

/**
 * _ffl_ctx_set_flag - flags some extra information in the state
 * @ctx: a reference to the state machine context
 * @flag: the flag(s) to set
 */
static inline void _ffl_ctx_set_flag(struct ffl_xfer_ctx *ctx,
				     unsigned int flag)
{
#ifdef DEBUG
	BUG_ON(flag & FFL_GLOBAL_STATE_MASK);
#endif

	ctx->state |= flag;
}

/**
 * _ffl_ctx_clear_flag - unflags some extra information in the state
 * @ctx: a reference to the state machine context
 * @flag: the flag(s) to clear
 */
static inline void _ffl_ctx_clear_flag(struct ffl_xfer_ctx *ctx,
				       unsigned int flag)
{
#ifdef DEBUG
	BUG_ON(flag & FFL_GLOBAL_STATE_MASK);
#endif

	ctx->state &= ~flag;
}

/*
 * Low-level fixed frame length management helper functions
 */

/**
 * ffl_virt - helper function for getting the virtual base address of a frame
 * @frame: a reference to the considered frame
 *
 * Returns the virtual base address of the frame
 */
static inline unsigned char *ffl_virt(struct hsi_msg *frame)
{
	return (unsigned char *) (sg_virt(frame->sgt.sgl));
}

/**
 * ffl_frame_room - helper function for evaluating room in a frame
 * @frame: a reference to the considered frame
 * @used_len: the used length of the frame in bytes
 *
 * Returns the room in byte in the current frame
 */
static inline unsigned int ffl_frame_room(struct hsi_msg *frame,
					  unsigned int used_len)
{
#ifdef CONFIG_HSI_FFL_ENSURE_LAST_WORD_NULL
	return min(frame->sgt.sgl->length,
		   (unsigned int)(FFL_FRAME_LENGTH-4)) - used_len
		   - FFL_HEADER_LENGTH;
#else
	return frame->sgt.sgl->length - used_len - FFL_HEADER_LENGTH;
#endif
}

/**
 * room_in - helper function for getting current room in a frame
 * @frame: a reference to the considered frame
 *
 * Returns the room in byte in the current frame
 */
static inline unsigned int room_in(struct hsi_msg *frame)
{
	return ffl_frame_room(frame, frame->actual_len);
}

/**
 * ffl_data_ptr - helper function for getting the actual virtual address of a
 *		  frame data, taking into account the header offset
 * @frame_ptr: a pointer to the virtual base address of a frame
 * @offset: an offset to add to the current virtual address of the frame data
 *
 * Returns the virtual base address of the actual frame data
 */
static inline __attribute_const__
unsigned char *ffl_data_ptr(unsigned char *frame_ptr, unsigned int offset)
{
	return &(frame_ptr[FFL_HEADER_LENGTH+offset]);
}

/**
 * ffl_set_length - write down the length information to the frame header
 * @frame_ptr: a pointer to the virtual base address of a frame
 * @sz: the length information to encode in the header
 */
static inline void ffl_set_length(unsigned char *frame_ptr, u32 sz)
{
#if FFL_HEADER_LENGTH > 0
	u32 *header = (u32 *) frame_ptr;
	*header = sz;
#endif
}

/**
 * ffl_get_length - read the length information from the frame header
 * @frame_ptr: a pointer to the virtual base address of a frame
 *
 * Returns the length information to encode in the header
 */
static inline unsigned int ffl_get_length(unsigned char *frame_ptr)
{
#if FFL_HEADER_LENGTH > 0
	return (unsigned int) *((u32 *) frame_ptr);
#else
	return FFL_DATA_LENGTH;
#endif
}

/**
 * ffl_rx_frame_init - initialise a frame for entering the RX wait FIFO
 * @frame: a reference to the considered frame
 *
 * This helper function is simply updating the scatterlist information and
 * detecting errors.
 */
static void ffl_rx_frame_init(struct hsi_msg *frame)
{
	struct scatterlist	*sg	= frame->sgt.sgl;

	/* Use a non null frame length when an error occur to forward it to
	 * the upper layers.
	 * Do not use the in-frame length which can be broken */
	if (likely((!frame->break_frame) &&
		   (frame->status == HSI_STATUS_COMPLETED))) {
		frame->actual_len	= ffl_get_length(ffl_virt(frame));
	} else {
		pr_debug(DRVNAME ": Invalid FFL RX frame status (%d)",
			 frame->status);
		frame->actual_len	= 1;
	}
	sg->length			= 0;

	/* If the decoded frame size is invalid, we are in big trouble */
	if (unlikely((frame->actual_len > FFL_DATA_LENGTH) ||
		     (!frame->actual_len))) {
		pr_debug(DRVNAME ": Invalid FFL frame size (%u bytes)\n",
			 frame->actual_len);
		frame->status		= HSI_STATUS_ERROR;
		frame->actual_len	= 1;
	}
};

/**
 * ffl_rx_frame_skip - skip a chunk of data at the beginning of a frame
 * @frame: a reference to the considered frame
 * @copied: the length of the chunk to skip
 *
 * This helper function is simply updating the scatterlist information.
 */
static inline void ffl_rx_frame_skip(struct hsi_msg *frame,
				     unsigned int copied)
{
	struct scatterlist *sg = frame->sgt.sgl;

	sg->offset		+= copied;
	sg->length		+= copied;
	frame->actual_len	-= copied;
};

/**
 * ffl_rx_frame_reset - revert a frame to a working order
 * @ctx: a reference to the FFL context (RX or TX) to consider
 * @frame: a reference to the considered frame
 *
 * This helper function is simply updating the scatterlist information.
 */
static inline void ffl_rx_frame_reset(struct ffl_xfer_ctx *ctx,
				      struct hsi_msg *frame)
{
	struct scatterlist	*sg	= frame->sgt.sgl;

	sg->offset		-= sg->length;
	sg->length		 = (ctx->data_len + FFL_HEADER_LENGTH);
	frame->actual_len	 = 0;
};

/**
 * ffl_frame_of - get the frame virtual address from its link address
 * @link: the virtual address of the linked list structure of a frame
 *
 * Returns the corresponding frame virtual address.
 */
static inline __attribute_const__
struct hsi_msg *ffl_frame_of(struct list_head *link)
{
	return list_entry(link, struct hsi_msg, link);
}

/*
 * Low-level FIFO managing
 */

/**
 * _ffl_frame_pop - pop the frame from its containing FIFO
 * @frame: a reference to the frame being popped
 */
static inline void _ffl_frame_pop(struct hsi_msg *frame)
{
	list_del_init(&frame->link);
}

/**
 * _ffl_fifo_head - get a reference to the top frame of a FIFO
 * @fifo: a reference of the FIFO to consider
 *
 * Returns a reference to the first frame of this FIFO.
 *
 * BEWARE: calling this with an empty FIFO gives unexpected results!
 */
static inline struct hsi_msg *_ffl_fifo_head(struct list_head *fifo)
{
	return ffl_frame_of(fifo->next);
}

/**
 * _ffl_fifo_head_safe - get a reference to the top frame of a FIFO or NULL
 *			 if the FIFO is empty
 * @fifo: a reference of the FIFO to consider
 *
 * Returns a reference to the first frame of this FIFO or NULL if the FIFO is
 * empty.
 */
static inline __must_check
struct hsi_msg *_ffl_fifo_head_safe(struct list_head *fifo)
{
	struct list_head	*first = fifo->next;

	if (first == fifo)
		return NULL;
	return ffl_frame_of(first);
}

/**
 * _ffl_fifo_head_pop - get a reference to the top frame of a FIFO and pop it
 *			from the FIFO
 * @fifo: a reference of the FIFO to consider
 *
 * Returns a reference to the first frame of this FIFO, which has been popped
 *
 * BEWARE: calling this with an empty FIFO gives unexpected results!
 */
static inline struct hsi_msg *_ffl_fifo_head_pop(struct list_head *fifo)
{
	struct hsi_msg		*frame = _ffl_fifo_head(fifo);
	_ffl_frame_pop(frame);
	return frame;
}

/**
 * _ffl_fifo_head_safe_pop - get a reference to the top frame of a FIFO or NULL
 *			     if the FIFO is empty, and then pop it if it exists
 * @fifo: a reference of the FIFO to consider
 *
 * Returns a reference to the first frame of this FIFO, which has been popped
 * or NULL if the FIFO is empty.
 */
static inline __must_check
struct hsi_msg *_ffl_fifo_head_safe_pop(struct list_head *fifo)
{
	struct hsi_msg		*frame = _ffl_fifo_head_safe(fifo);
	if (frame)
		_ffl_frame_pop(frame);
	return frame;
}

/**
 * _ffl_fifo_tail_safe - get a reference to the bottom frame of a FIFO or NULL
 *			 if the FIFO is empty
 * @fifo: a reference of the FIFO to consider
 *
 * Returns a reference to the last frame of this FIFO or NULL if the FIFO is
 * empty.
 */
static inline __must_check
struct hsi_msg *_ffl_fifo_tail_safe(struct list_head *fifo)
{
	struct list_head	*last = fifo->prev;

	if (last == fifo)
		return NULL;
	return ffl_frame_of(last);
}

/**
 * _ffl_fifo_frame_push - push a frame at the bottom of a FIFO
 * @frame: a reference to the frame to push
 * @fifo: a reference to the FIFO
 */
static inline void _ffl_fifo_frame_push(struct hsi_msg *frame,
					struct list_head *fifo)
{
	list_add_tail(&frame->link, fifo);
}

/**
 * _ffl_fifo_frame_push_back - push back a frame at the top of a FIFO
 * @frame: a reference to the frame to push back
 * @fifo: a reference to the FIFO
 */
static inline void _ffl_fifo_frame_push_back(struct hsi_msg *frame,
					     struct list_head *fifo)
{
	list_add(&frame->link, fifo);
}

/*
 * Specialised FIFO handling methods
 */

/**
 * _ffl_fifo_wait_pop - pop a frame from the FIFO of waiting frames
 * @ctx: a reference to the FFL context (RX or TX) to consider
 * @frame: a reference to the frame to pop
 *
 * This function is not only popping the frame, but also updating the counters
 * related to the FIFO of waiting frames in the considered context.
 */
static inline void _ffl_fifo_wait_pop(struct ffl_xfer_ctx *ctx,
				      struct hsi_msg *frame)
{
	_ffl_frame_pop(frame);
	--ctx->wait_len;
	ctx->buffered -= frame->actual_len;
}

/**
 * _ffl_fifo_wait_push - push a frame to the FIFO of waiting frames
 * @ctx: a reference to the FFL context (RX or TX) to consider
 * @frame: a reference to the frame to push
 *
 * This function is not only pushing the frame, but also updating the counters
 * related to the FIFO of waiting frames in the considered context.
 */
static inline void _ffl_fifo_wait_push(struct ffl_xfer_ctx *ctx,
				       struct hsi_msg *frame)
{
	++ctx->wait_len;
	ctx->buffered += frame->actual_len;
	_ffl_fifo_frame_push(frame, &ctx->wait_frames);
}

/**
 * _ffl_fifo_wait_push_back - push back a frame in the FIFO of waiting frames
 * @ctx: a reference to the FFL context (RX or TX) to consider
 * @frame: a reference to the frame to push back
 *
 * This function is not only pushing back the frame, but also updating the
 * counters related to the FIFO of waiting frames in the considered context.
 */
static inline void _ffl_fifo_wait_push_back(struct ffl_xfer_ctx *ctx,
					    struct hsi_msg *frame)
{
	++ctx->wait_len;
	ctx->buffered += frame->actual_len;
	_ffl_fifo_frame_push_back(frame, &ctx->wait_frames);
}

/**
 * _ffl_fifo_ctrl_pop - pop a frame from the HSI controller FIFO
 * @ctx: a reference to the FFL context (TX or RX) to consider
 *
 * This function is only updating the counters related to the FIFO of
 * outstanding frames in the considered context.
 */
static inline void _ffl_fifo_ctrl_pop(struct ffl_xfer_ctx *ctx)
{
	--ctx->ctrl_len;
}

/**
 * _ffl_fifo_ctrl_push - push a frame to the HSI controller FIFO
 * @ctx: a reference to the FFL context (TX or RX) to consider
 * @frame: a reference to the frame to push
 * @flags: a reference to the flag used by the external spinlock, passed in to
 *	   unlock it and end the atomic context temporarily.
 *
 * Returns 0 on success or an error code on failure.
 *
 * This function is not only pushing the frame, but also updating the counters
 * related to the FIFO of outstanding frames in the considered context.
 */
static inline __must_check int _ffl_fifo_ctrl_push(struct ffl_xfer_ctx *ctx,
		struct hsi_msg *frame, unsigned long *flags)
{
	unsigned int lost_room = room_in(frame);
	int err;

#ifdef USE_IPC_ERROR_RECOVERY
	/* Prevent sending messages to the controller when shutting down or in
	 * error recovery */
	if (unlikely(_ffl_ctx_has_any_flag(ctx, ERROR_RECOVERY_ONGOING_BIT |
						TTY_OFF_BIT)) &&
	    (!frame->break_frame))
#else
	/* Prevent sending messages to the controller when shutting down */
	if (unlikely(_ffl_ctx_has_flag(ctx, TTY_OFF_BIT)) &&
	    (!frame->break_frame))
#endif
		return -EBUSY;

	/* Update the context room prior to removing the spinlock */
	ctx->room -= lost_room;
	spin_unlock_irqrestore(&ctx->lock, *flags);
	err = hsi_async(frame->cl, frame);
	spin_lock_irqsave(&ctx->lock, *flags);
	if (likely(!err))
		++ctx->ctrl_len;
	else
		ctx->room += lost_room;

	return err;
}

/*
 * FIFO transfer functions
 */

/**
 * _ffl_from_wait_to_ctrl - transfer a TX frame from the wait FIFO to the
 *			    controller FIFO
 * @ctx: a reference to the FFL TX context to consider
 * @frame: a reference to the frame to transfer
 * @flags: a reference to the flag used by the external spinlock, passed in to
 *	   unlock it and end the atomic context temporarily.
 *
 * Returns 0 if successful or an error if anything went wrong.
 */
static int _ffl_from_wait_to_ctrl(struct ffl_xfer_ctx *ctx,
				  struct hsi_msg *frame, unsigned long *flags)
{
	unsigned int	 actual_len = frame->actual_len;
	struct ffl_ctx	*main_ctx;
	int		 err;

	if (unlikely(_ffl_ctx_has_flag(ctx, TX_TTY_WRITE_FORWARDING_BIT)))
		return -EBUSY;

	_ffl_ctx_set_flag(ctx, TX_TTY_WRITE_FORWARDING_BIT);
	_ffl_fifo_wait_pop(ctx, frame);
	err = _ffl_fifo_ctrl_push(ctx, frame, flags);
	_ffl_ctx_clear_flag(ctx, TX_TTY_WRITE_FORWARDING_BIT);

	if (unlikely(err)) {
		/* Keep the data for the future */
		_ffl_fifo_wait_push_back(ctx, frame);
	} else {
		if (ctx->ctrl_len > 0) {
			main_ctx = container_of(ctx, struct ffl_ctx, tx);
			mod_timer(&main_ctx->hangup.timer,
				  jiffies + usecs_to_jiffies(TTY_HANGUP_DELAY));
			del_timer(&ctx->timer);
			_ffl_ctx_set_state(ctx, ACTIVE);
		}
#ifdef CONFIG_HSI_FFL_TTY_STATS
		ctx->data_sz += actual_len;
		ctx->frame_cnt++;
#endif
	}

	return err;
}

/**
 * _ffl_update_state_tx - update the tx state and timers
 * @ctx: a reference to the FFL TX context to consider
 */
static void _ffl_update_state_tx(struct ffl_xfer_ctx *ctx)
{
	struct ffl_ctx	*main_ctx = container_of(ctx, struct ffl_ctx, tx);

	if ((ctx->ctrl_len <= 0) &&
	    (likely(!_ffl_ctx_has_flag(ctx, TX_TTY_WRITE_FORWARDING_BIT)))) {
		del_timer(&main_ctx->hangup.timer);
		if (ctx->wait_len == 0) {
			wake_up(&main_ctx->tx_full_pipe_clean_event);
			mod_timer(&ctx->timer, jiffies + ctx->delay);
		}
	}
}

/**
 * _ffl_pop_wait_push_ctrl - transfer as many TX frames as possible from the
 *			     wait FIFO to the controller FIFO, unless a frame
 *			     is being updated (marked as not completed)
 * @ctx: a reference to the FFL TX context to consider
 * @flags: a reference to the flag used by the external spinlock, passed in to
 *	   unlock it and end the atomic context temporarily.
 */
static void _ffl_pop_wait_push_ctrl(struct ffl_xfer_ctx *ctx,
				    unsigned long *flags)
{
	struct hsi_msg		*frame;

	while ((ctx->wait_len > 0) && (ctx->ctrl_len < ctx->ctrl_max)) {

		frame = _ffl_fifo_head(&ctx->wait_frames);

		if ((frame->status != HSI_STATUS_COMPLETED) ||
		    (unlikely(_ffl_ctx_has_flag(ctx, TTY_OFF_BIT))) ||
		    (_ffl_from_wait_to_ctrl(ctx, frame, flags) != 0))
			break;
	}

	_ffl_update_state_tx(ctx);
}

/*
 * Frame (hsi_msg) creation and deletion
 */

/**
 * ffl_delete_frame - helper function to delete and free an existing frame
 * @frame: a reference to the frame to delete
 * @ctx: a reference to the related FFL context
 *
 * This function shall only be called by the pool of frame management routines.
 */
static void ffl_delete_frame(struct hsi_msg *frame, struct ffl_ctx *ctx)
{
	/* Revert to the actual allocated size */
	frame->sgt.sgl->length = FFL_FRAME_LENGTH;

	if ((ctx->controller) &&
	    (is_device_dma_capable(ctx->controller))) {
		dma_free_coherent(ctx->controller, FFL_FRAME_LENGTH,
				  ffl_virt(frame),
				  sg_dma_address(frame->sgt.sgl));
	} else {
#ifdef FFL_FRAME_ALLOC_PAGES
		__free_pages(sg_page(frame->sgt.sgl), FFL_FRAME_ALLOC_ORDER);
#else
		kfree(ffl_virt(frame));
#endif
	}

	sg_free_table(&frame->sgt);
	kfree(frame);
}

/* Forward declarations for ffl_create_frame() */
static void ffl_complete_tx(struct hsi_msg *frame);
static void ffl_complete_rx(struct hsi_msg *frame);
static void ffl_destruct_frame(struct hsi_msg *frame);

/**
 * ffl_create_frame - helper function to allocate and initialise a new frame
 * @ctx: a reference to the FFL context (RX or TX) to consider
 *
 * Returns a reference to the newly created frame or NULL if an error occured.
 *
 * This function shall only be called by the pool of frame management routines.
 */
static __must_check struct hsi_msg *ffl_create_frame(struct ffl_xfer_ctx *ctx)
{
	struct ffl_ctx	*main_ctx = main_ctx(ctx);
	struct hsi_msg	*new;
	void		*buffer;

	/* Be careful: might sleep ! */
	new = kzalloc(sizeof(struct hsi_msg), GFP_KERNEL);
	if (unlikely(!new))
		goto fail0;

	if (unlikely(sg_alloc_table(&new->sgt, 1, GFP_KERNEL)))
		goto fail1;

	if ((main_ctx->controller) &&
	    (is_device_dma_capable(main_ctx->controller))) {
		buffer = dma_alloc_coherent(main_ctx->controller,
					    FFL_FRAME_LENGTH,
					    &sg_dma_address(new->sgt.sgl),
					    GFP_KERNEL);
	} else {
#ifdef FFL_FRAME_ALLOC_PAGES
		buffer = (void *) __get_free_pages(GFP_KERNEL,
						   FFL_FRAME_ALLOC_ORDER);
#else
		buffer = kmalloc(FFL_FRAME_LENGTH, GFP_KERNEL);
#endif
#ifdef CONFIG_HSI_FFL_ENSURE_LAST_WORD_NULL
		((u32 *)buffer)[FFL_FRAME_LENGTH/4-1] = 0;
#endif
	}

	if (unlikely(!buffer))
		goto fail2;

	sg_set_buf(new->sgt.sgl, buffer, FFL_FRAME_LENGTH);

	ffl_set_length(buffer, 0);

	new->cl		= main_ctx(ctx)->client;
	new->context	= ctx;

	if (xfer_ctx_is_tx_ctx(ctx)) {
		new->complete	= &ffl_complete_tx;
		new->destructor	= &ffl_destruct_frame;
		new->ttype	= HSI_MSG_WRITE;
	} else {
		new->complete	= &ffl_complete_rx;
		new->destructor	= &ffl_destruct_frame;
		new->ttype	= HSI_MSG_READ;
	}

	return new;

fail2:
	sg_free_table(&new->sgt);

fail1:
	kfree(new);

fail0:
	return NULL;
}

/*
 * FIFO length management
 */

/**
 * _ffl_ctx_is_empty - checks if a context is empty (all FIFO are empty)
 * @ctx: a reference to the FFL context (RX or TX) to consider
 *
 * This helper function is returning a non-zero value if both the wait FIFO and
 * the controller FIFO are empty. Note that this does not mean that there are
 * no data pending in the controller hardware.
 */
static __must_check inline int _ffl_ctx_is_empty(struct ffl_xfer_ctx *ctx)
{
	return ((ctx->wait_len == 0) && (ctx->ctrl_len <= 0));
}

/**
 * ffl_tx_full_pipe_is_clean - checks if the TX pipe is clean or about to be
 * @ctx: a reference to the main FFL context to consider
 *
 * This helper function is returning a non-zero value if both the wait FIFO and
 * the controller FIFO are empty or if a hangup of any kind is currently
 * outstanding
 */
static __must_check int ffl_tx_full_pipe_is_clean(struct ffl_ctx *ctx)
{
	struct ffl_xfer_ctx	*tx_ctx = &ctx->tx;
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&tx_ctx->lock, flags);
	ret = (_ffl_ctx_is_empty(tx_ctx) &&
	       (likely(!_ffl_ctx_has_flag(tx_ctx,
					  TX_TTY_WRITE_FORWARDING_BIT)))) ||
	      (ctx->hangup.tx_timeout);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

	return ret;
}

/**
 * ffl_tx_write_pipe_is_clean - checks if there are still outstanding writes
 * @ctx: a reference to the TX context to consider
 *
 * This helper function is returning a non-zero value if there is no
 * outstanding writes in the TX pipeline.
 */
static __must_check int ffl_tx_write_pipe_is_clean(struct ffl_xfer_ctx *ctx)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&ctx->lock, flags);
	ret = !_ffl_ctx_has_flag(ctx, TX_TTY_WRITE_ONGOING_BIT);
	spin_unlock_irqrestore(&ctx->lock, flags);

	return ret;
}

#ifdef USE_WAKE_POST_BOOT_HANDSHAKE
/**
 * ffl_modem_is_awake - checks if the RX side is active or not
 * @ctx: a reference to the RX context to consider
 *
 * This helper function is returning a non-zero value if the RX state is active.
 */
static __must_check int ffl_modem_is_awake(struct ffl_xfer_ctx *ctx)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&ctx->lock, flags);
	ret = _ffl_ctx_is_state(ctx, ACTIVE);
	spin_unlock_irqrestore(&ctx->lock, flags);

	return ret;
}
#endif

/*
 * State machines
 */

/**
 * ffl_do_start_tx - making a synchronous HSI TX start request
 * @work: a reference to work queue element
 */
static void ffl_do_start_tx(struct work_struct *work)
{
	struct ffl_ctx		*main_ctx;
	struct ffl_xfer_ctx	*ctx;
	unsigned long		 flags;
	int			 exit;

	main_ctx = container_of(work, struct ffl_ctx, start_tx);
	ctx = &main_ctx->tx;

	spin_lock_irqsave(&ctx->lock, flags);
	exit = !_ffl_ctx_is_state(ctx, IDLE);
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (unlikely(exit))
		return;

	exit = hsi_start_tx(main_ctx->client);

	if (unlikely(exit)) {
		pr_err(DRVNAME ": hsi_start_tx error (%d)", exit);
		return;
	}

#if (ACWAKE_TO_DATA_MINIMAL_UDELAY > 0)
	/* Prevent too soon a data write further to a ACWAKE */
	udelay(ACWAKE_TO_DATA_MINIMAL_UDELAY);
#endif

	spin_lock_irqsave(&ctx->lock, flags);
	/* The HSI controller is ready, push as many frames as possible */
	_ffl_ctx_set_state(ctx, READY);
	_ffl_pop_wait_push_ctrl(ctx, &flags);
	spin_unlock_irqrestore(&ctx->lock, flags);
}

/**
 * ffl_do_stop_tx - making a synchronous HSI TX stop request
 * @work: a reference to work queue element
 */
static void ffl_do_stop_tx(struct work_struct *work)
{
	struct ffl_ctx		*main_ctx;
	struct ffl_xfer_ctx	*ctx;
	unsigned long		 flags;
	int			 exit;

	main_ctx = container_of(work, struct ffl_ctx, stop_tx);
	ctx = &main_ctx->tx;

	exit = hsi_stop_tx(main_ctx->client);

	if (unlikely(exit)) {
		pr_err(DRVNAME ": hsi_stop_tx error (%d)", exit);
		spin_lock_irqsave(&ctx->lock, flags);
		_ffl_ctx_set_state(ctx, READY);
		spin_unlock_irqrestore(&ctx->lock, flags);
		return;
	}

#if (ACWAKE_MINIMAL_PULSE_UDELAY > 0)
	/* Prevent too small a ACWAKE pulse */
	udelay(ACWAKE_MINIMAL_PULSE_UDELAY);
#endif
}

/**
 * _ffl_start_tx - update the TX state machine on every new transfer
 * @ctx: a reference to the FFL TX context to consider
 *
 * This helper function updates the TX state if it is currently idle and
 * inform the HSI framework and attached controller.
 */
static void _ffl_start_tx(struct ffl_xfer_ctx *ctx)
{
	struct ffl_ctx *main_ctx;

	if (_ffl_ctx_is_state(ctx, IDLE)) {
		main_ctx = container_of(ctx, struct ffl_ctx, tx);
		queue_work(ffl_tx_wq, &main_ctx->start_tx);
	} else {
		_ffl_ctx_set_state(ctx, READY);
		del_timer(&ctx->timer);
	}
}

/**
 * _ffl_stop_tx - update the TX state machine after expiration of the TX active
 *		  timeout further to a no outstanding TX transaction status
 * @ctx: a reference to the FFL TX context to consider
 *
 * This helper function updates the TX state if it is currently active and
 * inform the HSI framework and attached controller.
 */
static void _ffl_stop_tx(struct ffl_xfer_ctx *ctx)
{
	struct ffl_ctx *main_ctx;

	if (_ffl_ctx_is_state(ctx, ACTIVE)) {
		main_ctx = container_of(ctx, struct ffl_ctx, tx);
		_ffl_ctx_set_state(ctx, IDLE);
		queue_work(ffl_tx_wq, &main_ctx->stop_tx);
	}
}

/**
 * ffl_stop_tx - update the TX state machine after expiration of the TX active
 *		 timeout further to a no outstanding TX transaction status
 * @param: a hidden reference to the FFL TX context to consider
 *
 * This helper function updates the TX state if it is currently active and
 * inform the HSI framework and attached controller.
 */
static void ffl_stop_tx(unsigned long param)
{
	struct ffl_xfer_ctx	*ctx = (struct ffl_xfer_ctx *) param;
	unsigned long		flags;

	spin_lock_irqsave(&ctx->lock, flags);
	_ffl_stop_tx(ctx);
	spin_unlock_irqrestore(&ctx->lock, flags);
}

/**
 * ffl_start_rx - update the internal RX state machine
 * @cl: a reference to HSI client to consider
 *
 * This helper function updates the RX state and wakes the device.
 */
static void ffl_start_rx(struct hsi_client *cl)
{
	struct ffl_ctx		*main_ctx	=
				(struct ffl_ctx *) hsi_client_drvdata(cl);
	struct ffl_xfer_ctx	*ctx		= &main_ctx->rx;
	unsigned long		flags;

	spin_lock_irqsave(&ctx->lock, flags);
	_ffl_ctx_set_state(ctx, ACTIVE);
#ifdef USE_WAKE_POST_BOOT_HANDSHAKE
	/**
	* Excute only if not in flashing mode
	* <=> if !(tx_ctx->data_len<(FFL_DATA_LENGTH - 4))
	*/
	if (!(ctx->data_len < (FFL_DATA_LENGTH - 4)))
		wake_up(&main_ctx->modem_awake_event);
#endif
	spin_unlock_irqrestore(&ctx->lock, flags);
}

/**
 * _ffl_state_rx_not_active - update the RX state machine upon reception of an
 * @ctx: a reference to the FFL RX context to consider
 *
 * This helper function updates the RX state in accordance with the status of
 * the RX FIFO.
 */
static inline void _ffl_state_rx_not_active(struct ffl_xfer_ctx *ctx)
{
	if (!_ffl_ctx_is_empty(ctx))
		_ffl_ctx_set_state(ctx, READY);
	else
		_ffl_ctx_set_state(ctx, IDLE);
}

/**
 * _ffl_update_state_rx - update the RX state machine upon recycling of a
 *			  RX frame
 * @ctx: a reference to the FFL RX context to consider
 *
 * This helper function updates the RX state in accordance with the status of
 * the RX FIFO, unless the RX is required active.
 */
static inline void _ffl_update_state_rx(struct ffl_xfer_ctx *ctx)
{
	if (!_ffl_ctx_is_state(ctx, ACTIVE))
		_ffl_state_rx_not_active(ctx);
}

/**
 * _ffl_stop_rx - update the internal RX state machine
 * @ctx: a reference to the FFL RX context to consider
 * @main_ctx: a reference to related main FFL context
 *
 * This helper function updates the RX state and allows the HSI device to
 * sleep.
 */
static inline void _ffl_stop_rx(struct ffl_xfer_ctx *ctx,
				struct ffl_ctx *main_ctx)
{
	_ffl_state_rx_not_active(ctx);
}

/**
 * ffl_stop_rx - update the internal RX state machine
 * @cl: a reference to HSI client to consider
 *
 * This helper function updates the RX state and allows the HSI device to
 * sleep.
 */
static void ffl_stop_rx(struct hsi_client *cl)
{
	struct ffl_ctx		*main_ctx	=
				(struct ffl_ctx *) hsi_client_drvdata(cl);
	struct ffl_xfer_ctx	*ctx		= &main_ctx->rx;
	unsigned long		flags;

	spin_lock_irqsave(&ctx->lock, flags);
	_ffl_stop_rx(ctx, main_ctx);
	spin_unlock_irqrestore(&ctx->lock, flags);
}

/**
 * ffl_hsi_ehandler - HSI client events handler
 * @cl: a reference to HSI client to consider
 * @ev: the HSI event
 *
 */
static void ffl_hsi_ehandler(struct hsi_client *cl, unsigned long event)
{
	switch (event) {
	case HSI_EVENT_START_RX:
		ffl_start_rx(cl);
		break;

	case HSI_EVENT_STOP_RX:
		ffl_stop_rx(cl);
		break;
	}
}

/*
 * Frame recycling helper functions
 */

/**
 * _ffl_new_frame - creating a new empty file from the recycling FIFO
 * @ctx: a reference to the FFL context (RX or TX) to consider
 *
 * Returns a reference to the new empty frame or NULL if there are no recycled
 * frames left.
 */
static inline __must_check
struct hsi_msg *_ffl_new_frame(struct ffl_xfer_ctx *ctx)
{
	/* If there are recycled items, then we are sure that there is room
	 * in either the waiting FIFO or controller FIFO */
	return _ffl_fifo_head_safe_pop(&ctx->recycled_frames);
}

/**
 * _ffl_update_ctx_status - updating the channel status further to config
 *			    changes (channel, frame length)
 * @ctx: a reference to the FFL context (RX or TX) to consider
 */
static void _ffl_update_ctx_status(struct ffl_xfer_ctx *ctx)
{
	struct list_head	*index;
	struct hsi_msg		*frame;
	struct scatterlist	*sg;

	list_for_each(index, &ctx->recycled_frames) {
		frame		 = ffl_frame_of(index);
		sg		 = frame->sgt.sgl;

		frame->channel	 = ctx->channel;

		ctx->room	+= ctx->data_len;
		ctx->room	-= (sg->length - FFL_HEADER_LENGTH);
		sg->length	 = ctx->data_len + FFL_HEADER_LENGTH;
	}

	if (xfer_ctx_is_tx_ctx(ctx))
		main_ctx(ctx)->client->tx_cfg = ctx->config;
	else
		main_ctx(ctx)->client->rx_cfg = ctx->config;
}

/**
 * _ffl_recycle_frame - recycling a frame in the recycling FIFO
 * @ctx: a reference to the FFL context (RX or TX) to consider
 * @frame: a reference to the frame to recycle
 */
static inline void _ffl_recycle_frame(struct ffl_xfer_ctx *ctx,
				      struct hsi_msg *frame)
/* Needs locking ! */
{
	_ffl_fifo_frame_push(frame, &ctx->recycled_frames);
}

/**
 * _ffl_free_frame - deleting a frame created by a call to ffl_create_frame
 * @ctx: a reference to the FFL context (RX or TX) to consider
 * @frame: a reference to the frame to delete
 * @flags: a reference to the flag used by the external spinlock, passed in to
 *	   unlock it and end the atomic context temporarily.
 *
 * This function is either recycling the frame if there are not too many frames
 * in the system, otherwise destroy it and free its resource.
 */
static void _ffl_free_frame(struct ffl_xfer_ctx *ctx,
			    struct hsi_msg *frame,
				unsigned long *flags)
{
	if (unlikely(ctx->all_len > (ctx->wait_max+ctx->ctrl_max))) {
		if (flags)
			spin_unlock_irqrestore(&ctx->lock, *flags);
		ffl_delete_frame(frame, main_ctx(ctx));

		if (flags)
			spin_lock_irqsave(&ctx->lock, *flags);
		--ctx->all_len;
	} else {
		frame->status		= HSI_STATUS_COMPLETED;
		frame->actual_len	= 0;
		frame->break_frame	= 0;
#ifdef DEBUG
		/* Initialise the frame with a buggy length to ensure that it
		 * is correctly updated prior TX and after RX */
		ffl_set_length(ffl_virt(frame), FFL_BUGGY_FRAME_SIZE);
#endif
		ctx->room += ffl_frame_room(frame, 0);
		_ffl_recycle_frame(ctx, frame);
	}
}

/**
 * ffl_destruct_frame - delete or recycle an existing frame
 * @frame: a reference to the frame to delete
 *
 * This function shall only be called as an HSI destruct callback.
 */
static void ffl_destruct_frame(struct hsi_msg *frame)
{
	struct ffl_xfer_ctx	*ctx = frame->context;
	unsigned long		flags;

	spin_lock_irqsave(&ctx->lock, flags);
	_ffl_fifo_ctrl_pop(ctx);
	_ffl_free_frame(ctx, frame, &flags);
	if (xfer_ctx_is_rx_ctx(ctx))
		_ffl_update_state_rx(ctx);
	else
		_ffl_update_state_tx(ctx);
	spin_unlock_irqrestore(&ctx->lock, flags);
}

/**
 * _ffl_fifo_frames_delete - deletes the whole content of a FIFO
 * @fifo: a reference to the FIFO to empty
 * @ctx: a reference to the main context related to the FIFO
 *
 * This helper function is emptying a FIFO and deleting all its frames.
 */
static void _ffl_fifo_frames_delete(struct list_head	*fifo,
				    struct ffl_ctx	*ctx)
{
	struct hsi_msg *frame;

	while ((frame = _ffl_fifo_head_safe(fifo))) {
		_ffl_frame_pop(frame);
		ffl_delete_frame(frame, ctx);
	}
}

/**
 * _ffl_tx_fifo_wait_recycle - recycle the whole content of the TX waiting FIFO
 * @ctx: a reference to the TX FFL context to consider
 *
 * This helper function is emptying a waiting TX FIFO and recycling all its
 * frames.
 */
static void _ffl_tx_fifo_wait_recycle(struct ffl_xfer_ctx *ctx)
{
	struct hsi_msg	*frame;

	_ffl_ctx_clear_flag(ctx, TX_TTY_WRITE_PENDING_BIT);

	while ((frame = _ffl_fifo_head_safe(&ctx->wait_frames))) {
		_ffl_fifo_wait_pop(ctx, frame);
		ctx->room -= room_in(frame);
		if (frame->status == HSI_STATUS_COMPLETED)
			_ffl_free_frame(ctx, frame, NULL);
		else
			frame->status = HSI_STATUS_ERROR;
	}
}

/**
 * _ffl_rx_fifo_wait_recycle - recycle the whole content of the RX waiting FIFO
 * @ctx: a reference to the RX FFL context to consider
 *
 * This helper function is emptying a waiting RX FIFO and recycling all its
 * frames.
 */
static void _ffl_rx_fifo_wait_recycle(struct ffl_xfer_ctx *ctx)
{
	struct hsi_msg *frame;

	while ((frame = _ffl_fifo_head_safe(&ctx->wait_frames))) {
		_ffl_fifo_wait_pop(ctx, frame);
		ffl_rx_frame_reset(ctx, frame);
		_ffl_free_frame(ctx, frame, NULL);
	}
}

/**
 * ffl_increase_pool_of_frames - background work aimed at creating new frames
 * @work: a reference to the work context
 *
 * This function is called as a background job (in the ffl_rx_wq/ffl_tx_wq work
 * queues) for performing the frame resource allocation (which can then sleep).
 *
 * An error message is sent upon the failure of FFL_FRAME_ALLOC_RETRY_MAX_CNT
 * allocation requests.
 */
static void ffl_increase_pool_of_frames(struct work_struct *work)
{
	struct ffl_xfer_ctx	*ctx = container_of(work, struct ffl_xfer_ctx,
						    increase_pool);
	struct hsi_msg		*new;
	int			retry;
	unsigned long		flags;

	spin_lock_irqsave(&ctx->lock, flags);
	while (ctx->all_len < (ctx->wait_max+ctx->ctrl_max)) {
		spin_unlock_irqrestore(&ctx->lock, flags);

		retry = 0;
		new = ffl_create_frame(ctx);
		while (!new) {
			++retry;
			if (retry == FFL_FRAME_ALLOC_RETRY_MAX_CNT) {
				pr_err(DRVNAME
				       ": cannot allocate a frame after %d"
				       " retries...", retry);
				retry = 0;
			}

			/* No memory available: do something more urgent ! */
			schedule();
			new = ffl_create_frame(ctx);
		}

		spin_lock_irqsave(&ctx->lock, flags);
		new->channel	 = ctx->channel;
#ifdef CONFIG_HSI_FFL_ENSURE_LAST_WORD_NULL
		ctx->room	+= min(ctx->data_len,
				       (unsigned int)(FFL_DATA_LENGTH - 4));
#else
		ctx->room	+= ctx->data_len;
#endif
		new->sgt.sgl->length	 = ctx->data_len + FFL_HEADER_LENGTH;
		_ffl_recycle_frame(ctx, new);
		++ctx->all_len;
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	pr_debug(DRVNAME ": done creating pool of frames.");
}

/*
 * TX data flow functions
 */

/* The top-down flow is made in the ffl_tty_write() TTY function */

/**
 * ffl_tty_wakeup - wakeup an asleep TTY write function call
 * @ctx: a reference to the context related to this TTY
 *
 * This helper function awakes any asleep TTY write callback function.
 */
static void ffl_tty_wakeup(struct ffl_ctx *ctx)
{
	struct tty_struct	*tty;

	tty = tty_port_tty_get(&ctx->tty_prt);
	if (likely(tty)) {
		tty_wakeup(tty);
		tty_kref_put(tty);
	}
}

/**
 * ffl_complete_tx - bottom-up flow for the TX side
 * @frame: a reference to the completed frame
 *
 * A TX transfer has completed: recycle the completed frame and kick a new
 * delayed request to enter the IDLE state if nothing else is expected.
 */
static void ffl_complete_tx(struct hsi_msg *frame)
{
	struct ffl_xfer_ctx	*ctx = frame->context;
	struct ffl_ctx		*main_ctx = container_of(ctx,
							 struct ffl_ctx, tx);
	int			wakeup;
	unsigned long		flags;

	spin_lock_irqsave(&ctx->lock, flags);
	_ffl_fifo_ctrl_pop(ctx);
	_ffl_free_frame(ctx, frame, &flags);

	/* Start new waiting frames (if any) */
	_ffl_pop_wait_push_ctrl(ctx, &flags);

	/* Wake-up the TTY write whenever the TX wait FIFO is half empty and
	 * has been full, or is currently empty to prevent too many wakeups */
	wakeup = ((_ffl_ctx_has_flag(ctx, TX_TTY_WRITE_PENDING_BIT)) &&
		  (ctx->wait_len <= ctx->wait_max/2)) || (ctx->wait_len == 0);
	if (wakeup)
		_ffl_ctx_clear_flag(ctx, TX_TTY_WRITE_PENDING_BIT);
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (wakeup)
		ffl_tty_wakeup(main_ctx);
}

/*
 * RX data flow functions
 */

/**
 * _ffl_rx_push_controller - Push as many recycled frames as possible to the
 *			     controller FIFO
 * @ctx: a reference to the RX context where the FIFO of recycled frames sits
 * @flags: a reference to the flag used by the external spinlock, passed in to
 *	   unlock it and end the atomic context temporarily.
 *
 * Returns 0 upon success or an error code.
 *
 * This helper method is returning 0 on success, or an error code.
 */
static __must_check int _ffl_rx_push_controller(struct ffl_xfer_ctx *ctx,
						unsigned long *flags)
{
	struct hsi_msg *new;

	while (ctx->ctrl_len < ctx->ctrl_max) {
		new = _ffl_new_frame(ctx);
		if (!new)
			return -ENOMEM;
		if (unlikely(_ffl_fifo_ctrl_push(ctx, new, flags))) {
			_ffl_recycle_frame(ctx, new);
			return -EAGAIN;
		}
	}

	return 0;
}

/**
 * _ffl_rx_free_frame - frame recycling helper function for the RX side
 * @ctx: a reference to the RX context where the FIFO of recycled frames sits
 * @frame: a reference to the frame that shall be recycled
 * @flags: a reference to the flag used by the external spinlock, passed in to
 *	   unlock it and end the atomic context temporarily.
 *
 * This helper method is recycling the frame and pushing a new frame to the
 * controller if there is room available in the controller FIFO, and finally
 * updating the state of the RX state machine.
 */
static void _ffl_rx_free_frame(struct ffl_xfer_ctx *ctx,
			       struct hsi_msg *frame, unsigned long *flags)
{
	struct hsi_msg *new;

	ffl_rx_frame_reset(ctx, frame);
	_ffl_free_frame(ctx, frame, NULL);
	if (ctx->ctrl_len < ctx->ctrl_max) {
		new = _ffl_new_frame(ctx);
		if (new && (unlikely(_ffl_fifo_ctrl_push(ctx, new, flags))))
			_ffl_recycle_frame(ctx, new);
	}
	_ffl_update_state_rx(ctx);
}

/**
 * _ffl_forward_tty - RX data TTY forwarding helper function
 * @tty: a reference to the TTY where the data shall be forwarded
 * @ctx: a reference to the RX context where the FIFO of waiting frames sits
 * @flags: a reference to the flag used by the external spinlock, passed in to
 *	   unlock it and end the atomic context temporarily.
 *
 * Data contained in the waiting frame FIFO shall be forwarded to the TTY.
 * This function is pushing as much data as possible to the TTY interface, is
 * recycling frames that have been fully forwarded and is kicking a TTY insert
 * restart delayed job if some data is remaining in the waiting FIFO or if the
 * controller FIFO is not full yet.
 */
static void _ffl_forward_tty(struct tty_struct		*tty,
			     struct ffl_xfer_ctx	*ctx,
			     unsigned long		 *flags)
{
	struct hsi_msg		*frame;
	unsigned char		*data_ptr;
	unsigned int		 copied;
	int			 do_push;
	int			 err;
	char			 tty_flag;
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	struct ffl_ctx 		*main_ctx = ctx->main_ctx;
#endif

	/* Initialised to 1 to prevent unexpected TTY forwarding resume
	 * function when there is no TTY or when it is throttled */
	copied	= 1;
	do_push	= 0;
	err	= 0;

	del_timer(&ctx->timer);

	while (ctx->wait_len > 0) {
		frame = _ffl_fifo_head(&ctx->wait_frames);
		if (likely(frame->status == HSI_STATUS_COMPLETED))
			tty_flag = (likely(!frame->break_frame)) ?
				   TTY_NORMAL : TTY_BREAK;
		else
			tty_flag = TTY_FRAME;

		_ffl_fifo_wait_pop(ctx, frame);

		if (unlikely(!tty))
			goto free_frame;

		while (frame->actual_len > 0) {

#ifndef IGNORE_TTY_THROTTLE
			/* In some rare race condition, we can't rely on
			 * TTY_THROTTLE bit if low_latency is set.
			 * The FFL can ignore this bit as
			 * tty_insert_flip_string_fixed_flag will return 0
			 * when no more space is left
			 */
			if (test_bit(TTY_THROTTLED, &tty->flags)) {
				/* Initialised to 1 to prevent unexpected TTY
				 * forwarding resume function schedule */
				copied = 1;
				_ffl_fifo_wait_push_back(ctx, frame);
				goto no_more_tty_insert;
			}
#endif
			spin_unlock_irqrestore(&ctx->lock, *flags);

			/* Copy the data to the flip buffers */
			data_ptr = ffl_data_ptr(ffl_virt(frame), 0);

#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
			copied = (unsigned int)
			tty_insert_flip_string_fixed_flag(&main_ctx->tty_prt,
							  data_ptr,
							  tty_flag,
							  frame->actual_len);
#else
			copied = (unsigned int)
			tty_insert_flip_string_fixed_flag(tty,
							  data_ptr,
							  tty_flag,
							  frame->actual_len);
#endif
			ffl_rx_frame_skip(frame, copied);

			/* We'll push the flip buffers each time something has
			 * been written to them to allow low latency */
			do_push |= (copied > 0);

			spin_lock_irqsave(&ctx->lock, *flags);

			if (copied == 0) {
				_ffl_fifo_wait_push_back(ctx, frame);
				goto no_more_tty_insert;
			}
		}

free_frame:
		_ffl_rx_free_frame(ctx, frame, flags);
	}

no_more_tty_insert:
	if (do_push) {
		/* Using tty_flip_buffer_push() with a low latency flag to
		 * bypass the shared system work queue */
		spin_unlock_irqrestore(&ctx->lock, *flags);
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
		tty_flip_buffer_push(&main_ctx->tty_prt);
#else
		tty_flip_buffer_push(tty);
#endif
		spin_lock_irqsave(&ctx->lock, *flags);
	}

	if (unlikely(ctx->ctrl_len < ctx->ctrl_max))
		err = _ffl_rx_push_controller(ctx, flags);

	/* Shoot again later if there is still pending data to serve or if
	 * the RX controller FIFO is not ready yet */
	if ((!copied) || (unlikely(err == -EAGAIN)))
		mod_timer(&ctx->timer, jiffies + ctx->delay);
}

/**
 * ffl_do_tty_forward - forwarding data to the above line discipline
 * @work: a reference to work queue element
 */
static void ffl_do_tty_forward(struct work_struct *work)
{
	struct ffl_ctx		*ctx;
	struct tty_struct	*tty;
	unsigned long		 flags;

	ctx = container_of(work, struct ffl_ctx, do_tty_forward);
	tty = tty_port_tty_get(&ctx->tty_prt);

	if (tty) {
		spin_lock_irqsave(&ctx->rx.lock, flags);
		_ffl_forward_tty(tty, &ctx->rx, &flags);
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		tty_kref_put(tty);
	}
}

#ifdef USE_IPC_ERROR_RECOVERY
/* Forward declarations for ffl_complete_rx() */
static void do_recovery_drain_unless(struct ffl_xfer_ctx *xfer_ctx,
				     unsigned int condition);
#endif

/**
 * ffl_complete_rx - bottom-up flow for the RX side
 * @frame: a reference to the completed frame
 *
 * A RX transfer has completed: push the data conveyed in the frame to the TTY
 * interface and signal any existing error.
 */
static void ffl_complete_rx(struct hsi_msg *frame)
{
	struct ffl_xfer_ctx	*ctx = frame->context;
	struct ffl_ctx		*main_ctx = container_of(ctx,
							 struct ffl_ctx, rx);
	unsigned long		flags;

#ifdef DEBUG
	if (unlikely(frame->actual_len != (ctx->data_len + FFL_HEADER_LENGTH)))
		pr_err(DRVNAME ": [%08x] Invalid FFL frame length %d bytes\n",
		       (u32) frame, frame->actual_len);
#endif

	ffl_rx_frame_init(frame);

	spin_lock_irqsave(&ctx->lock, flags);

#ifdef USE_IPC_ERROR_RECOVERY
	/* Tag frames as being error frames when in error recovery mode */
	if (unlikely(_ffl_ctx_has_flag(ctx, ERROR_RECOVERY_RX_ERROR_BIT))) {
		frame->actual_len	= 1;
		frame->status		= HSI_STATUS_ERROR;
	} else if (unlikely(frame->status != HSI_STATUS_COMPLETED)) {
		_ffl_ctx_set_flag(ctx, ERROR_RECOVERY_RX_ERROR_BIT);
		spin_unlock_irqrestore(&ctx->lock, flags);
		do_recovery_drain_unless(&main_ctx->tx,
					 ERROR_RECOVERY_ONGOING_BIT |
					 TTY_OFF_BIT);
		spin_lock_irqsave(&ctx->lock, flags);
	}
#endif

	_ffl_fifo_ctrl_pop(ctx);
#ifdef CONFIG_HSI_FFL_TTY_STATS
	ctx->data_sz += frame->actual_len;
	ctx->frame_cnt++;
	if (ctx->ctrl_len <= 0)
		ctx->overflow_cnt++;
#endif
	_ffl_fifo_wait_push(ctx, frame);
	spin_unlock_irqrestore(&ctx->lock, flags);
	queue_work(ffl_rx_wq, &main_ctx->do_tty_forward);
}

/**
 * ffl_rx_forward_retry - TTY forwarding retry job
 * @param: a casted reference to the to the RX context where the FIFO of
 *	   waiting frames sits
 *
 * This simply calls the TTY forwarding function in a tasklet shell.
 */
static void ffl_rx_forward_retry(unsigned long param)
{
	struct ffl_xfer_ctx	*ctx = (struct ffl_xfer_ctx *) param;
	struct ffl_ctx		*main_ctx = container_of(ctx,
							 struct ffl_ctx, rx);

	queue_work(ffl_rx_wq, &main_ctx->do_tty_forward);
}

/**
 * ffl_rx_forward_resume - TTY forwarding resume callback
 * @tty: a reference to the TTY requesting the resume
 *
 * This simply calls the TTY forwarding function as a response to a TTY
 * unthrottle event.
 */
static void ffl_rx_forward_resume(struct tty_struct *tty)
{
	struct ffl_ctx		*main_ctx;

	/* Get the context reference from the driver data if already opened */
	main_ctx = (struct ffl_ctx *) tty->driver_data;

	if (main_ctx)
		queue_work(ffl_rx_wq, &main_ctx->do_tty_forward);
}

/*
 * Time handling methods
 */

/**
 * from_usecs - translating usecs to jiffies
 * @delay: the delay in usecs
 *
 * Returns the delay rounded up to the next jiffy and prevent it to be set
 * to zero, as all delayed function calls shall occur to the next jiffy (at
 * least).
 */
static inline unsigned long from_usecs(const unsigned long delay)
{
	unsigned long j = usecs_to_jiffies(delay);

	if (j == 0)
		j = 1;

	return j;
}

/*
 * TTY handling methods
 */

/**
 * ffl_wait_until_ctx_sent - waits for all the TX FIFO to be empty
 * @ctx: a reference to the considered context
 * @timeout: a timeout value expressed in jiffies
 */
static inline void ffl_wait_until_ctx_sent(struct ffl_ctx *ctx, int timeout)
{
	wait_event_interruptible_timeout(ctx->tx_full_pipe_clean_event,
					 ffl_tx_full_pipe_is_clean(ctx),
					 timeout);
}

/**
 * ffl_tty_port_activate - callback to the TTY port activate function
 * @port: a reference to the calling TTY port
 * @tty: a reference to the calling TTY
 *
 * Return 0 on success or a negative error code on error.
 *
 * The TTY port activate is only called on the first port open.
 */
static int ffl_tty_port_activate(struct tty_port *port, struct tty_struct *tty)
{
	struct ffl_ctx		*ctx;
	struct ffl_xfer_ctx	*tx_ctx;
	struct ffl_xfer_ctx	*rx_ctx;
	int			err;
	unsigned long		flags;

	/* Get the context reference stored in the TTY open() */
	ctx = (struct ffl_ctx *) tty->driver_data;
	tx_ctx = &ctx->tx;
	rx_ctx = &ctx->rx;

	/* Update the TX and RX HSI configuration */
	_ffl_update_ctx_status(tx_ctx);
	_ffl_update_ctx_status(rx_ctx);

	/* Claim the HSI port */
	err = hsi_claim_port(ctx->client, 0);
	if (unlikely(err)) {
		pr_err(DRVNAME ": HSI port claim failed (%d)", err);
		return err;
	}

	/* Setup the HSI controller */
	err = hsi_setup(ctx->client);
	if (unlikely(err)) {
		pr_err(DRVNAME ": HSI setup failed (%d)", err);
		hsi_release_port(ctx->client);
		return err;
	}

	hsi_register_port_event(ctx->client, ffl_hsi_ehandler);
	spin_lock_irqsave(&rx_ctx->lock, flags);
	_ffl_ctx_clear_flag(rx_ctx, TTY_OFF_BIT|ERROR_RECOVERY_RX_ERROR_BIT);
	err = _ffl_rx_push_controller(rx_ctx, &flags);
	spin_unlock_irqrestore(&rx_ctx->lock, flags);

#ifdef USE_IPC_ERROR_RECOVERY
	hsi_async(ctx->client, &ctx->recovery.rx_break);
#endif

	if (unlikely(err == -EAGAIN))
		mod_timer(&rx_ctx->timer, jiffies + rx_ctx->delay);

	/* Broadcast the port ready information */
	spin_lock_irqsave(&tx_ctx->lock, flags);
	ctx->hangup.tx_timeout = 0;
	_ffl_ctx_clear_flag(tx_ctx, TTY_OFF_BIT|ERROR_RECOVERY_TX_DRAINED_BIT);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

#ifdef USE_WAKE_POST_BOOT_HANDSHAKE
	/**
	* Execute only if not in flashing mode
	* <=> if !(tx_ctx->data_len<(FFL_DATA_LENGTH - 4))
	*/
	if (!(tx_ctx->data_len < (FFL_DATA_LENGTH - 4))) {
		/* Wait for modem post boot WAKE handshake before continuing */
		hsi_start_tx(ctx->client);
		wait_event_interruptible_timeout(ctx->modem_awake_event,
						 ffl_modem_is_awake(rx_ctx),
					 POST_BOOT_HANDSHAKE_TIMEOUT_JIFFIES);
		err = !ffl_modem_is_awake(rx_ctx);
		hsi_stop_tx(ctx->client);
		if (unlikely(err)) {
			pr_err(DRVNAME ": Modem wakeup failed (-EXDEV)");
			hsi_release_port(ctx->client);
			return -EXDEV;
		}
	}
#endif

	/* Set the TTY with low latency to bypass the system work queue */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	port->low_latency = 1;
#else
	tty->low_latency = 1;
#endif

	return 0;
}

/* Forward declarations for ffl_tty_port_shutdown() */
static void ffl_hangup_ctx_clear(struct ffl_hangup_ctx *ctx_hangup);
#ifdef USE_IPC_ERROR_RECOVERY
static void ffl_recovery_ctx_clear(struct ffl_recovery_ctx *ctx_recovery);
#endif

/**
 * ffl_tty_port_shutdown - callback to the TTY port shutdown function
 * @port: a reference to the calling TTY port
 *
 * The TTY port shutdown is only called on the last port close.
 */
static void ffl_tty_port_shutdown(struct tty_port *port)
{
	struct ffl_ctx		*ctx;
	struct ffl_xfer_ctx	*tx_ctx;
	struct ffl_xfer_ctx	*rx_ctx;
	unsigned long		flags;

	ctx = container_of(port, struct ffl_ctx, tty_prt);
	tx_ctx = &ctx->tx;
	rx_ctx = &ctx->rx;

	/* Broadcast the shutdown information */
	spin_lock_irqsave(&tx_ctx->lock, flags);
	_ffl_ctx_set_flag(tx_ctx, TTY_OFF_BIT);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);
	spin_lock_irqsave(&rx_ctx->lock, flags);
	_ffl_ctx_set_flag(rx_ctx, TTY_OFF_BIT);
	spin_unlock_irqrestore(&rx_ctx->lock, flags);

	/* Wait for TX write pipeline to be at least partially cleaned */
	wait_event_interruptible(ctx->tx_write_pipe_clean_event,
				 ffl_tx_write_pipe_is_clean(&ctx->tx));
	ffl_wait_until_ctx_sent(ctx, 0);

#ifdef USE_IPC_ERROR_RECOVERY
	/* Wait for error recovery completion */
	ffl_recovery_ctx_clear(&ctx->recovery);
#endif

	/* Wait for hangup completion */
	ffl_hangup_ctx_clear(&ctx->hangup);

	/* Flush any pending fw work */
	flush_work_sync(&ctx->do_tty_forward);

	pr_warn(DRVNAME ": port shutdown (flushing the HSI controller)");
	hsi_flush(ctx->client);

	del_timer_sync(&rx_ctx->timer);
	spin_lock_irqsave(&rx_ctx->lock, flags);
	_ffl_rx_fifo_wait_recycle(rx_ctx);
	_ffl_stop_rx(rx_ctx, ctx);
	spin_unlock_irqrestore(&rx_ctx->lock, flags);

	del_timer_sync(&tx_ctx->timer);
	spin_lock_irqsave(&tx_ctx->lock, flags);
	_ffl_tx_fifo_wait_recycle(tx_ctx);
	_ffl_stop_tx(tx_ctx);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

	/* Flush the ACWAKE works */
	flush_work_sync(&ctx->start_tx);
	flush_work_sync(&ctx->stop_tx);

	pr_warn(DRVNAME ": port shutdown (releasing the HSI controller)");
	hsi_unregister_port_event(ctx->client);
	hsi_release_port(ctx->client);
}

/**
 * ffl_tty_open - callback to the TTY open function
 * @tty: a reference to the calling TTY
 * @filp: a reference to the calling file
 *
 * Return 0 on success or a negative error code on error.
 *
 * The HSI layer is only initialised during the first opening.
 */
static int ffl_tty_open(struct tty_struct *tty, struct file *filp)
{
	struct ffl_ctx		*ctx;
	int			 err;

	/* Get the context reference from the driver data if already opened */
	ctx = (struct ffl_ctx *) tty->driver_data;

	/* Otherwise parse the context list to find the correct one */
	if (!ctx) {
		ctx = ffl_drv.ctx[tty->index];
		tty->driver_data	= ctx;
	}

	if (unlikely(!ctx)) {
		err = -ENODEV;
		pr_err(DRVNAME ": Cannot find TTY context (%d)", err);
		return err;
	}

	/* Open the TTY port (calls port->activate on first opening) */
	err = tty_port_open(&ctx->tty_prt, tty, filp);
	if (unlikely(err))
		pr_err(DRVNAME ": TTY open failed (%d)", err);

	/* Set the TTY_NO_WRITE_SPLIT to transfer as much data as possible on
	 * the first write request. This shall not introduce denial of service
	 * as this flag will later adapt to the available TX buffer size. */
	set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);

	return err;
}

/**
 * ffl_flush_tx_buffer - flushes the TX waiting FIFO
 * @tty: a reference to the requesting TTY
 */
static void ffl_flush_tx_buffer(struct tty_struct *tty)
{
	struct ffl_ctx		*main_ctx = (struct ffl_ctx *) tty->driver_data;
	struct ffl_xfer_ctx	*ctx = &main_ctx->tx;
	unsigned long		 flags;

	spin_lock_irqsave(&ctx->lock, flags);
	_ffl_tx_fifo_wait_recycle(ctx);
	spin_unlock_irqrestore(&ctx->lock, flags);
}

/**
 * ffl_tty_tx_timeout - timer function for tx timeout hangup request
 * @param: a hidden reference to the main FFL context
 */
static void ffl_tty_tx_timeout(unsigned long int param)
{
	struct ffl_ctx *ctx = (struct ffl_ctx *)param;
	struct ffl_xfer_ctx *tx_ctx = &ctx->tx;
	unsigned long flags;
	int do_hangup = 0;

	spin_lock_irqsave(&tx_ctx->lock, flags);
	if (likely(tx_ctx->ctrl_len > 0)) {
		do_hangup = ((!_ffl_ctx_has_flag(tx_ctx, TTY_OFF_BIT)) &&
			     (!ctx->hangup.tx_timeout));
		ctx->hangup.tx_timeout = 1;
		wake_up(&ctx->tx_full_pipe_clean_event);
	}
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

	if (do_hangup) {
		pr_err(DRVNAME ": TX timeout");
		queue_work(ffl_hu_wq, &ctx->hangup.work);
	}
}

/**
 * ffl_do_tx_hangup - initiate a hangup (TX timeout, modem reset or core dump)
 * @work: a reference to work queue element
 *
 * Required since port shutdown calls a mutex that might sleep
 */
static void ffl_do_tx_hangup(struct work_struct *work)
{
	struct ffl_hangup_ctx	*hangup_ctx;
	struct ffl_ctx		*ctx;
	struct tty_struct	*tty;
	unsigned long		 flags;
	int			 exit;

	hangup_ctx	= container_of(work, struct ffl_hangup_ctx, work);
	ctx		= container_of(hangup_ctx, struct ffl_ctx, hangup);

	spin_lock_irqsave(&ctx->tx.lock, flags);
	exit = _ffl_ctx_has_flag(&ctx->tx, TTY_OFF_BIT);
	spin_unlock_irqrestore(&ctx->tx.lock, flags);
	if (unlikely(exit))
		return;

	tty		= tty_port_tty_get(&ctx->tty_prt);
	if (tty) {
		tty_vhangup(tty);
		tty_kref_put(tty);
	}
}

/**
 * ffl_tty_hangup - callback to a TTY hangup request
 * @tty: a reference to the requesting TTY
 */
static void ffl_tty_hangup(struct tty_struct *tty)
{
	struct ffl_ctx *ctx = (struct ffl_ctx *) tty->driver_data;

	tty_port_hangup(&ctx->tty_prt);
}

/**
 * ffl_wait_until_sent - callback to a TTY wait until sent request
 * @tty: a reference to the requesting TTY
 * @timeout: a timeout value expressed in jiffies
 */
static void ffl_wait_until_sent(struct tty_struct *tty, int timeout)
{
	struct ffl_ctx		*ctx = (struct ffl_ctx *) tty->driver_data;

	ffl_wait_until_ctx_sent(ctx, timeout);
}

/**
 * ffl_tty_close - callback to the TTY close function
 * @tty: a reference to the calling TTY
 * @filp: a reference to the calling file
 *
 * The HSI layer is only released during the last closing.
 */
static void ffl_tty_close(struct tty_struct *tty, struct file *filp)
{
	struct ffl_ctx		*main_ctx = (struct ffl_ctx *) tty->driver_data;

	if ((filp != NULL) && (likely(main_ctx != NULL)))
		tty_port_close(&main_ctx->tty_prt, tty, filp);
}

/**
 * do_ffl_tty_write - writes data coming from the TTY to the TX FIFO
 * @ctx: a reference to the considered TX context
 * @buf: the virtual address of the current input buffer (from TTY)
 * @len: the remaining buffer size
 *
 * Returns the total size of what has been transferred.
 *
 * This is a recursive function, the core of the TTY write callback function.
 */
static int do_ffl_tty_write(struct ffl_xfer_ctx *ctx, unsigned char *buf,
			    int len)
{
	struct hsi_msg		*frame;
	unsigned char		*frame_ptr;
	int			 offset, avail, copied;
	unsigned int		 updated_actual_len;
	unsigned long		 flags;

	offset	= 0;
	avail	= 0;

	spin_lock_irqsave(&ctx->lock, flags);

	if (unlikely(_ffl_ctx_has_flag(ctx, TTY_OFF_BIT))) {
		spin_unlock_irqrestore(&ctx->lock, flags);
		return 0;
	}

	frame = _ffl_fifo_tail_safe(&ctx->wait_frames);
	if (frame) {
		offset	= frame->actual_len;
#ifdef CONFIG_HSI_FFL_ENSURE_LAST_WORD_NULL
		avail	= min(ctx->data_len,
			      (unsigned int)(FFL_DATA_LENGTH - 4)) - offset;
#else
		avail	= ctx->data_len - offset;
#endif
	}

	if (avail == 0) {
		frame = _ffl_new_frame(ctx);
		if (frame) {
			offset = 0;
#ifdef CONFIG_HSI_FFL_ENSURE_LAST_WORD_NULL
			avail = min(ctx->data_len,
				    (unsigned int) (FFL_DATA_LENGTH - 4));
#else
			avail = ctx->data_len;
#endif
			_ffl_fifo_wait_push(ctx, frame);
		}
	}

	if (frame) {
		frame->status = HSI_STATUS_PENDING;
		/* Do a start TX on new frames only and after having marked
		 * the current frame as pending, e.g. don't touch ! */
		if (offset == 0)
			_ffl_start_tx(ctx);
	} else {
		_ffl_ctx_set_flag(ctx, TX_TTY_WRITE_PENDING_BIT);
#ifdef CONFIG_HSI_FFL_TTY_STATS
		ctx->overflow_cnt++;
#endif
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	if (!frame)
		return 0;

	copied = min(avail, len);
	frame_ptr = ffl_virt(frame);
	updated_actual_len = frame->actual_len + copied;
	ffl_set_length(frame_ptr, updated_actual_len);
	(void) memcpy(ffl_data_ptr(frame_ptr, offset), buf, copied);

	spin_lock_irqsave(&ctx->lock, flags);
	if (likely(frame->status != HSI_STATUS_ERROR)) {
		frame->actual_len	 = updated_actual_len;
		ctx->buffered		+= copied;
		ctx->room		-= copied;
		frame->status		 = HSI_STATUS_COMPLETED;
		if (!_ffl_ctx_is_state(ctx, IDLE))
			_ffl_pop_wait_push_ctrl(ctx, &flags);
	} else {
		/* ERROR frames have already been popped from the wait FIFO */
		_ffl_free_frame(ctx, frame, &flags);
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	return copied;
}

/**
 * ffl_tty_write - writes data coming from the TTY to the TX FIFO
 * @tty: a reference to the calling TTY
 * @buf: the virtual address of the current input buffer (from TTY)
 * @len: the TTY buffer size
 *
 * Returns the total size of what has been transferred in the TX FIFO
 *
 * This is the TTY write callback function.
 */
static int ffl_tty_write(struct tty_struct *tty, const unsigned char *buf,
			 int len)
{
	struct	ffl_ctx		*main_ctx =
				(struct ffl_ctx *) tty->driver_data;
	struct	ffl_xfer_ctx	*ctx = &main_ctx->tx;
	int			 enough_room;
	int			 already_copied, copied;
	unsigned char		*ptr;
	unsigned long		 flags;

	spin_lock_irqsave(&ctx->lock, flags);
	if (likely(!_ffl_ctx_has_flag(ctx, TX_TTY_WRITE_ONGOING_BIT))) {
		enough_room = (ctx->room >= len);
	} else {
		enough_room = -1;
		_ffl_ctx_set_flag(ctx, TX_TTY_WRITE_PENDING_BIT);
	}
	_ffl_ctx_set_flag(ctx, TX_TTY_WRITE_ONGOING_BIT);
	spin_unlock_irqrestore(&ctx->lock, flags);

	/* Prevent a new write if there is one currently ongoing */
	if (unlikely(enough_room < 0)) {
		pr_err("%s: write ongoing !\n", __func__);
		return 0;
	}
	if (enough_room)
		set_bit(TTY_NO_WRITE_SPLIT, &tty->flags);
	else
		clear_bit(TTY_NO_WRITE_SPLIT, &tty->flags);

	already_copied = 0;
	while (len > 0) {
		ptr = (unsigned char *) &buf[already_copied];
		copied = do_ffl_tty_write(ctx, ptr, len);
		if (copied == 0)
			break;
		already_copied	+= copied;
		len		-= copied;
	}

	spin_lock_irqsave(&ctx->lock, flags);
	_ffl_ctx_clear_flag(ctx, TX_TTY_WRITE_ONGOING_BIT);
	wake_up(&main_ctx->tx_write_pipe_clean_event);
	spin_unlock_irqrestore(&ctx->lock, flags);

	return already_copied;
}

/**
 * ffl_tty_write_room - returns the available buffer size on the TX FIFO
 * @tty: a reference to the calling TTY
 *
 * Returns the total available size in the TX wait FIFO.
 */
static int ffl_tty_write_room(struct tty_struct *tty)
{
	struct	ffl_xfer_ctx *ctx = &((struct ffl_ctx *) tty->driver_data)->tx;
	unsigned int	room;
	unsigned long	flags;

	spin_lock_irqsave(&ctx->lock, flags);
	if (likely(!_ffl_ctx_has_flag(ctx, TX_TTY_WRITE_ONGOING_BIT))) {
		room = ctx->room;
	} else {
		room = 0;
		_ffl_ctx_set_flag(ctx, TX_TTY_WRITE_PENDING_BIT);
	}
	spin_unlock_irqrestore(&ctx->lock, flags);

	return room;
}

/**
 * ffl_tty_chars_in_buffer - returns the size of the data hold in the TX FIFO
 * @tty: a reference to the calling TTY
 *
 * Returns the total size of data hold in the TX wait FIFO. It does not take
 * into account the data which has already been passed to the HSI controller
 * in both in software and hardware FIFO.
 */
static int ffl_tty_chars_in_buffer(struct tty_struct *tty)
{
	struct	ffl_xfer_ctx *ctx = &((struct ffl_ctx *) tty->driver_data)->tx;
	unsigned int	buffered;
	unsigned long	flags;

	spin_lock_irqsave(&ctx->lock, flags);
	buffered = ctx->buffered;
	spin_unlock_irqrestore(&ctx->lock, flags);

	return buffered;
}

/**
 * ffl_tty_ioctl - manages the IOCTL read and write requests
 * @tty: a reference to the calling TTY
 * @filp: a reference to the calling file
 * @cmd: the IOCTL command
 * @arg: the I/O argument to pass or retrieve data
 *
 * Returns 0 upon normal completion or the error code in case of an error.
 */
static int ffl_tty_ioctl(struct tty_struct *tty,
			 unsigned int cmd, unsigned long arg)
{
	struct ffl_ctx		*ctx = (struct ffl_ctx *) tty->driver_data;
	struct work_struct	*increase_pool = NULL;
	static struct workqueue_struct *increase_pool_wq;
	unsigned int		data;
#ifdef CONFIG_HSI_FFL_TTY_STATS
	struct hsi_ffl_stats	stats;
#endif
	unsigned long		flags;

	switch (cmd) {
	case FFL_TTY_RESET_TX:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		_ffl_tx_fifo_wait_recycle(&ctx->tx);
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		break;

	case FFL_TTY_RESET_RX:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		_ffl_rx_fifo_wait_recycle(&ctx->rx);
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		break;

	case FFL_TTY_GET_TX_STATE:
		data = ffl_ctx_get_state(&ctx->tx);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_GET_RX_STATE:
		data = ffl_ctx_get_state(&ctx->rx);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_WAIT_MAX:
		if (arg > 0) {
			spin_lock_irqsave(&ctx->tx.lock, flags);
			if (arg >  ctx->tx.wait_max)
				increase_pool = &ctx->tx.increase_pool;
			ctx->tx.wait_max = arg;
			increase_pool_wq = ffl_tx_wq;
			spin_unlock_irqrestore(&ctx->tx.lock, flags);
		} else {
			dev_dbg(&ctx->client->device,
				"Invalid TX wait FIFO size %li\n",
				arg);
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_TX_WAIT_MAX:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.wait_max;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_WAIT_MAX:
		if (arg > 0) {
			spin_lock_irqsave(&ctx->rx.lock, flags);
			if (arg > ctx->rx.ctrl_max)
				increase_pool = &ctx->rx.increase_pool;
			ctx->rx.wait_max = arg;
			increase_pool_wq = ffl_rx_wq;
			spin_unlock_irqrestore(&ctx->rx.lock, flags);
		} else {
			dev_dbg(&ctx->client->device,
				"Invalid RX wait FIFO size %li\n",
				arg);
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_RX_WAIT_MAX:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = ctx->rx.wait_max;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_CTRL_MAX:
		if (arg > 0) {
			spin_lock_irqsave(&ctx->tx.lock, flags);
			if (arg > ctx->tx.ctrl_max)
				increase_pool = &ctx->tx.increase_pool;
			ctx->tx.ctrl_max = arg;
			increase_pool_wq = ffl_tx_wq;
			spin_unlock_irqrestore(&ctx->tx.lock, flags);
		} else {
			dev_dbg(&ctx->client->device,
				"Invalid TX controller FIFO size %li\n",
				arg);
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_TX_CTRL_MAX:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.ctrl_max;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_CTRL_MAX:
		if (arg > 0) {
			spin_lock_irqsave(&ctx->rx.lock, flags);
			if (arg > ctx->rx.ctrl_max)
				increase_pool = &ctx->rx.increase_pool;
			ctx->rx.ctrl_max = arg;
			increase_pool_wq = ffl_rx_wq;
			spin_unlock_irqrestore(&ctx->rx.lock, flags);
		} else {
			dev_dbg(&ctx->client->device,
				"Invalid RX controller FIFO size %li\n",
				arg);
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_RX_CTRL_MAX:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = ctx->rx.ctrl_max;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_DELAY:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		ctx->tx.delay = from_usecs(arg);
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		break;

	case FFL_TTY_GET_TX_DELAY:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = jiffies_to_usecs(ctx->tx.delay);
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_DELAY:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		ctx->rx.delay = from_usecs(arg);
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		break;

	case FFL_TTY_GET_RX_DELAY:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = jiffies_to_usecs(ctx->rx.delay);
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_FLOW:
		switch (arg) {
		case HSI_FLOW_SYNC:
		case HSI_FLOW_PIPE:
			spin_lock_irqsave(&ctx->tx.lock, flags);
			ctx->tx.config.flow = arg;
			spin_unlock_irqrestore(&ctx->tx.lock, flags);
			break;
		default:
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_TX_FLOW:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.config.flow;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_FLOW:
		switch (arg) {
		case HSI_FLOW_SYNC:
		case HSI_FLOW_PIPE:
			spin_lock_irqsave(&ctx->rx.lock, flags);
			ctx->client->rx_cfg.flow = arg;
			spin_unlock_irqrestore(&ctx->rx.lock, flags);
			break;
		default:
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_RX_FLOW:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = ctx->rx.config.flow;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_MODE:
		switch (arg) {
		case HSI_MODE_STREAM:
		case HSI_MODE_FRAME:
			spin_lock_irqsave(&ctx->tx.lock, flags);
			ctx->tx.config.mode = arg;
			spin_unlock_irqrestore(&ctx->tx.lock, flags);
			break;
		default:
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_TX_MODE:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.config.mode;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_MODE:
		switch (arg) {
		case HSI_MODE_STREAM:
		case HSI_MODE_FRAME:
			spin_lock_irqsave(&ctx->rx.lock, flags);
			ctx->rx.config.mode = arg;
			spin_unlock_irqrestore(&ctx->rx.lock, flags);
			break;
		default:
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_RX_MODE:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = ctx->rx.config.mode;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_CHANNELS:
		if ((arg > 16) || (arg <= ctx->tx.channel))
			return -EINVAL;
		spin_lock_irqsave(&ctx->tx.lock, flags);
		ctx->tx.config.channels = arg;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		break;

	case FFL_TTY_GET_TX_CHANNELS:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.config.channels;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_CHANNELS:
		if ((arg > 16) || (arg <= ctx->rx.channel))
			return -EINVAL;
		spin_lock_irqsave(&ctx->rx.lock, flags);
		ctx->rx.config.channels = arg;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		break;

	case FFL_TTY_GET_RX_CHANNELS:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = ctx->rx.config.channels;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_CHANNEL:
		if (arg >= ctx->tx.config.channels)
			return -EINVAL;
		spin_lock_irqsave(&ctx->tx.lock, flags);
		ctx->tx.channel = arg;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		break;

	case FFL_TTY_GET_TX_CHANNEL:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.channel;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_CHANNEL:
		if (arg >= ctx->rx.config.channels)
			return -EINVAL;
		spin_lock_irqsave(&ctx->rx.lock, flags);
		ctx->rx.channel = arg;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		break;

	case FFL_TTY_GET_RX_CHANNEL:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = ctx->rx.channel;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_FRAME_LEN:
		if ((arg <= FFL_HEADER_LENGTH) || (arg > FFL_FRAME_LENGTH))
			return -EINVAL;
		spin_lock_irqsave(&ctx->tx.lock, flags);
		ctx->tx.data_len = ((arg+3)/4)*4 - FFL_HEADER_LENGTH;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		break;

	case FFL_TTY_GET_TX_FRAME_LEN:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.data_len + FFL_HEADER_LENGTH;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_RX_FRAME_LEN:
		if ((arg <= FFL_HEADER_LENGTH) || (arg > FFL_FRAME_LENGTH))
			return -EINVAL;
		spin_lock_irqsave(&ctx->rx.lock, flags);
		ctx->rx.data_len = ((arg+3)/4)*4 - FFL_HEADER_LENGTH;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		break;

	case FFL_TTY_GET_RX_FRAME_LEN:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		data = ctx->rx.data_len + FFL_HEADER_LENGTH;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_ARB_MODE:
		switch (arg) {
		case HSI_ARB_RR:
		case HSI_ARB_PRIO:
			spin_lock_irqsave(&ctx->tx.lock, flags);
			ctx->tx.config.arb_mode = arg;
			spin_unlock_irqrestore(&ctx->tx.lock, flags);
			break;
		default:
			return -EINVAL;
		}
		break;

	case FFL_TTY_GET_TX_ARB_MODE:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.config.arb_mode;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

	case FFL_TTY_SET_TX_FREQUENCY:
		if (arg == 0)
			return -EINVAL;
		spin_lock_irqsave(&ctx->tx.lock, flags);
		ctx->tx.config.speed = arg;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		break;

	case FFL_TTY_GET_TX_FREQUENCY:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		data = ctx->tx.config.speed;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return put_user(data, (unsigned int __user *) arg);
		break;

#ifdef CONFIG_HSI_FFL_TTY_STATS
	case FFL_TTY_RESET_TX_STATS:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		ctx->tx.data_sz		= 0;
		ctx->tx.frame_cnt	= 0;
		ctx->tx.overflow_cnt	= 0;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		break;

	case FFL_TTY_GET_TX_STATS:
		spin_lock_irqsave(&ctx->tx.lock, flags);
		stats.data_sz		= ctx->tx.data_sz;
		stats.frame_cnt		= ctx->tx.frame_cnt;
		stats.overflow_cnt	= ctx->tx.overflow_cnt;
		spin_unlock_irqrestore(&ctx->tx.lock, flags);
		return copy_to_user((void __user *) arg, &stats,
				    sizeof(stats));
		break;

	case FFL_TTY_RESET_RX_STATS:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		ctx->rx.data_sz		= 0;
		ctx->rx.frame_cnt	= 0;
		ctx->rx.overflow_cnt	= 0;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		break;

	case FFL_TTY_GET_RX_STATS:
		spin_lock_irqsave(&ctx->rx.lock, flags);
		stats.data_sz		= ctx->rx.data_sz;
		stats.frame_cnt		= ctx->rx.frame_cnt;
		stats.overflow_cnt	= ctx->rx.overflow_cnt;
		spin_unlock_irqrestore(&ctx->rx.lock, flags);
		return copy_to_user((void __user *) arg, &stats,
				    sizeof(stats));
		break;
#endif

	default:
		return -ENOIOCTLCMD;
}

	if (increase_pool)
		(void) queue_work(increase_pool_wq, increase_pool);

	return 0;
}

#ifdef USE_IPC_ERROR_RECOVERY
/*
 * Error recovery mechanisms
 */

/**
 * do_recovery_drain_unless - helper function for firing RX or TX drain work
 * @xfer_ctx: a reference to the requesting transfer context
 * @condition: set of flags for preventing queueing of the work
 */
static void do_recovery_drain_unless(struct ffl_xfer_ctx *xfer_ctx,
				     unsigned int condition)
{
	struct ffl_ctx *main_ctx = main_ctx(xfer_ctx);
	struct ffl_recovery_ctx	*recovery_ctx = &main_ctx->recovery;
	unsigned long		 flags;

	spin_lock_irqsave(&xfer_ctx->lock, flags);
	if (!_ffl_ctx_has_any_flag(xfer_ctx, condition)) {
		_ffl_ctx_set_flag(xfer_ctx, ERROR_RECOVERY_ONGOING_BIT);
		if (xfer_ctx == &main_ctx->rx) {
			del_timer(&recovery_ctx->rx_drain_timer);
			queue_work(ffl_rx_wq, &recovery_ctx->do_rx_drain);
		} else {
			queue_work(ffl_tx_wq, &recovery_ctx->do_tx_drain);
		}
	}
	spin_unlock_irqrestore(&xfer_ctx->lock, flags);
}

/**
 * ffl_recovery_rx_drain_timeout - timer function for RX drain recovery request
 * @param: a reference to the requesting main context
 */
static void ffl_recovery_rx_drain_timeout(unsigned long int param)
{
	struct ffl_xfer_ctx *rx_ctx = (struct ffl_xfer_ctx *) param;

	do_recovery_drain_unless(rx_ctx, ERROR_RECOVERY_ONGOING_BIT);
}

/**
 * ffl_destruct_break - callback for break frame destruction
 * @frame: a reference to the break frame to delete
 */
static void ffl_destruct_break(struct hsi_msg *frame)
{
	/* Do nothing ! */
}

/**
 * ffl_complete_rx_break - callback for RX break frame completion
 * @frame: a reference to the completed break frame
 */
static void ffl_complete_rx_break(struct hsi_msg *frame)
{
	struct ffl_xfer_ctx *rx_ctx = &((struct ffl_ctx *)frame->context)->rx;

	do_recovery_drain_unless(rx_ctx,
				 ERROR_RECOVERY_ONGOING_BIT|TTY_OFF_BIT);
}

/**
 * ffl_tx_is_drained - checks if the TX path has been drained or not
 * @ctx: a reference to the TX context to consider
 */
static __must_check int ffl_tx_is_drained(struct ffl_xfer_ctx *ctx)
{
	int ret;
	unsigned long flags;

	spin_lock_irqsave(&ctx->lock, flags);
	ret = _ffl_ctx_has_flag(ctx, ERROR_RECOVERY_TX_DRAINED_BIT);
	spin_unlock_irqrestore(&ctx->lock, flags);

	return ret;
}

/**
 * ffl_recovery_tx_drain - initiate a TX draining sequence
 * @work: a reference to work queue element
 */
static void ffl_recovery_tx_drain(struct work_struct *work)
{
	struct ffl_recovery_ctx	*recovery_ctx;
	struct ffl_ctx		*main_ctx;
	struct ffl_xfer_ctx	*rx_ctx, *tx_ctx;
	unsigned long		 flags;

	recovery_ctx = container_of(work, struct ffl_recovery_ctx, do_tx_drain);
	main_ctx = container_of(recovery_ctx, struct ffl_ctx, recovery);
	tx_ctx = &main_ctx->tx;
	rx_ctx = &main_ctx->rx;

	/* Prevent re-entry in the TX drain sequence and ensure that no more TX
	 * messages will be sent */
	spin_lock_irqsave(&tx_ctx->lock, flags);
	if (_ffl_ctx_has_flag(tx_ctx, ERROR_RECOVERY_ONGOING_BIT)) {
		spin_unlock_irqrestore(&tx_ctx->lock, flags);
		return;
	}
	_ffl_ctx_set_flag(tx_ctx, ERROR_RECOVERY_ONGOING_BIT);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

	/* Do a start_tx() to ensure ACWAKE is high */
	hsi_start_tx(main_ctx->client);

	/* Wait for the TX path to be empty with a timeout */
	ffl_wait_until_ctx_sent(main_ctx, RECOVERY_TX_DRAIN_TIMEOUT_JIFFIES);

	/* Send a break message */
	hsi_async(main_ctx->client, &main_ctx->recovery.tx_break);

	/* Set the TX drain as being complete */
	spin_lock_irqsave(&tx_ctx->lock, flags);
	_ffl_ctx_set_flag(tx_ctx, ERROR_RECOVERY_TX_DRAINED_BIT);
	wake_up(&recovery_ctx->tx_drained_event);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

	/* Schedule a RX drain if no BREAK has been received before a timeout */
	spin_lock_irqsave(&rx_ctx->lock, flags);
	if (!_ffl_ctx_has_flag(rx_ctx, ERROR_RECOVERY_ONGOING_BIT))
		mod_timer(&main_ctx->recovery.rx_drain_timer,
			  jiffies + RECOVERY_BREAK_RESPONSE_TIMEOUT_JIFFIES);
	spin_unlock_irqrestore(&rx_ctx->lock, flags);
}

/**
 * ffl_recovery_rx_drain - initiate a RX draining sequence
 * @work: a reference to work queue element
 */
static void ffl_recovery_rx_drain(struct work_struct *work)
{
	struct ffl_recovery_ctx	*recovery_ctx;
	struct ffl_ctx		*main_ctx;
	struct ffl_xfer_ctx	*rx_ctx, *tx_ctx;
	unsigned long		 flags;
	int			 err = 0;

	recovery_ctx = container_of(work, struct ffl_recovery_ctx, do_rx_drain);
	main_ctx = container_of(recovery_ctx, struct ffl_ctx, recovery);
	tx_ctx = &main_ctx->tx;
	rx_ctx = &main_ctx->rx;

	/* Ensure all received messages are tagged as error as they will be
	 * flushed and these messages shall not re-enter the controller */
	spin_lock_irqsave(&rx_ctx->lock, flags);
	_ffl_ctx_set_flag(rx_ctx, ERROR_RECOVERY_ONGOING_BIT);
	spin_unlock_irqrestore(&rx_ctx->lock, flags);

	/* Force CAREADY low not to receive further messages and flush what's
	 * in the controller */
	hsi_flush(main_ctx->client);

	/* Wait for the TX side to be drained prior starting the recovery */
	do_recovery_drain_unless(tx_ctx, ERROR_RECOVERY_ONGOING_BIT);
	wait_event_interruptible(recovery_ctx->tx_drained_event,
				 ffl_tx_is_drained(tx_ctx));

	/* Wait for a guard time to ensure a proper recovery on both sides */
	set_current_state(TASK_INTERRUPTIBLE);
	schedule_timeout(RECOVERY_TO_NORMAL_DELAY_JIFFIES);

	/* Error recovery is no longer in use in both directions */
	spin_lock_irqsave(&rx_ctx->lock, flags);
	_ffl_ctx_clear_flag(rx_ctx, ERROR_RECOVERY_ONGOING_BIT |
				    ERROR_RECOVERY_RX_ERROR_BIT);
	if (!_ffl_ctx_has_flag(rx_ctx, TTY_OFF_BIT)) {
		hsi_async(main_ctx->client, &recovery_ctx->rx_break);
		err = _ffl_rx_push_controller(rx_ctx, &flags);
	}
	spin_unlock_irqrestore(&rx_ctx->lock, flags);

	if (unlikely(err == -EAGAIN))
		mod_timer(&rx_ctx->timer, jiffies + rx_ctx->delay);

	spin_lock_irqsave(&tx_ctx->lock, flags);
	_ffl_ctx_clear_flag(tx_ctx, ERROR_RECOVERY_ONGOING_BIT |
				    ERROR_RECOVERY_TX_DRAINED_BIT);
	_ffl_pop_wait_push_ctrl(tx_ctx, &flags);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

	/* Do a stop_tx() to release ACWAKE forcing */
	hsi_stop_tx(main_ctx->client);

	/* Wake up the TTY to restart */
	ffl_tty_wakeup(main_ctx);
}

/**
 * ffl_recovery_ctx_init - initialises a recovery context after its creation
 * @ctx_recovery: a reference to the considered recovery context
 */
static void ffl_recovery_ctx_init(struct ffl_recovery_ctx *ctx_recovery)
{
	struct ffl_ctx *ctx =
			container_of(ctx_recovery, struct ffl_ctx, recovery);

	init_waitqueue_head(&ctx_recovery->tx_drained_event);
	init_timer(&ctx_recovery->rx_drain_timer);
	ctx_recovery->rx_drain_timer.function = ffl_recovery_rx_drain_timeout;
	ctx_recovery->rx_drain_timer.data = (unsigned long int) &ctx->rx;

	INIT_WORK(&ctx_recovery->do_tx_drain, ffl_recovery_tx_drain);
	INIT_WORK(&ctx_recovery->do_rx_drain, ffl_recovery_rx_drain);

	ctx_recovery->rx_break.cl		= ctx->client;
	ctx_recovery->rx_break.context		= ctx;
	ctx_recovery->rx_break.complete		= &ffl_complete_rx_break;
	ctx_recovery->rx_break.destructor	= &ffl_destruct_break;
	ctx_recovery->rx_break.ttype		= HSI_MSG_READ;
	ctx_recovery->rx_break.break_frame	= 1;

	ctx_recovery->tx_break.cl		= ctx->client;
	ctx_recovery->tx_break.context		= ctx;
	ctx_recovery->tx_break.complete		= &ffl_destruct_break;
	ctx_recovery->tx_break.destructor	= &ffl_destruct_break;
	ctx_recovery->tx_break.ttype		= HSI_MSG_WRITE;
	ctx_recovery->tx_break.break_frame	= 1;
}

/**
 * ffl_recovery_ctx_clear - clears a recovery context prior to its deletion
 * @ctx_recovery: a reference to the considered recovery context
 */
static void ffl_recovery_ctx_clear(struct ffl_recovery_ctx *ctx_recovery)
{
	/* Flush any running TX drain work (if started) first to trigger the
	 * RX drain timer if necessary then if this latter timer was pending,
	 * queue the RX drain work ASAP before flushing it (unless it has not
	 * been started in any case) */
	flush_work(&ctx_recovery->do_tx_drain);
	if (del_timer_sync(&ctx_recovery->rx_drain_timer))
		queue_work(ffl_rx_wq, &ctx_recovery->do_rx_drain);
	flush_work(&ctx_recovery->do_rx_drain);
}
#endif

/*
 * Hangup context initialisation and destruction helper functions
 */
/**
 * ffl_hangup_ctx_init - initialises a hangup context after its creation
 * @ctx_hangup: a reference to the considered hangup context
 */
static void ffl_hangup_ctx_init(struct ffl_hangup_ctx *ctx_hangup)
{
	struct ffl_ctx *ctx = container_of(ctx_hangup, struct ffl_ctx, hangup);

	init_timer(&ctx_hangup->timer);
	INIT_WORK(&ctx_hangup->work, ffl_do_tx_hangup);
	ctx_hangup->timer.function = ffl_tty_tx_timeout;
	ctx_hangup->timer.data = (unsigned long int) ctx;
}

/**
 * ffl_hangup_ctx_clear - clears a hangup context prior to its deletion
 * @ctx_hangup: a reference to the considered hangup context
 */
static void ffl_hangup_ctx_clear(struct ffl_hangup_ctx *ctx_hangup)
{
	struct ffl_ctx *ctx = container_of(ctx_hangup, struct ffl_ctx, hangup);
	struct ffl_xfer_ctx *tx_ctx = &ctx->tx;
	unsigned long flags;
	int is_hunging_up;

	spin_lock_irqsave(&tx_ctx->lock, flags);
	is_hunging_up = (ctx_hangup->tx_timeout);
	spin_unlock_irqrestore(&tx_ctx->lock, flags);

	/* No need to wait for the end of the calling work! */
	if (is_hunging_up)
		return;

	if (del_timer_sync(&ctx_hangup->timer))
		cancel_work_sync(&ctx_hangup->work);
	else
		flush_work(&ctx_hangup->work);
}

/*
 * RX/TX context initialisation and destruction helper functions
 */

/**
 * ffl_xfer_ctx_init - initialise a TX or RX context after its creation
 * @ctx: a reference to the considered TX or RX context
 * @main_ctx: a reference to its related main context
 * @channel: the HSI channel for FFL
 * @delay: the initial delay for the timer related to the TX or RX context
 * @wait_max: the maximal size of the wait FIFO for this context
 * @ctrl_max: the maximal size of the HSI controller FIFO for this context
 * @config: a reference to the default HSI interface configuration
 *
 * This helper function is simply filling in the initial data into a newly
 * created TX or RX context.
 */
static void ffl_xfer_ctx_init(struct ffl_xfer_ctx	*ctx,
			      struct ffl_ctx		*main_ctx,
			      unsigned int		channel,
			      unsigned int		delay,
			      unsigned int		wait_max,
			      unsigned int		ctrl_max,
			      const struct hsi_config	*config)
{
	INIT_LIST_HEAD(&ctx->wait_frames);
	INIT_LIST_HEAD(&ctx->recycled_frames);
	init_timer(&ctx->timer);
	spin_lock_init(&ctx->lock);
	ctx->timer.data	= (unsigned long) ctx;
	ctx->delay	= from_usecs(delay);
	ctx->wait_len	= 0;
	ctx->ctrl_len	= 0;
	ctx->all_len	= 0;
	ctx->state	= IDLE;
	ctx->wait_max	= wait_max;
	ctx->ctrl_max	= ctrl_max;
	ctx->buffered	= 0;
	ctx->room	= 0;
	ctx->main_ctx	= main_ctx;
	ctx->data_len	= FFL_DATA_LENGTH;
	ctx->channel	= channel;
	INIT_WORK(&ctx->increase_pool, ffl_increase_pool_of_frames);
#ifdef CONFIG_HSI_FFL_TTY_STATS
	ctx->data_sz		= 0;
	ctx->frame_cnt		= 0;
	ctx->overflow_cnt	= 0;
#endif
	ctx->config	= *config;
}

/**
 * ffl_xfer_ctx_clear - clears a TX or RX context prior to its deletion
 * @ctx: a reference to the considered TX or RX context
 *
 * This helper function is simply calling the relevant destructors and reseting
 * the context information.
 */
static void ffl_xfer_ctx_clear(struct ffl_xfer_ctx *ctx)
{
	struct ffl_ctx		*main_ctx = main_ctx(ctx);
	unsigned long		 flags;

	spin_lock_irqsave(&ctx->lock, flags);
	ctx->wait_max	= 0;
	ctx->ctrl_max	= 0;
	ctx->state	= IDLE;
	del_timer_sync(&ctx->timer);
	_ffl_fifo_frames_delete(&ctx->wait_frames, main_ctx);
	_ffl_fifo_frames_delete(&ctx->recycled_frames, main_ctx);
	spin_unlock_irqrestore(&ctx->lock, flags);
	flush_work(&ctx->increase_pool);
}

/*
 * Protocol driver handling routines
 */

/*
 * ffl_termios_init - default termios initialisation
 */
static const struct ktermios ffl_termios_init = {
	.c_iflag	= 0,
	.c_oflag	= 0,
	.c_cflag	= B115200 | CS8,
	.c_lflag	= 0,
	.c_cc		= INIT_C_CC,
	.c_ispeed	= 0,
	.c_ospeed	= 0
};

/*
 * ffl_driver_tty_ops - table of supported TTY operations
 */
static const struct tty_operations ffl_driver_tty_ops = {
	.open			= ffl_tty_open,
	.close			= ffl_tty_close,
	.write			= ffl_tty_write,
	.write_room		= ffl_tty_write_room,
	.chars_in_buffer	= ffl_tty_chars_in_buffer,
	.ioctl			= ffl_tty_ioctl,
	.hangup			= ffl_tty_hangup,
	.wait_until_sent	= ffl_wait_until_sent,
	.unthrottle		= ffl_rx_forward_resume,
	.flush_buffer		= ffl_flush_tx_buffer,
};

/*
 * ffl_port_tty_ops - table of supported TTY port operations
 */
static const struct tty_port_operations ffl_port_tty_ops = {
	.activate		= ffl_tty_port_activate,
	.shutdown		= ffl_tty_port_shutdown,
};

/**
 * ffl_driver_probe - creates a new context in the FFL driver
 * @dev: a reference to the HSI device requiring a context
 *
 * Returns 0 upon success or an error in case of an error
 *
 * This function is creating a new context per HSI controller requiring a
 * FFL protocol driver, creates the related TTY port and TTY entry in the
 * filesystem.
 */
static int ffl_driver_probe(struct device *dev)
{
	struct hsi_client	*client = to_hsi_client(dev);
	struct tty_port		*tty_prt;
	struct ffl_ctx		*ctx;
	int			i = 0;
	int			l = -1;
	int			err;

	dev_dbg(dev, "ffl_driver_probe entered\n");

	/* Get a free line number and check that the client is not already
	 * registered (is that possible anyway ?) */
	for (i = FFL_TTY_MAX_LINES-1; i >= 0; --i) {
		if (!(ffl_drv.ctx[i])) {
			l = i;
		} else {
			if (unlikely(ffl_drv.ctx[i]->client == client)) {
				dev_dbg(dev, "ignoring subsequent detection");
				return -ENODEV;
			}
		}
	}

	/* No line is available... */
	if (l < 0) {
		dev_dbg(dev, "no line available\n");
		return -ENODEV;
	}

	/* Create the main context */
	ctx = kzalloc(sizeof(struct ffl_ctx), GFP_KERNEL);
	if (unlikely(!ctx)) {
		pr_err(DRVNAME ": Cannot allocate main context");
		return -ENOMEM;
	}

	hsi_client_set_drvdata(client, (void *) ctx);

	ctx->index	= l;
	ctx->client	= client;

	/* The parent of our device is the HSI port, the parent of the HSI
	 * port is the HSI controller device */
	ctx->controller	= dev->parent->parent;

	init_waitqueue_head(&ctx->tx_full_pipe_clean_event);
	init_waitqueue_head(&ctx->tx_write_pipe_clean_event);

	ffl_xfer_ctx_init(&ctx->tx, ctx, CONFIG_HSI_FFL_TTY_CHANNEL,
			  FFL_TX_DELAY, FFL_TX_WAIT_FIFO, FFL_TX_CTRL_FIFO,
			  &client->tx_cfg);
	ffl_xfer_ctx_init(&ctx->rx, ctx, CONFIG_HSI_FFL_TTY_CHANNEL,
			  FFL_RX_DELAY, FFL_RX_WAIT_FIFO, FFL_RX_CTRL_FIFO,
			  &client->rx_cfg);
	INIT_WORK(&ctx->do_tty_forward, ffl_do_tty_forward);

	INIT_WORK(&ctx->start_tx, ffl_do_start_tx);
	INIT_WORK(&ctx->stop_tx, ffl_do_stop_tx);

	ctx->tx.timer.function = ffl_stop_tx;
	ctx->rx.timer.function = ffl_rx_forward_retry;

	ffl_drv.ctx[l]	= ctx;

	/* Warn if no DMA capability has been found */
	if (!is_device_dma_capable(ctx->controller))
		pr_warn(DRVNAME ": HSI device is not DMA capable");

	/* Create the TTY port */
	tty_prt = &(ctx->tty_prt);
	tty_port_init(tty_prt);
	tty_prt->ops = &ffl_port_tty_ops;

	/* Register the TTY device */
#if LINUX_VERSION_CODE >= KERNEL_VERSION(3,9,0)
	if (unlikely(!tty_port_register_device(tty_prt, ffl_drv.tty_drv, 1, dev))) {
#else
	if (unlikely(!tty_register_device(ffl_drv.tty_drv, 1, dev))) {
#endif
		err = -EFAULT;
		pr_err(DRVNAME ": TTY device registration failed (%d)", err);
		goto no_tty_device_registration;
	}

	/* Initialise the hangup context */
	ffl_hangup_ctx_init(&ctx->hangup);

#ifdef USE_WAKE_POST_BOOT_HANDSHAKE
	init_waitqueue_head(&ctx->modem_awake_event);
#endif

#ifdef USE_IPC_ERROR_RECOVERY
	/* Initialise the error recovery context */
	ffl_recovery_ctx_init(&ctx->recovery);
#endif

	/* Allocate FIFO in background */
	(void) queue_work(ffl_tx_wq, &ctx->tx.increase_pool);
	(void) queue_work(ffl_rx_wq, &ctx->rx.increase_pool);

	dev_dbg(dev, "ffl_driver_probe completed\n");
	return 0;

no_tty_device_registration:
	hsi_client_set_drvdata(client, NULL);

	ffl_xfer_ctx_clear(&ctx->tx);
	ffl_xfer_ctx_clear(&ctx->rx);

	ffl_drv.ctx[ctx->index]	= NULL;
	kfree(ctx);

	return err;
}

/**
 * ffl_driver_removes - removes a context from the FFL driver
 * @dev: a reference to the device requiring the context
 *
 * Returns 0 on success or an error code
 *
 * This function is freeing all resources hold by the context attached to the
 * requesting HSI device.
 */
static int ffl_driver_remove(struct device *dev)
{
	struct hsi_client	*client = to_hsi_client(dev);
	struct ffl_ctx		*ctx =
				(struct ffl_ctx *) hsi_client_drvdata(client);

#ifdef USE_IPC_ERROR_RECOVERY
	ffl_recovery_ctx_clear(&ctx->recovery);
#endif

	ffl_hangup_ctx_clear(&ctx->hangup);

	tty_unregister_device(ffl_drv.tty_drv, ctx->index);

	hsi_unregister_port_event(client);
	hsi_client_set_drvdata(client, NULL);

	flush_work(&ctx->do_tty_forward);

	ffl_xfer_ctx_clear(&ctx->tx);
	ffl_xfer_ctx_clear(&ctx->rx);

	ffl_drv.ctx[ctx->index]	= NULL;
	kfree(ctx);

	return 0;
}

/*
 * Protocol driver main init / exit functions
 */

/*
 * ffl_driver_setup - configuration of the FFL driver
 */
static struct hsi_client_driver ffl_driver_setup = {
	.driver = {
		.name	= DRVNAME,
		.owner	= THIS_MODULE,
		.probe	= ffl_driver_probe,
		.remove	= ffl_driver_remove,
	},
};

/**
 * ffl_driver_init - initialises the FFL driver common parts
 *
 * Returns 0 on success or an error code
 */
static int __init ffl_driver_init(void)
{
	struct tty_driver	*tty_drv = NULL;
	int			 err;
	int			 i;

	/* Clear the initial context content */
	for (i = 0; i < FFL_TTY_MAX_LINES; i++)
		ffl_drv.ctx[i] = NULL;

	/* Create a single thread workqueue for serialising tx background
	 * tasks */
	ffl_tx_wq = alloc_workqueue(DRVNAME "-tq", WQ_UNBOUND, 1);
	if (unlikely(!ffl_tx_wq)) {
		pr_err(DRVNAME ": unable to create FFL TX-side workqueue");
		err = -EFAULT;
		goto no_tx_wq;
	}

	/* Create a high priority and single thread workqueue for serialising
	 * rx background tasks */
	ffl_rx_wq = alloc_workqueue(DRVNAME "-rq", WQ_HIGHPRI |
							WQ_NON_REENTRANT, 1);
	if (unlikely(!ffl_rx_wq)) {
		pr_err(DRVNAME ": unable to create FFL RX-side workqueue");
		err = -EFAULT;
		goto no_rx_wq;
	}

	/* Create a single thread workqueue for hangup background tasks */
	ffl_hu_wq = alloc_workqueue(DRVNAME "-hq", WQ_UNBOUND, 1);
	if (unlikely(!ffl_hu_wq)) {
		pr_err(DRVNAME ": unable to create FFL hangup workqueue");
		err = -EFAULT;
		goto no_hu_wq;
	}

	/* Allocate the TTY interface */
	tty_drv = alloc_tty_driver(FFL_TTY_MAX_LINES);
	if (unlikely(!tty_drv)) {
		pr_err(DRVNAME ": Cannot allocate TTY driver");
		err = -ENOMEM;
		goto no_tty_driver_allocation;
	}

	/* Configure the TTY */
	tty_drv->magic		= TTY_DRIVER_MAGIC;
	tty_drv->owner		= THIS_MODULE;
	tty_drv->driver_name	= DRVNAME;
	tty_drv->name		= TTYNAME;
	tty_drv->minor_start	= 0;
	tty_drv->num		= FFL_TTY_MAX_LINES;
	tty_drv->type		= TTY_DRIVER_TYPE_SERIAL;
	tty_drv->subtype	= SERIAL_TYPE_NORMAL;
	tty_drv->flags		= TTY_DRIVER_REAL_RAW |
				  TTY_DRIVER_DYNAMIC_DEV;
	tty_drv->init_termios	= ffl_termios_init;

	ffl_drv.tty_drv = tty_drv;

	tty_set_operations(tty_drv, &ffl_driver_tty_ops);

	/* Register the TTY prior to probing the HSI devices */
	err = tty_register_driver(tty_drv);
	if (unlikely(err)) {
		pr_err(DRVNAME ": TTY driver registration failed (%d)", err);
		goto no_tty_driver_registration;
	}

	/* Now, register the client */
	err = hsi_register_client_driver(&ffl_driver_setup);
	if (unlikely(err)) {
		pr_err(DRVNAME
		       ": error whilst registering the " DRVNAME " driver %d",
		       err);
		goto no_hsi_client_registration;
	}

	pr_debug(DRVNAME ": driver initialised");

	return 0;

no_hsi_client_registration:
	tty_unregister_driver(ffl_drv.tty_drv);
no_tty_driver_registration:
	put_tty_driver(ffl_drv.tty_drv);
	ffl_drv.tty_drv = NULL;
no_tty_driver_allocation:
	destroy_workqueue(ffl_hu_wq);
no_hu_wq:
	destroy_workqueue(ffl_rx_wq);
no_rx_wq:
	destroy_workqueue(ffl_tx_wq);
no_tx_wq:
	return err;
}

/**
 * ffl_driver_exit - frees the resources taken by the FFL driver common parts
 */
static void __exit ffl_driver_exit(void)
{
	hsi_unregister_client_driver(&ffl_driver_setup);

	tty_unregister_driver(ffl_drv.tty_drv);
	put_tty_driver(ffl_drv.tty_drv);
	ffl_drv.tty_drv = NULL;

	destroy_workqueue(ffl_hu_wq);
	destroy_workqueue(ffl_rx_wq);
	destroy_workqueue(ffl_tx_wq);

	pr_debug(DRVNAME ": driver removed");
}

module_init(ffl_driver_init);
module_exit(ffl_driver_exit);

/*
 * Module information
 */
MODULE_AUTHOR("Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>");
MODULE_AUTHOR("Faouaz TENOUTIT <faouazx.tenoutit@intel.com>");
MODULE_DESCRIPTION("Fixed frame length protocol on HSI driver");
MODULE_LICENSE("GPL");
MODULE_VERSION("1.0-HSI-FFL");
