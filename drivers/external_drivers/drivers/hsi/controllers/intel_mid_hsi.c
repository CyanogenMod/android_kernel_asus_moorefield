/*
 * intel_mid_hsi.c
 *
 * Implements the Intel HSI driver.
 *
 * Copyright (C) 2010 Nokia Corporation. All rights reserved.
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
 *
 * Contact: Olivier Stoltz-Douchet <olivierx.stoltz-douchet@intel.com>
 *          Faouaz Tenoutit <faouaz.tenoutit@intel.com>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin St, Fifth Floor, Boston, MA
 * 02110-1301 USA
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/irq.h>
#include <linux/slab.h>
#include <linux/clk.h>
#include <linux/dma-mapping.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/seq_file.h>
#include <linux/scatterlist.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/hsi/hsi.h>
#include <linux/hsi/intel_mid_hsi.h>
#include <linux/hsi/hsi_info_board.h>
#include <linux/debugfs.h>
#include <linux/delay.h>
#include <linux/pci.h>
#include <linux/ioport.h>
#include <linux/gpio.h>
#include <linux/lnw_gpio.h>
#include <asm/intel-mid.h>
#ifdef CONFIG_HAS_WAKELOCK
#include <linux/wakelock.h>
#endif
#include "hsi_arasan.h"
#include "hsi_dwahb_dma.h"

#define DRVNAME "hsi-ctrl"

#define HSI_MPU_IRQ_NAME	"HSI_CONTROLLER_IRQ"

#define HSI_RESETDONE_TIMEOUT	10	/* 10 ms */
#define HSI_RESETDONE_RETRIES	20	/* => max 200 ms waiting for reset */
#define HSI_BASE_FREQUENCY	200000	/* in KHz */

#define HSI_ACWAKE_DELAY	100		/* 100 usec */

#define IDLE_POLL_JIFFIES (usecs_to_jiffies(10000)) /* 10 ms */
#define IDLE_TO_SUSPEND_DELAY 10 /* 10 ms */
#define HSI_CLOCK_SETUP_DELAY_WAIT 20000 /* 20 ms according to HW spec */

#define HSI_BYTES_TO_FRAMES(x) (((x) + 3) >> 2)
#define HSI_FRAMES_TO_BYTES(x) ((x) << 2)

/* Set the following if the modem is crashing on ACREADY de-asssertion whilst
 * ACWAKE is asserted. */
#undef PREVENT_RX_SLEEP_WHEN_NOT_SUSPENDED

/* Set the following if wanting to schedule a later suspend on idle state */
#define SCHEDULE_LATER_SUSPEND_ON_IDLE


/* This maps each channel to a single bit in DMA and HSI channels busy fields */
#define DMA_BUSY(ch)		(1<<(ch))
#define QUEUE_BUSY(ch)		(1<<(ch))

/* TX, RX and PM states */
enum {
	TX_SLEEPING,
	TX_READY
};

enum {
	RX_SLEEPING,
	RX_READY,
	RX_CAN_SLEEP
};

enum {
	DEVICE_READY,
	DEVICE_SUSPENDED,
	DEVICE_AND_IRQ_SUSPENDED
};

/* Master DMA config low register configuration:
 * No multi-block support, no maximal burst, no AHB bus locking, hardware
 * handshaking and a priority set to the channel id */
#define HSI_DWAHB_CFG_LO_CFG(dma_ch) \
	(DWAHB_PRIORITY(dma_ch) | DWAHB_SUSPEND(0) | \
	 DWAHB_DST_SW_HANDSHAKE(0) | DWAHB_SRC_SW_HANDSHAKE(0) | \
	 DWAHB_LOCKING(0) | DWAHB_DST_HANDSHAKE_ACTIVE_LOW(0) | \
	 DWAHB_SRC_HANDSHAKE_ACTIVE_LOW(0) | \
	 DWAHB_MAX_AMBA_BURST(0) | DWAHB_SRC_RELOAD(0) | DWAHB_DST_RELOAD(0))

/* Master DMA config high register configuration:
 * Use the channel id handshaking interface, no link list status update, no
 * opcode / data only access, low latency, no prefetch */
#define HSI_DWAHB_CFG_HI_CFG(dma_ch) \
	(DWAHB_PREFETCH(0) | DWAHB_USE_FIFO(0) | DWAHB_DATA_ONLY | \
	 DWAHB_DST_STATUS_UPDATE(0) | DWAHB_SRC_STATUS_UPDATE(0) | \
	 DWAHB_SRC_HW_HANDSHAKE(dma_ch) | DWAHB_DST_HW_HANDSHAKE(dma_ch))

/* Master DMA control low register configuration:
 * No interrupt, 32-bit data, increment on both source and destination,
 * 32-word bursts, no scatter, no gather, slave DMA control, and link
 * listing (if requested) */
#define HSI_DWAHB_CTL_LO_CFG(is_tx, lli_enb) \
	(DWAHB_IRQ_ENABLE(0) | DWAHB_DST_WIDTH(32) | DWAHB_SRC_WIDTH(32) | \
	 DWAHB_DST_INC | DWAHB_SRC_INC | DWAHB_DST_BURST(32) | \
	 DWAHB_SRC_BURST(32) | DWAHB_SRC_GATHER(0) | DWAHB_DST_SCATTER(0) | \
	 DWAHB_IS_NOT_FLOW_CTL(is_tx) | \
	 DWAHB_DST_LINK_LIST(lli_enb) | \
	 DWAHB_SRC_LINK_LIST(lli_enb))

/* Maximum burst size of the HSI IP version 2 is 32 words,
 * but it is limited to 16 word by Tangier fabric */
#define HSI_DMA_MAX_BURST_SZ_V2		0x33333333

/**
 * struct intel_dma_lli - DMA link list structure
 * @sar: DMA source address
 * @dar: DMA destination address
 * @llp: next DMA link list entry reference
 * @ctl_lo: DMA control register lower word
 * @ctl_hi: DMA control register upper word
 */
struct intel_dma_lli {
	u32			 sar;
	u32			 dar;
	u32			 llp;
	u32			 ctl_lo;
	u32			 ctl_hi;
};

/**
 * struct intel_dma_lld - DMA link list descriptor structure
 * @ctrl: DMA link list descriptor control field
 * @size: DMA link list descriptor size field
 * @llp: next DMA link list descriptor pointer
 * @data: pointer to the DMA link list data buffer
 */
struct intel_dma_lld {
	u32			 ctrl;
	u32			 size;
	u32			 llp;
	u32			 data;
};

/**
 * struct intel_dma_lli_xfer - DMA transfer configuration using link listing
 * @blk: reference to the block being transferred
 * @llp_addr: DMA address of the first link list entry
 * @lli: array of DMA link list entries
 */
struct intel_dma_lli_xfer {
	struct scatterlist	*blk;
	dma_addr_t		 llp_addr;
	struct intel_dma_lli	 lli[0];
};

/**
 * struct intel_dma_plain_xfer - DMA transfer configuration without link list
 * @size: size of the transfer in 32-bit words
 * @src_addr: DMA source address
 * @dst_addr: DMA destination address
 */
struct intel_dma_plain_xfer {
	u32			 size;
	dma_addr_t		 src_addr;
	dma_addr_t		 dst_addr;
};

/**
 * struct intel_dma_xfer_v1 - Internal DMA transfer context for IP version 1
 * @mst_enable: master DMA enabling register
 * @slv_enable: slave DMA enabling register
 * @with_link_list: DMA transfer with link listing context
 * @without_link_list: DMA transfer without link listing context
 */
struct intel_dma_xfer_v1 {
	u32 mst_enable;
	u32 slv_enable;
	union {
		struct intel_dma_lli_xfer	 with_link_list;
		struct intel_dma_plain_xfer	 without_link_list;
	};
};

/**
 * struct intel_dma_xfer_v2 - Internal DMA transfer context for IP version 2
 * @llp_addr: DMA address of the first link list entry
 * @lld: array of DMA link list descriptors
 */
struct intel_dma_xfer_v2 {
	dma_addr_t llp_addr;
	struct intel_dma_lld lld[0];
};

/**
 * struct intel_dma_xfer - Internal DMA transfer context (IP version 1 or 2)
 * @msg: reference of the message being transferred
 * @v1: for use with version 1 IP
 * @v2: for use with version 2 IP
 */
struct intel_dma_xfer {
	struct hsi_msg *msg;
	union {
		struct intel_dma_xfer_v1 v1;
		struct intel_dma_xfer_v2 v2;
	};
};

/**
 * struct intel_dma_ctx - Internal DMA context
 * @sg_entries: the maximal number of link listing entries for this context
 * @ongoing: reference to ongoing DMA transfer
 * @ready: reference to next ready DMA transfer
 */
struct intel_dma_ctx {
	int			 sg_entries;
	struct intel_dma_xfer	*ongoing;
	struct intel_dma_xfer	*ready;
};

/**
 * struct intel_pio_ctx - Internal PIO context
 * @blk: reference to the block being processed
 * @offset: offset in the block being processed
 */
struct intel_pio_ctx {
	struct scatterlist	*blk;
	unsigned int		 offset;
};

/**
 * struct intel_xfer_ctx - Internal xfer context (PIO or DMA)
 * @dma: DMA context storage
 * @pio: PIO context storage
 */
struct intel_xfer_ctx {
	union {
		struct intel_dma_ctx	dma;
		struct intel_pio_ctx	pio;
	};
};

/**
 * struct intel_controller - Arasan HSI controller data
 * @dev: device associated to the controller (HSI controller)
 * @pdev: PCI dev* for HSI controller
 * @dmac: PCI dev* for master DMA controller
 * @ctrl_io: HSI I/O ctrl address
 * @dma_io: GDD I/O ctrl address
 * @irq: interrupt line index of the HSI controller
 * @irq_wake: hsi_cawake irq connected to gpio_wake
 * @gpio_wake: gpio used for out of band wake; mainly used on Tangier
 * @isr_tasklet: first-level high priority interrupt handling tasklet
 * @fwd_tasklet: second level response forwarding tasklet
 * @tx_idle_poll: timer for polling the TX side idleness
 * @rx_idle_poll: timer for polling the RX side idleness
 * @sw_lock: spinlock for accessing software FIFO
 * @hw_lock: spinlock for accessing hardware FIFO
 * @tx_queue: channel-indexed array of FIFO of messages awaiting transmission
 * @rx_queue: channel-indexed array of FIFO of messages awaiting reception
 * @brk_queue: FIFO for RX break messages
 * @fwd_queue: FIFO of messages awaiting of being forwarded back to client
 * @tx_ctx: Context for the ongoing TX transfers
 * @rx_ctx: Context for the ongoing RX transfers
 * @tx_queue_busy: bitmap of busy (frozen) TX queues
 * @rx_queue_busy: bitmap of busy (frozen) RX queues
 * @tx_dma_ch: mapping for TX HSI channel to dma channel (-1 if no DMA used)
 * @rx_dma_ch: mapping for RX HSI channel to dma channel (-1 if no DMA used)
 * @dma_ctx: mapping of DMA contexts to HSI channel contexts
 * @dma_running: bitfield of running DMA transactions
 * @dma_resumed: bitfield of resumed DMA transactions
 * @tx_state: current state of the TX side (0 for idle, >0 for ACWAKE)
 * @rx_state: current state of the RX state (0 for idle, 1 for ACWAKE)
 * @suspend_state: current power state (0 for powered, >0 for suspended)
 * @version: version of the HSI controller IP
 * @irq_status: current interrupt status register
 * @err_status: current error interrupt status register
 * @dma_status: current DMA interrupt status register
 * @irq_cfg: current interrupt configuration register
 * @err_cfg: current error interrupt configuration register
 * @clk_cfg: current clock configuration register
 * @prg_cfg: current program configuration register
 * @tx_fifo_sz: channel-indexed array of TX FIFO sizes expressed in HSI frames
 * @tx_fifo_th: channel-indexed array of TX FIFO thresholds in HSI frames
 * @rx_fifo_sz: channel-indexed array of TX FIFO sizes expressed in HSI frames
 * @rx_fifo_th: channel-indexed array of TX FIFO thresholds in HSI frames
 * @arb_cfg: current arbiter priority configuration register
 * @sz_cfg: current program1 configuration register
 * @ip_freq: HSI controller IP frequency in kHz
 * @tx_speed: HSI controller IP TX speed (<= ip_freq) in kHz
 * @brk_us_delay: Minimal BREAK sequence delay in us
 * @acwake_delay: Delay between ACWAKE assertion and data xfer
 * @stay_awake: Android wake lock for preventing entering low power mode
 * @dir: debugfs HSI root directory
 * @wakeup_packet_log: Enable/Disable the dump of the wakeup packet
 * @resumed: Set to TRUE when the HSI driver exit the resume state
 * @use_oob_cawake: Set to true if intended to be used
 * @hsi_wake_raised: Set to TRUE when interrupt fires on hsi_cawke gpio
 * @hsi_wake_disabled: Set to TRUE when interrupt of hsi_cawake gpio is disabled
 * @pm_do_reset: reset ARASAN IP in rtpm_resume if needed
 */
struct intel_controller {
	/* Devices and resources */
	struct device		*dev;
	struct device		*pdev;
	struct pci_dev		*dmac;
	void __iomem		*ctrl_io;
	void __iomem		*dma_io;
	unsigned int		 irq;
	/* GPIO and associated irq when using oob wake */
	unsigned int		 irq_wake;
	unsigned int		 gpio_wake;
	/* Dual-level interrupt tasklets */
	struct tasklet_struct	 isr_tasklet;
	struct tasklet_struct	 fwd_tasklet;
	u16			isr_tasklet_running;
	/* Timers for polling TX and RX states */
	struct timer_list	 tx_idle_poll;
	struct timer_list	 rx_idle_poll;
	/* Queues and registers access locks */
	spinlock_t		 sw_lock;
	spinlock_t		 hw_lock;
	/* Software FIFO */
	struct list_head	 tx_queue[HSI_MID_MAX_CHANNELS];
	struct list_head	 rx_queue[HSI_MID_MAX_CHANNELS];
	struct list_head	 brk_queue;
	struct list_head	 fwd_queue;
	struct intel_xfer_ctx	 tx_ctx[HSI_MID_MAX_CHANNELS];
	struct intel_xfer_ctx	 rx_ctx[HSI_MID_MAX_CHANNELS];
	u16			 tx_queue_busy;
	u16			 rx_queue_busy;
	/* Current DMA processed messages */
	s8			 tx_dma_ch[HSI_MID_MAX_CHANNELS];
	s8			 rx_dma_ch[HSI_MID_MAX_CHANNELS];
	struct intel_dma_ctx	*dma_ctx[DWAHB_CHAN_CNT];
	u16			 dma_running;
	u16			 dma_resumed;
	/* Current RX and TX states (0 for idle) */
	int			 tx_state;
	int			 rx_state;
	int			 suspend_state;
	/* HSI controller version */
	int			 version;
	/* HSI controller register images */
	u32			 irq_status;
	u32			 err_status;
	u32			 dma_status;
	/* HSI controller setup */
	u32			 irq_cfg;
	u32			 err_cfg;
	u32			 clk_cfg;
	u32			 prg_cfg;
	u16			 tx_fifo_sz[HSI_MID_MAX_CHANNELS];
	u16			 tx_fifo_th[HSI_MID_MAX_CHANNELS];
	u16			 rx_fifo_sz[HSI_MID_MAX_CHANNELS];
	u16			 rx_fifo_th[HSI_MID_MAX_CHANNELS];
	u32			 arb_cfg;
	u32			 sz_cfg;
	/* HSI controller IP frequency */
	unsigned int		 ip_freq;
	unsigned int		 tx_speed;
	unsigned int		 brk_us_delay;

	/* AWCAKE delay (in usec) */
	unsigned int acwake_delay;
#ifdef CONFIG_HAS_WAKELOCK
	/* Android PM support */
	struct wake_lock	 stay_awake;
#endif
#ifdef CONFIG_DEBUG_FS
	struct dentry		*dir;
#endif
	u16 wakeup_packet_log;
	u16 resumed;
	u16 hsi_wake_raised;
	u16 hsi_wake_disabled;
	u16 pm_do_reset;

	bool use_oob_cawake;
};

/* Disable the following to deactivate the runtime power management
 * (for debugging purpose only)
 * Runtime PM is enabled by default
 */
static unsigned int runtime_pm = 1;
module_param(runtime_pm, uint, 0644);

/*
 * Disable the following to prevent ACWAKE toggling (HSI PM)
 * (for debugging purpose only)
 * This also disbales runtime power management
 * HSI PM is enabled by default
 */
static unsigned int hsi_pm = 1;
module_param(hsi_pm, uint, 0644);

/*
 * Dump HSI wakeup packet
 *
 * Help debugging PnP/Power consumption issues
 * This will enable dumping the HSI wakeup packets
 */
static unsigned int wakeup_packet_len = CONFIG_HSI_WAKEUP_PACKET_DUMP_LEN;
module_param(wakeup_packet_len, uint, 0644);

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif


static inline unsigned int hsi_ioread32(struct intel_controller *intel_hsi,
		void __iomem *p)
{
	if (intel_hsi->suspend_state != DEVICE_READY) {
		pr_err(DRVNAME ": Invalid PM status to read into HSI controller");
		pr_err(DRVNAME ": tx_state = %d, rx_state = %d, suspend_state=%d",
				intel_hsi->tx_state, intel_hsi->rx_state,
				intel_hsi->suspend_state);
		BUG();
	}
	return ioread32(p);
}

static inline void hsi_iowrite32(struct intel_controller *intel_hsi,
u32 val, void __iomem *p)
{
	if (intel_hsi->suspend_state != DEVICE_READY) {
		pr_err(DRVNAME ": Invalid PM status to write from HSI controller");
		pr_err(DRVNAME ": tx_state = %d, rx_state = %d, suspend_state=%d",
				intel_hsi->tx_state, intel_hsi->rx_state,
				intel_hsi->suspend_state);
		BUG();
	}
	iowrite32(val, p);
}


static inline bool use_oob_cawake(struct intel_controller *intel_hsi)
{
	return intel_hsi->use_oob_cawake;
}

/* This function will:
 *   - Dump the msg data if the msg was received when
 *     the HSI driver was in sleep mode
 *   - Call the msg complete callback
 */
static inline void hsi_msg_complete(struct intel_controller *intel_hsi,
				struct hsi_msg *msg)
{
	if (!wakeup_packet_len)
		goto done;

	if (intel_hsi->wakeup_packet_log) {
		intel_hsi->wakeup_packet_log = 0;

		/* Any data do dump (ex: BREAK frame) ? */
		if (msg->sgt.sgl) {
			print_hex_dump(KERN_DEBUG,
				DRVNAME": WAKE", DUMP_PREFIX_ADDRESS,
				32, 4, sg_virt(msg->sgt.sgl),
				MIN(wakeup_packet_len, msg->actual_len), 1);
		}
	}

done:
	if (msg->complete)
		msg->complete(msg);
}

/**
 * is_using_link_list - checks if the DMA context is using link listing
 * @dma_ctx: a reference to the DMA context to query
 *
 * Returns 0 if not using link-listing or 1 if using it.
 */
static inline int is_using_link_list(struct intel_dma_ctx *dma_ctx)
{
	return (dma_ctx->sg_entries > 1);
}

/**
 * is_in_tx_frame_mode - checks if the HSI controller is set in frame mode
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns 0 if in stream mode or ARASAN_TX_FRAME_MODE if in frame mode.
 */
static inline int is_in_tx_frame_mode(struct intel_controller *intel_hsi)
{
	return intel_hsi->prg_cfg & ARASAN_TX_FRAME_MODE;
}

/**
 * hsi_tx_channel_count - getting the number of programmed HSI TX channels
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns the number of programmed HSI TX channels.
 */
static inline int hsi_tx_channel_count(struct intel_controller *intel_hsi)
{
	return ARASAN_TX_CHANNEL_CNT(intel_hsi->sz_cfg);
}

/**
 * hsi_rx_channel_count - getting the number of programmed HSI RX channels
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns the number of programmed HSI RX channels.
 */
static inline int hsi_rx_channel_count(struct intel_controller *intel_hsi)
{
	return ARASAN_RX_CHANNEL_CNT(intel_hsi->sz_cfg);
}

/**
 * hsi_enable_interrupt - enabling and signalling interrupts
 * @ctrl: the IO-based address of the HSI hardware
 * @version: the version of the Arasan IP
 * @irq_enable: the bitfield of interrupt sources to enable
 */
static inline void hsi_enable_interrupt(void __iomem *ctrl, int version,
					u32 irq_enable)
{
	iowrite32(irq_enable, ARASAN_REG(INT_STATUS_ENABLE));
	iowrite32(irq_enable, ARASAN_REG(INT_SIGNAL_ENABLE));
}

/**
 * hsi_enable_error_interrupt - enabling and signalling error interrupts
 * @ctrl: the IO-based address of the HSI hardware
 * @version: the version of the Arasan IP
 * @irq_enable: the bitfield of error interrupt sources to enable
 */
static inline void hsi_enable_error_interrupt(void __iomem *ctrl, int version,
					      u32 irq_enable)
{
	iowrite32(irq_enable, ARASAN_REG(ERR_INT_STATUS_ENABLE));
	iowrite32(irq_enable, ARASAN_REG(ERR_INT_SIGNAL_ENABLE));
}

/**
 * hsi_pm_wake_lock - acquire the wake lock whenever necessary
 * @intel_hsi: Intel HSI controller reference
 */
static inline void hsi_pm_wake_lock(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
#ifdef CONFIG_HAS_WAKELOCK
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	if (intel_hsi->suspend_state != DEVICE_READY)
		wake_lock(&intel_hsi->stay_awake);
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
#endif
}

/**
 * hsi_pm_runtime_get - getting a PM runtime reference to the HSI controller
 * @intel_hsi: Intel HSI controller reference
 *
 * This function is also getting the wake lock should wake lock is used.
 */
static void hsi_pm_runtime_get(struct intel_controller *intel_hsi)
{
	hsi_pm_wake_lock(intel_hsi);
	pm_runtime_get(intel_hsi->pdev);
}

/**
 * hsi_pm_runtime_get_sync - getting a synchronised PM runtime reference to the
 *			     HSI controller
 * @intel_hsi: Intel HSI controller reference
 *
 * This function is also getting the wake lock should wake lock is used.
 */
static void hsi_pm_runtime_get_sync(struct intel_controller *intel_hsi)
{
	hsi_pm_wake_lock(intel_hsi);
	pm_runtime_get_sync(intel_hsi->pdev);
}

/**
 * assert_acwake - asserting the ACWAKE line status
 * @intel_hsi: Intel HSI controller reference
 *
 * The actual ACWAKE assertion happens when tx_state was 0.
 */
static void assert_acwake(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	unsigned long flags;
	int do_wakeup = 0;
	int do_wait = 0;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	if (hsi_pm) {
		int version = intel_hsi->version;
		void __iomem *ctrl = intel_hsi->ctrl_io;

		do_wakeup = (intel_hsi->tx_state == TX_SLEEPING);
		if (do_wakeup) {
			intel_hsi->prg_cfg |= ARASAN_TX_ENABLE;
			if (intel_hsi->suspend_state == DEVICE_READY) {
				do_wait = intel_hsi->acwake_delay ? 1 : 0;
				iowrite32(intel_hsi->prg_cfg,
						ARASAN_REG(PROGRAM));
			}
		}
	}
	intel_hsi->tx_state++;
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	/* Wait only if needed */
	if (do_wait)
		udelay(intel_hsi->acwake_delay);

	if (do_wakeup)
		hsi_pm_runtime_get(intel_hsi);
}

/**
 * deassert_acwake - de-asserting the ACWAKE line status
 * @intel_hsi: Intel HSI controller reference
 *
 * The actual ACWAKE de-assertion happens only when tx_state reaches 0.
 *
 * Returns 1 on success or 0 if the tx_state count was already 0.
 */
static int deassert_acwake(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	/* The deassert AC wake function is also used in the release function
	 * so that it can be called more times than expected should some
	 * stop_tx calls happen simultaneously. This makes the code cleaner and
	 * more robust. */
	if (intel_hsi->tx_state <= TX_SLEEPING) {
		intel_hsi->tx_state = TX_SLEEPING;
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
		return 0;
	}

	--intel_hsi->tx_state;
	if (intel_hsi->tx_state == TX_SLEEPING)
		if (mod_timer(&intel_hsi->tx_idle_poll,
			    jiffies + IDLE_POLL_JIFFIES))
				pm_runtime_put(intel_hsi->pdev);

	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	return 1;
}

/**
 * tx_idle_poll - polling the TX FIFO emptiness
 * @param: hidden Intel HSI controller reference
 *
 * This polling timer is activated whenever tx_state is sleeping, and
 * re-activated until all internal TX FIFO are empty.
 */
static void tx_idle_poll(unsigned long param)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	struct intel_controller *intel_hsi = (struct intel_controller *) param;
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	unsigned long flags;
	int do_sleep = 0;

	if (!hsi_pm)
		return;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	if (intel_hsi->tx_state != TX_SLEEPING) {
		/* This timer has been called when tx_state == 0, if no longer
		 * to 0, then the pm_runtime_put has still to be taken */
		do_sleep = 1;
		goto tx_poll_out;
	}

	if (likely(intel_hsi->suspend_state == DEVICE_READY)) {
		/* Prevent TX sleep and ACWAKE de-assertion until not empty */
		do_sleep = ((ioread32(ARASAN_REG(HSI_STATUS))
				& ARASAN_ALL_TX_EMPTY) == ARASAN_ALL_TX_EMPTY);
		if (do_sleep) {
			intel_hsi->prg_cfg &= ~ARASAN_TX_ENABLE;
			iowrite32(intel_hsi->prg_cfg, ARASAN_REG(PROGRAM));
		} else {
			mod_timer(&intel_hsi->tx_idle_poll,
					jiffies + IDLE_POLL_JIFFIES);
		}
	}
tx_poll_out:
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	if (do_sleep)
		pm_runtime_put(intel_hsi->pdev);
}

/**
 * rx_idle_poll - polling the RX FIFO emptiness and CAWAKE line status
 * @param: hidden Intel HSI controller reference
 *
 * This polling timer is activated on CAWAKE interrupt, and re-activated
 * until the CAWAKE is low and all internal RX FIFO are empty.
 */
static void rx_idle_poll(unsigned long param)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	struct intel_controller *intel_hsi = (struct intel_controller *) param;
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	int schedule_rx_sleep = 0;
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);

	if (unlikely(intel_hsi->rx_state != RX_READY)) {
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
		return;
	}

	/* This shall almost never happen, but timer will be restarted on
	 * resume anyway */
	if (unlikely(intel_hsi->suspend_state != DEVICE_READY)) {
		pr_warn("hsi: poll whilst suspended !\n");
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
		return;
	}

	/* Prevent RX side disabling as long as CAWAKE is asserted or any RX
	 * hardware FIFO is not empty */
	if (ioread32(ARASAN_REG(HSI_STATUS)) &
	    (ARASAN_RX_WAKE|ARASAN_ANY_RX_NOT_EMPTY)) {
		mod_timer(&intel_hsi->rx_idle_poll,
			  jiffies + IDLE_POLL_JIFFIES);
	} else {
		if (!use_oob_cawake(intel_hsi))
			intel_hsi->irq_cfg |= ARASAN_IRQ_RX_WAKE;
		hsi_enable_interrupt(ctrl, version, intel_hsi->irq_cfg);
		intel_hsi->rx_state = RX_CAN_SLEEP;
		schedule_rx_sleep = 1;
	}
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	if (schedule_rx_sleep)
		tasklet_hi_schedule(&intel_hsi->isr_tasklet);
}

/**
 * has_enabled_acready - enable the ACREADY line
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns 1 if enabled or 0 if already enabled.
 */
static int has_enabled_acready(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem	*ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	unsigned long	 flags;
	int do_wakeup;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);

	/* Do not re-wakeup the device if already woken up */
	do_wakeup = (intel_hsi->rx_state == RX_SLEEPING);

	intel_hsi->prg_cfg |= ARASAN_RX_ENABLE;
	if (!use_oob_cawake(intel_hsi))
		intel_hsi->irq_cfg &= ~ARASAN_IRQ_RX_WAKE;
	intel_hsi->irq_cfg |= ARASAN_IRQ_RX_SLEEP(version);

	if ((do_wakeup) && (intel_hsi->suspend_state == DEVICE_READY)) {
		iowrite32(intel_hsi->prg_cfg, ARASAN_REG(PROGRAM));
		if (is_arasan_v1(version))
			mod_timer(&intel_hsi->rx_idle_poll,
					jiffies + IDLE_POLL_JIFFIES);
		else
			hsi_enable_interrupt(ctrl, version, intel_hsi->irq_cfg);
	}

	if (do_wakeup)
		intel_hsi->rx_state = RX_READY;
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	if (do_wakeup)
		hsi_pm_runtime_get(intel_hsi);

	return do_wakeup;
}

/**
 * has_disabled_acready - try to disable the ACREADY line
 * @intel_hsi: Intel HSI controller reference
 *
 * The actual RX disable can only happen if the CAWAKE line was low and all
 * RX hardware FIFO are empty, in which case rx_state has been set to
 * RX_CAN_SLEEP.
 *
 * Returns 1 if disabled or 0 if not.
 */
static int has_disabled_acready(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem	*ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	int do_sleep = 0;
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	if (intel_hsi->rx_state != RX_CAN_SLEEP) {
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
		return 0;
	}

	do_sleep = !(hsi_ioread32(intel_hsi, ARASAN_REG(HSI_STATUS)) &
		     (ARASAN_RX_WAKE|ARASAN_ANY_RX_NOT_EMPTY));
	if (likely(do_sleep)) {
#ifndef PREVENT_RX_SLEEP_WHEN_NOT_SUSPENDED
		intel_hsi->prg_cfg &= ~ARASAN_RX_ENABLE;
		hsi_iowrite32(intel_hsi, intel_hsi->prg_cfg,
				ARASAN_REG(PROGRAM));
#endif
		do_sleep = !(hsi_ioread32(intel_hsi, ARASAN_REG(HSI_STATUS)) &
			(ARASAN_RX_WAKE|ARASAN_ANY_RX_NOT_EMPTY));
		if (!(do_sleep)) {
#ifndef PREVENT_RX_SLEEP_WHEN_NOT_SUSPENDED
			intel_hsi->prg_cfg |= ARASAN_RX_ENABLE;
			hsi_iowrite32(intel_hsi, intel_hsi->prg_cfg,
					ARASAN_REG(PROGRAM));
#endif
		}
	}


	if (likely(do_sleep))
		intel_hsi->rx_state = RX_SLEEPING;
	else {
		if (is_arasan_v1(version)) {
			mod_timer(&intel_hsi->rx_idle_poll,
					jiffies + IDLE_POLL_JIFFIES);
		} else {
			intel_hsi->irq_cfg |= ARASAN_IRQ_RX_SLEEP(ARASAN_IP_V2);
			if (likely(intel_hsi->suspend_state == DEVICE_READY))
				hsi_enable_interrupt(ctrl, version,
						intel_hsi->irq_cfg);
		}
		intel_hsi->irq_status &= ~ARASAN_IRQ_RX_WAKE;
		intel_hsi->rx_state = RX_READY;
	}
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	if (do_sleep) {
		/* Wait for 2 us (more than 1 HSI frame at 20 MHz) to ensure
		 * that the CAREADY will not rise back too soon */
		udelay(2);
		pm_runtime_put(intel_hsi->pdev);
	}

	return do_sleep;
}

/**
 * force_disable_acready - force disable of the ACREADY line
 * @intel_hsi: Intel HSI controller reference
 */
static void force_disable_acready(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem *ctrl		= intel_hsi->ctrl_io;
	int version			= intel_hsi->version;
	struct hsi_controller *hsi	= to_hsi_controller(intel_hsi->dev);
	int do_sleep = 0;
	u32 irq_clr;
	unsigned int i;
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	do_sleep = (intel_hsi->rx_state != RX_SLEEPING);
	if (do_sleep) {
		intel_hsi->prg_cfg &= ~ARASAN_RX_ENABLE;
		hsi_iowrite32(intel_hsi, intel_hsi->prg_cfg,
				ARASAN_REG(PROGRAM));
		intel_hsi->rx_state = RX_SLEEPING;
	}

	/* Prevent the ACREADY change because of the CAWAKE toggling.
	 * The CAWAKE event interrupt shall be re-enabled whenever the
	 * RX fifo is no longer empty */
	del_timer(&intel_hsi->rx_idle_poll);
	if (use_oob_cawake(intel_hsi))
		irq_clr = ARASAN_IRQ_RX_SLEEP(version);
	else
		irq_clr = ARASAN_IRQ_RX_WAKE | ARASAN_IRQ_RX_SLEEP(version);
	intel_hsi->irq_status	&= ~irq_clr;
	intel_hsi->irq_cfg		&= ~irq_clr;
	if (likely(intel_hsi->suspend_state == DEVICE_READY)) {
		if (use_oob_cawake(intel_hsi)
				&& !intel_hsi->hsi_wake_disabled) {
			disable_irq(intel_hsi->irq_wake);
			intel_hsi->hsi_wake_disabled = 1;
		}
		hsi_enable_interrupt(ctrl, version, intel_hsi->irq_cfg);
		iowrite32(irq_clr, ARASAN_REG(INT_STATUS));
	}
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	if (do_sleep) {
		for (i = 0; i < hsi->num_ports; i++)
			hsi_event(hsi->port[i], HSI_EVENT_STOP_RX);
		pm_runtime_put(intel_hsi->pdev);
	}
}

/**
 * unforce_disable_acready - unforce a previously disabled ACREADY line
 * @intel_hsi: Intel HSI controller reference
 */
static void unforce_disable_acready(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem *ctrl		= intel_hsi->ctrl_io;
	int version			= intel_hsi->version;
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	if (unlikely((intel_hsi->rx_state == RX_SLEEPING) &&
		     (!(intel_hsi->irq_cfg & ARASAN_IRQ_RX_WAKE)))) {
		if (!use_oob_cawake(intel_hsi))
			intel_hsi->irq_cfg |= ARASAN_IRQ_RX_WAKE;
		else if (intel_hsi->hsi_wake_disabled) {
			enable_irq(intel_hsi->irq_wake);
			intel_hsi->hsi_wake_disabled = 0;
		}
		if (intel_hsi->suspend_state == DEVICE_READY)
			hsi_enable_interrupt(ctrl, version, intel_hsi->irq_cfg);
	}
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
}

/**
 * hsi_get_dma_channel - get DMA channel index from HSI channel index
 * @intel_hsi: Intel HSI controller reference
 * @msg: reference to the HSI message
 * @hsi_ch: HSI channel for which to find DMA channel
 *
 * Return the DMA channel associated with an HSI channel for read or write.
 * Return -1 in case there is no associated DMA channel.
 */
static int hsi_get_dma_channel(struct intel_controller *intel_hsi,
			       struct hsi_msg *msg, unsigned int hsi_ch)
{
	if ((hsi_ch >= HSI_MID_MAX_CHANNELS) || (!msg->sgt.nents))
		return -1;

	return (msg->ttype == HSI_MSG_READ) ?
		(int) intel_hsi->rx_dma_ch[hsi_ch] :
		(int) intel_hsi->tx_dma_ch[hsi_ch];
}

/**
 * hsi_set_master_dma_cfg - setting common parts of the master DMA registers
 * @dma: the IO-based address of the DMA hardware
 * @dma_ch: the DMA channel to configure
 * @is_tx: the direction of the DMA channel (TX=1, RX=0)
 * @lli_enb: link-list enable flag
 */
static inline void hsi_set_master_dma_cfg(void __iomem *dma,
						int dma_ch,
						int is_tx,
						int lli_enb)
{

	/* Disable link listing */
	iowrite32(0, HSI_DWAHB_LLP(dma, dma_ch));

	/* Set the common master DMA static configuration */
	iowrite32(HSI_DWAHB_CFG_LO_CFG(dma_ch),
		  HSI_DWAHB_CFG_LO(dma, dma_ch));

	iowrite32(HSI_DWAHB_CFG_HI_CFG(dma_ch),
		  HSI_DWAHB_CFG_HI(dma, dma_ch));

	iowrite32(HSI_DWAHB_CTL_LO_CFG(is_tx, lli_enb),
		  HSI_DWAHB_CTL_LO(dma, dma_ch));

	/* Nothing to transfer yet ! */
	iowrite32(0, HSI_DWAHB_CTL_HI(dma, dma_ch));
}

/**
 * hsi_disable_master_dma_cfg - disable the master DMA registers
 * @dma: the IO-based address of the DMA hardware
 */
static inline void hsi_disable_master_dma_cfg(void __iomem *dma)
{
	int i;

	/* Disable all DMA channels */
	iowrite32(DWAHB_CHAN_DISABLE(DWAHB_ALL_CHANNELS), HSI_DWAHB_CHEN(dma));
	iowrite32(DWAHB_DISABLE, HSI_DWAHB_DMACFG(dma));

	for (i = 0; i < DWAHB_CHAN_CNT; i++) {
		/* Disable link listing */
		iowrite32(0, HSI_DWAHB_LLP(dma, i));

		/* Suspend channel, set software handshaking and set an invalid
		 * hardware handshaking IF (-1) */
		iowrite32(DWAHB_SUSPEND(1) | DWAHB_SRC_SW_HANDSHAKE(1) |
			  DWAHB_DST_SW_HANDSHAKE(1), HSI_DWAHB_CFG_LO(dma, i));
		iowrite32(DWAHB_DATA_ONLY | DWAHB_SRC_HW_HANDSHAKE(-1) |
			  DWAHB_DST_HW_HANDSHAKE(-1), HSI_DWAHB_CFG_HI(dma, i));
		iowrite32(0, HSI_DWAHB_CTL_LO(dma, i));
		iowrite32(0, HSI_DWAHB_CTL_HI(dma, i));
	}
}

/**
 * hsi_ctrl_set_cfg - HSI controller hardware configure
 * @intel_hsi: Intel HSI controller reference
 * @do_reset: flag to reset or not ARASAN IP
 *
 * Reset the ARASAN IP if requested and Program the hardware in accordance
 * with the settings stored in the HSI controller software structure.
 *
 * Returns success or an error if it is not possible to reprogram the device.
 */
static int hsi_ctrl_set_cfg(struct intel_controller *intel_hsi, int do_reset)
{
	void __iomem	*ctrl	= intel_hsi->ctrl_io;
	void __iomem	*dma	= intel_hsi->dma_io;
	int		 version = intel_hsi->version;
	u32 sz, status, mask, fifo_th, fifo_sz, start, dma_arb, program_reg;
	int i;

	/* If the reset bit is set then nothing has been configured yet ! */
	if (intel_hsi->prg_cfg & ARASAN_RESET)
		return 0;

	/* Prepare the internal clock without enabling it */
	iowrite32(0, ARASAN_REG(PROGRAM));
	iowrite32((intel_hsi->clk_cfg & ~ARASAN_CLK_ENABLE),
		  ARASAN_REG(CLOCK_CTRL));
	/* reset the ARASAN IP only during setup */
	if (do_reset) {
		status = ioread32(ARASAN_REG(CLOCK_CTRL));
		i = 0;
		while (!(status & ARASAN_CLK_STABLE)) {
			i++;
			if (i > HSI_CLOCK_SETUP_DELAY_WAIT)
				return -ETIME;
			udelay(1);
			status = ioread32(ARASAN_REG(CLOCK_CTRL));
		}
		iowrite32(intel_hsi->clk_cfg, ARASAN_REG(CLOCK_CTRL));

		iowrite32(ARASAN_RESET, ARASAN_REG(PROGRAM));
		do {
			program_reg = ioread32(ARASAN_REG(PROGRAM));
		} while ((program_reg & ARASAN_RESET));

		iowrite32(0, ARASAN_REG(PROGRAM));
		iowrite32((intel_hsi->clk_cfg & ~ARASAN_CLK_ENABLE),
			  ARASAN_REG(CLOCK_CTRL));
	}

	/* Disable FIFO and configure fixed DMA registers */
	if (is_arasan_v1(version)) {
		int dma_ch, lli_enb;

		iowrite32(0, ARASAN_REG_V1(ctrl, TX_FIFO_SIZE));
		iowrite32(0, ARASAN_REG_V1(ctrl, RX_FIFO_SIZE));

		for (i = 0; i < DWAHB_CHAN_CNT; i++)
			iowrite32(0, ARASAN_CHN_V1(SDMA_CFG, i));

		iowrite32(DWAHB_ENABLE, HSI_DWAHB_DMACFG(dma));
		for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			dma_ch = intel_hsi->tx_dma_ch[i];
			lli_enb = is_using_link_list(&intel_hsi->tx_ctx[i].dma);
			if (dma_ch >= 0)
				hsi_set_master_dma_cfg(dma,
						dma_ch, 1, lli_enb);
		}

		for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			dma_ch = intel_hsi->rx_dma_ch[i];
			lli_enb = is_using_link_list(&intel_hsi->rx_ctx[i].dma);
			if (dma_ch >= 0)
				hsi_set_master_dma_cfg(dma,
						dma_ch, 0, lli_enb);
		}
	} else {
		for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			iowrite32(ARASAN_FIFO_DISABLE,
					ARASAN_CHN_V2(TX_FIFO_SIZE, i));
			iowrite32(ARASAN_THRES_DISABLE,
					ARASAN_CHN_V2(TX_FIFO_THRES, i));
			iowrite32(ARASAN_FIFO_DISABLE,
					ARASAN_CHN_V2(RX_FIFO_SIZE, i));
			iowrite32(ARASAN_THRES_DISABLE,
					ARASAN_CHN_V2(RX_FIFO_THRES, i));
		}

		for (i = 0, dma_arb = 0; i < HSI_MID_MAX_CHANNELS; i++)
			dma_arb |= (intel_hsi->tx_dma_ch[i] >= 0) << (i*4);
		iowrite32(dma_arb, ARASAN_REG_V2(ctrl, TX_DMA_ARB));

		for (i = 0, dma_arb = 0; i < HSI_MID_MAX_CHANNELS; i++)
			dma_arb |= (intel_hsi->rx_dma_ch[i] >= 0) << (i*4);
		iowrite32(dma_arb, ARASAN_REG_V2(ctrl, RX_DMA_ARB));

		iowrite32(HSI_DMA_MAX_BURST_SZ_V2,
				ARASAN_REG_V2(ctrl, TX_DMA_BST_SZ));
		iowrite32(HSI_DMA_MAX_BURST_SZ_V2,
				ARASAN_REG_V2(ctrl, RX_DMA_BST_SZ));
	}

	/* Enable the internal HSI clock */
	status = ioread32(ARASAN_REG(CLOCK_CTRL));
	i = 0;
	while (!(status & ARASAN_CLK_STABLE)) {
		i++;
		if (i > HSI_CLOCK_SETUP_DELAY_WAIT)
			return -ETIME;
		udelay(1);
		status = ioread32(ARASAN_REG(CLOCK_CTRL));
	}
	iowrite32(intel_hsi->clk_cfg, ARASAN_REG(CLOCK_CTRL));

	/* Configure the main controller parameters (except wake status) */
	if (!hsi_pm) {
		intel_hsi->prg_cfg |= ARASAN_TX_ENABLE;
		mask = intel_hsi->prg_cfg & ~ARASAN_RX_ENABLE;
	} else {
		mask = intel_hsi->prg_cfg;
		mask &= ~(ARASAN_TX_ENABLE|ARASAN_RX_ENABLE);
	}
	iowrite32(intel_hsi->prg_cfg & mask, ARASAN_REG(PROGRAM));

	/* Configure the number of HSI channels */
	iowrite32(intel_hsi->sz_cfg, ARASAN_REG(PROGRAM1));

	/* Configure the arbitration scheme */
	iowrite32(intel_hsi->arb_cfg, ARASAN_REG(ARBITER_PRIORITY));

	/* Configure the hardware FIFO */
	if (is_arasan_v1(version)) {
		/* Configure TX FIFO */
		for (i = 0, fifo_sz = 0, fifo_th = 0;
				i < HSI_MID_MAX_CHANNELS; i++) {
			sz = intel_hsi->tx_fifo_sz[i];
			fifo_th |= (intel_hsi->tx_fifo_th[i] > (sz/2)) << i;
			sz = (sz > 0) ? ilog2(sz) : 0;
			fifo_sz |= (sz << (4*i));
		}

		iowrite32(fifo_sz, ARASAN_REG_V1(ctrl, TX_FIFO_SIZE));
		iowrite32(fifo_th, ARASAN_REG_V1(ctrl, TX_FIFO_THRES));

		/* Configure RX FIFO */
		for (i = 0, fifo_sz = 0, fifo_th = 0;
				i < HSI_MID_MAX_CHANNELS; i++) {
			sz = intel_hsi->rx_fifo_sz[i];
			fifo_th |= (intel_hsi->rx_fifo_th[i] > (sz/2)) << i;
			sz = (sz > 0) ? ilog2(sz) : 0;
			fifo_sz |= (sz << (4*i));
		}

		iowrite32(fifo_sz, ARASAN_REG_V1(ctrl, RX_FIFO_SIZE));
		iowrite32(fifo_th, ARASAN_REG_V1(ctrl, RX_FIFO_THRES));

		/* Start the RX idle poll mechanism if RX is enabled */
		if (intel_hsi->prg_cfg & ARASAN_RX_ENABLE)
			mod_timer(&intel_hsi->rx_idle_poll,
					jiffies + IDLE_POLL_JIFFIES);
	} else {
		/* Configure TX FIFO */
		for (i = 0, start = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			sz = intel_hsi->tx_fifo_sz[i];
			if (sz == 0)
				continue;
			fifo_sz = start | ((start+sz-1) << 16);
			iowrite32(fifo_sz, ARASAN_CHN_V2(TX_FIFO_SIZE, i));
			start += sz;

			fifo_th = (intel_hsi->tx_fifo_th[i] << 16);
			iowrite32(fifo_th, ARASAN_CHN_V2(TX_FIFO_THRES, i));
		}

		/* Configure RX FIFO */
		for (i = 0, start = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			sz = intel_hsi->rx_fifo_sz[i];
			if (sz == 0)
				continue;
			fifo_sz = start | ((start+sz-1) << 16);
			iowrite32(fifo_sz, ARASAN_CHN_V2(RX_FIFO_SIZE, i));
			start += sz;

			fifo_th = sz | (intel_hsi->rx_fifo_th[i] << 16);
			iowrite32(fifo_th, ARASAN_CHN_V2(RX_FIFO_THRES, i));
		}
	}

	/* Enable then signal interrupts */
	hsi_enable_interrupt(ctrl, version, intel_hsi->irq_cfg);
	hsi_enable_error_interrupt(ctrl, version, intel_hsi->err_cfg);

	/* Enable the RX and TX parts if necessary */
	iowrite32(intel_hsi->prg_cfg, ARASAN_REG(PROGRAM));

	return 0;
}

/**
 * hsi_ctrl_resume - HSI controller hardware resume
 * @intel_hsi: Intel HSI controller reference
 * @rtpm: Called from PM (S3) or RTMP (S0iX) state machine
 *
 * Program the hardware back to its prior-suspend state and re-enable IRQ.
 *
 * Returns success or an error if it is not possible to reprogram the device.
 */
static int hsi_ctrl_resume(struct intel_controller *intel_hsi, int rtpm,
							int do_reset)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	if (intel_hsi->suspend_state != DEVICE_READY) {
		if (hsi_ctrl_set_cfg(intel_hsi, do_reset))
			err = -EAGAIN;

		intel_hsi->dma_resumed = 0;
		intel_hsi->suspend_state--;
	}

	if (intel_hsi->suspend_state != DEVICE_READY) {
		enable_irq(intel_hsi->irq);
		intel_hsi->suspend_state--;
	}

	if (!rtpm)
		intel_hsi->resumed = 1;

	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	return err;
}

/**
 * hsi_ctrl_suspend - HSI controller hardware suspend
 * @intel_hsi: Intel HSI controller reference
 * @rtpm: Called from PM (S3) or RTMP (S0iX) state machine
 *
 * Stops pending DMA transfers, disable all interrupts and shut down the
 * controller clock.
 *
 * Returns 0 if successful or an error code
 */
static int hsi_ctrl_suspend(struct intel_controller *intel_hsi, int rtpm)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem *ctrl	= intel_hsi->ctrl_io;
	void __iomem *dma	= intel_hsi->dma_io;
	int version = intel_hsi->version;
	int i, err = 0;
	unsigned long flags;
	u32 pending_interrupt = 0;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);

#ifdef PREVENT_RX_SLEEP_WHEN_NOT_SUSPENDED
	if (likely(intel_hsi->suspend_state == DEVICE_READY)) {
		if (!(ioread32(ARASAN_REG(HSI_STATUS))
				& ARASAN_RX_WAKE)) {
			intel_hsi->prg_cfg &= ~ARASAN_RX_ENABLE;
			iowrite32(intel_hsi->prg_cfg, ARASAN_REG(PROGRAM));
			if (unlikely(ioread32(ARASAN_REG(HSI_STATUS)) &
				     (ARASAN_RX_WAKE |
						ARASAN_ANY_RX_NOT_EMPTY))) {
				intel_hsi->prg_cfg |= ARASAN_RX_ENABLE;
				iowrite32(intel_hsi->prg_cfg,
						ARASAN_REG(PROGRAM));
				/* ACWAKE rising edge will be detected
				 * by the ISR */
				err = -EBUSY;
				pr_info(DRVNAME ": Prevent suspend (RX wake or RX not empty)\n");
				goto exit_ctrl_suspend;
			}
		} else {
			/* ACWAKE rising edge will be detected by the ISR */
			err = -EBUSY;
			pr_info(DRVNAME ": Prevent suspend (RX wake or RX not empty)\n");
			goto exit_ctrl_suspend;
		}
	}
#endif

	/* check whether the ISR_tasklet is running or is/will be scheduled */
	pending_interrupt = intel_hsi->isr_tasklet_running |
			intel_hsi->irq_status |
			intel_hsi->err_status |
			intel_hsi->dma_status;

	if ((intel_hsi->tx_state != TX_SLEEPING) ||
		(intel_hsi->rx_state != RX_SLEEPING) ||
				(pending_interrupt)) {
		err = -EBUSY;
		pr_info(DRVNAME ": Prevent suspend (RX state: %d, TX state: %d)\n",
				intel_hsi->rx_state,
				intel_hsi->tx_state);
		goto exit_ctrl_suspend;
	}

	/* Disable all DMA */
	if (is_arasan_v1(version)) {
		hsi_iowrite32(intel_hsi, DWAHB_CHAN_DISABLE(DWAHB_ALL_CHANNELS),
			  HSI_DWAHB_CHEN(dma));
		hsi_iowrite32(intel_hsi, DWAHB_DISABLE, HSI_DWAHB_DMACFG(dma));

		for (i = 0; i < DWAHB_CHAN_CNT; i++)
			hsi_iowrite32(intel_hsi, 0,
					ARASAN_CHN_V1(SDMA_CFG, i));
	} else {
		for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			hsi_iowrite32(intel_hsi, 0,
					ARASAN_CHN_V2(TX_CHAN_CURR_PTR, i));
			hsi_iowrite32(intel_hsi, 0,
					ARASAN_CHN_V2(RX_CHAN_CURR_PTR, i));
		}
	}

	intel_hsi->dma_running = 0;

	/* Disable all interrupts */
	if (likely(intel_hsi->suspend_state == DEVICE_READY)) {
		hsi_enable_interrupt(ctrl, version, 0);
		hsi_enable_error_interrupt(ctrl, version, 0);
	}

	/* Disable everyting and set the device as being suspended */
	hsi_iowrite32(intel_hsi, 0, ARASAN_REG(PROGRAM));
	intel_hsi->suspend_state = DEVICE_SUSPENDED;

	if (!rtpm)
		intel_hsi->resumed = 0;

#ifdef CONFIG_HAS_WAKELOCK
		wake_unlock(&intel_hsi->stay_awake);
#endif

exit_ctrl_suspend:
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	return err;
}

/**
 * do_free_dma_xfer - do free a single DMA transfer context
 * @dma_xfer: DMA transfer context reference to free
 * @sg_entries: maximal number of supported scatter gather entries
 * @intel_hsi: Intel HSI controller reference
 */
static void do_free_dma_xfer(struct intel_dma_xfer *dma_xfer, int sg_entries,
			     struct intel_controller *intel_hsi)
{
	int	version = intel_hsi->version;
	dma_addr_t dma_addr = 0;
	int size = 0;

	if (!dma_xfer)
		return;

	if (is_arasan_v1(version)) {
		if (sg_entries > 1) {
			size = sg_entries * sizeof(struct intel_dma_lli);
			dma_addr = dma_xfer->v1.with_link_list.llp_addr;
		}
	} else {
		if (sg_entries > 0) {
			size = sg_entries * sizeof(struct intel_dma_lld);
			dma_addr = dma_xfer->v1.with_link_list.llp_addr;
		}
	}

	if (size > 0)
		dma_unmap_single(intel_hsi->pdev, dma_addr,
				size, DMA_TO_DEVICE);

	kfree(dma_xfer);
}

/**
 * do_alloc_dma_xfer - allocate and initialise a single DMA transfer context
 * @is_tx: DMA channel direction (1 for TX, 0 for RX)
 * @hsi_ch: HSI channel number
 * @dma_ch: DMA channel number
 * @sg_entries: maximal number of supported scatter gather entries
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns a pointer to a newly created DMA channel or NULL if not possible
 */
static
struct intel_dma_xfer *do_alloc_dma_xfer(int is_tx,
					int hsi_ch, int dma_ch,
					int sg_entries,
					struct intel_controller *intel_hsi)
{
	int version = intel_hsi->version;
	struct intel_dma_xfer *dma_xfer;
	dma_addr_t dma_addr = 0;
	int size, sg_size;

	/* Compute the maximal full size for the buffer to allocate */
	size = 0;
	sg_size = 0;
	if (is_arasan_v1(version)) {
		if (sg_entries > 1) {
			size += offsetof(struct intel_dma_xfer_v1,
					with_link_list);
			sg_size = sg_entries * sizeof(struct intel_dma_lli);
			size += sg_size;
			size += offsetof(struct intel_dma_lli_xfer, lli);
		} else {
			size += offsetof(struct intel_dma_xfer_v1,
					without_link_list);
			size += sizeof(struct intel_dma_plain_xfer);
		}
	} else {
		if (sg_entries > 0) {
			size += offsetof(struct intel_dma_xfer_v2, lld);
			sg_size = sg_entries * sizeof(struct intel_dma_lld);
			size += sg_size;
		}
	}

	/* Allocate the buffer */
	if (unlikely(size == 0))
		return NULL;

	dma_xfer = kzalloc(size, GFP_ATOMIC);
	if (!dma_xfer)
		return NULL;

	/* Map any the scatter gather link lists */
	if (sg_size) {
		void *sg_ptr;
		if (is_arasan_v1(version))
			sg_ptr = &dma_xfer->v1.with_link_list.lli;
		else
			sg_ptr = &dma_xfer->v2.lld;

		dma_addr = dma_map_single(intel_hsi->pdev,
						sg_ptr, sg_size, DMA_TO_DEVICE);
		if (!dma_addr) {
			kfree(dma_xfer);
			return NULL;
		}
	}

	if (!is_arasan_v1(version)) {
		dma_xfer->v2.llp_addr = dma_addr;
		return dma_xfer;
	}

	if (sg_entries > 1) {
		int i;
		struct intel_dma_lli_xfer *lli_xfer;
		lli_xfer = &dma_xfer->v1.with_link_list;
		lli_xfer->llp_addr = dma_addr;

		for (i = 0; i < sg_entries; i++) {
			if (is_tx)
				lli_xfer->lli[i].dar =
						HSI_DWAHB_TX_ADDRESS(hsi_ch);
			else
				lli_xfer->lli[i].sar =
						HSI_DWAHB_RX_ADDRESS(hsi_ch);

			lli_xfer->lli[i].ctl_lo =
					HSI_DWAHB_CTL_LO_CFG(is_tx, 1);
		}
	} else {
		struct intel_dma_plain_xfer	*plain_xfer;
		plain_xfer = &dma_xfer->v1.without_link_list;

		if (is_tx)
			plain_xfer->dst_addr = HSI_DWAHB_TX_ADDRESS(hsi_ch);
		else
			plain_xfer->src_addr = HSI_DWAHB_RX_ADDRESS(hsi_ch);
	}

	dma_xfer->v1.mst_enable = DWAHB_CHAN_START(dma_ch);
	return dma_xfer;
}

/**
 * free_dma_xfer - Freeing a single DMA transfer context
 * @is_tx: DMA channel direction (1 for TX, 0 for RX)
 * @hsi_ch: HSI channel number
 * @dma_ch: DMA channel number
 * @intel_hsi: Intel HSI controller reference
 */
static void free_dma_xfer(int is_tx,
				int hsi_ch, int dma_ch,
				struct intel_controller *intel_hsi)
{
	struct intel_dma_ctx *dma_ctx;

	dma_ctx = (is_tx) ?
				&intel_hsi->tx_ctx[hsi_ch].dma :
				&intel_hsi->rx_ctx[hsi_ch].dma;

	do_free_dma_xfer(dma_ctx->ongoing, dma_ctx->sg_entries, intel_hsi);
	do_free_dma_xfer(dma_ctx->ready, dma_ctx->sg_entries, intel_hsi);
	dma_ctx->ongoing = NULL;
	dma_ctx->ready = NULL;

	if (is_arasan_v1(intel_hsi->version))
		intel_hsi->dma_ctx[dma_ch] = NULL;
}

/**
 * alloc_dma_xfer - allocating and initialising a single DMA transfer context
 * @is_tx: DMA channel direction (1 for TX, 0 for RX)
 * @hsi_ch: HSI channel number
 * @dma_ch: DMA channel number
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns 0 if successful or an error code
 */
static int alloc_dma_xfer(int is_tx,
				int hsi_ch, int dma_ch,
				struct intel_controller *intel_hsi)
{
	int version = intel_hsi->version;
	struct intel_dma_ctx *dma_ctx	= (is_tx) ?
					&intel_hsi->tx_ctx[hsi_ch].dma :
					&intel_hsi->rx_ctx[hsi_ch].dma;

	if (is_arasan_v1(version))
		intel_hsi->dma_ctx[dma_ch] = dma_ctx;

	dma_ctx->ongoing = do_alloc_dma_xfer(is_tx,
							hsi_ch, dma_ch,
							dma_ctx->sg_entries,
							intel_hsi);
	if (!dma_ctx->ongoing)
		goto exit_error;

	dma_ctx->ready = do_alloc_dma_xfer(is_tx,
							hsi_ch, dma_ch,
							dma_ctx->sg_entries,
							intel_hsi);
	/* Everything is OK */
	if (dma_ctx->ready)
		return 0;

	/* Error => Free the ongoing xfer */
	do_free_dma_xfer(dma_ctx->ongoing, dma_ctx->sg_entries, intel_hsi);
exit_error:
	if (is_arasan_v1(version))
		intel_hsi->dma_ctx[dma_ch] = NULL;

	return -ENOMEM;
}

/**
 * free_xfer_ctx - freeing all contexts
 * @intel_hsi: Intel HSI controller reference
 */
static void free_xfer_ctx(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	int i, dma_ch;
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
		dma_ch = intel_hsi->tx_dma_ch[i];
		if (dma_ch >= 0)
			free_dma_xfer(1, i, dma_ch, intel_hsi);

		dma_ch = intel_hsi->rx_dma_ch[i];
		if (dma_ch >= 0)
			free_dma_xfer(0, i, dma_ch, intel_hsi);
	}
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
}

/**
 * alloc_xfer_ctx - allocating and initialising xfer contexts
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns 0 if successful or an error code
 */
static int alloc_xfer_ctx(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	int i, dma_ch;
	unsigned long flags;
	int err = 0;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
		dma_ch = intel_hsi->tx_dma_ch[i];
		if (dma_ch >= 0) {
			err = alloc_dma_xfer(1, i, dma_ch, intel_hsi);
			if (err)
				break;
		}
		dma_ch = intel_hsi->rx_dma_ch[i];
		if (dma_ch >= 0) {
			err = alloc_dma_xfer(0, i, dma_ch, intel_hsi);
			if (err)
				break;
		}
	}
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	if (unlikely(err))
		free_xfer_ctx(intel_hsi);

	return err;
}

/**
 * hsi_ctrl_clean_reset - quick and clean hardware halt
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_ctrl_clean_reset(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem *ctrl	= intel_hsi->ctrl_io;
	void __iomem *dma	= intel_hsi->dma_io;
	int i, version = intel_hsi->version;
	unsigned long flags;
	u32 program_reg = 0;

	/* Disable the interrupt line */
	disable_irq(intel_hsi->irq);
	if (use_oob_cawake(intel_hsi))
		disable_irq(intel_hsi->irq_wake);

	/* Disable (and flush) all tasklets */
	tasklet_disable(&intel_hsi->isr_tasklet);
	tasklet_disable(&intel_hsi->fwd_tasklet);

	pr_debug(DRVNAME ": clean reset requested !\n");

	/* Deassert ACWAKE and ACREADY as shutting down */
	while (deassert_acwake(intel_hsi))
		;

	/* Remove the TX idle poll timer */
	if (del_timer_sync(&intel_hsi->tx_idle_poll))
		pm_runtime_put(intel_hsi->pdev);

	/* Remove the RX idle poll timer */
	del_timer_sync(&intel_hsi->rx_idle_poll);

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);

	/* If suspended then there is nothing to do on the hardware side */
	if (intel_hsi->suspend_state != DEVICE_READY)
		goto exit_clean_reset;

	/* Disable DMA */
	if (is_arasan_v1(version)) {
		for (i = 0; i < DWAHB_CHAN_CNT; i++)
			iowrite32(0, ARASAN_CHN_V1(SDMA_CFG, i));
		hsi_disable_master_dma_cfg(dma);
	} else {
		for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			iowrite32(0, ARASAN_CHN_V2(TX_CHAN_CURR_PTR, i));
			iowrite32(0, ARASAN_CHN_V2(RX_CHAN_CURR_PTR, i));
		}
	}

	/* Disable IRQ */
	hsi_enable_interrupt(ctrl, version, 0);
	hsi_enable_error_interrupt(ctrl, version, 0);

	/* Reset the HSI HW (Set HSI lines to low) */
	iowrite32(1, ARASAN_REG(PROGRAM));
	do {
		program_reg = ioread32(ARASAN_REG(PROGRAM));
	} while ((program_reg & ARASAN_RESET));

	/* Kill RX and TX wake sources and disable all channels */
	iowrite32(0, ARASAN_REG(PROGRAM));

exit_clean_reset:
	intel_hsi->dma_running	 = 0;
	intel_hsi->irq_status	 = 0;
	intel_hsi->err_status	 = 0;
	intel_hsi->prg_cfg	 = ARASAN_RESET;

	/* Free all contexts to restart from scratch */
	free_xfer_ctx(intel_hsi);

	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	force_disable_acready(intel_hsi);

	/* Re-enable all tasklets */
	tasklet_enable(&intel_hsi->fwd_tasklet);
	tasklet_enable(&intel_hsi->isr_tasklet);

	/* Do not forget to re-enable the interrupt */
	enable_irq(intel_hsi->irq);
	if (use_oob_cawake(intel_hsi))
		enable_irq(intel_hsi->irq_wake);
}

/**
 * hsi_ctrl_full_reset - hardware soft reset
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns 0 if successful, -EIO if it doesn't work.
 */
static int hsi_ctrl_full_reset(struct intel_controller *intel_hsi)
{
	void __iomem *ctrl = intel_hsi->ctrl_io;
	void __iomem *dma = intel_hsi->dma_io;
	int version = intel_hsi->version;
	u32 reset_ongoing = 1;
	unsigned int ip_freq;
	int retries = 0;

	/* Read the IP frequency once at the beginning (if defined) */
	ip_freq = ARASAN_TX_BASE_CLK_KHZ(ioread32(ARASAN_REG(CAPABILITY)));
	if (ip_freq == 0)
		ip_freq = HSI_BASE_FREQUENCY;
	intel_hsi->ip_freq = ip_freq;

	/* Perform software reset then loop until controller is ready */
	iowrite32(ARASAN_RESET, ARASAN_REG(PROGRAM));
	while ((reset_ongoing) && (retries < HSI_RESETDONE_RETRIES)) {
		set_current_state(TASK_UNINTERRUPTIBLE);
		schedule_timeout(msecs_to_jiffies(HSI_RESETDONE_TIMEOUT));
		reset_ongoing = ioread32(ARASAN_REG(PROGRAM)) &
				ARASAN_RESET;
		retries++;
	}
	iowrite32(0, ARASAN_REG(PROGRAM));

	if (reset_ongoing) {
		dev_err(intel_hsi->dev, "HSI reset failed");
		return -EIO;
	}

	/* Disable all master DMA channels */
	if (is_arasan_v1(version))
		hsi_disable_master_dma_cfg(dma);

	/* Tag the controller has being reset */
	intel_hsi->prg_cfg = ARASAN_RESET;

	return 0;
}

#ifdef CONFIG_DEBUG_FS

#define REG_OFFSET(r) (r-ctrl)

#define HSI_PRINT_REG(r)                           \
	seq_printf(m, "[0x%03X] " #r "\t\t: 0x%08x\n", \
		REG_OFFSET(ARASAN_REG(r)),                 \
		ioread32(ARASAN_REG(r)))

#define HSI_PRINT_REG_V1(r)                        \
	seq_printf(m, "[0x%03X] " #r "\t\t: 0x%08x\n", \
		REG_OFFSET(ARASAN_REG_V1(ctrl, r)),        \
		ioread32(ARASAN_REG_V1(ctrl, r)))

#define HSI_PRINT_REG_V2(r)                        \
	seq_printf(m, "[0x%03X] " #r "\t\t: 0x%08x\n", \
		REG_OFFSET(ARASAN_REG_V2(ctrl, r)),        \
		ioread32(ARASAN_REG_V2(ctrl, r)))

#define HSI_PRINT_CHN(r, ch)                         \
	seq_printf(m, "[0x%03X] " #r "%d\t\t: 0x%08x\n", \
		REG_OFFSET(ARASAN_CHN_REG(r, ch)),           \
		ch, ioread32(ARASAN_CHN_REG(r, ch)))

#define HSI_PRINT_CHN_V1(r, ch)                      \
	seq_printf(m, "[0x%03X] " #r "%d\t\t: 0x%08x\n", \
		REG_OFFSET(ARASAN_CHN_V1(r, ch)),            \
		ch, ioread32(ARASAN_CHN_V1(r, ch)))

#define HSI_PRINT_CHN_V2(r, ch)                    \
	seq_printf(m, "[0x%03X] " #r "%d\t: 0x%08x\n", \
		REG_OFFSET(ARASAN_CHN_V2(r, ch)),          \
		ch, ioread32(ARASAN_CHN_V2(r, ch)))

static int hsi_regs_show(struct seq_file *m, void *p)
{
	struct hsi_port *port = m->private;
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	int ch;

	/* PM */
	hsi_pm_runtime_get_sync(intel_hsi);
	seq_printf(m, "PM (HSI, RT)\t\t: %d, %d\n", hsi_pm, runtime_pm);

	/* Version */
	HSI_PRINT_REG(VERSION);

	/* FIFO configurations */
	if (is_arasan_v1(version)) {
		HSI_PRINT_REG_V1(TX_FIFO_SIZE);
		HSI_PRINT_REG_V1(TX_FIFO_THRES);
		HSI_PRINT_REG_V1(RX_FIFO_SIZE);
		HSI_PRINT_REG_V1(RX_FIFO_THRES);
	} else {
		for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
			HSI_PRINT_CHN_V2(TX_FIFO_SIZE, ch);
		for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
			HSI_PRINT_CHN_V2(TX_FIFO_THRES, ch);

		for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
			HSI_PRINT_CHN_V2(RX_FIFO_SIZE, ch);
		for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
			HSI_PRINT_CHN_V2(RX_FIFO_THRES, ch);
	}

	/* Main configuration setup */
	HSI_PRINT_REG(CLOCK_CTRL);
	HSI_PRINT_REG(PROGRAM);
	HSI_PRINT_REG(PROGRAM1);

	/* Current HSI controller status */
	HSI_PRINT_REG(HSI_STATUS);
	HSI_PRINT_REG(HSI_STATUS1);
	HSI_PRINT_REG(INT_STATUS);
	HSI_PRINT_REG(INT_STATUS_ENABLE);
	HSI_PRINT_REG(INT_SIGNAL_ENABLE);
	HSI_PRINT_REG(ERR_INT_STATUS);
	HSI_PRINT_REG(ERR_INT_STATUS_ENABLE);
	HSI_PRINT_REG(ERR_INT_SIGNAL_ENABLE);
	if (!is_arasan_v1(version))
		HSI_PRINT_REG_V2(DMA_INT_STATUS);

	/* HSI channel arbitration */
	HSI_PRINT_REG(ARBITER_PRIORITY);
	if (is_arasan_v1(version)) {
		HSI_PRINT_REG_V1(ARBITER_BANDWIDTH1);
		HSI_PRINT_REG_V1(ARBITER_BANDWIDTH2);
	}

	/* HSI controller capabilities */
	HSI_PRINT_REG(CAPABILITY);

	pm_runtime_put(intel_hsi->pdev);

	return 0;
}

#define HSI_GDD_PRINT(F) \
	seq_printf(m, #F "\t\t: 0x%08x\n", ioread32(HSI_DWAHB_ ## F(dma)))
#define HSI_GDD_PRINT_CHN(F, i) \
	seq_printf(m, #F " %d\t\t: 0x%08x\n", i,\
		   ioread32(HSI_DWAHB_ ## F(dma, i)))

static int hsi_dma_regs_show(struct seq_file *m, void *p)
{
	struct hsi_controller *hsi = m->private;
	struct intel_controller *intel_hsi = hsi_controller_drvdata(hsi);
	void __iomem *ctrl = intel_hsi->ctrl_io;
	void __iomem *dma = intel_hsi->dma_io;
	int version = intel_hsi->version;
	int ch;

	hsi_pm_runtime_get_sync(intel_hsi);

	if (is_arasan_v1(version)) {
		/* Global master DMA config */
		HSI_GDD_PRINT(DMACFG);
		HSI_GDD_PRINT(CHEN);

		/* Global master DMA status */
		HSI_GDD_PRINT(STATUSINT);
		HSI_GDD_PRINT(STATUSTFR);
		HSI_GDD_PRINT(STATUSBLOCK);
		HSI_GDD_PRINT(STATUSSRCTRAN);
		HSI_GDD_PRINT(STATUSDSTTRAN);
		HSI_GDD_PRINT(STATUSERR);

		HSI_GDD_PRINT(MASKTFR);
		HSI_GDD_PRINT(MASKBLOCK);
		HSI_GDD_PRINT(MASKSRCTRAN);
		HSI_GDD_PRINT(MASKDSTTRAN);
		HSI_GDD_PRINT(MASKERR);

		/* Slave and master DMA configurations per DMA channel */
		for (ch = 0; ch < DWAHB_CHAN_CNT; ch++) {
			HSI_PRINT_CHN_V1(SDMA_CFG, ch);
			HSI_GDD_PRINT_CHN(SAR, ch);
			HSI_GDD_PRINT_CHN(DAR, ch);
			HSI_GDD_PRINT_CHN(CTL_LO, ch);
			HSI_GDD_PRINT_CHN(CTL_HI, ch);
			HSI_GDD_PRINT_CHN(CFG_LO, ch);
			HSI_GDD_PRINT_CHN(CFG_HI, ch);
		}
	} else {
		/* Global DMA configuration */
		HSI_PRINT_REG_V2(TX_DMA_ARB);
		HSI_PRINT_REG_V2(RX_DMA_ARB);
		HSI_PRINT_REG_V2(TX_DMA_BST_SZ);
		HSI_PRINT_REG_V2(RX_DMA_BST_SZ);

		/* DMA configurations per TX and RX HSI channel */
		for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
			HSI_PRINT_CHN_V2(TX_CHAN_CURR_PTR, ch);

		for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
			HSI_PRINT_CHN_V2(RX_CHAN_CURR_PTR, ch);
	}

	pm_runtime_put(intel_hsi->pdev);
	return 0;
}

static void hsi_dump_queue(struct list_head *queue,
						int ch,
						struct seq_file *m)
{
	if (list_empty(queue))
		seq_printf(m, "   [%d]: Empty\n", ch);
	else {
		int i = 0;
		struct hsi_msg *msg;

		seq_printf(m, "   [%d]:\n", ch);
		list_for_each_entry(msg, queue, link)
			seq_printf(m, "      %02d: 0x%p\n", i++, msg);
	}
}

static int hsi_debug_show(struct seq_file *m, void *p)
{
	struct hsi_controller *hsi = m->private;
	struct intel_controller *intel_hsi = hsi_controller_drvdata(hsi);
	struct wakeup_source *ws = &intel_hsi->stay_awake.ws;
	int ch;

	/* Devices and resources */
	seq_printf(m, "PCI Region  : 0x%p\n", intel_hsi->ctrl_io);
	seq_printf(m, "DMA Region  : 0x%p\n", intel_hsi->dma_io);
	seq_printf(m, "IRQ number  : %d\n", intel_hsi->irq);
	seq_printf(m, "TX Frequen. : %d KHz\n", intel_hsi->tx_speed);
	seq_printf(m, "Break delay : %d\n", intel_hsi->brk_us_delay);

	/* Dump the DMA/PIO mapping */
	seq_puts(m, "TX DMA/PIO  : [");
	for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
		if (intel_hsi->tx_dma_ch[ch] == -1)
			seq_puts(m, "PIO  ");
		else
			seq_printf(m, "DMA%d  ", intel_hsi->tx_dma_ch[ch]);

	seq_puts(m, "]\nRX DMA/PIO  : [");
	for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
		if (intel_hsi->rx_dma_ch[ch] == -1)
			seq_puts(m, "PIO  ");
		else
			seq_printf(m, "DMA%d  ", intel_hsi->rx_dma_ch[ch]);

	/* Current RX and TX states (0 for idle) */
	seq_printf(m, "]\nHSI States  : RX:%d, TX:%d, Suspend:%d\n",
						intel_hsi->rx_state,
						intel_hsi->tx_state,
						intel_hsi->suspend_state);

	seq_printf(m, "DMA Running : 0x%08x\n", intel_hsi->dma_running);
	seq_printf(m, "DMA Resumed : 0x%08x\n", intel_hsi->dma_resumed);

#ifdef CONFIG_HAS_WAKELOCK
	/* Android PM support */
	seq_puts(m, "\nWAKELOCK INFO:\n");
	seq_printf(m, "   timer_expires: %ld\n",     ws->timer_expires);
	seq_printf(m, "   total_time   : %lld us\n",
			ktime_to_us(ws->total_time));
	seq_printf(m, "   max_time     : %lld us\n",
			ktime_to_us(ws->max_time));
	seq_printf(m, "   last_time    : %lld us\n",
			ktime_to_us(ws->last_time));
	seq_printf(m, "   start_prevent_time: %lld us\n",
			ktime_to_us(ws->start_prevent_time));
	seq_printf(m, "   prevent_sleep_time: %lld us\n",
			ktime_to_us(ws->prevent_sleep_time));
	seq_printf(m, "   event_count       : %ld\n", ws->event_count);
	seq_printf(m, "   active_count      : %ld\n", ws->active_count);
	seq_printf(m, "   relax_count       : %ld\n", ws->relax_count);
	seq_printf(m, "   expire_count      : %ld\n", ws->expire_count);
	seq_printf(m, "   wakeup_count      : %ld\n", ws->wakeup_count);
	seq_printf(m, "   active            : %d\n",  ws->active);
	seq_printf(m, "   autosleep_enabled : %d\n",  ws->autosleep_enabled);
#endif

	/* Software FIFO */
	seq_puts(m, "\nTX Queues:\n");
	for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
		hsi_dump_queue(&intel_hsi->tx_queue[ch], ch, m);

	seq_puts(m, "\nRX Queues:\n");
	for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++)
		hsi_dump_queue(&intel_hsi->rx_queue[ch], ch, m);

	seq_puts(m, "\nBREAK Queue:\n");
	hsi_dump_queue(&intel_hsi->brk_queue, 0, m);

	seq_puts(m, "\nFWD Queue:\n");
	hsi_dump_queue(&intel_hsi->fwd_queue, 0, m);
	return 0;
}

static int hsi_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsi_regs_show, inode->i_private);
}

static int hsi_dma_regs_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsi_dma_regs_show, inode->i_private);
}

static int hsi_debug_open(struct inode *inode, struct file *file)
{
	return single_open(file, hsi_debug_show, inode->i_private);
}

static const struct file_operations hsi_regs_fops = {
	.open		= hsi_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations hsi_dma_regs_fops = {
	.open		= hsi_dma_regs_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static const struct file_operations hsi_debug_fops = {
	.open		= hsi_debug_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= single_release,
};

static int hsi_debug_add_ctrl(struct hsi_controller *hsi)
{
	struct intel_controller *intel_hsi = hsi_controller_drvdata(hsi);
	struct dentry *dir;

	dir = debugfs_create_dir(dev_name(&hsi->device), NULL);
	if (IS_ERR(dir))
		return PTR_ERR(dir);

	/* HSI controller */
	debugfs_create_file("regs_hsi", S_IRUGO, dir, hsi, &hsi_regs_fops);

	/* HSI slave DMA */
	debugfs_create_file("regs_dma", S_IRUGO, dir, hsi, &hsi_dma_regs_fops);

	/* HSI useful debug information */
	debugfs_create_file("debug", S_IRUGO, dir, hsi, &hsi_debug_fops);

	intel_hsi->dir = dir;
	return 0;
}
#endif /* CONFIG_DEBUG_FS */

/**
 * do_hsi_prepare_dma_v1 - prepare a DMA context (for Arasan IP v1)
 * @msg: reference to the message
 * @dma_ctx: the DMA context to use
 * @intel_hsi: Intel HSI controller reference
 */
static void do_hsi_prepare_dma_v1(struct hsi_msg *msg,
				struct intel_dma_ctx *dma_ctx,
				struct intel_controller *intel_hsi)
{
	struct intel_dma_xfer *ready_xfer = dma_ctx->ready;
	struct intel_dma_lli_xfer *lli_xfer;
	struct intel_dma_plain_xfer *plain_xfer;
	struct sg_table *sgt = &msg->sgt;
	struct scatterlist *sgl = sgt->sgl;
	u32 rx_not_tx = (msg->ttype == HSI_MSG_READ);
	u32 size;
	struct scatterlist *sg;
	u32 len, next_llp;
	int i;

	if (is_using_link_list(dma_ctx)) {
		lli_xfer = &ready_xfer->v1.with_link_list;
		size = 0;
		next_llp = (u32) lli_xfer->llp_addr;
		lli_xfer->blk = sgl;

		for_each_sg(sgl, sg, sgt->nents, i) {
			if (!sg)	/* To please Klocwork */
				break;
			next_llp += sizeof(struct intel_dma_lli);
			len		  = HSI_BYTES_TO_FRAMES(sg->length);
			size	 += len;

			if (rx_not_tx)
				lli_xfer->lli[i].dar = sg_dma_address(sg);
			else
				lli_xfer->lli[i].sar = sg_dma_address(sg);
			lli_xfer->lli[i].llp = (sg_is_last(sg)) ? 0 : next_llp;
			lli_xfer->lli[i].ctl_hi = len;
		}

		msg->actual_len = HSI_FRAMES_TO_BYTES(size);
		size = 0; /* on slave, size is updated on each block xfer */
	} else {
		size = HSI_BYTES_TO_FRAMES(sgl->length);
		plain_xfer = &ready_xfer->v1.without_link_list;
		plain_xfer->size = size;
		if (rx_not_tx)
			plain_xfer->dst_addr = sg_dma_address(msg->sgt.sgl);
		else
			plain_xfer->src_addr = sg_dma_address(msg->sgt.sgl);

		msg->actual_len = HSI_FRAMES_TO_BYTES(size);
	}

	ready_xfer->v1.slv_enable = ARASAN_DMA_DIR(rx_not_tx) |
				 ARASAN_DMA_CHANNEL(msg->channel) |
				 ARASAN_DMA_XFER_FRAMES(size) |
				 ARASAN_DMA_BURST_SIZE(32) | ARASAN_DMA_ENABLE;

	ready_xfer->msg = msg;
}

/**
 * do_hsi_prepare_dma_v2 - prepare a DMA context (for Arasan IP v2)
 * @msg: reference to the message
 * @dma_ctx: the DMA context to use
 * @intel_hsi: Intel HSI controller reference
 */
static void do_hsi_prepare_dma_v2(struct hsi_msg *msg,
				  struct intel_dma_ctx *dma_ctx,
				  struct intel_controller *intel_hsi)
{
	struct intel_dma_xfer *ready_xfer = dma_ctx->ready;
	struct scatterlist *sg, *sgl;
	struct intel_dma_lld *lld;
	u32 len, size, next_llp, ctrl;
	int i, last;

	size = 0;
	sgl = msg->sgt.sgl;
	lld = ready_xfer->v2.lld;
	next_llp = (u32) ready_xfer->v2.llp_addr;

	for_each_sg(sgl, sg, msg->sgt.nents, i) {
		if (!sg)	/* To please Klocwork */
			break;
		next_llp += sizeof(struct intel_dma_lld);
		len		  = sg->length;
		size	 += len;

		last = sg_is_last(sg);
		ctrl = ARASAN_LLD_HARD;
		if (last)
			ctrl |= ARASAN_LLD_NO_CHAIN
				| ARASAN_LLD_IOC
				| ARASAN_LLD_LAST;
		else
			ctrl |= ARASAN_LLD_CHAINED;
		lld[i].ctrl = ctrl;
		lld[i].size = len;
		lld[i].llp  = last ? 0 : next_llp;
		lld[i].data = sg_dma_address(sg);
	}

	msg->actual_len = size;
	ready_xfer->msg = msg;
}

/**
 * do_hsi_prepare_dma - prepare a DMA context
 * @msg: reference to the message
 * @dma_ctx: the DMA context to use
 * @intel_hsi: Intel HSI controller reference
 */
static inline void do_hsi_prepare_dma(struct hsi_msg *msg,
					struct intel_dma_ctx *dma_ctx,
					struct intel_controller *intel_hsi)
{
	if (is_arasan_v1(intel_hsi->version))
		do_hsi_prepare_dma_v1(msg, dma_ctx, intel_hsi);
	else
		do_hsi_prepare_dma_v2(msg, dma_ctx, intel_hsi);
}

/**
 * try_hsi_prepare_dma - try preparing a ready DMA context
 * @queue: reference to queue ready to start transfer
 * @dma_ctx: the DMA context to use
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_try_prepare_dma(struct list_head *queue,
				struct intel_dma_ctx *dma_ctx,
				struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	unsigned long	 flags;
	struct hsi_msg	*ongoing_msg, *ready_msg;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);

	/* We are fine:
	 *  - If the DMA context was already released
	 *  - If some DMA context is already ready
	 */
	if (!dma_ctx || dma_ctx->ready->msg)
		goto all_dma_prepared;

	ongoing_msg = dma_ctx->ongoing->msg;
	if (ongoing_msg) {
		ready_msg = list_entry(ongoing_msg->link.next,
				       struct hsi_msg, link);
		/* Done if there is a single ongoing message in the queue */
		if (&(ready_msg->link) == queue)
			goto all_dma_prepared;
	} else {
		if (list_empty(queue))
			goto all_dma_prepared;
		ready_msg = list_first_entry(queue, struct hsi_msg, link);
	}

	if (likely(ready_msg->sgt.nents))
		do_hsi_prepare_dma(ready_msg, dma_ctx, intel_hsi);

all_dma_prepared:
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
}

/**
 * hsi_start_pio - start PIO data transfer
 * @msg: Pointer to message to transfer
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_start_pio(struct hsi_msg *msg,
			  struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	int hsi_ch = msg->channel;
	int version = intel_hsi->version;
	void __iomem	*ctrl = intel_hsi->ctrl_io;
	unsigned long	 flags;
	u32		 irq_en, err_en;

	irq_en = (msg->ttype == HSI_MSG_WRITE) ?
				ARASAN_IRQ_TX_THRESHOLD(hsi_ch) :
				ARASAN_IRQ_RX_THRESHOLD(hsi_ch);
	err_en = (msg->ttype == HSI_MSG_WRITE) ?
				0 : ARASAN_IRQ_DATA_TIMEOUT(hsi_ch);

	/* Enable the threshold reached signal */
	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	intel_hsi->irq_cfg |= irq_en;
	intel_hsi->err_cfg |= err_en;
	if (likely(intel_hsi->suspend_state == DEVICE_READY)) {
		hsi_enable_interrupt(ctrl, version, intel_hsi->irq_cfg);
		if (err_en)
			hsi_enable_error_interrupt(ctrl,
					version,
					intel_hsi->err_cfg);
	}
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
}

/**
 * do_hsi_start_dma - start/restart DMA data transfer helper function
 * @msg: reference to the message to transfer
 * @dma_ch: DMA channel to consider
 * @dma_ctx: the DMA context to use
 * @intel_hsi: Intel HSI controller reference
 * @resuming: flag stating if this is a first start or a resume restart
 */
static void do_hsi_start_dma(struct hsi_msg *msg,
				int dma_ch,
				struct intel_dma_ctx *dma_ctx,
				struct intel_controller *intel_hsi,
				int resuming)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem *ctrl		= intel_hsi->ctrl_io;
	void __iomem *dma		= intel_hsi->dma_io;
	struct intel_dma_xfer *dma_xfer;
	int version = intel_hsi->version;
	struct intel_dma_plain_xfer *plain_xfer;
	u32 mask = DMA_BUSY(dma_ch);
	u32 blk_len_overwrite = 0;
	unsigned int blk_len;
	unsigned long flags;
	int nothing_to_do;

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	dma_xfer = dma_ctx->ongoing;
	/* Check if the DMA IT was scheduled BUT not serverd just
	   before the calling hsi_ctrl_clean_reset (SMP config) */
	if (!dma_xfer) {
		pr_debug(DRVNAME ": Transfer dropped (RESET requested) !");
		goto do_start_dma_done;
	}

	if (resuming)
		nothing_to_do = ((intel_hsi->dma_resumed & mask) ||
				 (msg->status != HSI_STATUS_PROCEEDING));
	else
		nothing_to_do = ((intel_hsi->suspend_state != DEVICE_READY) ||
				(intel_hsi->dma_running & mask));

	if (nothing_to_do)
		goto do_start_dma_done;

	if (is_arasan_v1(version)) {
		unsigned int dma_bit;
		/* Wait for CH_EN bit is automatically cleared by hardware before setting next DMA */
		dma_bit = 1 << dma_ch;
		while (hsi_ioread32(intel_hsi, HSI_DWAHB_CHEN(dma)) & dma_bit)
			;
		if (is_using_link_list(dma_ctx)) {
			struct intel_dma_lli_xfer *lli_xfer;

			/* Set the link list pointer */
			lli_xfer = &dma_xfer->v1.with_link_list;
			iowrite32(lli_xfer->llp_addr,
					HSI_DWAHB_LLP(dma, dma_ch));

			/* Overwrite the block length */
			blk_len = HSI_BYTES_TO_FRAMES(lli_xfer->blk->length);
			blk_len_overwrite = ARASAN_DMA_XFER_FRAMES(blk_len);
		} else {
			/* Clear 'done' bit & set the xfer size and addresses */
			plain_xfer = &dma_xfer->v1.without_link_list;
			iowrite32(plain_xfer->size,
					HSI_DWAHB_CTL_HI(dma, dma_ch));
			iowrite32(plain_xfer->src_addr,
					HSI_DWAHB_SAR(dma, dma_ch));
			iowrite32(plain_xfer->dst_addr,
					HSI_DWAHB_DAR(dma, dma_ch));
		}

		/* Enable slave then master DMA to start the transfer */
		iowrite32(dma_xfer->v1.slv_enable | blk_len_overwrite,
				ARASAN_CHN_V1(SDMA_CFG, dma_ch));
		iowrite32(dma_xfer->v1.mst_enable, HSI_DWAHB_CHEN(dma));
	} else {
		void __iomem *dma_lld;
		if (msg->ttype == HSI_MSG_WRITE)
			dma_lld = ARASAN_CHN_V2(TX_CHAN_CURR_PTR, msg->channel);
		else
			dma_lld = ARASAN_CHN_V2(RX_CHAN_CURR_PTR, msg->channel);

		/* Simply write down the LLD DMA pointer */
		iowrite32(dma_xfer->v2.llp_addr | ARASAN_LLD_EN, dma_lld);
	}

	intel_hsi->dma_running |= mask;
	intel_hsi->dma_resumed |= mask;

do_start_dma_done:
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
}

/**
 * hsi_start_dma - start DMA data transfer
 * @msg: reference to the message to transfer
 * @dma_ch: DMA channel to consider
 * @dma_ctx: the DMA context to use
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_start_dma(struct hsi_msg *msg,
				int dma_ch,
				struct intel_dma_ctx *dma_ctx,
				struct intel_controller *intel_hsi)
{
	do_hsi_start_dma(msg, dma_ch, dma_ctx, intel_hsi, 0);
}

/**
 * hsi_restart_dma - restarting DMA data transfer further to a resume
 * @msg: reference to the message to transfer
 * @dma_ch: DMA channel to consider
 * @dma_ctx: the DMA context to use
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_restart_dma(struct hsi_msg *msg,
				int dma_ch,
				struct intel_dma_ctx *dma_ctx,
				struct intel_controller *intel_hsi)
{
	do_hsi_start_dma(msg, dma_ch, dma_ctx, intel_hsi, 1);
}

/**
 * hsi_transfer - starting transfer from TX or RX queue
 * @intel_hsi: Intel HSI controller reference
 * @is_tx: direction to consider (RX = 0, TX = anything else)
 * @hsi_ch: HSI channel to consider
 * @dma_ch: DMA channel to consider (<0 if no DMA)
 */
static void hsi_transfer(struct intel_controller *intel_hsi, int is_tx,
			 unsigned int hsi_ch, int dma_ch)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	struct list_head	*queue;
	struct hsi_msg		*msg;
	unsigned long		flags;
	struct intel_dma_ctx	*dma_ctx = NULL;
	struct intel_dma_xfer	*done_dma_xfer;
	struct intel_pio_ctx	*pio_ctx;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	queue = (is_tx) ?
			&intel_hsi->tx_queue[hsi_ch] :
			&intel_hsi->rx_queue[hsi_ch];

	if (list_empty(queue)) {
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		return;
	}

	msg = list_first_entry(queue, struct hsi_msg, link);
	if (msg->status != HSI_STATUS_QUEUED) {
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		return;
	}

	msg->status = HSI_STATUS_PROCEEDING;

	if (dma_ch >= 0) {
		if (is_arasan_v1(intel_hsi->version))
			dma_ctx = intel_hsi->dma_ctx[dma_ch];
		else
			dma_ctx = (is_tx) ?
				   &intel_hsi->tx_ctx[hsi_ch].dma :
				   &intel_hsi->rx_ctx[hsi_ch].dma;
		if (!dma_ctx) {
			spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
			return;
		}

		if (dma_ctx->ready->msg == NULL)
			do_hsi_prepare_dma(msg, dma_ctx, intel_hsi);

		/* The ongoing DMA is now the ready DMA message */
		done_dma_xfer		= dma_ctx->ongoing;
		dma_ctx->ongoing	= dma_ctx->ready;
		dma_ctx->ready		= done_dma_xfer;
	} else {
		msg->actual_len = 0;
		pio_ctx = (is_tx) ?
			   &intel_hsi->tx_ctx[hsi_ch].pio :
			   &intel_hsi->rx_ctx[hsi_ch].pio;
		pio_ctx->blk = (!msg->sgt.nents) ? NULL : msg->sgt.sgl;
		pio_ctx->offset = 0;
	}
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	/* Assert ACWAKE (deasserted on complete or destruct) */
	if (is_tx)
		assert_acwake(intel_hsi);
	else
		unforce_disable_acready(intel_hsi);

	if (dma_ch < 0)
		hsi_start_pio(msg, intel_hsi);
	else {
		hsi_start_dma(msg, dma_ch, dma_ctx, intel_hsi);
		hsi_try_prepare_dma(queue, dma_ctx, intel_hsi);
	}
}

/**
 * hsi_resume_dma_transfers - resuming DMA transfers hold on suspend
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_resume_dma_transfers(struct intel_controller *intel_hsi)
{
	struct hsi_msg *msg;
	struct intel_dma_ctx *dma_ctx;
	int i, dma_ch;

	if (!intel_hsi)
		return;

	if (is_arasan_v1(intel_hsi->version)) {
		for (dma_ch = 0; dma_ch < DWAHB_CHAN_CNT; dma_ch++) {
			dma_ctx = intel_hsi->dma_ctx[dma_ch];
			msg = (dma_ctx && dma_ctx->ongoing) ?
				dma_ctx->ongoing->msg : NULL;
			if (msg)
				hsi_restart_dma(msg, dma_ch, dma_ctx,
						intel_hsi);
		}
	} else {
		for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
			dma_ch = intel_hsi->tx_dma_ch[i];
			dma_ctx = (dma_ch >= 0) ?
					&intel_hsi->tx_ctx[i].dma : NULL;
			msg = (dma_ctx && dma_ctx->ongoing) ?
				dma_ctx->ongoing->msg : NULL;
			if (msg)
				hsi_restart_dma(msg,
						dma_ch, dma_ctx, intel_hsi);

			dma_ch = intel_hsi->rx_dma_ch[i];
			dma_ctx = (dma_ch >= 0) ?
					&intel_hsi->rx_ctx[i].dma : NULL;
			msg = (dma_ctx && dma_ctx->ongoing) ?
				dma_ctx->ongoing->msg : NULL;
			if (msg)
				hsi_restart_dma(msg,
						dma_ch, dma_ctx, intel_hsi);
		}
	}
}

/**
 * hsi_break_complete - break interrupt callback
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_break_complete(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	struct hsi_msg	*msg, *tmp_msg;
	unsigned long	flags;

	pr_info(DRVNAME " : HWBREAK received\n");

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	list_for_each_entry_safe(msg, tmp_msg, &intel_hsi->brk_queue, link) {
		/* Remove the msg from the queue */
		list_del(&msg->link);
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

		/* Call the msg complete callback */
		msg->status = HSI_STATUS_COMPLETED;
		hsi_msg_complete(intel_hsi, msg);
		spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	}
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
}

/**
 * hsi_rx_error - handle RX error interrupt sources
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_rx_error(struct intel_controller *intel_hsi)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	struct list_head		*queue;
	struct hsi_msg			*msg = NULL;
	unsigned int			 i;
	unsigned long			 flags;

	pr_err(DRVNAME ": RX error\n");

	for (i = 0; i < hsi_rx_channel_count(intel_hsi); i++) {
		queue = &intel_hsi->rx_queue[i];
		spin_lock_irqsave(&intel_hsi->sw_lock, flags);
		if (!list_empty(queue)) {
			msg = list_first_entry(queue, struct hsi_msg, link);
			msg->status = HSI_STATUS_ERROR;
		}
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
	}
}

/**
 * hsi_timeout - handle all timeout interrupt sources
 * @intel_hsi: Intel HSI controller reference
 * @timeout_reg: interrupt timeout register value
 *
 * Returns the timeout channels that have been cleared
 */
static u32 hsi_timeout(struct intel_controller *intel_hsi, u32 timeout_reg)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem			*ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	struct list_head		*queue;
	struct hsi_msg			*msg = NULL;
	struct intel_pio_ctx		*pio_ctx;
	unsigned int			 i;
	u32				*buf;
	u32				 timeout_clr, timeout_mask;
	unsigned long			 flags;
	int msg_status;
	unsigned int msg_actual_len;


	timeout_clr = 0;

	/* handle data timeout errors */
	/* ch0 timeout is bit 2 of int status reg */
	timeout_mask = ARASAN_IRQ_DATA_TIMEOUT(0);
	for (i = 0; i < hsi_rx_channel_count(intel_hsi);
	     i++, timeout_mask <<= 1) {
		if (!(timeout_reg & timeout_mask))
			continue;

		queue = &intel_hsi->rx_queue[i];
		pio_ctx = &intel_hsi->rx_ctx[i].pio;

hsi_pio_timeout_try:
		spin_lock_irqsave(&intel_hsi->sw_lock, flags);

		/* if no msg waiting for read, leave and wait for it */
		if (list_empty(queue))
			goto hsi_pio_timeout_next;

		msg = list_first_entry(queue, struct hsi_msg, link);
		if (unlikely(msg->status != HSI_STATUS_PROCEEDING))
			goto hsi_pio_timeout_next;

		if (unlikely(!msg->sgt.nents)) {
			msg->actual_len = 0;
			msg->status = HSI_STATUS_COMPLETED;
			goto hsi_pio_timeout_done;
		}

		msg_status = msg->status;
		msg_actual_len = msg->actual_len;
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

		timeout_clr |= timeout_mask;

		while ((hsi_ioread32(intel_hsi, ARASAN_REG(HSI_STATUS)) &
			ARASAN_RX_NOT_EMPTY(i)) &&
			   (msg_status == HSI_STATUS_PROCEEDING)) {
			spin_lock_irqsave(&intel_hsi->hw_lock, flags);
			if (likely(pio_ctx->blk->length > 0)) {
				buf = sg_virt(pio_ctx->blk) + pio_ctx->offset*4;
				*buf = hsi_ioread32(intel_hsi,
						ARASAN_CHN_REG(RX_DATA, i));
				msg_actual_len += HSI_FRAMES_TO_BYTES(1);
				pio_ctx->offset += 1;
			}

			if (pio_ctx->offset >=
				HSI_BYTES_TO_FRAMES(
						pio_ctx->blk->length)) {
				if (!sg_is_last(pio_ctx->blk)) {
					pio_ctx->blk =
						sg_next(pio_ctx->blk);
					pio_ctx->offset = 0;
				} else
					msg_status =
						HSI_STATUS_COMPLETED;
			}
			spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
		}

		spin_lock_irqsave(&intel_hsi->sw_lock, flags);
		msg->status = msg_status;
		msg->actual_len = msg_actual_len;

		if (msg->status == HSI_STATUS_COMPLETED) {
hsi_pio_timeout_done:
			list_del(&msg->link);
			spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
			hsi_msg_complete(intel_hsi, msg);
			hsi_transfer(intel_hsi, 0, i, -1);
			goto hsi_pio_timeout_try;
		}

hsi_pio_timeout_next:
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
	}

	return timeout_clr;
}

/**
 * hsi_async_break - send break message or queue break receive msg
 * @msg: reference to the HSI break message
 *
 * Return 0 if successful, -EINVAL if not in frame mode.
 */
static int hsi_async_break(struct hsi_msg *msg)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	struct hsi_port *port = hsi_get_port(msg->cl);
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	unsigned long flags;

	/* Return an error if not in frame mode */
	if (unlikely(!is_in_tx_frame_mode(intel_hsi)))
		return -EINVAL;

	hsi_pm_runtime_get_sync(intel_hsi);
	if (msg->ttype == HSI_MSG_WRITE) {
		assert_acwake(intel_hsi);
		spin_lock_irqsave(&intel_hsi->hw_lock, flags);
		intel_hsi->clk_cfg |= ARASAN_TX_BREAK;
		hsi_iowrite32(intel_hsi, intel_hsi->clk_cfg,
				ARASAN_REG(CLOCK_CTRL));
		/* Dummy read to ensure that at least the minimal delay for a
		 * break sequence will be met */
		(void) hsi_ioread32(intel_hsi, ARASAN_REG(CLOCK_CTRL));
		udelay(intel_hsi->brk_us_delay);
		intel_hsi->clk_cfg &= ~ARASAN_TX_BREAK;
		hsi_iowrite32(intel_hsi, intel_hsi->clk_cfg,
				ARASAN_REG(CLOCK_CTRL));
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
		msg->status = HSI_STATUS_COMPLETED;
		hsi_msg_complete(intel_hsi, msg);
		(void) deassert_acwake(intel_hsi);
	} else {
		spin_lock_irqsave(&intel_hsi->sw_lock, flags);
		list_add_tail(&msg->link, &intel_hsi->brk_queue);
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
	}
	pm_runtime_put(intel_hsi->pdev);

	return 0;
}

/**
 * hsi_mid_async - queue a HSI message and start transfer if possible
 * @msg: reference to the HSI message
 *
 * Queue message to send when possible.
 *
 * Returns 0 if successful, -EINVAL if message pointer is NULL or channel
 * number is invalid, or transfer error if any.
 */
static int hsi_mid_async(struct hsi_msg *msg)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	struct hsi_port *port = hsi_get_port(msg->cl);
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);
	struct list_head *queue;
	struct intel_dma_ctx *dma_ctx;
	u16 *queue_busy;
	unsigned int hsi_ch, nents;
	int dma_ch;
	u8 dir;
	int is_tx, err;
	unsigned long flags;

	if (msg->break_frame) {
		err =  hsi_async_break(msg);
		goto out;
	}

	hsi_ch = msg->channel;
	dma_ch = hsi_get_dma_channel(intel_hsi, msg, hsi_ch);

	if (msg->ttype == HSI_MSG_WRITE) {
		if (hsi_ch >= hsi_tx_channel_count(intel_hsi)) {
			pr_err(DRVNAME ": Invalid TX channel (%d)\n", hsi_ch);
			err = -EINVAL;
			goto out;
		}

		dma_ctx = (dma_ch >= 0) ?
			&intel_hsi->tx_ctx[hsi_ch].dma : NULL;
		is_tx = 1;
		queue = &intel_hsi->tx_queue[hsi_ch];
		queue_busy = &intel_hsi->tx_queue_busy;
		dir = DMA_TO_DEVICE;
	} else {
		if (hsi_ch >= hsi_rx_channel_count(intel_hsi)) {
			pr_err(DRVNAME ": Invalid RX channel (%d)\n", hsi_ch);
			err = -EINVAL;
			goto out;
		}

		dma_ctx = (dma_ch >= 0) ?
			&intel_hsi->rx_ctx[hsi_ch].dma : NULL;
		is_tx = 0;
		queue = &intel_hsi->rx_queue[hsi_ch];
		queue_busy = &intel_hsi->rx_queue_busy;
		dir = DMA_FROM_DEVICE;
	}

	nents = msg->sgt.nents;
	if (dma_ctx) {
		if (nents > dma_ctx->sg_entries) {
			pr_err(DRVNAME ": Unsupported number of SG entries (%d)\n",
					nents);
			err = -ENOSYS;
			goto out;
		}

		err = dma_map_sg(intel_hsi->pdev, msg->sgt.sgl, nents, dir);
		if (err < 0) {
			pr_err(DRVNAME": dma_map_sg error: 0x%02X", err);
			goto out;
		}
	}

	if (unlikely((*queue_busy) & QUEUE_BUSY(hsi_ch))) {
		if (dma_ch >= 0)
			dma_unmap_sg(intel_hsi->pdev, msg->sgt.sgl, nents, dir);
		err = -EBUSY;
	} else {
		msg->status = HSI_STATUS_QUEUED;
		spin_lock_irqsave(&intel_hsi->sw_lock, flags);
		list_add_tail(&msg->link, queue);
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		hsi_transfer(intel_hsi, is_tx, hsi_ch, dma_ch);
		err = 0;
	}
out:
	return err;
}

/**
 * hsi_destruct_msg - helper function for cleanly destructing a message
 * @msg: reference to the message to destruct
 * @dma_ch: DMA channel to consider (<0 if in PIO mode)
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_destruct_msg(struct hsi_msg *msg, int dma_ch,
			     struct intel_controller *intel_hsi)
{
	u8 dir;

	if (msg->ttype == HSI_MSG_WRITE)
		(void) deassert_acwake(intel_hsi);

	if (dma_ch >= 0) {
		dir = (msg->ttype == HSI_MSG_READ) ?
					DMA_FROM_DEVICE : DMA_TO_DEVICE;
		dma_unmap_sg(intel_hsi->pdev, msg->sgt.sgl,
			     msg->sgt.nents, dir);
	}

	if (msg->destructor)
		msg->destructor(msg);
	else
		hsi_free_msg(msg);
}

/**
 * hsi_flush_queue - flushing all messages of a client from a queue
 * @queue: reference to the message queue to flush
 * @cl: HSI client reference
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_flush_queue(struct list_head *queue, struct hsi_client *cl,
			    struct intel_controller *intel_hsi)
{
	struct intel_dma_ctx *dma_ctx;
	struct hsi_msg *msg, *tmp_msg;
	int dma_ch;
	unsigned long flags;
	LIST_HEAD(msgs_to_delete);

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	list_for_each_entry_safe(msg, tmp_msg, queue, link) {
		if (cl != msg->cl)
			continue;

		dma_ch = hsi_get_dma_channel(intel_hsi, msg, msg->channel);
		if (dma_ch < 0)
			goto del_node;

		/* Do not remove the ongoing DMA message yet ! */
		if (is_arasan_v1(intel_hsi->version))
			dma_ctx = intel_hsi->dma_ctx[dma_ch];
		else
			dma_ctx = (msg->ttype == HSI_MSG_WRITE) ?
				&intel_hsi->tx_ctx[msg->channel].dma :
				&intel_hsi->rx_ctx[msg->channel].dma;

		/* Already cleaned by shutdown callback? */
		if (dma_ctx == NULL)
			continue;

		if (dma_ctx->ongoing->msg == msg) {
			msg->break_frame = 1;
			continue;
		}

		/* Clear any ready msg */
		if (dma_ctx->ready->msg == msg)
			dma_ctx->ready->msg = NULL;

del_node:
		/* Move the msg to the local list (will be deleted after) */
		list_move_tail(&msg->link, &msgs_to_delete);
	}
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	/* Clear & destroy msgs list */
	list_for_each_entry_safe(msg, tmp_msg, &msgs_to_delete, link) {
		list_del(&msg->link);
		dma_ch = hsi_get_dma_channel(intel_hsi, msg, msg->channel);
		hsi_destruct_msg(msg, dma_ch, intel_hsi);
	}
}

/**
 * hsi_cleanup_dma_ch - cleanup a DMA channel activities related to a client
 * @intel_hsi: Intel HSI controller reference
 * @cl: HSI client reference
 * @hsi_ch: the considered HSI channel
 * @is_tx: direction of the transfer (RX = 0, TX != 0)
 */
static void hsi_cleanup_dma_ch(struct intel_controller *intel_hsi,
					struct hsi_client *cl,
					unsigned int hsi_ch,
					int is_tx)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	void __iomem	*ctrl = intel_hsi->ctrl_io;
	void __iomem	*dma = intel_hsi->dma_io;
	struct intel_dma_ctx *dma_ctx;
	struct hsi_msg *msg;
	int dma_ch;
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	if (is_tx) {
		dma_ch = intel_hsi->tx_dma_ch[hsi_ch];
		dma_ctx = &intel_hsi->tx_ctx[hsi_ch].dma;
	} else {
		dma_ch = intel_hsi->rx_dma_ch[hsi_ch];
		dma_ctx = &intel_hsi->rx_ctx[hsi_ch].dma;
	}

	/* Some ongoing xfer ? */
	if (!dma_ctx->ongoing) {
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		return;
	}

	msg = (dma_ch >= 0) ? dma_ctx->ongoing->msg : NULL;
	if ((!msg) || (msg->cl != cl)) {
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		return;
	}

	dma_ctx->ongoing->msg = NULL;
	msg->break_frame = 0;
	msg->status = HSI_STATUS_ERROR;
	list_del(&msg->link);
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	intel_hsi->dma_running &= ~DMA_BUSY(dma_ch);

	if (is_arasan_v1(intel_hsi->version)) {
		hsi_iowrite32(intel_hsi, 0,
				ARASAN_CHN_V1(SDMA_CFG, dma_ch));
		hsi_iowrite32(intel_hsi, DWAHB_CHAN_STOP(dma_ch),
				HSI_DWAHB_CHEN(dma));
	}
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	/* Destroy any ongoing msg (skipped by the flush_queue function) */
	hsi_destruct_msg(msg, dma_ch, intel_hsi);

	/* Restart transfers of other clients */
	hsi_transfer(intel_hsi, is_tx, hsi_ch, dma_ch);
}

/**
 * hsi_cleanup_dma - cleanup DMA activities related to a client
 * @intel_hsi: Intel HSI controller reference
 * @cl: HSI client reference
 */
static void hsi_cleanup_dma(struct intel_controller *intel_hsi,
						struct hsi_client *cl)
{
	unsigned int i;

	for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
		hsi_cleanup_dma_ch(intel_hsi, cl, i, 1);
		hsi_cleanup_dma_ch(intel_hsi, cl, i, 0);
	}
}

/**
 * hsi_mid_setup - setting up the controller from client configuration
 * @cl: HSI client reference
 *
 * This stores the hardware setup and applies it in conformance with the
 * client settings.
 *
 * Return success or an error code if the cleint configuration is invalid.
 */
static int hsi_mid_setup(struct hsi_client *cl)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	struct hsi_port *port = hsi_get_port(cl);
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);
	struct hsi_platform_data *hsi_info;
	struct hsi_mid_platform_data *pd;
	int version = intel_hsi->version;
	int is_v1 = is_arasan_v1(version);
	u16 tx_fifo_sz[HSI_MID_MAX_CHANNELS];
	u16 rx_fifo_sz[HSI_MID_MAX_CHANNELS];
	u16 tx_fifo_th[HSI_MID_MAX_CHANNELS];
	u16 rx_fifo_th[HSI_MID_MAX_CHANNELS];
	int full_tx_sz, tx_sz, tx_en, tx_th;
	int full_rx_sz, rx_sz, rx_en, rx_th;
	unsigned int divisor, rx_timeout, data_timeout;
	unsigned long flags;
	u32 arb_cfg, sz_cfg, err_cfg, clk_cfg, prg_cfg;
	int i, dma_ch, err = 0;
	u32 irq_cfg = 0;

	/* Read the platform data to initialise the device */

	hsi_info = (struct hsi_platform_data *) cl->device.platform_data;
	if (hsi_info == NULL) {
		pr_err(DRVNAME": Platform data not found\n");
		return -EINVAL;
	} else
		pr_info("%s: hsi platform data recovered (modem: %d)", __func__,
				hsi_info->hsi_client_info.mdm_ver);

	pd = (struct hsi_mid_platform_data *) &(hsi_info->hsi_mid_info);

	/* Save the ACWAKE delay */
	intel_hsi->acwake_delay = HSI_ACWAKE_DELAY;

	/* Save the current TX speed */
	intel_hsi->tx_speed = cl->tx_cfg.speed;

	/* Compute the arbiter control register */
	arb_cfg	= cl->tx_cfg.arb_mode;

	/* Compute the RX timeout value (for inserting RX error) */
	rx_timeout = rounddown_pow_of_two(200000/intel_hsi->ip_freq);

	/* Compute the program1 register */
	sz_cfg	= ARASAN_TX_CHANNEL_SIZE(cl->tx_cfg.channels) |
		  ARASAN_RX_CHANNEL_SIZE(cl->rx_cfg.channels);

	/* Compute the program, FIFO , DMA and interrupt registers */
	prg_cfg		= ARASAN_RX_TIMEOUT_CNT(rx_timeout) |
			  ARASAN_TX_MODE(cl->tx_cfg.mode) |
			  ARASAN_RX_FLOW(cl->rx_cfg.flow) |
			  ARASAN_RX_MODE(cl->rx_cfg.mode);
	if (!use_oob_cawake(intel_hsi))
		irq_cfg		= ARASAN_IRQ_RX_WAKE;
	err_cfg		= ARASAN_IRQ_BREAK | ARASAN_IRQ_RX_ERROR;

	full_tx_sz   = 0;
	full_rx_sz   = 0;
	data_timeout = 0;
	for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
		tx_sz = pd->tx_fifo_sizes[i];
		tx_en = (tx_sz > 0);
		tx_sz = (tx_en) ?
			(is_v1 ? roundup_pow_of_two(tx_sz) : tx_sz) : 0;
		tx_th = max(min(pd->tx_fifo_thres[i], tx_sz), 1);
		tx_th = is_v1 ?
			((tx_th > tx_sz/2) ? 3*tx_sz/4 : tx_sz/2) : tx_th;
		full_tx_sz += tx_sz;
		tx_fifo_sz[i] = tx_sz;
		tx_fifo_th[i] = tx_th;

		rx_sz = pd->rx_fifo_sizes[i];
		rx_en = (rx_sz > 0);
		rx_sz = (rx_en) ?
			(is_v1 ? roundup_pow_of_two(rx_sz) : rx_sz) : 0;
		rx_th = max(min(pd->rx_fifo_thres[i], rx_sz), 0);
		rx_th = is_v1 ?
			((rx_th > rx_sz/2) ? 3*rx_sz/4 : rx_sz/2) : rx_th;
		full_rx_sz += rx_sz;
		rx_fifo_sz[i] = rx_sz;
		rx_fifo_th[i] = rx_th;

		prg_cfg |= ARASAN_TX_CHANNEL_ENABLE(tx_en, i);
		prg_cfg |= ARASAN_RX_CHANNEL_ENABLE(rx_en, i);

		dma_ch = pd->tx_dma_channels[i];
		if ((dma_ch >= 0) && (is_v1))
			irq_cfg |= ARASAN_IRQ_DMA_COMPLETE(dma_ch);

		dma_ch = pd->rx_dma_channels[i];
		if ((dma_ch >= 0) && (is_v1))
			irq_cfg |= ARASAN_IRQ_DMA_COMPLETE(dma_ch);
		else
			data_timeout += rx_th;
		arb_cfg |= (pd->tx_priorities[i] & 0x7) << (3*i+1);
	}

	if ((full_tx_sz > ARASAN_MAX_TX_FIFO_SZ(version)) ||
		(full_rx_sz > ARASAN_MAX_RX_FIFO_SZ(version))) {
		pr_err(DRVNAME ": Wrong FIFO size (rx: %d (max: %d), tx: %d (max: %d)\n",
			full_rx_sz, ARASAN_MAX_TX_FIFO_SZ(version),
			full_tx_sz, ARASAN_MAX_RX_FIFO_SZ(version));
		return -EINVAL;
	}

	/* Give every RX HSI frame at least 128 TX clock cycles to arrive */
	data_timeout = order_base_2((data_timeout*128)/8192);

	/* Compute the clock control register */
	divisor = max(intel_hsi->ip_freq/max(cl->tx_cfg.speed, 1u), 1u);
	divisor = min(divisor, 256u);
	divisor = rounddown_pow_of_two(divisor);
	clk_cfg = ARASAN_CLK_ENABLE | ARASAN_CLK_START |
			ARASAN_CLK_DIVISOR(divisor/2) |
			ARASAN_DATA_TIMEOUT(data_timeout) |
			ARASAN_RX_FRAME_BURST_COUNT(256) |
			ARASAN_RX_TAILING_BIT_COUNT(50);

	if (is_v1)
		clk_cfg |= ARASAN_RX_TAP_DELAY_NS(3);

	/* A HSI break frame shall be at least 38 TX cycles long */
	intel_hsi->brk_us_delay = ((38000*divisor)/intel_hsi->ip_freq)+1;

	/* Setup the HSI controller accordingly */
	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	intel_hsi->irq_cfg = irq_cfg;
	intel_hsi->err_cfg = err_cfg;
	intel_hsi->clk_cfg = clk_cfg;

	/* Keep the current RX and TX wake status */
	intel_hsi->prg_cfg &= ARASAN_TX_ENABLE|ARASAN_RX_ENABLE;
	intel_hsi->prg_cfg |= prg_cfg;
	intel_hsi->arb_cfg  = arb_cfg;
	intel_hsi->sz_cfg   = sz_cfg;
	intel_hsi->hsi_wake_raised = 0;

	for (i = 0; i < HSI_MID_MAX_CHANNELS; i++) {
		intel_hsi->tx_fifo_sz[i]  = tx_fifo_sz[i];
		intel_hsi->tx_fifo_th[i]  = tx_fifo_th[i];
		intel_hsi->rx_fifo_sz[i]  = rx_fifo_sz[i];
		intel_hsi->rx_fifo_th[i]  = rx_fifo_th[i];
		intel_hsi->tx_dma_ch[i]   = pd->tx_dma_channels[i];
		intel_hsi->rx_dma_ch[i]   = pd->rx_dma_channels[i];
		if (pd->tx_dma_channels[i] >= 0)
			intel_hsi->tx_ctx[i].dma.sg_entries =
						max(pd->tx_sg_entries[i], 1);
		if (pd->rx_dma_channels[i] >= 0)
			intel_hsi->rx_ctx[i].dma.sg_entries =
						max(pd->rx_sg_entries[i], 1);
	}

	/* Prepare the necessary DMA contexts */
	err = alloc_xfer_ctx(intel_hsi);
	/* The controller will be configured on resume if necessary */
	if (unlikely(!err && intel_hsi->suspend_state == DEVICE_READY)) {
		err = hsi_ctrl_set_cfg(intel_hsi, 1);
		intel_hsi->pm_do_reset = 0;
	} else
		intel_hsi->pm_do_reset = 1;

	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	return err;
}

/**
 * hsi_mid_flush - flushing resources belonging to a client
 * @cl: HSI client reference
 *
 * Returns success.
 */
static int hsi_mid_flush(struct hsi_client *cl)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	struct hsi_port *port = hsi_get_port(cl);
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	unsigned int i, unforce = 0;
	unsigned long flags;

	/* disable the ISR tasklet when finished to ensure there is no
	 * cross execution with RX queue flush
	 * disable the FWD tasklet to avoid cross execution with FWD queue flush
	 */
	tasklet_disable(&intel_hsi->isr_tasklet);
	tasklet_disable(&intel_hsi->fwd_tasklet);

	/* Prevent any new message in the software queues */
	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	intel_hsi->tx_queue_busy = ARASAN_ALL_CHANNELS;
	intel_hsi->rx_queue_busy = ARASAN_ALL_CHANNELS;
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	/* Wake the device not to react on the CAWAKE and to access hw */
	hsi_pm_runtime_get_sync(intel_hsi);

	/* Disable the ACREADY line not to be disturbed during flush */
	force_disable_acready(intel_hsi);

	/* Flush queues (Break, TX, RX, FW) */
	hsi_flush_queue(&intel_hsi->brk_queue, cl, intel_hsi);
	for (i = 0; i < hsi_tx_channel_count(intel_hsi); i++)
		hsi_flush_queue(&intel_hsi->tx_queue[i], cl, intel_hsi);
	for (i = 0; i < hsi_rx_channel_count(intel_hsi); i++)
		hsi_flush_queue(&intel_hsi->rx_queue[i], cl, intel_hsi);
	if (intel_hsi->suspend_state == DEVICE_READY)
		hsi_cleanup_dma(intel_hsi, cl);
	hsi_flush_queue(&intel_hsi->fwd_queue, cl, intel_hsi);

	/* Flush all RX HW FIFO which do not have any SW message queued
	* Don't flush if the device is not waken up yet: everything will
	* be cleared at next TTY port open (HSI reset)
	*/
	if (intel_hsi->suspend_state == DEVICE_READY) {
		for (i = 0; i < hsi_rx_channel_count(intel_hsi); i++) {
			if (list_empty(&intel_hsi->rx_queue[i])) {
				while (hsi_ioread32(intel_hsi, ARASAN_REG(HSI_STATUS)) &
						ARASAN_RX_NOT_EMPTY(i))
					hsi_ioread32(intel_hsi,
						ARASAN_CHN_REG(RX_DATA, i));
			} else
				unforce = 1;
		}
	}

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	intel_hsi->tx_queue_busy = 0;
	intel_hsi->rx_queue_busy = 0;
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	/* Unforce the ACREADY disable if any RX queue is not empty */
	if (unforce)
		unforce_disable_acready(intel_hsi);

	/* Get back to the original HSI controller power status */
	pm_runtime_put(intel_hsi->pdev);

	tasklet_enable(&intel_hsi->isr_tasklet);
	tasklet_enable(&intel_hsi->fwd_tasklet);

	return 0;
}

/**
 * hsi_mid_release - releasing resources belonging to a client
 * @cl: HSI client reference
 *
 * This is also resetting the hardware upon release of the last client.
 *
 * Returns success.
 */
static int hsi_mid_release(struct hsi_client *cl)
{
	struct hsi_port *port = hsi_get_port(cl);
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);

	/* Now cleanup all the queues related to the client */
	hsi_mid_flush(cl);

	/* Reset the controller if this client is the last in the list */
	if (port->claimed <= 1)
		hsi_ctrl_clean_reset(intel_hsi);

	return 0;
}

/**
 * hsi_mid_start_tx - asserting the ACWAKE line
 * @cl: HSI client reference
 *
 * Returns success.
 */
static int hsi_mid_start_tx(struct hsi_client *cl)
{
	struct hsi_port *port = hsi_get_port(cl);
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);

	assert_acwake(intel_hsi);

	return 0;
}

/**
 * hsi_mid_stop_tx - de-asserting the ACWAKE line
 * @cl: HSI client reference
 *
 * The implementation will only de-assert the line if the TX path is empty and
 * if there is no left start_tx outstanding.
 *
 * Returns success.
 */
static int hsi_mid_stop_tx(struct hsi_client *cl)
{
	struct hsi_port *port = hsi_get_port(cl);
	struct intel_controller *intel_hsi = hsi_port_drvdata(port);

	(void) deassert_acwake(intel_hsi);

	return 0;
}

/**
 * enable_acready - CA_WAKE assertion event handler
 * @hsi: Intel HSI controller reference
 */
static void enable_acready(struct intel_controller *intel_hsi)
{
	struct hsi_controller *hsi = to_hsi_controller(intel_hsi->dev);
	unsigned int i;

	if (has_enabled_acready(intel_hsi))
		for (i = 0; i < hsi->num_ports; i++)
			hsi_event(hsi->port[i], HSI_EVENT_START_RX);
}

/**
 * try_disable_acready - CA_WAKE de-assertion event handler
 * @hsi: Intel HSI controller reference
 */
static void try_disable_acready(struct intel_controller *intel_hsi)
{
	struct hsi_controller *hsi = to_hsi_controller(intel_hsi->dev);
	unsigned int i;

	if (has_disabled_acready(intel_hsi))
		for (i = 0; i < hsi->num_ports; i++)
			hsi_event(hsi->port[i], HSI_EVENT_STOP_RX);
}

/**
 * hsi_pio_xfer_complete - PIO threshold reached interrupt management
 * @hsi: Intel HSI controller reference
 * @ch: HSI channel
 * @is_tx: direction of the transfer (RX = 0, TX != 0)
 *
 * Returns 0 on completion or some interrupt enable bitfield for re-enabling
 * the PIO interrupt if there is still room in the current HSI message.
 */
static inline u32 hsi_pio_xfer_complete(struct intel_controller *intel_hsi,
					unsigned int ch, int is_tx)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	struct list_head *queue;
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	void __iomem *fifo;
	struct hsi_msg *msg;
	struct intel_pio_ctx *pio_ctx;
	u32 *buf;
	u32 avail, blk_len, sz;
	int msg_status;
	unsigned int msg_actual_len;
	unsigned long flags;

	queue = (is_tx) ?
		&intel_hsi->tx_queue[ch] : &intel_hsi->rx_queue[ch];

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	if (list_empty(queue)) {
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		return 0;
	}
	msg = list_first_entry(queue, struct hsi_msg, link);
	if (unlikely(!msg->sgt.nents)) {
		msg_actual_len = 0;
		goto hsi_pio_xfer_done;
	}

	msg_status = msg->status;
	msg_actual_len = msg->actual_len;
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	pio_ctx = (is_tx) ?
		&intel_hsi->tx_ctx[ch].pio : &intel_hsi->rx_ctx[ch].pio;

	if (msg_status == HSI_STATUS_PROCEEDING) {
		if (is_tx) {
			avail = intel_hsi->tx_fifo_th[ch];
			fifo = ARASAN_CHN_REG(TX_DATA, ch);
		} else {
			avail = intel_hsi->rx_fifo_th[ch];
			fifo = ARASAN_CHN_REG(RX_DATA, ch);
		}

		spin_lock_irqsave(&intel_hsi->hw_lock, flags);
		while ((avail > 0)
				|| (unlikely(!pio_ctx->blk->length))) {
			buf = sg_virt(pio_ctx->blk) +
					(pio_ctx->offset*4);
			blk_len = HSI_BYTES_TO_FRAMES(
					pio_ctx->blk->length);
			sz = min(avail, blk_len - pio_ctx->offset);
			msg_actual_len += HSI_FRAMES_TO_BYTES(sz);
			avail -= sz;
			pio_ctx->offset += sz;
			for (; sz > 0; sz--) {
				if (is_tx)
					hsi_iowrite32(intel_hsi, *buf, fifo);
				else
					*buf = hsi_ioread32(intel_hsi, fifo);
				buf++;
			}

			if (pio_ctx->offset >= blk_len) {
				pio_ctx->offset = 0;
				if (sg_is_last(pio_ctx->blk)) {
					spin_unlock_irqrestore(
						&intel_hsi->hw_lock,
						flags);
					spin_lock_irqsave(
						&intel_hsi->sw_lock,
						flags);
					goto hsi_pio_xfer_done;
				}
				pio_ctx->blk = sg_next(pio_ctx->blk);
			}
		}
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
	}

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	if ((pio_ctx->offset < HSI_BYTES_TO_FRAMES(pio_ctx->blk->length)) ||
	    ((is_tx) &&
	     (!(hsi_ioread32(intel_hsi, ARASAN_REG(HSI_STATUS))
			& ARASAN_TX_EMPTY(ch))))) {
		spin_lock_irqsave(&intel_hsi->sw_lock, flags);
		msg->actual_len = msg_actual_len;
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
		return (is_tx) ?
			ARASAN_IRQ_TX_THRESHOLD(ch) :
			ARASAN_IRQ_RX_THRESHOLD(ch);
	}
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
hsi_pio_xfer_done:
	msg->actual_len = msg_actual_len;
	msg->status = HSI_STATUS_COMPLETED;
	list_del(&msg->link);
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	hsi_transfer(intel_hsi, is_tx, ch, -1);
	if (is_tx)
		(void) deassert_acwake(intel_hsi);

	hsi_msg_complete(intel_hsi, msg);
	return 0;
}

/**
 * hsi_pio_rx_complete - Rx threshold reached interrupt management
 * @hsi: Intel HSI controller reference
 * @lch: DMA channel.
 *
 * Returns 0 on completion or some interrupt enable bitfield for re-enabling
 * the PIO interrupt if there is still room in the current HSI message.
 */
static u32 hsi_pio_rx_complete(struct intel_controller *intel_hsi,
			       unsigned int ch)
{
	return hsi_pio_xfer_complete(intel_hsi, ch, 0);
}

/**
 * hsi_pio_tx_complete - Tx threshold reached interrupt management
 * @hsi: Intel HSI controller reference
 * @lch: DMA channel.
 *
 * Returns 0 on completion or some interrupt enable bitfield for re-enabling
 * the PIO if there is still data to transfer from the current HSI message.
 */
static u32 hsi_pio_tx_complete(struct intel_controller *intel_hsi,
			       unsigned int ch)
{
	return hsi_pio_xfer_complete(intel_hsi, ch, 1);
}

/**
 * hsi_dma_forward - forwarding some message to the DMA queue
 * @hsi: Intel HSI controller reference
 * @is_tx: the direction of the DMA channel (TX=1, RX=0)
 * @hsi_ch: HSI channel
 * @dma_ch: DMA channel.
 * @dma_ctx: DMA context reference
 */
static void hsi_dma_forward(struct intel_controller *intel_hsi,
				int is_tx,
				unsigned int hsi_ch,
				unsigned int dma_ch,
				struct intel_dma_ctx *dma_ctx)
{
	unsigned long flags;
	struct hsi_msg *msg;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	msg = dma_ctx->ongoing->msg;
	/* Check if the msg was not clearead */
	if (msg) {
		dma_ctx->ongoing->msg = NULL;
		list_del(&msg->link);
		list_add_tail(&msg->link, &intel_hsi->fwd_queue);

		/* Set to QUEUED the remaining items */
		if (is_tx) {
			struct hsi_msg *msg;
			struct list_head *queue = &intel_hsi->tx_queue[hsi_ch];
			list_for_each_entry(msg, queue, link)
				msg->status = HSI_STATUS_QUEUED;
		}
	}
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	/* Kick another transfer if necessary */
	hsi_transfer(intel_hsi, is_tx, hsi_ch, dma_ch);
}

/**
 * hsi_dma_complete_v1 - DMA complete status handler (for IP version 1)
 * @hsi: Intel HSI controller reference
 * @dma_ch: DMA channel.
 *
 * Returns the number of managed DMA transfers.
 */
static int hsi_dma_complete_v1(struct intel_controller *intel_hsi,
			    unsigned int dma_ch)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	struct intel_dma_ctx *dma_ctx;
	struct intel_dma_xfer *ongoing_xfer;
	void __iomem *ctrl = intel_hsi->ctrl_io;
	struct hsi_msg *msg;
	unsigned long flags;
	struct intel_dma_lli_xfer *lli_xfer;
	u32 blk_len_overwrite = 0;
	unsigned int blk_len = 0;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	dma_ctx = intel_hsi->dma_ctx[dma_ch];
	/* Check if the DMA IT was scheduled BUT not serverd just
	   before the calling hsi_ctrl_clean_reset (SMP config) */
	if (!dma_ctx) {
		pr_debug(DRVNAME ": Interrupt ignored (RESET already done) !");
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		return 0;
	}

	ongoing_xfer = dma_ctx->ongoing;
	msg = ongoing_xfer->msg;

	lli_xfer = &ongoing_xfer->v1.with_link_list;
	if ((is_using_link_list(dma_ctx)) && (msg) &&
	    (!sg_is_last(lli_xfer->blk))) {
		lli_xfer->blk = sg_next(lli_xfer->blk);
		blk_len = HSI_BYTES_TO_FRAMES(lli_xfer->blk->length);
		blk_len_overwrite = ARASAN_DMA_XFER_FRAMES(blk_len);
		msg = NULL;
	}

	if ((msg) && (msg->status != HSI_STATUS_ERROR))
		msg->status = HSI_STATUS_COMPLETED;
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	if (blk_len) {
		spin_lock_irqsave(&intel_hsi->hw_lock, flags);
		hsi_iowrite32(intel_hsi,
				ongoing_xfer->v1.slv_enable | blk_len_overwrite,
				ARASAN_CHN_V1(SDMA_CFG, dma_ch));
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
	}

	if (unlikely(!msg))
		return 0;

	/* It is safe to disable the DMA channel right now, as no DMA transfer
	 * can start on this channel as long as the current message is not
	 * popped from the list, which happens later on! */
	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	hsi_iowrite32(intel_hsi, 0, ARASAN_CHN_V1(SDMA_CFG, dma_ch));
	intel_hsi->dma_running &= ~DMA_BUSY(dma_ch);
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	hsi_dma_forward(intel_hsi,
				(msg->ttype == HSI_MSG_WRITE),
				msg->channel,
				dma_ch, dma_ctx);
	return 1;
}

/**
 * hsi_dma_complete_v2 - DMA complete status handler (for IP version 2)
 * @hsi: Intel HSI controller reference
 * @hsi_ch: the considered HSI channel
 * @is_tx: a flag stating if this is the TX or RX complete
 */
static int hsi_dma_complete_v2(struct intel_controller *intel_hsi,
				unsigned int hsi_ch, int is_tx)
{
	struct intel_dma_ctx *dma_ctx;
	struct intel_dma_xfer *ongoing_xfer;
	struct hsi_msg *msg;
	unsigned long flags;
	int dma_ch;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	if (is_tx) {
		dma_ch = intel_hsi->tx_dma_ch[hsi_ch];
		dma_ctx = &intel_hsi->tx_ctx[hsi_ch].dma;
	} else {
		dma_ch = intel_hsi->rx_dma_ch[hsi_ch];
		dma_ctx = &intel_hsi->rx_ctx[hsi_ch].dma;
	}
	ongoing_xfer = dma_ctx->ongoing;
	/* Check if the DMA IT was scheduled BUT not serverd just
	   before the calling hsi_ctrl_clean_reset (SMP config) */
	if (!ongoing_xfer) {
		pr_debug(DRVNAME": Interrupt ignored (RESET already done) !");

		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
		return 0;
	}

	msg = ongoing_xfer->msg;
	if ((msg) && (!msg->status != HSI_STATUS_ERROR))
		msg->status = HSI_STATUS_COMPLETED;
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	intel_hsi->dma_running &= ~DMA_BUSY(dma_ch);
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	if (msg)
		hsi_dma_forward(intel_hsi,
				(msg->ttype == HSI_MSG_WRITE),
				msg->channel,
				dma_ch, dma_ctx);
	return 1;
}

/**
 * hsi_isr_tasklet - low-latency interrupt management out of interrupt state
 * @hsi: Intel HSI controller reference
 */
static void hsi_isr_tasklet(unsigned long hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	struct intel_controller *intel_hsi = (struct intel_controller *) hsi;
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	unsigned int ch;
	u32 irq_status, err_status, dma_status;
	u32 irq_cfg	= 0;
	u32 err_cfg	= 0;
	u32 tx_mask	= ARASAN_IRQ_TX_THRESHOLD(0);
	u32 rx_mask	= ARASAN_IRQ_RX_THRESHOLD(0);
	unsigned long flags;
	int do_fwd = 0;
	int hsi_wake_raised = 0;

	intel_hsi->isr_tasklet_running = 1;
	/* Get a local copy of the current interrupt status */
	spin_lock_irqsave(&intel_hsi->hw_lock, flags);
	irq_status = intel_hsi->irq_status;
	err_status = intel_hsi->err_status;
	dma_status = intel_hsi->dma_status;
	hsi_wake_raised = intel_hsi->hsi_wake_raised;
	intel_hsi->irq_status = 0;
	intel_hsi->err_status = 0;
	intel_hsi->dma_status = 0;
	intel_hsi->hsi_wake_raised = 0;
	spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);

	if ((irq_status & ARASAN_IRQ_RX_WAKE) || hsi_wake_raised) {
		if (!intel_hsi->resumed)
			intel_hsi->wakeup_packet_log = 1;
		enable_acready(intel_hsi);
	}

	if (err_status & ARASAN_IRQ_RX_ERROR)
		hsi_rx_error(intel_hsi);

	if (is_arasan_v1(version)) {
		u32 dma_mask = ARASAN_IRQ_DMA_COMPLETE(0);

		for (ch = 0; ch < DWAHB_CHAN_CNT; ch++) {
			if (irq_status & dma_mask)
				do_fwd |= hsi_dma_complete_v1(intel_hsi, ch);
			dma_mask <<= 1;
		}
	} else {
		u32 tx_dma_mask = ARASAN_DMA_IRQ_TX_COMPLETE(0);
		u32 rx_dma_mask = ARASAN_DMA_IRQ_RX_COMPLETE(0);

		for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++) {
			if (dma_status & tx_dma_mask)
				do_fwd |= hsi_dma_complete_v2(intel_hsi, ch, 1);

			if (dma_status & rx_dma_mask)
				do_fwd |= hsi_dma_complete_v2(intel_hsi, ch, 0);

			tx_dma_mask <<= 1;
			rx_dma_mask <<= 1;
		}

		if (irq_status & ARASAN_IRQ_RX_SLEEP(ARASAN_IP_V2))
			mod_timer(&intel_hsi->rx_idle_poll,
					jiffies + IDLE_POLL_JIFFIES);
	}

	for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++) {
		if (irq_status & tx_mask)
			irq_cfg |= hsi_pio_tx_complete(intel_hsi, ch);
		if (irq_status & rx_mask)
			irq_cfg |= hsi_pio_rx_complete(intel_hsi, ch);
		tx_mask <<= 1;
		rx_mask <<= 1;
	}

	if (err_status & ARASAN_IRQ_ANY_DATA_TIMEOUT)
		err_cfg = hsi_timeout(intel_hsi, err_status);

	if (err_status & ARASAN_IRQ_BREAK)
		hsi_break_complete(intel_hsi);

	try_disable_acready(intel_hsi);

	if (do_fwd)
		tasklet_schedule(&intel_hsi->fwd_tasklet);

	/* Re-enable relevant interrupts */
	if (irq_cfg || err_cfg) {
		spin_lock_irqsave(&intel_hsi->hw_lock, flags);
		if (irq_cfg) {
			intel_hsi->irq_cfg |= irq_cfg;
			if (intel_hsi->suspend_state == DEVICE_READY)
				hsi_enable_interrupt(ctrl, version,
							intel_hsi->irq_cfg);
		}
		if (err_cfg) {
			intel_hsi->err_cfg |= err_cfg;
			if (intel_hsi->suspend_state == DEVICE_READY)
				hsi_enable_error_interrupt(ctrl, version,
							intel_hsi->err_cfg);
		}
		spin_unlock_irqrestore(&intel_hsi->hw_lock, flags);
	}
	intel_hsi->isr_tasklet_running = 0;
}

/**
 * hsi_fwd_tasklet - forwarding tasklet to send HSI messages back to client
 * @hsi: Intel HSI controller reference
 */
static void hsi_fwd_tasklet(unsigned long hsi)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
	__acquires(&intel_hsi->sw_lock) __releases(&intel_hsi->sw_lock)
{
	struct intel_controller *intel_hsi = (struct intel_controller *) hsi;
	struct hsi_msg *msg;
	unsigned int dir;
	unsigned long flags;

	spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	while (!list_empty(&intel_hsi->fwd_queue)) {
		msg = list_first_entry(&intel_hsi->fwd_queue,
				       struct hsi_msg, link);
		list_del(&msg->link);
		spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);

		dir = (msg->ttype == HSI_MSG_READ) ?
		      DMA_FROM_DEVICE : DMA_TO_DEVICE;
		dma_unmap_sg(intel_hsi->pdev, msg->sgt.sgl,
			     msg->sgt.nents, dir);

		if (msg->ttype == HSI_MSG_WRITE)
			(void) deassert_acwake(intel_hsi);

		if (unlikely(msg->break_frame)) {
			msg->break_frame = 0;
			if (msg->destructor)
				msg->destructor(msg);
			else
				hsi_free_msg(msg);
		} else
			hsi_msg_complete(intel_hsi, msg);
		spin_lock_irqsave(&intel_hsi->sw_lock, flags);
	}
	spin_unlock_irqrestore(&intel_hsi->sw_lock, flags);
}

/**
 * hsi_isr - HSI controller interrupt service routine
 * @irq: IRQ number
 * @hsi: Intel HSI controller reference
 *
 * Clears and stores the interrupt sources and schedules a tasklet for handling
 * them efficiently.
 *
 * Returns IRQ_HANDLED as the interrupt sources are handled in all cases.
 */
static irqreturn_t hsi_isr(int irq, void *hsi)
	__acquires(&intel_hsi->hw_lock) __releases(&intel_hsi->hw_lock)
{
	struct intel_controller *intel_hsi = (struct intel_controller *) hsi;
	void __iomem *ctrl = intel_hsi->ctrl_io;
	int version = intel_hsi->version;
	u32 irq_status, err_status, irq_disable, err_disable, dma_status;

	spin_lock(&intel_hsi->hw_lock);
	/* The only interrupt source when suspended is an external wakeup, so
	 * notify it to the interrupt tasklet */
	if (unlikely(intel_hsi->suspend_state != DEVICE_READY)) {
		if (intel_hsi->suspend_state != DEVICE_AND_IRQ_SUSPENDED)
			disable_irq_nosync(intel_hsi->irq);
		intel_hsi->suspend_state = DEVICE_AND_IRQ_SUSPENDED;

		/* Ignore this wakeup signal if not configured */
		if (intel_hsi->prg_cfg & ARASAN_RESET) {
			spin_unlock(&intel_hsi->hw_lock);
			return IRQ_HANDLED;
		}

		intel_hsi->irq_status |= ARASAN_IRQ_RX_WAKE;
		goto exit_irq;
	}

	if (irq == intel_hsi->irq_wake)
		intel_hsi->hsi_wake_raised = 1;

	dma_status = 0;
	err_status = 0;
	irq_status = ioread32(ARASAN_REG(INT_STATUS));

	if (irq_status & ARASAN_IRQ_ERROR)
		err_status = ioread32(ARASAN_REG(ERR_INT_STATUS));
	if (irq_status & ARASAN_IRQ_DMA(version))
		dma_status = ioread32(ARASAN_REG_V2(ctrl, DMA_INT_STATUS));

	if (use_oob_cawake(intel_hsi))
		irq_disable = irq_status &
			(ARASAN_IRQ_RX_SLEEP(version)|
			 ARASAN_IRQ_ANY_RX_THRESHOLD |
			 ARASAN_IRQ_ANY_TX_THRESHOLD);
	else
		irq_disable = irq_status &
			(ARASAN_IRQ_RX_SLEEP(version) | ARASAN_IRQ_RX_WAKE |
			 ARASAN_IRQ_ANY_RX_THRESHOLD |
			 ARASAN_IRQ_ANY_TX_THRESHOLD);

	err_disable = err_status & ARASAN_IRQ_ANY_DATA_TIMEOUT;

	if (irq_disable) {
		intel_hsi->irq_cfg &= ~irq_disable;
		hsi_enable_interrupt(ctrl, version, intel_hsi->irq_cfg);
	}

	if (irq_status) {
		iowrite32(irq_status, ARASAN_REG(INT_STATUS));
		intel_hsi->irq_status |= irq_status;
	}

	if (err_disable) {
		intel_hsi->err_cfg &= ~err_disable;
		hsi_enable_error_interrupt(ctrl, version, intel_hsi->err_cfg);
	}

	if (err_status) {
		iowrite32(err_status, ARASAN_REG(ERR_INT_STATUS));
		intel_hsi->err_status |= err_status;
	}

	if (dma_status) {
		iowrite32(dma_status, ARASAN_REG_V2(ctrl, DMA_INT_STATUS));
		intel_hsi->dma_status |= dma_status;
	}

exit_irq:
	spin_unlock(&intel_hsi->hw_lock);
	tasklet_hi_schedule(&intel_hsi->isr_tasklet);

	return IRQ_HANDLED;
}

/**
 * hsi_ports_init - initialise the HSI port callback functions
 * @hsi: HSI controller reference
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_ports_init(struct hsi_controller *hsi,
			   struct intel_controller *intel_hsi)
{
	struct hsi_port *port;
	unsigned int i;

	for (i = 0; i < hsi->num_ports; i++) {
		port = hsi->port[i];
		port->async = hsi_mid_async;
		port->setup = hsi_mid_setup;
		port->flush = hsi_mid_flush;
		port->start_tx = hsi_mid_start_tx;
		port->stop_tx = hsi_mid_stop_tx;
		port->release = hsi_mid_release;
		hsi_port_set_drvdata(port, intel_hsi);
	}
}

/**
 * hsi_ports_init - exit the HSI port callback functions
 * @hsi: HSI controller reference
 */
static void hsi_ports_exit(struct hsi_controller *hsi)
{
	struct hsi_port *port;
	unsigned int i;

	for (i = 0; i < hsi->num_ports; i++) {
		port = hsi->port[i];
		port->async = NULL;
		port->setup = NULL;
		port->flush = NULL;
		port->start_tx = NULL;
		port->stop_tx = NULL;
		port->release = NULL;
		hsi_port_set_drvdata(port, NULL);
	}
}

/**
 * hsi_unmap_resources - reserve hardware resources for the driver
 * @intel_hsi: Intel HSI controller reference
 * @pdev: PCI device reference
 *
 * Returns success or an error code if any resource cannot be reserved.
 */
static int hsi_map_resources(struct intel_controller *intel_hsi,
			     struct pci_dev *pdev)
{
	int version, err;
	int pci_bar = 0;
	unsigned long paddr;
	unsigned int ext_dma_id;
	void __iomem *ctrl;
	u32 iolen;

	/* get hsi controller io resource and map it */
	intel_hsi->pdev = &pdev->dev;

	paddr = pci_resource_start(pdev, pci_bar);
	iolen = pci_resource_len(pdev, pci_bar);
	err = pci_request_region(pdev, pci_bar, dev_name(&pdev->dev));
	if (err) {
		pr_err(DRVNAME ": pci_request_region failed (%d)\n", err);
		goto no_sys_region;
	}

	ctrl = ioremap_nocache(paddr, iolen);
	if (!ctrl) {
		pr_err(DRVNAME ": ioremap_nocache failed\n");
		err = -EPERM;
		goto no_sys_remap;
	}
	intel_hsi->ctrl_io = ctrl;

	/*
	 * Read the HW plaform & HSI IP versions:
	 * Penwell/Clovertrail = 1.X
	 * Others HW platform  = 2.X
	 */
	version = ((pdev->device == HSI_PNW_PCI_DEVICE_ID) ||
			(pdev->device == HSI_CLV_PCI_DEVICE_ID)) ? 0x10 : 0x20;

	intel_hsi->version = hsi_ioread32(intel_hsi, ARASAN_REG(VERSION))
			& 0xff;

	/* If the 2 versions do not match => We have a BIG problem ! */
	if (is_arasan_v1(version) != is_arasan_v1(intel_hsi->version)) {
		pr_err(DRVNAME ": Wrong HW version (platform: 0x%X, ip: 0x%X)\n",
				version, intel_hsi->version);
		err = -ENODEV;
		goto wrong_ip_version;
	}

	/* Get master DMA info (for IP V1 only) */
	if (!is_arasan_v1(intel_hsi->version)) {
		pr_info(DRVNAME ": IP V2 (0x%X), HW PCI id (0x%X)\n",
				intel_hsi->version, pdev->device);
		return 0;
	}

	if (pdev->device == HSI_PNW_PCI_DEVICE_ID)
		ext_dma_id = HSI_PNW_MASTER_DMA_ID;
	else /* Assuming it's Cloverview */
		ext_dma_id = HSI_CLV_MASTER_DMA_ID;

	intel_hsi->dmac = pci_get_device(PCI_VENDOR_ID_INTEL, ext_dma_id, NULL);
	if (!intel_hsi->dmac) {
		pr_err(DRVNAME ": pci_get_device(dmac) failed\n");
		err = -EPERM;
		goto no_dmac_device;
	}

	paddr = pci_resource_start(intel_hsi->dmac, pci_bar);
	iolen = pci_resource_len(intel_hsi->dmac, pci_bar);
	err = pci_request_region(intel_hsi->dmac, pci_bar,
				 dev_name(&intel_hsi->dmac->dev));
	if (err) {
		pr_err(DRVNAME ": pci_request_region failed (%d)\n", err);
		goto no_dmac_region;
	}

	intel_hsi->dma_io = ioremap_nocache(paddr, iolen);
	if (!intel_hsi->dma_io) {
		pr_err(DRVNAME ": ioremap_nocache failed\n");
		err = -EPERM;
		goto no_dmac_remap;
	}

	pr_info(DRVNAME ": IP V1 (0x%X), HW PCI id (0x%X)\n",
				intel_hsi->version, pdev->device);
	return 0;

no_dmac_remap:
	pci_release_region(intel_hsi->dmac, pci_bar);
no_dmac_region:
	pci_dev_put(intel_hsi->dmac);
no_dmac_device:
wrong_ip_version:
	iounmap(intel_hsi->ctrl_io);
no_sys_remap:
	pci_release_region(pdev, pci_bar);
no_sys_region:

	return err;
}

/**
 * hsi_unmap_resources - free the hardware resources taken by the driver
 * @intel_hsi: Intel HSI controller reference
 * @pdev: PCI device reference
 */
static void hsi_unmap_resources(struct intel_controller *intel_hsi,
				struct pci_dev *pdev)
{
	int pci_bar = 0;

	/* Release the external DMA controller only if it exists */
	if (intel_hsi->dmac) {
		iounmap(intel_hsi->dma_io);
		pci_release_region(intel_hsi->dmac, pci_bar);
		pci_dev_put(intel_hsi->dmac);
	}

	iounmap(intel_hsi->ctrl_io);
	pci_release_region(pdev, pci_bar);
}

/**
 * hsi_controller_init - initialise the controller structure
 * @intel_hsi: Intel HSI controller reference
 *
 * Returns success or an error code if the controller IRQ cannot be requested.
 */
static int hsi_controller_init(struct intel_controller *intel_hsi,
	struct hsi_mid_pci_platform_data *pdata)
{
	unsigned int ch;
	int err = 0;

	for (ch = 0; ch < HSI_MID_MAX_CHANNELS; ch++) {
		INIT_LIST_HEAD(&intel_hsi->tx_queue[ch]);
		INIT_LIST_HEAD(&intel_hsi->rx_queue[ch]);
	}
	INIT_LIST_HEAD(&intel_hsi->brk_queue);
	INIT_LIST_HEAD(&intel_hsi->fwd_queue);

	spin_lock_init(&intel_hsi->sw_lock);
	spin_lock_init(&intel_hsi->hw_lock);

#ifdef CONFIG_HAS_WAKELOCK
	wake_lock_init(&intel_hsi->stay_awake, WAKE_LOCK_SUSPEND,
		       "hsi_wakelock");

	if (!runtime_pm)
		wake_lock(&intel_hsi->stay_awake);
#endif

	tasklet_init(&intel_hsi->isr_tasklet, hsi_isr_tasklet,
		     (unsigned long) intel_hsi);
	intel_hsi->isr_tasklet_running = 0;
	tasklet_init(&intel_hsi->fwd_tasklet, hsi_fwd_tasklet,
		     (unsigned long) intel_hsi);

	if (use_oob_cawake(intel_hsi)) {
		intel_hsi->gpio_wake = pdata->gpio_wake;
		gpio_request_one(intel_hsi->gpio_wake, GPIOF_IN, DRVNAME);
		intel_hsi->irq_wake = gpio_to_irq(intel_hsi->gpio_wake);
	}

	init_timer(&intel_hsi->tx_idle_poll);
	intel_hsi->tx_idle_poll.data = (unsigned long) intel_hsi;
	intel_hsi->tx_idle_poll.function = tx_idle_poll;

	init_timer(&intel_hsi->rx_idle_poll);
	intel_hsi->rx_idle_poll.data = (unsigned long) intel_hsi;
	intel_hsi->rx_idle_poll.function = rx_idle_poll;

	err = request_irq(intel_hsi->irq, hsi_isr,
			  IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
			  HSI_MPU_IRQ_NAME, intel_hsi);

	if (err < 0) {
		dev_err(intel_hsi->dev, "Request IRQ %d failed (%d)\n",
			intel_hsi->irq, err);
		goto out_err;
	}

	if (use_oob_cawake(intel_hsi)) {
		err = request_irq(intel_hsi->irq_wake, hsi_isr,
				IRQF_TRIGGER_RISING | IRQF_NO_SUSPEND,
				DRVNAME, intel_hsi);

		if (err < 0) {
			dev_err(intel_hsi->dev, "Request GPIO IRQ %d failed (%d)\n",
					intel_hsi->irq_wake, err);
			goto out_err;
		}
	}

	return err;

out_err:
#ifdef CONFIG_HAS_WAKELOCK
	if (!runtime_pm)
		wake_unlock(&intel_hsi->stay_awake);

	wake_lock_destroy(&intel_hsi->stay_awake);
#endif
	return err;
}

/**
 * hsi_controller_exit - set the controller driver to a reset state
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_controller_exit(struct intel_controller *intel_hsi)
{
	/* Reset the HSI hardware */
	hsi_ctrl_clean_reset(intel_hsi);

	/* Free the interrupt */
	free_irq(intel_hsi->irq, intel_hsi);

	if (use_oob_cawake(intel_hsi)) {
		free_irq(intel_hsi->irq_wake, intel_hsi);
		gpio_free(intel_hsi->gpio_wake);
	}


	/* Kill the tasklets */
	tasklet_kill(&intel_hsi->isr_tasklet);
	tasklet_kill(&intel_hsi->fwd_tasklet);

#ifdef CONFIG_HAS_WAKELOCK
	if (!runtime_pm)
		wake_unlock(&intel_hsi->stay_awake);

	wake_lock_destroy(&intel_hsi->stay_awake);
#endif
}

/**
 * hsi_rtpm_idle - runtime power management idle callback
 * @dev: PCI device reference
 *
 * Returns -EBUSY and schedule a cancellable delayed suspend if
 * SCHEDULE_LATER_SUSPEND_ON_IDLE is set otherwise returns 0 as the device is
 * actually idle.
 */
static int hsi_rtpm_idle(struct device *dev)
{
#ifdef SCHEDULE_LATER_SUSPEND_ON_IDLE
	pm_schedule_suspend(dev, IDLE_TO_SUSPEND_DELAY);

	/* Set the device as being busy not to immediately go on suspend */
	return -EBUSY;
#else
	/* The device is actually idle */
	return 0;
#endif
}

/**
 * hsi_rtpm_suspend - runtime power management suspend callback
 * @dev: PCI device reference
 *
 * Returns success or an error code if suspend to RAM is failing or impossible.
 */
static int hsi_rtpm_suspend(struct device *dev)
{
	struct hsi_controller *hsi =
		(struct hsi_controller *)dev_get_drvdata(dev);
	struct intel_controller *intel_hsi =
		(struct intel_controller *)hsi_controller_drvdata(hsi);

	dev_dbg(dev, "hsi enter runtime suspend\n");
	return hsi_ctrl_suspend(intel_hsi, 1);
}

/**
 * hsi_rtpm_resume - runtime power management resume callback
 * @dev: PCI device reference
 *
 * Returns success or an error code if resuming from RAM is failing.
 */
static int hsi_rtpm_resume(struct device *dev)
{
	struct hsi_controller *hsi =
		(struct hsi_controller *)dev_get_drvdata(dev);
	struct intel_controller *intel_hsi =
		(struct intel_controller *)hsi_controller_drvdata(hsi);
	int err;

	dev_dbg(dev, "hsi enter runtime resume\n");
	err = hsi_ctrl_resume(intel_hsi, 1, intel_hsi->pm_do_reset);
	intel_hsi->pm_do_reset = 0;
	if (!err)
		hsi_resume_dma_transfers(intel_hsi);

	return err;
}

#ifdef CONFIG_SUSPEND
/**
 * hsi_pm_suspend - called at system suspend request
 * @dev: PCI device reference
 *
 * Returns success or an error code if suspend to RAM is failing or impossible.
 */
static int hsi_pm_suspend(struct device *dev)
{
	struct hsi_controller *hsi =
		(struct hsi_controller *)dev_get_drvdata(dev);
	struct intel_controller *intel_hsi =
		(struct intel_controller *)hsi_controller_drvdata(hsi);
	int err;
	unsigned int i;

	dev_dbg(dev, "hsi enter suspend\n");
	err = hsi_ctrl_suspend(intel_hsi, 0);
	if (!err) {
		for (i = 0; i < hsi->num_ports; i++)
			hsi_event(hsi->port[i], HSI_EVENT_SUSPEND);
	}

	return err;
}

/**
 * hsi_pm_resume - called at system resume request
 * @dev: PCI device reference
 *
 * Returns success or an error code if resuming from RAM is failing.
 */
static int hsi_pm_resume(struct device *dev)
{
	struct hsi_controller *hsi =
		(struct hsi_controller *)dev_get_drvdata(dev);
	struct intel_controller *intel_hsi =
		(struct intel_controller *)hsi_controller_drvdata(hsi);
	int err;
	unsigned int i;

	dev_dbg(dev, "hsi enter resume\n");
	err = hsi_ctrl_resume(intel_hsi, 0, 0);
	if (!err) {
		hsi_resume_dma_transfers(intel_hsi);
		for (i = 0; i < hsi->num_ports; i++)
			hsi_event(hsi->port[i], HSI_EVENT_RESUME);

	}

	return err;
}

#else /* CONFIG_SUSPEND */
#define hsi_pm_suspend NULL
#define hsi_pm_resume  NULL
#endif /* CONFIG_SUSPEND */

/**
 * hsi_rtpm_init - initialising the runtime power management
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_rtpm_init(struct intel_controller *intel_hsi)
{
	struct device *dev = intel_hsi->pdev;

	pm_runtime_allow(dev);
	pm_runtime_put_noidle(dev);
}

/**
 * hsi_rtpm_exit - exiting the runtime power management
 * @intel_hsi: Intel HSI controller reference
 */
static void hsi_rtpm_exit(struct intel_controller *intel_hsi)
{
	struct device *dev = intel_hsi->pdev;

	pm_runtime_forbid(dev);
	pm_runtime_get_noresume(dev);
}

/**
 * hsi_add_controller - make and init intel_hsi controller
 * @hsi: HSI controller reference
 * @pdev: PCI device reference
 *
 * Allocate intel_hsi controller, attach to hsi_controller, activate
 * PCI device and map memory for HSI and master DMA, init ports, and
 * register controller with HSI (perform board info scan there).
 *
 * Returns success or an error code if any initialisation is failing.
 */
static int hsi_add_controller(struct hsi_controller *hsi,
				     struct pci_dev *pdev)
{
	struct intel_controller *intel_hsi;
	struct hsi_mid_pci_platform_data *pdata = pdev->dev.platform_data;
	int err;

	intel_hsi = kzalloc(sizeof(*intel_hsi), GFP_KERNEL);
	if (!intel_hsi) {
		pr_err(DRVNAME ": Out of memory (intel_hsi)\n");
		return -ENOMEM;
	}
	hsi->id = 0;
	hsi->num_ports = 1;
	hsi->device.parent = &pdev->dev;
	hsi->device.dma_mask = pdev->dev.dma_mask;

	dev_set_name(&hsi->device, "hsi%d", hsi->id);
	hsi_controller_set_drvdata(hsi, intel_hsi);
	intel_hsi->dev = &hsi->device;
	intel_hsi->irq = pdev->irq;
	intel_hsi->resumed = 1;
	intel_hsi->use_oob_cawake = pdata->use_oob_cawake;

	/* Initialize hsi_wake disabling status */
	intel_hsi->hsi_wake_disabled = 0;

	err = pci_enable_device(pdev);
	if (err) {
		pr_err(DRVNAME ": pci_enable_device failed (%d)\n", err);
		goto fail_pci_enable_device;
	}

	err = hsi_map_resources(intel_hsi, pdev);
	if (err)
		goto fail_map_resources;

	hsi_ports_init(hsi, intel_hsi);
	err = hsi_controller_init(intel_hsi, pdata);
	if (err < 0)
		goto fail_controller_init;

	err = hsi_ctrl_full_reset(intel_hsi);
	if (err < 0)
		goto fail_controller_reset;

#ifdef CONFIG_DEBUG_FS
	err = hsi_debug_add_ctrl(hsi);
	if (err < 0)
		goto fail_add_debug;
#endif

	err = hsi_register_controller(hsi);
	if (err < 0)
		goto fail_controller_register;

	pci_set_drvdata(pdev, (void *)hsi);

	if (runtime_pm)
		hsi_rtpm_init(intel_hsi);

	return 0;

fail_controller_register:
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(intel_hsi->dir);
fail_add_debug:
#endif
fail_controller_reset:
	hsi_controller_exit(intel_hsi);
fail_controller_init:
	hsi_unmap_resources(intel_hsi, pdev);
fail_map_resources:
	hsi_ports_exit(hsi);
	pci_disable_device(pdev);
fail_pci_enable_device:
	hsi_controller_set_drvdata(hsi, NULL);
	kfree(intel_hsi);
	return err;
}

/**
 * hsi_remove_controller - stop controller and unregister with HSI
 * @hsi: HSI controller reference
 * @pdev: PCI device reference
 *
 * Stop controller and unregister with HSI
 */
static void hsi_remove_controller(struct hsi_controller *hsi,
				  struct pci_dev *pdev)
{
	struct intel_controller *intel_hsi =
		(struct intel_controller *)hsi_controller_drvdata(hsi);

	if (runtime_pm)
		hsi_rtpm_exit(intel_hsi);

	pci_set_drvdata(pdev, NULL);
	hsi_unregister_controller(hsi);
#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(intel_hsi->dir);
#endif
	hsi_controller_exit(intel_hsi);
	hsi_unmap_resources(intel_hsi, pdev);
	hsi_ports_exit(hsi);
	pci_disable_device(pdev);
	hsi_controller_set_drvdata(hsi, NULL);
	kfree(intel_hsi);
}

/**
 * intel_hsi_probe - device PCI probe
 * @pdev: PCI device reference
 * @ent: PCI device id reference
 *
 * Allocate, add controller to the HSI framework and initialise its hardware.
 *
 * Returns success or an error code if any initialisation is failing.
 */
static int intel_hsi_probe(struct pci_dev *pdev,
			   const struct pci_device_id *ent)
{
	struct hsi_controller *hsi;
	int err = 0;

	hsi = hsi_alloc_controller(1, GFP_KERNEL);
	if (!hsi) {
		pr_err(DRVNAME ": Out of memory (hsi controller)\n");
		return -ENOMEM;
	}

	err = hsi_add_controller(hsi, pdev);
	if (err < 0)
		goto fail_add_controller;

	return 0;

fail_add_controller:
	kfree(hsi);
	pr_err(DRVNAME ": Controller probe failed\n");
	return err;
}

/**
 * intel_hsi_remove - called during PCI device exit
 * @pdev: PCI device reference
 *
 * Remove the HSI controller from the HSI framework and free its memory.
 */
static void intel_hsi_remove(struct pci_dev *pdev)
{
	struct hsi_controller *hsi =
		(struct hsi_controller *) pci_get_drvdata(pdev);

	pr_debug(DRVNAME ": Driver remove requested\n");

	if (hsi) {
		hsi_remove_controller(hsi, pdev);
		kfree(hsi);
	}
}

/**
 * intel_hsi_shutdown - called during PCI device shutdown
 * @pdev: PCI device reference
 *
 * Reset the HSI HW and free allocated memory
 */
static void intel_hsi_shutdown(struct pci_dev *pdev)
{
	struct hsi_controller *hsi =
		(struct hsi_controller *) pci_get_drvdata(pdev);
	struct intel_controller *intel_hsi =
		(struct intel_controller *)hsi_controller_drvdata(hsi);

	pr_debug(DRVNAME ": Driver shutdown requested\n");

	if (intel_hsi->suspend_state != DEVICE_READY) {
		pr_info(DRVNAME ":Nothing to do, driver suspended\n");
		return;
	}

	if (runtime_pm)
		hsi_rtpm_exit(intel_hsi);

	/*avoid accessing resource during shutdown*/
	intel_hsi->tx_state = TX_SLEEPING;
	intel_hsi->rx_state = RX_SLEEPING;
	intel_hsi->suspend_state = DEVICE_SUSPENDED;

	pci_set_drvdata(pdev, NULL);

#ifdef CONFIG_DEBUG_FS
	debugfs_remove_recursive(intel_hsi->dir);
#endif
	hsi_controller_exit(intel_hsi);
	hsi_unmap_resources(intel_hsi, pdev);
	pci_disable_device(pdev);
	kfree(intel_hsi);
}

/**
 * struct intel_mid_hsi_rtpm - runtime power management callbacks
 */
static const struct dev_pm_ops intel_mid_hsi_rtpm = {
	.suspend = hsi_pm_suspend,
	.resume = hsi_pm_resume,
	SET_RUNTIME_PM_OPS(
		hsi_rtpm_suspend,
		hsi_rtpm_resume,
		hsi_rtpm_idle
	)
};

/**
 * struct pci_ids - PCI IDs handled by the driver (ID of HSI controller)
 */
static DEFINE_PCI_DEVICE_TABLE(pci_ids) = {
	{ PCI_VDEVICE(INTEL, HSI_PNW_PCI_DEVICE_ID) },	/* HSI - Penwell */
	{ PCI_VDEVICE(INTEL, HSI_CLV_PCI_DEVICE_ID) },	/* HSI - Cloverview */
	{ PCI_VDEVICE(INTEL, HSI_TNG_PCI_DEVICE_ID) },	/* HSI - Tangier */
	{ }
};

/**
 * struct intel_hsi_driver - PCI structure for driver
 */
static struct pci_driver intel_hsi_driver = {
	.driver = {
		.pm = &intel_mid_hsi_rtpm,
	},
	.name =		"intel_hsi",
	.id_table =	pci_ids,
	.probe =	intel_hsi_probe,
	.remove =	intel_hsi_remove,
	.shutdown = intel_hsi_shutdown
};

/**
 * intel_hsi_init - HSI controller driver entry point and initialisation
 *
 * Returns success or an error code if the PCI driver registration is failing.
 */
static int __init intel_hsi_init(void)
{
	/* Disable the rtpm if the hsi_pm is disabled */
	if (!hsi_pm)
		runtime_pm = 0;

	pr_info(DRVNAME ": Controller driver regiseterd\n");
	return pci_register_driver(&intel_hsi_driver);
}
module_init(intel_hsi_init);

/**
 * intel_hsi_exit - frees resources taken by the HSI controller driver
 */
static void __exit intel_hsi_exit(void)
{
	pr_info(DRVNAME ": Controller driver removed\n");
	pci_unregister_driver(&intel_hsi_driver);
}
module_exit(intel_hsi_exit);

MODULE_ALIAS("pci:intel_hsi");
MODULE_AUTHOR("Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>");
MODULE_AUTHOR("Faouaz Tenoutit <faouazx.tenoutit@intel.com>");
MODULE_DESCRIPTION("Intel mid HSI Controller Driver");
MODULE_LICENSE("GPL v2");
