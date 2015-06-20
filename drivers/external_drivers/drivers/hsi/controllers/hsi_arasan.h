/*
 * hsi_arasan.h
 *
 * Implements HSI interface for Arasan controller.
 *
 * Copyright (C) 2012 Intel Corporation. All rights reserved.
 *
 * Contact: Faouaz Tenoutit <faouaz.tenoutit@intel.com>
 *          Olivier Stoltz Douchet <olivierx.stoltz-douchet@intel.com>
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

#ifndef _HSI_ARASAN_H_
#define _HSI_ARASAN_H_

/*
 * HSI IP register offset definitions
 */

/* Offsets for ARASAN v1.4 registers */
#define ARASAN_SDMA_CFG_V1               0x000
#define ARASAN_SDMA_CFG_STP_V1           4
#define ARASAN_TX_FIFO_SIZE_V1           0x040
#define ARASAN_TX_FIFO_SIZE_STP_V1       0
#define ARASAN_TX_FIFO_THRES_V1          0x044
#define ARASAN_TX_FIFO_THRES_STP_V1      0
#define ARASAN_RX_FIFO_SIZE_V1           0x048
#define ARASAN_RX_FIFO_SIZE_STP_V1       0
#define ARASAN_RX_FIFO_THRES_V1          0x04c
#define ARASAN_RX_FIFO_THRES_STP_V1      0
#define ARASAN_CLOCK_CTRL_V1             0x050
#define ARASAN_HSI_STATUS_V1             0x054
#define ARASAN_HSI_STATUS1_V1            0x0c4
#define ARASAN_INT_STATUS_V1             0x058
#define ARASAN_INT_STATUS_ENABLE_V1      0x05c
#define ARASAN_INT_SIGNAL_ENABLE_V1      0x060
#define ARASAN_PROGRAM_V1                0x064
#define ARASAN_PROGRAM1_V1               0x0c8
#define ARASAN_ARBITER_PRIORITY_V1       0x068
#define ARASAN_ARBITER_BANDWIDTH1_V1     0x06c
#define ARASAN_ARBITER_BANDWIDTH2_V1     0x070
#define ARASAN_CAPABILITY_V1             0x074
#define ARASAN_TX_DATA_V1                0x078
#define ARASAN_TX_DATA_STP_V1            4
#define ARASAN_RX_DATA_V1                0x098
#define ARASAN_RX_DATA_STP_V1            4
#define ARASAN_ERR_INT_STATUS_V1         0x0b8
#define ARASAN_ERR_INT_STATUS_ENABLE_V1  0x0bc
#define ARASAN_ERR_INT_SIGNAL_ENABLE_V1  0x0c0
#define ARASAN_VERSION_V1                0x0fc
#define ARASAN_TX_DATA_PORT_V1           UNSUPP
#define ARASAN_TX_DATA_PORT_STP_V1       UNSUPP
#define ARASAN_RX_DATA_PORT_V1           UNSUPP
#define ARASAN_RX_DATA_PORT_STP_V1       UNSUPP
#define ARASAN_TX_CHAN_CURR_PTR_V1       UNSUPP
#define ARASAN_TX_CHAN_CURR_PTR_STP_V1   UNSUPP
#define ARASAN_RX_CHAN_CURR_PTR_V1       UNSUPP
#define ARASAN_RX_CHAN_CURR_PTR_STP_V1   UNSUPP
#define ARASAN_TX_DMA_ARB_V1             UNSUPP
#define ARASAN_RX_DMA_ARB_V1             UNSUPP
#define ARASAN_DMA_INT_STATUS_V1         UNSUPP
#define ARASAN_OCP_CONFIG_V1             UNSUPP
#define ARASAN_TX_DMA_BST_SZ_V1          UNSUPP
#define ARASAN_RX_DMA_BST_SZ_V1          UNSUPP


/* Offsets for ARASAN v1.9+ registers */
#define ARASAN_SDMA_CFG_V2               UNSUPP
#define ARASAN_SDMA_CFG_STP_V2           UNSUPP
#define ARASAN_TX_FIFO_SIZE_V2           0x020
#define ARASAN_TX_FIFO_SIZE_STP_V2       8
#define ARASAN_TX_FIFO_THRES_V2          0x024
#define ARASAN_TX_FIFO_THRES_STP_V2      8
#define ARASAN_RX_FIFO_SIZE_V2           0x060
#define ARASAN_RX_FIFO_SIZE_STP_V2       8
#define ARASAN_RX_FIFO_THRES_V2          0x064
#define ARASAN_RX_FIFO_THRES_STP_V2      8
#define ARASAN_TX_DATA_V2                0x0a0
#define ARASAN_TX_DATA_STP_V2            4
#define ARASAN_RX_DATA_V2                0x0c0
#define ARASAN_RX_DATA_STP_V2            4
#define ARASAN_TX_CHAN_CURR_PTR_V2       0x0e0
#define ARASAN_TX_CHAN_CURR_PTR_STP_V2   4
#define ARASAN_RX_CHAN_CURR_PTR_V2       0x100
#define ARASAN_RX_CHAN_CURR_PTR_STP_V2   4
#define ARASAN_CLOCK_CTRL_V2             0x200
#define ARASAN_HSI_STATUS_V2             0x204
#define ARASAN_PROGRAM_V2                0x208
#define ARASAN_CAPABILITY_V2             0x20c
#define ARASAN_HSI_STATUS1_V2            0x210
#define ARASAN_PROGRAM1_V2               0x214
#define ARASAN_VERSION_V2                0x218
#define ARASAN_INT_STATUS_V2             0x21c
#define ARASAN_INT_STATUS_ENABLE_V2      0x220
#define ARASAN_INT_SIGNAL_ENABLE_V2      0x224
#define ARASAN_ERR_INT_STATUS_V2         0x228
#define ARASAN_ERR_INT_STATUS_ENABLE_V2  0x22c
#define ARASAN_ERR_INT_SIGNAL_ENABLE_V2  0x230
#define ARASAN_TX_DMA_ARB_V2             0x234
#define ARASAN_RX_DMA_ARB_V2             0x238
#define ARASAN_DMA_INT_STATUS_V2         0x23c
#define ARASAN_OCP_CONFIG_V2             0x240
#define ARASAN_TX_DMA_BST_SZ_V2          0x244
#define ARASAN_RX_DMA_BST_SZ_V2          0x248
#define ARASAN_ARBITER_PRIORITY_V2       0x24c
#define ARASAN_ARBITER_BANDWIDTH1_V2     UNSUPP
#define ARASAN_ARBITER_BANDWIDTH2_V2     UNSUPP

/**
 * is_arasan_v1 - test against Arasan V1 IP
 * @version: the version ID of the IP (0x14 for 1.4)
 *
 * Returns 1 if the given version ID is compatible with Arasan V1 IP,
 * 0 otherwise.
 */
#define ARASAN_IP_V1 0x10
#define ARASAN_IP_V2 0x20

static inline int is_arasan_v1(int version)
{
	return version < 0x19;
}

/*
 * Register access macros
 */
#define ARASAN_REG_V1(base, r) (base + ARASAN_ ## r ## _V1)
#define ARASAN_REG_V2(base, r) (base + ARASAN_ ## r ## _V2)
#define ARASAN_REG(r)  ((is_arasan_v1(version) ? \
			ARASAN_REG_V1(ctrl, r) : ARASAN_REG_V2(ctrl, r)))

#define ARASAN_CHN_V1(r, ch) \
	(ctrl+(ARASAN_ ## r ## _V1 + ch * ARASAN_ ## r ## _STP_V1))

#define ARASAN_CHN_V2(r, ch) \
	(ctrl+(ARASAN_ ## r ## _V2 + ch * ARASAN_ ## r ## _STP_V2))

#define ARASAN_CHN_REG(r, ch) ((is_arasan_v1(version) ? \
	ARASAN_CHN_V1(r, ch) : ARASAN_CHN_V2(r, ch)))

/*
 * Key register fields
 */
#define ARASAN_ALL_CHANNELS        ((1<<8)-1)
#define ARASAN_ANY_CHANNEL         ((1<<8)-1)
#define ARASAN_ANY_DMA_CHANNEL     ((1<<8)-1)

#define ARASAN_DMA_ENABLE          (1<<31)
#define ARASAN_DMA_BURST_SIZE(s)   (order_base_2(s/4)<<24)
#define ARASAN_DMA_XFER_FRAMES(s)  ((s)<<4)
#define ARASAN_DMA_CHANNEL(c)      ((c)<<1)
#define ARASAN_DMA_DIR(d)          ((d)<<0)

#define ARASAN_FIFO_MAX_BITS            10
#define ARASAN_FIFO_SIZE(s, c)          ((s)<<((c)*4))
#define ARASAN_FIFO_DEPTH(r, c)         (1<<(((r)>>((c)*4)) & 0xf))

#define ARASAN_RX_TAP_DELAY_NS(c)       (min((c), 7)<<27)
#define ARASAN_RX_TAILING_BIT_COUNT(c)  ((200/max((c), 50))<<24)
#define ARASAN_RX_FRAME_BURST_COUNT(c)  (((c) & 0xff)<<16)
#define ARASAN_TX_BREAK                 (1<<15)
#define ARASAN_DATA_TIMEOUT(t)          ((t)<<11)
#define ARASAN_CLK_DIVISOR(d)           ((d)<<3)
#define ARASAN_CLK_START                (1<<2)
#define ARASAN_CLK_STABLE               (1<<1)
#define ARASAN_CLK_ENABLE               (1<<0)

#define ARASAN_ALL_TX_EMPTY             (ARASAN_ANY_CHANNEL<<24)
#define ARASAN_TX_EMPTY(c)              (1<<((c)+24))
#define ARASAN_ANY_RX_NOT_EMPTY         (ARASAN_ANY_CHANNEL<<8)
#define ARASAN_RX_NOT_EMPTY(c)          (1<<((c)+8))
#define ARASAN_RX_READY                 (1<<7)
#define ARASAN_RX_WAKE                  (1<<4)

#define ARASAN_TX_ENABLE                (1<<31)
#define ARASAN_TX_DISABLE               (0<<31)
#define ARASAN_RX_MODE(m)               (((m) == HSI_MODE_FRAME)<<30)
#define ARASAN_RX_CHANNEL_ENABLE(en, c) ((en)<<(20+(c)))
#define ARASAN_TX_CHANNEL_ENABLE(en, c) ((en)<<(12+(c)))
#define ARASAN_RX_ENABLE                (1<<11)
#define ARASAN_RX_DISABLE               (0<<11)
#define ARASAN_RX_FLOW(f)               (((f) == HSI_FLOW_PIPE)<<9)
#define ARASAN_TX_MODE(m)               (((m) == HSI_MODE_FRAME)<<8)
#define ARASAN_TX_FRAME_MODE            ARASAN_TX_MODE(HSI_MODE_FRAME)
#define ARASAN_RX_TIMEOUT_CNT(cnt)      (((cnt) & 0x7f)<<1)
#define ARASAN_RESET                    (1<<0)

#define ARASAN_IRQ_ERROR                (1<<31)
#define ARASAN_IRQ_DMA(v)               (is_arasan_v1(v) ? 0 : (1<<30))
#define ARASAN_IRQ_ANY_DMA_COMPLETE     (ARASAN_ANY_DMA_CHANNEL<<17)
#define ARASAN_IRQ_DMA_COMPLETE(c)      (1<<((c)+17))
#define ARASAN_IRQ_RX_WAKE              (1<<16)
#define ARASAN_IRQ_RX_SLEEP(v)          (is_arasan_v1(v) ? 0 : (1<<17))
#define ARASAN_IRQ_ANY_RX_THRESHOLD     (ARASAN_ANY_CHANNEL<<8)
#define ARASAN_IRQ_RX_THRESHOLD(c)      (1<<((c)+8))
#define ARASAN_IRQ_ANY_TX_THRESHOLD     (ARASAN_ANY_CHANNEL)
#define ARASAN_IRQ_TX_THRESHOLD(c)      (1<<(c))

#define ARASAN_IRQ_ANY_DATA_TIMEOUT     (ARASAN_ANY_CHANNEL<<2)
#define ARASAN_IRQ_DATA_TIMEOUT(c)      (1<<((c)+2))
#define ARASAN_IRQ_RX_ERROR             (1<<1)
#define ARASAN_IRQ_BREAK                (1<<0)

#define ARASAN_RX_CHANNEL_BITS(b)       (((b) & 0x3)<<2)
#define ARASAN_RX_CHANNEL_SIZE(s)       ARASAN_RX_CHANNEL_BITS(order_base_2(s))
#define ARASAN_TX_CHANNEL_BITS(b)       ((b) & 0x03)
#define ARASAN_TX_CHANNEL_SIZE(s)       ARASAN_TX_CHANNEL_BITS(order_base_2(s))
#define ARASAN_TX_CHANNEL_CNT(r)        (1<<((r) & 0x3))
#define ARASAN_RX_CHANNEL_CNT(r)        (1<<(((r)>>2) & 0x3))

#define ARASAN_TX_BASE_CLK_KHZ(r)       ((((r)>>11) & 0x1ff) * 1000)

#define ARASAN_FIFO_DISABLE             0xffffffff
#define ARASAN_THRES_DISABLE            0xffffffff

#define ARASAN_DMA_IRQ_TX_COMPLETE(c)   (1<<((c)+16))
#define ARASAN_DMA_IRQ_RX_COMPLETE(c)   (1<<((c)+24))

#define ARASAN_LLD_LAST                 (1<<0)
#define ARASAN_LLD_IOC                  (1<<1)
#define ARASAN_LLD_NO_CHAIN             (1<<2)
#define ARASAN_LLD_CHAINED              (0<<2)
#define ARASAN_LLD_HARD                 (1<<3)
#define ARASAN_LLD_SOFT                 (0<<3)

#define ARASAN_LLD_EN                   0x01
#define ARASAN_LLD_DB                   0x02

/* Maximal size of Arasan's FIFO in HSI frames */
#define ARASAN_MAX_TX_FIFO_SZ(v) (is_arasan_v1(v) ? 1024 : 4096)
#define ARASAN_MAX_RX_FIFO_SZ(v) (is_arasan_v1(v) ? 1024 : 4096)

#endif /* _ARASAN_H */
