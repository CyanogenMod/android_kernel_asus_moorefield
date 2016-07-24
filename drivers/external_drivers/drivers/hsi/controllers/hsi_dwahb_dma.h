/*
 * ssi_dwahb_dma.h
 *
 * Implements interface for DW ahb DMA controller.
 *
 * Copyright (C) 2009 Intel Corporation. All rights reserved.
 *
 * Contact: Jim Stanley <jim.stanley@intel.com>
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

#ifndef _DWAHB_DMA_H_
#define _DWAHB_DMA_H_

#define DWAHB_CHAN_CNT					8
#define DWAHB_ALL_CHANNELS				((1<<DWAHB_CHAN_CNT)-1)

#define DWAHB_OFFSET					0x58

/* Channel registers */
#define HSI_DWAHB_SAR(dma_base, channel)		(dma_base +\
						((channel)*DWAHB_OFFSET))
#define HSI_DWAHB_DAR(dma_base, channel)		(dma_base + 0x008 +\
						((channel)*DWAHB_OFFSET))
#define HSI_DWAHB_LLP(dma_base, channel)		(dma_base + 0x010 +\
						((channel)*DWAHB_OFFSET))
#define HSI_DWAHB_CTL_LO(dma_base, channel)		(dma_base + 0x018 +\
						((channel)*DWAHB_OFFSET))
#define HSI_DWAHB_CTL_HI(dma_base, channel)		(dma_base + 0x01C +\
						((channel)*DWAHB_OFFSET))
#define HSI_DWAHB_CFG_LO(dma_base, channel)		(dma_base + 0x040 +\
						((channel)*DWAHB_OFFSET))
#define HSI_DWAHB_CFG_HI(dma_base, channel)		(dma_base + 0x044 +\
						((channel)*DWAHB_OFFSET))

#define HSI_DWAHB_DMACFG(dma_base)			(dma_base+0x398)

#define HSI_DWAHB_CHEN(dma_base)			(dma_base+0x3A0)

/* Interrupt registers */
#define HSI_DWAHB_STATUSINT(dma_base)			(dma_base+0x360)

#define HSI_DWAHB_STATUSTFR(dma_base)			(dma_base+0x2E8)
#define HSI_DWAHB_STATUSBLOCK(dma_base)			(dma_base+0x2F0)
#define HSI_DWAHB_STATUSSRCTRAN(dma_base)		(dma_base+0x2F8)
#define HSI_DWAHB_STATUSDSTTRAN(dma_base)		(dma_base+0x300)
#define HSI_DWAHB_STATUSERR(dma_base)			(dma_base+0x308)

#define HSI_DWAHB_MASKTFR(dma_base)			(dma_base+0x310)
#define HSI_DWAHB_MASKBLOCK(dma_base)			(dma_base+0x318)
#define HSI_DWAHB_MASKSRCTRAN(dma_base)			(dma_base+0x320)
#define HSI_DWAHB_MASKDSTTRAN(dma_base)			(dma_base+0x328)
#define HSI_DWAHB_MASKERR(dma_base)			(dma_base+0x330)

#define HSI_DWAHB_CLEARTFR(dma_base)			(dma_base+0x338)
#define HSI_DWAHB_CLEARBLOCK(dma_base)			(dma_base+0x340)
#define HSI_DWAHB_CLEARSRCTRAN(dma_base)		(dma_base+0x348)
#define HSI_DWAHB_CLEARDSTTRAN(dma_base)		(dma_base+0x350)
#define HSI_DWAHB_CLEARERR(dma_base)			(dma_base+0x358)

/* Map address of HSI controller port rx/tx register */
#define HSI_DWAHB_TX_BASE	0xFF600000
#define HSI_DWAHB_RX_BASE	0xFF400000

#define HSI_DWAHB_TX_ADDRESS(channel) \
				(HSI_DWAHB_TX_BASE + 0x40000*(channel))
#define HSI_DWAHB_RX_ADDRESS(channel) \
				(HSI_DWAHB_RX_BASE + 0x40000*(channel))

/* Key register fields */
#define DWAHB_ENABLE				(1<<0)
#define DWAHB_DISABLE				(0<<0)

#define DWAHB_CHAN_ENABLE(m)			((m) | ((m)<<8))
#define DWAHB_CHAN_DISABLE(m)			((m)<<8)
#define DWAHB_CHAN_START(c)			(0x101<<(c))
#define DWAHB_CHAN_STOP(c)			(0x100<<(c))

#define DWAHB_DST_LINK_LIST(e)			((!!(e))<<28)
#define DWAHB_SRC_LINK_LIST(e)			((!!(e))<<27)
#define DWAHB_IS_NOT_FLOW_CTL(tx_not_rx)	(((!!(tx_not_rx))<<21)|(1<<22))
#define DWAHB_IS_FLOW_CTL(tx_not_rx)		(((!(tx_not_rx))<<21)|\
						 ((!!(tx_not_rx)<<20)))
#define DWAHB_DST_SCATTER(e)			((!!(e))<<18)
#define DWAHB_SRC_GATHER(e)			((!!(e))<<17)
#define DWAHB_SRC_BURST(w)			((order_base_2(w/2))<<14)
#define DWAHB_DST_BURST(w)			((order_base_2(w/2))<<11)
#define DWAHB_SRC_INC				(0<<9)
#define DWAHB_DST_INC				(0<<7)
#define DWAHB_SRC_WIDTH(w)			((order_base_2(w/8))<<4)
#define DWAHB_DST_WIDTH(w)			((order_base_2(w/8))<<1)
#define DWAHB_IRQ_ENABLE(e)			((!!(e))<<0)

#define DWAHB_DST_HW_HANDSHAKE(c)		(((c)&0xF)<<11)
#define DWAHB_SRC_HW_HANDSHAKE(c)		(((c)&0xF)<<7)
#define DWAHB_SRC_STATUS_UPDATE(e)		((!!(e))<<6)
#define DWAHB_DST_STATUS_UPDATE(e)		((!!(e))<<5)
#define DWAHB_DATA_ONLY				(1<<2)
#define DWAHB_USE_FIFO(e)			((!!(e))<<1)
#define DWAHB_PREFETCH(e)			((!(e))<<0)

#define DWAHB_DST_RELOAD(e)			((!!(e))<<31)
#define DWAHB_SRC_RELOAD(e)			((!!(e))<<30)
#define DWAHB_MAX_AMBA_BURST(l)			((l)<<20)
#define DWAHB_SRC_HANDSHAKE_ACTIVE_LOW(e)	((!!(e))<<19)
#define DWAHB_DST_HANDSHAKE_ACTIVE_LOW(e)	((!!(e))<<18)
#define DWAHB_LOCKING(l)			((l)<<12)
#define DWAHB_SRC_SW_HANDSHAKE(e)		((!!(e))<<11)
#define DWAHB_DST_SW_HANDSHAKE(e)		((!!(e))<<10)
#define DWAHB_FIFO_EMPTY			(1<<9)
#define DWAHB_SUSPEND(s)			((!!(s))<<8)
#define DWAHB_PRIORITY(p)			(((p)&0x7)<<5)

#endif /* _DWAHB_DMA_H_ */
