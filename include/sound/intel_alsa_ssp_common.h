/*
 *	intel_alsa_ssp_common.h
 *
 *  Copyright (C) 2010 Intel Corp
 *  Authors:	Selma Bensaid <selma.bensaid@intel.com>
 *  ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 *
 *  This program is free software; you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation; version 2 of the License.
 *
 *  This program is distributed in the hope that it will be useful, but
 *  WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 *  General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License along
 *  with this program; if not, write to the Free Software Foundation, Inc.,
 *  59 Temple Place, Suite 330, Boston, MA 02111-1307 USA.
 *
 * ~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~
 */

#ifndef INTEL_ALSA_SSP_COMMON_H_
#define INTEL_ALSA_SSP_COMMON_H_

#define INTEL_ALSA_SSP_SND_CARD_MAX_DEVICES     3
#define INTEL_ALSA_IFX_SND_CARD_MAX_DEVICES     1


#define INTEL_ALSA_BT_DEVICE_ID      0
#define INTEL_ALSA_FM_DEVICE_ID      1
#define INTEL_ALSA_IFX_DEVICE_ID      0

enum intel_alsa_ssp_stream_status {
	INTEL_ALSA_SSP_STREAM_INIT = 0,
	INTEL_ALSA_SSP_STREAM_STARTED,
	INTEL_ALSA_SSP_STREAM_RUNNING,
	INTEL_ALSA_SSP_STREAM_PAUSED,
	INTEL_ALSA_SSP_STREAM_DROPPED,
};

enum intel_alsa_ssp_control_id {
	INTEL_ALSA_SSP_CTRL_SND_OPEN = 0x1000,
	INTEL_ALSA_SSP_CTRL_SND_PAUSE = 0x1001,
	INTEL_ALSA_SSP_CTRL_SND_RESUME = 0x1002,
	INTEL_ALSA_SSP_CTRL_SND_CLOSE = 0x1003,
};

/**
 * @period_req_index: ALSA index ring buffer updated by the DMA transfer
 *		request goes from 0 .. (period_size -1)
 * @period_cb_index: ALSA index ring buffer updated by the DMA transfer
 *		callback goes from 0 .. (period_size -1)
 * @period_index_max : ALSA ring Buffer number of periods
 * @addr: Virtual address of the DMA transfer
 * @length: length in bytes of the DMA transfer
 * @dma_running: Status of DMA transfer
 */
struct intel_alsa_ssp_dma_buf {
	u32 period_req_index;
	s32 period_cb_index;
	u32 period_index_max;
	u8 *addr;
	int length;
};

struct intel_alsa_ssp_stream_info {
	struct intel_alsa_ssp_dma_buf dma_slot;
	struct snd_pcm_substream *substream;
	struct work_struct ssp_ws;
	ssize_t dbg_cum_bytes;
	unsigned long stream_status;
	unsigned int device_id;
	unsigned int stream_index;
};

#endif /* INTEL_ALSA_SSP_COMMON_H_ */
