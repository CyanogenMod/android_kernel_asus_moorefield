/*
 *  ctp_rhb_machine.h - ASoc Machine driver for Intel Cloverview MID platform
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: Selma Bensaid <selma.bensaid@intel.com>
 *  Author: Dharageswari.R <dharageswari.r@intel.com>
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

#ifndef MFLD_SSP_WL1273_MACHINE_H_
#define MFLD_SSP_WL1273_MACHINE_H_
#include <linux/init.h>
#include <linux/device.h>
#include <linux/slab.h>
#include <sound/pcm.h>
#include <sound/pcm_params.h>
#include <sound/soc.h>
#include "../ssp/mid_ssp.h"

/* Data path functionalities */
struct snd_pcm_hardware BT_sco_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_8000),
		.rate_min = 8000,
		.rate_max = 8000,
		.channels_min = 1,
		.channels_max = 1,
		.buffer_bytes_max = (320*1024),
		.period_bytes_min = 32,
		.period_bytes_max = (320*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

struct snd_pcm_hardware FM_soc_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/* Data path functionalities */
struct snd_pcm_hardware IFX_modem_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/* Data path functionalities */
struct snd_pcm_hardware VOIP_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

/* Workaround solution */
#if 0
enum {
	CTP_AUD_ASP_DEV = 0,
	CTP_AUD_VSP_DEV,
	CTP_AUD_VIRTUAL_ASP_DEV,
	CTP_COMMS_BT_SCO_DEV,
	CTP_COMMS_MSIC_VOIP_DEV,
	CTP_COMMS_IFX_MODEM_DEV,
	CTP_COMMS_FM_DEV,
	CTP_AUD_COMP_ASP_DEV,
};
#endif

#if 1
enum {
	CTP_AUD_ASP_DEV = 0,
	CTP_AUD_VSP_DEV,
	CTP_AUD_COMP_ASP_DEV,
	CTP_COMMS_BT_SCO_DEV,
	CTP_COMMS_MSIC_VOIP_DEV,
	CTP_COMMS_IFX_MODEM_DEV,
	CTP_AUD_VIRTUAL_ASP_DEV,
	CTP_COMMS_FM_DEV,
};
#endif

/*
 * For FM 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_FM_SLOT_NB_SLOT		1
#define SSP_FM_SLOT_WIDTH		32
#define SSP_FM_SLOT_RX_MASK		0x1
#define SSP_FM_SLOT_TX_MASK		0x0
/*
 * For BT SCO 1 slot of 16 bits is used
 * to transfer mono 16 bits PCM samples
 */
#define SSP_BT_SLOT_NB_SLOT		1
#define SSP_BT_SLOT_WIDTH		16
#define SSP_BT_SLOT_RX_MASK		0x1
#define SSP_BT_SLOT_TX_MASK		0x1

/*
 * For VoIP 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_VOIP_SLOT_NB_SLOT	1
#define SSP_VOIP_SLOT_WIDTH		32
#define SSP_VOIP_SLOT_RX_MASK	0x1
#define SSP_VOIP_SLOT_TX_MASK	0x1

/*
 * For Modem IFX 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define SSP_IFX_SLOT_NB_SLOT	1
#define SSP_IFX_SLOT_WIDTH		32
#define SSP_IFX_SLOT_RX_MASK	0x1
#define SSP_IFX_SLOT_TX_MASK	0x1

/*
 * MIXER CONTROLS for SSP BT
 */
static const char * const ssp_master_mode_text[] = {"disabled", "enabled"};

static const struct soc_enum ssp_bt_sco_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

static const struct soc_enum ssp_voip_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

static const struct soc_enum ssp_modem_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

#endif /* MFLD_SSP_WL1273_MACHINE_H_ */
