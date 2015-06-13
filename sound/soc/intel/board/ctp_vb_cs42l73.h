/*
 *  ctp_vb_cs42l73.h - ASoc Machine driver for Intel VictoriaBay MID platform
 *
 *  Copyright (C) 2011-13 Intel Corp
 *  Author: Selma Bensaid <sylvainx.pichon@intel.com>
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


#ifndef _CTP_VB_CS42L73_H_
#define _CTP_VB_CS42L73_H_

/* Data path functionalities */
struct snd_pcm_hardware CTP_VB_COMMS_VOIP_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
				SNDRV_PCM_INFO_DOUBLE |
				SNDRV_PCM_INFO_PAUSE |
				SNDRV_PCM_INFO_RESUME |
				SNDRV_PCM_INFO_MMAP |
				SNDRV_PCM_INFO_MMAP_VALID |
				SNDRV_PCM_INFO_BATCH |
				SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
		.rates = (SNDRV_PCM_RATE_16000),
		.rate_min = 16000,
		.rate_max = 16000,
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
struct snd_pcm_hardware CTP_VB_COMMS_MIXING_hw_param = {
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

/*
 * For VoIP 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define CTP_VB_SSP_VOIP_SLOT_NB_SLOT	1
#define CTP_VB_SSP_VOIP_SLOT_WIDTH	32
#define CTP_VB_SSP_VOIP_SLOT_RX_MASK	0x1
#define CTP_VB_SSP_VOIP_SLOT_TX_MASK	0x1

/*
 * For Mixing 1 slot of 32 bits is used
 * to transfer stereo 16 bits PCM samples
 */
#define CTP_VB_SSP_MIXING_SLOT_NB_SLOT	1
#define CTP_VB_SSP_MIXING_SLOT_WIDTH	32
#define CTP_VB_SSP_MIXING_SLOT_RX_MASK	0x1
#define CTP_VB_SSP_MIXING_SLOT_TX_MASK	0x1

/*
 * MIXER CONTROLS for SSP BT
 */
static const char *const ssp_master_mode_text[] = {"disabled", "enabled"};

static const struct soc_enum ssp_bt_sco_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

static const struct soc_enum ssp_voip_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

static const struct soc_enum ssp_modem_master_mode_enum =
	SOC_ENUM_SINGLE_EXT(2, ssp_master_mode_text);

#endif /* _CTP_VB_CS42L73_H_ */
