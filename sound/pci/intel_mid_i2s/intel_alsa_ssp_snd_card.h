/*
 *	intel_alsa_ssp_snd_card.h
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
#ifndef INTEL_ALSA_SSP_SND_CARD_H_
#define INTEL_ALSA_SSP_SND_CARD_H_

#include <sound/initval.h>
#include <linux/pci.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/moduleparam.h>
#include <linux/sched.h>

#include "intel_alsa_ssp_hw_interface.h"
#include <sound/intel_alsa_ssp_common.h>


int snd_i2s_alsa_open(struct snd_pcm_substream *substream);
int snd_i2s_alsa_close(struct snd_pcm_substream *substream);
int snd_i2s_alsa_hw_params(struct snd_pcm_substream *substream,
					struct snd_pcm_hw_params *hw_params);
int snd_i2s_alsa_pcm_prepare(struct snd_pcm_substream *substream);
int snd_i2s_alsa_pcm_trigger(struct snd_pcm_substream *substream, int cmd);
int snd_i2s_alsa_hw_free(struct snd_pcm_substream *substream);
snd_pcm_uframes_t snd_i2s_alsa_pcm_pointer(struct snd_pcm_substream *substream);
int snd_i2s_alsa_pcm_prepare(struct snd_pcm_substream *substream);

/*
 * Defines
 */
#define INTEL_ALSA_SSP_DRIVER_VERSION	"1.0.2"
#define INTEL_ALSA_SSP_CARD_NAME		"IntelALSASSP"


/*
 * Structures Definition
 */
struct intel_alsa_ssp_card_info {
	struct snd_card *card;	/* ptr to the card details */
	struct pci_dev *pci;
	struct workqueue_struct *ssp_wq;
	char *card_id;		/* card id */
	int playback_cnt;
	int capture_cnt;
	int card_index;		/*  card index  */
};

struct intel_alsa_ssp_dev_info {
	char dev_name[32];
	int nb_play_stream;
	int nb_capt_stream;
	struct snd_pcm_hardware *stream_hw_param;
};

/*
 * Global Variables
 */

/* Data path functionalities */
struct snd_pcm_hardware BT_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_DOUBLE |
			  SNDRV_PCM_INFO_PAUSE |
			  SNDRV_PCM_INFO_RESUME |
			  SNDRV_PCM_INFO_MMAP |
			  SNDRV_PCM_INFO_MMAP_VALID |
			  SNDRV_PCM_INFO_BATCH | SNDRV_PCM_INFO_SYNC_START),
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

struct snd_pcm_hardware FM_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_DOUBLE |
			  SNDRV_PCM_INFO_PAUSE |
			  SNDRV_PCM_INFO_RESUME |
			  SNDRV_PCM_INFO_MMAP |
			  SNDRV_PCM_INFO_MMAP_VALID |
			  SNDRV_PCM_INFO_BATCH | SNDRV_PCM_INFO_SYNC_START),
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

struct snd_pcm_hardware BUILTIN_alsa_hw_param = {
		.info = (SNDRV_PCM_INFO_INTERLEAVED |
			  SNDRV_PCM_INFO_DOUBLE |
			  SNDRV_PCM_INFO_PAUSE |
			  SNDRV_PCM_INFO_RESUME |
			  SNDRV_PCM_INFO_MMAP |
			  SNDRV_PCM_INFO_MMAP_VALID |
			  SNDRV_PCM_INFO_BATCH | SNDRV_PCM_INFO_SYNC_START),
		.formats = (SNDRV_PCM_FMTBIT_S16_LE | SNDRV_PCM_FMTBIT_U16_LE),
#ifdef CONFIG_SND_INTEL_ALSA_SSP_BUILTIN_16K
		.rates = (SNDRV_PCM_RATE_16000),
		.rate_min = 16000,
		.rate_max = 16000,
#else
		.rates = (SNDRV_PCM_RATE_48000),
		.rate_min = 48000,
		.rate_max = 48000,
#endif
		.channels_min = 1,
		.channels_max = 2,
		.buffer_bytes_max = (640*1024),
		.period_bytes_min = 64,
		.period_bytes_max = (640*1024),
		.periods_min = 2,
		.periods_max = (1024*2),
		.fifo_size = 0,
};

struct snd_pcm_ops s_alsa_ssp_playback_ops = {
		.open = snd_i2s_alsa_open,
		.close = snd_i2s_alsa_close,
		.ioctl = snd_pcm_lib_ioctl,
		.hw_params = snd_i2s_alsa_hw_params,
		.hw_free = snd_i2s_alsa_hw_free,
		.trigger = snd_i2s_alsa_pcm_trigger,
		.pointer = snd_i2s_alsa_pcm_pointer,
		.prepare = snd_i2s_alsa_pcm_prepare,
};

struct snd_pcm_ops s_alsa_ssp_capture_ops = {
		.open = snd_i2s_alsa_open,
		.close = snd_i2s_alsa_close,
		.ioctl = snd_pcm_lib_ioctl,
		.hw_params = snd_i2s_alsa_hw_params,
		.hw_free = snd_i2s_alsa_hw_free,
		.trigger = snd_i2s_alsa_pcm_trigger,
		.pointer = snd_i2s_alsa_pcm_pointer,
		.prepare = snd_i2s_alsa_pcm_prepare,
};

struct intel_alsa_ssp_dev_info
	s_dev_info[INTEL_ALSA_SSP_SND_CARD_MAX_DEVICES] = {
		{
				.dev_name = "BT_DEVICE",
				.nb_play_stream = 1,
				.nb_capt_stream = 1,
				.stream_hw_param = &BT_alsa_hw_param,

		},
		{
				.dev_name = "FM_DEVICE",
				.nb_play_stream = 0,
				.nb_capt_stream = 1,
				.stream_hw_param = &FM_alsa_hw_param,

		},
		{
				.dev_name = "BUILTIN_DEVICE",
				.nb_play_stream = 1,
				.nb_capt_stream = 1,
				.stream_hw_param = &BUILTIN_alsa_hw_param,

		}
};



#endif /* INTEL_ALSA_SSP_SND_CARD_H_ */
